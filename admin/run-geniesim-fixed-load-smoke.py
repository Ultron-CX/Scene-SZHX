#!/usr/bin/env python3
"""Run a bounded, command-free Genie Sim PhysX load smoke."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    import yaml

    parser = argparse.ArgumentParser()
    parser.add_argument("--engine", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--scene-yaml", type=Path, required=True)
    parser.add_argument("--physics-params", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--robot-id", default="generic_probe")
    parser.add_argument("--startup-timeout-s", type=float, default=90.0)
    parser.add_argument("--run-duration-s", type=float, default=10.0)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise FileExistsError(f"refusing to overwrite smoke output: {args.output_dir}")
    args.output_dir.mkdir(parents=True)
    log_path = args.output_dir / "engine.log"
    receipt_path = args.output_dir / "result.json"

    config = yaml.safe_load(args.scene_yaml.read_text(encoding="utf-8"))
    source = config["robot"]["robot_source"]
    if source.get("pin_base_to_world") is not True or source.get("fix_base") is not True:
        raise ValueError("load smoke requires fixed-base probe scene")
    if config.get("cameras") or config.get("lidars"):
        raise ValueError("load smoke must not enable sensors")
    init_joint_pos = config["robot"].get("init_joint_pos", {})

    command = [
        os.environ.get("PYTHON", "python3"),
        str(args.engine),
        "--ros-args",
        "-p",
        f"stage_manifest:={args.manifest.resolve()}",
        "-p",
        "physics_engine:=isaac_physx",
        "-p",
        "physics_hz:=100",
        "-p",
        "render_hz:=10",
        "-p",
        "realtime_factor:=1.0",
        "-p",
        "fake_slam:=false",
        "-p",
        f"physics_params_file:={args.physics_params.resolve()}",
        "-p",
        f"init_joint_pos_json:={json.dumps(init_joint_pos, separators=(',', ':'))}",
    ]
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    started_wall = time.monotonic()
    line_queue: queue.Queue[str | None] = queue.Queue()
    lines: list[str] = []
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        env=env,
        start_new_session=True,
    )

    def reader() -> None:
        assert process.stdout is not None
        for line in process.stdout:
            line_queue.put(line)
        line_queue.put(None)

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    ready_at: float | None = None
    hold_completed_at: float | None = None
    forced_kill = False
    sigint_timeout = False
    termination_method = "natural"
    failure: str | None = None
    deadline = started_wall + args.startup_timeout_s
    with log_path.open("w", encoding="utf-8", newline="\n") as log:
        while time.monotonic() < deadline and ready_at is None:
            try:
                line = line_queue.get(timeout=0.25)
            except queue.Empty:
                if process.poll() is not None:
                    failure = f"engine_exited_before_ready:{process.returncode}"
                    break
                continue
            if line is None:
                failure = f"engine_output_closed_before_ready:{process.poll()}"
                break
            log.write(line)
            log.flush()
            lines.append(line)
            if "Isaac Sim physics engine ready" in line:
                ready_at = time.monotonic()
        if ready_at is None and failure is None:
            failure = "startup_timeout"

        if ready_at is not None:
            run_deadline = ready_at + args.run_duration_s
            while time.monotonic() < run_deadline:
                try:
                    line = line_queue.get(timeout=0.25)
                except queue.Empty:
                    if process.poll() is not None:
                        failure = f"engine_exited_during_hold:{process.returncode}"
                        break
                    continue
                if line is None:
                    failure = f"engine_output_closed_during_hold:{process.poll()}"
                    break
                log.write(line)
                log.flush()
                lines.append(line)
            hold_completed_at = time.monotonic()

        if process.poll() is None:
            termination_method = "SIGINT"
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                sigint_timeout = True
                termination_method = "SIGTERM"
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    termination_method = "SIGKILL"
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=10)
                    forced_kill = True
        thread.join(timeout=5)
        while True:
            try:
                line = line_queue.get_nowait()
            except queue.Empty:
                break
            if line is not None:
                log.write(line)
                lines.append(line)

    combined = "".join(lines).lower()
    fatal_markers = [marker for marker in ("traceback", "[genie_sim_engine] fatal", "nan detected") if marker in combined]
    if fatal_markers:
        fatal_failure = f"fatal_log_markers:{','.join(fatal_markers)}"
        failure = fatal_failure if failure is None else f"{failure};{fatal_failure}"
    if forced_kill and failure is None:
        failure = "forced_kill_required"
    accepted_returncodes = {
        "natural": {0},
        "SIGINT": {0, -signal.SIGINT, 128 + signal.SIGINT},
        "SIGTERM": {-signal.SIGTERM, 128 + signal.SIGTERM},
        "SIGKILL": set(),
    }
    if process.returncode not in accepted_returncodes[termination_method] and failure is None:
        failure = f"unexpected_exit:{termination_method}:{process.returncode}"
    held_s = max(
        0.0,
        (hold_completed_at - ready_at)
        if ready_at is not None and hold_completed_at is not None
        else 0.0,
    )
    result = {
        "schema_version": "sceneforge/geniesim-fixed-load-smoke/v1",
        "status": "PASSED" if failure is None else "FAILED",
        "purpose": f"{args.robot_id}_fixed_base_physx_load_smoke",
        "failure": failure,
        "engine_ready": ready_at is not None,
        "requested_hold_s": args.run_duration_s,
        "observed_hold_s": held_s,
        "process_returncode": process.returncode,
        "termination_method": termination_method,
        "sigint_timeout": sigint_timeout,
        "forced_kill": forced_kill,
        "fatal_log_markers": fatal_markers,
        "manifest_sha256": sha256(args.manifest),
        "scene_yaml_sha256": sha256(args.scene_yaml),
        "physics_params_sha256": sha256(args.physics_params),
        "engine_log_sha256": sha256(log_path),
        "published_control_topics": [],
        "qualification": False,
        "limitations": [
            "fixed_base_command_free_load_only",
            "no_controller_motion_navigation_or_contact_qualification",
        ],
        "real_robot_commands": False,
    }
    receipt_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    print(receipt_path)
    if failure is not None:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
