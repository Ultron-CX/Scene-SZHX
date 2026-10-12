#!/usr/bin/env python3
"""Seal one completed X2 clock-freeze diagnostic as immutable evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import tempfile
import time
from pathlib import Path


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def atomic_json(path: Path, payload: dict) -> None:
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        os.chmod(path, 0o444)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", required=True)
    parser.add_argument("--evidence", required=True)
    args = parser.parse_args()
    run = Path(args.run).resolve()
    evidence = Path(args.evidence).resolve()
    if evidence.exists():
        raise FileExistsError(f"refusing to overwrite evidence: {evidence}")
    probe_path = run / "clock-freeze-probe-receipt.json"
    status_path = run / "clock-freeze-status.json"
    if not probe_path.is_file() or not status_path.is_file():
        raise FileNotFoundError("completed probe receipt and freeze status are required")
    probe = json.loads(probe_path.read_text(encoding="utf-8"))
    freeze = json.loads(status_path.read_text(encoding="utf-8"))
    if probe.get("real_robot_commands") is not False or freeze.get("real_robot_commands") is not False:
        raise ValueError("diagnostic receipts must contain real_robot_commands=false")
    if freeze.get("status") != "COMPLETED" or not (probe.get("metrics") or {}).get("engine_frozen"):
        raise ValueError("authoritative PhysX clock freeze was not proven")

    evidence.mkdir(parents=True, exist_ok=False)
    include = (
        "clock-freeze-probe-receipt.json",
        "clock-freeze-probe.log",
        "clock-freeze-request.json",
        "clock-freeze-status.json",
        "adapter-receipt.json",
        "adapter.log",
        "controller.log",
        "engine.log",
        "engine-stop-proof.json",
        "elastic-band-release-proof.json",
        "elastic-band-status.json",
        "controller-policy-manifest.json",
    )
    for name in include:
        source = run / name
        if source.is_file():
            shutil.copy2(source, evidence / name)
    artifacts = {
        path.name: {"size": path.stat().st_size, "sha256": digest(path)}
        for path in sorted(evidence.iterdir()) if path.is_file()
    }
    classification = probe.get("classification")
    payload = {
        "schema": "sceneforge/x2-controller-clock-freeze-gate-result/v1",
        "status": "PASSED" if classification == "SIM_TIME_BOUND" else "BLOCKED",
        "qualification": False,
        "classification": classification,
        "decision": (
            "continue_state_command_pairing_gate"
            if classification == "SIM_TIME_BOUND"
            else "do_not_tune_dynamics_or_claim_mujoco_equivalence"
        ),
        "probe": probe,
        "freeze": freeze,
        "artifacts": artifacts,
        "source_run": run.name,
        "finished_at_unix": time.time(),
        "physics_authority": "genie_sim_isaac_physx",
        "mujoco_state_source_used": False,
        "real_robot_commands": False,
    }
    atomic_json(evidence / "result.json", payload)
    for path in evidence.rglob("*"):
        if path.is_file():
            path.chmod(0o444)
        elif path.is_dir():
            path.chmod(0o555)
    evidence.chmod(0o555)
    print(evidence / "result.json")
    return 0 if classification == "SIM_TIME_BOUND" else 3


if __name__ == "__main__":
    raise SystemExit(main())
