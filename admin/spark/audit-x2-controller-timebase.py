#!/usr/bin/env python3
"""Produce a machine-readable audit of the vendor X2 controller time base."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import tempfile
import time
from pathlib import Path


REQUIRED_FILES = {
    "managed_periodic_worker": "install/assistant/include/assistant/managed_periodic_worker.h",
    "time_wheel_executor": "install/assistant/include/assistant/time_wheel_executor.h",
    "executor_iface": "install/assistant/include/assistant/executor_iface.h",
    "control_module": "install/x1_rl_control/include/control_module/control_module.h",
    "controller_base": "install/x1_rl_control/include/control_module/controller_base.h",
    "controller_binary": "install/x1_rl_control/lib/x1_rl_control/x1_rl_control_node",
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_checks(texts: dict[str, str]) -> dict[str, bool]:
    return {
        "periodic_worker_delegates_wall_period": (
            "ExecutePeriodically(period" in texts["managed_periodic_worker"]
        ),
        "time_wheel_uses_steady_clock": (
            "std::chrono::steady_clock::now()" in texts["time_wheel_executor"]
        ),
        "time_wheel_sleeps_until_wall_deadline": (
            "std::this_thread::sleep_until" in texts["time_wheel_executor"]
            and "last_loop_std_tp" in texts["time_wheel_executor"]
        ),
        "executor_interface_has_no_external_tick": (
            "ExecuteAt(TimePoint" in texts["executor_iface"]
            and not re.search(r"ExternalTick|SimTime|AdvanceTo", texts["executor_iface"])
        ),
        "control_module_uses_steady_clock_for_controller_update": (
            "UpdateActiveControllersInline" in texts["control_module"]
            and "time_point<steady_clock>" in texts["control_module"]
        ),
        "controller_sensor_freshness_uses_steady_clock": (
            "joint_state_receive_time_" in texts["controller_base"]
            and "std::chrono::steady_clock::time_point" in texts["controller_base"]
        ),
        "controller_has_explicit_control_dt_hook": (
            "virtual void SetControlDt(double)" in texts["controller_base"]
        ),
    }


def atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
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
    parser.add_argument("--release-root", required=True)
    parser.add_argument("--freeze-evidence", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    root = Path(args.release_root).resolve()
    output = Path(args.output).resolve()
    if output.exists():
        raise FileExistsError(f"refusing to overwrite output: {output}")
    paths = {name: root / relative for name, relative in REQUIRED_FILES.items()}
    missing = [str(path) for path in paths.values() if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"missing vendor artifacts: {missing}")
    texts = {
        name: path.read_text(encoding="utf-8", errors="replace")
        for name, path in paths.items() if name != "controller_binary"
    }
    checks = source_checks(texts)
    strings = subprocess.run(
        ["strings", str(paths["controller_binary"])],
        check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    ).stdout
    binary_checks = {
        "steady_clock_symbol_present": "steady_clock" in strings,
        "controller_worker_loop_present": "ControllerWorkerLoop" in strings,
        "controller_frequency_parameter_present": "control_frequecy" in strings,
        "controller_workers_parameter_present": "enable_controller_worker_executors" in strings,
        "external_tick_interface_present": bool(
            re.search(r"ExternalTick|SimTimeAdvance|state_driven_control", strings, re.IGNORECASE)
        ),
    }
    freeze_result_path = Path(args.freeze_evidence).resolve()
    freeze_result = json.loads(freeze_result_path.read_text(encoding="utf-8"))
    if freeze_result.get("real_robot_commands") is not False:
        raise ValueError("freeze evidence must contain real_robot_commands=false")
    empirical_wall_bound = freeze_result.get("classification") == "CONTROLLER_WALL_CLOCK_BOUND"
    implementation_wall_bound = all(checks.values()) and all(
        value for name, value in binary_checks.items() if name != "external_tick_interface_present"
    ) and not binary_checks["external_tick_interface_present"]
    classification = (
        "CONTROLLER_WALL_CLOCK_BOUND_BY_IMPLEMENTATION"
        if empirical_wall_bound and implementation_wall_bound
        else "TIMEBASE_AUDIT_INCONCLUSIVE"
    )
    payload = {
        "schema": "sceneforge/x2-controller-timebase-source-audit/v1",
        "status": "BLOCKED" if classification.startswith("CONTROLLER_WALL_CLOCK_BOUND") else "FAILED",
        "classification": classification,
        "source_checks": checks,
        "binary_checks": binary_checks,
        "artifacts": {
            name: {"path": str(path), "size": path.stat().st_size, "sha256": sha256(path)}
            for name, path in paths.items()
        },
        "freeze_evidence": {
            "path": str(freeze_result_path),
            "sha256": sha256(freeze_result_path),
            "classification": freeze_result.get("classification"),
        },
        "required_vendor_interface": {
            "one_of": [
                "external_tick_or_step_api_that_invokes_one_control_update_per_physx_state",
                "executor_clock_provider_driven_by_ros_clock_sim_time",
                "state_callback_driven_controller_update_api",
            ],
            "must_expose": [
                "controller_state_sequence",
                "source_sim_timestamp",
                "cpg_phase_and_period",
                "single_update_without_internal_wall_sleep",
                "reset_of_policy_history_and_cpg_state",
            ],
            "existing_promising_hook": "ControllerBase::SetControlDt(double) does not by itself schedule Update()",
        },
        "decision": "do_not_tune_physx_dynamics_or_claim_mujoco_equivalence",
        "qualification": False,
        "finished_at_unix": time.time(),
        "real_robot_commands": False,
    }
    atomic_json(output, payload)
    print(output)
    return 3 if payload["status"] == "BLOCKED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
