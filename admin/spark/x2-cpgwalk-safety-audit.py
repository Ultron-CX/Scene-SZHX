#!/usr/bin/env python3
"""Audit the X2 CPGWalk command-freeze and stand hand-off safety contract."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import math
import os
import re
import sys
import tempfile
import time
from dataclasses import asdict
from pathlib import Path


CORE_PATH = Path(__file__).resolve().parents[2] / "src" / "sceneforge" / "x2_control.py"
CORE_SPEC = importlib.util.spec_from_file_location("sceneforge_x2_control_standalone", CORE_PATH)
if CORE_SPEC is None or CORE_SPEC.loader is None:
    raise RuntimeError(f"cannot load X2 control core: {CORE_PATH}")
CORE = importlib.util.module_from_spec(CORE_SPEC)
sys.modules[CORE_SPEC.name] = CORE
CORE_SPEC.loader.exec_module(CORE)
CpgWalkSafetyGate = CORE.CpgWalkSafetyGate
CpgWalkSafetyMode = CORE.CpgWalkSafetyMode
CpgWalkSafetyParams = CORE.CpgWalkSafetyParams
CpgWalkGaitSelector = CORE.CpgWalkGaitSelector
CpgWalkGaitSwitchParams = CORE.CpgWalkGaitSwitchParams


SCHEMA = "sceneforge/x2-cpgwalk-safety-audit/v1"
AUTO_STAND_PATTERN = re.compile(
    r"\[(?P<stamp>\d+\.\d+)\].*auto request trigger=/stand_default_mode "
    r"current=LOCOMOTION_DEFAULT source=rl_cpgwalk"
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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


def complete(row: dict[str, str]) -> bool:
    required = (
        "t", "policy_dt", "cmd_x", "cmd_y", "cmd_yaw", "current_gait",
        "is_standing", "freeze", "freeze_att", "freeze_cmd_candidate",
        "low_motion_ready", "base_roll", "base_pitch", "mean_leg_vel",
        "base_ang_vel_norm",
    )
    try:
        values = [float(row[name]) for name in required]
    except (KeyError, TypeError, ValueError):
        return False
    return all(math.isfinite(value) for value in values)


def finite_float(row: dict[str, str], name: str) -> float:
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}")
    return value


def params_from_config(config: dict) -> CpgWalkSafetyParams:
    walk = config["CPGWalkConfig"]["cpgwalk"]
    return CpgWalkSafetyParams(
        command_threshold=float(walk["cmd_threshold"]),
        tilt_freeze_enabled=bool(walk["tilt_freeze_enable"]),
        roll_pitch_threshold=float(walk["roll_pitch_threshold"]),
        require_low_motion_for_freeze=bool(walk["freeze_enter_requires_low_motion"]),
        require_gait1_for_freeze=bool(walk["freeze_enter_requires_gait1"]),
        leg_velocity_threshold=float(walk["freeze_enter_leg_vel_threshold"]),
        base_angular_velocity_threshold=float(
            walk["freeze_enter_base_ang_vel_threshold"]
        ),
        auto_stand_hold_seconds=float(walk["auto_stand_hold_sec"]),
    )


def gait_params_from_config(config: dict) -> CpgWalkGaitSwitchParams:
    walk = config["CPGWalkConfig"]["cpgwalk"]
    return CpgWalkGaitSwitchParams(
        gait1_to_gait2_speed_threshold=float(walk["gait1_to_gait2_speed_threshold"]),
        gait2_to_gait1_speed_threshold=float(walk["gait2_to_gait1_speed_threshold"]),
        gait2_to_gait1_yaw_block_threshold=float(
            walk["gait2_to_gait1_yaw_block_threshold"]
        ),
        initial_gait=int(walk["gait1"]),
    )


def auto_stand_timestamps(controller_log: str) -> list[float]:
    return [float(match.group("stamp")) for match in AUTO_STAND_PATTERN.finditer(controller_log)]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug-csv", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--controller-log", required=True)
    parser.add_argument("--receipt", required=True)
    parser.add_argument("--maximum-handoff-timing-error-s", type=float, default=0.03)
    args = parser.parse_args()

    debug_csv = Path(args.debug_csv).resolve()
    config_path = Path(args.config).resolve()
    controller_log = Path(args.controller_log).resolve()
    receipt = Path(args.receipt).resolve()
    for path in (debug_csv, config_path, controller_log):
        if not path.is_file():
            raise FileNotFoundError(path)
    if receipt.exists():
        raise FileExistsError(f"refusing to overwrite receipt: {receipt}")

    import yaml

    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    params = params_from_config(config)
    params.validate()
    gait_params = gait_params_from_config(config)
    gait_params.validate()
    with debug_csv.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    trailing_partial_rows_skipped = 0
    if rows and not complete(rows[-1]):
        rows.pop()
        trailing_partial_rows_skipped = 1
    if any(not complete(row) for row in rows):
        raise ValueError("vendor trace contains a malformed non-terminal row")
    log_text = controller_log.read_text(encoding="utf-8", errors="replace")
    handoff_stamps = auto_stand_timestamps(log_text)
    if len(handoff_stamps) != 1:
        raise ValueError(f"expected exactly one CPGWalk auto-stand request, got {handoff_stamps}")

    gate = CpgWalkSafetyGate(params)
    gait_selector = CpgWalkGaitSelector(gait_params)
    mismatch = {
        "freeze_command_candidate": 0,
        "freeze_by_attitude": 0,
        "low_motion_ready": 0,
        "freeze": 0,
        "is_standing": 0,
        "current_gait": 0,
    }
    active_seen = False
    final_stand_entry_stamp = None
    stand_to_active_count = 0
    active_to_stand_count = 0
    previous_mode = gate.mode
    internal_request_count = 0
    predicted_gait_switch_count = 0
    previous_predicted_gait = gait_selector.gait
    started = time.monotonic()
    for row in rows:
        command_x = finite_float(row, "cmd_x")
        command_yaw = finite_float(row, "cmd_yaw")
        predicted_gait = gait_selector.update(command_x, command_yaw)
        predicted_gait_switch_count += int(predicted_gait != previous_predicted_gait)
        previous_predicted_gait = predicted_gait
        decision = gate.update(
            dt=finite_float(row, "policy_dt"),
            command_xyz=(
                command_x,
                finite_float(row, "cmd_y"),
                command_yaw,
            ),
            roll=finite_float(row, "base_roll"),
            pitch=finite_float(row, "base_pitch"),
            mean_leg_velocity=finite_float(row, "mean_leg_vel"),
            base_angular_velocity_norm=finite_float(row, "base_ang_vel_norm"),
            gait=predicted_gait,
        )
        expected = {
            "freeze_command_candidate": row["freeze_cmd_candidate"] == "1",
            "freeze_by_attitude": row["freeze_att"] == "1",
            "low_motion_ready": row["low_motion_ready"] == "1",
            "freeze": row["freeze"] == "1",
            "is_standing": row["is_standing"] == "1",
            "current_gait": int(finite_float(row, "current_gait")),
        }
        actual = {
            "freeze_command_candidate": decision.freeze_command_candidate,
            "freeze_by_attitude": decision.freeze_by_attitude,
            "low_motion_ready": decision.low_motion_ready,
            "freeze": decision.freeze,
            "is_standing": decision.mode == CpgWalkSafetyMode.STAND,
            "current_gait": predicted_gait,
        }
        for name in mismatch:
            mismatch[name] += int(actual[name] != expected[name])
        if previous_mode == CpgWalkSafetyMode.STAND and decision.mode == CpgWalkSafetyMode.ACTIVE:
            stand_to_active_count += 1
            active_seen = True
        elif previous_mode == CpgWalkSafetyMode.ACTIVE and decision.mode == CpgWalkSafetyMode.STAND:
            active_to_stand_count += 1
            if active_seen:
                final_stand_entry_stamp = finite_float(row, "t")
        previous_mode = decision.mode
        internal_request_count += int(decision.request_stand_transition)

    if final_stand_entry_stamp is None:
        raise ValueError("final active-to-stand transition was not found")
    handoff_delay = handoff_stamps[0] - final_stand_entry_stamp
    handoff_timing_error = abs(handoff_delay - params.auto_stand_hold_seconds)
    metrics = {
        "row_count": len(rows),
        "mismatch_count": mismatch,
        "stand_to_active_count": stand_to_active_count,
        "active_to_stand_count": active_to_stand_count,
        "internal_request_count_within_recorded_rows": internal_request_count,
        "predicted_gait_switch_count": predicted_gait_switch_count,
        "final_stand_entry_stamp": final_stand_entry_stamp,
        "controller_log_auto_stand_stamp": handoff_stamps[0],
        "observed_auto_stand_delay_seconds": handoff_delay,
        "auto_stand_delay_error_seconds": handoff_timing_error,
    }
    passed = (
        not any(mismatch.values())
        and stand_to_active_count == 1
        and active_to_stand_count == 1
        and internal_request_count == 0
        and handoff_timing_error <= args.maximum_handoff_timing_error_s
    )
    payload = {
        "schema": SCHEMA,
        "status": "PASSED" if passed else "FAILED",
        "decision": (
            "sim_time_command_freeze_and_auto_stand_handoff_reproduced"
            if passed
            else "command_freeze_or_auto_stand_handoff_not_reproduced"
        ),
        "offline_only": True,
        "read_only_inputs": True,
        "published_topics": [],
        "debug_csv": {
            "path": str(debug_csv), "size": debug_csv.stat().st_size,
            "sha256": sha256(debug_csv), "complete_row_count": len(rows),
            "trailing_partial_rows_skipped": trailing_partial_rows_skipped,
            "recorded_decimal_places": 6,
        },
        "config": {
            "path": str(config_path), "size": config_path.stat().st_size,
            "sha256": sha256(config_path),
        },
        "controller_log": {
            "path": str(controller_log), "size": controller_log.stat().st_size,
            "sha256": sha256(controller_log),
        },
        "implementation": {
            "path": str(CORE_PATH), "size": CORE_PATH.stat().st_size,
            "sha256": sha256(CORE_PATH),
        },
        "parameters": {
            "safety": asdict(params),
            "gait_switch": asdict(gait_params),
        },
        "metrics": metrics,
        "thresholds": {
            "maximum_auto_stand_timing_error_seconds": args.maximum_handoff_timing_error_s,
        },
        "limitations": [
            "The run has no nonzero lateral command, so Euclidean command norm and a "
            "component-wise maximum cannot be distinguished from this trace alone.",
            "Tilt freeze is disabled in the accepted run and is covered only by unit tests.",
            "The controller-log timestamp is wall-clock formatted, while the gate itself is "
            "implemented using explicit simulation-time dt.",
            "This gate validates the request boundary; STAND_DEFAULT controller output parity "
            "is a separate qualification item.",
        ],
        "runtime": {"elapsed_wall_seconds": time.monotonic() - started},
        "qualification": False,
        "real_robot_commands": False,
        "finished_at_unix": time.time(),
    }
    atomic_json(receipt, payload)
    print(receipt)
    return 0 if passed else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except SystemExit:
        raise
    except Exception as exc:
        receipt_arg = None
        if "--receipt" in sys.argv:
            index = sys.argv.index("--receipt") + 1
            if index < len(sys.argv):
                receipt_arg = Path(sys.argv[index]).resolve()
        if receipt_arg is not None and not receipt_arg.exists():
            atomic_json(
                receipt_arg,
                {
                    "schema": SCHEMA,
                    "status": "FAILED",
                    "decision": "safety_audit_failed_closed",
                    "error": {"type": type(exc).__name__, "message": str(exc)},
                    "offline_only": True,
                    "published_topics": [],
                    "qualification": False,
                    "real_robot_commands": False,
                    "finished_at_unix": time.time(),
                },
            )
        raise
