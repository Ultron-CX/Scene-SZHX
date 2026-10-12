#!/usr/bin/env python3
"""Audit the deployed X2 CPGWalk 65-value observation contract.

The input is a vendor debug CSV captured from an offline simulation run.  The
tool reconstructs every non-CPG observation term from the recorded physical
state, command and previous policy action.  The four CPG terms are checked as
an explicit pass-through from the vendor oscillator diagnostics; independent
CPG reconstruction is deliberately a separate gate.

This tool is offline and creates no ROS publishers.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import tempfile
import time
from pathlib import Path
from typing import Sequence


SCHEMA = "sceneforge/x2-cpgwalk-observation-audit/v1"


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


def finite_float(row: dict[str, str], name: str, *, frame: int) -> float:
    try:
        value = float(row[name])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"frame {frame} field {name!r} is missing or non-numeric") from exc
    if not math.isfinite(value):
        raise ValueError(f"frame {frame} field {name!r} contains NaN or Inf")
    return value


def build_observation(
    row: dict[str, str],
    *,
    frame: int,
    previous_action: Sequence[float],
    default_dof_position: Sequence[float],
    lin_vel_scale: float,
    ang_vel_scale: float,
    dof_pos_scale: float,
    dof_vel_scale: float,
    command_obs_use_motion_flag: bool,
) -> tuple[float, ...]:
    if len(previous_action) != 17 or len(default_dof_position) != 17:
        raise ValueError("CPGWalk requires 17 previous actions and 17 default positions")
    values = [
        finite_float(row, "base_ang_vel_x", frame=frame) * ang_vel_scale,
        finite_float(row, "base_ang_vel_y", frame=frame) * ang_vel_scale,
        finite_float(row, "base_ang_vel_z", frame=frame) * ang_vel_scale,
        finite_float(row, "obs_gvec_x", frame=frame),
        finite_float(row, "obs_gvec_y", frame=frame),
        finite_float(row, "obs_gvec_z", frame=frame),
        finite_float(row, "cmd_x", frame=frame) * lin_vel_scale,
        finite_float(row, "cmd_y", frame=frame) * lin_vel_scale,
        finite_float(row, "cmd_yaw", frame=frame) * ang_vel_scale,
        (
            0.0 if not command_obs_use_motion_flag
            else float(not bool(int(finite_float(row, "is_standing", frame=frame))))
        ),
    ]
    values.extend(
        (finite_float(row, f"q_{index}", frame=frame) - default_dof_position[index])
        * dof_pos_scale
        for index in range(17)
    )
    values.extend(
        finite_float(row, f"dq_{index}", frame=frame) * dof_vel_scale
        for index in range(17)
    )
    values.extend(float(value) for value in previous_action)
    # This gate checks the observation wiring.  Independent oscillator
    # generation is intentionally not inferred from its own output here.
    values.extend(
        finite_float(row, f"obs_phase_{index}", frame=frame) for index in range(4)
    )
    if len(values) != 65:
        raise AssertionError(f"internal observation width is {len(values)}, expected 65")
    return tuple(values)


class SegmentErrors:
    SEGMENTS = {
        "angular_velocity": range(0, 3),
        "gravity_vector": range(3, 6),
        "velocity_command": range(6, 10),
        "joint_position": range(10, 27),
        "joint_velocity": range(27, 44),
        "previous_action": range(44, 61),
        "cpg_passthrough": range(61, 65),
    }

    def __init__(self) -> None:
        self.values = {name: [] for name in self.SEGMENTS}

    def add(self, actual: Sequence[float], expected: Sequence[float]) -> None:
        if len(actual) != 65 or len(expected) != 65:
            raise ValueError("observation width must be 65")
        for name, indices in self.SEGMENTS.items():
            self.values[name].extend(abs(actual[index] - expected[index]) for index in indices)

    def summary(self) -> dict[str, dict[str, float | int]]:
        return {
            name: {
                "value_count": len(values),
                "maximum_absolute_error": max(values),
                "mean_absolute_error": sum(values) / len(values),
            }
            for name, values in self.values.items()
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug-csv", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--receipt", required=True)
    parser.add_argument("--minimum-frames", type=int, default=1000)
    parser.add_argument("--maximum-absolute-error", type=float, default=1.1e-6)
    parser.add_argument("--recorded-decimal-places", type=int, default=6)
    args = parser.parse_args()

    debug_csv = Path(args.debug_csv).resolve()
    config_path = Path(args.config).resolve()
    receipt = Path(args.receipt).resolve()
    for path in (debug_csv, config_path):
        if not path.is_file():
            raise FileNotFoundError(path)
    if receipt.exists():
        raise FileExistsError(f"refusing to overwrite receipt: {receipt}")

    import yaml

    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    base = config["BaseConfig"]
    actor = config["ActorCriticConfig"]
    walk = config["CPGWalkConfig"]
    if int(base["obs_nums"]) != 65 or int(base["action_nums"]) != 17:
        raise ValueError("unexpected CPGWalk observation/action dimensions")
    default_position = tuple(float(value) for value in actor["robot"]["default_dof_pos"])
    scales = walk["obs_scales"]
    use_motion_flag = bool(walk["cpgwalk"]["command_obs_use_motion_flag"])

    with debug_csv.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
    previous_action = (0.0,) * 17
    errors = SegmentErrors()
    complete_frames = 0
    trailing_partial_rows_skipped = 0
    started = time.monotonic()
    for frame, row in enumerate(rows):
        try:
            actual = tuple(finite_float(row, f"obs_{index}", frame=frame) for index in range(65))
            expected = build_observation(
                row,
                frame=frame,
                previous_action=previous_action,
                default_dof_position=default_position,
                lin_vel_scale=float(scales["lin_vel"]),
                ang_vel_scale=float(scales["ang_vel"]),
                dof_pos_scale=float(scales["dof_pos"]),
                dof_vel_scale=float(scales["dof_vel"]),
                command_obs_use_motion_flag=use_motion_flag,
            )
            action = tuple(finite_float(row, f"action_{index}", frame=frame) for index in range(17))
        except ValueError:
            if frame == len(rows) - 1:
                trailing_partial_rows_skipped += 1
                break
            raise
        errors.add(actual, expected)
        previous_action = action
        complete_frames += 1

    metrics = errors.summary()
    maximum_error = max(
        float(segment["maximum_absolute_error"]) for segment in metrics.values()
    )
    passed = complete_frames >= args.minimum_frames and maximum_error <= args.maximum_absolute_error
    payload = {
        "schema": SCHEMA,
        "status": "PASSED" if passed else "FAILED",
        "decision": (
            "deployed_65_value_observation_wiring_reproduced"
            if passed
            else "observation_contract_not_reproduced"
        ),
        "offline_only": True,
        "read_only_inputs": True,
        "published_topics": [],
        "debug_csv": {
            "path": str(debug_csv),
            "size": debug_csv.stat().st_size,
            "sha256": sha256(debug_csv),
            "source_row_count": len(rows),
            "complete_frame_count": complete_frames,
            "trailing_partial_rows_skipped": trailing_partial_rows_skipped,
            "recorded_decimal_places": args.recorded_decimal_places,
        },
        "config": {
            "path": str(config_path),
            "size": config_path.stat().st_size,
            "sha256": sha256(config_path),
            "observation_count": int(base["obs_nums"]),
            "action_count": int(base["action_nums"]),
            "frame_stack": int(actor["infer"]["frame_stack"]),
            "command_obs_use_motion_flag": use_motion_flag,
            "scales": {
                key: float(scales[key])
                for key in ("lin_vel", "ang_vel", "dof_pos", "dof_vel")
            },
        },
        "layout": {
            "angular_velocity": [0, 3],
            "gravity_vector": [3, 6],
            "velocity_command": [6, 10],
            "joint_position": [10, 27],
            "joint_velocity": [27, 44],
            "previous_action": [44, 61],
            "cpg_terms": [61, 65],
        },
        "metrics": metrics,
        "maximum_absolute_error": maximum_error,
        "thresholds": {
            "minimum_frames": args.minimum_frames,
            "maximum_absolute_error": args.maximum_absolute_error,
        },
        "limitations": [
            "The four CPG terms are checked as vendor-diagnostic pass-through values; "
            "independent simulation-time-driven Hopf reconstruction remains a separate gate.",
            "The vendor debug CSV is decimal-quantized and is not full-precision state evidence.",
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
        if "--receipt" in os.sys.argv:
            index = os.sys.argv.index("--receipt") + 1
            if index < len(os.sys.argv):
                receipt_arg = Path(os.sys.argv[index]).resolve()
        if receipt_arg is not None and not receipt_arg.exists():
            atomic_json(
                receipt_arg,
                {
                    "schema": SCHEMA,
                    "status": "FAILED",
                    "decision": "observation_audit_failed_closed",
                    "error": {"type": type(exc).__name__, "message": str(exc)},
                    "offline_only": True,
                    "published_topics": [],
                    "qualification": False,
                    "real_robot_commands": False,
                    "finished_at_unix": time.time(),
                },
            )
        raise
