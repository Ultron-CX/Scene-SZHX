#!/usr/bin/env python3
"""Audit the engine-neutral STAND_DEFAULT session against vendor CSV evidence."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import tempfile
from pathlib import Path

from sceneforge.x2_stand import (
    StateDrivenX2StandSession,
    X2StandPhysicsState,
    X2_STAND_ACTION_JOINT_NAMES,
)


def vector(value: str, width: int, label: str) -> tuple[float, ...]:
    result = tuple(float(item) for item in value.split())
    if len(result) != width or not all(math.isfinite(item) for item in result):
        raise ValueError(f"invalid {label}")
    return result


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def atomic_json(path: Path, payload: dict) -> None:
    if path.exists():
        raise FileExistsError(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush(); os.fsync(stream.fileno())
        os.replace(temporary, path)
        os.chmod(path, 0o444)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", required=True)
    parser.add_argument("--outputs", required=True)
    parser.add_argument("--debug", required=True)
    parser.add_argument("--receipt", required=True)
    parser.add_argument("--minimum-frames", type=int, default=800)
    args = parser.parse_args()
    frames, outputs, debug, receipt = map(
        lambda value: Path(value).resolve(),
        (args.frames, args.outputs, args.debug, args.receipt),
    )
    for path in (frames, outputs, debug):
        if not path.is_file():
            raise FileNotFoundError(path)

    with frames.open(newline="", encoding="utf-8") as stream:
        frame_rows = list(csv.DictReader(stream))
    with outputs.open(newline="", encoding="utf-8") as stream:
        output_rows = list(csv.DictReader(stream))
    with debug.open(newline="", encoding="utf-8") as stream:
        debug_rows = list(csv.DictReader(stream))
    if not len(frame_rows) == len(output_rows) == len(debug_rows):
        raise ValueError("oracle row counts differ")

    expected_stacked: tuple[float, ...] = ()
    expected_action: tuple[float, ...] = ()
    maximum_stacked_error = 0.0
    maximum_action_error = 0.0
    maximum_target_error = 0.0
    policy_calls = 0

    def policy(values: tuple[float, ...]) -> tuple[float, ...]:
        nonlocal maximum_stacked_error, policy_calls
        policy_calls += 1
        maximum_stacked_error = max(
            maximum_stacked_error,
            max((abs(got - want) for got, want in zip(values, expected_stacked)), default=0.0),
        )
        return expected_action

    session = StateDrivenX2StandSession(policy)
    for index, (frame_row, output_row, debug_row) in enumerate(
        zip(frame_rows, output_rows, debug_rows)
    ):
        numbers = {int(frame_row["frame"]), int(output_row["frame"]), int(debug_row["frame"])}
        if numbers != {index}:
            raise ValueError(f"frame alignment failed at {index}: {numbers}")
        expected_stacked = vector(frame_row["frame_values"], 850, f"frame {index}")
        expected_action = vector(output_row["values"], 14, f"output {index}")
        latest = expected_stacked[-85:]
        state = X2StandPhysicsState(
            source_sequence=index,
            sim_time=index * 0.01,
            base_angular_velocity=latest[0:3],
            observation_gravity=latest[3:6],
            velocity_command=latest[6:9],
            pose_command=latest[9:13],
            joint_position=latest[13:36],
            joint_velocity=latest[36:59],
            arm_target_position=latest[73:81],
            phase_q=latest[81:85],
            waist_action_observation=latest[71:73],
        )
        result = session.update(state)
        maximum_action_error = max(
            maximum_action_error,
            max(
                abs(got - float(debug_row[f"{name}.raw_action"]))
                for name, got in zip(X2_STAND_ACTION_JOINT_NAMES, result.command.raw_action)
            ),
        )
        target_lookup = dict(zip(result.command.joint_names, result.command.position_target))
        maximum_target_error = max(
            maximum_target_error,
            max(
                abs(target_lookup[name] - float(debug_row[f"{name}.target"]))
                for name in X2_STAND_ACTION_JOINT_NAMES
            ),
        )

    checks = {
        "minimum_frames": len(frame_rows) >= args.minimum_frames,
        "one_policy_call_per_frame": policy_calls == len(frame_rows),
        "stacked_history_exact": maximum_stacked_error <= 1e-12,
        "raw_action_matches_six_decimal_debug": maximum_action_error <= 5.1e-7,
        "joint_target_matches_six_decimal_debug": maximum_target_error <= 5.1e-7,
        "real_robot_commands_false": True,
    }
    status = "PASSED" if all(checks.values()) else "FAILED"
    payload = {
        "schema": "sceneforge/x2-stand-default-session-oracle-audit/v1",
        "status": status,
        "decision": "stand_default_session_parity_passed" if status == "PASSED" else "stand_default_session_blocked",
        "qualification": False,
        "frame_count": len(frame_rows),
        "policy_call_count": policy_calls,
        "metrics": {
            "maximum_stacked_observation_error": maximum_stacked_error,
            "maximum_raw_action_error": maximum_action_error,
            "maximum_joint_target_error": maximum_target_error,
        },
        "checks": checks,
        "sources": [
            {"path": str(path), "sha256": digest(path), "size": path.stat().st_size}
            for path in (frames, outputs, debug)
        ],
        "published_topics": [],
        "real_robot_commands": False,
    }
    atomic_json(receipt, payload)
    return 0 if status == "PASSED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
