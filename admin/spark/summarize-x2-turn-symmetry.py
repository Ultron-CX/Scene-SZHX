#!/usr/bin/env python3
"""Summarize left/right X2 MuJoCo/PhysX turning symmetry receipts.

This tool consumes two immutable ``x2-locomotion-ab/v2`` development
receipts.  It never drives either simulator and deliberately does not turn a
two-case comparison into qualification evidence.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
from typing import Any


SCHEMA = "sceneforge/x2-turn-symmetry/v1"
INPUT_SCHEMA = "sceneforge/x2-locomotion-ab/v2"
RELATIVE_SYMMETRY_LIMIT = 0.15
STOP_DISTANCE_ABSOLUTE_LIMIT_M = 0.10
STOP_DISTANCE_RELATIVE_LIMIT = 0.20
_SHA256 = re.compile(r"[0-9a-fA-F]{64}")
REQUIRED_FROZEN_HASH_KEYS = frozenset({
    "mujoco_model_sha256",
    "mujoco_stand_policy_sha256",
    "mujoco_walk_policy_sha256",
    "mujoco_joint_limit_source_sha256",
    "physx_stand_policy_sha256",
    "physx_walk_policy_sha256",
    "physx_joint_limit_source_sha256",
})


def _reject_nonfinite_json(value: str) -> None:
    raise ValueError(f"non-finite JSON constant is forbidden: {value}")


def _validate_finite_tree(value: Any, location: str = "$") -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError(f"non-finite JSON number is forbidden at {location}")
    if isinstance(value, dict):
        for key, child in value.items():
            _validate_finite_tree(child, f"{location}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _validate_finite_tree(child, f"{location}[{index}]")


def load_json(path: Path) -> dict:
    payload = json.loads(
        path.read_text(encoding="utf-8"),
        parse_constant=_reject_nonfinite_json,
    )
    if not isinstance(payload, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    _validate_finite_tree(payload)
    return payload


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _object(parent: dict, name: str, location: str) -> dict:
    value = parent.get(name)
    if not isinstance(value, dict):
        raise ValueError(f"{location}.{name} must be an object")
    return value


def _finite_number(parent: dict, name: str, location: str) -> float:
    value = parent.get(name)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{location}.{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{location}.{name} must be a finite number")
    return result


def _validate_sha256(value: Any, location: str) -> str:
    if not isinstance(value, str) or _SHA256.fullmatch(value) is None:
        raise ValueError(f"{location} must be a SHA-256 hex digest")
    return value.lower()


def _validate_json_contract(value: Any, location: str) -> Any:
    """Require a finite JSON value and return its canonical representation."""

    _validate_finite_tree(value, location)
    try:
        return json.loads(
            json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
        )
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{location} is not a finite JSON contract") from exc


def _optional_shared_contract(receipt: dict, name: str) -> Any | None:
    """Collect an optional contract without pretending a missing field exists.

    Newer producers may put a frozen contract at the receipt root or retain it
    in both engine summaries.  If more than one copy is present, all copies
    must already agree inside that receipt.
    """

    candidates: list[Any] = []
    if name in receipt:
        candidates.append(receipt[name])
    for authority in ("mujoco", "physx"):
        side = receipt.get(authority)
        if isinstance(side, dict) and name in side:
            candidates.append(side[name])
    if not candidates:
        return None
    canonical = [
        _validate_json_contract(value, f"{name}[{index}]")
        for index, value in enumerate(candidates)
    ]
    if any(value != canonical[0] for value in canonical[1:]):
        raise ValueError(f"conflicting {name} copies inside one A/B receipt")
    return canonical[0]


def _optional_frozen_hashes(receipt: dict) -> dict[str, str] | None:
    value = _optional_shared_contract(receipt, "frozen_input_hashes")
    if value is None:
        return None
    if not isinstance(value, dict) or not value:
        raise ValueError("frozen_input_hashes must be a non-empty object")
    normalized = {
        str(name): _validate_sha256(digest, f"frozen_input_hashes.{name}")
        for name, digest in value.items()
    }
    missing = sorted(REQUIRED_FROZEN_HASH_KEYS - set(normalized))
    if missing:
        raise ValueError(f"frozen_input_hashes lacks required identities: {missing}")
    return normalized


def _validate_input_provenance(receipt: dict, location: str) -> dict:
    inputs = _object(receipt, "inputs", location)
    normalized = {}
    for authority in ("mujoco", "physx"):
        item = _object(inputs, authority, f"{location}.inputs")
        normalized[authority] = {
            "path": str(item.get("path", "")),
            "sha256": _validate_sha256(
                item.get("sha256"), f"{location}.inputs.{authority}.sha256"
            ),
        }
    return normalized


def _validate_side(receipt: dict, authority: str, location: str) -> dict:
    side = _object(receipt, authority, location)
    safety = _object(side, "position_target_safety_contract", f"{location}.{authority}")
    if safety.get("schema") != "sceneforge/x2-position-target-safety-envelope/v1":
        raise ValueError(f"{location}.{authority} has an unsupported safety contract")
    safety = _validate_json_contract(
        safety, f"{location}.{authority}.position_target_safety_contract"
    )
    return {
        "absolute_turn_angle_rad": abs(
            _finite_number(side, "walk_turn_angle_rad", f"{location}.{authority}")
        ),
        "absolute_steady_yaw_rate_rps": abs(
            _finite_number(side, "steady_yaw_rate_rps", f"{location}.{authority}")
        ),
        "absolute_steady_forward_speed_mps": abs(
            _finite_number(side, "steady_forward_speed_mps", f"{location}.{authority}")
        ),
        "absolute_stop_distance_xy_m": abs(
            _finite_number(side, "stop_distance_xy_m", f"{location}.{authority}")
        ),
        "position_target_safety_contract": safety,
    }


def load_turn_receipt(path: Path, expected_scenario: str) -> dict:
    receipt = load_json(path)
    location = str(path)
    exact_fields = {
        "schema": INPUT_SCHEMA,
        "execution_status": "COMPLETED",
        "gate_status": "PASSED",
        "status": "PASSED",
        "scenario": expected_scenario,
    }
    for name, expected in exact_fields.items():
        if receipt.get(name) != expected:
            raise ValueError(f"{location}.{name} must equal {expected!r}")
    for name in ("case_gate_passed", "common_metric_subset_passed"):
        if receipt.get(name) is not True:
            raise ValueError(f"{location}.{name} must be true")
    if receipt.get("qualification") is not False:
        raise ValueError(f"{location}.qualification must be false")
    if receipt.get("real_robot_commands") is not False:
        raise ValueError(f"{location}.real_robot_commands must be false")

    checks = _object(receipt, "checks", location)
    if not checks or any(value is not True for value in checks.values()):
        raise ValueError(f"{location}.checks must be non-empty and all true")
    for required_check in (
        "same_velocity_command_calibration",
        "same_position_target_safety_envelope",
        "real_robot_commands_false",
    ):
        if checks.get(required_check) is not True:
            raise ValueError(f"{location}.checks.{required_check} must be true")

    command = _object(receipt, "command", location)
    command_values = {
        name: _finite_number(command, name, f"{location}.command")
        for name in ("x_mps", "y_mps", "yaw_rps", "walk_s")
    }
    if command_values["walk_s"] <= 0.0:
        raise ValueError(f"{location}.command.walk_s must be positive")
    if expected_scenario == "turn_left" and command_values["yaw_rps"] <= 0.0:
        raise ValueError(f"{location} is not a positive-yaw left turn")
    if expected_scenario == "turn_right" and command_values["yaw_rps"] >= 0.0:
        raise ValueError(f"{location} is not a negative-yaw right turn")

    mujoco = _validate_side(receipt, "mujoco", location)
    physx = _validate_side(receipt, "physx", location)
    if mujoco["position_target_safety_contract"] != physx["position_target_safety_contract"]:
        raise ValueError(f"{location} engine safety contracts differ")

    return {
        "receipt": receipt,
        "command": command_values,
        "mujoco": mujoco,
        "physx": physx,
        "position_target_safety_contract": mujoco["position_target_safety_contract"],
        "velocity_command_calibration": _optional_shared_contract(
            receipt, "velocity_command_calibration"
        ),
        "frozen_input_hashes": _optional_frozen_hashes(receipt),
        "provenance": _validate_input_provenance(receipt, location),
    }


def _relative_difference(left: float, right: float) -> float:
    difference = abs(left - right)
    mean_magnitude = (abs(left) + abs(right)) / 2.0
    if mean_magnitude <= 1e-12:
        return 0.0 if difference <= 1e-12 else difference / 1e-12
    return difference / mean_magnitude


def _engine_symmetry(left: dict, right: dict) -> dict:
    angle_relative = _relative_difference(
        left["absolute_turn_angle_rad"], right["absolute_turn_angle_rad"]
    )
    yaw_relative = _relative_difference(
        left["absolute_steady_yaw_rate_rps"],
        right["absolute_steady_yaw_rate_rps"],
    )
    speed_relative = _relative_difference(
        left["absolute_steady_forward_speed_mps"],
        right["absolute_steady_forward_speed_mps"],
    )
    stop_delta = abs(
        left["absolute_stop_distance_xy_m"] - right["absolute_stop_distance_xy_m"]
    )
    stop_tolerance = max(
        STOP_DISTANCE_ABSOLUTE_LIMIT_M,
        STOP_DISTANCE_RELATIVE_LIMIT
        * max(
            left["absolute_stop_distance_xy_m"],
            right["absolute_stop_distance_xy_m"],
        ),
    )
    return {
        "left": {
            name: value for name, value in left.items()
            if name != "position_target_safety_contract"
        },
        "right": {
            name: value for name, value in right.items()
            if name != "position_target_safety_contract"
        },
        "difference": {
            "absolute_turn_angle_rad": abs(
                left["absolute_turn_angle_rad"] - right["absolute_turn_angle_rad"]
            ),
            "turn_angle_relative": angle_relative,
            "absolute_steady_yaw_rate_rps": abs(
                left["absolute_steady_yaw_rate_rps"]
                - right["absolute_steady_yaw_rate_rps"]
            ),
            "steady_yaw_rate_relative": yaw_relative,
            "absolute_steady_forward_speed_mps": abs(
                left["absolute_steady_forward_speed_mps"]
                - right["absolute_steady_forward_speed_mps"]
            ),
            "steady_forward_speed_relative": speed_relative,
            "absolute_stop_distance_xy_m": stop_delta,
            "stop_distance_tolerance_m": stop_tolerance,
        },
        "checks": {
            "turn_angle_relative_difference": (
                angle_relative <= RELATIVE_SYMMETRY_LIMIT
            ),
            "steady_yaw_rate_relative_difference": (
                yaw_relative <= RELATIVE_SYMMETRY_LIMIT
            ),
            "steady_forward_speed_relative_difference": (
                speed_relative <= RELATIVE_SYMMETRY_LIMIT
            ),
            "stop_distance_difference": stop_delta <= stop_tolerance,
        },
    }


def build_report(left_path: Path, right_path: Path) -> dict:
    left = load_turn_receipt(left_path, "turn_left")
    right = load_turn_receipt(right_path, "turn_right")
    left_command = left["command"]
    right_command = right["command"]

    command_checks = {
        "opposite_yaw_commands": math.isclose(
            left_command["yaw_rps"],
            -right_command["yaw_rps"],
            rel_tol=1e-9,
            abs_tol=1e-12,
        ),
        "same_forward_command": math.isclose(
            left_command["x_mps"], right_command["x_mps"], abs_tol=1e-12
        ),
        "same_lateral_command": math.isclose(
            left_command["y_mps"], right_command["y_mps"], abs_tol=1e-12
        ),
        "same_walk_duration": math.isclose(
            left_command["walk_s"], right_command["walk_s"], abs_tol=1e-12
        ),
    }
    safety_matches = (
        left["position_target_safety_contract"]
        == right["position_target_safety_contract"]
    )

    left_calibration = left["velocity_command_calibration"]
    right_calibration = right["velocity_command_calibration"]
    calibration_available = left_calibration is not None or right_calibration is not None
    calibration_matches = (
        left_calibration == right_calibration
        if calibration_available else True
    )

    left_hashes = left["frozen_input_hashes"]
    right_hashes = right["frozen_input_hashes"]
    hashes_available = left_hashes is not None or right_hashes is not None
    hashes_match = left_hashes == right_hashes if hashes_available else True

    engines = {
        authority: _engine_symmetry(left[authority], right[authority])
        for authority in ("mujoco", "physx")
    }
    checks = {
        "both_case_gates_passed": True,
        **command_checks,
        "same_position_target_safety_contract": safety_matches,
        "velocity_command_calibration_available": (
            left_calibration is not None and right_calibration is not None
        ),
        "available_velocity_command_calibration_matches": calibration_matches,
        "frozen_input_hashes_available": (
            left_hashes is not None and right_hashes is not None
        ),
        "available_frozen_input_hashes_match": hashes_match,
    }
    for authority, summary in engines.items():
        for name, value in summary["checks"].items():
            checks[f"{authority}_{name}"] = value

    gate_passed = all(checks.values())
    return {
        "schema": SCHEMA,
        "execution_status": "COMPLETED",
        "gate_status": "PASSED" if gate_passed else "FAILED",
        "status": "PASSED" if gate_passed else "FAILED",
        "decision": (
            "record_turn_symmetry_and_continue_cold_start_matrix"
            if gate_passed else "turn_symmetry_alignment_required"
        ),
        "qualification": False,
        "functional_equivalence": False,
        "symmetry_gate_passed": gate_passed,
        "thresholds": {
            "relative_difference_formula": (
                "abs(left-right)/mean(abs(left),abs(right))"
            ),
            "turn_angle_relative_limit": RELATIVE_SYMMETRY_LIMIT,
            "steady_yaw_rate_relative_limit": RELATIVE_SYMMETRY_LIMIT,
            "steady_forward_speed_relative_limit": RELATIVE_SYMMETRY_LIMIT,
            "stop_distance_absolute_limit_m": STOP_DISTANCE_ABSOLUTE_LIMIT_M,
            "stop_distance_relative_limit": STOP_DISTANCE_RELATIVE_LIMIT,
        },
        "commands": {
            "left": left_command,
            "right": right_command,
        },
        "checks": checks,
        "engines": engines,
        "shared_contracts": {
            "position_target_safety_contract": left[
                "position_target_safety_contract"
            ],
            "velocity_command_calibration_available": calibration_available,
            "velocity_command_calibration": (
                left_calibration if calibration_matches else None
            ),
            "frozen_input_hashes_available": hashes_available,
            "frozen_input_hashes": left_hashes if hashes_match else None,
        },
        "inputs": {
            "left_ab": {
                "path": str(left_path),
                "sha256": sha256(left_path),
                "source_results": left["provenance"],
            },
            "right_ab": {
                "path": str(right_path),
                "sha256": sha256(right_path),
                "source_results": right["provenance"],
            },
        },
        "limitations": [
            "This is a two-case development symmetry summary, not qualification.",
            "Calibration and frozen-input hashes must be present in both A/B inputs; missing, partial, or mismatched identity fails the symmetry gate.",
            "Five cold-start sequences and the remaining locomotion matrix are outside this summary.",
        ],
        "published_command_topics": [],
        "real_robot_commands": False,
    }


def atomic_write(path: Path, payload: dict) -> None:
    """Publish finite JSON atomically without replacing an existing receipt."""

    if path.exists():
        raise FileExistsError(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.parent / f".{path.name}.{os.getpid()}.tmp"
    try:
        with temporary.open("x", encoding="utf-8") as stream:
            stream.write(
                json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n"
            )
            stream.flush()
            os.fsync(stream.fileno())
        # A hard link is an atomic create-if-absent operation on the same
        # filesystem; unlike os.replace it can never overwrite evidence.
        os.link(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--left", type=Path, required=True)
    parser.add_argument("--right", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = build_report(args.left.resolve(), args.right.resolve())
    atomic_write(args.output.resolve(), report)
    print(args.output.resolve())
    return 0 if report["status"] == "PASSED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
