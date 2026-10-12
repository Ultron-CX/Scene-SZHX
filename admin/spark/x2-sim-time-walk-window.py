#!/usr/bin/env python3
"""Seal a walking command window using authoritative PhysX simulator time.

This utility never opens a ROS transport and never emits robot commands.  It
only reads the run-scoped elastic-band status produced inside the authoritative
PhysX loop and atomically maintains a timing receipt.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
from typing import Any


STATUS_SCHEMA = "sceneforge/physx-elastic-band-status/v1"
RECEIPT_SCHEMA = "sceneforge/x2-physx-sim-time-walk-window/v1"


def _finite_vec3(value: Any, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{label} must be a three-element list")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{label} contains NaN or Inf")
    return result


def load_status(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema") != STATUS_SCHEMA:
        raise ValueError(f"unexpected PhysX status schema: {data.get('schema')!r}")
    if data.get("state") != "RELEASED":
        raise ValueError("walking is forbidden before the elastic band is RELEASED")
    if data.get("support_ratio") != 0.0 or data.get("force_norm_n") != 0.0:
        raise ValueError("walking is forbidden while suspension force remains")
    if data.get("force_is_exactly_zero") is not True:
        raise ValueError("elastic-band force is not exactly zero")
    if data.get("root_pinned") is not False or data.get("gravity_modified") is not False:
        raise ValueError("invalid root pin or gravity modification in PhysX status")
    if data.get("real_robot_commands") is not False:
        raise ValueError("real_robot_commands must be false")
    sim_time = float(data["sim_time_s"])
    if not math.isfinite(sim_time) or sim_time < 0.0:
        raise ValueError("sim_time_s must be non-negative and finite")
    _finite_vec3(data.get("body_position_m"), "body_position_m")
    _finite_vec3(data.get("body_linear_velocity_mps"), "body_linear_velocity_mps")
    return data


def snapshot(status: dict[str, Any]) -> dict[str, Any]:
    velocity = _finite_vec3(status["body_linear_velocity_mps"], "body_linear_velocity_mps")
    return {
        "sim_time_s": float(status["sim_time_s"]),
        "sample_count": int(status["sample_count"]),
        "body_position_m": _finite_vec3(status["body_position_m"], "body_position_m"),
        "body_linear_velocity_mps": velocity,
        "body_linear_speed_mps": math.sqrt(sum(item * item for item in velocity)),
        "elastic_band_state": status["state"],
        "elastic_band_support_ratio": float(status["support_ratio"]),
        "elastic_band_force_norm_n": float(status["force_norm_n"]),
    }


def atomic_write(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def load_receipt(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema") != RECEIPT_SCHEMA:
        raise ValueError(f"unexpected walk-window schema: {data.get('schema')!r}")
    if data.get("real_robot_commands") is not False:
        raise ValueError("walk-window receipt must keep real_robot_commands=false")
    return data


def start(
    status_path: Path,
    receipt_path: Path,
    speed: float,
    duration: float,
    settle: float,
    deceleration_speed: float | None = None,
    deceleration_duration: float = 0.0,
) -> None:
    if receipt_path.exists():
        raise FileExistsError(f"refusing to overwrite walk-window receipt: {receipt_path}")
    if not math.isfinite(speed) or abs(speed) < 0.3:
        raise ValueError("the vendor locomotion fixture requires abs(speed) >= 0.3 m/s")
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("walk duration must be positive and finite")
    if not math.isfinite(settle) or settle <= 0.0:
        raise ValueError("stop-settle duration must be positive and finite")
    if not math.isfinite(deceleration_duration) or deceleration_duration < 0.0:
        raise ValueError("deceleration duration must be non-negative and finite")
    if deceleration_duration > 0.0:
        if deceleration_speed is None or not math.isfinite(deceleration_speed):
            raise ValueError("a finite deceleration speed is required")
        if abs(deceleration_speed) < 0.3:
            raise ValueError("the vendor locomotion fixture requires abs(deceleration_speed) >= 0.3 m/s")
        if speed * deceleration_speed <= 0.0 or abs(deceleration_speed) >= abs(speed):
            raise ValueError("deceleration speed must have the same sign and a smaller magnitude")
    elif deceleration_speed not in (None, 0.0):
        raise ValueError("deceleration speed requires a positive deceleration duration")
    status = load_status(status_path)
    payload = {
        "schema": RECEIPT_SCHEMA,
        "status": "WALKING",
        "speed_command_mps": speed,
        "requested_walk_duration_s": duration,
        "requested_stop_settle_s": settle,
        "deceleration_speed_command_mps": deceleration_speed if deceleration_duration > 0.0 else None,
        "requested_deceleration_duration_s": deceleration_duration,
        "start": snapshot(status),
        "deceleration_start": None,
        "command_stop": None,
        "settled": None,
        "checks": {},
        "real_robot_commands": False,
    }
    atomic_write(receipt_path, payload)


def reached(status_path: Path, receipt_path: Path, phase: str) -> bool:
    status = load_status(status_path)
    receipt = load_receipt(receipt_path)
    if phase == "walk":
        origin = receipt["start"]
        required = float(receipt["requested_walk_duration_s"])
    elif phase == "deceleration":
        origin = receipt.get("deceleration_start")
        if not isinstance(origin, dict):
            raise ValueError("deceleration_start snapshot is missing")
        required = float(receipt["requested_deceleration_duration_s"])
    else:
        origin = receipt.get("command_stop")
        if not isinstance(origin, dict):
            raise ValueError("command_stop snapshot is missing")
        required = float(receipt["requested_stop_settle_s"])
    return float(status["sim_time_s"]) - float(origin["sim_time_s"]) >= required


def mark_deceleration(status_path: Path, receipt_path: Path) -> None:
    status = load_status(status_path)
    receipt = load_receipt(receipt_path)
    if receipt.get("status") != "WALKING" or receipt.get("deceleration_start") is not None:
        raise ValueError("walk-window is not ready to begin deceleration")
    required = float(receipt.get("requested_deceleration_duration_s", 0.0))
    if required <= 0.0:
        raise ValueError("this walk-window has no deceleration phase")
    deceleration_start = snapshot(status)
    observed = deceleration_start["sim_time_s"] - float(receipt["start"]["sim_time_s"])
    if observed < float(receipt["requested_walk_duration_s"]):
        raise ValueError("refusing to decelerate before requested simulator-time walk duration")
    receipt["deceleration_start"] = deceleration_start
    receipt["observed_walk_duration_s"] = observed
    receipt["status"] = "DECELERATING"
    atomic_write(receipt_path, receipt)


def mark_stop(status_path: Path, receipt_path: Path) -> None:
    status = load_status(status_path)
    receipt = load_receipt(receipt_path)
    expected_state = "DECELERATING" if float(receipt.get("requested_deceleration_duration_s", 0.0)) > 0.0 else "WALKING"
    if receipt.get("status") != expected_state or receipt.get("command_stop") is not None:
        raise ValueError("walk-window is not in the WALKING state")
    stop_snapshot = snapshot(status)
    if expected_state == "DECELERATING":
        origin = receipt["deceleration_start"]
        observed_deceleration = stop_snapshot["sim_time_s"] - float(origin["sim_time_s"])
        if observed_deceleration < float(receipt["requested_deceleration_duration_s"]):
            raise ValueError("refusing to stop before requested simulator-time deceleration duration")
        receipt["observed_deceleration_duration_s"] = observed_deceleration
    else:
        observed = stop_snapshot["sim_time_s"] - float(receipt["start"]["sim_time_s"])
        if observed < float(receipt["requested_walk_duration_s"]):
            raise ValueError("refusing to stop before requested simulator-time walk duration")
        receipt["observed_walk_duration_s"] = observed
    receipt["command_stop"] = stop_snapshot
    receipt["status"] = "SETTLING"
    atomic_write(receipt_path, receipt)


def finish(status_path: Path, receipt_path: Path) -> None:
    status = load_status(status_path)
    receipt = load_receipt(receipt_path)
    if receipt.get("status") != "SETTLING" or not isinstance(receipt.get("command_stop"), dict):
        raise ValueError("walk-window is not in the SETTLING state")
    settled = snapshot(status)
    observed_settle = settled["sim_time_s"] - float(receipt["command_stop"]["sim_time_s"])
    start_position = receipt["start"]["body_position_m"]
    stop_position = receipt["command_stop"]["body_position_m"]
    forward = float(stop_position[0]) - float(start_position[0])
    lateral = float(stop_position[1]) - float(start_position[1])
    expected = abs(float(receipt["speed_command_mps"])) * float(receipt["requested_walk_duration_s"])
    expected += abs(float(receipt.get("deceleration_speed_command_mps") or 0.0)) * float(
        receipt.get("requested_deceleration_duration_s", 0.0)
    )
    checks = {
        "sim_time_walk_duration_met": float(receipt["observed_walk_duration_s"]) >= float(receipt["requested_walk_duration_s"]),
        "sim_time_stop_settle_met": observed_settle >= float(receipt["requested_stop_settle_s"]),
        "elastic_band_zero_through_finish": settled["elastic_band_force_norm_n"] == 0.0,
        "forward_displacement_positive": forward > 0.0,
        "settled_linear_speed_le_0_05": settled["body_linear_speed_mps"] <= 0.05,
    }
    receipt.update({
        "status": "PASSED" if all(checks.values()) else "FAILED",
        "settled": settled,
        "observed_stop_settle_s": observed_settle,
        "expected_distance_m": expected,
        "forward_displacement_m": forward,
        "lateral_displacement_m": lateral,
        "forward_displacement_ratio": forward / expected if expected > 0.0 else 0.0,
        "checks": checks,
    })
    atomic_write(receipt_path, receipt)
    if receipt["status"] != "PASSED":
        raise RuntimeError(f"walk-window checks failed: {checks}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "action",
        choices=("start", "walk-reached", "mark-deceleration", "deceleration-reached", "mark-stop", "settle-reached", "finish"),
    )
    parser.add_argument("--status", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--speed", type=float)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--settle", type=float)
    parser.add_argument("--deceleration-speed", type=float)
    parser.add_argument("--deceleration-duration", type=float, default=0.0)
    args = parser.parse_args()
    if args.action == "start":
        if args.speed is None or args.duration is None or args.settle is None:
            parser.error("start requires --speed, --duration and --settle")
        start(
            args.status,
            args.receipt,
            args.speed,
            args.duration,
            args.settle,
            args.deceleration_speed,
            args.deceleration_duration,
        )
    elif args.action == "walk-reached":
        return 0 if reached(args.status, args.receipt, "walk") else 3
    elif args.action == "mark-deceleration":
        mark_deceleration(args.status, args.receipt)
    elif args.action == "deceleration-reached":
        return 0 if reached(args.status, args.receipt, "deceleration") else 3
    elif args.action == "mark-stop":
        mark_stop(args.status, args.receipt)
    elif args.action == "settle-reached":
        return 0 if reached(args.status, args.receipt, "settle") else 3
    else:
        finish(args.status, args.receipt)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
