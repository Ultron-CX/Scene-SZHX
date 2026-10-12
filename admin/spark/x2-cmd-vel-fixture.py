#!/usr/bin/env python3
"""Run-scoped simulation-only cmd_vel publisher with atomic command updates."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import time
from typing import Any


COMMAND_SCHEMA = "sceneforge/x2-sim-cmd-vel-command/v1"
RECEIPT_SCHEMA = "sceneforge/x2-sim-cmd-vel-fixture/v1"


def load_command(path: Path, previous_sequence: int = -1) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema") != COMMAND_SCHEMA:
        raise ValueError(f"unexpected cmd_vel fixture schema: {data.get('schema')!r}")
    if data.get("real_robot_commands") is not False:
        raise ValueError("real_robot_commands must be false")
    sequence = int(data["sequence"])
    if sequence <= previous_sequence:
        raise ValueError(f"command sequence must increase: {sequence} <= {previous_sequence}")
    values = {
        "linear_x_mps": float(data["linear_x_mps"]),
        "linear_y_mps": float(data.get("linear_y_mps", 0.0)),
        "angular_z_rps": float(data.get("angular_z_rps", 0.0)),
    }
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError("cmd_vel fixture command contains NaN or Inf")
    if abs(values["linear_x_mps"]) > 1.0 or abs(values["linear_y_mps"]) > 1.0 or abs(values["angular_z_rps"]) > 1.0:
        raise ValueError("cmd_vel fixture command exceeds the vendor controller ranges")
    return {"sequence": sequence, **values}


def atomic_write(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("odometry quaternion contains NaN or Inf")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9:
        raise ValueError("odometry quaternion has zero norm")
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def heading_correction(reference_yaw: float, current_yaw: float, kp: float, maximum: float) -> float:
    error = (reference_yaw - current_yaw + math.pi) % (2.0 * math.pi) - math.pi
    return max(-maximum, min(maximum, kp * error))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--command", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--heading-hold", action="store_true")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--heading-kp", type=float, default=1.0)
    parser.add_argument("--heading-max-correction-rps", type=float, default=0.3)
    parser.add_argument("--odom-timeout-s", type=float, default=5.0)
    args = parser.parse_args()
    if os.environ.get("REAL_ROBOT_COMMANDS", "false").strip().lower() not in ("0", "false", "no", "off"):
        raise SystemExit("REAL_ROBOT_COMMANDS must remain false")
    if not math.isfinite(args.rate_hz) or args.rate_hz <= 0.0 or args.rate_hz > 100.0:
        raise SystemExit("rate-hz must be within (0, 100]")
    if not math.isfinite(args.heading_kp) or args.heading_kp <= 0.0:
        raise SystemExit("heading-kp must be positive and finite")
    if not math.isfinite(args.heading_max_correction_rps) or not 0.0 < args.heading_max_correction_rps <= 1.0:
        raise SystemExit("heading-max-correction-rps must be within (0, 1]")
    if not math.isfinite(args.odom_timeout_s) or args.odom_timeout_s <= 0.0:
        raise SystemExit("odom-timeout-s must be positive and finite")

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = Node("x2_sim_cmd_vel_fixture")
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    publisher = node.create_publisher(Twist, args.topic, qos)
    odom_message_count = 0
    current_yaw = None
    reference_yaw = None
    last_odom_at = None
    max_abs_heading_error = 0.0
    max_abs_heading_correction = 0.0
    last_heading_correction = 0.0

    def on_odom(message: Odometry) -> None:
        nonlocal odom_message_count, current_yaw, reference_yaw, last_odom_at
        orientation = message.pose.pose.orientation
        current_yaw = quaternion_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
        if reference_yaw is None:
            reference_yaw = current_yaw
        odom_message_count += 1
        last_odom_at = time.time()

    if args.heading_hold:
        node.create_subscription(Odometry, args.odom_topic, on_odom, qos)
    stopped = False
    started_at = time.time()
    signature = None
    command = None
    sequence = -1
    message_count = 0
    command_updates = []
    errors = []
    published_angular_z_rps = 0.0

    def stop(_signum, _frame) -> None:
        nonlocal stopped
        stopped = True

    def receipt(status: str) -> dict[str, Any]:
        return {
            "schema": RECEIPT_SCHEMA,
            "status": status,
            "topic": args.topic,
            "rate_hz": args.rate_hz,
            "started_at_unix": started_at,
            "updated_at_unix": time.time(),
            "message_count": message_count,
            "last_sequence": sequence,
            "last_command": command,
            "command_updates": command_updates,
            "published_angular_z_rps": published_angular_z_rps,
            "heading_hold": {
                "enabled": args.heading_hold,
                "odom_topic": args.odom_topic if args.heading_hold else None,
                "kp": args.heading_kp if args.heading_hold else None,
                "max_correction_rps": args.heading_max_correction_rps if args.heading_hold else None,
                "odom_message_count": odom_message_count,
                "reference_yaw_rad": reference_yaw,
                "current_yaw_rad": current_yaw,
                "last_odom_at_unix": last_odom_at,
                "max_abs_heading_error_rad": max_abs_heading_error,
                "max_abs_heading_correction_rps": max_abs_heading_correction,
                "last_heading_correction_rps": last_heading_correction,
            },
            "errors": errors,
            "real_robot_commands": False,
        }

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        while rclpy.ok() and not stopped:
            stat = args.command.stat()
            candidate_signature = (stat.st_mtime_ns, stat.st_size)
            if candidate_signature != signature:
                command = load_command(args.command, sequence)
                sequence = int(command["sequence"])
                signature = candidate_signature
                command_updates.append({"sequence": sequence, "observed_at_unix": time.time(), **command})
            assert command is not None
            if args.heading_hold and reference_yaw is None:
                if time.time() - started_at > args.odom_timeout_s:
                    raise RuntimeError("heading hold received no authoritative odometry")
                rclpy.spin_once(node, timeout_sec=1.0 / args.rate_hz)
                continue
            correction = 0.0
            moving_straight = (
                abs(command["linear_x_mps"]) + abs(command["linear_y_mps"]) > 0.05
                and abs(command["angular_z_rps"]) < 1e-9
            )
            if args.heading_hold and moving_straight:
                assert reference_yaw is not None and current_yaw is not None
                error = (reference_yaw - current_yaw + math.pi) % (2.0 * math.pi) - math.pi
                correction = heading_correction(reference_yaw, current_yaw, args.heading_kp, args.heading_max_correction_rps)
                max_abs_heading_error = max(max_abs_heading_error, abs(error))
                max_abs_heading_correction = max(max_abs_heading_correction, abs(correction))
            last_heading_correction = correction
            message = Twist()
            message.linear.x = command["linear_x_mps"]
            message.linear.y = command["linear_y_mps"]
            published_angular_z_rps = command["angular_z_rps"] + correction
            message.angular.z = published_angular_z_rps
            publisher.publish(message)
            message_count += 1
            if message_count == 1 or message_count % max(1, int(args.rate_hz)) == 0:
                atomic_write(args.receipt, receipt("RUNNING"))
            rclpy.spin_once(node, timeout_sec=1.0 / args.rate_hz)
    except Exception as exc:
        errors.append(str(exc))
        atomic_write(args.receipt, receipt("FAILED"))
        raise
    finally:
        terminal = "SUCCEEDED" if message_count > 0 and not errors else "FAILED"
        atomic_write(args.receipt, receipt(terminal))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if terminal == "SUCCEEDED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
