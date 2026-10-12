#!/usr/bin/env python3
"""Atomically record read-only X2 PhysX link transforms for offline rendering."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import tempfile
import time
from typing import Any, Mapping, Sequence


STATE_SCHEMA = "sceneforge/x2-geniesim-render-state/v1"
RECEIPT_SCHEMA = "sceneforge/x2-geniesim-render-state-receipt/v1"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def stamp_ns(stamp: Any) -> int:
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    if value < 0 or not 0 <= int(stamp.nanosec) < 1_000_000_000:
        raise ValueError("invalid ROS timestamp")
    return value


def finite(values: Sequence[Any], label: str) -> list[float]:
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} contains NaN or Inf")
    return result


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=path.parent, delete=False) as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
        temporary = Path(stream.name)
    os.replace(temporary, path)
    os.chmod(path, 0o644)


@dataclass(frozen=True)
class BaseState:
    stamp_ns: int
    position_xyz_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class TransformState:
    stamp_ns: int
    transforms: tuple[dict[str, Any], ...]


def base_from_message(message: Any) -> BaseState:
    pose = message.pose.pose
    position = tuple(finite((pose.position.x, pose.position.y, pose.position.z), "base position"))
    orientation = tuple(finite((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w), "base orientation"))
    if math.sqrt(sum(value * value for value in orientation)) < 1e-9:
        raise ValueError("base orientation has zero norm")
    return BaseState(stamp_ns(message.header.stamp), position, orientation)


def transforms_from_message(message: Any) -> TransformState:
    if not message.transforms:
        raise ValueError("TFMessage contains no transforms")
    rows: list[dict[str, Any]] = []
    stamps: list[int] = []
    children: set[str] = set()
    for item in message.transforms:
        child = str(item.child_frame_id)
        if not child.startswith("/") or child in children:
            raise ValueError("render transform child paths must be unique absolute paths")
        children.add(child)
        stamps.append(stamp_ns(item.header.stamp))
        translation = finite((item.transform.translation.x, item.transform.translation.y, item.transform.translation.z), "translation")
        rotation = finite((item.transform.rotation.x, item.transform.rotation.y, item.transform.rotation.z, item.transform.rotation.w), "rotation")
        if math.sqrt(sum(value * value for value in rotation)) < 1e-9:
            raise ValueError("render transform has zero quaternion")
        rows.append({
            "parent_path": str(item.header.frame_id),
            "child_path": child,
            "translation_xyz_m": translation,
            "rotation_xyzw": rotation,
        })
    if max(stamps) != min(stamps):
        raise ValueError("render transforms do not share one simulation timestamp")
    return TransformState(stamps[0], tuple(rows))


class Sampler:
    def __init__(self, rate_hz: float, maximum_skew_s: float) -> None:
        self.period_ns = max(1, round(1_000_000_000 / rate_hz))
        self.maximum_skew_ns = round(maximum_skew_s * 1_000_000_000)
        self.base: BaseState | None = None
        self.transforms: TransformState | None = None
        self.next_ns: int | None = None
        self.last_ns: int | None = None
        self.sample_count = 0
        self.skew_rejections = 0
        self.rate_limited = 0
        self.out_of_order = 0
        self.malformed = 0

    def update_base(self, state: BaseState) -> dict[str, Any] | None:
        self.base = state
        return self.sample()

    def update_transforms(self, state: TransformState) -> dict[str, Any] | None:
        self.transforms = state
        return self.sample()

    def sample(self) -> dict[str, Any] | None:
        if self.base is None or self.transforms is None:
            return None
        skew = abs(self.base.stamp_ns - self.transforms.stamp_ns)
        if skew > self.maximum_skew_ns:
            self.skew_rejections += 1
            return None
        current = max(self.base.stamp_ns, self.transforms.stamp_ns)
        if self.last_ns is not None and current <= self.last_ns:
            self.out_of_order += 1
            return None
        if self.next_ns is not None and current < self.next_ns:
            self.rate_limited += 1
            return None
        self.next_ns = current + self.period_ns if self.next_ns is None else self.next_ns + ((current - self.next_ns) // self.period_ns + 1) * self.period_ns
        self.last_ns = current
        result = {
            "record_type": "sample",
            "schema": STATE_SCHEMA,
            "sequence": self.sample_count,
            "sim_time_ns": current,
            "sim_time_s": current / 1_000_000_000,
            "pair_skew_s": skew / 1_000_000_000,
            "base_position_xyz_m": list(self.base.position_xyz_m),
            "base_orientation_xyzw": list(self.base.orientation_xyzw),
            "transforms": list(self.transforms.transforms),
        }
        self.sample_count += 1
        return result

    def counters(self) -> dict[str, int]:
        return {
            "sample_count": self.sample_count,
            "skew_rejection_count": self.skew_rejections,
            "rate_limited_count": self.rate_limited,
            "out_of_order_count": self.out_of_order,
            "malformed_message_count": self.malformed,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tf-topic", default="/tf_render")
    parser.add_argument("--base-topic", default="/odom")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--sample-rate-hz", type=float, default=30.0)
    parser.add_argument("--maximum-skew-s", type=float, default=0.05)
    parser.add_argument("--minimum-samples", type=int, default=180)
    parser.add_argument("--wall-timeout-s", type=float, default=900.0)
    args = parser.parse_args()
    if args.sample_rate_hz <= 0 or args.maximum_skew_s < 0 or args.minimum_samples < 1 or args.wall_timeout_s <= 0:
        raise SystemExit("invalid recorder limits")
    if args.output.exists() or args.receipt.exists() or args.output.with_suffix(args.output.suffix + ".partial").exists():
        raise SystemExit("refusing to overwrite render-state evidence")

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from tf2_msgs.msg import TFMessage

    sampler = Sampler(args.sample_rate_hz, args.maximum_skew_s)
    partial = args.output.with_suffix(args.output.suffix + ".partial")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    stream = partial.open("x", encoding="utf-8")
    header = {
        "record_type": "schema", "schema": STATE_SCHEMA, "sample_clock": "simulation_time",
        "tf_topic": args.tf_topic, "base_topic": args.base_topic, "target_sample_rate_hz": args.sample_rate_hz,
        "authoritative_physics_source": "geniesim_isaac_physx", "read_only": True,
        "published_topics": [], "real_robot_commands": False,
    }
    stream.write(json.dumps(header, separators=(",", ":"), sort_keys=True) + "\n")
    started_at = utc_now()
    first_s: float | None = None
    last_s: float | None = None
    stopping = False
    stop_reason = "wall_timeout"
    fatal_error: str | None = None

    def write(sample: dict[str, Any] | None) -> None:
        nonlocal first_s, last_s
        if sample is None:
            return
        stream.write(json.dumps(sample, separators=(",", ":"), sort_keys=True) + "\n")
        first_s = sample["sim_time_s"] if first_s is None else first_s
        last_s = sample["sim_time_s"]

    class RecorderNode(Node):
        def __init__(self) -> None:
            super().__init__("sceneforge_x2_render_state_recorder")
            self.create_subscription(Odometry, args.base_topic, self.base, qos_profile_sensor_data)
            self.create_subscription(TFMessage, args.tf_topic, self.tf, qos_profile_sensor_data)

        def base(self, message: Any) -> None:
            try:
                write(sampler.update_base(base_from_message(message)))
            except (AttributeError, TypeError, ValueError) as error:
                sampler.malformed += 1
                self.get_logger().warning(str(error))

        def tf(self, message: Any) -> None:
            try:
                write(sampler.update_transforms(transforms_from_message(message)))
            except (AttributeError, TypeError, ValueError) as error:
                sampler.malformed += 1
                self.get_logger().warning(str(error))

    def stop(signum: int, _frame: Any) -> None:
        nonlocal stopping, stop_reason
        stopping, stop_reason = True, signal.Signals(signum).name

    rclpy.init()
    # rclpy installs its own signal handlers during init.  Install the recorder
    # handlers afterwards so SIGINT/SIGTERM seals the atomic evidence instead
    # of shutting the ROS context out from under the finalizer.
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node = RecorderNode()
    atomic_json(args.receipt, {
        "schema": RECEIPT_SCHEMA, "status": "RUNNING", "started_at": started_at,
        "sample_count": 0, "subscribed_topics": [args.tf_topic, args.base_topic],
        "published_topics": [], "read_only": True,
        "authoritative_physics_source": "geniesim_isaac_physx", "real_robot_commands": False,
    })
    deadline = time.monotonic() + args.wall_timeout_s
    try:
        while rclpy.ok() and not stopping and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except Exception as error:
        fatal_error = f"{type(error).__name__}: {error}"
        stop_reason = "exception"
    finally:
        try:
            node.destroy_node()
        except Exception as error:
            fatal_error = fatal_error or f"node cleanup failed: {type(error).__name__}: {error}"
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as error:
            fatal_error = fatal_error or f"ROS shutdown failed: {type(error).__name__}: {error}"
        status = "SUCCEEDED" if fatal_error is None and sampler.sample_count >= args.minimum_samples else "FAILED"
        summary = {
            "record_type": "summary", "schema": STATE_SCHEMA, "status": status,
            "stop_reason": stop_reason, "fatal_error": fatal_error, "sample_count": sampler.sample_count,
            "first_sim_time_s": first_s, "last_sim_time_s": last_s, "counters": sampler.counters(),
            "read_only": True, "real_robot_commands": False,
        }
        stream.write(json.dumps(summary, separators=(",", ":"), sort_keys=True) + "\n")
        stream.flush(); os.fsync(stream.fileno()); stream.close(); os.replace(partial, args.output)
        os.chmod(args.output, 0o644)
        receipt = {
            "schema": RECEIPT_SCHEMA, "status": status, "started_at": started_at, "finished_at": utc_now(),
            "output": str(args.output), "output_bytes": args.output.stat().st_size,
            "output_sha256": sha256_file(args.output), "sample_count": sampler.sample_count,
            "minimum_samples": args.minimum_samples, "first_sim_time_s": first_s, "last_sim_time_s": last_s,
            "counters": sampler.counters(), "subscribed_topics": [args.tf_topic, args.base_topic],
            "published_topics": [], "read_only": True, "authoritative_physics_source": "geniesim_isaac_physx",
            "real_robot_commands": False,
        }
        atomic_json(args.receipt, receipt)
    return 0 if status == "SUCCEEDED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
