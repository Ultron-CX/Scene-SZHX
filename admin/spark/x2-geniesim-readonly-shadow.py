#!/usr/bin/env python3
"""Run the independent X2 policy as a read-only Genie Sim state shadow.

The node subscribes to authoritative PhysX state and computes policy outputs,
but deliberately creates no ROS publisher.  Simulation timestamps, never wall
time, decide when the 50 Hz policy session advances.  Wall time is used only as
an outer process timeout and for measuring inference cost.
"""

from __future__ import annotations

import argparse
import gc
import hashlib
import json
import math
import os
import signal
import tempfile
import time
from pathlib import Path
from typing import Iterable, Sequence


SCHEMA = "sceneforge/x2-geniesim-readonly-shadow/v1"


def finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def normalize_quaternion_xyzw(values: Sequence[float]) -> tuple[float, float, float, float]:
    if len(values) != 4 or not finite(values):
        raise ValueError("quaternion must contain four finite values")
    norm = math.sqrt(sum(float(value) ** 2 for value in values))
    if norm <= 1e-12:
        raise ValueError("quaternion norm is zero")
    return tuple(float(value) / norm for value in values)


def multiply_quaternion_xyzw(
    left: Sequence[float], right: Sequence[float]
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = normalize_quaternion_xyzw(left)
    rx, ry, rz, rw = normalize_quaternion_xyzw(right)
    return normalize_quaternion_xyzw((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def rotate_world_to_body(
    quaternion_world_body_xyzw: Sequence[float], vector_world: Sequence[float]
) -> tuple[float, float, float]:
    x, y, z, w = normalize_quaternion_xyzw(quaternion_world_body_xyzw)
    vx, vy, vz = (float(value) for value in vector_world)
    return (
        (1 - 2 * (y * y + z * z)) * vx
        + 2 * (x * y + w * z) * vy
        + 2 * (x * z - w * y) * vz,
        2 * (x * y - w * z) * vx
        + (1 - 2 * (x * x + z * z)) * vy
        + 2 * (y * z + w * x) * vz,
        2 * (x * z + w * y) * vx
        + 2 * (y * z - w * x) * vy
        + (1 - 2 * (x * x + y * y)) * vz,
    )


def roll_pitch(quaternion_xyzw: Sequence[float]) -> tuple[float, float]:
    x, y, z, w = normalize_quaternion_xyzw(quaternion_xyzw)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_sine = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return roll, math.asin(pitch_sine)


def percentile(values: Sequence[float], quantile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    position = quantile * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_json(path: Path, payload: dict) -> None:
    if path.exists():
        raise FileExistsError(f"refusing to overwrite receipt: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        os.chmod(path, 0o444)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def parse_command(value: str) -> tuple[float, float, float]:
    result = tuple(float(item.strip()) for item in value.split(","))
    if len(result) != 3 or not finite(result):
        raise argparse.ArgumentTypeError("velocity command must be three finite comma-separated values")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--receipt", required=True)
    parser.add_argument("--velocity-command", type=parse_command, default=(0.3, 0.0, 0.0))
    parser.add_argument("--policy-rate-hz", type=float, default=50.0)
    parser.add_argument("--maximum-sim-duration-s", type=float, default=4.0)
    parser.add_argument("--maximum-wall-duration-s", type=float, default=180.0)
    parser.add_argument("--maximum-pose-skew-s", type=float, default=0.10)
    parser.add_argument("--minimum-inferences", type=int, default=150)
    parser.add_argument("--fixed-base", action="store_true")
    args, ros_args = parser.parse_known_args()
    if ros_args and ros_args[0] != "--ros-args":
        parser.error(f"unrecognized arguments: {' '.join(ros_args)}")
    if args.policy_rate_hz <= 0 or args.maximum_sim_duration_s <= 0:
        parser.error("rates and durations must be positive")
    if args.maximum_wall_duration_s <= 0 or args.maximum_pose_skew_s <= 0:
        parser.error("wall duration and pose skew must be positive")
    if args.minimum_inferences < 1:
        parser.error("minimum inferences must be positive")

    model = Path(args.model).resolve()
    receipt = Path(args.receipt).resolve()
    if not model.is_file():
        raise FileNotFoundError(model)
    if receipt.exists():
        raise FileExistsError(f"refusing to overwrite receipt: {receipt}")

    import numpy as np
    import onnxruntime as ort
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage

    from sceneforge.x2_control import X2_CPGWALK_JOINT_NAMES
    from sceneforge.x2_session import StateDrivenX2ControllerSession, X2PhysicsState

    options = ort.SessionOptions()
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    inference = ort.InferenceSession(
        str(model), sess_options=options, providers=["CPUExecutionProvider"]
    )
    inputs = inference.get_inputs()
    outputs = inference.get_outputs()
    if len(inputs) != 1 or not outputs:
        raise ValueError("policy must have exactly one input and at least one output")

    class OnnxPolicy:
        def __init__(self) -> None:
            self.elapsed_ms: list[float] = []
            self.output_digest = hashlib.sha256()

        def __call__(self, values: tuple[float, ...]) -> tuple[float, ...]:
            matrix = np.asarray(values, dtype=np.float32).reshape(1, -1)
            started = time.perf_counter()
            result = inference.run([outputs[0].name], {inputs[0].name: matrix})[0]
            self.elapsed_ms.append((time.perf_counter() - started) * 1000.0)
            flat = np.asarray(result, dtype=np.float32).reshape(-1)
            if flat.size != 17 or not np.isfinite(flat).all():
                raise ValueError("ONNX output is not a finite 17-value vector")
            self.output_digest.update(flat.tobytes(order="C"))
            return tuple(float(value) for value in flat)

    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )
    policy = OnnxPolicy()
    policy_period = 1.0 / args.policy_rate_hz

    class Shadow(Node):
        def __init__(self) -> None:
            super().__init__("x2_geniesim_readonly_shadow")
            self.session = StateDrivenX2ControllerSession(policy)
            self.raw_state_count = 0
            self.inference_count = 0
            self.scheduler_skip_count = 0
            self.waiting_pose_count = 0
            self.nonmonotonic_stamp_count = 0
            self.pose_skew_reject_count = 0
            self.session_reject_count = 0
            self.last_error: str | None = None
            self.first_sim_time: float | None = None
            self.last_sim_time: float | None = None
            self.last_raw_sim_time: float | None = None
            self.last_sampled_sequence: int | None = None
            self.first_sampled_sequence: int | None = None
            self.last_sampled_sim_time: float | None = None
            self.next_policy_sim_time: float | None = None
            self.sample_gaps: list[float] = []
            self.max_output_abs = 0.0
            self.status_counts: dict[str, int] = {}
            self.latest_odom: Odometry | None = None
            self.latest_odom_stamp: float | None = None
            self.world_footprint = (0.0, 0.0, 0.0, 1.0)
            self.footprint_base = (0.0, 0.0, 0.0, 1.0)
            self.latest_tf_stamp: float | None = None
            self.done = False
            self.create_subscription(JointState, "/joint_states", self.on_joint_state, qos)
            if not args.fixed_base:
                self.create_subscription(Odometry, "/odom", self.on_odom, qos)
                self.create_subscription(TFMessage, "/tf", self.on_tf, qos)

        def on_odom(self, message: Odometry) -> None:
            orientation = message.pose.pose.orientation
            self.world_footprint = normalize_quaternion_xyzw(
                (orientation.x, orientation.y, orientation.z, orientation.w)
            )
            self.latest_odom = message
            self.latest_odom_stamp = stamp_seconds(message.header.stamp)

        def on_tf(self, message: TFMessage) -> None:
            residual = next(
                (
                    transform
                    for transform in message.transforms
                    if transform.header.frame_id.lstrip("/") == "base_footprint"
                    and transform.child_frame_id.lstrip("/") == "base_link"
                ),
                None,
            )
            if residual is None:
                return
            orientation = residual.transform.rotation
            self.footprint_base = normalize_quaternion_xyzw(
                (orientation.x, orientation.y, orientation.z, orientation.w)
            )
            self.latest_tf_stamp = stamp_seconds(residual.header.stamp)

        def reject(self, message: str) -> None:
            self.session_reject_count += 1
            self.last_error = message
            if self.session_reject_count <= 5:
                self.get_logger().error(message)

        def on_joint_state(self, message: JointState) -> None:
            self.raw_state_count += 1
            sequence = self.raw_state_count
            sim_time = stamp_seconds(message.header.stamp)
            if not math.isfinite(sim_time) or sim_time <= 0.0:
                self.reject("joint state has invalid simulation timestamp")
                return
            if self.last_raw_sim_time is not None and sim_time <= self.last_raw_sim_time:
                self.nonmonotonic_stamp_count += 1
                return
            self.last_raw_sim_time = sim_time
            if not args.fixed_base:
                if self.latest_odom is None or self.latest_odom_stamp is None or self.latest_tf_stamp is None:
                    self.waiting_pose_count += 1
                    return
                pose_skew = max(
                    abs(sim_time - self.latest_odom_stamp),
                    abs(sim_time - self.latest_tf_stamp),
                )
                if pose_skew > args.maximum_pose_skew_s:
                    self.pose_skew_reject_count += 1
                    return
            if self.next_policy_sim_time is None:
                self.next_policy_sim_time = sim_time
            if sim_time + 1e-6 < self.next_policy_sim_time:
                self.scheduler_skip_count += 1
                return
            lookup = {
                name: (float(position), float(velocity))
                for name, position, velocity in zip(
                    message.name,
                    message.position,
                    message.velocity if len(message.velocity) == len(message.name) else [0.0] * len(message.name),
                )
            }
            missing = sorted(set(X2_CPGWALK_JOINT_NAMES) - set(lookup))
            if missing:
                self.reject(f"joint state is missing X2 CPGWalk joints: {missing}")
                return
            if args.fixed_base:
                world_base = (0.0, 0.0, 0.0, 1.0)
                roll = pitch = 0.0
                linear_xyz = angular_xyz = (0.0, 0.0, 0.0)
            else:
                world_base = multiply_quaternion_xyzw(
                    self.world_footprint, self.footprint_base
                )
                roll, pitch = roll_pitch(world_base)
                odom = self.latest_odom
                linear = odom.twist.twist.linear
                angular = odom.twist.twist.angular
                linear_xyz = (linear.x, linear.y, linear.z)
                angular_xyz = (angular.x, angular.y, angular.z)
            try:
                result = self.session.update(X2PhysicsState(
                    source_sequence=sequence,
                    sim_time=sim_time,
                    command_stamp_sim_time=sim_time,
                    velocity_command=args.velocity_command,
                    joint_position=tuple(lookup[name][0] for name in X2_CPGWALK_JOINT_NAMES),
                    joint_velocity=tuple(lookup[name][1] for name in X2_CPGWALK_JOINT_NAMES),
                    base_angular_velocity=angular_xyz,
                    observation_gravity=rotate_world_to_body(world_base, (0.0, 0.0, -1.0)),
                    base_linear_velocity=linear_xyz,
                    roll=roll,
                    pitch=pitch,
                ))
            except Exception as exc:
                self.reject(f"state-driven session rejected live state: {exc}")
                self.done = True
                return
            if self.last_sampled_sim_time is not None:
                self.sample_gaps.append(sim_time - self.last_sampled_sim_time)
            self.inference_count += 1
            self.status_counts[result.status] = self.status_counts.get(result.status, 0) + 1
            self.max_output_abs = max(
                self.max_output_abs,
                *(abs(value) for value in result.policy_action),
            )
            self.last_sampled_sequence = sequence
            self.first_sampled_sequence = (
                sequence if self.first_sampled_sequence is None else self.first_sampled_sequence
            )
            self.last_sampled_sim_time = sim_time
            while self.next_policy_sim_time <= sim_time + 1e-6:
                self.next_policy_sim_time += policy_period
            self.first_sim_time = sim_time if self.first_sim_time is None else self.first_sim_time
            self.last_sim_time = sim_time
            if sim_time - self.first_sim_time >= args.maximum_sim_duration_s:
                self.done = True

        def result(self, status: str, timed_out: bool) -> dict:
            sim_duration = (
                self.last_sim_time - self.first_sim_time
                if self.first_sim_time is not None and self.last_sim_time is not None
                else 0.0
            )
            inferred_rate = (
                (self.inference_count - 1) / sim_duration
                if self.inference_count > 1 and sim_duration > 0
                else None
            )
            return {
                "schema": SCHEMA,
                "status": status,
                "decision": (
                    "live_physx_state_shadow_one_state_one_update_passed"
                    if status == "PASSED"
                    else "live_physx_state_shadow_not_qualified"
                ),
                "qualification": False,
                "offline_only": False,
                "read_only": True,
                "authoritative_physics_source": "geniesim_isaac_physx",
                "subscribed_topics": (
                    ["/joint_states", "/clock"]
                    if args.fixed_base
                    else ["/joint_states", "/odom", "/tf", "/clock"]
                ),
                "published_topics": [],
                "real_robot_commands": False,
                "model": {
                    "path": str(model),
                    "sha256": sha256_file(model),
                    "input_name": inputs[0].name,
                    "input_shape": inputs[0].shape,
                    "output_name": outputs[0].name,
                    "output_shape": outputs[0].shape,
                    "onnxruntime_version": ort.__version__,
                },
                "parameters": {
                    "velocity_command": list(args.velocity_command),
                    "policy_rate_hz": args.policy_rate_hz,
                    "maximum_sim_duration_s": args.maximum_sim_duration_s,
                    "maximum_wall_duration_s": args.maximum_wall_duration_s,
                    "maximum_pose_skew_s": args.maximum_pose_skew_s,
                    "minimum_inferences": args.minimum_inferences,
                    "fixed_base": args.fixed_base,
                    "cyclic_gc_disabled_during_shadow": True,
                },
                "metrics": {
                    "raw_state_count": self.raw_state_count,
                    "inference_count": self.inference_count,
                    "scheduler_skip_count": self.scheduler_skip_count,
                    "waiting_pose_count": self.waiting_pose_count,
                    "nonmonotonic_stamp_count": self.nonmonotonic_stamp_count,
                    "pose_skew_reject_count": self.pose_skew_reject_count,
                    "pose_skew_reject_ratio": (
                        self.pose_skew_reject_count / self.raw_state_count
                        if self.raw_state_count
                        else None
                    ),
                    "session_reject_count": self.session_reject_count,
                    "first_sampled_sequence": self.first_sampled_sequence,
                    "last_sampled_sequence": self.last_sampled_sequence,
                    "first_sim_time": self.first_sim_time,
                    "last_sim_time": self.last_sim_time,
                    "sampled_sim_duration_s": sim_duration,
                    "inferred_policy_rate_hz": inferred_rate,
                    "sample_gap_p95_s": percentile(self.sample_gaps, 0.95),
                    "sample_gap_max_s": max(self.sample_gaps, default=None),
                    "inference_wall_p50_ms": percentile(policy.elapsed_ms, 0.50),
                    "inference_wall_p95_ms": percentile(policy.elapsed_ms, 0.95),
                    "inference_wall_max_ms": max(policy.elapsed_ms, default=None),
                    "policy_output_sha256": policy.output_digest.hexdigest(),
                    "maximum_policy_output_abs": self.max_output_abs,
                    "status_counts": self.status_counts,
                    "timed_out": timed_out,
                },
                "checks": {
                    "minimum_inferences": self.inference_count >= args.minimum_inferences,
                    "one_policy_call_per_sample": len(policy.elapsed_ms) == self.inference_count,
                    "policy_not_faster_than_physics": self.inference_count <= self.raw_state_count,
                    "strict_sim_time": self.nonmonotonic_stamp_count == 0,
                    "pose_stream_synchronized": (
                        args.fixed_base
                        or (
                            self.raw_state_count > 0
                            and self.pose_skew_reject_count / self.raw_state_count <= 0.001
                        )
                    ),
                    "session_fail_closed_cleanly": self.session_reject_count == 0,
                    "sim_duration_reached": sim_duration >= args.maximum_sim_duration_s,
                    "policy_rate_within_five_percent": (
                        inferred_rate is not None
                        and args.policy_rate_hz * 0.95 <= inferred_rate <= args.policy_rate_hz * 1.05
                    ),
                    "not_timed_out": not timed_out,
                    "no_publishers": True,
                },
                "last_error": self.last_error,
                "limitations": [
                    "The shadow computes commands but never publishes them to PhysX.",
                    "This fixed-base gate proves live state/time integration, not standing or walking.",
                    "Fixed-base mode uses declared identity base pose and zero base velocity; floating odom/TF synchronization is a separate gate.",
                    "The command is a simulation-only deterministic fixture, not a robot command.",
                    "Python cyclic GC is disabled only during the bounded shadow window; reference counting remains active and GC is restored before exit.",
                ],
            }

    rclpy.init(args=ros_args)
    node = Shadow()
    stopped = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    started_wall = time.monotonic()
    timed_out = False
    gc_was_enabled = gc.isenabled()
    gc.collect()
    if gc_was_enabled:
        gc.disable()
    try:
        while rclpy.ok() and not stopped and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() - started_wall > args.maximum_wall_duration_s:
                timed_out = True
                break
    finally:
        if gc_was_enabled:
            gc.enable()
            gc.collect()
        draft = node.result("PENDING", timed_out)
        checks_passed = all(draft["checks"].values())
        final = node.result("PASSED" if checks_passed else "FAILED", timed_out)
        atomic_json(receipt, final)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if final["status"] == "PASSED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
