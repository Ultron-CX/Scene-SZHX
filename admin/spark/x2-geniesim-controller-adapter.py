#!/usr/bin/env python3
"""Bridge the vendor X2 controller to Genie Sim without touching real hardware.

The adapter intentionally carries only simulator ROS topics.  Genie Sim remains
the sole physics authority.  High-rate state is split into the four lanes used
by x1_rl_control and the four vendor command lanes are merged into the standard
JointState command consumed by genie_sim_engine.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence


LEG = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
)
# Vendor lane order is authoritative and differs from the URDF/PhysX order.
WAIST = ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint")
ARM = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_yaw_joint",
    "left_wrist_pitch_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_yaw_joint",
    "right_wrist_pitch_joint", "right_wrist_roll_joint",
)
HEAD = ("head_yaw_joint", "head_pitch_joint")
GROUPS = {"leg": LEG, "waist": WAIST, "arm": ARM, "head": HEAD}
ALL_JOINTS = LEG + WAIST + ARM + HEAD
EFFORT_LIMIT = {
    name: (
        120.0 if ("hip" in name or "knee" in name or name == "waist_yaw_joint")
        else 36.0 if ("ankle_pitch" in name or "shoulder" in name)
        else 24.0 if ("ankle_roll" in name or name in {"waist_roll_joint", "waist_pitch_joint"}
                      or "elbow" in name or "wrist_yaw" in name)
        else 4.8 if "wrist" in name
        else 2.6 if name == "head_yaw_joint"
        else 0.6
    )
    for name in ALL_JOINTS
}


def finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    values = (float(x), float(y), float(z), float(w))
    if not finite(values):
        raise ValueError("quaternion contains NaN or Inf")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def normalized_quaternion_xyzw(values: Sequence[float]) -> tuple[float, float, float, float]:
    if len(values) != 4 or not finite(values):
        raise ValueError("quaternion must contain four finite values")
    norm = math.sqrt(sum(float(value) ** 2 for value in values))
    if norm <= 1e-12:
        raise ValueError("quaternion norm is zero")
    return tuple(float(value) / norm for value in values)


def quaternion_multiply_xyzw(
    left: Sequence[float], right: Sequence[float]
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = normalized_quaternion_xyzw(left)
    rx, ry, rz, rw = normalized_quaternion_xyzw(right)
    return normalized_quaternion_xyzw((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def rotate_world_to_body(
    quaternion_world_body_xyzw: Sequence[float], vector_world: Sequence[float]
) -> tuple[float, float, float]:
    """Rotate a world-frame vector into the body frame."""
    x, y, z, w = normalized_quaternion_xyzw(quaternion_world_body_xyzw)
    vx, vy, vz = (float(value) for value in vector_world)
    # R(q)^T v, expanded to avoid allocating quaternion temporaries.
    return (
        (1 - 2 * (y * y + z * z)) * vx + 2 * (x * y + w * z) * vy + 2 * (x * z - w * y) * vz,
        2 * (x * y - w * z) * vx + (1 - 2 * (x * x + z * z)) * vy + 2 * (y * z + w * x) * vz,
        2 * (x * z + w * y) * vx + 2 * (y * z - w * x) * vy + (1 - 2 * (x * x + y * y)) * vz,
    )


def percentile(values: Sequence[float], quantile: float) -> float | None:
    if not values:
        return None
    if not 0.0 <= quantile <= 1.0:
        raise ValueError("quantile must be within [0, 1]")
    ordered = sorted(float(value) for value in values)
    index = quantile * (len(ordered) - 1)
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    fraction = index - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


@dataclass
class SpecificForceEstimator:
    cutoff_hz: float = 20.0
    last_stamp_s: float | None = None
    last_velocity_world: tuple[float, float, float] | None = None
    filtered_body: tuple[float, float, float] = (0.0, 0.0, 9.81)
    sample_count: int = 0
    rejected_dt_count: int = 0

    def update(
        self,
        stamp_s: float,
        velocity_world: Sequence[float],
        quaternion_world_body_xyzw: Sequence[float],
    ) -> tuple[float, float, float]:
        velocity = tuple(float(value) for value in velocity_world)
        if not math.isfinite(stamp_s) or not finite(velocity):
            raise ValueError("specific-force input contains NaN or Inf")
        gravity_specific_world = (0.0, 0.0, 9.81)
        if self.last_stamp_s is None or self.last_velocity_world is None:
            raw_world = gravity_specific_world
            alpha = 1.0
        else:
            dt = stamp_s - self.last_stamp_s
            if not 1e-6 <= dt <= 0.1:
                self.rejected_dt_count += 1
                self.last_stamp_s = stamp_s
                self.last_velocity_world = velocity
                return self.filtered_body
            raw_world = tuple(
                (velocity[index] - self.last_velocity_world[index]) / dt + gravity_specific_world[index]
                for index in range(3)
            )
            alpha = 1.0 - math.exp(-2.0 * math.pi * self.cutoff_hz * dt)
        raw_body = rotate_world_to_body(quaternion_world_body_xyzw, raw_world)
        self.filtered_body = tuple(
            previous + alpha * (current - previous)
            for previous, current in zip(self.filtered_body, raw_body)
        )
        self.last_stamp_s = stamp_s
        self.last_velocity_world = velocity
        self.sample_count += 1
        return self.filtered_body


def validate_vector(names: Sequence[str], values: Sequence[float], expected: Sequence[str]) -> None:
    if tuple(names) != tuple(expected):
        raise ValueError(f"joint names/order mismatch: got={list(names)!r}, expected={list(expected)!r}")
    if len(values) != len(expected):
        raise ValueError(f"joint vector length {len(values)} != {len(expected)}")
    if not finite(values):
        raise ValueError("joint vector contains NaN or Inf")


def atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        os.chmod(path, 0o644)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


@dataclass
class LaneCommand:
    names: List[str]
    position: List[float]
    velocity: List[float]
    effort: List[float]
    stiffness: List[float]
    damping: List[float]
    received_monotonic: float
    source_stamp_ns: int
    generation: int


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--receipt", required=True)
    parser.add_argument("--command-timeout-s", type=float, default=0.10)
    parser.add_argument("--publish-rate-hz", type=float, default=500.0)
    parser.add_argument("--publish-mode", choices=("timer", "state_driven"), default="timer")
    parser.add_argument("--timebase", choices=("wall", "sim"), default="wall")
    parser.add_argument("--require-command-stamp", action="store_true")
    parser.add_argument("--maximum-command-age-s", type=float, default=0.004)
    parser.add_argument("--minimum-state-messages", type=int, default=100)
    parser.add_argument("--minimum-command-sets", type=int, default=10)
    parser.add_argument("--command-mode", choices=("position", "effort_pd"), default="position")
    parser.add_argument("--effort-gain-scale", type=float, default=1.0)
    args, ros_args = parser.parse_known_args()
    if ros_args and ros_args[0] != "--ros-args":
        parser.error(f"unrecognized arguments: {' '.join(ros_args)}")

    if args.command_timeout_s <= 0 or args.publish_rate_hz <= 0 or args.maximum_command_age_s <= 0:
        parser.error("timeouts and rates must be positive")

    import rclpy
    from geometry_msgs.msg import Quaternion, Twist, Vector3
    from my_ros2_proto.msg import JointCommand
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu, JointState
    from tf2_msgs.msg import TFMessage

    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )

    class Adapter(Node):
        def __init__(self) -> None:
            super().__init__("x2_geniesim_controller_adapter")
            self.started_wall = time.time()
            self.state_by_name: Dict[str, tuple[float, float, float]] = {}
            self.initial_positions: Dict[str, float] = {}
            self.max_state_excursion = 0.0
            self.lanes: Dict[str, LaneCommand] = {}
            self.state_messages = 0
            self.state_sequence = 0
            self.current_state_stamp_ns = 0
            self.current_state_received_monotonic = 0.0
            self.last_published_state_sequence = 0
            self.skipped_state_count = 0
            self.timer_callbacks_without_new_state = 0
            self.command_age_sim_s: List[float] = []
            self.command_response_wall_s: List[float] = []
            self.command_set_stamp_span_s: List[float] = []
            self.command_stamp_zero_count = 0
            self.command_stamp_before_state_count = 0
            self.command_stamp_future_count = 0
            self.command_age_above_limit_count = 0
            self.lane_generations = {name: 0 for name in GROUPS}
            self.last_published_lane_generations = {name: 0 for name in GROUPS}
            self.odom_messages = 0
            self.initial_base_position = None
            self.final_base_position = None
            self.min_base_z = math.inf
            self.max_base_tilt_rad = 0.0
            self.max_base_linear_speed = 0.0
            self.max_base_angular_speed = 0.0
            self.last_planar_position = [0.0, 0.0]
            self.last_planar_yaw_rad = 0.0
            self.initial_planar_yaw_rad = None
            self.final_planar_yaw_rad = None
            self.trajectory = []
            self.last_trajectory_stamp_s = None
            self.planar_path_length_m = 0.0
            self.previous_trajectory_position = None
            self.last_base_linear = Vector3()
            self.last_base_angular = Vector3()
            self.latest_base_orientation = None
            self.world_footprint_orientation_xyzw = (0.0, 0.0, 0.0, 1.0)
            self.footprint_base_orientation_xyzw = (0.0, 0.0, 0.0, 1.0)
            self.latest_specific_force = Vector3(x=0.0, y=0.0, z=9.81)
            self.specific_force_estimator = SpecificForceEstimator()
            self.imu_messages_published = 0
            self.command_lane_messages = {name: 0 for name in GROUPS}
            self.command_sets = 0
            self.command_timeouts = 0
            self.rejected_messages = 0
            self.last_error = None
            self.last_publish_monotonic = 0.0
            self.max_command_abs = 0.0
            self.max_observed_stiffness = 0.0
            self.max_observed_damping = 0.0
            self.effort_sample_count = 0
            self.effort_saturation_count = 0
            self.max_raw_effort_ratio = 0.0
            self.forward_dynamic_drive_gains = False
            self.split_publishers = {
                group: self.create_publisher(JointState, f"/joint_states/{group}", qos)
                for group in GROUPS
            }
            self.command_publisher = self.create_publisher(JointState, "/joint_command", qos)
            self.imu_torso = self.create_publisher(Imu, "/imu/torso/data", qos)
            self.imu_chest = self.create_publisher(Imu, "/imu/chest/data", qos)
            self.create_subscription(JointState, "/joint_states", self.on_joint_state, qos)
            # /odom is a ground-projected base_footprint.  Authoritative
            # floating height/tilt is the base_footprint -> base_link TF edge.
            self.create_subscription(Odometry, "/odom", self.on_odom, qos)
            self.create_subscription(TFMessage, "/tf", self.on_tf, qos)
            self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, qos)
            for group, expected in GROUPS.items():
                self.create_subscription(
                    JointCommand,
                    f"/joint_cmd/{group}",
                    lambda msg, g=group, e=expected: self.on_command(g, e, msg),
                    qos,
                )
            self.timer = (
                self.create_timer(1.0 / args.publish_rate_hz, self.publish_merged_command)
                if args.publish_mode == "timer" else None
            )

        def on_cmd_vel(self, msg: Twist) -> None:
            if any(abs(float(value)) > 1e-6 for value in (
                msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z,
            )):
                self.forward_dynamic_drive_gains = True

        def reject(self, message: str) -> None:
            self.rejected_messages += 1
            self.last_error = message
            if self.rejected_messages <= 10:
                self.get_logger().error(message)

        def on_joint_state(self, msg: JointState) -> None:
            try:
                if len(msg.name) != len(msg.position):
                    raise ValueError("aggregate JointState name/position length mismatch")
                velocities = list(msg.velocity) if len(msg.velocity) == len(msg.name) else [0.0] * len(msg.name)
                efforts = list(msg.effort) if len(msg.effort) == len(msg.name) else [0.0] * len(msg.name)
                if not finite(msg.position) or not finite(velocities) or not finite(efforts):
                    raise ValueError("aggregate JointState contains NaN or Inf")
                lookup = {
                    name: (float(position), float(velocity), float(effort))
                    for name, position, velocity, effort in zip(msg.name, msg.position, velocities, efforts)
                }
                missing = sorted(set(ALL_JOINTS) - set(lookup))
                if missing:
                    raise ValueError(f"aggregate JointState missing X2 joints: {missing}")
                self.state_by_name = lookup
                self.state_sequence += 1
                self.current_state_stamp_ns = stamp_to_ns(msg.header.stamp)
                self.current_state_received_monotonic = time.monotonic()
                if not self.initial_positions:
                    self.initial_positions = {name: lookup[name][0] for name in ALL_JOINTS}
                else:
                    self.max_state_excursion = max(
                        self.max_state_excursion,
                        *(abs(lookup[name][0] - self.initial_positions[name]) for name in ALL_JOINTS),
                    )
                self.state_messages += 1
                # Publish the matching-stamp IMU first so the controller's
                # split joint callbacks can find it immediately instead of
                # pairing the new joints with the previous IMU sample.
                if self.latest_base_orientation is not None:
                    self.publish_imu(msg.header)
                for group, expected in GROUPS.items():
                    outgoing = JointState()
                    outgoing.header = msg.header
                    outgoing.name = list(expected)
                    outgoing.position = [lookup[name][0] for name in expected]
                    outgoing.velocity = [lookup[name][1] for name in expected]
                    outgoing.effort = [lookup[name][2] for name in expected]
                    self.split_publishers[group].publish(outgoing)
            except Exception as exc:
                self.reject(f"rejected /joint_states: {exc}")

        def publish_imu(self, header) -> None:
            outgoing = Imu()
            outgoing.header = header
            outgoing.header.frame_id = "torso_imu"
            outgoing.orientation = self.latest_base_orientation
            outgoing.angular_velocity = self.last_base_angular
            outgoing.linear_acceleration = self.latest_specific_force
            outgoing.orientation_covariance[0] = -1.0
            outgoing.angular_velocity_covariance[0] = -1.0
            self.imu_torso.publish(outgoing)
            chest = Imu()
            chest.header = outgoing.header
            chest.header.frame_id = "chest_imu"
            chest.orientation = outgoing.orientation
            chest.angular_velocity = outgoing.angular_velocity
            chest.linear_acceleration = outgoing.linear_acceleration
            chest.orientation_covariance = outgoing.orientation_covariance
            chest.angular_velocity_covariance = outgoing.angular_velocity_covariance
            chest.linear_acceleration_covariance = outgoing.linear_acceleration_covariance
            self.imu_chest.publish(chest)
            self.imu_messages_published += 1

        def on_odom(self, msg: Odometry) -> None:
            self.odom_messages += 1
            position = msg.pose.pose.position
            self.last_planar_position = [float(position.x), float(position.y)]
            orientation = msg.pose.pose.orientation
            self.world_footprint_orientation_xyzw = normalized_quaternion_xyzw((
                orientation.x, orientation.y, orientation.z, orientation.w,
            ))
            world_base = quaternion_multiply_xyzw(
                self.world_footprint_orientation_xyzw,
                self.footprint_base_orientation_xyzw,
            )
            self.latest_base_orientation = Quaternion(
                x=world_base[0], y=world_base[1], z=world_base[2], w=world_base[3]
            )
            yaw = quaternion_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
            self.last_planar_yaw_rad = yaw
            if self.initial_planar_yaw_rad is None:
                self.initial_planar_yaw_rad = yaw
            self.final_planar_yaw_rad = yaw
            self.last_base_linear = msg.twist.twist.linear
            self.last_base_angular = msg.twist.twist.angular
            linear = msg.twist.twist.linear
            angular = msg.twist.twist.angular
            stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            specific_force = self.specific_force_estimator.update(
                stamp_s,
                (linear.x, linear.y, linear.z),
                world_base,
            )
            self.latest_specific_force = Vector3(
                x=specific_force[0], y=specific_force[1], z=specific_force[2]
            )
            if self.last_trajectory_stamp_s is None or stamp_s - self.last_trajectory_stamp_s >= 0.05:
                sample_position = [float(position.x), float(position.y)]
                if self.previous_trajectory_position is not None:
                    self.planar_path_length_m += math.hypot(
                        sample_position[0] - self.previous_trajectory_position[0],
                        sample_position[1] - self.previous_trajectory_position[1],
                    )
                self.previous_trajectory_position = sample_position
                self.last_trajectory_stamp_s = stamp_s
                self.trajectory.append({
                    "stamp_s": stamp_s,
                    "x": sample_position[0],
                    "y": sample_position[1],
                    "vx": float(linear.x),
                    "vy": float(linear.y),
                    "wz": float(angular.z),
                    "yaw_rad": yaw,
                })
            self.max_base_linear_speed = max(
                self.max_base_linear_speed,
                math.sqrt(float(linear.x) ** 2 + float(linear.y) ** 2 + float(linear.z) ** 2),
            )
            self.max_base_angular_speed = max(
                self.max_base_angular_speed,
                math.sqrt(float(angular.x) ** 2 + float(angular.y) ** 2 + float(angular.z) ** 2),
            )

        def on_tf(self, msg: TFMessage) -> None:
            residual = next(
                (
                    transform for transform in msg.transforms
                    if transform.header.frame_id.lstrip("/") == "base_footprint"
                    and transform.child_frame_id.lstrip("/") == "base_link"
                ),
                None,
            )
            if residual is None:
                return
            position = residual.transform.translation
            orientation = residual.transform.rotation
            self.footprint_base_orientation_xyzw = normalized_quaternion_xyzw((
                orientation.x, orientation.y, orientation.z, orientation.w,
            ))
            world_base = quaternion_multiply_xyzw(
                self.world_footprint_orientation_xyzw,
                self.footprint_base_orientation_xyzw,
            )
            self.latest_base_orientation = Quaternion(
                x=world_base[0], y=world_base[1], z=world_base[2], w=world_base[3]
            )
            xyz = [self.last_planar_position[0], self.last_planar_position[1], float(position.z)]
            if self.initial_base_position is None:
                self.initial_base_position = xyz
            self.final_base_position = xyz
            self.min_base_z = min(self.min_base_z, xyz[2])
            rzz = 1.0 - 2.0 * (float(orientation.x) ** 2 + float(orientation.y) ** 2)
            self.max_base_tilt_rad = max(
                self.max_base_tilt_rad, math.acos(max(-1.0, min(1.0, rzz)))
            )

        def on_command(self, group: str, expected: Sequence[str], msg: JointCommand) -> None:
            try:
                validate_vector(msg.name, msg.position, expected)
                for label, values in (
                    ("velocity", msg.velocity), ("effort", msg.effort),
                    ("stiffness", msg.stiffness), ("damping", msg.damping),
                ):
                    if values and (len(values) != len(expected) or not finite(values)):
                        raise ValueError(f"{label} vector invalid")
                lane = LaneCommand(
                    names=list(msg.name),
                    position=[float(v) for v in msg.position],
                    velocity=[float(v) for v in msg.velocity] if msg.velocity else [0.0] * len(expected),
                    effort=[float(v) for v in msg.effort] if msg.effort else [0.0] * len(expected),
                    stiffness=[float(v) for v in msg.stiffness] if msg.stiffness else [0.0] * len(expected),
                    damping=[float(v) for v in msg.damping] if msg.damping else [0.0] * len(expected),
                    received_monotonic=time.monotonic(),
                    source_stamp_ns=stamp_to_ns(msg.header.stamp),
                    generation=self.lane_generations[group] + 1,
                )
                self.lanes[group] = lane
                self.lane_generations[group] = lane.generation
                self.command_lane_messages[group] += 1
                self.max_command_abs = max(self.max_command_abs, *(abs(v) for v in lane.position))
                self.max_observed_stiffness = max(self.max_observed_stiffness, *(abs(v) for v in lane.stiffness))
                self.max_observed_damping = max(self.max_observed_damping, *(abs(v) for v in lane.damping))
                if args.publish_mode == "state_driven":
                    # The fourth fresh lane closes one controller response for
                    # the current PhysX state.  Publishing here avoids a ROS
                    # timer racing the /clock callback ahead of joint state.
                    self.publish_merged_command()
            except Exception as exc:
                self.reject(f"rejected /joint_cmd/{group}: {exc}")

        def publish_merged_command(self) -> None:
            now = time.monotonic()
            if not self.state_by_name:
                return
            if args.publish_mode == "state_driven":
                if self.state_sequence <= self.last_published_state_sequence:
                    self.timer_callbacks_without_new_state += 1
                    return
                if any(
                    self.lane_generations[group] <= self.last_published_lane_generations[group]
                    for group in GROUPS
                ):
                    return
            source_stamps = [self.lanes[group].source_stamp_ns for group in GROUPS if group in self.lanes]
            if args.require_command_stamp and len(source_stamps) == len(GROUPS) and any(stamp <= 0 for stamp in source_stamps):
                self.command_stamp_zero_count += 1
                return
            if args.timebase == "sim" and len(source_stamps) == len(GROUPS) and all(stamp > 0 for stamp in source_stamps):
                now_ns = int(self.get_clock().now().nanoseconds)
                fresh = all(0 <= now_ns - stamp <= int(args.command_timeout_s * 1e9) for stamp in source_stamps)
            else:
                fresh = all(
                    group in self.lanes and now - self.lanes[group].received_monotonic <= args.command_timeout_s
                    for group in GROUPS
                )
            if fresh and args.publish_mode == "state_driven" and self.current_state_stamp_ns > 0 and source_stamps:
                command_age = (self.current_state_stamp_ns - max(source_stamps)) * 1e-9
                if command_age < 0.0:
                    self.command_stamp_future_count += 1
                    return
                if command_age > args.maximum_command_age_s:
                    self.command_age_above_limit_count += 1
                    return
                if command_age > 0.0:
                    self.command_stamp_before_state_count += 1
            outgoing = JointState()
            outgoing.header.stamp = self.get_clock().now().to_msg()
            outgoing.name = list(ALL_JOINTS)
            if fresh:
                by_name = {
                    name: (position, velocity, effort, stiffness, damping)
                    for lane in self.lanes.values()
                    for name, position, velocity, effort, stiffness, damping in zip(
                        lane.names, lane.position, lane.velocity, lane.effort,
                        lane.stiffness, lane.damping,
                    )
                }
                if args.command_mode == "effort_pd":
                    outgoing.position = []
                    outgoing.velocity = []
                    torques = []
                    for name in ALL_JOINTS:
                        target, target_velocity, feedforward, kp, kd = by_name[name]
                        measured_position, measured_velocity, _ = self.state_by_name[name]
                        torque = args.effort_gain_scale * (
                            feedforward
                            + kp * (target - measured_position)
                            + kd * (target_velocity - measured_velocity)
                        )
                        limit = EFFORT_LIMIT[name]
                        self.effort_sample_count += 1
                        self.max_raw_effort_ratio = max(self.max_raw_effort_ratio, abs(torque) / limit)
                        if abs(torque) >= limit:
                            self.effort_saturation_count += 1
                        torques.append(max(-limit, min(limit, torque)))
                    outgoing.effort = torques
                else:
                    outgoing.position = [by_name[name][0] for name in ALL_JOINTS]
                    outgoing.velocity = [by_name[name][1] for name in ALL_JOINTS]
                    outgoing.effort = [0.0] * len(ALL_JOINTS)
                    if self.forward_dynamic_drive_gains:
                        # Backend-private metadata rides in the same atomic
                        # JointState packet. Genie strips these pseudo names
                        # before applying articulation targets.
                        outgoing.name.extend(f"__roboease_gain__{name}" for name in ALL_JOINTS)
                        outgoing.position.extend(by_name[name][3] for name in ALL_JOINTS)
                        outgoing.velocity.extend(0.0 for _ in ALL_JOINTS)
                        outgoing.effort.extend(by_name[name][4] for name in ALL_JOINTS)
                self.command_sets += 1
                if self.last_published_state_sequence > 0:
                    self.skipped_state_count += max(
                        0, self.state_sequence - self.last_published_state_sequence - 1
                    )
                self.last_published_state_sequence = self.state_sequence
                self.last_published_lane_generations = dict(self.lane_generations)
                if source_stamps and self.current_state_stamp_ns > 0:
                    # Age is measured from the newest source-state stamp to the
                    # PhysX state at which the command is applied.  One-cycle
                    # latency is permitted; future or >4 ms commands fail.
                    command_age = (self.current_state_stamp_ns - max(source_stamps)) * 1e-9
                    self.command_age_sim_s.append(command_age)
                    self.command_response_wall_s.append(
                        max(0.0, time.monotonic() - self.current_state_received_monotonic)
                    )
                    self.command_set_stamp_span_s.append((max(source_stamps) - min(source_stamps)) * 1e-9)
            else:
                outgoing.position = [] if args.command_mode == "effort_pd" else [
                    self.state_by_name[name][0] for name in ALL_JOINTS
                ]
                outgoing.velocity = [] if args.command_mode == "effort_pd" else [0.0] * len(ALL_JOINTS)
                outgoing.effort = [0.0] * len(ALL_JOINTS)
                if now - self.last_publish_monotonic > args.command_timeout_s:
                    self.command_timeouts += 1
            self.command_publisher.publish(outgoing)
            self.last_publish_monotonic = now

        def receipt(self, status: str, synchronized_gate: bool) -> dict:
            return {
                "schema": "roboease-x2-geniesim-controller-adapter/v1",
                "status": status,
                "started_at_unix": self.started_wall,
                "finished_at_unix": time.time(),
                "physics_authority": "genie_sim_isaac_physx",
                "real_robot_commands": False,
                "joint_count": len(ALL_JOINTS),
                "joint_order": list(ALL_JOINTS),
                "state_messages": self.state_messages,
                "state_sequence": self.state_sequence,
                "last_published_state_sequence": self.last_published_state_sequence,
                "skipped_state_count": self.skipped_state_count,
                "publish_mode": args.publish_mode,
                "timebase": args.timebase,
                "use_sim_time": bool(self.get_parameter("use_sim_time").value),
                "timer_callbacks_without_new_state": self.timer_callbacks_without_new_state,
                "state_to_command_set_ratio": self.command_sets / self.state_messages if self.state_messages else None,
                "command_age_sim_s_p95": percentile(self.command_age_sim_s, 0.95),
                "command_age_sim_s_max": max(self.command_age_sim_s, default=None),
                "maximum_command_age_s": args.maximum_command_age_s,
                "synchronized_control_gate": synchronized_gate,
                "command_response_wall_s_p95": percentile(self.command_response_wall_s, 0.95),
                "command_response_wall_s_max": max(self.command_response_wall_s, default=None),
                "command_set_stamp_span_s_p95": percentile(self.command_set_stamp_span_s, 0.95),
                "command_stamp_zero_count": self.command_stamp_zero_count,
                "command_stamp_before_state_count": self.command_stamp_before_state_count,
                "command_stamp_future_count": self.command_stamp_future_count,
                "command_age_above_limit_count": self.command_age_above_limit_count,
                "odom_messages": self.odom_messages,
                "imu_messages_published": self.imu_messages_published,
                "imu_timestamp_mapping": "latest PhysX orientation held and stamped on authoritative joint-state tick",
                "initial_base_position": self.initial_base_position,
                "final_base_position": self.final_base_position,
                "planar_path_length_m": self.planar_path_length_m,
                "trajectory": self.trajectory,
                "initial_planar_yaw_rad": self.initial_planar_yaw_rad,
                "final_planar_yaw_rad": self.final_planar_yaw_rad,
                "min_base_z": None if math.isinf(self.min_base_z) else self.min_base_z,
                "max_base_tilt_rad": self.max_base_tilt_rad,
                "max_base_linear_speed": self.max_base_linear_speed,
                "max_base_angular_speed": self.max_base_angular_speed,
                "command_lane_messages": self.command_lane_messages,
                "complete_command_sets_published": self.command_sets,
                "command_timeouts": self.command_timeouts,
                "rejected_messages": self.rejected_messages,
                "last_error": self.last_error,
                "max_abs_position_target": self.max_command_abs,
                "max_observed_physx_joint_excursion": self.max_state_excursion,
                "final_physx_joint_position": {
                    name: self.state_by_name[name][0] for name in ALL_JOINTS
                } if self.state_by_name else {},
                "final_vendor_position_target": {
                    name: lane.position[index]
                    for group, lane in self.lanes.items()
                    for index, name in enumerate(GROUPS[group])
                },
                "max_observed_vendor_stiffness": self.max_observed_stiffness,
                "max_observed_vendor_damping": self.max_observed_damping,
                "effort_sample_count": self.effort_sample_count,
                "effort_saturation_count": self.effort_saturation_count,
                "effort_saturation_ratio": (
                    self.effort_saturation_count / self.effort_sample_count
                    if self.effort_sample_count else None
                ),
                "max_raw_effort_ratio": self.max_raw_effort_ratio,
                "command_mapping": "vendor dynamic PD + feedforward -> clamped PhysX effort" if args.command_mode == "effort_pd" else "vendor position target -> sensor_msgs/JointState.position",
                "effort_mapping": "authoritative effort command" if args.command_mode == "effort_pd" else "not_applied_by_current_genie_sim_engine",
                "gain_mapping": "vendor per-cycle stiffness/damping evaluated in adapter" if args.command_mode == "effort_pd" else "frozen_physx_drive_gains; vendor per-cycle stiffness/damping observed only",
                "effort_gain_scale": args.effort_gain_scale,
                "dynamic_drive_gains_forwarded": self.forward_dynamic_drive_gains,
                "imu_mapping": "PhysX odometry velocity finite-difference plus gravity, 20Hz low-pass, rotated world-to-base",
                "imu_specific_force_samples": self.specific_force_estimator.sample_count,
                "imu_rejected_dt_count": self.specific_force_estimator.rejected_dt_count,
            }

    rclpy.init(args=ros_args)
    node = Adapter()
    stopped = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    try:
        while rclpy.ok() and not stopped:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        command_age_p95 = percentile(node.command_age_sim_s, 0.95)
        synchronized_gate = (
            args.publish_mode != "state_driven"
            or (
                args.timebase == "sim"
                and bool(node.get_parameter("use_sim_time").value)
                and node.command_stamp_zero_count == 0
                and node.command_stamp_future_count == 0
                and node.command_age_above_limit_count == 0
                and node.skipped_state_count == 0
                and node.command_sets <= node.state_messages
                and command_age_p95 is not None
                and 0.0 <= command_age_p95 <= args.maximum_command_age_s
            )
        )
        succeeded = (
            node.state_messages >= args.minimum_state_messages
            and node.command_sets >= args.minimum_command_sets
            and node.rejected_messages == 0
            and synchronized_gate
        )
        atomic_json(
            Path(args.receipt),
            node.receipt("PASSED" if succeeded else "FAILED", synchronized_gate),
        )
        node.destroy_node()
        rclpy.shutdown()
    return 0 if succeeded else 2


if __name__ == "__main__":
    raise SystemExit(main())
