"""Engine-neutral X2 STAND_DEFAULT policy and position-PD contract."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Sequence


X2_STAND_JOINT_NAMES = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
)

X2_STAND_ACTION_JOINT_NAMES = (
    *X2_STAND_JOINT_NAMES[:12],
    "waist_pitch_joint",
    "waist_roll_joint",
)

X2_STAND_NOMINAL = (
    -0.235, 0.0, 0.0, 0.5, -0.265, 0.0,
    -0.235, 0.0, 0.0, 0.5, -0.265, 0.0,
    0.0, 0.0,
)
X2_STAND_ACTION_SCALE = (0.5,) * 14
X2_STAND_ACTION_CUTOFF_HZ = (5000.0,) * 14
X2_STAND_POLICY_DT = 0.01
X2_STAND_ACTION_LIMITS = (
    (-2.704, 2.556), (-0.235, 2.906), (-1.684, 3.43), (0.0, 2.4073),
    (-0.803, 0.453), (-0.262, 0.262),
    (-2.704, 2.556), (-2.906, 0.235), (-3.43, 1.684), (0.0, 2.4073),
    (-0.803, 0.453), (-0.2625, 0.2625),
    (-0.314, 0.314), (-0.488, 0.488),
)
X2_STAND_KP = (
    100.0, 100.0, 100.0, 150.0, 40.0, 30.0,
    100.0, 100.0, 100.0, 150.0, 40.0, 30.0,
    40.0, 40.0,
)
X2_STAND_KD = (
    4.0, 3.0, 3.0, 5.0, 3.0, 2.0,
    4.0, 3.0, 3.0, 5.0, 3.0, 2.0,
    5.0, 5.0,
)
X2_STAND_ARM_TARGET = (0.4, 0.0, 0.0, -1.2, 0.4, 0.0, 0.0, -1.2)


def _finite(values: Sequence[float], width: int, label: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != width:
        raise ValueError(f"{label} width {len(result)} != {width}")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} contains NaN or Inf")
    return result


@dataclass(frozen=True)
class X2StandPhysicsState:
    source_sequence: int
    sim_time: float
    joint_position: tuple[float, ...]
    joint_velocity: tuple[float, ...]
    base_angular_velocity: tuple[float, float, float]
    observation_gravity: tuple[float, float, float]
    velocity_command: tuple[float, float, float]
    pose_command: tuple[float, float, float, float]
    arm_target_position: tuple[float, ...] = X2_STAND_ARM_TARGET
    phase_q: tuple[float, float, float, float] = (0.0, 1.0, 0.0, 1.0)
    # The frozen vendor STAND_DEFAULT profile sets
    # ``freeze_waist_obs_to_current=true``.  Consequently the final two
    # action-history lanes are supplied by the waist-command preprocessing
    # path, rather than by the final two ONNX outputs.  Keeping those two
    # values explicit avoids silently treating policy residuals as measured
    # waist commands in another physics engine.
    waist_action_observation: tuple[float, float] = (0.0, 0.0)

    def validate(self) -> None:
        if not isinstance(self.source_sequence, int) or self.source_sequence < 0:
            raise ValueError("source_sequence must be a non-negative integer")
        if not math.isfinite(float(self.sim_time)):
            raise ValueError("sim_time contains NaN or Inf")
        _finite(self.joint_position, 23, "joint position")
        _finite(self.joint_velocity, 23, "joint velocity")
        _finite(self.base_angular_velocity, 3, "base angular velocity")
        _finite(self.observation_gravity, 3, "observation gravity")
        _finite(self.velocity_command, 3, "velocity command")
        _finite(self.pose_command, 4, "pose command")
        _finite(self.arm_target_position, 8, "arm target")
        _finite(self.phase_q, 4, "phase q")
        _finite(self.waist_action_observation, 2, "waist action observation")


@dataclass(frozen=True)
class X2StandCommand:
    joint_names: tuple[str, ...]
    position_target: tuple[float, ...]
    velocity_target: tuple[float, ...]
    feedforward_effort: tuple[float, ...]
    kp: tuple[float, ...]
    kd: tuple[float, ...]
    raw_action: tuple[float, ...]
    filtered_residual: tuple[float, ...]
    real_robot_commands: bool = False


@dataclass(frozen=True)
class X2StandSessionResult:
    source_sequence: int
    sim_time: float
    observation: tuple[float, ...]
    stacked_observation: tuple[float, ...]
    command: X2StandCommand
    real_robot_commands: bool = False


def build_stand_observation(
    state: X2StandPhysicsState,
    previous_action: Sequence[float],
) -> tuple[float, ...]:
    """Build the vendor CPGTeleCon 85-value observation in frozen order."""
    state.validate()
    values = (
        *_finite(state.base_angular_velocity, 3, "base angular velocity"),
        *_finite(state.observation_gravity, 3, "observation gravity"),
        *_finite(state.velocity_command, 3, "velocity command"),
        *_finite(state.pose_command, 4, "pose command"),
        *_finite(state.joint_position, 23, "joint position"),
        *_finite(state.joint_velocity, 23, "joint velocity"),
        *_finite(previous_action, 14, "previous action")[:12],
        *_finite(state.waist_action_observation, 2, "waist action observation"),
        *_finite(state.arm_target_position, 8, "arm target"),
        *_finite(state.phase_q, 4, "phase q"),
    )
    return _finite(values, 85, "stand observation")


def stand_action_to_command(
    action: Sequence[float],
    state: X2StandPhysicsState,
    previous_filtered_residual: Sequence[float] = (0.0,) * 14,
    *,
    policy_dt: float = X2_STAND_POLICY_DT,
) -> X2StandCommand:
    """Map 14 policy actions to the filtered 23-joint STAND_DEFAULT command.

    The released controller applies a first-order low-pass to the scaled
    residual before adding the nominal pose.  A nominal 5 kHz cutoff makes
    the effect small after the first frame, but omitting it is still enough
    to produce a measurable mismatch at controller start-up.
    """
    raw = _finite(action, 14, "stand action")
    previous = _finite(previous_filtered_residual, 14, "previous filtered residual")
    if not math.isfinite(float(policy_dt)) or policy_dt <= 0.0:
        raise ValueError("policy_dt must be finite and positive")
    scaled = tuple(scale * value for scale, value in zip(X2_STAND_ACTION_SCALE, raw))
    filtered = tuple(
        alpha * value + (1.0 - alpha) * old
        for value, old, cutoff in zip(scaled, previous, X2_STAND_ACTION_CUTOFF_HZ)
        for alpha in (policy_dt / (policy_dt + 1.0 / (2.0 * math.pi * cutoff)),)
    )
    controlled = {
        name: min(upper, max(lower, nominal + residual))
        for name, nominal, residual, (lower, upper) in zip(
            X2_STAND_ACTION_JOINT_NAMES,
            X2_STAND_NOMINAL,
            filtered,
            X2_STAND_ACTION_LIMITS,
        )
    }
    arms = dict(zip(X2_STAND_JOINT_NAMES[15:], state.arm_target_position))
    positions_list = []
    for name in X2_STAND_JOINT_NAMES:
        if name in controlled:
            positions_list.append(controlled[name])
        elif name == "waist_yaw_joint":
            positions_list.append(0.0)
        else:
            positions_list.append(arms[name])
    positions = tuple(positions_list)
    controlled_kp = dict(zip(X2_STAND_ACTION_JOINT_NAMES, X2_STAND_KP))
    controlled_kd = dict(zip(X2_STAND_ACTION_JOINT_NAMES, X2_STAND_KD))
    kp = tuple(controlled_kp.get(name, 40.0) for name in X2_STAND_JOINT_NAMES)
    kd = tuple(controlled_kd.get(name, 8.0) for name in X2_STAND_JOINT_NAMES)
    return X2StandCommand(
        joint_names=X2_STAND_JOINT_NAMES,
        position_target=_finite(positions, 23, "stand position target"),
        velocity_target=(0.0,) * 23,
        feedforward_effort=(0.0,) * 23,
        kp=kp,
        kd=kd,
        raw_action=raw,
        filtered_residual=filtered,
    )


class StateDrivenX2StandSession:
    """One policy call per increasing simulator state, using simulator time."""

    def __init__(self, policy: Callable[[tuple[float, ...]], Sequence[float]], *, frame_stack: int = 10) -> None:
        if frame_stack != 10:
            raise ValueError("the deployed STAND_DEFAULT model requires frame_stack=10")
        self.policy = policy
        self.frame_stack = frame_stack
        self.reset()

    def reset(self) -> None:
        self.last_source_sequence: int | None = None
        self.last_sim_time: float | None = None
        self.previous_action = (0.0,) * 14
        self.filtered_residual = (0.0,) * 14
        self.history: list[tuple[float, ...]] = []

    def update(self, state: X2StandPhysicsState) -> X2StandSessionResult:
        state.validate()
        if self.last_source_sequence is not None:
            if state.source_sequence <= self.last_source_sequence:
                raise ValueError("physics state sequence is duplicate or out of order")
            if state.sim_time <= float(self.last_sim_time):
                raise ValueError("simulation time did not advance")
            if state.sim_time - float(self.last_sim_time) > 0.10:
                raise ValueError("physics state gap exceeded the fail-closed limit")
        observation = build_stand_observation(state, self.previous_action)
        self.history = [observation] * 10 if not self.history else [*self.history[1:], observation]
        stacked = _finite(
            tuple(value for frame in self.history for value in frame),
            850,
            "stacked stand observation",
        )
        action = _finite(self.policy(stacked), 14, "stand policy output")
        command = stand_action_to_command(action, state, self.filtered_residual)
        self.previous_action = action
        self.filtered_residual = command.filtered_residual
        self.last_source_sequence = state.source_sequence
        self.last_sim_time = state.sim_time
        return X2StandSessionResult(
            source_sequence=state.source_sequence,
            sim_time=state.sim_time,
            observation=observation,
            stacked_observation=stacked,
            command=command,
        )
