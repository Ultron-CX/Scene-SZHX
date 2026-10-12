"""State-driven, engine-neutral X2 CPGWalk control session.

The session owns policy history, oscillator, gait selection, target filters and
safety state.  It advances exactly once for each strictly increasing physics
state sequence and uses simulation time only.  This module contains no ROS,
Genie Sim, Isaac, MuJoCo or physical-robot transport.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from typing import Callable, Sequence

from .x2_control import (
    AdaptiveHopfParams,
    CpgWalkActionOutput,
    CpgWalkActionParams,
    CpgWalkActionPipeline,
    CpgWalkGaitSelector,
    CpgWalkGaitSwitchParams,
    CpgWalkSafetyDecision,
    CpgWalkSafetyGate,
    CpgWalkSafetyMode,
    CpgWalkSafetyParams,
    HopfMode,
    TwoLegAdaptiveHopf,
)


Policy = Callable[[tuple[float, ...]], Sequence[float]]


def _finite(values: Sequence[float], width: int, label: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != width:
        raise ValueError(f"{label} width {len(result)} != {width}")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} contains NaN or Inf")
    return result


@dataclass(frozen=True)
class X2PhysicsState:
    source_sequence: int
    sim_time: float
    command_stamp_sim_time: float
    velocity_command: tuple[float, float, float]
    joint_position: tuple[float, ...]
    joint_velocity: tuple[float, ...]
    base_angular_velocity: tuple[float, float, float]
    # Historical compatibility name.  For CPGWalk this is the deployed
    # roll/pitch/zero-yaw attitude triplet, not projected gravity.
    observation_gravity: tuple[float, float, float]
    base_linear_velocity: tuple[float, float, float]
    roll: float
    pitch: float

    def validate(self) -> None:
        if not isinstance(self.source_sequence, int) or self.source_sequence < 0:
            raise ValueError("source_sequence must be a non-negative integer")
        scalar = (
            self.sim_time,
            self.command_stamp_sim_time,
            self.roll,
            self.pitch,
        )
        if not all(math.isfinite(float(value)) for value in scalar):
            raise ValueError("state scalar contains NaN or Inf")
        _finite(self.velocity_command, 3, "velocity command")
        _finite(self.joint_position, 17, "joint position")
        _finite(self.joint_velocity, 17, "joint velocity")
        _finite(self.base_angular_velocity, 3, "base angular velocity")
        _finite(self.observation_gravity, 3, "observation gravity")
        _finite(self.base_linear_velocity, 3, "base linear velocity")


@dataclass(frozen=True)
class X2SessionParams:
    policy_dt: float = 0.02
    frame_stack: int = 10
    maximum_state_gap_seconds: float = 0.10
    maximum_command_age_seconds: float = 0.10
    linear_velocity_scale: float = 2.0
    angular_velocity_scale: float = 1.0
    joint_position_scale: float = 1.0
    joint_velocity_scale: float = 0.05
    use_motion_flag: bool = False

    def validate(self) -> None:
        numeric = (
            self.policy_dt,
            self.maximum_state_gap_seconds,
            self.maximum_command_age_seconds,
            self.linear_velocity_scale,
            self.angular_velocity_scale,
            self.joint_position_scale,
            self.joint_velocity_scale,
        )
        if not all(math.isfinite(float(value)) for value in numeric):
            raise ValueError("session parameters contain NaN or Inf")
        if self.policy_dt <= 0 or self.frame_stack <= 0:
            raise ValueError("policy_dt and frame_stack must be positive")
        if self.maximum_state_gap_seconds < self.policy_dt:
            raise ValueError("maximum state gap cannot be shorter than policy_dt")
        if self.maximum_command_age_seconds < 0:
            raise ValueError("maximum command age cannot be negative")


@dataclass(frozen=True)
class X2SessionResult:
    status: str
    source_sequence: int
    sim_time: float
    gait: int
    command_was_stale: bool
    observation: tuple[float, ...]
    stacked_observation: tuple[float, ...]
    policy_action: tuple[float, ...]
    safety: CpgWalkSafetyDecision
    policy_joint_candidate: CpgWalkActionOutput
    joint_command: CpgWalkActionOutput | None
    real_robot_commands: bool = False


class StateDrivenX2ControllerSession:
    """Exactly-one-update-per-state X2 policy session without transport."""

    def __init__(
        self,
        policy: Policy,
        *,
        session_params: X2SessionParams = X2SessionParams(),
        action_params: CpgWalkActionParams = CpgWalkActionParams(),
        safety_params: CpgWalkSafetyParams = CpgWalkSafetyParams(),
        gait_params: CpgWalkGaitSwitchParams = CpgWalkGaitSwitchParams(),
        hopf_params: AdaptiveHopfParams = AdaptiveHopfParams(
            x2_style_adapter=True,
            gait_blend_tau=0.08,
            reset_on_walk_entry=False,
        ),
    ) -> None:
        session_params.validate()
        self.policy = policy
        self.session_params = session_params
        self.action_pipeline = CpgWalkActionPipeline(action_params)
        self.safety_gate = CpgWalkSafetyGate(safety_params)
        self.gait_selector = CpgWalkGaitSelector(gait_params)
        self.hopf = TwoLegAdaptiveHopf(hopf_params)
        self.reset()

    def reset(self) -> None:
        self.action_pipeline.reset()
        self.safety_gate.reset()
        self.gait_selector.reset()
        self.hopf.reset()
        self.last_source_sequence: int | None = None
        self.last_sim_time: float | None = None
        self.previous_action = (0.0,) * 17
        self.history: list[tuple[float, ...]] = []

    def state_digest(self) -> str:
        """Hash every mutable controller state for wall-clock freeze probes."""

        action = self.action_pipeline.snapshot()
        hopf = self.hopf.snapshot()
        payload = {
            "last_source_sequence": self.last_source_sequence,
            "last_sim_time": self.last_sim_time,
            "previous_action": self.previous_action,
            "history": self.history,
            "action": {
                "residual": action.residual,
                "filtered_position_target": action.filtered_position_target,
            },
            "gait_selector": self.gait_selector.gait,
            "safety": {
                "mode": int(self.safety_gate.mode),
                "stand_elapsed_seconds": self.safety_gate.stand_elapsed_seconds,
                "stand_transition_requested": self.safety_gate.stand_transition_requested,
            },
            "hopf": {
                "mode": int(hopf.mode),
                "gait": hopf.gait,
                "walk_initialized": hopf.walk_initialized,
                "period": hopf.period,
                "obs4": hopf.obs4,
                "filtered_velocity": hopf.filtered_velocity,
                "filtered_command": hopf.filtered_command,
            },
        }
        encoded = json.dumps(
            payload, sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()

    def _observation(
        self,
        state: X2PhysicsState,
        command: tuple[float, float, float],
        phase: tuple[float, float, float, float],
        active: bool,
    ) -> tuple[float, ...]:
        params = self.session_params
        values = [
            *(value * params.angular_velocity_scale for value in state.base_angular_velocity),
            *state.observation_gravity,
            command[0] * params.linear_velocity_scale,
            command[1] * params.linear_velocity_scale,
            command[2] * params.angular_velocity_scale,
            float(active) if params.use_motion_flag else 0.0,
        ]
        values.extend(
            (position - nominal) * params.joint_position_scale
            for position, nominal in zip(
                state.joint_position, self.action_pipeline.params.nominal_position
            )
        )
        values.extend(
            velocity * params.joint_velocity_scale for velocity in state.joint_velocity
        )
        values.extend(self.previous_action)
        values.extend(phase)
        return _finite(values, 65, "policy observation")

    def update(self, state: X2PhysicsState) -> X2SessionResult:
        state.validate()
        if self.last_source_sequence is not None:
            if state.source_sequence <= self.last_source_sequence:
                raise ValueError("physics state sequence is duplicate or out of order")
            if state.sim_time <= float(self.last_sim_time):
                raise ValueError("simulation time did not advance")
            dt = state.sim_time - float(self.last_sim_time)
            if dt > self.session_params.maximum_state_gap_seconds:
                raise ValueError("physics state gap exceeded the fail-closed limit")
        else:
            dt = self.session_params.policy_dt
        if state.command_stamp_sim_time > state.sim_time + 1e-12:
            raise ValueError("velocity command timestamp is in the future")
        command_age = state.sim_time - state.command_stamp_sim_time
        command_was_stale = command_age > self.session_params.maximum_command_age_seconds
        command = (0.0, 0.0, 0.0) if command_was_stale else _finite(
            state.velocity_command, 3, "velocity command"
        )

        gait = self.gait_selector.update(command[0], command[2])
        mean_leg_velocity = sum(abs(value) for value in state.joint_velocity[:12]) / 12.0
        angular_norm = math.sqrt(sum(value * value for value in state.base_angular_velocity))
        safety = self.safety_gate.update(
            dt=dt,
            command_xyz=command,
            roll=state.roll,
            pitch=state.pitch,
            mean_leg_velocity=mean_leg_velocity,
            base_angular_velocity_norm=angular_norm,
            gait=gait,
        )

        active = safety.mode == CpgWalkSafetyMode.ACTIVE
        self.hopf.set_gait(gait)
        self.hopf.set_mode(HopfMode.WALK if active else HopfMode.STAND)
        phase = self.hopf.obs4()
        observation = self._observation(state, command, phase, active)
        if not self.history:
            self.history = [observation] * self.session_params.frame_stack
        else:
            self.history = [*self.history[1:], observation]
        stacked = tuple(value for frame in self.history for value in frame)
        action = _finite(self.policy(stacked), 17, "policy output")
        command_output = self.action_pipeline.step_active(action, dt)
        self.previous_action = action

        self.hopf.update_base_velocity(dt, *state.base_linear_velocity)
        self.hopf.update_command_inputs(dt, command[1], command[2])
        if active:
            self.hopf.step(dt)

        self.last_source_sequence = state.source_sequence
        self.last_sim_time = state.sim_time
        status = (
            "STAND_HANDOFF_REQUEST"
            if safety.request_stand_transition
            else "ACTIVE_COMMAND"
            if active
            else "SAFE_HOLD"
        )
        return X2SessionResult(
            status=status,
            source_sequence=state.source_sequence,
            sim_time=state.sim_time,
            gait=gait,
            command_was_stale=command_was_stale,
            observation=observation,
            stacked_observation=stacked,
            policy_action=action,
            safety=safety,
            policy_joint_candidate=command_output,
            joint_command=command_output if active else None,
        )
