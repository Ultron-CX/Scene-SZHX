"""Engine-neutral X2 control primitives reconstructed from the release ABI.

The vendor release exposes the complete ``TwoLegAdaptiveHopf`` class contract
and ships an unstripped ARM64 implementation, but not its buildable control
source.  This module provides a small, deterministic implementation for the
state-driven simulator adapter.  It has no ROS or physics-engine dependency.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum
from typing import Mapping, Sequence


class HopfMode(IntEnum):
    STAND = 0
    WALK = 1


def x2_cpgwalk_attitude_observation(
    roll: float,
    pitch: float,
    *,
    imu_roll_offset: float = 0.0,
    imu_pitch_offset: float = -0.015,
) -> tuple[float, float, float]:
    """Return the release policy's attitude triplet.

    Despite the historical ``pelvis_gvec``/``obs_gvec`` debug names, the
    deployed X2 CPGWalk policy receives filtered roll, filtered pitch and a
    zeroed yaw term.  The successful ARM64 CSV records this directly; it does
    not receive a projected gravity vector with a ``-1`` Z component.
    """

    values = _finite_vector(
        (roll, pitch, imu_roll_offset, imu_pitch_offset), 4, "CPG attitude"
    )
    return (values[0] + values[2], values[1] + values[3], 0.0)


@dataclass(frozen=True)
class X2VelocityCommandFilterParams:
    """Deployed X2 CPGWalk command shaping contract.

    The release controller first applies a simulation-time low-pass filter and
    then applies the walking deadband/minimum-forward-speed smoothstep.  The
    effective low-pass coefficient is reconstructed from the unstripped ARM64
    release as ``filter_x * dt / (dt + T)``.  Keeping this at the engine adapter
    boundary lets the policy session continue to consume the same *effective*
    command represented by the vendor debug CSV.
    """

    filter_x: float = 0.6
    time_constant_s: float = 0.5
    command_threshold: float = 0.10
    minimum_forward_command: float = 0.30

    def validate(self) -> None:
        values = (
            self.filter_x,
            self.time_constant_s,
            self.command_threshold,
            self.minimum_forward_command,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("velocity-command filter parameters contain NaN or Inf")
        if not 0.0 <= self.filter_x <= 1.0:
            raise ValueError("filter_x must be within [0, 1]")
        if self.time_constant_s < 0.0 or self.command_threshold < 0.0:
            raise ValueError("time constant and command threshold cannot be negative")
        if self.minimum_forward_command < self.command_threshold:
            raise ValueError("minimum forward command cannot be below the threshold")


@dataclass(frozen=True)
class X2VelocityCommandFilterOutput:
    raw: tuple[float, float, float]
    low_pass: tuple[float, float, float]
    effective: tuple[float, float, float]
    alpha: float


def apply_minimum_forward_command_shape(
    value: float, threshold: float, minimum_forward_command: float
) -> float:
    """Match the release controller's sign-preserving smoothstep exactly."""

    value, threshold, minimum_forward_command = _finite_vector(
        (value, threshold, minimum_forward_command), 3, "forward command shape"
    )
    magnitude = abs(value)
    if magnitude <= threshold:
        return 0.0
    if minimum_forward_command <= threshold + 1e-12 or magnitude >= minimum_forward_command:
        return value
    progress = _clamp(
        (magnitude - threshold) / (minimum_forward_command - threshold), 0.0, 1.0
    )
    smoothstep = progress * progress * (3.0 - 2.0 * progress)
    shaped = threshold + smoothstep * (minimum_forward_command - threshold)
    return math.copysign(shaped, value)


class X2VelocityCommandFilter:
    """Stateful, explicit-dt vendor command filter with no wall-clock use."""

    def __init__(
        self, params: X2VelocityCommandFilterParams = X2VelocityCommandFilterParams()
    ) -> None:
        params.validate()
        self.params = params
        self.reset()

    def reset(self) -> None:
        self.initialized = False
        self.filtered = (0.0, 0.0, 0.0)

    def snapshot(self) -> dict[str, object]:
        """Return the complete mutable state used by sim-time freeze probes."""

        return {
            "initialized": bool(self.initialized),
            "filtered": list(self.filtered),
        }

    def update(
        self, raw_command: Sequence[float], dt: float
    ) -> X2VelocityCommandFilterOutput:
        raw = _finite_vector(raw_command, 3, "raw velocity command")
        dt = float(dt)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("command-filter dt must be finite and positive")
        if self.params.time_constant_s > 1e-12:
            alpha = self.params.filter_x * dt / (dt + self.params.time_constant_s)
        else:
            alpha = self.params.filter_x
        alpha = _clamp(alpha, 0.0, 1.0)
        if not self.initialized:
            low_pass = raw
            self.initialized = True
        else:
            low_pass = tuple(
                previous + alpha * (target - previous)
                for previous, target in zip(self.filtered, raw)
            )
        self.filtered = low_pass
        threshold = self.params.command_threshold
        effective = (
            apply_minimum_forward_command_shape(
                low_pass[0], threshold, self.params.minimum_forward_command
            ),
            0.0 if abs(low_pass[1]) <= threshold else low_pass[1],
            0.0 if abs(low_pass[2]) <= threshold else low_pass[2],
        )
        return X2VelocityCommandFilterOutput(
            raw=raw,
            low_pass=low_pass,
            effective=effective,
            alpha=alpha,
        )


@dataclass(frozen=True)
class X2VelocityCommandCalibration:
    """Map external SI velocity requests to the vendor policy command domain.

    The initial forward scale is a provisional common calibration obtained
    from paired MuJoCo/PhysX 0.3 m/s development samples.  Keeping it explicit
    avoids mislabelling the vendor policy input as a directly calibrated m/s
    command and lets both engines use the same adapter contract.
    """

    # Paired MuJoCo/PhysX development runs show that the vendor policy command
    # domain is nonlinear around its 0.10 forward deadband and substantially
    # under-reacts to a numerically equal yaw request.  These remain explicit,
    # engine-neutral adapter calibrations rather than policy/physics tuning.
    forward_scale: float = 0.62
    lateral_scale: float = 1.0
    yaw_scale: float = 2.40
    # The frozen vendor MuJoCo reference has a materially stronger response to
    # negative yaw and can drive the left ankle target past its URDF limit when
    # a symmetric scale is used.  Keep that asymmetry explicit and apply the
    # identical signed mapping in both physics backends.
    negative_yaw_scale: float = 1.82

    def __post_init__(self) -> None:
        values = (
            self.forward_scale,
            self.lateral_scale,
            self.yaw_scale,
            self.negative_yaw_scale,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("velocity calibration scales must be finite and positive")

    def apply(self, command: Sequence[float]) -> tuple[float, float, float]:
        x, y, yaw = _finite_vector(command, 3, "velocity calibration command")
        yaw_scale = self.yaw_scale if yaw >= 0.0 else self.negative_yaw_scale
        return (
            x * self.forward_scale,
            y * self.lateral_scale,
            yaw * yaw_scale,
        )


@dataclass(frozen=True)
class X2PositionTargetSafetyParams:
    """Fail-closed limits for small raw policy target excursions.

    The vendor action pipeline does not clamp its filtered joint-position
    target to the robot model limits.  A physics backend must never receive an
    out-of-range target, but silently clipping it would hide controller/model
    divergence.  This envelope therefore records every raw excursion and only
    accepts a run when both its frequency and magnitude remain explicitly
    bounded.
    """

    maximum_saturation_ratio: float = 0.01
    maximum_raw_limit_excess_rad: float = 0.02

    def validate(self) -> None:
        values = (
            self.maximum_saturation_ratio,
            self.maximum_raw_limit_excess_rad,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError(
                "position-target safety thresholds must be finite and non-negative"
            )
        if self.maximum_saturation_ratio > 1.0:
            raise ValueError("maximum position-target saturation ratio cannot exceed 1")


@dataclass(frozen=True)
class X2PositionTargetSafetyOutput:
    raw_position_target: tuple[float, ...]
    applied_position_target: tuple[float, ...]
    saturation_count: int
    maximum_raw_limit_excess_rad: float


class X2PositionTargetSafetyEnvelope:
    """Engine-neutral, auditable clamp at the controller/physics boundary."""

    def __init__(
        self,
        joint_limits: Mapping[str, Sequence[float]],
        params: X2PositionTargetSafetyParams = X2PositionTargetSafetyParams(),
    ) -> None:
        params.validate()
        if not joint_limits:
            raise ValueError("position-target safety envelope needs joint limits")
        normalized: dict[str, tuple[float, float]] = {}
        for raw_name, raw_limit in joint_limits.items():
            name = str(raw_name)
            if not name or name in normalized:
                raise ValueError("position-target joint names must be unique and non-empty")
            lower, upper = _finite_vector(
                raw_limit, 2, f"position-target limit for {name}"
            )
            if lower >= upper:
                raise ValueError(f"position-target limit is invalid for {name}")
            normalized[name] = (lower, upper)
        self.params = params
        self.joint_limits = normalized
        self.target_sample_count = 0
        self.saturation_count = 0
        self.maximum_raw_limit_excess_rad = 0.0
        self.saturation_count_by_joint = {name: 0 for name in normalized}
        self.maximum_raw_limit_excess_by_joint_rad = {
            name: 0.0 for name in normalized
        }
        self.events: list[dict[str, object]] = []

    def apply(
        self,
        joint_names: Sequence[str],
        position_target: Sequence[float],
        *,
        sim_time: float,
        phase: str,
        policy_sequence: int,
    ) -> X2PositionTargetSafetyOutput:
        names = tuple(str(name) for name in joint_names)
        if len(set(names)) != len(names):
            raise ValueError("position-target command contains duplicate joint names")
        raw = _finite_vector(position_target, len(names), "raw position target")
        sim_time = float(sim_time)
        policy_sequence = int(policy_sequence)
        if not math.isfinite(sim_time) or sim_time < 0.0:
            raise ValueError("position-target sim_time must be finite and non-negative")
        if policy_sequence < 0:
            raise ValueError("position-target policy_sequence cannot be negative")
        if not phase:
            raise ValueError("position-target phase must be non-empty")

        applied: list[float] = []
        local_saturation_count = 0
        local_maximum_excess = 0.0
        for name, value in zip(names, raw):
            if name not in self.joint_limits:
                raise ValueError(f"position-target limit missing for {name}")
            lower, upper = self.joint_limits[name]
            limited = _clamp(value, lower, upper)
            excess = max(lower - value, value - upper, 0.0)
            applied.append(limited)
            self.target_sample_count += 1
            if excess <= 0.0:
                continue
            local_saturation_count += 1
            local_maximum_excess = max(local_maximum_excess, excess)
            self.saturation_count += 1
            self.maximum_raw_limit_excess_rad = max(
                self.maximum_raw_limit_excess_rad, excess
            )
            self.saturation_count_by_joint[name] += 1
            self.maximum_raw_limit_excess_by_joint_rad[name] = max(
                self.maximum_raw_limit_excess_by_joint_rad[name], excess
            )
            self.events.append({
                "sim_time": sim_time,
                "phase": str(phase),
                "policy_sequence": policy_sequence,
                "joint_name": name,
                "raw_position_target_rad": value,
                "applied_position_target_rad": limited,
                "lower_limit_rad": lower,
                "upper_limit_rad": upper,
                "raw_limit_excess_rad": excess,
            })

        return X2PositionTargetSafetyOutput(
            raw_position_target=raw,
            applied_position_target=tuple(applied),
            saturation_count=local_saturation_count,
            maximum_raw_limit_excess_rad=local_maximum_excess,
        )

    def snapshot(self) -> dict[str, object]:
        ratio = (
            self.saturation_count / self.target_sample_count
            if self.target_sample_count
            else 1.0
        )
        return {
            "schema": "sceneforge/x2-position-target-safety-envelope/v1",
            "maximum_saturation_ratio": self.params.maximum_saturation_ratio,
            "maximum_raw_limit_excess_rad": (
                self.params.maximum_raw_limit_excess_rad
            ),
            "raw_target_sample_count": self.target_sample_count,
            "saturation_count": self.saturation_count,
            "saturation_ratio": ratio,
            "observed_maximum_raw_limit_excess_rad": (
                self.maximum_raw_limit_excess_rad
            ),
            "saturation_count_by_joint": {
                name: count
                for name, count in self.saturation_count_by_joint.items()
                if count
            },
            "maximum_raw_limit_excess_by_joint_rad": {
                name: value
                for name, value in self.maximum_raw_limit_excess_by_joint_rad.items()
                if value > 0.0
            },
            "events": list(self.events),
            "bounded": (
                self.target_sample_count > 0
                and ratio <= self.params.maximum_saturation_ratio
                and self.maximum_raw_limit_excess_rad
                <= self.params.maximum_raw_limit_excess_rad
            ),
        }


@dataclass(frozen=True)
class X2HeadingHoldParams:
    # Straight-line PhysX qualification at 0.3 m/s showed a small residual
    # 0.0605 rad/m yaw drift with kp=1.1 while the correction remained well
    # below its bound.  This common MuJoCo/PhysX gain adds margin without
    # changing any non-zero-yaw turning profile.
    kp: float = 1.4
    kd: float = 0.1
    maximum_yaw_rate_rps: float = 0.25

    def validate(self) -> None:
        values = (self.kp, self.kd, self.maximum_yaw_rate_rps)
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("heading-hold parameters must be finite and non-negative")
        if self.maximum_yaw_rate_rps <= 0.0:
            raise ValueError("maximum yaw rate must be positive")


def wrap_angle_radians(value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("angle must be finite")
    return math.atan2(math.sin(value), math.cos(value))


class X2HeadingHoldController:
    """Engine-neutral outer-loop heading hold for straight-line tests."""

    def __init__(self, params: X2HeadingHoldParams = X2HeadingHoldParams()) -> None:
        params.validate()
        self.params = params
        self.reference_yaw: float | None = None

    def reset(self, reference_yaw: float) -> None:
        self.reference_yaw = wrap_angle_radians(reference_yaw)

    def snapshot(self) -> dict[str, object]:
        return {"reference_yaw": self.reference_yaw}

    def update(self, current_yaw: float, body_yaw_rate: float) -> float:
        if self.reference_yaw is None:
            raise RuntimeError("heading hold must be reset at locomotion entry")
        current_yaw, body_yaw_rate = _finite_vector(
            (current_yaw, body_yaw_rate), 2, "heading-hold state"
        )
        error = wrap_angle_radians(self.reference_yaw - current_yaw)
        command = self.params.kp * error - self.params.kd * body_yaw_rate
        limit = self.params.maximum_yaw_rate_rps
        return _clamp(command, -limit, limit)


@dataclass(frozen=True)
class X2CrossTrackHoldParams:
    # The first final-calibration PhysX straight run missed the 0.05 m/m gate
    # narrowly (0.0526) while using at most 0.0233 m/s of the 0.08 bound.
    # Increase the common P-only correction to retain qualification margin.
    kp: float = 0.33
    # Keep the first qualified controller P-only.  The vendor MuJoCo model
    # reports velocity at an offset IMU site while PhysX reports base-link
    # velocity, so a shared D term is not engine-neutral until both are
    # projected from the same origin into the walk-entry path frame.
    kd: float = 0.0
    maximum_lateral_speed_mps: float = 0.08

    def validate(self) -> None:
        values = (self.kp, self.kd, self.maximum_lateral_speed_mps)
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("cross-track parameters must be finite and non-negative")
        if self.maximum_lateral_speed_mps <= 0.0:
            raise ValueError("maximum lateral speed must be positive")


class X2CrossTrackHoldController:
    """Small engine-neutral lateral outer loop for a zero-yaw straight task."""

    def __init__(
        self, params: X2CrossTrackHoldParams = X2CrossTrackHoldParams()
    ) -> None:
        params.validate()
        self.params = params
        self.reference_position_xy: tuple[float, float] | None = None
        self.reference_yaw: float | None = None

    def reset(self, position_xy: Sequence[float], reference_yaw: float) -> None:
        position = _finite_vector(position_xy, 2, "cross-track reference position")
        self.reference_position_xy = position
        self.reference_yaw = wrap_angle_radians(reference_yaw)

    def snapshot(self) -> dict[str, object]:
        return {
            "reference_position_xy": (
                list(self.reference_position_xy)
                if self.reference_position_xy is not None
                else None
            ),
            "reference_yaw": self.reference_yaw,
        }

    def update(
        self,
        position_xy: Sequence[float],
        body_lateral_velocity_mps: float,
    ) -> tuple[float, float]:
        if self.reference_position_xy is None or self.reference_yaw is None:
            raise RuntimeError("cross-track hold must be reset at locomotion entry")
        x, y = _finite_vector(position_xy, 2, "cross-track position")
        lateral_velocity = _finite_vector(
            (body_lateral_velocity_mps,), 1, "cross-track lateral velocity"
        )[0]
        dx = x - self.reference_position_xy[0]
        dy = y - self.reference_position_xy[1]
        cross_track = -dx * math.sin(self.reference_yaw) + dy * math.cos(
            self.reference_yaw
        )
        command = -self.params.kp * cross_track - self.params.kd * lateral_velocity
        limit = self.params.maximum_lateral_speed_mps
        return cross_track, _clamp(command, -limit, limit)


def apply_post_filter_lateral_stabilization(
    effective_command: Sequence[float],
    lateral_correction_mps: float,
    *,
    maximum_lateral_speed_mps: float,
) -> tuple[float, float, float]:
    """Apply a bounded stabilization correction after the vendor deadband.

    The release command filter zeros lateral requests below its 0.10 command
    threshold.  Cross-track hold intentionally produces smaller corrections,
    so feeding it through that deadband disables the controller entirely.  The
    outer-loop correction is therefore added after vendor command shaping while
    the user command continues to follow the unmodified release filter.
    """

    x, y, yaw = _finite_vector(effective_command, 3, "effective velocity command")
    correction, limit = _finite_vector(
        (lateral_correction_mps, maximum_lateral_speed_mps),
        2,
        "lateral stabilization command",
    )
    if limit <= 0.0:
        raise ValueError("maximum lateral stabilization speed must be positive")
    return (x, _clamp(y + correction, -limit, limit), yaw)


def apply_post_filter_yaw_stabilization(
    effective_command: Sequence[float],
    yaw_correction_rps: float,
    *,
    maximum_yaw_rate_rps: float,
) -> tuple[float, float, float]:
    """Apply straight-line heading hold after the vendor yaw deadband."""

    x, y, yaw = _finite_vector(effective_command, 3, "effective velocity command")
    correction, limit = _finite_vector(
        (yaw_correction_rps, maximum_yaw_rate_rps),
        2,
        "yaw stabilization command",
    )
    if limit <= 0.0:
        raise ValueError("maximum yaw stabilization rate must be positive")
    return (x, y, _clamp(yaw + correction, -limit, limit))


@dataclass(frozen=True)
class AdaptiveHopfParams:
    x2_style_adapter: bool = False
    alpha: float = 8.0
    mu0: float = 0.99
    k_mu: float = 0.06
    mu_min: float = 0.8281
    mu_max: float = 1.2321
    period_gait1: float = 0.87
    period_gait2: float = 0.60
    coupling_gain: float = 2.0
    use_coupling: bool = True
    reset_on_walk_entry: bool = True
    gait_blend_tau: float = 0.20
    phase_offset: float = 0.5
    vel_tau: float = 0.25
    yaw_tau: float = 0.25
    cmd_tau: float = 0.20
    lambda_y: float = 1.0
    lambda_yaw: float = 0.15
    asymmetry_gain: float = 0.006
    yaw_asymmetry_gain: float = 0.10

    def validate(self) -> None:
        finite = all(math.isfinite(float(value)) for value in self.__dict__.values())
        if not finite:
            raise ValueError("Hopf parameters contain NaN or Inf")
        if self.alpha <= 0 or self.mu_min < 0 or self.mu_max < self.mu_min:
            raise ValueError("invalid Hopf radial parameters")
        if self.period_gait1 <= 0 or self.period_gait2 <= 0:
            raise ValueError("gait periods must be positive")
        if self.vel_tau < 0 or self.yaw_tau < 0 or self.cmd_tau < 0:
            raise ValueError("filter time constants cannot be negative")


@dataclass(frozen=True)
class AdaptiveHopfSnapshot:
    mode: HopfMode
    gait: int
    walk_initialized: bool
    period: float
    obs4: tuple[float, float, float, float]
    filtered_velocity: tuple[float, float, float]
    filtered_command: tuple[float, float]


X2_CPGWALK_JOINT_NAMES = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_pitch_joint",
    "waist_roll_joint",
    "left_shoulder_pitch_joint",
    "right_shoulder_pitch_joint",
)


@dataclass(frozen=True)
class CpgWalkActionParams:
    """Deployed X2 CPGWalk action and position-PD contract.

    The phase-only profile keeps the CPG nominal pose at the configured default
    joint position.  The learned action is a residual which is low-pass filtered,
    added to that pose, and filtered again before being emitted as a position
    target.  All updates are explicit-``dt`` and engine neutral.
    """

    joint_names: tuple[str, ...] = X2_CPGWALK_JOINT_NAMES
    action_scale: tuple[float, ...] = (
        0.5, 0.5, 0.5, 0.5, 0.2, 0.02,
        0.5, 0.5, 0.5, 0.5, 0.2, 0.02,
        0.2, 0.2, 0.2, 0.2, 0.2,
    )
    action_cutoff_hz: tuple[float, ...] = (
        5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0,
        5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0,
        10.0, 10.0, 10.0, 50.0, 50.0,
    )
    nominal_position: tuple[float, ...] = (
        -0.2480, 0.0, 0.0, 0.5303, -0.2823, 0.0,
        -0.2480, 0.0, 0.0, 0.5303, -0.2823, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0,
    )
    kp: tuple[float, ...] = (
        120.0, 120.0, 120.0, 150.0, 40.0, 30.0,
        120.0, 120.0, 120.0, 150.0, 40.0, 30.0,
        160.0, 80.0, 80.0, 80.0, 80.0,
    )
    kd: tuple[float, ...] = (
        5.0, 5.0, 5.0, 5.0, 3.0, 2.0,
        5.0, 5.0, 5.0, 5.0, 3.0, 2.0,
        5.0, 5.0, 5.0, 4.0, 4.0,
    )
    action_clip: float = 18.0
    waist_pitch_residual_scale: float = 0.5
    waist_roll_residual_scale: float = 0.7
    waist_pitch_residual_max_abs: float = 0.25
    waist_roll_residual_max_abs: float = 0.14

    def validate(self) -> None:
        width = len(X2_CPGWALK_JOINT_NAMES)
        vectors = {
            "joint_names": self.joint_names,
            "action_scale": self.action_scale,
            "action_cutoff_hz": self.action_cutoff_hz,
            "nominal_position": self.nominal_position,
            "kp": self.kp,
            "kd": self.kd,
        }
        for label, values in vectors.items():
            if len(values) != width:
                raise ValueError(f"{label} width {len(values)} != {width}")
        if tuple(self.joint_names) != X2_CPGWALK_JOINT_NAMES:
            raise ValueError("CPGWalk joint order does not match the deployed contract")
        finite_values = (
            *self.action_scale,
            *self.action_cutoff_hz,
            *self.nominal_position,
            *self.kp,
            *self.kd,
            self.action_clip,
            self.waist_pitch_residual_scale,
            self.waist_roll_residual_scale,
            self.waist_pitch_residual_max_abs,
            self.waist_roll_residual_max_abs,
        )
        if not all(math.isfinite(float(value)) for value in finite_values):
            raise ValueError("CPGWalk action parameters contain NaN or Inf")
        if self.action_clip <= 0 or any(value <= 0 for value in self.action_cutoff_hz):
            raise ValueError("action clip and cutoff frequencies must be positive")
        if any(value < 0 for value in (*self.kp, *self.kd)):
            raise ValueError("PD gains cannot be negative")


@dataclass(frozen=True)
class CpgWalkActionSnapshot:
    residual: tuple[float, ...]
    filtered_position_target: tuple[float, ...]


@dataclass(frozen=True)
class CpgWalkActionOutput:
    residual: tuple[float, ...]
    nominal_position: tuple[float, ...]
    position_target_unfiltered: tuple[float, ...]
    position_target: tuple[float, ...]
    velocity_target: tuple[float, ...]
    feedforward_effort: tuple[float, ...]
    kp: tuple[float, ...]
    kd: tuple[float, ...]


def first_order_low_pass_alpha(dt: float, cutoff_hz: float) -> float:
    dt, cutoff_hz = _finite_vector((dt, cutoff_hz), 2, "low-pass parameters")
    if dt <= 0 or cutoff_hz <= 0:
        raise ValueError("dt and cutoff frequency must be positive")
    omega_dt = 2.0 * math.pi * cutoff_hz * dt
    return omega_dt / (1.0 + omega_dt)


class CpgWalkActionPipeline:
    """State-driven residual, target-filter and position-PD command builder.

    Only the active-locomotion path is implemented here.  Entry/exit planning
    and stand hand-off remain a separate safety state machine and must not be
    approximated by silently reusing the walking update.
    """

    def __init__(self, params: CpgWalkActionParams = CpgWalkActionParams()) -> None:
        params.validate()
        self.params = params
        self.reset()

    def reset(self) -> None:
        width = len(self.params.joint_names)
        self.residual = (0.0,) * width
        self.filtered_position_target = tuple(self.params.nominal_position)

    def snapshot(self) -> CpgWalkActionSnapshot:
        return CpgWalkActionSnapshot(
            residual=tuple(self.residual),
            filtered_position_target=tuple(self.filtered_position_target),
        )

    def restore(self, snapshot: CpgWalkActionSnapshot) -> None:
        width = len(self.params.joint_names)
        self.residual = _finite_vector(snapshot.residual, width, "residual snapshot")
        self.filtered_position_target = _finite_vector(
            snapshot.filtered_position_target, width, "position-target snapshot"
        )

    def step_active(self, action: Sequence[float], dt: float) -> CpgWalkActionOutput:
        """Advance one active walking policy tick.

        This method intentionally has no stand/freeze option.  Calling code must
        pass the dedicated transition safety gate before selecting this path.
        """

        width = len(self.params.joint_names)
        action = _finite_vector(action, width, "policy action")
        dt = float(dt)
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive")

        desired_residual = [
            _clamp(value, -self.params.action_clip, self.params.action_clip) * scale
            for value, scale in zip(action, self.params.action_scale)
        ]
        desired_residual[13] = _clamp(
            desired_residual[13] * self.params.waist_pitch_residual_scale,
            -self.params.waist_pitch_residual_max_abs,
            self.params.waist_pitch_residual_max_abs,
        )
        desired_residual[14] = _clamp(
            desired_residual[14] * self.params.waist_roll_residual_scale,
            -self.params.waist_roll_residual_max_abs,
            self.params.waist_roll_residual_max_abs,
        )
        alphas = tuple(
            first_order_low_pass_alpha(dt, cutoff)
            for cutoff in self.params.action_cutoff_hz
        )
        residual = tuple(
            previous + alpha * (desired - previous)
            for previous, desired, alpha in zip(
                self.residual, desired_residual, alphas
            )
        )
        unfiltered = tuple(
            nominal + correction
            for nominal, correction in zip(self.params.nominal_position, residual)
        )
        filtered = tuple(
            previous + alpha * (target - previous)
            for previous, target, alpha in zip(
                self.filtered_position_target, unfiltered, alphas
            )
        )
        self.residual = residual
        self.filtered_position_target = filtered
        zeros = (0.0,) * width
        return CpgWalkActionOutput(
            residual=residual,
            nominal_position=tuple(self.params.nominal_position),
            position_target_unfiltered=unfiltered,
            position_target=filtered,
            velocity_target=zeros,
            feedforward_effort=zeros,
            kp=tuple(self.params.kp),
            kd=tuple(self.params.kd),
        )


class CpgWalkSafetyMode(IntEnum):
    STAND = 0
    ACTIVE = 1


@dataclass(frozen=True)
class CpgWalkGaitSwitchParams:
    gait1_to_gait2_speed_threshold: float = 0.5
    gait2_to_gait1_speed_threshold: float = 0.4
    gait2_to_gait1_yaw_block_threshold: float = 0.35
    initial_gait: int = 1

    def validate(self) -> None:
        numeric = (
            self.gait1_to_gait2_speed_threshold,
            self.gait2_to_gait1_speed_threshold,
            self.gait2_to_gait1_yaw_block_threshold,
        )
        if not all(math.isfinite(float(value)) and value >= 0 for value in numeric):
            raise ValueError("gait thresholds must be finite and non-negative")
        if self.gait1_to_gait2_speed_threshold <= self.gait2_to_gait1_speed_threshold:
            raise ValueError("gait hysteresis thresholds overlap")
        if self.initial_gait not in (1, 2):
            raise ValueError("initial gait must be 1 or 2")


class CpgWalkGaitSelector:
    """Deterministic gait1/gait2 hysteresis driven by filtered commands."""

    def __init__(
        self, params: CpgWalkGaitSwitchParams = CpgWalkGaitSwitchParams()
    ) -> None:
        params.validate()
        self.params = params
        self.reset()

    def reset(self) -> None:
        self.gait = self.params.initial_gait

    def update(self, command_x: float, command_yaw: float) -> int:
        command_x, command_yaw = _finite_vector(
            (command_x, command_yaw), 2, "gait command"
        )
        speed = abs(command_x)
        if (
            self.gait == 1
            and speed > self.params.gait1_to_gait2_speed_threshold
        ):
            self.gait = 2
        elif (
            self.gait == 2
            and speed < self.params.gait2_to_gait1_speed_threshold
            and abs(command_yaw) < self.params.gait2_to_gait1_yaw_block_threshold
        ):
            self.gait = 1
        return self.gait


@dataclass(frozen=True)
class CpgWalkSafetyParams:
    command_threshold: float = 0.10
    tilt_freeze_enabled: bool = False
    roll_pitch_threshold: float = 0.06
    require_low_motion_for_freeze: bool = False
    require_gait1_for_freeze: bool = False
    leg_velocity_threshold: float = 0.8
    base_angular_velocity_threshold: float = 0.8
    auto_stand_hold_seconds: float = 1.0

    def validate(self) -> None:
        numeric = (
            self.command_threshold,
            self.roll_pitch_threshold,
            self.leg_velocity_threshold,
            self.base_angular_velocity_threshold,
            self.auto_stand_hold_seconds,
        )
        if not all(math.isfinite(float(value)) for value in numeric):
            raise ValueError("safety parameters contain NaN or Inf")
        if any(value < 0 for value in numeric):
            raise ValueError("safety thresholds cannot be negative")


@dataclass(frozen=True)
class CpgWalkSafetyDecision:
    mode: CpgWalkSafetyMode
    freeze_command_candidate: bool
    freeze_by_attitude: bool
    low_motion_ready: bool
    freeze: bool
    stand_elapsed_seconds: float
    request_stand_transition: bool


class CpgWalkSafetyGate:
    """Explicit-simulation-time command/freeze and stand hand-off gate."""

    def __init__(self, params: CpgWalkSafetyParams = CpgWalkSafetyParams()) -> None:
        params.validate()
        self.params = params
        self.reset()

    def reset(self) -> None:
        self.mode = CpgWalkSafetyMode.STAND
        self.stand_elapsed_seconds = 0.0
        self.stand_transition_requested = False

    def update(
        self,
        *,
        dt: float,
        command_xyz: Sequence[float],
        roll: float,
        pitch: float,
        mean_leg_velocity: float,
        base_angular_velocity_norm: float,
        gait: int,
    ) -> CpgWalkSafetyDecision:
        values = _finite_vector(
            (
                dt,
                *command_xyz,
                roll,
                pitch,
                mean_leg_velocity,
                base_angular_velocity_norm,
            ),
            8,
            "safety update",
        )
        dt, command_x, command_y, command_yaw, roll, pitch, leg_velocity, angular_velocity = values
        if dt <= 0:
            raise ValueError("dt must be positive")
        if gait not in (1, 2):
            raise ValueError("gait must be 1 or 2")
        command_norm = math.sqrt(
            command_x * command_x + command_y * command_y + command_yaw * command_yaw
        )
        command_candidate = command_norm < self.params.command_threshold
        freeze_by_attitude = self.params.tilt_freeze_enabled and (
            abs(roll) > self.params.roll_pitch_threshold
            or abs(pitch) > self.params.roll_pitch_threshold
        )
        low_motion_ready = (
            abs(leg_velocity) < self.params.leg_velocity_threshold
            and abs(angular_velocity) < self.params.base_angular_velocity_threshold
        )
        command_freeze_allowed = command_candidate
        if self.params.require_low_motion_for_freeze:
            command_freeze_allowed = command_freeze_allowed and low_motion_ready
        if self.params.require_gait1_for_freeze:
            command_freeze_allowed = command_freeze_allowed and gait == 1
        freeze = freeze_by_attitude or command_freeze_allowed

        request = False
        if freeze:
            if self.mode != CpgWalkSafetyMode.STAND:
                self.stand_elapsed_seconds = 0.0
                self.stand_transition_requested = False
            self.mode = CpgWalkSafetyMode.STAND
            self.stand_elapsed_seconds += dt
            if (
                not self.stand_transition_requested
                and self.stand_elapsed_seconds + 1e-12 >= self.params.auto_stand_hold_seconds
            ):
                self.stand_transition_requested = True
                request = True
        else:
            self.mode = CpgWalkSafetyMode.ACTIVE
            self.stand_elapsed_seconds = 0.0
            self.stand_transition_requested = False

        return CpgWalkSafetyDecision(
            mode=self.mode,
            freeze_command_candidate=command_candidate,
            freeze_by_attitude=freeze_by_attitude,
            low_motion_ready=low_motion_ready,
            freeze=freeze,
            stand_elapsed_seconds=self.stand_elapsed_seconds,
            request_stand_transition=request,
        )


def _finite_vector(values: Sequence[float], width: int, label: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != width:
        raise ValueError(f"{label} width {len(result)} != {width}")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} contains NaN or Inf")
    return result


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _wrap_to_pi(angle: float) -> float:
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    while angle > math.pi:
        angle -= 2.0 * math.pi
    return angle


class TwoLegAdaptiveHopf:
    """Two-leg adaptive Hopf oscillator with simulation-time-only updates."""

    def __init__(self, params: AdaptiveHopfParams = AdaptiveHopfParams()) -> None:
        params.validate()
        self.params = params
        self.reset()

    def reset(self) -> None:
        self.mode = HopfMode.STAND
        self.gait = 2
        self.walk_initialized = False
        self.period = self.params.period_gait2
        self.x_left = 0.0
        self.y_left = 1.0
        self.x_right = 0.0
        self.y_right = 1.0
        self.vx_filtered = 0.0
        self.vy_filtered = 0.0
        self.wz_filtered = 0.0
        self.cmd_y_filtered = 0.0
        self.cmd_yaw_filtered = 0.0

    def set_mode(self, mode: HopfMode) -> None:
        mode = HopfMode(mode)
        previous = self.mode
        if previous == mode:
            return
        self.mode = mode
        if previous == HopfMode.STAND and mode == HopfMode.WALK:
            self.walk_initialized = False
            if self.params.reset_on_walk_entry:
                self._reset_walk_state()
        elif mode == HopfMode.STAND:
            self.walk_initialized = False

    def set_gait(self, gait: int) -> None:
        if gait not in (1, 2):
            raise ValueError("gait must be 1 or 2")
        self.gait = gait

    def update_base_velocity(self, dt: float, vx: float, vy: float, wz: float) -> None:
        dt, vx, vy, wz = _finite_vector((dt, vx, vy, wz), 4, "base velocity update")
        if dt <= 0:
            raise ValueError("dt must be positive")
        velocity_alpha = dt / (dt + self.params.vel_tau)
        yaw_alpha = dt / (dt + self.params.yaw_tau)
        self.vx_filtered += velocity_alpha * (vx - self.vx_filtered)
        self.vy_filtered += velocity_alpha * (vy - self.vy_filtered)
        self.wz_filtered += yaw_alpha * (wz - self.wz_filtered)

    def update_command_inputs(self, dt: float, command_y: float, command_yaw: float) -> None:
        dt, command_y, command_yaw = _finite_vector(
            (dt, command_y, command_yaw), 3, "command update"
        )
        if dt <= 0:
            raise ValueError("dt must be positive")
        alpha = dt / (dt + self.params.cmd_tau)
        self.cmd_y_filtered += alpha * (command_y - self.cmd_y_filtered)
        self.cmd_yaw_filtered += alpha * (command_yaw - self.cmd_yaw_filtered)

    def locomotion_intensity(self) -> float:
        return math.sqrt(
            self.vx_filtered * self.vx_filtered
            + self.params.lambda_y * self.vy_filtered * self.vy_filtered
            + self.params.lambda_yaw * self.wz_filtered * self.wz_filtered
        )

    def current_mu(self) -> float:
        return _clamp(
            self.params.mu0 + self.params.k_mu * self.locomotion_intensity(),
            self.params.mu_min,
            self.params.mu_max,
        )

    def leg_mu(self) -> tuple[float, float]:
        base = self.current_mu()
        asymmetry = self.params.asymmetry_gain * (
            self.cmd_y_filtered
            + self.params.yaw_asymmetry_gain * self.cmd_yaw_filtered
        )
        return (
            _clamp(base + asymmetry, self.params.mu_min, self.params.mu_max),
            _clamp(base - asymmetry, self.params.mu_min, self.params.mu_max),
        )

    def _target_period(self) -> float:
        return self.params.period_gait1 if self.gait == 1 else self.params.period_gait2

    def _reset_walk_state(self) -> None:
        radius = math.sqrt(max(0.0, self.current_mu()))
        self.x_left = 0.0
        self.y_left = radius
        self.x_right = 0.0
        self.y_right = -radius
        self.walk_initialized = True

    def step(self, dt: float) -> None:
        dt = float(dt)
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive")
        if self.mode == HopfMode.STAND:
            return
        if not self.walk_initialized:
            self._reset_walk_state()

        target_period = self._target_period()
        if self.params.x2_style_adapter and self.params.gait_blend_tau > 1e-9:
            blend = dt / (self.params.gait_blend_tau + dt)
            self.period += blend * (target_period - self.period)
        else:
            self.period = target_period
        omega = 2.0 * math.pi / max(self.period, 1e-3)
        coupling = self.params.coupling_gain if self.params.use_coupling else 0.0

        radius_left = math.hypot(self.x_left, self.y_left)
        radius_right = math.hypot(self.x_right, self.y_right)
        # The deployed ABI defines phase as atan2(x, y), so x=sin(theta)
        # and y=cos(theta).  This is intentionally not conventional atan2(y,x).
        theta_left = math.atan2(self.x_left, self.y_left)
        theta_right = math.atan2(self.x_right, self.y_right)
        phase_offset = 2.0 * math.pi * self.params.phase_offset
        omega_left = omega + coupling * math.sin(
            theta_right - theta_left - phase_offset
        )
        omega_right = omega + coupling * math.sin(
            theta_left - theta_right + phase_offset
        )
        mu_left, mu_right = self.leg_mu()
        radius_left = max(
            0.0,
            radius_left
            + dt * self.params.alpha * (mu_left - radius_left * radius_left) * radius_left,
        )
        radius_right = max(
            0.0,
            radius_right
            + dt
            * self.params.alpha
            * (mu_right - radius_right * radius_right)
            * radius_right,
        )
        theta_left = _wrap_to_pi(theta_left + dt * omega_left)
        theta_right = _wrap_to_pi(theta_right + dt * omega_right)
        self.x_left = radius_left * math.sin(theta_left)
        self.y_left = radius_left * math.cos(theta_left)
        self.x_right = radius_right * math.sin(theta_right)
        self.y_right = radius_right * math.cos(theta_right)

    def obs4(self) -> tuple[float, float, float, float]:
        if self.mode == HopfMode.STAND:
            return (0.0, 1.0, 0.0, 1.0)
        return (self.x_left, self.y_left, self.x_right, self.y_right)

    def snapshot(self) -> AdaptiveHopfSnapshot:
        return AdaptiveHopfSnapshot(
            mode=self.mode,
            gait=self.gait,
            walk_initialized=self.walk_initialized,
            period=self.period,
            # Preserve the internal oscillator state even in stand mode;
            # obs4() intentionally exposes the deployed fixed stand vector.
            obs4=(self.x_left, self.y_left, self.x_right, self.y_right),
            filtered_velocity=(self.vx_filtered, self.vy_filtered, self.wz_filtered),
            filtered_command=(self.cmd_y_filtered, self.cmd_yaw_filtered),
        )

    def restore(self, snapshot: AdaptiveHopfSnapshot) -> None:
        period = float(snapshot.period)
        if not math.isfinite(period) or period <= 0:
            raise ValueError("snapshot period must be finite and positive")
        obs4 = _finite_vector(snapshot.obs4, 4, "snapshot obs4")
        velocity = _finite_vector(snapshot.filtered_velocity, 3, "snapshot velocity")
        command = _finite_vector(snapshot.filtered_command, 2, "snapshot command")
        self.mode = HopfMode(snapshot.mode)
        self.set_gait(int(snapshot.gait))
        self.walk_initialized = bool(snapshot.walk_initialized)
        self.period = period
        self.x_left, self.y_left, self.x_right, self.y_right = obs4
        self.vx_filtered, self.vy_filtered, self.wz_filtered = velocity
        self.cmd_y_filtered, self.cmd_yaw_filtered = command
