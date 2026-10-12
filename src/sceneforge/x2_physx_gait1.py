"""Independent X2 gait1 development capture with Genie Sim PhysX authority."""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Callable

from .x2_control import (
    X2_CPGWALK_JOINT_NAMES,
    X2CrossTrackHoldController,
    X2HeadingHoldController,
    X2PositionTargetSafetyEnvelope,
    X2VelocityCommandCalibration,
    X2VelocityCommandFilter,
    apply_post_filter_lateral_stabilization,
    apply_post_filter_yaw_stabilization,
    x2_cpgwalk_attitude_observation,
)
from .x2_fixed_base_stand import _OBSERVATION_DEFAULT, _atomic_json, _percentile, _sha256
from .x2_floating_stand import X2FloatingStandCapture, _project_gravity_with_vendor_pitch_offset
from .x2_inprocess_shadow import _OnnxPolicy
from .x2_locomotion_metrics import (
    pose_derived_forward_speed,
    pose_derived_local_displacement,
    pose_derived_turn_rate,
    summarize_stable_stop,
    summarize_support_slip,
)
from .x2_mujoco_motion import (
    X2MujocoMotionProfile,
    X2_JOINT_VELOCITY_NUMERIC_TOLERANCE_RATIO,
    quaternion_rpy_wxyz,
)
from .x2_session import StateDrivenX2ControllerSession, X2PhysicsState
from .x2_stand import (
    StateDrivenX2StandSession,
    X2StandPhysicsState,
    X2_STAND_ARM_TARGET,
    X2_STAND_JOINT_NAMES,
)


SCHEMA = "sceneforge/x2-geniesim-independent-gait1-development/v1"


class X2PhysxGait1Capture(X2FloatingStandCapture):
    """Run the same stand/CPGWalk adapter used by the MuJoCo baseline."""

    def __init__(
        self,
        sim,
        *,
        stand_model: str | Path,
        walk_model: str | Path,
        receipt: str | Path,
        trace: str | Path,
        stop_request: str | Path,
        band_command: str | Path,
        band_status: str | Path,
        state_reader: Callable[[float], object],
        contact_reader: Callable[[float], dict] | None,
        profile: X2MujocoMotionProfile,
        joint_limits: dict[str, tuple[float, float]],
        joint_velocity_limits: dict[str, float],
        joint_limit_source: str,
        physics_dt_s: float,
        freeze_probe_wall_s: float = 2.0,
        base_state_source: str = "unspecified",
    ) -> None:
        profile.validate()
        super().__init__(
            sim,
            model=stand_model,
            receipt=receipt,
            trace=trace,
            stop_request=stop_request,
            band_command=band_command,
            band_status=band_status,
            state_reader=state_reader,
            contact_reader=contact_reader,
            supported_s=profile.supported_stand_s,
            ramp_s=profile.band_ramp_s,
            unsupported_hold_s=profile.unsupported_stand_s,
        )
        self.profile = profile
        if tuple(joint_limits) != self.articulation_joint_names:
            raise ValueError("runtime joint-limit map does not match articulation order")
        self.joint_limits = dict(joint_limits)
        if tuple(joint_velocity_limits) != self.articulation_joint_names:
            raise ValueError("runtime joint-velocity map does not match articulation order")
        self.joint_velocity_limits = dict(joint_velocity_limits)
        self.joint_limit_source = str(joint_limit_source)
        if not self.joint_limit_source:
            raise ValueError("joint_limit_source must be non-empty")
        self.physics_dt_s = float(physics_dt_s)
        if not math.isfinite(self.physics_dt_s) or self.physics_dt_s <= 0.0:
            raise ValueError("physics_dt_s must be finite and positive")
        self.freeze_probe_wall_s = float(freeze_probe_wall_s)
        if not math.isfinite(self.freeze_probe_wall_s) or self.freeze_probe_wall_s <= 0.0:
            raise ValueError("freeze_probe_wall_s must be finite and positive")
        self.base_state_source = str(base_state_source)
        if not self.base_state_source:
            raise ValueError("base_state_source must be non-empty")
        self.total_duration_s = profile.total_duration_s
        self.walk_model = Path(walk_model).resolve()
        if not self.walk_model.is_file():
            raise FileNotFoundError(self.walk_model)
        self.walk_policy = _OnnxPolicy(self.walk_model)
        self.walk_session = StateDrivenX2ControllerSession(self.walk_policy)
        self.stop_stand_session: StateDrivenX2StandSession | None = None
        self.velocity_filter = X2VelocityCommandFilter()
        self.heading_hold = X2HeadingHoldController()
        self.cross_track_hold = X2CrossTrackHoldController()
        self.velocity_calibration = X2VelocityCommandCalibration()
        self.position_target_safety = X2PositionTargetSafetyEnvelope(
            self.joint_limits
        )
        # The release controller is initialized on a zero command before the
        # first walking request.  Seed the same state explicitly.
        self.velocity_filter.update((0.0, 0.0, 0.0), 0.02)
        self.next_stand_time: float | None = None
        self.next_walk_time: float | None = None
        self.stand_sequence = 0
        self.walk_sequence = 0
        self.walk_session_reset = False
        self.current_target: dict[str, float] = {}
        self.current_kp: dict[str, float] = {}
        self.current_kd: dict[str, float] = {}
        self.last_effective_command = (0.0, 0.0, 0.0)
        self.last_cross_track_m = 0.0
        self.last_cross_track_command_y_mps = 0.0
        self.last_heading_hold_command_yaw_rps = 0.0
        self.last_policy_action = (0.0,) * len(X2_CPGWALK_JOINT_NAMES)
        self.walk_base_records: list[dict] = []
        self.walk_contact_records: list[dict] = []
        self.minimum_z = float("inf")
        self.maximum_tilt = 0.0
        self.maximum_nonfoot_force = 0.0
        self.joint_limit_violation_count = 0
        self.joint_velocity_limit_violation_count = 0
        self.hard_joint_velocity_limit_violation_count = 0
        self.current_joint_velocity_exceedance_steps = 0
        self.maximum_joint_velocity_exceedance_steps = 0
        self.position_target_limit_violation_count = 0
        self.maximum_joint_limit_excess_rad = 0.0
        self.maximum_joint_velocity_ratio = 0.0
        self.maximum_position_target_limit_excess_rad = 0.0
        self.nonfinite_sample_count = 0
        self.freeze_probe: dict[str, object] | None = None
        self.walk_policy_sim_times: list[float] = []
        self.walk_policy_source_sequences: list[int] = []

    def _phase(self, elapsed: float) -> str:
        return self.profile.phase(elapsed)

    def _stand_update(
        self,
        *,
        now: float,
        q_all: tuple[float, ...],
        dq_all: tuple[float, ...],
        pose: tuple[float, ...],
        twist: tuple[float, ...],
        stop: bool,
        phase: str,
    ) -> None:
        if stop and self.stop_stand_session is None:
            self.stop_stand_session = StateDrivenX2StandSession(self.policy)
            self.next_stand_time = now
        session = self.stop_stand_session if stop else self.session
        assert session is not None
        if now + 1e-9 < float(self.next_stand_time):
            return
        self.stand_sequence += 1
        q = tuple(q_all[self.joint_index[name]] for name in X2_STAND_JOINT_NAMES)
        dq = tuple(dq_all[self.joint_index[name]] for name in X2_STAND_JOINT_NAMES)
        state = X2StandPhysicsState(
            source_sequence=self.stand_sequence,
            sim_time=now,
            joint_position=tuple(
                value - default for value, default in zip(q, _OBSERVATION_DEFAULT)
            ),
            joint_velocity=tuple(value * 0.05 for value in dq),
            base_angular_velocity=tuple(twist[3:6]),
            observation_gravity=_project_gravity_with_vendor_pitch_offset(
                tuple(pose[3:7])
            ),
            velocity_command=(0.0, 0.0, 0.0),
            pose_command=(0.0, 0.0, 0.0, 0.0),
            arm_target_position=X2_STAND_ARM_TARGET,
            waist_action_observation=(0.0, 0.0),
        )
        command = session.update(state).command
        limited = self.position_target_safety.apply(
            command.joint_names,
            command.position_target,
            sim_time=now,
            phase=phase,
            policy_sequence=self.stand_sequence,
        )
        self.current_target.update(
            zip(command.joint_names, limited.applied_position_target)
        )
        self.current_kp.update(zip(command.joint_names, command.kp))
        self.current_kd.update(zip(command.joint_names, command.kd))
        while float(self.next_stand_time) <= now + 1e-9:
            self.next_stand_time = float(self.next_stand_time) + 0.01

    def _walk_update(
        self,
        *,
        now: float,
        q_all: tuple[float, ...],
        dq_all: tuple[float, ...],
        pose: tuple[float, ...],
        twist: tuple[float, ...],
        desired_command: tuple[float, float, float],
        phase: str,
    ) -> None:
        if now + 1e-9 < float(self.next_walk_time):
            return
        if not self.walk_session_reset:
            self.walk_session.reset()
            self.walk_session_reset = True
            initial_yaw = quaternion_rpy_wxyz(pose[3:7])[2]
            self.heading_hold.reset(initial_yaw)
            self.cross_track_hold.reset(pose[:2], initial_yaw)
        roll, pitch, yaw = quaternion_rpy_wxyz(pose[3:7])
        requested_yaw_rate = desired_command[2]
        requested_lateral_speed = desired_command[1]
        apply_cross_track_stabilization = False
        apply_heading_stabilization = False
        # Heading hold is a straight-line outer loop only.  Applying it while
        # a non-zero turn is requested silently replaces the vendor yaw input.
        if abs(desired_command[0]) > 0.10 and abs(desired_command[2]) <= 1e-9:
            self.last_heading_hold_command_yaw_rps = self.heading_hold.update(
                yaw, twist[5]
            )
            apply_heading_stabilization = True
            if abs(desired_command[1]) <= 1e-9:
                (
                    self.last_cross_track_m,
                    self.last_cross_track_command_y_mps,
                ) = self.cross_track_hold.update(pose[:2], twist[1])
                apply_cross_track_stabilization = True
        else:
            self.last_cross_track_m = 0.0
            self.last_cross_track_command_y_mps = 0.0
            self.last_heading_hold_command_yaw_rps = 0.0
        calibrated = self.velocity_calibration.apply((
                desired_command[0],
                requested_lateral_speed,
                requested_yaw_rate,
        ))
        filtered = self.velocity_filter.update(
            calibrated,
            0.02,
        )
        effective_command = filtered.effective
        if apply_cross_track_stabilization:
            effective_command = apply_post_filter_lateral_stabilization(
                effective_command,
                self.last_cross_track_command_y_mps,
                maximum_lateral_speed_mps=(
                    self.cross_track_hold.params.maximum_lateral_speed_mps
                ),
            )
        if apply_heading_stabilization:
            effective_command = apply_post_filter_yaw_stabilization(
                effective_command,
                self.last_heading_hold_command_yaw_rps,
                maximum_yaw_rate_rps=(
                    self.heading_hold.params.maximum_yaw_rate_rps
                ),
            )
        self.last_effective_command = effective_command
        self.walk_sequence += 1
        self.walk_policy_sim_times.append(now)
        self.walk_policy_source_sequences.append(self.physics_sequence)
        state = X2PhysicsState(
            source_sequence=self.physics_sequence,
            sim_time=now,
            command_stamp_sim_time=now,
            velocity_command=effective_command,
            joint_position=tuple(
                q_all[self.articulation_joint_index[name]]
                for name in X2_CPGWALK_JOINT_NAMES
            ),
            joint_velocity=tuple(
                dq_all[self.articulation_joint_index[name]]
                for name in X2_CPGWALK_JOINT_NAMES
            ),
            base_angular_velocity=tuple(twist[3:6]),
            observation_gravity=x2_cpgwalk_attitude_observation(roll, pitch),
            base_linear_velocity=tuple(twist[:3]),
            roll=roll,
            pitch=pitch,
        )
        result = self.walk_session.update(state)
        self.last_policy_action = result.policy_action
        if result.joint_command is None:
            if result.status not in {"SAFE_HOLD", "STAND_HANDOFF_REQUEST"}:
                raise RuntimeError(
                    "locomotion command missing outside the expected stand hold"
                )
            command = result.policy_joint_candidate
        else:
            command = result.joint_command
        limited = self.position_target_safety.apply(
            X2_CPGWALK_JOINT_NAMES,
            command.position_target,
            sim_time=now,
            phase=phase,
            policy_sequence=self.walk_sequence,
        )
        self.current_target.update(
            zip(X2_CPGWALK_JOINT_NAMES, limited.applied_position_target)
        )
        self.current_kp.update(zip(X2_CPGWALK_JOINT_NAMES, command.kp))
        self.current_kd.update(zip(X2_CPGWALK_JOINT_NAMES, command.kd))
        while float(self.next_walk_time) <= now + 1e-9:
            self.next_walk_time = float(self.next_walk_time) + 0.02

    def _control_tuple(self) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
        return (
            tuple(self.current_target[name] for name in X2_STAND_JOINT_NAMES),
            tuple(self.current_kp[name] for name in X2_STAND_JOINT_NAMES),
            tuple(self.current_kd[name] for name in X2_STAND_JOINT_NAMES),
        )

    def _write_trace(self) -> None:
        _atomic_json(
            self.trace,
            {
                "schema": "sceneforge/x2-physx-locomotion-samples/v2",
                "physics_authority": "genie_sim_isaac_physx",
                "joint_names": list(self.articulation_joint_names),
                "physics_samples": self.trace_records,
                "base_samples": self.base_records,
                "contact_samples": self.contact_records,
                "commands": self.command_records,
                "position_target_safety_envelope": (
                    self.position_target_safety.snapshot()
                ),
                "real_robot_commands": False,
            },
        )

    def _result(self, requested_status: str) -> dict:
        metric_errors: list[str] = []

        def optional_metric(label: str, factory):
            try:
                return factory()
            except (IndexError, KeyError, TypeError, ValueError) as error:
                if requested_status == "PASSED":
                    raise
                metric_errors.append(f"{label}: {type(error).__name__}: {error}")
                return None

        final = self.base_records[-1] if self.base_records else None
        start = self.base_records[0]["position_m"] if self.base_records else [0.0, 0.0, 0.0]
        end = final["position_m"] if final else [0.0, 0.0, 0.0]
        band = json.loads(self.band_status.read_text(encoding="utf-8"))
        walk_contacts = [r for r in self.contact_records if r["phase"] == "GAIT1_FORWARD"]
        left = [r["feet"]["left_ankle_roll_link"] for r in walk_contacts]
        right = [r["feet"]["right_ankle_roll_link"] for r in walk_contacts]
        left_slip = summarize_support_slip([
            {
                "sim_time": record["sim_time"],
                "contact": foot["contact"],
                "normal_force_n": foot["net_force_n"][2],
                "tangential_slip_speed_mps": foot["tangential_slip_speed_mps"],
            }
            for record, foot in zip(walk_contacts, left)
        ])
        right_slip = summarize_support_slip([
            {
                "sim_time": record["sim_time"],
                "contact": foot["contact"],
                "normal_force_n": foot["net_force_n"][2],
                "tangential_slip_speed_mps": foot["tangential_slip_speed_mps"],
            }
            for record, foot in zip(walk_contacts, right)
        ])
        walk_bases = [r for r in self.base_records if r["phase"] == "GAIT1_FORWARD"]
        walk_start = walk_bases[0] if walk_bases else None
        walk_end = walk_bases[-1] if walk_bases else None
        walk_forward = (
            float(walk_end["position_m"][0]) - float(walk_start["position_m"][0])
            if walk_start and walk_end else 0.0
        )
        yaw_drift = (
            math.atan2(
                math.sin(float(walk_end["base_yaw_rad"]) - float(walk_start["base_yaw_rad"])),
                math.cos(float(walk_end["base_yaw_rad"]) - float(walk_start["base_yaw_rad"])),
            ) if walk_start and walk_end else 0.0
        )
        pose_speed = None
        pose_turn = None
        pose_turn_full = None
        pose_displacement = None
        steady_forward_speed = None
        steady_yaw_rate = None
        if len(walk_bases) >= 2:
            common_pose_rows = [
                {
                    "sim_time": row["sim_time"], "phase": row["phase"],
                    "position_m": row["position_m"], "yaw_rad": row["base_yaw_rad"],
                }
                for row in self.base_records
            ]
            pose_speed = optional_metric(
                "steady_forward_speed",
                lambda: pose_derived_forward_speed(common_pose_rows),
            )
            pose_turn = optional_metric(
                "steady_turn_rate",
                lambda: pose_derived_turn_rate(common_pose_rows),
            )
            pose_turn_full = optional_metric(
                "full_turn_angle",
                lambda: pose_derived_turn_rate(
                    common_pose_rows, trim_start_s=0.0, trim_end_s=0.0
                ),
            )
            pose_displacement = optional_metric(
                "walk_displacement",
                lambda: pose_derived_local_displacement(common_pose_rows),
            )
            steady_forward_speed = (
                float(pose_speed["mean_forward_speed_mps"])
                if pose_speed is not None else None
            )
            steady_yaw_rate = (
                float(pose_turn["mean_yaw_rate_rps"])
                if pose_turn is not None else None
            )
        stop_bases = [
            row for row in self.base_records
            if row["phase"] in {"DECELERATE", "CPG_STAND_HANDOFF", "STOP_STAND"}
        ]
        stop_distance = None
        if stop_bases:
            stop_start = stop_bases[0]["position_m"]
            stop_end = stop_bases[-1]["position_m"]
            stop_distance = math.hypot(
                float(stop_end[0]) - float(stop_start[0]),
                float(stop_end[1]) - float(stop_start[1]),
            )
        stable_stop = optional_metric(
            "stable_stop",
            lambda: summarize_stable_stop([
                {
                    "sim_time": row["sim_time"], "phase": row["phase"],
                    "position_m": row["position_m"],
                    "linear_speed_mps": row["linear_speed_mps"],
                    "angular_speed_rps": row["angular_speed_rad_s"],
                }
                for row in self.base_records
            ]),
        )
        command_ages = [
            float(row["command_age_sim_time_s"])
            for row in self.command_records
            if "command_age_sim_time_s" in row
            and math.isfinite(float(row["command_age_sim_time_s"]))
        ]
        target_safety = self.position_target_safety.snapshot()
        saturation = (
            self.effort_saturation_count / self.effort_sample_count
            if self.effort_sample_count else 1.0
        )
        checks = {
            "completed_profile": requested_status == "PASSED" and self.last_error is None,
            "floating_base": True,
            "minimum_base_height": self.minimum_z >= 0.55,
            "maximum_base_tilt": self.maximum_tilt <= 0.35,
            "band_released": band.get("state") == "RELEASED",
            "band_support_zero": band.get("support_ratio") == 0.0,
            "forward_displacement_observed": (
                pose_displacement is not None
                and float(pose_displacement["body_forward_distance_m"]) >= 0.50
            ),
            "straight_local_cross_track": (
                abs(self.profile.command_yaw_rps) > 1e-9
                or pose_displacement is not None
                and float(pose_displacement["local_cross_track_per_forward_m"]) <= 0.05
            ),
            "turn_direction_observed": (
                abs(self.profile.command_yaw_rps) <= 1e-9
                or steady_yaw_rate is not None
                and steady_yaw_rate * self.profile.command_yaw_rps > 0.0
            ),
            "turn_angle_accuracy": (
                abs(self.profile.command_yaw_rps) <= 1e-9
                or pose_turn_full is not None
                and abs(
                    float(pose_turn_full["yaw_change_rad"])
                    - self.profile.command_yaw_rps * self.profile.walk_s
                ) <= max(0.1, 0.10 * abs(
                    self.profile.command_yaw_rps * self.profile.walk_s
                ))
            ),
            "final_linear_speed": final is not None and final["linear_speed_mps"] <= 0.05,
            "final_angular_speed": final is not None and final["angular_speed_rad_s"] <= 0.05,
            "sustained_stable_stop": (
                stable_stop is not None
                and float(stable_stop["linear_speed_p95_mps"]) <= 0.05
                and float(stable_stop["angular_speed_p95_rps"]) <= 0.05
                and float(stable_stop["maximum_linear_speed_mps"]) <= 0.05
                and float(stable_stop["maximum_angular_speed_rps"]) <= 0.05
                and float(stable_stop["planar_drift_m"]) <= 0.03
            ),
            "both_feet_contact_observed": bool(left) and any(x["contact"] for x in left) and any(x["contact"] for x in right),
            "zero_nonfoot_contact": self.maximum_nonfoot_force <= 20.0,
            "effort_saturation_ratio": saturation <= 0.01,
            "zero_joint_limit_violations": self.joint_limit_violation_count == 0,
            "full_joint_state_and_target_within_limits": (
                self.joint_limit_violation_count == 0
                and self.position_target_limit_violation_count == 0
            ),
            "bounded_policy_position_target_saturation": (
                target_safety["bounded"] is True
            ),
            "joint_velocity_within_numeric_tolerance": (
                self.hard_joint_velocity_limit_violation_count == 0
                and self.maximum_joint_velocity_ratio
                <= X2_JOINT_VELOCITY_NUMERIC_TOLERANCE_RATIO
            ),
            "no_sustained_joint_velocity_limit_violation": (
                self.maximum_joint_velocity_exceedance_steps * self.physics_dt_s <= 0.010
            ),
            "no_nan_or_inf": self.nonfinite_sample_count == 0,
            "no_fall": self.minimum_z >= 0.55 and self.maximum_tilt <= 0.35,
            "one_command_per_physics_state": (
                len(self.command_records) == len(self.trace_records)
                and [int(row["physics_sequence"]) for row in self.trace_records]
                == list(range(1, len(self.trace_records) + 1))
                and [int(row["command_sequence"]) for row in self.command_records]
                == list(range(1, len(self.command_records) + 1))
                and all(
                    int(command["source_physics_sequence"]) == int(state["physics_sequence"])
                    for command, state in zip(self.command_records, self.trace_records)
                )
            ),
            "bounded_command_execution_age_sim_time": all(
                0.0 < float(command["command_age_sim_time_s"]) <= 0.004
                for command in self.command_records
                if "command_age_sim_time_s" in command
            ) and len(command_ages) == len(self.command_records),
            "walk_policy_strict_sim_time": (
                len(self.walk_policy_sim_times) > 1
                and all(after > before for before, after in zip(
                    self.walk_policy_sim_times, self.walk_policy_sim_times[1:]
                ))
                and all(after > before for before, after in zip(
                    self.walk_policy_source_sequences, self.walk_policy_source_sequences[1:]
                ))
            ),
            "sim_time_freeze_probe": (
                self.freeze_probe is not None
                and self.freeze_probe["controller_state_unchanged"] is True
                and self.freeze_probe["physics_sequence_unchanged"] is True
                and self.freeze_probe["command_sequence_unchanged"] is True
                and self.freeze_probe["sim_time_unchanged"] is True
            ),
            "real_robot_commands_false": True,
        }
        non_blocking_reference_checks = (
            "joint_velocity_within_numeric_tolerance",
            "no_sustained_joint_velocity_limit_violation",
        )
        passed = all(
            value for name, value in checks.items()
            if name not in non_blocking_reference_checks
        )
        return {
            "schema": SCHEMA,
            "status": "PASSED" if passed else "FAILED",
            "decision": "compare_mujoco_physx_gait1" if passed else "debug_physx_gait1",
            "qualification": False,
            "physics_authority": "genie_sim_isaac_physx",
            "base_mode": "floating",
            "controller": "independent_state_driven_x2_onnx_session",
            "root_pinned": False,
            "gravity_modified": False,
            "published_command_topics": [],
            "real_robot_commands": False,
            "profile": self.profile.__dict__,
            "velocity_command_calibration": self.velocity_calibration.__dict__,
            "position_target_safety_envelope": target_safety,
            "parameters": {
                "stand_policy_rate_hz": 100.0,
                "walk_policy_rate_hz": 50.0,
                "pd_rate_hz": 500.0,
                "freeze_probe_wall_s": self.freeze_probe_wall_s,
                "articulation_joint_count": len(self.articulation_joint_names),
                "physx_contact_report_threshold_n": 0.0,
                "base_state_source": self.base_state_source,
                "joint_limit_source": self.joint_limit_source,
                "joint_limit_source_sha256": _sha256(Path(self.joint_limit_source)),
                "joint_position_limits_rad": {
                    name: list(self.joint_limits[name])
                    for name in self.articulation_joint_names
                },
                "joint_velocity_limits_rps": self.joint_velocity_limits,
                "joint_velocity_numeric_tolerance_ratio": (
                    X2_JOINT_VELOCITY_NUMERIC_TOLERANCE_RATIO
                ),
            },
            "checks": checks,
            "non_blocking_reference_checks": list(non_blocking_reference_checks),
            "last_error": self.last_error,
            "metric_errors": metric_errors,
            "elastic_band_status": band,
            "model": {
                "stand_path": str(self.model),
                "stand_sha256": _sha256(self.model),
                "walk_path": str(self.walk_model),
                "walk_sha256": _sha256(self.walk_model),
            },
            "metrics": {
                "physics_step_count": self.physics_sequence,
                "stand_policy_call_count": len(self.policy.elapsed_ms),
                "walk_policy_call_count": len(self.walk_policy.elapsed_ms),
                "start_position_m": start,
                "end_position_m": end,
                "forward_displacement_m": end[0] - start[0],
                "lateral_displacement_m": end[1] - start[1],
                "minimum_base_height_m": self.minimum_z,
                "maximum_tilt_rad": self.maximum_tilt,
                "final_linear_speed_mps": final["linear_speed_mps"] if final else None,
                "final_angular_speed_rps": final["angular_speed_rad_s"] if final else None,
                "left_walk_contact_fraction": sum(bool(x["contact"]) for x in left) / len(left) if left else None,
                "right_walk_contact_fraction": sum(bool(x["contact"]) for x in right) / len(right) if right else None,
                "left_support_slip_p95_mps": left_slip["raw_contact_slip_p95_mps"],
                "right_support_slip_p95_mps": right_slip["raw_contact_slip_p95_mps"],
                "left_raw_contact_slip_p95_mps": left_slip["raw_contact_slip_p95_mps"],
                "right_raw_contact_slip_p95_mps": right_slip["raw_contact_slip_p95_mps"],
                "left_steady_support_slip_p95_mps": left_slip["steady_support_slip_p95_mps"],
                "right_steady_support_slip_p95_mps": right_slip["steady_support_slip_p95_mps"],
                "left_support_slip_summary": left_slip,
                "right_support_slip_summary": right_slip,
                "walk_start_yaw_rad": walk_start["base_yaw_rad"] if walk_start else None,
                "walk_end_yaw_rad": walk_end["base_yaw_rad"] if walk_end else None,
                "walk_yaw_drift_rad": yaw_drift,
                "walk_yaw_drift_rad_per_forward_m": abs(yaw_drift) / max(abs(walk_forward), 1e-9),
                "steady_forward_speed_mps": steady_forward_speed,
                "steady_forward_speed_relative_error": (
                    abs(float(steady_forward_speed) - self.profile.command_x_mps)
                    / max(abs(self.profile.command_x_mps), 1e-9)
                    if steady_forward_speed is not None else None
                ),
                "steady_forward_speed_sample_count": pose_speed["sample_count"] if pose_speed else 0,
                "steady_forward_speed_estimator": pose_speed,
                "walk_body_forward_distance_m": (
                    pose_displacement["body_forward_distance_m"]
                    if pose_displacement else None
                ),
                "walk_local_forward_displacement_m": (
                    pose_displacement["local_forward_displacement_m"]
                    if pose_displacement else None
                ),
                "walk_local_cross_track_m": (
                    pose_displacement["local_cross_track_m"]
                    if pose_displacement else None
                ),
                "walk_local_cross_track_per_forward_m": (
                    pose_displacement["local_cross_track_per_forward_m"]
                    if pose_displacement else None
                ),
                "walk_displacement_estimator": pose_displacement,
                "walk_turn_angle_rad": (
                    pose_turn_full["yaw_change_rad"] if pose_turn_full else None
                ),
                "steady_yaw_rate_rps": steady_yaw_rate,
                "steady_yaw_rate_relative_error": (
                    abs(float(steady_yaw_rate) - self.profile.command_yaw_rps)
                    / max(abs(self.profile.command_yaw_rps), 1e-9)
                    if steady_yaw_rate is not None and abs(self.profile.command_yaw_rps) > 1e-9
                    else 0.0
                ),
                "steady_yaw_rate_estimator": pose_turn,
                "stop_distance_xy_m": stop_distance,
                "stable_stop": stable_stop,
                "maximum_nonfoot_contact_force_n": self.maximum_nonfoot_force,
                "effort_saturation_ratio": saturation,
                "maximum_raw_effort_ratio": self.maximum_raw_effort_ratio,
                "joint_limit_violation_count": self.joint_limit_violation_count,
                "maximum_joint_limit_excess_rad": self.maximum_joint_limit_excess_rad,
                "joint_velocity_limit_violation_count": self.joint_velocity_limit_violation_count,
                "hard_joint_velocity_limit_violation_count": (
                    self.hard_joint_velocity_limit_violation_count
                ),
                "maximum_joint_velocity_ratio": self.maximum_joint_velocity_ratio,
                "maximum_joint_velocity_exceedance_duration_s": (
                    self.maximum_joint_velocity_exceedance_steps * self.physics_dt_s
                ),
                "position_target_limit_violation_count": self.position_target_limit_violation_count,
                "maximum_position_target_limit_excess_rad": (
                    self.maximum_position_target_limit_excess_rad
                ),
                "raw_policy_position_target_sample_count": (
                    target_safety["raw_target_sample_count"]
                ),
                "raw_policy_position_target_saturation_count": (
                    target_safety["saturation_count"]
                ),
                "raw_policy_position_target_saturation_ratio": (
                    target_safety["saturation_ratio"]
                ),
                "maximum_raw_policy_position_target_limit_excess_rad": (
                    target_safety["observed_maximum_raw_limit_excess_rad"]
                ),
                "nonfinite_sample_count": self.nonfinite_sample_count,
                "fall_event_count": int(
                    self.minimum_z < 0.55 or self.maximum_tilt > 0.35
                ),
                "physics_state_record_count": len(self.trace_records),
                "command_record_count": len(self.command_records),
                "maximum_command_age_sim_time_s": max(
                    command_ages,
                    default=None,
                ),
                "command_age_sim_time_p50_s": _percentile(
                    command_ages,
                    0.50,
                ),
                "command_age_sim_time_p95_s": _percentile(
                    command_ages,
                    0.95,
                ),
                "walk_policy_sim_time_start_s": (
                    self.walk_policy_sim_times[0] if self.walk_policy_sim_times else None
                ),
                "walk_policy_sim_time_end_s": (
                    self.walk_policy_sim_times[-1] if self.walk_policy_sim_times else None
                ),
                "walk_policy_update_count": len(self.walk_policy_sim_times),
                "sim_time_freeze_probe": self.freeze_probe,
            },
            "trace": {"path": str(self.trace), "sha256": _sha256(self.trace)},
            "limitations": [
                "This is a single development A/B sample, not locomotion qualification.",
                "Ground-truth base odometry is used; navigation and rendering are not involved.",
                "URDF joint velocity is compared against the vendor MuJoCo reference as a nominal value; strict exceedance evidence remains visible and unresolved for qualification.",
                "Raw stand/walk policy position targets are passed through the same auditable URDF-limit safety envelope used by the MuJoCo reference; every clamp remains in the immutable receipt and trace.",
            ],
        }

    def _finish(self, status: str, error: str | None = None) -> None:
        """Finalize exactly once and always request a controlled engine stop."""

        if self.completed:
            return
        # Set this before any filesystem work so a finalization exception can
        # never re-enter on the next physics callback and overwrite evidence.
        self.completed = True
        self.last_error = error
        try:
            self._write_trace()
            result = self._result(status)
            _atomic_json(self.receipt, result)
            self.status = result["status"]
        except Exception as finish_error:
            detail = (
                f"finalization failed: {type(finish_error).__name__}: {finish_error}"
            )
            self.last_error = (
                f"{self.last_error}; {detail}" if self.last_error else detail
            )
            self.status = "FAILED"
            if not self.receipt.exists():
                _atomic_json(self.receipt, {
                    "schema": SCHEMA,
                    "status": "FAILED",
                    "decision": "debug_physx_gait1_finalization",
                    "qualification": False,
                    "physics_authority": "genie_sim_isaac_physx",
                    "checks": {"result_finalized": False},
                    "last_error": self.last_error,
                    "trace": {
                        "path": str(self.trace),
                        "sha256": _sha256(self.trace) if self.trace.is_file() else None,
                    },
                    "published_command_topics": [],
                    "real_robot_commands": False,
                })
        finally:
            self._request_stop()

    def step(self, sim_time: float) -> None:
        if self.completed:
            return
        try:
            self.physics_sequence += 1
            now = float(sim_time)
            if not math.isfinite(now) or now < 0.0:
                raise ValueError("post-step simulation time is invalid")
            if self.last_sim_time is not None and now <= self.last_sim_time:
                raise ValueError("post-step simulation time did not advance")
            state = self.state_reader(now)
            if state is None:
                raise RuntimeError("floating base state is unavailable")
            pose7, twist6 = state
            pose = tuple(float(value) for value in pose7)
            twist = tuple(float(value) for value in twist6)
            q_raw, dq_raw = self.sim.get_joint_states()
            q_all = tuple(float(value) for value in q_raw)
            dq_all = tuple(float(value) for value in dq_raw)
            if not all(math.isfinite(v) for v in (*pose, *twist, *q_all, *dq_all)):
                self.nonfinite_sample_count += 1
                raise ValueError("PhysX state contains NaN or Inf")
            velocity_exceeded_this_step = False
            for name, value, velocity in zip(
                self.articulation_joint_names, q_all, dq_all
            ):
                lower, upper = self.joint_limits[name]
                excess = max(lower - value, value - upper, 0.0)
                self.maximum_joint_limit_excess_rad = max(
                    self.maximum_joint_limit_excess_rad, excess
                )
                if excess > 1e-4:
                    self.joint_limit_violation_count += 1
                    raise RuntimeError(
                        f"PhysX joint limit crossed: {name} value={value:.6f} "
                        f"range=[{lower:.6f},{upper:.6f}]"
                    )
                velocity_ratio = abs(velocity) / self.joint_velocity_limits[name]
                self.maximum_joint_velocity_ratio = max(
                    self.maximum_joint_velocity_ratio, velocity_ratio
                )
                if velocity_ratio > 1.0:
                    self.joint_velocity_limit_violation_count += 1
                    velocity_exceeded_this_step = True
                if velocity_ratio > X2_JOINT_VELOCITY_NUMERIC_TOLERANCE_RATIO:
                    self.hard_joint_velocity_limit_violation_count += 1
            if velocity_exceeded_this_step:
                self.current_joint_velocity_exceedance_steps += 1
                self.maximum_joint_velocity_exceedance_steps = max(
                    self.maximum_joint_velocity_exceedance_steps,
                    self.current_joint_velocity_exceedance_steps,
                )
            else:
                self.current_joint_velocity_exceedance_steps = 0
            if self.first_sim_time is None:
                self.first_sim_time = now
                self.next_stand_time = now
                self.next_walk_time = now + self.profile.walk_start_s
                self.baseline_all = q_all
                self.baseline = tuple(q_all[self.joint_index[n]] for n in X2_STAND_JOINT_NAMES)
                self.current_target = {n: q_all[self.articulation_joint_index[n]] for n in self.articulation_joint_names}
                self.current_kp = {n: 10.0 for n in self.articulation_joint_names}
                self.current_kd = {n: 1.0 for n in self.articulation_joint_names}
            elapsed = now - float(self.first_sim_time)
            phase = self.profile.phase(elapsed)
            roll, pitch, _ = quaternion_rpy_wxyz(pose[3:7])
            tilt = math.hypot(roll, pitch)
            self.minimum_z = min(self.minimum_z, pose[2])
            self.maximum_tilt = max(self.maximum_tilt, tilt)
            if phase == "COMPLETE":
                self.last_sim_time = now
                self._finish("PASSED")
                return
            if phase == "BAND_RAMP_DOWN":
                self._request_ramp_down()
            if phase in {"SUPPORTED_STAND", "BAND_RAMP_DOWN", "UNSUPPORTED_STAND"}:
                self._stand_update(
                    now=now, q_all=q_all, dq_all=dq_all, pose=pose,
                    twist=twist, stop=False, phase=phase,
                )
            elif phase in {"GAIT1_FORWARD", "DECELERATE", "CPG_STAND_HANDOFF"}:
                band = json.loads(self.band_status.read_text(encoding="utf-8"))
                if (
                    band.get("state") != "RELEASED"
                    or band.get("support_ratio") != 0.0
                    or band.get("force_norm_n") != 0.0
                ):
                    raise RuntimeError(
                        "locomotion forbidden until the elastic band is authoritatively released"
                    )
                self._walk_update(
                    now=now, q_all=q_all, dq_all=dq_all, pose=pose, twist=twist,
                    desired_command=self.profile.velocity_command(elapsed),
                    phase=phase,
                )
            elif phase == "STOP_STAND":
                self._stand_update(
                    now=now, q_all=q_all, dq_all=dq_all, pose=pose,
                    twist=twist, stop=True, phase=phase,
                )
            target, kp, kd = self._control_tuple()
            self._apply(target, q_all, dq_all, kp, kd, phase, now)
            command = self.command_records[-1]
            for name, desired in zip(
                self.articulation_joint_names,
                command["articulation_position_target"],
            ):
                lower, upper = self.joint_limits[name]
                excess = max(lower - float(desired), float(desired) - upper, 0.0)
                self.maximum_position_target_limit_excess_rad = max(
                    self.maximum_position_target_limit_excess_rad, excess
                )
                if excess > 1e-4:
                    self.position_target_limit_violation_count += 1
                    raise RuntimeError(
                        f"PhysX position target crossed joint limit: {name} "
                        f"target={float(desired):.6f} range=[{lower:.6f},{upper:.6f}]"
                    )
            if (
                self.freeze_probe is None
                and phase == "GAIT1_FORWARD"
                and elapsed >= self.profile.walk_start_s + 3.0
            ):
                adapter_state_before = {
                    "walk_session_digest": self.walk_session.state_digest(),
                    "velocity_filter": self.velocity_filter.snapshot(),
                    "heading_hold": self.heading_hold.snapshot(),
                    "cross_track_hold": self.cross_track_hold.snapshot(),
                    "position_target_safety": self.position_target_safety.snapshot(),
                    "next_walk_time": self.next_walk_time,
                    "walk_sequence": self.walk_sequence,
                    "walk_session_reset": self.walk_session_reset,
                    "current_target": dict(self.current_target),
                    "current_kp": dict(self.current_kp),
                    "current_kd": dict(self.current_kd),
                    "last_policy_action": list(self.last_policy_action),
                    "walk_policy_call_count": len(self.walk_policy.elapsed_ms),
                    "trace_record_count": len(self.trace_records),
                    "base_record_count": len(self.base_records),
                    "contact_record_count": len(self.contact_records),
                    "last_effective_command": list(self.last_effective_command),
                    "last_cross_track_m": self.last_cross_track_m,
                    "last_cross_track_command_y_mps": (
                        self.last_cross_track_command_y_mps
                    ),
                    "last_heading_hold_command_yaw_rps": (
                        self.last_heading_hold_command_yaw_rps
                    ),
                }
                physics_before = self.physics_sequence
                command_before = self.command_sequence
                sim_time_before = now
                wall_started = time.perf_counter()
                time.sleep(self.freeze_probe_wall_s)
                wall_elapsed = time.perf_counter() - wall_started
                adapter_state_after = {
                    "walk_session_digest": self.walk_session.state_digest(),
                    "velocity_filter": self.velocity_filter.snapshot(),
                    "heading_hold": self.heading_hold.snapshot(),
                    "cross_track_hold": self.cross_track_hold.snapshot(),
                    "position_target_safety": self.position_target_safety.snapshot(),
                    "next_walk_time": self.next_walk_time,
                    "walk_sequence": self.walk_sequence,
                    "walk_session_reset": self.walk_session_reset,
                    "current_target": dict(self.current_target),
                    "current_kp": dict(self.current_kp),
                    "current_kd": dict(self.current_kd),
                    "last_policy_action": list(self.last_policy_action),
                    "walk_policy_call_count": len(self.walk_policy.elapsed_ms),
                    "trace_record_count": len(self.trace_records),
                    "base_record_count": len(self.base_records),
                    "contact_record_count": len(self.contact_records),
                    "last_effective_command": list(self.last_effective_command),
                    "last_cross_track_m": self.last_cross_track_m,
                    "last_cross_track_command_y_mps": (
                        self.last_cross_track_command_y_mps
                    ),
                    "last_heading_hold_command_yaw_rps": (
                        self.last_heading_hold_command_yaw_rps
                    ),
                }
                self.freeze_probe = {
                    "method": "engine_thread_wall_sleep_without_physics_step",
                    "requested_wall_pause_s": self.freeze_probe_wall_s,
                    "observed_wall_pause_s": wall_elapsed,
                    "sim_time_before_s": sim_time_before,
                    "sim_time_after_s": now,
                    "physics_sequence_before": physics_before,
                    "physics_sequence_after": self.physics_sequence,
                    "command_sequence_before": command_before,
                    "command_sequence_after": self.command_sequence,
                    "controller_digest_before": adapter_state_before["walk_session_digest"],
                    "controller_digest_after": adapter_state_after["walk_session_digest"],
                    "adapter_state_before": adapter_state_before,
                    "adapter_state_after": adapter_state_after,
                    "controller_state_unchanged": (
                        adapter_state_before == adapter_state_after
                    ),
                    "physics_sequence_unchanged": physics_before == self.physics_sequence,
                    "command_sequence_unchanged": command_before == self.command_sequence,
                    "sim_time_unchanged": sim_time_before == now,
                }
                if not all((
                    adapter_state_before == adapter_state_after,
                    physics_before == self.physics_sequence,
                    command_before == self.command_sequence,
                    sim_time_before == now,
                    wall_elapsed >= 0.90 * self.freeze_probe_wall_s,
                )):
                    raise RuntimeError("sim-time freeze probe advanced controller state")
            # The command is derived from this post-step state and becomes the
            # zero-order-held input to the next PhysX step.
            self.command_records[-1]["command_age_sim_time_s"] = self.physics_dt_s
            self.command_records[-1]["applies_to_physics_sequence"] = (
                self.physics_sequence + 1
            )
            contact = dict(self.contact_reader(now)) if self.contact_reader else {}
            if contact:
                contact["phase"] = phase
                self.contact_records.append(contact)
                self.maximum_nonfoot_force = max(
                    self.maximum_nonfoot_force,
                    float(contact["maximum_non_foot_contact_force_n"]),
                )
            hopf = self.walk_session.hopf.snapshot()
            self.base_records.append({
                "physics_sequence": self.physics_sequence,
                "sim_time": now,
                "phase": phase,
                "position_m": list(pose[:3]),
                "orientation_wxyz": list(pose[3:7]),
                "z_m": pose[2],
                "tilt_rad": tilt,
                "linear_speed_mps": math.sqrt(sum(v * v for v in twist[:3])),
                "angular_speed_rad_s": math.sqrt(sum(v * v for v in twist[3:])),
                "base_linear_velocity_body_mps": list(twist[:3]),
                "base_angular_velocity_body_rps": list(twist[3:]),
                "cpg_phase_rad": math.atan2(hopf.obs4[0], hopf.obs4[1]),
                "cpg_period_s": hopf.period,
                "active_gait": hopf.gait,
                "requested_velocity": list(self.profile.velocity_command(elapsed)),
                "effective_velocity_command": list(self.last_effective_command),
                "base_yaw_rad": quaternion_rpy_wxyz(pose[3:7])[2],
                "walk_policy_action": list(self.last_policy_action),
                "straight_cross_track_m": self.last_cross_track_m,
                "straight_cross_track_command_y_mps": (
                    self.last_cross_track_command_y_mps
                ),
                "straight_heading_hold_command_yaw_rps": (
                    self.last_heading_hold_command_yaw_rps
                ),
            })
            self.trace_records.append({
                "physics_sequence": self.physics_sequence,
                "sim_time": now,
                "phase": phase,
                "position": list(q_all),
                "velocity": list(dq_all),
                "position_target": list(command["articulation_position_target"]),
                "effort_target": list(command["effort_target"]),
            })
            self.last_all_position = q_all
            self.last_all_velocity = dq_all
            self.last_sim_time = now
            if pose[2] < 0.40 or tilt > 0.80:
                raise RuntimeError(
                    f"PhysX gait1 safety limit crossed: z={pose[2]:.4f}, tilt={tilt:.4f}"
                )
        except Exception as exc:
            text = str(exc).lower()
            if self.nonfinite_sample_count == 0 and (
                "non-finite" in text or "nan" in text or "inf" in text
            ):
                self.nonfinite_sample_count += 1
            self._finish("FAILED", f"{type(exc).__name__}: {exc}")
