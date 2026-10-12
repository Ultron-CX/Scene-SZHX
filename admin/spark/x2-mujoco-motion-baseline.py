#!/usr/bin/env python3
"""Run the independent X2 controller on the frozen vendor MuJoCo model."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import tempfile
import time
from pathlib import Path

import numpy as np

from sceneforge.x2_control import (
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
from sceneforge.x2_fixed_base_stand import (
    _OBSERVATION_DEFAULT,
    _StandOnnxPolicy,
    _effort_limit,
)
from sceneforge.x2_floating_stand import _project_gravity_with_vendor_pitch_offset
from sceneforge.x2_inprocess_shadow import _OnnxPolicy
from sceneforge.x2_mujoco_model import X2_ALL_JOINT_NAMES, inspect_x2_mujoco_model
from sceneforge.x2_mujoco_motion import (
    X2MujocoMotionProfile,
    X2_JOINT_VELOCITY_NUMERIC_TOLERANCE_RATIO,
    load_urdf_joint_limits,
    load_urdf_joint_velocity_limits,
    percentile,
    quaternion_rpy_wxyz,
)
from sceneforge.x2_locomotion_metrics import (
    pose_derived_forward_speed,
    pose_derived_local_displacement,
    pose_derived_turn_rate,
    summarize_stable_stop,
    summarize_support_slip,
)
from sceneforge.x2_session import StateDrivenX2ControllerSession, X2PhysicsState
from sceneforge.x2_stand import (
    StateDrivenX2StandSession,
    X2StandPhysicsState,
    X2_STAND_ARM_TARGET,
    X2_STAND_JOINT_NAMES,
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_json(path: Path, payload: object, mode: int = 0o444) -> None:
    if path.exists():
        raise FileExistsError(f"refusing to overwrite {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        os.chmod(path, mode)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def sensor_values(mujoco, model, data, name: str) -> tuple[float, ...]:
    sensor_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name))
    if sensor_id < 0:
        raise ValueError(f"missing MuJoCo sensor {name}")
    start = int(model.sensor_adr[sensor_id])
    width = int(model.sensor_dim[sensor_id])
    return tuple(float(value) for value in data.sensordata[start:start + width])


class X2MujocoMotionRun:
    def __init__(self, mujoco, model, data, args) -> None:
        self.mujoco = mujoco
        self.model = model
        self.data = data
        self.args = args
        self.profile = X2MujocoMotionProfile(
            supported_stand_s=args.supported_stand_s,
            band_ramp_s=args.band_ramp_s,
            unsupported_stand_s=args.unsupported_stand_s,
            walk_s=args.walk_s,
            stop_s=args.stop_s,
            command_x_mps=args.command_x_mps,
            command_y_mps=args.command_y_mps,
            command_yaw_rps=args.command_yaw_rps,
        )
        self.profile.validate()
        self.joint_id = {
            name: int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))
            for name in X2_ALL_JOINT_NAMES
        }
        self.qpos_index = {name: int(model.jnt_qposadr[jid]) for name, jid in self.joint_id.items()}
        self.dof_index = {name: int(model.jnt_dofadr[jid]) for name, jid in self.joint_id.items()}
        self.joint_limits = {}
        for name, joint_id in self.joint_id.items():
            if not bool(model.jnt_limited[joint_id]):
                raise ValueError(f"frozen MuJoCo joint lacks a finite limit: {name}")
            lower, upper = (float(value) for value in model.jnt_range[joint_id])
            if not all(math.isfinite(value) for value in (lower, upper)) or lower >= upper:
                raise ValueError(f"frozen MuJoCo joint has invalid limits: {name}")
            self.joint_limits[name] = (lower, upper)
        self.joint_limit_urdf = Path(args.joint_limit_urdf).resolve()
        if not self.joint_limit_urdf.is_file():
            raise FileNotFoundError(self.joint_limit_urdf)
        urdf_joint_limits = load_urdf_joint_limits(
            self.joint_limit_urdf, X2_ALL_JOINT_NAMES
        )
        for name in X2_ALL_JOINT_NAMES:
            if any(
                not math.isclose(model_value, urdf_value, abs_tol=1e-5)
                for model_value, urdf_value in zip(
                    self.joint_limits[name], urdf_joint_limits[name]
                )
            ):
                raise ValueError(
                    f"MuJoCo/URDF position limits differ for {name}: "
                    f"{self.joint_limits[name]} != {urdf_joint_limits[name]}"
                )
        self.joint_velocity_limits = load_urdf_joint_velocity_limits(
            self.joint_limit_urdf, X2_ALL_JOINT_NAMES
        )
        self.actuator_id = {}
        for actuator in range(model.nu):
            joint = int(model.actuator_trnid[actuator, 0])
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            if name:
                self.actuator_id[name] = actuator
        if set(self.actuator_id) != set(X2_ALL_JOINT_NAMES):
            raise ValueError("MuJoCo actuator/joint mapping differs from the frozen 31-joint contract")
        self.pelvis_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis"))
        self.torso_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso_link"))
        self.left_foot_body = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_ankle_roll_link"))
        self.right_foot_body = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_ankle_roll_link"))
        self.environment_geoms = {
            int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name))
            for name in ("floor", "ramp_collision", "chassis_block", "chassis_center_ridge")
            if int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)) >= 0
        }
        self.left_foot_geoms = self._geom_ids("left_foot")
        self.right_foot_geoms = self._geom_ids("right_foot")
        self.stand_policy = _StandOnnxPolicy(Path(args.stand_model))
        self.walk_policy = _OnnxPolicy(Path(args.walk_model))
        self.pre_stand_session = StateDrivenX2StandSession(self.stand_policy)
        self.stop_stand_session: StateDrivenX2StandSession | None = None
        self.walk_session = StateDrivenX2ControllerSession(self.walk_policy)
        self.velocity_command_filter = X2VelocityCommandFilter()
        self.heading_hold = X2HeadingHoldController()
        self.cross_track_hold = X2CrossTrackHoldController()
        self.velocity_calibration = X2VelocityCommandCalibration()
        self.position_target_safety = X2PositionTargetSafetyEnvelope(
            self.joint_limits
        )
        self.walk_session_reset_for_profile = False
        self.last_raw_velocity_command = (0.0, 0.0, 0.0)
        self.last_effective_velocity_command = (0.0, 0.0, 0.0)
        self.last_command_filter_alpha = 0.0
        self.last_walk_policy_action = (0.0,) * len(X2_CPGWALK_JOINT_NAMES)
        self.last_cross_track_m = 0.0
        self.last_cross_track_command_y_mps = 0.0
        self.last_heading_hold_command_yaw_rps = 0.0
        self.next_stand_update = 0.0
        # The vendor CPG controller remains alive in stand mode before the
        # locomotion state transition.  Warm its 10-frame history, Hopf state
        # and target filters from t=0 instead of seeding ten identical active
        # command frames at walk entry.
        self.next_walk_update = 0.0
        self.stand_sequence = 0
        self.walk_sequence = 0
        self.current_target = {name: float(data.qpos[index]) for name, index in self.qpos_index.items()}
        self.current_kp = {name: 10.0 for name in X2_ALL_JOINT_NAMES}
        self.current_kd = {name: 1.0 for name in X2_ALL_JOINT_NAMES}
        self.trace: list[dict] = []
        self.last_trace_time = -1.0
        self.minimum_z = float("inf")
        self.maximum_tilt = 0.0
        self.maximum_nonfoot_contact_force = 0.0
        self.left_contact_steps = 0
        self.right_contact_steps = 0
        self.walk_steps = 0
        self.left_slip: list[float] = []
        self.right_slip: list[float] = []
        self.left_slip_samples: list[dict] = []
        self.right_slip_samples: list[dict] = []
        self.saturation_count = 0
        self.effort_sample_count = 0
        self.maximum_effort_ratio = 0.0
        self.support_force_during_walk_max = 0.0
        self.nan_detected = False
        self.joint_limit_violation_count = 0
        self.maximum_joint_limit_excess_rad = 0.0
        self.command_application_count = 0
        # Preserve the vendor MuJoCo contract: PD is recomputed for every
        # 1 kHz physics state.  PhysX likewise uses one PD command per native
        # 500 Hz state; policy inference remains 50 Hz and sim-time driven.
        self.pd_period_s = float(self.model.opt.timestep)
        self.next_pd_update = 0.0
        self.walk_policy_sim_times: list[float] = []
        self.joint_velocity_limit_violation_count = 0
        self.hard_joint_velocity_limit_violation_count = 0
        self.current_joint_velocity_exceedance_steps = 0
        self.maximum_joint_velocity_exceedance_steps = 0
        self.position_target_limit_violation_count = 0
        self.maximum_joint_velocity_ratio = 0.0
        self.maximum_position_target_limit_excess_rad = 0.0
        self.freeze_probe_wall_s = 2.0
        self.freeze_probe: dict[str, object] | None = None
        self.failure: str | None = None
        self.started_wall = time.perf_counter()

    def _geom_ids(self, prefix: str) -> set[int]:
        result = set()
        for index in range(self.model.ngeom):
            name = self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_GEOM, index)
            if name and name.startswith(prefix):
                result.add(index)
        return result

    def q(self, names) -> tuple[float, ...]:
        return tuple(float(self.data.qpos[self.qpos_index[name]]) for name in names)

    def dq(self, names) -> tuple[float, ...]:
        return tuple(float(self.data.qvel[self.dof_index[name]]) for name in names)

    def update_stand(self, now: float, stop: bool) -> None:
        session = self.stop_stand_session if stop else self.pre_stand_session
        if stop and session is None:
            session = StateDrivenX2StandSession(self.stand_policy)
            self.stop_stand_session = session
            self.next_stand_update = now
        assert session is not None
        if now + 1e-12 < self.next_stand_update:
            return
        self.stand_sequence += 1
        q = self.q(X2_STAND_JOINT_NAMES)
        dq = self.dq(X2_STAND_JOINT_NAMES)
        state = X2StandPhysicsState(
            source_sequence=self.stand_sequence,
            sim_time=now,
            joint_position=tuple(value - default for value, default in zip(q, _OBSERVATION_DEFAULT)),
            joint_velocity=tuple(value * 0.05 for value in dq),
            base_angular_velocity=sensor_values(self.mujoco, self.model, self.data, "body-angular-velocity"),
            observation_gravity=_project_gravity_with_vendor_pitch_offset(tuple(self.data.qpos[3:7])),
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
            phase=self.profile.phase(now),
            policy_sequence=self.stand_sequence,
        )
        self.current_target.update(
            zip(command.joint_names, limited.applied_position_target)
        )
        self.current_kp.update(zip(command.joint_names, command.kp))
        self.current_kd.update(zip(command.joint_names, command.kd))
        while self.next_stand_update <= now + 1e-12:
            self.next_stand_update += 0.01

    def update_walk(self, now: float, *, apply_command: bool) -> None:
        if now + 1e-12 < self.next_walk_update:
            return
        if apply_command and not self.walk_session_reset_for_profile:
            # The deployed controller creates/restarts the CPGWalk state at the
            # locomotion transition.  Do not carry the policy history, previous
            # action, Hopf phase or target filters accumulated by the dormant
            # shadow session through the preceding stand interval.
            self.walk_session.reset()
            self.walk_session_reset_for_profile = True
            initial_yaw = quaternion_rpy_wxyz(self.data.qpos[3:7])[2]
            self.heading_hold.reset(initial_yaw)
            self.cross_track_hold.reset(self.data.qpos[:2], initial_yaw)
        self.walk_sequence += 1
        self.walk_policy_sim_times.append(now)
        roll, pitch, _ = quaternion_rpy_wxyz(self.data.qpos[3:7])
        raw_command = self.profile.velocity_command(now)
        apply_cross_track_stabilization = False
        apply_heading_stabilization = False
        if apply_command:
            requested_yaw = raw_command[2]
            requested_lateral = raw_command[1]
            if abs(raw_command[0]) > 0.10 and abs(raw_command[2]) <= 1e-9:
                self.last_heading_hold_command_yaw_rps = self.heading_hold.update(
                    quaternion_rpy_wxyz(self.data.qpos[3:7])[2],
                    sensor_values(
                        self.mujoco,
                        self.model,
                        self.data,
                        "body-angular-velocity",
                    )[2],
                )
                apply_heading_stabilization = True
                if abs(raw_command[1]) <= 1e-9:
                    (
                        self.last_cross_track_m,
                        self.last_cross_track_command_y_mps,
                    ) = self.cross_track_hold.update(
                        self.data.qpos[:2],
                        sensor_values(
                            self.mujoco, self.model, self.data, "body-linear-vel"
                        )[1],
                    )
                    apply_cross_track_stabilization = True
            else:
                self.last_cross_track_m = 0.0
                self.last_cross_track_command_y_mps = 0.0
                self.last_heading_hold_command_yaw_rps = 0.0
            raw_command = self.velocity_calibration.apply((
                raw_command[0],
                requested_lateral,
                requested_yaw,
            ))
        filtered_command = self.velocity_command_filter.update(raw_command, 0.02)
        command = filtered_command.effective
        if apply_cross_track_stabilization:
            command = apply_post_filter_lateral_stabilization(
                command,
                self.last_cross_track_command_y_mps,
                maximum_lateral_speed_mps=(
                    self.cross_track_hold.params.maximum_lateral_speed_mps
                ),
            )
        if apply_heading_stabilization:
            command = apply_post_filter_yaw_stabilization(
                command,
                self.last_heading_hold_command_yaw_rps,
                maximum_yaw_rate_rps=(
                    self.heading_hold.params.maximum_yaw_rate_rps
                ),
            )
        self.last_raw_velocity_command = raw_command
        self.last_effective_velocity_command = command
        self.last_command_filter_alpha = filtered_command.alpha
        state = X2PhysicsState(
            source_sequence=self.walk_sequence,
            sim_time=now,
            command_stamp_sim_time=now,
            velocity_command=command,
            joint_position=self.q(X2_CPGWALK_JOINT_NAMES),
            joint_velocity=self.dq(X2_CPGWALK_JOINT_NAMES),
            base_angular_velocity=sensor_values(self.mujoco, self.model, self.data, "body-angular-velocity"),
            observation_gravity=x2_cpgwalk_attitude_observation(roll, pitch),
            base_linear_velocity=sensor_values(self.mujoco, self.model, self.data, "body-linear-vel"),
            roll=roll,
            pitch=pitch,
        )
        result = self.walk_session.update(state)
        self.last_walk_policy_action = result.policy_action
        while self.next_walk_update <= now + 1e-12:
            self.next_walk_update += 0.02
        if not apply_command:
            return
        if result.joint_command is None:
            # The deployed velocity filter intentionally keeps the first few
            # non-zero raw-command ticks below the 0.10 walking threshold.  The
            # vendor state machine remains in its CPG standing hold during that
            # interval, but still emits the policy candidate to bridge from the
            # previous stand controller before ACTIVE_COMMAND.
            if result.status in {"SAFE_HOLD", "STAND_HANDOFF_REQUEST"}:
                joint = result.policy_joint_candidate
            else:
                raise RuntimeError(
                    "locomotion command missing outside the expected stand hold"
                )
        else:
            joint = result.joint_command
        limited = self.position_target_safety.apply(
            X2_CPGWALK_JOINT_NAMES,
            joint.position_target,
            sim_time=now,
            phase=self.profile.phase(now),
            policy_sequence=self.walk_sequence,
        )
        self.current_target.update(
            zip(X2_CPGWALK_JOINT_NAMES, limited.applied_position_target)
        )
        self.current_kp.update(zip(X2_CPGWALK_JOINT_NAMES, joint.kp))
        self.current_kd.update(zip(X2_CPGWALK_JOINT_NAMES, joint.kd))

    def apply_pd(self) -> None:
        self.command_application_count += 1
        for name in X2_ALL_JOINT_NAMES:
            q = float(self.data.qpos[self.qpos_index[name]])
            dq = float(self.data.qvel[self.dof_index[name]])
            raw = self.current_kp[name] * (self.current_target[name] - q) - self.current_kd[name] * dq
            if not all(math.isfinite(value) for value in (
                q, dq, self.current_target[name], self.current_kp[name],
                self.current_kd[name], raw,
            )):
                self.nan_detected = True
                raise ValueError(f"non-finite MuJoCo PD value for {name}")
            actuator = self.actuator_id[name]
            low, high = (float(value) for value in self.model.actuator_ctrlrange[actuator])
            limit = min(abs(low), abs(high), _effort_limit(name))
            ratio = abs(raw) / limit
            self.maximum_effort_ratio = max(self.maximum_effort_ratio, ratio)
            self.effort_sample_count += 1
            if ratio >= 1.0:
                self.saturation_count += 1
            self.data.ctrl[actuator] = max(-limit, min(limit, raw))

    def apply_band(self, now: float) -> tuple[float, float, float]:
        scale = self.profile.band_scale(now)
        self.data.xfrc_applied[self.torso_id, :3] = 0.0
        if scale <= 0.0:
            return (0.0, 0.0, 0.0)
        anchor = np.asarray((0.0, 0.0, 2.5), dtype=float)
        position = np.asarray(self.data.xpos[self.torso_id], dtype=float)
        delta = position - anchor
        distance = float(np.linalg.norm(delta))
        if distance <= 1e-9:
            return (0.0, 0.0, 0.0)
        radial = delta / distance
        velocity6 = np.zeros(6, dtype=float)
        self.mujoco.mj_objectVelocity(
            self.model, self.data, self.mujoco.mjtObj.mjOBJ_BODY,
            self.torso_id, velocity6, 0,
        )
        distance_rate = float(np.dot(velocity6[3:6], radial))
        magnitude = max(0.0, 200.0 * distance + 100.0 * distance_rate)
        force = -scale * magnitude * radial
        self.data.xfrc_applied[self.torso_id, :3] = force
        return tuple(float(value) for value in force)

    def contact_state(self) -> dict:
        left = False
        right = False
        left_normal = 0.0
        right_normal = 0.0
        nonfoot = 0.0
        points = []
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            g1, g2 = int(contact.geom1), int(contact.geom2)
            if not ({g1, g2} & self.environment_geoms):
                continue
            force = np.zeros(6, dtype=float)
            self.mujoco.mj_contactForce(self.model, self.data, index, force)
            normal = abs(float(force[0]))
            robot_geom = g2 if g1 in self.environment_geoms else g1
            if robot_geom in self.left_foot_geoms:
                left = True
                left_normal += normal
            elif robot_geom in self.right_foot_geoms:
                right = True
                right_normal += normal
            else:
                nonfoot = max(nonfoot, normal)
            if len(points) < 8:
                points.append([float(value) for value in contact.pos])
        self.maximum_nonfoot_contact_force = max(self.maximum_nonfoot_contact_force, nonfoot)
        return {
            "left": left,
            "right": right,
            "left_normal_force_n": left_normal,
            "right_normal_force_n": right_normal,
            "maximum_nonfoot_force_n": nonfoot,
            "points_world": points,
        }

    def foot_slip(self, body_id: int) -> float:
        velocity = np.zeros(6, dtype=float)
        self.mujoco.mj_objectVelocity(
            self.model, self.data, self.mujoco.mjtObj.mjOBJ_BODY, body_id, velocity, 0
        )
        return math.hypot(float(velocity[3]), float(velocity[4]))

    def record(self, now: float, phase: str, band_force, contacts: dict) -> None:
        position = tuple(float(value) for value in self.data.qpos[:3])
        orientation = tuple(float(value) for value in self.data.qpos[3:7])
        roll, pitch, yaw = quaternion_rpy_wxyz(orientation)
        tilt = math.hypot(roll, pitch)
        self.minimum_z = min(self.minimum_z, position[2])
        self.maximum_tilt = max(self.maximum_tilt, tilt)
        if phase == "GAIT1_FORWARD":
            self.walk_steps += 1
            self.left_contact_steps += int(contacts["left"])
            self.right_contact_steps += int(contacts["right"])
            if contacts["left"]:
                self.left_slip.append(self.foot_slip(self.left_foot_body))
            if contacts["right"]:
                self.right_slip.append(self.foot_slip(self.right_foot_body))
            self.left_slip_samples.append({
                "sim_time": now,
                "contact": contacts["left"],
                "normal_force_n": contacts["left_normal_force_n"],
                "tangential_slip_speed_mps": self.foot_slip(self.left_foot_body),
            })
            self.right_slip_samples.append({
                "sim_time": now,
                "contact": contacts["right"],
                "normal_force_n": contacts["right_normal_force_n"],
                "tangential_slip_speed_mps": self.foot_slip(self.right_foot_body),
            })
            self.support_force_during_walk_max = max(
                self.support_force_during_walk_max,
                math.sqrt(sum(float(value) ** 2 for value in band_force)),
            )
        if now + 1e-12 >= self.last_trace_time + 0.01:
            body_linear = sensor_values(self.mujoco, self.model, self.data, "body-linear-vel")
            body_angular = sensor_values(self.mujoco, self.model, self.data, "body-angular-velocity")
            hopf = self.walk_session.hopf.snapshot()
            cpg_phase = math.atan2(hopf.obs4[0], hopf.obs4[1])
            self.trace.append({
                "sim_time": now,
                "phase": phase,
                "base_position_m": list(position),
                "base_orientation_wxyz": list(orientation),
                "base_rpy_rad": [roll, pitch, yaw],
                "base_linear_velocity_body_mps": list(body_linear),
                "base_angular_velocity_body_rps": list(body_angular),
                "cpg_phase_rad": cpg_phase,
                "cpg_period_s": hopf.period,
                "active_gait": hopf.gait,
                "requested_velocity": list(self.profile.velocity_command(now)),
                "effective_velocity_command": list(self.last_effective_velocity_command),
                "command_filter_alpha": self.last_command_filter_alpha,
                "walk_policy_action": list(self.last_walk_policy_action),
                "straight_cross_track_m": self.last_cross_track_m,
                "straight_cross_track_command_y_mps": (
                    self.last_cross_track_command_y_mps
                ),
                "straight_heading_hold_command_yaw_rps": (
                    self.last_heading_hold_command_yaw_rps
                ),
                "joint_position": list(self.q(X2_ALL_JOINT_NAMES)),
                "joint_velocity": list(self.dq(X2_ALL_JOINT_NAMES)),
                "position_target": [self.current_target[name] for name in X2_ALL_JOINT_NAMES],
                "band_scale": self.profile.band_scale(now),
                "band_force_world_n": list(band_force),
                "contacts": contacts,
                "real_robot_commands": False,
            })
            self.last_trace_time = now

    def run(self) -> dict:
        timestep = float(self.model.opt.timestep)
        while float(self.data.time) < self.profile.total_duration_s - 0.5 * timestep:
            now = float(self.data.time)
            phase = self.profile.phase(now)
            if phase in {
                "SUPPORTED_STAND", "BAND_RAMP_DOWN", "UNSUPPORTED_STAND",
                "GAIT1_FORWARD", "DECELERATE", "CPG_STAND_HANDOFF",
            }:
                self.update_walk(
                    now,
                    apply_command=phase in {
                        "GAIT1_FORWARD", "DECELERATE", "CPG_STAND_HANDOFF",
                    },
                )
            if phase in {"SUPPORTED_STAND", "BAND_RAMP_DOWN", "UNSUPPORTED_STAND"}:
                self.update_stand(now, stop=False)
            elif phase in {"GAIT1_FORWARD", "DECELERATE", "CPG_STAND_HANDOFF"}:
                pass
            elif phase == "STOP_STAND":
                self.update_stand(now, stop=True)
            if (
                self.freeze_probe is None
                and phase == "GAIT1_FORWARD"
                and now >= self.profile.walk_start_s + 3.0
            ):
                adapter_state_before = {
                    "walk_session_digest": self.walk_session.state_digest(),
                    "velocity_filter": self.velocity_command_filter.snapshot(),
                    "heading_hold": self.heading_hold.snapshot(),
                    "cross_track_hold": self.cross_track_hold.snapshot(),
                    "position_target_safety": self.position_target_safety.snapshot(),
                    "next_walk_update": self.next_walk_update,
                    "walk_sequence": self.walk_sequence,
                    "walk_session_reset": self.walk_session_reset_for_profile,
                    "current_target": dict(self.current_target),
                    "current_kp": dict(self.current_kp),
                    "current_kd": dict(self.current_kd),
                    "last_policy_action": list(self.last_walk_policy_action),
                    "walk_policy_call_count": len(self.walk_policy.elapsed_ms),
                    "trace_record_count": len(self.trace),
                    "last_effective_command": list(
                        self.last_effective_velocity_command
                    ),
                    "last_cross_track_m": self.last_cross_track_m,
                    "last_cross_track_command_y_mps": (
                        self.last_cross_track_command_y_mps
                    ),
                    "last_heading_hold_command_yaw_rps": (
                        self.last_heading_hold_command_yaw_rps
                    ),
                }
                command_before = self.command_application_count
                sim_time_before = float(self.data.time)
                wall_started = time.perf_counter()
                time.sleep(self.freeze_probe_wall_s)
                wall_elapsed = time.perf_counter() - wall_started
                adapter_state_after = {
                    "walk_session_digest": self.walk_session.state_digest(),
                    "velocity_filter": self.velocity_command_filter.snapshot(),
                    "heading_hold": self.heading_hold.snapshot(),
                    "cross_track_hold": self.cross_track_hold.snapshot(),
                    "position_target_safety": self.position_target_safety.snapshot(),
                    "next_walk_update": self.next_walk_update,
                    "walk_sequence": self.walk_sequence,
                    "walk_session_reset": self.walk_session_reset_for_profile,
                    "current_target": dict(self.current_target),
                    "current_kp": dict(self.current_kp),
                    "current_kd": dict(self.current_kd),
                    "last_policy_action": list(self.last_walk_policy_action),
                    "walk_policy_call_count": len(self.walk_policy.elapsed_ms),
                    "trace_record_count": len(self.trace),
                    "last_effective_command": list(
                        self.last_effective_velocity_command
                    ),
                    "last_cross_track_m": self.last_cross_track_m,
                    "last_cross_track_command_y_mps": (
                        self.last_cross_track_command_y_mps
                    ),
                    "last_heading_hold_command_yaw_rps": (
                        self.last_heading_hold_command_yaw_rps
                    ),
                }
                self.freeze_probe = {
                    "method": "controller_thread_wall_sleep_without_mujoco_step",
                    "requested_wall_pause_s": self.freeze_probe_wall_s,
                    "observed_wall_pause_s": wall_elapsed,
                    "sim_time_before_s": sim_time_before,
                    "sim_time_after_s": float(self.data.time),
                    "command_count_before": command_before,
                    "command_count_after": self.command_application_count,
                    "controller_digest_before": adapter_state_before["walk_session_digest"],
                    "controller_digest_after": adapter_state_after["walk_session_digest"],
                    "adapter_state_before": adapter_state_before,
                    "adapter_state_after": adapter_state_after,
                    "controller_state_unchanged": (
                        adapter_state_before == adapter_state_after
                    ),
                    "command_sequence_unchanged": (
                        command_before == self.command_application_count
                    ),
                    "sim_time_unchanged": sim_time_before == float(self.data.time),
                }
                if not all((
                    adapter_state_before == adapter_state_after,
                    command_before == self.command_application_count,
                    sim_time_before == float(self.data.time),
                    wall_elapsed >= 0.90 * self.freeze_probe_wall_s,
                )):
                    self.failure = "sim_time_freeze_probe_advanced_controller"
                    break
            # One command per authoritative MuJoCo physics state.
            if now + 1e-12 >= self.next_pd_update:
                self.apply_pd()
                while self.next_pd_update <= now + 1e-12:
                    self.next_pd_update += self.pd_period_s
            band_force = self.apply_band(now)
            self.mujoco.mj_step(self.model, self.data)
            contacts = self.contact_state()
            self.record(float(self.data.time), phase, band_force, contacts)
            values = np.concatenate((self.data.qpos, self.data.qvel, self.data.ctrl))
            if not np.isfinite(values).all():
                self.nan_detected = True
                self.failure = "NaN_or_Inf"
                break
            velocity_exceeded_this_step = False
            for name in X2_ALL_JOINT_NAMES:
                value = float(self.data.qpos[self.qpos_index[name]])
                velocity = float(self.data.qvel[self.dof_index[name]])
                lower, upper = self.joint_limits[name]
                excess = max(lower - value, value - upper, 0.0)
                self.maximum_joint_limit_excess_rad = max(
                    self.maximum_joint_limit_excess_rad, excess
                )
                if excess > 1e-4:
                    self.joint_limit_violation_count += 1
                    self.failure = (
                        f"joint_limit:{name}:{value:.6f}:"
                        f"[{lower:.6f},{upper:.6f}]"
                    )
                    break
                velocity_ratio = abs(velocity) / self.joint_velocity_limits[name]
                self.maximum_joint_velocity_ratio = max(
                    self.maximum_joint_velocity_ratio, velocity_ratio
                )
                if velocity_ratio > 1.0:
                    self.joint_velocity_limit_violation_count += 1
                    velocity_exceeded_this_step = True
                if velocity_ratio > X2_JOINT_VELOCITY_NUMERIC_TOLERANCE_RATIO:
                    self.hard_joint_velocity_limit_violation_count += 1
                desired = float(self.current_target[name])
                target_excess = max(lower - desired, desired - upper, 0.0)
                self.maximum_position_target_limit_excess_rad = max(
                    self.maximum_position_target_limit_excess_rad, target_excess
                )
                if target_excess > 1e-4:
                    self.position_target_limit_violation_count += 1
                    self.failure = (
                        f"position_target_limit:{name}:{desired:.6f}:"
                        f"[{lower:.6f},{upper:.6f}]"
                    )
                    break
            if velocity_exceeded_this_step:
                self.current_joint_velocity_exceedance_steps += 1
                self.maximum_joint_velocity_exceedance_steps = max(
                    self.maximum_joint_velocity_exceedance_steps,
                    self.current_joint_velocity_exceedance_steps,
                )
            else:
                self.current_joint_velocity_exceedance_steps = 0
            if self.failure is not None:
                break
            if self.data.qpos[2] < 0.40 or self.maximum_tilt > 0.80:
                self.failure = "fall_safety_limit"
                break
        return self.result()

    def result(self) -> dict:
        start = self.trace[0]["base_position_m"] if self.trace else [0.0, 0.0, 0.0]
        end = self.trace[-1]["base_position_m"] if self.trace else [0.0, 0.0, 0.0]
        final_linear = math.sqrt(sum(float(value) ** 2 for value in self.data.qvel[:3]))
        final_angular = math.sqrt(sum(float(value) ** 2 for value in self.data.qvel[3:6]))
        saturation = self.saturation_count / self.effort_sample_count if self.effort_sample_count else 1.0
        left_slip = summarize_support_slip(self.left_slip_samples)
        right_slip = summarize_support_slip(self.right_slip_samples)
        walk_trace = [row for row in self.trace if row["phase"] == "GAIT1_FORWARD"]
        walk_start = walk_trace[0] if walk_trace else None
        walk_end = walk_trace[-1] if walk_trace else None
        walk_forward = (
            float(walk_end["base_position_m"][0]) - float(walk_start["base_position_m"][0])
            if walk_start and walk_end else 0.0
        )
        yaw_drift = (
            math.atan2(
                math.sin(float(walk_end["base_rpy_rad"][2]) - float(walk_start["base_rpy_rad"][2])),
                math.cos(float(walk_end["base_rpy_rad"][2]) - float(walk_start["base_rpy_rad"][2])),
            ) if walk_start and walk_end else 0.0
        )
        common_pose_rows = [
            {
                "sim_time": row["sim_time"], "phase": row["phase"],
                "position_m": row["base_position_m"], "yaw_rad": row["base_rpy_rad"][2],
            }
            for row in self.trace
        ]
        pose_speed = None
        pose_turn = None
        pose_turn_full = None
        pose_displacement = None
        steady_forward_speed = None
        steady_yaw_rate = None
        if len(walk_trace) >= 2:
            try:
                pose_speed = pose_derived_forward_speed(common_pose_rows)
                pose_turn = pose_derived_turn_rate(common_pose_rows)
                pose_turn_full = pose_derived_turn_rate(
                    common_pose_rows, trim_start_s=0.0, trim_end_s=0.0
                )
                pose_displacement = pose_derived_local_displacement(common_pose_rows)
                steady_forward_speed = float(pose_speed["mean_forward_speed_mps"])
                steady_yaw_rate = float(pose_turn["mean_yaw_rate_rps"])
            except ValueError:
                # Preserve the original fail-closed reason in an incomplete
                # development run instead of masking it during result export.
                pass
        stop_trace = [
            row for row in self.trace
            if row["phase"] in {"DECELERATE", "CPG_STAND_HANDOFF", "STOP_STAND"}
        ]
        stop_distance = None
        if stop_trace:
            stop_start = stop_trace[0]["base_position_m"]
            stop_end = stop_trace[-1]["base_position_m"]
            stop_distance = math.hypot(
                float(stop_end[0]) - float(stop_start[0]),
                float(stop_end[1]) - float(stop_start[1]),
            )
        stop_rows = [
            {
                "sim_time": row["sim_time"], "phase": row["phase"],
                "position_m": row["base_position_m"],
                "linear_speed_mps": math.sqrt(sum(
                    float(value) ** 2 for value in row["base_linear_velocity_body_mps"]
                )),
                "angular_speed_rps": math.sqrt(sum(
                    float(value) ** 2 for value in row["base_angular_velocity_body_rps"]
                )),
            }
            for row in self.trace
        ]
        stable_stop = None
        if sum(row["phase"] == "STOP_STAND" for row in stop_rows) >= 2:
            stable_stop = summarize_stable_stop(stop_rows)
        target_safety = self.position_target_safety.snapshot()
        checks = {
            "completed_profile": self.failure is None and self.data.time >= self.profile.total_duration_s - 0.002,
            "no_nan_or_inf": not self.nan_detected,
            "minimum_base_height": self.minimum_z >= 0.55,
            "maximum_base_tilt": self.maximum_tilt <= 0.35,
            "zero_support_during_locomotion": self.support_force_during_walk_max <= 1e-9,
            "zero_nonfoot_contact": self.maximum_nonfoot_contact_force <= 20.0,
            "both_feet_contact_during_walk": (
                self.walk_steps > 0
                and self.left_contact_steps / self.walk_steps >= 0.20
                and self.right_contact_steps / self.walk_steps >= 0.20
            ),
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
            "final_linear_speed": final_linear <= 0.05,
            "final_angular_speed": final_angular <= 0.05,
            "sustained_stable_stop": (
                stable_stop is not None
                and float(stable_stop["linear_speed_p95_mps"]) <= 0.05
                and float(stable_stop["angular_speed_p95_rps"]) <= 0.05
                and float(stable_stop["maximum_linear_speed_mps"]) <= 0.05
                and float(stable_stop["maximum_angular_speed_rps"]) <= 0.05
                and float(stable_stop["planar_drift_m"]) <= 0.03
            ),
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
                self.maximum_joint_velocity_exceedance_steps
                * float(self.model.opt.timestep) <= 0.010
            ),
            "no_fall": self.minimum_z >= 0.55 and self.maximum_tilt <= 0.35,
            "one_pd_application_per_physics_step": (
                self.command_application_count
                == int(round(float(self.data.time) / self.pd_period_s))
            ),
            "walk_policy_strict_sim_time": (
                len(self.walk_policy_sim_times) > 1
                and all(after > before for before, after in zip(
                    self.walk_policy_sim_times, self.walk_policy_sim_times[1:]
                ))
            ),
            "sim_time_freeze_probe": (
                self.freeze_probe is not None
                and self.freeze_probe["controller_state_unchanged"] is True
                and self.freeze_probe["command_sequence_unchanged"] is True
                and self.freeze_probe["sim_time_unchanged"] is True
            ),
        }
        non_blocking_reference_checks = (
            "joint_velocity_within_numeric_tolerance",
            "no_sustained_joint_velocity_limit_violation",
        )
        development_passed = all(
            value for name, value in checks.items()
            if name not in non_blocking_reference_checks
        )
        return {
            "schema": "sceneforge/x2-mujoco-independent-motion-baseline/v1",
            "status": "PASSED" if development_passed else "FAILED",
            "qualification": False,
            "decision": (
                "proceed_to_physx_gait1_ab"
                if development_passed else "debug_mujoco_motion_adapter"
            ),
            "physics_authority": "google_mujoco_3_3_5",
            "controller": "independent_state_driven_x2_onnx_session",
            "profile": self.profile.__dict__,
            "velocity_command_calibration": self.velocity_calibration.__dict__,
            "position_target_safety_envelope": target_safety,
            "checks": checks,
            "non_blocking_reference_checks": list(non_blocking_reference_checks),
            "failure": self.failure,
            "parameters": {
                "integration_timestep_s": float(self.model.opt.timestep),
                "pd_period_sim_time_s": self.pd_period_s,
                "joint_limit_source": str(self.joint_limit_urdf),
                "joint_limit_source_sha256": sha256(self.joint_limit_urdf),
                "joint_position_limits_rad": {
                    name: list(self.joint_limits[name]) for name in X2_ALL_JOINT_NAMES
                },
                "joint_velocity_limits_rps": self.joint_velocity_limits,
                "joint_velocity_numeric_tolerance_ratio": (
                    X2_JOINT_VELOCITY_NUMERIC_TOLERANCE_RATIO
                ),
                "freeze_probe_wall_s": self.freeze_probe_wall_s,
            },
            "metrics": {
                "sim_duration_s": float(self.data.time),
                "wall_duration_s": time.perf_counter() - self.started_wall,
                "physics_step_count": int(round(self.data.time / self.model.opt.timestep)),
                "stand_policy_call_count": len(self.stand_policy.elapsed_ms),
                "walk_policy_call_count": len(self.walk_policy.elapsed_ms),
                "start_position_m": start,
                "end_position_m": end,
                "forward_displacement_m": end[0] - start[0],
                "lateral_displacement_m": end[1] - start[1],
                "minimum_base_height_m": self.minimum_z,
                "maximum_tilt_rad": self.maximum_tilt,
                "final_linear_speed_mps": final_linear,
                "final_angular_speed_rps": final_angular,
                "left_walk_contact_fraction": self.left_contact_steps / self.walk_steps if self.walk_steps else None,
                "right_walk_contact_fraction": self.right_contact_steps / self.walk_steps if self.walk_steps else None,
                "left_support_slip_p95_mps": percentile(self.left_slip, 0.95),
                "right_support_slip_p95_mps": percentile(self.right_slip, 0.95),
                "left_raw_contact_slip_p95_mps": left_slip["raw_contact_slip_p95_mps"],
                "right_raw_contact_slip_p95_mps": right_slip["raw_contact_slip_p95_mps"],
                "left_steady_support_slip_p95_mps": left_slip["steady_support_slip_p95_mps"],
                "right_steady_support_slip_p95_mps": right_slip["steady_support_slip_p95_mps"],
                "left_support_slip_summary": left_slip,
                "right_support_slip_summary": right_slip,
                "walk_start_yaw_rad": walk_start["base_rpy_rad"][2] if walk_start else None,
                "walk_end_yaw_rad": walk_end["base_rpy_rad"][2] if walk_end else None,
                "walk_yaw_drift_rad": yaw_drift,
                "walk_yaw_drift_rad_per_forward_m": abs(yaw_drift) / max(abs(walk_forward), 1e-9),
                "steady_forward_speed_mps": steady_forward_speed,
                "steady_forward_speed_relative_error": (
                    abs(float(steady_forward_speed) - self.profile.command_x_mps)
                    / max(abs(self.profile.command_x_mps), 1e-9)
                    if steady_forward_speed is not None else None
                ),
                "steady_forward_speed_sample_count": (
                    pose_speed["sample_count"] if pose_speed else 0
                ),
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
                    abs(steady_yaw_rate - self.profile.command_yaw_rps)
                    / max(abs(self.profile.command_yaw_rps), 1e-9)
                    if steady_yaw_rate is not None
                    and abs(self.profile.command_yaw_rps) > 1e-9 else 0.0
                ),
                "steady_yaw_rate_estimator": pose_turn,
                "stop_distance_xy_m": stop_distance,
                "stable_stop": stable_stop,
                "maximum_nonfoot_contact_force_n": self.maximum_nonfoot_contact_force,
                "support_force_during_walk_max_n": self.support_force_during_walk_max,
                "effort_saturation_ratio": saturation,
                "maximum_raw_effort_ratio": self.maximum_effort_ratio,
                "joint_limit_violation_count": self.joint_limit_violation_count,
                "maximum_joint_limit_excess_rad": self.maximum_joint_limit_excess_rad,
                "joint_velocity_limit_violation_count": self.joint_velocity_limit_violation_count,
                "hard_joint_velocity_limit_violation_count": (
                    self.hard_joint_velocity_limit_violation_count
                ),
                "maximum_joint_velocity_ratio": self.maximum_joint_velocity_ratio,
                "maximum_joint_velocity_exceedance_duration_s": (
                    self.maximum_joint_velocity_exceedance_steps
                    * float(self.model.opt.timestep)
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
                "fall_event_count": int(
                    self.minimum_z < 0.55 or self.maximum_tilt > 0.35
                ),
                "nonfinite_sample_count": int(self.nan_detected),
                "command_application_count": self.command_application_count,
                "command_hold_period_sim_time_s": self.pd_period_s,
                "walk_policy_update_count": len(self.walk_policy_sim_times),
                "sim_time_freeze_probe": self.freeze_probe,
            },
            "joint_names": list(X2_ALL_JOINT_NAMES),
            "trace_sample_count": len(self.trace),
            "published_command_topics": [],
            "real_robot_commands": False,
            "limitations": [
                "The vendor MuJoCo reference exceeds the URDF nominal joint-velocity value during short contact/transient intervals; strict exceedance count, peak ratio, and duration are retained, but this does not relabel the URDF velocity as a hard state constraint.",
                "Qualification remains false until vendor velocity-limit semantics are resolved or a common adapter cap is validated.",
                "Raw stand/walk policy position targets are passed through the same auditable URDF-limit safety envelope used by PhysX; every clamp remains in the immutable receipt and trace.",
            ],
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--stand-model", type=Path, required=True)
    parser.add_argument("--walk-model", type=Path, required=True)
    parser.add_argument("--joint-limit-urdf", type=Path, required=True)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--supported-stand-s", type=float, default=3.0)
    parser.add_argument("--band-ramp-s", type=float, default=5.0)
    parser.add_argument("--unsupported-stand-s", type=float, default=3.0)
    parser.add_argument("--walk-s", type=float, default=8.0)
    parser.add_argument("--stop-s", type=float, default=8.0)
    parser.add_argument("--command-x-mps", type=float, default=0.3)
    parser.add_argument("--command-y-mps", type=float, default=0.0)
    parser.add_argument("--command-yaw-rps", type=float, default=0.0)
    parser.add_argument("--disable-ramp", action="store_true")
    args = parser.parse_args()
    if os.environ.get("REAL_ROBOT_COMMANDS", "false").lower() != "false":
        raise RuntimeError("REAL_ROBOT_COMMANDS must be false")
    if args.result.resolve() == args.trace.resolve():
        raise ValueError("result and trace paths must differ")

    import mujoco

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    contract = inspect_x2_mujoco_model(mujoco, model)
    if not all(contract.checks().values()):
        raise RuntimeError("frozen MuJoCo model contract failed")
    if args.disable_ramp:
        ramp_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ramp_collision"))
        if ramp_id < 0:
            raise ValueError("requested ramp disable but ramp_collision is absent")
        model.geom_contype[ramp_id] = 0
        model.geom_conaffinity[ramp_id] = 0
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    run = X2MujocoMotionRun(mujoco, model, data, args)
    payload = run.run()
    payload["runtime"] = {
        "mujoco_version": mujoco.__version__,
        "model_path": str(args.model.resolve()),
        "model_sha256": sha256(args.model.resolve()),
        "stand_model_sha256": sha256(args.stand_model.resolve()),
        "walk_model_sha256": sha256(args.walk_model.resolve()),
        "timestep_s": float(model.opt.timestep),
        "ramp_collision_disabled": bool(args.disable_ramp),
        "model_contract": contract.as_dict(),
    }
    atomic_json(args.trace.resolve(), {
        "schema": "sceneforge/x2-locomotion-samples/v1",
        "physics_authority": "google_mujoco_3_3_5",
        "samples": run.trace,
        "position_target_safety_envelope": run.position_target_safety.snapshot(),
        "real_robot_commands": False,
    })
    payload["artifacts"] = {
        "trace": {
            "path": str(args.trace.resolve()),
            "size": args.trace.resolve().stat().st_size,
            "sha256": sha256(args.trace.resolve()),
        }
    }
    atomic_json(args.result.resolve(), payload)
    print(json.dumps({"status": payload["status"], "decision": payload["decision"]}))
    return 0 if payload["status"] == "PASSED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
