"""Suspension-assisted, then unsupported X2 STAND_DEFAULT development gate."""

from __future__ import annotations

import json
import math
import os
import tempfile
from pathlib import Path
from typing import Callable

from .x2_fixed_base_stand import (
    X2FixedBaseStandCapture,
    _OBSERVATION_DEFAULT,
    _atomic_json,
    _percentile,
    _sha256,
)
from .x2_stand import X2StandPhysicsState, X2_STAND_ARM_TARGET, X2_STAND_JOINT_NAMES


SCHEMA = "sceneforge/x2-geniesim-floating-stand-development/v1"


def _replace_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        os.chmod(path, 0o644)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _project_gravity_with_vendor_pitch_offset(
    quaternion_wxyz: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    qw, qx, qy, qz = (float(value) for value in quaternion_wxyz)
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm <= 1e-12 or not math.isfinite(norm):
        raise ValueError("base quaternion is invalid")
    qw, qx, qy, qz = (value / norm for value in (qw, qx, qy, qz))
    # R_world_from_body.T @ [0, 0, -1].
    gx = -2.0 * (qx * qz - qw * qy)
    gy = -2.0 * (qy * qz + qw * qx)
    gz = -(1.0 - 2.0 * (qx * qx + qy * qy))
    offset = 0.03
    return (
        math.cos(offset) * gx + math.sin(offset) * gz,
        gy,
        -math.sin(offset) * gx + math.cos(offset) * gz,
    )


class X2FloatingStandCapture(X2FixedBaseStandCapture):
    """Run STAND_DEFAULT through a physical elastic-band release."""

    def __init__(
        self,
        sim,
        *,
        model: str | Path,
        receipt: str | Path,
        trace: str | Path,
        stop_request: str | Path,
        band_command: str | Path,
        band_status: str | Path,
        state_reader: Callable[[float], object],
        contact_reader: Callable[[float], dict] | None = None,
        supported_s: float = 3.0,
        ramp_s: float = 5.0,
        unsupported_hold_s: float = 5.0,
    ) -> None:
        super().__init__(
            sim,
            model=model,
            receipt=receipt,
            trace=trace,
            stop_request=stop_request,
            warmup_s=0.1,
            active_s=0.1,
            restore_s=0.1,
        )
        values = (supported_s, ramp_s, unsupported_hold_s)
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("floating stand durations must be finite and positive")
        self.supported_s = float(supported_s)
        self.ramp_s = float(ramp_s)
        self.unsupported_hold_s = float(unsupported_hold_s)
        self.total_duration_s = sum(values)
        self.band_command = Path(band_command).resolve()
        self.band_status = Path(band_status).resolve()
        if not self.band_command.is_file():
            raise FileNotFoundError(self.band_command)
        # The authoritative PhysX elastic-band controller consumes the
        # pre-start command and creates its status receipt during
        # ``EngineSession.startup``.  This capture is constructed afterwards,
        # so the status file is an input, not one of our immutable outputs.
        if not self.band_status.is_file():
            raise FileNotFoundError(self.band_status)
        self.state_reader = state_reader
        self.contact_reader = contact_reader
        self.ramp_requested = False
        self.base_records: list[dict] = []
        self.contact_records: list[dict] = []

    def _phase(self, elapsed: float) -> str:
        if elapsed < self.supported_s:
            return "SUPPORTED_STAND"
        if elapsed < self.supported_s + self.ramp_s:
            return "BAND_RAMP_DOWN"
        if elapsed < self.total_duration_s:
            return "UNSUPPORTED_HOLD"
        return "COMPLETE"

    def _request_ramp_down(self) -> None:
        if self.ramp_requested:
            return
        _replace_json(self.band_command, {
            "schema": "sceneforge/physx-elastic-band-command/v1",
            "sequence": 2,
            "state": "RAMP_DOWN",
            "ramp_duration_s": self.ramp_s,
            "requested_by": "x2_floating_stand_capture",
            "real_robot_commands": False,
        })
        self.ramp_requested = True

    def _observed_unsupported_hold_s(self) -> float:
        if not self.band_status.is_file():
            return -1.0
        band = json.loads(self.band_status.read_text(encoding="utf-8"))
        release_times = [
            float(item["sim_time_s"])
            for item in band.get("transitions", [])
            if item.get("requested_state") == "RAMP_DOWN_COMPLETE"
            and item.get("effective_state") == "RELEASED"
        ]
        if not release_times:
            return -1.0
        return float(band.get("sim_time_s", 0.0)) - release_times[-1]

    def _write_trace(self) -> None:
        _atomic_json(self.trace, {
            "schema": "sceneforge/x2-floating-stand-trace/v1",
            "joint_names": list(X2_STAND_JOINT_NAMES),
            "physics_samples": self.trace_records,
            "base_samples": self.base_records,
            "contact_samples": self.contact_records,
            "commands": self.command_records,
            "real_robot_commands": False,
        })

    def _result(self, requested_status: str) -> dict:
        band = (
            json.loads(self.band_status.read_text(encoding="utf-8"))
            if self.band_status.is_file() else {}
        )
        release_times = [
            float(item["sim_time_s"])
            for item in band.get("transitions", [])
            if item.get("requested_state") == "RAMP_DOWN_COMPLETE"
            and item.get("effective_state") == "RELEASED"
        ]
        unsupported = [record for record in self.base_records if record["phase"] == "UNSUPPORTED_HOLD"]
        minimum_z = min((record["z_m"] for record in unsupported), default=None)
        maximum_tilt = max((record["tilt_rad"] for record in unsupported), default=None)
        final = self.base_records[-1] if self.base_records else None
        observed_hold = self._observed_unsupported_hold_s()
        saturation_ratio = (
            self.effort_saturation_count / self.effort_sample_count
            if self.effort_sample_count else None
        )
        unsupported_contacts = [
            record for record in self.contact_records if record["phase"] == "UNSUPPORTED_HOLD"
        ]
        contact_metrics = {}
        if unsupported_contacts:
            for foot_name in ("left_ankle_roll_link", "right_ankle_roll_link"):
                feet = [record["feet"][foot_name] for record in unsupported_contacts]
                contact_metrics[foot_name] = {
                    "contact_fraction": sum(bool(foot["contact"]) for foot in feet) / len(feet),
                    "vertical_force_median_n": _percentile(
                        [float(foot["net_force_n"][2]) for foot in feet], 0.5
                    ),
                    "tangential_slip_speed_p95_mps": _percentile(
                        [float(foot["tangential_slip_speed_mps"]) for foot in feet], 0.95
                    ),
                    "maximum_contact_count": max(int(foot["contact_count"]) for foot in feet),
                }
            contact_metrics["maximum_non_foot_contact_force_n"] = max(
                float(record["maximum_non_foot_contact_force_n"])
                for record in unsupported_contacts
            )
            contact_metrics["combined_vertical_force_median_n"] = _percentile([
                sum(float(record["feet"][name]["net_force_n"][2]) for name in (
                    "left_ankle_roll_link", "right_ankle_roll_link"
                )) for record in unsupported_contacts
            ], 0.5)
        contact_values_finite = all(
            all(math.isfinite(float(value)) for value in (
                *record["feet"][name]["net_force_n"],
                *record["feet"][name]["net_torque_about_foot_origin_nm"],
                record["feet"][name]["tangential_slip_speed_mps"],
            ))
            for record in unsupported_contacts
            for name in ("left_ankle_roll_link", "right_ankle_roll_link")
        )
        checks = {
            "floating_base": True,
            "ramp_requested": self.ramp_requested,
            "minimum_policy_calls": len(self.policy.elapsed_ms) >= int(
                self.total_duration_s * 100.0 * 0.98
            ),
            "all_31_efforts_only": all(
                record["position_command_count"] == 0 and record["effort_command_count"] == 31
                for record in self.command_records
            ),
            "band_released": band.get("state") == "RELEASED",
            "band_support_zero": band.get("support_ratio") == 0.0,
            "band_force_zero": band.get("force_norm_n") == 0.0,
            "band_root_not_pinned": band.get("root_pinned") is False,
            "gravity_not_modified": band.get("gravity_modified") is False,
            "unsupported_hold_completed": observed_hold >= self.unsupported_hold_s - 0.02,
            "base_height_bounded": minimum_z is not None and minimum_z >= 0.55,
            "base_tilt_bounded": maximum_tilt is not None and maximum_tilt <= 0.35,
            "final_base_speed_bounded": (
                final is not None
                and final["linear_speed_mps"] <= 0.10
                and final["angular_speed_rad_s"] <= 0.15
            ),
            "effort_saturation_ratio_bounded": saturation_ratio is not None and saturation_ratio <= 0.01,
            "physx_contact_telemetry_available": bool(unsupported_contacts),
            "both_feet_ground_contact_observed": (
                bool(unsupported_contacts)
                and all(contact_metrics[name]["contact_fraction"] >= 0.95 for name in (
                    "left_ankle_roll_link", "right_ankle_roll_link"
                ))
            ),
            "non_foot_ground_contact_absent": (
                bool(unsupported_contacts)
                and contact_metrics["maximum_non_foot_contact_force_n"] <= 20.0
            ),
            "foot_contact_wrenches_finite": bool(unsupported_contacts) and contact_values_finite,
            "support_load_plausible": (
                bool(unsupported_contacts)
                and 300.0 <= contact_metrics["combined_vertical_force_median_n"] <= 900.0
            ),
            "support_slip_bounded": (
                bool(unsupported_contacts)
                and all(
                    contact_metrics[name]["tangential_slip_speed_p95_mps"] <= 0.03
                    for name in ("left_ankle_roll_link", "right_ankle_roll_link")
                )
            ),
            "no_nonfinite_state": all(
                all(math.isfinite(value) for value in (*record["position"], *record["velocity"]))
                for record in self.trace_records
            ),
            "no_runtime_error": self.last_error is None,
            "real_robot_commands_false": True,
        }
        final_status = "PASSED" if requested_status == "PASSED" and all(checks.values()) else "FAILED"
        return {
            "schema": SCHEMA,
            "status": final_status,
            "decision": (
                (
                    "proceed_to_mujoco_motion_baseline"
                    if self.unsupported_hold_s >= 60.0
                    else "proceed_to_60s_unsupported_stand"
                ) if final_status == "PASSED"
                else "remain_at_suspension_assisted_floating_stand"
            ),
            "qualification": False,
            "physics_authority": "genie_sim_isaac_physx",
            "base_mode": "floating",
            "root_pinned": False,
            "gravity_modified": False,
            "published_command_topics": [],
            "real_robot_commands": False,
            "model": {
                "path": str(self.model),
                "sha256": _sha256(self.model),
                "onnxruntime_version": self.policy.ort.__version__,
                "output_sha256": self.policy.digest.hexdigest(),
            },
            "parameters": {
                "policy_rate_hz": 100.0,
                "pd_rate_hz": 500.0,
                "supported_s": self.supported_s,
                "ramp_s": self.ramp_s,
                "unsupported_hold_s": self.unsupported_hold_s,
                "articulation_joint_count": len(self.articulation_joint_names),
                "physx_contact_report_threshold_n": 0.0,
            },
            "metrics": {
                "physics_step_count": self.physics_sequence,
                "policy_call_count": len(self.policy.elapsed_ms),
                "command_application_calls": len(self.command_records),
                "unsupported_minimum_base_height_m": minimum_z,
                "unsupported_maximum_base_tilt_rad": maximum_tilt,
                "observed_unsupported_hold_s": observed_hold,
                "final_linear_speed_mps": final["linear_speed_mps"] if final else None,
                "final_angular_speed_rad_s": final["angular_speed_rad_s"] if final else None,
                "effort_saturation_ratio": saturation_ratio,
                "maximum_raw_effort_ratio": self.maximum_raw_effort_ratio,
                "inference_wall_p95_ms": _percentile(self.policy.elapsed_ms, 0.95),
                "contact_sample_count": len(self.contact_records),
                "unsupported_contact_sample_count": len(unsupported_contacts),
                "contact": contact_metrics,
            },
            "checks": checks,
            "elastic_band_status": band,
            "trace": {
                "path": str(self.trace),
                "sha256": _sha256(self.trace) if self.trace.is_file() else None,
            },
            "last_error": self.last_error,
            "limitations": [
                "This is a five-second unsupported development hold, not the required 60-second standing qualification.",
                "Contact wrenches come from PhysX tensor contact views; a dedicated sensor calibration is still pending.",
                "No locomotion, navigation, rendering or real robot is involved.",
            ],
        }

    def _finish(self, status: str, error: str | None = None) -> None:
        if self.completed:
            return
        self.last_error = error
        self._write_trace()
        result = self._result(status)
        _atomic_json(self.receipt, result)
        self._request_stop()
        self.status = result["status"]
        self.completed = True

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
            if len(pose) != 7 or len(twist) != 6 or not all(math.isfinite(value) for value in (*pose, *twist)):
                raise ValueError("floating base state is invalid")
            positions, velocities = self.sim.get_joint_states()
            q_all = tuple(float(value) for value in positions)
            dq_all = tuple(float(value) for value in velocities)
            q = tuple(q_all[self.joint_index[name]] for name in X2_STAND_JOINT_NAMES)
            dq = tuple(dq_all[self.joint_index[name]] for name in X2_STAND_JOINT_NAMES)
            if not all(math.isfinite(value) for value in (*q_all, *dq_all)):
                raise ValueError("joint state contains NaN or Inf")
            self.last_all_position = q_all
            self.last_all_velocity = dq_all
            if self.first_sim_time is None:
                self.first_sim_time = now
                self.next_policy_time = now
                self.baseline = q
                self.baseline_all = q_all
            elapsed = now - self.first_sim_time
            phase = self._phase(elapsed)
            # The capture clock and band controller start at slightly
            # different points in EngineSession startup, while band status is
            # intentionally sampled at 10 Hz.  Never infer release duration
            # from the nominal phase boundary: keep applying standing effort
            # until the authoritative band receipt itself proves five full
            # seconds at exact zero support.
            if phase == "COMPLETE" and self._observed_unsupported_hold_s() < self.unsupported_hold_s:
                phase = "UNSUPPORTED_HOLD"
            qw, qx, qy, qz = pose[3:7]
            tilt = math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (qx * qx + qy * qy))))
            base_record = {
                "physics_sequence": self.physics_sequence,
                "sim_time": now,
                "phase": phase,
                "position_m": list(pose[:3]),
                "orientation_wxyz": list(pose[3:7]),
                "z_m": pose[2],
                "tilt_rad": tilt,
                "linear_speed_mps": math.sqrt(sum(value * value for value in twist[:3])),
                "angular_speed_rad_s": math.sqrt(sum(value * value for value in twist[3:])),
            }
            self.base_records.append(base_record)
            if self.contact_reader is not None:
                contact = dict(self.contact_reader(now))
                contact["phase"] = phase
                self.contact_records.append(contact)
            self.trace_records.append({
                "physics_sequence": self.physics_sequence,
                "sim_time": now,
                "phase": phase,
                "position": list(q),
                "velocity": list(dq),
            })
            self.last_sim_time = now
            if phase == "COMPLETE":
                self._finish("PASSED")
                return
            if phase == "BAND_RAMP_DOWN":
                self._request_ramp_down()
            if now + 1e-9 >= float(self.next_policy_time):
                q_observation = tuple(value - default for value, default in zip(q, _OBSERVATION_DEFAULT))
                policy_state = X2StandPhysicsState(
                    source_sequence=self.physics_sequence,
                    sim_time=now,
                    joint_position=q_observation,
                    joint_velocity=tuple(value * 0.05 for value in dq),
                    base_angular_velocity=tuple(twist[3:6]),
                    observation_gravity=_project_gravity_with_vendor_pitch_offset(tuple(pose[3:7])),
                    velocity_command=(0.0, 0.0, 0.0),
                    pose_command=(0.0, 0.0, 0.0, 0.0),
                    arm_target_position=X2_STAND_ARM_TARGET,
                    waist_action_observation=(0.0, 0.0),
                )
                result = self.session.update(policy_state)
                self.last_target = result.command.position_target
                self.last_kp = result.command.kp
                self.last_kd = result.command.kd
                while float(self.next_policy_time) <= now + 1e-9:
                    self.next_policy_time = float(self.next_policy_time) + self.policy_period_s
            if self.last_target is None or self.last_kp is None or self.last_kd is None:
                raise RuntimeError("floating stand has no STAND_DEFAULT target")
            self._apply(
                self.last_target, q_all, dq_all, self.last_kp, self.last_kd, phase, now
            )
            if pose[2] < 0.40 or tilt > 0.80:
                raise RuntimeError(f"floating stand safety limit crossed: z={pose[2]:.4f}, tilt={tilt:.4f}")
        except Exception as exc:
            self._finish("FAILED", f"{type(exc).__name__}: {exc}")
