"""Bounded full-joint STAND_DEFAULT capture in authoritative fixed-base PhysX."""

from __future__ import annotations

import hashlib
import json
import math
import os
import struct
import tempfile
import time
from pathlib import Path

from .x2_stand import (
    StateDrivenX2StandSession,
    X2StandPhysicsState,
    X2_STAND_ARM_TARGET,
    X2_STAND_JOINT_NAMES,
    X2_STAND_NOMINAL,
)


SCHEMA = "sceneforge/x2-geniesim-fixed-base-stand-default/v1"
POLICY_RATE_HZ = 100.0
MAXIMUM_TOTAL_DURATION_S = 6.0
_OBSERVATION_DEFAULT = (*X2_STAND_NOMINAL[:12], 0.0, 0.0, 0.0, *(0.0,) * 8)
_AUXILIARY_JOINTS = (
    "left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
    "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint",
    "head_yaw_joint", "head_pitch_joint",
)


def _effort_limit(name: str) -> float:
    if "hip" in name or "knee" in name or name == "waist_yaw_joint":
        return 120.0
    if (
        "ankle_pitch" in name
        or "shoulder_pitch" in name
        or "shoulder_roll" in name
    ):
        return 36.0
    if (
        "ankle_roll" in name
        or name in {"waist_roll_joint", "waist_pitch_joint"}
        or "elbow" in name
        or "shoulder_yaw" in name
        or "wrist_yaw" in name
    ):
        return 24.0
    if "wrist" in name:
        return 4.8
    if name == "head_yaw_joint":
        return 2.6
    if name == "head_pitch_joint":
        return 0.6
    raise ValueError(f"no frozen X2 effort limit for {name}")


def _atomic_json(path: Path, payload: object, mode: int = 0o444) -> None:
    if path.exists():
        raise FileExistsError(f"refusing to overwrite {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
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


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _percentile(values: list[float], quantile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = quantile * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


class _StandOnnxPolicy:
    def __init__(self, model: Path) -> None:
        import numpy as np
        import onnxruntime as ort

        options = ort.SessionOptions()
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.np = np
        self.ort = ort
        self.session = ort.InferenceSession(
            str(model), sess_options=options, providers=["CPUExecutionProvider"]
        )
        inputs = self.session.get_inputs()
        outputs = self.session.get_outputs()
        if len(inputs) != 1 or not outputs:
            raise ValueError("STAND_DEFAULT policy must have one input and at least one output")
        self.input = inputs[0]
        self.output = outputs[0]
        self.elapsed_ms: list[float] = []
        self.digest = hashlib.sha256()

    def __call__(self, values: tuple[float, ...]) -> tuple[float, ...]:
        matrix = self.np.asarray(values, dtype=self.np.float32).reshape(1, -1)
        started = time.perf_counter()
        raw = self.session.run([self.output.name], {self.input.name: matrix})[0]
        self.elapsed_ms.append((time.perf_counter() - started) * 1000.0)
        flat = self.np.asarray(raw, dtype=self.np.float32).reshape(-1)
        if flat.size != 14 or not self.np.isfinite(flat).all():
            raise ValueError("STAND_DEFAULT ONNX output must contain 14 finite values")
        result = tuple(float(value) for value in flat)
        self.digest.update(struct.pack("<14f", *result))
        return result


class X2FixedBaseStandCapture:
    """Apply the independent STAND_DEFAULT adapter to all 23 released joints."""

    def __init__(
        self,
        sim,
        *,
        model: str | Path,
        receipt: str | Path,
        trace: str | Path,
        stop_request: str | Path,
        warmup_s: float = 0.5,
        active_s: float = 0.5,
        restore_s: float = 5.0,
    ) -> None:
        if not all(math.isfinite(value) and value > 0.0 for value in (warmup_s, active_s, restore_s)):
            raise ValueError("stand capture durations must be finite and positive")
        if warmup_s + active_s + restore_s > MAXIMUM_TOTAL_DURATION_S:
            raise ValueError("stand capture duration exceeds the hard limit")
        self.sim = sim
        self.model = Path(model).resolve()
        self.receipt = Path(receipt).resolve()
        self.trace = Path(trace).resolve()
        self.stop_request = Path(stop_request).resolve()
        if not self.model.is_file():
            raise FileNotFoundError(self.model)
        if len({self.receipt, self.trace, self.stop_request}) != 3:
            raise ValueError("stand output paths must be distinct")
        if any(path.exists() for path in (self.receipt, self.trace, self.stop_request)):
            raise FileExistsError("stand output path already exists")

        self.warmup_s = float(warmup_s)
        self.active_s = float(active_s)
        self.restore_s = float(restore_s)
        self.total_duration_s = self.warmup_s + self.active_s + self.restore_s
        self.policy_period_s = 1.0 / POLICY_RATE_HZ
        names = tuple(str(name) for name in sim.joint_names)
        missing = sorted(set(X2_STAND_JOINT_NAMES) - set(names))
        if missing:
            raise ValueError(f"PhysX articulation is missing STAND_DEFAULT joints: {missing}")
        unexpected = sorted(set(names) - set(X2_STAND_JOINT_NAMES) - set(_AUXILIARY_JOINTS))
        missing_auxiliary = sorted(set(_AUXILIARY_JOINTS) - set(names))
        if unexpected or missing_auxiliary or len(names) != 31:
            raise ValueError(
                f"frozen X2 articulation mismatch: unexpected={unexpected}, "
                f"missing_auxiliary={missing_auxiliary}, count={len(names)}"
            )
        self.articulation_joint_names = names
        self.articulation_joint_index = {name: index for index, name in enumerate(names)}
        self.joint_index = {name: names.index(name) for name in X2_STAND_JOINT_NAMES}
        self.policy = _StandOnnxPolicy(self.model)
        self.session = StateDrivenX2StandSession(self.policy)

        self.physics_sequence = 0
        self.command_sequence = 0
        self.first_sim_time: float | None = None
        self.last_sim_time: float | None = None
        self.next_policy_time: float | None = None
        self.baseline: tuple[float, ...] | None = None
        self.baseline_all: tuple[float, ...] | None = None
        self.last_target: tuple[float, ...] | None = None
        self.last_kp: tuple[float, ...] | None = None
        self.last_kd: tuple[float, ...] | None = None
        self.last_all_position: tuple[float, ...] | None = None
        self.last_all_velocity: tuple[float, ...] | None = None
        self.effort_sample_count = 0
        self.effort_saturation_count = 0
        self.maximum_raw_effort_ratio = 0.0
        self.active_tracking_errors: list[float] = []
        self.restore_tracking_errors: list[float] = []
        self.maximum_excursion = [0.0] * len(X2_STAND_JOINT_NAMES)
        self.trace_records: list[dict] = []
        self.command_records: list[dict] = []
        self.completed = False
        self.status = "RUNNING"
        self.last_error: str | None = None

    def _phase(self, elapsed: float) -> str:
        if elapsed < self.warmup_s:
            return "WARMUP"
        if elapsed < self.warmup_s + self.active_s:
            return "ACTIVE"
        if elapsed < self.total_duration_s:
            return "RESTORE"
        return "COMPLETE"

    def _apply(
        self,
        target: tuple[float, ...],
        q_all: tuple[float, ...],
        dq_all: tuple[float, ...],
        kp: tuple[float, ...],
        kd: tuple[float, ...],
        phase: str,
        sim_time: float,
    ) -> None:
        if phase not in {
            "ACTIVE", "RESTORE", "SAFE_RESTORE",
            "SUPPORTED_STAND", "BAND_RAMP_DOWN", "UNSUPPORTED_HOLD",
            "UNSUPPORTED_STAND",
            "GAIT1_FORWARD", "DECELERATE", "CPG_STAND_HANDOFF", "STOP_STAND",
        }:
            raise ValueError(f"command application forbidden in phase {phase}")
        if self.baseline_all is None:
            raise RuntimeError("effort command requires a complete articulation baseline")
        target_by_name = dict(zip(X2_STAND_JOINT_NAMES, target))
        kp_by_name = dict(zip(X2_STAND_JOINT_NAMES, kp))
        kd_by_name = dict(zip(X2_STAND_JOINT_NAMES, kd))
        efforts: dict[str, float] = {}
        maximum_ratio = 0.0
        saturated = 0
        for name in self.articulation_joint_names:
            index = self.articulation_joint_index[name]
            if name in target_by_name:
                desired = target_by_name[name]
                stiffness = kp_by_name[name]
                damping = kd_by_name[name]
            else:
                desired = self.baseline_all[index]
                stiffness = 20.0 if "wrist" in name else 10.0
                damping = 1.0
            raw_effort = stiffness * (desired - q_all[index]) - damping * dq_all[index]
            if not all(math.isfinite(value) for value in (
                desired, stiffness, damping, q_all[index], dq_all[index], raw_effort
            )):
                raise ValueError(f"non-finite PhysX PD value for {name}")
            limit = _effort_limit(name)
            ratio = abs(raw_effort) / limit
            maximum_ratio = max(maximum_ratio, ratio)
            self.effort_sample_count += 1
            if ratio >= 1.0:
                saturated += 1
                self.effort_saturation_count += 1
            efforts[name] = max(-limit, min(limit, raw_effort))
        if tuple(efforts) != self.articulation_joint_names:
            raise RuntimeError("effort command scope escaped the frozen 31-joint order")
        self.maximum_raw_effort_ratio = max(self.maximum_raw_effort_ratio, maximum_ratio)
        self.sim.apply_commands(
            cmd_positions={},
            cmd_efforts=efforts,
            cmd_4ws_steer_pos={},
            cmd_4ws_drive_vel={},
            cmd_4ws_stamp=0.0,
        )
        self.command_sequence += 1
        self.last_target = target
        self.command_records.append({
            "command_sequence": self.command_sequence,
            "source_physics_sequence": self.physics_sequence,
            "sim_time": sim_time,
            "phase": phase,
            "position_command_count": 0,
            "effort_command_count": len(efforts),
            "command_age_sim_time_s": 0.0,
            "position_target": list(target),
            "joint_names": list(self.articulation_joint_names),
            "articulation_position_target": [
                target_by_name.get(name, self.baseline_all[self.articulation_joint_index[name]])
                for name in self.articulation_joint_names
            ],
            "effort_target": [efforts[name] for name in self.articulation_joint_names],
            "maximum_raw_effort_ratio": maximum_ratio,
            "saturated_joint_count": saturated,
        })

    def _request_stop(self) -> None:
        _atomic_json(self.stop_request, {
            "schema": "sceneforge/physx-controlled-stop-request/v1",
            "request": "STOP",
            "requested_by": "x2_fixed_base_stand_default_capture",
            "real_robot_commands": False,
        }, mode=0o644)

    def _write_trace(self) -> None:
        _atomic_json(self.trace, {
            "schema": "sceneforge/x2-fixed-base-stand-default-trace/v1",
            "joint_names": list(X2_STAND_JOINT_NAMES),
            "physics_samples": self.trace_records,
            "commands": self.command_records,
            "real_robot_commands": False,
        })

    def _result(self, requested_status: str) -> dict:
        moved = sum(value >= 0.01 for value in self.maximum_excursion)
        active_p95 = _percentile(self.active_tracking_errors, 0.95)
        restore_p95 = _percentile(self.restore_tracking_errors[-100:], 0.95)
        duration = (
            float(self.last_sim_time) - float(self.first_sim_time)
            if self.first_sim_time is not None and self.last_sim_time is not None
            else 0.0
        )
        checks = {
            "fixed_base": True,
            "minimum_policy_calls": len(self.policy.elapsed_ms) >= 40,
            "five_hundred_hz_pd_over_one_hundred_hz_policy": (
                len(self.policy.elapsed_ms) > 0
                and 4.8 <= sum(
                    record["phase"] == "ACTIVE" for record in self.command_records
                ) / len(self.policy.elapsed_ms) <= 5.2
            ),
            "all_31_efforts_only": all(
                record["position_command_count"] == 0 and record["effort_command_count"] == 31
                for record in self.command_records
            ),
            "at_least_12_joints_responded": moved >= 12,
            # Fixed-base dynamics intentionally remove the policy's base
            # response, so this is a transport bound rather than a balance
            # quality metric.  Floating-base standing owns the tighter gate.
            "active_tracking_error_p95_bounded": active_p95 is not None and active_p95 <= 0.50,
            "restore_tracking_error_p95_bounded": restore_p95 is not None and restore_p95 <= 0.08,
            "restore_velocity_bounded": (
                bool(self.trace_records)
                and max(abs(value) for value in self.trace_records[-1]["velocity"]) <= 0.10
            ),
            "joint_speed_bounded": max(
                (abs(value) for record in self.trace_records for value in record["velocity"]),
                default=0.0,
            ) <= 15.0,
            "effort_saturation_ratio_bounded": (
                self.effort_sample_count > 0
                and self.effort_saturation_count / self.effort_sample_count <= 0.10
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
                "proceed_to_suspension_assisted_floating_stand"
                if final_status == "PASSED"
                else "remain_at_fixed_base_stand_default"
            ),
            "qualification": False,
            "physics_authority": "genie_sim_isaac_physx",
            "base_mode": "fixed",
            "command_transport": "in_process_sim_time_state_to_next_physx_step",
            "published_command_topics": [],
            "real_robot_commands": False,
            "model": {
                "path": str(self.model),
                "sha256": _sha256(self.model),
                "onnxruntime_version": self.policy.ort.__version__,
                "output_sha256": self.policy.digest.hexdigest(),
            },
            "parameters": {
                "policy_rate_hz": POLICY_RATE_HZ,
                "pd_rate_hz": 500.0,
                "warmup_s": self.warmup_s,
                "active_s": self.active_s,
                "restore_s": self.restore_s,
                "joint_count": len(X2_STAND_JOINT_NAMES),
                "articulation_joint_count": len(self.articulation_joint_names),
                "dynamic_gain_changes": False,
            },
            "metrics": {
                "physics_step_count": self.physics_sequence,
                "policy_call_count": len(self.policy.elapsed_ms),
                "command_application_calls": len(self.command_records),
                "sampled_sim_duration_s": duration,
                "responding_joint_count": moved,
                "maximum_joint_excursion_rad": max(self.maximum_excursion, default=0.0),
                "maximum_joint_speed_rad_s": max(
                    (abs(value) for record in self.trace_records for value in record["velocity"]),
                    default=0.0,
                ),
                "active_tracking_error_p95_rad": active_p95,
                "restore_tracking_error_p95_rad": restore_p95,
                "final_maximum_joint_speed_rad_s": (
                    max(abs(value) for value in self.trace_records[-1]["velocity"])
                    if self.trace_records else None
                ),
                "effort_sample_count": self.effort_sample_count,
                "effort_saturation_count": self.effort_saturation_count,
                "effort_saturation_ratio": (
                    self.effort_saturation_count / self.effort_sample_count
                    if self.effort_sample_count else None
                ),
                "maximum_raw_effort_ratio": self.maximum_raw_effort_ratio,
                "inference_wall_p95_ms": _percentile(self.policy.elapsed_ms, 0.95),
            },
            "checks": checks,
            "trace": {
                "path": str(self.trace),
                "sha256": _sha256(self.trace) if self.trace.is_file() else None,
            },
            "last_error": self.last_error,
            "limitations": [
                "This is a fixed-base full-joint controller/output gate, not standing qualification.",
                "It uses vendor position-PD gains as bounded PhysX efforts after atomic drive handover.",
                "Base dynamics, foot contact, suspension release and fall detection are not exercised.",
            ],
        }

    def _finish(self, status: str, error: str | None = None) -> None:
        if self.completed:
            return
        self.last_error = error
        if (
            self.baseline is not None
            and self.last_kp is not None
            and self.last_kd is not None
            and self.last_all_position is not None
            and self.last_all_velocity is not None
        ):
            try:
                self._apply(
                    self.baseline,
                    self.last_all_position,
                    self.last_all_velocity,
                    self.last_kp,
                    self.last_kd,
                    "SAFE_RESTORE",
                    float(self.last_sim_time or 0.0),
                )
            except Exception as restore_error:
                detail = f"safe restore failed: {type(restore_error).__name__}: {restore_error}"
                self.last_error = f"{self.last_error}; {detail}" if self.last_error else detail
                status = "FAILED"
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
                self.next_policy_time = now + self.warmup_s
            elapsed = now - self.first_sim_time
            phase = self._phase(elapsed)
            if phase == "WARMUP":
                self.baseline = q
                self.baseline_all = q_all
            elif self.baseline is None:
                raise RuntimeError("warmup did not establish a baseline")
            for index, (value, baseline) in enumerate(zip(q, self.baseline or q)):
                self.maximum_excursion[index] = max(self.maximum_excursion[index], abs(value - baseline))
            if phase == "ACTIVE" and self.last_target is not None:
                self.active_tracking_errors.extend(abs(value - target) for value, target in zip(q, self.last_target))
            elif phase in {"RESTORE", "COMPLETE"} and self.baseline is not None:
                self.restore_tracking_errors.extend(abs(value - target) for value, target in zip(q, self.baseline))
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
            if phase == "ACTIVE":
                if now + 1e-9 >= float(self.next_policy_time):
                    q_observation = tuple(value - default for value, default in zip(q, _OBSERVATION_DEFAULT))
                    state = X2StandPhysicsState(
                        source_sequence=self.physics_sequence,
                        sim_time=now,
                        joint_position=q_observation,
                        joint_velocity=tuple(value * 0.05 for value in dq),
                        base_angular_velocity=(0.0, 0.0, 0.0),
                        observation_gravity=(-math.sin(0.03), 0.0, -math.cos(0.03)),
                        velocity_command=(0.0, 0.0, 0.0),
                        pose_command=(0.0, 0.0, 0.0, 0.0),
                        arm_target_position=X2_STAND_ARM_TARGET,
                        waist_action_observation=(0.0, 0.0),
                    )
                    result = self.session.update(state)
                    self.last_target = result.command.position_target
                    self.last_kp = result.command.kp
                    self.last_kd = result.command.kd
                    while float(self.next_policy_time) <= now + 1e-9:
                        self.next_policy_time = float(self.next_policy_time) + self.policy_period_s
                if self.last_target is None or self.last_kp is None or self.last_kd is None:
                    raise RuntimeError("active phase has no STAND_DEFAULT target")
                # The policy target updates at 100 Hz; position-PD is evaluated
                # against every authoritative 500 Hz PhysX state.
                self._apply(
                    self.last_target, q_all, dq_all, self.last_kp, self.last_kd, phase, now
                )
            elif phase == "RESTORE":
                if self.last_kp is None or self.last_kd is None:
                    raise RuntimeError("restore began before STAND_DEFAULT produced gains")
                self._apply(
                    self.baseline, q_all, dq_all, self.last_kp, self.last_kd, phase, now
                )
        except Exception as exc:
            self._finish("FAILED", f"{type(exc).__name__}: {exc}")

    def abort(self, reason: str) -> None:
        self._finish("FAILED", reason)
