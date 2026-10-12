"""Read-only X2 policy shadow executed inside the Genie Sim physics loop."""

from __future__ import annotations

import hashlib
import json
import math
import os
import struct
import tempfile
import time
from pathlib import Path

from .x2_control import X2_CPGWALK_JOINT_NAMES
from .x2_session import StateDrivenX2ControllerSession, X2PhysicsState


SCHEMA = "sceneforge/x2-geniesim-inprocess-shadow/v1"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_json(path: Path, payload: dict, mode: int = 0o444) -> None:
    if path.exists():
        raise FileExistsError(f"refusing to overwrite {path}")
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
        os.chmod(path, mode)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


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


class _OnnxPolicy:
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
            raise ValueError("X2 policy must have one input and at least one output")
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
        if flat.size != 17 or not self.np.isfinite(flat).all():
            raise ValueError("ONNX output must contain 17 finite values")
        values_out = tuple(float(value) for value in flat)
        self.digest.update(struct.pack("<17f", *values_out))
        return values_out


class X2InProcessShadow:
    """A post-step hook that observes PhysX but never applies a command."""

    def __init__(
        self,
        sim,
        *,
        model: str | Path,
        receipt: str | Path,
        stop_request: str | Path,
        policy_rate_hz: float = 50.0,
        sim_duration_s: float = 10.0,
        minimum_inferences: int = 400,
        velocity_command: tuple[float, float, float] = (0.3, 0.0, 0.0),
    ) -> None:
        if policy_rate_hz <= 0 or sim_duration_s <= 0 or minimum_inferences < 1:
            raise ValueError("invalid shadow rate, duration or inference gate")
        self.sim = sim
        self.model = Path(model).resolve()
        self.receipt = Path(receipt).resolve()
        self.stop_request = Path(stop_request).resolve()
        if not self.model.is_file():
            raise FileNotFoundError(self.model)
        if self.receipt.exists() or self.stop_request.exists():
            raise FileExistsError("shadow output path already exists")
        self.policy_rate_hz = float(policy_rate_hz)
        self.policy_period = 1.0 / self.policy_rate_hz
        self.sim_duration_s = float(sim_duration_s)
        self.minimum_inferences = int(minimum_inferences)
        self.velocity_command = tuple(float(value) for value in velocity_command)
        if len(self.velocity_command) != 3 or not all(map(math.isfinite, self.velocity_command)):
            raise ValueError("velocity command must contain three finite values")
        self.joint_index = {name: index for index, name in enumerate(sim.joint_names)}
        missing = sorted(set(X2_CPGWALK_JOINT_NAMES) - set(self.joint_index))
        if missing:
            raise ValueError(f"PhysX articulation is missing CPGWalk joints: {missing}")
        self.policy = _OnnxPolicy(self.model)
        self.session = StateDrivenX2ControllerSession(self.policy)
        self.physics_sequence = 0
        self.inference_count = 0
        self.scheduler_skip_count = 0
        self.first_sim_time: float | None = None
        self.last_sim_time: float | None = None
        self.next_policy_time: float | None = None
        self.sample_gaps: list[float] = []
        self.max_output_abs = 0.0
        self.completed = False
        self.status = "RUNNING"
        self.last_error: str | None = None

    def _request_stop(self) -> None:
        _atomic_json(
            self.stop_request,
            {
                "schema": "sceneforge/physx-controlled-stop-request/v1",
                "request": "STOP",
                "requested_by": "x2_inprocess_readonly_shadow",
                "real_robot_commands": False,
            },
            mode=0o644,
        )

    def _result(self, status: str) -> dict:
        duration = (
            self.last_sim_time - self.first_sim_time
            if self.first_sim_time is not None and self.last_sim_time is not None
            else 0.0
        )
        rate = (
            (self.inference_count - 1) / duration
            if self.inference_count > 1 and duration > 0
            else None
        )
        checks = {
            "minimum_inferences": self.inference_count >= self.minimum_inferences,
            "sim_duration_reached": duration >= self.sim_duration_s,
            "one_policy_call_per_sample": len(self.policy.elapsed_ms) == self.inference_count,
            "policy_rate_within_one_percent": (
                rate is not None
                and self.policy_rate_hz * 0.99 <= rate <= self.policy_rate_hz * 1.01
            ),
            "no_session_error": self.last_error is None,
            "no_command_application": True,
        }
        final_status = "PASSED" if status == "PASSED" and all(checks.values()) else "FAILED"
        return {
            "schema": SCHEMA,
            "status": final_status,
            "decision": (
                "inprocess_physx_state_shadow_passed"
                if final_status == "PASSED"
                else "remain_read_only"
            ),
            "qualification": False,
            "read_only": True,
            "authoritative_physics_source": "geniesim_isaac_physx",
            "state_transport": "in_process_post_step_hook",
            "application_published_topics": [],
            "command_application_calls": 0,
            "real_robot_commands": False,
            "model": {
                "path": str(self.model),
                "sha256": _sha256(self.model),
                "onnxruntime_version": self.policy.ort.__version__,
                "input_name": self.policy.input.name,
                "input_shape": self.policy.input.shape,
                "output_name": self.policy.output.name,
                "output_shape": self.policy.output.shape,
            },
            "parameters": {
                "fixed_base": True,
                "policy_rate_hz": self.policy_rate_hz,
                "sim_duration_s": self.sim_duration_s,
                "minimum_inferences": self.minimum_inferences,
                "velocity_command": list(self.velocity_command),
            },
            "metrics": {
                "physics_step_count": self.physics_sequence,
                "inference_count": self.inference_count,
                "scheduler_skip_count": self.scheduler_skip_count,
                "first_sim_time": self.first_sim_time,
                "last_sim_time": self.last_sim_time,
                "sampled_sim_duration_s": duration,
                "inferred_policy_rate_hz": rate,
                "sample_gap_p95_s": _percentile(self.sample_gaps, 0.95),
                "sample_gap_max_s": max(self.sample_gaps, default=None),
                "inference_wall_p50_ms": _percentile(self.policy.elapsed_ms, 0.50),
                "inference_wall_p95_ms": _percentile(self.policy.elapsed_ms, 0.95),
                "inference_wall_max_ms": max(self.policy.elapsed_ms, default=None),
                "maximum_policy_output_abs": self.max_output_abs,
                "policy_output_sha256": self.policy.digest.hexdigest(),
            },
            "checks": checks,
            "last_error": self.last_error,
            "limitations": [
                "This fixed-base hook computes but never applies position-PD commands.",
                "Base pose, velocity and projected gravity are fixed identity/zero observations in this gate.",
                "Floating-base state and command application remain separate gates.",
            ],
        }

    def _finish(self, status: str, error: str | None = None) -> None:
        if self.completed:
            return
        self.last_error = error
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
            if not math.isfinite(now) or now < 0:
                raise ValueError("post-step simulation time is invalid")
            if self.next_policy_time is None:
                self.next_policy_time = now
            if now + 1e-9 < self.next_policy_time:
                self.scheduler_skip_count += 1
                return
            positions, velocities = self.sim.get_joint_states()
            q = tuple(float(positions[self.joint_index[name]]) for name in X2_CPGWALK_JOINT_NAMES)
            dq = tuple(float(velocities[self.joint_index[name]]) for name in X2_CPGWALK_JOINT_NAMES)
            result = self.session.update(X2PhysicsState(
                source_sequence=self.physics_sequence,
                sim_time=now,
                command_stamp_sim_time=now,
                velocity_command=self.velocity_command,
                joint_position=q,
                joint_velocity=dq,
                base_angular_velocity=(0.0, 0.0, 0.0),
                observation_gravity=(0.0, 0.0, -1.0),
                base_linear_velocity=(0.0, 0.0, 0.0),
                roll=0.0,
                pitch=0.0,
            ))
            if self.last_sim_time is not None:
                self.sample_gaps.append(now - self.last_sim_time)
            self.first_sim_time = now if self.first_sim_time is None else self.first_sim_time
            self.last_sim_time = now
            self.inference_count += 1
            self.max_output_abs = max(
                self.max_output_abs, *(abs(value) for value in result.policy_action)
            )
            while self.next_policy_time <= now + 1e-9:
                self.next_policy_time += self.policy_period
            if now - self.first_sim_time >= self.sim_duration_s:
                self._finish("PASSED")
        except Exception as exc:
            self._finish("FAILED", f"{type(exc).__name__}: {exc}")

    def abort(self, reason: str) -> None:
        self._finish("FAILED", reason)
