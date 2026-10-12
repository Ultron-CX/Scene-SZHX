"""Bounded single-joint command capture inside the authoritative PhysX loop.

This is deliberately smaller than a locomotion controller adapter.  It proves
that one position target, derived from one authoritative post-step state, is
accepted by the Genie Sim PhysX articulation and produces a bounded response.
It never publishes ROS commands and it cannot address more than one joint.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import tempfile
from pathlib import Path


SCHEMA = "sceneforge/x2-geniesim-fixed-base-command-capture/v1"
DEFAULT_JOINT = "left_shoulder_pitch_joint"
MAXIMUM_AMPLITUDE_RAD = 0.03
MAXIMUM_TOTAL_DURATION_S = 5.0


def _atomic_json(path: Path, payload: object, mode: int = 0o444) -> None:
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


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class X2FixedBaseCommandCapture:
    """Apply one bounded joint target and restore the measured baseline."""

    def __init__(
        self,
        sim,
        *,
        receipt: str | Path,
        trace: str | Path,
        stop_request: str | Path,
        joint_name: str = DEFAULT_JOINT,
        amplitude_rad: float = 0.02,
        command_rate_hz: float = 50.0,
        warmup_s: float = 0.5,
        excite_s: float = 1.0,
        restore_s: float = 1.0,
        minimum_command_count: int = 80,
    ) -> None:
        numeric = (
            amplitude_rad,
            command_rate_hz,
            warmup_s,
            excite_s,
            restore_s,
        )
        if not all(math.isfinite(float(value)) for value in numeric):
            raise ValueError("capture parameters contain NaN or Inf")
        if not 0.0 < abs(amplitude_rad) <= MAXIMUM_AMPLITUDE_RAD:
            raise ValueError("amplitude exceeds the bounded capture limit")
        if command_rate_hz <= 0 or min(warmup_s, excite_s, restore_s) <= 0:
            raise ValueError("capture rates and durations must be positive")
        if warmup_s + excite_s + restore_s > MAXIMUM_TOTAL_DURATION_S:
            raise ValueError("capture duration exceeds the hard limit")
        if minimum_command_count < 1:
            raise ValueError("minimum_command_count must be positive")
        if joint_name != DEFAULT_JOINT:
            raise ValueError(f"this gate is locked to {DEFAULT_JOINT}")

        self.sim = sim
        self.receipt = Path(receipt).resolve()
        self.trace = Path(trace).resolve()
        self.stop_request = Path(stop_request).resolve()
        if len({self.receipt, self.trace, self.stop_request}) != 3:
            raise ValueError("capture output paths must be distinct")
        if any(path.exists() for path in (self.receipt, self.trace, self.stop_request)):
            raise FileExistsError("capture output path already exists")

        self.joint_name = joint_name
        self.amplitude_rad = float(amplitude_rad)
        self.command_rate_hz = float(command_rate_hz)
        self.command_period_s = 1.0 / self.command_rate_hz
        self.warmup_s = float(warmup_s)
        self.excite_s = float(excite_s)
        self.restore_s = float(restore_s)
        self.minimum_command_count = int(minimum_command_count)
        self.total_duration_s = self.warmup_s + self.excite_s + self.restore_s

        names = tuple(str(name) for name in sim.joint_names)
        if self.joint_name not in names:
            raise ValueError(f"PhysX articulation is missing {self.joint_name}")
        self.joint_names = names
        self.joint_index = names.index(self.joint_name)

        self.physics_sequence = 0
        self.command_sequence = 0
        self.command_application_calls = 0
        self.first_sim_time: float | None = None
        self.last_sim_time: float | None = None
        self.next_command_time: float | None = None
        self.baseline_position: float | None = None
        self.baseline_all: tuple[float, ...] | None = None
        self.last_target: float | None = None
        self.excite_positions: list[float] = []
        self.restore_positions: list[float] = []
        self.maximum_other_joint_excursion = 0.0
        self.trace_records: list[dict] = []
        self.command_records: list[dict] = []
        self.completed = False
        self.status = "RUNNING"
        self.last_error: str | None = None

    def _phase(self, elapsed: float) -> str:
        if elapsed < self.warmup_s:
            return "WARMUP"
        if elapsed < self.warmup_s + self.excite_s:
            return "EXCITE"
        if elapsed < self.total_duration_s:
            return "RESTORE"
        return "COMPLETE"

    def _apply_target(self, target: float, phase: str, sim_time: float) -> None:
        if phase not in {"EXCITE", "RESTORE", "SAFE_RESTORE"}:
            raise ValueError(f"command application forbidden in phase {phase}")
        packet = {self.joint_name: float(target)}
        if tuple(packet) != (DEFAULT_JOINT,):
            raise RuntimeError("command scope escaped the single-joint allowlist")
        self.sim.apply_commands(
            cmd_positions=packet,
            cmd_efforts={},
            cmd_4ws_steer_pos={},
            cmd_4ws_drive_vel={},
            cmd_4ws_stamp=0.0,
        )
        self.command_sequence += 1
        self.command_application_calls += 1
        self.last_target = float(target)
        self.command_records.append(
            {
                "command_sequence": self.command_sequence,
                "source_physics_sequence": self.physics_sequence,
                "sim_time": sim_time,
                "phase": phase,
                "joint_name": self.joint_name,
                "position_target_rad": float(target),
                "position_command_count": 1,
                "effort_command_count": 0,
                "command_age_sim_time_s": 0.0,
            }
        )

    def _request_stop(self) -> None:
        _atomic_json(
            self.stop_request,
            {
                "schema": "sceneforge/physx-controlled-stop-request/v1",
                "request": "STOP",
                "requested_by": "x2_fixed_base_single_joint_capture",
                "real_robot_commands": False,
            },
            mode=0o644,
        )

    def _write_trace(self) -> None:
        _atomic_json(
            self.trace,
            {
                "schema": "sceneforge/x2-fixed-base-command-trace/v1",
                "joint_name": self.joint_name,
                "physics_samples": self.trace_records,
                "commands": self.command_records,
                "real_robot_commands": False,
            },
        )

    def _result(self, requested_status: str) -> dict:
        baseline = self.baseline_position
        target = None if baseline is None else baseline + self.amplitude_rad
        direction = 1.0 if self.amplitude_rad > 0 else -1.0
        peak_response = (
            max((value - baseline) * direction for value in self.excite_positions)
            if baseline is not None and self.excite_positions
            else None
        )
        final_position = self.restore_positions[-1] if self.restore_positions else None
        final_error = (
            abs(final_position - baseline)
            if baseline is not None and final_position is not None
            else None
        )
        elapsed = (
            self.last_sim_time - self.first_sim_time
            if self.first_sim_time is not None and self.last_sim_time is not None
            else 0.0
        )
        checks = {
            "fixed_single_joint_allowlist": self.joint_name == DEFAULT_JOINT,
            "bounded_amplitude": 0.0 < abs(self.amplitude_rad) <= MAXIMUM_AMPLITUDE_RAD,
            "duration_reached": elapsed >= self.total_duration_s,
            "minimum_command_count": self.command_application_calls >= self.minimum_command_count,
            "one_position_target_per_packet": all(
                record["position_command_count"] == 1
                and record["effort_command_count"] == 0
                for record in self.command_records
            ),
            "command_source_sequence_strict": all(
                right["source_physics_sequence"] > left["source_physics_sequence"]
                for left, right in zip(self.command_records, self.command_records[1:])
            ),
            "bounded_positive_response": (
                peak_response is not None
                and peak_response >= abs(self.amplitude_rad) * 0.20
                and peak_response <= abs(self.amplitude_rad) * 2.50
            ),
            "returned_to_baseline": (
                final_error is not None and final_error <= abs(self.amplitude_rad) * 0.50
            ),
            "no_nonfinite_state": all(
                math.isfinite(record["position_rad"])
                and math.isfinite(record["velocity_rad_s"])
                for record in self.trace_records
            ),
            "no_runtime_error": self.last_error is None,
            "real_robot_commands_false": True,
        }
        final_status = (
            "PASSED"
            if requested_status == "PASSED" and all(checks.values())
            else "FAILED"
        )
        return {
            "schema": SCHEMA,
            "status": final_status,
            "decision": (
                "proceed_to_stand_default_output_capture"
                if final_status == "PASSED"
                else "remain_at_fixed_base_command_capture"
            ),
            "qualification": False,
            "physics_authority": "genie_sim_isaac_physx",
            "base_mode": "fixed",
            "command_transport": "in_process_post_step_to_next_physx_step",
            "published_command_topics": [],
            "real_robot_commands": False,
            "parameters": {
                "joint_name": self.joint_name,
                "amplitude_rad": self.amplitude_rad,
                "command_rate_hz": self.command_rate_hz,
                "warmup_s": self.warmup_s,
                "excite_s": self.excite_s,
                "restore_s": self.restore_s,
                "dynamic_gain_changes": False,
            },
            "metrics": {
                "physics_step_count": self.physics_sequence,
                "command_application_calls": self.command_application_calls,
                "first_sim_time": self.first_sim_time,
                "last_sim_time": self.last_sim_time,
                "sampled_sim_duration_s": elapsed,
                "baseline_position_rad": baseline,
                "excite_target_rad": target,
                "peak_directional_response_rad": peak_response,
                "final_position_rad": final_position,
                "final_restore_error_rad": final_error,
                "maximum_other_joint_excursion_rad": self.maximum_other_joint_excursion,
            },
            "checks": checks,
            "trace": {
                "path": str(self.trace),
                "sha256": _sha256(self.trace) if self.trace.is_file() else None,
            },
            "last_error": self.last_error,
            "limitations": [
                "This is a fixed-base, one-joint transport/response gate, not locomotion.",
                "It reuses the frozen USD articulation drive gains and does not tune them.",
                "It does not establish STAND_DEFAULT, floating-base stability, or MuJoCo equivalence.",
            ],
        }

    def _finish(self, status: str, error: str | None = None) -> None:
        if self.completed:
            return
        self.last_error = error
        if self.baseline_position is not None:
            try:
                self._apply_target(
                    self.baseline_position,
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
            if not math.isfinite(now) or now < 0:
                raise ValueError("post-step simulation time is invalid")
            if self.last_sim_time is not None and now <= self.last_sim_time:
                raise ValueError("post-step simulation time did not advance")
            positions, velocities = self.sim.get_joint_states()
            q = tuple(float(value) for value in positions)
            dq = tuple(float(value) for value in velocities)
            if len(q) != len(self.joint_names) or len(dq) != len(self.joint_names):
                raise ValueError("joint state width changed during capture")
            if not all(math.isfinite(value) for value in (*q, *dq)):
                raise ValueError("joint state contains NaN or Inf")

            if self.first_sim_time is None:
                self.first_sim_time = now
                self.next_command_time = now + self.warmup_s
            elapsed = now - self.first_sim_time
            phase = self._phase(elapsed)
            position = q[self.joint_index]
            velocity = dq[self.joint_index]
            if phase == "WARMUP":
                self.baseline_position = position
                self.baseline_all = q
            elif self.baseline_position is None or self.baseline_all is None:
                raise RuntimeError("warmup did not establish a baseline")

            if self.baseline_all is not None:
                excursions = [
                    abs(value - baseline)
                    for index, (value, baseline) in enumerate(zip(q, self.baseline_all))
                    if index != self.joint_index
                ]
                self.maximum_other_joint_excursion = max(
                    self.maximum_other_joint_excursion, max(excursions, default=0.0)
                )
            if phase == "EXCITE":
                self.excite_positions.append(position)
            elif phase in {"RESTORE", "COMPLETE"}:
                self.restore_positions.append(position)

            self.trace_records.append(
                {
                    "physics_sequence": self.physics_sequence,
                    "sim_time": now,
                    "phase": phase,
                    "position_rad": position,
                    "velocity_rad_s": velocity,
                    "last_position_target_rad": self.last_target,
                }
            )
            self.last_sim_time = now

            if phase == "COMPLETE":
                self._finish("PASSED")
                return
            if phase != "WARMUP" and now + 1e-9 >= float(self.next_command_time):
                target = self.baseline_position + self.amplitude_rad if phase == "EXCITE" else self.baseline_position
                self._apply_target(target, phase, now)
                while float(self.next_command_time) <= now + 1e-9:
                    self.next_command_time = float(self.next_command_time) + self.command_period_s
        except Exception as exc:
            self._finish("FAILED", f"{type(exc).__name__}: {exc}")

    def abort(self, reason: str) -> None:
        self._finish("FAILED", reason)
