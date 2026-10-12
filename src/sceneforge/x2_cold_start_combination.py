"""Single-PhysX-lifecycle X2 straight/left/right qualification orchestration.

This module does not launch Docker or Isaac Sim.  It coordinates three
existing :class:`X2PhysxGait1Capture` instances inside one already-created
Genie Sim/PhysX engine session.  Intermediate captures are forbidden from
stopping the engine; the coordinator requests the existing controlled stop
only after the final case or immediately after a failed case.

The first case performs the physical elastic-band transition.  Continuation
cases start with an unsupported standing dwell and never re-engage or pretend
to ramp down the band.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Callable, Protocol

from .x2_mujoco_motion import X2MujocoMotionProfile
from .x2_physx_gait1 import X2PhysxGait1Capture


COMBINATION_SCHEMA = "sceneforge/x2-physx-cold-start-combination-runtime/v1"
FREEZE_SCHEMA = "sceneforge/x2-physx-combination-sim-time-freeze/v1"
SCENARIOS = ("straight", "turn_left", "turn_right")


@dataclass(frozen=True)
class X2ContinuationMotionProfile(X2MujocoMotionProfile):
    """A gait case after the first case has permanently released support."""

    supported_stand_s: float = 0.001
    band_ramp_s: float = 0.001
    unsupported_stand_s: float = 2.0

    @property
    def release_time_s(self) -> float:
        return 0.0

    @property
    def walk_start_s(self) -> float:
        return self.unsupported_stand_s

    def phase(self, sim_time_s: float) -> str:
        value = float(sim_time_s)
        if value < 0.0 or not math.isfinite(value):
            raise ValueError("sim time must be finite and non-negative")
        if value < self.walk_start_s:
            return "UNSUPPORTED_STAND"
        if value < self.deceleration_start_s:
            return "GAIT1_FORWARD"
        if value < self.cpg_stand_handoff_start_s:
            return "DECELERATE"
        if value < self.stop_start_s:
            return "CPG_STAND_HANDOFF"
        if value < self.total_duration_s:
            return "STOP_STAND"
        return "COMPLETE"


@dataclass(frozen=True)
class X2CombinationCasePlan:
    scenario: str
    profile: X2MujocoMotionProfile
    receipt: Path
    trace: Path
    performs_suspension_transition: bool

    def validate(self) -> None:
        if self.scenario not in SCENARIOS:
            raise ValueError(f"unsupported combination scenario: {self.scenario}")
        self.profile.validate()
        if self.receipt == self.trace:
            raise ValueError("case receipt and trace must be distinct")
        if self.receipt.exists() or self.trace.exists():
            raise FileExistsError("case evidence already exists")


class _Capture(Protocol):
    completed: bool
    status: str
    freeze_probe: dict[str, object] | None
    first_sim_time: float | None
    physics_sequence: int
    command_sequence: int
    walk_sequence: int
    walk_policy: object
    walk_session: object
    velocity_filter: object
    heading_hold: object
    cross_track_hold: object
    position_target_safety: object
    next_walk_time: float | None
    current_target: dict[str, float]
    current_kp: dict[str, float]
    current_kd: dict[str, float]
    last_policy_action: tuple[float, ...]
    trace_records: list[dict]
    base_records: list[dict]
    contact_records: list[dict]
    last_effective_command: tuple[float, float, float]
    last_cross_track_m: float
    last_cross_track_command_y_mps: float
    last_heading_hold_command_yaw_rps: float
    profile: X2MujocoMotionProfile

    def step(self, sim_time: float) -> None: ...

    def request_authoritative_stop(self) -> None: ...


class X2CombinationCaseCapture(X2PhysxGait1Capture):
    """Reuse the qualified capture while delegating engine stop to a coordinator."""

    def _request_stop(self) -> None:
        # Suppressed for every child.  The owning coordinator calls
        # request_authoritative_stop exactly once after transition handling.
        return None

    def request_authoritative_stop(self) -> None:
        super()._request_stop()


def _json_digest(payload: object) -> str:
    encoded = json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _controller_snapshot(capture: _Capture) -> dict[str, object]:
    hopf = capture.walk_session.hopf.snapshot()
    phase = math.atan2(float(hopf.obs4[0]), float(hopf.obs4[1]))
    snapshot = {
        "walk_session_digest": capture.walk_session.state_digest(),
        "hopf": asdict(hopf),
        "cpg_phase_rad": phase,
        "velocity_filter": capture.velocity_filter.snapshot(),
        "heading_hold": capture.heading_hold.snapshot(),
        "cross_track_hold": capture.cross_track_hold.snapshot(),
        "position_target_safety": capture.position_target_safety.snapshot(),
        "next_walk_time": capture.next_walk_time,
        "walk_sequence": capture.walk_sequence,
        "current_target": dict(capture.current_target),
        "current_kp": dict(capture.current_kp),
        "current_kd": dict(capture.current_kd),
        "last_policy_action": list(capture.last_policy_action),
        "walk_policy_call_count": len(capture.walk_policy.elapsed_ms),
        "trace_record_count": len(capture.trace_records),
        "base_record_count": len(capture.base_records),
        "contact_record_count": len(capture.contact_records),
        "last_effective_command": list(capture.last_effective_command),
        "last_cross_track_m": capture.last_cross_track_m,
        "last_cross_track_command_y_mps": (
            capture.last_cross_track_command_y_mps
        ),
        "last_heading_hold_command_yaw_rps": (
            capture.last_heading_hold_command_yaw_rps
        ),
    }
    snapshot["digest"] = _json_digest(snapshot)
    return snapshot


def _wrapped_angle_delta(after: float, before: float) -> float:
    return math.atan2(math.sin(after - before), math.cos(after - before))


def perform_sim_time_freeze_probe(
    capture: _Capture,
    *,
    sim_time: float,
    requested_wall_duration_s: float,
    sleep: Callable[[float], None] = time.sleep,
    perf_counter: Callable[[], float] = time.perf_counter,
) -> dict[str, object]:
    """Block the sole engine thread and measure every controller time lane."""

    duration = float(requested_wall_duration_s)
    if not math.isfinite(duration) or duration < 2.0:
        raise ValueError("qualification freeze must last at least two wall seconds")
    before = _controller_snapshot(capture)
    physics_before = int(capture.physics_sequence)
    command_before = int(capture.command_sequence)
    controller_before = int(capture.walk_sequence)
    policy_before = int(before["walk_policy_call_count"])
    wall_started = perf_counter()
    sleep(duration)
    wall_elapsed = perf_counter() - wall_started
    after = _controller_snapshot(capture)
    physics_after = int(capture.physics_sequence)
    command_after = int(capture.command_sequence)
    controller_after = int(capture.walk_sequence)
    policy_after = int(after["walk_policy_call_count"])
    phase_delta = _wrapped_angle_delta(
        float(after["cpg_phase_rad"]),
        float(before["cpg_phase_rad"]),
    )
    result = {
        "schema": FREEZE_SCHEMA,
        "status": "PASSED",
        "method": "single_engine_thread_wall_sleep_before_case_physics_callback",
        "requested_wall_duration_s": duration,
        "observed_wall_duration_s": wall_elapsed,
        "sim_time_before_s": float(sim_time),
        "sim_time_after_s": float(sim_time),
        "sim_time_advance_s": 0.0,
        # EngineSession publishes /clock and new state only inside the blocked
        # loop.  The invariant is recorded explicitly instead of sampled by a
        # second controller process.
        "clock_update_count": 0,
        "physics_state_update_count": physics_after - physics_before,
        "controller_update_count": controller_after - controller_before,
        "policy_update_count": policy_after - policy_before,
        "command_update_count": command_after - command_before,
        "cpg_phase_advance_rad": phase_delta,
        "controller_digest_before": before["digest"],
        "controller_digest_after": after["digest"],
        "controller_state_unchanged": before == after,
        "physics_sequence_unchanged": physics_before == physics_after,
        "command_sequence_unchanged": command_before == command_after,
        "sim_time_unchanged": True,
        "resume_completed": False,
        "real_robot_commands": False,
    }
    zero_fields = (
        "sim_time_advance_s",
        "clock_update_count",
        "physics_state_update_count",
        "controller_update_count",
        "policy_update_count",
        "command_update_count",
        "cpg_phase_advance_rad",
    )
    if (
        wall_elapsed < duration * 0.90
        or before != after
        or any(abs(float(result[name])) > 1e-12 for name in zero_fields)
    ):
        result["status"] = "FAILED"
    return result


class X2ColdStartCombinationCoordinator:
    """Advance three captures sequentially in one authoritative engine loop."""

    def __init__(
        self,
        plans: tuple[X2CombinationCasePlan, ...],
        capture_factory: Callable[[X2CombinationCasePlan], _Capture],
        *,
        freeze_wall_s: float = 2.0,
        freeze_after_walk_s: float = 3.0,
        sleep: Callable[[float], None] = time.sleep,
        perf_counter: Callable[[], float] = time.perf_counter,
    ) -> None:
        if tuple(plan.scenario for plan in plans) != SCENARIOS:
            raise ValueError(f"combination order must be {SCENARIOS}")
        for plan in plans:
            plan.validate()
        if freeze_wall_s < 2.0 or freeze_after_walk_s <= 0.0:
            raise ValueError("invalid freeze probe timing")
        self.plans = plans
        self.capture_factory = capture_factory
        self.freeze_wall_s = float(freeze_wall_s)
        self.freeze_after_walk_s = float(freeze_after_walk_s)
        self.sleep = sleep
        self.perf_counter = perf_counter
        self.case_index = 0
        self.current = capture_factory(plans[0])
        self.completed_cases: list[dict[str, object]] = []
        self.status = "RUNNING"
        self.completed = False
        self.stop_requested = False

    def _freeze_due(self, sim_time: float) -> bool:
        capture = self.current
        if capture.freeze_probe is not None or capture.first_sim_time is None:
            return False
        elapsed = float(sim_time) - float(capture.first_sim_time)
        return elapsed >= capture.profile.walk_start_s + self.freeze_after_walk_s

    def _request_stop_once(self) -> None:
        if not self.stop_requested:
            self.current.request_authoritative_stop()
            self.stop_requested = True

    def step(self, sim_time: float) -> None:
        if self.completed:
            return
        if self._freeze_due(sim_time):
            self.current.freeze_probe = perform_sim_time_freeze_probe(
                self.current,
                sim_time=sim_time,
                requested_wall_duration_s=self.freeze_wall_s,
                sleep=self.sleep,
                perf_counter=self.perf_counter,
            )
        self.current.step(sim_time)
        if not self.current.completed:
            return

        passed = self.current.status == "PASSED"
        if self.current.freeze_probe is not None:
            self.current.freeze_probe["resume_completed"] = passed
        self.completed_cases.append(
            {
                "scenario": self.plans[self.case_index].scenario,
                "status": self.current.status,
                "receipt": str(self.plans[self.case_index].receipt),
                "trace": str(self.plans[self.case_index].trace),
                "freeze_probe": self.current.freeze_probe,
            }
        )
        if not passed:
            self.status = "FAILED"
            self.completed = True
            self._request_stop_once()
            return

        if self.case_index + 1 == len(self.plans):
            self.status = "PASSED"
            self.completed = True
            self._request_stop_once()
            return

        self.case_index += 1
        self.current = self.capture_factory(self.plans[self.case_index])
        # The completed child intentionally emitted no command for this
        # already-advanced PhysX state.  Consume the same state immediately in
        # the next child so an intermediate boundary never creates a stale
        # zero-order-held command cycle.
        self.step(sim_time)

    def freeze_receipt(self) -> dict[str, object]:
        probes = [item.get("freeze_probe") for item in self.completed_cases]
        complete = len(probes) == len(self.plans) and all(
            isinstance(probe, dict) for probe in probes
        )
        if not complete:
            return {
                "schema": FREEZE_SCHEMA,
                "status": "INCOMPLETE",
                "qualification": False,
                "real_robot_commands": False,
                "cases": self.completed_cases,
            }
        typed = [probe for probe in probes if isinstance(probe, dict)]
        zero_fields = (
            "sim_time_advance_s",
            "clock_update_count",
            "physics_state_update_count",
            "controller_update_count",
            "policy_update_count",
            "command_update_count",
            "cpg_phase_advance_rad",
        )
        passed = all(
            probe.get("status") == "PASSED"
            and probe.get("resume_completed") is True
            for probe in typed
        )
        aggregate: dict[str, object] = {
            "schema": FREEZE_SCHEMA,
            "status": "PASSED" if passed else "FAILED",
            "observed_wall_duration_s": min(
                float(probe["observed_wall_duration_s"]) for probe in typed
            ),
            "resume_completed": passed,
            "cases": self.completed_cases,
            "qualification": False,
            "real_robot_commands": False,
        }
        for name in zero_fields:
            values = [float(probe[name]) for probe in typed]
            aggregate[name] = max(values, key=abs)
        return aggregate

    def result(self) -> dict[str, object]:
        return {
            "schema": COMBINATION_SCHEMA,
            "status": self.status,
            "qualification": False,
            "physics_authority": "genie_sim_isaac_physx",
            "single_physics_lifecycle": True,
            "case_order": list(SCENARIOS),
            "completed_case_count": len(self.completed_cases),
            "cases": self.completed_cases,
            "sim_time_freeze": self.freeze_receipt(),
            "controlled_stop_requested": self.stop_requested,
            "published_command_topics": [],
            "real_robot_commands": False,
        }
