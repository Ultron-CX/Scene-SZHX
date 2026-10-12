"""Wall-clock differential probe for the independent X2 control session.

The probe replays one immutable sequence of :class:`X2PhysicsState` values into
two fresh ``StateDrivenX2ControllerSession`` instances.  The delayed branch is
paused in wall time immediately before one selected state while simulation
timestamps remain identical.  Exact equality of the next control output and
of every subsequent complete session-state digest demonstrates that the
independent adapter is a function of the replayed simulation state rather than
of elapsed wall time.

This module does not start a simulator, publish commands, or implement the old
vendor controller binary.  Its conclusion is intentionally scoped to the
independent state-driven Python adapter and the supplied policy factory.
"""

from __future__ import annotations

import math
import time
from dataclasses import asdict
from typing import Any, Callable, Mapping, Sequence

from .canonical import fingerprint
from .x2_control import X2_CPGWALK_JOINT_NAMES, x2_cpgwalk_attitude_observation
from .x2_mujoco_motion import quaternion_rpy_wxyz
from .x2_session import (
    StateDrivenX2ControllerSession,
    X2PhysicsState,
    X2SessionResult,
)


STATE_SEQUENCE_SCHEMA = "sceneforge/x2-physics-state-sequence/v1"
PROBE_SCHEMA = "sceneforge/x2-sim-time-differential-probe/v1"

PolicyFactory = Callable[[], Callable[[tuple[float, ...]], Sequence[float]]]
SleepFunction = Callable[[float], None]
ClockFunction = Callable[[], float]


def extract_state_sequence_from_physx_trace(
    trace: Mapping[str, Any],
    *,
    phase: str = "GAIT1_FORWARD",
    start_offset_s: float = 1.0,
    duration_s: float = 3.0,
    sample_period_s: float = 0.02,
) -> dict[str, Any]:
    """Convert authoritative PhysX records into an immutable 50 Hz replay.

    The conversion deliberately uses only recorded full-joint/base state and
    the effective command that actually entered the independent controller.
    It neither advances physics nor reconstructs state from a video.
    """

    if trace.get("schema") != "sceneforge/x2-physx-locomotion-samples/v2":
        raise ValueError("unsupported PhysX locomotion trace schema")
    if trace.get("physics_authority") != "genie_sim_isaac_physx":
        raise ValueError("state sequence requires PhysX-authoritative input")
    if trace.get("real_robot_commands") is not False:
        raise ValueError("PhysX trace must declare real_robot_commands=false")
    phase = str(phase)
    if not phase:
        raise ValueError("phase must be non-empty")
    start_offset_s = float(start_offset_s)
    duration_s = float(duration_s)
    sample_period_s = float(sample_period_s)
    if not all(
        math.isfinite(value) and value > 0.0
        for value in (duration_s, sample_period_s)
    ) or not math.isfinite(start_offset_s) or start_offset_s < 0.0:
        raise ValueError("state-sequence timing must be finite and positive")

    joint_names = tuple(str(name) for name in trace.get("joint_names", ()))
    if len(set(joint_names)) != len(joint_names):
        raise ValueError("PhysX trace joint_names must be unique")
    try:
        indices = tuple(joint_names.index(name) for name in X2_CPGWALK_JOINT_NAMES)
    except ValueError as error:
        raise ValueError("PhysX trace lacks a CPGWalk joint") from error
    physics = trace.get("physics_samples")
    bases = trace.get("base_samples")
    if not isinstance(physics, list) or not isinstance(bases, list):
        raise ValueError("PhysX trace state arrays are missing")
    if len(physics) != len(bases) or len(physics) < 2:
        raise ValueError("PhysX trace state arrays differ or are empty")

    candidates: list[tuple[Mapping[str, Any], Mapping[str, Any]]] = []
    previous_sequence: int | None = None
    previous_time: float | None = None
    for state, base in zip(physics, bases):
        if not isinstance(state, Mapping) or not isinstance(base, Mapping):
            raise ValueError("PhysX trace state row must be an object")
        sequence = int(state["physics_sequence"])
        sim_time = float(state["sim_time"])
        if sequence != int(base["physics_sequence"]) or not math.isclose(
            sim_time, float(base["sim_time"]), abs_tol=1e-12
        ):
            raise ValueError("PhysX joint/base state is not sequence aligned")
        if previous_sequence is not None and (
            sequence <= previous_sequence or sim_time <= float(previous_time)
        ):
            raise ValueError("PhysX trace sequence/time is not strictly increasing")
        previous_sequence, previous_time = sequence, sim_time
        if state.get("phase") == phase and base.get("phase") == phase:
            candidates.append((state, base))
    if len(candidates) < 2:
        raise ValueError(f"PhysX trace does not contain phase {phase}")

    phase_start = float(candidates[0][0]["sim_time"])
    first_target = phase_start + start_offset_s
    last_target = first_target + duration_s
    next_target = first_target
    selected: list[tuple[Mapping[str, Any], Mapping[str, Any]]] = []
    for state, base in candidates:
        sim_time = float(state["sim_time"])
        if sim_time + 1e-12 < next_target:
            continue
        if sim_time > last_target + sample_period_s * 0.51:
            break
        if abs(sim_time - next_target) > sample_period_s * 0.51:
            raise ValueError("PhysX trace cannot supply the requested replay rate")
        selected.append((state, base))
        next_target += sample_period_s
        if next_target > last_target + 1e-12:
            break
    expected_count = int(math.floor(duration_s / sample_period_s + 1e-9)) + 1
    if len(selected) != expected_count:
        raise ValueError(
            f"PhysX trace yielded {len(selected)} states, expected {expected_count}"
        )

    states = []
    for state, base in selected:
        q = tuple(float(value) for value in state["position"])
        dq = tuple(float(value) for value in state["velocity"])
        if len(q) != len(joint_names) or len(dq) != len(joint_names):
            raise ValueError("PhysX trace joint state width differs from joint_names")
        roll, pitch, _ = quaternion_rpy_wxyz(base["orientation_wxyz"])
        sim_time = float(state["sim_time"])
        replay = X2PhysicsState(
            source_sequence=int(state["physics_sequence"]),
            sim_time=sim_time,
            command_stamp_sim_time=sim_time,
            velocity_command=tuple(
                float(value) for value in base["effective_velocity_command"]
            ),
            joint_position=tuple(q[index] for index in indices),
            joint_velocity=tuple(dq[index] for index in indices),
            base_angular_velocity=tuple(
                float(value) for value in base["base_angular_velocity_body_rps"]
            ),
            observation_gravity=x2_cpgwalk_attitude_observation(roll, pitch),
            base_linear_velocity=tuple(
                float(value) for value in base["base_linear_velocity_body_mps"]
            ),
            roll=roll,
            pitch=pitch,
        )
        replay.validate()
        states.append(physics_state_payload(replay))
    return {
        "schema": STATE_SEQUENCE_SCHEMA,
        "source": {
            "physics_authority": "genie_sim_isaac_physx",
            "trace_schema": trace["schema"],
            "phase": phase,
            "start_offset_s": start_offset_s,
            "duration_s": duration_s,
            "sample_period_s": sample_period_s,
        },
        "states": states,
        "real_robot_commands": False,
    }


def physics_state_from_mapping(value: Mapping[str, Any]) -> X2PhysicsState:
    """Build and validate one replay state from a JSON-compatible mapping."""

    state = X2PhysicsState(
        source_sequence=int(value["source_sequence"]),
        sim_time=float(value["sim_time"]),
        command_stamp_sim_time=float(value["command_stamp_sim_time"]),
        velocity_command=tuple(float(item) for item in value["velocity_command"]),
        joint_position=tuple(float(item) for item in value["joint_position"]),
        joint_velocity=tuple(float(item) for item in value["joint_velocity"]),
        base_angular_velocity=tuple(
            float(item) for item in value["base_angular_velocity"]
        ),
        observation_gravity=tuple(
            float(item) for item in value["observation_gravity"]
        ),
        base_linear_velocity=tuple(
            float(item) for item in value["base_linear_velocity"]
        ),
        roll=float(value["roll"]),
        pitch=float(value["pitch"]),
    )
    state.validate()
    return state


def load_state_sequence_payload(payload: Mapping[str, Any]) -> tuple[X2PhysicsState, ...]:
    """Validate the explicit offline replay contract.

    An explicit ``real_robot_commands=false`` is required rather than inferred.
    This prevents a generic or physical-robot capture from being used without a
    deliberate conversion into the offline-only evidence contract.
    """

    if payload.get("schema") != STATE_SEQUENCE_SCHEMA:
        raise ValueError(f"state sequence schema must be {STATE_SEQUENCE_SCHEMA!r}")
    if payload.get("real_robot_commands") is not False:
        raise ValueError("state sequence must declare real_robot_commands=false")
    raw_states = payload.get("states")
    if not isinstance(raw_states, list):
        raise ValueError("state sequence states must be a list")
    states = tuple(physics_state_from_mapping(value) for value in raw_states)
    _validate_state_sequence(states)
    return states


def physics_state_payload(state: X2PhysicsState) -> dict[str, Any]:
    """Return the canonical JSON-compatible representation used for hashing."""

    state.validate()
    return asdict(state)


def _validate_state_sequence(states: Sequence[X2PhysicsState]) -> None:
    if len(states) < 2:
        raise ValueError("differential probe requires at least two replay states")
    previous_sequence: int | None = None
    previous_time: float | None = None
    for state in states:
        state.validate()
        if previous_sequence is not None and state.source_sequence <= previous_sequence:
            raise ValueError("replay source_sequence must be strictly increasing")
        if previous_time is not None and state.sim_time <= previous_time:
            raise ValueError("replay sim_time must be strictly increasing")
        previous_sequence = state.source_sequence
        previous_time = state.sim_time


def _control_output_payload(result: X2SessionResult) -> dict[str, Any]:
    """Serialize the complete deterministic session output for exact comparison."""

    payload = asdict(result)
    if payload.get("real_robot_commands") is not False:
        raise RuntimeError("X2 session output violated real_robot_commands=false")
    # ``fingerprint`` below serializes with allow_nan=False.  Calling it here
    # also fails closed before an invalid output can enter a receipt.
    fingerprint(payload)
    return payload


def _applied_command_payload(result: X2SessionResult) -> dict[str, Any]:
    """Return the policy candidate and the optional active joint command."""

    return {
        "status": result.status,
        "source_sequence": result.source_sequence,
        "sim_time": result.sim_time,
        "gait": result.gait,
        "command_was_stale": result.command_was_stale,
        "policy_joint_candidate": asdict(result.policy_joint_candidate),
        "joint_command": (
            asdict(result.joint_command) if result.joint_command is not None else None
        ),
        "real_robot_commands": False,
    }


def run_sim_time_differential_probe(
    states: Sequence[X2PhysicsState],
    policy_factory: PolicyFactory,
    *,
    pause_before_index: int,
    wall_sleep_seconds: float = 2.0,
    sleep_fn: SleepFunction = time.sleep,
    clock_fn: ClockFunction = time.perf_counter,
    policy_identity: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Replay states through fresh baseline and wall-delayed X2 sessions.

    ``pause_before_index`` identifies the first state processed after the wall
    pause in the delayed branch.  Both sessions are advanced in lockstep before
    that point.  At the selected state the baseline branch runs first, wall
    time is paused, and then the delayed branch receives the exact same state.
    The injected clock and sleep functions make the probe fast and deterministic
    in unit tests without weakening the production default of a real wall sleep.
    """

    states = tuple(states)
    _validate_state_sequence(states)
    if not isinstance(pause_before_index, int):
        raise TypeError("pause_before_index must be an integer")
    if not 1 <= pause_before_index < len(states):
        raise ValueError("pause_before_index must select a state after a non-empty prefix")
    wall_sleep_seconds = float(wall_sleep_seconds)
    if not math.isfinite(wall_sleep_seconds) or wall_sleep_seconds <= 0.0:
        raise ValueError("wall_sleep_seconds must be finite and positive")

    baseline_policy = policy_factory()
    delayed_policy = policy_factory()
    baseline = StateDrivenX2ControllerSession(baseline_policy)
    delayed = StateDrivenX2ControllerSession(delayed_policy)
    distinct_instances = baseline is not delayed and baseline_policy is not delayed_policy

    input_payloads = [physics_state_payload(state) for state in states]
    baseline_output_hashes: list[str] = []
    delayed_output_hashes: list[str] = []
    baseline_command_hashes: list[str] = []
    delayed_command_hashes: list[str] = []
    baseline_state_digests: list[str] = []
    delayed_state_digests: list[str] = []
    outputs_are_safe = True
    digest_before_pause_baseline: str | None = None
    digest_before_pause_delayed: str | None = None
    observed_wall_pause_s = 0.0

    for index, state in enumerate(states):
        if index == pause_before_index:
            digest_before_pause_baseline = baseline.state_digest()
            digest_before_pause_delayed = delayed.state_digest()
            baseline_result = baseline.update(state)
            wall_started = float(clock_fn())
            sleep_fn(wall_sleep_seconds)
            observed_wall_pause_s = float(clock_fn()) - wall_started
            delayed_result = delayed.update(state)
        else:
            baseline_result = baseline.update(state)
            delayed_result = delayed.update(state)

        baseline_output = _control_output_payload(baseline_result)
        delayed_output = _control_output_payload(delayed_result)
        baseline_command = _applied_command_payload(baseline_result)
        delayed_command = _applied_command_payload(delayed_result)
        outputs_are_safe = outputs_are_safe and (
            baseline_result.real_robot_commands is False
            and delayed_result.real_robot_commands is False
        )
        baseline_output_hashes.append(fingerprint(baseline_output))
        delayed_output_hashes.append(fingerprint(delayed_output))
        baseline_command_hashes.append(fingerprint(baseline_command))
        delayed_command_hashes.append(fingerprint(delayed_command))
        baseline_state_digests.append(baseline.state_digest())
        delayed_state_digests.append(delayed.state_digest())

    next_index = pause_before_index
    checks = {
        "distinct_session_and_policy_instances": distinct_instances,
        "pre_pause_complete_state_digest_equal": (
            digest_before_pause_baseline == digest_before_pause_delayed
        ),
        "requested_wall_pause_observed": (
            math.isfinite(observed_wall_pause_s)
            and observed_wall_pause_s >= 0.90 * wall_sleep_seconds
        ),
        "next_command_exactly_equal": (
            baseline_command_hashes[next_index] == delayed_command_hashes[next_index]
        ),
        "next_control_output_exactly_equal": (
            baseline_output_hashes[next_index] == delayed_output_hashes[next_index]
        ),
        "next_complete_state_digest_exactly_equal": (
            baseline_state_digests[next_index] == delayed_state_digests[next_index]
        ),
        "full_command_stream_exactly_equal": (
            baseline_command_hashes == delayed_command_hashes
        ),
        "full_control_output_stream_exactly_equal": (
            baseline_output_hashes == delayed_output_hashes
        ),
        "full_complete_state_digest_stream_exactly_equal": (
            baseline_state_digests == delayed_state_digests
        ),
        "all_session_outputs_real_robot_commands_false": outputs_are_safe,
        "receipt_real_robot_commands_false": True,
    }
    passed = all(checks.values())
    return {
        "schema": PROBE_SCHEMA,
        "status": "PASSED" if passed else "FAILED",
        "decision": (
            "independent_adapter_sim_time_differential_passed"
            if passed
            else "independent_adapter_wall_clock_dependency_or_nondeterminism_detected"
        ),
        "qualification": False,
        "scope": "independent_state_driven_x2_python_adapter",
        "controller": "StateDrivenX2ControllerSession",
        "policy_identity": dict(policy_identity or {}),
        "input": {
            "schema": STATE_SEQUENCE_SCHEMA,
            "state_count": len(states),
            "state_sequence_sha256": fingerprint(input_payloads),
            "first_source_sequence": states[0].source_sequence,
            "last_source_sequence": states[-1].source_sequence,
            "first_sim_time_s": states[0].sim_time,
            "last_sim_time_s": states[-1].sim_time,
            "real_robot_commands": False,
        },
        "pause": {
            "before_state_index": pause_before_index,
            "before_source_sequence": states[pause_before_index].source_sequence,
            "requested_wall_pause_s": wall_sleep_seconds,
            "observed_wall_pause_s": observed_wall_pause_s,
        },
        "checks": checks,
        "evidence": {
            "pre_pause_complete_state_digest": {
                "baseline": digest_before_pause_baseline,
                "wall_delayed": digest_before_pause_delayed,
            },
            "next_command_sha256": {
                "baseline": baseline_command_hashes[next_index],
                "wall_delayed": delayed_command_hashes[next_index],
            },
            "next_control_output_sha256": {
                "baseline": baseline_output_hashes[next_index],
                "wall_delayed": delayed_output_hashes[next_index],
            },
            "next_complete_state_digest": {
                "baseline": baseline_state_digests[next_index],
                "wall_delayed": delayed_state_digests[next_index],
            },
            "full_command_stream_sha256": {
                "baseline": fingerprint(baseline_command_hashes),
                "wall_delayed": fingerprint(delayed_command_hashes),
            },
            "full_control_output_stream_sha256": {
                "baseline": fingerprint(baseline_output_hashes),
                "wall_delayed": fingerprint(delayed_output_hashes),
            },
            "full_complete_state_digest_stream_sha256": {
                "baseline": fingerprint(baseline_state_digests),
                "wall_delayed": fingerprint(delayed_state_digests),
            },
        },
        "limitations": [
            "This probe covers only the independent StateDrivenX2ControllerSession adapter and the supplied policy factory.",
            "It does not prove that the legacy vendor ARM64 controller binary is simulation-time driven.",
            "It replays recorded observations without advancing MuJoCo, PhysX, Genie Sim, ROS 2, or a physical robot.",
            "A pass establishes wall-delay invariance for the replayed sequence; it is not locomotion or physics qualification.",
        ],
        "serialization": {"format": "json", "allow_nan": False},
        "published_command_topics": [],
        "real_robot_commands": False,
    }
