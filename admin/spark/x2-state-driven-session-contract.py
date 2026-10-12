#!/usr/bin/env python3
"""Exercise fail-closed sequencing of the engine-neutral X2 control session."""

from __future__ import annotations

import hashlib
import json
import os
import sys
import tempfile
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from sceneforge.x2_session import (  # noqa: E402
    StateDrivenX2ControllerSession,
    X2PhysicsState,
    X2SessionParams,
)


SCHEMA = "sceneforge/x2-state-driven-session-contract/v1"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        os.chmod(path, 0o444)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


class ZeroPolicy:
    def __init__(self) -> None:
        self.call_count = 0
        self.input_widths: set[int] = set()

    def __call__(self, observation: tuple[float, ...]) -> tuple[float, ...]:
        self.call_count += 1
        self.input_widths.add(len(observation))
        return (0.0,) * 17


def physics_state(
    sequence: int,
    sim_time: float,
    *,
    command: tuple[float, float, float] = (0.3, 0.0, 0.0),
    command_stamp: float | None = None,
) -> X2PhysicsState:
    if command_stamp is None:
        command_stamp = sim_time
    return X2PhysicsState(
        source_sequence=sequence,
        sim_time=sim_time,
        command_stamp_sim_time=command_stamp,
        velocity_command=command,
        joint_position=(
            -0.248, 0.0, 0.0, 0.5303, -0.2823, 0.0,
            -0.248, 0.0, 0.0, 0.5303, -0.2823, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0,
        ),
        joint_velocity=(0.0,) * 17,
        base_angular_velocity=(0.0, 0.0, 0.0),
        observation_gravity=(0.0, 0.0, -1.0),
        base_linear_velocity=(0.0, 0.0, 0.0),
        roll=0.0,
        pitch=0.0,
    )


def rejected(callable_, text: str) -> bool:
    try:
        callable_()
    except ValueError as exc:
        return text in str(exc)
    return False


def main() -> int:
    if len(sys.argv) != 3 or sys.argv[1] != "--receipt":
        raise SystemExit("usage: x2-state-driven-session-contract.py --receipt PATH")
    receipt = Path(sys.argv[2]).resolve()
    if receipt.exists():
        raise FileExistsError(f"refusing to overwrite receipt: {receipt}")

    policy = ZeroPolicy()
    session = StateDrivenX2ControllerSession(policy)
    first = session.update(physics_state(1, 1.0))
    calls_after_first = policy.call_count
    duplicate_rejected = rejected(
        lambda: session.update(physics_state(1, 1.02)), "duplicate"
    )
    frozen_time_rejected = rejected(
        lambda: session.update(physics_state(2, 1.0)), "did not advance"
    )
    rejected_states_did_not_run_policy = policy.call_count == calls_after_first

    session.reset()
    stale = session.update(physics_state(1, 2.0, command_stamp=0.0))
    stale_command_safe_hold = stale.status == "SAFE_HOLD" and stale.joint_command is None

    session.reset()
    session.update(physics_state(1, 3.0))
    gap_rejected = rejected(
        lambda: session.update(physics_state(2, 3.11)), "gap"
    )

    session.reset()
    restarted = session.update(physics_state(1, 4.0))
    reset_reseeded_history = (
        restarted.stacked_observation[:65] == restarted.observation
        and restarted.stacked_observation[-65:] == restarted.observation
    )

    session.reset()
    session.update(physics_state(1, 5.0))
    handoff_statuses = []
    for index in range(50):
        result = session.update(
            physics_state(
                index + 2,
                5.0 + 0.02 * (index + 1),
                command=(0.0, 0.0, 0.0),
            )
        )
        handoff_statuses.append(result.status)
    handoff_request_count = handoff_statuses.count("STAND_HANDOFF_REQUEST")
    handoff_request_index = (
        handoff_statuses.index("STAND_HANDOFF_REQUEST") if handoff_request_count else None
    )

    checks = {
        "first_state_active_command": first.status == "ACTIVE_COMMAND" and first.joint_command is not None,
        "history_width_650": len(first.stacked_observation) == 650,
        "policy_input_width_650": policy.input_widths == {650},
        "duplicate_sequence_rejected": duplicate_rejected,
        "frozen_sim_time_rejected": frozen_time_rejected,
        "rejected_states_did_not_run_policy": rejected_states_did_not_run_policy,
        "stale_command_safe_hold": stale_command_safe_hold,
        "large_state_gap_rejected": gap_rejected,
        "reset_reseeded_history": reset_reseeded_history,
        "one_shot_handoff_at_1s": handoff_request_count == 1 and handoff_request_index == 49,
    }
    passed = all(checks.values())
    source_files = [ROOT / "src" / "sceneforge" / name for name in ("x2_control.py", "x2_session.py")]
    payload = {
        "schema": SCHEMA,
        "status": "PASSED" if passed else "FAILED",
        "decision": (
            "state_sequence_sim_time_history_timeout_reset_contract_passed"
            if passed
            else "state_driven_session_contract_failed"
        ),
        "offline_only": True,
        "published_topics": [],
        "policy": "deterministic_zero_test_double",
        "checks": checks,
        "metrics": {
            "accepted_policy_call_count": policy.call_count,
            "policy_input_widths": sorted(policy.input_widths),
            "handoff_request_count": handoff_request_count,
            "handoff_request_zero_based_stop_tick": handoff_request_index,
        },
        "session_parameters": X2SessionParams().__dict__,
        "implementation": [
            {"path": str(path), "size": path.stat().st_size, "sha256": sha256(path)}
            for path in source_files
        ],
        "limitations": [
            "This contract uses a deterministic zero policy and synthetic finite state data.",
            "It proves sequencing and fail-closed behavior, not ONNX numerical parity or PhysX transport.",
            "No ROS/Zenoh publisher or physics engine is present in this run.",
        ],
        "qualification": False,
        "real_robot_commands": False,
        "finished_at_unix": time.time(),
    }
    atomic_json(receipt, payload)
    print(receipt)
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
