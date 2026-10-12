import subprocess
import sys
import unittest
from pathlib import Path

from sceneforge.x2_control import CpgWalkSafetyParams
from sceneforge.x2_session import (
    StateDrivenX2ControllerSession,
    X2PhysicsState,
    X2SessionParams,
)


class RecordingPolicy:
    def __init__(self):
        self.inputs = []

    def __call__(self, values):
        self.inputs.append(tuple(values))
        return (0.0,) * 17


def state(
    sequence,
    sim_time,
    *,
    command=(0.3, 0.0, 0.0),
    command_stamp=None,
    joint_position=None,
):
    if command_stamp is None:
        command_stamp = sim_time
    return X2PhysicsState(
        source_sequence=sequence,
        sim_time=sim_time,
        command_stamp_sim_time=command_stamp,
        velocity_command=command,
        joint_position=joint_position or (-0.248, 0.0, 0.0, 0.5303, -0.2823, 0.0) * 2
        + (0.0,) * 5,
        joint_velocity=(0.0,) * 17,
        base_angular_velocity=(0.0, 0.0, 0.0),
        observation_gravity=(0.0, 0.0, -1.0),
        base_linear_velocity=(0.0, 0.0, 0.0),
        roll=0.0,
        pitch=0.0,
    )


class X2SessionTests(unittest.TestCase):
    def test_control_session_import_is_dependency_light(self):
        repository = Path(__file__).resolve().parents[1]
        script = (
            "import sys; import sceneforge.x2_session; "
            "assert 'sceneforge.models' not in sys.modules; "
            "assert 'pydantic' not in sys.modules"
        )
        completed = subprocess.run(
            [sys.executable, "-c", script],
            cwd=repository,
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_session_source_has_no_transport_or_wall_clock(self):
        source = (
            Path(__file__).resolve().parents[1] / "src" / "sceneforge" / "x2_session.py"
        ).read_text(encoding="utf-8").lower()
        for forbidden in ("rclpy", "publisher", "socket", "steady_clock", "time.monotonic"):
            self.assertNotIn(forbidden, source)

    def test_first_state_seeds_history_and_emits_active_command(self):
        policy = RecordingPolicy()
        session = StateDrivenX2ControllerSession(policy)
        result = session.update(state(10, 1.0))
        self.assertEqual(result.status, "ACTIVE_COMMAND")
        self.assertEqual(len(result.observation), 65)
        self.assertEqual(len(result.stacked_observation), 650)
        self.assertEqual(result.stacked_observation[:65], result.observation)
        self.assertEqual(result.stacked_observation[-65:], result.observation)
        self.assertIsNotNone(result.joint_command)
        self.assertIs(result.policy_joint_candidate, result.joint_command)
        self.assertFalse(result.real_robot_commands)
        self.assertEqual(len(policy.inputs), 1)

    def test_duplicate_or_frozen_state_does_not_run_policy(self):
        policy = RecordingPolicy()
        session = StateDrivenX2ControllerSession(policy)
        session.update(state(1, 1.0))
        with self.assertRaisesRegex(ValueError, "duplicate"):
            session.update(state(1, 1.02))
        with self.assertRaisesRegex(ValueError, "did not advance"):
            session.update(state(2, 1.0))
        self.assertEqual(len(policy.inputs), 1)

    def test_controller_digest_advances_only_with_sim_state(self):
        session = StateDrivenX2ControllerSession(RecordingPolicy())
        initial = session.state_digest()
        self.assertEqual(initial, session.state_digest())
        session.update(state(1, 1.0))
        advanced = session.state_digest()
        self.assertNotEqual(initial, advanced)
        self.assertEqual(advanced, session.state_digest())

    def test_stale_command_fails_to_safe_hold(self):
        policy = RecordingPolicy()
        session = StateDrivenX2ControllerSession(policy)
        result = session.update(state(1, 1.0, command_stamp=0.0))
        self.assertEqual(result.status, "SAFE_HOLD")
        self.assertTrue(result.command_was_stale)
        self.assertIsNone(result.joint_command)
        self.assertIsNotNone(result.policy_joint_candidate)

    def test_history_shifts_once_per_new_state_and_reset_clears_it(self):
        policy = RecordingPolicy()
        session = StateDrivenX2ControllerSession(policy)
        first = session.update(state(1, 1.0))
        changed_position = list(state(2, 1.02).joint_position)
        changed_position[0] += 0.1
        second = session.update(state(2, 1.02, joint_position=tuple(changed_position)))
        self.assertEqual(second.stacked_observation[:65], first.observation)
        self.assertEqual(second.stacked_observation[-65:], second.observation)
        session.reset()
        restarted = session.update(state(1, 5.0))
        self.assertEqual(restarted.stacked_observation[:65], restarted.observation)

    def test_large_state_gap_fails_closed_before_policy(self):
        policy = RecordingPolicy()
        session = StateDrivenX2ControllerSession(
            policy,
            session_params=X2SessionParams(maximum_state_gap_seconds=0.05),
        )
        session.update(state(1, 1.0))
        with self.assertRaisesRegex(ValueError, "gap"):
            session.update(state(2, 1.06))
        self.assertEqual(len(policy.inputs), 1)

    def test_zero_command_requests_stand_handoff_once(self):
        policy = RecordingPolicy()
        session = StateDrivenX2ControllerSession(
            policy,
            safety_params=CpgWalkSafetyParams(auto_stand_hold_seconds=0.06),
        )
        session.update(state(1, 1.0))
        results = [
            session.update(state(index + 2, 1.0 + 0.02 * (index + 1), command=(0.0, 0.0, 0.0)))
            for index in range(4)
        ]
        self.assertEqual(
            [result.status for result in results],
            ["SAFE_HOLD", "SAFE_HOLD", "STAND_HANDOFF_REQUEST", "SAFE_HOLD"],
        )


if __name__ == "__main__":
    unittest.main()
