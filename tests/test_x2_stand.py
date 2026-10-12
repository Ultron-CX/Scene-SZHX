import unittest

from sceneforge.x2_stand import (
    StateDrivenX2StandSession,
    X2StandPhysicsState,
    X2_STAND_ACTION_JOINT_NAMES,
    X2_STAND_JOINT_NAMES,
    X2_STAND_POLICY_DT,
    build_stand_observation,
    stand_action_to_command,
)


def state(sequence=1, sim_time=0.0):
    return X2StandPhysicsState(
        source_sequence=sequence,
        sim_time=sim_time,
        joint_position=(0.0,) * 23,
        joint_velocity=(0.0,) * 23,
        base_angular_velocity=(0.0, 0.0, 0.0),
        observation_gravity=(0.0, 0.0, -1.0),
        velocity_command=(0.0, 0.0, 0.0),
        pose_command=(0.0, 0.0, 0.0, 0.0),
    )


class X2StandTests(unittest.TestCase):
    def test_observation_layout_and_history(self):
        obs = build_stand_observation(state(), (0.0,) * 14)
        self.assertEqual(len(obs), 85)
        self.assertEqual(obs[3:6], (0.0, 0.0, -1.0))
        self.assertEqual(obs[59:73], (0.0,) * 14)
        calls = []
        session = StateDrivenX2StandSession(lambda values: calls.append(values) or (0.0,) * 14)
        first = session.update(state())
        self.assertEqual(len(first.stacked_observation), 850)
        self.assertEqual(first.stacked_observation[:85], first.observation)
        with self.assertRaises(ValueError):
            session.update(state())
        self.assertEqual(len(calls), 1)

    def test_action_mapping_matches_vendor_stand_contract(self):
        action = (
            0.193698, -0.102318, -0.068934, -0.566864, 0.439282, -0.113826,
            0.078680, 0.117079, -0.056696, -0.658028, 0.235577, 0.101454,
            0.649186, -0.051578,
        )
        command = stand_action_to_command(action, state())
        self.assertEqual(command.joint_names, X2_STAND_JOINT_NAMES)
        lookup = dict(zip(command.joint_names, command.position_target))
        alpha = X2_STAND_POLICY_DT / (
            X2_STAND_POLICY_DT + 1.0 / (2.0 * 3.141592653589793 * 5000.0)
        )
        self.assertAlmostEqual(
            lookup["left_hip_pitch_joint"],
            -0.235 + alpha * 0.5 * action[0],
            places=12,
        )
        self.assertAlmostEqual(lookup["waist_pitch_joint"], 0.314, places=6)
        self.assertAlmostEqual(
            lookup["waist_roll_joint"],
            alpha * 0.5 * action[13],
            places=12,
        )
        self.assertEqual(lookup["left_shoulder_pitch_joint"], 0.4)
        self.assertEqual(lookup["left_elbow_joint"], -1.2)
        self.assertTrue(all(value == 0.0 for value in command.velocity_target))
        self.assertTrue(all(value == 0.0 for value in command.feedforward_effort))
        self.assertFalse(command.real_robot_commands)

    def test_frozen_waist_observation_does_not_reuse_policy_waist_outputs(self):
        frozen = X2StandPhysicsState(
            **{
                **state().__dict__,
                "waist_action_observation": (0.001, -0.002),
            }
        )
        obs = build_stand_observation(frozen, tuple(range(14)))
        self.assertEqual(obs[59:71], tuple(float(value) for value in range(12)))
        self.assertEqual(obs[71:73], (0.001, -0.002))

    def test_action_residual_filter_is_stateful(self):
        action = (0.2,) * 14
        first = stand_action_to_command(action, state())
        second = stand_action_to_command(action, state(), first.filtered_residual)
        self.assertLess(first.filtered_residual[0], second.filtered_residual[0])
        self.assertLess(second.filtered_residual[0], 0.1)

    def test_contract_widths_and_transport_independence(self):
        self.assertEqual(len(X2_STAND_JOINT_NAMES), 23)
        self.assertEqual(len(X2_STAND_ACTION_JOINT_NAMES), 14)
        import inspect
        import sceneforge.x2_stand as module

        source = inspect.getsource(module)
        for forbidden in ("rclpy", "create_publisher", "apply_commands", "time.monotonic"):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
