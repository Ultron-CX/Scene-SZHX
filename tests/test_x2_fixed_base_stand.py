import inspect
import math
import unittest

import sceneforge.x2_fixed_base_stand as module


class FakeInput:
    name = "input"


class FakeOutput:
    name = "output"


class X2FixedBaseStandTests(unittest.TestCase):
    def test_hard_limits_and_transport_boundary(self):
        source = inspect.getsource(module)
        self.assertIn('POLICY_RATE_HZ = 100.0', source)
        self.assertIn('MAXIMUM_TOTAL_DURATION_S = 6.0', source)
        self.assertIn('cmd_positions={}', source)
        self.assertIn('cmd_efforts=efforts', source)
        self.assertIn('len(names) != 31', source)
        self.assertIn('active_s: float = 0.5', source)
        self.assertIn('restore_s: float = 5.0', source)
        self.assertIn('position-PD is evaluated', source)
        self.assertIn('"pd_rate_hz": 500.0', source)
        self.assertIn('"real_robot_commands": False', source)
        for forbidden in ("rclpy", "create_publisher", '"/joint_command"', '"/cmd_vel"'):
            self.assertNotIn(forbidden, source)

    def test_observation_defaults_cover_23_joints(self):
        self.assertEqual(len(module._OBSERVATION_DEFAULT), 23)
        self.assertEqual(module._OBSERVATION_DEFAULT[:12], module.X2_STAND_NOMINAL[:12])
        self.assertTrue(all(math.isfinite(value) for value in module._OBSERVATION_DEFAULT))

    def test_effort_limits_cover_frozen_articulation(self):
        names = tuple(module.X2_STAND_JOINT_NAMES) + module._AUXILIARY_JOINTS
        self.assertEqual(len(set(names)), 31)
        self.assertTrue(all(module._effort_limit(name) > 0.0 for name in names))
        self.assertEqual(module._effort_limit("left_shoulder_pitch_joint"), 36.0)
        self.assertEqual(module._effort_limit("left_shoulder_roll_joint"), 36.0)
        self.assertEqual(module._effort_limit("left_shoulder_yaw_joint"), 24.0)


if __name__ == "__main__":
    unittest.main()
