import inspect
import math
import unittest

import sceneforge.x2_floating_stand as module


class X2FloatingStandTests(unittest.TestCase):
    def test_projected_gravity_matches_vendor_upright_offset(self):
        gravity = module._project_gravity_with_vendor_pitch_offset((1.0, 0.0, 0.0, 0.0))
        self.assertAlmostEqual(gravity[0], -math.sin(0.03), places=12)
        self.assertEqual(gravity[1], 0.0)
        self.assertAlmostEqual(gravity[2], -math.cos(0.03), places=12)

    def test_safety_and_authority_boundaries_are_explicit(self):
        source = inspect.getsource(module)
        self.assertIn('"root_pinned": False', source)
        self.assertIn('"gravity_modified": False', source)
        self.assertIn('"real_robot_commands": False', source)
        self.assertIn('pose[2] < 0.40 or tilt > 0.80', source)
        self.assertIn('unsupported_hold_s: float = 5.0', source)
        self.assertIn('if not self.band_status.is_file()', source)
        self.assertNotIn('if self.band_status.exists():', source)
        self.assertIn('os.chmod(path, 0o644)', source)
        self.assertIn('self._observed_unsupported_hold_s() < self.unsupported_hold_s', source)
        self.assertIn('"physx_contact_telemetry_available"', source)
        self.assertIn('"non_foot_ground_contact_absent"', source)
        self.assertIn('"support_load_plausible"', source)
        self.assertIn('"support_slip_bounded"', source)
        self.assertIn('"foot_contact_wrenches_finite"', source)
        self.assertIn('"proceed_to_mujoco_motion_baseline"', source)
        for forbidden in ("rclpy", "create_publisher", '"/joint_command"', '"/cmd_vel"'):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
