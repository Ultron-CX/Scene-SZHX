import pathlib
import unittest


ROOT = pathlib.Path(__file__).parents[1]
MODULE = ROOT / "src" / "sceneforge" / "x2_inprocess_shadow.py"
DRIVER = ROOT / "admin" / "spark" / "x2-geniesim-inprocess-shadow-driver.py"


class InProcessShadowSourceTests(unittest.TestCase):
    def test_hook_has_no_command_or_transport_path(self):
        source = MODULE.read_text(encoding="utf-8")
        self.assertIn("post-step hook", source)
        for forbidden in ("apply_commands", "create_publisher", '"/joint_command"', '"/cmd_vel"'):
            self.assertNotIn(forbidden, source)
        self.assertIn('"command_application_calls": 0', source)
        self.assertIn('"real_robot_commands": False', source)

    def test_driver_uses_engine_post_step_hook(self):
        source = DRIVER.read_text(encoding="utf-8")
        self.assertIn("session.post_step_hooks.append(shadow.step)", source)
        self.assertNotIn("rclpy", source)
        self.assertNotIn("ros2 topic pub", source)


if __name__ == "__main__":
    unittest.main()
