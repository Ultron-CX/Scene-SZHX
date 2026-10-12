import importlib.util
import math
import pathlib
import sys
import unittest


MODULE_PATH = pathlib.Path(__file__).parents[1] / "admin" / "spark" / "x2-geniesim-readonly-shadow.py"
SPEC = importlib.util.spec_from_file_location("x2_readonly_shadow", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class ReadOnlyShadowTests(unittest.TestCase):
    def test_command_parser(self):
        self.assertEqual((0.3, 0.0, -0.2), MODULE.parse_command("0.3,0,-0.2"))

    def test_quaternion_gravity_and_angles(self):
        half = math.pi / 12.0
        quaternion = (0.0, math.sin(half), 0.0, math.cos(half))
        gravity = MODULE.rotate_world_to_body(quaternion, (0.0, 0.0, -1.0))
        roll, pitch = MODULE.roll_pitch(quaternion)
        self.assertAlmostEqual(0.5, gravity[0], places=6)
        self.assertAlmostEqual(0.0, roll, places=6)
        self.assertAlmostEqual(math.pi / 6.0, pitch, places=6)

    def test_source_has_subscriptions_but_no_command_transport(self):
        source = MODULE_PATH.read_text(encoding="utf-8")
        self.assertIn("create_subscription", source)
        self.assertNotIn("create_publisher", source)
        self.assertNotIn('"/joint_command"', source)
        self.assertNotIn('"/cmd_vel"', source)
        self.assertIn('"published_topics": []', source)
        self.assertIn('"real_robot_commands": False', source)


if __name__ == "__main__":
    unittest.main()
