import pathlib
import unittest


SCRIPT = pathlib.Path(__file__).parents[1] / "admin" / "spark" / "run-x2-geniesim-inprocess-shadow-gate.sh"


class InProcessShadowGateTests(unittest.TestCase):
    def test_runner_has_one_read_only_physx_authority(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("x2-geniesim-inprocess-shadow-driver.py", source)
        self.assertIn("usd-fixed-base/manifest.json", source)
        self.assertIn("CONTROLLED_PHYSICS_STOP_COMPLETE", source)
        self.assertIn('"real_robot_commands": False', source)
        self.assertNotIn("x1_rl_control_node", source)
        self.assertNotIn("ros2 topic pub", source)
        self.assertNotIn("/joint_command'", source)


if __name__ == "__main__":
    unittest.main()
