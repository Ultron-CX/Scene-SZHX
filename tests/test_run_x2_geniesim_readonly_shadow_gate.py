import pathlib
import unittest


SCRIPT = pathlib.Path(__file__).parents[1] / "admin" / "spark" / "run-x2-geniesim-readonly-shadow-gate.sh"


class ReadOnlyShadowGateScriptTests(unittest.TestCase):
    def test_gate_is_fixed_base_read_only_and_fail_closed(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("usd-fixed-base/manifest.json", source)
        self.assertIn("--fixed-base", source)
        self.assertIn("x2-geniesim-readonly-shadow.py", source)
        self.assertIn("ROBOEASE_GENIE_PUBLISH_RENDER_BODY_TF=false", source)
        self.assertNotIn("x1_rl_control_node", source)
        self.assertNotIn("/joint_command'", source)
        self.assertNotIn('ros2 topic pub', source)
        self.assertIn('"real_robot_commands": False', source)
        self.assertIn("CONTROLLED_PHYSICS_STOP_COMPLETE", source)


if __name__ == "__main__":
    unittest.main()
