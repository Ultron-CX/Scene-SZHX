import pathlib
import unittest


ROOT = pathlib.Path(__file__).parents[1]
SCRIPT = ROOT / "admin/spark/run-x2-geniesim-fixed-base-stand-gate.sh"
DRIVER = ROOT / "admin/spark/x2-geniesim-fixed-base-stand-driver.py"


class X2FixedBaseStandRunnerTests(unittest.TestCase):
    def test_runner_is_fixed_physx_and_offline_from_real_robot(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("usd-fixed-base/manifest.json", source)
        self.assertIn("REAL_ROBOT_COMMANDS=false", source)
        self.assertIn("physics_engine:=isaac_physx", source)
        self.assertIn("--network \"$network\"", source)
        self.assertIn("docker network create --internal", source)
        self.assertIn("runtime/x2-shadow-ort/onnxruntime/__init__.py", source)
        self.assertIn("$ort_python:/project/src:$engine_python", source)
        self.assertIn('mujoco_state_source_used": False', source)

    def test_driver_uses_post_step_and_physx_only(self):
        source = DRIVER.read_text(encoding="utf-8")
        self.assertIn("session.post_step_hooks.append(capture.step)", source)
        self.assertIn('physics_engine != "isaac_physx"', source)
        for forbidden in ("rclpy", "create_publisher", '"/joint_command"', '"/cmd_vel"'):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
