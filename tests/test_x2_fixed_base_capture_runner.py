import pathlib
import unittest


ROOT = pathlib.Path(__file__).parents[1]
RUNNER = ROOT / "admin/spark/run-x2-geniesim-fixed-base-capture-gate.sh"


class FixedBaseCaptureRunnerTests(unittest.TestCase):
    def test_runner_is_fixed_physx_and_fail_closed(self):
        source = RUNNER.read_text(encoding="utf-8")
        self.assertIn("/usd-fixed-base/manifest.json", source)
        self.assertIn("physics_engine:=isaac_physx", source)
        self.assertIn("REAL_ROBOT_COMMANDS=false", source)
        self.assertIn('docker network create --internal "$network"', source)
        self.assertIn("another physics authority appears to be running", source)
        self.assertIn('chmod -R a-w "$evidence_dir"', source)
        self.assertNotIn("mujoco-warp", source)

    def test_runner_records_controlled_stop_and_command_scope(self):
        source = RUNNER.read_text(encoding="utf-8")
        self.assertIn("CONTROLLED_PHYSICS_STOP_COMPLETE", source)
        self.assertIn("single_joint_hard_allowlist", source)
        self.assertIn("one_apply_commands_callsite", source)
        self.assertIn('"published_command_topics": []', source)
        self.assertIn('"mujoco_state_source_used": False', source)


if __name__ == "__main__":
    unittest.main()
