from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class X2PhysxGait1RunnerTests(unittest.TestCase):
    def test_runner_is_locked_to_physx_and_workspace_assets(self):
        source = (ROOT / "admin/spark/run-x2-geniesim-gait1-ab.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("physics_engine:=isaac_physx", source)
        self.assertIn("x2-floating-dev-v1/manifest.json", source)
        self.assertIn("REAL_ROBOT_COMMANDS=false", source)
        self.assertIn("ROBOEASE_GENIE_PUBLISH_RENDER_BODY_TF=false", source)
        self.assertIn("X2_COMMAND_X_MPS", source)
        self.assertIn("chmod -R a-w", source)
        self.assertIn('"qualification": False', source)
        self.assertNotIn("docker pull", source)
        self.assertNotIn("ssh ", source)

    def test_driver_uses_inprocess_capture_without_command_topics(self):
        source = (ROOT / "admin/spark/x2-geniesim-gait1-driver.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("X2PhysxGait1Capture", source)
        self.assertIn("PhysXBaseStateReader", source)
        self.assertIn("state_reader=base_state.read", source)
        self.assertIn("PhysXFootContactReader", source)
        contact_call = source[source.index("contacts = PhysXFootContactReader"):source.index("profile = X2MujocoMotionProfile")]
        capture_call = source[source.index("capture = X2PhysxGait1Capture"):source.index("session.post_step_hooks")]
        self.assertNotIn("freeze_probe_wall_s", contact_call)
        self.assertIn("freeze_probe_wall_s", capture_call)
        self.assertIn("physics_engine != \"isaac_physx\"", source)
        for forbidden in ("rclpy", "create_publisher", "/cmd_vel", "/joint_command"):
            self.assertNotIn(forbidden, source)

    def test_engine_publishes_authoritative_post_step_sim_time(self):
        source = (
            ROOT / "third_party/genie_sim_engine_patch/scripts/common/loop.py"
        ).read_text(encoding="utf-8")
        step = source.index("solver_ms = sim.step(dt, step_start)")
        advance = source.index("sim_time += dt", step)
        hook = source.index("for hook in post_step_hooks", advance)
        self.assertLess(step, advance)
        self.assertLess(advance, hook)


if __name__ == "__main__":
    unittest.main()
