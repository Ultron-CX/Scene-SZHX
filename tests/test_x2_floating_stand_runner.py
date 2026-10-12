import pathlib
import unittest


ROOT = pathlib.Path(__file__).parents[1]
SCRIPT = ROOT / "admin/spark/run-x2-geniesim-floating-stand-gate.sh"
DRIVER = ROOT / "admin/spark/x2-geniesim-floating-stand-driver.py"
CONTACT_READER = ROOT / "admin/spark/x2_physx_contact_reader.py"
STAGE = ROOT / "third_party/genie_sim_engine_patch/scripts/kit/stage.py"


class X2FloatingStandRunnerTests(unittest.TestCase):
    def test_runner_is_floating_physx_and_offline_from_real_robot(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("/project/assets/x2-floating-dev-v1/manifest.json", source)
        self.assertIn("/project/assets/x2-floating-dev-v1/physics_params_x2.yaml", source)
        self.assertIn("REAL_ROBOT_COMMANDS=false", source)
        self.assertIn("physics_engine:=isaac_physx", source)
        self.assertIn("docker network create --internal", source)
        self.assertIn("ROBOEASE_PHYSX_ELASTIC_BAND_ENABLED=true", source)
        self.assertIn("ROBOEASE_PHYSX_CONTACT_REPORTS_ENABLED=true", source)
        self.assertIn('X2_STAND_UNSUPPORTED_HOLD_S', source)
        self.assertIn('"state": "ENGAGED"', source)
        self.assertIn('mujoco_state_source_used": False', source)
        self.assertIn('"qualification": False', source)

    def test_runner_requires_auditable_band_release(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("ELASTIC_BAND_INITIALIZED", source)
        self.assertIn('band.get("state") != "RELEASED"', source)
        self.assertIn('band.get("force_norm_n") != 0.0', source)
        self.assertIn("elastic-band-telemetry.jsonl", source)

    def test_runner_preserves_capture_next_gate_decision(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertIn(
            'capture.get("decision", "manual_review_required") if not failures',
            source,
        )
        self.assertNotIn(
            '"decision": "proceed_to_contact_instrumentation_and_60s_stand"',
            source,
        )

    def test_driver_uses_in_process_physx_state_and_no_command_topics(self):
        source = DRIVER.read_text(encoding="utf-8")
        self.assertIn("session.post_step_hooks.append(capture.step)", source)
        self.assertIn("snapshot_odom", source)
        self.assertIn("PhysXFootContactReader", source)
        self.assertIn("enable_physx_contact_reports", source)
        self.assertIn('physics_engine != "isaac_physx"', source)
        for forbidden in ("rclpy", "create_publisher", '"/joint_command"', '"/cmd_vel"'):
            self.assertNotIn(forbidden, source)

    def test_contact_reader_is_physx_only_and_read_only(self):
        source = CONTACT_READER.read_text(encoding="utf-8")
        self.assertIn('create_simulation_view("numpy", stage_id)', source)
        self.assertIn("PhysxContactReportAPI.Apply", source)
        self.assertIn("CreateThresholdAttr().Set(0.0)", source)
        self.assertIn("create_rigid_contact_view", source)
        self.assertIn("get_net_contact_forces", source)
        self.assertIn("get_contact_data", source)
        self.assertIn("get_friction_data", source)
        self.assertIn('"real_robot_commands": False', source)
        for forbidden in ("rclpy", "create_publisher", "apply_commands", "simulate("):
            self.assertNotIn(forbidden, source)

    def test_contact_reports_are_authored_before_world_reset(self):
        source = STAGE.read_text(encoding="utf-8")
        report = source.index("PHYSX_CONTACT_REPORTS_AUTHORED")
        reset = source.index("self._world.reset()", report)
        self.assertLess(report, reset)
        self.assertIn("PhysxContactReportAPI.Apply", source[report - 1800:reset])
        self.assertIn("CreateThresholdAttr().Set(0.0)", source[report - 1800:reset])


if __name__ == "__main__":
    unittest.main()
