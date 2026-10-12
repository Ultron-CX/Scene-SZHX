from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class X2PhysxDynamicsAlignmentTests(unittest.TestCase):
    def test_alignment_is_runtime_only_and_single_scope(self):
        source = (ROOT / "admin/spark/x2_physx_dynamics_alignment.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"ground_friction_only"', source)
        self.assertIn('"pre_world_reset": True', source)
        self.assertIn('"ground_friction_and_vendor_foot_capsules"', source)
        self.assertIn('"joint_or_controller_parameters_changed": False', source)
        self.assertIn('CreateStaticFrictionAttr().Set(static_friction)', source)
        self.assertIn('CreateDynamicFrictionAttr().Set(dynamic_friction)', source)
        self.assertIn('CreateFrictionCombineModeAttr().Set("max")', source)
        self.assertIn('UsdGeom.Capsule.Define(stage, path)', source)
        self.assertIn('child.SetInstanceable(False)', source)
        self.assertIn('len(authored_collision_paths) != 14', source)
        self.assertIn('"real_robot_commands": False', source)
        for forbidden in ("rclpy", "create_publisher", "/cmd_vel", "/joint_command"):
            self.assertNotIn(forbidden, source)

    def test_runner_records_alignment_receipt(self):
        source = (ROOT / "admin/spark/run-x2-geniesim-gait1-ab.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("X2_PHYSX_DYNAMICS_RECEIPT", source)
        self.assertIn('"dynamics_alignment": alignment', source)
        self.assertIn("X2_PHYSX_FOOT_COLLISION_MODE", source)

    def test_stage_applies_alignment_before_world_reset(self):
        source = (
            ROOT / "third_party/genie_sim_engine_patch/scripts/kit/stage.py"
        ).read_text(encoding="utf-8")
        alignment = source.index("configure_physx_dynamics_pre_reset(")
        reset = source.index("self._world.reset()", alignment)
        self.assertLess(alignment, reset)
        self.assertIn("PHYSX_DYNAMICS_ALIGNMENT_PRE_RESET", source[alignment:reset])


if __name__ == "__main__":
    unittest.main()
