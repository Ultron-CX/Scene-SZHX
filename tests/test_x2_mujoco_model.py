import pathlib
import unittest

from sceneforge.x2_mujoco_model import (
    X2_ALL_JOINT_NAMES,
    X2MujocoModelContract,
)


ROOT = pathlib.Path(__file__).parents[1]


class X2MujocoModelTests(unittest.TestCase):
    def contract(self):
        names = ("floating_base_joint", *X2_ALL_JOINT_NAMES)
        ranges = tuple((-120.0, 120.0) for _ in X2_ALL_JOINT_NAMES)
        return X2MujocoModelContract(
            mujoco_version="3.3.5", timestep_s=0.001, nq=38, nv=37, nu=31,
            nbody=34, njnt=32, ngeom=84, nsensor=56,
            robot_body_mass_kg=42.0, floating_joint_name="floating_base_joint",
            joint_names=names,
            actuator_names=tuple(f"motor_{name}" for name in X2_ALL_JOINT_NAMES),
            actuator_joint_names=X2_ALL_JOINT_NAMES,
            actuator_control_ranges=ranges,
            sensor_names=(
                "body-orientation", "body-angular-velocity", "body-linear-pos",
                "body-linear-vel", "body-linear-acceleration",
            ),
            left_foot_geom_names=tuple(f"left_foot{i}_collision" for i in range(1, 8)),
            right_foot_geom_names=tuple(f"right_foot{i}_collision" for i in range(1, 8)),
        )

    def test_contract_requires_exact_engine_and_dimensions(self):
        checks = self.contract().checks()
        self.assertTrue(checks["mujoco_version_is_3_3_5"])
        self.assertTrue(checks["floating_dimensions_match"])
        self.assertTrue(checks["all_31_joints_actuated"])
        self.assertTrue(checks["seven_left_foot_geoms"])

    def test_wrong_version_fails_closed(self):
        base = self.contract()
        changed = X2MujocoModelContract(**{**base.__dict__, "mujoco_version": "3.2.0"})
        self.assertFalse(changed.checks()["mujoco_version_is_3_3_5"])
        self.assertFalse(changed.as_dict()["passed"])

    def test_runner_is_network_isolated_and_read_only(self):
        runner = (ROOT / "admin/spark/run-x2-mujoco-model-audit.sh").read_text(encoding="utf-8")
        probe = (ROOT / "admin/spark/x2-mujoco-model-audit.py").read_text(encoding="utf-8")
        self.assertIn("--network none", runner)
        self.assertIn("--read-only", runner)
        self.assertIn("--user 0:0", runner)
        self.assertIn("chmod -R a-w /evidence", runner)
        self.assertIn("REAL_ROBOT_COMMANDS=false", runner)
        self.assertIn('"physics_advanced": False', probe)
        self.assertIn('"command_application_count": 0', probe)
        self.assertNotIn("MjData(", probe)
        self.assertNotIn("mj_step", probe)


if __name__ == "__main__":
    unittest.main()
