import importlib.util
import math
import pathlib
import sys
import unittest


MODULE_PATH = pathlib.Path(__file__).parents[1] / "admin" / "spark" / "x2-geniesim-controller-adapter.py"
SPEC = importlib.util.spec_from_file_location("x2_adapter", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class AdapterContractTest(unittest.TestCase):
    def test_joint_contract_is_unique_and_complete(self):
        self.assertEqual(31, len(MODULE.ALL_JOINTS))
        self.assertEqual(31, len(set(MODULE.ALL_JOINTS)))
        self.assertEqual({"leg", "waist", "arm", "head"}, set(MODULE.GROUPS))

    def test_validate_vector_accepts_exact_lane(self):
        MODULE.validate_vector(MODULE.LEG, [0.0] * len(MODULE.LEG), MODULE.LEG)

    def test_validate_vector_rejects_order_change(self):
        with self.assertRaisesRegex(ValueError, "names/order"):
            MODULE.validate_vector(tuple(reversed(MODULE.LEG)), [0.0] * len(MODULE.LEG), MODULE.LEG)

    def test_validate_vector_rejects_nonfinite(self):
        values = [0.0] * len(MODULE.HEAD)
        values[-1] = math.nan
        with self.assertRaisesRegex(ValueError, "NaN"):
            MODULE.validate_vector(MODULE.HEAD, values, MODULE.HEAD)

    def test_quaternion_yaw(self):
        half = math.pi / 4.0
        yaw = MODULE.quaternion_yaw(0.0, 0.0, math.sin(half), math.cos(half))
        self.assertAlmostEqual(math.pi / 2.0, yaw)

    def test_quaternion_yaw_rejects_zero_norm(self):
        with self.assertRaisesRegex(ValueError, "norm is zero"):
            MODULE.quaternion_yaw(0.0, 0.0, 0.0, 0.0)

    def test_specific_force_rest_is_gravity_in_body_frame(self):
        estimator = MODULE.SpecificForceEstimator()
        first = estimator.update(1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        second = estimator.update(1.01, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        self.assertEqual((0.0, 0.0, 9.81), first)
        self.assertAlmostEqual(9.81, second[2])

    def test_specific_force_uses_physx_velocity_difference(self):
        estimator = MODULE.SpecificForceEstimator(cutoff_hz=1000.0)
        estimator.update(2.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        value = estimator.update(2.01, (0.01, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        self.assertAlmostEqual(1.0, value[0], places=4)
        self.assertAlmostEqual(9.81, value[2], places=4)

    def test_specific_force_rejects_non_monotonic_stamp(self):
        estimator = MODULE.SpecificForceEstimator()
        estimator.update(3.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        previous = estimator.filtered_body
        self.assertEqual(
            previous,
            estimator.update(2.9, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        )
        self.assertEqual(1, estimator.rejected_dt_count)

    def test_source_has_no_real_robot_transport(self):
        text = MODULE_PATH.read_text(encoding="utf-8")
        self.assertNotIn("203.0.113", text)
        self.assertNotIn("ssh", text.lower())
        self.assertIn('"real_robot_commands": False', text)


if __name__ == "__main__":
    unittest.main()
