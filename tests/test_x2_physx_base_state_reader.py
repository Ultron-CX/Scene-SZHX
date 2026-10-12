import importlib.util
import pathlib
import unittest

import numpy as np


MODULE_PATH = pathlib.Path(__file__).parents[1] / "admin" / "spark" / "x2_physx_base_state_reader.py"
SPEC = importlib.util.spec_from_file_location("x2_physx_base_state_reader", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class PhysXBaseStateMathTests(unittest.TestCase):
    def test_identity_preserves_world_twist(self):
        twist = MODULE.body_twist_from_world((1, 0, 0, 0), (1, 2, 3), (4, 5, 6))
        np.testing.assert_allclose(twist, (1, 2, 3, 4, 5, 6), atol=1e-12)

    def test_world_velocity_rotates_into_body_coordinates(self):
        half = 2.0 ** -0.5
        twist = MODULE.body_twist_from_world((half, 0, 0, half), (0, 1, 0), (0, 0, 2))
        np.testing.assert_allclose(twist, (1, 0, 0, 0, 0, 2), atol=1e-12)

    def test_non_unit_quaternion_is_normalized(self):
        rotation = MODULE.rotation_world_from_body_wxyz((2, 0, 0, 0))
        np.testing.assert_allclose(rotation, np.eye(3), atol=1e-12)

    def test_tensor_transform_is_converted_from_xyzw_to_wxyz(self):
        pose = MODULE.pose_wxyz_from_tensor_transform((1, 2, 3, 0.1, 0.2, 0.3, 0.9))
        np.testing.assert_allclose(pose, (1, 2, 3, 0.9, 0.1, 0.2, 0.3), atol=1e-12)

    def test_invalid_velocity_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "NaN or Inf"):
            MODULE.body_twist_from_world((1, 0, 0, 0), (float("nan"), 0, 0), (0, 0, 0))

    def test_reader_source_is_explicit_and_read_only(self):
        source = MODULE_PATH.read_text(encoding="utf-8")
        self.assertIn('source = "physx_rigid_prim_world_velocity_to_base"', source)
        self.assertNotIn("apply_forces", source)
        self.assertNotIn("set_world_pose", source)
        self.assertNotIn("set_velocities", source)


if __name__ == "__main__":
    unittest.main()
