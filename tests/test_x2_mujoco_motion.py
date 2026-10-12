import math
import pathlib
import tempfile
import unittest

from sceneforge.x2_mujoco_motion import (
    X2MujocoMotionProfile,
    load_urdf_joint_limits,
    load_urdf_joint_velocity_limits,
    quaternion_rpy_wxyz,
)


ROOT = pathlib.Path(__file__).parents[1]


class X2MujocoMotionTests(unittest.TestCase):
    def test_profile_has_exact_support_release_walk_and_stop_phases(self):
        profile = X2MujocoMotionProfile()
        profile.validate()
        self.assertEqual(profile.phase(0.0), "SUPPORTED_STAND")
        self.assertEqual(profile.phase(3.0), "BAND_RAMP_DOWN")
        self.assertAlmostEqual(profile.band_scale(5.5), 0.5)
        self.assertEqual(profile.phase(8.0), "UNSUPPORTED_STAND")
        self.assertEqual(profile.phase(11.0), "GAIT1_FORWARD")
        self.assertEqual(profile.velocity_command(11.0), (0.3, 0.0, 0.0))
        self.assertEqual(profile.phase(19.0), "DECELERATE")
        self.assertEqual(profile.phase(21.0), "CPG_STAND_HANDOFF")
        self.assertEqual(profile.phase(22.2), "STOP_STAND")
        self.assertEqual(profile.band_scale(19.0), 0.0)
        self.assertEqual(profile.phase(30.2), "COMPLETE")
        self.assertAlmostEqual(profile.velocity_command(20.0)[0], 0.15)

    def test_quaternion_angles_are_wxyz(self):
        half = 0.5
        roll, pitch, yaw = quaternion_rpy_wxyz((math.cos(half), 0.0, 0.0, math.sin(half)))
        self.assertAlmostEqual(roll, 0.0)
        self.assertAlmostEqual(pitch, 0.0)
        self.assertAlmostEqual(yaw, 1.0)

    def test_loads_exact_urdf_joint_limits(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "robot.urdf"
            path.write_text(
                '<robot name="x"><joint name="j" type="revolute">'
                '<parent link="a"/><child link="b"/>'
                '<limit lower="-1.2" upper="2.3" effort="1" velocity="4.5"/>'
                '</joint></robot>', encoding="utf-8",
            )
            self.assertEqual(
                load_urdf_joint_limits(path, ("j",)), {"j": (-1.2, 2.3)}
            )
            self.assertEqual(
                load_urdf_joint_velocity_limits(path, ("j",)), {"j": 4.5}
            )

    def test_driver_has_single_mujoco_authority_and_no_transport(self):
        driver = (ROOT / "admin/spark/x2-mujoco-motion-baseline.py").read_text(encoding="utf-8")
        runner = (ROOT / "admin/spark/run-x2-mujoco-motion-baseline.sh").read_text(encoding="utf-8")
        self.assertIn("mj_step", driver)
        self.assertIn("data.ctrl", driver)
        self.assertIn('"GAIT1_FORWARD", "DECELERATE", "CPG_STAND_HANDOFF"', driver)
        self.assertIn("Warm its 10-frame history", driver)
        update = driver.index("def update_walk")
        self.assertLess(
            driver.index("while self.next_walk_update", update),
            driver.index("if not apply_command", update),
        )
        self.assertIn('"google_mujoco_3_3_5"', driver)
        self.assertIn('"published_command_topics": []', driver)
        self.assertIn('"real_robot_commands": False', driver)
        self.assertNotIn("rclpy", driver)
        self.assertIn("--network none", runner)
        self.assertIn("--disable-ramp", runner)
        self.assertIn("supported_stand=${X2_SUPPORTED_STAND_S:-3}", runner)
        self.assertIn("band_ramp=${X2_BAND_RAMP_S:-5}", runner)
        self.assertIn("unsupported_stand=${X2_UNSUPPORTED_STAND_S:-3}", runner)
        self.assertIn("--supported-stand-s '$supported_stand'", runner)

    def test_filtered_command_startup_keeps_stand_hold(self):
        driver = (ROOT / "admin/spark/x2-mujoco-motion-baseline.py").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            'result.status in {"SAFE_HOLD", "STAND_HANDOFF_REQUEST"}', driver
        )
        self.assertIn("emits the policy candidate", driver)
        self.assertIn("self.walk_session.reset()", driver)
        self.assertIn("locomotion transition", driver)
        self.assertIn("joint = result.policy_joint_candidate", driver)


if __name__ == "__main__":
    unittest.main()
