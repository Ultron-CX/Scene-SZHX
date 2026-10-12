import inspect
import unittest

import sceneforge.x2_fixed_base_stand as stand_module
import sceneforge.x2_physx_gait1 as module


class X2PhysxGait1Tests(unittest.TestCase):
    def test_development_capture_has_explicit_authority_and_safety_boundaries(self):
        source = inspect.getsource(module)
        self.assertIn('"physics_authority": "genie_sim_isaac_physx"', source)
        self.assertIn('"qualification": False', source)
        self.assertIn('"real_robot_commands": False', source)
        self.assertIn('pose[2] < 0.40 or tilt > 0.80', source)
        self.assertIn('x2_cpgwalk_attitude_observation(roll, pitch)', source)
        self.assertIn('X2VelocityCommandFilter()', source)
        self.assertIn('command = result.policy_joint_candidate', source)
        self.assertIn("X2PositionTargetSafetyEnvelope", source)
        self.assertIn("limited.applied_position_target", source)
        self.assertIn('"bounded_policy_position_target_saturation"', source)
        self.assertIn('band.get("support_ratio") == 0.0', source)
        for forbidden in (
            "rclpy",
            "create_publisher",
            '"/joint_command"',
            '"/cmd_vel"',
            "mujoco.MjModel",
            "mujoco.mj_step",
        ):
            self.assertNotIn(forbidden, source)

    def test_effort_application_accepts_all_profile_phases(self):
        source = inspect.getsource(stand_module.X2FixedBaseStandCapture._apply)
        for phase in (
            "SUPPORTED_STAND",
            "BAND_RAMP_DOWN",
            "UNSUPPORTED_STAND",
            "GAIT1_FORWARD",
            "DECELERATE",
            "CPG_STAND_HANDOFF",
            "STOP_STAND",
        ):
            self.assertIn(f'"{phase}"', source)

    def test_desired_command_belongs_to_walk_not_stand_update(self):
        walk = inspect.signature(module.X2PhysxGait1Capture._walk_update)
        stand = inspect.signature(module.X2PhysxGait1Capture._stand_update)
        self.assertIn("desired_command", walk.parameters)
        self.assertNotIn("desired_command", stand.parameters)

    def test_finalization_is_fail_closed_and_single_shot(self):
        source = inspect.getsource(module.X2PhysxGait1Capture._finish)
        self.assertIn("self.completed = True", source)
        self.assertLess(
            source.index("self.completed = True"),
            source.index("self._write_trace()"),
        )
        self.assertIn("debug_physx_gait1_finalization", source)
        self.assertIn("finally:", source)
        self.assertIn("self._request_stop()", source)


if __name__ == "__main__":
    unittest.main()
