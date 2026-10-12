import math
import unittest

from sceneforge.x2_control import (
    AdaptiveHopfParams,
    AdaptiveHopfSnapshot,
    CpgWalkActionPipeline,
    CpgWalkActionSnapshot,
    CpgWalkGaitSelector,
    CpgWalkSafetyGate,
    CpgWalkSafetyMode,
    CpgWalkSafetyParams,
    HopfMode,
    TwoLegAdaptiveHopf,
    X2CrossTrackHoldController,
    X2HeadingHoldController,
    X2PositionTargetSafetyEnvelope,
    X2VelocityCommandCalibration,
    X2VelocityCommandFilter,
    apply_post_filter_lateral_stabilization,
    apply_post_filter_yaw_stabilization,
    apply_minimum_forward_command_shape,
    x2_cpgwalk_attitude_observation,
    first_order_low_pass_alpha,
)


PARAMS = AdaptiveHopfParams(
    x2_style_adapter=True,
    gait_blend_tau=0.08,
    reset_on_walk_entry=False,
)


class X2ControlTests(unittest.TestCase):
    def test_position_target_safety_envelope_clamps_and_audits_raw_excursion(self):
        envelope = X2PositionTargetSafetyEnvelope({
            "joint_a": (-0.5, 0.5),
            "joint_b": (-1.0, 1.0),
        })
        output = envelope.apply(
            ("joint_a", "joint_b"),
            (0.508, -0.25),
            sim_time=12.5,
            phase="GAIT1_FORWARD",
            policy_sequence=42,
        )
        self.assertEqual(output.applied_position_target, (0.5, -0.25))
        self.assertEqual(output.saturation_count, 1)
        self.assertAlmostEqual(output.maximum_raw_limit_excess_rad, 0.008)
        receipt = envelope.snapshot()
        self.assertEqual(receipt["raw_target_sample_count"], 2)
        self.assertEqual(receipt["saturation_count"], 1)
        self.assertFalse(receipt["bounded"])
        self.assertEqual(receipt["events"][0]["joint_name"], "joint_a")

    def test_position_target_safety_envelope_accepts_sparse_bounded_excursion(self):
        envelope = X2PositionTargetSafetyEnvelope({"joint": (-0.5, 0.5)})
        for sequence in range(101):
            target = 0.508 if sequence == 100 else 0.0
            envelope.apply(
                ("joint",), (target,), sim_time=sequence * 0.02,
                phase="GAIT1_FORWARD", policy_sequence=sequence,
            )
        receipt = envelope.snapshot()
        self.assertTrue(receipt["bounded"])
        self.assertLess(receipt["saturation_ratio"], 0.01)

    def test_position_target_safety_envelope_rejects_bad_contract(self):
        with self.assertRaisesRegex(ValueError, "missing"):
            envelope = X2PositionTargetSafetyEnvelope({"joint": (-0.5, 0.5)})
            envelope.apply(
                ("other",), (0.0,), sim_time=0.0,
                phase="STAND", policy_sequence=0,
            )

    def test_velocity_calibration_is_explicit_and_preserves_nonforward_defaults(self):
        calibration = X2VelocityCommandCalibration()
        calibrated = calibration.apply((0.3, 0.1, -0.2))
        self.assertAlmostEqual(calibrated[0], 0.186)
        self.assertEqual(calibrated[1], 0.1)
        self.assertAlmostEqual(calibrated[2], -0.364)
        self.assertAlmostEqual(calibration.apply((0.3, 0.0, 0.2))[2], 0.48)
        with self.assertRaisesRegex(ValueError, "positive"):
            X2VelocityCommandCalibration(forward_scale=0.0)

    def test_heading_hold_corrects_negative_yaw_drift(self):
        controller = X2HeadingHoldController()
        controller.reset(0.0)
        self.assertGreater(controller.update(-0.1, -0.02), 0.0)

    def test_heading_hold_clamps_and_requires_reset(self):
        controller = X2HeadingHoldController()
        with self.assertRaisesRegex(RuntimeError, "reset"):
            controller.update(0.0, 0.0)
        controller.reset(math.pi - 0.05)
        self.assertAlmostEqual(controller.update(-math.pi + 0.05, 0.0), -0.14)
        controller.reset(0.0)
        self.assertEqual(controller.update(-1.0, -10.0), 0.25)

    def test_cross_track_hold_corrects_rightward_drift(self):
        controller = X2CrossTrackHoldController()
        controller.reset((0.0, 0.0), 0.0)
        cross_track, command = controller.update((1.0, -0.1), -0.02)
        self.assertAlmostEqual(cross_track, -0.1)
        self.assertGreater(command, 0.0)

    def test_cross_track_hold_uses_walk_entry_heading_and_clamps(self):
        controller = X2CrossTrackHoldController()
        controller.reset((1.0, 2.0), math.pi / 2.0)
        cross_track, command = controller.update((1.2, 4.0), 0.0)
        self.assertAlmostEqual(cross_track, -0.2)
        self.assertAlmostEqual(command, 0.066)

    def test_cross_track_correction_is_applied_after_vendor_lateral_deadband(self):
        command_filter = X2VelocityCommandFilter()
        filtered = command_filter.update((0.3, 0.08, 0.0), 0.02)
        self.assertEqual(filtered.effective[1], 0.0)
        corrected = apply_post_filter_lateral_stabilization(
            filtered.effective,
            0.04,
            maximum_lateral_speed_mps=0.08,
        )
        self.assertEqual(corrected, (filtered.effective[0], 0.04, 0.0))

    def test_heading_correction_is_applied_after_vendor_yaw_deadband(self):
        command_filter = X2VelocityCommandFilter()
        filtered = command_filter.update((0.3, 0.0, 0.08), 0.02)
        self.assertEqual(filtered.effective[2], 0.0)
        corrected = apply_post_filter_yaw_stabilization(
            filtered.effective,
            -0.04,
            maximum_yaw_rate_rps=0.25,
        )
        self.assertEqual(corrected, (filtered.effective[0], 0.0, -0.04))

    def test_control_snapshots_cover_all_mutable_outer_loop_state(self):
        command_filter = X2VelocityCommandFilter()
        before = command_filter.snapshot()
        command_filter.update((0.3, 0.0, 0.0), 0.02)
        self.assertNotEqual(before, command_filter.snapshot())

        heading = X2HeadingHoldController()
        heading.reset(0.25)
        self.assertEqual(heading.snapshot()["reference_yaw"], 0.25)

        cross_track = X2CrossTrackHoldController()
        cross_track.reset((1.0, 2.0), -0.25)
        self.assertEqual(cross_track.snapshot()["reference_position_xy"], [1.0, 2.0])

    def test_cpgwalk_attitude_is_roll_pitch_zero_yaw_not_gravity(self):
        self.assertEqual(
            x2_cpgwalk_attitude_observation(0.02, 0.04),
            (0.02, 0.025, 0.0),
        )

    def test_vendor_forward_command_filter_matches_successful_arm64_trace(self):
        command_filter = X2VelocityCommandFilter()
        command_filter.update((0.0, 0.0, 0.0), 0.02)
        expected = (
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.100074,
            0.102691,
            0.108550,
            0.117015,
            0.127520,
            0.139560,
            0.152686,
            0.166501,
            0.180656,
            0.194844,
            0.208797,
            0.222282,
            0.235100,
        )
        actual = [
            command_filter.update((0.6, 0.0, 0.0), 0.02).effective[0]
            for _ in expected
        ]
        for observed, wanted in zip(actual, expected):
            self.assertAlmostEqual(observed, wanted, delta=5.1e-7)

    def test_minimum_forward_shape_is_odd_and_bounded(self):
        self.assertEqual(apply_minimum_forward_command_shape(0.1, 0.1, 0.3), 0.0)
        self.assertAlmostEqual(
            apply_minimum_forward_command_shape(0.2, 0.1, 0.3), 0.2
        )
        self.assertAlmostEqual(
            apply_minimum_forward_command_shape(-0.2, 0.1, 0.3), -0.2
        )
        self.assertEqual(apply_minimum_forward_command_shape(0.3, 0.1, 0.3), 0.3)

    def test_stand_output_and_first_walk_step(self):
        hopf = TwoLegAdaptiveHopf(PARAMS)
        self.assertEqual(hopf.obs4(), (0.0, 1.0, 0.0, 1.0))
        hopf.set_gait(1)
        hopf.set_mode(HopfMode.WALK)
        hopf.step(0.02)
        self.assertAlmostEqual(hopf.period, 0.654)
        left = hopf.obs4()[:2]
        right = hopf.obs4()[2:]
        self.assertAlmostEqual(left[0], -right[0], places=12)
        self.assertAlmostEqual(left[1], -right[1], places=12)

    def test_vendor_gait2_transition_matches_quantized_trace(self):
        hopf = TwoLegAdaptiveHopf(PARAMS)
        hopf.restore(
            AdaptiveHopfSnapshot(
                mode=HopfMode.WALK,
                gait=2,
                walk_initialized=True,
                period=0.601594,
                obs4=(0.865958, -0.520586, -0.865958, 0.520586),
                filtered_velocity=(0.517673, 0.0, 0.081223),
                filtered_command=(0.0, 0.0),
            )
        )
        hopf.step(0.02)
        expected = (0.739132, -0.688949, -0.739132, 0.688949)
        self.assertAlmostEqual(hopf.period, 0.6012752, places=7)
        for actual, wanted in zip(hopf.obs4(), expected):
            self.assertAlmostEqual(actual, wanted, delta=1.5e-5)

    def test_vendor_gait_switch_matches_quantized_trace(self):
        hopf = TwoLegAdaptiveHopf(PARAMS)
        hopf.restore(
            AdaptiveHopfSnapshot(
                mode=HopfMode.WALK,
                gait=1,
                walk_initialized=True,
                period=0.600000,
                obs4=(-0.954092, 0.326390, 0.954028, -0.326368),
                filtered_velocity=(0.435650, 0.0, 0.056409),
                filtered_command=(0.0, 0.0),
            )
        )
        hopf.step(0.02)
        expected = (-0.874123, 0.502536, 0.874064, -0.502502)
        self.assertAlmostEqual(hopf.period, 0.654)
        for actual, wanted in zip(hopf.obs4(), expected):
            self.assertAlmostEqual(actual, wanted, delta=1.5e-5)

    def test_filters_and_asymmetric_leg_mu(self):
        hopf = TwoLegAdaptiveHopf(PARAMS)
        hopf.update_base_velocity(0.02, 1.0, 0.5, 0.25)
        self.assertGreater(hopf.locomotion_intensity(), 0.0)
        hopf.update_command_inputs(0.02, 0.5, 0.5)
        left, right = hopf.leg_mu()
        self.assertGreater(left, right)

    def test_snapshot_preserves_hidden_stand_state(self):
        hopf = TwoLegAdaptiveHopf(PARAMS)
        hopf.x_left = 0.25
        hopf.y_right = -0.75
        snapshot = hopf.snapshot()
        self.assertEqual(snapshot.obs4, (0.25, 1.0, 0.0, -0.75))
        self.assertEqual(hopf.obs4(), (0.0, 1.0, 0.0, 1.0))

    def test_restore_rejects_non_finite_state(self):
        hopf = TwoLegAdaptiveHopf(PARAMS)
        snapshot = AdaptiveHopfSnapshot(
            mode=HopfMode.WALK,
            gait=1,
            walk_initialized=True,
            period=0.87,
            obs4=(0.0, math.nan, 0.0, -1.0),
            filtered_velocity=(0.0, 0.0, 0.0),
            filtered_command=(0.0, 0.0),
        )
        with self.assertRaisesRegex(ValueError, "NaN or Inf"):
            hopf.restore(snapshot)

    def test_deployed_action_filter_coefficients(self):
        self.assertAlmostEqual(first_order_low_pass_alpha(0.02, 5000.0), 0.9984109795736363)
        self.assertAlmostEqual(first_order_low_pass_alpha(0.02, 10.0), 0.5568627241441778)
        self.assertAlmostEqual(first_order_low_pass_alpha(0.02, 50.0), 0.8626974383015871)

    def test_vendor_active_action_transition_matches_quantized_trace(self):
        pipeline = CpgWalkActionPipeline()
        pipeline.restore(
            CpgWalkActionSnapshot(
                residual=(
                    -0.151985, -0.133964, -0.017189, 0.572889, 0.117137,
                    0.050708, 0.119542, -0.372366, 0.138456, -0.304295,
                    -0.075664, -0.080524, 0.127782, -0.069901, -0.038507,
                    0.159156, 0.117489,
                ),
                filtered_position_target=(
                    -0.399749, -0.133947, -0.017136, 1.102544, -0.165336,
                    0.050711, -0.128466, -0.371974, 0.138351, 0.226118,
                    -0.357924, -0.080511, 0.131631, -0.064943, -0.029875,
                    0.156483, 0.115728,
                ),
            )
        )
        output = pipeline.step_active(
            (
                -0.616135, -0.185942, -0.084391, 1.809991, 0.878525,
                2.492665, 0.197168, -1.065403, 0.270684, -0.589665,
                -0.530253, -4.405241, 0.643645, -0.613232, -0.338238,
                0.942338, 0.473060,
            ),
            0.02,
        )
        expected_residual = (
            -0.307819, -0.093036, -0.042156, 0.904468, 0.175612,
            0.049855, 0.098617, -0.532447, 0.135347, -0.294848,
            -0.106002, -0.088093, 0.128309, -0.065124, -0.043433,
            0.184443, 0.097753,
        )
        expected_filtered = (
            -0.555571, -0.093101, -0.042116, 1.434240, -0.106781,
            0.049856, -0.149349, -0.532192, 0.135352, 0.235438,
            -0.388254, -0.088081, 0.129781, -0.065044, -0.037425,
            0.180604, 0.100221,
        )
        for actual, wanted in zip(output.residual, expected_residual):
            self.assertAlmostEqual(actual, wanted, delta=1.1e-6)
        for actual, wanted in zip(output.position_target, expected_filtered):
            self.assertAlmostEqual(actual, wanted, delta=1.1e-6)
        self.assertEqual(output.velocity_target, (0.0,) * 17)
        self.assertEqual(output.feedforward_effort, (0.0,) * 17)

    def test_action_pipeline_fails_closed_on_bad_width(self):
        pipeline = CpgWalkActionPipeline()
        with self.assertRaisesRegex(ValueError, "width"):
            pipeline.step_active((0.0,) * 16, 0.02)

    def test_safety_gate_uses_explicit_dt_and_one_shot_handoff(self):
        gate = CpgWalkSafetyGate(CpgWalkSafetyParams(auto_stand_hold_seconds=0.06))
        active = gate.update(
            dt=0.02,
            command_xyz=(0.3, 0.0, 0.0),
            roll=0.0,
            pitch=0.0,
            mean_leg_velocity=1.0,
            base_angular_velocity_norm=1.0,
            gait=1,
        )
        self.assertEqual(active.mode, CpgWalkSafetyMode.ACTIVE)
        decisions = [
            gate.update(
                dt=0.02,
                command_xyz=(0.0, 0.0, 0.0),
                roll=0.0,
                pitch=0.0,
                mean_leg_velocity=1.0,
                base_angular_velocity_norm=1.0,
                gait=1,
            )
            for _ in range(4)
        ]
        self.assertEqual(
            [decision.request_stand_transition for decision in decisions],
            [False, False, True, False],
        )

    def test_low_motion_requirement_blocks_command_freeze(self):
        gate = CpgWalkSafetyGate(
            CpgWalkSafetyParams(require_low_motion_for_freeze=True)
        )
        decision = gate.update(
            dt=0.02,
            command_xyz=(0.0, 0.0, 0.0),
            roll=0.0,
            pitch=0.0,
            mean_leg_velocity=1.0,
            base_angular_velocity_norm=0.1,
            gait=1,
        )
        self.assertFalse(decision.freeze)
        self.assertEqual(decision.mode, CpgWalkSafetyMode.ACTIVE)

    def test_gait_selector_has_speed_hysteresis_and_yaw_block(self):
        selector = CpgWalkGaitSelector()
        self.assertEqual(selector.update(0.5, 0.0), 1)
        self.assertEqual(selector.update(0.500001, 0.0), 2)
        self.assertEqual(selector.update(0.39, 0.35), 2)
        self.assertEqual(selector.update(0.39, 0.349), 1)


if __name__ == "__main__":
    unittest.main()
