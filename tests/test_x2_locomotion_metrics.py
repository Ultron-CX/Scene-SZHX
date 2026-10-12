import math
import unittest

from sceneforge.x2_locomotion_metrics import (
    pose_derived_forward_speed,
    pose_derived_local_displacement,
    pose_derived_turn_rate,
    summarize_stable_stop,
    summarize_support_slip,
)


class X2LocomotionMetricsTests(unittest.TestCase):
    def test_steady_support_excludes_light_contact_and_edges(self):
        samples = []
        for index in range(31):
            now = index * 0.01
            samples.append({
                "sim_time": now,
                "contact": True,
                "normal_force_n": 10.0 if index == 0 else 100.0,
                "tangential_slip_speed_mps": 0.5 if index in {0, 1, 30} else 0.02,
            })
        result = summarize_support_slip(samples, load_threshold_n=20.0, edge_trim_s=0.10)
        self.assertEqual(result["raw_contact_sample_count"], 31)
        self.assertEqual(result["loaded_support_segment_count"], 1)
        self.assertGreater(result["raw_contact_slip_p95_mps"], 0.02)
        self.assertAlmostEqual(result["steady_support_slip_p95_mps"], 0.02)

    def test_rejects_non_monotonic_time(self):
        samples = [
            {"sim_time": 1.0, "contact": True, "normal_force_n": 30.0, "tangential_slip_speed_mps": 0.0},
            {"sim_time": 1.0, "contact": True, "normal_force_n": 30.0, "tangential_slip_speed_mps": 0.0},
        ]
        with self.assertRaisesRegex(ValueError, "strictly increase"):
            summarize_support_slip(samples)

    def test_pose_derived_speed_projects_world_motion_to_heading(self):
        samples = [
            {
                "sim_time": float(index), "phase": "GAIT1_FORWARD",
                "position_m": [0.0, 0.3 * index, 0.6],
                "yaw_rad": 1.5707963267948966,
            }
            for index in range(8)
        ]
        result = pose_derived_forward_speed(samples, trim_start_s=1.0, trim_end_s=1.0)
        self.assertAlmostEqual(result["mean_forward_speed_mps"], 0.3)
        self.assertEqual(result["sample_count"], 6)

    def test_pose_derived_turn_rate_unwraps_yaw(self):
        samples = [
            {
                "sim_time": float(index), "phase": "GAIT1_FORWARD",
                "position_m": [0.0, 0.0, 0.6],
                "yaw_rad": ((3.0 + 0.25 * index + 3.141592653589793)
                            % (2.0 * 3.141592653589793)) - 3.141592653589793,
            }
            for index in range(8)
        ]
        result = pose_derived_turn_rate(samples, trim_start_s=1.0, trim_end_s=1.0)
        self.assertAlmostEqual(result["mean_yaw_rate_rps"], 0.25)
        self.assertAlmostEqual(result["yaw_change_rad"], 1.25)

    def test_local_displacement_uses_walk_entry_heading(self):
        samples = [
            {
                "sim_time": float(index), "phase": "GAIT1_FORWARD",
                "position_m": [0.02 * index, 0.3 * index, 0.6],
                "yaw_rad": math.pi / 2.0,
            }
            for index in range(5)
        ]
        result = pose_derived_local_displacement(samples)
        self.assertAlmostEqual(result["local_forward_displacement_m"], 1.2)
        self.assertAlmostEqual(result["local_cross_track_m"], -0.08)
        self.assertAlmostEqual(result["body_forward_distance_m"], 1.2)
        self.assertAlmostEqual(
            result["local_cross_track_per_forward_m"], 0.08 / 1.2
        )

    def test_local_displacement_accumulates_forward_arc(self):
        samples = []
        radius = 2.0
        for index in range(5):
            yaw = 0.2 * index
            samples.append({
                "sim_time": float(index), "phase": "GAIT1_FORWARD",
                "position_m": [radius * math.sin(yaw), radius * (1.0 - math.cos(yaw)), 0.6],
                "yaw_rad": yaw,
            })
        result = pose_derived_local_displacement(samples)
        self.assertGreater(result["body_forward_distance_m"], 1.5)
        self.assertLess(result["body_forward_distance_m"], 1.7)

    def test_stable_stop_uses_final_window(self):
        samples = [
            {
                "sim_time": float(index), "phase": "STOP_STAND",
                "position_m": [0.01 * index, 0.0, 0.6],
                "linear_speed_mps": 0.02,
                "angular_speed_rps": 0.01,
            }
            for index in range(5)
        ]
        result = summarize_stable_stop(samples, window_s=2.0)
        self.assertEqual(result["sample_count"], 3)
        self.assertAlmostEqual(result["linear_speed_p95_mps"], 0.02)
        self.assertAlmostEqual(result["planar_drift_m"], 0.02)


if __name__ == "__main__":
    unittest.main()
