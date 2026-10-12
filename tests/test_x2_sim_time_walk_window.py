import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "admin" / "spark" / "x2-sim-time-walk-window.py"
SPEC = importlib.util.spec_from_file_location("x2_walk_window", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def status(sim_time, x=0.0, y=0.0, z=0.8, vx=0.0):
    return {
        "schema": MODULE.STATUS_SCHEMA,
        "state": "RELEASED",
        "support_ratio": 0.0,
        "force_norm_n": 0.0,
        "force_is_exactly_zero": True,
        "root_pinned": False,
        "gravity_modified": False,
        "real_robot_commands": False,
        "sim_time_s": sim_time,
        "sample_count": int(sim_time * 500),
        "body_position_m": [x, y, z],
        "body_linear_velocity_mps": [vx, 0.0, 0.0],
    }


class WalkWindowTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        self.status_path = root / "status.json"
        self.receipt_path = root / "receipt.json"

    def tearDown(self):
        self.temp.cleanup()

    def write_status(self, payload):
        self.status_path.write_text(json.dumps(payload), encoding="utf-8")

    def test_sim_time_window_round_trip(self):
        self.write_status(status(10.0))
        MODULE.start(self.status_path, self.receipt_path, 0.3, 4.0, 2.0)
        self.write_status(status(13.9, x=1.15, vx=0.3))
        self.assertFalse(MODULE.reached(self.status_path, self.receipt_path, "walk"))
        self.write_status(status(14.1, x=1.2, vx=0.3))
        self.assertTrue(MODULE.reached(self.status_path, self.receipt_path, "walk"))
        MODULE.mark_stop(self.status_path, self.receipt_path)
        self.write_status(status(16.2, x=1.22, vx=0.01))
        self.assertTrue(MODULE.reached(self.status_path, self.receipt_path, "settle"))
        MODULE.finish(self.status_path, self.receipt_path)
        receipt = json.loads(self.receipt_path.read_text(encoding="utf-8"))
        self.assertEqual("PASSED", receipt["status"])
        self.assertAlmostEqual(1.2, receipt["forward_displacement_m"])
        self.assertFalse(receipt["real_robot_commands"])

    def test_loaded_band_is_rejected(self):
        loaded = status(1.0)
        loaded["state"] = "ENGAGED"
        loaded["support_ratio"] = 1.0
        self.write_status(loaded)
        with self.assertRaises(ValueError):
            MODULE.start(self.status_path, self.receipt_path, 0.3, 4.0, 2.0)

    def test_early_stop_is_rejected(self):
        self.write_status(status(10.0))
        MODULE.start(self.status_path, self.receipt_path, 0.3, 4.0, 2.0)
        self.write_status(status(12.0, x=0.6, vx=0.3))
        with self.assertRaises(ValueError):
            MODULE.mark_stop(self.status_path, self.receipt_path)

    def test_nonzero_final_speed_fails(self):
        self.write_status(status(1.0))
        MODULE.start(self.status_path, self.receipt_path, 0.3, 2.0, 1.0)
        self.write_status(status(3.1, x=0.6, vx=0.3))
        MODULE.mark_stop(self.status_path, self.receipt_path)
        self.write_status(status(4.2, x=0.7, vx=0.2))
        with self.assertRaises(RuntimeError):
            MODULE.finish(self.status_path, self.receipt_path)
        receipt = json.loads(self.receipt_path.read_text(encoding="utf-8"))
        self.assertEqual("FAILED", receipt["status"])

    def test_staged_deceleration_round_trip(self):
        self.write_status(status(10.0))
        MODULE.start(self.status_path, self.receipt_path, 0.45, 3.0, 2.0, 0.3, 2.0)
        self.write_status(status(13.1, x=1.35, vx=0.45))
        MODULE.mark_deceleration(self.status_path, self.receipt_path)
        self.write_status(status(15.2, x=1.95, vx=0.3))
        self.assertTrue(MODULE.reached(self.status_path, self.receipt_path, "deceleration"))
        MODULE.mark_stop(self.status_path, self.receipt_path)
        self.write_status(status(17.3, x=1.98, vx=0.01))
        MODULE.finish(self.status_path, self.receipt_path)
        receipt = json.loads(self.receipt_path.read_text(encoding="utf-8"))
        self.assertEqual("PASSED", receipt["status"])
        self.assertAlmostEqual(1.95, receipt["expected_distance_m"])
        self.assertGreaterEqual(receipt["observed_deceleration_duration_s"], 2.0)

    def test_staged_deceleration_rejects_early_stop(self):
        self.write_status(status(10.0))
        MODULE.start(self.status_path, self.receipt_path, 0.45, 3.0, 2.0, 0.3, 2.0)
        self.write_status(status(13.1, x=1.35, vx=0.45))
        MODULE.mark_deceleration(self.status_path, self.receipt_path)
        self.write_status(status(14.0, x=1.6, vx=0.3))
        with self.assertRaises(ValueError):
            MODULE.mark_stop(self.status_path, self.receipt_path)


if __name__ == "__main__":
    unittest.main()
