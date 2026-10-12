import importlib.util
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "admin" / "spark" / "x2-clock-freeze-probe.py"
SPEC = importlib.util.spec_from_file_location("x2_clock_freeze_probe", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def event(wall_time, stamp_ns=100, fingerprint="same"):
    return {"wall_time": wall_time, "stamp_ns": stamp_ns, "fingerprint": fingerprint}


class ClockFreezeProbeTests(unittest.TestCase):
    def setUp(self):
        self.status = {
            "started_at_unix": 10.0,
            "finished_at_unix": 12.0,
            "sim_time_before_s": 4.5,
            "sim_time_after_s": 4.5,
        }

    def test_sim_time_bound_allows_one_edge_callback(self):
        commands = {lane: [event(10.0)] for lane in MODULE.LANES}
        result, metrics = MODULE.classify_freeze(
            self.status, [event(10.0)], [], commands
        )
        self.assertEqual(result, "SIM_TIME_BOUND")
        self.assertTrue(metrics["engine_frozen"])

    def test_wall_clock_bound_requires_all_lanes_to_continue(self):
        commands = {
            lane: [event(10.1 + index * 0.1, index, str(index)) for index in range(10)]
            for lane in MODULE.LANES
        }
        result, metrics = MODULE.classify_freeze(self.status, [], [], commands)
        self.assertEqual(result, "CONTROLLER_WALL_CLOCK_BOUND")
        self.assertEqual(metrics["command_messages_during_freeze"]["leg"], 10)
        self.assertEqual(metrics["command_fingerprint_changes_during_freeze"]["leg"], 9)

    def test_freeze_not_proven_when_sim_time_advances(self):
        status = dict(self.status, sim_time_after_s=4.6)
        result, _ = MODULE.classify_freeze(
            status, [], [], {lane: [] for lane in MODULE.LANES}
        )
        self.assertEqual(result, "FREEZE_NOT_PROVEN")

    def test_interval_excludes_outside_messages(self):
        selected = MODULE.count_interval([event(9.9), event(10.0), event(12.0), event(12.1)], 10, 12)
        self.assertEqual(len(selected), 2)

    def test_completed_status_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "status.json"
            path.write_text('{"schema":"wrong"}', encoding="utf-8")
            with self.assertRaises(ValueError):
                MODULE.load_completed_status(path)


if __name__ == "__main__":
    unittest.main()
