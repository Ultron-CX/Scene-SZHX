import csv
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "admin" / "spark" / "x2-onnx-parity-probe.py"
SPEC = importlib.util.spec_from_file_location("x2_onnx_parity_probe", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class X2OnnxParityProbeTests(unittest.TestCase):
    def write_csv(self, path: Path, header: list[str], rows: list[list[str]]) -> None:
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(header)
            writer.writerows(rows)

    def test_reads_aligned_finite_vendor_rows(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            frames = root / "frames.csv"
            outputs = root / "outputs.csv"
            self.write_csv(
                frames,
                ["frame", "profile", "frame_values"],
                [["7", "cpgtelecon", "1 2 3"], ["8", "cpgtelecon", "4 5 6"]],
            )
            self.write_csv(
                outputs,
                ["frame", "profile", "values"],
                [["7", "cpgtelecon", "0.1 0.2"], ["8", "cpgtelecon", "0.3 0.4"]],
            )
            rows = list(MODULE.iter_parity_rows(frames, outputs))
            self.assertEqual([row.frame for row in rows], [7, 8])
            self.assertEqual(rows[0].inputs, (1.0, 2.0, 3.0))
            self.assertEqual(rows[1].expected_outputs, (0.3, 0.4))

    def test_rejects_mismatched_frame_numbers(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            frames = root / "frames.csv"
            outputs = root / "outputs.csv"
            self.write_csv(frames, ["frame", "profile", "frame_values"], [["1", "p", "1"]])
            self.write_csv(outputs, ["frame", "profile", "values"], [["2", "p", "1"]])
            with self.assertRaisesRegex(ValueError, "frame index mismatch"):
                list(MODULE.iter_parity_rows(frames, outputs))

    def test_rejects_non_finite_values(self):
        with self.assertRaisesRegex(ValueError, "NaN or Inf"):
            MODULE.parse_vector("1 nan 3", label="test")

    def test_rebuilds_oldest_to_newest_debug_history(self):
        with tempfile.TemporaryDirectory() as temporary:
            debug = Path(temporary) / "debug.csv"
            self.write_csv(
                debug,
                ["t", "obs_0", "obs_1", "action_0", "action_1"],
                [
                    ["0", "1", "2", "0.1", "0.2"],
                    ["1", "3", "4", "0.3", "0.4"],
                    ["2", "5", "6", "0.5", "0.6"],
                ],
            )
            rows = list(MODULE.iter_debug_parity_rows(debug, frame_stack=3))
            self.assertEqual(rows[0].inputs, (1.0, 2.0, 1.0, 2.0, 1.0, 2.0))
            self.assertEqual(rows[1].inputs, (1.0, 2.0, 1.0, 2.0, 3.0, 4.0))
            self.assertEqual(rows[2].inputs, (1.0, 2.0, 3.0, 4.0, 5.0, 6.0))
            self.assertEqual(rows[2].expected_outputs, (0.5, 0.6))

    def test_skips_only_a_truncated_physical_last_debug_row(self):
        with tempfile.TemporaryDirectory() as temporary:
            debug = Path(temporary) / "debug.csv"
            self.write_csv(
                debug,
                ["obs_0", "action_0"],
                [["1.0", "0.1"], ["2.0", ""]],
            )
            stats = MODULE.DebugSourceStats()
            rows = list(
                MODULE.iter_debug_parity_rows(debug, frame_stack=2, stats=stats)
            )
            self.assertEqual(len(rows), 1)
            self.assertEqual(stats.source_row_count, 2)
            self.assertEqual(stats.trailing_partial_rows_skipped, 1)

    def test_rejects_a_truncated_middle_debug_row(self):
        with tempfile.TemporaryDirectory() as temporary:
            debug = Path(temporary) / "debug.csv"
            self.write_csv(
                debug,
                ["obs_0", "action_0"],
                [["1.0", "0.1"], ["2.0", ""], ["3.0", "0.3"]],
            )
            with self.assertRaisesRegex(ValueError, "frame 1 is incomplete"):
                list(MODULE.iter_debug_parity_rows(debug, frame_stack=2))

    def test_error_metrics_and_gate(self):
        accumulator = MODULE.ErrorAccumulator()
        accumulator.add([1.0, 2.0], [1.0, 2.000001])
        accumulator.add([3.0], [3.000002])
        metrics = accumulator.summary()
        self.assertAlmostEqual(metrics["maximum_absolute_error"], 2e-6, places=10)
        self.assertTrue(
            MODULE.parity_passes(
                frame_count=100,
                metrics=metrics,
                minimum_frames=100,
                maximum_absolute_error=5e-6,
                maximum_rmse=2e-6,
            )
        )
        self.assertFalse(
            MODULE.parity_passes(
                frame_count=99,
                metrics=metrics,
                minimum_frames=100,
                maximum_absolute_error=5e-6,
                maximum_rmse=2e-6,
            )
        )

    def test_static_width_ignores_dynamic_batch(self):
        self.assertEqual(MODULE.static_width(["batch", 850]), 850)
        self.assertEqual(MODULE.static_width([1, 14]), 14)
        self.assertIsNone(MODULE.static_width(["batch", "features"]))


if __name__ == "__main__":
    unittest.main()
