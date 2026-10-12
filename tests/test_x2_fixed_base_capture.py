import pathlib
import tempfile
import unittest

from sceneforge.x2_fixed_base_capture import (
    DEFAULT_JOINT,
    X2FixedBaseCommandCapture,
)


class FakeSim:
    joint_names = (DEFAULT_JOINT, "right_shoulder_pitch_joint")

    def __init__(self):
        self.position = [0.0, 0.0]
        self.velocity = [0.0, 0.0]
        self.target = 0.0
        self.calls = []

    def get_joint_states(self):
        return tuple(self.position), tuple(self.velocity)

    def apply_commands(self, **packet):
        self.calls.append(packet)
        self.target = packet["cmd_positions"][DEFAULT_JOINT]

    def advance(self, dt):
        previous = self.position[0]
        self.position[0] += (self.target - self.position[0]) * 0.25
        self.velocity[0] = (self.position[0] - previous) / dt


class FixedBaseCaptureTests(unittest.TestCase):
    def test_bounded_single_joint_response_and_restore_passes(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            sim = FakeSim()
            capture = X2FixedBaseCommandCapture(
                sim,
                receipt=root / "receipt.json",
                trace=root / "trace.json",
                stop_request=root / "stop.json",
                minimum_command_count=8,
            )
            dt = 0.01
            for step in range(400):
                sim.advance(dt)
                capture.step(step * dt)
                if capture.completed:
                    break
            self.assertEqual(capture.status, "PASSED")
            self.assertGreaterEqual(len(sim.calls), 8)
            self.assertTrue(all(tuple(call["cmd_positions"]) == (DEFAULT_JOINT,) for call in sim.calls))
            self.assertTrue(all(not call["cmd_efforts"] for call in sim.calls))
            self.assertAlmostEqual(sim.calls[-1]["cmd_positions"][DEFAULT_JOINT], 0.0)

    def test_rejects_scope_and_amplitude_expansion(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            sim = FakeSim()
            with self.assertRaises(ValueError):
                X2FixedBaseCommandCapture(
                    sim,
                    receipt=root / "a.json",
                    trace=root / "b.json",
                    stop_request=root / "c.json",
                    joint_name="right_shoulder_pitch_joint",
                )
            with self.assertRaises(ValueError):
                X2FixedBaseCommandCapture(
                    sim,
                    receipt=root / "d.json",
                    trace=root / "e.json",
                    stop_request=root / "f.json",
                    amplitude_rad=0.031,
                )

    def test_driver_has_no_ros_command_publisher(self):
        root = pathlib.Path(__file__).parents[1]
        source = (root / "admin/spark/x2-geniesim-fixed-base-capture-driver.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("session.post_step_hooks.append(capture.step)", source)
        for forbidden in ("create_publisher", '"/joint_command"', '"/cmd_vel"', "rclpy"):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
