import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "admin" / "spark" / "x2-cmd-vel-fixture.py"
SPEC = importlib.util.spec_from_file_location("x2_cmd_vel_fixture", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class CommandFixtureTests(unittest.TestCase):
    def command(self, **updates):
        data = {
            "schema": MODULE.COMMAND_SCHEMA,
            "sequence": 1,
            "linear_x_mps": 0.3,
            "linear_y_mps": 0.0,
            "angular_z_rps": 0.0,
            "real_robot_commands": False,
        }
        data.update(updates)
        return data

    def load(self, payload, previous=-1):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "command.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            return MODULE.load_command(path, previous)

    def test_valid_simulation_command(self):
        command = self.load(self.command())
        self.assertEqual(0.3, command["linear_x_mps"])
        self.assertEqual(1, command["sequence"])

    def test_sequence_must_increase(self):
        with self.assertRaises(ValueError):
            self.load(self.command(sequence=2), previous=2)

    def test_real_robot_flag_is_rejected(self):
        with self.assertRaises(ValueError):
            self.load(self.command(real_robot_commands=True))

    def test_vendor_range_is_enforced(self):
        with self.assertRaises(ValueError):
            self.load(self.command(linear_x_mps=1.01))

    def test_source_has_no_physical_robot_transport(self):
        source = MODULE_PATH.read_text(encoding="utf-8")
        self.assertNotIn("203.0.113.", source)
        self.assertNotIn("ssh", source.lower())

    def test_quaternion_yaw(self):
        self.assertAlmostEqual(0.0, MODULE.quaternion_yaw(0.0, 0.0, 0.0, 1.0))
        half = 2 ** -0.5
        self.assertAlmostEqual(1.5707963267948966, MODULE.quaternion_yaw(0.0, 0.0, half, half))

    def test_heading_correction_wraps_and_clamps(self):
        self.assertAlmostEqual(0.2, MODULE.heading_correction(0.0, -0.2, 1.0, 0.3))
        self.assertAlmostEqual(-0.3, MODULE.heading_correction(0.0, 1.0, 1.0, 0.3))
        self.assertAlmostEqual(0.02, MODULE.heading_correction(-3.13, 3.1331853071795863, 1.0, 0.3), places=6)


if __name__ == "__main__":
    unittest.main()
