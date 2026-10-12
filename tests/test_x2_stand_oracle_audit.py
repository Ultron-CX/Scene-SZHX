import importlib.util
import pathlib
import sys
import unittest


ROOT = pathlib.Path(__file__).parents[1]
SCRIPT = ROOT / "admin/spark/x2-stand-oracle-audit.py"
SPEC = importlib.util.spec_from_file_location("x2_stand_oracle_audit", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class StandOracleAuditTests(unittest.TestCase):
    def test_vector_is_strict_and_finite(self):
        self.assertEqual(MODULE.vector("1 2", 2, "x"), (1.0, 2.0))
        with self.assertRaises(ValueError):
            MODULE.vector("1", 2, "x")
        with self.assertRaises(ValueError):
            MODULE.vector("1 nan", 2, "x")

    def test_source_has_no_transport(self):
        source = SCRIPT.read_text(encoding="utf-8")
        for forbidden in ("rclpy", "create_publisher", "apply_commands", '"/joint_command"'):
            self.assertNotIn(forbidden, source)
        self.assertIn('"real_robot_commands": False', source)


if __name__ == "__main__":
    unittest.main()
