import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "admin" / "spark" / "x2-cpgwalk-safety-audit.py"
SPEC = importlib.util.spec_from_file_location("x2_cpgwalk_safety_audit", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class X2CpgWalkSafetyAuditTests(unittest.TestCase):
    def test_complete_rejects_missing_or_non_finite_signal(self):
        row = {
            name: "0"
            for name in (
                "t", "policy_dt", "cmd_x", "cmd_y", "cmd_yaw", "current_gait",
                "is_standing", "freeze", "freeze_att", "freeze_cmd_candidate",
                "low_motion_ready", "base_roll", "base_pitch", "mean_leg_vel",
                "base_ang_vel_norm",
            )
        }
        self.assertTrue(MODULE.complete(row))
        row["base_roll"] = "nan"
        self.assertFalse(MODULE.complete(row))

    def test_auto_stand_log_parser_is_source_specific(self):
        text = "\n".join(
            (
                "[INFO] [10.25] [node]: auto request trigger=/stand_default_mode "
                "current=LOCOMOTION_DEFAULT source=rl_cpgwalk_locomotion_default",
                "[INFO] [11.00] [node]: auto request trigger=/stand_default_mode "
                "current=OTHER source=not_cpgwalk",
            )
        )
        self.assertEqual(MODULE.auto_stand_timestamps(text), [10.25])


if __name__ == "__main__":
    unittest.main()
