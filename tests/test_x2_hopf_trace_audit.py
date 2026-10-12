import importlib.util
import math
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "admin" / "spark" / "x2-hopf-trace-audit.py"
SPEC = importlib.util.spec_from_file_location("x2_hopf_trace_audit", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class X2HopfTraceAuditTests(unittest.TestCase):
    def test_complete_requires_finite_contract_fields(self):
        row = {
            "policy_dt": "0.02",
            "current_gait": "1",
            "freeze": "0",
            "hopf_period": "0.87",
            "hopf_mu": "1.0",
            "hopf_vx": "0.3",
            "hopf_vy": "0",
            "hopf_wz": "0",
            "cmd_y": "0",
            "cmd_yaw": "0",
            **{f"obs_phase_{index}": "0" for index in range(4)},
        }
        self.assertTrue(MODULE.complete(row))
        row["hopf_mu"] = "nan"
        self.assertFalse(MODULE.complete(row))

    def test_percentile_is_nearest_rank(self):
        self.assertEqual(MODULE.percentile([1.0, 2.0, 3.0, 4.0], 0.5), 2.0)
        self.assertEqual(MODULE.percentile([1.0, 2.0, 3.0, 4.0], 0.95), 4.0)


if __name__ == "__main__":
    unittest.main()
