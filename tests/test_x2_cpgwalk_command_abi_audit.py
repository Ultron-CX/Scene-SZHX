import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "admin" / "spark" / "x2-cpgwalk-command-abi-audit.py"
SPEC = importlib.util.spec_from_file_location("x2_cpgwalk_command_abi_audit", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class X2CpgWalkCommandAbiAuditTests(unittest.TestCase):
    def test_symbol_bounds_uses_next_ordered_symbol(self):
        nm = "\n".join(
            (
                "00000010 T before()",
                f"00000020 T {MODULE.SYMBOL}",
                "00000080 T after()",
            )
        )
        self.assertEqual(MODULE.symbol_bounds(nm), (0x20, 0x80))

    def test_instruction_contract_requires_two_zero_lanes(self):
        accepted = "\n".join(
            (
                "str d2, [x1, x0]",
                "str xzr, [x1, x0]",
                "str xzr, [x1, x0]",
                "str d1, [x1, x0]",
                "str d0, [x1, x0]",
            )
        )
        self.assertTrue(MODULE.instruction_contract(accepted)["passed"])
        self.assertFalse(
            MODULE.instruction_contract(accepted.replace("str xzr", "str d3", 1))["passed"]
        )


if __name__ == "__main__":
    unittest.main()
