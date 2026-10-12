import pathlib
import unittest


ROOT = pathlib.Path(__file__).parents[1]
RUNNER = ROOT / "admin/spark/run-x2-geniesim-fixed-controller-gate.sh"


class FixedControllerRunnerBuildTests(unittest.TestCase):
    def test_fixed_and_floating_use_relocatable_copy_install(self):
        source = RUNNER.read_text(encoding="utf-8")
        self.assertEqual(source.count("X2_GENIESIM_BUILD_ROOT:-/project/geniesim-build-effort"), 2)
        self.assertNotIn("geniesim_build=/project/geniesim-build\n", source)

    def test_fixed_oracle_preserves_vendor_debug_even_when_sync_gate_fails(self):
        source = RUNNER.read_text(encoding="utf-8")
        self.assertIn('if [[ "$base_mode" == fixed && "$control_mode" != walk ]]', source)
        self.assertIn("capture_controller_debug || true", source)


if __name__ == "__main__":
    unittest.main()
