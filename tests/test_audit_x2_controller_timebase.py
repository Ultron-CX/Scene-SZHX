import importlib.util
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "admin" / "spark" / "audit-x2-controller-timebase.py"
SPEC = importlib.util.spec_from_file_location("audit_x2_timebase", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class TimebaseAuditTests(unittest.TestCase):
    def test_detects_wall_clock_contract_and_control_dt_hook(self):
        texts = {
            "managed_periodic_worker": "executor_.ExecutePeriodically(period, task);",
            "time_wheel_executor": (
                "std::chrono::steady_clock::now(); std::this_thread::sleep_until("
                "last_loop_std_tp);"
            ),
            "executor_iface": "void ExecuteAt(TimePoint, Task);",
            "control_module": (
                "UpdateActiveControllersInline(const time_point<steady_clock>& now);"
            ),
            "controller_base": (
                "virtual void SetControlDt(double) {} "
                "std::chrono::steady_clock::time_point joint_state_receive_time_;"
            ),
        }
        checks = MODULE.source_checks(texts)
        self.assertTrue(all(checks.values()))

    def test_external_tick_prevents_no_external_tick_check(self):
        texts = {
            "managed_periodic_worker": "ExecutePeriodically(period",
            "time_wheel_executor": "std::chrono::steady_clock::now(); std::this_thread::sleep_until(last_loop_std_tp);",
            "executor_iface": "ExecuteAt(TimePoint); ExternalTick();",
            "control_module": "UpdateActiveControllersInline time_point<steady_clock>",
            "controller_base": "virtual void SetControlDt(double) {} std::chrono::steady_clock::time_point joint_state_receive_time_;",
        }
        self.assertFalse(MODULE.source_checks(texts)["executor_interface_has_no_external_tick"])


if __name__ == "__main__":
    unittest.main()
