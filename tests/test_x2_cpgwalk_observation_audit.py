import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "admin" / "spark" / "x2-cpgwalk-observation-audit.py"
SPEC = importlib.util.spec_from_file_location("x2_cpgwalk_observation_audit", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class X2CpgWalkObservationAuditTests(unittest.TestCase):
    def row(self):
        row = {
            "base_ang_vel_x": "1",
            "base_ang_vel_y": "2",
            "base_ang_vel_z": "3",
            "obs_gvec_x": "4",
            "obs_gvec_y": "5",
            "obs_gvec_z": "6",
            "cmd_x": "0.1",
            "cmd_y": "0.2",
            "cmd_yaw": "0.3",
            "is_standing": "0",
        }
        for index in range(17):
            row[f"q_{index}"] = str(index + 0.5)
            row[f"dq_{index}"] = str(index + 1)
        for index in range(4):
            row[f"obs_phase_{index}"] = str(10 + index)
        return row

    def test_builds_deployed_layout_and_scales(self):
        obs = MODULE.build_observation(
            self.row(),
            frame=0,
            previous_action=tuple(range(17)),
            default_dof_position=(0.5,) * 17,
            lin_vel_scale=2.0,
            ang_vel_scale=0.25,
            dof_pos_scale=1.0,
            dof_vel_scale=0.05,
            command_obs_use_motion_flag=False,
        )
        self.assertEqual(len(obs), 65)
        self.assertEqual(obs[0:3], (0.25, 0.5, 0.75))
        self.assertEqual(obs[3:6], (4.0, 5.0, 6.0))
        self.assertEqual(obs[6:10], (0.2, 0.4, 0.075, 0.0))
        self.assertEqual(obs[10], 0.0)
        self.assertEqual(obs[26], 16.0)
        self.assertEqual(obs[27], 0.05)
        self.assertAlmostEqual(obs[43], 0.85)
        self.assertEqual(obs[44:61], tuple(float(value) for value in range(17)))
        self.assertEqual(obs[61:65], (10.0, 11.0, 12.0, 13.0))

    def test_motion_flag_is_one_only_outside_standing(self):
        kwargs = dict(
            frame=0,
            previous_action=(0.0,) * 17,
            default_dof_position=(0.0,) * 17,
            lin_vel_scale=1.0,
            ang_vel_scale=1.0,
            dof_pos_scale=1.0,
            dof_vel_scale=1.0,
            command_obs_use_motion_flag=True,
        )
        row = self.row()
        row["is_standing"] = "0"
        self.assertEqual(MODULE.build_observation(row, **kwargs)[9], 1.0)
        row["is_standing"] = "1"
        self.assertEqual(MODULE.build_observation(row, **kwargs)[9], 0.0)

    def test_rejects_non_finite_state(self):
        row = self.row()
        row["base_ang_vel_x"] = "nan"
        with self.assertRaisesRegex(ValueError, "NaN or Inf"):
            MODULE.build_observation(
                row,
                frame=4,
                previous_action=(0.0,) * 17,
                default_dof_position=(0.0,) * 17,
                lin_vel_scale=1.0,
                ang_vel_scale=1.0,
                dof_pos_scale=1.0,
                dof_vel_scale=1.0,
                command_obs_use_motion_flag=False,
            )


if __name__ == "__main__":
    unittest.main()
