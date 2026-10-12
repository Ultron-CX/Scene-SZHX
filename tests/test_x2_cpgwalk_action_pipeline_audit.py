import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "admin" / "spark" / "x2-cpgwalk-action-pipeline-audit.py"
SPEC = importlib.util.spec_from_file_location("x2_cpgwalk_action_pipeline_audit", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class X2CpgWalkActionPipelineAuditTests(unittest.TestCase):
    def row(self):
        row = {"policy_dt": "0.02", "current_gait": "1", "is_standing": "0"}
        for stem in ("action", "residual", "q_nominal", "q_target", "q_filtered"):
            for index in range(MODULE.WIDTH):
                row[f"{stem}_{index}"] = "0"
        return row

    def test_complete_requires_all_finite_vectors(self):
        row = self.row()
        self.assertTrue(MODULE.complete(row))
        row["q_filtered_16"] = ""
        self.assertFalse(MODULE.complete(row))
        row["q_filtered_16"] = "nan"
        self.assertFalse(MODULE.complete(row))

    def test_config_parser_requires_deployed_joint_order(self):
        config = {
            "BaseConfig": {
                "model_type": "CPGWalk",
                "action_seq": list(MODULE.X2_CPGWALK_JOINT_NAMES),
                "action_scale": [0.2] * MODULE.WIDTH,
            },
            "ActorCriticConfig": {
                "planner": {"enable": False},
                "clips": {"action_clip": 18.0},
                "robot": {
                    "default_dof_pos": [0.0] * MODULE.WIDTH,
                    "kps": [10.0] * MODULE.WIDTH,
                    "kds": [1.0] * MODULE.WIDTH,
                },
            },
            "CPGWalkConfig": {
                "filter": {"action_cutoff_freq": [50.0] * 31},
                "cpgwalk": {
                    "enable_cpg": True,
                    "cpg_profile": "phase_only",
                    "waist_pitch_residual_scale": 0.5,
                    "waist_roll_residual_scale": 0.7,
                    "waist_pitch_residual_max_abs": 0.25,
                    "waist_roll_residual_max_abs": 0.14,
                },
            },
        }
        params = MODULE.params_from_config(config)
        params.validate()
        config["BaseConfig"]["action_seq"][0] = "wrong_joint"
        with self.assertRaisesRegex(ValueError, "joint order"):
            MODULE.params_from_config(config).validate()


if __name__ == "__main__":
    unittest.main()
