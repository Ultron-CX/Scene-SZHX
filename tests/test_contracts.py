import json
import unittest
from pathlib import Path

from pydantic import ValidationError

from sceneforge.models import (
    AssetMode,
    AssetRecord,
    BuildingSpec,
    SceneSpec,
    assert_asset_allowed,
)


ROOT = Path(__file__).resolve().parents[1]


class ContractTests(unittest.TestCase):
    def test_hospital_building_example(self) -> None:
        for seed in (1101, 1102, 1103):
            data = json.loads((ROOT / f"examples/hospital/building-seed-{seed}.json").read_text())
            spec = BuildingSpec.model_validate(data)
            self.assertEqual(spec.seed, seed)
            self.assertGreaterEqual(spec.clearance_profile.minimum_door_clear_width_m, 1.2)

    def test_hospital_scene_example(self) -> None:
        for seed in (1101, 1102, 1103):
            data = json.loads((ROOT / f"examples/hospital/scene-seed-{seed}.json").read_text())
            spec = SceneSpec.model_validate(data)
            self.assertEqual(spec.seed, seed)
            self.assertFalse(spec.real_robot_commands)
            self.assertEqual(spec.patrol_route[0], "poi_entrance")

    def test_unknown_adjacency_is_rejected(self) -> None:
        data = json.loads((ROOT / "examples/hospital/building-seed-1101.json").read_text())
        data["adjacency"].append({"a": "entrance", "b": "missing_room"})
        with self.assertRaises(ValidationError):
            BuildingSpec.model_validate(data)

    def test_real_robot_commands_cannot_be_enabled(self) -> None:
        data = json.loads((ROOT / "examples/hospital/scene-seed-1101.json").read_text())
        data["real_robot_commands"] = True
        with self.assertRaises(ValidationError):
            SceneSpec.model_validate(data)

    def test_production_assets_fail_closed(self) -> None:
        data = {
            "schema_version": "sceneforge/asset-record/v1",
            "asset_id": "waiting_chair_01",
            "source": "geniesimassets",
            "source_version": "unverified",
            "uri": "asset://waiting_chair_01",
            "sha256": "0" * 64,
            "license": {
                "spdx_id": "NOASSERTION",
                "commercial_use_allowed": False,
                "redistribution_allowed": False,
                "source_url": "https://example.invalid/asset"
            },
            "geometry": {"width_m": 0.6, "depth_m": 0.6, "height_m": 0.9},
            "physics": {"collision_status": "authored", "rigid_body": False},
            "semantic_tags": ["chair", "waiting"],
            "approval": "research_only"
        }
        asset = AssetRecord.model_validate(data)
        assert_asset_allowed(asset, AssetMode.RESEARCH)
        with self.assertRaises(ValueError):
            assert_asset_allowed(asset, AssetMode.PRODUCTION)


if __name__ == "__main__":
    unittest.main()
