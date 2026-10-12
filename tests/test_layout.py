import json
import tempfile
import unittest
from pathlib import Path

from sceneforge.models import BuildingSpec, SceneSpec
from sceneforge.layout import plan_hospital
from sceneforge.usd_export import export_scene_bundle


ROOT = Path(__file__).resolve().parents[1]


def load_templates() -> tuple[BuildingSpec, SceneSpec]:
    building = BuildingSpec.model_validate_json(
        (ROOT / "examples/hospital/building-seed-1101.json").read_text()
    )
    scene = SceneSpec.model_validate_json(
        (ROOT / "examples/hospital/scene-seed-1101.json").read_text()
    )
    return building, scene


def with_seed(building: BuildingSpec, scene: SceneSpec, seed: int) -> tuple[BuildingSpec, SceneSpec]:
    updated_building = building.model_copy(
        update={"building_id": f"hospital_outpatient_{seed}", "seed": seed}
    )
    updated_scene = scene.model_copy(
        update={
            "scene_id": f"hospital_service_{seed}",
            "building_id": updated_building.building_id,
            "seed": seed,
        }
    )
    return updated_building, updated_scene


class HospitalLayoutTests(unittest.TestCase):
    def test_three_fixed_seeds_are_deterministic_and_distinct(self) -> None:
        template_building, template_scene = load_templates()
        variants = []
        layouts = []
        for seed in (1101, 1102, 1103):
            building, scene = with_seed(template_building, template_scene, seed)
            first = plan_hospital(building, scene)
            second = plan_hospital(building, scene)
            self.assertEqual(first.model_dump_json(), second.model_dump_json())
            variants.append(first.variant)
            layouts.append(first.model_dump_json())
        self.assertEqual(set(variants), {"spine", "mirror_x", "mirror_y"})
        self.assertEqual(len(set(layouts)), 3)

    def test_doors_have_no_wall_segment_across_opening(self) -> None:
        building, scene = load_templates()
        layout = plan_hospital(building, scene)
        for door in layout.doors:
            line_coordinate = door.center_x_m if door.wall_axis == "x" else door.center_y_m
            opening_center = door.center_y_m if door.wall_axis == "x" else door.center_x_m
            opening_start = opening_center - door.clear_width_m / 2
            opening_end = opening_center + door.clear_width_m / 2
            for wall in layout.walls:
                if wall.wall_axis != door.wall_axis:
                    continue
                wall_coordinate = wall.center_x_m if wall.wall_axis == "x" else wall.center_y_m
                if abs(line_coordinate - wall_coordinate) > 1e-8:
                    continue
                wall_center = wall.center_y_m if wall.wall_axis == "x" else wall.center_x_m
                wall_start = wall_center - wall.length_m / 2
                wall_end = wall_center + wall.length_m / 2
                overlap = min(opening_end, wall_end) - max(opening_start, wall_start)
                self.assertLessEqual(overlap, 1e-8)

    def test_export_is_reproducible(self) -> None:
        building, scene = load_templates()
        layout = plan_hospital(building, scene)
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            first = export_scene_bundle(root / "first", building, scene, layout)
            second = export_scene_bundle(root / "second", building, scene, layout)
            first_manifest = json.loads(first.read_text())
            second_manifest = json.loads(second.read_text())
            self.assertEqual(first_manifest["files"], second_manifest["files"])
            self.assertFalse(first_manifest["real_robot_commands"])
            self.assertEqual(first_manifest["variant"], "spine")
            physics = (root / "first/physics.usda").read_text()
            self.assertIn('over "floor" (\n            prepend apiSchemas', physics)
            self.assertNotIn('{\n            prepend apiSchemas', physics)


if __name__ == "__main__":
    unittest.main()
