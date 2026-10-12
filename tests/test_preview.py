import unittest

from test_layout import load_templates, with_seed
from sceneforge.layout import plan_hospital
from sceneforge.preview import render_layout_svg


class PreviewTests(unittest.TestCase):
    def test_preview_is_deterministic_and_complete(self) -> None:
        building, scene = load_templates()
        layout = plan_hospital(building, scene)
        first = render_layout_svg(layout, building)
        second = render_layout_svg(layout, building)
        self.assertEqual(first, second)
        self.assertEqual(first.count('data-space="'), 11)
        self.assertEqual(first.count('data-wall="'), 43)
        self.assertEqual(first.count('data-door="'), 11)
        self.assertIn("not X2 qualified", first)

    def test_seed_variants_render_differently(self) -> None:
        template_building, template_scene = load_templates()
        previews = []
        for seed in (1101, 1102, 1103):
            building, scene = with_seed(template_building, template_scene, seed)
            previews.append(render_layout_svg(plan_hospital(building, scene), building))
        self.assertEqual(len(set(previews)), 3)


if __name__ == "__main__":
    unittest.main()

