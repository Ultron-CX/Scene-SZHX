import importlib.util
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "admin" / "prepare-x2-geniesim-overlay.py"
SPEC = importlib.util.spec_from_file_location("prepare_x2_overlay", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class PrepareX2OverlayTests(unittest.TestCase):
    def test_derived_overlay_is_safe_and_complete(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "vendor" / "x2.urdf"
            meshes = source.parent / "meshes"
            meshes.mkdir(parents=True)
            (meshes / "pelvis.STL").write_bytes(b"solid pelvis\nendsolid pelvis\n")
            source.write_text(
                """<robot name="x2"><link name="base_link"/>
<link name="pelvis"><inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial><visual><geometry><mesh filename="./meshes/pelvis.STL"/></geometry></visual></link>
<link name="imu_link"/><joint name="base" type="fixed"><parent link="base_link"/><child link="pelvis"/></joint><joint name="imu" type="fixed"><parent link="pelvis"/><child link="imu_link"/></joint></robot>""",
                encoding="utf-8",
            )
            output = root / "overlay"
            manifest = MODULE.prepare(source, output)
            self.assertTrue(manifest["ghost_root_guard_added"])
            self.assertTrue(manifest["simulation_root_inertial_added"])
            self.assertEqual(manifest["simulation_root_mass_kg"], 0.001)
            self.assertEqual(manifest["placeholder_inertial_links"], ["imu_link"])
            self.assertFalse(manifest["real_robot_commands"])
            derived = ET.parse(output / manifest["derived_urdf"]).getroot()
            base = next(x for x in derived.findall("link") if x.attrib["name"] == "base_link")
            self.assertEqual(base.find("visual/geometry/sphere").attrib["radius"], "0.001")
            self.assertEqual(base.find("inertial/mass").attrib["value"], "0.001")
            imu = next(x for x in derived.findall("link") if x.attrib["name"] == "imu_link")
            self.assertEqual(imu.find("inertial/mass").attrib["value"], "0.001")
            mesh = derived.find(".//mesh")
            self.assertEqual(mesh.attrib["filename"], "package://genie_sim_x2_model/meshes/pelvis.STL")
            self.assertTrue((output / "src/genie_sim_x2_model/meshes/pelvis.STL").is_file())
            package_xml = (output / "src/genie_sim_x2_model/package.xml").read_text(encoding="utf-8")
            self.assertIn("<build_type>ament_cmake</build_type>", package_xml)

    def test_refuses_non_empty_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "x2.urdf"
            source.write_text('<robot name="x2"><link name="base_link"/></robot>', encoding="utf-8")
            output = root / "out"
            output.mkdir()
            (output / "owned.txt").write_text("keep", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "refusing to overwrite"):
                MODULE.prepare(source, output)


if __name__ == "__main__":
    unittest.main()
