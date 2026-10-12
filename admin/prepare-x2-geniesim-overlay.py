#!/usr/bin/env python3
"""Create an isolated, auditable Genie Sim model overlay from the X2 URDF.

The vendor input is never modified.  Only the minimum importer compatibility
changes are applied to the derived URDF: a non-colliding root visual prevents
the Isaac 6 ghost-root path, and tiny inertials are added to massless sensor
frames that have neither inertial nor collision geometry.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def indent_xml(root: ET.Element) -> None:
    ET.indent(root, space="  ")


def prepare(source_urdf: Path, output_root: Path) -> dict:
    if output_root.exists() and any(output_root.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {output_root}")

    tree = ET.parse(source_urdf)
    robot = tree.getroot()
    if robot.tag != "robot":
        raise ValueError("source is not a URDF robot")

    links = {link.attrib["name"]: link for link in robot.findall("link")}
    children = {joint.find("child").attrib["link"] for joint in robot.findall("joint")}
    roots = sorted(set(links) - children)
    if roots != ["base_link"]:
        raise ValueError(f"expected X2 root base_link, got {roots}")

    root = links["base_link"]
    ghost_root_fixed = False
    if all(root.find(tag) is None for tag in ("visual", "collision", "inertial")):
        visual = ET.SubElement(root, "visual", {"name": "geniesim_ghost_root_guard"})
        ET.SubElement(visual, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "sphere", {"radius": "0.001"})
        ghost_root_fixed = True

    # This is a simulation-only overlay.  Isaac 6 authors base_link as a
    # rigid body even when it is a pure URDF frame; without an inertial the
    # resulting PhysicsMassAPI has mass=0 and is not valid for a future
    # floating-base run.  The 1 g placeholder is negligible next to the
    # vendor pelvis mass and is declared explicitly in the manifest.  KDL
    # consumers should continue using the untouched vendor URDF.
    simulation_root_inertial_added = False
    if root.find("inertial") is None:
        inertial = ET.SubElement(root, "inertial")
        ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(inertial, "mass", {"value": "0.001"})
        ET.SubElement(
            inertial,
            "inertia",
            {"ixx": "1e-6", "iyy": "1e-6", "izz": "1e-6", "ixy": "0", "ixz": "0", "iyz": "0"},
        )
        simulation_root_inertial_added = True

    inertial_links: list[str] = []
    for name, link in links.items():
        if name == "base_link":
            continue
        if link.find("inertial") is None and link.find("collision") is None:
            inertial = ET.SubElement(link, "inertial")
            ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
            ET.SubElement(inertial, "mass", {"value": "0.001"})
            ET.SubElement(
                inertial,
                "inertia",
                {
                    "ixx": "1e-6",
                    "iyy": "1e-6",
                    "izz": "1e-6",
                    "ixy": "0",
                    "ixz": "0",
                    "iyz": "0",
                },
            )
            inertial_links.append(name)

    source_mesh_dir = source_urdf.parent / "meshes"
    mesh_names: set[str] = set()
    for mesh in robot.findall(".//mesh"):
        source_name = Path(mesh.attrib["filename"]).name
        source_mesh = source_mesh_dir / source_name
        if not source_mesh.is_file():
            raise FileNotFoundError(f"missing mesh: {source_mesh}")
        mesh_names.add(source_name)
        mesh.attrib["filename"] = f"package://genie_sim_x2_model/meshes/{source_name}"

    package = output_root / "src" / "genie_sim_x2_model"
    urdf_dir = package / "urdf"
    mesh_dir = package / "meshes"
    mesh_dir.mkdir(parents=True)
    urdf_dir.mkdir(parents=True)

    for name in sorted(mesh_names):
        shutil.copy2(source_mesh_dir / name, mesh_dir / name)

    output_urdf = urdf_dir / "x2_ultra_geniesim.urdf"
    indent_xml(robot)
    tree.write(output_urdf, encoding="utf-8", xml_declaration=True)

    package_xml = """<?xml version=\"1.0\"?>
<package format=\"3\">
  <name>genie_sim_x2_model</name>
  <version>0.1.0</version>
  <description>Derived X2 model overlay for internal Genie Sim evaluation.</description>
  <maintainer email=\"noreply@roboease.invalid\">RoboEase simulation</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""
    cmake = """cmake_minimum_required(VERSION 3.8)
project(genie_sim_x2_model)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes DESTINATION share/${PROJECT_NAME})
ament_package()
"""
    (package / "package.xml").write_text(package_xml, encoding="utf-8", newline="\n")
    (package / "CMakeLists.txt").write_text(cmake, encoding="utf-8", newline="\n")

    manifest = {
        "schema_version": "sceneforge/x2-geniesim-overlay/v1",
        "source_urdf": str(source_urdf),
        "source_urdf_sha256": sha256(source_urdf),
        "derived_urdf": str(output_urdf.relative_to(output_root)),
        "derived_urdf_sha256": sha256(output_urdf),
        "robot_name": robot.attrib.get("name"),
        "link_count": len(links),
        "joint_count": len(robot.findall("joint")),
        "mesh_count": len(mesh_names),
        "ghost_root_guard_added": ghost_root_fixed,
        "simulation_root_inertial_added": simulation_root_inertial_added,
        "simulation_root_mass_kg": 0.001 if simulation_root_inertial_added else None,
        "placeholder_inertial_links": sorted(inertial_links),
        "vendor_non_diagonal_inertia_preserved": True,
        "vendor_urdf_remains_authoritative_for_kdl": True,
        "usage_scope": "internal_evaluation_only_pending_vendor_license_review",
        "controller_included": False,
        "real_robot_commands": False,
    }
    manifest_path = output_root / "x2-overlay-manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-urdf", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        result = prepare(args.source_urdf.resolve(), args.output.resolve())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
