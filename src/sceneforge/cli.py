"""SceneForge phase-0 command line interface."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .canonical import fingerprint, load_json
from .models import AssetRecord, BuildingSpec, SceneSpec
from .layout import BuildingLayout, plan_hospital
from .usd_export import export_scene_bundle
from .preview import write_layout_svg


MODELS = {
    "asset": AssetRecord,
    "building": BuildingSpec,
    "layout": BuildingLayout,
    "scene": SceneSpec,
}

SCHEMA_FILENAMES = {
    "asset": "asset-record-v1.schema.json",
    "building": "building-spec-v1.schema.json",
    "layout": "building-layout-v1.schema.json",
    "scene": "scene-spec-v1.schema.json",
}


def validate_spec(kind: str, path: Path) -> None:
    model = MODELS[kind]
    validated = model.model_validate(load_json(path))
    print(json.dumps(validated.model_dump(mode="json"), ensure_ascii=False, indent=2))


def emit_schemas(output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    for kind, model in MODELS.items():
        destination = output / SCHEMA_FILENAMES[kind]
        destination.write_text(
            json.dumps(model.model_json_schema(), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(destination)


def derive_hospital_example(
    building_template_path: Path,
    scene_template_path: Path,
    seed: int,
    output: Path,
) -> None:
    building_template = BuildingSpec.model_validate(load_json(building_template_path))
    scene_template = SceneSpec.model_validate(load_json(scene_template_path))
    building = building_template.model_copy(
        update={"building_id": f"hospital_outpatient_{seed}", "seed": seed}
    )
    scene = scene_template.model_copy(
        update={
            "scene_id": f"hospital_service_{seed}",
            "building_id": building.building_id,
            "seed": seed,
        }
    )
    output.mkdir(parents=True, exist_ok=True)
    destinations = {
        output / f"building-seed-{seed}.json": building,
        output / f"scene-seed-{seed}.json": scene,
    }
    for destination, model in destinations.items():
        if destination.exists():
            raise FileExistsError(f"refusing to overwrite example: {destination}")
        destination.write_text(
            json.dumps(model.model_dump(mode="json"), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
            newline="\n",
        )
        print(destination)


def build_hospital(building_path: Path, scene_path: Path, output: Path) -> None:
    building = BuildingSpec.model_validate(load_json(building_path))
    scene = SceneSpec.model_validate(load_json(scene_path))
    layout = plan_hospital(building, scene)
    print(export_scene_bundle(output, building, scene, layout))


def render_hospital_preview(building_path: Path, scene_path: Path, output: Path) -> None:
    building = BuildingSpec.model_validate(load_json(building_path))
    scene = SceneSpec.model_validate(load_json(scene_path))
    layout = plan_hospital(building, scene)
    write_layout_svg(output, layout, building)
    print(output)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="sceneforge")
    subparsers = parser.add_subparsers(dest="command", required=True)

    validate = subparsers.add_parser("validate-spec")
    validate.add_argument("--kind", choices=sorted(MODELS), required=True)
    validate.add_argument("path", type=Path)

    hash_parser = subparsers.add_parser("fingerprint")
    hash_parser.add_argument("path", type=Path)

    schemas = subparsers.add_parser("emit-schemas")
    schemas.add_argument("--out", type=Path, required=True)

    derive = subparsers.add_parser("derive-hospital-example")
    derive.add_argument("--building-template", type=Path, required=True)
    derive.add_argument("--scene-template", type=Path, required=True)
    derive.add_argument("--seed", type=int, required=True)
    derive.add_argument("--out", type=Path, required=True)

    build = subparsers.add_parser("build-hospital")
    build.add_argument("--building", type=Path, required=True)
    build.add_argument("--scene", type=Path, required=True)
    build.add_argument("--out", type=Path, required=True)

    preview = subparsers.add_parser("render-hospital-preview")
    preview.add_argument("--building", type=Path, required=True)
    preview.add_argument("--scene", type=Path, required=True)
    preview.add_argument("--out", type=Path, required=True)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.command == "validate-spec":
        validate_spec(args.kind, args.path)
    elif args.command == "fingerprint":
        print(fingerprint(load_json(args.path)))
    elif args.command == "emit-schemas":
        emit_schemas(args.out)
    elif args.command == "derive-hospital-example":
        derive_hospital_example(
            args.building_template, args.scene_template, args.seed, args.out
        )
    elif args.command == "build-hospital":
        build_hospital(args.building, args.scene, args.out)
    elif args.command == "render-hospital-preview":
        render_hospital_preview(args.building, args.scene, args.out)


if __name__ == "__main__":
    main()
