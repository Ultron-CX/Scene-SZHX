#!/usr/bin/env python3
"""Render a robot-free SceneForge building preview with Isaac Sim."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise FileExistsError(f"refusing to overwrite preview output: {args.output_dir}")
    args.output_dir.mkdir(parents=True)

    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "renderer": "RaytracedLighting",
            "width": args.width,
            "height": args.height,
        }
    )
    error: str | None = None
    try:
        import omni.replicator.core as rep
        import omni.usd
        from pxr import Gf, UsdLux

        context = omni.usd.get_context()
        if not context.open_stage(str(args.scene.resolve())):
            raise RuntimeError("omni.usd failed to open SceneForge stage")
        stage = context.get_stage()
        if stage is None or not stage.GetDefaultPrim():
            raise RuntimeError("opened stage has no default prim")

        dome = UsdLux.DomeLight.Define(stage, "/SceneForgePreview/DomeLight")
        dome.CreateIntensityAttr(850.0)
        dome.CreateColorAttr(Gf.Vec3f(0.82, 0.88, 1.0))
        key = UsdLux.DistantLight.Define(stage, "/SceneForgePreview/KeyLight")
        key.CreateIntensityAttr(1800.0)
        key.CreateAngleAttr(2.0)

        camera = rep.create.camera(
            position=(39.0, -25.0, 31.0),
            look_at=(16.0, 11.0, 0.8),
            focal_length=30.0,
        )
        render_product = rep.create.render_product(camera, (args.width, args.height))
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(output_dir=str(args.output_dir), rgb=True)
        writer.attach([render_product])
        # A static review render only needs two frames.  The first frame warms
        # up RTX shaders and the second is the deliverable.  Keeping this
        # bounded avoids turning a preview helper into a long-running job.
        for _ in range(2):
            rep.orchestrator.step(rt_subframes=4)
        writer.detach()
        for _ in range(3):
            app.update()
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    finally:
        app.close()

    images = sorted(args.output_dir.rglob("*.png"))
    receipt = {
        "schema_version": "sceneforge/isaac-scene-preview/v1",
        "status": "SUCCEEDED" if error is None and images else "FAILED",
        "error": error,
        "scene": str(args.scene.resolve()),
        "scene_sha256": sha256(args.scene),
        "resolution": [args.width, args.height],
        "camera_position": [39.0, -25.0, 31.0],
        "camera_target": [16.0, 11.0, 0.8],
        "images": [
            {
                "path": str(path.relative_to(args.output_dir)),
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
            for path in images
        ],
        "robot_loaded": False,
        "physics_advanced": False,
        "qualification": False,
        "real_robot_commands": False,
    }
    receipt_path = args.output_dir / "receipt.json"
    receipt_path.write_text(
        json.dumps(receipt, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    print(receipt_path)
    if receipt["status"] != "SUCCEEDED":
        sys.exit(1)


if __name__ == "__main__":
    main()
