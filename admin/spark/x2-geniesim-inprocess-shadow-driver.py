#!/usr/bin/env python3
"""Genie Sim Isaac/PhysX driver with a read-only X2 post-step policy hook."""

from __future__ import annotations

import gc
import json
import os
import sys
import time
from pathlib import Path


engine_python = os.environ["GENIESIM_ENGINE_PYTHON_DIR"]
sys.path.insert(0, engine_python)

from common.early_params import _early_param  # noqa: E402
from kit.bootstrap import bootstrap_simulation_app  # noqa: E402


simulation_app, _HEADLESS = bootstrap_simulation_app(
    physics_engine=_early_param(sys.argv[1:], "physics_engine", "isaac_physx"),
    physics_solver=_early_param(sys.argv[1:], "physics_solver", "mujoco-warp"),
    render_mode=_early_param(sys.argv[1:], "render_mode", "raster"),
)

from common.loop import _parse_args  # noqa: E402
from common.session import EngineSession, SimpleLogger  # noqa: E402
from sceneforge.x2_inprocess_shadow import X2InProcessShadow  # noqa: E402


def run(params: dict) -> int:
    logger = SimpleLogger("genie_sim_engine")
    physics_engine = params.get("physics_engine", "isaac_physx").strip().lower()
    session = EngineSession(
        node_name="genie_sim_engine",
        params=params,
        simulation_app=simulation_app,
        physics_engine=physics_engine,
        logger=logger,
    )
    logger.info(
        f"Isaac Sim physics engine ready ({session.physics_hz:.0f} Hz physics, "
        f"{session.render_hz:.0f} Hz render target, scene={session.scene_usda}, "
        f"render={'headless' if _HEADLESS else 'decoupled'}, physics_engine={physics_engine})"
    )
    session.startup(_HEADLESS)
    shadow = X2InProcessShadow(
        session.sim,
        model=os.environ["X2_SHADOW_MODEL"],
        receipt=os.environ["X2_SHADOW_RECEIPT"],
        stop_request=os.environ["ROBOEASE_PHYSX_CONTROLLED_STOP_FILE"],
        policy_rate_hz=float(os.environ.get("X2_SHADOW_POLICY_HZ", "50")),
        sim_duration_s=float(os.environ.get("X2_SHADOW_SIM_DURATION_S", "10")),
        minimum_inferences=int(os.environ.get("X2_SHADOW_MINIMUM_INFERENCES", "400")),
    )
    session.post_step_hooks.append(shadow.step)
    gc_was_enabled = gc.isenabled()
    gc.collect()
    if gc_was_enabled:
        gc.disable()
    try:
        session.run(render_hook=lambda *_: (0.0, 0.0, False))
    finally:
        if not shadow.completed:
            shadow.abort("engine loop ended before the bounded shadow completed")
        if gc_was_enabled:
            gc.enable()
            gc.collect()
    return 0 if shadow.status == "PASSED" else 2


def main() -> int:
    params = _parse_args(sys.argv[1:], prefix="genie_sim_engine")
    try:
        return run(params)
    except Exception as exc:
        print(f"[genie_sim_engine] fatal: {exc}", file=sys.stderr, flush=True)
        import traceback

        traceback.print_exc()
        return 1
    finally:
        try:
            simulation_app.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
