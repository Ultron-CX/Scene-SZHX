#!/usr/bin/env python3
"""Run the bounded full-joint STAND_DEFAULT capture inside Genie Sim PhysX."""

from __future__ import annotations

import gc
import os
import sys


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
from sceneforge.x2_fixed_base_stand import X2FixedBaseStandCapture  # noqa: E402


def run(params: dict) -> int:
    logger = SimpleLogger("genie_sim_engine")
    physics_engine = params.get("physics_engine", "isaac_physx").strip().lower()
    if physics_engine != "isaac_physx":
        raise ValueError("fixed-base STAND_DEFAULT capture is locked to isaac_physx")
    session = EngineSession(
        node_name="genie_sim_engine",
        params=params,
        simulation_app=simulation_app,
        physics_engine=physics_engine,
        logger=logger,
    )
    logger.info(
        f"Isaac Sim physics engine ready ({session.physics_hz:.0f} Hz physics, "
        f"scene={session.scene_usda}, fixed-base STAND_DEFAULT capture)"
    )
    session.startup(_HEADLESS)
    capture = X2FixedBaseStandCapture(
        session.sim,
        model=os.environ["X2_STAND_MODEL"],
        receipt=os.environ["X2_STAND_RECEIPT"],
        trace=os.environ["X2_STAND_TRACE"],
        stop_request=os.environ["ROBOEASE_PHYSX_CONTROLLED_STOP_FILE"],
    )
    session.post_step_hooks.append(capture.step)
    gc_was_enabled = gc.isenabled()
    gc.collect()
    if gc_was_enabled:
        gc.disable()
    try:
        session.run(render_hook=lambda *_: (0.0, 0.0, False))
    finally:
        if not capture.completed:
            capture.abort("engine loop ended before the bounded STAND_DEFAULT capture completed")
        if gc_was_enabled:
            gc.enable()
            gc.collect()
    return 0 if capture.status == "PASSED" else 2


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
