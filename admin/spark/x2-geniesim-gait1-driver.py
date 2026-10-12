#!/usr/bin/env python3
"""Run the independent X2 gait1 development A/B with PhysX authority."""

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
from sceneforge.x2_mujoco_motion import (  # noqa: E402
    X2MujocoMotionProfile,
    load_urdf_joint_limits,
    load_urdf_joint_velocity_limits,
)
from sceneforge.x2_physx_gait1 import X2PhysxGait1Capture  # noqa: E402
from x2_physx_base_state_reader import PhysXBaseStateReader  # noqa: E402
from x2_physx_contact_reader import (  # noqa: E402
    PhysXFootContactReader,
    enable_physx_contact_reports,
)


def run(params: dict) -> int:
    logger = SimpleLogger("genie_sim_engine")
    physics_engine = params.get("physics_engine", "isaac_physx").strip().lower()
    if physics_engine != "isaac_physx":
        raise ValueError("X2 gait1 A/B is locked to isaac_physx")
    session = EngineSession(
        node_name="genie_sim_engine",
        params=params,
        simulation_app=simulation_app,
        physics_engine=physics_engine,
        logger=logger,
    )
    logger.info(
        f"Isaac Sim physics engine ready ({session.physics_hz:.0f} Hz physics, "
        f"scene={session.scene_usda}, floating gait1 development A/B)"
    )
    with open(os.environ["X2_PHYSX_DYNAMICS_RECEIPT"], "r", encoding="utf-8") as stream:
        import json

        alignment = json.load(stream)
    if alignment.get("pre_world_reset") is not True:
        raise RuntimeError("PhysX dynamics alignment was not authored before world reset")
    logger.info(
        "PHYSX_DYNAMICS_ALIGNMENT "
        f"scope={alignment['scope']} static={alignment['static_friction']:.3f} "
        f"dynamic={alignment['dynamic_friction']:.3f}"
    )
    contact_report_paths = enable_physx_contact_reports(
        session.sim.stage, session.sim.body_paths
    )
    logger.info(
        f"PHYSX_CONTACT_REPORTS_ENABLED body_count={len(contact_report_paths)} threshold_n=0"
    )
    session.startup(_HEADLESS)
    base_state = PhysXBaseStateReader(session.sim.stage, session.sim.body_paths)
    logger.info(
        "PHYSX_BASE_STATE_READER "
        f"source={base_state.source} body={base_state.path} read_only=true"
    )
    contacts = PhysXFootContactReader(
        session.sim.stage,
        session.sim.body_paths,
        physics_dt_s=1.0 / session.physics_hz,
    )
    profile = X2MujocoMotionProfile(
        supported_stand_s=float(os.environ.get("X2_SUPPORTED_STAND_S", "3.0")),
        band_ramp_s=float(os.environ.get("X2_BAND_RAMP_S", "5.0")),
        unsupported_stand_s=float(os.environ.get("X2_UNSUPPORTED_STAND_S", "3.0")),
        walk_s=float(os.environ.get("X2_WALK_S", "8.0")),
        stop_s=float(os.environ.get("X2_STOP_S", "8.0")),
        command_x_mps=float(os.environ.get("X2_COMMAND_X_MPS", "0.3")),
        command_y_mps=float(os.environ.get("X2_COMMAND_Y_MPS", "0.0")),
        command_yaw_rps=float(os.environ.get("X2_COMMAND_YAW_RPS", "0.0")),
    )
    joint_limit_path = os.environ["X2_JOINT_LIMIT_URDF"]
    joint_limits = load_urdf_joint_limits(
        joint_limit_path, tuple(session.sim.joint_names)
    )
    joint_velocity_limits = load_urdf_joint_velocity_limits(
        joint_limit_path, tuple(session.sim.joint_names)
    )
    capture = X2PhysxGait1Capture(
        session.sim,
        stand_model=os.environ["X2_STAND_MODEL"],
        walk_model=os.environ["X2_WALK_MODEL"],
        receipt=os.environ["X2_GAIT1_RECEIPT"],
        trace=os.environ["X2_GAIT1_TRACE"],
        stop_request=os.environ["ROBOEASE_PHYSX_CONTROLLED_STOP_FILE"],
        band_command=os.environ["ROBOEASE_PHYSX_ELASTIC_BAND_COMMAND_FILE"],
        band_status=os.environ["ROBOEASE_PHYSX_ELASTIC_BAND_STATUS_FILE"],
        state_reader=base_state.read,
        contact_reader=contacts,
        profile=profile,
        joint_limits=joint_limits,
        joint_velocity_limits=joint_velocity_limits,
        joint_limit_source=joint_limit_path,
        physics_dt_s=1.0 / session.physics_hz,
        freeze_probe_wall_s=float(
            os.environ.get("X2_SIM_TIME_FREEZE_PROBE_WALL_S", "2.0")
        ),
        base_state_source=base_state.source,
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
            capture._finish("FAILED", "engine loop ended before gait1 profile completed")
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
