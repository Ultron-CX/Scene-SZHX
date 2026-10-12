#!/usr/bin/env python3
"""Run straight, left turn and right turn in one Genie Sim PhysX lifecycle."""

from __future__ import annotations

import gc
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile


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
from sceneforge.x2_cold_start_combination import (  # noqa: E402
    SCENARIOS,
    X2ColdStartCombinationCoordinator,
    X2CombinationCaseCapture,
    X2CombinationCasePlan,
    X2ContinuationMotionProfile,
)
from sceneforge.x2_mujoco_motion import (  # noqa: E402
    X2MujocoMotionProfile,
    load_urdf_joint_limits,
    load_urdf_joint_velocity_limits,
    quaternion_rpy_wxyz,
)
from x2_physx_base_state_reader import PhysXBaseStateReader  # noqa: E402
from x2_physx_contact_reader import (  # noqa: E402
    PhysXFootContactReader,
    enable_physx_contact_reports,
)


def sha256_json(payload: object) -> str:
    encoded = json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def write_new_json(path: Path, payload: dict) -> None:
    if path.exists():
        raise FileExistsError(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(
                json.dumps(payload, indent=2, sort_keys=True, allow_nan=False)
                + "\n"
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, path)
        path.chmod(0o644)
    finally:
        if temporary.exists():
            temporary.unlink()


def make_run_artifacts_host_readable(run_root: Path) -> None:
    """Expose uid-1234 container artifacts to the host-side finalizer."""

    for path in sorted(
        run_root.rglob("*"), key=lambda item: len(item.parts), reverse=True
    ):
        try:
            path.chmod(0o755 if path.is_dir() else 0o644)
        except PermissionError:
            # The run root and host-authored files belong to the Spark login
            # user and are already readable; never escalate privileges here.
            continue


def load_expected_initial_state(scene_yaml: Path) -> tuple[dict, dict[str, float]]:
    import yaml

    payload = yaml.safe_load(scene_yaml.read_text(encoding="utf-8"))
    robot = payload.get("robot") if isinstance(payload, dict) else None
    if not isinstance(robot, dict):
        raise ValueError("scene YAML lacks robot block")
    base = robot.get("init_base_pose")
    joints = robot.get("init_joint_pos")
    if not isinstance(base, dict) or not isinstance(joints, dict):
        raise ValueError("scene YAML lacks authoritative initial pose")
    return base, {str(name): float(value) for name, value in joints.items()}


def reset_receipt(
    session,
    base_state: PhysXBaseStateReader,
    coordinator: X2ColdStartCombinationCoordinator,
    *,
    scene_yaml: Path,
    reset_id: str,
) -> dict:
    positions, velocities = session.sim.get_joint_states()
    q = tuple(float(value) for value in positions)
    dq = tuple(float(value) for value in velocities)
    pose_twist = base_state.read(0.0)
    if pose_twist is None:
        pose, twist = (), ()
    else:
        pose, twist = pose_twist
        pose = tuple(float(value) for value in pose)
        twist = tuple(float(value) for value in twist)
    expected_base, expected_joints = load_expected_initial_state(scene_yaml)
    q_by_name = dict(zip(session.sim.joint_names, q))
    missing_expected_joints = sorted(set(expected_joints) - set(q_by_name))
    joint_errors = {
        name: abs(q_by_name[name] - expected)
        for name, expected in expected_joints.items()
        if name in q_by_name
    }
    expected_xyz = (
        float(expected_base["x"]),
        float(expected_base["y"]),
        float(expected_base["z"]),
    )
    base_error = (
        max(abs(actual - expected) for actual, expected in zip(pose[:3], expected_xyz))
        if len(pose) == 7 else float("inf")
    )
    yaw_error = (
        abs(
            math.atan2(
                math.sin(quaternion_rpy_wxyz(pose[3:7])[2] - float(expected_base["theta"])),
                math.cos(quaternion_rpy_wxyz(pose[3:7])[2] - float(expected_base["theta"])),
            )
        )
        if len(pose) == 7 else float("inf")
    )
    joint_error = max(joint_errors.values(), default=float("inf"))
    finite = (
        len(q) == len(session.sim.joint_names) == 31
        and len(dq) == 31
        and len(pose) == 7
        and len(twist) == 6
        and all(math.isfinite(value) for value in (*q, *dq, *pose, *twist))
    )
    capture = coordinator.current
    stand_reset = (
        capture.session.last_source_sequence is None
        and capture.session.last_sim_time is None
        and capture.session.history == []
    )
    walk_reset = (
        capture.walk_session.last_source_sequence is None
        and capture.walk_session.last_sim_time is None
        and capture.walk_session.history == []
    )
    hopf = capture.walk_session.hopf.snapshot()
    controller_history_reset = stand_reset and walk_reset
    cpg_state_reset = not bool(hopf.walk_initialized)
    command_sequence_reset = (
        capture.physics_sequence == 0
        and capture.command_sequence == 0
        and capture.walk_sequence == 0
    )
    pose_within_tolerance = (
        finite
        and not missing_expected_joints
        and base_error <= float(os.environ.get("X2_RESET_BASE_TOLERANCE_M", "0.08"))
        and yaw_error <= float(os.environ.get("X2_RESET_YAW_TOLERANCE_RAD", "0.08"))
        and joint_error <= float(os.environ.get("X2_RESET_JOINT_TOLERANCE_RAD", "0.12"))
    )
    initial_state = {
        "joint_names": list(session.sim.joint_names),
        "joint_position": list(q),
        "joint_velocity": list(dq),
        "base_pose_wxyz": list(pose),
        "base_twist": list(twist),
    }
    checks = {
        "physics_reset_complete": True,
        "controller_history_reset": controller_history_reset,
        "cpg_state_reset": cpg_state_reset,
        "command_sequence_reset": command_sequence_reset,
        "initial_state_finite": finite,
        "initial_pose_within_tolerance": pose_within_tolerance,
    }
    return {
        "schema": "sceneforge/x2-physx-cold-start-reset-receipt/v1",
        "status": "PASSED" if all(checks.values()) else "FAILED",
        "reset_id": reset_id,
        **checks,
        "initial_state_sha256": sha256_json(initial_state),
        "initial_state": initial_state,
        "expected_source": str(scene_yaml),
        "maximum_base_position_error_m": base_error,
        "base_yaw_error_rad": yaw_error,
        "maximum_joint_position_error_rad": joint_error,
        "missing_expected_joint_names": missing_expected_joints,
        "physics_authority": "genie_sim_isaac_physx",
        "published_command_topics": [],
        "real_robot_commands": False,
    }


def profiles(run_root: Path) -> tuple[X2CombinationCasePlan, ...]:
    supported = float(os.environ.get("X2_SUPPORTED_STAND_S", "3.0"))
    ramp = float(os.environ.get("X2_BAND_RAMP_S", "5.0"))
    unsupported = float(os.environ.get("X2_UNSUPPORTED_STAND_S", "3.0"))
    intercase = float(os.environ.get("X2_INTERCASE_UNSUPPORTED_STAND_S", "2.0"))
    straight_walk = float(os.environ.get("X2_STRAIGHT_WALK_S", "8.0"))
    turn_walk = float(os.environ.get("X2_TURN_WALK_S", "8.0"))
    stop = float(os.environ.get("X2_STOP_S", "8.0"))
    straight_speed = float(os.environ.get("X2_STRAIGHT_X_MPS", "0.3"))
    turn_speed = float(os.environ.get("X2_TURN_X_MPS", "0.2"))
    turn_yaw = float(os.environ.get("X2_TURN_YAW_RPS", "0.25"))
    case_profiles = (
        X2MujocoMotionProfile(
            supported_stand_s=supported,
            band_ramp_s=ramp,
            unsupported_stand_s=unsupported,
            walk_s=straight_walk,
            stop_s=stop,
            command_x_mps=straight_speed,
        ),
        X2ContinuationMotionProfile(
            unsupported_stand_s=intercase,
            walk_s=turn_walk,
            stop_s=stop,
            command_x_mps=turn_speed,
            command_yaw_rps=turn_yaw,
        ),
        X2ContinuationMotionProfile(
            unsupported_stand_s=intercase,
            walk_s=turn_walk,
            stop_s=stop,
            command_x_mps=turn_speed,
            command_yaw_rps=-turn_yaw,
        ),
    )
    return tuple(
        X2CombinationCasePlan(
            scenario=scenario,
            profile=profile,
            receipt=run_root / "cases" / scenario / "physx-result.json",
            trace=run_root / "cases" / scenario / "state-command-trace.json",
            performs_suspension_transition=index == 0,
        )
        for index, (scenario, profile) in enumerate(zip(SCENARIOS, case_profiles))
    )


def run(params: dict) -> int:
    logger = SimpleLogger("genie_sim_engine")
    physics_engine = params.get("physics_engine", "isaac_physx").strip().lower()
    if physics_engine != "isaac_physx":
        raise ValueError("X2 cold-start combination is locked to isaac_physx")
    run_root = Path(os.environ["X2_COMBINATION_RUN_ROOT"]).resolve()
    run_root.mkdir(parents=True, exist_ok=True)
    combination_result_path = run_root / "combination-runtime-result.json"
    reset_path = run_root / "reset-receipt.json"
    session = EngineSession(
        node_name="genie_sim_engine",
        params=params,
        simulation_app=simulation_app,
        physics_engine=physics_engine,
        logger=logger,
    )
    logger.info(
        f"Isaac Sim physics engine ready ({session.physics_hz:.0f} Hz physics, "
        "single-lifecycle X2 straight+turn_left+turn_right)"
    )
    dynamics_path = Path(os.environ["X2_PHYSX_DYNAMICS_RECEIPT"])
    alignment = json.loads(dynamics_path.read_text(encoding="utf-8"))
    if alignment.get("pre_world_reset") is not True:
        raise RuntimeError("PhysX dynamics alignment was not authored before reset")
    contact_report_paths = enable_physx_contact_reports(
        session.sim.stage, session.sim.body_paths
    )
    logger.info(
        f"PHYSX_CONTACT_REPORTS_ENABLED body_count={len(contact_report_paths)} threshold_n=0"
    )
    session.startup(_HEADLESS)
    base_state = PhysXBaseStateReader(session.sim.stage, session.sim.body_paths)
    contacts = PhysXFootContactReader(
        session.sim.stage,
        session.sim.body_paths,
        physics_dt_s=1.0 / session.physics_hz,
    )
    joint_limit_path = os.environ["X2_JOINT_LIMIT_URDF"]
    joint_limits = load_urdf_joint_limits(
        joint_limit_path, tuple(session.sim.joint_names)
    )
    joint_velocity_limits = load_urdf_joint_velocity_limits(
        joint_limit_path, tuple(session.sim.joint_names)
    )
    plans = profiles(run_root)
    for plan in plans:
        plan.receipt.parent.mkdir(parents=True, exist_ok=False)

    def capture_factory(plan: X2CombinationCasePlan):
        return X2CombinationCaseCapture(
            session.sim,
            stand_model=os.environ["X2_STAND_MODEL"],
            walk_model=os.environ["X2_WALK_MODEL"],
            receipt=plan.receipt,
            trace=plan.trace,
            stop_request=os.environ["ROBOEASE_PHYSX_CONTROLLED_STOP_FILE"],
            band_command=os.environ["ROBOEASE_PHYSX_ELASTIC_BAND_COMMAND_FILE"],
            band_status=os.environ["ROBOEASE_PHYSX_ELASTIC_BAND_STATUS_FILE"],
            state_reader=base_state.read,
            contact_reader=contacts,
            profile=plan.profile,
            joint_limits=joint_limits,
            joint_velocity_limits=joint_velocity_limits,
            joint_limit_source=joint_limit_path,
            physics_dt_s=1.0 / session.physics_hz,
            freeze_probe_wall_s=float(
                os.environ.get("X2_SIM_TIME_FREEZE_PROBE_WALL_S", "2.0")
            ),
            base_state_source=base_state.source,
        )

    coordinator = X2ColdStartCombinationCoordinator(
        plans,
        capture_factory,
        freeze_wall_s=float(
            os.environ.get("X2_SIM_TIME_FREEZE_PROBE_WALL_S", "2.0")
        ),
    )
    reset = reset_receipt(
        session,
        base_state,
        coordinator,
        scene_yaml=Path(os.environ["X2_SCENE_YAML"]),
        reset_id=os.environ["X2_RESET_ID"],
    )
    write_new_json(reset_path, reset)
    if reset["status"] != "PASSED":
        coordinator.current.request_authoritative_stop()
        coordinator.stop_requested = True
        coordinator.status = "FAILED"
        coordinator.completed = True
    session.post_step_hooks.append(coordinator.step)
    gc_was_enabled = gc.isenabled()
    gc.collect()
    if gc_was_enabled:
        gc.disable()
    try:
        session.run(render_hook=lambda *_: (0.0, 0.0, False))
    finally:
        if not coordinator.completed:
            coordinator.status = "FAILED"
            coordinator.completed = True
            coordinator._request_stop_once()
        if gc_was_enabled:
            gc.enable()
            gc.collect()
    result = coordinator.result()
    result["reset_receipt"] = str(reset_path)
    result["reset_status"] = reset["status"]
    result["dynamics_alignment_receipt"] = str(dynamics_path)
    result["status"] = (
        "PASSED"
        if result["status"] == "PASSED" and reset["status"] == "PASSED"
        else "FAILED"
    )
    write_new_json(combination_result_path, result)
    return 0 if result["status"] == "PASSED" else 2


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
        run_root = os.environ.get("X2_COMBINATION_RUN_ROOT")
        if run_root:
            make_run_artifacts_host_readable(Path(run_root))


if __name__ == "__main__":
    raise SystemExit(main())
