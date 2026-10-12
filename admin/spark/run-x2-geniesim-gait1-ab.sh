#!/usr/bin/env bash
set -euo pipefail

root=${SCENEFORGE_X2_ROOT:-/opt/sceneforge/runtime}
domain=${ROS_DOMAIN_ID:-78}
physics_hz=${X2_PHYSICS_HZ:-500}
supported_s=${X2_SUPPORTED_STAND_S:-3.0}
ramp_s=${X2_BAND_RAMP_S:-5.0}
unsupported_s=${X2_UNSUPPORTED_STAND_S:-3.0}
walk_s=${X2_WALK_S:-8.0}
stop_s=${X2_STOP_S:-8.0}
command_x=${X2_COMMAND_X_MPS:-0.3}
command_y=${X2_COMMAND_Y_MPS:-0.0}
command_yaw=${X2_COMMAND_YAW_RPS:-0.0}
motion_case=${X2_MOTION_CASE:-straight}
ground_static_friction=${X2_PHYSX_GROUND_STATIC_FRICTION:-1.0}
ground_dynamic_friction=${X2_PHYSX_GROUND_DYNAMIC_FRICTION:-1.0}
foot_collision_mode=${X2_PHYSX_FOOT_COLLISION_MODE:-imported_mesh}
engine_image=${GENIESIM_IMAGE:-roboease/geniesim-isaac6-jazzy-arm64:20260818}
manifest=${X2_GENIESIM_MANIFEST:-/project/assets/x2-floating-dev-v1/manifest.json}
build=${X2_GENIESIM_BUILD_ROOT:-/project/geniesim-build-effort}
engine_python=$build/install/genie_sim_engine/lib/genie_sim_engine
physics=${X2_PHYSICS_PARAMS:-/project/assets/x2-floating-dev-v1/physics_params_x2.yaml}
overlay="$root/assets/x2-overlay-v3"
stand_model=${X2_STAND_MODEL:-/project/third_party/x2-release-linux/install/x1_rl_control/share/x1_rl_control/policy/cpgtelecon_v3_fix.onnx}
walk_model=${X2_WALK_MODEL:-/project/third_party/x2-release-linux/install/x1_rl_control/share/x1_rl_control/policy/cpgwalkrun_v25_v2.onnx}
joint_limit_urdf=${X2_JOINT_LIMIT_URDF:-/project/third_party/x2-release-linux/install/x1_sim/share/x1_sim/model/x2_ultra.urdf}
ort_python=/project/runtime/x2-shadow-ort
stamp=$(date -u +%Y%m%dT%H%M%SZ)
if [[ ! "$motion_case" =~ ^[a-z0-9_-]+$ ]]; then
  echo "invalid X2_MOTION_CASE" >&2
  exit 2
fi
run_id="x2-geniesim-${motion_case}-${stamp}"
run_dir="$root/runs/$run_id"
evidence_dir="$root/evidence/$run_id"
container="x2-geniesim-gait1-${stamp,,}"
network="x2-geniesim-gait1-${stamp,,}"

if [[ "$manifest" != "/project/assets/x2-floating-dev-v1/manifest.json" ]]; then
  echo "gait1 A/B requires the frozen floating-base manifest" >&2
  exit 2
fi
if [[ ! -f "$root/runtime/x2-shadow-ort/onnxruntime/__init__.py" ]]; then
  echo "missing workspace-local ONNX Runtime at $root/runtime/x2-shadow-ort" >&2
  exit 2
fi
if docker ps --format '{{.Names}}' | grep -Eq 'x2-geniesim|genie.*sim.*engine|mujoco|dimos'; then
  echo "another physics authority appears to be running" >&2
  exit 2
fi
if [[ -e "$run_dir" || -e "$evidence_dir" ]]; then
  echo "refusing to overwrite run/evidence" >&2
  exit 2
fi
mkdir -p "$run_dir" "$root/evidence"
if command -v setfacl >/dev/null 2>&1; then
  setfacl -m u:1234:rwx "$run_dir"
else
  chmod 0777 "$run_dir"
fi

python3 - "$run_dir/elastic-band-command.json" <<'PY'
import json, os, pathlib, sys
path = pathlib.Path(sys.argv[1])
payload = {
    "schema": "sceneforge/physx-elastic-band-command/v1",
    "sequence": 1,
    "state": "ENGAGED",
    "support_ratio": 1.0,
    "requested_by": "x2_physx_gait1_ab",
    "real_robot_commands": False,
}
temporary = path.with_name(path.name + ".tmp")
with temporary.open("w", encoding="utf-8") as stream:
    stream.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    stream.flush(); os.fsync(stream.fileno())
os.replace(temporary, path)
PY

cleanup() {
  set +e
  if docker inspect "$container" >/dev/null 2>&1; then
    docker logs "$container" >"$run_dir/engine.log" 2>&1
    docker stop --time 20 "$container" >/dev/null 2>&1
    docker rm "$container" >/dev/null 2>&1
  fi
  docker network rm "$network" >/dev/null 2>&1
}
trap cleanup EXIT INT TERM

docker network create --internal "$network" >/dev/null
docker run -d --name "$container" --network "$network" --gpus all --entrypoint bash \
  --ipc host \
  -e ROS_DOMAIN_ID="$domain" -e ROS_LOCALHOST_ONLY=0 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  -e ROS_DISTRO=jazzy -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.core/jazzy/lib:/opt/ros/jazzy/lib/aarch64-linux-gnu:/opt/ros/jazzy/lib:/isaac-sim/kit/lib \
  -e PYTHONUNBUFFERED=1 -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
  -e ISAACSIM_VIEWPORT_UPDATES=0 -e REAL_ROBOT_COMMANDS=false \
  -e ROBOEASE_GENIE_PUBLISH_RENDER_BODY_TF=false \
  -e ROBOEASE_PHYSX_ELASTIC_BAND_ENABLED=true \
  -e ROBOEASE_PHYSX_CONTACT_REPORTS_ENABLED=true \
  -e ROBOEASE_PHYSX_ELASTIC_BAND_BODY=torso_link \
  -e ROBOEASE_PHYSX_ELASTIC_BAND_ANCHOR=0,0,2.5 \
  -e ROBOEASE_PHYSX_ELASTIC_BAND_STIFFNESS=200 \
  -e ROBOEASE_PHYSX_ELASTIC_BAND_DAMPING=100 \
  -e ROBOEASE_PHYSX_ELASTIC_BAND_REST_LENGTH=0 \
  -e ROBOEASE_PHYSX_ELASTIC_BAND_MAX_FORCE=800 \
  -e ROBOEASE_PHYSX_ELASTIC_BAND_COMMAND_FILE="/project/runs/$run_id/elastic-band-command.json" \
  -e ROBOEASE_PHYSX_ELASTIC_BAND_STATUS_FILE="/project/runs/$run_id/elastic-band-status.json" \
  -e ROBOEASE_PHYSX_ELASTIC_BAND_TELEMETRY_FILE="/project/runs/$run_id/elastic-band-telemetry.jsonl" \
  -e ROBOEASE_PHYSX_CONTROLLED_STOP_FILE="/project/runs/$run_id/physics-stop-request.json" \
  -e GENIESIM_ENGINE_PYTHON_DIR="$engine_python" \
  -e X2_STAND_MODEL="$stand_model" \
  -e X2_WALK_MODEL="$walk_model" \
  -e X2_GAIT1_RECEIPT="/project/runs/$run_id/gait1-receipt.json" \
  -e X2_GAIT1_TRACE="/project/runs/$run_id/gait1-trace.json" \
  -e X2_SUPPORTED_STAND_S="$supported_s" \
  -e X2_BAND_RAMP_S="$ramp_s" \
  -e X2_UNSUPPORTED_STAND_S="$unsupported_s" \
  -e X2_WALK_S="$walk_s" \
  -e X2_STOP_S="$stop_s" \
  -e X2_COMMAND_X_MPS="$command_x" \
  -e X2_COMMAND_Y_MPS="$command_y" \
  -e X2_COMMAND_YAW_RPS="$command_yaw" \
  -e X2_SIM_TIME_FREEZE_PROBE_WALL_S=2.0 \
  -e X2_JOINT_LIMIT_URDF="$joint_limit_urdf" \
  -e X2_PHYSX_GROUND_STATIC_FRICTION="$ground_static_friction" \
  -e X2_PHYSX_GROUND_DYNAMIC_FRICTION="$ground_dynamic_friction" \
  -e X2_PHYSX_FOOT_COLLISION_MODE="$foot_collision_mode" \
  -e X2_PHYSX_DYNAMICS_RECEIPT="/project/runs/$run_id/dynamics-alignment.json" \
  -v "$root:/project" -v "$overlay:/workspace/x2-overlay-v3:ro" \
  -v "$root/isaac-cache/kit:/isaac-sim/kit/cache" \
  "$engine_image" -lc "
    source /opt/ros/jazzy/setup.bash
    source '$build/install/setup.bash'
    export PYTHONPATH='$ort_python:/project/src:$engine_python':\${PYTHONPATH:-}
    exec /isaac-sim/python.sh /project/admin/spark/x2-geniesim-gait1-driver.py --ros-args \
      -p stage_manifest:='$manifest' -p physics_engine:=isaac_physx \
      -p physics_hz:=$physics_hz -p render_hz:=10 -p realtime_factor:=1.0 \
      -p fake_slam:=false -p physics_params_file:='$physics' \
      -p init_joint_pos_json:='{}'
  " >/dev/null

ready=0
for _ in $(seq 1 360); do
  docker logs "$container" >"$run_dir/engine.log" 2>&1 || true
  if grep -q "floating gait1 development A/B" "$run_dir/engine.log"; then ready=1; break; fi
  if ! docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null | grep -q true; then break; fi
  sleep 0.5
done
if [[ $ready -ne 1 ]]; then
  echo "Genie Sim did not become ready" >&2
  exit 3
fi

complete=0
for _ in $(seq 1 12000); do
  docker logs "$container" >"$run_dir/engine.log" 2>&1 || true
  if [[ -s "$run_dir/gait1-receipt.json" ]] \
    && grep -q "CONTROLLED_PHYSICS_STOP_COMPLETE" "$run_dir/engine.log"; then
    complete=1
    break
  fi
  if ! docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null | grep -q true; then break; fi
  sleep 0.1
done
if [[ $complete -ne 1 ]]; then
  echo "floating gait1 capture or controlled stop did not complete" >&2
  exit 4
fi
if docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null | grep -q true; then
  docker kill "$container" >/dev/null
fi
docker logs "$container" >"$run_dir/engine.log" 2>&1 || true

python3 - "$run_dir" "$evidence_dir" "$domain" "$engine_image" "$physics_hz" "$manifest" \
  "$root/src/sceneforge/x2_control.py" "$root/src/sceneforge/x2_session.py" \
  "$root/src/sceneforge/x2_stand.py" "$root/src/sceneforge/x2_fixed_base_stand.py" \
  "$root/src/sceneforge/x2_floating_stand.py" "$root/src/sceneforge/x2_mujoco_motion.py" \
  "$root/src/sceneforge/x2_locomotion_metrics.py" \
  "$root/src/sceneforge/x2_physx_gait1.py" \
  "$root/admin/spark/x2-geniesim-gait1-driver.py" \
  "$root/admin/spark/x2_physx_base_state_reader.py" \
  "$root/admin/spark/x2_physx_dynamics_alignment.py" \
  "$root/admin/spark/x2_physx_contact_reader.py" \
  "$root/third_party/x2-release-linux/install/x1_sim/share/x1_sim/model/x2_ultra.urdf" \
  "$root/third_party/genie_sim_engine_patch/scripts/kit/stage.py" <<'PY'
import hashlib, json, os, pathlib, shutil, sys, time
run, evidence = map(pathlib.Path, sys.argv[1:3])
domain, image, physics_hz, manifest = sys.argv[3:7]
sources = [pathlib.Path(value) for value in sys.argv[7:]]
capture = json.loads((run / "gait1-receipt.json").read_text(encoding="utf-8"))
engine_log = (run / "engine.log").read_text(encoding="utf-8", errors="replace")
alignment = json.loads((run / "dynamics-alignment.json").read_text(encoding="utf-8"))
failures = []
if capture.get("status") != "PASSED": failures.append("capture_not_passed")
if capture.get("root_pinned") is not False: failures.append("root_pin_violation")
if capture.get("gravity_modified") is not False: failures.append("gravity_violation")
if capture.get("real_robot_commands") is not False: failures.append("real_robot_flag")
if capture.get("published_command_topics") != []: failures.append("published_command_transport_detected")
if capture.get("parameters", {}).get("articulation_joint_count") != 31: failures.append("articulation_joint_count_not_31")
if capture.get("parameters", {}).get("base_state_source") != "physx_rigid_prim_world_velocity_to_base": failures.append("non_authoritative_base_velocity_source")
if alignment.get("status") != "PASSED": failures.append("dynamics_alignment_not_passed")
if alignment.get("pre_world_reset") is not True: failures.append("dynamics_alignment_after_world_reset")
if alignment.get("foot_collision_mode") not in {"imported_mesh", "vendor_capsules"}: failures.append("unexpected_foot_collision_mode")
if alignment.get("foot_collision_mode") == "vendor_capsules" and alignment.get("authored_vendor_capsule_count") != 14: failures.append("vendor_foot_capsule_count_not_14")
if alignment.get("real_robot_commands") is not False: failures.append("dynamics_alignment_real_robot_flag")
if capture.get("metrics", {}).get("stand_policy_call_count", 0) < 1000: failures.append("insufficient_stand_policy_calls")
if capture.get("metrics", {}).get("walk_policy_call_count", 0) < 350: failures.append("insufficient_walk_policy_calls")
for text, name in (
    ("CONTROLLED_PHYSICS_STOP_COMPLETE", "controlled_stop_missing"),
    ("floating gait1 development A/B", "engine_not_ready"),
    ("EXTERNAL_EFFORT_HANDOVER_COMPLETE", "external_effort_handover_missing"),
    ("ELASTIC_BAND_INITIALIZED", "elastic_band_not_initialized"),
    ("PHYSX_DYNAMICS_ALIGNMENT_PRE_RESET", "pre_reset_dynamics_alignment_log_missing"),
    ("PHYSX_BASE_STATE_READER source=physx_rigid_prim_world_velocity_to_base", "native_base_state_reader_log_missing"),
):
    if text not in engine_log: failures.append(name)
band = capture.get("elastic_band_status", {})
if band.get("state") != "RELEASED": failures.append("elastic_band_not_released")
if band.get("support_ratio") != 0.0: failures.append("elastic_band_support_not_zero")
if band.get("force_norm_n") != 0.0: failures.append("elastic_band_force_not_zero")
if not (run / "elastic-band-telemetry.jsonl").is_file(): failures.append("elastic_band_telemetry_missing")
evidence.mkdir(parents=True, exist_ok=False)
for item in run.iterdir():
    if item.is_file(): shutil.copy2(item, evidence / item.name)
source_manifest = [{
    "path": str(path), "size": path.stat().st_size,
    "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
} for path in sources]
artifacts = [{
    "path": str(path.relative_to(evidence)), "size": path.stat().st_size,
    "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
} for path in sorted(item for item in evidence.rglob("*") if item.is_file())]
payload = {
    "schema": "sceneforge/x2-geniesim-gait1-ab-gate/v1",
    "status": "PASSED" if not failures else "FAILED",
    "decision": capture.get("decision", "manual_review_required") if not failures else "debug_physx_gait1",
    "qualification": False,
    "physics_authority": "genie_sim_isaac_physx",
    "runtime": {"ros_domain_id": int(domain), "engine_image": image, "physics_hz": float(physics_hz), "manifest": manifest},
    "capture": capture,
    "dynamics_alignment": alignment,
    "source_manifest": source_manifest,
    "failures": failures,
    "artifacts": artifacts,
    "finished_at_unix": time.time(),
    "published_command_topics": [],
    "mujoco_state_source_used": False,
    "real_robot_commands": False,
}
temporary = evidence / ".result.json.tmp"
with temporary.open("w", encoding="utf-8") as stream:
    stream.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    stream.flush(); os.fsync(stream.fileno())
os.replace(temporary, evidence / "result.json")
PY

chmod -R a-w "$evidence_dir"
echo "$evidence_dir"
python3 - "$evidence_dir/result.json" <<'PY'
import json, pathlib, sys
payload = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
if payload.get("status") != "PASSED":
    raise SystemExit(";".join(payload.get("failures", ["gait1_gate_failed"])))
PY
