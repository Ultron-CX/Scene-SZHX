#!/usr/bin/env bash
set -euo pipefail

root=${SCENEFORGE_X2_ROOT:-/opt/sceneforge/runtime}
domain=${ROS_DOMAIN_ID:-77}
physics_hz=${X2_PHYSICS_HZ:-500}
engine_image=${GENIESIM_IMAGE:-roboease/geniesim-isaac6-jazzy-arm64:20260818}
manifest=${X2_GENIESIM_MANIFEST:-/workspace/x2-overlay-v3/usd-fixed-base/manifest.json}
build=${X2_GENIESIM_BUILD_ROOT:-/project/geniesim-build-effort}
engine_python=$build/install/genie_sim_engine/lib/genie_sim_engine
physics=${X2_PHYSICS_PARAMS:-$build/install/genie_sim_engine/share/genie_sim_engine/config/physics_params.yaml}
overlay="$root/assets/x2-overlay-v3"
model=${X2_STAND_MODEL:-/project/third_party/x2-release-linux/install/x1_rl_control/share/x1_rl_control/policy/cpgtelecon_v3_fix.onnx}
ort_python=/project/runtime/x2-shadow-ort
stamp=$(date -u +%Y%m%dT%H%M%SZ)
run_id="x2-geniesim-fixed-base-stand-${stamp}"
run_dir="$root/runs/$run_id"
evidence_dir="$root/evidence/$run_id"
container="x2-geniesim-fixed-base-stand-${stamp,,}"
network="x2-geniesim-fixed-base-stand-${stamp,,}"

if [[ "$manifest" != *"/usd-fixed-base/manifest.json" ]]; then
  echo "STAND_DEFAULT gate requires the frozen fixed-base manifest" >&2
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

cleanup() {
  set +e
  docker logs "$container" >"$run_dir/engine.log" 2>&1
  docker stop --time 20 "$container" >/dev/null 2>&1
  docker rm "$container" >/dev/null 2>&1
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
  -e ROBOEASE_PHYSX_CONTROLLED_STOP_FILE="/project/runs/$run_id/physics-stop-request.json" \
  -e GENIESIM_ENGINE_PYTHON_DIR="$engine_python" \
  -e X2_STAND_MODEL="$model" \
  -e X2_STAND_RECEIPT="/project/runs/$run_id/stand-receipt.json" \
  -e X2_STAND_TRACE="/project/runs/$run_id/stand-trace.json" \
  -v "$root:/project" -v "$overlay:/workspace/x2-overlay-v3:ro" \
  -v "$root/isaac-cache/kit:/isaac-sim/kit/cache" \
  "$engine_image" -lc "
    source /opt/ros/jazzy/setup.bash
    source '$build/install/setup.bash'
    export PYTHONPATH='$ort_python:/project/src:$engine_python':\${PYTHONPATH:-}
    exec /isaac-sim/python.sh /project/admin/spark/x2-geniesim-fixed-base-stand-driver.py --ros-args \
      -p stage_manifest:='$manifest' -p physics_engine:=isaac_physx \
      -p physics_hz:=$physics_hz -p render_hz:=10 -p realtime_factor:=1.0 \
      -p fake_slam:=false -p physics_params_file:='$physics' \
      -p init_joint_pos_json:='{}'
  " >/dev/null

ready=0
for _ in $(seq 1 360); do
  docker logs "$container" >"$run_dir/engine.log" 2>&1 || true
  if grep -q "fixed-base STAND_DEFAULT capture" "$run_dir/engine.log"; then ready=1; break; fi
  if ! docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null | grep -q true; then break; fi
  sleep 0.5
done
if [[ $ready -ne 1 ]]; then
  echo "Genie Sim did not become ready" >&2
  exit 3
fi

complete=0
for _ in $(seq 1 2400); do
  docker logs "$container" >"$run_dir/engine.log" 2>&1 || true
  if [[ -s "$run_dir/stand-receipt.json" ]] \
    && grep -q "CONTROLLED_PHYSICS_STOP_COMPLETE" "$run_dir/engine.log"; then
    complete=1
    break
  fi
  if ! docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null | grep -q true; then break; fi
  sleep 0.1
done
if [[ $complete -ne 1 ]]; then
  echo "STAND_DEFAULT capture or controlled stop did not complete" >&2
  exit 4
fi
if docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null | grep -q true; then
  docker kill "$container" >/dev/null
fi
docker logs "$container" >"$run_dir/engine.log" 2>&1 || true

python3 - "$run_dir" "$evidence_dir" "$domain" "$engine_image" "$physics_hz" "$manifest" \
  "$root/src/sceneforge/x2_stand.py" "$root/src/sceneforge/x2_fixed_base_stand.py" \
  "$root/admin/spark/x2-geniesim-fixed-base-stand-driver.py" <<'PY'
import hashlib, json, os, pathlib, shutil, sys, time
run, evidence = map(pathlib.Path, sys.argv[1:3])
domain, image, physics_hz, manifest = sys.argv[3:7]
sources = [pathlib.Path(value) for value in sys.argv[7:]]
capture = json.loads((run / "stand-receipt.json").read_text(encoding="utf-8"))
engine_log = (run / "engine.log").read_text(encoding="utf-8", errors="replace")
failures = []
if capture.get("status") != "PASSED": failures.append("capture_not_passed")
if capture.get("base_mode") != "fixed": failures.append("base_not_fixed")
if capture.get("real_robot_commands") is not False: failures.append("real_robot_flag")
if capture.get("published_command_topics") != []: failures.append("published_command_transport_detected")
if capture.get("parameters", {}).get("joint_count") != 23: failures.append("joint_count_not_23")
if capture.get("parameters", {}).get("articulation_joint_count") != 31: failures.append("articulation_joint_count_not_31")
if capture.get("metrics", {}).get("policy_call_count", 0) < 40: failures.append("insufficient_policy_calls")
if "CONTROLLED_PHYSICS_STOP_COMPLETE" not in engine_log: failures.append("controlled_stop_missing")
if "fixed-base STAND_DEFAULT capture" not in engine_log: failures.append("engine_not_ready")
if "EXTERNAL_EFFORT_HANDOVER_COMPLETE" not in engine_log: failures.append("external_effort_handover_missing")
evidence.mkdir(parents=True, exist_ok=False)
for item in run.iterdir():
    if item.is_file(): shutil.copy2(item, evidence / item.name)
source_manifest = []
for path in sources:
    source_manifest.append({
        "path": str(path), "size": path.stat().st_size,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    })
artifacts = []
for path in sorted(item for item in evidence.rglob("*") if item.is_file()):
    artifacts.append({
        "path": str(path.relative_to(evidence)), "size": path.stat().st_size,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    })
payload = {
    "schema": "sceneforge/x2-geniesim-fixed-base-stand-gate/v1",
    "status": "PASSED" if not failures else "FAILED",
    "decision": "proceed_to_suspension_assisted_floating_stand" if not failures else "remain_at_fixed_base_stand_default",
    "qualification": False,
    "physics_authority": "genie_sim_isaac_physx",
    "runtime": {"ros_domain_id": int(domain), "engine_image": image, "physics_hz": float(physics_hz), "manifest": manifest},
    "capture": capture,
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
if failures: raise SystemExit(";".join(failures))
PY

chmod -R a-w "$evidence_dir"
echo "$evidence_dir"
