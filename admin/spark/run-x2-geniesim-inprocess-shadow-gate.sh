#!/usr/bin/env bash
set -euo pipefail

root=${SCENEFORGE_X2_ROOT:-/opt/sceneforge/runtime}
domain=${ROS_DOMAIN_ID:-75}
physics_hz=${X2_PHYSICS_HZ:-500}
sim_duration=${X2_SHADOW_SIM_DURATION_S:-10}
minimum_inferences=${X2_SHADOW_MINIMUM_INFERENCES:-400}
engine_image=${GENIESIM_IMAGE:-roboease/geniesim-isaac6-jazzy-arm64:20260818}
manifest=${X2_GENIESIM_MANIFEST:-/workspace/x2-overlay-v3/usd-fixed-base/manifest.json}
build=/project/geniesim-build-effort
engine_python=$build/install/genie_sim_engine/lib/genie_sim_engine
physics=${X2_PHYSICS_PARAMS:-$build/install/genie_sim_engine/share/genie_sim_engine/config/physics_params.yaml}
model=${X2_POLICY_MODEL:-/project/third_party/x2-release-linux/install/x1_rl_control/share/x1_rl_control/policy/cpgwalkrun_v25_v2.onnx}
ort_python=/project/runtime/x2-shadow-ort
overlay="$root/assets/x2-overlay-v3"
stamp=$(date -u +%Y%m%dT%H%M%SZ)
run_id="x2-geniesim-inprocess-shadow-${stamp}"
run_dir="$root/runs/$run_id"
evidence_dir="$root/evidence/$run_id"
container="x2-geniesim-inprocess-shadow-${stamp,,}"
network="x2-geniesim-inprocess-shadow-${stamp,,}"

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

python3 - "$root/src/sceneforge/x2_inprocess_shadow.py" "$root/admin/spark/x2-geniesim-inprocess-shadow-driver.py" "$run_dir/source-audit.json" <<'PY'
import hashlib, json, os, pathlib, sys
hook, driver, receipt = map(pathlib.Path, sys.argv[1:])
texts = [hook.read_text(encoding="utf-8"), driver.read_text(encoding="utf-8")]
checks = {
    "no_apply_commands": all("apply_commands" not in text for text in texts),
    "no_create_publisher": all("create_publisher" not in text for text in texts),
    "no_joint_command_topic": all('"/joint_command"' not in text for text in texts),
    "no_cmd_vel_topic": all('"/cmd_vel"' not in text for text in texts),
    "explicit_zero_command_calls": '"command_application_calls": 0' in texts[0],
    "real_robot_commands_false": '"real_robot_commands": False' in texts[0],
}
payload = {
    "schema": "sceneforge/x2-inprocess-shadow-source-audit/v1",
    "status": "PASSED" if all(checks.values()) else "FAILED",
    "checks": checks,
    "files": [
        {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        for path in (hook, driver)
    ],
    "real_robot_commands": False,
}
temporary = receipt.with_name(receipt.name + ".tmp")
with temporary.open("w", encoding="utf-8") as stream:
    stream.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    stream.flush(); os.fsync(stream.fileno())
os.replace(temporary, receipt)
if payload["status"] != "PASSED": raise SystemExit(1)
PY

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
  -e X2_SHADOW_MODEL="$model" \
  -e X2_SHADOW_RECEIPT="/project/runs/$run_id/shadow-receipt.json" \
  -e X2_SHADOW_POLICY_HZ=50 -e X2_SHADOW_SIM_DURATION_S="$sim_duration" \
  -e X2_SHADOW_MINIMUM_INFERENCES="$minimum_inferences" \
  -v "$root:/project" -v "$overlay:/workspace/x2-overlay-v3:ro" \
  -v "$root/isaac-cache/kit:/isaac-sim/kit/cache" \
  "$engine_image" -lc "
    source /opt/ros/jazzy/setup.bash
    source '$build/install/setup.bash'
    export PYTHONPATH='$ort_python:/project/src:$engine_python':\${PYTHONPATH:-}
    exec /isaac-sim/python.sh /project/admin/spark/x2-geniesim-inprocess-shadow-driver.py --ros-args \
      -p stage_manifest:='$manifest' -p physics_engine:=isaac_physx \
      -p physics_hz:=$physics_hz -p render_hz:=10 -p realtime_factor:=1.0 \
      -p fake_slam:=false -p physics_params_file:='$physics' \
      -p init_joint_pos_json:='{}'
  " >/dev/null

ready=0
for _ in $(seq 1 360); do
  docker logs "$container" >"$run_dir/engine.log" 2>&1 || true
  if grep -q "Isaac Sim physics engine ready" "$run_dir/engine.log"; then ready=1; break; fi
  if ! docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null | grep -q true; then break; fi
  sleep 0.5
done
if [[ $ready -ne 1 ]]; then
  echo "Genie Sim did not become ready" >&2
  exit 3
fi

complete=0
for _ in $(seq 1 1800); do
  docker logs "$container" >"$run_dir/engine.log" 2>&1 || true
  if [[ -s "$run_dir/shadow-receipt.json" ]] \
    && grep -q "CONTROLLED_PHYSICS_STOP_COMPLETE" "$run_dir/engine.log"; then
    complete=1
    break
  fi
  if ! docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null | grep -q true; then break; fi
  sleep 0.1
done
if [[ $complete -ne 1 ]]; then
  echo "in-process shadow or controlled stop did not complete" >&2
  exit 4
fi
if docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null | grep -q true; then
  docker kill "$container" >/dev/null
fi
docker logs "$container" >"$run_dir/engine.log" 2>&1 || true

python3 - "$run_dir" "$evidence_dir" "$domain" "$engine_image" "$physics_hz" "$root/runtime/x2-shadow-ort" <<'PY'
import hashlib, json, os, pathlib, shutil, sys, time
run, evidence = map(pathlib.Path, sys.argv[1:3])
domain, image, physics_hz, ort_path = sys.argv[3:]
shadow = json.loads((run / "shadow-receipt.json").read_text(encoding="utf-8"))
source = json.loads((run / "source-audit.json").read_text(encoding="utf-8"))
engine_log = (run / "engine.log").read_text(encoding="utf-8", errors="replace")
failures = []
if shadow.get("status") != "PASSED": failures.append("shadow_not_passed")
if source.get("status") != "PASSED": failures.append("source_audit_not_passed")
if shadow.get("command_application_calls") != 0: failures.append("command_application_detected")
if shadow.get("real_robot_commands") is not False: failures.append("real_robot_flag")
if "CONTROLLED_PHYSICS_STOP_COMPLETE" not in engine_log: failures.append("controlled_stop_missing")
if "Isaac Sim physics engine ready" not in engine_log: failures.append("engine_not_ready")
evidence.mkdir(parents=True, exist_ok=False)
for item in run.iterdir():
    if item.is_file(): shutil.copy2(item, evidence / item.name)
artifacts = []
for path in sorted(item for item in evidence.rglob("*") if item.is_file()):
    artifacts.append({
        "path": str(path.relative_to(evidence)),
        "size": path.stat().st_size,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    })
ort_files = sorted(pathlib.Path(ort_path).glob("onnxruntime*/METADATA"))
payload = {
    "schema": "sceneforge/x2-geniesim-inprocess-shadow-gate/v1",
    "status": "PASSED" if not failures else "FAILED",
    "decision": "proceed_to_bounded_fixed_base_command_capture" if not failures else "remain_read_only",
    "qualification": False,
    "physics_authority": "genie_sim_isaac_physx",
    "runtime": {
        "ros_domain_id": int(domain), "engine_image": image,
        "physics_hz": float(physics_hz), "onnxruntime_path": ort_path,
        "onnxruntime_metadata_sha256": [hashlib.sha256(path.read_bytes()).hexdigest() for path in ort_files],
    },
    "shadow": shadow,
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
