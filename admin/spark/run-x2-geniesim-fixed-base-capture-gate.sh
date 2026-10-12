#!/usr/bin/env bash
set -euo pipefail

root=${SCENEFORGE_X2_ROOT:-/opt/sceneforge/runtime}
domain=${ROS_DOMAIN_ID:-76}
physics_hz=${X2_PHYSICS_HZ:-500}
amplitude=${X2_CAPTURE_AMPLITUDE_RAD:-0.02}
engine_image=${GENIESIM_IMAGE:-roboease/geniesim-isaac6-jazzy-arm64:20260818}
manifest=${X2_GENIESIM_MANIFEST:-/workspace/x2-overlay-v3/usd-fixed-base/manifest.json}
build=/project/geniesim-build-effort
engine_python=$build/install/genie_sim_engine/lib/genie_sim_engine
physics=${X2_PHYSICS_PARAMS:-$build/install/genie_sim_engine/share/genie_sim_engine/config/physics_params.yaml}
overlay="$root/assets/x2-overlay-v3"
stamp=$(date -u +%Y%m%dT%H%M%SZ)
run_id="x2-geniesim-fixed-base-capture-${stamp}"
run_dir="$root/runs/$run_id"
evidence_dir="$root/evidence/$run_id"
container="x2-geniesim-fixed-base-capture-${stamp,,}"
network="x2-geniesim-fixed-base-capture-${stamp,,}"

if [[ "$manifest" != *"/usd-fixed-base/manifest.json" ]]; then
  echo "capture gate requires the frozen fixed-base manifest" >&2
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

python3 - \
  "$root/src/sceneforge/x2_fixed_base_capture.py" \
  "$root/admin/spark/x2-geniesim-fixed-base-capture-driver.py" \
  "$root/source/genie_sim/source/geniesim_ros/src/ros_ws/src/genie_sim_engine/scripts/common/loop.py" \
  "$root/source/genie_sim/source/geniesim_ros/src/ros_ws/src/genie_sim_engine/scripts/kit/stage.py" \
  "$run_dir/source-audit.json" <<'PY'
import ast, hashlib, json, os, pathlib, sys
capture, driver, loop, stage, receipt = map(pathlib.Path, sys.argv[1:])
capture_text = capture.read_text(encoding="utf-8")
driver_text = driver.read_text(encoding="utf-8")
tree = ast.parse(capture_text)
calls = [
    node for node in ast.walk(tree)
    if isinstance(node, ast.Call)
    and isinstance(node.func, ast.Attribute)
    and node.func.attr == "apply_commands"
]
checks = {
    "one_apply_commands_callsite": len(calls) == 1,
    "single_joint_hard_allowlist": 'DEFAULT_JOINT = "left_shoulder_pitch_joint"' in capture_text,
    "amplitude_hard_limit": "MAXIMUM_AMPLITUDE_RAD = 0.03" in capture_text,
    "duration_hard_limit": "MAXIMUM_TOTAL_DURATION_S = 5.0" in capture_text,
    "empty_effort_packet": "cmd_efforts={}" in capture_text,
    "no_ros_command_publisher": all(
        token not in capture_text + driver_text
        for token in ("create_publisher", '"/joint_command"', '"/cmd_vel"', "rclpy")
    ),
    "post_step_binding": "session.post_step_hooks.append(capture.step)" in driver_text,
    "fixed_physx_only": 'physics_engine != "isaac_physx"' in driver_text,
    "real_robot_commands_false": '"real_robot_commands": False' in capture_text,
    "engine_applies_buffer_before_step": (
        loop.read_text(encoding="utf-8").find("sim.apply_commands(")
        < loop.read_text(encoding="utf-8").find("solver_ms = sim.step(")
    ),
}
files = []
for path in (capture, driver, loop, stage):
    files.append({
        "path": str(path),
        "size": path.stat().st_size,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    })
payload = {
    "schema": "sceneforge/x2-fixed-base-capture-source-audit/v1",
    "status": "PASSED" if all(checks.values()) else "FAILED",
    "checks": checks,
    "files": files,
    "real_robot_commands": False,
}
temporary = receipt.with_name(receipt.name + ".tmp")
with temporary.open("w", encoding="utf-8") as stream:
    stream.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    stream.flush(); os.fsync(stream.fileno())
os.replace(temporary, receipt)
if payload["status"] != "PASSED":
    raise SystemExit(1)
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
  -e X2_CAPTURE_RECEIPT="/project/runs/$run_id/capture-receipt.json" \
  -e X2_CAPTURE_TRACE="/project/runs/$run_id/capture-trace.json" \
  -e X2_CAPTURE_AMPLITUDE_RAD="$amplitude" \
  -v "$root:/project" -v "$overlay:/workspace/x2-overlay-v3:ro" \
  -v "$root/isaac-cache/kit:/isaac-sim/kit/cache" \
  "$engine_image" -lc "
    source /opt/ros/jazzy/setup.bash
    source '$build/install/setup.bash'
    export PYTHONPATH='/project/src:$engine_python':\${PYTHONPATH:-}
    exec /isaac-sim/python.sh /project/admin/spark/x2-geniesim-fixed-base-capture-driver.py --ros-args \
      -p stage_manifest:='$manifest' -p physics_engine:=isaac_physx \
      -p physics_hz:=$physics_hz -p render_hz:=10 -p realtime_factor:=1.0 \
      -p fake_slam:=false -p physics_params_file:='$physics' \
      -p init_joint_pos_json:='{}'
  " >/dev/null

ready=0
for _ in $(seq 1 360); do
  docker logs "$container" >"$run_dir/engine.log" 2>&1 || true
  if grep -q "Isaac Sim physics engine ready" "$run_dir/engine.log"; then
    ready=1; break
  fi
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
  if [[ -s "$run_dir/capture-receipt.json" ]] \
    && grep -q "CONTROLLED_PHYSICS_STOP_COMPLETE" "$run_dir/engine.log"; then
    complete=1
    break
  fi
  if ! docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null | grep -q true; then break; fi
  sleep 0.1
done
if [[ $complete -ne 1 ]]; then
  echo "capture or controlled stop did not complete" >&2
  exit 4
fi
if docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null | grep -q true; then
  docker kill "$container" >/dev/null
fi
docker logs "$container" >"$run_dir/engine.log" 2>&1 || true

python3 - "$run_dir" "$evidence_dir" "$domain" "$engine_image" "$physics_hz" "$manifest" <<'PY'
import hashlib, json, os, pathlib, shutil, sys, time
run, evidence = map(pathlib.Path, sys.argv[1:3])
domain, image, physics_hz, manifest = sys.argv[3:]
capture = json.loads((run / "capture-receipt.json").read_text(encoding="utf-8"))
source = json.loads((run / "source-audit.json").read_text(encoding="utf-8"))
engine_log = (run / "engine.log").read_text(encoding="utf-8", errors="replace")
failures = []
if capture.get("status") != "PASSED": failures.append("capture_not_passed")
if source.get("status") != "PASSED": failures.append("source_audit_not_passed")
if capture.get("base_mode") != "fixed": failures.append("base_not_fixed")
if capture.get("real_robot_commands") is not False: failures.append("real_robot_flag")
if capture.get("published_command_topics") != []: failures.append("published_command_transport_detected")
if capture.get("metrics", {}).get("command_application_calls", 0) < 80: failures.append("insufficient_commands")
if "CONTROLLED_PHYSICS_STOP_COMPLETE" not in engine_log: failures.append("controlled_stop_missing")
if "Isaac Sim physics engine ready" not in engine_log: failures.append("engine_not_ready")
if "usd-fixed-base/manifest.json" not in manifest: failures.append("fixed_manifest_missing")
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
payload = {
    "schema": "sceneforge/x2-geniesim-fixed-base-command-gate/v1",
    "status": "PASSED" if not failures else "FAILED",
    "decision": "proceed_to_stand_default_output_capture" if not failures else "remain_at_fixed_base_command_capture",
    "qualification": False,
    "physics_authority": "genie_sim_isaac_physx",
    "runtime": {
        "ros_domain_id": int(domain),
        "engine_image": image,
        "physics_hz": float(physics_hz),
        "manifest": manifest,
    },
    "capture": capture,
    "source_audit": source,
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
