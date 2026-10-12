#!/usr/bin/env bash
set -euo pipefail

root=${SCENEFORGE_X2_ROOT:-/opt/sceneforge/runtime}
domain=${ROS_DOMAIN_ID:-74}
physics_hz=${X2_PHYSICS_HZ:-500}
sim_duration=${X2_SHADOW_SIM_DURATION_S:-10}
minimum_inferences=${X2_SHADOW_MINIMUM_INFERENCES:-400}
engine_image=${GENIESIM_IMAGE:-roboease/geniesim-isaac6-jazzy-arm64:20260818}
manifest=${X2_GENIESIM_MANIFEST:-/workspace/x2-overlay-v3/usd-fixed-base/manifest.json}
physics=${X2_PHYSICS_PARAMS:-/project/geniesim-build-effort/install/genie_sim_engine/share/genie_sim_engine/config/physics_params.yaml}
model=${X2_POLICY_MODEL:-/project/third_party/x2-release-linux/install/x1_rl_control/share/x1_rl_control/policy/cpgwalkrun_v25_v2.onnx}
geniesim_build=/project/geniesim-build-effort
engine=$geniesim_build/install/genie_sim_engine/lib/genie_sim_engine/genie_sim_engine_isaacsim.py
overlay="$root/assets/x2-overlay-v3"
stamp=$(date -u +%Y%m%dT%H%M%SZ)
run_id="x2-geniesim-readonly-shadow-${stamp}"
run_dir="$root/runs/$run_id"
evidence_dir="$root/evidence/$run_id"
engine_container="x2-geniesim-shadow-engine-${stamp,,}"
ros_network="x2-geniesim-shadow-${stamp,,}"

if [[ ! -d "$root" || ! -d "$overlay" ]]; then
  echo "SceneForge Spark root or X2 overlay is missing" >&2
  exit 2
fi
if [[ -e "$evidence_dir" || -e "$run_dir" ]]; then
  echo "refusing to overwrite run/evidence" >&2
  exit 2
fi
if docker ps --format '{{.Names}}' | grep -Eq 'x2-geniesim-(engine|shadow-engine)|genie.*sim.*engine'; then
  echo "another Genie Sim physics authority appears to be running" >&2
  exit 2
fi
if docker ps --format '{{.Names}}' | grep -Eq 'mujoco|dimos'; then
  echo "a MuJoCo authority appears to be running; refusing mixed authority" >&2
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
  docker logs "$engine_container" >"$run_dir/engine.log" 2>&1
  docker stop --time 20 "$engine_container" >/dev/null 2>&1
  docker rm "$engine_container" >/dev/null 2>&1
  docker network rm "$ros_network" >/dev/null 2>&1
}
trap cleanup EXIT INT TERM

docker network create --internal "$ros_network" >/dev/null
docker run -d --name "$engine_container" --network "$ros_network" --gpus all --entrypoint bash \
  --ipc host \
  -e ROS_DOMAIN_ID="$domain" -e ROS_LOCALHOST_ONLY=0 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  -e ROS_DISTRO=jazzy -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.core/jazzy/lib:/opt/ros/jazzy/lib/aarch64-linux-gnu:/opt/ros/jazzy/lib:/isaac-sim/kit/lib \
  -e PYTHONUNBUFFERED=1 -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
  -e ISAACSIM_VIEWPORT_UPDATES=0 -e REAL_ROBOT_COMMANDS=false \
  -e ROBOEASE_GENIE_PUBLISH_RENDER_BODY_TF=false \
  -e ROBOEASE_PHYSX_CONTROLLED_STOP_FILE="/project/runs/$run_id/physics-stop-request.json" \
  -v "$root:/project" -v "$overlay:/workspace/x2-overlay-v3:ro" \
  -v "$root/isaac-cache/kit:/isaac-sim/kit/cache" \
  "$engine_image" -lc "
    source /opt/ros/jazzy/setup.bash
    source '$geniesim_build/install/setup.bash'
    exec /isaac-sim/python.sh '$engine' --ros-args \
      -p stage_manifest:='$manifest' -p physics_engine:=isaac_physx \
      -p physics_hz:=$physics_hz -p render_hz:=10 -p realtime_factor:=1.0 \
      -p fake_slam:=false -p physics_params_file:='$physics' \
      -p init_joint_pos_json:='{}'
  " >/dev/null

ready=0
for _ in $(seq 1 360); do
  docker logs "$engine_container" >"$run_dir/engine.log" 2>&1 || true
  if grep -q "Isaac Sim physics engine ready" "$run_dir/engine.log"; then
    ready=1
    break
  fi
  if ! docker inspect -f '{{.State.Running}}' "$engine_container" 2>/dev/null | grep -q true; then
    break
  fi
  sleep 0.5
done
if [[ $ready -ne 1 ]]; then
  echo "Genie Sim did not become ready" >&2
  exit 3
fi

docker exec -d "$engine_container" bash -lc "
  source /opt/ros/jazzy/setup.bash
  export LD_LIBRARY_PATH=/opt/ros/jazzy/lib/aarch64-linux-gnu:/opt/ros/jazzy/lib:/isaac-sim/exts/isaacsim.ros2.core/jazzy/lib:/isaac-sim/kit/lib
  export PYTHONPATH=/project/.venv-onnx-parity/lib/python3.12/site-packages:/project/src:\${PYTHONPATH:-}
  python3 /project/admin/spark/x2-geniesim-readonly-shadow.py \
    --model '$model' --receipt '/project/runs/$run_id/shadow-receipt.json' \
    --velocity-command 0.3,0,0 --policy-rate-hz 50 \
    --fixed-base \
    --maximum-sim-duration-s '$sim_duration' --minimum-inferences '$minimum_inferences' \
    --maximum-wall-duration-s 180 --ros-args -p use_sim_time:=true \
    >'/project/runs/$run_id/shadow.log' 2>&1
  printf '%s\n' \$? >'/project/runs/$run_id/shadow.exit'
"

python3 - "$root/admin/spark/x2-geniesim-readonly-shadow.py" "$run_dir/shadow-source-audit.json" <<'PY'
import hashlib, json, os, pathlib, sys
source, receipt = map(pathlib.Path, sys.argv[1:])
text = source.read_text(encoding="utf-8")
checks = {
    "no_create_publisher": "create_publisher" not in text,
    "no_joint_command_topic": '"/joint_command"' not in text,
    "no_cmd_vel_topic": '"/cmd_vel"' not in text,
    "real_robot_commands_false": '"real_robot_commands": False' in text,
}
payload = {
    "schema": "sceneforge/x2-readonly-shadow-source-audit/v1",
    "status": "PASSED" if all(checks.values()) else "FAILED",
    "source": str(source),
    "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
    "checks": checks,
    "application_published_topics": [],
    "real_robot_commands": False,
}
temporary = receipt.with_name(receipt.name + ".tmp")
with temporary.open("w", encoding="utf-8") as stream:
    stream.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    stream.flush()
    os.fsync(stream.fileno())
os.replace(temporary, receipt)
if payload["status"] != "PASSED": raise SystemExit(1)
PY

shadow_complete=0
for _ in $(seq 1 1800); do
  if [[ -s "$run_dir/shadow-receipt.json" && -s "$run_dir/shadow.exit" ]]; then
    shadow_complete=1
    break
  fi
  if ! docker inspect -f '{{.State.Running}}' "$engine_container" 2>/dev/null | grep -q true; then
    break
  fi
  sleep 0.1
done
if [[ $shadow_complete -ne 1 ]]; then
  echo "read-only shadow did not complete" >&2
  exit 4
fi

python3 - "$run_dir/physics-stop-request.json" <<'PY'
import json, os, pathlib, sys, time
path = pathlib.Path(sys.argv[1])
payload = {
    "schema": "sceneforge/physx-controlled-stop-request/v1",
    "request": "STOP",
    "requested_at_unix": time.time(),
    "real_robot_commands": False,
}
temporary = path.with_name(path.name + ".tmp")
with temporary.open("w", encoding="utf-8") as stream:
    stream.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    stream.flush()
    os.fsync(stream.fileno())
os.replace(temporary, path)
PY

stop_complete=0
for _ in $(seq 1 600); do
  docker logs "$engine_container" >"$run_dir/engine.log" 2>&1 || true
  if grep -q "CONTROLLED_PHYSICS_STOP_COMPLETE" "$run_dir/engine.log"; then
    stop_complete=1
    break
  fi
  sleep 0.1
done
if [[ $stop_complete -ne 1 ]]; then
  echo "engine did not complete controlled stop" >&2
  exit 5
fi
if docker inspect -f '{{.State.Running}}' "$engine_container" 2>/dev/null | grep -q true; then
  docker kill "$engine_container" >/dev/null
fi
docker logs "$engine_container" >"$run_dir/engine.log" 2>&1 || true

python3 - "$run_dir" "$evidence_dir" "$domain" "$engine_image" "$physics_hz" <<'PY'
import hashlib, json, os, pathlib, shutil, sys, time
run, evidence = map(pathlib.Path, sys.argv[1:3])
domain, engine_image, physics_hz = sys.argv[3:]
shadow = json.loads((run / "shadow-receipt.json").read_text(encoding="utf-8"))
engine_log = (run / "engine.log").read_text(encoding="utf-8", errors="replace")
source_audit = json.loads((run / "shadow-source-audit.json").read_text(encoding="utf-8"))
exit_code = (run / "shadow.exit").read_text(encoding="utf-8").strip()
failures = []
if shadow.get("status") != "PASSED": failures.append("shadow_not_passed")
if shadow.get("published_topics") != []: failures.append("application_publisher_present")
if shadow.get("real_robot_commands") is not False: failures.append("real_robot_flag")
if exit_code != "0": failures.append("shadow_exit_nonzero")
if "Isaac Sim physics engine ready" not in engine_log: failures.append("engine_not_ready")
if "CONTROLLED_PHYSICS_STOP_COMPLETE" not in engine_log: failures.append("controlled_stop_missing")
if source_audit.get("status") != "PASSED": failures.append("shadow_source_audit_failed")
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
    "schema": "sceneforge/x2-geniesim-readonly-shadow-gate/v1",
    "status": "PASSED" if not failures else "FAILED",
    "decision": "proceed_to_bounded_fixed_base_capture" if not failures else "remain_read_only",
    "qualification": False,
    "physics_authority": "genie_sim_isaac_physx",
    "runtime": {"ros_domain_id": int(domain), "engine_image": engine_image, "physics_hz": float(physics_hz)},
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
    stream.flush()
    os.fsync(stream.fileno())
os.replace(temporary, evidence / "result.json")
if failures:
    raise SystemExit(";".join(failures))
PY

chmod -R a-w "$evidence_dir"
echo "$evidence_dir"
