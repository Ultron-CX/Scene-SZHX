#!/usr/bin/env bash
set -euo pipefail

# Capture one straight -> left -> right X2 runtime in exactly one newly-created
# Genie Sim/PhysX container.  This script intentionally stops before formal
# episode sealing: A/B case reports and a source-derived 30 fps video must be
# added by the evidence finalizer before x2_qualification_sealing is invoked.

root=${SCENEFORGE_X2_ROOT:-/opt/sceneforge/runtime}
physics_hz=${X2_PHYSICS_HZ:-500}
engine_image=${GENIESIM_IMAGE:-roboease/geniesim-isaac6-jazzy-arm64:20260818}
manifest=${X2_GENIESIM_MANIFEST:-/project/assets/x2-floating-dev-v1/manifest.json}
scene_yaml=${X2_SCENE_YAML:-/project/assets/x2-floating-dev-v1/scene_x2_floating.yaml}
physics=${X2_PHYSICS_PARAMS:-/project/assets/x2-floating-dev-v1/physics_params_x2.yaml}
build=${X2_GENIESIM_BUILD_ROOT:-/project/geniesim-build-effort}
engine_python=$build/install/genie_sim_engine/lib/genie_sim_engine
overlay="$root/assets/x2-overlay-v3"
stand_model=${X2_STAND_MODEL:-/project/third_party/x2-release-linux/install/x1_rl_control/share/x1_rl_control/policy/cpgtelecon_v3_fix.onnx}
walk_model=${X2_WALK_MODEL:-/project/third_party/x2-release-linux/install/x1_rl_control/share/x1_rl_control/policy/cpgwalkrun_v25_v2.onnx}
joint_limit_urdf=${X2_JOINT_LIMIT_URDF:-/project/third_party/x2-release-linux/install/x1_sim/share/x1_sim/model/x2_ultra.urdf}
ort_python=/project/runtime/x2-shadow-ort

# The reset receipt verifies the scene-authored initial joint pose. Pass that
# exact pose to the engine instead of silently overriding it with an empty map.
init_joint_pos_json=$(python3 - "$root/assets/x2-floating-dev-v1/scene_x2_floating.yaml" <<'PY'
import json, math, pathlib, sys, yaml
payload = yaml.safe_load(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
joints = payload.get("robot", {}).get("init_joint_pos")
if not isinstance(joints, dict) or len(joints) != 31:
    raise SystemExit("frozen X2 scene must define exactly 31 initial joints")
# Genie Sim's init_joint_pos transport uses degrees for revolute joints while
# SceneForge's scene/controller contracts use radians. X2 has 31 revolute DOFs.
degrees = {name: math.degrees(float(value)) for name, value in joints.items()}
print(json.dumps(degrees, separators=(",", ":"), sort_keys=True, allow_nan=False))
PY
)

[[ "${REAL_ROBOT_COMMANDS:-false}" == false ]] || {
  echo "REAL_ROBOT_COMMANDS must remain false" >&2
  exit 2
}
[[ "$manifest" == /project/assets/x2-floating-dev-v1/manifest.json ]] || {
  echo "cold-start combination requires the frozen X2 floating manifest" >&2
  exit 2
}
[[ -f "$root/runtime/x2-shadow-ort/onnxruntime/__init__.py" ]] || {
  echo "workspace-local ONNX Runtime is missing" >&2
  exit 2
}

target_count() {
  docker ps --format '{{.Names}}' | awk '
    /x2-geniesim|x2-mujoco|genie.*sim.*engine|dimos/ {count += 1}
    END {print count + 0}'
}

before_count=$(target_count)
[[ "$before_count" -eq 0 ]] || {
  echo "another simulation authority appears to be running" >&2
  exit 2
}

stamp=$(date -u +%Y%m%dT%H%M%SZ)-$$
run_id="x2-geniesim-cold-combination-$stamp"
run_dir="$root/runs/$run_id"
container="x2-geniesim-cold-combination-${stamp,,}"
network="x2-geniesim-cold-combination-${stamp,,}"
domain=$((120 + 10#${stamp:9:4} % 60))
reset_id="reset-$run_id"
[[ ! -e "$run_dir" ]] || { echo "refusing to overwrite $run_dir" >&2; exit 2; }
mkdir -p "$run_dir"
chmod 0777 "$run_dir"
# The runtime root is frozen read-only after capture.  Reserve one host-owned
# append-only area now so later video/finalization/sealing never need to make
# the raw episode root writable again.
mkdir -m 0755 "$run_dir/derived"

python3 - "$run_dir/elastic-band-command.json" <<'PY'
import json, os, pathlib, sys
path = pathlib.Path(sys.argv[1])
payload = {
    "schema": "sceneforge/physx-elastic-band-command/v1",
    "sequence": 1,
    "state": "ENGAGED",
    "support_ratio": 1.0,
    "requested_by": "x2_cold_start_combination",
    "real_robot_commands": False,
}
temporary = path.with_name(path.name + ".tmp")
with temporary.open("x", encoding="utf-8") as stream:
    stream.write(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n")
    stream.flush(); os.fsync(stream.fileno())
os.replace(temporary, path)
PY

container_removed=false
network_removed=false
cleanup() {
  set +e
  if docker inspect "$container" >/dev/null 2>&1; then
    docker logs "$container" >"$run_dir/engine.log" 2>&1 || true
    docker stop --time 20 "$container" >/dev/null 2>&1 || true
    docker rm "$container" >/dev/null 2>&1 || true
  fi
  docker network rm "$network" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

docker network create --internal "$network" >/dev/null
container_id=$(docker create --name "$container" --network "$network" --gpus all --entrypoint bash \
  --ipc host \
  -e ROS_DOMAIN_ID="$domain" -e ROS_LOCALHOST_ONLY=0 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  -e ROS_DISTRO=jazzy -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.core/jazzy/lib:/opt/ros/jazzy/lib/aarch64-linux-gnu:/opt/ros/jazzy/lib:/isaac-sim/kit/lib \
  -e PYTHONUNBUFFERED=1 -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
  -e REAL_ROBOT_COMMANDS=false -e ISAACSIM_VIEWPORT_UPDATES=0 \
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
  -e X2_COMBINATION_RUN_ROOT="/project/runs/$run_id" \
  -e X2_RESET_ID="$reset_id" -e X2_SCENE_YAML="$scene_yaml" \
  -e X2_STAND_MODEL="$stand_model" -e X2_WALK_MODEL="$walk_model" \
  -e X2_JOINT_LIMIT_URDF="$joint_limit_urdf" \
  -e X2_SIM_TIME_FREEZE_PROBE_WALL_S=2.0 \
  -e X2_PHYSX_DYNAMICS_RECEIPT="/project/runs/$run_id/dynamics-alignment.json" \
  -v "$root:/project" -v "$overlay:/workspace/x2-overlay-v3:ro" \
  -v "$root/isaac-cache/kit:/isaac-sim/kit/cache" \
  "$engine_image" -lc "
    source /opt/ros/jazzy/setup.bash
    source '$build/install/setup.bash'
    export PYTHONPATH='$ort_python:/project/src:$engine_python':\${PYTHONPATH:-}
    exec /isaac-sim/python.sh /project/admin/spark/x2-geniesim-cold-start-combination-driver.py --ros-args \
      -p stage_manifest:='$manifest' -p physics_engine:=isaac_physx \
      -p physics_hz:=$physics_hz -p render_hz:=10 -p realtime_factor:=1.0 \
      -p fake_slam:=false -p physics_params_file:='$physics' \
      -p init_joint_pos_json:='$init_joint_pos_json'
  ")

image_digest=$(docker image inspect -f '{{.Id}}' "$engine_image")
[[ "$image_digest" =~ ^sha256:[0-9a-f]{64}$ ]] || {
  echo "engine image has no immutable sha256 ID" >&2
  exit 2
}
docker start "$container" >/dev/null

ready=0
for _ in $(seq 1 600); do
  docker logs "$container" >"$run_dir/engine.log" 2>&1 || true
  if grep -Fq "single-lifecycle X2 straight+turn_left+turn_right" "$run_dir/engine.log"; then
    ready=1
    break
  fi
  [[ "$(docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null || true)" == true ]] || break
  sleep 0.5
done
[[ "$ready" -eq 1 ]] || { echo "combination engine did not become ready" >&2; exit 3; }

[[ "$(docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null || true)" == true ]] || {
  echo "combination authority exited before lifecycle observation" >&2
  exit 3
}
container_init_pid=$(docker inspect -f '{{.State.Pid}}' "$container")
started_at=$(docker inspect -f '{{.State.StartedAt}}' "$container")
docker top "$container" -eo pid,args >"$run_dir/docker-top-start.txt"
authority_process_count=$(awk '
  /\/isaac-sim\/kit\/python\/bin\/python3 .*x2-geniesim-cold-start-combination-driver.py/ {count += 1}
  END {print count + 0}' "$run_dir/docker-top-start.txt")
[[ "$authority_process_count" -eq 1 ]] || {
  echo "expected exactly one combination authority process" >&2
  exit 3
}
engine_pid=$(awk '
  /\/isaac-sim\/kit\/python\/bin\/python3 .*x2-geniesim-cold-start-combination-driver.py/ {print $1}
' "$run_dir/docker-top-start.txt")
[[ "$engine_pid" =~ ^[0-9]+$ ]] || {
  echo "could not resolve the authoritative Python process PID" >&2
  exit 3
}

finished=0
for _ in $(seq 1 18000); do
  docker logs "$container" >"$run_dir/engine.log" 2>&1 || true
  if [[ "$(docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null || true)" != true ]]; then
    finished=1
    break
  fi
  sleep 0.2
done
[[ "$finished" -eq 1 ]] || { echo "combination runtime timed out" >&2; exit 4; }

exit_code=$(docker inspect -f '{{.State.ExitCode}}' "$container")
finished_at=$(docker inspect -f '{{.State.FinishedAt}}' "$container")
docker logs "$container" >"$run_dir/engine.log" 2>&1 || true
grep -Fq "CONTROLLED_PHYSICS_STOP_COMPLETE" "$run_dir/engine.log" || {
  echo "controlled PhysX stop was not proven" >&2
  exit 4
}
[[ "$exit_code" -eq 0 ]] || { echo "combination driver exited $exit_code" >&2; exit 4; }
[[ -s "$run_dir/combination-runtime-result.json" ]] || {
  echo "combination runtime result is missing" >&2
  exit 4
}

docker rm "$container" >/dev/null
container_removed=true
docker network rm "$network" >/dev/null
network_removed=true
removed_at_unix=$(python3 -c 'import time; print(time.time())')
after_count=$(target_count)

python3 - "$run_dir" "$container_id" "$image_digest" "$engine_pid" "$container_init_pid" \
  "$started_at" "$finished_at" "$removed_at_unix" "$exit_code" \
  "$authority_process_count" "$before_count" "$after_count" <<'PY'
import datetime as dt, json, os, pathlib, sys

(
    run, container_id, image_digest, engine_pid, container_init_pid, started, finished,
    removed_at, exit_code, authority_count, before_count, after_count,
) = sys.argv[1:]
run = pathlib.Path(run)

def unix(value):
    return dt.datetime.fromisoformat(value.replace("Z", "+00:00")).timestamp()

started_unix = unix(started)
finished_unix = unix(finished)
identity = f"{container_id}:{engine_pid}:{started}"
lifecycle = {
    "schema": "sceneforge/x2-physx-container-lifecycle-receipt/v1",
    "cold_start": True,
    "container_recreated": True,
    "started_cleanly": True,
    "controlled_stop_complete": True,
    "teardown_complete": int(after_count) == 0,
    "container_id": container_id,
    "container_image_digest": image_digest,
    "physics_authority_process_count": int(authority_count),
    "residual_process_count": int(after_count),
    "target_processes_before_start": int(before_count),
    "target_processes_after_stop": int(after_count),
    "container_exit_code": int(exit_code),
    "container_started_at_unix": started_unix,
    "container_removed_at_unix": float(removed_at),
    "engine_process": {
        "mode": "in_process",
        "identity": identity,
        "pid": int(engine_pid),
        "started_at_unix": started_unix,
        "stopped_at_unix": finished_unix,
        "exit_code": int(exit_code),
    },
    "launcher_process": {
        "identity": f"{container_id}:{container_init_pid}:{started}",
        "pid": int(container_init_pid),
        "role": "isaac_python_sh_wrapper",
        "started_at_unix": started_unix,
        "stopped_at_unix": finished_unix,
        "exit_code": int(exit_code),
    },
    "controller_process": {
        "mode": "in_process",
        "identity": identity,
        "pid": int(engine_pid),
        "started_at_unix": started_unix,
        "stopped_at_unix": finished_unix,
        "exit_code": int(exit_code),
    },
    "physics_authority": "genie_sim_isaac_physx",
    "published_command_topics": [],
    "real_robot_commands": False,
}
path = run / "lifecycle-receipt.json"
temporary = path.with_name(path.name + ".tmp")
temporary.write_text(json.dumps(lifecycle, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
with temporary.open("rb") as stream: os.fsync(stream.fileno())
os.replace(temporary, path)
PY

python3 - "$run_dir" "$engine_image" "$image_digest" "$physics_hz" "$root" <<'PY'
import hashlib, json, os, pathlib, shutil, sys, time
run, image, image_digest, physics_hz, root = sys.argv[1:]
run, root = pathlib.Path(run), pathlib.Path(root)

def load(path): return json.loads(path.read_text(encoding="utf-8"))
def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def write(path, payload):
    if path.exists(): raise FileExistsError(path)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    with temporary.open("rb") as stream: os.fsync(stream.fileno())
    os.replace(temporary, path)

runtime = load(run / "combination-runtime-result.json")
reset = load(run / "reset-receipt.json")
lifecycle = load(run / "lifecycle-receipt.json")
if runtime.get("status") != "PASSED" or reset.get("status") != "PASSED":
    raise SystemExit("runtime/reset combination did not pass")
if lifecycle.get("teardown_complete") is not True:
    raise SystemExit("container teardown did not pass")
engine_log = (run / "engine.log").read_text(encoding="utf-8", errors="replace")
if "CONTROLLED_PHYSICS_STOP_COMPLETE" not in engine_log:
    raise SystemExit("controlled stop missing from engine log")
(run / "controller.log").write_text(
    "\n".join(line for line in engine_log.splitlines() if "X2" in line or "gait" in line) + "\n",
    encoding="utf-8",
)
write(run / "controlled-stop-proof.json", {
    "schema": "sceneforge/x2-physx-controlled-stop-proof/v1",
    "status": "PASSED",
    "request_present": (run / "physics-stop-request.json").is_file(),
    "engine_logged_completion": True,
    "container_exit_code": lifecycle["container_exit_code"],
    "real_robot_commands": False,
})
write(run / "sim-time-freeze-receipt.json", runtime["sim_time_freeze"])

all_physics, all_base, all_commands, all_contacts = [], [], [], []
joint_names = None
for scenario in ("straight", "turn_left", "turn_right"):
    trace = load(run / "cases" / scenario / "state-command-trace.json")
    if trace.get("real_robot_commands") is not False:
        raise SystemExit("trace safety flag failed")
    if joint_names is None: joint_names = trace["joint_names"]
    if trace["joint_names"] != joint_names: raise SystemExit("joint order changed between cases")
    for target, key in ((all_physics, "physics_samples"), (all_base, "base_samples"), (all_commands, "commands"), (all_contacts, "contact_samples")):
        for local_index, row in enumerate(trace[key], 1):
            record = dict(row); record["scenario"] = scenario; record["case_local_index"] = local_index
            target.append(record)
write(run / "state-command-trace.json", {
    "schema": "sceneforge/x2-physx-cold-start-state-command-trace/v1",
    "joint_names": joint_names,
    "physics_samples": all_physics,
    "base_samples": all_base,
    "commands": all_commands,
    "physics_authority": "genie_sim_isaac_physx",
    "real_robot_commands": False,
})
write(run / "contact-trace.json", {
    "schema": "sceneforge/x2-physx-cold-start-contact-trace/v1",
    "samples": all_contacts,
    "physics_authority": "genie_sim_isaac_physx",
    "real_robot_commands": False,
})

sources = [
    "src/sceneforge/x2_control.py", "src/sceneforge/x2_session.py",
    "src/sceneforge/x2_physx_gait1.py", "src/sceneforge/x2_cold_start_combination.py",
    "admin/spark/x2-geniesim-cold-start-combination-driver.py",
    "admin/spark/x2_physx_base_state_reader.py", "admin/spark/x2_physx_contact_reader.py",
    "third_party/genie_sim_engine_patch/scripts/common/loop.py",
    "third_party/genie_sim_engine_patch/scripts/kit/stage.py",
]
source_rows = [{"path": item, "size": (root/item).stat().st_size, "sha256": digest(root/item)} for item in sources]
write(run / "source-manifest.json", {"schema":"sceneforge/source-manifest/v1", "files":source_rows, "real_robot_commands":False})
asset_paths = [root/"assets/x2-floating-dev-v1/manifest.json", root/"assets/x2-floating-dev-v1/scene_x2_floating.yaml", root/"assets/x2-floating-dev-v1/physics_params_x2.yaml"]
write(run / "asset-manifest.json", {"schema":"sceneforge/asset-manifest/v1", "files":[{"path":str(p.relative_to(root)),"size":p.stat().st_size,"sha256":digest(p)} for p in asset_paths], "real_robot_commands":False})
first = load(run / "cases/straight/physx-result.json")
write(run / "controller-manifest.json", {"schema":"sceneforge/x2-controller-manifest/v1", "controller":first.get("controller"), "model":first.get("model"), "position_target_safety_envelope":first.get("position_target_safety_envelope"), "real_robot_commands":False})
write(run / "runtime-manifest.json", {"schema":"sceneforge/x2-runtime-manifest/v1", "engine_image":image, "engine_image_digest":image_digest, "physics_hz":float(physics_hz), "single_physics_lifecycle":True, "case_order":["straight","turn_left","turn_right"], "real_robot_commands":False})
write(run / "runtime-bundle-result.json", {
    "schema":"sceneforge/x2-cold-start-runtime-bundle/v1",
    "status":"PASSED",
    "qualification":False,
    "next":"produce_case_ab_reports_and_offline_video_then_seal_episode",
    "missing_for_qualification":["straight_result","turn_left_result","turn_right_result","video","video_receipt","frozen_input_manifests","aggregate_safety_and_stable_stop"],
    "physics_authority":"genie_sim_isaac_physx",
    "single_physics_lifecycle":True,
    "published_command_topics":[],
    "real_robot_commands":False,
    "finished_at_unix":time.time(),
})

# Bind every original runtime file before permissions are frozen.  Later tools
# may only add products below derived/ and must verify this exact raw file set.
index_name = "runtime-frozen-index.json"
rows = []
for path in sorted(run.rglob("*"), key=lambda item: item.relative_to(run).as_posix()):
    relative = path.relative_to(run)
    if relative.parts and relative.parts[0] == "derived":
        continue
    if path.is_symlink():
        raise SystemExit(f"raw runtime symlink is forbidden: {relative.as_posix()}")
    if path.is_file() and relative.as_posix() != index_name:
        rows.append({
            "path": relative.as_posix(),
            "size": path.stat().st_size,
            "sha256": digest(path),
        })
write(run / index_name, {
    "schema": "sceneforge/x2-frozen-runtime-index/v1",
    "derivation_root": "derived",
    "file_count": len(rows),
    "files": rows,
    "qualification": False,
    "real_robot_commands": False,
})
PY

# Root-own and freeze every raw entry so the Spark login user cannot restore a
# write bit with chmod.  The host-owned derived/ child is deliberately pruned
# and remains the only post-runtime publication area.  This short-lived utility
# container is scoped to the exact run and executes no project or physics code.
docker run --rm --user 0:0 --entrypoint bash \
  -v "$run_dir:/evidence-run" "$engine_image" -lc \
  'find /evidence-run -path /evidence-run/derived -prune -o -type f -exec chown 0:0 {} + -exec chmod 0444 {} +; find /evidence-run -path /evidence-run/derived -prune -o -type d -exec chown 0:0 {} + -exec chmod 0555 {} +; chmod 0755 /evidence-run/derived'
trap - EXIT INT TERM
printf '%s\n' "$run_dir"
