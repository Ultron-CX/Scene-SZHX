#!/usr/bin/env bash
set -euo pipefail

# Render one immutable successful X2 PhysX trace after the authority has stopped.
# This script never launches a physics engine or publishes robot commands.

if [[ $# -ne 1 ]]; then
  echo "usage: $0 <successful-x2-evidence-directory>" >&2
  exit 2
fi

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)
source_dir=$(realpath "$1")
if [[ "$source_dir" != "$root/evidence/"* ]]; then
  echo "source evidence must be inside $root/evidence" >&2
  exit 2
fi
if [[ "${REAL_ROBOT_COMMANDS:-false}" != false ]]; then
  echo "real robot commands are forbidden" >&2
  exit 2
fi
if docker ps --format '{{.Names}}' | grep -Eq '^x2-geniesim-engine-'; then
  echo "refusing offline replay while a PhysX authority is running" >&2
  exit 2
fi

stamp=$(date -u +%Y%m%dT%H%M%SZ)
demo="$root/evidence/x2-geniesim-render-demo-$stamp"
mkdir "$demo"
chmod 0777 "$demo"
source_rel=${source_dir#"$root/"}
# Fast DDS derives UDP ports from the domain ID and rejects values above its
# configured range on Spark.  Keep replay domains in a private, valid band.
domain=$((80 + 10#${stamp:9:4} % 100))
container="x2-geniesim-offline-render-${stamp,,}"
network="x2-geniesim-offline-render-${stamp,,}"
engine_image=${GENIESIM_IMAGE:-roboease/geniesim-isaac6-jazzy-arm64:20260818}
overlay="$root/assets/x2-overlay-v3"
manifest=/project/assets/x2-floating-dev-v1/manifest.json

cleanup() {
  set +e
  docker logs "$container" >"$demo/render.log" 2>&1 || true
  docker stop --time 30 "$container" >/dev/null 2>&1 || true
  docker rm "$container" >/dev/null 2>&1 || true
  docker network rm "$network" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

python3 - "$source_dir" "$demo/window.json" <<'PY'
import hashlib, json, os, pathlib, sys
source, output = map(pathlib.Path, sys.argv[1:])
result = json.loads((source / "result.json").read_text(encoding="utf-8"))
if result.get("status") != "PASSED" or result.get("real_robot_commands") is not False:
    raise SystemExit("source is not a safe successful PhysX run")
if result.get("physics_authority") != "genie_sim_isaac_physx":
    raise SystemExit("source physics authority is not Genie Sim PhysX")
trace = result.get("render_state_trace") or {}
if trace.get("status") != "SUCCEEDED" or trace.get("read_only") is not True:
    raise SystemExit("source does not contain a successful read-only render trace")
turn = result.get("turn_metrics") or {}
start, end = turn.get("start_stamp_s"), turn.get("stop_stamp_s")
if start is None or end is None or float(end) <= float(start):
    raise SystemExit("source result lacks a valid locomotion timestamp window")
payload = {
    "schema": "sceneforge/x2-geniesim-render-window/v1",
    "source_result": str(source / "result.json"),
    "source_result_sha256": hashlib.sha256((source / "result.json").read_bytes()).hexdigest(),
    "source_start_s": float(start) - 2.0,
    "source_end_s": float(end) + 2.0,
    "selection": "two_seconds_before_cmd_vel_start_through_two_seconds_after_zero_command",
    "offline_replay": True,
    "real_robot_commands": False,
}
temporary = output.with_name(output.name + ".tmp")
temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
os.chmod(temporary, 0o644)
os.replace(temporary, output)
print(payload["source_start_s"], payload["source_end_s"])
PY
read -r source_start source_end < <(python3 - "$demo/window.json" <<'PY'
import json, pathlib, sys
d=json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
print(d["source_start_s"], d["source_end_s"])
PY
)

docker network create --internal "$network" >/dev/null
docker run -d --name "$container" --network "$network" --user 0:0 --gpus all --entrypoint bash \
  --ipc host \
  -e ROS_DOMAIN_ID="$domain" -e ROS_LOCALHOST_ONLY=0 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  -e ROS_DISTRO=jazzy -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.core/jazzy/lib:/opt/ros/jazzy/lib/aarch64-linux-gnu:/opt/ros/jazzy/lib:/isaac-sim/kit/lib \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e REAL_ROBOT_COMMANDS=false \
  -v "$root:/project" -v "$root:/workspace" -v "$overlay:/workspace/x2-overlay-v3:ro" \
  -v "$root/isaac-cache/kit:/isaac-sim/kit/cache" \
  "$engine_image" -lc "
    source /opt/ros/jazzy/setup.bash
    source /project/geniesim-build-effort/install/setup.bash
    export LD_LIBRARY_PATH=/workspace/third_party/ovrtx-python-0.3/ovrtx/bin:\${LD_LIBRARY_PATH:-}
    exec /project/geniesim-build-effort/install/genie_sim_render/lib/genie_sim_render/genie_sim_render_node \
      --ros-args -p stage_manifest:='$manifest' -p render_fps:=30.0 \
      -p ovrtx_root:=/workspace/third_party/ovrtx-python-0.3/ovrtx/bin
  " >/dev/null

ready=0
for _ in $(seq 1 600); do
  docker logs "$container" >"$demo/render.log" 2>&1 || true
  if grep -Eq 'Render node initialized|render node ready' "$demo/render.log"; then ready=1; break; fi
  if [[ "$(docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null || true)" != true ]]; then break; fi
  sleep 0.5
done
if [[ $ready -ne 1 ]] || ! grep -Fq 'robot_root_pose applied to /x2' "$demo/render.log"; then
  echo "offline renderer did not become ready with the X2 root pose" >&2
  exit 4
fi

# The renderer needs Isaac's ROS bridge first, while system Python must load the
# matching /opt/ros Jazzy Python type-support libraries first.  Scope this
# ordering to the read-only replay process so the renderer ABI is unchanged.
docker exec \
  -e LD_LIBRARY_PATH=/opt/ros/jazzy/lib/aarch64-linux-gnu:/opt/ros/jazzy/lib:/isaac-sim/exts/isaacsim.ros2.core/jazzy/lib:/isaac-sim/kit/lib \
  "$container" bash -lc "
  source /opt/ros/jazzy/setup.bash
  exec python3 /project/admin/spark/replay-x2-geniesim-render-state.py \
    --trace '/project/$source_rel/render-state.jsonl' \
    --trace-receipt '/project/$source_rel/render-state-receipt.json' \
    --source-start-s '$source_start' --source-end-s '$source_end' \
    --fps 30 --minimum-frames 180 --frame-timeout-s 15 --wall-timeout-s 1200 \
    --output '/project/${demo#"$root/"}/x2-gait2-physx-offline-replay-30fps.mp4' \
    --receipt '/project/${demo#"$root/"}/replay-receipt.json'
" >"$demo/replay.log" 2>&1

python3 - "$source_dir" "$demo" "$engine_image" "$domain" <<'PY'
import hashlib, json, os, pathlib, sys, time
source, demo = map(pathlib.Path, sys.argv[1:3])
image, domain = sys.argv[3], int(sys.argv[4])
replay = json.loads((demo / "replay-receipt.json").read_text(encoding="utf-8"))
if replay.get("status") != "SUCCEEDED" or replay.get("real_robot_commands") is not False:
    raise SystemExit("offline replay receipt did not pass")
video = demo / "x2-gait2-physx-offline-replay-30fps.mp4"
if not video.is_file() or video.stat().st_size <= 0:
    raise SystemExit("offline replay video is missing")
def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()
payload = {
    "schema": "sceneforge/x2-geniesim-render-demo/v1",
    "status": "PASSED",
    "source_physics_result": str(source / "result.json"),
    "source_physics_result_sha256": digest(source / "result.json"),
    "source_physics_authority": "genie_sim_isaac_physx",
    "replay_physics_authority": "none_state_only_renderer",
    "renderer_advances_physics": False,
    "engine_image": image,
    "ros_domain_id": domain,
    "video": {"path": str(video), "size": video.stat().st_size, "sha256": digest(video)},
    "replay_receipt_sha256": digest(demo / "replay-receipt.json"),
    "qualification": False,
    "real_robot_commands": False,
    "finished_at_unix": time.time(),
}
temporary = demo / "result.json.tmp"
temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
os.chmod(temporary, 0o644)
os.replace(temporary, demo / "result.json")
print(demo / "result.json")
PY

trap - EXIT INT TERM
cleanup
