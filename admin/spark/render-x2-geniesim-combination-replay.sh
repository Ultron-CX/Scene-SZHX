#!/usr/bin/env bash
set -euo pipefail

# Render a frozen straight->left->right PhysX lifecycle.  Both helper
# containers are presentation-only: neither launches a physics-engine process
# nor publishes robot commands.

if [[ $# -ne 1 ]]; then
  echo "usage: $0 <successful-x2-combination-run-directory>" >&2
  exit 2
fi

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)
source_dir=$(realpath "$1")
case "$source_dir" in
  "$root"/runs/x2-geniesim-cold-combination-*) ;;
  "$root"/evidence/x2-geniesim-cold-combination-*) ;;
  *)
    echo "source must be a named X2 cold-combination run under $root/runs (or a compatible frozen copy under evidence)" >&2
    exit 2
    ;;
esac
if [[ "${REAL_ROBOT_COMMANDS:-false}" != false ]]; then
  echo "real robot commands are forbidden" >&2
  exit 2
fi
if docker ps --format '{{.Names}} {{.Image}}' | grep -Eiq 'x2-geniesim|genie.?sim'; then
  echo "refusing offline replay while another Genie Sim container is active" >&2
  exit 2
fi

for required in \
  combination-runtime-result.json state-command-trace.json contact-trace.json \
  elastic-band-telemetry.jsonl lifecycle-receipt.json runtime-manifest.json \
  runtime-frozen-index.json; do
  [[ -s "$source_dir/$required" ]] || {
    echo "missing source artifact: $required" >&2
    exit 2
  }
done

stamp=$(date -u +%Y%m%dT%H%M%SZ)
demo="$root/evidence/x2-combination-render-$stamp"
test ! -e "$demo" || { echo "refusing to overwrite $demo" >&2; exit 2; }
mkdir "$demo"
chmod 0777 "$demo"

source_rel=${source_dir#"$root/"}
engine_image=${GENIESIM_IMAGE:-roboease/geniesim-isaac6-jazzy-arm64:20260818}
video_image=${VIDEO_ENCODER_IMAGE:-roboease/x2-video-recorder-humble-arm64:20260818}
overlay="$root/assets/x2-overlay-v3"
ovrtx_cache="$root/third_party/ovrtx-python-0.3/ovrtx/bin/cache"
manifest_host="$root/assets/x2-floating-dev-v1/manifest.json"
manifest=/project/assets/x2-floating-dev-v1/manifest.json
urdf_host="$root/third_party/x2-release-linux/install/x1_sim/share/x1_sim/model/x2_ultra.urdf"
urdf=/project/third_party/x2-release-linux/install/x1_sim/share/x1_sim/model/x2_ultra.urdf
[[ -f "$manifest_host" && -f "$urdf_host" ]] || {
  echo "X2 manifest or URDF is missing" >&2
  exit 2
}
[[ -d "$ovrtx_cache/shadercache" && -d "$ovrtx_cache/DerivedDataCache" ]] || {
  echo "the frozen OVRTX shader/cache tree is missing" >&2
  exit 2
}
[[ -n "$(find "$ovrtx_cache/shadercache" -type f -print -quit)" \
   && -n "$(find "$ovrtx_cache/DerivedDataCache" -type f -print -quit)" ]] || {
  echo "the frozen OVRTX shader/cache tree is empty" >&2
  exit 2
}
docker image inspect "$engine_image" >/dev/null
docker image inspect "$video_image" >/dev/null
engine_image_id=$(docker image inspect -f '{{.Id}}' "$engine_image")
video_image_id=$(docker image inspect -f '{{.Id}}' "$video_image")
command -v flock >/dev/null || { echo "flock is required" >&2; exit 2; }

# One renderer owns the shared shader cache at a time.  Keep the advisory lock
# outside the root-owned cache, then test the exact nested bind from an
# isolated root container.  This catches a misleading preflight success caused
# by host ACLs while preserving the rest of /workspace as read-only.
exec 9>"$root/evidence/.sceneforge-ovrtx-render.lock"
flock -n 9 || { echo "the OVRTX shader cache is in use" >&2; exit 2; }
docker run --rm --network none --read-only --user 0:0 \
  --tmpfs /tmp:rw,noexec,nosuid,size=64m \
  -v "$root:/workspace:ro" \
  -v "$ovrtx_cache:/workspace/third_party/ovrtx-python-0.3/ovrtx/bin/cache:rw" \
  --entrypoint bash "$engine_image" -lc '
    set -euo pipefail
    probe=/workspace/third_party/ovrtx-python-0.3/ovrtx/bin/cache/.sceneforge-write-probe-$$
    trap '\''rm -f -- "$probe"'\'' EXIT
    printf "probe\n" >"$probe"
    python3 - "$probe" <<'\''PY'\''
import os, pathlib, sys
with pathlib.Path(sys.argv[1]).open("r+b") as stream:
    os.fsync(stream.fileno())
PY
  ' >/dev/null

domain=$((80 + 10#${stamp:9:4} % 100))
renderer="x2-combination-offline-render-${stamp,,}"
network="x2-combination-offline-render-${stamp,,}"
converter="x2-combination-offline-convert-${stamp,,}"
encoder="x2-combination-offline-encode-${stamp,,}"
video="$demo/x2-combination-physx-offline-replay-30fps.mp4"
intermediate="$demo/x2-combination-physx-offline-replay-intermediate-mp4v.mp4"

python3 - "$ovrtx_cache" "$demo/cache-before.json" <<'PY'
import json, os, pathlib, stat, sys
root, output = map(pathlib.Path, sys.argv[1:3])
errors = []
file_count = total_bytes = 0
for directory, _subdirs, names in os.walk(root, followlinks=False, onerror=lambda error: errors.append(str(error))):
    for name in names:
        path = pathlib.Path(directory) / name
        try:
            info = path.stat(follow_symlinks=False)
        except OSError as error:
            errors.append(str(error)); continue
        if stat.S_ISREG(info.st_mode):
            file_count += 1; total_bytes += info.st_size
payload = {"path": str(root), "file_count": file_count, "total_bytes": total_bytes, "inaccessible_entry_count": len(errors)}
output.write_text(json.dumps(payload, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
with output.open("rb") as stream: os.fsync(stream.fileno())
PY

cleanup() {
  set +e
  docker logs "$renderer" >"$demo/render.log" 2>&1 || true
  docker stop --time 30 "$renderer" >/dev/null 2>&1 || true
  docker rm "$renderer" >/dev/null 2>&1 || true
  docker rm -f "$converter" >/dev/null 2>&1 || true
  docker rm -f "$encoder" >/dev/null 2>&1 || true
  docker network rm "$network" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

# Use the frozen Isaac image only as a Python/Numpy runtime.  Network, GPU and
# physics are absent from this conversion step.
docker run --rm --name "$converter" --network none --read-only --user 0:0 \
  --tmpfs /tmp:rw,noexec,nosuid,size=512m \
  -e REAL_ROBOT_COMMANDS=false \
  -v "$root:/project:ro" -v "$demo:/evidence:rw" \
  --entrypoint bash "$engine_image" -lc \
  "/isaac-sim/python.sh /project/admin/spark/convert-x2-combination-trace-to-render-state.py \
    --run-dir '/project/$source_rel' --urdf '$urdf' --manifest '$manifest' \
    --fps 30 --output /evidence/render-state.jsonl \
    --receipt /evidence/render-state-receipt.json" \
  >"$demo/convert.log" 2>&1

python3 - "$demo/render-state-receipt.json" "$source_dir" <<'PY'
import hashlib, json, pathlib, sys
receipt = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
source = pathlib.Path(sys.argv[2])
def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()
required = {
    "schema": "sceneforge/x2-geniesim-render-state-receipt/v1",
    "status": "SUCCEEDED",
    "authoritative_physics_source": "geniesim_isaac_physx",
    "source_kind": "single_lifecycle_straight_left_right_combination",
    "overlay_schema": "sceneforge/x2-combination-video-overlay/v1",
    "target_sample_rate_hz": 30.0,
    "read_only": True,
    "published_topics": [],
    "real_robot_commands": False,
}
for name, value in required.items():
    if receipt.get(name) != value: raise SystemExit(f"render trace receipt {name} mismatch")
if receipt.get("source_result_sha256") != digest(source / "combination-runtime-result.json"):
    raise SystemExit("render trace is not bound to the combination result")
if receipt.get("source_trace_sha256") != digest(source / "state-command-trace.json"):
    raise SystemExit("render trace is not bound to the state trace")
if receipt.get("source_frozen_index_sha256") != digest(source / "runtime-frozen-index.json"):
    raise SystemExit("render trace is not bound to the frozen runtime index")
PY

docker network create --internal "$network" >/dev/null
docker run -d --name "$renderer" --network "$network" --user 0:0 --gpus all \
  --entrypoint bash --ipc host \
  -e ROS_DOMAIN_ID="$domain" -e ROS_LOCALHOST_ONLY=0 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  -e ROS_DISTRO=jazzy -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.core/jazzy/lib:/opt/ros/jazzy/lib/aarch64-linux-gnu:/opt/ros/jazzy/lib:/isaac-sim/kit/lib \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e REAL_ROBOT_COMMANDS=false \
  -v "$root:/project:ro" -v "$root:/workspace:ro" \
  -v "$demo:/evidence:rw" -v "$overlay:/workspace/x2-overlay-v3:ro" \
  -v "$root/isaac-cache/kit:/isaac-sim/kit/cache" \
  -v "$ovrtx_cache:/workspace/third_party/ovrtx-python-0.3/ovrtx/bin/cache:rw" \
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
  docker logs "$renderer" >"$demo/render.log" 2>&1 || true
  if grep -Eq 'Render node initialized|render node ready' "$demo/render.log"; then
    ready=1
    break
  fi
  if [[ "$(docker inspect -f '{{.State.Running}}' "$renderer" 2>/dev/null || true)" != true ]]; then
    break
  fi
  sleep 0.5
done
if [[ $ready -ne 1 ]] || ! grep -Fq 'robot_root_pose applied to /x2' "$demo/render.log"; then
  echo "offline renderer did not become ready with the X2 root pose" >&2
  exit 4
fi

docker exec \
  -e LD_LIBRARY_PATH=/opt/ros/jazzy/lib/aarch64-linux-gnu:/opt/ros/jazzy/lib:/isaac-sim/exts/isaacsim.ros2.core/jazzy/lib:/isaac-sim/kit/lib \
  "$renderer" bash -lc "
  source /opt/ros/jazzy/setup.bash
  exec python3 /project/admin/spark/replay-x2-geniesim-render-state.py \
    --trace /evidence/render-state.jsonl \
    --trace-receipt /evidence/render-state-receipt.json \
    --fps 30 --minimum-frames 300 --warmup-timeout-s 300 \
    --frame-timeout-s 15 --wall-timeout-s 3600 \
    --require-overlay --offset-x -2.2 --offset-y -2.2 --offset-z 1.45 \
    --target-x 0.15 --target-y 0.0 --target-z 0.68 \
    --output /evidence/x2-combination-physx-offline-replay-intermediate-mp4v.mp4 \
    --receipt /evidence/replay-receipt.json
  " >"$demo/replay.log" 2>&1

# The renderer image deliberately has no ffmpeg binary.  It records an
# immutable OpenCV/mp4v intermediate; a separate presentation-only image then
# produces and independently probes the browser-compatible H.264 deliverable.
frame_count=$(python3 - "$demo/replay-receipt.json" <<'PY'
import json, pathlib, sys
value = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8")).get("frame_count")
if not isinstance(value, int) or value < 3:
    raise SystemExit("replay frame count is invalid for QA extraction")
print(value)
PY
)
middle_frame=$((frame_count / 2))
last_frame=$((frame_count - 1))

docker run --rm --name "$encoder" --network none --read-only \
  --tmpfs /tmp:rw,noexec,nosuid,size=512m \
  -e REAL_ROBOT_COMMANDS=false -v "$demo:/evidence:rw" \
  --entrypoint bash "$video_image" -lc "
    set -euo pipefail
    ffmpeg -hide_banner -loglevel warning -y \
      -i /evidence/x2-combination-physx-offline-replay-intermediate-mp4v.mp4 \
      -map 0:v:0 -an -sn -dn -c:v libx264 -preset veryfast -crf 18 \
      -pix_fmt yuv420p -r 30 -movflags +faststart \
      /evidence/.x2-combination-physx-offline-replay-30fps.partial.mp4
    mv /evidence/.x2-combination-physx-offline-replay-30fps.partial.mp4 \
      /evidence/x2-combination-physx-offline-replay-30fps.mp4
    ffprobe -v error -count_frames -select_streams v:0 \
      -show_entries stream=codec_name,pix_fmt,width,height,avg_frame_rate,nb_read_frames,duration \
      -of json /evidence/x2-combination-physx-offline-replay-30fps.mp4 \
      >/evidence/video-probe.json
    timeout 120 ffmpeg -hide_banner -loglevel error -y \
      -i /evidence/x2-combination-physx-offline-replay-30fps.mp4 \
      -map 0:v:0 -an -sn -dn -vf 'select=eq(n\\,0)' -vsync vfr \
      -frames:v 1 -update 1 /evidence/qa-first.png
    timeout 120 ffmpeg -hide_banner -loglevel error -y \
      -i /evidence/x2-combination-physx-offline-replay-30fps.mp4 \
      -map 0:v:0 -an -sn -dn -vf 'select=eq(n\\,$middle_frame)' -vsync vfr \
      -frames:v 1 -update 1 /evidence/qa-middle.png
    timeout 120 ffmpeg -hide_banner -loglevel error -y \
      -i /evidence/x2-combination-physx-offline-replay-30fps.mp4 \
      -map 0:v:0 -an -sn -dn -vf 'select=eq(n\\,$last_frame)' -vsync vfr \
      -frames:v 1 -update 1 /evidence/qa-last.png
  " >"$demo/encode.log" 2>&1

for label in first middle last; do
  [[ -s "$demo/qa-$label.png" ]] || {
    echo "failed to extract QA $label frame" >&2
    exit 5
  }
done

python3 - "$ovrtx_cache" "$demo/cache-after.json" <<'PY'
import json, os, pathlib, stat, sys
root, output = map(pathlib.Path, sys.argv[1:3])
errors = []
file_count = total_bytes = 0
for directory, _subdirs, names in os.walk(root, followlinks=False, onerror=lambda error: errors.append(str(error))):
    for name in names:
        path = pathlib.Path(directory) / name
        try:
            info = path.stat(follow_symlinks=False)
        except OSError as error:
            errors.append(str(error)); continue
        if stat.S_ISREG(info.st_mode):
            file_count += 1; total_bytes += info.st_size
payload = {"path": str(root), "file_count": file_count, "total_bytes": total_bytes, "inaccessible_entry_count": len(errors)}
output.write_text(json.dumps(payload, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
with output.open("rb") as stream: os.fsync(stream.fileno())
PY

python3 - "$source_dir" "$demo" "$video" "$engine_image" "$video_image" \
  "$engine_image_id" "$video_image_id" "$domain" <<'PY'
from fractions import Fraction
import hashlib, json, os, pathlib, sys, time
source, demo, video = map(pathlib.Path, sys.argv[1:4])
image, video_image = sys.argv[4], sys.argv[5]
image_id, video_image_id, domain = sys.argv[6], sys.argv[7], int(sys.argv[8])
def load(path): return json.loads(path.read_text(encoding="utf-8"))
def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()
replay = load(demo / "replay-receipt.json")
trace_receipt = load(demo / "render-state-receipt.json")
required = {
    "status": "SUCCEEDED", "replay_physics_authority": "none_state_only_renderer",
    "renderer_advances_physics": False, "output_fps": 30.0,
    "encoder_backend": "opencv_ffmpeg_mp4v", "overlay_required": True,
    "overlay_schema": "sceneforge/x2-combination-video-overlay/v1",
    "overlay_position": "top_right", "real_robot_commands": False,
}
for name, value in required.items():
    if replay.get(name) != value: raise SystemExit(f"replay receipt {name} mismatch")
if replay.get("missing_samples") != []:
    raise SystemExit("replay contains missing frames")
if replay.get("frame_count", 0) < 300 or replay.get("overlay_frame_count") != replay.get("frame_count"):
    raise SystemExit("not every replay frame has the qualification HUD")
if replay.get("video_width", 0) < 1280 or replay.get("video_height", 0) < 720:
    raise SystemExit("qualification video resolution is below 1280x720")
if not video.is_file() or video.stat().st_size <= 0:
    raise SystemExit("qualification video is missing")
probe = load(demo / "video-probe.json")
streams = probe.get("streams")
if not isinstance(streams, list) or len(streams) != 1:
    raise SystemExit("qualification video probe must contain one video stream")
stream = streams[0]
if stream.get("codec_name") != "h264" or stream.get("pix_fmt") != "yuv420p":
    raise SystemExit("qualification video is not H.264/yuv420p")
if int(stream.get("width", 0)) < 1280 or int(stream.get("height", 0)) < 720:
    raise SystemExit("qualification video probe resolution is below 1280x720")
if Fraction(stream.get("avg_frame_rate", "0/1")) != Fraction(30, 1):
    raise SystemExit("qualification video probe frame rate is not 30 fps")
if int(stream.get("nb_read_frames", -1)) != replay["frame_count"]:
    raise SystemExit("qualification video probe frame count mismatch")
duration_s = float(stream.get("duration", "nan"))
expected_duration_s = replay["frame_count"] / 30.0
if abs(duration_s - expected_duration_s) > 0.05:
    raise SystemExit("qualification video duration indicates timing distortion")
qa_indices = (0, replay["frame_count"] // 2, replay["frame_count"] - 1)
qa_frames = []
for label, frame_index in zip(("first", "middle", "last"), qa_indices):
    path = demo / f"qa-{label}.png"
    if not path.is_file() or path.stat().st_size <= 8 or path.read_bytes()[:8] != b"\x89PNG\r\n\x1a\n":
        raise SystemExit(f"QA {label} frame is missing or not PNG")
    qa_frames.append({
        "label": label, "frame_index": frame_index, "path": str(path),
        "size": path.stat().st_size, "sha256": digest(path),
    })
payload = {
    "schema": "sceneforge/x2-combination-video-receipt/v1",
    "status": "PASSED",
    "source_runtime_sha256": digest(source / "combination-runtime-result.json"),
    "source_state_command_trace_sha256": digest(source / "state-command-trace.json"),
    "source_contact_trace_sha256": digest(source / "contact-trace.json"),
    "source_elastic_band_telemetry_sha256": digest(source / "elastic-band-telemetry.jsonl"),
    "source_frozen_index_sha256": digest(source / "runtime-frozen-index.json"),
    "source_physics_authority": "genie_sim_isaac_physx",
    "replay_physics_authority": "none_state_only_renderer",
    "renderer_advances_physics": False,
    "engine_image": image,
    "engine_image_id": image_id,
    "video_encoder_image": video_image,
    "video_encoder_image_id": video_image_id,
    "ros_domain_id": domain,
    "video": {
        "path": str(video), "size": video.stat().st_size,
        "sha256": digest(video), "codec": "h264", "fps": 30.0,
        "pixel_format": "yuv420p", "frame_count": replay["frame_count"],
        "duration_s": duration_s,
        "width": int(stream["width"]), "height": int(stream["height"]),
    },
    "overlay": {
        "schema": replay["overlay_schema"], "position": "top_right",
        "frame_count": replay["overlay_frame_count"],
        "fields": [
            "scenario_phase_gait", "requested_and_effective_velocity",
            "measured_velocity", "left_and_right_foot_contact", "cpg_phase",
            "suspension", "sim_time", "global_lifecycle_rtf_estimate",
        ],
    },
    "camera": {
        "mode": "robot_follow_full_body_profile_v1", "offset_xyz_m": [-2.2, -2.2, 1.45],
        "target_xyz_m": [0.15, 0.0, 0.68], "left_menu_visible": False,
        "visual_full_body_verification": "pending_user_review",
    },
    "qa_frames": qa_frames,
    "render_state_receipt_sha256": digest(demo / "render-state-receipt.json"),
    "replay_receipt_sha256": digest(demo / "replay-receipt.json"),
    "video_probe_sha256": digest(demo / "video-probe.json"),
    "cache_before_sha256": digest(demo / "cache-before.json"),
    "cache_after_sha256": digest(demo / "cache-after.json"),
    "intermediate": {
        "path": str(demo / "x2-combination-physx-offline-replay-intermediate-mp4v.mp4"),
        "size": (demo / "x2-combination-physx-offline-replay-intermediate-mp4v.mp4").stat().st_size,
        "sha256": digest(demo / "x2-combination-physx-offline-replay-intermediate-mp4v.mp4"),
        "codec": "mpeg4_mp4v",
    },
    "rtf": trace_receipt["rtf"],
    "qualification": False,
    "real_robot_commands": False,
    "finished_at_unix": time.time(),
}
temporary = demo / ".video-receipt.json.tmp"
temporary.write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
with temporary.open("rb") as stream: os.fsync(stream.fileno())
os.replace(temporary, demo / "video-receipt.json")
print(demo / "video-receipt.json")
PY

trap - EXIT INT TERM
cleanup
# Files created by the root renderer/encoder cannot be sealed by the host user.
# Use the already pinned presentation-only image to make the finished bundle
# root-owned and read-only without touching any source physics evidence.
docker run --rm --network none --read-only \
  --tmpfs /tmp:rw,noexec,nosuid,size=64m \
  -v "$demo:/evidence:rw" --entrypoint bash "$video_image" -lc '
    set -euo pipefail
    chown -R 0:0 /evidence
    find /evidence -type f -exec chmod 0444 {} +
    find /evidence -type d -exec chmod 0555 {} +
  '
printf '%s\n' "$demo"
