#!/usr/bin/env bash
set -euo pipefail

root=${SCENEFORGE_X2_ROOT:-/opt/sceneforge/runtime}
image=${X2_MUJOCO_IMAGE:-roboease/geniesim-isaac6-jazzy-arm64:20260818}
model=/project/third_party/x2-release-linux/install/x1_sim/share/x1_sim/model/scene.xml
stand_model=/project/third_party/x2-release-linux/install/x1_rl_control/share/x1_rl_control/policy/cpgtelecon_v3_fix.onnx
walk_model=/project/third_party/x2-release-linux/install/x1_rl_control/share/x1_rl_control/policy/cpgwalkrun_v25_v2.onnx
joint_limit_urdf=/project/third_party/x2-release-linux/install/x1_sim/share/x1_sim/model/x2_ultra.urdf
motion_case=${X2_MOTION_CASE:-straight}
command_x=${X2_COMMAND_X_MPS:-0.3}
command_y=${X2_COMMAND_Y_MPS:-0.0}
command_yaw=${X2_COMMAND_YAW_RPS:-0.0}
supported_stand=${X2_SUPPORTED_STAND_S:-3}
band_ramp=${X2_BAND_RAMP_S:-5}
unsupported_stand=${X2_UNSUPPORTED_STAND_S:-3}
walk_s=${X2_WALK_S:-8}
stop_s=${X2_STOP_S:-8}
stamp=$(date -u +%Y%m%dT%H%M%SZ)
if [[ ! "$motion_case" =~ ^[a-z0-9_-]+$ ]]; then
  echo "invalid X2_MOTION_CASE" >&2
  exit 2
fi
evidence="$root/evidence/x2-mujoco-${motion_case}-$stamp"

test "${REAL_ROBOT_COMMANDS:-false}" = false || { echo 'REAL_ROBOT_COMMANDS must be false' >&2; exit 2; }
test -f "$root/runtime/x2-mujoco-py/mujoco/__init__.py" || { echo 'MuJoCo runtime missing' >&2; exit 2; }
test -f "$root/runtime/x2-shadow-ort/onnxruntime/__init__.py" || { echo 'ONNX Runtime missing' >&2; exit 2; }
if docker ps --format '{{.Names}}' | grep -Eq 'x2-geniesim|x2-mujoco|genie-sim'; then
  echo 'another X2 simulation authority is active' >&2
  exit 2
fi
test ! -e "$evidence" || { echo "refusing to overwrite $evidence" >&2; exit 2; }
mkdir "$evidence"

set +e
timeout --signal=TERM --kill-after=30s 600s \
docker run --rm --name "x2-mujoco-motion-$stamp" --network none --read-only \
  --user 0:0 --tmpfs /tmp:rw,noexec,nosuid,size=512m \
  -e REAL_ROBOT_COMMANDS=false \
  -e PYTHONPATH=/project/src:/project/runtime/x2-mujoco-py:/project/runtime/x2-shadow-ort \
  -v "$root:/project:ro" -v "$evidence:/evidence:rw" \
  --entrypoint bash "$image" -lc \
  "/isaac-sim/python.sh /project/admin/spark/x2-mujoco-motion-baseline.py \
    --model '$model' --stand-model '$stand_model' --walk-model '$walk_model' \
    --joint-limit-urdf '$joint_limit_urdf' \
    --result /evidence/result.json --trace /evidence/trace.json \
    --supported-stand-s '$supported_stand' --band-ramp-s '$band_ramp' \
    --unsupported-stand-s '$unsupported_stand' \
    --walk-s '$walk_s' --stop-s '$stop_s' --command-x-mps '$command_x' \
    --command-y-mps '$command_y' --command-yaw-rps '$command_yaw' --disable-ramp; \
    status=\$?; chmod -R a-w /evidence; exit \$status" \
  >"$evidence/container.log" 2>&1
status=$?
set -e

if [[ ! -s "$evidence/result.json" ]]; then
  chmod a-w "$evidence/container.log" 2>/dev/null || true
  printf 'motion baseline did not produce result.json (exit %s)\n' "$status" >&2
  exit 1
fi
python3 - "$evidence/result.json" <<'PY'
import json,sys
p=json.load(open(sys.argv[1]))
if p.get("real_robot_commands") is not False: raise SystemExit("real robot flag violation")
if p.get("physics_authority") != "google_mujoco_3_3_5": raise SystemExit("authority mismatch")
if p.get("published_command_topics") != []: raise SystemExit("command transport detected")
print(json.dumps({"status":p["status"],"decision":p["decision"],"metrics":p["metrics"]},indent=2))
PY
printf '%s\n' "$evidence"
exit "$status"
