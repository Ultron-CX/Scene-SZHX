#!/usr/bin/env bash
set -euo pipefail

root="${ROBOEASE_X2_ROOT:-/workspace/third_party/x2-release}"
run_root="${ROBOEASE_RUN_ROOT:-/workspace/runs}"
run_id="x2-mujoco-arm64-$(date -u +%Y%m%dT%H%M%SZ)"
run_dir="$run_root/$run_id"
duration_s="${ROBOEASE_BASELINE_DURATION_S:-30}"

mkdir -p "$run_dir"

if [[ "${REAL_ROBOT_COMMANDS:-false}" != "false" ]]; then
  printf '%s\n' 'REAL_ROBOT_COMMANDS must be false' >&2
  exit 2
fi
if [[ ! -x "$root/run_sim.sh" ]]; then
  printf 'missing executable: %s\n' "$root/run_sim.sh" >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf '%s\n' 'ROS Humble is unavailable' >&2
  exit 2
fi

cat >"$run_dir/run-metadata.json" <<EOF
{
  "schema": "roboease-x2-mujoco-arm64-baseline/v1",
  "run_id": "$run_id",
  "real_robot_commands": false,
  "network_mode_required": "none",
  "duration_s": $duration_s,
  "status": "STARTING"
}
EOF

set +e
timeout --signal=INT --kill-after=10s "${duration_s}s" \
  xvfb-run --auto-servernum --server-args='-screen 0 1280x720x24 -nolisten tcp' \
  env \
    REAL_ROBOT_COMMANDS=false \
    ROS_LOCALHOST_ONLY=1 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    MUJOCO_GL=egl \
    LD_LIBRARY_PATH="$root/runtime_lib:$root/install/x1_rl_control/lib/x1_rl_control:${LD_LIBRARY_PATH:-}" \
    X1_INPUT_SOURCE=sdk \
    "$root/run_sim.sh" \
      --input sdk \
      enable_http_bridge:=false \
      enable_joy:=false \
      enable_digit_sdk:=false \
      publish_when_idle:=true \
      >"$run_dir/combined.log" 2>&1
status=$?
set -e

case "$status" in
  0|124|130) terminal="COMPLETED_BOUNDED" ;;
  *) terminal="FAILED" ;;
esac

if grep -Eq 'process has died|ERROR: could not initialize GLFW|Segmentation fault|terminate called|what\(\):' "$run_dir/combined.log"; then
  terminal="FAILED"
fi

python3 - "$run_dir/run-metadata.json" "$terminal" "$status" <<'PY'
import json
import os
import sys

path, terminal, status = sys.argv[1:]
with open(path, encoding="utf-8") as stream:
    result = json.load(stream)
result["status"] = terminal
result["process_exit_code"] = int(status)
result["real_robot_commands"] = False
tmp = path + ".tmp"
with open(tmp, "w", encoding="utf-8") as stream:
    json.dump(result, stream, ensure_ascii=False, indent=2, sort_keys=True)
    stream.write("\n")
    stream.flush()
    os.fsync(stream.fileno())
os.replace(tmp, path)
PY

printf '%s\n' "$run_dir"
[[ "$terminal" == "COMPLETED_BOUNDED" ]]
