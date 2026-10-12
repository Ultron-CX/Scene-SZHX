#!/usr/bin/env bash
set -euo pipefail

root=${SCENEFORGE_X2_ROOT:-/opt/sceneforge/runtime}
image=${X2_MUJOCO_AUDIT_IMAGE:-roboease/geniesim-isaac6-jazzy-arm64:20260818}
model=${X2_MUJOCO_MODEL:-/project/third_party/x2-release-linux/install/x1_sim/share/x1_sim/model/scene.xml}
stamp=$(date -u +%Y%m%dT%H%M%SZ)
evidence="$root/evidence/x2-mujoco-model-audit-$stamp"

test "${REAL_ROBOT_COMMANDS:-false}" = false || { echo 'REAL_ROBOT_COMMANDS must be false' >&2; exit 2; }
test -f "$root/runtime/x2-mujoco-py/mujoco/__init__.py" || { echo 'workspace MuJoCo 3.3.5 runtime missing' >&2; exit 2; }
test ! -e "$evidence" || { echo "refusing to overwrite $evidence" >&2; exit 2; }
mkdir "$evidence"

docker run --rm --network none --read-only --tmpfs /tmp:rw,noexec,nosuid,size=64m \
  --user 0:0 \
  -e REAL_ROBOT_COMMANDS=false \
  -e PYTHONPATH=/project/src:/project/runtime/x2-mujoco-py \
  -v "$root:/project:ro" -v "$evidence:/evidence:rw" \
  --entrypoint bash "$image" -lc \
  "/isaac-sim/python.sh /project/admin/spark/x2-mujoco-model-audit.py \
    --model '$model' --receipt /evidence/result.json && chmod -R a-w /evidence"

python3 - "$evidence/result.json" <<'PY'
import json, pathlib, sys
path = pathlib.Path(sys.argv[1])
payload = json.loads(path.read_text(encoding="utf-8"))
if payload.get("status") != "PASSED": raise SystemExit("model audit failed")
if payload.get("physics_advanced") is not False: raise SystemExit("physics was advanced")
if payload.get("command_application_count") != 0: raise SystemExit("command application detected")
if payload.get("real_robot_commands") is not False: raise SystemExit("real robot flag violation")
PY
printf '%s\n' "$evidence"
