#!/usr/bin/env bash
set -euo pipefail

root=${SCENEFORGE_X2_ROOT:-/opt/sceneforge/runtime}
domain=${ROS_DOMAIN_ID:-72}
duration=${X2_CONTROL_GATE_DURATION_S:-30}
control_mode=${X2_CONTROL_MODE:-idle}
walk_speed=${X2_WALK_SPEED_MPS:-0.3}
walk_lateral_speed=${X2_WALK_LATERAL_SPEED_MPS:-0.0}
walk_yaw_rate=${X2_WALK_YAW_RATE_RPS:-0.0}
walk_duration=${X2_WALK_DURATION_S:-8}
walk_stop_settle_s=${X2_WALK_STOP_SETTLE_S:-2}
walk_deceleration_speed=${X2_WALK_DECELERATION_SPEED_MPS:-0.0}
walk_deceleration_duration=${X2_WALK_DECELERATION_DURATION_S:-0.0}
gait1_to_gait2_speed=${X2_GAIT1_TO_GAIT2_SPEED_MPS:--1.0}
gait2_to_gait1_speed=${X2_GAIT2_TO_GAIT1_SPEED_MPS:--1.0}
gait_period_transition_tau=${X2_GAIT_PERIOD_TRANSITION_TAU_S:-0.25}
heading_hold_enabled=${X2_HEADING_HOLD_ENABLED:-false}
heading_hold_kp=${X2_HEADING_HOLD_KP:-1.0}
heading_hold_max_correction=${X2_HEADING_HOLD_MAX_CORRECTION_RPS:-0.3}
stand_settle_s=${X2_STAND_SETTLE_S:-3}
stand_default_settle_s=${X2_STAND_DEFAULT_SETTLE_S:-5}
effort_gain_scale=${X2_EFFORT_GAIN_SCALE:-1.0}
record_video=${X2_RECORD_VIDEO:-false}
record_state_trace=${X2_RECORD_STATE_TRACE:-false}
base_mode=${X2_BASE_MODE:-fixed}
physics_hz=${X2_PHYSICS_HZ:-500}
elastic_band_enabled=${X2_ELASTIC_BAND_ENABLED:-false}
elastic_band_ramp_s=${X2_ELASTIC_BAND_RAMP_S:-5}
elastic_band_unsupported_hold_s=${X2_ELASTIC_BAND_UNSUPPORTED_HOLD_S:-10}
elastic_band_max_force_n=${X2_ELASTIC_BAND_MAX_FORCE_N:-800}
clock_freeze_probe_enabled=${X2_CLOCK_FREEZE_PROBE_ENABLED:-false}
clock_freeze_duration_s=${X2_CLOCK_FREEZE_DURATION_S:-2.0}
render_body_tf_hz=${X2_RENDER_BODY_TF_HZ:-30.0}
adapter_publish_mode=${X2_ADAPTER_PUBLISH_MODE:-state_driven}
isaac_viewport_updates=${X2_ISAAC_VIEWPORT_UPDATES:-0}
adapter_command_mode=position
if [[ "$control_mode" != idle && "$control_mode" != stand && "$control_mode" != stand_default && "$control_mode" != walk ]]; then
  echo "X2_CONTROL_MODE must be idle, stand, stand_default, or walk" >&2
  exit 2
fi
if [[ "$base_mode" != fixed && "$base_mode" != floating ]]; then
  echo "X2_BASE_MODE must be fixed or floating" >&2
  exit 2
fi
if [[ "$base_mode" == floating ]]; then adapter_command_mode=${X2_ADAPTER_COMMAND_MODE:-effort_pd}; fi
if [[ "$adapter_command_mode" != position && "$adapter_command_mode" != effort_pd ]]; then
  echo "X2_ADAPTER_COMMAND_MODE must be position or effort_pd" >&2
  exit 2
fi
if [[ "$elastic_band_enabled" != true && "$elastic_band_enabled" != false ]]; then
  echo "X2_ELASTIC_BAND_ENABLED must be true or false" >&2
  exit 2
fi
if [[ "$heading_hold_enabled" != true && "$heading_hold_enabled" != false ]]; then
  echo "X2_HEADING_HOLD_ENABLED must be true or false" >&2
  exit 2
fi
if [[ "$record_state_trace" != true && "$record_state_trace" != false ]]; then
  echo "X2_RECORD_STATE_TRACE must be true or false" >&2
  exit 2
fi
if [[ "$clock_freeze_probe_enabled" != true && "$clock_freeze_probe_enabled" != false ]]; then
  echo "X2_CLOCK_FREEZE_PROBE_ENABLED must be true or false" >&2
  exit 2
fi
if [[ "$clock_freeze_probe_enabled" == true && ( "$base_mode" != floating || ( "$control_mode" != stand_default && "$control_mode" != walk ) ) ]]; then
  echo "the controller clock-freeze probe requires floating stand_default or walk mode" >&2
  exit 2
fi
if [[ "$adapter_publish_mode" != timer && "$adapter_publish_mode" != state_driven ]]; then
  echo "X2_ADAPTER_PUBLISH_MODE must be timer or state_driven" >&2
  exit 2
fi
if [[ "$isaac_viewport_updates" != 0 && "$isaac_viewport_updates" != 1 ]]; then
  echo "X2_ISAAC_VIEWPORT_UPDATES must be 0 or 1" >&2
  exit 2
fi
if [[ "$heading_hold_enabled" == true && "$control_mode" != walk ]]; then
  echo "heading hold is only valid for the walk/run gate" >&2
  exit 2
fi
if [[ "$heading_hold_enabled" == true ]] && python3 - "$walk_yaw_rate" <<'PY'
import sys
raise SystemExit(0 if abs(float(sys.argv[1])) > 1e-9 else 1)
PY
then
  echo "heading hold is only valid for a zero-yaw straight command" >&2
  exit 2
fi
heading_hold_flag=""
if [[ "$heading_hold_enabled" == true ]]; then heading_hold_flag="--heading-hold"; fi
if [[ "$elastic_band_enabled" == true && "$base_mode" != floating ]]; then
  echo "the PhysX elastic band is valid only for floating-base runs" >&2
  exit 2
fi
if [[ "$elastic_band_enabled" == true && "$control_mode" != stand_default && "$control_mode" != walk ]]; then
  echo "the PhysX elastic band is limited to stand_default or walk transition tests" >&2
  exit 2
fi
if [[ "$control_mode" == walk && ( "$base_mode" != floating || "$elastic_band_enabled" != true ) ]]; then
  echo "the current walk gate requires floating PhysX with the auditable elastic-band transition" >&2
  exit 2
fi
use_staged_deceleration=false
if [[ "$control_mode" == walk ]]; then
  python3 - "$walk_speed" "$walk_deceleration_speed" "$walk_deceleration_duration" <<'PY'
import math, sys
speed, deceleration_speed, duration = map(float, sys.argv[1:])
if not all(math.isfinite(value) for value in (speed, deceleration_speed, duration)):
    raise SystemExit("walking and deceleration values must be finite")
if duration < 0.0:
    raise SystemExit("X2_WALK_DECELERATION_DURATION_S must be non-negative")
if duration > 0.0:
    if abs(deceleration_speed) < 0.3:
        raise SystemExit("staged deceleration must retain the vendor minimum 0.3 m/s command")
    if speed * deceleration_speed <= 0.0 or abs(deceleration_speed) >= abs(speed):
        raise SystemExit("staged deceleration speed must have the same sign and smaller magnitude")
PY
  if python3 - "$walk_deceleration_duration" <<'PY'
import sys
raise SystemExit(0 if float(sys.argv[1]) > 0.0 else 1)
PY
  then
    use_staged_deceleration=true
  fi
fi
stamp=$(date -u +%Y%m%dT%H%M%SZ)
run_id="x2-geniesim-fixed-controller-${stamp}"
run_dir="$root/runs/$run_id"
evidence_dir="$root/evidence/$run_id"
engine_container="x2-geniesim-engine-${stamp,,}"
control_container="x2-geniesim-control-${stamp,,}"
render_container="x2-geniesim-render-${stamp,,}"
recorder_container="x2-geniesim-recorder-${stamp,,}"
state_recorder_container="x2-geniesim-state-recorder-${stamp,,}"
ros_network="x2-geniesim-ros-${stamp,,}"
engine_image=${GENIESIM_IMAGE:-roboease/geniesim-isaac6-jazzy-arm64:20260818}
control_image=${X2_CONTROL_IMAGE:-roboease/x2-control-humble-arm64:20260818-v4}
overlay="$root/assets/x2-overlay-v3"
if [[ "$base_mode" == fixed ]]; then
  manifest=${X2_GENIESIM_MANIFEST:-/workspace/x2-overlay-v3/usd-fixed-base/manifest.json}
  # The older fixed-base build is a symlink-install rooted at /workspace and
  # is not relocatable into this runner's /project mount.  Use the frozen
  # copy-install that also backs the accepted in-process PhysX gates.
  geniesim_build=${X2_GENIESIM_BUILD_ROOT:-/project/geniesim-build-effort}
else
  manifest=${X2_GENIESIM_MANIFEST:-/project/assets/x2-floating-dev-v1/manifest.json}
  geniesim_build=${X2_GENIESIM_BUILD_ROOT:-/project/geniesim-build-effort}
fi
if [[ "$base_mode" == fixed ]]; then
  physics=${X2_PHYSICS_PARAMS:-$geniesim_build/install/genie_sim_engine/share/genie_sim_engine/config/physics_params.yaml}
else
  physics=${X2_PHYSICS_PARAMS:-/project/assets/x2-floating-dev-v1/physics_params_x2.yaml}
fi
engine=$geniesim_build/install/genie_sim_engine/lib/genie_sim_engine/genie_sim_engine_isaacsim.py

mkdir -p "$run_dir" "$root/evidence"
# The official Isaac image runs as uid/gid 1234. Grant that runtime identity
# write access only to this new, run-scoped directory so it can atomically
# publish elastic-band status without broadening permissions on the project.
if command -v setfacl >/dev/null 2>&1; then
  setfacl -m u:1234:rwx "$run_dir"
else
  chmod 0777 "$run_dir"
fi
if [[ -e "$evidence_dir" ]]; then
  echo "refusing to overwrite evidence: $evidence_dir" >&2
  exit 2
fi

controller_config_source="$root/third_party/x2-release-linux/install/x1_rl_control/share/x1_rl_control/cfg/rl/cpgwalk_config.yaml"
controller_config_evidence_path="$controller_config_source"
controller_config_mount=()
if python3 - "$gait1_to_gait2_speed" "$gait2_to_gait1_speed" <<'PY'
import sys
enter, leave = map(float, sys.argv[1:])
raise SystemExit(0 if enter >= 0.0 or leave >= 0.0 else 1)
PY
then
  python3 - "$controller_config_source" "$run_dir/cpgwalk_config.override.yaml" \
    "$gait1_to_gait2_speed" "$gait2_to_gait1_speed" "$gait_period_transition_tau" <<'PY'
import math, os, pathlib, re, sys
source, output = map(pathlib.Path, sys.argv[1:3])
enter, leave, tau = map(float, sys.argv[3:])
if not all(math.isfinite(value) for value in (enter, leave, tau)):
    raise SystemExit("gait-switch values must be finite")
if not (0.0 < leave < enter <= 1.0):
    raise SystemExit("gait switch requires 0 < gait2_to_gait1 < gait1_to_gait2 <= 1.0")
if tau <= 0.0:
    raise SystemExit("gait period transition tau must be positive")
text = source.read_text(encoding="utf-8")
replacements = {
    "gait1_to_gait2_speed_threshold": enter,
    "gait2_to_gait1_speed_threshold": leave,
    "gait_period_transition_tau": tau,
}
for key, value in replacements.items():
    text, count = re.subn(rf"^(\s*{key}:\s*)[^#\n]+", rf"\g<1>{value}", text, count=1, flags=re.MULTILINE)
    if count != 1:
        raise SystemExit(f"failed to override exactly one {key}")
temporary = output.with_name(output.name + ".tmp")
with temporary.open("w", encoding="utf-8") as stream:
    stream.write(text)
    stream.flush()
    os.fsync(stream.fileno())
os.replace(temporary, output)
PY
  controller_config_evidence_path="$run_dir/cpgwalk_config.override.yaml"
  controller_config_mount=(
    -v "$run_dir/cpgwalk_config.override.yaml:/workspace/x2/install/x1_rl_control/share/x1_rl_control/cfg/rl/cpgwalk_config.yaml:ro"
  )
fi

write_band_command() {
  local sequence=$1
  local state=$2
  local ramp_duration=${3:-0}
  python3 - "$run_dir/elastic-band-command.json" "$sequence" "$state" "$ramp_duration" <<'PY'
import json, os, pathlib, sys
path = pathlib.Path(sys.argv[1])
payload = {
    "schema": "sceneforge/physx-elastic-band-command/v1",
    "sequence": int(sys.argv[2]),
    "state": sys.argv[3],
    "real_robot_commands": False,
}
if payload["state"] == "ENGAGED":
    payload["support_ratio"] = 1.0
if payload["state"] == "RAMP_DOWN":
    payload["ramp_duration_s"] = float(sys.argv[4])
temporary = path.with_name(path.name + ".tmp")
with temporary.open("w", encoding="utf-8") as stream:
    stream.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    stream.flush()
    os.fsync(stream.fileno())
os.replace(temporary, path)
PY
}

write_cmd_vel_fixture_command() {
  local sequence=$1
  local linear_x=$2
  local linear_y=$3
  local angular_z=$4
  python3 - "$run_dir/cmd-vel-command.json" "$sequence" "$linear_x" "$linear_y" "$angular_z" <<'PY'
import json, os, pathlib, sys
path = pathlib.Path(sys.argv[1])
payload = {
    "schema": "sceneforge/x2-sim-cmd-vel-command/v1",
    "sequence": int(sys.argv[2]),
    "linear_x_mps": float(sys.argv[3]),
    "linear_y_mps": float(sys.argv[4]),
    "angular_z_rps": float(sys.argv[5]),
    "real_robot_commands": False,
}
temporary = path.with_name(path.name + ".tmp")
with temporary.open("w", encoding="utf-8") as stream:
    stream.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    stream.flush()
    os.fsync(stream.fileno())
os.replace(temporary, path)
PY
}

write_clock_freeze_request() {
  python3 - "$run_dir/clock-freeze-request.json" "$clock_freeze_duration_s" <<'PY'
import json, math, os, pathlib, sys, time
path = pathlib.Path(sys.argv[1])
duration = float(sys.argv[2])
if not math.isfinite(duration) or not 0.1 <= duration <= 10.0:
    raise SystemExit("clock freeze duration must be within [0.1, 10.0] seconds")
payload = {
    "schema": "sceneforge/physx-clock-freeze-request/v1",
    "request": "FREEZE",
    "sequence": 1,
    "wall_duration_s": duration,
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
}

run_clock_freeze_probe_gate() {
  write_clock_freeze_request
  local probe_complete=0
  for _ in $(seq 1 1200); do
    if [[ -s "$run_dir/clock-freeze-probe-receipt.json" ]]; then
      probe_complete=1
      break
    fi
    if [[ -s "$run_dir/adapter.exit" ]]; then break; fi
    if ! docker inspect -f '{{.State.Running}}' "$engine_container" 2>/dev/null | grep -q true; then break; fi
    sleep 0.1
  done
  if [[ $probe_complete -ne 1 ]]; then
    echo "controller clock-freeze probe did not complete" >&2
    return 14
  fi
  python3 - "$run_dir/clock-freeze-probe-receipt.json" <<'PY'
import json, pathlib, sys
data = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
assert data["real_robot_commands"] is False
if data.get("classification") != "SIM_TIME_BOUND":
    raise SystemExit(
        "controller clock-freeze classification is " + str(data.get("classification"))
    )
PY
}

request_controlled_physics_stop() {
  # Freeze the authoritative PhysX loop while the adapter/controller is still
  # online.  Stopping command forwarding first lets a free humanoid fall during
  # Isaac teardown and leaves a contradictory final state.
  if [[ -s "$run_dir/adapter.exit" ]]; then
    echo "adapter exited before the controlled physics stop" >&2
    return 8
  fi
  local engine_running_before_stop
  engine_running_before_stop=$(docker inspect -f '{{.State.Running}}' "$engine_container" 2>/dev/null || echo false)
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
  local controlled_stop_complete=0
  for _ in $(seq 1 600); do
    docker logs "$engine_container" >"$run_dir/engine.log" 2>&1 || true
    if grep -q "CONTROLLED_PHYSICS_STOP_COMPLETE" "$run_dir/engine.log"; then
      controlled_stop_complete=1
      break
    fi
    sleep 0.1
  done
  docker logs "$engine_container" >"$run_dir/engine.log" 2>&1 || true
  if [[ $controlled_stop_complete -ne 1 ]]; then
    echo "engine did not complete the controlled physics stop" >&2
    return 9
  fi
  # SimulationApp may hang in renderer/UI teardown after world.stop(). Once the
  # in-loop COMPLETE marker exists, killing the residual process cannot advance
  # physics and avoids Docker's grace period.
  if docker inspect -f '{{.State.Running}}' "$engine_container" 2>/dev/null | grep -q true; then
    docker kill "$engine_container" >/dev/null
  fi
  local controlled_stop_marker=false
  local controlled_stop_complete_marker=false
  if grep -q "CONTROLLED_PHYSICS_STOP_REQUESTED" "$run_dir/engine.log"; then controlled_stop_marker=true; fi
  if grep -q "CONTROLLED_PHYSICS_STOP_COMPLETE" "$run_dir/engine.log"; then controlled_stop_complete_marker=true; fi
  python3 - "$run_dir/engine-stop-proof.json" "$engine_running_before_stop" "$controlled_stop_marker" "$controlled_stop_complete_marker" <<'PY'
import json, os, pathlib, sys, time
path = pathlib.Path(sys.argv[1])
payload = {
    "schema": "sceneforge/physx-safe-stop-proof/v1",
    "status": "PASSED" if all(value == "true" for value in sys.argv[2:5]) else "FAILED",
    "physics_was_running_before_stop": sys.argv[2] == "true",
    "controlled_stop_marker_observed": sys.argv[3] == "true",
    "controlled_stop_complete_marker_observed": sys.argv[4] == "true",
    "adapter_exit_present_before_physics_stop": False,
    "physics_stopped_before_adapter": True,
    "real_robot_commands": False,
    "checked_at_unix": time.time(),
}
temporary = path.with_name(path.name + ".tmp")
with temporary.open("w", encoding="utf-8") as stream:
    stream.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    stream.flush()
    os.fsync(stream.fileno())
os.replace(temporary, path)
if payload["status"] != "PASSED":
    raise SystemExit(1)
PY
  if [[ -s "$run_dir/cmd-vel-fixture.pid" ]]; then
    docker exec "$control_container" bash -lc "
      pid=\$(cat /project/runs/$run_id/cmd-vel-fixture.pid)
      kill -INT \"\$pid\" >/dev/null 2>&1 || true
      for _ in \$(seq 1 100); do kill -0 \"\$pid\" >/dev/null 2>&1 || break; sleep 0.05; done
      kill -KILL \"\$pid\" >/dev/null 2>&1 || true
    "
  fi
  docker exec "$control_container" bash -lc \
    "pkill -INT -f '[x]2-geniesim-controller-adapter.py' >/dev/null 2>&1 || true"
}

capture_controller_debug() {
  local debug_dir="$run_dir/controller-debug"
  mkdir -p "$debug_dir"
  chmod 0777 "$debug_dir"
  docker exec "$control_container" bash -lc "
    find /tmp -maxdepth 1 -type f -name '*.csv' \
      -exec cp -t '/project/runs/$run_id/controller-debug' -- {} +
  " >/dev/null 2>&1 || true
  python3 - "$debug_dir" "$run_dir/controller-policy-manifest.json" \
    "$controller_config_evidence_path" \
    "$root/third_party/x2-release-linux/install/x1_rl_control/share/x1_rl_control/policy/cpgwalkrun_v25_v2.onnx" <<'PY'
import collections, csv, hashlib, json, os, pathlib, re, sys, tarfile

debug_dir, manifest_path, config_path, policy_path = map(pathlib.Path, sys.argv[1:])
csv_files = sorted(path for path in debug_dir.glob("*.csv") if path.is_file())
archive_path = manifest_path.with_name("controller-debug-csv.tar.gz")
if csv_files:
    with tarfile.open(archive_path, "w:gz") as archive:
        for path in csv_files:
            archive.add(path, arcname=path.name, recursive=False)

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

config_text = config_path.read_text(encoding="utf-8")
def scalar(name):
    match = re.search(rf"^\s*{re.escape(name)}:\s*([^#\n]+)", config_text, re.MULTILINE)
    return match.group(1).strip() if match else None

runtime_gait = None
cpg_debug = next((path for path in csv_files if path.name == "rl_cpgwalk_debug.csv"), None)
if cpg_debug is not None:
    with cpg_debug.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    required_columns = {"cmd_x", "cmd_y", "cmd_yaw", "current_gait", "is_standing", "hopf_period"}
    if rows and required_columns.issubset(rows[0]):
        moving = [
            row for row in rows
            if abs(float(row["cmd_x"])) + abs(float(row["cmd_y"])) + abs(float(row["cmd_yaw"])) > 0.05
        ]
        runtime_gait = {
            "row_count": len(rows),
            "moving_row_count": len(moving),
            "moving_current_gait_counts": dict(sorted(collections.Counter(row["current_gait"] for row in moving).items())),
            "moving_hopf_period_counts": dict(sorted(collections.Counter(row["hopf_period"] for row in moving).items())),
            "moving_command_x_min": min((float(row["cmd_x"]) for row in moving), default=None),
            "moving_command_x_max": max((float(row["cmd_x"]) for row in moving), default=None),
            "moving_command_yaw_min": min((float(row["cmd_yaw"]) for row in moving), default=None),
            "moving_command_yaw_max": max((float(row["cmd_yaw"]) for row in moving), default=None),
        }

payload = {
    "schema": "sceneforge/x2-controller-policy-manifest/v1",
    "configured_policy": "cpgwalkrun_v25_v2.onnx",
    "configured_run_mode": scalar("run_mode") == "true",
    "configured_gait": int(scalar("gait")) if scalar("gait") is not None else None,
    "configured_cpg_period_s": float(scalar("cpg_t")) if scalar("cpg_t") is not None else None,
    "gait1_to_gait2_speed_threshold_mps": float(scalar("gait1_to_gait2_speed_threshold")) if scalar("gait1_to_gait2_speed_threshold") is not None else None,
    "gait2_to_gait1_speed_threshold_mps": float(scalar("gait2_to_gait1_speed_threshold")) if scalar("gait2_to_gait1_speed_threshold") is not None else None,
    "gait_period_transition_tau_s": float(scalar("gait_period_transition_tau")) if scalar("gait_period_transition_tau") is not None else None,
    "config": {"path": str(config_path), "size": config_path.stat().st_size, "sha256": digest(config_path)},
    "policy": {"path": str(policy_path), "size": policy_path.stat().st_size, "sha256": digest(policy_path)},
    "debug_csv": [
        {"name": path.name, "size": path.stat().st_size, "sha256": digest(path)} for path in csv_files
    ],
    "debug_archive": (
        {"name": archive_path.name, "size": archive_path.stat().st_size, "sha256": digest(archive_path)}
        if archive_path.is_file() else None
    ),
    "runtime_gait": runtime_gait,
    "runtime_gait_interpretation": "observed_from_rl_cpgwalk_debug" if runtime_gait is not None else "not_observed",
    "real_robot_commands": False,
}
temporary = manifest_path.with_name(manifest_path.name + ".tmp")
with temporary.open("w", encoding="utf-8") as stream:
    stream.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    stream.flush()
    os.fsync(stream.fileno())
os.replace(temporary, manifest_path)
PY
}

stop_state_recorder() {
  if [[ "$record_state_trace" != true ]]; then return 0; fi
  # The recorder is PID 1.  An in-container docker exec is torn down with exit
  # 137 as soon as PID 1 exits, which used to trip this runner's `set -e` even
  # after the trace had sealed successfully.  Signal and wait from the host.
  docker kill --signal=SIGINT "$state_recorder_container" >/dev/null 2>&1 || true
  for _ in $(seq 1 300); do
    if [[ "$(docker inspect -f '{{.State.Running}}' "$state_recorder_container" 2>/dev/null || true)" != true ]]; then
      break
    fi
    sleep 0.1
  done
  if [[ "$(docker inspect -f '{{.State.Running}}' "$state_recorder_container" 2>/dev/null || true)" == true ]]; then
    docker kill "$state_recorder_container" >/dev/null 2>&1 || true
  fi
  python3 - "$run_dir/render-state-receipt.json" <<'PY'
import json, pathlib, sys
data = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
assert data["status"] == "SUCCEEDED"
assert data["sample_count"] >= data["minimum_samples"]
assert data["published_topics"] == []
assert data["read_only"] is True
assert data["authoritative_physics_source"] == "geniesim_isaac_physx"
assert data["real_robot_commands"] is False
PY
}

if [[ "$elastic_band_enabled" == true ]]; then
  write_band_command 1 ENGAGED
fi
if docker container inspect "$engine_container" >/dev/null 2>&1 || docker container inspect "$control_container" >/dev/null 2>&1; then
  echo "refusing to reuse existing run container" >&2
  exit 2
fi

cleanup() {
  set +e
  docker stop --time 20 "$control_container" >/dev/null 2>&1
  docker stop --time 30 "$render_container" >/dev/null 2>&1
  docker stop --time 10 "$recorder_container" >/dev/null 2>&1
  docker stop --time 10 "$state_recorder_container" >/dev/null 2>&1
  docker stop --time 30 "$engine_container" >/dev/null 2>&1
  docker rm "$control_container" >/dev/null 2>&1
  docker rm "$render_container" >/dev/null 2>&1
  docker rm "$recorder_container" >/dev/null 2>&1
  docker rm "$state_recorder_container" >/dev/null 2>&1
  docker rm "$engine_container" >/dev/null 2>&1
  docker network rm "$ros_network" >/dev/null 2>&1
}
trap cleanup EXIT INT TERM

docker network create --internal "$ros_network" >/dev/null

# A free-base humanoid cannot wait through Isaac startup with no feedback.
# Preload the vendor controller before physics starts; it cannot command the
# simulator until the adapter starts after the engine-ready gate.
if [[ "$base_mode" == floating ]]; then
  docker run -d --name "$control_container" --network "$ros_network" \
    -e ROS_DOMAIN_ID="$domain" -e ROS_LOCALHOST_ONLY=0 \
    -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET -e REAL_ROBOT_COMMANDS=false \
    -v "$root:/project" -v "$root/third_party/x2-release-linux:/workspace/x2:ro" \
    "${controller_config_mount[@]}" \
    "$control_image" sleep infinity >/dev/null
  docker exec -d "$control_container" bash -lc "
    set +u
    source /opt/ros/humble/setup.bash
    source /workspace/x2/install/setup.bash
    set -u
    export LD_LIBRARY_PATH=/workspace/x2/runtime_lib:\${LD_LIBRARY_PATH:-}
    cd /workspace/x2/install/x1_rl_control/share/x1_rl_control
    /workspace/x2/install/x1_rl_control/lib/x1_rl_control/x1_rl_control_node \
      /workspace/x2/install/x1_rl_control/share/x1_rl_control/cfg/rl_x1_sim.yaml \
      --ros-args -p enable_embedded_x1_controller:=true \
      -p axes_config_path:=/workspace/x2/install/x1_controller/share/x1_controller/cfg/axes.yaml \
      -p controller_profile:=normal -p publish_when_idle:=true \
      -p continue_publish_when_disconnected:=true -p use_aggregate_joint_cmd_input:=false \
      -p command_topic_best_effort:=true -p enable_joint_state_subscriptions:=false \
      -p subscribe_joint_states:=false -p enable_dedicated_joint_state_consumer:=true \
      -p dedicated_joint_state_consumer_rt:=false -p dedicated_generic_callback_rt:=false \
      -p dedicated_mainloop_executor_rt:=false -p enable_dedicated_joint_cmd_publisher:=true \
      -p dedicated_joint_cmd_publisher_rt:=false -p ros_executor_rt:=false \
      -p use_sim_time:=true \
      >/project/runs/$run_id/controller.log 2>&1
  "
fi

docker run -d --name "$engine_container" --network "$ros_network" --gpus all --entrypoint bash \
  --ipc host \
  -e ROS_DOMAIN_ID="$domain" -e ROS_LOCALHOST_ONLY=0 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  -e ROS_DISTRO=jazzy -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.core/jazzy/lib:/opt/ros/jazzy/lib:/isaac-sim/kit/lib \
  -e PYTHONUNBUFFERED=1 \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
  -e ISAACSIM_VIEWPORT_UPDATES="$isaac_viewport_updates" \
  -e REAL_ROBOT_COMMANDS=false \
  -e ROBOEASE_PHYSX_ELASTIC_BAND_ENABLED="$elastic_band_enabled" \
  -e ROBOEASE_PHYSX_ELASTIC_BAND_BODY=torso_link \
  -e ROBOEASE_PHYSX_ELASTIC_BAND_ANCHOR=0,0,2.5 \
  -e ROBOEASE_PHYSX_ELASTIC_BAND_STIFFNESS=200 \
  -e ROBOEASE_PHYSX_ELASTIC_BAND_DAMPING=100 \
  -e ROBOEASE_PHYSX_ELASTIC_BAND_REST_LENGTH=0 \
  -e ROBOEASE_PHYSX_ELASTIC_BAND_MAX_FORCE="$elastic_band_max_force_n" \
  -e ROBOEASE_PHYSX_ELASTIC_BAND_COMMAND_FILE="/project/runs/$run_id/elastic-band-command.json" \
  -e ROBOEASE_PHYSX_ELASTIC_BAND_STATUS_FILE="/project/runs/$run_id/elastic-band-status.json" \
  -e ROBOEASE_PHYSX_ELASTIC_BAND_TELEMETRY_FILE="/project/runs/$run_id/elastic-band-telemetry.jsonl" \
  -e ROBOEASE_PHYSX_CONTROLLED_STOP_FILE="/project/runs/$run_id/physics-stop-request.json" \
  -e ROBOEASE_PHYSX_CLOCK_FREEZE_FILE="/project/runs/$run_id/clock-freeze-request.json" \
  -e ROBOEASE_PHYSX_CLOCK_FREEZE_STATUS_FILE="/project/runs/$run_id/clock-freeze-status.json" \
  -e ROBOEASE_GENIE_PUBLISH_RENDER_BODY_TF=true \
  -e ROBOEASE_GENIE_RENDER_BODY_TF_HZ="$render_body_tf_hz" \
  -v "$root:/project" \
  -v "$overlay:/workspace/x2-overlay-v3:ro" \
  -v "$root/isaac-cache/kit:/isaac-sim/kit/cache" \
  "$engine_image" -lc "
    set -e
    source /opt/ros/jazzy/setup.bash
    source '$geniesim_build/install/setup.bash'
    exec /isaac-sim/python.sh '$engine' --ros-args \
      -p stage_manifest:='$manifest' \
      -p physics_engine:=isaac_physx \
      -p physics_hz:=$physics_hz \
      -p render_hz:=10 \
      -p realtime_factor:=1.0 \
      -p fake_slam:=false \
      -p physics_params_file:='$physics' \
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

if [[ "$record_state_trace" == true ]]; then
  docker run -d --name "$state_recorder_container" --network "$ros_network" --entrypoint bash \
    -e ROS_DOMAIN_ID="$domain" -e ROS_LOCALHOST_ONLY=0 \
    -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
    -e ROS_DISTRO=jazzy -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e REAL_ROBOT_COMMANDS=false \
    -v "$root:/project" "$engine_image" -lc "
      source /opt/ros/jazzy/setup.bash
      echo \$\$ > '/project/runs/$run_id/render-state-recorder.pid'
      exec python3 /project/admin/spark/record-x2-geniesim-render-state.py \
        --tf-topic /tf_render --base-topic /odom \
        --output '/project/runs/$run_id/render-state.jsonl' \
        --receipt '/project/runs/$run_id/render-state-receipt.json' \
        --sample-rate-hz 30 --minimum-samples 180 --wall-timeout-s 900 \
        >'/project/runs/$run_id/render-state-recorder.log' 2>&1
    " >/dev/null
  state_recorder_ready=0
  for _ in $(seq 1 200); do
    if python3 - "$run_dir/render-state-receipt.json" <<'PY' >/dev/null 2>&1
import json, pathlib, sys
data = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
assert data["status"] == "RUNNING"
assert data["published_topics"] == []
assert data["real_robot_commands"] is False
PY
    then
      state_recorder_ready=1
      break
    fi
    sleep 0.1
  done
  if [[ $state_recorder_ready -ne 1 ]]; then
    echo "read-only X2 render-state recorder did not start" >&2
    exit 4
  fi
fi

if [[ "$record_video" == true ]]; then
  docker run -d --name "$render_container" --network "$ros_network" --user 0:0 --gpus all --entrypoint bash \
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
  render_ready=0
  for _ in $(seq 1 600); do
    docker logs "$render_container" >"$run_dir/render.log" 2>&1 || true
    if grep -Eq "Render node initialized|render node ready" "$run_dir/render.log"; then render_ready=1; break; fi
    if ! docker inspect -f '{{.State.Running}}' "$render_container" 2>/dev/null | grep -q true; then break; fi
    sleep 0.5
  done
  if [[ $render_ready -ne 1 ]]; then
    echo "Genie Sim renderer did not become ready" >&2
    exit 4
  fi
  if ! grep -Fq "robot_root_pose applied to /x2" "$run_dir/render.log"; then
    echo "Genie Sim renderer did not apply the X2 root spawn pose" >&2
    exit 4
  fi
  docker run -d --name "$recorder_container" --network "$ros_network" \
    -e ROS_DOMAIN_ID="$domain" -e ROS_LOCALHOST_ONLY=0 \
    -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET -e REAL_ROBOT_COMMANDS=false \
    -v "$root:/project" roboease/x2-video-recorder-humble-arm64:20260818 sleep infinity >/dev/null
  docker exec -d "$recorder_container" bash -lc "
    source /opt/ros/humble/setup.bash
    echo \$\$ > /project/runs/$run_id/recorder.pid
    exec python3 /project/admin/spark/record-geniesim-freecam.py \
      --image-topic /genie_sim/free_camera_rgb/image_raw --base-state-topic /odom \
      --camera-pose-topic /genie_sim_render/free_cam_pose \
      --output /project/runs/$run_id/x2-physx-walk-source-30fps.mp4 \
      --status /project/runs/$run_id/video-receipt.json \
      --fps 30 --minimum-frames 180 --wall-timeout-s 900 \
      --offset-x -1.4 --offset-y -1.5 --offset-z 1.25 \
      --target-x 0.15 --target-y 0.0 --target-z 0.65 \
      >/project/runs/$run_id/recorder.log 2>&1
  "
  first_frame_ready=0
  for _ in $(seq 1 480); do
    if grep -Eq '"image_message_count"[[:space:]]*:[[:space:]]*[1-9]' "$run_dir/video-receipt.json" 2>/dev/null; then
      first_frame_ready=1
      break
    fi
    if ! docker inspect -f '{{.State.Running}}' "$render_container" 2>/dev/null | grep -q true; then break; fi
    sleep 0.5
  done
  if [[ $first_frame_ready -ne 1 ]]; then
    echo "Genie Sim renderer produced no first frame" >&2
    exit 5
  fi
  sleep 1
fi

if [[ "$base_mode" == fixed ]]; then
docker run -d --name "$control_container" --network "$ros_network" \
  -e ROS_DOMAIN_ID="$domain" -e ROS_LOCALHOST_ONLY=0 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  -e REAL_ROBOT_COMMANDS=false \
  -v "$root:/project" \
  -v "$root/third_party/x2-release-linux:/workspace/x2:ro" \
  "${controller_config_mount[@]}" \
  "$control_image" sleep infinity >/dev/null
fi

docker exec -d "$control_container" bash -lc "
  set +u
  source /opt/ros/humble/setup.bash
  source /workspace/x2/install/setup.bash
  set -u
  export LD_LIBRARY_PATH=/workspace/x2/runtime_lib:\${LD_LIBRARY_PATH:-}
  timeout --signal=INT --kill-after=10 $duration \
    python3 /project/admin/spark/x2-geniesim-controller-adapter.py \
      --receipt /project/runs/$run_id/adapter-receipt.json \
      --command-mode $adapter_command_mode \
      --effort-gain-scale $effort_gain_scale \
      --publish-mode $adapter_publish_mode --timebase sim --require-command-stamp \
      --maximum-command-age-s 0.004 \
      --minimum-state-messages 100 --minimum-command-sets 10 \
      --ros-args -p use_sim_time:=true \
      >/project/runs/$run_id/adapter.log 2>&1
  code=\$?
  printf '%s\n' \"\$code\" >/project/runs/$run_id/adapter.exit
"

if [[ "$base_mode" == fixed ]]; then
sleep 2
docker exec -d "$control_container" bash -lc "
  set +u
  source /opt/ros/humble/setup.bash
  source /workspace/x2/install/setup.bash
  set -u
  export LD_LIBRARY_PATH=/workspace/x2/runtime_lib:\${LD_LIBRARY_PATH:-}
  cd /workspace/x2/install/x1_rl_control/share/x1_rl_control
  timeout --signal=INT --kill-after=10 $((duration - 4)) \
    /workspace/x2/install/x1_rl_control/lib/x1_rl_control/x1_rl_control_node \
      /workspace/x2/install/x1_rl_control/share/x1_rl_control/cfg/rl_x1_sim.yaml \
      --ros-args \
      -p enable_embedded_x1_controller:=true \
      -p axes_config_path:=/workspace/x2/install/x1_controller/share/x1_controller/cfg/axes.yaml \
      -p controller_profile:=normal \
      -p publish_when_idle:=true \
      -p continue_publish_when_disconnected:=true \
      -p use_aggregate_joint_cmd_input:=false \
      -p command_topic_best_effort:=true \
      -p enable_joint_state_subscriptions:=false \
      -p subscribe_joint_states:=false \
      -p enable_dedicated_joint_state_consumer:=true \
      -p dedicated_joint_state_consumer_rt:=false \
      -p dedicated_generic_callback_rt:=false \
      -p dedicated_mainloop_executor_rt:=false \
      -p enable_dedicated_joint_cmd_publisher:=true \
      -p dedicated_joint_cmd_publisher_rt:=false \
      -p ros_executor_rt:=false \
      -p use_sim_time:=true \
      >/project/runs/$run_id/controller.log 2>&1
  code=\$?
  printf '%s\n' \"\$code\" >/project/runs/$run_id/controller.exit
"
fi

if [[ "$clock_freeze_probe_enabled" == true ]]; then
  docker exec -d "$control_container" bash -lc "
    set +u
    source /opt/ros/humble/setup.bash
    source /workspace/x2/install/setup.bash
    set -u
    export LD_LIBRARY_PATH=/workspace/x2/runtime_lib:\${LD_LIBRARY_PATH:-}
    exec python3 /project/admin/spark/x2-clock-freeze-probe.py \
      --freeze-status /project/runs/$run_id/clock-freeze-status.json \
      --receipt /project/runs/$run_id/clock-freeze-probe-receipt.json \
      --wall-timeout-s 180 \
      >/project/runs/$run_id/clock-freeze-probe.log 2>&1
  "
fi

if [[ "$control_mode" == stand || "$control_mode" == stand_default || "$control_mode" == walk ]]; then
  if [[ "$base_mode" == floating ]]; then sleep 0.2; else sleep 6; fi
  docker exec "$control_container" bash -lc "
    set +u
    source /opt/ros/humble/setup.bash
    source /workspace/x2/install/setup.bash
    set -u
    export LD_LIBRARY_PATH=/workspace/x2/runtime_lib:\${LD_LIBRARY_PATH:-}
    ros2 topic pub --once /stand_mode std_msgs/msg/Float32 '{data: 1.0}'
  " >"$run_dir/trigger.log" 2>&1
fi
if [[ "$control_mode" == stand_default || "$control_mode" == walk ]]; then
  sleep "$stand_settle_s"
  docker exec "$control_container" bash -lc "
    set +u
    source /opt/ros/humble/setup.bash
    source /workspace/x2/install/setup.bash
    set -u
    export LD_LIBRARY_PATH=/workspace/x2/runtime_lib:\${LD_LIBRARY_PATH:-}
    ros2 topic pub --once /stand_default_mode std_msgs/msg/Float32 '{data: 1.0}'
  " >>"$run_dir/trigger.log" 2>&1
fi
if [[ "$elastic_band_enabled" == true ]]; then
  # Keep support through the vendor's transitional standing states, then
  # physically ramp it to zero.  Locomotion is forbidden until the engine's
  # own status receipt proves exact zero force and remains zero during an
  # unsupported hold interval.
  sleep "$stand_default_settle_s"
  write_band_command 2 RAMP_DOWN "$elastic_band_ramp_s"
  band_released=0
  for _ in $(seq 1 600); do
    if python3 - "$run_dir/elastic-band-status.json" <<'PY' >/dev/null 2>&1
import json, pathlib, sys
path = pathlib.Path(sys.argv[1])
data = json.loads(path.read_text(encoding="utf-8"))
assert data["state"] == "RELEASED"
assert data["support_ratio"] == 0.0
assert data["force_norm_n"] == 0.0
assert data["force_is_exactly_zero"] is True
assert data["root_pinned"] is False
assert data["gravity_modified"] is False
assert data["real_robot_commands"] is False
PY
    then
      band_released=1
      break
    fi
    if ! docker inspect -f '{{.State.Running}}' "$engine_container" 2>/dev/null | grep -q true; then break; fi
    sleep 0.1
  done
  if [[ $band_released -ne 1 ]]; then
    echo "PhysX elastic band did not reach an auditable zero-force RELEASED state" >&2
    exit 6
  fi
  # Measure unsupported hold in simulator time, not wall time. Spark currently
  # advances this PhysX workload below 1x realtime, so a wall-clock sleep would
  # overstate the physical duration and could release locomotion too early.
  unsupported_hold_complete=0
  for _ in $(seq 1 3600); do
    if python3 - "$run_dir/elastic-band-status.json" "$elastic_band_unsupported_hold_s" <<'PY' >/dev/null 2>&1
import json, pathlib, sys
status = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
release_times = [
    float(item["sim_time_s"])
    for item in status.get("transitions", [])
    if item.get("effective_state") == "RELEASED"
    and item.get("requested_state") == "RAMP_DOWN_COMPLETE"
]
assert release_times
assert status["state"] == "RELEASED"
assert status["support_ratio"] == 0.0
assert status["force_norm_n"] == 0.0
assert float(status["sim_time_s"]) - release_times[-1] >= float(sys.argv[2])
PY
    then
      unsupported_hold_complete=1
      break
    fi
    if [[ -s "$run_dir/adapter.exit" ]]; then break; fi
    if ! docker inspect -f '{{.State.Running}}' "$engine_container" 2>/dev/null | grep -q true; then break; fi
    sleep 0.1
  done
  if [[ $unsupported_hold_complete -ne 1 ]]; then
    echo "PhysX elastic band release did not complete the requested simulator-time unsupported hold" >&2
    exit 7
  fi
  python3 - "$run_dir/elastic-band-status.json" "$run_dir/elastic-band-release-proof.json" "$elastic_band_unsupported_hold_s" <<'PY'
import json, os, pathlib, sys, time
status_path, proof_path = map(pathlib.Path, sys.argv[1:3])
required_hold_s = float(sys.argv[3])
status = json.loads(status_path.read_text(encoding="utf-8"))
release_times = [
    float(item["sim_time_s"])
    for item in status.get("transitions", [])
    if item.get("effective_state") == "RELEASED"
    and item.get("requested_state") == "RAMP_DOWN_COMPLETE"
]
release_sim_time_s = release_times[-1] if release_times else None
observed_hold_s = float(status.get("sim_time_s", 0.0)) - release_sim_time_s if release_sim_time_s is not None else -1.0
checks = {
    "state_released": status.get("state") == "RELEASED",
    "support_ratio_zero": status.get("support_ratio") == 0.0,
    "force_norm_zero": status.get("force_norm_n") == 0.0,
    "force_exactly_zero": status.get("force_is_exactly_zero") is True,
    "root_not_pinned": status.get("root_pinned") is False,
    "gravity_not_modified": status.get("gravity_modified") is False,
    "real_robot_commands_false": status.get("real_robot_commands") is False,
    "sim_time_hold_met": observed_hold_s >= required_hold_s,
}
proof = {
    "schema": "sceneforge/physx-elastic-band-release-proof/v1",
    "status": "PASSED" if all(checks.values()) else "FAILED",
    "checks": checks,
    "status_sequence": status.get("sequence"),
    "status_sim_time_s": status.get("sim_time_s"),
    "status_sample_count": status.get("sample_count"),
    "release_sim_time_s": release_sim_time_s,
    "required_unsupported_hold_s": required_hold_s,
    "observed_unsupported_hold_s": observed_hold_s,
    "checked_at_unix": time.time(),
    "locomotion_triggered": False,
    "real_robot_commands": False,
}
temporary = proof_path.with_name(proof_path.name + ".tmp")
with temporary.open("w", encoding="utf-8") as stream:
    stream.write(json.dumps(proof, indent=2, sort_keys=True) + "\n")
    stream.flush()
    os.fsync(stream.fileno())
os.replace(temporary, proof_path)
if proof["status"] != "PASSED":
    raise SystemExit(1)
PY
  if [[ "$clock_freeze_probe_enabled" == true ]]; then
    if ! run_clock_freeze_probe_gate; then
      # Do not enter locomotion or tune dynamics when the controller continues
      # to advance against wall time.  Preserve the diagnostic run and stop
      # PhysX while the controller/adapter are still online.
      request_controlled_physics_stop || true
      stop_state_recorder || true
      capture_controller_debug || true
      docker logs "$engine_container" >"$run_dir/engine.log" 2>&1 || true
      mkdir -p "$evidence_dir"
      cp -a "$run_dir/." "$evidence_dir/"
      python3 - "$evidence_dir" <<'PY'
import hashlib, json, os, pathlib, sys, time
evidence = pathlib.Path(sys.argv[1])
probe_path = evidence / "clock-freeze-probe-receipt.json"
probe = (
    json.loads(probe_path.read_text(encoding="utf-8"))
    if probe_path.is_file()
    else {
        "schema": "sceneforge/x2-controller-clock-freeze-probe/v1",
        "status": "FAILED",
        "classification": "PROBE_INCOMPLETE",
        "real_robot_commands": False,
    }
)
artifacts = []
for path in sorted(item for item in evidence.rglob("*") if item.is_file() and item.name != "result.json"):
    artifacts.append({
        "path": str(path.relative_to(evidence)),
        "size": path.stat().st_size,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    })
payload = {
    "schema": "sceneforge/x2-geniesim-clock-freeze-gate-result/v1",
    "status": "BLOCKED",
    "qualification": False,
    "reason": probe.get("classification"),
    "decision": "do_not_tune_dynamics_or_claim_mujoco_equivalence",
    "probe": probe,
    "artifacts": artifacts,
    "finished_at_unix": time.time(),
    "physics_authority": "genie_sim_isaac_physx",
    "mujoco_state_source_used": False,
    "real_robot_commands": False,
}
temporary = evidence / ".result.json.tmp"
with temporary.open("w", encoding="utf-8") as stream:
    stream.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    stream.flush()
    os.fsync(stream.fileno())
os.replace(temporary, evidence / "result.json")
PY
      chmod -R a-w "$evidence_dir"
      echo "clock-freeze gate blocked locomotion; evidence: $evidence_dir" >&2
      exit 14
    fi
  fi
  if [[ "$control_mode" == stand_default ]]; then
    request_controlled_physics_stop
    if [[ "$clock_freeze_probe_enabled" == true ]]; then
      capture_controller_debug
    fi
  fi
fi
if [[ "$control_mode" == walk ]]; then
  # Locomotion must start from the vendor's stable RL standing state, not
  # directly from the transitional PD-assisted `stand` state. Validate the
  # zero-force proof before changing controller state.
  python3 - "$run_dir/elastic-band-release-proof.json" <<'PY'
import json, pathlib, sys
data = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
if data.get("status") != "PASSED":
    raise SystemExit("refusing locomotion without a passing band-release proof")
PY
  # Start one durable publisher before locomotion. Runtime changes are atomic
  # command-file updates, so DDS discovery and publisher replacement cannot
  # create a gap or make the controller bounce between stand and locomotion.
  write_cmd_vel_fixture_command 1 "$walk_speed" "$walk_lateral_speed" "$walk_yaw_rate"
  docker exec -d "$control_container" bash -lc "
    set +u
    source /opt/ros/humble/setup.bash
    echo \$\$ >/project/runs/$run_id/cmd-vel-fixture.pid
    exec python3 /project/admin/spark/x2-cmd-vel-fixture.py \
      --command /project/runs/$run_id/cmd-vel-command.json \
      --receipt /project/runs/$run_id/cmd-vel-fixture-receipt.json \
      --rate-hz 20 $heading_hold_flag \
      --heading-kp $heading_hold_kp \
      --heading-max-correction-rps $heading_hold_max_correction \
      >/project/runs/$run_id/cmd-vel-fixture.log 2>&1
  "
  cmd_fixture_ready=0
  for _ in $(seq 1 200); do
    if python3 - "$run_dir/cmd-vel-fixture-receipt.json" <<'PY' >/dev/null 2>&1
import json, pathlib, sys
data = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
assert data["status"] == "RUNNING"
assert data["last_sequence"] == 1
assert data["message_count"] >= 3
assert data["real_robot_commands"] is False
PY
    then
      cmd_fixture_ready=1
      break
    fi
    sleep 0.1
  done
  if [[ $cmd_fixture_ready -ne 1 ]]; then
    echo "simulation cmd_vel fixture did not start" >&2
    request_controlled_physics_stop || true
    exit 10
  fi
  docker exec "$control_container" bash -lc "
    set +u
    source /opt/ros/humble/setup.bash
    source /workspace/x2/install/setup.bash
    set -u
    export LD_LIBRARY_PATH=/workspace/x2/runtime_lib:\${LD_LIBRARY_PATH:-}
    ros2 topic pub --once /locomotion_default_mode std_msgs/msg/Float32 '{data: 1.0}'
  " >>"$run_dir/trigger.log" 2>&1
  python3 - "$run_dir/elastic-band-release-proof.json" <<'PY'
import json, os, pathlib, sys
path = pathlib.Path(sys.argv[1])
data = json.loads(path.read_text(encoding="utf-8"))
if data.get("status") != "PASSED":
    raise SystemExit("refusing locomotion without a passing band-release proof")
data["locomotion_triggered"] = True
temporary = path.with_name(path.name + ".tmp")
with temporary.open("w", encoding="utf-8") as stream:
    stream.write(json.dumps(data, indent=2, sort_keys=True) + "\n")
    stream.flush()
    os.fsync(stream.fileno())
os.replace(temporary, path)
PY
  python3 "$root/admin/spark/x2-sim-time-walk-window.py" start \
    --status "$run_dir/elastic-band-status.json" \
    --receipt "$run_dir/walk-timing.json" \
    --speed "$walk_speed" --duration "$walk_duration" --settle "$walk_stop_settle_s" \
    --deceleration-speed "$walk_deceleration_speed" \
    --deceleration-duration "$walk_deceleration_duration"
  walk_window_complete=0
  for _ in $(seq 1 7200); do
    if python3 "$root/admin/spark/x2-sim-time-walk-window.py" walk-reached \
      --status "$run_dir/elastic-band-status.json" --receipt "$run_dir/walk-timing.json"; then
      walk_window_complete=1
      break
    fi
    if [[ -s "$run_dir/adapter.exit" ]]; then break; fi
    if ! docker inspect -f '{{.State.Running}}' "$engine_container" 2>/dev/null | grep -q true; then break; fi
    sleep 0.1
  done
  if [[ $walk_window_complete -ne 1 ]]; then
    echo "walking command did not complete the requested simulator-time window" >&2
    request_controlled_physics_stop || true
    exit 11
  fi
  zero_sequence=2
  if [[ "$use_staged_deceleration" == true ]]; then
    # Keep the locomotion controller active while bleeding speed into the
    # already-qualified 0.3 m/s handoff band.  This avoids asking the standing
    # controller to absorb the full high-speed momentum in one transition.
    write_cmd_vel_fixture_command 2 "$walk_deceleration_speed" 0.0 "$walk_yaw_rate"
    deceleration_command_ready=0
    for _ in $(seq 1 200); do
      if python3 - "$run_dir/cmd-vel-fixture-receipt.json" "$walk_deceleration_speed" <<'PY' >/dev/null 2>&1
import json, pathlib, sys
data = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
assert data["status"] == "RUNNING"
assert data["last_sequence"] == 2
assert data["last_command"]["linear_x_mps"] == float(sys.argv[2])
assert data["real_robot_commands"] is False
PY
      then
        deceleration_command_ready=1
        break
      fi
      sleep 0.1
    done
    if [[ $deceleration_command_ready -ne 1 ]]; then
      echo "simulation cmd_vel fixture did not apply the staged deceleration command" >&2
      request_controlled_physics_stop || true
      exit 12
    fi
    python3 "$root/admin/spark/x2-sim-time-walk-window.py" mark-deceleration \
      --status "$run_dir/elastic-band-status.json" --receipt "$run_dir/walk-timing.json"
    deceleration_complete=0
    for _ in $(seq 1 3600); do
      if python3 "$root/admin/spark/x2-sim-time-walk-window.py" deceleration-reached \
        --status "$run_dir/elastic-band-status.json" --receipt "$run_dir/walk-timing.json"; then
        deceleration_complete=1
        break
      fi
      if [[ -s "$run_dir/adapter.exit" ]]; then break; fi
      if ! docker inspect -f '{{.State.Running}}' "$engine_container" 2>/dev/null | grep -q true; then break; fi
      sleep 0.1
    done
    if [[ $deceleration_complete -ne 1 ]]; then
      echo "staged deceleration did not complete the requested simulator-time window" >&2
      request_controlled_physics_stop || true
      exit 12
    fi
    zero_sequence=3
  fi
  # Change to zero through the same durable publisher and require the sequence
  # update before beginning the simulator-time stop window.
  write_cmd_vel_fixture_command "$zero_sequence" 0.0 0.0 0.0
  zero_command_ready=0
  for _ in $(seq 1 200); do
    if python3 - "$run_dir/cmd-vel-fixture-receipt.json" "$zero_sequence" <<'PY' >/dev/null 2>&1
import json, pathlib, sys
data = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
assert data["status"] == "RUNNING"
assert data["last_sequence"] == int(sys.argv[2])
assert data["message_count"] >= 4
assert data["last_command"]["linear_x_mps"] == 0.0
assert data["last_command"]["linear_y_mps"] == 0.0
assert data["last_command"]["angular_z_rps"] == 0.0
PY
    then
      zero_command_ready=1
      break
    fi
    sleep 0.1
  done
  if [[ $zero_command_ready -ne 1 ]]; then
    echo "simulation cmd_vel fixture did not apply the zero command" >&2
    request_controlled_physics_stop || true
    exit 12
  fi
  python3 "$root/admin/spark/x2-sim-time-walk-window.py" mark-stop \
    --status "$run_dir/elastic-band-status.json" --receipt "$run_dir/walk-timing.json"
  stop_settle_complete=0
  for _ in $(seq 1 3600); do
    if python3 "$root/admin/spark/x2-sim-time-walk-window.py" settle-reached \
      --status "$run_dir/elastic-band-status.json" --receipt "$run_dir/walk-timing.json"; then
      stop_settle_complete=1
      break
    fi
    if [[ -s "$run_dir/adapter.exit" ]]; then break; fi
    if ! docker inspect -f '{{.State.Running}}' "$engine_container" 2>/dev/null | grep -q true; then break; fi
    sleep 0.1
  done
  if [[ $stop_settle_complete -ne 1 ]]; then
    echo "walking stop did not complete the requested simulator-time settle" >&2
    request_controlled_physics_stop || true
    exit 13
  fi
  # Seal the physical terminal state before freezing the physics authority.
  if ! python3 "$root/admin/spark/x2-sim-time-walk-window.py" finish \
    --status "$run_dir/elastic-band-status.json" --receipt "$run_dir/walk-timing.json"; then
    echo "walking stop receipt recorded a failed convergence check" >&2
  fi
  request_controlled_physics_stop
  stop_state_recorder
  capture_controller_debug
fi

deadline=$((SECONDS + duration + 30))
while [[ $SECONDS -lt $deadline ]]; do
  if [[ -s "$run_dir/adapter.exit" && ( "$base_mode" == floating || -s "$run_dir/controller.exit" ) ]]; then
    break
  fi
  sleep 1
done

# Fixed-base controller runs are also oracle captures for the engine-neutral
# adapter.  Preserve the vendor debug CSVs even when the synchronized command
# gate correctly rejects the wall-clock-bound publisher.  Previously this
# collector ran only in the floating walk branch, losing STAND_DEFAULT traces.
if [[ "$base_mode" == fixed && "$control_mode" != walk ]]; then
  capture_controller_debug || true
fi

if [[ "$record_video" == true ]]; then
  docker exec "$recorder_container" bash -lc '
    if test -s /project/runs/'"$run_id"'/recorder.pid; then
      pid=$(cat /project/runs/'"$run_id"'/recorder.pid)
      kill -TERM "$pid" 2>/dev/null || true
      for _ in $(seq 1 480); do kill -0 "$pid" 2>/dev/null || exit 0; sleep 0.25; done
      kill -KILL "$pid" 2>/dev/null || true
    fi
  ' || true
  docker logs "$render_container" >"$run_dir/render.log" 2>&1 || true
  docker stop --time 30 "$render_container" >/dev/null || true
fi

docker logs "$engine_container" >"$run_dir/engine.log" 2>&1 || true
docker stop --time 30 "$engine_container" >/dev/null
docker logs "$engine_container" >"$run_dir/engine.log" 2>&1 || true

python3 - "$run_dir" "$evidence_dir" "$domain" "$engine_image" "$control_image" "$control_mode" "$base_mode" "$physics_hz" "$adapter_command_mode" "$walk_speed" "$walk_lateral_speed" "$walk_yaw_rate" "$walk_duration" "$walk_stop_settle_s" "$walk_deceleration_speed" "$walk_deceleration_duration" "$heading_hold_enabled" "$heading_hold_kp" "$heading_hold_max_correction" "$effort_gain_scale" "$record_video" "$record_state_trace" "$elastic_band_enabled" "$elastic_band_ramp_s" "$elastic_band_unsupported_hold_s" "$clock_freeze_probe_enabled" "$clock_freeze_duration_s" "$render_body_tf_hz" "$adapter_publish_mode" "$isaac_viewport_updates" <<'PY'
import hashlib, json, math, pathlib, re, shutil, sys, time
run, evidence = map(pathlib.Path, sys.argv[1:3])
domain, engine_image, control_image, control_mode, base_mode, requested_physics_hz, adapter_command_mode, walk_speed, walk_lateral_speed, walk_yaw_rate, walk_duration, walk_stop_settle_s, walk_deceleration_speed, walk_deceleration_duration, heading_hold_enabled, heading_hold_kp, heading_hold_max_correction, effort_gain_scale, record_video, record_state_trace, elastic_band_enabled, elastic_band_ramp_s, elastic_band_unsupported_hold_s, clock_freeze_probe_enabled, clock_freeze_duration_s, render_body_tf_hz, adapter_publish_mode, isaac_viewport_updates = sys.argv[3:]

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

failure = []
required = ["adapter-receipt.json", "adapter.log", "controller.log", "engine.log"]
for name in required:
    if not (run / name).is_file():
        failure.append(f"missing:{name}")
adapter = {}
if (run / "adapter-receipt.json").is_file():
    adapter = json.loads((run / "adapter-receipt.json").read_text())
    if adapter.get("status") != "PASSED": failure.append("adapter_not_passed")
    if adapter.get("real_robot_commands") is not False: failure.append("real_robot_flag")
    if adapter.get("publish_mode") != adapter_publish_mode: failure.append("adapter_publish_mode_mismatch")
    if adapter.get("timebase") != "sim": failure.append("adapter_not_sim_time")
    if adapter.get("use_sim_time") is not True: failure.append("adapter_use_sim_time_not_enabled")
    if adapter.get("synchronized_control_gate") is not True: failure.append("adapter_synchronized_gate_failed")
clock_freeze_probe = None
if clock_freeze_probe_enabled == "true":
    probe_path = run / "clock-freeze-probe-receipt.json"
    if not probe_path.is_file():
        failure.append("clock_freeze_probe_missing")
    else:
        clock_freeze_probe = json.loads(probe_path.read_text())
        if clock_freeze_probe.get("classification") != "SIM_TIME_BOUND":
            failure.append("controller_not_sim_time_bound")
        if clock_freeze_probe.get("real_robot_commands") is not False:
            failure.append("clock_freeze_probe_real_robot_flag")
controller_log = (run / "controller.log").read_text(errors="replace") if (run / "controller.log").is_file() else ""
engine_log = (run / "engine.log").read_text(errors="replace") if (run / "engine.log").is_file() else ""
for marker in ("segmentation fault", "nan detected", "process has died", "traceback"):
    if marker in (controller_log + engine_log).lower(): failure.append(f"fatal_marker:{marker}")
if "Isaac Sim physics engine ready" not in engine_log: failure.append("engine_not_ready")
if not any(token in controller_log for token in ("Init succeeded", "Started succeeded")):
    failure.append("controller_start_not_proven")
actual_hz = [float(value) for value in re.findall(r"Actual Hz:\s*([0-9.]+)", engine_log)]
if control_mode in ("stand", "stand_default", "walk"):
    if "StateMachine transitioned on [/stand_mode] -> stand" not in controller_log:
        failure.append("stand_controller_transition_not_proven")
    if float(adapter.get("max_abs_position_target", 0.0)) < 0.02:
        failure.append("stand_command_excursion_too_small")
    if float(adapter.get("max_observed_physx_joint_excursion", 0.0)) < 0.01:
        failure.append("physx_joint_response_not_proven")
if control_mode == "walk":
    if "StateMachine transitioned on [/stand_default_mode] -> STAND_DEFAULT" not in controller_log:
        failure.append("stand_default_transition_before_locomotion_not_proven")
    locomotion_transition_markers = (
        "StateMachine transitioned on [/locomotion_default_mode] -> LOCOMOTION_DEFAULT",
        "Auto-switched to state 'LOCOMOTION_DEFAULT' via trigger '/locomotion_default_mode'",
    )
    if not any(marker in controller_log for marker in locomotion_transition_markers):
        failure.append("locomotion_controller_transition_not_proven")
    if abs(float(walk_speed)) < 0.3:
        failure.append("walking_command_below_vendor_minimum")
walk_timing = None
cmd_vel_fixture = None
turn_metrics = None
if control_mode == "walk":
    timing_path = run / "walk-timing.json"
    if not timing_path.is_file():
        failure.append("walk_sim_time_receipt_missing")
    else:
        walk_timing = json.loads(timing_path.read_text())
        if walk_timing.get("schema") != "sceneforge/x2-physx-sim-time-walk-window/v1":
            failure.append("walk_sim_time_receipt_schema")
        if walk_timing.get("status") != "PASSED":
            failure.append("walk_sim_time_receipt_failed")
        if walk_timing.get("real_robot_commands") is not False:
            failure.append("walk_sim_time_real_robot_flag")
        if float(walk_timing.get("observed_walk_duration_s", 0.0)) < float(walk_duration):
            failure.append("walk_sim_time_duration_below_requested")
        if float(walk_timing.get("observed_stop_settle_s", 0.0)) < float(walk_stop_settle_s):
            failure.append("walk_stop_settle_below_requested")
        expected_distance = float(walk_timing.get("expected_distance_m", 0.0))
        forward_displacement = float(walk_timing.get("forward_displacement_m", 0.0))
        lateral_displacement = float(walk_timing.get("lateral_displacement_m", 0.0))
        if abs(float(walk_yaw_rate)) < 1e-6 and abs(float(walk_lateral_speed)) < 1e-6:
            if float(walk_timing.get("forward_displacement_ratio", 0.0)) < 0.5:
                failure.append("walking_forward_displacement_ratio_below_limit")
            if abs(lateral_displacement) > max(0.15, expected_distance * 0.25):
                failure.append("walking_lateral_drift_above_limit")
        else:
            planar_displacement_ratio = math.hypot(forward_displacement, lateral_displacement) / expected_distance if expected_distance > 0.0 else 0.0
            walk_timing["planar_displacement_ratio"] = planar_displacement_ratio
            if planar_displacement_ratio < 0.3:
                failure.append("turn_planar_displacement_ratio_below_limit")
        settled = walk_timing.get("settled") or {}
        if float(settled.get("body_linear_speed_mps", float("inf"))) > 0.05:
            failure.append("walking_final_linear_speed_above_limit")
    fixture_path = run / "cmd-vel-fixture-receipt.json"
    if not fixture_path.is_file():
        failure.append("cmd_vel_fixture_receipt_missing")
    else:
        cmd_vel_fixture = json.loads(fixture_path.read_text())
        if cmd_vel_fixture.get("schema") != "sceneforge/x2-sim-cmd-vel-fixture/v1":
            failure.append("cmd_vel_fixture_receipt_schema")
        if cmd_vel_fixture.get("status") != "SUCCEEDED":
            failure.append("cmd_vel_fixture_not_succeeded")
        zero_sequence = 3 if float(walk_deceleration_duration) > 0.0 else 2
        if cmd_vel_fixture.get("last_sequence") != zero_sequence:
            failure.append("cmd_vel_fixture_zero_sequence_missing")
        if cmd_vel_fixture.get("errors"):
            failure.append("cmd_vel_fixture_errors")
        if cmd_vel_fixture.get("real_robot_commands") is not False:
            failure.append("cmd_vel_fixture_real_robot_flag")
        heading_hold = cmd_vel_fixture.get("heading_hold") or {}
        if heading_hold_enabled == "true":
            if heading_hold.get("enabled") is not True:
                failure.append("heading_hold_not_enabled")
            if int(heading_hold.get("odom_message_count", 0)) < 1:
                failure.append("heading_hold_no_odometry")
            if not math.isfinite(float(heading_hold.get("max_abs_heading_correction_rps", float("nan")))):
                failure.append("heading_hold_correction_not_finite")
        updates = cmd_vel_fixture.get("command_updates") or []
        nonzero_update = next((item for item in updates if item.get("sequence") == 1), None)
        zero_update = next((item for item in updates if item.get("sequence") == zero_sequence), None)
        trajectory = adapter.get("trajectory") or []
        if nonzero_update and zero_update and trajectory and all("yaw_rad" in item for item in trajectory):
            start_wall = float(nonzero_update["observed_at_unix"])
            stop_wall = float(zero_update["observed_at_unix"])
            start_sample = min(trajectory, key=lambda item: abs(float(item["stamp_s"]) - start_wall))
            stop_sample = min(trajectory, key=lambda item: abs(float(item["stamp_s"]) - stop_wall))
            yaw_delta = (float(stop_sample["yaw_rad"]) - float(start_sample["yaw_rad"]) + 3.141592653589793) % (2.0 * 3.141592653589793) - 3.141592653589793
            expected_yaw = float(walk_yaw_rate) * float(walk_duration)
            turn_metrics = {
                "start_stamp_s": start_sample["stamp_s"],
                "stop_stamp_s": stop_sample["stamp_s"],
                "start_yaw_rad": start_sample["yaw_rad"],
                "stop_yaw_rad": stop_sample["yaw_rad"],
                "observed_yaw_delta_rad": yaw_delta,
                "expected_yaw_delta_rad": expected_yaw,
            }
            if abs(float(walk_yaw_rate)) < 1e-6:
                if abs(yaw_delta) > 0.35:
                    failure.append("straight_walk_yaw_drift_above_limit")
            elif yaw_delta * expected_yaw <= 0.0 or abs(yaw_delta) < abs(expected_yaw) * 0.4:
                failure.append("turn_yaw_response_below_limit")
        else:
            failure.append("walk_yaw_evidence_missing")
if record_video == "true":
    video_receipt_path = run / "video-receipt.json"
    if not video_receipt_path.is_file():
        failure.append("video_receipt_missing")
    else:
        video_receipt = json.loads(video_receipt_path.read_text())
        if video_receipt.get("status") != "SUCCEEDED": failure.append("video_not_succeeded")
    if not (run / "x2-physx-walk-source-30fps.mp4").is_file(): failure.append("video_missing")
render_state_receipt = None
if record_state_trace == "true":
    state_receipt_path = run / "render-state-receipt.json"
    state_trace_path = run / "render-state.jsonl"
    if not state_receipt_path.is_file() or not state_trace_path.is_file():
        failure.append("render_state_trace_missing")
    else:
        render_state_receipt = json.loads(state_receipt_path.read_text())
        if render_state_receipt.get("status") != "SUCCEEDED": failure.append("render_state_trace_failed")
        if render_state_receipt.get("published_topics") != []: failure.append("render_state_recorder_published_topics")
        if render_state_receipt.get("real_robot_commands") is not False: failure.append("render_state_real_robot_flag")
if base_mode == "floating":
    if float(adapter.get("min_base_z") or 0.0) < 0.45:
        failure.append("floating_base_height_below_limit")
    if float(adapter.get("max_base_tilt_rad") or 0.0) > 0.55:
        failure.append("floating_base_tilt_above_limit")
    if adapter_command_mode == "effort_pd" and "EXTERNAL_EFFORT_HANDOVER_COMPLETE" not in engine_log:
        failure.append("external_effort_handover_not_proven")
band_status = None
band_release_proof = None
controller_policy_manifest = None
policy_manifest_path = run / "controller-policy-manifest.json"
if policy_manifest_path.is_file():
    controller_policy_manifest = json.loads(policy_manifest_path.read_text())
    if controller_policy_manifest.get("real_robot_commands") is not False:
        failure.append("controller_policy_manifest_real_robot_flag")
    runtime_gait = controller_policy_manifest.get("runtime_gait") or {}
    configured_enter = controller_policy_manifest.get("gait1_to_gait2_speed_threshold_mps")
    if control_mode == "walk" and configured_enter is not None and float(configured_enter) >= 0.0:
        gait_counts = runtime_gait.get("moving_current_gait_counts") or {}
        if int(gait_counts.get("2", 0)) < 25:
            failure.append("runtime_gait2_not_proven")
        if int(gait_counts.get("1", 0)) < 25:
            failure.append("runtime_gait1_handoff_not_proven")
if elastic_band_enabled == "true":
    if "ELASTIC_BAND_INITIALIZED" not in engine_log:
        failure.append("elastic_band_not_initialized")
    status_path = run / "elastic-band-status.json"
    proof_path = run / "elastic-band-release-proof.json"
    telemetry_path = run / "elastic-band-telemetry.jsonl"
    if not status_path.is_file():
        failure.append("elastic_band_status_missing")
    else:
        band_status = json.loads(status_path.read_text())
        if band_status.get("state") != "RELEASED": failure.append("elastic_band_not_released")
        if band_status.get("support_ratio") != 0.0: failure.append("elastic_band_support_not_zero")
        if band_status.get("force_norm_n") != 0.0: failure.append("elastic_band_force_not_zero")
        if band_status.get("root_pinned") is not False: failure.append("elastic_band_root_pin_violation")
        if band_status.get("gravity_modified") is not False: failure.append("elastic_band_gravity_violation")
    if not proof_path.is_file():
        failure.append("elastic_band_release_proof_missing")
    else:
        band_release_proof = json.loads(proof_path.read_text())
        if band_release_proof.get("status") != "PASSED": failure.append("elastic_band_release_proof_failed")
        if control_mode == "walk" and band_release_proof.get("locomotion_triggered") is not True:
            failure.append("locomotion_not_ordered_after_band_release")
    if not telemetry_path.is_file() or telemetry_path.stat().st_size == 0:
        failure.append("elastic_band_telemetry_missing")
    stop_proof_path = run / "engine-stop-proof.json"
    if control_mode in ("stand_default", "walk"):
        if not stop_proof_path.is_file():
            failure.append("physics_safe_stop_proof_missing")
        else:
            stop_proof = json.loads(stop_proof_path.read_text())
            if stop_proof.get("status") != "PASSED": failure.append("physics_safe_stop_proof_failed")
            if stop_proof.get("physics_stopped_before_adapter") is not True:
                failure.append("physics_not_stopped_before_adapter")
        final_body = (band_status or {}).get("body_position_m") or []
        if len(final_body) != 3 or float(final_body[2]) < 0.5:
            failure.append("final_torso_height_below_limit")
result = {
    "schema": "sceneforge/x2-geniesim-fixed-controller-gate/v1",
    "status": "PASSED" if not failure else "FAILED",
    "failure": failure,
    "physics_authority": "genie_sim_isaac_physx",
    "adapter_command_mode": adapter_command_mode,
    "adapter_publish_mode": adapter_publish_mode,
    "effort_gain_scale": float(effort_gain_scale),
    "video_requested": record_video == "true",
    "render_state_trace_requested": record_state_trace == "true",
    "render_state_trace": render_state_receipt,
    "clock_freeze_probe": {
        "enabled": clock_freeze_probe_enabled == "true",
        "requested_wall_duration_s": float(clock_freeze_duration_s),
        "receipt": clock_freeze_probe,
    },
    "elastic_band": {
        "enabled": elastic_band_enabled == "true",
        "implementation": "physx_external_spring_damper_force_on_torso_link" if elastic_band_enabled == "true" else None,
        "ramp_duration_s": float(elastic_band_ramp_s) if elastic_band_enabled == "true" else None,
        "unsupported_hold_s": float(elastic_band_unsupported_hold_s) if elastic_band_enabled == "true" else None,
        "status": band_status,
        "release_proof": band_release_proof,
        "qualification_requires_release_before_locomotion": True,
    },
    "test_scope": f"{base_mode}_pd_idle_controller_transport" if control_mode == "idle" else (f"{base_mode}_pd_stand_controller_response" if control_mode == "stand" else (f"{base_mode}_locomotion_controller_response" if control_mode == "walk" else f"{base_mode}_rl_stand_controller_response")),
    "qualification": False,
    "ros_domain_id": int(domain),
    "engine_image": engine_image,
    "controller_image": control_image,
    "controller_policy": controller_policy_manifest,
    "adapter_metrics": adapter,
    "walking_command": {
        "speed_mps": float(walk_speed),
        "lateral_speed_mps": float(walk_lateral_speed),
        "yaw_rate_rps": float(walk_yaw_rate),
        "duration_s": float(walk_duration),
        "duration_clock": "authoritative_physx_sim_time",
        "stop_settle_s": float(walk_stop_settle_s),
        "deceleration_speed_mps": float(walk_deceleration_speed) if float(walk_deceleration_duration) > 0.0 else None,
        "deceleration_duration_s": float(walk_deceleration_duration),
        "heading_hold": {
            "enabled": heading_hold_enabled == "true",
            "kp": float(heading_hold_kp) if heading_hold_enabled == "true" else None,
            "max_correction_rps": float(heading_hold_max_correction) if heading_hold_enabled == "true" else None,
            "source": "authoritative_physx_odom" if heading_hold_enabled == "true" else None,
        },
    } if control_mode == "walk" else None,
    "walk_timing": walk_timing,
    "cmd_vel_fixture": cmd_vel_fixture,
    "turn_metrics": turn_metrics,
    "engine_metrics": {
        "requested_physics_hz": float(requested_physics_hz),
        "actual_hz_samples": actual_hz,
        "actual_hz_min": min(actual_hz) if actual_hz else None,
        "actual_hz_last": actual_hz[-1] if actual_hz else None,
        "wall_realtime_factor_samples": [value / float(requested_physics_hz) for value in actual_hz],
        "render_body_tf_hz": float(render_body_tf_hz),
        "isaac_viewport_updates": int(isaac_viewport_updates),
        "frequency_qualified": False,
    },
    "limitations": [
        "fixed_base_only" if base_mode == "fixed" else "floating_base_development_gate_not_qualified",
        "pd_idle_only_no_stand_or_locomotion" if control_mode == "idle" else ("floating_walk_development_gate_not_repeated_qualification" if control_mode == "walk" else f"{base_mode}_stand_response_only_no_locomotion_qualification"),
        "vendor_effort_pd_is_applied_after_an_atomic_startup-drive_handover",
        "imu_linear_acceleration_is_physx_odometry_finite_difference_not_a_native_imu_sensor",
        "physx_wall_throughput_below_realtime_but_controller_is_required_to_follow_sim_time",
        "usd_asset_was_generated_with_isaac_5_1_and_only_compatibility_loaded_in_isaac_6",
        "virtual_suspension_is_transition_assistance_only_not_locomotion_evidence" if elastic_band_enabled == "true" else "no_virtual_suspension_used",
    ],
    "real_robot_commands": False,
    "finished_at_unix": time.time(),
}
evidence.mkdir(parents=True, exist_ok=False)
for path in run.iterdir():
    if path.is_file(): shutil.copy2(path, evidence / path.name)
(evidence / "result.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
result["artifacts"] = {
    path.name: {"size": path.stat().st_size, "sha256": digest(path)}
    for path in evidence.iterdir() if path.is_file() and path.name != "result.json"
}
(evidence / "result.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
print(evidence / "result.json")
if failure: raise SystemExit(1)
PY

trap - EXIT INT TERM
cleanup
