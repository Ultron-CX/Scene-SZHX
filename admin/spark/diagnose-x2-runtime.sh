#!/usr/bin/env bash
set -euo pipefail

root="${ROBOEASE_X2_ROOT:-/workspace/third_party/x2-release-linux}"
report="${ROBOEASE_X2_DIAGNOSTIC_REPORT:-/workspace/evidence/x2-runtime-dependencies.txt}"

set +u
source /opt/ros/humble/setup.bash
source "$root/install/setup.bash"
set -u
export LD_LIBRARY_PATH="$root/runtime_lib:${LD_LIBRARY_PATH:-}"

mkdir -p "$(dirname "$report")"
tmp="${report}.partial.$$"
trap 'rm -f -- "$tmp"' EXIT

{
  printf 'schema=roboease-x2-runtime-diagnostic/v1\n'
  printf 'real_robot_commands=false\n'
  printf 'architecture=%s\n' "$(uname -m)"
  printf 'x2_root=%s\n' "$root"
  printf 'ld_library_path=%s\n' "$LD_LIBRARY_PATH"
  for executable in \
    "$root/install/x1_sim/lib/x1_sim/x1_sim_node" \
    "$root/install/x1_rl_control/lib/x1_rl_control/x1_rl_control_node"
  do
    printf '\n[executable] %s\n' "$executable"
    test -x "$executable"
    ldd "$executable"
  done
} | tee "$tmp"

if grep -Eq 'not found|GLIBCXX_[0-9.]+.*not found' "$tmp"; then
  printf '\nstatus=FAILED\n' | tee -a "$tmp"
  mv -f -- "$tmp" "$report"
  trap - EXIT
  exit 1
fi

printf '\nstatus=PASSED\n' | tee -a "$tmp"
mv -f -- "$tmp" "$report"
trap - EXIT
