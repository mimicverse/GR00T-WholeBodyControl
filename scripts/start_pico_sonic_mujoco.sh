#!/usr/bin/env bash
set -euo pipefail

# One-command launcher for the split-host PICO -> SONIC -> MuJoCo chain.
# Local: XRoboToolkit manager, MuJoCo viewer, sim-side DDS/ZMQ bridge.
# Remote: SONIC v1.1 TensorRT WBC and WBC-side DDS/ZMQ bridge.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_FILE="${CONFIG_FILE:-$ROOT/config/pico_wbc_split.env}"
RUN_ID="$(date +%Y%m%d-%H%M%S)"
RUN_LOG_DIR="$ROOT/logs/pico_wbc_split/orchestrator-$RUN_ID"

usage() {
  cat <<'EOF'
Usage: ./scripts/start_pico_sonic_mujoco.sh

Before starting:
  1. Put PICO XRoboToolkit-PICO in WORKING / Full Body mode.
  2. Keep the operator in the calibration pose.
  3. Enter the SSH password when prompted (it is never stored by this script).

Optional environment variables:
  CONFIG_FILE   Split-host configuration file.
  REMOTE_USER   SSH user from the configuration file.
  REMOTE_REPO   Repository path on the GPU host.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ $# -ne 0 ]]; then
  usage >&2
  exit 2
fi

[[ -f "$CONFIG_FILE" ]] || { echo "Configuration not found: $CONFIG_FILE" >&2; exit 1; }
set -a
# shellcheck disable=SC1090
source "$CONFIG_FILE"
set +a

: "${WBC_HOST:?WBC_HOST is required in $CONFIG_FILE}"
: "${SIM_HOST_IP:?SIM_HOST_IP is required in $CONFIG_FILE}"
: "${REMOTE_USER:?REMOTE_USER is required in $CONFIG_FILE}"
: "${REMOTE_REPO:?REMOTE_REPO is required in $CONFIG_FILE}"
[[ -x "$ROOT/scripts/run_local_pico_mujoco.sh" ]] || {
  echo "Local launcher is not executable" >&2
  exit 1
}
CONDA_BIN="${CONDA_EXE:-$HOME/miniforge3/bin/conda}"
[[ -x "$CONDA_BIN" ]] || {
  echo "Conda is missing: $CONDA_BIN" >&2
  echo "Run ./scripts/setup_local_pico_env.sh first." >&2
  exit 1
}
if ! "$CONDA_BIN" run -n "${LOCAL_CONDA_ENV:-groot-wbc-pico}" \
  python -c 'import mujoco, xrobotoolkit_sdk' >/dev/null; then
  echo "Local MuJoCo/XRoboToolkit Conda environment check failed." >&2
  echo "Run ./scripts/setup_local_pico_env.sh first." >&2
  exit 1
fi
[[ -f "$ROOT/gear_sonic/data/robot_model/model_data/g1/scene_43dof.xml" ]] || {
  echo "MuJoCo 43-DoF scene is missing" >&2
  exit 1
}

mkdir -p "$RUN_LOG_DIR"
LOCAL_LOG="$RUN_LOG_DIR/local_launcher.log"
REMOTE_LOG="$RUN_LOG_DIR/remote_wbc.log"
local_pid=""

cleanup() {
  trap - EXIT INT TERM HUP
  if [[ -n "$local_pid" ]] && kill -0 "$local_pid" 2>/dev/null; then
    kill -TERM -- "-$local_pid" 2>/dev/null || kill -TERM "$local_pid" 2>/dev/null || true
    wait "$local_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

echo "[all-in-one] run id: $RUN_ID"
echo "[all-in-one] logs: $RUN_LOG_DIR"
echo "[all-in-one] starting local PICO + MuJoCo stack..."

setsid "$ROOT/scripts/run_local_pico_mujoco.sh" >"$LOCAL_LOG" 2>&1 &
local_pid="$!"
sleep 3
if ! kill -0 "$local_pid" 2>/dev/null; then
  wait "$local_pid" || true
  echo "[all-in-one] local stack failed:" >&2
  tail -n 80 "$LOCAL_LOG" >&2 || true
  exit 1
fi
tail -n 10 "$LOCAL_LOG" || true

printf -v remote_repo_q '%q' "$REMOTE_REPO"
remote_command="cd $remote_repo_q && set -a && source config/pico_wbc_split.env && set +a && ./scripts/run_remote_sonic_wbc.sh --check && exec ./scripts/run_remote_sonic_wbc.sh"

echo "[all-in-one] starting SONIC on $REMOTE_USER@$WBC_HOST"
echo "[all-in-one] enter the SSH password when prompted"
echo "[all-in-one] wait for 'Init Done' before pressing A+B+X+Y"

set +e
ssh -tt \
  -o StrictHostKeyChecking=accept-new \
  -o ServerAliveInterval=5 \
  -o ServerAliveCountMax=3 \
  "$REMOTE_USER@$WBC_HOST" "$remote_command" 2>&1 | tee "$REMOTE_LOG"
ssh_status="${PIPESTATUS[0]}"
set -e

if [[ "$ssh_status" -ne 0 && "$ssh_status" -ne 130 ]]; then
  echo "[all-in-one] remote WBC/SSH exited with status $ssh_status" >&2
fi
echo "[all-in-one] stopping local stack"
exit "$ssh_status"
