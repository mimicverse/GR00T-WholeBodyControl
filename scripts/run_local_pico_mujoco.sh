#!/usr/bin/env bash
set -euo pipefail

# Local half: official 43-actuator MuJoCo scene, DDS bridge, and PICO manager.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_FILE="${CONFIG_FILE:-$ROOT/config/pico_wbc_split.env}"
if [[ -f "$CONFIG_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$CONFIG_FILE"
fi
CONDA_BIN="${CONDA_EXE:-$HOME/miniforge3/bin/conda}"
CONDA_ENV="${CONDA_ENV:-${LOCAL_CONDA_ENV:-groot-wbc-pico}}"
SIM_INTERFACE="${SIM_INTERFACE:-lo}"
WBC_HOST="${WBC_HOST:-}"
COMMAND_PORT="${COMMAND_PORT:-5556}"
FEEDBACK_PORT="${FEEDBACK_PORT:-5557}"
DDS_STATE_PORT="${DDS_STATE_PORT:-5560}"
DDS_COMMAND_PORT="${DDS_COMMAND_PORT:-5561}"
SIM_ONLY=0

usage() {
  echo "Usage: $0 [--sim-only] [--interface IFACE]"
}
while [[ $# -gt 0 ]]; do
  case "$1" in
    --sim-only) SIM_ONLY=1; shift ;;
    --interface) SIM_INTERFACE="${2:?--interface requires a name}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -x "$CONDA_BIN" ]] || { echo "Conda not found: $CONDA_BIN" >&2; exit 1; }
if [[ "$SIM_ONLY" == 0 && -z "$WBC_HOST" ]]; then
  echo "WBC_HOST is missing; copy and edit config/pico_wbc_split.env.example." >&2
  exit 1
fi
cd "$ROOT"
LOG_DIR="$ROOT/logs/pico_wbc_split/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$LOG_DIR"
export PYTHONUNBUFFERED=1
export XROBO_SERVICE_SCRIPT="${XROBO_SERVICE_SCRIPT:-$ROOT/scripts/start_pico_pc_service.sh}"

pids=()
cleanup() {
  trap - INT TERM EXIT
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  wait || true
}
trap cleanup EXIT
trap 'exit 143' INT TERM

setsid "$CONDA_BIN" run --no-capture-output -n "$CONDA_ENV" \
  python gear_sonic/scripts/dds_zmq_bridge.py \
    --role sim --dds-interface "$SIM_INTERFACE" \
    --state-port "$DDS_STATE_PORT" --command-port "$DDS_COMMAND_PORT" \
  >"$LOG_DIR/dds_bridge.log" 2>&1 &
pids+=("$!")

sleep 1
if ! kill -0 "${pids[0]}" 2>/dev/null; then
  wait "${pids[0]}" || true
  echo "[local] DDS bridge failed; see $LOG_DIR/dds_bridge.log" >&2
  exit 1
fi
setsid "$CONDA_BIN" run --no-capture-output -n "$CONDA_ENV" \
  python gear_sonic/scripts/run_sim_loop.py --interface "$SIM_INTERFACE" \
  >"$LOG_DIR/mujoco.log" 2>&1 &
pids+=("$!")

if [[ "$SIM_ONLY" == 0 ]]; then
  setsid "$CONDA_BIN" run --no-capture-output -n "$CONDA_ENV" \
    env XROBO_SERVICE_SCRIPT="$XROBO_SERVICE_SCRIPT" \
    python gear_sonic/scripts/pico_manager_thread_server.py \
      --manager --input-source xrt --port "$COMMAND_PORT" \
      --zmq_feedback_host "$WBC_HOST" --zmq_feedback_port "$FEEDBACK_PORT" \
    >"$LOG_DIR/pico_manager.log" 2>&1 &
  pids+=("$!")
fi

echo "[local] DDS bridge PID=${pids[0]}, ports=$DDS_STATE_PORT/$DDS_COMMAND_PORT"
echo "[local] MuJoCo viewer PID=${pids[1]}, DDS interface=$SIM_INTERFACE"
[[ "$SIM_ONLY" == 1 ]] || echo "[local] PICO manager PID=${pids[2]}, remote WBC=$WBC_HOST"
echo "[local] logs=$LOG_DIR"
while true; do
  for pid in "${pids[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
      wait "$pid" || true
      echo "[local] a component exited; stopping the remaining processes" >&2
      exit 1
    fi
  done
  sleep 0.5
done
