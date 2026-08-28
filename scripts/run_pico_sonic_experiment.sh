#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_FILE="${CONFIG_FILE:-$ROOT/config/pico_wbc_split.env}"
[[ -f "$CONFIG_FILE" ]] || {
  echo "Missing $CONFIG_FILE; copy config/pico_wbc_split.env.example first." >&2
  exit 1
}
# shellcheck disable=SC1090
source "$CONFIG_FILE"
CONDA_BIN="${CONDA_EXE:-$HOME/miniforge3/bin/conda}"
CONDA_ENV="${LOCAL_CONDA_ENV:-groot-wbc-pico}"
STATE_HOST="${WBC_HOST:?WBC_HOST is required in $CONFIG_FILE}"
PROTOCOL="baseline"
DURATION=""
NAME=""
OUTPUT_ROOT="$ROOT/logs/teleop_experiments"

usage() {
  cat <<'EOF'
Usage: ./scripts/run_pico_sonic_experiment.sh [options]

Record an already-running PICO + SONIC + MuJoCo chain and analyze it.

Options:
  --protocol NAME   baseline|static|right_arm|whole_body|hands|safety
  --duration SEC    Recording duration (protocol default if omitted)
  --name NAME       Run label (default: protocol + timestamp)
  --output-root DIR Experiment output root

The recorder is passive: it never creates a second XRoboToolkit client.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --protocol) PROTOCOL="${2:?}"; shift 2 ;;
    --duration) DURATION="${2:?}"; shift 2 ;;
    --name) NAME="${2:?}"; shift 2 ;;
    --output-root) OUTPUT_ROOT="${2:?}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

case "$PROTOCOL" in
  baseline) DEFAULT_DURATION=30; EXPECTED_MODE=PLANNER ;;
  static) DEFAULT_DURATION=90; EXPECTED_MODE=POSE ;;
  right_arm) DEFAULT_DURATION=75; EXPECTED_MODE=POSE ;;
  whole_body) DEFAULT_DURATION=150; EXPECTED_MODE=POSE ;;
  hands) DEFAULT_DURATION=60; EXPECTED_MODE=POSE ;;
  safety) DEFAULT_DURATION=30; EXPECTED_MODE=POSE ;;
  *) echo "Unknown protocol: $PROTOCOL" >&2; exit 2 ;;
esac
DURATION="${DURATION:-$DEFAULT_DURATION}"
RUN_ID="${NAME:-${PROTOCOL}-$(date +%Y%m%d-%H%M%S)}"
if [[ ! "$RUN_ID" =~ ^[A-Za-z0-9_.-]+$ ]]; then
  echo "Invalid run name '$RUN_ID': use letters, digits, dot, underscore or dash." >&2
  exit 2
fi
RUN_DIR="$OUTPUT_ROOT/$RUN_ID"
TRACE="$RUN_DIR/trace.sqlite3"
mkdir -p "$RUN_DIR"
[[ ! -e "$TRACE" ]] || { echo "Trace already exists: $TRACE" >&2; exit 1; }

[[ -x "$CONDA_BIN" ]] || { echo "Conda not found: $CONDA_BIN" >&2; exit 1; }

cat <<EOF
[experiment] protocol=$PROTOCOL duration=${DURATION}s
[experiment] output=$RUN_DIR
[experiment] Ensure the full chain is running.  Enter POSE for motion protocols.
EOF

case "$PROTOCOL" in
  baseline)
    echo "[protocol] Keep the robot in PLANNER/IDLE, band released, no PICO motion."
    ;;
  static)
    echo "[protocol] POSE: neutral 15s, arms forward 15s, arms side 15s, torso left/right 20s, neutral."
    ;;
  right_arm)
    echo "[protocol] POSE: neutral 10s, right-arm 0.5Hz side raise 20s, rest 10s, 1Hz 20s, neutral."
    ;;
  whole_body)
    echo "[protocol] POSE: neutral, left/right arm, torso yaw, shallow squat, in-place step; ~20s each."
    ;;
  hands)
    echo "[protocol] POSE: left trigger x5, right trigger x5, alternating x5, both x5; keep body still."
    ;;
  safety)
    echo "[protocol] Start in POSE, then stop/exit the PICO stream after 10s and wait for fail-closed STOP."
    ;;
esac

printf '\a'

echo "[preflight] requiring mode=$EXPECTED_MODE and a standing MuJoCo base"
"$CONDA_BIN" run --no-capture-output -n "$CONDA_ENV" \
  python "$ROOT/gear_sonic/scripts/check_pico_sonic_preflight.py" \
    --expected-mode "$EXPECTED_MODE" \
    --dds-interface "${SIM_INTERFACE:-lo}" \
    --output "$RUN_DIR/preflight.json"

marker_schedule() {
  sleep 1
  "$ROOT/scripts/send_trace_marker.sh" "${PROTOCOL}:begin"
  case "$PROTOCOL" in
    baseline)
      sleep 27; "$ROOT/scripts/send_trace_marker.sh" "baseline:end"
      ;;
    static)
      sleep 14; "$ROOT/scripts/send_trace_marker.sh" "static:arms_forward"
      sleep 15; "$ROOT/scripts/send_trace_marker.sh" "static:arms_side"
      sleep 15; "$ROOT/scripts/send_trace_marker.sh" "static:torso_yaw"
      sleep 20; "$ROOT/scripts/send_trace_marker.sh" "static:neutral"
      ;;
    right_arm)
      sleep 9; "$ROOT/scripts/send_trace_marker.sh" "right_arm:0.5Hz"
      sleep 20; "$ROOT/scripts/send_trace_marker.sh" "right_arm:rest"
      sleep 10; "$ROOT/scripts/send_trace_marker.sh" "right_arm:1Hz"
      sleep 20; "$ROOT/scripts/send_trace_marker.sh" "right_arm:neutral"
      ;;
    whole_body)
      sleep 9; "$ROOT/scripts/send_trace_marker.sh" "whole_body:left_arm"
      sleep 20; "$ROOT/scripts/send_trace_marker.sh" "whole_body:right_arm"
      sleep 20; "$ROOT/scripts/send_trace_marker.sh" "whole_body:torso_yaw"
      sleep 20; "$ROOT/scripts/send_trace_marker.sh" "whole_body:shallow_squat"
      sleep 20; "$ROOT/scripts/send_trace_marker.sh" "whole_body:in_place_step"
      sleep 20; "$ROOT/scripts/send_trace_marker.sh" "whole_body:neutral"
      ;;
    hands)
      sleep 9; "$ROOT/scripts/send_trace_marker.sh" "hands:right"
      sleep 10; "$ROOT/scripts/send_trace_marker.sh" "hands:alternating"
      sleep 15; "$ROOT/scripts/send_trace_marker.sh" "hands:both"
      sleep 15; "$ROOT/scripts/send_trace_marker.sh" "hands:neutral"
      ;;
    safety)
      sleep 9; "$ROOT/scripts/send_trace_marker.sh" "safety:stop_pico_now"
      ;;
  esac
}

READY_FILE="$RUN_DIR/recorder.ready.json"
RECORDER_PID=""
MARKER_PID=""
cleanup_children() {
  if [[ -n "$MARKER_PID" ]]; then
    kill "$MARKER_PID" 2>/dev/null || true
    wait "$MARKER_PID" 2>/dev/null || true
  fi
  if [[ -n "$RECORDER_PID" ]]; then
    kill "$RECORDER_PID" 2>/dev/null || true
    wait "$RECORDER_PID" 2>/dev/null || true
  fi
}
trap cleanup_children EXIT INT TERM

"$CONDA_BIN" run --no-capture-output -n "$CONDA_ENV" \
  python "$ROOT/gear_sonic/scripts/record_pico_sonic_trace.py" \
    --output "$TRACE" \
    --label "$PROTOCOL" \
    --duration "$DURATION" \
    --state-host "$STATE_HOST" \
    --dds-interface "${SIM_INTERFACE:-lo}" \
    --ready-file "$READY_FILE" \
    --metadata-json "{\"run_id\":\"$RUN_ID\",\"expected_mode\":\"$EXPECTED_MODE\"}" &
RECORDER_PID=$!

for _ in $(seq 1 200); do
  [[ -s "$READY_FILE" ]] && break
  if ! kill -0 "$RECORDER_PID" 2>/dev/null; then
    echo "[experiment] recorder exited before becoming ready" >&2
    wait "$RECORDER_PID" || true
    exit 1
  fi
  sleep 0.1
done
if [[ ! -s "$READY_FILE" ]]; then
  echo "[experiment] recorder did not become ready within 20 seconds" >&2
  exit 1
fi

marker_schedule &
MARKER_PID=$!

set +e
wait "$RECORDER_PID"
RECORDER_STATUS=$?
set -e
RECORDER_PID=""
if [[ "$RECORDER_STATUS" -ne 0 ]]; then
  echo "[experiment] recorder failed with status $RECORDER_STATUS" >&2
  exit "$RECORDER_STATUS"
fi

cleanup_children
MARKER_PID=""
trap - EXIT INT TERM

"$CONDA_BIN" run --no-capture-output -n "$CONDA_ENV" \
  python "$ROOT/gear_sonic/scripts/analyze_pico_sonic_trace.py" \
    "$TRACE" --output-dir "$RUN_DIR/analysis" \
    --model-xml "$ROOT/gear_sonic/data/robot_model/model_data/g1/scene_43dof.xml"

echo "[experiment] report: $RUN_DIR/analysis/summary.md"
