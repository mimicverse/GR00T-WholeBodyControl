#!/usr/bin/env bash
set -euo pipefail

# GPU-only half of the split-host SONIC v1.1 demo. MuJoCo and PICO stay on
# the local workstation; this process only runs the official C++ WBC stack.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_FILE="${CONFIG_FILE:-$ROOT/config/pico_wbc_split.env}"
if [[ -f "$CONFIG_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$CONFIG_FILE"
fi
CONDA_ENV="${CONDA_ENV:-${REMOTE_CONDA_ENV:-groot-wbc-sonic-trt1013}}"
ENV_ROOT="${ENV_ROOT:-$HOME/miniforge3/envs/$CONDA_ENV}"
SIM_HOST_IP="${SIM_HOST_IP:-}"
WBC_INTERFACE="${WBC_INTERFACE:-lo}"
WBC_INPUT_TYPE="${WBC_INPUT_TYPE:-zmq_manager}"
COMMAND_PORT="${COMMAND_PORT:-5556}"
FEEDBACK_PORT="${FEEDBACK_PORT:-5557}"
DDS_STATE_PORT="${DDS_STATE_PORT:-5560}"
DDS_COMMAND_PORT="${DDS_COMMAND_PORT:-5561}"

DEPLOY_ROOT="$ROOT/gear_sonic_deploy"
BIN="${WBC_BIN:-$DEPLOY_ROOT/target/release/g1_deploy_onnx_ref}"
DECODER="$DEPLOY_ROOT/policy/sonic_v1_1/model_decoder.onnx"
ENCODER="$DEPLOY_ROOT/policy/sonic_v1_1/model_encoder.onnx"
OBS_CONFIG="$DEPLOY_ROOT/policy/sonic_v1_1/observation_config.yaml"
PLANNER="$DEPLOY_ROOT/planner/target_vel/V2/planner_sonic.onnx"
MOTION_DATA="$DEPLOY_ROOT/reference/example/"
TRT_ROOT="$ENV_ROOT/tensorrt"
ORT_ROOT="${onnxruntime_ROOT:-$HOME/.local/onnxruntime-gpu}"
UNITREE_LIB="$DEPLOY_ROOT/thirdparty/unitree_sdk2/thirdparty/lib/x86_64"

if [[ "${1:-}" == "--keyboard" ]]; then
  WBC_INPUT_TYPE="keyboard"
  shift
fi

[[ -x "$BIN" ]] || { echo "WBC binary not found: $BIN" >&2; exit 1; }
[[ -x "$ENV_ROOT/bin/python" ]] || { echo "Conda environment not found: $ENV_ROOT" >&2; exit 1; }
for path in "$DECODER" "$ENCODER" "$OBS_CONFIG" "$PLANNER"; do
  [[ -f "$path" ]] || { echo "Missing model file: $path" >&2; exit 1; }
done
[[ -d "$MOTION_DATA" && -d "$TRT_ROOT/lib" && -d "$ORT_ROOT/lib" ]] || {
  echo "Missing motion data, TensorRT, or ONNX Runtime root" >&2
  exit 1
}

export CUDA_HOME="$ENV_ROOT"
export CUDAToolkit_ROOT="$ENV_ROOT"
export TensorRT_ROOT="$TRT_ROOT"
export onnxruntime_ROOT="$ORT_ROOT"
export LD_LIBRARY_PATH="$TRT_ROOT/lib:$ENV_ROOT/lib:$ORT_ROOT/lib:$UNITREE_LIB:${LD_LIBRARY_PATH:-}"

if [[ "${1:-}" == "--check" ]]; then
  "$ENV_ROOT/bin/python" - <<'PY'
import tensorrt as trt
import cyclonedds, unitree_sdk2py, zmq
print("TensorRT", trt.__version__)
assert trt.__version__ == "10.13.3.9"
print("DDS/ZMQ bridge imports OK")
PY
  "$ENV_ROOT/bin/nvcc" --version | tail -1
  "$BIN" >/dev/null
  ldd "$BIN" | grep -E 'nvinfer|nvonnxparser|cudart|onnxruntime|not found'
  echo "[remote] SONIC v1.1 preflight passed"
  exit 0
fi

[[ -n "$SIM_HOST_IP" ]] || {
  echo "SIM_HOST_IP is missing; copy and edit config/pico_wbc_split.env.example." >&2
  exit 1
}

cd "$DEPLOY_ROOT"
bridge_pid=""
cleanup() {
  trap - INT TERM EXIT
  if [[ -n "$bridge_pid" ]]; then
    kill -INT "$bridge_pid" 2>/dev/null || true
    wait "$bridge_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 143' INT TERM

"$ENV_ROOT/bin/python" "$ROOT/gear_sonic/scripts/dds_zmq_bridge.py" \
  --role wbc --peer-host "$SIM_HOST_IP" --dds-interface "$WBC_INTERFACE" \
  --state-port "$DDS_STATE_PORT" --command-port "$DDS_COMMAND_PORT" &
bridge_pid="$!"
sleep 1
if ! kill -0 "$bridge_pid" 2>/dev/null; then
  wait "$bridge_pid" || true
  echo "[remote] DDS bridge failed to start" >&2
  exit 1
fi

"$BIN" "$WBC_INTERFACE" "$DECODER" "$MOTION_DATA" \
  --obs-config "$OBS_CONFIG" \
  --encoder-file "$ENCODER" \
  --planner-file "$PLANNER" \
  --input-type "$WBC_INPUT_TYPE" \
  --output-type zmq \
  --zmq-host "$SIM_HOST_IP" \
  --zmq-port "$COMMAND_PORT" \
  --zmq-out-port "$FEEDBACK_PORT" \
  --disable-crc-check
