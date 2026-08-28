#!/usr/bin/env bash
set -euo pipefail

# Reproducible, user-scoped build environment for the GPU host.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONDA_BIN="${CONDA_EXE:-$HOME/miniforge3/bin/conda}"
ENV_NAME="groot-wbc-sonic-trt1013"
ENV_ROOT="$HOME/miniforge3/envs/$ENV_NAME"
TRT_VERSION="10.13.3.9"
CUDA_SUFFIX="cuda12.9"
NVIDIA_REPO="https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64"
CACHE_ROOT="$HOME/.cache/gr00t-wbc-deps"
ORT_ROOT="$HOME/.local/onnxruntime-gpu"

[[ -x "$CONDA_BIN" ]] || { echo "Conda not found: $CONDA_BIN" >&2; exit 1; }
cd "$ROOT"
mkdir -p "$CACHE_ROOT"
if [[ -x "$ENV_ROOT/bin/python" ]]; then
  "$CONDA_BIN" env update -n "$ENV_NAME" -f "$ROOT/environment.sonic-trt1013.yml"
else
  "$CONDA_BIN" env create -f "$ROOT/environment.sonic-trt1013.yml"
fi

# pip TensorRT contains the exact runtime libraries but no C++ headers. Extract
# NVIDIA's matching development packages into the same isolated environment.
TRT_STAGE="$ENV_ROOT/src/tensorrt-dev-$TRT_VERSION"
mkdir -p "$TRT_STAGE" "$ENV_ROOT/tensorrt/include"
for package in \
  "libnvinfer-headers-dev_${TRT_VERSION}-1+${CUDA_SUFFIX}_amd64.deb" \
  "libnvinfer-headers-plugin-dev_${TRT_VERSION}-1+${CUDA_SUFFIX}_amd64.deb" \
  "libnvonnxparsers-dev_${TRT_VERSION}-1+${CUDA_SUFFIX}_amd64.deb"; do
  if [[ ! -f "$CACHE_ROOT/$package" ]]; then
    curl -fL -o "$CACHE_ROOT/$package" "$NVIDIA_REPO/$package"
  fi
  dpkg-deb -x "$CACHE_ROOT/$package" "$TRT_STAGE"
done
cp -a "$TRT_STAGE/usr/include/x86_64-linux-gnu/." "$ENV_ROOT/tensorrt/include/"

TRT_PY_LIB="$ENV_ROOT/lib/python3.10/site-packages/tensorrt_libs"
[[ -d "$TRT_PY_LIB" ]] || { echo "TensorRT Python libraries missing: $TRT_PY_LIB" >&2; exit 1; }
[[ -e "$ENV_ROOT/tensorrt/lib" ]] || ln -s "$TRT_PY_LIB" "$ENV_ROOT/tensorrt/lib"
for library in nvinfer nvinfer_plugin nvonnxparser; do
  [[ -e "$TRT_PY_LIB/lib${library}.so" ]] || \
    ln -s "lib${library}.so.10" "$TRT_PY_LIB/lib${library}.so"
done

# Native ONNX Runtime is used by the official non-TensorRT helper paths.
ORT_ARCHIVE="$CACHE_ROOT/onnxruntime-linux-x64-gpu-1.19.2.tgz"
if [[ ! -f "$ORT_ROOT/lib/libonnxruntime.so" ]]; then
  [[ -f "$ORT_ARCHIVE" ]] || curl -fL -o "$ORT_ARCHIVE" \
    "https://github.com/microsoft/onnxruntime/releases/download/v1.19.2/onnxruntime-linux-x64-gpu-1.19.2.tgz"
  mkdir -p "$ORT_ROOT"
  tar -xzf "$ORT_ARCHIVE" --strip-components=1 -C "$ORT_ROOT"
fi

export PATH="$ENV_ROOT/bin:$PATH"
export CMAKE_PREFIX_PATH="$ENV_ROOT"
export CUDA_HOME="$ENV_ROOT"
export CUDAToolkit_ROOT="$ENV_ROOT"
export TensorRT_ROOT="$ENV_ROOT/tensorrt"
export onnxruntime_ROOT="$ORT_ROOT"
export HAS_ROS2=0
export LD_LIBRARY_PATH="$TensorRT_ROOT/lib:$ENV_ROOT/lib:$ORT_ROOT/lib:$ROOT/gear_sonic_deploy/thirdparty/unitree_sdk2/thirdparty/lib/x86_64:${LD_LIBRARY_PATH:-}"

"$ENV_ROOT/bin/python" "$ROOT/gear_sonic/scripts/download_pinned_sonic_v1_1.py" \
  --repo-root "$ROOT"

cd "$ROOT/gear_sonic_deploy"
cmake -S . -B build-trt1013 -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_C_COMPILER="$ENV_ROOT/bin/x86_64-conda-linux-gnu-cc" \
  -DCMAKE_CXX_COMPILER="$ENV_ROOT/bin/x86_64-conda-linux-gnu-c++"
cmake --build build-trt1013 --target g1_deploy_onnx_ref -j8
"$ROOT/scripts/run_remote_sonic_wbc.sh" --check
