#!/usr/bin/env bash
set -euo pipefail

# Reproducible, user-scoped local environment for PICO + MuJoCo.  This does
# not write system Python or /opt; the PC Service is extracted under ~/.local.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONDA_BIN="${CONDA_EXE:-$HOME/miniforge3/bin/conda}"
ENV_NAME="groot-wbc-pico"
SERVICE_VERSION="1.0.0"
SERVICE_ASSET="XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb"
SERVICE_URL="https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/$SERVICE_ASSET"
SERVICE_SHA256="61961067eb4b41f81ed7cae35f4690dbb0ddfefb329a12b24e0b90ebc46ada91"
CACHE_DIR="$HOME/.cache/gr00t-wbc-deps"
SERVICE_INSTALL="$HOME/.local/share/xrobotoolkit-pc-service/$SERVICE_VERSION"

[[ -x "$CONDA_BIN" ]] || { echo "Conda not found: $CONDA_BIN" >&2; exit 1; }
cd "$ROOT"
if "$CONDA_BIN" env list | awk '{print $1}' | grep -Fxq "$ENV_NAME"; then
  "$CONDA_BIN" env update -n "$ENV_NAME" -f environment.pico-wbc.yml --prune
else
  "$CONDA_BIN" env create -f environment.pico-wbc.yml
fi

# The vendored XRoboToolkit binding does not declare pybind11 as a build
# dependency, so make its pinned CMake package explicit inside this Conda env.
PYBIND_CMAKE="$($CONDA_BIN run -n "$ENV_NAME" python -m pybind11 --cmakedir)"
CMAKE_PREFIX_PATH="$PYBIND_CMAKE" "$CONDA_BIN" run --no-capture-output -n "$ENV_NAME" \
  python -m pip install --no-build-isolation -e \
  external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64

mkdir -p "$CACHE_DIR" "$SERVICE_INSTALL"
if ! printf '%s  %s\n' "$SERVICE_SHA256" "$CACHE_DIR/$SERVICE_ASSET" \
  | sha256sum -c - >/dev/null 2>&1; then
  curl -fL -o "$CACHE_DIR/$SERVICE_ASSET.part" "$SERVICE_URL"
  printf '%s  %s\n' "$SERVICE_SHA256" "$CACHE_DIR/$SERVICE_ASSET.part" | sha256sum -c -
  mv "$CACHE_DIR/$SERVICE_ASSET.part" "$CACHE_DIR/$SERVICE_ASSET"
fi
printf '%s  %s\n' "$SERVICE_SHA256" "$CACHE_DIR/$SERVICE_ASSET" | sha256sum -c -
dpkg-deb -x "$CACHE_DIR/$SERVICE_ASSET" "$SERVICE_INSTALL"

"$CONDA_BIN" run --no-capture-output -n "$ENV_NAME" \
  python gear_sonic/scripts/download_pinned_sonic_v1_1.py --repo-root "$ROOT"
"$CONDA_BIN" run --no-capture-output -n "$ENV_NAME" python - <<'PY'
import mujoco, pinocchio, torch, xrobotoolkit_sdk
print("MuJoCo", mujoco.__version__)
print("Pinocchio", pinocchio.__version__)
print("PyTorch", torch.__version__, "CUDA", torch.cuda.is_available())
print("XRoboToolkit SDK", xrobotoolkit_sdk.__file__)
PY
echo "[local-setup] isolated local environment and PC Service are ready"
