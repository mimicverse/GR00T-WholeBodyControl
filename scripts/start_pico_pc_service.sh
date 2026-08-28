#!/usr/bin/env bash
set -euo pipefail

# User-level launcher for XRoboToolkit PC Service.  The official .deb can be
# extracted under ~/.local when sudo is unavailable; override the root with
# XROBO_SERVICE_ROOT when using a system installation.
SERVICE_ROOT="${XROBO_SERVICE_ROOT:-$HOME/.local/share/xrobotoolkit-pc-service/1.0.0/opt/apps/roboticsservice}"
SERVICE_BIN="$SERVICE_ROOT/RoboticsServiceProcess"

if [[ ! -x "$SERVICE_BIN" ]]; then
    echo "XRoboToolkit PC Service not found: $SERVICE_BIN" >&2
    echo "Extract XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb there or set XROBO_SERVICE_ROOT." >&2
    exit 1
fi

# Reuse an already running instance.  This also covers a service installed by
# dpkg under /opt, avoiding a second process binding the SDK port.
if ss -ltn 2>/dev/null | awk '{print $4}' | grep -Eq '(^|:)60061$'; then
    echo "XRoboToolkit PC Service already listening on TCP 60061"
    exit 0
fi

export LD_LIBRARY_PATH="$SERVICE_ROOT:$SERVICE_ROOT/lib:$SERVICE_ROOT/SDK/x64:${LD_LIBRARY_PATH:-}"
export QT_PLUGIN_PATH="$SERVICE_ROOT/plugins:${QT_PLUGIN_PATH:-}"
export QT_QML_PATH="$SERVICE_ROOT/qml:${QT_QML_PATH:-}"
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"
fi
cd "$SERVICE_ROOT"
exec "$SERVICE_BIN"
