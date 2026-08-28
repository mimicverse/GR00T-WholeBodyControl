#!/usr/bin/env bash
set -euo pipefail

LABEL="${1:?Usage: $0 LABEL [PORT]}"
PORT="${2:-5572}"
printf '%s' "$LABEL" >"/dev/udp/127.0.0.1/$PORT"
