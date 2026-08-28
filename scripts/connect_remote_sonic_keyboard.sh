#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_FILE="${CONFIG_FILE:-$ROOT/config/pico_wbc_split.env}"
[[ -f "$CONFIG_FILE" ]] || {
  echo "Missing $CONFIG_FILE; copy config/pico_wbc_split.env.example first." >&2
  exit 1
}
set -a
# shellcheck disable=SC1090
source "$CONFIG_FILE"
set +a
: "${WBC_HOST:?WBC_HOST is required in $CONFIG_FILE}"
: "${REMOTE_USER:?REMOTE_USER is required in $CONFIG_FILE}"
: "${REMOTE_REPO:?REMOTE_REPO is required in $CONFIG_FILE}"
printf -v remote_repo_q '%q' "$REMOTE_REPO"

exec ssh -tt -o StrictHostKeyChecking=accept-new "$REMOTE_USER@$WBC_HOST" \
  "bash -lc 'cd $remote_repo_q && set -a && source config/pico_wbc_split.env && set +a && exec ./scripts/run_remote_sonic_wbc.sh --keyboard'"
