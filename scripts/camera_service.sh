#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${ROOT}/.venv/bin/python"
CONFIG="${ROOT}/configs/system/vlai_l1.toml"
ACTION="${1:-start}"

if [[ ! -x "${PYTHON}" ]]; then
  echo "FAIL repository environment is missing: ${PYTHON}" >&2
  exit 2
fi

case "${ACTION}" in
  start)
    STATE_DIR="$(
      "${PYTHON}" - "${CONFIG}" <<'PY'
import sys
from pathlib import Path

from vlai_l1_runtime.configuration import load_system_config

print(load_system_config(Path(sys.argv[1])).camera_preview.bridge_socket_path.parent)
PY
    )"
    sudo -n install -d -m 0755 -o "$(id -un)" -g "$(id -gn)" "${STATE_DIR}"
    ;;
  stop|status|logs) ;;
  *)
    echo "FAIL usage: $0 [start|stop|status|logs]" >&2
    exit 2
    ;;
esac

exec "${PYTHON}" -m vlai_l1_runtime.cli camera-service \
  --config "${CONFIG}" "${ACTION}"
