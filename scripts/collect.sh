#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

"${ROOT}/scripts/camera_service.sh" start
exec "${ROOT}/.venv/bin/python" -m vlai_l1_runtime.cli collect "$@"
