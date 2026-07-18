#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PYTHON_BIN="${LEROBOT_RECORDING_PYTHON:-${PROJECT_ROOT}/.venv/bin/python}"
export PATH="$(dirname "${PYTHON_BIN}"):${PATH}"
export HF_HOME="${HF_HOME:-/tmp/quest3_streamer_hf_cache}"

exec "${PYTHON_BIN}" "${PROJECT_ROOT}/src/tools/visualize_lerobot_dataset.py" "$@"
