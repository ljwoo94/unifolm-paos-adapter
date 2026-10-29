#!/bin/sh
# Existing W8A8 checkpoint/runtime; no robot connection or RTC.
set -eu
repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$repo_dir"
export WMA_MODEL_BUNDLE="${WMA_MODEL_BUNDLE:-/home/wla/WMA-JLee/models/C-W8A8-20261028}"
model_python="${WMA_MODEL_PYTHON:-/home/wla/WMA-JLee/.venv311/bin/python}"
export WMA_TORCH_COMPILE="${WMA_TORCH_COMPILE:-1}"
export WMA_COMPILE_PROFILE="${WMA_COMPILE_PROFILE:-baseline}"
export PYTHONPATH="$repo_dir/src:$WMA_MODEL_BUNDLE/src${PYTHONPATH:+:$PYTHONPATH}"
export TORCHINDUCTOR_CACHE_DIR="${TORCHINDUCTOR_CACHE_DIR:-$repo_dir/.cache/$WMA_COMPILE_PROFILE}"
export TRITON_CACHE_DIR="${TRITON_CACHE_DIR:-$repo_dir/.cache/triton}"
export PYTORCH_CUDA_ALLOC_CONF="${PYTORCH_CUDA_ALLOC_CONF:-expandable_segments:True}"
if [ ! -x "$model_python" ] || [ ! -f "$WMA_MODEL_BUNDLE/model.safetensors" ]; then
  echo "Missing existing model Python or W8A8 checkpoint: $model_python / $WMA_MODEL_BUNDLE" >&2
  exit 1
fi
if [ "${1:-}" = --check ]; then
  printf 'model_python=%s\ncheckpoint=%s\ncompile=%s\n' "$model_python" "$WMA_MODEL_BUNDLE" "$WMA_COMPILE_PROFILE"
  exit 0
fi
exec "$model_python" -u model_runtime/server.py --host 127.0.0.1 --port 18610 "$@"
