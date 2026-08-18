#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${FOUNDATIONPOSE_IMAGE:-shingarey/foundationpose_custom_cuda121:latest}"
SHOW=false
FORWARD_ARGS=()

for arg in "$@"; do
  if [[ "$arg" == "--show" ]]; then
    SHOW=true
  else
    FORWARD_ARGS+=("$arg")
  fi
done

if [[ "$SHOW" == true ]]; then
  HOST_PYTHON="${PLASTIC_CRATE_PYTHON:-/home/yons/test_plastic_crate/.conda/envs/plastic-frame-yolo/bin/python}"
  if [[ ! -x "$HOST_PYTHON" ]]; then
    echo "[x] Host GUI Python is unavailable: $HOST_PYTHON" >&2
    exit 2
  fi
  exec "$HOST_PYTHON" "$ROOT/show_live.py" -- "${FORWARD_ARGS[@]}"
fi

docker run --rm --gpus all --ipc=host --network=host \
  -e NVIDIA_VISIBLE_DEVICES="${NVIDIA_VISIBLE_DEVICES:-all}" \
  -v "$ROOT:/workspace/foundationpose_cad" \
  -w /workspace/foundationpose_cad \
  "$IMAGE" \
  bash -lc 'python run_live.py "$@"' bash "${FORWARD_ARGS[@]}"
