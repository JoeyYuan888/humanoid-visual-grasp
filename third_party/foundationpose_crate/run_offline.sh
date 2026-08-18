#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${FOUNDATIONPOSE_IMAGE:-shingarey/foundationpose_custom_cuda121:latest}"

docker run --rm --gpus all --ipc=host --network=host \
  -e NVIDIA_VISIBLE_DEVICES="${NVIDIA_VISIBLE_DEVICES:-all}" \
  -v "$ROOT:/workspace/foundationpose_cad" \
  -w /workspace/foundationpose_cad \
  "$IMAGE" \
  bash -lc 'python run_single.py "$@"' bash "$@"
