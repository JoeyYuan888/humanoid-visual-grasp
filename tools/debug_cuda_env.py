#!/usr/bin/env python3
"""
Inspect CUDA/PyTorch compatibility for the detect environment.

This script does not touch ROS or the robot. It only checks CUDA visibility and
runs a tiny tensor operation to catch "no kernel image" errors early.
"""

from __future__ import annotations

import os
import sys


def main():
    print("=" * 70)
    print("  CUDA / PyTorch environment check")
    print("=" * 70)
    print(f"python: {sys.executable}")
    print(f"CUDA_VISIBLE_DEVICES: {os.environ.get('CUDA_VISIBLE_DEVICES', '<unset>')}")

    try:
        import torch
    except Exception as exc:
        print(f"[x] import torch failed: {exc}")
        raise SystemExit(1)

    print(f"torch: {torch.__version__}")
    print(f"torch cuda build: {torch.version.cuda}")
    print(f"torch cuda available: {torch.cuda.is_available()}")
    print(f"torch cuda arch list: {torch.cuda.get_arch_list()}")
    print(f"torch device count: {torch.cuda.device_count()}")

    if not torch.cuda.is_available():
        print("[x] CUDA is not available to PyTorch.")
        raise SystemExit(1)

    for idx in range(torch.cuda.device_count()):
        print(f"\nGPU {idx}:")
        try:
            print(f"  name: {torch.cuda.get_device_name(idx)}")
            print(f"  capability: {torch.cuda.get_device_capability(idx)}")
        except Exception as exc:
            print(f"  [x] cannot query device info: {exc}")

    print("\nTiny CUDA tensor test:")
    try:
        x = torch.ones((16, 16), device="cuda")
        y = x @ x
        torch.cuda.synchronize()
        print(f"  [ok] tensor matmul result sum = {float(y.sum().item()):.1f}")
    except Exception as exc:
        print(f"  [x] tensor test failed: {type(exc).__name__}: {exc}")
        print("\n判断:")
        print("  如果这里也是 'no kernel image'，说明当前 PyTorch CUDA wheel")
        print("  和显卡架构不匹配。不要安装 CPU 版，应该换匹配的 CUDA/PyTorch 版本。")
        raise SystemExit(2)

    print("\n[ok] CUDA base tensor test passed.")


if __name__ == "__main__":
    main()
