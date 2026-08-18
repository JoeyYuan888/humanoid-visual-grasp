#!/usr/bin/env python3
"""Convert detected box grasp points from camera frame to BASE frame."""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from apps.grasp.visual_grasp_test_impl import (
    DEFAULT_CAM2HEAD,
    _connect,
    _load_transform,
    _lookup_transform,
    _sample_tf,
)


DEFAULT_INPUT = PROJECT_ROOT / "data" / "transport" / "box_grasp_target_latest.json"
DEFAULT_OUTPUT = PROJECT_ROOT / "data" / "runtime" / "transport_box_grasp_target_latest.json"


def _valid_objects(payload: dict) -> list[dict]:
    objects = []
    for obj in payload.get("objects", []):
        if obj.get("valid") is True and obj.get("status") == "valid" and "x_mm" in obj:
            objects.append(obj)
    return objects


def _camera_obj_to_base(obj: dict, cam2head: np.ndarray, head_to_base: np.ndarray) -> dict:
    point_cam = np.array(
        [
            float(obj["x_mm"]) / 1000.0,
            float(obj["y_mm"]) / 1000.0,
            float(obj["z_mm"]) / 1000.0,
            1.0,
        ],
        dtype=float,
    )
    point_head = cam2head @ point_cam
    point_base = head_to_base @ point_head
    return {
        "idx": obj.get("idx"),
        "label": obj.get("label", "blue_box"),
        "side": obj.get("side"),
        "confidence": float(obj.get("confidence", 1.0)),
        "camera_m": [float(v) for v in point_cam[:3]],
        "head_m": [float(v) for v in point_head[:3]],
        "base_m": [float(v) for v in point_base[:3]],
        "source": {
            "x_mm": float(obj["x_mm"]),
            "y_mm": float(obj["y_mm"]),
            "z_mm": float(obj["z_mm"]),
            "center": obj.get("center"),
            "bbox": obj.get("bbox"),
        },
    }


def _terminate_later(client, delay: float = 0.2) -> None:
    def run():
        time.sleep(delay)
        try:
            client.terminate()
        except Exception:
            pass

    threading.Thread(target=run, daemon=True).start()


def main() -> None:
    parser = argparse.ArgumentParser(description="Lock box grasp points in BASE frame.")
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument("--input", default=str(DEFAULT_INPUT))
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--cam2head", default=DEFAULT_CAM2HEAD)
    parser.add_argument("--tf-seconds", type=float, default=2.0)
    args = parser.parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output)
    with input_path.open("r", encoding="utf-8") as f:
        payload = json.load(f)

    objects = _valid_objects(payload)
    sides = {obj.get("side") for obj in objects}
    if "left" not in sides or "right" not in sides:
        raise RuntimeError(f"缺少有效左右抓取点: sides={sorted(s for s in sides if s)}")

    cam2head = _load_transform(args.cam2head, "cam2head")
    client = _connect(args.ws_url)
    try:
        print(f"[动作] 采样 TF/转换盒子抓取点开始 tf={args.tf_seconds:.1f}s")
        transforms = _sample_tf(client, args.tf_seconds)
        head_to_base = _lookup_transform(transforms, "BASE", "HEAD")
        if head_to_base is None:
            raise RuntimeError("TF 中没有找到 BASE -> HEAD，不能锁存盒子 BASE 抓取点")

        locked_objects = [_camera_obj_to_base(obj, cam2head, head_to_base) for obj in objects]
        by_side = {obj["side"]: obj for obj in locked_objects}
        out = {
            "type": "box_grasp_points_base",
            "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "input": str(input_path),
            "cam2head": str(args.cam2head),
            "frame": "BASE",
            "objects": locked_objects,
            "left": {
                "camera": by_side["left"]["camera_m"],
                "head": by_side["left"]["head_m"],
                "base": by_side["left"]["base_m"],
            },
            "right": {
                "camera": by_side["right"]["camera_m"],
                "head": by_side["right"]["head_m"],
                "base": by_side["right"]["base_m"],
            },
            "note": "Locked BASE box grasp points. Reuse after neck pose changes; do not recompute from old camera points with a different HEAD TF.",
        }
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"[✓] 已锁存盒子 BASE 抓取点: {output_path}")
        print(f"    left : [{by_side['left']['base_m'][0]:.4f}, {by_side['left']['base_m'][1]:.4f}, {by_side['left']['base_m'][2]:.4f}]")
        print(f"    right: [{by_side['right']['base_m'][0]:.4f}, {by_side['right']['base_m'][1]:.4f}, {by_side['right']['base_m'][2]:.4f}]")
    finally:
        _terminate_later(client)


if __name__ == "__main__":
    main()
