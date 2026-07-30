#!/usr/bin/env python3
"""
Dry-run a candidate CAM2HEAD matrix against camera-frame vision points.

This script is read-only. It never sends robot motion commands.
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import sys
from typing import Any

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot_grasp.coordinate_utils import make_transform


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MATRIX = os.path.join(
    SCRIPT_DIR,
    "calibration",
    "cam2head_vendor_20260724.json",
)


def _load_transform(path: str) -> np.ndarray:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if "transform_4x4" in data:
        matrix = data["transform_4x4"]
    else:
        matrix = data
    return make_transform(matrix, "cam2head")


def _latest_grasp_csv() -> str | None:
    files = glob.glob(os.path.join(PROJECT_ROOT, "data", "grasp_data_*.csv"))
    return max(files, key=os.path.getmtime) if files else None


def _load_latest_objects(path: str) -> list[dict[str, Any]]:
    with open(path, newline="", encoding="utf-8") as f:
        rows = [row for row in csv.DictReader(f) if row.get("type") == "perf"]
    latest = next(
        (row.get("object_results", "") for row in reversed(rows) if row.get("object_results", "")),
        "",
    )
    if not latest:
        return []
    return json.loads(latest)


def _validate_rotation(transform: np.ndarray) -> tuple[float, float]:
    rotation = transform[:3, :3]
    det = float(np.linalg.det(rotation))
    ortho_error = float(np.linalg.norm(rotation.T @ rotation - np.eye(3)))
    return det, ortho_error


def _transform_point(transform: np.ndarray, point_m: np.ndarray) -> np.ndarray:
    point = np.array([point_m[0], point_m[1], point_m[2], 1.0], dtype=float)
    return (transform @ point)[:3]


def _print_point(label: str, point_m: np.ndarray):
    print(
        f"{label}: "
        f"x={point_m[0]: .4f} m, "
        f"y={point_m[1]: .4f} m, "
        f"z={point_m[2]: .4f} m"
    )


def _object_camera_point(obj: dict[str, Any]) -> np.ndarray | None:
    if not obj.get("valid"):
        return None
    try:
        return np.array(
            [
                float(obj["x_mm"]) / 1000.0,
                float(obj["y_mm"]) / 1000.0,
                float(obj["z_mm"]) / 1000.0,
            ],
            dtype=float,
        )
    except (KeyError, TypeError, ValueError):
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix", default=DEFAULT_MATRIX)
    parser.add_argument(
        "--point-mm",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="Camera-frame point from depth_utils, in millimeters.",
    )
    parser.add_argument(
        "--latest-csv",
        action="store_true",
        help="Use the latest data/grasp_data_*.csv object_results.",
    )
    parser.add_argument(
        "--csv",
        default=None,
        help="Use object_results from a specific grasp_data CSV.",
    )
    parser.add_argument(
        "--show-inverse",
        action="store_true",
        help="Also show inverse-transform result for direction sanity check.",
    )
    args = parser.parse_args()

    transform = _load_transform(args.matrix)
    det, ortho_error = _validate_rotation(transform)

    print("=" * 70)
    print("  CAM2HEAD candidate dry-run")
    print("=" * 70)
    print(f"matrix: {args.matrix}")
    print(f"rotation det: {det:.6f}")
    print(f"orthogonality error: {ortho_error:.6e}")
    print(
        "translation: "
        f"x={transform[0, 3]:.4f} m, "
        f"y={transform[1, 3]:.4f} m, "
        f"z={transform[2, 3]:.4f} m"
    )
    if abs(det - 1.0) > 0.01 or ortho_error > 0.01:
        print("[!] rotation is not a clean rigid transform; verify the matrix before use.")
    else:
        print("[✓] rotation looks like a valid rigid transform.")

    points: list[tuple[str, np.ndarray]] = []
    if args.point_mm:
        points.append(("manual", np.array(args.point_mm, dtype=float) / 1000.0))

    csv_path = args.csv
    if args.latest_csv:
        csv_path = csv_path or _latest_grasp_csv()
        if not csv_path:
            print("[!] no data/grasp_data_*.csv found")
        else:
            print(f"\nlatest csv: {csv_path}")

    if csv_path:
        objects = _load_latest_objects(csv_path)
        for obj in objects:
            point_m = _object_camera_point(obj)
            if point_m is None:
                continue
            label = f"#{obj.get('idx')} {obj.get('label')} conf={obj.get('conf')}"
            points.append((label, point_m))

    if not points:
        print("\n没有输入点。示例:")
        print("  python handeye_calibration/test_cam2head_candidate.py --latest-csv")
        print("  python handeye_calibration/test_cam2head_candidate.py --point-mm 100 150 1200")
        return

    inverse = np.linalg.inv(transform)
    print("\n坐标转换结果:")
    for label, point_cam in points:
        point_head = _transform_point(transform, point_cam)
        print(f"\n{label}")
        _print_point("  camera", point_cam)
        _print_point("  head  ", point_head)
        if args.show_inverse:
            inverse_result = _transform_point(inverse, point_cam)
            _print_point("  inverse-as-test", inverse_result)

    print("\n判断方法:")
    print("  1. head 坐标数量级应在机器人头部附近，不能离谱到数米外。")
    print("  2. 左右移动物体时，HEAD 坐标的横向轴应随之单调变化。")
    print("  3. 前后移动物体时，HEAD 坐标的前向/深度相关轴应随之单调变化。")
    print("  4. 通过真实工装或尺子验证前，不要用这个候选矩阵发运动命令。")


if __name__ == "__main__":
    main()
