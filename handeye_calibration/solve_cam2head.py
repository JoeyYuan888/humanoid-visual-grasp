#!/usr/bin/env python3
"""
Solve the official CAM2HEAD transform from 3D point pairs.

Input CSV must contain:
  cam_x_m, cam_y_m, cam_z_m
  head_x_m, head_y_m, head_z_m

The solved transform maps camera optical-frame points to HEAD-frame points:
  p_head = CAM2HEAD @ p_cam
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from datetime import datetime

import numpy as np


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def load_pairs(path: str):
    cam = []
    head = []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            cam.append([float(row["cam_x_m"]), float(row["cam_y_m"]), float(row["cam_z_m"])])
            head.append([float(row["head_x_m"]), float(row["head_y_m"]), float(row["head_z_m"])])
    return np.asarray(cam, dtype=float), np.asarray(head, dtype=float)


def solve_rigid_transform(src: np.ndarray, dst: np.ndarray):
    if len(src) < 3:
        raise ValueError("至少需要 3 组非共线点；建议采 8-12 组，覆盖图像中心和四周")

    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_centered = src - src_mean
    dst_centered = dst - dst_mean

    h = src_centered.T @ dst_centered
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = vt.T @ u.T
    t = dst_mean - r @ src_mean
    return r, t


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument("--output", default=None)
    args = parser.parse_args()

    cam, head = load_pairs(args.csv_path)
    r, t = solve_rigid_transform(cam, head)
    pred = (r @ cam.T).T + t
    residual = pred - head
    errors = np.linalg.norm(residual, axis=1)

    transform = np.eye(4)
    transform[:3, :3] = r
    transform[:3, 3] = t

    result = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "input_csv": args.csv_path,
        "description": "CAM2HEAD: camera optical-frame meters -> HEAD-frame meters",
        "num_pairs": int(len(cam)),
        "rmse_m": float(np.sqrt(np.mean(errors ** 2))),
        "mean_error_m": float(errors.mean()),
        "max_error_m": float(errors.max()),
        "rotation": r.tolist(),
        "translation_m": t.tolist(),
        "transform_4x4": transform.tolist(),
        "per_pair_error_m": errors.tolist(),
    }

    print("=" * 70)
    print("  CAM2HEAD solve result")
    print("=" * 70)
    print(f"pairs: {result['num_pairs']}")
    print(f"rmse: {result['rmse_m'] * 1000.0:.1f} mm")
    print(f"mean: {result['mean_error_m'] * 1000.0:.1f} mm")
    print(f"max:  {result['max_error_m'] * 1000.0:.1f} mm")
    print("transform_4x4:")
    print(json.dumps(result["transform_4x4"], indent=2))

    if args.output:
        out_path = args.output
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(SCRIPT_DIR, "calibration", f"cam2head_{ts}.json")

    directory = os.path.dirname(out_path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print(f"[✓] 已保存: {out_path}")

    if result["rmse_m"] > 0.02:
        print("[!] RMSE > 20mm，建议检查工装点位、点击精度、深度质量或 TCP/工装偏移")


if __name__ == "__main__":
    main()
