#!/usr/bin/env python3
"""
Deprecated experimental helper.

Solve a rigid camera -> MPC transform from clicked point pairs.

Input CSV is produced by collect_handeye_pairs.py.
The solved transform maps camera points in meters to MPC points in meters:

    p_mpc = R @ p_cam + t

This is not the official course-manual hand-eye route. The production pipeline
must use CAM2HEAD -> HEAD2BASE(tf) -> BASE -> GRASP_OFFSET.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from datetime import datetime

import numpy as np


DEPRECATED_DIR = os.path.dirname(os.path.abspath(__file__))


def load_pairs(path: str):
    cam = []
    mpc = []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            cam.append([float(row["cam_x_m"]), float(row["cam_y_m"]), float(row["cam_z_m"])])
            mpc.append([float(row["mpc_x_m"]), float(row["mpc_y_m"]), float(row["mpc_z_m"])])
    return np.asarray(cam, dtype=float), np.asarray(mpc, dtype=float)


def solve_rigid_transform(src: np.ndarray, dst: np.ndarray):
    if len(src) < 3:
        raise ValueError("至少需要 3 组非共线点；建议采 6-10 组")

    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_centered = src - src_mean
    dst_centered = dst - dst_mean

    h = src_centered.T @ dst_centered
    u, s, vt = np.linalg.svd(h)
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
    parser.add_argument("--allow-deprecated", action="store_true")
    args = parser.parse_args()

    if not args.allow_deprecated:
        raise SystemExit(
            "该脚本已废弃：不要用 camera->MPC 点对拟合作为正式手眼路线。\n"
            "正式路线请使用 CAM2HEAD -> HEAD2BASE(tf) -> BASE -> GRASP_OFFSET。\n"
            "如仅需查看历史实验结果，可显式添加 --allow-deprecated。"
        )

    cam, mpc = load_pairs(args.csv_path)
    r, t = solve_rigid_transform(cam, mpc)
    pred = (r @ cam.T).T + t
    residual = pred - mpc
    errors = np.linalg.norm(residual, axis=1)

    transform = np.eye(4)
    transform[:3, :3] = r
    transform[:3, 3] = t

    result = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "input_csv": args.csv_path,
        "description": "camera point meters -> MPC point meters, with fixed neck/camera pose",
        "num_pairs": int(len(cam)),
        "rmse_m": float(np.sqrt(np.mean(errors ** 2))),
        "max_error_m": float(errors.max()),
        "mean_error_m": float(errors.mean()),
        "rotation": r.tolist(),
        "translation_m": t.tolist(),
        "transform_4x4": transform.tolist(),
        "per_pair_error_m": errors.tolist(),
    }

    print("=" * 70)
    print("  Camera -> MPC rigid transform")
    print("=" * 70)
    print(f"pairs: {len(cam)}")
    print(f"rmse: {result['rmse_m']:.4f} m")
    print(f"mean: {result['mean_error_m']:.4f} m")
    print(f"max:  {result['max_error_m']:.4f} m")
    print("transform_4x4:")
    print(json.dumps(result["transform_4x4"], indent=2))

    if args.output:
        out_path = args.output
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(DEPRECATED_DIR, "calibration", f"camera_to_mpc_{ts}.json")

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print(f"[✓] 已保存: {out_path}")

    if result["rmse_m"] > 0.03:
        print("[!] RMSE > 3cm，建议重新采点或检查点击点/TCP 标记是否一致")


if __name__ == "__main__":
    main()
