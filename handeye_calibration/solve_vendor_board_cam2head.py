#!/usr/bin/env python3
"""
Solve CAM2HEAD from vendor hand-back board samples without trusting vendor T/R.

The collector saves:
  G_i = HEAD_T_TCP_i
  C_i = CAM_T_BOARD_i

The rigid relation is:
  X * C_i = G_i * Y

where:
  X = HEAD_T_CAM      (wanted CAM2HEAD)
  Y = TCP_T_BOARD     (also estimated for diagnostics)

For sample pairs i,j:
  (G_i * inv(G_j)) * X = X * (C_i * inv(C_j))

This script solves A X = X B with linear least squares.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from datetime import datetime

import numpy as np


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def _matrix_to_quat(matrix: np.ndarray) -> np.ndarray:
    m = matrix[:3, :3]
    trace = float(np.trace(m))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    else:
        idx = int(np.argmax(np.diag(m)))
        if idx == 0:
            s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif idx == 1:
            s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s
    quat = np.array([x, y, z, w], dtype=float)
    return quat / np.linalg.norm(quat)


def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _quat_left_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q / np.linalg.norm(q)
    return np.array(
        [
            [w, -z, y, x],
            [z, w, -x, y],
            [-y, x, w, z],
            [-x, -y, -z, w],
        ],
        dtype=float,
    )


def _quat_right_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q / np.linalg.norm(q)
    return np.array(
        [
            [w, z, -y, x],
            [-z, w, x, y],
            [y, -x, w, z],
            [-x, -y, -z, w],
        ],
        dtype=float,
    )


def _rotation_angle(a: np.ndarray, b: np.ndarray) -> float:
    rel = a[:3, :3].T @ b[:3, :3]
    value = (np.trace(rel) - 1.0) / 2.0
    value = max(-1.0, min(1.0, float(value)))
    return math.acos(value)


def _average_transform(matrices: list[np.ndarray]) -> np.ndarray:
    translations = np.array([m[:3, 3] for m in matrices], dtype=float)
    quats = np.array([_matrix_to_quat(m) for m in matrices], dtype=float)
    ref = quats[0]
    for i in range(len(quats)):
        if float(np.dot(ref, quats[i])) < 0:
            quats[i] *= -1.0
    mean_quat = quats.mean(axis=0)
    mean_quat /= np.linalg.norm(mean_quat)
    result = np.eye(4)
    result[:3, :3] = _quat_to_matrix(mean_quat)
    result[:3, 3] = translations.mean(axis=0)
    return result


def _load_samples(path: str):
    head_t_tcp = []
    cam_t_board = []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            g = np.eye(4)
            c = np.eye(4)
            for r in range(4):
                for col in range(4):
                    g[r, col] = float(row[f"head_t_tcp_{r}{col}"])
                    c[r, col] = float(row[f"cam_t_board_{r}{col}"])
            head_t_tcp.append(g)
            cam_t_board.append(c)
    if len(head_t_tcp) < 3:
        raise ValueError("至少需要 3 组姿态；建议 8-12 组且姿态/深度变化明显")
    return head_t_tcp, cam_t_board


def _project_to_rotation(raw: np.ndarray) -> np.ndarray:
    u, _, vt = np.linalg.svd(raw)
    r = u @ vt
    if np.linalg.det(r) < 0:
        u[:, -1] *= -1
        r = u @ vt
    return r


def _solve_ax_xb(head_t_tcp: list[np.ndarray], cam_t_board: list[np.ndarray]) -> np.ndarray:
    rows = []
    rel_pairs = 0
    for i in range(len(head_t_tcp)):
        for j in range(i + 1, len(head_t_tcp)):
            a = head_t_tcp[i] @ np.linalg.inv(head_t_tcp[j])
            b = cam_t_board[i] @ np.linalg.inv(cam_t_board[j])
            if _rotation_angle(np.eye(4), a) < math.radians(3.0):
                continue
            qa = _matrix_to_quat(a)
            qb = _matrix_to_quat(b)
            if qa[3] < 0:
                qa *= -1.0
            if qb[3] < 0:
                qb *= -1.0
            rows.append(_quat_left_matrix(qa) - _quat_right_matrix(qb))
            rel_pairs += 1
    if rel_pairs < 3:
        raise ValueError("有效相对姿态太少；采样时需要明显改变手腕姿态，不要只平移")
    matrix = np.vstack(rows)
    _, _, vt = np.linalg.svd(matrix)
    qx = vt[-1]
    qx /= np.linalg.norm(qx)
    rx = _quat_to_matrix(qx)

    trans_rows = []
    trans_rhs = []
    for i in range(len(head_t_tcp)):
        for j in range(i + 1, len(head_t_tcp)):
            a = head_t_tcp[i] @ np.linalg.inv(head_t_tcp[j])
            b = cam_t_board[i] @ np.linalg.inv(cam_t_board[j])
            if _rotation_angle(np.eye(4), a) < math.radians(3.0):
                continue
            ra = a[:3, :3]
            ta = a[:3, 3]
            tb = b[:3, 3]
            trans_rows.append(ra - np.eye(3))
            trans_rhs.append(rx @ tb - ta)
    tx, *_ = np.linalg.lstsq(np.vstack(trans_rows), np.concatenate(trans_rhs), rcond=None)

    result = np.eye(4)
    result[:3, :3] = rx
    result[:3, 3] = tx
    return result


def _estimate_tcp_t_board(head_t_cam: np.ndarray, head_t_tcp: list[np.ndarray], cam_t_board: list[np.ndarray]):
    estimates = []
    for g, c in zip(head_t_tcp, cam_t_board):
        estimates.append(np.linalg.inv(g) @ head_t_cam @ c)
    return _average_transform(estimates), estimates


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument("--output", default=None)
    args = parser.parse_args()

    head_t_tcp, cam_t_board = _load_samples(args.csv_path)
    head_t_cam = _solve_ax_xb(head_t_tcp, cam_t_board)
    tcp_t_board, tcp_t_board_samples = _estimate_tcp_t_board(head_t_cam, head_t_tcp, cam_t_board)

    head_t_cam_errors = []
    tcp_translation_errors = []
    tcp_rotation_errors = []
    for g, c, y in zip(head_t_tcp, cam_t_board, tcp_t_board_samples):
        lhs = head_t_cam @ c
        rhs = g @ tcp_t_board
        head_t_cam_errors.append(float(np.linalg.norm(lhs[:3, 3] - rhs[:3, 3])))
        tcp_translation_errors.append(float(np.linalg.norm(y[:3, 3] - tcp_t_board[:3, 3])))
        tcp_rotation_errors.append(float(_rotation_angle(tcp_t_board, y)))

    result = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "input_csv": args.csv_path,
        "description": "CAM2HEAD / HEAD_T_CAM solved from self-defined vendor hand-back marker board; TCP_T_BOARD estimated, not assumed",
        "num_samples": len(head_t_tcp),
        "cam2head_transform_4x4": head_t_cam.tolist(),
        "transform_4x4": head_t_cam.tolist(),
        "estimated_tcp_to_board_4x4": tcp_t_board.tolist(),
        "mean_position_residual_m": float(np.mean(head_t_cam_errors)),
        "max_position_residual_m": float(np.max(head_t_cam_errors)),
        "mean_tcp_to_board_translation_spread_m": float(np.mean(tcp_translation_errors)),
        "max_tcp_to_board_translation_spread_m": float(np.max(tcp_translation_errors)),
        "mean_tcp_to_board_rotation_spread_deg": float(np.degrees(np.mean(tcp_rotation_errors))),
        "max_tcp_to_board_rotation_spread_deg": float(np.degrees(np.max(tcp_rotation_errors))),
        "per_sample_position_residual_m": head_t_cam_errors,
        "per_sample_tcp_to_board_translation_spread_m": tcp_translation_errors,
        "per_sample_tcp_to_board_rotation_spread_deg": [float(np.degrees(v)) for v in tcp_rotation_errors],
    }

    print("=" * 70)
    print("  Vendor board CAM2HEAD result")
    print("=" * 70)
    print(f"samples: {result['num_samples']}")
    print(f"position residual mean/max: {result['mean_position_residual_m'] * 1000:.1f} / {result['max_position_residual_m'] * 1000:.1f} mm")
    print(f"TCP_T_BOARD spread mean/max: {result['mean_tcp_to_board_translation_spread_m'] * 1000:.1f} / {result['max_tcp_to_board_translation_spread_m'] * 1000:.1f} mm")
    print(f"TCP_T_BOARD rot spread mean/max: {result['mean_tcp_to_board_rotation_spread_deg']:.3f} / {result['max_tcp_to_board_rotation_spread_deg']:.3f} deg")
    print("CAM2HEAD transform_4x4:")
    print(json.dumps(result["transform_4x4"], indent=2))
    print("estimated TCP_T_BOARD:")
    print(json.dumps(result["estimated_tcp_to_board_4x4"], indent=2))

    out_path = args.output
    if out_path is None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(SCRIPT_DIR, "calibration", f"cam2head_vendor_board_{ts}.json")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print(f"[✓] 已保存: {out_path}")

    if result["max_position_residual_m"] > 0.03:
        print("[!] 残差超过 30mm，优先检查 marker 尺寸/ID 布局，以及采样时是否有明显姿态变化")


if __name__ == "__main__":
    main()
