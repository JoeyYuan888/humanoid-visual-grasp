#!/usr/bin/env python3
"""Capture one low-head RGB/depth snapshot for box-geometry inspection.

This script is read-only for arms. It only uses MPC neck to look down/home,
then saves RGB/depth diagnostics so we can decide whether depth-only box
contour detection is viable.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from apps.transport.run_box_grasp_point import (  # noqa: E402
    _connect_control,
    _move_neck,
    _safe_terminate,
    _set_mpc_mode,
    _suppress_roslibpy_shutdown_noise,
)
from robot_grasp.common.ros_client import ROSClient  # noqa: E402


DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "data" / "transport" / "depth_snapshot_latest"
_suppress_roslibpy_shutdown_noise()


def _wait_for_frames(client: ROSClient, timeout: float):
    deadline = time.time() + timeout
    while time.time() < deadline:
        rgb, depth, camera_info, _count = client.get_frames()
        if rgb is not None and depth is not None and camera_info is not None:
            return rgb, depth, camera_info
        time.sleep(0.05)
    stats = client.get_stats()
    raise RuntimeError(
        "timeout waiting for RGB/depth/camera_info: "
        f"rgb={stats['rgb_count']} depth_msg={stats['depth_msg_count']} "
        f"depth={stats['depth_count']} camera_info={'yes' if client.camera_info else 'no'}"
    )


def _depth_to_vis(depth: np.ndarray) -> np.ndarray:
    valid = depth[(depth > 0) & np.isfinite(depth)]
    if valid.size == 0:
        return np.zeros((*depth.shape[:2], 3), dtype=np.uint8)
    lo = float(np.percentile(valid, 2))
    hi = float(np.percentile(valid, 98))
    if hi <= lo:
        hi = lo + 1.0
    norm = ((np.clip(depth.astype(np.float32), lo, hi) - lo) / (hi - lo) * 255.0).astype(np.uint8)
    norm[depth <= 0] = 0
    return cv2.applyColorMap(norm, cv2.COLORMAP_JET)


def _depth_edges(depth: np.ndarray) -> np.ndarray:
    valid = depth > 0
    if int(valid.sum()) < 100:
        return np.zeros(depth.shape[:2], dtype=np.uint8)
    depth_f = depth.astype(np.float32)
    median = float(np.median(depth_f[valid]))
    depth_f[~valid] = median
    smooth = cv2.medianBlur(depth_f, 5)
    valid_values = smooth[valid]
    lo = float(np.percentile(valid_values, 2))
    hi = float(np.percentile(valid_values, 98))
    if hi <= lo:
        hi = lo + 1.0
    norm = ((np.clip(smooth, lo, hi) - lo) / (hi - lo) * 255.0).astype(np.uint8)
    edges = cv2.Canny(norm, 35, 100)
    edges[~valid] = 0
    return edges


def _near_mask(depth: np.ndarray, near_percentile: float) -> np.ndarray:
    valid = depth[(depth > 0) & np.isfinite(depth)]
    if valid.size == 0:
        return np.zeros(depth.shape[:2], dtype=np.uint8)
    cutoff = float(np.percentile(valid, near_percentile))
    mask = ((depth > 0) & (depth <= cutoff)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    return mask


def _draw_overlay(rgb: np.ndarray, edges: np.ndarray, near: np.ndarray) -> np.ndarray:
    overlay = rgb.copy()
    near_pixels = near > 0
    if np.any(near_pixels):
        color = np.zeros_like(overlay)
        color[:, :] = (0, 255, 255)
        blended = cv2.addWeighted(overlay, 0.45, color, 0.55, 0)
        overlay[near_pixels] = blended[near_pixels]
    overlay[edges > 0] = (0, 0, 255)
    return overlay


def _save_outputs(output_dir: Path, rgb: np.ndarray, depth: np.ndarray, camera_info: dict, near_percentile: float) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    depth_vis = _depth_to_vis(depth)
    edges = _depth_edges(depth)
    near = _near_mask(depth, near_percentile)
    overlay = _draw_overlay(rgb, edges, near)

    cv2.imwrite(str(output_dir / "rgb.png"), rgb)
    cv2.imwrite(str(output_dir / "depth_vis.png"), depth_vis)
    cv2.imwrite(str(output_dir / "depth_edges.png"), edges)
    cv2.imwrite(str(output_dir / "near_mask.png"), near)
    cv2.imwrite(str(output_dir / "overlay.png"), overlay)
    np.save(str(output_dir / "depth_raw.npy"), depth)
    (output_dir / "camera_info.json").write_text(
        json.dumps(camera_info, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    meta = {
        "outputs": {
            "rgb": str(output_dir / "rgb.png"),
            "depth_vis": str(output_dir / "depth_vis.png"),
            "depth_edges": str(output_dir / "depth_edges.png"),
            "near_mask": str(output_dir / "near_mask.png"),
            "overlay": str(output_dir / "overlay.png"),
            "depth_raw": str(output_dir / "depth_raw.npy"),
            "camera_info": str(output_dir / "camera_info.json"),
        },
        "near_percentile": near_percentile,
        "depth_shape": list(depth.shape[:2]),
        "rgb_shape": list(rgb.shape[:2]),
    }
    (output_dir / "meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--frame-timeout", type=float, default=8.0)
    parser.add_argument("--near-percentile", type=float, default=35.0)
    parser.add_argument("--show-window", action="store_true")
    parser.add_argument("--skip-neck-down", action="store_true")
    parser.add_argument("--skip-neck-home", action="store_true")
    parser.add_argument("--neck-down-z", type=float, default=0.0)
    parser.add_argument("--neck-down-y", type=float, default=0.35)
    parser.add_argument("--neck-home-z", type=float, default=0.0)
    parser.add_argument("--neck-home-y", type=float, default=0.0)
    parser.add_argument("--neck-time", type=float, default=4.0)
    parser.add_argument("--neck-verify-tolerance", type=float, default=0.10)
    parser.add_argument("--no-neck-verify", action="store_true")
    args = parser.parse_args()

    print("=" * 70)
    print("  Depth snapshot for box contour inspection")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  Output dir: {args.output_dir}")
    print(f"  Neck down: [{args.neck_down_z}, {args.neck_down_y}]")
    print(f"  Neck home: [{args.neck_home_z}, {args.neck_home_y}]")

    control_client = None
    vision_client = None
    verify_neck = not args.no_neck_verify
    output_dir = Path(args.output_dir)

    try:
        if not args.skip_neck_down or not args.skip_neck_home:
            control_client = _connect_control(args.ws_url)
            _set_mpc_mode(control_client, True)

        if not args.skip_neck_down:
            _move_neck(
                control_client,
                args.neck_down_z,
                args.neck_down_y,
                args.neck_time,
                verify=verify_neck,
                tolerance=args.neck_verify_tolerance,
            )

        vision_client = ROSClient(args.ws_url)
        if not vision_client.connect():
            raise RuntimeError(f"failed to connect rosbridge: {args.ws_url}")
        rgb, depth, camera_info = _wait_for_frames(vision_client, args.frame_timeout)
        _save_outputs(output_dir, rgb, depth, camera_info, args.near_percentile)
        print(f"[✓] depth snapshot saved: {output_dir}")
        print("    rgb.png, depth_vis.png, depth_edges.png, near_mask.png, overlay.png")

        if args.show_window:
            overlay = cv2.imread(str(output_dir / "overlay.png"))
            cv2.imshow("depth snapshot overlay | q=exit", overlay)
            while True:
                if cv2.waitKey(30) & 0xFF == ord("q"):
                    break
            cv2.destroyAllWindows()
    finally:
        if control_client is not None and not args.skip_neck_home:
            try:
                _move_neck(
                    control_client,
                    args.neck_home_z,
                    args.neck_home_y,
                    args.neck_time,
                    verify=verify_neck,
                    tolerance=args.neck_verify_tolerance,
                )
            except Exception as exc:
                print(f"[!] neck home failed: {exc}")
        if vision_client is not None:
            vision_client.disconnect()
        _safe_terminate(control_client)


if __name__ == "__main__":
    main()
