#!/usr/bin/env python3
"""
Deprecated experimental helper.

Collect camera-to-MPC hand-eye point pairs.

This tool is for the fixed head RealSense look-down pose. It does not move the
robot. On each mouse click it records:
- clicked pixel and depth-derived camera point
- current MPC end-effector pose from /DualArmMobile/currentEEPose/FrameR or L

This is not the official course-manual hand-eye route. The production pipeline
must use CAM2HEAD -> HEAD2BASE(tf) -> BASE -> GRASP_OFFSET.

Recommended setup:
1. Keep the neck at the same look-down pose used for vision.
2. Put a small visible marker near the TCP/grasp reference point.
3. Move the arm to several visible positions.
4. Click the marker center in the image each time.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import threading
import time
from datetime import datetime

import cv2

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot_grasp.common import config
from robot_grasp.vision.depth_utils import get_depth_roi_stats, pixel_to_3d
from robot_grasp.common.ros_client import ROSClient

try:
    import roslibpy
except ImportError:
    print("缺少 roslibpy 库，请运行: pip install roslibpy")
    sys.exit(1)


FRAME_TOPICS = {
    "left": "/DualArmMobile/currentEEPose/FrameL",
    "right": "/DualArmMobile/currentEEPose/FrameR",
}

_latest_depth = None
_latest_cam_info = None
_latest_pose = None
_latest_frame = None
_pairs = []
_output_path = None
_click_radius = 3
_lock = threading.Lock()


def _parse_ws_url(ws_url: str) -> tuple[str, int]:
    stripped = ws_url.replace("ws://", "").replace("wss://", "")
    host, port = stripped.split(":")
    return host, int(port)


def _start_pose_subscriber(arm: str):
    host, port = _parse_ws_url(config.WS_URL)
    client = roslibpy.Ros(host=host, port=port)
    thread = threading.Thread(target=client.run, daemon=True)
    thread.start()

    start = time.time()
    while not client.is_connected:
        if time.time() - start > config.CONNECT_TIMEOUT:
            print(f"[✗] MPC pose 连接超时: {config.WS_URL}")
            sys.exit(1)
        time.sleep(0.1)

    topic = FRAME_TOPICS[arm]

    def callback(message):
        global _latest_pose
        with _lock:
            _latest_pose = message

    sub = roslibpy.Topic(client, topic, "geometry_msgs/PoseStamped")
    sub.subscribe(callback)
    return client, sub, topic


def _pose_fields(message: dict) -> dict:
    pose = message.get("pose", {})
    pos = pose.get("position", {})
    ori = pose.get("orientation", {})
    return {
        "mpc_x_m": float(pos.get("x", 0.0)),
        "mpc_y_m": float(pos.get("y", 0.0)),
        "mpc_z_m": float(pos.get("z", 0.0)),
        "mpc_qx": float(ori.get("x", 0.0)),
        "mpc_qy": float(ori.get("y", 0.0)),
        "mpc_qz": float(ori.get("z", 0.0)),
        "mpc_qw": float(ori.get("w", 1.0)),
    }


def _save_pairs(path: str):
    if not _pairs:
        return
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    fieldnames = list(_pairs[0].keys())
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(_pairs)
    print(f"[✓] 已保存 {len(_pairs)} 组点对: {path}")


def _mouse_callback(event, x, y, flags, param):
    if event != cv2.EVENT_LBUTTONDOWN:
        return

    global _latest_frame
    with _lock:
        depth = None if _latest_depth is None else _latest_depth.copy()
        cam_info = dict(_latest_cam_info) if _latest_cam_info is not None else None
        pose_msg = dict(_latest_pose) if _latest_pose is not None else None

    if depth is None or cam_info is None or "K" not in cam_info:
        print(f"[!] 点击 ({x},{y}) 无法记录: 没有 depth/camera_info")
        return
    if pose_msg is None:
        print(f"[!] 点击 ({x},{y}) 无法记录: 没有 MPC EE pose")
        return

    roi = get_depth_roi_stats(
        depth,
        x - _click_radius,
        y - _click_radius,
        x + _click_radius + 1,
        y + _click_radius + 1,
    )
    d = roi["median_mm"]
    if d is None:
        print(f"[!] 点击 ({x},{y}) 无法记录: 深度无效")
        return

    K = cam_info["K"]
    cam_x_mm, cam_y_mm, cam_z_mm = pixel_to_3d(x, y, d, K[0], K[4], K[2], K[5])
    pose = _pose_fields(pose_msg)
    record = {
        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
        "pixel_u": int(x),
        "pixel_v": int(y),
        "depth_mm": round(float(d), 2),
        "depth_roi_valid_count": roi["valid_count"],
        "depth_roi_total_count": roi["total_count"],
        "cam_x_m": round(cam_x_mm / 1000.0, 6),
        "cam_y_m": round(cam_y_mm / 1000.0, 6),
        "cam_z_m": round(cam_z_mm / 1000.0, 6),
        **pose,
    }
    _pairs.append(record)
    print(
        f"[PAIR {len(_pairs)}] pixel=({x},{y}) "
        f"cam=({record['cam_x_m']:.4f},{record['cam_y_m']:.4f},{record['cam_z_m']:.4f}) "
        f"mpc=({record['mpc_x_m']:.4f},{record['mpc_y_m']:.4f},{record['mpc_z_m']:.4f})"
    )
    _save_pairs(_output_path)


def main():
    global _latest_depth, _latest_cam_info, _latest_frame, _output_path, _click_radius

    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", choices=["right", "left"], default="right")
    parser.add_argument("--output", default=None)
    parser.add_argument("--window", default="HandEye Pair Collector")
    parser.add_argument("--click-radius", type=int, default=3, help="点击点周围深度中位数 ROI 半径，单位像素")
    parser.add_argument("--allow-deprecated", action="store_true")
    args = parser.parse_args()
    if not args.allow_deprecated:
        raise SystemExit(
            "该脚本已废弃：不要用固定脖子 camera->MPC 点对拟合作为正式手眼路线。\n"
            "正式路线请使用 CAM2HEAD -> HEAD2BASE(tf) -> BASE -> GRASP_OFFSET。\n"
            "如仅需查看历史实验工具，可显式添加 --allow-deprecated。"
        )
    _click_radius = max(0, int(args.click_radius))

    if args.output:
        _output_path = args.output
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        _output_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "data",
            f"deprecated_handeye_pairs_{ts}.csv",
        )

    print("=" * 70)
    print("  Hand-eye pair collection")
    print("=" * 70)
    print(f"  WebSocket: {config.WS_URL}")
    print(f"  MPC pose topic: {FRAME_TOPICS[args.arm]}")
    print(f"  Output: {_output_path}")
    print(f"  Depth ROI radius: {_click_radius}px")
    print("  鼠标左键: 记录点对 | s: 保存 | q: 退出")
    print("=" * 70)

    pose_client, pose_sub, pose_topic = _start_pose_subscriber(args.arm)
    print(f"[✓] 已订阅 {pose_topic}")

    ros_client = ROSClient()
    if not ros_client.connect():
        raise SystemExit(1)

    cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(args.window, _mouse_callback)

    last_frame_count = -1
    try:
        while True:
            rgb, depth, cam_info, fc = ros_client.get_frames()
            if rgb is None or fc == last_frame_count:
                key = cv2.waitKey(5) & 0xFF
                if key == ord("q"):
                    break
                continue
            last_frame_count = fc

            with _lock:
                _latest_depth = depth
                _latest_cam_info = cam_info
                _latest_frame = rgb

            shown = rgb.copy()
            cv2.putText(
                shown,
                f"pairs={len(_pairs)} click marker center | s save | q quit",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )
            cv2.imshow(args.window, shown)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                _save_pairs(_output_path)
    except KeyboardInterrupt:
        print("\n[*] 中断退出")
    finally:
        _save_pairs(_output_path)
        try:
            pose_sub.unsubscribe()
        except Exception:
            pass
        try:
            pose_client.terminate()
        except Exception:
            pass
        ros_client.disconnect()
        cv2.destroyAllWindows()
        print("[✓] 采集结束")


if __name__ == "__main__":
    main()
