#!/usr/bin/env python3
"""
Collect point pairs for the official CAM2HEAD calibration.

Method:
  1. Put a small visible marker at the right-arm TCP reference point.
  2. Move the marker to several positions visible by the head RealSense.
  3. Click the marker center in the RGB image.
  4. The script records:
     - clicked camera point from aligned depth, in camera optical frame
     - right TCP point in BASE frame from SDK tcp_pose
     - same TCP point converted to HEAD frame using /tf

This is read-only. It does not send motion commands.
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
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot_grasp.common import config
from robot_grasp.vision.depth_utils import get_depth_roi_stats, pixel_to_3d
from robot_grasp.common.ros_client import ROSClient

try:
    import roslibpy
except ImportError:
    print("缺少 roslibpy 库，请在 footpose 环境运行，或先安装: pip install roslibpy")
    sys.exit(1)


TCP_TOPICS = {
    "right": "/zj_humanoid/upperlimb/tcp_pose/right_arm",
    "left": "/zj_humanoid/upperlimb/tcp_pose/left_arm",
}

_lock = threading.Lock()
_latest_depth = None
_latest_cam_info = None
_latest_tcp_pose = None
_tf_transforms = {}
_pairs = []
_output_path = None
_click_radius = 3


def _parse_ws_url(ws_url: str) -> tuple[str, int]:
    stripped = ws_url.replace("ws://", "").replace("wss://", "")
    host, port = stripped.split(":")
    return host, int(port)


def _connect_roslibpy():
    host, port = _parse_ws_url(config.WS_URL)
    client = roslibpy.Ros(host=host, port=port)
    thread = threading.Thread(target=client.run, daemon=True)
    thread.start()
    start = time.time()
    while not client.is_connected:
        if time.time() - start > config.CONNECT_TIMEOUT:
            print(f"[✗] rosbridge 连接超时: {config.WS_URL}")
            sys.exit(1)
        time.sleep(0.1)
    return client


def _quat_to_matrix(q: dict) -> np.ndarray:
    x = float(q.get("x", 0.0))
    y = float(q.get("y", 0.0))
    z = float(q.get("z", 0.0))
    w = float(q.get("w", 1.0))
    norm = np.linalg.norm([x, y, z, w])
    if norm == 0:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _transform_to_matrix(item: dict) -> np.ndarray:
    tf = item.get("transform", {})
    trans = tf.get("translation", {})
    rot = tf.get("rotation", {})
    matrix = np.eye(4)
    matrix[:3, :3] = _quat_to_matrix(rot)
    matrix[:3, 3] = [
        float(trans.get("x", 0.0)),
        float(trans.get("y", 0.0)),
        float(trans.get("z", 0.0)),
    ]
    return matrix


def _store_tf_message(message: dict):
    with _lock:
        for transform in message.get("transforms", []):
            parent = transform.get("header", {}).get("frame_id", "").lstrip("/")
            child = transform.get("child_frame_id", "").lstrip("/")
            if not parent or not child:
                continue
            _tf_transforms[(parent, child)] = transform


def _start_tf_and_tcp_subscribers(client, arm: str):
    tcp_topic = TCP_TOPICS[arm]

    def tcp_callback(message):
        global _latest_tcp_pose
        with _lock:
            _latest_tcp_pose = message

    tcp_sub = roslibpy.Topic(client, tcp_topic, "upperlimb/Pose")
    tf_sub = roslibpy.Topic(client, "/tf", "tf2_msgs/TFMessage")
    tf_static_sub = roslibpy.Topic(client, "/tf_static", "tf2_msgs/TFMessage")
    tcp_sub.subscribe(tcp_callback)
    tf_sub.subscribe(_store_tf_message)
    tf_static_sub.subscribe(_store_tf_message)
    return [tcp_sub, tf_sub, tf_static_sub], tcp_topic


def _build_graph(transforms: dict):
    graph = {}
    for (parent, child), transform in transforms.items():
        graph.setdefault(parent, []).append((child, "forward", transform))
        graph.setdefault(child, []).append((parent, "inverse", transform))
    return graph


def _find_path(graph, start: str, goal: str):
    start = start.lstrip("/")
    goal = goal.lstrip("/")
    queue = [(start, [])]
    visited = {start}
    while queue:
        node, path = queue.pop(0)
        if node == goal:
            return path
        for nxt, direction, transform in graph.get(node, []):
            if nxt in visited:
                continue
            visited.add(nxt)
            queue.append((nxt, path + [(direction, transform)]))
    return None


def _lookup_transform(start: str, goal: str):
    """Return T_start_goal, mapping homogeneous points from goal frame to start frame."""
    with _lock:
        transforms = dict(_tf_transforms)
    graph = _build_graph(transforms)
    path = _find_path(graph, start, goal)
    if path is None:
        return None
    result = np.eye(4)
    for direction, transform in path:
        step = _transform_to_matrix(transform)
        if direction == "inverse":
            step = np.linalg.inv(step)
        result = result @ step
    return result


def _parse_tcp_position(message: dict):
    pos = message.get("position", {})
    quat = message.get("quaternion", {})
    return {
        "base_x_m": float(pos.get("x", 0.0)),
        "base_y_m": float(pos.get("y", 0.0)),
        "base_z_m": float(pos.get("z", 0.0)),
        "tcp_qx": float(quat.get("x", 0.0)),
        "tcp_qy": float(quat.get("y", 0.0)),
        "tcp_qz": float(quat.get("z", 0.0)),
        "tcp_qw": float(quat.get("w", 1.0)),
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
    print(f"[✓] 已保存 {len(_pairs)} 组 CAM2HEAD 点对: {path}")


def _mouse_callback(event, x, y, flags, param):
    if event != cv2.EVENT_LBUTTONDOWN:
        return

    with _lock:
        depth = None if _latest_depth is None else _latest_depth.copy()
        cam_info = dict(_latest_cam_info) if _latest_cam_info is not None else None
        tcp_msg = dict(_latest_tcp_pose) if _latest_tcp_pose is not None else None

    if depth is None or cam_info is None or "K" not in cam_info:
        print(f"[!] 点击 ({x},{y}) 无法记录: 没有 depth/camera_info")
        return
    if tcp_msg is None:
        print(f"[!] 点击 ({x},{y}) 无法记录: 没有 TCP pose")
        return

    t_base_head = _lookup_transform("BASE", "HEAD")
    if t_base_head is None:
        print("[!] 无法记录: TF 中没有 BASE -> HEAD 路径")
        return

    roi = get_depth_roi_stats(
        depth,
        x - _click_radius,
        y - _click_radius,
        x + _click_radius + 1,
        y + _click_radius + 1,
    )
    depth_mm = roi["median_mm"]
    if depth_mm is None:
        print(f"[!] 点击 ({x},{y}) 无法记录: 深度无效")
        return

    K = cam_info["K"]
    cam_x_mm, cam_y_mm, cam_z_mm = pixel_to_3d(x, y, depth_mm, K[0], K[4], K[2], K[5])
    tcp = _parse_tcp_position(tcp_msg)

    point_base = np.array([tcp["base_x_m"], tcp["base_y_m"], tcp["base_z_m"], 1.0])
    point_head = np.linalg.inv(t_base_head) @ point_base

    record = {
        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
        "pixel_u": int(x),
        "pixel_v": int(y),
        "depth_mm": round(float(depth_mm), 2),
        "depth_roi_valid_count": roi["valid_count"],
        "depth_roi_total_count": roi["total_count"],
        "cam_x_m": round(cam_x_mm / 1000.0, 6),
        "cam_y_m": round(cam_y_mm / 1000.0, 6),
        "cam_z_m": round(cam_z_mm / 1000.0, 6),
        **tcp,
        "head_x_m": round(float(point_head[0]), 6),
        "head_y_m": round(float(point_head[1]), 6),
        "head_z_m": round(float(point_head[2]), 6),
    }
    _pairs.append(record)
    print(
        f"[PAIR {len(_pairs)}] pixel=({x},{y}) "
        f"cam=({record['cam_x_m']:.4f},{record['cam_y_m']:.4f},{record['cam_z_m']:.4f}) "
        f"head=({record['head_x_m']:.4f},{record['head_y_m']:.4f},{record['head_z_m']:.4f})"
    )
    _save_pairs(_output_path)


def main():
    global _latest_depth, _latest_cam_info, _output_path, _click_radius

    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", choices=["right", "left"], default="right")
    parser.add_argument("--output", default=None)
    parser.add_argument("--window", default="CAM2HEAD Pair Collector")
    parser.add_argument("--click-radius", type=int, default=3)
    args = parser.parse_args()
    _click_radius = max(0, int(args.click_radius))

    if args.output:
        _output_path = args.output
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        _output_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "data",
            f"cam2head_pairs_{ts}.csv",
        )

    print("=" * 70)
    print("  CAM2HEAD 点对采集（只读，不发运动命令）")
    print("=" * 70)
    print(f"  WebSocket: {config.WS_URL}")
    print(f"  TCP topic: {TCP_TOPICS[args.arm]}")
    print(f"  Output: {_output_path}")
    print("  操作: 鼠标左键记录 TCP 标记中心 | s 保存 | q 退出")
    print("  注意: 必须点击 TCP 尖端或固定在 TCP 参考点的小标记中心")
    print("=" * 70)

    aux_client = _connect_roslibpy()
    subscribers, tcp_topic = _start_tf_and_tcp_subscribers(aux_client, args.arm)
    print(f"[✓] 已订阅 {tcp_topic}, /tf, /tf_static")

    ros_client = ROSClient()
    if not ros_client.connect():
        raise SystemExit(1)

    cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(args.window, _mouse_callback)

    last_frame_count = -1
    try:
        while True:
            rgb, depth, cam_info, frame_count = ros_client.get_frames()
            if rgb is None or frame_count == last_frame_count:
                key = cv2.waitKey(5) & 0xFF
                if key == ord("q"):
                    break
                continue
            last_frame_count = frame_count
            with _lock:
                _latest_depth = depth
                _latest_cam_info = cam_info

            shown = rgb.copy()
            cv2.putText(
                shown,
                f"CAM2HEAD pairs={len(_pairs)} | click TCP marker | s save | q quit",
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
        for sub in subscribers:
            try:
                sub.unsubscribe()
            except Exception:
                pass
        try:
            aux_client.terminate()
        except Exception:
            pass
        ros_client.disconnect()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
