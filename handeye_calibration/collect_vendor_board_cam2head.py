#!/usr/bin/env python3
"""
Collect CAM2HEAD samples using the vendor hand-back ArUco board.

This script is read-only. It does not send robot motion commands.

For each captured sample:
  CAM_T_BOARD comes from marker detection and camera intrinsics.
  BASE_T_TCP comes from /zj_humanoid/upperlimb/tcp_pose/<arm>_arm.
  BASE_T_HEAD comes from /tf.

The saved raw poses are enough to solve:
  HEAD_T_CAM * CAM_T_BOARD = HEAD_T_TCP * TCP_T_BOARD

The solver estimates both HEAD_T_CAM and TCP_T_BOARD, so this collection path
does not require trusting the vendor sample image T/R.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import threading
import time
from collections import deque
from datetime import datetime

import cv2
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot_grasp import config
from robot_grasp.depth_utils import get_depth_roi_stats, pixel_to_3d
from robot_grasp.ros_client import ROSClient

try:
    import roslibpy
except ImportError:
    print("缺少 roslibpy，请在 detect 环境安装/运行")
    sys.exit(1)


TCP_TOPICS = {
    "right": "/zj_humanoid/upperlimb/tcp_pose/right_arm",
    "left": "/zj_humanoid/upperlimb/tcp_pose/left_arm",
}

DEFAULT_BOARD_CONFIG = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "calibration",
    "vendor_handback_board_20260729.json",
)

_lock = threading.Lock()
_latest_tcp_pose = None
_tf_transforms = {}
_samples = []
_capture_requested = False


def _parse_ws_url(ws_url: str) -> tuple[str, int]:
    stripped = ws_url.replace("ws://", "").replace("wss://", "")
    host, port = stripped.split(":")
    return host, int(port)


def _connect_roslibpy(ws_url: str):
    host, port = _parse_ws_url(ws_url)
    client = roslibpy.Ros(host=host, port=port)
    thread = threading.Thread(target=client.run, daemon=True)
    thread.start()
    start = time.time()
    while not client.is_connected:
        if time.time() - start > config.CONNECT_TIMEOUT:
            raise RuntimeError(f"rosbridge 连接超时: {ws_url}")
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


def _matrix_to_quat(matrix: np.ndarray) -> tuple[float, float, float, float]:
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
    quat /= np.linalg.norm(quat)
    return tuple(float(v) for v in quat)


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


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


def _pose_msg_to_matrix(message: dict) -> np.ndarray:
    pos = message.get("position", {})
    quat = message.get("quaternion", {})
    matrix = np.eye(4)
    matrix[:3, :3] = _quat_to_matrix(quat)
    matrix[:3, 3] = [
        float(pos.get("x", 0.0)),
        float(pos.get("y", 0.0)),
        float(pos.get("z", 0.0)),
    ]
    return matrix


def _store_tf_message(message: dict):
    with _lock:
        for transform in message.get("transforms", []):
            parent = transform.get("header", {}).get("frame_id", "").lstrip("/")
            child = transform.get("child_frame_id", "").lstrip("/")
            if parent and child:
                _tf_transforms[(parent, child)] = transform


def _build_graph(transforms: dict):
    graph = {}
    for (parent, child), transform in transforms.items():
        graph.setdefault(parent, []).append((child, "forward", transform))
        graph.setdefault(child, []).append((parent, "inverse", transform))
    return graph


def _find_path(graph, start: str, goal: str):
    queue = [(start.lstrip("/"), [])]
    visited = {start.lstrip("/")}
    goal = goal.lstrip("/")
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
    path = _find_path(_build_graph(transforms), start, goal)
    if path is None:
        return None
    result = np.eye(4)
    for direction, transform in path:
        step = _transform_to_matrix(transform)
        if direction == "inverse":
            step = np.linalg.inv(step)
        result = result @ step
    return result


def _start_aux_subscribers(client, arm: str):
    def tcp_callback(message):
        global _latest_tcp_pose
        with _lock:
            _latest_tcp_pose = message

    tcp_sub = roslibpy.Topic(client, TCP_TOPICS[arm], "upperlimb/Pose")
    tf_sub = roslibpy.Topic(client, "/tf", "tf2_msgs/TFMessage")
    tf_static_sub = roslibpy.Topic(client, "/tf_static", "tf2_msgs/TFMessage")
    tcp_sub.subscribe(tcp_callback)
    tf_sub.subscribe(_store_tf_message)
    tf_static_sub.subscribe(_store_tf_message)
    return [tcp_sub, tf_sub, tf_static_sub]


def _load_board_config(path: str):
    with open(path, encoding="utf-8") as f:
        cfg = json.load(f)
    dictionary_name = cfg.get("dictionary", "DICT_4X4_50")
    if not hasattr(cv2.aruco, dictionary_name):
        raise ValueError(f"OpenCV aruco 不支持 dictionary: {dictionary_name}")
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    marker_size = float(cfg["marker_size_m"])
    markers = {int(k): v for k, v in cfg.get("markers", {}).items()}
    if not markers:
        raise ValueError("board config 没有 markers")
    return cfg, dictionary, marker_size, markers


def _tcp_to_board_from_config(cfg: dict) -> np.ndarray | None:
    tcp = cfg.get("tcp_to_board")
    if not tcp:
        return None
    tcp_t_board = np.eye(4)
    tcp_t_board[:3, 3] = np.asarray(tcp.get("translation_m", [0.0, 0.0, 0.0]), dtype=float)
    rpy = [math.radians(float(v)) for v in tcp.get("rpy_deg", [0.0, 0.0, 0.0])]
    tcp_t_board[:3, :3] = _rpy_to_matrix(rpy[0], rpy[1], rpy[2])
    return tcp_t_board


def _marker_object_corners(marker_cfg: dict, marker_size: float) -> np.ndarray:
    cx, cy, cz = [float(v) for v in marker_cfg["center_m"]]
    yaw = math.radians(float(marker_cfg.get("yaw_deg", 0.0)))
    half = marker_size / 2.0
    local = np.array(
        [
            [-half, -half, 0.0],
            [half, -half, 0.0],
            [half, half, 0.0],
            [-half, half, 0.0],
        ],
        dtype=np.float32,
    )
    rot = _rpy_to_matrix(0.0, 0.0, yaw)[:3, :3]
    return (local @ rot.T + np.array([cx, cy, cz], dtype=float)).astype(np.float32)


def _detect_markers(frame, dictionary):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    parameters.cornerRefinementWinSize = 5
    parameters.cornerRefinementMaxIterations = 50
    parameters.cornerRefinementMinAccuracy = 0.01
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 35
    parameters.perspectiveRemovePixelPerCell = 8
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    return corners, ids, rejected


def _pose_distance(a: np.ndarray, b: np.ndarray) -> float:
    trans = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    rot = float(np.linalg.norm(a[:3, :3] - b[:3, :3]))
    return trans + 0.08 * rot


def _estimate_cam_t_board(corners, ids, markers, marker_size, camera_info, previous_pose=None):
    if ids is None:
        return None, []
    object_points = []
    image_points = []
    used_ids = []
    for corner, marker_id in zip(corners, ids.flatten().tolist()):
        if marker_id not in markers:
            continue
        object_points.append(_marker_object_corners(markers[marker_id], marker_size))
        image_points.append(corner.reshape(4, 2).astype(np.float32))
        used_ids.append(int(marker_id))
    if len(used_ids) < 1:
        return None, used_ids
    obj = np.concatenate(object_points, axis=0)
    img = np.concatenate(image_points, axis=0)
    k = np.asarray(camera_info["K"], dtype=float).reshape(3, 3)
    d = np.asarray(camera_info.get("D", []), dtype=float)
    if d.size == 0:
        d = None
    candidates = []
    if hasattr(cv2, "solvePnPGeneric") and hasattr(cv2, "SOLVEPNP_IPPE"):
        ok, rvecs, tvecs, reproj = cv2.solvePnPGeneric(
            obj,
            img,
            k,
            d,
            flags=cv2.SOLVEPNP_IPPE,
        )
        if ok:
            reproj_values = np.asarray(reproj).reshape(-1).tolist() if reproj is not None else [0.0] * len(rvecs)
            for rvec, tvec, err in zip(rvecs, tvecs, reproj_values):
                if float(tvec.reshape(3)[2]) <= 0:
                    continue
                rot, _ = cv2.Rodrigues(rvec)
                matrix = np.eye(4)
                matrix[:3, :3] = rot
                matrix[:3, 3] = tvec.reshape(3)
                candidates.append((matrix, float(err)))
    if not candidates:
        ok, rvec, tvec = cv2.solvePnP(obj, img, k, d, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None, used_ids
        rot, _ = cv2.Rodrigues(rvec)
        matrix = np.eye(4)
        matrix[:3, :3] = rot
        matrix[:3, 3] = tvec.reshape(3)
        candidates.append((matrix, 0.0))
    if not candidates:
        return None, used_ids
    if previous_pose is not None:
        candidates.sort(key=lambda item: _pose_distance(item[0], previous_pose))
    else:
        candidates.sort(key=lambda item: item[1])
    return candidates[0][0], used_ids


def _solve_rigid_transform(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
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
    result = np.eye(4)
    result[:3, :3] = r
    result[:3, 3] = t
    return result


def _estimate_cam_t_board_depth_centers(corners, ids, markers, depth, camera_info, radius: int):
    if ids is None or depth is None:
        return None, []
    k = camera_info["K"]
    board_points = []
    cam_points = []
    used_ids = []
    for corner, marker_id in zip(corners, ids.flatten().tolist()):
        marker_id = int(marker_id)
        if marker_id not in markers:
            continue
        center = corner.reshape(4, 2).mean(axis=0)
        u = int(round(float(center[0])))
        v = int(round(float(center[1])))
        roi = get_depth_roi_stats(depth, u - radius, v - radius, u + radius + 1, v + radius + 1)
        depth_mm = roi["median_mm"]
        if depth_mm is None:
            continue
        cam_x_mm, cam_y_mm, cam_z_mm = pixel_to_3d(
            u,
            v,
            depth_mm,
            k[0],
            k[4],
            k[2],
            k[5],
        )
        board_points.append(np.asarray(markers[marker_id]["center_m"], dtype=float))
        cam_points.append(np.asarray([cam_x_mm, cam_y_mm, cam_z_mm], dtype=float) / 1000.0)
        used_ids.append(marker_id)
    if len(used_ids) < 3:
        return None, used_ids
    return _solve_rigid_transform(np.asarray(board_points), np.asarray(cam_points)), used_ids


def _draw_pose_axis(shown, transform, camera_info, axis_len: float, label: str):
    k = np.asarray(camera_info["K"], dtype=float).reshape(3, 3)
    d = np.asarray(camera_info.get("D", []), dtype=float)
    if d.size == 0:
        d = None
    points = np.array(
        [
            [0.0, 0.0, 0.0],
            [axis_len, 0.0, 0.0],
            [0.0, axis_len, 0.0],
            [0.0, 0.0, axis_len],
        ],
        dtype=np.float32,
    )
    cam_points = (transform[:3, :3] @ points.T).T + transform[:3, 3]
    rvec = np.zeros((3, 1), dtype=float)
    tvec = np.zeros((3, 1), dtype=float)
    image_points, _ = cv2.projectPoints(cam_points, rvec, tvec, k, d)
    pts = image_points.reshape(-1, 2).astype(int)
    origin = tuple(pts[0])
    cv2.line(shown, origin, tuple(pts[1]), (0, 0, 255), 3)
    cv2.line(shown, origin, tuple(pts[2]), (0, 255, 0), 3)
    cv2.line(shown, origin, tuple(pts[3]), (255, 0, 0), 3)
    cv2.putText(shown, f"{label} X", tuple(pts[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    cv2.putText(shown, f"{label} Y", tuple(pts[2]), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    cv2.putText(shown, f"{label} Z", tuple(pts[3]), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 0), 2)


def _save_samples(path: str):
    if not _samples:
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        fieldnames = list(_samples[0].keys())
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(_samples)
    print(f"[✓] 已保存 {len(_samples)} 组厂家板 CAM2HEAD 样本: {path}")


def _mouse_callback(event, x, y, flags, param):
    global _capture_requested
    if event == cv2.EVENT_LBUTTONDOWN:
        _capture_requested = True


def _record_sample(cam_t_board, used_ids, tcp_msg, output_path):
    base_t_head = _lookup_transform("BASE", "HEAD")
    if base_t_head is None:
        print("[!] 无法记录: TF 中没有 BASE -> HEAD")
        return
    if tcp_msg is None:
        print("[!] 无法记录: 没有 TCP pose")
        return

    head_t_tcp = np.linalg.inv(base_t_head) @ _pose_msg_to_matrix(tcp_msg)
    cam_qx, cam_qy, cam_qz, cam_qw = _matrix_to_quat(cam_t_board)
    tcp_qx, tcp_qy, tcp_qz, tcp_qw = _matrix_to_quat(head_t_tcp)
    row = {
        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
        "used_marker_ids": ";".join(str(v) for v in sorted(used_ids)),
        "num_markers": len(used_ids),
        "cam_t_board_tx": float(cam_t_board[0, 3]),
        "cam_t_board_ty": float(cam_t_board[1, 3]),
        "cam_t_board_tz": float(cam_t_board[2, 3]),
        "cam_t_board_qx": cam_qx,
        "cam_t_board_qy": cam_qy,
        "cam_t_board_qz": cam_qz,
        "cam_t_board_qw": cam_qw,
        "head_t_tcp_tx": float(head_t_tcp[0, 3]),
        "head_t_tcp_ty": float(head_t_tcp[1, 3]),
        "head_t_tcp_tz": float(head_t_tcp[2, 3]),
        "head_t_tcp_qx": tcp_qx,
        "head_t_tcp_qy": tcp_qy,
        "head_t_tcp_qz": tcp_qz,
        "head_t_tcp_qw": tcp_qw,
    }
    for r in range(4):
        for c in range(4):
            row[f"cam_t_board_{r}{c}"] = float(cam_t_board[r, c])
            row[f"head_t_tcp_{r}{c}"] = float(head_t_tcp[r, c])
    _samples.append(row)
    print(
        f"[SAMPLE {len(_samples)}] ids={row['used_marker_ids']} "
        f"CAM_T_BOARD t=({row['cam_t_board_tx']:.4f},"
        f"{row['cam_t_board_ty']:.4f},{row['cam_t_board_tz']:.4f}) "
        f"HEAD_T_TCP t=({row['head_t_tcp_tx']:.4f},"
        f"{row['head_t_tcp_ty']:.4f},{row['head_t_tcp_tz']:.4f})"
    )
    _save_samples(output_path)


def _history_is_stable(history, window_sec: float, cam_thresh_m: float, tcp_thresh_m: float):
    now = time.time()
    recent = [item for item in history if now - item[0] <= window_sec]
    if len(recent) < 3:
        return False, "waiting stable history"
    cam_pts = np.asarray([item[1][:3, 3] for item in recent], dtype=float)
    tcp_pts = np.asarray([item[2][:3, 3] for item in recent], dtype=float)
    cam_span = float(np.max(np.linalg.norm(cam_pts - cam_pts.mean(axis=0), axis=1)))
    tcp_span = float(np.max(np.linalg.norm(tcp_pts - tcp_pts.mean(axis=0), axis=1)))
    if cam_span > cam_thresh_m:
        return False, f"marker moving {cam_span * 1000:.1f}mm"
    if tcp_span > tcp_thresh_m:
        return False, f"tcp moving {tcp_span * 1000:.1f}mm"
    return True, f"stable cam={cam_span * 1000:.1f}mm tcp={tcp_span * 1000:.1f}mm"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws-url", default=config.WS_URL)
    parser.add_argument("--arm", choices=["right", "left"], default="right")
    parser.add_argument("--board-config", default=DEFAULT_BOARD_CONFIG)
    parser.add_argument("--output", default=None)
    parser.add_argument("--min-markers", type=int, default=2)
    parser.add_argument("--axis-len", type=float, default=0.06)
    parser.add_argument(
        "--pose-source",
        choices=["depth-centers", "pnp"],
        default="depth-centers",
        help="depth-centers uses RealSense depth at marker centers; pnp uses RGB intrinsics only.",
    )
    parser.add_argument("--center-depth-radius", type=int, default=4)
    parser.add_argument(
        "--auto-capture-sec",
        type=float,
        default=0.0,
        help="Automatically capture one valid sample every N seconds. 0 disables auto capture.",
    )
    parser.add_argument("--stable-window-sec", type=float, default=0.8)
    parser.add_argument("--stable-marker-mm", type=float, default=6.0)
    parser.add_argument("--stable-tcp-mm", type=float, default=4.0)
    parser.add_argument(
        "--draw-vendor-tcp-axis",
        action="store_true",
        help="Use tcp_to_board from config only for visualizing TCP axis; solver does not trust it.",
    )
    parser.add_argument("--window", default="Vendor Handback CAM2HEAD Collector")
    args = parser.parse_args()

    if args.output:
        output_path = args.output
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "data",
            f"vendor_board_cam2head_samples_{ts}.csv",
        )

    board_cfg, dictionary, marker_size, markers = _load_board_config(args.board_config)
    tcp_t_board = _tcp_to_board_from_config(board_cfg)

    print("=" * 70)
    print("  厂家手背标定板 CAM2HEAD 样本采集（只读，不发运动命令）")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  TCP topic: {TCP_TOPICS[args.arm]}")
    print(f"  Board config: {args.board_config}")
    print(f"  Dictionary: {board_cfg.get('dictionary')} marker_size={marker_size:.4f}m")
    print(f"  Output: {output_path}")
    print("  操作: c=记录当前姿态 | s=保存 | q=退出")
    print("=" * 70)

    aux_client = _connect_roslibpy(args.ws_url)
    subscribers = _start_aux_subscribers(aux_client, args.arm)
    ros_client = ROSClient(ws_url=args.ws_url)
    if not ros_client.connect():
        raise SystemExit(1)

    cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(args.window, _mouse_callback)
    last_frame_count = -1
    last_cam_t_board = None
    previous_cam_t_board = None
    last_used_ids = []
    last_auto_capture_at = 0.0
    pose_history = deque(maxlen=60)
    stable_ok = False
    stable_reason = "waiting"
    try:
        while True:
            global _capture_requested
            rgb, depth, cam_info, frame_count = ros_client.get_frames()
            if rgb is None or frame_count == last_frame_count:
                key = cv2.waitKey(5) & 0xFF
                if key == ord("q"):
                    break
                continue
            last_frame_count = frame_count
            shown = rgb.copy()
            corners, ids, _ = _detect_markers(shown, dictionary)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(shown, corners, ids)
            if cam_info and "K" in cam_info:
                if args.pose_source == "depth-centers":
                    last_cam_t_board, last_used_ids = _estimate_cam_t_board_depth_centers(
                        corners,
                        ids,
                        markers,
                        depth,
                        cam_info,
                        args.center_depth_radius,
                    )
                else:
                    last_cam_t_board, last_used_ids = _estimate_cam_t_board(
                        corners, ids, markers, marker_size, cam_info, previous_cam_t_board
                    )
                if last_cam_t_board is not None:
                    previous_cam_t_board = last_cam_t_board.copy()
                    with _lock:
                        current_tcp_msg = dict(_latest_tcp_pose) if _latest_tcp_pose is not None else None
                    base_t_head = _lookup_transform("BASE", "HEAD")
                    if current_tcp_msg is not None and base_t_head is not None:
                        current_head_t_tcp = np.linalg.inv(base_t_head) @ _pose_msg_to_matrix(current_tcp_msg)
                        pose_history.append((time.time(), last_cam_t_board.copy(), current_head_t_tcp.copy()))
                        stable_ok, stable_reason = _history_is_stable(
                            pose_history,
                            args.stable_window_sec,
                            args.stable_marker_mm / 1000.0,
                            args.stable_tcp_mm / 1000.0,
                        )
                    else:
                        stable_ok, stable_reason = False, "missing tcp/tf"
                    _draw_pose_axis(shown, last_cam_t_board, cam_info, args.axis_len, "BOARD")
                    if args.draw_vendor_tcp_axis and tcp_t_board is not None:
                        cam_t_tcp = last_cam_t_board @ np.linalg.inv(tcp_t_board)
                        _draw_pose_axis(shown, cam_t_tcp, cam_info, args.axis_len, "TCP?")
            else:
                last_cam_t_board, last_used_ids = None, []
                stable_ok, stable_reason = False, "missing camera_info"

            status = f"samples={len(_samples)} markers={len(last_used_ids)} ids={last_used_ids}"
            color = (0, 255, 0) if last_cam_t_board is not None and len(last_used_ids) >= args.min_markers and stable_ok else (0, 0, 255)
            cv2.putText(shown, status, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.putText(shown, "c/click capture | s save | q quit", (12, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(shown, stable_reason, (12, 94), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
            if args.draw_vendor_tcp_axis:
                cv2.putText(
                    shown,
                    "TCP? axis uses vendor T/R for display only",
                    (12, 126),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 255),
                    2,
                )
            cv2.imshow(args.window, shown)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                _save_samples(output_path)
            should_auto_capture = (
                args.auto_capture_sec > 0.0
                and last_cam_t_board is not None
                and len(last_used_ids) >= args.min_markers
                and stable_ok
                and time.time() - last_auto_capture_at >= args.auto_capture_sec
            )
            should_capture = key == ord("c") or _capture_requested or should_auto_capture
            if should_capture:
                _capture_requested = False
                if should_auto_capture:
                    last_auto_capture_at = time.time()
                if last_cam_t_board is None or len(last_used_ids) < args.min_markers:
                    print(f"[!] 未记录: marker 数不足 {args.min_markers}，当前 {len(last_used_ids)}")
                    continue
                if not stable_ok:
                    print(f"[!] 未记录: 姿态未稳定，{stable_reason}")
                    continue
                with _lock:
                    tcp_msg = dict(_latest_tcp_pose) if _latest_tcp_pose is not None else None
                _record_sample(last_cam_t_board, last_used_ids, tcp_msg, output_path)
    except KeyboardInterrupt:
        print("\n[*] 中断退出")
    finally:
        _save_samples(output_path)
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
