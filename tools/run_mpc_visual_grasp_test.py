#!/usr/bin/env python3
"""
Build a first MPC grasp-test request from the vision result.

Safety posture:
- Default is dry-run only.
- Uses vendor CAM2HEAD plus live TF BASE->HEAD when available.
- --execute requires --confirm-target and is distance-limited by default.
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import sys
import threading
import time
from typing import Any

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot_grasp import config
from robot_grasp.coordinate_utils import camera_point_to_base_pose, make_transform
from robot_grasp.grasp_flow import object_conf, select_grasp_target, summarize_target

try:
    import roslibpy
    from roslibpy.core import ServiceException
except ImportError:
    print("缺少 roslibpy 库，请运行: pip install roslibpy")
    sys.exit(1)


MPC_PREFIX = "/wa"
POINTS_SERVICE = f"{MPC_PREFIX}/points_seq_tracking"
POINTS_WITH_JOINTS_SERVICE = f"{MPC_PREFIX}/points_seq_tracking_with_joints"
CURRENT_STATE_TOPIC = "/DualArmMobile/currenState"
MOTION_EPS = 1e-3
ORIENTATION_PRESETS = {
    # Course example: Eigen::Quaterniond(0.707, 0, -0.707, 0)
    # geometry_msgs order is x, y, z, w.
    "doc-grasp": {"x": 0.0, "y": -0.70710678, "z": 0.0, "w": 0.70710678},
}
FRAME_TOPICS = {
    "left": "/DualArmMobile/currentEEPose/FrameL",
    "right": "/DualArmMobile/currentEEPose/FrameR",
}
DEFAULT_CAM2HEAD = os.path.join(
    PROJECT_ROOT,
    "handeye_calibration",
    "calibration",
    "cam2head_vendor_new_20260803.json",
)
DEFAULT_LOCKED_TARGET = os.path.join(PROJECT_ROOT, "data", "mpc_locked_target_latest.json")
DEFAULT_GRASP_OFFSET_X = -0.04
DEFAULT_GRASP_OFFSET_Y = -0.10
DEFAULT_GRASP_OFFSET_Z = 0.35


def _parse_ws_url(ws_url: str) -> tuple[str, int]:
    stripped = ws_url.replace("ws://", "").replace("wss://", "")
    host, port = stripped.split(":")
    return host, int(port)


def _connect(ws_url: str):
    host, port = _parse_ws_url(ws_url)
    client = roslibpy.Ros(host=host, port=port)
    thread = threading.Thread(target=client.run, daemon=True)
    thread.start()

    start = time.time()
    while not client.is_connected:
        if time.time() - start > config.CONNECT_TIMEOUT:
            print(f"[✗] 连接超时: {ws_url}")
            sys.exit(1)
        time.sleep(0.1)
    return client


def _call(client, name: str, service_type: str, request: dict | None = None) -> dict:
    service = roslibpy.Service(client, name, service_type)
    return service.call(roslibpy.ServiceRequest(request or {}))


def _print_mpc_target_manifest_hint(exc: Exception) -> bool:
    text = str(exc)
    if "mpc_target" not in text or "Unable to load the manifest" not in text:
        return False
    print("[✗] rosbridge 进程没有加载 mpc_target 包，MPC service call 无法序列化。")
    print("    注意：你当前 shell 里 rospack find mpc_target 成功，不代表 rosbridge 进程也 source 了同一个环境。")
    print("    请在启动 rosbridge 的终端/脚本里执行：")
    print("      source /opt/ros/noetic/setup.bash")
    print("      source /workspace/catkin_ws/mpc_ws/devel/setup.bash")
    print("      roslaunch rosbridge_server rosbridge_websocket.launch")
    print("    如果 rosbridge 是 systemd/supervisor/docker entrypoint 启动，要把 mpc_ws/devel/setup.bash 写进那个启动脚本。")
    return True


def _service_type(client, name: str) -> str:
    try:
        return _call(client, "/rosapi/service_type", "rosapi/ServiceType", {"service": name}).get("type", "")
    except Exception:
        return ""


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


def _load_transform(path: str, name: str) -> np.ndarray:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    matrix = data.get("transform_4x4", data) if isinstance(data, dict) else data
    return make_transform(matrix, name)


def _load_locked_target(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _load_orientation_file(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if "orientation" in data:
        return data["orientation"]
    if "pose" in data and "orientation" in data["pose"]:
        return data["pose"]["orientation"]
    raise ValueError(f"orientation file does not contain orientation: {path}")


def _load_pose_file(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    pose = data.get("pose", data) if isinstance(data, dict) else {}
    if "position" not in pose or "orientation" not in pose:
        raise ValueError(f"pose file does not contain full pose: {path}")
    position = pose["position"]
    return {
        "position": {
            "x": float(position["x"]),
            "y": float(position["y"]),
            "z": float(position["z"]),
        },
        "orientation": _normalize_orientation(pose["orientation"]),
    }


def _load_mpc_state_file(path: str) -> list[float] | None:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    state = data.get("mpc_state")
    if state is None:
        return None
    return [float(value) for value in state]


def _save_locked_target(path: str, payload: dict):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)


def _quat_to_matrix(q: dict) -> np.ndarray:
    x = float(q.get("x", 0.0))
    y = float(q.get("y", 0.0))
    z = float(q.get("z", 0.0))
    w = float(q.get("w", 1.0))
    norm = float(np.linalg.norm([x, y, z, w]))
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


def _tf_to_matrix(message: dict) -> np.ndarray:
    tf = message.get("transform", {})
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


def _sample_tf(client, sample_seconds: float) -> dict[tuple[str, str], dict]:
    transforms: dict[tuple[str, str], dict] = {}
    lock = threading.Lock()

    def callback(message):
        with lock:
            for transform in message.get("transforms", []):
                parent = transform.get("header", {}).get("frame_id", "").lstrip("/")
                child = transform.get("child_frame_id", "").lstrip("/")
                if parent and child:
                    transforms[(parent, child)] = transform

    subscribers = [
        roslibpy.Topic(client, "/tf_static", "tf2_msgs/TFMessage"),
        roslibpy.Topic(client, "/tf", "tf2_msgs/TFMessage"),
    ]
    for sub in subscribers:
        sub.subscribe(callback)
    time.sleep(sample_seconds)
    for sub in subscribers:
        try:
            sub.unsubscribe()
        except Exception:
            pass
    with lock:
        return dict(transforms)


def _build_graph(transforms: dict[tuple[str, str], dict]):
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


def _lookup_transform(transforms: dict[tuple[str, str], dict], start: str, goal: str) -> np.ndarray | None:
    """Return T_start_goal, mapping homogeneous points from goal frame to start frame."""
    path = _find_path(_build_graph(transforms), start, goal)
    if path is None:
        return None
    result = np.eye(4)
    for direction, transform in path:
        step = _tf_to_matrix(transform)
        if direction == "inverse":
            step = np.linalg.inv(step)
        result = result @ step
    return result


def _wait_for_pose(client, topic: str, timeout: float) -> dict | None:
    latest = {}
    event = threading.Event()

    def callback(message):
        latest["message"] = message
        event.set()

    sub = roslibpy.Topic(client, topic, "geometry_msgs/PoseStamped")
    sub.subscribe(callback)
    ok = event.wait(timeout)
    try:
        sub.unsubscribe()
    except Exception:
        pass
    if not ok:
        return None
    return latest.get("message")


def _wait_for_current_state(client, timeout: float) -> dict | None:
    latest = {}
    event = threading.Event()

    def callback(message):
        latest["message"] = message
        event.set()

    sub = roslibpy.Topic(client, CURRENT_STATE_TOPIC, "ocs2_msgs/mpc_target_trajectories")
    sub.subscribe(callback)
    ok = event.wait(timeout)
    try:
        sub.unsubscribe()
    except Exception:
        pass
    if not ok:
        return None
    return latest.get("message")


def _extract_current_mpc_state(message: dict) -> list[float] | None:
    states = message.get("stateTrajectory", [])
    if not states:
        return None
    return [float(value) for value in states[0].get("value", [])]


def _pose_stamped_to_pose(message: dict) -> dict:
    pose = message.get("pose", {})
    return {
        "position": {
            "x": float(pose.get("position", {}).get("x", 0.0)),
            "y": float(pose.get("position", {}).get("y", 0.0)),
            "z": float(pose.get("position", {}).get("z", 0.0)),
        },
        "orientation": {
            "x": float(pose.get("orientation", {}).get("x", 0.0)),
            "y": float(pose.get("orientation", {}).get("y", 0.0)),
            "z": float(pose.get("orientation", {}).get("z", 0.0)),
            "w": float(pose.get("orientation", {}).get("w", 1.0)),
        },
    }


def _pose_array(poses: list[dict]) -> dict:
    return {
        "header": {"seq": 0, "stamp": {"secs": 0, "nsecs": 0}, "frame_id": ""},
        "poses": poses,
    }


def _hold_poses(pose: dict, count: int) -> list[dict]:
    return [json.loads(json.dumps(pose)) for _ in range(count)]


def _build_points_request(arm: str, poses: list[dict],
                          duration: float, weight: float, way_type: str,
                          hold_pose: dict) -> dict:
    hold = _hold_poses(hold_pose, len(poses))
    return {
        "left_poses": _pose_array(hold) if arm == "right" else _pose_array(poses),
        "right_poses": _pose_array(poses) if arm == "right" else _pose_array(hold),
        "time_points": [duration for _ in poses],
        "max_period": duration * len(poses) + 2.0,
        "weight": weight,
        "type": way_type,
    }


def _build_points_with_joints_request(arm: str, poses: list[dict], states: list[list[float]],
                                      duration: float, weight: float, way_type: str,
                                      hold_pose: dict) -> dict:
    hold = _hold_poses(hold_pose, len(poses))
    joint_num = len(states[0]) if states else 0
    return {
        "left_poses": _pose_array(hold) if arm == "right" else _pose_array(poses),
        "right_poses": _pose_array(poses) if arm == "right" else _pose_array(hold),
        "time_points": [duration for _ in poses],
        "states": [float(value) for state in states for value in state],
        "joint_num": joint_num,
        "max_period": duration * len(poses) + 2.0,
        "weight": weight,
        "type": way_type,
    }


def _camera_point_m(target: dict) -> tuple[float, float, float]:
    return (
        float(target["x_mm"]) / 1000.0,
        float(target["y_mm"]) / 1000.0,
        float(target["z_mm"]) / 1000.0,
    )


def _object_base_from_target(target: dict, cam2head: np.ndarray, head_to_base: np.ndarray) -> np.ndarray:
    pose = camera_point_to_base_pose(
        x_mm=float(target["x_mm"]),
        y_mm=float(target["y_mm"]),
        z_mm=float(target["z_mm"]),
        cam2head=cam2head,
        head_to_base=head_to_base,
    )
    return pose[:3, 3]


def _with_position(current_pose: dict, position: np.ndarray) -> dict:
    pose = json.loads(json.dumps(current_pose))
    pose["position"] = {
        "x": float(position[0]),
        "y": float(position[1]),
        "z": float(position[2]),
    }
    return pose


def _normalize_orientation(orientation: dict) -> dict:
    x = float(orientation.get("x", 0.0))
    y = float(orientation.get("y", 0.0))
    z = float(orientation.get("z", 0.0))
    w = float(orientation.get("w", 1.0))
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    if norm == 0:
        raise ValueError("orientation quaternion norm is 0")
    return {"x": x / norm, "y": y / norm, "z": z / norm, "w": w / norm}


def _target_orientation(args) -> dict | None:
    if args.orientation_file:
        return _normalize_orientation(_load_orientation_file(args.orientation_file))
    if args.orientation:
        return _normalize_orientation(
            {
                "x": args.orientation[0],
                "y": args.orientation[1],
                "z": args.orientation[2],
                "w": args.orientation[3],
            }
        )
    if args.orientation_preset != "current":
        return _normalize_orientation(ORIENTATION_PRESETS[args.orientation_preset])
    return None


def _with_orientation(pose: dict, orientation: dict | None) -> dict:
    if orientation is None:
        return pose
    target = json.loads(json.dumps(pose))
    target["orientation"] = orientation
    return target


def _orientation_array(pose: dict) -> np.ndarray:
    ori = pose["orientation"]
    return np.array([ori["x"], ori["y"], ori["z"], ori["w"]], dtype=float)


def _array_orientation(values: np.ndarray) -> dict:
    values = values.astype(float)
    norm = float(np.linalg.norm(values))
    if norm == 0.0:
        values = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    else:
        values = values / norm
    return {
        "x": float(values[0]),
        "y": float(values[1]),
        "z": float(values[2]),
        "w": float(values[3]),
    }


def _slerp_orientation(start_pose: dict, target_pose: dict, ratio: float) -> dict:
    q0 = _orientation_array(start_pose)
    q1 = _orientation_array(target_pose)
    q0 = q0 / float(np.linalg.norm(q0))
    q1 = q1 / float(np.linalg.norm(q1))
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = min(max(dot, -1.0), 1.0)
    if dot > 0.9995:
        return _array_orientation(q0 + ratio * (q1 - q0))

    theta_0 = float(np.arccos(dot))
    theta = theta_0 * ratio
    sin_theta = float(np.sin(theta))
    sin_theta_0 = float(np.sin(theta_0))
    s0 = float(np.cos(theta)) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return _array_orientation((s0 * q0) + (s1 * q1))


def _distance(a: dict, b: dict) -> float:
    ap = a["position"]
    bp = b["position"]
    return float(
        (
            (ap["x"] - bp["x"]) ** 2
            + (ap["y"] - bp["y"]) ** 2
            + (ap["z"] - bp["z"]) ** 2
        ) ** 0.5
    )


def _pose_position(pose: dict) -> np.ndarray:
    pos = pose["position"]
    return np.array([pos["x"], pos["y"], pos["z"]], dtype=float)


def _clip_poses_to_path_distance(poses: list[dict], distance: float) -> list[dict]:
    if distance <= 0.0 or len(poses) <= 1:
        return poses

    clipped = [poses[0]]
    remaining = float(distance)
    for target in poses[1:]:
        start_pos = _pose_position(clipped[-1])
        target_pos = _pose_position(target)
        delta = target_pos - start_pos
        segment_len = float(np.linalg.norm(delta))
        if segment_len <= 1e-9:
            clipped.append(target)
            continue
        if remaining >= segment_len:
            clipped.append(target)
            remaining -= segment_len
            continue

        ratio = remaining / segment_len
        step_pos = start_pos + delta * ratio
        step_pose = _with_position(target, step_pos)
        step_pose["orientation"] = _slerp_orientation(clipped[-1], target, ratio)
        clipped.append(step_pose)
        return clipped
    return clipped


def _interp_state(start: list[float] | None, target: list[float] | None, ratio: float) -> list[float] | None:
    if start is None or target is None:
        return None
    if len(start) != len(target):
        raise ValueError(f"joint state length mismatch: {len(start)} vs {len(target)}")
    return [float(a + (b - a) * ratio) for a, b in zip(start, target)]


def _clip_path_to_distance(poses: list[dict], states: list[list[float]] | None,
                           distance: float) -> tuple[list[dict], list[list[float]] | None]:
    if states is None:
        return _clip_poses_to_path_distance(poses, distance), None
    if distance <= 0.0 or len(poses) <= 1:
        return poses, states

    clipped_poses = [poses[0]]
    clipped_states = [states[0]]
    remaining = float(distance)
    for pose, state in zip(poses[1:], states[1:]):
        start_pos = _pose_position(clipped_poses[-1])
        target_pos = _pose_position(pose)
        delta = target_pos - start_pos
        segment_len = float(np.linalg.norm(delta))
        if segment_len <= 1e-9:
            clipped_poses.append(pose)
            clipped_states.append(state)
            continue
        if remaining >= segment_len:
            clipped_poses.append(pose)
            clipped_states.append(state)
            remaining -= segment_len
            continue

        ratio = remaining / segment_len
        step_pos = start_pos + delta * ratio
        step_pose = _with_position(pose, step_pos)
        step_pose["orientation"] = _slerp_orientation(clipped_poses[-1], pose, ratio)
        step_state = _interp_state(clipped_states[-1], state, ratio)
        if step_state is None:
            raise ValueError("missing joint state while clipping joint path")
        clipped_poses.append(step_pose)
        clipped_states.append(step_state)
        return clipped_poses, clipped_states
    return clipped_poses, clipped_states


def _path_length(poses: list[dict]) -> float:
    total = 0.0
    for start, target in zip(poses, poses[1:]):
        total += float(np.linalg.norm(_pose_position(target) - _pose_position(start)))
    return total


def _print_vec(label: str, vec: np.ndarray | tuple[float, float, float]):
    print(f"{label}: x={vec[0]: .4f} m, y={vec[1]: .4f} m, z={vec[2]: .4f} m")


def _check_workspace(position: np.ndarray, args) -> list[str]:
    errors = []
    if not (args.min_x <= position[0] <= args.max_x):
        errors.append(f"x={position[0]:.3f} 超出 [{args.min_x:.3f}, {args.max_x:.3f}]")
    if not (args.min_y <= position[1] <= args.max_y):
        errors.append(f"y={position[1]:.3f} 超出 [{args.min_y:.3f}, {args.max_y:.3f}]")
    if not (args.min_z <= position[2] <= args.max_z):
        errors.append(f"z={position[2]:.3f} 超出 [{args.min_z:.3f}, {args.max_z:.3f}]")
    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws-url", default=config.WS_URL, help="rosbridge WebSocket URL")
    parser.add_argument("--arm", choices=["right", "left"], default="right")
    parser.add_argument("--csv", default=None, help="Use object_results from a specific grasp_data CSV.")
    parser.add_argument("--preferred-label", default="plastic bag")
    parser.add_argument("--target-idx", type=int, default=None)
    parser.add_argument("--cam2head", default=DEFAULT_CAM2HEAD)
    parser.add_argument("--save-target", nargs="?", const=DEFAULT_LOCKED_TARGET, default=None,
                        help="Save computed BASE target to this JSON path; default path is data/mpc_locked_target_latest.json.")
    parser.add_argument("--use-locked-target", nargs="?", const=DEFAULT_LOCKED_TARGET, default=None,
                        help="Use a saved BASE target JSON instead of recomputing from CSV and current HEAD TF.")
    parser.add_argument("--tf-seconds", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--approach-height",
        "--above-object-height",
        dest="approach_height",
        type=float,
        default=0.10,
        help="Final descend target height above the locked object point, in meters.",
    )
    parser.add_argument("--safe-travel-z", type=float, default=0.95, help="Travel height before moving above the object.")
    parser.add_argument("--no-auto-lift", action="store_true",
                        help="Do not insert the automatic vertical lift before via/target points.")
    parser.add_argument("--include-descend", action="store_true", help="Also include the lower pre-grasp point. Default only moves above target at safe height.")
    parser.add_argument(
        "--offset-x",
        type=float,
        default=DEFAULT_GRASP_OFFSET_X,
        help="TCP x compensation from visual object point in BASE/MPC meters. Default -0.04m toward body.",
    )
    parser.add_argument(
        "--offset-y",
        type=float,
        default=DEFAULT_GRASP_OFFSET_Y,
        help="TCP y compensation from visual object point in BASE/MPC meters. Default -0.10m toward robot right.",
    )
    parser.add_argument(
        "--offset-z",
        type=float,
        default=DEFAULT_GRASP_OFFSET_Z,
        help="TCP z compensation from visual object point in BASE/MPC meters. Default 0.35m from teammate grasp test.",
    )
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--execute-delay", type=float, default=2.0,
                        help="Seconds to wait before sending an execute request. Use 0 to disable.")
    parser.add_argument("--weight", type=float, default=1.0)
    parser.add_argument("--type", choices=["quintic", "spline"], default="quintic")
    parser.add_argument("--orientation-preset", choices=["current", *ORIENTATION_PRESETS.keys()], default="current",
                        help="Target TCP orientation preset. Default keeps current orientation.")
    parser.add_argument("--orientation", type=float, nargs=4, metavar=("X", "Y", "Z", "W"),
                        help="Target TCP orientation quaternion in geometry_msgs order x y z w.")
    parser.add_argument("--orientation-file", default=None,
                        help="JSON saved by tools/capture_mpc_pose.py; uses its orientation.")
    parser.add_argument("--orientation-apply", choices=["final", "all", "none"], default="final",
                        help="How to apply target orientation. Default final avoids twisting during small step tests.")
    parser.add_argument("--via-point", type=float, nargs=3, action="append", metavar=("X", "Y", "Z"),
                        help="Add an explicit MPC/BASE waypoint in meters. Can be repeated.")
    parser.add_argument("--via-file", action="append", default=None,
                        help="JSON saved by tools/capture_mpc_pose.py; appends a full pose waypoint. Can be repeated.")
    parser.add_argument("--stop-at-last-via", action="store_true",
                        help="Stop the generated path at the last via waypoint instead of appending the visual target.")
    parser.add_argument("--use-joints", action="store_true",
                        help="Call /wa/points_seq_tracking_with_joints using mpc_state saved in each --via-file.")
    parser.add_argument("--joint-weight", type=float, default=None,
                        help="Override weight when --use-joints is enabled. Default keeps --weight.")
    parser.add_argument("--step-distance", type=float, default=0.0,
                        help="Move only this many meters from current pose toward the generated target. 0 means use full generated path.")
    parser.add_argument("--max-motion", type=float, default=0.03, help="Max allowed execute distance in meters.")
    parser.add_argument("--min-x", type=float, default=-0.20)
    parser.add_argument("--max-x", type=float, default=1.00)
    parser.add_argument("--min-y", type=float, default=-0.80)
    parser.add_argument("--max-y", type=float, default=0.30)
    parser.add_argument("--min-z", type=float, default=0.20)
    parser.add_argument("--max-z", type=float, default=1.20)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm-target", action="store_true", help="Required together with --execute.")
    args = parser.parse_args()

    locked = _load_locked_target(args.use_locked_target) if args.use_locked_target else None

    if locked:
        csv_path = locked.get("source_csv", "<locked>")
        target = locked.get("target", {})
        object_base = np.array(locked["object_base_m"], dtype=float)
        source_mode = f"locked target: {args.use_locked_target}"
        point_cam = np.array(locked.get("camera_point_m", [0.0, 0.0, 0.0]), dtype=float)
        point_head = np.array(locked.get("head_point_m", [0.0, 0.0, 0.0]), dtype=float)
    else:
        csv_path = args.csv or _latest_grasp_csv()
        if not csv_path:
            print("[✗] 没找到 data/grasp_data_*.csv，请先运行 run_grasp.py 并保存一次")
            raise SystemExit(1)

        objects = _load_latest_objects(csv_path)
        if args.target_idx is not None:
            candidates = [obj for obj in objects if int(obj.get("idx", -1)) == args.target_idx]
            target = candidates[0] if candidates else None
        else:
            target = select_grasp_target(objects, preferred_label=args.preferred_label)
        if target is None:
            print(f"[✗] CSV 中没有可用目标: {csv_path}")
            print("    可尝试降低 --preferred-label 限制，或重新运行 run_grasp.py 保存更稳定的数据")
            raise SystemExit(1)

        cam2head = _load_transform(args.cam2head, "cam2head")
        point_cam = np.array(_camera_point_m(target), dtype=float)
        point_head = (cam2head @ np.array([*point_cam, 1.0], dtype=float))[:3]
        source_mode = "vision csv + live TF"

    print("=" * 70)
    print("  MPC visual grasp test request")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  CSV: {csv_path}")
    print(f"  CAM2HEAD: {args.cam2head}")
    print(f"  Source: {source_mode}")
    print(f"  Arm: {args.arm}")
    print(f"  Execute: {args.execute}")
    print("=" * 70)
    print(f"\n选择目标: {summarize_target(target)}")
    print(f"confidence={object_conf(target):.3f}")
    _print_vec("camera point", point_cam)
    _print_vec("head point  ", point_head)

    client = _connect(args.ws_url)
    print("\n[✓] 已连接 rosbridge")
    try:
        if not locked:
            print(f"[*] 采样 TF {args.tf_seconds:.1f}s，查找 BASE -> HEAD")
            transforms = _sample_tf(client, args.tf_seconds)
            head_to_base = _lookup_transform(transforms, "BASE", "HEAD")
            if head_to_base is None:
                print("[✗] TF 中没有找到 BASE -> HEAD，不能生成 MPC/BASE 目标")
                print("    现在只完成了 camera -> HEAD。请先确认 TF 帧名，或补 HEAD2BASE 矩阵。")
                raise SystemExit(1)

            object_base = _object_base_from_target(target, cam2head, head_to_base)
        pregrasp_position = object_base + np.array(
            [args.offset_x, args.offset_y, args.approach_height + args.offset_z],
            dtype=float,
        )
        _print_vec("object base ", object_base)
        _print_vec("pregrasp   ", pregrasp_position)
        if args.save_target:
            payload = {
                "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "source_csv": csv_path,
                "target": target,
                "camera_point_m": point_cam.tolist(),
                "head_point_m": point_head.tolist(),
                "object_base_m": object_base.tolist(),
                "default_approach_height_m": float(args.approach_height),
                "note": "Locked BASE target. Reuse this after neck pose changes; do not recompute from old CSV with a different HEAD TF.",
            }
            _save_locked_target(args.save_target, payload)
            print(f"[✓] 已锁存 BASE 目标: {args.save_target}")

        topic = FRAME_TOPICS[args.arm]
        hold_arm = "left" if args.arm == "right" else "right"
        hold_topic = FRAME_TOPICS[hold_arm]
        print(f"\n[*] 读取当前 MPC 末端位姿: {topic}")
        message = _wait_for_pose(client, topic, args.timeout)
        if message is None:
            print(f"[✗] {args.timeout:.1f}s 内没有收到 {topic}")
            raise SystemExit(1)
        print(f"[*] 读取保持不动侧 MPC 末端位姿: {hold_topic}")
        hold_message = _wait_for_pose(client, hold_topic, args.timeout)
        if hold_message is None:
            print(f"[✗] {args.timeout:.1f}s 内没有收到 {hold_topic}")
            raise SystemExit(1)
        current_mpc_state = None
        if args.use_joints:
            print(f"[*] 读取当前 MPC joint state: {CURRENT_STATE_TOPIC}")
            state_message = _wait_for_current_state(client, args.timeout)
            if state_message is None:
                print(f"[✗] {args.timeout:.1f}s 内没有收到 {CURRENT_STATE_TOPIC}")
                raise SystemExit(1)
            current_mpc_state = _extract_current_mpc_state(state_message)
            if not current_mpc_state:
                print("[✗] currenState 中没有 stateTrajectory[0].value")
                raise SystemExit(1)
            print(f"    joint_num={len(current_mpc_state)}")
        current_pose = _pose_stamped_to_pose(message)
        hold_pose = _pose_stamped_to_pose(hold_message)
        current_position = np.array(
            [
                current_pose["position"]["x"],
                current_pose["position"]["y"],
                current_pose["position"]["z"],
            ],
            dtype=float,
        )
        travel_z = max(float(args.safe_travel_z), float(current_position[2]), float(pregrasp_position[2]))
        lift_position = current_position.copy()
        lift_position[2] = travel_z
        above_position = pregrasp_position.copy()
        above_position[2] = travel_z
        orientation = _target_orientation(args)

        poses = [current_pose]
        joint_states = [current_mpc_state] if args.use_joints else None
        if not args.no_auto_lift and float(np.linalg.norm(lift_position - current_position)) > 0.005:
            poses.append(_with_position(current_pose, lift_position))
            if joint_states is not None:
                joint_states.append(current_mpc_state)
        has_explicit_via = bool(args.via_point or args.via_file)
        if args.via_point:
            if args.use_joints:
                print("[✗] --use-joints 不能配合 --via-point；请使用带 mpc_state 的 --via-file")
                raise SystemExit(1)
            for via in args.via_point:
                poses.append(_with_position(current_pose, np.array(via, dtype=float)))
        if args.via_file:
            for path in args.via_file:
                poses.append(_load_pose_file(path))
                if joint_states is not None:
                    state = _load_mpc_state_file(path)
                    if state is None:
                        print(f"[✗] {path} 中没有 mpc_state。请用 capture_mpc_pose.py --include-joints 重新记录")
                        raise SystemExit(1)
                    if len(state) != len(current_mpc_state):
                        print(
                            f"[✗] {path} joint_num={len(state)} 与当前 joint_num={len(current_mpc_state)} 不一致"
                        )
                        raise SystemExit(1)
                    joint_states.append(state)
        if args.stop_at_last_via and not has_explicit_via:
            print("[✗] --stop-at-last-via 需要至少一个 --via-file 或 --via-point")
            raise SystemExit(1)
        if args.stop_at_last_via and args.include_descend:
            print("[✗] --stop-at-last-via 不能和 --include-descend 同时使用")
            raise SystemExit(1)
        if not args.stop_at_last_via:
            poses.append(_with_position(current_pose, above_position))
            if joint_states is not None:
                joint_states.append(joint_states[-1])
        if args.include_descend and not args.stop_at_last_via:
            poses.append(_with_position(current_pose, pregrasp_position))
            if joint_states is not None:
                joint_states.append(joint_states[-1])

        full_target_pose = poses[-1]
        full_motion = _distance(current_pose, full_target_pose)
        full_path_length = _path_length(poses)
        planned_poses = json.loads(json.dumps(poses))
        reaches_full_target = True
        if args.step_distance > 0.0:
            if args.step_distance > args.max_motion + MOTION_EPS:
                print(f"[✗] --step-distance {args.step_distance:.3f}m 不能大于 --max-motion {args.max_motion:.3f}m")
                raise SystemExit(1)
            reaches_full_target = args.step_distance >= full_path_length - MOTION_EPS
            poses, joint_states = _clip_path_to_distance(poses, joint_states, args.step_distance)

        if orientation is not None:
            if args.orientation_apply == "all":
                poses = [poses[0]] + [_with_orientation(pose, orientation) for pose in poses[1:]]
                planned_poses = [planned_poses[0]] + [
                    _with_orientation(pose, orientation) for pose in planned_poses[1:]
                ]
            elif args.orientation_apply == "final":
                planned_poses[-1] = _with_orientation(planned_poses[-1], orientation)
                if reaches_full_target:
                    poses[-1] = _with_orientation(poses[-1], orientation)

        target_pose = poses[-1]
        if args.use_joints:
            if joint_states is None or any(state is None for state in joint_states):
                print("[✗] --use-joints 需要每个路径点都有 joint state")
                raise SystemExit(1)
            request = _build_points_with_joints_request(
                arm=args.arm,
                poses=poses,
                states=joint_states,
                duration=args.duration,
                weight=args.joint_weight if args.joint_weight is not None else args.weight,
                way_type=args.type,
                hold_pose=hold_pose,
            )
        else:
            request = _build_points_request(
                arm=args.arm,
                poses=poses,
                duration=args.duration,
                weight=args.weight,
                way_type=args.type,
                hold_pose=hold_pose,
            )

        motion = _distance(current_pose, target_pose)
        workspace_errors = []
        for idx, pose in enumerate(poses):
            pos = pose["position"]
            point = np.array([pos["x"], pos["y"], pos["z"]], dtype=float)
            for error in _check_workspace(point, args):
                workspace_errors.append(f"p{idx}: {error}")

        print("\n当前 MPC 末端 pose:")
        print(json.dumps(current_pose, indent=2, ensure_ascii=False))
        print("\n生成路径:")
        for idx, pose in enumerate(planned_poses):
            pos = pose["position"]
            ori = pose["orientation"]
            print(
                f"  p{idx}: x={pos['x']:.4f}, y={pos['y']:.4f}, z={pos['z']:.4f} "
                f"q=({ori['x']:.4f},{ori['y']:.4f},{ori['z']:.4f},{ori['w']:.4f})"
            )
        if args.step_distance > 0.0:
            print("\n本次裁剪后实际发送路径:")
            for idx, pose in enumerate(poses):
                pos = pose["position"]
                ori = pose["orientation"]
                print(
                    f"  p{idx}: x={pos['x']:.4f}, y={pos['y']:.4f}, z={pos['z']:.4f} "
                    f"q=({ori['x']:.4f},{ori['y']:.4f},{ori['z']:.4f},{ori['w']:.4f})"
                )
        if args.step_distance > 0.0:
            print(f"\n完整路径终点直线距离: {full_motion:.3f} m")
            print(f"完整路径累计长度: {full_path_length:.3f} m")
            print(f"本次小步执行距离: {motion:.3f} m")
        else:
            print(f"\n完整路径终点直线距离: {full_motion:.3f} m")
            print(f"完整路径累计长度: {full_path_length:.3f} m")
        print("\n最终目标 pose:")
        print(json.dumps(target_pose, indent=2, ensure_ascii=False))
        if orientation is not None:
            print(f"\n[!] 已加载目标 TCP orientation，应用模式: {args.orientation_apply}")
            if args.orientation_apply == "final" and not reaches_full_target:
                print("    本次是小步裁剪，尚未到完整路径终点，因此保持当前姿态。")
        print(f"\n保持不动侧 {hold_arm} 末端 pose:")
        print(json.dumps(hold_pose, indent=2, ensure_ascii=False))
        if args.use_joints:
            print("\nMPC joint state 路径:")
            for idx, state in enumerate(joint_states or []):
                print(
                    f"  p{idx}: joint_num={len(state)} "
                    f"waist_or_body_slice={state[3:7] if len(state) >= 7 else state}"
                )
        print(f"\n末端到目标直线距离: {motion:.3f} m")
        if workspace_errors:
            print("[!] workspace 检查:")
            for error in workspace_errors:
                print(f"    - {error}")
        service_name = POINTS_WITH_JOINTS_SERVICE if args.use_joints else POINTS_SERVICE
        print(f"\n准备发送的 request: {service_name}")
        print(json.dumps(request, indent=2, ensure_ascii=False))

        if not args.execute:
            print("\n[DRY RUN] 未发送运动命令。先核对 target base 是否合理。")
            return

        if not args.confirm_target:
            print("[✗] --execute 必须同时加 --confirm-target")
            raise SystemExit(1)
        if workspace_errors:
            print("[✗] 目标超出当前保守 workspace，取消执行")
            raise SystemExit(1)
        execute_distance = _path_length(poses)
        if execute_distance > args.max_motion + MOTION_EPS:
            print(f"[✗] 执行路径累计长度 {execute_distance:.4f}m > --max-motion {args.max_motion:.4f}m，取消执行")
            print("    先 dry-run 查看完整路径累计长度，再设置略大的 --max-motion。")
            raise SystemExit(1)
        if args.duration < 3.0:
            print("[✗] 为了实机安全，duration 不能小于 3.0s")
            raise SystemExit(1)

        srv_type = _service_type(client, service_name)
        if not srv_type:
            print(f"[✗] 找不到服务类型: {service_name}")
            raise SystemExit(1)

        print(f"\n[EXECUTE] 即将调用 {service_name}。请确认手在急停上。")
        if args.execute_delay > 0.0:
            print(f"          {args.execute_delay:.1f} 秒后发送，Ctrl+C 可取消。")
            time.sleep(args.execute_delay)
        else:
            print("          execute-delay=0，立即发送。")
        try:
            response = _call(client, service_name, srv_type, request)
            print(f"[MPC] response: {response}")
        except ServiceException as exc:
            if not _print_mpc_target_manifest_hint(exc):
                raise
    except KeyboardInterrupt:
        print("\n[*] 用户取消")
    finally:
        try:
            client.terminate()
        except Exception:
            pass


if __name__ == "__main__":
    main()
