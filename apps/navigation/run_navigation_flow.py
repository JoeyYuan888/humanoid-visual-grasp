#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Minimal ROS1 navigation action client.

This keeps only the useful part of the vendor demo: send waypoints to the
navigation action server, wait for the result, and print a compact status.
"""

from __future__ import annotations

import argparse
import ast
import math
import os
import sys
import threading
import time
from typing import Iterable

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


SUCCESS_STATE = 6
DEFAULT_ACTION_NAME = "/zj_humanoid/navigation/navigation"
DEFAULT_ODOM_TOPIC = "/zj_humanoid/navigation/odom_info"
DEFAULT_WS_URL = "ws://192.168.20.102:9091"
ACTION_TYPE = "navigation/NavigationAction"


def _load_yaml(path: str) -> dict:
    try:
        import yaml
    except ImportError as exc:
        return _load_navigation_yaml_fallback(path)

    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"配置文件格式错误: {path}")
    return data


def _load_navigation_yaml_fallback(path: str) -> dict:
    """Parse the small configs/navigation.yaml shape when PyYAML is absent."""
    data: dict = {"goals": {}}
    section = None
    current_goal = None
    with open(path, "r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.split("#", 1)[0].rstrip()
            if not line.strip():
                continue
            stripped = line.strip()
            indent = len(line) - len(line.lstrip(" "))

            if indent == 0 and stripped.endswith(":"):
                section = stripped[:-1]
                if section not in data:
                    data[section] = {}
                current_goal = None
                continue

            if section != "goals":
                continue

            if indent == 2 and stripped.endswith(":"):
                current_goal = stripped[:-1]
                data["goals"][current_goal] = {}
                continue

            if indent >= 4 and current_goal and ":" in stripped:
                key, value = stripped.split(":", 1)
                key = key.strip()
                value = value.strip()
                if not value:
                    continue
                if value.startswith("["):
                    data["goals"][current_goal][key] = ast.literal_eval(value)
                else:
                    try:
                        data["goals"][current_goal][key] = float(value)
                    except ValueError:
                        data["goals"][current_goal][key] = value

    return data


def _yaw_to_quaternion(yaw_deg: float) -> tuple[float, float, float, float]:
    yaw = math.radians(yaw_deg)
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _quaternion_to_yaw_deg(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def _parse_waypoint_item(item: Iterable[float]) -> tuple[float, float, float, float, float, float, float]:
    values = tuple(float(v) for v in item)
    if len(values) != 7:
        raise ValueError(f"waypoint 必须是 7 个数: x y z qx qy qz qw，收到 {len(values)} 个")
    return values


def _parse_waypoints_literal(value: str) -> list[tuple[float, float, float, float, float, float, float]]:
    parsed = ast.literal_eval(value)
    if not isinstance(parsed, list):
        raise ValueError("--waypoints 必须是 list")
    return [_parse_waypoint_item(item) for item in parsed]


def _waypoint_from_goal_config(goal: dict) -> tuple[float, float, float, float, float, float, float]:
    if "waypoint" in goal:
        return _parse_waypoint_item(goal["waypoint"])

    x = float(goal["x"])
    y = float(goal["y"])
    z = float(goal.get("z", 0.0))
    if "yaw_deg" in goal:
        qx, qy, qz, qw = _yaw_to_quaternion(float(goal["yaw_deg"]))
    else:
        q = goal.get("orientation", {})
        qx = float(q.get("x", goal.get("qx", 0.0)))
        qy = float(q.get("y", goal.get("qy", 0.0)))
        qz = float(q.get("z", goal.get("qz", 0.0)))
        qw = float(q.get("w", goal.get("qw", 1.0)))
    return x, y, z, qx, qy, qz, qw


def _load_named_goal(config_path: str, name: str) -> list[tuple[float, float, float, float, float, float, float]]:
    config = _load_yaml(config_path)
    goals = config.get("goals", {})
    if name not in goals:
        available = ", ".join(sorted(goals)) or "<none>"
        raise KeyError(f"configs/navigation.yaml 没有 goal={name}，可用: {available}")

    goal = goals[name]
    if isinstance(goal, list):
        return [_waypoint_from_goal_config(item) for item in goal]
    if isinstance(goal, dict):
        return [_waypoint_from_goal_config(goal)]
    raise ValueError(f"goal={name} 格式错误")


def _resolve_waypoints(args: argparse.Namespace) -> list[tuple[float, float, float, float, float, float, float]]:
    if args.waypoints:
        return _parse_waypoints_literal(args.waypoints)
    if args.waypoint:
        return [_parse_waypoint_item(item) for item in args.waypoint]
    if args.goal:
        return _load_named_goal(args.config, args.goal)
    raise ValueError("必须提供 --goal、--waypoint 或 --waypoints")


def _add_ros_python_path() -> None:
    ros_path = "/opt/ros/noetic/lib/python3/dist-packages"
    if ros_path not in sys.path:
        sys.path.insert(0, ros_path)


def _build_goal(args: argparse.Namespace, waypoints: list[tuple[float, float, float, float, float, float, float]]):
    from navigation.msg import NavigationGoal, Waypoint

    speed = float(args.speed_cm_per_s)
    safe_dist = float(args.safe_dist_cm)
    if not (0.0 < speed < 100.0):
        raise ValueError(f"--speed-cm-per-s 超出范围 (0,100): {speed}")
    if not (0.0 < safe_dist < 100.0):
        raise ValueError(f"--safe-dist-cm 超出范围 (0,100): {safe_dist}")

    goal = NavigationGoal()
    goal.header.stamp.secs = int(speed)
    goal.header.stamp.nsecs = int(safe_dist)
    goal.header.frame_id = args.frame_id
    goal.task_type.value = int(args.task_type)
    goal.translation.enable = False
    goal.translation.heading = 0.0

    for x, y, z, qx, qy, qz, qw in waypoints:
        wp = Waypoint()
        wp.pose.position.x = x
        wp.pose.position.y = y
        wp.pose.position.z = z
        wp.pose.orientation.x = qx
        wp.pose.orientation.y = qy
        wp.pose.orientation.z = qz
        wp.pose.orientation.w = qw
        wp.distance_tolerance = float(args.distance_tolerance)
        wp.heading_tolerance = float(args.heading_tolerance)
        goal.waypoints.append(wp)
    return goal


def _build_goal_dict(
    args: argparse.Namespace,
    waypoints: list[tuple[float, float, float, float, float, float, float]],
) -> dict:
    speed = float(args.speed_cm_per_s)
    safe_dist = float(args.safe_dist_cm)
    if not (0.0 < speed < 100.0):
        raise ValueError(f"--speed-cm-per-s 超出范围 (0,100): {speed}")
    if not (0.0 < safe_dist < 100.0):
        raise ValueError(f"--safe-dist-cm 超出范围 (0,100): {safe_dist}")

    return {
        "header": {
            "seq": 0,
            "stamp": {"secs": int(speed), "nsecs": int(safe_dist)},
            "frame_id": args.frame_id,
        },
        "task_type": {"value": int(args.task_type)},
        "translation": {"enable": False, "heading": 0.0},
        "waypoints": [
            {
                "pose": {
                    "position": {"x": x, "y": y, "z": z},
                    "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
                },
                "distance_tolerance": float(args.distance_tolerance),
                "heading_tolerance": float(args.heading_tolerance),
            }
            for x, y, z, qx, qy, qz, qw in waypoints
        ],
    }


def _print_waypoints(waypoints: list[tuple[float, float, float, float, float, float, float]]) -> None:
    for i, (x, y, z, qx, qy, qz, qw) in enumerate(waypoints, start=1):
        print(
            f"  waypoint {i}: pos=({x:.3f}, {y:.3f}, {z:.3f}) "
            f"q=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})",
            flush=True,
        )


def _check_odom(odom_topic: str, timeout: float) -> bool:
    import rospy
    from nav_msgs.msg import Odometry

    print(f"[动作] 检查 odom: {odom_topic}", flush=True)
    try:
        msg = rospy.wait_for_message(odom_topic, Odometry, timeout=timeout)
    except Exception as exc:
        print(f"[✗] {timeout:.1f}s 内没有收到 odom，不发送导航目标: {exc}", flush=True)
        return False

    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    yaw_deg = _quaternion_to_yaw_deg(ori.x, ori.y, ori.z, ori.w)
    print(
        f"[✓] odom 正常: frame={msg.header.frame_id or '<empty>'}, "
        f"child={msg.child_frame_id or '<empty>'}, "
        f"x={pos.x:.3f}, y={pos.y:.3f}, yaw={yaw_deg:.1f}deg",
        flush=True,
    )
    return True


def _parse_ws_url(ws_url: str) -> tuple[str, int]:
    stripped = ws_url.replace("ws://", "").replace("wss://", "")
    host, port = stripped.split(":", 1)
    return host, int(port)


def _connect_rosbridge(ws_url: str):
    try:
        import roslibpy
    except ImportError as exc:
        raise RuntimeError("当前环境没有 actionlib，也没有 roslibpy，不能发送导航目标") from exc

    host, port = _parse_ws_url(ws_url)
    ros = roslibpy.Ros(host=host, port=port)
    ros.run()
    if not ros.is_connected:
        raise RuntimeError(f"连接 rosbridge 失败: {ws_url}")
    return ros, roslibpy


def _close_rosbridge(ros) -> None:
    try:
        if ros and ros.is_connected:
            ros.terminate()
    except Exception:
        pass


def _check_odom_rosbridge(ros, roslibpy, odom_topic: str, timeout: float) -> bool:
    print(f"[动作] 检查 odom: {odom_topic}", flush=True)
    event = threading.Event()
    holder: dict = {}
    topic = roslibpy.Topic(ros, odom_topic, "nav_msgs/Odometry")

    def _callback(message):
        holder["message"] = message
        event.set()

    topic.subscribe(_callback)
    ok = event.wait(timeout)
    try:
        topic.unsubscribe()
    except Exception:
        pass

    if not ok:
        print(f"[✗] {timeout:.1f}s 内没有收到 odom，不发送导航目标", flush=True)
        return False

    msg = holder["message"]
    pose = msg.get("pose", {}).get("pose", {})
    pos = pose.get("position", {})
    ori = pose.get("orientation", {})
    yaw_deg = _quaternion_to_yaw_deg(
        float(ori.get("x", 0.0)),
        float(ori.get("y", 0.0)),
        float(ori.get("z", 0.0)),
        float(ori.get("w", 1.0)),
    )
    print(
        f"[✓] odom 正常: frame={msg.get('header', {}).get('frame_id') or '<empty>'}, "
        f"child={msg.get('child_frame_id') or '<empty>'}, "
        f"x={float(pos.get('x', 0.0)):.3f}, y={float(pos.get('y', 0.0)):.3f}, yaw={yaw_deg:.1f}deg",
        flush=True,
    )
    return True


def _result_state_value(result: dict) -> int | None:
    payload = result.get("result", result) if isinstance(result, dict) else {}
    state = payload.get("state") if isinstance(payload, dict) else None
    if isinstance(state, dict):
        value = state.get("value")
    else:
        value = state
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _result_causes(result: dict) -> list:
    payload = result.get("result", result) if isinstance(result, dict) else {}
    causes = payload.get("causes", []) if isinstance(payload, dict) else []
    return causes if isinstance(causes, list) else []


def _ros_time_now() -> dict:
    now = time.time()
    secs = int(now)
    return {"secs": secs, "nsecs": int((now - secs) * 1_000_000_000)}


def _action_topic(action_name: str, suffix: str) -> str:
    return action_name.rstrip("/") + "/" + suffix


def _build_action_goal(goal_id: str, args: argparse.Namespace, waypoints: list[tuple[float, float, float, float, float, float, float]]) -> dict:
    stamp = _ros_time_now()
    return {
        "header": {"seq": 0, "stamp": stamp, "frame_id": ""},
        "goal_id": {"stamp": stamp, "id": goal_id},
        "goal": _build_goal_dict(args, waypoints),
    }


def _action_result_goal_id(message: dict) -> str:
    return str(message.get("status", {}).get("goal_id", {}).get("id", ""))


def _action_status_value(message: dict) -> int | None:
    try:
        return int(message.get("status", {}).get("status"))
    except (TypeError, ValueError):
        return None


def _print_navigation_result(message: dict) -> int:
    action_status = _action_status_value(message)
    nav_state = _result_state_value(message)
    print(f"[导航] action_status={action_status}, nav_state={nav_state}", flush=True)

    if nav_state == SUCCESS_STATE or action_status == 3:
        print("[✓] 导航成功", flush=True)
        return 0

    text = message.get("status", {}).get("text", "")
    if text:
        print(f"[导航] status_text={text}", flush=True)
    for i, cause in enumerate(_result_causes(message), start=1):
        if isinstance(cause, dict):
            print(f"[导航] cause {i}: code={cause.get('code', -1)} msg={cause.get('msg', '')}", flush=True)
    print("[✗] 导航失败", flush=True)
    return 4


def _run_rosbridge_action(args: argparse.Namespace, waypoints: list[tuple[float, float, float, float, float, float, float]]) -> int:
    ros, roslibpy = _connect_rosbridge(args.ws_url)
    try:
        if not args.skip_odom_check and not _check_odom_rosbridge(ros, roslibpy, args.odom_topic, args.odom_timeout_sec):
            return 5

        goal_topic_name = _action_topic(args.action_name, "goal")
        result_topic_name = _action_topic(args.action_name, "result")
        cancel_topic_name = _action_topic(args.action_name, "cancel")
        goal_id = f"humanoid_navigation_{os.getpid()}_{int(time.time() * 1000)}"
        result_holder: dict = {}
        result_event = threading.Event()

        result_topic = roslibpy.Topic(ros, result_topic_name, "navigation/NavigationActionResult")
        goal_topic = roslibpy.Topic(ros, goal_topic_name, "navigation/NavigationActionGoal")
        cancel_topic = roslibpy.Topic(ros, cancel_topic_name, "actionlib_msgs/GoalID")

        def _result_callback(message):
            if _action_result_goal_id(message) and _action_result_goal_id(message) != goal_id:
                return
            result_holder["message"] = message
            result_event.set()

        result_topic.subscribe(_result_callback)
        goal_topic.advertise()
        cancel_topic.advertise()

        print(f"[动作] 发送导航目标 action={args.action_name}", flush=True)
        goal_topic.publish(roslibpy.Message(_build_action_goal(goal_id, args, waypoints)))

        print("[动作] 等待导航结果", flush=True)
        if not result_event.wait(float(args.result_timeout_sec)):
            print("[✗] 导航结果超时，取消目标", flush=True)
            cancel_topic.publish(roslibpy.Message({"stamp": _ros_time_now(), "id": goal_id}))
            return 3

        return _print_navigation_result(result_holder.get("message", {}))
    finally:
        for topic_name in ("result_topic", "goal_topic", "cancel_topic"):
            topic = locals().get(topic_name)
            if not topic:
                continue
            try:
                topic.unsubscribe()
            except Exception:
                pass
            try:
                topic.unadvertise()
            except Exception:
                pass
        _close_rosbridge(ros)


def main() -> int:
    parser = argparse.ArgumentParser(description="Send basic navigation waypoints.")
    parser.add_argument("--action-name", default=DEFAULT_ACTION_NAME)
    parser.add_argument("--ws-url", default=DEFAULT_WS_URL, help="rosbridge URL used when local actionlib is unavailable")
    parser.add_argument("--config", default=os.path.join(PROJECT_ROOT, "configs", "navigation.yaml"))
    parser.add_argument("--goal", help="Named goal in configs/navigation.yaml")
    parser.add_argument(
        "--waypoint",
        nargs=7,
        action="append",
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        help="Add a waypoint. Can be repeated.",
    )
    parser.add_argument(
        "--waypoints",
        help='Python list of waypoints, e.g. "[(0.17,1.35,0,0,0,0,1)]"',
    )
    parser.add_argument("--speed-cm-per-s", type=float, default=30.0)
    parser.add_argument("--safe-dist-cm", type=float, default=10.0)
    parser.add_argument("--distance-tolerance", type=float, default=0.10)
    parser.add_argument("--heading-tolerance", type=float, default=0.10)
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--task-type", type=int, default=0)
    parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    parser.add_argument("--odom-timeout-sec", type=float, default=3.0)
    parser.add_argument("--skip-odom-check", action="store_true")
    parser.add_argument("--wait-server-timeout-sec", type=float, default=10.0)
    parser.add_argument("--result-timeout-sec", type=float, default=300.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    waypoints = _resolve_waypoints(args)

    print("=" * 70)
    print("  Navigation flow")
    print("=" * 70)
    print(f"  Action: {args.action_name}")
    print(f"  Goal: {args.goal or '<direct waypoints>'}")
    print(f"  Speed: {args.speed_cm_per_s:.1f} cm/s")
    print(f"  Safe distance: {args.safe_dist_cm:.1f} cm")
    print(f"  Odom: {args.odom_topic} timeout={args.odom_timeout_sec:.1f}s")
    print(f"  Dry run: {args.dry_run}")
    _print_waypoints(waypoints)
    print("=" * 70)

    if args.dry_run:
        return 0

    _add_ros_python_path()
    try:
        import actionlib
        import rospy
        from navigation.msg import NavigationAction
    except ModuleNotFoundError as exc:
        missing = exc.name or str(exc)
        print(f"[!] 本地 ROS actionlib 不可用: {missing}，切换 rosbridge: {args.ws_url}", flush=True)
        return _run_rosbridge_action(args, waypoints)

    rospy.init_node("humanoid_basic_navigation_flow", anonymous=True)
    client = actionlib.SimpleActionClient(args.action_name, NavigationAction)

    if not args.skip_odom_check and not _check_odom(args.odom_topic, args.odom_timeout_sec):
        return 5

    print("[动作] 等待导航 action server", flush=True)
    if not client.wait_for_server(rospy.Duration.from_sec(args.wait_server_timeout_sec)):
        print(f"[✗] action server 超时: {args.action_name}", flush=True)
        return 2

    goal = _build_goal(args, waypoints)
    print(f"[动作] 发送导航目标 waypoints={len(goal.waypoints)}", flush=True)
    client.send_goal(goal)

    print("[动作] 等待导航结果", flush=True)
    if not client.wait_for_result(rospy.Duration.from_sec(args.result_timeout_sec)):
        print("[✗] 导航结果超时，取消目标", flush=True)
        client.cancel_goal()
        return 3

    result = client.get_result()
    state_value = getattr(getattr(result, "state", None), "value", None)
    causes = getattr(result, "causes", [])
    print(f"[导航] state={state_value}", flush=True)

    if state_value == SUCCESS_STATE:
        print("[✓] 导航成功", flush=True)
        return 0

    if causes:
        for i, cause in enumerate(causes, start=1):
            code = getattr(cause, "code", -1)
            msg = getattr(cause, "msg", "")
            print(f"[导航] cause {i}: code={code} msg={msg}", flush=True)
    print("[✗] 导航失败", flush=True)
    return 4


if __name__ == "__main__":
    raise SystemExit(main())
