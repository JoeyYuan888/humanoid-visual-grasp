"""
Inspect hand-eye related TF frames and ROS params.

This is read-only: it subscribes to /tf_static and /tf briefly, and calls rosapi
param/topic services. It does not switch modes or send motion commands.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot_grasp import config

try:
    import roslibpy
except ImportError:
    print("缺少 roslibpy 库，请在 footpose 环境运行，或先安装: pip install roslibpy")
    sys.exit(1)


PARAM_PREFIXES = [
    "/jzhw/calib/camera/up",
    "/jzhw/calib/camera/down",
    "/jzhw/camera/info/up/base/calib_param",
    "/jzhw/camera/info/down/base/calib_param",
    "/j1/jzhw/model/camera/0",
    "/j1/jzhw/model/camera/1",
    "/j3/jzhw/model/camera/0",
    "/j3/jzhw/model/camera/1",
]

PARAM_KEYS = [
    "name",
    "topic",
    "type",
    "position",
    "px",
    "py",
    "pz",
    "roll",
    "pitch",
    "yaw",
    "px_base2cam",
    "py_base2cam",
    "pz_base2cam",
    "roll_base2cam",
    "pitch_base2cam",
    "yaw_base2cam",
]

FRAME_KEYWORDS = (
    "realsense",
    "head",
    "neck",
    "base",
    "body",
    "torso",
    "waist",
    "camera",
    "cam",
)

PATH_QUERIES = [
    ("BASE", "HEAD"),
    ("root", "HEAD"),
    ("BASE", "realsense_head_link"),
    ("HEAD", "realsense_head_link"),
    ("BASE", "realsense_head_color_optical_frame"),
    ("HEAD", "realsense_head_color_optical_frame"),
]


def _connect(ws_url: str):
    host = ws_url.replace("ws://", "").split(":")[0]
    port = int(ws_url.replace("ws://", "").split(":")[1])
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


def _call(client, name, service_type, request=None):
    service = roslibpy.Service(client, name, service_type)
    return service.call(roslibpy.ServiceRequest(request or {}))


def _get_param(client, name):
    try:
        response = _call(
            client,
            "/rosapi/get_param",
            "rosapi/GetParam",
            {"name": name, "default": ""},
        )
        value = response.get("value", "")
        if value == "":
            return None
        try:
            return json.loads(value)
        except Exception:
            return value
    except Exception:
        return None


def _get_topics(client):
    try:
        response = _call(client, "/rosapi/topics", "rosapi/Topics")
        topics = response.get("topics", [])
        types = response.get("types", [])
        return dict(zip(topics, types))
    except Exception:
        return {}


def _sample_tf_topic(client, topic_name, sample_seconds):
    transforms = {}
    done = threading.Event()

    def callback(message):
        for transform in message.get("transforms", []):
            item = _format_transform(transform)
            key = (item["parent"], item["child"])
            transforms[key] = item
        done.set()

    topic = roslibpy.Topic(client, topic_name, "tf2_msgs/TFMessage")
    topic.subscribe(callback)
    end = time.time() + sample_seconds
    while time.time() < end:
        done.wait(0.2)
    try:
        topic.unsubscribe()
    except Exception:
        pass
    return transforms


def _format_transform(transform):
    header = transform.get("header", {})
    parent = header.get("frame_id", "")
    child = transform.get("child_frame_id", "")
    translation = transform.get("transform", {}).get("translation", {})
    rotation = transform.get("transform", {}).get("rotation", {})
    return {
        "parent": parent,
        "child": child,
        "translation": translation,
        "rotation": rotation,
    }


def _is_relevant_frame(parent, child):
    text = f"{parent} {child}".lower()
    return any(keyword in text for keyword in FRAME_KEYWORDS)


def _print_transform(item, indent="  "):
    print(f"{indent}{item['parent']} -> {item['child']}")
    print(f"{indent}  t={item['translation']}")
    print(f"{indent}  q={item['rotation']}")


def _build_graph(transforms):
    graph = {}
    for item in transforms.values():
        parent = item["parent"].lstrip("/")
        child = item["child"].lstrip("/")
        graph.setdefault(parent, []).append((child, "forward", item))
        graph.setdefault(child, []).append((parent, "inverse", item))
    return graph


def _find_path(graph, start, goal):
    start = start.lstrip("/")
    goal = goal.lstrip("/")
    queue = [(start, [])]
    visited = {start}
    while queue:
        node, path = queue.pop(0)
        if node == goal:
            return path
        for nxt, direction, item in graph.get(node, []):
            if nxt in visited:
                continue
            visited.add(nxt)
            queue.append((nxt, path + [(direction, item)]))
    return None


def _print_path(name, path):
    if path is None:
        print(f"  [✗] {name}: 未找到路径")
        return
    print(f"  [✓] {name}: {len(path)} 段")
    for direction, item in path:
        arrow = "->" if direction == "forward" else "<-"
        print(f"      {item['parent']} {arrow} {item['child']}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws-url", default=config.WS_URL)
    parser.add_argument("--sample-seconds", type=float, default=2.0)
    args = parser.parse_args()

    print("=" * 70)
    print("  手眼链路来源检查（只读）")
    print("=" * 70)
    print(f"WebSocket: {args.ws_url}")

    client = _connect(args.ws_url)
    print("[✓] 已连接 rosbridge")

    print("\n/tf_static 采样:")
    static_transforms = _sample_tf_topic(client, "/tf_static", args.sample_seconds)
    if not static_transforms:
        print("  [!] 未收到 /tf_static")
    for item in static_transforms.values():
        if _is_relevant_frame(item["parent"], item["child"]):
            _print_transform(item)

    print("\n/tf 动态采样:")
    dynamic_transforms = _sample_tf_topic(client, "/tf", args.sample_seconds)
    if not dynamic_transforms:
        print("  [!] 未收到 /tf")
    for item in dynamic_transforms.values():
        if _is_relevant_frame(item["parent"], item["child"]):
            _print_transform(item)

    print("\n关键 tf 路径检查:")
    all_transforms = {}
    all_transforms.update(static_transforms)
    all_transforms.update(dynamic_transforms)
    graph = _build_graph(all_transforms)
    for start, goal in PATH_QUERIES:
        _print_path(f"{start} -> {goal}", _find_path(graph, start, goal))

    print("\n相机/RealSense 相关话题:")
    topics = _get_topics(client)
    for topic, topic_type in sorted(topics.items()):
        text = topic.lower()
        if "realsense_head" in text or "camera_info" in text or "cam_" in text:
            print(f"  {topic} {topic_type}")

    print("\n候选相机标定参数:")
    found_any = False
    for prefix in PARAM_PREFIXES:
        values = {}
        for key in PARAM_KEYS:
            name = f"{prefix}/{key}"
            value = _get_param(client, name)
            if value is not None:
                values[key] = value
        if not values:
            continue
        found_any = True
        print(f"\n{prefix}")
        topic_name = values.get("topic", "")
        if topic_name and "realsense_head" not in str(topic_name):
            print("  note: 该参数 topic 不是 realsense_head，不能直接当头部 RealSense 手眼外参")
        for key in PARAM_KEYS:
            if key in values:
                print(f"  {key}: {values[key]}")
    if not found_any:
        print("  [!] 没读取到候选参数值，请确认 rosapi/get_param 是否可用")

    client.terminate()


if __name__ == "__main__":
    main()
