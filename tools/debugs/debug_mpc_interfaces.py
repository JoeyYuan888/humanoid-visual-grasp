"""
检查 MPC 接口是否在线，并采样关键话题结构。

这个脚本只查询服务/话题和短时间订阅状态，不调用 mpc_mode_setting，
不发送 points_seq/joints_seq，不会让机器人动作。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from collections import defaultdict

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot_grasp import config

try:
    import roslibpy
except ImportError:
    print("缺少 roslibpy 库，请运行: pip install roslibpy")
    sys.exit(1)


MPC_SERVICE_SUFFIXES = [
    "points_seq_tracking",
    "joints_seq_tracking",
    "points_seq_tracking_with_joints",
    "points_seq_tracking_with_admittance",
    "points_seq_collaborative_tracking",
    "hybrid_points_seq_tracking",
]

MPC_MODE_SERVICE_CANDIDATES = [
    "/wa/wa_hardware_interface/mpc_mode_setting",
    "/wa1/wa_hardware_interface/mpc_mode_setting",
    "/wa2/wa_hardware_interface/mpc_mode_setting",
]

MPC_TOPIC_CANDIDATES = [
    "/DualArmMobile/currenState",
    "/DualArmMobile/currentEEPose/FrameL",
    "/DualArmMobile/currentEEPose/FrameR",
    "/wrist_force_control/left_arm_compensated_force",
    "/wrist_force_control/right_arm_compensated_force",
]


def _connect():
    host = config.WS_URL.replace("ws://", "").split(":")[0]
    port = int(config.WS_URL.replace("ws://", "").split(":")[1])
    client = roslibpy.Ros(host=host, port=port)
    thread = threading.Thread(target=client.run, daemon=True)
    thread.start()

    start = time.time()
    while not client.is_connected:
        if time.time() - start > config.CONNECT_TIMEOUT:
            print(f"[✗] 连接超时: {config.WS_URL}")
            sys.exit(1)
        time.sleep(0.1)
    return client


def _call(client, name, service_type, request=None):
    service = roslibpy.Service(client, name, service_type)
    return service.call(roslibpy.ServiceRequest(request or {}))


def _get_services(client):
    try:
        return sorted(_call(client, "/rosapi/services", "rosapi/Services").get("services", []))
    except Exception as exc:
        print(f"[!] 无法通过 /rosapi/services 查询服务: {exc}")
        return []


def _get_topics(client):
    try:
        response = _call(client, "/rosapi/topics", "rosapi/Topics")
        topics = response.get("topics", [])
        types = response.get("types", [])
        return dict(zip(topics, types))
    except Exception as exc:
        print(f"[!] 无法通过 /rosapi/topics 查询话题: {exc}")
        return {}


def _service_type(client, name):
    try:
        return _call(client, "/rosapi/service_type", "rosapi/ServiceType", {"service": name}).get("type", "")
    except Exception:
        return ""


def _topic_type(client, name):
    try:
        return _call(client, "/rosapi/topic_type", "rosapi/TopicType", {"topic": name}).get("type", "")
    except Exception:
        return ""


def _prefix_from_service(name: str, suffix: str) -> str:
    if not name.endswith("/" + suffix):
        return ""
    return name[: -len("/" + suffix)]


def _compact_value(value, depth=0):
    if depth > 3:
        return "..."
    if isinstance(value, dict):
        return {k: _compact_value(v, depth + 1) for k, v in list(value.items())[:8]}
    if isinstance(value, list):
        if len(value) > 8:
            return {
                "len": len(value),
                "head": [_compact_value(v, depth + 1) for v in value[:3]],
                "tail": [_compact_value(v, depth + 1) for v in value[-2:]],
            }
        return [_compact_value(v, depth + 1) for v in value]
    return value


def _sample_topics(client, topics: dict[str, str], sample_seconds: float):
    samples = {}
    subscribers = []
    lock = threading.Lock()

    def make_callback(topic):
        def callback(message):
            with lock:
                if topic not in samples:
                    samples[topic] = message
        return callback

    for topic, topic_type in topics.items():
        if not topic_type:
            continue
        sub = roslibpy.Topic(client, topic, topic_type)
        sub.subscribe(make_callback(topic))
        subscribers.append(sub)

    deadline = time.time() + sample_seconds
    while time.time() < deadline:
        with lock:
            if len(samples) >= len([t for t in topics.values() if t]):
                break
        time.sleep(0.1)

    for sub in subscribers:
        try:
            sub.unsubscribe()
        except Exception:
            pass
    return samples


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sample-seconds", type=float, default=3.0, help="关键话题采样秒数")
    args = parser.parse_args()

    print("=" * 70)
    print("  MPC 接口检查（只查询，不发运动命令）")
    print("=" * 70)
    print(f"  WebSocket: {config.WS_URL}")
    print()

    client = _connect()
    print("[✓] 已连接 rosbridge")

    services = _get_services(client)
    topics = _get_topics(client)

    mpc_services = [
        name for name in services
        if any(name.endswith("/" + suffix) for suffix in MPC_SERVICE_SUFFIXES)
        or name in MPC_MODE_SERVICE_CANDIDATES
        or "mpc_mode_setting" in name
        or "admittance_mode_setting" in name
    ]

    prefixes = defaultdict(list)
    for name in mpc_services:
        for suffix in MPC_SERVICE_SUFFIXES:
            prefix = _prefix_from_service(name, suffix)
            if prefix:
                prefixes[prefix].append(suffix)

    print("\nMPC 服务:")
    if not mpc_services:
        print("  [✗] 没找到 MPC 相关服务")
    for name in mpc_services:
        print(f"  [✓] {name} {_service_type(client, name)}")

    print("\n候选 MPC 前缀:")
    if not prefixes:
        print("  [✗] 未能从 points_seq/joints_seq 服务推断前缀")
    for prefix, suffixes in sorted(prefixes.items()):
        print(f"  {prefix}: {', '.join(sorted(suffixes))}")

    print("\nMPC 关键话题:")
    candidate_topics = {}
    for name in MPC_TOPIC_CANDIDATES:
        if name in topics:
            topic_type = topics[name] or _topic_type(client, name)
            candidate_topics[name] = topic_type
            print(f"  [✓] {name} {topic_type}")
        else:
            print(f"  [✗] {name}")

    print(f"\n采样关键话题 {args.sample_seconds:.1f}s:")
    samples = _sample_topics(client, candidate_topics, args.sample_seconds)
    for topic, topic_type in candidate_topics.items():
        message = samples.get(topic)
        if message is None:
            print(f"  [!] {topic}: 未收到样本")
            continue
        print(f"  [✓] {topic}:")
        if topic.endswith("/currenState"):
            traj = message.get("stateTrajectory", [])
            if traj and isinstance(traj, list):
                value = traj[0].get("value", [])
                print(f"      stateTrajectory[0].value len = {len(value)}")
                print(f"      head = {value[:8]}")
                print(f"      tail = {value[-5:] if len(value) >= 5 else value}")
            else:
                print("      未发现 stateTrajectory[0].value")
        elif "currentEEPose" in topic:
            poses = message.get("poses", [])
            if poses:
                print(f"      poses len = {len(poses)}")
                print(f"      pose0 = {json.dumps(_compact_value(poses[0]), ensure_ascii=False)}")
            else:
                print(f"      msg = {json.dumps(_compact_value(message), ensure_ascii=False)}")
        else:
            print(f"      msg = {json.dumps(_compact_value(message), ensure_ascii=False)}")

    print("\n判断:")
    required_suffixes = {"points_seq_tracking", "joints_seq_tracking"}
    ok_prefixes = [
        prefix for prefix, suffixes in prefixes.items()
        if required_suffixes.issubset(set(suffixes))
    ]
    if ok_prefixes:
        print(f"  [✓] 至少一个前缀同时具备 points_seq_tracking 和 joints_seq_tracking: {ok_prefixes}")
    else:
        print("  [!] 还没确认可用的 MPC 前缀，先不要写死 /wa、/wa1 或 /wa2")

    current_state_sample = samples.get("/DualArmMobile/currenState")
    if current_state_sample:
        traj = current_state_sample.get("stateTrajectory", [])
        value = traj[0].get("value", []) if traj else []
        if len(value) == 23:
            print("  [✓] /DualArmMobile/currenState 当前是 23 维，符合 WA2 文档")
        elif value:
            print(f"  [!] /DualArmMobile/currenState 当前是 {len(value)} 维，需要按实机为准")
        else:
            print("  [!] currenState 有话题但没有解析到 value")
    else:
        print("  [!] 没采到 /DualArmMobile/currenState，无法确认 joint_num")
        print("      建议在机器人终端直接确认:")
        print("      rostopic echo -n 1 /DualArmMobile/currenState")
        print("      如果直接 rostopic 也没有输出，可能需要先启动/开启 MPC 控制节点。")

    try:
        client.terminate()
    except Exception:
        pass


if __name__ == "__main__":
    main()
