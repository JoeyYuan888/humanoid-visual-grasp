"""
检查 SDK/手掌接口是否在线。

这个脚本只查询服务和话题，不会调用运动服务，不会让机器人动作。
"""

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
    print("缺少 roslibpy 库，请运行: pip install roslibpy")
    sys.exit(1)


REQUIRED_SERVICES = [
    "/wa/wa_hardware_interface/mpc_mode_setting",
    "/zj_humanoid/upperlimb/teach_mode/enter",
    "/zj_humanoid/upperlimb/teach_mode/exit",
    "/zj_humanoid/upperlimb/movej_by_path/right_arm",
    "/zj_humanoid/upperlimb/movel/right_arm",
    "/zj_humanoid/upperlimb/movej/neck",
    "/zj_humanoid/upperlimb/movej_by_path/neck",
    "/zj_humanoid/upperlimb/go_home/neck",
    "/zj_humanoid/upperlimb/stop",
    "/zj_humanoid/hand/joint_switch/right",
    "/zj_humanoid/hand/finger_pressures/right/zero",
]

REQUIRED_TOPICS = [
    "/zj_humanoid/upperlimb/joint_states",
    "/zj_humanoid/upperlimb/tcp_pose/right_arm",
    "/zj_humanoid/hand/finger_pressures/right",
]

OPTIONAL_SERVICES = [
    # 新手册提到腕部六维力标零，但当前实机 rosservice list 未发现。
    "/zj_humanoid/hand/wrist_force_sensor/right/zero",
]

OPTIONAL_TOPICS = [
    # 新手册提到腕部六维力话题，但当前实机 rostopic list 未发现。
    "/zj_humanoid/hand/wrist_force_sensor/right",
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
        return set(_call(client, "/rosapi/services", "rosapi/Services").get("services", []))
    except Exception as exc:
        print(f"[!] 无法通过 /rosapi/services 查询服务: {exc}")
        return set()


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


def main():
    print("=" * 60)
    print("  SDK/手掌接口安全检查（不发运动命令）")
    print("=" * 60)
    print(f"  WebSocket: {config.WS_URL}")
    print("  前置条件: SDK 调用前需要 /wa/wa_hardware_interface/mpc_mode_setting 可用")
    print()

    client = _connect()
    print("[✓] 已连接 rosbridge")

    services = _get_services(client)
    topics = _get_topics(client)

    print("\n服务:")
    missing_services = []
    for name in REQUIRED_SERVICES:
        ok = name in services
        if not ok:
            missing_services.append(name)
        srv_type = _service_type(client, name) if ok else ""
        print(f"  {'[✓]' if ok else '[✗]'} {name} {srv_type}")

    print("\n可选服务:")
    for name in OPTIONAL_SERVICES:
        ok = name in services
        srv_type = _service_type(client, name) if ok else ""
        print(f"  {'[✓]' if ok else '[-]'} {name} {srv_type}")

    print("\n话题:")
    missing_topics = []
    for name in REQUIRED_TOPICS:
        ok = name in topics
        if not ok:
            missing_topics.append(name)
        print(f"  {'[✓]' if ok else '[✗]'} {name} {topics.get(name, '')}")

    print("\n可选话题:")
    for name in OPTIONAL_TOPICS:
        ok = name in topics
        print(f"  {'[✓]' if ok else '[-]'} {name} {topics.get(name, '')}")

    print("\n结果:")
    if not missing_services and not missing_topics:
        print("  [✓] SDK/手掌接口都在线，可以进入下一步手掌空跑测试")
        print("  [i] 手动调用任何 SDK 服务前，先执行:")
        print('      rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: false"')
    else:
        print("  [!] 有接口没找到，先不要发运动命令")
        if missing_services:
            print("  缺失服务:")
            for name in missing_services:
                print(f"    - {name}")
        if missing_topics:
            print("  缺失话题:")
            for name in missing_topics:
                print(f"    - {name}")

    try:
        client.terminate()
    except Exception:
        pass


if __name__ == "__main__":
    main()
