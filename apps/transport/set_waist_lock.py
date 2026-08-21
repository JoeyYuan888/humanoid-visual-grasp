#!/usr/bin/env python3
"""Set /wa/waist_lock_setting from current upperlimb joint_states."""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot_grasp.common import config

try:
    import roslibpy
except ImportError:
    roslibpy = None


WAIST_LOCK_SERVICE = "/wa/waist_lock_setting"
JOINT_STATES_TOPIC = "/zj_humanoid/upperlimb/joint_states"
JOINT_STATES_TYPE = "sensor_msgs/JointState"
DEFAULT_LOCK_INDEX = "0,1,2,3"


def _parse_ws_url(ws_url: str) -> tuple[str, int]:
    stripped = ws_url.replace("ws://", "").replace("wss://", "")
    host, port = stripped.split(":")
    return host, int(port)


def _connect(ws_url: str):
    if roslibpy is None:
        raise RuntimeError("缺少 roslibpy")
    host, port = _parse_ws_url(ws_url)
    client = roslibpy.Ros(host=host, port=port)
    thread = threading.Thread(target=client.run, daemon=True)
    thread.start()
    start = time.time()
    while not client.is_connected:
        if time.time() - start > config.CONNECT_TIMEOUT:
            raise RuntimeError(f"连接超时: {ws_url}")
        time.sleep(0.1)
    return client


def _call(client, name: str, service_type: str, request: dict | None = None) -> dict:
    service = roslibpy.Service(client, name, service_type)
    return service.call(roslibpy.ServiceRequest(request or {}))


def _service_type(client, name: str) -> str:
    try:
        return _call(client, "/rosapi/service_type", "rosapi/ServiceType", {"service": name}).get("type", "")
    except Exception:
        return ""


def _wait_joint_states(client, timeout: float) -> tuple[list[str], list[float]]:
    latest: dict[str, object] = {}

    def callback(message: dict) -> None:
        latest["name"] = message.get("name") or []
        latest["position"] = message.get("position") or []

    topic = roslibpy.Topic(client, JOINT_STATES_TOPIC, JOINT_STATES_TYPE)
    topic.subscribe(callback)
    start = time.time()
    while time.time() - start < timeout and "position" not in latest:
        time.sleep(0.05)
    try:
        topic.unsubscribe()
    except Exception:
        pass
    names = [str(value) for value in latest.get("name", [])]
    positions = [float(value) for value in latest.get("position", [])]
    if not positions:
        raise TimeoutError(f"{timeout:.1f}s 内没有收到 {JOINT_STATES_TOPIC}")
    return names, positions


def _parse_lock_index(value: str) -> list[int]:
    return [int(item.strip()) for item in value.split(",") if item.strip()]


def _validate_lock_index(lock_index: list[int]) -> None:
    invalid = [index for index in lock_index if index < 0 or index > 3]
    if invalid:
        raise ValueError(
            f"lock_index={invalid} 超出 /wa/waist_lock_setting 支持范围 [0, 3]。"
            "这里的编号是厂商接口内部锁定位，不是 22 维 joint_states 下标。"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Set WA waist lock from current upperlimb joint_states.")
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument(
        "--lock-index",
        default=DEFAULT_LOCK_INDEX,
        help="Comma-separated internal lock slots. Vendor service accepts 0-3, not joint_states indices.",
    )
    parser.add_argument(
        "--neck-track",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="正常锁定时应为 true；--disable 时脚本会强制发送 false。",
    )
    parser.add_argument("--disable", action="store_true", help="Clear lock_index while sending current joint_state.")
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()

    lock_index = [] if args.disable else _parse_lock_index(args.lock_index)
    neck_track = False if args.disable else bool(args.neck_track)
    _validate_lock_index(lock_index)
    print("=" * 70)
    print("  WA waist lock setting")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  lock_index: {lock_index}")
    print(f"  neck_track: {neck_track}")
    print("=" * 70)

    client = _connect(args.ws_url)
    try:
        names, positions = _wait_joint_states(client, args.timeout)
        if len(positions) != 22:
            raise RuntimeError(f"{JOINT_STATES_TOPIC} position 长度={len(positions)}，期望 22")
        print(f"[动作] 读取 joint_states 完成 len={len(positions)}")
        if lock_index:
            print("    lock slots:", lock_index, "(厂商接口内部 0-3 编号)")
        srv_type = _service_type(client, WAIST_LOCK_SERVICE)
        if not srv_type:
            raise RuntimeError(f"无法获取服务类型: {WAIST_LOCK_SERVICE}")
        response = _call(
            client,
            WAIST_LOCK_SERVICE,
            srv_type,
            {
                "joint_state": positions,
                "lock_index": lock_index,
                "neck_track": neck_track,
            },
        )
        print(f"[waist_lock] {response}")
    finally:
        try:
            client.terminate()
        except Exception:
            pass


if __name__ == "__main__":
    main()
