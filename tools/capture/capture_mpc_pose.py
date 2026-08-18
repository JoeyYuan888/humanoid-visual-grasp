#!/usr/bin/env python3
"""
Capture current MPC end-effector pose.

This is for pure MPC hand/palm pose workflow:
1. Put the hand at the desired waypoint position and palm orientation.
2. Read /DualArmMobile/currentEEPose/FrameR or FrameL.
3. Save the full pose for later --via-file or --orientation-file use.
"""

from __future__ import annotations

import argparse
import json
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
    print("缺少 roslibpy 库，请运行: pip install roslibpy")
    sys.exit(1)


FRAME_TOPICS = {
    "left": "/DualArmMobile/currentEEPose/FrameL",
    "right": "/DualArmMobile/currentEEPose/FrameR",
}
CURRENT_STATE_TOPIC = "/DualArmMobile/currenState"
DEFAULT_OUTPUT = os.path.join(PROJECT_ROOT, "data", "poses", "mpc_pose_right_latest.json")


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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws-url", default=config.WS_URL, help="rosbridge WebSocket URL")
    parser.add_argument("--arm", choices=["right", "left"], default="right")
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--include-joints", action="store_true",
                        help="Also capture /DualArmMobile/currenState stateTrajectory[0].value.")
    args = parser.parse_args()

    topic = FRAME_TOPICS[args.arm]
    print("=" * 70)
    print("  Capture MPC end-effector pose")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  Arm: {args.arm}")
    print(f"  Topic: {topic}")
    if args.include_joints:
        print(f"  Joint Topic: {CURRENT_STATE_TOPIC}")
    print(f"  Output: {args.output}")
    print("=" * 70)

    client = _connect(args.ws_url)
    print("[✓] 已连接 rosbridge")
    try:
        message = _wait_for_pose(client, topic, args.timeout)
        if message is None:
            print(f"[✗] {args.timeout:.1f}s 内没有收到 {topic}")
            raise SystemExit(1)

        pose = _pose_stamped_to_pose(message)
        payload = {
            "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "arm": args.arm,
            "topic": topic,
            "pose": pose,
            "orientation": pose["orientation"],
            "note": "geometry_msgs quaternion order: x y z w",
        }
        if args.include_joints:
            state_msg = _wait_for_current_state(client, args.timeout)
            if state_msg is None:
                print(f"[✗] {args.timeout:.1f}s 内没有收到 {CURRENT_STATE_TOPIC}")
                raise SystemExit(1)
            mpc_state = _extract_current_mpc_state(state_msg)
            if not mpc_state:
                print("[✗] currenState 中没有 stateTrajectory[0].value")
                raise SystemExit(1)
            payload["mpc_state"] = mpc_state
            payload["joint_num"] = len(mpc_state)
            payload["joint_topic"] = CURRENT_STATE_TOPIC

        os.makedirs(os.path.dirname(args.output), exist_ok=True)
        with open(args.output, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)

        print("\n当前 MPC 末端 pose:")
        print(json.dumps(pose, indent=2, ensure_ascii=False))
        print("\n可直接用于命令的 orientation:")
        o = pose["orientation"]
        print(f"--orientation {o['x']} {o['y']} {o['z']} {o['w']}")
        print(f"可直接作为完整约束点使用: --via-file {args.output}")
        if args.include_joints:
            print(f"已同时保存 MPC joints: joint_num={len(payload['mpc_state'])}")
        print(f"\n[✓] 已保存: {args.output}")
    finally:
        try:
            client.terminate()
        except Exception:
            pass


if __name__ == "__main__":
    main()
