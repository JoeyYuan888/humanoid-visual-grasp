#!/usr/bin/env python3
"""Replay captured dual-arm MPC pose waypoints.

This is intentionally small: it reads current left/right MPC poses, appends one
or more captured dual-arm pose files, and calls the MPC tracking service.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import sys
import time
from typing import Any

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from apps.grasp.visual_grasp_test_impl import (
    CURRENT_STATE_TOPIC,
    FRAME_TOPICS,
    POINTS_SERVICE,
    POINTS_WITH_JOINTS_SERVICE,
    _call,
    _connect,
    _extract_current_mpc_state,
    _pose_array,
    _pose_stamped_to_pose,
    _service_type,
    _wait_for_current_state,
    _wait_for_pose,
)


def _current_pose(client, arm: str, timeout: float) -> dict:
    msg = _wait_for_pose(client, FRAME_TOPICS[arm], timeout)
    if msg is None:
        raise TimeoutError(f"{timeout:.1f}s 内没有收到 {FRAME_TOPICS[arm]}")
    return _pose_stamped_to_pose(msg)


def _current_state(client, timeout: float) -> list[float]:
    msg = _wait_for_current_state(client, timeout)
    if msg is None:
        raise TimeoutError(f"{timeout:.1f}s 内没有收到 {CURRENT_STATE_TOPIC}")
    state = _extract_current_mpc_state(msg)
    if not state:
        raise RuntimeError(f"{CURRENT_STATE_TOPIC} 没有 stateTrajectory")
    return state


def _resolve_path(path: str) -> str:
    if os.path.isabs(path):
        return path
    candidates = [
        os.path.join(PROJECT_ROOT, path),
        os.path.join(PROJECT_ROOT, "data", "poses", "transport", path),
        os.path.join(PROJECT_ROOT, "data", "poses", path),
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return os.path.join(PROJECT_ROOT, path)


def _load_dual_pose(path: str) -> dict:
    resolved = _resolve_path(path)
    with open(resolved, "r", encoding="utf-8") as f:
        data = json.load(f)
    if data.get("type") != "dual_arm_mpc_pose":
        raise ValueError(f"不是 dual_arm_mpc_pose 文件: {path}")
    for arm in ("left", "right"):
        if arm not in data or "pose" not in data[arm]:
            raise ValueError(f"{path} 缺少 {arm}.pose")
    data["_path"] = resolved
    return data


def _distance(a: dict, b: dict) -> float:
    pa = a["position"]
    pb = b["position"]
    return math.sqrt(
        (pa["x"] - pb["x"]) ** 2
        + (pa["y"] - pb["y"]) ** 2
        + (pa["z"] - pb["z"]) ** 2
    )


def _path_length(poses: list[dict]) -> float:
    return sum(_distance(a, b) for a, b in zip(poses, poses[1:]))


def _build_request(left_poses: list[dict], right_poses: list[dict], duration: float, way_type: str) -> dict:
    return {
        "left_poses": _pose_array(left_poses),
        "right_poses": _pose_array(right_poses),
        "time_points": [duration for _ in left_poses],
        "max_period": duration * len(left_poses) + 2.0,
        "weight": 1.0,
        "type": way_type,
    }


def _build_request_with_joints(
    left_poses: list[dict],
    right_poses: list[dict],
    states: list[list[float]],
    duration: float,
    way_type: str,
) -> dict:
    joint_num = len(states[0]) if states else 0
    return {
        "left_poses": _pose_array(left_poses),
        "right_poses": _pose_array(right_poses),
        "time_points": [duration for _ in left_poses],
        "states": [float(value) for state in states for value in state],
        "joint_num": joint_num,
        "max_period": duration * len(left_poses) + 2.0,
        "weight": 1.0,
        "type": way_type,
    }


def _deep_pose(pose: dict) -> dict:
    return copy.deepcopy(pose)


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay captured dual-arm MPC pose path.")
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument("--target", action="append", required=True, help="dual_arm_mpc_pose JSON. Can be repeated.")
    parser.add_argument("--duration", type=float, default=5.0, help="MPC duration per waypoint.")
    parser.add_argument("--execute-delay", type=float, default=2.0)
    parser.add_argument("--max-motion", type=float, default=2.0, help="Max cumulative path length per arm.")
    parser.add_argument("--type", default="quintic", choices=["quintic", "cubic"])
    parser.add_argument("--use-joints", action="store_true", help="Use saved mpc_state from target files.")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    targets = [_load_dual_pose(path) for path in args.target]
    print("=" * 70)
    print("  Dual-arm MPC pose path")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  Targets: {', '.join(os.path.relpath(t['_path'], PROJECT_ROOT) for t in targets)}")
    print(f"  Execute: {args.execute}")
    print("=" * 70)

    client = _connect(args.ws_url)
    try:
        print("[动作] 读取当前双臂 MPC pose 开始")
        left_poses = [_current_pose(client, "left", 5.0)]
        right_poses = [_current_pose(client, "right", 5.0)]
        states: list[list[float]] = []
        if args.use_joints:
            print("[动作] 读取当前 MPC joint state 开始")
            states.append(_current_state(client, 5.0))

        for target in targets:
            left_poses.append(_deep_pose(target["left"]["pose"]))
            right_poses.append(_deep_pose(target["right"]["pose"]))
            if args.use_joints:
                state = target.get("mpc_state")
                if not state:
                    raise RuntimeError(f"{target['_path']} 没有 mpc_state，不能 --use-joints")
                states.append([float(v) for v in state])
        print(f"[动作] 生成双臂路径完成 points={len(left_poses)}")

        left_len = _path_length(left_poses)
        right_len = _path_length(right_poses)
        print(f"左臂路径累计长度: {left_len:.3f} m")
        print(f"右臂路径累计长度: {right_len:.3f} m")
        if max(left_len, right_len) > args.max_motion:
            raise RuntimeError(
                f"路径累计长度 {max(left_len, right_len):.3f}m > --max-motion {args.max_motion:.3f}m"
            )

        if args.use_joints:
            service_name = POINTS_WITH_JOINTS_SERVICE
            request = _build_request_with_joints(left_poses, right_poses, states, args.duration, args.type)
        else:
            service_name = POINTS_SERVICE
            request = _build_request(left_poses, right_poses, args.duration, args.type)

        print(f"[动作] 准备发送 MPC 轨迹 service={service_name}")
        if not args.execute:
            print("[DRY RUN] 未发送运动命令。加 --execute 才会执行。")
            print(json.dumps(request, indent=2, ensure_ascii=False))
            return

        if args.execute_delay > 0:
            print(f"[EXECUTE] {args.execute_delay:.1f}s 后发送，Ctrl+C 可取消")
            time.sleep(args.execute_delay)

        srv_type = _service_type(client, service_name)
        if not srv_type:
            raise RuntimeError(f"无法获取服务类型: {service_name}")
        response = _call(client, service_name, srv_type, request)
        print(f"[MPC] response: {response}")
    finally:
        try:
            client.terminate()
        except Exception:
            pass


if __name__ == "__main__":
    main()
