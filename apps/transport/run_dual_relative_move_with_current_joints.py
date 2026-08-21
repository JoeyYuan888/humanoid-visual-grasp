#!/usr/bin/env python3
"""Move both TCPs by a relative BASE-frame offset while referencing current joints."""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import sys
import time


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from apps.grasp.visual_grasp_test_impl import (
    CURRENT_STATE_TOPIC,
    FRAME_TOPICS,
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
    return [float(value) for value in state]


def _offset_pose(pose: dict, dx: float, dy: float, dz: float) -> dict:
    target = copy.deepcopy(pose)
    target["position"]["x"] = float(target["position"]["x"]) + dx
    target["position"]["y"] = float(target["position"]["y"]) + dy
    target["position"]["z"] = float(target["position"]["z"]) + dz
    return target


def _distance(a: dict, b: dict) -> float:
    pa = a["position"]
    pb = b["position"]
    return math.sqrt(
        (pa["x"] - pb["x"]) ** 2
        + (pa["y"] - pb["y"]) ** 2
        + (pa["z"] - pb["z"]) ** 2
    )


BODY_JOINT_NAMES = {
    3: "Pitch_Y_B",
    4: "Pitch_Y_M",
    5: "Waist_Z",
    6: "Waist_Y",
}


def _parse_body_joint_values(value: str) -> dict[int, float]:
    result: dict[int, float] = {}
    if not value.strip():
        return result
    for item in value.split(","):
        item = item.strip()
        if not item:
            continue
        if "=" not in item:
            raise ValueError(f"--body-joint-values 格式错误: {item}，应为 index=value")
        index_text, value_text = item.split("=", 1)
        index = int(index_text.strip())
        if index not in BODY_JOINT_NAMES:
            raise ValueError(f"只允许设置身体关节 3-6，收到 index={index}")
        result[index] = float(value_text.strip())
    return result


def _apply_body_joint_values(state: list[float], values: dict[int, float]) -> list[float]:
    result = [float(value) for value in state]
    for index, value in values.items():
        result[index] = float(value)
    return result


def _build_request(
    left_poses: list[dict],
    right_poses: list[dict],
    states: list[list[float]],
    duration: float,
    weight: float,
) -> dict:
    joint_num = len(states[0]) if states else 0
    return {
        "left_poses": _pose_array(left_poses),
        "right_poses": _pose_array(right_poses),
        "time_points": [duration for _ in left_poses],
        "states": [float(value) for state in states for value in state],
        "joint_num": joint_num,
        "max_period": duration * len(left_poses) + 2.0,
        "weight": weight,
        "type": "quintic",
    }


def _save_target(path: str, left_pose: dict, right_pose: dict, state: list[float], dx: float, dy: float, dz: float) -> None:
    payload = {
        "name": "transport_relative_move_current_joints",
        "type": "dual_arm_mpc_pose",
        "frame": "BASE",
        "left": {"arm": "left", "pose": left_pose, "orientation": left_pose["orientation"]},
        "right": {"arm": "right", "pose": right_pose, "orientation": right_pose["orientation"]},
        "mpc_state": state,
        "joint_num": len(state),
        "params": {"dx": dx, "dy": dy, "dz": dz},
        "note": "Relative move target generated from current TCPs and current MPC state.",
    }
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
        f.write("\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Relative dual-arm move with current joints as reference.")
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument("--dx", type=float, default=0.0, help="BASE x offset in meters. Negative moves toward robot body.")
    parser.add_argument("--dy", type=float, default=0.0)
    parser.add_argument("--dz", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--weight", type=float, default=1.0)
    parser.add_argument(
        "--body-joint-values",
        default="",
        help="Optional WA2 body joint targets for indices 3-6, e.g. '3=0,4=0,5=0.008,6=0'.",
    )
    parser.add_argument("--execute-delay", type=float, default=2.0)
    parser.add_argument("--max-motion", type=float, default=0.30)
    parser.add_argument(
        "--save-target",
        default=os.path.join(PROJECT_ROOT, "data", "runtime", "transport_relative_move_current_joints_latest.json"),
    )
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    print("=" * 70)
    print("  Transport relative dual-arm move with current joints")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  Offset: dx={args.dx:.3f}, dy={args.dy:.3f}, dz={args.dz:.3f}")
    body_joint_values = _parse_body_joint_values(args.body_joint_values)
    print(f"  Body joint values: {body_joint_values if body_joint_values else '{}'}")
    print(f"  Weight: {args.weight:.1f}")
    print(f"  Execute: {args.execute}")
    print("=" * 70)

    client = _connect(args.ws_url)
    try:
        print("[动作] 读取当前双臂 MPC pose 开始")
        left_current = _current_pose(client, "left", 5.0)
        right_current = _current_pose(client, "right", 5.0)
        print("[动作] 读取当前 MPC joint state 开始")
        current_state = _current_state(client, 5.0)

        left_target = _offset_pose(left_current, args.dx, args.dy, args.dz)
        right_target = _offset_pose(right_current, args.dx, args.dy, args.dz)
        left_len = _distance(left_current, left_target)
        right_len = _distance(right_current, right_target)
        print(f"左臂相对移动: {left_len:.3f} m")
        print(f"右臂相对移动: {right_len:.3f} m")
        if max(left_len, right_len) > args.max_motion:
            raise RuntimeError(f"相对移动 {max(left_len, right_len):.3f}m > --max-motion {args.max_motion:.3f}m")

        _save_target(args.save_target, left_target, right_target, current_state, args.dx, args.dy, args.dz)
        print(f"[✓] 已保存目标: {os.path.relpath(args.save_target, PROJECT_ROOT)}")
        target_state = _apply_body_joint_values(current_state, body_joint_values) if body_joint_values else current_state
        if body_joint_values:
            print("[动作] 身体关节参考")
            for index in sorted(body_joint_values):
                print(
                    f"  {index} {BODY_JOINT_NAMES[index]}: "
                    f"{current_state[index]:+.4f} -> {target_state[index]:+.4f}"
                )

        request = _build_request(
            [left_current, left_target],
            [right_current, right_target],
            [current_state, target_state],
            args.duration,
            args.weight,
        )
        print(f"[动作] 准备发送 MPC 轨迹 service={POINTS_WITH_JOINTS_SERVICE}")
        if not args.execute:
            print("[DRY RUN] 未发送运动命令。加 --execute 才会执行。")
            print(json.dumps(request, indent=2, ensure_ascii=False))
            return

        if args.execute_delay > 0:
            print(f"[EXECUTE] {args.execute_delay:.1f}s 后发送，Ctrl+C 可取消")
            time.sleep(args.execute_delay)

        srv_type = _service_type(client, POINTS_WITH_JOINTS_SERVICE)
        if not srv_type:
            raise RuntimeError(f"无法获取服务类型: {POINTS_WITH_JOINTS_SERVICE}")
        response = _call(client, POINTS_WITH_JOINTS_SERVICE, srv_type, request)
        print(f"[MPC] response: {response}")
    finally:
        try:
            client.terminate()
        except Exception:
            pass


if __name__ == "__main__":
    main()
