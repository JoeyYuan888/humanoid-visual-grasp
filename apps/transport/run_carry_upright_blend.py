#!/usr/bin/env python3
"""Blend current MPC state toward an upright reference while holding both TCPs.

This is an experimental carry-posture tuning tool. It keeps the current left
and right MPC end-effector poses as the Cartesian targets, and only changes the
joint reference sent through /wa/points_seq_tracking_with_joints.
"""

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


DEFAULT_REFERENCE = os.path.join(PROJECT_ROOT, "data", "poses", "transport", "transport_home_dual.json")
DEFAULT_OUTPUT = os.path.join(PROJECT_ROOT, "data", "runtime", "transport_carry_upright_blend_latest.json")


def _rel(path: str) -> str:
    try:
        return os.path.relpath(path, PROJECT_ROOT)
    except ValueError:
        return path


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


def _load_reference_state(path: str) -> list[float]:
    resolved = _resolve_path(path)
    with open(resolved, "r", encoding="utf-8") as f:
        data = json.load(f)
    state = data.get("mpc_state")
    if not state:
        raise RuntimeError(f"{path} 没有 mpc_state，不能作为 joints reference")
    return [float(value) for value in state]


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


def _pose_with_lift(pose: dict, lift: float) -> dict:
    result = copy.deepcopy(pose)
    result["position"]["z"] = float(result["position"]["z"]) + lift
    return result


def _blend_state(current: list[float], reference: list[float], blend: float, indices: list[int] | None) -> list[float]:
    if len(current) != len(reference):
        raise RuntimeError(f"mpc_state 长度不一致: current={len(current)} reference={len(reference)}")
    result = [float(value) for value in current]
    selected = set(indices if indices is not None else range(len(current)))
    for index in selected:
        if index < 0 or index >= len(current):
            raise RuntimeError(f"blend index out of range: {index}, joint_num={len(current)}")
        result[index] = current[index] + blend * (reference[index] - current[index])
    return result


def _parse_indices(value: str) -> list[int] | None:
    if not value.strip():
        return None
    indices: list[int] = []
    for item in value.split(","):
        item = item.strip()
        if not item:
            continue
        if ":" in item:
            start, end = item.split(":", 1)
            indices.extend(range(int(start), int(end)))
        else:
            indices.append(int(item))
    return sorted(set(indices))


def _parse_joint_values(value: str) -> dict[int, float]:
    result: dict[int, float] = {}
    if not value.strip():
        return result
    for item in value.split(","):
        item = item.strip()
        if not item:
            continue
        if "=" not in item:
            raise ValueError(f"--joint-values 格式错误: {item}，应为 index=value")
        index_text, value_text = item.split("=", 1)
        result[int(index_text.strip())] = float(value_text.strip())
    return result


def _apply_joint_values(state: list[float], values: dict[int, float]) -> list[float]:
    result = [float(value) for value in state]
    for index, value in values.items():
        if index < 0 or index >= len(result):
            raise RuntimeError(f"joint value index out of range: {index}, joint_num={len(result)}")
        result[index] = float(value)
    return result


def _distance(a: dict, b: dict) -> float:
    pa = a["position"]
    pb = b["position"]
    return math.sqrt(
        (pa["x"] - pb["x"]) ** 2
        + (pa["y"] - pb["y"]) ** 2
        + (pa["z"] - pb["z"]) ** 2
    )


def _build_request(left_pose: dict, right_pose: dict, current_state: list[float], target_state: list[float],
                   duration: float, way_type: str, weight: float) -> dict:
    left_poses = [left_pose, left_pose]
    right_poses = [right_pose, right_pose]
    states = [current_state, target_state]
    return {
        "left_poses": _pose_array(left_poses),
        "right_poses": _pose_array(right_poses),
        "time_points": [duration, duration],
        "states": [float(value) for state in states for value in state],
        "joint_num": len(current_state),
        "max_period": duration * 2.0 + 2.0,
        "weight": weight,
        "type": way_type,
    }


def _save_output(path: str, payload: dict) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
        f.write("\n")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Hold current dual TCP poses while blending MPC joints toward an upright reference."
    )
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument("--reference", default=DEFAULT_REFERENCE, help="dual_arm_mpc_pose JSON with mpc_state.")
    parser.add_argument(
        "--blend",
        type=float,
        default=0.20,
        help="Blend toward reference joints. Positive moves toward reference; small negative values can test the opposite direction.",
    )
    parser.add_argument("--indices", default="", help="Optional indices/ranges to blend, e.g. '0:3,19:23'. Empty=all.")
    parser.add_argument(
        "--joint-values",
        default="",
        help="Explicit joint overrides, e.g. '3=-0.08,4=-0.08,5=-0.02,6=-0.18'. Overrides --blend for listed joints.",
    )
    parser.add_argument("--lift", type=float, default=0.0, help="Optional shared TCP z lift while blending.")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--weight", type=float, default=1.0)
    parser.add_argument("--type", default="quintic", choices=["quintic", "cubic"])
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--execute-delay", type=float, default=2.0)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    if not -1.0 <= args.blend <= 1.0:
        raise ValueError("--blend 必须在 [-1, 1] 内")

    reference_path = _resolve_path(args.reference)
    reference_state = _load_reference_state(reference_path)
    indices = _parse_indices(args.indices)
    joint_values = _parse_joint_values(args.joint_values)

    print("=" * 70)
    print("  Transport carry upright blend")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  Reference: {_rel(reference_path)}")
    print(f"  Blend: {args.blend:.3f}")
    print(f"  Indices: {indices if indices is not None else 'all'}")
    print(f"  Joint values: {joint_values if joint_values else '{}'}")
    print(f"  TCP lift: {args.lift:.3f} m")
    print(f"  Execute: {args.execute}")
    print("=" * 70)

    client = _connect(args.ws_url)
    try:
        print("[动作] 读取当前双手 TCP 和 MPC state 开始")
        left_current = _current_pose(client, "left", 5.0)
        right_current = _current_pose(client, "right", 5.0)
        current_state = _current_state(client, 5.0)
        print("[动作] 读取当前双手 TCP 和 MPC state 完成")

        left_target = _pose_with_lift(left_current, args.lift)
        right_target = _pose_with_lift(right_current, args.lift)
        target_state = _blend_state(current_state, reference_state, args.blend, indices)
        if joint_values:
            target_state = _apply_joint_values(target_state, joint_values)
        tcp_motion = max(_distance(left_current, left_target), _distance(right_current, right_target))
        joint_delta = max(abs(a - b) for a, b in zip(current_state, target_state))

        print(f"当前双手间距: {_distance(left_current, right_current):.3f} m")
        print(f"TCP 最大位移: {tcp_motion:.3f} m")
        print(f"joints 最大变化: {joint_delta:.4f}")

        request = _build_request(
            left_target,
            right_target,
            current_state,
            target_state,
            args.duration,
            args.type,
            args.weight,
        )
        _save_output(
            args.output,
            {
                "type": "transport_carry_upright_blend",
                "reference": _rel(reference_path),
                "blend": args.blend,
                "indices": indices,
                "joint_values": joint_values,
                "lift": args.lift,
                "left_pose": left_target,
                "right_pose": right_target,
                "current_mpc_state": current_state,
                "target_mpc_state": target_state,
                "request": request,
            },
        )
        print(f"[✓] 已保存请求: {_rel(args.output)}")

        if not args.execute:
            print("[DRY RUN] 未发送运动命令。加 --execute 才会执行。")
            return

        if args.execute_delay > 0:
            print(f"[EXECUTE] {args.execute_delay:.1f}s 后发送，Ctrl+C 可取消")
            time.sleep(args.execute_delay)

        srv_type = _service_type(client, POINTS_WITH_JOINTS_SERVICE)
        if not srv_type:
            raise RuntimeError(f"无法获取服务类型: {POINTS_WITH_JOINTS_SERVICE}")
        print(f"[动作] 发送 MPC 轨迹开始 service={POINTS_WITH_JOINTS_SERVICE}")
        response = _call(client, POINTS_WITH_JOINTS_SERVICE, srv_type, request)
        print(f"[MPC] response: {response}")
        print("[动作] 发送 MPC 轨迹完成")
    finally:
        try:
            client.terminate()
        except Exception:
            pass


if __name__ == "__main__":
    main()
