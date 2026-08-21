#!/usr/bin/env python3
"""Tune WA2 body joints 3-6 through /wa/joints_seq_tracking.

Use only when the transported box is supported or otherwise safe. This command
does not constrain the end-effector TCP poses.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from apps.grasp.visual_grasp_test_impl import (
    CURRENT_STATE_TOPIC,
    _call,
    _connect,
    _extract_current_mpc_state,
    _service_type,
    _wait_for_current_state,
)


JOINTS_SERVICE = "/wa/joints_seq_tracking"
DEFAULT_OUTPUT = os.path.join(PROJECT_ROOT, "data", "runtime", "transport_body_joint_tune_latest.json")
BODY_JOINT_NAMES = {
    3: "Pitch_Y_B",
    4: "Pitch_Y_M",
    5: "Waist_Z",
    6: "Waist_Y",
}


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
        index = int(index_text.strip())
        if index not in BODY_JOINT_NAMES:
            raise ValueError(f"只允许调整身体关节 3-6，收到 index={index}")
        result[index] = float(value_text.strip())
    return result


def _target_state(current: list[float], values: dict[int, float], max_delta: float) -> list[float]:
    target = [float(value) for value in current]
    for index, value in values.items():
        delta = value - current[index]
        if abs(delta) > max_delta:
            raise RuntimeError(
                f"{index} {BODY_JOINT_NAMES[index]} 变化 {delta:+.4f}rad 超过 --max-delta {max_delta:.4f}rad"
            )
        target[index] = value
    return target


def _print_body_state(prefix: str, state: list[float]) -> None:
    print(prefix)
    for index in sorted(BODY_JOINT_NAMES):
        print(f"    {index} {BODY_JOINT_NAMES[index]}={state[index]:+.4f}")


def _build_request(current: list[float], target: list[float], duration: float, weight: float) -> dict:
    return {
        "states": [float(value) for state in (current, target) for value in state],
        "joint_num": len(current),
        "time_points": [duration, duration],
        "max_period": duration * 2.0 + 2.0,
        "weight": weight,
    }


def _save(path: str, payload: dict) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
        f.write("\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Tune WA2 body joints 3-6 with /wa/joints_seq_tracking.")
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument(
        "--joint-values",
        required=True,
        help="Explicit body joint targets, e.g. '3=-0.10,4=-0.10,5=-0.02,6=-0.22'.",
    )
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument(
        "--weight",
        type=float,
        default=1.0,
        help="Joints tracking weight. Keep 1.0 unless explicitly tuning waist response.",
    )
    parser.add_argument("--max-delta", type=float, default=0.08, help="Max per-command joint delta in radians.")
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--execute-delay", type=float, default=2.0)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    values = _parse_joint_values(args.joint_values)
    if not values:
        raise ValueError("--joint-values 不能为空")

    print("=" * 70)
    print("  Transport body joint tune")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  Joint targets: {values}")
    print(f"  Weight: {args.weight:.1f}")
    print(f"  Execute: {args.execute}")
    print("=" * 70)

    client = _connect(args.ws_url)
    try:
        msg = _wait_for_current_state(client, 5.0)
        if msg is None:
            raise TimeoutError(f"5.0s 内没有收到 {CURRENT_STATE_TOPIC}")
        current = _extract_current_mpc_state(msg)
        if not current:
            raise RuntimeError(f"{CURRENT_STATE_TOPIC} 没有 stateTrajectory")
        _print_body_state("[动作] 当前身体关节 3-6", current)
        target = _target_state(current, values, args.max_delta)
        request = _build_request(current, target, args.duration, args.weight)
        print("[动作] 当前 -> 目标")
        for index in sorted(values):
            print(
                f"    {index} {BODY_JOINT_NAMES[index]}: "
                f"{current[index]:+.4f} -> {target[index]:+.4f} "
                f"(delta={target[index] - current[index]:+.4f})"
            )
        _save(
            args.output,
            {
                "type": "transport_body_joint_tune",
                "joint_values": values,
                "current_mpc_state": current,
                "target_mpc_state": target,
                "request": request,
            },
        )
        print(f"[✓] 已保存请求: {os.path.relpath(args.output, PROJECT_ROOT)}")

        if not args.execute:
            print("[DRY RUN] 未发送运动命令。加 --execute 才会执行。")
            return

        if args.execute_delay > 0:
            print(f"[EXECUTE] {args.execute_delay:.1f}s 后发送，Ctrl+C 可取消")
            time.sleep(args.execute_delay)
        srv_type = _service_type(client, JOINTS_SERVICE)
        if not srv_type:
            raise RuntimeError(f"无法获取服务类型: {JOINTS_SERVICE}")
        print(f"[动作] 发送 joints 轨迹开始 service={JOINTS_SERVICE}")
        response = _call(client, JOINTS_SERVICE, srv_type, request)
        print(f"[MPC] response: {response}")
        print("[动作] 发送 joints 轨迹完成")
    finally:
        try:
            client.terminate()
        except Exception:
            pass


if __name__ == "__main__":
    main()
