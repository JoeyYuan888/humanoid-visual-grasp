#!/usr/bin/env python3
"""Move a clamped box to a standard carry pose with collaborative tracking."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from apps.grasp.visual_grasp_test_impl import (  # noqa: E402
    FRAME_TOPICS,
    _call,
    _connect,
    _pose_array,
    _pose_stamped_to_pose,
    _service_type,
    _wait_for_pose,
)


COLLABORATIVE_SERVICE = "/wa/points_seq_collaborative_tracking"
MPC_MODE_SERVICE = "/wa/wa_hardware_interface/mpc_mode_setting"


def _current_pose(client, arm: str, timeout: float) -> dict:
    msg = _wait_for_pose(client, FRAME_TOPICS[arm], timeout)
    if msg is None:
        raise TimeoutError(f"{timeout:.1f}s 内没有收到 {FRAME_TOPICS[arm]}")
    return _pose_stamped_to_pose(msg)


def _center(left_pose: dict, right_pose: dict) -> dict:
    lp = left_pose["position"]
    rp = right_pose["position"]
    return {
        "x": (float(lp["x"]) + float(rp["x"])) * 0.5,
        "y": (float(lp["y"]) + float(rp["y"])) * 0.5,
        "z": (float(lp["z"]) + float(rp["z"])) * 0.5,
    }


def _offset_pose(pose: dict, dx: float, dy: float, dz: float) -> dict:
    return {
        "position": {
            "x": float(pose["position"]["x"]) + dx,
            "y": float(pose["position"]["y"]) + dy,
            "z": float(pose["position"]["z"]) + dz,
        },
        "orientation": dict(pose["orientation"]),
    }


def _distance_xyz(dx: float, dy: float, dz: float) -> float:
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _set_mpc_mode(client) -> None:
    srv_type = _service_type(client, MPC_MODE_SERVICE)
    if not srv_type:
        raise RuntimeError(f"无法获取服务类型: {MPC_MODE_SERVICE}")
    response = _call(client, MPC_MODE_SERVICE, srv_type, {"data": True})
    print(f"[mpc_mode=True] {response}", flush=True)
    if response and response.get("success") is False:
        raise RuntimeError(f"MPC mode 设置失败: {response}")


def _build_request(major_pose: dict, duration: float, major_arm: str, way_type: str) -> dict:
    return {
        "major_arm_poses": _pose_array([major_pose]),
        "time_points": [duration],
        "max_period": duration + 2.0,
        "track_weight": 1.0,
        "collaborate_weight": 1.0,
        "way_type": way_type,
        "major_arm": major_arm,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Collaborative carry-pose test for clamped transport box.")
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument("--major-arm", choices=["left", "right"], default="right")
    parser.add_argument("--target-center-x", type=float, default=0.35)
    parser.add_argument("--target-center-y", type=float, default=None)
    parser.add_argument("--target-center-z", type=float, default=0.85)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--execute-delay", type=float, default=2.0)
    parser.add_argument("--max-shift", type=float, default=0.60)
    parser.add_argument("--type", default="spline", choices=["spline", "quintic"])
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    print("=" * 70)
    print("  Transport collaborative carry pose")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  Major arm: {args.major_arm}")
    print(f"  Target center x={args.target_center_x:.3f}, z={args.target_center_z:.3f}")
    print(f"  Execute: {args.execute}")
    print("=" * 70)

    client = _connect(args.ws_url)
    try:
        print("[动作] 读取当前左右手 MPC pose 开始", flush=True)
        left_pose = _current_pose(client, "left", 5.0)
        right_pose = _current_pose(client, "right", 5.0)
        center = _center(left_pose, right_pose)
        target_y = center["y"] if args.target_center_y is None else args.target_center_y
        dx = args.target_center_x - center["x"]
        dy = target_y - center["y"]
        dz = args.target_center_z - center["z"]
        shift = _distance_xyz(dx, dy, dz)
        print(
            f"当前双手中心: x={center['x']:.4f}, y={center['y']:.4f}, z={center['z']:.4f}",
            flush=True,
        )
        print(f"目标位移: dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}, length={shift:.4f}", flush=True)
        if shift > args.max_shift:
            raise RuntimeError(f"协同搬运位移 {shift:.3f}m > --max-shift {args.max_shift:.3f}m")

        source_pose = left_pose if args.major_arm == "left" else right_pose
        major_pose = _offset_pose(source_pose, dx, dy, dz)
        request = _build_request(major_pose, args.duration, args.major_arm, args.type)
        print(
            f"主臂目标: x={major_pose['position']['x']:.4f}, "
            f"y={major_pose['position']['y']:.4f}, z={major_pose['position']['z']:.4f}",
            flush=True,
        )

        if not args.execute:
            print("[DRY RUN] 未发送协同运动命令。加 --execute 才会执行。")
            print(json.dumps(request, indent=2, ensure_ascii=False))
            return

        _set_mpc_mode(client)
        if args.execute_delay > 0:
            print(f"[EXECUTE] {args.execute_delay:.1f}s 后发送，Ctrl+C 可取消", flush=True)
            time.sleep(args.execute_delay)
        srv_type = _service_type(client, COLLABORATIVE_SERVICE)
        if not srv_type:
            raise RuntimeError(f"无法获取服务类型: {COLLABORATIVE_SERVICE}")
        response = _call(client, COLLABORATIVE_SERVICE, srv_type, request)
        print(f"[MPC] response: {response}", flush=True)
    finally:
        try:
            client.terminate()
        except Exception:
            pass


if __name__ == "__main__":
    main()
