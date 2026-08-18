#!/usr/bin/env python3
"""Move the left hand above the shelf-box pull point derived from AprilTag.

This is a placement-stage probing tool. It keeps the right hand at its current
MPC pose and sends only the left hand to a target derived from the locked
AprilTag BASE pose.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from apps.grasp.visual_grasp_test_impl import (  # noqa: E402
    FRAME_TOPICS,
    POINTS_SERVICE,
    _call,
    _connect,
    _pose_array,
    _pose_stamped_to_pose,
    _service_type,
    _wait_for_pose,
)
from apps.place.run_apriltag_lock import MPC_MODE_SERVICE  # noqa: E402


DEFAULT_LOCKED_TAG = PROJECT_ROOT / "data" / "runtime" / "place_apriltag_target_latest.json"


def _load_locked_tag(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    base_pose = data.get("base", {}).get("tag_pose")
    if not base_pose or "position" not in base_pose:
        raise ValueError(f"{path} 没有 base.tag_pose.position，请先跑 run_apriltag_lock.py")
    return data


def _current_pose(client, arm: str, timeout: float) -> dict:
    msg = _wait_for_pose(client, FRAME_TOPICS[arm], timeout)
    if msg is None:
        raise TimeoutError(f"{timeout:.1f}s 内没有收到 {FRAME_TOPICS[arm]}")
    return _pose_stamped_to_pose(msg)


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


def _build_left_target(
    locked_tag: dict[str, Any],
    current_left: dict,
    *,
    offset_x: float,
    offset_y: float,
    z_offset: float,
    above_height: float,
) -> dict:
    tag_position = locked_tag["base"]["tag_pose"]["position"]
    target = copy.deepcopy(current_left)
    target["position"] = {
        "x": float(tag_position["x"]) + float(offset_x),
        "y": float(tag_position["y"]) + float(offset_y),
        "z": float(tag_position["z"]) + float(z_offset) + float(above_height),
    }
    return target


def _build_request(left_poses: list[dict], right_poses: list[dict], duration: float, way_type: str) -> dict:
    return {
        "left_poses": _pose_array(left_poses),
        "right_poses": _pose_array(right_poses),
        "time_points": [duration for _ in left_poses],
        "max_period": duration * len(left_poses) + 2.0,
        "weight": 1.0,
        "type": way_type,
    }


def _set_mpc_mode(client, enabled: bool) -> None:
    srv_type = _service_type(client, MPC_MODE_SERVICE)
    if not srv_type:
        raise RuntimeError(f"无法获取服务类型: {MPC_MODE_SERVICE}")
    response = _call(client, MPC_MODE_SERVICE, srv_type, {"data": bool(enabled)})
    print(f"[mpc_mode={enabled}] {response}")
    if response and response.get("success") is False:
        raise RuntimeError(f"MPC mode 设置失败: {response}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Move left hand above AprilTag-derived pull point.")
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument("--locked-tag", type=Path, default=DEFAULT_LOCKED_TAG)
    parser.add_argument("--offset-x", type=float, default=0.0, help="BASE x offset from tag. Negative moves toward body.")
    parser.add_argument("--offset-y", type=float, default=0.0, help="BASE y offset from tag.")
    parser.add_argument("--z-offset", type=float, default=0.05, help="Transport-consistent initial z offset.")
    parser.add_argument("--above-height", type=float, default=0.10, help="Approach height above the pull point.")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--execute-delay", type=float, default=2.0)
    parser.add_argument("--max-motion", type=float, default=1.2)
    parser.add_argument("--type", default="quintic", choices=["quintic", "cubic"])
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    locked_tag = _load_locked_tag(args.locked_tag)
    tag = locked_tag.get("tag", {})
    tag_position = locked_tag["base"]["tag_pose"]["position"]

    print("=" * 70)
    print("  Place left-hand pull approach")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  Locked tag: {args.locked_tag}")
    print(f"  Tag: id={tag.get('selected_id')} level={tag.get('shelf_level')}")
    print(
        "  Base tag: "
        f"x={tag_position['x']:.4f}, y={tag_position['y']:.4f}, z={tag_position['z']:.4f}"
    )
    print(f"  Offset: x={args.offset_x:.3f}, y={args.offset_y:.3f}, z={args.z_offset:.3f}")
    print(f"  Above height: {args.above_height:.3f} m")
    print(f"  Execute: {args.execute}")
    print("=" * 70)

    client = _connect(args.ws_url)
    try:
        print("[动作] 设置 MPC running mode 开始")
        _set_mpc_mode(client, True)
        print("[动作] 读取当前左右手 MPC pose 开始")
        current_left = _current_pose(client, "left", 5.0)
        current_right = _current_pose(client, "right", 5.0)
        target_left = _build_left_target(
            locked_tag,
            current_left,
            offset_x=args.offset_x,
            offset_y=args.offset_y,
            z_offset=args.z_offset,
            above_height=args.above_height,
        )
        left_poses = [current_left, target_left]
        right_poses = [current_right, current_right]
        length = _path_length(left_poses)
        print(
            "目标左手: "
            f"x={target_left['position']['x']:.4f}, "
            f"y={target_left['position']['y']:.4f}, "
            f"z={target_left['position']['z']:.4f}"
        )
        print(f"左手路径长度: {length:.3f} m")
        if length > args.max_motion:
            raise RuntimeError(f"路径长度 {length:.3f}m > --max-motion {args.max_motion:.3f}m")

        request = _build_request(left_poses, right_poses, args.duration, args.type)
        if not args.execute:
            print("[DRY RUN] 未发送运动命令。加 --execute 才会执行。")
            print(json.dumps(request, indent=2, ensure_ascii=False))
            return

        if args.execute_delay > 0:
            print(f"[EXECUTE] {args.execute_delay:.1f}s 后发送，Ctrl+C 可取消")
            time.sleep(args.execute_delay)

        srv_type = _service_type(client, POINTS_SERVICE)
        if not srv_type:
            raise RuntimeError(f"无法获取服务类型: {POINTS_SERVICE}")
        print(f"[动作] 发送 MPC 轨迹开始 service={POINTS_SERVICE}")
        response = _call(client, POINTS_SERVICE, srv_type, request)
        print(f"[MPC] response: {response}")
    finally:
        try:
            client.terminate()
        except Exception:
            pass


if __name__ == "__main__":
    main()
