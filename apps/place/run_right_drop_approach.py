#!/usr/bin/env python3
"""Move the right hand to a placement test point derived from AprilTag.

The left hand keeps its current TCP pose. This is used while the left hand is
holding/pulling the box and must not move.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
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


def _load_right_pose(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if data.get("type") == "dual_arm_mpc_pose":
        pose = data.get("right", {}).get("pose")
    else:
        pose = data.get("pose")
    if not pose or "position" not in pose or "orientation" not in pose:
        raise ValueError(f"{path} 没有 right.pose 或 pose")
    return copy.deepcopy(pose)


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


def _build_right_target(
    locked_tag: dict[str, Any],
    current_right: dict,
    *,
    offset_x: float,
    offset_y: float,
    z_offset: float,
    above_height: float,
) -> dict:
    tag_position = locked_tag["base"]["tag_pose"]["position"]
    target = copy.deepcopy(current_right)
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
        "time_points": [duration for _ in right_poses],
        "max_period": duration * len(right_poses) + 2.0,
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
    parser = argparse.ArgumentParser(description="Move right hand to AprilTag-derived placement point.")
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument("--locked-tag", type=Path, default=DEFAULT_LOCKED_TAG)
    parser.add_argument("--offset-x", type=float, default=None, help="BASE x offset from tag.")
    parser.add_argument("--offset-y", type=float, default=None, help="BASE y offset from tag.")
    parser.add_argument("--z-offset", type=float, default=None, help="BASE z offset from tag before above-height.")
    parser.add_argument("--above-height", type=float, default=0.0)
    parser.add_argument(
        "--target-file",
        type=Path,
        default=None,
        help="Optional pose JSON final target. Uses only right.pose; ignores tag offsets.",
    )
    parser.add_argument(
        "--via-file",
        type=Path,
        action="append",
        default=[],
        help="Optional pose JSON waypoint. Uses only right.pose; left hand stays current.",
    )
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--execute-delay", type=float, default=2.0)
    parser.add_argument("--max-motion", type=float, default=1.2)
    parser.add_argument("--type", default="quintic", choices=["quintic", "cubic"])
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    if args.target_file is None:
        if args.offset_x is None or args.offset_y is None or args.z_offset is None:
            raise SystemExit("--target-file 未设置时必须提供 --offset-x/--offset-y/--z-offset")
        locked_tag = _load_locked_tag(args.locked_tag)
        tag = locked_tag.get("tag", {})
        tag_position = locked_tag["base"]["tag_pose"]["position"]
    else:
        locked_tag = None
        tag = {}
        tag_position = None

    print("=" * 70)
    print("  Place right-hand drop approach")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    if args.target_file is None:
        print(f"  Locked tag: {args.locked_tag}")
        print(f"  Tag: id={tag.get('selected_id')} level={tag.get('shelf_level')}")
        print(
            "  Base tag: "
            f"x={tag_position['x']:.4f}, y={tag_position['y']:.4f}, z={tag_position['z']:.4f}"
        )
        print(f"  Offset: x={args.offset_x:.4f}, y={args.offset_y:.4f}, z={args.z_offset:.4f}")
        print(f"  Above height: {args.above_height:.3f} m")
    else:
        print(f"  Target file: {args.target_file}")
    if args.via_file:
        print("  Right via files: " + ", ".join(str(path) for path in args.via_file))
    print("  Orientation: keep current right hand")
    print("  Left hand: hold current TCP pose")
    print(f"  Execute: {args.execute}")
    print("=" * 70)

    client = _connect(args.ws_url)
    try:
        print("[动作] 设置 MPC running mode 开始")
        _set_mpc_mode(client, True)
        print("[动作] 读取当前左右手 MPC pose 开始")
        current_left = _current_pose(client, "left", 5.0)
        current_right = _current_pose(client, "right", 5.0)
        if args.target_file is None:
            target_right = _build_right_target(
                locked_tag,
                current_right,
                offset_x=args.offset_x,
                offset_y=args.offset_y,
                z_offset=args.z_offset,
                above_height=args.above_height,
            )
        else:
            target_right = _load_right_pose(args.target_file)
        right_waypoints = [_load_right_pose(path) for path in args.via_file]
        left_poses = [current_left for _ in range(2 + len(right_waypoints))]
        right_poses = [current_right, *right_waypoints, target_right]
        length = _path_length(right_poses)
        print(
            "目标右手: "
            f"x={target_right['position']['x']:.4f}, "
            f"y={target_right['position']['y']:.4f}, "
            f"z={target_right['position']['z']:.4f}"
        )
        print(f"右手路径长度: {length:.3f} m")
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
