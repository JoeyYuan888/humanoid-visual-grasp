#!/usr/bin/env python3
"""Capture current dual-arm MPC poses and optional MPC joint state.

Read-only helper for the transport stage.
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

from tools.capture.capture_mpc_pose import (  # noqa: E402
    CURRENT_STATE_TOPIC,
    FRAME_TOPICS,
    _connect,
    _extract_current_mpc_state,
    _pose_stamped_to_pose,
    _wait_for_current_state,
    _wait_for_pose,
)
from robot_grasp.common import config  # noqa: E402


DEFAULT_OUTPUT = os.path.join(PROJECT_ROOT, "data", "poses", "transport", "dual_mpc_pose_latest.json")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws-url", default=config.WS_URL)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--include-joints", action="store_true")
    parser.add_argument("--name", default="", help="Optional waypoint name, e.g. transport_home")
    args = parser.parse_args()

    print("=" * 70)
    print("  Capture dual-arm MPC pose")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  Left topic: {FRAME_TOPICS['left']}")
    print(f"  Right topic: {FRAME_TOPICS['right']}")
    if args.include_joints:
        print(f"  Joint topic: {CURRENT_STATE_TOPIC}")
    print(f"  Output: {args.output}")
    print("=" * 70)

    client = _connect(args.ws_url)
    print("[✓] 已连接 rosbridge")
    try:
        messages = {}
        for arm in ("left", "right"):
            message = _wait_for_pose(client, FRAME_TOPICS[arm], args.timeout)
            if message is None:
                print(f"[✗] {args.timeout:.1f}s 内没有收到 {FRAME_TOPICS[arm]}")
                raise SystemExit(1)
            messages[arm] = message

        poses = {arm: _pose_stamped_to_pose(message) for arm, message in messages.items()}
        payload = {
            "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "name": args.name,
            "type": "dual_arm_mpc_pose",
            "left": {
                "arm": "left",
                "topic": FRAME_TOPICS["left"],
                "pose": poses["left"],
                "orientation": poses["left"]["orientation"],
            },
            "right": {
                "arm": "right",
                "topic": FRAME_TOPICS["right"],
                "pose": poses["right"],
                "orientation": poses["right"]["orientation"],
            },
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

        print("\nleft MPC pose:")
        print(json.dumps(poses["left"], indent=2, ensure_ascii=False))
        print("\nright MPC pose:")
        print(json.dumps(poses["right"], indent=2, ensure_ascii=False))
        if args.include_joints:
            print(f"\n已同时保存 MPC joints: joint_num={len(payload['mpc_state'])}")
        print(f"\n[✓] 已保存: {args.output}")
    finally:
        try:
            client.terminate()
        except Exception:
            pass


if __name__ == "__main__":
    main()
