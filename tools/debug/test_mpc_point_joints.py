#!/usr/bin/env python3
"""Move one arm to an MPC/BASE point, then print the current joint angles."""

from __future__ import annotations

import argparse
import json
import math
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
    raise SystemExit(1)


POINTS_SERVICE = "/wa/points_seq_tracking"
MPC_MODE_SERVICE = "/wa/wa_hardware_interface/mpc_mode_setting"
CURRENT_STATE_TOPIC = "/DualArmMobile/currenState"
DEFAULT_TARGET_POSITION = (0.563262643, 0.162025684, 1.187653196)
DEFAULT_TARGET_ORIENTATION = (0.301482087, 0.425088674, 0.100563272, -0.847522978)
FRAME_TOPICS = {
    "left": "/DualArmMobile/currentEEPose/FrameL",
    "right": "/DualArmMobile/currentEEPose/FrameR",
}
WA2_JOINT_NAMES = [
    "x_dir_joint", "y_dir_joint", "z_dir_joint",
    "Pitch_Y_B", "Pitch_Y_M", "Waist_Z", "Waist_Y",
    "Shoulder_Z_L", "Shoulder_Y_L", "Shoulder_X_L",
    "Elbow_Z_L", "Elbow_Y_L", "Wrist_Z_L", "Wrist_Y_L", "Wrist_X_L",
    "Shoulder_Z_R", "Shoulder_Y_R", "Shoulder_X_R",
    "Elbow_Z_R", "Elbow_Y_R", "Wrist_Z_R", "Wrist_Y_R", "Wrist_X_R",
]


def _connect(ws_url: str):
    address = ws_url.removeprefix("ws://").removeprefix("wss://")
    host, port_text = address.rsplit(":", 1)
    client = roslibpy.Ros(host=host, port=int(port_text))
    threading.Thread(target=client.run, daemon=True).start()
    deadline = time.time() + config.CONNECT_TIMEOUT
    while not client.is_connected:
        if time.time() >= deadline:
            raise RuntimeError(f"连接 rosbridge 超时: {ws_url}")
        time.sleep(0.1)
    return client


def _call(client, name: str, service_type: str, request: dict | None = None):
    service = roslibpy.Service(client, name, service_type)
    return service.call(roslibpy.ServiceRequest(request or {}))


def _service_type(client, name: str) -> str:
    response = _call(client, "/rosapi/service_type", "rosapi/ServiceType", {"service": name})
    return response.get("type", "")


def _wait_for_message(client, topic: str, topic_type: str, timeout: float) -> dict:
    result: dict[str, dict] = {}
    event = threading.Event()

    def callback(message):
        result["message"] = message
        event.set()

    subscriber = roslibpy.Topic(client, topic, topic_type)
    subscriber.subscribe(callback)
    received = event.wait(timeout)
    try:
        subscriber.unsubscribe()
    except Exception:
        pass
    if not received:
        raise RuntimeError(f"{timeout:.1f}s 内没有收到话题: {topic}")
    return result["message"]


def _pose(message: dict) -> dict:
    pose = message["pose"]
    return {
        "position": {key: float(pose["position"][key]) for key in ("x", "y", "z")},
        "orientation": {key: float(pose["orientation"][key]) for key in ("x", "y", "z", "w")},
    }


def _pose_array(poses: list[dict]) -> dict:
    return {
        "header": {"seq": 0, "stamp": {"secs": 0, "nsecs": 0}, "frame_id": ""},
        "poses": poses,
    }


def _request(arm: str, current: dict, target: dict, hold: dict,
             duration: float, weight: float) -> dict:
    moving_poses = [current, target]
    hold_poses = [hold, json.loads(json.dumps(hold))]
    return {
        "left_poses": _pose_array(moving_poses if arm == "left" else hold_poses),
        "right_poses": _pose_array(moving_poses if arm == "right" else hold_poses),
        "time_points": [duration, duration],
        "max_period": duration * 2.0 + 2.0,
        "weight": weight,
        "type": "quintic",
    }


def _joint_values(message: dict) -> list[float]:
    trajectory = message.get("stateTrajectory", [])
    if not trajectory or not trajectory[0].get("value"):
        raise RuntimeError("currenState 中没有 stateTrajectory[0].value")
    return [float(value) for value in trajectory[0]["value"]]


def _print_joints(values: list[float]) -> None:
    print(f"\n运动后的 MPC 状态（joint_num={len(values)}）:")
    print(" idx  joint name                  radians       degrees")
    print("----  ------------------------  ------------  ------------")
    for index, value in enumerate(values):
        name = WA2_JOINT_NAMES[index] if index < len(WA2_JOINT_NAMES) else f"joint_{index}"
        print(f"{index:>3}  {name:<24}  {value:>12.6f}  {math.degrees(value):>12.3f}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="将单臂末端移动到 MPC/BASE 绝对点位，并读取各关节角度。"
    )
    parser.add_argument("x", type=float, nargs="?", default=DEFAULT_TARGET_POSITION[0],
                        help="目标 x，单位 m（默认使用脚本内测试点）")
    parser.add_argument("y", type=float, nargs="?", default=DEFAULT_TARGET_POSITION[1],
                        help="目标 y，单位 m（默认使用脚本内测试点）")
    parser.add_argument("z", type=float, nargs="?", default=DEFAULT_TARGET_POSITION[2],
                        help="目标 z，单位 m（默认使用脚本内测试点）")
    parser.add_argument("--arm", choices=("left", "right"), default="left",
                        help="运动手臂（当前内置测试位姿属于左手）")
    parser.add_argument("--ws-url", default=config.WS_URL)
    parser.add_argument("--duration", type=float, default=5.0, help="每个 MPC 路点时长，单位 s")
    parser.add_argument("--weight", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--settle-seconds", type=float, default=1.0, help="服务返回后等待再读角度")
    parser.add_argument("--max-motion", type=float, default=0.30, help="允许的最大直线位移，单位 m")
    parser.add_argument(
        "--orientation", type=float, nargs=4, metavar=("X", "Y", "Z", "W"),
        default=DEFAULT_TARGET_ORIENTATION,
        help="目标四元数，默认使用脚本内测试姿态",
    )
    parser.add_argument("--keep-current-orientation", action="store_true",
                        help="忽略目标四元数，保持当前末端姿态")
    parser.add_argument("--execute", action="store_true", help="实际发送运动命令；不加则仅预览")
    parser.add_argument("--confirm-target", action="store_true", help="确认点位属于 MPC/BASE 坐标系")
    parser.add_argument("--skip-mpc-mode", action="store_true",
                        help="不在运动前调用 mpc_mode_setting（默认会开启 MPC mode）")
    args = parser.parse_args()

    if args.duration < 3.0:
        parser.error("--duration 不能小于 3.0s")
    if args.max_motion <= 0.0:
        parser.error("--max-motion 必须大于 0")

    client = _connect(args.ws_url)
    try:
        service_type = _service_type(client, POINTS_SERVICE)
        if not service_type:
            raise RuntimeError(f"找不到 MPC 服务: {POINTS_SERVICE}")

        moving_message = _wait_for_message(
            client, FRAME_TOPICS[args.arm], "geometry_msgs/PoseStamped", args.timeout
        )
        hold_arm = "left" if args.arm == "right" else "right"
        hold_message = _wait_for_message(
            client, FRAME_TOPICS[hold_arm], "geometry_msgs/PoseStamped", args.timeout
        )
        current = _pose(moving_message)
        hold = _pose(hold_message)
        target = json.loads(json.dumps(current))
        target["position"] = {"x": args.x, "y": args.y, "z": args.z}
        if not args.keep_current_orientation:
            qx, qy, qz, qw = args.orientation
            norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
            if norm == 0.0:
                raise RuntimeError("目标 orientation 四元数的模不能为 0")
            target["orientation"] = {
                "x": qx / norm, "y": qy / norm, "z": qz / norm, "w": qw / norm,
            }
        distance = math.dist(
            [current["position"][key] for key in ("x", "y", "z")],
            [args.x, args.y, args.z],
        )
        request = _request(args.arm, current, target, hold, args.duration, args.weight)

        print(f"当前 {args.arm} 末端: {json.dumps(current, ensure_ascii=False)}")
        print(f"目标 MPC/BASE 点位: x={args.x:.4f}, y={args.y:.4f}, z={args.z:.4f} m")
        orientation_mode = "保持当前值" if args.keep_current_orientation else "使用目标四元数"
        print(f"直线距离: {distance:.4f} m；orientation: {orientation_mode}")
        print("\nMPC request:")
        print(json.dumps(request, indent=2, ensure_ascii=False))

        if not args.execute:
            print("\n[DRY RUN] 未运动。核对点位后添加 --execute --confirm-target。")
            return
        if not args.confirm_target:
            raise RuntimeError("实机执行必须同时添加 --confirm-target")
        if distance > args.max_motion:
            raise RuntimeError(
                f"目标距离 {distance:.4f}m 超过 --max-motion {args.max_motion:.4f}m，取消执行"
            )

        print("\n[EXECUTE] 2 秒后发送 MPC 命令，请准备急停；Ctrl+C 可取消。")
        time.sleep(2.0)
        if not args.skip_mpc_mode:
            mode_service_type = _service_type(client, MPC_MODE_SERVICE)
            if not mode_service_type:
                raise RuntimeError(f"找不到 MPC mode 服务: {MPC_MODE_SERVICE}")
            mode_response = _call(client, MPC_MODE_SERVICE, mode_service_type, {"data": True})
            print(f"[mpc_mode=True] {mode_response}")
            if mode_response.get("success") is False:
                raise RuntimeError(f"开启 MPC mode 失败: {mode_response}")
        response = _call(client, POINTS_SERVICE, service_type, request)
        print(f"[MPC] response: {response}")
        if args.settle_seconds > 0:
            time.sleep(args.settle_seconds)

        state = _wait_for_message(
            client, CURRENT_STATE_TOPIC, "ocs2_msgs/mpc_target_trajectories", args.timeout
        )
        _print_joints(_joint_values(state))
    except KeyboardInterrupt:
        print("\n[*] 用户取消")
    finally:
        try:
            client.terminate()
        except Exception:
            pass


if __name__ == "__main__":
    main()
