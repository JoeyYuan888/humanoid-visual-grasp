#!/usr/bin/env python3
"""
MPC points_seq_tracking dry run.

Current MPC route intentionally does not use:
- /wa/joints_seq_tracking
- /DualArmMobile/currenState
- admittance / wrist force interfaces

Default behavior is safe dry-run:
1. Read /DualArmMobile/currentEEPose/FrameR.
2. Build a /wa/points_seq_tracking request from the current pose.
3. Print the request and exit.

Use --execute only after checking the printed request and with a hand on E-stop.
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

from robot_grasp import config

try:
    import roslibpy
    from roslibpy.core import ServiceException
except ImportError:
    print("缺少 roslibpy 库，请运行: pip install roslibpy")
    sys.exit(1)


MPC_PREFIX = "/wa"
POINTS_SERVICE = f"{MPC_PREFIX}/points_seq_tracking"
ORIENTATION_PRESETS = {
    # Course example: Eigen::Quaterniond(0.707, 0, -0.707, 0)
    # geometry_msgs order is x, y, z, w.
    "doc-grasp": {"x": 0.0, "y": -0.70710678, "z": 0.0, "w": 0.70710678},
}
FRAME_TOPICS = {
    "left": "/DualArmMobile/currentEEPose/FrameL",
    "right": "/DualArmMobile/currentEEPose/FrameR",
}


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


def _call(client, name, service_type, request=None):
    service = roslibpy.Service(client, name, service_type)
    return service.call(roslibpy.ServiceRequest(request or {}))


def _print_mpc_target_manifest_hint(exc: Exception) -> bool:
    text = str(exc)
    if "mpc_target" not in text or "Unable to load the manifest" not in text:
        return False
    print("[✗] rosbridge 进程没有加载 mpc_target 包，MPC service call 无法序列化。")
    print("    注意：你当前 shell 里 rospack find mpc_target 成功，不代表 rosbridge 进程也 source 了同一个环境。")
    print("    请在启动 rosbridge 的终端/脚本里执行：")
    print("      source /opt/ros/noetic/setup.bash")
    print("      source /workspace/catkin_ws/mpc_ws/devel/setup.bash")
    print("      roslaunch rosbridge_server rosbridge_websocket.launch")
    print("    如果 rosbridge 是 systemd/supervisor/docker entrypoint 启动，要把 mpc_ws/devel/setup.bash 写进那个启动脚本。")
    return True


def _service_type(client, name: str) -> str:
    try:
        return _call(client, "/rosapi/service_type", "rosapi/ServiceType", {"service": name}).get("type", "")
    except Exception:
        return ""


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


def _offset_pose(pose: dict, dx: float, dy: float, dz: float) -> dict:
    target = json.loads(json.dumps(pose))
    target["position"]["x"] += dx
    target["position"]["y"] += dy
    target["position"]["z"] += dz
    return target


def _normalize_orientation(orientation: dict) -> dict:
    x = float(orientation.get("x", 0.0))
    y = float(orientation.get("y", 0.0))
    z = float(orientation.get("z", 0.0))
    w = float(orientation.get("w", 1.0))
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    if norm == 0:
        raise ValueError("orientation quaternion norm is 0")
    return {"x": x / norm, "y": y / norm, "z": z / norm, "w": w / norm}


def _target_orientation(args) -> dict | None:
    if args.orientation:
        return _normalize_orientation(
            {
                "x": args.orientation[0],
                "y": args.orientation[1],
                "z": args.orientation[2],
                "w": args.orientation[3],
            }
        )
    if args.orientation_preset != "current":
        return _normalize_orientation(ORIENTATION_PRESETS[args.orientation_preset])
    return None


def _pose_array(poses: list[dict]) -> dict:
    return {
        "header": {
            "seq": 0,
            "stamp": {"secs": 0, "nsecs": 0},
            "frame_id": "",
        },
        "poses": poses,
    }


def _hold_poses(pose: dict, count: int) -> list[dict]:
    return [json.loads(json.dumps(pose)) for _ in range(count)]


def build_points_request(arm: str, current_pose: dict, target_pose: dict,
                         duration: float, weight: float, way_type: str,
                         include_current: bool, hold_pose: dict) -> dict:
    poses = [current_pose, target_pose] if include_current else [target_pose]
    hold = _hold_poses(hold_pose, len(poses))
    if arm == "right":
        left_poses = _pose_array(hold)
        right_poses = _pose_array(poses)
    else:
        left_poses = _pose_array(poses)
        right_poses = _pose_array(hold)

    # The MPC document examples use one time point for each pose in the array.
    time_points = [duration for _ in poses]
    return {
        "left_poses": left_poses,
        "right_poses": right_poses,
        "time_points": time_points,
        "max_period": duration * len(poses) + 2.0,
        "weight": weight,
        "type": way_type,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws-url", default=config.WS_URL, help="rosbridge WebSocket URL")
    parser.add_argument("--arm", choices=["right", "left"], default="right")
    parser.add_argument("--dx", type=float, default=0.0, help="MPC x offset in meters")
    parser.add_argument("--dy", type=float, default=0.0, help="MPC y offset in meters")
    parser.add_argument("--dz", type=float, default=0.0, help="MPC z offset in meters; keep 0 for first dry-run")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--weight", type=float, default=1.0)
    parser.add_argument("--type", choices=["quintic", "spline"], default="quintic")
    parser.add_argument("--orientation-preset", choices=["current", *ORIENTATION_PRESETS.keys()], default="current",
                        help="Target TCP orientation preset. Default keeps current orientation.")
    parser.add_argument("--orientation", type=float, nargs=4, metavar=("X", "Y", "Z", "W"),
                        help="Target TCP orientation quaternion in geometry_msgs order x y z w.")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--target-only", action="store_true",
                        help="Only send target pose in PoseArray; default includes current pose then target pose")
    parser.add_argument("--execute", action="store_true",
                        help="Actually call /wa/points_seq_tracking. Default is print-only dry-run.")
    args = parser.parse_args()

    print("=" * 70)
    print("  MPC points_seq_tracking dry run")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  Service: {POINTS_SERVICE}")
    print(f"  Arm: {args.arm}")
    print(f"  Offset: dx={args.dx:.3f} dy={args.dy:.3f} dz={args.dz:.3f} m")
    print(f"  Execute: {args.execute}")
    print("=" * 70)

    client = _connect(args.ws_url)
    print("[✓] 已连接 rosbridge")

    try:
        srv_type = _service_type(client, POINTS_SERVICE)
        if not srv_type:
            print(f"[✗] 找不到服务类型: {POINTS_SERVICE}")
            raise SystemExit(1)
        print(f"[✓] 服务类型: {srv_type}")

        topic = FRAME_TOPICS[args.arm]
        hold_arm = "left" if args.arm == "right" else "right"
        hold_topic = FRAME_TOPICS[hold_arm]
        print(f"[*] 读取当前末端位姿: {topic}")
        message = _wait_for_pose(client, topic, args.timeout)
        if message is None:
            print(f"[✗] {args.timeout:.1f}s 内没有收到 {topic}")
            raise SystemExit(1)
        print(f"[*] 读取保持不动侧末端位姿: {hold_topic}")
        hold_message = _wait_for_pose(client, hold_topic, args.timeout)
        if hold_message is None:
            print(f"[✗] {args.timeout:.1f}s 内没有收到 {hold_topic}")
            raise SystemExit(1)

        current_pose = _pose_stamped_to_pose(message)
        hold_pose = _pose_stamped_to_pose(hold_message)
        target_pose = _offset_pose(current_pose, args.dx, args.dy, args.dz)
        orientation = _target_orientation(args)
        if orientation is not None:
            target_pose["orientation"] = orientation
        request = build_points_request(
            arm=args.arm,
            current_pose=current_pose,
            target_pose=target_pose,
            duration=args.duration,
            weight=args.weight,
            way_type=args.type,
            include_current=not args.target_only,
            hold_pose=hold_pose,
        )

        print("\n当前 MPC 末端 pose:")
        print(json.dumps(current_pose, indent=2, ensure_ascii=False))
        print("\n目标 MPC 末端 pose:")
        print(json.dumps(target_pose, indent=2, ensure_ascii=False))
        if orientation is not None:
            print("\n[!] 本次会改变目标 TCP orientation，请先做小步/原地姿态测试。")
        print(f"\n保持不动侧 {hold_arm} 末端 pose:")
        print(json.dumps(hold_pose, indent=2, ensure_ascii=False))
        print("\n准备发送的 PointsSeqTracking request:")
        print(json.dumps(request, indent=2, ensure_ascii=False))

        if not args.execute:
            print("\n[DRY RUN] 未发送运动命令。确认结构无误后，再考虑 --execute。")
            return

        if max(abs(args.dx), abs(args.dy), abs(args.dz)) > 0.03:
            print("[✗] 为了首轮安全，--execute 单轴位移绝对值不能超过 0.03m")
            raise SystemExit(1)
        if args.duration < 5.0:
            print("[✗] 为了首轮安全，--execute duration 不能小于 5.0s")
            raise SystemExit(1)
        if args.weight > 1.0:
            print("[✗] 为了首轮安全，--execute weight 不能大于 1.0")
            raise SystemExit(1)

        print("\n[EXECUTE] 即将调用 /wa/points_seq_tracking。请确认手在急停上。")
        print("          2 秒后发送，Ctrl+C 可取消。")
        time.sleep(2.0)
        try:
            response = _call(client, POINTS_SERVICE, srv_type, request)
            print(f"[MPC] response: {response}")
        except ServiceException as exc:
            if not _print_mpc_target_manifest_hint(exc):
                raise

    except KeyboardInterrupt:
        print("\n[*] 用户取消")
    finally:
        try:
            client.terminate()
        except Exception:
            pass


if __name__ == "__main__":
    main()
