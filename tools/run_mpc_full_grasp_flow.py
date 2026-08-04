#!/usr/bin/env python3
"""
End-to-end MPC visual grasp flow.

Default is plan-only: it prints the commands and does not move the robot.
Use --execute to run the full sequence.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot_grasp import config
from robot_grasp.hand_utils import HAND_CLOSE, HAND_OPEN, clamp_hand_q

try:
    import roslibpy
except ImportError:
    print("缺少 roslibpy 库，请运行: pip install roslibpy")
    sys.exit(1)


HAND_SERVICE = "/zj_humanoid/hand/joint_switch/right"
HAND_SERVICE_TYPE = "hand_controller/JointSwitch"


def _parse_ws_url(ws_url: str) -> tuple[str, int]:
    stripped = ws_url.replace("ws://", "").replace("wss://", "")
    host, port = stripped.split(":")
    return host, int(port)


def _connect(ws_url: str, timeout: float | None = None):
    timeout = config.CONNECT_TIMEOUT if timeout is None else float(timeout)
    host, port = _parse_ws_url(ws_url)
    client = roslibpy.Ros(host=host, port=port)
    thread = threading.Thread(target=client.run, daemon=True)
    thread.start()
    start = time.time()
    while not client.is_connected:
        if time.time() - start > timeout:
            raise RuntimeError(f"连接超时: {ws_url}")
        time.sleep(0.1)
    return client


class HandClient:
    def __init__(self, ws_url: str):
        self.ws_url = ws_url
        self.client = None
        self.service = None

    def connect(self, retries: int = 3, retry_delay: float = 1.0):
        last_exc = None
        for attempt in range(1, retries + 1):
            try:
                self.client = _connect(self.ws_url, timeout=10.0)
                self.service = roslibpy.Service(self.client, HAND_SERVICE, HAND_SERVICE_TYPE)
                print(f"[✓] hand client 已连接: {self.ws_url}")
                return
            except Exception as exc:
                last_exc = exc
                print(f"[!] hand client 连接失败 {attempt}/{retries}: {exc}")
                self.close()
                if attempt < retries:
                    time.sleep(retry_delay)
        raise RuntimeError(f"hand client connect failed after {retries} attempts: {last_exc}")

    def call(self, q: list[float], retries: int = 2, retry_delay: float = 1.0):
        q = clamp_hand_q(q)
        print(f"[*] hand joint_switch: {q}")
        last_exc = None
        for attempt in range(1, retries + 1):
            try:
                if self.client is None or self.service is None or not self.client.is_connected:
                    self.connect()
                response = self.service.call(roslibpy.ServiceRequest({"q": q}))
                print(f"[hand] {response}")
                if response and response.get("success") is False:
                    raise RuntimeError(f"hand joint_switch failed: {response}")
                return
            except Exception as exc:
                last_exc = exc
                print(f"[!] hand 调用失败 {attempt}/{retries}: {exc}")
                self.close()
                if attempt < retries:
                    time.sleep(retry_delay)
        raise RuntimeError(f"hand joint_switch failed after {retries} attempts: {last_exc}")

    def close(self):
        if self.client is None:
            return
        try:
            self.client.terminate()
        except Exception:
            pass
        self.client = None
        self.service = None


def _run_step(name: str, cmd: list[str], execute: bool):
    print("\n" + "=" * 70)
    print(f"STEP: {name}")
    print("=" * 70)
    print(" ".join(cmd))
    if not execute:
        return
    subprocess.run(cmd, cwd=PROJECT_ROOT, check=True)


def _append_execute_flags(cmd: list[str], execute: bool) -> list[str]:
    if execute:
        return [*cmd, "--execute", "--confirm-target"]
    return cmd


def _via_cmd(args, via_files: list[str], max_motion: float, use_joints: bool = True) -> list[str]:
    cmd = [
        sys.executable,
        "tools/run_mpc_visual_grasp_test.py",
        "--ws-url",
        args.ws_url,
        "--use-locked-target",
        args.locked_target,
    ]
    for path in via_files:
        cmd.extend(["--via-file", path])
    cmd.extend(["--stop-at-last-via", "--no-auto-lift"])
    if use_joints:
        cmd.append("--use-joints")
    cmd.extend([
        "--max-motion", f"{max_motion:.3f}",
        "--max-z", f"{args.max_z:.3f}",
        "--duration", f"{args.motion_duration:.3f}",
        "--execute-delay", f"{args.execute_delay:.3f}",
    ])
    return _append_execute_flags(cmd, args.execute)


def _descend_cmd(args) -> list[str]:
    cmd = [
        sys.executable,
        "tools/run_mpc_visual_grasp_test.py",
        "--ws-url",
        args.ws_url,
        "--use-locked-target",
        args.locked_target,
        "--include-descend",
        "--above-object-height",
        f"{args.above_object_height:.3f}",
        "--no-auto-lift",
        "--max-motion",
        f"{args.descend_max_motion:.3f}",
        "--max-z",
        f"{args.max_z:.3f}",
        "--duration",
        f"{args.motion_duration:.3f}",
        "--execute-delay",
        f"{args.execute_delay:.3f}",
    ]
    return _append_execute_flags(cmd, args.execute)


def _approach_and_descend_cmd(args, via_files: list[str]) -> list[str]:
    cmd = [
        sys.executable,
        "tools/run_mpc_visual_grasp_test.py",
        "--ws-url",
        args.ws_url,
        "--use-locked-target",
        args.locked_target,
    ]
    for path in via_files:
        cmd.extend(["--via-file", path])
    cmd.extend([
        "--include-descend",
        "--above-object-height", f"{args.above_object_height:.3f}",
        "--no-auto-lift",
        "--use-joints",
        "--max-motion", f"{args.combined_max_motion:.3f}",
        "--max-z", f"{args.max_z:.3f}",
        "--duration", f"{args.motion_duration:.3f}",
        "--execute-delay", f"{args.execute_delay:.3f}",
    ])
    return _append_execute_flags(cmd, args.execute)


def _lock_cmd(args) -> list[str]:
    cmd = [
        sys.executable,
        "tools/run_mpc_perception_lock.py",
        "--ws-url",
        args.ws_url,
        "--detect-seconds",
        f"{args.detect_seconds:.1f}",
        "--frame-timeout",
        f"{args.frame_timeout:.1f}",
        "--output",
        args.locked_target,
    ]
    if args.show_window:
        cmd.append("--show-window")
    if args.allow_cpu_detect:
        cmd.append("--allow-cpu-detect")
    return cmd


def _qr_scan_cmd(args) -> list[str]:
    cmd = [
        sys.executable,
        "tools/run_post_grasp_qr_scan.py",
        "--ws-url",
        args.ws_url,
        "--duration",
        f"{args.qr_scan_seconds:.1f}",
        "--output",
        args.qr_output,
    ]
    if args.show_window:
        cmd.append("--show-window")
    return cmd


def _load_locked_target(path: str) -> dict | None:
    if not os.path.exists(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws-url", default="ws://192.168.20.98:9091")
    parser.add_argument("--locked-target", default=os.path.join("data", "mpc_locked_target_latest.json"))
    parser.add_argument("--detect-seconds", type=float, default=12.0)
    parser.add_argument("--frame-timeout", type=float, default=8.0)
    parser.add_argument("--show-window", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--allow-cpu-detect", action="store_true")
    parser.add_argument("--above-object-height", type=float, default=0.02)
    parser.add_argument("--descend-max-motion", type=float, default=2.0)
    parser.add_argument("--via-max-motion", type=float, default=2.0)
    parser.add_argument("--return-max-motion", type=float, default=2.0)
    parser.add_argument("--combined-max-motion", type=float, default=3.0)
    parser.add_argument("--max-z", type=float, default=1.20,
                        help="Workspace upper z limit passed to MPC path child scripts.")
    parser.add_argument("--motion-duration", type=float, default=5.0,
                        help="Duration per MPC path point, lower is faster. Minimum accepted by child script is 3s.")
    parser.add_argument("--execute-delay", type=float, default=0.0,
                        help="Delay before each MPC service call. Default 0 for integrated flow.")
    parser.add_argument("--return-mode", choices=["none", "via3", "via0", "qr-present"], default="via3")
    parser.add_argument("--qr-present-file", default=os.path.join("data", "mpc_qr_present_pose_right.json"),
                        help="MPC pose captured at the post-grasp QR presentation position.")
    parser.add_argument("--scan-qr-after-present", action="store_true",
                        help="After moving to qr-present, run full-frame head-camera QR scan.")
    parser.add_argument("--qr-scan-seconds", type=float, default=30.0)
    parser.add_argument("--qr-output", default=os.path.join("data", "post_grasp_qr_latest.json"))
    parser.add_argument("--place-after-qr", action="store_true",
                        help="After QR presentation/scan, move back to visual grasp point, open hand, then return via3->2->1->0.")
    parser.add_argument("--skip-lock", action="store_true", help="Use existing locked target instead of running perception lock.")
    parser.add_argument("--skip-hand-open", action="store_true")
    parser.add_argument("--skip-hand-close", action="store_true")
    parser.add_argument("--combine-approach", action="store_true",
                        help="Experimental: send via0->via3->target in one MPC command.")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    via0 = os.path.join("data", "mpc_via0_home_right.json")
    via1 = os.path.join("data", "mpc_via1_pose_right.json")
    via2 = os.path.join("data", "mpc_via2_pose_right.json")
    via3 = os.path.join("data", "mpc_via3_pose_right.json")

    print("=" * 70)
    print("  MPC full visual grasp flow")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  Locked target: {args.locked_target}")
    print("  Default offsets are read from tools/run_mpc_visual_grasp_test.py")
    print("  Current expected default: x=-0.04, y=-0.10, z=+0.35")
    print(f"  Execute: {args.execute}")
    print("=" * 70)

    if not args.execute:
        print("[PLAN ONLY] 未发送任何运动/手掌命令。加 --execute 才会执行。")

    hand_client = None
    needs_hand = not args.skip_hand_open or not args.skip_hand_close or args.place_after_qr
    if args.execute and needs_hand:
        hand_client = HandClient(args.ws_url)
        hand_client.connect()

    try:
        if not args.skip_lock:
            _run_step("低头、视觉锁存、抬头", _lock_cmd(args), args.execute)
        elif _load_locked_target(args.locked_target) is None:
            raise SystemExit(f"[✗] --skip-lock 但目标文件不存在: {args.locked_target}")

        if not args.skip_hand_open:
            print("\n" + "=" * 70)
            print("STEP: 手掌复位/放开")
            print("=" * 70)
            print(f"{HAND_SERVICE} q={HAND_OPEN}")
            if args.execute:
                hand_client.call(HAND_OPEN)

        if args.combine_approach:
            _run_step(
                "via0 -> via1 -> via2 -> via3 -> 视觉抓取点",
                _approach_and_descend_cmd(args, [via0, via1, via2, via3]),
                args.execute,
            )
        else:
            _run_step(
                "via0 -> via1 -> via2 -> via3",
                _via_cmd(args, [via0, via1, via2, via3], args.via_max_motion, use_joints=True),
                args.execute,
            )

            _run_step(
                "via3 -> 视觉抓取点",
                _descend_cmd(args),
                args.execute,
            )

        if not args.skip_hand_close:
            print("\n" + "=" * 70)
            print("STEP: 手掌闭合抓取")
            print("=" * 70)
            print(f"{HAND_SERVICE} q={HAND_CLOSE}")
            if args.execute:
                hand_client.call(HAND_CLOSE)

        if args.return_mode == "via3":
            _run_step(
                "抓取后返回 via3",
                _via_cmd(args, [via3], args.return_max_motion, use_joints=False),
                args.execute,
            )
        elif args.return_mode == "via0":
            _run_step(
                "抓取后 via3 -> via2 -> via1 -> via0",
                _via_cmd(args, [via3, via2, via1, via0], args.return_max_motion, use_joints=True),
                args.execute,
            )
        elif args.return_mode == "qr-present":
            if not os.path.exists(args.qr_present_file):
                raise SystemExit(f"[✗] QR 展示点文件不存在: {args.qr_present_file}")
            _run_step(
                "抓取后直接到 QR 展示点",
                _via_cmd(args, [args.qr_present_file], args.return_max_motion, use_joints=True),
                args.execute,
            )
            if args.scan_qr_after_present:
                _run_step(
                    "头部相机整帧扫码",
                    _qr_scan_cmd(args),
                    args.execute,
                )
            if args.place_after_qr:
                _run_step(
                    "QR 展示点 -> via3",
                    _via_cmd(args, [via3], args.return_max_motion, use_joints=True),
                    args.execute,
                )
                _run_step(
                    "via3 -> 视觉抓取/放置点",
                    _descend_cmd(args),
                    args.execute,
                )
                print("\n" + "=" * 70)
                print("STEP: 手掌放开完成放置")
                print("=" * 70)
                print(f"{HAND_SERVICE} q={HAND_OPEN}")
                if args.execute:
                    hand_client.call(HAND_OPEN)
                _run_step(
                    "放置后 via3 -> via2 -> via1 -> via0",
                    _via_cmd(args, [via3, via2, via1, via0], args.return_max_motion, use_joints=True),
                    args.execute,
                )
        else:
            print("[*] return-mode=none，抓取后保持当前位置")
    finally:
        if hand_client is not None:
            hand_client.close()


if __name__ == "__main__":
    main()
