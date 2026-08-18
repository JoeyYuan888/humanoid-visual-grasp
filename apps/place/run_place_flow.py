#!/usr/bin/env python3
"""Placement-stage flow.

Validated sequence:
  AprilTag lock
  -> place-ready to left mid point
  -> left hand above pull point 12cm
  -> left hand grasp shape
  -> descend to pull point
  -> pull box out 20cm
  -> right hand via mid to drop point
  -> right hand open
  -> right hand return via mid
  -> push box back, over-push 1cm
  -> release by pulling back 1cm
  -> lift 12cm
  -> left hand home
  -> return via mid to place-ready
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import threading
import time


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

try:
    import roslibpy
except ImportError:
    print("缺少 roslibpy 库，请运行: pip install roslibpy")
    raise SystemExit(1)


HAND_SERVICE_TYPE = "hand_controller/JointSwitch"
LEFT_HAND_SERVICE = "/zj_humanoid/hand/joint_switch/left"
RIGHT_HAND_SERVICE = "/zj_humanoid/hand/joint_switch/right"

PLACE_READY = "data/poses/place/place_ready_after_grasp_dual.json"
LEFT_MID = "data/poses/place/place_left_pull_mid_dual.json"
RIGHT_MID = "data/poses/place/place_right_drop_mid_dual.json"
LOCKED_TAG = "data/runtime/place_apriltag_target_latest.json"

PULL_OFFSET = (-0.0819, -0.0180, 0.4883)
PULLED_OFFSET = (-0.2819, -0.0180, 0.4883)
PUSHED_OFFSET = (-0.0719, -0.0180, 0.4883)
RELEASE_OFFSET = (-0.0819, -0.0180, 0.4883)
RIGHT_DROP_OFFSET = (-0.1819, -0.0680, 0.6883)

LEFT_HAND_GRASP = [-0.5, 1.2, 0.5, 0.6, 0.6, 0.6]
LEFT_HAND_HOME = [-0.1, 0.05, 0.35, 0.35, 0.35, 0.35]
RIGHT_HAND_OPEN = [-0.1, 0.05, 0.35, 0.35, 0.35, 0.35]
STOP_STAGE_TO_STEP = {
    "tag-locked": 1,
    "mid": 2,
    "above-pull": 3,
    "hand-grasp": 4,
    "pull-grasp": 5,
    "pulled": 6,
    "right-drop": 7,
    "right-open": 8,
    "right-return": 9,
    "pushed": 10,
    "lifted": 11,
    "hand-home": 12,
    "ready": 13,
}

KEY_PATTERNS = (
    "步骤 ",
    "[动作]",
    "[✓]",
    "[!]",
    "[✗]",
    "[MPC]",
    "[mpc_mode",
    "[neck]",
    "selected tag:",
    "base tag",
    "目标左手:",
    "目标右手:",
    "左手路径长度:",
    "右手路径长度:",
    "路径累计长度",
    "Dual-arm",
)


def _parse_ws_url(ws_url: str) -> tuple[str, int]:
    stripped = ws_url.replace("ws://", "").replace("wss://", "")
    host, port = stripped.split(":")
    return host, int(port)


def _connect(ws_url: str, timeout: float = 10.0):
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


def _safe_close(client) -> None:
    try:
        if client is not None and getattr(client, "is_connected", False):
            client.close(timeout=1.0)
    except Exception:
        pass


def _call_hand(ws_url: str, service_name: str, label: str, q: list[float]) -> None:
    client = _connect(ws_url)
    try:
        service = roslibpy.Service(client, service_name, HAND_SERVICE_TYPE)
        response = service.call(roslibpy.ServiceRequest({"q": [float(v) for v in q]}))
        if response and response.get("success") is False:
            raise RuntimeError(f"{label} hand joint_switch failed: {response}")
        print(f"[hand-{label}] success", flush=True)
    finally:
        _safe_close(client)


def _line_is_key(line: str) -> bool:
    return any(pattern in line for pattern in KEY_PATTERNS)


def _run_command(name: str, cmd: list[str], *, verbose: bool = False) -> None:
    print(f"    [动作] {name} 开始", flush=True)
    if verbose:
        print("    " + " ".join(cmd), flush=True)
    process = subprocess.Popen(
        cmd,
        cwd=PROJECT_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    tail: list[str] = []
    assert process.stdout is not None
    for line in process.stdout:
        line = line.rstrip()
        if line:
            tail.append(line)
            tail = tail[-30:]
        if verbose or _line_is_key(line):
            print("    " + line, flush=True)
    returncode = process.wait()
    if returncode != 0:
        print("[子进程输出尾部]", flush=True)
        for item in tail[-20:]:
            print(item, flush=True)
        raise subprocess.CalledProcessError(returncode, cmd)
    print(f"    [动作] {name} 完成", flush=True)


class StopAfterStep(Exception):
    pass


def _step(index: int, total: int, name: str, fn, stop_after_step: int | None) -> None:
    print(f"\n步骤 {index}/{total} 开始: {name}", flush=True)
    try:
        fn()
    except Exception as exc:
        print(f"步骤 {index}/{total} 失败: {name} - {exc}", flush=True)
        raise
    print(f"步骤 {index}/{total} 完成: {name}", flush=True)
    if stop_after_step == index:
        print(f"\n[✓] 已按要求停止在步骤 {index}/{total}: {name}", flush=True)
        raise StopAfterStep


def _dual_path_cmd(args, targets: list[str], *, duration: float) -> list[str]:
    cmd = [
        sys.executable,
        "apps/transport/run_dual_pose_path.py",
        "--ws-url",
        args.ws_url,
    ]
    for target in targets:
        cmd += ["--target", target]
    cmd += [
        "--use-joints",
        "--max-motion",
        f"{args.dual_max_motion:.3f}",
        "--duration",
        f"{duration:.3f}",
        "--execute-delay",
        f"{args.execute_delay:.3f}",
    ]
    if args.execute:
        cmd.append("--execute")
    return cmd


def _left_pull_cmd(args, offset: tuple[float, float, float], above_height: float, duration: float, max_motion: float) -> list[str]:
    ox, oy, oz = offset
    cmd = [
        sys.executable,
        "apps/place/run_left_pull_approach.py",
        "--ws-url",
        args.ws_url,
        "--locked-tag",
        args.locked_tag,
        "--offset-x",
        f"{ox:.4f}",
        "--offset-y",
        f"{oy:.4f}",
        "--z-offset",
        f"{oz:.4f}",
        "--above-height",
        f"{above_height:.3f}",
        "--duration",
        f"{duration:.3f}",
        "--max-motion",
        f"{max_motion:.3f}",
        "--execute-delay",
        f"{args.execute_delay:.3f}",
    ]
    if args.execute:
        cmd.append("--execute")
    return cmd


def _right_drop_cmd(
    args,
    offset: tuple[float, float, float],
    above_height: float,
    duration: float,
    max_motion: float,
) -> list[str]:
    ox, oy, oz = offset
    cmd = [
        sys.executable,
        "apps/place/run_right_drop_approach.py",
        "--ws-url",
        args.ws_url,
        "--locked-tag",
        args.locked_tag,
        "--via-file",
        args.right_mid,
        "--offset-x",
        f"{ox:.4f}",
        "--offset-y",
        f"{oy:.4f}",
        "--z-offset",
        f"{oz:.4f}",
        "--above-height",
        f"{above_height:.3f}",
        "--duration",
        f"{duration:.3f}",
        "--max-motion",
        f"{max_motion:.3f}",
        "--execute-delay",
        f"{args.execute_delay:.3f}",
    ]
    if args.execute:
        cmd.append("--execute")
    return cmd


def _right_return_cmd(args, duration: float, max_motion: float) -> list[str]:
    cmd = [
        sys.executable,
        "apps/place/run_right_drop_approach.py",
        "--ws-url",
        args.ws_url,
        "--via-file",
        args.right_mid,
        "--target-file",
        args.place_ready,
        "--duration",
        f"{duration:.3f}",
        "--max-motion",
        f"{max_motion:.3f}",
        "--execute-delay",
        f"{args.execute_delay:.3f}",
    ]
    if args.execute:
        cmd.append("--execute")
    return cmd


def _run_right_drop_after_optional_pause(args) -> None:
    if args.execute and args.pause_after_pull:
        input("    箱子已拉出。按 Enter 开始右手投放；Ctrl+C 中止: ")
    if args.skip_right_drop:
        print("    [跳过右手投放路径]", flush=True)
        return
    _run_command(
        "右手到投放点",
        _right_drop_cmd(args, RIGHT_DROP_OFFSET, above_height=0.00, duration=5.0, max_motion=1.5),
        verbose=args.verbose_children,
    )


def _apriltag_cmd(args) -> list[str]:
    cmd = [
        sys.executable,
        "apps/place/run_apriltag_lock.py",
        "--ws-url",
        args.ws_url,
        "--shelf-level",
        str(args.shelf_level),
        "--output",
        args.locked_tag,
    ]
    if args.show_tag_window:
        cmd.append("--show-window")
    return cmd


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the validated placement flow.")
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument("--shelf-level", type=int, default=2, choices=[2, 3])
    parser.add_argument("--locked-tag", default=LOCKED_TAG)
    parser.add_argument("--place-ready", default=PLACE_READY)
    parser.add_argument("--left-mid", default=LEFT_MID)
    parser.add_argument("--right-mid", default=RIGHT_MID)
    parser.add_argument("--execute-delay", type=float, default=2.0)
    parser.add_argument("--dual-max-motion", type=float, default=2.0)
    parser.add_argument("--show-tag-window", action="store_true")
    parser.add_argument("--verbose-children", action="store_true")
    parser.add_argument("--pause-after-pull", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--skip-right-drop", action="store_true", help="Only run left-hand pull/push actions.")
    parser.add_argument(
        "--stop-after-step",
        type=int,
        default=None,
        help="Run through this numbered step and exit cleanly.",
    )
    parser.add_argument(
        "--stop-after-stage",
        choices=sorted(STOP_STAGE_TO_STEP),
        default=None,
        help="Named stop point, e.g. pulled means stop after the box is pulled out 20cm.",
    )
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if args.stop_after_stage:
        args.stop_after_step = STOP_STAGE_TO_STEP[args.stop_after_stage]

    print("=" * 70)
    print("  Place stage flow")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  Shelf level: {args.shelf_level}")
    print(f"  Locked tag: {args.locked_tag}")
    print(f"  Place ready: {args.place_ready}")
    print(f"  Left mid: {args.left_mid}")
    print(f"  Right mid: {args.right_mid}")
    print(
        "  Right drop offset: "
        f"x={RIGHT_DROP_OFFSET[0]:.4f}, y={RIGHT_DROP_OFFSET[1]:.4f}, z={RIGHT_DROP_OFFSET[2]:.4f}"
    )
    print(f"  Execute: {args.execute}")
    print("=" * 70)

    total = 13
    try:
        _step(1, total, "AprilTag 锁存目标箱", lambda: _run_command("AprilTag 锁存", _apriltag_cmd(args), verbose=args.verbose_children), args.stop_after_step)
        _step(
            2,
            total,
            "预放置点 -> 左手中间点",
            lambda: _run_command(
                "移动到左手中间点",
                _dual_path_cmd(args, [args.left_mid], duration=5.0),
                verbose=args.verbose_children,
            ),
            args.stop_after_step,
        )
        _step(
            3,
            total,
            "左手中间点 -> 拉箱点上方 12cm",
            lambda: _run_command(
                "左手到拉箱点上方",
                _left_pull_cmd(args, PULL_OFFSET, above_height=0.12, duration=6.0, max_motion=1.2),
                verbose=args.verbose_children,
            ),
            args.stop_after_step,
        )
        _step(
            4,
            total,
            "左手切换扣住/拉箱手型",
            lambda: (
                _call_hand(args.ws_url, LEFT_HAND_SERVICE, "left", LEFT_HAND_GRASP)
                if args.execute
                else print("    [DRY RUN] left hand grasp", flush=True)
            ),
            args.stop_after_step,
        )
        _step(
            5,
            total,
            "左手下降到拉箱点",
            lambda: _run_command(
                "左手下降到拉箱点",
                _left_pull_cmd(args, PULL_OFFSET, above_height=0.00, duration=3.0, max_motion=0.3),
                verbose=args.verbose_children,
            ),
            args.stop_after_step,
        )
        _step(
            6,
            total,
            "左手拉出箱子 20cm",
            lambda: _run_command(
                "左手拉出 20cm",
                _left_pull_cmd(args, PULLED_OFFSET, above_height=0.00, duration=5.0, max_motion=0.3),
                verbose=args.verbose_children,
            ),
            args.stop_after_step,
        )
        _step(
            7,
            total,
            "右手经中间点到投放点",
            lambda: _run_right_drop_after_optional_pause(args),
            args.stop_after_step,
        )
        _step(
            8,
            total,
            "右手松开塑料袋",
            lambda: (
                _call_hand(args.ws_url, RIGHT_HAND_SERVICE, "right", RIGHT_HAND_OPEN)
                if args.execute and not args.skip_right_drop
                else print("    [跳过右手松开]", flush=True)
            ),
            args.stop_after_step,
        )
        _step(
            9,
            total,
            "右手原路返回预放置姿态",
            lambda: (
                _run_command(
                    "右手经中间点返回",
                    _right_return_cmd(args, duration=5.0, max_motion=1.5),
                    verbose=args.verbose_children,
                )
                if not args.skip_right_drop
                else print("    [跳过右手返回]", flush=True)
            ),
            args.stop_after_step,
        )
        _step(
            10,
            total,
            "左手推回箱子并多推 1cm",
            lambda: _run_command(
                "左手推回",
                _left_pull_cmd(args, PUSHED_OFFSET, above_height=0.00, duration=5.0, max_motion=0.3),
                verbose=args.verbose_children,
            ),
            args.stop_after_step,
        )
        _step(
            11,
            total,
            "左手回拉 1cm 后上抬 12cm",
            lambda: (
                _run_command(
                    "左手回拉 1cm",
                    _left_pull_cmd(args, RELEASE_OFFSET, above_height=0.00, duration=3.0, max_motion=0.2),
                    verbose=args.verbose_children,
                ),
                _run_command(
                    "左手上抬 12cm",
                    _left_pull_cmd(args, RELEASE_OFFSET, above_height=0.12, duration=4.0, max_motion=0.3),
                    verbose=args.verbose_children,
                ),
            ),
            args.stop_after_step,
        )
        _step(
            12,
            total,
            "左手手指恢复 home",
            lambda: (
                _call_hand(args.ws_url, LEFT_HAND_SERVICE, "left", LEFT_HAND_HOME)
                if args.execute
                else print("    [DRY RUN] left hand home", flush=True)
            ),
            args.stop_after_step,
        )
        _step(
            13,
            total,
            "沿中间点返回预放置点",
            lambda: _run_command(
                "返回预放置点",
                _dual_path_cmd(args, [args.left_mid, args.place_ready], duration=5.0),
                verbose=args.verbose_children,
            ),
            args.stop_after_step,
        )
    except StopAfterStep:
        return
    print("\n[✓] place flow completed", flush=True)


if __name__ == "__main__":
    main()
