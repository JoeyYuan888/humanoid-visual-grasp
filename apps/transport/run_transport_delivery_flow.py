#!/usr/bin/env python3
"""Navigate to transport start, transport box, navigate to place area, then place and return."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PYTHON = sys.executable


KEY_LINE_PATTERNS = (
    "[✓]",
    "[✗]",
    "[!]",
    "[动作]",
    "[mpc_mode=",
    "[admittance=",
    "[neck]",
    "[MPC] response:",
    "[hand-",
    "[导航]",
    "waypoint",
    "left :",
    "right:",
    "FoundationPose 盒子识别开始",
    "detected box handles",
    "已锁存盒子 BASE 抓取点",
    "已生成箱子两侧目标",
    "生成双臂路径完成",
    "左臂路径累计长度",
    "右臂路径累计长度",
    "[EXECUTE]",
)


def _run_child_stream(cmd: list[str]) -> tuple[int, str]:
    process = subprocess.Popen(
        cmd,
        cwd=PROJECT_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    lines: list[str] = []
    assert process.stdout is not None
    for line in process.stdout:
        lines.append(line.rstrip("\n"))
        if any(pattern in line for pattern in KEY_LINE_PATTERNS):
            print("    " + line, end="", flush=True)
    return process.wait(), "\n".join(lines)


def _run_step(index: int, total: int, name: str, cmd: list[str]) -> None:
    print(f"\n步骤 {index}/{total} 开始: {name}", flush=True)
    returncode, output = _run_child_stream(cmd)
    if returncode != 0:
        print(f"步骤 {index}/{total} 失败: {name} - exit={returncode}", flush=True)
        tail = output.splitlines()[-50:]
        if tail:
            print("[子进程输出尾部]", flush=True)
            print("\n".join(tail), flush=True)
        raise subprocess.CalledProcessError(returncode, cmd, output=output)
    print(f"步骤 {index}/{total} 完成: {name}", flush=True)


def _transport_cmd(args: argparse.Namespace) -> list[str]:
    cmd = [
        PYTHON,
        "apps/transport/run_transport_flow.py",
        "--ws-url",
        args.ws_url,
        "--backend",
        args.backend,
        "--clamp-control",
        args.clamp_control,
        "--execute-delay",
        f"{args.execute_delay:.3f}",
    ]
    if args.show_window:
        cmd.append("--show-window")
    if args.execute:
        cmd.append("--execute")
    return cmd


def _navigation_cmd(args: argparse.Namespace, goal: str) -> list[str]:
    cmd = [
        PYTHON,
        "apps/navigation/run_navigation_flow.py",
        "--goal",
        goal,
        "--ws-url",
        args.ws_url,
        "--speed-cm-per-s",
        f"{args.speed_cm_per_s:.3f}",
        "--safe-dist-cm",
        f"{args.safe_dist_cm:.3f}",
        "--distance-tolerance",
        f"{args.distance_tolerance:.3f}",
        "--heading-tolerance",
        f"{args.heading_tolerance:.3f}",
        "--result-timeout-sec",
        f"{args.navigation_timeout:.3f}",
    ]
    if not args.execute:
        cmd.append("--dry-run")
    return cmd


def _return_cmd(args: argparse.Namespace) -> list[str]:
    cmd = [
        PYTHON,
        "apps/transport/run_transport_place_return.py",
        "--ws-url",
        args.ws_url,
        "--execute-delay",
        f"{args.execute_delay:.3f}",
    ]
    if args.execute:
        cmd.append("--execute")
    return cmd


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Transport start navigation -> box transport -> place navigation -> box place/return.",
    )
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument("--backend", choices=["foundationpose", "fastsam", "color", "both"], default="foundationpose")
    parser.add_argument("--show-window", action="store_true")
    parser.add_argument("--clamp-control", choices=["admittance", "points"], default="admittance")
    parser.add_argument("--start-navigation-goal", default="transport_start_area")
    parser.add_argument("--navigation-goal", default="transport_place_area", help="Destination navigation goal after the box is carried.")
    parser.add_argument("--speed-cm-per-s", type=float, default=30.0)
    parser.add_argument("--safe-dist-cm", type=float, default=10.0)
    parser.add_argument("--distance-tolerance", type=float, default=0.10)
    parser.add_argument("--heading-tolerance", type=float, default=0.10)
    parser.add_argument("--navigation-timeout", type=float, default=300.0)
    parser.add_argument("--execute-delay", type=float, default=0.0)
    parser.add_argument("--skip-transport", action="store_true")
    parser.add_argument("--skip-navigation", action="store_true", help="Skip both start and destination navigation.")
    parser.add_argument("--skip-start-navigation", action="store_true")
    parser.add_argument("--skip-place-navigation", action="store_true")
    parser.add_argument("--skip-return", action="store_true")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    steps: list[tuple[str, list[str]]] = []
    if not args.skip_navigation and not args.skip_start_navigation:
        steps.append((f"导航到搬运开始点 {args.start_navigation_goal}", _navigation_cmd(args, args.start_navigation_goal)))
    if not args.skip_transport:
        steps.append(("搬运抓取: 识别 -> 夹紧 -> 补夹 -> 抬高 -> 回收", _transport_cmd(args)))
    if not args.skip_navigation and not args.skip_place_navigation:
        steps.append((f"导航到搬运结束点 {args.navigation_goal}", _navigation_cmd(args, args.navigation_goal)))
    if not args.skip_return:
        steps.append(("搬运放置并回 home", _return_cmd(args)))

    print("=" * 70)
    print("  Transport delivery flow")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  Backend: {args.backend}")
    print(f"  Clamp control: {args.clamp_control}")
    print(f"  Start navigation goal: {args.start_navigation_goal}")
    print(f"  Place navigation goal: {args.navigation_goal}")
    print(f"  Execute: {args.execute}")
    print("=" * 70)

    if not steps:
        print("[!] 没有需要执行的步骤", flush=True)
        return

    for index, (name, cmd) in enumerate(steps, start=1):
        _run_step(index, len(steps), name, cmd)

    print("\n[✓] transport 起点导航 -> 搬运 -> 终点导航 -> 放置/return 总流程完成", flush=True)


if __name__ == "__main__":
    main()
