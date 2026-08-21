#!/usr/bin/env python3
"""Place transported box, retreat outward, then return through pregrasp to home."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PYTHON = sys.executable
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from apps.transport.run_transport_flow import (
    LEFT_HAND_SERVICE,
    RIGHT_HAND_SERVICE,
    _call_hand,
    _set_mpc_mode_true,
)

DEFAULT_PLACE = os.path.join(PROJECT_ROOT, "data", "poses", "transport", "transport_place_dual.json")
DEFAULT_OUTSIDE = os.path.join(PROJECT_ROOT, "data", "runtime", "transport_box_side_approach_latest.json")
DEFAULT_PREGRASP = os.path.join(PROJECT_ROOT, "data", "poses", "transport", "transport_pregrasp_dual.json")
DEFAULT_HOME = os.path.join(PROJECT_ROOT, "data", "poses", "transport", "transport_home_dual.json")
HAND_HOME_Q = [-0.1, 0.05, 0.35, 0.35, 0.35, 0.35]


def _rel(path: str) -> str:
    try:
        return os.path.relpath(path, PROJECT_ROOT)
    except ValueError:
        return path


def _run_step(index: int, total: int, name: str, cmd: list[str], ws_url: str, execute: bool) -> None:
    print(f"\n步骤 {index}/{total} 开始: {name}", flush=True)
    if execute:
        _set_mpc_mode_true(ws_url)
    subprocess.run(cmd, cwd=PROJECT_ROOT, check=True)
    print(f"步骤 {index}/{total} 完成: {name}", flush=True)


def _run_callable_step(index: int, total: int, name: str, fn, ws_url: str, execute: bool) -> None:
    print(f"\n步骤 {index}/{total} 开始: {name}", flush=True)
    if execute:
        _set_mpc_mode_true(ws_url)
        fn()
    else:
        print("    [DRY RUN] 未发送手指命令", flush=True)
    print(f"步骤 {index}/{total} 完成: {name}", flush=True)


def _open_hands(ws_url: str) -> None:
    _call_hand(ws_url, LEFT_HAND_SERVICE, "left", HAND_HOME_Q)
    _call_hand(ws_url, RIGHT_HAND_SERVICE, "right", HAND_HOME_Q)


def _pose_cmd(
    *,
    ws_url: str,
    targets: list[str],
    duration: float,
    max_motion: float,
    execute_delay: float,
    execute: bool,
    use_joints: bool = False,
) -> list[str]:
    cmd = [
        PYTHON,
        "apps/transport/run_dual_pose_path.py",
        "--ws-url",
        ws_url,
        "--max-motion",
        f"{max_motion:.3f}",
        "--duration",
        f"{duration:.3f}",
        "--execute-delay",
        f"{execute_delay:.3f}",
    ]
    for target in targets:
        cmd.extend(["--target", target])
    if use_joints:
        cmd.append("--use-joints")
    if execute:
        cmd.append("--execute")
    return cmd


def main() -> None:
    parser = argparse.ArgumentParser(description="Transport place and return flow.")
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument("--place-target", default=DEFAULT_PLACE)
    parser.add_argument("--outside-target", default=DEFAULT_OUTSIDE)
    parser.add_argument("--pregrasp-target", default=DEFAULT_PREGRASP)
    parser.add_argument("--home-target", default=DEFAULT_HOME)
    parser.add_argument("--place-duration", type=float, default=8.0)
    parser.add_argument("--retreat-duration", type=float, default=5.0)
    parser.add_argument("--return-duration", type=float, default=8.0)
    parser.add_argument("--max-motion", type=float, default=2.0)
    parser.add_argument("--execute-delay", type=float, default=0.0)
    parser.add_argument("--stop-after-place", action="store_true")
    parser.add_argument("--stop-after-outside", action="store_true")
    parser.add_argument("--skip-hand-open", action="store_true", help="Do not open hands after reaching place target.")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    print("=" * 70)
    print("  Transport place and return flow")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  Place target: {_rel(args.place_target)}")
    print(f"  Outside target: {_rel(args.outside_target)}")
    print(f"  Middle target: {_rel(args.pregrasp_target)}")
    print(f"  Home target: {_rel(args.home_target)}")
    print(f"  Execute: {args.execute}")
    print("=" * 70)

    steps: list[tuple[str, list[str] | None]] = [
        (
            "放到 transport_place_dual 实际点位",
            _pose_cmd(
                ws_url=args.ws_url,
                targets=[args.place_target],
                duration=args.place_duration,
                max_motion=args.max_motion,
                execute_delay=args.execute_delay,
                execute=args.execute,
            ),
        )
    ]
    if not args.stop_after_place and not args.skip_hand_open:
        steps.append(("双手复位/松开放箱", None))
    if not args.stop_after_place:
        steps.append(
            (
                "放置点 -> 外扩退开点",
                _pose_cmd(
                    ws_url=args.ws_url,
                    targets=[args.outside_target],
                    duration=args.retreat_duration,
                    max_motion=args.max_motion,
                    execute_delay=args.execute_delay,
                    execute=args.execute,
                ),
            )
        )
    if not args.stop_after_place and not args.stop_after_outside:
        steps.append(
            (
                "外扩点 -> 中间点 -> home",
                _pose_cmd(
                    ws_url=args.ws_url,
                    targets=[args.pregrasp_target, args.home_target],
                    duration=args.return_duration,
                    max_motion=args.max_motion,
                    execute_delay=args.execute_delay,
                    execute=args.execute,
                    use_joints=True,
                ),
            )
        )

    for index, (name, cmd) in enumerate(steps, start=1):
        if cmd is None:
            _run_callable_step(index, len(steps), name, lambda: _open_hands(args.ws_url), args.ws_url, args.execute)
        else:
            _run_step(index, len(steps), name, cmd, args.ws_url, args.execute)

    print("\n[✓] transport 放置/外扩/回 home 流程完成", flush=True)


if __name__ == "__main__":
    main()
