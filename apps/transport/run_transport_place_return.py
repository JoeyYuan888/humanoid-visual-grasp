#!/usr/bin/env python3
"""Place transported box, retreat outward, lift hands away, then return through pregrasp to home."""

from __future__ import annotations

import argparse
import copy
import json
import os
import subprocess
import sys
from pathlib import Path

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
DEFAULT_POST_OPEN_OUTSIDE = os.path.join(PROJECT_ROOT, "data", "runtime", "transport_post_open_outside_latest.json")
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


def _relative_lift_cmd(
    *,
    ws_url: str,
    dz: float,
    duration: float,
    max_motion: float,
    execute_delay: float,
    execute: bool,
) -> list[str]:
    cmd = [
        PYTHON,
        "apps/transport/run_dual_relative_move_with_current_joints.py",
        "--ws-url",
        ws_url,
        "--dx",
        "0.0",
        "--dy",
        "0.0",
        "--dz",
        f"{dz:.3f}",
        "--duration",
        f"{duration:.3f}",
        "--max-motion",
        f"{max(max_motion, abs(dz) + 0.05):.3f}",
        "--execute-delay",
        f"{execute_delay:.3f}",
    ]
    if execute:
        cmd.append("--execute")
    return cmd


def _offset_y(pose: dict, dy: float) -> dict:
    result = copy.deepcopy(pose)
    result["position"]["y"] = float(result["position"]["y"]) + dy
    return result


def _offset_z(pose: dict, dz: float) -> dict:
    result = copy.deepcopy(pose)
    result["position"]["z"] = float(result["position"]["z"]) + dz
    return result


def _write_post_open_outside_target(source_path: str, output_path: str, outside_offset: float, z_offset: float) -> None:
    with open(source_path, "r", encoding="utf-8") as f:
        source = json.load(f)
    if source.get("type") != "dual_arm_mpc_pose":
        raise ValueError(f"不是 dual_arm_mpc_pose 文件: {source_path}")

    left_pose = _offset_y(_offset_z(source["left"]["pose"], z_offset), abs(outside_offset))
    right_pose = _offset_y(_offset_z(source["right"]["pose"], z_offset), -abs(outside_offset))
    payload = {
        "created_from": os.path.relpath(source_path, PROJECT_ROOT),
        "name": "transport_post_open_outside",
        "type": "dual_arm_mpc_pose",
        "frame": "BASE",
        "left": {"arm": "left", "pose": left_pose, "orientation": left_pose["orientation"]},
        "right": {"arm": "right", "pose": right_pose, "orientation": right_pose["orientation"]},
        "params": {"outside_offset": abs(outside_offset), "z_offset": z_offset},
        "note": "Generated for post-open retreat. Moves both hands outward in BASE y before lifting.",
    }
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
        f.write("\n")
    print(f"    [✓] 已生成松手后外扩目标: {_rel(output_path)}", flush=True)
    print(f"        left y={left_pose['position']['y']:.4f}, right y={right_pose['position']['y']:.4f}", flush=True)


def _post_open_outside_cmd(
    *,
    ws_url: str,
    source_path: str,
    output_path: str,
    outside_offset: float,
    z_offset: float,
    duration: float,
    max_motion: float,
    execute_delay: float,
    execute: bool,
) -> list[str]:
    _write_post_open_outside_target(source_path, output_path, outside_offset, z_offset)
    return _pose_cmd(
        ws_url=ws_url,
        targets=[output_path],
        duration=duration,
        max_motion=max(max_motion, abs(outside_offset) + 0.05),
        execute_delay=execute_delay,
        execute=execute,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Transport place and return flow.")
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument("--place-target", default=DEFAULT_PLACE)
    parser.add_argument("--outside-target", default=DEFAULT_OUTSIDE)
    parser.add_argument("--post-open-outside-target", default=DEFAULT_POST_OPEN_OUTSIDE)
    parser.add_argument("--pregrasp-target", default=DEFAULT_PREGRASP)
    parser.add_argument("--home-target", default=DEFAULT_HOME)
    parser.add_argument("--place-duration", type=float, default=8.0)
    parser.add_argument("--post-open-lift", type=float, default=0.12, help="Lift both hands after horizontal outside retreat.")
    parser.add_argument("--post-open-lift-duration", type=float, default=4.0)
    parser.add_argument("--post-open-outside-offset", type=float, default=0.10, help="Move left +Y and right -Y after opening, before lift.")
    parser.add_argument("--post-open-outside-duration", type=float, default=4.0)
    parser.add_argument("--retreat-duration", type=float, default=5.0)
    parser.add_argument("--return-duration", type=float, default=8.0)
    parser.add_argument("--max-motion", type=float, default=2.0)
    parser.add_argument("--execute-delay", type=float, default=0.0)
    parser.add_argument("--stop-after-place", action="store_true")
    parser.add_argument("--stop-after-outside", action="store_true")
    parser.add_argument("--skip-hand-open", action="store_true", help="Do not open hands after reaching place target.")
    parser.add_argument(
        "--use-outside-retreat",
        action="store_true",
        help="Optionally go through latest side-approach outside target. Off by default because this target can be lower than the place pose.",
    )
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    print("=" * 70)
    print("  Transport place and return flow")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  Place target: {_rel(args.place_target)}")
    print(f"  Outside target: {_rel(args.outside_target)}")
    print(f"  Post-open outside target: {_rel(args.post_open_outside_target)}")
    print(f"  Middle target: {_rel(args.pregrasp_target)}")
    print(f"  Home target: {_rel(args.home_target)}")
    print(f"  Post-open lift: {args.post_open_lift:.3f} m")
    print(f"  Post-open outside offset: {args.post_open_outside_offset:.3f} m")
    print(f"  Use outside retreat: {args.use_outside_retreat}")
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
    if not args.stop_after_place and abs(args.post_open_outside_offset) > 1e-9:
        steps.append(
            (
                "松手后双手水平外扩退开",
                _post_open_outside_cmd(
                    ws_url=args.ws_url,
                    source_path=args.place_target,
                    output_path=args.post_open_outside_target,
                    outside_offset=args.post_open_outside_offset,
                    z_offset=0.0,
                    duration=args.post_open_outside_duration,
                    max_motion=args.max_motion,
                    execute_delay=args.execute_delay,
                    execute=args.execute,
                ),
            )
        )
    if not args.stop_after_place and abs(args.post_open_lift) > 1e-9:
        steps.append(
            (
                "外扩后双手上抬避开箱子/桌面",
                _relative_lift_cmd(
                    ws_url=args.ws_url,
                    dz=args.post_open_lift,
                    duration=args.post_open_lift_duration,
                    max_motion=args.max_motion,
                    execute_delay=args.execute_delay,
                    execute=args.execute,
                ),
            )
        )
    if not args.stop_after_place and args.use_outside_retreat:
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
        targets = [args.pregrasp_target, args.home_target]
        steps.append(
            (
                "经中间点 -> home",
                _pose_cmd(
                    ws_url=args.ws_url,
                    targets=targets,
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

    print("\n[✓] transport 放置/外扩/上抬避让/回 home 流程完成", flush=True)


if __name__ == "__main__":
    main()
