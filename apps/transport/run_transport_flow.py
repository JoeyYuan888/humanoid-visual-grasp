#!/usr/bin/env python3
"""Transport approach flow: detect box grasp points, then move to box sides."""

from __future__ import annotations

import argparse
import copy
import json
import os
import subprocess
import sys
import threading
import time
from collections.abc import Callable


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PYTHON = sys.executable
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

try:
    import roslibpy
except ImportError:
    roslibpy = None

from robot_grasp.common import config

DEFAULT_TARGET = os.path.join(PROJECT_ROOT, "data", "runtime", "transport_box_grasp_camera_latest.json")
DEFAULT_BASE_TARGET = os.path.join(PROJECT_ROOT, "data", "runtime", "transport_box_grasp_target_latest.json")
DEFAULT_PREGRASP = os.path.join(PROJECT_ROOT, "data", "poses", "transport", "transport_pregrasp_dual.json")
DEFAULT_SIDE_APPROACH = os.path.join(PROJECT_ROOT, "data", "runtime", "transport_box_side_approach_latest.json")
DEFAULT_CLAMP_TARGET = os.path.join(PROJECT_ROOT, "data", "runtime", "transport_box_clamp_latest.json")
MPC_MODE_SERVICE = "/wa/wa_hardware_interface/mpc_mode_setting"
PRESSURE_TOPIC_TYPE = "hand/PressureSensor"
LEFT_PRESSURE_TOPIC = "/zj_humanoid/hand/finger_pressures/left"
RIGHT_PRESSURE_TOPIC = "/zj_humanoid/hand/finger_pressures/right"
KEY_LINE_PATTERNS = (
    "[✓]",
    "[✗]",
    "[!]",
    "[动作]",
    "[mpc_mode=",
    "[neck]",
    "[MPC] response:",
    "left :",
    "right:",
    "height fallback:",
    "FoundationPose 盒子识别开始",
    "detected box handles",
    "debug images",
    "已锁存盒子 BASE 抓取点",
    "已生成箱子两侧目标",
    "生成双臂路径完成",
    "左臂路径累计长度",
    "右臂路径累计长度",
    "[DRY RUN]",
    "[EXECUTE]",
)


def _rel(path: str) -> str:
    try:
        return os.path.relpath(path, PROJECT_ROOT)
    except ValueError:
        return path


def _load_json(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _build_pose(base_xyz: list[float], orientation: dict, x_offset: float, y_offset: float, z_offset: float) -> dict:
    return {
        "position": {
            "x": float(base_xyz[0]) + x_offset,
            "y": float(base_xyz[1]) + y_offset,
            "z": float(base_xyz[2]) + z_offset,
        },
        "orientation": copy.deepcopy(orientation),
    }


def _write_side_approach_target(
    box_target_path: str,
    pregrasp_path: str,
    output_path: str,
    name: str,
    body_offset: float,
    outside_offset: float,
    z_offset: float,
) -> None:
    box_target = _load_json(box_target_path)
    pregrasp = _load_json(pregrasp_path)
    if box_target.get("type") != "box_grasp_points_base":
        raise ValueError(f"不是 box_grasp_points_base 文件: {box_target_path}")
    if pregrasp.get("type") != "dual_arm_mpc_pose":
        raise ValueError(f"不是 dual_arm_mpc_pose 文件: {pregrasp_path}")

    left_pose = _build_pose(
        box_target["left"]["base"],
        pregrasp["left"]["pose"]["orientation"],
        x_offset=body_offset,
        y_offset=abs(outside_offset),
        z_offset=z_offset,
    )
    right_pose = _build_pose(
        box_target["right"]["base"],
        pregrasp["right"]["pose"]["orientation"],
        x_offset=body_offset,
        y_offset=-abs(outside_offset),
        z_offset=z_offset,
    )
    payload = {
        "created_at": box_target.get("created_at"),
        "name": name,
        "type": "dual_arm_mpc_pose",
        "frame": "BASE",
        "left": {
            "arm": "left",
            "pose": left_pose,
            "orientation": left_pose["orientation"],
        },
        "right": {
            "arm": "right",
            "pose": right_pose,
            "orientation": right_pose["orientation"],
        },
        "params": {
            "body_offset": body_offset,
            "outside_offset": outside_offset,
            "z_offset": z_offset,
            "box_target": box_target_path,
            "pregrasp_file": pregrasp_path,
        },
        "note": "Generated from BASE-locked box grasp points. X body offset moves toward robot body; left uses +Y outside offset; right uses -Y outside offset.",
    }
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
        f.write("\n")
    print(f"[✓] 已生成箱子两侧目标: {_rel(output_path)}")
    print(f"    body_offset={body_offset:.3f}m, outside_offset={outside_offset:.3f}m, z_offset={z_offset:.3f}m")
    print(
        f"    left : x={left_pose['position']['x']:.4f}, "
        f"y={left_pose['position']['y']:.4f}, z={left_pose['position']['z']:.4f}"
    )
    print(
        f"    right: x={right_pose['position']['x']:.4f}, "
        f"y={right_pose['position']['y']:.4f}, z={right_pose['position']['z']:.4f}"
    )


StepAction = list[str] | Callable[[], None]
Step = tuple[str, StepAction, bool]


def _pressure_stats(pressure: list[float] | None) -> tuple[float, float, float]:
    if not pressure:
        return 0.0, 0.0, 0.0
    positive = [max(0.0, float(value)) for value in pressure]
    absolute = [abs(float(value)) for value in pressure]
    return max(absolute) if absolute else 0.0, max(positive) if positive else 0.0, sum(positive)


class DualPressureClient:
    def __init__(self, ws_url: str):
        self.ws_url = ws_url
        self.client = None
        self.topics = []
        self.latest = {"left": None, "right": None}
        self.latest_stamp = {"left": 0.0, "right": 0.0}
        self.lock = threading.Lock()

    def connect(self) -> None:
        self.client = _connect(self.ws_url)
        self.topics = [
            roslibpy.Topic(self.client, LEFT_PRESSURE_TOPIC, PRESSURE_TOPIC_TYPE),
            roslibpy.Topic(self.client, RIGHT_PRESSURE_TOPIC, PRESSURE_TOPIC_TYPE),
        ]
        self.topics[0].subscribe(lambda msg: self._callback("left", msg))
        self.topics[1].subscribe(lambda msg: self._callback("right", msg))
        print("    [动作] 已订阅左右手指尖压力", flush=True)

    def _callback(self, side: str, message: dict) -> None:
        pressure = message.get("pressure")
        if pressure is None:
            return
        with self.lock:
            self.latest[side] = [float(value) for value in pressure]
            self.latest_stamp[side] = time.time()

    def read(self, timeout: float) -> dict[str, list[float] | None]:
        start = time.time()
        while time.time() - start < timeout:
            with self.lock:
                if self.latest["left"] is not None and self.latest["right"] is not None:
                    return {
                        "left": list(self.latest["left"]),
                        "right": list(self.latest["right"]),
                    }
            time.sleep(0.05)
        with self.lock:
            return {
                "left": list(self.latest["left"]) if self.latest["left"] is not None else None,
                "right": list(self.latest["right"]) if self.latest["right"] is not None else None,
            }

    def close(self) -> None:
        for topic in self.topics:
            try:
                topic.unsubscribe()
            except Exception:
                pass
        _safe_close(self.client)
        self.client = None
        self.topics = []


def _parse_ws_url(ws_url: str) -> tuple[str, int]:
    stripped = ws_url.replace("ws://", "").replace("wss://", "")
    host, port = stripped.split(":")
    return host, int(port)


def _connect(ws_url: str):
    if roslibpy is None:
        raise RuntimeError("缺少 roslibpy，无法设置 MPC mode")
    host, port = _parse_ws_url(ws_url)
    client = roslibpy.Ros(host=host, port=port)
    thread = threading.Thread(target=client.run, daemon=True)
    thread.start()
    start = time.time()
    while not client.is_connected:
        if time.time() - start > config.CONNECT_TIMEOUT:
            raise RuntimeError(f"连接超时: {ws_url}")
        time.sleep(0.1)
    return client


def _call(client, name: str, service_type: str, request: dict | None = None) -> dict:
    service = roslibpy.Service(client, name, service_type)
    return service.call(roslibpy.ServiceRequest(request or {}))


def _service_type(client, name: str) -> str:
    try:
        return _call(client, "/rosapi/service_type", "rosapi/ServiceType", {"service": name}).get("type", "")
    except Exception:
        return ""


def _safe_close(client) -> None:
    if client is None:
        return
    try:
        if getattr(client, "is_connected", False):
            client.close(timeout=1.0)
    except Exception:
        pass


def _set_mpc_mode_true(ws_url: str) -> None:
    print("    [动作] 设置 MPC running mode 开始", flush=True)
    client = _connect(ws_url)
    try:
        srv_type = _service_type(client, MPC_MODE_SERVICE)
        if not srv_type:
            raise RuntimeError(f"找不到 MPC mode 服务: {MPC_MODE_SERVICE}")
        response = _call(client, MPC_MODE_SERVICE, srv_type, {"data": True})
        print(f"    [mpc_mode=True] {response}", flush=True)
        if response and response.get("success") is False:
            raise RuntimeError(f"MPC mode 设置失败: {response}")
    finally:
        _safe_close(client)
    print("    [动作] 设置 MPC running mode 完成", flush=True)


def _run_child_stream(cmd: list[str]) -> tuple[int, str]:
    process = subprocess.Popen(
        cmd,
        cwd=PROJECT_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    captured: list[str] = []
    assert process.stdout is not None
    for line in process.stdout:
        captured.append(line)
        if any(pattern in line for pattern in KEY_LINE_PATTERNS):
            print("    " + line.rstrip(), flush=True)
    return process.wait(), "".join(captured)


def _run_step(
    index: int,
    total: int,
    name: str,
    action: StepAction,
    *,
    ws_url: str,
    ensure_mpc_mode: bool = False,
    execute: bool = True,
) -> None:
    print(f"\n步骤 {index}/{total} 开始: {name}", flush=True)
    try:
        if ensure_mpc_mode and execute:
            _set_mpc_mode_true(ws_url)
        if callable(action):
            action()
        else:
            returncode, output = _run_child_stream(action)
            if returncode != 0:
                print(f"步骤 {index}/{total} 失败: {name} - exit={returncode}", flush=True)
                tail = output.splitlines()[-40:]
                if tail:
                    print("[子进程输出尾部]", flush=True)
                    print("\n".join(tail), flush=True)
                raise subprocess.CalledProcessError(returncode, action, output=output)
    except Exception as exc:
        if not isinstance(exc, subprocess.CalledProcessError):
            print(f"步骤 {index}/{total} 失败: {name} - {exc}", flush=True)
        raise
    print(f"步骤 {index}/{total} 完成: {name}", flush=True)


def _parse_clamp_offsets(value: str) -> list[float]:
    offsets = []
    for item in value.split(","):
        item = item.strip()
        if not item:
            continue
        offsets.append(abs(float(item)))
    if not offsets:
        raise ValueError("--clamp-offsets 不能为空")
    return offsets


def _clamp_offsets(outside_offset: float, clamp_offset: float, step: float, explicit_offsets: str) -> list[float]:
    if explicit_offsets:
        offsets = _parse_clamp_offsets(explicit_offsets)
        outside = abs(float(outside_offset))
        return [offset for offset in offsets if offset < outside + 1e-9]

    outside = abs(float(outside_offset))
    clamp = abs(float(clamp_offset))
    step = abs(float(step))
    if step <= 0:
        raise ValueError("--clamp-step must be > 0")
    if outside <= clamp:
        return [clamp]
    offsets = []
    current = outside
    while current - step > clamp:
        current -= step
        offsets.append(round(current, 4))
    offsets.append(round(clamp, 4))
    return offsets


def _pressure_reached(
    pressure_client: DualPressureClient,
    left_threshold: float,
    right_threshold: float,
    timeout: float,
) -> bool:
    pressures = pressure_client.read(timeout=timeout)
    left_abs, left_pos, left_sum = _pressure_stats(pressures.get("left"))
    right_abs, right_pos, right_sum = _pressure_stats(pressures.get("right"))
    print(
        f"    [pressure] left abs={left_abs:.3f} pos={left_pos:.3f} sum_pos={left_sum:.3f}; "
        f"right abs={right_abs:.3f} pos={right_pos:.3f} sum_pos={right_sum:.3f}",
        flush=True,
    )
    return left_abs >= left_threshold and right_abs >= right_threshold


def _run_clamp_sequence(args, side_motion_duration: float, execute: bool) -> None:
    offsets = _clamp_offsets(args.outside_offset, args.clamp_offset, args.clamp_step, args.clamp_offsets)
    print(f"    [动作] 分段夹紧 offset 序列: {', '.join(f'{value:.3f}' for value in offsets)}", flush=True)
    if not execute:
        return

    pressure_client = None
    if not args.disable_clamp_pressure:
        pressure_client = DualPressureClient(args.ws_url)
        pressure_client.connect()
    try:
        for offset in offsets:
            _write_side_approach_target(
                args.base_target_output,
                args.pregrasp_file,
                args.clamp_output,
                "transport_box_clamp",
                args.body_offset,
                offset,
                args.side_z_offset,
            )
            _set_mpc_mode_true(args.ws_url)
            cmd = [
                PYTHON,
                "apps/transport/run_dual_pose_path.py",
                "--ws-url",
                args.ws_url,
                "--target",
                args.clamp_output,
                "--max-motion",
                f"{args.max_motion:.3f}",
                "--duration",
                f"{args.clamp_step_duration:.3f}",
                "--execute-delay",
                f"{args.clamp_execute_delay:.3f}",
                "--execute",
            ]
            print(f"    [动作] 夹紧到 outside_offset={offset:.3f}m 开始", flush=True)
            returncode, output = _run_child_stream(cmd)
            if returncode != 0:
                tail = output.splitlines()[-40:]
                if tail:
                    print("[子进程输出尾部]", flush=True)
                    print("\n".join(tail), flush=True)
                raise subprocess.CalledProcessError(returncode, cmd, output=output)
            time.sleep(max(0.0, args.clamp_step_duration + args.clamp_pressure_settle_sec))
            if pressure_client is not None and _pressure_reached(
                pressure_client,
                args.clamp_left_pressure_threshold,
                args.clamp_right_pressure_threshold,
                args.clamp_pressure_timeout,
            ):
                print(f"    [✓] 压力达标，停止继续收紧 offset={offset:.3f}m", flush=True)
                return
            print(f"    [动作] offset={offset:.3f}m 压力未达标，继续收紧", flush=True)
        print("    [!] 已到最终夹紧 offset，压力仍未达到阈值", flush=True)
    finally:
        if pressure_client is not None:
            pressure_client.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Transport approach flow: low-head box detection -> pregrasp -> box side approach.",
    )
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument(
        "--target-output",
        default=DEFAULT_TARGET,
        help="Camera-frame detection JSON saved under data/runtime.",
    )
    parser.add_argument("--base-target-output", default=DEFAULT_BASE_TARGET)
    parser.add_argument("--pregrasp-file", default=DEFAULT_PREGRASP)
    parser.add_argument("--side-approach-output", default=DEFAULT_SIDE_APPROACH)
    parser.add_argument("--clamp-output", default=DEFAULT_CLAMP_TARGET)
    parser.add_argument("--backend", choices=["foundationpose", "fastsam", "color", "both"], default="foundationpose")
    parser.add_argument("--geometry", choices=["auto", "outer", "inner", "depth-rim"], default="depth-rim")
    parser.add_argument("--rim-fit-mode", choices=["free", "front-parallel", "side-mid"], default="side-mid")
    parser.add_argument("--show-window", action="store_true", help="Show box detection result window.")
    parser.add_argument("--skip-detect", action="store_true", help="Skip box detection and reuse latest target JSON.")
    parser.add_argument("--skip-lock-base", action="store_true", help="Skip camera->BASE conversion.")
    parser.add_argument("--skip-motion", action="store_true", help="Skip all arm motion after detection/lock.")
    parser.add_argument("--skip-side-approach", action="store_true", help="Stop after transport_pregrasp_dual.")
    parser.add_argument("--skip-clamp", action="store_true", help="Stop after the outer side approach point.")
    parser.add_argument("--skip-neck-down", action="store_true")
    parser.add_argument("--skip-neck-home", action="store_true")
    parser.add_argument("--neck-down-y", type=float, default=0.35)
    parser.add_argument("--neck-time", type=float, default=4.0)
    parser.add_argument("--tf-seconds", type=float, default=2.0)
    parser.add_argument("--motion-duration", type=float, default=5.0)
    parser.add_argument(
        "--side-motion-duration",
        type=float,
        default=None,
        help="Duration for pregrasp -> side approach. Default is motion-duration * 2, about half speed.",
    )
    parser.add_argument("--execute-delay", type=float, default=2.0)
    parser.add_argument("--max-motion", type=float, default=2.0)
    parser.add_argument(
        "--body-offset",
        type=float,
        default=-0.25,
        help="X offset for side approach in BASE frame. Negative moves toward robot body.",
    )
    parser.add_argument("--outside-offset", type=float, default=0.10, help="Outer wait offset in meters.")
    parser.add_argument("--clamp-offset", type=float, default=0.02, help="Final clamp offset in meters.")
    parser.add_argument("--clamp-step", type=float, default=0.02, help="Clamp inward step in meters when --clamp-offsets is empty.")
    parser.add_argument(
        "--clamp-offsets",
        default="0.08,0.04,0.03,0.02",
        help="Comma separated clamp offset sequence. Default: 8cm -> 4cm -> 3cm -> 2cm.",
    )
    parser.add_argument("--clamp-step-duration", type=float, default=3.0, help="MPC duration for each clamp step.")
    parser.add_argument("--clamp-execute-delay", type=float, default=0.0, help="Execute delay for each clamp step.")
    parser.add_argument("--disable-clamp-pressure", action="store_true")
    parser.add_argument("--clamp-left-pressure-threshold", type=float, default=0.15)
    parser.add_argument("--clamp-right-pressure-threshold", type=float, default=0.15)
    parser.add_argument("--clamp-pressure-timeout", type=float, default=1.0)
    parser.add_argument("--clamp-pressure-settle-sec", type=float, default=0.2)
    parser.add_argument(
        "--side-z-offset",
        type=float,
        default=0.05,
        help="Additional Z offset for side approach after lowering 0.30m from plastic-bag TCP z offset.",
    )
    parser.add_argument("--execute", action="store_true", help="Actually execute arm motions.")
    args = parser.parse_args()
    side_motion_duration = args.side_motion_duration
    if side_motion_duration is None:
        side_motion_duration = args.motion_duration * 2.0

    print("=" * 70)
    print("  Transport approach flow")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  Box camera temp: {args.target_output}")
    print(f"  Box BASE target output: {_rel(args.base_target_output)}")
    print(f"  Pregrasp: {_rel(args.pregrasp_file)}")
    print(f"  Side approach: {_rel(args.side_approach_output)}")
    print(f"  Clamp target: {_rel(args.clamp_output)}")
    print(f"  Body offset: {args.body_offset:.3f} m")
    print(f"  Outside offset: {args.outside_offset:.3f} m")
    print(f"  Clamp offset: {args.clamp_offset:.3f} m")
    print(f"  Clamp step: {args.clamp_step:.3f} m")
    print(f"  Clamp offsets: {args.clamp_offsets}")
    print(
        f"  Clamp pressure threshold: "
        f"L={args.clamp_left_pressure_threshold:.3f}, R={args.clamp_right_pressure_threshold:.3f}"
    )
    print(f"  Side z offset: {args.side_z_offset:.3f} m")
    print(f"  Motion duration: pregrasp={args.motion_duration:.1f}s side={side_motion_duration:.1f}s")
    print(f"  Execute motion: {args.execute}")
    print("=" * 70)

    steps: list[Step] = []
    if not args.skip_detect:
        if args.backend == "foundationpose":
            detect_cmd = [
                PYTHON,
                "apps/transport/run_foundationpose_box_grasp_point.py",
                "--ws-url",
                args.ws_url,
                "--neck-down-y",
                f"{args.neck_down_y:.3f}",
                "--neck-time",
                f"{args.neck_time:.3f}",
                "--output",
                args.target_output,
            ]
        else:
            detect_cmd = [
                PYTHON,
                "apps/transport/run_box_grasp_point.py",
                "--ws-url",
                args.ws_url,
                "--backend",
                args.backend,
                "--geometry",
                args.geometry,
                "--rim-fit-mode",
                args.rim_fit_mode,
                "--neck-down-y",
                f"{args.neck_down_y:.3f}",
                "--neck-time",
                f"{args.neck_time:.3f}",
                "--output",
                args.target_output,
            ]
        if args.show_window:
            detect_cmd.append("--show-window")
        if args.skip_neck_down:
            detect_cmd.append("--skip-neck-down")
        if args.skip_neck_home:
            detect_cmd.append("--skip-neck-home")
        steps.append(("低头、识别盒子抓取点、保存结果、抬头", detect_cmd, False))

    if not args.skip_lock_base:
        lock_cmd = [
            PYTHON,
            "apps/transport/lock_box_grasp_target.py",
            "--ws-url",
            args.ws_url,
            "--input",
            args.target_output,
            "--output",
            args.base_target_output,
            "--tf-seconds",
            f"{args.tf_seconds:.3f}",
        ]
        steps.append(("相机系左右抓取点 -> BASE 锁存", lock_cmd, False))

    if not args.skip_motion:
        motion_cmd = [
            PYTHON,
            "apps/transport/run_dual_pose_path.py",
            "--ws-url",
            args.ws_url,
            "--target",
            args.pregrasp_file,
            "--use-joints",
            "--max-motion",
            f"{args.max_motion:.3f}",
            "--duration",
            f"{args.motion_duration:.3f}",
            "--execute-delay",
            f"{args.execute_delay:.3f}",
        ]
        if args.execute:
            motion_cmd.append("--execute")
        steps.append(("home -> transport_pregrasp_dual", motion_cmd, True))

        if not args.skip_side_approach:
            steps.append(
                (
                    "生成箱子两侧外扩目标",
                    lambda: _write_side_approach_target(
                        args.base_target_output,
                        args.pregrasp_file,
                        args.side_approach_output,
                        "transport_box_side_approach",
                        args.body_offset,
                        args.outside_offset,
                        args.side_z_offset,
                    ),
                    False,
                )
            )
            side_motion_cmd = [
                PYTHON,
                "apps/transport/run_dual_pose_path.py",
                "--ws-url",
                args.ws_url,
                "--target",
                args.side_approach_output,
                "--max-motion",
                f"{args.max_motion:.3f}",
                "--duration",
                f"{side_motion_duration:.3f}",
                "--execute-delay",
                f"{args.execute_delay:.3f}",
            ]
            if args.execute:
                side_motion_cmd.append("--execute")
            steps.append(("transport_pregrasp_dual -> 箱子两侧外扩点", side_motion_cmd, True))

            if not args.skip_clamp:
                steps.append(
                    (
                        "分段夹紧并检测左右手压力",
                        lambda: _run_clamp_sequence(args, side_motion_duration, args.execute),
                        False,
                    )
                )

    if not steps:
        print("[!] 没有需要执行的步骤")
        return

    for idx, (name, action, ensure_mpc_mode) in enumerate(steps, start=1):
        _run_step(
            idx,
            len(steps),
            name,
            action,
            ws_url=args.ws_url,
            ensure_mpc_mode=ensure_mpc_mode,
            execute=args.execute,
        )

    print("\n[✓] transport 抓取靠近/夹紧流程完成")


if __name__ == "__main__":
    main()
