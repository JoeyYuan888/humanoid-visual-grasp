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

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot_grasp.common import config
from robot_grasp.hand.hand_utils import HAND_CLOSE, HAND_OPEN, clamp_hand_q

try:
    import roslibpy
except ImportError:
    print("缺少 roslibpy 库，请运行: pip install roslibpy")
    sys.exit(1)


HAND_SERVICE = "/zj_humanoid/hand/joint_switch/right"
HAND_SERVICE_TYPE = "hand_controller/JointSwitch"
PRESSURE_TOPIC = "/zj_humanoid/hand/finger_pressures/right"
PRESSURE_TOPIC_TYPE = "hand/PressureSensor"
_STEP_INDEX = 0
_STEP_TOTAL = 0
_CURRENT_QR_OUTPUT = os.path.join("data", "runtime", "post_grasp_qr_latest.json")
_KEY_LINE_PATTERNS = (
    "[阶段]",
    "[动作]",
    "选择目标:",
    "已锁存 BASE 目标",
    "base  :",
    "object base ",
    "pregrasp",
    "完整路径累计长度",
    "末端到目标直线距离",
    "[MPC] response:",
    "[QR]",
    "[OCR]",
    "source:",
    "output_dir:",
    "frames:",
    "normalized:",
    "accepted:",
    "report:",
    "已保存 QR raw 快照目录",
    "已保存 QR",
    "已保存 OCR",
    "OCR 未通过",
    "已保存:",
    "[✗]",
    "[!]",
)

_QR_DETAIL_PATTERNS = (
    "批量获取 QR 快照完成",
    "[OCR]",
    "[QR]",
    "未识别到 QR",
    "OCR 快照",
    "PP-OCRv4 快照",
    "QR 快照",
    "小模型 OCR 未通过",
    "PP-OCRv4 fallback",
    "轻量 QR fallback",
)


def _log(message: str = ""):
    print(message, flush=True)


def _safe_ros_close(client) -> None:
    if client is None:
        return
    try:
        if getattr(client, "is_connected", False):
            client.close(timeout=1.0)
    except Exception:
        pass


def _load_qr_result(path: str) -> dict | None:
    if not path or not os.path.exists(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            result = json.load(f)
    except Exception:
        return None
    return result if isinstance(result, dict) else None


def _step_start(name: str) -> int:
    global _STEP_INDEX
    _STEP_INDEX += 1
    if _STEP_TOTAL > 0:
        _log(f"\n步骤 {_STEP_INDEX}/{_STEP_TOTAL} 开始: {name}")
    else:
        _log(f"\n步骤 {_STEP_INDEX} 开始: {name}")
    return _STEP_INDEX


def _step_done(step_no: int, name: str):
    if _STEP_TOTAL > 0:
        _log(f"步骤 {step_no}/{_STEP_TOTAL} 完成: {name}")
    else:
        _log(f"步骤 {step_no} 完成: {name}")


def _step_failed(step_no: int, name: str, detail: str):
    if _STEP_TOTAL > 0:
        _log(f"步骤 {step_no}/{_STEP_TOTAL} 失败: {name} - {detail}")
    else:
        _log(f"步骤 {step_no} 失败: {name} - {detail}")


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
                _log("[✓] 手掌接口已连接")
                return
            except Exception as exc:
                last_exc = exc
                _log(f"[!] hand client 连接失败 {attempt}/{retries}: {exc}")
                self.close()
                if attempt < retries:
                    time.sleep(retry_delay)
        raise RuntimeError(f"hand client connect failed after {retries} attempts: {last_exc}")

    def call(self, q: list[float], retries: int = 2, retry_delay: float = 1.0):
        q = clamp_hand_q(q)
        last_exc = None
        for attempt in range(1, retries + 1):
            try:
                if self.client is None or self.service is None or not self.client.is_connected:
                    self.connect()
                response = self.service.call(roslibpy.ServiceRequest({"q": q}))
                if response and response.get("success") is False:
                    raise RuntimeError(f"hand joint_switch failed: {response}")
                _log("[hand] success")
                return
            except Exception as exc:
                last_exc = exc
                _log(f"[!] hand 调用失败 {attempt}/{retries}: {exc}")
                self.close()
                if attempt < retries:
                    time.sleep(retry_delay)
        raise RuntimeError(f"hand joint_switch failed after {retries} attempts: {last_exc}")

    def close(self):
        if self.client is None:
            return
        _safe_ros_close(self.client)
        self.client = None
        self.service = None


class PressureClient:
    def __init__(self, ws_url: str):
        self.ws_url = ws_url
        self.client = None
        self.topic = None
        self.latest_pressure = None
        self.latest_stamp = 0.0
        self.lock = threading.Lock()

    def connect(self):
        self.client = _connect(self.ws_url, timeout=10.0)
        self.topic = roslibpy.Topic(self.client, PRESSURE_TOPIC, PRESSURE_TOPIC_TYPE)
        self.topic.subscribe(self._callback)
        _log("[✓] 压力话题已订阅")

    def _callback(self, message: dict):
        pressure = message.get("pressure")
        if pressure is None:
            return
        with self.lock:
            self.latest_pressure = [float(value) for value in pressure]
            self.latest_stamp = time.time()

    def clear(self):
        with self.lock:
            self.latest_pressure = None
            self.latest_stamp = 0.0

    def read(self, timeout: float = 2.0) -> list[float] | None:
        start = time.time()
        while time.time() - start <= timeout:
            with self.lock:
                if self.latest_pressure is not None:
                    return list(self.latest_pressure)
            time.sleep(0.05)
        return None

    def close(self):
        if self.topic is not None:
            try:
                self.topic.unsubscribe()
            except Exception:
                pass
        self.topic = None
        if self.client is not None:
            _safe_ros_close(self.client)
        self.client = None


def _extract_key_lines(output: str) -> list[str]:
    lines = []
    for raw_line in output.splitlines():
        line = raw_line.strip()
        if any(pattern in line for pattern in _KEY_LINE_PATTERNS):
            lines.append(line)
    return lines[-2:]


def _print_failure_output(result: subprocess.CompletedProcess):
    output = "\n".join(part for part in (result.stdout, result.stderr) if part)
    tail = output.splitlines()[-40:]
    if tail:
        _log("[子进程输出尾部]")
        _log("\n".join(tail))


def _run_child_stream(cmd: list[str]) -> tuple[int, str]:
    process = subprocess.Popen(
        cmd,
        cwd=PROJECT_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=1,
    )
    captured = []
    assert process.stdout is not None
    for raw_line in process.stdout:
        captured.append(raw_line)
        line = raw_line.strip()
        if any(pattern in line for pattern in _KEY_LINE_PATTERNS):
            _log(f"    {line}")
    return process.wait(), "".join(captured)


def _start_qr_scan_background(name: str, cmd: list[str], execute: bool):
    step_no = _step_start(name)
    if not execute:
        _log("    " + " ".join(cmd))
        _step_done(step_no, name)
        return None

    process = subprocess.Popen(
        cmd,
        cwd=PROJECT_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=1,
    )
    captured = []
    printed_qr_lines = set()
    assert process.stdout is not None

    for raw_line in process.stdout:
        captured.append(raw_line)
        line = raw_line.strip()
        if any(pattern in line for pattern in _QR_DETAIL_PATTERNS):
            _log(f"    {line}")
            printed_qr_lines.add(line)
        if "批量获取 QR 快照完成" in line:
            break

    if process.poll() is not None:
        returncode = process.wait()
        if returncode != 0:
            _step_failed(step_no, name, f"exit={returncode}")
            raise subprocess.CalledProcessError(returncode, cmd, output="".join(captured))
        _step_done(step_no, name)
        return None

    _log("    [动作] QR/OCR 快照采集完成，后台识别中")
    _step_done(step_no, f"{name} 快照完成，后台解码中")

    def drain():
        assert process.stdout is not None
        for raw_line in process.stdout:
            captured.append(raw_line)
            line = raw_line.strip()
            if any(pattern in line for pattern in _QR_DETAIL_PATTERNS) and line not in printed_qr_lines:
                printed_qr_lines.add(line)
                _log(f"    {line}")

    thread = threading.Thread(target=drain, daemon=True)
    thread.start()
    return {
        "name": name,
        "cmd": cmd,
        "process": process,
        "thread": thread,
        "captured": captured,
        "printed_qr_lines": printed_qr_lines,
    }


def _print_qr_scan_summary(captured: list[str]) -> None:
    lines = [line.strip() for line in captured if line.strip()]
    result = _load_qr_result(_CURRENT_QR_OUTPUT)
    if result:
        mode = str(result.get("mode", "") or "").strip()
        texts = [str(text).strip() for text in result.get("texts", []) if str(text).strip()]
        if texts:
            label = "OCR" if mode.startswith("ocr") else "QR"
            unique_texts = list(dict.fromkeys(texts))
            _log(f"    [{label} 结果] {'; '.join(unique_texts)}")
        else:
            _log("    [!] 后台扫码结果文件存在，但没有有效文本")
    save_lines = [
        line for line in lines
        if "已保存 OCR 结果" in line or "已保存 QR 结果" in line or "已保存 QR raw 快照目录" in line
    ]
    if not result and not any(any(pattern in line for pattern in _QR_DETAIL_PATTERNS) for line in lines):
        _log("    [!] 后台扫码未输出识别结果")
    if save_lines:
        _log(f"    {save_lines[-1]}")


def _finish_qr_scan_background(handle):
    if handle is None:
        return
    name = handle["name"]
    process = handle["process"]
    thread = handle["thread"]
    step_no = _step_start(f"等待后台扫码完成: {name}")
    try:
        returncode = process.wait(timeout=5.0)
        completed_output = "抓后识别完成" in "".join(handle["captured"])
    except subprocess.TimeoutExpired:
        captured_text = "".join(handle["captured"])
        if "抓后识别完成" in captured_text:
            _log("    [动作] 后台扫码已完成输出，清理未退出子进程")
            completed_output = True
        else:
            _log("    [!] 后台扫码超时，清理子进程；机器人流程已完成")
            completed_output = False
        process.terminate()
        try:
            returncode = process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            returncode = process.wait()
    thread.join(timeout=1.0)
    if completed_output and returncode in (-15, -9):
        _print_qr_scan_summary(handle["captured"])
        _step_done(step_no, f"等待后台扫码完成: {name}")
        return
    if returncode in (-15, -9):
        _print_qr_scan_summary(handle["captured"])
        _step_done(step_no, f"等待后台扫码完成: {name}")
        return
    if returncode != 0:
        _step_failed(step_no, f"等待后台扫码完成: {name}", f"exit={returncode}")
        tail = "".join(handle["captured"]).splitlines()[-40:]
        if tail:
            _log("[扫码子进程输出尾部]")
            _log("\n".join(tail))
        raise subprocess.CalledProcessError(returncode, handle["cmd"], output="".join(handle["captured"]))
    _print_qr_scan_summary(handle["captured"])
    _step_done(step_no, f"等待后台扫码完成: {name}")


def _run_step(
    name: str,
    cmd: list[str],
    execute: bool,
    verbose: bool = False,
    allowed_returncodes: tuple[int, ...] = (0,),
):
    step_no = _step_start(name)
    if not execute:
        _log("    " + " ".join(cmd))
        _step_done(step_no, name)
        return
    if verbose:
        subprocess.run(cmd, cwd=PROJECT_ROOT, check=True)
        _step_done(step_no, name)
        return

    returncode, output = _run_child_stream(cmd)
    if returncode not in allowed_returncodes:
        _step_failed(step_no, name, f"exit={returncode}")
        tail = output.splitlines()[-40:]
        if tail:
            _log("[子进程输出尾部]")
            _log("\n".join(tail))
        raise subprocess.CalledProcessError(
            returncode,
            cmd,
            output=output,
        )
    _step_done(step_no, name)


def _append_execute_flags(cmd: list[str], execute: bool) -> list[str]:
    if execute:
        return [*cmd, "--execute", "--confirm-target"]
    return cmd


def _via_cmd(args, via_files: list[str], max_motion: float, use_joints: bool = True) -> list[str]:
    cmd = [
        sys.executable,
        "-u",
        "apps/grasp/run_visual_grasp_test.py",
        "--ws-url",
        args.ws_url,
        "--connect-retries",
        str(args.connect_retries),
        "--connect-retry-delay",
        f"{args.connect_retry_delay:.1f}",
        "--use-locked-target",
        args.locked_target,
    ]
    for path in via_files:
        cmd.extend(["--via-file", path])
    cmd.extend(["--stop-at-last-via"])
    if use_joints:
        cmd.append("--use-joints")
    cmd.extend([
        "--max-motion", f"{max_motion:.3f}",
        "--max-z", f"{args.max_z:.3f}",
        "--duration", f"{args.motion_duration:.3f}",
        "--execute-delay", f"{args.execute_delay:.3f}",
    ])
    if not args.verbose_children and not args.qr_verbose:
        cmd.append("--quiet")
    return _append_execute_flags(cmd, args.execute)


def _descend_cmd(args, grasp_profile: str | None = None, include_descend: bool = True) -> list[str]:
    grasp_profile = grasp_profile if grasp_profile is not None else args.grasp_profile
    cmd = [
        sys.executable,
        "-u",
        "apps/grasp/run_visual_grasp_test.py",
        "--ws-url",
        args.ws_url,
        "--connect-retries",
        str(args.connect_retries),
        "--connect-retry-delay",
        f"{args.connect_retry_delay:.1f}",
        "--use-locked-target",
        args.locked_target,
        "--grasp-profile",
        grasp_profile,
        "--above-object-height",
        f"{args.above_object_height:.3f}",
        "--grasp-height",
        f"{args.grasp_height:.3f}",
        "--max-motion",
        f"{args.descend_max_motion:.3f}",
        "--max-z",
        f"{args.max_z:.3f}",
        "--duration",
        f"{args.motion_duration:.3f}",
        "--execute-delay",
        f"{args.execute_delay:.3f}",
    ]
    if include_descend:
        cmd.append("--include-descend")
    if not args.verbose_children:
        cmd.append("--quiet")
    return _append_execute_flags(cmd, args.execute)


def _approach_and_descend_cmd(args, via_files: list[str]) -> list[str]:
    cmd = [
        sys.executable,
        "-u",
        "apps/grasp/run_visual_grasp_test.py",
        "--ws-url",
        args.ws_url,
        "--connect-retries",
        str(args.connect_retries),
        "--connect-retry-delay",
        f"{args.connect_retry_delay:.1f}",
        "--use-locked-target",
        args.locked_target,
    ]
    for path in via_files:
        cmd.extend(["--via-file", path])
    cmd.extend([
        "--include-descend",
        "--grasp-profile", args.grasp_profile,
        "--above-object-height", f"{args.above_object_height:.3f}",
        "--grasp-height", f"{args.grasp_height:.3f}",
        "--use-joints",
        "--max-motion", f"{args.combined_max_motion:.3f}",
        "--max-z", f"{args.max_z:.3f}",
        "--duration", f"{args.motion_duration:.3f}",
        "--execute-delay", f"{args.execute_delay:.3f}",
    ])
    if not args.verbose_children:
        cmd.append("--quiet")
    return _append_execute_flags(cmd, args.execute)


def _lock_cmd(args) -> list[str]:
    cmd = [
        sys.executable,
        "-u",
        "apps/grasp/run_perception_lock.py",
        "--ws-url",
        args.ws_url,
        "--detect-seconds",
        f"{args.detect_seconds:.1f}",
        "--frame-timeout",
        f"{args.frame_timeout:.1f}",
        "--min-lock-hits",
        str(args.min_lock_hits),
        "--lock-match-distance",
        f"{args.lock_match_distance:.3f}",
        "--lock-max-missed",
        str(args.lock_max_missed),
        "--lock-target-policy",
        args.lock_target_policy,
        "--highlight-suppression",
        args.highlight_suppression,
        "--output",
        args.locked_target,
    ]
    if args.show_window:
        cmd.append("--show-window")
    else:
        cmd.append("--no-show-window")
    if args.allow_cpu_detect:
        cmd.append("--allow-cpu-detect")
    if not args.verbose_children:
        cmd.append("--quiet")
    return cmd


def _qr_scan_cmd(args) -> list[str]:
    cmd = [
        sys.executable,
        "-u",
        "apps/grasp/run_post_grasp_qr_scan.py",
        "--ws-url",
        args.ws_url,
        "--transport",
        args.qr_transport,
        "--raw-throttle-ms",
        str(args.qr_raw_throttle_ms),
        "--compressed-throttle-ms",
        str(args.qr_compressed_throttle_ms),
        "--duration",
        f"{args.qr_scan_seconds:.1f}",
        "--snapshot-attempts",
        str(args.qr_snapshot_attempts),
        "--snapshot-interval",
        f"{args.qr_snapshot_interval:.3f}",
        "--save-raw-frames",
        args.qr_raw_frame_dir,
        "--recover-output-dir",
        args.qr_recover_output_dir,
        "--recover-min-consensus",
        str(args.qr_recover_min_consensus),
        "--output",
        args.qr_output,
    ]
    if args.auto_recover_qr_offline:
        cmd.append("--auto-recover-offline")
    else:
        cmd.append("--no-auto-recover-offline")
    if args.show_qr_window:
        cmd.append("--show-window")
    else:
        cmd.append("--no-show-window")
    if not args.verbose_children:
        cmd.append("--quiet")
    return cmd


def _load_locked_target(path: str) -> dict | None:
    if not os.path.exists(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _pressure_stats(pressure: list[float]) -> tuple[float, float]:
    positive = [max(0.0, value) for value in pressure]
    return max(positive) if positive else 0.0, sum(positive)


def _check_pressure(args, pressure_client: PressureClient | None, name: str, threshold: float) -> bool:
    step_no = _step_start(f"压力检查: {name}")
    _log(f"    threshold={threshold:.3f}")
    if not args.execute:
        _step_done(step_no, f"压力检查: {name}")
        return True
    if args.disable_pressure_checks:
        _log("    [pressure] disabled")
        _step_done(step_no, f"压力检查: {name}")
        return True
    if pressure_client is None:
        raise RuntimeError("pressure client is not initialized")

    pressure_client.clear()
    if args.pressure_settle_sec > 0:
        time.sleep(args.pressure_settle_sec)
    pressure = pressure_client.read(timeout=args.pressure_timeout)
    if pressure is None:
        _step_failed(step_no, f"压力检查: {name}", f"未在 {args.pressure_timeout:.1f}s 内收到 pressure")
        return False
    max_pressure, sum_positive = _pressure_stats(pressure)
    if max_pressure < threshold:
        _log(f"    [pressure] FAILED max={max_pressure:.3f}, sum={sum_positive:.3f}, values={pressure}")
        _step_failed(step_no, f"压力检查: {name}", "压力不足")
        return False
    _log(f"    [pressure] OK max={max_pressure:.3f}, sum={sum_positive:.3f}")
    _step_done(step_no, f"压力检查: {name}")
    return True


def _call_hand_open(args, hand_client: HandClient | None):
    step_no = _step_start("手掌复位/放开")
    if args.execute:
        if hand_client is None:
            raise RuntimeError("hand client is not initialized")
        hand_client.call(HAND_OPEN)
    _step_done(step_no, "手掌复位/放开")


def _call_hand_close(args, hand_client: HandClient | None):
    step_no = _step_start("手掌闭合抓取")
    if args.execute:
        if hand_client is None:
            raise RuntimeError("hand client is not initialized")
        hand_client.call(HAND_CLOSE)
    _step_done(step_no, "手掌闭合抓取")


def _move_to_via3_and_grasp(args, via_files: list[str], max_motion: float):
    _run_step(
        " -> ".join(os.path.splitext(os.path.basename(path))[0] for path in via_files),
        _via_cmd(args, via_files, max_motion, use_joints=True),
        args.execute,
    )
    _run_step(
        "via3 -> 视觉抓取点",
        _descend_cmd(args),
        args.execute,
    )


def _recover_to_via1(args, current: str, via1: str, via2: str, via3: str):
    print(f"\n[*] 恢复到 via1: current={current}")
    if current == "grasp":
        _run_step(
            "抓取点 -> via3",
            _via_cmd(args, [via3], args.return_max_motion, use_joints=False),
            args.execute,
        )
        _run_step(
            "via3 -> via2 -> via1",
            _via_cmd(args, [via2, via1], args.return_max_motion, use_joints=True),
            args.execute,
        )
    elif current == "qr":
        _run_step(
            "QR 展示点 -> via3 -> via2 -> via1",
            _via_cmd(args, [via3, via2, via1], args.return_max_motion, use_joints=True),
            args.execute,
        )
    elif current == "via3":
        _run_step(
            "via3 -> via2 -> via1",
            _via_cmd(args, [via2, via1], args.return_max_motion, use_joints=True),
            args.execute,
        )
    elif current == "via2":
        _run_step(
            "via2 -> via1",
            _via_cmd(args, [via1], args.return_max_motion, use_joints=True),
            args.execute,
        )
    elif current == "via1":
        print("[recover] already at via1")
    else:
        _run_step(
            "当前位置 -> via3 -> via2 -> via1",
            _via_cmd(args, [via3, via2, via1], args.return_max_motion, use_joints=True),
            args.execute,
        )


def _return_via1_to_via0(args, via0: str, via1: str):
    _run_step(
        "via1 -> via0",
        _via_cmd(args, [via1, via0], args.return_max_motion, use_joints=True),
        args.execute,
    )


def _estimate_step_total(args) -> int:
    total = 0
    if not args.skip_lock:
        total += 1
    if not args.skip_hand_open:
        total += 1
    if args.combine_approach:
        total += 1
    else:
        total += 2
    if not args.skip_hand_close:
        total += 1
    total += 0 if args.disable_pressure_checks else 1

    if args.return_mode == "via3":
        total += 1
    elif args.return_mode == "via1":
        if args.scan_qr_after_present:
            total += 1
            total += 0 if args.disable_pressure_checks else 1
            total += 1
            total += 0 if args.disable_pressure_checks else 1
            total += 1
        else:
            total += 1
    elif args.return_mode == "via0":
        total += 1
    elif args.return_mode == "qr-present":
        total += 1
        total += 0 if args.disable_pressure_checks else 1
        if args.scan_qr_after_present:
            total += 1
            total += 0 if args.disable_pressure_checks else 1
            total += 1
        if args.place_after_qr:
            total += 1
            total += 1
            total += 1
            total += 1
    return total


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument("--locked-target", default=os.path.join("data", "runtime", "mpc_locked_target_latest.json"))
    parser.add_argument("--detect-seconds", type=float, default=12.0)
    parser.add_argument("--frame-timeout", type=float, default=8.0)
    parser.add_argument("--min-lock-hits", type=int, default=3,
                        help="Perception lock target must match this many valid YOLO/depth detections.")
    parser.add_argument("--lock-match-distance", type=float, default=0.12,
                        help="3D continuity threshold in meters for perception lock filtering.")
    parser.add_argument("--lock-max-missed", type=int, default=2,
                        help="Allowed missed detections while building a stable lock target.")
    parser.add_argument("--lock-target-policy", choices=["image_center", "highest_conf"], default="image_center",
                        help="When multiple stable targets exist, choose nearest lower-half image center by default.")
    parser.add_argument("--highlight-suppression", choices=["none", "mild"], default="mild",
                        help="Optional lock-stage highlight suppression for over-bright white bags. Default mild in full flow; use none to disable.")
    parser.add_argument("--show-window", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--show-qr-window", action=argparse.BooleanOptionalAction, default=False,
                        help="Show blocking post-grasp OCR/QR result window. Default false for full automatic flow.")
    parser.add_argument("--allow-cpu-detect", action="store_true")
    parser.add_argument("--above-object-height", type=float, default=0.10)
    parser.add_argument("--grasp-height", type=float, default=0.02)
    parser.add_argument("--grasp-profile", choices=["legacy_no_orientation", "tuned_with_orientation"],
                        default="tuned_with_orientation",
                        help="Grasp offset/orientation profile passed to run_visual_grasp_test.py.")
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
    parser.add_argument("--verbose-children", action="store_true",
                        help="Do not pass --quiet to child scripts; useful when debugging a specific step.")
    parser.add_argument("--qr-verbose", action="store_true",
                        help="Print post-grasp OCR/QR per-frame details without enabling verbose output for all child steps.")
    parser.add_argument("--connect-retries", type=int, default=3,
                        help="Retry rosbridge connection for each MPC child step.")
    parser.add_argument("--connect-retry-delay", type=float, default=1.0,
                        help="Seconds to wait between rosbridge connection retries.")
    parser.add_argument("--return-mode", choices=["none", "via3", "via1", "via0", "qr-present"], default="via3")
    parser.add_argument("--qr-present-file", default=os.path.join("data", "poses", "mpc_qr_present_pose_right.json"),
                        help="MPC pose captured at the post-grasp QR presentation position.")
    parser.add_argument("--scan-qr-after-present", action="store_true",
                        help="After moving to qr-present, run full-frame head-camera QR scan.")
    parser.add_argument("--qr-scan-seconds", type=float, default=30.0)
    parser.add_argument("--qr-transport", choices=["raw", "compressed"], default="raw",
                        help="Image transport used by post-grasp QR scan.")
    parser.add_argument("--qr-raw-throttle-ms", type=int, default=config.QR_RAW_RGB_THROTTLE_MS,
                        help="rosbridge throttle for post-grasp raw QR scan.")
    parser.add_argument("--qr-compressed-throttle-ms", type=int, default=0,
                        help="rosbridge throttle for post-grasp compressed QR scan.")
    parser.add_argument("--qr-snapshot-attempts", type=int, default=5)
    parser.add_argument("--qr-snapshot-interval", type=float, default=0.2)
    parser.add_argument("--qr-raw-frame-dir",
                        default=os.path.join("data", "runtime", "qr_multiframe_debug", "latest_raw", "raw"),
                        help="Directory where post-grasp QR scan saves raw frames for offline recovery.")
    parser.add_argument("--qr-recover-output-dir",
                        default=os.path.join("data", "runtime", "qr_multiframe_debug", "latest_recover"))
    parser.add_argument("--qr-recover-min-consensus", type=int, default=2)
    parser.add_argument("--auto-recover-qr-offline", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--qr-output", default=os.path.join("data", "runtime", "post_grasp_qr_latest.json"))
    parser.add_argument("--place-after-qr", action="store_true",
                        help="After QR presentation/scan, move back to visual place point without tuned grasp orientation, open hand, then return via3->2->1->0.")
    parser.add_argument("--skip-lock", action="store_true", help="Use existing locked target instead of running perception lock.")
    parser.add_argument("--skip-hand-open", action="store_true")
    parser.add_argument("--skip-hand-close", action="store_true")
    parser.add_argument("--disable-pressure-checks", action="store_true")
    parser.add_argument("--grasp-pressure-threshold", type=float, default=0.0)
    parser.add_argument("--carry-pressure-threshold", type=float, default=0.05)
    parser.add_argument("--qr-pressure-threshold", type=float, default=0.0,
                        help="Pressure threshold at QR presentation / after QR scan. Kept lower than grasp threshold.")
    parser.add_argument("--pressure-timeout", type=float, default=2.0)
    parser.add_argument("--pressure-settle-sec", type=float, default=0.5)
    parser.add_argument("--max-grasp-retries", type=int, default=1,
                        help="Retry grasp after pressure failure by returning to via1 and relocking. Does not change hand q or offsets.")
    parser.add_argument("--combine-approach", action="store_true",
                        help="Experimental: send via0->via3->target in one MPC command.")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    global _STEP_INDEX, _STEP_TOTAL
    global _CURRENT_QR_OUTPUT
    _STEP_INDEX = 0
    _STEP_TOTAL = _estimate_step_total(args)
    _CURRENT_QR_OUTPUT = args.qr_output

    via0 = os.path.join("data", "poses", "mpc_via0_home_right.json")
    via1 = os.path.join("data", "poses", "mpc_via1_pose_right.json")
    via2 = os.path.join("data", "poses", "mpc_via2_pose_right.json")
    via3 = os.path.join("data", "poses", "mpc_via3_pose_right.json")

    print("=" * 70)
    print("  MPC full visual grasp flow")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  Locked target: {args.locked_target}")
    print(f"  Grasp profile: {args.grasp_profile}")
    print(f"  Execute: {args.execute}")
    print("=" * 70)

    if not args.execute:
        print("[PLAN ONLY] 未发送任何运动/手掌命令。加 --execute 才会执行。")

    hand_client = None
    pressure_client = None
    needs_hand = not args.skip_hand_open or not args.skip_hand_close or args.place_after_qr
    if args.execute and needs_hand:
        hand_client = HandClient(args.ws_url)
        hand_client.connect()
    if args.execute and not args.disable_pressure_checks:
        pressure_client = PressureClient(args.ws_url)
        pressure_client.connect()

    try:
        qr_scan_handle = None
        if not args.skip_lock:
            _run_step("低头、视觉锁存、抬头", _lock_cmd(args), args.execute)
        elif _load_locked_target(args.locked_target) is None:
            raise SystemExit(f"[✗] --skip-lock 但目标文件不存在: {args.locked_target}")

        if not args.skip_hand_open:
            _call_hand_open(args, hand_client)

        attempt = 0
        start_from = "via0"
        while True:
            current = start_from
            if attempt > 0:
                print(f"\n[*] 抓取重试 {attempt}/{args.max_grasp_retries}")
                _run_step("重新低头、视觉锁存、抬头", _lock_cmd(args), args.execute)
                _call_hand_open(args, hand_client)

            if args.combine_approach and start_from == "via0":
                _run_step(
                    "via0 起步: via1 -> via2 -> via3 -> 视觉抓取点",
                    _approach_and_descend_cmd(args, [via1, via2, via3]),
                    args.execute,
                )
            else:
                if start_from == "via0":
                    _move_to_via3_and_grasp(args, [via1, via2, via3], args.via_max_motion)
                elif start_from == "via1":
                    _move_to_via3_and_grasp(args, [via2, via3], args.via_max_motion)
                else:
                    _move_to_via3_and_grasp(args, [via3], args.via_max_motion)
            current = "grasp"

            if not args.skip_hand_close:
                _call_hand_close(args, hand_client)

            if not _check_pressure(args, pressure_client, "抓取点", args.grasp_pressure_threshold):
                if attempt < args.max_grasp_retries:
                    _recover_to_via1(args, current, via1, via2, via3)
                    _call_hand_open(args, hand_client)
                    attempt += 1
                    start_from = "via1"
                    continue
                _recover_to_via1(args, current, via1, via2, via3)
                _call_hand_open(args, hand_client)
                _return_via1_to_via0(args, via0, via1)
                raise SystemExit("[✗] 抓取点压力不足，重试后仍失败")

            if args.return_mode == "via3":
                _run_step(
                    "抓取后返回 via3",
                    _via_cmd(args, [via3], args.return_max_motion, use_joints=False),
                    args.execute,
                )
                break
            if args.return_mode == "via1" and not args.scan_qr_after_present:
                _run_step(
                    "抓取后 via3 -> via2 -> via1",
                    _via_cmd(args, [via3, via2, via1], args.return_max_motion, use_joints=True),
                    args.execute,
                )
                break
            if args.return_mode == "via0":
                _run_step(
                    "抓取后 via3 -> via2 -> via1 -> via0",
                    _via_cmd(args, [via3, via2, via1, via0], args.return_max_motion, use_joints=True),
                    args.execute,
                )
                break
            if args.return_mode not in ("qr-present", "via1"):
                print("[*] return-mode=none，抓取后保持当前位置")
                break

            if not os.path.exists(args.qr_present_file):
                raise SystemExit(f"[✗] QR 展示点文件不存在: {args.qr_present_file}")
            _run_step(
                "抓取后直接到 QR 展示点",
                _via_cmd(args, [args.qr_present_file], args.return_max_motion, use_joints=True),
                args.execute,
            )
            current = "qr"

            if not _check_pressure(args, pressure_client, "QR 展示点", args.qr_pressure_threshold):
                if attempt < args.max_grasp_retries:
                    _recover_to_via1(args, current, via1, via2, via3)
                    _call_hand_open(args, hand_client)
                    attempt += 1
                    start_from = "via1"
                    continue
                _recover_to_via1(args, current, via1, via2, via3)
                _call_hand_open(args, hand_client)
                _return_via1_to_via0(args, via0, via1)
                raise SystemExit("[✗] QR 展示点压力不足，重试后仍失败")

            if args.scan_qr_after_present:
                if args.place_after_qr:
                    qr_scan_handle = _start_qr_scan_background(
                        "头部相机 OCR/QR 识别",
                        _qr_scan_cmd(args),
                        args.execute,
                    )
                else:
                    _run_step(
                        "头部相机 OCR/QR 识别",
                        _qr_scan_cmd(args),
                        args.execute,
                    )
                if not _check_pressure(args, pressure_client, "扫码后", args.qr_pressure_threshold):
                    if attempt < args.max_grasp_retries:
                        _recover_to_via1(args, current, via1, via2, via3)
                        _call_hand_open(args, hand_client)
                        attempt += 1
                        start_from = "via1"
                        continue
                    _recover_to_via1(args, current, via1, via2, via3)
                    _call_hand_open(args, hand_client)
                    _return_via1_to_via0(args, via0, via1)
                    raise SystemExit("[✗] 扫码后压力不足，重试后仍失败")

            if args.return_mode == "via1":
                _recover_to_via1(args, current, via1, via2, via3)
                break

            if args.place_after_qr:
                _run_step(
                    "QR 展示点 -> via3",
                    _via_cmd(args, [via3], args.return_max_motion, use_joints=True),
                    args.execute,
                )
                current = "via3"
                _run_step(
                    "via3 -> 视觉放置点上方",
                    _descend_cmd(args, grasp_profile="legacy_no_orientation", include_descend=False),
                    args.execute,
                )
                _call_hand_open(args, hand_client)
                _run_step(
                    "放置后 via3 -> via2 -> via1 -> via0",
                    _via_cmd(args, [via3, via2, via1, via0], args.return_max_motion, use_joints=True),
                    args.execute,
                )
            _finish_qr_scan_background(qr_scan_handle)
            break
    finally:
        if pressure_client is not None:
            pressure_client.close()
        if hand_client is not None:
            hand_client.close()


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as exc:
        print(f"[✗] 流程中止: 子步骤执行失败 exit={exc.returncode}")
        sys.exit(exc.returncode or 1)
    except RuntimeError as exc:
        print(f"[✗] 流程中止: {exc}")
        sys.exit(1)
