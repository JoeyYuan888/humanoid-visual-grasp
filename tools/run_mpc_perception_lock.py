#!/usr/bin/env python3
"""
MPC perception lock workflow.

Sequence:
1. Move neck to a look-down pose through /wa/wa_hardware_interface/neck_movej.
2. Run the existing vision pipeline headlessly for a short window.
3. Select the best valid target and convert camera point -> HEAD -> BASE.
4. Save data/mpc_locked_target_latest.json while the neck TF is still low.
5. Move neck back to home through the same MPC neck service.
"""

from __future__ import annotations

import argparse
import builtins
import json
import os
import sys
import threading
import time

import cv2
import numpy as np
import roslibpy
from roslibpy.core import ServiceException

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot_grasp import config
from robot_grasp.grasp_flow import object_conf, summarize_target
from robot_grasp.logger import DataLogger
from robot_grasp.ros_client import ROSClient
from robot_grasp.vision_pipeline import VisionPipeline

from tools.run_mpc_visual_grasp_test import (
    DEFAULT_CAM2HEAD,
    DEFAULT_LOCKED_TARGET,
    _build_graph,
    _camera_point_m,
    _connect,
    _call,
    _find_path,
    _load_transform,
    _lookup_transform,
    _object_base_from_target,
    _sample_tf,
    _save_locked_target,
    _service_type,
)


NECK_SERVICE = "/wa/wa_hardware_interface/neck_movej"
MPC_MODE_SERVICE = "/wa/wa_hardware_interface/mpc_mode_setting"
WAIST_LOCK_SERVICE = "/wa/waist_lock_setting"
JOINT_STATES_TOPIC = "/zj_humanoid/upperlimb/joint_states"
_ORIGINAL_PRINT = builtins.print


def _enable_quiet_print():
    keep_patterns = (
        "[✗]",
        "[!]",
        "[阶段]",
        "[动作]",
        "[✓] 选择目标:",
        "[✓] 已锁存 BASE 目标:",
        "base  :",
        "[✓] neck-only",
    )

    def quiet_print(*args, **kwargs):
        text = " ".join(str(arg) for arg in args)
        if any(pattern in text for pattern in keep_patterns):
            kwargs.setdefault("flush", True)
            _ORIGINAL_PRINT(*args, **kwargs)

    builtins.print = quiet_print


def _print_tf_debug(transforms: dict[tuple[str, str], dict]):
    frames = sorted({frame for edge in transforms for frame in edge})
    print(f"[TF] 本次共采到 {len(transforms)} 条 transform, {len(frames)} 个 frame")
    if frames:
        print("[TF] frame 列表:")
        for name in frames:
            print(f"    - {name}")

    keywords = ("BASE", "HEAD", "NECK", "WAIST", "realsense", "head", "base", "neck")
    relevant = [
        (parent, child)
        for parent, child in sorted(transforms)
        if any(key in parent or key in child for key in keywords)
    ]
    if relevant:
        print("[TF] 相关 transform:")
        for parent, child in relevant:
            print(f"    {parent} -> {child}")
    else:
        print("[TF] 没有采到包含 BASE/HEAD/NECK/realsense 的 transform")

    graph = _build_graph(transforms)
    candidates = [
        ("BASE", "HEAD"),
        ("root", "HEAD"),
        ("BASE", "NECK"),
        ("BASE", "realsense_head_link"),
        ("HEAD", "realsense_head_link"),
    ]
    print("[TF] 常见路径检查:")
    for start, goal in candidates:
        path = _find_path(graph, start, goal)
        print(f"    {start} -> {goal}: {'OK' if path is not None else 'missing'}")


def _set_mpc_mode(client, enabled: bool):
    srv_type = _service_type(client, MPC_MODE_SERVICE)
    if not srv_type:
        raise RuntimeError(f"找不到 MPC mode 服务: {MPC_MODE_SERVICE}")
    response = _call(client, MPC_MODE_SERVICE, srv_type, {"data": bool(enabled)})
    print(f"[mpc_mode={enabled}] {response}")
    if response and response.get("success") is False:
        raise RuntimeError(f"MPC mode 设置失败: {response}")
    return response


def _wait_for_joint_state(client, timeout: float = 2.0) -> dict | None:
    latest = {}
    event = threading.Event()

    def callback(message):
        names = message.get("name", [])
        positions = message.get("position", [])
        if names and positions:
            latest["message"] = message
            event.set()

    sub = roslibpy.Topic(client, JOINT_STATES_TOPIC, "sensor_msgs/JointState")
    sub.subscribe(callback)
    ok = event.wait(timeout)
    try:
        sub.unsubscribe()
    except Exception:
        pass
    if not ok:
        return None
    return latest.get("message")


def _set_neck_track(client, enabled: bool):
    srv_type = _service_type(client, WAIST_LOCK_SERVICE)
    if not srv_type:
        raise RuntimeError(f"找不到 waist/neck track 服务: {WAIST_LOCK_SERVICE}")
    joint_msg = _wait_for_joint_state(client, timeout=2.0)
    if joint_msg is None:
        raise RuntimeError(f"未读到 {JOINT_STATES_TOPIC}，无法设置 neck_track")
    joint_state = [float(value) for value in joint_msg.get("position", [])]
    if len(joint_state) != 22:
        raise RuntimeError(f"{JOINT_STATES_TOPIC} position 长度不是 22: {len(joint_state)}")
    request = {
        "joint_state": joint_state,
        "lock_index": [],
        "neck_track": bool(enabled),
    }
    print(f"[*] waist_lock_setting neck_track={enabled}, joint_state_len={len(joint_state)}")
    response = _call(client, WAIST_LOCK_SERVICE, srv_type, request)
    print(f"[neck_track={enabled}] {response}")
    if response and response.get("success") is False:
        message = str(response.get("message", ""))
        if "mpc_target_lock_waist_setting" in message and "unavailable" in message:
            raise RuntimeError(
                "neck_track 后端服务未启动: /wa/mpc_target_lock_waist_setting unavailable。"
                "当前 /wa/waist_lock_setting 只是外层入口，无法真正打开 neck_track。"
                "请在机器人端确认该服务是否存在，或询问厂商需要启动哪个 MPC target/waist lock 节点。"
            )
        raise RuntimeError(f"neck_track 设置失败: {response}")
    return response


def _wait_for_neck_state(client, timeout: float = 2.0) -> tuple[float, float] | None:
    latest = {}
    event = threading.Event()

    def callback(message):
        names = message.get("name", [])
        positions = message.get("position", [])
        try:
            neck_z = float(positions[names.index("Neck_Z")])
            neck_y = float(positions[names.index("Neck_Y")])
        except (ValueError, IndexError, TypeError):
            return
        latest["state"] = (neck_z, neck_y)
        event.set()

    sub = roslibpy.Topic(client, JOINT_STATES_TOPIC, "sensor_msgs/JointState")
    sub.subscribe(callback)
    ok = event.wait(timeout)
    try:
        sub.unsubscribe()
    except Exception:
        pass
    if not ok:
        return None
    return latest.get("state")


def _call_neck(client, neck_z: float, neck_y: float, duration: float, required: bool = True,
               verify: bool = True, tolerance: float = 0.04):
    srv_type = _service_type(client, NECK_SERVICE)
    if not srv_type:
        msg = f"[✗] 找不到 neck 服务类型: {NECK_SERVICE}"
        if required:
            raise RuntimeError(msg)
        print(msg)
        return None
    request = {
        "neck_joint": [float(neck_z), float(neck_y)],
        "t": float(duration),
    }
    print(f"[*] neck_movej -> z={neck_z:.3f}, y={neck_y:.3f}, t={duration:.1f}s")
    before = _wait_for_neck_state(client, timeout=1.0) if verify else None
    if before is not None:
        print(f"    before Neck_Z={before[0]:.3f}, Neck_Y={before[1]:.3f}")
    try:
        response = _call(client, NECK_SERVICE, srv_type, request)
    except ServiceException as exc:
        text = str(exc)
        if "mpc_hardware_interface" in text:
            raise RuntimeError(
                "rosbridge 环境缺 mpc_hardware_interface，无法代理 MPC neck 服务。"
                "请把本项目的 ros_pkgs/mpc_hardware_interface/ 放到机器人容器 "
                "/workspace/catkin_ws/mpc_ws/src/，catkin_make 后重启 9091 rosbridge。"
            ) from exc
        raise
    print(f"[neck] {response}")
    if response and response.get("success") is False:
        raise RuntimeError(f"neck_movej 返回失败: {response}")
    if verify:
        time.sleep(duration + 0.3)
        after = _wait_for_neck_state(client, timeout=2.0)
        if after is None:
            print(f"[!] 未读到 {JOINT_STATES_TOPIC}，无法确认 neck 是否到位")
        else:
            err_z = abs(after[0] - neck_z)
            err_y = abs(after[1] - neck_y)
            print(f"    after  Neck_Z={after[0]:.3f}, Neck_Y={after[1]:.3f}, err=({err_z:.3f},{err_y:.3f})")
            if err_z > tolerance or err_y > tolerance:
                raise RuntimeError(
                    f"neck_movej 调用完成但关节未到目标: target=({neck_z:.3f},{neck_y:.3f}), "
                    f"actual=({after[0]:.3f},{after[1]:.3f})"
                )
    return response


def _make_vision_cleanup(pipeline: VisionPipeline, client: ROSClient,
                         show_window: bool, window_name: str):
    def _cleanup(disconnect_client: bool = True):
        try:
            pipeline.stop()
        except Exception as exc:
            print(f"[!] 视觉 pipeline 清理异常，已忽略: {exc}")
        if show_window:
            try:
                cv2.destroyWindow(window_name)
                cv2.waitKey(1)
            except Exception:
                pass
        if not disconnect_client:
            return
        try:
            client.disconnect()
        except Exception as exc:
            print(f"[!] 视觉 rosbridge 清理异常，已忽略: {exc}")

    return _cleanup


def _target_xyz_m(target: dict) -> np.ndarray | None:
    try:
        return np.array([
            float(target.get("x_mm")) / 1000.0,
            float(target.get("y_mm")) / 1000.0,
            float(target.get("z_mm")) / 1000.0,
        ], dtype=float)
    except (TypeError, ValueError):
        return None


def _valid_lock_candidates(object_results: list[dict], preferred_label: str | None) -> list[dict]:
    candidates = []
    for obj in object_results:
        if preferred_label and obj.get("label") != preferred_label:
            continue
        if not obj.get("valid"):
            continue
        if object_conf(obj) < config.OBJECT_MIN_CONF:
            continue
        if _target_xyz_m(obj) is None:
            continue
        candidates.append(obj)
    return candidates


def _draw_selected_target(frame: np.ndarray, target: dict | None) -> np.ndarray:
    if target is None:
        return frame
    bbox = target.get("bbox")
    if not bbox or len(bbox) != 4:
        return frame
    try:
        x1, y1, x2, y2 = [int(round(float(value))) for value in bbox]
    except (TypeError, ValueError):
        return frame
    out = frame.copy()
    color = (0, 0, 255)
    cv2.rectangle(out, (x1, y1), (x2, y2), color, 4)
    label = f"LOCK #{target.get('idx', '')} {target.get('label', '')}"
    cv2.putText(
        out,
        label,
        (max(0, x1), max(24, y1 - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        color,
        2,
        cv2.LINE_AA,
    )
    return out


def _suppress_highlights_mild(rgb: np.ndarray) -> np.ndarray:
    """Conservative RGB highlight suppression for over-bright white bags."""
    if rgb is None:
        return rgb
    lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB)
    l_channel, a_channel, b_channel = cv2.split(lab)

    l_float = l_channel.astype(np.float32) / 255.0
    # gamma > 1 darkens highlights slightly without changing geometry.
    l_float = np.power(l_float, 1.12)
    l_channel = np.clip(l_float * 255.0, 0, 255).astype(np.uint8)

    highlight_mask = l_channel > 225
    if np.any(highlight_mask):
        compressed = 225 + (l_channel.astype(np.int16) - 225) * 0.35
        l_channel = np.where(highlight_mask, compressed, l_channel).clip(0, 255).astype(np.uint8)

    clahe = cv2.createCLAHE(clipLimit=1.2, tileGridSize=(8, 8))
    l_channel = clahe.apply(l_channel)
    processed = cv2.merge((l_channel, a_channel, b_channel))
    return cv2.cvtColor(processed, cv2.COLOR_LAB2RGB)


def _preprocess_lock_rgb(rgb: np.ndarray, mode: str) -> np.ndarray:
    if mode == "mild":
        return _suppress_highlights_mild(rgb)
    return rgb


class _StableTargetFilter:
    """Reject one-off lock-stage false positives by requiring repeated 3D continuity."""

    def __init__(self, min_hits: int, match_distance_m: float, max_missed: int,
                 target_policy: str = "image_center"):
        self.min_hits = max(1, int(min_hits))
        self.match_distance_m = max(0.001, float(match_distance_m))
        self.max_missed = max(0, int(max_missed))
        self.target_policy = target_policy
        self.tracks: list[dict] = []
        self.frame_index = 0

    @staticmethod
    def _center_distance(target: dict, image_shape) -> float:
        if image_shape is None:
            return float("inf")
        height, width = image_shape[:2]
        center = target.get("center")
        if not center or width <= 0 or height <= 0:
            return float("inf")
        try:
            u = float(center[0])
            v = float(center[1])
        except (TypeError, ValueError, IndexError):
            return float("inf")
        target_u = width * 0.5
        target_v = height * 0.75
        dx = (u - target_u) / max(1.0, width)
        dy = (v - target_v) / max(1.0, height)
        return float((dx * dx + dy * dy) ** 0.5)

    def update(self, candidates: list[dict], image_shape=None) -> dict | None:
        self.frame_index += 1
        matched_tracks: set[int] = set()

        for candidate in sorted(candidates, key=object_conf, reverse=True):
            xyz = _target_xyz_m(candidate)
            if xyz is None:
                continue

            best_index = None
            best_dist = None
            for index, track in enumerate(self.tracks):
                if index in matched_tracks:
                    continue
                if track["label"] != candidate.get("label"):
                    continue
                dist = float(np.linalg.norm(xyz - track["xyz"]))
                if dist > self.match_distance_m:
                    continue
                if best_dist is None or dist < best_dist:
                    best_dist = dist
                    best_index = index

            if best_index is None:
                self.tracks.append({
                    "label": candidate.get("label"),
                    "xyz": xyz,
                    "target": candidate,
                    "hits": 1,
                    "score_sum": object_conf(candidate),
                    "last_seen": self.frame_index,
                })
                matched_tracks.add(len(self.tracks) - 1)
                continue

            track = self.tracks[best_index]
            track["xyz"] = xyz
            track["target"] = candidate
            track["hits"] += 1
            track["score_sum"] += object_conf(candidate)
            track["last_seen"] = self.frame_index
            matched_tracks.add(best_index)

        self.tracks = [
            track for track in self.tracks
            if self.frame_index - track["last_seen"] <= self.max_missed
        ]
        stable_tracks = [track for track in self.tracks if track["hits"] >= self.min_hits]
        if not stable_tracks:
            return None
        if self.target_policy == "highest_conf":
            best_track = max(
                stable_tracks,
                key=lambda track: (
                    track["hits"],
                    track["score_sum"] / max(1, track["hits"]),
                    object_conf(track["target"]),
                ),
            )
        else:
            best_track = min(
                stable_tracks,
                key=lambda track: (
                    self._center_distance(track["target"], image_shape),
                    -track["hits"],
                    -object_conf(track["target"]),
                ),
            )
        return best_track["target"]

    def summary(self) -> str:
        if not self.tracks:
            return "no tracks"
        parts = []
        for track in sorted(self.tracks, key=lambda item: item["hits"], reverse=True):
            xyz = track["xyz"]
            parts.append(
                f"{track['label']} hits={track['hits']} "
                f"xyz=({xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f})"
            )
        return "; ".join(parts[:4])


def _run_vision(ws_url: str, seconds: float, preferred_label: str,
                show_window: bool = False, window_name: str = "MPC Perception Lock",
                frame_timeout: float = 5.0, min_lock_hits: int = 3,
                lock_match_distance_m: float = 0.12, lock_max_missed: int = 2,
                lock_target_policy: str = "image_center",
                highlight_suppression: str = "none"):
    logger = DataLogger()
    pipeline = VisionPipeline()
    client = ROSClient(ws_url=ws_url)
    cleanup = _make_vision_cleanup(pipeline, client, show_window, window_name)
    stable_filter = _StableTargetFilter(
        min_hits=min_lock_hits,
        match_distance_m=lock_match_distance_m,
        max_missed=lock_max_missed,
        target_policy=lock_target_policy,
    )
    latest_result = None
    best_target = None
    start = time.time()
    last_frame_count = -1
    last_perf_time = time.time()
    last_stats = None
    sample_frames = 0
    sample_detects = 0
    total_frames = 0

    if not client.connect():
        cleanup()
        raise RuntimeError(f"无法连接视觉 rosbridge: {ws_url}")
    last_stats = client.get_stats()
    print(f"[*] 开始视觉检测 {seconds:.1f}s，目标类别: {preferred_label}")
    if show_window:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        print(f"[*] 已打开视觉窗口: {window_name} | q=提前结束检测")
    while time.time() - start < seconds:
        rgb, depth, cam_info, fc = client.get_frames()
        if rgb is None or fc == last_frame_count:
            if total_frames == 0 and time.time() - start > frame_timeout:
                stats = client.get_stats()
                print(
                    f"[!] {frame_timeout:.1f}s 内没有收到可用 RGB 帧: "
                    f"rgb_count={stats.get('rgb_count', 0)} depth_msg_count={stats.get('depth_msg_count', 0)} "
                    f"depth_count={stats.get('depth_count', 0)} camera_info={'yes' if cam_info else 'no'}"
                )
                break
            if show_window and (cv2.waitKey(5) & 0xFF) == ord("q"):
                print("[*] 用户提前结束视觉检测")
                break
            time.sleep(0.01)
            continue
        last_frame_count = fc
        sample_frames += 1
        total_frames += 1
        raw_rgb, _, raw_rgb_updated_at = client.get_raw_rgb()
        detect_rgb = _preprocess_lock_rgb(rgb, highlight_suppression)
        result = pipeline.process(
            rgb=detect_rgb,
            depth=depth,
            cam_info=cam_info,
            frame_count=fc,
            client_stats=client.get_stats(),
            raw_rgb=raw_rgb,
            raw_rgb_updated_at=raw_rgb_updated_at,
            fps=0.0,
        )
        latest_result = result
        if result["should_detect"]:
            sample_detects += 1
            candidates = _valid_lock_candidates(result["object_results"], preferred_label=preferred_label)
            target = stable_filter.update(candidates, image_shape=rgb.shape)
        else:
            target = None
        if target is not None and (
            best_target is None or object_conf(target) > object_conf(best_target)
        ):
            best_target = target
        if show_window:
            cv2.imshow(window_name, _draw_selected_target(result["annotated"], target or best_target))
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                print("[*] 用户提前结束视觉检测")
                break

        now = time.time()
        if now - last_perf_time >= config.PERF_LOG_INTERVAL_SEC and last_stats is not None:
            stats = client.get_stats()
            elapsed = now - last_perf_time
            debug = result["debug"]
            logger.log_perf(fc, {
                **debug,
                "display_fps": sample_frames / elapsed,
                "rgb_rx_fps": (stats["rgb_count"] - last_stats["rgb_count"]) / elapsed,
                "depth_msg_rx_fps": (stats["depth_msg_count"] - last_stats["depth_msg_count"]) / elapsed,
                "depth_rx_fps": (stats["depth_count"] - last_stats["depth_count"]) / elapsed,
                "detect_fps": sample_detects / elapsed,
                "infer_ms": result["avg_infer_ms"],
                "last_infer_ms": result["last_infer_ms"],
                "det_count": len(result["detections"]),
                "depth_age_ms": max(0.0, (now - stats["depth_updated_at"]) * 1000)
                if stats["depth_updated_at"] > 0 else 0.0,
                "raw_rgb_age_ms": max(0.0, (now - stats.get("raw_rgb_updated_at", 0)) * 1000)
                if stats.get("raw_rgb_updated_at", 0) > 0 else 0.0,
            })
            last_perf_time = now
            last_stats = stats
            sample_frames = 0
            sample_detects = 0

    csv_path = logger.save(os.path.join(PROJECT_ROOT, "data"))
    if latest_result is not None:
        print("[*] 最后一帧 object_results:")
        for obj in latest_result["object_results"]:
            print(f"    {summarize_target(obj)}")
        if best_target is None:
            print(f"[!] 连续性过滤未通过: {stable_filter.summary()}")
    if best_target is not None:
        print(f"[✓] 选择目标: {summarize_target(best_target)}")
    return best_target, csv_path, cleanup


def _sample_head_to_base(control_client, tf_seconds: float) -> np.ndarray:
    print(f"[*] 采样 TF {tf_seconds:.1f}s，查找 BASE -> HEAD")
    transforms = _sample_tf(control_client, tf_seconds)
    head_to_base = _lookup_transform(transforms, "BASE", "HEAD")
    if head_to_base is None:
        _print_tf_debug(transforms)
        raise RuntimeError("TF 中没有找到 BASE -> HEAD，不能锁存 BASE 目标")
    return head_to_base


def _lock_target(control_client, target: dict, csv_path: str, cam2head_path: str,
                 output_path: str, tf_seconds: float, approach_height: float,
                 head_to_base: np.ndarray | None = None):
    cam2head = _load_transform(cam2head_path, "cam2head")
    point_cam = np.array(_camera_point_m(target), dtype=float)
    point_head = (cam2head @ np.array([*point_cam, 1.0], dtype=float))[:3]

    if head_to_base is None:
        head_to_base = _sample_head_to_base(control_client, tf_seconds)

    object_base = _object_base_from_target(target, cam2head, head_to_base)
    payload = {
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "source_csv": csv_path,
        "target": target,
        "camera_point_m": point_cam.tolist(),
        "head_point_m": point_head.tolist(),
        "object_base_m": object_base.tolist(),
        "default_approach_height_m": float(approach_height),
        "note": "Locked BASE target from automated MPC neck perception flow. Reuse after neck pose changes.",
    }
    _save_locked_target(output_path, payload)
    print(f"[✓] 已锁存 BASE 目标: {output_path}")
    print(f"    camera: {point_cam.tolist()}")
    print(f"    head  : {point_head.tolist()}")
    print(f"    base  : {object_base.tolist()}")
    return payload


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws-url", default="ws://192.168.20.98:9091")
    parser.add_argument("--preferred-label", default="plastic bag")
    parser.add_argument("--detect-seconds", type=float, default=8.0)
    parser.add_argument("--neck-down-z", type=float, default=0.0)
    parser.add_argument("--neck-down-y", type=float, default=0.35)
    parser.add_argument("--neck-home-z", type=float, default=0.0)
    parser.add_argument("--neck-home-y", type=float, default=0.0)
    parser.add_argument("--neck-time", type=float, default=4.0)
    parser.add_argument("--neck-backend", choices=["mpc", "manual"], default="mpc",
                        help="头部控制后端；默认 mpc：先开启 MPC mode，再调用 /wa/wa_hardware_interface/neck_movej")
    parser.add_argument("--enable-mpc-for-neck", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--enable-neck-track", action=argparse.BooleanOptionalAction, default=False,
                        help="默认关闭；当前 neck_movej 只要求 MPC mode=true，不强依赖 waist_lock_setting neck_track")
    parser.add_argument("--disable-neck-track-after", action="store_true")
    parser.add_argument("--verify-neck", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--neck-verify-tolerance", type=float, default=0.04)
    parser.add_argument("--settle-seconds", type=float, default=1.0)
    parser.add_argument("--tf-seconds", type=float, default=2.0)
    parser.add_argument("--cam2head", default=DEFAULT_CAM2HEAD)
    parser.add_argument("--output", default=DEFAULT_LOCKED_TARGET)
    parser.add_argument("--approach-height", type=float, default=0.10)
    parser.add_argument("--allow-cpu-detect", action="store_true",
                        help="临时允许 YOLO 在 CPU 上运行；只用于 CUDA/driver 未恢复时锁存坐标，速度会明显下降")
    parser.add_argument("--cpu-detect-every-n-frames", type=int, default=6,
                        help="CPU 检测时降低 YOLO 频率，避免 rosbridge/显示被推理阻塞")
    parser.add_argument("--show-window", action=argparse.BooleanOptionalAction, default=False,
                        help="检测阶段显示 OpenCV 窗口，方便确认是否捕捉到塑料袋；默认关闭")
    parser.add_argument("--window-name", default="MPC Perception Lock")
    parser.add_argument("--frame-timeout", type=float, default=5.0,
                        help="视觉阶段等待首帧的最长时间；超时后保存空 CSV 并继续安全抬头")
    parser.add_argument("--min-lock-hits", type=int, default=3,
                        help="锁存目标至少需要连续匹配的有效检测次数，用于过滤偶发 FP")
    parser.add_argument("--lock-match-distance", type=float, default=0.12,
                        help="同一目标连续匹配的 3D 距离阈值，单位 m")
    parser.add_argument("--lock-max-missed", type=int, default=2,
                        help="连续性过滤允许目标短暂丢失的检测次数")
    parser.add_argument("--lock-target-policy", choices=["image_center", "highest_conf"], default="image_center",
                        help="多个稳定目标的选择策略；默认 image_center：选择最靠近画面下半部中点的目标")
    parser.add_argument("--highlight-suppression", choices=["none", "mild"], default="none",
                        help="锁存阶段可选轻量高光抑制；默认 none，mild 用于白色塑料袋过曝时测试")
    parser.add_argument("--neck-only", action="store_true", help="只执行 neck down/home，不运行视觉检测")
    parser.add_argument("--skip-neck-down", action="store_true")
    parser.add_argument("--skip-neck-home", action="store_true")
    parser.add_argument("--quiet", action="store_true", help="只打印关键结果和错误")
    args = parser.parse_args()

    if args.quiet:
        _enable_quiet_print()

    if args.allow_cpu_detect:
        config.REQUIRE_CUDA = False
        config.YOLO_HALF = False
        config.DETECT_EVERY_N_FRAMES = max(1, int(args.cpu_detect_every_n_frames))
        print(
            f"[!] 临时 CPU 检测模式: REQUIRE_CUDA=False, "
            f"DETECT_EVERY_N_FRAMES={config.DETECT_EVERY_N_FRAMES}"
        )

    print("=" * 70)
    print("  MPC neck perception lock flow")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  Neck down: [{args.neck_down_z}, {args.neck_down_y}]")
    print(f"  Neck home: [{args.neck_home_z}, {args.neck_home_y}]")
    print(f"  Detect seconds: {args.detect_seconds}")
    print(f"  Show window: {args.show_window}")
    print(f"  Output: {args.output}")
    print("=" * 70)

    control_client = _connect(args.ws_url)
    primary_error = None
    vision_cleanup = None
    try:
        if args.neck_backend == "mpc" and args.enable_mpc_for_neck and (not args.skip_neck_down or not args.skip_neck_home):
            _set_mpc_mode(control_client, True)
        if args.neck_backend == "mpc" and args.enable_neck_track and (not args.skip_neck_down or not args.skip_neck_home):
            _set_neck_track(control_client, True)

        if not args.skip_neck_down:
            if args.neck_backend == "mpc":
                print("[动作] 低头开始")
                _call_neck(
                    control_client,
                    args.neck_down_z,
                    args.neck_down_y,
                    args.neck_time,
                    verify=args.verify_neck,
                    tolerance=args.neck_verify_tolerance,
                )
                print("[动作] 低头完成")
            else:
                print("[manual] 请人工确认头部已低头到检测姿态")
            time.sleep(args.settle_seconds)

        low_head_to_base = None
        if not args.neck_only:
            print("[动作] 锁存准备: 采样低头 TF")
            low_head_to_base = _sample_head_to_base(control_client, args.tf_seconds)
            print("[动作] 锁存准备完成")

        if args.neck_only:
            print("[*] neck-only 模式：跳过视觉检测，只执行未被 skip 的 neck 动作")
        else:
            print("[动作] 视觉识别开始")
            target, csv_path, vision_cleanup = _run_vision(
                args.ws_url,
                args.detect_seconds,
                args.preferred_label,
                show_window=args.show_window,
                window_name=args.window_name,
                frame_timeout=args.frame_timeout,
                min_lock_hits=args.min_lock_hits,
                lock_match_distance_m=args.lock_match_distance,
                lock_max_missed=args.lock_max_missed,
                lock_target_policy=args.lock_target_policy,
                highlight_suppression=args.highlight_suppression,
            )
            if target is None or csv_path is None:
                raise RuntimeError("没有检测到 valid 目标，未锁存 BASE 坐标")
            print("[动作] 视觉识别完成")

            print("[动作] 坐标转换/锁存开始")
            _lock_target(
                control_client=control_client,
                target=target,
                csv_path=csv_path,
                cam2head_path=args.cam2head,
                output_path=args.output,
                tf_seconds=args.tf_seconds,
                approach_height=args.approach_height,
                head_to_base=low_head_to_base,
            )
            print("[动作] 坐标转换/锁存完成")

        if args.neck_only:
            print("[✓] neck-only 模式完成，未运行视觉检测")
        if args.neck_backend == "mpc" and args.disable_neck_track_after and args.enable_neck_track:
            _set_neck_track(control_client, False)
    except Exception as exc:
        primary_error = exc
        raise
    finally:
        if not args.skip_neck_home:
            try:
                if args.neck_backend == "mpc":
                    print("[动作] 抬头开始")
                    _call_neck(
                        control_client,
                        args.neck_home_z,
                        args.neck_home_y,
                        args.neck_time,
                        verify=args.verify_neck,
                        tolerance=args.neck_verify_tolerance,
                    )
                    print("[动作] 抬头完成")
                else:
                    print("[manual] 请人工确认头部已复位")
            except Exception as home_exc:
                if primary_error is not None:
                    print(f"[!] 主流程已失败，尝试 neck home 也失败: {home_exc}")
                else:
                    raise
        if primary_error is None:
            if vision_cleanup is not None:
                vision_cleanup(disconnect_client=False)
            try:
                cv2.destroyAllWindows()
                cv2.waitKey(1)
            except Exception:
                pass
            os._exit(0)
        else:
            if vision_cleanup is not None:
                vision_cleanup()
            try:
                control_client.terminate()
            except Exception:
                pass


if __name__ == "__main__":
    main()
