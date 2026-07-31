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
from robot_grasp.grasp_flow import object_conf, select_grasp_target, summarize_target
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
                "请把本项目的 mpc_hardware_interface/ 放到机器人容器 "
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


def _cleanup_vision_async(pipeline: VisionPipeline, client: ROSClient,
                          show_window: bool, window_name: str):
    def _cleanup():
        try:
            pipeline.stop()
        except Exception as exc:
            print(f"[!] 视觉 pipeline 清理异常，已忽略: {exc}")
        if show_window:
            try:
                cv2.destroyWindow(window_name)
            except Exception:
                pass
        try:
            client.disconnect()
        except Exception as exc:
            print(f"[!] 视觉 rosbridge 清理异常，已忽略: {exc}")

    threading.Thread(target=_cleanup, daemon=True).start()


def _run_vision(ws_url: str, seconds: float, preferred_label: str,
                show_window: bool = False, window_name: str = "MPC Perception Lock",
                frame_timeout: float = 5.0) -> tuple[dict | None, str | None]:
    logger = DataLogger()
    pipeline = VisionPipeline()
    client = ROSClient(ws_url=ws_url)
    latest_result = None
    best_target = None
    start = time.time()
    last_frame_count = -1
    last_perf_time = time.time()
    last_stats = None
    sample_frames = 0
    sample_detects = 0

    if not client.connect():
        _cleanup_vision_async(pipeline, client, show_window, window_name)
        raise RuntimeError(f"无法连接视觉 rosbridge: {ws_url}")
    last_stats = client.get_stats()
    print(f"[*] 开始视觉检测 {seconds:.1f}s，目标类别: {preferred_label}")
    if show_window:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        print(f"[*] 已打开视觉窗口: {window_name} | q=提前结束检测")
    while time.time() - start < seconds:
        rgb, depth, cam_info, fc = client.get_frames()
        if rgb is None or fc == last_frame_count:
            if sample_frames == 0 and time.time() - start > frame_timeout:
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
        raw_rgb, _, raw_rgb_updated_at = client.get_raw_rgb()
        result = pipeline.process(
            rgb=rgb,
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
        target = select_grasp_target(result["object_results"], preferred_label=preferred_label)
        if target is not None and (
            best_target is None or object_conf(target) > object_conf(best_target)
        ):
            best_target = target
        if show_window:
            cv2.imshow(window_name, result["annotated"])
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
    if best_target is not None:
        print(f"[✓] 选择目标: {summarize_target(best_target)}")
    _cleanup_vision_async(pipeline, client, show_window, window_name)
    return best_target, csv_path


def _lock_target(control_client, target: dict, csv_path: str, cam2head_path: str,
                 output_path: str, tf_seconds: float, approach_height: float):
    cam2head = _load_transform(cam2head_path, "cam2head")
    point_cam = np.array(_camera_point_m(target), dtype=float)
    point_head = (cam2head @ np.array([*point_cam, 1.0], dtype=float))[:3]

    print(f"[*] 采样 TF {tf_seconds:.1f}s，查找 BASE -> HEAD")
    transforms = _sample_tf(control_client, tf_seconds)
    head_to_base = _lookup_transform(transforms, "BASE", "HEAD")
    if head_to_base is None:
        _print_tf_debug(transforms)
        raise RuntimeError("TF 中没有找到 BASE -> HEAD，不能锁存 BASE 目标")

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
    parser.add_argument("--neck-down-y", type=float, default=0.40)
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
    parser.add_argument("--neck-only", action="store_true", help="只执行 neck down/home，不运行视觉检测")
    parser.add_argument("--skip-neck-down", action="store_true")
    parser.add_argument("--skip-neck-home", action="store_true")
    args = parser.parse_args()

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
    try:
        if args.neck_backend == "mpc" and args.enable_mpc_for_neck and (not args.skip_neck_down or not args.skip_neck_home):
            _set_mpc_mode(control_client, True)
        if args.neck_backend == "mpc" and args.enable_neck_track and (not args.skip_neck_down or not args.skip_neck_home):
            _set_neck_track(control_client, True)

        if not args.skip_neck_down:
            if args.neck_backend == "mpc":
                _call_neck(
                    control_client,
                    args.neck_down_z,
                    args.neck_down_y,
                    args.neck_time,
                    verify=args.verify_neck,
                    tolerance=args.neck_verify_tolerance,
                )
            else:
                print("[manual] 请人工确认头部已低头到检测姿态")
            time.sleep(args.settle_seconds)

        if args.neck_only:
            print("[*] neck-only 模式：跳过视觉检测，只执行未被 skip 的 neck 动作")
        else:
            target, csv_path = _run_vision(
                args.ws_url,
                args.detect_seconds,
                args.preferred_label,
                show_window=args.show_window,
                window_name=args.window_name,
                frame_timeout=args.frame_timeout,
            )
            if target is None or csv_path is None:
                raise RuntimeError("没有检测到 valid 目标，未锁存 BASE 坐标")

            _lock_target(
                control_client=control_client,
                target=target,
                csv_path=csv_path,
                cam2head_path=args.cam2head,
                output_path=args.output,
                tf_seconds=args.tf_seconds,
                approach_height=args.approach_height,
            )

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
                    _call_neck(
                        control_client,
                        args.neck_home_z,
                        args.neck_home_y,
                        args.neck_time,
                        verify=args.verify_neck,
                        tolerance=args.neck_verify_tolerance,
                    )
                else:
                    print("[manual] 请人工确认头部已复位")
            except Exception as home_exc:
                if primary_error is not None:
                    print(f"[!] 主流程已失败，尝试 neck home 也失败: {home_exc}")
                else:
                    raise
        try:
            control_client.terminate()
        except Exception:
            pass


if __name__ == "__main__":
    main()
