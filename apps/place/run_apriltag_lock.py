#!/usr/bin/env python3
"""Read-only AprilTag lock for the shelf placement stage.

This script does not move arms. It only:
  1. sets MPC running mode,
  2. moves the neck to a selected look-down angle,
  3. detects AprilTag 36h11 target id,
  4. converts tag pose camera -> HEAD -> BASE,
  5. saves the locked BASE result,
  6. optionally returns the neck home.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import roslibpy


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from apps.grasp.visual_grasp_test_impl import (  # noqa: E402
    DEFAULT_CAM2HEAD,
    _call,
    _load_transform,
    _lookup_transform,
    _sample_tf,
    _service_type,
)
from robot_grasp.common import config  # noqa: E402
from robot_grasp.common.ros_client import ROSClient  # noqa: E402
from robot_grasp.vision.blue_box import camera_from_camera_info  # noqa: E402


NECK_SERVICE = "/wa/wa_hardware_interface/neck_movej"
MPC_MODE_SERVICE = "/wa/wa_hardware_interface/mpc_mode_setting"
JOINT_STATES_TOPIC = "/zj_humanoid/upperlimb/joint_states"
DEFAULT_OUTPUT = PROJECT_ROOT / "data" / "runtime" / "place_apriltag_target_latest.json"
DEFAULT_DEBUG_IMAGE = PROJECT_ROOT / "data" / "runtime" / "place_apriltag_debug_latest.png"
DEFAULT_NECK_TEST_DIR = PROJECT_ROOT / "data" / "runtime" / "place_apriltag_neck_test"
WINDOW_NAME = "Place AprilTag Lock"


class TagDetectionError(RuntimeError):
    def __init__(self, message: str, annotated: np.ndarray, raw: np.ndarray | None = None):
        super().__init__(message)
        self.annotated = annotated
        self.raw = raw


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer, np.floating)):
        return value.item()
    if isinstance(value, dict):
        return {key: _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _save_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_jsonable(payload), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def _show_image(enabled: bool, image: np.ndarray, delay_ms: int = 500) -> None:
    if not enabled:
        return
    cv2.imshow(WINDOW_NAME, image)
    cv2.waitKey(delay_ms)


def _annotate_capture_context(image: np.ndarray, args, neck_y: float) -> np.ndarray:
    out = image.copy()
    shelf = args.shelf_level if args.shelf_level is not None else "manual"
    text = f"shelf={shelf} neck_y={neck_y:.3f} target_ids={args.tag_ids}"
    cv2.putText(out, text, (24, out.shape[0] - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
    return out


def _parse_ws_url(ws_url: str) -> tuple[str, int]:
    stripped = ws_url.replace("ws://", "").replace("wss://", "")
    host, port = stripped.split(":")
    return host, int(port)


def _connect_control(ws_url: str, timeout: float = 5.0):
    host, port = _parse_ws_url(ws_url)
    client = roslibpy.Ros(host=host, port=port)
    thread = threading.Thread(target=client.run, daemon=True)
    thread.start()
    start = time.time()
    while not client.is_connected:
        if time.time() - start > timeout:
            try:
                client.terminate()
            except Exception:
                pass
            raise RuntimeError(f"连接超时: {ws_url}")
        time.sleep(0.1)
    return client


def _set_mpc_mode(client, enabled: bool) -> dict:
    srv_type = _service_type(client, MPC_MODE_SERVICE)
    if not srv_type:
        raise RuntimeError(f"找不到 MPC mode 服务: {MPC_MODE_SERVICE}")
    response = _call(client, MPC_MODE_SERVICE, srv_type, {"data": bool(enabled)})
    print(f"[mpc_mode={enabled}] {response}")
    if response and response.get("success") is False:
        raise RuntimeError(f"MPC mode 设置失败: {response}")
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


def _move_neck(
    client,
    neck_z: float,
    neck_y: float,
    duration: float,
    *,
    verify: bool = True,
    tolerance: float = 0.10,
) -> dict:
    srv_type = _service_type(client, NECK_SERVICE)
    if not srv_type:
        raise RuntimeError(f"找不到 neck 服务类型: {NECK_SERVICE}")
    print(f"[*] neck_movej -> z={neck_z:.3f}, y={neck_y:.3f}, t={duration:.1f}s")
    before = _wait_for_neck_state(client, timeout=1.0) if verify else None
    if before is not None:
        print(f"    before Neck_Z={before[0]:.3f}, Neck_Y={before[1]:.3f}")
    response = _call(
        client,
        NECK_SERVICE,
        srv_type,
        {"neck_joint": [float(neck_z), float(neck_y)], "t": float(duration)},
    )
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


def _wait_for_frames(
    client: ROSClient,
    timeout: float,
    *,
    transport: str = "compressed",
    min_rgb_count: int = 0,
    min_updated_at: float = 0.0,
) -> tuple[np.ndarray, np.ndarray | None, dict]:
    start = time.time()
    while time.time() - start < timeout:
        with client._lock:
            if transport == "raw":
                rgb = None if client.raw_rgb_frame is None else client.raw_rgb_frame.copy()
                rgb_count = client.raw_rgb_count
                rgb_updated_at = client.raw_rgb_updated_at
            else:
                rgb = None if client.rgb_frame is None else client.rgb_frame.copy()
                rgb_count = client.frame_count
                rgb_updated_at = client.rgb_updated_at
            depth = None if client.depth_frame is None else client.depth_frame.copy()
            camera_info = None if client.camera_info is None else dict(client.camera_info)
        if (
            rgb is not None
            and camera_info is not None
            and rgb_count > min_rgb_count
            and rgb_updated_at >= min_updated_at
        ):
            return rgb, depth, camera_info
        time.sleep(0.05)
    raise RuntimeError(
        "等待头部相机超时: "
        f"rgb={client.frame_count}, raw_rgb={client.raw_rgb_count}, "
        f"depth={client.depth_count}, camera_info={'yes' if client.camera_info else 'no'}"
    )


def _clear_vision_cache(client: ROSClient) -> None:
    with client._lock:
        client.rgb_frame = None
        client.raw_rgb_frame = None
        client.depth_frame = None
        client.frame_count = 0
        client.raw_rgb_count = 0
        client.depth_msg_count = 0
        client.depth_count = 0
        client.rgb_updated_at = 0.0
        client.raw_rgb_updated_at = 0.0
        client.depth_updated_at = 0.0


def _camera_matrix_and_dist(camera_info: dict) -> tuple[np.ndarray, np.ndarray]:
    camera = camera_from_camera_info(camera_info)
    k = camera.K.astype(np.float64)
    d = np.asarray(camera_info.get("D") or [], dtype=np.float64).reshape(-1, 1)
    if d.size == 0:
        d = np.zeros((5, 1), dtype=np.float64)
    return k, d


def _aruco_dictionary(name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("当前 OpenCV 没有 cv2.aruco，请安装 opencv-contrib-python")
    if not hasattr(cv2.aruco, name):
        raise RuntimeError(f"当前 OpenCV aruco 不支持 {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def _detect_tags_once(gray: np.ndarray, dictionary, parameters) -> tuple[list[np.ndarray], np.ndarray | None]:
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, _rejected = detector.detectMarkers(gray)
    else:
        corners, ids, _rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    return corners, ids


def _detect_tags(
    rgb: np.ndarray,
    dictionary_name: str,
    preferred_ids: list[int] | None = None,
) -> tuple[list[np.ndarray], np.ndarray | None, str]:
    dictionary = _aruco_dictionary(dictionary_name)
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    preferred = set(preferred_ids or [])

    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    variants: list[tuple[str, np.ndarray, float]] = [("orig", gray, 1.0)]
    for scale in (2.0, 3.0, 4.0):
        variants.append((
            f"x{scale:g}",
            cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC),
            scale,
        ))

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    variants.append(("clahe", clahe, 1.0))
    for scale in (2.0, 3.0, 4.0):
        variants.append((
            f"clahe_x{scale:g}",
            cv2.resize(clahe, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC),
            scale,
        ))

    fallback: tuple[list[np.ndarray], np.ndarray, str] | None = None
    for name, image, scale in variants:
        corners, ids = _detect_tags_once(image, dictionary, parameters)
        if ids is None or len(ids) == 0:
            continue
        if scale != 1.0:
            corners = [corner.astype(np.float32) / float(scale) for corner in corners]
        ids_flat = [int(value) for value in ids.flatten().tolist()]
        if fallback is None or len(ids_flat) > len(fallback[1].flatten().tolist()):
            fallback = (corners, ids, name)
        if preferred and any(tag_id in preferred for tag_id in ids_flat):
            return corners, ids, name
        if not preferred:
            return corners, ids, name
    if fallback is not None:
        return fallback
    return [], None, "none"


def _matrix_to_quaternion(matrix: np.ndarray) -> dict[str, float]:
    r = matrix[:3, :3]
    trace = float(np.trace(r))
    if trace > 0.0:
        s = (trace + 1.0) ** 0.5 * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = (1.0 + r[0, 0] - r[1, 1] - r[2, 2]) ** 0.5 * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = (1.0 + r[1, 1] - r[0, 0] - r[2, 2]) ** 0.5 * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = (1.0 + r[2, 2] - r[0, 0] - r[1, 1]) ** 0.5 * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    quat = np.asarray([x, y, z, w], dtype=np.float64)
    norm = float(np.linalg.norm(quat))
    if norm > 0:
        quat /= norm
    return {"x": float(quat[0]), "y": float(quat[1]), "z": float(quat[2]), "w": float(quat[3])}


def _pose_from_matrix(matrix: np.ndarray) -> dict[str, Any]:
    return {
        "position": {
            "x": float(matrix[0, 3]),
            "y": float(matrix[1, 3]),
            "z": float(matrix[2, 3]),
        },
        "orientation": _matrix_to_quaternion(matrix),
    }


def _estimate_tag_pose(
    rgb: np.ndarray,
    camera_info: dict,
    *,
    dictionary_name: str,
    tag_ids: list[int],
    tag_size_m: float,
) -> tuple[dict[str, Any], np.ndarray]:
    corners, ids, detection_method = _detect_tags(rgb, dictionary_name, preferred_ids=tag_ids)
    annotated = rgb.copy()
    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
    if ids is None or len(ids) == 0:
        cv2.putText(annotated, "NO APRILTAG", (24, 48), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        raise TagDetectionError("未检测到 AprilTag", annotated, rgb)

    ids_flat = [int(value) for value in ids.flatten().tolist()]
    selected_id = next((tag_id for tag_id in tag_ids if tag_id in ids_flat), None)
    if selected_id is None:
        cv2.putText(
            annotated,
            f"DETECTED {ids_flat}, TARGET {tag_ids}",
            (24, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
        )
        raise TagDetectionError(f"检测到 tags={ids_flat}，但没有目标 ids={tag_ids}", annotated, rgb)
    index = ids_flat.index(selected_id)
    target_corners = [corners[index]]
    k, d = _camera_matrix_and_dist(camera_info)
    rvecs, tvecs, _obj_points = cv2.aruco.estimatePoseSingleMarkers(target_corners, tag_size_m, k, d)
    rvec = rvecs[0][0].astype(np.float64)
    tvec = tvecs[0][0].astype(np.float64)
    rotation, _ = cv2.Rodrigues(rvec)
    t_camera_tag = np.eye(4, dtype=np.float64)
    t_camera_tag[:3, :3] = rotation
    t_camera_tag[:3, 3] = tvec
    cv2.drawFrameAxes(annotated, k, d, rvec, tvec, tag_size_m * 1.5)
    center = target_corners[0].reshape(4, 2).mean(axis=0)
    cv2.circle(annotated, (int(round(center[0])), int(round(center[1]))), 6, (0, 0, 255), -1)
    payload = {
        "detected_ids": ids_flat,
        "detection_method": detection_method,
        "target_ids": tag_ids,
        "selected_id": selected_id,
        "corners_px": target_corners[0].reshape(4, 2).tolist(),
        "center_px": center.tolist(),
        "rvec": rvec.tolist(),
        "tvec": tvec.tolist(),
        "transform_4x4": t_camera_tag.tolist(),
        "pose": _pose_from_matrix(t_camera_tag),
    }
    return payload, annotated


def _convert_to_head_base(
    client,
    camera_tag_matrix: np.ndarray,
    *,
    cam2head_path: str,
    tf_sample_seconds: float,
) -> tuple[np.ndarray, np.ndarray]:
    t_head_camera = _load_transform(cam2head_path, "CAM2HEAD")
    transforms = _sample_tf(client, tf_sample_seconds)
    t_base_head = _lookup_transform(transforms, "BASE", "HEAD")
    if t_base_head is None:
        raise RuntimeError("TF 中没有找到 BASE -> HEAD，不能锁存 BASE Tag 位姿")
    t_head_tag = t_head_camera @ camera_tag_matrix
    t_base_tag = t_base_head @ t_head_tag
    return t_head_tag, t_base_tag


def _parse_neck_values(text: str | None) -> list[float]:
    if not text:
        return []
    values = []
    for part in text.split(","):
        part = part.strip()
        if part:
            values.append(float(part))
    return values


def _parse_tag_ids(text: str) -> list[int]:
    values = []
    for part in text.split(","):
        part = part.strip()
        if part:
            values.append(int(part))
    if not values:
        raise ValueError("--tag-ids 不能为空")
    return values


def _parse_tag_level_map(text: str) -> dict[str, str]:
    mapping: dict[str, str] = {}
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if ":" not in part:
            raise ValueError(f"tag level map 格式错误: {part}")
        key, value = part.split(":", 1)
        mapping[str(int(key.strip()))] = value.strip()
    return mapping


def _apply_shelf_defaults(args) -> None:
    if args.shelf_level == 2:
        args.tag_ids = [2]
        args.neck_down_y = 0.40
    elif args.shelf_level == 3:
        args.tag_ids = [3]
        args.neck_down_y = 0.25
    elif args.tag_id is not None:
        args.tag_ids = [int(args.tag_id)]
    else:
        args.tag_ids = _parse_tag_ids(args.tag_ids)


def _run_single_capture(
    args,
    control_client,
    vision_client: ROSClient,
    neck_y: float,
) -> tuple[dict[str, Any], np.ndarray, np.ndarray]:
    if not args.skip_neck_down:
        print(f"[动作] 低头开始 neck_y={neck_y:.3f}")
        _move_neck(
            control_client,
            args.neck_down_z,
            neck_y,
            args.neck_time,
            verify=not args.no_neck_verify,
            tolerance=args.neck_verify_tolerance,
        )
        print("[动作] 低头完成")
        if args.settle_seconds > 0:
            print(f"[动作] 等待头部稳定 {args.settle_seconds:.1f}s")
            time.sleep(args.settle_seconds)
    frame_ready_after = time.time()

    if not vision_client.is_connected:
        print("[动作] 连接相机开始")
        if not vision_client.connect():
            raise RuntimeError(f"连接 rosbridge 失败: {args.ws_url}")
        print("[动作] 连接相机完成")
    else:
        _clear_vision_cache(vision_client)

    with vision_client._lock:
        start_rgb_count = int(vision_client.raw_rgb_count if args.transport == "raw" else vision_client.frame_count)
    min_rgb_count = start_rgb_count + max(0, int(args.discard_frames))
    if args.discard_frames > 0:
        print(f"[动作] 等待低头后的新图像帧 discard={args.discard_frames}")

    rgb, _depth, camera_info = _wait_for_frames(
        vision_client,
        args.frame_timeout,
        transport=args.transport,
        min_rgb_count=min_rgb_count if not args.skip_neck_down else 0,
        min_updated_at=frame_ready_after if not args.skip_neck_down else 0.0,
    )
    if not args.skip_neck_down:
        with vision_client._lock:
            current_rgb_count = int(vision_client.raw_rgb_count if args.transport == "raw" else vision_client.frame_count)
        print(f"[动作] 已获取低头后的新图像帧 count={current_rgb_count}, start={start_rgb_count}")
    tag_payload, annotated = _estimate_tag_pose(
        rgb,
        camera_info,
        dictionary_name=args.dictionary,
        tag_ids=args.tag_ids,
        tag_size_m=args.tag_size_m,
    )
    result: dict[str, Any] = {
        "ok": True,
        "timestamp": time.time(),
        "ws_url": args.ws_url,
        "neck": {"z": args.neck_down_z, "y": neck_y},
        "tag": {
            "family": args.dictionary.replace("DICT_APRILTAG_", "tag"),
            "target_ids": args.tag_ids,
            "selected_id": tag_payload["selected_id"],
            "requested_shelf_level": args.shelf_level,
            "shelf_level": f"shelf{args.shelf_level}" if args.shelf_level is not None else args.tag_level_map.get(str(tag_payload["selected_id"]), "unknown"),
            "size_m": args.tag_size_m,
        },
        "camera": {"tag": tag_payload},
    }

    if not args.neck_test_only:
        print("[动作] 采样 TF 并转换 BASE 开始")
        t_camera_tag = np.asarray(tag_payload["transform_4x4"], dtype=np.float64)
        t_head_tag, t_base_tag = _convert_to_head_base(
            control_client,
            t_camera_tag,
            cam2head_path=args.cam2head,
            tf_sample_seconds=args.tf_sample_seconds,
        )
        result["head"] = {
            "tag_transform_4x4": t_head_tag.tolist(),
            "tag_pose": _pose_from_matrix(t_head_tag),
        }
        result["base"] = {
            "tag_transform_4x4": t_base_tag.tolist(),
            "tag_pose": _pose_from_matrix(t_base_tag),
        }
        print("[动作] 采样 TF 并转换 BASE 完成")

    return result, annotated, rgb


def main() -> None:
    parser = argparse.ArgumentParser(description="Read-only AprilTag lock for shelf placement.")
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument("--dictionary", default="DICT_APRILTAG_36h11")
    parser.add_argument("--shelf-level", type=int, choices=[2, 3], default=None, help="按货架层自动设置 tag-id 和 neck_y")
    parser.add_argument("--tag-id", type=int, default=None, help="兼容旧参数；等价于 --tag-ids 单个值")
    parser.add_argument("--tag-ids", default="2", help="逗号分隔；默认二层 id=2；三层/二层同时测试: 3,2")
    parser.add_argument("--tag-level-map", default="3:shelf3,2:shelf2")
    parser.add_argument("--tag-size-m", type=float, default=0.020)
    parser.add_argument("--cam2head", default=DEFAULT_CAM2HEAD)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--debug-image", type=Path, default=DEFAULT_DEBUG_IMAGE)
    parser.add_argument("--neck-test-dir", type=Path, default=DEFAULT_NECK_TEST_DIR)
    parser.add_argument("--neck-y-values", default="", help="逗号分隔；用于测试两层货架可见性，例如 0.28,0.35,0.42")
    parser.add_argument("--neck-test-only", action="store_true", help="只测试不同 neck_y 是否看得到 tag，不做 BASE 锁存")
    parser.add_argument("--skip-neck-down", action="store_true")
    parser.add_argument("--skip-neck-home", action="store_true")
    parser.add_argument("--neck-down-z", type=float, default=0.0)
    parser.add_argument("--neck-down-y", type=float, default=0.40)
    parser.add_argument("--neck-home-z", type=float, default=0.0)
    parser.add_argument("--neck-home-y", type=float, default=0.0)
    parser.add_argument("--neck-time", type=float, default=4.0)
    parser.add_argument("--neck-verify-tolerance", type=float, default=0.10)
    parser.add_argument("--no-neck-verify", action="store_true")
    parser.add_argument("--frame-timeout", type=float, default=8.0)
    parser.add_argument("--tf-sample-seconds", type=float, default=2.0)
    parser.add_argument("--show-window", action="store_true")
    parser.add_argument("--transport", choices=("compressed", "raw"), default="raw")
    parser.add_argument("--raw-throttle-ms", type=int, default=500)
    parser.add_argument("--settle-seconds", type=float, default=0.8, help="neck 到位后额外等待，再取下一帧")
    parser.add_argument("--discard-frames", type=int, default=0, help="neck 到位后额外丢弃若干新图像帧")
    parser.add_argument("--clean-output", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--terminate-clients", action="store_true", help="默认不主动 terminate roslibpy，避免 twisted 清理噪声")
    args = parser.parse_args()

    if args.transport == "raw":
        config.ENABLE_QR = True
        config.QR_RAW_RGB_THROTTLE_MS = int(args.raw_throttle_ms)
        config.USE_SEPARATE_QR_CLIENT = False
    _apply_shelf_defaults(args)
    args.tag_level_map = _parse_tag_level_map(args.tag_level_map)
    neck_values = _parse_neck_values(args.neck_y_values) or [args.neck_down_y]
    if args.neck_test_only and len(neck_values) == 1 and not args.neck_y_values:
        print("[!] --neck-test-only 未提供 --neck-y-values，将只测试默认 neck_y")

    print("=" * 70)
    print("  Place AprilTag lock/read-only")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  Tag: {args.dictionary}, ids={args.tag_ids}, size={args.tag_size_m:.3f}m")
    print(f"  Transport: {args.transport}")
    print(f"  Neck values: {neck_values}")
    print(f"  Output: {args.output}")
    print("=" * 70)

    control_client = None
    vision_client = None
    summaries: list[dict[str, Any]] = []
    fatal_error: Exception | None = None
    try:
        if args.clean_output and (args.neck_test_only or len(neck_values) > 1):
            args.neck_test_dir.mkdir(parents=True, exist_ok=True)
            for path in args.neck_test_dir.glob("*"):
                if path.is_file():
                    path.unlink()
        control_client = _connect_control(args.ws_url)
        _set_mpc_mode(control_client, True)
        vision_client = ROSClient(args.ws_url)
        for idx, neck_y in enumerate(neck_values, start=1):
            print(f"\n步骤 {idx}/{len(neck_values)} 开始: AprilTag 读取 neck_y={neck_y:.3f}")
            try:
                result, annotated, raw_rgb = _run_single_capture(args, control_client, vision_client, neck_y)
                annotated = _annotate_capture_context(annotated, args, neck_y)
                if args.neck_test_only or len(neck_values) > 1:
                    args.neck_test_dir.mkdir(parents=True, exist_ok=True)
                    stem = f"neck_y_{neck_y:.3f}".replace(".", "p")
                    raw_image_path = args.neck_test_dir / f"{stem}_raw.png"
                    image_path = args.neck_test_dir / f"{stem}_annotated.png"
                    json_path = args.neck_test_dir / f"{stem}.json"
                    cv2.imwrite(str(raw_image_path), raw_rgb)
                    cv2.imwrite(str(image_path), annotated)
                    _save_json(json_path, result)
                    _show_image(args.show_window, annotated)
                    summaries.append({
                        "neck_y": neck_y,
                        "ok": True,
                        "raw_image": str(raw_image_path),
                        "annotated_image": str(image_path),
                        "json": str(json_path),
                    })
                else:
                    raw_debug = args.debug_image.with_name(args.debug_image.stem + "_raw.png")
                    cv2.imwrite(str(raw_debug), raw_rgb)
                    cv2.imwrite(str(args.debug_image), annotated)
                    _save_json(args.output, result)
                    _show_image(args.show_window, annotated)
                selected_id = result["tag"]["selected_id"]
                shelf_level = result["tag"]["shelf_level"]
                camera_pose = result["camera"]["tag"]["pose"]["position"]
                print(f"    selected tag: id={selected_id}, level={shelf_level}")
                print(
                    "    camera tag : "
                    f"x={camera_pose['x']:.3f}, y={camera_pose['y']:.3f}, z={camera_pose['z']:.3f}"
                )
                if "base" in result:
                    base_pose = result["base"]["tag_pose"]["position"]
                    print(
                        "    base tag  : "
                        f"x={base_pose['x']:.3f}, y={base_pose['y']:.3f}, z={base_pose['z']:.3f}"
                    )
                print(f"步骤 {idx}/{len(neck_values)} 完成: AprilTag 读取")
            except Exception as exc:
                print(f"步骤 {idx}/{len(neck_values)} 失败: {exc}")
                annotated = getattr(exc, "annotated", None)
                raw_rgb = getattr(exc, "raw", None)
                if annotated is not None:
                    annotated = _annotate_capture_context(annotated, args, neck_y)
                failure_image_path = None
                failure_raw_path = None
                if annotated is not None and (args.neck_test_only or len(neck_values) > 1):
                    args.neck_test_dir.mkdir(parents=True, exist_ok=True)
                    stem = f"neck_y_{neck_y:.3f}_failed".replace(".", "p")
                    failure_raw_path = args.neck_test_dir / f"{stem}_raw.png"
                    failure_image_path = args.neck_test_dir / f"{stem}_annotated.png"
                    if raw_rgb is not None:
                        cv2.imwrite(str(failure_raw_path), raw_rgb)
                    cv2.imwrite(str(failure_image_path), annotated)
                    _show_image(args.show_window, annotated)
                elif annotated is not None:
                    failure_raw_path = args.debug_image.with_name(args.debug_image.stem + "_failed_raw.png")
                    failure_image_path = args.debug_image.with_name(args.debug_image.stem + "_failed_annotated.png")
                    if raw_rgb is not None:
                        cv2.imwrite(str(failure_raw_path), raw_rgb)
                    cv2.imwrite(str(failure_image_path), annotated)
                    _show_image(args.show_window, annotated)
                    print(f"    [调试图] raw: {failure_raw_path}")
                    print(f"    [调试图] annotated: {failure_image_path}")
                if args.neck_test_only or len(neck_values) > 1:
                    entry = {"neck_y": neck_y, "ok": False, "error": str(exc)}
                    if failure_raw_path is not None and raw_rgb is not None:
                        entry["raw_image"] = str(failure_raw_path)
                    if failure_image_path is not None:
                        entry["annotated_image"] = str(failure_image_path)
                    summaries.append(entry)
                else:
                    fatal_error = exc
                    break

        if fatal_error is not None:
            print(f"\n[✗] AprilTag 锁存失败: {fatal_error}")
        elif summaries:
            args.neck_test_dir.mkdir(parents=True, exist_ok=True)
            summary_path = args.neck_test_dir / "summary.json"
            _save_json(summary_path, {"runs": summaries})
            print(f"\n[✓] 脖子角度测试结果: {summary_path}")
        elif not args.neck_test_only:
            print(f"\n[✓] 已锁存 AprilTag BASE 目标: {args.output}")
            print(f"[✓] 调试图: {args.debug_image}")
    finally:
        if control_client is not None and not args.skip_neck_home:
            try:
                print("[动作] 抬头开始")
                _move_neck(
                    control_client,
                    args.neck_home_z,
                    args.neck_home_y,
                    args.neck_time,
                    verify=not args.no_neck_verify,
                    tolerance=args.neck_verify_tolerance,
                )
                print("[动作] 抬头完成")
            except Exception as exc:
                print(f"[!] 抬头失败，需要人工确认头部状态: {exc}")
        if args.show_window:
            try:
                cv2.destroyWindow(WINDOW_NAME)
                cv2.destroyAllWindows()
                cv2.waitKey(1)
            except Exception:
                pass
        if vision_client is not None:
            if args.terminate_clients:
                try:
                    vision_client.disconnect()
                except Exception:
                    pass
        if control_client is not None:
            if args.terminate_clients:
                try:
                    control_client.terminate()
                except Exception:
                    pass
    if fatal_error is not None:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
