#!/usr/bin/env python3
"""Detect transport-box dual grasp points with the bundled FoundationPose crate pipeline.

Output schema is intentionally compatible with lock_box_grasp_target.py.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import roslibpy
from twisted.internet import error as twisted_error
from twisted.python import failure as twisted_failure
from twisted.python import log as twisted_log


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
FOUNDATIONPOSE_ROOT = PROJECT_ROOT / "third_party" / "foundationpose_crate"
DEFAULT_OUTPUT = PROJECT_ROOT / "data" / "transport" / "box_grasp_target_latest.json"
DEFAULT_BASE_OUTPUT = PROJECT_ROOT / "data" / "runtime" / "transport_box_grasp_target_latest.json"
DEFAULT_DEBUG_DIR = PROJECT_ROOT / "data" / "transport" / "foundationpose_box_grasp_debug_latest"
MPC_MODE_SERVICE = "/wa/wa_hardware_interface/mpc_mode_setting"
NECK_SERVICE = "/wa/wa_hardware_interface/neck_movej"
JOINT_STATES_TOPIC = "/zj_humanoid/upperlimb/joint_states"

from apps.grasp.visual_grasp_test_impl import DEFAULT_CAM2HEAD, _load_transform, _lookup_transform, _sample_tf
from apps.transport.lock_box_grasp_target import _camera_obj_to_base, _valid_objects


def _suppress_roslibpy_shutdown_noise() -> None:
    original_err = twisted_log.err

    def quiet_err(_stuff=None, _why=None, **kwargs):
        failure_obj = None
        if isinstance(_stuff, twisted_failure.Failure):
            failure_obj = _stuff
        elif _stuff is None:
            failure_obj = twisted_failure.Failure()
        if failure_obj is not None and failure_obj.check(twisted_error.ReactorNotRunning):
            return None
        return original_err(_stuff, _why, **kwargs)

    twisted_log.err = quiet_err


_suppress_roslibpy_shutdown_noise()


def _connect_control(ws_url: str) -> roslibpy.Ros:
    host_port = ws_url.replace("ws://", "")
    host, port_text = host_port.split(":", 1)
    client = roslibpy.Ros(host=host, port=int(port_text))
    client.run(timeout=8)
    if not client.is_connected:
        raise RuntimeError(f"failed to connect rosbridge for control: {ws_url}")
    return client


def _service_type(client: roslibpy.Ros, service_name: str) -> str:
    rosapi = roslibpy.Service(client, "/rosapi/service_type", "rosapi/ServiceType")
    response = rosapi.call(roslibpy.ServiceRequest({"service": service_name}))
    srv_type = response.get("type", "")
    if not srv_type:
        raise RuntimeError(f"service unavailable or unknown type: {service_name}")
    return srv_type


def _call_service(client: roslibpy.Ros, service_name: str, request: dict) -> dict:
    srv_type = _service_type(client, service_name)
    service = roslibpy.Service(client, service_name, srv_type)
    return service.call(roslibpy.ServiceRequest(request))


def _set_mpc_mode(client: roslibpy.Ros, enabled: bool) -> None:
    response = _call_service(client, MPC_MODE_SERVICE, {"data": bool(enabled)})
    print(f"[mpc_mode={enabled}] {response}", flush=True)
    if response and response.get("success") is False:
        raise RuntimeError(f"MPC mode 设置失败: {response}")


def _wait_for_neck_state(client: roslibpy.Ros, timeout: float = 2.0) -> tuple[float, float] | None:
    latest: dict[str, tuple[float, float]] = {}

    def callback(message):
        names = message.get("name", [])
        positions = message.get("position", [])
        try:
            latest["state"] = (
                float(positions[names.index("Neck_Z")]),
                float(positions[names.index("Neck_Y")]),
            )
        except (ValueError, IndexError, TypeError):
            return

    sub = roslibpy.Topic(client, JOINT_STATES_TOPIC, "sensor_msgs/JointState")
    sub.subscribe(callback)
    deadline = time.time() + timeout
    while time.time() < deadline and "state" not in latest:
        time.sleep(0.05)
    try:
        sub.unsubscribe()
    except Exception:
        pass
    return latest.get("state")


def _move_neck(
    client: roslibpy.Ros,
    neck_z: float,
    neck_y: float,
    duration: float,
    *,
    verify: bool,
    tolerance: float,
) -> None:
    print(f"[*] neck_movej -> z={neck_z:.3f}, y={neck_y:.3f}, t={duration:.1f}s", flush=True)
    before = _wait_for_neck_state(client, timeout=1.0) if verify else None
    if before is not None:
        print(f"    before Neck_Z={before[0]:.3f}, Neck_Y={before[1]:.3f}", flush=True)
    response = _call_service(
        client,
        NECK_SERVICE,
        {"neck_joint": [float(neck_z), float(neck_y)], "t": float(duration)},
    )
    print(f"[neck] {response}", flush=True)
    if response and response.get("success") is False:
        raise RuntimeError(f"neck_movej 返回失败: {response}")
    if not verify:
        time.sleep(duration)
        return
    time.sleep(duration + 0.3)
    after = _wait_for_neck_state(client, timeout=2.0)
    if after is None:
        print(f"[!] 未读到 {JOINT_STATES_TOPIC}，无法确认 neck 是否到位", flush=True)
        return
    err_z = abs(after[0] - neck_z)
    err_y = abs(after[1] - neck_y)
    print(f"    after  Neck_Z={after[0]:.3f}, Neck_Y={after[1]:.3f}, err=({err_z:.3f},{err_y:.3f})", flush=True)
    if err_z > tolerance or err_y > tolerance:
        raise RuntimeError(
            f"neck_movej 调用完成但关节未到目标: target=({neck_z:.3f},{neck_y:.3f}), "
            f"actual=({after[0]:.3f},{after[1]:.3f})"
        )


def _safe_terminate(client: roslibpy.Ros | None) -> None:
    if client is None:
        return
    try:
        client.terminate()
    except Exception:
        pass


def _lock_payload_to_base(
    client: roslibpy.Ros,
    payload: dict,
    output_path: Path,
    cam2head_path: str,
    tf_seconds: float,
) -> None:
    objects = _valid_objects(payload)
    sides = {obj.get("side") for obj in objects}
    if "left" not in sides or "right" not in sides:
        raise RuntimeError(f"缺少有效左右抓取点，不能换算 BASE: sides={sorted(s for s in sides if s)}")

    print(f"[动作] 采样 TF/转换 BASE 抓取点开始 tf={tf_seconds:.1f}s", flush=True)
    cam2head = _load_transform(cam2head_path, "cam2head")
    transforms = _sample_tf(client, tf_seconds)
    head_to_base = _lookup_transform(transforms, "BASE", "HEAD")
    if head_to_base is None:
        raise RuntimeError("TF 中没有找到 BASE -> HEAD，不能锁存盒子 BASE 抓取点")

    locked_objects = [_camera_obj_to_base(obj, cam2head, head_to_base) for obj in objects]
    by_side = {obj["side"]: obj for obj in locked_objects}
    output = {
        "type": "box_grasp_points_base",
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "input": str(payload.get("output_path", "")),
        "cam2head": str(cam2head_path),
        "frame": "BASE",
        "objects": locked_objects,
        "left": {
            "camera": by_side["left"]["camera_m"],
            "head": by_side["left"]["head_m"],
            "base": by_side["left"]["base_m"],
        },
        "right": {
            "camera": by_side["right"]["camera_m"],
            "head": by_side["right"]["head_m"],
            "base": by_side["right"]["base_m"],
        },
        "note": "Locked BASE box grasp points. Reuse after neck pose changes; do not recompute from old camera points with a different HEAD TF.",
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(output, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"[✓] 已锁存盒子 BASE 抓取点: {output_path}", flush=True)
    print(
        f"    left base : [{by_side['left']['base_m'][0]:.4f}, "
        f"{by_side['left']['base_m'][1]:.4f}, {by_side['left']['base_m'][2]:.4f}]",
        flush=True,
    )
    print(
        f"    right base: [{by_side['right']['base_m'][0]:.4f}, "
        f"{by_side['right']['base_m'][1]:.4f}, {by_side['right']['base_m'][2]:.4f}]",
        flush=True,
    )


def _bbox_from_polygon(points: list[list[float]] | None) -> list[float] | None:
    if not points:
        return None
    arr = np.asarray(points, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 2:
        return None
    x1, y1 = np.min(arr, axis=0)
    x2, y2 = np.max(arr, axis=0)
    return [float(x1), float(y1), float(x2), float(y2)]


def _point_to_object(point: dict, index: int) -> dict:
    camera_m = point.get("point_camera_m") or point.get("cad_midpoint_in_camera_m")
    if camera_m is None or len(camera_m) != 3:
        raise RuntimeError(f"FoundationPose grasp point #{index} has no camera point")
    center = point.get("sample_pixel") or point.get("prediction_pixel") or point.get("projected_midpoint_pixel")
    side = point.get("image_slot") or ("left" if index == 0 else "right")
    return {
        "idx": index,
        "label": "plastic_crate",
        "side": side,
        "valid": bool(point.get("coordinate_valid")),
        "status": "valid" if point.get("coordinate_valid") else "invalid",
        "confidence": 1.0,
        "x_mm": round(float(camera_m[0]) * 1000.0, 1),
        "y_mm": round(float(camera_m[1]) * 1000.0, 1),
        "z_mm": round(float(camera_m[2]) * 1000.0, 1),
        "center": center,
        "bbox": _bbox_from_polygon(point.get("safe_rim_polygon_pixels")),
        "source": point.get("source"),
        "occluded": point.get("occluded"),
        "grasp_clear": point.get("grasp_clear"),
        "cad_short_edge": point.get("cad_short_edge"),
        "height_source": "measured",
    }


def _apply_occluded_height_fallback(objects: list[dict]) -> list[dict]:
    valid_by_side = {obj.get("side"): obj for obj in objects if obj.get("valid")}
    left = valid_by_side.get("left")
    right = valid_by_side.get("right")
    if left is None or right is None:
        return []

    corrections = []
    average_z = round((float(left["z_mm"]) + float(right["z_mm"])) * 0.5, 1)
    for obj in (left, right):
        old_z = obj["z_mm"]
        obj["z_mm"] = average_z
        obj["height_source"] = "average_valid_sides"
        obj["height_original_z_mm"] = old_z
        obj["height_corrected_z_mm"] = obj["z_mm"]
        corrections.append(
            {
                "side": obj["side"],
                "old_z_mm": old_z,
                "new_z_mm": obj["z_mm"],
                "reason": "average_valid_left_right",
                "occluded": obj.get("occluded"),
            }
        )
    return corrections


def _has_valid_left_right(grasp: dict) -> bool:
    sides = set()
    for index, point in enumerate(grasp.get("points_left_to_right") or []):
        side = point.get("image_slot") or ("left" if index == 0 else "right")
        if point.get("coordinate_valid") is True:
            sides.add(side)
    return "left" in sides and "right" in sides


def _side_points(grasp: dict) -> dict[str, dict]:
    by_side = {}
    for index, point in enumerate(grasp.get("points_left_to_right") or []):
        side = point.get("image_slot") or ("left" if index == 0 else "right")
        if point.get("coordinate_valid") is True:
            by_side[side] = point
    return by_side


def _load_valid_grasp_history(output_dir: Path) -> list[dict]:
    history_path = output_dir / "valid_grasp_points_history.jsonl"
    if not history_path.exists():
        return []
    history = []
    for line in history_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            grasp = json.loads(line)
        except json.JSONDecodeError:
            continue
        if _has_valid_left_right(grasp):
            history.append(grasp)
    return history


def _average_grasp_history(history: list[dict], average_frames: int) -> tuple[dict, Path | None]:
    selected = history[-max(1, int(average_frames)) :]
    base = copy.deepcopy(selected[-1])
    point_map = _side_points(base)
    frame_count_by_side: dict[str, int] = {}
    for side in ("left", "right"):
        samples = []
        for grasp in selected:
            point = _side_points(grasp).get(side)
            if not point:
                continue
            camera_m = point.get("point_camera_m") or point.get("cad_midpoint_in_camera_m")
            if camera_m is None or len(camera_m) != 3:
                continue
            samples.append([float(value) for value in camera_m])
        if not samples or side not in point_map:
            continue
        mean = np.asarray(samples, dtype=np.float64).mean(axis=0).tolist()
        point_map[side]["point_camera_m"] = mean
        point_map[side]["source"] = "averaged_valid_frames"
        point_map[side]["average_frame_count"] = len(samples)
        point_map[side]["average_source"] = "valid_grasp_points_history.jsonl"
        frame_count_by_side[side] = len(samples)
    base["valid_average_frame_count"] = len(selected)
    base["valid_average_frame_count_by_side"] = frame_count_by_side
    base["selection_policy"] = "average_recent_valid_frames"
    base["reason_codes"] = list(base.get("reason_codes") or [])
    return base, None


def _load_best_grasp_points(output_dir: Path, average_frames: int) -> tuple[dict, Path]:
    history = _load_valid_grasp_history(output_dir)
    if history:
        averaged, _ = _average_grasp_history(history, average_frames)
        history_path = output_dir / "valid_grasp_points_history.jsonl"
        print(
            f"[✓] 使用最近 {averaged.get('valid_average_frame_count')} 帧 valid FoundationPose 抓取点平均",
            flush=True,
        )
        return averaged, history_path

    latest_path = output_dir / "latest_grasp_points.json"
    if not latest_path.exists():
        raise RuntimeError(f"FoundationPose did not write grasp points: {latest_path}")
    latest = json.loads(latest_path.read_text(encoding="utf-8"))
    if _has_valid_left_right(latest):
        return latest, latest_path

    last_valid_path = output_dir / "last_valid_grasp_points_DIAGNOSTIC_ONLY.json"
    if last_valid_path.exists():
        last_valid = json.loads(last_valid_path.read_text(encoding="utf-8"))
        if _has_valid_left_right(last_valid):
            print(
                "[!] latest_grasp_points 缺少有效左右点，改用最后一组有效 FoundationPose 抓取点",
                flush=True,
            )
            return last_valid, last_valid_path
    return latest, latest_path


def _load_camera_info(camera_matrix_path: Path, latest_image: Path) -> dict:
    matrix = np.loadtxt(camera_matrix_path).reshape(3, 3)
    width = 0
    height = 0
    image = cv2.imread(str(latest_image), cv2.IMREAD_COLOR)
    if image is not None:
        height, width = image.shape[:2]
    return {
        "fx": float(matrix[0, 0]),
        "fy": float(matrix[1, 1]),
        "cx": float(matrix[0, 2]),
        "cy": float(matrix[1, 2]),
        "width": int(width),
        "height": int(height),
    }


def _convert_foundationpose_output(output_dir: Path, output_path: Path, args: argparse.Namespace) -> dict:
    grasp, grasp_path = _load_best_grasp_points(output_dir, args.grasp_average_frames)
    points = grasp.get("points_left_to_right") or []
    if len(points) != 2:
        raise RuntimeError(f"FoundationPose did not produce two grasp points: reasons={grasp.get('reason_codes')}")
    objects = [_point_to_object(point, index) for index, point in enumerate(points)]
    sides = {obj["side"] for obj in objects if obj["valid"]}
    if "left" not in sides or "right" not in sides:
        raise RuntimeError(f"FoundationPose grasp points invalid: sides={sorted(sides)}, reasons={grasp.get('reason_codes')}")
    height_corrections = _apply_occluded_height_fallback(objects)

    debug_paths = {
        "latest": str(output_dir / "latest.png"),
        "initial_rgb": str(output_dir / "initial_rgb.png"),
        "initial_depth_mm": str(output_dir / "initial_depth_mm.png"),
        "initial_mask": str(output_dir / "initial_mask.png"),
        "latest_pose": str(output_dir / "latest_pose.txt"),
        "latest_grasp_points": str(grasp_path),
    }
    payload = {
        "type": "blue_box_grasp_points",
        "frame": "camera",
        "ok": bool(grasp.get("valid")),
        "note": "foundationpose_crate",
        "source": "foundationpose",
        "objects": objects,
        "height_corrections": height_corrections,
        "camera_info": _load_camera_info(Path(args.camera_matrix), output_dir / "latest.png"),
        "params": {
            "backend": "foundationpose",
            "foundationpose_root": str(FOUNDATIONPOSE_ROOT),
            "selected_grasp_points": str(grasp_path),
            "max_frames": args.max_frames,
            "mask_profile": args.mask_profile or "",
            "register_iteration": args.register_iteration,
            "track_iteration": args.track_iteration,
        },
        "foundationpose": {
            "valid": grasp.get("valid"),
            "coordinate_valid": grasp.get("coordinate_valid"),
            "robot_execution_allowed": grasp.get("robot_execution_allowed"),
            "reason_codes": grasp.get("reason_codes", []),
            "sequence": grasp.get("sequence"),
            "crate_color_profile": grasp.get("crate_color_profile"),
            "rgb_depth_delta_ms": grasp.get("rgb_depth_delta_ms"),
            "height_policy": "if left/right are both valid, use their average z for both sides",
            "selection_policy": grasp.get("selection_policy", "latest_or_last_valid"),
            "valid_average_frame_count": grasp.get("valid_average_frame_count"),
            "valid_average_frame_count_by_side": grasp.get("valid_average_frame_count_by_side"),
        },
        "debug_images": debug_paths,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return payload


def _run_foundationpose(args: argparse.Namespace, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable,
        "run_live.py",
        "--ws-url",
        args.ws_url,
        "--output-dir",
        str(output_dir),
        "--max-frames",
        str(args.max_frames),
        "--save-every",
        str(args.save_every),
        "--register-iteration",
        str(args.register_iteration),
        "--track-iteration",
        str(args.track_iteration),
        "--camera-matrix",
        str(args.camera_matrix),
        "--mesh",
        str(args.mesh),
        "--metadata",
        str(args.metadata),
        "--mask-config",
        str(args.mask_config),
        "--grasp-depth-tolerance",
        str(args.grasp_depth_tolerance),
        "--grasp-center-color-min-ratio",
        str(args.grasp_center_color_min_ratio),
        "--grasp-prediction-min-inliers",
        str(args.grasp_prediction_min_inliers),
        "--grasp-prediction-min-edge-span",
        str(args.grasp_prediction_min_edge_span),
        "--grasp-prediction-max-rmse",
        str(args.grasp_prediction_max_rmse),
    ]
    if args.mask_profile:
        cmd.extend(["--mask-profile", args.mask_profile])
    if args.show_window:
        cmd.append("--show")
    subprocess.run(cmd, cwd=FOUNDATIONPOSE_ROOT, check=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="FoundationPose transport-box grasp-point detection.")
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--debug-dir", default=str(DEFAULT_DEBUG_DIR))
    parser.add_argument("--max-frames", type=int, default=8)
    parser.add_argument("--save-every", type=int, default=1)
    parser.add_argument("--register-iteration", type=int, default=5)
    parser.add_argument("--track-iteration", type=int, default=2)
    parser.add_argument("--mask-profile")
    parser.add_argument("--camera-matrix", default=str(FOUNDATIONPOSE_ROOT / "config" / "cam_K.txt"))
    parser.add_argument("--mesh", default=str(FOUNDATIONPOSE_ROOT / "assets" / "plastic_crate_m.obj"))
    parser.add_argument("--metadata", default=str(FOUNDATIONPOSE_ROOT / "assets" / "crate_metadata.json"))
    parser.add_argument("--mask-config", default=str(FOUNDATIONPOSE_ROOT / "config" / "bootstrap_mask.json"))
    parser.add_argument("--grasp-depth-tolerance", type=float, default=0.08)
    parser.add_argument("--grasp-center-color-min-ratio", type=float, default=0.25)
    parser.add_argument("--grasp-average-frames", type=int, default=5, help="Average the latest N valid FoundationPose grasp frames.")
    parser.add_argument("--grasp-prediction-min-inliers", type=int, default=20)
    parser.add_argument("--grasp-prediction-min-edge-span", type=float, default=0.20)
    parser.add_argument("--grasp-prediction-max-rmse", type=float, default=0.012)
    parser.add_argument("--show-window", action="store_true")
    parser.add_argument("--skip-base-lock", action="store_true", help="Only save camera-frame grasp points.")
    parser.add_argument("--base-output", default=str(DEFAULT_BASE_OUTPUT))
    parser.add_argument("--cam2head", default=DEFAULT_CAM2HEAD)
    parser.add_argument("--tf-seconds", type=float, default=2.0)
    parser.add_argument("--skip-neck-down", action="store_true")
    parser.add_argument("--skip-neck-home", action="store_true")
    parser.add_argument("--neck-down-z", type=float, default=0.0)
    parser.add_argument("--neck-down-y", type=float, default=0.35)
    parser.add_argument("--neck-home-z", type=float, default=0.0)
    parser.add_argument("--neck-home-y", type=float, default=0.0)
    parser.add_argument("--neck-time", type=float, default=4.0)
    parser.add_argument("--neck-verify-tolerance", type=float, default=0.10)
    parser.add_argument("--no-neck-verify", action="store_true")
    args = parser.parse_args()

    output_path = Path(args.output)
    debug_dir = Path(args.debug_dir)
    control_client = None
    try:
        control_client = _connect_control(args.ws_url)
        _set_mpc_mode(control_client, True)
        if not args.skip_neck_down:
            _move_neck(
                control_client,
                args.neck_down_z,
                args.neck_down_y,
                args.neck_time,
                verify=not args.no_neck_verify,
                tolerance=args.neck_verify_tolerance,
            )
        print("[动作] FoundationPose 盒子识别开始", flush=True)
        _run_foundationpose(args, debug_dir)
        payload = _convert_foundationpose_output(debug_dir, output_path, args)
        payload["output_path"] = str(output_path)
        print(f"[✓] foundationpose: detected box handles: {output_path}", flush=True)
        for obj in payload["objects"]:
            print(
                f"    {obj['side']}: valid={obj['valid']} "
                f"camera=({obj['x_mm']}, {obj['y_mm']}, {obj['z_mm']}) mm "
                f"occluded={obj.get('occluded')} height={obj.get('height_source')}",
                flush=True,
            )
        for correction in payload.get("height_corrections", []):
            print(
                "    height align: "
                f"{correction['side']} z {correction['old_z_mm']} -> {correction['new_z_mm']} mm "
                f"reason={correction['reason']} occluded={correction.get('occluded')}",
                flush=True,
            )
        print(f"[✓] foundationpose: debug images: {debug_dir}", flush=True)
        if not args.skip_base_lock:
            _lock_payload_to_base(
                control_client,
                payload,
                Path(args.base_output),
                args.cam2head,
                args.tf_seconds,
            )
    finally:
        if control_client is not None and not args.skip_neck_home:
            _move_neck(
                control_client,
                args.neck_home_z,
                args.neck_home_y,
                args.neck_time,
                verify=not args.no_neck_verify,
                tolerance=args.neck_verify_tolerance,
            )
        _safe_terminate(control_client)


if __name__ == "__main__":
    main()
