#!/usr/bin/env python3
"""Detect transport-box dual grasp points with the bundled FoundationPose crate pipeline.

Output schema is intentionally compatible with lock_box_grasp_target.py.
"""

from __future__ import annotations

import argparse
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
FOUNDATIONPOSE_ROOT = PROJECT_ROOT / "third_party" / "foundationpose_crate"
DEFAULT_OUTPUT = PROJECT_ROOT / "data" / "transport" / "box_grasp_target_latest.json"
DEFAULT_DEBUG_DIR = PROJECT_ROOT / "data" / "transport" / "foundationpose_box_grasp_debug_latest"
MPC_MODE_SERVICE = "/wa/wa_hardware_interface/mpc_mode_setting"
NECK_SERVICE = "/wa/wa_hardware_interface/neck_movej"
JOINT_STATES_TOPIC = "/zj_humanoid/upperlimb/joint_states"


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

    pairs = [(left, right), (right, left)]
    corrections = []
    for obj, reference in pairs:
        if obj.get("occluded") is True and reference.get("occluded") is False:
            old_z = obj["z_mm"]
            obj["z_mm"] = reference["z_mm"]
            obj["height_source"] = "fallback_from_unoccluded_side"
            obj["height_reference_side"] = reference["side"]
            obj["height_original_z_mm"] = old_z
            obj["height_corrected_z_mm"] = obj["z_mm"]
            corrections.append(
                {
                    "side": obj["side"],
                    "reference_side": reference["side"],
                    "old_z_mm": old_z,
                    "new_z_mm": obj["z_mm"],
                    "reason": "side_occluded",
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


def _load_best_grasp_points(output_dir: Path) -> tuple[dict, Path]:
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
    grasp, grasp_path = _load_best_grasp_points(output_dir)
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
            "height_policy": "if one valid side is occluded, reuse the unoccluded side z",
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
    parser.add_argument("--grasp-prediction-min-inliers", type=int, default=20)
    parser.add_argument("--grasp-prediction-min-edge-span", type=float, default=0.20)
    parser.add_argument("--grasp-prediction-max-rmse", type=float, default=0.012)
    parser.add_argument("--show-window", action="store_true")
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
                "    height fallback: "
                f"{correction['side']} z {correction['old_z_mm']} -> {correction['new_z_mm']} mm "
                f"from {correction['reference_side']}",
                flush=True,
            )
        print(f"[✓] foundationpose: debug images: {debug_dir}", flush=True)
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
