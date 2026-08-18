#!/usr/bin/env python3
"""Detect blue box left/right grasp points from the head RealSense.

This is a read-only transport-stage tool. It does not send motion commands.
"""

from __future__ import annotations

import argparse
import json
import os
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

from robot_grasp.common.ros_client import ROSClient
from robot_grasp.vision.blue_box import (
    blue_box_to_object_results,
    camera_from_camera_info,
    draw_blue_box_result,
    estimate_blue_box_grasp,
    estimate_blue_box_grasp_from_mask,
    segment_blue,
)
from robot_grasp.vision.box_rim import (
    draw_box_rim_result,
    estimate_box_rim_grasp_from_mask,
)
from tools.debug.debug_fastsam_box_opening import (  # noqa: E402
    _detect_opening_by_hough,
    _draw_candidates,
    _draw_hough_opening,
    _objects_from_hough,
    _select_box_mask,
)
from ultralytics import YOLO


DEFAULT_OUTPUT = PROJECT_ROOT / "data" / "transport" / "box_grasp_target_latest.json"
DEFAULT_SNAPSHOT = PROJECT_ROOT / "data" / "transport" / "box_grasp_target_latest.png"
DEFAULT_DEBUG_DIR = PROJECT_ROOT / "data" / "transport" / "box_grasp_debug_latest"
DEFAULT_COMPARE_DIR = PROJECT_ROOT / "data" / "transport" / "box_grasp_compare_latest"
DEFAULT_FASTSAM_MODEL = PROJECT_ROOT / "models" / "yolo" / "FastSAM-s.pt"
MPC_MODE_SERVICE = "/wa/wa_hardware_interface/mpc_mode_setting"
NECK_SERVICE = "/wa/wa_hardware_interface/neck_movej"
JOINT_STATES_TOPIC = "/zj_humanoid/upperlimb/joint_states"


def _suppress_roslibpy_shutdown_noise() -> None:
    """Hide Twisted ReactorNotRunning tracebacks emitted during roslibpy cleanup."""
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
    host = ws_url.replace("ws://", "").split(":")[0]
    port = int(ws_url.replace("ws://", "").split(":")[1])
    client = roslibpy.Ros(host=host, port=port)
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
    print(f"[mpc_mode={enabled}] {response}")
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
    print(f"[*] neck_movej -> z={neck_z:.3f}, y={neck_y:.3f}, t={duration:.1f}s")
    before = _wait_for_neck_state(client, timeout=1.0) if verify else None
    if before is not None:
        print(f"    before Neck_Z={before[0]:.3f}, Neck_Y={before[1]:.3f}")

    response = _call_service(
        client,
        NECK_SERVICE,
        {"neck_joint": [float(neck_z), float(neck_y)], "t": float(duration)},
    )
    print(f"[neck] {response}")
    if response and response.get("success") is False:
        raise RuntimeError(f"neck_movej 返回失败: {response}")

    if not verify:
        time.sleep(duration)
        return

    time.sleep(duration + 0.3)
    after = _wait_for_neck_state(client, timeout=2.0)
    if after is None:
        print(f"[!] 未读到 {JOINT_STATES_TOPIC}，无法确认 neck 是否到位")
        return
    err_z = abs(after[0] - neck_z)
    err_y = abs(after[1] - neck_y)
    print(f"    after  Neck_Z={after[0]:.3f}, Neck_Y={after[1]:.3f}, err=({err_z:.3f},{err_y:.3f})")
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
    except Exception as exc:
        print(f"[!] rosbridge control 清理异常，已忽略: {exc}")


def _wait_for_frames(client: ROSClient, timeout: float):
    deadline = time.time() + timeout
    while time.time() < deadline:
        rgb, depth, camera_info, _count = client.get_frames()
        if rgb is not None and depth is not None and camera_info is not None:
            return rgb, depth, camera_info
        time.sleep(0.05)
    stats = client.get_stats()
    raise RuntimeError(
        "timeout waiting for RGB/depth/camera_info: "
        f"rgb={stats['rgb_count']} depth_msg={stats['depth_msg_count']} "
        f"depth={stats['depth_count']} camera_info={'yes' if client.camera_info else 'no'}"
    )


def _save_result(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def _depth_to_vis(depth):
    valid = depth[depth > 0]
    if valid.size == 0:
        return cv2.cvtColor(depth.astype("uint8"), cv2.COLOR_GRAY2BGR)
    lo = float(max(0.0, valid.min()))
    hi = float(np.percentile(valid, 95))
    if hi <= lo:
        hi = lo + 1.0
    clipped = np.clip(depth.astype("float32"), lo, hi)
    norm = ((clipped - lo) / (hi - lo) * 255.0).astype("uint8")
    norm[depth <= 0] = 0
    return cv2.applyColorMap(norm, cv2.COLORMAP_JET)


def _save_debug_images(
    debug_dir: Path,
    rgb,
    depth,
    result: dict,
    annotated,
    extra_images: dict[str, object] | None = None,
) -> dict[str, str]:
    debug_dir.mkdir(parents=True, exist_ok=True)
    for old_png in debug_dir.glob("*.png"):
        old_png.unlink()
    paths = {
        "rgb": debug_dir / "rgb.png",
        "depth_vis": debug_dir / "depth_vis.png",
        "blue_mask": debug_dir / "blue_mask.png",
        "annotated": debug_dir / "annotated.png",
    }
    cv2.imwrite(str(paths["rgb"]), rgb)
    cv2.imwrite(str(paths["depth_vis"]), _depth_to_vis(depth))
    mask = result.get("mask")
    if mask is None:
        mask = np.zeros(rgb.shape[:2], dtype=np.uint8)
    cv2.imwrite(str(paths["blue_mask"]), mask)
    cv2.imwrite(str(paths["annotated"]), annotated)
    if extra_images:
        for name, image in extra_images.items():
            path = debug_dir / f"{name}.png"
            cv2.imwrite(str(path), image)
            paths[name] = path
    return {key: str(path) for key, path in paths.items()}


def _estimate_with_fastsam(
    rgb,
    depth,
    camera,
    *,
    model_path: str,
    imgsz: int,
    conf: float,
    handle_y_frac: float,
    geometry: str,
    rim_mode: str,
    rim_fit_mode: str,
):
    model = YOLO(model_path)
    sam_result = model.predict(
        rgb,
        device="cpu",
        retina_masks=True,
        imgsz=imgsz,
        conf=conf,
        iou=0.7,
        verbose=False,
    )[0]
    mask, candidates = _select_box_mask(sam_result, rgb)
    if mask is None:
        result = {"ok": False, "note": "FastSAM produced no candidate mask", "mask": None, "source": "fastsam"}
        return result, [], {}

    fallback = estimate_blue_box_grasp_from_mask(
        rgb,
        depth,
        camera,
        mask,
        handle_y_frac=handle_y_frac,
        geometry=geometry,
        source="fastsam",
    )

    rim_result = {"ok": False, "note": "disabled"}
    rim_objects = []
    rim_images = {}
    if rim_mode in ("depth-rim", "hybrid"):
        rim_result, rim_objects, rim_images = estimate_box_rim_grasp_from_mask(
            rgb,
            depth,
            camera,
            mask,
            handle_y_frac=handle_y_frac,
            rim_fit_mode=rim_fit_mode,
        )

    hough = _detect_opening_by_hough(rgb, mask)
    hough_objects = _objects_from_hough(hough, depth, camera, handle_y_frac, mask) if hough.get("ok") else []

    def _mean_point_distance_px(first_objects, second_objects):
        first = {obj.get("side"): obj.get("center") for obj in first_objects}
        second = {obj.get("side"): obj.get("center") for obj in second_objects}
        distances = []
        for side in ("left", "right"):
            if first.get(side) is None or second.get(side) is None:
                continue
            distances.append(float(np.linalg.norm(np.asarray(first[side], dtype=np.float32) - np.asarray(second[side], dtype=np.float32))))
        return float(np.mean(distances)) if distances else None

    rim_validation = {
        "mode": rim_mode,
        "ok": bool(rim_result.get("ok")),
        "note": rim_result.get("note", ""),
        "edge_support": rim_result.get("edge_support"),
        "mean_point_distance_px": _mean_point_distance_px(hough_objects, rim_objects) if hough_objects and rim_objects else None,
    }

    if rim_mode == "depth-rim" and rim_result.get("ok"):
        result = rim_result
        objects = rim_objects
        annotated = draw_box_rim_result(rgb, result, objects)
        annotated_name = "fastsam_rim"
        result["rim_validation"] = rim_validation | {"decision": "depth_rim_primary"}
    elif hough.get("ok"):
        objects = hough_objects
        result = {
            "ok": any(obj.get("valid") for obj in objects),
            "note": "hybrid_hough_ok" if rim_mode == "hybrid" else ("hough_ok" if any(obj.get("valid") for obj in objects) else "hough_no_depth"),
            "mask": mask,
            "source": "fastsam_hybrid" if rim_mode == "hybrid" else "fastsam_hough",
            "rim_note": rim_result.get("note", ""),
            "rim_validation": rim_validation,
            "hough_opening": {key: value for key, value in hough.items() if key != "edges"},
        }
        if rim_mode == "hybrid":
            distance = rim_validation.get("mean_point_distance_px")
            if rim_result.get("ok") and distance is not None and distance < 70.0:
                result["rim_validation"]["decision"] = "agree_keep_hough"
            elif rim_result.get("ok"):
                result["rim_validation"]["decision"] = "disagree_keep_hough"
            else:
                result["rim_validation"]["decision"] = "rim_failed_keep_hough"
        annotated = _draw_hough_opening(rgb, hough, objects)
        annotated_name = "fastsam_hough"
    else:
        objects = blue_box_to_object_results(fallback)
        result = dict(fallback)
        result["source"] = "fastsam_hybrid" if rim_mode == "hybrid" else result.get("source", "fastsam")
        result["note"] = f"hough_failed:{hough.get('note')}; mask_fallback:{fallback.get('note')}"
        result["rim_validation"] = rim_validation | {"decision": "hough_failed_use_mask_fallback"}
        result["hough_opening"] = {key: value for key, value in hough.items() if key != "edges"}
        annotated = draw_blue_box_result(rgb, fallback, objects)
        annotated_name = "fastsam_fallback"
    cv2.putText(
        annotated,
        result.get("note", ""),
        (20, 36),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255) if hough.get("ok") else (0, 0, 255),
        2,
    )

    extra_images = {
        "sam_candidates": _draw_candidates(rgb, candidates),
        "sam_selected_mask": mask,
        annotated_name: annotated,
    }
    for name, image in rim_images.items():
        extra_images[name] = image
    if rim_mode == "hybrid" and rim_result.get("ok"):
        extra_images["fastsam_rim_validation"] = draw_box_rim_result(rgb, rim_result, rim_objects)
    return result, objects, extra_images


def _run_backend_once(args, rgb, depth, camera, backend: str):
    extra_images = {}
    if backend == "fastsam":
        rim_mode = "depth-rim" if args.geometry == "depth-rim" else args.rim_mode
        result, objects, extra_images = _estimate_with_fastsam(
            rgb,
            depth,
            camera,
            model_path=args.fastsam_model,
            imgsz=args.fastsam_imgsz,
            conf=args.fastsam_conf,
            handle_y_frac=args.handle_y_frac,
            geometry=args.geometry,
            rim_mode=rim_mode,
            rim_fit_mode=args.rim_fit_mode,
        )
        annotated = extra_images.get("fastsam_rim")
        if annotated is None:
            annotated = extra_images.get("fastsam_hough")
        if annotated is None:
            annotated = extra_images.get("fastsam_fallback")
    else:
        if args.geometry == "depth-rim":
            mask = segment_blue(rgb, blue_b_thresh=args.blue_b_thresh)
            if mask is None:
                result = {"ok": False, "note": "no blue box detected", "mask": None, "source": "color_depth_rim"}
                objects = []
                extra_images = {}
                annotated = draw_blue_box_result(rgb, result, objects)
            else:
                result, objects, extra_images = estimate_box_rim_grasp_from_mask(
                    rgb,
                    depth,
                    camera,
                    mask,
                    handle_y_frac=args.handle_y_frac,
                    depth_scale=args.depth_scale,
                    min_edge_support=args.rim_min_edge_support,
                    rim_fit_mode=args.rim_fit_mode,
                )
                result["source"] = "color_depth_rim"
                annotated = draw_box_rim_result(rgb, result, objects)
        else:
            result = estimate_blue_box_grasp(
                rgb,
                depth,
                camera,
                blue_b_thresh=args.blue_b_thresh,
                depth_scale=args.depth_scale,
                patch=args.patch,
                handle_y_frac=args.handle_y_frac,
                geometry=args.geometry,
            )
            objects = blue_box_to_object_results(result)
            annotated = draw_blue_box_result(rgb, result, objects)
    return result, objects, annotated, extra_images


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer, np.floating)):
        return value.item()
    if isinstance(value, dict):
        return {key: _jsonable(item) for key, item in value.items() if key != "mask"}
    if isinstance(value, list):
        return [_jsonable(item) for item in value]
    if isinstance(value, tuple):
        return [_jsonable(item) for item in value]
    return value


def _build_payload(args, backend: str, result: dict, objects: list, camera, debug_paths: dict):
    payload = {
        "type": "blue_box_grasp_points",
        "frame": "camera",
        "ok": bool(result.get("ok")),
        "note": result.get("note", ""),
        "source": result.get("source", ""),
        "objects": objects,
        "camera_info": {
            "fx": camera.fx,
            "fy": camera.fy,
            "cx": camera.cx,
            "cy": camera.cy,
            "width": camera.width,
            "height": camera.height,
        },
        "params": {
            "blue_b_thresh": args.blue_b_thresh,
            "backend": backend,
            "rim_mode": args.rim_mode if backend == "fastsam" else "",
            "rim_min_edge_support": args.rim_min_edge_support,
            "rim_fit_mode": args.rim_fit_mode,
            "fastsam_model": args.fastsam_model if backend == "fastsam" else "",
            "fastsam_imgsz": args.fastsam_imgsz,
            "fastsam_conf": args.fastsam_conf,
            "depth_scale": args.depth_scale,
            "patch": args.patch,
            "handle_y_frac": args.handle_y_frac,
            "geometry": args.geometry,
        },
        "debug_images": debug_paths,
    }
    for key in ("rim", "rim_corners", "rim_source", "rim_meta", "edge_support", "handle_y_frac", "rim_validation", "hough_opening"):
        if key in result:
            payload[key] = _jsonable(result[key])
    return payload


def _print_backend_result(backend: str, output: Path, debug_dir: Path, result: dict, objects: list) -> None:
    if result.get("ok"):
        print(f"[✓] {backend}: detected blue box handles: {output}")
        for obj in objects:
            print(
                f"    {obj['side']}: valid={obj['valid']} "
                f"camera=({obj['x_mm']}, {obj['y_mm']}, {obj['z_mm']}) mm"
            )
    else:
        print(f"[!] {backend}: blue box detection failed: {result.get('note')}")
    print(f"[✓] {backend}: debug images: {debug_dir}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument("--frame-timeout", type=float, default=8.0)
    parser.add_argument("--blue-b-thresh", type=int, default=125)
    parser.add_argument("--backend", choices=["color", "fastsam", "both"], default="color")
    parser.add_argument(
        "--rim-mode",
        choices=["hybrid", "hough", "depth-rim"],
        default="hybrid",
        help="fastsam 后的盒口几何模式；默认 hybrid 使用 hough 主导、depth-rim 校验",
    )
    parser.add_argument("--fastsam-model", default=str(DEFAULT_FASTSAM_MODEL))
    parser.add_argument("--fastsam-imgsz", type=int, default=1024)
    parser.add_argument("--fastsam-conf", type=float, default=0.25)
    parser.add_argument("--depth-scale", type=float, default=0.001)
    parser.add_argument("--patch", type=int, default=7)
    parser.add_argument("--handle-y-frac", type=float, default=0.50)
    parser.add_argument(
        "--geometry",
        choices=["outer", "inner", "auto", "depth-rim"],
        default="outer",
        help="盒口几何来源：outer=旧外轮廓，inner=只用内框Hough，auto=内框失败后回退外轮廓，depth-rim=RGB+深度边缘拟合斜盒口",
    )
    parser.add_argument("--rim-min-edge-support", type=float, default=0.08)
    parser.add_argument(
        "--rim-fit-mode",
        choices=["free", "front-parallel", "side-mid"],
        default="free",
        help="depth-rim 盒口拟合：free=原四线自由拟合，front-parallel=实验版红线约束绿线，side-mid=左右侧壁中点",
    )
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--snapshot", default=str(DEFAULT_SNAPSHOT))
    parser.add_argument("--debug-dir", default=str(DEFAULT_DEBUG_DIR))
    parser.add_argument("--compare-dir", default=str(DEFAULT_COMPARE_DIR))
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
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    if not args.quiet:
        print("=" * 70)
        print("  Blue box grasp-point detection (read-only)")
        print("=" * 70)
        print(f"  WebSocket: {args.ws_url}")
        print(f"  Output: {args.output}")
        print(f"  Snapshot: {args.snapshot}")
        print(f"  Debug dir: {args.debug_dir}")
        print(f"  Compare dir: {args.compare_dir}")
        print(f"  Backend: {args.backend}")
        print(f"  Geometry: {args.geometry}")
        print(f"  Neck down: [{args.neck_down_z}, {args.neck_down_y}]")
        print(f"  Neck home: [{args.neck_home_z}, {args.neck_home_y}]")

    control_client = None
    client = None
    verify_neck = not args.no_neck_verify

    try:
        if not args.skip_neck_down or not args.skip_neck_home:
            control_client = _connect_control(args.ws_url)
            _set_mpc_mode(control_client, True)

        if not args.skip_neck_down:
            _move_neck(
                control_client,
                args.neck_down_z,
                args.neck_down_y,
                args.neck_time,
                verify=verify_neck,
                tolerance=args.neck_verify_tolerance,
            )

        client = ROSClient(args.ws_url)
        if not client.connect():
            raise RuntimeError(f"failed to connect rosbridge: {args.ws_url}")

        rgb, depth, camera_info = _wait_for_frames(client, args.frame_timeout)
        if rgb.shape[:2] != depth.shape[:2]:
            raise RuntimeError(f"RGB/depth size mismatch: rgb={rgb.shape}, depth={depth.shape}")

        camera = camera_from_camera_info(camera_info)

        if args.backend == "both":
            compare_dir = Path(args.compare_dir)
            compare_dir.mkdir(parents=True, exist_ok=True)
            summary = {"type": "blue_box_grasp_points_compare", "backends": {}}
            window_images = []
            for backend in ("color", "fastsam"):
                result, objects, annotated, extra_images = _run_backend_once(args, rgb, depth, camera, backend)
                backend_dir = compare_dir / backend
                output = backend_dir / "box_grasp_target_latest.json"
                snapshot = backend_dir / "box_grasp_target_latest.png"
                debug_dir = backend_dir / "debug"
                snapshot.parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(snapshot), annotated)
                debug_paths = _save_debug_images(debug_dir, rgb, depth, result, annotated, extra_images)
                payload = _build_payload(args, backend, result, objects, camera, debug_paths)
                _save_result(output, payload)
                summary["backends"][backend] = {
                    "ok": bool(result.get("ok")),
                    "note": result.get("note", ""),
                    "output": str(output),
                    "snapshot": str(snapshot),
                    "debug_dir": str(debug_dir),
                    "objects": objects,
                }
                _print_backend_result(backend, output, debug_dir, result, objects)
                window_images.append((backend, annotated))
            _save_result(compare_dir / "compare_summary.json", summary)
            print(f"[✓] compare summary: {compare_dir / 'compare_summary.json'}")
            if args.show_window:
                for backend, annotated in window_images:
                    cv2.imshow(f"blue box grasp points - {backend}", annotated)
                print("[*] q=退出")
                while True:
                    if cv2.waitKey(30) & 0xFF == ord("q"):
                        break
                cv2.destroyAllWindows()
        else:
            result, objects, annotated, extra_images = _run_backend_once(args, rgb, depth, camera, args.backend)
            snapshot = Path(args.snapshot)
            snapshot.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(snapshot), annotated)
            debug_paths = _save_debug_images(Path(args.debug_dir), rgb, depth, result, annotated, extra_images)
            payload = _build_payload(args, args.backend, result, objects, camera, debug_paths)
            _save_result(Path(args.output), payload)
            _print_backend_result(args.backend, Path(args.output), Path(args.debug_dir), result, objects)
            if not result.get("ok"):
                print(f"    snapshot: {snapshot}")

            if args.show_window:
                cv2.imshow("blue box grasp points", annotated)
                print("[*] q=退出")
                while True:
                    if cv2.waitKey(30) & 0xFF == ord("q"):
                        break
                cv2.destroyAllWindows()
    finally:
        if control_client is not None and not args.skip_neck_home:
            try:
                _move_neck(
                    control_client,
                    args.neck_home_z,
                    args.neck_home_y,
                    args.neck_time,
                    verify=verify_neck,
                    tolerance=args.neck_verify_tolerance,
                )
            except Exception as exc:
                print(f"[!] neck home failed: {exc}")
        if client is not None:
            client.disconnect()
        _safe_terminate(control_client)


if __name__ == "__main__":
    main()
