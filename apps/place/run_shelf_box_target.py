#!/usr/bin/env python3
"""Detect the shelf-place target on the center-facing blue box.

Output target:
  the midpoint of the front/top rim edge of the box closest to the configured
  image center. This is read-only; it does not send arm motion commands.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from ultralytics import YOLO


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from apps.transport.run_box_grasp_point import (  # noqa: E402
    _connect_control,
    _move_neck,
    _safe_terminate,
    _set_mpc_mode,
    _wait_for_frames,
)
from robot_grasp.common.ros_client import ROSClient  # noqa: E402
from robot_grasp.vision.blue_box import (  # noqa: E402
    camera_from_camera_info,
    robust_depth_at,
    unproject,
)
from robot_grasp.vision.box_rim import (  # noqa: E402
    draw_box_rim_result,
    estimate_box_rim_grasp_from_mask,
)


DEFAULT_FASTSAM_MODEL = PROJECT_ROOT / "models" / "yolo" / "FastSAM-s.pt"
DEFAULT_OUTPUT = PROJECT_ROOT / "data" / "place" / "shelf_box_target_latest.json"
DEFAULT_SNAPSHOT = PROJECT_ROOT / "data" / "place" / "shelf_box_target_latest.png"
DEFAULT_DEBUG_DIR = PROJECT_ROOT / "data" / "place" / "shelf_box_debug_latest"


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


def _save_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_jsonable(payload), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def _depth_to_vis(depth: np.ndarray) -> np.ndarray:
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


def _mask_candidate_metrics(
    mask: np.ndarray,
    blue_mask: np.ndarray,
    white_mask: np.ndarray,
    image_shape: tuple[int, int],
    target_uv: tuple[float, float],
) -> dict[str, float]:
    h, w = image_shape
    area = float(mask.sum())
    if area <= 0:
        return {"score": -999.0, "center_distance": 999.0, "blue_overlap": 0.0, "area_frac": 0.0, "aspect": 0.0}
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return {"score": -999.0, "center_distance": 999.0, "blue_overlap": 0.0, "area_frac": 0.0, "aspect": 0.0}
    x1, x2 = float(xs.min()), float(xs.max())
    y1, y2 = float(ys.min()), float(ys.max())
    width = x2 - x1 + 1.0
    height = y2 - y1 + 1.0
    area_frac = area / float(h * w)
    width_frac = width / float(w)
    height_frac = height / float(h)
    aspect = width / max(1.0, height)
    center = np.array([(x1 + x2) * 0.5, (y1 + y2) * 0.5], dtype=np.float64)
    target = np.array(target_uv, dtype=np.float64)
    center_distance = float(np.linalg.norm((center - target) / np.array([w, h], dtype=np.float64)))
    blue_overlap = float(np.logical_and(mask, blue_mask > 0).sum()) / max(area, 1.0)
    white_overlap = float(np.logical_and(mask, white_mask > 0).sum()) / max(area, 1.0)

    # Place-stage selection is strict preset-point selection. The candidate
    # closest to the configured image point wins; color/size are debug metadata
    # only and must not change the target shelf slot.
    score = -center_distance
    is_blue_box = blue_overlap >= 0.08
    is_white_box = white_overlap >= 0.75
    color_label = "blue" if is_blue_box else ("white" if is_white_box else "none")
    is_box_like = (
        (is_blue_box or is_white_box)
        and
        0.02 <= area_frac <= 0.42
        and 0.55 <= aspect <= 3.2
        and height_frac >= 0.18
        and width_frac <= 0.60
        and height_frac <= 0.78
    )
    return {
        "score": float(score),
        "center_distance": center_distance,
        "blue_overlap": float(blue_overlap),
        "white_overlap": float(white_overlap),
        "is_blue_box": bool(is_blue_box),
        "is_white_box": bool(is_white_box),
        "color_label": color_label,
        "area_frac": float(area_frac),
        "width_frac": float(width_frac),
        "height_frac": float(height_frac),
        "aspect": float(aspect),
        "is_box_like": bool(is_box_like),
    }


def _strict_place_blue_mask(rgb: np.ndarray) -> np.ndarray:
    """Strict blue mask for shelf-place box selection.

    Transport detection uses a wider cyan threshold to keep complete box rims.
    Shelf placement needs the opposite: reject gray/low-saturation shelf edges
    before selecting the preset box.
    """
    lab = cv2.cvtColor(rgb, cv2.COLOR_BGR2LAB)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    b_channel = lab[..., 2]
    h_channel = hsv[..., 0]
    s_channel = hsv[..., 1]
    v_channel = hsv[..., 2]
    mask = (
        (h_channel >= 82)
        & (h_channel <= 110)
        & (s_channel >= 85)
        & (v_channel >= 70)
        & (b_channel <= 112)
    )
    return mask.astype(np.uint8) * 255


def _strict_place_white_mask(rgb: np.ndarray) -> np.ndarray:
    """Strict white mask for shelf-place box selection."""
    lab = cv2.cvtColor(rgb, cv2.COLOR_BGR2LAB)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    l_channel = lab[..., 0]
    a_channel = lab[..., 1]
    b_channel = lab[..., 2]
    s_channel = hsv[..., 1]
    v_channel = hsv[..., 2]
    mask = (
        (l_channel >= 185)
        & (s_channel <= 55)
        & (v_channel >= 170)
        & (a_channel >= 116)
        & (a_channel <= 140)
        & (b_channel >= 116)
        & (b_channel <= 144)
    )
    return mask.astype(np.uint8) * 255


def _fastsam_candidates(result, rgb: np.ndarray, target_uv: tuple[float, float]) -> list[dict[str, Any]]:
    if result.masks is None:
        return []
    blue_mask = _strict_place_blue_mask(rgb)
    white_mask = _strict_place_white_mask(rgb)

    masks = result.masks.data.cpu().numpy()
    conf = result.boxes.conf.cpu().numpy()
    boxes = result.boxes.xyxy.cpu().numpy()
    candidates: list[dict[str, Any]] = []
    for idx, raw_mask in enumerate(masks):
        mask_bool = cv2.resize(raw_mask, (rgb.shape[1], rgb.shape[0])) > 0.5
        ys, xs = np.nonzero(mask_bool)
        if len(xs) == 0:
            continue
        x1, y1, x2, y2 = [int(v) for v in boxes[idx]]
        metrics = _mask_candidate_metrics(mask_bool, blue_mask, white_mask, rgb.shape[:2], target_uv)
        center = [float((xs.min() + xs.max()) * 0.5), float((ys.min() + ys.max()) * 0.5)]
        bbox_distance = float(np.linalg.norm((np.asarray(center, dtype=np.float64) - np.asarray(target_uv, dtype=np.float64)) / np.array([rgb.shape[1], rgb.shape[0]], dtype=np.float64)))
        candidates.append(
            {
                "idx": int(idx),
                "score": metrics["score"],
                "center_distance": metrics["center_distance"],
                "bbox_center_distance": bbox_distance,
                "blue_overlap": metrics["blue_overlap"],
                "white_overlap": metrics["white_overlap"],
                "is_blue_box": metrics["is_blue_box"],
                "is_white_box": metrics["is_white_box"],
                "color_label": metrics["color_label"],
                "area_frac": metrics["area_frac"],
                "width_frac": metrics["width_frac"],
                "height_frac": metrics["height_frac"],
                "aspect": metrics["aspect"],
                "is_box_like": metrics["is_box_like"],
                "conf": float(conf[idx]),
                "bbox": [x1, y1, x2, y2],
                "center": center,
                "area": int(mask_bool.sum()),
                "mask": mask_bool.astype(np.uint8) * 255,
            }
        )
    candidates.sort(key=lambda item: (not item["is_box_like"], item["bbox_center_distance"]))
    return candidates


def _draw_candidates(rgb: np.ndarray, candidates: list[dict[str, Any]], target_uv: tuple[float, float]) -> np.ndarray:
    vis = rgb.copy()
    cv2.drawMarker(
        vis,
        (int(round(target_uv[0])), int(round(target_uv[1]))),
        (255, 255, 255),
        markerType=cv2.MARKER_CROSS,
        markerSize=28,
        thickness=2,
    )
    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255)]
    for rank, item in enumerate(candidates[:8]):
        color = colors[rank % len(colors)]
        x1, y1, x2, y2 = item["bbox"]
        cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
        label = (
            f"rank={rank} idx={item['idx']} box_d={item['bbox_center_distance']:.2f} "
            f"color={item['color_label']} box={int(item['is_box_like'])}"
        )
        cv2.putText(vis, label, (x1, max(20, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    return vis


def _front_edge_midpoint(corners: dict[str, Any]) -> tuple[int, int] | None:
    front_left = corners.get("front_left")
    front_right = corners.get("front_right")
    if front_left is None or front_right is None:
        return None
    return (
        int(round((float(front_left[0]) + float(front_right[0])) * 0.5)),
        int(round((float(front_left[1]) + float(front_right[1])) * 0.5)),
    )


def _target_object(point_uv: tuple[int, int], depth: np.ndarray, camera) -> dict[str, Any]:
    u, v = point_uv
    depth_m = robust_depth_at(depth, u, v, patch=9)
    valid = depth_m is not None
    p3d = None if depth_m is None else unproject(u, v, depth_m, camera)
    if valid:
        x_mm, y_mm, z_mm = [round(float(value) * 1000.0, 1) for value in p3d]
        point_3d_m = [round(float(value), 4) for value in p3d]
        depth_mm = round(float(depth_m) * 1000.0, 1)
    else:
        x_mm = y_mm = z_mm = depth_mm = ""
        point_3d_m = None
    return {
        "label": "shelf_box_front_edge_midpoint",
        "center": [int(u), int(v)],
        "valid": bool(valid),
        "status": "valid" if valid else "no depth",
        "x_mm": x_mm,
        "y_mm": y_mm,
        "z_mm": z_mm,
        "depth_mm": depth_mm,
        "point_3d_m": point_3d_m,
    }


def _save_debug(debug_dir: Path, rgb: np.ndarray, depth: np.ndarray, result: dict[str, Any], annotated: np.ndarray, candidates_vis: np.ndarray) -> dict[str, str]:
    debug_dir.mkdir(parents=True, exist_ok=True)
    for old in debug_dir.glob("*.png"):
        old.unlink()
    paths = {
        "rgb": debug_dir / "rgb.png",
        "depth_vis": debug_dir / "depth_vis.png",
        "annotated": debug_dir / "annotated.png",
        "candidates": debug_dir / "candidates.png",
    }
    cv2.imwrite(str(paths["rgb"]), rgb)
    cv2.imwrite(str(paths["depth_vis"]), _depth_to_vis(depth))
    cv2.imwrite(str(paths["annotated"]), annotated)
    cv2.imwrite(str(paths["candidates"]), candidates_vis)
    mask = result.get("mask")
    if mask is not None:
        paths["selected_mask"] = debug_dir / "selected_mask.png"
        cv2.imwrite(str(paths["selected_mask"]), mask)
    return {key: str(path) for key, path in paths.items()}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument("--frame-timeout", type=float, default=8.0)
    parser.add_argument("--fastsam-model", default=str(DEFAULT_FASTSAM_MODEL))
    parser.add_argument("--fastsam-imgsz", type=int, default=1024)
    parser.add_argument("--fastsam-conf", type=float, default=0.25)
    parser.add_argument("--rim-fit-mode", choices=["free", "front-parallel", "side-mid"], default="side-mid")
    parser.add_argument("--center-x-frac", type=float, default=0.50)
    parser.add_argument("--center-y-frac", type=float, default=0.50)
    parser.add_argument("--candidate-count", type=int, default=30)
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--snapshot", default=str(DEFAULT_SNAPSHOT))
    parser.add_argument("--debug-dir", default=str(DEFAULT_DEBUG_DIR))
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

    print("=" * 70)
    print("  Shelf box place target detection (read-only)")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  Output: {args.output}")
    print(f"  Target image center: ({args.center_x_frac:.2f}, {args.center_y_frac:.2f})")

    control_client = None
    client = None
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
                verify=not args.no_neck_verify,
                tolerance=args.neck_verify_tolerance,
            )

        client = ROSClient(args.ws_url)
        if not client.connect():
            raise RuntimeError(f"failed to connect rosbridge: {args.ws_url}")
        rgb, depth, camera_info = _wait_for_frames(client, args.frame_timeout)
        if rgb.shape[:2] != depth.shape[:2]:
            raise RuntimeError(f"RGB/depth size mismatch: rgb={rgb.shape}, depth={depth.shape}")
        camera = camera_from_camera_info(camera_info)

        h, w = rgb.shape[:2]
        target_uv = (float(w * args.center_x_frac), float(h * args.center_y_frac))
        model = YOLO(args.fastsam_model)
        sam_result = model.predict(
            rgb,
            device="cpu",
            retina_masks=True,
            imgsz=args.fastsam_imgsz,
            conf=args.fastsam_conf,
            iou=0.7,
            verbose=False,
        )[0]
        candidates = _fastsam_candidates(sam_result, rgb, target_uv)
        candidates_vis = _draw_candidates(rgb, candidates, target_uv)
        if not candidates:
            raise RuntimeError("FastSAM produced no box candidates")

        selected = None
        selected_result = None
        selected_objects = None
        selected_target = None
        selected_distance = None
        evaluated_targets: list[dict[str, Any]] = []
        for item in candidates[: max(1, args.candidate_count)]:
            if not item.get("is_box_like"):
                evaluated_targets.append(
                    {
                        "idx": item["idx"],
                        "candidate_center": item["center"],
                        "box_center_distance_px": round(float(np.linalg.norm(np.asarray(item["center"], dtype=np.float64) - np.asarray(target_uv, dtype=np.float64))), 2),
                        "color": item.get("color_label", "none"),
                        "is_box_like": False,
                        "skipped": "not_color_shape_box",
                    }
                )
                continue
            result, objects, _extra = estimate_box_rim_grasp_from_mask(
                rgb,
                depth,
                camera,
                item["mask"],
                handle_y_frac=0.50,
                rim_fit_mode=args.rim_fit_mode,
            )
            if not result.get("ok"):
                continue
            midpoint = _front_edge_midpoint(result.get("rim_corners") or {})
            if midpoint is None:
                continue
            target_obj = _target_object(midpoint, depth, camera)
            box_distance_px = float(np.linalg.norm(np.asarray(item["center"], dtype=np.float64) - np.asarray(target_uv, dtype=np.float64)))
            target_distance_px = float(np.linalg.norm(np.asarray(midpoint, dtype=np.float64) - np.asarray(target_uv, dtype=np.float64)))
            evaluated_targets.append(
                {
                    "idx": item["idx"],
                    "candidate_center": item["center"],
                    "front_edge_midpoint": [int(midpoint[0]), int(midpoint[1])],
                    "box_center_distance_px": round(box_distance_px, 2),
                    "front_edge_midpoint_distance_px": round(target_distance_px, 2),
                    "target_valid": bool(target_obj["valid"]),
                }
            )
            if not target_obj["valid"]:
                continue
            if selected_distance is None or box_distance_px < selected_distance:
                selected = item
                selected_result = result
                selected_objects = objects
                selected_target = target_obj
                selected_distance = box_distance_px
        if selected is None or selected_result is None or selected_target is None:
            raise RuntimeError("no candidate produced a valid rim/front-edge midpoint")

        target = selected_target
        annotated = draw_box_rim_result(rgb, selected_result, selected_objects or [])
        color = (255, 255, 255) if target["valid"] else (0, 0, 255)
        cv2.circle(annotated, tuple(target["center"]), 10, color, -1)
        cv2.putText(
            annotated,
            "place target: front-edge midpoint",
            (max(10, target["center"][0] - 180), max(24, target["center"][1] - 16)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )

        debug_paths = _save_debug(Path(args.debug_dir), rgb, depth, selected_result, annotated, candidates_vis)
        snapshot = Path(args.snapshot)
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(snapshot), annotated)
        payload = {
            "type": "shelf_box_place_target",
            "frame": "camera",
            "ok": bool(target["valid"]),
            "note": "front_edge_midpoint_ok" if target["valid"] else "front_edge_midpoint_no_depth",
            "target": target,
            "selected_candidate": {key: value for key, value in selected.items() if key != "mask"},
            "selected_box_center_distance_px": round(float(selected_distance), 2),
            "evaluated_targets": evaluated_targets,
            "candidate_summary": [{key: value for key, value in item.items() if key != "mask"} for item in candidates[:10]],
            "rim": selected_result.get("rim"),
            "rim_corners": selected_result.get("rim_corners"),
            "rim_source": selected_result.get("rim_source"),
            "rim_meta": selected_result.get("rim_meta"),
            "debug_images": debug_paths,
        }
        _save_json(Path(args.output), payload)

        print(f"[✓] shelf box place target: {args.output}")
        print(
            "    target: "
            f"valid={target['valid']} camera=({target['x_mm']}, {target['y_mm']}, {target['z_mm']}) mm "
            f"pixel={target['center']} selected_box_center_distance_px={selected_distance:.1f}"
        )
        print(f"[✓] debug images: {args.debug_dir}")

        if args.show_window:
            cv2.imshow("shelf box place target", annotated)
            cv2.imshow("shelf box candidates", candidates_vis)
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
                    verify=not args.no_neck_verify,
                    tolerance=args.neck_verify_tolerance,
                )
            except Exception as exc:
                print(f"[!] neck home failed: {exc}")
        if client is not None:
            client.disconnect()
        _safe_terminate(control_client)


if __name__ == "__main__":
    main()
