#!/usr/bin/env python3
"""Offline FastSAM box-opening experiment.

Input: an RGB/depth snapshot from apps/transport/capture_depth_snapshot.py.
Output: FastSAM candidate mask + opening-rectangle grasp points.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot_grasp.vision.blue_box import (  # noqa: E402
    CameraModel,
    blue_box_to_object_results,
    draw_blue_box_result,
    estimate_blue_box_grasp_from_mask,
    robust_depth_at,
    unproject,
    segment_blue,
)


DEFAULT_INPUT_DIR = PROJECT_ROOT / "data" / "transport" / "depth_snapshot_latest"
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "data" / "transport" / "sam_opening_test"
DEFAULT_MODEL = PROJECT_ROOT / "models" / "yolo" / "FastSAM-x.pt"


def _load_camera(input_dir: Path, rgb: np.ndarray) -> CameraModel:
    info_path = input_dir / "camera_info.json"
    if info_path.exists():
        info = json.loads(info_path.read_text(encoding="utf-8"))
        k = info["K"]
        return CameraModel(
            fx=float(k[0]),
            fy=float(k[4]),
            cx=float(k[2]),
            cy=float(k[5]),
            width=int(info.get("width") or rgb.shape[1]),
            height=int(info.get("height") or rgb.shape[0]),
        )
    h, w = rgb.shape[:2]
    return CameraModel(fx=916.3634, fy=917.2302, cx=w / 2, cy=h / 2, width=w, height=h)


def _mask_color_score(mask: np.ndarray, blue_mask: np.ndarray, image_shape: tuple[int, int]) -> float:
    h, w = image_shape
    area = float(mask.sum())
    if area <= 0:
        return -1.0
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return -1.0
    x1, x2 = float(xs.min()), float(xs.max())
    y1, y2 = float(ys.min()), float(ys.max())
    width = x2 - x1 + 1.0
    height = y2 - y1 + 1.0
    center_y = (y1 + y2) / 2.0
    intersection = float(np.logical_and(mask, blue_mask > 0).sum())
    blue_area = float((blue_mask > 0).sum())
    overlap = intersection / area
    blue_recall = intersection / blue_area if blue_area > 0 else 0.0
    area_frac = area / float(h * w)
    aspect = width / max(1.0, height)
    height_frac = height / float(h)
    score = overlap * 2.0 + blue_recall * 4.0
    score += 0.8 if center_y > h * 0.45 else -1.0
    score += 0.5 if 0.03 <= area_frac <= 0.35 else -0.8
    score += 0.3 if 0.8 <= aspect <= 4.0 else -0.3
    score += 0.8 if height_frac >= 0.20 else -0.8
    score += min(0.7, height_frac * 2.0)
    return score


def _select_box_mask(result, rgb: np.ndarray) -> tuple[np.ndarray | None, list[dict]]:
    if result.masks is None:
        return None, []

    blue_mask = segment_blue(rgb, blue_b_thresh=125)
    if blue_mask is None:
        blue_mask = np.zeros(rgb.shape[:2], dtype=np.uint8)

    masks = result.masks.data.cpu().numpy()
    conf = result.boxes.conf.cpu().numpy()
    boxes = result.boxes.xyxy.cpu().numpy()
    candidates: list[dict] = []
    for idx, raw_mask in enumerate(masks):
        mask = cv2.resize(raw_mask, (rgb.shape[1], rgb.shape[0])) > 0.5
        score = _mask_color_score(mask, blue_mask, rgb.shape[:2])
        area = int(mask.sum())
        x1, y1, x2, y2 = boxes[idx].astype(int)
        candidates.append(
            {
                "idx": idx,
                "score": float(score),
                "conf": float(conf[idx]),
                "area": area,
                "bbox": [int(x1), int(y1), int(x2), int(y2)],
                "mask": mask,
            }
        )
    candidates.sort(key=lambda item: item["score"], reverse=True)
    return (candidates[0]["mask"].astype(np.uint8) * 255 if candidates else None), candidates


def _draw_candidates(rgb: np.ndarray, candidates: list[dict], max_draw: int = 8) -> np.ndarray:
    vis = rgb.copy()
    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255), (0, 255, 255)]
    for rank, item in enumerate(candidates[:max_draw]):
        color = np.array(colors[rank % len(colors)], dtype=np.uint8)
        mask = item["mask"].astype(bool)
        blended = cv2.addWeighted(vis, 0.65, np.full_like(vis, color), 0.35, 0)
        vis[mask] = blended[mask]
        x1, y1, x2, y2 = item["bbox"]
        label = f"{rank}:score={item['score']:.2f} area={item['area']}"
        cv2.rectangle(vis, (x1, y1), (x2, y2), tuple(int(v) for v in color), 2)
        cv2.putText(vis, label, (x1, max(20, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tuple(int(v) for v in color), 2)
    return vis


def _line_from_segment(p1: tuple[float, float], p2: tuple[float, float]) -> np.ndarray:
    x1, y1 = p1
    x2, y2 = p2
    line = np.cross(np.array([x1, y1, 1.0]), np.array([x2, y2, 1.0]))
    norm = np.linalg.norm(line[:2])
    return line / norm if norm > 1e-9 else line


def _intersect_lines(line_a: np.ndarray, line_b: np.ndarray) -> tuple[int, int] | None:
    p = np.cross(line_a, line_b)
    if abs(p[2]) < 1e-9:
        return None
    return int(round(p[0] / p[2])), int(round(p[1] / p[2]))


def _fit_line_from_segments(segments: list[tuple[int, int, int, int]]) -> np.ndarray | None:
    points = []
    for x1, y1, x2, y2 in segments:
        points.append([x1, y1])
        points.append([x2, y2])
    if len(points) < 2:
        return None
    pts = np.asarray(points, dtype=np.float32)
    vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).reshape(-1)
    return _line_from_segment((x0 - vx * 1000, y0 - vy * 1000), (x0 + vx * 1000, y0 + vy * 1000))


def _segment_mid_y(seg: tuple[int, int, int, int]) -> float:
    return (seg[1] + seg[3]) / 2.0


def _segment_mid_x(seg: tuple[int, int, int, int]) -> float:
    return (seg[0] + seg[2]) / 2.0


def _segment_angle(seg: tuple[int, int, int, int]) -> float:
    x1, y1, x2, y2 = seg
    return float(np.degrees(np.arctan2(y2 - y1, x2 - x1)))


def _detect_opening_by_hough(rgb: np.ndarray, mask: np.ndarray) -> dict:
    ys, xs = np.nonzero(mask)
    if len(xs) < 100:
        return {"ok": False, "note": "mask too small"}
    x0, y0, x1, y1 = int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())
    roi = rgb[y0 : y1 + 1, x0 : x1 + 1]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 40, 120)
    mask_roi = mask[y0 : y1 + 1, x0 : x1 + 1]
    mask_roi = (mask_roi > 0).astype(np.uint8) * 255
    # Only keep edges close to the candidate-box boundary. When the box carries
    # bags or other objects, interior edges are stronger than the plastic rim
    # and must not participate in Hough line grouping.
    boundary = cv2.morphologyEx(mask_roi, cv2.MORPH_GRADIENT, np.ones((17, 17), np.uint8))
    boundary = cv2.dilate(boundary, np.ones((9, 9), np.uint8))
    edges = cv2.bitwise_and(edges, boundary)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=35, minLineLength=80, maxLineGap=25)
    if lines is None:
        return {"ok": False, "note": "no hough lines", "edges": edges}

    horizontal: list[tuple[int, int, int, int]] = []
    vertical: list[tuple[int, int, int, int]] = []
    for raw in lines[:, 0]:
        xa, ya, xb, yb = [int(v) for v in raw]
        seg = (x0 + xa, y0 + ya, x0 + xb, y0 + yb)
        dx, dy = xb - xa, yb - ya
        length = float(np.hypot(dx, dy))
        if length < 70:
            continue
        angle = _segment_angle(seg)
        angle_abs = abs(angle)
        angle_from_horizontal = min(angle_abs, abs(180.0 - angle_abs))
        angle_from_vertical = abs(90.0 - angle_abs)
        if angle_from_horizontal < 35:
            horizontal.append(seg)
        elif angle_from_vertical < 45:
            vertical.append(seg)

    if len(horizontal) < 2 or len(vertical) < 2:
        return {
            "ok": False,
            "note": f"insufficient lines h={len(horizontal)} v={len(vertical)}",
            "edges": edges,
        }

    top_y_limit = y0 + (y1 - y0) * 0.45
    front_y_limit = y0 + (y1 - y0) * 0.45
    top_candidates = [seg for seg in horizontal if _segment_mid_y(seg) <= top_y_limit]
    front_candidates = [seg for seg in horizontal if _segment_mid_y(seg) >= front_y_limit]
    if not top_candidates or not front_candidates:
        return {"ok": False, "note": "missing top/front candidates", "edges": edges}

    top_seed_y = min(_segment_mid_y(seg) for seg in top_candidates)
    front_seed_y = min(_segment_mid_y(seg) for seg in front_candidates)
    top_group = [seg for seg in horizontal if abs(_segment_mid_y(seg) - top_seed_y) <= 12]
    front_group = [seg for seg in horizontal if abs(_segment_mid_y(seg) - front_seed_y) <= 16]

    box_center_x = (x0 + x1) / 2.0
    left_candidates = [seg for seg in vertical if _segment_mid_x(seg) < box_center_x]
    right_candidates = [seg for seg in vertical if _segment_mid_x(seg) >= box_center_x]
    if not left_candidates or not right_candidates:
        return {"ok": False, "note": "missing side candidates", "edges": edges}

    left_seed_x = max(_segment_mid_x(seg) for seg in left_candidates)
    right_seed_x = min(_segment_mid_x(seg) for seg in right_candidates)
    left_group = [seg for seg in vertical if abs(_segment_mid_x(seg) - left_seed_x) <= 45]
    right_group = [seg for seg in vertical if abs(_segment_mid_x(seg) - right_seed_x) <= 45]

    top_line = _fit_line_from_segments(top_group)
    front_line = _fit_line_from_segments(front_group)
    left_line = _fit_line_from_segments(left_group)
    right_line = _fit_line_from_segments(right_group)
    if any(line is None for line in (top_line, front_line, left_line, right_line)):
        return {"ok": False, "note": "line fitting failed", "edges": edges}

    top_left = _intersect_lines(top_line, left_line)
    top_right = _intersect_lines(top_line, right_line)
    front_left = _intersect_lines(front_line, left_line)
    front_right = _intersect_lines(front_line, right_line)
    if any(p is None for p in (top_left, top_right, front_left, front_right)):
        return {"ok": False, "note": "line intersections failed", "edges": edges}

    corners = [top_left, top_right, front_left, front_right]
    pad = 40
    for px, py in corners:
        if px < x0 - pad or px > x1 + pad or py < y0 - pad or py > y1 + pad:
            return {"ok": False, "note": "corner outside mask bbox", "edges": edges}
    top_width = float(np.hypot(top_right[0] - top_left[0], top_right[1] - top_left[1]))
    front_width = float(np.hypot(front_right[0] - front_left[0], front_right[1] - front_left[1]))
    left_height = float(np.hypot(front_left[0] - top_left[0], front_left[1] - top_left[1]))
    right_height = float(np.hypot(front_right[0] - top_right[0], front_right[1] - top_right[1]))
    if min(top_width, front_width, left_height, right_height) < 40:
        return {"ok": False, "note": "opening quadrilateral too small", "edges": edges}
    if max(top_width, front_width) / max(1.0, min(top_width, front_width)) > 2.2:
        return {"ok": False, "note": "top/front width mismatch", "edges": edges}
    if max(left_height, right_height) / max(1.0, min(left_height, right_height)) > 2.5:
        return {"ok": False, "note": "left/right side length mismatch", "edges": edges}

    return {
        "ok": True,
        "note": "ok",
        "corners": {
            "top_left": top_left,
            "top_right": top_right,
            "front_left": front_left,
            "front_right": front_right,
        },
        "groups": {
            "top": top_group,
            "front": front_group,
            "left": left_group,
            "right": right_group,
        },
        "edges": edges,
        "bbox": (x0, y0, x1, y1),
    }


def _draw_hough_opening(rgb: np.ndarray, opening: dict, objects: list[dict]) -> np.ndarray:
    vis = rgb.copy()
    colors = {"top": (0, 255, 0), "front": (0, 0, 255), "left": (255, 0, 0), "right": (255, 0, 0)}
    for name, segments in (opening.get("groups") or {}).items():
        for x1, y1, x2, y2 in segments:
            cv2.line(vis, (x1, y1), (x2, y2), colors.get(name, (255, 255, 255)), 2)
    corners = opening.get("corners") or {}
    for a, b, color in [
        ("top_left", "top_right", (0, 255, 0)),
        ("front_left", "front_right", (0, 0, 255)),
        ("top_left", "front_left", (255, 0, 0)),
        ("top_right", "front_right", (255, 0, 0)),
    ]:
        if corners.get(a) and corners.get(b):
            cv2.line(vis, corners[a], corners[b], color, 3)
    for obj in objects:
        u, v = obj["center"]
        color = (255, 0, 255) if obj.get("side") == "left" else (0, 255, 255)
        cv2.circle(vis, (int(u), int(v)), 8, color, -1)
        cv2.putText(vis, f"{obj['side']} {obj['status']}", (int(u) - 80, max(20, int(v) - 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    return vis


def _line_fit_point_at_y(points: np.ndarray, y_target: float) -> tuple[int, int] | None:
    if len(points) < 3:
        return None
    pts = np.asarray(points, dtype=np.float32)
    ys = pts[:, 1]
    if float(ys.max() - ys.min()) < 5.0:
        return None
    a, b = np.polyfit(ys.astype(np.float64), pts[:, 0].astype(np.float64), 1)
    return int(round(float(a * y_target + b))), int(round(y_target))


def _line_group_endpoints(segments: list[tuple[int, int, int, int]]) -> tuple[tuple[int, int], tuple[int, int]] | None:
    points = []
    for x1, y1, x2, y2 in segments:
        points.append([x1, y1])
        points.append([x2, y2])
    if len(points) < 2:
        return None
    pts = np.asarray(points, dtype=np.float32)
    vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).reshape(-1)
    direction = np.array([vx, vy], dtype=np.float32)
    origin = np.array([x0, y0], dtype=np.float32)
    rel = pts - origin
    t = rel @ direction
    p0 = origin + direction * float(t.min())
    p1 = origin + direction * float(t.max())
    endpoints = sorted(
        [
            (int(round(float(p0[0]))), int(round(float(p0[1])))),
            (int(round(float(p1[0]))), int(round(float(p1[1])))),
        ],
        key=lambda p: (p[0], p[1]),
    )
    return endpoints[0], endpoints[1]


def _red_green_side_midpoints(opening: dict, handle_y_frac: float) -> dict[str, tuple[int, int]]:
    groups = opening.get("groups") or {}
    top_endpoints = _line_group_endpoints(groups.get("top") or [])
    front_endpoints = _line_group_endpoints(groups.get("front") or [])
    if top_endpoints is not None and front_endpoints is not None:
        top_left, top_right = top_endpoints
        front_left, front_right = front_endpoints
    else:
        corners = opening["corners"]
        top_left = corners["top_left"]
        top_right = corners["top_right"]
        front_left = corners["front_left"]
        front_right = corners["front_right"]
    frac = float(np.clip(handle_y_frac, 0.0, 1.0))
    points = {
        "left": (
            int(round(top_left[0] + (front_left[0] - top_left[0]) * frac)),
            int(round(top_left[1] + (front_left[1] - top_left[1]) * frac)),
        ),
        "right": (
            int(round(top_right[0] + (front_right[0] - top_right[0]) * frac)),
            int(round(top_right[1] + (front_right[1] - top_right[1]) * frac)),
        ),
    }
    return points


def _objects_from_hough(
    opening: dict,
    depth: np.ndarray,
    camera: CameraModel,
    handle_y_frac: float,
    mask: np.ndarray | None = None,
) -> list[dict]:
    corners = opening["corners"]
    points = _red_green_side_midpoints(opening, handle_y_frac)
    objects = []
    for idx, side in enumerate(("left", "right"), start=1):
        u, v = points[side]
        depth_m = robust_depth_at(depth, u, v, patch=9)
        valid = depth_m is not None
        p3d = None if depth_m is None else unproject(u, v, depth_m, camera)
        if valid:
            x_mm, y_mm, z_mm = [round(float(value) * 1000.0, 1) for value in p3d]
            grasp_3d_m = [round(float(value), 4) for value in p3d]
            depth_mm = round(float(depth_m) * 1000.0, 1)
        else:
            x_mm = y_mm = z_mm = depth_mm = ""
            grasp_3d_m = None
        objects.append(
            {
                "idx": idx,
                "label": "blue_box",
                "side": side,
                "confidence": 1.0,
                "bbox": [u - 15, v - 15, u + 15, v + 15],
                "center": [u, v],
                "valid": bool(valid),
                "status": "valid" if valid else "no depth",
                "x_mm": x_mm,
                "y_mm": y_mm,
                "z_mm": z_mm,
                "depth_mm": depth_mm,
                "grasp_3d_m": grasp_3d_m,
            }
        )
    return objects


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", default=str(DEFAULT_INPUT_DIR))
    parser.add_argument("--image", default="")
    parser.add_argument("--model", default=str(DEFAULT_MODEL))
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--imgsz", type=int, default=1024)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--handle-y-frac", type=float, default=0.50)
    parser.add_argument("--show-window", action="store_true")
    args = parser.parse_args()

    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    image_path = Path(args.image) if args.image else input_dir / "rgb.png"
    rgb = cv2.imread(str(image_path))
    if rgb is None:
        raise FileNotFoundError(image_path)
    depth_path = input_dir / "depth_raw.npy"
    if depth_path.exists():
        depth = np.load(str(depth_path))
    else:
        depth = np.full(rgb.shape[:2], 1000, dtype=np.uint16)
    camera = _load_camera(input_dir, rgb)

    model = YOLO(args.model)
    sam_result = model.predict(
        str(image_path),
        device="cpu",
        retina_masks=True,
        imgsz=args.imgsz,
        conf=args.conf,
        iou=0.7,
        verbose=False,
    )[0]
    mask, candidates = _select_box_mask(sam_result, rgb)
    cv2.imwrite(str(output_dir / "sam_candidates.png"), _draw_candidates(rgb, candidates))

    if mask is None:
        raise RuntimeError("FastSAM produced no candidate mask")
    cv2.imwrite(str(output_dir / "selected_mask.png"), mask)

    result = estimate_blue_box_grasp_from_mask(
        rgb,
        depth,
        camera,
        mask,
        handle_y_frac=args.handle_y_frac,
        source="fastsam",
    )
    objects = blue_box_to_object_results(result)
    annotated = draw_blue_box_result(rgb, result, objects)
    cv2.imwrite(str(output_dir / "opening_result_mask_span.png"), annotated)

    hough = _detect_opening_by_hough(rgb, mask)
    if hough.get("ok"):
        hough_objects = _objects_from_hough(hough, depth, camera, args.handle_y_frac, mask)
        hough_annotated = _draw_hough_opening(rgb, hough, hough_objects)
        cv2.imwrite(str(output_dir / "opening_result_hough.png"), hough_annotated)
        final_objects = hough_objects
        final_ok = any(obj.get("valid") for obj in hough_objects)
        final_note = "hough_ok" if final_ok else "hough_no_depth"
        final_source = "hough"
    else:
        cv2.putText(
            annotated,
            f"hough_failed:{hough.get('note')}; fallback:{result.get('note')}",
            (20, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )
        cv2.imwrite(str(output_dir / "opening_result_fallback.png"), annotated)
        final_objects = objects
        final_ok = bool(result.get("ok"))
        final_note = f"hough_failed:{hough.get('note')}; fallback:{result.get('note')}"
        final_source = "fallback"

    report = {
        "ok": bool(final_ok),
        "note": final_note,
        "source": final_source,
        "model": args.model,
        "objects": final_objects,
        "mask_span_opening": result.get("opening"),
        "hough_opening": {
            key: value for key, value in hough.items() if key not in ("edges",)
        },
        "candidate_summary": [
            {k: v for k, v in item.items() if k != "mask"} for item in candidates[:10]
        ],
        "outputs": {
            "sam_candidates": str(output_dir / "sam_candidates.png"),
            "selected_mask": str(output_dir / "selected_mask.png"),
            "opening_result_mask_span": str(output_dir / "opening_result_mask_span.png"),
            "opening_result_hough": str(output_dir / "opening_result_hough.png"),
        },
    }
    (output_dir / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    print(f"ok={report['ok']} note={report['note']}")
    print(f"saved: {output_dir}")
    for obj in final_objects:
        print(obj["side"], obj["status"], obj["center"], obj["x_mm"], obj["y_mm"], obj["z_mm"])

    if args.show_window:
        display = cv2.imread(str(output_dir / "opening_result_hough.png"))
        cv2.imshow("FastSAM Hough opening result | q=exit", display)
        while True:
            if cv2.waitKey(30) & 0xFF == ord("q"):
                break
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
