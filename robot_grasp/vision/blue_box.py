"""Blue box grasp-point detection for the transport stage.

This module imports the useful parts from the blue-box draft:
- Lab b* segmentation for the blue plastic box.
- Opening rectangle estimation.
- Left/right short-edge midpoint handles.
- Depth median unprojection to camera-frame 3D points.

It intentionally does not include ROS or motion code.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int | None = None
    height: int | None = None

    @property
    def K(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )


def camera_from_camera_info(camera_info: dict[str, Any]) -> CameraModel:
    """Build a camera model from rosbridge sensor_msgs/CameraInfo."""
    k = camera_info["K"]
    return CameraModel(
        fx=float(k[0]),
        fy=float(k[4]),
        cx=float(k[2]),
        cy=float(k[5]),
        width=int(camera_info.get("width") or 0) or None,
        height=int(camera_info.get("height") or 0) or None,
    )


def segment_blue(
    image_bgr: np.ndarray,
    *,
    blue_b_thresh: int = 125,
    l_min: int = 15,
    l_max: int = 250,
    morph_open: int = 5,
    morph_close: int = 21,
) -> np.ndarray | None:
    """Segment the largest cyan-blue box region.

    Lab b* alone was too strict on the real turquoise box and only kept the
    strongest-blue interior. Combining a wider Lab b* threshold with HSV hue
    and saturation gives a more complete box contour while rejecting gray desks.
    """
    lab = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    l_channel, _, b_channel = lab[..., 0], lab[..., 1], lab[..., 2]
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV).astype(np.float32)
    h_channel, s_channel, v_channel = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    mask = (
        (b_channel < blue_b_thresh)
        & (l_channel > l_min)
        & (l_channel < l_max)
        & (h_channel > 70)
        & (h_channel < 105)
        & (s_channel > 45)
        & (v_channel > 80)
    )
    mask = mask.astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((morph_open, morph_open), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((morph_close, morph_close), np.uint8))

    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if count < 2:
        return None
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return (labels == largest).astype(np.uint8) * 255


def find_front_rim(
    mask: np.ndarray,
    image_bgr: np.ndarray,
    *,
    rim_top_frac: float = 0.05,
    rim_bot_frac: float = 0.85,
    min_width_frac: float = 0.30,
) -> dict[str, Any]:
    """Find the near rim row of the box opening using row brightness gradients."""
    ys, xs = np.nonzero(mask)
    if len(xs) < 100:
        return {"ok": False, "note": "mask too small"}

    top, bottom = int(ys.min()), int(ys.max())
    height = bottom - top + 1
    lab = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2LAB)
    l_channel = lab[..., 0].astype(np.float32)

    rows_l = np.full(height, np.nan)
    rows_w = np.zeros(height, dtype=int)
    for y in range(top, bottom + 1):
        cols = xs[ys == y]
        if len(cols) >= 5:
            rows_l[y - top] = float(l_channel[y, cols].mean())
            rows_w[y - top] = int(cols.max() - cols.min())

    max_w = int(rows_w.max())
    valid = (~np.isnan(rows_l)) & (rows_w >= min_width_frac * max_w)
    if int(valid.sum()) < 10:
        return {"ok": False, "note": "no valid rows"}

    valid_idx = np.where(valid)[0]
    profile = rows_l[valid_idx].astype(np.float32)
    smooth = cv2.GaussianBlur(profile.reshape(-1, 1), (5, 1), 0).ravel()
    gradient = np.abs(np.diff(smooth))

    k0 = int(np.searchsorted(valid_idx, height * rim_top_frac))
    k1 = int(np.searchsorted(valid_idx, height * rim_bot_frac))
    k1 = max(k0 + 1, min(k1, len(gradient)))
    segment = gradient[k0:k1]
    if len(segment) == 0 or not np.isfinite(segment).any():
        return {"ok": False, "note": "no gradient"}

    rim_y = top + int(valid_idx[k0 + int(np.nanargmax(segment))])
    cols = xs[ys == rim_y]
    if len(cols) < 10:
        return {"ok": False, "note": "too few mask pixels on rim row"}

    x_left, x_right = int(cols.min()), int(cols.max())
    return {
        "ok": True,
        "pa": (x_left, rim_y),
        "pb": (x_right, rim_y),
        "mid": ((x_left + x_right) // 2, rim_y),
        "rim_row": rim_y,
        "note": "ok",
    }


def _row_span_edges(mask: np.ndarray, *, min_width_frac: float = 0.55) -> dict[str, Any]:
    ys, xs = np.nonzero(mask)
    if len(xs) < 100:
        return {"ok": False, "note": "mask too small"}

    ys, xs = np.nonzero(mask)
    top, bottom = int(ys.min()), int(ys.max())
    max_w = int(xs.max() - xs.min())
    valid_rows: list[tuple[int, int, int]] = []
    for y in range(top, bottom + 1):
        cols = xs[ys == y]
        if len(cols) < 10:
            continue
        x_left, x_right = int(cols.min()), int(cols.max())
        if (x_right - x_left) >= min_width_frac * max_w:
            valid_rows.append((y, x_left, x_right))

    if len(valid_rows) < 5:
        return {"ok": False, "note": "no stable row span"}
    return {
        "ok": True,
        "rows": valid_rows,
        "top": valid_rows[0],
        "bottom": valid_rows[-1],
        "note": "ok",
    }


def _nearest_row_span(rows: list[tuple[int, int, int]], y_target: int) -> tuple[int, int, int]:
    return min(rows, key=lambda row: abs(row[0] - y_target))


def _find_opening_horizontal_edges(mask: np.ndarray, image_bgr: np.ndarray) -> dict[str, Any]:
    """Find top/back and front opening edges inside the blue-box candidate."""
    ys, xs = np.nonzero(mask)
    if len(xs) < 100:
        return {"ok": False, "note": "mask too small"}

    x0, x1 = int(xs.min()), int(xs.max())
    y0, y1 = int(ys.min()), int(ys.max())
    roi = image_bgr[y0 : y1 + 1, x0 : x1 + 1]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 50, 120)
    row_counts = edges.sum(axis=1).astype(np.float32) / 255.0

    height = max(1, y1 - y0 + 1)
    top_lo, top_hi = 0, max(1, int(0.30 * height))
    front_lo, front_hi = int(0.45 * height), max(int(0.45 * height) + 1, int(0.85 * height))
    front_hi = min(front_hi, height)

    if row_counts[top_lo:top_hi].size == 0 or row_counts[front_lo:front_hi].size == 0:
        return {"ok": False, "note": "empty edge search bands"}

    top_y = y0 + int(top_lo + np.argmax(row_counts[top_lo:top_hi]))
    front_y = y0 + int(front_lo + np.argmax(row_counts[front_lo:front_hi]))
    if front_y <= top_y + 20:
        return {"ok": False, "note": "opening edges too close"}

    return {
        "ok": True,
        "top_y": int(top_y),
        "front_y": int(front_y),
        "bbox": (x0, y0, x1, y1),
        "note": "ok",
    }


def _interp_point(p0: tuple[int, int], p1: tuple[int, int], frac: float) -> tuple[int, int]:
    return (
        int(round(p0[0] + (p1[0] - p0[0]) * frac)),
        int(round(p0[1] + (p1[1] - p0[1]) * frac)),
    )


def _order_quad_by_yx(points: np.ndarray) -> tuple[tuple[int, int], tuple[int, int], tuple[int, int], tuple[int, int]]:
    """Return quad as top-left, top-right, bottom-left, bottom-right."""
    pts = np.asarray(points, dtype=np.float32).reshape(4, 2)
    by_y = sorted(pts, key=lambda p: (float(p[1]), float(p[0])))
    top = sorted(by_y[:2], key=lambda p: float(p[0]))
    bottom = sorted(by_y[2:], key=lambda p: float(p[0]))
    return (
        (int(round(top[0][0])), int(round(top[0][1]))),
        (int(round(top[1][0])), int(round(top[1][1]))),
        (int(round(bottom[0][0])), int(round(bottom[0][1]))),
        (int(round(bottom[1][0])), int(round(bottom[1][1]))),
    )


def _line_fit_point_at_y(points: np.ndarray, y_target: float) -> tuple[int, int] | None:
    if len(points) < 3:
        return None
    pts = np.asarray(points, dtype=np.float32)
    ys = pts[:, 1]
    if float(ys.max() - ys.min()) < 5.0:
        return None
    # x = a*y + b is stable for near-vertical image boundaries.
    a, b = np.polyfit(ys.astype(np.float64), pts[:, 0].astype(np.float64), 1)
    return int(round(float(a * y_target + b))), int(round(y_target))


def _segment_angle_deg(seg: tuple[int, int, int, int]) -> float:
    x1, y1, x2, y2 = seg
    return float(np.degrees(np.arctan2(y2 - y1, x2 - x1)))


def _segment_length(seg: tuple[int, int, int, int]) -> float:
    x1, y1, x2, y2 = seg
    return float(np.hypot(x2 - x1, y2 - y1))


def _fit_x_at_y_from_segments(segments: list[tuple[int, int, int, int]], y_target: float) -> tuple[int, int] | None:
    points: list[tuple[int, int]] = []
    for x1, y1, x2, y2 in segments:
        points.append((x1, y1))
        points.append((x2, y2))
    if len(points) < 4:
        return None
    return _line_fit_point_at_y(np.asarray(points, dtype=np.float32), y_target)


def _median_y_from_segments(segments: list[tuple[int, int, int, int]]) -> int | None:
    if not segments:
        return None
    ys = [0.5 * (seg[1] + seg[3]) for seg in segments]
    return int(round(float(np.median(ys))))


def _find_inner_opening_by_hough(mask: np.ndarray, image_bgr: np.ndarray, *, handle_y_frac: float) -> dict[str, Any]:
    """Find the inner opening quadrilateral instead of the outer box contour.

    The blue color mask gives the box search ROI. Inside that ROI, Canny/Hough
    is used to find the visible top/back rim, front inner rim, and left/right
    side walls. This intentionally ignores the lower outer box boundary.
    """
    ys, xs = np.nonzero(mask)
    if len(xs) < 100:
        return {"ok": False, "note": "mask too small"}

    x0, y0, x1, y1 = int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())
    width = x1 - x0 + 1
    height = y1 - y0 + 1
    if width < 80 or height < 60:
        return {"ok": False, "note": "roi too small"}

    roi = image_bgr[y0 : y1 + 1, x0 : x1 + 1]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 35, 110)

    mask_roi = (mask[y0 : y1 + 1, x0 : x1 + 1] > 0).astype(np.uint8) * 255
    search = cv2.dilate(mask_roi, np.ones((11, 11), np.uint8))
    edges = cv2.bitwise_and(edges, search)

    min_len = max(45, int(round(0.16 * width)))
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=28, minLineLength=min_len, maxLineGap=28)
    if lines is None:
        return {"ok": False, "note": "no hough lines"}

    horizontal: list[tuple[int, int, int, int]] = []
    side_like: list[tuple[int, int, int, int]] = []
    for raw in lines[:, 0]:
        lx1, ly1, lx2, ly2 = [int(v) for v in raw]
        seg = (x0 + lx1, y0 + ly1, x0 + lx2, y0 + ly2)
        length = _segment_length(seg)
        if length < min_len:
            continue
        angle = _segment_angle_deg(seg)
        angle_abs = abs(angle)
        angle_from_horizontal = min(angle_abs, abs(180.0 - angle_abs))
        angle_from_vertical = abs(90.0 - angle_abs)
        mid_y = 0.5 * (seg[1] + seg[3])
        if angle_from_horizontal <= 25.0:
            horizontal.append(seg)
        elif angle_from_vertical <= 42.0 and (y0 + 0.05 * height) <= mid_y <= (y0 + 0.90 * height):
            side_like.append(seg)

    if len(horizontal) < 2:
        return {"ok": False, "note": f"not enough horizontal lines: {len(horizontal)}"}

    top_band = [seg for seg in horizontal if 0.5 * (seg[1] + seg[3]) <= y0 + 0.42 * height]
    if not top_band:
        return {"ok": False, "note": "no top rim line"}
    top_seed = max(top_band, key=_segment_length)
    top_seed_y = 0.5 * (top_seed[1] + top_seed[3])
    top_lines = [seg for seg in horizontal if abs(0.5 * (seg[1] + seg[3]) - top_seed_y) <= 18.0]
    top_y = _median_y_from_segments(top_lines)
    if top_y is None:
        return {"ok": False, "note": "top line fit failed"}

    front_candidates = [
        seg
        for seg in horizontal
        if (top_y + 35) <= 0.5 * (seg[1] + seg[3]) <= (y0 + 0.78 * height)
    ]
    if not front_candidates:
        return {"ok": False, "note": "no front inner rim line"}
    # Prefer the lower long inner rim, but do not let the lower outside bottom
    # edge win; restrict to upper/middle box height above.
    front_seed = max(front_candidates, key=lambda seg: _segment_length(seg) + 0.35 * (0.5 * (seg[1] + seg[3]) - top_y))
    front_seed_y = 0.5 * (front_seed[1] + front_seed[3])
    front_lines = [seg for seg in horizontal if abs(0.5 * (seg[1] + seg[3]) - front_seed_y) <= 20.0]
    front_y = _median_y_from_segments(front_lines)
    if front_y is None or front_y <= top_y + 25:
        return {"ok": False, "note": "front line fit failed"}

    roi_center_x = x0 + width * 0.5
    opening_mid_y = 0.5 * (top_y + front_y)
    valid_side_segments = [
        seg
        for seg in side_like
        if min(seg[1], seg[3]) <= front_y + 20
        and max(seg[1], seg[3]) >= top_y - 20
    ]
    left_segments = [
        seg
        for seg in valid_side_segments
        if 0.5 * (seg[0] + seg[2]) < roi_center_x
    ]
    right_segments = [
        seg
        for seg in valid_side_segments
        if 0.5 * (seg[0] + seg[2]) >= roi_center_x
    ]
    if len(left_segments) < 1 or len(right_segments) < 1:
        return {"ok": False, "note": f"missing side candidates left={len(left_segments)} right={len(right_segments)}"}

    # Keep side segments that are closest to the expected opening side walls,
    # not the far lower outside contour.
    left_segments = sorted(left_segments, key=lambda seg: (abs(0.5 * (seg[1] + seg[3]) - opening_mid_y), -_segment_length(seg)))[:4]
    right_segments = sorted(right_segments, key=lambda seg: (abs(0.5 * (seg[1] + seg[3]) - opening_mid_y), -_segment_length(seg)))[:4]

    handle_y_frac = float(np.clip(handle_y_frac, 0.0, 1.0))
    handle_y = int(round(top_y + (front_y - top_y) * handle_y_frac))
    left_mid = _fit_x_at_y_from_segments(left_segments, handle_y)
    right_mid = _fit_x_at_y_from_segments(right_segments, handle_y)
    top_left = _fit_x_at_y_from_segments(left_segments, top_y)
    top_right = _fit_x_at_y_from_segments(right_segments, top_y)
    front_left = _fit_x_at_y_from_segments(left_segments, front_y)
    front_right = _fit_x_at_y_from_segments(right_segments, front_y)
    if any(point is None for point in (left_mid, right_mid, top_left, top_right, front_left, front_right)):
        return {"ok": False, "note": "side line fitting failed"}

    if left_mid[0] >= right_mid[0] - 30:
        return {"ok": False, "note": "side lines crossed or too close"}

    return {
        "ok": True,
        "front_rim": (front_left, front_right, _interp_point(front_left, front_right, 0.5), front_y),
        "back_rim": (top_left, top_right),
        "outline": {
            "top": (top_left, top_right),
            "bottom": (front_left, front_right),
            "handle_y_frac": handle_y_frac,
            "handle_y": handle_y,
            "edge_source": "inner_hough",
            "edge_note": "opening inner quadrilateral from color ROI edges",
            "top_y": top_y,
            "front_y": front_y,
        },
        "left_mid": left_mid,
        "right_mid": right_mid,
        "note": "ok",
    }


def _find_opening_side_boundaries(mask: np.ndarray, *, handle_y_frac: float) -> dict[str, Any]:
    """Estimate side grasp points from visible left/right mask boundaries."""
    ys, xs = np.nonzero(mask)
    if len(xs) < 100:
        return {"ok": False, "note": "mask too small"}

    rows: list[tuple[int, int, int]] = []
    for y in range(int(ys.min()), int(ys.max()) + 1):
        cols = xs[ys == y]
        if len(cols) < 8:
            continue
        rows.append((y, int(cols.min()), int(cols.max())))
    if len(rows) < 10:
        return {"ok": False, "note": "not enough boundary rows"}

    widths = np.asarray([right - left for _, left, right in rows], dtype=np.float32)
    max_width = float(widths.max())
    if max_width < 40:
        return {"ok": False, "note": "boundary span too narrow"}

    stable_rows = [row for row, width in zip(rows, widths) if width >= max_width * 0.45]
    if len(stable_rows) < 10:
        return {"ok": False, "note": "not enough stable boundary rows"}

    top_index = int(round((len(stable_rows) - 1) * 0.08))
    bottom_index = int(round((len(stable_rows) - 1) * 0.92))
    top_y = stable_rows[top_index][0]
    bottom_y = stable_rows[bottom_index][0]
    if bottom_y <= top_y + 20:
        return {"ok": False, "note": "boundary height too small"}

    y0, y1 = int(ys.min()), int(ys.max())
    height = max(1, y1 - y0)
    side_rows = [row for row in stable_rows if (y0 + height * 0.45) <= row[0] <= (y0 + height * 0.85)]
    if len(side_rows) < 8:
        side_rows = stable_rows

    left_points = np.asarray([(left, y) for y, left, _ in side_rows], dtype=np.float32)
    right_points = np.asarray([(right, y) for y, _, right in side_rows], dtype=np.float32)
    handle_y_frac = float(np.clip(handle_y_frac, 0.0, 1.0))
    handle_y = int(round(top_y + (bottom_y - top_y) * handle_y_frac))
    left_mid = _line_fit_point_at_y(left_points, handle_y)
    right_mid = _line_fit_point_at_y(right_points, handle_y)
    if left_mid is None or right_mid is None:
        return {"ok": False, "note": "side line fitting failed"}

    _, top_left_x, top_right_x = _nearest_row_span(stable_rows, top_y)
    _, bottom_left_x, bottom_right_x = _nearest_row_span(stable_rows, bottom_y)
    top_left = (top_left_x, top_y)
    top_right = (top_right_x, top_y)
    bottom_left = (bottom_left_x, bottom_y)
    bottom_right = (bottom_right_x, bottom_y)

    return {
        "ok": True,
        "front_rim": (bottom_left, bottom_right, _interp_point(bottom_left, bottom_right, 0.5), int(round((bottom_left[1] + bottom_right[1]) / 2.0))),
        "back_rim": (top_left, top_right),
        "outline": {
            "top": (top_left, top_right),
            "bottom": (bottom_left, bottom_right),
            "handle_y_frac": handle_y_frac,
            "handle_y": handle_y,
            "edge_source": "side_boundary",
            "edge_note": "fallback from mask left/right boundaries",
        },
        "left_mid": left_mid,
        "right_mid": right_mid,
        "note": "ok",
    }


def find_opening_rect(
    mask: np.ndarray,
    image_bgr: np.ndarray,
    *,
    handle_y_frac: float = 0.50,
    geometry: str = "outer",
) -> dict[str, Any]:
    """Estimate box outline and return left/right upper-side handle points."""
    geometry = str(geometry or "outer").lower()
    inner_hough: dict[str, Any] | None = None
    if geometry in {"inner", "auto"}:
        inner_hough = _find_inner_opening_by_hough(mask, image_bgr, handle_y_frac=handle_y_frac)
        if inner_hough.get("ok"):
            return inner_hough
        if geometry == "inner":
            return {"ok": False, "note": f"inner_hough failed: {inner_hough.get('note')}"}

    side_boundary = _find_opening_side_boundaries(mask, handle_y_frac=handle_y_frac)
    if side_boundary.get("ok"):
        if inner_hough is not None:
            side_boundary["note"] = f"inner_hough failed: {inner_hough.get('note')}; {side_boundary.get('note')}"
        return side_boundary

    spans = _row_span_edges(mask)
    if not spans.get("ok"):
        return {"ok": False, "note": f"side boundary failed: {side_boundary.get('note')}; {spans.get('note', 'no row span')}"}

    edges = _find_opening_horizontal_edges(mask, image_bgr)
    if edges.get("ok"):
        top_y = int(edges["top_y"])
        bottom_y = int(edges["front_y"])
        _, top_left_x, top_right_x = _nearest_row_span(spans["rows"], top_y)
        _, bottom_left_x, bottom_right_x = _nearest_row_span(spans["rows"], bottom_y)
    else:
        top_y, top_left_x, top_right_x = spans["top"]
        bottom_y, bottom_left_x, bottom_right_x = spans["bottom"]

    handle_y_frac = float(np.clip(handle_y_frac, 0.0, 1.0))
    top_left = (top_left_x, top_y)
    top_right = (top_right_x, top_y)
    bottom_left = (bottom_left_x, bottom_y)
    bottom_right = (bottom_right_x, bottom_y)
    left_mid = _interp_point(top_left, bottom_left, handle_y_frac)
    right_mid = _interp_point(top_right, bottom_right, handle_y_frac)
    handle_y = int(round((left_mid[1] + right_mid[1]) / 2.0))

    return {
        "ok": True,
        "front_rim": (bottom_left, bottom_right, ((bottom_left_x + bottom_right_x) // 2, bottom_y), bottom_y),
        "back_rim": (top_left, top_right),
        "outline": {
            "top": (top_left, top_right),
            "bottom": (bottom_left, bottom_right),
            "handle_y_frac": handle_y_frac,
            "handle_y": handle_y,
            "edge_source": "canny" if edges.get("ok") else "mask_span",
            "edge_note": edges.get("note", ""),
        },
        "left_mid": left_mid,
        "right_mid": right_mid,
        "note": "ok",
    }


def robust_depth_at(
    depth: np.ndarray,
    px: int,
    py: int,
    *,
    patch: int = 7,
    depth_scale: float = 0.001,
    min_depth_m: float = 0.05,
) -> float | None:
    """Median depth around a pixel, returned in meters."""
    h, w = depth.shape[:2]
    px, py = int(round(px)), int(round(py))
    y0, y1 = max(0, py - patch // 2), min(h, py + patch // 2 + 1)
    x0, x1 = max(0, px - patch // 2), min(w, px + patch // 2 + 1)
    values = depth[y0:y1, x0:x1].astype(np.float64) * depth_scale
    valid = values[(values > min_depth_m) & np.isfinite(values)]
    if valid.size < 3:
        return None
    return float(np.median(valid))


def unproject(px: int, py: int, depth_m: float, camera: CameraModel) -> np.ndarray:
    z = float(depth_m)
    x = (float(px) - camera.cx) / camera.fx * z
    y = (float(py) - camera.cy) / camera.fy * z
    return np.array([x, y, z], dtype=np.float64)


def estimate_blue_box_grasp(
    image_bgr: np.ndarray,
    depth: np.ndarray,
    camera: CameraModel,
    *,
    blue_b_thresh: int = 125,
    depth_scale: float = 0.001,
    patch: int = 7,
    handle_y_frac: float = 0.50,
    geometry: str = "outer",
) -> dict[str, Any]:
    """Estimate left/right box handle grasp points in camera frame."""
    mask = segment_blue(image_bgr, blue_b_thresh=blue_b_thresh)
    if mask is None:
        return {"ok": False, "note": "no blue box detected"}

    return estimate_blue_box_grasp_from_mask(
        image_bgr,
        depth,
        camera,
        mask,
        depth_scale=depth_scale,
        patch=patch,
        handle_y_frac=handle_y_frac,
        geometry=geometry,
        source="color",
    )


def estimate_blue_box_grasp_from_mask(
    image_bgr: np.ndarray,
    depth: np.ndarray,
    camera: CameraModel,
    mask: np.ndarray,
    *,
    depth_scale: float = 0.001,
    patch: int = 7,
    handle_y_frac: float = 0.50,
    geometry: str = "outer",
    source: str = "mask",
) -> dict[str, Any]:
    """Estimate box handles from an externally supplied candidate mask."""
    mask = (mask > 0).astype(np.uint8) * 255
    opening = find_opening_rect(mask, image_bgr, handle_y_frac=handle_y_frac, geometry=geometry)
    if not opening.get("ok"):
        return {"ok": False, "note": f"opening rect failed: {opening.get('note')}", "mask": mask}

    result: dict[str, Any] = {
        "ok": True,
        "note": "ok",
        "mask": mask,
        "opening": opening,
        "source": source,
    }
    for side in ("left", "right"):
        px, py = opening[f"{side}_mid"]
        depth_m = robust_depth_at(depth, px, py, patch=patch, depth_scale=depth_scale)
        result[f"{side}_2d"] = (int(px), int(py))
        result[f"{side}_depth_m"] = depth_m
        result[f"{side}_3d"] = None if depth_m is None else unproject(px, py, depth_m, camera)

    result["ok"] = result["left_3d"] is not None or result["right_3d"] is not None
    if not result["ok"]:
        result["note"] = "no valid depth at handle midpoints"
    return result


def blue_box_to_object_results(result: dict[str, Any], *, label: str = "blue_box") -> list[dict[str, Any]]:
    """Convert box grasp result to object_results-like entries."""
    if not result.get("ok"):
        return []

    objects: list[dict[str, Any]] = []
    for idx, side in enumerate(("left", "right"), start=1):
        p2d = result.get(f"{side}_2d")
        p3d = result.get(f"{side}_3d")
        valid = p2d is not None and p3d is not None
        if p2d is None:
            center = (0, 0)
            bbox = (0, 0, 0, 0)
        else:
            u, v = int(p2d[0]), int(p2d[1])
            center = (u, v)
            bbox = (u - 15, v - 15, u + 15, v + 15)

        if valid:
            x_mm, y_mm, z_mm = [round(float(v) * 1000.0, 1) for v in p3d]
            depth_mm = round(float(p3d[2]) * 1000.0, 1)
            grasp_3d_m = [round(float(v), 4) for v in p3d]
        else:
            x_mm = y_mm = z_mm = depth_mm = ""
            grasp_3d_m = None

        objects.append(
            {
                "idx": idx,
                "label": label,
                "side": side,
                "confidence": 1.0,
                "bbox": bbox,
                "center": center,
                "qr_text": "",
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


def draw_blue_box_result(image_bgr: np.ndarray, result: dict[str, Any], objects: list[dict[str, Any]] | None = None) -> np.ndarray:
    vis = image_bgr.copy()
    mask = result.get("mask")
    if mask is not None:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(vis, contours, -1, (0, 165, 255), 2)

    opening = result.get("opening") or {}
    if opening.get("ok"):
        fl, fr, _, _ = opening["front_rim"]
        bl, br = opening["back_rim"]
        cv2.line(vis, fl, fr, (0, 0, 255), 3)
        cv2.line(vis, bl, br, (0, 255, 0), 3)
        cv2.line(vis, fl, bl, (255, 0, 0), 3)
        cv2.line(vis, fr, br, (255, 0, 0), 3)

    for obj in objects or []:
        u, v = obj["center"]
        color = (255, 0, 255) if obj.get("side") == "left" else (0, 255, 255)
        cv2.circle(vis, (int(u), int(v)), 8, color, -1)
        label = f"{obj.get('side')} {obj.get('status')}"
        if obj.get("valid"):
            label += f" ({obj['x_mm']},{obj['y_mm']},{obj['z_mm']})mm"
        cv2.putText(vis, label, (int(u) - 80, max(20, int(v) - 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    if not result.get("ok"):
        cv2.putText(vis, f"FAIL: {result.get('note', '')}", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    return vis
