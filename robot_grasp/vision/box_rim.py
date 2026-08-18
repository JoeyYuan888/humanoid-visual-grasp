"""Box rim geometry helpers for transport grasp-point detection.

The functions here use candidate masks only as a search region. They do not
assume the box is empty or sitting on a table. The primary objective is to
estimate the visible upper rim quadrilateral and derive left/right grasp points.
"""

from __future__ import annotations

from itertools import combinations
from typing import Any

import cv2
import numpy as np

from robot_grasp.vision.blue_box import CameraModel, robust_depth_at, unproject


def metric_depth_edges(
    depth_mm: np.ndarray,
    *,
    min_depth_mm: float = 150.0,
    max_depth_mm: float = 3000.0,
    min_depth_jump_mm: float = 32.0,
    use_invalid_depth_edges: bool = True,
) -> tuple[np.ndarray, np.ndarray]:
    """Extract metric depth discontinuities and valid/invalid boundaries."""
    valid = (
        np.isfinite(depth_mm)
        & (depth_mm >= min_depth_mm)
        & (depth_mm <= max_depth_mm)
    )
    jump = np.zeros(depth_mm.shape, dtype=np.float32)

    horizontal_valid = valid[:, 1:] & valid[:, :-1]
    horizontal_jump = np.abs(depth_mm[:, 1:] - depth_mm[:, :-1])
    horizontal_jump[~horizontal_valid] = 0.0
    jump[:, 1:] = np.maximum(jump[:, 1:], horizontal_jump)
    jump[:, :-1] = np.maximum(jump[:, :-1], horizontal_jump)

    vertical_valid = valid[1:, :] & valid[:-1, :]
    vertical_jump = np.abs(depth_mm[1:, :] - depth_mm[:-1, :])
    vertical_jump[~vertical_valid] = 0.0
    jump[1:, :] = np.maximum(jump[1:, :], vertical_jump)
    jump[:-1, :] = np.maximum(jump[:-1, :], vertical_jump)

    edges = np.zeros(depth_mm.shape, dtype=np.uint8)
    edges[jump >= min_depth_jump_mm] = 255
    if use_invalid_depth_edges:
        valid_u8 = valid.astype(np.uint8) * 255
        invalid_boundary = cv2.morphologyEx(valid_u8, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8))
        edges = cv2.bitwise_or(edges, invalid_boundary)

    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    return edges, valid


def edge_support(polygon: np.ndarray, edges: np.ndarray) -> float:
    perimeter = np.zeros(edges.shape, dtype=np.uint8)
    cv2.polylines(perimeter, [polygon.astype(np.int32)], True, 255, 5, cv2.LINE_8)
    nearby_edges = cv2.dilate(edges, np.ones((7, 7), np.uint8))
    supported = cv2.countNonZero(cv2.bitwise_and(perimeter, nearby_edges))
    return supported / max(cv2.countNonZero(perimeter), 1)


def _line_intersection(first: np.ndarray, second: np.ndarray) -> np.ndarray | None:
    first_line = np.cross(
        np.array([first[0], first[1], 1.0]),
        np.array([first[2], first[3], 1.0]),
    )
    second_line = np.cross(
        np.array([second[0], second[1], 1.0]),
        np.array([second[2], second[3], 1.0]),
    )
    point = np.cross(first_line, second_line)
    if abs(point[2]) < 1e-6:
        return None
    return point[:2] / point[2]


def _line_from_points(start: np.ndarray, end: np.ndarray) -> np.ndarray:
    return np.array([float(start[0]), float(start[1]), float(end[0]), float(end[1])], dtype=np.float64)


def _fit_line_from_segments(segments: list[np.ndarray]) -> np.ndarray | None:
    points: list[np.ndarray] = []
    for line in segments:
        points.append(np.asarray(line[:2], dtype=np.float32))
        points.append(np.asarray(line[2:], dtype=np.float32))
    if len(points) < 4:
        return None
    pts = np.asarray(points, dtype=np.float32)
    vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
    extent = 2000.0
    return np.array([x0 - extent * vx, y0 - extent * vy, x0 + extent * vx, y0 + extent * vy], dtype=np.float64)


def _fit_line_from_points(points: np.ndarray) -> np.ndarray | None:
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    if len(pts) < 6:
        return None
    vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
    extent = 2000.0
    return np.array([x0 - extent * vx, y0 - extent * vy, x0 + extent * vx, y0 + extent * vy], dtype=np.float64)


def _line_angle(line: np.ndarray) -> float:
    return float(np.arctan2(float(line[3] - line[1]), float(line[2] - line[0])))


def _angle_delta(first: float, second: float) -> float:
    # Undirected-line angle difference in radians.
    diff = abs(np.arctan2(np.sin(first - second), np.cos(first - second)))
    return min(diff, abs(np.pi - diff))


def _point_line_signed_distance(point: np.ndarray, line: np.ndarray) -> float:
    start = line[:2].astype(np.float64)
    end = line[2:].astype(np.float64)
    direction = end - start
    length = float(np.linalg.norm(direction))
    if length < 1e-6:
        return 0.0
    return float(np.cross(direction, point.astype(np.float64) - start) / length)


def _shift_line(line: np.ndarray, signed_distance: float) -> np.ndarray:
    start = line[:2].astype(np.float64)
    end = line[2:].astype(np.float64)
    direction = end - start
    length = float(np.linalg.norm(direction))
    if length < 1e-6:
        return line.copy()
    normal = np.array([-direction[1], direction[0]], dtype=np.float64) / length
    offset = normal * float(signed_distance)
    return np.array([*(start + offset), *(end + offset)], dtype=np.float64)


def _line_point_at_x(line: np.ndarray, x: float) -> np.ndarray | None:
    x1, y1, x2, y2 = [float(v) for v in line]
    if abs(x2 - x1) < 1e-6:
        return None
    t = (float(x) - x1) / (x2 - x1)
    return np.array([float(x), y1 + t * (y2 - y1)], dtype=np.float64)


def _line_point_at_y(line: np.ndarray, y: float) -> np.ndarray | None:
    x1, y1, x2, y2 = [float(v) for v in line]
    if abs(y2 - y1) < 1e-6:
        return None
    t = (float(y) - y1) / (y2 - y1)
    return np.array([x1 + t * (x2 - x1), float(y)], dtype=np.float64)


def _robust_top_boundary_line(
    mask: np.ndarray,
    *,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    reference_angle: float,
    bin_width: int = 10,
) -> tuple[np.ndarray | None, dict[str, Any]]:
    """Fit the dominant upper mask boundary while rejecting local protrusions."""
    h, w = mask.shape[:2]
    x0 = max(0, int(np.floor(min(x_min, x_max))))
    x1 = min(w - 1, int(np.ceil(max(x_min, x_max))))
    y0 = max(0, int(np.floor(y_min)))
    y1 = min(h - 1, int(np.ceil(y_max)))
    if x1 <= x0 + 30 or y1 <= y0 + 10:
        return None, {"top_fit": "invalid search window"}

    raw_points: list[tuple[float, float]] = []
    for xx in range(x0, x1 + 1):
        ys = np.where(mask[y0 : y1 + 1, xx] > 0)[0]
        if ys.size == 0:
            continue
        raw_points.append((float(xx), float(y0 + int(ys.min()))))
    if len(raw_points) < 20:
        return None, {"top_fit": f"too few top boundary columns: {len(raw_points)}"}

    points: list[tuple[float, float]] = []
    for start in range(x0, x1 + 1, bin_width):
        end = min(x1, start + bin_width - 1)
        bucket = [(xx, yy) for xx, yy in raw_points if start <= xx <= end]
        if len(bucket) < max(3, bin_width // 3):
            continue
        xs_bucket = np.asarray([item[0] for item in bucket], dtype=np.float64)
        ys_bucket = np.asarray([item[1] for item in bucket], dtype=np.float64)
        points.append((float(np.median(xs_bucket)), float(np.median(ys_bucket))))
    if len(points) < 8:
        return None, {"top_fit": f"too few binned top points: {len(points)}"}

    pts = np.asarray(points, dtype=np.float64)
    best: tuple[int, float, float, np.ndarray] | None = None
    n = len(pts)
    for i in range(n):
        for j in range(i + 1, n):
            dx = pts[j, 0] - pts[i, 0]
            if abs(dx) < 0.25 * (x1 - x0):
                continue
            slope = (pts[j, 1] - pts[i, 1]) / dx
            angle = float(np.arctan(slope))
            if _angle_delta(angle, reference_angle) > np.deg2rad(18.0):
                continue
            intercept = pts[i, 1] - slope * pts[i, 0]
            residuals = np.abs(pts[:, 1] - (slope * pts[:, 0] + intercept))
            inliers = residuals <= 8.0
            inlier_count = int(inliers.sum())
            if inlier_count < max(6, int(0.42 * n)):
                continue
            mean_residual = float(residuals[inliers].mean()) if inlier_count else 999.0
            angle_penalty = float(np.rad2deg(_angle_delta(angle, reference_angle)))
            score = inlier_count * 100.0 - mean_residual * 6.0 - angle_penalty * 2.0
            if best is None or score > best[1]:
                best = (inlier_count, score, mean_residual, inliers)
    if best is None:
        return None, {"top_fit": f"ransac failed points={n}"}

    inliers = best[3]
    fit_pts = pts[inliers]
    slope, intercept = np.polyfit(fit_pts[:, 0], fit_pts[:, 1], 1)
    line = np.array([x0, slope * x0 + intercept, x1, slope * x1 + intercept], dtype=np.float64)
    return line, {
        "top_fit": "robust_top_boundary",
        "top_points": int(n),
        "top_inliers": int(best[0]),
        "top_mean_residual": round(float(best[2]), 2),
        "top_angle_deg": round(float(np.degrees(np.arctan(slope))), 2),
    }


def refine_rim_from_edge_pixels(rim: np.ndarray, edges: np.ndarray) -> np.ndarray:
    """Robustly refit each rim side to nearby edge pixels."""
    ys, xs = np.where(edges > 0)
    if len(xs) < 40:
        return rim
    points = np.column_stack((xs, ys)).astype(np.float32)
    fitted_lines: list[np.ndarray] = []

    for index in range(4):
        start = rim[index].astype(np.float32)
        end = rim[(index + 1) % 4].astype(np.float32)
        vector = end - start
        length = float(np.linalg.norm(vector))
        if length < 20.0:
            return rim
        direction = vector / length
        relative = points - start
        along = relative @ direction
        normal_distance = np.abs(relative[:, 0] * direction[1] - relative[:, 1] * direction[0])
        selected = points[
            (along >= -0.05 * length)
            & (along <= 1.05 * length)
            & (normal_distance <= 3.5)
        ]
        if len(selected) < 18:
            return rim
        vx, vy, x0, y0 = cv2.fitLine(selected, cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
        extent = 2000.0
        fitted_lines.append(np.array([x0 - extent * vx, y0 - extent * vy, x0 + extent * vx, y0 + extent * vy]))

    refined: list[np.ndarray] = []
    for index in range(4):
        point = _line_intersection(fitted_lines[(index - 1) % 4], fitted_lines[index])
        if point is None or not np.all(np.isfinite(point)):
            return rim
        refined.append(point)
    refined_array = np.array(refined, dtype=np.float32)
    if np.max(np.linalg.norm(refined_array - rim, axis=1)) > 18.0:
        return rim
    return np.rint(refined_array).astype(np.int32)


def extract_front_parallel_rim(edges: np.ndarray, contour: np.ndarray) -> tuple[np.ndarray | None, dict[str, Any]]:
    """Fit rim with the back edge constrained to be parallel to the front edge.

    For loaded boxes, inner objects create strong edges that can hijack the
    back/top rim in an unconstrained four-line fit. The front rim is more
    reliable because it is the nearest long lower edge. This routine fits the
    front line first, then derives a parallel back line from upper mask geometry.
    """
    hull = cv2.convexHull(contour)
    x, y, width, height = cv2.boundingRect(hull)
    if width < 80 or height < 60:
        return None, {"note": "contour too small"}

    roi = np.zeros(edges.shape, dtype=np.uint8)
    cv2.drawContours(roi, [hull], -1, 255, -1)
    roi = cv2.dilate(roi, np.ones((13, 13), np.uint8))
    candidate_edges = cv2.bitwise_and(edges, roi)

    min_line_length = max(45, int(round(0.18 * width)))
    lines = cv2.HoughLinesP(
        candidate_edges,
        1,
        np.pi / 360.0,
        threshold=20,
        minLineLength=min_line_length,
        maxLineGap=35,
    )
    if lines is None:
        return None, {"note": "no hough lines"}

    raw_lines = [line.astype(np.float64) for line in lines[:, 0]]
    mostly_horizontal: list[np.ndarray] = []
    for line in raw_lines:
        angle = _line_angle(line)
        if _angle_delta(angle, 0.0) <= np.deg2rad(35.0):
            mostly_horizontal.append(line)
    if len(mostly_horizontal) < 2:
        return None, {"note": f"not enough mostly-horizontal lines: {len(mostly_horizontal)}"}

    center = np.array([x + width * 0.5, y + height * 0.5], dtype=np.float64)
    front_band_lo = y + height * 0.38
    front_band_hi = y + height * 0.66
    front_target_y = y + height * 0.54
    front_candidates = [
        line
        for line in mostly_horizontal
        if front_band_lo <= 0.5 * (line[1] + line[3]) <= front_band_hi
    ]
    if not front_candidates:
        front_candidates = [
            line
            for line in mostly_horizontal
            if (y + height * 0.36) <= 0.5 * (line[1] + line[3]) <= (y + height * 0.72)
        ]
    if not front_candidates:
        return None, {"note": "no front line candidates"}

    front_seed = max(
        front_candidates,
        key=lambda line: np.linalg.norm(line[2:] - line[:2]) - 0.75 * abs(0.5 * (line[1] + line[3]) - front_target_y),
    )
    front_angle = _line_angle(front_seed)
    front_group = [
        line
        for line in mostly_horizontal
        if _angle_delta(_line_angle(line), front_angle) <= np.deg2rad(8.0)
        and abs(0.5 * (line[1] + line[3]) - 0.5 * (front_seed[1] + front_seed[3])) <= 24.0
    ]
    front_line = _fit_line_from_segments(front_group)
    if front_line is None:
        return None, {"note": "front line fit failed"}

    front_mid = 0.5 * (front_line[:2] + front_line[2:])
    if front_mid[1] < y + 0.38 * height:
        return None, {"note": "front line too high"}

    parallel_above: list[tuple[np.ndarray, float, float, float]] = []
    for line in mostly_horizontal:
        if _angle_delta(_line_angle(line), front_angle) > np.deg2rad(10.0):
            continue
        midpoint = 0.5 * (line[:2] + line[2:])
        front_point = _line_point_at_x(front_line, float(midpoint[0]))
        if front_point is None:
            continue
        if midpoint[1] >= front_point[1] - 25.0:
            continue
        distance = _point_line_signed_distance(midpoint, front_line)
        if abs(distance) < max(35.0, 0.12 * height) or abs(distance) > 0.62 * height:
            continue
        length = float(np.linalg.norm(line[2:] - line[:2]))
        target_distance = 0.42 * height
        score = length - 0.45 * abs(abs(distance) - target_distance)
        parallel_above.append((line, distance, length, score))
    if parallel_above:
        back_seed, back_distance_seed, _, _ = max(parallel_above, key=lambda item: item[3])
        back_group = [
            line
            for line, distance, _, _ in parallel_above
            if abs(distance - back_distance_seed) <= 22.0
        ]
        back_line = _fit_line_from_segments(back_group)
        if back_line is None:
            back_line = _shift_line(front_line, back_distance_seed)
        back_distance = _point_line_signed_distance(0.5 * (back_line[:2] + back_line[2:]), front_line)
    else:
        contour_points = hull.reshape(-1, 2).astype(np.float64)
        candidate_distances: list[float] = []
        for point in contour_points:
            line_point = _line_point_at_x(front_line, float(point[0]))
            if line_point is None:
                continue
            if point[1] < line_point[1] - 12.0:
                candidate_distances.append(_point_line_signed_distance(point, front_line))
        if len(candidate_distances) < 2:
            return None, {"note": "no parallel back candidates"}
        distances = np.asarray(candidate_distances, dtype=np.float64)
        positive = distances[distances > 0]
        negative = distances[distances < 0]
        if positive.size >= negative.size and positive.size >= 2:
            back_distance = float(np.percentile(positive, 70))
        elif negative.size >= 2:
            back_distance = float(np.percentile(negative, 30))
        else:
            return None, {"note": "back distance side ambiguous"}
        min_back_distance = max(35.0, 0.18 * height)
        max_back_distance = max(min_back_distance + 1.0, 0.58 * height)
        if abs(back_distance) < min_back_distance:
            return None, {"note": "parallel back line distance too small"}
        if abs(back_distance) > max_back_distance:
            back_distance = float(np.sign(back_distance) * max_back_distance)
        back_line = _shift_line(front_line, back_distance)

    # Left/right side locations come from the mask hull, then intersections with
    # front/back parallel lines produce a slanted quadrilateral.
    contour_points = hull.reshape(-1, 2).astype(np.float64)
    left_x = float(np.percentile(contour_points[:, 0], 5))
    right_x = float(np.percentile(contour_points[:, 0], 95))
    front_left = _line_point_at_x(front_line, left_x)
    front_right = _line_point_at_x(front_line, right_x)
    back_left = _line_point_at_x(back_line, left_x)
    back_right = _line_point_at_x(back_line, right_x)
    if any(point is None for point in (front_left, front_right, back_left, back_right)):
        return None, {"note": "parallel line endpoint failed"}

    rim = np.rint(np.array([back_left, back_right, front_right, front_left], dtype=np.float64)).astype(np.int32)
    rim[:, 0] = np.clip(rim[:, 0], 0, edges.shape[1] - 1)
    rim[:, 1] = np.clip(rim[:, 1], 0, edges.shape[0] - 1)
    if abs(float(cv2.contourArea(rim))) < 0.15 * abs(float(cv2.contourArea(contour))):
        return None, {"note": "parallel rim area too small"}
    return rim, {
        "note": "front_parallel_ok",
        "front_group": len(front_group),
        "back_candidates": len(parallel_above),
        "back_distance": back_distance,
        "front_angle_deg": float(np.degrees(front_angle)),
    }


def extract_side_mid_rim(mask: np.ndarray, contour: np.ndarray, edges: np.ndarray | None = None) -> tuple[np.ndarray | None, dict[str, Any]]:
    """Build a box rim from stable left/right mask side walls.

    This mode is intended for loaded boxes. It avoids fitting the back/green
    edge from internal object edges. The quadrilateral is derived from the
    outer side walls of the selected mask; the green edge is only the connection
    between the two upper side points.
    """
    ys, xs = np.nonzero(mask)
    if len(xs) < 100:
        return None, {"note": "mask too small"}

    x, y, width, height = cv2.boundingRect(cv2.convexHull(contour))
    rows: list[tuple[int, int, int, int]] = []
    for yy in range(int(ys.min()), int(ys.max()) + 1):
        cols = xs[ys == yy]
        if len(cols) < 12:
            continue
        rows.append((yy, int(cols.min()), int(cols.max()), int(cols.max() - cols.min())))
    if len(rows) < 20:
        return None, {"note": "not enough mask rows"}

    widths = np.asarray([row[3] for row in rows], dtype=np.float32)
    max_width = float(widths.max())
    if max_width < 80:
        return None, {"note": "mask span too narrow"}

    # Ignore top protrusions/background fragments and the very bottom taper.
    y_lo = y + 0.18 * height
    y_hi = y + 0.88 * height
    side_rows = [
        row
        for row, row_width in zip(rows, widths)
        if y_lo <= row[0] <= y_hi and row_width >= 0.52 * max_width
    ]
    if len(side_rows) < 20:
        side_rows = [
            row
            for row, row_width in zip(rows, widths)
            if y_lo <= row[0] <= y_hi and row_width >= 0.42 * max_width
        ]
    if len(side_rows) < 12:
        return None, {"note": "not enough stable side rows"}

    left_points = np.asarray([(left, yy) for yy, left, _, _ in side_rows], dtype=np.float32)
    right_points = np.asarray([(right, yy) for yy, _, right, _ in side_rows], dtype=np.float32)
    left_line = _fit_line_from_points(left_points)
    right_line = _fit_line_from_points(right_points)
    if left_line is None or right_line is None:
        return None, {"note": "side line fit failed"}

    top_y = int(np.percentile([row[0] for row in side_rows], 8))
    bottom_y = int(np.percentile([row[0] for row in side_rows], 92))
    if bottom_y <= top_y + 35:
        return None, {"note": "side wall height too small"}

    top_left = _line_point_at_y(left_line, top_y)
    top_right = _line_point_at_y(right_line, top_y)
    front_left = _line_point_at_y(left_line, bottom_y)
    front_right = _line_point_at_y(right_line, bottom_y)
    if any(point is None for point in (top_left, front_left, top_right, front_right)):
        return None, {"note": "side endpoint failed"}

    front_line_note = "row_percentile"
    if edges is not None:
        roi = np.zeros(mask.shape, dtype=np.uint8)
        cv2.drawContours(roi, [cv2.convexHull(contour)], -1, 255, -1)
        roi = cv2.dilate(roi, np.ones((13, 13), np.uint8))
        candidate_edges = cv2.bitwise_and(edges, roi)
        min_line_length = max(45, int(round(0.18 * width)))
        lines = cv2.HoughLinesP(
            candidate_edges,
            1,
            np.pi / 360.0,
            threshold=20,
            minLineLength=min_line_length,
            maxLineGap=35,
        )
        if lines is not None:
            candidates: list[tuple[np.ndarray, float, float]] = []
            for raw in lines[:, 0].astype(np.float64):
                angle = _line_angle(raw)
                if _angle_delta(angle, 0.0) > np.deg2rad(18.0):
                    continue
                mid_y = 0.5 * (raw[1] + raw[3])
                frac_y = (mid_y - y) / max(float(height), 1.0)
                # Box front/opening rim is below the upper edge but above the
                # lower outside bottom edge. This avoids the 92-percentile row.
                if not 0.52 <= frac_y <= 0.78:
                    continue
                length = float(np.linalg.norm(raw[2:] - raw[:2]))
                score = length - 0.45 * abs(frac_y - 0.66) * height
                candidates.append((raw, frac_y, score))
            if candidates:
                seed, seed_frac, _ = max(candidates, key=lambda item: item[2])
                seed_angle = _line_angle(seed)
                seed_mid_y = 0.5 * (seed[1] + seed[3])
                group = [
                    raw
                    for raw, _, _ in candidates
                    if _angle_delta(_line_angle(raw), seed_angle) <= np.deg2rad(8.0)
                    and abs(0.5 * (raw[1] + raw[3]) - seed_mid_y) <= 18.0
                ]
                front_line = _fit_line_from_segments(group)
                if front_line is not None:
                    fl = _line_intersection(left_line, front_line)
                    fr = _line_intersection(right_line, front_line)
                    if fl is not None and fr is not None:
                        x_min = min(float(fl[0]), float(fr[0]), float(top_left[0]), float(top_right[0]))
                        x_max = max(float(fl[0]), float(fr[0]), float(top_left[0]), float(top_right[0]))
                        front_mid_y = 0.5 * (float(fl[1]) + float(fr[1]))
                        top_line, top_meta = _robust_top_boundary_line(
                            mask,
                            x_min=x_min,
                            x_max=x_max,
                            y_min=y + 0.02 * height,
                            y_max=front_mid_y - 28.0,
                            reference_angle=seed_angle,
                        )
                        if top_line is None:
                            top_mid = 0.5 * (np.asarray(top_left, dtype=np.float64) + np.asarray(top_right, dtype=np.float64))
                            top_distance = _point_line_signed_distance(top_mid, front_line)
                            top_line = _shift_line(front_line, top_distance)
                            top_meta = {"top_fit": "parallel_fallback"}
                        tl = _line_intersection(left_line, top_line)
                        tr = _line_intersection(right_line, top_line)
                        if (
                            tl is not None
                            and tr is not None
                            and np.isfinite(tl).all()
                            and np.isfinite(tr).all()
                            and 0.5 * (tl[1] + tr[1]) < 0.5 * (fl[1] + fr[1]) - 25.0
                        ):
                            top_left = tl
                            top_right = tr
                        front_left = fl
                        front_right = fr
                        bottom_y = int(round(0.5 * (front_left[1] + front_right[1])))
                        top_note = ",".join(f"{key}={value}" for key, value in top_meta.items())
                        front_line_note = f"hough_front_robust_top frac={seed_frac:.2f} group={len(group)} {top_note}"

    rim = np.rint(np.array([top_left, top_right, front_right, front_left], dtype=np.float64)).astype(np.int32)
    rim[:, 0] = np.clip(rim[:, 0], 0, mask.shape[1] - 1)
    rim[:, 1] = np.clip(rim[:, 1], 0, mask.shape[0] - 1)
    if abs(float(cv2.contourArea(rim))) < 0.18 * abs(float(cv2.contourArea(contour))):
        return None, {"note": "side-mid rim area too small"}
    return rim, {
        "note": "side_mid_ok",
        "stable_rows": len(side_rows),
        "top_y": top_y,
        "bottom_y": bottom_y,
        "max_width": max_width,
        "front_line": front_line_note,
    }


def extract_top_rim(
    edges: np.ndarray,
    contour: np.ndarray,
    *,
    target_area_ratio: float = 0.75,
    min_area_ratio: float = 0.30,
    max_area_ratio: float = 1.10,
) -> np.ndarray | None:
    """Fit a four-side rim quadrilateral around a candidate contour."""
    hull = cv2.convexHull(contour)
    x, y, width, height = cv2.boundingRect(hull)
    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        return None
    center = np.array([moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]], dtype=np.float64)

    roi = np.zeros(edges.shape, dtype=np.uint8)
    cv2.drawContours(roi, [hull], -1, 255, -1)
    search_radius = max(9, int(round(0.18 * min(width, height))))
    roi = cv2.dilate(roi, np.ones((search_radius * 2 + 1, search_radius * 2 + 1), np.uint8))
    candidate_edges = cv2.bitwise_and(edges, roi)
    min_line_length = max(35, int(round(0.10 * max(width, height))))
    lines = cv2.HoughLinesP(
        candidate_edges,
        1,
        np.pi / 360.0,
        threshold=22,
        minLineLength=min_line_length,
        maxLineGap=40,
    )
    if lines is None:
        return None

    object_area = abs(float(cv2.contourArea(contour)))
    min_radial_distance = 0.06 * min(width, height)
    candidates: list[tuple[np.ndarray, float, float, float]] = []
    for line in lines[:, 0].astype(np.float64):
        start, end = line[:2], line[2:]
        direction = end - start
        line_length = float(np.linalg.norm(direction))
        if line_length < 1.0:
            continue
        normal = np.array([-direction[1], direction[0]]) / line_length
        constant = -float(np.dot(normal, start))
        center_value = float(np.dot(normal, center) + constant)
        if center_value > 0:
            normal = -normal
            constant = -constant
            center_value = -center_value
        distance = -center_value
        if distance < min_radial_distance:
            continue
        normal_angle = float(np.arctan2(normal[1], normal[0]))
        candidates.append((line, normal_angle, distance, line_length))

    unique: list[tuple[np.ndarray, float, float, float]] = []
    for candidate in sorted(candidates, key=lambda item: item[3], reverse=True):
        _, angle, distance, _ = candidate
        duplicate = False
        for _, kept_angle, kept_distance, _ in unique:
            angle_delta = abs(np.arctan2(np.sin(angle - kept_angle), np.cos(angle - kept_angle)))
            if angle_delta < np.deg2rad(5.0) and abs(distance - kept_distance) < 12.0:
                duplicate = True
                break
        if not duplicate:
            unique.append(candidate)
        if len(unique) >= 28:
            break

    best_rim: np.ndarray | None = None
    best_score = -float("inf")
    margin_x, margin_y = 0.45 * width, 0.45 * height
    for group in combinations(unique, 4):
        ordered = sorted(group, key=lambda item: item[1])
        angles = np.array([item[1] for item in ordered])
        gaps = np.diff(np.r_[angles, angles[0] + 2.0 * np.pi])
        if np.any(gaps < np.deg2rad(25.0)) or np.any(gaps > np.deg2rad(155.0)):
            continue
        opposite_ok = True
        for opposite_index in (0, 1):
            difference = abs(np.arctan2(
                np.sin(angles[opposite_index + 2] - angles[opposite_index]),
                np.cos(angles[opposite_index + 2] - angles[opposite_index]),
            ))
            if difference < np.deg2rad(140.0):
                opposite_ok = False
                break
        if not opposite_ok:
            continue

        corners: list[np.ndarray] = []
        for index in range(4):
            point = _line_intersection(ordered[index][0], ordered[(index + 1) % 4][0])
            if point is None or not np.all(np.isfinite(point)):
                corners = []
                break
            corners.append(point)
        if not corners:
            continue
        rim_float = np.array(corners)
        if (
            np.any(rim_float[:, 0] < x - margin_x)
            or np.any(rim_float[:, 0] > x + width + margin_x)
            or np.any(rim_float[:, 1] < y - margin_y)
            or np.any(rim_float[:, 1] > y + height + margin_y)
        ):
            continue
        rim = np.rint(rim_float).astype(np.int32)
        if cv2.pointPolygonTest(rim, tuple(center), False) < 0:
            continue
        rim_area = abs(float(cv2.contourArea(rim)))
        area_ratio = rim_area / max(object_area, 1.0)
        if not min_area_ratio <= area_ratio <= max_area_ratio:
            continue
        total_line_length = sum(item[3] for item in ordered)
        score = total_line_length - 450.0 * abs(area_ratio - target_area_ratio)
        if score > best_score:
            best_score = score
            best_rim = rim

    if best_rim is None:
        return None
    best_rim[:, 0] = np.clip(best_rim[:, 0], 0, edges.shape[1] - 1)
    best_rim[:, 1] = np.clip(best_rim[:, 1], 0, edges.shape[0] - 1)
    best_rim = refine_rim_from_edge_pixels(best_rim, candidate_edges)
    start = int(np.argmin(best_rim[:, 1]))
    return np.roll(best_rim, -start, axis=0)


def _order_rim_top_bottom(rim: np.ndarray) -> dict[str, tuple[int, int]]:
    pts = np.asarray(rim, dtype=np.float32).reshape(4, 2)
    top = sorted(sorted(pts, key=lambda p: (float(p[1]), float(p[0])))[:2], key=lambda p: float(p[0]))
    bottom = sorted(sorted(pts, key=lambda p: (float(p[1]), float(p[0])))[2:], key=lambda p: float(p[0]))
    return {
        "top_left": (int(round(top[0][0])), int(round(top[0][1]))),
        "top_right": (int(round(top[1][0])), int(round(top[1][1]))),
        "front_left": (int(round(bottom[0][0])), int(round(bottom[0][1]))),
        "front_right": (int(round(bottom[1][0])), int(round(bottom[1][1]))),
    }


def _interp(p0: tuple[int, int], p1: tuple[int, int], frac: float) -> tuple[int, int]:
    return (
        int(round(p0[0] + (p1[0] - p0[0]) * frac)),
        int(round(p0[1] + (p1[1] - p0[1]) * frac)),
    )


def _point_in_mask(mask: np.ndarray, point: tuple[int, int], *, radius: int = 2) -> bool:
    h, w = mask.shape[:2]
    x, y = point
    if x < 0 or y < 0 or x >= w or y >= h:
        return False
    x0 = max(0, x - radius)
    x1 = min(w, x + radius + 1)
    y0 = max(0, y - radius)
    y1 = min(h, y + radius + 1)
    return bool(np.any(mask[y0:y1, x0:x1] > 0))


def _interp_on_mask(
    mask: np.ndarray,
    p0: tuple[int, int],
    p1: tuple[int, int],
    frac: float,
) -> tuple[tuple[int, int], dict[str, Any]]:
    """Interpolate on a side edge, then clamp to the nearest in-mask side point."""
    target = _interp(p0, p1, frac)
    if _point_in_mask(mask, target, radius=2):
        return target, {"adjusted": False}

    samples: list[tuple[float, tuple[int, int]]] = []
    for t in np.linspace(0.0, 1.0, 121):
        point = _interp(p0, p1, float(t))
        if _point_in_mask(mask, point, radius=4):
            samples.append((abs(float(t) - frac), point))
    if not samples:
        return target, {"adjusted": False, "reason": "no side sample in mask"}

    _, point = min(samples, key=lambda item: item[0])
    return point, {
        "adjusted": True,
        "from": [int(target[0]), int(target[1])],
        "to": [int(point[0]), int(point[1])],
    }


def _objects_from_points(points: dict[str, tuple[int, int]], depth: np.ndarray, camera: CameraModel) -> list[dict[str, Any]]:
    objects: list[dict[str, Any]] = []
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


def estimate_box_rim_grasp_from_mask(
    image_bgr: np.ndarray,
    depth: np.ndarray,
    camera: CameraModel,
    mask: np.ndarray,
    *,
    handle_y_frac: float = 0.50,
    depth_scale: float = 0.001,
    min_edge_support: float = 0.08,
    rim_fit_mode: str = "free",
) -> tuple[dict[str, Any], list[dict[str, Any]], dict[str, np.ndarray]]:
    """Estimate grasp points from a candidate mask using RGB/depth rim edges."""
    mask = (mask > 0).astype(np.uint8) * 255
    depth_mm = depth.astype(np.float32) / max(depth_scale, 1e-9)
    depth_edges, _ = metric_depth_edges(depth_mm)
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    color_edges = cv2.Canny(gray, 50, 140)
    candidate_edges = cv2.bitwise_or(color_edges, depth_edges)
    candidate_edges = cv2.bitwise_and(candidate_edges, cv2.dilate(mask, np.ones((15, 15), np.uint8)))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return {"ok": False, "note": "no mask contour", "mask": mask}, [], {"rim_edges": candidate_edges}
    contour = max(contours, key=cv2.contourArea)
    rim_fit_mode = str(rim_fit_mode or "free").lower()
    rim_meta: dict[str, Any] = {"mode": rim_fit_mode}
    if rim_fit_mode == "side-mid":
        rim, rim_meta = extract_side_mid_rim(mask, contour, candidate_edges)
        rim_meta["mode"] = rim_fit_mode
        rim_source = "side_mid"
    elif rim_fit_mode == "front-parallel":
        rim, rim_meta = extract_front_parallel_rim(candidate_edges, contour)
        rim_meta["mode"] = rim_fit_mode
        rim_source = "front_parallel"
    else:
        rim = None
        rim_source = "free_four_line"

    if rim is None:
        rim = extract_top_rim(candidate_edges, contour)
        previous_note = rim_meta.get("note", "")
        rim_meta = {"mode": rim_fit_mode, "note": f"{previous_note}; fallback free_four_line".strip("; ")}
        rim_source = "free_four_line"
    if rim is None:
        return {"ok": False, "note": "rim extraction failed", "mask": mask, "rim_meta": rim_meta}, [], {"rim_edges": candidate_edges}

    support = edge_support(rim, candidate_edges)
    if support < min_edge_support:
        return {
            "ok": False,
            "note": f"rim edge support too low: {support:.3f}",
            "mask": mask,
            "rim": rim.tolist(),
            "edge_support": support,
        }, [], {"rim_edges": candidate_edges}

    corners = _order_rim_top_bottom(rim)
    frac = float(np.clip(handle_y_frac, 0.0, 1.0))
    left_point, left_adjust = _interp_on_mask(mask, corners["top_left"], corners["front_left"], frac)
    right_point, right_adjust = _interp_on_mask(mask, corners["top_right"], corners["front_right"], frac)
    points = {
        "left": left_point,
        "right": right_point,
    }
    if left_adjust.get("adjusted") or right_adjust.get("adjusted"):
        rim_meta["handle_point_adjustment"] = {
            "left": left_adjust,
            "right": right_adjust,
        }
    objects = _objects_from_points(points, depth, camera)
    result = {
        "ok": any(obj.get("valid") for obj in objects),
        "note": "rim_ok" if any(obj.get("valid") for obj in objects) else "rim_no_depth",
        "mask": mask,
        "source": "box_rim",
        "rim": rim.tolist(),
        "rim_corners": corners,
        "rim_source": rim_source,
        "rim_meta": rim_meta,
        "edge_support": support,
        "handle_y_frac": frac,
    }
    if not result["ok"]:
        result["note"] = "rim_no_depth"
    return result, objects, {"rim_edges": candidate_edges}


def draw_box_rim_result(image_bgr: np.ndarray, result: dict[str, Any], objects: list[dict[str, Any]]) -> np.ndarray:
    vis = image_bgr.copy()
    mask = result.get("mask")
    if mask is not None:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(vis, contours, -1, (0, 165, 255), 2)
    rim = result.get("rim")
    if rim is not None:
        rim_arr = np.asarray(rim, dtype=np.int32).reshape(-1, 2)
        cv2.polylines(vis, [rim_arr], True, (0, 255, 0), 3, cv2.LINE_AA)
        corners = result.get("rim_corners") or {}
        for a, b, color in [
            ("top_left", "top_right", (0, 255, 0)),
            ("front_left", "front_right", (0, 0, 255)),
            ("top_left", "front_left", (255, 0, 0)),
            ("top_right", "front_right", (255, 0, 0)),
        ]:
            if corners.get(a) and corners.get(b):
                cv2.line(vis, tuple(corners[a]), tuple(corners[b]), color, 3)
    for obj in objects:
        u, v = obj["center"]
        color = (255, 0, 255) if obj.get("side") == "left" else (0, 255, 255)
        cv2.circle(vis, (int(u), int(v)), 8, color, -1)
        cv2.putText(vis, f"{obj['side']} {obj['status']}", (int(u) - 80, max(20, int(v) - 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    cv2.putText(
        vis,
        f"{result.get('note', '')} support={result.get('edge_support', 0.0):.2f}",
        (20, 36),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255) if result.get("ok") else (0, 0, 255),
        2,
    )
    return vis
