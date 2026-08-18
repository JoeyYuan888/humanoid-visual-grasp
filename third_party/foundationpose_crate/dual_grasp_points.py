#!/usr/bin/env python3
"""Measure or predict both CAD short-edge centres and report occlusion."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np


SHORT_EDGES = {
    "-x": {"indices": (0, 3), "outward_normal": (-1.0, 0.0, 0.0)},
    "+x": {"indices": (1, 2), "outward_normal": (1.0, 0.0, 0.0)},
}


def transform_points(points: np.ndarray, pose: np.ndarray) -> np.ndarray:
    homogeneous = np.column_stack((points, np.ones(len(points))))
    return (pose @ homogeneous.T).T[:, :3]


def project_points(
    points: np.ndarray,
    pose: np.ndarray,
    camera_matrix: np.ndarray,
) -> np.ndarray:
    camera_points = transform_points(points, pose)
    if np.any(camera_points[:, 2] <= 0.001):
        raise ValueError("short-edge rim is behind the camera")
    pixels_h = (camera_matrix @ camera_points.T).T
    return pixels_h[:, :2] / pixels_h[:, 2:3]


def edge_geometry(
    edge_name: str,
    outer: np.ndarray,
    inner: np.ndarray,
    pose: np.ndarray,
    camera_matrix: np.ndarray,
) -> dict[str, Any]:
    spec = SHORT_EDGES[edge_name]
    first, second = spec["indices"]
    band_cad = np.stack((outer[first], outer[second], inner[second], inner[first]))
    midpoint_cad = band_cad.mean(axis=0)
    midpoint_camera = transform_points(midpoint_cad[None], pose)[0]
    band_pixels = project_points(band_cad, pose, camera_matrix)
    midpoint_pixel = project_points(midpoint_cad[None], pose, camera_matrix)[0]
    centreline_cad = np.stack((
        (outer[first] + inner[first]) * 0.5,
        (outer[second] + inner[second]) * 0.5,
    ))
    centreline_pixels = project_points(centreline_cad, pose, camera_matrix)
    outward_camera = pose[:3, :3] @ np.asarray(spec["outward_normal"], dtype=np.float64)
    outward_camera /= np.linalg.norm(outward_camera)
    return {
        "cad_short_edge": edge_name,
        "cad_rim_midpoint_m": midpoint_cad.tolist(),
        "cad_midpoint_in_camera_m": midpoint_camera.tolist(),
        "projected_midpoint_pixel": midpoint_pixel.tolist(),
        "projected_edge_centerline_pixels": centreline_pixels.tolist(),
        "rim_band_polygon_pixels": band_pixels.tolist(),
        "outward_direction_camera": outward_camera.tolist(),
    }


def expected_top_plane_depths(
    columns: np.ndarray,
    rows: np.ndarray,
    camera_matrix: np.ndarray,
    pose: np.ndarray,
    top_z_m: float,
) -> np.ndarray:
    """Intersect camera rays with the CAD top-rim plane."""
    fx, fy = float(camera_matrix[0, 0]), float(camera_matrix[1, 1])
    cx, cy = float(camera_matrix[0, 2]), float(camera_matrix[1, 2])
    rays = np.column_stack(((columns - cx) / fx, (rows - cy) / fy, np.ones(len(columns))))
    normal = pose[:3, :3] @ np.array((0.0, 0.0, 1.0), dtype=np.float64)
    point = transform_points(np.array([[0.0, 0.0, top_z_m]]), pose)[0]
    denominator = rays @ normal
    numerator = float(point @ normal)
    depths = np.full(len(columns), np.nan, dtype=np.float64)
    usable = np.abs(denominator) > 1e-6
    depths[usable] = numerator / denominator[usable]
    return depths


def deproject_pixel(pixel: np.ndarray, depth_m: float, camera_matrix: np.ndarray) -> list[float]:
    fx, fy = float(camera_matrix[0, 0]), float(camera_matrix[1, 1])
    cx, cy = float(camera_matrix[0, 2]), float(camera_matrix[1, 2])
    u, v = float(pixel[0]), float(pixel[1])
    return [(u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m]


def robust_visible_rim_prediction(
    rows: np.ndarray,
    columns: np.ndarray,
    observed_depths: np.ndarray,
    expected_depths: np.ndarray,
    midpoint_pixel: np.ndarray,
    centreline_pixels: np.ndarray,
    expected_midpoint_depth: float,
    camera_matrix: np.ndarray,
    minimum_inliers: int,
    minimum_span_ratio: float,
    maximum_rmse_m: float,
) -> dict[str, Any]:
    residuals = observed_depths - expected_depths
    median = float(np.median(residuals))
    mad = float(np.median(np.abs(residuals - median)))
    inlier_limit = max(0.006, 3.0 * 1.4826 * mad)
    inliers = np.abs(residuals - median) <= inlier_limit
    inlier_count = int(np.count_nonzero(inliers))
    diagnostic: dict[str, Any] = {
        "visible_rim_depth_candidates": int(len(residuals)),
        "prediction_inliers": inlier_count,
        "prediction_residual_median_m": median,
        "prediction_residual_mad_m": mad,
        "prediction_inlier_limit_m": inlier_limit,
    }
    if inlier_count < minimum_inliers:
        diagnostic["prediction_error"] = (
            f"only {inlier_count} visible rim inliers; need {minimum_inliers}"
        )
        return diagnostic

    line = centreline_pixels[1] - centreline_pixels[0]
    line_length = float(np.linalg.norm(line))
    if line_length < 1.0:
        diagnostic["prediction_error"] = "projected short edge is too small"
        return diagnostic
    axis = line / line_length
    pixels = np.column_stack((columns[inliers], rows[inliers]))
    longitudinal = (pixels - midpoint_pixel) @ axis
    low, high = np.percentile(longitudinal, (5.0, 95.0))
    span_ratio = float((high - low) / line_length)
    diagnostic["visible_edge_span_ratio"] = span_ratio
    diagnostic["visible_samples_bracket_center"] = bool(low <= 0.0 <= high)
    if span_ratio < minimum_span_ratio:
        diagnostic["prediction_error"] = (
            f"visible short-edge span is {span_ratio:.3f}; need {minimum_span_ratio:.3f}"
        )
        return diagnostic

    correction = float(np.median(residuals[inliers]))
    errors = residuals[inliers] - correction
    rmse = float(np.sqrt(np.mean(errors * errors)))
    diagnostic["prediction_fit_rmse_m"] = rmse
    diagnostic["prediction_depth_correction_m"] = correction
    if rmse > maximum_rmse_m:
        diagnostic["prediction_error"] = (
            f"visible rim depth fit RMSE is {rmse:.4f}m; limit is {maximum_rmse_m:.4f}m"
        )
        return diagnostic

    predicted_depth = expected_midpoint_depth + correction
    if not np.isfinite(predicted_depth) or predicted_depth <= 0.001:
        diagnostic["prediction_error"] = "predicted centre depth is invalid"
        return diagnostic
    diagnostic.update({
        "prediction_valid": True,
        "predicted_depth_m": predicted_depth,
        "point_camera_m": deproject_pixel(midpoint_pixel, predicted_depth, camera_matrix),
    })
    return diagnostic


def measure_edge(
    edge_name: str,
    depth_m: np.ndarray,
    crate_color_mask: np.ndarray,
    camera_matrix: np.ndarray,
    pose: np.ndarray,
    outer: np.ndarray,
    inner: np.ndarray,
    depth_tolerance_m: float,
    max_sample_offset_pixels: float,
    interior_scale: float,
    center_color_radius_pixels: float,
    center_color_min_ratio: float,
    direct_max_offset_pixels: float,
    prediction_min_inliers: int,
    prediction_min_edge_span_ratio: float,
    prediction_max_fit_rmse_m: float,
) -> dict[str, Any]:
    result = edge_geometry(edge_name, outer, inner, pose, camera_matrix)
    band_pixels = np.asarray(result["rim_band_polygon_pixels"], dtype=np.float64)
    midpoint_pixel = np.asarray(result["projected_midpoint_pixel"], dtype=np.float64)
    centreline_pixels = np.asarray(result["projected_edge_centerline_pixels"], dtype=np.float64)

    safe_pixels = midpoint_pixel + interior_scale * (band_pixels - midpoint_pixel)
    allowed = np.zeros(depth_m.shape, dtype=np.uint8)
    cv2.fillConvexPoly(allowed, np.rint(safe_pixels).astype(np.int32), 1, cv2.LINE_8)
    height, width = depth_m.shape
    center_in_image = bool(
        0.0 <= midpoint_pixel[0] < width and 0.0 <= midpoint_pixel[1] < height
    )
    yy, xx = np.ogrid[:height, :width]
    center_circle = (
        (xx - midpoint_pixel[0]) ** 2 + (yy - midpoint_pixel[1]) ** 2
        <= center_color_radius_pixels ** 2
    )
    center_region = (allowed > 0) & center_circle
    center_region_pixels = int(np.count_nonzero(center_region))
    center_color_pixels = int(np.count_nonzero(center_region & (crate_color_mask > 0)))
    center_color_ratio = (
        float(center_color_pixels / center_region_pixels) if center_region_pixels else 0.0
    )
    center_visible = bool(center_in_image and center_color_ratio >= center_color_min_ratio)
    occluded: bool | None = (not center_visible) if center_in_image else None
    occlusion_state = (
        "visible"
        if center_visible
        else "occluded_by_non_crate_color"
        if center_in_image
        else "unknown_out_of_view"
    )
    result.update({
        "valid": False,
        "coordinate_valid": False,
        "source": "invalid",
        "occluded": occluded,
        "occlusion_state": occlusion_state,
        "grasp_clear": bool(center_visible),
        "center_in_image": center_in_image,
        "center_color_ratio": center_color_ratio,
        "center_color_pixels": center_color_pixels,
        "center_region_pixels": center_region_pixels,
        "safe_rim_polygon_pixels": safe_pixels.tolist(),
    })
    if not center_in_image:
        result["error"] = f"{edge_name} short-edge centre is outside the image"
        return result

    raw_candidates = (
        (allowed > 0)
        & (crate_color_mask > 0)
        & np.isfinite(depth_m)
        & (depth_m > 0.001)
    )
    rows, columns = np.where(raw_candidates)
    if len(columns) == 0:
        result["visible_rim_depth_candidates"] = 0
        result["error"] = f"no visible crate-colour depth inside {edge_name} rim band"
        return result

    top_z_m = float(np.mean(np.concatenate((outer[:, 2], inner[:, 2]))))
    expected_depths = expected_top_plane_depths(
        columns.astype(np.float64), rows.astype(np.float64), camera_matrix, pose, top_z_m,
    )
    observed_depths = depth_m[rows, columns].astype(np.float64)
    consistent = (
        np.isfinite(expected_depths)
        & (expected_depths > 0.001)
        & (np.abs(observed_depths - expected_depths) <= depth_tolerance_m)
    )
    rows = rows[consistent]
    columns = columns[consistent]
    observed_depths = observed_depths[consistent]
    expected_depths = expected_depths[consistent]
    result["valid_depth_candidates_in_safe_rim"] = int(len(columns))
    if len(columns) == 0:
        result["error"] = f"no CAD-consistent visible depth inside {edge_name} rim band"
        return result

    squared = (columns - midpoint_pixel[0]) ** 2 + (rows - midpoint_pixel[1]) ** 2
    best = int(np.argmin(squared))
    offset = float(np.sqrt(squared[best]))
    result["nearest_visible_sample_offset_pixels"] = offset

    if center_visible and offset <= direct_max_offset_pixels:
        u, v = int(columns[best]), int(rows[best])
        z = float(observed_depths[best])
        result.update({
            "valid": True,
            "coordinate_valid": True,
            "source": "measured",
            "sample_pixel": [u, v],
            "sample_offset_pixels": offset,
            "depth_m": z,
            "point_camera_m": deproject_pixel(np.array((u, v)), z, camera_matrix),
            "depth_error_to_cad_m": z - float(expected_depths[best]),
            "sample_strictly_inside_rim_band": bool(allowed[v, u]),
        })
        return result

    prediction = robust_visible_rim_prediction(
        rows,
        columns,
        observed_depths,
        expected_depths,
        midpoint_pixel,
        centreline_pixels,
        float(result["cad_midpoint_in_camera_m"][2]),
        camera_matrix,
        prediction_min_inliers,
        prediction_min_edge_span_ratio,
        prediction_max_fit_rmse_m,
    )
    result.update(prediction)
    if prediction.get("prediction_valid"):
        result.update({
            "valid": True,
            "coordinate_valid": True,
            "source": "predicted_from_visible_rim",
            "depth_m": prediction["predicted_depth_m"],
            "prediction_pixel": midpoint_pixel.tolist(),
        })
    else:
        result["error"] = prediction.get("prediction_error", "visible rim prediction failed")
    return result


def find_dual_grasp_points(
    depth_m: np.ndarray,
    crate_color_mask: np.ndarray,
    camera_matrix: np.ndarray,
    pose: np.ndarray,
    metadata: dict[str, Any],
    depth_tolerance_m: float = 0.08,
    max_sample_offset_pixels: float = 12.0,
    interior_scale: float = 0.72,
    center_color_radius_pixels: float = 3.0,
    center_color_min_ratio: float = 0.25,
    direct_max_offset_pixels: float = 2.5,
    prediction_min_inliers: int = 20,
    prediction_min_edge_span_ratio: float = 0.20,
    prediction_max_fit_rmse_m: float = 0.012,
) -> dict[str, Any]:
    """Return two coordinates and per-point occlusion, ordered image-left/right."""
    if depth_m.ndim != 2:
        raise ValueError("depth image must be HxW")
    if crate_color_mask.shape != depth_m.shape:
        raise ValueError("crate colour mask must match the aligned depth image")
    outer = np.asarray(metadata["rim_outer_m"], dtype=np.float64)
    inner = np.asarray(metadata["rim_inner_m"], dtype=np.float64)
    points = [
        measure_edge(
            name, depth_m, crate_color_mask, camera_matrix, pose, outer, inner,
            depth_tolerance_m, max_sample_offset_pixels, interior_scale,
            center_color_radius_pixels, center_color_min_ratio,
            direct_max_offset_pixels, prediction_min_inliers,
            prediction_min_edge_span_ratio, prediction_max_fit_rmse_m,
        )
        for name in ("-x", "+x")
    ]
    points.sort(key=lambda item: float(item["projected_midpoint_pixel"][0]))
    for slot, point in zip(("left", "right"), points):
        point["image_slot"] = slot

    valid = all(point["coordinate_valid"] for point in points)
    known_occlusions = [point["occluded"] for point in points if point["occluded"] is not None]
    any_occluded = any(known_occlusions)
    all_clear = bool(valid and len(known_occlusions) == 2 and not any_occluded)
    result: dict[str, Any] = {
        "valid": valid,
        "coordinate_valid": valid,
        "coordinate_frame": "realsense_head_color_optical_frame",
        "point_order": ["image_left", "image_right"],
        "points_left_to_right": points,
        "any_grasp_point_occluded": bool(any_occluded),
        "all_grasp_points_clear": all_clear,
        "robot_execution_allowed": all_clear,
        "depth_tolerance_m": depth_tolerance_m,
        "max_sample_offset_pixels": max_sample_offset_pixels,
        "cad_short_edge_center_distance_m": float(
            np.linalg.norm(
                np.asarray(points[1]["cad_rim_midpoint_m"])
                - np.asarray(points[0]["cad_rim_midpoint_m"])
            )
        ),
    }
    if valid:
        measured = [np.asarray(point["point_camera_m"]) for point in points]
        result["measured_or_predicted_grasp_width_m"] = float(
            np.linalg.norm(measured[1] - measured[0])
        )
        result["depth_difference_m"] = float(abs(measured[1][2] - measured[0][2]))
        result["reason_codes"] = (
            ["GRASP_POINT_OCCLUDED"] if any_occluded else []
        )
    else:
        result["reason_codes"] = [
            f"{point['cad_short_edge'].upper()}_SHORT_EDGE_COORDINATE_INVALID"
            for point in points if not point["coordinate_valid"]
        ]
    return result


def draw_dual_grasp_points(image: np.ndarray, result: dict[str, Any]) -> None:
    clear_colors = {"left": (255, 0, 255), "right": (255, 255, 0)}
    for point in result["points_left_to_right"]:
        slot = point["image_slot"]
        clear_color = clear_colors[slot]
        color = (0, 165, 255) if point.get("occluded") else clear_color
        polygon = np.rint(point["safe_rim_polygon_pixels"]).astype(np.int32)
        cv2.polylines(
            image, [polygon.reshape((-1, 1, 2))], True,
            (0, 255, 255), 2, cv2.LINE_AA,
        )
        if point["coordinate_valid"]:
            if point["source"] == "measured":
                u, v = point["sample_pixel"]
                marker = cv2.MARKER_CROSS
            else:
                u, v = np.rint(point["prediction_pixel"]).astype(int).tolist()
                marker = cv2.MARKER_DIAMOND
            cv2.drawMarker(image, (u, v), color, marker, 24, 3, cv2.LINE_AA)
            cv2.circle(image, (u, v), 7, (255, 255, 255), 2, cv2.LINE_AA)
        else:
            u, v = np.rint(point["projected_midpoint_pixel"]).astype(int).tolist()
            cv2.drawMarker(image, (u, v), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 22, 3)
            color = (0, 0, 255)
        occ = "?" if point.get("occluded") is None else str(int(point["occluded"]))
        cv2.putText(
            image,
            f"GRASP {slot.upper()} {point['source']} OCC={occ}",
            (u + 10, v - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            color,
            2,
            cv2.LINE_AA,
        )
