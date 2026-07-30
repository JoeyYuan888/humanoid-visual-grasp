"""
深度图工具函数
- 获取某像素点的 3D 坐标 (相机坐标系)
"""

import numpy as np

from . import config


def get_depth_at(depth: np.ndarray, u: int, v: int) -> float | None:
    """
    获取像素 (u, v) 处的深度值（毫米）。
    若无效则返回 None。
    """
    h, w = depth.shape[:2]
    if not (0 <= u < w and 0 <= v < h):
        return None
    d = float(depth[v, u])
    if d < config.DEPTH_MIN_MM or d > config.DEPTH_MAX_MM:
        return None
    return d


def get_depth_roi_median(depth: np.ndarray, x1: int, y1: int, x2: int, y2: int) -> float | None:
    """
    获取 bbox 区域内深度值的中位数（毫米），比中心单点更鲁棒。
    若有效像素不足则返回 None。
    """
    roi = depth[y1:y2, x1:x2]
    mask = (roi >= config.DEPTH_MIN_MM) & (roi <= config.DEPTH_MAX_MM)
    valid = roi[mask]
    if len(valid) < 10:   # 有效点太少
        return None
    return float(np.median(valid))


def get_depth_roi_stats(depth: np.ndarray, x1: int, y1: int, x2: int, y2: int):
    """统计 ROI 内有效深度数量和中位数。"""
    h, w = depth.shape[:2]
    x1 = max(0, min(w, x1))
    x2 = max(0, min(w, x2))
    y1 = max(0, min(h, y1))
    y2 = max(0, min(h, y2))
    if x2 <= x1 or y2 <= y1:
        return {"valid_count": 0, "total_count": 0, "median_mm": None}

    roi = depth[y1:y2, x1:x2]
    mask = (roi >= config.DEPTH_MIN_MM) & (roi <= config.DEPTH_MAX_MM)
    valid = roi[mask]
    return {
        "valid_count": int(len(valid)),
        "total_count": int(roi.size),
        "median_mm": float(np.median(valid)) if len(valid) else None,
    }


def pixel_to_3d(u: int, v: int, depth_mm: float,
                fx: float, fy: float, cx: float, cy: float):
    """
    将像素坐标 + 深度 转换到相机坐标系下的 3D 点。

    返回:
        (x, y, z) 单位: 毫米
        相机坐标系: z 向前, x 向右, y 向下
    """
    z = depth_mm
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return x, y, z


def _ratio_point(x1: int, y1: int, x2: int, y2: int, rx: float, ry: float) -> tuple[int, int]:
    """Return a pixel inside bbox selected by normalized ratios."""
    rx = max(0.0, min(1.0, float(rx)))
    ry = max(0.0, min(1.0, float(ry)))
    u = int(round(x1 + (x2 - x1) * rx))
    v = int(round(y1 + (y2 - y1) * ry))
    return u, v


def _ratio_roi(x1: int, y1: int, x2: int, y2: int) -> tuple[int, int, int, int]:
    """Return the configured depth sampling ROI inside bbox."""
    rx1 = max(0.0, min(1.0, float(config.DEPTH_ROI_X1_RATIO)))
    rx2 = max(0.0, min(1.0, float(config.DEPTH_ROI_X2_RATIO)))
    ry1 = max(0.0, min(1.0, float(config.DEPTH_ROI_Y1_RATIO)))
    ry2 = max(0.0, min(1.0, float(config.DEPTH_ROI_Y2_RATIO)))
    if rx2 < rx1:
        rx1, rx2 = rx2, rx1
    if ry2 < ry1:
        ry1, ry2 = ry2, ry1
    return (
        int(round(x1 + (x2 - x1) * rx1)),
        int(round(y1 + (y2 - y1) * ry1)),
        int(round(x1 + (x2 - x1) * rx2)),
        int(round(y1 + (y2 - y1) * ry2)),
    )


def compute_grasp_point(det: dict, depth: np.ndarray,
                        fx: float, fy: float, cx: float, cy: float):
    """
    给定一个检测结果和深度图，计算抓取点 3D 坐标。

    策略:
        1. 在 bbox 内按 config.DEPTH_ROI_*_RATIO 取深度 ROI
           当前默认是 bbox 下方 2/3 区域。
        2. 在 bbox 内按 config.GRASP_POINT_*_RATIO 选 3D 射线像素点。
        3. 用 ROI 深度中位数 + 抓取点像素计算 3D 坐标。

    返回:
        dict: { "x_mm", "y_mm", "z_mm", "depth_mm", "valid" }
    """
    x1, y1, x2, y2 = det["bbox"]
    cu, cv = _ratio_point(
        x1, y1, x2, y2,
        config.GRASP_POINT_X_RATIO,
        config.GRASP_POINT_Y_RATIO,
    )

    rx1, ry1, rx2, ry2 = _ratio_roi(x1, y1, x2, y2)
    roi_stats = get_depth_roi_stats(depth, rx1, ry1, rx2, ry2)
    depth_val = get_depth_roi_median(depth, rx1, ry1, rx2, ry2)

    # 如果 ROI 无效，退而求其次用抓取点像素。
    if depth_val is None:
        depth_val = get_depth_at(depth, cu, cv)

    if depth_val is None:
        return {
            "valid": False,
            "status": "invalid depth roi",
            "roi_valid_count": roi_stats["valid_count"],
            "roi_total_count": roi_stats["total_count"],
            "point_u": cu,
            "point_v": cv,
            "depth_roi": (rx1, ry1, rx2, ry2),
        }

    x3d, y3d, z3d = pixel_to_3d(cu, cv, depth_val, fx, fy, cx, cy)

    return {
        "valid": True,
        "x_mm": round(x3d, 1),
        "y_mm": round(y3d, 1),
        "z_mm": round(z3d, 1),
        "depth_mm": round(depth_val, 1),
        "roi_valid_count": roi_stats["valid_count"],
        "roi_total_count": roi_stats["total_count"],
        "point_u": cu,
        "point_v": cv,
        "depth_roi": (rx1, ry1, rx2, ry2),
    }
