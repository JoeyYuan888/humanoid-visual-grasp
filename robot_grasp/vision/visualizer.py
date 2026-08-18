"""
OpenCV 可视化：绘制检测框、深度信息、3D 坐标等
"""

import cv2
import numpy as np

from ..common import config

# 鼠标点击位置 (由 main 中的回调设置)
_click_point = None


def set_click_point(u: int, v: int):
    global _click_point
    _click_point = (u, v)


def draw_overlay(frame: np.ndarray, fps: float, det_count: int, infer_ms: float):
    """绘制顶部信息栏。"""
    h, w = frame.shape[:2]
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w, config.INFO_BAR_HEIGHT), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

    cv2.putText(frame, f"Robot Grasp | {w}x{h}",
                (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    cv2.putText(frame, f"FPS: {fps:.1f} | Infer: {infer_ms:.0f}ms | Det: {det_count}",
                (12, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
    return frame


def draw_detection(frame: np.ndarray, det: dict, idx: int):
    """在帧上绘制单个检测框及 3D 信息。"""
    x1, y1, x2, y2 = det["bbox"]
    cx, cy = det["center"]
    label = det["label"]
    conf = det["confidence"]

    # 随机颜色 (根据 class_id)
    color = _class_color(det["class_id"])

    # 框
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    # 中心点
    cv2.circle(frame, (cx, cy), 4, color, -1)

    # 标签背景
    label_text = f"{idx}. {label} {conf:.2f}"
    (tw, th), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 6, y1), color, -1)
    cv2.putText(frame, label_text, (x1 + 3, y1 - 3),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    return frame


def draw_grasp_info(frame: np.ndarray, det: dict, p3d: dict | None):
    """在检测框下方绘制 3D 坐标信息。"""
    x1, y1, x2, y2 = det["bbox"]
    color = _class_color(det["class_id"])

    if p3d:
        roi = p3d.get("depth_roi")
        if roi:
            rx1, ry1, rx2, ry2 = [int(v) for v in roi]
            cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), (255, 180, 0), 2)
            cv2.putText(frame, "depth ROI", (rx1, max(18, ry1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 180, 0), 1)

        point_u = p3d.get("point_u")
        point_v = p3d.get("point_v")
        if point_u != "" and point_v != "" and point_u is not None and point_v is not None:
            u, v = int(point_u), int(point_v)
            cv2.drawMarker(frame, (u, v), (0, 0, 255),
                           markerType=cv2.MARKER_CROSS, markerSize=18, thickness=2)
            cv2.circle(frame, (u, v), 5, (0, 0, 255), 2)
            cv2.putText(frame, "grasp point", (u + 8, max(18, v - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

    if p3d and p3d.get("valid"):
        text = f"3D: ({p3d['x_mm']}, {p3d['y_mm']}, {p3d['z_mm']}) mm"
        depth_text = f"D: {p3d['depth_mm']}mm"
        samples = p3d.get("samples")
        suffix = f" S:{samples}" if samples else ""
        info = f"{text}  {depth_text}{suffix}"
        (tw, th), _ = cv2.getTextSize(info, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        cv2.rectangle(frame, (x1, y2), (x1 + tw + 6, y2 + th + 6), color, -1)
        cv2.putText(frame, info, (x1 + 3, y2 + th + 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    else:
        status = p3d.get("status", "no depth") if p3d else "no depth"
        cv2.putText(frame, status, (x1, y2 + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

    return frame


def draw_click_info(frame: np.ndarray, depth: np.ndarray | None,
                    fx: float, fy: float, cx: float, cy: float):
    """在鼠标点击位置显示 3D 坐标（点击后显示 2 秒）。"""
    if _click_point is None:
        return frame
    u, v = _click_point
    color = (255, 255, 0)
    cv2.circle(frame, (u, v), 5, color, 2)
    cv2.line(frame, (u - 15, v), (u + 15, v), color, 1)
    cv2.line(frame, (u, v - 15), (u, v + 15), color, 1)

    if depth is not None and fx > 0:
        from .depth_utils import get_depth_at, pixel_to_3d
        d = get_depth_at(depth, u, v)
        if d:
            x3d, y3d, z3d = pixel_to_3d(u, v, d, fx, fy, cx, cy)
            info = f"Click: ({x3d:.0f}, {y3d:.0f}, {z3d:.0f}) mm | D: {d:.0f}mm"
            # 半透明背景
            (tw, th), _ = cv2.getTextSize(info, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            bx1, by1 = u + 10, v - th - 10
            bx2, by2 = u + 10 + tw + 10, v + 10
            overlay = frame.copy()
            cv2.rectangle(overlay, (bx1, by1), (bx2, by2), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
            cv2.putText(frame, info, (u + 15, v),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        else:
            cv2.putText(frame, "no depth", (u + 15, v),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    return frame


def draw_crosshair(frame: np.ndarray, depth: np.ndarray | None,
                   fx: float, fy: float, cx: float, cy: float):
    """绘制屏幕中心十字准星 + 实时 3D 坐标。"""
    h, w = frame.shape[:2]
    center_x, center_y = w // 2, h // 2

    # 十字线
    color = (0, 255, 255)
    cv2.line(frame, (center_x - 30, center_y), (center_x - 10, center_y), color, 1)
    cv2.line(frame, (center_x + 10, center_y), (center_x + 30, center_y), color, 1)
    cv2.line(frame, (center_x, center_y - 30), (center_x, center_y - 10), color, 1)
    cv2.line(frame, (center_x, center_y + 10), (center_x, center_y + 30), color, 1)
    cv2.circle(frame, (center_x, center_y), 3, color, 1)

    # 显示 3D 坐标
    if depth is not None and fx > 0:
        from .depth_utils import get_depth_at, pixel_to_3d
        d = get_depth_at(depth, center_x, center_y)
        if d:
            x3d, y3d, z3d = pixel_to_3d(center_x, center_y, d, fx, fy, cx, cy)
            info = f"Center 3D: ({x3d:.0f}, {y3d:.0f}, {z3d:.0f}) mm | D: {d:.0f}mm"
            cv2.putText(frame, info, (center_x + 35, center_y + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        else:
            cv2.putText(frame, "Center: no depth", (center_x + 35, center_y + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

    return frame


def _class_color(class_id: int) -> tuple:
    """根据 class_id 生成固定颜色。"""
    np.random.seed(class_id)
    return tuple(int(c) for c in np.random.randint(0, 255, 3))
