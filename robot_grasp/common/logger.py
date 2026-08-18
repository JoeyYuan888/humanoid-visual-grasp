"""
数据记录器 — 将检测数据和鼠标点击数据保存到 CSV 文件，用于事后校验。
"""

import csv
import os
from datetime import datetime


def _float_or_blank(value):
    if value in ("", None):
        return ""
    try:
        return float(value)
    except (TypeError, ValueError):
        return ""


def _round_or_blank(value, digits: int = 1):
    number = _float_or_blank(value)
    if number == "":
        return ""
    return round(number, digits)


class DataLogger:
    """记录检测结果和鼠标点击的 3D 坐标数据。"""

    def __init__(self):
        self.detections: list[dict] = []
        self.clicks: list[dict] = []
        self.perf_samples: list[dict] = []

    def log_detection(self, frame_idx: int, det: dict, p3d: dict | None):
        """记录一次 YOLO 检测结果。"""
        record = {
            "frame": frame_idx,
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "type": "detection",
            "label": det["label"],
            "confidence": round(det["confidence"], 3),
            "bbox_x1": det["bbox"][0],
            "bbox_y1": det["bbox"][1],
            "bbox_x2": det["bbox"][2],
            "bbox_y2": det["bbox"][3],
            "center_u": det["center"][0],
            "center_v": det["center"][1],
            "depth_mm": round(p3d["depth_mm"], 1) if p3d and p3d.get("valid") else "",
            "x_mm": round(p3d["x_mm"], 1) if p3d and p3d.get("valid") else "",
            "y_mm": round(p3d["y_mm"], 1) if p3d and p3d.get("valid") else "",
            "z_mm": round(p3d["z_mm"], 1) if p3d and p3d.get("valid") else "",
        }
        self.detections.append(record)

    def log_click(self, u: int, v: int, depth_mm: float | None,
                  x_mm: float | None, y_mm: float | None, z_mm: float | None):
        """记录一次鼠标点击测距。"""
        record = {
            "frame": "",
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "type": "click",
            "label": "mouse_click",
            "confidence": "",
            "bbox_x1": "",
            "bbox_y1": "",
            "bbox_x2": "",
            "bbox_y2": "",
            "center_u": u,
            "center_v": v,
            "depth_mm": round(depth_mm, 1) if depth_mm is not None else "",
            "x_mm": round(x_mm, 1) if x_mm is not None else "",
            "y_mm": round(y_mm, 1) if y_mm is not None else "",
            "z_mm": round(z_mm, 1) if z_mm is not None else "",
        }
        self.clicks.append(record)

    def log_perf(self, frame_idx: int, sample: dict):
        """记录一次运行性能采样。"""
        record = {
            "frame": frame_idx,
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "type": "perf",
            "label": "runtime",
            "confidence": "",
            "bbox_x1": "",
            "bbox_y1": "",
            "bbox_x2": "",
            "bbox_y2": "",
            "center_u": "",
            "center_v": "",
            "depth_mm": "",
            "x_mm": "",
            "y_mm": "",
            "z_mm": "",
            "display_fps": _round_or_blank(sample.get("display_fps", 0.0), 2),
            "rgb_rx_fps": _round_or_blank(sample.get("rgb_rx_fps", 0.0), 2),
            "depth_msg_rx_fps": _round_or_blank(sample.get("depth_msg_rx_fps", 0.0), 2),
            "depth_rx_fps": _round_or_blank(sample.get("depth_rx_fps", 0.0), 2),
            "detect_fps": _round_or_blank(sample.get("detect_fps", 0.0), 2),
            "infer_ms": _round_or_blank(sample.get("infer_ms", 0.0), 1),
            "last_infer_ms": _round_or_blank(sample.get("last_infer_ms", 0.0), 1),
            "det_count": sample.get("det_count", 0),
            "depth_age_ms": _round_or_blank(sample.get("depth_age_ms", 0.0), 1),
            "grasp_status": sample.get("grasp_status", ""),
            "stable_frames": sample.get("stable_frames", ""),
            "grasp_sample_count": sample.get("grasp_sample_count", ""),
            "raw_depth_valid": sample.get("raw_depth_valid", ""),
            "roi_valid_count": sample.get("roi_valid_count", ""),
            "roi_total_count": sample.get("roi_total_count", ""),
            "primary_conf": _round_or_blank(sample.get("primary_conf", ""), 3),
            "raw_center_u": sample.get("raw_center_u", ""),
            "raw_center_v": sample.get("raw_center_v", ""),
            "smooth_center_u": sample.get("smooth_center_u", ""),
            "smooth_center_v": sample.get("smooth_center_v", ""),
            "grasp_x_mm": _round_or_blank(sample.get("grasp_x_mm", ""), 1),
            "grasp_y_mm": _round_or_blank(sample.get("grasp_y_mm", ""), 1),
            "grasp_z_mm": _round_or_blank(sample.get("grasp_z_mm", ""), 1),
            "grasp_depth_mm": _round_or_blank(sample.get("grasp_depth_mm", ""), 1),
            "qr_texts": sample.get("qr_texts", ""),
            "qr_live_count": sample.get("qr_live_count", ""),
            "qr_memory_count": sample.get("qr_memory_count", ""),
            "qr_decode_ms": _round_or_blank(sample.get("qr_decode_ms", 0.0), 1),
            "qr_source": sample.get("qr_source", ""),
            "qr_scan_mode": sample.get("qr_scan_mode", ""),
            "qr_scan_rois": sample.get("qr_scan_rois", ""),
            "qr_busy": sample.get("qr_busy", ""),
            "raw_rgb_age_ms": _round_or_blank(sample.get("raw_rgb_age_ms", 0.0), 1),
            "det_summary": sample.get("det_summary", ""),
            "object_results": sample.get("object_results", ""),
        }
        self.perf_samples.append(record)

    @property
    def total_detections(self) -> int:
        return len(self.detections)

    @property
    def total_clicks(self) -> int:
        return len(self.clicks)

    @property
    def total_perf_samples(self) -> int:
        return len(self.perf_samples)

    def save(self, directory: str = ".") -> str:
        """将所有记录保存到 CSV 文件，返回文件路径。"""
        os.makedirs(directory, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(directory, f"grasp_data_{timestamp}.csv")

        all_records = self.detections + self.clicks + self.perf_samples
        all_records.sort(key=lambda r: r["timestamp"])

        fieldnames = [
            "frame", "timestamp", "type", "label", "confidence",
            "bbox_x1", "bbox_y1", "bbox_x2", "bbox_y2",
            "center_u", "center_v",
            "depth_mm", "x_mm", "y_mm", "z_mm",
            "display_fps", "rgb_rx_fps", "depth_msg_rx_fps", "depth_rx_fps", "detect_fps",
            "infer_ms", "last_infer_ms", "det_count", "depth_age_ms",
            "grasp_status", "stable_frames", "grasp_sample_count", "raw_depth_valid",
            "roi_valid_count", "roi_total_count", "primary_conf",
            "raw_center_u", "raw_center_v", "smooth_center_u", "smooth_center_v",
            "grasp_x_mm", "grasp_y_mm", "grasp_z_mm", "grasp_depth_mm",
            "qr_texts", "qr_live_count", "qr_memory_count", "qr_decode_ms",
            "qr_source", "qr_scan_mode", "qr_scan_rois", "qr_busy",
            "raw_rgb_age_ms", "det_summary", "object_results",
        ]

        with open(filepath, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(all_records)

        print(f"[✓] 数据已保存: {filepath}")
        print(f"    - 检测记录: {len(self.detections)} 条")
        print(f"    - 点击记录: {len(self.clicks)} 条")
        print(f"    - 性能采样: {len(self.perf_samples)} 条")
        self._print_perf_summary()
        return filepath

    def _print_perf_summary(self):
        if not self.perf_samples:
            return

        def avg(key: str) -> float:
            vals = [float(r[key]) for r in self.perf_samples if r.get(key) != ""]
            return sum(vals) / len(vals) if vals else 0.0

        print("    - 性能均值:")
        print(f"      display_fps={avg('display_fps'):.1f}, rgb_rx_fps={avg('rgb_rx_fps'):.1f}, "
              f"depth_msg_rx_fps={avg('depth_msg_rx_fps'):.1f}, depth_rx_fps={avg('depth_rx_fps'):.1f}, "
              f"detect_fps={avg('detect_fps'):.1f}")
        print(f"      infer_ms={avg('infer_ms'):.0f}, depth_age_ms={avg('depth_age_ms'):.0f}")
