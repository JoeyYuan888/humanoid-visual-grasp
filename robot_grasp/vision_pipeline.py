"""
视觉感知管线。

输入一帧 RGB + 深度 + camera_info，输出每个物体的:
- label/conf/bbox/center
- 3D 坐标
- 已锁存 QR 文本

这个模块不连接 ROS、不处理按键、不保存 CSV，方便被主程序和 debug 脚本复用。
"""

from __future__ import annotations

import json
import time

import cv2
import numpy as np

from . import config
from .depth_utils import compute_grasp_point
from .detector import Detector
from .qr_detector import draw_qr_detections
from .qr_worker import QRWorker
from .visualizer import draw_detection, draw_grasp_info, draw_overlay


def center_distance(a, b) -> float:
    if a is None or b is None:
        return float("inf")
    return float(((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5)


def median_grasp_point(samples: list[dict]):
    if not samples:
        return {"valid": False, "status": "sampling"}
    keys = ["x_mm", "y_mm", "z_mm", "depth_mm"]
    p3d = {key: round(float(np.median([sample[key] for sample in samples])), 1) for key in keys}
    if samples[-1].get("point_u") is not None:
        p3d["point_u"] = int(round(float(np.median([sample["point_u"] for sample in samples]))))
    if samples[-1].get("point_v") is not None:
        p3d["point_v"] = int(round(float(np.median([sample["point_v"] for sample in samples]))))
    if samples[-1].get("depth_roi") is not None:
        p3d["depth_roi"] = samples[-1]["depth_roi"]
    p3d["valid"] = True
    p3d["samples"] = len(samples)
    return p3d


def smooth_bbox(prev_bbox, new_bbox):
    if prev_bbox is None:
        return tuple(float(v) for v in new_bbox)
    alpha = config.BBOX_SMOOTH_ALPHA
    return tuple((1.0 - alpha) * prev + alpha * new for prev, new in zip(prev_bbox, new_bbox))


def with_smoothed_bbox(det: dict, smoothed_bbox):
    x1, y1, x2, y2 = [int(round(v)) for v in smoothed_bbox]
    smoothed = dict(det)
    smoothed["bbox"] = (x1, y1, x2, y2)
    smoothed["center"] = ((x1 + x2) // 2, (y1 + y2) // 2)
    return smoothed


def scale_rois(rois, src_shape, dst_shape):
    src_h, src_w = src_shape[:2]
    dst_h, dst_w = dst_shape[:2]
    if src_w == 0 or src_h == 0:
        return rois
    sx = dst_w / src_w
    sy = dst_h / src_h
    return [
        (int(x1 * sx), int(y1 * sy), int(x2 * sx), int(y2 * sy))
        for x1, y1, x2, y2 in rois
    ]


def scale_qr_detections(detections: list[dict], src_shape, dst_shape):
    src_h, src_w = src_shape[:2]
    dst_h, dst_w = dst_shape[:2]
    if src_w == dst_w and src_h == dst_h:
        return detections
    if src_w == 0 or src_h == 0:
        return detections

    sx = dst_w / src_w
    sy = dst_h / src_h
    scaled = []
    for det in detections:
        item = dict(det)
        pts = item["points"].copy()
        pts[:, 0] *= sx
        pts[:, 1] *= sy
        center = pts.mean(axis=0)
        item["points"] = pts
        item["center"] = (int(center[0]), int(center[1]))
        scaled.append(item)
    return scaled


def merge_qr_memory(memory: dict, detections: list[dict], now: float):
    for det in detections:
        memory[det["text"]] = {
            "det": dict(det),
            "last_seen": now,
        }

    stale = [
        text
        for text, item in memory.items()
        if now - item["last_seen"] > config.QR_MEMORY_TTL_SEC
    ]
    for text in stale:
        del memory[text]

    return [item["det"] for item in memory.values()]


def qr_texts(detections: list[dict]) -> str:
    return ";".join(det["text"] for det in detections)


def det_summary(detections: list[dict]) -> str:
    return ";".join(f"{det['label']}:{det['confidence']:.2f}" for det in detections)


def bbox_contains(bbox, point) -> bool:
    x1, y1, x2, y2 = bbox
    x, y = point
    return x1 <= x <= x2 and y1 <= y <= y2


def qr_text_for_detection(det_index: int, det: dict, qr_detections: list[dict],
                          qr_memory_detections: list[dict]) -> str:
    for qr_det in qr_detections:
        if qr_det.get("roi_index") == det_index:
            return qr_det["text"]

    for qr_det in qr_memory_detections:
        center = qr_det.get("center")
        if center and bbox_contains(det["bbox"], center):
            return qr_det["text"]
    return ""


def object_results_summary(objects: list[dict]) -> str:
    compact = []
    for obj in objects:
        compact.append({
            "idx": obj["idx"],
            "label": obj["label"],
            "conf": round(obj["confidence"], 3),
            "qr": obj.get("qr_text", ""),
            "valid": bool(obj.get("valid")),
            "x_mm": obj.get("x_mm", ""),
            "y_mm": obj.get("y_mm", ""),
            "z_mm": obj.get("z_mm", ""),
            "status": obj.get("status", ""),
        })
    return json.dumps(compact, ensure_ascii=False, separators=(",", ":"))


def qr_priority(det: dict) -> tuple[int, float]:
    try:
        class_rank = config.QR_PRIORITY_CLASSES.index(det["label"])
    except ValueError:
        class_rank = len(config.QR_PRIORITY_CLASSES)
    return class_rank, -det["confidence"]


class VisionPipeline:
    """可复用视觉管线：YOLO、深度坐标、QR 锁存、标注图。"""

    def __init__(self, enable_qr: bool | None = None, detector: Detector | None = None):
        self.detector = detector if detector is not None else Detector()
        self.enable_qr = config.ENABLE_QR if enable_qr is None else enable_qr
        self.qr_worker = QRWorker() if self.enable_qr else None

        self.detect_interval = max(1, int(config.DETECT_EVERY_N_FRAMES))
        self.last_detect_frame = -1
        self.last_detections = []
        self.infer_times = []
        self.avg_infer = 0.0
        self.last_infer_ms = 0.0

        self.smoothed_bbox = None
        self.stable_frames = 0
        self.lost_frames = 0
        self.last_primary_det = None
        self.grasp_samples = []
        self.last_sample_depth_time = 0.0

        self.qr_memory = {}
        self.last_qr_decode_time = 0.0
        self.last_qr_full_scan_time = 0.0
        self.qr_detections = []
        self.qr_memory_detections = []
        self.qr_busy = False
        self.qr_stats = {
            "decode_ms": 0.0,
            "source": "",
            "mode": "",
            "roi_count": 0,
            "result_count": 0,
            "error": "",
        }
        self.seen_qr_texts = set()

    def backend_info(self) -> str:
        if self.qr_worker is None:
            return ""
        return self.qr_worker.backend_info()

    def stop(self):
        if self.qr_worker is not None:
            self.qr_worker.stop()

    def process(self, rgb, depth, cam_info, frame_count: int, client_stats: dict,
                raw_rgb=None, raw_rgb_updated_at: float = 0.0, fps: float = 0.0) -> dict:
        now = time.time()
        detections, should_detect = self._detect(rgb, frame_count)
        detections = [
            det for det in sorted(detections, key=lambda det: det["confidence"], reverse=True)
            if det["confidence"] >= config.OBJECT_MIN_CONF
        ]

        fx = fy = cx = cy = 0
        if cam_info and "K" in cam_info:
            K = cam_info["K"]
            fx, fy, cx, cy = K[0], K[4], K[2], K[5]

        depth_updated_at = client_stats.get("depth_updated_at", 0.0)
        depth_age_ms = float("inf")
        if depth_updated_at > 0:
            depth_age_ms = max(0.0, (now - depth_updated_at) * 1000)

        qr_new_texts = self._update_qr(detections, rgb, raw_rgb, raw_rgb_updated_at, now)
        primary_det = self._update_primary_tracking(detections)

        if primary_det is None and self.last_primary_det is not None and self.lost_frames <= config.TARGET_LOST_GRACE_FRAMES:
            primary_det = self.last_primary_det
            detections = [primary_det]

        annotated = rgb.copy()
        object_results, debug = self._build_object_results(
            detections=detections,
            primary_det=primary_det,
            depth=depth,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            depth_age_ms=depth_age_ms,
            depth_updated_at=depth_updated_at,
            annotated=annotated,
        )

        debug.update({
            "qr_texts": qr_texts(self.qr_memory_detections),
            "qr_live_count": len(self.qr_detections),
            "qr_memory_count": len(self.qr_memory_detections),
            "qr_decode_ms": self.qr_stats.get("decode_ms", 0.0),
            "qr_source": self.qr_stats.get("source", ""),
            "qr_scan_mode": self.qr_stats.get("mode", ""),
            "qr_scan_rois": self.qr_stats.get("roi_count", ""),
            "qr_busy": int(self.qr_busy),
            "det_summary": det_summary(detections),
            "object_results": object_results_summary(object_results),
        })

        if self.enable_qr:
            annotated = draw_qr_detections(annotated, self.qr_memory_detections)

        annotated = draw_overlay(annotated, fps, len(detections), self.avg_infer)
        cv2.putText(
            annotated,
            f"QR: {len(self.qr_detections)}/{len(self.qr_memory_detections)} "
            f"{qr_texts(self.qr_memory_detections)}",
            (12, 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            1,
        )

        return {
            "annotated": annotated,
            "detections": detections,
            "object_results": object_results,
            "debug": debug,
            "should_detect": should_detect,
            "avg_infer_ms": self.avg_infer,
            "last_infer_ms": self.last_infer_ms,
            "qr_new_texts": qr_new_texts,
        }

    def _detect(self, rgb, frame_count: int):
        should_detect = self.last_detect_frame < 0 or (frame_count - self.last_detect_frame) >= self.detect_interval
        if should_detect:
            t0 = time.time()
            detections, _ = self.detector.detect(rgb)
            self.last_infer_ms = (time.time() - t0) * 1000
            self.infer_times.append(self.last_infer_ms)
            if len(self.infer_times) > 30:
                self.infer_times.pop(0)
            self.avg_infer = sum(self.infer_times) / len(self.infer_times)
            self.last_detections = detections
            self.last_detect_frame = frame_count
            return detections, True
        return self.last_detections, False

    def _update_qr(self, detections, rgb, raw_rgb, raw_rgb_updated_at: float, now: float):
        new_texts = []
        self.qr_detections = []
        self.qr_memory_detections = merge_qr_memory(self.qr_memory, [], now)

        if self.qr_worker is None or not detections:
            self.qr_memory_detections = merge_qr_memory(self.qr_memory, [], now)
            self.qr_busy = False
            return new_texts

        expected_qr_count = config.QR_EXPECTED_COUNT or len(detections)
        qr_locked = len(self.qr_memory_detections) >= expected_qr_count
        full_scan_interval = (
            config.QR_LOCKED_FULL_RESCAN_INTERVAL_SEC
            if qr_locked else config.QR_ACTIVE_FULL_RESCAN_INTERVAL_SEC
        )
        full_scan_due = now - self.last_qr_full_scan_time >= full_scan_interval
        if full_scan_due:
            scan_pairs = sorted(enumerate(detections), key=lambda item: qr_priority(item[1]))
            qr_scan_mode = "full_locked" if qr_locked else "full_active"
        else:
            scan_pairs = sorted([
                (idx, det)
                for idx, det in enumerate(detections)
                if not qr_text_for_detection(idx, det, [], self.qr_memory_detections)
            ], key=lambda item: qr_priority(item[1]))
            qr_scan_mode = "missing"

        scan_pairs = scan_pairs[:max(1, int(config.QR_MAX_ROIS_PER_SCAN))]
        qr_frame = rgb
        qr_source = "compressed"
        qr_frame_shape = rgb.shape
        raw_age_ms = float("inf")
        if raw_rgb_updated_at > 0:
            raw_age_ms = max(0.0, (now - raw_rgb_updated_at) * 1000)
        if raw_rgb is not None and raw_age_ms <= config.QR_MAX_RAW_AGE_MS:
            qr_frame = raw_rgb
            qr_frame_shape = raw_rgb.shape
            qr_source = "raw"

        qr_rois = [det["bbox"] for _, det in scan_pairs]
        if qr_source == "raw":
            qr_rois = scale_rois(qr_rois, rgb.shape, raw_rgb.shape)
        roi_indices = [idx for idx, _ in scan_pairs]

        if qr_rois and now - self.last_qr_decode_time >= config.QR_DECODE_INTERVAL_SEC:
            qr_submit = self.qr_worker.submit(
                qr_frame,
                qr_rois,
                qr_source,
                roi_indices=roi_indices,
                mode=qr_scan_mode,
            )
            if qr_submit:
                self.last_qr_decode_time = now
                if qr_scan_mode.startswith("full"):
                    self.last_qr_full_scan_time = now

        self.qr_detections, self.qr_busy, self.qr_stats = self.qr_worker.latest()
        qr_result_shape = self.qr_stats.get("shape") or qr_frame_shape
        self.qr_detections = scale_qr_detections(self.qr_detections, qr_result_shape, rgb.shape)
        self.qr_memory_detections = merge_qr_memory(self.qr_memory, self.qr_detections, now)

        for qr_det in self.qr_detections:
            text = qr_det["text"]
            if text not in self.seen_qr_texts:
                self.seen_qr_texts.add(text)
                new_texts.append(qr_det)
        return new_texts

    def _update_primary_tracking(self, detections):
        primary_det = detections[0] if detections else None
        if primary_det is None:
            self.lost_frames += 1
            if self.lost_frames > config.TARGET_LOST_GRACE_FRAMES:
                self.smoothed_bbox = None
                self.stable_frames = 0
                self.last_primary_det = None
                self.grasp_samples.clear()
            return primary_det

        if self.smoothed_bbox is None:
            self.lost_frames = 0
            self.smoothed_bbox = smooth_bbox(None, primary_det["bbox"])
            self.stable_frames = 1
            self.last_primary_det = primary_det
            self.grasp_samples.clear()
            return primary_det

        smoothed_center = with_smoothed_bbox(primary_det, self.smoothed_bbox)["center"]
        if center_distance(primary_det["center"], smoothed_center) <= config.TARGET_STABLE_PIXEL_THRESH:
            self.lost_frames = 0
            self.stable_frames += 1
            self.smoothed_bbox = smooth_bbox(self.smoothed_bbox, primary_det["bbox"])
            self.last_primary_det = primary_det
        else:
            self.lost_frames = 0
            self.smoothed_bbox = smooth_bbox(None, primary_det["bbox"])
            self.stable_frames = 1
            self.last_primary_det = primary_det
            self.grasp_samples.clear()
        return primary_det

    def _build_object_results(self, detections, primary_det, depth, fx, fy, cx, cy,
                              depth_age_ms, depth_updated_at, annotated):
        object_results = []
        debug = {
            "grasp_status": "no target",
            "depth_age_ms": round(depth_age_ms, 1) if depth_age_ms != float("inf") else "",
            "stable_frames": self.stable_frames,
            "grasp_sample_count": len(self.grasp_samples),
            "raw_depth_valid": "",
            "roi_valid_count": "",
            "roi_total_count": "",
            "primary_conf": primary_det["confidence"] if primary_det else "",
            "raw_center_u": primary_det["center"][0] if primary_det else "",
            "raw_center_v": primary_det["center"][1] if primary_det else "",
            "smooth_center_u": "",
            "smooth_center_v": "",
            "grasp_x_mm": "",
            "grasp_y_mm": "",
            "grasp_z_mm": "",
            "grasp_depth_mm": "",
        }

        for i, det in enumerate(detections):
            p3d = {"valid": False, "status": "tracking"}
            is_primary = primary_det is not None and det is primary_det
            det_for_depth = with_smoothed_bbox(det, self.smoothed_bbox) if is_primary and self.smoothed_bbox is not None else det
            if is_primary:
                debug["smooth_center_u"] = det_for_depth["center"][0]
                debug["smooth_center_v"] = det_for_depth["center"][1]

            if is_primary and self.stable_frames < config.TARGET_STABLE_FRAMES:
                p3d = {"valid": False, "status": f"stabilizing {self.stable_frames}/{config.TARGET_STABLE_FRAMES}"}
            elif not config.ENABLE_DEPTH or depth is None or fx <= 0:
                if is_primary and len(self.grasp_samples) >= config.STATIC_GRASP_SAMPLES:
                    p3d = median_grasp_point(self.grasp_samples)
                else:
                    p3d = {"valid": False, "status": "no depth"}
            elif depth_age_ms > config.DEPTH_MAX_AGE_MS:
                if is_primary and len(self.grasp_samples) >= config.STATIC_GRASP_SAMPLES:
                    p3d = median_grasp_point(self.grasp_samples)
                else:
                    p3d = {"valid": False, "status": f"stale depth {depth_age_ms:.0f}ms"}
            else:
                raw_p3d = compute_grasp_point(det_for_depth, depth, fx, fy, cx, cy)
                if is_primary:
                    debug["raw_depth_valid"] = bool(raw_p3d.get("valid"))
                    debug["roi_valid_count"] = raw_p3d.get("roi_valid_count", "")
                    debug["roi_total_count"] = raw_p3d.get("roi_total_count", "")
                if is_primary and raw_p3d.get("valid") and depth_updated_at != self.last_sample_depth_time:
                    self.grasp_samples.append(raw_p3d)
                    if len(self.grasp_samples) > config.STATIC_GRASP_SAMPLES:
                        self.grasp_samples.pop(0)
                    self.last_sample_depth_time = depth_updated_at

                if is_primary and len(self.grasp_samples) >= config.STATIC_GRASP_SAMPLES:
                    p3d = median_grasp_point(self.grasp_samples)
                elif raw_p3d.get("valid"):
                    p3d = raw_p3d
                    if not is_primary:
                        p3d = dict(p3d)
                        p3d["status"] = "secondary valid"
                else:
                    status = raw_p3d.get("status", f"sampling {len(self.grasp_samples)}/{config.STATIC_GRASP_SAMPLES}")
                    p3d = {"valid": False, "status": status}

            if is_primary:
                debug["grasp_status"] = "valid" if p3d.get("valid") else p3d.get("status", "")
                debug["stable_frames"] = self.stable_frames
                debug["grasp_sample_count"] = len(self.grasp_samples)
                if p3d.get("valid"):
                    debug["grasp_x_mm"] = p3d.get("x_mm", "")
                    debug["grasp_y_mm"] = p3d.get("y_mm", "")
                    debug["grasp_z_mm"] = p3d.get("z_mm", "")
                    debug["grasp_depth_mm"] = p3d.get("depth_mm", "")

            qr_text = qr_text_for_detection(i, det, self.qr_detections, self.qr_memory_detections)
            object_results.append({
                "idx": i + 1,
                "label": det["label"],
                "confidence": det["confidence"],
                "bbox": det["bbox"],
                "center": det["center"],
                "qr_text": qr_text,
                "valid": bool(p3d.get("valid")),
                "status": p3d.get("status", "valid" if p3d.get("valid") else ""),
                "x_mm": p3d.get("x_mm", ""),
                "y_mm": p3d.get("y_mm", ""),
                "z_mm": p3d.get("z_mm", ""),
                "depth_mm": p3d.get("depth_mm", ""),
                "point_u": p3d.get("point_u", ""),
                "point_v": p3d.get("point_v", ""),
                "depth_roi": p3d.get("depth_roi", ""),
            })

            draw_detection(annotated, det_for_depth, i + 1)
            if config.ENABLE_DEPTH:
                draw_grasp_info(annotated, det_for_depth, p3d)
            if qr_text:
                x1, y1, _, _ = det_for_depth["bbox"]
                cv2.putText(annotated, f"QR: {qr_text}", (x1, max(18, y1 - 22)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        return object_results, debug
