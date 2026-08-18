#!/usr/bin/env python3
"""
Post-grasp QR scan from the head RealSense RGB image.

This script is intentionally independent from YOLO/depth. After the robot has
picked up the plastic bag and moved it near the head camera, scan the full RGB
frame for QR codes and save the latest result.
"""

from __future__ import annotations

import argparse
import base64
import builtins
import contextlib
import json
import os
import re
import subprocess
import sys
import threading
import time
from datetime import datetime

import cv2
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot_grasp.common import config
from robot_grasp.vision.camera_guard import ensure_realsense_ready_on_client
from robot_grasp.vision.detector import Detector
from robot_grasp.vision.qr_detector import QRDetector, draw_qr_detections
from robot_grasp.vision.ocr_detector import (
    OCRDetector,
    draw_ocr_detections,
    _WARD_LIST,
    _find_qr_finders,
    _ocr_correct,
    _ward_canonical,
)

try:
    import roslibpy
except ImportError:
    print("缺少 roslibpy 库，请运行: pip install roslibpy")
    sys.exit(1)


DEFAULT_OUTPUT = os.path.join(PROJECT_ROOT, "data", "runtime", "post_grasp_qr_latest.json")
DEFAULT_SNAPSHOT = os.path.join(PROJECT_ROOT, "data", "runtime", "post_grasp_qr_snapshot.png")
DEFAULT_ROI_SNAPSHOT = os.path.join(PROJECT_ROOT, "data", "runtime", "post_grasp_qr_roi.png")
DEFAULT_RAW_FRAME_DIR = os.path.join(PROJECT_ROOT, "data", "qr_multiframe_debug", "latest_raw", "raw")
DEFAULT_RECOVER_DIR = os.path.join(PROJECT_ROOT, "data", "qr_multiframe_debug", "latest_recover")
DEFAULT_PADDLE_FRAME_DIR = os.path.join(PROJECT_ROOT, "data", "qr_multiframe_debug", "latest_paddle")
_ORIGINAL_PRINT = builtins.print


def _enable_quiet_print():
    keep_patterns = (
        "[✗]",
        "[!]",
        "[动作]",
        "[QR]",
        "[OCR]",
        "已保存 QR",
        "已保存 OCR",
        "未识别到 QR",
        "OCR 未通过",
        "OCR 快照",
        "PP-OCRv4 快照",
        "QR 快照",
        "小模型 OCR 未通过",
        "PP-OCRv4 fallback",
        "轻量 QR fallback",
    )

    def quiet_print(*args, **kwargs):
        text = " ".join(str(arg) for arg in args)
        if any(pattern in text for pattern in keep_patterns):
            kwargs.setdefault("flush", True)
            _ORIGINAL_PRINT(*args, **kwargs)

    builtins.print = quiet_print


def _parse_ws_url(ws_url: str) -> tuple[str, int]:
    stripped = ws_url.replace("ws://", "").replace("wss://", "")
    host, port = stripped.split(":")
    return host, int(port)


class RGBSubscriber:
    def __init__(self):
        self.frame = None
        self.seq = 0
        self.lock = threading.Lock()

    def compressed_callback(self, message):
        data_b64 = message.get("data")
        if not data_b64:
            return
        try:
            img_bytes = base64.b64decode(data_b64)
            img_array = np.frombuffer(img_bytes, dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        except Exception:
            return
        if frame is None:
            return
        with self.lock:
            self.frame = frame
            self.seq += 1

    def raw_callback(self, message):
        try:
            height = int(message.get("height", 0))
            width = int(message.get("width", 0))
            encoding = message.get("encoding", "")
            data_b64 = message.get("data")
            if not data_b64 or height <= 0 or width <= 0:
                return

            raw = base64.b64decode(data_b64)
            arr = np.frombuffer(raw, dtype=np.uint8)

            if encoding in ("rgb8", "bgr8"):
                frame = arr.reshape((height, width, 3))
                if encoding == "rgb8":
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif encoding in ("mono8", "8UC1"):
                gray = arr.reshape((height, width))
                frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            else:
                return
        except Exception:
            return

        with self.lock:
            self.frame = frame
            self.seq += 1

    def latest(self):
        with self.lock:
            if self.frame is None:
                return None, self.seq
            return self.frame.copy(), self.seq


class CameraInfoSubscriber:
    def __init__(self):
        self.camera_info = None
        self.lock = threading.Lock()

    def callback(self, message):
        with self.lock:
            if self.camera_info is None:
                self.camera_info = dict(message)

    def latest(self):
        with self.lock:
            return dict(self.camera_info) if self.camera_info is not None else None


class Undistorter:
    def __init__(self):
        self._cache_key = None
        self._map1 = None
        self._map2 = None

    @staticmethod
    def _matrix_from_camera_info(camera_info: dict) -> tuple[np.ndarray, np.ndarray] | None:
        k = camera_info.get("K") or []
        d = camera_info.get("D") or []
        if len(k) != 9 or len(d) < 4:
            return None
        camera_matrix = np.asarray(k, dtype=np.float64).reshape(3, 3)
        dist_coeffs = np.asarray(d, dtype=np.float64).reshape(-1, 1)
        return camera_matrix, dist_coeffs

    def apply(self, frame: np.ndarray, camera_info: dict | None) -> tuple[np.ndarray, bool]:
        if camera_info is None:
            return frame, False
        matrices = self._matrix_from_camera_info(camera_info)
        if matrices is None:
            return frame, False

        h, w = frame.shape[:2]
        camera_matrix, dist_coeffs = matrices
        key = (
            w,
            h,
            tuple(np.round(camera_matrix.reshape(-1), 8)),
            tuple(np.round(dist_coeffs.reshape(-1), 8)),
        )
        if key != self._cache_key:
            new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
                camera_matrix,
                dist_coeffs,
                (w, h),
                alpha=0,
                newImgSize=(w, h),
            )
            self._map1, self._map2 = cv2.initUndistortRectifyMap(
                camera_matrix,
                dist_coeffs,
                None,
                new_camera_matrix,
                (w, h),
                cv2.CV_16SC2,
            )
            self._cache_key = key

        if self._map1 is None or self._map2 is None:
            return frame, False
        return cv2.remap(frame, self._map1, self._map2, interpolation=cv2.INTER_LINEAR), True


def _connect(ws_url: str):
    host, port = _parse_ws_url(ws_url)
    client = roslibpy.Ros(host=host, port=port)
    thread = threading.Thread(target=client.run, daemon=True)
    thread.start()

    start = time.time()
    while not client.is_connected:
        if time.time() - start > config.CONNECT_TIMEOUT:
            raise RuntimeError(f"连接超时: {ws_url}")
        time.sleep(0.1)
    return client


def _wait_for_frame(
    client,
    topic_name: str,
    msg_type: str,
    callback,
    subscriber: RGBSubscriber,
    timeout: float,
    throttle_ms: int = 0,
) -> tuple[np.ndarray | None, int]:
    topic = roslibpy.Topic(
        client,
        topic_name,
        msg_type,
        throttle_rate=throttle_ms,
        queue_length=1,
    )
    topic.subscribe(callback)
    start = time.time()
    try:
        while time.time() - start < timeout:
            frame, seq = subscriber.latest()
            if frame is not None:
                return frame, seq
            time.sleep(0.02)
    finally:
        try:
            topic.unsubscribe()
        except Exception:
            pass
    return None, 0


def _save_result(output: str, detections: list[dict], frame_seq: int) -> None:
    payload = {
        "created_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "mode": "qr",
        "frame_seq": frame_seq,
        "count": len(detections),
        "texts": [det.get("text", "") for det in detections],
        "detections": [
            {
                "text": det.get("text", ""),
                "backend": det.get("backend", ""),
                "source": det.get("source", ""),
                "center": list(det.get("center", [])),
                "points": np.asarray(det.get("points", [])).astype(float).tolist(),
            }
            for det in detections
        ],
    }
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)


def _save_ocr_result(output: str, detections: list[dict], frame_seq: int, mode: str = "ocr_small") -> None:
    payload = {
        "created_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "mode": mode,
        "frame_seq": frame_seq,
        "count": len(detections),
        "texts": [det.get("text", "") for det in detections],
        "detections": [
            {
                "text": det.get("text", ""),
                "raw_text": det.get("raw_text", det.get("text", "")),
                "confidence": float(det.get("confidence", 0.0) or 0.0),
                "backend": det.get("backend", ""),
                "bbox": list(det.get("bbox", [])),
                "roi_index": det.get("roi_index"),
                "source": det.get("source", ""),
                "vote_count": det.get("vote_count"),
                "raw_texts": list(det.get("raw_texts", [])) if det.get("raw_texts") else [],
            }
            for det in detections
        ],
    }
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)


def _prepare_raw_frame_dir(path: str | None) -> None:
    if not path:
        return
    os.makedirs(path, exist_ok=True)
    for name in os.listdir(path):
        if name.startswith("raw_") and name.lower().endswith((".png", ".jpg", ".jpeg")):
            try:
                os.unlink(os.path.join(path, name))
            except OSError:
                pass


def _save_raw_snapshot(path: str | None, index: int, frame: np.ndarray | None) -> str | None:
    if not path or frame is None:
        return None
    os.makedirs(path, exist_ok=True)
    filename = os.path.join(path, f"raw_{index:03d}.png")
    cv2.imwrite(filename, frame)
    return filename


def _run_offline_recover(args) -> bool:
    if not args.auto_recover_offline or not args.save_raw_frames:
        return False
    cmd = [
        sys.executable,
        "tools/debug/debug_qr_multiframe_recover.py",
        "--input-dir",
        args.save_raw_frames,
        "--output-dir",
        args.recover_output_dir,
        "--count",
        str(max(1, args.snapshot_attempts)),
        "--fast",
        "--no-wechat",
        "--min-consensus",
        str(max(1, args.recover_min_consensus)),
        "--result-output",
        args.output,
    ]
    print("[动作] 在线未命中，启动离线多帧恢复")
    result = subprocess.run(
        cmd,
        cwd=PROJECT_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    for line in result.stdout.splitlines():
        stripped = line.strip()
        if (
            stripped.startswith("[OK]")
            or stripped.startswith("accepted:")
            or stripped.startswith("frames:")
            or stripped.startswith("normalized:")
            or stripped.startswith("report:")
            or stripped.startswith("result:")
        ):
            print(f"    {stripped}")
    if result.returncode == 0:
        print("[动作] 离线多帧恢复成功")
        return True
    print("[动作] 离线多帧恢复未命中")
    return False


def _expand_roi(bbox: tuple[int, int, int, int], width: int, height: int, margin_ratio: float) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = bbox
    bw = max(1, x2 - x1)
    bh = max(1, y2 - y1)
    mx = int(bw * margin_ratio)
    my = int(bh * margin_ratio)
    return (
        max(0, x1 - mx),
        max(0, y1 - my),
        min(width, x2 + mx),
        min(height, y2 + my),
    )


def _select_yolo_rois(
    detector: Detector,
    frame: np.ndarray,
    max_rois: int,
    margin_ratio: float,
) -> tuple[list[tuple[int, int, int, int]], list[dict]]:
    dets, _ = detector.detect(frame)
    if not dets:
        return [], []

    h, w = frame.shape[:2]
    selected = sorted(dets, key=lambda item: item.get("confidence", 0.0), reverse=True)[:max_rois]
    rois = [_expand_roi(tuple(det["bbox"]), w, h, margin_ratio) for det in selected]
    return rois, selected


def _filter_ocr_detections(args, detections: list[dict]) -> list[dict]:
    accepted = []
    pattern = re.compile(args.ocr_required_regex) if args.ocr_required_regex else None
    ward_pattern = re.compile(r"^\s*\d{2,4}.*病区")
    ward_like_tokens = ("病区", "化疗", "化区", "肺科")

    def normalize_ward(text: str) -> str | None:
        canonical = _ward_canonical(text)
        if canonical is not None:
            return canonical
        if ward_pattern.search(text):
            corrected = _ocr_correct(text)
            canonical = _ward_canonical(corrected)
            return canonical if canonical is not None else text
        if not any(token in text for token in ward_like_tokens):
            return None
        import difflib

        best_ward = None
        best_ratio = 0.0
        for ward in _WARD_LIST:
            ratio = difflib.SequenceMatcher(None, text, ward).ratio()
            if ratio > best_ratio:
                best_ratio = ratio
                best_ward = ward
        if best_ward is not None and best_ratio >= args.ocr_ward_fuzzy_ratio:
            return best_ward
        return None

    for det in detections:
        raw_text = str(det.get("text", "")).strip()
        if not raw_text:
            continue
        text = _ocr_correct(raw_text).strip()
        compact = text.replace(" ", "")
        confidence = float(det.get("confidence", 0.0) or 0.0)
        if args.ocr_target == "ward":
            if confidence < args.ocr_ward_min_conf:
                continue
            ward_text = normalize_ward(text)
            if ward_text is None:
                continue
            text = ward_text
            compact = text.replace(" ", "")
        else:
            if confidence < args.ocr_min_conf:
                continue
            if len(compact) < args.ocr_min_text_len:
                continue
        if pattern is not None and pattern.search(text) is None:
            continue
        item = dict(det)
        item["raw_text"] = raw_text
        item["text"] = text
        item["confidence"] = confidence
        accepted.append(item)
    return accepted


def _filter_with_min_results(args, detections: list[dict], min_results: int) -> list[dict]:
    accepted = _filter_ocr_detections(args, detections)
    return accepted if len(accepted) >= min_results else []


def _extract_paddle_ocr_detections(result_items) -> list[dict]:
    detections = []
    for result in result_items or []:
        if not isinstance(result, dict) and hasattr(result, "json"):
            try:
                result = result.json
            except Exception:
                pass
        if not isinstance(result, dict):
            continue
        if "res" in result and isinstance(result["res"], dict):
            result = result["res"]
        texts = result.get("rec_texts", [])
        scores = result.get("rec_scores", [])
        boxes = result.get("rec_boxes", [])
        polys = result.get("rec_polys", [])
        if texts is None:
            texts = []
        if scores is None:
            scores = []
        if boxes is None:
            boxes = []
        if polys is None:
            polys = []
        for index, text in enumerate(texts):
            score = float(scores[index]) if index < len(scores) else 0.0
            bbox = []
            if index < len(boxes):
                arr = np.asarray(boxes[index]).astype(float)
                if arr.size == 4:
                    bbox = arr.reshape(-1).tolist()
                elif arr.size >= 4:
                    pts = arr.reshape(-1, 2)
                    bbox = [
                        float(np.min(pts[:, 0])),
                        float(np.min(pts[:, 1])),
                        float(np.max(pts[:, 0])),
                        float(np.max(pts[:, 1])),
                    ]
            elif index < len(polys):
                pts = np.asarray(polys[index]).astype(float).reshape(-1, 2)
                bbox = [
                    float(np.min(pts[:, 0])),
                    float(np.min(pts[:, 1])),
                    float(np.max(pts[:, 0])),
                    float(np.max(pts[:, 1])),
                ]
            if len(bbox) == 4:
                x1, y1, x2, y2 = bbox
                bbox = [int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2))]
            else:
                bbox = [0, 0, 1, 1]
            detections.append({
                "text": str(text),
                "raw_text": str(text),
                "confidence": score,
                "backend": "paddleocr_ppocrv4",
                "source": "paddleocr",
                "bbox": bbox,
                "roi_index": None,
            })
    return detections


def _load_paddle_ocr(args):
    cache_home = os.path.join(PROJECT_ROOT, ".cache", "paddle_home")
    os.makedirs(cache_home, exist_ok=True)
    os.environ.setdefault("HOME", cache_home)
    os.environ.setdefault("PADDLE_HOME", os.path.join(cache_home, "paddle"))
    os.environ.setdefault("PADDLE_PDX_DISABLE_MODEL_SOURCE_CHECK", "True")
    os.environ.setdefault("FLAGS_use_mkldnn", "false")
    from paddleocr import PaddleOCR

    return PaddleOCR(
        ocr_version=args.paddle_ocr_version,
        lang=args.paddle_ocr_lang,
        use_doc_orientation_classify=False,
        use_doc_unwarping=False,
        use_textline_orientation=False,
        enable_mkldnn=False,
    )


@contextlib.contextmanager
def _suppress_paddle_output(enabled: bool):
    if not enabled:
        yield
        return
    with open(os.devnull, "w", encoding="utf-8") as devnull:
        with contextlib.redirect_stdout(devnull), contextlib.redirect_stderr(devnull):
            yield


def _run_paddle_ocr_fallback(
    args,
    snapshots,
    undistorter,
    cam_info,
) -> tuple[list[dict], int, np.ndarray | None]:
    if not args.paddle_ocr_fallback:
        return [], 0, None
    print("[动作] PP-OCRv4 fallback 加载开始")
    try:
        with _suppress_paddle_output(not args.paddle_ocr_verbose):
            paddle_ocr = _load_paddle_ocr(args)
    except Exception as exc:
        print(f"[!] PP-OCRv4 初始化失败，继续 QR fallback: {exc}")
        return [], 0, None
    print("[动作] PP-OCRv4 fallback 加载完成")

    os.makedirs(args.paddle_ocr_frame_dir, exist_ok=True)
    for name in os.listdir(args.paddle_ocr_frame_dir):
        if name.startswith("paddle_") and name.lower().endswith((".png", ".jpg", ".jpeg")):
            try:
                os.unlink(os.path.join(args.paddle_ocr_frame_dir, name))
            except OSError:
                pass

    votes: dict[str, dict] = {}
    best_single: dict | None = None
    for index, (preview_frame, preview_seq, raw_frame, raw_seq) in enumerate(snapshots, start=1):
        decode_source = raw_frame if raw_frame is not None else preview_frame
        if decode_source is None:
            continue
        decode_frame = decode_source
        if args.undistort:
            decode_frame, _ = undistorter.apply(decode_source, cam_info)
        frame_path = os.path.join(args.paddle_ocr_frame_dir, f"paddle_{index:03d}.png")
        cv2.imwrite(frame_path, decode_frame)
        try:
            with _suppress_paddle_output(not args.paddle_ocr_verbose):
                result = paddle_ocr.predict(frame_path)
        except Exception as exc:
            print(f"[动作] PP-OCRv4 快照 {index}/{len(snapshots)} 推理失败: {exc}")
            continue
        raw_detections = _extract_paddle_ocr_detections(result)
        accepted = _filter_with_min_results(args, raw_detections, args.paddle_ocr_min_results)
        print(
            f"[动作] PP-OCRv4 快照 {index}/{len(snapshots)} "
            f"raw={len(raw_detections)} accepted={len(accepted)}"
        )
        for det in accepted:
            text = str(det.get("text", "")).strip()
            if not text:
                continue
            frame_seq = raw_seq if raw_frame is not None else preview_seq
            record = votes.setdefault(text, {
                "count": 0,
                "detections": [],
                "frame_seq": frame_seq,
                "frame": decode_frame,
                "score_sum": 0.0,
                "raw_texts": [],
            })
            record["count"] += 1
            record["detections"].append(det)
            record["score_sum"] += float(det.get("confidence", 0.0) or 0.0)
            record["raw_texts"].append(str(det.get("raw_text", text)))
            if best_single is None or float(det.get("confidence", 0.0) or 0.0) > best_single["confidence"]:
                best_single = {
                    "text": text,
                    "confidence": float(det.get("confidence", 0.0) or 0.0),
                    "raw_text": str(det.get("raw_text", text)),
                }

    if not votes:
        return [], 0, None

    best_text, best_record = max(
        votes.items(),
        key=lambda item: (item[1]["count"], item[1]["score_sum"]),
    )
    required_votes = max(1, args.paddle_ocr_min_votes)
    if best_record["count"] < required_votes:
        if best_single:
            print(
                "[动作] PP-OCRv4 投票不足 "
                f"best={best_single['text']} votes={best_record['count']}/{required_votes}, "
                f"raw={best_single['raw_text']}"
            )
        return [], 0, None

    chosen = dict(best_record["detections"][0])
    chosen["confidence"] = best_record["score_sum"] / max(1, best_record["count"])
    chosen["vote_count"] = best_record["count"]
    chosen["raw_texts"] = best_record["raw_texts"]
    print(f"[OCR] {best_text} votes={best_record['count']}/{len(snapshots)}")
    return [chosen], int(best_record["frame_seq"]), best_record["frame"]


def _draw_yolo_rois(frame: np.ndarray, rois: list[tuple[int, int, int, int]], dets: list[dict]) -> None:
    for index, roi in enumerate(rois):
        x1, y1, x2, y2 = roi
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 180, 0), 2)
        label = "YOLO ROI"
        if index < len(dets):
            det = dets[index]
            label = f"{det.get('label', 'object')} {det.get('confidence', 0.0):.2f}"
        cv2.putText(
            frame,
            label,
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 180, 0),
            2,
        )


def _scale_rois(
    rois: list[tuple[int, int, int, int]],
    source_shape: tuple[int, int],
    target_shape: tuple[int, int],
) -> list[tuple[int, int, int, int]]:
    if not rois:
        return []
    src_h, src_w = source_shape
    dst_h, dst_w = target_shape
    if src_h <= 0 or src_w <= 0 or dst_h <= 0 or dst_w <= 0:
        return []
    sx = dst_w / src_w
    sy = dst_h / src_h
    scaled = []
    for x1, y1, x2, y2 in rois:
        scaled.append((
            max(0, min(dst_w, int(round(x1 * sx)))),
            max(0, min(dst_h, int(round(y1 * sy)))),
            max(0, min(dst_w, int(round(x2 * sx)))),
            max(0, min(dst_h, int(round(y2 * sy)))),
        ))
    return scaled


def _expand_bbox(
    bbox: tuple[int, int, int, int],
    width: int,
    height: int,
    mx_ratio: float,
    my_ratio: float,
) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = bbox
    bw = max(1, x2 - x1)
    bh = max(1, y2 - y1)
    mx = int(bw * mx_ratio)
    my = int(bh * my_ratio)
    return (
        max(0, x1 - mx),
        max(0, y1 - my),
        min(width, x2 + mx),
        min(height, y2 + my),
    )


def _clip_bbox(
    bbox: tuple[int, int, int, int],
    width: int,
    height: int,
) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = bbox
    return max(0, x1), max(0, y1), min(width, x2), min(height, y2)


def _find_label_rois(
    frame: np.ndarray,
    search_rois: list[tuple[int, int, int, int]],
    max_rois: int,
    expand_x: float,
    expand_y: float,
) -> list[tuple[int, int, int, int]]:
    """Find small medicine-label ROIs inside plastic-bag ROIs.

    The YOLO ROI is the whole bag. OCR is much more stable if we crop the
    actual printed label first.
    """
    h, w = frame.shape[:2]
    gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found = []

    qr_finders = _find_qr_finders(frame, upscale=3.0)
    if len(qr_finders) >= 3:
        xs = [p[0] for p in qr_finders]
        ys = [p[1] for p in qr_finders]
        qx1, qx2 = min(xs), max(xs)
        qy1, qy2 = min(ys), max(ys)
        qr_w = max(30, qx2 - qx1)
        qr_h = max(30, qy2 - qy1)
        # The hospital sticker places the QR at the lower-left area. Ward text
        # is above/right of it, while the dosage sticker has no QR. Anchor OCR
        # to this sticker first when two stickers are visible.
        roi = (
            int(qx1 - 2.4 * qr_w),
            int(qy1 - 7.0 * qr_h),
            int(qx2 + 4.8 * qr_w),
            int(qy2 + 4.8 * qr_h),
        )
        roi = _clip_bbox(roi, w, h)
        for search_roi in search_rois:
            sx1, sy1, sx2, sy2 = search_roi
            if not (roi[2] < sx1 or roi[0] > sx2 or roi[3] < sy1 or roi[1] > sy2):
                found.append((1e9, roi))
                break

    for sx1, sy1, sx2, sy2 in search_rois:
        sx1, sy1, sx2, sy2 = max(0, sx1), max(0, sy1), min(w, sx2), min(h, sy2)
        if sx2 <= sx1 or sy2 <= sy1:
            continue
        gray = gray_full[sy1:sy2, sx1:sx2]
        dark = cv2.inRange(gray, 0, 135)
        contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        small = np.zeros_like(dark)
        for cnt in contours:
            x, y, cw, ch = cv2.boundingRect(cnt)
            area = cv2.contourArea(cnt)
            if 4 <= area <= 2500 and 2 <= cw <= 90 and 2 <= ch <= 90:
                cv2.drawContours(small, [cnt], -1, 255, -1)
        grouped = cv2.dilate(
            small,
            cv2.getStructuringElement(cv2.MORPH_RECT, (45, 35)),
            iterations=1,
        )
        grouped = cv2.morphologyEx(
            grouped,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_RECT, (31, 31)),
            iterations=1,
        )
        contours, _ = cv2.findContours(grouped, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            x, y, cw, ch = cv2.boundingRect(cnt)
            if cw < 70 or ch < 50:
                continue
            if cw > (sx2 - sx1) * 0.55 or ch > (sy2 - sy1) * 0.55:
                continue
            aspect = cw / max(ch, 1)
            if not (0.45 <= aspect <= 2.3):
                continue
            lx1, ly1, lx2, ly2 = _expand_bbox(
                (sx1 + x, sy1 + y, sx1 + x + cw, sy1 + y + ch),
                w,
                h,
                expand_x,
                expand_y,
            )
            roi_dark = small[y:y + ch, x:x + cw]
            dark_count = int(cv2.countNonZero(roi_dark))
            if dark_count < 80:
                continue
            mean_v = float(np.mean(gray_full[ly1:ly2, lx1:lx2]))
            bg_bonus = max(0.0, min(1.0, (mean_v - 80.0) / 150.0))
            score = dark_count * (0.7 + bg_bonus)
            found.append((score, (lx1, ly1, lx2, ly2)))
    found.sort(key=lambda item: item[0], reverse=True)
    unique = []
    for _, roi in found:
        if roi not in unique:
            unique.append(roi)
        if len(unique) >= max_rois:
            break
    return unique


def _ocr_detect_label_crops(
    ocr_detector: OCRDetector,
    frame: np.ndarray,
    rois: list[tuple[int, int, int, int]],
    upscale: float,
    use_variants: bool = True,
) -> list[dict]:
    results = []
    h, w = frame.shape[:2]

    def rotate_bound(image: np.ndarray, angle: float) -> np.ndarray:
        ih, iw = image.shape[:2]
        center = (iw / 2.0, ih / 2.0)
        matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
        cos = abs(matrix[0, 0])
        sin = abs(matrix[0, 1])
        new_w = int(ih * sin + iw * cos)
        new_h = int(ih * cos + iw * sin)
        matrix[0, 2] += new_w / 2.0 - center[0]
        matrix[1, 2] += new_h / 2.0 - center[1]
        return cv2.warpAffine(
            image,
            matrix,
            (new_w, new_h),
            flags=cv2.INTER_CUBIC,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(255, 255, 255),
        )

    for roi_index, (x1, y1, x2, y2) in enumerate(rois):
        x1, y1, x2, y2 = max(0, x1), max(0, y1), min(w, x2), min(h, y2)
        if x2 <= x1 or y2 <= y1:
            continue
        crop = frame[y1:y2, x1:x2]
        if crop.size == 0:
            continue
        variants = [("orig", crop)]
        if use_variants:
            variants.extend([
                ("rot_-15", rotate_bound(crop, -15)),
                ("rot_+15", rotate_bound(crop, 15)),
            ])
        for variant_name, variant in variants:
            crop_results = ocr_detector.detect(
                variant,
                rois=[(0, 0, variant.shape[1], variant.shape[0])],
                upscale=upscale,
            )
            for item in crop_results:
                adjusted = dict(item)
                bx1, by1, bx2, by2 = adjusted.get("bbox", (0, 0, crop.shape[1], crop.shape[0]))
                adjusted["bbox"] = (x1 + int(bx1), y1 + int(by1), x1 + int(bx2), y1 + int(by2))
                adjusted["roi_index"] = roi_index
                adjusted["source_variant"] = variant_name
                results.append(adjusted)
    return results


def _run_snapshot_scan(args, qr_detector: QRDetector) -> None:
    print("[动作] 连接 rosbridge 开始")
    client = _connect(args.ws_url)
    ensure_realsense_ready_on_client(
        client,
        require_depth=False,
        require_camera_info=args.undistort,
        require_raw_rgb=args.transport == "raw",
    )
    raw_client = None
    if args.transport == "raw":
        raw_client = _connect(args.ws_url)
    print("[动作] 连接 rosbridge 完成")

    info_subscriber = CameraInfoSubscriber()
    info_topic = None
    if args.undistort:
        info_topic = roslibpy.Topic(
            client,
            args.camera_info_topic,
            "sensor_msgs/CameraInfo",
            throttle_rate=1000,
            queue_length=1,
        )
        info_topic.subscribe(info_subscriber.callback)

    os.makedirs(os.path.dirname(args.snapshot), exist_ok=True)
    _prepare_raw_frame_dir(args.save_raw_frames)
    snapshots = []

    cam_info = None
    if args.undistort:
        deadline = time.time() + 1.5
        cam_info = info_subscriber.latest()
        while cam_info is None and time.time() < deadline:
            time.sleep(0.05)
            cam_info = info_subscriber.latest()

    detections = []
    ocr_detections = []
    decode_frame = None
    decode_rois = []
    latest_yolo_dets = []
    frame_seq = 0
    qr_debug_start = time.time()
    undistorter = Undistorter()
    warned_no_camera_info = False

    def qr_progress(message: str):
        if args.qr_debug:
            print(f"    [QR-debug {time.time() - qr_debug_start:5.1f}s] {message}", flush=True)

    print(f"[动作] 批量获取 QR 快照开始 count={args.snapshot_attempts}")
    for attempt in range(max(1, args.snapshot_attempts)):
        preview_subscriber = RGBSubscriber()
        preview_frame, preview_seq = _wait_for_frame(
            client,
            config.TOPIC_RGB,
            "sensor_msgs/CompressedImage",
            preview_subscriber.compressed_callback,
            preview_subscriber,
            timeout=args.duration,
            throttle_ms=args.compressed_throttle_ms,
        )
        if preview_frame is None:
            if not snapshots:
                _save_result(args.output, [], 0)
                print("[!] 未收到 compressed RGB 快照")
                return
            break

        raw_seq = 0
        raw_frame = None
        if args.transport == "raw":
            raw_subscriber = RGBSubscriber()
            raw_frame, raw_seq = _wait_for_frame(
                raw_client,
                config.TOPIC_RGB_RAW,
                "sensor_msgs/Image",
                raw_subscriber.raw_callback,
                raw_subscriber,
                timeout=args.duration,
                throttle_ms=args.raw_throttle_ms,
            )
            if raw_frame is None:
                if not snapshots:
                    _save_result(args.output, [], preview_seq)
                    print("[!] 未收到 raw RGB 快照，未执行 QR 解码")
                    return
                break

        snapshots.append((preview_frame, preview_seq, raw_frame, raw_seq))
        _save_raw_snapshot(
            args.save_raw_frames,
            len(snapshots),
            raw_frame if raw_frame is not None else preview_frame,
        )
        print(f"[动作] QR 快照 {attempt + 1}/{args.snapshot_attempts} 获取完成")
        if attempt + 1 < args.snapshot_attempts:
            time.sleep(max(0.0, args.snapshot_interval))

    print(f"[动作] 批量获取 QR 快照完成 got={len(snapshots)}")
    if args.save_raw_frames:
        print(f"[动作] 已保存 QR raw 快照目录: {args.save_raw_frames}")

    if not snapshots:
        _save_result(args.output, [], 0)
        print("[!] 未获取到可用于识别的快照")
        return

    yolo_detector = None
    if args.yolo_roi:
        print("[动作] YOLO ROI 检测器加载开始")
        yolo_detector = Detector(conf=config.YOLO_CONF, imgsz=config.YOLO_IMGSZ)
        print("[动作] YOLO ROI 检测器加载完成")

    ocr_detector = None
    if args.prefer_ocr:
        print("[动作] OCR 检测器加载开始")
        ocr_detector = OCRDetector(conf_threshold=args.ocr_detector_conf)
        if ocr_detector._rec_session is None:
            print("[!] OCR 初始化失败，将直接使用轻量 QR")
            ocr_detector = None
        else:
            print("[动作] OCR 检测器加载完成")

    for index, (preview_frame, preview_seq, raw_frame, raw_seq) in enumerate(snapshots, start=1):
        decode_source = raw_frame if raw_frame is not None else preview_frame
        yolo_source = preview_frame
        decode_frame = decode_source
        yolo_frame = yolo_source
        if args.undistort:
            if cam_info is None:
                cam_info = info_subscriber.latest()
            decode_frame, undistorted = undistorter.apply(decode_source, cam_info)
            yolo_frame, _ = undistorter.apply(yolo_source, cam_info)
            if not undistorted and not warned_no_camera_info:
                warned_no_camera_info = True
                print("[!] 未拿到有效 camera_info，实际未进行畸变矫正")

        decode_rois = []
        latest_yolo_dets = []
        if yolo_detector is not None:
            latest_rois, latest_yolo_dets = _select_yolo_rois(
                yolo_detector,
                yolo_frame,
                max(1, args.max_yolo_rois),
                max(0.0, args.yolo_roi_margin),
            )
            decode_rois = _scale_rois(latest_rois, yolo_frame.shape[:2], decode_frame.shape[:2])
            print(f"[动作] 识别快照 {index}/{len(snapshots)} YOLO rois={len(decode_rois)}")

        if decode_rois:
            x1, y1, x2, y2 = decode_rois[0]
            roi_img = decode_frame[y1:y2, x1:x2]
            if roi_img.size > 0:
                cv2.imwrite(args.roi_snapshot, roi_img)

        frame_seq = raw_seq if raw_frame is not None else preview_seq
        cv2.imwrite(args.snapshot, decode_frame)

        if ocr_detector is not None:
            ocr_rois = decode_rois or [(0, 0, decode_frame.shape[1], decode_frame.shape[0])]
            label_rois = []
            if args.ocr_label_crop:
                label_rois = _find_label_rois(
                    decode_frame,
                    ocr_rois,
                    max(1, args.ocr_max_label_rois),
                    max(0.0, args.ocr_label_expand_x),
                    max(0.0, args.ocr_label_expand_y),
                )
                if label_rois:
                    ocr_rois = label_rois
                    lx1, ly1, lx2, ly2 = label_rois[0]
                    label_img = decode_frame[ly1:ly2, lx1:lx2]
                    if label_img.size > 0:
                        cv2.imwrite(args.roi_snapshot, label_img)
            if label_rois:
                raw_ocr = _ocr_detect_label_crops(
                    ocr_detector,
                    decode_frame,
                    label_rois,
                    args.ocr_upscale,
                    args.ocr_label_variants,
                )
            else:
                raw_ocr = ocr_detector.detect(decode_frame, rois=ocr_rois, upscale=args.ocr_upscale)
            ocr_detections = _filter_with_min_results(args, raw_ocr, args.ocr_min_results)
            print(
                f"[动作] OCR 快照 {index}/{len(snapshots)} "
                f"label_rois={len(label_rois)} raw={len(raw_ocr)} accepted={len(ocr_detections)}"
            )
            if ocr_detections:
                texts = "; ".join(det["text"] for det in ocr_detections)
                print(f"[OCR] {texts}")
                break

    if not ocr_detections:
        print("[动作] 小模型 OCR 未通过，开始 PP-OCRv4 fallback")
        paddle_detections, paddle_frame_seq, paddle_frame = _run_paddle_ocr_fallback(
            args,
            snapshots,
            undistorter,
            cam_info,
        )
        if paddle_detections:
            ocr_detections = paddle_detections
            frame_seq = paddle_frame_seq
            if paddle_frame is not None:
                decode_frame = paddle_frame
    if not ocr_detections:
        print("[动作] OCR 未通过，开始轻量 QR fallback")
        for index, (preview_frame, preview_seq, raw_frame, raw_seq) in enumerate(snapshots, start=1):
            decode_source = raw_frame if raw_frame is not None else preview_frame
            yolo_source = preview_frame
            decode_frame = decode_source
            yolo_frame = yolo_source
            if args.undistort:
                if cam_info is None:
                    cam_info = info_subscriber.latest()
                decode_frame, _ = undistorter.apply(decode_source, cam_info)
                yolo_frame, _ = undistorter.apply(yolo_source, cam_info)
            decode_rois = []
            latest_yolo_dets = []
            if yolo_detector is not None:
                latest_rois, latest_yolo_dets = _select_yolo_rois(
                    yolo_detector,
                    yolo_frame,
                    max(1, args.max_yolo_rois),
                    max(0.0, args.yolo_roi_margin),
                )
                decode_rois = _scale_rois(latest_rois, yolo_frame.shape[:2], decode_frame.shape[:2])
            frame_seq = raw_seq if raw_frame is not None else preview_seq
            detections = qr_detector.detect(
                decode_frame,
                rois=decode_rois,
                heavy=args.qr_fallback_heavy,
                search_full=not decode_rois,
                progress=qr_progress,
            )
            if detections:
                texts = "; ".join(det["text"] for det in detections)
                print(f"[QR] {texts}")
                break
            print(f"[动作] QR 快照 {index}/{len(snapshots)} 未命中")

    if ocr_detections:
        backend = str(ocr_detections[0].get("backend", ""))
        mode = "ocr_paddle_v4" if backend.startswith("paddleocr") else "ocr_small"
        _save_ocr_result(args.output, ocr_detections, frame_seq, mode=mode)
    else:
        if not detections:
            print("[!] OCR 未通过，且未识别到 QR")
        _save_result(args.output, detections, frame_seq)
    recovered = False
    if not ocr_detections and not detections:
        recovered = _run_offline_recover(args)
    if decode_frame is None:
        decode_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    annotated = draw_ocr_detections(decode_frame, ocr_detections) if ocr_detections else draw_qr_detections(decode_frame, detections)
    _draw_yolo_rois(annotated, decode_rois, latest_yolo_dets)
    if args.save_snapshot:
        cv2.imwrite(args.snapshot, annotated)
        print(f"[✓] 已保存截图: {args.snapshot}")
    if ocr_detections:
        print(f"[✓] 已保存 OCR 结果: {args.output}")
    elif detections or recovered:
        print(f"[✓] 已保存 QR 结果: {args.output}")
    print("[动作] 抓后识别完成")

    if args.show_window:
        cv2.imshow("Post-grasp QR Snapshot", annotated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    if info_topic is not None:
        try:
            info_topic.unsubscribe()
        except Exception:
            pass
    try:
        client.terminate()
    except Exception:
        pass
    if raw_client is not None:
        try:
            raw_client.terminate()
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws-url", default=config.WS_URL)
    parser.add_argument("--transport", choices=["raw", "compressed"], default="raw",
                        help="Use raw Image by default for QR quality; compressed is fallback.")
    parser.add_argument("--rgb-topic", default=None)
    parser.add_argument("--camera-info-topic", default=config.TOPIC_CAMERA_INFO)
    parser.add_argument("--raw-throttle-ms", type=int, default=config.QR_RAW_RGB_THROTTLE_MS,
                        help="rosbridge throttle for raw Image transport. Default follows config.QR_RAW_RGB_THROTTLE_MS.")
    parser.add_argument("--compressed-throttle-ms", type=int, default=0,
                        help="rosbridge throttle for compressed transport.")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--decode-interval", type=float, default=0.4)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--show-window", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--save-snapshot", action="store_true")
    parser.add_argument("--snapshot", default=DEFAULT_SNAPSHOT)
    parser.add_argument("--roi-snapshot", default=DEFAULT_ROI_SNAPSHOT)
    parser.add_argument(
        "--save-raw-frames",
        default=DEFAULT_RAW_FRAME_DIR,
        help="Directory for raw QR snapshot frames. Use empty string to disable.",
    )
    parser.add_argument("--auto-recover-offline", action=argparse.BooleanOptionalAction, default=False,
                        help="If online QR scan fails, run multi-frame raw offline recovery automatically.")
    parser.add_argument("--recover-output-dir", default=DEFAULT_RECOVER_DIR)
    parser.add_argument("--recover-min-consensus", type=int, default=2)
    parser.add_argument("--snapshot-attempts", type=int, default=5,
                        help="Default snapshot mode retries with fresh frames before declaring QR failure.")
    parser.add_argument("--snapshot-interval", type=float, default=1.0,
                        help="Seconds between snapshot QR retries.")
    parser.add_argument("--qr-debug", action="store_true",
                        help="Print detailed QR decoder progress for each snapshot.")
    parser.add_argument("--live", action="store_true", help="Use old live loop instead of default single-snapshot mode.")
    parser.add_argument("--undistort", action=argparse.BooleanOptionalAction, default=True,
                        help="Use color camera_info to undistort RGB before QR decoding.")
    parser.add_argument("--yolo-roi", action=argparse.BooleanOptionalAction, default=True,
                        help="Run low-rate YOLO and scan QR only inside plastic-bag ROI when available.")
    parser.add_argument("--yolo-interval", type=float, default=2.0,
                        help="Seconds between YOLO ROI updates during static QR scanning.")
    parser.add_argument("--yolo-roi-margin", type=float, default=0.0,
                        help="Expand YOLO bbox before QR scanning.")
    parser.add_argument("--max-yolo-rois", type=int, default=1)
    parser.add_argument("--prefer-ocr", action=argparse.BooleanOptionalAction, default=True,
                        help="Try OCR first. If OCR does not pass validation, fall back to lightweight QR.")
    parser.add_argument("--ocr-target", choices=["ward", "any"], default="ward",
                        help="Default ward accepts only ward-name text, preventing dosage/usage labels from passing OCR.")
    parser.add_argument("--ocr-min-conf", type=float, default=0.60)
    parser.add_argument("--ocr-ward-min-conf", type=float, default=0.30,
                        help="Minimum confidence before ward-name normalization.")
    parser.add_argument("--ocr-ward-fuzzy-ratio", type=float, default=0.33,
                        help="Minimum similarity to known ward names for ward-only OCR.")
    parser.add_argument("--ocr-detector-conf", type=float, default=0.10,
                        help="Internal OCR detector threshold. Keep lower than ocr-min-conf to avoid dropping small text early.")
    parser.add_argument("--ocr-min-text-len", type=int, default=2)
    parser.add_argument("--ocr-min-results", type=int, default=2)
    parser.add_argument("--ocr-upscale", type=float, default=4.0)
    parser.add_argument("--ocr-label-crop", action=argparse.BooleanOptionalAction, default=True,
                        help="Locate the small printed label inside the plastic-bag ROI before OCR.")
    parser.add_argument("--ocr-label-variants", action=argparse.BooleanOptionalAction, default=True,
                        help="OCR original label crop plus small rotation variants.")
    parser.add_argument("--ocr-max-label-rois", type=int, default=3)
    parser.add_argument("--ocr-label-expand-x", type=float, default=0.35)
    parser.add_argument("--ocr-label-expand-y", type=float, default=0.50)
    parser.add_argument("--ocr-required-regex", default="",
                        help="Optional regex; OCR accepted only if at least one accepted line matches it.")
    parser.add_argument("--paddle-ocr-fallback", action=argparse.BooleanOptionalAction, default=True,
                        help="After small ONNX OCR fails, try PaddleOCR PP-OCRv4 before QR fallback.")
    parser.add_argument("--paddle-ocr-version", default="PP-OCRv4")
    parser.add_argument("--paddle-ocr-lang", default="ch")
    parser.add_argument("--paddle-ocr-min-results", type=int, default=1)
    parser.add_argument("--paddle-ocr-min-votes", type=int, default=2,
                        help="Minimum number of snapshots that must agree before accepting PaddleOCR result.")
    parser.add_argument("--paddle-ocr-frame-dir", default=DEFAULT_PADDLE_FRAME_DIR,
                        help="Directory for temporary frames passed to PaddleOCR.")
    parser.add_argument("--paddle-ocr-verbose", action="store_true",
                        help="Print PaddleOCR internal model logs. Default suppresses them.")
    parser.add_argument("--qr-fallback-heavy", action=argparse.BooleanOptionalAction, default=False,
                        help="Use heavy QR decoder after OCR fails. Default false keeps fallback lightweight.")
    parser.add_argument("--quiet", action="store_true", help="只打印 QR 结果和错误")
    args = parser.parse_args()

    if args.quiet:
        _enable_quiet_print()

    print("=" * 70)
    print("  Post-grasp OCR/QR scan")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    rgb_topic = args.rgb_topic
    if rgb_topic is None:
        rgb_topic = config.TOPIC_RGB_RAW if args.transport == "raw" else config.TOPIC_RGB
    msg_type = "sensor_msgs/Image" if args.transport == "raw" else "sensor_msgs/CompressedImage"
    throttle_ms = args.raw_throttle_ms if args.transport == "raw" else args.compressed_throttle_ms

    print(f"  RGB topic: {rgb_topic}")
    if args.transport == "raw":
        print(f"  Preview/Yolo topic: {config.TOPIC_RGB}")
    print(f"  CameraInfo topic: {args.camera_info_topic}")
    print(f"  Transport: {args.transport} ({msg_type})")
    print(f"  Throttle: {throttle_ms}ms")
    print(f"  Undistort: {args.undistort}")
    print(f"  YOLO ROI: {args.yolo_roi}")
    print(f"  Prefer OCR: {args.prefer_ocr}")
    print(f"  OCR target: {args.ocr_target}")
    print(f"  OCR label crop: {args.ocr_label_crop}")
    print(f"  OCR upscale: {args.ocr_upscale}")
    print(f"  PaddleOCR fallback: {args.paddle_ocr_fallback}")
    print(f"  QR fallback heavy: {args.qr_fallback_heavy}")
    print(f"  Duration: {args.duration:.1f}s")
    print(f"  Output: {args.output}")
    print(f"  Mode: {'live' if args.live else 'snapshot'}")
    print("=" * 70)

    detector = QRDetector()
    print(f"[*] QR backends: {', '.join(detector.backends)}")
    if not args.live:
        try:
            _run_snapshot_scan(args, detector)
        finally:
            if args.show_window:
                cv2.destroyAllWindows()
        return

    yolo_detector = None

    print("[动作] 连接 rosbridge 开始")
    client = _connect(args.ws_url)
    ensure_realsense_ready_on_client(
        client,
        require_depth=False,
        require_camera_info=args.undistort,
        require_raw_rgb=args.transport == "raw",
    )
    raw_client = None
    if args.transport == "raw":
        raw_client = _connect(args.ws_url)
    print("[动作] 连接 rosbridge 完成")

    subscriber = RGBSubscriber()
    preview_subscriber = None
    topic = None
    preview_topic = None
    if args.transport == "raw":
        topic = roslibpy.Topic(
            raw_client,
            rgb_topic,
            msg_type,
            throttle_rate=throttle_ms,
            queue_length=1,
        )
        topic.subscribe(subscriber.raw_callback)
        preview_subscriber = RGBSubscriber()
        preview_topic = roslibpy.Topic(
            client,
            config.TOPIC_RGB,
            "sensor_msgs/CompressedImage",
            throttle_rate=args.compressed_throttle_ms,
            queue_length=1,
        )
        preview_topic.subscribe(preview_subscriber.compressed_callback)
    else:
        topic = roslibpy.Topic(
            client,
            rgb_topic,
            msg_type,
            throttle_rate=throttle_ms,
            queue_length=1,
        )
        topic.subscribe(subscriber.compressed_callback)
    info_subscriber = CameraInfoSubscriber()
    info_topic = None
    if args.undistort:
        info_topic = roslibpy.Topic(
            client,
            args.camera_info_topic,
            "sensor_msgs/CameraInfo",
            throttle_rate=1000,
            queue_length=1,
        )
        info_topic.subscribe(info_subscriber.callback)
    print(f"[动作] QR 扫码开始 transport={args.transport}, duration={args.duration:.1f}s")

    latest_detections = []
    latest_seq = 0
    last_decode = 0.0
    start = time.time()
    fps_start = start
    frames = 0
    undistorter = Undistorter()
    undistort_seen = False
    latest_rois: list[tuple[int, int, int, int]] = []
    latest_decode_rois: list[tuple[int, int, int, int]] = []
    latest_yolo_dets: list[dict] = []
    last_yolo = 0.0
    last_wait_log = 0.0

    try:
        while time.time() - start < args.duration:
            raw_or_main_frame, seq = subscriber.latest()
            preview_frame = None
            preview_seq = 0
            if preview_subscriber is not None:
                preview_frame, preview_seq = preview_subscriber.latest()

            display_available = preview_frame is not None or raw_or_main_frame is not None
            if not display_available:
                now = time.time()
                if args.show_window:
                    status = np.zeros((360, 900, 3), dtype=np.uint8)
                    camera_info_state = "yes" if info_subscriber.latest() is not None else "no"
                    cv2.putText(
                        status,
                        "Waiting for RGB frames",
                        (24, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 255, 255),
                        2,
                    )
                    cv2.putText(
                        status,
                        f"raw topic: {rgb_topic}",
                        (24, 120),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (220, 220, 220),
                        2,
                    )
                    cv2.putText(
                        status,
                        f"preview topic: {config.TOPIC_RGB}",
                        (24, 165),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (220, 220, 220),
                        2,
                    )
                    cv2.putText(
                        status,
                        f"preview={'yes' if preview_frame is not None else 'no'} camera_info={camera_info_state}",
                        (24, 220),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.75,
                        (0, 200, 255),
                        2,
                    )
                    cv2.putText(
                        status,
                        "q=quit",
                        (24, 290),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.75,
                        (180, 180, 180),
                        2,
                    )
                    cv2.imshow("Post-grasp QR Scan", status)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        break
                if now - last_wait_log >= 1.0:
                    last_wait_log = now
                    camera_info_state = "yes" if info_subscriber.latest() is not None else "no"
                    preview_state = "yes" if preview_frame is not None else "no"
                    print(f"[动作] 等待 raw RGB 帧 preview={preview_state}, camera_info={camera_info_state}")
                time.sleep(0.02)
                continue

            frames += 1
            now = time.time()
            decode_source = raw_or_main_frame if raw_or_main_frame is not None else preview_frame
            yolo_source = preview_frame if preview_frame is not None else decode_source
            display_source = yolo_source
            decode_frame = decode_source
            yolo_frame = yolo_source
            display_frame = display_source
            undistorted = False
            if args.undistort:
                cam_info = info_subscriber.latest()
                decode_frame, undistorted = undistorter.apply(decode_source, cam_info)
                yolo_frame, _ = undistorter.apply(yolo_source, cam_info)
                display_frame = yolo_frame
                undistort_seen = undistort_seen or undistorted

            if args.show_window:
                annotated = display_frame.copy()
                _draw_yolo_rois(annotated, latest_rois, latest_yolo_dets)
                elapsed = max(1e-6, now - fps_start)
                fps = frames / elapsed
                cv2.putText(
                    annotated,
                    (
                        f"Preview compressed | raw={'yes' if raw_or_main_frame is not None else 'no'} "
                        f"| fps={fps:.1f} | undistort={undistorted} "
                        f"| yolo_roi={len(latest_rois)} | q=quit | QR={len(latest_detections)}"
                    ),
                    (16, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2,
                )
                cv2.imshow("Post-grasp QR Scan", annotated)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break

            if args.yolo_roi and yolo_detector is None:
                yolo_detector = Detector(conf=config.YOLO_CONF, imgsz=config.YOLO_IMGSZ)

            if yolo_detector is not None and (now - last_yolo >= args.yolo_interval or not latest_rois):
                last_yolo = now
                latest_rois, latest_yolo_dets = _select_yolo_rois(
                    yolo_detector,
                    yolo_frame,
                    max(1, args.max_yolo_rois),
                    max(0.0, args.yolo_roi_margin),
                )
                latest_decode_rois = _scale_rois(
                    latest_rois,
                    yolo_frame.shape[:2],
                    decode_frame.shape[:2],
                )

            if now - last_decode >= args.decode_interval or not latest_detections:
                last_decode = now
                latest_detections = detector.detect(
                    decode_frame,
                    rois=latest_decode_rois,
                    heavy=True,
                    search_full=not latest_decode_rois,
                )
                latest_seq = seq if raw_or_main_frame is not None else preview_seq
                if latest_detections:
                    texts = "; ".join(det["text"] for det in latest_detections)
                    print(f"[QR] {texts}")
                    _save_result(args.output, latest_detections, latest_seq)
                    if args.save_snapshot:
                        snap = draw_qr_detections(decode_frame, latest_detections)
                        snap_path = os.path.join(
                            PROJECT_ROOT,
                            "data",
                            f"post_grasp_qr_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png",
                        )
                        cv2.imwrite(snap_path, snap)
                        print(f"[✓] 已保存截图: {snap_path}")
                    break

            if not args.show_window:
                time.sleep(0.02)

        if not latest_detections:
            _save_result(args.output, [], latest_seq)
            print("[!] 未识别到 QR")
        else:
            print(f"[✓] 已保存 QR 结果: {args.output}")
        if args.undistort and not undistort_seen:
            print("[!] 未拿到有效 camera_info，实际未进行畸变矫正")
        print("[动作] QR 扫码完成")
    finally:
        try:
            topic.unsubscribe()
        except Exception:
            pass
        if info_topic is not None:
            try:
                info_topic.unsubscribe()
            except Exception:
                pass
        if preview_topic is not None:
            try:
                preview_topic.unsubscribe()
            except Exception:
                pass
        try:
            client.terminate()
        except Exception:
            pass
        if raw_client is not None:
            try:
                raw_client.terminate()
            except Exception:
                pass
        if args.show_window:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
