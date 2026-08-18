#!/usr/bin/env python3
"""QR-only multi-frame recovery debugger.

No OCR is used. The pipeline is:

  frames -> qrdet QR quad -> perspective normalize -> multi-frame fusion
         -> QR-only enhancement -> local QR decoders

It can either read images from a directory or capture raw RGB snapshots through
rosbridge. It does not send robot motion commands.
"""

from __future__ import annotations

import argparse
import base64
import json
import logging
import os
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
os.environ.setdefault("YOLO_VERBOSE", "False")
try:
    from ultralytics.utils import LOGGER as _ULTRALYTICS_LOGGER

    _ULTRALYTICS_LOGGER.setLevel(logging.ERROR)
except Exception:
    pass

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot_grasp.common import config
from robot_grasp.vision.camera_guard import ensure_realsense_ready_on_client
from tools.debug.debug_qr_recover_pipeline import (
    _decode,
    _detect_quads,
    _enhance_variants,
    _make_wechat_detector,
    _save_image,
    _warp_quad,
)

try:
    import roslibpy
except ImportError:
    roslibpy = None

try:
    from qreader import QReader
except ImportError:
    QReader = None


class _FrameSubscriber:
    def __init__(self):
        self.frame = None
        self.seq = 0
        self.lock = threading.Lock()

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
                frame = cv2.cvtColor(arr.reshape((height, width)), cv2.COLOR_GRAY2BGR)
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


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="QR-only multi-frame recovery debugger")
    parser.add_argument("--input-dir", help="Read local images instead of capturing ROS frames")
    parser.add_argument("--ws-url", default=config.WS_URL)
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--interval", type=float, default=0.15)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--side", type=int, default=840)
    parser.add_argument("--expand", type=float, default=1.15)
    parser.add_argument(
        "--output-dir",
        default=str(PROJECT_ROOT / "data" / "runtime" / "qr_multiframe_debug"),
    )
    parser.add_argument("--save-raw", action="store_true")
    parser.add_argument("--capture-only", action="store_true", help="Only save frames/report, skip QR recovery")
    parser.add_argument("--no-wechat", action="store_true")
    parser.add_argument("--fast", action="store_true")
    parser.add_argument(
        "--min-consensus",
        type=int,
        default=2,
        help="Accept a QR text only after it appears this many times in crop decoding",
    )
    parser.add_argument(
        "--result-output",
        default=str(PROJECT_ROOT / "data" / "runtime" / "post_grasp_qr_latest.json"),
        help="Write accepted QR result here",
    )
    return parser.parse_args()


def _parse_ws_url(ws_url: str) -> tuple[str, int]:
    stripped = ws_url.replace("ws://", "").replace("wss://", "")
    host, port = stripped.split(":")
    return host, int(port)


def _connect(ws_url: str):
    if roslibpy is None:
        raise RuntimeError("缺少 roslibpy，不能从 ROS 抓帧")
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


def _load_local_frames(input_dir: Path) -> list[tuple[np.ndarray, str]]:
    paths = []
    for pattern in ("*.png", "*.jpg", "*.jpeg", "*.bmp"):
        paths.extend(sorted(input_dir.glob(pattern)))
    frames = []
    for path in paths:
        image = cv2.imread(str(path))
        if image is not None:
            frames.append((image, path.name))
    return frames


def _capture_raw_frames(args: argparse.Namespace, output_dir: Path) -> list[tuple[np.ndarray, str]]:
    client = _connect(args.ws_url)
    ensure_realsense_ready_on_client(
        client,
        require_depth=False,
        require_camera_info=False,
        require_raw_rgb=True,
    )
    subscriber = _FrameSubscriber()
    topic = roslibpy.Topic(
        client,
        config.TOPIC_RGB_RAW,
        "sensor_msgs/Image",
        throttle_rate=0,
        queue_length=1,
    )
    topic.subscribe(subscriber.raw_callback)
    frames: list[tuple[np.ndarray, str]] = []
    seen_seq = -1
    start = time.time()
    try:
        while len(frames) < args.count and time.time() - start < args.timeout:
            frame, seq = subscriber.latest()
            if frame is not None and seq != seen_seq:
                seen_seq = seq
                name = f"raw_{len(frames) + 1:03d}.png"
                frames.append((frame, name))
                if args.save_raw:
                    _save_image(output_dir / "raw" / name, frame)
                time.sleep(max(0.0, args.interval))
            else:
                time.sleep(0.02)
    finally:
        try:
            topic.unsubscribe()
        except Exception:
            pass
        try:
            client.terminate()
        except Exception:
            pass
    return frames


def _normalize_frames(
    frames: list[tuple[np.ndarray, str]],
    output_dir: Path,
    side: int,
    expand: float,
) -> tuple[list[np.ndarray], list[dict[str, Any]]]:
    normalized = []
    records = []
    for index, (frame, name) in enumerate(frames, start=1):
        quads, meta = _detect_quads(frame)
        record: dict[str, Any] = {
            "index": index,
            "name": name,
            "detections": meta,
            "used": False,
        }
        if not quads:
            records.append(record)
            continue

        quad = quads[0]
        try:
            warped = _warp_quad(frame, quad, expand=expand, side=side)
        except cv2.error as exc:
            record["error"] = repr(exc)
            records.append(record)
            continue

        border = max(12, int(side * 4.0 / 29.0))
        warped = cv2.copyMakeBorder(
            warped,
            border,
            border,
            border,
            border,
            borderType=cv2.BORDER_CONSTANT,
            value=(255, 255, 255),
        )
        normalized.append(warped)
        record["used"] = True
        record["normalized"] = f"normalized_{len(normalized):03d}.png"
        _save_image(output_dir / "normalized" / record["normalized"], warped)
        records.append(record)
    return normalized, records


def _fuse_frames(frames: list[np.ndarray]) -> list[tuple[str, np.ndarray]]:
    if not frames:
        return []
    stack = np.stack([frame.astype(np.float32) for frame in frames], axis=0)
    mean = np.clip(np.mean(stack, axis=0), 0, 255).astype(np.uint8)
    median = np.clip(np.median(stack, axis=0), 0, 255).astype(np.uint8)
    clarity = []
    for frame in frames:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        clarity.append(float(cv2.Laplacian(gray, cv2.CV_64F).var()))
    best_index = int(np.argmax(clarity))
    best = frames[best_index]

    weights = np.asarray(clarity, dtype=np.float32)
    if float(weights.sum()) <= 1e-6:
        weighted = mean
    else:
        weights = weights / weights.sum()
        weighted_stack = stack * weights.reshape((-1, 1, 1, 1))
        weighted = np.clip(np.sum(weighted_stack, axis=0), 0, 255).astype(np.uint8)

    fused = [
        ("single_middle", frames[len(frames) // 2]),
        ("sharpest", best),
        ("mean", mean),
        ("median", median),
        ("laplacian_weighted", weighted),
    ]
    expanded = []
    for name, image in fused:
        expanded.append((name, image))
        for scale in (4, 8):
            expanded.append((
                f"{name}_sr_x{scale}",
                cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_LANCZOS4),
            ))
    return expanded


def _looks_like_valid_qr_text(text: str) -> bool:
    if not text:
        return False
    if any(ord(ch) < 32 for ch in text):
        return False
    if len(text) < 4:
        return False
    return True


def _qreader_detect_and_decode(reader, image: np.ndarray) -> list[str]:
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) if image.ndim == 3 else image
    outputs = []
    for kwargs in (
        {"return_detections": True, "is_bgr": False},
        {"return_detections": False, "is_bgr": False},
        {"return_detections": True},
        {},
    ):
        try:
            result = reader.detect_and_decode(image=rgb, **kwargs)
        except TypeError:
            continue
        except Exception:
            return outputs
        if isinstance(result, tuple) and len(result) == 2 and isinstance(result[0], (list, tuple)):
            texts = result[0]
        elif isinstance(result, (list, tuple)):
            texts = result
        else:
            texts = [result]
        for text in texts:
            if isinstance(text, bytes):
                try:
                    text = text.decode("utf-8")
                except Exception:
                    text = str(text)
            if isinstance(text, str) and _looks_like_valid_qr_text(text):
                outputs.append(text)
        break
    return outputs


def _decode_per_frame_crops(
    frames: list[tuple[np.ndarray, str]],
    output_dir: Path,
    min_consensus: int,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    if QReader is None:
        return [], [{"backend": "qreader-crop", "error": "qreader not installed"}]

    try:
        reader = QReader(model_size="n")
    except TypeError:
        reader = QReader()

    hits: list[dict[str, Any]] = []
    attempts: list[dict[str, Any]] = []
    pads = (20, 35, 50, 70, 100, 130, 160, 220)

    for frame_index, (frame, frame_name) in enumerate(frames, start=1):
        quads, meta = _detect_quads(frame)
        attempts.append({
            "frame_index": frame_index,
            "frame": frame_name,
            "detections": meta,
        })
        for det_index, quad in enumerate(quads, start=1):
            x1, y1 = np.floor(quad.min(axis=0)).astype(int)
            x2, y2 = np.ceil(quad.max(axis=0)).astype(int)
            for pad in pads:
                xx1 = max(0, x1 - pad)
                yy1 = max(0, y1 - pad)
                xx2 = min(frame.shape[1], x2 + pad)
                yy2 = min(frame.shape[0], y2 + pad)
                if xx2 <= xx1 or yy2 <= yy1:
                    continue
                crop = frame[yy1:yy2, xx1:xx2]
                texts = _qreader_detect_and_decode(reader, crop)
                if not texts:
                    continue
                crop_name = f"{Path(frame_name).stem}_det{det_index}_pad{pad}.png"
                crop_path = output_dir / "hits" / crop_name
                _save_image(crop_path, crop)
                for text in texts:
                    hits.append({
                        "backend": "qreader-crop",
                        "text": text,
                        "accepted": True,
                        "frame_index": frame_index,
                        "frame": frame_name,
                        "det_index": det_index,
                        "pad": pad,
                        "crop": str(crop_path),
                        "quad_xy": quad.tolist(),
                    })

    counts: dict[str, int] = {}
    for hit in hits:
        counts[hit["text"]] = counts.get(hit["text"], 0) + 1

    accepted = []
    for text, count in sorted(counts.items(), key=lambda item: item[1], reverse=True):
        if count >= min_consensus:
            first = next(hit for hit in hits if hit["text"] == text)
            accepted.append({
                **first,
                "count": count,
                "consensus": True,
            })
    return accepted, attempts + hits


def _try_decode_fused(
    fused: list[tuple[str, np.ndarray]],
    output_dir: Path,
    use_wechat: bool,
    fast: bool,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    wechat = _make_wechat_detector() if use_wechat else None
    accepted = []
    attempts = []
    attempt_index = 0
    for base_name, image in fused:
        _save_image(output_dir / f"{base_name}.png", image)
        for variant in _enhance_variants(image, fast=fast):
            attempt_index += 1
            name = f"{attempt_index:04d}_{base_name}_{variant.name}.png"
            _save_image(output_dir / "variants" / name, variant.image)
            decoded = _decode(variant.image, wechat, use_wechat=use_wechat, fast=fast)
            useful = [item for item in decoded if item.get("accepted") or item.get("error") or item.get("text")]
            if useful:
                attempts.append({
                    "index": attempt_index,
                    "name": name,
                    "shape": list(variant.image.shape),
                    "decoded": useful,
                })
            for item in decoded:
                if item.get("accepted"):
                    accepted.append({"attempt": attempt_index, "name": name, **item})
    return accepted, attempts


def _write_result(path: Path, accepted: list[dict[str, Any]], source: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    texts = []
    for item in accepted:
        text = item.get("text")
        if text and text not in texts:
            texts.append(text)
    payload = {
        "created_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "source": source,
        "count": len(texts),
        "texts": texts,
        "detections": accepted,
    }
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)


def main() -> int:
    args = _parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.input_dir:
        frames = _load_local_frames(Path(args.input_dir))[: max(1, args.count)]
        source = f"input_dir:{args.input_dir}"
    else:
        frames = _capture_raw_frames(args, output_dir)
        source = f"ros:{args.ws_url}"

    if args.capture_only:
        report = {
            "created_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "source": source,
            "frame_count": len(frames),
            "capture_only": True,
            "raw_dir": str(output_dir / "raw") if args.save_raw else None,
        }
        report_path = output_dir / "report.json"
        with report_path.open("w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        print(f"source: {source}")
        print(f"output_dir: {output_dir}")
        print(f"frames: {len(frames)}")
        print("normalized: 0")
        print("accepted: 0")
        print(f"report: {report_path}")
        return 0 if frames else 3

    crop_accepted, crop_attempts = _decode_per_frame_crops(
        frames,
        output_dir,
        min_consensus=max(1, args.min_consensus),
    )

    normalized = []
    records = []
    fused_accepted = []
    fused_attempts = []
    if not crop_accepted:
        normalized, records = _normalize_frames(frames, output_dir, side=args.side, expand=args.expand)
        fused = _fuse_frames(normalized)
        fused_accepted, fused_attempts = _try_decode_fused(
            fused,
            output_dir,
            use_wechat=not args.no_wechat,
            fast=args.fast,
        )

    accepted = crop_accepted or fused_accepted

    report = {
        "created_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "source": source,
        "frame_count": len(frames),
        "normalized_count": len(normalized),
        "side": args.side,
        "expand": args.expand,
        "crop_consensus_min": max(1, args.min_consensus),
        "crop_accepted": crop_accepted,
        "crop_attempts": crop_attempts[:160],
        "records": records,
        "accepted": accepted,
        "attempts": fused_attempts[:120],
    }
    report_path = output_dir / "report.json"
    with report_path.open("w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    if accepted:
        _write_result(Path(args.result_output), accepted, source)
        _write_result(output_dir / "qr_result.json", accepted, source)

    print(f"source: {source}")
    print(f"output_dir: {output_dir}")
    print(f"frames: {len(frames)}")
    print(f"normalized: {len(normalized)}")
    print(f"accepted: {len(accepted)}")
    for item in accepted:
        detail = ""
        if "frame" in item:
            detail = f" frame={item.get('frame')} pad={item.get('pad')} count={item.get('count')}"
        elif "attempt" in item:
            detail = f" attempt={item.get('attempt')}"
        print(f"  [OK] {item['backend']}: {item['text']}{detail}")
    if not accepted:
        print(f"attempts_with_text_or_error: {len(fused_attempts)}")
        for attempt in fused_attempts[:10]:
            for item in attempt["decoded"][:3]:
                print(
                    f"  [TRY] #{attempt['index']} {item.get('backend')} "
                    f"text={item.get('text')!r} error={item.get('error')}"
                )
    else:
        print(f"result: {args.result_output}")
    print(f"report: {report_path}")
    return 0 if accepted else 2


if __name__ == "__main__":
    raise SystemExit(main())
