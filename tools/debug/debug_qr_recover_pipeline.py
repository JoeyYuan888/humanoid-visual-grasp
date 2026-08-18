#!/usr/bin/env python3
"""Offline QR recovery pipeline debug tool.

This script does not connect to ROS and does not move the robot. It takes one
local image, detects QR quadrangles, rectifies them, generates enhanced
variants, and runs available local decoders. It is intended for diagnosing why
phone apps can decode a frame while local decoders fail.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import sys
from dataclasses import dataclass
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

try:
    import zxingcpp
except ImportError:
    zxingcpp = None

try:
    from pyzbar import pyzbar
except ImportError:
    pyzbar = None

try:
    from qrdet import QRDetector as QRDDetector
except ImportError:
    QRDDetector = None


@dataclass
class Candidate:
    name: str
    image: np.ndarray
    points: np.ndarray | None = None


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline QR recovery pipeline debugger")
    parser.add_argument("image", help="Input image path")
    parser.add_argument(
        "--output-dir",
        default=str(PROJECT_ROOT / "data" / "runtime" / "qr_recover_debug"),
        help="Directory for intermediate images and report",
    )
    parser.add_argument("--max-variants", type=int, default=400)
    parser.add_argument("--fast", action="store_true", help="Use a bounded fast variant/backend set")
    parser.add_argument("--no-wechat", action="store_true", help="Skip OpenCV WeChatQRCode decoder")
    parser.add_argument("--quiet", action="store_true")
    return parser.parse_args()


def _safe_name(name: str) -> str:
    keep = []
    for ch in name:
        keep.append(ch if ch.isalnum() or ch in ("-", "_", ".") else "_")
    return "".join(keep)[:180]


def _order_quad(points: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float32).reshape(4, 2)
    sums = pts.sum(axis=1)
    diffs = np.diff(pts, axis=1).reshape(-1)
    return np.asarray(
        [
            pts[np.argmin(sums)],
            pts[np.argmin(diffs)],
            pts[np.argmax(sums)],
            pts[np.argmax(diffs)],
        ],
        dtype=np.float32,
    )


def _make_wechat_detector():
    detector_cls = getattr(cv2, "wechat_qrcode_WeChatQRCode", None)
    module = getattr(cv2, "wechat_qrcode", None)
    if detector_cls is None and module is not None:
        detector_cls = getattr(module, "WeChatQRCode", None)
    if detector_cls is None:
        return None

    model_dir = PROJECT_ROOT / "models" / "qr" / "wechat_qrcode"
    model_paths = [
        model_dir / "detect.prototxt",
        model_dir / "detect.caffemodel",
        model_dir / "sr.prototxt",
        model_dir / "sr.caffemodel",
    ]
    try:
        if all(path.exists() for path in model_paths):
            return detector_cls(*(str(path) for path in model_paths))
        return detector_cls()
    except Exception:
        return None


def _detect_quads(image: np.ndarray) -> tuple[list[np.ndarray], list[dict[str, Any]]]:
    quads: list[np.ndarray] = []
    meta: list[dict[str, Any]] = []

    qr = cv2.QRCodeDetector()
    try:
        ok, points = qr.detectMulti(image)
        if ok and points is not None:
            for pts in points:
                quad = _order_quad(np.asarray(pts, dtype=np.float32).reshape(4, 2))
                quads.append(quad)
                meta.append({"source": "opencv-detectMulti", "quad_xy": quad.tolist()})
    except cv2.error:
        pass

    if QRDDetector is not None:
        try:
            detector = QRDDetector(model_size="n")
            rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            for det in detector.detect(image=rgb, is_bgr=False):
                quad = det.get("quad_xy")
                bbox = det.get("bbox_xyxy")
                if quad is None and bbox is not None:
                    x1, y1, x2, y2 = [float(v) for v in bbox]
                    quad = np.asarray([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32)
                if quad is None:
                    continue
                quad = _order_quad(np.asarray(quad, dtype=np.float32).reshape(4, 2))
                quads.append(quad)
                meta.append({
                    "source": "qrdet",
                    "confidence": float(det.get("confidence", 0.0)),
                    "bbox_xyxy": np.asarray(bbox).tolist() if bbox is not None else None,
                    "quad_xy": quad.tolist(),
                })
        except Exception as exc:
            meta.append({"source": "qrdet", "error": repr(exc)})

    deduped: list[np.ndarray] = []
    deduped_meta: list[dict[str, Any]] = []
    for quad, item in zip(quads, meta):
        center = quad.mean(axis=0)
        if any(np.linalg.norm(center - old.mean(axis=0)) < 12.0 for old in deduped):
            continue
        deduped.append(quad)
        deduped_meta.append(item)
    return deduped, deduped_meta


def _warp_quad(image: np.ndarray, quad: np.ndarray, expand: float, side: int) -> np.ndarray:
    ordered = _order_quad(quad)
    center = ordered.mean(axis=0)
    expanded = (ordered - center) * expand + center
    dst = np.asarray(
        [[0, 0], [side - 1, 0], [side - 1, side - 1], [0, side - 1]],
        dtype=np.float32,
    )
    matrix = cv2.getPerspectiveTransform(expanded.astype(np.float32), dst)
    return cv2.warpPerspective(
        image,
        matrix,
        (side, side),
        flags=cv2.INTER_LANCZOS4,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(255, 255, 255),
    )


def _enhance_variants(image: np.ndarray, fast: bool = False) -> list[Candidate]:
    variants = [Candidate("bgr", image)]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    variants.append(Candidate("gray", gray))
    variants.append(Candidate("gray_equalize", cv2.equalizeHist(gray)))
    variants.append(Candidate("clahe", cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)).apply(gray)))

    sharpen_kernel = np.asarray([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
    variants.append(Candidate("sharpen_gray", cv2.filter2D(gray, -1, sharpen_kernel)))
    if image.ndim == 3:
        variants.append(Candidate("sharpen_bgr", cv2.filter2D(image, -1, sharpen_kernel)))

    blocks = (21, 41) if fast else (15, 21, 31, 41, 61)
    for block in blocks:
        adaptive = cv2.adaptiveThreshold(
            cv2.GaussianBlur(gray, (3, 3), 0),
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            block,
            3,
        )
        variants.append(Candidate(f"adaptive_b{block}", adaptive))

    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    _, otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    variants.append(Candidate("otsu", otsu))
    variants.append(Candidate("otsu_inv", cv2.bitwise_not(otsu)))

    resized_variants = []
    scales = (2.0, 4.0) if fast else (2.0, 3.0, 4.0, 6.0)
    for candidate in variants:
        for scale in scales:
            resized_variants.append(
                Candidate(
                    f"{candidate.name}_x{scale:g}",
                    cv2.resize(candidate.image, None, fx=scale, fy=scale, interpolation=cv2.INTER_LANCZOS4),
                    candidate.points,
                )
            )
    return variants + resized_variants


def _decode(image: np.ndarray, wechat_detector, use_wechat: bool = True,
            fast: bool = False) -> list[dict[str, Any]]:
    decoded: list[dict[str, Any]] = []

    if zxingcpp is not None:
        binarizer_names = ("LocalAverage", "FixedThreshold") if fast else (
            "LocalAverage", "GlobalHistogram", "FixedThreshold", "BoolCast"
        )
        for binarizer_name in binarizer_names:
            binarizer = getattr(zxingcpp.Binarizer, binarizer_name, None)
            if binarizer is None:
                continue
            pure_options = (False,) if fast else (False, True)
            for is_pure in pure_options:
                try:
                    results = zxingcpp.read_barcodes(
                        image,
                        formats=zxingcpp.BarcodeFormat.QRCode,
                        try_rotate=True,
                        try_downscale=False,
                        try_invert=True,
                        binarizer=binarizer,
                        is_pure=is_pure,
                        return_errors=not fast,
                    )
                except Exception:
                    results = []
                for result in results:
                    text = getattr(result, "text", "")
                    error = getattr(result, "error", None)
                    decoded.append({
                        "backend": "zxing-cpp",
                        "text": text,
                        "error": str(error) if error else None,
                        "binarizer": binarizer_name,
                        "is_pure": is_pure,
                        "accepted": bool(text and not error),
                    })

    if pyzbar is not None:
        try:
            pyzbar_results = pyzbar.decode(image)
        except Exception:
            pyzbar_results = []
        for result in pyzbar_results:
            try:
                text = result.data.decode("utf-8")
            except Exception:
                text = str(result.data)
            decoded.append({"backend": "pyzbar", "text": text, "error": None, "accepted": bool(text)})

    if use_wechat and wechat_detector is not None:
        try:
            texts, _ = wechat_detector.detectAndDecode(image)
        except Exception:
            texts = []
        for text in texts or []:
            decoded.append({"backend": "wechat-qrcode", "text": text, "error": None, "accepted": bool(text)})

    qr = cv2.QRCodeDetector()
    try:
        ok, texts, _, _ = qr.detectAndDecodeMulti(image)
        if ok:
            for text in texts:
                decoded.append({"backend": "opencv-multi", "text": text, "error": None, "accepted": bool(text)})
    except cv2.error:
        pass
    try:
        text, _, _ = qr.detectAndDecode(image)
        decoded.append({"backend": "opencv", "text": text, "error": None, "accepted": bool(text)})
    except cv2.error:
        pass

    return decoded


def _save_image(path: Path, image: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), image)


def main() -> int:
    args = _parse_args()
    image_path = Path(args.image)
    image = cv2.imread(str(image_path))
    if image is None:
        raise SystemExit(f"cannot read image: {image_path}")

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    report: dict[str, Any] = {
        "input": str(image_path),
        "shape": list(image.shape),
        "opencv": cv2.__version__,
        "detections": [],
        "attempts": [],
    }

    _save_image(output_dir / "00_input.png", image)
    wechat_detector = _make_wechat_detector()
    quads, quad_meta = _detect_quads(image)
    report["detections"] = quad_meta

    candidates: list[Candidate] = [Candidate("full", image)]
    for index, quad in enumerate(quads, start=1):
        x1, y1 = np.floor(quad.min(axis=0)).astype(int)
        x2, y2 = np.ceil(quad.max(axis=0)).astype(int)
        expands = (1.0, 1.15, 1.7, 2.2) if args.fast else (1.0, 1.08, 1.15, 1.35, 1.7, 2.2, 3.0)
        sides = (420, 840) if args.fast else (210, 420, 840, 1260)
        quiet_zones = (0.0, 4.0) if args.fast else (0.0, 2.0, 4.0, 8.0)
        for expand in expands:
            for side in sides:
                try:
                    warped = _warp_quad(image, quad, expand=expand, side=side)
                except cv2.error:
                    continue
                for border_modules in quiet_zones:
                    if border_modules <= 0:
                        bordered = warped
                    else:
                        border = max(8, int(side * border_modules / 29.0))
                        bordered = cv2.copyMakeBorder(
                            warped,
                            border,
                            border,
                            border,
                            border,
                            borderType=cv2.BORDER_CONSTANT,
                            value=(255, 255, 255),
                        )
                    candidates.append(
                        Candidate(
                            f"det{index}_warp_e{expand:g}_s{side}_qz{border_modules:g}",
                            bordered,
                            quad,
                        )
                    )
        pads = (48, 160) if args.fast else (0, 24, 48, 96, 160, 240)
        for pad in pads:
            xx1 = max(0, x1 - pad)
            yy1 = max(0, y1 - pad)
            xx2 = min(image.shape[1], x2 + pad)
            yy2 = min(image.shape[0], y2 + pad)
            if xx2 <= xx1 or yy2 <= yy1:
                continue
            candidates.append(Candidate(f"det{index}_crop_pad{pad}", image[yy1:yy2, xx1:xx2], quad))

    checked = 0
    accepted = []
    all_errors = []
    for candidate in candidates:
        for variant in _enhance_variants(candidate.image, fast=args.fast):
            checked += 1
            if checked > args.max_variants:
                break
            name = _safe_name(f"{candidate.name}_{variant.name}")
            image_file = output_dir / f"{checked:04d}_{name}.png"
            _save_image(image_file, variant.image)
            decoded = _decode(
                variant.image,
                wechat_detector,
                use_wechat=not args.no_wechat,
                fast=args.fast,
            )
            accepted_here = [item for item in decoded if item.get("accepted")]
            errors_here = [item for item in decoded if item.get("error") or item.get("text")]
            attempt = {
                "index": checked,
                "name": name,
                "image": str(image_file),
                "shape": list(variant.image.shape),
                "decoded": decoded,
            }
            if accepted_here or errors_here:
                report["attempts"].append(attempt)
            accepted.extend({"attempt": checked, "name": name, **item} for item in accepted_here)
            all_errors.extend({"attempt": checked, "name": name, **item} for item in errors_here)
        if checked > args.max_variants:
            break

    report["checked_variants"] = checked
    report["accepted"] = accepted
    report["non_empty_or_error_results"] = all_errors[:80]
    report_path = output_dir / "report.json"
    with report_path.open("w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    if not args.quiet:
        print(f"input: {image_path}")
        print(f"output_dir: {output_dir}")
        print(f"detected_quads: {len(quads)}")
        for item in quad_meta:
            print(f"  - {item.get('source')} conf={item.get('confidence', '')} bbox={item.get('bbox_xyxy')}")
        print(f"checked_variants: {checked}")
        print(f"accepted_results: {len(accepted)}")
        for item in accepted:
            print(f"  [OK] #{item['attempt']} {item['backend']}: {item['text']}")
        if not accepted:
            print(f"non_empty_or_error_results: {len(all_errors)}")
            for item in all_errors[:12]:
                print(
                    f"  [ERR] #{item['attempt']} {item['backend']} "
                    f"text={item.get('text')!r} error={item.get('error')}"
                )
        print(f"report: {report_path}")

    return 0 if accepted else 2


if __name__ == "__main__":
    raise SystemExit(main())
