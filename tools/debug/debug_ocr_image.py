"""
OCR 本地图片测试 — 不依赖 ROS，直接在图片上测试文字识别效果。

用法:
    python tools/debug/debug_ocr_image.py <图片路径/目录>

示例:
    python tools/debug/debug_ocr_image.py test.jpg
    python tools/debug/debug_ocr_image.py data/          # 批量测试目录下所有图片

快捷键:
    q / ESC  退出
    空格      下一张（批量模式）
    s         保存结果图
"""

from __future__ import annotations

import argparse
import os
import sys
import time
import glob

import cv2
import numpy as np

# 复用 ocr_detector.py 的 OCRDetector / OCRWorker
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

# 直接从 ocr_detector 导入核心类
from robot_grasp.vision.ocr_detector import (
    OCRDetector,
    OCRWorker,
    draw_ocr_detections,
    _ensure_dict,
    _ensure_onnx_model,
    _split_text_lines,
    _ocr_correct,
    _ppocr_rec_preprocess,
    _ctc_greedy_decode,
)


DEFAULT_DEBUG_DIR = os.path.join(PROJECT_ROOT, "data", "runtime", "ocr_label_debug", "latest")


def load_images(path: str) -> list[str]:
    """加载图片路径。支持单文件或目录。"""
    if os.path.isfile(path):
        exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
        if os.path.splitext(path)[1].lower() in exts:
            return [path]
        print(f"[!] 不支持的文件格式: {path}")
        return []

    if os.path.isdir(path):
        exts = ("*.jpg", "*.jpeg", "*.png", "*.bmp", "*.webp")
        files = []
        for ext in exts:
            files.extend(glob.glob(os.path.join(path, ext)))
            files.extend(glob.glob(os.path.join(path, ext.upper())))
        return sorted(files)

    print(f"[!] 路径不存在: {path}")
    return []


def _clip_bbox(bbox, w, h):
    x1, y1, x2, y2 = bbox
    return max(0, x1), max(0, y1), min(w, x2), min(h, y2)


def _expand_bbox(bbox, w, h, mx_ratio=0.45, my_ratio=0.70):
    x1, y1, x2, y2 = bbox
    bw = max(1, x2 - x1)
    bh = max(1, y2 - y1)
    mx = int(bw * mx_ratio)
    my = int(bh * my_ratio)
    return _clip_bbox((x1 - mx, y1 - my, x2 + mx, y2 + my), w, h)


def _rotate_bound(image: np.ndarray, angle: float) -> np.ndarray:
    h, w = image.shape[:2]
    center = (w / 2.0, h / 2.0)
    matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    cos = abs(matrix[0, 0])
    sin = abs(matrix[0, 1])
    new_w = int(h * sin + w * cos)
    new_h = int(h * cos + w * sin)
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


def _find_label_candidates(
    frame: np.ndarray,
    max_candidates: int = 6,
    expand_x: float = 0.18,
    expand_y: float = 0.28,
):
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    dark = cv2.inRange(gray, 0, 135)
    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    small = np.zeros_like(dark)
    for cnt in contours:
        x, y, cw, ch = cv2.boundingRect(cnt)
        area = cv2.contourArea(cnt)
        if 4 <= area <= 2500 and 2 <= cw <= 90 and 2 <= ch <= 90:
            cv2.drawContours(small, [cnt], -1, 255, -1)

    grouped = cv2.dilate(small, cv2.getStructuringElement(cv2.MORPH_RECT, (45, 35)), iterations=1)
    grouped = cv2.morphologyEx(
        grouped,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (31, 31)),
        iterations=1,
    )

    candidates = []
    contours, _ = cv2.findContours(grouped, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        x, y, cw, ch = cv2.boundingRect(cnt)
        if cw < 70 or ch < 50:
            continue
        if cw > w * 0.45 or ch > h * 0.45:
            continue
        aspect = cw / max(ch, 1)
        if not (0.45 <= aspect <= 2.3):
            continue
        x1, y1, x2, y2 = _expand_bbox((x, y, x + cw, y + ch), w, h, expand_x, expand_y)
        roi_gray = gray[y1:y2, x1:x2]
        roi_dark = small[y1:y2, x1:x2]
        dark_count = int(cv2.countNonZero(roi_dark))
        if dark_count < 80:
            continue
        mean_v = float(np.mean(roi_gray))
        bg_bonus = max(0.0, min(1.0, (mean_v - 80.0) / 150.0))
        compact = dark_count / max(1.0, (x2 - x1) * (y2 - y1))
        score = dark_count * (0.7 + bg_bonus) + compact * 6000.0
        candidates.append({"bbox": (x1, y1, x2, y2), "score": score})

    candidates.sort(key=lambda item: item["score"], reverse=True)
    return candidates[:max_candidates]


def _label_variants(crop: np.ndarray, fast: bool = True):
    variants = [("orig", crop)]
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    dark = cv2.inRange(gray, 0, 150)
    pts = cv2.findNonZero(dark)
    if pts is not None and len(pts) > 30:
        rect = cv2.minAreaRect(pts)
        angle = rect[2]
        if angle < -45:
            angle += 90
        if abs(angle) > 2:
            variants.append((f"deskew_{angle:.1f}", _rotate_bound(crop, angle)))
    angles = (-15, 15) if fast else (-25, -15, -8, 8, 15, 25)
    for angle in angles:
        variants.append((f"rot_{angle:+d}", _rotate_bound(crop, angle)))
    return variants


def _recognize_lines_direct(ocr: OCRDetector, image: np.ndarray, min_conf: float):
    out = []
    for y1, y2, line_patch in _split_text_lines(image):
        if line_patch.size == 0:
            continue
        try:
            inp = _ppocr_rec_preprocess(line_patch)
            outputs = ocr._rec_session.run([ocr._rec_output], {ocr._rec_input: inp})
            decoded = _ctc_greedy_decode(outputs[0], ocr._char_list)
        except Exception:
            continue
        for det in decoded:
            text = _ocr_correct(str(det.get("text", "")).strip())
            conf = float(det.get("confidence", 0.0) or 0.0)
            if text and conf >= min_conf:
                out.append({
                    "text": text,
                    "confidence": conf,
                    "bbox": (0, y1, image.shape[1], y2),
                    "backend": "direct-lines",
                    "roi_index": 0,
                })
    return out


def _run_ocr_on_image(ocr: OCRDetector, image: np.ndarray, args):
    if args.force_lines:
        return _recognize_lines_direct(ocr, image, args.min_conf)
    rois = [(0, 0, image.shape[1], image.shape[0])]
    results = ocr.detect(image, rois=rois, upscale=args.upscale)
    for det in results:
        det["text"] = _ocr_correct(det["text"])
    return results


def _detect_with_label_crop(ocr: OCRDetector, frame: np.ndarray, args, image_stem: str):
    candidates = _find_label_candidates(
        frame,
        args.max_label_candidates,
        args.label_expand_x,
        args.label_expand_y,
    )
    all_results = []
    best = []
    best_variant = None

    debug_dir = args.debug_output
    if args.save_debug:
        os.makedirs(debug_dir, exist_ok=True)

    vis = frame.copy()
    for idx, candidate in enumerate(candidates, start=1):
        x1, y1, x2, y2 = candidate["bbox"]
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 200, 255), 2)
        cv2.putText(
            vis,
            f"label{idx}:{candidate['score']:.0f}",
            (x1, max(20, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 200, 255),
            2,
        )
        crop = frame[y1:y2, x1:x2]
        if args.save_debug:
            cv2.imwrite(os.path.join(debug_dir, f"{image_stem}_label_{idx:02d}.png"), crop)

        for variant_name, variant in _label_variants(crop, fast=args.fast):
            results = _run_ocr_on_image(ocr, variant, args)
            for det in results:
                det["roi_index"] = idx - 1
                det["source_variant"] = variant_name
            all_results.extend(results)
            if len(results) > len(best):
                best = results
                best_variant = (idx, variant_name, variant)
            if args.save_debug:
                out_name = f"{image_stem}_label_{idx:02d}_{variant_name}.png"
                cv2.imwrite(os.path.join(debug_dir, out_name), variant)
                if results:
                    annotated = draw_ocr_detections(variant, results, color=(0, 255, 0))
                    cv2.imwrite(os.path.join(debug_dir, out_name.replace(".png", "_ocr.png")), annotated)

    if args.save_debug:
        cv2.imwrite(os.path.join(debug_dir, f"{image_stem}_label_candidates.png"), vis)

    return best if best else all_results, candidates, best_variant


def main():
    parser = argparse.ArgumentParser(description="OCR 本地图片测试")
    parser.add_argument("path", help="图片路径或目录")
    parser.add_argument("--label-crop", action="store_true",
                        help="先自动定位白色标签区域，再对标签裁剪/旋正后 OCR")
    parser.add_argument("--max-label-candidates", type=int, default=2)
    parser.add_argument("--label-expand-x", type=float, default=0.18)
    parser.add_argument("--label-expand-y", type=float, default=0.28)
    parser.add_argument("--upscale", type=float, default=2.0)
    parser.add_argument("--min-conf", type=float, default=0.30)
    parser.add_argument("--force-lines", action="store_true",
                        help="跳过 OCR 检测模型，直接按水平投影分行后识别")
    parser.add_argument("--fast", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--no-window", action="store_true")
    parser.add_argument("--save-debug", action="store_true")
    parser.add_argument("--debug-output", default=DEFAULT_DEBUG_DIR)
    args = parser.parse_args()

    # ── 准备模型 ──
    print("=" * 60, flush=True)
    print("  OCR 本地图片测试", flush=True)
    print("=" * 60, flush=True)
    print(flush=True)

    if not _ensure_dict():
        sys.exit(1)
    if not _ensure_onnx_model():
        sys.exit(1)

    ocr = OCRDetector()
    if ocr._rec_session is None:
        print("[✗] OCR 引擎初始化失败")
        sys.exit(1)
    print(f"[*] OCR backends: {ocr.backend_info()}", flush=True)
    print(flush=True)

    # ── 加载图片 ──
    images = load_images(args.path)
    if not images:
        sys.exit(1)

    print(f"[*] 共 {len(images)} 张图片\n", flush=True)

    total_idx = 0
    for img_path in images:
        total_idx += 1
        print(f"[{total_idx}/{len(images)}] {os.path.basename(img_path)}", flush=True)

        frame = cv2.imread(img_path)
        if frame is None:
            print(f"    [!] 无法读取图片，跳过")
            continue

        # 可选：如果图片太大，缩放以便查看
        h, w = frame.shape[:2]
        scale = 1.0
        if max(h, w) > 1920:
            scale = 1920.0 / max(h, w)
            frame = cv2.resize(frame, None, fx=scale, fy=scale,
                               interpolation=cv2.INTER_AREA)
            print(f"    [*] 缩放至 {frame.shape[1]}x{frame.shape[0]}")

        # ── 执行 OCR ──
        t0 = time.time()

        if args.label_crop:
            results, candidates, best_variant = _detect_with_label_crop(
                ocr,
                frame,
                args,
                os.path.splitext(os.path.basename(img_path))[0],
            )
            print(f"    label candidates: {len(candidates)}", flush=True)
            if best_variant is not None:
                print(f"    best label: candidate={best_variant[0]} variant={best_variant[1]}", flush=True)
        else:
            rois = [(0, 0, frame.shape[1], frame.shape[0])]
            results = ocr.detect(frame, rois=rois, upscale=args.upscale)

        elapsed = (time.time() - t0) * 1000.0

        # OCR 后纠错 + 打印前后对比
        print(f"    [{elapsed:.0f}ms] 识别到 {len(results)} 段文字:", flush=True)
        for i, det in enumerate(results, 1):
            raw = det["text"]
            corrected = _ocr_correct(raw)
            det["text"] = corrected
            if raw != corrected:
                print(f"      {i}. [{det['confidence']:.2f}] {raw}", flush=True)
                print(f"          -> 纠正: {corrected}", flush=True)
            else:
                print(f"      {i}. [{det['confidence']:.2f}] {raw}", flush=True)

        # ── 显示结果 ──
        annotated = draw_ocr_detections(frame, results, color=(0, 255, 0))

        # 状态信息
        _, w = annotated.shape[:2]
        cv2.rectangle(annotated, (0, 0), (w, 40), (0, 0, 0), -1)
        cv2.putText(annotated,
                    f"OCR: {len(results)} texts | {elapsed:.0f}ms | "
                    f"q:quit  space:next  s:save",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 2)

        # ── 显示 ──
        if args.no_window:
            if args.save_debug:
                os.makedirs(args.debug_output, exist_ok=True)
                save_path = os.path.join(
                    args.debug_output,
                    f"{os.path.splitext(os.path.basename(img_path))[0]}_ocr_result.png",
                )
                cv2.imwrite(save_path, annotated)
                print(f"    [✓] 已保存: {save_path}")
            continue

        cv2.imshow("OCR Image Test", annotated)
        while True:
            key = cv2.waitKey(0) & 0xFF
            if key in (ord("q"), 27):  # q or ESC
                cv2.destroyAllWindows()
                print()
                return
            elif key == ord(" "):  # 下一张
                break
            elif key == ord("s"):
                save_path = f"ocr_result_{total_idx}.png"
                cv2.imwrite(save_path, annotated)
                print(f"    [✓] 已保存: {save_path}")

    cv2.destroyAllWindows()
    print("=" * 60)
    print("全部完成")
    print("=" * 60)


if __name__ == "__main__":
    main()
