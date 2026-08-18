"""
OCR 摄像头实时测试 — 使用本地摄像头替代 ROS 订阅。

用法:
    python tools/debug/debug_ocr_webcam.py [摄像头ID]

默认摄像头 ID 为 0。按屏幕提示操作。

快捷键:
    q / ESC  退出
    s         保存当前帧截图到 data/
    r         重置 OCR 记忆
"""

from __future__ import annotations

import os
import sys
import time
from collections import deque

import cv2
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot_grasp.vision.ocr_detector import (
    OCRDetector,
    OCRWorker,
    draw_ocr_detections,
    _ensure_dict,
    _ensure_onnx_model,
    _merge_ocr_memory,
    _ocr_summary,
    _enhance_label,
    _ocr_correct,
    OCR_MEMORY_TTL_SEC,
    OCR_DECODE_INTERVAL_SEC,
)

# ── 配置 ──────────────────────────────────────────────────

CAMERA_ID = int(sys.argv[1]) if len(sys.argv) > 1 else 0
FRAME_WIDTH = 1280   # 摄像头采集宽度
FRAME_HEIGHT = 720   # 摄像头采集高度
OCR_INTERVAL = max(OCR_DECODE_INTERVAL_SEC, 1.0)   # OCR 间隔（秒）
SHOW_RAW_FRAME = True  # 是否显示原始帧（不缩放）
DISPLAY_SCALE = 1.0    # 显示缩放（1.0=原始，0.5=半屏）
OCR_UPSCALE = 2.0      # 检测前放大倍数（文字太小设 >1.0）

# ── 多帧共识 ────────────────────────────────────────────


def _text_consensus(results_buffer: list[list[dict]],
                    min_votes: int = 2) -> list[dict]:
    """多帧 OCR 结果投票，返回出现在 ≥min_votes 帧中的文字。

    按 text 分组，统计出现次数，只保留高频结果。
    """
    if not results_buffer:
        return []

    # 统计每个 text 出现的次数和最大置信度
    vote: dict[str, dict] = {}
    for frame_results in results_buffer:
        seen_in_frame = set()
        for det in frame_results:
            text = det["text"]
            if text in seen_in_frame:
                continue
            seen_in_frame.add(text)
            if text not in vote:
                vote[text] = {"count": 0, "det": det}
            vote[text]["count"] += 1
            if det["confidence"] > vote[text]["det"]["confidence"]:
                vote[text]["det"] = det

    out = []
    for text, entry in vote.items():
        if entry["count"] >= min_votes:
            out.append(entry["det"])
    return out


def main():
    print("=" * 60)
    print("  OCR 摄像头实时测试")
    print("=" * 60)
    print(f"  摄像头: ID {CAMERA_ID}")
    print(f"  分辨率: {FRAME_WIDTH}x{FRAME_HEIGHT}")
    print(f"  OCR 间隔: {OCR_INTERVAL:.1f}s")
    print("=" * 60)
    print()

    # ── 准备模型 ──
    if not _ensure_dict():
        sys.exit(1)
    if not _ensure_onnx_model():
        sys.exit(1)

    ocr_worker = OCRWorker()
    if ocr_worker._detector._rec_session is None:
        print("[✗] OCR 引擎初始化失败")
        sys.exit(1)
    print(f"[*] OCR backends: {ocr_worker.backend_info()}")
    print()

    # ── 打开摄像头 ──
    cap = cv2.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        print(f"[✗] 无法打开摄像头 ID {CAMERA_ID}")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[✓] 摄像头已打开: {actual_w}x{actual_h} @ {actual_fps:.0f}fps\n")

    # ── 状态 ──
    ocr_memory = {}
    seen_texts: "set[str]" = set()
    last_ocr_time = 0.0
    last_ocr_results = []
    ocr_busy = False
    ocr_stats = {}
    last_decode_id = 0

    # 多帧共识缓冲（保留最近 3 帧的 OCR 结果）
    result_buffer: "deque[list[dict]]" = deque(maxlen=3)

    fps_history = []
    fps = 0.0
    frame_count = 0
    prev_time = time.time()

    print("[*] 开始实时识别...")
    print("[*] q=退出 | s=保存截图 | r=重置记忆\n")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[!] 摄像头断流")
                break

            frame_count += 1
            now = time.time()

            # ── FPS 统计 ──
            fps_history.append(now)
            cutoff = now - 1.0
            fps_history = [t for t in fps_history if t > cutoff]
            fps = len(fps_history)

            # ── 提交 OCR（异步，不阻塞采集） ──
            # 使用全图作为 ROI
            rois = [(0, 0, frame.shape[1], frame.shape[0])]
            if now - last_ocr_time >= OCR_INTERVAL:
                if ocr_worker.submit(frame, rois, "camera", mode=f"U{OCR_UPSCALE:.1f}",
                                     upscale=OCR_UPSCALE):
                    last_ocr_time = now

            last_ocr_results, ocr_busy, ocr_stats = ocr_worker.latest()

            # OCR 后纠错
            for det in last_ocr_results:
                det["text"] = _ocr_correct(det["text"])

            # 仅当有新解码完成时才入缓冲（避免同一结果重复投票）
            decode_id = ocr_stats.get("decode_id", 0)
            if decode_id != last_decode_id:
                last_decode_id = decode_id
                if last_ocr_results:
                    result_buffer.append(last_ocr_results)

            # 多帧投票：显示在 ≥2 帧中出现的文字
            if len(result_buffer) >= 2:
                display_results = _text_consensus(list(result_buffer), min_votes=2)
            else:
                display_results = last_ocr_results

            memory_results = _merge_ocr_memory(ocr_memory, display_results, now)

            # ── 新文字首次识别时打印 ──
            for det in display_results:
                text = det["text"]
                if text not in seen_texts:
                    seen_texts.add(text)
                    print(f"  [OCR {len(seen_texts)}] {text}  "
                          f"(conf={det['confidence']:.2f})")

            # ── 标注画面（只绘制投票通过的文字） ──
            annotated = draw_ocr_detections(frame, display_results, color=(0, 255, 0))

            # ── HUD ──
            h, w = annotated.shape[:2]
            cv2.rectangle(annotated, (0, 0), (w, 70), (0, 0, 0), -1)

            busy_text = "BUSY" if ocr_busy else "idle"
            decode_ms = float(ocr_stats.get("decode_ms", 0.0) or 0.0)
            vote_count = len(result_buffer)
            line1 = (f"OCR Cam Test | FPS: {fps:.1f} | "
                     f"OCR: {len(display_results)}/{len(memory_results)} "
                     f"| Vote: {vote_count}/3 | Unique: {len(seen_texts)} | "
                     f"{busy_text} {decode_ms:.0f}ms")
            line2 = "q:quit  s:save  r:reset memory"

            cv2.putText(annotated, line1, (12, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(annotated, line2, (12, 54),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # ── 显示 ──
            display = annotated
            if DISPLAY_SCALE != 1.0:
                dw = int(w * DISPLAY_SCALE)
                dh = int(h * DISPLAY_SCALE)
                display = cv2.resize(display, (dw, dh), interpolation=cv2.INTER_AREA)

            cv2.imshow("OCR Camera Test", display)

            # ── 按键 ──
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                os.makedirs("data", exist_ok=True)
                ts = time.strftime("%Y%m%d_%H%M%S")
                path = os.path.join("data", f"ocr_cam_{ts}.png")
                cv2.imwrite(path, annotated)
                print(f"  [✓] 已保存截图: {path}")
            if key == ord("r"):
                ocr_memory.clear()
                seen_texts.clear()
                print("  [*] OCR 记忆已重置")

    except KeyboardInterrupt:
        print("\n[*] 中断退出")
    finally:
        cap.release()
        ocr_worker.stop()
        cv2.destroyAllWindows()

        print()
        print("=" * 60)
        print(f"共识别到 {len(seen_texts)} 个唯一文字:")
        for idx, text in enumerate(sorted(seen_texts), start=1):
            print(f"  {idx}. {text}")
        print("=" * 60)


if __name__ == "__main__":
    main()
