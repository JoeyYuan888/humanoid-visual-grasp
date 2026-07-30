"""
二维码识别测试 — 机器人头部 RealSense RGB 图像。

用法:
    python tools/test_qr.py

窗口中:
    q 退出
    s 保存当前标注画面到 data/qr_snapshot_*.png
    r 保存当前 ROI 裁剪图到 data/qr_roi_*.png
"""

import base64
import csv
import os
import sys
import threading
import time
from datetime import datetime

import cv2
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot_grasp import config
from robot_grasp.detector import Detector
from robot_grasp.qr_detector import draw_qr_detections
from robot_grasp.qr_worker import QRWorker

try:
    import roslibpy
except ImportError:
    print("缺少 roslibpy 库，请运行: pip install roslibpy")
    sys.exit(1)


QR_OBJECT_CLASSES = ["cup", "bottle"]
QR_OBJECT_CONF = 0.10
DETECT_EVERY_N_FRAMES = 5
QR_DECODE_INTERVAL_SEC = 1.0
RAW_RGB_THROTTLE_MS = 3000
QR_MEMORY_TTL_SEC = 10.0
# QR 解码只使用 YOLO 给出的目标 ROI；后续塑料袋应改成 segmentation mask 区域。

latest_frame = None
latest_raw_frame = None
latest_raw_time = 0.0
frame_count = 0
raw_frame_count = 0
frame_lock = threading.Lock()
raw_frame_lock = threading.Lock()


def on_rgb_message(message):
    global latest_frame, frame_count

    try:
        data_b64 = message.get("data")
        if not data_b64:
            return

        img_bytes = base64.b64decode(data_b64)
        img_array = np.frombuffer(img_bytes, dtype=np.uint8)
        frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        if frame is None:
            return

        with frame_lock:
            latest_frame = frame
            frame_count += 1
    except Exception:
        pass


def on_raw_rgb_message(message):
    global latest_raw_frame, latest_raw_time, raw_frame_count

    try:
        height = message.get("height", 0)
        width = message.get("width", 0)
        encoding = message.get("encoding", "")
        data_b64 = message.get("data")
        if not data_b64 or height == 0 or width == 0:
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

        with raw_frame_lock:
            latest_raw_frame = frame
            latest_raw_time = time.time()
            raw_frame_count += 1
    except Exception:
        pass


def on_connection():
    print("[✓] 已连接到机器人")
    print(f"    RGB: {config.TOPIC_RGB}")
    print(f"    RAW: {config.TOPIC_RGB_RAW} throttle={RAW_RGB_THROTTLE_MS}ms")
    print()


def _safe_terminate(client):
    try:
        client.terminate()
    except Exception:
        pass


def _save_snapshot(frame, prefix="qr_snapshot"):
    os.makedirs("data", exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join("data", f"{prefix}_{timestamp}.png")
    cv2.imwrite(path, frame)
    print(f"[✓] 已保存截图: {path}")


def _save_rois(frame, rois, prefix="qr_roi"):
    os.makedirs("data", exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    h, w = frame.shape[:2]
    for idx, (x1, y1, x2, y2) in enumerate(rois):
        x1, x2 = max(0, x1), min(w, x2)
        y1, y2 = max(0, y1), min(h, y2)
        if x2 <= x1 or y2 <= y1:
            continue
        path = os.path.join("data", f"{prefix}_{timestamp}_roi{idx}.png")
        cv2.imwrite(path, frame[y1:y2, x1:x2])
        print(f"[✓] 已保存 ROI: {path}")


def _save_debug_rois(frame, raw_frame, object_rois):
    _save_rois(frame, object_rois, prefix="qr_tight_compressed")
    if raw_frame is not None:
        raw_tight = _scale_rois(object_rois, frame.shape, raw_frame.shape)
        _save_rois(raw_frame, raw_tight, prefix="qr_tight_raw")


def _scale_rois(rois, src_shape, dst_shape):
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


def _roi_summary(rois):
    return ";".join(f"{x1},{y1},{x2},{y2}" for x1, y1, x2, y2 in rois)


def _detection_summary(detections):
    items = []
    for det in detections:
        x1, y1, x2, y2 = det["bbox"]
        items.append(f"{det['label']}:{det['confidence']:.3f}@{x1},{y1},{x2},{y2}")
    return ";".join(items)


def _qr_summary(detections):
    items = []
    for det in detections:
        backend = det.get("backend", "unknown")
        source = det.get("source", "unknown")
        roi_index = det.get("roi_index")
        roi_text = f"roi{roi_index}" if roi_index is not None else source
        items.append(f"{backend}/{roi_text}:{det['text']}")
    return ";".join(items)


def _merge_qr_memory(memory, detections, now):
    for det in detections:
        text = det["text"]
        memory[text] = {
            "det": dict(det),
            "last_seen": now,
        }

    stale = [
        text
        for text, item in memory.items()
        if now - item["last_seen"] > QR_MEMORY_TTL_SEC
    ]
    for text in stale:
        del memory[text]

    return [item["det"] for item in memory.values()]


def main():
    print("=" * 60)
    print("  QR Code Test — RealSense RGB")
    print("=" * 60)
    print(f"  WebSocket: {config.WS_URL}")
    print(f"  RGB Topic: {config.TOPIC_RGB}")
    print("=" * 60)
    print()

    host = config.WS_URL.replace("ws://", "").split(":")[0]
    port = int(config.WS_URL.replace("ws://", "").split(":")[1])

    client = roslibpy.Ros(host=host, port=port)
    client.on_ready(on_connection, run_in_thread=False)

    ros_thread = threading.Thread(target=client.run, daemon=True)
    ros_thread.start()

    start = time.time()
    while not client.is_connected:
        if time.time() - start > config.CONNECT_TIMEOUT:
            print(f"[✗] 连接超时: {config.WS_URL}")
            _safe_terminate(client)
            sys.exit(1)
        time.sleep(0.1)

    raw_subscriber = None
    subscriber = roslibpy.Topic(client, config.TOPIC_RGB, "sensor_msgs/CompressedImage")
    subscriber.subscribe(on_rgb_message)
    try:
        raw_subscriber = roslibpy.Topic(
            client,
            config.TOPIC_RGB_RAW,
            "sensor_msgs/Image",
            throttle_rate=RAW_RGB_THROTTLE_MS,
            queue_length=1,
        )
    except TypeError:
        raw_subscriber = roslibpy.Topic(client, config.TOPIC_RGB_RAW, "sensor_msgs/Image")
    raw_subscriber.subscribe(on_raw_rgb_message)

    object_detector = Detector(target_classes=QR_OBJECT_CLASSES, conf=QR_OBJECT_CONF, imgsz=512)
    qr_worker = QRWorker()
    os.makedirs("data", exist_ok=True)
    diag_path = os.path.join("data", f"qr_debug_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    diag_file = open(diag_path, "w", newline="", encoding="utf-8")
    diag_writer = csv.DictWriter(diag_file, fieldnames=[
        "time",
        "frame_count",
        "fps",
        "rgb_rx_count",
        "raw_rx_count",
        "raw_age_ms",
        "object_count",
        "object_max_conf",
        "object_detections",
        "roi_count",
        "roi_fallback",
        "rois",
        "qr_submit",
        "qr_busy",
        "qr_source_current",
        "qr_source_last_decode",
        "qr_decode_ms",
        "qr_result_count",
        "qr_memory_count",
        "qr_unique_count",
        "qr_results",
        "qr_memory_results",
        "qr_error",
    ])
    diag_writer.writeheader()
    seen_texts: set[str] = set()
    qr_memory = {}
    fps = 0.0
    prev_time = time.time()
    fps_counter = 0
    last_frame_count = -1
    annotated = None
    last_object_rois = []
    object_detections = []
    last_qr_detections = []
    last_qr_decode_time = 0.0
    qr_busy = False
    qr_source = "compressed"
    qr_stats = {}
    last_diag_print = 0.0

    print(f"[*] 正在识别物体上的小二维码：先检测 {QR_OBJECT_CLASSES}，再放大 ROI 解码。")
    print(f"[*] YOLO ROI confidence threshold: {QR_OBJECT_CONF}")
    print(f"[*] YOLO every {DETECT_EVERY_N_FRAMES} frames | QR decode every {QR_DECODE_INTERVAL_SEC:.1f}s")
    print(f"[*] QR memory TTL: {QR_MEMORY_TTL_SEC:.1f}s")
    print(f"[*] QR backends: {qr_worker.backend_info()}")
    print(f"[*] debug CSV: {diag_path}")
    print("[*] q=退出 | s=保存当前截图 | r=保存当前 ROI\n")

    try:
        while True:
            with frame_lock:
                frame = latest_frame.copy() if latest_frame is not None else None
                fc = frame_count

            if frame is None or fc == last_frame_count:
                key = cv2.waitKey(5) & 0xFF
                if key == ord("q"):
                    break
                continue
            last_frame_count = fc

            fps_counter += 1
            now = time.time()
            if now - prev_time >= 1.0:
                fps = fps_counter / (now - prev_time)
                fps_counter = 0
                prev_time = now

            if fc % DETECT_EVERY_N_FRAMES == 0 or not object_detections:
                object_detections, _ = object_detector.detect(frame)
            object_rois = [det["bbox"] for det in object_detections]
            roi_fallback = False
            if not object_rois:
                h0, w0 = frame.shape[:2]
                object_rois.append((w0 // 4, h0 // 4, 3 * w0 // 4, 3 * h0 // 4))
                roi_fallback = True
            last_object_rois = object_rois
            qr_frame = frame
            qr_rois = object_rois
            qr_source = "compressed"
            with raw_frame_lock:
                raw_frame = latest_raw_frame.copy() if latest_raw_frame is not None else None
                raw_count = raw_frame_count
                raw_time = latest_raw_time
            if raw_frame is not None:
                qr_frame = raw_frame
                qr_rois = _scale_rois(object_rois, frame.shape, raw_frame.shape)
                qr_source = "raw"

            qr_submit = False
            if now - last_qr_decode_time >= QR_DECODE_INTERVAL_SEC:
                if qr_worker.submit(qr_frame, qr_rois, qr_source):
                    last_qr_decode_time = now
                    qr_submit = True
            last_qr_detections, qr_busy, qr_stats = qr_worker.latest()
            detections = last_qr_detections
            memory_detections = _merge_qr_memory(qr_memory, detections, now)
            annotated = frame.copy()

            for idx, det in enumerate(object_detections, start=1):
                x1, y1, x2, y2 = det["bbox"]
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 128, 0), 2)
                cv2.putText(annotated, f"{det['label']} {idx} {det['confidence']:.2f}",
                            (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (255, 128, 0), 2)

            annotated = draw_qr_detections(annotated, memory_detections)

            for det in detections:
                text = det["text"]
                if text not in seen_texts:
                    seen_texts.add(text)
                    backend = det.get("backend", "unknown")
                    source = det.get("source", "unknown")
                    roi_index = det.get("roi_index")
                    roi_text = f"roi{roi_index}" if roi_index is not None else source
                    print(f"[QR {len(seen_texts)}] {text}  ({backend}, {roi_text})")

            h, w = annotated.shape[:2]
            cv2.rectangle(annotated, (0, 0), (w, 64), (0, 0, 0), -1)
            busy_text = "busy" if qr_busy else "idle"
            decode_ms = float(qr_stats.get("decode_ms", 0.0) or 0.0)
            cv2.putText(annotated, f"QR Test | FPS: {fps:.1f} | Objects: {len(object_detections)} | QR: {len(detections)}/{len(memory_detections)} | Unique: {len(seen_texts)} | QR {busy_text} {qr_source} {decode_ms:.0f}ms",
                        (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            cv2.putText(annotated, "q: quit | s: save snapshot | r: save ROI",
                        (12, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)

            raw_age_ms = (now - raw_time) * 1000.0 if raw_time > 0 else -1.0
            object_max_conf = max((det["confidence"] for det in object_detections), default=0.0)
            diag_writer.writerow({
                "time": f"{now:.3f}",
                "frame_count": fc,
                "fps": f"{fps:.2f}",
                "rgb_rx_count": fc,
                "raw_rx_count": raw_count,
                "raw_age_ms": f"{raw_age_ms:.1f}",
                "object_count": len(object_detections),
                "object_max_conf": f"{object_max_conf:.3f}",
                "object_detections": _detection_summary(object_detections),
                "roi_count": len(object_rois),
                "roi_fallback": int(roi_fallback),
                "rois": _roi_summary(object_rois),
                "qr_submit": int(qr_submit),
                "qr_busy": int(qr_busy),
                "qr_source_current": qr_source,
                "qr_source_last_decode": qr_stats.get("source", ""),
                "qr_decode_ms": f"{decode_ms:.1f}",
                "qr_result_count": len(detections),
                "qr_memory_count": len(memory_detections),
                "qr_unique_count": len(seen_texts),
                "qr_results": _qr_summary(detections),
                "qr_memory_results": _qr_summary(memory_detections),
                "qr_error": qr_stats.get("error", ""),
            })
            if now - last_diag_print >= 2.0:
                last_diag_print = now
                print(
                    "[DBG] "
                    f"fps={fps:.1f} objects={len(object_detections)} "
                    f"max_conf={object_max_conf:.2f} rois={len(object_rois)} "
                    f"fallback={int(roi_fallback)} raw_age_ms={raw_age_ms:.0f} "
                    f"qr_busy={int(qr_busy)} qr_ms={decode_ms:.0f} qr={len(detections)}/{len(memory_detections)} "
                    f"roi_xy={_roi_summary(object_rois)}"
                )

            cv2.imshow("QR Code Test - RealSense", annotated)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s") and annotated is not None:
                _save_snapshot(annotated)
            if key == ord("r") and frame is not None:
                _save_debug_rois(frame, raw_frame, last_object_rois)

            if not client.is_connected:
                print("[!] 连接已断开")
                break
    except KeyboardInterrupt:
        print("\n[*] 中断退出")
    finally:
        try:
            subscriber.unsubscribe()
        except Exception:
            pass
        if raw_subscriber is not None:
            try:
                raw_subscriber.unsubscribe()
            except Exception:
                pass
        try:
            qr_worker.stop()
        except Exception:
            pass
        try:
            diag_file.close()
            print(f"[✓] debug CSV 已保存: {diag_path}")
        except Exception:
            pass
        _safe_terminate(client)
        cv2.destroyAllWindows()

        print()
        print("=" * 60)
        print(f"共识别到 {len(seen_texts)} 个唯一二维码:")
        for idx, text in enumerate(sorted(seen_texts), start=1):
            print(f"  {idx}. {text}")
        print("=" * 60)


if __name__ == "__main__":
    main()
