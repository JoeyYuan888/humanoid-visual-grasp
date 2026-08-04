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
import json
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
from robot_grasp.qr_detector import QRDetector, draw_qr_detections

try:
    import roslibpy
except ImportError:
    print("缺少 roslibpy 库，请运行: pip install roslibpy")
    sys.exit(1)


DEFAULT_OUTPUT = os.path.join(PROJECT_ROOT, "data", "post_grasp_qr_latest.json")


def _parse_ws_url(ws_url: str) -> tuple[str, int]:
    stripped = ws_url.replace("ws://", "").replace("wss://", "")
    host, port = stripped.split(":")
    return host, int(port)


class RGBSubscriber:
    def __init__(self):
        self.frame = None
        self.seq = 0
        self.lock = threading.Lock()

    def callback(self, message):
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

    def latest(self):
        with self.lock:
            if self.frame is None:
                return None, self.seq
            return self.frame.copy(), self.seq


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


def _save_result(output: str, detections: list[dict], frame_seq: int) -> None:
    payload = {
        "created_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws-url", default=config.WS_URL)
    parser.add_argument("--rgb-topic", default=config.TOPIC_RGB)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--decode-interval", type=float, default=0.4)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--show-window", action="store_true")
    parser.add_argument("--save-snapshot", action="store_true")
    args = parser.parse_args()

    print("=" * 70)
    print("  Post-grasp QR scan")
    print("=" * 70)
    print(f"  WebSocket: {args.ws_url}")
    print(f"  RGB topic: {args.rgb_topic}")
    print(f"  Duration: {args.duration:.1f}s")
    print(f"  Output: {args.output}")
    print("=" * 70)

    detector = QRDetector()
    print(f"[*] QR backends: {', '.join(detector.backends)}")

    client = _connect(args.ws_url)
    print("[✓] 已连接 rosbridge")

    subscriber = RGBSubscriber()
    topic = roslibpy.Topic(client, args.rgb_topic, "sensor_msgs/CompressedImage")
    topic.subscribe(subscriber.callback)

    latest_detections = []
    latest_seq = 0
    last_decode = 0.0
    start = time.time()
    fps_start = start
    frames = 0

    try:
        while time.time() - start < args.duration:
            frame, seq = subscriber.latest()
            if frame is None:
                time.sleep(0.02)
                continue

            frames += 1
            now = time.time()
            if now - last_decode >= args.decode_interval or not latest_detections:
                last_decode = now
                latest_detections = detector.detect(frame, heavy=True, search_full=True)
                latest_seq = seq
                if latest_detections:
                    texts = "; ".join(det["text"] for det in latest_detections)
                    print(f"[QR] {texts}")
                    _save_result(args.output, latest_detections, latest_seq)
                    if args.save_snapshot:
                        snap = draw_qr_detections(frame, latest_detections)
                        snap_path = os.path.join(
                            PROJECT_ROOT,
                            "data",
                            f"post_grasp_qr_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png",
                        )
                        cv2.imwrite(snap_path, snap)
                        print(f"[✓] 已保存截图: {snap_path}")
                    break

            if args.show_window:
                annotated = draw_qr_detections(frame, latest_detections)
                elapsed = max(1e-6, now - fps_start)
                fps = frames / elapsed
                cv2.putText(
                    annotated,
                    f"Post-grasp QR | fps={fps:.1f} | q=quit | QR={len(latest_detections)}",
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
            else:
                time.sleep(0.02)

        if not latest_detections:
            _save_result(args.output, [], latest_seq)
            print("[!] 未识别到 QR")
        else:
            print(f"[✓] 已保存 QR 结果: {args.output}")
    finally:
        try:
            topic.unsubscribe()
        except Exception:
            pass
        try:
            client.terminate()
        except Exception:
            pass
        if args.show_window:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
