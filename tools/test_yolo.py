"""
YOLO 实时物体检测 — 机器人头部 RealSense RGB 图像
"""

import sys
import time
import base64
import threading
import os

import numpy as np
import cv2
import torch
from ultralytics import YOLO

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot_grasp import config
from robot_grasp.camera_guard import ensure_realsense_ready_on_client
from robot_grasp.depth_utils import compute_grasp_point

try:
    import roslibpy
except ImportError:
    print("缺少 roslibpy 库，请运行: pip install roslibpy")
    sys.exit(1)

# ==================== 配置 ====================

WS_URL = "ws://192.168.20.98:9091"
RGB_TOPIC = "/zj_humanoid/sensor/realsense_head/color/image_raw/compressed"
DEPTH_TOPIC = "/zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw/compressedDepth"
CAMERA_INFO_TOPIC = "/zj_humanoid/sensor/realsense_head/color/camera_info"
# YOLO_MODEL = "models/yolov8n.pt"  # 通用 nano 模型
YOLO_MODEL = "models/best.pt"  # 当前塑料袋模型
YOLO_CONF = 0.50        # 置信度过滤阈值，只显示高于该阈值的检测框
DETECT_EVERY_N_FRAMES = 3  # 每 N 帧做一次 YOLO 推理，降低检测延迟/卡顿
YOLO_DEVICE = "cuda"

# ==================== 全局状态 ====================

latest_frame = None
latest_depth = None
latest_camera_info = None
depth_updated_at = 0.0
frame_lock = threading.Lock()
frame_count = 0
fps = 0.0

# ==================== 回调 ====================

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


def on_depth_message(message):
    global latest_depth, depth_updated_at

    try:
        data_b64 = message.get("data")
        if not data_b64:
            return

        raw = base64.b64decode(data_b64)
        png_start = raw.find(b"\x89PNG\r\n\x1a\n")
        if png_start < 0:
            return

        png = np.frombuffer(raw[png_start:], dtype=np.uint8)
        depth = cv2.imdecode(png, cv2.IMREAD_UNCHANGED)
        if depth is None:
            return
        if depth.dtype != np.uint16:
            depth = depth.astype(np.uint16)

        with frame_lock:
            latest_depth = depth
            depth_updated_at = time.time()
    except Exception:
        pass


def on_camera_info(message):
    global latest_camera_info
    with frame_lock:
        if latest_camera_info is None:
            latest_camera_info = message


def on_connection():
    print("[✓] 已连接到机器人")
    print(f"    话题: {RGB_TOPIC}")
    print(f"    深度: {DEPTH_TOPIC}")
    print(f"    相机: {CAMERA_INFO_TOPIC}")
    print(f"    模型: {YOLO_MODEL}")
    print(f"    置信度阈值: {YOLO_CONF:.2f}")
    print(f"    检测间隔: 每 {DETECT_EVERY_N_FRAMES} 帧")
    print(f"    抓取点比例: x={config.GRASP_POINT_X_RATIO:.2f}, y={config.GRASP_POINT_Y_RATIO:.2f}")
    print(
        "    深度ROI比例: "
        f"x=[{config.DEPTH_ROI_X1_RATIO:.2f},{config.DEPTH_ROI_X2_RATIO:.2f}], "
        f"y=[{config.DEPTH_ROI_Y1_RATIO:.2f},{config.DEPTH_ROI_Y2_RATIO:.2f}]"
    )
    print()


def draw_grasp_debug(annotated, det, p3d):
    roi = p3d.get("depth_roi") if p3d else None
    if roi:
        rx1, ry1, rx2, ry2 = [int(v) for v in roi]
        cv2.rectangle(annotated, (rx1, ry1), (rx2, ry2), (255, 180, 0), 2)
        cv2.putText(
            annotated,
            "depth ROI",
            (rx1, max(18, ry1 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 180, 0),
            1,
        )

    if p3d and p3d.get("point_u") is not None and p3d.get("point_v") is not None:
        u, v = int(p3d["point_u"]), int(p3d["point_v"])
        cv2.drawMarker(
            annotated,
            (u, v),
            (0, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=22,
            thickness=2,
        )
        cv2.circle(annotated, (u, v), 6, (0, 0, 255), 2)
        label = "grasp point"
        if p3d.get("valid"):
            label += f" D:{p3d['depth_mm']}mm"
        cv2.putText(
            annotated,
            label,
            (u + 10, max(18, v - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1,
        )

    if p3d and not p3d.get("valid"):
        x1, _, _, y2 = det["bbox"]
        cv2.putText(
            annotated,
            p3d.get("status", "no depth"),
            (x1, y2 + 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1,
        )


# ==================== 主循环 ====================

def main():
    print("=" * 60)
    print("  YOLO 实时物体检测 — RealSense 头部摄像头")
    print("=" * 60)
    print(f"  WebSocket: {WS_URL}")
    print(f"  模型:      {YOLO_MODEL}")
    print(f"  Conf:      {YOLO_CONF:.2f}")
    print(f"  Device:    {YOLO_DEVICE}")
    print(f"  Detect:    every {DETECT_EVERY_N_FRAMES} frames")
    print("=" * 60)
    print()

    # 连接机器人
    host = WS_URL.replace("ws://", "").split(":")[0]
    port = int(WS_URL.replace("ws://", "").split(":")[1])

    client = roslibpy.Ros(host=host, port=port)
    client.on_ready(on_connection, run_in_thread=False)

    ros_thread = threading.Thread(target=client.run, daemon=True)
    ros_thread.start()

    timeout = 10.0
    start = time.time()
    while not client.is_connected:
        if time.time() - start > timeout:
            print(f"\n[✗] 连接超时")
            client.terminate()
            sys.exit(1)
        time.sleep(0.1)

    ensure_realsense_ready_on_client(
        client,
        require_depth=True,
        require_camera_info=True,
        require_raw_rgb=False,
    )

    # 加载 YOLO 模型
    if YOLO_DEVICE == "cuda" and not torch.cuda.is_available():
        raise RuntimeError(
            "CUDA 不可用，test_yolo 已拒绝退回 CPU。请先确认 nvidia-smi 和 "
            "python -c \"import torch; print(torch.__version__, torch.cuda.is_available())\""
        )
    print("[*] 加载 YOLO 模型...")
    model = YOLO(YOLO_MODEL)
    model.to(YOLO_DEVICE)
    print(f"[✓] 模型加载完成 ({YOLO_MODEL}, device={YOLO_DEVICE})")
    # 预热：推理一次
    model(np.zeros((480, 640, 3), dtype=np.uint8), conf=YOLO_CONF, device=YOLO_DEVICE, verbose=False)
    print()

    subscriber = roslibpy.Topic(client, RGB_TOPIC, "sensor_msgs/CompressedImage")
    depth_subscriber = roslibpy.Topic(client, DEPTH_TOPIC, "sensor_msgs/CompressedImage")
    info_subscriber = roslibpy.Topic(client, CAMERA_INFO_TOPIC, "sensor_msgs/CameraInfo")
    subscriber.subscribe(on_rgb_message)
    depth_subscriber.subscribe(on_depth_message)
    info_subscriber.subscribe(on_camera_info)

    print("[*] 正在接收图像，按 'q' 退出\n")

    prev_time = time.time()
    fps_counter = 0
    fps = 0.0
    infer_times = []
    last_detect_frame = -1
    last_results = None
    last_infer_time = 0.0

    try:
        while True:
            with frame_lock:
                frame = latest_frame.copy() if latest_frame is not None else None
                depth = latest_depth.copy() if latest_depth is not None else None
                camera_info = dict(latest_camera_info) if latest_camera_info is not None else None
                depth_age_ms = (time.time() - depth_updated_at) * 1000 if depth_updated_at else -1
                fc = frame_count

            if frame is not None:
                # FPS 统计
                fps_counter += 1
                now = time.time()
                if now - prev_time >= 1.0:
                    fps = fps_counter / (now - prev_time)
                    fps_counter = 0
                    prev_time = now

                # YOLO 推理：降低检测频率，显示仍使用最新相机帧。
                should_detect = (
                    last_results is None
                    or last_detect_frame < 0
                    or (fc - last_detect_frame) >= DETECT_EVERY_N_FRAMES
                )
                if should_detect:
                    t0 = time.time()
                    last_results = model(frame, conf=YOLO_CONF, device=YOLO_DEVICE, verbose=False)
                    last_infer_time = (time.time() - t0) * 1000  # ms
                    infer_times.append(last_infer_time)
                    if len(infer_times) > 30:
                        infer_times.pop(0)
                    last_detect_frame = fc
                avg_infer = sum(infer_times) / len(infer_times) if infer_times else 0.0

                # 绘制检测结果：在最新相机帧上复用上一轮检测框。
                annotated = frame.copy()
                K = camera_info.get("K", []) if camera_info else []
                fx = K[0] if len(K) >= 9 else 0.0
                fy = K[4] if len(K) >= 9 else 0.0
                cx = K[2] if len(K) >= 9 else 0.0
                cy = K[5] if len(K) >= 9 else 0.0
                for box in last_results[0].boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    label = last_results[0].names.get(cls_id, str(cls_id))
                    det = {
                        "class_id": cls_id,
                        "label": label,
                        "confidence": conf,
                        "bbox": (x1, y1, x2, y2),
                        "center": ((x1 + x2) // 2, (y1 + y2) // 2),
                    }
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        annotated,
                        f"{label} {conf:.2f}",
                        (x1, max(18, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        2,
                    )
                    if depth is not None and fx > 0:
                        p3d = compute_grasp_point(det, depth, fx, fy, cx, cy)
                        draw_grasp_debug(annotated, det, p3d)

                # 叠加信息栏
                h, w = annotated.shape[:2]
                overlay = annotated.copy()
                cv2.rectangle(overlay, (0, 0), (w, 70), (0, 0, 0), -1)
                cv2.addWeighted(overlay, 0.5, annotated, 0.5, 0, annotated)

                det_count = len(last_results[0].boxes)
                cv2.putText(annotated, f"YOLO | {w}x{h}",
                            (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                cv2.putText(annotated, f"FPS: {fps:.1f} | Infer: {avg_infer:.0f}ms | Last: {last_infer_time:.0f}ms | Det: {det_count} | N: {DETECT_EVERY_N_FRAMES}",
                            (12, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
                cv2.putText(
                    annotated,
                    f"Depth age: {depth_age_ms:.0f}ms | ROI full bbox | point ratio: {config.GRASP_POINT_X_RATIO:.2f},{config.GRASP_POINT_Y_RATIO:.2f}",
                    (12, 68),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 180, 0),
                    1,
                )

                cv2.imshow("YOLO Detection - RealSense", annotated)

            key = cv2.waitKey(30) & 0xFF
            if key == ord("q"):
                print("[*] 用户请求退出")
                break
            if not client.is_connected:
                print("[!] 连接已断开")
                break

    except KeyboardInterrupt:
        print("\n[*] 正在退出...")
    finally:
        subscriber.unsubscribe()
        depth_subscriber.unsubscribe()
        info_subscriber.unsubscribe()
        client.terminate()
        cv2.destroyAllWindows()
        print("[✓] 已断开")


if __name__ == "__main__":
    main()
