"""
机器人 RealSense 摄像头图像读取 Demo
通过 rosbridge WebSocket 连接到机器人，订阅 Realsense 图像话题并显示。

依赖安装:
    pip install roslibpy opencv-python numpy

用法:
    python tools/debug/robot_camera_demo.py
"""

import sys
import time
import base64
import threading

import numpy as np
import cv2

try:
    import roslibpy
except ImportError:
    print("缺少 roslibpy 库，请运行: pip install roslibpy")
    sys.exit(1)

# ==================== 配置 ====================

WS_URL = "ws://192.168.20.102:9090"
IMAGE_TOPIC = "/zj_humanoid/sensor/realsense_head/color/image_raw/compressed"

# ==================== 全局状态 ====================

latest_frame = None
frame_lock = threading.Lock()
frame_count = 0
fps = 0.0

# ==================== 回调 ====================

def on_image_message(message):
    """处理接收到的压缩图像消息（在 ROS 线程中调用）。"""
    global latest_frame, frame_count

    try:
        fmt = message.get('format', 'jpeg')
        data_b64 = message.get('data')

        if not data_b64:
            return

        # base64 解码 -> numpy 数组 -> OpenCV 解码
        img_bytes = base64.b64decode(data_b64)
        img_array = np.frombuffer(img_bytes, dtype=np.uint8)
        frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

        if frame is None:
            return

        # 线程安全地保存最新帧
        with frame_lock:
            latest_frame = frame
            frame_count += 1

    except Exception:
        pass


def on_connection():
    """连接建立后的回调。"""
    print("[✓] 已连接到机器人 WebSocket 服务器")
    print(f"    URL: {WS_URL}")
    print(f"    订阅话题: {IMAGE_TOPIC}")
    print()


# ==================== 显示循环 ====================

def display_loop(client):
    """主线程中的 OpenCV 显示循环。"""
    global fps

    print("[*] 正在接收图像数据... (在窗口中按 'q' 退出)\n")

    prev_time = time.time()
    fps_counter = 0

    while True:
        # 获取最新帧
        with frame_lock:
            frame = latest_frame.copy() if latest_frame is not None else None
            fc = frame_count

        if frame is not None:
            # 计算 FPS
            fps_counter += 1
            now = time.time()
            if now - prev_time >= 1.0:
                fps = fps_counter / (now - prev_time)
                fps_counter = 0
                prev_time = now

            # 在图像上叠加信息
            info_frame = frame.copy()
            h, w = info_frame.shape[:2]

            # 画半透明信息栏
            overlay = info_frame.copy()
            cv2.rectangle(overlay, (0, 0), (w, 60), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.5, info_frame, 0.5, 0, info_frame)

            cv2.putText(info_frame, f"RealSense Camera | {w}x{h}",
                        (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv2.putText(info_frame, f"FPS: {fps:.1f} | Frame: {fc}",
                        (12, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)

            cv2.imshow('Robot RealSense Camera', info_frame)

        # 按 'q' 退出
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            print("[*] 用户请求退出")
            break

        # 检查 ROS 客户端是否还连着
        if not client.is_connected:
            print("[!] 与机器人的连接已断开")
            break

    client.terminate()
    cv2.destroyAllWindows()


# ==================== 主逻辑 ====================

def main():
    print("=" * 60)
    print("  机器人 RealSense 摄像头图像读取 Demo")
    print("=" * 60)
    print(f"  WebSocket: {WS_URL}")
    print(f"  话题:      {IMAGE_TOPIC}")
    print("=" * 60)
    print()

    # 解析主机和端口
    host = WS_URL.replace("ws://", "").split(":")[0]
    port = int(WS_URL.replace("ws://", "").split(":")[1])

    # 创建 ROS bridge 客户端
    client = roslibpy.Ros(host=host, port=port)
    client.on_ready(on_connection, run_in_thread=False)

    # 在后台线程中运行 ROS 事件循环
    print("[*] 正在连接到机器人 WebSocket 服务器...")
    ros_thread = threading.Thread(target=client.run, daemon=True)
    ros_thread.start()

    # 等待连接建立（最多 10 秒）
    timeout = 10.0
    start = time.time()
    while not client.is_connected:
        if time.time() - start > timeout:
            print(f"\n[✗] 连接超时，无法连接到 {WS_URL}")
            print("    请检查:")
            print("      1. 机器人是否已开机并联网")
            print("      2. rosbridge 服务是否正在运行 (roslaunch rosbridge_server rosbridge_websocket.launch)")
            print("      3. IP 地址和端口是否正确")
            client.terminate()
            sys.exit(1)
        time.sleep(0.1)

    # 订阅图像话题
    subscriber = roslibpy.Topic(client, IMAGE_TOPIC, "sensor_msgs/CompressedImage")
    subscriber.subscribe(on_image_message)

    # 主线程显示循环
    try:
        display_loop(client)
    except KeyboardInterrupt:
        print("\n[*] 正在退出...")
    finally:
        subscriber.unsubscribe()
        client.terminate()
        cv2.destroyAllWindows()
        print("[✓] 已断开连接")


if __name__ == "__main__":
    main()
