"""
测试机器人 RealSense 关键话题是否通联
验证 README 中整理的 3 个必需话题:
  1. color/image_raw/compressed    — RGB 彩色图
  2. aligned_depth_to_color/image_raw — 对齐深度图
  3. color/camera_info             — 相机内参
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

WS_URL = "ws://192.168.20.98:9090"

TOPICS = {
    "rgb": "/zj_humanoid/sensor/realsense_head/color/image_raw/compressed",
    "depth": "/zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw",
    "camera_info": "/zj_humanoid/sensor/realsense_head/color/camera_info",
}

TIMEOUT = 10.0  # 等待数据超时（秒）

# ==================== 状态 ====================

results = {
    "rgb": {"ok": False, "count": 0},
    "depth": {"ok": False, "count": 0},
    "camera_info": {"ok": False, "count": 0},
}
lock = threading.Lock()


# ==================== 回调 ====================

def on_rgb(message):
    with lock:
        results["rgb"]["count"] += 1
        if not results["rgb"]["ok"] and message.get("data"):
            results["rgb"]["ok"] = True
            print(f"  [✓] RGB 彩色图  — 已收到数据 ({len(message['data'])} bytes)")


def on_depth(message):
    with lock:
        results["depth"]["count"] += 1
        if not results["depth"]["ok"] and message.get("data"):
            height = message.get("height", "?")
            width = message.get("width", "?")
            encoding = message.get("encoding", "?")
            results["depth"]["ok"] = True
            print(f"  [✓] 对齐深度图 — 已收到数据 ({width}x{height}, {encoding})")


def on_camera_info(message):
    with lock:
        results["camera_info"]["count"] += 1
        if not results["camera_info"]["ok"]:
            h = message.get("height", "?")
            w = message.get("width", "?")
            K = message.get("K", [])
            results["camera_info"]["ok"] = True
            print(f"  [✓] 相机内参   — 已收到 (分辨率 {w}x{h})")
            if K:
                print(f"      fx={K[0]:.4f}, fy={K[4]:.4f}, cx={K[2]:.4f}, cy={K[5]:.4f}")


# ==================== 主逻辑 ====================

def main():
    print("=" * 60)
    print("  机器人 RealSense 话题通联测试")
    print("=" * 60)
    print(f"  WebSocket: {WS_URL}")
    print(f"  测试话题:")
    for k, v in TOPICS.items():
        print(f"    - {k:12s}: {v}")
    print("=" * 60)
    print()

    # 解析主机和端口
    host = WS_URL.replace("ws://", "").split(":")[0]
    port = int(WS_URL.replace("ws://", "").split(":")[1])

    # 创建 ROS bridge 客户端
    client = roslibpy.Ros(host=host, port=port)

    # 后台线程运行 ROS 事件循环
    print("[*] 正在连接机器人...")
    ros_thread = threading.Thread(target=client.run, daemon=True)
    ros_thread.start()

    # 等待连接
    start = time.time()
    while not client.is_connected:
        if time.time() - start > TIMEOUT:
            print(f"\n[✗] 连接超时 (>{TIMEOUT}s)")
            print("  请检查: 机器人是否开机? rosbridge 是否运行? IP 是否正确?")
            sys.exit(1)
        time.sleep(0.1)

    print("[✓] 已连接，正在订阅话题（等待数据...）\n")

    # 订阅 RGB
    sub_rgb = roslibpy.Topic(client, TOPICS["rgb"], "sensor_msgs/CompressedImage")
    sub_rgb.subscribe(on_rgb)

    # 订阅深度
    sub_depth = roslibpy.Topic(client, TOPICS["depth"], "sensor_msgs/Image")
    sub_depth.subscribe(on_depth)

    # 订阅相机信息
    sub_info = roslibpy.Topic(client, TOPICS["camera_info"], "sensor_msgs/CameraInfo")
    sub_info.subscribe(on_camera_info)

    # 等待足够的数据到达（最长 TIMEOUT 秒）
    deadline = time.time() + TIMEOUT
    while time.time() < deadline:
        with lock:
            all_ok = all(v["ok"] for v in results.values())
        if all_ok:
            break
        time.sleep(0.1)

    print()
    print("=" * 60)
    print("  测试结果")
    print("=" * 60)

    all_ok = True
    for name in ["rgb", "depth", "camera_info"]:
        r = results[name]
        status = "✅ 通过" if r["ok"] else "❌ 失败"
        if not r["ok"]:
            all_ok = False
        print(f"  {status}  {name:12s} — 收到 {r['count']} 条消息")

    print("=" * 60)
    if all_ok:
        print("  🎉 所有必需话题均正常通联！")
    else:
        print("  ⚠️  部分话题未收到数据，请检查机器人端配置")

    # 清理
    sub_rgb.unsubscribe()
    sub_depth.unsubscribe()
    sub_info.unsubscribe()
    client.terminate()
    print()


if __name__ == "__main__":
    main()
