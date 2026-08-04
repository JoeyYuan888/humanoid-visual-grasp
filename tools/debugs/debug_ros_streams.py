"""
单独 debug ROS 相机/深度流。

只连接 rosbridge 并打印 RGB、raw RGB、depth 的接收频率和延迟；
不加载 YOLO，不跑 QR，适合先判断卡顿是不是来自话题订阅/rosbridge。
"""

import os
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot_grasp.ros_client import ROSClient


def main():
    client = ROSClient()
    if not client.connect():
        raise SystemExit(1)

    print("[*] ROS stream debug: Ctrl+C 退出")
    last = client.get_stats()
    last_time = time.time()
    try:
        while True:
            time.sleep(1.0)
            now = time.time()
            stats = client.get_stats()
            elapsed = max(1e-6, now - last_time)
            rgb_fps = (stats["rgb_count"] - last["rgb_count"]) / elapsed
            raw_fps = (stats["raw_rgb_count"] - last["raw_rgb_count"]) / elapsed
            depth_msg_fps = (stats["depth_msg_count"] - last["depth_msg_count"]) / elapsed
            depth_fps = (stats["depth_count"] - last["depth_count"]) / elapsed
            rgb_age = (now - stats["rgb_updated_at"]) * 1000 if stats["rgb_updated_at"] else -1
            raw_age = (now - stats["raw_rgb_updated_at"]) * 1000 if stats["raw_rgb_updated_at"] else -1
            depth_age = (now - stats["depth_updated_at"]) * 1000 if stats["depth_updated_at"] else -1
            print(
                f"rgb={rgb_fps:.1f}fps age={rgb_age:.0f}ms | "
                f"raw={raw_fps:.1f}fps age={raw_age:.0f}ms | "
                f"depth_msg={depth_msg_fps:.1f}fps depth={depth_fps:.1f}fps age={depth_age:.0f}ms"
            )
            last = stats
            last_time = now
    except KeyboardInterrupt:
        print("\n[*] 中断退出")
    finally:
        client.disconnect()
        print("[✓] 已断开连接")


if __name__ == "__main__":
    main()
