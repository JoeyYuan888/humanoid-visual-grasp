"""
单独 debug 视觉管线。

连接 ROS 后调用 robot_grasp.vision_pipeline.VisionPipeline，打印每个物体
的 3D 坐标和 QR 绑定结果。它和 run_grasp.py 共用同一套核心逻辑。
"""

import argparse
import os
import sys
import time

import cv2

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot_grasp import config
from robot_grasp.ros_client import ROSClient
from robot_grasp.vision_pipeline import VisionPipeline


def _print_objects(objects):
    if not objects:
        print("objects: none")
        return
    print("objects:")
    for obj in objects:
        print(
            f"  #{obj['idx']} {obj['label']} conf={obj['confidence']:.3f} "
            f"qr={obj.get('qr_text', '')} "
            f"valid={obj.get('valid')} "
            f"xyz=({obj.get('x_mm', '')}, {obj.get('y_mm', '')}, {obj.get('z_mm', '')}) "
            f"status={obj.get('status', '')}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws-url", default=config.WS_URL, help="rosbridge WebSocket URL")
    parser.add_argument("--no-window", action="store_true", help="只打印结果，不打开 OpenCV 窗口")
    parser.add_argument("--print-interval", type=float, default=1.0)
    args = parser.parse_args()

    pipeline = VisionPipeline()
    if pipeline.backend_info():
        print(f"[*] QR backends: {pipeline.backend_info()}")

    client = ROSClient(ws_url=args.ws_url)
    if not client.connect():
        raise SystemExit(1)

    print("[*] VisionPipeline debug: q/Ctrl+C 退出")
    last_frame_count = -1
    last_print = 0.0
    fps = 0.0
    fps_count = 0
    fps_start = time.time()
    try:
        while True:
            rgb, depth, cam_info, fc = client.get_frames()
            if rgb is None or fc == last_frame_count:
                key = cv2.waitKey(5) & 0xFF
                if key == ord("q"):
                    break
                continue
            last_frame_count = fc

            fps_count += 1
            now = time.time()
            if now - fps_start >= 1.0:
                fps = fps_count / (now - fps_start)
                fps_count = 0
                fps_start = now

            raw_rgb, _, raw_rgb_updated_at = client.get_raw_rgb()
            result = pipeline.process(
                rgb=rgb,
                depth=depth,
                cam_info=cam_info,
                frame_count=fc,
                client_stats=client.get_stats(),
                raw_rgb=raw_rgb,
                raw_rgb_updated_at=raw_rgb_updated_at,
                fps=fps,
            )

            if now - last_print >= args.print_interval:
                debug = result["debug"]
                print(
                    f"[DBG] fps={fps:.1f} det={len(result['detections'])} "
                    f"infer={result['avg_infer_ms']:.0f}ms "
                    f"grasp={debug.get('grasp_status', '')} "
                    f"qr={debug.get('qr_live_count', '')}/{debug.get('qr_memory_count', '')} "
                    f"qr_text={debug.get('qr_texts', '')}"
                )
                _print_objects(result["object_results"])
                last_print = now

            if not args.no_window:
                cv2.imshow("VisionPipeline Debug", result["annotated"])
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
    except KeyboardInterrupt:
        print("\n[*] 中断退出")
    finally:
        pipeline.stop()
        client.disconnect()
        cv2.destroyAllWindows()
        print("[✓] 已断开连接")


if __name__ == "__main__":
    main()
