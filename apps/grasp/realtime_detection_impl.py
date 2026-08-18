"""
主程序 — 机器人视觉抓取系统。

这里保持为运行编排层：
- ROS 连接与取帧
- 调用 VisionPipeline 产出 object_results
- 显示、点击测距、CSV 记录
"""

import os
import sys
import time
import argparse

import cv2

from robot_grasp.common import config
from robot_grasp.common.logger import DataLogger
from robot_grasp.common.ros_client import ROSClient
from robot_grasp.vision.depth_utils import get_depth_at, pixel_to_3d
from robot_grasp.vision.vision_pipeline import VisionPipeline
from robot_grasp.vision.visualizer import draw_click_info, draw_crosshair, set_click_point

_logger = DataLogger()
_latest_depth = None
_latest_fx = _latest_fy = _latest_cx = _latest_cy = 0


def _mouse_callback(event, x, y, flags, param):
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    set_click_point(x, y)
    d = get_depth_at(_latest_depth, x, y) if _latest_depth is not None else None
    if d and _latest_fx > 0:
        x3d, y3d, z3d = pixel_to_3d(x, y, d, _latest_fx, _latest_fy, _latest_cx, _latest_cy)
        _logger.log_click(x, y, d, x3d, y3d, z3d)
        print(f"  [点击] ({x},{y}) -> ({x3d:.0f}, {y3d:.0f}, {z3d:.0f}) mm, D={d:.0f}mm")
    else:
        _logger.log_click(x, y, None, None, None, None)
        print(f"  [点击] ({x},{y}) -> no depth")


def _print_header(ws_url: str):
    print("=" * 60)
    print("  机器人视觉抓取系统（实时检测）")
    print("=" * 60)
    print(f"  WebSocket: {ws_url}")
    print(f"  YOLO 模型: {config.YOLO_MODEL}")
    print(f"  目标类别: {config.YOLO_TARGET_CLASSES}")
    print(f"  深度订阅: {'开启' if config.ENABLE_DEPTH else '关闭'}")
    print(f"  QR 识别: {'开启' if config.ENABLE_QR else '关闭'}")
    print("=" * 60)
    print()


def main():
    global _latest_depth, _latest_fx, _latest_fy, _latest_cx, _latest_cy

    parser = argparse.ArgumentParser()
    parser.add_argument("--ws-url", default=config.WS_URL, help="rosbridge WebSocket URL")
    args = parser.parse_args()

    _print_header(args.ws_url)
    pipeline = VisionPipeline()
    if pipeline.backend_info():
        print(f"[*] QR backends: {pipeline.backend_info()}")
    print()

    client = ROSClient(ws_url=args.ws_url)
    if not client.connect():
        print("[✗] 无法连接机器人，退出")
        sys.exit(1)
    print()

    print("[*] 鼠标左键=测距 | s=保存 | q=退出\n")

    prev_time = time.time()
    fps_counter = 0
    fps = 0.0
    last_frame_count = -1
    last_perf_time = time.time()
    last_perf_stats = client.get_stats()
    sample_display_frames = 0
    sample_detects = 0
    last_status_time = time.time()
    last_result = None

    cv2.namedWindow("Robot Grasp - RealSense", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Robot Grasp - RealSense", _mouse_callback)

    try:
        while True:
            rgb, depth, cam_info, fc = client.get_frames()

            if rgb is None or fc == last_frame_count:
                key = cv2.waitKey(5) & 0xFF
                if key == ord("q"):
                    break
                if not client.is_connected:
                    print("[!] 连接已断开")
                    break
                continue
            last_frame_count = fc

            _latest_depth = depth
            if cam_info and "K" in cam_info:
                K = cam_info["K"]
                _latest_fx, _latest_fy, _latest_cx, _latest_cy = K[0], K[4], K[2], K[5]

            fps_counter += 1
            sample_display_frames += 1
            now = time.time()
            if now - prev_time >= 1.0:
                fps = fps_counter / (now - prev_time)
                fps_counter = 0
                prev_time = now

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
            last_result = result
            if result["should_detect"]:
                sample_detects += 1

            for qr_det in result["qr_new_texts"]:
                roi_index = qr_det.get("roi_index")
                roi_text = f"roi{roi_index}" if roi_index is not None else qr_det.get("source", "")
                print(f"  [QR {len(pipeline.seen_qr_texts)}] {qr_det['text']} ({qr_det.get('backend', '')}, {roi_text})")

            annotated = result["annotated"]
            if config.ENABLE_DEPTH and config.SHOW_DEPTH_OVERLAYS:
                annotated = draw_crosshair(annotated, _latest_depth,
                                           _latest_fx, _latest_fy, _latest_cx, _latest_cy)
                annotated = draw_click_info(annotated, _latest_depth,
                                            _latest_fx, _latest_fy, _latest_cx, _latest_cy)
            cv2.imshow("Robot Grasp - RealSense", annotated)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                data_dir = os.path.join(os.path.dirname(__file__), "..", "data", "samples")
                _logger.save(data_dir)

            if now - last_perf_time >= config.PERF_LOG_INTERVAL_SEC:
                sample_time = time.time()
                stats = client.get_stats()
                elapsed = sample_time - last_perf_time
                rgb_rx_fps = (stats["rgb_count"] - last_perf_stats["rgb_count"]) / elapsed
                raw_rgb_age_ms = 0.0
                if stats.get("raw_rgb_updated_at", 0) > 0:
                    raw_rgb_age_ms = max(0.0, (sample_time - stats["raw_rgb_updated_at"]) * 1000)
                depth_msg_rx_fps = (stats["depth_msg_count"] - last_perf_stats["depth_msg_count"]) / elapsed
                depth_rx_fps = (stats["depth_count"] - last_perf_stats["depth_count"]) / elapsed
                detect_fps = sample_detects / elapsed
                display_sample_fps = sample_display_frames / elapsed
                depth_age_ms = 0.0
                if stats["depth_updated_at"] > 0:
                    depth_age_ms = max(0.0, (sample_time - stats["depth_updated_at"]) * 1000)

                debug = result["debug"]
                perf_sample = {
                    **debug,
                    "display_fps": display_sample_fps,
                    "rgb_rx_fps": rgb_rx_fps,
                    "depth_msg_rx_fps": depth_msg_rx_fps,
                    "depth_rx_fps": depth_rx_fps,
                    "detect_fps": detect_fps,
                    "infer_ms": result["avg_infer_ms"],
                    "last_infer_ms": result["last_infer_ms"],
                    "det_count": len(result["detections"]),
                    "depth_age_ms": depth_age_ms,
                    "raw_rgb_age_ms": raw_rgb_age_ms,
                }
                _logger.log_perf(fc, perf_sample)

                if sample_time - last_status_time >= 2.0:
                    print(
                        f"  [性能] display={display_sample_fps:.1f}fps "
                        f"rgb={rgb_rx_fps:.1f}fps depth_msg={depth_msg_rx_fps:.1f}fps "
                        f"depth={depth_rx_fps:.1f}fps detect={detect_fps:.1f}fps "
                        f"infer={result['avg_infer_ms']:.0f}ms depth_age={depth_age_ms:.0f}ms "
                        f"det={len(result['detections'])} raw_age={raw_rgb_age_ms:.0f}ms "
                        f"classes={debug.get('det_summary', '')} "
                        f"grasp={debug.get('grasp_status', '')} "
                        f"samples={debug.get('grasp_sample_count', '')} "
                        f"qr_ms={debug.get('qr_decode_ms', 0):.0f} "
                        f"qr_src={debug.get('qr_source', '')} "
                        f"qr_mode={debug.get('qr_scan_mode', '')} "
                        f"qr_rois={debug.get('qr_scan_rois', '')} "
                        f"qr={debug.get('qr_live_count', '')}/{debug.get('qr_memory_count', '')} "
                        f"qr_text={debug.get('qr_texts', '')} "
                        f"roi={debug.get('roi_valid_count', '')}"
                    )
                    last_status_time = sample_time

                last_perf_time = sample_time
                last_perf_stats = stats
                sample_display_frames = 0
                sample_detects = 0

            if not client.is_connected:
                print("[!] 连接已断开")
                break

    except KeyboardInterrupt:
        print("\n[*] 中断退出")
    finally:
        pipeline.stop()
        client.disconnect()
        cv2.destroyAllWindows()
        print("[✓] 已断开连接")
        data_dir = os.path.join(os.path.dirname(__file__), "..", "data", "samples")
        _logger.save(data_dir)
        if last_result is not None:
            print("[*] 最后一帧 object_results:")
            for obj in last_result["object_results"]:
                print(
                    f"    #{obj['idx']} {obj['label']} conf={obj['confidence']:.3f} "
                    f"qr={obj.get('qr_text', '')} "
                    f"xyz=({obj.get('x_mm', '')}, {obj.get('y_mm', '')}, {obj.get('z_mm', '')}) "
                    f"status={obj.get('status', '')}"
                )
        print()


if __name__ == "__main__":
    main()
