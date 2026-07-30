#!/usr/bin/env python3
"""
SDK grasp dry run.

Current stage:
1. Move neck to the validated look-down pose.
2. Run the shared vision pipeline to detect the plastic bag.
3. Select a valid grasp target.
4. Print the SDK commands that will be sent later.

This script does not move the arm and does not close the hand.
"""

import argparse
import os
import sys
import time

import cv2

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot_grasp import config
from robot_grasp.grasp_flow import select_grasp_target, summarize_target
from robot_grasp.ros_client import ROSClient
from robot_grasp.sdk_motion_client import SDKMotionClient, load_path_config, movej_by_path_request
from robot_grasp.vision_pipeline import VisionPipeline


def _print_objects(objects):
    if not objects:
        print("  objects: none")
        return
    for obj in objects:
        print(
            f"  #{obj['idx']} {obj['label']} conf={obj['confidence']:.3f} "
            f"valid={obj.get('valid')} "
            f"xyz=({obj.get('x_mm', '')}, {obj.get('y_mm', '')}, {obj.get('z_mm', '')}) "
            f"status={obj.get('status', '')}"
        )


def _print_plan(target: dict, approach_path: dict):
    print("\n" + "=" * 60)
    print("[DRY RUN] 已选中抓取目标")
    print(f"  {summarize_target(target)}")
    print()
    print("[DRY RUN] 现在只打印计划，不发手臂/手掌命令")
    print()
    print("1. 已执行或准备执行：头部低头视觉位")
    print("   /zj_humanoid/upperlimb/movej/neck joints=[0.0, 0.43]")
    print()
    print("2. 下一阶段会发送：右臂安全接近路径 P1 -> P2 -> P3")
    print(f"   service: {approach_path['service']}")
    print(f"   timestamp: {approach_path['timestamp']}")
    for point in approach_path["points"]:
        print(f"   {point['id']}: {point['joint']}")
    print()
    print("3. 视觉目标坐标，当前仍是相机坐标系，不能直接发给 movel")
    print(
        f"   camera xyz mm = "
        f"({target.get('x_mm')}, {target.get('y_mm')}, {target.get('z_mm')})"
    )
    print("   下一步需要补 camera -> TCP/base 的标定转换，再生成 movel 目标")
    print()
    print("4. 预生成 movej_by_path request 结构如下，下一阶段验证后才会启用发送")
    print(f"   {movej_by_path_request(approach_path)}")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=20.0, help="等待有效目标的最长时间")
    parser.add_argument("--preferred-label", default=None, help="优先选择的类别名；默认不按类别过滤")
    parser.add_argument("--skip-neck", action="store_true", help="不执行脖子低头，只跑视觉干跑")
    parser.add_argument("--no-window", action="store_true", help="不打开 OpenCV 窗口")
    parser.add_argument("--print-interval", type=float, default=1.0)
    args = parser.parse_args()

    print("=" * 60)
    print("  SDK 抓取干跑：低头 + 视觉检测 + 选目标 + 打印运动计划")
    print("=" * 60)
    print(f"  WebSocket: {config.WS_URL}")
    print(f"  YOLO 模型: {config.YOLO_MODEL}")
    print(f"  目标类别过滤: {args.preferred_label or '不按类别过滤'}")
    print("  手臂运动: 关闭")
    print("  手掌闭合: 关闭")
    print("=" * 60)

    sdk = SDKMotionClient()
    vision_client = ROSClient()
    pipeline = VisionPipeline(enable_qr=False)
    approach_path = load_path_config("paths/teach_path_right_arm.json")

    try:
        if not args.skip_neck:
            if not sdk.connect():
                raise SystemExit(1)
            print("[*] 关闭 MPC 模式，释放 SDK 控制权...")
            sdk.disable_mpc_mode(required=True)
            print("[*] 调整脖子到低头视觉位...")
            response = sdk.neck_look_down()
            print(f"[neck] {response}")
            time.sleep(0.5)

        if not vision_client.connect():
            raise SystemExit(1)

        print("[*] 开始视觉检测，找到第一个 valid 目标后打印计划")
        last_frame_count = -1
        last_print = 0.0
        start = time.time()
        best_target = None
        fps = 0.0
        fps_count = 0
        fps_start = time.time()

        while time.time() - start < args.timeout:
            rgb, depth, cam_info, fc = vision_client.get_frames()
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

            result = pipeline.process(
                rgb=rgb,
                depth=depth,
                cam_info=cam_info,
                frame_count=fc,
                client_stats=vision_client.get_stats(),
                raw_rgb=None,
                raw_rgb_updated_at=0.0,
                fps=fps,
            )

            target = select_grasp_target(result["object_results"], preferred_label=args.preferred_label)
            if target is not None:
                best_target = target
                _print_plan(best_target, approach_path)
                if not args.no_window:
                    cv2.imshow("SDK Grasp Dry Run", result["annotated"])
                    cv2.waitKey(300)
                return

            if now - last_print >= args.print_interval:
                debug = result["debug"]
                print(
                    f"[DBG] fps={fps:.1f} det={len(result['detections'])} "
                    f"infer={result['avg_infer_ms']:.0f}ms "
                    f"depth_age={debug.get('depth_age_ms', '')} "
                    f"target={summarize_target(target)}"
                )
                _print_objects(result["object_results"])
                last_print = now

            if not args.no_window:
                cv2.imshow("SDK Grasp Dry Run", result["annotated"])
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break

        print("\n[!] 超时，没有选出 valid 抓取目标")
        print("    先检查画面中是否有塑料袋、深度是否 valid、OBJECT_MIN_CONF 是否过高。")

    except KeyboardInterrupt:
        print("\n[*] 中断退出")
    finally:
        pipeline.stop()
        vision_client.disconnect()
        sdk.disconnect()
        cv2.destroyAllWindows()
        print("[✓] dry run 结束")


if __name__ == "__main__":
    main()
