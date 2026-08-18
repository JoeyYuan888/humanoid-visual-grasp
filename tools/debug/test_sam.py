#!/usr/bin/env python3
"""FastSAM snapshot test.

Camera mode:
  1. Enable MPC mode.
  2. Move the neck down once.
  3. Capture one compressed RGB frame.
  4. Run FastSAM on the full frame.
  5. Save candidate masks/overlays, then move the neck home.

This is a debug tool only. It does not send arm/hand motion commands.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import sys
import tempfile
import threading
import time
from pathlib import Path
from urllib.parse import urlparse

os.environ.setdefault("MPLCONFIGDIR", os.path.join(tempfile.gettempdir(), "matplotlib"))

import cv2
import numpy as np
import roslibpy
from twisted.internet import error as twisted_error
from twisted.python import failure as twisted_failure
from twisted.python import log as twisted_log
from ultralytics import YOLO


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot_grasp.common import config  # noqa: E402


DEFAULT_MODEL = PROJECT_ROOT / "models" / "yolo" / "FastSAM-s.pt"
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "data" / "runtime" / "sam_debug_latest"
MPC_MODE_SERVICE = "/wa/wa_hardware_interface/mpc_mode_setting"
NECK_SERVICE = "/wa/wa_hardware_interface/neck_movej"
JOINT_STATES_TOPIC = "/zj_humanoid/upperlimb/joint_states"


def _suppress_roslibpy_shutdown_noise() -> None:
    original_err = twisted_log.err

    def quiet_err(_stuff=None, _why=None, **kwargs):
        failure_obj = None
        if isinstance(_stuff, twisted_failure.Failure):
            failure_obj = _stuff
        elif _stuff is None:
            failure_obj = twisted_failure.Failure()
        if failure_obj is not None and failure_obj.check(twisted_error.ReactorNotRunning):
            return None
        return original_err(_stuff, _why, **kwargs)

    twisted_log.err = quiet_err


_suppress_roslibpy_shutdown_noise()


class CompressedRGBSubscriber:
    def __init__(self, client: roslibpy.Ros, rgb_topic: str, throttle_ms: int) -> None:
        self.topic = roslibpy.Topic(
            client,
            rgb_topic,
            "sensor_msgs/CompressedImage",
            throttle_rate=throttle_ms,
            queue_length=1,
        )
        self.lock = threading.Lock()
        self.latest: np.ndarray | None = None
        self.count = 0

    def subscribe(self) -> None:
        self.topic.subscribe(self._on_rgb)

    def _on_rgb(self, msg: dict) -> None:
        try:
            payload = base64.b64decode(msg["data"])
            arr = np.frombuffer(payload, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except Exception:
            return
        if frame is None:
            return
        with self.lock:
            self.latest = frame
            self.count += 1

    def get_latest(self) -> tuple[np.ndarray | None, int]:
        with self.lock:
            return None if self.latest is None else self.latest.copy(), self.count

    def close(self) -> None:
        try:
            self.topic.unsubscribe()
        except Exception:
            pass


def _connect_ros(ws_url: str, timeout: float) -> roslibpy.Ros:
    parsed = urlparse(ws_url)
    if parsed.scheme not in {"ws", "wss"} or not parsed.hostname or not parsed.port:
        raise ValueError(f"非法 ws-url: {ws_url}")
    client = roslibpy.Ros(host=parsed.hostname, port=parsed.port)
    client.run(timeout=timeout)
    if not client.is_connected:
        raise RuntimeError(f"连接 rosbridge 失败: {ws_url}")
    return client


def _service_type(client: roslibpy.Ros, service_name: str) -> str:
    rosapi = roslibpy.Service(client, "/rosapi/service_type", "rosapi/ServiceType")
    response = rosapi.call(roslibpy.ServiceRequest({"service": service_name}))
    srv_type = response.get("type", "")
    if not srv_type:
        raise RuntimeError(f"service unavailable or unknown type: {service_name}")
    return srv_type


def _call_service(client: roslibpy.Ros, service_name: str, request: dict) -> dict:
    srv_type = _service_type(client, service_name)
    service = roslibpy.Service(client, service_name, srv_type)
    return service.call(roslibpy.ServiceRequest(request))


def _set_mpc_mode(client: roslibpy.Ros, enabled: bool) -> None:
    response = _call_service(client, MPC_MODE_SERVICE, {"data": bool(enabled)})
    print(f"[mpc_mode={enabled}] {response}")
    if response and response.get("success") is False:
        raise RuntimeError(f"MPC mode 设置失败: {response}")


def _wait_for_neck_state(client: roslibpy.Ros, timeout: float = 2.0) -> tuple[float, float] | None:
    latest: dict[str, tuple[float, float]] = {}

    def callback(message):
        names = message.get("name", [])
        positions = message.get("position", [])
        try:
            latest["state"] = (
                float(positions[names.index("Neck_Z")]),
                float(positions[names.index("Neck_Y")]),
            )
        except (ValueError, IndexError, TypeError):
            return

    sub = roslibpy.Topic(client, JOINT_STATES_TOPIC, "sensor_msgs/JointState")
    sub.subscribe(callback)
    deadline = time.time() + timeout
    while time.time() < deadline and "state" not in latest:
        time.sleep(0.05)
    try:
        sub.unsubscribe()
    except Exception:
        pass
    return latest.get("state")


def _move_neck(
    client: roslibpy.Ros,
    neck_z: float,
    neck_y: float,
    duration: float,
    *,
    verify: bool,
    tolerance: float,
) -> None:
    print(f"[动作] neck_movej: z={neck_z:.3f}, y={neck_y:.3f}, t={duration:.1f}s")
    response = _call_service(
        client,
        NECK_SERVICE,
        {"neck_joint": [float(neck_z), float(neck_y)], "t": float(duration)},
    )
    if response and response.get("success") is False:
        raise RuntimeError(f"neck_movej 返回失败: {response}")

    time.sleep(duration + 0.3)
    if not verify:
        return
    after = _wait_for_neck_state(client, timeout=2.0)
    if after is None:
        print(f"[!] 未读到 {JOINT_STATES_TOPIC}，无法确认 neck 是否到位")
        return
    err_z = abs(after[0] - neck_z)
    err_y = abs(after[1] - neck_y)
    print(f"[neck] actual=({after[0]:.3f},{after[1]:.3f}) err=({err_z:.3f},{err_y:.3f})")
    if err_z > tolerance or err_y > tolerance:
        raise RuntimeError(
            f"neck_movej 调用完成但关节未到目标: target=({neck_z:.3f},{neck_y:.3f}), "
            f"actual=({after[0]:.3f},{after[1]:.3f})"
        )


def _ensure_clean_output_dir(path: Path, keep: set[Path] | None = None) -> None:
    path.mkdir(parents=True, exist_ok=True)
    keep_resolved = {item.resolve() for item in keep or set()}
    for child in path.iterdir():
        if child.is_file() and child.suffix.lower() in {".png", ".jpg", ".jpeg", ".json"}:
            if child.resolve() in keep_resolved:
                continue
            child.unlink()


def _capture_one_frame_on_client(client: roslibpy.Ros, args: argparse.Namespace) -> np.ndarray:
    subscriber = CompressedRGBSubscriber(client, args.rgb_topic, args.rgb_throttle_ms)
    subscriber.subscribe()
    print(f"[动作] 等待 RGB 快照: {args.rgb_topic}")
    try:
        deadline = time.time() + args.frame_timeout
        last_count = -1
        while time.time() < deadline:
            frame, count = subscriber.get_latest()
            if frame is not None and count != last_count:
                print(f"[动作] RGB 快照获取完成 count={count}")
                return frame
            time.sleep(0.03)
        raise RuntimeError(f"{args.frame_timeout:.1f}s 内没有收到 RGB 快照")
    finally:
        subscriber.close()


def _mask_bbox(mask: np.ndarray) -> tuple[int, int, int, int] | None:
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return None
    return int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())


def _candidate_from_mask(index: int, raw_mask: np.ndarray, frame: np.ndarray, conf: float | None) -> dict | None:
    h, w = frame.shape[:2]
    mask = (raw_mask > 0.5).astype(np.uint8)
    if mask.shape != (h, w):
        mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
    bbox = _mask_bbox(mask)
    if bbox is None:
        return None

    area = int(mask.sum())
    x1, y1, x2, y2 = bbox
    center_dist = float(np.hypot(((x1 + x2) / 2.0) - (w / 2.0), ((y1 + y2) / 2.0) - (h * 0.55)))
    return {
        "index": index,
        "conf": None if conf is None else float(conf),
        "bbox": [x1, y1, x2, y2],
        "center": [float((x1 + x2) / 2.0), float((y1 + y2) / 2.0)],
        "area_px": area,
        "area_frac": float(area / (h * w)),
        "center_dist": center_dist,
        "_mask": mask,
    }


def _rank_candidates(candidates: list[dict]) -> list[dict]:
    def score(item: dict) -> float:
        area = item["area_frac"]
        area_score = -abs(area - 0.08) * 8.0
        center_score = -item["center_dist"] / 500.0
        conf_score = float(item["conf"] or 0.0)
        return area_score + center_score + conf_score

    return sorted(candidates, key=score, reverse=True)


def _draw_candidates(frame: np.ndarray, candidates: list[dict], max_draw: int) -> np.ndarray:
    out = frame.copy()
    colors = [
        (0, 255, 0),
        (0, 180, 255),
        (255, 0, 0),
        (255, 0, 255),
        (0, 255, 255),
        (180, 255, 0),
        (255, 180, 0),
        (180, 0, 255),
    ]
    for rank, item in enumerate(candidates[:max_draw], start=1):
        color = np.array(colors[(rank - 1) % len(colors)], dtype=np.uint8)
        mask = item["_mask"].astype(bool)
        tint = np.full_like(out, color)
        blended = cv2.addWeighted(out, 0.68, tint, 0.32, 0)
        out[mask] = blended[mask]
        x1, y1, x2, y2 = item["bbox"]
        cv2.rectangle(out, (x1, y1), (x2, y2), tuple(int(v) for v in color), 2)
        label = f"#{rank} area={item['area_frac']:.3f}"
        if item["conf"] is not None:
            label += f" conf={item['conf']:.2f}"
        cv2.putText(out, label, (x1, max(22, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, tuple(int(v) for v in color), 2)
    cv2.circle(out, (frame.shape[1] // 2, int(frame.shape[0] * 0.55)), 8, (0, 0, 255), -1)
    return out


def _save_candidate_images(output_dir: Path, frame: np.ndarray, candidates: list[dict], max_save: int) -> None:
    cv2.imwrite(str(output_dir / "rgb.png"), frame)
    cv2.imwrite(str(output_dir / "sam_candidates.png"), _draw_candidates(frame, candidates, max_save))

    report = []
    for rank, item in enumerate(candidates[:max_save], start=1):
        mask = (item["_mask"] * 255).astype(np.uint8)
        candidate_overlay = frame.copy()
        mask_bool = item["_mask"].astype(bool)
        blended = cv2.addWeighted(candidate_overlay, 0.65, np.full_like(candidate_overlay, (0, 255, 0)), 0.35, 0)
        candidate_overlay[mask_bool] = blended[mask_bool]
        cv2.imwrite(str(output_dir / f"mask_{rank:02d}.png"), mask)
        cv2.imwrite(str(output_dir / f"candidate_{rank:02d}.png"), candidate_overlay)
        clean = {k: v for k, v in item.items() if k != "_mask"}
        clean["rank"] = rank
        report.append(clean)

    (output_dir / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def _run_sam(model: YOLO, frame: np.ndarray, args: argparse.Namespace) -> list[dict]:
    result = model.predict(
        source=frame,
        imgsz=args.imgsz,
        conf=args.conf,
        iou=args.iou,
        device=args.device,
        verbose=False,
        retina_masks=True,
    )[0]
    if result.masks is None:
        return []

    masks = result.masks.data.detach().cpu().numpy()
    confs = result.boxes.conf.detach().cpu().numpy() if result.boxes is not None else [None] * len(masks)
    candidates = []
    for i, raw_mask in enumerate(masks):
        item = _candidate_from_mask(i, raw_mask, frame, None if i >= len(confs) else float(confs[i]))
        if item is None:
            continue
        if item["area_frac"] < args.min_area_frac or item["area_frac"] > args.max_area_frac:
            continue
        candidates.append(item)
    return _rank_candidates(candidates)


def _process_one_frame(model: YOLO, frame: np.ndarray, args: argparse.Namespace, output_dir: Path) -> np.ndarray:
    keep = {Path(args.image)} if args.image else set()
    _ensure_clean_output_dir(output_dir, keep=keep)
    candidates = _run_sam(model, frame, args)
    _save_candidate_images(output_dir, frame, candidates, args.max_masks)
    if candidates:
        top = candidates[0]
        print(
            f"[SAM] masks={len(candidates)} best area={top['area_frac']:.3f} "
            f"bbox={top['bbox']} center=({top['center'][0]:.1f},{top['center'][1]:.1f})"
        )
    else:
        print("[SAM] masks=0")
    return _draw_candidates(frame, candidates, args.max_masks)


def _run_image(args: argparse.Namespace) -> None:
    frame = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError(f"无法读取图片: {args.image}")
    model = YOLO(str(args.model))
    annotated = _process_one_frame(model, frame, args, Path(args.output_dir))
    print(f"[✓] 输出: {args.output_dir}")
    if args.show_window:
        cv2.imshow("SAM snapshot test", annotated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def _run_camera_snapshot(args: argparse.Namespace) -> None:
    control_client: roslibpy.Ros | None = None
    try:
        control_client = _connect_ros(args.ws_url, args.connect_timeout)
        if not args.skip_mpc_mode:
            _set_mpc_mode(control_client, True)
        if not args.skip_neck_down:
            _move_neck(
                control_client,
                args.neck_down_z,
                args.neck_down_y,
                args.neck_time,
                verify=not args.no_neck_verify,
                tolerance=args.neck_verify_tolerance,
            )
        frame = _capture_one_frame_on_client(control_client, args)
        model = YOLO(str(args.model))
        annotated = _process_one_frame(model, frame, args, Path(args.output_dir))
        print(f"[✓] 输出: {args.output_dir}")
        if args.show_window:
            cv2.imshow("SAM snapshot test", annotated)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    finally:
        if control_client is not None and not args.skip_neck_home:
            try:
                _move_neck(
                    control_client,
                    args.neck_home_z,
                    args.neck_home_y,
                    args.neck_time,
                    verify=not args.no_neck_verify,
                    tolerance=args.neck_verify_tolerance,
                )
            except Exception as exc:
                print(f"[!] 抬头失败: {exc}")
        if control_client is not None:
            try:
                control_client.terminate()
            except Exception:
                pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="FastSAM snapshot segmentation test")
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument("--rgb-topic", default=config.TOPIC_RGB)
    parser.add_argument("--rgb-throttle-ms", type=int, default=0)
    parser.add_argument("--connect-timeout", type=float, default=10.0)
    parser.add_argument("--frame-timeout", type=float, default=8.0)
    parser.add_argument("--image", type=Path, help="本地图片路径；指定后不连接机器人")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--imgsz", type=int, default=1024)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.70)
    parser.add_argument("--min-area-frac", type=float, default=0.003)
    parser.add_argument("--max-area-frac", type=float, default=0.60)
    parser.add_argument("--max-masks", type=int, default=10)
    parser.add_argument("--show-window", action="store_true")
    parser.add_argument("--neck-down-z", type=float, default=0.0)
    parser.add_argument("--neck-down-y", type=float, default=0.35)
    parser.add_argument("--neck-home-z", type=float, default=0.0)
    parser.add_argument("--neck-home-y", type=float, default=0.0)
    parser.add_argument("--neck-time", type=float, default=4.0)
    parser.add_argument("--neck-verify-tolerance", type=float, default=0.08)
    parser.add_argument("--skip-neck-down", action="store_true")
    parser.add_argument("--skip-neck-home", action="store_true")
    parser.add_argument("--skip-mpc-mode", action="store_true")
    parser.add_argument("--no-neck-verify", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not Path(args.model).exists():
        raise RuntimeError(f"SAM 模型不存在: {args.model}")
    if args.image:
        _run_image(args)
    else:
        _run_camera_snapshot(args)


if __name__ == "__main__":
    main()
