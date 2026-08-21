#!/usr/bin/env python3
"""Track one plastic crate from aligned RealSense RGB-D over rosbridge."""

from __future__ import annotations

import argparse
import base64
from collections import deque
import json
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any

import cv2
import numpy as np
import trimesh
import websocket

from dual_grasp_points import draw_dual_grasp_points, find_dual_grasp_points
from make_bootstrap_mask import build_color_mask, build_mask, load_mask_config
from run_single import draw_closed_polyline, load_mesh, project


ROOT = Path(__file__).resolve().parent
VENDOR = ROOT / "vendor" / "FoundationPose"
DEFAULT_WS_URL = "ws://192.168.20.102:9091"
DEFAULT_COLOR_TOPIC = "/zj_humanoid/sensor/realsense_head/color/image_raw/compressed"
DEFAULT_DEPTH_TOPIC = "/zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw/compressedDepth"
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def message_stamp(message: dict[str, Any]) -> float:
    stamp = message.get("header", {}).get("stamp", {})
    seconds = stamp.get("secs", stamp.get("sec", 0))
    nanoseconds = stamp.get("nsecs", stamp.get("nanosec", 0))
    return float(seconds) + float(nanoseconds) * 1e-9


def message_bytes(message: dict[str, Any]) -> bytes:
    data = message.get("data")
    if not data:
        raise ValueError("compressed image has no data")
    return base64.b64decode(data) if isinstance(data, str) else bytes(data)


def decode_color(message: dict[str, Any]) -> np.ndarray:
    image = cv2.imdecode(np.frombuffer(message_bytes(message), np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("could not decode compressed color image")
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def decode_depth(message: dict[str, Any]) -> np.ndarray:
    payload = message_bytes(message)
    offset = payload.find(PNG_SIGNATURE)
    if offset < 0:
        raise ValueError("compressedDepth payload has no PNG signature")
    depth = cv2.imdecode(np.frombuffer(payload[offset:], np.uint8), cv2.IMREAD_UNCHANGED)
    if depth is None or depth.dtype != np.uint16:
        raise ValueError("aligned depth is not a 16-bit PNG")
    return depth


class RosbridgeRgbd:
    def __init__(self, url: str, color_topic: str, depth_topic: str) -> None:
        self.url = url
        self.color_topic = color_topic
        self.depth_topic = depth_topic
        self.colors: deque[tuple[float, np.ndarray]] = deque(maxlen=8)
        self.depths: deque[tuple[float, np.ndarray]] = deque(maxlen=8)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.error: Exception | None = None
        self.socket: websocket.WebSocket | None = None
        self.thread: threading.Thread | None = None

    def start(self) -> None:
        self.socket = websocket.create_connection(self.url, timeout=10, http_proxy_host=None)
        self.socket.settimeout(1.0)
        for identity, topic in (("crate_color", self.color_topic), ("crate_depth", self.depth_topic)):
            self.socket.send(json.dumps({
                "op": "subscribe",
                "id": identity,
                "topic": topic,
                "type": "sensor_msgs/CompressedImage",
                "queue_length": 1,
            }))
        self.thread = threading.Thread(target=self._receive, name="rosbridge-rgbd", daemon=True)
        self.thread.start()

    def _receive(self) -> None:
        assert self.socket is not None
        while not self.stop_event.is_set():
            try:
                packet = json.loads(self.socket.recv())
                if packet.get("op") != "publish":
                    continue
                topic = packet.get("topic")
                message = packet["msg"]
                stamp = message_stamp(message)
                if topic == self.color_topic:
                    decoded = decode_color(message)
                    target = self.colors
                elif topic == self.depth_topic:
                    decoded = decode_depth(message)
                    target = self.depths
                else:
                    continue
                with self.lock:
                    target.append((stamp, decoded))
            except websocket.WebSocketTimeoutException:
                continue
            except Exception as exc:
                if not self.stop_event.is_set():
                    self.error = exc
                return

    def next_pair(
        self,
        after_stamp: float,
        timeout: float = 10.0,
        max_delta: float = 0.010,
    ) -> tuple[float, float, np.ndarray, np.ndarray]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.error is not None:
                raise RuntimeError(f"rosbridge receiver stopped: {self.error}")
            with self.lock:
                candidates = [item for item in self.colors if item[0] > after_stamp]
                depths = list(self.depths)
            best: tuple[float, float, np.ndarray, np.ndarray] | None = None
            best_delta = float("inf")
            for color_stamp, color in candidates:
                for depth_stamp, depth in depths:
                    delta = abs(color_stamp - depth_stamp)
                    if delta < best_delta:
                        best = (color_stamp, depth_stamp, color, depth)
                        best_delta = delta
            if best is not None and best_delta <= max_delta:
                return best
            time.sleep(0.005)
        raise TimeoutError("timed out waiting for synchronized RGB/aligned-depth")

    def close(self) -> None:
        self.stop_event.set()
        if self.socket is not None:
            for identity, topic in (("crate_color", self.color_topic), ("crate_depth", self.depth_topic)):
                try:
                    self.socket.send(json.dumps({"op": "unsubscribe", "id": identity, "topic": topic}))
                except Exception:
                    pass
            self.socket.close()
        if self.thread is not None:
            self.thread.join(timeout=2)


def render(
    rgb: np.ndarray,
    pose: np.ndarray,
    camera_matrix: np.ndarray,
    metadata: dict[str, Any],
    grasp_points: dict[str, Any],
) -> np.ndarray:
    overlay = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    outer = project(np.asarray(metadata["rim_outer_m"]), pose, camera_matrix)
    inner = project(np.asarray(metadata["rim_inner_m"]), pose, camera_matrix)
    draw_closed_polyline(overlay, outer, (0, 0, 255))
    draw_closed_polyline(overlay, inner, (0, 255, 0))
    position = pose[:3, 3]
    cv2.putText(
        overlay,
        f"xyz camera: {position[0]:+.3f} {position[1]:+.3f} {position[2]:+.3f} m",
        (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 0), 2, cv2.LINE_AA,
    )
    if grasp_points.get("points_left_to_right"):
        draw_dual_grasp_points(overlay, grasp_points)
    if grasp_points.get("valid"):
        left, right = grasp_points["points_left_to_right"]
        cv2.putText(
            overlay,
            "left:  {:+.3f} {:+.3f} {:+.3f}m occ={} {}".format(
                *left["point_camera_m"], int(left["occluded"]), left["source"],
            ),
            (20, 67), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 0, 255), 2, cv2.LINE_AA,
        )
        cv2.putText(
            overlay,
            "right: {:+.3f} {:+.3f} {:+.3f}m occ={} {}".format(
                *right["point_camera_m"], int(right["occluded"]), right["source"],
            ),
            (20, 96), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 0), 2, cv2.LINE_AA,
        )
        if not grasp_points["robot_execution_allowed"]:
            cv2.putText(
                overlay,
                "COORDINATE VALID - GRASP POINT OCCLUDED",
                (20, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 165, 255), 2, cv2.LINE_AA,
            )
    else:
        reasons = ",".join(grasp_points.get("reason_codes", []))[:90]
        cv2.putText(
            overlay,
            f"DUAL GRASP INVALID: {reasons}",
            (20, 67), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 255), 2, cv2.LINE_AA,
        )
    return overlay


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    temporary = path.with_name(f"{path.stem}.tmp{path.suffix}")
    temporary.write_text(
        json.dumps(value, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def append_valid_grasp_history(output_dir: Path, result: dict[str, Any]) -> None:
    if not result.get("valid"):
        return
    history_path = output_dir / "valid_grasp_points_history.jsonl"
    with history_path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(result, ensure_ascii=False) + "\n")


def invalid_grasp_result(reason: str, sequence: int) -> dict[str, Any]:
    return {
        "valid": False,
        "coordinate_valid": False,
        "any_grasp_point_occluded": False,
        "all_grasp_points_clear": False,
        "robot_execution_allowed": False,
        "coordinate_frame": "realsense_head_color_optical_frame",
        "point_order": ["image_left", "image_right"],
        "points_left_to_right": [],
        "reason_codes": [reason],
        "sequence": sequence,
        "published_at_unix_sec": time.time(),
    }


def publish_grasp_points(output_dir: Path, result: dict[str, Any]) -> None:
    """Publish coordinates and per-point occlusion using atomic files."""
    atomic_json(output_dir / "latest_grasp_points.json", result)
    if result.get("valid"):
        append_valid_grasp_history(output_dir, result)
        # Kept for post-run inspection only.  Robot code must consume the
        # fail-closed latest files below, never this diagnostic snapshot.
        atomic_json(output_dir / "last_valid_grasp_points_DIAGNOSTIC_ONLY.json", result)
    matrix = np.full((2, 3), np.nan, dtype=np.float64)
    if result.get("valid"):
        matrix = np.asarray(
            [point["point_camera_m"] for point in result["points_left_to_right"]],
            dtype=np.float64,
        )
    temporary = output_dir / "latest_grasp_points.tmp.txt"
    np.savetxt(temporary, matrix, fmt="%.9f")
    os.replace(temporary, output_dir / "latest_grasp_points.txt")

    occlusion = np.full(2, -1, dtype=np.int32)
    if len(result.get("points_left_to_right", [])) == 2:
        for index, point in enumerate(result["points_left_to_right"]):
            if point.get("occluded") is not None:
                occlusion[index] = int(point["occluded"])
    temporary_occlusion = output_dir / "latest_grasp_occlusion.tmp.txt"
    np.savetxt(temporary_occlusion, occlusion, fmt="%d")
    os.replace(temporary_occlusion, output_dir / "latest_grasp_occlusion.txt")

    combined = np.column_stack((matrix, occlusion))
    temporary_combined = output_dir / "latest_grasp_points_with_occlusion.tmp.txt"
    np.savetxt(temporary_combined, combined, fmt=("%.9f", "%.9f", "%.9f", "%d"))
    os.replace(
        temporary_combined,
        output_dir / "latest_grasp_points_with_occlusion.txt",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ws-url", default=DEFAULT_WS_URL)
    parser.add_argument("--color-topic", default=DEFAULT_COLOR_TOPIC)
    parser.add_argument("--depth-topic", default=DEFAULT_DEPTH_TOPIC)
    parser.add_argument("--camera-matrix", type=Path, default=ROOT / "config" / "cam_K.txt")
    parser.add_argument("--mesh", type=Path, default=ROOT / "assets" / "plastic_crate_m.obj")
    parser.add_argument("--metadata", type=Path, default=ROOT / "assets" / "crate_metadata.json")
    parser.add_argument(
        "--mask-config", type=Path, default=ROOT / "config" / "bootstrap_mask.json",
        help="full-frame colour-mask configuration used only for first-frame registration",
    )
    parser.add_argument(
        "--mask-profile",
        help="override active_profile in the mask JSON, for example: blue, green, red",
    )
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "live")
    parser.add_argument("--register-iteration", type=int, default=5)
    parser.add_argument("--track-iteration", type=int, default=2)
    parser.add_argument("--max-frames", type=int, default=0, help="0 runs until Ctrl-C")
    parser.add_argument("--save-every", type=int, default=10)
    parser.add_argument("--grasp-depth-tolerance", type=float, default=0.08)
    parser.add_argument("--grasp-max-pixel-offset", type=float, default=12.0)
    parser.add_argument("--grasp-center-color-min-ratio", type=float, default=0.25)
    parser.add_argument("--grasp-prediction-min-inliers", type=int, default=20)
    parser.add_argument("--grasp-prediction-min-edge-span", type=float, default=0.20)
    parser.add_argument("--grasp-prediction-max-rmse", type=float, default=0.012)
    parser.add_argument("--show", action="store_true", help="show an OpenCV window (requires a display)")
    parser.add_argument(
        "--mark-not-running-on-exit",
        action="store_true",
        help="overwrite latest_grasp_points.json with PROCESS_NOT_RUNNING when exiting",
    )
    args = parser.parse_args()

    mask_config = load_mask_config(args.mask_config)

    os.chdir(VENDOR)
    sys.path.insert(0, str(VENDOR))
    from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor, dr, set_logging_format, set_seed

    set_logging_format()
    set_seed(0)
    camera_matrix = np.loadtxt(args.camera_matrix).reshape(3, 3)
    metadata = json.loads(args.metadata.read_text(encoding="utf-8"))
    mesh: trimesh.Trimesh = load_mesh(args.mesh)
    symmetry = np.repeat(np.eye(4, dtype=np.float32)[None], 2, axis=0)
    symmetry[1, 0, 0] = symmetry[1, 1, 1] = -1
    args.output_dir.mkdir(parents=True, exist_ok=True)
    history_path = args.output_dir / "valid_grasp_points_history.jsonl"
    try:
        history_path.unlink()
    except FileNotFoundError:
        pass
    publish_grasp_points(
        args.output_dir,
        invalid_grasp_result("PROCESS_INITIALIZING", -1),
    )

    estimator = FoundationPose(
        model_pts=mesh.vertices,
        model_normals=mesh.vertex_normals,
        symmetry_tfs=symmetry,
        mesh=mesh,
        scorer=ScorePredictor(),
        refiner=PoseRefinePredictor(),
        glctx=dr.RasterizeCudaContext(),
        debug=0,
        debug_dir=str(args.output_dir / "debug"),
    )

    source = RosbridgeRgbd(args.ws_url, args.color_topic, args.depth_topic)
    source.start()
    print(f"connected: {args.ws_url}")
    previous_stamp = -1.0
    frame_index = 0
    try:
        while args.max_frames <= 0 or frame_index < args.max_frames:
            color_stamp, depth_stamp, rgb, depth_mm = source.next_pair(previous_stamp)
            previous_stamp = color_stamp
            depth = depth_mm.astype(np.float32) * 0.001
            depth[(depth < 0.001) | (depth > 10.0)] = 0
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            crate_color_mask, active_color_profile = build_color_mask(
                bgr, mask_config, args.mask_profile,
            )
            if frame_index == 0:
                # Always retain the actual initialization inputs.  If automatic
                # masking fails because the crate moved or illumination changed,
                # these files make the failure directly inspectable.
                cv2.imwrite(
                    str(args.output_dir / "initial_rgb.png"),
                    bgr,
                )
                cv2.imwrite(str(args.output_dir / "initial_depth_mm.png"), depth_mm)
                mask, stats = build_mask(
                    cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                    mask_config,
                    args.mask_profile,
                )
                cv2.imwrite(str(args.output_dir / "initial_mask.png"), mask)
                pose = estimator.register(
                    K=camera_matrix, rgb=rgb, depth=depth,
                    ob_mask=mask > 0, iteration=args.register_iteration,
                )
                print(
                    f"initialized mask profile={stats['profile']} area={stats['area']} "
                    f"bbox=({stats['x']},{stats['y']},{stats['width']},{stats['height']})"
                )
                if stats["small_component_warning"]:
                    print("warning: initialization component is small; continuing without an area lockout")
            else:
                pose = estimator.track_one(
                    rgb=rgb, depth=depth, K=camera_matrix,
                    iteration=args.track_iteration,
                )
            if not np.isfinite(pose).all():
                raise RuntimeError("tracker returned non-finite pose")

            try:
                grasp_points = find_dual_grasp_points(
                    depth,
                    crate_color_mask,
                    camera_matrix,
                    pose,
                    metadata,
                    depth_tolerance_m=args.grasp_depth_tolerance,
                    max_sample_offset_pixels=args.grasp_max_pixel_offset,
                    center_color_min_ratio=args.grasp_center_color_min_ratio,
                    prediction_min_inliers=args.grasp_prediction_min_inliers,
                    prediction_min_edge_span_ratio=args.grasp_prediction_min_edge_span,
                    prediction_max_fit_rmse_m=args.grasp_prediction_max_rmse,
                )
            except (KeyError, TypeError, ValueError) as exc:
                grasp_points = invalid_grasp_result("DUAL_GRASP_GEOMETRY_ERROR", frame_index)
                grasp_points["error"] = str(exc)
            grasp_points.update({
                "sequence": frame_index,
                "published_at_unix_sec": time.time(),
                "rgb_stamp": color_stamp,
                "depth_stamp": depth_stamp,
                "rgb_depth_delta_ms": abs(color_stamp - depth_stamp) * 1000.0,
                "crate_color_profile": active_color_profile,
            })
            publish_grasp_points(args.output_dir, grasp_points)

            overlay = render(rgb, pose, camera_matrix, metadata, grasp_points)
            # The host-side viewer may read this file while inference is
            # producing the next frame.  Write to a complete temporary PNG and
            # atomically replace the public file to avoid partial-image reads.
            latest_image = args.output_dir / "latest.png"
            temporary_image = args.output_dir / "latest.tmp.png"
            cv2.imwrite(str(temporary_image), overlay)
            os.replace(temporary_image, latest_image)

            latest_pose = args.output_dir / "latest_pose.txt"
            temporary_pose = args.output_dir / "latest_pose.tmp.txt"
            np.savetxt(temporary_pose, pose, fmt="%.9f")
            os.replace(temporary_pose, latest_pose)
            if args.save_every > 0 and frame_index % args.save_every == 0:
                cv2.imwrite(str(args.output_dir / f"frame_{frame_index:06d}.png"), overlay)
            grasp_log = "INVALID"
            if grasp_points["valid"]:
                left, right = grasp_points["points_left_to_right"]
                grasp_log = (
                    "L=({:+.3f},{:+.3f},{:+.3f},occ={}) "
                    "R=({:+.3f},{:+.3f},{:+.3f},occ={})"
                ).format(
                    *left["point_camera_m"], int(left["occluded"]),
                    *right["point_camera_m"], int(right["occluded"]),
                )
            print(
                f"frame={frame_index} stamp={color_stamp:.9f} sync={abs(color_stamp-depth_stamp)*1000:.1f}ms "
                f"xyz=({pose[0,3]:+.3f},{pose[1,3]:+.3f},{pose[2,3]:+.3f})m grasp={grasp_log}"
            )
            if args.show:
                cv2.imshow("FoundationPose plastic crate", overlay)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            frame_index += 1
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        source.close()
        if args.mark_not_running_on_exit:
            publish_grasp_points(
                args.output_dir,
                invalid_grasp_result("PROCESS_NOT_RUNNING", frame_index),
            )
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
