"""
RealSense health guard for camera-dependent tasks.

Before running vision/QR tasks, verify the head camera topics are present and
actually publishing frames. If the stream is missing, call the robot-side
restart service and wait for frames to recover.
"""

from __future__ import annotations

import threading
import time

import roslibpy

from . import config


def _parse_ws_url(ws_url: str) -> tuple[str, int]:
    host_port = ws_url.replace("ws://", "", 1)
    host, port = host_port.split(":", 1)
    return host, int(port)


def _connect(ws_url: str) -> roslibpy.Ros:
    host, port = _parse_ws_url(ws_url)
    client = roslibpy.Ros(host=host, port=port)
    thread = threading.Thread(target=client.run, daemon=True)
    thread.start()
    start = time.time()
    while not client.is_connected:
        if time.time() - start > config.CONNECT_TIMEOUT:
            try:
                client.terminate()
            except Exception:
                pass
            raise RuntimeError(f"连接超时: {ws_url}")
        time.sleep(0.1)
    return client


def _call(client: roslibpy.Ros, service_name: str, service_type: str, request: dict | None = None):
    service = roslibpy.Service(client, service_name, service_type)
    return service.call(roslibpy.ServiceRequest(request or {}))


def _service_type(client: roslibpy.Ros, service_name: str) -> str:
    try:
        response = _call(client, "/rosapi/service_type", "rosapi/ServiceType", {"service": service_name})
        return response.get("type", "")
    except Exception:
        return ""


def _topic_list(client: roslibpy.Ros) -> list[str]:
    try:
        response = _call(client, "/rosapi/topics", "rosapi/Topics", {})
        return list(response.get("topics", []))
    except Exception:
        return []


def _make_topic(client: roslibpy.Ros, topic: str, msg_type: str, throttle_ms: int) -> roslibpy.Topic:
    try:
        return roslibpy.Topic(
            client,
            topic,
            msg_type,
            throttle_rate=throttle_ms,
            queue_length=1,
        )
    except TypeError:
        try:
            return roslibpy.Topic(
                client,
                topic,
                msg_type,
                throttle_rate=throttle_ms,
                queue_size=1,
            )
        except TypeError:
            return roslibpy.Topic(client, topic, msg_type)


def _wait_for_camera_streams(
    client: roslibpy.Ros,
    timeout: float,
    require_depth: bool,
    require_camera_info: bool,
    require_raw_rgb: bool,
) -> tuple[bool, dict[str, int]]:
    counts = {"rgb": 0, "raw_rgb": 0, "depth": 0, "camera_info": 0}
    lock = threading.Lock()

    def count_rgb(_message):
        with lock:
            counts["rgb"] += 1

    def count_raw_rgb(_message):
        with lock:
            counts["raw_rgb"] += 1

    def count_depth(_message):
        with lock:
            counts["depth"] += 1

    def count_info(_message):
        with lock:
            counts["camera_info"] += 1

    subscribers = [_make_topic(client, config.TOPIC_RGB, "sensor_msgs/CompressedImage", 500)]
    subscribers[0].subscribe(count_rgb)

    if require_raw_rgb:
        subscribers.append(
            _make_topic(client, config.TOPIC_RGB_RAW, "sensor_msgs/Image", 1000)
        )
        subscribers[-1].subscribe(count_raw_rgb)

    if require_depth:
        subscribers.append(
            _make_topic(
                client,
                config.TOPIC_DEPTH_COMPRESSED,
                "sensor_msgs/CompressedImage",
                500,
            )
        )
        subscribers[-1].subscribe(count_depth)

    if require_camera_info:
        subscribers.append(
            _make_topic(client, config.TOPIC_CAMERA_INFO, "sensor_msgs/CameraInfo", 1000)
        )
        subscribers[-1].subscribe(count_info)

    deadline = time.time() + timeout
    try:
        while time.time() < deadline:
            with lock:
                rgb_ok = counts["rgb"] > 0
                raw_ok = (not require_raw_rgb) or counts["raw_rgb"] > 0
                depth_ok = (not require_depth) or counts["depth"] > 0
                info_ok = (not require_camera_info) or counts["camera_info"] > 0
                if rgb_ok and raw_ok and depth_ok and info_ok:
                    return True, dict(counts)
            time.sleep(0.1)
        return False, dict(counts)
    finally:
        for sub in subscribers:
            try:
                sub.unsubscribe()
            except Exception:
                pass


def _restart_realsense(client: roslibpy.Ros) -> None:
    service_type = _service_type(client, config.SERVICE_REALSENSE_RESTART) or "std_srvs/Trigger"
    print(f"[动作] RealSense restart: {config.SERVICE_REALSENSE_RESTART}")
    response = _call(client, config.SERVICE_REALSENSE_RESTART, service_type, {})
    print(f"[realsense] {response}")
    if response and response.get("success") is False:
        raise RuntimeError(f"RealSense restart 失败: {response}")


def ensure_realsense_ready(
    ws_url: str,
    *,
    require_depth: bool = True,
    require_camera_info: bool = True,
    require_raw_rgb: bool = False,
    sample_timeout: float | None = None,
    restart_wait: float | None = None,
) -> None:
    """Verify RealSense streams and restart the camera if needed."""
    if not config.CAMERA_GUARD_ENABLED:
        return

    sample_timeout = config.CAMERA_GUARD_SAMPLE_TIMEOUT if sample_timeout is None else sample_timeout
    restart_wait = config.CAMERA_GUARD_RESTART_WAIT if restart_wait is None else restart_wait

    client = _connect(ws_url)
    try:
        ensure_realsense_ready_on_client(
            client,
            require_depth=require_depth,
            require_camera_info=require_camera_info,
            require_raw_rgb=require_raw_rgb,
            sample_timeout=sample_timeout,
            restart_wait=restart_wait,
        )
    finally:
        try:
            client.terminate()
        except Exception:
            pass


def ensure_realsense_ready_on_client(
    client: roslibpy.Ros,
    *,
    require_depth: bool = True,
    require_camera_info: bool = True,
    require_raw_rgb: bool = False,
    sample_timeout: float | None = None,
    restart_wait: float | None = None,
) -> None:
    """Same guard as ensure_realsense_ready, but reuses an existing Ros client.

    Use this inside long-running scripts. Twisted/roslibpy cannot reliably stop
    and restart the reactor in the same Python process.
    """
    if not config.CAMERA_GUARD_ENABLED:
        return

    sample_timeout = config.CAMERA_GUARD_SAMPLE_TIMEOUT if sample_timeout is None else sample_timeout
    restart_wait = config.CAMERA_GUARD_RESTART_WAIT if restart_wait is None else restart_wait

    required_topics = [config.TOPIC_RGB]
    if require_raw_rgb:
        required_topics.append(config.TOPIC_RGB_RAW)
    if require_depth:
        required_topics.append(config.TOPIC_DEPTH_COMPRESSED)
    if require_camera_info:
        required_topics.append(config.TOPIC_CAMERA_INFO)

    topics = _topic_list(client)
    missing_topics = [topic for topic in required_topics if topic not in topics]
    streams_ok = False
    counts = {"rgb": 0, "raw_rgb": 0, "depth": 0, "camera_info": 0}
    if not missing_topics:
        streams_ok, counts = _wait_for_camera_streams(
            client,
            sample_timeout,
            require_depth,
            require_camera_info,
            require_raw_rgb,
        )

    if not missing_topics and streams_ok:
        print(
            "[✓] RealSense 图像流正常: "
            f"rgb={counts['rgb']}, raw_rgb={counts['raw_rgb']}, "
            f"depth={counts['depth']}, camera_info={counts['camera_info']}"
        )
        return

    if missing_topics:
        print(f"[!] RealSense 话题缺失: {missing_topics}")
    else:
        print(
            "[!] RealSense 未收到完整图像流: "
            f"rgb={counts['rgb']}, raw_rgb={counts['raw_rgb']}, "
            f"depth={counts['depth']}, camera_info={counts['camera_info']}"
        )
    _restart_realsense(client)

    recovered, recovered_counts = _wait_for_camera_streams(
        client,
        restart_wait,
        require_depth,
        require_camera_info,
        require_raw_rgb,
    )
    if not recovered:
        raise RuntimeError(
            "RealSense restart 后仍未收到完整图像流: "
            f"rgb={recovered_counts['rgb']}, raw_rgb={recovered_counts['raw_rgb']}, "
            f"depth={recovered_counts['depth']}, "
            f"camera_info={recovered_counts['camera_info']}"
        )
    print(
        "[✓] RealSense 图像流已恢复: "
        f"rgb={recovered_counts['rgb']}, raw_rgb={recovered_counts['raw_rgb']}, "
        f"depth={recovered_counts['depth']}, "
        f"camera_info={recovered_counts['camera_info']}"
    )
