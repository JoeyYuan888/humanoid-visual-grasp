"""
ROS WebSocket 客户端封装
- 连接 / 断开 rosbridge
- 订阅 RGB、深度、camera_info 三个话题
- 线程安全地提供最新帧
"""

import sys
import time
import base64
import threading

import numpy as np
import cv2
import roslibpy

from . import config
from .camera_guard import ensure_realsense_ready_on_client


class ROSClient:
    """管理 rosbridge 连接与话题订阅。"""

    def __init__(self, ws_url: str | None = None):
        self.ws_url = ws_url or config.WS_URL
        self._client: roslibpy.Ros | None = None
        self._depth_client: roslibpy.Ros | None = None
        self._qr_client: roslibpy.Ros | None = None
        self._subscribers: list[roslibpy.Topic] = []
        self._thread: threading.Thread | None = None
        self._depth_thread: threading.Thread | None = None
        self._qr_thread: threading.Thread | None = None
        self._depth_decode_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        # 线程安全的帧缓存
        self._lock = threading.Lock()
        self.rgb_frame: np.ndarray | None = None
        self.raw_rgb_frame: np.ndarray | None = None
        self.depth_frame: np.ndarray | None = None
        self.camera_info: dict | None = None
        self.frame_count = 0
        self.raw_rgb_count = 0
        self.depth_msg_count = 0
        self.depth_count = 0
        self.rgb_updated_at = 0.0
        self.raw_rgb_updated_at = 0.0
        self.depth_updated_at = 0.0
        self._last_depth_decode_started_at = 0.0
        self._depth_packet_lock = threading.Lock()
        self._latest_depth_b64: str | None = None
        self._latest_depth_seq = 0
        self._decoded_depth_seq = 0

    # ---- 连接 ----

    def connect(self) -> bool:
        """连接到机器人 rosbridge，阻塞直到连接成功或超时。"""
        host = self.ws_url.replace("ws://", "").split(":")[0]
        port = int(self.ws_url.replace("ws://", "").split(":")[1])

        self._stop_event.clear()

        self._client = roslibpy.Ros(host=host, port=port)
        self._client.on_ready(self._on_connected)

        self._thread = threading.Thread(target=self._client.run, daemon=True)
        self._thread.start()

        # 等待连接
        start = time.time()
        while not self._client.is_connected:
            if time.time() - start > config.CONNECT_TIMEOUT:
                print(f"[✗] 连接超时 ({self.ws_url})")
                self._client.terminate()
                return False
            time.sleep(0.1)

        try:
            ensure_realsense_ready_on_client(
                self._client,
                require_depth=config.ENABLE_DEPTH,
                require_camera_info=config.ENABLE_DEPTH,
                require_raw_rgb=config.ENABLE_QR,
            )
        except Exception as exc:
            print(f"[✗] RealSense 图像流检查失败: {exc}")
            self._client.terminate()
            return False

        if config.ENABLE_DEPTH and config.USE_SEPARATE_DEPTH_CLIENT:
            self._depth_client = roslibpy.Ros(host=host, port=port)
            self._depth_thread = threading.Thread(target=self._depth_client.run, daemon=True)
            self._depth_thread.start()

            start = time.time()
            while not self._depth_client.is_connected:
                if time.time() - start > config.CONNECT_TIMEOUT:
                    print(f"[✗] 深度连接超时 ({self.ws_url})")
                    self._depth_client.terminate()
                    self._depth_client = None
                    return False
                time.sleep(0.1)

        if config.ENABLE_QR and config.USE_SEPARATE_QR_CLIENT:
            self._qr_client = roslibpy.Ros(host=host, port=port)
            self._qr_thread = threading.Thread(target=self._qr_client.run, daemon=True)
            self._qr_thread.start()

            start = time.time()
            while not self._qr_client.is_connected:
                if time.time() - start > config.CONNECT_TIMEOUT:
                    print(f"[✗] QR raw RGB 连接超时 ({self.ws_url})")
                    self._qr_client.terminate()
                    self._qr_client = None
                    return False
                time.sleep(0.1)

        self._start_depth_decoder()
        self._subscribe_all()
        return True

    def disconnect(self):
        """断开连接并清理。"""
        self._stop_event.set()
        for sub in self._subscribers:
            try:
                sub.unsubscribe()
            except Exception:
                pass
        self._subscribers.clear()
        self._safe_terminate(self._qr_client)
        self._qr_client = None
        self._safe_terminate(self._depth_client)
        self._depth_client = None
        self._safe_terminate(self._client)
        self._client = None

    def _start_depth_decoder(self):
        if not config.ENABLE_DEPTH or config.DEPTH_TRANSPORT != "compressedDepth":
            return
        self._depth_decode_thread = threading.Thread(target=self._depth_decode_loop, daemon=True)
        self._depth_decode_thread.start()

    def _safe_terminate(self, client: roslibpy.Ros | None):
        """roslibpy 某些版本 terminate() 会在退出时抛内部 AttributeError。"""
        if client is None:
            return
        try:
            client.terminate()
        except AttributeError as exc:
            if "_thread" not in str(exc):
                raise
            print("[!] roslibpy 退出清理时忽略内部 _thread 异常")
        except Exception as exc:
            print(f"[!] roslibpy 退出清理异常，已忽略: {exc}")

    @property
    def is_connected(self) -> bool:
        rgb_ok = self._client is not None and self._client.is_connected
        depth_ok = (
            not config.ENABLE_DEPTH
            or not config.USE_SEPARATE_DEPTH_CLIENT
            or (self._depth_client is not None and self._depth_client.is_connected)
        )
        qr_ok = (
            not config.ENABLE_QR
            or not config.USE_SEPARATE_QR_CLIENT
            or (self._qr_client is not None and self._qr_client.is_connected)
        )
        return rgb_ok and depth_ok and qr_ok

    # ---- 订阅 ----

    def _subscribe_all(self):
        """订阅三个必需话题。"""
        subs = [
            (self._client, config.TOPIC_RGB, "sensor_msgs/CompressedImage", self._on_rgb, config.ROS_RGB_THROTTLE_MS),
        ]
        if config.ENABLE_QR:
            qr_client = self._qr_client if config.USE_SEPARATE_QR_CLIENT else self._client
            subs.append(
                (qr_client, config.TOPIC_RGB_RAW, "sensor_msgs/Image", self._on_raw_rgb, config.QR_RAW_RGB_THROTTLE_MS)
            )
        if config.ENABLE_DEPTH:
            depth_client = self._depth_client if config.USE_SEPARATE_DEPTH_CLIENT else self._client
            if config.DEPTH_TRANSPORT == "compressedDepth":
                depth_topic = config.TOPIC_DEPTH_COMPRESSED
                depth_type = "sensor_msgs/CompressedImage"
                depth_callback = self._on_compressed_depth
            else:
                depth_topic = config.TOPIC_DEPTH
                depth_type = "sensor_msgs/Image"
                depth_callback = self._on_raw_depth
            subs.extend([
                (depth_client, depth_topic, depth_type, depth_callback, config.ROS_DEPTH_THROTTLE_MS),
                (depth_client, config.TOPIC_CAMERA_INFO, "sensor_msgs/CameraInfo", self._on_camera_info, 1000),
            ])
        for client, topic, msg_type, callback, throttle_ms in subs:
            sub = self._make_topic(client, topic, msg_type, throttle_ms)
            sub.subscribe(callback)
            self._subscribers.append(sub)
            print(f"    订阅: {topic} throttle={throttle_ms}ms")

    def _make_topic(self, client: roslibpy.Ros, topic: str, msg_type: str, throttle_ms: int):
        """兼容不同 roslibpy 版本创建低延迟订阅。"""
        try:
            return roslibpy.Topic(
                client,
                topic,
                msg_type,
                throttle_rate=throttle_ms,
                queue_length=config.ROS_QUEUE_LENGTH,
            )
        except TypeError:
            try:
                return roslibpy.Topic(
                    client,
                    topic,
                    msg_type,
                    throttle_rate=throttle_ms,
                    queue_size=config.ROS_QUEUE_LENGTH,
                )
            except TypeError:
                print(f"[!] 当前 roslibpy 不支持订阅队列参数: {topic}")
                return roslibpy.Topic(client, topic, msg_type)

    # ---- 回调 ----

    def _on_connected(self):
        print(f"[✓] 已连接到 {self.ws_url}")

    def _on_rgb(self, message: dict):
        """处理 RGB 压缩图。"""
        try:
            data_b64 = message.get("data")
            if not data_b64:
                return
            img_bytes = base64.b64decode(data_b64)
            img_array = np.frombuffer(img_bytes, dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            if frame is None:
                return
            with self._lock:
                self.rgb_frame = frame
                self.frame_count += 1
                self.rgb_updated_at = time.time()
        except Exception:
            pass

    def _on_raw_rgb(self, message: dict):
        """低频缓存 raw RGB，供小二维码识别使用。"""
        try:
            height = message.get("height", 0)
            width = message.get("width", 0)
            encoding = message.get("encoding", "")
            data_b64 = message.get("data")
            if not data_b64 or height == 0 or width == 0:
                return

            raw = base64.b64decode(data_b64)
            arr = np.frombuffer(raw, dtype=np.uint8)

            if encoding in ("rgb8", "bgr8"):
                frame = arr.reshape((height, width, 3))
                if encoding == "rgb8":
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif encoding in ("mono8", "8UC1"):
                gray = arr.reshape((height, width))
                frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            else:
                return

            with self._lock:
                self.raw_rgb_frame = frame
                self.raw_rgb_count += 1
                self.raw_rgb_updated_at = time.time()
        except Exception:
            pass

    def _on_raw_depth(self, message: dict):
        """处理对齐深度图 (16UC1, 毫米)。"""
        try:
            height = message.get("height", 0)
            width = message.get("width", 0)
            data_b64 = message.get("data")
            if not data_b64 or height == 0 or width == 0:
                return

            # 解码 base64 -> raw bytes -> uint16
            raw = base64.b64decode(data_b64)
            arr = np.frombuffer(raw, dtype=np.uint16).reshape((height, width))
            with self._lock:
                self.depth_frame = arr
                self.depth_msg_count += 1
                self.depth_count += 1
                self.depth_updated_at = time.time()
        except Exception:
            pass

    def _on_compressed_depth(self, message: dict):
        """缓存最新 compressedDepth 消息，实际解码放到后台线程。"""
        try:
            data_b64 = message.get("data")
            if not data_b64:
                return
            with self._depth_packet_lock:
                self._latest_depth_b64 = data_b64
                self._latest_depth_seq += 1
            with self._lock:
                self.depth_msg_count += 1
        except Exception:
            pass

    def _depth_decode_loop(self):
        """后台解码最新深度，避免在 rosbridge 回调线程里做 PNG 解码。"""
        while not self._stop_event.is_set():
            now = time.time()
            if now - self._last_depth_decode_started_at < config.DEPTH_DECODE_INTERVAL_SEC:
                time.sleep(0.01)
                continue

            with self._depth_packet_lock:
                data_b64 = self._latest_depth_b64
                seq = self._latest_depth_seq

            if not data_b64 or seq == self._decoded_depth_seq:
                time.sleep(0.01)
                continue

            try:
                self._last_depth_decode_started_at = now

                raw = base64.b64decode(data_b64)
                png_start = raw.find(b"\x89PNG\r\n\x1a\n")
                if png_start < 0:
                    self._decoded_depth_seq = seq
                    continue

                png = np.frombuffer(raw[png_start:], dtype=np.uint8)
                depth = cv2.imdecode(png, cv2.IMREAD_UNCHANGED)
                if depth is None:
                    self._decoded_depth_seq = seq
                    continue

                if depth.dtype != np.uint16:
                    depth = depth.astype(np.uint16)

                with self._lock:
                    self.depth_frame = depth
                    self.depth_count += 1
                    self.depth_updated_at = time.time()
                self._decoded_depth_seq = seq
            except Exception:
                self._decoded_depth_seq = seq

    def _on_camera_info(self, message: dict):
        """缓存 camera_info（只存一次）。"""
        with self._lock:
            if self.camera_info is None:
                self.camera_info = message

    # ---- 读取最新帧（线程安全） ----

    def get_frames(self):
        """原子化获取 RGB、深度、camera_info。"""
        with self._lock:
            return self.rgb_frame, \
                   self.depth_frame, \
                   dict(self.camera_info) if self.camera_info is not None else None, \
                   self.frame_count

    def get_raw_rgb(self):
        """获取低频 raw RGB 缓存。"""
        with self._lock:
            return self.raw_rgb_frame, self.raw_rgb_count, self.raw_rgb_updated_at

    def get_stats(self):
        """获取接收端计数和时间戳。"""
        with self._lock:
            return {
                "rgb_count": self.frame_count,
                "raw_rgb_count": self.raw_rgb_count,
                "depth_msg_count": self.depth_msg_count,
                "depth_count": self.depth_count,
                "rgb_updated_at": self.rgb_updated_at,
                "raw_rgb_updated_at": self.raw_rgb_updated_at,
                "depth_updated_at": self.depth_updated_at,
            }
