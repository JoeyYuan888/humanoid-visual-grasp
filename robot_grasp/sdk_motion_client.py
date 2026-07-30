"""
SDK motion service wrapper.

This module keeps real robot SDK calls in one place. The dry-run grasp script
uses it to move the neck only, and to print arm commands before we enable them.
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import roslibpy

from . import config


DEFAULT_SERVICE_TYPES = {
    "/rosapi/service_type": "rosapi/ServiceType",
    "/wa/wa_hardware_interface/mpc_mode_setting": "std_srvs/SetBool",
    "/zj_humanoid/upperlimb/movej/neck": "upperlimb/MoveJ",
    "/zj_humanoid/upperlimb/go_home/neck": "std_srvs/Trigger",
    "/zj_humanoid/upperlimb/movej_by_path/right_arm": "upperlimb/MoveJByPath",
    "/zj_humanoid/upperlimb/movel/right_arm": "upperlimb/MoveL",
    "/zj_humanoid/upperlimb/unlock": "std_srvs/Trigger",
    "/zj_humanoid/upperlimb/stop": "std_srvs/Trigger",
    "/zj_humanoid/hand/joint_switch/right": "hand_controller/JointSwitch",
    "/zj_humanoid/hand/finger_pressures/right/zero": "std_srvs/Trigger",
}

MPC_MODE_SERVICE = "/wa/wa_hardware_interface/mpc_mode_setting"

SDK_SERVICE_PREFIXES = (
    "/zj_humanoid/upperlimb/",
    "/zj_humanoid/hand/",
)


class SDKMotionClient:
    """Thin roslibpy client for upperlimb SDK services."""

    def __init__(self, ws_url: str | None = None, auto_disable_mpc: bool = True):
        self.ws_url = ws_url or config.WS_URL
        self.auto_disable_mpc = auto_disable_mpc
        self._client: roslibpy.Ros | None = None
        self._thread: threading.Thread | None = None
        self._service_type_cache: dict[str, str] = {}
        self._mpc_disable_warned = False

    def connect(self) -> bool:
        host, port = self._parse_ws_url(self.ws_url)
        self._client = roslibpy.Ros(host=host, port=port)
        self._thread = threading.Thread(target=self._client.run, daemon=True)
        self._thread.start()

        start = time.time()
        while not self._client.is_connected:
            if time.time() - start > config.CONNECT_TIMEOUT:
                print(f"[✗] SDK 连接超时: {self.ws_url}")
                self.disconnect()
                return False
            time.sleep(0.1)
        print(f"[✓] SDK 已连接: {self.ws_url}")
        return True

    def disconnect(self):
        if self._client is None:
            return
        try:
            self._client.terminate()
        except AttributeError as exc:
            if "_thread" not in str(exc):
                raise
            print("[!] roslibpy SDK 退出清理时忽略内部 _thread 异常")
        except Exception as exc:
            print(f"[!] roslibpy SDK 退出清理异常，已忽略: {exc}")
        self._client = None

    @property
    def is_connected(self) -> bool:
        return self._client is not None and self._client.is_connected

    def call_service(self, name: str, request: dict | None = None) -> dict:
        if self._client is None:
            raise RuntimeError("SDKMotionClient is not connected")
        if self.auto_disable_mpc and self._is_sdk_service(name):
            self.disable_mpc_mode(required=False)
        service_type = self.service_type(name)
        service = roslibpy.Service(self._client, name, service_type)
        return service.call(roslibpy.ServiceRequest(request or {}))

    def service_type(self, name: str) -> str:
        if name in self._service_type_cache:
            return self._service_type_cache[name]

        service_type = ""
        if self._client is not None and self._client.is_connected:
            try:
                rosapi = roslibpy.Service(self._client, "/rosapi/service_type", "rosapi/ServiceType")
                response = rosapi.call(roslibpy.ServiceRequest({"service": name}))
                service_type = response.get("type", "")
            except Exception:
                service_type = ""

        if not service_type:
            service_type = DEFAULT_SERVICE_TYPES.get(name, "")
        if not service_type:
            raise RuntimeError(f"无法确定服务类型: {name}")

        self._service_type_cache[name] = service_type
        return service_type

    def neck_look_down(self, path_file: str | Path = "paths/neck_look_down.json") -> dict:
        cfg = load_path_config(path_file)
        movej = cfg["movej"]
        request = {
            "joints": cfg["commanded_joint"],
            "v": movej["v"],
            "acc": movej["acc"],
            "t": movej["t"],
            "is_async": movej["is_async"],
            "arm_type": cfg["arm_type"],
        }
        return self.call_service(cfg["service"], request)

    def neck_home(self, path_file: str | Path = "paths/neck_home.json") -> dict:
        cfg = load_path_config(path_file)
        return self.call_service(cfg["service"], {})

    def unlock(self) -> dict:
        return self.call_service("/zj_humanoid/upperlimb/unlock", {})

    def disable_mpc_mode(self, required: bool = False) -> dict | None:
        """Disable MPC mode before SDK calls to avoid controller contention."""
        if self._client is None:
            raise RuntimeError("SDKMotionClient is not connected")
        try:
            service_type = self.service_type(MPC_MODE_SERVICE)
            service = roslibpy.Service(self._client, MPC_MODE_SERVICE, service_type)
            response = service.call(roslibpy.ServiceRequest({"data": False}))
            if response.get("success", True):
                print(f"[✓] 已关闭 MPC 模式: {MPC_MODE_SERVICE}")
            else:
                message = response.get("message", "")
                text = f"MPC 模式关闭返回失败: {message}"
                if required:
                    raise RuntimeError(text)
                print(f"[!] {text}")
            return response
        except Exception as exc:
            if required:
                raise RuntimeError(f"关闭 MPC 模式失败，取消 SDK 调用: {exc}") from exc
            if not self._mpc_disable_warned:
                print(f"[!] 未能确认关闭 MPC 模式，继续前请人工确认: {exc}")
                self._mpc_disable_warned = True
            return None

    @staticmethod
    def _is_sdk_service(name: str) -> bool:
        return any(name.startswith(prefix) for prefix in SDK_SERVICE_PREFIXES)

    @staticmethod
    def _parse_ws_url(ws_url: str) -> tuple[str, int]:
        stripped = ws_url.replace("ws://", "").replace("wss://", "")
        host, port = stripped.split(":")
        return host, int(port)


def load_path_config(path_file: str | Path) -> dict:
    path = Path(path_file)
    if not path.is_absolute():
        path = Path.cwd() / path
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def movej_by_path_request(path_cfg: dict) -> dict:
    return {
        "path": [{"joint": point["joint"]} for point in path_cfg["points"]],
        "time": path_cfg.get("time", 0.0),
        "timestamp": path_cfg["timestamp"],
        "is_async": False,
        "arm_type": path_cfg["arm_type"],
    }
