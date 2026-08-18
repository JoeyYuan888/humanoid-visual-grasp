#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time

import roslibpy


REQUIRED_SERVICES = [
    "/wa/points_seq_tracking",
    "/wa/points_seq_tracking_with_joints",
    "/wa/wa_hardware_interface/mpc_mode_setting",
    "/wa/wa_hardware_interface/neck_movej",
]

REQUIRED_TOPICS = [
    "/tf",
    "/tf_static",
    "/zj_humanoid/sensor/realsense_head/color/image_raw/compressed",
    "/zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw/compressedDepth",
]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws-url", default="ws://192.168.20.102:9091")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()

    host_port = args.ws_url.replace("ws://", "").split("/")
    host, port_text = host_port[0].split(":")
    client = roslibpy.Ros(host=host, port=int(port_text))
    client.run()
    deadline = time.time() + args.timeout
    while not client.is_connected and time.time() < deadline:
        time.sleep(0.1)
    if not client.is_connected:
        print(f"[x] rosbridge connection failed: {args.ws_url}")
        return 1

    services = set(client.get_services())
    topics = set(client.get_topics())

    missing_services = [name for name in REQUIRED_SERVICES if name not in services]
    missing_topics = [name for name in REQUIRED_TOPICS if name not in topics]

    client.terminate()

    if missing_services or missing_topics:
        for name in missing_services:
            print(f"[x] missing service: {name}")
        for name in missing_topics:
            print(f"[x] missing topic: {name}")
        return 2

    print("[ok] robot rosbridge basics ready")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
