#!/usr/bin/env python3
"""Inspect MPC neck-related services through rosbridge."""

from __future__ import annotations

import argparse
import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from tools.run_mpc_visual_grasp_test import _connect, _call


KEYWORDS = ("neck", "waist", "lock", "track", "hardware")


def _rosapi_call(client, name: str, request: dict | None = None):
    return _call(client, name, "rosapi/" + name.rsplit("/", 1)[-1], request or {})


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws-url", default="ws://192.168.20.98:9091")
    args = parser.parse_args()

    client = _connect(args.ws_url)
    try:
        services_resp = _rosapi_call(client, "/rosapi/services")
        services = services_resp.get("services", [])
        matched = [s for s in services if any(k in s.lower() for k in KEYWORDS)]
        print("MPC neck/waist/lock/track candidate services:")
        for service in sorted(matched):
            try:
                type_resp = _rosapi_call(client, "/rosapi/service_type", {"service": service})
                srv_type = type_resp.get("type", "")
            except Exception as exc:
                srv_type = f"<type query failed: {exc}>"
            print(f"  {service}\n    type: {srv_type}")

        print("\n重点找这个类型:")
        print("  mpc_hardware_interface/MPCWaistLockSetting")
        print("\n如果存在，下一步测试:")
        print('  rosservice call <SERVICE_NAME> "joint_state: []')
        print("  lock_index: []")
        print('  neck_track: true"')
    finally:
        try:
            client.terminate()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
