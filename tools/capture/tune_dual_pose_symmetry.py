#!/usr/bin/env python3
"""Create a small symmetric adjustment proposal for a captured dual-arm pose.

This tool is offline and read-only. It does not connect to ROS and does not
send motion commands.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from copy import deepcopy


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _pos(pose: dict) -> dict:
    return pose["position"]


def _apply_position_symmetry(payload: dict, max_delta_m: float) -> tuple[dict, dict]:
    out = deepcopy(payload)
    left = _pos(payload["left"]["pose"])
    right = _pos(payload["right"]["pose"])

    target_x = 0.5 * (float(left["x"]) + float(right["x"]))
    target_z = 0.5 * (float(left["z"]) + float(right["z"]))
    target_y_abs = 0.5 * (abs(float(left["y"])) + abs(float(right["y"])))

    targets = {
        "left": {"x": target_x, "y": target_y_abs, "z": target_z},
        "right": {"x": target_x, "y": -target_y_abs, "z": target_z},
    }
    report = {"position": {}}
    for arm in ("left", "right"):
        src = _pos(payload[arm]["pose"])
        dst = _pos(out[arm]["pose"])
        report["position"][arm] = {}
        for axis in ("x", "y", "z"):
            before = float(src[axis])
            raw_delta = float(targets[arm][axis] - before)
            delta = _clamp(raw_delta, max_delta_m)
            after = before + delta
            dst[axis] = after
            report["position"][arm][axis] = {
                "before": before,
                "target": float(targets[arm][axis]),
                "raw_delta_m": raw_delta,
                "applied_delta_m": delta,
                "after": after,
                "clamped": abs(raw_delta) > max_delta_m,
            }
    return out, report


def _quat_norm(q: dict) -> float:
    return math.sqrt(sum(float(q[k]) ** 2 for k in ("x", "y", "z", "w")))


def _orientation_report(payload: dict) -> dict:
    left = payload["left"]["pose"]["orientation"]
    right = payload["right"]["pose"]["orientation"]
    return {
        "left_norm": _quat_norm(left),
        "right_norm": _quat_norm(right),
        "note": "orientation is preserved; tune palm orientation manually, then recapture with --include-joints",
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="Captured dual pose JSON")
    parser.add_argument("--output", required=True)
    parser.add_argument("--max-position-delta", type=float, default=0.02, help="Max adjustment per axis in meters.")
    parser.add_argument("--name", default="")
    args = parser.parse_args()

    with open(args.input, "r", encoding="utf-8") as f:
        payload = json.load(f)
    if payload.get("type") != "dual_arm_mpc_pose":
        raise SystemExit(f"not a dual_arm_mpc_pose file: {args.input}")

    tuned, report = _apply_position_symmetry(payload, args.max_position_delta)
    report["orientation"] = _orientation_report(payload)
    tuned["created_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    tuned["name"] = args.name or f"{payload.get('name', '')}_symmetry_tuned".strip("_")
    tuned["source_file"] = args.input
    tuned["symmetry_tuning"] = {
        "max_position_delta_m": args.max_position_delta,
        "report": report,
        "warning": "mpc_state is copied from source. For strict body/joint reproduction, move robot to this tuned pose first, then recapture.",
    }

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(tuned, f, indent=2, ensure_ascii=False)

    print("=" * 70)
    print("  Dual pose symmetry tuning proposal")
    print("=" * 70)
    print(f"input : {args.input}")
    print(f"output: {args.output}")
    print(f"max per-axis delta: {args.max_position_delta:.3f} m")
    for arm in ("left", "right"):
        print(f"\n{arm}:")
        for axis in ("x", "y", "z"):
            item = report["position"][arm][axis]
            print(
                f"  {axis}: {item['before']:.4f} -> {item['after']:.4f} "
                f"(delta={item['applied_delta_m']:+.4f} m"
                f"{', clamped' if item['clamped'] else ''})"
            )
    print("\n[!] orientation preserved; hand/palm angle should be tuned by teaching and recaptured.")
    print("[!] For final MPC joint constraints, recapture after moving to the tuned pose.")


if __name__ == "__main__":
    main()
