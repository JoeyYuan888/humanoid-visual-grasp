#!/usr/bin/env python3
"""
Try likely MPCNeckJointMove.srv definitions and find the one whose ROS MD5
matches the running server.

Run this inside the ROS container that has rossrv and the editable
mpc_hardware_interface package:

  python /path/to/find_neck_srv_md5_candidate.py \
    --srv /workspace/catkin_ws/mpc_ws/src/mpc_hardware_interface/srv/MPCNeckJointMove.srv
"""

from __future__ import annotations

import argparse
import itertools
import subprocess
from pathlib import Path


TARGET_MD5 = "f8b6a92b69d3c18aa7af99d7e3e46e20"
SRV_TYPE = "mpc_hardware_interface/MPCNeckJointMove"


NECK_ARRAY_TYPES = ["float64[]", "float32[]"]
T_TYPES = ["int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "float32", "float64", "time", "duration"]

BOOL_NAMES = ["success", "result", "ok", "is_success"]
STRING_NAMES = ["message", "msg", "result", "error_msg"]
CODE_TYPES = ["int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64"]
CODE_NAMES = ["code", "result", "error_code", "status"]


def build_requests() -> list[str]:
    requests: list[str] = []
    for neck_type, t_type in itertools.product(NECK_ARRAY_TYPES, T_TYPES):
        requests.append(f"{neck_type} neck_joint\n{t_type} t\n")
    return requests


def build_responses() -> list[str]:
    responses: set[str] = {""}
    for bool_name in BOOL_NAMES:
        responses.add(f"bool {bool_name}\n")
        for string_name in STRING_NAMES:
            responses.add(f"bool {bool_name}\nstring {string_name}\n")
            responses.add(f"string {string_name}\nbool {bool_name}\n")
        for code_type, code_name in itertools.product(CODE_TYPES, CODE_NAMES):
            responses.add(f"bool {bool_name}\n{code_type} {code_name}\n")
            responses.add(f"{code_type} {code_name}\nbool {bool_name}\n")
            for string_name in STRING_NAMES:
                responses.add(f"bool {bool_name}\n{code_type} {code_name}\nstring {string_name}\n")
                responses.add(f"{code_type} {code_name}\nbool {bool_name}\nstring {string_name}\n")
                responses.add(f"bool {bool_name}\nstring {string_name}\n{code_type} {code_name}\n")
    for string_name in STRING_NAMES:
        responses.add(f"string {string_name}\n")
    for code_type, code_name in itertools.product(CODE_TYPES, CODE_NAMES):
        responses.add(f"{code_type} {code_name}\n")
        for string_name in STRING_NAMES:
            responses.add(f"{code_type} {code_name}\nstring {string_name}\n")
            responses.add(f"string {string_name}\n{code_type} {code_name}\n")
    return sorted(responses)


def rossrv_md5() -> str | None:
    proc = subprocess.run(
        ["rossrv", "md5", SRV_TYPE],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if proc.returncode != 0:
        print(proc.stderr.strip())
        return None
    return proc.stdout.strip().splitlines()[-1].strip()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--srv", required=True, help="Path to MPCNeckJointMove.srv")
    parser.add_argument("--target-md5", default=TARGET_MD5)
    args = parser.parse_args()

    srv_path = Path(args.srv)
    original = srv_path.read_text()
    matches: list[str] = []

    try:
        requests = build_requests()
        responses = build_responses()
        total = len(requests) * len(responses)
        checked = 0
        for req, resp in itertools.product(requests, responses):
            checked += 1
            spec = req.rstrip() + "\n---\n" + resp.rstrip() + ("\n" if resp else "")
            srv_path.write_text(spec)
            md5 = rossrv_md5()
            if md5 == args.target_md5:
                print("\n[FOUND] matched server MD5")
                print(f"md5: {md5}")
                print("srv definition:")
                print(spec)
                matches.append(spec)
            elif checked % 250 == 0:
                print(f"checked {checked}/{total}")
    finally:
        srv_path.write_text(original)

    if not matches:
        print(f"\n[NOT FOUND] no candidate matched {args.target_md5}")
        print("The real srv definition is outside the built-in candidate list.")
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
