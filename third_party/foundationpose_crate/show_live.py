#!/usr/bin/env python3
"""Show the container's latest rendered frame in a host-side OpenCV window."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

# OpenCV's bundled Qt plugin does not include fonts.  Point it at the host's
# standard DejaVu set before importing cv2 to avoid repeated font warnings.
os.environ.setdefault("QT_QPA_FONTDIR", "/usr/share/fonts/truetype/dejavu")

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
WINDOW = "FoundationPose plastic crate - q to quit"


def stop_process(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=8)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("forward", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    forwarded = args.forward[1:] if args.forward[:1] == ["--"] else args.forward

    command = [str(ROOT / "run_live.sh"), *forwarded]
    process = subprocess.Popen(command, start_new_session=True)
    latest = ROOT / "outputs" / "live" / "latest.png"
    launch_ns = time.time_ns()
    last_mtime_ns = -1

    waiting = np.zeros((360, 720, 3), dtype=np.uint8)
    cv2.putText(
        waiting,
        "Starting FoundationPose, please wait...",
        (55, 190),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.85,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.imshow(WINDOW, waiting)

    try:
        while True:
            if latest.is_file():
                mtime_ns = latest.stat().st_mtime_ns
                if mtime_ns >= launch_ns and mtime_ns != last_mtime_ns:
                    frame = cv2.imread(str(latest), cv2.IMREAD_COLOR)
                    if frame is not None:
                        cv2.imshow(WINDOW, frame)
                        last_mtime_ns = mtime_ns

            key = cv2.waitKey(20) & 0xFF
            if key == ord("q"):
                break
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
            if process.poll() is not None:
                # Drain the final frame before closing a finite-frame test.
                time.sleep(0.15)
                if latest.is_file():
                    frame = cv2.imread(str(latest), cv2.IMREAD_COLOR)
                    if frame is not None:
                        cv2.imshow(WINDOW, frame)
                        cv2.waitKey(300)
                return int(process.returncode or 0)
    except KeyboardInterrupt:
        pass
    finally:
        stop_process(process)
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
