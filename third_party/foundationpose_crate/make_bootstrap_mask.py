#!/usr/bin/env python3
"""Create a configurable full-frame colour mask for pose initialization."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = ROOT / "config" / "bootstrap_mask.json"


def load_mask_config(path: Path) -> dict[str, Any]:
    try:
        config = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise RuntimeError(f"bootstrap mask config does not exist: {path}") from exc
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"invalid bootstrap mask config {path}: {exc}") from exc
    if not isinstance(config.get("profiles"), dict) or not config["profiles"]:
        raise RuntimeError("bootstrap mask config must contain non-empty 'profiles'")
    return config


def _hsv_triplet(value: Any, field: str) -> np.ndarray:
    triplet = np.asarray(value, dtype=np.int64)
    if triplet.shape != (3,):
        raise RuntimeError(f"{field} must contain exactly [H, S, V]")
    if not (0 <= triplet[0] <= 179 and np.all((0 <= triplet[1:]) & (triplet[1:] <= 255))):
        raise RuntimeError(f"{field} is outside OpenCV HSV limits")
    return triplet.astype(np.uint8)


def build_color_mask(
    image: np.ndarray,
    config: dict[str, Any],
    profile_name: str | None = None,
) -> tuple[np.ndarray, str]:
    """Return every pixel matching the selected crate-colour profile."""
    profiles = config["profiles"]
    selected = profile_name or config.get("active_profile")
    if selected not in profiles:
        available = ", ".join(sorted(profiles))
        raise RuntimeError(f"unknown mask profile '{selected}'; available: {available}")
    ranges = profiles[selected].get("hsv_ranges")
    if not isinstance(ranges, list) or not ranges:
        raise RuntimeError(f"mask profile '{selected}' has no hsv_ranges")

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    for index, limits in enumerate(ranges):
        lower = _hsv_triplet(limits.get("lower"), f"profiles.{selected}.hsv_ranges[{index}].lower")
        upper = _hsv_triplet(limits.get("upper"), f"profiles.{selected}.hsv_ranges[{index}].upper")
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

    # There is deliberately no image-position ROI.  The crate may be located
    # anywhere in the camera view.
    kernel_size = int(config.get("morphology_kernel_pixels", 3))
    if kernel_size > 1:
        if kernel_size % 2 == 0:
            raise RuntimeError("morphology_kernel_pixels must be odd")
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=int(config.get("morphology_close_iterations", 2)),
        )

    return mask, selected


def build_mask(
    image: np.ndarray,
    config: dict[str, Any],
    profile_name: str | None = None,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Find the largest configured-colour component anywhere in the image."""
    mask, selected = build_color_mask(image, config, profile_name)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if count <= 1:
        raise RuntimeError(f"no component found for colour profile '{selected}'")
    component = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x, y, width, height, area = stats[component].tolist()

    # Disabled by default because pixel area changes with camera distance.  A
    # site may opt in after the final camera geometry has been measured.
    minimum_area = int(config.get("minimum_component_area_pixels", 0))
    if minimum_area > 0 and area < minimum_area:
        raise RuntimeError(
            f"largest '{selected}' component is {area} pixels; configured minimum is {minimum_area}"
        )

    output = np.where(labels == component, 255, 0).astype(np.uint8)
    warning_area = int(config.get("small_component_warning_below_pixels", 1000))
    return output, {
        "profile": selected,
        "x": x,
        "y": y,
        "width": width,
        "height": height,
        "area": area,
        "small_component_warning": bool(warning_area > 0 and area < warning_area),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--profile",
        help="override active_profile from the JSON, for example: blue, green, red",
    )
    args = parser.parse_args()

    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit(f"could not read image: {args.image}")
    mask, stats = build_mask(image, load_mask_config(args.config), args.profile)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), mask):
        raise SystemExit(f"could not write mask: {args.output}")
    print(
        "mask saved:", args.output,
        f"profile={stats['profile']}",
        f"bbox=({stats['x']},{stats['y']},{stats['width']},{stats['height']})",
        f"area={stats['area']}",
    )
    if stats["small_component_warning"]:
        print("warning: selected component is small; continuing because minimum area is disabled")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
