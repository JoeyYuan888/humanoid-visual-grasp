"""
Coordinate transforms for the course-manual grasp pipeline.

Official chain:
    camera -> CAM2HEAD -> HEAD2BASE(tf) -> object in BASE -> GRASP_OFFSET -> TCP target

This module intentionally does not provide a simplified camera_to_mpc shortcut.
Do not send camera-frame vision coordinates directly to MPC motion APIs.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class TransformConfig:
    cam2head: np.ndarray
    grasp_offset: np.ndarray


def make_transform(matrix: Iterable[Iterable[float]], name: str) -> np.ndarray:
    transform = np.asarray(matrix, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError(f"{name} must be a 4x4 homogeneous transform")
    return transform


def camera_point_to_base_pose(
    x_mm: float,
    y_mm: float,
    z_mm: float,
    cam2head: np.ndarray,
    head_to_base: np.ndarray,
) -> np.ndarray:
    """
    Convert a depth-derived camera point into an object pose in BASE frame.

    The input point comes from robot_grasp.vision.depth_utils and is in millimeters.
    The returned 4x4 pose has identity orientation until a grasp policy defines
    object orientation.
    """
    cam2head = make_transform(cam2head, "cam2head")
    head_to_base = make_transform(head_to_base, "head_to_base")

    point_cam = np.array([x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0, 1.0])
    point_base = head_to_base @ cam2head @ point_cam

    object_pose_base = np.eye(4)
    object_pose_base[:3, 3] = point_base[:3]
    return object_pose_base


def object_pose_to_tcp_target(
    object_pose_base: np.ndarray,
    grasp_offset: np.ndarray,
) -> np.ndarray:
    """Apply the calibrated object/grasp-point to TCP offset."""
    object_pose_base = make_transform(object_pose_base, "object_pose_base")
    grasp_offset = make_transform(grasp_offset, "grasp_offset")
    return object_pose_base @ grasp_offset


def camera_point_to_tcp_target(
    x_mm: float,
    y_mm: float,
    z_mm: float,
    cam2head: np.ndarray,
    head_to_base: np.ndarray,
    grasp_offset: np.ndarray,
) -> np.ndarray:
    object_pose_base = camera_point_to_base_pose(
        x_mm=x_mm,
        y_mm=y_mm,
        z_mm=z_mm,
        cam2head=cam2head,
        head_to_base=head_to_base,
    )
    return object_pose_to_tcp_target(object_pose_base, grasp_offset)
