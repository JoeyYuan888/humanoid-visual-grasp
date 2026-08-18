#!/usr/bin/env python3
"""Estimate one crate pose and project its CAD top rim into the RGB image."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

import cv2
import imageio.v2 as imageio
import numpy as np
import trimesh


ROOT = Path(__file__).resolve().parent
VENDOR = ROOT / "vendor" / "FoundationPose"


def project(points: np.ndarray, pose: np.ndarray, camera_matrix: np.ndarray) -> np.ndarray:
    homogeneous = np.column_stack((points, np.ones(len(points))))
    camera_points = (pose @ homogeneous.T).T[:, :3]
    if np.any(camera_points[:, 2] <= 0):
        raise RuntimeError("one or more CAD rim points are behind the camera")
    pixels_h = (camera_matrix @ camera_points.T).T
    return pixels_h[:, :2] / pixels_h[:, 2:3]


def draw_closed_polyline(image: np.ndarray, points: np.ndarray, color: tuple[int, int, int]) -> None:
    pixels = np.rint(points).astype(np.int32).reshape((-1, 1, 2))
    cv2.polylines(image, [pixels], True, color, 3, cv2.LINE_AA)
    for index, point in enumerate(pixels[:, 0]):
        cv2.circle(image, tuple(point), 5, color, -1, cv2.LINE_AA)
        cv2.putText(
            image,
            str(index),
            tuple(point + np.array((6, -6))),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )


def load_mesh(path: Path) -> trimesh.Trimesh:
    loaded = trimesh.load(path)
    if isinstance(loaded, trimesh.Scene):
        loaded = trimesh.util.concatenate(tuple(loaded.geometry.values()))
    if not isinstance(loaded, trimesh.Trimesh):
        raise RuntimeError(f"unsupported mesh type: {type(loaded)!r}")
    # An OBJ that references an MTL but has no texture map is represented by
    # trimesh as TextureVisuals(image=None).  FoundationPose assumes every
    # TextureVisuals object has an image, so explicitly use neutral vertex
    # colors.  CAD material values are intentionally not treated as truth.
    neutral = np.tile(np.array((140, 140, 140, 255), dtype=np.uint8), (len(loaded.vertices), 1))
    loaded.visual = trimesh.visual.ColorVisuals(mesh=loaded, vertex_colors=neutral)
    return loaded


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-dir", type=Path, default=ROOT / "data" / "live_sample_001")
    parser.add_argument("--mesh", type=Path, default=ROOT / "assets" / "plastic_crate_m.obj")
    parser.add_argument("--metadata", type=Path, default=ROOT / "assets" / "crate_metadata.json")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "live_sample_001")
    parser.add_argument("--iteration", type=int, default=5)
    parser.add_argument("--debug", type=int, default=1)
    args = parser.parse_args()

    color_path = args.scene_dir / "rgb" / "000000.png"
    depth_path = args.scene_dir / "depth" / "000000.png"
    mask_path = args.scene_dir / "masks" / "000000.png"
    camera_path = args.scene_dir / "cam_K.txt"
    for path in (color_path, depth_path, mask_path, camera_path, args.mesh, args.metadata):
        if not path.is_file():
            raise SystemExit(f"missing input: {path}")

    # FoundationPose uses imports relative to its repository root.
    os.chdir(VENDOR)
    sys.path.insert(0, str(VENDOR))
    from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor, dr, set_logging_format, set_seed

    set_logging_format()
    set_seed(0)

    rgb = imageio.imread(color_path)[..., :3]
    depth_mm = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
    mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
    if depth_mm is None or depth_mm.dtype != np.uint16:
        raise RuntimeError("depth must be a 16-bit millimetre PNG")
    if mask is None:
        raise RuntimeError("could not load initialization mask")
    depth = depth_mm.astype(np.float32) * 0.001
    depth[(depth < 0.001) | (depth > 10.0)] = 0.0
    mask = mask > 0
    camera_matrix = np.loadtxt(camera_path, dtype=np.float64).reshape(3, 3)

    mesh = load_mesh(args.mesh)
    symmetry = np.repeat(np.eye(4, dtype=np.float32)[None], 2, axis=0)
    symmetry[1, 0, 0] = -1
    symmetry[1, 1, 1] = -1

    args.output_dir.mkdir(parents=True, exist_ok=True)
    estimator = FoundationPose(
        model_pts=mesh.vertices,
        model_normals=mesh.vertex_normals,
        symmetry_tfs=symmetry,
        mesh=mesh,
        scorer=ScorePredictor(),
        refiner=PoseRefinePredictor(),
        glctx=dr.RasterizeCudaContext(),
        debug_dir=str(args.output_dir / "debug"),
        debug=args.debug,
    )
    pose = estimator.register(
        K=camera_matrix,
        rgb=rgb,
        depth=depth,
        ob_mask=mask,
        iteration=args.iteration,
    )
    if not np.isfinite(pose).all():
        raise RuntimeError("FoundationPose returned non-finite values")
    np.savetxt(args.output_dir / "object_in_camera.txt", pose, fmt="%.9f")

    metadata = json.loads(args.metadata.read_text(encoding="utf-8"))
    outer = np.asarray(metadata["rim_outer_m"], dtype=np.float64)
    inner = np.asarray(metadata["rim_inner_m"], dtype=np.float64)
    outer_pixels = project(outer, pose, camera_matrix)
    inner_pixels = project(inner, pose, camera_matrix)

    overlay = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    draw_closed_polyline(overlay, outer_pixels, (0, 0, 255))
    draw_closed_polyline(overlay, inner_pixels, (0, 255, 0))
    cv2.putText(overlay, "outer rim", (24, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    cv2.putText(overlay, "inner rim", (24, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.imwrite(str(args.output_dir / "rim_projection.png"), overlay)

    rotation = pose[:3, :3]
    result = {
        "object_in_camera": pose.tolist(),
        "translation_m": pose[:3, 3].tolist(),
        "rotation_determinant": float(np.linalg.det(rotation)),
        "orthogonality_error": float(np.linalg.norm(rotation.T @ rotation - np.eye(3))),
        "rim_outer_pixels": outer_pixels.tolist(),
        "rim_inner_pixels": inner_pixels.tolist(),
        "mask_pixels": int(mask.sum()),
        "valid_depth_pixels_in_mask": int(np.count_nonzero(mask & (depth > 0))),
    }
    (args.output_dir / "result.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print(f"saved: {args.output_dir / 'rim_projection.png'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
