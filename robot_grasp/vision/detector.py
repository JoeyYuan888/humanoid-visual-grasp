"""
YOLO 物体检测器封装
"""

import os
import tempfile

import numpy as np
import torch

os.environ.setdefault("MPLCONFIGDIR", os.path.join(tempfile.gettempdir(), "matplotlib"))

from ultralytics import YOLO

from ..common import config


class Detector:
    """YOLO 检测器。"""

    def __init__(self, target_classes: list[str] | None = None,
                 conf: float | None = None,
                 iou: float | None = None,
                 imgsz: int | None = None):
        self._target_class_names = target_classes if target_classes is not None else config.YOLO_TARGET_CLASSES
        self._conf = conf if conf is not None else config.YOLO_CONF
        self._iou = iou if iou is not None else config.YOLO_IOU
        self._imgsz = imgsz if imgsz is not None else config.YOLO_IMGSZ
        cuda_available = torch.cuda.is_available()
        if config.REQUIRE_CUDA and not cuda_available:
            raise RuntimeError(
                "CUDA 不可用，已拒绝退回 CPU。请在 detect 环境中确认 NVIDIA driver/GPU 可见，"
                "例如运行: python -c \"import torch; print(torch.__version__, torch.cuda.is_available())\""
            )
        self._device = "cuda" if cuda_available else "cpu"
        print(f"[*] 加载 YOLO 模型: {config.YOLO_MODEL}  (device: {self._device})")
        self._model = YOLO(config.YOLO_MODEL)
        # 显式指定设备
        self._model.to(self._device)
        self._target_classes = self._resolve_target_classes()
        # 预热
        self._model(
            np.zeros((480, 640, 3), dtype=np.uint8),
            imgsz=self._imgsz,
            classes=self._target_classes,
            verbose=False,
        )
        print(f"[✓] YOLO 模型加载完成")

    def _resolve_target_classes(self):
        if not self._target_class_names:
            return None

        names = self._model.names
        class_ids = [
            class_id
            for class_id, name in names.items()
            if name in self._target_class_names
        ]
        missing = sorted(set(self._target_class_names) - {names[i] for i in class_ids})
        if missing:
            print(f"[!] 模型类别中找不到: {missing}，将仅在后处理中按名称过滤")
            return None
        return class_ids

    def detect(self, frame: np.ndarray):
        """
        对一帧 RGB 图像执行检测。

        返回:
            list[dict]: 每个检测结果包含:
                - class_id: int
                - label: str
                - confidence: float
                - bbox: (x1, y1, x2, y2)   — 像素坐标
                - center: (cx, cy)          — bbox 中心
        """
        results = self._model(
            frame,
            conf=self._conf,
            iou=self._iou,
            imgsz=self._imgsz,
            classes=self._target_classes,
            device=self._device,
            verbose=False,
        )
        dets = []
        target_set = set(self._target_class_names) if self._target_class_names else None

        for box in results[0].boxes:
            label = results[0].names[int(box.cls[0])]
            # 类别过滤
            if target_set and label not in target_set:
                continue
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            dets.append({
                "class_id": int(box.cls[0]),
                "label": label,
                "confidence": float(box.conf[0]),
                "bbox": (x1, y1, x2, y2),
                "center": (cx, cy),
            })

        dets = self._dedupe_detections(dets)

        # 只绘制需要的框
        annotated = results[0].plot() if not target_set else frame.copy()
        return dets, annotated

    def _dedupe_detections(self, dets: list[dict]) -> list[dict]:
        """同类别近距离重复框只保留最高置信度。"""
        if not dets:
            return dets

        kept = []
        thresh = float(config.YOLO_DEDUP_CENTER_THRESH)
        for det in sorted(dets, key=lambda item: item["confidence"], reverse=True):
            duplicate = False
            for old in kept:
                if old["label"] != det["label"]:
                    continue
                dx = old["center"][0] - det["center"][0]
                dy = old["center"][1] - det["center"][1]
                if (dx * dx + dy * dy) ** 0.5 <= thresh:
                    duplicate = True
                    break
            if not duplicate:
                kept.append(det)
        return kept
