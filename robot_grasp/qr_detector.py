"""
二维码检测工具。

优先使用 zxing-cpp / pyzbar 这类原生解码器；不可用时回退到 OpenCV QRCodeDetector。

注意：QR 解码只使用传入的目标 ROI。后续塑料袋建议传入分割 mask 的外接框
或由 mask 裁出的图像区域，不要为了扫码额外扩大到背景。
"""

from __future__ import annotations

import cv2
import numpy as np

try:
    import zxingcpp
except ImportError:
    zxingcpp = None

try:
    from pyzbar import pyzbar
except ImportError:
    pyzbar = None


class QRDetector:
    """二维码检测的轻量封装，支持多个可选后端。"""

    def __init__(self):
        self._detector = cv2.QRCodeDetector()
        self.backends = []
        if zxingcpp is not None:
            self.backends.append("zxing-cpp")
        if pyzbar is not None:
            self.backends.append("pyzbar")
        self.backends.append("opencv")
        try:
            self._detector.setEpsX(0.4)
            self._detector.setEpsY(0.4)
        except AttributeError:
            pass

    def detect(self, frame: np.ndarray, rois: list[tuple[int, int, int, int]] | None = None,
               heavy: bool = False, search_full: bool = True) -> list[dict]:
        """检测并解码一帧图像中的二维码。

        返回:
            list[dict]: 每个元素包含 text、points、center。
        """
        candidates = []
        if search_full:
            for image, scale in self._variants(frame, heavy=heavy):
                candidates.extend(self._detect_on_variant(image, scale))

        for roi_index, roi in enumerate(rois or []):
            candidates.extend(self._detect_in_roi(frame, roi, heavy=heavy, roi_index=roi_index))

        return self._dedupe(candidates)

    def _variants(self, frame: np.ndarray, heavy: bool = False):
        yield frame, 1.0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        yield gray, 1.0

        if heavy:
            equalized = cv2.equalizeHist(gray)
            yield equalized, 1.0

        scales = (2.0,) if not heavy else (1.5, 2.0, 3.0)
        for scale in scales:
            resized = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
            yield resized, scale

    def _roi_variants(self, roi_img: np.ndarray, heavy: bool = False):
        if zxingcpp is not None:
            scales = (2.0, 3.0) if not heavy else (2.0, 3.0, 4.0)
        else:
            scales = (3.0,) if not heavy else (2.0, 3.0, 4.0, 6.0, 8.0)
        for scale in scales:
            resized = cv2.resize(roi_img, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
            yield resized, scale

            gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
            yield gray, scale

            if heavy and zxingcpp is not None:
                clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)).apply(gray)
                yield clahe, scale

                sharpened = cv2.filter2D(
                    gray,
                    -1,
                    np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]]),
                )
                yield sharpened, scale

            if not heavy or zxingcpp is not None:
                continue

            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(gray)
            yield clahe, scale

            sharpened = cv2.filter2D(resized, -1, np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]]))
            yield sharpened, scale

            gray_blur = cv2.GaussianBlur(gray, (3, 3), 0)
            adaptive = cv2.adaptiveThreshold(
                gray_blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY, 31, 3
            )
            yield adaptive, scale

            _, otsu = cv2.threshold(gray_blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            yield otsu, scale
            yield cv2.bitwise_not(otsu), scale

    def _detect_on_variant(self, image: np.ndarray, scale: float) -> list[dict]:
        decoded = []

        decoded.extend(self._detect_with_zxing(image, scale))
        if zxingcpp is not None:
            return decoded

        decoded.extend(self._detect_with_pyzbar(image, scale))

        try:
            ok, texts, points, _ = self._detector.detectAndDecodeMulti(image)
            if ok and points is not None:
                for text, pts in zip(texts, points):
                    if text:
                        decoded.append(self._make_result(text, pts, scale, backend="opencv"))
        except cv2.error:
            pass

        try:
            text, pts, _ = self._detector.detectAndDecode(image)
            if text and pts is not None:
                decoded.append(self._make_result(text, pts, scale, backend="opencv"))
        except cv2.error:
            pass

        return decoded

    def _detect_with_zxing(self, image: np.ndarray, scale: float) -> list[dict]:
        if zxingcpp is None:
            return []

        try:
            scan_image = image
            if image.ndim == 3:
                scan_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = zxingcpp.read_barcodes(scan_image)
        except Exception:
            return []

        decoded = []
        for result in results:
            text = getattr(result, "text", "")
            if not text:
                continue
            pts = self._zxing_points(result)
            if pts is None:
                continue
            decoded.append(self._make_result(text, pts, scale, backend="zxing-cpp"))
        return decoded

    def _zxing_points(self, result) -> np.ndarray | None:
        pos = getattr(result, "position", None)
        if pos is None:
            return None

        points = []
        for name in ("top_left", "top_right", "bottom_right", "bottom_left"):
            point = getattr(pos, name, None)
            if point is None:
                return None
            points.append((float(point.x), float(point.y)))
        return np.asarray(points, dtype=np.float32)

    def _detect_with_pyzbar(self, image: np.ndarray, scale: float) -> list[dict]:
        if pyzbar is None:
            return []

        try:
            results = pyzbar.decode(image)
        except Exception:
            return []

        decoded = []
        for result in results:
            try:
                text = result.data.decode("utf-8")
            except Exception:
                text = str(result.data)
            if not text:
                continue

            polygon = getattr(result, "polygon", None)
            if polygon and len(polygon) >= 4:
                pts = np.asarray([(p.x, p.y) for p in polygon[:4]], dtype=np.float32)
            else:
                rect = result.rect
                x, y, w, h = rect.left, rect.top, rect.width, rect.height
                pts = np.asarray(
                    [(x, y), (x + w, y), (x + w, y + h), (x, y + h)],
                    dtype=np.float32,
                )
            decoded.append(self._make_result(text, pts, scale, backend="pyzbar"))

        return decoded

    def _detect_in_roi(self, frame: np.ndarray, roi: tuple[int, int, int, int],
                       heavy: bool = False, roi_index: int | None = None) -> list[dict]:
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = roi
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(w, x2)
        y2 = min(h, y2)
        if x2 <= x1 or y2 <= y1:
            return []

        roi_img = frame[y1:y2, x1:x2]
        decoded = []
        for image, scale in self._roi_variants(roi_img, heavy=heavy):
            for det in self._detect_on_variant(image, scale):
                decoded.append(self._to_frame_roi_detection(det, x1, y1, roi_index))
        return decoded

    def _to_frame_roi_detection(self, det: dict, x_offset: int, y_offset: int,
                                roi_index: int | None) -> dict:
        pts = det["points"].copy()
        pts[:, 0] += x_offset
        pts[:, 1] += y_offset
        center = pts.mean(axis=0)
        det["points"] = pts
        det["center"] = (int(center[0]), int(center[1]))
        det["source"] = "roi"
        det["roi_index"] = roi_index
        return det

    def _make_result(self, text: str, points: np.ndarray, scale: float, backend: str) -> dict:
        pts = np.asarray(points, dtype=np.float32).reshape(-1, 2) / scale
        center = pts.mean(axis=0)
        return {
            "text": text,
            "points": pts,
            "center": (int(center[0]), int(center[1])),
            "source": "full",
            "backend": backend,
        }

    def _dedupe(self, detections: list[dict]) -> list[dict]:
        by_text: dict[str, dict] = {}
        for det in detections:
            text = det["text"]
            if text not in by_text:
                by_text[text] = det
                continue

            old_area = self._quad_area(by_text[text]["points"])
            new_area = self._quad_area(det["points"])
            if new_area > old_area:
                by_text[text] = det

        return list(by_text.values())

    def _quad_area(self, points: np.ndarray) -> float:
        pts = points.astype(np.float32)
        return float(abs(cv2.contourArea(pts)))


def draw_qr_detections(frame: np.ndarray, detections: list[dict]) -> np.ndarray:
    """在图像上绘制二维码检测结果。"""
    output = frame.copy()
    for idx, det in enumerate(detections, start=1):
        pts = det["points"].astype(int)
        cv2.polylines(output, [pts], isClosed=True, color=(0, 255, 255), thickness=2)

        cx, cy = det["center"]
        cv2.circle(output, (cx, cy), 4, (0, 255, 255), -1)

        label = f"QR {idx}: {det['text']}"
        if len(label) > 48:
            label = label[:45] + "..."
        x, y = pts[0]
        y = max(20, y - 8)
        cv2.putText(output, label, (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

    return output
