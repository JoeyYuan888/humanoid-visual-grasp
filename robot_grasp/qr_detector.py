"""
二维码检测工具。

优先使用 zxing-cpp / pyzbar 这类原生解码器；不可用时回退到 OpenCV QRCodeDetector。

注意：QR 解码只使用传入的目标 ROI。后续塑料袋建议传入分割 mask 的外接框
或由 mask 裁出的图像区域，不要为了扫码额外扩大到背景。
"""

from __future__ import annotations

import time
from pathlib import Path

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

try:
    from qreader import QReader
except ImportError:
    QReader = None


class QRDetector:
    """二维码检测的轻量封装，支持多个可选后端。"""

    def __init__(self):
        self._detector = cv2.QRCodeDetector()
        self._wechat_detector = None
        self._qreader = None
        self.backends = []
        if zxingcpp is not None:
            self.backends.append("zxing-cpp")
        if pyzbar is not None:
            self.backends.append("pyzbar")
        self._wechat_detector = self._create_wechat_detector()
        if self._wechat_detector is not None:
            self.backends.append("wechat-qrcode")
        if QReader is not None:
            self.backends.append("qreader")
        self.backends.append("opencv")
        try:
            self._detector.setEpsX(0.4)
            self._detector.setEpsY(0.4)
        except AttributeError:
            pass

    def _create_wechat_detector(self):
        """Create OpenCV WeChat QR detector with local detection/SR models."""
        detector_cls = getattr(cv2, "wechat_qrcode_WeChatQRCode", None)
        if detector_cls is None:
            module = getattr(cv2, "wechat_qrcode", None)
            detector_cls = getattr(module, "WeChatQRCode", None) if module is not None else None
        if detector_cls is None:
            return None

        project_root = Path(__file__).resolve().parents[1]
        model_dir = project_root / "models" / "wechat_qrcode"
        model_paths = [
            model_dir / "detect.prototxt",
            model_dir / "detect.caffemodel",
            model_dir / "sr.prototxt",
            model_dir / "sr.caffemodel",
        ]
        try:
            if all(path.exists() for path in model_paths):
                return detector_cls(*(str(path) for path in model_paths))
            return detector_cls()
        except Exception:
            return None

    def detect(self, frame: np.ndarray, rois: list[tuple[int, int, int, int]] | None = None,
               heavy: bool = False, search_full: bool = True, progress=None) -> list[dict]:
        """检测并解码一帧图像中的二维码。

        返回:
            list[dict]: 每个元素包含 text、points、center。
        """
        candidates = []
        if search_full:
            for index, (image, scale) in enumerate(self._variants(frame, heavy=heavy), start=1):
                if progress:
                    progress(f"full variant {index} scale={scale}")
                candidates.extend(self._detect_on_variant(image, scale))
            if heavy:
                if progress:
                    progress("full qreader start")
                candidates.extend(self._detect_with_qreader(frame, roi_index=None))
                if progress:
                    progress("full qreader done")
                if progress:
                    progress("full perspective start")
                candidates.extend(self._detect_perspective_recovered(frame))
                if progress:
                    progress("full perspective done")

        for roi_index, roi in enumerate(rois or []):
            if progress:
                progress(f"roi {roi_index + 1}/{len(rois or [])} start roi={roi}")
            start = time.time()
            candidates.extend(self._detect_in_roi(frame, roi, heavy=heavy, roi_index=roi_index, progress=progress))
            if progress:
                progress(f"roi {roi_index + 1}/{len(rois or [])} done cost={time.time() - start:.2f}s")

        return self._dedupe(candidates)

    def _get_qreader(self):
        if QReader is None:
            return None
        if self._qreader is None:
            try:
                self._qreader = QReader(model_size="n")
            except TypeError:
                try:
                    self._qreader = QReader()
                except Exception:
                    self._qreader = False
            except Exception:
                self._qreader = False
        return self._qreader if self._qreader is not False else None

    def _detect_with_qreader(self, image: np.ndarray, roi_index: int | None = None) -> list[dict]:
        reader = self._get_qreader()
        if reader is None:
            return []
        try:
            rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) if image.ndim == 3 else cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
            texts, detections = reader.detect_and_decode(
                image=rgb,
                return_detections=True,
                is_bgr=False,
            )
        except Exception:
            return []
        decoded = []
        for text, det in zip(texts or [], detections or []):
            if not text:
                continue
            pts = det.get("quad_xy")
            if pts is None:
                pts = det.get("bbox_xyxy")
                if pts is None:
                    continue
                x1, y1, x2, y2 = [float(value) for value in pts]
                pts = np.asarray([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32)
            pts = np.asarray(pts, dtype=np.float32).reshape(4, 2)
            center = pts.mean(axis=0)
            decoded.append({
                "text": text,
                "points": pts,
                "center": (int(center[0]), int(center[1])),
                "source": "full" if roi_index is None else "roi",
                "roi_index": roi_index,
                "backend": "qreader",
            })
        return decoded

    def _variants(self, frame: np.ndarray, heavy: bool = False):
        yield frame, 1.0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        yield gray, 1.0

        if heavy:
            equalized = cv2.equalizeHist(gray)
            yield equalized, 1.0

            sharpened = cv2.filter2D(
                gray,
                -1,
                np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]]),
            )
            yield sharpened, 1.0

            gray_blur = cv2.GaussianBlur(gray, (3, 3), 0)
            adaptive = cv2.adaptiveThreshold(
                gray_blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY, 31, 3
            )
            yield adaptive, 1.0

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

            if not heavy:
                continue

            clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)).apply(gray)
            yield clahe, scale

            sharpened_gray = cv2.filter2D(
                gray,
                -1,
                np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]]),
            )
            yield sharpened_gray, scale

            sharpened_bgr = cv2.filter2D(resized, -1, np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]]))
            yield sharpened_bgr, scale

            gray_blur = cv2.GaussianBlur(gray, (3, 3), 0)
            adaptive = cv2.adaptiveThreshold(
                gray_blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY, 31, 3
            )
            yield adaptive, scale

            _, otsu = cv2.threshold(gray_blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            yield otsu, scale
            yield cv2.bitwise_not(otsu), scale

            for angle in (-20.0, -10.0, 10.0, 20.0):
                rotated = self._rotate_keep_size(resized, angle)
                yield rotated, scale
                rotated_gray = cv2.cvtColor(rotated, cv2.COLOR_BGR2GRAY)
                yield rotated_gray, scale

    def _rotate_keep_size(self, image: np.ndarray, angle_deg: float) -> np.ndarray:
        h, w = image.shape[:2]
        matrix = cv2.getRotationMatrix2D((w / 2.0, h / 2.0), angle_deg, 1.0)
        border = 255 if image.ndim == 2 else (255, 255, 255)
        return cv2.warpAffine(
            image,
            matrix,
            (w, h),
            flags=cv2.INTER_CUBIC,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=border,
        )

    def _detect_on_variant(self, image: np.ndarray, scale: float) -> list[dict]:
        decoded = []

        decoded.extend(self._detect_with_zxing(image, scale))
        if decoded:
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

    def _detect_perspective_recovered(self, frame: np.ndarray) -> list[dict]:
        """Detect QR quadrangles, rectify them, then decode the rectified patch.

        This handles cases where the QR is visible but skewed enough that normal
        decode fails. The returned points stay in the original frame.
        """
        decoded = []
        for image, scale in self._perspective_detect_variants(frame):
            quads = self._detect_qr_quads(image)
            for quad in quads:
                original_quad = np.asarray(quad, dtype=np.float32).reshape(4, 2) / scale
                warped = self._warp_quad(frame, original_quad)
                if warped is None:
                    continue
                for text, backend in self._decode_texts(warped):
                    decoded.append(self._make_result_from_original_quad(
                        text,
                        original_quad,
                        backend=f"{backend}+perspective",
                    ))
        return decoded

    def _perspective_detect_variants(self, frame: np.ndarray):
        yield frame, 1.0
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        yield gray, 1.0
        equalized = cv2.equalizeHist(gray)
        yield equalized, 1.0
        for scale in (1.5, 2.0):
            resized = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
            yield resized, scale
            resized_gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
            yield resized_gray, scale

    def _detect_qr_quads(self, image: np.ndarray) -> list[np.ndarray]:
        quads = []
        try:
            ok, points = self._detector.detectMulti(image)
            if ok and points is not None:
                for pts in points:
                    quads.append(np.asarray(pts, dtype=np.float32).reshape(4, 2))
        except cv2.error:
            pass

        try:
            ok, points = self._detector.detect(image)
            if ok and points is not None:
                quads.append(np.asarray(points, dtype=np.float32).reshape(4, 2))
        except cv2.error:
            pass

        return self._dedupe_quads(quads)

    def _dedupe_quads(self, quads: list[np.ndarray]) -> list[np.ndarray]:
        kept = []
        for quad in quads:
            center = quad.mean(axis=0)
            area = self._quad_area(quad)
            if area < 100.0:
                continue
            duplicate = False
            for old in kept:
                old_center = old.mean(axis=0)
                if np.linalg.norm(center - old_center) < 10.0:
                    duplicate = True
                    break
            if not duplicate:
                kept.append(quad)
        return kept

    def _warp_quad(self, frame: np.ndarray, quad: np.ndarray) -> np.ndarray | None:
        ordered = self._order_quad_points(quad)
        tl, tr, br, bl = ordered
        width_a = np.linalg.norm(br - bl)
        width_b = np.linalg.norm(tr - tl)
        height_a = np.linalg.norm(tr - br)
        height_b = np.linalg.norm(tl - bl)
        side = int(max(width_a, width_b, height_a, height_b))
        side = max(160, min(720, side))
        if side <= 0:
            return None

        dst = np.asarray(
            [
                [0, 0],
                [side - 1, 0],
                [side - 1, side - 1],
                [0, side - 1],
            ],
            dtype=np.float32,
        )
        try:
            matrix = cv2.getPerspectiveTransform(ordered, dst)
            warped = cv2.warpPerspective(frame, matrix, (side, side))
        except cv2.error:
            return None
        return cv2.copyMakeBorder(
            warped,
            24,
            24,
            24,
            24,
            borderType=cv2.BORDER_CONSTANT,
            value=(255, 255, 255),
        )

    def _order_quad_points(self, points: np.ndarray) -> np.ndarray:
        pts = np.asarray(points, dtype=np.float32).reshape(4, 2)
        ordered = np.zeros((4, 2), dtype=np.float32)
        sums = pts.sum(axis=1)
        diffs = np.diff(pts, axis=1).reshape(-1)
        ordered[0] = pts[np.argmin(sums)]
        ordered[2] = pts[np.argmax(sums)]
        ordered[1] = pts[np.argmin(diffs)]
        ordered[3] = pts[np.argmax(diffs)]
        return ordered

    def _decode_texts(self, image: np.ndarray) -> list[tuple[str, str]]:
        decoded = []

        if zxingcpp is not None:
            try:
                scan_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) if image.ndim == 3 else image
                for result in zxingcpp.read_barcodes(scan_image):
                    text = getattr(result, "text", "")
                    if text:
                        decoded.append((text, "zxing-cpp"))
            except Exception:
                pass

        if pyzbar is not None:
            try:
                for result in pyzbar.decode(image):
                    try:
                        text = result.data.decode("utf-8")
                    except Exception:
                        text = str(result.data)
                    if text:
                        decoded.append((text, "pyzbar"))
            except Exception:
                pass

        if self._wechat_detector is not None:
            try:
                texts, _ = self._wechat_detector.detectAndDecode(image)
                for text in texts or []:
                    if text:
                        decoded.append((text, "wechat-qrcode"))
            except Exception:
                pass

        try:
            ok, texts, _, _ = self._detector.detectAndDecodeMulti(image)
            if ok:
                for text in texts:
                    if text:
                        decoded.append((text, "opencv"))
        except cv2.error:
            pass

        try:
            text, _, _ = self._detector.detectAndDecode(image)
            if text:
                decoded.append((text, "opencv"))
        except cv2.error:
            pass

        unique = []
        seen = set()
        for text, backend in decoded:
            if text in seen:
                continue
            seen.add(text)
            unique.append((text, backend))
        return unique

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
                       heavy: bool = False, roi_index: int | None = None, progress=None) -> list[dict]:
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
        if heavy:
            if progress:
                progress("roi qreader start")
            for det in self._detect_with_qreader(roi_img, roi_index=roi_index):
                decoded.append(self._to_frame_roi_detection(det, x1, y1, roi_index))
            if progress:
                progress(f"roi qreader done decoded={len(decoded)}")
            if decoded:
                return decoded

            if progress:
                progress("roi sliding-window fallback start")
            for det in self._detect_sliding_windows(roi_img, progress=progress):
                decoded.append(self._to_frame_roi_detection(det, x1, y1, roi_index))
            if progress:
                progress(f"roi sliding-window fallback done decoded={len(decoded)}")
            if decoded:
                return decoded

            if progress:
                progress("roi perspective start")
            for det in self._detect_perspective_recovered(roi_img):
                decoded.append(self._to_frame_roi_detection(det, x1, y1, roi_index))
            if progress:
                progress(f"roi perspective done decoded={len(decoded)}")
            if decoded:
                return decoded

            if min(roi_img.shape[:2]) > 220:
                return decoded

        for variant_index, (image, scale) in enumerate(self._roi_variants(roi_img, heavy=heavy), start=1):
            if progress:
                progress(f"roi variant {variant_index} scale={scale} shape={image.shape[:2]}")
            for det in self._detect_on_variant(image, scale):
                decoded.append(self._to_frame_roi_detection(det, x1, y1, roi_index))
            if decoded:
                if progress:
                    progress(f"roi variant {variant_index} decoded={len(decoded)}")
                break
        if heavy:
            if progress:
                progress("roi perspective start")
            for det in self._detect_perspective_recovered(roi_img):
                decoded.append(self._to_frame_roi_detection(det, x1, y1, roi_index))
            if progress:
                progress(f"roi perspective done decoded={len(decoded)}")
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

    def _make_result_from_original_quad(self, text: str, points: np.ndarray, backend: str) -> dict:
        pts = np.asarray(points, dtype=np.float32).reshape(4, 2)
        center = pts.mean(axis=0)
        return {
            "text": text,
            "points": pts,
            "center": (int(center[0]), int(center[1])),
            "source": "full-perspective",
            "backend": backend,
        }

    def _detect_sliding_windows(self, frame: np.ndarray, progress=None) -> list[dict]:
        """Fallback for small QR labels anywhere inside the object ROI.

        The phone succeeds on these frames because it effectively crops and
        enhances the label area. This scans a bounded grid of windows across
        the ROI, then maps detections back to the ROI frame.
        """
        if zxingcpp is None:
            return []

        h, w = frame.shape[:2]
        if h < 40 or w < 40:
            return []

        x_step = max(18, int(w * 0.03))
        y_step = max(14, int(h * 0.03))
        x_starts = range(0, max(1, int(0.86 * w)), x_step)
        y_starts = range(0, max(1, int(0.76 * h)), y_step)
        widths = [int(w * ratio) for ratio in (0.22, 0.28, 0.34)]
        heights = [int(h * ratio) for ratio in (0.22, 0.32, 0.42)]

        checked = 0
        for x1 in x_starts:
            for y1 in y_starts:
                for ww in widths:
                    for hh in heights:
                        x2 = min(w, x1 + max(50, ww))
                        y2 = min(h, y1 + max(50, hh))
                        if x2 - x1 < 50 or y2 - y1 < 50:
                            continue
                        checked += 1
                        crop = frame[y1:y2, x1:x2]
                        for prepared, scale in self._window_decode_variants(crop):
                            texts = self._decode_texts_with_backend(prepared, backend_filter="zxing-cpp")
                            if not texts:
                                continue
                            if progress:
                                progress(
                                    "sliding-window hit "
                                    f"box=({x1},{y1},{x2},{y2}) scale={scale} checked={checked}"
                                )
                            return [
                                self._make_result_from_window(
                                    text,
                                    (x1, y1, x2, y2),
                                    backend=f"{backend}+sliding-window",
                                )
                                for text, backend in texts
                            ]
        if progress:
            progress(f"sliding-window checked={checked}")
        return []

    def _window_decode_variants(self, crop: np.ndarray):
        for scale in (1.0, 2.0):
            resized = crop if scale == 1.0 else cv2.resize(
                crop,
                None,
                fx=scale,
                fy=scale,
                interpolation=cv2.INTER_CUBIC,
            )
            bordered = cv2.copyMakeBorder(
                resized,
                20,
                20,
                20,
                20,
                borderType=cv2.BORDER_CONSTANT,
                value=(255, 255, 255),
            )
            yield bordered, scale

            gray = cv2.cvtColor(bordered, cv2.COLOR_BGR2GRAY)
            yield gray, scale

    def _decode_texts_with_backend(self, image: np.ndarray, backend_filter: str | None = None) -> list[tuple[str, str]]:
        decoded = []
        seen = set()

        if backend_filter in (None, "zxing-cpp") and zxingcpp is not None:
            try:
                scan_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) if image.ndim == 3 else image
                results = zxingcpp.read_barcodes(
                    scan_image,
                    formats=zxingcpp.BarcodeFormat.QRCode,
                    try_rotate=True,
                    try_downscale=False,
                    try_invert=True,
                    return_errors=False,
                )
            except Exception:
                results = []
            for result in results:
                text = getattr(result, "text", "")
                if not text or text in seen:
                    continue
                seen.add(text)
                decoded.append((text, "zxing-cpp"))

        if backend_filter in (None, "wechat-qrcode") and self._wechat_detector is not None:
            try:
                texts, _ = self._wechat_detector.detectAndDecode(image)
            except Exception:
                texts = []
            for text in texts or []:
                if not text or text in seen:
                    continue
                seen.add(text)
                decoded.append((text, "wechat-qrcode"))
        return decoded

    def _make_result_from_window(self, text: str, box: tuple[int, int, int, int], backend: str) -> dict:
        x1, y1, x2, y2 = box
        pts = np.asarray(
            [
                [x1, y1],
                [x2, y1],
                [x2, y2],
                [x1, y2],
            ],
            dtype=np.float32,
        )
        center = pts.mean(axis=0)
        return {
            "text": text,
            "points": pts,
            "center": (int(center[0]), int(center[1])),
            "source": "roi-sliding-window",
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
