"""
后台二维码解码 worker。

主循环只负责提交 ROI；实际 QR 解码放到线程里，避免 zxing/OpenCV
偶发耗时把 RGB 显示、深度接收拖住。
"""

from __future__ import annotations

import threading
import time

from .qr_detector import QRDetector


class QRWorker:
    """后台 QR 解码，避免阻塞抓取主循环。"""

    def __init__(self):
        self._detector = QRDetector()
        self._request = None
        self._result = []
        self._busy = False
        self._stop = False
        self._stats = {
            "decode_ms": 0.0,
            "source": "",
            "mode": "",
            "roi_count": 0,
            "shape": None,
            "result_count": 0,
            "error": "",
        }
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def submit(self, frame, rois, source: str, roi_indices: list[int] | None = None,
               mode: str = "") -> bool:
        with self._lock:
            if self._busy or self._request is not None:
                return False
            self._request = (
                frame.copy(),
                list(rois),
                source,
                list(roi_indices or range(len(rois))),
                mode,
            )
            return True

    def latest(self):
        with self._lock:
            return list(self._result), self._busy, dict(self._stats)

    def backend_info(self) -> str:
        return ", ".join(self._detector.backends)

    def stop(self):
        with self._lock:
            self._stop = True
        self._thread.join(timeout=1.0)

    def _loop(self):
        while True:
            with self._lock:
                if self._stop:
                    return
                request = self._request
                self._request = None
                if request is not None:
                    self._busy = True

            if request is None:
                time.sleep(0.01)
                continue

            frame, rois, source, roi_indices, mode = request
            start = time.time()
            error = ""
            try:
                result = self._detector.detect(frame, rois=rois, heavy=True, search_full=False)
            except Exception as exc:
                result = []
                error = repr(exc)

            if roi_indices:
                mapped_result = []
                for det in result:
                    item = dict(det)
                    roi_index = item.get("roi_index")
                    if roi_index is not None and 0 <= roi_index < len(roi_indices):
                        item["roi_index"] = roi_indices[roi_index]
                    mapped_result.append(item)
                result = mapped_result

            decode_ms = (time.time() - start) * 1000.0
            with self._lock:
                self._result = result
                self._stats = {
                    "decode_ms": decode_ms,
                    "source": source,
                    "mode": mode,
                    "roi_count": len(rois),
                    "shape": frame.shape,
                    "result_count": len(result),
                    "error": error,
                }
                self._busy = False
