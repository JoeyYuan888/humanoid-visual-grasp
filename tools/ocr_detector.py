"""
文字识别（OCR）测试 — 机器人头部 RealSense RGB 图像。

针对 NVIDIA Orin 优化:
  - 使用 ONNX Runtime + TensorRT 推理 PP-OCRv3 识别模型
  - YOLO 提供 ROI → recognition-only（跳过检测模型）→ 每 ROI ~1-3ms
  - 纯 GPU 流水线: YOLO(Orin GPU) → ROI → ONNX TensorRT(Orin GPU)

Orin 上安装:
    pip install onnxruntime-gpu

模型准备（两种方式）:
  方式 A — 自动下载（推荐，首次运行会自动下载）:
      脚本会自动下载 PP-OCRv3 识别 ONNX 模型和字典文件

  方式 B — 手动转换（离线环境）:
      在任何有网机器上:
        pip install paddle2onnx
        wget https://paddleocr.bj.bcebos.com/PP-OCRv3/chinese/ch_PP-OCRv3_rec_infer.tar
        tar xf ch_PP-OCRv3_rec_infer.tar
        paddle2onnx --model_dir ./ch_PP-OCRv3_rec_infer \
                    --model_filename inference.pdmodel \
                    --params_filename inference.pdiparams \
                    --save_file ./models/ch_PP-OCRv3_rec.onnx \
                    --opset_version 15
      把生成的 ch_PP-OCRv3_rec.onnx 放到 Orin 的 models/ 目录

用法:
    python tools/ocr_detector.py

窗口中:
    q 退出
    s 保存当前标注画面到 data/ocr_snapshot_*.png
    r 保存当前 ROI 裁剪图到 data/ocr_roi_*.png
"""

from __future__ import annotations
import base64
import csv
import os
import sys
import threading
import time
import urllib.request
import tarfile
from datetime import datetime

import cv2
import numpy as np

_DEBUG_SAVE = False  # 调试中间图保存开关（默认关，避免 Orin 上频繁写盘）

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot_grasp import config

try:
    import roslibpy
    _ROS_OK = True
except ImportError:
    roslibpy = None
    _ROS_OK = False

# ── ONNX Runtime（Orin 上唯一推理后端） ───────────────────
#
# CPU 版可用 pip install onnxruntime（调试用）
# Orin 上必须装 GPU 版: pip install onnxruntime-gpu
# TensorRT EP 由 onnxruntime-gpu 自动加载

try:
    import onnxruntime as ort
    _ORT_OK = True
except ImportError:
    ort = None
    _ORT_OK = False

# ── 配置（Orin 优化默认值） ──────────────────────────────

OCR_OBJECT_CLASSES = ["plastic bag"]  # models/best.pt 是塑料包模型
OCR_OBJECT_CONF = 0.10
DETECT_EVERY_N_FRAMES = 5
OCR_DECODE_INTERVAL_SEC = 0.8
RAW_RGB_THROTTLE_MS = 3000
OCR_MEMORY_TTL_SEC = 10.0
OCR_CONF_THRESHOLD = 0.3
_USE_QR_DESKEW = True    # QR 角度纠偏开关（实际倾斜标签上有效，启用；minAreaRect 兜底）

# 模型路径（相对于 PROJECT_ROOT）
MODEL_DIR = os.path.join(PROJECT_ROOT, "models")
ONNX_REC_PATH = os.path.join(MODEL_DIR, "ch_PP-OCRv6_rec_small.onnx")
ONNX_DET_PATH = os.path.join(MODEL_DIR, "ch_PP-OCRv3_det.onnx")
CHAR_DICT_PATH = os.path.join(MODEL_DIR, "ppocrv6_dict.txt")

# 模型下载 URL
CHAR_DICT_URL = (
    "https://raw.githubusercontent.com/PaddlePaddle/PaddleOCR/"
    "main/ppocr/utils/ppocr_keys_v1.txt"
)
# PP-OCRv3 识别模型
PADDLE_REC_TAR_URL = (
    "https://paddleocr.bj.bcebos.com/PP-OCRv3/chinese/ch_PP-OCRv3_rec_infer.tar"
)
PADDLE_REC_DIR = os.path.join(MODEL_DIR, "ch_PP-OCRv3_rec_infer")
# PP-OCRv3 检测模型
PADDLE_DET_TAR_URL = (
    "https://paddleocr.bj.bcebos.com/PP-OCRv3/chinese/ch_PP-OCRv3_det_infer.tar"
)
PADDLE_DET_DIR = os.path.join(MODEL_DIR, "ch_PP-OCRv3_det_infer")

# ── 全局状态 ──────────────────────────────────────────────

latest_frame = None
latest_raw_frame = None
latest_raw_time = 0.0
frame_count = 0
raw_frame_count = 0
frame_lock = threading.Lock()
raw_frame_lock = threading.Lock()

# ── 模型管理 ──────────────────────────────────────────────


def _download(url, dest, desc="文件"):
    """下载文件。优先用 requests，回退到 urllib。"""
    print(f"[*] 下载 {desc} ...")
    os.makedirs(os.path.dirname(dest) or ".", exist_ok=True)

    # 优先 requests（SSL 处理更好）
    try:
        import requests as _req
        resp = _req.get(url, timeout=60, stream=True)
        resp.raise_for_status()
        with open(dest, "wb") as f:
            for chunk in resp.iter_content(chunk_size=65536):
                if chunk:
                    f.write(chunk)
        print(f"[✓] {desc} 已保存: {dest}")
        return True
    except ImportError:
        pass
    except Exception as e:
        print(f"[!] requests 下载失败: {e}")

    # 回退 urllib
    try:
        urllib.request.urlretrieve(url, dest)
        print(f"[✓] {desc} 已保存: {dest}")
        return True
    except Exception as e:
        print(f"[✗] 下载失败: {e}")

    # 最后尝试：不验证证书
    try:
        import ssl
        ctx = ssl._create_unverified_context()
        with urllib.request.urlopen(url, context=ctx, timeout=30) as resp:
            with open(dest, "wb") as f:
                f.write(resp.read())
        print(f"[✓] {desc} 已保存: {dest} (不验证证书)")
        return True
    except Exception as e:
        print(f"[✗] 不验证证书也失败: {e}")

    return False


def _ensure_dict():
    """确保 v6 字典存在。

    注意：v6 识别模型需要 18708 字符的 ppocrv6_dict.txt，
    不能误用 v3 的 ppocr_keys_v1.txt（只有 6623 字符）。
    """
    if os.path.exists(CHAR_DICT_PATH):
        # 验证行数，防止 v3 字典被误写入 v6 路径
        with open(CHAR_DICT_PATH, "r", encoding="utf-8") as f:
            count = sum(1 for _ in f)
        if count < 10000:
            print(f"[✗] 字典文件行数异常（{count} 行），疑似 v3 字典误写入 v6 路径")
            print("    请手动放置正确的 ppocrv6_dict.txt (18708 字符)")
            return False
        return True
    print("[*] 缺少 v6 字典文件（ppocrv6_dict.txt），请手动放置到 models/")
    print("    可从 RapidOCR 仓库或 HuggingFace 获取（18708 字符）")
    return False


def _ensure_onnx_model():
    """确保 ONNX 模型存在。

    尝试顺序:
      1. 已存在 ONNX 文件
      2. 从社区源下载预转换 ONNX
      3. Paddle 推理模型 → paddle2onnx 转换
      4. 打印手动转换指引
    """
    if os.path.exists(ONNX_REC_PATH):
        return True

    os.makedirs(MODEL_DIR, exist_ok=True)
    print("[*] 未找到 ONNX 识别模型，尝试自动获取 ...")

    import importlib.util
    if importlib.util.find_spec("paddle2onnx") is None:
        print("[!] 未安装 paddle2onnx，无法自动转换。")
        _print_manual_setup()
        return False

    # 下载 Paddle 推理模型
    tar_path = os.path.join(MODEL_DIR, "ch_PP-OCRv3_rec_infer.tar")
    if not os.path.exists(tar_path):
        print("[*] 下载 PP-OCRv3 推理模型 ...")
        if not _download(PADDLE_REC_TAR_URL, tar_path, "PP-OCRv3 识别模型"):
            print("[✗] 下载失败，无法自动准备模型")
            _print_manual_setup()
            return False

    # 解压
    if not os.path.exists(PADDLE_REC_DIR):
        print("[*] 解压推理模型 ...")
        with tarfile.open(tar_path, "r") as tar:
            tar.extractall(path=MODEL_DIR)

    # 转换为 ONNX
    print("[*] 转换为 ONNX 格式 ...")
    import subprocess

    ret = subprocess.call([
        sys.executable, "-m", "paddle2onnx",
        "--model_dir", PADDLE_REC_DIR,
        "--model_filename", "inference.pdmodel",
        "--params_filename", "inference.pdiparams",
        "--save_file", ONNX_REC_PATH,
        "--opset_version", "15",
    ])
    if ret == 0 and os.path.exists(ONNX_REC_PATH):
        print(f"[✓] ONNX 模型已生成: {ONNX_REC_PATH}")
        return True

    print("[✗] ONNX 转换失败")
    _print_manual_setup()
    return False


def _print_manual_setup():
    model_dir = MODEL_DIR.replace(PROJECT_ROOT, "$PROJECT_ROOT")
    print()
    print("=" * 60)
    print("  手动设置指引")
    print("=" * 60)
    print(f"  在任何有网机器上:")
    print(f"    pip install paddle2onnx")
    print(f"    wget {PADDLE_REC_TAR_URL}")
    tar_name = os.path.basename(PADDLE_REC_TAR_URL).replace(".tar", "")
    print(f"    tar xf {os.path.basename(PADDLE_REC_TAR_URL)}")
    print(f"    paddle2onnx --model_dir ./{tar_name} \\")
    print(f"                --model_filename inference.pdmodel \\")
    print(f"                --params_filename inference.pdiparams \\")
    print(f"                --save_file {model_dir}/ch_PP-OCRv3_rec.onnx \\")
    print(f"                --opset_version 15")
    print(f"  将生成的 ch_PP-OCRv3_rec.onnx 复制到 {model_dir}/")
    print("=" * 60)
    print()

# ── ROS 回调 ──────────────────────────────────────────────


rgb_rx_count = 0       # 收到的 compressed 消息数
rgb_decode_fail = 0    # 解码失败数


_msg_sampled = False


def on_rgb_message(message):
    global latest_frame, frame_count, rgb_rx_count, rgb_decode_fail, _msg_sampled
    rgb_rx_count += 1
    try:
        data_b64 = message.get("data")
        if not _msg_sampled:
            _msg_sampled = True
            print(f"[MSG] 消息字段: {list(message.keys())}")
            print(f"[MSG] encoding: {message.get('encoding', 'N/A')}")
            print(f"[MSG] data 类型: {type(data_b64)}, 长度: {len(data_b64) if data_b64 else 0}")
            if data_b64:
                raw = base64.b64decode(data_b64)
                print(f"[MSG] 解码后前16字节: {raw[:16].hex()}")
                print(f"[MSG] 前16字节ASCII: {raw[:16]}")
        if not data_b64:
            return
        img_bytes = base64.b64decode(data_b64)
        img_array = np.frombuffer(img_bytes, dtype=np.uint8)
        frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        if frame is None:
            rgb_decode_fail += 1
            return
        with frame_lock:
            latest_frame = frame
            frame_count += 1
    except Exception:
        rgb_decode_fail += 1


def on_raw_rgb_message(message):
    global latest_raw_frame, latest_raw_time, raw_frame_count
    try:
        height = message.get("height", 0)
        width = message.get("width", 0)
        encoding = message.get("encoding", "")
        data_b64 = message.get("data")
        if not data_b64 or height == 0 or width == 0:
            return
        raw = base64.b64decode(data_b64)
        arr = np.frombuffer(raw, dtype=np.uint8)
        if encoding in ("rgb8", "bgr8"):
            frame = arr.reshape((height, width, 3))
            if encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif encoding in ("mono8", "8UC1"):
            gray = arr.reshape((height, width))
            frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        else:
            return
        with raw_frame_lock:
            latest_raw_frame = frame
            latest_raw_time = time.time()
            raw_frame_count += 1
    except Exception:
        pass


def on_connection():
    print("[✓] 已连接到机器人")
    print(f"    RGB: {config.TOPIC_RGB}")
    print(f"    RAW: {config.TOPIC_RGB_RAW} throttle={RAW_RGB_THROTTLE_MS}ms")
    print()

# ── 工具函数 ──────────────────────────────────────────────


def _safe_terminate(client):
    try:
        client.terminate()
    except Exception:
        pass


def _save_snapshot(frame, prefix="ocr_snapshot"):
    os.makedirs("data", exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join("data", f"{prefix}_{ts}.png")
    cv2.imwrite(path, frame)
    print(f"[✓] 已保存截图: {path}")


def _save_rois(frame, rois, prefix="ocr_roi"):
    os.makedirs("data", exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    h, w = frame.shape[:2]
    for idx, (x1, y1, x2, y2) in enumerate(rois):
        x1, x2 = max(0, x1), min(w, x2)
        y1, y2 = max(0, y1), min(h, y2)
        if x2 <= x1 or y2 <= y1:
            continue
        path = os.path.join("data", f"{prefix}_{ts}_roi{idx}.png")
        cv2.imwrite(path, frame[y1:y2, x1:x2])
        print(f"[✓] 已保存 ROI: {path}")


def _save_debug_rois(frame, raw_frame, object_rois):
    _save_rois(frame, object_rois, prefix="ocr_tight_compressed")
    if raw_frame is not None:
        raw_tight = _scale_rois(object_rois, frame.shape, raw_frame.shape)
        _save_rois(raw_frame, raw_tight, prefix="ocr_tight_raw")


def _scale_rois(rois, src_shape, dst_shape):
    src_h, src_w = src_shape[:2]
    dst_h, dst_w = dst_shape[:2]
    if src_w == 0 or src_h == 0:
        return rois
    sx = dst_w / src_w
    sy = dst_h / src_h
    return [
        (int(x1 * sx), int(y1 * sy), int(x2 * sx), int(y2 * sy))
        for x1, y1, x2, y2 in rois
    ]


def _roi_summary(rois):
    return ";".join(f"{x1},{y1},{x2},{y2}" for x1, y1, x2, y2 in rois)


def _detection_summary(detections):
    return ";".join(
        f"{d['label']}:{d['confidence']:.3f}@"
        f"{d['bbox'][0]},{d['bbox'][1]},{d['bbox'][2]},{d['bbox'][3]}"
        for d in detections
    )

# ── OCR 核心：ONNX Runtime + TensorRT ─────────────────────
#
# 架构:
#   YOLO ROI → 水平投影切分行 → 每行独立 PP-OCRv3 识别 → TensorRT EP
#
# 多行文字处理:
#   同一个 ROI（如瓶身标签）内先通过 horizontal projection 找到每行文字，
#   逐行裁剪后送入 recognition-only 模型，支持多行独立输出。


def _split_text_lines(patch: np.ndarray,
                      min_line_h: int = 8,
                      pad: int = 2) -> list[tuple[int, int, np.ndarray]]:
    """用水平投影法将 ROI 内的文字切成单行。

    返回:
        list[(y1, y2, crop)] — 每行在原 patch 内的 y 范围 + 裁剪图
    """
    h, w = patch.shape[:2]
    if h < min_line_h:
        return [(0, h, patch)]

    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    # 反色二值化：文字=白，背景=黑
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # 水平投影：每行白色像素数
    proj = binary.sum(axis=1) // 255
    threshold = max(3, w * 0.02)  # ≥2% 的宽度算有文字

    in_text = proj > threshold

    # 分组连续行
    lines = []
    start = None
    for i in range(h):
        if in_text[i] and start is None:
            start = i
        elif not in_text[i] and start is not None:
            if i - start >= min_line_h:
                lines.append((start, i))
            start = None
    if start is not None and h - start >= min_line_h:
        lines.append((start, h))

    if not lines:
        # 投影没找到行 → 用整张图
        return [(0, h, patch)]

    # 合并间距太近的行（同一行被噪声断开）
    merged = [lines[0]]
    for y1, y2 in lines[1:]:
        if y1 - merged[-1][1] < pad * 2:
            merged[-1] = (merged[-1][0], y2)
        else:
            merged.append((y1, y2))
    lines = merged

    # 把行裁出来，带 padding
    result = []
    for y1, y2 in lines:
        y1 = max(0, y1 - pad)
        y2 = min(h, y2 + pad)
        line_patch = patch[y1:y2, :]
        if line_patch.size == 0:
            continue
        result.append((y1, y2, line_patch))
    return result


def _to_binary_text(crop: np.ndarray) -> np.ndarray:
    """转成纯黑白二值图：白底黑字，去除所有噪点和背景干扰。

    管道:
      1. 灰度 + OTSU 二值化（自适阈值分黑白）
      2. 形态学闭运算（填充文字断裂/空洞）
      3. 文字膨胀加粗（细笔画变清晰）
      4. 转回 BGR 3 通道（识别模型需要）
    """
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)

    # OTSU 自动阈值：黑字在前（inverse 让文字变白）
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # 闭运算：填充文字内部的空洞和断裂
    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_close)

    # 轻微膨胀：加粗细笔画
    kernel_dilate = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
    dilated = cv2.dilate(closed, kernel_dilate, iterations=1)

    # 反转回黑底白字（视觉上白底黑字更自然）
    result = cv2.bitwise_not(dilated)

    # 转回 3 通道
    return cv2.cvtColor(result, cv2.COLOR_GRAY2BGR)


def _enhance_label(crop: np.ndarray) -> np.ndarray:
    """针对白底黑字标签的文字增强。

    透视校正后的标签做最后润色：
      1. 灰度 + 对比度拉伸（让背景更白、文字更黑）
      2. 双边滤波（去噪同时保持文字边缘）
      3. 轻度锐化（比通用 unsharp mask 更温和）
    """
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)

    # 对比度拉伸：把灰度范围拉满到 [0, 255]
    p_low, p_high = np.percentile(gray, (2, 98))
    if p_high > p_low:
        stretched = np.clip(
            (gray.astype(np.float32) - p_low) * 255.0 / (p_high - p_low), 0, 255
        ).astype(np.uint8)
    else:
        stretched = gray

    # 双边滤波：去噪同时保留文字边缘
    denoised = cv2.bilateralFilter(stretched, d=7, sigmaColor=20, sigmaSpace=20)

    # 转回 BGR
    result = cv2.cvtColor(denoised, cv2.COLOR_GRAY2BGR)

    # 轻度锐化
    blurred = cv2.GaussianBlur(result, (0, 0), 1.0)
    result = cv2.addWeighted(result, 1.4, blurred, -0.4, 0)

    return result


# ── 文字纠偏 ────────────────────────────────────────────


def _deskew_crop(crop: np.ndarray) -> np.ndarray:
    """检测文字倾斜角度并旋转摆正。

    水平检测框 + 倾斜文字 → 框内文字是歪的，识别模型认不出。
    用 minAreaRect 找到文字主方向，反旋转使文字水平。
    """
    h, w = crop.shape[:2]
    if h < 10 or w < 10:
        return crop

    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    pts = cv2.findNonZero(binary)
    if pts is None or len(pts) < 10:
        return crop

    rect = cv2.minAreaRect(pts)
    angle = rect[2]

    if angle < -45:
        angle = 90 + angle

    if abs(angle) < 2.0:
        return crop

    center = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated = cv2.warpAffine(crop, M, (w, h), flags=cv2.INTER_CUBIC,
                             borderMode=cv2.BORDER_CONSTANT, borderValue=(255, 255, 255))
    return rotated


# ── QR 定位 + 旋转摆正 ────────────────────────────────


def _find_qr_finders(patch: np.ndarray,
                     upscale: float = 3.0) -> list[tuple[int, int]]:
    """检测 QR 码的 3 个回字定位图案中心。

    回字定位图案 = 同心黑方块（外黑 → 内白 → 内黑），比完整 QR 四边形可靠。
    即使 QR 太小解不出内容，3 个定位图案也能被轮廓分析找到。

    返回:
        [(x1,y1), (x2,y2), (x3,y3)] 三个定位图案中心（patch 坐标）
    """
    h, w = patch.shape[:2]
    if h < 30 or w < 30:
        return []

    if upscale > 1.0:
        img = cv2.resize(patch, None, fx=upscale, fy=upscale,
                         interpolation=cv2.INTER_CUBIC)
        inv = 1.0 / upscale
    else:
        img = patch
        inv = 1.0

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return []
    h_arr = hierarchy[0]

    def _is_squareish(c):
        peri = cv2.arcLength(c, True)
        if peri <= 0:
            return False
        return len(cv2.approxPolyDP(c, 0.05 * peri, True)) == 4

    centers = []
    for i, cnt in enumerate(contours):
        child = h_arr[i][2]
        if child < 0:
            continue
        grandchild = h_arr[child][2]
        if grandchild < 0:
            continue
        a0 = cv2.contourArea(cnt)
        a1 = cv2.contourArea(contours[child])
        a2 = cv2.contourArea(contours[grandchild])
        # 三层嵌套 + 面积递减 + 外/中层方形（内层小，可能不算规则四边形）
        if not (a0 > 50 and a0 > a1 > a2 > 5):
            continue
        if not (_is_squareish(cnt) and _is_squareish(contours[child])):
            continue
        if a0 / max(a2, 1) < 3:
            continue
        M = cv2.moments(cnt)
        if M["m00"] > 0:
            centers.append((
                int(M["m10"] / M["m00"] * inv),
                int(M["m01"] / M["m00"] * inv),
            ))

    # 只保留互相靠近的（同一 QR 的 3 个定位图案）
    if len(centers) > 3:
        centers = centers[:3]
    return centers


def _order_finders(pts: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """将 3 个定位点排序为 [TL, TR, BL]（左上、右上、左下）。"""
    pts = np.array(pts, dtype=np.float32)
    if len(pts) != 3:
        return pts.tolist()
    d01 = np.linalg.norm(pts[0] - pts[1])
    d02 = np.linalg.norm(pts[0] - pts[2])
    d12 = np.linalg.norm(pts[1] - pts[2])
    pairs = sorted([(d01, 0, 1), (d02, 0, 2), (d12, 1, 2)], key=lambda x: x[0])
    (_, a1, b1), (_, a2, b2), _ = pairs
    tl_idx = a1 if (a1 == a2 or a1 == b2) else b1
    others = [i for i in range(3) if i != tl_idx]
    o1, o2 = others
    tl = pts[tl_idx]
    v1 = pts[o1] - tl
    v2 = pts[o2] - tl
    ang1 = abs(np.degrees(np.arctan2(v1[1], v1[0])))
    ang2 = abs(np.degrees(np.arctan2(v2[1], v2[0])))
    if ang1 < ang2:
        tr_idx, bl_idx = o1, o2
    else:
        tr_idx, bl_idx = o2, o1
    return [pts[tl_idx].tolist(), pts[tr_idx].tolist(), pts[bl_idx].tolist()]


def _find_qr_angle(patch: np.ndarray) -> float:
    """用 QR 回字定位图案估算标签倾斜角。

    二维码在标签左下角，TL→TR 方向 = 文字方向。
    比全 QR 四边形可靠（定位图案不受小码/模糊影响）。

    返回:
        倾斜角度（度），未找到返回 0
    """
    finders = _find_qr_finders(patch)
    if len(finders) != 3:
        return 0.0
    tl, tr, bl = _order_finders(finders)
    top_vec = np.array(tr, dtype=np.float32) - np.array(tl, dtype=np.float32)
    return float(np.degrees(np.arctan2(top_vec[1], top_vec[0])))

    return 0.0


def _minarea_angle(patch: np.ndarray) -> float:
    """用 minAreaRect 估算整块文字倾斜角（度）。"""
    h, w = patch.shape[:2]
    if h < 30 or w < 30:
        return 0.0
    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    pts = cv2.findNonZero(binary)
    if pts is None or len(pts) < 100:
        return 0.0
    rect = cv2.minAreaRect(pts)
    angle = rect[2]
    if angle < -45:
        angle = 90 + angle
    return angle


# ── 整块文字纠偏（自适应角度） ────────────────────────


def _deskew_patch(patch: np.ndarray, min_angle: float = 3.0) -> np.ndarray:
    """整块文字纠偏：用 minAreaRect 估计整体倾斜角并旋转摆正。

    在整块标签上计算（多行文字），比单行 crop 的角度估计更可靠。
    """
    h, w = patch.shape[:2]
    if h < 30 or w < 30:
        return patch

    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    pts = cv2.findNonZero(binary)
    if pts is None or len(pts) < 100:
        return patch

    rect = cv2.minAreaRect(pts)
    angle = rect[2]
    if angle < -45:
        angle = 90 + angle

    # 角度太小不用转
    if abs(angle) < min_angle:
        return patch

    center = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated = cv2.warpAffine(patch, M, (w, h), flags=cv2.INTER_LANCZOS4,
                             borderMode=cv2.BORDER_CONSTANT, borderValue=(255, 255, 255))
    return rotated


# ── 标签定位 + 透视校正 ────────────────────────────────


def _find_and_correct_label(patch: np.ndarray) -> tuple[np.ndarray, np.ndarray | None]:
    """在 ROI 中找白色矩形标签，做透视校正摆正。

    白色标签（如药品标签）在深色背景下有明显轮廓：
      1. 灰度 + 高斯模糊
      2. 自适应阈值 → 找最大矩形轮廓
      3. 获取四个角点 → 透视变换拉正

    返回:
        (处理后的图像, 变换矩阵或 None)
    """
    h, w = patch.shape[:2]
    if h < 30 or w < 30:
        return patch, None

    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # 多阈值查找白色标签：OTSU + 固定高阈值，提升帧间稳定性
    quads = []
    thresholds = []

    # OTSU 自适应阈值
    otsu_val, binary_otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    thresholds.append(("otsu", binary_otsu, otsu_val))

    # 固定高阈值（白标签通常很亮）
    for t in (230, 200):
        _, binary_t = cv2.threshold(blur, t, 255, cv2.THRESH_BINARY)
        thresholds.append((f"fixed{t}", binary_t, t))

    for name, binary in [t[:2] for t in thresholds]:
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue

        # 按面积排序，找最大的矩形轮廓
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        for cnt in contours[:30]:
            area = cv2.contourArea(cnt)
            if area < 100:  # 小于 100 像素的跳过
                continue

            # 用更精确的近似，确保找到四边形
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.015 * peri, True)

            if len(approx) == 4:
                pts = approx.reshape(4, 2).astype(np.float32)
                pts_sorted = _sort_corners(pts)
                # 检查是否接近矩形（四个角接近 90°）
                angles = []
                for j in range(4):
                    p0 = pts_sorted[j]
                    p1 = pts_sorted[(j + 1) % 4]
                    p2 = pts_sorted[(j + 2) % 4]
                    v1 = p1 - p0
                    v2 = p2 - p1
                    dot = float(v1[0] * v2[0] + v1[1] * v2[1])
                    norm = float(np.linalg.norm(v1) * np.linalg.norm(v2))
                    angle = abs(dot / norm) if norm > 0 else 1.0
                    angles.append(angle)
                # 角度偏差容忍：cos(θ) 接近 0 时 θ 接近 90°
                if max(angles) > 0.5:
                    continue
                # 记录来源阈值，面积越大越可信
                quads.append((area, pts_sorted))

    if quads:
        # 取面积最大的矩形
        quads.sort(key=lambda x: x[0], reverse=True)
        pts_sorted = quads[0][1]

        # 计算目标矩形尺寸
        (tl, tr, br, bl) = pts_sorted
        width_a = np.linalg.norm(br - bl)
        width_b = np.linalg.norm(tr - tl)
        max_w = max(int(width_a), int(width_b))
        height_a = np.linalg.norm(tr - br)
        height_b = np.linalg.norm(tl - bl)
        max_h = max(int(height_a), int(height_b))
        max_w = max(max_w, 1)
        max_h = max(max_h, 1)

        dst = np.array([
            [0, 0],
            [max_w - 1, 0],
            [max_w - 1, max_h - 1],
            [0, max_h - 1],
        ], dtype=np.float32)

        M = cv2.getPerspectiveTransform(pts_sorted, dst)
        warped = cv2.warpPerspective(patch, M, (max_w, max_h),
                                     flags=cv2.INTER_LANCZOS4)
        return warped, M

    return patch, None


def _sort_corners(pts: np.ndarray) -> np.ndarray:
    """将四边形四点排序为：左上、右上、右下、左下。"""
    rect = np.zeros((4, 2), dtype=np.float32)
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]   # 左上 (和最小)
    rect[2] = pts[np.argmax(s)]   # 右下 (和最大)
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]  # 右上
    rect[3] = pts[np.argmax(diff)]  # 左下
    return rect


# ── 文字检测（PP-OCRv3 det） ──────────────────────────────


def _ppocr_det_preprocess(patch: np.ndarray,
                          target_size: int = 960) -> tuple[np.ndarray, float, tuple]:
    """PP-OCRv3 检测模型预处理。

    返回:
        (input_tensor, scale, (orig_h, orig_w))
    """
    h, w = patch.shape[:2]
    # 短边对齐到 target_size，长边按比例
    scale = target_size / min(h, w) if min(h, w) < target_size else 1.0
    nh = int(round(h * scale / 32) * 32)
    nw = int(round(w * scale / 32) * 32)
    nh = max(32, nh)
    nw = max(32, nw)
    resized = cv2.resize(patch, (nw, nh), interpolation=cv2.INTER_LINEAR)
    # ImageNet 归一化
    img = resized.astype(np.float32) / 255.0
    mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
    img = (img - mean) / std
    img = img.transpose((2, 0, 1))[np.newaxis, ...]
    return img, scale, (h, w)


def _det_postprocess(output: np.ndarray,
                     orig_shape: tuple,
                     scale: float,
                     threshold: float = 0.3) -> list[tuple[int, int, int, int]]:
    """检测模型后处理：概率图 → 阈值 → 轮廓 → bbox。

    返回:
        [(x1,y1,x2,y2), ...] 在原图坐标系下的文字区域
    """
    # 输出已经是 sigmoid 后的概率图 [0,1]，直接取
    prob = (output[0, 0] * 255).astype(np.uint8)

    # 二值化
    _, binary = cv2.threshold(prob, int(threshold * 255), 255, cv2.THRESH_BINARY)

    # 形态学闭运算：填充文字内部空洞
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    # 找轮廓
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    h, w = output.shape[2:]
    oh, ow = orig_shape
    scale_x = ow / w
    scale_y = oh / h

    boxes = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 20:  # 太小就跳过
            continue

        # 外接矩形
        x, y, bw, bh = cv2.boundingRect(cnt)
        # 轻微膨胀
        pad = 3
        x1 = max(0, int((x - pad) * scale_x))
        y1 = max(0, int((y - pad) * scale_y))
        x2 = min(ow, int((x + bw + pad) * scale_x))
        y2 = min(oh, int((y + bh + pad) * scale_y))
        if x2 <= x1 or y2 <= y1:
            continue
        boxes.append((x1, y1, x2, y2))

    # 合并高度重叠的框（同一行文字被断开时）
    if boxes:
        boxes = _merge_overlapping_boxes(boxes)

    return boxes


def _merge_overlapping_boxes(boxes: list[tuple],
                              overlap_thresh: float = 0.3) -> list[tuple]:
    """合并高度重叠的相邻框。"""
    if not boxes:
        return []
    # 按 y 坐标排序
    boxes = sorted(boxes, key=lambda b: (b[1], b[0]))
    merged = [boxes[0]]
    for b in boxes[1:]:
        prev = merged[-1]
        # 计算垂直重叠
        y_overlap = max(0, min(prev[3], b[3]) - max(prev[1], b[1]))
        y_range = min(prev[3] - prev[1], b[3] - b[1])
        if y_range > 0 and y_overlap / y_range > overlap_thresh:
            # 合并：取并集
            merged[-1] = (
                min(prev[0], b[0]),
                min(prev[1], b[1]),
                max(prev[2], b[2]),
                max(prev[3], b[3]),
            )
        else:
            merged.append(b)
    return merged


def _ppocr_rec_preprocess(patch: np.ndarray,
                          target_h: int = 48,
                          max_w: int = 960,
                          min_w: int = 16) -> np.ndarray:
    """PP-OCRv6 识别模型预处理。模型宽度是动态的，放宽到 960 容纳长行。"""
    h, w = patch.shape[:2]
    ratio = target_h / max(h, 1)
    tw = max(min_w, min(int(w * ratio), max_w))
    resized = cv2.resize(patch, (tw, target_h), interpolation=cv2.INTER_LINEAR)
    # Normalize: (x / 255 - 0.5) / 0.5
    img = resized.astype(np.float32) / 255.0
    img = (img - 0.5) / 0.5
    # HWC → CHW + batch
    img = img.transpose((2, 0, 1))[np.newaxis, ...]
    return img


def _ctc_greedy_decode(preds: np.ndarray,
                       char_list: list[str]) -> list[dict]:
    """CTC 贪婪解码。

    preds: (batch, W, C) — softmax 后的概率
    char_list[0] = 'blank'（CTC blank 占位符，不输出）
    """
    results = []
    for batch_idx in range(preds.shape[0]):
        probs = preds[batch_idx]  # (W, C)
        pred_ids = probs.argmax(axis=1)  # (W,)

        chars = []
        confs = []
        prev = -1
        for idx, prob in zip(pred_ids, probs.max(axis=1)):
            idx_int = int(idx)
            if idx_int != 0 and idx_int != prev:  # 跳过 blank 和重复
                if idx_int < len(char_list):
                    chars.append(char_list[idx_int])
                    confs.append(float(prob))
            prev = idx_int

        text = "".join(chars)
        confidence = sum(confs) / len(confs) if confs else 0.0
        results.append({"text": text, "confidence": confidence})
    return results


# ── OCRDetector ───────────────────────────────────────────


class OCRDetector:
    """ONNX Runtime 文字识别。

    - YOLO 提供 ROI → 水平投影切分文字行 → 逐行 recognition-only
    - 支持单行和多行文字，每行独立输出 bbox + 置信度
    - CUDA EP → CPU EP 自动降级
    """

    def __init__(self, conf_threshold: float = OCR_CONF_THRESHOLD,
                 rec_path: str = ONNX_REC_PATH, det_path: str = ONNX_DET_PATH,
                 dict_path: str = CHAR_DICT_PATH):
        self.conf_threshold = conf_threshold
        self.backends: list[str] = []
        self._rec_session = None
        self._det_session = None
        self._char_list = ["blank"]
        self._rec_input = None
        self._rec_output = None
        self._det_input = None
        self._det_output = None

        if not self._load_dict(dict_path):
            return

        if not self._load_rec_model(rec_path):
            return

        self._load_det_model(det_path)

    def _load_dict(self, dict_path: str) -> bool:
        if not os.path.exists(dict_path):
            print(f"[OCR模型] 字典文件不存在: {dict_path}")
            return False
        try:
            with open(dict_path, "r", encoding="utf-8") as f:
                lines = [line.strip() for line in f]
            # char_list[0] = blank（CTC blank），char_list[1..] = 实际字符
            self._char_list = ["blank"] + lines
            # 模型输出可能比字典多几列，补齐避免越界
            while len(self._char_list) < 18710:
                self._char_list.append("")
            print(f"[OCR模型] 加载字典: {len(lines)} 个字符")
            return True
        except Exception as e:
            print(f"[OCR模型] 字典加载失败: {e}")
            return False

    def _load_rec_model(self, model_path: str) -> bool:
        if not os.path.exists(model_path):
            print(f"[OCR模型] ONNX 模型不存在: {model_path}")
            return False
        if not _ORT_OK:
            print("[OCR模型] 未安装 onnxruntime。Orin 上请运行: pip install onnxruntime-gpu")
            return False

        try:
            # 默认 CPU，避免桌面环境缺 CUDA runtime 时产生 provider fallback
            # 噪声。需要测试 ORT GPU 时设置 OCR_USE_CUDA=1。
            providers = self._get_providers()
            self._rec_session = ort.InferenceSession(model_path, providers=providers)
            self._rec_input = self._rec_session.get_inputs()[0].name
            self._rec_output = self._rec_session.get_outputs()[0].name

            # 打印使用的 provider
            active = self._rec_session.get_providers()
            self.backends.append(f"rec({active[0]})")
            print(f"[OCR模型] 加载识别模型: {model_path}")
            print(f"[OCR模型] 识别后端: {active[0]}")
            if len(active) > 1:
                print(f"[OCR模型] 备用后端: {', '.join(active[1:])}")
            return True
        except Exception as e:
            print(f"[OCR模型] 模型加载失败: {e}")
            return False

    def _load_det_model(self, model_path: str):
        """加载检测模型（可选，失败时用投影法回退）。"""
        if not os.path.exists(model_path):
            print(f"[OCR模型] 检测模型不存在: {model_path}，使用投影法回退")
            return
        if not _ORT_OK:
            return
        try:
            providers = self._get_providers()
            sess = ort.InferenceSession(model_path, providers=providers)
            self._det_session = sess
            self._det_input = sess.get_inputs()[0].name
            self._det_output = sess.get_outputs()[0].name
            self.backends.append(f"det({sess.get_providers()[0]})")
            print(f"[OCR模型] 加载检测模型: {model_path}")
        except Exception as e:
            print(f"[OCR模型] 检测模型加载失败: {e}，使用投影法回退")

    def _get_providers(self):
        """按优先级返回 provider 列表。"""
        providers = []
        available = ort.get_available_providers()

        if os.environ.get("OCR_USE_CUDA", "0") == "1" and "CUDAExecutionProvider" in available:
            providers.append("CUDAExecutionProvider")

        # CPU 回退
        if "CPUExecutionProvider" in available:
            providers.append("CPUExecutionProvider")

        if not providers:
            providers = available or ["CPUExecutionProvider"]

        return providers

    def backend_info(self) -> str:
        return ", ".join(self.backends) if self.backends else "无可用后端"

    # ── 公开接口 ──────────────────────────────────────────

    def detect(self, frame: np.ndarray,
               rois: list[tuple[int, int, int, int]] | None = None,
               heavy: bool = False,
               upscale: float = 2.0) -> list[dict]:
        """对每个 ROI 执行文字识别。

        参数:
            frame: BGR 图像
            rois: [(x1,y1,x2,y2), ...] 物体 bbox（来自 YOLO）
            heavy: 忽略
            upscale: 检测前放大倍数（文字太小时用 >1.0 提高识别率）

        返回:
            list[dict]: {text, confidence, bbox, backend, roi_index}
        """
        if self._rec_session is None:
            return []

        results = []
        for idx, roi in enumerate(rois or []):
            results.extend(self._recognize_in_roi(frame, roi, idx, upscale))
        return results

    def _recognize_in_roi(self, frame, roi, roi_index, upscale=1.0):
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = roi
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        if x2 <= x1 or y2 <= y1:
            return []

        pad = 8
        x1, y1 = max(0, x1 - pad), max(0, y1 - pad)
        x2, y2 = min(w, x2 + pad), min(h, y2 + pad)
        patch = frame[y1:y2, x1:x2]
        if patch.size == 0:
            return []

        _dbg_roi_idx = getattr(self, "_dbg_counter", 0) + 1
        self._dbg_counter = _dbg_roi_idx
        _dbg_prefix = f"data/debug/r{_dbg_roi_idx}"

        if _DEBUG_SAVE:
            os.makedirs("data/debug", exist_ok=True)
            # 保存原始 ROI（YOLO 给出的区域）
            cv2.imwrite(f"{_dbg_prefix}_00_roi.png", patch)

        # 小字放大
        if upscale > 1.0 and min(patch.shape[:2]) > 10:
            pu_h, pu_w = patch.shape[:2]
            new_h, new_w = int(pu_h * upscale), int(pu_w * upscale)
            patch = cv2.resize(patch, (new_w, new_h), interpolation=cv2.INTER_LANCZOS4)

        if _DEBUG_SAVE:
            cv2.imwrite(f"{_dbg_prefix}_01_upscaled.png", patch)

        # 旋转摆正：QR 找到就用 QR 角度（倾斜标签上可靠），找不到才用 minAreaRect
        if _USE_QR_DESKEW:
            angle = _find_qr_angle(patch)
            if abs(angle) > 2.0:
                ph, pw = patch.shape[:2]
                M = cv2.getRotationMatrix2D((pw // 2, ph // 2), angle, 1.0)
                patch = cv2.warpAffine(patch, M, (pw, ph), flags=cv2.INTER_LANCZOS4,
                                       borderMode=cv2.BORDER_CONSTANT,
                                       borderValue=(255, 255, 255))
            else:
                patch = _deskew_patch(patch)
        else:
            patch = _deskew_patch(patch)

        if _DEBUG_SAVE:
            cv2.imwrite(f"{_dbg_prefix}_02_deskewed.png", patch)

        # 检测模型找文字区域
        sub_rois = None
        if self._det_session is not None:
            sub_rois = self._detect_text_regions(patch)

        if _DEBUG_SAVE and sub_rois:
            vis = patch.copy()
            for s in sub_rois:
                if len(s) == 3:
                    continue
                sx1, sy1, sx2, sy2 = s
                cv2.rectangle(vis, (sx1, sy1), (sx2, sy2), (0, 255, 0), 2)
            cv2.imwrite(f"{_dbg_prefix}_03_det_boxes.png", vis)

        # 无检测结果 → 用投影法分行
        if not sub_rois:
            sub_rois = _split_text_lines(patch)
            if _DEBUG_SAVE:
                vis2 = patch.copy()
                for s in sub_rois:
                    if len(s) == 3:
                        sy1, sy2, _ = s
                        cv2.rectangle(vis2, (0, sy1), (patch.shape[1], sy2), (255, 0, 0), 2)
                cv2.imwrite(f"{_dbg_prefix}_03_proj_boxes.png", vis2)

        out = []
        for sub in sub_rois:
            # sub: (sy1, sy2, crop) 投影法 或 (sx1, sy1, sx2, sy2) 检测法
            if len(sub) == 3:
                sy1, sy2, crop = sub
                crop_x1, crop_x2 = 0, patch.shape[1]
                crop_y1, crop_y2 = sy1, sy2
            else:
                sx1, sy1, sx2, sy2 = sub
                crop = patch[sy1:sy2, sx1:sx2]
                if crop.size == 0:
                    continue
                crop_x1, crop_x2 = sx1, sx2
                crop_y1, crop_y2 = sy1, sy2

            if _DEBUG_SAVE:
                cv2.imwrite(f"{_dbg_prefix}_04_crop_{len(out)}_raw.png", crop)

            # 纠偏
            # crop = _deskew_crop(crop)
            # 转黑白二值图（白底黑字，去噪）

            if _DEBUG_SAVE:
                cv2.imwrite(f"{_dbg_prefix}_04_crop_{len(out)}_deskewed.png", crop)

            try:
                inp = _ppocr_rec_preprocess(crop)
                outputs = self._rec_session.run(
                    [self._rec_output],
                    {self._rec_input: inp},
                )
                decoded = _ctc_greedy_decode(outputs[0], self._char_list)
            except Exception:
                continue

            for det in decoded:
                if not det["text"] or det["confidence"] < self.conf_threshold:
                    continue
                inv = 1.0 / upscale
                fx = x1 + int(crop_x1 * inv)
                fy1 = y1 + int(crop_y1 * inv)
                fx2 = x1 + int(crop_x2 * inv)
                fy2 = y1 + int(crop_y2 * inv)
                out.append({
                    "text": det["text"],
                    "confidence": det["confidence"],
                    "bbox": (fx, fy1, fx2, fy2),
                    "backend": self.backends[0] if self.backends else "onnx",
                    "roi_index": roi_index,
                })
        # 全都没结果 → 回退整张 patch
        if not out:
            try:
                # ep = _enhance_label(patch)
                inp = _ppocr_rec_preprocess(patch)
                outputs = self._rec_session.run(
                    [self._rec_output],
                    {self._rec_input: inp},
                )
                decoded = _ctc_greedy_decode(outputs[0], self._char_list)
            except Exception:
                return []

            for det in decoded:
                if not det["text"] or det["confidence"] < self.conf_threshold:
                    continue
                out.append({
                    "text": det["text"],
                    "confidence": det["confidence"],
                    "bbox": (x1, y1, x2, y2),
                    "backend": self.backends[0] if self.backends else "onnx",
                    "roi_index": roi_index,
                })

        return out

    def _detect_text_regions(self, patch: np.ndarray) -> list[tuple[int, int, int, int]]:
        """用 PP-OCR 检测模型找 patch 中的文字区域。

        返回:
            [(x1, y1, x2, y2), ...] 在 patch 坐标系下的文字区域
        """
        if self._det_session is None:
            return []

        try:
            # 先增强，让检测模型更容易找到浅色文字
            # enhanced_patch = _enhance_label(patch)
            input_tensor, scale, orig_shape = _ppocr_det_preprocess(patch)
            outputs = self._det_session.run(
                [self._det_output],
                {self._det_input: input_tensor},
            )
            boxes = _det_postprocess(outputs[0], orig_shape, scale)
            return boxes
        except Exception:
            return []


# ── OCR Worker（异步） ─────────────────────────────────────


class OCRWorker:
    """后台 OCR 解码，避免阻塞主循环。"""

    def __init__(self):
        self._detector = OCRDetector()
        self._request = None
        self._result = []
        self._busy = False
        self._stop = False
        self._decode_id = 0
        self._stats = {
            "decode_ms": 0.0,
            "source": "",
            "mode": "",
            "roi_count": 0,
            "result_count": 0,
            "error": "",
            "decode_id": 0,
        }
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def submit(self, frame, rois, source: str, mode: str = "", upscale: float = 1.0) -> bool:
        with self._lock:
            if self._busy or self._request is not None:
                return False
            self._request = (frame.copy(), list(rois), source, mode, upscale)
            return True

    def latest(self):
        with self._lock:
            return list(self._result), self._busy, dict(self._stats)

    def backend_info(self) -> str:
        return self._detector.backend_info()

    def stop(self):
        with self._lock:
            self._stop = True
        self._thread.join(timeout=1.0)

    def _loop(self):
        while True:
            with self._lock:
                if self._stop:
                    return
                req = self._request
                self._request = None
                if req is not None:
                    self._busy = True
            if req is None:
                time.sleep(0.01)
                continue

            frame, rois, source, mode, upscale = req
            start = time.time()
            error = ""
            try:
                result = self._detector.detect(frame, rois=rois, upscale=upscale)
            except Exception as exc:
                result = []
                error = repr(exc)

            decode_ms = (time.time() - start) * 1000.0
            with self._lock:
                self._result = result
                self._decode_id += 1
                self._stats = {
                    "decode_ms": decode_ms,
                    "source": source,
                    "mode": mode,
                    "roi_count": len(rois),
                    "result_count": len(result),
                    "error": error,
                    "decode_id": self._decode_id,
                }
                self._busy = False


# ── 绘制 ──────────────────────────────────────────────────


def draw_ocr_detections(frame: np.ndarray,
                         detections: list[dict],
                         color: tuple = (0, 255, 0)) -> np.ndarray:
    """在图像上绘制 OCR 结果：绿色框 + 文字（支持中文）。"""
    out = frame.copy()

    # 尝试用 Pillow 绘制中文（回退到 OpenCV ASCII）
    try:
        from PIL import Image, ImageDraw, ImageFont
        _PIL_OK = True
    except ImportError:
        _PIL_OK = False

    # 找系统中文字体
    _font = None
    if _PIL_OK:
        import platform
        if platform.system() == "Windows":
            candidates = [
                "C:/Windows/Fonts/msyh.ttc",        # Microsoft YaHei
                "C:/Windows/Fonts/simhei.ttf",       # SimHei
                "C:/Windows/Fonts/msyhbd.ttc",       # YaHei Bold
                "C:/Windows/Fonts/STSONG.TTF",       # 华文宋体
            ]
        elif platform.system() == "Linux":
            candidates = [
                "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
                "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
                "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
            ]
        else:  # macOS
            candidates = [
                "/System/Library/Fonts/PingFang.ttc",
                "/System/Library/Fonts/STHeiti Light.ttc",
            ]
        for path in candidates:
            if os.path.exists(path):
                try:
                    _font = ImageFont.truetype(path, 16)
                    break
                except Exception:
                    continue

    for det in detections:
        try:
            bbox_values = np.asarray(det.get("bbox", [0, 0, 1, 1]), dtype=float).reshape(-1)
            if bbox_values.size != 4:
                continue
            x1, y1, x2, y2 = [int(round(v)) for v in bbox_values.tolist()]
        except Exception:
            continue
        if x2 <= x1 or y2 <= y1:
            continue
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)

        label = det["text"]
        if len(label) > 50:
            label = label[:47] + "..."

        label_y = max(18, y1 - 6)

        if _PIL_OK and _font is not None:
            # 用 Pillow 绘制中文
            pil_img = Image.fromarray(cv2.cvtColor(out, cv2.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(pil_img)
            bbox = draw.textbbox((0, 0), label, font=_font)
            tw = bbox[2] - bbox[0]
            th = bbox[3] - bbox[1]
            draw.rectangle([x1, label_y - th - 2, x1 + tw + 4, label_y + 2],
                           fill=(0, 0, 0))
            draw.text((x1 + 2, label_y - th), label, font=_font,
                      fill=(color[2], color[1], color[0]))
            out = cv2.cvtColor(np.asarray(pil_img), cv2.COLOR_RGB2BGR)
        else:
            # 回退：OpenCV 只支持 ASCII
            cv2.putText(out, label, (x1 + 2, label_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # 置信度（数字，ASCII 没问题）
        conf_text = f"{det['confidence']:.2f}"
        cv2.putText(out, conf_text, (x2 - 44, y2 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    return out

# ── Memory ────────────────────────────────────────────────


def _ocr_summary(detections):
    return ";".join(
        f"{d.get('backend','?')}/roi{d.get('roi_index','?')}:{d['text']}"
        for d in detections
    )


def _merge_ocr_memory(memory, detections, now):
    for det in detections:
        memory[det["text"]] = {"det": dict(det), "last_seen": now}
    stale = [t for t, v in memory.items() if now - v["last_seen"] > OCR_MEMORY_TTL_SEC]
    for t in stale:
        del memory[t]
    return [v["det"] for v in memory.values()]


# ── OCR 后纠错 ────────────────────────────────────────────


# 病区列表（核心识别目标，ID+名称互相佐证）
# 后续新病区都加在这里
_WARD_LIST = [
    "253放化疗病区（方桥）",
    "703肺科(2)病区",
]

# 通用标签文本词典
_OCR_DICT = [
    # 医院科室
    "宁波大学附属第一医院（方桥）",
    # 住院信息
    "住院号：",
    "床号：",
    # 患者信息
    "培理",
    "男",
    "女",
    # 药品相关
    "口服药",
    "用法",
    "用量",
    "规格",
    "日期",
    "发药时间：",
    "2026/07/15",
    # 数字序列
    "253", "13675186", "253005", "1956/06/06",
    "62", "201",
]


def _ocr_correct(text: str, cutoff: float = 0.4) -> str:
    """模糊匹配纠错：优先匹配病区列表，其次匹配通用词典。

    保守策略：
    - 纯数字/数字为主的文本不参与病区模糊匹配（避免误伤床号/住院号等 ID）
    - 词典替换要求长度相近，防止把长文本截断成短词典条目
    """
    import difflib

    # 不超过 3 个字或完全匹配，不用纠
    if len(text) <= 3:
        return text

    # 1. 优先匹配病区列表（ID + 名称互相佐证）
    if text in _WARD_LIST:
        return text

    # 日期/ID 类文本宁可保留原始结果，不做词典模糊替换。
    # 例如 1990/08/17 不能被误纠成发药日期。
    digit_ratio = sum(c.isdigit() for c in text) / max(len(text), 1)
    if "/" in text and any(c.isdigit() for c in text):
        return text
    if digit_ratio > 0.5:
        return text

    # 医院名/机构名不能进入病区模糊匹配，否则“宁波大学附属第一医院”
    # 会被误纠成相似度较高的病区名。
    if any(token in text for token in ("医院", "大学", "附属")):
        match = difflib.get_close_matches(text, _OCR_DICT, n=1, cutoff=cutoff)
        if match:
            candidate = match[0]
            if len(candidate) >= len(text) * 0.8:
                return candidate
        return text

    # 数字占比较低且包含病区证据的文本才参与病区模糊匹配。
    # 不能只凭“方桥”这类院区词纠成某个病区。
    ward_like_tokens = ("病区", "化疗", "化区", "肺科")
    if digit_ratio <= 0.5 and any(token in text for token in ward_like_tokens):
        # 直接算完整 SequenceMatcher ratio（get_close_matches 的 quick_ratio 会误拒带垃圾的串）
        best_ward, best_ratio = None, 0.0
        for ward in _WARD_LIST:
            r = difflib.SequenceMatcher(None, text, ward).ratio()
            if r > best_ratio:
                best_ratio, best_ward = r, ward
        if best_ratio >= 0.3:
            return best_ward

    # 2. 匹配通用词典：仅当长度相近才替换（防止截断长文本）
    match = difflib.get_close_matches(text, _OCR_DICT, n=1, cutoff=cutoff)
    if match:
        candidate = match[0]
        if len(candidate) >= len(text) * 0.8:
            return candidate

    return text


# ── 主函数 ────────────────────────────────────────────────


def main():
    print("=" * 60)
    print("  OCR Test — RealSense RGB (ONNX + TensorRT)")
    print("=" * 60)
    print(f"  WebSocket: {config.WS_URL}")
    print(f"  RGB Topic: {config.TOPIC_RGB}")
    print(f"  推理引擎: ONNX Runtime + TensorRT (Orin GPU)")
    print("=" * 60)
    print()

    # 检查环境
    if not _ORT_OK:
        print("[✗] 缺少 onnxruntime。请运行:")
        print("    pip install onnxruntime-gpu    # Orin")
        print("    pip install onnxruntime        # CPU 调试")
        sys.exit(1)

    # 自动准备模型
    if not _ensure_dict():
        print("[✗] 无法获取字典文件，退出")
        sys.exit(1)
    if not _ensure_onnx_model():
        print("[✗] 无法获取 ONNX 模型，退出")
        sys.exit(1)

    # ROS 连接
    if not _ROS_OK:
        print("[✗] 缺少 roslibpy，无法连接机器人。请运行: pip install roslibpy")
        sys.exit(1)

    host = config.WS_URL.replace("ws://", "").split(":")[0]
    port = int(config.WS_URL.replace("ws://", "").split(":")[1])

    client = roslibpy.Ros(host=host, port=port)
    client.on_ready(on_connection, run_in_thread=False)
    ros_thread = threading.Thread(target=client.run, daemon=True)
    ros_thread.start()

    start = time.time()
    while not client.is_connected:
        if time.time() - start > config.CONNECT_TIMEOUT:
            print(f"[✗] 连接超时: {config.WS_URL}")
            _safe_terminate(client)
            sys.exit(1)
        time.sleep(0.1)

    # ROS 订阅
    raw_subscriber = None
    subscriber = roslibpy.Topic(client, config.TOPIC_RGB, "sensor_msgs/CompressedImage")
    subscriber.subscribe(on_rgb_message)
    try:
        raw_subscriber = roslibpy.Topic(
            client, config.TOPIC_RGB_RAW, "sensor_msgs/Image",
            throttle_rate=RAW_RGB_THROTTLE_MS, queue_length=1,
        )
    except TypeError:
        raw_subscriber = roslibpy.Topic(client, config.TOPIC_RGB_RAW, "sensor_msgs/Image")
    raw_subscriber.subscribe(on_raw_rgb_message)

    # 初始化
    from robot_grasp.detector import Detector
    object_detector = Detector(
        target_classes=OCR_OBJECT_CLASSES, conf=OCR_OBJECT_CONF, imgsz=512
    )
    ocr_worker = OCRWorker()
    os.makedirs("data", exist_ok=True)

    diag_path = os.path.join(
        "data", f"ocr_debug_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )
    diag_file = open(diag_path, "w", newline="", encoding="utf-8")
    diag_writer = csv.DictWriter(diag_file, fieldnames=[
        "time", "frame_count", "fps",
        "rgb_rx_count", "raw_rx_count", "raw_age_ms",
        "object_count", "object_max_conf", "object_detections",
        "roi_count", "roi_fallback", "rois",
        "ocr_submit", "ocr_busy",
        "ocr_source_current", "ocr_decode_ms",
        "ocr_result_count", "ocr_memory_count", "ocr_unique_count",
        "ocr_results", "ocr_memory_results", "ocr_error",
    ])
    diag_writer.writeheader()

    seen_texts: set[str] = set()
    ocr_memory = {}
    fps = 0.0
    prev_time = time.time()
    fps_counter = 0
    last_frame_count = -1
    annotated = None
    last_object_rois = []
    object_detections = []
    last_ocr_results = []
    last_ocr_decode_time = 0.0
    ocr_busy = False
    ocr_source = "compressed"
    ocr_stats = {}
    last_diag_print = 0.0

    print(f"\n[*] 识别物体上的文字：检测 {OCR_OBJECT_CLASSES}，再对 ROI 执行 OCR。")
    print(f"[*] YOLO confidence: {OCR_OBJECT_CONF}")
    print(f"[*] YOLO every {DETECT_EVERY_N_FRAMES} frames | OCR every {OCR_DECODE_INTERVAL_SEC:.1f}s")
    print(f"[*] OCR memory TTL: {OCR_MEMORY_TTL_SEC:.1f}s")
    print(f"[*] OCR backends: {ocr_worker.backend_info()}")
    print(f"[*] 识别模式: recognition-only (PP-OCRv3 rec on TensorRT)")
    print(f"[*] debug CSV: {diag_path}")
    print("[*] q=退出 | s=保存截图 | r=保存 ROI\n")

    try:
        while True:
            # ── 以 raw 帧为主处理源（跟 debug_ocr_image.py 一致） ──
            with raw_frame_lock:
                frame = latest_raw_frame.copy() if latest_raw_frame is not None else None
                fc = raw_frame_count
                raw_count = raw_frame_count
                raw_time = latest_raw_time
            if frame is None or fc == last_frame_count:
                # 诊断：无新 raw 帧时每 2 秒打印一次接收情况
                if time.time() - locals().get("_diag_t", 0) >= 2.0:
                    _diag_t = time.time()
                    print(f"[DIAG] raw消息={raw_frame_count} 压缩消息={rgb_rx_count} 压缩解码失败={rgb_decode_fail} 等待raw帧...")
                key = cv2.waitKey(5) & 0xFF
                if key == ord("q"):
                    break
                continue
            last_frame_count = fc

            fps_counter += 1
            now = time.time()
            if now - prev_time >= 1.0:
                fps = fps_counter / (now - prev_time)
                fps_counter = 0
                prev_time = now

            # ── YOLO 检测（在 raw 帧上） ──
            if fc % DETECT_EVERY_N_FRAMES == 0 or not object_detections:
                object_detections, _ = object_detector.detect(frame)

            object_rois = [det["bbox"] for det in object_detections]
            roi_fallback = False
            if not object_rois:
                h0, w0 = frame.shape[:2]
                object_rois.append((w0 // 4, h0 // 4, 3 * w0 // 4, 3 * h0 // 4))
                roi_fallback = True
            last_object_rois = object_rois

            # ── OCR 用 raw 帧 ──
            ocr_frame = frame
            ocr_rois = object_rois
            ocr_source = "raw"

            # ── 提交 OCR 任务 ──
            ocr_submit = False
            if now - last_ocr_decode_time >= OCR_DECODE_INTERVAL_SEC:
                if ocr_worker.submit(ocr_frame, ocr_rois, ocr_source, upscale=2.0):
                    last_ocr_decode_time = now
                    ocr_submit = True

            last_ocr_results, ocr_busy, ocr_stats = ocr_worker.latest()

            # OCR 后纠错（词典匹配）
            for det in last_ocr_results:
                det["text"] = _ocr_correct(det["text"])

            memory_results = _merge_ocr_memory(ocr_memory, last_ocr_results, now)

            # ── 标注 ──
            annotated = frame.copy()
            for idx, det in enumerate(object_detections, start=1):
                x1, y1, x2, y2 = det["bbox"]
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 128, 0), 2)
                cv2.putText(annotated, f"{det['label']} {idx} {det['confidence']:.2f}",
                            (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (255, 128, 0), 2)

            annotated = draw_ocr_detections(annotated, memory_results, color=(0, 255, 0))

            for det in last_ocr_results:
                text = det["text"]
                if text not in seen_texts:
                    seen_texts.add(text)
                    print(f"[OCR {len(seen_texts)}] {text}  "
                          f"({det.get('backend','?')}, "
                          f"roi{det.get('roi_index','?')})")

            # ── 顶部状态栏 ──
            _, w = annotated.shape[:2]
            cv2.rectangle(annotated, (0, 0), (w, 64), (0, 0, 0), -1)
            busy_text = "busy" if ocr_busy else "idle"
            decode_ms = float(ocr_stats.get("decode_ms", 0.0) or 0.0)
            cv2.putText(annotated,
                        f"OCR Test | FPS: {fps:.1f} | Objects: {len(object_detections)} "
                        f"| OCR: {len(last_ocr_results)}/{len(memory_results)} "
                        f"| Unique: {len(seen_texts)} | {busy_text} {ocr_source} {decode_ms:.0f}ms",
                        (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            cv2.putText(annotated, "q: quit | s: save snapshot | r: save ROI",
                        (12, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)

            # ── CSV 日志 ──
            raw_age_ms = (now - raw_time) * 1000.0 if raw_time > 0 else -1.0
            object_max_conf = max((det["confidence"] for det in object_detections), default=0.0)
            diag_writer.writerow({
                "time": f"{now:.3f}",
                "frame_count": fc,
                "fps": f"{fps:.2f}",
                "rgb_rx_count": rgb_rx_count,
                "raw_rx_count": raw_count,
                "raw_age_ms": f"{raw_age_ms:.1f}",
                "object_count": len(object_detections),
                "object_max_conf": f"{object_max_conf:.3f}",
                "object_detections": _detection_summary(object_detections),
                "roi_count": len(object_rois),
                "roi_fallback": int(roi_fallback),
                "rois": _roi_summary(object_rois),
                "ocr_submit": int(ocr_submit),
                "ocr_busy": int(ocr_busy),
                "ocr_source_current": ocr_source,
                "ocr_decode_ms": f"{decode_ms:.1f}",
                "ocr_result_count": len(last_ocr_results),
                "ocr_memory_count": len(memory_results),
                "ocr_unique_count": len(seen_texts),
                "ocr_results": _ocr_summary(last_ocr_results),
                "ocr_memory_results": _ocr_summary(memory_results),
                "ocr_error": ocr_stats.get("error", ""),
            })

            if now - last_diag_print >= 2.0:
                last_diag_print = now
                print(f"[DBG] fps={fps:.1f} objects={len(object_detections)} "
                      f"max_conf={object_max_conf:.2f} rois={len(object_rois)} "
                      f"fallback={int(roi_fallback)} raw_age_ms={raw_age_ms:.0f} "
                      f"ocr_busy={int(ocr_busy)} ocr_ms={decode_ms:.0f} "
                      f"ocr={len(last_ocr_results)}/{len(memory_results)}")

            cv2.imshow("OCR Test - RealSense", annotated)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s") and annotated is not None:
                _save_snapshot(annotated)
            if key == ord("r") and frame is not None:
                _save_debug_rois(frame, frame, last_object_rois)
            if not client.is_connected:
                print("[!] 连接已断开")
                break

    except KeyboardInterrupt:
        print("\n[*] 中断退出")
    finally:
        try:
            subscriber.unsubscribe()
        except Exception:
            pass
        if raw_subscriber is not None:
            try:
                raw_subscriber.unsubscribe()
            except Exception:
                pass
        try:
            ocr_worker.stop()
        except Exception:
            pass
        try:
            diag_file.close()
            print(f"[✓] debug CSV 已保存: {diag_path}")
        except Exception:
            pass
        _safe_terminate(client)
        cv2.destroyAllWindows()

        print()
        print("=" * 60)
        print(f"共识别到 {len(seen_texts)} 个唯一文字:")
        for idx, text in enumerate(sorted(seen_texts), start=1):
            print(f"  {idx}. {text}")
        print("=" * 60)


if __name__ == "__main__":
    main()
