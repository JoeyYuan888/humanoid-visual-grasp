"""
半自动文字标注工具 — 检测图片中的文字区域，用户输入正确文本。

用法:
    python tools/debug/label_ocr_dataset.py <图片路径/目录>

流程:
    1. 自动检测图片中的文字区域并裁剪
    2. 弹出窗口显示裁剪图和原图位置
    3. 用户输入正确文字后保存到 CSV
    4. 所有裁剪图保存到 data/train_crops/

操作:
    输入文字后按 Enter      → 保存并进入下一条
    直接按 Enter（空输入）  → 跳过该区域
    q 按 Enter              → 退出保存
    s 按 Enter              → 手动保存进度
"""

import os
import sys
import csv
import time

import cv2
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot_grasp.vision.ocr_detector import (
    OCRDetector, _ensure_dict, _ensure_onnx_model,
)


def load_images(path):
    import glob
    if os.path.isfile(path):
        return [path]
    exts = ("*.jpg", "*.jpeg", "*.png", "*.bmp")
    files = []
    for e in exts:
        files.extend(glob.glob(os.path.join(path, e)))
        files.extend(glob.glob(os.path.join(path, e.upper())))
    return sorted(files)


def input_with_opencv(prompt: str, image: np.ndarray) -> str:
    """在 OpenCV 窗口中显示图片和提示，用按键输入文字。

    使用 ASCII 按键输入，Enter 确认，Backspace 退格。
    注意：此输入法不适合中文输入，仅用于简单的中文字符。
    """
    return input(prompt)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return

    # 准备检测模型
    _ensure_dict()
    _ensure_onnx_model()
    detector = OCRDetector()
    if detector._rec_session is None:
        print("[✗] OCR 引擎初始化失败")
        return

    OUTPUT_CSV = os.path.join(PROJECT_ROOT, "data", "train_labels.csv")
    CROP_DIR = os.path.join(PROJECT_ROOT, "data", "train_crops")
    os.makedirs(CROP_DIR, exist_ok=True)

    images = load_images(sys.argv[1])
    print(f"[*] 共 {len(images)} 张图片\n")

    # 加载已有标注，避免重复
    existing = set()
    if os.path.exists(OUTPUT_CSV):
        with open(OUTPUT_CSV, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                existing.add(f"{row['image']}_{row['crop_file']}")
        print(f"[*] 已加载 {len(existing)} 条已有标注")

    f_out = open(OUTPUT_CSV, "a", newline="", encoding="utf-8")
    writer = csv.DictWriter(f_out, fieldnames=[
        "image", "crop_file", "text",
    ])
    if os.path.getsize(OUTPUT_CSV) == 0:
        writer.writeheader()

    total = 0
    try:
        for img_path in images:
            name = os.path.basename(img_path)
            print(f"\n── {name} ──")

            img = cv2.imread(img_path)
            if img is None:
                continue

            h, w = img.shape[:2]
            results = detector.detect(img, rois=[(0, 0, w, h)], upscale=2.0)

            if not results:
                print("  未检测到文字区域")
                cv2.imshow("Image", img)
                print("  手动输入整图文字（空=跳过）: ", end="", flush=True)
                text = input().strip()
                cv2.destroyWindow("Image")
                if text == "q":
                    break
                if not text:
                    continue
                crop_file = f"{name.replace('.', '_')}_full.png"
                cv2.imwrite(os.path.join(CROP_DIR, crop_file), img)
                writer.writerow({"image": name, "crop_file": crop_file, "text": text})
                total += 1
                print(f"  ✓ [{total}] \"{text}\"")
                continue

            text = ""  # 确保循环后 text 已定义（所有 ROI 被跳过时）
            for i, det in enumerate(results):
                x1, y1, x2, y2 = det["bbox"]
                crop = img[y1:y2, x1:x2]
                if crop.size == 0:
                    continue

                crop_file = f"{name.replace('.', '_')}_roi{i:02d}.png"
                key = f"{name}_{crop_file}"
                if key in existing:
                    continue

                # 显示裁剪放大图
                disp = cv2.resize(crop, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC)
                cv2.imshow("Text region (press q+Enter to quit)", disp)

                # 显示原图位置
                vis = img.copy()
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(vis, f"ROI {i+1}/{len(results)}",
                            (x1, max(20, y1-6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow("Source image", vis)
                cv2.waitKey(1)

                # 用户输入
                prompt = f"  正确文字 (Enter=跳过, q=退出): "
                text = input(prompt).strip()

                if text == "q":
                    break
                if not text:
                    print("  ⏭ 跳过")
                    continue

                cv2.imwrite(os.path.join(CROP_DIR, crop_file), crop)
                writer.writerow({
                    "image": name,
                    "crop_file": crop_file,
                    "text": text,
                })
                total += 1
                print(f"  ✓ [{total}] \"{text}\"")

            if text == "q":
                break

    except KeyboardInterrupt:
        pass
    finally:
        f_out.close()
        cv2.destroyAllWindows()

    print(f"\n{'='*60}")
    print(f"标注完成! 共 {total} 条")
    print(f"标注文件: {OUTPUT_CSV}")
    print(f"裁剪图片: {CROP_DIR}/")
    print(f"{'='*60}")
    print(f"\n接下来运行训练脚本即可开始训练。")
    print(f"训练需要至少 20 张标注图片才能启动。")


if __name__ == "__main__":
    main()
