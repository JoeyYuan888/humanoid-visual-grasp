#!/usr/bin/env python3
"""
从 Ultralytics NDJSON 导出文件下载 Marine Plastic Debris Detection Dataset。

使用方法:
    python tools/download_dataset.py

将会在 data/marine-plastic-debris/ 目录下创建 YOLO 格式的数据集:
    data/marine-plastic-debris/
    ├── images/
    │   ├── train/
    │   ├── val/
    │   └── test/
    ├── labels/
    │   ├── train/
    │   ├── val/
    │   └── test/
    └── dataset.yaml
"""

import json
import sys
import time
import urllib.request
from pathlib import Path

# ============================================================
# 配置
# ============================================================
NDJSON_PATH = "/home/hmit/Downloads/marine-plastic-debris-detection-dataset (1).ndjson"
OUTPUT_DIR = Path("/home/hmit/naviai/center/data/marine-plastic-debris")

# 类别名称 (来自 NDJSON 第一行的 class_names)
CLASS_NAMES = {
    "0": "Bottle Cap",
    "1": "Chips Bag",
    "2": "Juice Box",
    "3": "Plastic Debris",
    "4": "Plastic Bag",
    "5": "Plastic Bottle",
    "6": "Plastic Cup",
}


class ProgressBar:
    """终端进度条，带速度显示"""

    def __init__(self, total_items, bar_width=40):
        self.total = total_items
        self.bar_width = bar_width
        self.start_time = time.time()
        self.current = 0
        self.current_file = ""
        self.file_progress = 0.0  # 0~1
        self.phase = "preparing"  # preparing / downloading / labeling
        self.last_update = 0.0
        self.bytes_downloaded = 0  # 实际已下载字节数

    def set_file(self, filename):
        self.current_file = filename

    def set_phase(self, phase):
        self.phase = phase
        self._render()

    def update_file_progress(self, fraction):
        """更新当前文件下载进度 (0~1)"""
        self.file_progress = fraction
        now = time.time()
        if now - self.last_update > 0.1:  # 限制刷新率 ~10fps
            self.last_update = now
            self._render()

    def add_bytes(self, n):
        self.bytes_downloaded += n

    def inc(self, n=1):
        self.current += n
        self.file_progress = 0.0
        self._render()

    def _format_speed(self, elapsed):
        """基于实际下载字节数计算速度"""
        if elapsed <= 0 or self.bytes_downloaded <= 0:
            return "?"
        speed = self.bytes_downloaded / elapsed
        if speed > 1024 * 1024:
            return f"{speed / 1024 / 1024:.1f} MB/s"
        elif speed > 1024:
            return f"{speed / 1024:.0f} KB/s"
        return f"{speed:.0f} B/s"

    def _format_eta(self, elapsed):
        if self.current == 0:
            return "?"
        remaining = self.total - self.current
        rate = self.current / elapsed
        eta_sec = remaining / rate if rate > 0 else 0
        if eta_sec > 3600:
            return f"{eta_sec / 3600:.1f}h"
        elif eta_sec > 60:
            return f"{eta_sec / 60:.0f}m"
        return f"{eta_sec:.0f}s"

    def _render(self):
        elapsed = time.time() - self.start_time
        pct = self.current / self.total if self.total > 0 else 0
        filled = int(self.bar_width * pct)
        bar = "█" * filled + "░" * (self.bar_width - filled)

        speed = self._format_speed(elapsed)
        eta = self._format_eta(elapsed)

        # 状态行: 总体进度条 + 统计
        line = (
            f"\r  {bar} {self.current}/{self.total} "
            f"| ⚡ {speed} | ⏱ ETA {eta} | {self.current * 100 // self.total}%"
        )

        # 第二行: 当前文件信息
        fname = self.current_file[:50] if len(self.current_file) > 50 else self.current_file
        if self.phase == "downloading" and self.current_file:
            file_pct = int(self.file_progress * 100)
            file_bar = "█" * (file_pct // 5) + "░" * (20 - file_pct // 5)
            line += f"\n  📄 [{file_bar}] {fname} ({file_pct}%)"
        elif self.phase == "labeling":
            line += f"\n  🏷️  生成标签: {fname}"
        else:
            line += f"\n  {'  ':<52}"

        sys.stdout.write(line)
        sys.stdout.flush()

    def done(self):
        elapsed = time.time() - self.start_time
        speed = self._format_speed(elapsed)
        bar = "█" * self.bar_width
        total_mb = self.bytes_downloaded / 1024 / 1024
        print(f"\r  {bar} {self.total}/{self.total} | ✅ 完成 | 总耗时 {elapsed:.1f}s | 平均 {speed} | 总计 {total_mb:.1f} MB")


def download_with_progress(url, save_path, progress_callback, bytes_callback):
    """带进度的文件下载，使用 urllib 的 reporthook"""
    try:

        def reporthook(block_count, block_size, total_size):
            if total_size > 0:
                fraction = min(block_count * block_size / total_size, 1.0)
                progress_callback(fraction)
            bytes_callback(block_size)

        urllib.request.urlretrieve(url, save_path, reporthook)
        return True
    except Exception as e:
        print(f"\n  ❌ 下载失败: {e}")
        return False


def save_yolo_label(annotations, img_width, img_height, label_path):
    """将标注保存为 YOLO 格式 (class_id cx cy w h)"""
    if not annotations or "boxes" not in annotations:
        return

    boxes = annotations["boxes"]
    with open(label_path, "w") as f:
        for box in boxes:
            class_id = int(box[0])
            cx = box[1]
            cy = box[2]
            w = box[3]
            h = box[4]
            f.write(f"{class_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")


def main():
    print("=" * 60)
    print("🌊 Marine Plastic Debris Detection Dataset 下载工具")
    print("=" * 60)

    # 确保输出目录存在
    for split in ["train", "val", "test"]:
        (OUTPUT_DIR / "images" / split).mkdir(parents=True, exist_ok=True)
        (OUTPUT_DIR / "labels" / split).mkdir(parents=True, exist_ok=True)

    # 统计信息
    stats = {"train": 0, "val": 0, "test": 0}
    downloaded = 0
    skipped = 0
    failed = 0

    # 读取 NDJSON 文件
    print(f"\n📖 正在读取 {NDJSON_PATH} ...")
    with open(NDJSON_PATH, "r") as f:
        lines = f.readlines()

    # 第一行是 dataset 元数据，跳过
    entries = []
    for line in lines[1:]:
        line = line.strip()
        if line:
            try:
                entry = json.loads(line)
                entries.append(entry)
            except json.JSONDecodeError as e:
                print(f"  ⚠️ 解析 JSON 失败: {e}")

    total = len(entries)
    print(f"📊 共发现 {total} 张图片\n")

    # 初始化进度条
    pb = ProgressBar(total)
    pb.set_phase("downloading")

    for i, entry in enumerate(entries):
        img_file = entry.get("file", f"image_{i:06d}.jpg")
        img_url = entry.get("url", "")
        split = entry.get("split", "train")
        width = entry.get("width", 640)
        height = entry.get("height", 640)
        annotations = entry.get("annotations", {})

        if not img_url:
            pb.inc()
            failed += 1
            continue

        img_path = OUTPUT_DIR / "images" / split / img_file

        # 检查是否已下载
        if img_path.exists():
            stats[split] += 1
            skipped += 1
            pb.inc()
            continue

        # 下载图片 (带进度)
        pb.set_file(img_file)
        pb.set_phase("downloading")
        success = download_with_progress(
            img_url, img_path,
            lambda f: pb.update_file_progress(f),
            lambda b: pb.add_bytes(b)
        )

        if success:
            stats[split] += 1
            downloaded += 1

            # 保存标签
            pb.set_phase("labeling")
            pb.update_file_progress(0)
            label_file = Path(img_file).stem + ".txt"
            label_path = OUTPUT_DIR / "labels" / split / label_file
            save_yolo_label(annotations, width, height, label_path)
        else:
            failed += 1

        pb.inc()

    # 完成
    pb.done()

    # 生成 dataset.yaml
    yaml_path = OUTPUT_DIR / "dataset.yaml"
    class_lines = []
    for cls_id in sorted(CLASS_NAMES.keys(), key=int):
        class_lines.append(f"  {cls_id}: {CLASS_NAMES[cls_id]}")

    yaml_content = f"""# Marine Plastic Debris Detection Dataset
# 来源: https://platform.ultralytics.com/meerkat-swan/datasets/marine-plastic-debris-detection-dataset

train: {OUTPUT_DIR}/images/train
val: {OUTPUT_DIR}/images/val
test: {OUTPUT_DIR}/images/test

nc: {len(CLASS_NAMES)}
names:
{chr(10).join(class_lines)}
"""
    with open(yaml_path, "w") as f:
        f.write(yaml_content)

    print("\n" + "=" * 60)
    print("✅ 处理完成!")
    print(f"   📂 保存路径: {OUTPUT_DIR}")
    print(f"   📊 数据集划分:")
    print(f"      train: {stats['train']} 张")
    print(f"      val:   {stats['val']} 张")
    print(f"      test:  {stats['test']} 张")
    print(f"   ✅ 本次新增下载: {downloaded}")
    print(f"   ⏭️  已跳过(已存在): {skipped}")
    print(f"   ❌ 失败: {failed}")
    print(f"   📝 标签: YOLO 格式 (class_id cx cy w h)")
    print(f"   📄 配置: {yaml_path}")
    print("=" * 60)


if __name__ == "__main__":
    main()
