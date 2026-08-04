"""
分析 run_grasp.py 保存的性能采样。

用法:
    python tools/debugs/analyze_perf.py
    python tools/debugs/analyze_perf.py data/grasp_data_YYYYMMDD_HHMMSS.csv
"""

import csv
import json
import glob
import os
import sys
from collections import Counter


METRICS = [
    "display_fps",
    "rgb_rx_fps",
    "depth_msg_rx_fps",
    "depth_rx_fps",
    "detect_fps",
    "infer_ms",
    "depth_age_ms",
    "qr_decode_ms",
    "raw_rgb_age_ms",
]


def _latest_csv() -> str | None:
    files = glob.glob(os.path.join("data", "grasp_data_*.csv"))
    return max(files, key=os.path.getmtime) if files else None


def _values(rows: list[dict], key: str) -> list[float]:
    vals = []
    for row in rows:
        value = row.get(key, "")
        if value == "":
            continue
        vals.append(float(value))
    return vals


def _summary(vals: list[float]) -> str:
    if not vals:
        return "n/a"
    return f"avg={sum(vals) / len(vals):.1f}, min={min(vals):.1f}, max={max(vals):.1f}"


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else _latest_csv()
    if not path:
        print("没有找到 data/grasp_data_*.csv")
        sys.exit(1)

    with open(path, newline="") as f:
        rows = [row for row in csv.DictReader(f) if row.get("type") == "perf"]

    print(f"文件: {path}")
    print(f"性能采样: {len(rows)} 条")
    if not rows:
        print("没有 type=perf 的性能采样；请先运行新版 run_grasp.py 并按 s 保存。")
        return

    print()
    for metric in METRICS:
        print(f"{metric:14s} {_summary(_values(rows, metric))}")

    statuses = Counter(row.get("grasp_status", "") for row in rows if row.get("grasp_status", ""))
    if statuses:
        print()
        print("grasp_status:")
        for status, count in statuses.most_common():
            print(f"  {status}: {count}")

    roi_valid = _values(rows, "roi_valid_count")
    roi_total = _values(rows, "roi_total_count")
    if roi_valid:
        print()
        print(f"roi_valid_count {_summary(roi_valid)}")
    if roi_total:
        print(f"roi_total_count {_summary(roi_total)}")

    qr_live = _values(rows, "qr_live_count")
    qr_memory = _values(rows, "qr_memory_count")
    if qr_live or qr_memory:
        print()
        if qr_live:
            print(f"qr_live_count {_summary(qr_live)}")
        if qr_memory:
            print(f"qr_memory_count {_summary(qr_memory)}")
        scan_modes = Counter(row.get("qr_scan_mode", "") for row in rows if row.get("qr_scan_mode", ""))
        if scan_modes:
            print("qr_scan_mode:")
            for mode, count in scan_modes.most_common():
                print(f"  {mode}: {count}")
        qr_sources = Counter(row.get("qr_source", "") for row in rows if row.get("qr_source", ""))
        if qr_sources:
            print("qr_source:")
            for source, count in qr_sources.most_common():
                print(f"  {source}: {count}")
        latest_qr = next((row.get("qr_texts", "") for row in reversed(rows) if row.get("qr_texts", "")), "")
        if latest_qr:
            print(f"最后锁存 QR: {latest_qr}")

    det_summaries = [row.get("det_summary", "") for row in rows if row.get("det_summary", "")]
    if det_summaries:
        class_counts = Counter()
        for summary in det_summaries:
            labels = {item.split(":", 1)[0] for item in summary.split(";") if ":" in item}
            class_counts.update(labels)
        print()
        print("检测类别出现采样数:")
        for label, count in class_counts.most_common():
            print(f"  {label}: {count}")
        print(f"最后检测: {det_summaries[-1]}")

    latest_objects = next((row.get("object_results", "") for row in reversed(rows) if row.get("object_results", "")), "")
    if latest_objects:
        print()
        print("最后对象结果:")
        try:
            objects = json.loads(latest_objects)
            for obj in objects:
                xyz = (
                    f"({obj.get('x_mm')}, {obj.get('y_mm')}, {obj.get('z_mm')})"
                    if obj.get("valid") else obj.get("status", "")
                )
                print(
                    f"  #{obj.get('idx')} {obj.get('label')} conf={obj.get('conf')} "
                    f"qr={obj.get('qr', '')} xyz={xyz}"
                )
        except json.JSONDecodeError:
            print(f"  {latest_objects}")

    display = _values(rows, "display_fps")
    detect = _values(rows, "detect_fps")
    depth = _values(rows, "depth_rx_fps")
    depth_msg = _values(rows, "depth_msg_rx_fps")
    depth_age = _values(rows, "depth_age_ms")
    infer = _values(rows, "infer_ms")

    avg_display = sum(display) / len(display) if display else 0
    avg_detect = sum(detect) / len(detect) if detect else 0
    avg_depth = sum(depth) / len(depth) if depth else 0
    avg_depth_msg = sum(depth_msg) / len(depth_msg) if depth_msg else avg_depth
    avg_depth_age = sum(depth_age) / len(depth_age) if depth_age else 9999
    avg_infer = sum(infer) / len(infer) if infer else 9999

    ok = (
        avg_display >= 10
        and avg_detect >= 4
        and avg_depth >= 1.5
        and avg_depth_msg >= 1.5
        and avg_depth_age <= 1000
        and avg_infer <= 250
    )

    print()
    print("抓取测试建议阈值:")
    print("  display_fps >= 10, detect_fps >= 4, depth_rx_fps >= 1.5, depth_age_ms <= 1000, infer_ms <= 250")
    print(f"结论: {'基本够用' if ok else '还需要优化'}")


if __name__ == "__main__":
    main()
