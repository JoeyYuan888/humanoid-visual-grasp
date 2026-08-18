"""
从 CSV 的最后一条 object_results 中单独 debug 抓取目标选择逻辑。

用法:
    python tools/debug/debug_select_target.py
    python tools/debug/debug_select_target.py data/samples/grasp_data_xxx.csv --label cup
"""

import argparse
import csv
import glob
import json
import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from robot_grasp.common.grasp_flow import select_grasp_target, summarize_target


def _latest_csv() -> str | None:
    files = glob.glob(os.path.join("data", "grasp_data_*.csv"))
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def _load_latest_objects(path: str) -> list[dict]:
    latest = ""
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            if row.get("object_results"):
                latest = row["object_results"]
    if not latest:
        return []
    return json.loads(latest)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", nargs="?")
    parser.add_argument("--label", help="只从指定类别里选目标，例如 cup/bottle")
    args = parser.parse_args()

    path = args.csv_path or _latest_csv()
    if not path:
        raise SystemExit("没有找到 data/samples/grasp_data_*.csv")

    objects = _load_latest_objects(path)
    print(f"文件: {path}")
    print("最后对象结果:")
    for obj in objects:
        print("  " + summarize_target(obj))

    target = select_grasp_target(objects, preferred_label=args.label)
    print("\n选择结果:")
    print("  " + summarize_target(target))


if __name__ == "__main__":
    main()
