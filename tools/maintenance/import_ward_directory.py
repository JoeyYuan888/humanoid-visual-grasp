#!/usr/bin/env python3
"""Import hospital ward directory Excel into the OCR ward dictionary JSON."""

from __future__ import annotations

import argparse
import json
import shutil
from datetime import datetime
from pathlib import Path

import pandas as pd


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SOURCE = Path("/home/hmit/Downloads/病区对应关系.xls")
DEFAULT_OUTPUT = PROJECT_ROOT / "configs" / "ocr" / "ward_directory.json"
DEFAULT_BACKUP = PROJECT_ROOT / "configs" / "ocr" / "ward_directory_source_20260812.xls"
REQUIRED_COLUMNS = ["BINGQUID", "BINGQUMC", "PAODANWZ"]


def _unique(values: list[str]) -> list[str]:
    return list(dict.fromkeys(v for v in values if v))


def _build_aliases(ward_id: str, name: str) -> list[str]:
    aliases = [f"{ward_id}{name}", name]
    for text in list(aliases):
        aliases.append(text.replace("（", "(").replace("）", ")"))
        aliases.append(text.replace("(", "（").replace(")", "）"))
    return _unique(aliases)


def import_ward_directory(source: Path, output: Path, backup: Path | None) -> dict:
    df = pd.read_excel(source, dtype=str).fillna("")
    missing = [column for column in REQUIRED_COLUMNS if column not in df.columns]
    if missing:
        raise ValueError(f"missing required columns: {missing}")

    wards = []
    seen = set()
    for _, row in df.iterrows():
        ward_id = str(row["BINGQUID"]).strip()
        name = str(row["BINGQUMC"]).strip()
        location = str(row["PAODANWZ"]).strip()
        if not ward_id or not name:
            continue
        canonical = f"{ward_id}{name}"
        if canonical in seen:
            continue
        seen.add(canonical)
        wards.append(
            {
                "id": ward_id,
                "name": name,
                "location": location,
                "canonical": canonical,
                "aliases": _build_aliases(ward_id, name),
            }
        )

    output.parent.mkdir(parents=True, exist_ok=True)
    if backup is not None:
        backup.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, backup)

    payload = {
        "source": str(backup.relative_to(PROJECT_ROOT) if backup else source),
        "source_columns": REQUIRED_COLUMNS,
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "count": len(wards),
        "wards": wards,
    }
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return payload


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default=str(DEFAULT_SOURCE))
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--backup", default=str(DEFAULT_BACKUP))
    parser.add_argument("--no-backup", action="store_true")
    args = parser.parse_args()

    payload = import_ward_directory(
        Path(args.source),
        Path(args.output),
        None if args.no_backup else Path(args.backup),
    )
    print(f"[✓] imported wards: {payload['count']}")
    print(f"[✓] dictionary: {args.output}")
    if not args.no_backup:
        print(f"[✓] source backup: {args.backup}")
    for item in payload["wards"][:10]:
        print(f"  {item['canonical']}  {item['location']}")


if __name__ == "__main__":
    main()
