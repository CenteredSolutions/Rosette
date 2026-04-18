#!/usr/bin/env python3
"""Generate binary fuzz corpus files from a hex vector manifest."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vectors", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    args = ap.parse_args()

    with args.vectors.open("r", encoding="utf-8") as f:
        blob = json.load(f)

    args.out.mkdir(parents=True, exist_ok=True)

    for item in blob.get("vectors", []):
        name = item["name"]
        hex_bytes = item["hex"].replace(" ", "").replace("\n", "")
        payload = bytes.fromhex(hex_bytes)
        (args.out / f"{name}.bin").write_bytes(payload)

    print(f"Generated {len(blob.get('vectors', []))} corpus files in {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

