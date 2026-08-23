#!/usr/bin/env python3
"""Import compatible NES .cht files from a local libretro-database checkout.

Usage:
  python tools/import_libretro_nes_cheats.py /path/to/libretro-database

The importer copies files into cheats/nes and reports how many contain at least
one code form currently understood by NESUltimateEmu (NES Game Genie or AAAA:VV).
The source database remains licensed CC-BY-SA-4.0; see cheats/README.md.
"""
from __future__ import annotations

import argparse
import re
import shutil
from pathlib import Path

GG = set("APZLGITYEOXUKSVN")
RAW = re.compile(r"^[0-9A-Fa-f]{1,4}:[0-9A-Fa-f]{1,2}$")
CODE = re.compile(r'^cheat\d+_code\s*=\s*"(.*)"\s*$')

def supported_token(token: str) -> bool:
    token = token.strip().replace("-", "").upper()
    return (len(token) in (6, 8) and all(ch in GG for ch in token)) or bool(RAW.fullmatch(token))

def file_has_supported_code(path: Path) -> bool:
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return False
    for line in text.splitlines():
        m = CODE.match(line.strip())
        if not m:
            continue
        if any(supported_token(part) for part in m.group(1).split("+")):
            return True
    return False

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("libretro_database", type=Path)
    ap.add_argument("--dest", type=Path, default=Path("cheats/nes"))
    args = ap.parse_args()

    source = args.libretro_database / "cht" / "Nintendo - Nintendo Entertainment System"
    if not source.is_dir():
        ap.error(f"NES cheat directory not found: {source}")

    args.dest.mkdir(parents=True, exist_ok=True)
    copied = skipped = 0
    for path in sorted(source.glob("*.cht")):
        if not file_has_supported_code(path):
            skipped += 1
            continue
        shutil.copy2(path, args.dest / path.name)
        copied += 1

    print(f"Imported {copied} compatible NES cheat files; skipped {skipped} unsupported files.")
    print("Data source: libretro/libretro-database (CC-BY-SA-4.0).")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
