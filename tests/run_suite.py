#!/usr/bin/env python3
"""Run NESUltimateEmu.Tests over one ROM or every .nes ROM under a directory."""
from __future__ import annotations
import argparse
import pathlib
import subprocess
import sys


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("runner", type=pathlib.Path, help="path to NESUltimateEmu.Tests.exe")
    parser.add_argument("roms", type=pathlib.Path, help="test ROM or directory")
    parser.add_argument("--max-cycles", type=int, default=120_000_000)
    args = parser.parse_args()

    if args.roms.is_file():
        roms = [args.roms]
    else:
        roms = sorted(args.roms.rglob("*.nes"))
    if not roms:
        print("No .nes test ROMs found.", file=sys.stderr)
        return 2

    results: list[tuple[pathlib.Path, int]] = []
    for rom in roms:
        print(f"\n=== {rom} ===")
        cp = subprocess.run([
            str(args.runner), str(rom), "--max-cycles", str(args.max_cycles)
        ])
        results.append((rom, cp.returncode))

    passed = sum(code == 0 for _, code in results)
    failed = len(results) - passed
    print(f"\n=== SUMMARY: {passed}/{len(results)} passed; {failed} failed/unsupported ===")
    for rom, code in results:
        print(f"{'PASS' if code == 0 else 'FAIL'} [{code}] {rom}")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
