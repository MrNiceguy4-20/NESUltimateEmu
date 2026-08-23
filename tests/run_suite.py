#!/usr/bin/env python3
"""Run NESUltimate built-in probes and external hardware-conformance ROMs.

Supports simple recursive ROM discovery and the christopherpow/nes-test-roms
`test_roms.xml` manifest. Manifest mode honors each test's NTSC/PAL system tag
by forcing the headless runner's timing profile, so PAL tests are not silently
run with an NTSC header/default.
"""
from __future__ import annotations
import argparse
import json
import pathlib
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, asdict
from typing import Iterable

@dataclass
class TestCase:
    path: pathlib.Path
    timing: str = "auto"
    runframes: int | None = None
    source: str = "discovery"

@dataclass
class TestResult:
    path: str
    timing: str
    exit_code: int
    outcome: str
    seconds: float

def classify(code: int) -> str:
    return {0: "pass", 1: "fail", 2: "load-error", 3: "timeout", 4: "unsupported"}.get(code, "error")

def run_command(args: list[str]) -> int:
    return subprocess.run(args).returncode

def discover_roms(root: pathlib.Path) -> list[TestCase]:
    if root.is_file():
        return [TestCase(root)]
    return [TestCase(p) for p in sorted(root.rglob("*.nes"))]

def manifest_cases(manifest: pathlib.Path, rom_root: pathlib.Path | None) -> list[TestCase]:
    tree = ET.parse(manifest)
    base = rom_root if rom_root is not None else manifest.parent
    cases: list[TestCase] = []
    for node in tree.getroot().findall("test"):
        filename = node.get("filename")
        if not filename:
            continue
        path = base / pathlib.PurePosixPath(filename)
        system = (node.get("system") or "").strip().lower()
        timing = system if system in {"ntsc", "pal", "dendy"} else "auto"
        frames = None
        raw_frames = node.get("runframes")
        if raw_frames and raw_frames.isdigit():
            frames = int(raw_frames)
        cases.append(TestCase(path=path, timing=timing, runframes=frames, source="manifest"))
    return cases

def filter_cases(cases: Iterable[TestCase], selectors: list[str]) -> list[TestCase]:
    out = list(cases)
    if not selectors:
        return out
    wanted = [s.replace("\\", "/").lower() for s in selectors]
    return [c for c in out if any(s in c.path.as_posix().lower() for s in wanted)]

def write_json(path: pathlib.Path, builtins_passed: bool, results: list[TestResult]) -> None:
    summary: dict[str, int] = {}
    for r in results:
        summary[r.outcome] = summary.get(r.outcome, 0) + 1
    payload = {
        "builtins_passed": builtins_passed,
        "summary": summary,
        "tests": [asdict(r) for r in results],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("runner", type=pathlib.Path, help="path to NESUltimateEmu.Tests executable")
    parser.add_argument("roms", nargs="?", type=pathlib.Path, help="optional test ROM or directory")
    parser.add_argument("--manifest", type=pathlib.Path, help="test_roms.xml manifest from an external test corpus")
    parser.add_argument("--rom-root", type=pathlib.Path, help="root containing manifest ROM paths (defaults to manifest directory)")
    parser.add_argument("--suite", action="append", default=[], help="only run paths containing this text; repeatable")
    parser.add_argument("--timing", choices=["auto", "ntsc", "pal", "dendy"], default=None,
                        help="force one timing for all tests; otherwise manifest metadata/ROM timing is used")
    parser.add_argument("--max-cycles", type=int, default=150_000_000)
    parser.add_argument("--skip-builtins", action="store_true", help="skip built-in hardware probes")
    parser.add_argument("--json", type=pathlib.Path, help="write machine-readable result report")
    parser.add_argument("--list", action="store_true", help="list selected external tests without running them")
    parser.add_argument("--fail-fast", action="store_true")
    args = parser.parse_args()

    runner_path = args.runner.expanduser().resolve()
    if not runner_path.is_file():
        print(f"ERROR: regression runner not found: {runner_path}", file=sys.stderr)
        return 2
    runner = str(runner_path)
    builtins_passed = True
    if not args.skip_builtins and not args.list:
        print("=== BUILT-IN REGRESSION PROBES ===", flush=True)
        code = run_command([runner, "--self-test"])
        builtins_passed = code == 0
        if not builtins_passed:
            print(f"Built-in regression probes failed with exit code {code}.", file=sys.stderr)
            if args.json:
                write_json(args.json, False, [])
            return 1

    if args.manifest:
        cases = manifest_cases(args.manifest, args.rom_root)
    elif args.roms is not None:
        cases = discover_roms(args.roms)
    else:
        return 0

    cases = filter_cases(cases, args.suite)
    if args.timing:
        for case in cases:
            case.timing = args.timing

    if not cases:
        print("No external .nes tests selected.", file=sys.stderr)
        return 2

    missing = [c.path for c in cases if not c.path.is_file()]
    if missing:
        print(f"WARNING: {len(missing)} manifest ROM(s) are missing and will be reported as load-error.", file=sys.stderr)

    if args.list:
        for c in cases:
            print(f"{c.timing:5}  {c.path}")
        return 0

    results: list[TestResult] = []
    for index, case in enumerate(cases, 1):
        print(f"\n=== [{index}/{len(cases)}] {case.path} | {case.timing.upper()} ===", flush=True)
        command = [runner, str(case.path), "--max-cycles", str(args.max_cycles)]
        if case.timing != "auto":
            command += ["--timing", case.timing]
        start = time.monotonic()
        code = run_command(command) if case.path.is_file() else 2
        elapsed = time.monotonic() - start
        outcome = classify(code)
        results.append(TestResult(str(case.path), case.timing, code, outcome, round(elapsed, 3)))
        if args.fail_fast and code != 0:
            break

    counts: dict[str, int] = {}
    for r in results:
        counts[r.outcome] = counts.get(r.outcome, 0) + 1
    passed = counts.get("pass", 0)
    print(f"\n=== EXTERNAL ROM SUMMARY: {passed}/{len(results)} passed ===")
    for name in ("pass", "fail", "timeout", "unsupported", "load-error", "error"):
        if counts.get(name):
            print(f"  {name:11} {counts[name]}")
    for r in results:
        marker = "PASS" if r.outcome == "pass" else "FAIL"
        print(f"{marker} [{r.exit_code}] {r.timing.upper():5} {r.path}")

    if args.json:
        write_json(args.json, builtins_passed, results)
        print(f"JSON report: {args.json}")

    return 0 if all(r.exit_code == 0 for r in results) else 1

if __name__ == "__main__":
    raise SystemExit(main())
