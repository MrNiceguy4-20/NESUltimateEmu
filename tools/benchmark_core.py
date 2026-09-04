
"""Repeat the headless core benchmark and report robust median throughput."""
from __future__ import annotations
import argparse, json, statistics, subprocess, sys
from pathlib import Path

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('runner', type=Path)
    ap.add_argument('rom', type=Path)
    ap.add_argument('--cycles', type=int, default=20_000_000)
    ap.add_argument('--runs', type=int, default=5)
    args = ap.parse_args()
    if args.runs < 1 or args.cycles < 1:
        ap.error('--runs and --cycles must be positive')

    samples = []
    for i in range(args.runs):
        proc = subprocess.run(
            [str(args.runner), str(args.rom), '--benchmark-cycles', str(args.cycles), '--json'],
            check=True, text=True, capture_output=True,
        )
        line = next((ln for ln in reversed(proc.stdout.splitlines()) if ln.startswith('{')), None)
        if not line:
            print(proc.stdout, file=sys.stderr)
            raise RuntimeError('benchmark JSON was not produced')
        result = json.loads(line)
        samples.append(result)
        print(f"run {i+1}/{args.runs}: {result['realtime_multiple']:.4f}x, "
              f"{result['cycles_per_second']:.0f} cycles/s")

    cps = [float(s['cycles_per_second']) for s in samples]
    rt = [float(s['realtime_multiple']) for s in samples]
    fps = [float(s['equivalent_fps']) for s in samples]
    summary = {
        'runs': args.runs,
        'cycles_per_run': args.cycles,
        'median_cycles_per_second': statistics.median(cps),
        'min_cycles_per_second': min(cps),
        'max_cycles_per_second': max(cps),
        'median_realtime_multiple': statistics.median(rt),
        'median_equivalent_fps': statistics.median(fps),
    }
    print(json.dumps(summary, indent=2))
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
