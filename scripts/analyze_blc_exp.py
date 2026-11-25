#!/usr/bin/env python3
"""
Analyze OV7725 dark-frame BLC / COM13 calibration captures (BLC_*.dng)
and optionally compare cold vs warm directories in 10-bit linearized space.

Usage:
    python3 scripts/analyze_blc_exp.py <cold_dir> [<warm_dir>]
"""

import argparse
import glob
import math
import os
import struct
import sys

CH_NAMES = ("G0", "R ", "B ", "G1")


def analyze_dir(dirpath):
    results = {}
    for path in sorted(glob.glob(os.path.join(dirpath, "*.[dD][nN][gG]"))):
        data = open(path, "rb").read()
        if len(data) != 310240:
            print(f"Skipping non-310KB DNG: {path} ({len(data)} bytes)", file=sys.stderr)
            continue
        inv = struct.unpack("<256H", data[0x260:0x460])
        px = data[0x460:]

        ch = ([], [], [], [])
        for y in range(0, 480, 2):
            r0, r1 = y * 644, (y + 1) * 644
            for x in range(0, 640, 2):
                ch[0].append(inv[px[r0 + x]])
                ch[1].append(inv[px[r0 + x + 1]])
                ch[2].append(inv[px[r1 + x]])
                ch[3].append(inv[px[r1 + x + 1]])

        stats = []
        for arr in ch:
            n = len(arr)
            mean = sum(arr) / n
            std = math.sqrt(sum((x - mean) ** 2 for x in arr) / n)
            zpct = 100.0 * sum(1 for x in arr if x == 0) / n
            sarr = sorted(arr)
            stats.append((mean, sarr[n // 2], std, sarr[int(n * 0.99)], zpct))
        results[os.path.basename(path)] = stats
    return results


def main():
    ap = argparse.ArgumentParser(description="Analyze OV7725 dark-frame BLC captures (10-bit linearized).")
    ap.add_argument("cold_dir", help="Directory with cold dark-frame DNGs (e.g. test_blk_0)")
    ap.add_argument("warm_dir", nargs="?", help="Optional directory with warmed-up dark-frame DNGs (e.g. test_blk_1)")
    args = ap.parse_args()

    cold = analyze_dir(args.cold_dir)
    if not cold:
        print(f"No 310 KB DNGs found in {args.cold_dir}", file=sys.stderr)
        return 1

    if not args.warm_dir:
        for name, stats in sorted(cold.items()):
            print(f"\n=== {name} ===")
            for i, (mean, med, std, p99, zpct) in enumerate(stats):
                print(f"  {CH_NAMES[i]}: 10bit mean={mean:6.2f} med={med:3d} std={std:5.2f} p99={p99:3d} zero={zpct:5.1f}%")
        return 0

    warm = analyze_dir(args.warm_dir)
    for name in sorted(set(cold) & set(warm)):
        print(f"\n=== {name} (COLD -> WARM) ===")
        for i in range(4):
            cm, cmed, cstd, cp99, cz = cold[name][i]
            wm, wmed, wstd, wp99, wz = warm[name][i]
            print(
                f"  {CH_NAMES[i]}: mean {cm:6.2f} -> {wm:6.2f} ({wm - cm:+6.2f}) | "
                f"med {cmed:3d}->{wmed:3d} | std {cstd:5.2f}->{wstd:5.2f} | "
                f"p99 {cp99:3d}->{wp99:3d} | zero {cz:4.1f}%->{wz:4.1f}%"
            )
    return 0


if __name__ == "__main__":
    sys.exit(main())
