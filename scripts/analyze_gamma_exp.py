#!/usr/bin/env python3
"""
Analyze OV7725 7-mode RAW Gamma/DSP_Ctrl experiment captures (GAM_G0..G6.dng)
and optionally export a standards-compliant 8-bit linearized DNG (310 KB) using
Tag 0xC618 (LinearizationTable) inverted from OV7725 GAM1..GAM15.

Usage:
    python3 scripts/analyze_gamma_exp.py <dir_with_GAM_G_dngs> [--export-dng out.dng]
"""

import argparse
import glob
import os
import struct
import sys

# OV7725 built-in GAM1..GAM15 + SLOP piecewise linear curve (from ov7725cfg.h)
GAM_XS_8  = [0x00, 0x04, 0x08, 0x10, 0x20, 0x28, 0x30, 0x38, 0x40, 0x48, 0x50, 0x60, 0x70, 0x90, 0xB0, 0xD0, 0x100]
GAM_YS_8  = [0x00, 0x0C, 0x16, 0x2A, 0x4E, 0x61, 0x6F, 0x7B, 0x86, 0x8E, 0x97, 0xA4, 0xAF, 0xC5, 0xD7, 0xE8, 0x100]


def build_inv_gamma_10():
    """Invert OV7725 GAM1..GAM15 into a 256-entry 10-bit linear LUT (0..1016)."""
    xs_10 = [x * 4 for x in GAM_XS_8]
    inv = []
    for y in range(256):
        val = 1016
        for i in range(len(GAM_YS_8) - 1):
            if GAM_YS_8[i] <= y <= GAM_YS_8[i + 1]:
                dy = GAM_YS_8[i + 1] - GAM_YS_8[i]
                val = xs_10[i] if dy == 0 else round(xs_10[i] + (xs_10[i + 1] - xs_10[i]) * (y - GAM_YS_8[i]) / dy)
                break
        inv.append(min(1016, max(0, val)))
    return inv


def eval_gamma_8(x):
    for i in range(len(GAM_XS_8) - 1):
        if GAM_XS_8[i] <= x <= GAM_XS_8[i + 1]:
            return GAM_YS_8[i] + (GAM_YS_8[i + 1] - GAM_YS_8[i]) * (x - GAM_XS_8[i]) / (GAM_XS_8[i + 1] - GAM_XS_8[i])
    return 255.0


def load_raw_8bit(path):
    """Load 644x480 8-bit raw payload from either legacy 16-bit-container (618 KB) or 8-bit (310 KB) DNG."""
    data = open(path, "rb").read()
    if len(data) == 618832:
        return data[0x250:][1::2], data[:0x250]
    if len(data) == 310240:
        return data[0x460:], data[:0x460]
    raise ValueError(f"Unexpected DNG size {len(data)} for {path}")


def export_linearized_dng(g0_path, g4_px, inv_10, out_path):
    orig_hdr = bytearray(open(g0_path, "rb").read()[:0x250])
    n_tags = struct.unpack_from("<H", orig_hdr, 8)[0]
    entries = [list(struct.unpack_from("<HHII", orig_hdr, 10 + i * 12)) for i in range(n_tags)]

    shift = 16
    lin_off = 0x250 + shift
    strip_off = lin_off + 512

    type_size = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8, 7: 1, 10: 8, 12: 8}
    new_entries = []
    for tag, typ, cnt, val in entries:
        is_ptr = (cnt * type_size.get(typ, 1)) > 4
        if tag == 0x0102:
            val = 8
        elif tag == 0x0111:
            val = strip_off
        elif tag == 0x0117:
            val = 644 * 480
        elif tag == 0xC61D:
            val = 1016
        elif is_ptr:
            val += shift
        new_entries.append((tag, typ, cnt, val))

    new_entries.append((0xC618, 3, 256, lin_off))
    new_entries.sort(key=lambda e: e[0])

    new_hdr = bytearray(strip_off)
    new_hdr[0:8] = orig_hdr[0:8]
    struct.pack_into("<H", new_hdr, 8, len(new_entries))
    for i, (tag, typ, cnt, val) in enumerate(new_entries):
        struct.pack_into("<HHII", new_hdr, 10 + i * 12, tag, typ, cnt, val)
    new_hdr[0x180 + shift : 0x250 + shift] = orig_hdr[0x180:0x250]

    bl_off = [v for t, _, _, v in new_entries if t == 0xC61A][0]
    struct.pack_into("<HHHH", new_hdr, bl_off, 8, 8, 8, 8)
    for i, v in enumerate(inv_10):
        struct.pack_into("<H", new_hdr, lin_off + i * 2, v)

    with open(out_path, "wb") as f:
        f.write(new_hdr)
        f.write(g4_px)
    print(f"\nExported 8-bit linearized DNG: {out_path} ({os.path.getsize(out_path)} bytes)")


def main():
    ap = argparse.ArgumentParser(description="Analyze OV7725 GAM_G0..G6 DNG experiment captures.")
    ap.add_argument("directory", help="Directory containing GAM_G*.dng files")
    ap.add_argument("--export-dng", help="Optional path to export linearized 310 KB DNG from GAM_G4")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.directory, "GAM_G*.[dD][nN][gG]")))
    if not files:
        print(f"No GAM_G*.dng found in {args.directory}", file=sys.stderr)
        return 1

    inv_10 = build_inv_gamma_10()
    imgs = {}
    paths = {}
    for path in files:
        name = os.path.basename(path)
        px, _ = load_raw_8bit(path)
        imgs[name] = px
        paths[name] = path

        hist = [0] * 256
        for v in px:
            hist[v] += 1
        mean = sum(i * c for i, c in enumerate(hist)) / len(px)
        nz_1_16 = sum(1 for i in range(1, 17) if hist[i] > 0)
        nz_1_42 = sum(1 for i in range(1, 43) if hist[i] > 0)

        ch = [0, 0, 0, 0]
        cnt = 0
        for y in range(0, 480, 2):
            r0, r1 = y * 644, (y + 1) * 644
            for x in range(0, 640, 2):
                ch[0] += px[r0 + x]
                ch[1] += px[r0 + x + 1]
                ch[2] += px[r1 + x]
                ch[3] += px[r1 + x + 1]
                cnt += 1
        ch = [c / cnt for c in ch]
        print(f"=== {name} ===")
        print(f"  mean={mean:.2f} | nz[1..16]={nz_1_16}/16, nz[1..42]={nz_1_42}/42")
        print(f"  GRBG: G0={ch[0]:.2f}, R={ch[1]:.2f}, B={ch[2]:.2f}, G1={ch[3]:.2f}")
        print(f"  hist[0..24]: {hist[:25]}")

    g0_keys = [k for k in imgs if "G0" in k.upper()]
    g4_keys = [k for k in imgs if "G4" in k.upper()]
    if g0_keys and g4_keys:
        g0, g4 = imgs[g0_keys[0]], imgs[g4_keys[0]]
        g4_lin = [inv_10[v] / 4.0 for v in g4]
        print(f"\nRound-trip mean: {g0_keys[0]}={sum(g0)/len(g0):.2f} vs {g4_keys[0]} linearized (/4)={sum(g4_lin)/len(g4_lin):.2f}")
        print("Empirical G0 -> G4 vs theoretical GAM1..GAM15:")
        for x in [4, 6, 8, 12, 16, 20, 24, 32, 40, 48, 64, 80, 96, 128]:
            vals = sorted(g4[i] for i in range(len(g0)) if g0[i] == x)
            if vals:
                med = vals[len(vals) // 2]
                theo = eval_gamma_8(x)
                print(f"  in={x:3d} (0x{x:02x}): theo={theo:6.2f}, emp_med={med:3d}, diff={med - theo:+5.2f} (n={len(vals)})")

        if args.export_dng:
            export_linearized_dng(paths[g0_keys[0]], g4, inv_10, args.export_dng)

    return 0


if __name__ == "__main__":
    sys.exit(main())
