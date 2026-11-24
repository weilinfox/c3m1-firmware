#!/usr/bin/env python3
"""
Minimal binary patcher for C3M1 factory firmware (C3M1生产烧录.hex)
fixing ONLY the CDI_456.DNG frame-wrap tear issue (NO SDIO changes applied).

Root cause (verified from CDI_456.DNG + factory disassembly):
  At 0x080016B2 -> 0x080016B6, wait_8_frames() clears ov_sta = 0 and enters
  camera_save_dng() (0x08000B98), which runs f_open(), f_write(592B header),
  and STMFLASH_Write(0x08070000) BEFORE waiting `while (ov_sta != 2)` at
  0x08000BF0. On slow SD cards, f_open/f_write(header) frequently finishes after
  VSYNC #1 sets ov_sta = 1 (FIFO_WEN = 1), so the subsequent 20-40 ms CPU/IRQ
  freeze during STMFLASH_Write's Flash page erase swallows VSYNC #2 and lets
  Frame 2 wrap around the 384 KB AL422B ring buffer, overwriting the first
  ~358 rows of Frame 1 with an odd horizontal shift (Bayer color flip).

Fix (single 4-byte call patch + 20-byte Thumb-2 trampoline at 0x0800E900):
  - 0x080016B6: `bl 0x08000B98` -> `bl 0x0800E900` (dng_lock_trampoline)
  - 0x0800E900: spin until ov_sta == 2 (FIFO_WEN locked LOW), then tail-call
    original camera_save_dng() (0x08000B99) with SP/LR untouched.

Optional Step-2 fix (`--fix-mirror`, Method 2: hardware upright + Orientation=1):
  - 0x08002632: OV7725 RAW COM3 (0x0C) `0x50` (VFLIP=0,HMIRROR=1) -> `0xD0`
                (VFLIP=1,HMIRROR=1, matching viewfinder preview at 0x0800254E)
  - 0x080080C6: DNG Tag 0x0112 Orientation `3` (180 deg) -> `1` (Normal)
  - 0x0800811A: DNG Tag 0x828E CFAPattern `BGGR` (02 01 01 00) -> `--cfa`
                (default `grbg` = 01 00 02 01 for VFLIP over 480 even rows)
"""

import argparse
import hashlib
import os
import struct
import subprocess
import tempfile

FLASH_BASE = 0x08000000
CAVE_ADDR  = 0x0800E900

CFA_PATTERNS = {
    "rggb": bytes([0, 1, 1, 2]),
    "grbg": bytes([1, 0, 2, 1]),
    "gbrg": bytes([1, 2, 0, 1]),
    "bggr": bytes([2, 1, 1, 0]),
}

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
DEFAULT_IN   = os.path.join(SCRIPT_DIR, "C3M1生产烧录.hex")
DEFAULT_OUT  = os.path.join(SCRIPT_DIR, "C3M1_patched_dng_tear.hex")
TRAMPOLINE_S = os.path.join(SCRIPT_DIR, "trampoline_dng_lock.s")


def parse_ihex(path):
    """Parse Intel HEX with per-line checksum validation."""
    chunks = {}
    upper = 0
    start_addr = None
    saw_eof = False
    with open(path, "r", encoding="ascii") as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            if not line.startswith(":"):
                raise ValueError(f"{path}:{lineno}: invalid Intel HEX line (missing ':')")
            raw = bytes.fromhex(line[1:])
            if len(raw) < 5:
                raise ValueError(f"{path}:{lineno}: Intel HEX record too short")
            cnt = raw[0]
            if len(raw) != cnt + 5:
                raise ValueError(f"{path}:{lineno}: expected {cnt + 5} bytes, got {len(raw)}")
            if (sum(raw) & 0xFF) != 0:
                raise ValueError(f"{path}:{lineno}: Intel HEX checksum mismatch")
            addr = (raw[1] << 8) | raw[2]
            rtype = raw[3]
            payload = raw[4:-1]
            if rtype == 0:
                chunks[upper + addr] = payload
            elif rtype == 1:
                saw_eof = True
                break
            elif rtype == 2:
                upper = int.from_bytes(payload, "big") << 4
            elif rtype == 4:
                upper = int.from_bytes(payload, "big") << 16
            elif rtype == 5:
                start_addr = int.from_bytes(payload, "big")
            else:
                raise ValueError(f"{path}:{lineno}: unsupported Intel HEX record type 0x{rtype:02X}")
    if not saw_eof or not chunks:
        raise ValueError(f"{path}: incomplete Intel HEX file")
    base = min(chunks)
    end = max(a + len(b) for a, b in chunks.items())
    buf = bytearray(b"\xFF" * (end - base))
    for a, b in chunks.items():
        buf[a - base : a - base + len(b)] = b
    return base, buf, start_addr


def write_ihex(path, base_addr, data, start_addr=None):
    """Write Intel HEX (16 bytes per record, with extended linear address)."""
    def record(addr16, rtype, payload):
        raw = bytes([len(payload), (addr16 >> 8) & 0xFF, addr16 & 0xFF, rtype]) + payload
        cksum = (-sum(raw)) & 0xFF
        return ":" + raw.hex().upper() + f"{cksum:02X}\n"

    lines = []
    cur_upper = None
    offset = 0
    total = len(data)
    while offset < total:
        full_addr = base_addr + offset
        upper = (full_addr >> 16) & 0xFFFF
        if upper != cur_upper:
            lines.append(record(0, 4, struct.pack(">H", upper)))
            cur_upper = upper
        chunk_len = min(16, total - offset, 0x10000 - (full_addr & 0xFFFF))
        lines.append(record(full_addr & 0xFFFF, 0, bytes(data[offset : offset + chunk_len])))
        offset += chunk_len

    if start_addr is not None:
        lines.append(record(0, 5, struct.pack(">I", start_addr)))
    lines.append(record(0, 1, b""))

    with open(path, "w", encoding="ascii", newline="\n") as f:
        f.writelines(lines)


def encode_thumb2_bl(src_addr, dst_addr):
    """Encode ARM Thumb-2 32-bit BL instruction (little-endian two halfwords)."""
    imm = dst_addr - (src_addr + 4)
    if imm % 2 != 0 or not (-(1 << 24) <= imm < (1 << 24)):
        raise ValueError(f"BL target out of range: 0x{src_addr:08X} -> 0x{dst_addr:08X}")
    s = (imm >> 24) & 1
    i1 = (imm >> 23) & 1
    i2 = (imm >> 22) & 1
    imm10 = (imm >> 12) & 0x3FF
    imm11 = (imm >> 1) & 0x7FF
    j1 = (~(i1 ^ s)) & 1
    j2 = (~(i2 ^ s)) & 1
    hw1 = 0xF000 | (s << 10) | imm10
    hw2 = 0xD000 | (j1 << 13) | (1 << 12) | (j2 << 11) | imm11
    return struct.pack("<HH", hw1, hw2)


def assemble_trampoline(asm_path, cave_addr):
    """Assemble trampoline_dng_lock.s at cave_addr and return (bytes, sym_dict)."""
    with tempfile.TemporaryDirectory() as tmp:
        obj = os.path.join(tmp, "tramp.o")
        elf = os.path.join(tmp, "tramp.elf")
        raw = os.path.join(tmp, "tramp.bin")
        subprocess.run(
            ["arm-none-eabi-as", "-mcpu=cortex-m3", "-mthumb", asm_path, "-o", obj],
            check=True,
        )
        subprocess.run(
            ["arm-none-eabi-ld", "-Ttext", hex(cave_addr), obj, "-o", elf],
            check=True,
        )
        subprocess.run(
            ["arm-none-eabi-objcopy", "-O", "binary", elf, raw],
            check=True,
        )
        nm_out = subprocess.check_output(["arm-none-eabi-nm", elf], text=True)
        syms = {}
        for line in nm_out.splitlines():
            parts = line.split()
            if len(parts) == 3:
                syms[parts[2]] = int(parts[0], 16)
        with open(raw, "rb") as f:
            cave_bytes = f.read()
    return cave_bytes, syms


def patch_bytes(buf, base, addr, expected, replacement, desc):
    off = addr - base
    actual = bytes(buf[off : off + len(expected)])
    if actual != expected:
        raise RuntimeError(
            f"Mismatch at 0x{addr:08X} ({desc}): expected {expected.hex(' ')}, got {actual.hex(' ')}"
        )
    buf[off : off + len(replacement)] = replacement
    print(f"[PATCH] 0x{addr:08X} ({desc}): {expected.hex(' ')} -> {replacement.hex(' ')}")


def main():
    ap = argparse.ArgumentParser(description="Patch C3M1 factory hex to fix the DNG tear race (and optionally L/R mirror).")
    ap.add_argument("--in", dest="in_hex", default=DEFAULT_IN, help="Input factory .hex")
    ap.add_argument("--out", dest="out_hex", default=None, help="Output patched .hex")
    ap.add_argument(
        "--fix-mirror",
        action="store_true",
        help="Also apply Step-2 Method-2 mirror fix (COM3=0xD0, Orientation=1, CFAPattern)",
    )
    ap.add_argument(
        "--cfa",
        choices=sorted(CFA_PATTERNS),
        default="grbg",
        help="Bayer CFAPattern when --fix-mirror is set (default: grbg)",
    )
    args = ap.parse_args()

    if args.out_hex is None:
        args.out_hex = (
            os.path.join(SCRIPT_DIR, "C3M1_patched_dng_tear_orient.hex")
            if args.fix_mirror
            else DEFAULT_OUT
        )

    base, buf, start_addr = parse_ihex(args.in_hex)
    assert base == FLASH_BASE, f"Unexpected base 0x{base:08X}"
    orig_end = base + len(buf)
    orig_sha = hashlib.sha256(buf).hexdigest()[:16]
    print(f"Loaded {os.path.basename(args.in_hex)}: 0x{base:08X}..0x{orig_end:08X} ({len(buf)} bytes, sha256={orig_sha}...)")
    assert orig_end <= CAVE_ADDR, f"Image end 0x{orig_end:08X} overlaps cave 0x{CAVE_ADDR:08X}"

    # 1. Assemble frame-latch-first trampoline
    cave_bytes, syms = assemble_trampoline(TRAMPOLINE_S, CAVE_ADDR)
    tramp_addr = syms["dng_lock_trampoline"]
    print(f"Assembled trampoline ({len(cave_bytes)} bytes): dng_lock_trampoline=0x{tramp_addr:08X}")

    # 2. Single 4-byte patch: redirect `bl camera_save_dng` at 0x080016B6
    patch_bytes(
        buf, base, 0x080016B6,
        expected=bytes.fromhex("ff f7 6f fa"),  # bl 0x08000B98
        replacement=encode_thumb2_bl(0x080016B6, tramp_addr),
        desc=f"main shoot -> dng_lock_trampoline (0x{tramp_addr:08X})",
    )

    # 3. Optional Step-2 fix: hardware upright (COM3=0xD0) + Orientation=1 + CFAPattern
    if args.fix_mirror:
        patch_bytes(
            buf, base, 0x08002632,
            expected=bytes.fromhex("50 21"),       # movs r1, #0x50 (COM3: HMIRROR only)
            replacement=bytes.fromhex("d0 21"),    # movs r1, #0xD0 (COM3: VFLIP+HMIRROR)
            desc="OV7725 RAW init: COM3 0x50 -> 0xD0 (VFLIP=1, HMIRROR=1)",
        )
        patch_bytes(
            buf, base, 0x080080C6,
            expected=bytes.fromhex("03 00 00 00"), # Tag 0x0112 Orientation = 3 (180 deg)
            replacement=bytes.fromhex("01 00 00 00"), # Orientation = 1 (Normal)
            desc="DNG header +0x066: Orientation 3 -> 1",
        )
        cfa_bytes = CFA_PATTERNS[args.cfa]
        patch_bytes(
            buf, base, 0x0800811A,
            expected=bytes.fromhex("02 01 01 00"), # Tag 0x828E CFAPattern = BGGR
            replacement=cfa_bytes,
            desc=f"DNG header +0x0BA: CFAPattern BGGR -> {args.cfa.upper()}",
        )

    # 4. Append 0xFF padding up to CAVE_ADDR and place trampoline
    buf.extend(b"\xFF" * (CAVE_ADDR - (base + len(buf))))
    buf.extend(cave_bytes)
    print(f"[INJECT] 0x{CAVE_ADDR:08X}..0x{base+len(buf):08X} ({len(cave_bytes)} bytes trampoline)")

    # 5. Write patched HEX and BIN
    write_ihex(args.out_hex, base, buf, start_addr=start_addr)
    out_bin = os.path.splitext(args.out_hex)[0] + ".bin"
    with open(out_bin, "wb") as f:
        f.write(buf)
    print(f"Wrote: {args.out_hex} ({os.path.getsize(args.out_hex)} bytes)")
    print(f"Wrote: {out_bin} ({len(buf)} bytes)")


if __name__ == "__main__":
    main()
