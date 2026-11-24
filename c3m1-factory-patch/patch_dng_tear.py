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
"""

import argparse
import hashlib
import os
import struct
import subprocess
import tempfile

FLASH_BASE = 0x08000000
CAVE_ADDR  = 0x0800E900

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
    ap = argparse.ArgumentParser(description="Patch C3M1 factory hex to fix ONLY the DNG tear race.")
    ap.add_argument("--in", dest="in_hex", default=DEFAULT_IN, help="Input factory .hex")
    ap.add_argument("--out", dest="out_hex", default=DEFAULT_OUT, help="Output patched .hex")
    args = ap.parse_args()

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

    # 3. Append 0xFF padding up to CAVE_ADDR and place trampoline
    buf.extend(b"\xFF" * (CAVE_ADDR - (base + len(buf))))
    buf.extend(cave_bytes)
    print(f"[INJECT] 0x{CAVE_ADDR:08X}..0x{base+len(buf):08X} ({len(cave_bytes)} bytes trampoline)")

    # 4. Write patched HEX and BIN
    write_ihex(args.out_hex, base, buf, start_addr=start_addr)
    out_bin = os.path.splitext(args.out_hex)[0] + ".bin"
    with open(out_bin, "wb") as f:
        f.write(buf)
    print(f"Wrote: {args.out_hex} ({os.path.getsize(args.out_hex)} bytes)")
    print(f"Wrote: {out_bin} ({len(buf)} bytes)")


if __name__ == "__main__":
    main()
