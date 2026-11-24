#!/usr/bin/env python3
"""
Binary patcher for C3M1 factory firmware (C3M1生产烧录.hex).

Fixes 3 verified SDIO card-writing defects directly in the official binary:
  1. 0x080018F6: Disable SDIO Hardware Flow Control (0x4000 -> 0x0000)
                 Fixes STM32F103 silicon errata ES0340 (SDIOCLK glitch / CRC errors).
  2. 0x08005C52 & 0x08005C76: Redirect BSP_SD_ReadBlocks / BSP_SD_WriteBlocks
                 to a Thumb-2 trampoline at 0x0800E900 that masks NVIC external
                 IRQs (VSYNC/EXTI/TIM6/USART1) during polling transfer while
                 keeping SysTick alive for HAL timeouts. Prevents SDIO TXUNDERR/RXOVERR.
  3. 0x08007468 & 0x0800749C: Increase SD_read / SD_write card-ready busy-wait
                 counter from 100,001 (0x186A1) to 1,000,000 (0xF4240) for slow
                 SD card internal Flash erase cycles.
  4. Optional --clkdiv N at 0x080018FC (default 6 = 9MHz; e.g., 10 = 6MHz).
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
DEFAULT_OUT  = os.path.join(SCRIPT_DIR, "C3M1_patched_sdio.hex")
TRAMPOLINE_S = os.path.join(SCRIPT_DIR, "trampoline.s")


def parse_ihex(path):
    """Parse Intel HEX into (base_addr, bytearray, start_linear_addr)."""
    chunks = {}
    upper = 0
    start_addr = None
    with open(path, "r", encoding="ascii") as f:
        for line in f:
            line = line.strip()
            if not line.startswith(":"):
                continue
            cnt = int(line[1:3], 16)
            addr = int(line[3:7], 16)
            rtype = int(line[7:9], 16)
            payload = bytes.fromhex(line[9 : 9 + cnt * 2])
            if rtype == 0:
                chunks[upper + addr] = payload
            elif rtype == 4:
                upper = int.from_bytes(payload, "big") << 16
            elif rtype == 5:
                start_addr = int.from_bytes(payload, "big")
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
    """Assemble trampoline.s at cave_addr and return (bytes, sym_dict)."""
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
    ap = argparse.ArgumentParser(description="Patch C3M1 factory hex for SDIO reliability.")
    ap.add_argument("--in", dest="in_hex", default=DEFAULT_IN, help="Input factory .hex")
    ap.add_argument("--out", dest="out_hex", default=DEFAULT_OUT, help="Output patched .hex")
    ap.add_argument("--clkdiv", type=int, default=6, help="SDIO ClockDiv (default 6=9MHz, e.g. 10=6MHz)")
    args = ap.parse_args()

    if not (0 <= args.clkdiv <= 255):
        raise ValueError("--clkdiv must be 0..255")

    base, buf, start_addr = parse_ihex(args.in_hex)
    assert base == FLASH_BASE, f"Unexpected base 0x{base:08X}"
    orig_end = base + len(buf)
    orig_sha = hashlib.sha256(buf).hexdigest()[:16]
    print(f"Loaded {os.path.basename(args.in_hex)}: 0x{base:08X}..0x{orig_end:08X} ({len(buf)} bytes, sha256={orig_sha}...)")
    assert orig_end <= CAVE_ADDR, f"Image end 0x{orig_end:08X} overlaps cave 0x{CAVE_ADDR:08X}"

    # 1. Assemble NVIC IRQ guard trampoline
    cave_bytes, syms = assemble_trampoline(TRAMPOLINE_S, CAVE_ADDR)
    rd_tramp = syms["sd_read_trampoline"]
    wr_tramp = syms["sd_write_trampoline"]
    print(f"Assembled trampoline ({len(cave_bytes)} bytes): read=0x{rd_tramp:08X}, write=0x{wr_tramp:08X}")

    # 2. Patch 1: Disable SDIO Hardware Flow Control (STM32F103 ES0340 errata)
    patch_bytes(
        buf, base, 0x080018F6,
        expected=bytes.fromhex("4f f4 80 43"),      # mov.w r3, #0x4000
        replacement=bytes.fromhex("00 23 00 bf"),   # movs r3, #0 ; nop
        desc="MX_SDIO_SD_Init: HardwareFlowControl=DISABLE",
    )

    # Optional ClockDiv adjustment
    if args.clkdiv != 6:
        patch_bytes(
            buf, base, 0x080018FC,
            expected=bytes([6, 0x23]),              # movs r3, #6
            replacement=bytes([args.clkdiv, 0x23]), # movs r3, #clkdiv
            desc=f"MX_SDIO_SD_Init: ClockDiv={args.clkdiv}",
        )

    # 3. Patch 2: Redirect BSP_SD_ReadBlocks / BSP_SD_WriteBlocks to NVIC IRQ guard trampoline
    patch_bytes(
        buf, base, 0x08005C52,
        expected=bytes.fromhex("fe f7 4f fb"),      # bl 0x080042F4
        replacement=encode_thumb2_bl(0x08005C52, rd_tramp),
        desc=f"BSP_SD_ReadBlocks -> sd_read_trampoline (0x{rd_tramp:08X})",
    )
    patch_bytes(
        buf, base, 0x08005C76,
        expected=bytes.fromhex("fe f7 67 fc"),      # bl 0x08004548
        replacement=encode_thumb2_bl(0x08005C76, wr_tramp),
        desc=f"BSP_SD_WriteBlocks -> sd_write_trampoline (0x{wr_tramp:08X})",
    )

    # 4. Patch 3: Increase SD_read / SD_write ready-wait timeout (100,001 -> 1,000,000)
    patch_bytes(
        buf, base, 0x08007468,
        expected=struct.pack("<I", 100001),
        replacement=struct.pack("<I", 1000000),
        desc="SD_read: card-ready wait timeout 100k -> 1M",
    )
    patch_bytes(
        buf, base, 0x0800749C,
        expected=struct.pack("<I", 100001),
        replacement=struct.pack("<I", 1000000),
        desc="SD_write: card-ready wait timeout 100k -> 1M",
    )

    # 5. Append padding (0xFF) up to CAVE_ADDR and place trampoline
    buf.extend(b"\xFF" * (CAVE_ADDR - (base + len(buf))))
    buf.extend(cave_bytes)
    print(f"[INJECT] 0x{CAVE_ADDR:08X}..0x{base+len(buf):08X} ({len(cave_bytes)} bytes trampoline)")

    # 6. Write patched HEX and BIN
    write_ihex(args.out_hex, base, buf, start_addr=start_addr)
    out_bin = os.path.splitext(args.out_hex)[0] + ".bin"
    with open(out_bin, "wb") as f:
        f.write(buf)
    print(f"Wrote: {args.out_hex} ({os.path.getsize(args.out_hex)} bytes)")
    print(f"Wrote: {out_bin} ({len(buf)} bytes)")


if __name__ == "__main__":
    main()
