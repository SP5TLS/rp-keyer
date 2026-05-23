#!/usr/bin/env python3
"""Rewrite the UF2 family ID in every block of a UF2 file.

elf2uf2-rs 2.2.0 hardcodes the RP2040 family ID (0xe48bff56), so its output
is rejected by the RP2350 BOOTSEL drive. The default here is RP2350-ARM-S
(0xe48bff59); pass --family to choose a different one. Family IDs are
documented at
https://github.com/raspberrypi/pico-feedback/blob/main/UF2_FAMILY_IDS.md.

UF2 layout (one 512-byte block per page):
    0x00  magic1     0x0A324655  "UF2\\n"
    0x04  magic2     0x9E5D5157
    0x08  flags
    0x0C  targetAddr
    0x10  payloadSize
    0x14  blockNo
    0x18  numBlocks
    0x1C  familyID   <-- rewritten in place
    0x20..0x1FB  payload (≤476 bytes)
    0x1FC  magicEnd  0x0AB16F30
"""
import argparse
import struct
import sys

BLOCK = 512
FAMILY_OFFSET = 28
MAGIC1 = 0x0A324655
MAGIC2 = 0x9E5D5157
MAGIC_END = 0x0AB16F30

# Single source of truth — referenced by both shell callers (release.yml,
# build-web.sh) which now omit the family argument.
RP2350_ARM_S = 0xE48BFF59


def patch(input_path: str, output_path: str, family: int) -> int:
    with open(input_path, "rb") as f:
        data = bytearray(f.read())

    if len(data) % BLOCK != 0 or len(data) == 0:
        print(f"error: {input_path} is not a multiple of {BLOCK} bytes", file=sys.stderr)
        return 1

    for off in range(0, len(data), BLOCK):
        m1, m2 = struct.unpack_from("<II", data, off)
        end = struct.unpack_from("<I", data, off + BLOCK - 4)[0]
        if m1 != MAGIC1 or m2 != MAGIC2 or end != MAGIC_END:
            print(f"error: bad UF2 magic at offset {off:#x}", file=sys.stderr)
            return 1
        struct.pack_into("<I", data, off + FAMILY_OFFSET, family)

    with open(output_path, "wb") as f:
        f.write(data)

    print(f"patched {len(data) // BLOCK} blocks in {output_path} -> family {family:#010x}")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("input", help="Input .uf2 file")
    p.add_argument(
        "-o", "--output",
        help="Output path (defaults to overwriting INPUT in place)",
    )
    p.add_argument(
        "--family",
        type=lambda x: int(x, 0),
        default=RP2350_ARM_S,
        help=f"Family ID (default {RP2350_ARM_S:#010x} = RP2350-ARM-S)",
    )
    # Backwards compat: the previous CLI was `script.py FILE FAMILY_HEX`.
    # If a positional 2nd arg looks like a hex/int constant, accept it.
    args, extra = p.parse_known_args()
    if extra:
        if len(extra) == 1:
            args.family = int(extra[0], 0)
        else:
            p.error(f"unexpected extra args: {extra}")

    return patch(args.input, args.output or args.input, args.family)


if __name__ == "__main__":
    sys.exit(main())
