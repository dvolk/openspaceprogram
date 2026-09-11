#!/usr/bin/env python3
"""Generate a solid-fill RGB PNG (a flat part texture) with no dependencies.

Some parts are a single flat colour (the wing is a light gray), so instead of
hand-exporting a texture from an editor this writes a small solid PNG -- the
part's shader just multiplies that fill by its Lambert shading.

    python3 gen_flat_tex.py res/wing.png --rgb 200 200 200 --size 4
"""

import argparse
import struct
import zlib


def png_chunk(tag, data):
    c = struct.pack(">I", len(data)) + tag + data
    return c + struct.pack(">I", zlib.crc32(tag + data) & 0xffffffff)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("dst", help="destination .png")
    ap.add_argument("--rgb", type=int, nargs=3, default=[200, 200, 200],
                    metavar=("R", "G", "B"), help="fill colour 0-255 (default 200 200 200)")
    ap.add_argument("--size", type=int, default=4,
                    help="square pixel size (default 4; the texture is a flat fill)")
    a = ap.parse_args()
    r, g, b = a.rgb
    if any(x < 0 or x > 255 for x in (r, g, b)):
        ap.error("rgb values must be 0-255")
    if a.size < 1:
        ap.error("size must be >= 1")

    w = h = a.size
    row = b"\x00" + bytes([r, g, b]) * w   # filter byte 0 + RGB pixels
    raw = row * h
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)  # 8-bit, colour type 2 (RGB)
    png = (b"\x89PNG\r\n\x1a\n"
           + png_chunk(b"IHDR", ihdr)
           + png_chunk(b"IDAT", zlib.compress(raw, 9))
           + png_chunk(b"IEND", b""))
    with open(a.dst, "wb") as f:
        f.write(png)
    print("%s: %dx%d solid rgb(%d,%d,%d)" % (a.dst, w, h, r, g, b))


if __name__ == "__main__":
    main()
