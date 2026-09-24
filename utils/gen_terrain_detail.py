#!/usr/bin/env python3
"""Generate the terrain detail texture: a seamlessly tiling albedo
modulation map the terrain shader multiplies into the baked vertex
colours, so the flat ground reads as ground at the landed/kerbal scale.

Seamless: every octave is value noise on a TORUS -- the lattice wraps mod
P, so the map tiles in BOTH axes. That is what lets the shader's REPEAT
wrap close the equirectangular antimeridian, and the poles.

Centred on white (255): the shader does albedo = vertexColour *
mix(1, tex, strength), so a flat 255 map is the identity and the noise is
only a darkening modulation of the body's palette colour (it never
re-colours sea or gas-giant bands).

    python3 gen_terrain_detail.py res/textures/terrain_detail.png --size 1024
"""

import argparse
import random
import struct
import zlib


def png_chunk(tag, data):
    c = struct.pack(">I", len(data)) + tag + data
    return c + struct.pack(">I", zlib.crc32(tag + data) & 0xffffffff)


def write_png(dst, w, h, rgb):
    """rgb: w*h*3 bytes, row-major, unfiltered."""
    raw = b"".join(b"\x00" + rgb[y * w * 3:(y + 1) * w * 3] for y in range(h))
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)  # 8-bit, colour type 2 (RGB)
    png = (b"\x89PNG\r\n\x1a\n"
           + png_chunk(b"IHDR", ihdr)
           + png_chunk(b"IDAT", zlib.compress(raw, 9))
           + png_chunk(b"IEND", b""))
    with open(dst, "wb") as f:
        f.write(png)


def octave_noise(N, T, L, rng):
    """One octave of tileable value noise: N*N values in [-1, 1].

    The field has period T pixels (T divides N, so it tiles the N*N image)
    and L random lattice cells per period (the torus resolution; a few
    samples per cell keeps the interpolation smooth). Sampling in lattice
    units u = x*L/T with the index wrapped % L tiles for ANY L: shifting x
    by N adds N*L/T, a whole number of cells. (Wrapping the index as
    (x//T) % L instead would tile only when L divides N/L.)"""
    lat = [rng.uniform(-1.0, 1.0) for _ in range(L * L)]
    # per-pixel lattice coords, precomputed
    ux = [x * L / T for x in range(N)]
    xs = [int(u) % L for u in ux]
    fx = [u - int(u) for u in ux]
    uy = [y * L / T for y in range(N)]
    ys = [int(u) % L for u in uy]
    fy = [u - int(u) for u in uy]
    out = [0.0] * (N * N)
    for y in range(N):
        iy = ys[y]
        gy = 6 * fy[y] ** 5 - 15 * fy[y] ** 4 + 10 * fy[y] ** 3
        row0 = iy * L
        row1 = ((iy + 1) % L) * L
        o = y * N
        for x in range(N):
            ix = xs[x]
            gx = 6 * fx[x] ** 5 - 15 * fx[x] ** 4 + 10 * fx[x] ** 3
            i1 = (ix + 1) % L
            top = lat[row0 + ix] * (1 - gx) + lat[row0 + i1] * gx
            bot = lat[row1 + ix] * (1 - gx) + lat[row1 + i1] * gx
            out[o + x] = top * (1 - gy) + bot * gy
    return out


def lattice_sample(lat, L, T, x, y):
    """Bilinear lattice lookup at an ARBITRARY (possibly warped) pixel
    coordinate; wraps in lattice space, so it tiles for any (T | N, L)."""
    u = (x * L / T) % L
    v = (y * L / T) % L
    ix = int(u)
    iy = int(v)
    fx = u - ix
    fy = v - iy
    gx = 6 * fx ** 5 - 15 * fx ** 4 + 10 * fx ** 3
    gy = 6 * fy ** 5 - 15 * fy ** 4 + 10 * fy ** 3
    i1 = (ix + 1) % L
    j1 = (iy + 1) % L
    top = lat[iy * L + ix] * (1 - gx) + lat[iy * L + i1] * gx
    bot = lat[j1 * L + ix] * (1 - gx) + lat[j1 * L + i1] * gx
    return top * (1 - gy) + bot * gy


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("dst", help="destination .png")
    ap.add_argument("--size", type=int, default=1024,
                    help="square pixel size, a multiple of 128 (default 1024)")
    ap.add_argument("--seed", type=int, default=1337,
                    help="lattice RNG seed (default 1337)")
    ap.add_argument("--warp", type=float, default=1.0,
                    help="domain-warp strength, 0 = off (default 1.0). "
                         "Displaces each octave's sampling coordinates with "
                         "a low-frequency tileable field, breaking the "
                         "regular cellular lattice that reads as 'digital' "
                         "and giving organic ground-like clumps")
    ap.add_argument("--contrast", type=float, default=1.6,
                    help="contrast multiplier applied before re-centring on "
                         "255 (default 1.6; 1.0 = the raw octave sum)")
    a = ap.parse_args()
    if a.size < 128 or a.size % 128 != 0:
        ap.error("--size must be a multiple of 128 (>= 128)")

    n = a.size
    # Ground-like spectrum: fine grain (pebbles) to broad tonal patches.
    # With the default 64 m tile the world periods are 0.25 m .. 16 m.
    # The grain octaves are BELOW screen Nyquist at mid range, so the mip
    # chain averages them (no moire) and they only read up close; the
    # 2-16 m mottle is what carries the ground look at 10-100 m.
    octaves = [
        (n // 128, 4, 10),    # 0.25 m grain
        (n // 64, 8, 14),     # 0.5 m
        (n // 32, 16, 18),    # 1 m
        (n // 16, 32, 22),    # 2 m mottle
        (n // 8, 64, 26),     # 4 m
        (n // 4, 128, 24),    # 8 m broad
        (n // 2, 256, 18),    # 16 m broad patches
    ]

    # Domain warp: a low-frequency tileable displacement field (two
    # independent lattices, one per axis). Period n//8 = 8 m world at the
    # default tile -- large enough to swirl the mottle into organic
    # clumps, small enough to repeat several times per image. Each octave
    # is displaced by a fraction of its own period, so fine grain wiggles
    # and broad patches flow, coherently (the same field drives all).
    wx = wy = None
    if a.warp > 0.0:
        wx = octave_noise(n, n // 8, 8, random.Random(a.seed + 900))
        wy = octave_noise(n, n // 8, 8, random.Random(a.seed + 901))

    img = [0.0] * (n * n)
    for k, (T, L, A) in enumerate(octaves):
        lat = [random.Random(a.seed + k).uniform(-1.0, 1.0) for _ in range(L * L)]
        s = (T / 4.0) * a.warp   # warp displacement, in px (a quarter period)
        if wx is None:
            for y in range(n):
                o = y * n
                for x in range(n):
                    img[o + x] = A * lattice_sample(lat, L, T, x, y)
        else:
            for y in range(n):
                o = y * n
                for x in range(n):
                    i = o + x
                    img[i] = A * lattice_sample(lat, L, T,
                                                x + s * wx[i], y + s * wy[i])

    # Contrast + re-centre on exactly 255: the shader multiplies the map
    # in (albedo *= mix(1, tex, strength)), so 255 is the identity and the
    # map only darkens -- it can never re-colour sea or bands. The random
    # lattices carry a DC bias and the sum's tails are Gaussian-ish, so
    # scale the deviation, re-centre, and clamp the floor above 0 (pure
    # black = a hole in the ground).
    mean = sum(img) / len(img)
    lo = 90
    hi = 255
    for i in range(n * n):
        v = (img[i] - mean) * a.contrast + 255.0
        img[i] = lo if v < lo else (hi if v > hi else v)

    # stats + write (grayscale: r = g = b)
    vmin = min(img)
    rgb = bytearray(3 * n * n)
    j = 0
    for v in img:
        rgb[j] = rgb[j + 1] = rgb[j + 2] = int(round(v))
        j += 3

    write_png(a.dst, n, n, rgb)
    print("%s: %dx%d, seed %d, warp %g, contrast %g (range %d..255)"
          % (a.dst, n, n, a.seed, a.warp, a.contrast, vmin))


if __name__ == "__main__":
    main()
