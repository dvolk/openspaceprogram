#!/usr/bin/env python3
"""Bake a skybox from a STAR CATALOG (staging/skybox/data/bsc5.dat, the Yale
Bright Star Catalogue 5th ed., CDS V/50) instead of from imagery.

Why try it: an image bake inherits the source's brightness scale, which is
how the committed placeholder faces ended up with stars saturating at 255 --
as bright per pixel as the star itself (the reason for the runtime sky-dim
fade). A catalog bake authors the scale directly: every star's flux comes
from its V magnitude, so the brightest star in the sky is a number we choose
(--bright), and every other star sits where its magnitude puts it.

What it costs: BSC5 stops at V=6.5 (9110 entries, the naked-eye sky), so
there is no Milky Way band and no telescopic haze. The band is what orients
the ship (see make_skybox.py), so a bare catalog sky is too empty -- hence
--band, which composites the catalog stars over a diffuse layer.

Modes:
  --stats   parse + report (counts, magnitude/color range, parser pins)
  --cross   are the catalog's coordinates and the rail embedding in
            agreement with an IMAGE bake? Reads the brightest catalog
            stars out of an existing bake's faces. NOT circular: the
            catalog positions come from bsc5, the faces come from NASA.
  --bake    write faces to --out

Cubemap/rail math is imported from skybox_common.py, so this bake and the
image bake cannot drift apart on where a sky direction lives. Bakes land
under staging/skybox/bakes/ -- see staging/skybox/NOTES.md.
"""
import argparse
import math
import os
import sys

import numpy as np

BASE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DATA = os.path.join(BASE, 'staging', 'skybox', 'data')

from skybox_common import (FACES, LUM_W, STARS, load_faces, radec_to_rail,
                           save_faces, spec_uv, verify)

CAT = os.path.join(DATA, 'bsc5.dat')
OUT = os.path.join(BASE, 'staging', 'skybox', 'bakes', 'catalog')

# CDS V/50 byte-by-byte (1-based, inclusive) for the columns a bake needs.
# Positions are J2000 -- the same epoch as the Deep Star Maps layers and the
# obliquity in skybox_common.
C = dict(name=(5, 14), rah=(76, 77), ram=(78, 79), ras=(80, 83),
         dsign=(84, 84), ded=(85, 86), dem=(87, 88), des=(89, 90),
         v=(103, 107), bv=(110, 114), sp=(128, 147))


def parse_bsc5(path=CAT):
    """bsc5.dat -> dict of arrays (ra, dec in degrees; V; B-V; name).
    Rows with a blank V or position are dropped: the catalog keeps its
    numbering gaps (novae, extragalactic objects) with empty fields."""
    rows = {'ra': [], 'dec': [], 'v': [], 'bv': [], 'name': []}
    with open(path) as f:
        for line in f:
            if len(line.rstrip('\n')) < 147:
                continue
            g = lambda k: line[C[k][0] - 1:C[k][1]]
            try:
                rah, ram, ras = int(g('rah')), int(g('ram')), float(g('ras'))
                ded, dem, des = int(g('ded')), int(g('dem')), float(g('des'))
                v = float(g('v'))
            except ValueError:
                continue
            ra = (rah + ram / 60.0 + ras / 3600.0) * 15.0
            dec = (1 if g('dsign') == '+' else -1) * (ded + dem / 60.0 + des / 3600.0)
            try:
                bv = float(g('bv'))
            except ValueError:
                bv = float('nan')
            rows['ra'].append(ra % 360.0)
            rows['dec'].append(dec)
            rows['v'].append(v)
            rows['bv'].append(bv)
            rows['name'].append(g('name').strip())
    out = {k: (np.array(v) if k != 'name' else v) for k, v in rows.items()}
    return out


def bv_to_rgb(bv):
    """B-V -> unit RGB. Ballesteros (2012) colour temperature, then the
    Tanner-Helland blackbody->sRGB fit, normalized so the top channel is 1:
    colour must not carry brightness (flux does)."""
    bv = np.nan_to_num(bv, nan=0.0)
    # 4600, not the 460 that circulates online: at 460 the Sun (B-V 0.656)
    # comes out at 580 K and every star bakes deep red.
    t = 4600.0 * (1.0 / (0.92 * bv + 1.7) + 1.0 / (0.92 * bv + 0.62))
    t = np.clip(t, 1000.0, 40000.0) / 100.0
    r = np.where(t <= 66, 255.0, 329.7 * np.power(np.clip(t - 60, 1e-3, None), -0.1332))
    g = np.where(t <= 66, 99.47 * np.log(np.clip(t, 1e-3, None)) - 161.12,
                 288.12 * np.log(np.clip(t - 60, 1e-3, None)) - 441.67)
    b = np.where(t >= 66, 255.0, np.where(t <= 19, 0.0,
                 138.52 * np.log(np.clip(t - 10, 1e-3, None)) - 305.04))
    rgb = np.clip(np.stack([r, g, b]), 0.0, None)
    return rgb / np.maximum(rgb.max(axis=0), 1e-6)


def splat(faces, cat, sigma, zero):
    """Accumulate every star into the six faces as linear radiance.

    Each star is a stamp of directions around its own, mapped through
    spec_uv() -- so a star near a cube edge lands on BOTH faces it covers
    instead of getting clipped at the border. Stars are point sources
    (seeing ~1 arcmin vs %.3f deg/texel at this face size), so sigma is a
    filtering choice, not a physical size: too small and the shader's
    bilinear cube sampling makes stars flicker as the sky turns."""
    n = faces[FACES[0]].shape[1]
    px = (math.pi / 2) / n                      # rad per texel at face centre
    rad = math.ceil(3.0 * sigma)
    off = np.arange(-rad, rad + 1)
    dx, dy = np.meshgrid(off, off)
    w = np.exp(-(dx * dx + dy * dy) / (2.0 * sigma * sigma))
    w /= w.sum()

    rgb = bv_to_rgb(cat['bv'])                  # (3,N)
    flux = zero * np.power(10.0, -0.4 * cat['v'])
    for i in range(len(cat['v'])):
        d = np.array(radec_to_rail(cat['ra'][i], cat['dec'][i]))
        # Tangent basis at d; pick a stable helper axis.
        h = np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        a = np.cross(d, h); a /= np.linalg.norm(a)
        b = np.cross(d, a)
        for k in range(dx.size):
            u = d + px * dx.ravel()[k] * a + px * dy.ravel()[k] * b
            u /= np.linalg.norm(u)
            face, uu, vv = spec_uv(tuple(u))
            col, row = int(uu * n), int(vv * n)
            if not (0 <= col < n and 0 <= row < n):
                continue
            for c in range(3):
                faces[face][c, row, col] += flux[i] * rgb[c, i] * w.ravel()[k]
    return faces


def tone_map(faces, gain):
    out = {}
    for f in FACES:
        out[f] = 1.0 - np.exp(-gain * faces[f])
    return out


def stats(cat):
    v = cat['v']
    print('rows parsed: %d' % len(v))
    print('V: min %.2f max %.2f   B-V: min %.2f max %.2f (blank B-V: %d)'
          % (v.min(), v.max(), np.nanmin(cat['bv']), np.nanmax(cat['bv']),
             int(np.isnan(cat['bv']).sum())))
    for lo in range(-2, 7):
        c = int(((v >= lo) & (v < lo + 1)).sum())
        print('  V %2d..%2d : %5d' % (lo, lo + 1, c))
    order = np.argsort(v)[:10]
    print('brightest:')
    for i in order:
        print('  %-10s V %5.2f  B-V %5.2f  RA %8.3f Dec %+7.3f'
              % (cat['name'][i][:10], v[i], cat['bv'][i], cat['ra'][i], cat['dec'][i]))
    # Parser pins: the six hardcoded STARS in skybox_common are independent
    # of this file. A column off-by-one lands the wrong star on the name.
    print('parser vs skybox_common.STARS:')
    for nm, ra, dec in STARS:
        d = np.hypot((cat['ra'] - ra) * np.cos(np.radians(dec)), cat['dec'] - dec)
        i = int(np.argmin(d))
        print('  %-10s nearest %s  dpos %.4f deg  V %5.2f  B-V %5.2f'
              % (nm, cat['name'][i][:14] or '(unnamed)', d[i], cat['v'][i], cat['bv'][i]))


def cross(cat, faces, n, top=25):
    """Read the brightest catalog stars out of an IMAGE bake. If the parser
    or the rail embedding were wrong, these land on empty sky."""
    order = np.argsort(cat['v'])[:top]
    offs, lums = [], []
    for i in order:
        face, u, v = spec_uv(radec_to_rail(cat['ra'][i], cat['dec'][i]))
        col, row = int(u * n), int(v * n)
        r0, c0 = max(0, row - 4), max(0, col - 4)
        patch = faces[face][:, r0:row + 5, c0:col + 5]
        lum = (LUM_W[:, None, None] * patch.astype(np.float64)).sum(0)
        k = np.unravel_index(np.argmax(lum), lum.shape)
        offs.append(math.hypot(k[0] + r0 - row, k[1] + c0 - col))
        lums.append(float(lum[k]))
    print('catalog vs image bake (%d brightest): median offset %.2f texel, '
          'median peak %.1f, faintest peak %.1f'
          % (top, float(np.median(offs)), float(np.median(lums)), min(lums)))
    assert float(np.median(offs)) <= 2.0, 'catalog positions miss the image stars'
    assert float(np.median(lums)) > 60, 'catalog positions land on empty sky'


def band_faces(layer, n, src_dir):
    """The diffuse layer of an image bake (galactic band, bright stars
    suppressed) as linear faces, to sit UNDER catalog stars. This is the
    half of Deep Star Maps a catalog cannot give us."""
    from make_skybox import load_layer, sample_map
    from skybox_common import face_dirs, rail_to_radec
    img = load_layer(src_dir, layer)
    faces = {}
    for f in FACES:
        dx, dy, dz = face_dirs(f, n)
        ra, dec = rail_to_radec(dx, dy, dz)
        faces[f] = np.clip(sample_map(img, ra, dec), 0.0, 1.0).astype(np.float64)
    return faces


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--size', type=int, default=1024)
    ap.add_argument('--out', default=OUT)
    ap.add_argument('--vmax', type=float, default=6.6, help='cut at this V')
    ap.add_argument('--sigma', type=float, default=1.2, help='PSF sigma [texels]')
    ap.add_argument('--bright', type=float, default=200.0,
                    help='8-bit luminance of the brightest star')
    ap.add_argument('--gain', type=float, default=1.0, help='tone-map gain')
    ap.add_argument('--band', default=None,
                    help='composite over this diffuse layer (e.g. milkyway_2020_8k)')
    ap.add_argument('--src-dir', default=DATA)
    ap.add_argument('--stats', action='store_true')
    ap.add_argument('--cross', action='store_true')
    ap.add_argument('--against',
                    default=os.path.join(BASE, 'staging', 'skybox', 'bakes', 'dsm'),
                    help='image bake to cross-check coordinates against')
    ap.add_argument('--bake', action='store_true')
    args = ap.parse_args()

    cat = parse_bsc5()
    keep = cat['v'] <= args.vmax
    cat = {k: (v[keep] if k != 'name' else [n for n, m in zip(cat['name'], keep) if m])
           for k, v in cat.items()}

    if args.stats:
        stats(cat)
    if args.cross:
        faces, n = load_faces(args.against)
        cross(cat, faces, n)
    if args.bake:
        n = args.size
        faces = {f: np.zeros((3, n, n), dtype=np.float64) for f in FACES}
        # Band first, stars added on top in linear space: the milkyway layer
        # already has bright stars suppressed, so adding is not double-count.
        if args.band:
            faces = band_faces(args.band, n, args.src_dir)
        # Scale so the brightest star in the sky reads --bright after the
        # tone map: that number is the whole point of a catalog bake.
        vmin = float(cat['v'].min())
        lin = -math.log(1.0 - min(args.bright, 254.0) / 255.0) / args.gain
        zero = lin * 2.0 * math.pi * args.sigma ** 2 / 10.0 ** (-0.4 * vmin)
        splat(faces, cat, args.sigma, zero)
        faces = tone_map(faces, args.gain)
        save_faces(faces, args.out)
        print('wrote %s (%d faces, %dx%d, %d stars, sigma %.2f, brightest V %.2f -> %.0f)'
              % (args.out, len(FACES), n, n, len(cat['v']), args.sigma, vmin, args.bright))
        report(faces)
        verify({f: (np.clip(faces[f], 0, 1) * 255).astype(np.uint8) for f in FACES},
               n, band=args.band is not None, min_lum=30)
    return 0


def report(faces):
    """Luminance distribution of a bake -- the number that decides whether
    the star field competes with the sun."""
    lum = np.concatenate([(LUM_W[:, None, None] * faces[f]).sum(0).ravel()
                          for f in FACES])
    lum8 = lum * 255.0
    print('  luminance: mean %.2f  p99 %.1f  p99.9 %.1f  p99.99 %.1f  max %.0f  '
          '>128: %.3f%%' % (lum8.mean(), np.percentile(lum8, 99),
                            np.percentile(lum8, 99.9), np.percentile(lum8, 99.99),
                            lum8.max(), 100.0 * (lum8 > 128).mean()))


if __name__ == '__main__':
    sys.exit(main())
