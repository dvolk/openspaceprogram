#!/usr/bin/env python3
"""Bake Earth-like elevation from NASA BMNG/GEBCO 2008 topography + bathymetry.

Source (PUBLIC DOMAIN / GEBCO credit requested; see heightmap_ATTRIBUTION.txt):
  staging/heightmaps/earth/data/gebco_08_rev_elev_5400x2700.tif  land,  0..~252 -> 0..6400 m
  staging/heightmaps/earth/data/gebco_08_rev_bath_5400x2700.tif  ocean, 255 = 0 m, 0 = -8000 m
Both 8-bit greyscale equirectangular, lon -180 left, north up. Use the
GeoTIFFs (never the JPEG twins: lossy coastlines).

Merge -> signed metres vs sea level -> downsample -> int16 LE. Game layout
matches src/surfmap.h (lon 0 at the LEFT edge, north up), so the bake rolls
the NASA half-turn (their lon 0 is the image centre).

Output (default res/heightmaps/earth/ -- the committed game asset):
  earth_hm.i16           self-describing heightmap (see HM16 below)
  heightmap_params.json  bake provenance
  heightmap_ATTRIBUTION.txt

The grey preview PNG goes to staging/heightmaps/earth/preview/ (gitignored).

HM16 container (little-endian):
  0   4s   magic "HM16"
  4   u32  width   (columns, longitude)
  8   u32  height  (rows, latitude; row 0 = north)
  12  u32  format  (1 = int16 metres vs sea level)
  16  i16  samples[width * height]  row-major

Usage:
  python3 utils/heightmaps/gen_earth_hm.py
  python3 utils/heightmaps/gen_earth_hm.py --size 5400 2700   # full-res bake
  python3 utils/heightmaps/gen_earth_hm.py --verify           # pin known elevations
  python3 utils/heightmaps/gen_earth_hm.py --check            # re-bake in memory, compare
"""
import argparse
import json
import os
import struct
import sys

import numpy as np
from PIL import Image

BASE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DATA = os.path.join(BASE, 'staging', 'heightmaps', 'earth', 'data')
OUT = os.path.join(BASE, 'res', 'heightmaps', 'earth')
PREVIEW = os.path.join(BASE, 'staging', 'heightmaps', 'earth', 'preview')

ELEV_TIF = 'gebco_08_rev_elev_5400x2700.tif'
BATH_TIF = 'gebco_08_rev_bath_5400x2700.tif'
ELEV_SCALE = 6400.0 / 255.0     # land:  pixel 0..255 -> 0..6400 m
BATH_SCALE = 8000.0 / 255.0     # ocean: pixel 255 = 0 m, 0 = -8000 m

MAGIC = b'HM16'
FORMAT_METRES_I16 = 1

# Direction pins (game lon/lat, degrees -> metres). Tolerances are coarse on
# purpose: GEBCO 8 km/px + the 0..6400 land scale is "vaguely Earth", not a
# survey DEM. Values measured from a full-res merge of these two TIFFs.
PINS = [
    # lon,   lat,  name,              lo,     hi
    ( 87.0,  28.0, 'Himalaya',       4500.0, 6500.0),
    (  0.0,   0.0, 'Gulf of Guinea', -6000.0, -1000.0),
    (-150.0,   0.0, 'Central Pacific', -6000.0, -2500.0),
    (145.0, -30.0, 'Australia',         -50.0,  400.0),
    (-100.0,  40.0, 'US mid',           100.0, 1500.0),
]


def load_merged(src_dir):
    """GeoTIFF pair -> float32 metres, NASA layout (lon -180 left, north up)."""
    elev_p = os.path.join(src_dir, ELEV_TIF)
    bath_p = os.path.join(src_dir, BATH_TIF)
    for p in (elev_p, bath_p):
        if not os.path.exists(p):
            sys.exit('missing %s -- see the header for the download URL' % p)
    elev = np.asarray(Image.open(elev_p), dtype=np.float32)
    bath = np.asarray(Image.open(bath_p), dtype=np.float32)
    if elev.shape != bath.shape or elev.ndim != 2:
        sys.exit('elev/bath shape mismatch: %s vs %s' % (elev.shape, bath.shape))
    # elev > 0 = land (bath is 255 there); else seafloor from bath.
    land = elev * ELEV_SCALE
    sea = (bath - 255.0) * BATH_SCALE
    return np.where(elev > 0.0, land, sea)


def to_game_layout(m):
    """NASA (lon -180 left) -> game (lon 0 left, see src/surfmap.h)."""
    return np.roll(m, m.shape[1] // 2, axis=1)


def downsample(m, w, h):
    """Box-filter to w x h (exact when the ratio is an integer)."""
    mh, mw = m.shape
    if (mw, mh) == (w, h):
        return m
    if mw % w == 0 and mh % h == 0:
        ry, rx = mh // h, mw // w
        return m.reshape(h, ry, w, rx).mean(axis=(1, 3))
    return np.asarray(
        Image.fromarray(m).resize((w, h), Image.Resampling.BOX),
        dtype=np.float32)


def write_hm16(path, m):
    """m: float32 metres, game layout, row 0 = north -> HM16 + rounded i16."""
    h, w = m.shape
    q = np.clip(np.rint(m), -32768, 32767).astype('<i2')
    with open(path, 'wb') as f:
        f.write(MAGIC)
        f.write(struct.pack('<III', w, h, FORMAT_METRES_I16))
        f.write(q.tobytes())


def read_hm16(path):
    with open(path, 'rb') as f:
        raw = f.read()
    if raw[:4] != MAGIC:
        sys.exit('%s: not an HM16 heightmap' % path)
    w, h, fmt = struct.unpack_from('<III', raw, 4)
    if fmt != FORMAT_METRES_I16:
        sys.exit('%s: unknown format %d' % (path, fmt))
    q = np.frombuffer(raw, dtype='<i2', count=w * h, offset=16)
    return q.reshape(h, w).astype(np.float32)


def sample_dir(m, lon_deg, lat_deg):
    """Bilinear, game layout (matches Heightmap::sample in src/terragen.h)."""
    h, w = m.shape
    u = (lon_deg % 360.0) / 360.0 * w
    v = (90.0 - lat_deg) / 180.0 * (h - 1)
    v = min(max(v, 0.0), h - 1.000001)
    i0 = int(u) % w
    j0 = min(int(v), h - 2)
    fu = u - int(u)
    fv = v - j0
    i1 = (i0 + 1) % w
    a = m[j0, i0] * (1 - fu) + m[j0, i1] * fu
    b = m[j0 + 1, i0] * (1 - fu) + m[j0 + 1, i1] * fu
    return float(a * (1 - fv) + b * fv)


def write_preview(path, m):
    """Grey ramp: 0 m = mid grey, land up, sea down (clipped +-8000)."""
    x = np.clip(m / 8000.0, -1.0, 1.0)
    px = (128.0 + 127.0 * x).astype(np.uint8)
    Image.fromarray(px, mode='L').save(path)


def write_meta(out_dir, src_dir, w, h, m):
    meta = {
        'source': [
            'staging/heightmaps/earth/data/' + ELEV_TIF,
            'staging/heightmaps/earth/data/' + BATH_TIF,
        ],
        'elev_scale_m_per_count': ELEV_SCALE,
        'bath_scale_m_per_count': BATH_SCALE,
        'layout': 'game equirect: lon 0 left, north up (src/surfmap.h)',
        'width': w,
        'height': h,
        'format': 'HM16 int16 metres vs sea level',
        'min_m': float(m.min()),
        'max_m': float(m.max()),
    }
    with open(os.path.join(out_dir, 'heightmap_params.json'), 'w') as f:
        json.dump(meta, f, indent=2)
        f.write('\n')
    with open(os.path.join(out_dir, 'heightmap_ATTRIBUTION.txt'), 'w') as f:
        f.write(
            'earth_hm.i16 is baked by utils/heightmaps/gen_earth_hm.py from\n'
            'NASA Earth Observatory Blue Marble: Next Generation topography +\n'
            'bathymetry (GEBCO 2008, British Oceanographic Data Centre):\n'
            '  https://science.nasa.gov/earth/earth-observatory/'
            'blue-marble-next-generation/\n'
            'Credit: Jesse Allen, NASA Earth Observatory, using data from\n'
            'GEBCO produced by the British Oceanographic Data Centre.\n'
        )


def bake(src_dir, w, h):
    m = to_game_layout(load_merged(src_dir))
    m = downsample(m, w, h)
    return m


def verify(path):
    m = read_hm16(path)
    fails = 0
    for lon, lat, name, lo, hi in PINS:
        v = sample_dir(m, lon, lat)
        ok = lo <= v <= hi
        print('%s  %-16s %8.1f m  [%g, %g]  %s'
              % ('ok  ' if ok else 'FAIL', name, v, lo, hi, ''))
        if not ok:
            fails += 1
    if fails:
        sys.exit('gen_earth_hm: %d pin(s) failed' % fails)
    print('gen_earth_hm: %d pin(s) ok' % len(PINS))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--src-dir', default=DATA,
                    help='directory holding the two GeoTIFFs (default %(default)s)')
    ap.add_argument('--out', default=OUT,
                    help='bake directory (default %(default)s)')
    ap.add_argument('--size', nargs=2, type=int, default=(2700, 1350),
                    metavar=('W', 'H'), help='output resolution (default 2700 1350)')
    ap.add_argument('--verify', action='store_true',
                    help='pin known elevations in --out/earth_hm.i16 and exit')
    ap.add_argument('--check', action='store_true',
                    help='re-bake in memory and compare to --out/earth_hm.i16')
    args = ap.parse_args()

    hm_path = os.path.join(args.out, 'earth_hm.i16')
    if args.verify:
        if not os.path.exists(hm_path):
            sys.exit('missing %s -- bake first' % hm_path)
        verify(hm_path)
        return

    w, h = args.size
    m = bake(args.src_dir, w, h)

    if args.check:
        if not os.path.exists(hm_path):
            sys.exit('missing %s -- bake first' % hm_path)
        old = read_hm16(hm_path)
        if old.shape != m.shape or not np.array_equal(
                old, np.clip(np.rint(m), -32768, 32767).astype(np.float32)):
            sys.exit('gen_earth_hm: %s drifted from a fresh bake' % hm_path)
        print('gen_earth_hm: %s matches a fresh bake' % hm_path)
        return

    os.makedirs(args.out, exist_ok=True)
    os.makedirs(PREVIEW, exist_ok=True)
    write_hm16(hm_path, m)
    prev = os.path.join(PREVIEW, 'earth_hm_preview.png')
    write_preview(prev, m)
    write_meta(args.out, args.src_dir, w, h, m)
    print('wrote %s  %dx%d  m=[%.0f, %.0f]' % (hm_path, w, h, m.min(), m.max()))
    print('wrote %s' % prev)
    verify(hm_path)


if __name__ == '__main__':
    main()
