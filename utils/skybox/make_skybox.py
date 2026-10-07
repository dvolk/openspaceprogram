#!/usr/bin/env python3
"""Bake the skybox cubemap faces from NASA's Deep Star Maps 2020
(https://svs.gsfc.nasa.gov/4851/, PUBLIC DOMAIN).

Two layers composited (both equirectangular, equatorial J2000 layout):
  starmap_2020_8k.exr  -- sharp stars (galaxy band faint)
  milkyway_2020_8k.exr -- diffuse galaxy band (bright stars suppressed)
The band is what orients the ship; the stars are what make it feel real.

Map layout (per the page, verified empirically against the six STARS
below -- "centered at 0h right ascension, r.a. increases to the left",
north up):
  x = ((180 - RA) mod 360) / 360 * W,   y = (90 - Dec) / 180 * H

The cubemap face math, the rail-frame sky embedding and the alignment
pins live in skybox_common.py (shared with any other bake); this file is
only about the Deep Star Maps layers and how they are sampled.

EXR decode goes through ffmpeg (PIL can't parse NASA's variant); the
maps are already normalized to [0,1] there.

Source EXRs are NOT committed (130 MB each). Fetch into --src-dir
(staging/skybox/data/):
  curl -O https://svs.gsfc.nasa.gov/vis/a000000/a004800/a004851/starmap_2020_8k.exr
  curl -O https://svs.gsfc.nasa.gov/vis/a000000/a004800/a004851/milkyway_2020_8k.exr

Usage:
  python3 utils/skybox/make_skybox.py              # bake into staging/skybox/bakes/dsm/
  python3 utils/skybox/make_skybox.py --check      # bake + compare against OUT
  python3 utils/skybox/make_skybox.py --verify     # pins against OUT's faces,
                                                   # no EXRs (runs in test-py)

The sky belongs to the SYSTEM: "skybox" in the system JSON names a DIRECTORY
holding skybox_px/nx/py/ny/pz/nz.png (src/system.cpp builds the six names
itself, in GL cubemap order, so the JSON cannot reorder faces). Committed
sets live in res/skybox/<set>/; a bake here is tried by naming it instead:

  "skybox": "staging/skybox/bakes/dsm_1024_g3_ds2",

which resolves because names outside "res/" are cwd-relative (src/resdir.h)
-- run the game from the repo root while trying a staged bake.

Face identity then rests entirely on the file names, so the alignment pins
in skybox_common.verify() are the tripwire for a mirrored or swapped set:
they locate six stars by RA/Dec (mutation-tested: a mirrored sky reads 3.7
texels off, a +X/-X swap 2.0). Always --verify a set before copying it into
res/skybox/, and note that test-py runs those same pins against the
COMMITTED faces. A directory missing one of the six is a load error, so a
typo fails the system load rather than showing a mystery sky.
"""
import argparse
import math
import os
import subprocess
import sys

import numpy as np
from PIL import Image

from skybox_common import FACES, STARS, face_dirs, load_faces, rail_to_radec, \
    verify

BASE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DATA = os.path.join(BASE, 'staging', 'skybox', 'data')      # source EXRs + caches
OUT = os.path.join(BASE, 'staging', 'skybox', 'bakes', 'dsm')   # see header


def load_layer(src_dir, name):
    """EXR -> float32 RGB planes (3,H,W) via ffmpeg rawvideo."""
    exr = os.path.join(src_dir, name + '.exr')
    if not os.path.exists(exr):
        sys.exit('missing %s -- see the header for the download URL' % exr)
    raw = os.path.join(src_dir, name + '.raw')
    if (not os.path.exists(raw)
            or os.path.getmtime(raw) < os.path.getmtime(exr)):
        subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
                        '-i', exr, '-f', 'rawvideo', '-pix_fmt', 'gbrpf32le',
                        raw], check=True)
    a = np.fromfile(raw, dtype='<f4')
    h = int(math.isqrt(a.size // 6))          # W = 2H equirectangular
    assert 3 * h * 2 * h == a.size, 'unexpected raw size %d' % a.size
    # gbrpf32le plane order is G,B,R (verified vs ffmpeg's packed rgb24);
    # reading them as R,G,B bakes a purple/yellow-green cast.
    return a.reshape(3, h, 2 * h)[[2, 0, 1]]

def sample_map(img, ra, dec):
    """Bilinear sample of the equirectangular map at RA/Dec arrays."""
    _, H, W = img.shape
    x = ((180.0 - ra) % 360.0) / 360.0 * W - 0.5
    y = (90.0 - dec) / 180.0 * H - 0.5
    x0 = np.floor(x).astype(np.int64); y0 = np.floor(y).astype(np.int64)
    fx = x - x0; fy = y - y0
    x0w, x1w = x0 % W, (x0 + 1) % W
    y0c, y1c = np.clip(y0, 0, H - 1), np.clip(y0 + 1, 0, H - 1)
    out = np.empty((3,) + np.shape(ra), dtype=np.float32)
    for c in range(3):
        m = img[c]
        out[c] = ((1 - fx) * (1 - fy) * m[y0c, x0w] + fx * (1 - fy) * m[y0c, x1w]
                  + (1 - fx) * fy * m[y1c, x0w] + fx * fy * m[y1c, x1w])
    return out

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--size', type=int, default=2048)
    ap.add_argument('--src-dir', default=DATA)
    ap.add_argument('--out', default=OUT,
                    help='face directory (a second bake would otherwise '
                         'overwrite the one you are comparing against)')
    ap.add_argument('--gain', type=float, default=3.0)
    ap.add_argument('--ds', type=int, default=4,
                    help='box-downsample of the source map (kills speckle)')
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--verify', action='store_true',
                    help='run the pins against OUT\'s faces (no EXRs)')
    args = ap.parse_args()

    if args.verify:
        faces, n = load_faces(args.out)
        verify(faces, n)
        return 0

    sm = load_layer(args.src_dir, 'starmap_2020_8k')
    mw = load_layer(args.src_dir, 'milkyway_2020_8k')
    assert sm.shape == mw.shape, 'layer resolution mismatch'
    # Screen composite: band underneath, stars on top, no clipping blowout.
    img = 1.0 - (1.0 - sm) * (1.0 - mw)
    del sm, mw

    # Box-downsample before sampling: at face resolution a faint star is a
    # lone speckle (digital-noise look); area-averaging melts those into the
    # background while saturated bright stars and the band survive.
    if args.ds > 1:
        H, W = img.shape[1], img.shape[2]
        assert H % args.ds == 0 and W % args.ds == 0, 'ds must divide the map'
        img = img.reshape(3, H // args.ds, args.ds, W // args.ds, args.ds)
        img = img.mean(axis=(2, 4))

    n = args.size
    faces = {}
    for face in FACES:
        dx, dy, dz = face_dirs(face, n)
        ra, dec = rail_to_radec(dx, dy, dz)
        rgb = sample_map(img, ra, dec)
        rgb = 1.0 - np.exp(-args.gain * rgb)          # tone map
        faces[face] = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)

    # Channel-order tripwire (cores saturate, so compare source wing
    # means): Betelgeuse is red, Rigel is blue.
    def wing_rb(ra, dec, r=10):
        _, H, W = img.shape
        x = int(((180.0 - ra) % 360.0) / 360.0 * W)
        y = int((90.0 - dec) / 180.0 * H)
        xs = (x + np.arange(-r, r + 1)) % W           # wrap at the RA seam
        p = img[:, y - r:y + r + 1, xs]
        return float(p[0].mean()), float(p[2].mean())
    betel = next((ra, dec) for nm, ra, dec in STARS if nm == 'Betelgeuse')
    rigel = next((ra, dec) for nm, ra, dec in STARS if nm == 'Rigel')
    br, bb = wing_rb(*betel)
    rg, rb = wing_rb(*rigel)
    assert br > bb and rb > rg, \
        'channel pin: Betelgeuse R/B %.4f/%.4f, Rigel R/B %.4f/%.4f' % (br, bb, rg, rb)

    verify(faces, n)

    rc = 0
    for face in FACES:
        arr = faces[face].transpose(1, 2, 0)      # (H,W,3)
        path = os.path.join(args.out, 'skybox_%s.png' % face)
        if args.check:
            if not os.path.exists(path):
                print('MISSING ' + path); rc = 1; continue
            with open(path, 'rb') as f:
                committed = np.asarray(Image.open(f))
            same = np.array_equal(committed, arr)
            print(('OK      ' if same else 'DRIFT   ') + path)
            rc = rc or (0 if same else 1)
        else:
            os.makedirs(args.out, exist_ok=True)
            Image.fromarray(arr).save(path)
            print('wrote %s (%dx%d)' % (path, n, n))
    if not args.check:
        with open(os.path.join(args.out, 'skybox_ATTRIBUTION.txt'), 'w') as f:
            f.write('skybox_px/nx/py/ny/pz/nz.png are baked by '
                    'utils/skybox/make_skybox.py\nfrom NASA\'s Deep Star Maps 2020 '
                    '(https://svs.gsfc.nasa.gov/4851/) --\npublic domain, '
                    'credit requested.\n')
        # Provenance travels with the faces: a promoted set in res/skybox/
        # must be reproducible, and --check against it is only meaningful if
        # the knobs that made it are recorded.
        with open(os.path.join(args.out, 'skybox_params.json'), 'w') as f:
            json.dump({'cmd': ' '.join(sys.argv), 'size': n, 'gain': args.gain,
                       'ds': args.ds}, f, sort_keys=True)
            f.write('\n')
    return rc

if __name__ == '__main__':
    sys.exit(main())
