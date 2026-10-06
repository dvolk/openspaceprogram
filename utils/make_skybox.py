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

The skybox shader samples the cubemap at the ROOT-frame direction (the
.vs passes the raw cube vertex; the draw's view*skyRot cancels to r_root).
So a star at sky direction (RA, Dec) must sit at the rail-embedded
direction of that same sky point:
  rail = (x_ecl, z_ecl, -y_ecl) -- the SAME embedding as the orbital
  rails and the #143 WGCCRE orientations (rail longitude atan2(-z, x) =
  ecliptic longitude, +Y = ecliptic north; #146). A mirrored bake gives a
  mirrored sky; the star pins below are the tripwire.

Face math follows the GL cubemap table (OpenGL 4.6 core Table 8.19,
identical to ES 3.2 Table 8.20): per major axis, s=(sc/|ma|+1)/2,
t=(tc/|ma|+1)/2. face_dirs() bakes with it; spec_uv() re-derives it
independently for the pins, and the seam check catches a wrong sign on
any single face (its four edges would disagree with its neighbours).

EXR decode goes through ffmpeg (PIL can't parse NASA's variant); the
maps are already normalized to [0,1] there.

Source EXRs are NOT committed (130 MB each). Fetch into --src-dir:
  curl -O https://svs.gsfc.nasa.gov/vis/a000000/a004800/a004851/starmap_2020_8k.exr
  curl -O https://svs.gsfc.nasa.gov/vis/a000000/a004800/a004851/milkyway_2020_8k.exr

Usage:
  python3 utils/make_skybox.py                 # bake into tmp/newskybox/
  python3 utils/make_skybox.py --check         # bake + compare against OUT
  python3 utils/make_skybox.py --verify        # pins against OUT's faces,
                                               # no EXRs (runs in test-py)

The sky belongs to the SYSTEM: the loader reads a "skybox" object from the
system JSON (src/system.cpp) naming all six cubemap faces, keyed by the axis
each one is. To try a bake before its look is settled, point a system at the
staged faces:

  "skybox": {
    "+X": "tmp/newskybox/skybox_px.png", "-X": "tmp/newskybox/skybox_nx.png",
    "+Y": "tmp/newskybox/skybox_py.png", "-Y": "tmp/newskybox/skybox_ny.png",
    "+Z": "tmp/newskybox/skybox_pz.png", "-Z": "tmp/newskybox/skybox_nz.png"
  },

The KEYS decide which image is which face, not the file names -- that is why
the field is an object rather than six names in GL order, where a swapped
pair would load happily and give a mirrored sky.

Faces stay out of git until the look is settled; copy them into
res/textures/ (and commit + name them in every res/systems/*.json) when it
is. A named face that does not exist is a load error, so a typo in the
field fails the system load rather than showing a mystery sky. Names
outside "res/" resolve against the CURRENT directory (src/resdir.h), so
run the game from the repo root while trying a staged bake.
"""
import argparse
import math
import os
import subprocess
import sys

import numpy as np
from PIL import Image

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(BASE, 'tmp', 'newskybox')   # staging area; see header
OBLIQ = math.radians(23.4392911)   # J2000 mean obliquity (as make_solar_system)

# Bright stars for the alignment pins: (name, RA_deg, Dec_deg).
STARS = [('Sirius', 101.287, -16.716), ('Canopus', 95.988, -52.696),
         ('Arcturus', 213.915, 19.182), ('Vega', 279.235, 38.784),
         ('Rigel', 78.634, -8.209), ('Betelgeuse', 88.793, 7.407)]
# Galactic center (Sgr A*): the landmark for the band's orientation.
GC_RA, GC_DEC = 266.417, -28.94

FACES = ['px', 'nx', 'py', 'ny', 'pz', 'nz']
LUM_W = np.array([0.2126, 0.7152, 0.0722], dtype=np.float64)

def face_dirs(face, n):
    """Unit directions GL samples at each pixel of one cubemap face
    (row-major, row 0 = first uploaded row), s,t = u,v in [-1,1].
    Literal from OpenGL 4.6 core Table 8.19: +X (1,-t,-s), -X (-1,-t,s),
    +Y (s,1,t), -Y (s,-1,-t), +Z (s,-t,1), -Z (-s,-t,-1)."""
    j = (np.arange(n) + 0.5) / n * 2.0 - 1.0     # s along columns, t along rows
    s, t = np.meshgrid(j, j)
    one = np.ones_like(s)
    if face == 'px':   d = (one, -t, -s)
    elif face == 'nx': d = (-one, -t, s)
    elif face == 'py': d = (s, one, t)
    elif face == 'ny': d = (s, -one, -t)
    elif face == 'pz': d = (s, -t, one)
    elif face == 'nz': d = (-s, -t, -one)
    else: assert False, face
    L = np.sqrt(d[0]**2 + d[1]**2 + d[2]**2)
    return d[0] / L, d[1] / L, d[2] / L

# Table 8.19 again, as (major, sc, tc) signed axis unit vectors -- the
# basis spec_uv() and the seam check are built from, written out
# separately from face_dirs so a transcription slip shows up somewhere.
SPEC = {'px': ((1, 0, 0), (0, 0, -1), (0, -1, 0)),
        'nx': ((-1, 0, 0), (0, 0, 1), (0, -1, 0)),
        'py': ((0, 1, 0), (1, 0, 0), (0, 0, 1)),
        'ny': ((0, -1, 0), (1, 0, 0), (0, 0, -1)),
        'pz': ((0, 0, 1), (1, 0, 0), (0, -1, 0)),
        'nz': ((0, 0, -1), (-1, 0, 0), (0, -1, 0))}

def spec_uv(d):
    """(face, u, v) for a direction, straight from the spec formulas
    s=(sc/|ma|+1)/2, t=(tc/|ma|+1)/2. Independent of face_dirs/face_uv."""
    x, y, z = d
    ax, ay, az = abs(x), abs(y), abs(z)
    if ax >= ay and ax >= az:
        face, ma, sc, tc = ('px', ax, -z, -y) if x > 0 else ('nx', ax, z, -y)
    elif ay >= ax and ay >= az:
        face, ma, sc, tc = ('py', ay, x, z) if y > 0 else ('ny', ay, x, -z)
    else:
        face, ma, sc, tc = ('pz', az, x, -y) if z > 0 else ('nz', az, -x, -y)
    return face, (sc / ma + 1) / 2, (tc / ma + 1) / 2

def rail_to_radec(rx, ry, rz):
    """Rail-frame (root-frame) direction -> RA/Dec degrees. Inverse of the
    rail embedding rail=(x_ecl, z_ecl, -y_ecl), then ecliptic->equatorial
    about X by the obliquity."""
    x_e, y_e, z_e = rx, -rz, ry
    xq = x_e
    yq = y_e * math.cos(OBLIQ) - z_e * math.sin(OBLIQ)
    zq = y_e * math.sin(OBLIQ) + z_e * math.cos(OBLIQ)
    ra = np.degrees(np.arctan2(yq, xq)) % 360.0
    dec = np.degrees(np.arcsin(np.clip(zq, -1.0, 1.0)))
    return ra, dec

def radec_to_rail(ra_deg, dec_deg):
    """RA/Dec (deg) -> unit rail-frame direction (inverse of rail_to_radec)."""
    ra, dec = math.radians(ra_deg), math.radians(dec_deg)
    xq = math.cos(dec) * math.cos(ra)
    yq = math.cos(dec) * math.sin(ra)
    zq = math.sin(dec)
    y_e = yq * math.cos(OBLIQ) + zq * math.sin(OBLIQ)
    z_e = -yq * math.sin(OBLIQ) + zq * math.cos(OBLIQ)
    return (xq, z_e, -y_e)     # rail = (x_ecl, z_ecl, -y_ecl)

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

def verify(faces, n):
    """Pins that only need the faces (uint8 (3,n,n) each): run against a
    fresh bake or against the committed PNGs. All read through spec_uv, so
    they are NOT circular with respect to face_dirs."""
    sizes = {f: faces[f].shape for f in FACES}
    assert all(s == (3, n, n) for s in sizes.values()), \
        'face shape mismatch: %s' % sizes

    # face_dirs and spec_uv are two encodings of Table 8.19; they must agree.
    for face in FACES:
        dx, dy, dz = face_dirs(face, 64)
        d = np.stack([dx.ravel(), dy.ravel(), dz.ravel()])
        f2 = np.array([spec_uv(tuple(col))[0] for col in d.T])
        assert (f2 == face).all(), 'face_dirs/spec_uv disagree on %s' % face

    # ---- star pins: a real star, bright, AT the expected texel ----
    def star_pin(name, ra, dec, halfpix=3):
        face, u, v = spec_uv(radec_to_rail(ra, dec))
        col, row = int(u * n), int(v * n)
        r0, c0 = max(0, row - halfpix), max(0, col - halfpix)
        patch = faces[face][:, r0:row + halfpix + 1, c0:col + halfpix + 1]
        lum = (LUM_W[:, None, None] * patch.astype(np.float64)).sum(0)
        k = np.unravel_index(np.argmax(lum), lum.shape)
        L = float(lum[k])
        off = math.hypot(k[0] + r0 - row, k[1] + c0 - col)
        assert L > 120, 'star pin failed: %s reads %.1f' % (name, L)
        assert off <= 2.0, 'star pin offset: %s peak %.1f texels away' % (name, off)
    for name, ra, dec in STARS:
        star_pin(name, ra, dec)

    # ---- band orientation: baked luminance toward the galactic center
    # must dominate the antipodal cone (the orientation landmark) ----
    gc = radec_to_rail(GC_RA, GC_DEC)
    tot_gc = tot_an = 0.0; cnt_gc = cnt_an = 0
    g = 192
    for face in FACES:
        dx, dy, dz = face_dirs(face, g)      # probe directions on this face
        cos_gc = (dx * gc[0] + dy * gc[1] + dz * gc[2]).ravel()
        dxf, dyf, dzf = dx.ravel(), dy.ravel(), dz.ravel()
        pix = np.empty(g * g, dtype=np.float64)
        for k in range(g * g):
            fs, u, v = spec_uv((dxf[k], dyf[k], dzf[k]))
            c, r = min(int(u * n), n - 1), min(int(v * n), n - 1)
            pix[k] = float(LUM_W @ faces[fs][:, r, c].astype(np.float64))
        m_gc = cos_gc > math.cos(math.radians(25.0))
        m_an = cos_gc < math.cos(math.radians(155.0))
        tot_gc += pix[m_gc].sum(); cnt_gc += int(m_gc.sum())
        tot_an += pix[m_an].sum(); cnt_an += int(m_an.sum())
    gc_l, an_l = tot_gc / cnt_gc, tot_an / cnt_an
    assert gc_l > 2.5 * an_l, 'band pin: gc=%.1f anti=%.1f' % (gc_l, an_l)

    # ---- seam check: each cube edge read from just inside each side must
    # agree. A wrong sign on one face breaks its four edges against its
    # neighbours while every same-face test still round-trips. ----
    eps = 1.0 / n
    ts = (np.arange(96) + 0.5) / 96 * 2.0 - 1.0
    worst = 0.0
    for face in FACES:
        m, a, b = SPEC[face]
        for ab, sgn in [(a, 1), (a, -1), (b, 1), (b, -1)]:
            other = b if ab is a else a
            diffs = []
            for tt in ts:
                # two directions straddling this edge: d1 has face as major
                # axis, d2 has the neighbour's axis take over
                d1 = tuple(m[k] + ab[k] * sgn * (1 - 2 * eps) + other[k] * tt
                           for k in range(3))
                d2 = tuple(m[k] + ab[k] * sgn * (1 + 2 * eps) + other[k] * tt
                           for k in range(3))
                f1, u1, v1 = spec_uv(d1)
                f2, u2, v2 = spec_uv(d2)
                assert f1 == face, 'seam probe left its own face'
                if f2 == face: continue
                L1 = LUM_W @ faces[f1][:, min(int(v1 * n), n - 1),
                                       min(int(u1 * n), n - 1)].astype(np.float64)
                L2 = LUM_W @ faces[f2][:, min(int(v2 * n), n - 1),
                                       min(int(u2 * n), n - 1)].astype(np.float64)
                diffs.append(abs(float(L1) - float(L2)))
            worst = max(worst, float(np.median(diffs)))
    assert worst < 6.0, 'seam check: median edge diff %.1f' % worst
    print('pins OK: 6 stars located; band gc/anti = %.2f; seams < %.1f'
          % (gc_l / an_l, worst))

def load_faces():
    faces = {}
    n = None
    for face in FACES:
        path = os.path.join(OUT, 'skybox_%s.png' % face)
        if not os.path.exists(path):
            sys.exit('missing %s -- run utils/make_skybox.py first' % path)
        with open(path, 'rb') as f:
            a = np.asarray(Image.open(f).convert('RGB'))
        faces[face] = a.transpose(2, 0, 1)      # (3,H,W)
        n = a.shape[0] if n is None else n
    return faces, n

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--size', type=int, default=2048)
    ap.add_argument('--src-dir', default=os.path.join(BASE, 'tmp'))
    ap.add_argument('--gain', type=float, default=3.0)
    ap.add_argument('--ds', type=int, default=4,
                    help='box-downsample of the source map (kills speckle)')
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--verify', action='store_true',
                    help='run the pins against OUT\'s faces (no EXRs)')
    args = ap.parse_args()

    if args.verify:
        faces, n = load_faces()
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
        path = os.path.join(OUT, 'skybox_%s.png' % face)
        if args.check:
            if not os.path.exists(path):
                print('MISSING ' + path); rc = 1; continue
            with open(path, 'rb') as f:
                committed = np.asarray(Image.open(f))
            same = np.array_equal(committed, arr)
            print(('OK      ' if same else 'DRIFT   ') + path)
            rc = rc or (0 if same else 1)
        else:
            os.makedirs(OUT, exist_ok=True)
            Image.fromarray(arr).save(path)
            print('wrote %s (%dx%d)' % (path, n, n))
    if not args.check:
        with open(os.path.join(OUT, 'skybox_ATTRIBUTION.txt'), 'w') as f:
            f.write('skybox_px/nx/py/ny/pz/nz.png are baked by '
                    'utils/make_skybox.py\nfrom NASA\'s Deep Star Maps 2020 '
                    '(https://svs.gsfc.nasa.gov/4851/) --\npublic domain, '
                    'credit requested.\n')
    return rc

if __name__ == '__main__':
    sys.exit(main())
