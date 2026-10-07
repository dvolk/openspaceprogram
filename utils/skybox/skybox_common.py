#!/usr/bin/env python3
"""Skybox cubemap machinery shared by every bake (make_skybox.py and any
catalog-driven bake). Nothing here knows where the sky data comes from: it
covers the geometry (which texel is which sky direction), the game's sky
frame convention, and the pins that catch a bake pointing the wrong way.

The skybox shader samples the cubemap at the ROOT-frame direction (the
.vs passes the raw cube vertex; the draw's view*skyRot cancels to r_root).
So a star at sky direction (RA, Dec) must sit at the rail-embedded
direction of that same sky point:
  rail = (x_ecl, z_ecl, -y_ecl) -- the SAME embedding as the orbital
  rails and the #143 WGCCRE orientations (rail longitude atan2(-z, x) =
  ecliptic longitude, +Y = ecliptic north; #146). A mirrored bake gives a
  mirrored sky; the star pins in verify() are the tripwire.

A bake's own coordinates are NOT evidence about that embedding: place
stars with radec_to_rail() and then pin them with the same function and
the check is circular. Cross-check against a bake built from an
INDEPENDENT source (verify() against Deep Star Maps faces, say) before
trusting a synthetic sky's alignment.

Face math follows the GL cubemap table (OpenGL 4.6 core Table 8.19,
identical to ES 3.2 Table 8.20): per major axis, s=(sc/|ma|+1)/2,
t=(tc/|ma|+1)/2. face_dirs() bakes with it; spec_uv() re-derives it
independently for the pins, and the seam check catches a wrong sign on
any single face (its four edges would disagree with its neighbours).
"""
import math
import os
import sys

import numpy as np
from PIL import Image

OBLIQ = math.radians(23.4392911)   # J2000 mean obliquity (as make_solar_system)

# Bright stars for the alignment pins: (name, RA_deg, Dec_deg). J2000.
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


def face_uv(ra, dec, n):
    """(face, column, row) of the texel covering a sky point, at face size n.
    Goes through spec_uv so a bake that splats with this and a pin that
    reads with it cannot disagree about where a star lives."""
    face, u, v = spec_uv(radec_to_rail(ra, dec))
    return face, u * n, v * n


def verify(faces, n, stars=STARS, band=True, min_lum=120):
    """Pins that only need the faces (uint8 (3,n,n) each): run against a
    fresh bake or against the committed PNGs. All read through spec_uv, so
    they are NOT circular with respect to face_dirs.

    band=False skips the galactic-band orientation pin: a sky with no
    Milky Way in it (a bare star catalog, say) has nothing to measure.

    min_lum is the floor a pinned star must reach. 120 suits an image bake,
    where the bright stars saturate; a catalog bake deliberately puts
    magnitude into the pixel values, so its faintest pinned stars are much
    dimmer and the caller lowers this instead of the bake brightening them."""
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
        L = float(lum.max())
        assert L > min_lum, 'star pin failed: %s reads %.1f' % (name, L)
        # WHERE the star is = the luminance centroid of its core, not the
        # argmax texel. A box-downsampled star is a blob and its brightest
        # pixel hops around (2.8 texels at 2048 with --ds 4) while its
        # centre stays under half a texel off; the argmax test was measuring
        # which pixel wins, not where the star is. The floor keeps a
        # neighbour or the band from dragging the centroid.
        w = np.clip(lum - 0.25 * L, 0.0, None)
        rr, cc = np.mgrid[r0:r0 + lum.shape[0], c0:c0 + lum.shape[1]]
        off = math.hypot((w * rr).sum() / w.sum() - row,
                         (w * cc).sum() / w.sum() - col)
        assert off <= 1.5, 'star pin offset: %s centroid %.2f texels off' % (name, off)
    for name, ra, dec in stars:
        star_pin(name, ra, dec)

    # ---- band orientation: baked luminance toward the galactic center
    # must dominate the antipodal cone (the orientation landmark) ----
    gc_l = an_l = 0.0
    if band:
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
    print('pins OK: %d stars located; band gc/anti = %.2f; seams < %.1f'
          % (len(stars), gc_l / an_l if band else 0.0, worst))


def load_faces(dir):
    """The six faces in a skybox directory as uint8 (3,n,n) each -- the same
    layout the game's loader builds (src/system.cpp: <dir>/skybox_<axis>.png)."""
    faces = {}
    n = None
    for face in FACES:
        path = os.path.join(dir, 'skybox_%s.png' % face)
        if not os.path.exists(path):
            sys.exit('missing %s -- run utils/skybox/make_skybox.py first' % path)
        with open(path, 'rb') as f:
            a = np.asarray(Image.open(f).convert('RGB'))
        faces[face] = a.transpose(2, 0, 1)      # (3,H,W)
        n = a.shape[0] if n is None else n
    return faces, n


def save_faces(faces, dir):
    """Write the six faces as PNGs (float [0,1] (3,n,n) in, file out)."""
    os.makedirs(dir, exist_ok=True)
    for face in FACES:
        arr = (np.clip(faces[face], 0, 1) * 255).astype(np.uint8).transpose(1, 2, 0)
        Image.fromarray(arr).save(os.path.join(dir, 'skybox_%s.png' % face))
