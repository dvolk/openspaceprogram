#!/usr/bin/env python3
"""otree.py -- reader for Orbiter .tree tile archives (staging/orbiter-data).

Format verified against the Orbiter source in ../references/orbiter (MIT):
  Utils/texpack/texpack.cpp  (TreeTOC, TOCEntry, level/ilat/ilng addressing)
  Src/Orbiter/ZTreeMgr.cpp   (TreeFileHeader, ReadData -> zlib uncompress)
  Src/Orbiter/elevmgr.cpp    (ELEVFILEHEADER, dtype -> int16 grid)
and against the data itself (Moon Surf/Elev read end-to-end).

Container:
  header 48 B: 'TX' + u8 ver + u8 0, u32 hdrsize(48), u32 flags(1 = deflated),
               u32 dataOfs, s64 dataLength, u32 ntoc,
               u32 rootPos1, rootPos2, rootPos3, rootPos4[2]   (index into the TOC;
               level 1,2,3 = one whole-globe tile each, level 4 = the quadtree roots)
  TOC    ntoc x 32 B at offset 48: {s64 pos, u32 size, s32 child[4]}
               pos   = offset of the DEFLATED payload, relative to dataOfs
               size  = INFLATED size; the deflated length is NOT stored, so feed the
                       decompressor a generous window and stop at the stream end
  data   one zlib stream per node

Tile payload: 'DDS ' + 124 B DDS header (dwSize, dwFlags=0x81007, height, width,
  linearsize, ..., pixel-format fourcc 'DXT1'/'DXT3'/'DXT5') + raster at +128.
Elevation payload: ELEVFILEHEADER (100 B): 'ELE\\1', hdrsize, dtype (-16 = int16,
  0 = flat), xgrd, ygrd, xpad, ypad, then 9 doubles: scale, offset, latmin, latmax,
  lngmin, lngmax, emin, emax, emean -- elevation = raw*scale + offset [m], and the
  tile carries its OWN lat/lng box in radians, which is the easy way to re-bake.

Addressing (texpack.cpp/tileblock.h): equirect, level L >= 4 has nLng = 2^(L-3)
columns x nLat = 2^(L-4) rows of 512 px tiles; child index = (ilat&1)*2 + (ilng&1).

Gotchas learned the hard way, pinned here so they are not re-introduced:
  * BC1 endpoints are RGB565 = 5/6/5 bits. Masking green with &31 (5 bits) tints
    every image green/magenta -- that bug cost a whole "the pixels look broken"
    detour.
  * Do the u16 assembly with uint32 operands: np.uint8 << 8 wraps to 0.
  * dataLength is NOT truncated (it equals the member size minus dataOfs); reading
    it as a u32 is what makes it look wrong.

Usage
  otree.py info   SRC                     header + TOC shape + roots
  otree.py levels SRC                     tile count per level + metres/pixel
  otree.py tile   SRC NODE [PNG]          decode one colour tile (DXT1)
  otree.py elev   SRC NODE [PNG]          one elevation tile: box, range, stats
  otree.py scan   SRC [--sample N]        inflate a sample of tiles, report failures

SRC is either a .tree file or 'zip:path/inside.zip!member'
"""
import math
import struct
import sys
import zipfile
import zlib

import numpy as np

R_EARTH = 6371000.0


class Tree:
    """[random access] One .tree archive, from a file or from inside a zip member."""

    def __init__(self, src):
        if '!' in src:
            zpath, self.member = src.split('!', 1)
            zpath = zpath[4:] if zpath.startswith('zip:') else zpath
            self.zf = zipfile.ZipFile(zpath)
            self.f = self.zf.open(self.member)
            self.size = self.zf.getinfo(self.member).file_size
        else:
            self.zf = None
            self.member = src
            self.f = open(src, 'rb')
            import os
            self.size = os.path.getsize(src)
        self.f.seek(0)
        head = self.f.read(48)
        if head[:2] != b'TX':
            raise ValueError('%s: not a .tree (magic %r)' % (src, head[:2]))
        self.version = head[2]
        self.hdrsize, self.flags, self.dataOfs = struct.unpack_from('<III', head, 4)
        self.dataLength = struct.unpack_from('<q', head, 16)[0]
        self.ntoc, self.root1, self.root2, self.root3, self.root4a, self.root4b = \
            struct.unpack_from('<IIIIII', head, 24)
        self.f.seek(self.hdrsize)
        self.toc = self.f.read(32 * self.ntoc)

    def node(self, i):
        """[(pos, inflated_size, child[4])] TOC entry i (pos is relative to dataOfs)."""
        o = 32 * i
        pos, size = struct.unpack_from('<qI', self.toc, o)
        child = struct.unpack_from('<4i', self.toc, o + 12)
        return pos, size, child

    def roots(self):
        """[(toc index, level)] the roots the header names; -1 entries are skipped."""
        return [(r, L) for r, L in ((self.root1, 1), (self.root2, 2), (self.root3, 3),
                                    (self.root4a, 4), (self.root4b, 4)) if r != 0xFFFFFFFF]

    def levels(self):
        """[{toc index: level}] walking the child links from the named roots."""
        lvl = {}
        stack = list(self.roots())
        while stack:
            i, L = stack.pop()
            if i in lvl:
                continue
            lvl[i] = L
            for c in self.node(i)[2]:
                if c >= 0:
                    stack.append((c, L + 1))
        return lvl

    def payload(self, i, window=4 << 20):
        """[bytes] inflated tile payload of node i; seeks, so cheap for early nodes."""
        pos, size, _ = self.node(i)
        self.f.seek(self.dataOfs + pos)
        win = self.f.read(max(window, size))
        raw = zlib.decompressobj().decompress(win)
        if len(raw) < size:
            raise ValueError('node %d: inflated %d of declared %d' % (i, len(raw), size))
        return raw


def decode_dxt1(buf, w, h):
    """[HxWx3 uint8] BC1/DXT1. Endpoints are RGB565 = 5/6/5 -- green is SIX bits."""
    if w % 4 or h % 4:
        raise ValueError('DXT1 needs a multiple of 4 (%dx%d)' % (w, h))
    B = w // 4
    b = np.frombuffer(buf[:B * B * 8], np.uint8).reshape(B, B, 8)

    def le(i):
        # uint32 operands: np.uint8 << 8 wraps to zero.
        return b[:, :, i].astype(np.uint32) | (b[:, :, i + 1].astype(np.uint32) << 8)

    c0, c1, ix = le(0), le(2), b[:, :, 4:8]

    def unp(c):
        return ((((c >> 11) & 31) * 255 + 15) // 31).astype(float), \
               ((((c >> 5) & 63) * 255 + 31) // 63).astype(float), \
               ((((c & 31)) * 255 + 15) // 31).astype(float)

    wide = (c0 > c1)[..., None]
    pal = np.zeros((B, B, 4, 3))
    pal[:, :, 0] = np.stack(unp(c0), -1)
    pal[:, :, 1] = np.stack(unp(c1), -1)
    pal[:, :, 2] = np.where(wide, (2 * pal[:, :, 0] + pal[:, :, 1]) / 3,
                            (pal[:, :, 0] + pal[:, :, 1]) / 2)
    pal[:, :, 3] = np.where(wide, (pal[:, :, 0] + 2 * pal[:, :, 1]) / 3, 0.0)
    px = np.zeros((B, B, 4, 4, 3))
    for r in range(4):
        for c in range(4):
            sel = (ix[:, :, r] >> (2 * c)) & 3
            px[:, :, r, c] = pal[np.arange(B)[:, None], np.arange(B)[None, :], sel]
    return px.reshape(h, w, 3).astype(np.uint8)


def tile_shape(raw):
    """[(fourcc, w, h, linearsize)] from the 124-byte DDS header after 'DDS '."""
    if raw[:4] != b'DDS ':
        raise ValueError('tile magic %r, expected DDS ' % raw[:4])
    hdrsize, flags, h, w, linearsize = struct.unpack_from('<IIIII', raw, 4)
    # These packs put the pixel format 4 bytes earlier than a stock DDS_HEADER says it
    # should be, so find the fourCC instead of trusting the offset. (Raster at +128 is
    # confirmed independently: it is the only offset where the block grid correlates.)
    fourcc = b''
    for o in range(4, 124):
        if raw[o:o + 4] in (b'DXT1', b'DXT3', b'DXT5'):
            fourcc = raw[o:o + 4]
            break
    return fourcc.decode(errors='replace'), w, h, linearsize, hdrsize


def elev_header(raw):
    """[dict] the 100-byte ELEVFILEHEADER (see Src/Orbiter/elevmgr.cpp)."""
    id4, hdrsize, dtype, xgrd, ygrd, xpad, ypad = struct.unpack_from('<4s6i', raw, 0)
    scale, offset, latmin, latmax, lngmin, lngmax, emin, emax, emean = \
        struct.unpack_from('<9d', raw, 28)
    return dict(id=id4.decode(errors='replace'), hdrsize=hdrsize, dtype=dtype,
                xgrd=xgrd, ygrd=ygrd, xpad=xpad, ypad=ypad, scale=scale, offset=offset,
                latmin=latmin, latmax=latmax, lngmin=lngmin, lngmax=lngmax,
                emin=emin, emax=emax, emean=emean)


def elev_grid(raw, hdr):
    """[float array metres] the int16 grid, scaled and offset."""
    if hdr['dtype'] == 0:
        return np.full(hdr['xgrd'] * hdr['ygrd'], hdr['offset'])
    n = hdr['xgrd'] * hdr['ygrd']
    body = raw[hdr['hdrsize']:]
    if len(body) < n * 2:
        raise ValueError('elev body %d bytes, need %d' % (len(body), n * 2))
    if hdr['dtype'] not in (-16, 16):
        raise ValueError('dtype %d not handled' % hdr['dtype'])
    v = np.frombuffer(body[:n * 2], dtype='<i2' if hdr['dtype'] == -16 else '<u2')
    return v.astype(float) * hdr['scale'] + hdr['offset']


def metres_per_px(level):
    """[m] metres per pixel at the equator for a 512-px tile at this level."""
    px = {1: 128, 2: 256, 3: 512}.get(level)
    if px is None:
        px = 512 * (1 << (level - 3))
    return 2 * math.pi * R_EARTH / px


def save_png(img, path):
    from PIL import Image
    Image.fromarray(img).save(path)


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 2
    cmd, src = argv[1], argv[2]
    t = Tree(src)
    if cmd == 'info':
        print('version %d  flags %d  hdrsize %d  ntoc %d' % (t.version, t.flags, t.hdrsize, t.ntoc))
        print('dataOfs %d  dataLength %.3f GB  member %d  hdr+data check %s' % (
            t.dataOfs, t.dataLength / 1e9, t.size,
            'OK' if t.dataOfs + t.dataLength == t.size else 'MISMATCH'))
        print('roots', t.roots())
        return 0
    if cmd == 'levels':
        lvl = t.levels()
        from collections import Counter
        cnt = Counter(lvl.values())
        unreached = t.ntoc - len(lvl)
        print('levels (%d nodes unreached)' % unreached)
        for L in sorted(cnt):
            print('  level %2d  %7d tiles  %9.1f m/px at equator' % (L, cnt[L], metres_per_px(L)))
        return 0
    if cmd == 'tile':
        i = int(argv[3])
        raw = t.payload(i)
        fourcc, w, h, linearsize, hdrsize = tile_shape(raw)
        print('node %d  %s %dx%d linearsize %d  hdr %d  raster at +128' % (
            i, fourcc, w, h, linearsize, hdrsize))
        img = decode_dxt1(raw[128:], w, h)
        print('  mean %s  std %.1f  |dx| %.2f |dy| %.2f (ratio %.1f = banding)' % (
            img.astype(float).mean(axis=(0, 1)).round(1), img.std(),
            np.abs(np.diff(img.astype(float), axis=1)).mean(),
            np.abs(np.diff(img.astype(float), axis=0)).mean(),
            np.abs(np.diff(img.astype(float), axis=0)).mean() /
            np.abs(np.diff(img.astype(float), axis=1)).mean()))
        if len(argv) > 4:
            save_png(img, argv[4])
            print('  wrote', argv[4])
        return 0
    if cmd == 'elev':
        i = int(argv[3])
        raw = t.payload(i)
        hdr = elev_header(raw)
        print('node %d  %s hdr %d dtype %d grid %dx%d pad %d,%d scale %g offset %g' % (
            i, hdr['id'], hdr['hdrsize'], hdr['dtype'], hdr['xgrd'], hdr['ygrd'],
            hdr['xpad'], hdr['ypad'], hdr['scale'], hdr['offset']))
        print('  box lat [%7.3f, %7.3f] lng [%8.3f, %8.3f] deg   emin %.0f emax %.0f emean %.0f m'
              % (math.degrees(hdr['latmin']), math.degrees(hdr['latmax']),
                 math.degrees(hdr['lngmin']), math.degrees(hdr['lngmax']),
                 hdr['emin'], hdr['emax'], hdr['emean']))
        g = elev_grid(raw, hdr)
        print('  grid metres min %.0f max %.0f mean %.0f' % (g.min(), g.max(), g.mean()))
        if len(argv) > 4:
            w = hdr['xgrd']
            g2 = g.reshape(hdr['ygrd'], w)
            g2 = ((g2 - g2.min()) * 255 // max(1, g2.max() - g2.min())).astype(np.uint8)
            save_png(g2, argv[4])
            print('  wrote', argv[4])
        return 0
    if cmd == 'scan':
        n = int(argv[argv.index('--sample') + 1]) if '--sample' in argv else 40
        idx = np.linspace(0, t.ntoc - 1, min(n, t.ntoc)).astype(int)
        bad = 0
        for i in idx:
            try:
                raw = t.payload(int(i))
                fourcc, w, h, ls, hs = tile_shape(raw)
                if len(raw) < 128 + ls:
                    print('  node %6d short raster: %d < %d' % (i, len(raw), 128 + ls))
                    bad += 1
            except Exception as e:
                print('  node %6d %s' % (i, e))
                bad += 1
        print('sampled %d nodes, %d failed' % (len(idx), bad))
        return 0 if bad == 0 else 1
    print('unknown command %r' % cmd)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv))
