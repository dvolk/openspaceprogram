#!/usr/bin/env python3
"""Decode Orbiter .tree tiles to PNG (numpy BC1/BC2 decoder)."""
import struct, zlib, sys
import numpy as np


def _565(c):
    r = (c >> 11) & 31
    g = (c >> 5) & 31
    b = c & 31
    return np.stack([(r * 255 // 31), (g * 255 // 31), (b * 255 // 31)], axis=-1)


def decode_dxt(buf, w, h, alpha=False):
    """[HxWx3 uint8] BC1 (4 B/block colour + 4 B indices) or, with alpha, BC2-style:
    Orbiter's cloud tiles are 1 B/px, taken here as DXT3 (8 B alpha + 8 B colour)."""
    bw, bh = w // 4, h // 4
    stride = 16 if alpha else 8
    blocks = np.frombuffer(buf[:bw * bh * stride], dtype=np.uint8).reshape(bh, bw, stride)
    end = 8 if alpha else 0
    c0 = blocks[:, :, end:end + 2].view(np.uint16)[..., 0].astype(np.uint32)
    c1 = blocks[:, :, end + 2:end + 4].view(np.uint16)[..., 0].astype(np.uint32)
    idx = blocks[:, :, end + 4:end + 8].astype(np.uint32)
    def unpack(c):
        r = (c >> 11) & 31
        g = (c >> 5) & 31
        b = c & 31
        return (r * 255 // 31).astype(np.float64), (g * 255 // 31).astype(np.float64), (b * 255 // 31).astype(np.float64)
    r0, g0, b0 = unpack(c0)
    r1, g1, b1 = unpack(c1)
    use3 = (c0 > c1)[..., None]
    pal = np.zeros((bh, bw, 4, 3))
    pal[:, :, 0] = np.stack([r0, g0, b0], -1)
    pal[:, :, 1] = np.stack([r1, g1, b1], -1)
    pal[:, :, 2] = np.where(use3, (2 * pal[:, :, 0] + pal[:, :, 1]) / 3, (pal[:, :, 0] + pal[:, :, 1]) / 2)
    pal[:, :, 3] = np.where(use3, (pal[:, :, 0] + 2 * pal[:, :, 1]) / 3, 0.0)
    px = np.zeros((bh, bw, 4, 4, 3))
    for row in range(4):
        for col in range(4):
            sel = (idx[:, :, row] >> (2 * col)) & 3
            px[:, :, row, col] = pal[np.arange(bh)[:, None], np.arange(bw)[None, :], sel]
    img = px.reshape(bh * 4, bw * 4, 3)
    if alpha:
        ab = blocks[:, :, 0:8].astype(np.uint32)   # DXT3: 4 bits per texel, 2 texels per byte
        a = np.zeros((bh, bw, 4, 4))
        for row in range(4):
            for col in range(4):
                byte = ab[:, :, row]
                nib = np.where(col % 2 == 0, byte & 15, (byte >> 4) & 15)
                a[:, :, row, col] = nib * 17
        amap = a.reshape(bh * 4, bw * 4, 1)
        img = (img * (amap / 255.0)).astype(np.uint8)
    return img.astype(np.uint8)


def open_tree(path):
    d = open(path, 'rb').read()
    ver = struct.unpack_from('<H', d, 2)[0]
    idxsz = struct.unpack_from('<I', d, 12)[0]
    datasz = struct.unpack_from('<I', d, 16)[0]
    nrec = struct.unpack_from('<I', d, 24)[0]
    nodes = []
    for i in range(nrec):
        o = 48 + 32*i
        off, sz = struct.unpack_from('<QI', d, o)
        ch = struct.unpack_from('<4i', d, o+12)
        key = struct.unpack_from('<I', d, o+28)[0]
        nodes.append(dict(i=i, off=off, size=sz, child=ch, key=key))
    return dict(ver=ver, idxsz=idxsz, datasz=datasz, nrec=nrec, nodes=nodes, data=d)


def tile(a, i):
    n = a['nodes'][i]
    raw = zlib.decompress(a['data'][a['idxsz'] + n['off']:])
    magic = struct.unpack_from('<I', raw, 0)[0]
    if magic == 0x01454945:          # 'EIE' -- an elevation tile, own 100 B header
        w, h = struct.unpack_from('<II', raw, 12)
        return dict(kind='elev', w=w, h=h, hdr=raw[:100], body=raw[100:])
    w, h, nbytes, fmt = struct.unpack_from('<IIII', raw, 12), None, None, None
    w, h = w[0], w[1]
    nbytes = struct.unpack_from('<I', raw, 20)[0]
    fmt = struct.unpack_from('<I', raw, 80)[0]
    return dict(kind='img', w=w, h=h, fmt=fmt, hdr=raw[:128], body=raw[128:])


if __name__ == '__main__':
    from PIL import Image
    path, outdir = sys.argv[1], sys.argv[2]
    import os
    os.makedirs(outdir, exist_ok=True)
    a = open_tree(path)
    picks = [int(x) for x in sys.argv[3:]] or list(range(min(6, a['nrec'])))
    for i in picks:
        t = tile(a, i)
        if t['kind'] == 'elev':
            hm = np.frombuffer(t['body'][:t['w']*t['h']*2], dtype='<u2').reshape(t['h'], t['w'])
            hm = hm - hm.min()
            hm = (hm * 255 // max(1, hm.max())).astype(np.uint8)
            Image.fromarray(hm).save('%s/node%03d_elev.png' % (outdir, i))
        else:
            alpha = (t['fmt'] != 4)
            img = decode_dxt(t['body'], t['w'], t['h'], alpha=alpha)
            Image.fromarray(img).save('%s/node%03d_fmt%d_%dx%d.png' % (outdir, i, t['fmt'], t['w'], t['h']))
        print('wrote node', i, t['kind'], t['w'], t['h'], t.get('fmt'))
