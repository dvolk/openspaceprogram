#!/usr/bin/env python3
"""A reader for Orbiter's .tree tile archives (staging/orbiter-data).

Format as cracked from the shipped files (Titan Cloud/Label/Elev, Io Surf,
Phobos Elev):

  header   48 B   'TX', u16 version=1, u32 48, u32 1, u32 index_bytes,
                  u32 data_bytes, u32 0, u32 nrec, then u32 roots 0,1,2,3
  nodes    32 B x nrec, depth-first: u64 tile offset (relative to data start),
                  u32 UNCOMPRESSED tile size, s32 child[4] (-1 = leaf), u32 key
  data     each tile payload is one raw zlib stream that inflates to
                  128-byte tile header + raster

Tile header (128 B): u32 magic, u32 124, u32 csum, u32 w, u32 h, u32 raster
bytes, ..., u32 32, u32 4, u32 fourcc(0x31545844='DXT1'), ..., u32 blocks...
Raster is DXT1 (0.5 B/px) for Surf, DXT3 (1 B/px) for Cloud, 16-bit-ish for
Elev.
"""
import struct, zlib, sys


def rgba565(c):
    r = (c >> 11) & 31
    g = (c >> 5) & 31
    b = c & 31
    return (r * 255 // 31, g * 255 // 31, b * 255 // 31)


def dxt1_block(blk):
    c0, c1 = struct.unpack_from('<HH', blk, 0)
    col = [rgba565(c0), rgba565(c1)]
    if c0 > c1:
        col += [((col[0][0]*2+col[1][0])//3, (col[0][1]*2+col[1][1])//3, (col[0][2]*2+col[1][2])//3),
                ((col[0][0]+col[1][0]*2)//3, (col[0][1]+col[1][1]*2)//3, (col[0][2]+col[1][2]*2)//3)]
    else:
        col += [((col[0][0]+col[1][0])//2, (col[0][1]+col[1][1])//2, (col[0][2]+col[1][2])//2), (0, 0, 0, 1)]
    out = []
    for row in range(4):
        for col_i in range(4):
            byte = blk[8 + row]
            out.append(col[(byte >> (2 * col_i)) & 3])
    return out


def decode_dxt1(buf, w, h):
    px = []
    for by in range(0, h, 4):
        for bx in range(0, w, 4):
            blk = buf[(by//4)*(w//4)*8 + (bx//4)*8:(by//4)*(w//4)*8 + (bx//4)*8 + 8]
            blk = bytes(blk) + b'\x00' * (8 - len(blk))
            px.append((bx, by, dxt1_block(blk)))
    return px


def to_png(buf, w, h, path):
    from PIL import Image
    img = Image.new('RGB', (w, h))
    for bx, by, blk in decode_dxt1(buf, w, h):
        for i, c in enumerate(blk):
            x, y = bx + i % 4, by + i // 4
            if x < w and y < h:
                img.putpixel((x, y), c[:3])
    img.save(path)
    return w, h


def open_tree(path):
    d = open(path, 'rb').read()
    magic = d[:2]
    ver, h1, one, idxsz, datasz = struct.unpack_from('<HIIII', d, 2)
    nrec = struct.unpack_from('<I', d, 24)[0]
    roots = struct.unpack_from('<4I', d, 32)
    nodes = []
    for i in range(nrec):
        o = 48 + 32*i
        off, sz = struct.unpack_from('<QI', d, o)
        ch = struct.unpack_from('<4i', d, o+12)
        key = struct.unpack_from('<I', d, o+28)[0]
        nodes.append(dict(i=i, off=off, size=sz, child=ch, key=key))
    return dict(magic=magic, ver=ver, idxsz=idxsz, datasz=datasz, nrec=nrec,
                roots=roots, nodes=nodes, data=d)


def tile(arch, i):
    n = arch['nodes'][i]
    raw = zlib.decompress(arch['data'][arch['idxsz'] + n['off']:])
    hdr = raw[:128]
    magic, hlen, csum, w, h, nbytes = struct.unpack_from('<IIIIII', hdr, 0)
    fourcc = struct.unpack_from('<I', hdr, 80)[0]
    return dict(w=w, h=h, raster=nbytes, fourcc=fourcc, body=raw[128:],
                hdr=struct.unpack_from('<32I', hdr, 0))


if __name__ == '__main__':
    path = sys.argv[1]
    a = open_tree(path)
    print('%s: ver %d, %d nodes, index %d B, data %d B, roots %s'
          % (path.split('/')[-1], a['ver'], a['nrec'], a['idxsz'], a['datasz'], a['roots']))
    sizes = {}
    for n in a['nodes']:
        t = tile(a, n['i'])
        sizes.setdefault((t['w'], t['h'], t['fourcc']), 0)
        sizes[(t['w'], t['h'], t['fourcc'])] += 1
    for k, v in sorted(sizes.items()):
        print('   %4dx%-4d fourcc 0x%08x  x%d' % (k[0], k[1], k[2], v))
    t = tile(a, 0)
    print('   node0 tile header words:', t['hdr'][:12])
