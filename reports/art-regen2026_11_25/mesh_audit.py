#!/usr/bin/env python3
"""Catalogue-wide art audit: shading style, UV density spread, island count, per part.

Answers three questions the plan needs numbers for:
  1. how many meshes are smooth shaded vs flat (corner normals vs the face normal),
  2. how even is the texel density across each mesh (#1176),
  3. how much of the catalogue is generated vs hand-authored.

  python3 tmp/night7/mesh_audit.py [res/meshes]
"""
import glob
import json
import math
import os
import sys
from collections import defaultdict


def parse_obj(path):
    v, vt, vn, faces = [], [], [], []
    for ln in open(path, encoding="utf-8", errors="replace"):
        t = ln.split()
        if not t:
            continue
        if t[0] == "v":
            v.append(tuple(float(x) for x in t[1:4]))
        elif t[0] == "vt":
            vt.append((float(t[1]), float(t[2])))
        elif t[0] == "vn":
            vn.append(tuple(float(x) for x in t[1:4]))
        elif t[0] == "f":
            cs = []
            ok = True
            for tok in t[1:]:
                parts = tok.split("/")

                def idx(k, n):
                    if k >= len(parts) or parts[k] == "":
                        return None
                    i = int(parts[k])
                    return i + n if i < 0 else i - 1
                c = (idx(0, len(v)), idx(1, len(vt)), idx(2, len(vn)))
                if c[0] is None:
                    ok = False
                cs.append(c)
            if ok:
                faces.append(cs)
    return v, vt, vn, faces


def newell(pts):
    nx = ny = nz = 0.0
    n = len(pts)
    for i in range(n):
        x1, y1, z1 = pts[i]
        x2, y2, z2 = pts[(i + 1) % n]
        nx += (y1 - y2) * (z1 + z2)
        ny += (z1 - z2) * (x1 + x2)
        nz += (x1 - x2) * (y1 + y2)
    return (nx, ny, nz)


def norm(a):
    m = math.sqrt(sum(c * c for c in a))
    return tuple(c / m for c in a) if m > 1e-12 else (0.0, 0.0, 0.0)


def ang(a, b):
    d = max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b))))
    return math.degrees(math.acos(d))


def tri_area_3d(pts):
    """Fan triangulation area (works for convex ngons, which is all we have)."""
    a = 0.0
    for i in range(1, len(pts) - 1):
        u = [pts[i][k] - pts[0][k] for k in range(3)]
        w = [pts[i + 1][k] - pts[0][k] for k in range(3)]
        cr = (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0])
        a += 0.5 * math.sqrt(sum(c * c for c in cr))
    return a


def uv_area(uvs):
    a = 0.0
    n = len(uvs)
    for i in range(n):
        x1, y1 = uvs[i]
        x2, y2 = uvs[(i + 1) % n]
        a += x1 * y2 - x2 * y1
    return abs(a) / 2.0


def islands(vt, faces):
    parent = list(range(len(faces)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    by_edge = defaultdict(list)
    for k, f in enumerate(faces):
        for j in range(len(f)):
            vi = [c[0] for c in f]
            key = (vi[j], vi[j - 1])
            by_edge[(min(key), max(key))].append((k, j))
    for sides in by_edge.values():
        if len(sides) != 2:
            continue
        (ka, ja), (kb, jb) = sides
        va, vb = [c[0] for c in faces[ka]], [c[0] for c in faces[kb]]
        ua = [vt[c[1]] if c[1] is not None else None for c in faces[ka]]
        ub = [vt[c[1]] if c[1] is not None else None for c in faces[kb]]
        if ua[ja] is None or ub[jb - 1] is None:
            continue
        if va[ja] != vb[jb - 1] or va[ja - 1] != vb[jb]:
            continue
        if ua[ja] == ub[jb - 1] and ua[ja - 1] == ub[jb]:
            ra, rb = find(ka), find(kb)
            if ra != rb:
                parent[rb] = ra
    return len({find(k) for k in range(len(faces))})


def audit(path):
    v, vt, vn, faces = parse_obj(path)
    if not faces or not v:
        return None
    smooth_faces = 0
    gap_max = 0.0
    dens = []
    total_area = 0.0
    no_uv = 0
    no_norm = 0
    for f in faces:
        pts = [v[c[0]] for c in f]
        fn = norm(newell(pts))
        area = tri_area_3d(pts)
        total_area += area
        # shading: do the corner normals sit on the face plane?
        if all(c[2] is not None for c in f):
            worst = 0.0
            for c in f:
                worst = max(worst, ang(norm(vn[c[2]]), fn))
            gap_max = max(gap_max, worst)
            if worst > 5.0:
                smooth_faces += 1
        else:
            no_norm += 1
        if all(c[1] is not None for c in f):
            ua = uv_area([vt[c[1]] for c in f])
            if area > 1e-9 and ua > 1e-12:
                dens.append(math.sqrt(ua / area))
        else:
            no_uv += 1
    dens.sort()
    med = dens[len(dens) // 2] if dens else 0.0
    return dict(name=os.path.basename(path), verts=len(v), faces=len(faces),
                ngons=sum(1 for f in faces if len(f) > 4),
                islands=islands(vt, faces) if any(c[1] is not None for f in faces for c in f) else 0,
                smooth=smooth_faces, gap=gap_max, no_norm=no_norm, no_uv=no_uv,
                area=total_area, dmin=dens[0] if dens else 0.0, dmed=med,
                dmax=dens[-1] if dens else 0.0, ratio=(dens[-1] / dens[0] if dens and dens[0] > 1e-9 else 0.0))


def main(root="res/meshes"):
    rows = []
    for p in sorted(glob.glob(os.path.join(root, "*.obj"))):
        r = audit(p)
        if r:
            rows.append(r)

    parts = json.load(open("res/data/parts.json"))
    used = defaultdict(list)
    for part in (parts if isinstance(parts, list) else parts.get("parts", [])):
        m = part.get("mesh")
        if m:
            used[os.path.basename(m)].append(part.get("name") or part.get("part") or "?")

    print("%-34s %5s %5s %4s %9s %7s %9s %9s %6s %s"
          % ("mesh", "verts", "faces", "uvisl", "smooth", "gap°", "px/m lo", "px/m med", "ratio", "parts"))
    for r in rows:
        tag = "SMOOTH" if r["smooth"] else ("flat" if not r["no_norm"] else "?n")
        print("%-34s %5d %5d %4d %9s %7.1f %9.1f %9.1f %6.1f %s"
              % (r["name"], r["verts"], r["faces"], r["islands"], "%d/%s" % (r["smooth"], tag),
                 r["gap"], r["dmin"] * 512, r["dmed"] * 512, r["ratio"],
                 ",".join(used.get(r["name"], []))[:40] or "-"))

    n = len(rows)
    sm = [r for r in rows if r["smooth"]]
    print("\n%d meshes: %d have smooth-shaded faces, %d fully flat" % (n, len(sm), n - len(sm)))
    print("smoothed faces total: %d of %d"
          % (sum(r["smooth"] for r in rows), sum(r["faces"] for r in rows)))
    rat = sorted(r["ratio"] for r in rows if r["ratio"])
    print("texel-density ratio: median %.1f, 25th %.1f, 75th %.1f, worst %.1f (1.0 = perfectly even)"
          % (rat[len(rat) // 2], rat[len(rat) // 4], rat[(3 * len(rat)) // 4], rat[-1]))
    print("worst 8 by ratio: %s" % ", ".join("%s %.1f" % (r["name"], r["ratio"])
          for r in sorted((x for x in rows if x["ratio"]), key=lambda x: -x["ratio"])[:8]))
    print("most smoothed: %s" % ", ".join("%s %d/%d" % (r["name"], r["smooth"], r["faces"])
          for r in sorted(rows, key=lambda x: -x["smooth"])[:10]))
    unused = [r["name"] for r in rows if not used.get(r["name"])]
    print("mesh files not referenced by parts.json: %d -> %s"
          % (len(unused), ", ".join(unused[:20])))


if __name__ == "__main__":
    main(*sys.argv[1:])
