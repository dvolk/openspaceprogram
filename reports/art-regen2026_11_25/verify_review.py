#!/usr/bin/env python3
"""Re-derive the numbers the final review disputes, independently of mesh_audit.py."""
import json, math, os, sys

R = "/home/ubuntu/openspaceprogram/res"
M = os.path.join(R, "meshes")


def parse(path):
    v, vt, vn, faces = [], [], [], []
    objs = 0
    for line in open(path):
        t = line.split()
        if not t:
            continue
        k = t[0]
        if k == "v":
            v.append(tuple(float(x) for x in t[1:4]))
        elif k == "vt":
            vt.append(tuple(float(x) for x in t[1:3]))
        elif k == "vn":
            vn.append(tuple(float(x) for x in t[1:4]))
        elif k == "f":
            faces.append([tuple(int(x) if x else 0 for x in c.split("/")) for c in t[1:]])
        elif k == "o":
            objs += 1
    return v, vt, vn, faces, objs


def unit(n):
    L = math.sqrt(sum(c * c for c in n))
    return tuple(c / L for c in n) if L else (0.0, 0.0, 0.0)


def newell(pts3):
    nx = ny = nz = 0.0
    for i in range(len(pts3)):
        x1, y1, z1 = pts3[i]
        x2, y2, z2 = pts3[(i + 1) % len(pts3)]
        nx += (y1 - y2) * (z1 + z2)
        ny += (z1 - z2) * (x1 + x2)
        nz += (x1 - x2) * (y1 + y2)
    return unit((nx, ny, nz))


def gap_max(path):
    """Largest angle between an interpolated vertex normal and a face normal."""
    v, vt, vn, faces, _ = parse(path)
    worst = 0.0
    smooth = 0
    for f in faces:
        fn = newell([v[c[0] - 1] for c in f])
        for c in f:
            if len(c) < 3 or c[2] == 0:
                continue
            n = vn[c[2] - 1]
            d = max(-1.0, min(1.0, sum(a * b for a, b in zip(fn, n))))
            worst = max(worst, math.degrees(math.acos(d)))
    # a face is "smooth" when its corners do not all share one normal VALUE
    for f in faces:
        vals = {vn[c[2] - 1] for c in f if len(c) >= 3 and c[2]}
        if len(vals) > 1:
            smooth += 1
    return worst, smooth


rows = sorted(x for x in os.listdir(M) if x.endswith(".obj"))
print("mesh files:", len(rows))

parts = json.load(open(os.path.join(R, "data/parts.json")))["parts"]
blds = json.load(open(os.path.join(R, "data/buildings.json")))["buildings"]
print("parts.json rows:", len(parts))
keys = ("mesh", "texture", "chute_mesh", "chute_texture", "shroud", "shroud_texture")
used = {}
for r in parts:
    for k in keys:
        if k in r:
            used.setdefault(r[k], []).append("parts.json:" + k)
bused = {}
for r in blds:
    for k in ("mesh", "collision_mesh", "texture"):
        if k in r:
            bused.setdefault(r[k], []).append("buildings.json:" + k)

allmesh = sorted("meshes/" + x for x in rows)
alltex = sorted("textures/" + x for x in os.listdir(os.path.join(R, "textures")) if x.endswith(".png"))
print("textures:", len(alltex))
un = [m for m in allmesh if m not in used]
print("unreferenced by parts.json (all 6 keys):", len(un))
for m in un:
    print("   ", m, "LIVE-via-buildings" if m in bused else "DEAD")
print("textures unreferenced by parts.json:", [t for t in alltex if t not in used])
print("  of those, live via buildings.json:", [t for t in alltex if t not in used and t in bused])

# engine family
print("\nengine-family lathe check")
for name in ("engine", "engine_r0.25h1", "engine_r1.5h3", "engine_r2.25h4.5", "nuclear_engine",
             "orbital_engine", "orbital_engine_r1.5h1.5", "orbital_engine_r2.25h2.25", "jet"):
    p = os.path.join(M, name + ".obj")
    if not os.path.exists(p):
        print("   %-26s NO FILE" % name)
        continue
    v, vt, vn, faces, _ = parse(p)
    zs = [x[2] for x in v]
    sizes = {}
    for f in faces:
        sizes[len(f)] = sizes.get(len(f), 0) + 1
    print("   %-26s v%d vn%d f%d %s zspan %.3f" % (name, len(v), len(vn), len(faces), sizes, max(zs) - min(zs)))

# per-file shading / UV buckets
flat = smoothmesh = nous = 0
totfaces = 0
smoothfaces = 0
gaps = {}
one_texel = []
strip = []
for name in rows:
    p = os.path.join(M, name)
    v, vt, vn, faces, _ = parse(p)
    totfaces += len(faces)
    g, sf = gap_max(p)
    smoothfaces += sf
    gaps[name] = g
    if not vt or not any(c[1] for f in faces for c in f):
        nous += 1
    else:
        area = 0.0
        for f in faces:
            p = [vt[c[1] - 1] for c in f if c[1]]
            if len(p) >= 3:
                a = 0.0
                for i in range(len(p)):
                    x1, y1 = p[i]
                    x2, y2 = p[(i + 1) % len(p)]
                    a += x1 * y2 - x2 * y1
                area += abs(a) / 2.0
        uniq = {tuple(round(c, 6) for c in vt[i - 1]) for f in faces for i in (c[1] for c in f) if i}
        if area == 0.0:
            (one_texel if len(uniq) == 1 else strip).append(name)
    if sf == 0:
        flat += 1
    else:
        smoothmesh += 1
print("\nflat-by-value meshes:", flat, " with interpolated faces:", smoothmesh)
print("interpolated faces total:", smoothfaces, " file faces total:", totfaces)
print("no vt layer:", nous)
print("zero-area, one texel:", len(one_texel), sorted(one_texel))
print("zero-area, degenerate strip:", len(strip), sorted(strip))
top = sorted(gaps.items(), key=lambda kv: -kv[1])[:8]
print("worst gaps:", [(k, round(g, 1)) for k, g in top])

# after deleting wing.obj
print("\nafter deleting wing.obj:")
print("  meshes:", len(rows) - 1, " faces:", totfaces - len(parse(os.path.join(M, "wing.obj"))[3]),
      " mean %.1f" % ((totfaces - len(parse(os.path.join(M, "wing.obj"))[3])) / (len(rows) - 1)))
g2 = {k: g for k, g in gaps.items() if k != "wing.obj"}
print("  worst gap then:", [(k, round(g, 1)) for k, g in sorted(g2.items(), key=lambda kv: -kv[1])[:5]])

# island separation for adapters: min corner-to-corner px between islands of different faces
print("\nisland separation (gen_adapter families), 512 px canvas")


def sep(path, px=512):
    v, vt, vn, faces, _ = parse(path)
    # island = per-face uv polygon (adapters give one island per face)
    polys = [[vt[c[1] - 1] for c in f if c[1]] for f in faces]
    polys = [p for p in polys if len(p) >= 3]
    best = None
    for i in range(len(polys)):
        for j in range(i + 1, len(polys)):
            for a in polys[i]:
                for b in polys[j]:
                    d = math.hypot(a[0] - b[0], a[1] - b[1]) * px
                    if best is None or d < best:
                        best = d
    return best, len(polys)


for name in ("adapter_r1to1.5", "adapter_r1.5to1", "capsule_cyl_r1h2", "nose_cap", "capsule"):
    p = os.path.join(M, name + ".obj")
    if os.path.exists(p):
        print("   %-22s min corner gap %.2f px over %d uv polys" % (name, *(sep(p),)))

# multi-object / mtl reality
mo = [n for n in rows if parse(os.path.join(M, n))[4] > 1]
mtl = [n for n in rows if any(l.startswith("mtllib") or l.startswith("usemtl") for l in open(os.path.join(M, n)))]
print("\nmulti-object files:", len(mo), " files with mtllib/usemtl:", len(mtl),
      " .mtl files in res/:", sum(1 for dp, _, fs in os.walk(R) for f in fs if f.endswith(".mtl")))
