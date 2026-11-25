#!/usr/bin/env python3
"""Rewrite an .obj so every face carries its own normal instead of corner-averaged ones.

#1178: a plate whose corners average the three meeting faces shades as one soft gradient,
with no face edge readable. The engine keeps one normal per vertex (no GenSmoothNormals in
src/mesh.cpp) and interpolates normal0 in partsShader, so the `vn` lines alone decide
flat-vs-smooth.

Only the normals move. `v` and `vt` lines are copied through as written and an `f` corner
keeps the vertex/uv index text it shipped with, so nothing is re-serialised: no digits lost,
a no-op the second time it runs, and no position that can shift. Volume, mass, collision
hulls and every mass pin stay exactly as they were.

What is NOT preserved is the file's layout: comments and `mtllib`/`o`/`usemtl`/`s` lines move
above the vertex block, blank lines go, and CRLF becomes LF. assimp reads either order, and
nothing diffs these files by hand.

  python3 utils/flatten_normals.py --check res/meshes/gear.obj   # report, write nothing
  python3 utils/flatten_normals.py --in-place tmp/leg.obj        # rewrite one file

There is no default write path on purpose (#1051's rule): a generator that wants this applies
flatten() to the text before writing it, the way gen_station.py and gen_leg.py do.
"""
import argparse
import sys


def newell(pts):
    """Unit normal of a planar polygon -- ngons included, which is why this is not a cross
    product of the first three corners."""
    nx = ny = nz = 0.0
    n = len(pts)
    for i in range(n):
        x1, y1, z1 = pts[i]
        x2, y2, z2 = pts[(i + 1) % n]
        nx += (y1 - y2) * (z1 + z2)
        ny += (z1 - z2) * (x1 + x2)
        nz += (x1 - x2) * (y1 + y2)
    m = (nx * nx + ny * ny + nz * nz) ** 0.5
    if m == 0.0:
        return None
    return (nx / m, ny / m, nz / m)


def corner_field(field, counts, line):
    """One `v[/vt[/vn]]` token as three 1-based indices, None where the token leaves that
    field out, negatives counting from the end as the spec says. Raises rather than guessing:
    a corner that does not resolve means any normal written for it is fiction."""
    out = []
    for k, part in enumerate(field.split("/")[:3]):
        if not part:
            out.append(None)
            continue
        try:
            i = int(part)
        except ValueError:
            raise ValueError("face corner %r of %r does not parse" % (field, line))
        size = counts[k]
        i = i if i > 0 else size + 1 + i
        if not 1 <= i <= size:
            raise ValueError("face corner %r of %r is out of range (%d %s)"
                             % (field, line, size, ("vertex", "uv", "normal")[k]))
        out.append(i)
    while len(out) < 3:
        out.append(None)
    return out


def parse(text):
    """(lead, v lines, vt lines, normal values, face fields, vertices).

    Raises on geometry after a face (the index tables would be ambiguous) and on a file split
    into more than one object or material group: src/mesh.cpp:47 reads mMeshes[0] only, and
    assimp starts a new mesh at each `o`/`g`/`usemtl`, so flattening such a file would merge
    the groups into one and hoist the later `usemtl` over every face."""
    lead, v_lines, vt_lines, vn_values, faces, verts = [], [], [], [], [], []
    splits = {}
    seen_face = False
    for ln in text.splitlines():
        t = ln.split()
        if not t:
            continue
        kind = t[0]
        if kind in ("v", "vt", "vn"):
            if seen_face:
                raise ValueError("`%s`: geometry after a face is not supported" % ln.strip())
            if kind == "v":
                if len(t) < 4:
                    raise ValueError("`%s`: a vertex needs three coordinates" % ln.strip())
                v_lines.append(ln.rstrip("\n"))
                verts.append((float(t[1]), float(t[2]), float(t[3])))
            elif kind == "vt":
                if len(t) < 3:
                    raise ValueError("`%s`: a uv needs two coordinates" % ln.strip())
                vt_lines.append(ln.rstrip("\n"))
            elif len(t) < 4:
                raise ValueError("`%s`: a normal needs three components" % ln.strip())
            else:
                vn_values.append((float(t[1]), float(t[2]), float(t[3])))
        elif kind == "f":
            seen_face = True
            faces.append(t[1:])
        else:
            if kind in ("o", "g", "usemtl"):
                splits[kind] = splits.get(kind, 0) + 1
            lead.append(ln.rstrip("\n"))
    for kind, n in sorted(splits.items()):
        if n > 1:
            raise ValueError("%d `%s` lines in one file: assimp starts a mesh at each of "
                             "them and src/mesh.cpp reads mMeshes[0] only, so flattening "
                             "here would merge the groups and lose the rest" % (n, kind))
    return lead, v_lines, vt_lines, vn_values, faces, verts


def flatten(text):
    """New file text: same geometry, one normal per face, and every corner of a face naming
    that face's normal. Returns (text, face_count)."""
    lead, v_lines, vt_lines, vn_values, faces, verts = parse(text)
    counts = (len(verts), len(vt_lines), len(vn_values))
    normals = []
    for fields in faces:
        line = "f " + " ".join(fields)
        corners = [corner_field(f, counts, line) for f in fields]
        n = newell([verts[c[0] - 1] for c in corners])
        if n is None:
            raise ValueError("degenerate face `%s`: refusing to invent a normal" % line)
        normals.append(n)
    out = list(lead)
    out.extend(v_lines)
    out.extend(vt_lines)
    for k, n in enumerate(normals):
        out.append("vn %.6f %.6f %.6f" % tuple(x + 0.0 for x in n))   # -0.0 + 0.0 is 0.0
    for k, fields in enumerate(faces):
        refs = []
        for f in fields:
            parts = f.split("/")[:3]
            while len(parts) < 3:
                parts.append("")
            parts[2] = str(k + 1)               # vertex and uv tokens pass through untouched
            refs.append("/".join(parts))
        out.append("f " + " ".join(refs))
    return "\n".join(out) + "\n", len(faces)


def not_flat_faces(text, tol=1e-9):
    """(faces that do not already carry one normal across all their corners, face count).

    "Flat" means the corners agree, not that they match a normal recomputed here: capsule.obj
    and the lathes hold per-face normals that are a hair off the Newell figure of a slightly
    non-planar quad, and they are flat in every way the renderer can see. Compared by value,
    so a file reads flat whatever precision or index layout its writer chose."""
    _, _, vt_lines, vn_values, faces, verts = parse(text)
    counts = (len(verts), len(vt_lines), len(vn_values))
    bad = 0
    for fields in faces:
        line = "f " + " ".join(fields)
        corners = [corner_field(f, counts, line) for f in fields]
        if len(corners) < 3:
            # flatten() refuses such a face rather than invent a normal, so --check must not
            # certify it healthy: the two functions have to agree about what is broken.
            bad += 1
            continue
        if any(c[2] is None for c in corners):
            bad += 1                      # no normal of its own to agree with
            continue
        got = [vn_values[c[2] - 1] for c in corners]
        if any(max(abs(a - b) for a, b in zip(got[0], g)) > tol for g in got[1:]):
            bad += 1
    return bad, len(faces)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("paths", nargs="+")
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true",
                      help="report which files are not per-face flat, write nothing")
    mode.add_argument("--in-place", action="store_true",
                      help="rewrite each file (a generator should call flatten() instead)")
    a = ap.parse_args()
    dirty = failed = 0
    for p in a.paths:
        try:
            text = open(p, encoding="utf-8").read()      # strict: a lossy read must not be
            bad, nf = not_flat_faces(text)               # written back as if it were a fix
            if not bad:
                print("%s: already per-face flat (%d faces)" % (p, nf))
                continue
            dirty += 1
            if a.check:
                print("%s: %d of %d faces do not carry one normal across their corners"
                      % (p, bad, nf))
                continue
            flat, _ = flatten(text)
        except (OSError, ValueError) as exc:
            # A file this tool cannot read is a report, not a traceback: the caller is a
            # generator loop or a shell one-liner over the whole catalogue.
            print("%s: %s" % (p, exc))
            failed += 1
            continue
        with open(p, "w", encoding="utf-8") as fh:
            fh.write(flat)
        print("%s: rewrote %d faces with one normal each" % (p, nf))
    return 1 if (failed or (a.check and dirty)) else 0


if __name__ == "__main__":
    sys.exit(main())
