#!/usr/bin/env python3
"""One spelling of "what moved between the committed system JSON and what the
generator would write today" (#725).

Both system generators needed this answer and each wrote its own. gen_systems.py
printed the leaf that moved; make_solar_system.py printed one WORD per file
("DRIFT   .../solar_system_full.json"), because its compare was a single dict
equality over ~240 bodies. So a stale Solar System read identically whether one
key in one body had moved or four hundred keys had, and a partial regenerate
looked like a total one. Same rule as #636/#689/#690: a shared behaviour gets one
module, because two spellings can drift and only one of them gets noticed.

Callers pre-strip whatever they do not want weighed (sci_dist.py re-stamps
science_mult/transfer_dv, so make_solar_system.py drops those before comparing),
then call diff_documents(); it returns the difference lines, empty meaning OK.
"""


def deep_diff(a, b, path, out):
    """Value-level diff (a = committed, b = generated); numeric compare ignores
    float spelling. Which keys are INCLUDED is the caller's business -- see the
    module docstring and each generator's own comment about its overrides."""
    if isinstance(a, dict) and isinstance(b, dict):
        for k in a:
            p = f"{path}.{k}" if path else k
            if k not in b:
                out.append(f"{p}: in committed file, missing from generated ({a[k]!r})")
            else:
                deep_diff(a[k], b[k], p, out)
        for k in b:
            p = f"{path}.{k}" if path else k
            if k not in a:
                out.append(f"{p}: in generated, missing from committed file ({b[k]!r})")
    elif isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            out.append(f"{path}: list length committed={len(a)} generated={len(b)}")
        for i in range(min(len(a), len(b))):
            deep_diff(a[i], b[i], f"{path}[{i}]", out)
    else:
        if a != b:
            out.append(f"{path}: committed={a!r} generated={b!r}")


def diff_documents(committed, generated):
    """Root keys except "bodies" compared whole, bodies compared BY NAME.

    The generator is the source of truth for every root key, so any difference
    there is drift -- it used to be that home/skybox/belts were whitelisted, and
    that whitelist is exactly why hand-edited pads went unnoticed (#226).
    Bodies are indexed by name rather than compared positionally so a rename or a
    dropped body reports as itself, not as a cascade of positional noise across
    every body after it. The ORDER is still checked, but only when the name set
    matches: the array order is the loader's bodies vector, so a hand-reorder of
    the JSON is drift too.
    """
    out = []
    deep_diff({k: v for k, v in committed.items() if k != "bodies"},
              {k: v for k, v in generated.items() if k != "bodies"}, "", out)
    cb = {x["name"]: x for x in committed.get("bodies", [])}
    gb = {x["name"]: x for x in generated.get("bodies", [])}
    for name in cb:
        if name not in gb:
            out.append(f"body {name}: in committed file, missing from generated")
    for name in gb:
        if name not in cb:
            out.append(f"body {name}: in generated, missing from committed file")
    cn = [x["name"] for x in committed.get("bodies", [])]
    gn = [x["name"] for x in generated.get("bodies", [])]
    if cn != gn and sorted(cn) == sorted(gn):
        out.append("bodies: same names in a different order")
    for name in cb:
        if name in gb:
            body_out = []
            deep_diff(cb[name], gb[name], "", body_out)
            for line in body_out:
                out.append(f"{name}: {line}")
    return out


def report(path, diff, limit=40):
    """The one failure output both generators print: the word DRIFT, the file, and
    the leaves that moved -- capped, but always with the TOTAL, so "1 key" and
    "400 keys" are separable at a glance (#725)."""
    if not diff:
        print(f"OK      {path}")
        return True
    print(f"DRIFT   {path}:")
    for line in diff[:limit]:
        print(f"    {line}")
    if len(diff) > limit:
        print(f"    ... {len(diff) - limit} more leaf differences"
              f" ({len(diff)} in total)")
    return False


def _self_test():
    a = {"home": "Earth", "bodies": [{"name": "Earth", "radius": 1.0},
                                      {"name": "Mars", "radius": 0.5}]}
    b = {"home": "Earth", "bodies": [{"name": "Mars", "radius": 0.5},
                                      {"name": "Earth", "radius": 1.0}]}
    assert diff_documents(a, b) == ["bodies: same names in a different order"], \
        diff_documents(a, b)
    # A dropped body names itself instead of cascading over everything after it.
    c = {"bodies": [{"name": "Earth", "radius": 1.0}, {"name": "Mars", "radius": 0.5}]}
    d = {"bodies": [{"name": "Earth", "radius": 1.0}]}
    got = diff_documents(c, d)
    assert got == ["body Mars: in committed file, missing from generated"], got
    # A leaf deep in one body names the body and the key path.
    e = {"bodies": [{"name": "Earth", "surface": {"atmosphere": {"pressure": 101.3}}}]}
    f = {"bodies": [{"name": "Earth", "surface": {"atmosphere": {"pressure": 100.0}}}]}
    got = diff_documents(e, f)
    assert got == ["Earth: surface.atmosphere.pressure: committed=101.3 generated=100.0"], got
    # A key the generator added and one it lost, both named.
    g = {"bodies": [{"name": "E", "a": 1}]}
    h = {"bodies": [{"name": "E", "b": 2}]}
    got = sorted(diff_documents(g, h))
    assert got == sorted(["E: a: in committed file, missing from generated (1)",
                          "E: b: in generated, missing from committed file (2)"]), got
    assert diff_documents({"x": [1, 2]}, {"x": [1, 2, 3]}) == \
        ["x: list length committed=2 generated=3"]
    print("sysdiff: all tests passed")
    return 0


if __name__ == '__main__':
    _self_test()
