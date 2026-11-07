#!/usr/bin/env python3
"""The altitude-unit law for authored atmosphere tables (#690), in one module the way
#636 put the named gas mixes in one module.

Both system generators emit layer tables, and each only ever compares its OWN output
against itself (--check), so this law spelled twice would drift while both checks stayed
green. A loader never sees any of it: it reads GEOMETRIC metres, and system.cpp refuses a
file that carries the marker.
"""


def geo_to_geom(Z, R):
    """A published atmosphere table's GEOPOTENTIAL height Z [m] -> the geometric altitude it
    sits at on a body of radius R. The inverse of geom_to_depth below (and of
    atmo.h:geopotentialDepth); diverges as Z -> R, which Earth's rows never get near (its top
    row is 71 km, its air top 85 km, of a 6371 km radius) but which a KSP-sized body does --
    85 km is 14% of a 600 km radius. #690."""
    return R * Z / (R - Z)


def geom_to_depth(z, R):
    """The other way: atmo.h:geopotentialDepth, the height at which a constant-g column
    weighs the same as this body's 1/r^2 column weighs up to geometric altitude z."""
    return R * z / (R + z)


def resolve_layer_heights(surface, radius_m):
    """#690: a layer table copied from a published standard marks itself
    heights: "geopotential", because US76 and friends state their rows at GEOPOTENTIAL heights
    while a system file's altitudes are GEOMETRIC metres -- the same unit the vacuum cut, the
    science bands, the shells and the atlas all speak. So the conversion happens HERE, against
    the datum radius this body's altitudes are measured from, and the marker is dropped: the
    loader never sees a second unit of altitude (system.cpp refuses the key outright if one
    reaches it). Earth's "51 km" row is 51,411.6 m here; the same row on a 600 km world is
    55,737.7 m, which is why nobody gets to paste a table in by hand.
    A lapse rate converts with its row, because it is K per unit of that SAME height
    coordinate, and it converts as a CHORD: the published gradient times dZ/dz ACROSS THE SPAN
    the level lapses through -- (Z_next - Z) / (z_next - z), with the last level's span running
    to the body's geometric air top. That puts every base_T back exactly on the published
    temperature, which is what atmo.h's seam rule then checks. Two consequences of the chord
    worth knowing: the published gradient is being stretched over a span the standard itself
    states, so raising `height` on a copied table extrapolates that gradient past the standard's
    own top row; and the level's true gradient keeps falling with height (dZ/dz =
    (R/(R+z))^2), so the chord is a secant, not the profile -- worst 0.045 K inside one of
    Earth's levels, 0.03 % of its pressure at 71 km.
    The two tempting shortcuts both get refused at load, which is the point: keep the published
    gradient beside converted bases and the column arrives 0.53 K off Earth's stratopause and
    1.09 K off its 71.8 km row; use the tangent (R/(R+z))^2 AT the row instead of the chord and
    Earth's worst seam is 0.18 K -- fine -- but a 600 km world's troposphere drifts 1.34 K,
    and kAtmoSeamTol is 0.5 K.
    No marker means the table was authored in geometric metres, which is what every hand-written
    system file states, and it is left alone. Copied rather than mutated: one authored surface
    dict is shared by every body and every system emitted from it."""
    atmo = surface.get('atmosphere') or {}
    if 'heights' not in atmo:
        return surface
    assert atmo['heights'] == 'geopotential', \
        "atmosphere.heights says %r; this is the only marker" % (atmo['heights'],)
    rows = atmo.get('layers', [])
    assert rows and atmo.get('height'), \
        "a geopotential layer table needs rows and an authored air top to chord against"
    out = dict(surface)
    at = dict(atmo)
    del at['heights']
    # Altitudes are measured from the sea-level DATUM, which is the radius plus any
    # authored sea_level (terrain.h:seaLevelRadius), so that is the radius to convert on.
    R = radius_m + surface.get('sea_level', 0.0)
    zs = [0.0 if L['alt'] == 0 else geo_to_geom(L['alt'], R) for L in rows]
    ends = list(zip([L['alt'] for L in rows[1:]], zs[1:]))      # chord ends: the next row
    ends.append((geom_to_depth(at['height'], R), float(at['height'])))
    layers = []
    for i, L in enumerate(rows):
        M = dict(L)
        M['alt'] = 0 if L['alt'] == 0 else round(zs[i], 1)
        if L.get('lapse'):
            Z_end, z_end = ends[i]
            M['lapse'] = round(L['lapse'] * (Z_end - L['alt']) / (z_end - zs[i]), 8)
        layers.append(M)
    at['layers'] = layers
    out['atmosphere'] = at
    return out
