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


def anchor_sea_level_pressure(surface, radius_m, g):
    """#739: the file states its air's datum PRESSURE; the density is the consequence.

    Both generators still THINK in datum densities -- that is the row the RSS sheets and
    the KSP wiki publish (kg/m3 at the datum) and it is what utils/check_atmo_band.py and
    make_solar_system's fact-air guard compare against -- but a body's JSON now says how
    much the column WEIGHTS at the datum, in Pa. The loader resolves the density from it
    (src/system.cpp, through src/atmo.h), which is the direction the physics wants: with
    the density authored, sea-level pressure is a consequence of the datum temperature,
    so a warmer world invents weight (drag.h's "THE THREE ARE LOCKED", #684/#686).

    Two authored shapes, one rule -- anchor on the datum row the source actually states:
      * an exponential column: p0 = rho0 * g * scale_height, exact arithmetic on numbers
        this file already authors, and the gas cancels (the datum T that H implies is
        g H M / R, so rho0 = p0 / (g H) recovers the authored density to the last bit --
        nothing in the game moves).
      * a layer table: the generator authors `sea_level_pressure` itself, because the
        source it copies states it -- US76 says 101,325 Pa at 288.15 K, and its
        1.225 kg/m3 is that pair's ROUNDED consequence. Converting the rounded density
        back would need the gas table on this side of the boundary, and utils/gas_mixes.py
        deliberately keeps species PROPERTIES in C++ (the split tests/test_scenarios.cpp
        pins). So the four table bodies drift by ~0.014% in density, on purpose, toward
        the number their source states.

    `g` here is the body's authored g [m/s2] at its MEAN radius; the air sits on the
    datum sphere, so the same (mean / datum)^2 thinning atmo.h:datumGravity applies to the
    column and to dragAtm() per hull (#697) is applied here.
    """
    if not surface:
        return surface
    atmo = surface.get("atmosphere")
    if not atmo or "sea_level_density" not in atmo:
        return surface
    rho0 = atmo["sea_level_density"]
    if "layers" in atmo:
        raise ValueError("a layer table authors sea_level_pressure directly (#739): "
                         "the density it implies is the loader's consequence, not a "
                         "number this generator should convert")
    p0 = None
    if rho0 > 0.0 and g and atmo.get("scale_height", 0.0) > 0.0:
        datum_r = radius_m + float(surface.get("sea_level", 0.0) or 0.0)
        k = radius_m / datum_r if datum_r > 0.0 else 1.0
        p0 = rho0 * g * k * k * atmo["scale_height"]
    # Rebuild in the SAME key order, with the pressure taking the density's slot: the
    # emitted file's diff against the last one is then the one line that changed, not
    # every key after it shuffling down the page.
    rebuilt = {}
    for key, value in atmo.items():
        if key == "sea_level_density":
            if p0 is not None:
                rebuilt["sea_level_pressure"] = p0
        else:
            rebuilt[key] = value
    atmo.clear()
    atmo.update(rebuilt)
    return surface


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


# The loader's G (src/system.cpp). A guard that checks a field with a DIFFERENT
# constant is a guard that approves a file the loader then refuses.
G = 6.674e-11

# src/solar.h:kStefanBoltzmann -- the Stefan-Boltzmann constant, the same value the
# loader integrates a star's output with (#687). Written once here for the same
# reason #689 wrote G once: a guard that approves a file with a different constant
# than the loader reads is a guard that has already lost.
SIGMA_SB = 5.670374419e-8


def semimajor_axis(mu, w):
    """A circular rail's radius from its angular speed [rad/s] and the parent's GM.
    Both generators write `orb_ang_speed`, never a radius, so this is the distance the
    loader will measure a body's insolation at (#687)."""
    return (mu / (w * w)) ** (1.0 / 3.0)


def star_temperature_from_flux(flux, dist_m, radius_m):
    """[K] The INVERSE of src/solar.h's S(d) = sigma T^4 (R/d)^2: the effective
    temperature a star of this radius needs for `flux` to fall at `dist_m`. Both
    generators size their star exactly this way (#687), so the inversion is written once
    -- the same reason #690 put the geopotential conversion in this module."""
    return (flux / SIGMA_SB) ** 0.25 * (dist_m / radius_m) ** 0.5


def blackbody_temperature(flux, albedo):
    """[K] src/solar.h:blackbodyTemperature spelled in python: absorbed pi r^2 S (1-A),
    emitted 4 pi r^2 sigma T^4. The duplication is forced -- a generator cannot include a
    game header -- so SIGMA_SB above carries the same literal as kStefanBoltzmann, both
    say which module owns the law, and tests/test_solar.cpp pins the C++ half against the
    published rows the generator reads (#687)."""
    return (flux * (1.0 - albedo) / (4.0 * SIGMA_SB)) ** 0.25


# src/atmo.h:kAtmoGravityTol. Keep the two in step: this is the same rule stated on
# the authoring side, and a generator that is looser than the loader has already lost.
GRAVITY_TOL = 0.02


def check_constants_match_src(verbose=True):
    """G and SIGMA_SB above are COPIES of what the loader spells (src/system.cpp:165,
    src/solar.h:kStefanBoltzmann). Comments say so, and a comment is not a link (#707:
    G is written out five times in utils/ alone, #689 pinned one of those pairs by hand).
    So --check greps the two headers and refuses if they part company: generated data can
    be perfectly self-consistent and still describe a physically different universe from
    the one the game integrates. Returns the list of complaints; empty means they match.
    """
    import os
    import re
    src = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, 'src')
    want = [('system.cpp', r'\bG\s*=\s*([0-9.eE+-]+);', G, 'G'),
            ('solar.h', r'kStefanBoltzmann\s*=\s*([0-9.eE+-]+);', SIGMA_SB, 'sigma')]
    bad = []
    for fname, pat, mine, label in want:
        path = os.path.join(src, fname)
        try:
            with open(path, encoding='utf-8', errors='replace') as f:
                text = f.read()
        except OSError:
            bad.append('cannot read %s to check %s' % (path, label))
            continue
        m = re.search(pat, text)
        if not m:
            bad.append('src/%s: no %s literal found to compare against' % (fname, label))
            continue
        theirs = float(m.group(1))
        if verbose:
            print('  constants %-6s here %12.6g  src/%-11s %12.6g  %s'
                  % (label, mine, fname, theirs,
                     'ok' if theirs == mine else 'MISMATCH'))
        if theirs != mine:
            bad.append('src/%s states %s %.10g, utils/atmo_units.py uses %.10g'
                       % (fname, label, theirs, mine))
    return bad


def check_body_gravity(bodies, verbose=True):
    """Refuse a body whose typed `g` is not the field its own mass and radius imply
    (#689), because a body with air states that field TWICE and the loader then uses
    both: g weighs the air's column (atmo.h:resolveAtmoLevels, #683) while mass is
    what every arc about the body answers to. Equal at the surface by construction,
    they part company aloft, where the column multiplies the gap -- ln p spans 10.7
    scale heights across Kerbin's air, so a 5 % g is 41 % off on the density at its
    85 km edge. A barometer and a ballistic arc then disagree about one sky and the
    file never says which one to believe.
    Air bodies only, exactly as the loader's check does: on an airless rock the typed
    g is a readout, and the small moons' published figures carry one significant
    digit (the shipped catalogue's worst is Daphnis, 10 % off its own implied field,
    harmlessly). Prints a row per air body whether or not it agrees -- a guard that
    prints nothing is indistinguishable from one that checked nothing (#685)."""
    bad = []
    for b in bodies:
        atmo = (b.get('surface') or {}).get('atmosphere')
        g, mass, radius = b.get('g'), b.get('mass'), b.get('radius')
        if not atmo or not g or not mass or not radius:
            continue
        implied = G * mass / (radius * radius)
        drift = (g - implied) / implied
        if verbose:
            print('  gravity   %-9s g %10.5f  G*m/r^2 %10.5f  %+.2f%%'
                  % (b['name'], g, implied, 100.0 * drift))
        if abs(drift) > GRAVITY_TOL:
            bad.append('%s: authored g %.6g, G*mass/radius^2 = %.6g (%+.1f%%, tol %.0f%%)'
                       % (b['name'], g, implied, 100.0 * drift, 100.0 * GRAVITY_TOL))
    return bad
