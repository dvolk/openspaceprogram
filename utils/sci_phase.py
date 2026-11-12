#!/usr/bin/env python3
# sci_phase.py -- co-orbital phasing cost, the missing term behind
# sci_dist.py's sibling-hop transfer_dv (issue #128).
#
# A Hohmann hop between near-equal SMAs is degenerate: the phase gap never
# closes, so waiting for a window (free in this value model) never helps.
# The real cost is a drift: two small burns offset the orbital period just
# enough to sweep the gap in the drift time. From n' - n = -(3/2) n (da/a)
# and dv ~ v_c da/(2a), ONE burn is (1/3) a |dlam| / t; the factor 2 is the
# pair (drift out + recircularize), the same two-burn convention as
# hohmann_delta_v:
#
#     dv_drift = (2/3) * a * |dlam| / t_drift
#
# t_drift = PHASING_DRIFT_PERIODS * the target's orbital period. One period
# puts a 60-deg Kerbin-trojan hop at ~1.0 km/s (Eve/Duna-hop scale); the
# small-da linearization is ~15% optimistic at that operating point, fine
# for a value model. Destination-value exceptions belong in the generator
# (gen_systems.py overrides), not in post-generation JSON edits.
#
# The planner-side twin of this (multi-rev solutions in the game) is #136.
#
# INCLINED CO-ORBITALS (#177). A Hohmann is degenerate at equal SMA; it is NOT degenerate at
# equal radius and different PLANE, and the real Solar System's Earth co-orbitals (2020 XL5,
# Kamo'oalewa) are exactly that -- a within 0.1% of Earth's and 8-14 deg off its plane. So the
# longitude read below is parent-frame (option 1 of that issue), and the tilt a hop has to
# buy, 2 v sin(di/2) at a node crossing, is priced by sci_dist._sibling_leg_dv using
# plane_angle_rad. Both terms, because the drift alone would price a 15-deg-tilted hop
# cheaper than a flat one.
import math

PHASING_DRIFT_PERIODS = 1.0
CO_ORBITAL_REL_TOL = 1e-3   # |a1/a2 - 1| below this: Hohmann is degenerate


def orb_w(body):
    """Mean angular rate [rad/s] from the inertial block (0 if none)."""
    return float((body.get("inertial") or {}).get("orb_ang_speed") or 0.0)


def is_co_orbital(a1, a2, rel_tol=CO_ORBITAL_REL_TOL):
    if a1 <= 0.0 or a2 <= 0.0:
        return False
    return abs(a1 / a2 - 1.0) < rel_tol


def orbit_angle_rad(body):
    """Epoch rail LONGITUDE [rad], X-Z plane, y up, measured in the PARENT's frame.

    #177 left this returning two different things: `arg_peri + true_anomaly0` is the angle
    in the body's OWN rail plane (measured from that body's ascending node), while an
    authored `pos` is read as atan2(-z, x) in the PARENT's frame. Option 1 of that issue --
    apply the plane before returning -- is what this does now, because the real Solar System
    ships inclined co-orbitals (2020 XL5 and Kamo'oalewa: a within 0.1% of Earth's and
    13.9 / 7.8 deg off its plane), so the precondition the assert used to guard cannot hold.

    The rotation mirrors the loader rather than a textbook: pos_at_elements puts the body at
    railAz(0) * (cos phi, 0, -sin phi) with phi = arg_peri + true_anomaly (orbit.h:497) and
    planeOrient(incl, raan) = railAz(raan) * Rx(incl) carries the tilt (system.cpp:383-398),
    so the parent-frame longitude is
        atan2( sin(raan) cos(phi) + cos(raan) cos(incl) sin(phi),
               cos(raan) cos(phi) - sin(raan) cos(incl) sin(phi) )
    which reduces to phi exactly when incl and raan are zero -- every pair this module priced
    before, so no shipped number moves."""
    inertial = body.get("inertial") or {}
    if "true_anomaly0" in inertial:
        phi = float(inertial.get("arg_peri") or 0.0) + float(inertial["true_anomaly0"])
        incl = float(inertial.get("orb_incl") or 0.0)
        raan = float(inertial.get("lon_asc_node") or 0.0)
        if incl == 0.0 and raan == 0.0:
            return phi
        x = math.cos(raan) * math.cos(phi) - math.sin(raan) * math.cos(incl) * math.sin(phi)
        z = -math.sin(raan) * math.cos(phi) - math.cos(raan) * math.cos(incl) * math.sin(phi)
        return math.atan2(-z, x)
    pos = inertial.get("pos")
    if not isinstance(pos, list) or len(pos) < 3:
        return 0.0
    return math.atan2(-float(pos[2]), float(pos[0]))


def orbit_normal_rad(body):
    """[unit vector] The orbit plane's normal in the PARENT's frame, +Y for a body that
    authors no plane angles (the flat plane, which is what the loader assumes too).

    Mirrors planeOrient(incl, raan) * +Y: railAz(raan) applied to Rx(incl)'s +Y, giving
    (sin raan sin incl, cos incl, cos raan sin incl)."""
    inertial = body.get("inertial") or {}
    incl = float(inertial.get("orb_incl") or 0.0)
    raan = float(inertial.get("lon_asc_node") or 0.0)
    si, ci = math.sin(incl), math.cos(incl)
    return (math.sin(raan) * si, ci, math.cos(raan) * si)


def plane_angle_rad(b1, b2):
    """[rad] The angle between two bodies' ORBITAL PLANES (0-180 deg): the angle between
    their normals, which is the tilt a hop between them has to buy and is NOT the same number
    as their inclination difference (two orbits at i = 60 deg with nodes 180 deg apart are
    120 deg apart). This is the quantity the co-orbital plane-change cost wants; the
    along-track gap it drifts around is phase_gap_rad."""
    n1, n2 = orbit_normal_rad(b1), orbit_normal_rad(b2)
    d = sum(a * b for a, b in zip(n1, n2))
    return math.acos(max(-1.0, min(1.0, d)))


def phase_gap_rad(b1, b2):
    """Shorter epoch longitude angle [rad] between two bodies of the same parent
    (drift either way; the short arc is the cheap one).

    Both readings are parent-frame now (#177 option 1), so this is defined for inclined
    bodies: what a phasing burn sweeps is the along-track longitude gap, and a longitude is a
    parent-frame quantity. It is a PROJECTION for a tilted pair -- the 3-D separation also
    has an out-of-plane part -- and that part is what sci_dist._sibling_leg_dv prices as a
    plane change, so the pair of terms together covers what the gap alone cannot."""
    d = abs(orbit_angle_rad(b1) - orbit_angle_rad(b2)) % (2.0 * math.pi)
    return min(d, 2.0 * math.pi - d)


def phasing_delta_v(sma, orb_ang_speed, dlam):
    """Drift cost [m/s] to sweep dlam [rad] over PHASING_DRIFT_PERIODS
    periods of the sma orbit."""
    if sma <= 0.0 or orb_ang_speed <= 0.0 or dlam <= 0.0:
        return 0.0
    t = PHASING_DRIFT_PERIODS * (2.0 * math.pi / orb_ang_speed)
    return (2.0 / 3.0) * sma * dlam / t


if __name__ == "__main__":
    # Self-test on hand-computable cases.
    # Kerbin-like orbit: a=1.3601e10, w=6.8269e-7 (T=106.5 d).
    w = 6.826918953790463e-07
    a = 1.3601e10
    T = 2.0 * math.pi / w
    # 60 deg gap, drift over 1 period: (2/3)*a*(pi/3)/T
    got = phasing_delta_v(a, w, math.pi / 3.0)
    want = (2.0 / 3.0) * a * (math.pi / 3.0) / T
    assert abs(got - want) < 1e-9, (got, want)
    assert 1000.0 < got < 1100.0, got          # ~1032 m/s, Eve/Duna-hop scale
    # Co-orbital test: Shay vs Kerbin SMa are equal to ~1e-8.
    assert is_co_orbital(13600958.1, 13600957.7)
    assert not is_co_orbital(13600958.1, 20727858.2)
    # Epoch angle mirrors system.cpp: true_anomaly0 wins over pos, and an
    # authored pos is read as the longitude atan2(-z, x). Shay's vector is
    # the committed ksp_system.json one (gen_systems.py pos_at_longitude).
    b_ta = {"name": "H", "inertial": {"orb_ang_speed": w,
              "arg_peri": 0.0, "true_anomaly0": 3.14}}
    b_pos = {"name": "T", "inertial": {"orb_ang_speed": w,
               "pos": [-6818669465, 0, 11766962302]}}
    gap = phase_gap_rad(b_ta, b_pos)
    assert abs(gap - math.pi / 3.0) < 1e-3, gap   # Kerbin->Shay L4: 60 deg
    # Gap is symmetric and takes the short arc (240 -> 120).
    assert abs(phase_gap_rad(b_pos, b_ta) - gap) < 1e-12
    b_120 = {"name": "T2", "inertial": {"orb_ang_speed": w,
                "true_anomaly0": math.radians(60.0)}}
    assert abs(phase_gap_rad(b_ta, b_120) - (3.14 - math.radians(60.0))) < 1e-9
    # Nonzero arg_peri must be ADDED (Moho's authored elements): dropping it
    # would still pass every case above.
    moho = {"name": "Moho", "inertial": {
        "arg_peri": 0.2617993877991494, "true_anomaly0": 3.140508989974855}}
    assert abs(orbit_angle_rad(moho)
               - (0.2617993877991494 + 3.140508989974855)) < 1e-12
    # #177 option 1, the inclined case: an elements-authored body's longitude is measured
    # in the PARENT's frame, so the rail angle and the parent angle are DIFFERENT numbers
    # and the difference is the plane. Hand-checked: incl 60 deg, raan 0, rail longitude
    # 45 deg sits at (cos45, 0, -cos60*sin45) after the tilt, so the parent longitude is
    # atan2(0.3536, 0.7071) = 26.565 deg. Before this fix the function returned 45 deg,
    # and differencing that against a coplanar body's longitude was the silent wrong dv.
    tilt = {"name": "Tilt", "inertial": {"orb_ang_speed": w, "arg_peri": 0.0,
              "true_anomaly0": math.radians(45.0), "orb_incl": math.radians(60.0)}}
    want = math.atan2(0.5 * math.sin(math.radians(45.0)), math.cos(math.radians(45.0)))
    assert abs(orbit_angle_rad(tilt) - want) < 1e-12, orbit_angle_rad(tilt)
    assert abs(math.degrees(orbit_angle_rad(tilt)) - 26.56505) < 1e-4
    # On the ascending node the rail longitude is 0, so the parent longitude IS the node
    # longitude -- the frame difference #177 measured on Minmus (89.57 local against 167.56
    # parent, exactly its 78.0 deg node).
    node = {"name": "Node", "inertial": {"orb_incl": 0.1047, "lon_asc_node": 1.3614,
              "arg_peri": 0.0, "true_anomaly0": 0.0}}
    assert abs(orbit_angle_rad(node) - 1.3614) < 1e-12
    # The PLANE angle is between normals, so it is not the inclination difference: two 60
    # deg orbits whose nodes are 180 deg apart are 120 deg apart. This is what the
    # co-orbital plane-change cost reads, and a coplanar pair -- every pair shipped before
    # the real Earth co-orbitals -- reads exactly 0, so no existing number moves.
    n0 = {"name": "n0", "inertial": {"orb_incl": math.radians(60.0), "lon_asc_node": 0.0}}
    n180 = {"name": "n180", "inertial": {"orb_incl": math.radians(60.0),
              "lon_asc_node": math.pi}}
    assert abs(math.degrees(plane_angle_rad(n0, n180)) - 120.0) < 1e-9
    assert abs(math.degrees(plane_angle_rad(n0, {"name": "F", "inertial": {}})) - 60.0) < 1e-9
    assert plane_angle_rad(tilt, tilt) < 1e-9
    # And the gap the drift sweeps is now defined across planes instead of throwing: Tilt at
    # 26.565 deg against b_pos at -120.091 deg is the 146.656 deg short arc.
    assert abs(math.degrees(phase_gap_rad(tilt, b_pos)) - 146.656) < 1e-3
    print("sci_phase self-test ok: 60deg/1T drift = %.1f m/s" % got)
