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
    """Epoch rail LONGITUDE [rad] in the parent frame (X-Z plane, y up).
    Mirrors system.cpp: arg_peri + true_anomaly0 when authored (both are
    physical longitudes, #146), else the circular orbit through inertial.pos
    read as atan2(-z, x), else on +X (angle 0)."""
    inertial = body.get("inertial") or {}
    if "true_anomaly0" in inertial:
        return float(inertial.get("arg_peri") or 0.0) \
            + float(inertial["true_anomaly0"])
    pos = inertial.get("pos")
    if not isinstance(pos, list) or len(pos) < 3:
        return 0.0
    return math.atan2(-float(pos[2]), float(pos[0]))


def phase_gap_rad(b1, b2):
    """Shorter epoch angle [rad] between two bodies of the same parent
    (drift either way; the short arc is the cheap one)."""
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
    print("sci_phase self-test ok: 60deg/1T drift = %.1f m/s" % got)
