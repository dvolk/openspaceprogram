#!/usr/bin/env python3
# Reproduce the worked examples in binary-systems.md (sections 7.1-7.4).
# Pure stdlib; run with: python3 binary_numbers.py

import math

G = 6.674e-11


def pair(mA, mB, a, label):
    """Barycenter geometry of a rigid circular pair (report section 4.1).

    a: separation between the members
    rA/rB: each member's radius about the barycenter
    P: pair period (Kepler III), omega: common angular rate
    """
    M = mA + mB
    P = 2 * math.pi * math.sqrt(a**3 / (G * M))
    rA = a * mB / M
    rB = a * mA / M
    w = 2 * math.pi / P
    print(f"{label}")
    print(f"  separation a     = {a/1e3:10.1f} km")
    print(f"  period           = {P:10.0f} s  ({P/60:.1f} min)")
    print(f"  rA (about bary)  = {rA/1e3:10.1f} km   (barycenter offset from A)")
    print(f"  rB (about bary)  = {rB/1e3:10.1f} km")
    print(f"  omega            = {w:10.3e} rad/s")
    print()
    return rA, rB, P


def separation_for_period(mA, mB, P, label):
    """Invert Kepler III: masses + desired period -> separation."""
    a = (G * (mA + mB) * (P / (2 * math.pi)) ** 2) ** (1 / 3)
    rA, rB, _ = pair(mA, mB, a, label)
    return a, rA, rB


def hill_radius(a_parent, M, M_parent, label):
    """Hill radius of a pair of total mass M at a_parent from M_parent."""
    R = a_parent * (M / (3 * M_parent)) ** (1 / 3)
    print(f"{label}")
    print(f"  Hill radius      = {R/1e3:10.0f} km")
    print(f"  0.5*R (stable)   = {0.5*R/1e3:10.0f} km  (prograde limit, use ~half)")
    print()
    return R


print("=" * 62)
print("7.1 planet-planet 'double planet'")
pair(5.0e22, 2.5e22, 2.0e6, "     primary 5e22 kg r=600 km; companion 2.5e22 kg r=400 km; a = 2000 km")
print("     -> barycenter 667 km from A's center, outside its 600 km radius")
print()

print("=" * 62)
print("7.2 Pluto-Charon analog (mass ratio 0.122, P = 6.39 d)")
mA = 4.515e21
a, rA, rB = separation_for_period(mA, 0.122 * mA, 6.387 * 86400,
                                  f"     primary {mA:.3e} kg (Duna-like); companion 0.122x; P fixed")
print(f"     -> barycenter {rA/1e3:.0f} km from A's center, outside a 320 km radius")
print()

print("=" * 62)
print("7.3 moon-moon pair")
pair(1.0e20, 5.0e19, 5.0e5, "     moon1 1e20 kg r=200 km; moon2 5e19 kg r=120 km; a = 500 km")
print("     -> barycenter 167 km from moon1's center, inside its 200 km radius")
print()

print("=" * 62)
print("7.4 stability: 7.5e22 kg pair at Kerbin's distance from Kerbol")
R = hill_radius(1.36e10, 7.5e22, 1.757e28,
                "     pair 7.5e22 kg at a_parent = 1.36e10 m; Kerbol 1.757e28 kg")
print(f"     -> 2000 km separation is ~{0.5*R/2000:.0f}x inside the 0.5*R_H limit")
