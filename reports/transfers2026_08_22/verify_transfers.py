#!/usr/bin/env python3
# Numbers cited in transfer_report.md, computed from the same constants the
# game uses (G = 6.674e-11, src/main.cpp L487/L1031; masses/radii/positions
# from ksp_system.json and gen_systems.py). Run: python3 verify_transfers.py
import math

G = 6.674e-11  # the game's G

def hohmann(mu, r1, r2):
    """One-tangent Hohmann between two circular orbits about the same body."""
    a_t = (r1 + r2) / 2.0
    vc1, vc2 = math.sqrt(mu / r1), math.sqrt(mu / r2)
    vp = math.sqrt(mu * (2.0 / r1 - 1.0 / a_t))   # transfer speed at r1
    va = math.sqrt(mu * (2.0 / r2 - 1.0 / a_t))   # transfer speed at r2
    return {
        "a_t": a_t, "vc1": vc1, "vc2": vc2, "vp": vp, "va": va,
        "dv1": vp - vc1, "dv2": abs(vc2 - va),
        "tof": math.pi * math.sqrt(a_t ** 3 / mu),  # half period
    }

def show(title, h, extra=None):
    print(f"== {title} ==")
    print(f"  dv1 (depart)  = {h['dv1']:9.1f} m/s   "
          f"(vc1={h['vc1']:.1f} -> vp={h['vp']:.1f})")
    print(f"  dv2 (capture) = {h['dv2']:9.1f} m/s   "
          f"(va={h['va']:.1f} -> vc2={h['vc2']:.1f})")
    tof = h["tof"]
    print(f"  ToF           = {tof:9.0f} s = {tof/3600:6.2f} h = {tof/86400:6.1f} d")
    if extra:
        extra(h)

# --- bodies (ksp_system.json / gen_systems.py) ------------------------------
Kerbol_mu  = G * 1.757e28
Kerbin_mu  = G * 5.292e22
Mun_mu     = G * 9.76e20
Duna_mu    = G * 4.515e21
Eerbon_mu  = G * 5.2915793e22   # system.json

Kerbin_r, Mun_r = 600e3, 200e3
Duna_r          = 320e3
Mun_dist   = 12.0e6            # |pos| of Mun in Kerbin frame
Kerbin_dist = 13.59984026e9    # Kerbol frame
Duna_dist   = 20.72615526e9

LKO = Kerbin_r + 100e3         # 100 km LKO

# 1. Kerbin LKO -> Mun orbit (the canonical first transfer)
h = hohmann(Kerbin_mu, LKO, Mun_dist)
show("Kerbin 100 km LKO -> Mun (12,000 km)", h)

# 2. Interplanetary: Kerbin -> Duna (heliocentric Hohmann)
h = hohmann(Kerbol_mu, Kerbin_dist, Duna_dist)
def duna_extra(hh):
    vinf = hh["dv1"]
    vesc_lko = math.sqrt(2 * Kerbin_mu / LKO)
    v_circ_lko = math.sqrt(Kerbin_mu / LKO)
    v_peri = math.sqrt(vinf ** 2 + vesc_lko ** 2)
    print(f"  combined LKO departure burn = {v_peri - v_circ_lko:6.1f} m/s "
          f"(vinf={vinf:.1f}, vesc(LKO)={vesc_lko:.1f}, v_circ(LKO)={v_circ_lko:.1f})")
    # synodic period + departure phase angle (Duna ahead of Kerbin)
    P_k = 2 * math.pi * math.sqrt(Kerbin_dist ** 3 / Kerbol_mu)  # Kerbol orbit of Kerbin
    P_d = 2 * math.pi * math.sqrt(Duna_dist ** 3 / Kerbol_mu)    # Kerbol orbit of Duna
    P_syn = 1.0 / abs(1.0 / P_k - 1.0 / P_d)
    lead = math.pi - (2 * math.pi / P_d) * hh["tof"]
    print(f"  synodic period   = {P_syn/86400:6.1f} d   "
          f"(windows every ~{P_syn/86400:.0f} d)")
    print(f"  Duna lead angle  = {math.degrees(lead):5.1f} deg at departure")
show("Kerbin -> Duna (interplanetary, heliocentric Hohmann)", h, duna_extra)

# 3. Capture costs once inside the target
for name, mu, r_t, v_inf in [
    ("Mun LMO  100 km", Mun_mu,  Mun_r + 100e3,  None),
    ("Duna LDO 100 km", Duna_mu, Duna_r + 100e3, 826.2),
]:
    v_circ = math.sqrt(mu / r_t)
    line = f"  {name}: v_circ = {v_circ:6.1f} m/s"
    if v_inf is not None:
        v_capt = math.sqrt(v_inf ** 2 + 2 * mu / r_t)
        line += f"  capture dv = {v_capt - v_inf:6.1f} m/s (from v_inf={v_inf})"
    print(line)

# 4. Eerbon system (system.json)
h = hohmann(Eerbon_mu, Kerbin_r + 100e3, 12.0e6)
show("Eerbon 100 km LKO -> Moon (12,000 km)", h)

# 5. Ship delta-v budget (src/main.cpp)
#    parts: capsule 0.5 + reaction wheel 1.0 + engine 3.0 (L1971-1977);
#    fuel per engine: 1.0 H2 + 1.0 LOX (L933-934). getMass() sums part
#    masses (fuel included) = 6.5 kg wet, 4.5 kg dry.
dry, fuel = 0.5 + 1.0 + 3.0, 1.0 + 1.0
m0, mf = dry + fuel, dry
# thrust model (L989-1110): F = 0.01 kg/s * 40492 m/s = 404.9 N per engine,
# consuming 0.01/60 kg H2 AND 0.01/60 kg LOX per tick (consp_factor=60, L945),
# at 50 ticks/s (dt = 1/50, L2127):
F_tick = 0.01 * 40492                  # N, applied per tick
dt = 1.0 / 50.0
mdot = 2 * (0.01 / 60) * 50            # kg consumed per second
ve_eff = (F_tick * dt) / (2 * (0.01 / 60))   # impulse per tick / mass lost per tick
actual = ve_eff * math.log(m0 / mf)
displayed = 10123 * math.log(m0 / (m0 - fuel / 2))  # getDeltaV(): old ve, fuel/2
print("\n== ship delta-v budget ==")
print(f"  mass = {m0} kg wet ({dry} dry + {fuel} propellant)")
print(f"  thrust model: F = {F_tick:.1f} N per tick, mdot = {mdot:.4f} kg/s -> ve_eff = {ve_eff:.0f} m/s")
print(f"  ENGINE BUDGET (thrust model, both propellants): {actual:6.0f} m/s   <- what the engine can deliver")
print(f"  DISPLAYED getDeltaV() (ve=10123, fuel/2):       {displayed:6.0f} m/s   <- VESSEL window")
print(f"  => displayed budget is ~{actual/displayed:.1f}x lower than the engine can actually deliver")
