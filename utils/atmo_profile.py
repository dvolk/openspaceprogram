#!/usr/bin/env python3
"""What a shipped atmosphere actually DOES aloft -- printed, with a band (#682, #685).

A body's air top is not a rendering decision: top() sets shellEdge = max(100 km,
1.2 top), so one number moves the rotating frame, the LowOrbit/HighOrbit bands, every
spawn bed and the sky rim with it. And the top is where a satellite starts to decay, so
the thing worth QUOTING about a top is the dynamic pressure a hull meets there. Until
now that number was derived by hand, in gitignored scratch, per decision -- which is how
#682 shipped on a pre-#683 number and turned out to put 2.7x more drag at Kerbin's edge
than its own rationale claimed.

This is a SECOND implementation of atmo.h's law, on purpose: T piecewise linear in
GEOMETRIC altitude, p by QUADRATURE of -dp/dz = rho g(z) with g(z) = g R^2/(R+z)^2.
The C++ uses closed forms (atmo.h:levelPressure, #683); quadrature is what proves a
closed form is the same integral. It agrees with the loader to the digits the pins
carry -- `--heights 0,5000,8500,69999 ksp_system.json` against check_air's `aloft`
table in tests/test_scenarios.cpp, Kerbin:

    p(0)        101324.8548 Pa      pinned 101324.85477637536
    rho 5 km    0.7386735 kg/m3     pinned 0.7386735114
    rho 8.5 km  0.5019472 kg/m3     pinned 0.5019472257
    rho 69999 m 2.027780e-4 kg/m3   pinned 2.0277803e-4

which is also the answer to "can anyone reproduce the ten-digit air numbers, or only
whoever happened to have the scratch file open". Where the two disagree by more than a
rounding, one of them is wrong, and the pins are the tiebreaker. This tool PRINTS and
never refuses: a second spelling of a law is evidence, not a gate.

Usage:  python3 utils/atmo_profile.py [system.json ...]
        python3 utils/atmo_profile.py --heights 260.8,5000,8500 ksp_system.json
"""
import json, math, sys, os

# src/constants.h and src/atmo.h. A guard that checks a field with a different
# constant approves a world the loader then builds differently.
G = 6.674e-11
K_GAS_R = 8.314462618

# atmo.h:gasSpecies, duplicated here deliberately: this file cross-checks the loader,
# so it must not import the loader's arithmetic. (name: molar_mass [kg/mol], gamma)
SPECIES = {'N2': (0.0280134, 1.40), 'O2': (0.0319988, 1.40), 'CO2': (0.0440095, 1.30),
           'H2': (0.00201588, 1.40), 'He': (0.0040026, 1.667), 'CH4': (0.0160425, 1.32),
           'H2O': (0.0180153, 1.33), 'Ar': (0.039948, 1.667)}
DRY_AIR = (('N2', .78084), ('O2', .20946), ('Ar', .00934), ('CO2', .00033))


def read_system(path):
    with open(path) as f:
        return json.load(f)


def mix(constit):
    """(M [kg/mol], gamma) from the authored mole fractions -- atmo.h:resolveMix.

    The fractions are NORMALISED by their own total, and that is not rounding noise:
    dry air's four rows sum to 0.99997, so M comes out 0.003 % higher than a raw sum,
    which is 3 Pa on Kerbin's datum pressure -- the fourth digit, which is where the
    pins in tests/test_scenarios.cpp live. gamma comes from the mole-weighted molar
    heat capacity (cp_i = R g_i/(g_i-1)), never from averaging gammas."""
    sp = DRY_AIR if not constit else [(c['gas'], c['fraction']) for c in constit]
    tot = sum(f for _, f in sp)
    M = sum(SPECIES[g][0] * f for g, f in sp) / tot
    cp = sum(K_GAS_R * SPECIES[g][1] / (SPECIES[g][1] - 1.0) * f for g, f in sp) / tot
    return M, cp / (cp - K_GAS_R)

# The band a top wants, in Pa of dynamic pressure on a hull in a circular orbit THERE
# (#682's question: where does the air end for something that flies past it). The
# reference is Kerbin's shipped edge: 97.8 Pa, which brings the yardstick hull
# res/ships/skiff.json down in roughly 13 orbits (integrated in scratch when the 85 km
# top was picked -- an order of magnitude, not a pin). Two decades is the slack a
# catalogue this varied gets: below ~10 Pa a top is a fiction -- nothing decays there,
# so the "air ends" line is drawn where nothing can see it; above ~1 kPa the top is a
# wall -- a hull at that altitude is not orbiting, it is flying, and one metre higher
# it is in vacuum.
Q_BAND = (10.0, 1000.0)

DEFAULTS = ['res/systems/ksp_system.json', 'res/systems/solar_system.json']
HEIGHTS = [0.0, 5000.0, 20000.0, 50000.0]


class Profile:
    """The shipped profile re-integrated from scratch. `levels` are GEOMETRIC rows
    (alt, T, lapse) or empty for a scale-height body; `top` is the air ceiling."""

    def __init__(self, body):
        su = body.get('surface') or {}
        at = su.get('atmosphere') or {}
        self.name = body['name']
        self.radius = float(body.get('radius', 600000.0)) + float(su.get('sea_level', 0.0))
        self.g = float(body['g'])
        self.mu = G * float(body['mass'])
        self.rho0 = float(at['sea_level_density'])
        self.M, self.gamma = mix(at.get('composition'))
        self.Rs = K_GAS_R / self.M
        self.levels = at.get('layers') or []
        self.top = float(at['height']) if 'height' in at else 10.0 * float(at['scale_height'])
        self.H = at.get('scale_height')

    def T(self, z):
        if not self.levels:
            return self.g * self.H / self.Rs        # one exponential: isothermal by law
        L = self.levels[0]
        for k in self.levels:
            if z >= k['alt']:
                L = k
        return L['T'] + L['lapse'] * (z - L['alt'])

    def p(self, z, steps=6000):
        p = self.rho0 * self.Rs * self.T(0.0)       # p0 = rho0 (R/M) T0, no authored p
        x, h = 0.0, z / steps
        for k in range(steps):
            xm = x + 0.5 * h
            gz = self.g * self.radius * self.radius / ((self.radius + xm) ** 2)
            p *= math.exp(-(gz / (self.Rs * self.T(xm))) * h)
            x += h
        return p

    def rho(self, z):
        return self.p(z) / (self.Rs * self.T(z))

    def vcirc(self, z):
        return math.sqrt(self.mu / (self.radius + z))

    def q(self, z):
        return 0.5 * self.rho(z) * self.vcirc(z) ** 2

    def rows(self, heights):
        out = []
        for z in heights:
            if z > self.top:
                continue
            out.append((z, self.T(z), self.p(z), self.rho(z), self.q(z)))
        return out


def report(path):
    doc = read_system(path)
    print('\n=== %s' % path)
    for b in doc['bodies']:
        su = b.get('surface') or {}
        at = su.get('atmosphere')
        if not at or 'sea_level_density' not in at:
            continue
        pr = Profile(b)
        shape = ('%d levels' % len(pr.levels)) if pr.levels else ('H %g m' % pr.H)
        print('%-10s R %9.0f m  g %8.4f  rho0 %7.4f  %-11s top %9.0f m'
              % (pr.name, pr.radius, pr.g, pr.rho0, shape, pr.top))
        print('    datum: T %7.2f K  p %12.4f Pa  a %7.2f m/s   (M %.8f, gamma %.5f)'
              % (pr.T(0.0), pr.rho0 * pr.Rs * pr.T(0.0),
                 math.sqrt(pr.gamma * pr.Rs * pr.T(0.0)), pr.M, pr.gamma))
        for z, T, p, rho, q in pr.rows(HEIGHTS):
            print('    %9.0f m: T %7.2f K  p %12.4f Pa  rho %.6e  q %11.3f Pa' % (z, T, p, rho, q))
        qt = pr.q(pr.top)
        lo, hi = Q_BAND
        flag = '' if lo <= qt <= hi else '   <-- OUT OF BAND (%g..%g Pa)' % (lo, hi)
        print('    top %8.0f m: rho %.4e  vcirc %8.1f m/s  q %11.3f Pa%s'
              % (pr.top, pr.rho(pr.top), pr.vcirc(pr.top), qt, flag))


def main(argv):
    args = argv[1:]
    if '--heights' in args:
        i = args.index('--heights')
        global HEIGHTS
        HEIGHTS = [float(x) for x in args[i + 1].split(',')]
        del args[i:i + 2]
    if '--help' in args:
        print(__doc__)
        return 0
    paths = [p for p in args if not p.startswith('-')] or DEFAULTS
    for path in paths:
        if not os.path.isabs(path) and not os.path.exists(path):
            path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), path)
        report(path)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
