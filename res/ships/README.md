# The shipped airframes: what they are FOR

Every aero case in `e2e/cases/flight/` has been re-baselined after every law change,
which means the suite has been pinning incidents rather than a design. This file is
the design. A law change is judged against these numbers, not against last week's.

All beds start in the air (`flying-cruise` / `flying-slow`, `--sim-steps`), so there
is no takeoff constraint: no wheels, no runway, no thrust-to-weight floor.

## As built, 2026_11_01 (measured, not intended)

| | mass | main wing | loading | start | L/D at cruise | CdA at cruise |
| --- | --- | --- | --- | --- | --- | --- |
| skiff | 552 kg | 21.6 m2, span 12 m, AR 6.7 | 25.6 kg/m2 | 76 m/s at 7 km | **2.8** | hull 0.66 + profile 1.10 + induced 0.15 = 1.91 |
| jet | 2992 kg | 16 m2, span 8 m, AR 4.0 | 187 kg/m2 | 173 m/s at 7 km | **~1.1** | hull 2.65 + profile 0.86 = 3.51 |
| glider | 2887 kg | 16 m2, span 8 m, AR 4.0 | 180 kg/m2 | 173 m/s at 7 km | **0.08** | hull 2.65 + profile 0.72 = 3.37 |

Two facts explain all of it:

1. **The hull is a stack of barrels.** A capsule (Cd 0.35 nose-first, 1.3 base-first)
   plus a fuel tank (1.0) plus a shrouded engine (0.9) plus a reaction wheel (1.1),
   each billed on its own projected area. The jet's 3.14 m2 disc comes out at a mean
   Cd of 0.84; a real 2 m fuselage is 0.10.
2. **The wings are flat plates.** The authored profile cd is 0.03. The generator has
   always documented the smooth-section value as 0.006-0.013; at 21.6 m2 of planform
   0.03 alone is 1.10 m2 of CdA on the skiff.

The "glider" is not a glider: it is a 2 m space capsule (1150 kg) + a 500 kg rocket
engine + a 100 kg reaction wheel with two 8 m2 panels bolted on, 180 kg/m2.

## Design point (targets)

| | analogue | mass | wing | loading | cruise | L/D | stall at 7 km |
| --- | --- | --- | --- | --- | --- | --- | --- |
| skiff | light utility plane (Cessna 152 class) | ~550 kg | 21.6 m2, AR 6.7 | 26 kg/m2 | 76 m/s | **>= 10** | 26 m/s |
| jet | fast trainer (L-39 class) | ~3000 kg | 16 m2, AR 4.0 | 187 kg/m2 | 170 m/s | **>= 7** | 71 m/s |
| glider | motorglider | ~600 kg | 16 m2, AR 4.0 | 38 kg/m2 | 50 m/s | **>= 10** | 32 m/s |

The stall figures are what the wing loading and CLmax 2.1 already give at 7 km
(Kerbin's air is 0.35 kg/m3 there). The glider's AR 4 is the limitation: a real
sailplane is AR 20+, and that needs a long-wing part, not a law change.

## The fuselage part (built, measured, not yet shipped)

`utils/gen_fuselage.py` generates a spindle (ogive nose, parallel midbody,
boat-tail) and the plan is one body per airframe with Cd 0.10 nose-first instead of
a stack whose mean is 0.84. The part was added to the catalogue as
`fuselage_r0.4h4` / `_r0.5h3.5` / `_r1h6` (crew, declared dry mass, half the
bounding volume as jet fuel), and the three airframes were rebuilt around it.

Measured on the rebuilt skiff (`flying-cruise`, 626 kg with fuel):

- at the wing station the old stack implied (ahead of the pod's centre) the airframe
  is **pitch-unstable** and tumbles inside a second: the wings sat 0.7 m AHEAD of the
  centre of mass and the tail volume could not recover it. `SM` went negative.
- with the wings moved aft (`z = -0.5`, i.e. 0.5 m behind the pod's centre) it flies
  level at **76 m/s, L/D 8.6, SM +0.39, 9 Nm of residual torque** -- close to the
  design point, with the smooth section included.

So the fuselage is sound; the **wing station is a design input that no shipped ship
had ever needed**, because the old stacks put the wing behind the centre of mass by
accident. The jet and the glider need the same treatment, and the jet additionally
needs incidence or tail trim: at AoA 0 a symmetric section makes no lift, which is
why the shipped jet "cruises" by falling.

## The changes this implies

1. Ship the fuselage part (Cd 0.10, crew, fuel capacity) and rebuild all three
   airframes around it, with the wing station chosen for a positive static margin.
2. Ship the smooth section (profile cd 0.010).
3. Give the jet incidence (or a trimmed tail) so level flight is possible at cruise
   AoA, and rebuild the glider light (r0.4 pod, no reaction wheel, the small jet).
4. Re-baseline the aero suite ONCE against the design point, and add a design-point
   case that checks L/D, cruise speed and stall margin instead of an attitude.

Predicted (and partly measured): skiff CdA ~0.42 -> L/D ~8.6, jet ~0.76 -> L/D ~7.2.

## How the next step should be verified

`flight/490` (skiff) and `flight/492` (skiff_trim) carry the cruise envelope;
`flight/458-jet` and `flight/458-glider` carry the handling envelopes. The new pins
are L/D, cruise speed and stall margin; the old attitude pins stay as incidental
colour, not as the contract.
