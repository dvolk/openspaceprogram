# The shipped airframes: what they are FOR

Every aero case in `e2e/cases/flight/` has been re-baselined after every law change,
which means the suite has been pinning incidents rather than a design. This file is
the design. A law change is judged against these numbers, not against last week's.

All beds start in the air (`flying-cruise` / `flying-slow`, `--sim-steps`), so there
is no takeoff constraint: no wheels, no runway, no thrust-to-weight floor.

## As built, 2026_11_01 (measured, not intended)

| | mass | main wing | loading | start | L/D at cruise | CdA at cruise |
| --- | --- | --- | --- | --- | --- | --- |
| skiff | 552 kg | 21.6 m2, span 12 m, AR 6.7 | 25.6 kg/m2 | 76 m/s at 7 km | **2.8** | hull 0.66 + profile 1.10 + induced 0.15 |
| jet | 2992 kg | 16 m2, span 8 m, AR 4.0 | 187 kg/m2 | 173 m/s at 7 km | **~1.1** | hull 2.65 + profile 0.86 |
| glider | 2887 kg | 16 m2, span 8 m, AR 4.0 | 180 kg/m2 | 173 m/s at 7 km | **0.08** | hull 2.65 + profile 0.72 |

Two facts explain all of it:

1. **The hull is a stack of barrels.** A capsule (Cd 0.35 nose-first, 1.3 base-first)
   plus a fuel tank (1.0) plus a shrouded engine (0.9) plus a reaction wheel (1.1),
   each billed on its own projected area. The jet's 3.14 m2 disc comes out at a mean
   Cd of 0.84; a real 2 m fuselage is 0.10.
2. **The wings are flat plates.** The authored profile cd is 0.03. The generator has
   always documented the smooth-section value as 0.006-0.013; at 21.6 m2 of planform
   0.03 alone is 1.10 m2 of CdA on the skiff.

The "glider" is not a glider: a 2 m space capsule (1150 kg) + a 500 kg rocket engine
+ a 100 kg reaction wheel with two 8 m2 panels bolted on, 180 kg/m2.

## Design point (targets)

| | analogue | mass | wing | loading | cruise | L/D | stall at 7 km |
| --- | --- | --- | --- | --- | --- | --- | --- |
| skiff | light utility plane (Cessna 152 class) | ~550 kg | 21.6 m2, AR 6.7 | 26 kg/m2 | 76 m/s | **>= 10** | 26 m/s |
| jet | fast trainer (L-39 class) | ~2500 kg | 16 m2, AR 4.0 | 155 kg/m2 | 170 m/s | **>= 7** | 60 m/s |
| glider | motorglider | ~460 kg | 16 m2, AR 4.0 | 29 kg/m2 | 75 m/s | **>= 10** | 24 m/s |

The stall figures are what the loading and CLmax 2.1 already give at 7 km (Kerbin's
air is 0.35 kg/m3 there). The glider's AR 4 is the limitation: a real sailplane is
AR 20+, and that needs a long-wing part, not a law change.

## THE RECIPE (measured end to end, 2026_11_01)

Attempting the redesign showed every one of these is needed; the numbers are what
the runs gave, so the next pass is mechanical rather than exploratory.

1. **Fuselage part.** `utils/gen_fuselage.py <dst> --radius R --height H` makes a
   watertight spindle (ogive nose at +Z, parallel midbody, boat-tail). Catalogue
   entry: type `fuselage`, Cd `forward 0.10 / side 0.9 / backward 0.20`, declared
   dry mass (130 kg r0.4x4, 180 kg r0.5x3.5, 520 kg r1x6), `crew_capacity`, and
   `capacity = 0.5 x bounding volume x JET_FUEL_DENSITY` (the body is the tank).
2. **Smooth section.** wing/rudder/elevator/aileron `drag_forward`/`drag_backward`
   0.03 -> **0.010**. Engine nacelle (`jet`) `drag_forward` 0.9 -> **0.15** (a
   shrouded duct; the BACK anchor stays 0.9, which is what a rocket ascent sees).
3. **Wing station is a design input.** Measured on the rebuilt skiff: wings at the
   station the old barrel stack implied (0.55 m AHEAD of the new pod centre) put the
   wing 0.7 m ahead of the centre of mass, the static margin went negative, and the
   airframe tumbled inside a second. At `z = -0.5` on the pod it flies level.
   The old stacks had been putting the wing behind the CG by accident.
4. **The jet needs incidence.** A symmetric section at AoA 0 makes no lift; the
   shipped jet "cruises" by falling, which is why its L/D reads ~1. Measured: wings
   at `z = -1.5` on the r1x6 pod and **0.5 deg of elevator trim** gives AoA 1.8-2.2,
   SM +0.17, 176 m/s, L/D ~10.
5. **The glider needs a tail and a lighter airframe.** A glider with no pitch
   surface cannot trim: it flew at AoA 0 with L = 0. With the r0.4x4 pod and a
   `jet_r0.25h1`, wings at `z = -1.0`, and an `elevator_r0.8h0.7` pair trimmed 6 deg,
   it flies 74 m/s, SM +0.34, 0.5 Nm residual torque, L/D ~13.
   (Wing `roll` with the SAME sign on both panels is a dihedral break; OPPOSITE
   signs are incidence: `roll: +3 / -3` gave +2.5 deg of lift at AoA 0.)

Measured after the recipe, same beds:

| | mass | cruise | L/D | SM |
| --- | --- | --- | --- | --- |
| skiff | 626 kg | 76 m/s | **8.5** | +0.39 |
| jet | 2428 kg | 176 m/s | **~10** | +0.17 |
| glider | 461 kg | 74 m/s | **~13** | +0.34 |

The skiff is the one short of target (8.5 vs 10). Its induced bill is 0.17 m2 at
cruise and the profile 0.42 -- the next lever is span (AR 6.7 -> 9), not the law.

## Landed (2026_11_01)

All of the above is shipped: the fuselage family, the smooth section, the nacelle Cd,
the rebuilt airframes, the re-pinned `test_stability`, and the aero cases rewritten
against the design point. Measured after, same beds:

  skiff   626 kg, 76 m/s, AoA 3.0-3.3, SM +0.39, L/D 8.5
  jet    2428 kg, 176 m/s, AoA 1.8-2.2, SM +0.20, L/D ~10
  glider  434 kg,  80 m/s, AoA 2.1,     SM +0.49, L/D ~13

Two design fixes came out of the re-pin rather than a number:

  - the jet's two rudders were 2 m2 each -- four times a trainer's fin. At 176 m/s a
    held pedal drove it to 118 deg of sideslip and through a tumble. Swapped to the
    1.2 m span / 0.48 m2 control surface; full pedal now takes it past 90 deg and it
    recovers (flight/453 pins the direction and the size, not a recovery).
  - flight/80 (the glider's Duna-pad AoA sweep) lost its premise: it pinned a glider
    that could not trim. The rebuilt one trims, so the case is DELETED rather than
    widened. flight/458-glider now carries the stall-and-recover and yaw-swing story.

## The next question the envelope cases now ask

flight/458-glider and flight/453 both show the same gap: a vertical surface makes NO
side force from sideslip, only from deflection, so the fins do not weathervane -- the
hull's own drag is the only yaw damping on either airframe. That is #454's other half
and the obvious next law change: a fin's lift from beta, which every player-built
aircraft needs.
