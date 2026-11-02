# The shipped airframes: what they are FOR

Every aero case used to be re-baselined after every law change, which means the suite
was pinning incidents instead of a design. This file is the design. A law change is
judged against these numbers, not against last week's. History: `git log`, tickets.

All flight beds start in the air (`flying-cruise` / `flying-slow`, `--sim-steps`), so
there is no takeoff constraint: no wheels, no runway, no thrust-to-weight floor.

## The ship: skiff (the template)

| | |
| --- | --- |
| analogue | light utility plane (Cessna 152 class) |
| hull | `capsule_r0.5h1.5` at the root, `jet_tank_r0.4h2` stacked 1.75 m aft of it, engine 3.25 m aft |
| mass | 559 kg (462 kg of parts with the tank full + one 97 kg kerbal; the pod seats one) |
| wing | 21.6 m2 authored, span 12 m, AR 6.7, bolted to the TANK, 1.2 m aft of the pod's centre |
| as the model groups it | one 22.7 m2 system at AR 6.34 (the two wings plus the ailerons hung on their trailing edge), one 5.1 m2 tail system at AR 3.12, fin and rudder each their own (`OSPAEROSYS=1`, #556/#570) |
| loading | 26 kg/m2 |
| cruise | 46..49 m/s at 7 km once the phugoid damps (#585), +5.1 deg of angle of attack |
| glide | sinks 7.4 m/s unpowered at that speed |
| L/D | 7.5 (target >= 10; the levers are span -- AR 6.7 -> 9, #567 -- and the hull bill below) |
| static margin | +0.58 m |
| roll | aileron pair on the wing, `roll` axis (563-roll-ailerons), and the pod's 100 N m wheel |
| stall (7 km) | 24 m/s |
| bill at cruise | body 0.59 x 0.82 m2, wing profile 0.58 m2, induced 0.49 m2 |

Those are `skiff_trim.json`'s numbers (`skiff.json` plus `"trim": 12` on both elevators
and nothing else), measured 2026_11_02 on `flying-cruise` with `--drag-log` and no
stick: once the phugoid damps -- `t > 170 s` of a 266 s run -- it holds AoA 5.1 deg at
46..49 m/s, sinks 7.4 m/s and bills L/D 7.52 in every row. The pinned case
(`flight/492-elevator-trim`: 50 s at `--time-accel 1`, throttle opened at t=25 s) is the
same hold caught before it damps: AoA 5.0..5.9 deg, L/D up to 7.6, speed swinging
29..78 m/s against 7.0..7.4 km of height. `skiff.json` itself -- no trim -- flies this
hull's zero-moment AoA of +2.6 deg: 69..80 m/s sinking 16 m/s, and only L/D 4.9, because
at that speed the body bill and not the wing pays the drag. So the 12 deg of tab is not
only a pitch offset, it is the difference between this hull's worst and best glide.
#570 then hung the control surfaces off the trailing edges they hinge on instead of
burying them 0.4-0.5 m inside those surfaces: worth +0.04 of L/D and -0.07 m/s of sink,
which is not why it was worth doing -- a plate inside another plate is a thing the
player can see and the law has no word for. What the airframe still leaves open is
span (#567), not the tab or the tail: the tailplane pair stands 1.65 m behind the wing
where the spindle's stood 0.85 m, so the same 12 deg buys roughly twice the pitch
authority -- which is why this hull sits 2 deg nose-high at the same setting.

The stall row is arithmetic off the same law, not a flight measurement: 5.36 kN of
weight at 7 km (559 kg x 9.81 x (600/607)^2) against the airframe's 153.6 m2/rad of lift
slope at the shipped wing's 20.1 deg stall angle, in 7 km air (rho 0.346). The old row's
26 m/s is the same expression with 633 kg in it, which is why the two agree to a m/s.

**What the swap costs, measured on both hulls** (same bed, same `--drag-log`, each with
`"trim": 12` on its elevators and read once its phugoid is damping;
`git show a18b0d9^:res/ships/skiff.json` is the pod-on-tank shape this went back to,
`012434c` the spindle it replaces):

| hull | mass | AoA | glide speed | sink | L/D | SM | body bill Cd x A |
| --- | --- | --- | --- | --- | --- | --- | --- |
| one `fuselage_r0.5h3.5` spindle | 633 kg | +3.1 | 64..70 m/s | 10.4 m/s | 7.91 | +0.73 | 0.11 x 0.87 = 0.10 m2 |
| pod on a `jet_tank_r0.4h2` | 559 kg | +5.1 | 46..49 m/s | 7.4 m/s | 7.52 | +0.58 | 0.59 x 0.82 = 0.48 m2 |

The pod row is the shipped hull re-read after #570 (its buried control surfaces made
this bed 7.48 / +0.586); the spindle row is as it was measured for the swap.

All of that gap is coefficient, not shape: the two hulls present almost the same
silhouette (0.82 vs 0.87 m2) and differ fivefold in what it is charged. Nose-on, the
spindle's authored `drag_forward` 0.1 bills a union Cd of 0.11, while the stack's
capsule 0.35, tank 1.0 and engine 0.15 area-weight to 0.59. The tank's 1.0 is a flat
end charged as a flat plate, which is what #558 is about (2 m long on a 0.8 m diameter
is not a flat plate); #514/#491 own the union area and the force point per part. This is
NOT a reason to author a lower Cd: the stack is what a player builds, so the number it
pays is the honest baseline. What it buys: a cockpit with a seat, 2000 Wh of charge,
propellant in
a part that is not also the crew compartment (staging a `fuselage_r*` away throws the
crew out with the fuel), the pod's 100 N m reaction wheel -- fuselage parts carry no
`torque`, so the spindle had no wheel at all -- and 74 kg less to carry, which is why
the heavier-looking hull sinks 3 m/s SLOWER in the same air.

The jet and the glider were deleted (2026_11_01): they were rockets with wings bolted
on, and their envelope cases cost more to keep than they were worth. A new airframe
should be built from this recipe and get its own row here.

## The recipe for a new airframe (measured, so it is mechanical)

1. **Body: a pod on a tank.** `capsule_r0.5h1.5` (crew, charge, a 100 N m wheel) with a
   `jet_tank_r*` stacked on it -- `attach: "down"`, so the tank's top node mates the
   pod's bottom one -- and the engine at the back of the tank. That is the shape a
   player builds and the shape every other window reads: crew, charge, propellant and
   staging all live on pods and tanks. A `fuselage_r*` (the smooth spindle from
   `utils/gen_fuselage.py`, Cd 0.10 nose-first, crew, declared dry mass, half the
   bounding volume as jet fuel) is the smooth ALTERNATIVE, not the default: it flies
   better and it is a thing nobody would build. A body MAY be several parts either way:
   same-cd parts average in the area-weighted mean, and the silhouette unions, so a
   stack bills correctly -- it just bills more. Segments are a content job (see the
   ticket), not a law one.
2. **Wing station is a design input**, and the wings go on the TANK -- a wing belongs to
   what it is bolted through (#565), and that is where the ailerons get their
   trailing-edge nodes (#566). The old barrel stacks put the wing behind the centre of
   mass by accident; a one-piece body put it 0.7 m ahead and the airframe tumbled
   inside a second. On the skiff, tank-local `z: +0.55` puts the wing 1.2 m aft of the
   pod's centre and the COM 0.32 m ahead of the wing: SM +0.58 m.
3. **Incidence and trim.** A symmetric section at AoA 0 makes no lift, so an airframe
   with neither incidence nor trim "cruises" by falling. The wing station does NOT trim
   -- it sets stability: the skiff's zero-moment AoA is 2.6 deg while its trimmed hold
   is 5.1 deg, and the tailplane pair is authored at 5 deg of `roll` incidence on top of
   that. `skiff_trim.json` is that same hull with `"trim": 12` on both elevators and
   nothing else changed, and once its phugoid damps it holds 5.1 deg of AoA gliding at
   46..49 m/s, sinking 7.4 m/s; hands off, the hull flies 69..80 m/s at 2.6 deg and
   sinks 16 m/s. So trim is either authored incidence or a held elevator setting,
   decided from a measurement -- never a launch condition: a stable hull rotates to its
   own zero-moment AoA inside a second whatever attitude it spawns in (#569).
4. **Tail.** Pitch: a tailplane pair (a `wing_r1h1` pair) with elevators behind it.
   Yaw: a `rudder_r*`. Both are needed; a glider with no pitch surface cannot trim.
   Hang a control surface off the TRAILING EDGE of the surface it hinges to, with a
   surface `point` plus `normal` -- not with a `z` that slides it inside the part it is
   bolted through (#570). The tree parent need not be that surface: the skiff's rudder
   is a child of the ENGINE and hangs off the FIN's trailing edge at 3.95 m. The line
   to hold is the hinge: two plates may MEET along one line (aileron and wing touch at
   -2.10, rudder and fin at -3.55) and a tab may overlap a little (the elevators stand
   0.05 m behind the tailplanes, 7% of their chord), but a chord that vanishes inside
   another plate has no hinge the law could honor if the lift model ever wanted one.
   The skiff's ailerons used to sit 0.5 m of their 0.7 m chord inside the wing and the
   rudder half its chord inside the fin. Measure before believing the fix matters to
   the air: the grouping merges a flap into the surface it is bolted through either way
   -- four lifting systems before #570, four after.
5. **Section**: profile cd 0.010 (smooth), not 0.03 (the old faceted plate).
6. Then measure and pin: L/D, cruise speed, static margin, stall margin.

## What the envelope cases taught

- **Fins must answer sideslip** (a control surface reads its own alpha as well as the
  hinge) or nothing weathervanes. With it, the jet's full-pedal departure became a
  28 deg swing that recovers, and its 88 deg stall-and-recover pull became 15 deg.
  Gated on `LiftingSystem::liftArea` so a flap on a wing does not double-count.
- **Elevators cannot roll.** That was the gap behind #563, now closed the other way: a
  part declaring `control_axis: roll`, hinged on the wing's trailing edge, rolls the
  hull. On `flying-cruise` with the stick held 1-3 s, 0.40 rad/s (23 deg/s) of the
  hull's rate is roll about the nose and the bank reaches 44 deg by t=3 s -- reproduce
  with `--att-log --sim-press 1000,2000,Q` and dot `w` onto `nose`. The pair's own
  leverage (8.91 kN m of roll, with 240 N m of pitch alongside it) is on #566. Two
  lessons lived in the mount itself: the child's `normal` is what lays the plate flat
  (`normal: [1,0,0]`, spanwise, works; `[0,1,0]` stood it on edge and it still answered
  with 1.33 kN m while its hinge showed full travel -- rig travel is not authority,
  #566/#509), and until #568 the shipped skiff had NO reaction wheel at all, because
  fuselage parts carry no `torque`: on that hull the aero row WAS the whole attitude
  budget (`[slew] alpha=0.000`). The pod hull declares 100 N m, which is 1% of the
  8.91 kN m #566 measured from this aileron pair at full deflection -- decisive for
  pointing a ship out of atmosphere, negligible for rolling this one in it. That pair
  has since moved 0.5 m aft (#570) and nothing in the repo re-measures the figure: the
  roll half is priced off the spanwise station, which did not move, so read 8.91 kN m
  as #566's pair at #566's station.
- **One lifting system pays one induced bill** (#556): two panels side by side are one
  AR-4 wing, a bridge (the fuselage) joins roots that do not touch, and a tailplane
  three metres aft is a different system. #570 makes the merge test explicit: a pane
  joins a system only if its span runs across the airframe AND its chord fore-and-aft,
  and its spanwise and chordwise intervals then touch or bridge (`drag.h`, the
  `aLateral` guard). There is no coplanarity test -- the aileron sits 0.02 m off the
  wing's mid-plane and merges anyway -- and a pane whose span runs vertical is never a
  candidate, which is why the fin and the rudder are each their own system even though
  #570 left the rudder touching the fin along exactly one line.
