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
| mass | 666 kg (180 kg fuselage + 96 kg of jet fuel + crew + wings + ailerons) |
| wing | 21.6 m2, span 12 m, AR 6.7, at 0.5 m aft of the pod's centre |
| loading | 29 kg/m2 |
| cruise | 76 m/s at 7 km, +3.0..3.3 deg of angle of attack |
| L/D | 8.5 (target >= 10; the next lever is span, AR 6.7 -> 9) |
| static margin | +0.39 m |
| roll | aileron pair on the wing, `roll` axis (563-roll-ailerons) |
| stall (7 km) | 26 m/s |
| bill at cruise | body 0.11 x 0.87 m2, wing profile 0.42 m2, induced 0.17 m2 |

That row describes a TRIMMED hull, and as of 2026_11_02 the file that meets it is
`skiff_trim.json` (3.1..3.2 deg AoA, 75..85 m/s, steady altitude on `flying-cruise`),
not `skiff.json` (1.5 deg, descending). Two other rows are stale the same way: the mass
(666 kg authored, 633 kg measured on the shipped hull, which is the 6.2 kN of weight
quoted below) and the static margin (+0.39 m was the pod-on-tank shape; the shipped
spindle measures +0.73 m). #568 puts the pod back and this table gets re-stated from
that measurement.

The jet and the glider were deleted (2026_11_01): they were rockets with wings bolted
on, and their envelope cases cost more to keep than they were worth. A new airframe
should be built from this recipe and get its own row here.

## The recipe for a new airframe (measured, so it is mechanical)

1. **Body**: one `fuselage_r*` part (a smooth spindle from `utils/gen_fuselage.py`,
   Cd 0.10 nose-first, crew, declared dry mass, half the bounding volume as jet
   fuel). A body MAY be several parts: same-cd parts average in the area-weighted
   mean, and the silhouette unions, so stacking fuselage segments bills correctly.
   Segments are a content job (see the ticket), not a law one.
2. **Wing station is a design input.** The old barrel stacks put the wing behind the
   centre of mass by accident; a one-piece body put it 0.7 m ahead and the airframe
   tumbled inside a second. 0.5 m aft of the pod's centre gave SM +0.39.
3. **Incidence and trim.** A symmetric section at AoA 0 makes no lift, so an airframe
   with neither incidence nor trim "cruises" by falling. The wing station does NOT trim
   -- it sets stability: as shipped the skiff has SM +0.73 m and a zero-moment AoA of
   1.6 deg, while its cruise needs 3.2 deg, so it descends on the cruise bed (t=5 6928 m
   -> t=20 6091 m at `--time-accel 1`, 2.4 kN of lift against 6.2 kN of weight) even
   though its tailplane pair is already authored at 5 deg of `roll` incidence.
   `skiff_trim.json` is that same hull with `"trim": 12` on both elevators and nothing
   else changed, and it holds the design point: 3.1..3.2 deg of AoA at 75..85 m/s,
   altitude steady. So trim is either authored incidence or a held elevator setting,
   decided from a measurement -- never a launch condition: a stable hull rotates to its
   own zero-moment AoA inside a second whatever attitude it spawns in (#569).
4. **Tail.** Pitch: a tailplane pair (a `wing_r1h1` pair) with elevators behind it.
   Yaw: a `rudder_r*`. Both are needed; a glider with no pitch surface cannot trim.
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
  #566/#509), and the shipped skiff has NO reaction wheel at all, because fuselage
  parts carry no `torque`: on this airframe the aero row IS the whole attitude budget
  (`[slew] alpha=0.000`). #568's pod hull would give it a 100 N m wheel.
- **One lifting system pays one induced bill** (#556): two co-planar panels are one
  AR-4 wing, a bridge (the fuselage) joins roots that do not touch, and a tailplane
  three metres aft is a different system.
