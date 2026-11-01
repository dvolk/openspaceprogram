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
3. **Incidence.** A symmetric section at AoA 0 makes no lift, so an airframe with no
   incidence and no trim "cruises" by falling. The skiff trims by wing station; the
   jet needed 0.5 deg of elevator trim.
4. **Tail.** Pitch: a tailplane pair (a `wing_r1h1` pair) with elevators behind it.
   Yaw: a `rudder_r*`. Both are needed; a glider with no pitch surface cannot trim.
5. **Section**: profile cd 0.010 (smooth), not 0.03 (the old faceted plate).
6. Then measure and pin: L/D, cruise speed, static margin, stall margin.

## What the envelope cases taught

- **Fins must answer sideslip** (a control surface reads its own alpha as well as the
  hinge) or nothing weathervanes. With it, the jet's full-pedal departure became a
  28 deg swing that recovers, and its 88 deg stall-and-recover pull became 15 deg.
  Gated on `LiftingSystem::liftArea` so a flap on a wing does not double-count.
- **Elevators cannot roll.** Roll is reaction-wheel only today -- a real gap, filed
  as the top of the work order (#563): planes need ailerons and a roll axis.
- **One lifting system pays one induced bill** (#556): two co-planar panels are one
  AR-4 wing, a bridge (the fuselage) joins roots that do not touch, and a tailplane
  three metres aft is a different system.
