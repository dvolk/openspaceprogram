#include "guidance.h"
#include "vehicle.h"
#include "frame.h"
#include "terrain.h"
#include <cassert>
#include <cmath>
#include <map>
#include <algorithm>

// glm::length2 lives in gtx/norm; the experimental opt-in must precede the
// include. This file only compiled before because vehicle.h opens it first.
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>

/* Tuning constants, hoisted so the flight test can be re-flown against a
   different value with a one-line diff. Every number here is what the closed loop in
   tests/test_guidance.cpp settled on, not what looked plausible -- except kFlatAir,
   which only a hull flying through air can settle; the suite pins that one's FORM
   (one flat-over altitude, scaled by the height and the weight of the air), and its
   value comes from the flights quoted by pathAngle() below. */
static const double kFlatHalf = 0.03;  // pitch-over half-angle altitude, as a fraction of the ask
/* Flight-tuned like the line above, and still under 1 so the scheduled turn ends INSIDE
   the air it was sized from: at and above the air top lastDynPressure is exactly 0, so a
   turn that outlasts the sky is flying blind in vacuum. Capped by the ask in pathAngle(). */
static const double kFlatAir = 0.40;   // ... and as a fraction of the air OVERHEAD (#960, #962)
/* The air column (kg per m^2 above the launch point) at which a sky counts FULLY in the
   schedule above. Not flight-tuned: it is where "air that can push back" starts, and the
   authored bodies leave a wide gap around it. The roster below quotes each body's DATUM
   column (p0/g) -- what the law weighed before #962 -- because that is the figure the
   threshold was chosen against. Out: Pluto 2, Mars 171, Duna 204. In and saturated:
   Kerbin 10328, Earth 10318, Shay 11267, Eve 11904, Jool 12743, Laythe 12910,
   Uranus 11102, Titan 106000, Venus 1.04e6. Between: Jupiter 3858, Neptune 8870, Saturn
   8940, which get that fraction of the bend -- correct for what their columns are worth,
   and none of the three has a launch pad yet (#960 review).

   Reading from the hull moves some of those figures a long way: Venus has exactly one
   pad, the cloud deck at 56 km, and it arms at 5,274 kg/m^2 -- weight 0.53, no longer
   saturated. Which leaves the ceiling question unflown: whether 0.40 x the overhead is
   the right bend in air 100x Kerbin's thick is a guess this term does not argue for.

   Two honesty notes on what the hull-relative reading does and does not fix. The HEIGHT
   half now stops at the authored ceiling (overhead = top - launch altitude). The WEIGHT
   half is airPressure(launch)/g_datum -- the hydrostatic integral of the profile as
   authored -- so it still counts air above that ceiling: Pluto reads 2.11 kg/m^2 by this
   rule against 1.41 inside its own 20 km cut, which is the same 0.1-3.8% agreement the
   old p0/g reading had, five thousand-fold under the threshold. What #962 did fix is the
   other direction: a launch that starts at 56 km of a 116 km air is weighed at the 47 kPa
   still over it rather than the 9.2 MPa under it.

   And the roster's margins are not all wide. Kerbin's datum column is 10,328 kg/m^2,
   three percent over this threshold, and its shipped pad sits some 265 m up it, so what
   the law weighs at Kerbin is 10,002 for mars_explorer_mk1 and heavy_one, 10,009 for
   racer, 10,013 for kerbal -- the reading follows the hull's COM, not the pad, so the
   slack is 0.02% to 0.13%. Retune Kerbin's relief a few hundred metres and the
   flagship body's bend stops saturating and falls a percent or two. That is not a bug
   in the measurement; it says the threshold's placement is untested in exactly the band
   where the flagship body lives, and #978 owns the decision (move the line into the
   roster's empty 204..3858 band, or make the weight smooth rather than a step). */
static const double kAirColumnFull = 10e3;
static const double kLeadFrac = 0.5;   // node burn starts this fraction of its length early
/* #941: how long the law waits before asking for a press it was refused. Logic
   ticks, not seconds -- step() takes no dt -- so this is about a second of real
   time whatever the warp, which is what the caller's refusal print wants: often
   enough that a player who puts the kerbal on EVA sees the mission carry on, not
   often enough to bury the log. */
static const int kPressRetryTicks = 60;

/* The pitch schedule: a commanded PATH ANGLE against ALTITUDE. 90 deg (straight
   up) off the pad, 45 deg at h_half metres of climb, -> 0 as we climb.

   That altitude is metres of CLIMB SINCE THE LAUNCH, not height above the datum
   (#979). For every flight written down in this file the two differ by the pad's
   own height -- 270 m on Kerbin, about 130 m at moho_pad -- so the datum spelling
   hid it. A launch born inside its sky does not: at 56 km of a 116 km air, reading
   the datum put the deck 56 km INTO its own 12.7 km turn, so its first tick asked
   12.7 deg, #930's ratchet kept it there for the whole flight, and the hull sank
   below the deck with 1.4 km/s of its 1.6 km/s spent on drag.

   Two failure modes this replaced. Keyed to a fraction of the target apoapsis
   (alt / 0.25 * ask), it flew nearly vertical for 25 km and 34 s: periapsis at
   the body's CENTRE, no downrange speed, no way to circularise at any amount of
   fuel. Keyed to the live APOAPSIS instead, it jammed vertical forever: on Moho
   a 19 m/s^2 stage opens the conic by 50 km, and "no apoapsis" read as "not made
   yet" -- pointing up harder is what opened it.

   On a body with no air, what matters is being flat EARLY and LOW, because
   horizontal speed at low altitude is what buys apoapsis cheaply; that is what
   kFlatHalf * ask is for (3 km on a 100 km ask). Inside air the same shape is the
   wrong one. What the air bills is (CdA/m) * integral(1/2 rho v dt) = (CdA/m) *
   integral(1/2 rho * v ds): the PATH flown through dense air, times how fast it is
   flown. A hull whose thrust only balances drag at ~250 m/s at sea level (1.84 MN
   against CdA ~47 m^2) cannot buy apoapsis down there at any throttle -- it has to
   climb out of the air first and lean over afterwards. Hence the air term: in air,
   h_half is at least kFlatAir * the air top.

   Flown, not guessed, and the difference is the whole mission. mars_explorer_mk1 off
   Kerbin, 150 km ask, same bed, air term off: apoapsis never past 9.4 km, 11.7 km/s of
   drag, the tanks dry at t=1109 s. With kFlatAir = 0.40 -- a 34 km turn, so 74 deg of
   pitch at 10 km and 45 deg at 34 km -- the same hull reaches 150 x 159 km for 4.8 km/s
   of delta-v. Swept six values: 0.10 and 0.18 never leave the air (16 km and 31 km of
   apoapsis); 0.24, 0.30, 0.40 and 0.55 all reach space and all overshoot a 150 km ask --
   apoapsis 384, 337, 159 and 227 km -- so 0.40 ships for landing closest, not for being
   right. That is steeper than the curve MechJeb's AutoPath builds by keying its turn to
   the air's depth (0.05..0.85 of it, ~60 deg at 10 km on Kerbin); 0.18, which does
   reproduce their curve, was flown here and lost, because the hulls of this game drag far
   harder than the point-mass model their defaults are tuned against.

   The air term only ever DELAYS the flatten: kFlatHalf * ask stays the floor and
   air.overhead is 0 on a body with no atmosphere, or on one whose air is all below
   the launch point, so an airless flight -- and every shape
   pinned in tests/test_guidance.cpp -- is bit for bit what it used to be (the Moho
   control flight still ends at peri 100068 m, apo 101120 m). What it does not fix is
   the SHAPE of the answer: at these values the profile now reaches its node and
   overshoots the asked apoapsis by 6-70% depending on the ask, which is the finite-burn
   question #611 owns.

   The caller feeds this the HIGH-WATER CLIMB, not the live height (#930). Measured
   on Kerbin with mars_explorer_mk1 (reports/autopilot-v22026_11_16/927_probe_kerbin.log):
   apoapsis peaked at 6.7 km, drag took the hull back down, and the schedule asked
   for the pad's own nose again -- commanded pitch climbed from 34 deg back to 87.1
   while the ship sank, ending parked on the surface at 1 cm/s with the lever at
   1.00. A schedule over a coordinate is two-way in time; only the profile's
   history says which way the flight is going. */
static double pathAngle(double climb, double ask_alt, const AirFacts &air) {
    const double h_ask = kFlatHalf * std::max(ask_alt, 0.0);
    /* Two bounds on the air term, both measured, both found on Duna -- 50 km of sky
       over a 20 km mission.
         CAPPED BY THE ASK. Uncapped, the schedule asked for a 20 km turn where the ask
       wanted 600 m. MechJeb's AutoPath caps the same way -- min(0.85 x depth,
       targetAltitude) -- and for the same reason: the schedule's job is to reach the
       ask, and a turn that outlasts the ask has no stopping point.
         WEIGHTED BY THE AIR. The cap alone did not save Duna -- flown, the capped
       hull still ran out to 1.35 million m of apoapsis with its tanks dry -- because
       the term was sizing the turn by the LENGTH of the path through air while drag
       bills its MASS: (CdA/m) * integral(rho v ds). Duna's column is 204 kg per square
       metre against Kerbin's 10328 (p0/g, hydrostatic, from res/systems/ksp_system.json),
       so the air there bills a fiftieth of what the term pays for. A thin sky buys no
       trajectory, so it gets none: below kAirColumnFull it stops counting at all.
         MEASURED FROM THE HULL, not from the datum (#962). Both numbers here are the
       air OVERHEAD -- top minus where the launch began, and the profile's own pressure
       there -- so a deck that starts inside its own sky sizes its turn from the 60 km
       left to climb and the 5300 kg/m^2 left to push back, instead of the 116 km and
       1.04e6 kg/m^2 of the whole body. A ground pad is under all but the 270 m its own
       hull sits at, so Kerbin reads 84,730 m and 10,002 kg/m^2 instead of 85 km and
       10,328 -- still over kAirColumnFull, so the weight term does not move at all and
       the only effect is 0.4 x 270 = 108 m off a 34 km turn. That 0.3% is worth taking
       seriously, because this term is sensitive: on the datum denominator that trim
       moved the standard bed flight's peak apoapsis 1.6 km (170.0 -> 168.4 km), and
       #979 -- taking the pad's own height out of the turn's PROGRESS, the same 270 m --
       moved it 5.8% the other way. The current bed (mars_explorer_mk1, 100 km ask,
       42000 steps at 10x) peaks at 178.1 km having spent 4803.6 m/s. */
    const double weight = std::min(1.0, std::max(air.column, 0.0) / kAirColumnFull);
    const double h_air = kFlatAir * weight
                       * std::min(std::max(air.overhead, 0.0), std::max(ask_alt, 0.0));
    const double h_half = std::max(1000.0, std::max(h_ask, h_air));
    return std::atan2(h_half, std::max(0.0, climb));   // 90 deg at launch, -> 0 as we climb
}

/* The speed ceiling: how fast the ship may go at this radius and still have its
   apoapsis at the ask (treating the current point as the low end of the arc).
   v^2 = mu (2/r - 2/(r + r_ask)).

   The throttle is not decoration on this game's stages: the test ship is a racer
   -- 50 kN on 2670 kg, 19 m/s^2 against 2.7 m/s^2 of gravity -- and 1652 m/s of
   delta-v against the ~900 m/s a 100 km Moho orbit costs. At full throttle it
   spends the whole mission's speed in 74 s and leaves the SOI with an open conic
   (measured: 1485 m/s at 50 km, tanks empty). So the law steers ENERGY with the
   throttle and DIRECTION with the aim, and this ceiling makes the two agree:
   the cap falls as the ship climbs (887 m/s at the pad, 694 m/s at the ask,
   where it equals circular speed exactly), so following it IS the transfer. */
static double speedCap(double mu, double r, double ask_r) {
    const double v2 = mu * (2.0 / std::max(1.0, r) - 2.0 / (std::max(1.0, r) + std::max(r, ask_r)));
    return std::sqrt(std::max(0.0, v2));
}

/* A unit vector perpendicular to `up` that does not need the body to spin: seed
   with the coordinate least aligned with up, then remove the radial part. The
   seed can never be parallel to up, so the difference can never be the zero
   vector. The first cut seeded +x unconditionally, which at a pole whose x
   happens to be 1 leaves nothing to normalize -- and NaN then sailed through
   every gate below, because NaN < eps is false. */
static glm::dvec3 tangent(glm::dvec3 up) {
    const glm::dvec3 seed = (std::fabs(up.x) < 0.9) ? glm::dvec3(1.0, 0.0, 0.0)
                                                    : glm::dvec3(0.0, 0.0, 1.0);
    return glm::normalize(seed - up * glm::dot(seed, up));
}

void AscentLaw::setLaunchHeading(const GuidanceState &s) {
    const glm::dvec3 up = glm::normalize(s.pos);
    /* east: the direction the body's pole-line carries a point at this latitude.
       The axis is the body's POLE, not its angular velocity: a body that does not
       rotate still has a pole (Frame::spinAxisRelTo), and that is the plane the
       asked inclination is measured from. Reading omega() instead made this cross
       product zero for a non-spinning body, and the fallback below then handed
       back a RADIAL heading -- straight up, for the whole flight, with the ship
       climbing for ever and never building a plane (#610 review). */
    glm::dvec3 e1 = glm::cross(s.spin_axis, up);
    if(!(glm::length2(e1) > 1e-12)) {
        /* Standing on the pole, or no usable axis: azimuth is meaningless there.
           Take any HORIZONTAL direction and say so. */
        launch_h_ = tangent(up);
        why_ = "no launch azimuth at this site: inclination left to the launch heading";
        return;
    }
    e1 = glm::normalize(e1);
    const glm::dvec3 e2 = glm::normalize(glm::cross(up, e1));   // toward the pole
    if(target.inc_rad < 0.0) { launch_h_ = e1; return; }        // due east
    /* The inclination of a horizontal launch is set by how much of its heading
       points POLEWARD: cos(i) = cos(az_from_east) * cos(lat). e1 is east and e2
       is poleward at every latitude (e2 . spin_axis is cos(lat), positive in both
       hemispheres), so the poleward share of the heading IS cos(i)/cos(lat).
       Measured from east rather than from north because e1 falls out of the
       cross product above; the textbook "azimuth from north" is 90 deg off it. */
    const double cos_lat = std::abs(glm::dot(e2, s.spin_axis));
    if(!(cos_lat > 1e-6)) {
        /* The pole itself: cos(lat) is 0 there, so the division below has no
           answer. Any heading from a pole gives the same inclination (90 deg),
           so take one and say what happened. */
        launch_h_ = tangent(up);
        why_ = "asked inclination unreachable from a pole: flew a meridian instead";
        return;
    }
    const double poleward = std::cos(target.inc_rad) / cos_lat;
    if(std::abs(poleward) > 1.0) {
        /* Unreachable from this latitude: an inclination flatter than the site's
           own latitude cannot be had by any heading. Go due east, which is the
           closest the geometry allows, and say so rather than pretending.
           This is a RANGE gate, not a NaN gate -- std::abs(NaN) > 1.0 is false, so
           it would wave a NaN into the acos below (#622). Nothing gets here with a
           NaN inclination: the Guidance window's arm door refuses one (the
           !(inc_deg >= 0 && <= 180) refuse in gameui.cpp) and --ascent
           range-checks its argument, and step() asserts the heading finite. */
        launch_h_ = e1;
        why_ = "asked inclination unreachable from this site: flew due east instead";
        return;
    }
    const double az = std::acos(poleward);   // [0,pi] from due east, toward the pole
    launch_h_ = glm::normalize(std::cos(az) * e1 + std::sin(az) * e2);
}

void AscentLaw::pressRefused(PressRefusal kind) {
    /* #941: the press this law asked for did not happen. Nothing the law sees
       records that -- stages_left, its only staging feedback, stays exactly where
       it was -- so the caller has to say so. A refusal is not a verdict about the
       stage, it is a door held shut: the phase machine and the throttle latch stay
       as they were, and a stage that runs dry while the door is shut still ends the
       mission the tick after its ask (guidance.h: waiting on a player for an
       unbounded time is a different design, and a worse default). */
    if(!active()) { return; }
    press_refused_ = kind;
    if(kind != PressRefusal::Crew) { return; }
    /* Only the fixable refusal is asked again: EVA the kerbal and the next ask
       separates. A root seam is how the craft was built in the VAB, so re-asking
       would print one refusal per second until the mission ended. */
    stage_asked_at_ = -1;
    stage_retry_in_ = kPressRetryTicks;
}

AscentCommand AscentLaw::step(const GuidanceState &s) {
    AscentCommand c;
    assert(s.sea_r >= 0.0);   // a datum below the centre would make altitudes lie
    conic_ok_ = false;
    if(!active()) { return c; }
    if(s.railed) {
        /* Bullet is not stepping this hull, so an armed thrust here is a silent
           no-op (tick.cpp skips railed ships in the force loop). Say so rather
           than logging a burn that never happened. */
        why_ = "ship is on rails: no commands sent";
        return c;
    }
    if(!(glm::length2(s.pos) > 1.0) || !std::isfinite(s.mu) || s.mu <= 0.0) {
        phase_ = AscentPhase::Failed; why_ = "no usable conic around this body"; return c;
    }

    /* A hull that has come back to the ground is not a mission that is still
       flying (#931). The Pad -> Ascent edge below is one-way and nothing else
       re-reads grounded, so a ship that lands kept a live mission: measured on
       Kerbin, phase=ascent with the lever pinned at 1.00 and |v| = 0.01 m/s for
       the last 20 s of the flight, and the verdict it finally printed was
       "the tanks are dry" -- the consequence, not the cause. Ask the question
       first, so the landing outranks the dry tanks. Anything past Pad was
       airborne at least once, which is what makes this a return rather than a
       refusal to start. */
    if(phase_ != AscentPhase::Pad && s.grounded) {
        phase_ = AscentPhase::Failed; why_ = "the ship came back down"; return c;
    }

    const OrbitElements o = computeOrbitElements(s.pos, s.vel, s.mu);
    last_el_ = o;
    conic_ok_ = true;
    const glm::dvec3 up = glm::normalize(s.pos);
    const double r = glm::length(s.pos);
    const double speed = glm::length(s.vel);
    const bool elliptic = o.traj == TrajectoryClass::Elliptic;
    const double apo_asl = elliptic ? apoapsisAsl(o, s.sea_r) : -1.0;
    const double peri_asl = periapsisAsl(o, s.sea_r);

    if(phase_ == AscentPhase::Pad && launch_h_ == glm::dvec3(0.0)) {
        setLaunchHeading(s);
        /* The turn follows this vector for the whole flight, so a bad one is not
           a local bug: a non-horizontal heading climbs for ever and a NaN one
           poisons every command after it. */
        assert(std::isfinite(launch_h_.x) && std::isfinite(launch_h_.y) && std::isfinite(launch_h_.z));
        assert(std::fabs(glm::dot(launch_h_, up)) < 1e-6);
    }

    /* A stage that cannot push is not a stage to fly, and the two ways a stage
       cannot push are different answers: no engine pointing the way, or nothing
       left to burn. availableAccel() alone cannot tell them apart -- it is a
       potential that never reads the tanks (#607) -- so it reads healthy at zero
       fuel and, without the driver's has_propellant, the law would fly a dry
       hull for ever: aiming, arming a burn that produces nothing, and never
       reaching Done or Failed. */
    /* Staging (#928): the law asks, it never presses. Two ways a stage is over --
       the plan says so (s.stage_due, which is Vehicle::stageIsDue() asking the VAB's
       own question on live tanks), or the stage has run dry while presses remain.
       The second is the one that used to end a mission: "the tanks are dry" was the
       verdict for a hull with three stages still folded under it.
       One ask per value of stages_left (stage_asked_at_), so a press does not fire
       every tick and bury the log; a press that separated nothing stepped the
       counter, so a later press can still be asked. A REFUSAL is the third case and
       the one the counter cannot see: Game::stageVehicle says no, stages_left stays
       exactly where it was, and the law never asked again -- for the rest of the
       mission, even after the player put the kerbal on EVA (#941). pressRefused()
       carries that answer back here, and stage_retry_in_ holds an unfixed refusal to
       about one ask a second. */
    if(stage_retry_in_ > 0) { stage_retry_in_--; }
    if(s.stages_left != last_stages_seen_) {
        /* The counter moved, so a press did happen -- and any refusal the caller
           reported before it is not the reason the stack is still together. */
        press_refused_ = PressRefusal::None;
        last_stages_seen_ = s.stages_left;
    }
    bool press = false;
    if(stage_retry_in_ <= 0 && s.stages_left > 0 && s.stages_left != stage_asked_at_) {
        if(s.stage_due || !s.has_propellant) { press = true; }
    }
    if(press) {
        c.want_stage = true;
        stage_asked_at_ = s.stages_left;
    }

    const bool can_thrust = s.accel_max > 0.0 && s.has_propellant;
    if(!can_thrust && !press && (phase_ == AscentPhase::Pad || phase_ == AscentPhase::Ascent
                       || phase_ == AscentPhase::Coast || phase_ == AscentPhase::Circ)) {
        phase_ = AscentPhase::Failed;
        /* The honest version of "the tanks are dry": true, and useless on its own
           when the reason the stack is still in one piece is a press the caller
           refused (#941 -- the mission ends saying "no fuel" with full stages still
           folded under it). Static strings, because why_ is a const char *. */
        why_ = s.accel_max <= 0.0 ? "no thrust available"
             : press_refused_ == PressRefusal::Crew ? "the tanks are dry, and a stage press was refused (crew aboard)"
             : press_refused_ == PressRefusal::Root ? "the tanks are dry, and a stage press was refused (root seam)"
             : "the tanks are dry";
        return c;
    }

    /* The third way a stage cannot fly: it has an engine and fuel, but the push
       is smaller than the planet's pull. Nothing downstream can discover this --
       the phase machine waits for an apoapsis that a grounded hull with TWR<1
       can never build -- so the only verdict the player ever gets is the empty
       tanks. Measured on the shipped tanker at Kerbin: lever pinned at 1.00,
       apoapsis stuck at 264 m and speed stuck at the pad's own 175 m/s for
       1617 s of sim (30000 logic ticks) before "the tanks are dry". Ask the
       question on the first tick instead: on the ground, thrust/mass under
       local gravity means the flight never starts. accel_max is rated full
       throttle (guidance.h), so a partly closed lever is not a false alarm.
       Only while grounded: in the air a weak stage is a slow climb, which is the
       law's business, not a refusal. */
    if(s.grounded && s.accel_max < s.mu / (r * r)) {
        phase_ = AscentPhase::Failed;
        why_ = "this stage cannot lift off: its thrust is below its own weight";
        return c;
    }

    /* Done is a pair of apsides, never a speed, and the pair is a FLOOR and a
       CEILING: periapsis at or above the ask (the ground is cleared) and
       apoapsis no farther above the ask than the band (we did not overshoot).
       Both halves carry weight. Periapsis alone would call a 20 x 300 km answer
       to an 80 x 200 km ask a success the moment periapsis crossed 80, and it
       would call a ship thrown past the target at 900 m/s -- periapsis AT the
       ask, apoapsis 18 times further out -- Done. The band is not a fudge: it is
       how far a finite burn overshoots the number it is steering at, measured at
       1.5 km on a 50 km ask and 9.4 km on a 250 km ask, i.e. about 4%. Anything
       outside it is a different orbit from the one asked for. */
    const double shape_band = std::max(3e3, 0.06 * std::max(target.apo_asl, 0.0));
    if(elliptic && peri_asl >= target.peri_asl && apo_asl <= target.apo_asl + shape_band) {
        phase_ = AscentPhase::Done; why_ = "periapsis at the ask, apoapsis inside the band";
        c.throttle = 0.0; return c;
    }

    // ---- phase machine ----------------------------------------------------
    if(phase_ == AscentPhase::Pad && !s.grounded) { phase_ = AscentPhase::Ascent; }

    if(phase_ == AscentPhase::Ascent) {
        /* Burn until the APOAPSIS is at the ask: that is the transfer, and the
           rest of the mission is one burn at its far end. */
        if(elliptic && apo_asl >= target.apo_asl) { phase_ = AscentPhase::Coast; }
    } else if(phase_ == AscentPhase::Coast && can_thrust) {
        /* Start the circularisation burn BEFORE the node, by half of its own
           length: the node is where dv is cheapest, so the burn should straddle
           it rather than all happen after it. Sizing the lead from the burn
           rather than from a constant matters at low asks, where the dv is big:
           a 50 km ask owed 485 m/s at its node, which is 19 s of this stage and
           a visible slice of a 1200 s orbit. A burn that long is not impulsive,
           and the further it strays from the node the more of it raises the
           apsis it is NOT trying to fix (measured: a 1.5-burn lead left a 15 km
           apoapsis gap on a 50 km ask -- 30% of the mission -- against 3 km on a
           100 km ask, where the same burn is only 10 s). */
        const double r_node = std::max(1.0, o.apoapsis_r);
        const double v_node = (elliptic && o.semi_major > 0.0)
            ? std::sqrt(std::max(0.0, s.mu * (2.0/r_node - 1.0/o.semi_major))) : 0.0;
        const double dv_need = std::sqrt(s.mu / r_node) - v_node;   // what the burn owes
        assert(can_thrust);   // the branch above tests it: a dry tick that is
                              // pressing this tick has no burn to size, and the next
                              // tick either has thrust or has nothing left to press
                              // and Fails (#928)
        const double burn_len = dv_need / s.accel_max;
        /* The lead is sized from the burn, never from the tick. Every in-envelope
           ask owes hundreds of m/s at its node, so half a burn-length is seconds
           to tens of seconds and no tick rate makes it vanish: adding a 2*dt floor
           here changed no flown case, so it left (#610 review). And sleeping past
           the time test is not a failure mode -- the radial_vel term below catches
           an approach that has already passed the node. */
        const double lead = burn_len * kLeadFrac;
        const bool at_node = (o.time_to_apo >= 0.0 && o.time_to_apo <= lead) || o.radial_vel <= 0.0;
        if(at_node) {
            phase_ = AscentPhase::Circ;
            /* Capture the node. The old prototype (reports/autopilots2026_10_24)
               gated its cutoff on the LIVE apoapsis, which its own burn was
               raising as it spoke, and stopped early: a 16.9 x 50.1 km answer to
               a 50 km ask. Stop against this number instead. */
            circ_node_r_ = std::max(o.apoapsis_r, r);
        }
    } else if(phase_ == AscentPhase::Circ
              && o.periapsis_r >= std::min(circ_node_r_, s.sea_r + target.peri_asl)) {
        /* The burn has caught the node it was aimed at: periapsis cannot be
           lifted past the apoapsis it is being matched to without
           over-circularising somewhere else, so this is as done as this profile
           gets. Now the shape test decides what to say, and BOTH answers are
           on offer. The top-of-tick Done rule above usually beats this one to
           the post; this is the branch for the asks it cannot land. A 400 km ask
           on Moho ends here at 400 x 519 km -- 30% of the mission in apoapsis --
           and used to read as Done, because the old stop rule only looked for
           periapsis at the ask. Claiming an orbit nobody asked for is worse than
           admitting the profile missed it. */
        if(elliptic && peri_asl >= target.peri_asl && apo_asl <= target.apo_asl + shape_band) {
            phase_ = AscentPhase::Done; why_ = "circularised at the captured node";
        } else {
            phase_ = AscentPhase::Failed;
            why_ = "the burn caught the node but the orbit is not the ask: this profile "
                   "overshoots at this altitude";
        }
        c.throttle = 0.0;
        return c;
    }

    // ---- the aim ----------------------------------------------------------
    const bool at_node_burn = (phase_ == AscentPhase::Coast || phase_ == AscentPhase::Circ);
    if(at_node_burn) {
        /* Prograde: while coasting it costs nothing and the ship is ready the
           instant the burn starts; during the burn itself the dv has to be
           exactly along the velocity or it bends the plane instead of raising
           the periapsis. */
        c.burn = (phase_ == AscentPhase::Circ) && can_thrust;
        pitch_ = 0.0;
        c.aim = speed > 1.0 ? glm::normalize(s.vel) : up;
        if(!c.burn) { return c; }
    } else {
        const double alt = r - s.sea_r;
        /* The turn's zero, set ONCE, on the first tick the schedule runs (#979). The
           question is "what ground did this hull come from", which is not the same
           question as "where was it when the mission was armed".

           A hull that was ON the ground at that tick -- a pad, or an air platform's
           deck (#977) -- came from its own height, so the zero is that height. On the
           shipped pads the two spellings differ by well under a degree (Kerbin's pad is
           270 m up, Moho's about 130 m), so every flight written down in this file
           moves a little and nothing more. On Venus's deck at 56 km it is everything:
           the datum reading put the hull 56 km INTO its own 12.7 km turn, asking 12.7
           deg on its first tick, and #930's ratchet means it can never ask steeper.

           A hull that was already FLYING did not start there, and its climb is read
           from the datum -- which is what "45 deg at h_half metres of climb" means for
           a ship that lifted off the surface. Reading the arm tick as a launch instead
           holds a hull at 30 km and 1.7 km/s at exactly 90 deg, because the ratchet
           cannot flatten it while the hull is LOSING height, and it spends a kilometre
           a second of the speed the mission needed: in 40 s its apoapsis went 29,977 ->
           29,067 m and its periapsis buried itself 565 km underground (#982). */
        if(apex_alt_ < 0.0) {
            apex_alt_ = alt;
            launch_alt_ = s.grounded ? alt : 0.0;
        } else if(alt > apex_alt_) {
            /* The schedule reads the high-water mark, not this tick's height: see
               pathAngle()'s note. On a climb this is the same number the live
               altitude gives, so a vacuum profile is untouched -- what it stops is
               the nose swinging back up when the hull sinks. */
            apex_alt_ = alt;
        }
        const double gamma = pathAngle(apex_alt_ - launch_alt_, target.apo_asl, air_);
        pitch_ = glm::degrees(gamma);
        /* Until the ship is actually moving downrange, "horizontal" is the
           heading we chose for the target inclination. After that it is where
           the ship is really going, which keeps the turn in the plane the conic
           already has. */
        const glm::dvec3 horiz_v = s.vel - up * glm::dot(s.vel, up);
        const glm::dvec3 hdir = (glm::length(horiz_v) > 20.0) ? glm::normalize(horiz_v) : launch_h_;
        glm::dvec3 aim = std::sin(gamma) * up + std::cos(gamma) * hdir;
        if(!(glm::length2(aim) > 1e-12)) { aim = up; }   // NaN must fall back too
        c.aim = glm::normalize(aim);
        c.burn = can_thrust;
    }

    // ---- the throttle: close on the speed ceiling ------------------------
    const double v_cap = speedCap(s.mu, r, s.sea_r + std::max(target.apo_asl, 0.0));
    const double band = 0.1 * v_cap + 20.0;    // proportional band, about a second of full thrust
    c.throttle = c.burn ? std::clamp((v_cap - speed) / band, 0.05, 1.0) : 0.0;
    return c;
}

const char *AscentLaw::phaseName() const {
    switch(phase_) {
        case AscentPhase::Pad:      return "pad";
        case AscentPhase::Ascent:   return "ascent";
        case AscentPhase::Coast:    return "coast";
        case AscentPhase::Circ:     return "circ";
        case AscentPhase::Done:     return "done";
        case AscentPhase::Failed:   return "failed";
    }
    return "?";
}

/* The air a mission is armed inside, read off the hull once (#961 moves it off the
   tick, #962 moves it from the datum to the hull).

   The height half is how much of the climb happens inside air -- a pitch schedule
   that flattens at 4.5 km inside an 85 km air spent 82% of the mission's delta-v
   pushing air out of the way (the before/after flights are quoted by pathAngle()).
   NOT atmosphere.enabled, which is the LOADER's flag: physical() is this repo's one
   answer to "is there air here?" -- the aero pass and the rails gate ask it too, so
   the law cannot disagree with them. An airless body hands the law zeros and gets the
   schedule it has always had.

   The weight half is the profile's own pressure ABOVE the hull, weighed the same way
   the loader and dragAtm() weigh air (#697, #739): dividing by airGravity() turns Pa
   into the kg per m^2 that the drag integral gets to spend. Reading it at the hull is
   what makes Venus's cloud deck honest -- at 56 km of a 116 km air there is 47 kPa
   overhead, not the 9.2 MPa under the deck, so the turn is sized at half a Kerbin's
   bend over 60 km instead of a full one over 116.

   A ground pad is under all but the height its own hull sits at, and that is NOT
   nothing: Kerbin's pad is 270 m up the datum, so the reading there is 84,730 m and
   10,002 kg/m^2 against the body's 85 km and 10,328. It is the hull's centre, not the
   pad's plaque, that sets it -- racer reads 10,009 at the same site where
   mars_explorer_mk1 reads 10,002. The column stays over kAirColumnFull either way, so
   only the height moves the schedule, and only through this overhead: 0.4 x 270 =
   108 m off a 34 km turn. It used to enter twice, because the schedule also counted
   the pad's own height as climb, which is why the same 270 m moved the bed flight's
   peak apoapsis 1.6 km through this term and 5.8% through that one (#979). */
AirFacts airAboveHull(const Vehicle &v) {
    if(v.m_parent == nullptr) { return AirFacts(); }
    const AtmosphereParams &atmo = v.m_parent->surface.atmosphere;
    if(!atmo.physical()) { return AirFacts(); }
    /* A body with air is a body to fly in, so by here the mission needs a built hull:
       comPos() below walks its bullet body. A frameless fixture returned earlier. */
    assert(v.hull != nullptr);
    /* Metres above the datum, from the body's ROTATING frame. A building bubble's
       origin is the building, not the body centre (#850), so |com| at kerbin_dock
       would read a few hundred metres and credit a hull that is above every kilometre
       of Kerbin's air with all of it. */
    const double r = glm::length(v.posInBodyRot(v.comPos()));
    const double alt_asl = r - v.m_parent->seaLevelRadius();
    const double air_g = v.m_parent->airGravity();
    /* Same rule as dragAtm's (#708): air needs a field to be weighed in. A loaded
       body always has a mu, so this only fires on a hand-built one. */
    assert(air_g > 0.0);
    /* One profile read, at arm time. The aero pass builds its DragAtmosphere per hull
       per substep; this builds one once per mission, and it is the same copy so the
       pressure cannot come from a different profile than the drag will bill. */
    const DragAtmosphere atm = dragAtm(atmo, *v.m_parent);
    return airOverhead(atmo.top(), alt_asl, airPressure(atm, alt_asl), air_g);
}

/* The SlewFree latch, and only the latch. Idempotent: Command's Sas case TOGGLES,
   so sending the mode that IS latched releases it and the next call finds
   SlewNone and sends nothing. flyAscent runs this on every tick it does not fly
   and Game::cancelAscent runs it once, so it must be safe either way. */
static void releaseAscentSas(Vehicle &v) {
    if(v.slewRequestMode() == SlewFree) { v.Command(ShipCmd(Sas, SlewFree), true); }
}

void releaseAscent(Vehicle &v) {
    releaseAscentSas(v);
    if(!v.throttleKeyHeld()) {
        v.Command(ShipCmd(ThrottleSet, 0.0f), true);
    }
}

bool flyAscent(Vehicle &v, AscentLaw &law, double dt, bool *want_stage) {
    /* The law's press request, if this caller wants one (#928). Cleared first so a
       bail-out tick -- inactive law, no frame, a NaN aim -- reports no ask rather
       than last tick's. */
    if(want_stage != nullptr) { *want_stage = false; }
    if(!law.active()) {
        /* Let go of the ship. SlewFree is a latch the player's SAS keys toggle,
           and a latched slew keeps a hull awake (off rails) forever, so leaving
           it set after the law is Done would hand the ship to a guidance law that
           has already stopped thinking. The LEVER is not touched here: the tick
           the verdict arrived on already wrote it (a Done/Failed step answers with
           a zero aim, and a zero aim cancels the aim only -- see below), and
           writing it every tick afterwards would hold the player's own throttle
           key down for them. A Cancel mid-burn has no such verdict tick, so it
           calls releaseAscent() to close both. */
        releaseAscentSas(v);
        return false;
    }
    if(v.m_parent == nullptr || v.frame == nullptr) { return false; }

    /* One frame dance per tick, then the law never sees a frame. The rotating
       frame is what a hull near the surface lives in, and its GetVel() is the
       ground-relative velocity: on a pad ship that is literally zero. */
    Frame *inertial = v.frame->getNonRotFrame();
    if(inertial == nullptr) { inertial = v.frame; }
    GuidanceState s;
    v.comStateIn(inertial, s.pos, s.vel);
    s.mu = v.m_parent->mu;
    s.sea_r = v.m_parent->seaLevelRadius();
    /* Nothing about the air is read here any more (#961): the law holds what it was
       armed with, from airAboveHull() above. This stays a snapshot of what MOVES. */
    s.grounded = v.isGrounded();
    s.railed = v.onRails;
    s.accel_max = v.availableAccel();
    /* The tanks, not the potential: see the can_thrust gate in step(). Vehicle::canBurn()
       is the gated twin of the reads above (#607): per IGNITED engine, over the resources
       THAT engine actually draws and the fuel groups it can reach -- so a stage carrying
       unburnt ore or a colleague's monoprop is still not a stage that can fly (#610
       review), and neither is a dry live stage with a full upper stage. The pair this
       replaces was getDeltaV()'s ship-wide union mask, a looser question than the law
       needs: it walked every thruster regardless of stage and never asked whether an
       engine missing one of two propellants could light. */
    s.has_propellant = v.canBurn();
    /* The staging plan, from the hull and not from the law (#928): which presses are
       still reachable, and whether this one is due. stageIsDue() walks the fuel graph
       the way canBurn() does, so it lives with the graph; the law sees two facts. */
    s.stages_left = v.stagesLeft();
    s.stage_due = v.stageIsDue();
    /* The body's pole, from the frame that owns it -- not omega(), which is zero
       for a body that does not spin and therefore left the launch heading
       degenerate. spinAxisRelTo is the one spelling the game already uses for
       the pole (frame.cpp:27). */
    s.spin_axis = v.frame->spinAxisRelTo(inertial);

    const AscentCommand c = law.step(s);
    /* Report the press here rather than at the end: a press has nothing to do with
       the aim, so a tick that bails below on a bad aim still honours the law's ask
       (the counter has already been spent in the law, so swallowing the request
       here would strand the stage for the rest of the mission). */
    if(want_stage != nullptr) { *want_stage = c.want_stage; }
    /* A NaN aim means something upstream is broken: ShipCmd(Aim) would leave the
       hull holding its last good setpoint, so refuse the whole tick rather than
       send it (pointingTo() fails closed on the same thing, #609). */
    if(!(std::isfinite(c.aim.x) && std::isfinite(c.aim.y) && std::isfinite(c.aim.z))) { return false; }
    /* A ZERO aim is a different answer -- "there is no direction to point at" --
       and it is what the Done and Failed ticks return. It cancels the aim, not
       the rest of the command: an early return here left the throttle lever
       wherever the circularisation burn had parked it (the 610 case's
       phase=done rows read lever=0.05), so the mission ended with a partly open
       engine still in the law's hand. */
    const bool aimed = glm::length2(c.aim) > 1e-12;
    if(aimed) {
        /* Back into the hull's basis: pointingTo() and ShipCmd(Aim) both speak
           the frame the hull lives in, exactly as slewTargetDir() does when it
           resolves prograde from the conic. */
        const glm::dvec3 aim_hull = glm::transpose(v.frame->GetOrientRelTo(inertial)) * c.aim;
        /* The latch, not the toggle: Command's Sas case releases a mode that is
           already latched, so it is only ever sent when the mode is NOT latched.
           A law that re-sent it every tick would flicker itself off. */
        if(v.slewRequestMode() != SlewFree) { v.Command(ShipCmd(Sas, SlewFree), true); }
        v.Command(ShipCmd(Aim, aim_hull), true);
    }
    assert(!c.burn || aimed);   // burning along no direction is a contradiction, not a coast

    /* burn is not decoration. Armed thrust is cleared every tick (tick.cpp), so
       ShipCmd(Thrust) is what lights the engine -- and the throttle LEVER is a
       separate, latching state. Arming unconditionally left the engine lit
       through the whole coast phase of the first hull test: the law asked for
       throttle 0, but the lever it had itself set to full two minutes earlier
       was still there, and the ship burned its way out of Moho's SOI with a
       cheerful "coast" in the log. So the driver does what the law said: light
       it when it says burn, and put the lever back to zero when it does not. */
    if(c.burn) {
        v.Command(ShipCmd(Thrust), true, dt);   // armed thrust is cleared every tick
    }
    if(!v.throttleKeyHeld()) {
        /* #400 F3's yield rule: while a throttle key is down the player owns the
           lever, and the law takes it back the tick the key is up. */
        v.Command(ShipCmd(ThrottleSet, (float)(c.burn ? c.throttle : 0.0)), true);
    }
    return aimed;
}
