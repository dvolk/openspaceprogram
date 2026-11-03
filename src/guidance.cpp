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
   different value with a one-line diff. Every number here is what the closed
   loop in tests/test_guidance.cpp settled on, not what looked plausible. */
static const double kFlatHalf = 0.03;  // pitch-over half-angle altitude, as a fraction of the ask
static const double kLeadFrac = 0.5;   // node burn starts this fraction of its length early

/* The pitch schedule: a commanded PATH ANGLE against ALTITUDE. 90 deg (straight
   up) off the pad, halving every kFlatHalf * ask metres of climb -- so a 100 km
   ask is 45 deg by 3 km and under 10 deg by 30 km.

   Two failure modes this replaced. Keyed to a fraction of the target apoapsis
   (alt / 0.25 * ask), it flew nearly vertical for 25 km and 34 s: periapsis at
   the body's CENTRE, no downrange speed, no way to circularise at any amount of
   fuel. Keyed to the live APOAPSIS instead, it jammed vertical forever: on Moho
   a 19 m/s^2 stage opens the conic by 50 km, and "no apoapsis" read as "not made
   yet" -- pointing up harder is what opened it.

   What matters is being flat EARLY and LOW, because horizontal speed at low
   altitude is what buys apoapsis cheaply. A gravity turn does this by itself
   when you stop steering; the schedule's job is to ask for the lean before
   gravity gets around to it. */
static double pathAngle(double alt, double ask_alt) {
    const double h_half = std::max(1000.0, kFlatHalf * std::max(ask_alt, 0.0));
    return std::atan2(h_half, std::max(0.0, alt));   // 90 deg at the pad, -> 0 as we climb
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
           closest the geometry allows, and say so rather than pretending. */
        launch_h_ = e1;
        why_ = "asked inclination unreachable from this site: flew due east instead";
        return;
    }
    const double az = std::acos(poleward);   // [0,pi] from due east, toward the pole
    launch_h_ = glm::normalize(std::cos(az) * e1 + std::sin(az) * e2);
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
    const bool can_thrust = s.accel_max > 0.0 && s.has_propellant;
    if(!can_thrust && (phase_ == AscentPhase::Pad || phase_ == AscentPhase::Ascent
                       || phase_ == AscentPhase::Coast || phase_ == AscentPhase::Circ)) {
        phase_ = AscentPhase::Failed;
        why_ = s.accel_max > 0.0 ? "the tanks are dry" : "no thrust available";
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
    } else if(phase_ == AscentPhase::Coast) {
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
        assert(can_thrust);   // the gate above has already Failed a stage that cannot push
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
        const double gamma = pathAngle(alt, target.apo_asl);
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

bool flyAscent(Vehicle &v, AscentLaw &law, double dt) {
    if(!law.active()) {
        /* Let go of the ship. SlewFree is a latch the player's SAS keys toggle,
           and a latched slew keeps a hull awake (off rails) forever, so leaving
           it set after the law is Done would hand the ship to a guidance law that
           has already stopped thinking. Command's Sas case toggles, so sending
           the mode that IS latched releases it -- and the next tick sees
           SlewNone and sends nothing, which makes this idempotent. */
        if(v.slewRequestMode() == SlewFree) { v.Command(ShipCmd(Sas, SlewFree), true); }
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
    s.grounded = v.isGrounded();
    s.railed = v.onRails;
    s.accel_max = v.availableAccel();
    /* The tanks, not the potential: see the can_thrust gate in step(). And the
       resources THIS ship's thrusters actually draw, not "any mass in any tank"
       -- a stage carrying unburnt ore or a colleague's monoprop is not a stage
       that can fly (#610 review). Same pair getDeltaV() uses. */
    bool burn_res[(int)ResourceType::Num];
    v.enginePropellantMask(burn_res, /*includeJets=*/false);
    s.has_propellant = v.fuelMassMasked(burn_res) > 0.0;
    /* The body's pole, from the frame that owns it -- not omega(), which is zero
       for a body that does not spin and therefore left the launch heading
       degenerate. spinAxisRelTo is the one spelling the game already uses for
       the pole (frame.cpp:27). */
    s.spin_axis = v.frame->spinAxisRelTo(inertial);

    const AscentCommand c = law.step(s);
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
