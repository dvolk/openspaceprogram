// test_wheels -- the landing-gear ground law (src/wheel.h), as pure math.
// No Bullet, no GL: wheel::force turns a contact + patch velocity into a force,
// and these pins keep that turn honest. The contact-finding (ray + analytic
// fall-through) and the one-body application live in Vehicle::applyGroundForces
// and are covered end-to-end by the runway e2e cases.
//
// Build & run (from repo root): make test_wheels   (or: make test)

#include <cmath>
#include <cstdio>

#include <glm/glm.hpp>

#include "wheel.h"
#include "tchecks.h"

// A wheel on flat ground: normal +Z, rolling +Y, at rest unless told otherwise.
static wheel::Input rest(double compression) {
    wheel::Input in;
    in.compression = compression;
    in.compressRate = 0.0;
    in.spring = 10000.0;
    in.damp = 1000.0;
    in.normal = glm::dvec3(0.0, 0.0, 1.0);
    in.forward = glm::dvec3(0.0, 1.0, 0.0);
    in.contactVel = glm::dvec3(0.0);
    in.brake = 0.0;
    in.stopRate = 0.0;   // momentum clamp off: these pins are about the gains, not
                         // the clamp -- at warp 1 the clamp cannot bind (2.7*g*h is
                         // ~0.04), so the clamp has its own pins below and a hull
                         // case at warp 10 in e2e.
    in.normalStopRate = 0.0;   // likewise for the strut damper (#862)
    return in;
}

int main() {
    printf("== wheel::force ==\n");
    const glm::dvec3 Z(0.0, 0.0, 1.0);

    // Airborne: no compression, no force (whatever the velocity).
    {
        wheel::Input in = rest(0.0);
        in.contactVel = glm::dvec3(0.0, 50.0, -5.0);
        CHECK_NEAR(glm::length(wheel::force(in)), 0.0, 1e-12, "airborne -> zero");
        in.compression = -0.1;
        CHECK_NEAR(glm::length(wheel::force(in)), 0.0, 1e-12, "negative compression -> zero");
    }

    // Static spring: F = k*x along the normal, nothing in the plane.
    {
        wheel::Input in = rest(0.1);
        const glm::dvec3 F = wheel::force(in);
        CHECK_NEAR(F.z, 10000.0 * 0.1, 1e-9, "spring push = k*x");
        CHECK_NEAR(F.x, 0.0, 1e-12, "static: no lateral");
        CHECK_NEAR(F.y, 0.0, 1e-12, "static: no rolling");
    }

    // Damper: closing fast adds push; opening fast subtracts, but never pulls.
    {
        wheel::Input in = rest(0.1);
        in.compressRate = 1.0;   // closing
        CHECK_NEAR(wheel::force(in).z, 10000.0 * 0.1 + 1000.0 * 1.0, 1e-9,
                   "damper adds on closing");
        in.compressRate = -1.0;  // opening hard: k*x - c = 1000 - 1000 = 0
        CHECK_NEAR(wheel::force(in).z, 0.0, 1e-9, "damper subtracts on opening");
        in.compressRate = -5.0;  // would go negative -> clamped to 0 (no stick)
        CHECK_NEAR(glm::length(wheel::force(in)), 0.0, 1e-12, "suspension never pulls");
    }

    // Rolling resistance opposes forward motion (and is small).
    {
        wheel::Input in = rest(0.1);
        in.contactVel = glm::dvec3(0.0, 20.0, 0.0);   // rolling forward
        const glm::dvec3 F = wheel::force(in);
        CHECK_TRUE(F.y < 0.0, "rolling resistance opposes forward");
        CHECK_TRUE(std::fabs(F.y) < 0.1 * F.z, "rolling resistance is a small fraction of load");
    }

    // Brake opposes forward motion, harder than rolling resistance.
    {
        wheel::Input in = rest(0.1);
        in.contactVel = glm::dvec3(0.0, 20.0, 0.0);
        const double roll = wheel::force(in).y;
        in.brake = 1.0;
        const double braked = wheel::force(in).y;
        CHECK_TRUE(braked < roll, "brake pushes back harder than rolling");
        CHECK_TRUE(braked < 0.0, "brake opposes forward");
    }

    // Lateral grip opposes sideways slip.
    {
        wheel::Input in = rest(0.1);
        in.contactVel = glm::dvec3(5.0, 0.0, 0.0);   // slipping +X (lateral)
        const glm::dvec3 F = wheel::force(in);
        CHECK_TRUE(F.x < 0.0, "lateral grip opposes slip");
        CHECK_NEAR(F.y, 0.0, 1e-9, "pure lateral slip: no rolling force");
    }

    // Friction circle: a huge slip cannot exceed mu*N in the plane.
    {
        wheel::Input in = rest(0.1);
        in.contactVel = glm::dvec3(500.0, 500.0, 0.0);
        const glm::dvec3 F = wheel::force(in);
        const double N = F.z;
        const double tang = std::sqrt(F.x * F.x + F.y * F.y);
        CHECK_TRUE(tang <= wheel::kGrip * N + 1e-6, "tangential capped at mu*N");
        CHECK_NEAR(N, 10000.0 * 0.1, 1e-9, "normal unaffected by slip");
    }

    // The normal component is always along the ground normal (a tilted deck).
    {
        wheel::Input in = rest(0.1);
        const glm::dvec3 n = glm::normalize(glm::dvec3(0.1, 0.0, 1.0));
        in.normal = n;
        in.forward = glm::normalize(glm::dvec3(0.0, 1.0, 0.0) - n * glm::dot(glm::dvec3(0.0, 1.0, 0.0), n));
        const glm::dvec3 F = wheel::force(in);
        const glm::dvec3 tang = F - n * glm::dot(F, n);
        CHECK_NEAR(glm::length(tang), 0.0, 1e-9, "static force is along the normal");
        CHECK_TRUE(glm::dot(F, n) > 0.0, "pushes out of the deck");
    }

    /* The momentum clamp (#770): the tangential pair may not spend more than
       stopping this substep's slip needs. Without it, a braked wheel's near-zero
       slip gain -- (kBrakeGrip+kRollResist)/kSlipEps = 2.7 N per N of load per
       m/s, against a one-substep budget of m/(n*h) -- over-corrects the patch
       velocity: the ratio is 2.7*g*h, mass-free, and it passes 1 at h = 0.042 s on
       Venus (warp 8 at --physics-hz 60). The sign flips every step, the suspension
       pumps, and a parked hull walks itself across a balloon deck at warp. */
    {
        // Lateral slip of 0.05 m/s: the raw gain is 4000 * 0.05 = 200 N.
        wheel::Input noClamp = rest(0.1);
        noClamp.contactVel = glm::dvec3(0.05, 0.0, 0.0);
        const glm::dvec3 raw = wheel::force(noClamp);
        CHECK_NEAR(std::fabs(raw.x), wheel::kLatStiff * 0.05, 1e-9,
                   "no clamp: lateral gain sets the force");

        noClamp.stopRate = 2000.0;   // 2000 kg/s * 0.05 m/s = 100 N of momentum
        const glm::dvec3 held = wheel::force(noClamp);
        CHECK_NEAR(std::fabs(held.x), 100.0, 1e-9,
                   "the contact spends its slip momentum, not its gain");
        CHECK_TRUE(held.x < 0.0 && raw.x < 0.0, "and it still opposes the slip");
        CHECK_NEAR(held.z, 10000.0 * 0.1, 1e-9, "the clamp leaves the normal alone");

        // A slip the friction circle already owns is unaffected: 5 m/s wants
        // 20 kN, mu*N is 900 N, and the momentum budget (10 kN) is not the limit.
        noClamp.contactVel = glm::dvec3(5.0, 0.0, 0.0);
        const glm::dvec3 slid = wheel::force(noClamp);
        CHECK_NEAR(std::hypot(slid.x, slid.y), wheel::kGrip * 1000.0, 1e-9,
                   "a slipping wheel still gets mu*N");

        // The brake is the term that bit: 0.05 m/s of rolling under a full brake
        // asks for 820 * tanh(0.05/0.3) = 135 N, above the 100 N in front of it.
        wheel::Input braked = rest(0.1);
        braked.brake = 1.0;
        braked.contactVel = glm::dvec3(0.0, 0.05, 0.0);
        const double want = (wheel::kRollResist + wheel::kBrakeGrip) * 1000.0
                            * std::tanh(0.05 / wheel::kSlipEps);
        const double unbuilt = wheel::force(braked).y;
        CHECK_NEAR(std::fabs(unbuilt), want, 1e-9, "no clamp: the brake drinks the slip");
        braked.stopRate = 2000.0;
        const double clamped = wheel::force(braked).y;
        CHECK_NEAR(std::fabs(clamped), 100.0, 1e-9, "a braked wheel cannot brake past a stop");
        CHECK_TRUE(clamped < 0.0, "still backwards");
    }

    /* The strut's DAMPER gets the same budget as the tire pair (#862). It is an
       explicit force held for a whole substep, so damp*compressRate over-corrects
       the closing rate once damp*h passes the mass one strut carries: shipped gear
       is 3000 N s/m under ~220 kg per wheel, so past h = 0.073 s -- which a
       hand-set --physics-hz 30 crosses and warp 10 at 60 Hz does not. */
    {
        wheel::Input strut = rest(0.1);
        strut.compressRate = 1.0;                     // closing at 1 m/s
        CHECK_NEAR(wheel::force(strut).z, 10000.0 * 0.1 + 1000.0, 1e-9,
                   "uncapped: the strut bills its own gain");
        strut.normalStopRate = 400.0;                 // 400 N of momentum in front of it
        CHECK_NEAR(wheel::force(strut).z, 10000.0 * 0.1 + 400.0, 1e-9,
                   "capped: the strut spends its substep momentum, not its gain");
        strut.compressRate = -1.0;                    // rebounding
        CHECK_NEAR(wheel::force(strut).z, 10000.0 * 0.1 - 400.0, 1e-9,
                   "the same cap on the rebound");
        wheel::Input soft = rest(0.01);
        soft.compressRate = -1.0;
        soft.normalStopRate = 400.0;
        CHECK_NEAR(wheel::force(soft).z, 0.0, 1e-9, "and a wheel still cannot stick");
        wheel::Input still = rest(0.1);
        still.normalStopRate = 1.0;                   // a tiny budget, no closing rate
        CHECK_NEAR(wheel::force(still).z, 10000.0 * 0.1, 1e-9,
                   "the SPRING is never capped -- it is what holds the hull up");
    }

    /* #862's mechanism, pinned at the one place it happens: a substep of the
       uncapped damper does not stop the strut closing, it REVERSES the closing
       rate past zero, and the rebound it invents is the energy the strut pumps.
       Shipped gear (wheel_damp 3000) under the ~220 kg one strut of a 658 kg hull
       carries, at h = 0.111 s (a hand-set --physics-hz 30) closing at 0.1 m/s:
       the damper alone asks 300 N, which over one substep is 0.151 m/s of change
       against 0.1 m/s of motion -- 1.5x too much. Capped at m/h = 1982 kg/s it
       asks 198 N, which spends the closing exactly (deadbeat). The real caller
       divides the budget across all the gear as well, so it lands under this.
       An envelope loop over 400 substeps was tried first and does NOT show growth
       at these constants (0.321 m decays to 0.049 m either way: the strut is
       damped at zeta 0.64); the growth shows up in the hull, through the friction
       circle, which is what the --physics-hz 30 platform run measures. */
    {
        const double m = 220.0, h = 0.111, v0 = 0.1;
        wheel::Input in = rest(0.1);
        in.damp = 3000.0;                              // shipped wheel_damp
        const double springOnly = wheel::force(in).z;   // compressRate 0
        in.compressRate = v0;
        const double damperRaw = wheel::force(in).z - springOnly;
        const double dvRaw = damperRaw / m * h;
        CHECK_NEAR(damperRaw, 3000.0 * v0, 1e-9, "the damper alone is c * closing rate");
        CHECK_TRUE(dvRaw > v0, "uncapped, one substep buys MORE velocity change than there is motion");
        in.normalStopRate = m / h;
        const double damperCap = wheel::force(in).z - springOnly;
        const double dvCap = damperCap / m * h;
        CHECK_NEAR(dvCap, v0, 1e-9, "capped at m/h the substep spends the closing, deadbeat");
        printf("   [strut h=%.3f] raw %.0f N = %.3f m/s against %.3f m/s of closing;"
               " capped %.0f N\n", h, damperRaw, dvRaw, v0, damperCap);
    }

    /* STATIC FRICTION (#1184). A surface holds a patch still; it does not merely
       damp it. Kerbin authors 5 m/s of surface wind, which is ~380 N of air on a
       parked 2.2 t lander, and against a tire pair proportional to SLIP that
       parked the hull at a permanent crawl -- load/gain, with the gain m/h from
       the momentum budget above, so the crawl grew with the substep: 0.01 m/s at
       warp 1, 0.05 m/s at warp 10, 21 m of track and the edge of the pad in three
       minutes of sim. The pins below are that wind, in N, on a contact whose
       whole friction circle (mu*N = 900 N here) is far bigger than the load. */
    {
        // A footpad (brake held on) answers a steady fore-and-aft load exactly.
        wheel::Input pad = rest(0.1);            // N = 10000*0.1 = 1000, circle 900
        pad.brake = 1.0;
        pad.push = glm::dvec3(0.0, 500.0, 0.0);  // the substep's push, +forward
        const glm::dvec3 held = wheel::force(pad);
        CHECK_NEAR(held.y, -500.0, 1e-9, "the hold cancels the push, not a fraction of it");
        CHECK_NEAR(held.z, 1000.0, 1e-9, "the suspension is untouched by it");

        // A free wheel holds ACROSS itself but rolls fore-and-aft: otherwise a
        // wheeled hull could never be pushed along its own wheels by the wind.
        wheel::Input free_ = rest(0.1);
        free_.push = glm::dvec3(0.0, 500.0, 0.0);
        CHECK_NEAR(wheel::force(free_).y, 0.0, 1e-9, "unbraked: no fore-and-aft hold");
        // l = n x f = -X, so a +X push asks for +500 along l, which is a force
        // applied at -X -- opposing the push, as it must be.
        free_.push = glm::dvec3(500.0, 0.0, 0.0);
        CHECK_NEAR(wheel::force(free_).x, -500.0, 1e-9, "a free wheel still grips sideways");

        // A push along the normal is the suspension's business, not the tire's.
        // With n = +Z these two CHECKs cannot tell whether force() projects push
        // into the ground plane at all (l and f are already in it), so the
        // tilted-deck block below is what weighs that projection.
        wheel::Input down = rest(0.1);
        down.brake = 1.0;
        down.push = glm::dvec3(0.0, 0.0, -5000.0);
        CHECK_NEAR(wheel::force(down).z, 1000.0, 1e-9, "no hold along the strut");
        CHECK_NEAR(wheel::force(down).y, 0.0, 1e-9, "and no invented tangential force");

        // The circle, not the momentum budget, bounds the hold. The old note here
        // warned that a caller capping the pair at stopRate*|v| would cancel a
        // hold in silence, so pin that the budget bounds only the velocity terms.
        wheel::Input poor = rest(0.1);
        poor.brake = 1.0;
        poor.stopRate = 1.0;                     // a hull with almost no slip momentum
        poor.push = glm::dvec3(0.0, 500.0, 0.0);
        CHECK_NEAR(wheel::force(poor).y, -500.0, 1e-9,
                   "a steady load is held even where the slip budget is nothing");

        // Overwhelm it and the surface gives, at mu*N exactly.
        wheel::Input slip = rest(0.1);
        slip.brake = 1.0;
        slip.push = glm::dvec3(0.0, 5000.0, 0.0);
        CHECK_NEAR(wheel::force(slip).y, -wheel::kGrip * 1000.0, 1e-9,
                   "past the circle the hull slides, at mu*N");

        // One circle for the sum: damping and holding together cannot exceed it.
        // The slip is inside the creep band (0.1 m/s) so the hold is live at all.
        wheel::Input both = rest(0.1);
        both.brake = 1.0;
        both.push = glm::dvec3(600.0, 0.0, 0.0);      // 600 N of hold, sideways
        both.contactVel = glm::dvec3(0.1, 0.0, 0.0);  // and 400 N of tire to damp
        const glm::dvec3 tot = wheel::force(both);
        const double tangential = std::sqrt(tot.x * tot.x + tot.y * tot.y);
        CHECK_NEAR(tangential, wheel::kGrip * 1000.0, 1e-9,
                   "the pair shares one mu*N rather than each getting one");
        CHECK_NEAR(tot.z, 1000.0, 1e-9, "and the sum leaves the strut alone");
    }

    /* The hold is STICTION, so it only exists while the patch creeps (the same
       kSlipEps band the tanh knee uses). An ungated hold answers a SLIDING
       contact too, and that is a different bug with the same signature: a braked
       hull rolling out is under kinetic friction, and cancelling this substep's
       air force there silently deletes the drag from its stopping distance (and
       a throttled braked plane from its acceleration). */
    {
        // 6 m/s of rolling under a 500 N fore-and-aft load: the load is not held.
        wheel::Input roll = rest(0.1);
        roll.brake = 1.0;
        roll.contactVel = glm::dvec3(0.0, 6.0, 0.0);
        roll.push = glm::dvec3(0.0, 500.0, 0.0);
        const double withLoad = wheel::force(roll).y;
        roll.push = glm::dvec3(0.0);
        CHECK_NEAR(withLoad, wheel::force(roll).y, 1e-12,
                   "a sliding wheel neither holds nor un-holds the substep's air");
        CHECK_TRUE(withLoad < 0.0, "and it still brakes the roll");
        CHECK_NEAR(withLoad, -(wheel::kRollResist + wheel::kBrakeGrip) * 1000.0, 1e-9,
                   "the brake at full kinetic grip, unmodified");

        // Same across the wheel: a drifting wheel grips by slip, not by holding.
        // N is 5x here (compression 0.5) so mu*N = 4500 leaves the 1600 N of slip
        // grip unscaled -- otherwise the circle clips both sides and the pin proves
        // nothing.
        wheel::Input drift = rest(0.5);
        drift.brake = 1.0;
        drift.contactVel = glm::dvec3(0.4, 0.0, 0.0);
        drift.push = glm::dvec3(500.0, 0.0, 0.0);
        const double drifted = wheel::force(drift).x;
        drift.push = glm::dvec3(0.0);
        CHECK_NEAR(drifted, wheel::force(drift).x, 1e-12, "no lateral hold past the band");
        CHECK_NEAR(drifted, -wheel::kLatStiff * 0.4, 1e-9, "just the slip grip");

        // The band edge: 0.2 m/s creeps (full hold), 0.4 m/s slides (none). The
        // load is 400 N, not 500, because at 0.2 m/s of braked slip the tyre is
        // already 478 N into mu*N = 900 and a bigger load would clip the circle
        // instead of showing the hold.
        wheel::Input edge = rest(0.1);
        edge.brake = 1.0;
        edge.push = glm::dvec3(0.0, 400.0, 0.0);
        edge.contactVel = glm::dvec3(0.0, 0.2, 0.0);
        const double creep = wheel::force(edge).y;
        edge.push = glm::dvec3(0.0);
        CHECK_NEAR(creep - wheel::force(edge).y, -400.0, 1e-9, "creeping: the hold is full");
        edge.contactVel = glm::dvec3(0.0, 0.4, 0.0);
        edge.push = glm::dvec3(0.0, 400.0, 0.0);
        const double sliding = wheel::force(edge).y;
        edge.push = glm::dvec3(0.0);
        CHECK_NEAR(sliding - wheel::force(edge).y, 0.0, 1e-12, "sliding: no hold at all");

        // A free wheel holds ACROSS itself at rest, so the gate is about slip, not
        // about the brake: without this, a wheeled hull would weathervane downwind.
        wheel::Input parked = rest(0.1);
        parked.contactVel = glm::dvec3(0.0, 0.2, 0.0);   // rolling free, slowly
        parked.push = glm::dvec3(500.0, 0.0, 0.0);       // and a crossload on it
        CHECK_NEAR(wheel::force(parked).x, -500.0, 1e-9,
                   "a slow free wheel still grips sideways");
        CHECK_NEAR(wheel::force(parked).y,
                   -wheel::kRollResist * 1000.0 * std::tanh(0.2 / wheel::kSlipEps), 1e-9,
                   "and rolls fore-and-aft: rolling resistance only, no hold");
    }

    /* The player-facing shape of #1184: a hull parked on a SLOPE, where the load
       to hold is gravity itself. n = normalize(0, 0.6, 1) is a 31 deg deck, whose
       downhill axis is f and whose normal is n, and a 1000 N vertical load then
       asks 1000*sin(31) = 514.5 N in the plane -- well inside the 900 N circle of
       this patch, so a held patch reads exactly that and nothing else.
       (Note what this does NOT pin: dropping force()'s projection of push into
       the ground plane changes nothing here or anywhere, because l and f are
       perpendicular to n by construction, so the normal part of push cannot
       reach dot(pushT, l) or dot(pushT, f). That line is insurance for a caller
       that fails to project its own `forward`, which both shipped loops do.) */
    {
        const glm::dvec3 n = glm::normalize(glm::dvec3(0.0, 0.6, 1.0));
        const glm::dvec3 f = glm::normalize(glm::dvec3(0.0, 1.0, 0.0)
                                             - n * n.y);
        const double downSlope = 1000.0 * 0.6 / std::sqrt(1.36);
        wheel::Input hill = rest(0.1);
        hill.normal = n;
        hill.forward = f;
        hill.brake = 1.0;
        hill.push = glm::dvec3(0.0, 0.0, -1000.0);   // a weight-like load, straight down
        const glm::dvec3 F = wheel::force(hill);
        CHECK_NEAR(glm::dot(F, f), -downSlope, 1e-6, "the deck holds the slope component");
        CHECK_NEAR(glm::dot(F, n), 1000.0, 1e-6, "while the strut carries the rest");
        CHECK_NEAR(glm::dot(F, glm::cross(n, f)), 0.0, 1e-6, "and nothing appears across it");

        // The same deck with the brake off: a wheel is free to roll away, and the
        // hold must not invent a force that keeps it on the hill.
        hill.brake = 0.0;
        CHECK_NEAR(glm::dot(wheel::force(hill), f), 0.0, 1e-12,
                   "an unbraked wheel holds no slope at all");
    }

    /* sizeToStrut: any hull stands on its springs, not on Bullet collision. Five
       things to hold: at or under the designed ride the authored pair survives
       exactly (so no shipped hull moves); past it k = share / ride, which parks the
       static stroke on the designed ride instead of on the bump stop; the damper
       scales LINEARLY with k, which is what keeps the heave mode at a constant
       damping ratio; the ride never exceeds what the part's own travel can reach;
       and the stroke below travel is left over for a landing. */
    {
        const double kAuth = 25000.0, cAuth = 3000.0, travel = 0.15;
        // The shipped datum: the skiff's loaded main wheel is 25000 N/m compressing
        // 0.106 of its 0.15 (e2e 1158-b reads the free stroke). That hull has to
        // pass through this rule untouched, bit for bit.
        double k = kAuth, c = cAuth;
        wheel::sizeToStrut(kAuth * wheel::kRideStroke, travel, k, c);
        CHECK_NEAR(k, kAuth, 1e-9, "a hull at the designed ride keeps its authored spring");
        CHECK_NEAR(c, cAuth, 1e-9, "and its authored damper");

        // A 4.1 t spaceplane on three wheels is ~13.3 kN a wheel, against the
        // 3.75 kN (k x travel) the authored strut can ever push. Before this rule
        // that hull sat on the runway on collision: 9.2 kN of spring against
        // 39.8 kN of weight, nose wheel on its bump stops, and the player read it
        // as a plane glued to the deck.
        const double heavyShare = 13273.0;
        CHECK_TRUE(kAuth * travel < heavyShare, "the authored strut cannot hold a 4 t hull");
        k = kAuth; c = cAuth;
        wheel::sizeToStrut(heavyShare, travel, k, c);
        CHECK_NEAR(k, heavyShare / wheel::kRideStroke, 1e-6,
                   "a heavier hull rates the strut to what it carries");
        CHECK_NEAR(k * wheel::kRideStroke, heavyShare, 1e-6,
                   "so the static stroke lands back on the designed ride");
        CHECK_TRUE(wheel::kRideStroke < travel, "which stays inside the stroke a landing needs");
        // The damper goes up with k one-for-one. Scaling it sqrt(k) instead -- the
        // reflex from c = 2 sqrt(k m) -- is what the comment in wheel.h refuses: on
        // this branch omega is pinned at sqrt(g/ride), mass-free, while k carries the
        // mass, so sqrt leaves zeta falling like 1/sqrt(mass). Measured: the skiff
        // sits at 3000 / (2 sqrt(25000 * 219.4)) = 0.64 while sqrt-scaling a 4 t
        // hull lands at 0.26, i.e. ~40% overshoot and a couple of seconds of heave
        // ring at every touchdown, on exactly the hull this rule exists to rescue.
        CHECK_NEAR(c, cAuth * (k / kAuth), 1e-6, "the damper scales with k, linearly");
        CHECK_NEAR(c / k, cAuth / kAuth, 1e-12, "so c/k, and with it zeta, does not move");

        // A short-travel part is never sized for a stroke it cannot reach: the ride
        // clamps to kRideMaxFraction of its travel, which leaves a stiffer strut that
        // still stands on its spring rather than a soft one parked on its stop.
        const double shortTravel = 0.02;
        k = kAuth; c = cAuth;
        wheel::sizeToStrut(heavyShare, shortTravel, k, c);
        CHECK_NEAR(k, heavyShare / (wheel::kRideMaxFraction * shortTravel), 1e-6,
                   "a short-travel strut sizes to ITS stroke, not the default ride");
        CHECK_TRUE(k * wheel::kRideMaxFraction * shortTravel <= heavyShare,
                   "and never sits past that fraction of its travel");
        CHECK_TRUE(k > heavyShare / wheel::kRideStroke, "which makes it stiffer still");

        // And nothing happens where no sizing is needed.
        k = kAuth; c = cAuth;
        wheel::sizeToStrut(1000.0, travel, k, c);
        CHECK_NEAR(k, kAuth, 1e-9, "a light hull is not stiffened");
        wheel::sizeToStrut(0.0, travel, k, c);
        CHECK_NEAR(c, cAuth, 1e-9, "and a wheel in the air changes nothing");
        wheel::sizeToStrut(heavyShare, 0.0, k, c);
        CHECK_NEAR(k, kAuth, 1e-9, "and a part with no travel to size against is left alone");
    }

    printf("test_wheels: %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
