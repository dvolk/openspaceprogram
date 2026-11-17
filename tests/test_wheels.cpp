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

static int g_failures = 0;
static int g_checks = 0;

#define CHECK_TRUE(cond, msg)                                                  \
    do {                                                                       \
        g_checks++;                                                            \
        if(!(cond)) { g_failures++; printf("FAIL: %s\n", msg); }               \
    } while(0)

#define CHECK_NEAR(actual, expected, tol, msg)                                 \
    do {                                                                       \
        g_checks++;                                                            \
        const double _a = (actual), _e = (expected);                           \
        if(!std::isfinite(_a) || std::fabs(_a - _e) > (tol)) {                 \
            g_failures++;                                                      \
            printf("FAIL: %s (got %.9g, want %.9g, tol %.3g)\n",               \
                   msg, _a, _e, (double)(tol));                                \
        }                                                                      \
    } while(0)

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

    printf("test_wheels: %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
