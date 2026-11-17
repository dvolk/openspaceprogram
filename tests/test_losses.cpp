/* test_losses: the burn totals add up (src/losses.h, #675).

   The claim #675 asks to pin is that the three losses and the delta-v the tanks
   paid for are ONE decomposition, not three numbers measured three ways:

     dv - steer - grav - drag == the change in SPEED the BURN bought

   Proving that against the algebra alone would be circular -- the four terms are
   built from the same dot products, so a test that only re-arranges them agrees by
   construction. So this flies one: a hull burns, coasts, relights and falls through
   air, integrating the very forces it hands lossStep(), and the totals are compared
   against what the hull's speed actually did while the engine was lit. A sign slip,
   a wrong mass basis, a force taken from a different substep, or a loss scored
   against the wrong vector all show up as a gap.

   "While the engine was lit" is not a fudge to make the sums close. The totals are
   burn-gated by design (losses.h), so a coast's own speed change -- 67 m/s in the
   baseline run below, all of it gravity picking the hull back up -- belongs to no
   burn and is charged to none. The gap that IS left is the integrator's, and it is
   pinned to shrink with the timestep rather than to a number that quietly grows.

   Header-only module, so this links nothing but itself. */
#include "losses.h"

#include <cmath>
#include <cstdio>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>   // angleAxis / mat3_cast, for the frame test
#include <glm/gtx/norm.hpp>         // glm::length2
#include "tchecks.h"

// Moho, so the numbers are the game's and not invented (src/system.cpp's G * mass).
static const double G_CONST = 6.674e-11;
static const double MU_MOHO = G_CONST * 2.526e21;
static const double R_MOHO = 250000.0;
static const double THRUST = 50000.0;   // N, one engine
static const double MDOT = 11.36;       // kg/s, the same stage
static const double M_FULL = 2670.23;   // racer: capsule+wheel+tank+engine
static const double PROPPANT = 1100.0;  // enough for both burns to run their window

struct Flown {
    AscentLosses L;
    double gain_lit = 0.0;    // speed change during the substeps that had thrust
    double gain_coast = 0.0;  // and during the ones that did not (charged to nobody)
    double dv_summed = 0.0;   // an independent sum of |F_t| h / m, m/s
    double coast_gap = 0.0;   // gain_lit + gain_coast vs the totals: what the gate leaves out
};

/* Fly it. `tilt` is how far the nozzle points off the velocity line, rad (0 =
   dead prograde, so the steering loss has a known answer: none). `dragK` is a
   quadratic air bill, -dragK |v| v; 0 = vacuum. The burns run [0, t1) and
   [t2, t3); a coast between them is what tests the burn gate. */
static Flown fly(double dt, double t1, double t2, double t3, double tilt,
                 double dragK) {
    Flown f;
    glm::dvec3 pos(R_MOHO + 1.0, 0.0, 0.0);
    glm::dvec3 vel(0.0, 0.0, 0.0);
    double mass = M_FULL;
    const int steps = (int)(t3 / dt);
    for(int i = 0; i < steps; i++) {
        const double t = i * dt;
        const bool want = (t < t1) || (t >= t2 && t < t3);
        const bool has_propellant = mass > M_FULL - PROPPANT + 1e-9;
        const glm::dvec3 up = glm::normalize(pos);
        const glm::dvec3 gravF = -up * (MU_MOHO * mass / glm::length2(pos));
        const double speed = glm::length(vel);
        glm::dvec3 thrust(0.0);
        if(want && has_propellant) {
            /* The aim: the velocity line rotated `tilt` toward the local forward.
               At speed 0 there is no line, so the pad burns straight up -- the case
               losses.h has a floor for. */
            const glm::dvec3 vhat = (speed > 1.0) ? vel / speed : up;
            glm::dvec3 fwd = up - vhat * glm::dot(up, vhat);
            if(glm::length2(fwd) < 1e-12) { fwd = glm::dvec3(0.0, 1.0, 0.0); }
            else { fwd = glm::normalize(fwd); }
            /* Normalize the aim before scaling it. A hull that has just left the pad
               has vhat nearly parallel to up, so the fwd above is a cancellation and
               cos*vhat + sin*fwd is unit only to ~1e-5 -- which would put a wobble of
               this harness's own into dv, the one quantity compared between a prograde
               and a tilted burn. In the game the thrust magnitude comes from engine
               specs, never from combining two unit vectors, so normalizing here models
               that seam honestly. */
            thrust = glm::normalize(std::cos(tilt) * vhat + std::sin(tilt) * fwd) * THRUST;
        }
        const glm::dvec3 aero = -vel * (dragK * speed);
        lossStep(f.L, vel, mass, dt, thrust, gravF, aero);
        /* The same forces, on the same clock, into the integrator. Semi-implicit
           Euler: velocity first (orbit.h's own harness does the same -- explicit
           Euler gains energy in orbit). */
        const glm::dvec3 vel_new = vel + (thrust + gravF + aero) * (dt / mass);
        const double speed_new = glm::length(vel_new);
        const double gained = speed_new - speed;
        if(glm::length2(thrust) > 0.0) { f.gain_lit += gained; }
        else { f.gain_coast += gained; }
        vel = vel_new;
        pos += vel * dt;
        if(glm::length2(thrust) > 0.0) {
            f.dv_summed += THRUST * dt / mass;   // the tanks' side, on the mass it spent
            mass -= MDOT * dt;
        }
    }
    f.coast_gap = (f.gain_lit + f.gain_coast) - (f.L.dv - f.L.steer - f.L.grav - f.L.drag);
    return f;
}

int main() {
    const double dt = 1.0 / 240.0;
    // One 40 s burn, 20 s of coast, a 30 s relight. Prograde, vacuum.
    const Flown a = fly(dt, 40.0, 60.0, 90.0, 0.0, 0.0);
    printf("[prograde vacuum] dv=%.1f grav=%.1f drag=%.1f steer=%.1f burns=%d burn_s=%.1f\n",
           a.L.dv, a.L.grav, a.L.drag, a.L.steer, a.L.burns, a.L.burn_s);
    printf("                   speed gain while lit=%.2f, while coasting=%.2f, "
           "identity gap=%.3f m/s\n", a.gain_lit, a.gain_coast,
           (a.L.dv - a.L.steer - a.L.grav - a.L.drag) - a.gain_lit);

    // --- the decomposition, against the flight ----------------------------
    CHECK_NEAR(a.L.dv - a.L.steer - a.L.grav - a.L.drag, a.gain_lit, 0.25,
               "dv minus the three losses is the speed the burns bought");

    // --- what the gate leaves out, and that it is on purpose --------------
    // A 20 s coast on a rising path costs speed, and the totals do not see it. If
    // someone dropped the burn gate -- MechJeb's shape -- this number moves and the
    // check above still passes: it is THIS pin that says the gate is load-bearing.
    CHECK_NEAR(a.coast_gap, a.gain_coast, 0.25,
               "the totals exclude the coast's speed change (a loss is a burn's price)");
    CHECK_TRUE(a.gain_coast < -20.0,
               "and the coast really did change the speed: the gate is not vacuous");

    // --- the burn gate, on the clock and on the count ---------------------
    CHECK_NEAR(a.L.burn_s, 70.0, dt, "the totals cover 40 s + 30 s of burning, not the coast");
    CHECK_TRUE(a.L.burns == 2, "two burns, counted on their rising edges");

    // --- a prograde burn throws nothing away -------------------------------
    // If steering loss were scored against the nose, the hull's heading, or last
    // substep's velocity, this would not be zero.
    CHECK_NEAR(a.L.steer, 0.0, 1e-9, "a burn dead along the velocity line has no steering loss");
    CHECK_NEAR(a.L.drag, 0.0, 1e-9, "and no drag loss, in a vacuum");

    // --- gravity loss is the uphill part, not the whole pull ---------------
    // A prograde burn from the pad climbs, so the body pulls against it. It is NOT
    // the full g x t: by the end the hull flies nearly sideways and the pull is
    // across its path, which costs nothing.
    CHECK_TRUE(a.L.grav > 0.0, "climbing costs gravity loss");
    CHECK_TRUE(a.L.grav < (MU_MOHO / (R_MOHO * R_MOHO)) * a.L.burn_s,
               "and only the component along the path: g x burn time is the whole pull, not the loss");

    // --- the tanks' side of the same number --------------------------------
    CHECK_NEAR(a.L.dv, a.dv_summed, 1e-9, "dv is |F| h / m summed on the integrator's mass");

    // --- the gap is the integrator's, so it shrinks with the timestep ------
    {
        const Flown half = fly(dt / 2.0, 40.0, 60.0, 90.0, 0.0, 0.0);
        const double gap_a = std::fabs((a.L.dv - a.L.grav) - a.gain_lit);
        const double gap_h = std::fabs((half.L.dv - half.L.grav) - half.gain_lit);
        printf("[dt %.5f -> %.5f] identity gap %.4f -> %.4f m/s\n", dt, dt / 2.0, gap_a, gap_h);
        CHECK_TRUE(gap_h < gap_a, "halving the step halves the gap: it is discretisation, not a term that is missing");
    }

    // --- tilted: steering loss appears, the identity still closes ----------
    const Flown b = fly(dt, 40.0, 60.0, 90.0, 0.35, 0.0);
    printf("[tilt 20deg]     dv=%.1f grav=%.1f drag=%.1f steer=%.1f burns=%d speed gain lit=%.2f\n",
           b.L.dv, b.L.grav, b.L.drag, b.L.steer, b.L.burns, b.gain_lit);
    CHECK_NEAR(b.L.dv - b.L.steer - b.L.grav - b.L.drag, b.gain_lit, 0.25,
               "a burn pointed 20 deg off the velocity line still closes");
    // dv is the same figure it was prograde (same thrust, same clock, same mass),
    // and the tilt moves part of it into steering loss.
    CHECK_NEAR(b.L.dv, a.L.dv, 1e-6, "pointing sideways does not change what the tanks paid");
    CHECK_TRUE(b.L.steer > 50.0, "and it does change what the pointing threw away");
    // (1 - cos 20 deg) = 0.0603 of the impulse, and the tilt is held all flight.
    CHECK_NEAR(b.L.steer / b.L.dv, 1.0 - std::cos(0.35), 0.02,
               "steering loss is (1 - cos of the angle off the velocity line), measured");

    // --- with air: the drag bill joins and the identity still closes -------
    const Flown c = fly(dt, 40.0, 60.0, 90.0, 0.0, 0.0025);
    printf("[prograde +air]   dv=%.1f grav=%.1f drag=%.1f steer=%.1f burns=%d speed gain lit=%.2f\n",
           c.L.dv, c.L.grav, c.L.drag, c.L.steer, c.L.burns, c.gain_lit);
    CHECK_NEAR(c.L.dv - c.L.steer - c.L.grav - c.L.drag, c.gain_lit, 0.25,
               "adding a quadratic air bill keeps the four terms one decomposition");
    CHECK_TRUE(c.L.drag > 0.0, "the air took something along the velocity line");
    CHECK_TRUE(c.L.drag > a.L.drag, "and it took more than the vacuum run did");

    // --- a shared rotation of everything changes nothing -------------------
    // The flight model hands the forces in the hull's frame and the velocity from
    // the non-rotating one; the two are reconciled by one rotation (vehicle.cpp's
    // lossTick). If any term were dotted across that seam, this would move.
    {
        const glm::dvec3 axis = glm::normalize(glm::dvec3(0.3, 0.5, 0.81));
        const glm::dmat3 rot = glm::mat3_cast(glm::angleAxis(0.9, axis));
        AscentLosses one, two;
        for(int i = 0; i < 500; i++) {
            const glm::dvec3 v(120.0, -40.0, 7.5);
            const glm::dvec3 ft(30000.0, 9000.0, -2000.0);
            const glm::dvec3 fg(-100.0, 2000.0, 400.0);
            const glm::dvec3 fa(-250.0, -60.0, 12.0);
            lossStep(one, v, 2400.0, 0.01, ft, fg, fa);
            lossStep(two, rot * v, 2400.0, 0.01, rot * ft, rot * fg, rot * fa);
        }
        CHECK_NEAR(one.dv, two.dv, 1e-12, "dv is frame-free");
        CHECK_NEAR(one.steer, two.steer, 1e-12, "and so is every loss: one frame, one v_hat");
        CHECK_NEAR(one.grav, two.grav, 1e-12, "gravity loss included");
        CHECK_NEAR(one.drag, two.drag, 1e-12, "drag loss included");
    }

    // --- the pad: no velocity line, so the split waits ---------------------
    // Not a cosmetic case: on a body that does not turn, the first substeps have
    // exactly zero velocity, and an unguarded normalize hands the whole first
    // impulse to steering loss (the MechJeb behaviour #675 records as a deviation).
    {
        AscentLosses L;
        const glm::dvec3 up(0.0, 0.0, 1.0);
        lossStep(L, glm::dvec3(0.0), 1000.0, 0.01, up * 20000.0, -up * 2700.0,
                 glm::dvec3(0.0));
        CHECK_NEAR(L.dv, 0.2, 1e-12, "the impulse is still charged at zero speed");
        CHECK_NEAR(L.steer, 0.0, 1e-12, "and nothing is called steering loss without a line to score");
        CHECK_NEAR(L.grav, 0.0, 1e-12, "no gravity loss either, at the same zero");
        CHECK_TRUE(L.burns == 1, "the burn has started");
    }

    // --- a coast charges nothing, and the next burn is a new burn ----------
    {
        AscentLosses L;
        const glm::dvec3 v(0.0, 400.0, 0.0);
        lossStep(L, v, 1000.0, 0.01, glm::dvec3(0.0, 1000.0, 0.0),
                 glm::dvec3(0.0, 0.0, -2700.0), glm::dvec3(0.0));
        const double afterBurn = L.dv;
        for(int i = 0; i < 1000; i++) {   // 10 s of falling with the engine shut
            lossStep(L, v, 1000.0, 0.01, glm::dvec3(0.0),
                     glm::dvec3(0.0, 0.0, -2700.0), glm::dvec3(-10.0, 0.0, 0.0));
        }
        CHECK_NEAR(L.dv, afterBurn, 1e-12, "coasting spends no delta-v");
        CHECK_NEAR(L.grav, 0.0, 1e-12, "and it does not charge gravity for the fall (#675's gate)");
        CHECK_NEAR(L.drag, 0.0, 1e-12, "nor the air for what the hull coasted through");
        CHECK_NEAR(L.burn_s, 0.01, 1e-12, "the clock stopped with the engine");
        lossStep(L, v, 1000.0, 0.01, glm::dvec3(0.0, 1000.0, 0.0),
                 glm::dvec3(0.0, 0.0, -2700.0), glm::dvec3(0.0));
        CHECK_TRUE(L.burns == 2, "relit, it is the second burn, not a continuation");
    }

    // --- guards: a mass or a clock that is not positive bills nothing ------
    {
        AscentLosses L;
        const glm::dvec3 v(0.0, 400.0, 0.0);
        lossStep(L, v, 0.0, 0.01, glm::dvec3(0.0, 1000.0, 0.0), glm::dvec3(0.0),
                 glm::dvec3(0.0));
        lossStep(L, v, 1000.0, 0.0, glm::dvec3(0.0, 1000.0, 0.0), glm::dvec3(0.0),
                 glm::dvec3(0.0));
        lossStep(L, glm::dvec3(0.0), 0.0, 0.0, glm::dvec3(0.0), glm::dvec3(0.0),
                 glm::dvec3(0.0));
        CHECK_NEAR(L.dv, 0.0, 0.0, "no mass, no step, no bill");
        CHECK_TRUE(L.burns == 0, "and no burn was ever counted");
    }

    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
