/* The ascent law, flown in a spreadsheet.
   AscentLaw is pure (no Vehicle, no Frame), so the honest test of a guidance law
   is to CLOSE THE LOOP: integrate its aim against the body's gravity for a few
   thousand ticks and look at the orbit that comes out. A pin on "the aim at 5 km
   altitude is 30 deg over vertical" proves the schedule; this pin proves the
   mission. Moho's numbers are the shipped ones (res/systems/ksp_system.json via
   system.cpp's G*mass) so the delta-v budget is the game's, not a guess.

   The prototype this replaces (reports/autopilots2026_10_24) got the plane right
   and the SHAPE wrong -- 16.9 x 50.1 km against a 50 km ask -- so the shape
   numbers, not the phase names, are the load-bearing checks here. */
#include "guidance.h"
#include "orbit.h"
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <string>

static int checks = 0, failures = 0;
#define CHECK_TRUE(cond, msg) do { checks++; if(!(cond)) { failures++; printf("FAIL: %s\n", msg); } } while(0)
#define CHECK_NEAR(a, e, tol, msg) do { checks++; const double _a=(a), _e=(e); \
    if(!std::isfinite(_a) || !std::isfinite(_e) || std::fabs(_a-_e) > (tol)) { failures++; \
    printf("FAIL: %s (got %.6g, want %.6g, tol %.3g)\n", msg, _a, _e, (double)(tol)); } } while(0)

static const double G_CONST = 6.674e-11;            // system.cpp:161
static const double MU_MOHO = G_CONST * 2.526e21;   // ksp_system.json Moho mass
static const double R_MOHO  = 250000.0;             // Moho radius, no sea level
static const double THRUST  = 50000.0;              // racer: one engine part
static const double M_FULL  = 2670.23;              // capsule+wheel+tank+engine
static const double MDOT    = 11.36;                // hydrogen 5.68 + lox 5.68 kg/s
static const double PROPPANT = 840.0;             // racer: one fuel_tank of h2+lox

struct Flight {
    bool done = false;
    bool failed = false;
    std::string why;
    double t = 0.0, peri = 0.0, apo = 0.0, fuel = 0.0, v_max = 0.0, inc_deg = 0.0;
    int phases = 0;             // bitmask of phases visited
    double min_clear = 1e30;    // lowest ground clearance after liftoff
    std::string trail;
};

/* One closed-loop run: the law's aim becomes an acceleration, gravity is the
   only other force (Moho has no air), and mass falls away as the engine eats.
   `accel_cap` clamps what the stage can put out, so a case can model a weak
   stage without changing anything else. */
static Flight simulate(const AscentTarget &target, double dt = 1.0/120.0,
                       int max_s = 900, bool trace = false) {
    Flight f;
    AscentLaw law(target);
    glm::dvec3 pos(R_MOHO + 1.0, 0.0, 0.0);      // standing on the equator, +x
    glm::dvec3 vel(0.0, 0.0, 0.0);
    double mass = M_FULL;
    bool lifted = false;
    std::string last_phase = "";
    for(int step = 0; step * dt < max_s; step++) {
        GuidanceState s;
        s.pos = pos; s.vel = vel;
        s.mu = MU_MOHO; s.sea_r = R_MOHO;
        s.spin_axis = glm::dvec3(0.0, 0.0, 1.0);
        s.accel_max = mass > M_FULL - PROPPANT ? THRUST / mass : 0.0;
        const double r = glm::length(pos);
        s.grounded = !lifted && r - R_MOHO < 100.0 && glm::length(vel) < 3.0;
        s.railed = false;
        const AscentCommand c = law.step(s, dt);
        if(c.burn) { mass -= MDOT * c.throttle * dt; f.fuel += MDOT * c.throttle * dt; }
        if(std::string(law.phaseName()) != last_phase) {
            last_phase = law.phaseName();
            f.trail += last_phase + " ";
            f.phases |= (1 << (int)law.phase());
            if(trace) {
                const OrbitElements o = computeOrbitElements(pos, vel, MU_MOHO);
                printf("      -> %-7s t=%6.1fs alt=%8.1f km v=%7.1f m/s pitch=%5.1f deg "
                       "peri=%8.1f km apo=%8.1f km node=%8.1f km\n", last_phase.c_str(),
                       (step+1)*dt, (r - R_MOHO)/1e3, glm::length(vel), law.pitch_deg(),
                       periapsisAsl(o, R_MOHO)/1e3,
                       (o.traj == TrajectoryClass::Elliptic ? apoapsisAsl(o, R_MOHO) : -1.0)/1e3,
                       law.node_r()/1e3);
            }
        }
        glm::dvec3 acc(0.0);
        if(c.burn && glm::dot(c.aim, c.aim) > 0.0) {
            acc = glm::normalize(c.aim) * (s.accel_max * c.throttle);
        }
        // Semi-implicit Euler: velocity first, then position with the new
        // velocity. Explicit Euler gains energy in orbit and would fake a
        // higher apoapsis than the law actually earned.
        vel += acc * dt;
        const double r2 = glm::dot(pos, pos);
        vel += (-(MU_MOHO / (r2 * std::sqrt(r2))) * pos) * dt;
        pos += vel * dt;
        f.t = (step + 1) * dt;
        f.v_max = std::max(f.v_max, glm::length(vel));
        const double clr = glm::length(pos) - R_MOHO;
        if(glm::length(vel) > 10.0) { lifted = true; }
        if(lifted && clr > 0.0) { f.min_clear = std::min(f.min_clear, clr); }
        if(lifted && clr < 0.0) { f.trail += "IMPACT "; break; }
        if(!law.active()) {
            f.done = law.phase() == AscentPhase::Done;
            f.failed = law.phase() == AscentPhase::Failed;
            f.why = law.why();
            break;
        }
    }
    const OrbitElements o = computeOrbitElements(pos, vel, MU_MOHO);
  if(!f.failed && !f.done) { f.why = "ran out of sim time"; }
    f.peri = periapsisAsl(o, R_MOHO);
    f.apo = (o.traj == TrajectoryClass::Elliptic) ? apoapsisAsl(o, R_MOHO) : -1.0;
    // The plane the burn actually built, measured against the body's spin axis
    // -- the same reference the navball's inclination uses.
    RefPlane ref;
    ref.n_hat = glm::dvec3(0.0, 0.0, 1.0);
    ref.x_hat0 = glm::dvec3(1.0, 0.0, 0.0);
    f.inc_deg = glm::degrees(orbitPlaneAngles(o, ref).inc);
    return f;
}

/* v1's envelope, pinned as a limit rather than wished away. This profile is a
   gravity turn to a transfer plus one burn at the node, and that shape has a
   working range: on Moho with the racer it lands 50 km, 150 km and 250 km asks
   inside 4%, and misses both ends of the range -- a 10 km ask comes back
   10 x 395 km (the cutoff fires at 10 km apoapsis, before the ship has any
   orbital speed to shape), a 400 km ask comes back 400 x 519 km (the ascent
   carries too much speed to stop at 400). What the test insists on is that the
   law SAYS it missed: before the shape ceiling was added, a 400 km ask read
   "Done" at 400 x 519 km, which is an orbit nobody asked for. Low orbits are
   their own problem (a 10 km orbit is 30 km of terrain clearance, not a
   mission), and the user has explicitly deferred them. */
static void test_asks_it_cannot_land_say_so() {
    printf("== the envelope: an ask this profile misses is reported, not claimed ==\n");
    for(double alt : {10e3, 400e3}) {
        AscentTarget t; t.apo_asl = t.peri_asl = alt; t.inc_rad = -1.0;
        const Flight f = simulate(t, 1.0/120.0, 1500);
        printf("   [moho %.0fkm] done=%d failed=%d peri=%.1f km apo=%.1f km why=\"%s\"\n",
               alt/1e3, (int)f.done, (int)f.failed, f.peri/1e3, f.apo/1e3, f.why.c_str());
        CHECK_TRUE(!f.done, "an out-of-envelope ask never reads Done");
        CHECK_TRUE(f.failed, "and it ends Failed, with the reason recorded");
        CHECK_TRUE(f.why.size() > 0, "why() is not empty");
    }
}

static void test_it_actually_gets_to_orbit() {
    printf("== the law, flown: Moho pad to a 100 km circular orbit ==\n");
    AscentTarget t; t.apo_asl = t.peri_asl = 100e3; t.inc_rad = -1.0;
    const Flight f = simulate(t, 1.0/120.0, 900, true);
    printf("   [moho 100km] done=%d t=%.1fs peri=%.1f km apo=%.1f km gap=%.1f km "
           "fuel=%.0f kg vmax=%.0f m/s trail=%s\n",
           (int)f.done, f.t, f.peri/1e3, f.apo/1e3, (f.apo-f.peri)/1e3, f.fuel,
           f.v_max, f.trail.c_str());
    CHECK_TRUE(f.done, "the run reaches Done rather than running out of sky");
    CHECK_TRUE(f.peri >= 100e3 - 2e3, "periapsis at or above the ask (the orbit is real)");
    CHECK_NEAR(f.apo, 100e3, 6e3, "apoapsis within 6 km of the ask (the SHAPE, which is "
               "what the old prototype got wrong)");
    CHECK_TRUE(f.min_clear > 0.0, "never came back down through the ground");
    CHECK_TRUE(f.fuel < 840.0, "spent less than the 840 kg the tanks hold");
}

static void test_the_shape_holds_at_other_altitudes() {
    printf("== the same law at other asks ==\n");
    for(double alt : {50e3, 150e3, 250e3}) {
        AscentTarget t; t.apo_asl = t.peri_asl = alt; t.inc_rad = -1.0;
        const Flight f = simulate(t, 1.0/120.0, 2400);
        printf("   [moho %.0fkm] done=%d t=%.1fs peri=%.1f km apo=%.1f km gap=%.1f km fuel=%.0f kg\n",
               alt/1e3, (int)f.done, f.t, f.peri/1e3, f.apo/1e3, (f.apo-f.peri)/1e3, f.fuel);
        CHECK_TRUE(f.done, "reaches Done at this altitude too");
        CHECK_NEAR(f.peri, alt, 0.1 * alt + 3e3, "periapsis near the ask");
        CHECK_NEAR(f.apo, alt, 0.06 * alt + 3e3, "apoapsis within 6% of the ask");
    }
}

static void test_the_launch_heading_buys_the_inclination() {
    printf("== inclination comes from the heading, not from luck ==\n");
    /* Flown, not tabled: launch due east from the equator and the orbit must
       end in the equatorial plane; ask 45 deg and it must end at 45; ask 120
       (a retrograde-ish launch) and it must end at 120. This is the one pair of
       numbers in the law that is textbook trig (cos i = cos az * cos lat), so
       the test checks the answer, not the algebra. */
    for(double inc : {0.0, 45.0, 90.0, 120.0}) {
        AscentTarget t; t.apo_asl = t.peri_asl = 100e3;
        t.inc_rad = glm::radians(inc);
        const Flight f = simulate(t, 1.0/120.0, 900);
        printf("   [inc %.0f deg] flown inc %.2f deg (run peri=%.1f apo=%.1f fuel=%.0f kg)\n",
               inc, f.inc_deg, f.peri/1e3, f.apo/1e3, f.fuel);
        CHECK_NEAR(f.inc_deg, inc, 1.0, "the orbit plane the launch built is the asked inclination");
    }
    /* And the heading itself: at the equator the inclination IS the heading's
       angle from due east (the law measures azimuth from east because east is
       what falls out of the cross product, not from north). */
    AscentTarget t; t.apo_asl = t.peri_asl = 100e3; t.inc_rad = glm::radians(45.0);
    AscentLaw law(t);
    GuidanceState s; s.pos = glm::dvec3(R_MOHO, 0.0, 0.0); s.vel = glm::dvec3(0.0);
    s.mu = MU_MOHO; s.sea_r = R_MOHO; s.spin_axis = glm::dvec3(0.0, 0.0, 1.0);
    s.accel_max = 18.0; s.grounded = true;
    law.step(s, 1.0/60.0);
    const glm::dvec3 east(0.0, 1.0, 0.0);
    const double az = glm::degrees(std::acos(std::clamp(glm::dot(law.launchHeading(), east), -1.0, 1.0)));
    CHECK_NEAR(az, 45.0, 0.01, "the pad heading is 45 deg from due east");
}

static void test_the_law_fails_loudly_not_silently() {
    printf("== the doors: rails, a dead stage, no conic ==\n");
    AscentTarget t; t.apo_asl = t.peri_asl = 100e3; t.inc_rad = -1.0;
    AscentLaw law(t);
    GuidanceState s; s.pos = glm::dvec3(R_MOHO, 0.0, 0.0); s.vel = glm::dvec3(0.0, 1.0, 0.0);
    s.mu = MU_MOHO; s.sea_r = R_MOHO; s.spin_axis = glm::dvec3(0.0, 0.0, 1.0);
    s.accel_max = 18.0;
    s.railed = true;
    const AscentCommand railed = law.step(s, 1.0/60.0);
    CHECK_NEAR(glm::length(railed.aim), 0.0, 0.0,
               "a railed hull gets no commands (its thrust would be silently dropped)");
    CHECK_TRUE(std::string(law.why()).size() > 0, "and the reason is recorded");

    s.railed = false;
    s.accel_max = 0.0;                       // the stage cannot push
    const AscentCommand dead = law.step(s, 1.0/60.0);
    CHECK_NEAR(glm::length(dead.aim), 0.0, 0.0, "a dead stage buys no aim");
    CHECK_TRUE(law.phase() == AscentPhase::Failed, "and the law says failed");
    CHECK_TRUE(std::string(law.why()).find("thrust") != std::string::npos, "with the reason");

    AscentLaw law2(t);
    s.accel_max = 18.0; s.mu = 0.0;
    law2.step(s, 1.0/60.0);
    CHECK_TRUE(law2.phase() == AscentPhase::Failed, "no mu, no mission");
}

static void test_done_is_a_periapsis_not_a_speed() {
    printf("== Done means the ground is cleared, not that the ship is fast ==\n");
    AscentTarget t; t.apo_asl = t.peri_asl = 100e3; t.inc_rad = -1.0;
    AscentLaw law(t);
    GuidanceState s; s.mu = MU_MOHO; s.sea_r = R_MOHO; s.accel_max = 18.0;
    s.spin_axis = glm::dvec3(0.0, 0.0, 1.0);
    s.pos = glm::dvec3(R_MOHO + 100e3, 0.0, 0.0);
    // Moho's escape speed at 350 km is sqrt(2) x 694 = 981 m/s, so 1100 is a
    // departure, not an orbit: no apoapsis at all.
    s.vel = glm::dvec3(0.0, 1100.0, 0.0);
    law.step(s, 1.0/60.0);
    const OrbitElements o = law.elements();
    CHECK_TRUE(o.traj != TrajectoryClass::Elliptic, "that state is an escape, not an orbit");
    CHECK_TRUE(law.phase() != AscentPhase::Done, "so it is not Done, however fast it is");

    // Faster than orbital but still bound: periapsis sits AT the ask, apoapsis
    // 1500 km past it. A rule that only looked for periapsis >= ask would call
    // this mission accomplished.
    AscentLaw law3(t);
    s.vel = glm::dvec3(0.0, 900.0, 0.0);
    law3.step(s, 1.0/60.0);
    printf("   [900 m/s at the ask] peri=%.1f km apo=%.1f km phase=%s\n",
           periapsisAsl(law3.elements(), R_MOHO)/1e3, apoapsisAsl(law3.elements(), R_MOHO)/1e3,
           law3.phaseName());
    CHECK_TRUE(periapsisAsl(law3.elements(), R_MOHO) >= 100e3, "its periapsis IS at the ask");
    CHECK_TRUE(apoapsisAsl(law3.elements(), R_MOHO) > 100e3 + 6e3, "and its apoapsis is far past it");
    CHECK_TRUE(law3.phase() != AscentPhase::Done, "so a 100 x 1850 km orbit is not a 100 x 100 km orbit");
    s.vel = glm::dvec3(0.0, 900.0, 0.0);

    AscentLaw law2(t);
    const double v_circ = std::sqrt(MU_MOHO / (R_MOHO + 100e3));
    s.pos = glm::dvec3(R_MOHO + 100e3, 0.0, 0.0);
    s.vel = glm::dvec3(0.0, v_circ, 0.0);   // a made orbit at the ask
    law2.step(s, 1.0/60.0);
    CHECK_NEAR(v_circ, 693.7, 0.5, "Moho's 100 km circular speed (the budget this law must beat)");
    CHECK_TRUE(law2.phase() == AscentPhase::Done, "an orbit at the ask reads Done");
}

static void test_a_pad_ship_aims_at_the_sky_and_never_at_nan() {
    printf("== the pad: no velocity, no NaN ==\n");
    AscentTarget t; t.apo_asl = t.peri_asl = 100e3; t.inc_rad = -1.0;
    AscentLaw law(t);
    GuidanceState s; s.pos = glm::dvec3(R_MOHO, 0.0, 0.0); s.vel = glm::dvec3(0.0);
    s.mu = MU_MOHO; s.sea_r = R_MOHO; s.spin_axis = glm::dvec3(0.0, 0.0, 1.0);
    s.accel_max = 18.0; s.grounded = true;
    const AscentCommand c = law.step(s, 1.0/60.0);
    CHECK_TRUE(std::isfinite(c.aim.x) && std::isfinite(c.aim.y) && std::isfinite(c.aim.z),
               "a standing still ship gets a finite aim");
    CHECK_NEAR(glm::dot(glm::normalize(c.aim), glm::normalize(s.pos)), 1.0, 1e-9,
               "and it points straight up off the pad");
    CHECK_NEAR(c.throttle, 1.0, 0.0, "full throttle from the pad");
    CHECK_TRUE(c.burn, "and the engine is lit");
}

int main() {
    test_a_pad_ship_aims_at_the_sky_and_never_at_nan();
    test_done_is_a_periapsis_not_a_speed();
    test_the_law_fails_loudly_not_silently();
    test_the_launch_heading_buys_the_inclination();
    test_it_actually_gets_to_orbit();
    test_the_shape_holds_at_other_altitudes();
    test_asks_it_cannot_land_say_so();
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
