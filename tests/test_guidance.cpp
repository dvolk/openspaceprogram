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
#include "vehicle.h"   // a bare Vehicle for the releaseAscent() pin (TCOMMON_OBJS links it)
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
    double min_clear = 1e30;    // lowest ground clearance after liftoff, negative = hit
    double node_at_circ = 0.0;  // what the law captured as its node, entering Circ
    double node_at_end = 0.0;   // and what it still says at the end
    std::string trail;
};

/* One closed-loop run: the law's aim becomes an acceleration, gravity is the
   only other force (Moho has no air), and mass falls away as the engine eats.
   The dry stage is modelled the way the game reports it: availableAccel() is a
   potential that never reads the tanks (#607), so accel_max stays healthy at
   zero fuel and it is has_propellant that goes false. Getting this backwards
   would have the test certify a path the driver never takes.
   `start` lets a flight begin somewhere other than the equator, because the
   launch-heading maths has a cos(lat) divisor that is exactly 1 at the equator
   -- a test fleet that only ever lifts off from there cannot tell a correct
   divisor from a missing one. `thrust` is the stage's rated thrust, so a case
   can be weaker than its own planet. */
static Flight simulate(const AscentTarget &target, double dt = 1.0/120.0,
                       int max_s = 900, bool trace = false,
                       glm::dvec3 start = glm::dvec3(R_MOHO + 1.0, 0.0, 0.0),
                       double thrust = THRUST) {
    Flight f;
    AscentLaw law(target);
    glm::dvec3 pos = start;
    glm::dvec3 vel(0.0, 0.0, 0.0);
    double mass = M_FULL;
    bool lifted = false;
    std::string last_phase = "";
    for(int step = 0; step * dt < max_s; step++) {
        GuidanceState s;
        s.pos = pos; s.vel = vel;
        s.mu = MU_MOHO; s.sea_r = R_MOHO;
        s.spin_axis = glm::dvec3(0.0, 0.0, 1.0);
        s.accel_max = thrust / mass;     // a potential: fuel-blind, as in the game
        s.has_propellant = mass > M_FULL - PROPPANT + 1e-9;
        const double r = glm::length(pos);
        s.grounded = !lifted && r - R_MOHO < 100.0 && glm::length(vel) < 3.0;
        s.railed = false;
        const AscentCommand c = law.step(s);
        if(c.burn) { mass -= MDOT * c.throttle * dt; f.fuel += MDOT * c.throttle * dt; }
        if(std::string(law.phaseName()) != last_phase) {
            last_phase = law.phaseName();
            f.trail += last_phase + " ";
            f.phases |= (1 << (int)law.phase());
            if(law.phase() == AscentPhase::Circ) { f.node_at_circ = law.node_r(); }
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
        /* Record the clearance whatever it is. The first version only kept
           positive values, so the "never came back down through the ground" pin
           could not fail: min_clear was positive by construction. */
        if(lifted) { f.min_clear = std::min(f.min_clear, clr); }
        if(lifted && clr < 0.0) { f.trail += "IMPACT "; break; }
        if(!law.active()) {
            f.done = law.phase() == AscentPhase::Done;
            f.failed = law.phase() == AscentPhase::Failed;
            f.why = law.why();
            break;
        }
    }
    f.node_at_end = law.node_r();
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
    CHECK_TRUE(f.trail.find("IMPACT") == std::string::npos, "never came back down through the ground");
    CHECK_TRUE(f.min_clear > 0.0, "and that clearance was measured, not filtered: the "
               "harness used to skip negative values, which made this pin unfailable");
    CHECK_TRUE(f.fuel < 840.0, "spent less than the 840 kg the tanks hold");
    /* The node the burn is sized against is captured once, on entering Circ, and
       does not move while the burn is raising the apsis it is chasing (the old
       prototype read its own live apoapsis every tick and so kept moving the
       finish line as it ran toward it: a 16.9 x 50.1 km answer to a 50 km ask).
       Honest about its own reach: for a CIRCULAR ask the captured node and the
       ask's own radius are the same number, so this pin cannot tell the two
       spellings apart there -- the elliptical-ask test below is where they part. */
    CHECK_NEAR(f.node_at_end, f.node_at_circ, 0.0, "node_r() is the same number at the end as at the node");
    CHECK_TRUE(f.apo + R_MOHO > f.node_at_circ, "and the orbit it built is above that captured node");
}

/* apo_asl != peri_asl: an ask with two different apsides, which is what an
   Autopilot window will offer and which nothing else here flies. The ascent
   burns to the apoapsis ask, then the node burn stops at the LOWER of that node
   and the periapsis ask, so a 100 x 200 km ask should come back with periapsis
   at 100 and apoapsis near 200 -- not the 200 x 200 a rule that treated the two
   asks as one would build. */
static void test_an_elliptical_ask_keeps_both_apsides() {
    printf("== two different asks: 100 km periapsis, 200 km apoapsis ==\n");
    AscentTarget t; t.apo_asl = 200e3; t.peri_asl = 100e3; t.inc_rad = -1.0;
    const Flight f = simulate(t, 1.0/120.0, 900);
    printf("   [100x200] done=%d peri=%.1f km apo=%.1f km gap=%.1f km fuel=%.0f kg node=%.0f km\n",
           (int)f.done, f.peri/1e3, f.apo/1e3, (f.apo-f.peri)/1e3, f.fuel, f.node_at_circ/1e3);
    CHECK_TRUE(f.done, "the mixed ask is inside the profile's envelope");
    CHECK_NEAR(f.peri, 100e3, 13e3, "periapsis at the periapsis ask, not at the apoapsis one");
    CHECK_NEAR(f.apo, 200e3, 0.06 * 200e3 + 3e3, "apoapsis at the apoapsis ask");
    CHECK_NEAR(f.node_at_circ, R_MOHO + 200e3, 0.06 * 200e3 + 3e3,
               "and the node it captured is the apoapsis ask, so the cutoff really did bind on the pair");
}

/* Three ways the launch geometry degenerates, all of them reachable in the
   shipped system file: a body with no rotation block (Kerbol -- omega() is zero
   there, and the first version read omega() rather than the pole), a site
   standing on a pole, and a pole that happens to point along the axis the
   fallback seed used. The bugs those exposed were not cosmetic: a zero-length
   fallback left a RADIAL heading (the ship climbs for ever and never builds a
   plane), and a NaN one passed every "is this too small" gate on its way to the
   hull, because NaN < eps is false (#610 review). */
static void test_degenerate_geometry_still_points_somewhere_horizontal() {
    printf("== degenerate frames: a body that does not spin, and sites on a pole ==\n");
    struct Site { const char *name; glm::dvec3 pos; glm::dvec3 spin; };
    const Site sites[] = {
        {"no spin, equator",    {R_MOHO, 0.0, 0.0}, {0.0, 0.0, 0.0}},
        {"no spin, pole at +z", {0.0, 0.0, R_MOHO}, {0.0, 0.0, 0.0}},
        {"spin +x, site at +x", {R_MOHO, 0.0, 0.0}, {1.0, 0.0, 0.0}},
        {"spin +z, site at +z", {0.0, 0.0, R_MOHO}, {0.0, 0.0, 1.0}},
        {"tilted spin, pole",   {0.0, 0.0, R_MOHO}, {0.0, 1.0, 1.0}},
    };
    for(const Site &site : sites) {
        for(double inc : {-1.0, 45.0}) {
            AscentTarget t; t.apo_asl = t.peri_asl = 100e3; t.inc_rad = inc;
            AscentLaw law(t);
            GuidanceState s; s.pos = site.pos; s.vel = glm::dvec3(0.0);
            s.mu = MU_MOHO; s.sea_r = R_MOHO; s.spin_axis = site.spin;
            s.accel_max = 18.0; s.grounded = true;
            const AscentCommand c = law.step(s);
            const glm::dvec3 up = glm::normalize(site.pos);
            const glm::dvec3 h = law.launchHeading();
            printf("   [%-18s inc %4.0f] aim=(%5.2f,%5.2f,%5.2f) heading=(%6.3f,%6.3f,%6.3f) "
                   "dot(h,up)=%.4f why=\"%s\"\n", site.name, inc,
                   c.aim.x, c.aim.y, c.aim.z, h.x, h.y, h.z, glm::dot(h, up), law.why());
            CHECK_TRUE(std::isfinite(c.aim.x) && std::isfinite(c.aim.y) && std::isfinite(c.aim.z),
                       "a degenerate frame still gets a finite aim");
            CHECK_NEAR(glm::length(c.aim), 1.0, 1e-9, "and it is a direction, not a zero or shrunk one");
            CHECK_NEAR(glm::dot(h, up), 0.0, 1e-9, "the heading the turn follows is horizontal, never straight up");
        }
    }
}

/* East, not west; 60 deg, not 45. Two mutants the equator-only fleet could not
   see: a heading that mirrors around north builds the SAME inclination (cos i is
   even in the azimuth), and a missing cos(lat) divisor changes nothing at lat 0,
   where that divisor is exactly 1. */
static void test_the_heading_has_a_sense_and_a_working_cos_lat() {
    printf("== the heading: due east, and a site that is not on the equator ==\n");
    AscentTarget t; t.apo_asl = t.peri_asl = 100e3; t.inc_rad = -1.0;
    AscentLaw law(t);
    GuidanceState s; s.pos = glm::dvec3(R_MOHO, 0.0, 0.0); s.vel = glm::dvec3(0.0);
    s.mu = MU_MOHO; s.sea_r = R_MOHO; s.spin_axis = glm::dvec3(0.0, 0.0, 1.0);
    s.accel_max = 18.0; s.grounded = true;
    law.step(s);
    const glm::dvec3 east = glm::normalize(glm::cross(s.spin_axis, glm::normalize(s.pos)));
    printf("   [equator, no ask] heading=(%.3f,%.3f,%.3f), east=(%.3f,%.3f,%.3f), dot=%.3f\n",
           law.launchHeading().x, law.launchHeading().y, law.launchHeading().z,
           east.x, east.y, east.z, glm::dot(law.launchHeading(), east));
    CHECK_TRUE(glm::dot(law.launchHeading(), east) > 0.99,
               "with no inclination asked the ship launches due EAST, the way the body turns");

    /* 45 deg north, asking 60: the heading must sit 45 deg off east, because
       cos(az) = cos(i)/cos(lat) = 0.5/0.707. Drop the divisor and az becomes 60
       deg, which lands the orbit at 45 -- the ask and the site swap places. */
    const double lat = glm::radians(45.0);
    const glm::dvec3 start(R_MOHO * std::cos(lat), 0.0, R_MOHO * std::sin(lat));
    AscentTarget ask; ask.apo_asl = ask.peri_asl = 100e3; ask.inc_rad = glm::radians(60.0);
    AscentLaw law45(ask);
    GuidanceState s45; s45.pos = start; s45.vel = glm::dvec3(0.0);
    s45.mu = MU_MOHO; s45.sea_r = R_MOHO; s45.spin_axis = glm::dvec3(0.0, 0.0, 1.0);
    s45.accel_max = 18.0; s45.grounded = true;
    law45.step(s45);
    const glm::dvec3 up45 = glm::normalize(start);
    const glm::dvec3 east45 = glm::normalize(glm::cross(s45.spin_axis, up45));
    const double az = glm::degrees(std::acos(std::clamp(glm::dot(law45.launchHeading(), east45), -1.0, 1.0)));
    printf("   [45N ask 60] azimuth from east = %.2f deg (want 45.00)\n", az);
    CHECK_NEAR(az, 45.0, 0.01, "the cos(lat) divisor is in the heading maths");

    const Flight f = simulate(ask, 1.0/120.0, 900, false, start);
    printf("   [45N ask 60, flown] done=%d inc=%.2f deg peri=%.1f km apo=%.1f km fuel=%.0f kg\n",
           (int)f.done, f.inc_deg, f.peri/1e3, f.apo/1e3, f.fuel);
    CHECK_TRUE(f.done, "a launch from 45 deg north still gets to orbit");
    CHECK_NEAR(f.inc_deg, 60.0, 1.0, "and it ends in the asked plane, not the site's own latitude");
}

/* The game hands the law dt * time_accel, so a 10x flight gives it 1/6 s per
   tick -- 20x the finest rate here. The node lead and the cutoff floor scale
   with dt; the unguided gravity turn in the middle of the profile does not, so
   the shape the law lands moves with the tick rate. Measuring that drift beats
   treating shape_band as purely a finite-burn allowance (#611). */
static void test_the_shape_at_the_tick_rates_the_game_uses() {
    printf("== tick rate: one 100 km ask at 1/120, 1/60 and warped 1/6 ==\n");
    for(double dt : {1.0/120.0, 1.0/60.0, 10.0/60.0}) {
        AscentTarget t; t.apo_asl = t.peri_asl = 100e3; t.inc_rad = -1.0;
        const Flight f = simulate(t, dt, 900);
        printf("   [dt %.5fs] done=%d failed=%d peri=%.1f km apo=%.1f km fuel=%.0f kg t=%.1fs why=\"%s\"\n",
               dt, (int)f.done, (int)f.failed, f.peri/1e3, f.apo/1e3, f.fuel, f.t, f.why.c_str());
        CHECK_TRUE(f.done, "the ask is still reached at this tick rate");
        CHECK_NEAR(f.peri, 100e3, 13e3, "periapsis still at the ask");
        CHECK_NEAR(f.apo, 100e3, 13e3, "apoapsis still at the ask (the drift is real but small)");
    }
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
    law.step(s);
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
    const AscentCommand railed = law.step(s);
    CHECK_NEAR(glm::length(railed.aim), 0.0, 0.0,
               "a railed hull gets no commands (its thrust would be silently dropped)");
    CHECK_TRUE(std::string(law.why()).size() > 0, "and the reason is recorded");
    /* elements() would otherwise be a default-constructed conic, which the
       --ascent-log row printed as "peri_asl=0 m" as if it had been measured. */
    CHECK_TRUE(!law.conicOk(), "and the law reports it has no conic for that tick");

    s.railed = false;
    s.accel_max = 0.0;                       // the stage cannot push
    const AscentCommand dead = law.step(s);
    CHECK_NEAR(glm::length(dead.aim), 0.0, 0.0, "a dead stage buys no aim");
    CHECK_TRUE(law.phase() == AscentPhase::Failed, "and the law says failed");
    CHECK_TRUE(std::string(law.why()).find("thrust") != std::string::npos, "with the reason");

    /* The hole the potential read leaves: engines rated 18 m/s^2 and nothing to
       burn. availableAccel() says healthy, so without the driver's tanks check
       the law would aim and arm a burn that does nothing, for ever. */
    AscentLaw law_dry(t);
    s.accel_max = 18.0; s.has_propellant = false;
    const AscentCommand dry = law_dry.step(s);
    CHECK_NEAR(glm::length(dry.aim), 0.0, 0.0, "a dry stage buys no aim");
    CHECK_TRUE(law_dry.phase() == AscentPhase::Failed, "and the law says failed, not still flying");
    CHECK_TRUE(std::string(law_dry.why()).find("dry") != std::string::npos,
               "naming the tanks rather than the engines");

    AscentLaw law2(t);
    s.accel_max = 18.0; s.has_propellant = true; s.mu = 0.0;
    law2.step(s);
    CHECK_TRUE(law2.phase() == AscentPhase::Failed, "no mu, no mission");

    /* The stage that can push and has fuel but weighs more than its engine can
       lift. Moho pulls at 2.70 m/s^2 at the surface; 1.0 m/s^2 is TWR 0.37. The
       phase machine has no way to find this out for itself -- it waits for an
       apoapsis the hull can never build -- so before the gate the player's only
       answer was an empty tank and a ship that never moved. */
    AscentLaw law_weak(t);
    s.mu = MU_MOHO; s.accel_max = 1.0; s.has_propellant = true; s.grounded = true;
    const AscentCommand weak = law_weak.step(s);
    printf("   [grounded, 1.0 m/s^2 on Moho] phase=%s why=\"%s\"\n",
           law_weak.phaseName(), law_weak.why());
    CHECK_TRUE(law_weak.phase() == AscentPhase::Failed, "and the law says failed, not still flying");
    CHECK_TRUE(std::string(law_weak.why()).find("lift off") != std::string::npos,
               "naming the weight rather than the tanks or the engines");
    CHECK_TRUE(!weak.burn && weak.throttle == 0.0, "and it arms no burn");

    /* The other side of the inequality: a stage with TWR 6.7 on the same pad
       must still be allowed to fly, so the gate cannot just be "grounded". */
    AscentLaw law_ok(t);
    s.accel_max = 18.0;
    const AscentCommand able = law_ok.step(s);
    CHECK_TRUE(law_ok.phase() == AscentPhase::Pad, "a stage that can lift off is still flying");
    CHECK_TRUE(able.burn, "and it lights the engine on the first tick");
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
    law.step(s);
    const OrbitElements o = law.elements();
    CHECK_TRUE(o.traj != TrajectoryClass::Elliptic, "that state is an escape, not an orbit");
    CHECK_TRUE(law.phase() != AscentPhase::Done, "so it is not Done, however fast it is");

    // Faster than orbital but still bound: periapsis sits AT the ask, apoapsis
    // 1500 km past it. A rule that only looked for periapsis >= ask would call
    // this mission accomplished.
    AscentLaw law3(t);
    s.vel = glm::dvec3(0.0, 900.0, 0.0);
    law3.step(s);
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
    law2.step(s);
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
    const AscentCommand c = law.step(s);
    CHECK_TRUE(std::isfinite(c.aim.x) && std::isfinite(c.aim.y) && std::isfinite(c.aim.z),
               "a standing still ship gets a finite aim");
    CHECK_NEAR(glm::dot(glm::normalize(c.aim), glm::normalize(s.pos)), 1.0, 1e-9,
               "and it points straight up off the pad");
    CHECK_NEAR(c.throttle, 1.0, 0.0, "full throttle from the pad");
    CHECK_TRUE(c.burn, "and the engine is lit");
}

/* What the Guidance window's fields and --ascent share (Game::armAscent, #612):
   the mapping from "a player typed these numbers" to "what the law steers at".
   Nothing else reads it, so nothing else can break it -- and the plane flag is
   exactly where the CLI once lost a due-equator ask by treating the sentinel as
   a range (#610 review). */
static void test_the_fields_become_a_mission() {
    printf("== the ask: fields to target ==\n");
    AscentAsk ask;
    ask.peri_asl = 100e3;
    ask.apo_asl = 150e3;
    ask.ask_inc = false;
    ask.inc_deg = 60.0;   // typed, but the box that asks for it is unticked

    AscentTarget t = ask.target();
    CHECK_NEAR(t.peri_asl, 100e3, 0.0, "periapsis field to the law's floor");
    CHECK_NEAR(t.apo_asl, 150e3, 0.0, "apoapsis field to the law's ceiling");
    CHECK_TRUE(t.inc_rad < 0.0,
               "an unticked inclination box asks no plane, whatever the field says");

    ask.ask_inc = true;
    t = ask.target();
    CHECK_NEAR(t.inc_rad, glm::radians(60.0), 1e-12, "a ticked box asks its inclination");
    ask.inc_deg = 0.0;
    t = ask.target();
    CHECK_NEAR(t.inc_rad, 0.0, 0.0, "and 0 deg is sayable -- the sentinel is not a range");

    /* The window opens on these defaults and Go arms them straight into a law
       that asserts peri <= apo, so a default that trips its own assert is an
       abort on the first click. */
    const AscentAsk dflt;
    CHECK_TRUE(dflt.peri_asl > 0.0 && dflt.peri_asl <= dflt.apo_asl,
               "the defaults satisfy the law's floor and ceiling");
}

/* releaseAscent() is the Cancel half of the driver (Game::cancelAscent, #612).
   The e2e case can only watch one of its two writes: armed thrust is cleared at
   the top of every tick, so a stale lever moves nothing until the player arms
   the engine, and --ascent-log is gated on Game::ascent, which Cancel clears.
   So the lever half gets pinned here, against a hull with no parts -- both
   writes are plain state, and Command is the door the law uses. */
static void test_letting_go_closes_both_channels() {
    printf("== the handback: latch and lever ==\n");
    Vehicle *v = new Vehicle;
    v->Command(ShipCmd(Sas, SlewFree), true);      // what the law does to aim
    v->Command(ShipCmd(ThrottleSet, 0.7f), true);  // and what it does to the lever
    CHECK_TRUE(v->slewRequestMode() == SlewFree, "the law holds the attitude channel");
    CHECK_NEAR(v->throttle(), 0.7, 1e-6, "and the lever sits where it put it");

    releaseAscent(*v);
    CHECK_TRUE(v->slewRequestMode() == SlewNone, "letting go releases the latch");
    CHECK_NEAR(v->throttle(), 0.0, 1e-6, "letting go closes the lever");

    /* flyAscent calls the latch half on every tick it does not fly, and
       Command's Sas case TOGGLES, so a second call must not latch it back on. */
    releaseAscent(*v);
    CHECK_TRUE(v->slewRequestMode() == SlewNone, "and calling it twice keeps it off");
}

int main() {
    test_a_pad_ship_aims_at_the_sky_and_never_at_nan();
    test_the_fields_become_a_mission();
    test_letting_go_closes_both_channels();
    test_done_is_a_periapsis_not_a_speed();
    test_the_law_fails_loudly_not_silently();
    test_degenerate_geometry_still_points_somewhere_horizontal();
    test_the_heading_has_a_sense_and_a_working_cos_lat();
    test_the_launch_heading_buys_the_inclination();
    test_it_actually_gets_to_orbit();
    test_an_elliptical_ask_keeps_both_apsides();
    test_the_shape_holds_at_other_altitudes();
    test_the_shape_at_the_tick_rates_the_game_uses();
    test_asks_it_cannot_land_say_so();
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
