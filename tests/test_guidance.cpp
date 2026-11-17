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
#include <array>
#include <string>
#include "tchecks.h"

static const double G_CONST = 6.674e-11;            // src/system.cpp's G (#707)
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

/* #931: the Pad -> Ascent edge is one-way and nothing else re-read `grounded`,
   so a hull that came back to the surface kept a live mission and kept burning.
   On Kerbin that ran 20 s of phase=ascent, lever 1.00, |v| = 0.01 m/s, and the
   verdict printed was "the tanks are dry" -- the consequence of the landing, not
   the landing. The cause has to outrank the consequence, so the dry-tank test
   sits after this one. */
static void test_a_hull_that_lands_is_a_failed_mission() {
    printf("== a hull that comes back down ==\n");
    AscentTarget t; t.apo_asl = t.peri_asl = 100e3; t.inc_rad = -1.0;
    AscentLaw law(t);
    GuidanceState s; s.pos = glm::dvec3(R_MOHO, 0.0, 0.0); s.vel = glm::dvec3(0.0);
    s.mu = MU_MOHO; s.sea_r = R_MOHO; s.spin_axis = glm::dvec3(0.0, 0.0, 1.0);
    s.accel_max = 18.0; s.grounded = true;
    law.step(s);
    CHECK_TRUE(law.phase() == AscentPhase::Pad, "armed on the pad it is still on the pad");
    CHECK_TRUE(law.active(), "and a ship at rest on the ground is not yet a failure");

    s.pos = glm::dvec3(R_MOHO + 5000.0, 0.0, 0.0);
    s.vel = glm::dvec3(0.0, 300.0, 0.0);
    s.grounded = false;
    law.step(s);
    CHECK_TRUE(law.phase() == AscentPhase::Ascent, "airborne, it is flying");

    s.grounded = true;                     // back on the surface, engines still rated
    const AscentCommand c = law.step(s);
    printf("   [landed] phase=%s why=\"%s\" |aim|=%.3f throttle=%.2f\n",
           law.phaseName(), law.why(), glm::length(c.aim), c.throttle);
    CHECK_TRUE(law.phase() == AscentPhase::Failed, "a hull back on the ground is Failed");
    CHECK_TRUE(!law.active(), "and the mission is over, not still commanding");
    CHECK_NEAR(glm::length(c.aim), 0.0, 0.0, "it sends no aim (the driver cancels the setpoint)");
    CHECK_NEAR(c.throttle, 0.0, 0.0, "and no throttle");
    CHECK_TRUE(std::string(law.why()).find("came back down") != std::string::npos,
               "and it says the ship landed, not that the tanks ran dry");

    /* The ordering, pinned: a hull that is both parked and dry must not be
       blamed for the tanks. */
    AscentLaw law2(t);
    GuidanceState s2 = s; s2.grounded = false; s2.has_propellant = true;
    law2.step(s2);                          // leave the pad
    s2.grounded = true; s2.has_propellant = false;
    law2.step(s2);
    CHECK_TRUE(std::string(law2.why()).find("came back down") != std::string::npos,
               "the landing outranks the dry tanks");
}

/* #930: pathAngle() is a schedule over ALTITUDE, so a hull that loses height is
   told to point further up than it did at the pad. On Kerbin the commanded pitch
   climbed from 34 deg back to 87.1 deg while the ship sank, and the flight ended
   parked on the surface. The schedule reads the high-water mark instead, so the
   pitch the law asks for can only flatten as the flight goes on. Pinning the
   property rather than the number keeps it true whatever profile v2 picks. */
static void test_a_sinking_ship_does_not_get_its_nose_back() {
    printf("== the pitch schedule cannot walk backwards ==\n");
    AscentTarget t; t.apo_asl = t.peri_asl = 100e3; t.inc_rad = -1.0;
    AscentLaw law(t);
    GuidanceState s; s.vel = glm::dvec3(0.0, 300.0, 0.0);
    s.mu = MU_MOHO; s.sea_r = R_MOHO; s.spin_axis = glm::dvec3(0.0, 0.0, 1.0);
    s.accel_max = 18.0; s.grounded = false;
    double prev = 1e9;
    bool fell = false;
    for(double alt : {2e3, 3e3, 5e3, 8e3, 4e3, 2e3, 0.5e3}) {
        s.pos = glm::dvec3(R_MOHO + alt, 0.0, 0.0);
        law.step(s);
        const double pitch = law.pitch_deg();
        printf("   [alt %6.0f m] pitch %.2f deg phase=%s\n", alt, pitch, law.phaseName());
        CHECK_TRUE(pitch <= prev + 1e-9, "the commanded pitch never steepens");
        if(alt < 5e3 && prev < 40.0) { fell = true; }   // the sinking part actually ran
        prev = pitch;
    }
    CHECK_TRUE(fell, "the test really flew DOWN from above the apex it had reached");
    CHECK_NEAR(prev, std::atan2(3000.0, 8000.0) * 180.0 / 3.14159265358979, 1e-6,
               "and it holds the pitch the HIGHEST altitude asked for, not the current one");
}

/* #928: the law asks for a press, and asks ONCE per stage number.
   The retry rule matters because a press can be refused -- a crewed capsule rides
   the stage, or the seam is the hull's own root (#545) -- and Game prints and toasts
   on every press. A law that re-asked every tick would bury the log in refusals and
   never fly anything; a law that gave up blindly would leave a ship carrying stages
   it could have lit. So the ask is keyed to the counter: once per value of
   stages_left, and the counter moving (or not) on the next tick IS the feedback. */
static void test_the_law_asks_for_a_press_and_asks_once() {
    printf("== the press request: once per stage number ==\n");
    AscentTarget t; t.apo_asl = t.peri_asl = 100e3; t.inc_rad = -1.0;
    AscentLaw law(t);
    GuidanceState s; s.pos = glm::dvec3(R_MOHO + 20000.0, 0.0, 0.0);
    s.vel = glm::dvec3(0.0, 400.0, 0.0);
    s.mu = MU_MOHO; s.sea_r = R_MOHO; s.spin_axis = glm::dvec3(0.0, 0.0, 1.0);
    s.accel_max = 18.0; s.grounded = false;
    s.stages_left = 2; s.stage_due = true;
    const AscentCommand c1 = law.step(s);
    printf("   [due] want_stage=%d phase=%s\n", (int)c1.want_stage, law.phaseName());
    CHECK_TRUE(c1.want_stage, "a stage the hull says is due gets a press asked for");
    CHECK_TRUE(law.phase() == AscentPhase::Ascent, "asking is not a verdict");
    const AscentCommand c2 = law.step(s);
    CHECK_TRUE(!c2.want_stage, "and it is not asked twice for the same stage number");

    /* The refusal, and where it ends: still dry, still the same counter, so the
       ask is spent and the mission says the honest thing rather than looping. */
    s.has_propellant = false;
    const AscentCommand c3 = law.step(s);
    printf("   [refused] want_stage=%d phase=%s why=\"%s\"\n",
           (int)c3.want_stage, law.phaseName(), law.why());
    CHECK_TRUE(!c3.want_stage, "a dry hull does not re-ask a press it was refused");
    CHECK_TRUE(law.phase() == AscentPhase::Failed, "and the mission ends rather than spins");
    CHECK_TRUE(std::string(law.why()).find("tanks are dry") != std::string::npos,
               "with the reason it has always given");

    /* The other direction, which is the one that used to end a mission: a stage
       that has run dry with presses still unasked is a press, not a failure. */
    AscentLaw law2(t);
    GuidanceState s2 = s; s2.has_propellant = true;
    s2.stages_left = 2; s2.stage_due = false;
    law2.step(s2);
    CHECK_TRUE(law2.phase() == AscentPhase::Ascent, "flying, nothing due, no ask");
    s2.has_propellant = false;
    const AscentCommand d1 = law2.step(s2);
    printf("   [dry with presses] want_stage=%d phase=%s\n",
           (int)d1.want_stage, law2.phaseName());
    CHECK_TRUE(d1.want_stage, "a dry stage with presses left asks for one");
    CHECK_TRUE(law2.phase() != AscentPhase::Failed, "instead of failing on that tick");
    const AscentCommand d2 = law2.step(s2);
    CHECK_TRUE(!d2.want_stage && law2.phase() == AscentPhase::Failed,
               "and if the press did not arrive, it fails on the next tick");

    /* A press that separated nothing still stepped the counter (#545's shape), so a
       later press is askable again -- the law is keyed to the number, not to whether
       the last one dropped anything. */
    AscentLaw law3(t);
    GuidanceState s3 = s; s3.has_propellant = true;
    s3.stages_left = 3; s3.stage_due = true;
    CHECK_TRUE(law3.step(s3).want_stage, "asked at three presses left");
    CHECK_TRUE(!law3.step(s3).want_stage, "not asked again at three");
    s3.stages_left = 2;                       // the counter moved, whatever it dropped
    CHECK_TRUE(law3.step(s3).want_stage, "asked again at two, due or not being the hull's call");
}

/* #941: the case the counter cannot see. A press the caller REFUSED leaves
   stages_left exactly where it was, so the once-per-value latch made the refusal
   permanent -- the mission burned that stage dry and reported "the tanks are dry"
   with presses still folded under it, even after the player put the kerbal on EVA.
   The caller now reports the refusal, so the latch must release (a mid-flight fix
   has to be able to arrive) while the re-ask stays throttled -- an unfixed refusal
   that asks every tick buries the log, which is why the latch exists at all (#928).
   The window is a tick count, not a clock: step() takes no dt. 60 logic ticks is
   about a second at 1x, so an unfixed refusal is one line a second, not one a frame. */
static void test_a_reported_refusal_is_retried_and_named() {
    printf("== a reported refusal: retried, throttled, named ==\n");
    AscentTarget t; t.apo_asl = t.peri_asl = 100e3; t.inc_rad = -1.0;
    AscentLaw law(t);
    GuidanceState s; s.pos = glm::dvec3(R_MOHO + 20000.0, 0.0, 0.0);
    s.vel = glm::dvec3(0.0, 400.0, 0.0);
    s.mu = MU_MOHO; s.sea_r = R_MOHO; s.spin_axis = glm::dvec3(0.0, 0.0, 1.0);
    s.accel_max = 18.0; s.grounded = false;
    s.stages_left = 2; s.stage_due = true;
    CHECK_TRUE(law.step(s).want_stage, "asked at two presses left");
    law.pressRefused(PressRefusal::Crew);
    for(int i = 0; i < 20; i++) {
        CHECK_TRUE(!law.step(s).want_stage, "silent while the retry window counts down");
    }
    int ticks = 20;
    bool asked = false;
    while(!asked && ticks < 200) { asked = law.step(s).want_stage; ticks++; }
    printf("   [reported] the next ask came %d ticks after the refusal\n", ticks);
    CHECK_TRUE(asked, "the ask comes back, so a player who EVAed the kerbal gets her stage");
    CHECK_TRUE(ticks > 20 && ticks <= 120, "and it comes back on a second, not every tick and not never");
    CHECK_TRUE(!law.step(s).want_stage, "still one ask per stage number after the retry");

    /* Reporting a refusal is not a promise to wait for the player. The tanks keep
       draining while the window runs, and a stage that empties with its press
       unanswered ends the mission the tick it always would have (#941's open
       question, answered: hold the throttle forever and a mis-crafted hull flies a
       mission that can never end). What changes is what the verdict SAYS, which is
       the part of #941 that actually fooled anyone. */
    s.has_propellant = false; s.stage_due = false;
    const AscentCommand f = law.step(s);
    CHECK_TRUE(!f.want_stage && law.phase() == AscentPhase::Failed,
               "a hull that runs dry inside the window still gets its verdict");
    CHECK_TRUE(std::string(law.why()).find("press was refused (crew aboard)") != std::string::npos,
               "and it says so instead of only naming the empty tanks");

    /* A root seam is not fixable mid-flight -- which side of a decoupler is the
       ship's root is decided in the VAB -- so it is remembered and never re-asked:
       one refusal line for the flight, not one a second. */
    AscentLaw law2(t);
    GuidanceState s2 = s; s2.has_propellant = true; s2.stages_left = 3; s2.stage_due = true;
    CHECK_TRUE(law2.step(s2).want_stage, "asked at three presses left");
    law2.pressRefused(PressRefusal::Root);
    bool re_asked = false;
    for(int i = 0; i < 120; i++) { re_asked = re_asked || law2.step(s2).want_stage; }
    CHECK_TRUE(!re_asked, "a root refusal is not re-asked");
    s2.has_propellant = false;
    law2.step(s2);
    CHECK_TRUE(law2.phase() == AscentPhase::Failed
               && std::string(law2.why()).find("refused (root seam)") != std::string::npos,
               "but the verdict still names it");

    /* The memory lasts as long as the refusal does. Once stages_left moves, a press
       HAS happened, and an old no is not why the stack is still together. */
    AscentLaw law3(t);
    GuidanceState s3 = s; s3.has_propellant = true; s3.stages_left = 2; s3.stage_due = true;
    law3.step(s3);
    law3.pressRefused(PressRefusal::Crew);
    s3.stages_left = 1;                       // EVAed her, and the next press separated
    s3.has_propellant = false;
    law3.step(s3);
    CHECK_TRUE(law3.phase() == AscentPhase::Failed && std::string(law3.why()) == "the tanks are dry",
               "a refusal is forgotten once the counter has moved on");

    law.pressRefused(PressRefusal::Crew);
    CHECK_TRUE(law.phase() == AscentPhase::Failed,
               "and reporting to a law that already answered changes nothing");
}

/* #928's claim, made checkable: the SAME hull flown by the SAME law with presses
   allowed and with presses refused. The stage table is sized so the pad stage
   cannot reach the ask on its own (684 m/s against the ~1.1 km/s a 100 km Moho
   orbit costs), so staging is not a tuning knob in this test -- it is the
   difference between Done and "the tanks are dry".
   The hull is the stager's shape, not a fantasy: one stage lights its own tanks,
   its structure leaves with it when it is dropped, and the next stage's tank is
   unreachable until that stage lights (no fuel links in res/ships/stager.json).
   The press timing is pinned too: a law that pressed early would drop a tankful
   of propellant no engine can reach, so the test asserts the ask came when THAT
   stage's pool was empty, not merely that it came. */
struct StageSpec {
    double thrust;   // N, what this stage's engines rate
    double mdot;     // kg/s they draw, at full lever
    double prop;     // kg in the pool this stage burns (and is dropped with)
    double dry;      // kg of structure that leaves the hull when this stage goes
};

struct StagedFlight {
    bool done = false, failed = false;
    std::string why;
    int presses = 0;
    double prop_at_press = -1.0;   // the dropped stage's leftover propellant, kg
    double peri = 0.0, apo = 0.0;
};

/* The gram floor stageIsDue() uses, mirrored: the live drain leaves float residue,
   so "empty" is a gram and not zero. Kept here rather than shared because the test
   is checking the law's DECISION, and the number it decides on is the pool. */
static const double kStageEmpty = 1e-3;

static StagedFlight simulateStaged(const AscentTarget &target, const StageSpec *st,
                                   int nStage, double baseDry, bool allow_press,
                                   double dt = 1.0/120.0, int max_s = 900) {
    StagedFlight f;
    AscentLaw law(target);
    std::vector<double> prop(nStage);
    for(int i = 0; i < nStage; i++) { prop[i] = st[i].prop; }
    int cur = 0;                        // which stage is lit (0 = the pad stage)
    const int pressesMax = nStage - 1;
    glm::dvec3 pos(R_MOHO + 1.0, 0.0, 0.0), vel(0.0, 0.0, 0.0);
    for(int step = 0; step * dt < max_s; step++) {
        double mass = baseDry;
        for(int i = cur; i < nStage; i++) { mass += st[i].dry + prop[i]; }
        const double pos_r = glm::length(pos);
        GuidanceState s;
        s.pos = pos; s.vel = vel;
        s.mu = MU_MOHO; s.sea_r = R_MOHO;
        s.spin_axis = glm::dvec3(0.0, 0.0, 1.0);
        s.accel_max = st[cur].thrust / mass;   // a potential: fuel-blind, as in the game
        s.has_propellant = prop[cur] > 0.0;
        s.stages_left = pressesMax - cur;
        s.stage_due = prop[cur] <= kStageEmpty;   // the pool THIS press drops
        s.grounded = (pos_r - R_MOHO) < 1.0;
        const AscentCommand c = law.step(s);
        if(c.want_stage) {
            f.presses++;
            f.prop_at_press = prop[cur];
            if(allow_press) { cur++; }   // refused: the counter does not move, and
                                         // the structure stays aboard with its tank
        }
        double thrust_now = 0.0;
        if(c.burn && prop[cur] > 0.0) {
            thrust_now = st[cur].thrust * c.throttle;
            prop[cur] -= st[cur].mdot * c.throttle * dt;
            if(prop[cur] < 0.0) { prop[cur] = 0.0; }
        }
        const glm::dvec3 grav = -glm::normalize(pos) * (MU_MOHO / (pos_r * pos_r));
        const glm::dvec3 aim = (glm::length2(c.aim) > 1e-12) ? glm::normalize(c.aim)
                                                             : glm::dvec3(0.0);
        vel += (aim * (thrust_now / mass) + grav) * dt;
        pos += vel * dt;
        if(!law.active()) {
            f.done = law.phase() == AscentPhase::Done;
            f.failed = law.phase() == AscentPhase::Failed;
            f.why = law.why();
            break;
        }
    }
    const OrbitElements o = law.elements();
    f.peri = periapsisAsl(o, R_MOHO);
    f.apo = (o.traj == TrajectoryClass::Elliptic) ? apoapsisAsl(o, R_MOHO) : -1.0;
    if(f.why.empty()) { f.why = std::string("ran out of steps in ") + law.phaseName(); }
    return f;
}

static void test_a_staged_hull_flies_and_an_unstaged_one_does_not() {
    printf("== staging is the difference between Done and dry ==\n");
    const StageSpec st[2] = {
        {56250.0, 12.5, 500.0, 900.0},    // the pad stage: 4500 m/s of ve, 684 m/s of dv
        {25000.0, 6.25, 700.0, 0.0},      // the stage under it: 1.56 km/s, and it lights
    };
    AscentTarget t; t.apo_asl = t.peri_asl = 100e3; t.inc_rad = -1.0;
    const StagedFlight fly = simulateStaged(t, st, 2, 1470.23, true);
    const StagedFlight stuck = simulateStaged(t, st, 2, 1470.23, false);
    printf("   [presses allowed] presses=%d leftAtPress=%.4f kg peri=%.1f apo=%.1f why=\"%s\"\n",
           fly.presses, fly.prop_at_press, fly.peri, fly.apo, fly.why.c_str());
    printf("   [presses refused] presses=%d peri=%.1f apo=%.1f why=\"%s\"\n",
           stuck.presses, stuck.peri, stuck.apo, stuck.why.c_str());
    CHECK_TRUE(fly.done, "with presses the law reaches the ask");
    CHECK_TRUE(stuck.failed && stuck.why.find("tanks are dry") != std::string::npos,
               "without them the same hull runs dry and says so");
    CHECK_TRUE(fly.presses == 1, "exactly one press for a two-stage stack");
    CHECK_TRUE(fly.prop_at_press >= 0.0 && fly.prop_at_press <= 1e-3,
               "and it came when that stage's own pool was empty, not with a tankful left");
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

/* pathAngle()'s air term (#960). The harness has no drag -- Moho has none -- so this
   pins the SCHEDULE, not a mission: at the same high-water altitude, a body with air
   must ask for a nose at least as high as a body without one, because what the air
   bills is the path flown through it and a hull that leans over at 4 km pays for a long
   one. Measured off Kerbin with mars_explorer_mk1: with the air term off, a 150 km ask
   never got past 9.4 km of apoapsis and spent 11.7 km/s of drag; with it on the same
   hull on the same bed reached 150 x 159 km for 4.8 km/s.

   kFlatAir itself is a FLIGHT-tuned number, so it is deliberately not spelled here. What
   the suite pins is the form: the nose comes down as the hull climbs, one flat-over
   altitude read as h_half = alt * tan(pitch) and the same at every altitude, keyed on
   the height of the air, capped by the ask, and never below the ask-keyed one. The two
   size checks at the bottom DO bound kFlatAir from below -- with an 85 km air they need
   it above ~0.21 -- which is the deliberate half: the shipped 0.40 must not drift back
   toward the 4.5 km turn this issue is about. */
static double schedPitch(const AscentTarget &t, double air_top, double air_column, double alt) {
    AscentLaw law(t);
    GuidanceState s;
    /* A slow climb, so the apoapsis stays put and the law keeps answering with its
       schedule instead of moving to Coast and aiming prograde. */
    s.pos = glm::dvec3(R_MOHO + alt, 0.0, 0.0);
    s.vel = glm::dvec3(0.0, 30.0, 0.0);
    s.mu = MU_MOHO; s.sea_r = R_MOHO; s.spin_axis = glm::dvec3(0.0, 0.0, 1.0);
    s.accel_max = 19.0; s.has_propellant = true;
    s.air_top_asl = air_top;
    s.air_column = air_column;
    law.step(s);
    return law.pitch_deg();
}

static double schedHHalf(const AscentTarget &t, double air_top, double air_column, double alt) {
    return alt * std::tan(glm::radians(schedPitch(t, air_top, air_column, alt)));
}

static void test_the_air_holds_the_nose_up() {
    printf("== the pitch schedule in air: the air, not only the ask, says when to flatten ==\n");
    const double ask = 150e3;
    /* Authored air, res/systems/ksp_system.json: height and column mass (p0/g). */
    const double air_top = 85e3, air_column = 10328.0;      // Kerbin
    const double thin_top = 50e3, thin_column = 204.0;      // Duna
    AscentTarget t; t.apo_asl = t.peri_asl = ask; t.inc_rad = -1.0;
    const std::array<double, 6> alts = {1e3, 5e3, 10e3, 20e3, 40e3, 80e3};
    std::array<double, alts.size()> no_air{}, air{};
    for(size_t i = 0; i < alts.size(); i++) {
        no_air[i] = schedHHalf(t, 0.0, 0.0, alts[i]);
        air[i] = schedHHalf(t, air_top, air_column, alts[i]);
        printf("   [alt %5.0f km] no air %5.1f deg (flat-over %6.1f km), air %5.1f deg (%6.1f km)\n",
               alts[i] / 1e3, schedPitch(t, 0.0, 0.0, alts[i]), no_air[i] / 1e3,
               schedPitch(t, air_top, air_column, alts[i]), air[i] / 1e3);
    }
    /* One flat-over altitude, whatever altitude it is read from: the schedule is
       atan2(h_half, alt), and a second term keyed on something else would show up here
       as two different h_half's. */
    CHECK_NEAR(air[3], air[1], 1.0, "the air-keyed schedule has ONE flat-over altitude");
    CHECK_NEAR(no_air[3], no_air[1], 1.0, "...and so does the ask-keyed one");
    /* The air term scales with the height of the air, and with its weight, not with how
       much air is left above the ship: halve either, halve the turn. The weight pair is
       deliberately thinner than Kerbin's 10328 kg/m^2, which is past the point where a
       sky counts fully -- a saturated term would not halve, and that is the clamp. */
    CHECK_NEAR(schedHHalf(t, air_top / 2.0, air_column, 10e3), air[2] / 2.0, 1.0,
               "the turn scales with how tall the air is");
    const double heavy = schedHHalf(t, air_top, 5000.0, 10e3);
    CHECK_NEAR(schedHHalf(t, air_top, 2500.0, 10e3), heavy / 2.0, 1.0,
               "...and with how heavy it is");
    for(size_t i = 0; i < alts.size(); i++) {
        CHECK_TRUE(air[i] >= no_air[i] - 1e-6, "the air term only ever DELAYS the flatten");
        CHECK_TRUE(air[i] > 0.0, "and stays a path angle");
        if(i > 0) {
            /* The nose on the pitch, not on h_half: h_half is what stays put. */
            const double down = schedPitch(t, air_top, air_column, alts[i]);
            const double up = schedPitch(t, air_top, air_column, alts[i-1]);
            CHECK_TRUE(down < up, "the nose comes down as the hull climbs");
        }
    }
    /* The size of it, and why it is there: on Kerbin the turn moves from a few km to
       tens of km, so the fat part of the air is flown steep. Measured: this is the
       difference between 9.4 km of apoapsis and reaching space. */
    CHECK_TRUE(air[2] > 4.0 * no_air[2], "in air the turn is far above the ask's own scale");
    CHECK_TRUE(schedPitch(t, air_top, air_column, 10e3) > 60.0, "and the hull is still climbing steeply at 10 km");

    /* And the three bounds, which the checks above cannot see. The ask still owns the
       floor: on a 1500 km ask the air term is smaller than kFlatHalf * ask, so the
       schedule must be the ask-keyed one exactly -- delete the max() and every check
       above still passes, because they all use an ask the air term beats. */
    AscentTarget high; high.apo_asl = high.peri_asl = 1500e3; high.inc_rad = -1.0;
    CHECK_NEAR(schedHHalf(high, air_top, air_column, 10e3), schedHHalf(high, 0.0, 0.0, 10e3), 1.0,
               "an ask taller than the air still flies the ask's own schedule");
    /* The air never outlasts the ask either. Duna's sky is 50 km over a 20 km mission:
       uncapped, that asked a 20 km turn where the ask wanted 600 m, and the hull flew
       to 1150 km of apoapsis with its tanks dry (#960 review). A body whose air is TALLER
       than the ask must schedule like one whose air is exactly as tall. */
    AscentTarget low; low.apo_asl = low.peri_asl = 20e3; low.inc_rad = -1.0;
    const double tall = schedHHalf(low, air_top, air_column, 10e3);
    CHECK_NEAR(tall, schedHHalf(low, 20e3, air_column, 10e3), 1.0,
               "air taller than the ask is capped at the ask");
    CHECK_TRUE(tall < 20e3, "so the turn ends below the altitude the mission is asking for");
    /* And thin air buys no trajectory at all. Duna has 2% of Kerbin's air per square
       metre; the flown hull there was better off flying the ask's own schedule, and a
       height-only term steered it into the ground of its own apoapsis instead. */
    CHECK_NEAR(schedHHalf(low, thin_top, thin_column, 10e3), schedHHalf(low, 0.0, 0.0, 10e3), 1.0,
               "a sky too thin to push back does not bend the schedule");
}

int main() {
    test_a_pad_ship_aims_at_the_sky_and_never_at_nan();
    test_a_hull_that_lands_is_a_failed_mission();
    test_a_sinking_ship_does_not_get_its_nose_back();
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
    test_the_law_asks_for_a_press_and_asks_once();
    test_a_reported_refusal_is_retried_and_named();
    test_a_staged_hull_flies_and_an_unstaged_one_does_not();
    test_the_air_holds_the_nose_up();
    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
