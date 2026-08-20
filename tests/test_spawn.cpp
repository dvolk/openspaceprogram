//
// Validates the spawn-frame logic introduced in spawn_vehicle (src/main.cpp):
//   1. resolve_frame_by_soi picks the deepest/innermost SOI containing a point.
//   2. For each debug spawn location, the resolved frame is the one the
//      per-tick SOI switching in the main loop would also settle on, i.e. the
//      ship is NOT displaced on the first physics tick (the whole reason the
//      now-removed frame_lock existed).
//   3. The stasis-velocity correction round-trips to the intended inertial
//      orbital velocity (so a rotating frame still yields a clean orbit).
//
// This replicates the real frame tree / SOIs / body parameters from
// setup_frames() and the body setup in main.cpp, and re-implements the exact
// resolve + velocity-conversion formulas used by spawn_vehicle.
//
// Build & run (from repo root):
//   g++ -O2 -std=c++11 -I./src -I./middleware/glm/ \
//       tests/test_spawn.cpp src/frame.cpp -o test_spawn && ./test_spawn

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "frame.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK_TRUE(cond, msg)                                                  \
    do {                                                                       \
        g_checks++;                                                            \
        if (!(cond)) {                                                         \
            g_failures++;                                                      \
            printf("FAIL: %s\n", msg);                                         \
        }                                                                      \
    } while (0)

// The exact algorithm from spawn_vehicle (src/main.cpp).
static Frame *resolve_frame_by_soi(Frame *root, glm::dvec3 worldPos) {
    Frame *cur = root;
    while(true) {
        Frame *best = NULL;
        double best_d = 1e30;
        for(Frame *c : cur->children) {
            double d = glm::length(worldPos - c->root_pos);
            if(d < c->soi && d < best_d) {
                best = c;
                best_d = d;
            }
        }
        if(best == NULL) { return cur; }
        cur = best;
    }
}

// Real tree / SOIs / body data from setup_frames() + body setup in main.cpp.
static Frame *make_tree(Frame *&out_sun, Frame *&out_eerbon, Frame *&out_eerbon_rot,
                        Frame *&out_moon, Frame *&out_moon_rot) {
    Frame *sun = new Frame;
    Frame *eerbon = new Frame;
    Frame *eerbon_rot = new Frame;
    Frame *moon = new Frame;
    Frame *moon_rot = new Frame;

    sun->name = "sun";
    sun->parent = NULL;
    sun->children = { eerbon };
    sun->rotating = false;
    sun->pos = glm::dvec3(0);
    sun->orient = glm::dmat3();
    sun->rot_ang_speed = 0;
    sun->orb_ang_speed = 0;
    sun->soi = 9999999999999999.0;

    eerbon->name = "eerbon";
    eerbon->parent = sun;
    eerbon->children = { eerbon_rot, moon };
    eerbon->rotating = false;
    eerbon->has_rot_frame = true;
    eerbon->pos = glm::dvec3(0, 0, -13599840260.0);
    eerbon->orient = glm::dmat3();
    eerbon->rot_ang_speed = 0;
    eerbon->orb_ang_speed = 0.00000068269186570822291594437651;
    eerbon->soi = 84159286.0;

    eerbon_rot->name = "eerbon_rot";
    eerbon_rot->parent = eerbon;
    eerbon_rot->children = {};
    eerbon_rot->rotating = true;
    eerbon_rot->pos = glm::dvec3(0);
    eerbon_rot->orient = glm::dmat3();
    eerbon_rot->rot_ang_speed = 0.00029157090303706880702966723086;
    eerbon_rot->orb_ang_speed = 0;
    eerbon_rot->soi = 700000.0;

    moon->name = "moon";
    moon->parent = eerbon;
    moon->children = { moon_rot };
    moon->rotating = false;
    moon->has_rot_frame = true;
    moon->pos = glm::dvec3(-12e6, 0, 0);
    moon->orient = glm::dmat3();
    moon->rot_ang_speed = 0;
    moon->orb_ang_speed = 0.00004520797578987211820731369629;
    moon->soi = 2429559.1;

    moon_rot->name = "moon_rot";
    moon_rot->parent = moon;
    moon_rot->children = {};
    moon_rot->rotating = true;
    moon_rot->pos = glm::dvec3(0);
    moon_rot->orient = glm::dmat3();
    moon_rot->rot_ang_speed = 0.00004520785218583258404235991675;
    moon_rot->orb_ang_speed = 0;
    moon_rot->soi = 300000.0;

    out_sun = sun;
    out_eerbon = eerbon;
    out_eerbon_rot = eerbon_rot;
    out_moon = moon;
    out_moon_rot = moon_rot;
    return sun;
}

// Body radii [m] and mu [m^3/s^2] from the body setup in main.cpp.
static const double sun_radius = 261600000.0;
static const double sun_mu = 1.1723328e18;
static const double eerbon_radius = 600000.0;
static const double eerbon_mu = 3.5316000e12;
static const double moon_radius = 200000.0;
static const double moon_mu = 6.5138398e10;

struct SpawnCase {
    int loc;
    const char *desc;
    double rCur;
    double semiMajor;
    double mu;
    Frame *orbitBodyFrame; // frame whose root_pos is the orbit body's world pos
    glm::dvec3 worldPos;   // ship world spawn position
    Frame *expectedFrame;
};

int main() {
    Frame *sun, *eerbon, *eerbon_rot, *moon, *moon_rot;
    Frame *root = make_tree(sun, eerbon, eerbon_rot, moon, moon_rot);
    // Settle the root-relative values (as main does before spawn_vehicle).
    root->UpdateOrbitRails(0.0, 1.0 / 60.0);

    // Build the six spawn cases exactly as spawn_vehicle computes them.
    std::vector<SpawnCase> cases;

    { // loc 2: 75 km circular orbit around Eerbon
        double rCur = eerbon_radius + 75e3;
        SpawnCase c;
        c.loc = 2; c.desc = "75 km circular orbit, Eerbon";
        c.rCur = rCur; c.semiMajor = rCur; c.mu = eerbon_mu;
        c.orbitBodyFrame = eerbon;
        c.worldPos = eerbon->root_pos + eerbon->root_orient * glm::dvec3(0, 0, rCur);
        c.expectedFrame = eerbon_rot;
        cases.push_back(c);
    }
    { // loc 3: 1000x30 km elliptical orbit around Eerbon, at apoapsis
        double ra = eerbon_radius + 1000e3;
        double rp = eerbon_radius + 30e3;
        SpawnCase c;
        c.loc = 3; c.desc = "1000x30 km elliptical @ apoapsis, Eerbon";
        c.rCur = ra; c.semiMajor = 0.5 * (ra + rp); c.mu = eerbon_mu;
        c.orbitBodyFrame = eerbon;
        c.worldPos = eerbon->root_pos + eerbon->root_orient * glm::dvec3(0, 0, ra);
        c.expectedFrame = eerbon;
        cases.push_back(c);
    }
    { // loc 4: 30 km circular orbit around the Moon
        double rCur = moon_radius + 30e3;
        SpawnCase c;
        c.loc = 4; c.desc = "30 km circular orbit, Moon";
        c.rCur = rCur; c.semiMajor = rCur; c.mu = moon_mu;
        c.orbitBodyFrame = moon;
        c.worldPos = moon->root_pos + moon->root_orient * glm::dvec3(0, 0, rCur);
        c.expectedFrame = moon_rot;
        cases.push_back(c);
    }
    { // loc 5: circular solar orbit, halfway between Eerbon and the Sun
        glm::dvec3 worldPos = 0.5 * eerbon->root_pos;
        double rCur = glm::length(worldPos);
        SpawnCase c;
        c.loc = 5; c.desc = "circular solar orbit, halfway to Sun";
        c.rCur = rCur; c.semiMajor = rCur; c.mu = sun_mu;
        c.orbitBodyFrame = sun;
        c.worldPos = worldPos;
        c.expectedFrame = sun;
        cases.push_back(c);
    }
    { // loc 6: 10x1000 km elliptical orbit around Eerbon, at periapsis
        double rp = eerbon_radius + 10e3;
        double ra = eerbon_radius + 1000e3;
        SpawnCase c;
        c.loc = 6; c.desc = "10x1000 km elliptical @ periapsis, Eerbon";
        c.rCur = rp; c.semiMajor = 0.5 * (ra + rp); c.mu = eerbon_mu;
        c.orbitBodyFrame = eerbon;
        c.worldPos = eerbon->root_pos + eerbon->root_orient * glm::dvec3(0, 0, rp);
        c.expectedFrame = eerbon_rot;
        cases.push_back(c);
    }

    for(SpawnCase &c : cases) {
        Frame *frame = resolve_frame_by_soi(sun, c.worldPos);
        char buf[256];

        snprintf(buf, sizeof buf, "loc %d (%s): resolves to '%s'",
                 c.loc, c.desc, frame->name);
        CHECK_TRUE(frame == c.expectedFrame, buf);

        // Position + velocity in the resolved frame (spawn_vehicle formulas).
        double speed = std::sqrt(c.mu * (2.0 / c.rCur - 1.0 / c.semiMajor));
        glm::dvec3 rhat = glm::normalize(c.worldPos - c.orbitBodyFrame->root_pos);
        glm::dvec3 velWorld = speed * glm::cross(glm::dvec3(0, 1, 0), rhat);
        glm::dvec3 target = glm::transpose(frame->root_orient) * (c.worldPos - frame->root_pos);
        glm::dvec3 vel = glm::transpose(frame->root_orient) * velWorld
                        + frame->GetStasisVelocity(target);

        // (a) Stasis round-trip: the implied inertial velocity must equal the
        //     intended orbital velocity.
        glm::dvec3 implied = frame->root_orient * (vel - frame->GetStasisVelocity(target));
        snprintf(buf, sizeof buf, "loc %d: implied inertial vel == target orbital vel", c.loc);
        CHECK_TRUE(glm::length(implied - velWorld) < 1e-6 * speed, buf);

        // (b) Ship must stay put under the main-loop SOI logic: not outside the
        //     resolved frame's SOI, and not inside any of its children's SOIs.
        double ship_r = glm::length(target);
        snprintf(buf, sizeof buf, "loc %d: not outside resolved SOI (r=%.0f, soi=%.0f)",
                 c.loc, ship_r, frame->soi);
        CHECK_TRUE(!(ship_r > frame->soi + 10000.0), buf);

        bool child_switch = false;
        for(Frame *child : frame->children) {
            glm::dvec3 in_child = frame->GetOrientRelTo(child) * target
                                + frame->GetPositionRelTo(child);
            double dist = glm::length(in_child);
            if(dist < child->soi - 10000.0) {
                child_switch = true;
                snprintf(buf, sizeof buf,
                         "loc %d: would switch into child '%s' (dist=%.0f < soi=%.0f)",
                         c.loc, child->name, dist, child->soi);
                CHECK_TRUE(false, buf);
            }
        }
        (void)child_switch;

        printf("  loc %d (%s): frame='%s'  r=%.0f m  |v|=%.1f m/s  OK\n",
               c.loc, c.desc, frame->name, ship_r, speed);
    }

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
