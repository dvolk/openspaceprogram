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
#include <functional>
#include <utility>
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
    eerbon_rot->initial_orient = glm::dmat3();
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
    moon_rot->initial_orient = glm::dmat3();
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
                 c.loc, c.desc, frame->name.c_str());
        CHECK_TRUE(frame == c.expectedFrame, buf);

        // Position + velocity in the resolved frame (spawn_vehicle formulas).
        double speed = std::sqrt(c.mu * (2.0 / c.rCur - 1.0 / c.semiMajor));
        glm::dvec3 rhat = glm::normalize(c.worldPos - c.orbitBodyFrame->root_pos);
        glm::dvec3 velWorld = speed * glm::cross(glm::dvec3(0, 1, 0), rhat);
        glm::dvec3 target = glm::transpose(frame->root_orient) * (c.worldPos - frame->root_pos);
        // (matches the fixed spawn_vehicle: stasis of the TARGET frame is
        // subtracted; see the sign note on GetStasisVelocity in frame.h)
        glm::dvec3 vel = glm::transpose(frame->root_orient) * velWorld
                        - frame->GetStasisVelocity(target);

        // (a) Stasis round-trip: reconstruct the INERTIAL velocity with the
        //     frame-dynamics relation  v_root = R*(v + stasis(p)) + V
        //     (verified against the rotate(-ang) frame motion; see frame.h).
        //     It must equal the body's velocity plus the intended relative
        //     orbital velocity. (Comparing against velWorld alone would be
        //     circular; the V term is what ties it to real inertial motion.)
        glm::dvec3 implied = frame->root_orient * (vel + frame->GetStasisVelocity(target))
                           + frame->root_vel;
        glm::dvec3 intended = frame->root_vel + velWorld;
        snprintf(buf, sizeof buf, "loc %d: implied inertial vel == body vel + orbital vel", c.loc);
        CHECK_TRUE(glm::length(implied - intended) < 1e-6 * speed, buf);

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
                         c.loc, child->name.c_str(), dist, child->soi);
                CHECK_TRUE(false, buf);
            }
        }
        (void)child_switch;

        printf("  loc %d (%s): frame='%s'  r=%.0f m  |v|=%.1f m/s  OK\n",
               c.loc, c.desc, frame->name.c_str(), ship_r, speed);
    }

    // -----------------------------------------------------------------
    // Frame switching (GetVelocityRelTo + moveToFrame in src/main.cpp):
    // switching frames must preserve the ship's inertial state EXACTLY —
    // this is what used to be broken (the new frame's stasis term had the
    // wrong sign, injecting ~2*stasis of delta-v at every inertial<->rot
    // SOI crossing and visibly reshaping the orbit).
    // These replicate the exact main.cpp formulas.
    auto inertial_vel = [](Frame *F, const glm::dvec3 &p, const glm::dvec3 &v) {
        return F->root_orient * (v + F->GetStasisVelocity(p)) + F->root_vel;
    };
    auto switch_state = [](Frame *F, Frame *N, const glm::dvec3 &p, const glm::dvec3 &v)
        -> std::pair<glm::dvec3, glm::dvec3> {
        // GetVelocityRelTo (main.cpp): OLD frame's stasis term is added...
        glm::dvec3 vel = v;
        if (F != N) vel += F->GetStasisVelocity(p);
        glm::dvec3 vel_rel = F->GetOrientRelTo(N) * vel + F->GetVelocityRelTo(N);
        glm::dvec3 p_n = F->GetOrientRelTo(N) * p + F->GetPositionRelTo(N);
        // ...moveToFrame: NEW frame's stasis term is subtracted.
        glm::dvec3 v_n = vel_rel - N->GetStasisVelocity(p_n);
        return std::make_pair(p_n, v_n);
    };
    {
        Frame *F = eerbon;      // inertial
        Frame *N = eerbon_rot;  // rotating, same origin
        glm::dvec3 p_F(120e3, 40e3, 610e3);
        glm::dvec3 v_F(2895.0, 30.0, -120.0);

        // inertial state must be identical before and after the switch
        glm::dvec3 root_before = inertial_vel(F, p_F, v_F);
        std::pair<glm::dvec3, glm::dvec3> st = switch_state(F, N, p_F, v_F);
        glm::dvec3 root_after = inertial_vel(N, st.first, st.second);
        CHECK_TRUE(glm::length(root_after - root_before) < 1e-9,
                   "F->N switch preserves inertial velocity");

        // and a round trip must be the exact identity
        std::pair<glm::dvec3, glm::dvec3> back = switch_state(N, F, st.first, st.second);
        CHECK_TRUE(glm::length(back.first - p_F) < 1e-9 * glm::length(p_F),
                   "N->F round trip preserves position");
        CHECK_TRUE(glm::length(back.second - v_F) < 1e-9,
                   "N->F round trip preserves velocity");

        // the other direction too (the one Denis hits: inertial -> rotational)
        glm::dvec3 p_N(110e3, -25e3, 680e3);
        glm::dvec3 v_N(-2400.0, 15.0, 80.0);
        root_before = inertial_vel(N, p_N, v_N);
        st = switch_state(N, F, p_N, v_N);
        root_after = inertial_vel(F, st.first, st.second);
        CHECK_TRUE(glm::length(root_after - root_before) < 1e-9,
                   "N->F switch preserves inertial velocity");
        back = switch_state(F, N, st.first, st.second);
        CHECK_TRUE(glm::length(back.first - p_N) < 1e-9 * glm::length(p_N),
                   "F->N round trip preserves position");
        CHECK_TRUE(glm::length(back.second - v_N) < 1e-9,
                   "F->N round trip preserves velocity");
    }

    // -----------------------------------------------------------------
    // Fictitious forces (Frame::GetFictitiousAccel):
    //
    // The game integrates ships in whatever frame they are currently in.
    // In the inertial frame that is pure Kepler.  In a ROTATING frame the
    // integration must additionally include the Coriolis and centrifugal
    // terms, otherwise the ship's true (inertial) trajectory is perturbed
    // for as long as it stays there — every crossing into a rotating frame
    // visibly reshaped the orbit (the "ApA 2699 -> 1664 -> 2836" bounce).
    // This test: the same initial INERTIAL state, integrated for 300 s
    // either in the inertial frame (pure Kepler) or in the rotating frame
    // (Kepler + fictitious), must end at the same inertial state.
    {
        Frame *F = eerbon;      // inertial
        Frame *N = eerbon_rot;  // rotating, same origin
        const double mu = eerbon_mu;

        // Position-6 spawn state at t=0 (R(0)=identity): 610/1600 km
        // ellipse, at periapsis.
        const double rp0 = 610e3, ra0 = 1600e3;
        const double a0 = 0.5 * (rp0 + ra0);
        const double vI0 = std::sqrt(mu * (2.0 / rp0 - 1.0 / a0));
        glm::dvec3 p_I0(0.0, 0.0, rp0);
        glm::dvec3 v_I0(vI0, 0.0, 0.0);
        glm::dvec3 p_F0 = p_I0;
        glm::dvec3 v_F0 = v_I0 - N->GetStasisVelocity(p_F0);

        auto accel_inertial = [&](const glm::dvec3 &p, const glm::dvec3 &v) -> glm::dvec3 {
            (void)v;
            const double r = glm::length(p);
            return -mu * p / (r * r * r);
        };
        auto accel_rotating = [&](const glm::dvec3 &p, const glm::dvec3 &v) -> glm::dvec3 {
            const double r = glm::length(p);
            return -mu * p / (r * r * r) + N->GetFictitiousAccel(p, v);
        };
        auto rk4 = [&](const std::function<glm::dvec3(const glm::dvec3 &, const glm::dvec3 &)> &a,
                       glm::dvec3 p, glm::dvec3 v, double h)
            -> std::pair<glm::dvec3, glm::dvec3> {
            auto f = [&](const glm::dvec3 &p, const glm::dvec3 &v)
                -> std::pair<glm::dvec3, glm::dvec3> { return std::make_pair(v, a(p, v)); };
            auto k1 = f(p, v);
            auto k2 = f(p + 0.5 * h * k1.first, v + 0.5 * h * k1.second);
            auto k3 = f(p + 0.5 * h * k2.first, v + 0.5 * h * k2.second);
            auto k4 = f(p + h * k3.first, v + h * k3.second);
            return std::make_pair(p + h / 6.0 * (k1.first + 2.0 * k2.first + 2.0 * k3.first + k4.first),
                                  v + h / 6.0 * (k1.second + 2.0 * k2.second + 2.0 * k3.second + k4.second));
        };

        const double T = 300.0, h = 0.5;
        glm::dvec3 p1 = p_I0, v1 = v_I0; // integrated in the inertial frame
        glm::dvec3 p2 = p_F0, v2 = v_F0; // integrated in the rotating frame
        for(double t = 0.0; t < T - 1e-12; t += h) {
            auto s1 = rk4(accel_inertial, p1, v1, h);
            p1 = s1.first; v1 = s1.second;
            auto s2 = rk4(accel_rotating, p2, v2, h);
            p2 = s2.first; v2 = s2.second;
        }

        // Bring the rotating-frame end state back to inertial coordinates.
        sun->UpdateOrbitRails(T, 1.0);
        glm::dmat3 R = N->GetOrientRelTo(F);
        glm::dvec3 p2_I = R * p2;
        glm::dvec3 v2_I = R * (v2 + N->GetStasisVelocity(p2)) + N->GetVelocityRelTo(F);

        char buf[160];
        snprintf(buf, sizeof buf,
                 "rotating + fictitious == inertial integration (position: %.3e vs %.3e)",
                 glm::length(p2_I), glm::length(p1));
        CHECK_TRUE(glm::length(p2_I - p1) < 1e-6 * glm::length(p1), buf);
        snprintf(buf, sizeof buf,
                 "rotating + fictitious == inertial integration (velocity: %.3e vs %.3e)",
                 glm::length(v2_I), glm::length(v1));
        CHECK_TRUE(glm::length(v2_I - v1) < 1e-6 * glm::length(v1), buf);
    }

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
