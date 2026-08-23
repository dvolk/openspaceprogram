//
// Validates the spawn-frame logic introduced in spawn_vehicle (src/main.cpp):
//   1. resolve_frame_by_soi picks the deepest/innermost SOI containing a point.
//   2. For each starting scenario, the resolved frame is the one the
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
    sun->orient = glm::dmat3(1.0);
    sun->rot_ang_speed = 0;
    sun->orb_ang_speed = 0;
    sun->soi = 9999999999999999.0;

    eerbon->name = "eerbon";
    eerbon->parent = sun;
    eerbon->children = { eerbon_rot, moon };
    eerbon->rotating = false;
    eerbon->has_rot_frame = true;
    eerbon->pos = glm::dvec3(0, 0, -13599840260.0);
    eerbon->orient = glm::dmat3(1.0);
    eerbon->rot_ang_speed = 0;
    eerbon->orb_ang_speed = 0.00000068269186570822291594437651;
    eerbon->soi = 84159286.0;

    eerbon_rot->name = "eerbon_rot";
    eerbon_rot->parent = eerbon;
    eerbon_rot->children = {};
    eerbon_rot->rotating = true;
    eerbon_rot->pos = glm::dvec3(0);
    eerbon_rot->orient = glm::dmat3(1.0);
    eerbon_rot->initial_orient = glm::dmat3(1.0);
    eerbon_rot->rot_ang_speed = 0.00029157090303706880702966723086;
    eerbon_rot->orb_ang_speed = 0;
    eerbon_rot->soi = 700000.0;

    moon->name = "moon";
    moon->parent = eerbon;
    moon->children = { moon_rot };
    moon->rotating = false;
    moon->has_rot_frame = true;
    moon->pos = glm::dvec3(-12e6, 0, 0);
    moon->orient = glm::dmat3(1.0);
    moon->rot_ang_speed = 0;
    moon->orb_ang_speed = 0.00004520797578987211820731369629;
    moon->soi = 2429559.1;

    moon_rot->name = "moon_rot";
    moon_rot->parent = moon;
    moon_rot->children = {};
    moon_rot->rotating = true;
    moon_rot->pos = glm::dvec3(0);
    moon_rot->orient = glm::dmat3(1.0);
    moon_rot->initial_orient = glm::dmat3(1.0);
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
    const char *desc;
    bool on_pad;
    double r;             // orbit radius from body center (pad: surface radius)
    double mu;
    bool polar;           // polar plane (body local +Y) vs equatorial (+Z)
    Frame *bodyFrame;     // frame whose root_pos is the body's world pos
    Frame *expectedFrame;
    bool ellipse;         // 10x1000 km ASL elliptical case (overrides r/polar)
    int ell_phase;        // 0: at periapsis; 1: at apoapsis; 2: at 90 deg
    double rp;            // periapsis distance from body center
    double ra;            // apoapsis distance from body center
};

int main() {
    Frame *sun, *eerbon, *eerbon_rot, *moon, *moon_rot;
    Frame *root = make_tree(sun, eerbon, eerbon_rot, moon, moon_rot);
    // Settle the root-relative values (as main does before spawn_vehicle).
    root->UpdateOrbitRails(0.0, 1.0 / 60.0);

    // Build the spawn cases (scenarios) exactly as spawn_vehicle computes them.
    std::vector<SpawnCase> cases;

    // Helper: r = radius + alt_frac * (rot soi - radius); rot soi = radius + 100 km.
    auto add = [&](const char *desc, bool on_pad, double alt_frac, bool polar,
                   double bodyRadius, double bodyMu, Frame *bodyFrame,
                   Frame *expectedFrame) {
        SpawnCase c;
        c.desc = desc; c.on_pad = on_pad;
        c.r = on_pad ? bodyRadius
                     : bodyRadius + alt_frac * 100000.0;
        c.mu = bodyMu; c.polar = polar; c.bodyFrame = bodyFrame;
        c.expectedFrame = expectedFrame;
        c.ellipse = false; c.ell_phase = -1; c.rp = 0; c.ra = 0;
        cases.push_back(c);
    };

    // Helper: the ellipse-* scenarios (10 km x 1000 km ASL), spawned at
    // periapsis (0), apoapsis (1), or 90 deg true anomaly (2).
    auto add_ell = [&](const char *desc, int phase, double bodyRadius, double bodyMu,
                       Frame *bodyFrame, Frame *expectedFrame) {
        SpawnCase c;
        c.desc = desc; c.on_pad = false;
        c.r = 0; c.mu = bodyMu; c.polar = false; c.bodyFrame = bodyFrame;
        c.expectedFrame = expectedFrame;
        c.ellipse = true; c.ell_phase = phase;
        c.rp = bodyRadius + 10e3;
        c.ra = bodyRadius + 1000e3;
        cases.push_back(c);
    };

    // Pad scenarios: on the surface (frame must be the rotating one).
    add("pad (Eerbon)",            true,  0.0,  false, eerbon_radius, eerbon_mu, eerbon, eerbon_rot);
    add("pad-polar (Eerbon)",      true,  0.0,  true,  eerbon_radius, eerbon_mu, eerbon, eerbon_rot);
    // Orbit scenarios around Eerbon (rot soi = 700 km):
    //   0.85 -> 685 km (inside rot soi), 1.25 -> 725 km (outside), 5 -> 1100 km.
    add("rot-orbit (Eerbon)",      false, 0.85, false, eerbon_radius, eerbon_mu, eerbon, eerbon_rot);
    add("inertial-orbit (Eerbon)", false, 1.25, false, eerbon_radius, eerbon_mu, eerbon, eerbon);
    add("high-orbit (Eerbon)",     false, 5.0,  false, eerbon_radius, eerbon_mu, eerbon, eerbon);
    add("high-polar (Eerbon)",     false, 5.0,  true,  eerbon_radius, eerbon_mu, eerbon, eerbon);
    // Same scenarios around the Moon (rot soi = 300 km): 285 km / 325 km.
    add("rot-orbit (Moon)",        false, 0.85, false, moon_radius, moon_mu, moon, moon_rot);
    add("inertial-orbit (Moon)",   false, 1.25, false, moon_radius, moon_mu, moon, moon);
    // Ellipse scenarios around Eerbon (rot soi = 700 km):
    //   peri 610 km (inside), apo 7000 km (outside), mid p = 1122 km (outside).
    add_ell("ellipse-peri (Eerbon)", 0, eerbon_radius, eerbon_mu, eerbon, eerbon_rot);
    add_ell("ellipse-apo (Eerbon)",  1, eerbon_radius, eerbon_mu, eerbon, eerbon);
    add_ell("ellipse-mid (Eerbon)",  2, eerbon_radius, eerbon_mu, eerbon, eerbon);
    // Same around the Moon (rot soi = 300 km): peri 210 km, apo 1200 km,
    // mid p = 357 km (outside).
    add_ell("ellipse-peri (Moon)", 0, moon_radius, moon_mu, moon, moon_rot);
    add_ell("ellipse-apo (Moon)",  1, moon_radius, moon_mu, moon, moon);
    add_ell("ellipse-mid (Moon)",  2, moon_radius, moon_mu, moon, moon);

    // faceAlong -- same function as spawn_vehicle (src/main.cpp): nose
    // (local +Z) along dir, roll axis = coordinate axis most orthogonal to dir.
    auto faceAlong = [](const glm::dvec3 &dir) -> glm::dmat3 {
        const glm::dvec3 z = glm::normalize(dir);
        const glm::dvec3 refs[3] = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
        int best = 0;
        for(int i = 1; i < 3; i++) {
            if(std::fabs(glm::dot(refs[i], z)) < std::fabs(glm::dot(refs[best], z))) best = i;
        }
        const glm::dvec3 x = glm::normalize(refs[best] - glm::dot(refs[best], z) * z);
        const glm::dvec3 y = glm::cross(z, x);
        return glm::dmat3(x, y, z);
    };

    for(SpawnCase &c : cases) {
        const glm::dvec3 center = c.bodyFrame->root_pos;
        glm::dvec3 worldPos, velWorld;
        if(c.ellipse) {
            // The exact ellipse branch of spawn_vehicle (main.cpp):
            // equatorial plane, periapsis along +Z, 90 deg prograde is +X.
            const double p = 2.0 * c.rp * c.ra / (c.rp + c.ra);
            const double e = (c.ra - c.rp) / (c.ra + c.rp);
            const double h = std::sqrt(c.mu * p);
            const glm::dvec3 xhat(1, 0, 0);
            const glm::dvec3 zhat(0, 0, 1);
            if(c.ell_phase == 0) { // at periapsis
                worldPos = center + zhat * c.rp;
                velWorld = xhat * (h / c.rp);
            } else if(c.ell_phase == 1) { // at apoapsis
                worldPos = center - zhat * c.ra;
                velWorld = -xhat * (h / c.ra);
            } else { // 90 deg true anomaly
                worldPos = center + xhat * p;
                velWorld = (xhat * e - zhat) * (h / p);
            }
        } else {
            const glm::dvec3 rhat_local = c.polar ? glm::dvec3(0, 1, 0) : glm::dvec3(0, 0, 1);
            worldPos = center + c.bodyFrame->root_orient * (rhat_local * c.r);
            // Circular orbital speed (vis-viva with semi-major axis == r).
            const double speed = std::sqrt(c.mu / c.r);
            glm::dvec3 rhat = glm::normalize(worldPos - center);
            glm::dvec3 vhat = c.polar ? glm::cross(glm::dvec3(1, 0, 0), rhat)
                                      : glm::cross(glm::dvec3(0, 1, 0), rhat);
            velWorld = speed * vhat;
        }
        const double speed = glm::length(velWorld);
        glm::dvec3 vhat = glm::normalize(velWorld);
        Frame *frame = resolve_frame_by_soi(sun, worldPos);
        char buf[256];

        snprintf(buf, sizeof buf, "%s: resolves to '%s'", c.desc, frame->name.c_str());
        CHECK_TRUE(frame == c.expectedFrame, buf);

        if(c.on_pad) {
            printf("  %s: frame='%s'  r=%.0f m  OK\n",
                   c.desc, frame->name.c_str(), c.r);
            continue; // pad cases are frame-resolution checks only
        }
        glm::dvec3 target = glm::transpose(frame->root_orient) * (worldPos - frame->root_pos);
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
        snprintf(buf, sizeof buf, "%s: implied inertial vel == body vel + orbital vel", c.desc);
        CHECK_TRUE(glm::length(implied - intended) < 1e-6 * speed, buf);

        // (b) Ship must stay put under the main-loop SOI logic: not outside the
        //     resolved frame's SOI, and not inside any of its children's SOIs.
        double ship_r = glm::length(target);
        snprintf(buf, sizeof buf, "%s: not outside resolved SOI (r=%.0f, soi=%.0f)",
                 c.desc, ship_r, frame->soi);
        CHECK_TRUE(!(ship_r > frame->soi + 10000.0), buf);

        for(Frame *child : frame->children) {
            glm::dvec3 in_child = frame->GetOrientRelTo(child) * target
                                + frame->GetPositionRelTo(child);
            double dist = glm::length(in_child);
            snprintf(buf, sizeof buf,
                     "%s: would NOT switch into child '%s' (dist=%.0f >= soi=%.0f)",
                     c.desc, child->name.c_str(), dist, child->soi);
            CHECK_TRUE(dist > child->soi - 10000.0, buf);
        }

        // (c) Prograde orientation: the nose (local +Z, 3rd matrix column)
        //     must point along vhat, and the orient must be orthonormal.
        glm::dmat3 orient = faceAlong(vhat);
        glm::dvec3 nose = orient * glm::dvec3(0, 0, 1);
        snprintf(buf, sizeof buf, "%s: nose along prograde", c.desc);
        CHECK_TRUE(glm::length(nose - vhat) < 1e-9, buf);
        bool ortho = true;
        const glm::dmat3 gram = orient * glm::transpose(orient);
        for(int i = 0; i < 3 && ortho; i++)
            for(int j = 0; j < 3; j++)
                if(std::fabs(gram[i][j] - ((i == j) ? 1.0 : 0.0)) > 1e-9) ortho = false;
        snprintf(buf, sizeof buf, "%s: orientation orthonormal", c.desc);
        CHECK_TRUE(ortho, buf);

        printf("  %s: frame='%s'  r=%.0f m  |v|=%.1f m/s  OK\n",
               c.desc, frame->name.c_str(), ship_r, speed);
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
