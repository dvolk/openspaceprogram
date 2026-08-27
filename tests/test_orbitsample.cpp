// test_orbitsample.cpp -- unit tests for the orbit-sampling cache
// (src/orbitsample.h). The cache samples a closed orbit's points by
// propagating its state over one period; it must return the correct points
// for a given (pos, vel, mu), and it must not serve stale points for a
// different orbit (the element key must invalidate). Header-only pure math
// (orbit.h + glm + <vector>), so this links no imgui / Bullet / GL.

#include "orbitsample.h"

#include <cmath>

#include <cstdio>

static int g_failures = 0;

static void check(bool cond, const char *what) {
    if(!cond) {
        std::printf("FAIL %s\n", what);
        ++g_failures;
    }
}

// min/max sampled radius over the orbit.
static void orbitRadius(const std::vector<glm::dvec3> &pts,
                        double &mn, double &mx) {
    mn = 1e300; mx = 0.0;
    for(const glm::dvec3 &p : pts) {
        const double d = glm::length(p);
        mn = (d < mn) ? d : mn;
        mx = (d > mx) ? d : mx;
    }
}

int main() {
    const double mu = 3.5316e12;  // Kerbin-like
    const int N = 64;

    // 1. Circular orbit: every sampled point sits at the orbit radius.
    {
        const double r = 7.0e6, v = std::sqrt(mu / r);
        OrbitSampleCache c;
        const std::vector<glm::dvec3> &pts =
            c.sample(glm::dvec3(r, 0, 0), glm::dvec3(0, v, 0), mu, N);
        check(pts.size() == (size_t)N, "circular: N points");
        double mn, mx; orbitRadius(pts, mn, mx);
        check(std::fabs(mn - r) < 1e-3 * r && std::fabs(mx - r) < 1e-3 * r,
              "circular: |p| ~= radius");
    }

    // 2. Re-sampling the SAME state (coasting) gives identical points.
    {
        const double r = 7.0e6, v = std::sqrt(mu / r);
        const glm::dvec3 pos(r, 0, 0), vel(0, v, 0);
        OrbitSampleCache c;
        const std::vector<glm::dvec3> &a = c.sample(pos, vel, mu, N);
        const std::vector<glm::dvec3> &b = c.sample(pos, vel, mu, N);
        bool same = (a.size() == b.size());
        if(same) { for(size_t i = 0; i < a.size(); i++) { same = same && (a[i] == b[i]); } }
        check(same, "cache: same state -> identical points");
    }

    // 3. A DIFFERENT orbit must not reuse the previous one's points.
    {
        OrbitSampleCache c;
        const double r1 = 7.0e6, v1 = std::sqrt(mu / r1);
        const double r2 = 9.0e6, v2 = std::sqrt(mu / r2);
        // sample() returns a reference to the cache's own storage, which the
        // next sample() overwrites -- so copy orbit A's result before
        // sampling orbit B (main.cpp samples each entry once per frame, so it
        // never hits this aliasing).
        std::vector<glm::dvec3> a =
            c.sample(glm::dvec3(r1, 0, 0), glm::dvec3(0, v1, 0), mu, N);
        const std::vector<glm::dvec3> &b =
            c.sample(glm::dvec3(r2, 0, 0), glm::dvec3(0, v2, 0), mu, N);
        const double ra = glm::length(a.front()), rb = glm::length(b.front());
        check(std::fabs(ra - r1) < 1e-3 * r1, "cache: orbit A radius");
        check(std::fabs(rb - r2) < 1e-3 * r2, "cache: orbit B radius");
        check(ra != rb, "cache: distinct orbits differ");
    }

    // 4. Eccentric orbit: min/max sampled radius match periapsis / apoapsis.
    {
        const double a_ax = 7.0e6, e = 0.5;
        const double peri = a_ax * (1.0 - e);
        const double v_p = std::sqrt(mu * (1.0 + e) / peri);  // speed at periapsis
        OrbitSampleCache c;
        const std::vector<glm::dvec3> &pts =
            c.sample(glm::dvec3(peri, 0, 0), glm::dvec3(0, v_p, 0), mu, N);
        double mn, mx; orbitRadius(pts, mn, mx);
        check(std::fabs(mn - peri) < 1e-2 * peri, "ecc: min ~= periapsis");
        check(std::fabs(mx - a_ax * (1.0 + e)) < 1e-2 * a_ax, "ecc: max ~= apoapsis");
    }

    // 5. A non-closed (hyperbolic) trajectory samples nothing.
    {
        const double r = 7.0e6, v_esc = std::sqrt(2.0 * mu / r);
        OrbitSampleCache c;
        const std::vector<glm::dvec3> &pts =
            c.sample(glm::dvec3(r, 0, 0), glm::dvec3(0, 1.1 * v_esc, 0), mu, N);
        check(pts.empty(), "hyperbolic: empty");
    }

    // 6. use_cache=false (a ship off its rails) re-samples but still returns
    // the correct points for the current state -- the gate must not corrupt
    // or serve stale data.
    {
        const double r = 7.0e6, v = std::sqrt(mu / r);
        const glm::dvec3 pos(r, 0, 0), vel(0, v, 0);
        OrbitSampleCache c;
        // Copy the cached result first: use_cache=false re-samples into the
        // cache's own storage, overwriting any reference held to it.
        std::vector<glm::dvec3> cached = c.sample(pos, vel, mu, N, true);
        const std::vector<glm::dvec3> &fresh = c.sample(pos, vel, mu, N, false);
        bool same = (cached.size() == fresh.size());
        if(same) { for(size_t i = 0; i < cached.size(); i++) { same = same && (cached[i] == fresh[i]); } }
        check(same, "gate: use_cache=false -> same correct points");
        double mn, mx; orbitRadius(fresh, mn, mx);
        check(std::fabs(mn - r) < 1e-3 * r && std::fabs(mx - r) < 1e-3 * r,
              "gate: off-rails points still on the circle");
    }

    if(g_failures == 0) {
        std::printf("test_orbitsample: all checks passed\n");
        return 0;
    }
    std::printf("test_orbitsample: %d check(s) failed\n", g_failures);
    return 1;
}
