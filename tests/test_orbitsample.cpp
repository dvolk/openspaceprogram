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

    // 7. EVEN sampling: two states on the SAME ellipse (different phase) must
    // yield the same point set. This is the fix for the lopsided eccentric
    // outline -- the grid is a fixed even set in eccentric anomaly, NOT
    // time-measured-from-the-start-state (which would offset the grid by the
    // ship's phase and starve periapsis). The orbit is equatorial, so
    // in-plane angle -> radius is one-to-one: equal angle sets == equal
    // point sets.
    {
        const double a = 7.0e6, e = 0.6;
        const double rp = a * (1.0 - e);
        const double vp = std::sqrt(mu * (1.0 + e) / rp);   // periapsis speed
        const glm::dvec3 p1(rp, 0.0, 0.0), v1(0.0, vp, 0.0);  // periapsis, equatorial
        const double period = 2.0 * M_PI * std::sqrt(a * a * a / mu);
        glm::dvec3 p2, v2;
        propagateKepler(p1, v1, mu, 0.25 * period, p2, v2);  // same ellipse, +90 deg

        OrbitSampleCache c1, c2;   // separate instances -> no storage aliasing
        std::vector<glm::dvec3> s1 = c1.sample(p1, v1, mu, N);   // copies (see test 3)
        std::vector<glm::dvec3> s2 = c2.sample(p2, v2, mu, N);
        check(s1.size() == (size_t)N && s2.size() == (size_t)N, "even: N points each");

        // Set comparison via nearest-neighbour distance, in Cartesian coords.
        // (A sorted-angle comparison trips on the 0 / 2pi wrap: the periapsis
        // point reads as angle 0 for one phase and 2pi for the other.) Every
        // point of one sampling must have a near-identical partner in the
        // other; the grid spacing (~5e5 m here) is >> the tolerance, so the
        // nearest neighbour is unambiguous.
        bool same = (s1.size() == s2.size());
        if(same) {
            for(const glm::dvec3 &p : s1) {
                double best = 1e300;
                for(const glm::dvec3 &q : s2) {
                    const double d = glm::length(p - q);
                    if(d < best) { best = d; }
                }
                if(best > 1e-3 * rp) { same = false; break; }   // 0.1% of periapsis radius
            }
        }
        check(same, "even: same ellipse, two phases -> same point set");
    }

    // 8. Open (hyperbolic) trajectory: a finite arc around periapsis,
    // truncated at r_cap. The periapsis is the arc's minimum radius, the
    // endpoints sit at r_cap, every point lies in the orbital plane, and the
    // arc is symmetric about the periapsis direction.
    {
        const double e = 1.5, rp = 7.0e6;
        const double v_p = std::sqrt(mu * (1.0 + e) / rp);  // speed at periapsis
        const glm::dvec3 pos(rp, 0.0, 0.0), vel(0.0, v_p, 0.0); // periapsis, equatorial
        const double r_cap = 10.0 * rp;
        std::vector<glm::dvec3> pts = sampleOpenTrajectory(pos, vel, mu, N, r_cap);
        check(pts.size() == (size_t)N, "open: N points");
        double mn, mx; orbitRadius(pts, mn, mx);
        check(mn >= rp * (1.0 - 1e-9), "open: min radius >= periapsis");
        check(mn < rp * 1.01, "open: min radius ~= periapsis");
        check(mx <= r_cap * (1.0 + 1e-9), "open: max radius <= r_cap");
        check(mx > r_cap * 0.99, "open: max radius ~= r_cap (truncated)");
        bool in_plane = true;
        for(const glm::dvec3 &q : pts) {
            if(std::fabs(q.z) > 1e-6 * rp) { in_plane = false; break; }
        }
        check(in_plane, "open: points lie in the orbital plane");
        // Symmetric about the periapsis direction (+X): each point (x, y) has
        // a partner (x, -y). (The grid nu_i = -nu_{N-1-i} guarantees this.)
        bool symmetric = true;
        for(const glm::dvec3 &q : pts) {
            double best = 1e300;
            for(const glm::dvec3 &w : pts) {
                const double d = glm::length(glm::dvec3(w.x, -w.y, 0.0) - q);
                if(d < best) { best = d; }
            }
            if(best > 1e-3 * rp) { symmetric = false; break; }
        }
        check(symmetric, "open: arc symmetric about periapsis direction");
    }

    // 9. The open-trajectory sampler returns nothing for a CLOSED orbit.
    {
        const double r = 7.0e6, v = std::sqrt(mu / r);
        std::vector<glm::dvec3> pts = sampleOpenTrajectory(
            glm::dvec3(r, 0, 0), glm::dvec3(0, v, 0), mu, N, 10.0 * r);
        check(pts.empty(), "open: closed orbit -> empty");
    }

    // 10. TRANSFER ARC (elliptic): a leg from periapsis to apoapsis. The
    // endpoints must be exactly the departure and arrival states, and the
    // points must be even in ECCENTRIC ANOMALY (radius r_i = a(1 - e cos
    // E_i), E_i = pi i / N) -- NOT uniform in time, which would crowd the
    // slow apoapsis end and starve the fast periapsis end.
    {
        const double e = 0.5;
        const double peri = 7.0e6;
        const double a = peri / (1.0 - e);
        const double v_p = std::sqrt(mu * (1.0 + e) / peri);   // periapsis speed
        const glm::dvec3 pos(peri, 0.0, 0.0), vel(0.0, v_p, 0.0); // periapsis
        const double period = 2.0 * M_PI * std::sqrt(a * a * a / mu);
        const double tof = 0.5 * period;   // periapsis -> apoapsis
        std::vector<glm::dvec3> pts = sampleTransferArc(pos, vel, mu, tof, N);
        check(pts.size() == (size_t)(N + 1), "xfer-ell: N+1 points");
        check(glm::length(pts.front() - pos) < 1e-6 * peri,
              "xfer-ell: departure exact");
        glm::dvec3 arr_p, arr_v;
        propagateKepler(pos, vel, mu, tof, arr_p, arr_v);
        check(glm::length(pts.back() - arr_p) < 1e-6 * peri,
              "xfer-ell: arrival exact");
        bool even = true;
        for(int i = 0; i <= N; i++) {
            const double r_exp = a * (1.0 - e * std::cos(M_PI * i / N));
            const double r_act = glm::length(pts[i]);
            if(std::fabs(r_act - r_exp) > 1e-3 * r_exp) { even = false; break; }
        }
        check(even, "xfer-ell: even-in-anomaly radii");
    }

    // 10b. TRANSFER ARC (high eccentricity, e = 0.9995): the most lopsided
    // legs are the ones that most need even-in-anomaly. Pin the conic gate: a
    // high-e elliptic leg must take the even-in-anomaly path, not the
    // uniform-in-time fallback (which would crowd the slow apoapsis end).
    {
        const double e = 0.9995;
        const double peri = 7.0e6;
        const double a = peri / (1.0 - e);
        const double v_p = std::sqrt(mu * (1.0 + e) / peri);   // periapsis speed
        const glm::dvec3 pos(peri, 0.0, 0.0), vel(0.0, v_p, 0.0); // periapsis
        const double period = 2.0 * M_PI * std::sqrt(a * a * a / mu);
        const double tof = 0.5 * period;   // periapsis -> apoapsis
        std::vector<glm::dvec3> pts = sampleTransferArc(pos, vel, mu, tof, N);
        check(pts.size() == (size_t)(N + 1), "xfer-he: N+1 points");
        check(glm::length(pts.front() - pos) < 1e-6 * peri, "xfer-he: dep exact");
        glm::dvec3 arr_p, arr_v;
        propagateKepler(pos, vel, mu, tof, arr_p, arr_v);
        check(glm::length(pts.back() - arr_p) < 1e-6 * peri, "xfer-he: arr exact");
        bool even = true;
        for(int i = 0; i <= N; i++) {
            const double r_exp = a * (1.0 - e * std::cos(M_PI * i / N));
            const double r_act = glm::length(pts[i]);
            if(std::fabs(r_act - r_exp) > 1e-3 * r_exp) { even = false; break; }
        }
        check(even, "xfer-he: even-in-anomaly radii");
    }

    // 11. TRANSFER ARC (hyperbolic): a leg leaving periapsis. Endpoints exact
    // and even in HYPERBOLIC ANOMALY (radius r_i = |a|(e cosh H_i - 1),
    // H_i = H2 i / N).
    {
        const double e = 1.5;
        const double rp = 7.0e6;
        const double a_abs = rp / (e - 1.0);
        const double v_p = std::sqrt(mu * (1.0 + e) / rp);      // periapsis speed
        const glm::dvec3 pos(rp, 0.0, 0.0), vel(0.0, v_p, 0.0); // periapsis
        const double tof = 600.0;
        std::vector<glm::dvec3> pts = sampleTransferArc(pos, vel, mu, tof, N);
        check(pts.size() == (size_t)(N + 1), "xfer-hyp: N+1 points");
        check(glm::length(pts.front() - pos) < 1e-6 * rp,
              "xfer-hyp: departure exact");
        glm::dvec3 arr_p, arr_v;
        propagateKepler(pos, vel, mu, tof, arr_p, arr_v);
        check(glm::length(pts.back() - arr_p) < 1e-6 * rp,
              "xfer-hyp: arrival exact");
        const double H2 = computeOrbitElements(arr_p, arr_v, mu).ecc_anomaly;
        bool even = true;
        for(int i = 0; i <= N; i++) {
            const double r_exp = a_abs * (e * std::cosh(H2 * i / N) - 1.0);
            const double r_act = glm::length(pts[i]);
            if(std::fabs(r_act - r_exp) > 1e-3 * r_exp) { even = false; break; }
        }
        check(even, "xfer-hyp: even-in-H radii");
    }

    // 12. TRANSFER ARC (near-circular): e < 1e-3 has no well-defined anomaly,
    // so the even-in-time fallback is used -- which is already even for a
    // circular orbit. All points sit at the orbit radius.
    {
        const double r = 7.0e6, v = std::sqrt(mu / r);
        const glm::dvec3 pos(r, 0.0, 0.0), vel(0.0, v, 0.0);
        std::vector<glm::dvec3> pts = sampleTransferArc(pos, vel, mu, 100.0, N);
        check(pts.size() == (size_t)(N + 1), "xfer-circ: N+1 points");
        check(glm::length(pts.front() - pos) < 1e-6 * r,
              "xfer-circ: departure exact");
        // The arc must SPREAD along the orbit (a nonzero ToF moves the ship),
        // not collapse to the departure point (which the anomaly path would do
        // for a degenerate e = 0 reference direction).
        check(glm::length(pts.back() - pts.front()) > 1e-3 * r,
              "xfer-circ: arc spans > 0");
        bool on_circle = true;
        for(const glm::dvec3 &p : pts) {
            if(std::fabs(glm::length(p) - r) > 1e-3 * r) { on_circle = false; break; }
        }
        check(on_circle, "xfer-circ: points at radius");
    }

    // 13. TRANSFER ARC (zero ToF): the arc degenerates to the departure point.
    {
        const double e = 0.5, peri = 7.0e6;
        const double v_p = std::sqrt(mu * (1.0 + e) / peri);
        const glm::dvec3 pos(peri, 0.0, 0.0), vel(0.0, v_p, 0.0);
        std::vector<glm::dvec3> pts = sampleTransferArc(pos, vel, mu, 0.0, N);
        check(pts.size() == (size_t)(N + 1), "xfer-t0: N+1 points");
        bool all_dep = true;
        for(const glm::dvec3 &p : pts) {
            if(glm::length(p - pos) > 1e-6 * peri) { all_dep = false; break; }
        }
        check(all_dep, "xfer-t0: all points at departure");
    }

    // 14. Cache survives coasting drift. railsTick / UpdateOrbitRails
    // re-propagate the live state every step, which perturbs it in the last
    // bits while leaving the conic unchanged; an exact == key used to miss
    // every frame and re-run the Kepler solves (the map's cost center).
    {
        const double r = 7.0e6, v = std::sqrt(mu / r);
        glm::dvec3 p(r, 0, 0), vel(0, v, 0);
        const glm::dvec3 p0 = p, v0 = vel;
        OrbitSampleCache c;
        std::vector<glm::dvec3> first = c.sample(p0, v0, mu, N);
        for(int i = 0; i < 2000; i++) {
            propagateKepler(p, vel, mu, 0.016, p, vel);
        }
        // A further last-bit nudge of the kind a second propagator leaves.
        p.x += 1e-4;
        const std::vector<glm::dvec3> &later = c.sample(p, vel, mu, N);
        bool same = (later.size() == first.size());
        if(same) {
            for(size_t i = 0; i < first.size(); i++) {
                if(first[i] != later[i]) { same = false; break; }
            }
        }
        check(same, "cache: coasting drift still hits");
    }

    // 15. N is part of the key: a LOD change of the sample count must not
    // serve the previous grid.
    {
        OrbitSampleCache c;
        const double r = 7.0e6, v = std::sqrt(mu / r);
        const glm::dvec3 pos(r, 0, 0), vel(0, v, 0);
        c.sample(pos, vel, mu, 16);
        const std::vector<glm::dvec3> &b = c.sample(pos, vel, mu, 32);
        check(b.size() == 32, "cache: N is part of the key");
    }

    // 16. A real orbit change (a different radius) still invalidates even
    // with the tolerant key.
    {
        OrbitSampleCache c;
        const double r1 = 7.0e6, v1 = std::sqrt(mu / r1);
        const double r2 = 8.0e6, v2 = std::sqrt(mu / r2);
        std::vector<glm::dvec3> a =
            c.sample(glm::dvec3(r1, 0, 0), glm::dvec3(0, v1, 0), mu, N);
        const std::vector<glm::dvec3> &b =
            c.sample(glm::dvec3(r2, 0, 0), glm::dvec3(0, v2, 0), mu, N);
        check(std::fabs(glm::length(b.front()) - r2) < 1e-3 * r2,
              "cache: tolerant key still invalidates on a real change");
        check(glm::length(a.front()) != glm::length(b.front()),
              "cache: distinct radii after invalidation");
    }

    // 17. Near-circular: periapsis direction is noise and must NOT thrash
    // the key (e_hat is ignored below e = 1e-2). Same plane (XY), just a
    // rotated start phase -- a different plane must and does miss.
    {
        const double r = 7.0e6, v = std::sqrt(mu / r);
        const glm::dvec3 pos(r, 0, 0), vel(0, v, 0);
        OrbitSampleCache c;
        std::vector<glm::dvec3> first = c.sample(pos, vel, mu, N);
        // Same circle in the XY plane, rotated start phase.
        const double ang = 0.37;
        const glm::dvec3 pos2(r * std::cos(ang), r * std::sin(ang), 0.0);
        const glm::dvec3 vel2(-v * std::sin(ang), v * std::cos(ang), 0.0);
        const std::vector<glm::dvec3> &later = c.sample(pos2, vel2, mu, N);
        bool same = (later.size() == first.size());
        if(same) {
            for(size_t i = 0; i < first.size(); i++) {
                if(first[i] != later[i]) { same = false; break; }
            }
        }
        check(same, "cache: near-circle ignores periapsis direction");
    }

    // 18. orbitConicSize: circular + eccentric apoapsis, and the open conic
    // reports no closed size.
    {
        const double r = 7.0e6, v = std::sqrt(mu / r);
        double a = 0.0, apo = -1.0;
        orbitConicSize(glm::dvec3(r, 0, 0), glm::dvec3(0, v, 0), mu, a, apo);
        check(std::fabs(a - r) < 1e-3 * r, "size: circular sma = r");
        check(std::fabs(apo - r) < 1e-3 * r, "size: circular apo = r");
        const double e = 0.5, peri = 7.0e6;
        const double v_p = std::sqrt(mu * (1.0 + e) / peri);
        orbitConicSize(glm::dvec3(peri, 0, 0), glm::dvec3(0, v_p, 0), mu, a, apo);
        check(std::fabs(a - peri / (1.0 - e)) < 1e-2 * peri,
              "size: eccentric sma");
        check(std::fabs(apo - a * (1.0 + e)) < 1e-2 * peri,
              "size: eccentric apo");
        orbitConicSize(glm::dvec3(r, 0, 0), glm::dvec3(0, 1.5 * v, 0), mu, a, apo);
        check(apo < 0.0, "size: hyperbolic -> no closed apo");
    }

    if(g_failures == 0) {
        std::printf("test_orbitsample: all checks passed\n");
        return 0;
    }
    std::printf("test_orbitsample: %d check(s) failed\n", g_failures);
    return 1;
}
