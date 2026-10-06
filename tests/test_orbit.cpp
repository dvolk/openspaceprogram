// test_orbit: two-body orbital elements + time-to-apsis (src/orbit.h).
// Runs from the repo root:
//   make test   (or: g++ -O2 -std=c++20 -I./src -I./middleware/glm/ tests/test_orbit.cpp -o test_orbit && ./test_orbit)
//
// Pins the ApT/PeT fix: the OLD HUD math printed time-SINCE-periapsis as
// "PeT" and time-to-next-PERIAPSIS as "ApT". The contract now:
//   time_to_peri = (2pi - M) / n           (full period at periapsis itself)
//   time_to_apo  = (pi - M) / n  mod T     (full period at apoapsis itself)
// plus the hyperbolic case (no apoapsis, no period, one periapsis passage).
#include "orbit.h"

#include <cmath>
#include <cstdio>

static int failures = 0;
#define CHECK(cond) do { \
        if(!(cond)) { \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++; \
        } \
    } while(0)

#define CHECK_NEAR(a, b, tol) do { \
        double _a = (a), _b = (b), _t = (tol); \
        if(!(std::fabs(_a - _b) <= _t)) { \
            printf("FAIL %s:%d: %s = %.12g, want %.12g +- %g\n", \
                   __FILE__, __LINE__, #a, _a, _b, _t); \
            failures++; \
        } \
    } while(0)

static const double MU = 1.0e12;   // m^3/s^2

/* State on a conic with semi-major a and eccentricity e, at true anomaly
   nu (rad), in the perifocal frame: periapsis along +X, angular momentum
   along +Z. Valid for elliptic (a > 0) and hyperbolic (a < 0) alike. */
static void conic_state(double a, double e, double nu,
                        glm::dvec3 &pos, glm::dvec3 &vel) {
    const double p = a * (1.0 - e * e);             // semi-latus rectum (> 0)
    const double r = p / (1.0 + e * cos(nu));
    pos = glm::dvec3(r * cos(nu), r * sin(nu), 0.0);
    const double s = sqrt(MU / p);
    vel = glm::dvec3(-s * sin(nu), s * (e + cos(nu)), 0.0);
}

static void check_all_finite(const OrbitElements &o, const char *label) {
    const double f[] = { o.distance, o.speed, o.semi_major, o.ecc,
                         o.periapsis, o.apoapsis, o.period,
                         o.ang_momentum, o.energy, o.radial_vel,
                         o.true_anomaly, o.ecc_anomaly,
                         o.mean_anomaly, o.time_to_peri, o.time_to_apo };
    for(size_t i = 0; i < sizeof(f) / sizeof(f[0]); i++) {
        if(!std::isfinite(f[i])) {
            printf("FAIL %s: field %zu not finite (%g)\n", label, i, f[i]);
            failures++;
        }
    }
}

int main() {
    const double TWOPI = 2.0 * M_PI;

    // --- circular orbit ------------------------------------------------------
    {
        const double rc = 1.0e6;
        const double vc = sqrt(MU / rc);
        OrbitElements o = computeOrbitElements(glm::dvec3(rc, 0, 0),
                                               glm::dvec3(0, vc, 0), MU);
        check_all_finite(o, "circular");
        const double T = TWOPI * sqrt(rc * rc * rc / MU);
        CHECK_NEAR(o.ecc, 0.0, 1e-12);
        CHECK_NEAR(o.semi_major, rc, 1e-6 * rc);
        CHECK_NEAR(o.periapsis, rc, 1e-6 * rc);
        CHECK_NEAR(o.apoapsis, rc, 1e-6 * rc);
        CHECK_NEAR(o.period, T, 1e-9 * T);
        CHECK_NEAR(o.energy, -MU / (2.0 * rc), 1e-6 * MU / rc);
        // countdowns: periapsis a full period away, apoapsis half
        CHECK_NEAR(o.time_to_peri, T, 1e-9 * T);
        CHECK_NEAR(o.time_to_apo, T / 2.0, 1e-9 * T);
    }

    // --- ellipse at periapsis: the old bug showed PeT=0, ApT=T here ---------
    {
        const double rp = 1.0e6, ra = 4.0e6;
        const double a = (rp + ra) / 2.0;
        const double e = (ra - rp) / (ra + rp);       // 0.6
        const double vp = sqrt(MU * (2.0 / rp - 1.0 / a));
        OrbitElements o = computeOrbitElements(glm::dvec3(rp, 0, 0),
                                               glm::dvec3(0, vp, 0), MU);
        check_all_finite(o, "ellipse@peri");
        const double T = TWOPI * sqrt(a * a * a / MU);
        CHECK_NEAR(o.ecc, e, 1e-12);
        CHECK_NEAR(o.semi_major, a, 1e-9 * a);
        CHECK_NEAR(o.periapsis, rp, 1e-6 * rp);
        CHECK_NEAR(o.apoapsis, ra, 1e-6 * ra);
        CHECK_NEAR(o.true_anomaly, 0.0, 1e-9);
        CHECK_NEAR(o.mean_anomaly, 0.0, 1e-9);
        CHECK_NEAR(o.time_to_peri, T, 1e-9 * T);      // next periapsis = full orbit
        CHECK_NEAR(o.time_to_apo, T / 2.0, 1e-9 * T); // NOT T (the old ApT)
    }

    // --- ellipse at apoapsis --------------------------------------------------
    {
        const double rp = 1.0e6, ra = 4.0e6;
        const double a = (rp + ra) / 2.0;
        const double e = (ra - rp) / (ra + rp);
        const double va = sqrt(MU * (2.0 / ra - 1.0 / a));
        OrbitElements o = computeOrbitElements(glm::dvec3(-ra, 0, 0),
                                               glm::dvec3(0, -va, 0), MU);
        check_all_finite(o, "ellipse@apo");
        const double T = TWOPI * sqrt(a * a * a / MU);
        CHECK_NEAR(o.true_anomaly, M_PI, 1e-9);
        CHECK_NEAR(o.mean_anomaly, M_PI, 1e-9);
        CHECK_NEAR(o.time_to_peri, T / 2.0, 1e-9 * T);
        CHECK_NEAR(o.time_to_apo, T, 1e-9 * T);       // next apoapsis = full orbit
        CHECK_NEAR(o.ecc, e, 1e-12);
    }

    // --- ellipse at nu = pi/2: pins the countdown math for general M ---------
    {
        const double a = 2.5e6, e = 0.6;
        const double nu = M_PI / 2.0;
        glm::dvec3 pos, vel;
        conic_state(a, e, nu, pos, vel);
        OrbitElements o = computeOrbitElements(pos, vel, MU);
        check_all_finite(o, "ellipse@pi/2");
        const double T = TWOPI * sqrt(a * a * a / MU);
        const double n = TWOPI / T;
        CHECK_NEAR(o.true_anomaly, nu, 1e-9);
        CHECK_NEAR(o.distance, a * (1.0 - e * e) / (1.0 + e * cos(nu)),
                   1e-9 * a);
        // independent anomaly chain: tan(E/2) = sqrt((1-e)/(1+e)) tan(nu/2)
        const double E = 2.0 * atan(sqrt((1.0 - e) / (1.0 + e)) * tan(nu / 2.0));
        const double M = E - e * sin(E);
        CHECK_NEAR(o.ecc_anomaly, E, 1e-9);
        CHECK_NEAR(o.mean_anomaly, M, 1e-9);
        // the fix: time-to-Ap/Pe are countdowns to the NEXT passage.
        CHECK_NEAR(o.time_to_peri, (TWOPI - M) / n, 1e-9 * T);
        CHECK_NEAR(o.time_to_apo, (M_PI - M) / n, 1e-9 * T);
        // both strictly inside (0, T]
        CHECK(o.time_to_peri > 0.0 && o.time_to_peri <= T);
        CHECK(o.time_to_apo > 0.0 && o.time_to_apo <= T);
        // radial velocity: receding on the peri->apo half
        CHECK(o.radial_vel > 0.0);
    }

    // --- ellipse in the 3rd quadrant (past apoapsis): mod-T wrap --------------
    {
        const double a = 2.5e6, e = 0.6;
        const double nu = -M_PI / 2.0;   // == 3pi/2: inbound leg
        glm::dvec3 pos, vel;
        conic_state(a, e, nu, pos, vel);
        OrbitElements o = computeOrbitElements(pos, vel, MU);
        check_all_finite(o, "ellipse@3pi/2");
        const double T = TWOPI * sqrt(a * a * a / MU);
        const double n = TWOPI / T;
        CHECK_NEAR(o.true_anomaly, TWOPI - M_PI / 2.0, 1e-9);
        const double E = TWOPI + 2.0 * atan(sqrt((1.0 - e) / (1.0 + e)) * tan(nu / 2.0));
        const double M = E - e * sin(E);
        CHECK_NEAR(o.mean_anomaly, M, 1e-9);
        CHECK_NEAR(o.time_to_peri, (TWOPI - M) / n, 1e-9 * T);
        CHECK_NEAR(o.time_to_apo, (3.0 * M_PI - M) / n, 1e-9 * T); // wraps past T/2
        CHECK(o.radial_vel < 0.0);
    }

    /* --- plane angles, in the BODY-RAIL convention (issue #171) --------------
       The rails live in the XZ plane with normal +Y and prograde = +Y x r_hat,
       so rail longitude is atan2(-z, x) (railAz, #146). These states are built
       exactly the way railStateFromElements builds them, which is what makes
       the expected angles the ones a system file would author. Measuring from
       +Z instead -- the old computeOrbitElements contract -- made every
       rail-plane orbit read 90 deg tilted. */
    const RefPlane rail{glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(1.0, 0.0, 0.0)};
    /* The angles call takes the elements, not the raw state -- it only needs
       h_hat / ecc_dir / ecc, which computeOrbitElements already derives. */
    auto railAngles = [&](const glm::dvec3 &pos, const glm::dvec3 &vel) {
        return orbitPlaneAngles(computeOrbitElements(pos, vel, MU), rail);
    };
    {
        // Prograde and IN the rail plane, periapsis at rail longitude 0:
        // inc 0, and the orbit lying in the plane leaves NO node, so LAN is
        // undefined -- but the periapsis still points somewhere, so LPe lives.
        const double rp = 1.0e6, ra = 4.0e6;
        const double a = (rp + ra) / 2.0;
        const double vp = sqrt(MU * (2.0 / rp - 1.0 / a));
        const PlaneAngles p = railAngles(glm::dvec3(rp, 0, 0), glm::dvec3(0, 0, -vp));
        CHECK_NEAR(p.inc, 0.0, 1e-12);
        CHECK(p.lon_ok);
        CHECK(!p.node_ok);
        CHECK(p.lpe_ok);
        CHECK_NEAR(p.lpe, 0.0, 1e-9);
    }
    {
        // Same orbit, periapsis at rail longitude 90: r_hat = (cos f, 0, -sin f)
        // with f = 90, and prograde vel = +Y x r_hat.
        const double rp = 1.0e6, ra = 4.0e6;
        const double a = (rp + ra) / 2.0;
        const double vp = sqrt(MU * (2.0 / rp - 1.0 / a));
        const PlaneAngles p = railAngles(glm::dvec3(0, 0, -rp), glm::dvec3(-vp, 0, 0));
        CHECK_NEAR(p.inc, 0.0, 1e-12);
        CHECK(!p.node_ok);
        CHECK_NEAR(p.lpe, M_PI / 2.0, 1e-9);
    }
    {
        // Retrograde in the rail plane (h along -Y): inc 180, node undefined,
        // periapsis at rail longitude 270. This is the state the old +Z test
        // called "i=90, raan=0, arg_pe=90".
        const double rp = 1.0e6, ra = 4.0e6;
        const double a = (rp + ra) / 2.0;
        const double vp = sqrt(MU * (2.0 / rp - 1.0 / a));
        const PlaneAngles p = railAngles(glm::dvec3(0, 0, rp), glm::dvec3(-vp, 0, 0));
        CHECK_NEAR(p.inc, M_PI, 1e-12);
        CHECK(!p.node_ok);
        CHECK_NEAR(p.lpe, 3.0 * M_PI / 2.0, 1e-9);
    }
    {
        // That first orbit rotated 90 deg about X: the plane normal becomes +Z,
        // so inc 90 and the node is the rail +X (lan 0); periapsis sits ON the
        // node, so argp 0.
        const double rp = 1.0e6, ra = 4.0e6;
        const double a = (rp + ra) / 2.0;
        const double vp = sqrt(MU * (2.0 / rp - 1.0 / a));
        const PlaneAngles p = railAngles(glm::dvec3(rp, 0, 0), glm::dvec3(0, vp, 0));
        CHECK_NEAR(p.inc, M_PI / 2.0, 1e-12);
        CHECK(p.node_ok);
        CHECK_NEAR(p.lan, 0.0, 1e-9);
        CHECK_NEAR(p.argp, 0.0, 1e-9);
        CHECK_NEAR(p.lpe, 0.0, 1e-9);
    }
    {
        // Same plane, periapsis 90 deg past the node: argp 90, lpe 90.
        const double rp = 1.0e6, ra = 4.0e6;
        const double a = (rp + ra) / 2.0;
        const double vp = sqrt(MU * (2.0 / rp - 1.0 / a));
        const PlaneAngles p = railAngles(glm::dvec3(0, rp, 0), glm::dvec3(-vp, 0, 0));
        CHECK_NEAR(p.inc, M_PI / 2.0, 1e-12);
        CHECK_NEAR(p.lan, 0.0, 1e-9);
        CHECK_NEAR(p.argp, M_PI / 2.0, 1e-9);
        CHECK_NEAR(p.lpe, M_PI / 2.0, 1e-9);
    }
    {
        // LPe must stay inside [0, 2pi): here lan 270 + argp 180 is 450, and
        // the naive wrap (adding 2pi to a negative) would leave 450 on screen.
        // h along -X puts the node at rail longitude 270; periapsis at -Z is
        // 180 past it, so the periapsis longitude is 90.
        const double rp = 1.0e6, ra = 4.0e6;
        const double a = (rp + ra) / 2.0;
        const double vp = sqrt(MU * (2.0 / rp - 1.0 / a));
        const PlaneAngles p = railAngles(glm::dvec3(0, 0, -rp), glm::dvec3(0, -vp, 0));
        CHECK_NEAR(p.inc, M_PI / 2.0, 1e-12);
        CHECK_NEAR(p.lan, 3.0 * M_PI / 2.0, 1e-9);
        CHECK_NEAR(p.argp, M_PI, 1e-9);
        CHECK_NEAR(p.lpe, M_PI / 2.0, 1e-9);
    }
    {
        // Circular: no periapsis to point at, so LPe is undefined (the UI
        // dashes it) while inc still reads.
        const double rc = 1.0e6, vc = sqrt(MU / rc);
        const PlaneAngles p = railAngles(glm::dvec3(rc, 0, 0), glm::dvec3(0, 0, -vc));
        CHECK_NEAR(p.inc, 0.0, 1e-12);
        CHECK(!p.lpe_ok);
    }
    {
        // A barely-elliptical orbit still has an ecc_dir, but it is noise:
        // e = 1e-5 is below the guard, so LPe dashes rather than reporting a
        // direction that jitters freely.
        const double rp = 1.0e6, ra = rp * (1.0 + 2.0e-5);
        const double a = (rp + ra) / 2.0;
        const double vp = sqrt(MU * (2.0 / rp - 1.0 / a));
        const OrbitElements o = computeOrbitElements(glm::dvec3(rp, 0, 0),
                                                     glm::dvec3(0, 0, -vp), MU);
        CHECK(o.ecc < 1e-4);
        const PlaneAngles p = orbitPlaneAngles(o, rail);
        CHECK(!p.lpe_ok);
        CHECK(p.inc < 1e-12);
    }
    {
        // A reference whose zero direction is not IN the plane has no usable
        // longitude at all; the inclination still reads. Here x_hat0 is the
        // normal itself, so projecting it leaves nothing.
        const double rp = 1.0e6, ra = 4.0e6;
        const double a = (rp + ra) / 2.0;
        const double vp = sqrt(MU * (2.0 / rp - 1.0 / a));
        const OrbitElements o = computeOrbitElements(glm::dvec3(rp, 0, 0),
                                                     glm::dvec3(0, 0, -vp), MU);
        const glm::dvec3 n = glm::normalize(glm::dvec3(0.0, 1.0, 1.0));
        const PlaneAngles p = orbitPlaneAngles(o, RefPlane{n, n});
        CHECK_NEAR(p.inc, M_PI / 4.0, 1e-12);
        CHECK(!p.lon_ok);
        CHECK(!p.node_ok);
        CHECK(!p.lpe_ok);
        // A zero direction merely SLOPPY about the plane still works: it gets
        // projected, which is what lets a slightly-off reference keep reading
        // -- and it reads the SAME longitude as the exact one (+X already lies
        // in this plane).
        const glm::dvec3 sloppy = glm::normalize(n * 0.4 + glm::dvec3(1.0, 0.0, 0.0));
        const PlaneAngles q = orbitPlaneAngles(o, RefPlane{n, sloppy});
        const PlaneAngles exact = orbitPlaneAngles(o, RefPlane{n, glm::dvec3(1.0, 0.0, 0.0)});
        CHECK(q.lon_ok);
        CHECK(q.node_ok);
        CHECK_NEAR(q.inc, M_PI / 4.0, 1e-12);
        CHECK_NEAR(q.lan, exact.lan, 1e-12);
        CHECK_NEAR(q.lpe, exact.lpe, 1e-12);
    }
    {
        // A reference with no zero direction (the map's Orbital view) reports
        // the inclination and nothing else.
        const double rp = 1.0e6, ra = 4.0e6;
        const double a = (rp + ra) / 2.0;
        const double vp = sqrt(MU * (2.0 / rp - 1.0 / a));
        const OrbitElements o = computeOrbitElements(glm::dvec3(rp, 0, 0),
                                                     glm::dvec3(0, 0, -vp), MU);
        RefPlane own{glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(1.0, 0.0, 0.0), false};
        const PlaneAngles p = orbitPlaneAngles(o, own);
        CHECK_NEAR(p.inc, 0.0, 1e-12);
        CHECK(!p.lon_ok);
        CHECK(!p.node_ok);
        CHECK(!p.lpe_ok);
    }
    {
        // Near-equatorial noise guard: an in-game h carries ~1e-8 relative
        // jitter, so a genuinely equatorial orbit must NOT report a confident
        // random LAN. sin(inc) ~ 1e-8 must fall inside the node guard.
        const double rc = 1e6, vc = sqrt(MU / rc);
        const glm::dvec3 h_hat = glm::normalize(glm::dvec3(1e-8, 1.0, 0.0));
        const glm::dvec3 pos(rc, 0, 0);
        const glm::dvec3 vel = vc * glm::normalize(glm::cross(h_hat, pos));
        const PlaneAngles p = railAngles(pos, vel);
        CHECK(p.inc < 1e-7);
        CHECK(!p.node_ok);
    }

    // --- hyperbolic at periapsis ----------------------------------------------
    {
        const double e = 2.0, rp = 1.0e6;
        const double a = rp / (1.0 - e);              // -1e6
        const double vp = sqrt(MU * (2.0 / rp - 1.0 / a));
        OrbitElements o = computeOrbitElements(glm::dvec3(rp, 0, 0),
                                               glm::dvec3(0, vp, 0), MU);
        check_all_finite(o, "hyperbolic@peri");
        CHECK_NEAR(o.ecc, e, 1e-12);
        CHECK_NEAR(o.semi_major, a, 1e-9 * -a);
        CHECK_NEAR(o.periapsis, rp, 1e-6 * rp);       // finite, not NaN
        CHECK(o.apoapsis == -1.0);                    // no apoapsis
        CHECK(o.period == -1.0);                      // no period
        CHECK(o.time_to_apo == -1.0);
        CHECK(o.time_to_peri == -1.0);                // already passing it: gone
        CHECK(o.energy > 0.0);
    }

    // --- hyperbolic inbound: one future periapsis ------------------------------
    {
        const double e = 2.0, rp = 1.0e6;
        const double a = rp / (1.0 - e);
        glm::dvec3 pos1, vel1, pos2, vel2;
        conic_state(a, e, -1.0, pos1, vel1);
        conic_state(a, e, -1.5, pos2, vel2);
        OrbitElements o1 = computeOrbitElements(pos1, vel1, MU);
        OrbitElements o2 = computeOrbitElements(pos2, vel2, MU);
        check_all_finite(o1, "hyperbolic inbound nu=-1");
        check_all_finite(o2, "hyperbolic inbound nu=-1.5");
        CHECK_NEAR(o1.ecc, e, 1e-12);
        CHECK_NEAR(o1.true_anomaly, TWOPI - 1.0, 1e-9);   // wrapped inbound
        CHECK(o1.radial_vel < 0.0);                        // approaching
        // independent: H = 2 atanh(sqrt((e-1)/(e+1)) tan(nu/2)), ttPe = -Mh/nh
        const double nh = sqrt(MU / (-a * -a * -a));
        for(int k = 0; k < 2; k++) {
            const OrbitElements &oo = (k == 0) ? o1 : o2;
            const double nu = (k == 0) ? -1.0 : -1.5;
            const double H = 2.0 * atanh(sqrt((e - 1.0) / (e + 1.0)) * tan(nu / 2.0));
            const double Mh = e * sinh(H) - H;             // < 0 inbound
            CHECK_NEAR(oo.time_to_peri, -Mh / nh, 1e-9 * (-Mh / nh));
            CHECK(oo.time_to_peri > 0.0);
        }
        CHECK(o2.time_to_peri > o1.time_to_peri);          // farther out = longer
        CHECK(o1.time_to_apo == -1.0);
        CHECK(o1.period == -1.0);
    }

    // --- hyperbolic outbound: periapsis is gone --------------------------------
    {
        const double e = 2.0, rp = 1.0e6;
        const double a = rp / (1.0 - e);
        glm::dvec3 pos, vel;
        conic_state(a, e, +1.0, pos, vel);
        OrbitElements o = computeOrbitElements(pos, vel, MU);
        check_all_finite(o, "hyperbolic outbound");
        CHECK_NEAR(o.true_anomaly, 1.0, 1e-9);
        CHECK(o.radial_vel > 0.0);
        CHECK(o.time_to_peri == -1.0);
        CHECK(o.time_to_apo == -1.0);
    }

    // --- exactly parabolic: finite, no timing ----------------------------------
    {
        const double rp = 1.0e6;
        const double vp = sqrt(2.0 * MU / rp);        // escape speed, tangential
        OrbitElements o = computeOrbitElements(glm::dvec3(rp, 0, 0),
                                               glm::dvec3(0, vp, 0), MU);
        check_all_finite(o, "parabolic");
        CHECK_NEAR(o.ecc, 1.0, 1e-9);
        CHECK_NEAR(o.periapsis, rp, 1e-6 * rp);
        CHECK_NEAR(o.energy, 0.0, 1e-6 * MU / rp);
        CHECK(o.time_to_apo == -1.0);
    }

    // --- propagateKepler: circular quarter/whole period ----------------------
    {
        const double rc = 1.0e6;
        const double vc = sqrt(MU / rc);
        const double T = TWOPI * sqrt(rc * rc * rc / MU);
        const glm::dvec3 p0(rc, 0, 0), v0(0, vc, 0);
        glm::dvec3 p, v;
        propagateKepler(p0, v0, MU, T / 4.0, p, v);
        CHECK_NEAR(p.x, 0.0, 1e-6 * rc);
        CHECK_NEAR(p.y, rc, 1e-6 * rc);                 // 90 deg along +Y
        CHECK_NEAR(v.x, -vc, 1e-9 * vc);
        CHECK_NEAR(v.y, 0.0, 1e-9 * vc);
        propagateKepler(p0, v0, MU, T, p, v);
        CHECK_NEAR(glm::length(p - p0), 0.0, 1e-6 * rc); // full period: back
        CHECK_NEAR(glm::length(v - v0), 0.0, 1e-9 * vc);
    }

    // --- propagateKepler: ellipse periapsis -> apoapsis in T/2 ---------------
    {
        const double rp = 1.0e6, ra = 4.0e6;
        const double a = (rp + ra) / 2.0;
        const double vp = sqrt(MU * (2.0 / rp - 1.0 / a));
        const double va = sqrt(MU * (2.0 / ra - 1.0 / a));
        const double T = TWOPI * sqrt(a * a * a / MU);
        const glm::dvec3 p0(rp, 0, 0), v0(0, vp, 0);
        glm::dvec3 p, v;
        propagateKepler(p0, v0, MU, T / 2.0, p, v);
        CHECK_NEAR(p.x, -ra, 1e-6 * ra);
        CHECK_NEAR(p.y, 0.0, 1e-6 * ra);
        CHECK_NEAR(v.x, 0.0, 1e-9 * va);
        CHECK_NEAR(v.y, -va, 1e-9 * va);
        // whole-period folding: 10.25 periods == 0.25 periods
        glm::dvec3 p2, v2;
        propagateKepler(p0, v0, MU, 0.25 * T, p, v);
        propagateKepler(p0, v0, MU, 10.25 * T, p2, v2);
        CHECK_NEAR(glm::length(p2 - p), 0.0, 1e-6 * ra);
        CHECK_NEAR(glm::length(v2 - v), 0.0, 1e-9 * va);
    }

    // --- propagateKepler vs computeOrbitElements: apsis arrival --------------
    {
        const double a = 2.5e6, e = 0.6;
        glm::dvec3 p0, v0;
        conic_state(a, e, 1.1, p0, v0);   // arbitrary point on the ellipse
        OrbitElements o = computeOrbitElements(p0, v0, MU);
        glm::dvec3 p, v;
        propagateKepler(p0, v0, MU, o.time_to_apo, p, v);
        CHECK_NEAR(glm::length(p), o.apoapsis, 1e-7 * o.apoapsis);
        CHECK_NEAR(glm::dot(p, v), 0.0, 1e-5 * o.apoapsis * o.speed); // r_dot = 0
        propagateKepler(p0, v0, MU, o.time_to_peri, p, v);
        CHECK_NEAR(glm::length(p), o.periapsis, 1e-7 * o.periapsis);
        CHECK_NEAR(glm::dot(p, v), 0.0, 1e-5 * o.periapsis * o.speed);
    }

    // --- propagateKepler: conservation + reversibility -----------------------
    {
        const double a = 2.5e6, e = 0.6;
        glm::dvec3 p0, v0;
        conic_state(a, e, 2.2, p0, v0);
        OrbitElements o0 = computeOrbitElements(p0, v0, MU);
        glm::dvec3 p, v;
        propagateKepler(p0, v0, MU, 12345.678, p, v);
        OrbitElements o1 = computeOrbitElements(p, v, MU);
        CHECK_NEAR(o1.semi_major, o0.semi_major, 1e-9 * o0.semi_major);
        CHECK_NEAR(o1.ecc, o0.ecc, 1e-9);
        CHECK_NEAR(o1.ang_momentum, o0.ang_momentum, 1e-9 * o0.ang_momentum);
        CHECK_NEAR(o1.energy, o0.energy, 1e-9 * fabs(o0.energy));
        // and back
        glm::dvec3 p2, v2;
        propagateKepler(p, v, MU, -12345.678, p2, v2);
        CHECK_NEAR(glm::length(p2 - p0), 0.0, 1e-7 * a);
        CHECK_NEAR(glm::length(v2 - v0), 0.0, 1e-7 * glm::length(v0));
    }

    // --- propagateKepler: inbound state, long dt (fallback + bracket) -------
    // Inbound (vr < 0) pushes the universal-Kepler root PAST the small-dt
    // bracket end (target/r0): F(target/r0) < 0 there, so the bisection
    // fallback must expand the bracket outward. Long dt also keeps the
    // Newton initial guess near a zero of C(z), where it oscillates.
    {
        const double a = 2.5e6, e = 0.6;
        glm::dvec3 p0, v0;
        conic_state(a, e, 4.0, p0, v0);   // nu = 4 rad: inbound (sin nu < 0)
        OrbitElements o0 = computeOrbitElements(p0, v0, MU);
        CHECK(o0.radial_vel < 0.0);
        glm::dvec3 p, v;
        propagateKepler(p0, v0, MU, 0.4 * o0.period, p, v);
        OrbitElements o1 = computeOrbitElements(p, v, MU);
        CHECK_NEAR(o1.semi_major, a, 1e-9 * a);
        CHECK_NEAR(o1.ecc, e, 1e-9);
        CHECK_NEAR(o1.energy, o0.energy, 1e-9 * fabs(o0.energy));
        CHECK_NEAR(o1.ang_momentum, o0.ang_momentum, 1e-9 * o0.ang_momentum);
        // and back (negative dt: the bracket expands on the negative side)
        glm::dvec3 p2, v2;
        propagateKepler(p, v, MU, -0.4 * o0.period, p2, v2);
        CHECK_NEAR(glm::length(p2 - p0), 0.0, 1e-7 * a);
        CHECK_NEAR(glm::length(v2 - v0), 0.0, 1e-9 * glm::length(v0));
    }

    // --- propagateKepler: hyperbolic inbound -> periapsis passage ------------
    {
        const double e = 2.0, rp = 1.0e6;
        const double a = rp / (1.0 - e);
        glm::dvec3 p0, v0;
        conic_state(a, e, -0.9, p0, v0);  // inbound
        OrbitElements o = computeOrbitElements(p0, v0, MU);
        CHECK(o.time_to_peri > 0.0);
        glm::dvec3 p, v;
        propagateKepler(p0, v0, MU, o.time_to_peri, p, v);
        CHECK_NEAR(glm::length(p), rp, 1e-7 * rp);
        CHECK_NEAR(glm::dot(p, v), 0.0, 1e-5 * rp * glm::length(v));
        // conservation across an arbitrary hyperbolic step
        propagateKepler(p0, v0, MU, 500.0, p, v);
        OrbitElements o1 = computeOrbitElements(p, v, MU);
        CHECK_NEAR(o1.energy, o.energy, 1e-9 * o.energy);
        CHECK_NEAR(o1.ang_momentum, o.ang_momentum, 1e-9 * o.ang_momentum);
        // reversibility
        glm::dvec3 p2, v2;
        propagateKepler(p, v, MU, -500.0, p2, v2);
        CHECK_NEAR(glm::length(p2 - p0), 0.0, 1e-7 * glm::length(p0));
    }

    // --- propagateKepler: hyperbolic, large dt (issue 113: was silent NaN) ---
    // Overflow is in the Newton SEED z = alpha*chi^2 (chi ~ sqrt(mu)dt/r0),
    // not the solution |z| = dH^2. For a=-4e6, r0~1e6 the seed crosses |z|~5e5
    // near dt ~ 1e7; dt=1e8 is well past it.
    {
        const double e = 1.5, a = -4.0e6;   // v_inf ~ 10 km/s
        glm::dvec3 p0, v0;
        conic_state(a, e, -1.0, p0, v0);    // inbound
        OrbitElements o0 = computeOrbitElements(p0, v0, MU);
        glm::dvec3 p, v;
        propagateKepler(p0, v0, MU, 1.0e7, p, v);
        CHECK(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z));
        CHECK(std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z));
        OrbitElements o1 = computeOrbitElements(p, v, MU);
        check_all_finite(o1, "hyperbolic dt=1e7");
        CHECK_NEAR(o1.semi_major, a, 1e-6 * -a);
        CHECK_NEAR(o1.ecc, e, 1e-9);
        CHECK_NEAR(o1.energy, o0.energy, 1e-9 * o0.energy);
        CHECK_NEAR(o1.ang_momentum, o0.ang_momentum, 1e-9 * o0.ang_momentum);
        // further out still finite + conserved
        propagateKepler(p0, v0, MU, 1.0e8, p, v);
        CHECK(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z));
        CHECK(std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z));
        o1 = computeOrbitElements(p, v, MU);
        check_all_finite(o1, "hyperbolic dt=1e8");
        CHECK_NEAR(o1.energy, o0.energy, 1e-9 * o0.energy);
        CHECK_NEAR(o1.ang_momentum, o0.ang_momentum, 1e-9 * o0.ang_momentum);
        // reversibility at large dt
        glm::dvec3 p2, v2;
        propagateKepler(p, v, MU, -1.0e8, p2, v2);
        CHECK_NEAR(glm::length(p2 - p0), 0.0, 1e-5 * glm::length(p0));
        CHECK_NEAR(glm::length(v2 - v0), 0.0, 1e-5 * glm::length(v0));
    }

    // --- propagateKepler: outbound large dt and negative dt ------------------
    {
        const double e = 1.5, a = -4.0e6;
        glm::dvec3 p0, v0, p, v, p2, v2;
        conic_state(a, e, +1.0, p0, v0);    // outbound, periapsis gone
        propagateKepler(p0, v0, MU, 1.0e8, p, v);
        CHECK(std::isfinite(p.x) && std::isfinite(v.x));
        propagateKepler(p, v, MU, -1.0e8, p2, v2);
        CHECK_NEAR(glm::length(p2 - p0), 0.0, 1e-5 * glm::length(p0));
        propagateKepler(p0, v0, MU, -1.0e8, p, v);
        CHECK(std::isfinite(p.x) && std::isfinite(v.x));
        propagateKepler(p, v, MU, 1.0e8, p2, v2);
        CHECK_NEAR(glm::length(p2 - p0), 0.0, 1e-5 * glm::length(p0));
    }

    // --- propagateKepler: exactly radial hyperbolic (e=1, h=0) through M=0 --
    // e=1 at H=0 is the e cosh H - 1 = 0 Newton singularity. State: inbound
    // along -X at r=1e6 on a=-4e6; periapsis (r=0) is dt = -M0/n away.
    {
        const double a = -4.0e6, r0 = 1.0e6;
        const double v = sqrt(MU * (2.0 / r0 - 1.0 / a));
        const glm::dvec3 p0(r0, 0, 0), v0(-v, 0, 0);
        const double sinh_H0 = -0.75;   // r vr / (e sqrt(mu|a|)), e=1
        const double H0 = asinh(sinh_H0);
        const double M0 = sinh_H0 - H0;
        const double nh = sqrt(MU / (-a * -a * -a)); // mean motion, |a|^3
        const double dt_peri = -M0 / nh;
        glm::dvec3 p, v1;
        propagateKepler(p0, v0, MU, dt_peri, p, v1);
        CHECK(std::isfinite(p.x) && std::isfinite(v1.x));
        CHECK_NEAR(glm::length(p), 0.0, 1e3);          // periapsis at r=0
        CHECK_NEAR(p.y, 0.0, 1e-6);
        CHECK_NEAR(p.z, 0.0, 1e-6);
        // and still finite at large dt
        propagateKepler(p0, v0, MU, 1.0e6, p, v1);
        CHECK(std::isfinite(p.x) && std::isfinite(v1.x));
        CHECK_NEAR(p.y, 0.0, 1e-6 * glm::length(p));
        CHECK_NEAR(p.z, 0.0, 1e-6 * glm::length(p));
    }

    // --- propagateKepler: near-radial hyperbolic (e -> 1, the e cosh H - 1
    // Newton well). Keep periapsis above a few km. Round-trip must return;
    // e~1 long-coast reverse is ~1e-4 relative (f,g cancellation when
    // |a|/r0 is huge), so the return pin is 1e-4 not 1e-5.
    {
        // rp = |a|(e-1) = 7e11 * 1e-6 = 700 km, e-1 = 1e-6
        const double e = 1.0 + 1e-6, a = -7.0e11;
        glm::dvec3 p0, v0;
        conic_state(a, e, -0.5, p0, v0);
        OrbitElements o0 = computeOrbitElements(p0, v0, MU);
        CHECK(o0.ecc > 1.0);
        glm::dvec3 p, v, p2, v2;
        propagateKepler(p0, v0, MU, 1.0e7, p, v);
        CHECK(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z));
        CHECK(std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z));
        OrbitElements o1 = computeOrbitElements(p, v, MU);
        check_all_finite(o1, "near-radial hyperbolic");
        CHECK_NEAR(o1.energy, o0.energy, 1e-7 * o0.energy);
        // h is a small cross product on a near-radial conic: 1e-6 relative.
        CHECK_NEAR(o1.ang_momentum, o0.ang_momentum, 1e-6 * o0.ang_momentum);
        propagateKepler(p, v, MU, -1.0e7, p2, v2);
        CHECK_NEAR(glm::length(p2 - p0), 0.0, 1e-4 * glm::length(p0));
        CHECK_NEAR(glm::length(v2 - v0), 0.0, 1e-4 * glm::length(v0));
    }

    // --- propagateKepler: e~1 long coast round-trip (the M0 + n dt well) ----
    // Near e~1 a reverse coast used to land the wrong point on the right
    // conic (energy/h conserved, position off by many r0) because M0 + n dt
    // cancels. Solver now works in dH and must return (~1e-4 relative).
    {
        // rp = 7e12 * 1e-8 = 70 km
        const double e = 1.0 + 1e-8, a = -7.0e12;
        glm::dvec3 p0, v0, p, v, p2, v2;
        conic_state(a, e, -0.2, p0, v0);
        propagateKepler(p0, v0, MU, 1.0e7, p, v);
        CHECK(std::isfinite(p.x) && std::isfinite(v.x));
        propagateKepler(p, v, MU, -1.0e7, p2, v2);
        CHECK_NEAR(glm::length(p2 - p0), 0.0, 1e-4 * glm::length(p0));
        CHECK_NEAR(glm::length(v2 - v0), 0.0, 1e-4 * glm::length(v0));
    }
    {
        // rp = 4e9 * 1e-7 = 400 km
        const double e = 1.0 + 1e-7, a = -4.0e9;
        glm::dvec3 p0, v0, p, v, p2, v2;
        conic_state(a, e, -0.01, p0, v0);
        propagateKepler(p0, v0, MU, 1.0e7, p, v);
        CHECK(std::isfinite(p.x) && std::isfinite(v.x));
        propagateKepler(p, v, MU, -1.0e7, p2, v2);
        CHECK_NEAR(glm::length(p2 - p0), 0.0, 1e-4 * glm::length(p0));
        CHECK_NEAR(glm::length(v2 - v0), 0.0, 1e-4 * glm::length(v0));
    }

    // --- propagateKepler: in-place call (rails uses p,v as both in & out) ---
    {
        const double a = 2.5e6, e = 0.6;
        glm::dvec3 pa, va, pb, vb;
        conic_state(a, e, 0.7, pa, va);
        pb = pa; vb = va;
        // 30 sequential in-place steps must equal one distinct-var step
        for(int i = 0; i < 30; i++) {
            propagateKepler(pa, va, MU, 2.0, pa, va);
        }
        propagateKepler(pb, vb, MU, 60.0, pb, vb);
        CHECK_NEAR(glm::length(pa - pb), 0.0, 1e-7 * a);
        CHECK_NEAR(glm::length(va - vb), 0.0, 1e-7 * glm::length(vb));
        OrbitElements o1 = computeOrbitElements(pa, va, MU);
        CHECK_NEAR(o1.semi_major, a, 1e-9 * a);
        CHECK_NEAR(o1.ecc, e, 1e-9);
    }

    // --- railStateFromElements: epoch state in the body-rail convention ------
    // (XZ plane, +Y normal, prograde = +Y x r_hat = INCREASING rail longitude
    //  atan2(-z, x): at +X the velocity is -Z, at -Z it is -X -- the direction
    //  the old R_Y rotation rails produced. Authored angles are physical, so a
    //  positive longitude puts the body at -Z, not +Z.)
    {
        const double a = 1.0e8;
        const double vc = sqrt(MU / a);
        glm::dvec3 p, v;

        // circular, periapsis reference at +X
        CHECK(railStateFromElements(a, 0.0, 0.0, 0.0, MU, p, v));
        CHECK_NEAR(glm::length(p), a, 1e-9 * a);
        CHECK_NEAR(p.x, a, 1e-9 * a);
        CHECK_NEAR(p.y, 0.0, 1e-12);
        CHECK_NEAR(p.z, 0.0, 1e-9 * a);
        CHECK_NEAR(v.x, 0.0, 1e-12);
        CHECK_NEAR(v.y, 0.0, 1e-12);
        CHECK_NEAR(v.z, -vc, 1e-12 * vc);

        // circular, quarter phase: longitude pi/2 sits at -Z, prograde is -X
        CHECK(railStateFromElements(a, 0.0, 0.0, 0.5 * M_PI, MU, p, v));
        CHECK_NEAR(p.x, 0.0, 1e-9 * a);
        CHECK_NEAR(p.z, -a, 1e-9 * a);
        CHECK_NEAR(v.x, -vc, 1e-12 * vc);
        CHECK_NEAR(v.z, 0.0, 1e-12 * vc);

        // circular energy + angular momentum
        {
            const double E = 0.5 * glm::dot(v, v) - MU / a;
            CHECK_NEAR(E, -MU / (2.0 * a), 1e-12 * MU / a);
            CHECK_NEAR(glm::length(glm::cross(p, v)), a * vc, 1e-10 * a * vc);
        }

        // elliptic at periapsis: r = a(1-e), v = vis-viva, transverse
        const double e = 0.5, rp = a * (1.0 - e), ra = a * (1.0 + e);
        const double vp = sqrt(MU * (2.0 / rp - 1.0 / a));
        const double va = sqrt(MU * (2.0 / ra - 1.0 / a));
        CHECK(railStateFromElements(a, e, 0.0, 0.0, MU, p, v));
        CHECK_NEAR(glm::length(p), rp, 1e-9 * rp);
        CHECK_NEAR(p.x, rp, 1e-9 * rp);
        CHECK_NEAR(glm::length(v), vp, 1e-12 * vp);
        CHECK_NEAR(v.z, -vp, 1e-12 * vp);

        // elliptic at apoapsis: r = a(1+e), prograde flips to +Z
        CHECK(railStateFromElements(a, e, 0.0, M_PI, MU, p, v));
        CHECK_NEAR(glm::length(p), ra, 1e-9 * ra);
        CHECK_NEAR(p.x, -ra, 1e-9 * ra);
        CHECK_NEAR(v.z, va, 1e-12 * va);

        // elliptic MID-ORBIT (nu = pi/2): maximal radial component, the case
        // the apsis pins can't catch. r = p (semi-latus rectum); the state
        // must satisfy vis-viva, i.e. |v| = sqrt(MU(2/r - 1/a)) -- NOT the
        // total speed stacked onto the radial component. Past periapsis
        // (nu > 0) the body moves OUTWARD.
        const double pl = a * (1.0 - e * e);          // semi-latus rectum
        const double sm = sqrt(MU / pl);              // sqrt(mu/p)
        CHECK(railStateFromElements(a, e, 0.0, 0.5 * M_PI, MU, p, v));
        CHECK_NEAR(glm::length(p), pl, 1e-9 * pl);
        CHECK_NEAR(p.z, -pl, 1e-9 * pl);              // longitude pi/2 => -Z
        CHECK_NEAR(v.x, -sm, 1e-12 * sm);             // transverse = h/r = sm
        CHECK_NEAR(v.z, -sm * e, 1e-12 * sm);         // +sm*e OUTWARD along -Z
        const double vm = sqrt(MU * (2.0 / pl - 1.0 / a));
        CHECK_NEAR(glm::length(v), vm, 1e-12 * vm);   // vis-viva

        // argument of periapsis rotates the ellipse in-plane: periapsis at
        // longitude pi/2 lands on -Z, prograde there is -X
        CHECK(railStateFromElements(a, e, 0.5 * M_PI, 0.0, MU, p, v));
        CHECK_NEAR(p.x, 0.0, 1e-9 * rp);
        CHECK_NEAR(p.z, -rp, 1e-9 * rp);
        CHECK_NEAR(v.x, -vp, 1e-12 * vp);
        CHECK_NEAR(v.z, 0.0, 1e-12 * vp);

        // Round trip through the eccentricity vector (#172): the angles are
        // physical, so the recovered periapsis rail LONGITUDE atan2(-ev.z, ev.x)
        // must equal the authored arg_peri and the body must sit at longitude
        // arg_peri + true_anomaly. Only those two discriminate a mirrored
        // radial term (|e_vec|, radius and energy are sign-invariant); negating
        // it puts periapsis at arg_peri + 2*true_anomaly, -125.7 deg off for
        // Gilly. wrap() keeps |ap| > pi (Eeloo) / |nu0| > pi (Enceladus) legal.
        {
            struct RailPair { double e, ap, nu; };
            const RailPair cases[] = {
                {0.550,  0.1745,  2.0449},   // Gilly      (ksp_system)
                {0.235,  0.4363,  1.3351},   // Bop
                {0.260,  4.5379,  3.1406},   // Eeloo      arg_peri > pi
                {0.206, -0.5083, -3.0366},   // Mercury    (solar_system)
                {0.249, -1.9855, -0.3430},   // Pluto
                {0.0045, 0.0,     5.1185},   // Enceladus  nu0 > pi
                {0.7512, 0.0,     0.7124},   // Nereid     most eccentric shipped
            };
            auto wrap = [](double d) {
                while(d > M_PI) { d -= 2.0 * M_PI; }
                while(d < -M_PI) { d += 2.0 * M_PI; }
                return d;
            };
            for(const RailPair &c : cases) {
                CHECK(railStateFromElements(a, c.e, c.ap, c.nu, MU, p, v));
                const glm::dvec3 h = glm::cross(p, v);
                CHECK(h.y > 0.0);                 // prograde about +Y
                const glm::dvec3 ev =
                    glm::cross(v, h) / MU - p / glm::length(p);
                CHECK_NEAR(glm::length(ev), c.e, 1e-9);
                CHECK_NEAR(wrap(atan2(-ev.z, ev.x) - c.ap), 0.0, 1e-9);
                CHECK_NEAR(wrap(atan2(-p.z, p.x) - (c.ap + c.nu)), 0.0, 1e-9);
                // radius on the conic the elements describe
                const double pc = a * (1.0 - c.e * c.e);
                CHECK_NEAR(glm::length(p), pc / (1.0 + c.e * cos(c.nu)),
                           1e-9 * a);
                // energy: vis-viva on that same conic
                const double E = 0.5 * glm::dot(v, v) - MU / glm::length(p);
                CHECK_NEAR(E, -MU / (2.0 * a), 1e-12 * MU / a);
                // elements -> state -> elements. true_anomaly is built from
                // dot(e_vec, pos) and radial_vel*|h|/(mu*e), both free of any
                // reference plane, so this is independent of #171 -- and a
                // mirrored radial term returns -nu, which is what caught #172.
                const OrbitElements oe = computeOrbitElements(p, v, MU);
                CHECK_NEAR(oe.ecc, c.e, 1e-9);
                CHECK_NEAR(wrap(oe.true_anomaly - c.nu), 0.0, 1e-9);
            }
        }

        // propagate periapsis -> apoapsis is exactly half a period, and the
        // state must land on the apoapsis solution above (elements preserved)
        const double T = 2.0 * M_PI * sqrt(a * a * a / MU);
        glm::dvec3 p0, v0;
        CHECK(railStateFromElements(a, e, 0.0, 0.0, MU, p0, v0));
        propagateKepler(p0, v0, MU, 0.5 * T, p, v);
        CHECK_NEAR(glm::length(p), ra, 1e-8 * ra);
        CHECK_NEAR(p.x, -ra, 1e-8 * ra);
        CHECK_NEAR(v.z, va, 1e-9 * va);
        OrbitElements o1 = computeOrbitElements(p, v, MU);
        CHECK_NEAR(o1.semi_major, a, 1e-9 * a);
        CHECK_NEAR(o1.ecc, e, 1e-9);

        // guards
        CHECK(!railStateFromElements(-a, 0.0, 0.0, 0.0, MU, p, v));
        CHECK(!railStateFromElements(0.0, 0.0, 0.0, 0.0, MU, p, v));
        CHECK(!railStateFromElements(a, -0.1, 0.0, 0.0, MU, p, v));
        CHECK(!railStateFromElements(a, 1.0, 0.0, 0.0, MU, p, v));
        CHECK(!railStateFromElements(a, 0.0, 0.0, 0.0, 0.0, p, v));
    }

    if(failures == 0) {
        printf("test_orbit: all checks passed\n");
        return 0;
    }
    printf("test_orbit: %d FAILURES\n", failures);
    return 1;
}
