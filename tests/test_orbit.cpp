// test_orbit: two-body orbital elements + time-to-apsis (src/orbit.h).
// Runs from the repo root:
//   make test   (or: g++ -O2 -std=c++11 -I./src -I./middleware/glm/ tests/test_orbit.cpp -o test_orbit && ./test_orbit)
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
                         o.periapsis, o.apoapsis, o.inclination, o.period,
                         o.ang_momentum, o.energy, o.radial_vel, o.raan,
                         o.arg_periapsis, o.true_anomaly, o.ecc_anomaly,
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
        CHECK_NEAR(o.inclination, 0.0, 1e-12);        // h along +Z
        CHECK_NEAR(o.energy, -MU / (2.0 * rc), 1e-6 * MU / rc);
        // h || Z -> equatorial -> node undefined -> 0, not NaN
        CHECK_NEAR(o.raan, 0.0, 1e-12);
        CHECK_NEAR(o.arg_periapsis, 0.0, 1e-12);      // circular -> undefined
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

    // --- tilted orbit: inclination, RAAN, argument of periapsis ---------------
    {
        // in-plane ellipse rotated 90 deg about X: i=90, node along +X,
        // periapsis still on the node -> arg_pe = 0
        const double rp = 1.0e6, ra = 4.0e6;
        const double a = (rp + ra) / 2.0;
        const double vp = sqrt(MU * (2.0 / rp - 1.0 / a));
        OrbitElements o = computeOrbitElements(glm::dvec3(rp, 0, 0),
                                               glm::dvec3(0, 0, vp), MU);
        check_all_finite(o, "tilted w=0");
        CHECK_NEAR(o.inclination, M_PI / 2.0, 1e-12);
        CHECK_NEAR(o.raan, 0.0, 1e-12);
        CHECK_NEAR(o.arg_periapsis, 0.0, 1e-9);
    }
    {
        // periapsis along +Z (the spawn convention): i=90, raan=0, arg_pe=90
        const double rp = 1.0e6, ra = 4.0e6;
        const double a = (rp + ra) / 2.0;
        const double vp = sqrt(MU * (2.0 / rp - 1.0 / a));
        OrbitElements o = computeOrbitElements(glm::dvec3(0, 0, rp),
                                               glm::dvec3(-vp, 0, 0), MU);
        check_all_finite(o, "tilted w=90");
        CHECK_NEAR(o.inclination, M_PI / 2.0, 1e-12);
        CHECK_NEAR(o.raan, 0.0, 1e-12);
        CHECK_NEAR(o.arg_periapsis, M_PI / 2.0, 1e-9);
        CHECK_NEAR(o.periapsis, rp, 1e-6 * rp);
        CHECK_NEAR(o.apoapsis, ra, 1e-6 * ra);
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

    if(failures == 0) {
        printf("test_orbit: all checks passed\n");
        return 0;
    }
    printf("test_orbit: %d FAILURES\n", failures);
    return 1;
}
