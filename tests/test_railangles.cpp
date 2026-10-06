// test_railangles.cpp -- the angles a system file AUTHORS must be the angles
// the game MEASURES (issue #171). For every orbiting body in the shipped
// systems, feed its own epoch rail state back through the plane-angle routine
// and require it to reproduce the authored orb_incl / lon_asc_node.
//
// The state is `orient * orbit_pos0` -- the epoch vector expressed in the
// PARENT's axes -- not the raw orbit_pos0, which lives in the body's own local
// rail plane where the tilt has not been applied yet.
//
// The reference is a FRAME, not a normal: `incl_ref: "orbit"` bodies are
// authored against the parent's rail plane (+Y, longitude from +X), and
// `incl_ref: "equator"` bodies against the parent's EQUATOR (its spin axis,
// longitude from the equator frame's +X). Getting the reference wrong is a
// silent wrong answer, which is exactly how #171 hid.
//
// Runs from the repo root (it loads res/systems/*.json).

#include "orbit.h"
#include "system.h"
#include "frame.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

static int failures = 0;

#define CHECK_NEAR(what, got, want, tol) do { \
        double _g = (got), _w = (want), _t = (tol); \
        if(std::fabs(_g - _w) > _t) { \
            printf("FAIL %s:%d: %s = %.9g, want %.9g +- %g\n", \
                   __FILE__, __LINE__, what, _g, _w, _t); \
            failures++; \
        } \
    } while(0)

#define CHECK_TRUE(cond, what) do { \
        if(!(cond)) { \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, what); \
            failures++; \
        } \
    } while(0)

static nlohmann::json read_json(const char *path) {
    std::ifstream f(path);
    if(!f) {
        std::printf("FAIL: cannot open %s (run from the repo root)\n", path);
        std::exit(1);
    }
    return nlohmann::json::parse(f);
}

// Wrap a difference into (-pi, pi] so 359.9999 vs 0 compares as ~0.
static double angleDiff(double a, double b) {
    const double two = 2.0 * std::acos(-1.0);
    double d = std::fmod(a - b, two);
    if(d > std::acos(-1.0)) { d -= two; }
    if(d < -std::acos(-1.0)) { d += two; }
    return d;
}

static void checkSystem(const char *path) {
    nlohmann::json doc = read_json(path);
    System sys = load_system(path, nullptr, nullptr);
    sys.root->frame->UpdateOrbitRails(0.0);
    int checked = 0;

    for(const auto &bj : doc.value("bodies", nlohmann::json::array())) {
        if(!bj.contains("orbits") || !bj.contains("inertial")) { continue; }
        const auto &in = bj["inertial"];
        TerrainBody *tb = nullptr;
        for(TerrainBody *c : sys.bodies) {
            if(c->name == bj["name"].get<std::string>()) { tb = c; break; }
        }
        if(!tb || !tb->frame || !tb->frame->parent) { continue; }
        Frame *fr = tb->frame;
        Frame *par = fr->parent;

        const glm::dvec3 pos = fr->orient * fr->orbit_pos0;
        const glm::dvec3 vel = fr->orient * fr->orbit_vel0;
        if(glm::length(pos) == 0.0 || fr->parent_mu <= 0.0) { continue; }

        const bool equator_ref = in.value("incl_ref", std::string("orbit")) == "equator";
        glm::dvec3 n_hat(0.0, 1.0, 0.0), x_hat0(1.0, 0.0, 0.0);
        if(equator_ref) {
            n_hat = par->spinAxisRelTo(par);
            x_hat0 = par->rot_frame->equator_orient * glm::dvec3(1.0, 0.0, 0.0);
        }

        char what[160];
        /* uiRefPlane is what the ORBITAL readout and the map combo build their
           reference from; the loader is what the authored angles were measured
           against. If the two drift apart, the window and the system file
           disagree and nothing here notices. */
        if(equator_ref) {
            const RefPlane ui = uiRefPlane(par, glm::dvec3(0.0, 1.0, 0.0), kRefEquator);
            snprintf(what, sizeof what, "%s/%s uiRefPlane matches the loader's equatorial reference",
                     path, bj["name"].get<std::string>().c_str());
            CHECK_TRUE(glm::dot(ui.n_hat, n_hat) > 1.0 - 1e-12
                    && glm::dot(ui.x_hat0, x_hat0) > 1.0 - 1e-12, what);
        }

        const OrbitElements o = computeOrbitElements(pos, vel, fr->parent_mu);
        const PlaneAngles a = orbitPlaneAngles(o, RefPlane{n_hat, x_hat0});
        const double incl = in.value("orb_incl", 0.0);
        const double lan = in.value("lon_asc_node", 0.0);
        const double argp = in.value("arg_peri", 0.0);
        const double ecc = in.value("ecc", 0.0);
        const double two = 2.0 * std::acos(-1.0);
        snprintf(what, sizeof what, "%s/%s d_inc", path, bj["name"].get<std::string>().c_str());
        /* 1e-7 rad (5.7e-6 deg) rather than the atan2-level 1e-9: acos flattens
           near dot = 1, so an orbit authored IN the plane cannot read tighter
           than ~1.5e-8 no matter how exact the state is. */
        CHECK_NEAR(what, angleDiff(a.inc, incl), 0.0, 1e-7);
        // The state must carry the authored eccentricity too, or the LPe
        // comparison below is checking a different orbit.
        snprintf(what, sizeof what, "%s/%s d_ecc", path, bj["name"].get<std::string>().c_str());
        CHECK_NEAR(what, o.ecc, ecc, 1e-9);
        /* Every shipped reference supplies a zero direction lying in its own
           plane. If one did not, all the longitude checks below would be
           skipped silently and the test would still pass. */
        snprintf(what, sizeof what, "%s/%s lon_ok", path, bj["name"].get<std::string>().c_str());
        CHECK_TRUE(a.lon_ok, what);

        // The node is degenerate exactly for a body authored IN the reference
        // plane; the guard has to agree, or the readout dashes a real LAN (or
        // prints a random one).
        const bool authored_node = std::sin(incl) > 1e-6;
        snprintf(what, sizeof what, "%s/%s node_ok %d (authored incl %.6g)",
                 path, bj["name"].get<std::string>().c_str(), (int)a.node_ok, incl);
        CHECK_TRUE(a.node_ok == authored_node, what);
        // Same for the periapsis guard: state the rule instead of letting the
        // check below skip whenever it trips.
        snprintf(what, sizeof what, "%s/%s lpe_ok %d (authored ecc %.6g)",
                 path, bj["name"].get<std::string>().c_str(), (int)a.lpe_ok, ecc);
        CHECK_TRUE(a.lpe_ok == (o.ecc > 1e-4), what);
        snprintf(what, sizeof what, "%s/%s lpe in [0, 2pi) = %.9g",
                 path, bj["name"].get<std::string>().c_str(), a.lpe);
        CHECK_TRUE(a.lpe >= 0.0 && a.lpe < two, what);

        if(a.node_ok) {
            snprintf(what, sizeof what, "%s/%s d_lan", path, bj["name"].get<std::string>().c_str());
            CHECK_NEAR(what, angleDiff(a.lan, lan), 0.0, 1e-9);
        }
        /* LPe is the angle that survives a degenerate node: a coplanar body
           still states where its periapsis points, as lon_asc_node + arg_peri
           (0 + arg_peri). A circular body has no periapsis direction, which is
           what lpe_ok says above, so this runs whenever there is one. */
        if(a.lpe_ok) {
            snprintf(what, sizeof what, "%s/%s d_lpe", path, bj["name"].get<std::string>().c_str());
            CHECK_NEAR(what, angleDiff(a.lpe, lan + argp), 0.0, 1e-9);
        }
        /* The map's Orbital view measures against the orbit's OWN plane, where
           no direction is privileged: inc must read 0 and no longitude may be
           printed at all. Pinned per body because it is the one reference with
           no authored data to check it against. */
        RefPlane own(o.h_hat, glm::dvec3(1.0, 0.0, 0.0), false);
        const PlaneAngles oa = orbitPlaneAngles(o, own);
        /* acos(dot(h_hat, h_hat)) cannot read 0 tighter than ~1.5e-8 rad: the
           normal is unit to double precision, so its squared length is 1 +- eps
           and acos flattens there. 1e-6 rad is 0.00006 deg, far inside what any
           readout prints, and still catches a wrong-plane regression. */
        snprintf(what, sizeof what, "%s/%s own-plane inc", path, bj["name"].get<std::string>().c_str());
        CHECK_NEAR(what, oa.inc, 0.0, 1e-6);
        snprintf(what, sizeof what, "%s/%s own-plane dashes lon/node/lpe",
                 path, bj["name"].get<std::string>().c_str());
        CHECK_TRUE(!oa.lon_ok && !oa.node_ok && !oa.lpe_ok, what);
        ++checked;
    }
    std::printf("  %s: %d bodies round-tripped\n", path, checked);
}

int main() {
    checkSystem("res/systems/ksp_system.json");
    checkSystem("res/systems/old_system.json");
    // The solar systems are where "incl_ref": "equator" actually appears.
    checkSystem("res/systems/solar_system.json");

    if(failures == 0) {
        std::printf("test_railangles: all checks passed\n");
        return 0;
    }
    std::printf("test_railangles: %d check(s) failed\n", failures);
    return 1;
}
