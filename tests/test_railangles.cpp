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

// The routine under test: textbook right-handed plane angles measured against
// the rail reference FRAME. #171's bug was measuring inclination from +Z and
// the node from cross((0,0,1), h) -- a plane the rails do not live in.
static PlaneAngles measure(const glm::dvec3 &pos, const glm::dvec3 &vel, double mu,
                          const glm::dvec3 &n_hat, const glm::dvec3 &x_hat0) {
    return orbitPlaneAngles(pos, vel, mu, RefPlane{n_hat, x_hat0});
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

        const PlaneAngles a = measure(pos, vel, fr->parent_mu, n_hat, x_hat0);
        const double incl = in.value("orb_incl", 0.0);
        const double lan = in.value("lon_asc_node", 0.0);
        const double argp = in.value("arg_peri", 0.0);
        const double ecc = in.value("ecc", 0.0);
        char what[160];
        snprintf(what, sizeof what, "%s/%s d_inc", path, bj["name"].get<std::string>().c_str());
        CHECK_NEAR(what, angleDiff(a.inc, incl), 0.0, 1e-9);

        // The node is degenerate exactly for a body authored IN the reference
        // plane; the guard has to agree, or the readout dashes a real LAN (or
        // prints a random one).
        const bool authored_node = std::sin(incl) > 1e-6;
        snprintf(what, sizeof what, "%s/%s node_ok %d (authored incl %.6g)",
                 path, bj["name"].get<std::string>().c_str(), (int)a.node_ok, incl);
        CHECK_TRUE(a.node_ok == authored_node, what);

        if(a.node_ok) {
            snprintf(what, sizeof what, "%s/%s d_lan", path, bj["name"].get<std::string>().c_str());
            CHECK_NEAR(what, angleDiff(a.lan, lan), 0.0, 1e-9);
        }
        /* LPe is the angle that survives a degenerate node: a coplanar body
           still states where its periapsis points, as lon_asc_node + arg_peri
           (0 + arg_peri). Circular bodies have none, so skip those. */
        if(a.peri_ok && ecc > 1e-6) {
            snprintf(what, sizeof what, "%s/%s d_lpe", path, bj["name"].get<std::string>().c_str());
            CHECK_NEAR(what, angleDiff(a.lpe, lan + argp), 0.0, 1e-9);
        }
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
