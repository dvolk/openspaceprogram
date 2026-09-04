// system.cpp -- the JSON star-system loader (see system.h).
#include "system.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "orbit.h"  // railStateFromElements

System load_system(const char *path, Shader *terrainshader, Shader *sunshader) {
    std::ifstream f(path);
    if(!f.is_open()) {
        throw std::runtime_error(std::string("system: cannot open ") + path);
    }
    nlohmann::json doc;
    try {
        doc = nlohmann::json::parse(f, nullptr, true);
    } catch(const std::exception &e) {
        throw std::runtime_error(std::string("system: bad JSON in ") + path
                                 + std::string(": ") + e.what());
    }

    if(!doc.is_object() || !doc.contains("bodies") || !doc["bodies"].is_array()
       || doc["bodies"].empty()) {
        throw std::runtime_error(std::string("system: no bodies in ") + path);
    }
    const nlohmann::json &bodies = doc["bodies"];
    const std::string home_name = doc.value("home", std::string(""));

    const double G = 6.674e-11;

    System sys;
    sys.root = nullptr;
    sys.home = nullptr;
    sys.moon = nullptr;

    // --- pass 1: create every body and its frames --------------------------
    for(size_t i = 0; i < bodies.size(); i++) {
        const nlohmann::json &bv = bodies[i];

        TerrainBody *body = new TerrainBody;
        body->frame = nullptr;
        body->rot_frame = nullptr;

        body->name       = bv.value("name", std::string("body"));
        const std::string type = bv.value("type", std::string("planet"));
        const double radius = bv.value("radius", 600000.0);
        const double mass   = bv.value("mass", 5e22);
        body->radius       = (float)radius;
        body->mass         = (float)mass;
        body->g            = bv.value("g", 9.81);
        body->mu           = G * mass;
        body->seed         = bv.value("seed", 0.0);

        // Legacy flat fields act as defaults for the surface parameters.
        Surface &s = body->surface;
        s.has_sea = bv.value("has_sea", false);

        if(bv.contains("surface") && bv["surface"].is_object()) {
            const nlohmann::json &sv = bv["surface"];
            s.amplitude   = sv.value("amplitude", s.amplitude);
            s.octaves     = sv.value("octaves", s.octaves);
            s.persistence = sv.value("persistence", s.persistence);
            s.frequency   = sv.value("frequency", s.frequency);
            if(sv.contains("sea_level")) {
                s.has_sea = true;
                s.sea_level = sv.value("sea_level", 0.0);
            }
            if(sv.contains("sea_color") && sv["sea_color"].is_array()
               && sv["sea_color"].size() >= 3) {
                const nlohmann::json &c = sv["sea_color"];
                s.sea_color = glm::vec3(c[0].get<float>(),
                                        c[1].get<float>(),
                                        c[2].get<float>());
            }
            if(sv.contains("palette") && sv["palette"].is_array()) {
                for(const nlohmann::json &stop : sv["palette"]) {
                    if(!stop.is_array() || stop.size() < 2
                       || !stop[1].is_array() || stop[1].size() < 3) {
                        continue;
                    }
                    PaletteStop ps;
                    ps.t = stop[0].get<float>();
                    ps.color = glm::vec3(stop[1][0].get<float>(),
                                         stop[1][1].get<float>(),
                                         stop[1][2].get<float>());
                    s.palette.push_back(ps);
                }
                std::sort(s.palette.begin(), s.palette.end(),
                          [](const PaletteStop &a, const PaletteStop &b) {
                              return a.t < b.t;
                          });
            }
            s.bands = sv.value("bands", false);
            if(sv.contains("band_count") && sv["band_count"].is_number_integer()) {
                s.band_count = std::max(1, sv["band_count"].get<int>());
            }
            if(sv.contains("atmosphere") && sv["atmosphere"].is_object()) {
                const nlohmann::json &av = sv["atmosphere"];
                s.atmosphere.enabled = true;
                if(av.contains("color") && av["color"].is_array()
                   && av["color"].size() >= 3) {
                    const nlohmann::json &c = av["color"];
                    s.atmosphere.color = glm::vec3(c[0].get<float>(),
                                                   c[1].get<float>(),
                                                   c[2].get<float>());
                }
                // thickness omitted => a visible rim at ~2% of the radius
                s.atmosphere.thickness =
                    av.value("thickness", (float)(body->radius * 0.02));
                s.atmosphere.power = av.value("power", 3.0f);
                s.atmosphere.intensity = av.value("intensity", 1.0f);
            }
        }
        // Per-body noise orientation: two irrational-angle turns so
        // neighbouring seeds land on uncorrelated surfaces.
        {
            const double a = body->seed * 2.39996322972865332;   // golden angle
            const double b = a * 1.618033988749895;
            const double ca = std::cos(a), sa = std::sin(a);
            const double cb = std::cos(b), sb = std::sin(b);
            const glm::dmat3 ry(ca, 0.0, -sa,  0.0, 1.0, 0.0,  sa, 0.0, ca);
            const glm::dmat3 rx(1.0, 0.0, 0.0,  0.0, cb, sb,  0.0, -sb, cb);
            s.seed_rot = glm::mat3(ry * rx);
        }

        // The highest relief above sea level, measured on the height
        // function itself: normalizes the palette elevation ramp and
        // sizes the atmosphere shell + the shadow-test bound. Fibonacci
        // sphere spread, 5% margin over the sampled max.
        if(s.bands) {
            s.max_height = 0.0f;   // gas giant: smooth sphere
        } else {
            const TerrainParams tp{s, (float)radius, nullptr};
            const int N = 2048;
            const float golden = 2.39996322972865332f;   // golden angle
            float hi = 0.0f;
            for(int i = 0; i < N; i++) {
                const float y = 1.0f - 2.0f * (i + 0.5f) / (float)N;
                const float rr = std::sqrt(std::max(0.0f, 1.0f - y * y));
                const glm::vec3 d(rr * std::cos(i * golden), y,
                                  rr * std::sin(i * golden));
                hi = std::max(hi, terrainHeight(d, tp) - (float)radius);
            }
            s.max_height = std::max(1.0f, (hi - s.sea_level) * 1.05f);
        }

        // Shader + elevation palette by body type.
        if(type == "star") {
            body->shader = sunshader;
            body->colour_func = GetColourSun;
        } else if(type == "moon") {
            body->shader = terrainshader;
            body->colour_func = GetColourMoon;
        } else { // "planet"
            body->shader = terrainshader;
            body->colour_func = GetColourEarth;
        }

        // --- inertial (non-rotating) frame ---------------------------------
        Frame *f = new Frame;
        f->name  = body->name + " (inertial)";
        f->body  = body;
        f->parent = nullptr;                 // wired in pass 2
        f->children.clear();
        f->rotating = false;
        f->has_rot_frame = false;
        f->pos = glm::dvec3(0, 0, 0);
        // GLM 1.0.0+: default-constructed matrices are zero, not identity.
        f->initial_orient = glm::dmat3(1.0);
        f->orient = glm::dmat3(1.0);
        f->vel = glm::dvec3(0);
        f->orb_ang_speed = 0;
        f->rot_ang_speed = 0;
        f->spin_axis = glm::dvec3(0, 1, 0);   // inertial frame does not spin
        f->soi = 1e6;
        f->root_pos = glm::dvec3(0);
        f->root_vel = glm::dvec3(0);
        f->root_orient = glm::dmat3(1.0);

        if(bv.contains("inertial") && bv["inertial"].is_object()) {
            const nlohmann::json &in = bv["inertial"];
            f->soi = in.value("soi", 1e6);
            if(in.contains("pos") && in["pos"].is_array() && in["pos"].size() >= 3) {
                const nlohmann::json &pos = in["pos"];
                f->pos = glm::dvec3(pos[0].get<double>(),
                                    pos[1].get<double>(),
                                    pos[2].get<double>());
            }
            f->orb_ang_speed = in.value("orb_ang_speed", 0.0);
            // Optional orbital plane orientation (radians): inclination i
            // tilts the orbital plane, and lon_asc_node (raan) rotates the
            // line of nodes from the parent's +X out to its longitude.
            // orient = R_Y(-raan) * R_X(i) maps the local orbital plane
            // (where pos lives) into the parent frame; identity when both are
            // zero, and it reduces to the old X-tilt when raan is absent.
            const double orb_incl = in.value("orb_incl", 0.0);
            const double lon_asc_node = in.value("lon_asc_node", 0.0);
            if(orb_incl != 0.0 || lon_asc_node != 0.0) {
                const double ci = std::cos(orb_incl), si = std::sin(orb_incl);
                const double co = std::cos(lon_asc_node), so = std::sin(lon_asc_node);
                f->orient = glm::dmat3(glm::dvec3(co, 0.0, so),
                                       glm::dvec3(-so * si, ci, co * si),
                                       glm::dvec3(-so * ci, -si, co * ci));
            }
        }
        body->frame = f;
        body->soi = f->soi;   // keep the body's soi in sync (display only)

        // --- rotating (near-body) frame -------------------------------------
        // Every body gets one. A body without a "rotating" JSON section (e.g.
        // the star) gets a DUMMY frame: zero spin (no stasis velocity, no
        // fictitious forces) and the standard near-body SOI (radius + 100 km,
        // the same convention the real data uses), so scenario radii, frame
        // resolution and frame switching work uniformly for every body.
        Frame *rf = new Frame;
        rf->name  = body->name + " (rotational)";
        rf->body  = body;
        rf->parent = f;                    // child of its own inertial frame
        rf->children.clear();
        rf->rotating = true;
        rf->has_rot_frame = false;
        rf->pos = glm::dvec3(0, 0, 0);
        rf->initial_orient = glm::dmat3(1.0);
        rf->orient = glm::dmat3(1.0);
        rf->vel = glm::dvec3(0);
        rf->orb_ang_speed = 0;
        rf->spin_axis = glm::dvec3(0, 1, 0);   // no axial tilt by default
        rf->root_pos = glm::dvec3(0);
        rf->root_vel = glm::dvec3(0);
        rf->root_orient = glm::dmat3(1.0);
        if(bv.contains("rotating") && bv["rotating"].is_object()) {
            const nlohmann::json &rot = bv["rotating"];
            rf->soi = rot.value("soi", 1e5);
            rf->rot_ang_speed = rot.value("rot_ang_speed", 0.0);
            // Optional axial tilt (radians): lean the spin axis away from the
            // orbital normal (local +Y) toward +X. The spin axis in the body
            // frame is then (sin t, cos t, 0); (0,1,0) when absent / zero.
            const double axial_tilt = rot.value("axial_tilt", 0.0);
            if(axial_tilt != 0.0) {
                rf->spin_axis = glm::dvec3(std::sin(axial_tilt),
                                           std::cos(axial_tilt), 0.0);
            }
        } else {
            rf->soi = radius + 100e3;       // near-body SOI convention
            rf->rot_ang_speed = 0.0;        // dummy: does not spin
        }
        body->rot_frame = rf;
        f->has_rot_frame = true;
        f->children.push_back(rf);

        // Build the terrain mesh + collision (needs shader/colour_func set).
        body->Create((float)radius, (float)mass);

        sys.bodies.push_back(body);
    }

    // --- pass 2: wire the parent/child frame tree --------------------------
    for(size_t i = 0; i < sys.bodies.size(); i++) {
        TerrainBody *body = sys.bodies[i];
        const nlohmann::json &bv = bodies[i];
        const std::string parent_name = bv.value("orbits", std::string(""));
        if(parent_name.empty()) {
            // The star: root of the frame tree.
            if(sys.root != nullptr) {
                throw std::runtime_error("system: more than one root body");
            }
            sys.root = body;
        } else {
            TerrainBody *parent = sys.find(parent_name);
            if(parent == nullptr) {
                throw std::runtime_error("system: '" + body->name
                                         + "' orbits unknown body '"
                                         + parent_name + "'");
            }
            body->frame->parent = parent->frame;
            parent->frame->children.push_back(body->frame);

            // Epoch orbital state for the Kepler rail. a comes from the
            // (existing) mean angular rate via Kepler's third law, so the
            // period and calendar are unchanged; e, arg_peri and the epoch
            // true anomaly are optional and default to the circular orbit
            // through the given pos (nu0 = its in-plane angle).
            Frame *f = body->frame;
            if(f->orb_ang_speed != 0.0) {
                const nlohmann::json &in =
                    bv.value("inertial", nlohmann::json::object());
                const double mu = parent->mu;
                const double w = f->orb_ang_speed;
                const double a = cbrt(mu / (w * w));
                const double e = in.value("ecc", 0.0);
                const double arg_peri = in.value("arg_peri", 0.0);
                const double nu0 = in.contains("true_anomaly0")
                    ? in["true_anomaly0"].get<double>()
                    : atan2(f->pos.z, f->pos.x) - arg_peri;
                if(!railStateFromElements(a, e, arg_peri, nu0, mu,
                                          f->orbit_pos0, f->orbit_vel0)) {
                    throw std::runtime_error("system: bad orbital elements "
                                             "for '" + body->name + "'");
                }
                f->parent_mu = mu;
                f->pos = f->orbit_pos0;
                f->vel = f->orbit_vel0;
            }
        }
    }

    if(sys.root == nullptr) {
        throw std::runtime_error("system: no root (star) body found");
    }

    // --- resolve the home planet and its first moon ------------------------
    if(!home_name.empty()) {
        sys.home = sys.find(home_name);
        if(sys.home == nullptr) {
            throw std::runtime_error("system: home body '" + home_name
                                     + "' not found");
        }
        // First moon = the first body whose parent is the home planet.
        for(size_t i = 0; i < sys.bodies.size(); i++) {
            const nlohmann::json &bv = bodies[i];
            if(bv.value("orbits", std::string("")) == home_name) {
                sys.moon = sys.bodies[i];
                break;
            }
        }
    } else if(sys.bodies.size() > 1) {
        // No explicit home: default to the first non-star body.
        for(size_t i = 0; i < sys.bodies.size(); i++) {
            if(sys.bodies[i] != sys.root) { sys.home = sys.bodies[i]; break; }
        }
    }

    // --- calendars ----------------------------------------------------------
    // Every body gets its own calendar from its spin (day) and orbit (year)
    // rates, so the HUD can show LOCAL time on whatever body the ship is in.
    // The year is snapped to a whole number of days (see calendar.h), so all
    // date boundaries fall on local midnight. Stars get an invalid calendar
    // (dummy zero-spin frame).
    const int epoch_year = 4724;  // the year the game starts in
    for(size_t i = 0; i < sys.bodies.size(); i++) {
        TerrainBody *b = sys.bodies[i];
        const double D = (b->rot_frame && b->rot_frame->rot_ang_speed > 0.0)
                       ? 2.0 * M_PI / b->rot_frame->rot_ang_speed : 0.0;
        const double Y = (b->frame && b->frame->orb_ang_speed > 0.0)
                       ? 2.0 * M_PI / b->frame->orb_ang_speed : 0.0;
        b->cal = Calendar::make(D, Y, epoch_year);
    }

    // Recompute the root-relative frame values so positions/velocities/orients
    // are consistent before the first render.
    sys.root->frame->UpdateOrbitRails(0.0);

    printf("Loaded system '%s': %zu bodies (home=%s, moon=%s)\n",
           path, sys.bodies.size(),
           sys.home ? sys.home->name.c_str() : "(none)",
           sys.moon ? sys.moon->name.c_str() : "(none)");

    return sys;
}
