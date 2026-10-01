// system.cpp -- the JSON star-system loader (see system.h).
#include "system.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <cstdio>
#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "orbit.h"  // railStateFromElements
#include "resdir.h"
#include "bodylimits.h"  // the derived shell + SOI laws + the ordering asserts

System load_system(const char *path, Shader *terrainshader, Shader *sunshader,
                   std::function<void(size_t i, size_t total,
                                      const std::string &name)> progress) {
    // `path` is the logical name ("res/systems/...") for logs/e2e; only the
    // open sees the resolved filesystem path (resdir.h).
    std::ifstream f(resdir::path(path));
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

    // Which SOI law derives the inertial spheres (bodylimits.h). Default
    // patched_conic: it reproduces the KSP wiki values the old data
    // hardcoded; solar_system*.json author "hill" (their old values).
    const SoiLaw law =
        soiLawFromName(doc.value("soi_law", std::string("patched_conic")));

    const double G = 6.674e-11;

    System sys;
    sys.root = nullptr;
    sys.home = nullptr;
    // Delete partially-created bodies on a throw mid-build (return commits).
    struct BodyCleanup {
        std::vector<TerrainBody *> *b;
        bool commit;
        BodyCleanup(std::vector<TerrainBody *> &b_) : b(&b_), commit(false) {}
        ~BodyCleanup() { if(!commit && b) { for(TerrainBody *x : *b) { delete x; } } }
    } cleanup{ sys.bodies };

    // --- pass 1: create every body and its frames --------------------------
    for(size_t i = 0; i < bodies.size(); i++) {
        const nlohmann::json &bv = bodies[i];

        TerrainBody *body = new TerrainBody;
        body->frame = nullptr;
        body->rot_frame = nullptr;

        body->name       = bv.value("name", std::string("body"));
        const std::string type = bv.value("type", std::string("planet"));
        body->type       = (type == "star") ? BodyType::Star
                         : (type == "moon") ? BodyType::Moon
                                            : BodyType::Planet;
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
                // Physical drag (src/drag.h): both optional, 0 = no drag.
                // A rim can exist without these, and vice versa.
                s.atmosphere.sea_level_density =
                    av.value("sea_level_density", 0.0);
                s.atmosphere.scale_height =
                    av.value("scale_height", 0.0);
                // The hard top (above it: vacuum). 0 => top() derives
                // scale_height * 10.
                s.atmosphere.height = av.value("height", 0.0);
                // Air needs both density halves (or neither: a render-only
                // rim). One without the other is authored nonsense.
                if((s.atmosphere.sea_level_density > 0.0)
                   != (s.atmosphere.scale_height > 0.0)) {
                    throw std::runtime_error(
                        "system: " + body->name + ": atmosphere needs both "
                        "sea_level_density and scale_height (or neither)");
                }
            }
            if(sv.contains("clouds") && sv["clouds"].is_object()) {
                const nlohmann::json &cv = sv["clouds"];
                s.clouds.enabled = true;
                if(cv.contains("color") && cv["color"].is_array()
                   && cv["color"].size() >= 3) {
                    const nlohmann::json &c = cv["color"];
                    s.clouds.color = glm::vec3(c[0].get<float>(),
                                               c[1].get<float>(),
                                               c[2].get<float>());
                }
                s.clouds.height = cv.value("height", 2500.0f);
                s.clouds.coverage = cv.value("coverage", 0.6f);
                s.clouds.freq = cv.value("freq", 10.0f);
                s.clouds.drift = cv.value("drift", 0.0f);
            }
            if(sv.contains("rings") && sv["rings"].is_array()) {
                for(const nlohmann::json &rv : sv["rings"]) {
                    if(!rv.is_object()) {
                        continue;
                    }
                    RingParams rp;
                    rp.name = rv.value("name", std::string(""));
                    rp.inner = rv.value("inner", 0.0);
                    rp.outer = rv.value("outer", 0.0);
                    rp.thickness = rv.value("thickness", 0.0);
                    rp.albedo = rv.value("albedo", 0.5f);
                    rp.opacity = rv.value("opacity", 0.8f);
                    // skip malformed bands (outer must exceed inner)
                    if(rp.outer > rp.inner) {
                        s.rings.push_back(rp);
                    }
                }
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

        // max_height + root terrain are the heavy phase (deferred to the
        // worker so the title can appear first).

        // Shader + elevation palette by body type.
        switch(body->type) {
        case BodyType::Star:
            body->shader = sunshader;
            body->colour_func = GetColourSun;
            break;
        case BodyType::Moon:
            body->shader = terrainshader;
            body->colour_func = GetColourMoon;
            break;
        case BodyType::Planet:
            body->shader = terrainshader;
            body->colour_func = GetColourEarth;
            break;
        }

        // --- inertial (non-rotating) frame ---------------------------------
        Frame *f = new Frame;
        f->name  = body->name + " (inertial)";
        f->body  = body;
        f->parent = nullptr;                 // wired in pass 2
        f->children.clear();
        f->rotating = false;
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
            // Optional orbital plane orientation (radians): orient =
            // R_Y(-raan) * R_X(i) maps the local orbital plane into the
            // parent frame.
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

        // --- rotating (near-body) frame -------------------------------------
        // Every body gets one. Its SOI is DERIVED (bodylimits.h shellEdge):
        // the atmosphere top plus the low-orbit band, floored at kMinShell,
        // so the air always fits inside the frame's enter band. Authored
        // rotating.soi is ignored. No "rotating" JSON section (e.g. the
        // star) => a DUMMY frame: zero spin, same derived SOI, so scenario
        // radii and frame switching work uniformly.
        const double shell = shellEdge(s.atmosphere.top());
        Frame *rf = new Frame;
        rf->name  = body->name + " (rotational)";
        rf->body  = body;
        rf->parent = f;                    // child of its own inertial frame
        rf->children.clear();
        rf->rotating = true;
        rf->pos = glm::dvec3(0, 0, 0);
        rf->initial_orient = glm::dmat3(1.0);
        rf->orient = glm::dmat3(1.0);
        rf->vel = glm::dvec3(0);
        rf->orb_ang_speed = 0;
        rf->spin_axis = glm::dvec3(0, 1, 0);   // no axial tilt by default
        rf->root_pos = glm::dvec3(0);
        rf->root_vel = glm::dvec3(0);
        rf->root_orient = glm::dmat3(1.0);
        rf->soi = radius + (double)s.sea_level + shell;
        if(bv.contains("rotating") && bv["rotating"].is_object()) {
            const nlohmann::json &rot = bv["rotating"];
            rf->rot_ang_speed = rot.value("rot_ang_speed", 0.0);
            // Optional axial tilt (radians): lean the pole away from the
            // orbital normal toward +X, folded into initial_orient. The spin
            // stays about +Y (the figure axis) so the pole IS the spin axis --
            // tilting spin_axis instead makes the terrain pole/bands/rings
            // precess once per rotation.
            const double axial_tilt = rot.value("axial_tilt", 0.0);
            if(axial_tilt != 0.0) {
                const double ct = std::cos(axial_tilt), st = std::sin(axial_tilt);
                rf->initial_orient = glm::dmat3(
                    glm::dvec3(ct, -st, 0.0),
                    glm::dvec3(st,  ct, 0.0),
                    glm::dvec3(0.0, 0.0, 1.0));
            }
        } else {
            rf->rot_ang_speed = 0.0;        // dummy: does not spin
        }
        body->rot_frame = rf;
        f->rot_frame = rf;
        f->children.push_back(rf);

        // Heavy phase is NOT built here (deferred; see postHeavyPhase).

        body->refreshParamsCache();   // surface/radius/colour_func are final
        sys.bodies.push_back(body);

        // Per-body progress so the caller can draw a "loading..." frame.
        if(progress) { progress(i, bodies.size(), body->name); }
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

            Frame *f = body->frame;
            const double mu = parent->mu;
            const double w = f->orb_ang_speed;
            // Semi-major axis from the mean angular rate (Kepler III).
            const double a = (w != 0.0) ? cbrt(mu / (w * w)) : 0.0;

            // DERIVED inertial SOI (bodylimits.h): the system's law value,
            // lifted clear of the near-body shell's hysteresis band AND wide
            // enough to contain the inertial-orbit spawn bed. Authored
            // inertial.soi is ignored for non-root bodies; the root keeps its
            // authored universe bound (pass 1).
            f->soi = inertialSoi(soiByLaw(law, a, body->mass, parent->mass),
                                 body->rot_frame->soi,
                                 shellEdge(body->surface.atmosphere.top()));

            // Epoch orbital state for the Kepler rail. e, arg_peri and the
            // epoch true anomaly default to the circular orbit through pos.
            if(w != 0.0) {
                const nlohmann::json &in =
                    bv.value("inertial", nlohmann::json::object());
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

    // The ordering the game needs (bodylimits.h): air inside the shell's
    // enter band, the shell enterable, the inertial SOI clear of the
    // shell's hysteresis and containing the inertial-orbit spawn bed.
    // Derived values cannot fail these; the check is the tripwire for
    // future edits.
    for(TerrainBody *b : sys.bodies) {
        validateBodyLimits(b->name, b->radius, b->surface.sea_level,
                           b->surface.atmosphere.top(),
                           b->rot_frame->soi, b->frame->soi);
    }

    // --- resolve the home planet (the calendar + default spawn body) ------
    if(!home_name.empty()) {
        sys.home = sys.find(home_name);
        if(sys.home == nullptr) {
            throw std::runtime_error("system: home body '" + home_name
                                     + "' not found");
        }
    } else if(sys.bodies.size() > 1) {
        // No explicit home: default to the first non-star body.
        for(size_t i = 0; i < sys.bodies.size(); i++) {
            if(sys.bodies[i] != sys.root) { sys.home = sys.bodies[i]; break; }
        }
    }

    // --- calendars ----------------------------------------------------------
    // Per-body calendar from its spin (day) and orbit (year) rates. The year
    // snaps to whole days (calendar.h) so boundaries fall on local midnight.
    // Stars get an invalid calendar (dummy zero-spin frame).
    const int epoch_year = 4724;  // the year the game starts in
    for(size_t i = 0; i < sys.bodies.size(); i++) {
        TerrainBody *b = sys.bodies[i];
        const double D = (b->rot_frame && b->rot_frame->rot_ang_speed > 0.0)
                       ? 2.0 * std::numbers::pi / b->rot_frame->rot_ang_speed : 0.0;
        const double Y = (b->frame && b->frame->orb_ang_speed > 0.0)
                       ? 2.0 * std::numbers::pi / b->frame->orb_ang_speed : 0.0;
        b->cal = Calendar::make(D, Y, epoch_year);
    }

    // Recompute root-relative frame values before the first render.
    sys.root->frame->UpdateOrbitRails(0.0);

    printf("Loaded system '%s': %zu bodies (home=%s)\n",
           path, sys.bodies.size(),
           sys.home ? sys.home->name.c_str() : "(none)");

    cleanup.commit = true;   // build complete: the caller now owns the bodies
    return sys;
}
