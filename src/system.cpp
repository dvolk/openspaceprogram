// system.cpp -- the JSON star-system loader (see system.h).
#include "system.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <numbers>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>  // glm::rotate (updateChain)

#include "equirect.h"  // equirectLonLat (subsolar surface longitude)
#include "orbit.h"  // railStateFromElements
#include "resdir.h"
#include "bodylimits.h"  // the derived shell + SOI laws + the ordering asserts

namespace {
// Advance just the parent chain of `f` to time t (a single-node version of
// UpdateOrbitRails). Measuring every body's solar day must not walk the
// whole forest per sample.
void updateChain(Frame *f, double t) {
    if(f == nullptr) { return; }
    updateChain(f->parent, t);
    if(f->parent != nullptr && !f->rotating) {
        if(f->orb_ang_speed != 0) {
            propagateKepler(f->orbit_pos0, f->orbit_vel0, f->parent_mu,
                            t, f->pos, f->vel);
        }
        f->root_pos = f->parent->root_orient * f->orient * f->pos
                    + f->parent->root_pos;
        f->root_vel = f->parent->root_orient * f->orient * f->vel
                    + f->parent->root_vel;
        f->root_orient = f->parent->root_orient * f->orient;
    } else if(f->rotating) {
        const double ang = std::fmod(f->rot_ang_speed * t, 2.0 * std::numbers::pi);
        f->orient = f->initial_orient * glm::dmat3(glm::rotate(ang, f->spin_axis));
        if(f->parent != nullptr) {
            f->root_pos = f->parent->root_orient * f->orient * f->pos
                        + f->parent->root_pos;
            f->root_vel = f->parent->root_orient * f->orient * f->vel
                        + f->parent->root_vel;
            f->root_orient = f->parent->root_orient * f->orient;
        }
    } else if(f->parent == nullptr) {
        // root: root_pos/root_orient stay at their load-time values
    }
    f->rail_time = t;
}

// Direction TOWARD the star in `body`'s rotating frame (same math as
// surfmap.cpp:sunDirRot). False when there is no terminator.
bool subsolarLonAt(TerrainBody *body, TerrainBody *sun,
                   double t, double &lon) {
    if(sun == nullptr || body == sun || body->frame == nullptr
       || sun->frame == nullptr || body->frame->getRotFrame() == nullptr) {
        return false;
    }
    updateChain(body->frame->getRotFrame(), t);
    updateChain(sun->frame, t);
    const glm::dvec3 to_sun = sun->frame->root_pos - body->frame->root_pos;
    const double l = glm::length(to_sun);
    if(l < 1e-9) { return false; }
    const glm::dmat3 to_rot =
        glm::transpose(body->frame->getRotFrame()->root_orient);
    const glm::dvec3 dir = to_rot * (to_sun / l);
    double lat = 0.0;
    equirectLonLat(dir, lon, lat);
    return true;
}

// Wrap to (-pi, pi].
double wrapPi(double a) {
    const double pi = std::numbers::pi;
    a = std::fmod(a + pi, 2.0 * pi);
    if(a < 0.0) { a += 2.0 * pi; }
    return a - pi;
}

} // namespace

double measureSolarDay(Frame *root, TerrainBody *body, TerrainBody *sun) {
    const double twopi = 2.0 * std::numbers::pi;
    const double sidereal =
        (body && body->rot_frame && body->rot_frame->rot_ang_speed > 0.0)
        ? twopi / body->rot_frame->rot_ang_speed
        : 0.0;
    if(sidereal <= 0.0 || body->frame == nullptr) { return 0.0; }

    // Probe over the HELIOCENTRIC year (the frame that directly orbits the
    // star), not the body's own orbital period. A moon's local orbit does
    // not close the sun: it inherits the parent's true-anomaly rate and
    // the answer wobbles with epoch (Earth e=0.0167 is +-0.3% over a
    // sidereal month). One solar year averages that out. Not the apparent
    // first-return at t=0 either (that is 86429 s on Jan 1, walking
    // through the equation of time).
    Frame *helio = body->frame;
    while(helio->parent != nullptr && helio->parent->parent != nullptr) {
        helio = helio->parent;
    }
    double t_probe = sidereal;   // no solar orbit: the sun is fixed
    if(helio->orb_ang_speed > 0.0) {
        t_probe = twopi / helio->orb_ang_speed;
    }

    double lon0 = 0.0;
    if(!subsolarLonAt(body, sun, 0.0, lon0)) {
        return sidereal;   // no sun: solar day == sidereal
    }

    // Step finely enough that the spin alone cannot alias across pi
    // (64 samples minimum; 8192 cap keeps a 12-y Jupiter year cheap).
    const int n = (int)std::min(8192.0,
                     std::max(64.0, std::ceil(8.0 * t_probe / sidereal)));
    double lon_prev = lon0;
    double sweep = 0.0;
    for(int i = 1; i <= n; i++) {
        double lon = 0.0;
        const double t = t_probe * (double)i / (double)n;
        if(!subsolarLonAt(body, sun, t, lon)) { return sidereal; }
        sweep += wrapPi(lon - lon_prev);
        lon_prev = lon;
    }

    // |sweep| = 2pi * (solar days per probe). Tiny sweep = the body is
    // locked to the sun (no day); fall back so the clock still runs.
    if(std::fabs(sweep) < 1e-6) { return sidereal; }
    return t_probe * twopi / std::fabs(sweep);
}

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

    // --- the star field (JSON "skybox") -----------------------------------
    // A DIRECTORY holding skybox_px/nx/py/ny/pz/nz.png. The loader builds the
    // six names itself, in GL cubemap order, so the data cannot reorder them:
    // the only way to end up with a mirrored sky is to put the wrong image at
    // the wrong name, and the alignment pins in utils/skybox/skybox_common.py
    // catch that (test-py runs them against the committed set). The sky
    // belongs to the system, so switching systems can change it
    // (Game::switchSystem re-loads these faces).
    static constexpr const char *kFaceSuffs[6]
        = { "px", "nx", "py", "ny", "pz", "nz" };
    if(!doc.contains("skybox")) {
        throw std::runtime_error(std::string("system: no \"skybox\" in ") + path
                + " -- name a directory holding skybox_px/nx/py/ny/pz/nz.png, "
                "e.g. \"res/skybox/v1\"");
    }
    const nlohmann::json &sb = doc["skybox"];
    if(!sb.is_string() || sb.get<std::string>().empty()) {
        throw std::runtime_error(std::string("system: \"skybox\" must be a "
                "directory name, e.g. \"res/skybox/v1\""));
    }
    std::string dir = sb.get<std::string>();
    if(dir.back() == '/') { dir.pop_back(); }
    // Resolve with resdir::path here: it is what Skybox::load opens with, so
    // the check and the open agree -- which also means a directory outside
    // "res/" (a staging/ bake under test) is cwd-relative.
    for(const char *suff : kFaceSuffs) {
        std::string face = dir + "/skybox_" + suff + ".png";
        std::error_code ec;
        if(!std::filesystem::exists(resdir::path(face), ec)) {
            throw std::runtime_error("system: \"skybox\" face '" + face
                                     + "' does not exist");
        }
        sys.skybox_faces.push_back(face);
    }

    // --- debris belts (JSON "belts") --------------------------------------
    // Optional array of named annuli orbiting the star, drawn on the orbital
    // maps. Same entry shape as a body's "surface.rings" band. Strict unlike
    // rings (which skip malformed bands): these are few, hand-authored, and a
    // dropped belt is invisible in-game, so a typo must fail the load.
    if(doc.contains("belts")) {
        const nlohmann::json &bl = doc["belts"];
        if(!bl.is_array()) {
            throw std::runtime_error(std::string("system: \"belts\" in ") + path
                    + " must be an array of {\"name\", \"inner\", \"outer\"}");
        }
        for(const nlohmann::json &bv : bl) {
            if(!bv.is_object() || !bv.contains("name")
               || !bv["name"].is_string() || bv["name"].get<std::string>().empty()) {
                throw std::runtime_error(std::string("system: \"belts\" entry in ")
                        + path + " needs a non-empty \"name\"");
            }
            BeltParams bp;
            bp.name = bv["name"].get<std::string>();
            if(!bv.contains("inner") || !bv["inner"].is_number()
               || !bv.contains("outer") || !bv["outer"].is_number()) {
                throw std::runtime_error("system: belt '" + bp.name
                        + "' needs numeric \"inner\" and \"outer\" [m]");
            }
            bp.inner = bv["inner"].get<double>();
            bp.outer = bv["outer"].get<double>();
            if(bp.inner <= 0.0 || bp.outer <= bp.inner) {
                throw std::runtime_error("system: belt '" + bp.name + "' needs"
                        " 0 < inner < outer (got " + std::to_string(bp.inner)
                        + ", " + std::to_string(bp.outer) + ")");
            }
            sys.belts.push_back(bp);
        }
    }

    // Rail longitude a -> R_Y(+a): maps local +X to longitude +a, measured as
    // atan2(-z, x) about the parent inertial frame's +Y. That is the ecliptic
    // longitude of the sky embedding (rail = (x_ecl, z_ecl, -y_ecl), a proper
    // rotation -- make_skybox.py), and prograde motion sweeps it UP. Authored
    // angles therefore go in un-negated and read like the fact sheets (#146).
    auto railAz = [](double a) {
        const double c = std::cos(a), s = std::sin(a);
        return glm::dmat3(glm::dvec3(c, 0.0, -s),
                          glm::dvec3(0.0, 1.0, 0.0),
                          glm::dvec3(s, 0.0, c));
    };

    // railAz(raan) * R_X(incl): the orbital-plane orientation from the
    // authored pair -- frame +X on the ascending node at ecliptic longitude
    // raan, frame +Y the orbit normal at (raan - 90 deg, 90 deg - incl).
    // Used by pass 1 (orbit-referred) and pass 2 (equator-referred, #147) --
    // one spelling, no duplicated literal.
    auto planeOrient = [&railAz](double incl, double raan) {
        const double ci = std::cos(incl), si = std::sin(incl);
        return railAz(raan) * glm::dmat3(glm::dvec3(1.0, 0.0, 0.0),
                                         glm::dvec3(0.0, ci, si),
                                         glm::dvec3(0.0, -si, ci));
    };

    // --- pass 1: create every body and its frames --------------------------
    for(size_t i = 0; i < bodies.size(); i++) {
        const nlohmann::json &bv = bodies[i];

        TerrainBody *body = new TerrainBody;
        body->frame = nullptr;
        body->rot_frame = nullptr;
        // Register before any throw below: BodyCleanup deletes these bodies,
        // and ~TerrainBody frees frame/rot_frame, so a mid-build throw (data
        // bugs like the rate checks) leaks nothing. main.cpp keeps running
        // after a failed load, so a leak here would persist in the session.
        sys.bodies.push_back(body);

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
        // science_mult is required and hand-editable (utils/sci_dist.py
        // writes it). Missing / non-finite / non-positive is a data bug:
        // scoring it as 1.0 would silently under-value the body.
        if(!bv.contains("science_mult") || !bv["science_mult"].is_number()) {
            throw std::runtime_error("system: '" + body->name
                                     + "': missing numeric \"science_mult\"");
        }
        body->science_mult = bv["science_mult"].get<double>();
        if(!(body->science_mult > 0.0) || !std::isfinite(body->science_mult)) {
            throw std::runtime_error("system: '" + body->name
                                     + "': science_mult must be finite and > 0");
        }
        body->transfer_dv = bv.value("transfer_dv", 0.0);
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
            // How the sea renders (terragen.h OceanMode). Absent = "mesh".
            // has_sea / sea_level are resolved above, so a body that names a
            // renderer but has no sea is a data bug like a bad mode is: the
            // key would do nothing at all.
            if(sv.contains("ocean")) {
                if(!sv["ocean"].is_string()) {
                    throw std::runtime_error("system: '" + body->name
                                             + "': \"ocean\" must be a string");
                }
                if(!s.has_sea) {
                    throw std::runtime_error("system: '" + body->name
                                             + "': \"ocean\" needs a sea"
                                             " (\"has_sea\" or"
                                             " \"surface.sea_level\")");
                }
                s.ocean_mode = parseOceanMode(sv["ocean"].get<std::string>(),
                                              body->name);
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
            // Optional authored macro relief (terragen.h Heightmap). The
            // path is game-asset style ("res/..." or a cwd-relative staging
            // bake); a missing/bad file is a data bug, not a silent fallback.
            if(sv.contains("heightmap")) {
                if(!sv["heightmap"].is_string()
                   || sv["heightmap"].get<std::string>().empty()) {
                    throw std::runtime_error("system: '" + body->name
                                             + "': \"heightmap\" must be a "
                                               "file path");
                }
                s.heightmap = loadHeightmap(
                    resdir::path(sv["heightmap"].get<std::string>()));
            }
            s.detail_amplitude = sv.value("detail_amplitude", s.detail_amplitude);
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
        body->frame = f;                   // owned by body from here on
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
            // Rates are magnitudes; the ORBITAL sense lives in orb_incl
            // (> 90 deg flips the plane normal, and the rail's local prograde
            // becomes parent-frame retrograde -- the source-data convention,
            // see make_solar_system.py and test_retrograde). A negative rate
            // would double-encode (and silently load prograde: a = cbrt(mu/w^2)
            // drops the sign), so reject it like other data bugs (issue #139).
            if(f->orb_ang_speed < 0.0) {
                throw std::runtime_error("system: '" + body->name
                        + "' has negative orb_ang_speed; encode retrograde "
                        "orbits with orb_incl > pi/2, not a negative rate");
            }
            // Optional orbital plane orientation (radians): orient =
            // R_Y(+raan) * R_X(i) maps the local orbital plane into the
            // parent frame.
            const double orb_incl = in.value("orb_incl", 0.0);
            const double lon_asc_node = in.value("lon_asc_node", 0.0);
            // #147: what orb_incl/lon_asc_node are REFERRED to. "orbit"
            // (default): the parent's orbital plane, composed here.
            // "equator": the parent's equator (the fact sheets' convention
            // for regular moons) -- composed in pass 2, where the parent's
            // rotating frame (its axial tilt) is resolved. The sheets'
            // second plane is really the ECLIPTIC; "orbit" matches it for
            // planet parents (their rails are ecliptic-embedded, <= 2.5 deg
            // off), and no body needs a distinct value yet.
            const std::string incl_ref =
                in.value("incl_ref", std::string("orbit"));
            if(incl_ref != "orbit" && incl_ref != "equator") {
                throw std::runtime_error("system: '" + body->name
                        + "': inertial.incl_ref must be \"orbit\" or "
                        "\"equator\", got \"" + incl_ref + "\"");
            }
            if(incl_ref != "orbit" && !bv.contains("orbits")) {
                throw std::runtime_error("system: '" + body->name
                        + "': inertial.incl_ref needs a parent (no "
                        "\"orbits\" field)");
            }
            if(incl_ref == "orbit") {
                f->orient = planeOrient(orb_incl, lon_asc_node);
            }
        }

        // --- rotating (near-body) frame -------------------------------------
        // Every body gets one. Its SOI is DERIVED (bodylimits.h shellEdge):
        // the atmosphere top plus the low-orbit band, floored at kMinShell,
        // so the air always fits inside the frame's enter band. Authored
        // rotating.soi is ignored. No "rotating" JSON section (e.g. the
        // star) => a DUMMY frame: zero spin, same derived SOI, so scenario
        // radii and frame switching work uniformly.
        const double shell = shellEdge(s.atmosphere.top());
        Frame *rf = new Frame;
        body->rot_frame = rf;              // owned by body from here on
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
            // Same policy as orb_ang_speed above: the SPIN sense lives in
            // axial_tilt (> 90 deg; the fact sheets' negative rotation
            // periods are abs()'d by the generator, make_solar_system.py:174).
            // A negative rate would double-flip against the tilt and would
            // invalidate the calendar (D -> 0 gate), so reject it (#139).
            if(rf->rot_ang_speed < 0.0) {
                throw std::runtime_error("system: '" + body->name
                        + "' has negative rot_ang_speed; encode retrograde "
                        "spin with axial_tilt > pi/2, not a negative rate");
            }
            // Optional axial tilt (radians): lean the pole away from the
            // orbital normal toward +X, folded into initial_orient. The spin
            // stays about +Y (the figure axis) so the pole IS the spin axis --
            // tilting spin_axis instead makes the terrain pole/bands/rings
            // precess once per rotation.
            const double axial_tilt = rot.value("axial_tilt", 0.0);
            // #141: rail-longitude angles, same convention as lon_asc_node
            // (railAz above: the named direction lands at longitude +a =
            // atan2(-z, x) about the parent inertial frame's +Y, measured
            // from +X, i.e. the orbit's node line). tilt_azimuth = the
            // direction the pole leans (default 0 = toward +X, the ascending
            // node -- the old permanent node-lock). spin_phase0 = the
            // epoch spin angle about the figure axis +Y -- for an untilted
            // body that is the rail longitude of the longitude-0 point (rot
            // frame +X = surface lon 0, equirect.h; the spin analogue of
            // true_anomaly0), but for tilted bodies the longitude reading
            // degrades past ~90 deg tilt; the figure-axis angle is the
            // sound meaning. Free-form angles: negative is legal, like
            // lon_asc_node. Systems that OMIT them load byte-identically
            // to pre-#141.
            const double tilt_az = rot.value("tilt_azimuth", 0.0);
            const double phase0 = rot.value("spin_phase0", 0.0);
            if(axial_tilt != 0.0 || tilt_az != 0.0 || phase0 != 0.0) {
                const double ct = std::cos(axial_tilt), st = std::sin(axial_tilt);
                glm::dmat3 m(glm::dvec3(ct, -st, 0.0),
                             glm::dvec3(st,  ct, 0.0),
                             glm::dvec3(0.0, 0.0, 1.0));
                if(tilt_az != 0.0) {
                    m = railAz(tilt_az) * m;
                }
                // #147: the tilt part alone (no spin-phase pre-rotation)
                // is the body's equator frame; equator-referred child
                // rails hang under it (pass 2), NOT under initial_orient.
                rf->equator_orient = m;
                if(phase0 != 0.0) {
                    m = m * railAz(phase0);
                }
                rf->initial_orient = m;
            }
        } else {
            rf->rot_ang_speed = 0.0;        // dummy: does not spin
        }
        f->rot_frame = rf;
        f->children.push_back(rf);

        // Heavy phase is NOT built here (deferred; see postHeavyPhase).

        body->refreshParamsCache();   // surface/radius/colour_func are final

        // Per-body progress so the caller can draw a "loading..." frame.
        if(progress) { progress(i, bodies.size(), body->name); }
    }

    // --- pass 2: wire the parent/child frame tree --------------------------
    // The root's authored universe bound (inertial.soi), if any: the tripwire
    // below rejects orbits that do not fit inside it. 0 = not authored ->
    // containment check skipped (issue #140).
    double root_soi_bound = 0.0;
    for(const nlohmann::json &bv : bodies) {
        if(bv.value("orbits", std::string("")).empty()) {
            root_soi_bound = bv.value("inertial", nlohmann::json::object())
                                 .value("soi", 0.0);
        }
    }
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
            // A self-orbit would make the body its own child frame and turn
            // any tree walk (the Research Lab's System Atlas) into infinite
            // recursion -- a data bug, so reject it like the unknown-parent
            // case above. (A mutual A<->B cycle is still possible; see the
            // follow-up issue on full acyclicity.)
            if(parent == body) {
                throw std::runtime_error("system: '" + body->name
                                         + "' cannot orbit itself");
            }
            body->frame->parent = parent->frame;
            parent->frame->children.push_back(body->frame);

            // #147: equator-referred moon rails. orient = parent's EQUATOR
            // frame * R_Y(+raan) * R_X(i): the moon's plane tips WITH the
            // parent's axis (Charon rides Pluto's equator, ~54 deg off
            // Pluto's orbital plane; the giant planets' regular moons sit
            // within a few degrees of their equators). The parent's
            // equator_orient (tilt only, no spin phase) -- composing under
            // full initial_orient would node-lock the moon's rail to the
            // parent's prime meridian. Orientation only: the frame
            // hierarchy stays inertial-to-inertial, so positions, rails
            // and SOIs are untouched.
            {
                const nlohmann::json &ine =
                    bv.value("inertial", nlohmann::json::object());
                if(ine.value("incl_ref", std::string("orbit")) == "equator") {
                    // Pass 1 gives every body a rot frame; pass 1 fully
                    // composed equator_orient before pass 2 reads it, so
                    // bodies-array order does not matter.
                    assert(parent->rot_frame && parent->rot_frame->rotating);
                    body->frame->orient =
                        parent->rot_frame->equator_orient
                        * planeOrient(ine.value("orb_incl", 0.0),
                                      ine.value("lon_asc_node", 0.0));
                }
            }

            Frame *f = body->frame;
            const double mu = parent->mu;
            const double w = f->orb_ang_speed;
            // Semi-major axis from the mean angular rate (Kepler III).
            const double a = (w != 0.0) ? cbrt(mu / (w * w)) : 0.0;
            // #140 tripwire (the magnitude twin of the rate-sign guard): a
            // tiny w*w underflows and a becomes inf (NaN rails), or stays
            // finite but absurd, parking the body beyond the universe bound
            // forever. railStateFromElements only rejects a <= 0.
            if(w != 0.0 && !std::isfinite(a)) {
                throw std::runtime_error("system: '" + body->name
                        + "': orb_ang_speed too small to place an orbit "
                        "(semi-major axis overflows to infinity)");
            }
            if(w != 0.0 && root_soi_bound > 0.0 && a > root_soi_bound) {
                char ab[32], sb[32];
                std::snprintf(ab, sizeof ab, "%g", a);
                std::snprintf(sb, sizeof sb, "%g", root_soi_bound);
                throw std::runtime_error("system: '" + body->name
                        + "': orbit (semi-major axis " + ab + " m) exceeds "
                        "the universe bound " + sb + " m; orb_ang_speed is "
                        "too small");
            }

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
                    : atan2(-f->pos.z, f->pos.x) - arg_peri;
                // The pos fallback keeps only the DIRECTION: the rail puts the
                // body at the conic's own radius, so an eccentric orbit
                // authored by position would silently lose that radius (#175).
                // A data bug, so it throws like every other guard here rather
                // than asserting (an assert also vanishes under NDEBUG).
                if(!in.contains("true_anomaly0") && e != 0.0) {
                    throw std::runtime_error(
                        "system: '" + body->name + "' authors nonzero ecc with no"
                        " true_anomaly0: the epoch falls back to the DIRECTION of"
                        " inertial.pos (zero when pos is omitted), so the radius"
                        " is not the authored one -- author true_anomaly0");
                }
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
    // Per-body calendar from a SOLAR day (#201: the geometric mean of the
    // subsolar surface longitude over a heliocentric year, NOT the sidereal
    // 2*pi/spin -- that drifts 3m56s/solar-day against the sun on Earth and
    // is simply wrong for Venus / Triton / the tidally locked Moon) and the
    // orbital period (year).
    // The year snaps to whole days (calendar.h) so boundaries fall on local
    // midnight. Stars get an invalid calendar (dummy zero-spin frame).
    // The displayed year at t == 0. Solar systems author 2000: their orbital
    // phases are J2000-referenced (fact sheets), so the clock matches the
    // sky. Default 1 (the fictional systems start there, like the game they
    // evoke); the old game-wide 4724 predates this field and meant nothing.
    const int epoch_year = doc.value("epoch_year", 1);
    // #202: optional proleptic-Gregorian civil calendar. The system's t=0
    // is this civil epoch (the solar-system data is already 2000-01-01
    // 00:00 UT). The CLOCK runs a fixed 86400 s civil day; the measured
    // solar day stays on solar_day_seconds for the body panel. Seasonal
    // drift (sidereal year 365.256 d vs Gregorian 365.2425) is documented
    // in calendar.h -- year_seconds is the seasonal handle.
    long civil_epoch_days = 0;
    bool use_civil = false;
    if(doc.contains("epoch_utc") && doc["epoch_utc"].is_string()) {
        const std::string s = doc["epoch_utc"].get<std::string>();
        int y = 0, mo = 1, d = 1;
        if(std::sscanf(s.c_str(), "%d-%d-%d", &y, &mo, &d) == 3) {
            civil_epoch_days = days_from_civil(y, (unsigned)mo, (unsigned)d);
            use_civil = true;
        } else {
            throw std::runtime_error(
                std::string("system: bad epoch_utc '") + s
                + "' (want YYYY-MM-DD or YYYY-MM-DDTHH:MM:SSZ)");
        }
    }
    // The measure walks the rail tree; every body shares one sun (the star).
    for(size_t i = 0; i < sys.bodies.size(); i++) {
        TerrainBody *b = sys.bodies[i];
        const double D = measureSolarDay(sys.root->frame, b, sys.root);
        const double Y = (b->frame && b->frame->orb_ang_speed > 0.0)
                       ? 2.0 * std::numbers::pi / b->frame->orb_ang_speed : 0.0;
        b->cal = use_civil ? Calendar::makeCivil(civil_epoch_days, Y, D)
                           : Calendar::make(D, Y, epoch_year);
    }

    // Recompute root-relative frame values before the first render.
    sys.root->frame->UpdateOrbitRails(0.0);

    printf("Loaded system '%s': %zu bodies (home=%s, belts=%zu)\n",
           path, sys.bodies.size(),
           sys.home ? sys.home->name.c_str() : "(none)", sys.belts.size());

    cleanup.commit = true;   // build complete: the caller now owns the bodies
    return sys;
}
