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

#include "equirect.h"  // the (lon, lat) <-> direction convention
#include "insolation.h"  // starFromBody / subsolarLonLat: one star direction (#724)
#include "constants.h"  // kAirMolarMass / kAirGamma (the atmosphere defaults)
#include "orbit.h"  // railStateFromElements
#include "resdir.h"
#include "bodylimits.h"  // the derived shell + SOI laws + the ordering asserts
#include "solar.h"  // starLuminosity: the star's output, from its authored T (#687)
#include "vehicle.h"  // dragAtm: the one place a body's air becomes a profile (#687 audit)
#include "drag.h"     // airTemperature: the law for the air's own datum temperature

namespace {
// Advance just the parent chain of `f` to time t (single-node
// UpdateOrbitRails): the solar-day measure must not walk the whole forest.
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

// Direction TOWARD the star in `body`'s rotating frame, at time t. False when there is
// no terminator. The frame arithmetic lives in insolation.h; what this adds is the
// advance, which is the only reason it is not the header's function verbatim: the
// calendar asks what the frames hold at a time, while the map bakes whatever they hold.
// (getRotFrame() is not nullable -- it falls back to the frame itself -- so the only
// pointers guarded here are the two frames themselves.)
bool subsolarLonAt(TerrainBody *body, TerrainBody *sun,
                   double t, double &lon) {
    if(sun == nullptr || body == sun || body->frame == nullptr
       || sun->frame == nullptr) {
        return false;
    }
    updateChain(body->frame->getRotFrame(), t);
    updateChain(sun->frame, t);
    const StarFromBody sv = starFromBody(body->frame->root_pos,
                                         sun->frame->root_pos,
                                         body->frame->getRotFrame()->root_orient);
    if(!sv.valid) { return false; }
    double lat = 0.0;
    subsolarLonLat(sv, lon, lat);
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
    // star), not the body's own period: a moon's local orbit does not close the
    // sun, so the answer wobbles with epoch. Not the first-return at t=0 either
    // (that walks the equation of time).
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

    // Fine enough that the spin alone cannot alias across pi; the cap keeps a
    // long year cheap.
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

    // |sweep| = 2pi * solar days per probe. A tiny sweep means the body is
    // locked to the sun: fall back to the sidereal day.
    if(std::fabs(sweep) < 1e-6) { return sidereal; }
    return t_probe * twopi / std::fabs(sweep);
}

System load_system(const char *path, Shader *terrainshader, Shader *sunshader,
                   std::function<void(size_t i, size_t total,
                                      const std::string &name)> progress) {
    // `path` is the logical name for logs/e2e; only the open sees the resolved
    // path (resdir.h).
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
    // patched_conic; solar_system*.json author "hill".
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

    // --- the star field (JSON "skybox": a directory of skybox_px/...png) ---
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
    // resdir::path is what Skybox::load opens with, so check and open agree.
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
    // Strict unlike rings: a dropped belt is invisible in-game, so a typo must
    // fail the load.
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

    // --- world buildings (JSON "world_buildings" + "world_buildings_files") --
    // Instances only; place_world_buildings (buildings.h) makes them pads.
    sys.world_buildings = parse_world_buildings_all(doc, path);

    // Rail longitude a -> R_Y(+a): local +X at longitude +a, measured as
    // atan2(-z, x) about the parent inertial +Y -- the sky embedding's ecliptic
    // longitude (rail = (x_ecl, z_ecl, -y_ecl), a proper rotation). Prograde
    // motion sweeps it UP, so authored angles go in un-negated (#146).
    auto railAz = [](double a) {
        const double c = std::cos(a), s = std::sin(a);
        return glm::dmat3(glm::dvec3(c, 0.0, -s),
                          glm::dvec3(0.0, 1.0, 0.0),
                          glm::dvec3(s, 0.0, c));
    };

    // railAz(raan) * R_X(incl): the orbital plane from the authored pair --
    // frame +X on the ascending node at ecliptic longitude raan, frame +Y the
    // orbit normal at (raan - 90 deg, 90 deg - incl) (#147).
    auto planeOrient = [&railAz](double incl, double raan) {
        const double ci = std::cos(incl), si = std::sin(incl);
        return railAz(raan) * glm::dmat3(glm::dvec3(1.0, 0.0, 0.0),
                                         glm::dvec3(0.0, ci, si),
                                         glm::dvec3(0.0, -si, ci));
    };

    // --- pass 1: create every body and its frames --------------------------
    for(size_t i = 0; i < bodies.size(); i++) {
        const nlohmann::json &bv = bodies[i];

        // Member initializers cover every owner pointer, so the body is
        // destructible from here on with nothing set yet (#227).
        TerrainBody *body = new TerrainBody;
        // Register before any throw below, so BodyCleanup + ~TerrainBody can
        // leak nothing on a mid-build throw.
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
        // Required: defaulting it to 1.0 would silently under-value the body.
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
            /* #687: the surface's thermal numbers, authored only where a source
               states them. PRESENT-but-silly is a data bug, not a fallback: a
               negative temperature is a typo that a T^4 would hide (the star would
               light the whole system at the wrong flux), and an albedo outside
               0..1 asks a body to reflect more light than falls on it. ABSENT is
               fine and means "nobody sourced this" -- and it has its OWN spelling,
               which is not any legal value: albedo defaults to kAlbedoUnauthored and
               temperature to kThermalUnauthored (-1, authored.h #726/#729), because
               0.0 is a READING in both cases -- a body that absorbs everything, and a
               body at 0 K. So the bound here excludes the sentinel AND 0.0: a file can
               never author its own absence, and a file that means "cold" can say 40 K.
               A star authoring no temperature is not a crash and not a dark sun: it is
               guarded at the one site that would divide by it, below. The bound here is a typo filter, not physics -- 1e6 K
               admits any plausible sun and rejects a decimal-point mistake, and the
               load-time `solar` audit line below is what catches a 10x error that
               fits inside it, because that error moves the home world's flux by
               10,000x and 1361 W/m^2 is a number a reader knows. */
            if(sv.contains("temperature")) {
                if(!sv["temperature"].is_number()) {
                    throw std::runtime_error("system: '" + body->name
                                             + "': \"surface.temperature\" must"
                                             " be a number [K]");
                }
                const double tK = sv["temperature"].get<double>();
                if(!std::isfinite(tK) || tK <= 0.0 || tK > 1e6) {
                    throw std::runtime_error("system: '" + body->name
                                             + "': \"surface.temperature\"="
                                             + std::to_string(tK)
                                             + " K is not in (0, 1e6]: 0 K is an "
                                             "absence, not a temperature (omit the key)");
                }
                s.temperature = tK;
            }
            /* The JSON key is "albedo" and it is the BOND albedo (terragen.h).
               RingParams::albedo is a different thing -- a ring's visual brightness --
               and lives in a different block; the two must never be confused by a
               reader who sees `albedo` twice in one body. */
            if(sv.contains("albedo")) {
                if(!sv["albedo"].is_number()) {
                    throw std::runtime_error("system: '" + body->name
                                             + "': \"surface.albedo\" must be a"
                                             " number [0..1]");
                }
                const double a = sv["albedo"].get<double>();
                if(!std::isfinite(a) || a < 0.0 || a >= 1.0) {
                    throw std::runtime_error("system: '" + body->name
                                             + "': \"surface.albedo\"="
                                             + std::to_string(a)
                                             + " must be in [0, 1)");
                }
                s.bond_albedo = a;
            }
            /* #724 rung C: the ground's swing knob, authored only where a source states a
               night/day pair (utils/ground_model.py fits it and prints the residual). The
               range is [0,1] and the absence is a sentinel, not 1.0 -- a body that states no
               pair must not read as Venus's stated no-range (#726). */
            if(sv.contains("even_share")) {
                if(!sv["even_share"].is_number()) {
                    throw std::runtime_error("system: '" + body->name
                                             + "': \"surface.even_share\" must be a"
                                             " number [0..1]");
                }
                const double f = sv["even_share"].get<double>();
                if(!std::isfinite(f) || f < 0.0 || f > 1.0) {
                    throw std::runtime_error("system: '" + body->name
                                             + "': \"surface.even_share\"="
                                             + std::to_string(f)
                                             + " must be in [0, 1]");
                }
                s.even_share = f;
            }
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
            // How the sea renders (terragen.h OceanMode). Absent = "mesh". A
            // renderer named with no sea is a data bug: the key does nothing.
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
            // Optional authored macro relief (terragen.h Heightmap): a missing
            // or bad file is a data bug, not a silent fallback.
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
                // Physical drag (src/drag.h): both optional, 0 = no drag. A rim
                // can exist without these, and vice versa.
                /* #739: what a file AUTHORS about its air is the weight of the
                   column at the datum, in Pa. The density every law in drag.h reads
                   is resolved from it below, once the gas and this sphere's g are
                   known. Same column, opposite direction of consequence: with the
                   density authored, p0 = rho0 (R/M) T0 meant a warmer datum invented
                   weight -- the wrong sign for anything diurnal or seasonal (#724). */
                if(av.contains("sea_level_density")) {
                    throw std::runtime_error(
                        "system: " + body->name + ": atmosphere.sea_level_density is"
                        " not authored any more -- state atmosphere.sea_level_pressure"
                        " [Pa] at the datum instead (#739). On an exponential column"
                        " p0 = density * g * scale_height; beside a layer table p0 is"
                        " level 0's base_p, so density = p0 M / (R base_T).");
                }
                s.atmosphere.sea_level_pressure =
                    av.value("sea_level_pressure", 0.0);
                s.atmosphere.scale_height =
                    av.value("scale_height", 0.0);
                // The hard top (above it: vacuum). 0 => top() derives
                // scale_height * 10.
                s.atmosphere.height = av.value("height", 0.0);
                /* What the air is made of (#552 A4b, #630): "composition" as
                   [{"gas": "CO2", "fraction": 0.9532}, ...], by MOLE. The three
                   scalars the law reads -- molar_mass, gamma and oxidizer_share --
                   are what that mix RESOLVES to (atmo.h:resolveMix), so they are not
                   authorable any more: typing molar_mass beside the mix states one
                   fact twice, and the two then drift apart exactly like an authored
                   base_p would beside a temperature profile (atmo.h). A body with no
                   composition keeps dry Earth air, the default constants.h spells. */
                if(av.contains("molar_mass") || av.contains("gamma")) {
                    throw std::runtime_error(
                        "system: " + body->name + ": atmosphere.molar_mass /"
                        " atmosphere.gamma are derived from"
                        " atmosphere.composition -- a list of { gas, fraction }"
                        " (N2, O2, CO2, H2, He, CH4, H2O, Ar). Author the mix.");
                }
                /* A published standard atmosphere states its rows at GEOPOTENTIAL
                   heights; that unit is converted where such a table is written
                   (utils/atmo_units.py, called by both generators) and the marker it
                   travels with is dropped, because a system file speaks GEOMETRIC metres
                   like every other altitude in it (#690). A file still carrying the key
                   is claiming a unit this loader will not apply, and reading it
                   geometric silently is exactly the mistake the marker was meant to
                   prevent -- so refuse rather than ignore. */
                if(av.contains("heights")) {
                    throw std::runtime_error(
                        "system: " + body->name + ": atmosphere.heights is a generator"
                        " marker, not a system-file key -- convert the table to GEOMETRIC"
                        " metres (z = R Z / (R - Z), each lapse by dZ/dz across its span)"
                        " and drop the key (#690)");
                }
                if(av.contains("composition")) {
                    if(!av["composition"].is_array()) {
                        throw std::runtime_error(
                            "system: " + body->name + ": atmosphere.composition must"
                            " be an array of { gas, fraction } objects");
                    }
                    /* The whole sky is a handful of gases (Earth's dry air is four,
                       Titan's three), so a long list is a generator bug rather than
                       an atmosphere -- and it is the one cap that keeps a mix cheap
                       to read at load. */
                    if(av["composition"].size() > 16) {
                        throw std::runtime_error(
                            "system: " + body->name + ": atmosphere.composition has"
                            " too many gases (16 max)");
                    }
                    int gi = 0;
                    for(const nlohmann::json &gm : av["composition"]) {
                        ++gi;
                        if(!gm.is_object()) {
                            throw std::runtime_error(
                                "system: " + body->name + ": atmosphere.composition"
                                " entry " + std::to_string(gi) + " must be an object"
                                " of { gas, fraction }");
                        }
                        /* Checked rather than read with .value(): a mistyped entry
                           ({"gas": 5}, {"fraction": "0.5"}) would otherwise throw
                           nlohmann's type_error out of load_system with a message that
                           names neither the body nor the key -- the two things an
                           author needs to find the line. */
                        if(!gm.contains("gas") || !gm["gas"].is_string()) {
                            throw std::runtime_error(
                                "system: " + body->name + ": atmosphere.composition"
                                " entry " + std::to_string(gi) + " needs a gas name"
                                " (N2, O2, CO2, H2, He, CH4, H2O, Ar)");
                        }
                        const std::string gname = gm["gas"].get<std::string>();
                        const GasSpecies *gs = gasSpecies(gname);
                        if(gs == nullptr) {
                            throw std::runtime_error(
                                "system: " + body->name + ": atmosphere.composition"
                                " entry " + std::to_string(gi) + " names an unknown"
                                " gas '" + gname + "' (expected: N2, O2, CO2, H2,"
                                " He, CH4, H2O, Ar)");
                        }
                        if(!gm.contains("fraction") || !gm["fraction"].is_number()) {
                            throw std::runtime_error(
                                "system: " + body->name + ": atmosphere.composition "
                                + gname + " needs a numeric fraction (by mole; the"
                                " list need not sum to 1)");
                        }
                        MixFraction mf;
                        mf.gas = gs;
                        mf.fraction = gm["fraction"].get<double>();
                        s.atmosphere.composition.push_back(mf);
                    }
                    /* An authored-but-empty list is not the same as omitting the
                       key: omitting it says "dry air, the default", while "[]" says
                       "this body's air is made of nothing" and would otherwise be
                       indistinguishable from the first. */
                    if(s.atmosphere.composition.empty()) {
                        throw std::runtime_error(
                            "system: " + body->name + ": atmosphere.composition names"
                            " no gas (leave the key out for the dry-air default)");
                    }
                }
                /* Resolve the mix into the scalars, defaulting to dry air's mix.
                   Doing it HERE is what keeps the three reads (T, p, a) and the jet
                   gate on one statement of the gas: the level table below is then
                   integrated with the same Rs the mix implies. */
                {
                    std::vector<MixFraction> mix = s.atmosphere.composition;
                    if(mix.empty()) {
                        int n = 0;
                        const MixFraction *air = dryAirMix(n);
                        mix.assign(air, air + n);
                    }
                    std::string why;
                    if(!resolveMix(mix, s.atmosphere.molar_mass, s.atmosphere.gamma,
                                   s.atmosphere.oxidizer_share, why)) {
                        throw std::runtime_error(
                            "system: " + body->name + ": atmosphere.composition --"
                            " " + why);
                    }
                }
                /* The altitude/temperature LEVELS (#552 A4): [{alt, T, lapse}, ...].
                   base_p above level 0 is NOT authorable -- resolveAtmoLevels integrates each level
                   from the sea-level anchor below it, so a file cannot state a
                   pressure that contradicts its own density (atmo.h). */
                if(av.contains("layers")) {
                    if(!av["layers"].is_array()) {
                        throw std::runtime_error(
                            "system: " + body->name + ": atmosphere.layers must be"
                            " an array of { alt, T, lapse } objects");
                    }
                    for(const nlohmann::json &lv : av["layers"]) {
                        if(!lv.is_object()) {
                            throw std::runtime_error(
                                "system: " + body->name + ": atmosphere.layers"
                                " entries must be objects");
                        }
                        AtmoLayer L;
                        L.base_alt = lv.value("alt", 0.0);
                        L.base_T = lv.value("T", 0.0);
                        L.lapse = lv.value("lapse", 0.0);
                        s.atmosphere.layers.push_back(L);
                    }
                }
                /* How the air MOVES (#688): [{alt, east, north}, ...], atmo.h:WindLevel.
                   A row is the air's OWN velocity RELATIVE TO THE GROUND at that
                   geometric altitude -- never its velocity in some inertial frame,
                   because then every spinning body would author its own spin into every
                   row and a datum would change with the clock. airRelativeVel adds the
                   co-rotation beside this, in the one place the flow is composed.
                   Unknown keys are refused rather than ignored: {"easts": 5} reads as
                   "no wind", which is the one silent outcome an author would never see. */
                if(av.contains("wind")) {
                    if(!av["wind"].is_array()) {
                        throw std::runtime_error(
                            "system: " + body->name + ": atmosphere.wind must be"
                            " an array of { alt, east, north } objects");
                    }
                    if(av["wind"].empty()) {
                        throw std::runtime_error(
                            "system: " + body->name + ": atmosphere.wind is empty"
                            " -- omit the key for air that stands still");
                    }
                    /* windAt() scans the rows linearly once per air-relative read per
                       hull, so an unbounded table is a loop on the aero path -- same
                       reason (and same cap) as atmosphere.layers above. */
                    if(av["wind"].size() > 64) {
                        throw std::runtime_error(
                            "system: " + body->name + ": atmosphere.wind has too"
                            " many rows (64 max)");
                    }
                    static const char *const kWindKeys[] =
                        {"alt", "east", "north"};
                    double prev_alt = 0.0;
                    bool first_row = true;
                    int row_no = 0;
                    for(const nlohmann::json &wv : av["wind"]) {
                        ++row_no;
                        const std::string at = " row " + std::to_string(row_no) + ": ";
                        if(!wv.is_object()) {
                            throw std::runtime_error(
                                "system: " + body->name + ": atmosphere.wind"
                                + at + "must be an object");
                        }
                        /* An unknown key is a typo, and a typo here reads as "no wind"
                           -- the one silent outcome an author would never notice (the
                           same net --startships got in #709). A MISSING key is the same
                           trap from the other side: {"alt": 100} would otherwise load
                           as a row of calm air. So all three, named, or refuse. */
                        for(const auto &kv : wv.items()) {
                            bool known = false;
                            for(const char *k : kWindKeys) {
                                if(kv.key() == k) { known = true; break; }
                            }
                            if(!known) {
                                throw std::runtime_error(
                                    "system: " + body->name + ": atmosphere.wind"
                                    + at + "unknown field \"" + kv.key() + "\""
                                    " (known: alt, east, north)");
                            }
                        }
                        WindLevel W;
                        for(const char *k : kWindKeys) {
                            if(!wv.contains(k)) {
                                throw std::runtime_error(
                                    "system: " + body->name + ": atmosphere.wind"
                                    + at + "needs \"alt\", \"east\" and \"north\""
                                    " (missing \"" + k + "\")");
                            }
                            /* Checked before the read: nlohmann throws its own type
                               error, which names no body, no row and no field. */
                            if(!wv[k].is_number()) {
                                throw std::runtime_error(
                                    "system: " + body->name + ": atmosphere.wind"
                                    + at + "\"" + k + "\" must be a number");
                            }
                        }
                        W.alt = wv["alt"].get<double>();
                        W.east = wv["east"].get<double>();
                        W.north = wv["north"].get<double>();
                        /* Rows must RISE, strictly. A NaN alt fails this the same way a
                           backwards table does, which is the NaN-closed shape
                           src/angmath.h asks for. */
                        if(!first_row && !(W.alt > prev_alt)) {
                            throw std::runtime_error(
                                "system: " + body->name + ": atmosphere.wind"
                                + at + "\"alt\" does not rise above the row before"
                                " it (rows must be listed bottom-up)");
                        }
                        /* A wind feeds a Mach number, a dynamic pressure and every aero
                           force, so it must be a finite, bounded horizontal speed --
                           PER COMPONENT (atmo.h:kWindMaxSpeed). The alt is checked here
                           too because the rise rule only compares rows, so the FIRST
                           row would otherwise wave a NaN through. */
                        if(!std::isfinite(W.alt)
                           || !(std::isfinite(W.east) && std::isfinite(W.north))
                           || !(std::fabs(W.east) <= kWindMaxSpeed)
                           || !(std::fabs(W.north) <= kWindMaxSpeed)) {
                            throw std::runtime_error(
                                "system: " + body->name + ": atmosphere.wind"
                                + at + "needs finite numeric alt/east/north, and a"
                                " wind within +/-"
                                + std::to_string((int)kWindMaxSpeed)
                                + " m/s in each of east and north");
                        }
                        s.atmosphere.wind.push_back(W);
                        prev_alt = W.alt;
                        first_row = false;
                    }
                }
                /* THE ANCHOR (#739). p0 is authored; rho0 -- the number airDensity,
                   airPressure's equation of state and the dossier all read -- is what
                   it implies through the gas and this sphere's g.
                   An exponential column needs NO gas constant to invert: its datum T is
                   g H M / R (drag.h:airTemperature) and p0 = rho0 (R/M) T0, so M
                   cancels and rho0 = p0 / (g H) -- which is also why the derived
                   temperature cannot drift into being a third statement of one column.
                   A layer table inverts through its own datum ideal gas.
                   Both use datumGravity(body->g, radius, seaLevelRadius) -- the SAME
                   call dragAtm() makes per hull (#697) and the same one the level
                   integration below uses, so the datum cannot step.
                   Garbage is left to refuse below rather than divided through: if the
                   profile is missing, or level 0 sits at 0 K, rho0 stays 0 and the
                   profile guards fire with their own messages. */
                const double air_g = datumGravity(body->g, body->radius,
                                                  body->seaLevelRadius());
                if(s.atmosphere.sea_level_pressure > 0.0) {
                    if(!s.atmosphere.layers.empty()) {
                        if(s.atmosphere.layers[0].base_T > 0.0) {
                            s.atmosphere.sea_level_density =
                                s.atmosphere.sea_level_pressure
                                * s.atmosphere.molar_mass
                                / kGasR
                                / s.atmosphere.layers[0].base_T;
                        }
                    } else if(s.atmosphere.scale_height > 0.0 && air_g > 0.0) {
                        s.atmosphere.sea_level_density =
                            s.atmosphere.sea_level_pressure
                            / (air_g * s.atmosphere.scale_height);
                    }
                }
                // Air needs a profile: one exponential or a level table, and not both
                // (render-only rim = neither). A rim can exist without either.
                if(!s.atmosphere.layers.empty()) {
                    /* A layer table IS the density profile, so a scale_height beside it
                       would ask which one rho() follows. It also needs an authored top:
                       top()'s scale_height * kAtmoScaleHeights default is meaningless
                       for a profile, and the last level's ceiling -- which
                       resolveAtmoLevels must keep above 0 K -- IS that top. != rather
                       than > because a NEGATIVE scale_height beside a table is not "no
                       scale height": it is a dead number in the file that nothing reads
                       and nobody would ever notice. */
                    if(!(s.atmosphere.sea_level_pressure > 0.0)
                       || !(s.atmosphere.height > 0.0)
                       || s.atmosphere.scale_height != 0.0) {
                        throw std::runtime_error(
                            "system: " + body->name + ": atmosphere.layers needs"
                            " sea_level_pressure and height, and no scale_height");
                    }
                    /* atmoLevel() scans the table linearly, once per aero sample per
                       hull (drag.h), and no real profile is anywhere near this long --
                       the 1976 standard atmosphere uses seven levels to 85 km. A table
                       past the cap is a generator bug, and refusing it beats making
                       every aero sample on that body a thousand-compare loop. */
                    if(s.atmosphere.layers.size() > 64) {
                        throw std::runtime_error(
                            "system: " + body->name + ": atmosphere.layers has too"
                            " many levels (64 max)");
                    }
                } else if(s.atmosphere.sea_level_pressure < 0.0
                          || s.atmosphere.scale_height < 0.0
                          || (s.atmosphere.sea_level_pressure > 0.0)
                             != (s.atmosphere.scale_height > 0.0)) {
                    /* Both or neither -- and neither may be negative. A -4000 in a
                       vacuum body reads as "no air" either way, which is precisely why
                       it would otherwise survive a decade of nobody looking. */
                    throw std::runtime_error(
                        "system: " + body->name + ": atmosphere needs "
                        "sea_level_pressure and scale_height (or neither)");
                }
                /* Wind needs air to move, and air that ends. A render-only rim has no
                   air at all, and a row above top() sits in the vacuum the same file
                   already declared -- drag.h cuts the density there, so such a row is a
                   number nothing can ever reach. windAt() HOLDS the last row above the
                   table, so the top row itself is fine and worth pinning. */
                if(!s.atmosphere.wind.empty()) {
                    if(!s.atmosphere.physical()) {
                        throw std::runtime_error(
                            "system: " + body->name + ": atmosphere.wind needs"
                            " physical air to move (sea_level_pressure with"
                            " scale_height or layers) -- a render rim has none");
                    }
                    if(!(s.atmosphere.wind.back().alt <= s.atmosphere.top())) {
                        throw std::runtime_error(
                            "system: " + body->name + ": atmosphere.wind's highest"
                            " row is above the air's top ("
                            + std::to_string((long long)s.atmosphere.top())
                            + " m) -- nothing can reach it");
                    }
                }
                /* The resolved gas feeds a divide and a sqrt, and g feeds all three
                   thermodynamic reads, so a bad one is a LOAD error rather than a 0 K,
                   a negative pressure or a NaN air reaching a Mach number several steps
                   later. The !(a && b) shape is NaN-closed the way src/angmath.h
                   requires. Since #630 the two gas numbers are not authored, so this is
                   no longer advice an author could follow -- it is the species table's
                   own invariant: every gas has a real molar mass (M < 1 kg/mol is the
                   g/mol typo guard: 28.964 typed where 0.028964 was meant prints 188,000
                   K for Kerbin) and a gamma inside [1, 2], and a mix of them cannot
                   leave those bounds. If it ever trips, the fix is in atmo.h:gasSpecies,
                   and naming the body is what points a reader there. */
                if(s.atmosphere.sea_level_pressure > 0.0) {
                    if(!(body->g > 0.0)) {
                        throw std::runtime_error(
                            "system: " + body->name + ": a body with air needs g > 0");
                    }
                    /* #689: g and mass are two published statements of ONE field, and
                       the air reads both -- g weighs the air's own column (#683), mass is
                       what every arc about this body answers to. Nothing else compares
                       them, they are pinned equal at the surface by construction, and the
                       mismatch then MULTIPLIES with altitude, so it arrives in play as a
                       barometer that disagrees with a ballistic arc. The tolerance and
                       why it is 2 %: atmo.h:kAtmoGravityTol. */
                    const double implied_g = body->mu / (radius * radius);
                    char gbuf[32], ibuf[32];
                    std::snprintf(gbuf, sizeof gbuf, "%g", body->g);
                    std::snprintf(ibuf, sizeof ibuf, "%g", implied_g);
                    if(!(implied_g > 0.0)
                       || !(std::fabs(body->g / implied_g - 1.0) <= kAtmoGravityTol)) {
                        throw std::runtime_error(
                            "system: " + body->name + ": authored g " + gbuf
                            + " disagrees with G*mass/radius^2 = " + ibuf
                            + " by more than 2%: the air's column and the field an orbit"
                            " feels have to be one field (atmo.h:kAtmoGravityTol)");
                    }
                    if(!(s.atmosphere.molar_mass > 0.0
                         && s.atmosphere.molar_mass < 1.0)
                       || !(s.atmosphere.gamma >= 1.0 && s.atmosphere.gamma <= 2.0)) {
                        throw std::runtime_error(
                            "system: " + body->name + ": atmosphere.composition"
                            " resolves to a molar mass or gamma no gas in the"
                            " species table gives (atmo.h:gasSpecies)");
                    }
                }
                /* Now the level table can be integrated: it needs the gas (Rs), the
                   body's g, an anchored first level and a top, and it must stay above
                   0 K all the way up (atmo.h's invariants). Filling base_p here -- and
                   nowhere else -- is what keeps rho, p and T one column of air: the
                   density at the datum is EXACTLY the one the anchor above resolved
                   from the authored p0, because both directions are the same gas law.
                   The sea-level radius rides along (#683): this column is weighed in
                   the body's own 1/r^2 field, and it must be the SAME sphere the aero
                   pass reads through dragAtm(), or the air steps at every level seam.
                   The g rides along on that same sphere (#697): the file types it at
                   the MEAN radius, the way an atlas publishes it. */
                if(!s.atmosphere.layers.empty()
                   && !resolveAtmoLevels(s.atmosphere.layers,
                                         s.atmosphere.sea_level_pressure,
                                         s.atmosphere.molar_mass, air_g,
                                         s.atmosphere.top(), body->seaLevelRadius())) {
                    throw std::runtime_error(
                        "system: " + body->name + ": atmosphere.layers must start at"
                        " alt 0, rise strictly to below height, keep T continuous"
                        " across each level seam and above 0 K, stay stably"
                        " stratified, and keep no warming level's lapse aimed at the"
                        " body's own gravity falloff (atmo.h:resolveAtmoLevels)");
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
            // (> 90 deg flips the plane normal). A negative rate would
            // double-encode and silently load prograde (a = cbrt(mu/w^2) drops
            // the sign) (#139).
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
            // (default): the parent's orbital plane, composed here. "equator":
            // the parent's equator (the sheets' convention for regular moons);
            // composed in pass 2, where the axial tilt is resolved.
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
        // Every body gets one. Its SOI is DERIVED (bodylimits.h shellEdge) so
        // the air always fits inside the frame's enter band; an authored
        // rotating.soi is ignored. No "rotating" section (e.g. the star) => a
        // DUMMY frame with zero spin, so radii and frame switching still work.
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
        // Datum + shell, in ONE expression, off the same float radius physics
        // uses: soi - seaLevelRadius() is then exactly shellEdge() for every body,
        // which is what science.h's orbit cut and bodylimits.h's asserts read.
        rf->soi = body->seaLevelRadius() + shell;
        if(bv.contains("rotating") && bv["rotating"].is_object()) {
            const nlohmann::json &rot = bv["rotating"];
            rf->rot_ang_speed = rot.value("rot_ang_speed", 0.0);
            // Same policy as orb_ang_speed: the SPIN sense lives in axial_tilt
            // (> 90 deg). A negative rate double-flips and breaks the calendar
            // (D -> 0 gate), so reject it (#139).
            if(rf->rot_ang_speed < 0.0) {
                throw std::runtime_error("system: '" + body->name
                        + "' has negative rot_ang_speed; encode retrograde "
                        "spin with axial_tilt > pi/2, not a negative rate");
            }
            // Optional axial tilt (radians): lean the pole off the orbital
            // normal toward +X, folded into initial_orient. The spin stays
            // about +Y (the figure axis), so the pole IS the spin axis --
            // tilting spin_axis makes the pole/bands/rings precess once per
            // rotation.
            const double axial_tilt = rot.value("axial_tilt", 0.0);
            // #141: rail-longitude angles (see railAz for the convention).
            // tilt_azimuth = the direction the pole leans (default 0 = +X, the
            // ascending node). spin_phase0 = the epoch spin angle about the
            // figure axis +Y; for an untilted body that is the rail longitude
            // of surface lon 0, but that reading degrades past 90 deg of tilt
            // -- the figure-axis angle is the sound meaning. Omitting them
            // loads as pre-#141.
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
                // #147: the tilt part alone (no spin phase) is the body's
                // equator frame; equator-referred child rails hang under it
                // (pass 2), NOT under initial_orient.
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

        body->refreshParamsCache();   // surface/radius/colour_func are final

        // Per-body progress so the caller can draw a "loading..." frame.
        if(progress) { progress(i, bodies.size(), body->name); }
    }

    // --- pass 2: wire the parent/child frame tree --------------------------
    // The root's authored universe bound (inertial.soi): the tripwire below
    // rejects orbits that do not fit inside it. 0 = not authored -> skipped.
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
            // A self-orbit makes the body its own child frame and recurses any
            // tree walk. (A mutual A<->B cycle is still possible; see the
            // follow-up issue on full acyclicity.)
            if(parent == body) {
                throw std::runtime_error("system: '" + body->name
                                         + "' cannot orbit itself");
            }
            body->frame->parent = parent->frame;
            parent->frame->children.push_back(body->frame);

            // #147: equator-referred moon rails. orient = the parent's EQUATOR
            // frame * R_Y(+raan) * R_X(i): the moon's plane tips WITH the
            // parent's axis. Use equator_orient (tilt only, no spin phase) --
            // full initial_orient node-locks the rail to the prime meridian.
            {
                const nlohmann::json &ine =
                    bv.value("inertial", nlohmann::json::object());
                if(ine.value("incl_ref", std::string("orbit")) == "equator") {
                    // Pass 1 composes equator_orient before pass 2 reads it, so
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
            // #140 tripwire (the magnitude twin of the rate-sign guard): a tiny
            // w*w underflows and a becomes inf (NaN rails), or lands absurdly
            // far. railStateFromElements only rejects a <= 0.
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

            // DERIVED inertial SOI (bodylimits.h): clear of the near-body
            // shell's hysteresis band and wide enough to contain the spawn bed.
            // Authored inertial.soi is ignored here.
            f->soi = inertialSoi(soiByLaw(law, a, body->mass, parent->mass),
                                 body->rot_frame->soi,
                                 shellEdge(body->surface.atmosphere.top()));

            // Epoch orbital state for the Kepler rail: e, arg_peri and the
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
                // body at the conic's own radius, so an eccentric orbit written
                // by position silently loses it (#175). Throws, not asserts.
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

    // The ordering the game needs (bodylimits.h). Derived values cannot fail
    // these; this is the tripwire for future edits.
    for(TerrainBody *b : sys.bodies) {
        validateBodyLimits(b->name, b->seaLevelRadius(),
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
    // Per-body calendar from the SOLAR day (measureSolarDay, #201) and the
    // orbital period (year). The year snaps to whole days (calendar.h) so
    // boundaries fall on local midnight. Stars get an invalid calendar.
    // Displayed year at t == 0; solar systems author 2000, since their orbital
    // phases are J2000-referenced. Default 1.
    const int epoch_year = doc.value("epoch_year", 1);
    // #202: optional proleptic-Gregorian civil calendar; the system's t=0 is
    // this civil epoch. The CLOCK runs a fixed 86400 s civil day; the measured
    // solar day goes to solar_day_seconds (calendar.h).
    long civil_epoch_days = 0;
    double civil_epoch_sod = 0.0;   // seconds-of-day at t=0
    bool use_civil = false;
    if(doc.contains("epoch_utc") && doc["epoch_utc"].is_string()) {
        const std::string s = doc["epoch_utc"].get<std::string>();
        int y = 0, mo = 1, d = 1, hh = 0, mi = 0, se = 0;
        const int nf = std::sscanf(s.c_str(), "%d-%d-%dT%d:%d:%d",
                                   &y, &mo, &d, &hh, &mi, &se);
        const int nd = std::sscanf(s.c_str(), "%d-%d-%d", &y, &mo, &d);
        const bool ok = (nf == 6 || nd == 3)
            && mo >= 1 && mo <= 12
            && d >= 1 && d <= civil_month_days(y, mo)
            && hh >= 0 && hh <= 23 && mi >= 0 && mi <= 59
            && se >= 0 && se <= 59;
        if(!ok) {
            throw std::runtime_error(
                std::string("system: bad epoch_utc '") + s
                + "' (want YYYY-MM-DD or YYYY-MM-DDTHH:MM:SSZ)");
        }
        civil_epoch_days = days_from_civil(y, (unsigned)mo, (unsigned)d);
        if(nf == 6) {
            civil_epoch_sod = (double)hh * 3600.0 + (double)mi * 60.0 + (double)se;
        }
        use_civil = true;
    }
    // Every body shares one sun (the star). Civil (#202) is the HOME clock
    // only; every other body keeps its own derived day.
    for(size_t i = 0; i < sys.bodies.size(); i++) {
        TerrainBody *b = sys.bodies[i];
        const double D = measureSolarDay(sys.root->frame, b, sys.root);
        const double Y = (b->frame && b->frame->orb_ang_speed > 0.0)
                       ? 2.0 * std::numbers::pi / b->frame->orb_ang_speed : 0.0;
        if(use_civil && b == sys.home) {
            b->cal = Calendar::makeCivil(civil_epoch_days, civil_epoch_sod, Y, D);
        } else {
            b->cal = Calendar::make(D, Y, epoch_year);
        }
    }

    /* #687: the star's output, derived ONCE here from its authored
       surface.temperature so nothing downstream re-derives it or disagrees with
       anything else about it (solar.h owns the law). Only a star lights anything: a
       fixture that puts surface.temperature on a PLANET root would otherwise light
       its whole system, so the derivation is gated on isStar() and the mistake is
       printed rather than carried. A star with no authored temperature lights nothing,
       which is also said out loud: a system that loads with a dark sun and prints
       dashes everywhere is a data bug worth an evening less than finding silently. */
    /* #729: starLuminosity asserts a non-negative temperature (its fourth power is what
       lights the system), so the sentinel must never reach it. That is the whole guard --
       one rung, and the dark-sun warning below still fires. */
    if(sys.root->isStar() && thermalAuthored(sys.root->surface.temperature)) {
        sys.root->star_luminosity = starLuminosity(sys.root->radius,
                                                   sys.root->surface.temperature);
    } else if(!sys.root->isStar() && thermalAuthored(sys.root->surface.temperature)) {
        printf("[solar] WARNING: the frame-tree root '%s' is not a star yet authors "
               "surface.temperature; nothing in this system receives insolation.\n",
               sys.root->name.c_str());
    }
    if(sys.root->star_luminosity <= 0.0) {
        printf("[solar] WARNING: star '%s' authors no surface.temperature, so no "
               "body in this system receives any insolation.\n",
               sys.root->name.c_str());
    }

    /* #687: the same audit for the ground. A system file can state the ground's
       temperature twice -- once as the surface field, once as the air's datum
       temperature -- and the two answer to different things: the first is what a
       ground radiometer and a part's skin START at, the second is what the column
       above weighs. When the generators read ONE published row for both (a sheet's
       "Average temperature", or a giant's "Temperature at 1 bar", which is this
       file's datum level -- #689) they ARE the same number and the line says so. When
       the two rows differ the difference is PRINTED, because some do: Jupiter's authored
       scale height implies a 1-bar level of 186.7 K while its sheet's own 1-bar row says
       165 K, a 21.7 K gap that is #685's "the giants' sheet disagrees with itself" on a
       new axis and #625's layer-table question in one number. It prints rather than
       WARNINGs because a warning the shipped catalogue trips on every load is a warning
       nobody reads. */
    for(TerrainBody *b : sys.bodies) {
        /* What the air's datum temperature IS, by the only law that defines it:
           drag.h:airTemperature at alt 0 -- the first level's base_T for a layered
           profile, g H M / R for a body that authors a scale height instead (#686).
           Quoting the authored field instead would be a different number on half the
           catalogue. */
        const double airT = airTemperature(dragAtm(b->surface.atmosphere, *b), 0.0);
        const double groundT = b->surface.temperature;
        if(!(airT > 0.0) && !thermalAuthored(groundT)) {
            continue;   // nothing sourced: the readouts dash, and say nothing here
        }
        char surface_s[24], air[24], note[64];
        if(thermalAuthored(groundT)) {
            std::snprintf(surface_s, sizeof surface_s, "%11.2f K", groundT);
        } else {
            std::snprintf(surface_s, sizeof surface_s, "          - ");
        }
        if(airT > 0.0) {
            std::snprintf(air, sizeof air, "%11.2f K", airT);
        } else {
            std::snprintf(air, sizeof air, "          - ");
        }
        if(groundT > 0.0 && airT > 0.0) {
            const double dT = groundT - airT;
            if(std::fabs(dT) < 0.5) {
                std::snprintf(note, sizeof note,
                              "  (one published row states both)");
            } else {
                std::snprintf(note, sizeof note,
                              "  (two rows, %.2f K apart)", dT);
            }
        } else {
            note[0] = '\0';
        }
        /* On a star this line's "surface" is its photosphere -- the number the whole
           system's light comes from, which is exactly why the line is worth printing
           for the star too. The readouts that mean a GROUND dash for a star instead
           (tick.cpp's --solar-log does). */
        printf("         thermal  %-13s albedo %6s surface %s air base %s%s\n",
               b->name.c_str(),
               albedoAuthored(b->surface.bond_albedo) ? "authored" : "  -   ",
               surface_s, air, note);
    }

    // Recompute root-relative frame values before the first render.
    sys.root->frame->UpdateOrbitRails(0.0);

    /* #687: read the whole insolation chain back as one number a reader KNOWS. The
       star's temperature is a single authored figure, so no internal check can see it
       move (the #684/#686 meta-law); the second statement lives in the generator's
       --check against the published irradiance rows, and this is its load-time twin.
       Kerbin and Earth both expect ~1361 W/m^2 at their own rails (utils/gen_systems.py,
       utils/rss/html/earthfact.html), so a decimal-point mistake in a star's temperature
       -- which a [0, 1e6] K bound happily admits -- shows up here as a home world
       sitting in 13,610,000 W/m^2 of light. */
    if(sys.root->star_luminosity > 0.0 && sys.home && sys.home->frame) {
        const SunSample ss = sunSample(sys.root->star_luminosity,
                                       sys.root->frame->root_pos,
                                       sys.home->frame->root_pos);
        printf("         solar    %-13s L %14.4g W; home %-9s reads %10.1f W/m2 at "
               "%.4g m\n",
               sys.root->name.c_str(), sys.root->star_luminosity,
               sys.home->name.c_str(), ss.flux, ss.dist_m);
    }

    printf("Loaded system '%s': %zu bodies (home=%s, belts=%zu, buildings=%zu)\n",
           path, sys.bodies.size(),
           sys.home ? sys.home->name.c_str() : "(none)", sys.belts.size(),
           sys.world_buildings.size());

    cleanup.commit = true;   // build complete: the caller now owns the bodies
    return sys;
}
