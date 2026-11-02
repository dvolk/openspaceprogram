#include "shipdef.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <numbers>
#include <stdexcept>

#include <glm/gtc/quaternion.hpp>   // angleAxis / mat3_cast
#include <nlohmann/json.hpp>

#include "hash.h"     // partGeomFingerprint's FNV-1a mixer
#include "resdir.h"   // the part mesh path, resolved the way part.cpp resolves it
#include "drag.h"     // buildStability: the same LiftSlopeSum the flight lift loop bills

PartDef::PartDef()
    : mass(0.0), radius(1.0), height(2.0), torque(0.0),
      exhaust_velocity(0.0), jet(false), jet_fan_thrust(0.0),
      jet_intake_area(0.0), rcs_thrust(0.0), gimbal_max(0.0), gimbal_rate(0.0),
      power_draw(0.0),
      power_draw_constant(0.0), power_gen(0.0),
      crew_capacity(0), inventory_capacity(0), decoupler(false), docking_port(false),
      fuel_barrier(false), fuel_link(false), hull_margin(-1.0),
      drag(0.0), drag_forward(0.0), drag_side(0.0), drag_backward(0.0),
      lift_area(0.0), cl(0.0), stall_angle(0.0),
      control_area(0.0), control_axis(ControlAxis::Pitch), cl_control(0.0),
      max_deflection(0.0), chute_area(0.0) {
    capacity.resize((int)ResourceType::Num, 0.0);
    propellant_rate.resize((int)ResourceType::Num, 0.0);
}

const PartDef *PartsCatalog::find(const std::string &name) const {
    for(size_t i = 0; i < parts.size(); i++) {
        if(parts[i].name == name) { return &parts[i]; }
    }
    return nullptr;
}

static int resource_index_from_string(const std::string &s, const std::string &ctx) {
    if(s == "hydrogen") { return (int)ResourceType::Hydrogen; }
    if(s == "lox") { return (int)ResourceType::LOX; }
    if(s == "ec") { return (int)ResourceType::EC; }
    if(s == "oxygen") { return (int)ResourceType::Oxygen; }
    if(s == "water") { return (int)ResourceType::Water; }
    if(s == "food") { return (int)ResourceType::Food; }
    if(s == "hydrazine") { return (int)ResourceType::Hydrazine; }
    if(s == "jetfuel") { return (int)ResourceType::JetFuel; }
    // ctx already ends with ": " (both callers build it that way); a second colon
    // here read as "parts: x: : unknown resource" (#496 review).
    throw std::runtime_error(ctx + "unknown resource '" + s
                             + "' (expected: hydrogen, lox, ec, oxygen, water, food, hydrazine, jetfuel)");
}

/* One NUMBER, read so that the error names the file, the part and the field (#496).
   `what` is the caller's own naming of the value -- "parts: pod in
   res/data/parts.json: \"mass\"", or a map entry's key spelled out -- because
   pv.value(key, def) answers a string with nlohmann's type_error ("type must be
   number, but is string"), which is true and useless to whoever typed it. `unit`
   says what the number measures ("" when it is dimensionless): "must be a number"
   is only half the fix, the other half is knowing that "angle" is degrees and "z"
   is metres. Keyed fields and map-valued entries (propellant rates, capacities)
   both read through this so they cannot drift into different wording for the same
   typo. The array-valued fields ("point"/"normal", "rcs_nozzle_dirs") and the
   object-valued "attitude" still word their own refusals; they are shaped problems,
   so they are not this helper's shape. */
static double wantNumber(const nlohmann::json &v, const std::string &what,
                         const char *unit = "") {
    if(!v.is_number()) {
        // What it actually got, because "must be a number" sent people hunting for a
        // quote they had already removed -- the usual cause is a neighbouring field.
        throw std::runtime_error(what + " must be a number"
                                 + (unit && *unit ? std::string(" (") + unit + ")" : "")
                                 + ", got " + v.type_name());
    }
    return v.get<double>();
}

/* wantNumber for a keyed optional: absent takes the default, present-and-wrong throws. */
static double optNumber(const nlohmann::json &pv, const char *key, double def,
                        const std::string &ctx, const char *unit = "") {
    if(!(pv.is_object() && pv.contains(key))) { return def; }
    return wantNumber(pv.at(key), ctx + std::string("\"") + key + "\"", unit);
}

/* The same for a COUNT. is_number() would let 1.5 through, and a stage number is
   not a quantity to round -- a ship that stages halfway is a typo, not a fraction.
   A whole one written 2.0 is fine: JSON has one number type and authors write
   whatever their editor produced.
   The int window is part of "whole", and it is checked BEFORE the cast. The old
   version trusted is_number_integer(), which is true of an UNSIGNED literal, so
   4294967296 sailed through it and get<int>() handed back 0 -- measured on the
   previous tree, "crew_capacity": 4294967296 loaded as no seats at all, and the
   wrapped number then passed every >= test below it. get<double>() does not wrap
   (2^63 reads as 9.22337e18); the narrowing cast does, and on a double outside
   int's range that cast is UB. (#496 review) */
static int optInt(const nlohmann::json &pv, const char *key, int def,
                  const std::string &ctx) {
    if(!(pv.is_object() && pv.contains(key))) { return def; }
    const nlohmann::json &v = pv.at(key);
    if(!v.is_number()) {
        throw std::runtime_error(ctx + std::string("\"") + key + "\" must be a whole number"
                                 + ", got " + v.type_name());
    }
    /* One comparison covers an integer literal as well as a float one: the window is
       well inside what a double holds exactly, so floor() is not an approximation
       here. */
    const double d = v.get<double>();
    if(!std::isfinite(d) || d != std::floor(d)) {
        throw std::runtime_error(ctx + std::string("\"") + key + "\" must be a whole number");
    }
    if(d < (double)std::numeric_limits<int>::min()
       || d > (double)std::numeric_limits<int>::max()) {
        throw std::runtime_error(ctx + std::string("\"") + key
                                 + "\" is outside the range of counts this game can store"
                                   " (-2147483648..2147483647)");
    }
    return (int)d;
}

static glm::dvec3 parse_vec3(const nlohmann::json &obj, const char *key,
                             const std::string &ctx) {
    if(!obj.contains(key) || !obj[key].is_array() || obj[key].size() != 3) {
        throw std::runtime_error(ctx + std::string("\"") + key
                                 + "\" must be a 3-element array [x, y, z]");
    }
    const nlohmann::json &a = obj[key];
    for(int k = 0; k < 3; k++) {
        if(!a[k].is_number()) {
            throw std::runtime_error(ctx + std::string("\"") + key
                                     + "\" must hold three numbers");
        }
    }
    return glm::dvec3(a[0].get<double>(), a[1].get<double>(), a[2].get<double>());
}

/* What a part's SHAPE was, hashed (#456). A save keeps SOLVED part poses, not
   the attach spec that produced them (save.h), so the shapes a pose was solved
   between are the only way to tell an old save from a reshaped one: the parent's
   radius and nodes say where the child was bolted, and the child's own size says
   how far it stuck out. a27bcd1 is the repro: the wing's mesh went 2x2 -> 4x2
   and its radius 1.0 -> 2.0, so every pre-commit save rebuilds its wings 1 m
   inside the fuselage, +30 kg and +6 m2 of lift each, with nobody told.
   The mesh goes in as the FILE's bytes rather than parsed vertices: assimp is
   not in this translation unit (the parse stays GL-free and headless-testable),
   and an in-place edit and a swapped path are the same event to a save. The cost
   is that a re-export changing only comments or float precision moves the value
   too -- a false refusal, which is the safe direction: it costs a sentence
   rather than flying a ship nobody built. */
/* Bump when the SHAPE a mesh file yields changes without the file's bytes
   changing. A saved pose was solved against the hull assimp BUILT from the .obj
   (vab.cpp raycasts body->hullVerts, part.cpp feeds get_mesh through
   mesh.cpp's post-processing flags), so an assimp pin bump or a flag change
   moves every hull and leaves every .obj byte identical -- #456's failure mode
   with nothing in the file to see it. This is also the lever for any change to
   what partGeomFingerprint itself covers: bump it and every existing save says
   so in words, rather than as a pair of meaningless hex numbers. */
static const uint64_t kGeomEpoch = 1;

/* A part's mesh file, read once: the bytes that go into geom_fingerprint and
   the bounding box of its vertices (#457). Both are properties of the FILE, so
   the 19 catalog entries that share a mesh with another entry could share this
   read too; they do not, because the whole catalog parse costs ~3 ms and a
   cache would be the only thing in it. */
MeshGeom readMeshGeom(const std::string &meshSubpath) {
    MeshGeom g;
    /* A directory is readable until you try to read it, and then libstdc++
       throws out of the stream buffer rather than setting failbit -- and a
       catalog entry with no "mesh" resolves to res/ itself. Check for a regular
       file before opening it. */
    std::error_code ec;
    const std::string full = resdir::path("res/" + meshSubpath);
    if(!std::filesystem::is_regular_file(full, ec)) { return g; }
    std::ifstream m(full, std::ios::binary);
    if(!m.is_open()) {
        /* Unreadable folds in one stable value rather than throwing: the
           catalog has never required the file to exist (tests name "a.obj"),
           and a file that appears later moves the fingerprint, as it should. An
           EMPTY file hashes the same as a missing one -- both mean "no geometry
           to read here", and the mesh path mixed above keeps the two parts
           apart. */
        return g;
    }
    g.readable = true;
    /* A read error part-way through a file throws out of the stream buffer in
       libstdc++, and this runs while booting: an unreadable mesh has to degrade
       to "no extents, no geometry to hash", not abort the process. */
    try {
        g.bytes.assign(std::istreambuf_iterator<char>(m),
                       std::istreambuf_iterator<char>());
    } catch(const std::ios_base::failure &) {
        g.readable = false;
        return g;
    }
    /* The vertices, scanned out of the same bytes: a vertex line is 'v' at the
       start of a line plus three floats, which is how assimp reads them too, and
       the aiProcess flags mesh.cpp passes (CalcTangentSpace/Triangulate/
       JoinIdenticalVertices/SortByPType) transform nothing -- so this box is the
       box BuildHull convex-hulls and the box drag.h projects. Measured over the
       shipped catalog (tmp/ext_vs_assimp.cpp): all 46 unique meshes agree with
       what assimp loads to <1e-8 m (assimp holds vertices as float, the file has
       doubles). The one way they could part: mesh.cpp takes scene->mMeshes[0]
       only, so a .obj holding several objects would leave the sim with less
       geometry than this box claims -- no such file today, and an assimp change
       that moves the hull is what kGeomEpoch is for.
       Scanned by hand rather than with getline + an istringstream per line: that
       version cost ~5 ms over the catalog, several times the hash over the same
       bytes, for a boot-only step. An unparseable line only costs an extent, not
       the hash, which still covers the exact bytes. */
    char *base = g.bytes.data();
    char *end = base + g.bytes.size();
    for(char *p = base; p < end; ) {
        char *s = p;
        // assimp tolerates an indent; a scan that does not would read a
        // hand-edited .obj as a part with no vertices.
        while(s < end && (*s == ' ' || *s == '\t')) { s++; }
        if(s < end && s[0] == 'v' && s + 1 < end && (s[1] == ' ' || s[1] == '\t')) {
            char *q = s + 2;
            double val[3];
            bool ok = true;
            for(int k = 0; k < 3; k++) {
                while(q < end && (*q == ' ' || *q == '\t')) { q++; }
                char *stop = nullptr;
                val[k] = std::strtod(q, &stop);
                if(stop == q) { ok = false; break; }
                q = stop;
            }
            if(ok) {
                const glm::dvec3 v(val[0], val[1], val[2]);
                if(!g.hasExtents) {
                    g.lo = g.hi = v;
                    g.hasExtents = true;
                } else {
                    g.lo = glm::min(g.lo, v);
                    g.hi = glm::max(g.hi, v);
                }
            }
        }
        char *nl = static_cast<char *>(std::memchr(p, '\n', (size_t)(end - p)));
        if(nl == nullptr) { break; }
        p = nl + 1;
    }
    return g;
}

uint64_t partGeomFingerprint(const PartDef &d, const MeshGeom &mg) {
    if(d.fuel_link) { return 0; }   // a marker entry: no body, so no shape
    uint64_t h = fnv::start();
    fnv::mix(h, kGeomEpoch);
    fnv::mixText(h, d.mesh);
    fnv::mixDouble(h, d.radius);
    fnv::mixDouble(h, d.height);
    if(!mg.readable) {
        fnv::mix(h, 0);
    } else {
        for(size_t k = 0; k < mg.bytes.size(); k++) {
            h = (h ^ (uint64_t)(unsigned char)mg.bytes[k]) * 1099511628211ull;
        }
        fnv::mix(h, (uint64_t)mg.bytes.size());
    }
    /* Nodes last: they are usually synthesized from radius/height, and an
       authored set is exactly the kind of edit that moves a bolt point. */
    fnv::mix(h, (uint64_t)d.nodes.size());
    for(const Node &nd : d.nodes) {
        fnv::mixText(h, nd.id);
        fnv::mixDouble(h, nd.pos.x);
        fnv::mixDouble(h, nd.pos.y);
        fnv::mixDouble(h, nd.pos.z);
        fnv::mixDouble(h, nd.dir.x);
        fnv::mixDouble(h, nd.dir.y);
        fnv::mixDouble(h, nd.dir.z);
        fnv::mix(h, nd.surface ? 1 : 0);
    }
    return h;
}

std::string geomFingerprintHex(uint64_t v) {
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%016llx", (unsigned long long)v);
    return std::string(buf);
}

PartsCatalog load_parts_catalog(const char *path) {
    std::ifstream f(path);
    if(!f.is_open()) {
        throw std::runtime_error(std::string("parts: cannot open ") + path);
    }
    nlohmann::json doc;
    try {
        doc = nlohmann::json::parse(f, nullptr, true);
    } catch(const std::exception &e) {
        throw std::runtime_error(std::string("parts: bad JSON in ") + path
                                 + std::string(": ") + e.what());
    }
    if(!doc.is_object() || !doc.contains("parts") || !doc["parts"].is_array()
       || doc["parts"].empty()) {
        throw std::runtime_error(std::string("parts: no parts in ") + path);
    }

    PartsCatalog cat;
    const nlohmann::json &arr = doc["parts"];
    for(size_t i = 0; i < arr.size(); i++) {
        const nlohmann::json &pv = arr[i];

        PartDef d;
        d.name = pv.value("name", std::string(""));
        if(d.name.empty()) {
            throw std::runtime_error(std::string("parts: entry ") + std::to_string(i)
                                     + " of " + path + ": missing \"name\"");
        }
        /* The prefix every bad field in this entry quotes (#496). The file is in it
           because a catalog is often split across several, and because the ship
           loader's prefix has carried the path since the first version -- without it
           the two loaders answered the same typo differently. */
        const std::string ctx = "parts: " + d.name + " in " + path + ": ";
        if(cat.find(d.name) != nullptr) {
            throw std::runtime_error(ctx + "duplicate part name");
        }

        d.type = pv.value("type", std::string(""));
        d.display_name = pv.value("display_name", std::string(""));

        // a marker entry: no geometry, so the mesh/mass/size validation is skipped
        d.fuel_link = pv.value("fuel_link", false);

        if(!d.fuel_link) {
            d.mesh = pv.value("mesh", std::string(""));
            d.texture = pv.value("texture", std::string(""));
            if(d.mesh.empty() || d.texture.empty()) {
                throw std::runtime_error(ctx + "missing \"mesh\"/\"texture\"");
            }
            // all-or-nothing (a half-set is a catalog bug)
            d.shroud = pv.value("shroud", std::string(""));
            d.shroud_texture = pv.value("shroud_texture", std::string(""));
            if((d.shroud.empty()) != (d.shroud_texture.empty())) {
                throw std::runtime_error(ctx
                                         + "\"shroud\" and \"shroud_texture\" must be set together");
            }
            d.mass = optNumber(pv, "mass", -1.0, ctx, "kg");
            if(d.mass <= 0.0) {
                throw std::runtime_error(ctx + "\"mass\" must be > 0 (kg)");
            }

            /* Defaults are the legacy 2 m cube, so pre-size parts are unchanged. */
            d.radius = optNumber(pv, "radius", 1.0, ctx, "m");
            d.height = optNumber(pv, "height", 2.0, ctx, "m");
            if(d.radius <= 0.0) {
                throw std::runtime_error(ctx + "\"radius\" must be > 0 (m)");
            }
            if(d.height <= 0.0) {
                throw std::runtime_error(ctx + "\"height\" must be > 0 (m)");
            }
        }

        d.torque = optNumber(pv, "torque", 0.0, ctx, "N m");
        if(d.torque < 0.0) {
            throw std::runtime_error(ctx + "\"torque\" must be >= 0 (N m)");
        }

        d.rcs_thrust = optNumber(pv, "rcs_thrust", 0.0, ctx, "N");
        if(d.rcs_thrust < 0.0) {
            throw std::runtime_error(ctx + "\"rcs_thrust\" must be >= 0 (N)");
        }

        /* Nozzle BELL dirs (exhaust rays, part-local, unit; the ship is
           pushed opposite -- see PartDef::rcs_nozzles, src/rcs.h #428). A
           zero vector or a malformed triple is authored garbage: refuse
           the load. */
        d.rcs_nozzles.clear();
        if(pv.contains("rcs_nozzle_dirs")) {
            if(!pv["rcs_nozzle_dirs"].is_array()) {
                throw std::runtime_error(ctx + "\"rcs_nozzle_dirs\" must be an array");
            }
            for(const auto &e : pv["rcs_nozzle_dirs"]) {
                if(!e.is_array() || e.size() != 3
                   || !e[0].is_number() || !e[1].is_number() || !e[2].is_number()) {
                    throw std::runtime_error(ctx + "\"rcs_nozzle_dirs\" entries must be [x,y,z] numbers");
                }
                const glm::dvec3 v(e[0].get<double>(), e[1].get<double>(), e[2].get<double>());
                const double L = glm::length(v);
                if(L <= 1e-6) {
                    throw std::runtime_error(ctx + "\"rcs_nozzle_dirs\" entry must be non-zero");
                }
                d.rcs_nozzles.push_back(v / L);
            }
            // Part::rcsFired (the render plume slots) is fixed at kMaxRcsNozzles.
            if(d.rcs_nozzles.size() > PartDef::kMaxRcsNozzles) {
                throw std::runtime_error(ctx + "\"rcs_nozzle_dirs\" has more than "
                                         + std::to_string(PartDef::kMaxRcsNozzles) + " dirs");
            }
        }

        /* Engine gimbal (TVC): a part either cannot gimbal at all or needs a
           nonzero slew rate to reach its limit -- a limit with a dead
           actuator is authored data nobody can use. */
        d.gimbal_max = optNumber(pv, "gimbal_max", 0.0, ctx, "rad");
        if(d.gimbal_max < 0.0) {
            throw std::runtime_error(ctx + "\"gimbal_max\" must be >= 0 (rad)");
        }
        /* A nozzle gimbal past 90 deg is not a thing: at 90 the exhaust is sideways,
           and swinging further points it FORWARD of the mount while sin() of the
           limit -- the lateral-force fraction the attitude laws budget with -- falls
           back DOWN. Accepting it means a 120 deg nozzle is scored as WEAKER than a
           60 deg one, and a negative sin() would hand the laws a negative budget,
           whose `<= 0` test is the "this ship cannot turn" early-out (live wheels
           going dead). Nothing near this is authored: every shipped engine is 5 deg. */
        if(d.gimbal_max > std::numbers::pi_v<double> * 0.5) {
            throw std::runtime_error(ctx + "\"gimbal_max\" must be <= pi/2 (rad)");
        }
        d.gimbal_rate = optNumber(pv, "gimbal_rate", 0.0, ctx, "rad/s");
        if(d.gimbal_rate < 0.0) {
            throw std::runtime_error(ctx + "\"gimbal_rate\" must be >= 0 (rad/s)");
        }
        if(d.gimbal_max > 0.0 && d.gimbal_rate <= 0.0) {
            throw std::runtime_error(ctx + "\"gimbal_max\" > 0 requires a positive "
                                          "\"gimbal_rate\" (rad/s)");
        }

        d.power_draw = optNumber(pv, "power_draw", 0.0, ctx, "W");
        if(d.power_draw < 0.0) {
            throw std::runtime_error(ctx + "\"power_draw\" must be >= 0 (W)");
        }
        d.power_draw_constant = optNumber(pv, "power_draw_constant", 0.0, ctx, "W");
        if(d.power_draw_constant < 0.0) {
            throw std::runtime_error(ctx + "\"power_draw_constant\" must be >= 0 (W)");
        }
        d.power_gen = optNumber(pv, "power_gen", 0.0, ctx, "W");
        if(d.power_gen < 0.0) {
            throw std::runtime_error(ctx + "\"power_gen\" must be >= 0 (W)");
        }

        bool has_prop = pv.contains("propellant");
        bool has_ve = pv.contains("exhaust_velocity");
        d.exhaust_velocity = optNumber(pv, "exhaust_velocity", 0.0, ctx, "m/s");
        if(has_prop) {
            if(!pv["propellant"].is_object() || pv["propellant"].empty()) {
                throw std::runtime_error(std::string(ctx)
                                         + "\"propellant\" must be a non-empty object");
            }
            for(auto it = pv["propellant"].begin(); it != pv["propellant"].end(); ++it) {
                /* The KEY before the VALUE, in both directions of the mistake: the
                   resource name first (on a typo'd key, "unknown resource 'loox'"
                   explains the mistake while a complaint about its value does not),
                   and the rules about that resource -- is it one a thruster can
                   burn at all -- before rules about the number written under it.
                   Otherwise {"ec": -3} answers "must be >= 0 (kg/s)" about a key
                   that is not legal whatever its number is. */
                const int idx = resource_index_from_string(it.key(), ctx);
                if(idx == (int)ResourceType::EC) {
                    throw std::runtime_error(ctx + "\"propellant\" cannot be EC "
                                             "(energy in Wh, not a mass propellant)");
                }
                const std::string what = std::string(ctx) + "\"propellant\" rate for '"
                                         + it.key() + "'";
                const double rate = wantNumber(it.value(), what, "kg/s");
                if(rate < 0.0) {
                    // Names the propellant: a flow block can hold several, and the
                    // author needs to know which line has the stray minus.
                    throw std::runtime_error(what + " must be >= 0 (kg/s)");
                }
                d.propellant_rate[idx] = rate;
            }
        }
        if(has_prop != has_ve) {
            throw std::runtime_error(std::string(ctx)
                                     + "\"propellant\" and \"exhaust_velocity\" must be given together");
        }
        if(has_prop && (d.totalPropellantRate() <= 0.0 || d.exhaust_velocity <= 0.0)) {
            throw std::runtime_error(std::string(ctx)
                                     + "\"propellant\" (total) and \"exhaust_velocity\" must be > 0");
        }

        if(pv.contains("jet")) {
            d.jet = pv["jet"].get<bool>();
        }
        d.jet_fan_thrust = optNumber(pv, "jet_fan_thrust", 0.0, ctx, "N");
        if(d.jet_fan_thrust < 0.0) {
            throw std::runtime_error(ctx + "\"jet_fan_thrust\" must be >= 0 (N)");
        }
        d.jet_intake_area = optNumber(pv, "jet_intake_area", 0.0, ctx, "m^2");
        if(d.jet_intake_area < 0.0) {
            throw std::runtime_error(ctx + "\"jet_intake_area\" must be >= 0 (m^2)");
        }
        if(d.jet && !(d.totalPropellantRate() > 0.0 && d.exhaust_velocity > 0.0)) {
            throw std::runtime_error(ctx + "\"jet\" requires \"propellant\" "
                                          "and \"exhaust_velocity\" (the real exhaust velocity)");
        }
        // The flight jet branch draws JetFuel specifically: any other
        // propellant would make fan/ram thrust while burning nothing.
        if(d.jet) {
            if(d.propellant_rate[(int)ResourceType::JetFuel] <= 0.0) {
                throw std::runtime_error(ctx + "\"jet\" propellant must include \"jetfuel\"");
            }
            for(int r = 0; r < (int)ResourceType::Num; r++) {
                if(r != (int)ResourceType::JetFuel && d.propellant_rate[r] > 0.0) {
                    throw std::runtime_error(ctx + "\"jet\" propellant must be ONLY "
                                             "\"jetfuel\" (air is the free oxidizer)");
                }
            }
        }

        if(pv.contains("capacity")) {
            if(!pv["capacity"].is_object() || pv["capacity"].empty()) {
                throw std::runtime_error(std::string(ctx)
                                         + "\"capacity\" must be a non-empty object");
            }
            for(auto it = pv["capacity"].begin(); it != pv["capacity"].end(); ++it) {
                /* Name before value, as in "propellant": an unknown resource key is
                   the typo worth reporting first. */
                const int idx = resource_index_from_string(it.key(), ctx);
                /* Same array, different unit: EC is watt-hours there, not kilograms
                   (Vehicle::getPower sums it as Wh; partLoadMass skips it as "energy,
                   no mass"), so the message cannot say kg for it. */
                const char *unit = (idx == (int)ResourceType::EC) ? "Wh" : "kg";
                const std::string what = std::string(ctx) + "\"capacity\" for '"
                                         + it.key() + "'";
                const double amount = wantNumber(it.value(), what, unit);
                // A negative tank used to load: it cancels a neighbour in the
                // total > 0 test below, then hands the flow solver a negative
                // ceiling (#496 review).
                if(amount < 0.0) {
                    throw std::runtime_error(what + " must be >= 0 (" + unit + ")");
                }
                /* No narrowing: see PartDef::capacity (#379 made the contents double,
                   #530 made this match them). */
                d.capacity[idx] = amount;
            }
            double cap_total = 0.0;
            for(size_t r = 0; r < d.capacity.size(); r++) { cap_total += d.capacity[r]; }
            if(cap_total <= 0.0) {
                // No unit in the parentheses: the array adds kilograms AND
                // watt-hours, so that total is not a physical quantity. The rule is
                // just that a tank has to hold something.
                throw std::runtime_error(std::string(ctx)
                                         + "\"capacity\" must hold more than 0 of some resource");
            }
            /* ...and an infinity has to be refused by this gate too. #530 widened the
               store so a legal number can no longer BE an infinity, but two legal
               numbers still add to one: JSON refuses a single literal over DBL_MAX, so
               the reachable route is an overflow in the sum above (measured: 1e308 +
               1e308 -> cap_total inf, accepted). From there inf > 0 clears every test
               the code has, partLoadMass hands Bullet an infinite mass -- whose
               principal-axis transform returns a NaN inertia tensor and centre of mass
               -- and a burn divides its pro-rata share by an infinite layer total
               (staging.cpp:304). Same stance as the widening: no invented ceiling, only
               the rule that what the game cannot represent must not load -- which is
               already this file's answer for counts (optInt: "outside the range of
               counts this game can store"). */
            if(!std::isfinite(cap_total)) {
                throw std::runtime_error(std::string(ctx)
                                         + "\"capacity\" totals more than the game can store");
            }
        }

        d.crew_capacity = optInt(pv, "crew_capacity", d.crew_capacity, ctx);
        if(d.crew_capacity < 0) {
            throw std::runtime_error(std::string(ctx)
                                     + "\"crew_capacity\" must be >= 0 (seats)");
        }

        d.inventory_capacity = optInt(pv, "inventory_capacity", d.inventory_capacity, ctx);
        if(d.inventory_capacity < 0) {
            throw std::runtime_error(std::string(ctx)
                                     + "\"inventory_capacity\" must be >= 0 (slots)");
        }

        if(pv.contains("experiment_family")) {
            d.experiment_family = pv["experiment_family"].get<std::string>();
        }

        if(pv.contains("experiment_storage")) {
            const std::string s = pv["experiment_storage"].get<std::string>();
            if(s == "instrument")      { d.experiment_storage = ExpStorage::Instrument; }
            else if(s == "courier")    { d.experiment_storage = ExpStorage::Courier; }
            else if(s == "container")  { d.experiment_storage = ExpStorage::Container; }
            else if(s == "none")       { d.experiment_storage = ExpStorage::None; }
            else {
                throw std::runtime_error(std::string(ctx)
                    + "\"experiment_storage\" must be instrument/courier/container/none, got \""
                    + s + "\"");
            }
        }

        if(pv.contains("decoupler")) {
            d.decoupler = pv["decoupler"].get<bool>();
        }

        /* A decoupler is a fuel barrier by definition: force it, so a stale
           catalog regen cannot silently drop the flag. */
        if(pv.contains("fuel_barrier")) {
            d.fuel_barrier = pv["fuel_barrier"].get<bool>();
        }
        if(d.decoupler) { d.fuel_barrier = true; }

        // A docking port is a fuel barrier too (the ships' fuel systems meet there).
        if(pv.contains("docking_port")) {
            d.docking_port = pv["docking_port"].get<bool>();
        }
        if(d.docking_port) { d.fuel_barrier = true; }

        /* hull_margin is a distance, and its ABSENT form is the sentinel -1 ("use
           the physics default"). So the range check stays inside the presence test:
           -1 is the field's default, not an author's number. */
        if(pv.contains("hull_margin")) {
            d.hull_margin = optNumber(pv, "hull_margin", d.hull_margin, ctx, "m");
            if(d.hull_margin < 0.0) {
                throw std::runtime_error(std::string(ctx)
                                         + "\"hull_margin\" must be >= 0 (m)");
            }
        }

        d.drag = optNumber(pv, "drag", 0.0, ctx);
        if(d.drag < 0.0) {
            throw std::runtime_error(ctx +
                "\"drag\" must be >= 0 (dimensionless drag coefficient)");
        }
        d.drag_forward = optNumber(pv, "drag_forward", d.drag, ctx);
        if(d.drag_forward < 0.0) {
            throw std::runtime_error(ctx +
                "\"drag_forward\" must be >= 0 (dimensionless)");
        }
        d.drag_side = optNumber(pv, "drag_side", d.drag, ctx);
        if(d.drag_side < 0.0) {
            throw std::runtime_error(ctx +
                "\"drag_side\" must be >= 0 (dimensionless)");
        }
        d.drag_backward = optNumber(pv, "drag_backward", d.drag, ctx);
        if(d.drag_backward < 0.0) {
            throw std::runtime_error(ctx +
                "\"drag_backward\" must be >= 0 (dimensionless)");
        }
        d.lift_area = optNumber(pv, "lift_area", 0.0, ctx, "m^2");
        if(d.lift_area < 0.0) {
            throw std::runtime_error(ctx + "\"lift_area\" must be >= 0 (m^2)");
        }
        d.cl = optNumber(pv, "cl", 0.0, ctx, "per rad");
        if(d.cl < 0.0) {
            throw std::runtime_error(ctx + "\"cl\" must be >= 0 (per rad)");
        }
        d.stall_angle = optNumber(pv, "stall_angle", 0.0, ctx, "rad");
        if(d.stall_angle < 0.0) {
            throw std::runtime_error(ctx + "\"stall_angle\" must be >= 0 (rad)");
        }
        d.control_area = optNumber(pv, "control_area", 0.0, ctx, "m^2");
        if(d.control_area < 0.0) {
            throw std::runtime_error(ctx + "\"control_area\" must be >= 0 (m^2)");
        }
        // An unknown axis is a load error, not a silent default.
        const std::string axis = pv.value("control_axis", std::string("pitch"));
        if(axis == "pitch")      { d.control_axis = ControlAxis::Pitch; }
        else if(axis == "yaw")   { d.control_axis = ControlAxis::Yaw; }
        else if(axis == "roll")  { d.control_axis = ControlAxis::Roll; }
        else {
            throw std::runtime_error(ctx + "\"control_axis\" must be "
                                          "\"pitch\", \"yaw\" or \"roll\"");
        }
        d.cl_control = optNumber(pv, "cl_control", 0.0, ctx, "per rad");
        if(d.cl_control < 0.0) {
            throw std::runtime_error(ctx + "\"cl_control\" must be >= 0 (per rad)");
        }
        d.max_deflection = optNumber(pv, "max_deflection", 0.0, ctx, "rad");
        if(d.max_deflection < 0.0) {
            throw std::runtime_error(ctx + "\"max_deflection\" must be >= 0 (rad)");
        }
        /* A part that STEERS has to say where it stops biting (#494). controlForce
           droops through liftCurve exactly like a wing, and stall_angle is the angle
           where its authority peaks; a control surface shipped without it would keep
           the old linear-to-the-stop law while every wing in the catalog stalls at
           0.35, and at full travel that is a factor of ~1.6 in authority -- exactly
           the kind of difference a plane's tuning silently absorbs instead of
           noticing. Required rather than defaulted, so the next steering part is
           authored with the number instead of inheriting a fiction. */
        if(d.isControlSurface() && d.stall_angle <= 0.0) {
            throw std::runtime_error(ctx + "a control surface needs \"stall_angle\" "
                                          "(rad): where a deflected surface stops biting");
        }
        /* ... and the travel stop has to sit inside the droop. liftCurve is zero at
           twice the stall angle and NEGATIVE past it, so a surface that can be
           cranked past 2x its stall angle pulls the wrong way for the last part of
           the stick's throw. Full lock should mean maximum authority, even if it no
           longer means MORE of it. */
        if(d.isControlSurface() && d.max_deflection >= 2.0 * d.stall_angle) {
            throw std::runtime_error(ctx + "\"max_deflection\" (" + std::to_string(d.max_deflection)
                                          + " rad) must be < 2x \"stall_angle\" ("
                                          + std::to_string(d.stall_angle)
                                          + " rad): past twice the stall angle a surface "
                                            "reverses, so the stick would run backwards");
        }
        /* Three more authoring traps in the same family, all found by trying to make
           the two-readings stall_angle safe rather than lucky (#494's review). */
        /* One surface, one law. lift_area makes the part a lifting surface billed on
           its OWN angle of attack; control_area makes a second, independent system
           billed on deflection. Authored on the same part, one stall_angle would mean
           both things at once -- the incidence number would cap the travel stop -- and
           the plate would be billed induced drag twice, as two systems with no
           interaction. No shipped part does this; refuse the next one instead of
           flying it. A flapped tail is a tailplane PART plus an elevator PART. */
        if(d.lift_area > 0.0 && d.control_area > 0.0) {
            throw std::runtime_error(ctx + "a part cannot carry both \"lift_area\" and "
                                          "\"control_area\": one surface, one law");
        }
        /* A steering part needs its travel stop as well as its area, because
           isControlSurface() is the pair. control_area with no max_deflection is not a
           control surface at all: it loads, never moves, never steers, never appears in
           [ctrl], and says nothing -- the exact typo a catalog edit makes. */
        if((d.control_area > 0.0) != (d.max_deflection > 0.0)) {
            throw std::runtime_error(ctx + "\"control_area\" and \"max_deflection\" come as a "
                                          "pair: a part with one and not the other is a "
                                          "decoration, not a control surface");
        }
        /* And it needs a slope to push with. cl_control falls back to cl, so a steering
           part with neither makes 0 N at any deflection and is invisible in every log
           (vehicle.cpp's controlCl <= 0 skip). */
        if(d.isControlSurface() && (d.cl_control > 0.0 ? d.cl_control : d.cl) <= 0.0) {
            throw std::runtime_error(ctx + "a control surface needs \"cl_control\" (or \"cl\") "
                                          "> 0: with no slope it makes no force at any deflection");
        }
        /* A chute must ship its canopy: the overlay pair is all-or-nothing
           (like shroud) and REQUIRED when chute_area > 0. */
        d.chute_area = optNumber(pv, "chute_area", 0.0, ctx, "m^2");
        if(d.chute_area < 0.0) {
            throw std::runtime_error(ctx + "\"chute_area\" must be >= 0 (m^2)");
        }
        d.chute_mesh = pv.value("chute_mesh", std::string(""));
        d.chute_texture = pv.value("chute_texture", std::string(""));
        if((d.chute_mesh.empty()) != (d.chute_texture.empty())) {
            throw std::runtime_error(ctx
                                     + "\"chute_mesh\" and \"chute_texture\" must be set together");
        }
        if(d.chute_area > 0.0 && d.chute_mesh.empty()) {
            throw std::runtime_error(ctx + "\"chute_area\" > 0 requires "
                                          "\"chute_mesh\"/\"chute_texture\"");
        }

        /* An explicit "nodes" array wins; fuel links get none. */
        if(!d.fuel_link) {
            if(pv.contains("nodes")) {
                if(!pv["nodes"].is_array()) {
                    throw std::runtime_error(ctx + "\"nodes\" must be an array");
                }
                const nlohmann::json &na = pv["nodes"];
                for(size_t k = 0; k < na.size(); k++) {
                    const nlohmann::json &nv = na[k];
                    const std::string nctx = ctx + "node " + std::to_string(k) + ": ";
                    Node nd;
                    nd.id = nv.value("id", std::string(""));
                    if(nd.id.empty()) {
                        throw std::runtime_error(nctx + "missing \"id\"");
                    }
                    if(d.findNode(nd.id) != nullptr) {
                        throw std::runtime_error(ctx + "duplicate node id \"" + nd.id + "\"");
                    }
                    nd.pos = parse_vec3(nv, "pos", ctx + "node \"" + nd.id + "\": ");
                    nd.dir = parse_vec3(nv, "dir", ctx + "node \"" + nd.id + "\": ");
                    const double len = glm::length(nd.dir);
                    if(len < 1e-9) {
                        throw std::runtime_error(ctx + "node \"" + nd.id
                                                 + "\": \"dir\" must be non-zero");
                    }
                    nd.dir /= len;   // nodes mate by direction; keep them unit
                    d.nodes.push_back(nd);
                }
                /* BuildShip::fillNodeTaken keeps a taken port as one bit of a
                   uint64_t per part, so the bound belongs HERE: an authoring typo
                   should name the def in a load error, not abort the game at draw
                   time through the assert there. */
                if(d.nodes.size() > 64) {
                    throw std::runtime_error(ctx + "\"nodes\" holds "
                                             + std::to_string(d.nodes.size())
                                             + " nodes; a part def carries at most 64");
                }
            }
            d.synthesizeNodes();   // no-op if explicit nodes were given
        }

        /* One read of the mesh file feeds both records. The fingerprint is
           computed last so it covers the synthesized nodes as well as the
           authored ones (#456); the extents are the mesh's own bounding box,
           which is what a surface attach and an induced span ask for (#457).
           A virtual marker skips it: no body, no shape. */
        const MeshGeom mg = readMeshGeom(d.mesh);
        d.geom_fingerprint = partGeomFingerprint(d, mg);
        if(mg.hasExtents) {
            d.mesh_min = mg.lo;
            d.mesh_max = mg.hi;
            d.has_mesh_extents = true;
        }

        cat.parts.push_back(d);
    }
    return cat;
}

ShipDef shipDefFromJson(const nlohmann::json &doc, const PartsCatalog &catalog,
                        const std::string &path) {
    if(!doc.is_object() || !doc.contains("parts") || !doc["parts"].is_array()
       || doc["parts"].empty()) {
        throw std::runtime_error(std::string("ship: no parts in ") + path);
    }

    ShipDef def;
    def.name = doc.value("name", std::string(""));
    def.controller = -1;
    def.hull_margin = -1.0;
    if(doc.contains("hull_margin")) {
        /* Ship-level fields keep the "<thing> in <file> must be" wording they have
           always had; only the read goes through the shared helper, so a string
           answers with its type instead of nlohmann's unnamed type_error. */
        const std::string what = std::string("ship: \"hull_margin\" in ") + path;
        def.hull_margin = wantNumber(doc.at("hull_margin"), what, "m");
        if(def.hull_margin < 0.0) {
            throw std::runtime_error(what + " must be >= 0 (m)");
        }
    }
    /* "attitude": {"pitch": deg, "yaw": deg} -- a pad-only tilt (ShipDef).
       Checked rather than clamped: a pitch of 180 means the author meant
       something else, and quietly tilting a ship into the ground to find that
       out wastes a launch. Unknown keys are refused for the same reason -- a
       typo'd "pith" would otherwise read as "no tilt asked for". */
    if(doc.contains("attitude")) {
        const nlohmann::json &a = doc["attitude"];
        if(!a.is_object()) {
            throw std::runtime_error(std::string("ship: \"attitude\" in ") + path
                                     + " must be an object of \"pitch\"/\"yaw\" degrees");
        }
        for(auto it = a.begin(); it != a.end(); ++it) {
            if(it.key() != "pitch" && it.key() != "yaw") {
                throw std::runtime_error(std::string("ship: \"attitude\" has \"")
                                         + it.key() + "\" in " + path
                                         + " (known: pitch, yaw)");
            }
        }
        def.has_attitude = true;
        if(a.contains("pitch")) {
            if(!a["pitch"].is_number()) {
                throw std::runtime_error(std::string("ship: \"attitude\" \"pitch\" in ")
                                         + path + " must be a number (deg)");
            }
            def.attitude_pitch = a["pitch"].get<double>();
        }
        if(a.contains("yaw")) {
            if(!a["yaw"].is_number()) {
                throw std::runtime_error(std::string("ship: \"attitude\" \"yaw\" in ")
                                         + path + " must be a number (deg)");
            }
            def.attitude_yaw = a["yaw"].get<double>();
        }
        if(def.attitude_pitch < 0.0 || def.attitude_pitch > 180.0) {
            throw std::runtime_error(std::string("ship: \"attitude\" pitch ")
                                     + std::to_string(def.attitude_pitch)
                                     + " in " + path + " must be 0..180 deg off the local vertical");
        }
    }

    const nlohmann::json &arr = doc["parts"];
    std::map<std::string, size_t> idToIndex;   // instance id -> part index (defined so far)
    std::map<std::string, int> autoCount;      // catalog name -> auto-id counter
    for(size_t i = 0; i < arr.size(); i++) {
        const nlohmann::json &pv = arr[i];

        ShipPart sp;
        sp.def = nullptr;
        sp.part = pv.value("part", std::string(""));
        if(sp.part.empty()) {
            throw std::runtime_error(std::string("ship: part entry ") + std::to_string(i)
                                     + " of " + path + ": missing \"part\"");
        }
        sp.def = catalog.find(sp.part);
        if(sp.def == nullptr) {
            std::string avail;
            for(size_t k = 0; k < catalog.parts.size(); k++) {
                if(k) { avail += ", "; }
                avail += catalog.parts[k].name;
            }
            throw std::runtime_error(std::string("ship: unknown part '") + sp.part
                                     + "' in " + path + " (catalog has: " + avail + ")");
        }

        // Must be unique within the ship.
        sp.id = pv.value("id", std::string(""));
        if(sp.id.empty()) {
            int &n = autoCount[sp.part];
            n++;
            sp.id = sp.part + "_" + std::to_string(n);
        }
        /* The prefix every field of this entry quotes (#496), named the way this
           loader names a bad part and built once per entry. Every throw in this loop
           below uses it: re-spelling "ship: part '...' in <file>: " at fifteen sites
           is fifteen chances for the wording to drift; the #496 review found two
           spellings of it inside one entry. */
        const std::string pctx = std::string("ship: part '") + sp.id + "' in " + path + ": ";
        if(idToIndex.count(sp.id)) {
            throw std::runtime_error(pctx + "duplicate part id");
        }

        /* The parent must already be defined: construction order is the one rule
           that catches dangling refs AND cycles. Default: the previous part. */
        sp.parent = (i > 0) ? (int)(i - 1) : -1;
        if(pv.contains("parent")) {
            if(!pv["parent"].is_string()) {
                throw std::runtime_error(pctx
                                         + "\"parent\" must be a part id (string)");
            }
            std::string pid = pv["parent"].get<std::string>();
            std::map<std::string, size_t>::iterator it = idToIndex.find(pid);
            if(it == idToIndex.end()) {
                throw std::runtime_error(pctx
                                         + "parent '" + pid + "' is not defined before it");
            }
            sp.parent = (int)it->second;
        }
        /* A fuel link is virtual -- nothing hangs off it. build_ship has always
           refused this (vehicle.cpp:37-40) while BuildShip::fromShipDef
           re-rooted it to -1, so the editor showed a second root for a design
           that would not launch. Refuse it where the def is read instead, so
           both consumers see the same answer. This also catches the OMITTED
           "parent": the default is the previous entry, and real defs interleave
           link rows among parts (res/ships/heavy_asp.json). */
        if(!sp.isFuelLink() && sp.parent >= 0) {
            const ShipPart &pp = def.parts[(size_t)sp.parent];
            if(pp.isFuelLink()) {
                /* Name the rule the author has to act on: with no "parent" key
                   at all, quoting a parent they never wrote is confusing. */
                const std::string why = pv.contains("parent")
                    ? ("parent '" + pp.id + "' is a fuel link")
                    : ("no \"parent\", so it defaults to the previous entry '" + pp.id
                       + "', which is a fuel link");
                throw std::runtime_error(pctx
                                         + why
                                         + " (a fuel link cannot have parts attached)");
            }
        }

        sp.attach = AttachMode::Down;
        if(pv.contains("attach")) {
            if(!pv["attach"].is_string()) {
                throw std::runtime_error(pctx
                                         + "\"attach\" must be a string");
            }
            std::string m = pv["attach"].get<std::string>();
            if(m == "down") { sp.attach = AttachMode::Down; }
            else if(m == "up") { sp.attach = AttachMode::Up; }
            else if(m == "surface") { sp.attach = AttachMode::Surface; }
            else {
                throw std::runtime_error(pctx
                                         + "\"attach\" must be 'down', 'up', or 'surface' (got '"
                                         + m + "')");
            }
        }
        sp.angle = optNumber(pv, "angle", 0.0, pctx, "deg");
        if(!std::isfinite(sp.angle)) {
            throw std::runtime_error(pctx
                                     + "\"angle\" must be a finite number of degrees");
        }

        sp.offset = optNumber(pv, "offset", 0.0, pctx, "m");
        if(sp.offset < 0.0) {
            throw std::runtime_error(pctx
                                     + "\"offset\" must be >= 0 (m)");
        }

        /* Node ids and contacts are validated here so a typo is a load error,
           not a null deref in build_ship. */
        sp.parentNode = pv.value("parentNode", std::string(""));
        sp.childNode  = pv.value("childNode", std::string(""));
        sp.roll          = optNumber(pv, "roll", 0.0, pctx, "deg");
        sp.contactPoint  = glm::dvec3(0.0);
        sp.contactNormal = glm::dvec3(0.0);
        if(!sp.isFuelLink() && sp.parent >= 0) {
            const ShipPart &pp = def.parts[(size_t)sp.parent];
            if(sp.isStackEdge()) {
                const bool down = (sp.attach == AttachMode::Down);
                if(sp.parentNode.empty()) { sp.parentNode = down ? "bottom" : "top"; }
                if(sp.childNode.empty())  { sp.childNode  = down ? "top" : "bottom"; }
                if(pp.def && !pp.def->findNode(sp.parentNode)) {
                    throw std::runtime_error(pctx + "parentNode '" + sp.parentNode
                                             + "' is not a node of parent part '" + pp.part + "'");
                }
                if(!sp.def->findNode(sp.childNode)) {
                    throw std::runtime_error(pctx + "childNode '" + sp.childNode
                                             + "' is not a node of part '" + sp.part + "'");
                }
            }
            else if(sp.isSurfaceEdge()) {
                if(pv.contains("point") || pv.contains("normal")) {
                    sp.contactPoint  = parse_vec3(pv, "point",  pctx);
                    sp.contactNormal = parse_vec3(pv, "normal", pctx);
                } else {
                    /* The "angle" shorthand is a CLOCK ANGLE about the parent's
                       stack axis, so it needs the parent's cross-section at that
                       azimuth -- not its authored radius (#457). radius is a
                       CYLINDER radius, and for a plate it is half the SPAN: the
                       wing's 2.0 would plant a part 2 m of air above a plate
                       0.1 m thick, hanging off a surface it claims to touch.
                       Model the cross-section as the ELLIPSE bounded by the
                       mesh's own bounding box (ellipseContact). That is identical
                       to the cylinder when the parent is round, exact at a plate's
                       tip and at its face, and its normal keeps the surface-of-
                       revolution property snapSurfaceContact already assumes. A
                       bounding BOX would have been just as exact on the axes and
                       jumps the contact normal 90 deg as the azimuth crosses a
                       corner, which the 10 deg snap grid and radial symmetry
                       clones would show. Measured before writing it: none of the
                       angle-shorthand edges in the shipped defs and fixtures (51
                       of them, plus the local saves) moves its contact, scored
                       with a replica of this rule; a scan written independently
                       agrees. */
                    assert(pp.def != nullptr);   // every part resolved its def at parse
                    const double z = optNumber(pv, "z", 0.0, pctx, "m");
                    double hx, hy;
                    pp.def->meshHalfSpan(hx, hy);
                    if(hx <= 0.0 && hy <= 0.0) {
                        throw std::runtime_error(pctx + "cannot attach '" + sp.part
                                                 + "' to parent '" + pp.id + "' ('" + pp.part
                                                 + "'): its mesh has no cross-section to bolt onto"
                                                 + " (use \"point\"/\"normal\" to place the contact)");
                    }
                    const EllipseContact ec = ellipseContact(hx, hy, glm::radians(sp.angle), z);
                    sp.contactPoint  = ec.point;
                    sp.contactNormal = ec.normal;
                    /* `z` used to be checked against nothing, so a typo'd "z"
                       quietly planted a part beside the ship instead of on it. */
                    const double hz = pp.def->meshHalfLength();
                    if(std::fabs(z) > hz) {
                        throw std::runtime_error(pctx + "surface \"z\" " + std::to_string(z)
                                                 + " is off the end of parent '" + pp.id + "': "
                                                 + pp.part + "' spans z " + std::to_string(-hz)
                                                 + " to " + std::to_string(hz)
                                                 + " m (use \"point\"/\"normal\" to attach past a part)");
                    }
                }
                const double nl = glm::length(sp.contactNormal);
                if(!(nl > 1e-9)) {
                    // NaN-safe: `nl < 1e-9` is false for a NaN normal, which
                    // would otherwise ride into the pose solve.
                    throw std::runtime_error(pctx + "surface contact normal must be non-zero");
                }
                sp.contactNormal /= nl;
                if(sp.childNode.empty()) {
                    const Node *srf = sp.def->findSurfaceNode();
                    if(srf == nullptr) {
                        throw std::runtime_error(pctx + "part '" + sp.part
                                                 + "' has no surface node to attach with");
                    }
                    sp.childNode = srf->id;
                } else if(!sp.def->findNode(sp.childNode)) {
                    throw std::runtime_error(pctx + "childNode '" + sp.childNode
                                             + "' is not a node of part '" + sp.part + "'");
                }
            }
        }

        /* Stage numbers count down from launch, so they are small. The ceiling is a
           data bound, not a physics one and no longer a cost one: #517 made
           computeStaging walk the stage numbers the ship carries instead of every
           integer between the highest and the lowest, so a lone "stage": 9999 on a
           six-stage ship buys no work. It is refused because that number rewrites the
           flight sequence rather than because it is slow: the stage counter starts at
           the highest stage any part carries (vehicle.cpp:1650-1658), so the outlier
           fires its decoupler on the first stage press and the real stages sit
           thousands of presses behind it. shipdef.h's kMaxStage and save.cpp's
           validateShipParts spell the whole harm out; the VAB stage box and
           validateShipParts apply the same ceiling so nothing creates what this
           refuses. */
        sp.stage = optInt(pv, "stage", 1, pctx);
        if(sp.stage < 1) {
            throw std::runtime_error(pctx
                                     + "\"stage\" must be >= 1 (stages count down from launch)");
        }
        if(sp.stage > kMaxStage) {
            throw std::runtime_error(pctx + "\"stage\" must be <= " + std::to_string(kMaxStage)
                                     + " (a stage number outside the ship's sequence"
                                     + " becomes its top stage)");
        }

        /* TRIM (#492), degrees of standing control-surface deflection. Rejected
           rather than ignored when the part cannot use one: a "trim" on a wing or a
           fuel link would otherwise load clean and do nothing, which reads as the
           feature being broken. Rejected past the part's own travel for the same
           reason -- the law could clamp it, but an author who wrote 90 into a 20 deg
           surface wrote the wrong number. */
        sp.trim = optNumber(pv, "trim", 0.0, pctx, "deg");
        if(!std::isfinite(sp.trim)) {
            // The JSON parser already refuses a literal it cannot represent as a
            // number, so this only bites if a value starts arriving from somewhere
            // else. Cheap, and a NaN would sail through the fabs() test below.
            throw std::runtime_error(pctx
                                     + "\"trim\" must be a finite number of degrees");
        }
        if(sp.trim != 0.0) {
            if(sp.def == nullptr || !sp.def->isControlSurface()) {
                throw std::runtime_error(pctx
                                         + "\"trim\" needs a control surface"
                                         + " (a part with control_area and max_deflection)");
            }
            const double limit = glm::degrees(sp.def->max_deflection);
            if(std::fabs(sp.trim) > limit) {
                // Printed, not cast to int: the shipped surfaces travel 20.05 deg,
                // and an author told "20" would be refused again for writing 20.1.
                char got[32], lim[32];
                snprintf(got, sizeof got, "%.2f", sp.trim);
                snprintf(lim, sizeof lim, "%.2f", limit);
                throw std::runtime_error(pctx
                                         + "\"trim\" " + got + " deg exceeds this part's "
                                         + lim + " deg of travel");
            }
        }

        /* A fuel link's from/to are required, distinct, and not its own id.
           parent/attach/angle/offset/stage are ignored. */
        if(sp.isFuelLink()) {
            sp.from = pv.value("from", std::string(""));
            sp.to = pv.value("to", std::string(""));
            if(sp.from.empty() || sp.to.empty()) {
                throw std::runtime_error(pctx
                                         + "a fuel link needs \"from\" and \"to\"");
            }
            if(sp.from == sp.to) {
                throw std::runtime_error(pctx
                                         + "a fuel link's \"from\" and \"to\" must differ");
            }
            if(sp.from == sp.id || sp.to == sp.id) {
                throw std::runtime_error(pctx
                                         + "a fuel link cannot reference itself");
            }
        }

        idToIndex[sp.id] = i;
        def.parts.push_back(sp);
    }

    // Resolved now that all ids are known.
    /* A fuel link's endpoints have to name PHYSICAL parts. build_ship already
       refuses an endpoint that names nothing (vehicle.cpp, "references unknown
       part"), while the staging snapshot and the editor's loop check skip one
       silently -- so a typo was a design that showed a sane tree in the editor
       and refused to launch. Same boundary as the parent check above; links may
       point FORWARD, so this waits for every id. */
    for(size_t i = 0; i < def.parts.size(); i++) {
        const ShipPart &lk = def.parts[i];
        if(!lk.isFuelLink()) { continue; }
        const std::string endp[2] = { lk.from, lk.to };
        for(int side = 0; side < 2; side++) {
            const char *key = side ? "to" : "from";
            std::map<std::string, size_t>::iterator it = idToIndex.find(endp[side]);
            if(it == idToIndex.end()) {
                throw std::runtime_error(std::string("ship: fuel link '") + lk.id + "' in " + path
                                         + ": \"" + key + "\" '" + endp[side]
                                         + "' is not a part of the ship");
            }
            if(def.parts[it->second].isFuelLink()) {
                throw std::runtime_error(std::string("ship: fuel link '") + lk.id + "' in " + path
                                         + ": \"" + key + "\" '" + endp[side]
                                         + "' is itself a fuel link");
            }
        }
    }
    if(doc.contains("controller")) {
        if(!doc["controller"].is_string()) {
            throw std::runtime_error(std::string("ship: \"controller\" in ") + path
                                     + " must be a part id (string)");
        }
        std::string cid = doc["controller"].get<std::string>();
        std::map<std::string, size_t>::iterator it = idToIndex.find(cid);
        if(it == idToIndex.end()) {
            throw std::runtime_error(std::string("ship: controller id '") + cid + "' in " + path
                                     + " is not a part of the ship");
        }
        /* Same class as the parent check: build_ship refuses a fuel link here
           (vehicle.cpp, "controller is a fuel link") while fromShipDef just
           records the id, which toShipDef later drops -- so the def loaded, the
           editor looked fine, and the ship flew on the DEFAULT controller. */
        if(def.parts[it->second].isFuelLink()) {
            throw std::runtime_error(std::string("ship: controller id '") + cid + "' in " + path
                                     + " is a fuel link (a fuel link cannot fly the ship)");
        }
        def.controller = (int)it->second;
    }
    /* A ship is ONE Body, so its parts' masses ADD -- and the per-def capacity
       gate above can only ever see one part. Measured in #547: two parts at
       1e308 each give Bullet an infinite compound mass, and
       calculatePrincipalAxisTransform answers that with a NaN inertia tensor and a
       NaN centre of mass. Not a crash but garbage: the ship would fly with an
       unrotatable or exploding body rather than tell anybody anything. Same stance
       as that gate and as optInt's counts -- no invented ceiling, just that what
       the game cannot represent does not load. The shipped fleet is 300 orders of
       magnitude away from this (measured in #547: heaviest def 1.06e5 kg), so it only
       ever refuses a hand-written file. */
    double launchMass = 0.0;
    for(const ShipPart &sp : def.parts) {
        if(sp.isFuelLink()) { continue; }   // an edge, not a body
        /* Every part has a def here -- an unknown name threw in the loop above -- and
           partLoadMass is at least the dry mass, which the parser requires to be > 0,
           so the running total only climbs and the first part that tips it is the one
           worth naming. */
        launchMass += partLoadMass(*sp.def);
        if(!std::isfinite(launchMass)) {
            throw std::runtime_error(std::string("ship: ") + path
                                     + ": its parts together weigh more than the game"
                                     + " can store (part '" + sp.part
                                     + "' takes the running total past it)");
        }
    }
    return def;
}

ShipDef load_ship_def(const char *path, const PartsCatalog &catalog) {
    std::ifstream f(path);
    if(!f.is_open()) {
        throw std::runtime_error(std::string("ship: cannot open ") + path);
    }
    nlohmann::json doc;
    try {
        doc = nlohmann::json::parse(f, nullptr, true);
    } catch(const std::exception &e) {
        throw std::runtime_error(std::string("ship: bad JSON in ") + path
                                 + std::string(": ") + e.what());
    }
    return shipDefFromJson(doc, catalog, path);
}

/* Shortest-arc rotation taking unit `a` onto unit `b`; the roll about that arc
   is the caller's to resolve. The anti-parallel case picks a deterministic
   perpendicular. */
static glm::dmat3 rotationFromTo(const glm::dvec3 &a, const glm::dvec3 &b) {
    const double d = glm::clamp(glm::dot(a, b), -1.0, 1.0);
    if(d > 1.0 - 1e-12) { return glm::dmat3(1.0); }   // already aligned
    if(d < -1.0 + 1e-12) {                            // opposed: 180 about a perpendicular
        const glm::dvec3 axis = glm::normalize(
            (std::fabs(a.x) < 0.9) ? glm::cross(a, glm::dvec3(1.0, 0.0, 0.0))
                                   : glm::cross(a, glm::dvec3(0.0, 1.0, 0.0)));
        return glm::mat3_cast(glm::angleAxis(std::acos(-1.0), axis));
    }
    const glm::dvec3 axis = glm::normalize(glm::cross(a, b));
    return glm::mat3_cast(glm::angleAxis(std::acos(d), axis));
}

/* The child's pose in the shared frame. `rrel` is the child frame relative to
   the parent frame; the authored roll goes about the mating axis (-dP). The
   node positions coincide, pushed apart by `offset` along the parent node dir. */
static AttachPose matePose(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                           const glm::dvec3 &parentLocalPos, const glm::dvec3 &dP,
                           const Node &childNode, const glm::dmat3 &rrel,
                           double rollDeg, double offset)
{
    glm::dmat3 rot = rrel;
    if(rollDeg != 0.0) {
        rot = glm::mat3_cast(glm::angleAxis(glm::radians(rollDeg), -dP)) * rot;
    }
    AttachPose p;
    p.childRot = parentRot * rot;
    const glm::dvec3 contact = parentPos + parentRot * (parentLocalPos + dP * offset);
    p.childPos = contact - p.childRot * childNode.pos;
    return p;
}

AttachPose attachNodes(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                       const Node &parentNode, const Node &childNode,
                       double rollDeg, double offset)
{
    const glm::dvec3 dP = glm::normalize(parentNode.dir);   // parent-local outward
    const glm::dvec3 dC = glm::normalize(childNode.dir);    // child-local outward
    /* Stack ports are authored square, so the shortest arc is the whole
       story: the roll about it is authored (`rollDeg`), not implied. */
    return matePose(parentPos, parentRot, parentNode.pos, dP, childNode,
                    rotationFromTo(dC, -dP), rollDeg, offset);
}

static glm::dvec3 perpToward(const glm::dvec3 &d, const glm::dvec3 &pref)
{
    glm::dvec3 u = pref - glm::dot(pref, d) * d;
    if(glm::dot(u, u) < 1e-12) {
        u = glm::cross(d, (std::fabs(d.x) < 0.9) ? glm::dvec3(1.0, 0.0, 0.0)
                                                 : glm::dvec3(0.0, 1.0, 0.0));
    }
    return glm::normalize(u);
}

/* SURFACE-edge mating rotation, in the parent's local frame, with the ROLL
   pinned to the stack axes instead of left to the shortest arc: the shortest
   arc's incidental roll depends on where the target sits on the sphere, so
   sliding the contact around a cone twists the part and flips it 180 deg on the
   far side. Parts are authored +Z = stack axis, so the child's +Z stays parallel
   to the parent's +Z projected into the contact plane -- deterministic, and
   independent of how the contact got where it is.
   Both up-references must be COVARIANT with a rotation about the parent's long
   axis, or radialSymmetryClones' ring is not congruent with the primary. */
static glm::dmat3 surfaceMatingRot(const glm::dvec3 &dC, const glm::dvec3 &target,
                                   const glm::dvec3 &point)
{
    const glm::dvec3 Z(0.0, 0.0, 1.0);
    /* Parent side: the parent's stack axis projected into the contact plane. At
       an axial contact that projection vanishes; its limit is minus the
       contact's own radial direction -- continuous through the shoulder and
       covariant about the axis. At a cap centre even that vanishes, and any
       deterministic perpendicular will do. */
    glm::dvec3 upP = Z - glm::dot(Z, target) * target;
    if(glm::dot(upP, upP) < 1e-12) {
        const glm::dvec3 radial = point - glm::dot(point, target) * target;
        upP = (glm::dot(radial, radial) < 1e-12)
            ? perpToward(target, glm::dvec3(1.0, 0.0, 0.0))
            : -radial;
    }
    upP = glm::normalize(upP);
    /* Child side: the choice is arbitrary when the node dir is along the
       child's own axis, but it depends only on dC -- so every clone of a
       symmetry ring makes the same one and the ring stays congruent. */
    const glm::dvec3 upC = perpToward(dC, Z);
    /* Orthonormal frames (node dir, up, dir x up) as columns. Both are
       right-handed, so F * E^T is a rotation rather than a reflection. */
    const glm::dmat3 E(dC, upC, glm::cross(dC, upC));
    const glm::dmat3 F(target, upP, glm::cross(target, upP));
    return F * glm::transpose(E);
}

AttachPose attachSurface(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                         const glm::dvec3 &point, const glm::dvec3 &normal,
                         const Node &childNode, double rollDeg, double offset)
{
    /* Surface attach IS node mating: a synthetic parent node at the contact
       (pos = the contact point, dir = the outward normal). Only the roll rule
       differs from a stack port: surfaceMatingRot, not the shortest arc. */
    assert(glm::length(normal) > 1e-9 && glm::length(childNode.dir) > 1e-9);
    const glm::dvec3 dP = glm::normalize(normal);
    const glm::dvec3 dC = glm::normalize(childNode.dir);
    const glm::dvec3 target = -dP;      // the child's node dir opposes the normal
    return matePose(parentPos, parentRot, point, dP, childNode,
                    surfaceMatingRot(dC, target, point), rollDeg, offset);
}

EllipseContact ellipseContact(double hx, double hy, double azimuthRad, double z) {
    const double ca = std::cos(azimuthRad), sa = std::sin(azimuthRad);
    EllipseContact out;
    /* A round parent is the circle this rule always used. Kept as an equality
       test rather than falling into the general form below: that form's sqrt is
       not bit-identical to hx*(ca, sa), and every shipped surface contact -- all
       of them on round parents -- rides the difference. */
    if(std::fabs(hx - hy) <= 1e-9 * std::max(hx, hy)) {
        out.point = glm::dvec3(hx * ca, hx * sa, z);
        out.normal = glm::dvec3(ca, sa, 0.0);
        return out;
    }
    /* Where the ray at azimuth `a` meets the ellipse x^2/hx^2 + y^2/hy^2 = 1,
       written with no division by a half-span. The obvious 1/sqrt(ca^2/hx^2 +
       sa^2/hy^2) overflows for a thin parent -- a plate modelled with no
       thickness has hy = 0, sa^2/hy^2 = inf, and the part lands at NaN with a
       pose that solves and a ship that cannot be flown. The scaled form degrades
       instead: a zero-thickness plate meets every ray on its own face. */
    const double t = (ca == 0.0) ? hy
                   : (sa == 0.0) ? hx
                   : (hx * hy) / std::sqrt(ca * ca * hy * hy + sa * sa * hx * hx);
    out.point = glm::dvec3(t * ca, t * sa, z);
    /* The gradient of that ellipse, scaled by (hx*hy)^2 for the same reason: it
       points off the NEAREST face of the curve rather than off the diagonal, so
       the normal's azimuth keeps tracking the contact's azimuth -- the surface-
       of-revolution property snapSurfaceContact's 10 deg grid and the radial
       symmetry clones both assume. */
    glm::dvec3 n(ca * hy * hy, sa * hx * hx, 0.0);
    const double n2 = n.x * n.x + n.y * n.y;
    if(n2 > 0.0) {
        out.normal = n * (1.0 / std::sqrt(n2));
    } else {
        out.normal = glm::dvec3(ca, sa, 0.0);   // a mesh with no area at all
    }
    return out;
}

double snapAngleDeg(double deg) {
    return std::round(deg / kSnapAngDeg) * kSnapAngDeg;
}

double gridStepDeg(double cur, double delta) {
    if(delta > 0.0) {
        return std::floor(cur / kSnapAngDeg + 1e-9) * kSnapAngDeg + kSnapAngDeg;
    }
    return std::ceil(cur / kSnapAngDeg - 1e-9) * kSnapAngDeg - kSnapAngDeg;
}

/* degrees wrapped to [-180, 180) */
static double wrap180Deg(double deg) {
    double d = std::fmod(deg + 180.0, 360.0);
    if(d < 0.0) { d += 360.0; }
    return d - 180.0;
}

void snapSurfaceContact(glm::dvec3 &point, glm::dvec3 &normal,
                        bool snapLen, bool snapAng)
{
    const glm::dvec3 Z(0.0, 0.0, 1.0);
    if(snapLen) {
        point.z = std::round(point.z / kSnapLenM) * kSnapLenM;
    }
    if(!snapAng) { return; }
    const double r = std::hypot(point.x, point.y);
    const bool havePt = (r > 1e-4);
    double pa = 0.0, pa2 = 0.0;
    if(havePt) {
        pa = std::atan2(point.y, point.x);
        pa2 = glm::radians(snapAngleDeg(glm::degrees(pa)));
        point = glm::mat3_cast(glm::angleAxis(pa2 - pa, Z)) * point;
    }
    /* The pick hull is faceted, so a facet's normal stays constant while the hit
       POSITION sweeps degrees across it. Parts are surfaces of revolution, where
       the true normal azimuth equals the contact azimuth: when the two raw
       azimuths agree, the normal takes the point's SNAPPED azimuth (keeping its
       polar tilt). Only a genuinely non-revolution contact snaps its own azimuth. */
    const double rn = std::hypot(normal.x, normal.y);
    if(rn > 1e-6) {
        const double an = std::atan2(normal.y, normal.x);
        double target;
        if(havePt
           && std::fabs(wrap180Deg(glm::degrees(an) - glm::degrees(pa))) < 20.0) {
            target = pa2;
        } else {
            target = glm::radians(snapAngleDeg(glm::degrees(an)));
        }
        normal = glm::mat3_cast(glm::angleAxis(target - an, Z)) * normal;
    }
}

std::vector<SymClone> radialSymmetryClones(const glm::dvec3 &parentPos,
                                           const glm::dmat3 &parentRot,
                                           const Node &childNode,
                                           const glm::dvec3 &point,
                                           const glm::dvec3 &normal,
                                           double rollDeg, double offset,
                                           int n)
{
    std::vector<SymClone> out;
    if(n <= 1) { return out; }
    const glm::dvec3 axisL(0.0, 0.0, 1.0);          // the parent's long axis
    const double step = 2.0 * std::acos(-1.0) / (double)n;
    for(int k = 1; k < n; k++) {
        const double th = step * (double)k;
        const glm::dmat3 RzL = glm::mat3_cast(glm::angleAxis(th, axisL));
        SymClone c;
        c.edge.point  = RzL * point;
        c.edge.normal = RzL * normal;
        c.edge.rollDeg = rollDeg;
        /* Congruent by construction, no residual roll to fold in: RzL fixes the
           parent's axis, so it carries the whole solution -- target, upP and
           target x upP all rotate with it, E depends only on the child's node
           dir, and the roll about -dP commutes the same way. */
        c.pose = attachSurface(parentPos, parentRot, c.edge.point,
                               c.edge.normal, childNode, c.edge.rollDeg, offset);
        out.push_back(c);
    }
    return out;
}

AttachPose attachPose(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                      const PartDef &parentDef, const PartDef &childDef,
                      AttachMode mode, double angleDeg, double offset)
{
    // attachNodes is the single source of stack-attach geometry.
    if(mode != AttachMode::Down && mode != AttachMode::Up) {
        throw std::runtime_error("attachPose: only stack modes (down/up); a "
                                 "surface edge uses attachSurface(point + normal)");
    }
    const bool down = (mode == AttachMode::Down);
    const Node *pn = parentDef.findNode(down ? "bottom" : "top");
    const Node *cn = childDef.findNode(down ? "top" : "bottom");
    if(pn == nullptr || cn == nullptr) {
        throw std::runtime_error(std::string("attachPose: ")
                                 + (pn == nullptr ? parentDef.name : childDef.name)
                                 + " has no axial stack node (a part that declares "
                                   "explicit nodes must include top/bottom to be "
                                   "stacked with attach down/up)");
    }
    return attachNodes(parentPos, parentRot, *pn, *cn, angleDeg, offset);
}

AttachPose solveEdge(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                     const PartDef &parentDef, const PartDef &childDef,
                     AttachMode mode,
                     const std::string &parentNode, const std::string &childNode,
                     const glm::dvec3 &contactPoint, const glm::dvec3 &contactNormal,
                     double angleDeg, double rollDeg, double offset)
{
    if(mode == AttachMode::Down || mode == AttachMode::Up) {
        const Node *pn = parentDef.findNode(parentNode);
        const Node *cn = childDef.findNode(childNode);
        if(pn == nullptr || cn == nullptr) {
            throw std::runtime_error(std::string("solveEdge: stack edge references a "
                                                 "missing node (parent '") + parentNode
                                     + "' / child '" + childNode + "')");
        }
        return attachNodes(parentPos, parentRot, *pn, *cn, angleDeg, offset);
    }
    // surface edge
    const Node *cn = childDef.findNode(childNode);
    if(cn == nullptr) {
        throw std::runtime_error(std::string("solveEdge: surface edge references a "
                                             "missing child node '") + childNode + "'");
    }
    return attachSurface(parentPos, parentRot, contactPoint, contactNormal,
                         *cn, rollDeg, offset);
}

void BuildShip::recomputePoses() {
    for(size_t i = 0; i < parts.size(); i++) {
        BuildPart &bp = parts[i];
        if(bp.parent < 0) {
            bp.localPos = glm::dvec3(0.0);
            bp.localRot = glm::dmat3(1.0);
            continue;
        }
        /* Construction order: a parent must precede its child. Solving against a
           later index would silently use that part's STALE pose (or run off the
           end), so this is a tripwire, not a fix. */
        assert(bp.parent < (int)i &&
               "BuildShip part must come after its parent");
        const BuildPart &pp = parts[(size_t)bp.parent];
        const AttachPose ap = solveEdge(pp.localPos, pp.localRot,
                                        *pp.def, *bp.def, bp.attach,
                                        bp.parentNode, bp.childNode,
                                        bp.contactPoint, bp.contactNormal,
                                        bp.angle, bp.roll, bp.offset);
        bp.localPos = ap.childPos;
        bp.localRot = ap.childRot;
    }
}

bool BuildShip::nodeOccupied(int partIdx, const std::string &nodeId) const {
    for(size_t i = 0; i < parts.size(); i++) {
        const BuildPart &p = parts[i];
        if(p.attach == AttachMode::Surface) { continue; }
        if(p.parent == partIdx && p.parentNode == nodeId) { return true; }
        if((int)i == partIdx && p.childNode == nodeId) { return true; }
    }
    return false;
}

/* nodeOccupied's two halves, applied to every part at once (#542). Each stack edge
   contributes exactly two bits: the node it consumes on its parent, and the node it
   consumes on itself. A surface edge consumes no stack port, which is why the whole
   edge is skipped here and in nodeOccupied. The node id -> index lookups are short
   scans of a def's own node list (three nodes today), so the pass is linear in parts. */
void BuildShip::fillNodeTaken(std::vector<uint64_t> &out) const {
    out.assign(parts.size(), 0);
    for(size_t i = 0; i < parts.size(); i++) {
        const BuildPart &p = parts[i];
        /* Every def in the build, not only the ones this pass writes: a
           surface-attached part is skipped below, yet it can still be a stack
           PARENT, and the parent half shifts by an index into its node list. */
        assert((p.def == nullptr || p.def->nodes.size() <= 64) &&
               "a part def needs <= 64 nodes for the taken mask");
        if(p.attach == AttachMode::Surface) { continue; }
        if(p.def != nullptr) {
            /* No break: nodeOccupied matches by id, so every node carrying that id
               must read taken. The parse refuses duplicate node ids, so today this is
               form rather than behaviour, and it costs about +1 us/frame at 268 parts
               on a stacked column (the compares the break used to skip) against a loop
               that already saved 33 us there. */
            for(size_t k = 0; k < p.def->nodes.size(); k++) {
                if(p.def->nodes[k].id == p.childNode) { out[i] |= (1ull << k); }
            }
        }
        const int pi = p.parent;
        if(pi < 0 || (size_t)pi >= parts.size()) { continue; }
        const BuildPart &par = parts[(size_t)pi];
        if(par.def == nullptr) { continue; }
        for(size_t k = 0; k < par.def->nodes.size(); k++) {
            if(par.def->nodes[k].id == p.parentNode) { out[(size_t)pi] |= (1ull << k); }
        }
    }
}

BuildShip BuildShip::fromShipDef(const ShipDef &def) {
    BuildShip bs;
    bs.name = def.name;
    bs.hull_margin = def.hull_margin;
    bs.has_attitude = def.has_attitude;
    bs.attitude_pitch = def.attitude_pitch;
    bs.attitude_yaw = def.attitude_yaw;
    if(def.controller >= 0 && (size_t)def.controller < def.parts.size()) {
        bs.controllerId = def.parts[(size_t)def.controller].id;
    }
    /* Fuel links leave the tree but stay as records for the round trip. */
    std::map<size_t, size_t> physIndex;   // def.parts index -> build index
    for(size_t i = 0; i < def.parts.size(); i++) {
        const ShipPart &sp = def.parts[i];
        if(sp.isFuelLink()) {
            bs.fuelLinks.push_back(FuelLink{sp.def, sp.id, sp.from, sp.to});
            continue;
        }
        physIndex[i] = bs.parts.size();
        BuildPart bp;
        bp.def = sp.def;
        bp.id = sp.id;
        bp.attach = sp.attach;
        bp.parentNode = sp.parentNode;
        bp.childNode = sp.childNode;
        bp.contactPoint = sp.contactPoint;
        bp.contactNormal = sp.contactNormal;
        bp.angle = sp.angle;
        bp.roll = sp.roll;
        bp.trim = sp.trim;
        bp.offset = sp.offset;
        bp.stage = sp.stage;
        if(sp.parent >= 0) {
            auto it = physIndex.find((size_t)sp.parent);
            /* load_ship_def refuses a part parented to a fuel link (see the
               check in the parser), so a link index reaching here is a def that
               never went through it. Used to re-root to -1, which quietly made
               the part a second root -- a different ship from the one
               build_ship would have built. */
            assert(it != physIndex.end());
            bp.parent = (int)it->second;
        }
        bs.parts.push_back(bp);
    }
    bs.recomputePoses();
    return bs;
}

/* Construction order puts every parent before its children, so one
   forward pass suffices. */
static std::vector<bool> subtreeMask(const std::vector<BuildPart> &parts,
                                     int idx) {
    std::vector<bool> sub(parts.size(), false);
    sub[(size_t)idx] = true;
    for(size_t i = 1; i < parts.size(); i++) {
        if(parts[i].parent >= 0 && sub[(size_t)parts[i].parent]) { sub[i] = true; }
    }
    return sub;
}

bool BuildShip::removePart(int idx) {
    if(idx <= 0 || idx >= (int)parts.size()) { return false; }   // root stays
    const std::vector<bool> dead = subtreeMask(parts, idx);
    std::vector<int> remap(parts.size(), -1);
    std::vector<BuildPart> keep;
    keep.reserve(parts.size());
    for(size_t i = 0; i < parts.size(); i++) {
        if(dead[i]) { continue; }
        remap[i] = (int)keep.size();
        keep.push_back(parts[i]);
    }
    for(size_t k = 0; k < keep.size(); k++) {
        if(keep[k].parent >= 0) { keep[k].parent = remap[(size_t)keep[k].parent]; }
    }
    parts.swap(keep);
    /* purge fuel links whose endpoint died with the subtree */
    std::vector<FuelLink> liveLinks;
    for(size_t i = 0; i < fuelLinks.size(); i++) {
        const FuelLink &fl = fuelLinks[i];
        bool haveFrom = false, haveTo = false;
        for(size_t k = 0; k < parts.size(); k++) {
            if(parts[k].id == fl.from) { haveFrom = true; }
            if(parts[k].id == fl.to)   { haveTo = true; }
        }
        if(haveFrom && haveTo) { liveLinks.push_back(fl); }
    }
    fuelLinks.swap(liveLinks);
    recomputePoses();
    return true;
}

BuildShip BuildShip::detachSubtree(int idx) {
    BuildShip out;
    if(idx <= 0 || idx >= (int)parts.size()) { return out; }   // root stays
    const std::vector<bool> sub = subtreeMask(parts, idx);

    // idx comes first in construction order, so it becomes the new root.
    out.name = name;
    out.hull_margin = hull_margin;
    std::map<size_t, size_t> toNew;   // old index -> out index
    for(size_t i = 0; i < parts.size(); i++) {
        if(!sub[i]) { continue; }
        toNew[i] = out.parts.size();
        BuildPart bp = parts[i];
        if((int)i == idx) {
            bp.parent = -1;   // the new root: its edge is re-made at graft
            bp.attach = AttachMode::Down;
            bp.parentNode.clear();
            bp.childNode.clear();
            bp.contactPoint = glm::dvec3(0.0);
            bp.contactNormal = glm::dvec3(0.0);
            bp.angle = 0.0;
            bp.roll = 0.0;
            bp.offset = 0.0;
        } else {
            bp.parent = (int)toNew[(size_t)parts[i].parent];
        }
        out.parts.push_back(bp);
    }

    /* Both endpoints inside -> moves with the assembly; boundary-crossing ->
       dropped (a detached pipe feeds nothing). */
    std::vector<FuelLink> stay;
    for(size_t i = 0; i < fuelLinks.size(); i++) {
        const FuelLink &fl = fuelLinks[i];
        bool fromIn = false, toIn = false, fromKnown = false, toKnown = false;
        for(size_t k = 0; k < parts.size(); k++) {
            if(parts[k].id == fl.from) { fromKnown = true; fromIn = sub[k]; }
            if(parts[k].id == fl.to)   { toKnown = true;   toIn = sub[k]; }
        }
        if(fromIn && toIn) { out.fuelLinks.push_back(fl); }
        else if(fromKnown && toKnown && !fromIn && !toIn) { stay.push_back(fl); }
    }
    fuelLinks.swap(stay);

    // the explicit controller follows its part
    if(!controllerId.empty()) {
        for(size_t i = 0; i < parts.size(); i++) {
            if(parts[i].id == controllerId && sub[i]) {
                out.controllerId = controllerId;
                controllerId.clear();
                break;
            }
        }
    }

    // Survivors: the same remove + remap pass as removePart.
    std::vector<int> remap(parts.size(), -1);
    std::vector<BuildPart> keep;
    keep.reserve(parts.size());
    for(size_t i = 0; i < parts.size(); i++) {
        if(sub[i]) { continue; }
        remap[i] = (int)keep.size();
        keep.push_back(parts[i]);
    }
    for(size_t k = 0; k < keep.size(); k++) {
        if(keep[k].parent >= 0) { keep[k].parent = remap[(size_t)keep[k].parent]; }
    }
    parts.swap(keep);
    recomputePoses();
    out.recomputePoses();
    return out;
}

namespace {
std::string uniquePartId(const std::vector<BuildPart> &parts,
                         const std::string &base) {
    std::string id = base;
    for(int n = 2;; n++) {
        bool dup = false;
        for(size_t i = 0; i < parts.size(); i++) {
            if(parts[i].id == id) { dup = true; break; }
        }
        if(!dup) { return id; }
        id = base + "_" + std::to_string(n);
    }
}
std::string uniqueLinkId(const std::vector<BuildShip::FuelLink> &links,
                         const std::string &base) {
    std::string id = base;
    for(int n = 2;; n++) {
        bool dup = false;
        for(size_t i = 0; i < links.size(); i++) {
            if(links[i].id == id) { dup = true; break; }
        }
        if(!dup) { return id; }
        id = base + "_" + std::to_string(n);
    }
}
} // namespace

size_t BuildShip::graftTree(const BuildShip &sub, const BuildPart &root) {
    if(sub.parts.empty()) { return (size_t)-1; }
    std::map<std::string, std::string> idMap;   // sub id -> grafted id
    const size_t rootIdx = parts.size();

    BuildPart rp = root;
    rp.id = uniquePartId(parts, root.id.empty()
                         ? std::string(sub.parts[0].def != nullptr
                                       ? sub.parts[0].def->name : "part")
                         : root.id);
    idMap[sub.parts[0].id] = rp.id;
    parts.push_back(rp);
    for(size_t i = 1; i < sub.parts.size(); i++) {
        BuildPart bp = sub.parts[i];
        const std::string newId = uniquePartId(parts, bp.id);
        idMap[bp.id] = newId;
        bp.id = newId;
        // sub is in construction order, so every parent index maps by the
        // same offset (the assembly root lands on rootIdx). That offset only
        // works if sub has ONE root, at index 0: a second -1 would offset into
        // the host's last part instead of the grafted root.
        assert(bp.parent >= 0 && "a subassembly has exactly one root, at index 0");
        bp.parent = (int)(rootIdx + (size_t)bp.parent);
        parts.push_back(bp);
    }
    for(size_t i = 0; i < sub.fuelLinks.size(); i++) {
        const FuelLink &fl = sub.fuelLinks[i];
        FuelLink nl = fl;
        nl.id = uniqueLinkId(fuelLinks, fl.id);
        std::map<std::string, std::string>::const_iterator f = idMap.find(fl.from);
        std::map<std::string, std::string>::const_iterator t = idMap.find(fl.to);
        nl.from = (f != idMap.end()) ? f->second : fl.from;
        nl.to   = (t != idMap.end()) ? t->second : fl.to;
        fuelLinks.push_back(nl);
    }
    recomputePoses();
    return rootIdx;
}

void BuildShip::rotatePart(int idx, double deltaDeg) {
    if(idx <= 0 || idx >= (int)parts.size()) { return; }   // root has no edge
    BuildPart &bp = parts[(size_t)idx];
    if(bp.parent < 0) { return; }
    if(bp.attach == AttachMode::Surface) { bp.roll += deltaDeg; }
    else { bp.angle += deltaDeg; }
    recomputePoses();
}

ShipDef BuildShip::toShipDef() const {
    ShipDef def;
    def.name = name;
    def.hull_margin = hull_margin;
    def.has_attitude = has_attitude;
    def.attitude_pitch = attitude_pitch;
    def.attitude_yaw = attitude_yaw;
    def.controller = -1;
    for(size_t i = 0; i < parts.size(); i++) {
        const BuildPart &bp = parts[i];
        ShipPart sp{};
        sp.part = (bp.def != nullptr) ? bp.def->name : std::string("");
        sp.id = bp.id;
        sp.def = bp.def;
        sp.parent = bp.parent;
        sp.attach = bp.attach;
        sp.angle = bp.angle;
        sp.offset = bp.offset;
        sp.parentNode = bp.parentNode;
        sp.childNode = bp.childNode;
        sp.contactPoint = bp.contactPoint;
        sp.contactNormal = bp.contactNormal;
        sp.roll = bp.roll;
        sp.trim = bp.trim;
        sp.stage = bp.stage;
        if(!controllerId.empty() && bp.id == controllerId) { def.controller = (int)i; }
        def.parts.push_back(sp);
    }
    /* A link whose endpoint part was deleted is dropped. */
    for(size_t k = 0; k < fuelLinks.size(); k++) {
        const FuelLink &fl = fuelLinks[k];
        bool haveFrom = false, haveTo = false;
        for(size_t i = 0; i < parts.size(); i++) {
            if(parts[i].id == fl.from) { haveFrom = true; }
            if(parts[i].id == fl.to)   { haveTo = true; }
        }
        if(!haveFrom || !haveTo) { continue; }
        ShipPart sp{};
        sp.part = (fl.def != nullptr) ? fl.def->name : std::string("fuel_link");
        sp.id = fl.id;
        sp.def = fl.def;
        sp.parent = -1;
        sp.attach = AttachMode::Down;   // ignored for a link
        sp.stage = 1;
        sp.from = fl.from;
        sp.to = fl.to;
        def.parts.push_back(sp);
    }
    return def;
}

/* See shipdef.h. Adjacency over the tree, then a flood per seed: the shape all
   the consumers share. */
int partitionFuelGroups(const std::vector<int> &parent,
                        const std::vector<bool> &wall, std::vector<int> &group) {
    const size_t n = parent.size();
    assert(wall.size() == n);   // one tree, one wall flag per part
    group.assign(n, -1);
    std::vector<std::vector<size_t> > adj(n);
    for(size_t i = 0; i < n; i++) {
        const int p = parent[i];
        if(p < 0) { continue; }
        /* A parent outside the tree is a bug, not a state. No build config here
           defines NDEBUG, so this trips in debug AND in the release binary. */
        assert(p < (int)n);
        adj[i].push_back((size_t)p);
        adj[(size_t)p].push_back(i);
    }
    int next = 0;
    for(size_t i = 0; i < n; i++) {
        if(wall[i] || group[i] != -1) { continue; }
        group[i] = next;
        std::vector<size_t> stack;
        stack.push_back(i);
        while(!stack.empty()) {
            const size_t q = stack.back();
            stack.pop_back();
            for(size_t k = 0; k < adj[q].size(); k++) {
                const size_t r = adj[q][k];
                if(wall[r] || group[r] != -1) { continue; }
                group[r] = next;
                stack.push_back(r);
            }
        }
        next++;
    }
    return next;
}

namespace {

struct FuelEdge { int to; size_t link; };

/* White/grey/black DFS over the group graph. `trail` holds the link indices on
   the current grey path and `entry[g]` is where group g joined it, so a back
   edge can name the whole loop it closes (not just itself). */
bool fuelLoopDfs(int u, const std::vector<std::vector<FuelEdge> > &out,
                 std::vector<int> &color, std::vector<int> &entry,
                 std::vector<size_t> &trail, std::vector<size_t> &loop) {
    color[(size_t)u] = 1;
    entry[(size_t)u] = (int)trail.size();
    for(size_t k = 0; k < out[(size_t)u].size(); k++) {
        const FuelEdge &e = out[(size_t)u][k];
        if(color[(size_t)e.to] == 1) {
            loop.assign(trail.begin() + (size_t)entry[e.to], trail.end());
            loop.push_back(e.link);
            return true;
        }
        if(color[(size_t)e.to] == 0) {
            trail.push_back(e.link);
            if(fuelLoopDfs(e.to, out, color, entry, trail, loop)) { return true; }
            trail.pop_back();
        }
    }
    color[(size_t)u] = 2;
    return false;
}

}  // namespace

std::string BuildShip::fuelLinkLoop(const FuelLink *extra) const {
    std::vector<const FuelLink *> links;
    links.reserve(fuelLinks.size() + (extra != nullptr ? 1 : 0));
    for(size_t k = 0; k < fuelLinks.size(); k++) { links.push_back(&fuelLinks[k]); }
    if(extra != nullptr) { links.push_back(extra); }

    /* The same partition and the same wall rule flight uses, so a loop found
       here is the loop build_ship throws on. */
    std::vector<int> parent(parts.size());
    std::vector<bool> wall(parts.size());
    for(size_t i = 0; i < parts.size(); i++) {
        parent[i] = parts[i].parent;
        wall[i] = parts[i].def != nullptr && parts[i].def->isFuelBarrier();
    }
    std::vector<int> group;
    const int nGroups = partitionFuelGroups(parent, wall, group);
    std::map<std::string, size_t> index;
    for(size_t i = 0; i < parts.size(); i++) { index[parts[i].id] = i; }

    /* Only group-to-group links are edges: a link inside one group moves
       nothing (that group already shares its fuel), and one ending on a barrier
       crosses a wall that never drains. An id that is not in the tree is not an
       edge either -- vabLinkClick re-checks its source and toShipDef drops a
       link whose endpoint died, so a dangling id is not a state to report. */
    std::vector<std::vector<FuelEdge> > out((size_t)nGroups);
    for(size_t k = 0; k < links.size(); k++) {
        std::map<std::string, size_t>::const_iterator f = index.find(links[k]->from);
        std::map<std::string, size_t>::const_iterator t = index.find(links[k]->to);
        if(f == index.end() || t == index.end()) { continue; }
        const int a = group[f->second];
        const int b = group[t->second];
        if(a < 0 || b < 0 || a == b) { continue; }
        out[(size_t)a].push_back(FuelEdge{ b, k });
    }

    std::vector<int> color((size_t)nGroups, 0), entry((size_t)nGroups, -1);
    std::vector<size_t> trail, loop;
    for(int g = 0; g < nGroups; g++) {
        if(color[(size_t)g] != 0) { continue; }
        if(!fuelLoopDfs(g, out, color, entry, trail, loop)) { continue; }
        std::string s;
        for(size_t k = 0; k < loop.size(); k++) {
            if(k > 0) { s += ", "; }
            s += links[loop[k]]->from + "->" + links[loop[k]]->to;
        }
        return s;
    }
    return std::string();
}

/* [0, 360) for tidy saved files; the solvers are wrap-agnostic. */
static double wrap360(double deg) {
    double a = std::fmod(deg, 360.0);
    if(a < 0.0) { a += 360.0; }
    return a;
}

bool save_ship_def(const BuildShip &bs, const char *path) {
    if(bs.parts.empty()) { return false; }
    const ShipDef def = bs.toShipDef();

    nlohmann::ordered_json doc;
    doc["name"] = def.name;
    if(def.controller >= 0 && (size_t)def.controller < def.parts.size()) {
        doc["controller"] = def.parts[(size_t)def.controller].id;
    }
    if(def.hull_margin >= 0.0) { doc["hull_margin"] = def.hull_margin; }
    if(def.has_attitude) {
        doc["attitude"] = nlohmann::ordered_json{
            {"pitch", def.attitude_pitch}, {"yaw", def.attitude_yaw}};
    }
    nlohmann::ordered_json arr = nlohmann::ordered_json::array();
    for(size_t i = 0; i < def.parts.size(); i++) {
        const ShipPart &sp = def.parts[i];
        nlohmann::ordered_json pv;
        pv["part"] = sp.part;
        pv["id"] = sp.id;
        if(sp.isFuelLink()) {
            pv["from"] = sp.from;
            pv["to"] = sp.to;
            arr.push_back(pv);
            continue;
        }
        // Name the parent explicitly: the load default (previous part) is a
        // trap for edited trees.
        if(sp.parent >= 0 && (size_t)sp.parent < def.parts.size()) {
            pv["parent"] = def.parts[(size_t)sp.parent].id;
        }
        pv["attach"] = (sp.attach == AttachMode::Surface) ? "surface"
                     : (sp.attach == AttachMode::Up) ? "up" : "down";
        if(sp.isStackEdge()) {
            // omit empty keys so the load defaults apply
            if(!sp.parentNode.empty()) { pv["parentNode"] = sp.parentNode; }
            if(!sp.childNode.empty())  { pv["childNode"] = sp.childNode; }
            pv["angle"] = wrap360(sp.angle);
        } else {
            pv["childNode"] = sp.childNode;
            pv["point"] = { sp.contactPoint.x, sp.contactPoint.y, sp.contactPoint.z };
            pv["normal"] = { sp.contactNormal.x, sp.contactNormal.y, sp.contactNormal.z };
            pv["roll"] = wrap360(sp.roll);
        }
        if(sp.offset != 0.0) { pv["offset"] = sp.offset; }
        if(sp.stage != 1) { pv["stage"] = sp.stage; }
        /* Trim, unlike roll/angle, is a signed small angle around a neutral: no
           wrap, and omitted at neutral so a def that never trims stays as it was. */
        if(sp.trim != 0.0) { pv["trim"] = sp.trim; }
        arr.push_back(pv);
    }
    doc["parts"] = arr;

    std::ofstream f(path);
    if(!f.is_open()) { return false; }
    f << doc.dump(2) << "\n";
    f.flush();
    return !f.fail();
}

double partLoadMass(const PartDef &def) {
    double m = def.mass;
    for(size_t r = 0; r < def.capacity.size(); r++) {
        if(r == (size_t)ResourceType::EC) { continue; }   // energy, no mass
        m += (double)def.capacity[r];
    }
    return m;
}

BuildStability buildStability(const BuildShip &bs, double occupantMass) {
    BuildStability out;
    if(bs.parts.empty()) { return out; }

    /* The COM the airframe pivots about at launch: partLoadMass is the same
       arithmetic Part::effectiveMass reports with the tanks seeded full. Contents
       are not walked -- a build tree carries them as ordinary parts (or not at
       all).
       A filled seat is the second term: `spawn_crew`/`fill_seat` park a suit Part
       in the capsule, whose mass Part::effectiveMass folds into that part's body,
       so it counts AT THE SEAT'S POSITION (measured: the skiff's 97.05 kg kerbal
       suit moves its COM 0.18 m forward, which is the difference between the
       +0.60 m of margin it flies with and the +0.42 m the same parts would have
       uncrewed). `occupantMass` is what one occupant weighs; 0 asks for a
       ship that launches empty. */
    glm::dvec3 com(0.0);
    double mass = 0.0;
    for(const BuildPart &bp : bs.parts) {
        if(bp.def == nullptr) { continue; }
        double m = partLoadMass(*bp.def);
        if(occupantMass > 0.0 && bp.def->crew_capacity > 0) {
            m += (double)bp.def->crew_capacity * occupantMass;
        }
        com += bp.localPos * m;
        mass += m;
    }
    if(mass <= 0.0) { return out; }
    out.mass = mass;
    out.com = com / mass;

    /* Flying straight ahead at unit speed: the only flow a build can be asked
       about without instruments. |v| cancels in a per-unit-q slope, so the length
       here is a direction with an honest magnitude. A wing mounted at incidence
       reports its slope AT that incidence, which is what its mount means. */
    const glm::dvec3 nose(0.0, 0.0, 1.0);
    const glm::dvec3 up(0.0, 1.0, 0.0);
    const glm::dvec3 right(1.0, 0.0, 0.0);
    LiftSlopeSum sum;
    for(const BuildPart &bp : bs.parts) {
        if(bp.def == nullptr || bp.def->lift_area <= 0.0) { continue; }
        const glm::dmat3 mount = bp.localRot;
        const PartAero pa = partAero(mount, nose);
        if(!pa.valid) { continue; }   // a mount with no lift axis lifts nothing
        sum.add(liftSlopePerQ(pa, mount, nose, bp.def->lift_area, bp.def->cl,
                              bp.def->stall_angle, right),
                bp.localPos - out.com, up, right);
    }
    out.slope = sum.slope();
    out.valid = sum.usable();
    if(out.valid) { out.margin = sum.margin(); }
    return out;
}

double seatOccupantMass(const ShipDef &suitDef) {
    double m = 0.0;
    for(const ShipPart &sp : suitDef.parts) {
        if(sp.def == nullptr) { continue; }
        m += partLoadMass(*sp.def);
    }
    return m;
}

double resolveHullMargin(double shipMargin, double partMargin) {
    if(shipMargin >= 0.0) { return shipMargin; }
    if(partMargin >= 0.0) { return partMargin; }
    return -1.0;
}

bool kerbalDefConsistent(const ShipDef &def) {
    const bool kerbalRoot = !def.parts.empty()
        && def.parts[0].def != nullptr
        && def.parts[0].def->type == "kerbal";
    return !kerbalRoot || def.parts.size() == 1;
}

// Every start-ship entry must name all four; a missing/empty one is a config error.
static const char *const kStartShipFields[] = {"ship", "name", "body", "scenario"};

DebugStartShips loadDebugStartShips(const char *path) {
    std::ifstream f(path);
    if(!f.is_open()) {
        throw std::runtime_error(std::string("start ships: cannot open ") + path);
    }
    nlohmann::json doc;
    try {
        doc = nlohmann::json::parse(f, nullptr, true);
    } catch(const std::exception &e) {
        throw std::runtime_error(std::string("start ships: bad JSON in ") + path
                                 + std::string(": ") + e.what());
    }
    if(!doc.is_object() || !doc.contains("ships") || !doc["ships"].is_array()
       || doc["ships"].empty()) {
        throw std::runtime_error(std::string("start ships: no ships in ") + path);
    }

    DebugStartShips list;
    const nlohmann::json &arr = doc["ships"];
    for(size_t i = 0; i < arr.size(); i++) {
        const nlohmann::json &ev = arr[i];
        if(!ev.is_object()) {
            throw std::runtime_error(std::string("start ships: entry ")
                                     + std::to_string(i) + " of " + path
                                     + " must be an object");
        }

        DebugStartShip e;
        for(const char *const field : kStartShipFields) {
            if(!ev.contains(field) || !ev[field].is_string()
               || ev[field].get<std::string>().empty()) {
                throw std::runtime_error(std::string("start ships: entry ")
                                         + std::to_string(i) + " of " + path
                                         + " is missing required field \""
                                         + field + "\"");
            }
        }
        e.ship = ev["ship"].get<std::string>();
        e.name = ev["name"].get<std::string>();
        e.body = ev["body"].get<std::string>();
        e.scenario = ev["scenario"].get<std::string>();
        if(ev.contains("site")) {
            if(!ev["site"].is_string()) {
                throw std::runtime_error(std::string("start ships: entry ")
                                         + std::to_string(i) + " of " + path
                                         + ": \"site\" must be a string");
            }
            e.site = ev["site"].get<std::string>();
        }
        list.ships.push_back(e);
    }
    return list;
}
