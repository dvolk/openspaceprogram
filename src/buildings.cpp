// buildings.cpp -- building catalog + world instances (see buildings.h).
#include "buildings.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <numbers>
#include <stdexcept>

#include "body.h"        // create_body
#include "buildingair.h" // air-relative drift rate
#include "equirect.h"    // equirectDir
#include "mesh.h"     // get_mesh
#include "physics.h"  // setPosRot
#include "resdir.h"   // resdir::path
#include "system.h"   // System
#include "terrain.h"  // TerrainBody, StaticBuilding
#include "texture.h"  // get_texture
#include "vehicle.h"  // faceAlong

const BuildingDef *BuildingsCatalog::find(const std::string &name) const {
    for(size_t i = 0; i < buildings.size(); i++) {
        if(buildings[i].name == name) { return &buildings[i]; }
    }
    return nullptr;
}

BuildingsCatalog load_buildings_catalog(const char *path) {
    std::ifstream f(path);
    if(!f.is_open()) {
        throw std::runtime_error(std::string("buildings: cannot open ") + path);
    }
    nlohmann::json doc;
    try {
        doc = nlohmann::json::parse(f, nullptr, true);
    } catch(const std::exception &e) {
        throw std::runtime_error(std::string("buildings: bad JSON in ") + path
                                 + std::string(": ") + e.what());
    }
    if(!doc.is_object() || !doc.contains("buildings") || !doc["buildings"].is_array()
       || doc["buildings"].empty()) {
        throw std::runtime_error(std::string("buildings: no buildings in ") + path);
    }

    BuildingsCatalog cat;
    const nlohmann::json &arr = doc["buildings"];
    for(size_t i = 0; i < arr.size(); i++) {
        const nlohmann::json &bv = arr[i];
        BuildingDef d;
        d.name = bv.value("name", std::string(""));
        if(d.name.empty()) {
            throw std::runtime_error(std::string("buildings: entry ")
                                     + std::to_string(i) + " of " + path
                                     + ": missing \"name\"");
        }
        const std::string ctx = "buildings: " + d.name + ": ";
        if(cat.find(d.name) != nullptr) {
            throw std::runtime_error(ctx + "duplicate building name");
        }
        d.display_name = bv.value("display_name", std::string(""));
        d.mesh = bv.value("mesh", std::string(""));
        d.texture = bv.value("texture", std::string(""));
        if(d.mesh.empty() || d.texture.empty()) {
            throw std::runtime_error(ctx + "missing \"mesh\"/\"texture\"");
        }
        if(bv.contains("collision_mesh")) {
            if(!bv["collision_mesh"].is_string()
               || bv["collision_mesh"].get<std::string>().empty()) {
                throw std::runtime_error(ctx + "\"collision_mesh\" must be a non-empty string");
            }
            d.collision_mesh = bv["collision_mesh"].get<std::string>();
        }
        cat.buildings.push_back(d);
    }
    return cat;
}

bool WorldBuilding::hasFlag(const std::string &f) const {
    for(size_t i = 0; i < flags.size(); i++) {
        if(flags[i] == f) { return true; }
    }
    return false;
}

std::string WorldBuilding::label(const BuildingsCatalog &cat) const {
    if(!display_name.empty()) { return display_name; }
    if(const BuildingDef *d = cat.find(type)) { return d->label(); }
    return type;
}

glm::dvec3 WorldBuilding::dir() const {
    const double lon = lon_deg * std::numbers::pi / 180.0;
    const double lat = lat_deg * std::numbers::pi / 180.0;
    return equirectDir(lon, lat);
}

// One "world_buildings" array -> entries; `src` names the file for messages.
// Duplicate ids: here within an array, in parse_world_buildings_all across
// arrays.
static std::vector<WorldBuilding> parse_building_array(const nlohmann::json &arr,
                                                       const std::string &src) {
    std::vector<WorldBuilding> out;
    for(size_t i = 0; i < arr.size(); i++) {
        const nlohmann::json &ev = arr[i];
        if(!ev.is_object()) {
            throw std::runtime_error(std::string("system: world_buildings entry ")
                                     + std::to_string(i) + " of " + src
                                     + " must be an object");
        }
        WorldBuilding w;
        w.id = ev.value("id", std::string(""));
        w.type = ev.value("type", std::string(""));
        w.display_name = ev.value("display_name", std::string(""));
        w.body = ev.value("body", std::string(""));
        const std::string ctx = "system: world_buildings[" + std::to_string(i)
                                + "] of " + src + ": ";
        if(w.id.empty()) { throw std::runtime_error(ctx + "missing \"id\""); }
        if(w.type.empty()) { throw std::runtime_error(ctx + "missing \"type\""); }
        if(w.body.empty()) { throw std::runtime_error(ctx + "missing \"body\""); }
        for(size_t k = 0; k < out.size(); k++) {
            if(out[k].id == w.id) {
                throw std::runtime_error(ctx + "duplicate id '" + w.id + "'");
            }
        }
        // Optional numeric fields with contextual errors (a bare .value()
        // would throw a type_error without the file+field on a non-numeric
        // value). Shared by the orbital and air paths.
        auto optNum = [&](const nlohmann::json &obj, const char *key) -> double {
            if(!obj.contains(key)) { return 0.0; }
            if(!obj[key].is_number() || !std::isfinite(obj[key].get<double>())) {
                throw std::runtime_error(ctx + std::string("\"") + key
                                         + "\" must be a finite number");
            }
            return obj[key].get<double>();
        };
        if(ev.contains("orbit")) {
            /* Orbital placement (a space dock / station): a circular-orbit
               authoring form that becomes a full Kepler state at placement.
               Mutually exclusive with the air and surface forms below. */
            if(ev.contains("lat_deg") || ev.contains("lon_deg") || ev.contains("air")) {
                throw std::runtime_error(ctx + "an orbital building must not also "
                                              "author \"lat_deg\"/\"lon_deg\"/\"air\"");
            }
            const nlohmann::json &ov = ev["orbit"];
            w.orbital = true;
            if(!ov.contains("alt_m") || !ov["alt_m"].is_number()
               || !std::isfinite(ov["alt_m"].get<double>())
               || ov["alt_m"].get<double>() <= 0.0) {
                throw std::runtime_error(ctx + "orbit needs a positive finite \"alt_m\"");
            }
            w.orb_alt_m = ov["alt_m"].get<double>();
            w.orb_inc_deg = optNum(ov, "inc_deg");
            w.orb_raan_deg = optNum(ov, "raan_deg");
            w.orb_phase_deg = optNum(ov, "phase_deg");
            if(ev.contains("attitude") && !ev["attitude"].is_string()) {
                throw std::runtime_error(ctx + "\"attitude\" must be a string");
            }
            w.att_mode = ev.value("attitude", std::string("body-fixed"));
            if(w.att_mode != "body-fixed" && w.att_mode != "sky-fixed") {
                throw std::runtime_error(ctx + "\"attitude\" must be \"body-fixed\" "
                                              "or \"sky-fixed\"");
            }
            if(ev.contains("spin")) {
                const nlohmann::json &sv = ev["spin"];
                w.spin_rate_deg_s = optNum(sv, "rate_deg_s");
                if(sv.contains("axis")) {
                    const nlohmann::json &av = sv["axis"];
                    if(!av.is_array() || av.size() != 3
                       || !av[0].is_number() || !av[1].is_number()
                       || !av[2].is_number()) {
                        throw std::runtime_error(ctx + "spin.axis must be [x, y, z] numbers");
                    }
                    w.spin_axis = glm::dvec3(av[0].get<double>(), av[1].get<double>(),
                                             av[2].get<double>());
                    if(glm::length(w.spin_axis) < 1e-9) {
                        throw std::runtime_error(ctx + "spin.axis must be nonzero");
                    }
                    w.spin_axis = glm::normalize(w.spin_axis);
                }
            }
        } else if(ev.contains("air")) {
            /* Air-relative placement (a balloon platform): holds lat/alt and
               drifts with the zonal wind (buildingair.h). Mutually exclusive
               with the surface lat/lon; the body's atmosphere/wind check lives
               in validate_world_buildings, where the body is in hand. */
            if(ev.contains("lat_deg") || ev.contains("lon_deg")) {
                throw std::runtime_error(ctx + "an air building must not also "
                                              "author \"lat_deg\"/\"lon_deg\"");
            }
            if(ev.contains("attitude") || ev.contains("spin")) {
                throw std::runtime_error(ctx + "\"attitude\"/\"spin\" are orbital-only; "
                                              "an air platform holds its heading_deg");
            }
            const nlohmann::json &av = ev["air"];
            w.air = true;
            if(!av.contains("alt_m") || !av["alt_m"].is_number()
               || !std::isfinite(av["alt_m"].get<double>())
               || av["alt_m"].get<double>() <= 0.0) {
                throw std::runtime_error(ctx + "air needs a positive finite \"alt_m\"");
            }
            w.air_alt_m = av["alt_m"].get<double>();
            if(!av.contains("lat_deg")) {
                throw std::runtime_error(ctx + "air needs a \"lat_deg\"");
            }
            w.air_lat_deg = optNum(av, "lat_deg");
            // The drift rate divides by cos(lat): a polar platform is a
            // singularity, refused rather than clamped.
            if(w.air_lat_deg < -85.0 || w.air_lat_deg > 85.0) {
                throw std::runtime_error(ctx + "air \"lat_deg\" must be in [-85, 85]");
            }
            if(!av.contains("lon_deg")) {
                throw std::runtime_error(ctx + "air needs a \"lon_deg\"");
            }
            w.air_lon_deg = optNum(av, "lon_deg");
        } else {
            if(!ev.contains("lat_deg") || !ev["lat_deg"].is_number()
               || !ev.contains("lon_deg") || !ev["lon_deg"].is_number()
               || !std::isfinite(ev["lat_deg"].get<double>())
               || !std::isfinite(ev["lon_deg"].get<double>())) {
                throw std::runtime_error(ctx + "needs finite numeric \"lat_deg\"/\"lon_deg\"");
            }
            w.lat_deg = ev["lat_deg"].get<double>();
            // Kept as authored -- a pad file may write lon -44 or 316 for the same
            // meridian, and which one it wrote is what the dossier echoes. dir() below
            // takes it through cos/sin, so the pair is equivalent either way; the
            // finite check above is what keeps a huge value from becoming a NaN
            // direction down the line.
            w.lon_deg = ev["lon_deg"].get<double>();
            if(w.lat_deg < -90.0 || w.lat_deg > 90.0) {
                throw std::runtime_error(ctx + "\"lat_deg\" must be in [-90, 90]");
            }
        }
        /* The SoI bubble of a driven building (#850); surface pads ignore it. */
        if(ev.contains("bubble_km")) {
            w.bubble_km = optNum(ev, "bubble_km");
            if(w.bubble_km <= 0.0) {
                throw std::runtime_error(ctx + "\"bubble_km\" must be positive");
            }
        }
        w.height_m = 5.0;
        w.heading_deg = 0.0;
        if(ev.contains("height_m")) {
            if(!ev["height_m"].is_number() || !std::isfinite(ev["height_m"].get<double>())) {
                throw std::runtime_error(ctx + "\"height_m\" must be a finite number");
            }
            w.height_m = ev["height_m"].get<double>();
        }
        if(ev.contains("heading_deg")) {
            // Finite: heading drives the air pose, so a NaN here is a NaN pose.
            if(!ev["heading_deg"].is_number() || !std::isfinite(ev["heading_deg"].get<double>())) {
                throw std::runtime_error(ctx + "\"heading_deg\" must be a finite number");
            }
            w.heading_deg = ev["heading_deg"].get<double>();
        }
        if(ev.contains("flags")) {
            if(!ev["flags"].is_array()) {
                throw std::runtime_error(ctx + "\"flags\" must be an array of strings");
            }
            for(const nlohmann::json &fv : ev["flags"]) {
                if(!fv.is_string() || fv.get<std::string>().empty()) {
                    throw std::runtime_error(ctx + "\"flags\" entries must be non-empty strings");
                }
                w.flags.push_back(fv.get<std::string>());
            }
        }
        out.push_back(w);
    }
    return out;
}

std::vector<WorldBuilding> parse_world_buildings(const nlohmann::json &doc,
                                                 const std::string &path) {
    if(!doc.contains("world_buildings")) { return {}; }
    const nlohmann::json &arr = doc["world_buildings"];
    if(!arr.is_array()) {
        throw std::runtime_error(std::string("system: \"world_buildings\" in ")
                                 + path + " must be an array");
    }
    return parse_building_array(arr, path);
}

/* One id names one building, across the WHOLE system (#890). Ids are handles: the VAB
   picker, find_launch_site, each body's pad list and save.cpp's building keys all resolve
   by id, so a second building with the same id silently shadows the first -- unlike
   display_name, which dedupBuildingName renames (#883).
   Stated once and called twice rather than written twice, because the two writers had
   already picked different scopes: parse checked the merged list (inline + every fragment
   it names) and validate checked per body, which anything reaching it from load_system had
   passed already. That made validate's term unreachable in the game and left it biting
   only a hand-built sys.world_buildings -- and it is how `WorldBuilding::id`'s comment came
   to say one thing while the nearer code implied another. */
static void checkUniqueBuildingIds(const std::vector<WorldBuilding> &ws,
                                   const std::string &where) {
    for(size_t i = 0; i < ws.size(); i++) {
        for(size_t k = i + 1; k < ws.size(); k++) {
            if(ws[i].id != ws[k].id) { continue; }
            throw std::runtime_error(where + " building id '" + ws[i].id + "'");
        }
    }
}

std::vector<WorldBuilding> parse_world_buildings_all(
        const nlohmann::json &doc, const std::string &path) {
    std::vector<WorldBuilding> out;
    if(doc.contains("world_buildings_files")) {
        if(!doc["world_buildings_files"].is_array()) {
            throw std::runtime_error(std::string("system: \"world_buildings_files\" in ")
                                     + path + " must be an array of file names");
        }
        for(const nlohmann::json &fv : doc["world_buildings_files"]) {
            if(!fv.is_string() || fv.get<std::string>().empty()) {
                throw std::runtime_error(std::string("system: \"world_buildings_files\" entry of ")
                                         + path + " must be a non-empty file name");
            }
            const std::string name = fv.get<std::string>();
            std::ifstream f(resdir::path(name));
            if(!f.is_open()) {
                throw std::runtime_error("system: '" + path + "': cannot open"
                                         " world_buildings_files entry '" + name + "'");
            }
            nlohmann::json frag;
            try {
                frag = nlohmann::json::parse(f, nullptr, true);
            } catch(const std::exception &e) {
                throw std::runtime_error("system: '" + path + "': bad JSON in"
                                         " world_buildings_files entry '" + name
                                         + "': " + e.what());
            }
            // One level only: nesting turns a typo into an unbounded merge.
            if(frag.contains("world_buildings_files")) {
                throw std::runtime_error("system: '" + path + "': world buildings"
                                         " file '" + name + "' nests"
                                         " \"world_buildings_files\"");
            }
            if(!frag.contains("world_buildings")) {
                throw std::runtime_error("system: '" + path + "': world buildings"
                                         " file '" + name + "' has no"
                                         " \"world_buildings\" array");
            }
            const nlohmann::json &farr = frag["world_buildings"];
            if(!farr.is_array()) {
                throw std::runtime_error("system: \"world_buildings\" in '" + name
                                         + "' must be an array");
            }
            // An empty list is a typo: it would silently drop the site list.
            if(farr.empty()) {
                throw std::runtime_error("system: '" + path + "': world buildings"
                                         " file '" + name + "' names no pads");
            }
            for(const WorldBuilding &w : parse_building_array(farr, name)) {
                out.push_back(w);
            }
        }
    }
    // Fragments first: find_launch_site's default is the first VAB_LAUNCH_PAD
    // in this list, so the site file picks it.
    for(const WorldBuilding &w : parse_world_buildings(doc, path)) {
        out.push_back(w);
    }
    // Ids are handles; a duplicate would silently shadow one of the two. The message
    // keeps its old shape -- it names the file the merge came from.
    checkUniqueBuildingIds(out, "system: '" + path + "': duplicate world");
    return out;
}

glm::dmat3 buildingOrient(const glm::dvec3 &up, double heading_rad) {
    const glm::dmat3 base = faceAlong(up);
    const double c = std::cos(heading_rad), s = std::sin(heading_rad);
    // Spin about the building's local +Z (the up axis).
    return base * glm::dmat3(glm::dvec3(c, s, 0.0),
                             glm::dvec3(-s, c, 0.0),
                             glm::dvec3(0.0, 0.0, 1.0));
}

/* Everything place_world_buildings can reject, checked before anything is
   instantiated. A cross-system load runs this on the incoming system so a bad
   building definition refuses BEFORE the switch tears the running game down
   (#302): the same throw from inside the placement loop lands after the
   teardown, by which point the fleet and the science are already gone. */
void validate_world_buildings(const System &sys, const BuildingsCatalog &cat) {
    for(const WorldBuilding &w : sys.world_buildings) {
        if(cat.find(w.type) == nullptr) {
            std::string avail;
            for(size_t i = 0; i < cat.buildings.size(); i++) {
                if(i) { avail += ", "; }
                avail += cat.buildings[i].name;
            }
            throw std::runtime_error("buildings: '" + w.id + "' has unknown type '"
                                     + w.type + "' (catalog has: " + avail + ")");
        }
        TerrainBody *hb = sys.find(w.body);
        if(hb == nullptr) {
            throw std::runtime_error("buildings: '" + w.id + "' names unknown body '"
                                     + w.body + "'");
        }
        /* An air platform needs air to float in AND a wind to ride: windAt
           asserts a non-empty table, and a render-only rim is not a drag
           regime. Checked here, pre-teardown (#302), not in placement. */
        if(w.air) {
            const AtmosphereParams &atm = hb->surface.atmosphere;
            if(!atm.physical() || !atm.has_wind()) {
                throw std::runtime_error("buildings: '" + w.id + "' is air-relative but "
                                         + w.body + " has no wind-bearing atmosphere");
            }
            if(w.air_alt_m >= atm.top()) {
                throw std::runtime_error("buildings: '" + w.id + "' floats at "
                                         + std::to_string(w.air_alt_m) + " m, above "
                                         + w.body + "'s air top");
            }
        }
        /* A driven building's bubble must stay clear of the terrain (#850):
           inside the bubble the deck is the ground, and a bubble that reached
           the surface would put two grounds in one frame. */
        if(w.air || w.orbital) {
            const double alt = w.air ? w.air_alt_m : w.orb_alt_m;
            if(alt - w.bubble_km * 1000.0 <= hb->surface.max_height) {
                throw std::runtime_error("buildings: '" + w.id + "' bubble (radius "
                                         + std::to_string(w.bubble_km) + " km) reaches "
                                         + w.body + "'s terrain -- raise the building "
                                         "or shrink \"bubble_km\"");
            }
        }
    }
    /* Same rule parse enforces on a merged file, called here because a caller can hand
       validate a list it built itself (tests do exactly that). One implementation, one
       scope: an earlier copy here compared per body, which every system reaching this
       from load_system had already survived, so the term it added could not fire (#890). */
    checkUniqueBuildingIds(sys.world_buildings, "buildings: duplicate world");
}

/* A driven building carries a frame node with a small SoI bubble (#850): a
   ship inside rides the building's rest frame, landed like on a planet. The
   node is a child of the pose frame (rot for air, inertial for orbital), and
   the tree's own clock drives it through the pose law (StaticBuilding::tickFrame).
   The building owns the node; ~StaticBuilding detaches it. */
static void attach_bubble_frame(StaticBuilding *sp, TerrainBody *hb,
                                const WorldBuilding &w) {
    Frame *f = new Frame;
    f->name = w.id + " (bubble)";
    f->parent = sp->poseFrame();
    f->body = hb;
    f->rot_frame = hb->rot_frame;
    f->building = sp;
    f->soi = w.bubble_km * 1000.0;
    f->driven_tick = [sp](Frame *, double t) { sp->tickFrame(t); };
    f->parent->children.push_back(f);
    sp->frame = f;
    /* The tree is clocked at t=0 by load_system before placement; pre-fill the
       node so a paused spawn beside the building sees a valid frame. */
    sp->tickFrame(0.0);
}

std::string dedupBuildingName(std::vector<std::string> &taken,
                              const std::string &base) {
    std::string nm = base;
    int n = 2;
    while(true) {
        bool hit = false;
        for(const std::string &s : taken) {
            if(s == nm) { hit = true; break; }
        }
        if(!hit) { break; }
        nm = base + " #" + std::to_string(n);
        n++;
    }
    // Remembers what it handed out, so a caller cannot forget to and silently
    // restore the collision.
    taken.push_back(nm);
    return nm;
}

void place_world_buildings(System &sys, const BuildingsCatalog &cat,
                           Shader *shader, TerrainBody *sun) {
    validate_world_buildings(sys, cat);
    /* One pass over world_buildings, in file order, is the whole rule: parse
       refuses a duplicate instance id across the merged list (fragments and
       inline together, parse_world_buildings_all below) and the id guard in the
       loop skips an already-placed building, so no label is asked for twice and
       the first entry in the file keeps the bare name. Nothing seeds from pads
       already on the bodies: the second placement call (a system switch, #850)
       skips every entry, and the first runs on fresh bodies. */
    std::vector<std::string> taken;
    for(const WorldBuilding &w : sys.world_buildings) {
        const BuildingDef *def = cat.find(w.type);   // validated above
        TerrainBody *hb = sys.find(w.body);          // validated above
        /* Idempotent: a cross-system load places the incoming buildings BEFORE
           building the save's ships (a bubble pose resolves against the pads),
           and installSystem places again after the switch (#850). */
        bool placed = false;
        for(const StaticBuilding *p : hb->pads) {
            if(p->id == w.id) { placed = true; break; }
        }
        if(placed) { continue; }
        Mesh *m = get_mesh(std::string("res/") + def->mesh);
        Texture *t = get_texture(std::string("res/") + def->texture);
        // The collider may be a subset of the visual (a balloon platform's
        // deck, not its balloons); empty -> hull the visual mesh as always.
        Mesh *cm = def->collision_mesh.empty()
                 ? nullptr : get_mesh(std::string("res/") + def->collision_mesh);
        StaticBuilding *sp = new StaticBuilding;
        sp->body = create_body(m, shader, t, 0, 0, 0, 0, cm);
        sp->parent = hb;
        sp->sun = sun;
        sp->id = w.id;
        const std::string base = w.label(cat);
        sp->display_name = dedupBuildingName(taken, base);
        if(sp->display_name != base) {
            /* `warning:`, not the throwing messages' `buildings:` prefix, so an
               author reading a load log can tell a rename from a refusal. Name
               the building that already owns the label -- that is the half the
               author cannot see from the file's order alone. */
            std::string holder;
            for(const WorldBuilding &o : sys.world_buildings) {
                if(&o != &w && o.label(cat) == base) { holder = o.id; break; }
            }
            const std::string who = holder.empty()
                ? std::string("another building in this system")
                : "'" + holder + "'";
            printf("warning: building '%s' answers to \"%s\" -- \"%s\" is "
                   "already taken by %s\n", w.id.c_str(),
                   sp->display_name.c_str(), base.c_str(), who.c_str());
        }
        sp->flags = w.flags;
        if(w.orbital) {
            /* A space dock / station: a driven pose on a Kepler orbit, in the
               body's inertial frame. The circular authoring form becomes a full
               epoch state here; the engine propagates it each tick. */
            sp->orbital = true;
            const double d2r = std::numbers::pi / 180.0;
            const double r = hb->seaLevelRadius() + w.orb_alt_m;
            if(!buildingorbit::circularEpochState(r, w.orb_inc_deg * d2r,
                    w.orb_raan_deg * d2r, w.orb_phase_deg * d2r, hb->mu,
                    sp->epochPos, sp->epochVel)) {
                throw std::runtime_error("buildings: bad orbit for '" + w.id + "'");
            }
            sp->orbMu = hb->mu;
            sp->attMode = (w.att_mode == "sky-fixed")
                        ? buildingorbit::Mode::SkyFixed : buildingorbit::Mode::BodyFixed;
            sp->spinAxis = w.spin_axis;
            sp->spinRate = w.spin_rate_deg_s * d2r;
            // Immovable scenery: a mass-0 STATIC collider whose pose we drive each
            // tick (a ship bounces off; the building never moves). Out of the world
            // until the proximity gate adds it; drawn regardless. We deliberately
            // do NOT MakeKinematic here: a teleported kinematic body did not block
            // the ship in this Bullet build (see the docking follow-up issue) --
            // static is the simple v1 answer for scenery you collide with.
            RemoveBody(sp->body);
            sp->tickPose(0.0);
            attach_bubble_frame(sp, hb, w);
        } else if(w.air) {
            /* A balloon platform: rides the zonal wind at its anchor, in the
               body's ROTATING frame (where the wind is authored and an in-atmo
               ship integrates). The drift rate is sampled ONCE here -- for a
               steady zonal wind that is the exact advection (buildingair.h).
               Immovable scenery like the orbital case: a mass-0 static
               collider, out of the world until the proximity gate adds it. */
            sp->air = true;
            const double d2r = std::numbers::pi / 180.0;
            sp->airR = hb->seaLevelRadius() + w.air_alt_m;
            sp->airLat = w.air_lat_deg * d2r;
            sp->airLon0 = w.air_lon_deg;
            sp->airHeading = w.heading_deg * d2r;
            const glm::dvec3 wv = windAt(hb->surface.atmosphere.wind, w.air_alt_m,
                                         w.air_lat_deg, w.air_lon_deg);
            if(std::fabs(wv.y) > 1e-9 || std::fabs(wv.z) > 1e-9) {
                printf("warning: air building '%s': north/up wind ignored "
                       "(v1 drifts with the zonal component only)\n", w.id.c_str());
            }
            sp->airEast = wv.x;
            sp->airDriftRate = buildingair::driftRate(wv.x, sp->airR, sp->airLat);
            /* The deck keeps ordinary friction: a parked ship rides the
               platform's bubble (#850), where the deck IS static, so the
               solver's zero-velocity static body is the truth and friction
               grips instead of scrubbing. (The old frictionless hack existed
               for the pre-bubble world, where a parked ship moved at the wind
               speed through the deck's frame.) */
            RemoveBody(sp->body);
            sp->tickPose(0.0);
            attach_bubble_frame(sp, hb, w);
        } else {
            const glm::dvec3 up = w.dir();
            sp->dir = up;
            sp->height_m = w.height_m;
            sp->heading_deg = w.heading_deg;
            const double heading = w.heading_deg * std::numbers::pi / 180.0;
            sp->posePos = sp->origin();
            sp->poseOrient = buildingOrient(up, heading);
            setPosRot(sp->body, sp->posePos, sp->poseOrient);
        }
        hb->pads.push_back(sp);
    }
}

StaticBuilding *find_launch_site(TerrainBody *body, const std::string &siteId) {
    if(body == nullptr) { return nullptr; }
    StaticBuilding *first = nullptr;
    for(auto *p : body->pads) {
        if(!p->hasFlag("VAB_LAUNCH_PAD")) { continue; }
        if(first == nullptr) { first = p; }
        if(!siteId.empty() && p->id == siteId) { return p; }
    }
    /* A named site that is missing, or is a building that is not a pad, is a
       config error rather than a request for the default. */
    return siteId.empty() ? first : nullptr;
}
