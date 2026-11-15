// buildings.h -- building types (catalog) + world instances (system JSON).
//
// Catalog (res/data/buildings.json) is global; instances live in the system
// JSON under "world_buildings" (or the fragments its "world_buildings_files"
// array names) and are placed onto TerrainBody::pads at system load. lat/lon
// are DEGREES in the JSON and convert through equirectDir (radians). height_m
// is the building ORIGIN above the terrain surface along local up; heading_deg
// spins about that up.
#pragma once

#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <nlohmann/json.hpp>

struct System;
class Shader;
class TerrainBody;
class StaticBuilding;

// One building TYPE (catalog entry; instances reference it by name).
// Mesh paths are res-relative ("meshes/space_port.obj").
struct BuildingDef {
    std::string name;
    std::string display_name; // empty -> fall back to name
    std::string mesh;
    std::string texture;
    /* Optional collision mesh, when the visual one must not hull: a balloon
       platform's deck collides, its balloons don't (the hull of the whole
       visual mesh would enclose the deck). Empty -> hull the visual mesh. */
    std::string collision_mesh;

    const std::string &label() const {
        return display_name.empty() ? name : display_name;
    }
};

struct BuildingsCatalog {
    std::vector<BuildingDef> buildings;
    const BuildingDef *find(const std::string &name) const;
};

// Parse + validate; throws std::runtime_error naming the file + field.
BuildingsCatalog load_buildings_catalog(const char *path);

// One world instance (the system JSON "world_buildings" entry).
struct WorldBuilding {
    std::string id;           // instance id (unique in the system)
    std::string type;         // catalog name
    std::string display_name; // optional; empty -> catalog label
    std::string body;         // body name
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double height_m = 5.0;    // origin above terrain (space_port pad top)
    double heading_deg = 0.0; // spin about local up
    std::vector<std::string> flags;

    /* Orbital placement (a space dock / station): mutually exclusive with the
       surface lat/lon above. The circular spec is authoring sugar -- it becomes
       a full epoch state the engine propagates as a Kepler conic (buildingorbit.h).
       Angles in DEGREES here, like lat/lon. */
    bool orbital = false;
    double orb_alt_m = 0.0;      // circular altitude above sea level
    double orb_inc_deg = 0.0;    // inclination
    double orb_raan_deg = 0.0;   // longitude of the ascending node
    double orb_phase_deg = 0.0;  // true anomaly at t = 0
    /* Attitude: "body-fixed" (nadir-pointing, tracks the orbit) or "sky-fixed"
       (constant inertial). An optional spin rides on top about a LOCAL axis. */
    std::string att_mode = "body-fixed";
    double spin_rate_deg_s = 0.0;
    glm::dvec3 spin_axis = glm::dvec3(0.0, 1.0, 0.0);

    /* Air-relative placement (a balloon platform): mutually exclusive with
       both the orbital spec and the surface lat/lon. Holds its lat/alt and
       drifts east with the zonal wind sampled at the anchor (buildingair.h).
       The body must have a physical atmosphere WITH a wind table. */
    bool air = false;
    double air_alt_m = 0.0;     // above the sea-level datum
    double air_lat_deg = 0.0;   // held; |lat| <= 85 (the drift divides by cos lat)
    double air_lon_deg = 0.0;   // anchor longitude at t = 0

    bool hasFlag(const std::string &f) const;
    // display_name, else the catalog label, else the type.
    std::string label(const BuildingsCatalog &cat) const;
    // Unit up direction in the body's rotating frame (equirect convention).
    glm::dvec3 dir() const;
};

// Parse the root "world_buildings" array (optional; empty if absent).
// Throws on bad/missing fields. `path` is for error messages.
std::vector<WorldBuilding> parse_world_buildings(const nlohmann::json &doc,
                                                 const std::string &path);

/* The inline array PLUS the fragments named by the root
   "world_buildings_files" array. A fragment is a JSON file with its own
   "world_buildings" array -- the same spelling, so it is a drop-in piece of
   a system -- and "res/..." names resolve through resdir. Fragments may not
   nest.

   Fragment entries come FIRST, then the inline array: find_launch_site's
   default site is the first VAB_LAUNCH_PAD in the merged list, so a system's
   inline pads can never pre-empt a site file's chosen default.

   Throws on a bad entry, an unreadable fragment, or a duplicate id. */
std::vector<WorldBuilding> parse_world_buildings_all(
    const nlohmann::json &doc, const std::string &path);

/* Building orientation: +Z along `up` (radial), then spun `heading` rad
   about that axis. Ship placement uses the same matrix so a headed pad
   carries its ships. */
glm::dmat3 buildingOrient(const glm::dvec3 &up, double heading_rad);

/* The checks place_world_buildings would throw on, without instantiating or
   needing physics: type in the catalog, body in the system, unique instance id
   per body. A cross-system load runs this on the incoming system so a bad
   definition refuses before the switch destroys the running game (#302). */
void validate_world_buildings(const System &sys, const BuildingsCatalog &cat);

/* Instantiate every world building on its body (TerrainBody::pads).
   `sys.world_buildings` must already be parsed; needs live physics
   (create_physics) and a part shader. Validates first (see above). */
void place_world_buildings(System &sys, const BuildingsCatalog &cat,
                           Shader *shader, TerrainBody *sun);

/* The body's launch site for `siteId`: that pad, if the body carries one with
   that id flagged "VAB_LAUNCH_PAD", else nullptr. An empty `siteId` asks for
   the body's default -- its first VAB_LAUNCH_PAD. */
StaticBuilding *find_launch_site(TerrainBody *body, const std::string &siteId);
