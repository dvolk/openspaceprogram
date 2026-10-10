// buildings.h -- building types (catalog) + world instances (system JSON).
//
// Catalog (res/data/buildings.json) is global, like parts.json. Instances
// live in the system JSON under "world_buildings" and are placed onto
// TerrainBody::pads at system load. lat/lon are DEGREES in the JSON and
// convert through equirectDir (radians). height_m is the building ORIGIN
// above the terrain surface along local up; heading_deg spins about that up.
#pragma once

#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <nlohmann/json.hpp>

struct System;
class Shader;
class TerrainBody;
class StaticBuilding;

// One building TYPE (catalog entry; world instances reference it by name).
// Mesh paths are res-relative ("meshes/space_port.obj"), like PartDef.
struct BuildingDef {
    std::string name;
    std::string display_name; // empty -> fall back to name
    std::string mesh;
    std::string texture;

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

/* Building orientation: +Z along `up` (radial), then spun `heading` rad
   about that axis. Ship placement uses the same matrix so a headed pad
   carries its ships. */
glm::dmat3 buildingOrient(const glm::dvec3 &up, double heading_rad);

/* Instantiate every world building on its body (TerrainBody::pads).
   `sys.world_buildings` must already be parsed. Needs live physics
   (create_physics) and a part shader for the Body. */
void place_world_buildings(System &sys, const BuildingsCatalog &cat,
                           Shader *shader, TerrainBody *sun);

// Find a launch site on `body`: `siteId` if non-empty, else the first pad
// with flag "VAB_LAUNCH_PAD". `preferPolar` prefers a "POLAR_PAD" site.
// Returns nullptr when nothing matches.
StaticBuilding *find_launch_site(TerrainBody *body, const std::string &siteId,
                                 bool preferPolar);
