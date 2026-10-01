// ships.h -- ship construction and placement (the Ships builder).
//
// Ownership of the ships themselves is NOT here: each ship lives in the
// ships list of the body of its SoI (TerrainBody::ships, terrain.h), and
// a character aboard a ship lives on that ship (Vehicle::crew, eva.h).
#pragma once

#include <string>
#include <vector>

#include "body.h"     // Body, create_body
#include "camera.h"   // Camera
#include "frame.h"    // Frame
#include "mesh.h"     // Mesh
#include "shader.h"   // Shader
#include "shipdef.h"  // PartsCatalog
#include "texture.h"  // Texture
#include "terrain.h"  // TerrainBody, StaticBuilding
#include "vehicle.h"  // Vehicle, ScenarioDef

// DebugStartShip: one ship the game spawns at boot.
// All four fields are required; loadDebugStartShips errors if any is missing.
struct DebugStartShip {
    std::string ship;      // ship def path
    std::string name;      // display name
    std::string body;      // the body it sits on (a name in the system)
    std::string scenario;  // the spawn scenario (a --startship field)
};

// The collection of them (the parsed "ships" array).
struct DebugStartShips {
    std::vector<DebugStartShip> ships;
};

// Parse + validate the debug-start-ships JSON (load_system style): throws
// std::runtime_error naming the file + the offending entry on any bad/missing data.
DebugStartShips loadDebugStartShips(const char *path);

struct System;  // only used by reference in the signatures below
struct Kerbal;  // the crew characters (eva.h); spawn_crew_kerbal returns one

/* The canonical ship order across the system: bodies in file order, then
   each body's ships in order, with each ship's crew right after it. */
std::vector<Vehicle *> collectVehicles(System &sys);

/* The same walk, appended into a reused buffer (cleared here). */
void collectVehiclesInto(System &sys, std::vector<Vehicle *> &out);

class Ships {
public:
    /* parts_file:  the parts catalog to build ships from (owned, loaded here).
       partsshader: the part shader (caller-owned; must outlive us).
       sun:         the star (caller-owned; light source for ships + pads). */
    Ships(const std::string &parts_file, Shader *partsshader, TerrainBody *sun);
    /* The ships + pads are owned by the bodies (terrain.h) and die with
       them; the part shader + star are borrowed. Nothing to free here. */
    ~Ships() = default;

    Ships(const Ships &) = delete;
    Ships &operator=(const Ships &) = delete;

    // the parts catalog (for out-of-band ship builders, e.g. the radial test)
    const PartsCatalog &catalog() const { return part_catalog; }

    // Re-point the star (the in-process system switch).
    void setSun(TerrainBody *sun) { this->sun = sun; }

    // --- placement (the ships land in the body's list, not here) ---------
    // `t` (sim time) is the flight-journal stamp for the placement.
    // Build one ship from defPath and place it on body hb. Does NOT apply
    // the scenario -- spawn_vehicle is the caller's job.
    Vehicle *place_ship(const std::string &shipDefPath, const std::string &wantName,
                        TerrainBody *hb, const ScenarioDef *sc, System &sys,
                        double t);

    // Same, from an already-loaded def (the VAB Launch builds one in memory).
    Vehicle *place_ship_def(const ShipDef &def, const std::string &defPath,
                            const std::string &wantName,
                            TerrainBody *hb, const ScenarioDef *sc, System &sys,
                            double t);

    // Runtime spawn: place + apply the scenario + park on rails.
    Vehicle *spawn_ship(const std::string &defPath, const std::string &wantName,
                        TerrainBody *hb, const ScenarioDef *sc, System &sys,
                        double t);

    // Startup crew: one kerbal ABOARD each of `ship`'s capsule parts.
    // Called from buildDebugStartShips; runtime copies (spawn_ship) do NOT
    // get crew.
    void spawn_crew(Vehicle *ship, System &sys, double t);

    // Apply each ship's scenario (the startup spawn_vehicle pass).
    void apply_scenarios(System &sys, double t);

    // Register a ship built out-of-band (the --radial-test ship).
    void add_ship(Vehicle *v, TerrainBody *home, const ScenarioDef *sc, int slot,
                  double t);

    // Build the start ships from the resolved entries. Returns the first
    // ship built (the natural active one) or nullptr for an empty list.
    Vehicle *buildDebugStartShips(const std::vector<DebugStartShip> &entries,
                                  System &sys, double t);

private:
    // Ensure the (body, pad-site) pad exists; build it on demand.
    void place_pad(TerrainBody *hb, bool polar, const glm::dvec3 &dir, double pad_height);

    // One crew kerbal aboard (ship, part); nullptr if the part is no capsule.
    Kerbal *spawn_crew_kerbal(Vehicle *ship, size_t part, System &sys, double t);

    // De-duplicate a name across all the ships + crew (first keeps the
    // bare name, later ones get #2, #3 ..).
    std::string dedupName(System &sys, const std::string &nm);

    PartsCatalog part_catalog;
    Shader *partsshader;   // caller-owned
    TerrainBody *sun;      // caller-owned
};
