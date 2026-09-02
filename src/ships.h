// ships.h -- ship construction and placement (the Ships builder).
//
// Ownership of the ships themselves is NOT here: each ship lives in the
// ships list of the body of its SoI (TerrainBody::ships, terrain.h), and
// a character aboard a ship lives on that ship (Vehicle::crew, eva.h).
// This class holds only the shared resources every built ship references
// (the parts catalog, the shared space-port model, the part shader, the
// star) plus the operations that build ships/crew and place them into the
// world. The active-ship selection (game.h) and the tick (tick.cpp) walk
// the bodies' lists, not this class.
#pragma once

#include <string>
#include <vector>

#include "body.h"     // Body, create_body
#include "camera.h"   // Camera
#include "frame.h"    // Frame
#include "mesh.h"     // Mesh
#include "model.h"    // Model
#include "shader.h"   // Shader
#include "shipdef.h"  // PartsCatalog
#include "texture.h"  // Texture
#include "terrain.h"  // TerrainBody, StaticBuilding
#include "vehicle.h"  // Vehicle, ScenarioDef

#include "fleet.h"    // FleetEntry (the startup fleet's entries)

struct System;  // only used by reference in the signatures below
struct Kerbal;  // the crew characters (eva.h); spawn_crew_kerbal returns one

/* The canonical ship order across the system: bodies in file order, then
   each body's ships in order, with each ship's crew right after it. This
   is the single order selection (F6 / "N of M"), the Ship List window,
   the tick snapshot and the name de-duplication all walk, so "ship N of
   M" means the same thing everywhere. */
std::vector<Vehicle *> collectVehicles(System &sys);

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

    // --- placement (the ships land in the body's list, not here) ---------
    // Build one ship from defPath and place it on body hb: slot it (next
    // free for the (body, scenario) group), de-dup its name, make sure the
    // pad exists, build it on the pad. Does NOT apply the scenario --
    // spawn_vehicle is the caller's job, so startup keeps a single spawn
    // pass (apply_scenarios).
    Vehicle *place_ship(const std::string &shipDefPath, const std::string &wantName,
                        TerrainBody *hb, const ScenarioDef *sc, System &sys);

    // Runtime spawn: place + apply the scenario + park on rails. Appended
    // at the end of the body's list, so it is never the active one.
    Vehicle *spawn_ship(const std::string &defPath, const std::string &wantName,
                        TerrainBody *hb, const ScenarioDef *sc, System &sys);

    // Startup crew: one kerbal ABOARD each of `ship`'s capsule parts
    // (partDefs[i]->crew_capacity > 0) -- parked inside (out of the
    // physics world, the railFrozen convention), its mass folded into the
    // capsule part (the ship is heavier with crew aboard), aboard state
    // set, and stored on the ship (Vehicle::crew). Called from build_fleet
    // after each ship is placed; runtime copies (spawn_ship) deliberately
    // do NOT get crew.
    void spawn_crew(Vehicle *ship, System &sys);

    // Apply each ship's scenario (the startup spawn_vehicle pass).
    void apply_scenarios(System &sys);

    // Register a ship built out-of-band (the --radial-test ship).
    void add_ship(Vehicle *v, TerrainBody *home, const ScenarioDef *sc, int slot);

    // Build the startup fleet from the resolved entries: resolve each
    // entry's body (name -> System body, or the home body) + scenario
    // (name -> table), then place the ship + its startup crew. Returns the
    // first ship built (the natural active one) or nullptr for an empty
    // fleet. Throws std::runtime_error naming the entry + body on an
    // unknown body.
    Vehicle *build_fleet(const std::vector<FleetEntry> &entries, System &sys,
                         TerrainBody *home, const std::string &default_scenario);

private:
    // Ensure the (body, pad-site) pad exists; build it on demand (the
    // body's pads list, terrain.h).
    void place_pad(TerrainBody *hb, bool polar, const glm::dvec3 &dir, double pad_height);

    // One crew kerbal aboard (ship, part); the spawn_crew loop calls it per
    // capsule. Returns the kerbal (nullptr if the part is no capsule).
    Kerbal *spawn_crew_kerbal(Vehicle *ship, size_t part, System &sys);

    // De-duplicate a name across all the ships + crew (first keeps the
    // bare name, later ones get #2, #3 ..).
    std::string dedupName(System &sys, const std::string &nm);

    PartsCatalog part_catalog;
    Shader *partsshader;   // caller-owned
    TerrainBody *sun;      // caller-owned
};
