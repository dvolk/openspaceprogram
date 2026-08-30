// ships.h -- the runtime fleet: every ship in play, the per-ship
// bookkeeping (home body, starting scenario, slot within its (body,
// scenario) group), the space pads the pad ships stand on, and the
// operations that place / spawn / remove them (see ships.cpp).
//
// What stays in main() is the active-ship selection (which ship the player
// controls), the camera and the rails-warp state -- those are sim/UI state,
// not fleet data. Ships exposes the fleet data and the pure fleet
// operations; main() layers the active-ship handoff on top (its
// select_ship / remove_ship lambdas).
#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "body.h"     // Body, create_body
#include "camera.h"   // Camera
#include "frame.h"    // Frame
#include "mesh.h"     // Mesh
#include "model.h"    // Model
#include "shader.h"   // Shader
#include "shipdef.h"  // PartsCatalog
#include "texture.h"  // Texture
#include "terrain.h"  // TerrainBody, ComputeTerrainShadow
#include "vehicle.h"  // Vehicle, ScenarioDef

#include "fleet.h"    // FleetEntry (the startup fleet's entries)

struct System;  // only used by reference in the signatures below

/* A space pad -- a static building shared by every ship standing on the same
   (body, pad site). Drawn like terrain (culls itself when the active ship is
   not on the pad's body); the light source is the star. */
class StaticBuilding {
public:
    TerrainBody *parent;
    TerrainBody *sun = nullptr; // the star (light source)
    Body *body;

    void Draw(const Camera* camera, const TerrainBody *current, Frame *renderFrame) {
        if(current == parent) {
            const Frame *posFrame = parent->frame->getRotFrame();
            const float shadow = ComputeTerrainShadow(parent, posFrame,
                                                      GetPosition(body), sun);
            // Light direction at the pad (sun -> pad); stays defined if the
            // pad's body were ever the star (SunlightDir would be a zero vector).
            const glm::dvec3 pad_root =
                posFrame->root_orient * GetPosition(body) + posFrame->root_pos;
            glm::vec3 sunlightVec =
                glm::vec3(TerrainBody::LightDirFrom(pad_root, sun, renderFrame));
            body->Draw(camera, sunlightVec, shadow);
        }
    }
};

/* The runtime fleet. Owns the ships, the space pads, the shared pad model
   and the parts catalog it builds ships from. Non-copyable (raw ownership).

   Lifetime contract: the ships and pads are torn down by clear() -- which
   the destructor also runs -- and main() calls clear() at the point the old
   inline cleanup ran (BEFORE the System bodies and shaders are deleted), so
   the ships never outlive the bodies/shader they reference. */
class Ships {
public:
    /* parts_file:  the parts catalog to build ships from (owned, loaded here).
       partsshader: the part shader (caller-owned; must outlive us).
       sun:         the star (caller-owned; light source for ships + pads). */
    Ships(const std::string &parts_file, Shader *partsshader, TerrainBody *sun);
    ~Ships();

    Ships(const Ships &) = delete;
    Ships &operator=(const Ships &) = delete;

    // --- fleet access ----------------------------------------------------
    size_t size() const { return ships.size(); }
    bool empty() const { return ships.empty(); }
    Vehicle *at(size_t i) const { return ships[i]; }
    Vehicle *operator[](size_t i) { return ships[i]; }
    Vehicle *operator[](size_t i) const { return ships[i]; }

    // range-for support: for(auto *s : ships) in the tick / draw loops
    std::vector<Vehicle *>::iterator begin() { return ships.begin(); }
    std::vector<Vehicle *>::iterator end() { return ships.end(); }
    std::vector<Vehicle *>::const_iterator begin() const { return ships.begin(); }
    std::vector<Vehicle *>::const_iterator end() const { return ships.end(); }

    // per-ship bookkeeping (parallel to ships[])
    TerrainBody *homeOf(size_t i) const { return ship_homes[i]; }
    const ScenarioDef *scenarioOf(size_t i) const { return ship_sc[i]; }
    int slotOf(size_t i) const { return ship_slots[i]; }

    // the space pads (one per (body, pad site)); the render loop draws them
    const std::map<std::pair<TerrainBody *, bool>, StaticBuilding *> &pads() const {
        return space_ports;
    }

    // the parts catalog (for out-of-band ship builders, e.g. the radial test)
    const PartsCatalog &catalog() const { return part_catalog; }

    // --- fleet operations ------------------------------------------------
    // Place one ship on a body/scenario: load its def, slot it (next free
    // slot for the body+scenario), de-dup its name, make sure the pad exists,
    // build it on the pad. Returns the new ship's index. Does NOT apply the
    // scenario -- spawn_vehicle is the caller's job, so startup keeps a
    // single spawn pass.
    int place_ship(const std::string &shipDefPath, const std::string &wantName,
                   TerrainBody *hb, const ScenarioDef *sc);

    // Runtime spawn: place + apply the scenario + park on rails. Appended at
    // the end, so it is never the active one. Returns the new ship's index.
    int spawn_ship(const std::string &defPath, const std::string &wantName,
                   TerrainBody *hb, const ScenarioDef *sc, System &sys);

    // Apply each ship's scenario (the startup spawn_vehicle pass).
    void apply_scenarios(System &sys);

    // Remove a ship + its bookkeeping (the Vehicle dtor detaches the welds).
    // A raw bookkeeping op: no "last ship" guard -- the caller decides.
    void erase_ship(size_t idx);

    // Register a ship built out-of-band (the --radial-test ship).
    void add_ship(Vehicle *v, TerrainBody *home, const ScenarioDef *sc, int slot);

    // Build the startup fleet from the resolved entries: resolve each entry's
    // body (name -> System body, or the home body) + scenario (name -> table),
    // then place the ship. Throws std::runtime_error naming the entry + body
    // on an unknown body.
    void build_fleet(const std::vector<FleetEntry> &entries, System &sys,
                     TerrainBody *home, const std::string &default_scenario);

    // Tear down the ships + space pads. Idempotent; the destructor runs it.
    void clear();

private:
    // Ensure the (body, pad-site) pad exists; build it on demand.
    void place_pad(TerrainBody *hb, bool polar, const glm::dvec3 &dir, double pad_height);

    std::vector<Vehicle *> ships;
    std::vector<TerrainBody *> ship_homes;     // per ship: the body it starts on
    std::vector<const ScenarioDef *> ship_sc;  // per ship: its scenario
    std::vector<int> ship_slots;               // per ship: slot in its (body, scenario) group
    std::map<std::pair<TerrainBody *, bool>, StaticBuilding *> space_ports; // one per (body, pad site)

    PartsCatalog part_catalog;
    Shader *partsshader;   // caller-owned
    TerrainBody *sun;      // caller-owned

    Model *space_port_model; // shared by every pad; caller's shader, its own mesh+texture
};
