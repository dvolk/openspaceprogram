// ships.h -- ship construction and placement (the Ships builder).
// Ships and pads are owned by the body of their SoI (terrain.h); crew live on
// their ship (vehicle.h).
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

// One ship the game spawns at boot. The first four fields are required; `site`
// is an optional pad id, meaningful only for an on_pad scenario.
struct DebugStartShip {
    std::string ship;      // ship def path
    std::string name;      // display name
    std::string body;      // the body it sits on (a name in the system)
    std::string scenario;  // the spawn scenario (a --startship field)
    std::string site;      // pad id ("" = the body's default site)
};

// The parsed "ships" array.
struct DebugStartShips {
    std::vector<DebugStartShip> ships;
};

// Parse + validate the debug-start-ships JSON; throws std::runtime_error naming
// the file + the offending entry.
DebugStartShips loadDebugStartShips(const char *path);

struct System;
struct Kerbal;  // crew characters (eva.h)
struct CrewRoster;  // the named crew pool (crew.h)

/* Canonical ship order: bodies in file order, each body's ships in order, each
   ship's crew right after it. */
std::vector<Vehicle *> collectVehicles(System &sys);

/* Same walk, appended into a reused buffer (cleared here). */
void collectVehiclesInto(System &sys, std::vector<Vehicle *> &out);

/* How many vehicles the canonical walk yields, without handing back a list.
   For callers that only need the count (a per-frame readout). */
size_t countVehicles(System &sys);

/* A name made unique against the live fleet: first use keeps the bare name,
   later ones get " #2", " #3" .. Every ship-naming site must go through this,
   because the save keys a vehicle by DISPLAY NAME (SaveShip::aboard,
   SaveMeta::active_ship) -- two vehicles sharing a name make a load resolve the
   wrong one.

   `roster` (nullable) additionally reserves the kerbals' names. A kerbal's
   display name IS its roster name and is never deduped, so without this a ship
   could be named 'Ash' while a kerbal named 'Ash' flies. */
std::string dedupVehicleName(System &sys, const std::string &base,
                             const CrewRoster *roster = nullptr);

class Ships {
public:
    /* parts_file:  parts catalog to build from (owned, loaded here).
       partsshader: the part shader (caller-owned; must outlive us).
       sun:         the star (caller-owned; lights ships + pads).
       roster:      the crew pool a capsule seat draws from (caller-owned; must
                    outlive us). We only borrow it to NAME a minted kerbal --
                    the roster is game state, not part of the builder. */
    Ships(const std::string &parts_file, Shader *partsshader, TerrainBody *sun,
          CrewRoster &roster);
    /* Ships + pads die with the bodies (terrain.h); shader + star are borrowed. */
    ~Ships() = default;

    Ships(const Ships &) = delete;
    Ships &operator=(const Ships &) = delete;

    // Parts catalog for out-of-band ship builders.
    const PartsCatalog &catalog() const { return part_catalog; }

    // Re-point the star on a system switch.
    void setSun(TerrainBody *sun) { this->sun = sun; }

    // --- placement (the ships land in the body's list, not here) ---------
    // `t` (sim time) is the flight-journal stamp for the placement.
    // `site`: the pad to stand on (on_pad scenarios); nullptr -> the body's
    // default site, its first VAB_LAUNCH_PAD.
    // Scenario is NOT applied -- spawn_vehicle is the caller's job.
    Vehicle *place_ship(const std::string &shipDefPath, const std::string &wantName,
                        TerrainBody *hb, const ScenarioDef *sc, System &sys,
                        double t, const StaticBuilding *site = nullptr);

    // Same, from an already-loaded def.
    Vehicle *place_ship_def(const ShipDef &def, const std::string &defPath,
                            const std::string &wantName,
                            TerrainBody *hb, const ScenarioDef *sc, System &sys,
                            double t, const StaticBuilding *site = nullptr);

    // One kerbal ABOARD each of `ship`'s capsule parts.
    void spawn_crew(Vehicle *ship, System &sys, double t);

    // Apply each ship's scenario (the startup spawn_vehicle pass).
    void apply_scenarios(System &sys, double t);

    // Register an out-of-band ship; its builder already placed it, so it takes
    // no orbit slot.
    void add_ship(Vehicle *v, TerrainBody *home, const ScenarioDef *sc, double t);

    // Returns the first ship built (the natural active one), nullptr if empty.
    Vehicle *buildDebugStartShips(const std::vector<DebugStartShip> &entries,
                                  System &sys, double t);

private:
    /* One crew kerbal aboard (ship, part). nullptr if the part is no capsule,
       or if the roster has no kerbal left -- an empty seat is a legal state,
       not a failure. */
    Kerbal *spawn_crew_kerbal(Vehicle *ship, size_t part, System &sys, double t);

    PartsCatalog part_catalog;
    Shader *partsshader;   // caller-owned
    TerrainBody *sun;      // caller-owned
    CrewRoster &roster;    // caller-owned: the pool a seat draws from
};
