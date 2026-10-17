// ships.h -- ship construction and placement (the Ships builder).
// Ships and pads are owned by the body of their SoI (terrain.h); a seated
// kerbal is a seat on its capsule's Part (part.h).
#pragma once

#include <cstdint>    // uint64_t (a roster id)
#include <string>
#include <vector>

#include "body.h"     // Body, create_body
#include "camera.h"   // Camera
#include "frame.h"    // Frame
#include "mesh.h"     // Mesh
#include "science.h"  // Experiment (findings ride on the part)
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
struct Part;    // a seat occupant is a suit Part (part.h)

/* Canonical ship order: bodies in file order, each body's ships in order. Every
   Vehicle here is FLYABLE -- a seated kerbal is not in it (part.h). */
std::vector<Vehicle *> collectVehicles(System &sys);

/* Same walk, appended into a reused buffer (cleared here). */
void collectVehiclesInto(System &sys, std::vector<Vehicle *> &out);

/* How many vehicles the canonical walk yields, without handing back a list.
   For callers that only need the count (a per-frame readout). */
size_t countVehicles(System &sys);

/* A name made unique against the live fleet: first use keeps the bare name,
   later ones get " #2", " #3" .. Names are display text -- the save keys a
   vehicle by its slug (#266), so a clash no longer mis-resolves a load. Worth
   preventing anyway: the ship list, the tracking window, toasts and the
   [undock] line all show names, and two ships called 'racer' are unreadable
   there. Name-keyed resolution also still lives in the transfer planner, which
   looks a --transfer-target up by display name (transferplanner.cpp) -- a clash
   there does pick the wrong vessel, so this dedup is not only cosmetic.

   `roster` (nullable) additionally reserves the kerbals' names. A kerbal's
   display name IS its roster name and is never deduped, so without this a ship
   could be named 'Ash' while a kerbal named 'Ash' flies. */
std::string dedupVehicleName(System &sys, const std::string &base,
                             const CrewRoster *roster = nullptr);

class Ships {
public:
    /* The ship def a NEW suit is minted from (make_suit, via fill_seat). A save
       does NOT write it: a seat's part entry names the catalog PART by name
       (SavePart::part), and buildSeatParts resolves that against the parts file.
       Only a WALKING kerbal has a defPath, and buildKerbalFromSave now refuses
       one unless the def is a single kerbal part. */
    static constexpr const char *const kKerbalShipDef = "res/ships/kerbal.json";

    /* A def whose ROOT part is kerbal-typed describes a PERSON, and a person is
       exactly one part (part.h). True when `def` is consistent with that: no
       kerbal root, or exactly one part. The spawn path enforces it so the game
       can never write a save its loader then refuses -- a Kerbal minted from a
       multi-part def loses its extra parts on the way out (the writer reads a
       walker's part 0 only), and Game::kerbalBoard asserts one part the moment
       anyone boards. Each site keeps its own message: the e2e cases pin them. */
    static bool kerbalDefConsistent(const ShipDef &def);

    /* Mint a suit Part from a kerbal ship def: the occupant of a seat (part.h).
       Its Body is created but never enters the physics world -- a seat's mass
       rides the containment edge through Part::effectiveMass. Tanks are seeded
       full like Vehicle::init does; a load overwrites the contents from the
       save. Throws if the def is unusable. */
    Part *make_suit(const std::string &defPath) const;

    /* The ONE way a Part is minted from a catalog def (#278): render assets,
       collision hull, id, resources, findings, the chute canopy and the engine
       shroud. Everything the call sites used to differ in -- a save's own mass /
       hull_margin / fuel / findings vs the catalog's for a fresh part -- is an
       argument, so a field added to Part lands in every path or none. `fuel` and
       `experiments` may be null: a fresh part then takes capacity from the def,
       nothing in it, and no findings. Callers keep their own refusals (the e2e
       cases pin those messages), stage + chute deploy state, and identity. */
    Part *new_part_from_def(const PartDef &pd, double mass, double hull_margin,
                            const std::vector<double> *fuel,
                            const std::vector<Experiment> *experiments,
                            const std::string &id) const;

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

    /* Every seat of every capsule part of `ship`, drawn FIFO from the home pad's
       crew pool: a 3-berth capsule launches with three people while the pad can
       supply them, and the rest fly empty once it cannot. */
    void spawn_crew(Vehicle *ship, System &sys);

    // Mint one suit Part per roster entry onto the home pad (new game).
    // The pool is the cast: launch takes, recover returns, death does not.
    void mintCrewPool(const CrewRoster &roster, System &sys);

    // The pad unassigned kerbals stand on (the body's default VAB_LAUNCH_PAD).
    static StaticBuilding *homeCrewPad(System &sys);

    // Take a suit from the home pad (nullptr if the pool is empty). Caller owns it.
    static Part *takeCrew(System &sys);

    /* Take ONE named kerbal off the home pad by roster id (nullptr if nobody
       standing there wears that id). The VAB's seat assignments name people, so
       the pad is searched rather than served FIFO. Caller owns the result. */
    static Part *takeCrewById(System &sys, uint64_t profileId);

    // Park a suit back on the home pad (recover). Takes ownership.
    static void returnCrew(System &sys, Part *suit);

    /* Fill one seat of `ship` (part). False if the part is no capsule or the pad
       has no kerbal left -- an empty seat is a legal state, not a failure. The
       occupant is a suit Part parked in the capsule (part.h); no Kerbal vehicle
       exists until someone goes EVA.
       `suit` = the person to park (nullptr: draw FIFO from the pad);
       `slot` = which seat (0-based; -1 = the first free one). */
    bool fill_seat(Vehicle *ship, size_t part, System &sys,
                   Part *suit = nullptr, int slot = -1);

    // Apply each ship's scenario (the startup spawn_vehicle pass).
    void apply_scenarios(System &sys, double t);

    // Register an out-of-band ship; its builder already placed it, so it takes
    // no orbit slot.
    void add_ship(Vehicle *v, TerrainBody *home, const ScenarioDef *sc, double t);

    // Returns the first ship built (the natural active one), nullptr if empty.
    Vehicle *buildDebugStartShips(const std::vector<DebugStartShip> &entries,
                                  System &sys, double t);

private:
    PartsCatalog part_catalog;
    Shader *partsshader;   // caller-owned
    TerrainBody *sun;      // caller-owned
    CrewRoster &roster;    // caller-owned: the pool a seat draws from
};
