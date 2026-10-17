// ships.cpp -- ship construction + placement (see ships.h).
#include "ships.h"

#include <cassert>
#include <cstdio>
#include <fstream>
#include <numbers>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "body.h"     // create_body
#include "buildings.h" // find_launch_site
#include "crew.h"     // CrewRoster (the pool a seat draws from)
#include "eva.h"      // Kerbal (the crew characters)
#include "inventory.h" // inventorySetOwner (the suit's pockets ride its vehicle)
#include "mesh.h"     // get_mesh
#include "physics.h"  // setPosRot
#include "resdir.h"   // resdir::path
#include "shipdef.h"  // load_ship_def, ShipDef, PartsCatalog
#include "system.h"   // System
#include "texture.h"  // get_texture
#include "vehicle.h"  // build_ship, spawn_vehicle, Vehicle

// Ships sharing a (body, scenario) orbit are pushed this far apart along the
// orbit binormal so they don't spawn on top of each other.
static const double ORBIT_SLOT_SPACING = 20.0;

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

Ships::Ships(const std::string &parts_file, Shader *partsshader, TerrainBody *sun,
             CrewRoster &roster)
    : part_catalog(load_parts_catalog(resdir::path(parts_file).c_str())),
      partsshader(partsshader),
      sun(sun),
      roster(roster)
{
}

Vehicle *Ships::place_ship(const std::string &shipDefPath, const std::string &wantName,
                           TerrainBody *hb, const ScenarioDef *sc, System &sys,
                           Fleet &fleet, double t, const StaticBuilding *site)
{
    return place_ship_def(load_ship_def(resdir::path(shipDefPath).c_str(), part_catalog),
                          shipDefPath, wantName, hb, sc, sys, fleet, t, site);
}

bool Ships::kerbalDefConsistent(const ShipDef &def) {
    const bool kerbalRoot = !def.parts.empty()
        && def.parts[0].def != nullptr
        && def.parts[0].def->type == "kerbal";
    return !kerbalRoot || def.parts.size() == 1;
}

Vehicle *Ships::place_ship_def(const ShipDef &def, const std::string &defPath,
                               const std::string &wantName,
                               TerrainBody *hb, const ScenarioDef *sc, System &sys,
                               Fleet &fleet, double t, const StaticBuilding *site)
{
    /* slot feeds spawn_vehicle's orbit spread; pads do not slot. Every vessel in
       the fleet is flyable -- a seated kerbal is not in it (part.h). */
    int slot = 0;
    if(!sc->on_pad) {
        for(auto *s : fleet) {
            if(s->home == hb && s->scenario == sc) { slot++; }
        }
    }

    std::string nm = wantName.empty() ? def.name : wantName;
    if(nm.empty()) { nm = "Ship"; }
    // The roster's names are reserved too: a kerbal's name is never deduped.
    nm = dedupVehicleName(fleet, nm, &roster);

    /* A named site only means anything for a pad scenario: supportMargin below
       reads the site's hull as what the ship rests on, a lie if the base comes
       from terrain instead. */
    assert(site == nullptr || sc->on_pad);

    /* Pad scenarios stand on a world building (system JSON world_buildings). */
    glm::dvec3 pad_dir = glm::normalize(glm::dvec3(0.005, 0.005, 1.0));
    glm::dmat3 pad_orient = faceAlong(pad_dir);
    if(sc->on_pad) {
        if(site == nullptr) {
            site = find_launch_site(hb, "");
        }
        if(site == nullptr) {
            throw std::runtime_error(std::string("fleet: no launch site on ")
                                     + hb->name + " (add a world_buildings "
                                     + "entry with flag VAB_LAUNCH_PAD)");
        }
        pad_dir = site->dir;
        pad_orient = buildingOrient(pad_dir, site->heading_deg * std::numbers::pi / 180.0);
    }

    /* Root part type "kerbal" builds the EVA subclass (eva.h); anything else
       builds a plain ship. A kerbal's name comes from the roster, not the
       caller: --startship kerbal,... places a NAMED person, and the roster is
       what a save makes durable. Ship names take the roster's names as already
       taken (dedupVehicleName), so a ship and a kerbal can never share one --
       the two are different rows everywhere the player reads a name. */
    const bool is_kerbal = !def.parts.empty()
        && def.parts[0].def->type == "kerbal";
    /* Refuse the def, do not half-build it. A Kerbal minted from a multi-part
       def is a vehicle whose extra parts the save writer never sees (it reads a
       walker's part 0), so the game would write a save its own loader refuses
       -- and Game::kerbalBoard asserts one part the moment anyone boards. The
       --startship path catches this and reports it; vabLaunch guards first. */
    if(is_kerbal && !kerbalDefConsistent(def)) {
        throw std::runtime_error("ship def '"
                                 + (defPath.empty() ? def.name : defPath)
                                 + "': a kerbal-rooted def must be exactly one part (got "
                                 + std::to_string(def.parts.size()) + ")");
    }
    Vehicle *v = is_kerbal ? static_cast<Vehicle *>(new Kerbal) : new Vehicle;
    v->name = nm;
    v->defPath = defPath;
    v->home = hb;
    v->scenario = sc;
    v->slot = slot;
    v->m_parent = hb;
    v->sun = sun;
    v->frame = hb->rot_frame;
    /* Pad ships take the site pose verbatim: no per-slot offset (deliberate --
       whether an occupied pad is a problem is a gameplay decision, not
       placement arithmetic). Non-pad scenarios only need a staging pose. */
    const glm::dvec3 base = (sc->on_pad && site != nullptr)
        ? site->origin()
        : pad_dir * ((double)hb->GetTerrainHeight(glm::vec3(pad_dir)) + 5.0);
    // A pad is a static convex hull, so its own margin is what the ship's hull
    // margin meets; non-pad is repositioned by spawn_vehicle, so raw terrain.
    const double supportMargin = site != nullptr
        ? effectiveHullMargin(site->body) : terrainMargin();
    /* build_ship throws on a data bug (an unreadable mesh, a bad fuel link) and
       v is in no list yet, so nothing else would free it. */
    try {
        build_ship(v, def, partsshader, base, pad_orient, supportMargin);
    } catch(...) {
        delete v;
        throw;
    }
    /* A walker takes its suit from the home pad pool (part.h: a kerbal is a
       Part at one place). The minted def suit is replaced by the pooled one. */
    if(is_kerbal && !v->parts.empty()) {
        Part *suit = takeCrew(sys);
        if(suit != nullptr) {
            Part *minted = v->parts[0];
            suit->localPos = minted->localPos;
            suit->localRot = minted->localRot;
            suit->stage = minted->stage;
            suit->owner = v;
            v->parts[0] = suit;
            /* build_ship left `controller` and the compound's part list on the
               minted suit. Re-point before the delete, then finalize: the
               pooled suit may weigh differently (partly-burned tank, pocket
               items), and the hull/compoundParts must follow parts[0] or both
               dangle at freed memory (Kerbal::applyEva reads controller). */
            if(v->controller == minted) { v->controller = suit; }
            delete minted;
            inventorySetOwner(suit, v);
            v->finalize();
            if(suit->kerbal != nullptr) { v->name = suit->kerbal->name; }
        } else {
            printf("[crew] pool spent: '%s' has no identity\n", nm.c_str());
        }
    }
    v->setVelocity(glm::dvec3(0, 0, 0));
    // Re-home: starts the flight journal at `t`. Then the fleet adopts it.
    v->setSoi(hb->rot_frame, t);
    fleet.push_back(v);
    return v;
}

std::string dedupVehicleName(const Fleet &fleet, const std::string &base,
                             const CrewRoster *roster)
{
    /* No snapshot: the fleet is the live container, and this call cannot change
       its membership. The retry loop below only reads names. */
    std::string nm = base;
    int n = 2;
    while(true) {
        bool taken = roster != nullptr && roster->byName(nm) != nullptr;
        for(Vehicle *s : fleet) {
            if(s->name == nm) { taken = true; break; }
        }
        if(!taken) { return nm; }
        nm = base + " #" + std::to_string(n);
        n++;
    }
}

/* The one way a Part is minted from a catalog def (ships.h, #278). Four call
   sites used to carry this block themselves and had already drifted: only the
   def-spawn path resolved a shroud, only the attached-part loader resolved a
   chute canopy, and only a fresh suit resolved a ship-level hull_margin. */
Part *Ships::new_part_from_def(const PartDef &pd, double mass, double hull_margin,
                               const std::vector<double> *fuel,
                               const std::vector<Experiment> *experiments,
                               const std::string &id) const {
    Mesh *mesh = get_mesh(std::string("res/") + pd.mesh);
    Texture *tex = get_texture(std::string("res/") + pd.texture);
    Part *p = new Part;
    p->body = create_part_body(mesh, partsshader, tex, (float)mass, hull_margin);
    p->def = &pd;
    p->id = id;
    /* The chute canopy and the engine shroud are resolved at build time, so every
       path resolves them. Only a chute def has the first and only an engine the
       second today; a def that grows one cannot be forgotten here. The shroud
       especially: it was resolved ONLY on the def-spawn path, so every loaded
       engine lost the cylinder that hides what it shrouds (#278). */
    if(!pd.chute_mesh.empty()) {
        p->chute_mesh = get_mesh(std::string("res/") + pd.chute_mesh);
        p->chute_texture = get_texture(std::string("res/") + pd.chute_texture);
    }
    if(!pd.shroud.empty()) {
        p->shroud = get_mesh(std::string("res/") + pd.shroud);
        p->shroud_texture = get_texture(std::string("res/") + pd.shroud_texture);
    }
    for(int r = 0; r < (int)ResourceType::Num; r++) {
        p->resources.capacity[r] = pd.capacity[r];
        p->resources.current[r] = (fuel != nullptr && r < (int)fuel->size())
            ? (float)(*fuel)[r] : 0.0f;
    }
    if(experiments != nullptr) { p->experiments = *experiments; }
    return p;
}

/* Mint a suit Part from a kerbal ship def (the occupant of a seat, part.h). The
   Body exists to carry mass + render assets; it never enters the physics world,
   because a seat's mass rides the containment edge (Part::effectiveMass), not
   the capsule's compound. */
Part *Ships::make_suit(const std::string &defPath) const {
    ShipDef def = load_ship_def(resdir::path(defPath).c_str(), part_catalog);
    /* A seat is exactly ONE Part, and it must be a kerbal-type part: that is
       what makes it a seat rather than an inventory item (Part::isSeat). A
       multi-part def would silently drop the rest, and parking a non-kerbal
       part in a capsule breaks checkPartInvariants -- a hand-edited save's
       defPath must be refused here, not asserted on two calls later. */
    if(def.parts.size() != 1) {
        throw std::runtime_error(std::string("suit: '") + defPath
                                 + "' is not exactly one part (got "
                                 + std::to_string(def.parts.size()) + ")");
    }
    const PartDef *pd = def.parts[0].def;
    if(pd->type != "kerbal") {
        throw std::runtime_error(std::string("suit: '") + defPath + "' is a '"
                                 + pd->type + "' part, not a kerbal suit");
    }
    // A fresh part takes the def's own mass and the def's ship-level margin.
    Part *suit = new_part_from_def(*pd, pd->mass,
                                   resolveHullMargin(def.hull_margin,
                                                     pd->hull_margin),
                                   nullptr, nullptr, def.parts[0].id);
    /* Seeded like Vehicle::init does for a tank: a partly-burned suit re-seeds
       full here, which is right for a fresh crew member. */
    if(suit->isTank()) {
        for(int r = 0; r < (int)ResourceType::Num; r++) {
            suit->resources.current[r] = suit->resources.capacity[r];
        }
    }
    return suit;
}

StaticBuilding *Ships::homeCrewPad(System &sys) {
    /* The home body's pad is THE pad; a body-order scan alone would stand the
       cast on whichever launch site happens to come first in sys.bodies. */
    if(sys.home != nullptr) {
        if(StaticBuilding *site = find_launch_site(sys.home, "")) { return site; }
    }
    for(TerrainBody *b : sys.bodies) {
        StaticBuilding *site = find_launch_site(b, "");
        if(site != nullptr) { return site; }
    }
    return nullptr;
}

Part *Ships::takeCrew(System &sys) {
    StaticBuilding *home = homeCrewPad(sys);
    if(home == nullptr || home->crew.empty()) { return nullptr; }
    // First in, first out: the pool is minted in roster order (the shipped file
    // is id order, so a fresh cast hands out the lowest id first). A returned
    // suit goes to the BACK (returnCrew), so the line round-robins after a
    // recover rather than always serving the lowest free id.
    Part *suit = home->crew.front();
    home->crew.erase(home->crew.begin());
    suit->container = nullptr;
    /* Unown the suit AND whatever its pockets hold: a caller that forgets leaves
       a dangling Vehicle* in the containment tree, and checkPartInvariants does
       not look at a seat occupant's owner. Each caller then wires the vehicle
       the suit lands on (fill_seat, place_ship_def). */
    inventorySetOwner(suit, nullptr);
    return suit;
}

Part *Ships::takeCrewById(System &sys, uint64_t profileId) {
    StaticBuilding *home = homeCrewPad(sys);
    if(home == nullptr) { return nullptr; }
    for(size_t i = 0; i < home->crew.size(); i++) {
        Part *suit = home->crew[i];
        if(suit->kerbal == nullptr || suit->kerbal->id != profileId) { continue; }
        home->crew.erase(home->crew.begin() + (std::ptrdiff_t)i);
        suit->container = nullptr;
        inventorySetOwner(suit, nullptr);   // as takeCrew: no dangling owner
        return suit;
    }
    return nullptr;
}

void Ships::returnCrew(System &sys, Part *suit) {
    if(suit == nullptr) { return; }
    StaticBuilding *home = homeCrewPad(sys);
    if(home == nullptr) {
        /* Nowhere to stand, so the person is gone -- and LOUDLY so: a system
           with no VAB_LAUNCH_PAD eats its crew at every recover, and a silent
           delete reads exactly like a successful flight home. */
        printf("[crew] '%s' is gone: this system has no VAB_LAUNCH_PAD to "
               "stand them on\n",
               (suit->kerbal ? suit->kerbal->name : "?").c_str());
        fflush(stdout);
        delete suit;
        return;
    }
    suit->container = nullptr;
    inventorySetOwner(suit, nullptr);
    home->crew.push_back(suit);
}

void Ships::mintCrewPool(const CrewRoster &roster, System &sys) {
    StaticBuilding *home = homeCrewPad(sys);
    if(home == nullptr) {
        printf("[crew] no VAB_LAUNCH_PAD: nowhere to stand the crew pool\n");
        return;
    }
    for(const KerbalProfile &p : roster.profiles) {
        Part *suit = nullptr;
        try {
            suit = make_suit(kKerbalShipDef);
        } catch(const std::exception &e) {
            printf("Crew: %s\n", e.what());
            continue;
        }
        suit->kerbal = std::make_unique<KerbalProfile>(p);
        home->crew.push_back(suit);
    }
    printf("[crew] %zu kerbal(s) standing at '%s'\n",
           home->crew.size(), home->id.c_str());
}

/* Fill one seat: park a suit Part in the capsule (ship, part) as a seat occupant
   (part.h). There is no Kerbal vehicle here -- minting one is what going EVA
   means (Game::kerbalEVA). */
bool Ships::fill_seat(Vehicle *ship, size_t part, System &sys, Part *suit,
                      int slot) {
    if(part >= ship->parts.size()) { return false; }
    Part *capPart = ship->parts[part];
    const PartDef *capDef = capPart->def;
    if(capDef->crew_capacity <= 0) { return false; }

    if(suit == nullptr) {
        if(capPart->seatCount() >= capDef->crew_capacity) { return false; }
        /* Draw the person from the home pad's pool. An empty pool leaves the
           seat EMPTY and the ship flies uncrewed: a legal state, not a failure. */
        suit = takeCrew(sys);
        if(suit == nullptr) {
            printf("Crew: no kerbal left for '%s' part %zu (%d seats)\n",
                   ship->name.c_str(), part, capDef->crew_capacity);
            return false;
        }
    }

    /* The seat edge: the capsule OWNS the suit (so ~Part frees it). `owner` is
       the vehicle the container sits on, as for an item. Identity rides the
       suit (Part::kerbal) -- the person owns it. A named assignment has already
       been checked against the build, so a refused slot here is a bug. */
    assert(capPart->addSeat(suit, slot < 0 ? (size_t)-1 : (size_t)slot));
    suit->owner = ship;
    /* Whatever the suit carries in its pockets rides the ship too (a recovered
       kerbal comes home with its loot). */
    inventorySetOwner(suit, ship);
    // Rebuild so the compound carries the crew mass.
    ship->rebuildCompound();
    printf("Crew: '%s' aboard '%s' part %zu (%d/%d)\n",
           (suit->kerbal ? suit->kerbal->name : "?").c_str(),
           ship->name.c_str(), part,
           capPart->seatCount(), capDef->crew_capacity);
    return true;
}

void Ships::spawn_crew(Vehicle *ship, System &sys) {
    for(size_t i = 0; i < ship->parts.size(); i++) {
        /* EVERY seat, not one per capsule: a 3-berth capsule launches with a
           full crew while the pad can supply one. fill_seat says false when the
           pad is spent -- then the remaining seats fly empty, so stop asking. */
        const int cap = ship->parts[i]->def->crew_capacity;
        for(int s = 0; s < cap; s++) {
            if(!fill_seat(ship, i, sys)) { break; }
        }
    }
}

void Ships::apply_scenarios(Fleet &fleet, System &sys, double t) {
    /* No snapshot: spawn_vehicle re-homes a ship (setSoi), and re-homing no
       longer moves it between containers, so the fleet cannot change under the
       loop. */
    for(auto *s : fleet) {
        if(s->scenario == nullptr) { continue; }
        spawn_vehicle(s, *s->scenario, s->home, sys, ORBIT_SLOT_SPACING * (double)s->slot, t);
    }
}

void Ships::add_ship(Fleet &fleet, Vehicle *v, TerrainBody *home,
                     const ScenarioDef *sc, double t)
{
    v->home = home;
    v->scenario = sc;
    v->slot = 0;   // the builder placed it; there is no slot to space out
    v->setSoi(v->frame, t);   // journal start (the builder set frame)
    fleet.push_back(v);
}

Vehicle *Ships::buildDebugStartShips(const std::vector<DebugStartShip> &entries,
                                     System &sys, Fleet &fleet, double t)
{
    Vehicle *first = nullptr;
    for(size_t i = 0; i < entries.size(); i++) {
        const DebugStartShip &fe = entries[i];
        TerrainBody *hb = sys.find(fe.body);
        if(hb == nullptr) {
            std::string avail;
            for(size_t k = 0; k < sys.bodies.size(); k++) {
                if(k) { avail += ", "; }
                avail += sys.bodies[k]->name;
            }
            throw std::runtime_error("start ships: ship entry " + std::to_string(i)
                                     + ": unknown body '" + fe.body
                                     + "' (available: " + avail + ")");
        }
        const ScenarioDef *sc = scenario_by_name(fe.scenario);
        /* A named pad only means anything on a pad bed: place_ship's
           supportMargin reads the site's hull as what the ship rests on, a lie
           if the base comes from terrain instead. */
        const StaticBuilding *site = nullptr;
        if(!fe.site.empty()) {
            if(!sc->on_pad) {
                throw std::runtime_error("start ships: entry " + std::to_string(i)
                                         + ": \"site\" needs an on_pad scenario, "
                                         "not '" + fe.scenario + "'");
            }
            site = find_launch_site(hb, fe.site);
            if(site == nullptr) {
                std::string avail;
                for(const StaticBuilding *p : hb->pads) {
                    if(!p->hasFlag("VAB_LAUNCH_PAD")) { continue; }
                    if(!avail.empty()) { avail += ", "; }
                    avail += p->id;
                }
                throw std::runtime_error("start ships: entry " + std::to_string(i)
                                         + ": '" + fe.site + "' is not a launch pad on "
                                         + hb->name + " (available: " + avail + ")");
            }
        }
        Vehicle *v = place_ship(fe.ship, fe.name, hb, sc, sys, fleet, t, site);
        spawn_crew(v, sys);
        if(first == nullptr) { first = v; }
    }
    return first;
}
