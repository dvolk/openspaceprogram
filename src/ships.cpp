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

void collectVehiclesInto(System &sys, std::vector<Vehicle *> &out) {
    out.clear();
    /* Every vessel in the world. A seated kerbal is NOT here -- it is a seat on
       its capsule's Part (part.h), so the walk has no crew leg. */
    for(auto *b : sys.bodies) {
        for(auto *s : b->ships) { out.push_back(s); }
    }
}

std::vector<Vehicle *> collectVehicles(System &sys) {
    std::vector<Vehicle *> out;
    collectVehiclesInto(sys, out);
    return out;
}

size_t countVehicles(System &sys) {
    // Walked through collectVehiclesInto so the membership rule cannot drift
    // from the canonical list. The buffer is reused, so this costs no
    // allocation per call.
    static thread_local std::vector<Vehicle *> buf;
    collectVehiclesInto(sys, buf);
    return buf.size();
}

Ships::Ships(const std::string &parts_file, Shader *partsshader, TerrainBody *sun,
             CrewRoster &roster)
    : part_catalog(load_parts_catalog(resdir::path(parts_file).c_str())),
      partsshader(partsshader),
      sun(sun),
      roster(roster)
{
}

/* The next kerbal for a seat: the roster entry nobody in `sys` is wearing.
   Derived from the world rather than tracked on the roster, so a delete that
   forgets to "release" a person cannot shrink the pool forever. Two sources: a
   walking kerbal wears its id on the vehicle, a SEATED one only on the suit Part
   parked in its capsule (part.h). nullptr = the pool is spent; the seat stays
   empty. */
static const KerbalProfile *claimProfile(CrewRoster &roster, System &sys) {
    static thread_local std::vector<uint64_t> worn;
    static thread_local std::vector<Vehicle *> fleet;
    worn.clear();
    // collectVehiclesInto, not collectVehicles: this runs once per seat, and
    // the fleet walk is the allocation to avoid (see countVehicles).
    collectVehiclesInto(sys, fleet);
    for(Vehicle *v : fleet) {
        if(!v->isEva()) { continue; }
        const uint64_t id = static_cast<Kerbal *>(v)->profileId;
        if(id == 0) { continue; }
        worn.push_back(id);
    }
    for(Vehicle *v : fleet) {
        for(Part *p : v->parts) {
            for(Part *s : p->seats()) {
                if(s->seatProfile != 0) { worn.push_back(s->seatProfile); }
            }
        }
    }
    /* The invariant the whole pool rests on: one person, one wearer. A repeat
       means two things claim the same kerbal -- which is what a name-keyed load
       mis-resolving used to do -- and the pool would then lose people with
       nothing left to show for it. */
    for(size_t i = 0; i < worn.size(); i++) {
        for(size_t j = 0; j < i; j++) { assert(worn[i] != worn[j]); }
    }
    return roster.claim(worn);
}

Vehicle *Ships::place_ship(const std::string &shipDefPath, const std::string &wantName,
                           TerrainBody *hb, const ScenarioDef *sc, System &sys,
                           double t, const StaticBuilding *site)
{
    return place_ship_def(load_ship_def(resdir::path(shipDefPath).c_str(), part_catalog),
                          shipDefPath, wantName, hb, sc, sys, t, site);
}

Vehicle *Ships::place_ship_def(const ShipDef &def, const std::string &defPath,
                               const std::string &wantName,
                               TerrainBody *hb, const ScenarioDef *sc, System &sys,
                               double t, const StaticBuilding *site)
{
    /* slot feeds spawn_vehicle's orbit spread; pads do not slot. Every vessel in
       the walk is flyable -- a seated kerbal is not in it (part.h). */
    int slot = 0;
    if(!sc->on_pad) {
        for(auto *s : collectVehicles(sys)) {
            if(s->home == hb && s->scenario == sc) { slot++; }
        }
    }

    std::string nm = wantName.empty() ? def.name : wantName;
    if(nm.empty()) { nm = "Ship"; }
    // The roster's names are reserved too: a kerbal's name is never deduped.
    nm = dedupVehicleName(sys, nm, &roster);

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
    Vehicle *v = is_kerbal ? static_cast<Vehicle *>(new Kerbal) : new Vehicle;
    v->name = nm;
    if(is_kerbal) {
        Kerbal *k = static_cast<Kerbal *>(v);
        const KerbalProfile *p = claimProfile(roster, sys);
        if(p != nullptr) {
            k->profileId = p->id;
            v->name = p->name;
        } else {
            printf("[crew] roster spent: '%s' has no identity\n", nm.c_str());
        }
    }
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
    v->setVelocity(glm::dvec3(0, 0, 0));
    // Re-home: enters the body's ships list and starts the flight journal at `t`.
    v->setSoi(hb->rot_frame, t);
    return v;
}

std::string dedupVehicleName(System &sys, const std::string &base,
                             const CrewRoster *roster)
{
    /* Walked once, into a reused buffer: the retry loop below must not pay an
       allocation per attempt (the countVehicles pattern). The fleet cannot
       change during this call, so one snapshot covers every attempt. */
    static thread_local std::vector<Vehicle *> fleet;
    collectVehiclesInto(sys, fleet);
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

/* Mint a suit Part from a kerbal ship def (the occupant of a seat, part.h). The
   Body exists to carry mass + render assets; it never enters the physics world,
   because a seat's mass rides the containment edge (Part::effectiveMass), not
   the capsule's compound. */
Part *Ships::make_suit(const std::string &defPath) const {
    ShipDef def = load_ship_def(resdir::path(defPath).c_str(), part_catalog);
    if(def.parts.empty()) {
        throw std::runtime_error(std::string("suit: '") + defPath
                                 + "' has no root part");
    }
    const PartDef *pd = def.parts[0].def;
    Mesh *mesh = get_mesh(std::string("res/") + pd->mesh);
    Texture *tex = get_texture(std::string("res/") + pd->texture);
    Body *b = create_part_body(mesh, partsshader, tex, (float)pd->mass,
                               resolveHullMargin(def.hull_margin, pd->hull_margin));
    Part *suit = new Part;
    suit->body = b;
    suit->def = pd;
    suit->id = def.parts[0].id;
    /* Seeded like Vehicle::init does for a tank: a partly-burned suit re-seeds
       full here, which is right for a fresh crew member. */
    if(suit->isTank()) {
        for(int r = 0; r < (int)ResourceType::Num; r++) {
            suit->resources.capacity[r] = pd->capacity[r];
            suit->resources.current[r]  = pd->capacity[r];
        }
    }
    return suit;
}

/* Fill one seat: park a suit Part in the capsule (ship, part) as a seat occupant
   (part.h). There is no Kerbal vehicle here -- minting one is what going EVA
   means (Game::kerbalEVA). */
bool Ships::fill_seat(Vehicle *ship, size_t part, System &sys) {
    if(part >= ship->parts.size()) { return false; }
    Part *capPart = ship->parts[part];
    const PartDef *capDef = capPart->def;
    if(capDef->crew_capacity <= 0) { return false; }

    /* Draw the person from the roster. A spent pool leaves the seat EMPTY and
       the ship flies uncrewed: a legal state, not a failure. */
    const KerbalProfile *p = claimProfile(roster, sys);
    if(p == nullptr) {
        printf("Crew: no kerbal left for '%s' part %zu (%d seats)\n",
               ship->name.c_str(), part, capDef->crew_capacity);
        return false;
    }

    Part *suit = nullptr;
    try {
        suit = make_suit(kKerbalShipDef);
    } catch(const std::exception &e) {
        printf("Crew: %s\n", e.what());
        return false;
    }

    /* The seat edge: containment, and the capsule OWNS the suit (so ~Part frees
       it). `owner` is the vehicle the container sits on, as for an item. */
    capPart->contents.push_back(suit);
    capPart->ownedContents.push_back(suit);
    suit->container = capPart;
    suit->owner = ship;
    suit->seatProfile = p->id;
    // Rebuild so the compound carries the crew mass.
    ship->rebuildCompound();
    printf("Crew: '%s' aboard '%s' part %zu (%d/%d)\n",
           p->name.c_str(), ship->name.c_str(), part,
           capPart->seatCount(), capDef->crew_capacity);
    return true;
}

void Ships::spawn_crew(Vehicle *ship, System &sys) {
    for(size_t i = 0; i < ship->parts.size(); i++) {
        if(ship->parts[i]->def->crew_capacity > 0) {
            fill_seat(ship, i, sys);
        }
    }
}

void Ships::apply_scenarios(System &sys, double t) {
    for(auto *b : sys.bodies) {
        /* Snapshot, not a reference: spawn_vehicle can re-home a ship into
           another body's list (setSoi moves it). */
        const std::vector<Vehicle *> ships = b->ships;
        for(auto *s : ships) {
            if(s->scenario == nullptr) { continue; }
            spawn_vehicle(s, *s->scenario, s->home, sys, ORBIT_SLOT_SPACING * (double)s->slot, t);
        }
    }
}

void Ships::add_ship(Vehicle *v, TerrainBody *home, const ScenarioDef *sc, double t)
{
    v->home = home;
    v->scenario = sc;
    v->slot = 0;   // the builder placed it; there is no slot to space out
    v->setSoi(v->frame, t);   // register + journal (the builder set frame)
}

Vehicle *Ships::buildDebugStartShips(const std::vector<DebugStartShip> &entries,
                                     System &sys, double t)
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
        Vehicle *v = place_ship(fe.ship, fe.name, hb, sc, sys, t, site);
        spawn_crew(v, sys);
        if(first == nullptr) { first = v; }
    }
    return first;
}
