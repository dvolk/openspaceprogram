// vehicle.cpp -- Vehicle's method definitions + ship building + spawn scenarios.
#include "vehicle.h"

#include <algorithm>
#include <bit>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <map>
#include <numbers>
#include <stdexcept>

#include "buildings.h" // find_launch_site, buildingOrient (a pad's own axes)
#include "crew.h"      // CrewRoster (the names a vehicle may not take)
#include "eva.h"       // Kerbal (a kerbal-rooted def builds this subclass)
#include "game.h"      // Game (the parts catalog, the part shader, the star)
#include "inventory.h" // inventorySetOwner (the suit's pockets ride its vehicle)
#include "resdir.h"    // resdir::path
#include "system.h"    // System (spawn_vehicle resolves the home body's SOI)
#include "drag.h"      // the drag law (airDensity / dragForce)
#include "gimbal.h"    // the engine-gimbal law (gimbalDir / gimbalStep)
#include "rcs.h"       // the RCS nozzle allocator (rcsAllocate)
#include "constants.h" // kSoiMargin, kShipGround*, kTerrainBand

/* One rigid body per part, welded parent-first in the def's construction order.
   The catalog must outlive the ship (the partDefs point into it). */
void build_ship_structure(Vehicle *ship, const ShipDef &def, Shader *partsshader)
{
    printf("Building ship '%s' (%d parts)\n", def.name.c_str(), (int)def.parts.size());

    /* 0) partition into physical parts (a Body each) and virtual fuel links; remap
       parent indices -- a fuel link can never be a parent. */
    std::vector<ShipPart> physical;
    std::vector<const ShipPart *> links;
    std::map<size_t, size_t> physIndex;  // def.parts index -> physical index
    for(size_t i = 0; i < def.parts.size(); i++) {
        const ShipPart &sp = def.parts[i];
        if(sp.isFuelLink()) {
            links.push_back(&sp);
            continue;
        }
        physIndex[i] = physical.size();
        physical.push_back(sp);
        if(physical.back().parent >= 0) {
            auto it = physIndex.find((size_t)physical.back().parent);
            if(it == physIndex.end()) {
                throw std::runtime_error("build_ship: parent of '" + sp.id
                                         + "' is a fuel link (virtual parts cannot be parents)");
            }
            physical.back().parent = (int)it->second;
        }
    }

    int controllerIdx = def.controllerIndex();
    auto cit = physIndex.find((size_t)controllerIdx);
    if(cit == physIndex.end()) {
        throw std::runtime_error("build_ship: controller is a fuel link");
    }
    const size_t n = physical.size();

    /* 1) relative poses in a canonical frame: root at origin, +Z = stack axis. */
    std::vector<glm::dvec3> pos(n);
    std::vector<glm::dmat3> rot(n);
    pos[0] = glm::dvec3(0.0);
    rot[0] = glm::dmat3(1.0);
    for(size_t i = 1; i < n; i++) {
        const ShipPart &sp = physical[i];
        const ShipPart &pp = physical[(size_t)sp.parent];
        const glm::dvec3 &pPos = pos[(size_t)sp.parent];
        const glm::dmat3 &pRot = rot[(size_t)sp.parent];
        // one solver for flight and the editor, so they can never disagree
        const AttachPose ap = solveEdge(pPos, pRot, *pp.def, *sp.def, sp.attach,
                                        sp.parentNode, sp.childNode,
                                        sp.contactPoint, sp.contactNormal,
                                        sp.angle, sp.roll, sp.offset);
        pos[i] = ap.childPos;
        rot[i] = ap.childRot;
    }

    for(size_t i = 0; i < n; i++) {
        const PartDef &pd = *physical[i].def;
        /* The one mint (#278): render assets, collision hull, id, resources,
           the chute canopy and the engine shroud. build_ship hand-rolled this
           block, so a field added to Part landed in the save path and not here
           (#278 caught exactly that for the shroud). A fresh ship takes the
           def's ship-level margin, empty tanks and no findings; build_ship's
           Vehicle::init() call then tops every tank up. */
        Part *part = make_part(pd,
                               resolveHullMargin(def.hull_margin, pd.hull_margin),
                               nullptr, nullptr, physical[i].id, partsshader);
        part->stage = physical[i].stage;

        if(i == 0) {
            ship->setRoot(part);
        } else {
            const ShipPart &sp = physical[i];
            /* pos[i]/rot[i] are already ship-local (S = the root's frame): the pad
               shift and world orient apply uniformly, so they cancel. */
            ship->attach(part, (size_t)sp.parent, pos[i], rot[i]);
        }
    }
    /* The shroud state is LIVE -- staging can drop the child below. */
    for(size_t i = 0; i < n; i++) {
        Part *p = ship->parts[i];
        if(p->shroud == nullptr) { continue; }
        printf("[shroud] %s: %s (%s over %s)\n", p->id.c_str(),
               ship->hasChildBelow(p) ? "shrouded" : "bare",
               p->def->shroud.c_str(), p->def->mesh.c_str());
        fflush(stdout);
    }

    ship->controller = ship->parts[cit->second];

    /* 4) resolve the fuel links (ids -> Part*). Reject cycles: the drain model
       requires a DAG. */
    if(!links.empty()) {
        std::map<std::string, Part *> idToPart;
        for(size_t i = 0; i < n; i++) {
            idToPart[physical[i].id] = ship->parts[i];
        }
        for(size_t k = 0; k < links.size(); k++) {
            const ShipPart *lk = links[k];
            auto f = idToPart.find(lk->from);
            auto t = idToPart.find(lk->to);
            if(f == idToPart.end()) {
                throw std::runtime_error("build_ship: fuel link '" + lk->id
                                         + "' references unknown part '" + lk->from + "'");
            }
            if(t == idToPart.end()) {
                throw std::runtime_error("build_ship: fuel link '" + lk->id
                                         + "' references unknown part '" + lk->to + "'");
            }
            ship->fuelLinks.push_back(Vehicle::FuelLink{ f->second, t->second });
        }
        // Cycle check over fuel GROUPS (not parts).
        ship->buildFuelGroups();
        std::map<int, std::vector<int>> dag;  // fuelGroup -> outgoing fuelGroups
        for(size_t k = 0; k < ship->fuelLinks.size(); k++) {
            int a = ship->fuelLinks[k].from->fuelGroup;
            int b = ship->fuelLinks[k].to->fuelGroup;
            if(a < 0 || b < 0) { continue; }  // an endpoint in no group (a barrier)
            if(a != b) { dag[a].push_back(b); }
        }
        std::map<int, int> color;  // 0 = white, 1 = grey, 2 = black
        for(auto it = dag.begin(); it != dag.end(); ++it) { color[it->first] = 0; }
        for(size_t k = 0; k < ship->fuelLinks.size(); k++) {
            int a = ship->fuelLinks[k].from->fuelGroup;
            if(a >= 0) { color[a] = 0; }
        }
        std::function<bool(int, std::string&)> dfs = [&](int u, std::string &path) -> bool {
            color[u] = 1;
            for(size_t i = 0; i < dag[u].size(); i++) {
                int v = dag[u][i];
                if(color[v] == 1) { return true; }  // back-edge -> cycle
                if(color[v] == 0 && dfs(v, path)) { return true; }
            }
            color[u] = 2;
            return false;
        };
        for(auto it = dag.begin(); it != dag.end(); ++it) {
            if(color[it->first] == 0) {
                std::string path;
                if(dfs(it->first, path)) {
                    throw std::runtime_error("build_ship: fuel links form a cycle "
                                             "(a two-way link is not allowed)");
                }
            }
        }
    }

}

void build_ship(Vehicle *ship, const ShipDef &def, Shader *partsshader,
                       const glm::dvec3 &base, const glm::dmat3 &orient,
                       double supportMargin)
{
    build_ship_structure(ship, def, partsshader);
    ship->init();  // seed the tanks full (a pad ship lifts off with a full load)
    /* Pad placement: the inflated hull bottom just touches the pad top plus the
       support's margin. Measure it off the part HULL AABBs (getAabb inflates by each
       part's own margin), not the nominal catalog radius/height. */
    double lowest = 1e30;
    for(size_t i = 0; i < ship->parts.size(); i++) {
        const Part *p = ship->parts[i];
        btVector3 lo, hi;
        p->body->shape->getAabb(Vehicle::toBt(p->localPos, p->localRot), lo, hi);
        lowest = std::min(lowest, (double)lo.getZ());
    }
    assert(lowest < 1e30 && "build_ship: a built ship has no parts to measure");
    const glm::dvec3 shift = glm::dvec3(0.0, 0.0, supportMargin - lowest);
    ship->placeShip(base + orient * shift, orient);
    ship->enterWorld();
}

// Resolve the reference frame that owns a world position
static Frame *resolve_frame_by_soi(Frame *root, glm::dvec3 worldPos) {
    Frame *cur = root;
    while(true) {
        Frame *best = NULL;
        double best_d = 1e30;
        for(Frame *c : cur->children) {
            double d = glm::length(worldPos - c->root_pos);
            if(d < c->soi && d < best_d) {
                best = c;
                best_d = d;
            }
        }
        if(best == NULL) { return cur; }
        cur = best;
    }
}

// Starting scenarios (see ScenarioDef in vehicle.h). Column 2 is dev_only:
// only "pad" shows in the VAB dropdown unless --dev-scenarios.
static const ScenarioDef kScenarios[] = {
    {"pad",            false, true,  0.0,  false, -1, 0.0,     0.0, 0.0, 0.0},
    {"rot-orbit",      true,  false, 0.85, false, -1, 0.0,     0.0, 0.0, 0.0},
    /* Same bed about the EQUATOR rather than the rail plane (#182): they differ by
       exactly the axial tilt. */
    {"rot-orbit-eq",   true,  false, 0.85, false, -1, 0.0,     0.0, 0.0, 0.0, false, true},
    {"inertial-orbit", true,  false, kInertialOrbitFrac, false, -1, 0.0, 0.0, 0.0, 0.0},
    {"high-orbit",     true,  false, 5.0,  false, -1, 0.0,     0.0, 0.0, 0.0},
    {"high-polar",     true,  false, 5.0,  true,  -1, 0.0,     0.0, 0.0, 0.0},
    /* In-atmosphere "flying" beds (science.h). atmo_frac is a fraction of the
       atmosphere top, matching the science bands, so a bed lands in its band on
       every body (#99). */
    {"flying-low",     true,  false, 0.1,  false, -1, 0.0,     0.0, 0.0, 0.0, true},
    {"flying-high",    true,  false, 0.5,  false, -1, 0.0,     0.0, 0.0, 0.0, true},
    {"ellipse-peri",   true,  false, 0.0,  false,  0, 10e3, 1000e3, 0.0, 0.0},
    {"ellipse-apo",    true,  false, 0.0,  false,  1, 10e3, 1000e3, 0.0, 0.0},
    {"ellipse-mid",    true,  false, 0.0,  false,  2, 10e3, 1000e3, 0.0, 0.0},
    /* Periapsis under the surface: the departure shape a burn leaves behind
       (high and receding, conic meets the ground). Rails must accept it. */
    {"ellipse-low",    true,  false, 0.0,  false,  2, -100e3, 2000e3, 0.0, 0.0},
    {"escape",         true,  false, 0.85, false, -1, 0.0,     0.0, 2.0, 0.0},
    /* the absolute-radius distance ladder (see ScenarioDef) */
    {"neptune",        true,  false, 0.0,  false, -1, 0.0,     0.0, 0.0, 4.495e12},
    {"oort",           true,  false, 0.0,  false, -1, 0.0,     0.0, 0.0, 1.0e15},
    {"interstellar",   true,  false, 0.0,  false, -1, 0.0,     0.0, 0.0, 1.0e17},
    /* Heliocentric beds at a planet's semi-major axis (abs_r from
       res/systems/solar_system*.json). The transfer planner only lists children of the
       orbited body, so a planet->planet transfer needs a Sun-centred start. */
    {"heli-mercury",   true,  false, 0.0,  false, -1, 0.0,     0.0, 0.0, 5.7914e10},
    {"heli-earth",     true,  false, 0.0,  false, -1, 0.0,     0.0, 0.0, 1.4961e11},
    {"heli-jupiter",   true,  false, 0.0,  false, -1, 0.0,     0.0, 0.0, 7.7815e11},
    {"heli-uranus",    true,  false, 0.0,  false, -1, 0.0,     0.0, 0.0, 2.8698e12},
};

const ScenarioDef *scenario_by_name(const std::string &name) {
    for(size_t i = 0; i < sizeof(kScenarios) / sizeof(kScenarios[0]); i++) {
        if(kScenarios[i].name == name) { return &kScenarios[i]; }
    }
    std::string avail;
    for(size_t i = 0; i < sizeof(kScenarios) / sizeof(kScenarios[0]); i++) {
        if(i) { avail += ", "; }
        avail += kScenarios[i].name;
    }
    throw std::runtime_error("unknown scenario '" + name
                             + "' (available: " + avail + ")");
}

size_t scenario_count() {
    return sizeof(kScenarios) / sizeof(kScenarios[0]);
}

const char *scenario_name_at(size_t i) {
    return kScenarios[i].name;   // caller bounds-checks against scenario_count()
}

bool scenario_dev_only_at(size_t i) {
    return kScenarios[i].dev_only;
}

// Nose (local +Z) along `dir`; roll axis = the coordinate axis most orthogonal to
// dir, so it is never singular for a unit dir.
glm::dmat3 faceAlong(const glm::dvec3 &dir)
{
    const glm::dvec3 z = glm::normalize(dir);
    const glm::dvec3 refs[3] = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
    int best = 0;
    for(int i = 1; i < 3; i++) {
        if(fabs(glm::dot(refs[i], z)) < fabs(glm::dot(refs[best], z))) best = i;
    }
    const glm::dvec3 x = glm::normalize(refs[best] - glm::dot(refs[best], z) * z);
    const glm::dvec3 y = glm::cross(z, x);
    return glm::dmat3(x, y, z);
}

/* slot_offset (m): lateral separation for ships sharing an orbit scenario, applied
   along the orbit binormal so each orbit keeps its shape. Pads ignore it. */
void spawn_vehicle(Vehicle *ship, const ScenarioDef &sc, TerrainBody *home,
                          System &sys, double slot_offset, double t)
{
    if(sc.on_pad) { return; } // already on the pad, set up in main

    /* An abs_r bed pins an ABSOLUTE radius, meaningful only about the body it was
       authored for: elsewhere the ship lands outside that SOI with a speed from the
       wrong mu. */
    if(sc.abs_r > 0.0 && sys.root != home) {
        printf("warning: scenario '%s' pins r = %.4g m, which is not an orbit "
               "about %s (the system root is %s): the ship will not be in %s's "
               "frame and its spawn speed comes from the wrong mu\n",
               sc.name, sc.abs_r, home->name.c_str(),
               sys.root ? sys.root->name.c_str() : "(none)", home->name.c_str());
    }

    const glm::dvec3 center = home->frame->root_pos;
    glm::dvec3 shipWorldPos, velWorld;

    if(sc.ell_phase >= 0) {
        // Ellipse in the equatorial plane (world X-Z), prograde like the circular
        // beds: periapsis along +Z, apsides are root-frame directions.
        /* The bed apsides are authored ABOVE MEAN SEA LEVEL (#404), so they
           build radii from seaLevelRadius(), not the bare radius. */
        const double rp = home->seaLevelRadius() + sc.peri_alt;
        const double ra = home->seaLevelRadius() + sc.apo_alt;
        const double p = 2.0 * rp * ra / (rp + ra); // semi-latus rectum a(1-e^2)
        const double e = (ra - rp) / (ra + rp);
        const double h = sqrt(home->mu * p);        // specific angular momentum
        const glm::dvec3 xhat = glm::dvec3(1, 0, 0);
        const glm::dvec3 zhat = glm::dvec3(0, 0, 1); // periapsis direction
        if(sc.ell_phase == 0) { // at periapsis
            shipWorldPos = center + zhat * rp;
            velWorld = xhat * (h / rp);
        } else if(sc.ell_phase == 1) { // at apoapsis
            shipWorldPos = center - zhat * ra;
            velWorld = -xhat * (h / ra);
        } else { // 90 deg true anomaly (halfway by angle between the apsides)
            shipWorldPos = center + xhat * p;
            velWorld = (xhat * e - zhat) * (h / p);
        }
    } else {
        // Circular bed: radius from the home frame origin. abs_r > 0 pins an absolute
        // radius (the neptune/oort ladder); otherwise alt_frac scales the ALTITUDE
        // ABOVE SEA LEVEL -- against the atmosphere top for flying beds, against the
        // near-body shell for the orbit beds.
        const double sea_r = home->seaLevelRadius();
        const double top = home->surface.atmosphere.top();
        const double shell = home->rot_frame->soi - sea_r;
        const double alt = (sc.atmo_frac && top > 0.0)
                         ? sc.alt_frac * top
                         : sc.alt_frac * shell;
        const double r = sc.abs_r > 0.0 ? sc.abs_r : sea_r + alt;
        /* Which plane the bed's orbit sits in, in the home frame's axes. A polar bed
           sits over the GEOGRAPHIC pole (#174) -- the frame's +Y is the RAIL plane's
           normal, off the spin axis by the axial tilt. An -eq bed (#182) hangs off
           equator_orient, whose +Y is the pole. */
        const glm::dvec3 pole = home->frame->spinAxisRelTo(home->frame);
        const glm::dvec3 rhat_local = sc.polar ? pole
                                   : sc.eq_plane ? home->rot_frame->equator_orient
                                                                 * glm::dvec3(1, 0, 0)
                                   : glm::dvec3(0, 0, 1);
        shipWorldPos = center + home->frame->root_orient * (rhat_local * r);

        // Circular speed (vis-viva with semi-major axis == r); esc_frac makes
        // it a hyperbola instead.
        const double speed = sc.esc_frac > 0.0
                           ? sc.esc_frac * sqrt(2.0 * home->mu / r)
                           : sqrt(home->mu / r);

        // Prograde, about the plane's OWN normal. Normalize: with an inclined
        // body rhat is not orthogonal to the reference axis, so the raw cross
        // product is short by cos(incl).
        const glm::dvec3 rhat = glm::normalize(shipWorldPos - center);
        const glm::dvec3 vhat = glm::normalize(
            sc.polar ? glm::cross(glm::dvec3(1, 0, 0), rhat)
          : sc.eq_plane ? glm::cross(home->frame->root_orient * pole, rhat)
          : glm::cross(glm::dvec3(0, 1, 0), rhat));
        velWorld = speed * vhat;
    }

    if(slot_offset != 0.0) {
        const glm::dvec3 rhat = glm::normalize(shipWorldPos - center);
        const glm::dvec3 vhat = glm::normalize(velWorld);
        shipWorldPos += glm::normalize(glm::cross(rhat, vhat)) * slot_offset;
    }

    Frame *frame = resolve_frame_by_soi(sys.root->frame, shipWorldPos);

    // velWorld is RELATIVE to home in root-frame axes; the root-frame velocity is
    // that plus home's own root velocity. Then express both in the resolved frame.
    const glm::dvec3 target = glm::transpose(frame->root_orient) * (shipWorldPos - frame->root_pos);
    const glm::dvec3 vel = glm::transpose(frame->root_orient)
                          * (velWorld + home->frame->root_vel - frame->root_vel)
                          - frame->GetStasisVelocity(target);

    if(frame != ship->frame) {
        ship->moveToFrame(frame, t);
    }

    // Nose (local +Z) along prograde. `target` is in the resolved frame's axes, so
    // `orient` must be too: faceAlong returns a root-frame attitude.
    const glm::dmat3 orient =
        glm::transpose(frame->root_orient) * faceAlong(velWorld);
    ship->placeShipAtCom(target, orient);
    ship->setVelocity(vel);

    printf("Spawn '%s' around %s: frame '%s' @ world (%.0f, %.0f, %.0f), r = %.0f m, |v| = %.1f m/s\n",
           sc.name, home->name.c_str(), frame->name.c_str(),
           shipWorldPos.x, shipWorldPos.y, shipWorldPos.z,
           glm::length(shipWorldPos - center), glm::length(velWorld));
}

/* Ships sharing a (body, scenario) orbit bed are pushed this far apart along the
   orbit binormal so they don't spawn on top of each other. */
static const double ORBIT_SLOT_SPACING = 20.0;

std::string dedupVehicleName(const std::vector<Vehicle *> &vehicles,
                             const std::string &base,
                             const CrewRoster *roster)
{
    /* No snapshot: this is the live container, and this call cannot change
       its membership. The retry loop below only reads names. */
    std::string nm = base;
    int n = 2;
    while(true) {
        bool taken = roster != nullptr && roster->byName(nm) != nullptr;
        for(Vehicle *s : vehicles) {
            if(s->displayName() == nm) { taken = true; break; }
        }
        if(!taken) { return nm; }
        nm = base + " #" + std::to_string(n);
        n++;
    }
}

/* The def-path form: only build_start_vehicles uses it, so it stays here rather
   than in the header next to place_vehicle_def. */
static Vehicle *place_vehicle(Game &g, const std::string &shipDefPath,
                              const std::string &wantName, TerrainBody *hb,
                              const ScenarioDef *sc, const StaticBuilding *site)
{
    return place_vehicle_def(g,
                             load_ship_def(resdir::path(shipDefPath).c_str(), g.parts),
                             shipDefPath, wantName, hb, sc, site);
}

Vehicle *place_vehicle_def(Game &g, const ShipDef &def, const std::string &defPath,
                           const std::string &wantName, TerrainBody *hb,
                           const ScenarioDef *sc, const StaticBuilding *site)
{
    std::vector<Vehicle *> &vehicles = g.vehicles;
    /* slot feeds spawn_vehicle's orbit spread; pads do not slot. Every vehicle
       in the list is flyable -- a seated kerbal is not in it (part.h). */
    int slot = 0;
    if(!sc->on_pad) {
        for(auto *s : vehicles) {
            if(s->home == hb && s->scenario == sc) { slot++; }
        }
    }

    std::string nm = wantName.empty() ? def.name : wantName;
    if(nm.empty()) { nm = "Ship"; }
    // The roster's names are reserved too: a kerbal's name is never deduped.
    nm = dedupVehicleName(vehicles, nm, &g.roster);

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
            throw std::runtime_error(std::string("place_vehicle: no launch site on ")
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
    /* A Kerbal's display name is the roster profile's (eva.h), so its requested
       label is not stored; `nm` survives only in the empty-pool refusal below.
       Ships keep their deduped name here. */
    if(!is_kerbal) { v->name = nm; }
    v->defPath = defPath;
    v->home = hb;
    v->scenario = sc;
    v->slot = slot;
    v->m_parent = hb;
    v->sun = g.sun;
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
        build_ship(v, def, g.partsshader, base, pad_orient, supportMargin);
    } catch(...) {
        delete v;
        throw;
    }
    /* A walker takes its suit from the home pad pool (part.h: a kerbal is a
       Part at one place). The minted def suit is replaced by the pooled one.
       A kerbal IS a roster entry (crew.h: the roster is the cast), so a spent
       pool means there is no person left to place -- refuse the spawn rather
       than mint somebody off the def. Nothing in the game creates a person:
       mintCrewPool at boot and a load's crew_pool are the only sources, and
       both name the roster they came from (#394). */
    if(is_kerbal && !v->parts.empty()) {
        Part *suit = takeCrew(g.sys);
        if(suit == nullptr) {
            delete v;
            throw std::runtime_error("place_vehicle: no kerbal on the home pad for '"
                                     + nm + "': the cast is already in the world");
        }
        Part *minted = v->parts[0];
        suit->localPos = minted->localPos;
        suit->localRot = minted->localRot;
        suit->stage = minted->stage;
        suit->owner = v;
        v->parts[0] = suit;
        /* build_ship left `controller` and the compound's part list on the
           minted suit. Re-point before the delete, then finalize: the pooled
           suit may weigh differently (partly-burned tank, pocket items), and the
           hull/compoundParts must follow parts[0] or both dangle at freed memory
           (Kerbal::applyEva reads controller). */
        if(v->controller == minted) { v->controller = suit; }
        delete minted;
        inventorySetOwner(suit, v);
        v->finalize();
        assert(suit->kerbal != nullptr && suit->kerbal->id != 0);
        assert(v->name.empty());   // the profile owns the walker's name
    }
    v->setVelocity(glm::dvec3(0, 0, 0));
    // Re-home: starts the flight journal. Then the list adopts it.
    v->setSoi(hb->rot_frame, g.time);
    vehicles.push_back(v);
    return v;
}

void add_vehicle(Game &g, Vehicle *v, TerrainBody *home, const ScenarioDef *sc)
{
    v->home = home;
    v->scenario = sc;
    v->slot = 0;   // the builder placed it; there is no slot to space out
    v->setSoi(v->frame, g.time);   // journal start (the builder set frame)
    g.vehicles.push_back(v);
}

void apply_scenarios(Game &g) {
    /* No snapshot: spawn_vehicle re-homes a ship (setSoi), and re-homing no
       longer moves it between containers, so the list cannot change under the
       loop. */
    for(auto *s : g.vehicles) {
        if(s->scenario == nullptr) { continue; }
        spawn_vehicle(s, *s->scenario, s->home, g.sys,
                      ORBIT_SLOT_SPACING * (double)s->slot, g.time);
    }
}

Vehicle *build_start_vehicles(Game &g, const std::vector<DebugStartShip> &entries)
{
    Vehicle *first = nullptr;
    for(size_t i = 0; i < entries.size(); i++) {
        const DebugStartShip &fe = entries[i];
        TerrainBody *hb = g.sys.find(fe.body);
        if(hb == nullptr) {
            std::string avail;
            for(size_t k = 0; k < g.sys.bodies.size(); k++) {
                if(k) { avail += ", "; }
                avail += g.sys.bodies[k]->name;
            }
            throw std::runtime_error("start ships: ship entry " + std::to_string(i)
                                     + ": unknown body '" + fe.body
                                     + "' (available: " + avail + ")");
        }
        const ScenarioDef *sc = scenario_by_name(fe.scenario);
        /* A named pad only means anything on a pad bed: place_vehicle_def's
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
        Vehicle *v = place_vehicle(g, fe.ship, fe.name, hb, sc, site);
        spawn_crew(v, g.sys);
        if(first == nullptr) { first = v; }
    }
    return first;
}

StaticBuilding *homeCrewPad(const System &sys) {
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

Part *takeCrew(System &sys) {
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
       the suit lands on (fill_seat, place_vehicle_def). */
    inventorySetOwner(suit, nullptr);
    return suit;
}

Part *takeCrewById(System &sys, uint64_t profileId) {
    StaticBuilding *home = homeCrewPad(sys);
    if(home == nullptr) { return nullptr; }
    for(size_t i = 0; i < home->crew.size(); i++) {
        Part *suit = home->crew[i];
        assert(suit->kerbal != nullptr);   // the pool is minted from the roster
        if(suit->kerbal->id != profileId) { continue; }
        home->crew.erase(home->crew.begin() + (std::ptrdiff_t)i);
        suit->container = nullptr;
        inventorySetOwner(suit, nullptr);   // as takeCrew: no dangling owner
        return suit;
    }
    return nullptr;
}

void returnCrew(System &sys, Part *suit) {
    if(suit == nullptr) { return; }
    assert(suit->kerbal != nullptr);   // a person IS a roster entry (crew.h)
    StaticBuilding *home = homeCrewPad(sys);
    if(home == nullptr) {
        /* Nowhere to stand, so the person is gone -- and LOUDLY so: a system
           with no VAB_LAUNCH_PAD eats its crew at every recover, and a silent
           delete reads exactly like a successful flight home. */
        printf("[crew] '%s' is gone: this system has no VAB_LAUNCH_PAD to "
               "stand them on\n",
               suit->kerbal->name.c_str());
        fflush(stdout);
        delete suit;
        return;
    }
    suit->container = nullptr;
    inventorySetOwner(suit, nullptr);
    home->crew.push_back(suit);
}

/* Fill one seat: park a suit Part in the capsule (ship, part) as a seat occupant
   (part.h). There is no Kerbal vehicle here -- minting one is what going EVA
   means (Game::kerbalEVA). */
bool fill_seat(Vehicle *ship, size_t part, System &sys, Part *suit, int slot) {
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
                   ship->displayName().c_str(), part, capDef->crew_capacity);
            return false;
        }
    }

    /* The seat edge: the capsule OWNS the suit (so ~Part frees it). `owner` is
       the vehicle the container sits on, as for an item. Identity rides the
       suit (Part::kerbal) -- the person owns it. A named assignment has already
       been checked against the build, so a refused slot here is a bug. */
    assert(capPart->addSeat(suit, slot < 0 ? (size_t)-1 : (size_t)slot));
    assert(suit->kerbal != nullptr);   // a person IS a roster entry (crew.h)
    suit->owner = ship;
    /* Whatever the suit carries in its pockets rides the ship too (a recovered
       kerbal comes home with its loot). */
    inventorySetOwner(suit, ship);
    // Rebuild so the compound carries the crew mass.
    ship->rebuildCompound();
    printf("Crew: '%s' aboard '%s' part %zu (%d/%d)\n",
           suit->kerbal->name.c_str(),
           ship->displayName().c_str(), part,
           capPart->seatCount(), capDef->crew_capacity);
    return true;
}

void spawn_crew(Vehicle *ship, System &sys) {
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

/* --spin-log: the ship's rotational state, plus the tidal torque that is left to
   spin it.
   `mass` is the analytic part sum -- the same basis --mass-log calls `mass` --
   rather than hull->mass: a ship's mass is a property of its parts, and the
   compound is a physics-world quantity that --mass-log already reports
   separately as hullmass. The KINEMATICS are the hull's, so they print only
   while the hull is in the world. goOnRails() does RemoveBody(hull) and the
   hull keeps its last in-world velocity, so printing them for a railed ship
   reads as a live state for a body nothing is accelerating -- the trap
   --mass-log deliberately refuses. railed= makes the omission a stated fact
   instead of a gap a reader guesses at, and one printf keeps the optional
   fragment from tearing. The COM and the tidal torque are frame-analytic
   (partWorldPose goes through frameS, not the hull), so they stay honest on
   the rails. */
void spin_log(Vehicle *ship, double time) {
    if(ship->hull == nullptr) { return; }

    const glm::dvec3 com = ship->get_center_of_mass();
    char kin[128] = "";
    if(ship->hullInWorld()) {
        const glm::dvec3 v = GetVelocity(ship->hull);
        const glm::dvec3 w = GetAngVelocity(ship->hull);
        snprintf(kin, sizeof kin,
                 " |v|=%.3f m/s w=[%.3e %.3e %.3e] |w|=%.3e rad/s",
                 glm::length(v), w.x, w.y, w.z, glm::length(w));
    }
    printf("[spin] t=%.2fs ship=%s parts=%zu mass=%.1f kg"
           " com=[%.1f %.1f %.1f] railed=%d%s\n",
           time, ship->displayName().c_str(), ship->parts.size(), ship->getMass(),
           com.x, com.y, com.z, ship->onRails ? 1 : 0, kin);

    const double G = 6.674e-11;
    const double M = ship->m_parent->mass;
    glm::dvec3 tau(0, 0, 0);
    for(size_t i = 0; i < ship->parts.size(); i++) {
        const glm::dvec3 p = ship->partPos(ship->parts[i]);
        const double r = glm::length(p);
        // effectiveMass: same mass basis as applyGravity
        const glm::dvec3 F = -G * M * ship->parts[i]->effectiveMass() * p / (r * r * r);
        tau += glm::cross(p - com, F);
    }
    printf("[spin]   tidal gravity torque |tau|=%.3e\n", glm::length(tau));
    fflush(stdout);
}

/* --mass-log: the two mass bases for every vehicle, not just the active one.
   `mass` is the analytic part sum (getMass, what the HUD and the fuel book
   claim); `hullmass` is the compound Bullet accelerates, which only re-makes
   once the two drift past kMassRebuildFrac apart -- so they differ by design,
   and a case pins whichever pair it is actually claiming about. One printf, not
   several: a torn line would let the optional hullmass read as absent (dv_log
   prints whole lines for the same reason). A hull out of the world prints no
   hullmass -- that number would be the last one it had in physics, not anything
   being accelerated now -- and railed= says so rather than leaving the gap for
   a parser to guess at. */
void mass_log(Vehicle *ship, double time) {
    char hull[48] = "";
    if(ship->hullInWorld()) {
        snprintf(hull, sizeof(hull), " hullmass=%.2f kg", ship->hull->mass);
    }
    printf("[mass] t=%.3fs ship=\"%s\" parts=%zu mass=%.2f kg%s railed=%d\n",
           time, ship->displayName().c_str(), ship->parts.size(), ship->getMass(),
           hull, ship->onRails ? 1 : 0);
    fflush(stdout);
}

/* --- Vehicle method definitions --- */

bool Vehicle::hullInWorld() const {
    return hull != nullptr && hull->btBody != nullptr
        && hull->btBody->getBroadphaseHandle() != nullptr;
}

btCompoundShape * Vehicle::compoundShape() const {
    return (hull != nullptr && hull->shape != nullptr)
         ? static_cast<btCompoundShape *>(hull->shape) : nullptr;
}

btTransform Vehicle::toBt(const glm::dvec3 &pos, const glm::dmat3 &rot) {
    btTransform t;
    t.setIdentity();
    t.setOrigin(btVector3(pos.x, pos.y, pos.z));
    const glm::dquat q = glm::quat_cast(rot);
    t.setRotation(btQuaternion(q.x, q.y, q.z, q.w));
    return t;
}

void Vehicle::fromBt(const btTransform &t, glm::dvec3 &pos, glm::dmat3 &rot) {
    const btVector3 &o = t.getOrigin();
    pos = glm::dvec3(o.getX(), o.getY(), o.getZ());
    btQuaternion q;
    t.getBasis().getRotation(q);
    rot = glm::mat3_cast(glm::dquat(q.w(), q.x(), q.y(), q.z()));
}

void Vehicle::frameS(glm::dvec3 &pos, glm::dmat3 &rot) const {
    glm::dvec3 bodyPos; glm::dmat3 bodyRot;
    fromBt(hull->btBody->getCenterOfMassTransform(), bodyPos, bodyRot);
    glm::dvec3 pOrigin; glm::dmat3 pBasis;
    fromBt(principal, pOrigin, pBasis);
    rot = bodyRot * glm::transpose(pBasis);
    pos = bodyPos - rot * pOrigin;
}

glm::dvec3 Vehicle::comPos() const {
    const btVector3 &o = hull->btBody->getCenterOfMassPosition();
    return glm::dvec3(o.getX(), o.getY(), o.getZ());
}

void Vehicle::placeShip(const glm::dvec3 &sPos, const glm::dmat3 &sRot) {
    glm::dvec3 pOrigin; glm::dmat3 pBasis;
    fromBt(principal, pOrigin, pBasis);
    setPosRot(hull, sPos + sRot * pOrigin, sRot * pBasis);
}

void Vehicle::placeShipAtCom(const glm::dvec3 &com, const glm::dmat3 &sRot) {
    glm::dvec3 pOrigin; glm::dmat3 pBasis;
    fromBt(principal, pOrigin, pBasis);
    placeShip(com - sRot * pOrigin, sRot);
}

void Vehicle::rebuildCompound() {
    /* `principal` moves when the masses do, and the body's transform IS the COM
       transform: leaving it alone across a rebuild shifts the ship and leaves it
       spinning about the wrong point. */
    const bool have = (hull != nullptr && hull->btBody != nullptr);
    // Out of the world before the delete, back in after: the rebuild
    // replaces the rigid body, and a registered one cannot just be freed.
    const bool wasInWorld = hullInWorld();
    if(wasInWorld) { RemoveBody(hull); }
    glm::dvec3 sPos(0.0), vCom(0.0), omega(0.0), oldOrigin(0.0);
    glm::dmat3 sRot(1.0);
    if(have) {
        frameS(sPos, sRot);
        vCom = GetVelocity(hull);
        omega = GetAngVelocity(hull);
        glm::dmat3 ignore;
        fromBt(principal, oldOrigin, ignore);
    }

    delete hull; hull = nullptr;
    compoundParts.clear();
    principal = btTransform::getIdentity();
    if(parts.empty()) { aeroHull.clear(); return; }

    // ownership back-pointers (part.h)
    for(Part *p : parts) { p->owner = this; }

    btCompoundShape *inS = new btCompoundShape(true, (int)parts.size());
    std::vector<btScalar> masses(parts.size());
    btScalar total = 0;
    for(size_t i = 0; i < parts.size(); i++) {
        Part *p = parts[i];
        inS->addChildShape(toBt(p->localPos, p->localRot), p->body->shape);
        // effectiveMass: body mass + contained crew/inventory (part.h)
        masses[i] = (btScalar)p->effectiveMass();
        /* Bullet's calculatePrincipalAxisTransform btAsserts every child mass
           > 0, but btAssert compiles out unless BT_DEBUG is defined (btScalar.h)
           and we never define it, so that guard is not live in ANY config we
           build (#389). The invariant is ours: a zero-mass child would keep its
           collision hull and its drag silhouette while weighing nothing, and
           --mass-log would still report mass and hullmass agreeing.
           make_part asserts the def's mass and the save loader refuses negative
           contents, so a zero here means a Part was minted some other way. */
        assert(masses[i] > 0 && "a part in the compound weighs nothing");
        total += masses[i];
        compoundParts.push_back(p);
    }

    btVector3 inertiaDiag(0, 0, 0);
    inS->calculatePrincipalAxisTransform(&masses[0], principal, inertiaDiag);

    /* Re-base the children into the COM/principal frame: left in S, the body's origin
       sits at the root part while its inertia is diagonal about the principal axes --
       inconsistent under off-axis torque. */
    btCompoundShape *nc = new btCompoundShape(true, inS->getNumChildShapes());
    const btTransform toBody = principal.inverse();
    for(int i = 0; i < inS->getNumChildShapes(); i++) {
        nc->addChildShape(toBody * inS->getChildTransform(i),
                          inS->getChildShape(i));
    }
    delete inS;

    hull = new Body;            // mesh/shader/texture stay null: drawn part by part
    hull->shape = nc;
    hull->mass  = (double)total;
    btRigidBody::btRigidBodyConstructionInfo ci(total, nullptr, nc,
                                                inertiaDiag);
    ci.m_friction = (btScalar)hullFriction;
    hull->btBody = new btRigidBody(ci);
    NeverSleep(hull);

    /* Restore frame S under the new principal, and the COM's velocity:
       the COM moved WITHIN S, so its world velocity changed by
       omega x the shift. */
    glm::dvec3 pOrigin; glm::dmat3 pBasis;
    fromBt(principal, pOrigin, pBasis);
    hull->btBody->setWorldTransform(toBt(sPos + sRot * pOrigin, sRot * pBasis));
    if(have) {
        SetVelocity(hull, vCom + glm::cross(omega, sRot * (pOrigin - oldOrigin)));
        SetAngVelocity(hull, omega);
    }
    if(wasInWorld) { AddPhysicsBody(hull); }

    rebuildAeroHull();

    checkCompoundInvariants();
    assert(checkPartInvariants() && "containment invariant broken (see [part] above)");
}

/* The silhouette inputs: the part set, each part's collision body, and its
   mount. Anything that changes what the ship presents to the air changes
   this fingerprint; fuel burn (mass, and the COM shift that makes
   refreshCompound rebuild) does not. splitmix64 mixing; doubles enter as
   exact bits, so a collision means the geometry was bit-identical anyway
   (modulo 2^64 bad luck). */
static uint64_t aeroGeomFingerprint(const std::vector<Part *> &parts) {
    const auto mix = [](uint64_t h, uint64_t v) {
        uint64_t x = h ^ (v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2));
        x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ull;
        x = (x ^ (x >> 27)) * 0x94d049bb133111ebull;
        return x ^ (x >> 31);
    };
    uint64_t h = mix(0, (uint64_t)parts.size());
    for(const Part *p : parts) {
        h = mix(h, (uint64_t)(uintptr_t)p->body);
        for(int i = 0; i < 3; i++) { h = mix(h, std::bit_cast<uint64_t>(p->localPos[i])); }
        for(int c = 0; c < 3; c++) {
            for(int r = 0; r < 3; r++) { h = mix(h, std::bit_cast<uint64_t>(p->localRot[c][r])); }
        }
    }
    return h;
}

void Vehicle::rebuildAeroHull() {
    // Gate: a compound rebuild driven by mass drift (the common burn case)
    // must not pay for the convex hull again; only a GEOMETRY change does.
    const uint64_t key = aeroGeomFingerprint(parts);
    if(aeroGeomValid && key == aeroGeomKey) { return; }
    aeroGeomKey = key;
    aeroGeomValid = true;  // every exit below leaves the hull right for this key
    aeroHull.clear();
    size_t cap = 0;
    for(Part *p : parts) {
        if(p->body != nullptr) { cap += p->body->hullVerts.size(); }
    }
    if(cap < 3) { return; }
    // union of the parts' hull verts, in frame S
    std::vector<glm::dvec3> u;
    u.reserve(cap);
    for(Part *p : parts) {
        if(p->body == nullptr) { continue; }
        for(const glm::dvec3 &v : p->body->hullVerts) {
            u.push_back(p->localRot * v + p->localPos);
        }
    }
    /* Reduce to the extreme points: same silhouette, fewer points for the
       per-substep projectedArea sort. */
    btConvexHullShape shape(reinterpret_cast<const btScalar *>(&u[0].x),
                            (int)u.size(), 3 * sizeof(double));
    shape.optimizeConvexHull();
    const int n = shape.getNumVertices();
    if(n < 3) { aeroHull.swap(u); return; }  // degenerate reduce: keep the union
    aeroHull.reserve(n);
    for(int i = 0; i < n; i++) {
        btVector3 v;
        shape.getVertex(i, v);
        aeroHull.push_back(glm::dvec3(v.getX(), v.getY(), v.getZ()));
    }
}

/* The containment tree under one attached part, depth-first. effectiveMass and
   ~Part recurse without limit, so the checks must too: a violation two levels
   down -- a kerbal-typed part parked in a suit's pocket, which is what a
   hand-edited save produces -- is as fatal as one at the top. `depth` exists
   only to turn a wired cycle into a refusal instead of a stack overflow; real
   trees nest a crate inside a pod, not sixteen deep. */
bool Vehicle::checkContainmentTree(Part *p, int depth) const {
    if(depth > 16) {
        printf("[part] '%s': part uid %llu is contained %d deep -- deeper than "
               "the walk allows (a cycle, or a hand-edited tree)\n", displayName().c_str(),
               (unsigned long long)p->uid, depth);
        return false;
    }
    /* A capsule cannot hold more occupants than it has seats. Both the live
       board and the loader refuse it, so a count over the ceiling means
       something wired a seat behind their backs -- and effectiveMass would
       happily carry the extra person. */
    if(p->isCapsule() && p->seatCount() > p->def->crew_capacity) {
        printf("[part] '%s': part uid %llu holds %d seat occupant(s) but has "
               "only %d seat(s)\n", displayName().c_str(), (unsigned long long)p->uid,
               p->seatCount(), p->def->crew_capacity);
        return false;
    }
    /* Seats: a person in a capsule, owned through `seats`. Not cargo. */
    for(Part *suit : p->seats) {
        if(suit == nullptr) { continue; }
        if(suit->container != p) {
            printf("[part] '%s': seats holds part uid %llu, but "
                   "its container points elsewhere\n", displayName().c_str(),
                   (unsigned long long)suit->uid);
            return false;
        }
        if(suit->def == nullptr || suit->def->type != "kerbal") {
            printf("[part] '%s': part uid %llu sits in a seat, but is "
                   "not a kerbal\n", displayName().c_str(),
                   (unsigned long long)suit->uid);
            return false;
        }
        if(!p->isCapsule()) {
            printf("[part] '%s': part uid %llu is a seat occupant, but "
                   "this part is not a capsule\n", displayName().c_str(),
                   (unsigned long long)suit->uid);
            return false;
        }
        // A suit's pockets are part of the tree like any other container's.
        if(!checkContainmentTree(suit, depth + 1)) { return false; }
    }
    /* Inventory: owned cargo. `contents` IS the ownership list. */
    for(Part *c : p->contents) {
        if(c->container != p) {
            printf("[part] '%s': contents lists part uid %llu, but its "
                   "container points elsewhere\n", displayName().c_str(),
                   (unsigned long long)c->uid);
            return false;
        }
        if(c->isContainedPerson()) {
            printf("[part] '%s': part uid %llu is a kerbal in inventory; "
                   "a person is not cargo\n", displayName().c_str(),
                   (unsigned long long)c->uid);
            return false;
        }
        if(p->def == nullptr || p->def->inventory_capacity <= 0) {
            printf("[part] '%s': part uid %llu is an inventory "
                   "item, but this part is not a container\n",
                   displayName().c_str(), (unsigned long long)c->uid);
            return false;
        }
        if(!checkContainmentTree(c, depth + 1)) { return false; }
    }
    return true;
}

bool Vehicle::checkPartInvariants() const {
    /* Every failure path flushes. The callers assert on the result and stdout is
       BLOCK-buffered when redirected, so the [part] line that explains the abort
       otherwise dies in the buffer along with the process. */
    for(Part *p : parts) {
        if(p->owner != this) {
            printf("[part] '%s': part '%s' (uid %llu) is attached to the wrong "
                   "vehicle\n", displayName().c_str(), p->id.c_str(),
                   (unsigned long long)p->uid);
            fflush(stdout);
            return false;
        }
        if(p->container != nullptr) {
            /* Nothing in a parts list is held by another part. A seat occupant
               lives in the CAPSULE's `seats` and is not part of any vehicle; an
               inventory item is never in a parts list either. A suit Part that
               is both in a parts list and held would be deleted twice. */
            printf("[part] '%s': part '%s' (uid %llu) is attached to this "
                   "vehicle AND contained by part uid %llu\n", displayName().c_str(),
                   p->id.c_str(), (unsigned long long)p->uid,
                   (unsigned long long)p->container->uid);
            fflush(stdout);
            return false;
        }
        if(!checkContainmentTree(p, 0)) { fflush(stdout); return false; }
    }
    return true;
}

glm::dvec3 Vehicle::compoundCom() const {
    double total = 0.0;
    glm::dvec3 com(0.0);
    for(size_t i = 0; i < parts.size(); i++) {
        // effectiveMass, as in rebuildCompound
        const double m = parts[i]->effectiveMass();
        total += m;
        com += m * parts[i]->localPos;
    }
    return (total > 0.0) ? com / total : com;
}

glm::dvec3 Vehicle::comOffset() const {
    if(hull == nullptr || parts.empty()) { return glm::dvec3(0.0); }
    glm::dvec3 sPos; glm::dmat3 sRot;
    frameS(sPos, sRot);
    return sPos + sRot * compoundCom() - comPos();
}

void Vehicle::refreshCompound() {
    if(hull == nullptr) { rebuildCompound(); return; }
    glm::dvec3 pOrigin; glm::dmat3 pBasis;
    fromBt(principal, pOrigin, pBasis);
    const double total = getMass();
    if(glm::length(compoundCom() - pOrigin) > kComRebuildTol
       || std::fabs(total - hull->mass) > kMassRebuildFrac * hull->mass) {
        rebuildCompound();
    }
}

void Vehicle::checkCompoundInvariants() const {
    btCompoundShape *cs = compoundShape();
    if(cs == nullptr) { return; }
    double total = 0.0, extent = 0.0;
    glm::dvec3 com(0.0);
    for(size_t i = 0; i < parts.size(); i++) {
        // effectiveMass, as in rebuildCompound
        const double m = parts[i]->effectiveMass();
        total += m;
        com += m * parts[i]->localPos;
        extent = std::max(extent, glm::length(parts[i]->localPos));
    }
    if(total <= 0.0) { return; }
    com /= total;

    /* The analytic tensor about the authored COM, in S axes. The shape's inertia is
       scaled from body mass up to effectiveMass (linear for a fixed shape), as is the
       parallel-axis term. */
    glm::dmat3 want(0.0);
    for(size_t i = 0; i < parts.size(); i++) {
        Part *p = parts[i];
        const double m = p->effectiveMass();
        const double bm = p->body->mass;
        const double scale = (bm > 0.0) ? m / bm : 0.0;
        const glm::dvec3 il = getInertiaDiag(p->body);
        const glm::dmat3 d(il.x * scale, 0.0, 0.0,
                           0.0, il.y * scale, 0.0,
                           0.0, 0.0, il.z * scale);
        want += p->localRot * d * glm::transpose(p->localRot);
        const glm::dvec3 o = p->localPos - com;
        want += m
              * (glm::dot(o, o) * glm::dmat3(1.0) - glm::outerProduct(o, o));
    }

    /* Bullet's, rotated out of the principal frame back into S */
    glm::dvec3 pOrigin; glm::dmat3 pBasis;
    fromBt(principal, pOrigin, pBasis);
    const btVector3 &bi = hull->btBody->getLocalInertia();
    const glm::dmat3 diag(bi.getX(), 0.0, 0.0,
                          0.0, bi.getY(), 0.0,
                          0.0, 0.0, bi.getZ());
    const glm::dmat3 got = pBasis * diag * glm::transpose(pBasis);

    /* iTol cannot be tighter than btMatrix3x3::diagonalize's Jacobi stop:
       every off-diagonal under 1e-5 x the diagonal trace. */
    double trace = 0.0;
    for(int c = 0; c < 3; c++) { trace += std::fabs(want[c][c]); }
    const double iTol = 1e-5 * std::max(1.0, trace);
    double iErr = 0.0;
    for(int c = 0; c < 3; c++) {
        for(int r = 0; r < 3; r++) {
            iErr = std::max(iErr, std::fabs(got[c][r] - want[c][r]));
        }
    }
    const double comErr = glm::length(pOrigin - com);
    const double comTol = 1e-9 * std::max(1.0, glm::length(com));

    double childPosErr = 0.0, childRotErr = 0.0;
    if((size_t)cs->getNumChildShapes() != compoundParts.size()) {
        childPosErr = 1e30;   // the mapping is broken; report it as such
    } else {
        for(size_t i = 0; i < compoundParts.size(); i++) {
            const Part *p = compoundParts[i];
            glm::dvec3 cp; glm::dmat3 cr;
            fromBt(principal * cs->getChildTransform((int)i), cp, cr);
            childPosErr = std::max(childPosErr, glm::length(cp - p->localPos));
            for(int c = 0; c < 3; c++) {
                for(int r = 0; r < 3; r++) {
                    childRotErr = std::max(childRotErr,
                        std::fabs(cr[c][r] - p->localRot[c][r]));
                }
            }
        }
    }
    const double childTol = 1e-9 * std::max(1.0, extent);

    if(iErr > iTol || comErr > comTol
       || childPosErr > childTol || childRotErr > 1e-12) {
        printf("[compound] '%s': does NOT reproduce the part assembly "
               "(com err %.4g m, tol %.4g; inertia err %.4g kg m^2, tol "
               "%.4g; child pos err %.4g m, tol %.4g; child rot err "
               "%.4g)\n",
               displayName().c_str(), comErr, comTol, iErr, iTol,
               childPosErr, childTol, childRotErr);
        fflush(stdout);
        assert(false && "compound must reproduce the part assembly");
    }
}

Part * Vehicle::rootPart() const {
    for(size_t i = 0; i < parts.size(); i++) {
        if(parts[i]->parent == nullptr) { return parts[i]; }
    }
    return parts.empty() ? nullptr : parts[0];
}

bool Vehicle::hasChildBelow(const Part *p) const {
    /* childBelow (shipdef.h): the shared flight/VAB test -- the child is on p's
       exhaust face (in p's own frame) and axial. */
    for(Part *c : parts) {
        if(c->parent != p) { continue; }
        if(childBelow(p->def->radius, p->localPos, p->localRot,
                      c->localPos)) { return true; }
    }
    return false;
}

void Vehicle::partWorldPose(const Part *p, glm::dvec3 &pos, glm::dmat3 &rot) const {
    glm::dvec3 sPos; glm::dmat3 sRot;
    frameS(sPos, sRot);
    pos = sPos + sRot * p->localPos;
    rot = sRot * p->localRot;
}

void Vehicle::partPoseRelCom(const Part *p, glm::dvec3 &pos, glm::dmat3 &rot) const {
    // principal's origin IS the COM in S, so the COM-relative pose is sRot * (localPos - pOrigin).
    glm::dvec3 pOrigin; glm::dmat3 pBasis;
    fromBt(principal, pOrigin, pBasis);
    glm::dvec3 bodyPos; glm::dmat3 bodyRot;
    fromBt(hull->btBody->getCenterOfMassTransform(), bodyPos, bodyRot);
    const glm::dmat3 sRot = bodyRot * glm::transpose(pBasis);
    pos = sRot * (p->localPos - pOrigin);
    rot = sRot * p->localRot;
}

glm::dvec3 Vehicle::partPos(const Part *p) const {
    glm::dvec3 pos; glm::dmat3 rot;
    partWorldPose(p, pos, rot);
    return pos;
}

glm::dmat3 Vehicle::partRot(const Part *p) const {
    glm::dvec3 pos; glm::dmat3 rot;
    partWorldPose(p, pos, rot);
    return rot;
}

glm::dvec3 Vehicle::partAxis(const Part *p, int n) const { return partRot(p)[n]; }

glm::dvec3 Vehicle::partThrustDir(const Part *p) const {
    // Zero gimbal (every part without TVC, and every TVC part at rest) is
    // EXACTLY the authored axis: gimbalDir early-returns partRot[2].
    return gimbalDir(partRot(p), p->gimbal[0], p->gimbal[1]);
}

glm::dvec3 Vehicle::partVel(const Part *p) const {
    return GetVelocity(hull)
         + glm::cross(GetAngVelocity(hull), partPos(p) - comPos());
}

glm::dvec3 Vehicle::partAngVel(const Part *p) const { return GetAngVelocity(hull); }

void Vehicle::compoundCheck(double time) {
    rebuildCompound();
    if(hull == nullptr) { return; }
    const glm::dvec3 com = comPos();
    const btVector3 &I = hull->btBody->getLocalInertia();
    printf("[compound] t=%.2fs ship=%s parts=%zu mass=%.1f kg "
           "com=[%.1f %.1f %.1f] |v|=%.3f m/s |w|=%.5f rad/s "
           "I=[%.4g %.4g %.4g] kg m^2\n",
           time, displayName().c_str(), parts.size(), hull->mass,
           com.x, com.y, com.z,
           glm::length(GetVelocity(hull)),
           glm::length(GetAngVelocity(hull)),
           I.getX(), I.getY(), I.getZ());
    fflush(stdout);
}

void Vehicle::setSlewRequest(SlewMode m) { slewRequest = m; }

void Vehicle::setRoot(Part *part) {
    part->parent   = nullptr;
    part->localPos = glm::dvec3(0.0);
    part->localRot = glm::dmat3(1.0);
    part->owner    = this;   // containment edge (part.h): parts list is the ownership list
    parts.push_back(part);
}

void Vehicle::attach(Part *part, size_t parentIdx, const glm::dvec3 &localPos, const glm::dmat3 &localRot) {
    part->parent   = parts[parentIdx];
    part->localPos = localPos;
    part->localRot = localRot;
    part->owner    = this;   // containment edge (part.h): parts list is the ownership list
    parts.push_back(part);
}

void Vehicle::attachMode(Part *part, size_t parentIdx, AttachMode mode,
                         double angleDeg, double offset) {
    const Part *pp = parts[parentIdx];
    // attachPose is relative, so feeding it the parent's ship-local pose
    // returns the child's ship-local pose directly.
    const AttachPose ap = attachPose(pp->localPos, pp->localRot, *pp->def,
                                     *part->def, mode, angleDeg, offset);
    attach(part, parentIdx, ap.childPos, ap.childRot);
}

void Vehicle::attachDown(Part *part) {
    attachMode(part, parts.size() - 1, AttachMode::Down);
}

void Vehicle::attachSurface(Part *part, size_t parentIdx,
                            const glm::dvec3 &point, const glm::dvec3 &normal,
                            double rollDeg, double offset) {
    const Part *pp = parts[parentIdx];
    const Node *cn = part->def->findSurfaceNode();
    if(cn == nullptr) {
        throw std::runtime_error(std::string("attachSurface: part '") + part->def->name
                                 + "' has no surface node");
    }
    // ::attachSurface is the free solver in shipdef.cpp (qualify so this
    // isn't a recursive call).
    const AttachPose ap = ::attachSurface(pp->localPos, pp->localRot, point,
                                          normal, *cn, rollDeg, offset);
    attach(part, parentIdx, ap.childPos, ap.childRot);
}

void Vehicle::init() {
    if(parts.empty()) { return; }
    /* Seed each tank's resources at construction only -- extractSubtreeAsShip() must
       NOT re-seed a stage that has been burning. */
    for(size_t i = 0; i < parts.size(); i++) {
        parts[i]->fillTanks();
    }
    finalize();
}

void Vehicle::finalize() {
    if(parts.empty()) { return; }
    if(controller == nullptr) { controller = parts[0]; }
    /* totalStages_ = highest stage, minStage_ = lowest; activeStage_ starts at the
       HIGHEST, so stage 1 fires last. */
    totalStages_ = 1;
    int lowest = parts[0]->stage;
    for(size_t i = 1; i < parts.size(); i++) {
        if(parts[i]->stage > totalStages_) { totalStages_ = parts[i]->stage; }
        if(parts[i]->stage < lowest) { lowest = parts[i]->stage; }
    }
    minStage_ = lowest;
    activeStage_ = totalStages_;
    // fuel groups: an engine draws from the tanks it's connected to
    buildFuelGroups();
    // The ship's single rigid body. Registering it is enterWorld()'s job.
    rebuildCompound();
}

void Vehicle::enterWorld() {
    if(hull != nullptr && !hullInWorld()) { AddPhysicsBody(hull); }
}

bool Vehicle::isEva() const { return false; }

void Vehicle::buildFuelGroups() {
    // The groups change, so the cached drain layers are stale -- drop them.
    drainLayers_.clear();
    /* The one partition (shipdef.h, #292): index the parts so the tree can be
       handed over as parent indices, exactly like the staging snapshot and the
       VAB's loop check do. A parent outside this ship would be a torn tree --
       setRoot / attach / absorbShip keep the tree inside `parts`, and
       extractSubtreeAsShip re-roots the part it moves out (vehicle.cpp:2135), so
       neither side is left pointing across. Group numbering follows `parts`
       order in every caller. */
    std::map<Part *, size_t> index;
    for(size_t i = 0; i < parts.size(); i++) { index[parts[i]] = i; }
    std::vector<int> parent(parts.size(), -1);
    std::vector<bool> wall(parts.size(), false);
    for(size_t i = 0; i < parts.size(); i++) {
        const Part *p = parts[i];
        if(p->parent != nullptr) {
            std::map<Part *, size_t>::const_iterator it = index.find(p->parent);
            /* A parent outside this ship is a bug, not a state -- and since no
               build config defines NDEBUG, this trips in the release binary too.
               The old walk keyed by Part* and coped silently, writing fuelGroup
               onto the foreign part. */
            assert(it != index.end());
            parent[i] = (int)it->second;
        }
        wall[i] = p->isFuelBarrier();   /* a wall: stays -1 */
    }
    std::vector<int> group;
    partitionFuelGroups(parent, wall, group);
    for(size_t i = 0; i < parts.size(); i++) { parts[i]->fuelGroup = group[i]; }
}

std::vector<Part *> Vehicle::fuelPool(Part *engine) const {
    std::vector<Part *> pool;
    const int g = engine->fuelGroup;
    for(size_t i = 0; i < parts.size(); i++) {
        Part *p = parts[i];
        if(!p->isTank()) { continue; }
        if(p->fuelGroup != g) { continue; }
        pool.push_back(p);
    }
    return pool;
}

const std::vector<std::vector<int> > &Vehicle::fuelDrainLayers(Part *engine) const {
    const int g = engine->fuelGroup;
    if(g < 0) {
        static const std::vector<std::vector<int> > empty;
        return empty;
    }
    // Cached per group: a repeat call is a map lookup, not a BFS (hot while thrusting).
    auto it = drainLayers_.find(g);
    if(it != drainLayers_.end()) { return it->second; }
    std::vector<std::vector<int> > layers;
    /* reverse adjacency over fuel groups: rev[Y] = { X : X feeds Y }. */
    std::map<int, std::vector<int> > rev;
    for(size_t k = 0; k < fuelLinks.size(); k++) {
        int a = fuelLinks[k].from->fuelGroup;
        int b = fuelLinks[k].to->fuelGroup;
        if(a < 0 || b < 0 || a == b) { continue; }
        rev[b].push_back(a);
    }
    /* BFS from G in the reverse graph; dist[X] = hops from X to G. */
    std::map<int, int> dist;
    dist[g] = 0;
    std::vector<int> queue;
    queue.push_back(g);
    for(size_t qi = 0; qi < queue.size(); qi++) {
        int u = queue[qi];
        std::map<int, std::vector<int> >::const_iterator it = rev.find(u);
        if(it == rev.end()) { continue; }
        for(size_t i = 0; i < it->second.size(); i++) {
            int v = it->second[i];
            if(dist.count(v)) { continue; }
            dist[v] = dist[u] + 1;
            queue.push_back(v);
        }
    }
    // Bucket by distance, then take the buckets furthest-first (map walks ascending).
    std::map<int, std::vector<int> > byDist;
    for(std::map<int, int>::const_iterator it = dist.begin(); it != dist.end(); ++it) {
        byDist[it->second].push_back(it->first);
    }
    for(std::map<int, std::vector<int> >::const_reverse_iterator it = byDist.rbegin();
        it != byDist.rend(); ++it) {
        layers.push_back(it->second);
    }
    auto res = drainLayers_.emplace(g, std::move(layers));
    return res.first->second;
}

bool Vehicle::consumeResourceMass(enum ResourceType type, double amt, Part *engine) {
    const std::vector<std::vector<int> > &layers = fuelDrainLayers(engine);
    if(layers.empty()) { return false; }
    // Drain-or-nothing: no partial burn if the layers cannot cover the request.
    if(availableResourceMass(type, engine) < amt) { return false; }
    /* Drain layer by layer (furthest first), pro-rata across the layer's tanks. */
    double remaining = amt;
    for(size_t li = 0; li < layers.size() && remaining > 0.0; li++) {
        double layerTotal = 0;
        for(size_t gi = 0; gi < layers[li].size(); gi++) {
            const int grp = layers[li][gi];
            for(Part *p : parts) {
                if(!p->isTank() || p->fuelGroup != grp) { continue; }
                if(p->resources.current[(int)type] > 0.0) {
                    layerTotal += p->resources.current[(int)type];
                }
            }
        }
        if(layerTotal <= 0.0) { continue; }
        const double take = remaining < layerTotal ? remaining : layerTotal;
        for(size_t gi = 0; gi < layers[li].size(); gi++) {
            const int grp = layers[li][gi];
            for(Part *p : parts) {
                if(!p->isTank() || p->fuelGroup != grp) { continue; }
                const double have = p->resources.current[(int)type];
                if(have <= 0.0) { continue; }
                double share = take * have / layerTotal;
                if(share > have) { share = have; }
                p->resources.current[(int)type] = have - share;
                /* Fuel leaves resources.current; the body mass is the DRY structure. */
            }
        }
        remaining -= take;
    }
    return true;
}

double Vehicle::availableResourceMass(enum ResourceType type, Part *engine) const {
    double total = 0.0;
    for(const std::vector<int> &layer : fuelDrainLayers(engine)) {
        for(const int grp : layer) {
            for(Part *p : parts) {
                if(!p->isTank() || p->fuelGroup != grp) { continue; }
                total += p->resources.current[(int)type];
            }
        }
    }
    return total;
}

// Fill burn[ResourceType::Num] with the resources the ship's engines draw.
// includeJets: also count jet fuel (jets shed mass but produce no vacuum dv).
void Vehicle::enginePropellantMask(bool *burn, bool includeJets) const {
    for(int r = 0; r < (int)ResourceType::Num; r++) { burn[r] = false; }
    for(Part *p : parts) {
        if(!p->isThruster()) { continue; }
        if(p->isJet() && !includeJets) { continue; }
        for(int r = 0; r < (int)ResourceType::Num; r++) {
            if(p->def->propellant_rate[r] > 0.0) { burn[r] = true; }
        }
    }
}

// Current mass (kg) of the resources flagged in `burn`, summed over all parts.
double Vehicle::fuelMassMasked(const bool *burn) const {
    double fuel = 0.0;
    for(Part *p : parts) {
        for(int r = 0; r < (int)ResourceType::Num; r++) {
            if(burn[r]) { fuel += p->resources.current[r]; }
        }
    }
    return fuel;
}

double Vehicle::getDeltaV() {
    // Rocket propellant only (jets give no vacuum dv). LIMITATION: ship-wide UNION
    // mask and `ve` from the FIRST rocket, so MIXED engine types are an estimate (#41).
    bool burn[(int)ResourceType::Num];
    enginePropellantMask(burn, /*includeJets=*/false);
    const double remaining_fuel = fuelMassMasked(burn); /* kg */
    double ve = 0;   // first ROCKET thruster's exhaust velocity
    for(Part *p : parts) { if(p->isThruster() && !p->isJet()) { ve = p->exhaustVelocity(); break; } }
    // Tsiolkovsky: dv = ve * ln(m_fueled / m_dry).
    return ve * exhaust_scale * log(getMass() / (getMass() - remaining_fuel));
}

double Vehicle::getMass() {
    // effectiveMass: the ship's mass includes what is parked in it (containment).
    double r = 0.0;
    for(Part *p : parts) {
        r += p->effectiveMass();
    }
    return r;
}

double Vehicle::feltAccel() {
    const double m = getMass();
    return m > 0.0 ? glm::length(lastThrustForce + lastAeroForce) / m : 0.0;
}

void Vehicle::powerTick(double h) {
    double totalGen = 0.0, constantDraw = 0.0;
    double ecCharge = 0.0, ecCapacity = 0.0;
    for(size_t i = 0; i < parts.size(); i++) {
        Part *p = parts[i];
        totalGen += p->powerGen();
        constantDraw += p->powerDrawConstant();
        if(p->isBattery()) {
            ecCharge += p->resources.current[(int)ResourceType::EC];
            ecCapacity += p->capacityOf((int)ResourceType::EC);
        }
    }
    // No EC system -> ungated. Otherwise the wheels need power left over after
    // life support.
    const bool hasEC = (ecCapacity > 0.0) || (totalGen > 0.0) || (constantDraw > 0.0);
    powered_ = hasEC ? ((totalGen > constantDraw) || (ecCharge > 0.0)) : true;
    // The wheels draw only while actually working (powered AND commanding attitude).
    double activeDraw = 0.0;
    const bool wheelsActive = powered_
        && ((stick[0] != 0.0f || stick[1] != 0.0f || stick[2] != 0.0f)
            || slew != SlewNone);
    if(wheelsActive) {
        for(size_t i = 0; i < parts.size(); i++) {
            if(parts[i]->isWheel()) { activeDraw += parts[i]->powerDraw(); }
        }
    }
    const double netWh = (totalGen - constantDraw - activeDraw) * h / 3600.0;
    if(netWh < 0.0) { drainEC(-netWh); }
    else if(netWh > 0.0) { chargeEC(netWh); }
}

void Vehicle::drainEC(double wh) {
    if(wh <= 0.0) { return; }
    double total = 0.0;
    for(Part *p : parts) {
        if(p->isBattery()) { total += p->resources.current[(int)ResourceType::EC]; }
    }
    if(total <= 0.0) { return; }
    double take = (wh < total) ? wh : total;
    for(Part *p : parts) {
        if(!p->isBattery()) { continue; }
        const double have = p->resources.current[(int)ResourceType::EC];
        if(have <= 0.0) { continue; }
        double share = take * have / total;
        if(share > have) { share = have; }
        p->resources.current[(int)ResourceType::EC] = have - share;
    }
}

void Vehicle::chargeEC(double wh) {
    if(wh <= 0.0) { return; }
    double freeCap = 0.0;
    for(Part *p : parts) {
        if(p->isBattery()) {
            freeCap += p->capacityOf((int)ResourceType::EC)
                     - p->resources.current[(int)ResourceType::EC];
        }
    }
    if(freeCap <= 0.0) { return; }
    double add = (wh < freeCap) ? wh : freeCap;
    for(Part *p : parts) {
        if(!p->isBattery()) { continue; }
        const double cap = p->capacityOf((int)ResourceType::EC);
        const double have = p->resources.current[(int)ResourceType::EC];
        const double free = cap - have;
        if(free <= 0.0) { continue; }
        double share = add * free / freeCap;
        if(share > free) { share = free; }
        p->resources.current[(int)ResourceType::EC] = have + share;
    }
}

void Vehicle::getPower(double *gen, double *constDraw, double *charge, double *capacity) {
    double g = 0.0, c = 0.0, q = 0.0, cap = 0.0;
    for(Part *p : parts) {
        g += p->powerGen();
        c += p->powerDrawConstant();
        if(p->isBattery()) {
            q += p->resources.current[(int)ResourceType::EC];
            cap += p->capacityOf((int)ResourceType::EC);
        }
    }
    if(gen) { *gen = g; }
    if(constDraw) { *constDraw = c; }
    if(charge) { *charge = q; }
    if(capacity) { *capacity = cap; }
}

void Vehicle::power_log(double time) {
    double gen, constDraw, charge, capacity;
    getPower(&gen, &constDraw, &charge, &capacity);
    printf("[powerlog] t=%.1fs ship=\"%s\" powered=%d gen=%.1fW "
           "const_draw=%.1fW charge=%.1fWh capacity=%.1fWh\n",
           time, displayName().c_str(), (int)powered_, gen, constDraw, charge, capacity);
    fflush(stdout);
}

int Vehicle::activeStage() { return activeStage_; }

void Vehicle::advanceStage() { if(activeStage_ > minStage_) { activeStage_--; } }

int Vehicle::deployChutes(int stage) {
    int n = 0;
    for(Part *p : parts) {
        if(p->chute != ChuteStowed || !p->isChute()) { continue; }
        if(p->stage != stage) { continue; }
        p->chute = ChuteDeployed;
        n++;
    }
    return n;
}

int Vehicle::numStages() { return totalStages_; }

double Vehicle::getThrust() {
    /* Actual thrust at the current throttle and air state (jets via jetThrust, drag.h). */
    const int as = activeStage();
    const double v_air = glm::length(airRelativeVel(get_center_of_mass()));
    const double rho = airDensityAtCom();
    const double rho_sea = (m_parent != nullptr)
        ? (double)m_parent->surface.atmosphere.sea_level_density : 0.0;
    double t = 0;
    for(Part *p : parts) {
        if(!p->isThruster() || p->stage < as) { continue; }
        if(p->isJet()) {
            t += jetThrust(v_air, rho, rho_sea, p->def->jet_fan_thrust,
                           p->jetFuelRate(), p->def->exhaust_velocity,
                           p->def->jet_intake_area);
        } else {
            t += p->thrust();
        }
    }
    return t * thruster_util * exhaust_scale;
}

double Vehicle::getTWR() {
    return getThrust() / (getMass() * m_parent->g);
}

double Vehicle::getFullThrustTWR() {
    return GetActiveThrust() / (getMass() * m_parent->g);
}

double Vehicle::getMaxTWR() {
    // ALL burnable propellant (rocket and jet): max TWR is at the lightest mass.
    bool burn[(int)ResourceType::Num];
    enginePropellantMask(burn, /*includeJets=*/true);
    const double remaining_fuel = fuelMassMasked(burn); /* kg */
    return GetActiveThrust() / ((getMass() - remaining_fuel) * m_parent->g);
}

void Vehicle::setVelocity(glm::dvec3 vel) {
    SetVelocity(hull, vel);
}

const glm::dvec3& Vehicle::get_center_of_mass(void) {
    m_com = comPos();
    return m_com;
}

glm::dvec3 Vehicle::applyGravity() {
    const double& parent_mass = m_parent->mass;
    const double G = 6.674e-11;
    const glm::dvec3 com = comPos();
    glm::dvec3 gf(0.0);
    glm::dvec3 ff_total(0.0);
    for(Part *p : parts) {
        // effectiveMass: the force must carry the same mass as the COM and the inertia.
        const double m = p->effectiveMass();
        if(m == 0) { continue; }
        const glm::dvec3 b1b2 = partPos(p);
        const double m1m2 = m * parent_mass;
        const double invrsqr = 1.0 / glm::length2(b1b2);
        const double mag = G * m1m2 * invrsqr;
        const glm::dvec3 f = mag * sqrt(invrsqr) * -b1b2;
        /* Applied AT the part, not the COM: the lever delivers the tidal torque, the
           only legitimate external torque on a rigid ship. */
        ApplyForce(hull, b1b2 - com, f);
        gf += f;
        if(frame->isRotFrame()) {
            // In rotating coordinates the ship additionally feels
            // Coriolis + centrifugal (see GetFictitiousAccel in frame.h).
            const glm::dvec3 a_fict = frame->GetFictitiousAccel(b1b2, partVel(p));
            const glm::dvec3 ff = m * a_fict;
            ApplyForce(hull, b1b2 - com, ff);
            ff_total += ff;
        }
    }
    /* The per-part levers hang off the hull's transform origin, which lags the true
       COM during a burn: cancel the spurious (comOffset x F) torque it would add. */
    const glm::dvec3 dcom = comOffset();
    if(glm::length2(dcom) > 0.0) {
        ApplyTorque(hull, -glm::cross(dcom, gf + ff_total));
    }
    return gf;
}

Vehicle::Vehicle() { }

Vehicle::~Vehicle() {
    if(hullInWorld()) { RemoveBody(hull); }
    delete hull; hull = nullptr;
    compoundParts.clear();
    /* ~Part owns what it contains, so a capsule frees its seat occupants
       (part.h). No crew list to walk: a seated kerbal is not a Vehicle. */
    for(Part *p : parts) { delete p; }   // ~Part deletes the Body
}

glm::dvec3 Vehicle::processGravity() {
    return applyGravity();
}

void Vehicle::applyThrustForce(double h) {
    // The whole delta-v budget rests on h being the duration Bullet advances.
    assert(h > 0.0 && "thrust force needs the substep it is applied over");
    const glm::dvec3 com = comPos();
    glm::dvec3 ftotal(0.0);
    /* TVC aims every nozzle at ONE lateral direction in the root part's
       frame (the frame rcsWorldDir uses): a ring of surface-mounted engines
       then ADDS pitch authority instead of coupling stick pitch into a roll
       couple around the ring. The same stick the wheels answer to drives it;
       no EC/wheel gate -- a mechanical gimbal works when the power is dead.
       No stick recenters, so a coasting ship settles.
       The autopilot's aim (tvcSlewLat_, from the slew law's torque split)
       adds onto the stick's, and the SUM is clamped to the stop. Sign per
       engine: a deflection of shared lateral L levers an engine at height
       h (along the stack axis) by torque ~ -h * (shipZ x L), so engines on
       the far side of the COM (reversed engines in a docked pair, upper
       stages above COM mid-burn) must deflect OPPOSITE for their torques
       to add instead of fight. Ring symmetry is untouched: same-height
       engines share the sign. */
    const Part *tvcRef = rootPart();
    glm::dvec3 latShip(0.0);
    if(tvcRef != nullptr && (stick[1] != 0.0f || stick[2] != 0.0f)) {
        latShip = (double)stick[2] * partAxis(tvcRef, 0)
                - (double)stick[1] * partAxis(tvcRef, 1);
        const double l = glm::length(latShip);   // both sticks: keep |lat| <= 1
        if(l > 1.0) { latShip /= l; }
    }
    const glm::dvec3 shipZ = (tvcRef != nullptr) ? partAxis(tvcRef, 2) : glm::dvec3(0.0);
    for(Part *p : parts) {
        if(!p->isThruster()) { continue; }
        if(p->def->gimbal_max > 0.0) {
            glm::dvec3 lat(0.0);
            if(p->armedThrust != 0.0f) {
                lat = p->def->gimbal_max * latShip + tvcSlewLat_;
                const double l = glm::length(lat);
                if(l > p->def->gimbal_max) { lat *= p->def->gimbal_max / l; }
            }
            if(tvcRef != nullptr) {
                const double height = glm::dot(partPos(p) - com, shipZ);
                // Same near-COM dead zone tvcAuthority budgets with: an
                // engine there has no useful lever, so no flip either.
                if(height > 1e-6) { lat = -lat; }
            }
            double target[2];
            gimbalTargets(partRot(p), lat, target);
            gimbalStep(p->gimbal, target, p->def->gimbal_rate, h);
            /* Targets project a <=gimbal_max vector, so the state cannot
               leave its bounds -- only an authoring bug can trip this. */
            assert(std::fabs(p->gimbal[0]) <= p->def->gimbal_max + 1e-9
                && std::fabs(p->gimbal[1]) <= p->def->gimbal_max + 1e-9);
        }
        if(p->armedThrust == 0.0f) { continue; }
        // Along the (possibly gimballed) engine axis, applied AT the engine:
        // an off-axis or tilted engine torques the ship directly.
        const glm::dvec3 ft = partThrustDir(p) * (double)p->armedThrust;
        ApplyForce(hull, partPos(p) - com, ft);
        ftotal += ft;
        if(recordForceVecs) {
            // Same arm Bullet got; the drawer anchors it at the CURRENT COM
            // (see the ForceVec doc in vehicle.h).
            lastForceVecs.push_back({ partPos(p) - com, ft, kForceThrust });
        }
    }
    lastThrustForce = ftotal;
    if(glm::length2(ftotal) < 1e-24) { return; }
    // Same spurious-torque cancellation as applyGravity.
    const glm::dvec3 dcom = comOffset();
    if(glm::length2(dcom) > 0.0) {
        ApplyTorque(hull, -glm::cross(dcom, ftotal));
    }
    /* --dv-log: the impulse this substep hands the integrator. hull->mass is
       the mass Bullet actually accelerates (rebuildCompound), NOT getMass(),
       so a hull mass that lags the draining fuel shows up as a gap against
       dvRocket_. The along-velocity term uses the NON-ROTATING velocity: what
       a burn buys is measured against the conic, and inside the surface frame
       GetVel() is ground-relative. */
    if(dv_log_enabled && hull->mass > 0.0) {
        const double inv_m = 1.0 / (double)hull->mass;
        dvDelivered_ += glm::length(ftotal) * h * inv_m;
        Frame *inertial = frame->getNonRotFrame();
        glm::dvec3 ip, iv;
        comStateIn(inertial, ip, iv);
        const double speed = glm::length(iv);
        if(speed > 0.0) {
            const glm::dvec3 f_inertial = frame->GetOrientRelTo(inertial) * ftotal;
            dvAlong_ += glm::dot(f_inertial, iv) * h * inv_m / speed;
        }
    }
}

// The authored atmosphere as drag.h sees it.
static DragAtmosphere dragAtm(const AtmosphereParams &a) {
    return DragAtmosphere { a.sea_level_density, a.scale_height, a.top() };
}

double Vehicle::airDensityAtCom() const {
    if(m_parent == nullptr) { return 0.0; }
    const AtmosphereParams &atm = m_parent->surface.atmosphere;
    if(atm.sea_level_density <= 0.0 || atm.scale_height <= 0.0) {
        return 0.0;   // no physical atmosphere (a limb rim alone does not count)
    }
    const glm::dvec3 com = comPos();
    const double r = glm::length(com);
    if(r <= 0.0) { return 0.0; }
    // Altitude above SEA LEVEL (the fixed reference radius), like
    // applyAeroForce: the atmosphere is a spherically-symmetric shell.
    return airDensity(dragAtm(atm), r - m_parent->seaLevelRadius());
}

/* The ship's velocity relative to the AIR (#60/#97). The air is at rest in the body's
   ROT frame, so there GetVel() is already air-relative; outside it, subtract the
   co-rotation cross(omega, pos), omega rotated into this frame's axes. */
glm::dvec3 Vehicle::airRelativeVel(const glm::dvec3 &com) {
    if(frame == nullptr || frame->isRotFrame() || m_parent == nullptr
       || m_parent->rot_frame == nullptr) {
        return GetVel();
    }
    Frame *R = m_parent->rot_frame;
    const glm::dmat3 o = R->GetOrientRelTo(frame);   // rot -> this frame
    return GetVel() - glm::cross(o * R->omega(), com);
}

glm::dvec3 Vehicle::applyAeroForce(double h) {
    (void)h;  // a force (not an impulse); Bullet integrates it over the substep
    // Reset so a substep with no air reports zero.
    lastAeroForce = glm::dvec3(0.0);
    lastLiftForce = glm::dvec3(0.0);
    lastAeroTorque = glm::dvec3(0.0);
    lastDragAlt = 0.0;
    lastDragRho = 0.0;
    lastDragAlpha = 0.0;
    lastDragArea = 0.0;
    lastDragCd = 0.0;
    lastChuteCdA = 0.0;
    lastControlDeflections.clear();  // a no-air substep reports no steering
    // Fins swing back to neutral whenever this substep produces no steering
    // (vacuum, no-atmosphere body, below sea level, --drag-cd 0).
    for(Part *p : parts) {
        if(p->def != nullptr && p->def->control_area > 0.0) { p->controlDefl = 0.0; }
    }

    // --drag-cd 0 = no aero at all (the master off switch).
    if(drag_cd <= 0.0) { return lastAeroForce; }

    // Only a body with a PHYSICAL atmosphere (a density model) produces aero.
    if(m_parent == nullptr) { return lastAeroForce; }
    const AtmosphereParams &atm = m_parent->surface.atmosphere;
    if(atm.sea_level_density <= 0.0 || atm.scale_height <= 0.0) {
        return lastAeroForce;
    }

    const glm::dvec3 com = comPos();
    const double r = glm::length(com);
    if(r <= 0.0) { return lastAeroForce; }
    const double alt = r - m_parent->seaLevelRadius();
    lastDragAlt = alt;
    if(alt <= 0.0) { return lastAeroForce; }  // at / below sea level (in the sea)

    const DragAtmosphere da = dragAtm(atm);
    const double rho = airDensity(da, alt);
    lastDragRho = rho;
    // Above the atmosphere top rho is 0; below kRhoFloor the force is unmeasurable.
    if(rho < kRhoFloor) { return lastAeroForce; }

    // GetVel() minus the air's co-rotation when outside the body's rot frame (#97).
    const glm::dvec3 vrel = airRelativeVel(com);
    const double v2 = glm::length2(vrel);
    if(v2 <= 0.0) { return lastAeroForce; }  // at rest in air

    // Frame S read once: constant within a substep.
    glm::dvec3 sPos; glm::dmat3 sRot;
    frameS(sPos, sRot);
    const auto partPosS = [&](const Part *p) {
        return sPos + sRot * p->localPos;
    };

    // One flow frame for every part (one rigid body). Root local axes: nose +Z,
    // right +X, up +Y.
    const Part *root = rootPart();
    const glm::dmat3 rootRot = (root != nullptr) ? sRot * root->localRot
                                                 : glm::dmat3(1.0);
    const glm::dvec3 nose  = rootRot[2];
    const glm::dvec3 right = rootRot[0];
    const glm::dvec3 up    = rootRot[1];
    const AeroFrame fr = aeroFrame(vrel, right, up, nose);
    const double alpha   = fr.valid ? fr.alpha   : 0.0;
    lastDragAlpha = alpha;
    const glm::dvec3 vhat    = vrel / std::sqrt(v2);
    // The flow in S: area and dots are rotation-invariant, so rotate the flow, not the verts.
    const glm::dvec3 vhatS   = glm::transpose(sRot) * vhat;
    const glm::dvec3 liftDir = liftDirection(vrel, right, nose);
    const double q = 0.5 * rho * v2;  // dynamic pressure (shared by all parts)

    glm::dvec3 ftotal(0.0);
    glm::dvec3 lift_total(0.0);
    glm::dvec3 moment(0.0);

    // DRAG: AREA = the ship's hull silhouette facing the flow (projectedArea);
    // COEFFICIENT = the parts' 3-anchor cd (partCd) area-weighted. Applied at the
    // center of pressure so a banked ship weathervanes into the flow.
    {
        glm::dvec3 cp(0.0);      // center of pressure (area-weighted centroid)
        double cpArea = 0.0;
        double cdNum = 0.0;      // sum of (partArea x part cd) for the cd mean
        for(Part *p : parts) {
            if(p->body == nullptr || p->body->hullVerts.empty()) { continue; }
            // hullVerts are part-local: rotate the flow to the part, not the verts to the world.
            const double a = projectedArea(p->body->hullVerts,
                                           glm::transpose(p->localRot) * vhatS);
            cp += a * partPosS(p);
            cpArea += a;
            // cd AS IT FACES THE FLOW (partCd): localRot[2] is the part's nose in S
            // and sRot preserves dots.
            if(p->def != nullptr) {
                const double c = glm::dot(p->localRot[2], vhatS);
                const double cd = partCd(p->def->drag_forward,
                                         p->def->drag_side,
                                         p->def->drag_backward, c);
                if(cd > 0.0) { cdNum += a * cd; }
            }
        }
        // The ship's silhouette from the precomputed union hull (aeroHull).
        const double A_ship = projectedArea(aeroHull, vhatS);
        lastDragArea = A_ship;
        lastDragCd = (cpArea > 0.0) ? (cdNum / cpArea) : 0.0;
        if(cpArea > 0.0) { cp /= cpArea; }
        else { cp = com; }  // degenerate: no per-part area -> act through the COM
        const glm::dvec3 fdrag = dragForce(da, drag_cd * lastDragCd, A_ship,
                                           alt, vrel);
        if(glm::length2(fdrag) > 0.0) {
            const glm::dvec3 rcp = cp - com;
            ApplyForce(hull, rcp, fdrag);        // translation + (rcp x fdrag)
            moment += glm::cross(rcp, fdrag);
            ftotal += fdrag;
        }
    }

    // CHUTE drag (PartDef.chute_area): a FIXED Cd·A per chute. The deployed canopy is
    // not a collision shape, so it cannot join the hull's cd mean.
    for(Part *p : parts) {
        if(p->chute != ChuteDeployed || !p->isChute()) { continue; }
        // The canopy anchor (PartDef::chuteCanopyOffset): its offset from the COM is the
        // weathervane moment hanging the ship below it.
        const glm::dvec3 ri = partPosS(p)
            + sRot * (p->localRot[2] * PartDef::chuteCanopyOffset(*p->def))
            - com;
        // The flow AT THE CANOPY: vrel plus the rotation sweep (ω x r). That sweep IS
        // the harness damping -- without it the restoring torque spins the ship up
        // without bound under its own canopy.
        const glm::dvec3 vp = vrel + glm::cross(GetAngVelocity(hull), ri);
        const double vpl = glm::length(vp);
        if(vpl <= 0.0) { continue; }
        const double qpt = 0.5 * rho * vpl * vpl;
        // Deployment limiter (kChuteAccelLimit): the canopy never pulls harder than
        // a_lim -- physical, and it caps the weathervane torque inside the substep's
        // stability envelope.
        const double area = std::min(p->def->chute_area,
                                     kChuteAccelLimit * getMass() / qpt);
        const glm::dvec3 fchute = -(vp / vpl) * (qpt * area);
        ApplyForce(hull, ri, fchute);            // translation + (ri x fchute)
        moment += glm::cross(ri, fchute);
        ftotal += fchute;
        lastChuteCdA += area;  // the EFFECTIVE (limited) area: the drag log
                               // then shows the limiter working.
    }

    // LIFT (per part): each surface lifts on its OWN area at its OWN position, so the
    // moment comes from the surfaces' distribution.
    for(Part *p : parts) {
        if(p->def == nullptr) { continue; }
        const PartDef *d = p->def;
        const glm::dvec3 flift = liftForce(q, d->lift_area, d->cl, alpha,
                                           liftDir, d->stall_angle);
        if(glm::length2(flift) <= 0.0) { continue; }
        const glm::dvec3 ri = partPosS(p) - com;  // surface's offset from the COM
        ApplyForce(hull, ri, flift);             // translation + (ri x flift)
        moment += glm::cross(ri, flift);
        ftotal += flift;
        lift_total += flift;
    }

    // Control surfaces: each steers ONE axis, driven by that axis's stick alone. The
    // force is the lift law with deflection in place of AoA (controlForce), applied at
    // the surface so its offset from the COM is the steering leverage.
    {
        lastControlDeflections.clear();
        // Force directions out of the flow (mirrors liftDirection): pitch/roll share the
        // "up" plane, yaw uses the ship's right axis.
        const glm::dvec3 yawDirRaw = right - glm::dot(right, vhat) * vhat;
        const double yawLen = glm::length(yawDirRaw);
        const glm::dvec3 yawDir = (yawLen > 0.0) ? yawDirRaw / yawLen
                                                 : glm::dvec3(0.0);
        int ctrlIndex = 0;  // the Nth control surface (disambiguates instances)
        for(Part *p : parts) {
            if(p->def == nullptr) { continue; }
            const PartDef *d = p->def;
            if(d->control_area <= 0.0 || d->max_deflection <= 0.0) { continue; }
            // cl_control if set, else the lift slope cl
            const double clc = controlCl(d->cl, d->cl_control);
            // pure controlAxisParams (pinned in test_shipload)
            const ControlAxisParams ax = controlAxisParams(d->control_axis);
            const float sv = stick[ax.stickIndex];   // 0 when that stick is free
            const glm::dvec3 forceDir = (ax.forceDirKind == 0) ? liftDir : yawDir;
            const glm::dvec3 about =
                (ax.aboutAxis == 0) ? right : (ax.aboutAxis == 1) ? up : nose;
            const glm::dvec3 ri = partPosS(p) - com;
            // The deflection sign is POSITION-DEPENDENT: a tail and a canard need
            // opposite deflections for the same torque (controlDeflectionSign).
            const double sign =
                controlDeflectionSign(ri, forceDir, about, ax.targetSign);
            const double deflection = sign * (double)sv * d->max_deflection;
            p->controlDefl = deflection;  // the mesh swing (Vehicle::Draw)
            lastControlDeflections.push_back({d, ctrlIndex++, deflection});
            if(sv == 0.0f || clc <= 0.0 || deflection == 0.0) { continue; }
            const glm::dvec3 F = controlForce(q, d->control_area, clc,
                                              deflection, forceDir);
            if(glm::length2(F) <= 0.0) { continue; }
            ApplyForce(hull, ri, F);
            ftotal += F;
            moment += glm::cross(ri, F);
        }
    }

    lastAeroForce = ftotal;
    lastLiftForce = lift_total;
    lastAeroTorque = moment;
    if(glm::length2(ftotal) <= 0.0) { return lastAeroForce; }

    // Same spurious-torque cancellation as applyThrustForce / applyGravity.
    const glm::dvec3 dcom = comOffset();
    if(glm::length2(dcom) > 0.0) {
        ApplyTorque(hull, -glm::cross(dcom, ftotal));
    }
    return lastAeroForce;
}

void Vehicle::applyControlForces(double h) {
    /* The force overlay records one substep's worth: clear here, the only
       caller of both force appliers. Off means off: stale vectors stay
       unrecorded (the overlay shows the last armed substep while paused). */
    if(recordForceVecs) { lastForceVecs.clear(); }
    /* Rotation runs BEFORE thrust so the slew law's TVC aim (tvcSlewLat_) is
       aimed before applyThrustForce steps the actuators and pushes along
       them -- same-substep, no lag. */
    applyRotationForce(h);
    applyThrustForce(h);
    applyRcsForce(h);
}

Part * Vehicle::firstWheel() {
    for(Part *p : parts) { if(p->isWheel()) { return p; } }
    return nullptr;
}

void Vehicle::applyRotationForce(double h) {
    /* The autopilot's TVC aim is per-substep: cleared here, re-armed by the
       law below, consumed by applyThrustForce this same substep. */
    tvcSlewLat_ = glm::dvec3(0.0);
    /* Power gate: the wheels are electric -- no power, no wheel torque.
       powered_ comes from powerTick this substep. */
    if(firstWheel() != nullptr && powered_
    && (stick[0] != 0.0f || stick[1] != 0.0f || stick[2] != 0.0f)) {
        /* Manual stick, body-relative: pitch (W/S) about the ship's right axis,
           yaw (A/D) about its up axis, roll (Q/E) about the nose. */
        Part *rw0 = firstWheel();
        const glm::dvec3 pitchAxis = -partAxis(rw0, 0);  // right (W/S)
        const glm::dvec3 yawAxis   = -partAxis(rw0, 1);  // up    (A/D)
        const glm::dvec3 rollAxis  =  partAxis(rw0, 2);  // nose  (Q/E)
        const glm::dvec3 worldAxis =
            (double)stick[0] * rollAxis
            + (double)stick[1] * pitchAxis
            + (double)stick[2] * yawAxis;
        // Every wheel turns the same body on the same axis, so their torques sum.
        ApplyTorque(hull, maxTorque() * worldAxis);
    }
    /* Autopilot: one authority-bounded step of the slew/kill-rot law. It
       runs on ANY attitude authority -- wheels and/or TVC -- so a wheel-less
       ship (rcs_test, tvc_test) or an unpowered one still flies on its
       gimballed engines while they burn, and a ship with neither gets
       silence, not fake slewing (the #401 lesson: never promise a maneuver
       the hardware cannot deliver). */
    if(slew == SlewKillRot) {
        killRotStep(h);
    }
    else if(slew != SlewNone) {
        slewToward(slewTargetDir(), h);
    }
}

void Vehicle::clearRcs() { rcsDir = glm::dvec3(0.0); rcsFiring = false; }

glm::dvec3 Vehicle::rcsWorldDir() const {
    if(glm::length2(rcsDir) < 1e-12) { return glm::dvec3(0.0); }
    const Part *ref = rootPart();
    if(ref == nullptr) { return glm::dvec3(0.0); }
    const glm::dvec3 d = rcsDir.x * partAxis(ref, 0)
                       + rcsDir.y * partAxis(ref, 1)
                       + rcsDir.z * partAxis(ref, 2);
    return glm::normalize(d);
}

Part * Vehicle::firstRcsPart() {
    for(Part *p : parts) { if(p->isRcs()) { return p; } }
    return nullptr;
}

double Vehicle::maxRcsThrust() {
    double t = 0.0;
    for(Part *p : parts) { if(p->isRcs()) { t += p->rcsThrust(); } }
    return t;
}

/* Log label for a BELL (part-local): "+Z"-style, the axis of the biggest
   component -- the nozzle that fired, NOT the force: [rcs] thrust acts along
   the OPPOSITE of the label. Authored bells are axis-aligned today; a
   diagonal one still labels by its dominant axis. */
static std::string nozzleTag(const glm::dvec3 &d) {
    const int ax = (std::fabs(d.x) >= std::fabs(d.y) && std::fabs(d.x) >= std::fabs(d.z)) ? 0
                 : (std::fabs(d.y) >= std::fabs(d.z)) ? 1 : 2;
    const char letters[3] = {'X', 'Y', 'Z'};
    const double comp[3] = {d.x, d.y, d.z};
    return std::string(comp[ax] >= 0.0 ? "+" : "-") + letters[ax];
}

void Vehicle::applyRcsForce(double h) {
    // Reset the log: a substep with no translation demand reports none.
    lastRcsNozzles.clear();
    lastRcsCmd = glm::dvec3(0.0);
    lastRcsAch = glm::dvec3(0.0);
    // Likewise the plume slots: only nozzles FIRING this substep stay lit.
    for(Part *p : parts) {
        if(!p->isRcs()) { continue; }
        for(double &f : p->rcsFired) { f = 0.0; }
    }
    if(glm::length2(rcsDir) < 1e-12) { return; }
    const double Fmax = maxRcsThrust();
    const glm::dvec3 dirW = rcsWorldDir();
    if(Fmax <= 0.0 || glm::length2(dirW) < 1e-12) { return; }
    const glm::dvec3 Fdem = dirW * Fmax;  // full authority on the key, as before
    lastRcsCmd = Fdem;

    /* Nozzle model (#428): authored dirs are the pods' BELL directions
       (exhaust rays, facing out of the exposed faces) in the part's LOCAL
       frame; the ship is pushed OPPOSITE a bell. A pod without dirs is the
       legacy omnidirectional unit -- six dirs along the part's own axes
       (symmetric, so the sign flip is invisible there). rcs_thrust is PER
       nozzle. The enumeration buffers are members (rcsNoz...): this runs
       every substep and must not malloc. */
    std::vector<RcsNozzle> &noz = rcsNoz;
    std::vector<size_t> &nozPart = rcsNozPart;
    std::vector<size_t> &nozLocal = rcsNozLocal;
    noz.clear();
    nozPart.clear();
    nozLocal.clear();
    const glm::dvec3 com = comPos();
    for(size_t pi = 0; pi < parts.size(); pi++) {
        const Part *p = parts[pi];
        if(!p->isRcs()) { continue; }
        const glm::dvec3 axes[3] = { partAxis(p, 0), partAxis(p, 1), partAxis(p, 2) };
        const auto addNozzle = [&](const glm::dvec3 &local, size_t localIdx) {
            glm::dvec3 d(0.0);
            for(int k = 0; k < 3; k++) { d += local[k] * axes[k]; }
            const double L = glm::length(d);
            if(L <= 1e-9) { return; }  // degenerate mount: drop that nozzle
            d /= L;
            // Reaction: a bell throwing gas along d pushes the ship along -d.
            noz.push_back({ -d, partPos(p) - com, p->rcsThrust(), nozzleTag(local) });
            nozPart.push_back(pi);
            nozLocal.push_back(localIdx);
        };
        // The one enumeration (PartDef::rcsNozzleDir): authored dirs in order,
        // else the legacy six. Render walks the same slots for the plumes.
        for(size_t ni = 0; ni < p->def->rcsNozzleCount(); ni++) {
            addNozzle(p->def->rcsNozzleDir(ni), ni);
        }
    }
    if(noz.empty()) { return; }

    rcsAllocateInto(noz, Fdem, rcsScratch);
    const std::vector<double> &c = rcsScratch.c;

    /* Fuel: hydrazine mono, flow from the FIRING thrust (drain-or-nothing,
       as the engines do it). */
    double fired = 0.0;
    for(size_t i = 0; i < noz.size(); i++) { fired += c[i] * noz[i].thrust; }
    if(fired <= 0.0) { return; }
    // flow this substep (kg) = thrust / (Isp * g0) * h
    const double flow = (fired / (kRcsIsp * 9.81)) * h;
    if(!consumeResourceMass(ResourceType::Hydrazine, flow, firstRcsPart())) {
        return;  // no propellant, no force
    }

    /* Force AT THE POD (rel COM), not the COM: an off-center pod translates
       AND spins the ship. The allocator cancels spin where it can; the rest
       is the honest physics #428 exists to deliver. */
    std::vector<glm::dvec3> &fpart = rcsFpart;  // member: reused, see rcsNoz above
    fpart.assign(parts.size(), glm::dvec3(0.0));
    for(size_t i = 0; i < noz.size(); i++) {
        if(c[i] <= 0.0) { continue; }
        const double N = c[i] * noz[i].thrust;
        fpart[nozPart[i]] += noz[i].dir * N;
        parts[nozPart[i]]->rcsFired[nozLocal[i]] = N;  // the plume (render.cpp)
        lastRcsNozzles.push_back({ parts[nozPart[i]]->id, noz[i].tag, N });
    }
    glm::dvec3 Fach(0.0);
    for(size_t pi = 0; pi < parts.size(); pi++) {
        if(glm::length2(fpart[pi]) <= 0.0) { continue; }
        ApplyForce(hull, partPos(parts[pi]) - com, fpart[pi]);
        Fach += fpart[pi];
        if(recordForceVecs) {
            // Same arm Bullet got; the drawer anchors it at the CURRENT COM
            // (see the ForceVec doc in vehicle.h).
            lastForceVecs.push_back({ partPos(parts[pi]) - com, fpart[pi], kForceRcs });
        }
    }
    lastRcsAch = Fach;
    rcsFiring = true;
    // Same spurious-torque cancellation as applyThrustForce / applyAeroForce:
    // RCS now lands at the pod, so it is genuinely off-COM (comOffset is the
    // internal hull-COM vs physical-COM gap that Bullet's rel-force arm uses).
    const glm::dvec3 dcom = comOffset();
    if(glm::length2(dcom) > 0.0) {
        ApplyTorque(hull, -glm::cross(dcom, Fach));
    }
}

void Vehicle::slew_log(double time) {
    if(slew == SlewNone) { return; }
    Part *wheel = firstWheel();
    if(wheel == nullptr) { wheel = rootPart(); }
    if(wheel == nullptr) { return; }
    const glm::dvec3 facing = partAxis(wheel, 2);
    if(slew == SlewKillRot) {
        const glm::dvec3 w = partAngVel(wheel);
        printf("[slew] t=%.3f mode=killrot |w|=%.4f rad/s "
               "w=[%+.4f %+.4f %+.4f]\n",
               time, glm::length(w), w.x, w.y, w.z);
        fflush(stdout);
        return;
    }
    glm::dvec3 target = slewTargetDir();
    const char *mode;
    switch(slew) {
        case SlewPrograde:   mode = "prograde"; break;
        case SlewRetrograde: mode = "retrograde"; break;
        case SlewRadialOut:  mode = "radial-out"; break;
        case SlewRadialIn:   mode = "radial-in"; break;
        case SlewNormal:     mode = "normal"; break;
        case SlewAntiNormal: mode = "anti-normal"; break;
        default:             mode = "slew"; break;
    }
    if(glm::length2(target) < 0.5) { target = glm::dvec3(0.0); }
    target = glm::normalize(target);
    const double E = glm::acos(glm::clamp(glm::dot(facing, target), -1.0, 1.0));
    glm::dvec3 axis = glm::cross(facing, target);
    if(glm::length2(axis) < 1e-12) {
        axis = (std::fabs(facing.y) > 0.9) ? glm::dvec3(1, 0, 0) : glm::dvec3(0, 1, 0);
        axis = glm::normalize(axis - facing * glm::dot(axis, facing));
    }
    axis = glm::normalize(axis);
    const glm::dvec3 rollAxis  = glm::normalize(facing);
    const glm::dvec3 thirdAxis = glm::cross(axis, rollAxis);
    const glm::dvec3 w = partAngVel(wheel);
    const double w_slew  = glm::dot(w, axis);
    const double w_roll  = glm::dot(w, rollAxis);
    const double w_third = glm::dot(w, thirdAxis);
    const glm::dmat3 I = getInertia();
    const double Ieff = glm::dot(axis, I * axis);
    /* Log the same authority budget the law actually has (wheels + TVC), or a
       wheel-less ship's trace would read alpha=0 while it flies on its engines. */
    const double wheelMax = (firstWheel() != nullptr && powered_) ? maxTorque() : 0.0;
    double tvcMax, heffMag;
    tvcAuthority(tvcMax, heffMag);
    const double alpha = (Ieff > 0.0) ? (wheelMax + tvcMax) / Ieff : 0.0;
    const double w_des = (alpha > 0.0) ? std::sqrt(2.0 * alpha * E) : 0.0;
    /* The two directions at 6 dp, not 3: e2e pairs target= with --plane-log's
       n_orb, and a 3 dp component leaves ~3e-3 of dot-product slack -- the same
       size as the error it is there to catch. */
    printf("[slew] t=%.3f mode=%s E=%.4f rad (%.2f deg) "
           "w_slew=%+.4f w_roll=%+.4f w_third=%+.4f |w|=%.4f "
           "alpha=%.3f Ieff=%.0f w_des=%+.4f "
           "nose=[%+.6f %+.6f %+.6f] target=[%+.6f %+.6f %+.6f]\n",
           time, mode, E, glm::degrees(E),
           w_slew, w_roll, w_third, glm::length(w),
           alpha, Ieff, w_des,
           facing.x, facing.y, facing.z, target.x, target.y, target.z);
    fflush(stdout);
}

void Vehicle::att_log(double time) {
    if(hull == nullptr || parts.empty()) { return; }
    const glm::dvec3 nose = partAxis(rootPart(), 2);
    const glm::dvec3 w = GetAngVelocity(hull);
    printf("[attlog] t=%.3fs nose=[%+.4f %+.4f %+.4f] "
           "w=[%+.4f %+.4f %+.4f] |w|=%.4f rad/s\n",
           time, nose.x, nose.y, nose.z,
           w.x, w.y, w.z, glm::length(w));
    fflush(stdout);
}

void Vehicle::tq_log(double time) {
    if(hull == nullptr || parts.empty()) { return; }
    const double G = 6.674e-11;
    const double& parent_mass = m_parent->mass;
    glm::dvec3 F(0.0);
    // effectiveMass: same mass basis as applyGravity
    for(Part *p : parts) {
        const double m = p->effectiveMass();
        if(m == 0) { continue; }
        const glm::dvec3 b1b2 = partPos(p);
        const double r2 = glm::length2(b1b2);
        F += G * m * parent_mass * (-b1b2) / (r2 * sqrt(r2));
        if(frame->isRotFrame()) {
            F += m * frame->GetFictitiousAccel(b1b2, partVel(p));
        }
    }
    for(Part *p : parts) {
        if(p->isThruster() && p->armedThrust != 0.0f) {
            F += partThrustDir(p) * (double)p->armedThrust;
        }
    }
    const glm::dvec3 dcom = comOffset();
    const glm::dvec3 w = GetAngVelocity(hull);
    printf("[tqlog] t=%.3fs |dcom|=%.3e m F=[%.4e %.4e %.4e] "
           "|F|=%.4e N |dxF|=%.3e N m |w|=%.3e rad/s\n",
           time, glm::length(dcom), F.x, F.y, F.z, glm::length(F),
           glm::length(glm::cross(dcom, F)), glm::length(w));
    fflush(stdout);
}

const char * Vehicle::resourceName(int r) {
    switch((ResourceType)r) {
        case ResourceType::Hydrogen:  return "H2";
        case ResourceType::LOX:       return "LOX";
        case ResourceType::EC:        return "EC";
        case ResourceType::Oxygen:    return "O2";
        case ResourceType::Water:     return "H2O";
        case ResourceType::Food:      return "food";
        case ResourceType::Hydrazine: return "N2H4";
        case ResourceType::JetFuel:   return "JF";
        case ResourceType::Num:       break;   /* count, not a resource */
    }
    return "?";
}

const char * Vehicle::resourceUnit(int r) {
    return (ResourceType)r == ResourceType::EC ? "Wh" : "kg";
}

void Vehicle::fuel_log(double time) {
    // distinct group ids, ascending
    std::vector<int> groups;
    for(size_t i = 0; i < parts.size(); i++) {
        const int g = parts[i]->fuelGroup;
        if(g < 0) { continue; }   /* a fuel barrier: in no group */
        bool seen = false;
        for(size_t k = 0; k < groups.size(); k++) {
            if(groups[k] == g) { seen = true; break; }
        }
        if(!seen) { groups.push_back(g); }
    }
    std::sort(groups.begin(), groups.end());
    // One line per sample (e2e-greppable).
    printf("[fuel] t=%.3fs ship=\"%s\"", time, displayName().c_str());
    for(size_t gi = 0; gi < groups.size(); gi++) {
        const int g = groups[gi];
        std::vector<int> res;
        for(int r = 0; r < (int)ResourceType::Num; r++) {
            for(size_t i = 0; i < parts.size(); i++) {
                Part *p = parts[i];
                if(p->fuelGroup != g || !p->isTank()) { continue; }
                if(p->def->capacity[r] > 0.0f) { res.push_back(r); break; }
            }
        }
        printf(" g%d=", g);
        for(size_t ri = 0; ri < res.size(); ri++) {
            if(ri > 0) { printf(" "); }
            const int r = res[ri];
            double cur = 0.0, cap = 0.0;
            std::vector<double> tankCur;
            for(size_t i = 0; i < parts.size(); i++) {
                Part *p = parts[i];
                if(p->fuelGroup != g || !p->isTank()) { continue; }
                if(p->def->capacity[r] <= 0.0f) { continue; }
                cur += p->resources.current[r];
                cap += p->def->capacity[r];
                tankCur.push_back(p->resources.current[r]);
            }
            printf("%s:%.1f/%.1f[", resourceName(r), cur, cap);
            for(size_t ti = 0; ti < tankCur.size(); ti++) {
                if(ti > 0) { printf(","); }
                printf("%.1f", tankCur[ti]);
            }
            printf("]");
        }
    }
    // the fuel links as group ids
    if(!fuelLinks.empty()) {
        printf(" links=");
        bool first = true;
        for(size_t k = 0; k < fuelLinks.size(); k++) {
            const int a = fuelLinks[k].from->fuelGroup;
            const int b = fuelLinks[k].to->fuelGroup;
            if(a < 0 || b < 0 || a == b) { continue; }
            if(!first) { printf(","); }
            printf("g%d->g%d", a, b);
            first = false;
        }
    }
    printf("\n");
    fflush(stdout);
}

/* Each group's total fuel mass now, in one pass over the parts. A group whose
   tanks have all run dry still appears, holding 0.0 -- that is how drainSample
   tells "this group emptied" (a spend of its whole load) from "this group left
   with a stage" (no key at all, so nothing is reported for it). */
std::map<int, double> Vehicle::groupFuelMasses() const {
    std::map<int, double> cur;
    for(size_t i = 0; i < parts.size(); i++) {
        Part *p = parts[i];
        if(p->fuelGroup < 0 || !p->isTank()) { continue; }
        double &total = cur[p->fuelGroup];   // 0.0 for a group that is now empty
        for(int r = 0; r < (int)ResourceType::Num; r++) {
            if(r == (int)ResourceType::EC) { continue; }   // energy (Wh), no mass
            total += p->resources.current[r];
        }
    }
    return cur;
}

/* buildFuelGroups() re-derives the numbering from parts order, so after a
   decouple or a dock the previous sample describes different tanks. Drop the
   baseline AND what has accumulated: losing up to one print interval of drain
   data at a staging event is honest, a rate built by subtracting two different
   group sets is not. */
void Vehicle::dropDrainWindow() {
    drainPrevMass_.clear();
    drainSpent_.clear();
    drainSpentSpan_.clear();
    drainSpan_ = 0.0;
    drainPrimed_ = false;
}

/* --drain-log's sampler: called on every tick the sim steps this ship, with
   the sim time that tick advanced by (g.dt * g.time_accel). Each sample adds
   one tick's spend to the running totals, so the rows tile the timeline --
   whatever the warp, every sim second between two prints is counted exactly
   once. The step is the caller's, not a difference of this ship's clock: a
   ship the player stopped flying is not sampled, and resuming from its stale
   timestamp would bill it for the whole stretch it sat inactive. */
void Vehicle::drainSample(double step) {
    assert(step > 0.0);   // the caller only samples while the sim advances
    const std::map<int, double> cur = groupFuelMasses();
    if(!drainPrimed_) {   // the first sample only sets the baseline
        drainPrimed_ = true;
        drainPrevMass_ = cur;
        return;
    }
    drainSpan_ += step;
    for(std::map<int, double>::const_iterator it = drainPrevMass_.begin();
         it != drainPrevMass_.end(); ++it) {
        std::map<int, double>::const_iterator now = cur.find(it->first);
        if(now == cur.end()) { continue; }   // the group left with a stage
        const double spent = it->second - now->second;
        // No flow, or a transfer in: --fuel-log is where masses are reported.
        if(spent <= 0.0) { continue; }
        drainSpent_[it->first] += spent;
        drainSpentSpan_[it->first] += step;
    }
    drainPrevMass_ = cur;
}

/* --drain-log: thrust this print (N) plus each group's drain rate (kg/s) over
   the sim time covered since the last print. A group that spent nothing prints
   0.000, so the row still shows which groups the ship has. dt= is that covered
   interval, which is what makes a rate readable at any warp: at 10x the rows
   are ~5s of sim apart and dt= says so. Nothing prints while nothing steps --
   a paused or just-loaded ship goes silent instead of repeating a stale row. */
void Vehicle::drain_log(double time) {
    if(drainSpan_ <= 0.0) { return; }
    double thrust = 0.0;
    for(size_t i = 0; i < parts.size(); i++) { thrust += (double)parts[i]->armedThrust; }
    printf("[drainlog] t=%.3fs dt=%.3fs ship=\"%s\" thrust=%.1fN",
           time, drainSpan_, displayName().c_str(), thrust);
    for(std::map<int, double>::const_iterator it = drainPrevMass_.begin();
         it != drainPrevMass_.end(); ++it) {
        double rate = 0.0;
        std::map<int, double>::const_iterator spent = drainSpent_.find(it->first);
        if(spent != drainSpent_.end()) {
            const double span = drainSpentSpan_.count(it->first)
                ? drainSpentSpan_[it->first] : 0.0;
            if(span > 0.0) { rate = spent->second / span; }
        }
        printf(" g%d=%.3f", it->first, rate);
    }
    printf(" kg/s\n");
    fflush(stdout);
    // The row consumed its interval: print-once is what keeps a paused ship
    // silent instead of re-reading the same numbers forever.
    drainSpent_.clear();
    drainSpentSpan_.clear();
    drainSpan_ = 0.0;
}

/* --dv-log: the running delta-v budget. dv_del and dv_rocket are two routes to
   the same number -- the impulse the integrator was handed, and the Tsiolkovsky
   cost of the propellant the engines drained -- so a burn that does not buy the
   velocity it costs shows up as a gap between them. dv_along is the delivered
   impulse projected onto the (non-rotating) velocity direction: on a prograde
   burn it equals dv_del, and the shortfall is what the pointing threw away.
   dv_left is the HUD's number, so the readout is pinned against the flight.
   Two masses, deliberately: `mass` is the analytic ship (getMass, what the fuel
   bookkeeping and the HUD use), `hullmass` is the compound Bullet actually
   accelerates, which rebuildCompound() only re-makes once the two drift past
   kMassRebuildFrac apart. `ve` is the LAST rocket in part order, so on a
   mixed-Isp ship the line reads coherently but that number is not a ship
   property. */
void Vehicle::dv_log(double time) {
    if(hull == nullptr) { return; }
    printf("[dv] t=%.3fs ship=\"%s\" mass=%.2f kg hullmass=%.2f kg "
           "thrust=%.1fN dv_del=%.3f dv_along=%.3f dv_rocket=%.3f "
           "prop=%.3f kg ve=%.1f m/s dv_left=%.1f m/s\n",
           time, displayName().c_str(), getMass(), hull->mass,
           glm::length(lastThrustForce),
           dvDelivered_, dvAlong_, dvRocket_, propSpent_, dvVeLast_,
           getDeltaV());
    fflush(stdout);
}

float Vehicle::GetWheelTorque() {
    double t = 0.0;
    for(Part *p : parts) {
        if(p->isWheel() && p->wheelTorque() > t) { t = p->wheelTorque(); }
    }
    return (float)t;
}

void Vehicle::clearThrust() {
    for(Part *p : parts) { p->armedThrust = 0.0f; }
}

void Vehicle::clearRotCmd() {
    stick[0] = stick[1] = stick[2] = 0.0f;
    slew = SlewNone;
}

void Vehicle::releaseControl() {
    thruster_util = 0.0f;
    clearThrust();
    clearRotCmd();
}

std::vector<Part *> Vehicle::droppedPartsAtStage(int stage) {
    // parent -> children over Part::parent (a tree: one parent edge per non-root part).
    std::map<Part *, std::vector<Part *>> children;
    for(Part *p : parts) {
        if(p->parent != nullptr) { children[p->parent].push_back(p); }
    }
    std::set<Part *> dropped;
    for(Part *p : parts) {
        if(!p->isDecoupler() || p->stage != stage) { continue; }
        dropped.insert(p);   // the decoupler flies off with its stage
        // BFS over the decoupler's child side.
        std::vector<Part *> stack;
        for(size_t i = 0; i < children[p].size(); i++) { stack.push_back(children[p][i]); }
        while(!stack.empty()) {
            Part *q = stack.back(); stack.pop_back();
            if(dropped.count(q)) { continue; }
            dropped.insert(q);
            auto it = children.find(q);
            if(it != children.end()) {
                for(size_t i = 0; i < it->second.size(); i++) { stack.push_back(it->second[i]); }
            }
        }
    }
    return std::vector<Part *>(dropped.begin(), dropped.end());
}

void Vehicle::absorbShip(Vehicle *B, Part *portA) {
    if(B == nullptr || B == this || B->parts.empty()) { return; }
    if(portA == nullptr || !portA->isDockingPort()) { return; }
    /* Before any part surgery: displayName() reads parts[0] for a kerbal. */
    const std::string bName = B->displayName();

    /* Rigid rebase of B's parts into this ship's S: with S at (p, R) and S_B at
       (p_B, R_B), x_S = R^T R_B x_Sb + R^T (p_B - p), and R^T R_B for rotations. */
    glm::dvec3 pA, pB; glm::dmat3 RA, RB;
    frameS(pA, RA);
    B->frameS(pB, RB);
    const glm::dmat3 T_rot = glm::transpose(RA) * RB;
    const glm::dvec3 T_pos = glm::transpose(RA) * (pB - pA);
    for(Part *q : B->parts) {
        q->localPos = T_rot * q->localPos + T_pos;
        q->localRot = T_rot * q->localRot;
        q->gimbal[0] = q->gimbal[1] = 0.0;  // fresh actuators in the merged frame
    }

    // rigid-body state before the move
    const double mA = getMass();
    const double mB = B->getMass();
    const glm::dvec3 vA = GetVelocity(hull);
    const glm::dvec3 vB = GetVelocity(B->hull);
    const glm::dvec3 wA = GetAngVelocity(hull);

    /* Topology: B's root hangs off this ship's port part -- the ports are the joint. */
    Part *bRoot = B->rootPart();
    bRoot->parent = portA;

    for(Part *q : B->parts) { q->owner = this; }   // they are OUR parts now
    // Inventory in B's containers rides B's vehicle via the container's owner.
    {
        std::vector<Part *> stack;
        for(Part *q : B->parts) { stack.push_back(q); }
        while(!stack.empty()) {
            Part *p = stack.back(); stack.pop_back();
            for(Part *it : p->contents) {
                it->owner = this;
                stack.push_back(it);
            }
            for(Part *suit : p->seats) {
                if(suit != nullptr) {
                    suit->owner = this;
                    stack.push_back(suit);
                }
            }
        }
    }
    parts.insert(parts.end(), B->parts.begin(), B->parts.end());
    B->parts.clear();
    // the cached aero hull went with the parts
    B->aeroHull.clear();
    fuelLinks.insert(fuelLinks.end(), B->fuelLinks.begin(), B->fuelLinks.end());
    B->fuelLinks.clear();

    /* Nothing to do for crew: a seat belongs to its capsule Part, and the parts
       moved above. */

    /* Stage counters: the union -- the more-advanced ship (lower counter and floor)
       governs. */
    if(B->totalStages_ > totalStages_) { totalStages_ = B->totalStages_; }
    if(B->activeStage_ < activeStage_) { activeStage_ = B->activeStage_; }
    if(B->minStage_ < minStage_) { minStage_ = B->minStage_; }

    clearThrust();
    clearRotCmd();

    /* B's seams move in BEFORE the new seam, so popping the last peels the outermost
       dock (B) first. */
    seams.insert(seams.end(), B->seams.begin(), B->seams.end());
    B->seams.clear();
    seams.push_back(DockSeam{ portA, bRoot, bName });

    // Rebuild as the union and regroup fuel: the ports are fuel barriers, so the two
    // fuel systems stay separate.
    rebuildCompound();
    buildFuelGroups();
    // The merge renumbered the groups AND brought in tanks this ship never
    // sampled, so --drain-log has no valid baseline for them.
    dropDrainWindow();
    SetVelocity(hull, (mA * vA + mB * vB) / (mA + mB));
    SetAngVelocity(hull, wA);
}

Vehicle * Vehicle::extractSubtreeAsShip(Part *root, const std::string &name, double t) {
    if(root == nullptr) { return nullptr; }
    bool found = false;
    for(Part *p : parts) { if(p == root) { found = true; break; } }
    if(!found) { return nullptr; }

    // The subtree: `root` plus its descendants (BFS over the children map).
    std::map<Part *, std::vector<Part *>> children;
    for(Part *p : parts) {
        if(p->parent != nullptr) { children[p->parent].push_back(p); }
    }
    std::set<Part *> droppedSet;
    std::vector<Part *> dropped;
    droppedSet.insert(root);
    dropped.push_back(root);
    for(size_t i = 0; i < dropped.size(); i++) {
        auto it = children.find(dropped[i]);
        if(it == children.end()) { continue; }
        for(size_t k = 0; k < it->second.size(); k++) {
            Part *q = it->second[k];
            if(droppedSet.count(q)) { continue; }
            droppedSet.insert(q);
            dropped.push_back(q);
        }
    }
    if(droppedSet.size() == parts.size()) { return nullptr; }  // can't split the whole ship

    /* New frame S' = the root's old frame: x' = R_r^T (x - p_r),
       R' = R_r^T R. */
    const glm::dvec3 pR = root->localPos;
    const glm::dmat3 RR = root->localRot;

    // Rigid velocity of the dropped side's COM (before the rebase --
    // partPos needs the current hull).
    double M = 0.0;
    glm::dvec3 comDropped(0.0);
    // effectiveMass: same basis as the rest of the simulation
    for(Part *q : dropped) {
        const double m = q->effectiveMass();
        M += m;
        comDropped += m * partPos(q);
    }
    comDropped /= M;
    const glm::dvec3 vOut = GetVelocity(hull)
                          + glm::cross(GetAngVelocity(hull), comDropped - comPos());
    const glm::dvec3 w = GetAngVelocity(hull);
    glm::dvec3 rootWorldPos; glm::dmat3 rootWorldRot;
    partWorldPose(root, rootWorldPos, rootWorldRot);

    // same frame/home/sun: the split is local
    Vehicle *nv = new Vehicle();
    nv->name = name;
    nv->defPath = "";
    nv->setSoi(frame, t);
    nv->home = home;
    nv->sun = sun;
    nv->scenario = nullptr;
    // A split survivor is the same vessel: keep the original's grip (a plain Vehicle
    // here would lose a subclass's setting).
    nv->hullFriction = hullFriction;

    /* Keep the original order (parent before child): the save format resolves parents
       by reference. Rebase the poses into S'. */
    std::vector<Part *> nvParts;
    for(size_t i = 0; i < parts.size(); i++) {
        if(!droppedSet.count(parts[i])) { continue; }
        Part *q = parts[i];
        q->localPos = glm::transpose(RR) * (q->localPos - pR);
        q->localRot = glm::transpose(RR) * q->localRot;
        q->gimbal[0] = q->gimbal[1] = 0.0;  // the old ship's deflection is not this stage's
        nvParts.push_back(q);
    }
    root->parent = nullptr;   // root of the new ship
    nv->parts = nvParts;
    for(Part *q : nvParts) { q->owner = nv; }   // the dropped side is ITS ship now
    // inventory follows its container
    {
        std::vector<Part *> stack;
        for(Part *q : nvParts) { stack.push_back(q); }
        while(!stack.empty()) {
            Part *p = stack.back(); stack.pop_back();
            for(Part *it : p->contents) {
                it->owner = nv;
                stack.push_back(it);
            }
            for(Part *suit : p->seats) {
                if(suit != nullptr) {
                    suit->owner = nv;
                    stack.push_back(suit);
                }
            }
        }
    }
    nv->controller = nullptr;
    for(size_t i = 0; i < nvParts.size(); i++) {
        if(nvParts[i]->isWheel()) { nv->controller = nvParts[i]; break; }
    }
    if(nv->controller == nullptr) { nv->controller = root; }

    // fuel links: both endpoints dropped -> the new ship; both kept ->
    // this ship; crossing the cut -> dangling, dropped.
    std::vector<FuelLink> nvLinks, keepLinks;
    for(size_t k = 0; k < fuelLinks.size(); k++) {
        const bool fIn = droppedSet.count(fuelLinks[k].from) > 0;
        const bool tIn = droppedSet.count(fuelLinks[k].to) > 0;
        if(fIn && tIn) { nvLinks.push_back(fuelLinks[k]); }
        else if(!fIn && !tIn) { keepLinks.push_back(fuelLinks[k]); }
    }
    nv->fuelLinks = nvLinks;
    fuelLinks = keepLinks;

    /* seams: the same containment rule as fuel links. */
    std::vector<DockSeam> nvSeams, keepSeams;
    for(size_t k = 0; k < seams.size(); k++) {
        const bool portIn = droppedSet.count(seams[k].port) > 0;
        const bool rootIn = droppedSet.count(seams[k].root) > 0;
        if(portIn && rootIn) { nvSeams.push_back(seams[k]); }
        else if(!portIn && !rootIn) { keepSeams.push_back(seams[k]); }
    }
    nv->seams = nvSeams;
    seams = keepSeams;

    /* A kerbal follows its capsule automatically: the seat is containment on
       the capsule Part, which the split above already routed. */

    {
        std::vector<Part *> keep;
        for(Part *p : parts) { if(!droppedSet.count(p)) { keep.push_back(p); } }
        parts.swap(keep);
    }
    const bool controllerDropped = (controller != nullptr && droppedSet.count(controller) > 0);
    rebuildCompound();
    if(controllerDropped) { controller = parts[0]; }
    clearThrust();
    buildFuelGroups();
    dropDrainWindow();   // the regroup just renumbered the tanks --drain-log tracks

    /* Finalize (no tank re-seed), place at the root's world pose, give it its COM
       velocity. The caller enters it into the world. */
    nv->finalize();
    // Its parts left the active ship mid-tick, still armed; this ship was never commanded.
    nv->clearThrust();
    nv->placeShip(rootWorldPos, rootWorldRot);
    nv->setVelocity(vOut);
    SetAngVelocity(nv->hull, w);
    return nv;
}

glm::dmat4 Vehicle::renderXform(Frame *renderFrame) const {
    if(frame == renderFrame) { return glm::dmat4(1.0); }
    return glm::translate(frame->GetPositionRelTo(renderFrame))
         * glm::dmat4(frame->GetOrientRelTo(renderFrame));
}

void Vehicle::Draw(const Camera* camera, Frame *renderFrame) {
    // Light direction (sun -> ship COM) in the render frame's axes. The ship's own
    // position keeps it defined inside the sun's own SOI.
    const glm::dvec3 com_root =
        frame->root_orient * get_center_of_mass() + frame->root_pos;
    glm::vec3 sunlightVec =
        glm::vec3(TerrainBody::LightDirFrom(com_root, sun, renderFrame));

    const glm::dmat4 xform = renderXform(renderFrame);

    /* Precision: ONE common shift (the absolute COM), per-part models COM-relative in
       small coords. Absolute per-part coords round onto the huge coords' ULP grid and
       shake the ship apart. */
    const glm::dmat4 xformShip = xform * glm::translate(get_center_of_mass());

    /* One planet-shadow test per ship per frame, not one per part (#119): its march
       resolution is far coarser than a ship, so per-part sampling buys nothing. */
    const float shadow = ComputeTerrainShadow(m_parent, frame,
                                              get_center_of_mass(), sun);

    /* Deflected fins swing on their VISUAL hinge (controlSurfaceVisualRot):
       the mesh tips the same way the aero law's force bends the ship. The
       hinge axes are the ship's, so the root rotation is hoisted lazily --
       only ships with stick-deflected surfaces pay for it. */
    glm::dmat3 rootRot;
    bool rootRotReady = false;

    for(Part *p : parts) {
        // One body per ship, so a part's pose is derived, not read off its own body.
        glm::dvec3 pp; glm::dmat3 pr;
        partPoseRelCom(p, pp, pr);
        if(p->def != nullptr && p->def->control_area > 0.0 && p->controlDefl != 0.0) {
            if(not rootRotReady) {
                glm::dvec3 rp;
                partPoseRelCom(rootPart(), rp, rootRot);
                rootRotReady = true;
            }
            const ControlAxisParams ax = controlAxisParams(p->def->control_axis);
            pr = glm::mat3_cast(
                     controlSurfaceVisualRot(ax.aboutAxis, p->controlDefl, rootRot))
                 * pr;
        }
        const glm::dmat4 model = glm::translate(pp) * glm::dmat4(pr);
        /* A person, not a module: the flat green texture is tinted per kerbal
           (crew.h), so kerbals who walk at the same time are not clones. A
           seated suit is not in `parts` and a pad suit is not drawn at all, so
           in practice this colors the one walking kerbal. Only a kerbal-type
           part owns a profile (part.h), so every other part stays neutral. */
        DrawOpts opts;
        if(p->kerbal != nullptr) { opts.tint = p->kerbal->tint(); }
        p->body->DrawAt(camera, sunlightVec, shadow, model, xformShip, opts);

        // shroud: the open cylinder hides the engine below
        if(p->shroud != nullptr && hasChildBelow(p)) {
            DrawModelAt(camera, p->shroud, p->body->shader,
                        p->shroud_texture, model, sunlightVec, shadow, xformShip);
        }

        /* Deployed canopy (PartDef.chute_mesh): a quad at the canopy anchor. Placeholder
           visual -- the analytic physics acts at this SAME anchor. */
        if(p->chute == ChuteDeployed && p->chute_mesh != nullptr) {
            const double side = std::sqrt(p->def->chute_area);
            const glm::dmat4 cmodel = model
                * glm::translate(glm::dvec3(
                      0.0, 0.0, PartDef::chuteCanopyOffset(*p->def)))
                * glm::scale(glm::dvec3(side, side, side));
            DrawModelAt(camera, p->chute_mesh, p->body->shader,
                        p->chute_texture, cmodel,
                        sunlightVec, shadow, xformShip);
        }
    }
}

void Vehicle::Command(ShipCmd cmd, bool simActive, double step) {
    if(not simActive)
        return;
    switch(cmd.type) {
        case ThrottleUp:
            // A rate, not a per-tick step: the ramp must feel the same at any
            // --physics-hz.
            assert(step > 0.0 && "throttle ramp needs the tick's duration");
            adjustThrottle(0.6f * (float)step);
            break;
        case ThrottleDown:
            assert(step > 0.0 && "throttle ramp needs the tick's duration");
            adjustThrottle(-0.6f * (float)step);
            break;
        case Thrust:
            ApplyThrust(step);
            break;
        // Stick components in the camera frame (see the `stick` member).
        case Pitch:
            stick[1] = (cmd.amount >= 0) ? +1.0f : -1.0f;
            break;
        case Yaw:
            stick[2] = (cmd.amount >= 0) ? +1.0f : -1.0f;
            break;
        case Roll:
            stick[0] = (cmd.amount >= 0) ? +1.0f : -1.0f;
            break;
        // RCS translation in the ship's own axes (rcsWorldDir).
        case RcsNose:
            rcsDir.z = (cmd.amount >= 0) ? +1.0 : -1.0;
            break;
        case RcsUp:
            rcsDir.y = (cmd.amount >= 0) ? +1.0 : -1.0;
            break;
        case RcsRight:
            rcsDir.x = (cmd.amount >= 0) ? +1.0 : -1.0;
            break;
        case KillRot:
            slew = SlewKillRot;
            break;
        case Prograde:
            slew = SlewPrograde;
            break;
        case Retrograde:
            slew = SlewRetrograde;
            break;
    }
}

glm::dvec3 Vehicle::GetVel() {
    return GetVelocity(hull);
}

void Vehicle::adjustThrottle(float delta) {
    thruster_util += delta;
    if(thruster_util > 1) { thruster_util = 1; }
    if(thruster_util < 0) { thruster_util = 0; }
}

double Vehicle::GetActiveThrust() {
    /* FULL-THROTTLE thrust of the ignited engines: rockets at their rated value, jets
       at their sea-level peak (v = v_e/2), so a jet is dead in vacuum. */
    const int as = activeStage();
    const double rho_sea = (m_parent != nullptr)
        ? (double)m_parent->surface.atmosphere.sea_level_density : 0.0;
    double t = 0;
    for(Part *p : parts) {
        if(!p->isThruster() || p->stage < as) { continue; }
        if(p->isJet()) {
            t += jetThrust(p->def->exhaust_velocity * 0.5, rho_sea, rho_sea,
                           p->def->jet_fan_thrust, p->jetFuelRate(),
                           p->def->exhaust_velocity, p->def->jet_intake_area);
        } else {
            t += p->thrust();
        }
    }
    return t * exhaust_scale;
}

void Vehicle::ApplyThrust(double step) {
    if(thruster_util == 0.0f) { return; } /* zero throttle: no burn, no plume */
    const int as = activeStage();
    // Jet state shared by all jet parts this tick: air-relative speed, COM density.
    const double v_air = glm::length(airRelativeVel(get_center_of_mass()));
    const double rho = airDensityAtCom();
    const double rho_sea = (m_parent != nullptr)
        ? (double)m_parent->surface.atmosphere.sea_level_density : 0.0;
    /* --dv-log: the ship mass as this tick's drains proceed. getMass() walks
       every part and recurses into containment, so take it ONCE here and walk
       it down by each engine's draw instead of once per engine. The drain
       removes exactly what it is told from resources.current, which is what
       effectiveMass() sums, so the cursor stays exact. */
    double dvMass = dv_log_enabled ? getMass() : 0.0;
    for(Part *p : parts) {
        if(!p->isThruster()) { continue; }
        if(p->stage < as) { continue; } /* not ignited yet */
        if(p->isJet()) {
            /* Air-breathing: momentum balance (drag.h jetThrust). Vacuum rho = 0 -> no
               thrust AND no burn. Draws JET FUEL only. */
            const double T = jetThrust(
                v_air, rho, rho_sea, p->def->jet_fan_thrust, p->jetFuelRate(),
                p->def->exhaust_velocity, p->def->jet_intake_area);
            if(T <= 0.0) { continue; }
            const double flow = p->jetFuelRate() * (double)thruster_util * step; /* kg this tick */
            if(consumeResourceMass(ResourceType::JetFuel, flow, p)) {
                p->armedThrust = (float)(T * thruster_util * exhaust_scale);
                m_thrust = 1.0;
            }
            continue;
        }
        /* Size the whole burn by s = min over propellants of (available / desired), so
           they drain in lockstep and the scarcest caps the burn. Sizing BEFORE
           draining is what stops a multi-propellant engine from taking the
           propellants it can get and then failing on one it cannot (#371). */
        double s = 1.0;
        double avail[(int)ResourceType::Num] = {0.0};
        for(int r = 0; r < (int)ResourceType::Num; r++) {
            const double rate_r = p->def->propellant_rate[r];
            if(rate_r <= 0.0) { continue; }
            const double desired = rate_r * (double)thruster_util * step;
            avail[r] = availableResourceMass((ResourceType)r, p);
            // desired <= 0: a zero-length tick
            const double si = (desired > 0.0) ? (avail[r] / desired) : 0.0;
            if(si < s) { s = si; }
        }
        if(s > 1.0) { s = 1.0; }
        if(s <= 0.0) { continue; }
        double dm = 0.0;   // kg this engine actually took, for --dv-log
        for(int r = 0; r < (int)ResourceType::Num; r++) {
            const double rate_r = p->def->propellant_rate[r];
            if(rate_r <= 0.0) { continue; }
            double amt = rate_r * (double)thruster_util * step * s;
            /* desired * (avail / desired) is not avail: it can land an ULP above
               it (25 * (7/25) = 7.000000000000000888), and consumeResourceMass
               refuses a draw the pool cannot cover -- so the last tick of an
               ordinary burn failed its own draw and tripped the assert below.
               Clamp to what the sizing loop measured. */
            if(amt > avail[r]) { amt = avail[r]; }
            const bool drained = consumeResourceMass((ResourceType)r, amt, p);
            assert(drained && "s must cover every draw; no build sets NDEBUG");
            (void)drained;
            dm += amt;
        }
        p->armedThrust = (float)(p->thrust() * thruster_util * exhaust_scale * s);
        m_thrust = 1.0;
        /* --dv-log: the rocket-equation side of the budget. Charge this
           engine's draw against the ship mass as it stands AFTER its drain, so
           a multi-engine ship charges each engine at the mass that engine
           actually pushed. Jets are excluded: they shed mass into the air and
           their thrust already balances intake momentum, so they own no
           vacuum delta-v. */
        if(dv_log_enabled && dm > 0.0) {
            const double m_after = dvMass - dm;
            if(m_after > 0.0) {
                const double ve = p->exhaustVelocity() * exhaust_scale;
                dvRocket_ += ve * log(dvMass / m_after);
                propSpent_ += dm;
                dvVeLast_ = ve;   // last rocket in part order: see dv_log()
            }
            dvMass = m_after;
        }
    }
}

double Vehicle::maxTorque() {
    double t = 0;
    for(Part *p : parts) {
        if(p->isWheel()) { t += p->wheelTorque(); }
    }
    return t;
}

/* TVC attitude authority from the ARMED gimballed engines, so it scales
   with throttle (and falls to zero when the engines quit mid-slew). Small
   angle (gimbal_max ~ 5 deg): a deflection of |lat| rad off the stack axis
   pushes a lateral force T*|lat| at lever |h|, so
     heffMag = Sum(T_i*|h_i|)   N m / rad,  tvcMax = heffMag*sin(gimbal_max).
   The lever is the COM height along the MIXER's stack axis (the root part,
   the frame applyThrustForce aims in); engines near COM height have no
   useful pitch/yaw lever -- excluded, exactly as the mixer's sign flip is
   moot for them. */
void Vehicle::tvcAuthority(double &tvcMax, double &heffMag) const {
    tvcMax = 0.0; heffMag = 0.0;
    const Part *ref = rootPart();
    if(ref == nullptr) { return; }
    const glm::dvec3 com = comPos();
    const glm::dvec3 shipZ = partAxis(ref, 2);
    for(const Part *p : parts) {
        const double armed = p->armedThrust;
        if(armed == 0.0 || p->def->gimbal_max <= 0.0) { continue; }
        const double lever = std::fabs(glm::dot(partPos(p) - com, shipZ));
        if(lever < 1e-6) { continue; }
        heffMag += armed * lever;
        tvcMax  += armed * std::sin(p->def->gimbal_max) * lever;
    }
}

glm::dmat3 Vehicle::getInertia() {
    glm::dvec3 com; glm::dmat3 R;
    fromBt(hull->btBody->getCenterOfMassTransform(), com, R);
    const btVector3 &bi = hull->btBody->getLocalInertia();
    const glm::dmat3 diag(bi.getX(), 0.0, 0.0,
                          0.0, bi.getY(), 0.0,
                          0.0, 0.0, bi.getZ());
    return R * diag * glm::transpose(R);
}

glm::dvec3 Vehicle::slewTargetDir() {
    const glm::dvec3 pos = get_center_of_mass();
    // Radial is the same line in either frame: both are measured from the body's
    // centre, which the surface frame and the inertial one share.
    if(slew == SlewRadialOut) { return  pos; }
    if(slew == SlewRadialIn) { return -pos; }
    const bool prg = (slew == SlewPrograde || slew == SlewRetrograde);
    const bool nrm = (slew == SlewNormal || slew == SlewAntiNormal);
    if(!prg && !nrm) { return glm::dvec3(0.0); }
    /* Prograde and the normal belong to the CONIC, so take the state from the
       non-rotating node: inside the rotating surface frame GetVel() is the
       ground-relative velocity, which for a ship flying slowly points nowhere
       near its orbit's prograde, and the cross product of that pair is not the
       orbit's normal. Then back into this frame -- slewToward steers in it.
       Matches the drawn markers (render.cpp), so what SAS aims at is what the
       prograde icon shows. */
    Frame *inertial = frame->getNonRotFrame();
    glm::dvec3 p, v;
    comStateIn(inertial, p, v);
    const glm::dmat3 to_frame = glm::transpose(frame->GetOrientRelTo(inertial));
    glm::dvec3 dir = to_frame * (nrm ? glm::cross(p, v) : v);
    return (slew == SlewRetrograde || slew == SlewAntiNormal) ? -dir : dir;
}

/* Route one commanded attitude torque through the available actuators.
   wheelMax/tvcMax/heffMag come from the caller's authority budget (the
   wheels' rating when live, the armed engines' TVC authority); tvcSplit
   caps each actuator, so the delivered total never exceeds the budget --
   the autopilot stays no stronger than maxed manual hardware (the
   test_slew3d authority pin), whatever the law commanded. */
void Vehicle::actuateAttitude(glm::dvec3 torque, double wheelMax, double tvcMax,
                              double heffMag) {
    glm::dvec3 tauW, lat;
    tvcSplit(torque, partAxis(rootPart(), 2), tvcMax, heffMag, wheelMax, tauW, lat);
    tvcSlewLat_ = lat;
    if(glm::length2(tauW) > 0.0) { ApplyTorque(hull, tauW); }
}

void Vehicle::slewToward(glm::dvec3 dir, double h) {
    if(glm::length2(dir) < 1e-12) { return; } /* no direction to align to */
    dir = glm::normalize(dir);
    /* Facing reference: the wheels' nose when there are wheels (the law's
       long-standing frame), else the root part's stack axis -- which is the
       frame the TVC mixer steers in, so a wheel-less ship aims consistently
       with its engines. */
    Part *ref = firstWheel();
    if(ref == nullptr) { ref = rootPart(); }
    if(ref == nullptr) { return; }
    const glm::dvec3 facing = partAxis(ref, 2);
    const double E = glm::acos(glm::clamp(glm::dot(facing, dir), -1.0, 1.0));
    if(E < 1e-9) { return; } /* already aligned */
    glm::dvec3 axis = glm::cross(facing, dir); /* + turns the nose toward dir */
    if(glm::length2(axis) < 1e-12) {
        /* nose ~ opposite dir: any axis perpendicular to facing works */
        axis = (std::fabs(facing.y) > 0.9) ? glm::dvec3(1, 0, 0) : glm::dvec3(0, 1, 0);
        axis = glm::normalize(axis - facing * glm::dot(axis, facing));
    }
    axis = glm::normalize(axis);
    const glm::dmat3 I = getInertia();
    const double Ieff = glm::dot(axis, I * axis); /* kg m^2 about the slew axis */
    if(Ieff <= 0.0) { return; }
    /* Authority budget this substep: wheels (when live) + TVC (when the
       gimballed engines are armed), so the braking curve and the actuator
       bound scale with throttle. Nothing available: hold position -- the law
       does not pretend to slew (#401's lesson). */
    const double wheelMax = (firstWheel() != nullptr && powered_) ? maxTorque() : 0.0;
    double tvcMax, heffMag;
    tvcAuthority(tvcMax, heffMag);
    const double authMax = wheelMax + tvcMax;
    if(authMax <= 0.0) { return; }
    const double alpha = authMax / Ieff; /* rad/s^2, actuator-limited */
    const double w_des = std::min(std::sqrt(2.0 * alpha * E), E / (2.0 * h));
    /* Drive the FULL transverse angular velocity toward the braking-curve rate: damping
       only the slew-axis component leaves the third-axis spin alive, which is the
       sustained wobble about a prograde/retrograde target. Roll is left free. */
    const glm::dvec3 w_now = partAngVel(ref);
    const glm::dvec3 w_transverse = w_now - facing * glm::dot(w_now, facing);
    glm::dvec3 dW = axis * w_des - w_transverse; /* desired change in rate */
    glm::dvec3 torque = I * dW / h;
    /* authority bound: at most wheelMax + tvcMax N m, split between them */
    actuateAttitude(torque, wheelMax, tvcMax, heffMag);
}

void Vehicle::killRotStep(double h) {
    Part *ref = firstWheel();
    if(ref == nullptr) { ref = rootPart(); }
    if(ref == nullptr) { return; }
    const glm::dvec3 w = partAngVel(ref);
    if(glm::length2(w) == 0.0) { return; } /* at rest: nothing to kill */
    const double wheelMax = (firstWheel() != nullptr && powered_) ? maxTorque() : 0.0;
    double tvcMax, heffMag;
    tvcAuthority(tvcMax, heffMag);
    if(wheelMax + tvcMax <= 0.0) { return; } /* no attitude authority to fight with */
    const glm::dmat3 I = getInertia();
    /* Full-tensor cancel, as in slewToward: tau = I * (-w) / h. Scaling each world axis
       by I[i][i] alone needs a world-aligned principal basis, else the under-cancel
       couples axes into a period-2 limit cycle. */
    glm::dvec3 torque = I * (-w) / h;
    /* Bound + split: a wheel-less ship cannot kill its ROLL about the stack
       axis (a lateral deflection never levers that way), and the split drops
       exactly that component instead of pretending. */
    actuateAttitude(torque, wheelMax, tvcMax, heffMag);
}

glm::dvec3 Vehicle::GetPositionRelTo(const Part *part, Frame *relTo) {
    glm::dvec3 fpos = frame->GetPositionRelTo(relTo);
    glm::dmat3 forient = frame->GetOrientRelTo(relTo);
    return forient * partPos(part) + fpos;
}

/* --- root-frame state helpers, shared by the two frame-switch guards -------
   A state (pos, vel) in `f`, mapped to universe-root axes. The stasis term is what
   makes a rotating frame's stored velocity a frame-coordinate velocity, so it must
   appear on BOTH sides of a switch or the inertial velocity is off by 2*stasis. */
static glm::dvec3 rootPosIn(Frame *f, const glm::dvec3 &pos) {
    return f->root_orient * pos + f->root_pos;
}

static glm::dvec3 rootVelIn(Frame *f, const glm::dvec3 &pos, const glm::dvec3 &vel) {
    return f->root_orient * (vel + f->GetStasisVelocity(pos)) + f->root_vel;
}

/* Squared Frobenius distance between two rotations: for a small relative
   rotation of theta radians it is 2*theta^2, so 1e-20 is ~1e-10 rad. */
static double rotDist2(const glm::dmat3 &a, const glm::dmat3 &b) {
    double d2 = 0.0;
    for(int c = 0; c < 3; c++) {
        for(int r = 0; r < 3; r++) {
            const double d = a[c][r] - b[c][r];
            d2 += d * d;
        }
    }
    return d2;
}

void Vehicle::moveToFrame(Frame *newFrame, double t) {
    // One rigid body, so a frame change is one pose write and one
    // velocity write; the transform is rigid, so the COM maps like any
    // other point.
    const glm::dvec3 oldCom = get_center_of_mass();
    const glm::dvec3 oldVel = GetVelocity(hull);
    const glm::dvec3 fpos = frame->GetPositionRelTo(newFrame);
    const glm::dmat3 forient = frame->GetOrientRelTo(newFrame);

    glm::dvec3 sPos; glm::dmat3 sRot;
    frameS(sPos, sRot);

    /* Both frames must be clocked at the same instant. Do NOT assert
       `t == frame->rail_time`: t is deliberately one tick AHEAD of the tree here, and
       the t into setSoi is a journal timestamp, not an epoch. */
    assert(frame->rail_time == newFrame->rail_time);

    const glm::dvec3 p0 = rootPosIn(frame, oldCom);
    const glm::dvec3 v0 = rootVelIn(frame, oldCom, oldVel);
    const glm::dmat3 att0 = frame->root_orient * sRot;

    placeShip(forient * sPos + fpos, forient * sRot);

    /* The stored velocity is the frame-coordinate velocity, so inertial velocity is
       R*(v + stasis(p)) + V: the OLD frame's stasis is added here, the NEW frame's
       SUBTRACTED below. Both halves gate together, or a same-frame call is short by
       stasis. */
    const bool reanchor = (frame != newFrame);
    glm::dvec3 vel = oldVel;
    if(reanchor) { vel += frame->GetStasisVelocity(oldCom); }
    vel = forient * vel + frame->GetVelocityRelTo(newFrame);
    const glm::dvec3 newCom = forient * oldCom + fpos;
    const glm::dvec3 newVel = reanchor ? vel - newFrame->GetStasisVelocity(newCom)
                                       : vel;

    // Logged before the asserts: this is the detail you want when one fires.
    printf("@@@ %s frame %s -> %s: com (%.0f %.0f %.0f) -> (%.0f %.0f %.0f)"
           " vel (%.0f %.0f %.0f) -> (%.0f %.0f %.0f) |fpos| %.3g\n",
           displayName().c_str(), frame->name.c_str(), newFrame->name.c_str(),
           oldCom.x, oldCom.y, oldCom.z, newCom.x, newCom.y, newCom.z,
           oldVel.x, oldVel.y, oldVel.z, newVel.x, newVel.y, newVel.z,
           glm::length(fpos));

    // stdout is block-buffered when redirected and abort() never flushes.
    fflush(stdout);
    /* Continuity tripwires: algebraic identities, so they fire on an edit to the
       arithmetic above or a root_orient that has drifted from orthonormal -- dropping
       either stasis term trips these while the epoch assert stays silent. The second
       pair checks the hull's actual transform after placeShip: the COM, not the
       S-origin, survives a switch. Untested on the frame-offset term -- every switch
       the battery drives is same-body, so fpos is zero (#161). */
    assert(glm::length(rootPosIn(newFrame, newCom) - p0) < 1e-6);
    assert(glm::length(rootVelIn(newFrame, newCom, newVel) - v0) < 1e-6);
    assert(rotDist2(att0, newFrame->root_orient * (forient * sRot)) < 1e-20);
    {
        glm::dvec3 pOrigin; glm::dmat3 pBasis;
        fromBt(principal, pOrigin, pBasis);
        glm::dvec3 hullPos; glm::dmat3 hullRot;
        fromBt(hull->btBody->getCenterOfMassTransform(), hullPos, hullRot);
        assert(glm::length(rootPosIn(newFrame, hullPos) - p0) < 1e-6);
        assert(rotDist2(frame->root_orient * (sRot * pBasis),
                        newFrame->root_orient * hullRot) < 1e-20);
    }

    /* Angular velocity is the rotational analogue of the stasis terms: the hull stores
       the spin RELATIVE to its frame, in that frame's axes, so a switch adds the OLD
       frame's omega, rotates into the new frame's axes and subtracts the NEW frame's.
       Left alone, |w| survives but the spin re-axes. Identity when frame == newFrame,
       so unlike the velocity it needs no reanchor gate. */
    const glm::dvec3 wOld = GetAngVelocity(hull);
    SetAngVelocity(hull, forient * (wOld + frame->omega()) - newFrame->omega());

    /* Continuity tripwire, same shape as the pos/vel/attitude ones: an algebraic
       identity that fires only if the transform is wrong. */
    {
        const glm::dvec3 wNew = GetAngVelocity(hull);
        const glm::dvec3 wiOld = frame->root_orient * (wOld + frame->omega());
        const glm::dvec3 wiNew = newFrame->root_orient * (wNew + newFrame->omega());
        const double wscale = glm::length(wiOld) + glm::length(frame->omega())
                            + glm::length(newFrame->omega());
        assert(glm::length(wiNew - wiOld) < 1e-9 * std::max(1.0, wscale));
    }

    /* placeShip teleports the pose but does not clear velocities, so the live
       velocity is written back explicitly. */
    SetVelocity(hull, newVel);

    setSoi(newFrame, t);
}

Frame *Vehicle::soiTarget(const glm::dvec3 &posInFrame, bool skipSameBody) {
    if(glm::length(posInFrame) > frame->soi + kSoiMargin) {
        return frame->parent;   // nullptr at the system root: nowhere to go
    }
    Frame *best = nullptr;
    double bestDist = 0.0;
    for(Frame *child : frame->children) {
        if(skipSameBody && child->body == frame->body) { continue; }
        // ship position in the child's coordinates
        const glm::dvec3 rel = frame->GetOrientRelTo(child) * posInFrame
                             + frame->GetPositionRelTo(child);
        const double dist = glm::length(rel);
        if(dist < child->soi - kSoiMargin
           && (best == nullptr || dist < bestDist)) {
            best = child;
            bestDist = dist;
        }
    }
    return best;
}

void Vehicle::setSoi(Frame *newFrame, double t) {
    if(newFrame == nullptr || newFrame->body == nullptr) {
        /* Headless fixtures (the split tests) build vehicles with no frame tree:
           record the frame, nothing else to re-home. Only valid PRE-placement. */
        frame = newFrame;
        return;
    }
    frame = newFrame;
    m_parent = newFrame->body;
    flog.observe(t, m_parent->name);
}

void Vehicle::switchFrames(double t) {
    if(Frame *target = soiTarget(get_center_of_mass(), false)) {
        moveToFrame(target, t);   // prints the switch with com/vel detail
    }
}

void Vehicle::writeRailPose() {
    // ONE write, and it is the whole ship. rail_pos is the COM, which is
    // exactly the hull's transform origin.
    glm::dvec3 pOrigin; glm::dmat3 pBasis;
    fromBt(principal, pOrigin, pBasis);
    setPosRot(hull, rail_pos, (rail_orient * railRot) * pBasis);
    SetVelocity(hull, rail_vel);
    SetAngVelocity(hull, glm::dvec3(0.0));
}

void Vehicle::comStateIn(Frame *inertial, glm::dvec3 &p, glm::dvec3 &v) {
    p = get_center_of_mass();
    v = GetVelocity(hull);
    if(frame != inertial) {
        v += frame->GetStasisVelocity(p);
        v = frame->GetOrientRelTo(inertial) * v + frame->GetVelocityRelTo(inertial);
        p = frame->GetOrientRelTo(inertial) * p + frame->GetPositionRelTo(inertial);
    }
}

double Vehicle::distanceTo(Vehicle *o) {
    Frame *root = frame;
    while(root->parent) { root = root->parent; }
    glm::dvec3 p, v, q, w;
    comStateIn(root, p, v);
    o->comStateIn(root, q, w);
    return glm::length(p - q);
}

bool Vehicle::inTerrainBand() {
    Frame *inertial = frame->getNonRotFrame();
    glm::dvec3 p, v;
    comStateIn(inertial, p, v);
    const OrbitElements el = computeOrbitElements(p, v, inertial->body->mu);
    return periapsisAsl(el, inertial->body->seaLevelRadius()) <= kTerrainBand;
}

/* isGrounded's speed threshold must sit above a walking kerbal's (kWalkSpeed);
   see kShipGroundBand/kShipGroundSpeed in constants.h. */

bool Vehicle::inAtmosphere() {
    if(m_parent == nullptr) { return false; }
    const AtmosphereParams &atm = m_parent->surface.atmosphere;
    if(atm.sea_level_density <= 0.0 || atm.scale_height <= 0.0) {
        return false;   // a visual-only rim is not a drag regime
    }
    return glm::length(comPos()) - m_parent->seaLevelRadius() < atm.top();
}

bool Vehicle::isGrounded() {
    if(m_parent == nullptr || !frame->isRotFrame()) { return false; }
    if(glm::length(GetVel()) >= kShipGroundSpeed) { return false; }
    const glm::dvec3 com = get_center_of_mass();   // valid on rails too
    const double r = glm::length(com);
    if(r <= 0.0) { return false; }
    const double alt = r - m_parent->GetTerrainHeight(glm::vec3(glm::normalize(com)));
    return alt < kShipGroundBand;
}

bool Vehicle::canRail() {
    if(onRails) { return true; }
    /* A parked ship freezes; anything clear of the air is a valid conic. A
       suborbital arc is NOT refused: see the header for why periapsis is not a
       gate, and railsTick for where such an arc actually stops. */
    return isGrounded() || !inAtmosphere();
}

bool Vehicle::goOnRails() {
    if(onRails) { return true; }
    if(!canRail()) { return false; }

    /* The COM state in the body's inertial frame node, where the trajectory
       is a Kepler conic. The parked ship's residual spin is discarded with
       its attitude (writeRailPose zeroes it). */
    Frame *oldFrame = frame;
    Frame *inertial = frame->getNonRotFrame();
    const glm::dvec3 com_frame = get_center_of_mass();  // old frame coords
    const glm::dvec3 vel_frame = GetVelocity(hull);     // old frame coords
    glm::dvec3 p, v;
    comStateIn(inertial, p, v);

    // Freeze only a ship actually resting on the surface; an orbiting one
    // follows its conic.
    const bool grounded = isGrounded();

    // Frame S's axes at park time. Nothing per-part to snapshot: the ship
    // is rigid, so the authored local poses ARE the parked geometry.
    glm::dvec3 sPosPark;
    frameS(sPosPark, railRot);

    if(grounded) {
        // Frozen: the pose is static in the rotating surface frame.
        rail_pos = com_frame;
        rail_vel = vel_frame;
        rail_orient = glm::dmat3(1.0);
        railFrozen = true;
        rail_epoch = frame->rail_time;
    } else {
        rail_pos = p;
        rail_vel = v;
        rail_orient = oldFrame->GetOrientRelTo(inertial);
        // Same-body frame hop (a rail conic lives in the inertial node),
        // not an SoI re-home: deliberately a bare write, not setSoi.
        frame = inertial;   // on rails, ship->frame == its inertial node
        railFrozen = false;
        // From the frame, not g.time: the live Bullet pose this came from is
        // expressed against the transforms as last snapshotted, and that is the
        // instant rail_pos describes.
        rail_epoch = inertial->rail_time;
    }

    /* Fins go neutral with the sim: nothing rewrites controlDefl on rails, so
       a grounded parked ship (canRail() admits it in air) would otherwise
       hold its last stick deflection forever. */
    for(Part *p : parts) {
        p->controlDefl = 0.0;
        if(p->isRcs()) { for(double &f : p->rcsFired) { f = 0.0; } }
    }
    lastRcsNozzles.clear();  // likewise the [rcs] log: no substeps, no nozzles
    lastForceVecs.clear();   // and the force overlay: no substeps, no arrows

    // out of the world: the one body
    if(hullInWorld()) { RemoveBody(hull); }

    onRails = true;
    if(!railFrozen) { writeRailPose(); }
    if(grounded) {
        printf("@@@ %s frozen on rails (grounded around %s)\n",
               displayName().c_str(), frame->body->name.c_str());
    } else {
        const OrbitElements el =
            computeOrbitElements(p, v, inertial->body->mu);
        printf("@@@ %s parked on rails around %s: sma=%.6g m ecc=%.4f\n",
               displayName().c_str(), inertial->body->name.c_str(),
               el.semi_major, el.ecc);
    }
    return true;
}

void Vehicle::leaveRails() {
    if(!onRails) { return; }
    writeRailPose();
    if(!hullInWorld()) { AddPhysicsBody(hull); }
    onRails = false;
    railFrozen = false;
    printf("@@@ %s left the rails around %s\n",
           displayName().c_str(), frame->body->name.c_str());
}

RailEnd Vehicle::railsTick(double t, const double step) {
    if(!onRails || railFrozen) { return RailEndNone; }
    assert(frame->body != nullptr);
    // The caller must hand us the instant the frame tree was last snapshotted at: the
    // conic advances by `step` but its RESULT composes with those transforms. Exact
    // equality holds because UpdateOrbitRails() gets this same g.time.
    assert(t == frame->rail_time);
    /* One advance, one SoI test at the end of it. propagateKepler is exact for any
       step; the SoI TEST is not -- at high warp a tick can carry the ship clean through
       a planet's sphere, and the player brings the warp down for that. Sampling
       MID-step cannot be honest: the handoff must land on the tree's instant, but the
       crossing happened earlier (#164). */
    const glm::dvec3 prev_pos = rail_pos;   // an exact-radial conic can hit r = 0
    propagateKepler(rail_pos, rail_vel, frame->body->mu, step,
                    rail_pos, rail_vel);
    rail_epoch = t;   // the advance ends at the caller's instant, == the tree's
    if(Frame *target = soiTarget(rail_pos, true)) {
        printf("@@@ %s rails SoI switch: %s -> %s\n",
               displayName().c_str(), frame->name.c_str(), target->name.c_str());
        moveToRailFrame(target, t);
    }

    /* A conic models a vacuum coast. Stop where the air or the ground needs
       physics -- the endpoint test shares the SoI test's blind spot, so a step
       long enough to jump the air is the player's to avoid. */
    RailEnd end = RailEndNone;
    const double r = glm::length(rail_pos);
    const double ref = m_parent->seaLevelRadius();
    const AtmosphereParams &atm = m_parent->surface.atmosphere;
    const bool has_air = atm.sea_level_density > 0.0 && atm.scale_height > 0.0;
    if(has_air && r - ref < atm.top()) { end = RailEndAtmosphere; }
    /* max_height is the highest relief above sea level: a cheap sphere that
       contains every peak, so the terrain noise runs only for a ship that
       could actually be in it. */
    if(r < ref + m_parent->surface.max_height) {
        if(r > 0.0) {
            Frame *rot = frame->getRotFrame();
            const glm::dvec3 com =
                frame->GetOrientRelTo(rot) * rail_pos + frame->GetPositionRelTo(rot);
            const double ground =
                m_parent->GetTerrainHeight(glm::vec3(glm::normalize(com)));
            if(r < ground) {
                // Land the COM on the surface; the ground park below freezes it.
                rail_pos *= ground / r;
                end = RailEndGround;
            }
        } else {
            /* Through the focus: a radial conic has no direction left to place
               it on, so put it back where the step started. */
            rail_pos = prev_pos;
            end = RailEndGround;
        }
    }
    if(end != RailEndNone) {
        if(end == RailEndGround) {
            /* A warp stop cannot model an impact (Bullet would tunnel at this
               speed), so park the way a landed ship parks: frozen in the spin
               frame, at rest. Control input wakes it. */
            Frame *rot = frame->getRotFrame();
            const glm::dmat3 O = frame->GetOrientRelTo(rot);
            rail_pos = O * rail_pos + frame->GetPositionRelTo(rot);
            rail_orient = O * rail_orient;
            rail_vel = glm::dvec3(0.0);
            frame = rot;
            rail_epoch = rot->rail_time;
            railFrozen = true;
            writeRailPose();
            return end;
        }
        leaveRails();   // writes the pose and re-adds the hull
        return end;
    }
    writeRailPose();
    return RailEndNone;
}

void Vehicle::railRootState(glm::dvec3 &p, glm::dvec3 &v) const {
    p = rootPosIn(frame, rail_pos);
    v = rootVelIn(frame, rail_pos, rail_vel);
}

void Vehicle::moveToRailFrame(Frame *newFrame, double t) {
    /* Epoch consistency: the rail state and BOTH frames must describe the same
       instant. The tree is re-snapshotted at the END of each tick, so composing a rail
       state from earlier puts the ship where its parent WILL be. Exact equality is
       deliberate: a mismatch is a desync, not rounding drift. */
    assert(frame->rail_time == newFrame->rail_time);
    assert(rail_epoch == frame->rail_time);

    glm::dvec3 p0, v0;
    railRootState(p0, v0);
    const glm::dmat3 att0 = frame->root_orient * rail_orient;
    // Round-off scales with the FRAMES' root speeds, not the ship's: a ship can have
    // near-zero ROOT speed (a retrograde burn cancelling the parent's), so |v0| is not
    // a usable scale.
    const double vscale = glm::length(rail_vel) + glm::length(frame->root_vel)
                        + glm::length(newFrame->root_vel);

    const glm::dmat3 O = frame->GetOrientRelTo(newFrame);
    rail_vel = O * rail_vel + frame->GetVelocityRelTo(newFrame);
    rail_pos = O * rail_pos + frame->GetPositionRelTo(newFrame);
    rail_orient = O * rail_orient;

    /* The ship must end up INSIDE the frame it was handed to: a tautology for the
       endpoint-only soiTarget test, but it fails loudly if a target is ever chosen from
       a sample the ship has since left (#164). The upward case is exempt. */
    assert(newFrame == frame->parent
           || glm::length(rail_pos) < newFrame->soi);

    setSoi(newFrame, t);
    // Re-anchored, not advanced: same instant, new frame. Taken from the frame so
    // it cannot drift from the invariant the asserts above just checked.
    rail_epoch = frame->rail_time;

    /* Continuity tripwires: with GetPositionRelTo / GetOrientRelTo / GetVelocityRelTo
       as written these are algebraic identities. They fire on an edit to the arithmetic
       or a non-orthonormal root_orient, never on a stale epoch -- the asserts above
       see that. */
    glm::dvec3 p1, v1;
    railRootState(p1, v1);
    assert(glm::length(p1 - p0) < 1e-10 * std::max(1.0, glm::length(p0)));
    assert(glm::length(v1 - v0) < 1e-10 * std::max(1.0, vscale));
    assert(rotDist2(att0, newFrame->root_orient * rail_orient) < 1e-20);
}
