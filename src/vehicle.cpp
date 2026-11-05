// vehicle.cpp -- Vehicle's method definitions + ship building + spawn scenarios.
#include "vehicle.h"

#include <algorithm>
#include <bit>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <map>
#include <numbers>
#include <stdexcept>

#include "buildings.h" // find_launch_site, buildingOrient (a pad's own axes)
#include "angmath.h"   // acosUnit: the one acos(clamp) spelling (#622)
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
        // Authored trim (load_ship_def refused it on a part that cannot use one).
        part->trimDeg = physical[i].trim;

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
    /* flying-low's altitude, but SUBORBITAL: its circular speed (2.4 km/s inertial,
       2.2 km/s of AIR on Kerbin) puts ~700 g of drag on the shipped jet at the
       instant it spawns, which is a chute-entry rig, not an airframe test. 0.08 of
       the air-relative circular speed starts level at 180 m/s on Kerbin -- 2.5x the
       71 m/s stall speed of a 3 t two-wing hull at 7 km -- 75 m/s on Duna (under
       its 122 m/s stall there, so the bed glides) and 267 m/s on Eve (3.4x stall in
       thick air: a hard start) (#451). What the bed does not fix is what it
       exposes: at these speeds the shipped hulls drag as ~31 m^2 of bluff body at
       Cd 0.76, so the jet bleeds 180 -> 103 m/s and sinks ~65 m/s while the aero
       torque weathervanes it back to zero AoA, which zeroes its lift (#454/#399). */
    {"flying-slow",    true,  false, 0.1,  false, -1, 0.0,     0.0, 0.0, 0.0, true, false, 0.08},
    /* The same altitude band at an airspeed an airframe CRUISES at rather than
       2.5x it. 0.08 of the air-relative circular speed (flying-slow) suits hulls
       that fly at several hundred m/s; a light wing trims near 70-90 m/s, so that
       bed spends the first 40 s of every run trading the excess for height and
       coming back through a phugoid (the shipped skiff, 6 s samples: 110 -> 13 m/s
       and back, 1.0 km of altitude).
       This one starts at the trim point instead: 80 m/s of air on Kerbin, ~33 m/s
       on Duna, ~120 m/s on Eve. Speed alone is not a trim: aimed along the air the
       shipped skiff settles at its zero-moment AoA of 2.5 deg, which is 50..64 m/s
       of flight against a 10..12 m/s sink and 5.6 kN of weight. The trimmed template
       (skiff_trim, 12 deg of elevator) holds 4.9 deg and glides at 47..50 m/s once its
       phugoid damps -- the design
       point (#568 re-measured both on the pod-on-tank hull; the one-piece spindle
       this replaces sat at 1.6 deg hands off and dove to 130 m/s). The bed gives the
       airspeed; the airframe brings the angle of attack, and the launch AoA prints on
       the Spawn line so a bed that is not trimmed is visible without a --drag-log
       (#569). */
    {"flying-cruise",  true,  false, 0.1,  false, -1, 0.0,     0.0, 0.0, 0.0, true, false, 0.0356},
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

// The same basis with the roll NAMED: +Y along `up_ref` once its component along
// the nose is removed. The one above picks its roll axis from three candidates,
// and a candidate is chosen by comparison, so `dir` tilting a degree or two off
// a coordinate plane rotates the whole airframe 90 deg about its own nose.
glm::dmat3 faceAlong(const glm::dvec3 &dir, const glm::dvec3 &up_ref)
{
    const glm::dvec3 z = glm::normalize(dir);
    const glm::dvec3 y0 = up_ref - glm::dot(up_ref, z) * z;
    assert(glm::length2(y0) > 1e-12);   // up_ref along the nose: no roll to solve for
    const glm::dvec3 y = glm::normalize(y0);
    const glm::dvec3 x = glm::cross(y, z);
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

        // Circular speed (vis-viva with semi-major axis == r); esc_frac makes it
        // a hyperbola instead. speed_frac is NOT applied here: it scales the
        // frame-relative velocity below, where the air is what it moves through.
        assert(!(sc.esc_frac > 0.0 && sc.speed_frac > 0.0));
        const double v_circ = sqrt(home->mu / r);
        const double speed = sc.esc_frac > 0.0 ? sc.esc_frac * sqrt(2.0) * v_circ
                         : v_circ;

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
    glm::dvec3 vel = glm::transpose(frame->root_orient)
                   * (velWorld + home->frame->root_vel - frame->root_vel)
                   - frame->GetStasisVelocity(target);

    /* A speed_frac bed is slow RELATIVE TO THE AIR, which means relative to the
       frame: the atmosphere co-rotates, so in the body's rotational frame this
       velocity IS the air-relative one (Vehicle::airRelativeVel). Scaling the
       inertial circular speed instead leaves Kerbin's flying-slow bed with 16 m/s
       of air -- the ground and its air already move 177 m/s prograde -- i.e. a
       drop from the pad, not a start in flight (#451). */
    if(sc.speed_frac > 0.0) {
        assert(sc.ell_phase < 0);   // "of circular speed" is meaningless on an ellipse
        vel *= sc.speed_frac;
    }

    if(frame != ship->frame) {
        ship->moveToFrame(frame, t);
    }

    /* The nose along prograde, the roll from the local vertical -- but only for a
       bed that FLIES. `target` is in the resolved frame's axes, so `orient` must
       be too: faceAlong returns a root-frame attitude. A flying bed's roll is not
       free: on Kerbin the co-rotation term tilts the air-relative velocity 1.5 deg
       out of the orbit plane, which is enough to flip faceAlong's roll AXIS 90 deg
       and start the airframe on its side -- lifting sideways, with W and S yawing
       it instead of pitching it (#451). No orbital bed cares about its roll, and
       its goldens were authored with whichever axis faceAlong preferred. */
    const glm::dmat3 orient = sc.atmo_frac
        ? glm::transpose(frame->root_orient)
          * faceAlong(frame->root_orient * vel,
                      frame->root_orient * glm::normalize(target))
        : glm::transpose(frame->root_orient) * faceAlong(velWorld);
    ship->placeShipAtCom(target, orient);
    ship->setVelocity(vel);

    /* |v| is the authored start: the orbital speed for a circular/elliptical bed,
       the AIR-relative speed for a speed_frac one -- in the body's rotational
       frame the air sits still, so that is the only speed a flight bed feels.
       That one field meaning two things costs debugging sessions (#459): on
       Kerbin flying-low |v| says 2412 m/s while the air only ever feels
       2112-2251 m/s, because the atmosphere co-rotates. So an in-atmosphere bed
       also prints the number the aero pass will actually use, from the same
       helper applyAeroForce uses and after the ship is placed and its velocity
       set -- so the two cannot drift apart. */
    char air_txt[64] = "";
    if(sc.atmo_frac) {
        /* The launch angle of attack, from the law's own AeroFrame so a readout and
           the law cannot drift apart. Read it for what it is: it prints 0.0 for every
           shipped bed BY CONSTRUCTION, because spawn_vehicle points the nose along
           the velocity with faceAlong() just above and the air sits still in a
           rotational frame. It is a canary, not a trim diagnostic -- wind, or a bed
           that did author an attitude, would put a real signed angle here -- and like
           every other AoA readout it refuses to answer where the flow has no
           direction (no air, or under kAoAFlowFloor), printing `-` rather than a 0.0
           that would look like a trimmed hull. What decides whether a flight bed is
           trimmed is the airframe's incidence/trim (#569, res/ships/README.md step
           3); the per-tick AoA is in --drag-log. */
        const glm::dvec3 air = ship->airRelativeVel(target);
        const Part *ref = ship->attitudeRef();
        const AeroFrame fr = aeroFrame(air, ship->partAxis(ref, 0),
                                       ship->partAxis(ref, 1), ship->partAxis(ref, 2));
        if(ship->airDensityAtCom() > 0.0 && fr.valid && std::isfinite(fr.alpha)
           && fr.v >= kAoAFlowFloor) {
            double aoa = glm::degrees(fr.alpha);
            /* Below the print's own resolution the sign is floating-point dust (the
               nose is placed ON the flow, so this is zero plus roundoff), and a bare
               "-0.0" invites a search for a tilt that is not there. */
            if(std::fabs(aoa) < 0.05) { aoa = 0.0; }
            snprintf(air_txt, sizeof air_txt, " |v_air| = %.1f m/s, AoA = %+.1f deg",
                     glm::length(air), aoa);
        } else {
            snprintf(air_txt, sizeof air_txt, " |v_air| = %.1f m/s, AoA = -",
                     glm::length(air));
        }
    }
    printf("Spawn '%s' around %s: frame '%s' @ world (%.0f, %.0f, %.0f), r = %.0f m, |v| = %.1f m/s%s\n",
           sc.name, home->name.c_str(), frame->name.c_str(),
           shipWorldPos.x, shipWorldPos.y, shipWorldPos.z,
           glm::length(shipWorldPos - center),
           sc.speed_frac > 0.0 ? glm::length(vel) : glm::length(velWorld),
           air_txt);
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
        /* A def may tilt the airframe off the local vertical (ShipDef::attitude,
           #461): the site only ever adds a yaw, so every hull -- wings included --
           stands on its tail and the wings lift sideways up the whole climb.
           The tilt also settles the roll: once the nose is off the vertical, the
           vertical can serve as faceAlong's up reference instead of its
           three-axis pick, so the airframe starts wings level whatever the site
           heading is (#460). Pitch 0 keeps the pad pose exactly as it was,
           including that pick -- with the nose ON the vertical there is no roll
           left to solve for. */
        if(def.has_attitude) {
            const double pitch = glm::radians(def.attitude_pitch);
            const glm::dmat3 site_basis = buildingOrient(
                pad_dir,
                (site->heading_deg + def.attitude_yaw) * std::numbers::pi / 180.0);
            pad_orient = pitch > 0.0
                ? faceAlong(glm::normalize(site_basis
                                          * glm::dvec3(std::sin(pitch), 0.0, std::cos(pitch))),
                            pad_dir)
                : site_basis;
        }
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
        inventorySetOwner(suit, v);
        v->finalize();
        /* Free the minted part only after the rebuild: until finalize() swaps
           them, the compound and its geomS mirror still hold child pointers
           into its shape stack. */
        delete minted;
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
    delete geomS; geomS = nullptr;    // its child shapes belong to the parts
    principal = btTransform::getIdentity();
    if(parts.empty()) {
        // An emptied hull has no cached geometry: refreshAeroGeom must re-key
        // (and re-generation the silhouette cache) rather than trust valid. The
        // table itself goes too -- a node filled by the old part set would
        // survive as a non-zero silhouette for a ship with no hull to apply it
        // to, and the balloon law could not lie that way.
        aeroSilParts.clear();
        aeroSil.clear();
        aeroGeomValid = false;
        return;
    }

    // ownership back-pointers (part.h)
    for(Part *p : parts) { p->owner = this; }

    geomS = new btCompoundShape(false, (int)parts.size());
    geomMasses.resize(parts.size());
    btScalar total = 0;
    for(size_t i = 0; i < parts.size(); i++) {
        Part *p = parts[i];
        geomS->addChildShape(toBt(p->localPos, p->localRot), p->body->shape);
        // effectiveMass: body mass + contained crew/inventory (part.h)
        geomMasses[i] = (btScalar)p->effectiveMass();
        /* Bullet's calculatePrincipalAxisTransform btAsserts every child mass
           > 0, but btAssert compiles out unless BT_DEBUG is defined (btScalar.h)
           and we never define it, so that guard is not live in ANY config we
           build (#389). The invariant is ours: a zero-mass child would keep its
           collision hull and its drag silhouette while weighing nothing, and
           --mass-log would still report mass and hullmass agreeing.
           make_part asserts the def's mass and the save loader refuses negative
           contents, so a zero here means a Part was minted some other way. */
        assert(geomMasses[i] > 0 && "a part in the compound weighs nothing");
        total += geomMasses[i];
        compoundParts.push_back(p);
    }
    /* A ship is ONE Body, so it is the SUM that has to fit: an infinite total makes
       calculatePrincipalAxisTransform answer with a NaN inertia tensor and a NaN
       centre of mass (#547) -- an unrotatable ship rather than a refusal. Both
       loaders refuse such a ship (shipDefFromJson, validateShipParts), so landing
       here means a Part was minted past those doors -- by a debug command, or a def
       that skipped a gate. The one place in play the total ADDS two ships is the
       dock, and absorbShip refuses before it merges (#583). Costs nothing in a
       release. */
    assert(std::isfinite((double)total)
           && "the ship's total mass overflows what the game can store");

    btVector3 inertiaDiag(0, 0, 0);
    geomS->calculatePrincipalAxisTransform(&geomMasses[0], principal, inertiaDiag);

    /* The solve's OTHER input (#582). Mass is finite per part and in total, and it
       is still not the only quantity this forms: btCompoundShape.cpp:213 multiplies
       each child pose by its mass for the centre, and :244 weighs a SQUARED offset
       for the tensor. A finite mass at an absurd distance overflows either --
       1e307 kg spaced 1e10 m apart is 1e327 of tensor -- and diagonalize() answers
       +inf with a NaN basis and a NaN centre, i.e. a hull that cannot be turned.
       validateShipParts refuses such a pose at load (SavedPoseSum), so landing here
       means a part moved past that door -- which is what absorbShip's own door
       (#583) exists to prevent in play. Costs nothing in a release, and
       refreshMassProps re-solves these SAME child poses, so it needs no copy. */
    {
        const btVector3 &po = principal.getOrigin();
        assert((std::isfinite((double)po.x()) && std::isfinite((double)po.y())
                && std::isfinite((double)po.z())
                && std::isfinite((double)inertiaDiag.x())
                && std::isfinite((double)inertiaDiag.y())
                && std::isfinite((double)inertiaDiag.z()))
               && "the ship's pose spread overflows what the game can store");
    }

    /* Re-base the children into the COM/principal frame: left in S, the body's origin
       sits at the root part while its inertia is diagonal about the principal axes --
       inconsistent under off-axis torque.
       geomS gets no dbvt AABB tree (it never enters a world and nothing walks its
       children the way Bullet does); the live compound keeps its tree, because
       btCollisionWorld::rayTest/convexSweepTest (btCollisionWorld.cpp:572/908) and
       btCompoundCollisionAlgorithm (btCompoundCollisionAlgorithm.cpp:58/252) cull
       children through getDynamicAabbTree(). Re-minting that tree was most of
       rebuildCompound's cost -- 13.4k allocations per 20s of throttle -- so the
       point of keeping it is that refreshMassProps() no longer rebuilds. An
       A/B (tmp/heap_pass_c_tree.sh, tree on vs off, orbit-burn and pad-launch
       captures) measured the same allocation count both ways: 41.2k vs 41.1k
       per 20s flying, 26.2k both ways on the pad. */
    btCompoundShape *nc = new btCompoundShape(true, geomS->getNumChildShapes());
    const btTransform toBody = principal.inverse();
    for(int i = 0; i < geomS->getNumChildShapes(); i++) {
        nc->addChildShape(toBody * geomS->getChildTransform(i),
                          geomS->getChildShape(i));
    }

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
    /* The ctor diagonalized the world tensor about the pose the body was MINTED
       at (identity here), and btCollisionObject::setWorldTransform() does not
       refresh it. Without this the ship answers torque with its principal axes
       read as world axes until something calls proceedToTransform (#438). */
    hull->btBody->updateInertiaTensor();
    if(have) {
        SetVelocity(hull, vCom + glm::cross(omega, sRot * (pOrigin - oldOrigin)));
        SetAngVelocity(hull, omega);
    }
    if(wasInWorld) { AddPhysicsBody(hull); }

    refreshAeroGeom();

    checkCompoundInvariants();
    assert(checkPartInvariants() && "containment invariant broken (see [part] above)");
}

/* Fuel burn: the MASSES move, the geometry does not, so the ship does not need
   a new compound or a new rigid body. This re-solves the principal frame
   against `geomS` -- the same shapes, poses and order the live compound was
   built from -- re-bases that compound's children into the new frame, and
   re-sets the body's mass props, pose and COM velocity. It stands in for the
   rebuildCompound() this call would otherwise pay, so it must land on the same
   numbers: same solve, same child poses, same pose/velocity carry. */
bool Vehicle::refreshMassProps() {
    if(hull == nullptr || hull->btBody == nullptr) { return false; }
    btCompoundShape *nc = compoundShape();
    if(nc == nullptr || geomS == nullptr
       || geomS->getNumChildShapes() != (int)parts.size()
       || nc->getNumChildShapes() != (int)parts.size()) { return false; }

    geomMasses.resize(parts.size());
    btScalar total = 0;
    for(size_t i = 0; i < parts.size(); i++) {
        // effectiveMass, exactly as rebuildCompound counts it
        geomMasses[i] = (btScalar)parts[i]->effectiveMass();
        assert(geomMasses[i] > 0 && "a part in the compound weighs nothing");
        total += geomMasses[i];
    }
    // Same door as rebuildCompound's: burning only moves mass, it must not create
    // a total the compound cannot be solved for (#547).
    assert(std::isfinite((double)total)
           && "the ship's total mass overflows what the game can store");

    /* Frame S and the velocities are read BEFORE `principal` moves: S is
       derived from the body pose through the OLD principal. */
    glm::dvec3 sPos, vCom, omega, oldOrigin;
    glm::dmat3 sRot(1.0);
    frameS(sPos, sRot);
    vCom = GetVelocity(hull);
    omega = GetAngVelocity(hull);
    glm::dmat3 ignore;
    fromBt(principal, oldOrigin, ignore);

    btVector3 inertiaDiag(0, 0, 0);
    geomS->calculatePrincipalAxisTransform(&geomMasses[0], principal, inertiaDiag);

    const btTransform toBody = principal.inverse();
    for(int i = 0; i < nc->getNumChildShapes(); i++) {
        // shouldRecalculateLocalAabb=false: refresh the bounds once below rather
        // than once per child.
        nc->updateChildTransform(i, toBody * geomS->getChildTransform(i), false);
    }
    nc->recalculateLocalAabb();   // getAabb() reads the cached local bounds

    hull->mass = (double)total;
    hull->btBody->setMassProps(total, inertiaDiag);
    glm::dvec3 pOrigin; glm::dmat3 pBasis;
    fromBt(principal, pOrigin, pBasis);
    hull->btBody->setWorldTransform(toBt(sPos + sRot * pOrigin, sRot * pBasis));
    hull->btBody->updateInertiaTensor();
    // The COM moved WITHIN S, so its world velocity changed by omega x shift.
    SetVelocity(hull, vCom + glm::cross(omega, sRot * (pOrigin - oldOrigin)));
    // The mass path now owns most rebuilds, so its invariants run here too:
    // this is the only place that notices a stale geomS mirror or a world
    // tensor that stopped following the pose (#438).
    checkHullMirrorInvariants();
    return true;
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
        // uid first: Part/Body addresses are recycled by the allocator, so a
        // pointer alone could alias an old part; uid is process-unique.
        h = mix(h, p->uid);
        h = mix(h, (uint64_t)(uintptr_t)p->body);
        /* The shape pointer too: geomS and the live compound hold it. A part
           that re-makes its Body/hull in place (part damage eating a panel, say)
           would otherwise keep a matching fingerprint and leave the mass path
           updating children that point at freed shapes. */
        h = mix(h, (uint64_t)(uintptr_t)p->body->shape);
        for(int i = 0; i < 3; i++) { h = mix(h, std::bit_cast<uint64_t>(p->localPos[i])); }
        for(int c = 0; c < 3; c++) {
            for(int r = 0; r < 3; r++) { h = mix(h, std::bit_cast<uint64_t>(p->localRot[c][r])); }
        }
    }
    return h;
}

/* The silhouette cache's lifetime (#463). There is no hull to build any more --
   applyAeroForce projects the parts themselves and unions their polygons -- so
   all a geometry change costs here is a generation bump: every cached node
   reads as unfilled and refills the next time the flow needs it. Staging,
   docking and part damage take this path; a burn (the common rebuild) does not,
   which is what the fingerprint gate at the top is for. */
void Vehicle::refreshAeroGeom() {
    const uint64_t key = aeroGeomFingerprint(parts);
    if(aeroGeomValid && key == aeroGeomKey) { return; }
    aeroGeomKey = key;
    aeroGeomValid = true;
    aeroSil.invalidate();
    groupLiftingSystems();
}

/* #556: which panels are one wing. The law is drag.h::classifyLiftingSystems; this
   is the Part/PartDef plumbing, and it runs on a geometry rebuild (the same
   fingerprint the silhouette cache rides) rather than per substep. A part with
   neither a lift nor a control area is hull, not a surface, and never groups --
   isAeroSurface is the one predicate for that, as it is for the drag split (#455).

   The pane's area is the planform the part declares: lift_area if it lifts, else
   control_area. A part that declares both contributes its lift_area, because that
   is the planform the plate IS; the control_area is a fraction of it by convention
   (#478). Its span is the mesh's wider across-axis extent (meshSpanHalf, #457) and
   its chord the along-axis one (#559's edge), both read where the part is bolted:
   localRot[meshSpanAxis()] and localRot[2] are the part-frame axes carried into
   ship frame by the mount. */
void Vehicle::groupLiftingSystems() {
    std::vector<LiftingPane> panes;
    std::vector<Part *> owners;
    std::vector<LiftingBridge> bridges;
    for(Part *p : parts) {
        if(p->def == nullptr) { continue; }
        const PartDef *d = p->def;
        p->aeroSystem = -1;
        if(!isAeroSurface(d->lift_area, d->control_area)) {
            /* A BODY part (hull, tank, stack, engine): its width across the airframe
               is where a wing can continue through structure this model does not
               lift. The shipped jets mount their wings on the hull's sides with their
               roots 2 m apart -- one wing, carried by the fuselage -- and this is how
               the grouping learns it without a new authored number. Project the mesh
               box's two across-axes onto the lateral axis. */
            double hx, hy;
            d->meshHalfSpan(hx, hy);
            const double half = std::fabs(p->localRot[0].x) * hx
                              + std::fabs(p->localRot[1].x) * hy;
            if(half > 0.0) {
                bridges.push_back({ p->localPos.x - half, p->localPos.x + half });
            }
            continue;
        }
        const double area = (d->lift_area > 0.0) ? d->lift_area : d->control_area;
        if(!(area > 0.0)) { continue; }
        LiftingPane pane;
        pane.spanAxis = p->localRot[d->meshSpanAxis()];
        pane.chordAxis = p->localRot[2];
        pane.mount = p->localPos;
        pane.spanHalf = d->meshSpanHalf();
        pane.chordHalf = d->meshHalfLength();
        pane.area = area;
        panes.push_back(pane);
        owners.push_back(p);
    }
    /* The airframe's own axes, in ship frame: X across, Z fore/aft. A wing groups
       with another wing when their spans run along X; the vertical fins in this
       catalogue answer to neither test and stay single-panel systems. */
    const std::vector<int> ids = classifyLiftingSystems(
        panes, bridges, glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.0, 0.0, 1.0),
        aeroSystems);
    if(std::getenv("OSPAEROSYS")) {
        for(size_t i = 0; i < panes.size(); i++) {
            const LiftingPane &q = panes[i];
            printf("[liftsys] pane %zu: mount (%6.2f %6.2f %6.2f) spanAxis (%5.2f %5.2f %5.2f)"
                   " chordAxis (%5.2f %5.2f %5.2f) h=(%5.2f %5.2f) area %5.2f -> sys %d\n",
                   i, q.mount.x, q.mount.y, q.mount.z,
                   q.spanAxis.x, q.spanAxis.y, q.spanAxis.z,
                   q.chordAxis.x, q.chordAxis.y, q.chordAxis.z,
                   q.spanHalf, q.chordHalf, q.area, ids[i]);
        }
    }
    assert(ids.size() == owners.size());
    for(size_t i = 0; i < owners.size(); i++) {
        owners[i]->aeroSystem = ids[i];
        aeroSystems[(size_t)ids[i]].liftArea += owners[i]->def->lift_area;
    }
    aeroSysLiftOverQ.assign(aeroSystems.size(), 0.0);
    aeroSysCdi.assign(aeroSystems.size(), 0.0);
    if(std::getenv("OSPAEROSYS")) {
        printf("[liftsys] %s: %zu system(s) from %zu pane(s)\n", displayName().c_str(),
               aeroSystems.size(), panes.size());
        for(size_t s = 0; s < aeroSystems.size(); s++) {
            const LiftingSystem &g = aeroSystems[s];
            const double ar = (g.area > 0.0) ? g.span * g.span / g.area : 0.0;
            printf("[liftsys]   span %6.2f m  area %6.2f m2  AR %5.2f  n=%d\n",
                   g.span, g.area, ar, g.members);
        }
    }
}

/* #556: the price of the lift held in aeroLiftSamples, billed per SYSTEM.

   CL_sys = sum(|L_i| / q_i) / sum(S_i): the area-equivalent lift each member made,
   over the wing's whole planform. Each part's own q, because a rotating airframe
   gives its two tips different dynamic pressures (#483). Then

       CDi = CL_sys^2 / (pi e AR_sys),   AR_sys = span_sys^2 / S_sys

   once for the wing, and each surface pays drag_cd * q_i * S_i * CDi along the flow
   IT feels, at its own mount -- so the total is exactly q S CDi when the qs agree,
   the moment story #455 established is untouched, and a two-panel wing stops paying
   twice for one vortex sheet. A sample whose system is unknown (a part that never
   grouped: no mesh, no area) is skipped rather than billed at AR 0. */
void Vehicle::applyInducedDrag(Body *forceHull, double dragCd, glm::dvec3 &ftotal,
                               glm::dvec3 &moment, bool record) {
    if(aeroLiftSamples.empty()) { return; }
    if(aeroSysLiftOverQ.size() != aeroSystems.size()
       || aeroSysCdi.size() != aeroSystems.size()) {
        aeroSysLiftOverQ.assign(aeroSystems.size(), 0.0);
        aeroSysCdi.assign(aeroSystems.size(), 0.0);
    }
    std::fill(aeroSysLiftOverQ.begin(), aeroSysLiftOverQ.end(), 0.0);
    std::fill(aeroSysCdi.begin(), aeroSysCdi.end(), 0.0);
    for(const AeroLiftSample &s : aeroLiftSamples) {
        if(s.system < 0 || (size_t)s.system >= aeroSystems.size()) { continue; }
        if(s.q > 0.0) { aeroSysLiftOverQ[(size_t)s.system] += s.lift / s.q; }
    }
    for(size_t i = 0; i < aeroSystems.size(); i++) {
        const LiftingSystem &sys = aeroSystems[i];
        if(!(sys.area > 0.0) || !(sys.span > 0.0)) { continue; }
        aeroSysCdi[i] = inducedCd(aeroSysLiftOverQ[i] / sys.area, sys.span,
                                  sys.area, sys.oswald);
    }
    for(const AeroLiftSample &s : aeroLiftSamples) {
        if(s.system < 0 || (size_t)s.system >= aeroSystems.size()) { continue; }
        const double cdi = aeroSysCdi[(size_t)s.system];
        /* Negated test on purpose: a NaN cdi must not reach the force. */
        if(!(cdi > 0.0)) { continue; }
        const double vp = std::sqrt(glm::dot(s.vp, s.vp));
        if(!(vp > 0.0)) { continue; }
        const glm::dvec3 find = -(s.vp / vp) * (dragCd * s.q * s.area * cdi);
        ApplyForce(forceHull, s.r, find);
        ftotal += find;
        moment += glm::cross(s.r, find);
        lastInducedCdA += dragCd * s.area * cdi;
        if(record) {
            lastForceVecs.push_back({ s.r, find, kForceInduced });
        }
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
    if(glm::length(compoundCom() - pOrigin) <= kComRebuildTol
       && std::fabs(total - hull->mass) <= kMassRebuildFrac * hull->mass) {
        return;
    }
    /* A burn is what usually trips that gate, and burning changes only the
       masses: while the parts still fingerprint as the geometry the compound
       and `geomS` were built from, update the SAME body in place. Staging,
       docking and part damage change the fingerprint and pay the full rebuild;
       so does anything that leaves the mirrors inconsistent. */
    if(!(aeroGeomValid && aeroGeomFingerprint(parts) == aeroGeomKey)
       || !refreshMassProps()) {
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

    checkHullMirrorInvariants();
}

/* The two cheap halves of the compound invariants: the S-frame geometry mirror
   and the world inverse inertia. Both are O(parts) with no analytic recompute,
   so refreshMassProps() can afford them -- and must run them, because a mass
   refresh is the hot path now and checkCompoundInvariants() only runs at a
   full rebuild. */
void Vehicle::checkHullMirrorInvariants() const {
    if(hull == nullptr || hull->btBody == nullptr) { return; }

    /* `geomS` is the mirror refreshMassProps() re-solves against, so it must
       hold the same shapes at the same authored poses, in part order. It is
       only rebuilt at a full rebuild, so this is the tripwire for a part whose
       shape or mount moved while the fingerprint still said otherwise: today
       every runtime writer of localPos/localRot (setRoot, attach*, absorbShip,
       extractSubtreeAsShip) ends in rebuildCompound(), so if that pairing ever
       slips, the mass path is the first place it would bite. */
    if(geomS == nullptr || geomS->getNumChildShapes() != (int)parts.size()) {
        printf("[compound] '%s': the S-frame geometry mirror holds %d children "
               "for %zu parts\n", displayName().c_str(),
               geomS ? geomS->getNumChildShapes() : -1, parts.size());
        fflush(stdout);
        assert(false && "the S-frame geometry mirror must mirror the parts");
    } else {
        for(size_t i = 0; i < parts.size(); i++) {
            glm::dvec3 cp; glm::dmat3 cr;
            fromBt(geomS->getChildTransform((int)i), cp, cr);
            const Part *p = parts[i];
            assert(geomS->getChildShape((int)i) == p->body->shape
                   && "the S-frame mirror points at another part's shape");
            assert(glm::length(cp - p->localPos) < 1e-12
                   && "the S-frame mirror lags a part's mount");
            // Both sides are glm (fromBt inverts toBt), so the authored pose is
            // the yardstick -- no Bullet round trip in the comparison.
            for(int c = 0; c < 3; c++) {
                for(int r = 0; r < 3; r++) {
                    assert(std::fabs(cr[c][r] - p->localRot[c][r]) < 1e-12
                           && "the S-frame mirror lags a part's orientation");
                }
            }
            /* A mount must be a ROTATION, not a reflection and not a scale. The aero
               law asks a mount which way its surface pushes (drag.h::controlRigLever
               reads mount[1]) and partAero measures alpha against its columns, all of
               which assume det +1 with unit columns: a reflected mount inverts a
               control surface's rigging silently, and a mount whose quaternion drifted
               off unit length (glm's mat3_cast does not normalize) skews the angle it
               reports. sRot itself comes from Bullet and is not checked here. Every
               writer of localRot builds from quaternions, so this is a tripwire on the
               assumption rather than a fix for a known bad path. */
            const double det = glm::dot(p->localRot[0],
                                        glm::cross(p->localRot[1], p->localRot[2]));
            assert(std::fabs(det - 1.0) < 1e-9 && "a part mount must be a rotation");
        }
    }

    /* The tensor Bullet actually answers torque with has to be the local
       diagonal rotated into the body's ACTUAL pose -- setWorldTransform() does
       not refresh it, so a rebuild or a mass refresh that forgets
       updateInertiaTensor() leaves the principal axes read as world axes (#438). */
    glm::dvec3 wPos; glm::dmat3 wRot;
    fromBt(hull->btBody->getWorldTransform(), wPos, wRot);
    const btVector3 li = hull->btBody->getLocalInertia();
    if(li.getX() > 0.0 && li.getY() > 0.0 && li.getZ() > 0.0) {
        const glm::dmat3 localInv(1.0 / li.getX(), 0.0, 0.0,
                                  0.0, 1.0 / li.getY(), 0.0,
                                  0.0, 0.0, 1.0 / li.getZ());
        const glm::dmat3 wantInv = wRot * localInv * glm::transpose(wRot);
        const btMatrix3x3 &gotInv = hull->btBody->getInvInertiaTensorWorld();
        double wInvErr = 0.0;
        double wInvScale = 0.0;
        for(int c = 0; c < 3; c++) {
            for(int r = 0; r < 3; r++) {
                wInvErr = std::max(wInvErr, std::fabs(gotInv[r][c] - wantInv[r][c]));
                wInvScale = std::max(wInvScale, std::fabs(wantInv[r][c]));
            }
        }
        /* Relative to the tensor's own magnitude, and scaled by btScalar's
           precision: btcommon.h forces double, but a float build would fail a
           fixed 1e-12 for arithmetic alone. */
        const double tol = 100.0 * std::numeric_limits<btScalar>::epsilon() * wInvScale;
        if(wInvErr > tol) {
            printf("[compound] '%s': the world inverse inertia is stale (max err "
                   "%.4g, scale %.4g) -- the pose was set without refreshing it\n",
                   displayName().c_str(), wInvErr, wInvScale);
            fflush(stdout);
            assert(false && "the world inverse inertia must follow the body pose");
        }
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
    assert(p != nullptr);   // every part* the accessors take comes from `parts`
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

/* The compound is one rigid body: there is no per-part rate (see vehicle.h), which is
   also why the attitude laws ask for the hull's rate directly. */
glm::dvec3 Vehicle::hullAngVel() const { return GetAngVelocity(hull); }

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

void Vehicle::restoreSlewRequest(SlewMode m) {
    /* The one place the latched mode arrives without going through Command()'s
       toggle rule, so it is also the one place a value can come from outside the sim
       at all. An out-of-range mode passes tick.cpp's "is anything latched" test and
       then has no target in slewTargetDir(): the ship drops out of warp and cannot be
       turned. Nothing latched is the recoverable reading of a corrupt field. */
    if((int)m < SlewNone || (int)m > SlewKillRot) {
        printf("[load] '%s': autopilot mode %d is not a mode, nothing latched\n",
               displayName().c_str(), (int)m);
        m = SlewNone;
    }
    slewRequest = m;
}

void Vehicle::setRoot(Part *part) {
    part->parent   = nullptr;
    part->localPos = glm::dvec3(0.0);
    part->localRot = glm::dmat3(1.0);
    part->owner    = this;   // containment edge (part.h): parts list is the ownership list
    parts.push_back(part);
}

void Vehicle::attach(Part *part, size_t parentIdx, const glm::dvec3 &localPos, const glm::dmat3 &localRot) {
    /* Every caller derives this index from `parts` (the save loader resolves a uid
       through a map it asserts first; attachMode/attachSurface check below, before
       their own read). An out-of-range index here silently adopts the wrong parent. */
    assert(parentIdx < parts.size());
    part->parent   = parts[parentIdx];
    part->localPos = localPos;
    part->localRot = localRot;
    part->owner    = this;   // containment edge (part.h): parts list is the ownership list
    parts.push_back(part);
}

void Vehicle::attachMode(Part *part, size_t parentIdx, AttachMode mode,
                         double angleDeg, double offset) {
    assert(parentIdx < parts.size());   // read before attach() gets there (see attach)
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
    assert(parentIdx < parts.size());   // read before attach() gets there (see attach)
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
       HIGHEST, so stage 1 fires last. Both ends seed from parts[0] and the scan covers
       every part: seeding the high end at 1 (#519) skipped the ROOT part's own label --
       the one part every ship has -- so a ship whose root carries the top label never
       had that label counted. The part staged itself live from the first frame
       (producedThrust only skips `stage < as`, and the counter only falls), its decoupler and
       chute never fired (both test `stage == stage`), the HUD's `Stage: x / y` under-read
       the part popup's own number, and minStage_ could end up above activeStage_. No
       clamp to 1 is needed: every writer either BOUNDS the number (the ship loader,
       the save's refuseSavedStage, the VAB's stage box) or leaves the default 1
       (Part::stage's default, and the EVA suit mints copy the part they came from).
       Cited by symbol, not line: this function moves and the numbers drift. Where the
       shape does NOT come from is the stage box -- it is not drawn for the root part at
       all (gameui's selected-part panel refuses index 0), so a root's own label arrives
       from a ship FILE that authored one, which the loader accepts, or from a save. */
    totalStages_ = parts[0]->stage;
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

Vehicle::FuelDraw Vehicle::consumeResourceMass(enum ResourceType type, double amt, Part *engine) {
    const std::vector<std::vector<int> > &layers = fuelDrainLayers(engine);
    if(layers.empty()) { return {false, 0.0}; }
    // Drain-or-nothing: no partial burn if the layers cannot cover the request.
    if(availableResourceMass(type, engine) < amt) { return {false, 0.0}; }
    /* Drain layer by layer (furthest first), pro-rata across the layer's tanks.
       Each layer is sized from what is still OWED (amt - drawn), not from a running
       budget advanced by what was asked: the pro-rata shares can clamp at a tank's own
       level and leave the layer a hair short, and that remainder belongs to the next
       layer rather than being dropped on the floor (#531). */
    double drawn = 0.0;
    for(size_t li = 0; li < layers.size() && drawn < amt; li++) {
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
        const double owed = amt - drawn;
        const double take = owed < layerTotal ? owed : layerTotal;
        for(size_t gi = 0; gi < layers[li].size(); gi++) {
            const int grp = layers[li][gi];
            for(Part *p : parts) {
                if(!p->isTank() || p->fuelGroup != grp) { continue; }
                const double have = p->resources.current[(int)type];
                if(have <= 0.0) { continue; }
                double share = take * have / layerTotal;
                if(share > have) { share = have; }
                p->resources.current[(int)type] = have - share;
                drawn += share;
                /* Fuel leaves resources.current; the body mass is the DRY structure. */
            }
        }
    }
    return {true, drawn};
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

double Vehicle::burnShare(double available, double desired) {
    /* Fail CLOSED on a non-number availability: the (x < 1.0) shape below would answer
       "fully covered" to a NaN share, and a NaN in resources.current is a bug elsewhere
       that this should not launder into a burn. */
    if(!(desired > 0.0)) { return 0.0; }
    if(!(available > 0.0)) { return 0.0; }
    const double s = available / desired;
    return (s < 1.0) ? s : 1.0;
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

double Vehicle::wheelDemandWatts() const {
    /* What the wheels COST the hull: the rating sum behind the enable mask, commanded
       or not. powerTick's gate asks this and never wheelDrawWatts(), because the billed
       figure is that gate's own verdict -- asking it is what let the wheels re-arm the
       substep after cutting them (#642). The stick stays out of it for the same reason
       one layer down: a hull whose generator cannot pay its drums is short of power
       whether or not anyone is steering, and a verdict that moves with the stick makes
       the Power row promise authority that the next keypress refuses.
       wheels_enabled is in here because a hull that has switched its drums off has no
       unaffordable demand to brown out over (#425 D-2). isWheel() asks the torque rating,
       not the part type, so a command pod's built-in drums are billed exactly like a
       wheel block's: the bill follows the authority the hull spends rather than the
       cosmetic choice of part (#596). What the catalog charges per N m of rating -- one
       constant, WHEEL_DRAW_WATTS_PER_NM in utils/gen_parts.py -- and why it charges per
       housing rather than per drum (#593) is said there, not here. */
    if(not wheels_enabled) { return 0.0; }
    double w = 0.0;
    for(Part *p : parts) {
        /* powerActiveW() is the gate AND the figure in one call, and it is the same
           accessor buildPower() sums over an assembly on the VAB pad (#644), so the
           build screen prices the drums off this number rather than a second sum of
           its own. */
        w += p->powerActiveW();
    }
    return w;
}

double Vehicle::wheelDrawWatts() const {
    /* What the wheels are BILLING this substep: what they DELIVERED last time the mixer
       ran (#648), and only while a command is live and the EC gate is open. The power
       block on the Game Debug overlay and --power-log's active= column quote this, so
       the readout cannot disagree with the bill (#575) -- it used to re-derive the sum,
       and dropped the power gate while doing it.
       The figure is a substep behind by construction: actuateAttitude fills it and the
       tick bills power before control forces so the gate is current (#647), so powerTick
       pays what the previous substep delivered. At the sim's own step that is 0.1 s of
       sim -- a fifth of one row of a 0.5 s power log. */
    return actuatorsEngaged() ? wheelSpendWatts_ : 0.0;
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
    // No EC system -> ungated. Otherwise the wheels' bill has to be payable.
    const bool hasEC = (ecCapacity > 0.0) || (totalGen > 0.0) || (constantDraw > 0.0);
    /* The gate pays the WHEELS, not only life support (#642). It used to read
       (gen > life support) || (charge > 0), so a capsule + rtg hull (298 W over
       100 W) steered forever once its pool was empty: drainEC() has nothing to take
       from a dry pool and the gate never noticed -- #418's other half, and the reason
       #596's pod bill changed nothing on exactly the hulls with power to spare.
       `cost` is what the drums COST, commanded or not (wheelDemandWatts): the billed
       figure is this gate's own verdict, so a gate that reads it cuts the wheels and
       then finds nothing to pay for, which re-arms them the very next substep.
       Two thresholds, not one, so the boundary is a decision rather than a shudder:
       the wheels drop once the pool cannot cover kWheelsCutS of commanding, and come
       back once it banks kWheelsRecoverS of commanding -- or is half full, the backstop
       for a store too small to ever bank a whole maneuver. Both are DURATIONS rather
       than the caller's `h` because powerTick also runs for railed hulls (#647), where
       `h` is a whole warp hop: a step-scaled threshold would be a duty cycle set by how
       finely the sim happens to step, which is the thing this latch exists to prevent.
       The latch only speaks for hulls that have an EC system at all, so it stays false
       on the ungated hulls that a later readout (#649) would otherwise have to
       explain. */
    const double cost = wheelDemandWatts();
    const bool genCovers = (totalGen > constantDraw + cost);
    if(hasEC) {
        if(wheels_brown_out_) {
            if(ecCharge * 3600.0 >= cost * kWheelsRecoverS
               || ecCharge >= 0.5 * ecCapacity) { wheels_brown_out_ = false; }
        } else if(cost > 0.0 && not genCovers
                  && ecCharge * 3600.0 <= cost * kWheelsCutS) {
            wheels_brown_out_ = true;   // not even a short burst of commanding is in the pool
        }
    }
    powered_ = hasEC ? (genCovers || (ecCharge > 0.0 && not wheels_brown_out_)) : true;
    /* The wheels draw only while actually working: gated by the verdict, by a live
       command, and by there being a control pass to work them -- a hull coasting on
       rails has none, so it pays for breathing and generation but not for drums it is
       not turning (#647). The power block on the Game Debug overlay and --power-log's
       active= column quote the same predicate (#575, and ui/601 reads it off the
       screen).
       What it pays is the DELIVERED figure, not `cost`: the last actuated substep's
       scaled package sum, which is what wheelDrawWatts() reads (#648). `cost` stays on
       the gate above and nowhere near the balance -- paying the rating is how a hull
       that asked 4 N m of a 4500 N m drum ate the pool at 2250 W. */
    const double activeDraw = wheelDrawWatts();
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
    /* active is the term charge already moves by (powerTick's netWh), so a test can
       pin the identity instead of inferring a slope from columns that omit it (#643).
       It is wheelDrawWatts(), so it reads 0 unless the actuators are engaged: what
       this hull is BILLING, not what its drums are rated at (#599's same split). */
    const double active = wheelDrawWatts();
    printf("[powerlog] t=%.1fs ship=\"%s\" powered=%d gen=%.1fW "
           "const_draw=%.1fW charge=%.1fWh capacity=%.1fWh active=%.1fW\n",
           time, displayName().c_str(), (int)powered_, gen, constDraw, charge,
           capacity, active);
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

double Vehicle::producedThrust() {
    /* Actual thrust at the current throttle and air state (jets via jetThrust, drag.h). */
    const int as = activeStage();
    const double v_air = glm::length(airRelativeVel(get_center_of_mass()));
    const double rho = airDensityAtCom();
    const double ox_share = (m_parent != nullptr)
        ? m_parent->surface.atmosphere.oxidizer_share : 0.0;
    double t = 0;
    for(Part *p : parts) {
        if(!p->isThruster() || p->stage < as) { continue; }
        if(p->isJet()) {
            t += jetThrust(v_air, rho, ox_share, p->def->jet_fan_thrust,
                           p->jetFuelRate(), p->def->exhaust_velocity,
                           p->def->jet_intake_area);
        } else {
            t += p->thrust();
        }
    }
    return t * thruster_util * exhaust_scale;
}

double Vehicle::getTWR() {
    return producedThrust() / (getMass() * m_parent->g);
}

double Vehicle::getFullThrustTWR() {
    return availableThrust() / (getMass() * m_parent->g);
}

double Vehicle::getMaxTWR() {
    // ALL burnable propellant (rocket and jet): max TWR is at the lightest mass.
    bool burn[(int)ResourceType::Num];
    enginePropellantMask(burn, /*includeJets=*/true);
    const double remaining_fuel = fuelMassMasked(burn); /* kg */
    return availableThrust() / ((getMass() - remaining_fuel) * m_parent->g);
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
    // geomS only borrows the part shapes; deleting it frees no part geometry.
    delete geomS; geomS = nullptr;
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
       couple around the ring. Where that direction comes from is no longer
       this mixer's business: actuateAttitude() split ONE demanded torque
       between the drums and the gimbals earlier in this same substep, so the
       stick and the autopilot come through the same door (#425 step D), and the
       clamp below bounds the TVC half at each engine's own stop -- which is what
       #420 (SAS budgeting TVC as if the stick were not sitting on it) reduces to.
       The drums' half is bounded elsewhere, per line, by actuators.h actuatorSolve.
       No EC/wheel gate: a
       mechanical gimbal works when the power is dead. No stick recenters, so a
       coasting ship settles.
       Sign per engine: a deflection of shared lateral L levers an engine at
       height h (along the stack axis) by torque ~ -h * (shipZ x L), so engines
       on the far side of the COM (reversed engines in a docked pair, upper
       stages above COM mid-burn) must deflect OPPOSITE for their torques
       to add instead of fight. Ring symmetry is untouched: same-height
       engines share the sign. */
    const Part *tvcRef = attitudeRef();
    const glm::dvec3 shipZ = (tvcRef != nullptr) ? partAxis(tvcRef, 2) : glm::dvec3(0.0);
    for(Part *p : parts) {
        if(!p->isThruster()) { continue; }
        if(p->def->gimbal_max > 0.0) {
            glm::dvec3 lat(0.0);
            if(p->armedThrust != 0.0f) {
                lat = tvcLat_;
                const double l = glm::length(lat);
                if(l > p->def->gimbal_max) { lat *= p->def->gimbal_max / l; }
            }
            if(tvcRef != nullptr) {
                const double height = glm::dot(partPos(p) - com, shipZ);
                // Same near-COM dead zone attitudeAuthority() budgets with: an
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
        const glm::dvec3 arm = partPos(p) - com;
        const glm::dvec3 ft = partThrustDir(p) * (double)p->armedThrust;
        ApplyForce(hull, arm, ft);
        ftotal += ft;
        if(p->def->gimbal_max > 0.0) {
            /* #599: the N m this nozzle actually turned the hull with, off the same
               force Bullet just got. An engine that is not armed, has not finished
               swinging on its gimbal RATE, or is levershort of the budget shows up
               here short, which is the point: the split's share would have read the
               rating instead. Ungimballed engines are left out -- their thrust is not
               steering however much it torques. See AttitudeAuthority's neighbour
               AttitudeDelivery in vehicle.h. */
            attDel_.tvc += glm::cross(arm, ft);
        }
        if(recordForceVecs) {
            // Same arm Bullet got; the drawer anchors it at the CURRENT COM
            // (see the ForceVec doc in vehicle.h).
            lastForceVecs.push_back({ arm, ft, kForceThrust });
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

// The authored atmosphere as drag.h sees it (declared in vehicle.h -- including the
// borrow contract on the layer pointer and what g does and does not move). The body's
// g rides along because the thermodynamic reads (T/p/a, #552 A2) are derived from H
// AND g; on an exponential body rho and top() are not, on a layer body the weighed
// column is (#552 A4).
// The layer table (#552 A4) rides as a POINTER into a's own vector: this copy runs
// per hull per substep, and the AtmosphereParams it borrows from outlives every one
// of them -- the loader resolved the levels once, when the system file loaded.
DragAtmosphere dragAtm(const AtmosphereParams &a, double g) {
    DragAtmosphere da { a.sea_level_density, a.scale_height, a.top() };
    da.gravity = g;
    da.molar_mass = a.molar_mass;
    da.gamma = a.gamma;
    da.layers = a.layers.empty() ? nullptr : a.layers.data();
    da.layer_count = (int)a.layers.size();
    return da;
}

/* One profile read at a hull's COM, for callers that need more than one number out of
   the same air (#487's budget wants density AND the sound speed; #543 A5 is why the
   second question exists). Fills `alt` -- height above the datum, the same convention
   applyAeroForce uses per surface -- and `atm`, the body's own profile, so a layer
   table and a single scale height read identically here. */
static bool comAirProfile(const TerrainBody *body, const glm::dvec3 &com,
                          DragAtmosphere &atm, double &alt) {
    if(body == nullptr) { return false; }
    const AtmosphereParams &surfaceAtm = body->surface.atmosphere;
    if(!surfaceAtm.physical()) {
        return false;   // no physical atmosphere (a limb rim alone does not count)
    }
    const double r = glm::length(com);
    if(r <= 0.0) { return false; }
    // Altitude above SEA LEVEL (the fixed reference radius), like applyAeroForce: the
    // atmosphere is a spherically-symmetric shell.
    alt = r - body->seaLevelRadius();
    atm = dragAtm(surfaceAtm, body->g);
    return true;
}

double Vehicle::airDensityAtCom() const {
    DragAtmosphere atm; double alt = 0.0;
    if(!comAirProfile(m_parent, comPos(), atm, alt)) { return 0.0; }
    return airDensity(atm, alt);
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
    lastLiftVert = 0.0;
    lastUpVert = 0.0;
    lastAeroTorque = glm::dvec3(0.0);
    lastDragAlt = 0.0;
    lastDragRho = 0.0;
    lastAirT = 0.0;
    lastAirP = 0.0;
    lastAirA = 0.0;
    lastAirV = 0.0;
    lastDragAlpha = 0.0;
    lastDragBeta = 0.0;
    lastDragAlphaValid = false;
    lastDragArea = 0.0;
    lastDragCd = 0.0;
    lastCpLever = 0.0;
    lastFormCpLever = 0.0;
    lastLiftSlope = 0.0;
    lastStaticMargin = 0.0;
    lastStaticValid = false;
    lastChuteCdA = 0.0;
    lastInducedCdA = 0.0;
    lastSurfCdA = 0.0;
    aeroLiftSamples.clear();  // #556: last substep's lift must not be re-billed
    lastControlDeflections.clear();  // a no-air substep reports no steering
    // Fins swing back to neutral whenever this substep produces no steering
    // (vacuum, no-atmosphere body, above the atmosphere top, --drag-cd 0). Same sweep
    // invalidates #485's per-surface readout: a part this pass does not reach
    // quotes nothing rather than quoting last substep's angle.
    for(Part *p : parts) {
        p->aeroValid = false;
        if(p->def != nullptr && p->def->control_area > 0.0) { p->controlDefl = 0.0; }
    }

    // --drag-cd 0 = no aero at all (the master off switch).
    if(drag_cd <= 0.0) { return lastAeroForce; }

    // Only a body with a PHYSICAL atmosphere (a density model) produces aero.
    if(m_parent == nullptr) { return lastAeroForce; }
    const AtmosphereParams &atm = m_parent->surface.atmosphere;
    if(!atm.physical()) {
        return lastAeroForce;
    }

    const glm::dvec3 com = comPos();
    const double r = glm::length(com);
    if(r <= 0.0) { return lastAeroForce; }
    const double alt = r - m_parent->seaLevelRadius();
    lastDragAlt = alt;
    /* No floor at the datum. The exponential's base IS the datum (drag.h), so a
       hull below it -- skimming the sea, or a valley under a sea_level that sits
       above this body's relief -- is in the densest air the model has, not in
       vacuum (#245). The rho floor a few lines down is the one place THIS PASS
       decides "no air"; inAtmosphere() (the rails gate), the biome rule and the
       science bands still decide it by altitude against atm.top() alone, and that
       split is deliberate (#309, #310). */

    const DragAtmosphere da = dragAtm(atm, m_parent->g);
    const double rho = airDensity(da, alt);
    lastDragRho = rho;
    /* The air's own numbers ride the same seam (#552 A2/A3) so the log and the
       atlas cannot disagree about what this body's profile means. Set BEFORE
       the density floor: thin air is still air -- it just bills nothing -- and a
       readout that vanished at kRhoFloor would be a cliff a player cannot see.
       All three take `alt`: on an exponential body they are constant with it (that
       is what one scale height means), and on a LAYERED body they are not -- Earth's
       sound speed is 340 m/s at the datum and 295 m/s at 11 km, and the Mach number
       this feeds is the one the flight model divides by (#543). */
    lastAirT = airTemperature(da, alt);
    lastAirP = airPressure(da, alt);
    lastAirA = airSoundSpeed(da, alt);
    /* GetVel() minus the air's co-rotation when outside the body's rot frame (#97).
       Read before the density floor, not after it: lastAirV is what the Mach readout
       divides, and a hull at 120 km moving 800 m/s in air too thin to bill still has a
       Mach number -- zeroing it here would print 0.000 as a measurement. */
    const glm::dvec3 vrel = airRelativeVel(com);
    const double v2 = glm::length2(vrel);
    lastAirV = std::sqrt(v2);  // at the COM; a surface adds its own omega x r
    // Above the atmosphere top rho is 0; below kRhoFloor the force is unmeasurable.
    if(rho < kRhoFloor) { return lastAeroForce; }
    /* rho > 0 means the loader's air checks already passed (molar_mass, gamma and
       g > 0, system.cpp), so the sound speed is real. The per-surface Mach divides
       by it, so say the invariant here rather than defending the division. */
    assert(lastAirA > 0.0);
    const double invAirA = 1.0 / lastAirA;  // loop-invariant over the surface loops
    if(v2 <= 0.0) { return lastAeroForce; }  // at rest in air

    // #556: the lifting systems this pass bills against. The call is fingerprint-
    // gated and normally a no-op -- rebuildCompound has already refreshed it for any
    // geometry change -- but the consumer asking here keeps the pair honest, so an
    // airframe can never fly on systems built for a previous set of parts.
    refreshAeroGeom();

    // Frame S read once: constant within a substep.
    glm::dvec3 sPos; glm::dmat3 sRot;
    frameS(sPos, sRot);
    const auto partPosS = [&](const Part *p) {
        return sPos + sRot * p->localPos;
    };
    /* The body's spin, relative to frame S -- GetAngVelocity is already S-frame
       and frame-relative (see the frame-change law, which subtracts the new
       frame's omega), so vrel + w x r is the air-relative velocity of a point at
       offset r. Same convention the chute harness uses below.
       The exact sweep would be (w - THE AIR'S own rotation, expressed in S) x r,
       because the atmosphere co-rotates with the body. There is nothing to drop in
       the case that matters: aero runs inside the body's ROTATING frame for the
       whole atmosphere shell (soiTarget puts the ship in rot_frame), where w is
       already measured against the air and the correction is identically zero. It
       is only non-zero if aero runs in the non-rotating SOI frame, and there
       Kerbin's 2.9e-4 rad/s is at most 1.5e-3 m/s of sweep at r = 5 m. Do NOT
       subtract the body's omega from w -- it is not in there. */
    const glm::dvec3 w = GetAngVelocity(hull);
    /* A non-finite rate must not reach the law: it is silent if it does. Every
       surface would read a NaN flow, partAero calls a NaN flow VALID (it only
       tests length <= 0), and the airframe keeps simulating with NaN forces.
       Nothing normal produces one, and nothing validates it either: save.cpp has
       no isfinite anywhere, while a save carrying NaN cannot even be read (the
       JSON parser refuses the literal). The realistic route is arithmetic that has
       already diverged -- the #340 high-warp regime -- and the assert is there so
       it announces itself here instead of a NaN ship drifting into a frame change. */
    assert(std::isfinite(w.x) && std::isfinite(w.y) && std::isfinite(w.z));

    /* One flow frame for every part (one rigid body). This is THE SHIP'S FRAME,
       asked the way every other consumer asks it (#571): attitudeRef()'s axes.
       partRot(p) is exactly sRot * p->localRot, and sRot is already in hand, so
       asking costs nothing that the old hand-rolled rootPart() version did not.
       Ship-local axes: nose +Z, right +X, up +Y. */
    const Part *ref = attitudeRef();
    const glm::dmat3 shipRot = (ref != nullptr) ? sRot * ref->localRot
                                                : glm::dmat3(1.0);
    const glm::dvec3 nose  = shipRot[2];
    const glm::dvec3 right = shipRot[0];
    const glm::dvec3 up    = shipRot[1];
    /* The same axes in the reference part's own local frame. The readout's build-fact questions (#485)
       are asked here, once per pass, in these vectors rather than in the world ones, so
       their answers are fixed at load. In world vectors the arithmetic is mathematically
       the same but recomputed from sRot every substep, and a mount near the 45-deg tie
       changes its mind with the ship's attitude -- see drag.h::surfaceReadoutFrame. */
    const glm::dvec3 shipLocalUp    = (ref != nullptr) ? ref->localRot[1] : glm::dvec3(0.0, 1.0, 0.0);
    const glm::dvec3 shipLocalRight = (ref != nullptr) ? ref->localRot[0] : glm::dvec3(1.0, 0.0, 0.0);
    lastUpVert = glm::dot(up, com / r);  // upright? (see lastUpVert)
    const AeroFrame fr = aeroFrame(vrel, right, up, nose);
    const double alpha   = fr.valid ? fr.alpha   : 0.0;
    lastDragAlpha = alpha;
    lastDragBeta  = fr.valid ? fr.beta : 0.0;
    /* A flow slower than kAoAFlowFloor has no usable direction: a parked ship's
       jitter against the co-rotating air would read as a full side-slip.
       isfinite because AeroFrame::valid cannot refuse a NaN axis (length() <= 0
       is false for NaN), and a NaN printed here silently drops the whole
       [surfinfo] row in e2e rather than failing loudly. */
    lastDragAlphaValid = fr.valid && std::isfinite(fr.v)
                      && std::isfinite(fr.alpha) && std::isfinite(fr.beta)
                      && fr.v >= kAoAFlowFloor;
    const glm::dvec3 vhat    = vrel / std::sqrt(v2);
    // The flow in S: area and dots are rotation-invariant, so rotate the flow, not the verts.
    const glm::dvec3 vhatS   = glm::transpose(sRot) * vhat;
    /* There is no ship-wide dynamic pressure any more. Every surface law now
       carries its own, 0.5 * rho * |vrel + w x r|^2, because the flow it answers
       to is its own (#483). The hull's FORM drag stays a COM-frame quantity on
       purpose: its area is one union projected on one flow direction, which is a
       shape property of the whole body rather than of a surface, and per-surface
       flows would break the shared projection plane the union needs (#463). */

    glm::dvec3 ftotal(0.0);
    glm::dvec3 lift_total(0.0);
    glm::dvec3 moment(0.0);
    /* #455: where the airframe's FORM drag pushes, counted over everything that
       pays it -- the hull union below and each surface's own planform bill. Weighted
       by CdA, because that is what the force is made of. lastCpLever keeps the hull's
       own answer; lastFormCpLever is the one that says whether the tail is holding the
       airframe straight. */
    double formCdA = 0.0;
    double formCdArm = 0.0;

    // Node fill (#463): the union at one flow direction, projected through that
    // direction's own plane. Separate scratch because the CURRENT direction's
    // polygons are the exact fallback and have to survive the fill.
    auto unionAtS = [&](const glm::dvec3 &dirS) {
        const FlowPlane pl = flowPlane(dirS);
        aeroSilNode.clear();
        aeroSilNode.reserve(parts.size());
        for(Part *p : parts) {
            if(p->body == nullptr || p->body->hullVerts.empty()) { continue; }
            /* #455: a lifting or steering part is not body. The chart's cells are
               the union this law charges, so the exclusion has to apply here too --
               a node filled WITH the wings and queried WITHOUT them would answer the
               cache with an area the force never pays. */
            if(p->def != nullptr
               && isAeroSurface(p->def->lift_area, p->def->control_area)) { continue; }
            aeroSilNode.emplace_back();
            projectPartHull(p->body->hullVerts, p->localRot, p->localPos, pl,
                            aeroSilNode.back());
        }
        return unionProjectedArea(aeroSilNode);
    };
    /* How many silhouette nodes one substep may compute. A node is a whole
       project-and-union pass -- 0.009 ms on a 13-part hull, but the union's
       overlap prune is pair-wise, so it grows quadratically (0.58 ms at 200
       parts) -- so filling all four corners of a fresh cell in one substep would
       spike the frame the first time a ship bows into a new corner of its own
       envelope; one per substep spreads it, and a 5 deg cell is still fully
       filled within four substeps of arriving in it. Measured on the shipped jet
       (13 parts, 336 hull verts): 6 s of level flight fills 9 nodes and hits
       about 99% of its queries; a deliberate 14 s tumble that sweeps the chart
       fills 88 and still answers about 10% of its queries with the exact union. */
    static constexpr int kSilFillNodes = 1;

    // DRAG: AREA = the UNION of the parts' silhouettes facing the flow;
    // COEFFICIENT = the parts' 3-anchor cd (partCd) area-weighted. Applied at the
    // center of pressure, and answering to the flow THERE, so a banked ship
    // weathervanes into the flow AND the same term damps the swing (#486).
    {
        /* One flow plane for the whole ship (#463): the polygons can only be
           unioned if they share a 2-D basis, so the PLANE rotates into each part
           rather than each part picking its own basis. Their areas drive the
           centre of pressure and the cd mean, so this projects once per part per
           substep, as before, and keeps the polygons for the union. */
        const FlowPlane plS = flowPlane(vhatS);
        aeroSilParts.clear();
        aeroSilParts.reserve(parts.size());
        glm::dvec3 cp(0.0);      // center of pressure (area-weighted centroid)
        double cpArea = 0.0;
        glm::dvec3 cpOffsetSum(0.0); // area-weighted cp MOVE from #512, as a vector (audit)
        double cpOffsetMagSum = 0.0; // the same offsets without cancelling (audit)
        double cdNum = 0.0;      // sum of (partArea x part cd) for the cd mean
        /* OSP_SHIP_DUMP: one design-audit dump of the whole ship as this pass sees
           it (per-part arm, projected area, cd as it faces the flow), plus the cp
           this pass computed. Throwaway instrumentation like AUDIO_DEBUG/PRECDBG: ONE
           shot per process, for whichever Vehicle reaches the drag pass first, at
           whatever attitude that happens to be -- not a per-ship readout. Columns are
           mixed accountings on purpose and labelled: a part's mass is effectiveMass()
           while the header is the compound body's, and SUM projA is the parts' areas
           ADDED while the force below pays on unionA (their union), which on an
           overlapping ship can be half of it. */
        /* OSP_SHIP_DUMP_BETA moves the dump off the first substep (which is usually
           a straight-and-level attitude where every hull projects symmetrically and
           the cp question is invisible) to the first pass at AT LEAST that much
           sideslip. It is a floor, not a target: a ship that never holds a steady
           sideslip fires wherever it happens to cross it, which on the shipped beds
           is a post-stall tumble (beta near 90). Read the alpha/beta header line
           before trusting any column. */
        static int aeroDump = getenv("OSP_SHIP_DUMP") != nullptr ? 1 : 0;   // 1 now, 2 pending, 0 done
        static const double dumpBeta = getenv("OSP_SHIP_DUMP_BETA")
            ? atof(getenv("OSP_SHIP_DUMP_BETA")) : 0.0;   // static: not a getenv+atof per substep
        // M_PI is a POSIX/MSVC extension, not standard C++; the rest of the tree
        // uses std::numbers::pi, and so does this. constexpr: this scope runs every
        // substep whether or not the dump is on, and 180 / M_PI used to fold.
        constexpr double rad2deg = 180.0 / std::numbers::pi;
        if(aeroDump == 2 && std::fabs(fr.beta * rad2deg) >= dumpBeta) { aeroDump = 1; }
        else if(aeroDump == 1 && dumpBeta > 0.0
                && std::fabs(fr.beta * rad2deg) < dumpBeta) { aeroDump = 2; }
        if(aeroDump == 1) {
            printf("[ship] compound mass %.1f  vrel %.1f m/s  alpha %.1f  beta %.1f  alt %.0f\n",
                   hull->mass, glm::length(vrel), alpha * rad2deg,
                   fr.beta * rad2deg, alt);
        }
        for(Part *p : parts) {
            if(p->body == nullptr || p->body->hullVerts.empty()) { continue; }
            /* #455: surfaces are billed below, at their own place, on their own
               planform. Excluding them here is what makes A_ship, lastDragCd and cp
               BODY properties: the jet used to print A=4.32 with Cd=0.77, where the
               0.77 was dragged down from ~0.9 by two wings whose 0.4 m2 of edge-on
               mesh sat in the same average as a 3.1 m2 fuel tank. The area the two
               conventions share is a coincidence of the same word. */
            if(p->def != nullptr
               && isAeroSurface(p->def->lift_area, p->def->control_area)) { continue; }
            // hullVerts are part-local: projectPartHull carries them through the
            // mount into the SHARED plane, so a wing 4 m off the centreline lands
            // 4 m off the centreline in the union. (Its own area is translation-
            // invariant, which is why the mount position used to be ignorable.)
            aeroSilParts.emplace_back();
            std::vector<glm::dvec2> &poly = aeroSilParts.back();
            projectPartHull(p->body->hullVerts, p->localRot, p->localPos, plS, poly);
            const double a = polygonArea2(poly);
            /* #512: the arm is where the AREA is, not where the part is bolted on.
               cpOff2 is the part's projected silhouette centroid minus the same mount
               projected into the SAME ship-local flow plane -- a ship-local free
               vector, hence the sRot that joins it to the S-frame part position. The
               coordinate the projection cannot see (along the flow) stays with the
               mount; drag.h sizes what that costs. */
            const glm::dvec2 mount2{glm::dot(p->localPos, plS.u),
                                    glm::dot(p->localPos, plS.w)};
            const glm::dvec2 cpOff2 = silhouetteCpOffset2(poly, mount2);
            if(aeroDump == 1) {
                cpOffsetSum += a * (cpOff2.x * plS.u + cpOff2.y * plS.w);
                cpOffsetMagSum += a * std::sqrt(cpOff2.x * cpOff2.x + cpOff2.y * cpOff2.y);
            }
            cp += a * silhouetteCpPointS(sRot, partPosS(p), cpOff2, plS);
            cpArea += a;
            // cd AS IT FACES THE FLOW (partCd): localRot[2] is the part's nose in S
            // and sRot preserves dots.
            double cd = 0.0;
            if(p->def != nullptr) {
                const double c = glm::dot(p->localRot[2], vhatS);
                cd = partCd(p->def->drag_forward,
                            p->def->drag_side,
                            p->def->drag_backward, c);
                if(cd > 0.0) { cdNum += a * cd; }
            }
            if(aeroDump == 1) {   // throwaway design audit: arms in SHIP axes, from the COM
                const glm::dvec3 r = partPosS(p) - com;
                /* #512: cpOff is how far this part's AREA sits from the bolt that
                   holds it -- the cp's arm per part. Zero means the hull is centred
                   on its mount and the mount says something; a third of a metre or
                   more means the mount is not where the drag pushes. */
                const glm::dvec3 offS = silhouetteCpPointS(sRot, partPosS(p), cpOff2, plS)
                                       - partPosS(p);
                printf("[ship] %-16s m %7.1f fwd %6.2f right %6.2f up %6.2f"
                       " projA %5.2f cd %.2f liftA %.2f ctrlA %.2f noseDot %5.2f"
                       " cpOff %5.2f (f %6.2f r %6.2f u %6.2f)\n",
                       p->def ? p->def->name.c_str() : "?", p->effectiveMass(),
                       glm::dot(r, nose), glm::dot(r, right), glm::dot(r, up), a, cd,
                       p->def ? p->def->lift_area : 0.0, p->def ? p->def->control_area : 0.0,
                       p->def ? glm::dot(p->localRot[2], vhatS) : 0.0,
                       glm::length(offS),
                       glm::dot(offS, nose), glm::dot(offS, right), glm::dot(offS, up));
            }
        }
        /* The ship's silhouette: the union of those polygons, cached per flow
           DIRECTION (speed and density multiply the force, they do not change
           its shape). A miss falls back to the exact union of the polygons this
           substep already projected, so a MISS cannot be wrong -- a HIT is only
           as right as the key that found it, and a mis-keyed cell that was
           already filled is invisible from here (#504: mutating the fold in
           drag.h leaves every flight case green; only tests/test_drag.cpp dies).
           The fallback is not free, either -- a filling substep pays that exact
           pass plus up to kSilFillNodes node passes, and the budget only decides
           how quickly the cache stops being needed.
           A substep with no meaningful flow direction does not consult the chart
           at all -- the same test the AoA readout uses (lastDragAlphaValid).
           aeroFrame::valid only refuses v == 0, so a parked ship's jitter passes
           it, and warming the table with those angles spends fills on a direction
           this ship is not flying; the log then prints a concrete area for a row
           whose angle it has just dashed.
           */
        double A_ship = 0.0;
        if(lastDragAlphaValid) {
            if(!aeroSil.lookup(alpha, fr.beta, A_ship)) {
                aeroSil.fillAround(alpha, fr.beta, kSilFillNodes,
                                   [&](double na, double nb) {
                                       return unionAtS(glm::transpose(sRot)
                                           * flowDirection(na, nb, right, up, nose));
                                   });
                A_ship = unionProjectedArea(aeroSilParts);
            }
        }
        lastDragArea = A_ship;
        /* The coefficient and the centre of pressure stay weighted by the SUM of
           the parts' areas, while the force uses their UNION. That is not a
           contradiction: the union answers "how much air does this shape stop",
           the mean answers "what is that air worth per m^2 of it", and the CP
           answers "where does it push" -- none of which is the other. A ship whose
           parts overlap in the view gets no extra drag, but its cd still leans
           toward whichever part presents the most material. */
        lastDragCd = (cpArea > 0.0) ? (cdNum / cpArea) : 0.0;
        if(cpArea > 0.0) { cp /= cpArea; }
        else { cp = com; }  // degenerate: no per-part area -> act through the COM
        lastCpLever = glm::dot(cp - com, nose);   // + = cp ahead: AoA push diverges
        /* The hull's share of the airframe's form-drag cp (#455), weighted by the
           CdA this substep actually charges it -- the same product the force below
           is built from, not the sum of the parts' areas. */
        {
            const double hullCdA = drag_cd * lastDragCd * A_ship;
            formCdA += hullCdA;
            formCdArm += hullCdA * lastCpLever;
        }
        if(aeroDump == 1) {
            aeroDump = 0;
            const glm::dvec3 rc = cp - com;
            /* The ship's cp MOVE from this fix, as a vector. Parts whose silhouettes
               lean opposite ways cancel: on the glider at a broadside tumble the
               capsule's area moves 0.23 m aft and the engine's 0.18 m forward, which
               is 0.06 m of average per-part offset and 0.004 m of actual cp. Quote the
               first to see which parts are wrong, the second to see what the ship
               feels. */
            const glm::dvec3 cpMove = cpArea > 0.0 ? sRot * (cpOffsetSum / cpArea)
                                                   : glm::dvec3(0.0);
            printf("[ship] cp moved by this fix: %.3f m (f %6.3f r %6.3f u %6.3f) of %.2f m2;"
                   " per-part offsets average %.3f m (mean of magnitudes)\n",
                   glm::length(cpMove), glm::dot(cpMove, nose), glm::dot(cpMove, right),
                   glm::dot(cpMove, up), cpArea,
                   cpArea > 0.0 ? cpOffsetMagSum / cpArea : 0.0);
            printf("[ship] SUM projA %.2f  unionA %.2f  cp fwd %.2f right %.2f up %.2f  cd %.2f\n",
                   cpArea, A_ship, glm::dot(rc, nose), glm::dot(rc, right), glm::dot(rc, up),
                   lastDragCd);
            fflush(stdout);
        }
        /* #486: the hull answers to the flow AT ITS OWN PLACE. Wherever cp sits,
           this airframe's rotation carries it through the air at w x (cp - com),
           and with the COM flow that sweep is invisible -- which left the largest
           area on most ships (35 m2 broadside on the jet) unable to react to a
           rotation at all. Rockets, capsules and cargo sleds are ALL hull, so this
           was the only damping they could ever have had, and they had none. That
           sweep term OPPOSES the rotation for either sign of the lever; what the
           lever's sign decides is the other half -- hull drag at an angle of attack
           pushes the nose back into the flow only with cp behind the COM (#454's
           static margin).
           The AREA stays keyed to the COM flow direction: #463's union needs one
           projection plane for the whole ship, so a per-point silhouette is not
           available here. That makes this a body-shape term with a local flow, not
           a surface -- a surface gets both (see the lift and control loops). */
        const LocalFlow lfCp = localFlow(vrel, w, cp - com, rho);
        const glm::dvec3 fdrag = dragForce(da, drag_cd * lastDragCd, A_ship,
                                           alt, lfCp.vp);
        if(glm::length2(fdrag) > 0.0) {
            const glm::dvec3 rcp = cp - com;
            ApplyForce(hull, rcp, fdrag);        // translation + (rcp x fdrag)
            moment += glm::cross(rcp, fdrag);
            ftotal += fdrag;
            // The overlay sees drag AT THE CENTER OF PRESSURE: that offset
            // IS the weathervane story.
            if(recordForceVecs) {
                lastForceVecs.push_back({ rcp, fdrag, kForceDrag });
            }
        }
    }

    /* SURFACE FORM DRAG (#455): every part the hull union just refused pays here --
       on the planform its author declared rather than on whatever silhouette the
       mesh happens to throw at the flow, and AT ITSELF rather than at the hull's
       center of pressure. Three things change and they are one change:
        - the AREA is the planform (lift_area / control_area), the same number the
          lift law below is paid on. A wing edge-on is 0.4 m2 of mesh and 8 m2 of
          wing; the 8 is the one that means something, and charging the projection
          instead was the double bill this issue is about.
        - the COEFFICIENT is the plate's own shape drag (drag.h::surfaceFormCdA) read
          at the flow the surface itself stands in (#483): chordwise that is the
          authored profile cd, out of its plane the surface pays its planform as the
          plate it is, and spanwise it pays its edge (thickness x chord off the mesh
          box), which is #559 fixing #455's one-axis blend. There is deliberately no
          alpha-linear plate term here on top of the induced term below -- turning the
          flow downward is charged once, as inducedCd.
        - the PLACE is the mount, answering to w x r there. A surface the airframe
          rotates through the air therefore damps the rotation on its own; the hull
          term cannot do that for a surface, because it answers at one cp for the
          whole ship (#486). On the jet that is the difference between 15 m2 of
          stalled wing pushing on nothing and pushing 3 m off the roll axis. */
    {
        for(Part *p : parts) {
            if(p->def == nullptr) { continue; }
            const PartDef *d = p->def;
            /* The same predicate the two silhouette loops above use to SKIP this part:
               a part is charged as a body or as a surface, never as both, and the one
               function decides which. */
            if(!isAeroSurface(d->lift_area, d->control_area)) { continue; }
            /* One part, one area: shipdef.cpp refuses a part with both, so this is
               a choice between "lifting surface" and "steering surface", never a
               sum (#551 would merge those two laws, not this test). */
            const double ref = (d->lift_area > 0.0) ? d->lift_area : d->control_area;
            const glm::dvec3 ri = partPosS(p) - com;
            const LocalFlow lf = localFlow(vrel, w, ri, rho);
            if(!(lf.vp2 > 0.0)) { continue; }
            const double vpl = std::sqrt(lf.vp2);
            /* cd at the surface's OWN incidence, not the COM's: the sweep that
               makes this law damp a rotation also tilts the flow this part sees.
               The rotation matters twice over here. lf.vp is an S-frame vector and
               the plate's axes are as their author declared them, which are
               SHIP-LOCAL -- so the flow comes to them, not the other way round (the
               hull loop above does the same with vhatS = transpose(sRot) * vhat, and
               getting this backwards reads a ship's cd off its attitude relative to
               the world rather than relative to its own air: measured 0.633 against
               a true 1.000 on a jet flying dead straight down its own nose). */
            const glm::dvec3 flowLocal = glm::transpose(sRot) * (lf.vp / vpl);
            /* A plate has three ways to meet the flow and they are three different
               areas (#559): chordwise it pays its profile cd on the planform, out of
               its plane it pays the planform as a flat plate, and along its SPAN it
               pays the thickness x chord its mesh actually throws that way. Which of
               X or Y is the normal comes off the mesh box, so a wing (normal up) and a
               fin (normal sideways) both read honestly and a builder who authors the
               planform the other way round still gets the right shape drag. */
            const glm::dvec3 plateNormal = (d->meshNormalAxis() == 0) ? p->localRot[0]
                                                                     : p->localRot[1];
            /* #543 A5b: the same one-read-per-surface rule A5 used for the slope. This
               surface's Mach, its thickness ratio, and the wave cd that follows, spent
               ONCE on the two chordwise anchors below -- never on the broadside or edge
               share, because those are not a section meeting the flow chordwise. A
               subsonic surface pays nothing (waveCd is 0 under kTransonicMach), so
               every Kerbin golden in the battery is untouched by this line. */
            const double wave = waveCd(d->meshThicknessToChord(), flowMach(lf, invAirA));
            const double cda = drag_cd * surfaceFormCdA(
                ref, d->meshEdgeArea(),
                d->drag_forward + wave, d->drag_side, d->drag_backward + wave,
                glm::dot(p->localRot[2], flowLocal), glm::dot(plateNormal, flowLocal));
            if(!(cda > 0.0)) { continue; }
            const glm::dvec3 fprof = -(lf.vp / vpl) * (lf.q * cda);
            ApplyForce(hull, ri, fprof);
            moment += glm::cross(ri, fprof);
            ftotal += fprof;
            /* A CdA, like Induced= and Chute= on the [drag] line: not a force, and
               not this surface's force divided by the ship's q either (#483 gives
               each surface its own). */
            lastSurfCdA += cda;
            /* ... and it pushes HERE, which is the half the hull's cp never saw:
               a fin 3 m aft is a stabilizer whether or not the hull's own cp is
               anywhere near it. */
            formCdA += cda;
            formCdArm += cda * glm::dot(ri, nose);
            if(recordForceVecs) {
                lastForceVecs.push_back({ ri, fprof, kForceSurfDrag });
            }
        }
        /* + = the airframe's form drag sits AHEAD of the COM (an angle of attack
           drives the nose further in), - = behind it (the weathervane restores).
           Hull and surfaces weighted by what each one pays, so a tail fin counts at
           its own arm instead of being averaged into a hull silhouette (#455). */
        lastFormCpLever = (formCdA > 0.0) ? (formCdArm / formCdA) : 0.0;
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
        // The flow AT THE CANOPY: vrel plus the rotation sweep (w x r). That sweep IS
        // the harness damping -- without it the restoring torque spins the ship up
        // without bound under its own canopy. Every other surface law now does the
        // same (#483); this one always has.
        const LocalFlow lf = localFlow(vrel, w, ri, rho);
        const double vpl = glm::length(lf.vp);
        if(vpl <= 0.0) { continue; }
        const double qpt = 0.5 * rho * vpl * vpl;   // from vpl, not lf.q: same flow,
                                                    // and a canopy descent is a chaotic
                                                    // baseline not worth moving by an ulp
        // Deployment limiter (kChuteAccelLimit): the canopy never pulls harder than
        // a_lim -- physical, and it caps the weathervane torque inside the substep's
        // stability envelope.
        const double area = std::min(p->def->chute_area,
                                     kChuteAccelLimit * getMass() / qpt);
        const glm::dvec3 fchute = -(lf.vp / vpl) * (qpt * area);
        ApplyForce(hull, ri, fchute);            // translation + (ri x fchute)
        moment += glm::cross(ri, fchute);
        ftotal += fchute;
        if(recordForceVecs) {
            lastForceVecs.push_back({ ri, fchute, kForceChute });
        }
        lastChuteCdA += area;  // the EFFECTIVE (limited) area: the drag log
                               // then shows the limiter working.
    }

    // LIFT (per part): each surface lifts on its OWN area at its OWN position, in
    // its OWN frame (#453), against the flow AT ITSELF (#483) -- the angle a wing
    // is mounted at is incidence, not decoration, and a wing the body is sweeping
    // down through the air meets a tilted flow that its twin on the other tip does
    // not. The moment comes from the surfaces' distribution.
    /* #454: the derivative of that distribution -- see drag.h::LiftSlopeSum. Two
       dots and a cross per lifting surface per substep, riding a pass that is
       already paying for an atan2 pair and a flow composition there, and it is what
       lets the drag log answer "is this airframe stable" instead of guessing. */
    LiftSlopeSum npSum;
    for(Part *p : parts) {
        if(p->def == nullptr) { continue; }
        const PartDef *d = p->def;
        if(d->lift_area <= 0.0) { continue; }   // the frame is only worth building
                                                // for a surface that lifts
        const glm::dvec3 ri = partPosS(p) - com;  // surface's offset from the COM
        /* The rotation sweep, w x r, is the whole difference: a body turning at w
           carries this surface through the air, so its angle of attack tilts by
           atan(|w x r| / along-flow) and its twin's tilts the other way. The chute
           harness above has always done this and said outright that the sweep IS its
           damping; this is that term on the lifting law.
           What it does here is NOT one behaviour but three, all measured in situ on
           both shipped airframes (every aero substep: the moment this term ADDS over
           the old COM-flow law, dotted with w -- negative opposes the spin, positive
           feeds it):
             Below the stall the slope is positive, so the surface the rotation drives
             toward the stall peak gains lift and the one it pulls away loses it, in
             the direction that OPPOSES w. Damping. Jet: dM.w from -1 to -3885 W, peak
             at |w| = 2.64 rad/s flying 3.6 deg off the flow. Glider: down to -513 kW.
             Inside the deep-stall droop -- alpha past stallAngle, under 2x it -- the
             slope is NEGATIVE, so the surface the rotation pulls back toward the
             stall peak gains lift instead of losing it and the moment ALIGNS with w:
             the airframe feeds its own swing back in. Both ships pump there (jet
             dM.w up to +3080 W, glider up to +413 kW, and the pumping samples sit at
             |AoA| 27-41 deg, i.e. inside 20.1-40.1). Whether a shipped part should
             droop at all is a question, not a fact to pin; it is filed as #487.
             Past 2x stallAngle the curve is identically zero, so there is nothing to
             damp: every sample with |AoA| > 70 on either ship measures dM.w = 0.0.
             What recovers a ship from broadside is the hull's form drag (#486), not
             this term.
           Sizes, and they are small on the jet: at |w| up to 2.65 rad/s its lifting
           surfaces see at most 1.46 m/s of sweep (they sit about 0.55 m fore/aft of
           the pitch pivot, 3 m out along the span), while its rudders and elevators
           see up to 6.39 m/s -- and those make no lift at all, so their sweep moves
           their q, not their force. The glider, tumbling at |w| up to 5.28, sees
           16.2 m/s at its lifting surfaces.
           A body that is not spinning is bit-identical to the old law: w x r is
           exactly zero, so vp and qp come out of the same arithmetic as the vrel and
           v2 this loop used. */
        const LocalFlow lf = localFlow(vrel, w, ri, rho);
        const glm::dvec3 vp = lf.vp;
        const double qp = lf.q;                        // this surface's own q
        /* The mount in S: localRot is part-local, sRot carries it into the same
           frame the flow and the hull live in. One mat3 multiply plus two atan2
           per LIFTING part per substep -- not per part, which is why the
           lift_area test above comes first. The neutral point below pays two more
           tilted samples of the same thing per lifting part (drag.h::liftSlopePerQ
           takes this `pa` instead of sampling it a third time). */
        const glm::dmat3 mount = sRot * p->localRot;
        const PartAero pa = partAero(mount, vp);
        if(!pa.valid) { continue; }   // a mount with no usable axis lifts nothing
        /* #485: what this surface's law actually read, for the F4 overlay, the
           SURFACE window and the [surface] log. Recorded after the valid gate (an
           edge-on or axis-less mount quotes nothing rather than a stale angle) and
           before the zero-force test below -- a surface at alpha 0 is measured, it
           just is not pushing. Free: the angle and q are already in hand here, and
           Mach is one sqrt on a square the flow already computed. */
        p->aeroAlphaDeg = (float)surfaceAlphaDegForReadout(
            pa, surfaceReadoutFrame(p->localRot[1], shipLocalUp, shipLocalRight),
            up, right);
        p->aeroQ = (float)qp;
        const double mach = flowMach(lf, invAirA);
        p->aeroMach = (float)mach;
        p->aeroValid = true;
        /* #543 A5: THE ONE COMPRESSIBILITY READ. This surface's Mach, corrected
           slope, once. Everything downstream -- the neutral-point slope below, the
           lift force, and (in the control loop) the hinge force and #487's rate
           damping -- spends this number rather than the authored d->cl, so the
           regimes cannot disagree with each other the way three separate Mach
           constants would. drag.h::liftSlopeMach owns the shape and the band edges. */
        const double clq = liftSlopeMach(d->cl, mach);
        /* #454: this surface's contribution to where an alpha change puts the change
           in lift -- see drag.h::liftSlopePerQ for why the derivative is taken by
           tilting the flow rather than by weighting liftDir. Accumulated BEFORE the
           zero-force test below, on purpose: a surface at alpha 0 makes no force yet
           but has all the slope in the world, and the neutral point of a build is
           exactly the sum of those. A surface inside the stall droop contributes
           negatively, so the readout turns against the airframe at the same angle the
           airframe does (#487's band) rather than lying about it -- and where the
           surfaces stop agreeing with each other the pair dashes rather than quoting
           the average of a cancelled sum (drag.h::LiftSlopeSum::usable). */
        npSum.add(liftSlopePerQ(pa, mount, vp, d->lift_area, clq, d->stall_angle,
                                right),
                  ri, up, right);
        const glm::dvec3 flift = liftForce(qp, d->lift_area, clq, pa.alpha,
                                           pa.liftDir, d->stall_angle);
        if(glm::length2(flift) <= 0.0) { continue; }
        ApplyForce(hull, ri, flift);             // translation + (ri x flift)
        moment += glm::cross(ri, flift);
        ftotal += flift;
        lift_total += flift;
        if(recordForceVecs) {
            lastForceVecs.push_back({ ri, flift, kForceLift, p->aeroAlphaDeg, true });
        }
        /* The price of that lift (#462) is billed per SYSTEM, after every surface
           has contributed (#556): the bill is CL_sys^2/(pi e AR_sys), and CL_sys
           needs the whole wing's lift. Park the sample and let applyInducedDrag
           price it -- see that function for the arithmetic. The comment used to say
           the term "follows any change to it" of the lift law; the sample carries the
           FORCE the law just made, so it still does. */
        aeroLiftSamples.push_back(
            { ri, vp, qp, d->lift_area, glm::length(flift), p->aeroSystem });
    }

    // Control surfaces: each steers ONE axis, driven by that axis's stick alone. The
    // force is the lift law with deflection in place of AoA (controlForce), applied at
    // the surface so its offset from the COM is the steering leverage.
    {
        lastControlDeflections.clear();
        int ctrlIndex = 0;  // the Nth control surface (disambiguates instances)
        for(Part *p : parts) {
            if(p->def == nullptr) { continue; }
            const PartDef *d = p->def;
            if(!d->isControlSurface()) { continue; }
            // cl_control if set, else the lift slope cl
            const double clc = controlCl(d->cl, d->cl_control);
            // pure controlAxisParams (pinned in test_shipload)
            const ControlAxisParams ax = controlAxisParams(d->control_axis);
            /* #426: THE ASK IS THE SOLVER'S, NOT THE STICK'S. The stick stopped
               wiring straight to a surface: three keys become one demanded torque
               (#425 D), actuateAttitude splits it over drums AND surfaces in one
               actuatorSolve call, and what comes back for this part is radians of
               COMMAND -- already in the authored sense, because the row it came from was
               built with controlDeflectionSign (#495), so it joins the trim and goes
               through controlDeflection below exactly as the stick used to. Two things
               this buys and one it costs, all deliberate:
               - any demand can reach any surface whose rig levers part of it, which is
                 the mix (#426). control_axis stays the label for WHICH KEY asks and
                 which axis decides the rig sign, not a wall on what the plate is for.
               - the same solve bounds what the plates can promise, so a law and a key
                 can no longer both ask a surface for its full travel (#420's shape).
               - and the plates answer nothing when the solver has no rows: no air, or
                 no q, or a rig the solver was right to skip. A hull sitting on the pad
                 used to throw its surfaces at nothing. "No air, no throw" is the
                 honest answer; if a builder wants a rig check, that is a display
                 feature, not a second physics wire.
            TRIM (#492) still joins the COMMAND, in the same units (fractions of this
            part's travel), which is what makes a mirrored pair and a canard take the
            same trim number and still torque the same way -- see drag.h::
            controlCommandFrac. */
            double ask = 0.0;
            for(const std::pair<const Part *, double> &sl : surfaceLat_) {
                if(sl.first == p) { ask = sl.second / d->max_deflection; break; }
            }
            const double cmd = controlCommandFrac(ask,
                                                  glm::radians(p->trimDeg)
                                                      / d->max_deflection);
            if(cmd == 0.0) {
                /* No command, no deflection, no force: controlForce is a deflection
                   law, so a surface nobody touched contributes exactly nothing however
                   the airframe is swinging. Skipped BEFORE the offset, the flow and the
                   mount frame, which is most of this loop's cost and, at shipped part
                   counts, mostly paid for surfaces sitting neutral. The surface index
                   still advances so a held pair keeps its numbering (flight/453 filters
                   on it), and controlDefl stays where its per-frame reset left it.
                   The test is against the COMMAND, not the stick: a trimmed surface is
                   commanded even with nothing held, and skipping it there would make
                   trim do nothing -- which is also why there is no dead band. Since
                   #426 `cmd` comes out of actuatorSolve rather than a key, and the
                   solve's answer for a surface it did not touch is exactly 0.0 (its
                   setpoint vector starts at zero and only a non-zero line share moves
                   it), so an exact compare stays safe. A dead band would swallow the
                   small trims that make an airframe fly level -- and now it would also
                   swallow a solver's small corrections, which is what a law near
                   attitude relies on. */
                if(recordSurfaceAero) {
                    /* #485: a NEUTRAL surface's own angle is exactly what #454's
                       alpha term and #487's pump band want to see, so when a readout
                       is open pay for the flow and mount frame this early-out exists
                       to skip. Only when one is open -- the hot path stays as cheap
                       as it was. */
                    const LocalFlow lf0 = localFlow(vrel, w, partPosS(p) - com, rho);
                    const glm::dmat3 mount0 = sRot * p->localRot;
                    const PartAero pa0 = partAero(mount0, lf0.vp);
                    p->aeroAlphaDeg = (float)surfaceAlphaDegForReadout(
                        pa0, surfaceReadoutFrame(p->localRot[1], shipLocalUp,
                                                 shipLocalRight),
                        up, right);
                    p->aeroQ = (float)lf0.q;
                    p->aeroMach = (float)flowMach(lf0, invAirA);
                    p->aeroValid = pa0.valid;
                }
                ++ctrlIndex;
                continue;
            }
            /* The force leaves along the SURFACE's own normal out of the flow (#453),
               not along a ship-wide direction. A v-tail's fin then pushes a pitch/yaw
               mix, and a fin mounted upside down pushes the way it is bolted rather
               than the way the pod points. The COMMAND axis below stays the ship's:
               the stick promises a ship torque, and the sign picks the deflection
               that delivers it whichever way the surface faces. On jet.json and
               glider.json at ZERO spin the total control force and moment are
               unchanged by that (pinned in test_shipload): a fin and its twin,
               mounted 180 deg apart on the stack, flip their normals and their
               deflections together. */
            /* The flow AT THE SURFACE (#483), for the same reason the lift loop pays
               it: a body turning at w drags its surfaces through the air. On this jet
               the rudders and elevators see up to 6.39 m/s of sweep (a 3.65 m lever,
               against |w| up to 2.65 rad/s mid-swing). It changes three things here,
               and the magnitude is one of them: an actuated surface's q ranges 0.93
               to 1.07 of the COM's on the jet (0.85 to 1.11 on the tumbling glider)
               and, because a twin pair stands either side of the rotation axis, it
               moves UNEQUALLY -- so the pair's sideways forces no longer cancel, which
               is part of why this ship's max |Beta| over the 458 rows went 0.3 deg
               before #483 to 2.5 after. It also tilts the force DIRECTION (liftDir)
               and with it the sign of the leverage. There is still no alpha term:
               controlForce answers deflection only, so a surface the stick has not
               touched makes no force however it is swinging (#454 is what gives a
               surface a say from its own angle). Note when it is biggest: at the top
               of a 77 deg excursion the rate is near zero (a turning point), so the
               sweep is largest mid-swing, not where the angle looks worst. */
            const glm::dvec3 ri = partPosS(p) - com;
            const LocalFlow lf = localFlow(vrel, w, ri, rho);
            const double qp = lf.q;
            const glm::dmat3 mount = sRot * p->localRot;
            const PartAero pa = partAero(mount, lf.vp);
            /* #485, same as the lift loop. The law below still answers deflection
               alone -- recording the angle makes it visible, not load-bearing. */
            p->aeroAlphaDeg = (float)surfaceAlphaDegForReadout(
                pa, surfaceReadoutFrame(p->localRot[1], shipLocalUp, shipLocalRight),
                up, right);
            p->aeroQ = (float)qp;
            const double mach = flowMach(lf, invAirA);
            p->aeroMach = (float)mach;
            p->aeroValid = pa.valid;
            const glm::dvec3 forceDir = pa.liftDir;   // zero when the frame is unusable
            const glm::dvec3 about =
                (ax.aboutAxis == 0) ? right : (ax.aboutAxis == 1) ? up : nose;
            /* THE RIGGING IS THE MOUNT'S, NOT THE WIND'S (#495): which way a positive
               command swings this surface, and whether it swings it at all, is decided by
               mount[1] -- the direction a plate pushes at zero angle of attack, the same
               axis partAero measures alpha against and controlSurfaceVisualRot hinges
               about. Why the mount rather than partAero's live liftDir, and the two
               corners where that matters (a sign that flips mid-flight on an off-centreline
               mount; a rig whose push line runs through its pivot, now asked for NO travel
               rather than full travel with a sign the rig cannot decide), is carried by
               drag.h::controlDeflection -- which is where the measurements and the #508 and
               #509 pointers live. The call below is the whole implementation. */
            const double deflection = controlDeflection(pa, ri, mount, about,
                                                        ax.targetSign, cmd,
                                                        d->max_deflection);
            p->controlDefl = deflection;  // the mesh swing (Vehicle::Draw)
            lastControlDeflections.push_back({d, ctrlIndex++, deflection});
            // cmd != 0 here (the early-out above), so deflection is 0 only on an
            // unusable frame or a rig with no lever about this axis (#495) -- either
            // way there is nothing to bill, and the cl and force tests stay for a
            // surface with no slope and one that makes no force anyway.
            if(clc <= 0.0 || deflection == 0.0) { continue; }
            /* The surface's own stall_angle, the same droop a wing gets (#494's first
               half): a rudder turned 20 deg into the flow has separated, and the
               loader will not let a steering part ship without the number.

               #454's other half: a control surface answers its OWN angle of attack
               as well as the hinge. For a VERTICAL fin pa.alpha IS the sideslip
               (partAero measures about the plate normal, and a fin's normal is
               lateral), so a fin now weathervanes -- side force grows with beta and
               the tail swings back into the flow. That is what every player-built
               aircraft was missing: the fins only answered the pedal.

               A FLAP is different: it sits in a system that already carries a lifting
               planform (a wing), whose lift is billed from the same angle on that
               area. Adding the flap's own incidence there would count the trailing
               edge twice, so the term is gated on the system's liftArea (#556's
               grouping is what makes "is this a freestanding fin?" answerable). */
            const bool systemLifts =
                p->aeroSystem >= 0 && (size_t)p->aeroSystem < aeroSystems.size()
                && aeroSystems[(size_t)p->aeroSystem].liftArea > 0.0;
            const double aeroAngle = deflection + (systemLifts ? 0.0 : pa.alpha);
            const glm::dvec3 F = controlForce(qp, d->control_area,
                                              liftSlopeMach(clc, mach),
                                              aeroAngle, forceDir, d->stall_angle);
            if(glm::length2(F) <= 0.0) { continue; }
            ApplyForce(hull, ri, F);
            ftotal += F;
            moment += glm::cross(ri, F);
            /* #599: the plate's ACTUAL say, from the same force Bullet just got, so
               the readout cannot quote a promise the air refused: it is 0 in vacuum and
               where a plate has no q, droops past stall_angle or sweeps through its own
               turn (#483, #494). Trim is part of the command, so a trimmed airframe
               spends here too -- which is what a player watching "what is turning me"
               wants. Only the CONTROLLED surfaces count; a fixed plate's lift is not
               steering. */
            attDel_.surfaces += glm::cross(ri, F);
            if(recordForceVecs) {
                lastForceVecs.push_back({ ri, F, kForceCtrl, p->aeroAlphaDeg, pa.valid });
            }
            /* THE PRICE OF THAT DEFLECTION (#494), now billed per SYSTEM (#556). A
               steered surface turns the flow and pays induced drag for turning it,
               exactly as a wing pays for turning it up. Nothing billed it: until #492
               a deflection was a transient a player held for a second, but a trimmed
               airframe runs a standing 12-20 deg forever, and on skiff_trim the
               elevator pair downloads ~1.5 kN at cruise for a bill of exactly 0 N
               (both figures are the pre-#568 spindle hull's, whose tail stood half as
               far behind the CG; the shipped stack's Induced= totals 0.33 m2 once its
               phugoid damps).
               CL comes from the force the law JUST made, not from (alpha + deflection):
               the angle half of a tail's true angle of attack is already generated, and
               already billed, by the tailplane PART hanging next to the flap (the
               skiff's wing_r1h1, lift_area 2), so a cl*(alpha+delta)^2 bill would
               charge the same angle twice -- and would read zero bill at
               delta = -alpha, where the surface is pushing as hard as it ever will.
               Recovering it from F keeps the bill glued to the force: it vanishes with
               the command, is continuous through zero, and follows for free if the
               control law ever grows a hinge-effectiveness term (which would change F
               and thus this).
               #494 had to fold the tab's own downwash in alone, and recorded the cost
               of NOT folding it into the tail: the same skiff_trim state bills 0.24
               instead of 0.185 m2, ~35% more. With #556's grouping that fold-in is
               what happens -- geometry decides, not an authored linkage -- so an
               elevator whose chord touches its tailplane's now bills the tailplane's
               AR. Substitute controlForce's own F and the AREA cancels out of the
               charge:
                 bill = |F|^2 / (qp * pi * oswald * span_sys^2)
               so the only geometric knob is the SYSTEM's span, and the bill is
               QUADRATIC in the authority the surface has -- double a tab's cl_control
               and its price quadruples. */
            aeroLiftSamples.push_back(
                { ri, lf.vp, qp, d->control_area, glm::length(F), p->aeroSystem });
        }
    }

    /* #556: every surface that made lift or deflection this substep is in
       aeroLiftSamples; price the whole wing at once, along the flow each surface
       stands in (#483), at its own mount. */
    applyInducedDrag(hull, drag_cd, ftotal, moment, recordForceVecs);

    lastAeroForce = ftotal;
    lastLiftForce = lift_total;
    lastLiftVert = glm::dot(lift_total, com / r);
    lastAeroTorque = moment;
    /* #454: reported before the zero-force return below, because a ship flying at
       alpha 0 has no lift right now and still has a slope and a margin -- that is the
       whole point of these being derivatives. */
    lastLiftSlope = npSum.slope();
    /* A NaN here would print CLa=nan and the e2e parser would drop the whole tail, so
       the run would report a missing field rather than a broken one. */
    assert(std::isfinite(lastLiftSlope) && std::isfinite(npSum.dm));
    lastStaticValid = npSum.usable();
    if(lastStaticValid) {
        lastStaticMargin = npSum.margin();
    }
    if(glm::length2(ftotal) <= 0.0) { return lastAeroForce; }

    // Same spurious-torque cancellation as applyThrustForce / applyGravity.
    const glm::dvec3 dcom = comOffset();
    if(glm::length2(dcom) > 0.0) {
        ApplyTorque(hull, -glm::cross(dcom, ftotal));
    }
    return lastAeroForce;
}

void Vehicle::applyControlForces(double h) {
    /* The force overlay's per-substep clear lives in the tick.cpp substep
       loop (it feeds the aero + control appliers together); this override
       chain cannot skip it. */
    /* Rotation runs BEFORE thrust so the substep's TVC aim (tvcLat_, armed by the
       one allocator call) is set before applyThrustForce steps the actuators and
       pushes along them -- same-substep, no lag. */
    applyRotationForce(h);
    applyThrustForce(h);
    applyRcsForce(h);
}

Part * Vehicle::attitudeRef() const {
    return rootPart();
}

glm::dmat3 Vehicle::attitudeBasis() const {
    const Part *ref = attitudeRef();
    /* A hull with no parts has no axes to report; the identity is the honest answer
       and the drawers guard on it the same way they guard on a null controller.
       One partRot() rather than three partAxis() calls: partAxis(p, n) IS partRot(p)[n],
       so the axis-by-axis spelling solves the ship's frame S and composes this part's
       local rotation three times to arrive at one basis. */
    if(ref == nullptr) { return glm::dmat3(1.0); }
    return partRot(ref);
}

void Vehicle::applyRotationForce(double h) {
    /* The TVC aim is per-substep: cleared here, armed by the single allocator call
       below, consumed by applyThrustForce this same substep. */
    tvcLat_ = glm::dvec3(0.0);
    tvcCalls_ = 0;
    /* Same clear-and-assign discipline for the fin setpoints (#426): whoever asks
       gets an ask, whoever does not gets the plates back to neutral. actuateAttitude
       is not called at all when the demand is zero, so this is where the reset lives. */
    surfaceLat_.clear();
    /* And for the delivery record (#599): a substep nobody asked for shows 0 N m, not
       the last substep that was. The #648 EC bill clears here for the same reason, and
       it matters rather than being tidy: actuateAttitude is not called at all when the
       demand is zero, so a latched law that has ARRIVED (kill-rot on a hull that has
       stopped, a slew that has aligned) would otherwise keep paying the last substep that
       had an ask, for as long as the mode stays latched. */
    attDel_ = Vehicle::AttitudeDelivery{};
    wheelSpendWatts_ = 0.0;
    Part *ref = attitudeRef();
    /* ONE capability answer per substep (#594): the stick path, the slew law, the
       kill-rot law and the [slew] trace each used to ask attitudeAuthority() for a
       copy of its own -- a parts scan each time for the TVC levers and the drum
       axes, when everything in it except the power gate and the COM is unchanged a
       substep later. Everything below reads this one. */
    const AttitudeAuthority auth = attitudeAuthority();
    /* The frame the stick speaks, the frame the mixer aims in and the frame the
       pilot-vs-SAS policy masks in are ONE frame (#571): attitudeRef(), the root
       part, is also what AttitudeAuthority::stackAxis names. Key order is the
       stick's (roll, pitch, yaw); the senses match the old worldAxis sum: +nose,
       -right, -up. */
    const bool stickLive = ref != nullptr
        && (stick[0] != 0.0f || stick[1] != 0.0f || stick[2] != 0.0f);
    glm::dvec3 key[3] = {};
    if(stickLive) {
        /* partRot() is two Bullet quaternion round trips and this path runs per
           substep per hull, so the frame is only worth solving when a key is held.
           A hull with no key and no latched law asks for nothing below. */
        const glm::dmat3 axes = partRot(ref);
        key[0] = axes[2]; key[1] = -axes[0]; key[2] = -axes[1];
    }

    /* The pilot's demand. The stick is INTENT, not wiring (#425 step D): three keys
       become ONE demanded torque vector and actuateAttitude() is the only thing that
       decides who answers -- wheels, TVC, or both. It used to be two wires: this
       block drove the drums directly and applyThrustForce mixed the stick into the
       gimbals by hand, so the two actuators each took the stick at full rate and only
       a per-engine clamp sorted out the excess (#420). A wheel mounted on a tank's
       side is ROLLED 90 deg about the stack, so reading the stick in THAT part's frame
       swapped pitch and yaw: W yaws, A pitches.
       Each key is scaled by what the hull can actually bring to bear ABOUT its own
       axis, so a held key asks for full authority and no more. A multi-key hold is
       NOT normalized: it can over-subscribe a line, which is what #577 left alone on
       purpose -- the allocator, not the stick, is the physical bound.
       The power gate lives in those rows now: no EC, no wheel rows (#418) -- and no
       rows when the player has switched the drums off (#425 D-2), which is what makes
       a wheels-off hull route the same keys to the gimbals instead of going mute. */
    glm::dvec3 stickDemand(0.0);
    if(stickLive) {
        for(int k = 0; k < 3; k++) {
            if(stick[k] == 0.0f) { continue; }
            stickDemand += (double)stick[k] * attitudeTorqueAbout(auth, key[k]) * key[k];
        }
    }

    /* The law's demand, kept SEPARATE from the pilot's because the conflict policy
       (#421) decides per axis who gets it -- once they are one vector that question
       cannot be asked any more.
       Autopilot: one authority-bounded step of the slew/kill-rot law. It runs on ANY
       attitude authority -- wheels and/or TVC -- so a wheel-less ship (rcs_test,
       tvc_test) or an unpowered one still flies on its gimballed engines while they
       burn, and a ship with neither gets silence, not fake slewing (the #401 lesson:
       never promise a maneuver the hardware cannot deliver).
       The law STATES a demand rather than taking its own trip through the allocator:
       two trips double-bill the drums, because each call only hands the wheels its own
       share of what it was shown, so a latched kill-rot plus a held key on a 200 N m
       triad pushed -400 N m through drums rated 200 (#420). */
    glm::dvec3 lawDemand(0.0);
    if(slew == SlewKillRot) {
        lawDemand = killRotDemand(h, auth);
    }
    else if(slew != SlewNone) {
        lawDemand = slewDemand(slewTargetDir(), h, auth);
    }
    if(stickLive && sas_policy == SasStickOwnsAxis) {
        /* A held key owns its axis: take the law's component along that key axis away
           from it (the key axes are unit, so at full deflection this removes the
           component exactly). The law keeps the axes nobody is steering, so a prograde
           hold still corrects pitch while the pilot rolls. The reverse case -- a law
           whose error lies ALONG the steered axis -- is paused until the key comes up,
           which is what "the stick owns the axis" means. SasKeepsAsk skips this and
           lets the two sum, the pre-policy behaviour, pinned as the alternate end.
           The share is |stick| rather than all-or-nothing, so a key an inch off
           centre fades the law out by an inch instead of silencing a kN m ask with a
           touch. Keys only ever send +-1 today (tick.cpp), so nothing changes until
           analog input arrives; #598 asks the same question about how the stick mixes
           at partial deflection, and #421 still owes whether a linear crossfade is the
           right shape at all.
           One honesty note: this projects the law's TORQUE demand (I * dW / h), not
           its rate error, so on a hull whose inertia tensor is not diagonal in the key
           frame the leftover is not exactly "no ask about the unsteered axes". The
           leak is cross-axis and small; no shipped hull is asymmetric enough to show
           it, and #421 is where it gets settled if it ever matters. */
        for(int k = 0; k < 3; k++) {
            if(stick[k] == 0.0f) { continue; }
            lawDemand -= (double)std::fabs(stick[k]) * glm::dot(lawDemand, key[k]) * key[k];
        }
    }

    /* One DEMANDED torque per substep: whoever is asking adds to it, and
       actuateAttitude() answers it exactly once (#420). */
    const glm::dvec3 demand = stickDemand + lawDemand;
    if(glm::length2(demand) > 0.0) { actuateAttitude(demand, auth); }
}

/* The tick's release: no demand and no claim of having burned. The REQUEST goes
   with it, or a key released last tick keeps asking through this one. */
void Vehicle::clearRcs() {
    rcsDir = glm::dvec3(0.0);
    rcsRequest = glm::dvec3(0.0);
    rcsFiring = false;
}

glm::dvec3 Vehicle::rcsWorldDir() const {
    if(glm::length2(rcsDir) < 1e-12) { return glm::dvec3(0.0); }
    const Part *ref = attitudeRef();
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
    if(!consumeResourceMass(ResourceType::Hydrazine, flow, firstRcsPart()).ok) {
        return;  // no propellant, no force
    }
    // RCS owns no delta-v line to bill, so how much left the tank is not needed here.

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
    markRcsBurned();
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
    /* THE SPEND (#599), read the same way for every mode this row prints. The demand
       and the channels come from the last ACTUATED substep -- applyRotationForce clears
       the record every substep, so a law that asked for nothing reads 0/0 rather than
       the last substep that did. tau_ask well above tau_spent is the hardware refusing: a
       demand about a line no drum covers, a saturated actuator, #577's over-subscribed
       multi-key hold. tau_spent above tau_ask is the other way round and normal -- a
       gimbal keeps its aim between substeps (AttitudeDelivery in vehicle.h). Both columns
       are the whole vector's length, so they do not decompose: a channel that levers
       pitch and one that levers yaw sum into one magnitude. */
    const AttitudeDelivery del = attitudeDelivery();
    const double tau_ask = glm::length(del.asked);
    const double tau_spent = glm::length(del.wheels + del.tvc + del.surfaces);
    if(slew == SlewKillRot) {
        const glm::dvec3 w = hullAngVel();
        printf("[slew] t=%.3f mode=killrot |w|=%.4f rad/s "
               "w=[%+.4f %+.4f %+.4f] tau_ask=%.0f tau_spent=%.0f\n",
               time, glm::length(w), w.x, w.y, w.z, tau_ask, tau_spent);
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
        /* Named, not the "slew" default: an e2e CHECK on a free-aim law reads these
           rows, and a nameless row cannot be told apart from a construction bug (#606). */
        case SlewFree:       mode = "free"; break;
        default:             mode = "slew"; break;
    }
    /* One answer with the law (#400), the target test included -- they used to be two
       copies, and the log's was looser: at 0.5 it dropped a target the law was busy
       steering toward, and normalizing what was left (a zero vector) made E and axis
       NaN, so the row read alpha=0.000 next to a ship actually slewing on its engines.
       The authority comes from ONE build here: the row quotes two of its numbers, and
       asking for the second one separately would rebuild the budget again for one row
       (#594). */
    const AttitudeAuthority auth = attitudeAuthority();
    const Pointing p = pointingTo(target, auth);
    if(!p.valid) {
        printf("[slew] t=%.3f mode=%s %s\n", time, mode,
               glm::length2(target) < 1e-12 ? "no-target" : "no-frame");
        fflush(stdout);
        return;
    }
    const double E = p.angle;
    const double Ieff = p.Ieff;
    const double alpha = p.alphaMax;
    const glm::dvec3 axis = p.axis;
    /* alpha is what the actuators can spend THIS TICK: its TVC term rides the thrust
       the last ApplyThrust armed, so a TVC-only hull reads alpha=0 while it coasts and
       reads a fraction of itself while its law trims the lever. alpha_full is the same
       axis at full throttle (#608). The pair is what tells "this hull cannot turn"
       apart from "this hull is not burning" -- which the 96 case's engine-off drift
       has always depended on, and which a reader could not see before. */
    const double alpha_full = (Ieff > 0.0) ? availableTurnTorque(auth, axis) / Ieff : 0.0;
    /* alpha and alpha_full are RATINGS; the tau_ columns at the end of the row are what
       the last actuated substep cost, and tau_ask 0 next to a nonzero E is the story
       flight/96 used to have to tell through its drift alone: a law that has been handed
       no authority asks nothing, so the hull coasts with SAS lit and nothing spent. */
    target = glm::normalize(target);   // the printed direction is a unit vector
    /* The frame this rate decomposition belongs to (#421): nose, the axis the slew
       law actually drives, and the third that completes it. A hull-level |w| cannot
       say how much of itself is the ask and how much is an uncommanded roll.
       alpha is quoted ABOUT THAT AXIS, which is the tripwire this row has always
       been: a stack-axis error reads no TVC budget, exactly as tvcSplit hands that
       component to the wheels (#576) -- if the law and the mixer ever come apart
       again, this row shows it. alpha and alpha_full stay BUDGETS: under
       SasStickOwnsAxis an axis the pilot is holding shows its full authority even
       while the law gives it nothing (#421). The tau_ columns are the delivery --
       about no particular axis, since a channel that levers pitch and one that
       levers yaw sum into one magnitude. */
    const glm::dvec3 rollAxis  = p.facing;
    const glm::dvec3 thirdAxis = glm::cross(axis, rollAxis);
    const glm::dvec3 w = hullAngVel();
    const double w_slew  = glm::dot(w, axis);
    const double w_roll  = glm::dot(w, rollAxis);
    const double w_third = glm::dot(w, thirdAxis);
    const double w_des = (alpha > 0.0) ? std::sqrt(2.0 * alpha * E) : 0.0;
    /* The two directions at 6 dp, not 3: e2e pairs target= with --plane-log's
       n_orb, and a 3 dp component leaves ~3e-3 of dot-product slack -- the same
       size as the error it is there to catch. */
    printf("[slew] t=%.3f mode=%s E=%.4f rad (%.2f deg) "
           "w_slew=%+.4f w_roll=%+.4f w_third=%+.4f |w|=%.4f "
           "alpha=%.3f alpha_full=%.3f Ieff=%.0f w_des=%+.4f "
           "tau_ask=%.0f tau_spent=%.0f "
           "nose=[%+.6f %+.6f %+.6f] target=[%+.6f %+.6f %+.6f]\n",
           time, mode, E, glm::degrees(E),
           w_slew, w_roll, w_third, glm::length(w),
           alpha, alpha_full, Ieff, w_des, tau_ask, tau_spent,
           p.facing.x, p.facing.y, p.facing.z, target.x, target.y, target.z);
    fflush(stdout);
}

void Vehicle::att_log(double time) {
    if(hull == nullptr || parts.empty()) { return; }
    /* All three axes of the frame the controls speak (#571), not just the nose:
       pitch is about right, yaw about up, and a row that quotes only the nose
       cannot say which of the two a rate belongs to -- which is exactly what
       flight/571 has to tell apart. One partRot, three columns, no re-solve. */
    Part *ref = attitudeRef();
    assert(ref != nullptr);   // rootPart() falls back to parts[0], so non-empty means non-null
    const glm::dmat3 axes = partRot(ref);
    const glm::dvec3 w = hullAngVel();
    printf("[attlog] t=%.3fs nose=[%+.4f %+.4f %+.4f] "
           "right=[%+.4f %+.4f %+.4f] up=[%+.4f %+.4f %+.4f] "
           "w=[%+.4f %+.4f %+.4f] |w|=%.4f rad/s\n",
           time, axes[2].x, axes[2].y, axes[2].z,
           axes[0].x, axes[0].y, axes[0].z,
           axes[1].x, axes[1].y, axes[1].z,
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
    const glm::dvec3 w = hullAngVel();
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
                if(p->def->capacity[r] > 0.0) { res.push_back(r); break; }
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
                if(p->def->capacity[r] <= 0.0) { continue; }
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

void Vehicle::clearThrust() {
    for(Part *p : parts) { p->armedThrust = 0.0f; }
    /* Same per-tick intent as the armed thrust it sits next to (#400 F3): a tick
       without the keys must not keep claiming the player is on the throttle. */
    throttleKeyHeld_ = false;
}

void Vehicle::clearRotCmd() {
    stick[0] = stick[1] = stick[2] = 0.0f;
    slew = SlewNone;
}

void Vehicle::releaseControl() {
    setThrottle(0.0);
    clearThrust();
    clearRotCmd();
    /* The demand goes with the rest of the armed state. tick.cpp clears RCS once a
       tick for the ACTIVE ship only, while applyRcsForce runs for every live ship
       off the rails -- so a ship that loses its pilot and cannot rail (an atmosphere
       keeps it off rails, and goOnRails's failure is ignored) would keep thrusting
       on whatever keys it last held, and quietly drain its hydrazine doing it. */
    clearRcs();
}

/* This walk has TWO siblings, and one stage press pays for all of them: Sim::droppedAt()
   in staging.cpp answers the same question for the VAB preview before anything has
   separated, and extractSubtreeAsShip below rebuilds this same parent->children table and
   recomputes this same closure -- once per decoupler, so a press with three decouplers
   rebuilds it 1 + 3 times (#541). Sharing one table across a press is tempting but is NOT
   the free win it was for the preview: the preview walks per stage label every frame, which
   is what justified #520/#533's CSR table, while this runs once per keypress.
   Compared with Sim::droppedAt, this walk differs where the
   two situations differ: no liveness test here (Part has no dead flag -- a part leaves a
   Vehicle by moving to another one, so `parts` is exactly what is aboard), seeds on
   isDecoupler() rather than a precomputed flag, and the result is a pointer set, so the
   returned order is address order, which changes between runs of the same save, so no
   caller may read it as an ordering: game.cpp goes through stagedDecouplers below,
   which re-sorts and does not stop at depth. Both walks are iterative DFS over the same
   parent edges, and the `dropped` test above is what keeps a malformed parent edge from
   looping -- staging's seenEpoch does the same job. (#535) */
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
        // Iterative DFS over the decoupler's child side. Order inside the set is address
        // order, not this walk's order, so nothing downstream reads it as an ordering.
        std::vector<Part *> stack;
        auto kids = children.find(p);
        if(kids != children.end()) {
            for(size_t i = 0; i < kids->second.size(); i++) { stack.push_back(kids->second[i]); }
        }
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

/* The decouplers of one stage, in the order their split ships get made (#540). Two
   facts make the tie matter. droppedPartsAtStage hands back a std::set<Part*>, so
   its order is part ADDRESS order, which changes between runs of the same save; and
   decouplers that share a stage AND a depth are shipped, not hypothetical --
   measured over res/ships, 4 of 29 have them: heavy_one stage 4 is a 4-ring,
   mars_explorer_mk1 stage 4 has four at depth 12, and heavy_two has one 2-way tie
   while heavy_asp has three (stages 4, 5 and 6). Sorting on depth alone leaves the
   tied ones in address order, and that order decides which split ship keeps the
   bare name (dedupVehicleName gives the rest " #2", " #3") and which one the "next
   ship" handoff picks. std::stable_sort would not fix it: the input order is
   already the coin flip, and stability just preserves it. Build order -- index in
   `parts` -- breaks the tie because it is what a player reasons about. Depth still
   outranks build order: an outer decoupler has to separate before an inner one is
   reachable, and the inner one then extracts as no longer aboard.
   Only the stage's OWN decouplers, which are the walk's seeds. The walk also hands
   back decouplers of OTHER stages that merely ride along in a dropped subtree, and
   those are not split roots for this press: they stay aboard the ship that just
   split off and separate on their own label. Offering them is what makes a press
   fire stages it was not asked to fire when a root split is refused (#541).
   Re-walks the stage rather than taking the caller's `dropped`: a stage press is a
   rare event, and passing the vector in would be a second way for the two to drift. */
std::vector<Part *> Vehicle::stagedDecouplers(int stage) {
    std::vector<Part *> decs;
    for(Part *p : droppedPartsAtStage(stage)) {
        if(p->isDecoupler() && p->stage == stage) { decs.push_back(p); }
    }
    auto depth = [this](Part *p) {
        int d = 0;
        /* droppedPartsAtStage's walk is cycle-safe because it marks visited; this one
           follows a chain, so a malformed parent edge would spin here. It runs before
           any assert could fire, so bound the chain: a part can only have as many
           ancestors as the ship has parts. */
        while(p->parent != nullptr) {
            d++; p = p->parent;
            assert(d <= (int)parts.size());
        }
        return d;
    };
    auto buildIndex = [this](Part *p) {
        /* Every element came from `parts` through the walk above. A miss would
           return parts.size(), make every tie compare equivalent, and leave this
           function answering in the address order it exists to remove.
           "Build order" is a property of `parts`, not of this line: the vector stays in
           build order because every mutation appends or filters (see vehicle.h), which is
           what makes an index here mean what a player built. #544. */
        auto it = std::find(parts.begin(), parts.end(), p);
        assert(it != parts.end());
        return (int)(it - parts.begin());
    };
    std::sort(decs.begin(), decs.end(), [&](Part *x, Part *y) {
        const int dx = depth(x), dy = depth(y);
        if(dx != dy) { return dx < dy; }
        return buildIndex(x) < buildIndex(y);
    });
    return decs;
}

/* #583. getMass() is the same sum rebuildCompound() hands Bullet (effectiveMass over
   the parts, part.h), so the two totals here are exactly the two the merged Body is
   about to add. No ceiling beyond that: the game stores mass in a double, and this
   refuses only a pair whose sum stops being one. Reaching it means two hulls at
   ~1e308 kg each, i.e. hand-edited saves -- but the loaded-pair case is NOT covered
   by the save doors, which each judge ONE ship, and this is the one place in play the
   quantity changes. The merged POSE spread gets no door here: both hulls passed
   validateShipParts, and docking needs the ports within kDockCapture (game.h) of
   each other, so a merged spread can only reach the bound if the ships were already
   ~1e154 m across -- hulls broken in a hundred other ways long before the dock. */
std::string Vehicle::refuseDockWith(Vehicle *B) {
    assert(B != nullptr && B != this);   // the dock gate ruled these out
    if(std::isfinite(getMass() + B->getMass())) { return ""; }
    return "cannot dock with \"" + B->displayName()
           + "\": the two hulls together weigh more than the game can store";
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
    // B's cached silhouette went with its parts (and its key with it)
    B->aeroSilParts.clear();
    B->aeroSilNode.clear();
    B->aeroSil.clear();
    B->aeroGeomValid = false;
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
    if(!found) { return nullptr; }   // ordinary after an outer cut on the same stage

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
    if(droppedSet.size() == parts.size()) {
        /* A caller that has to explain this refusal says "this part is the ship's root".
           That holds only while the parent edges form a tree -- a cycle inside one ship
           reaches every part while the root still has a parent -- so trap the state that
           would make the sentence a lie. Sound for a tree: a non-null parent inside
           `parts` would be both an ancestor and a descendant of `root`. */
        assert(root->parent == nullptr);
        return nullptr;      // can't split the whole ship
    }

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
    /* Where the crew sits on a hull that has just lost its seat, or never named one.
       A capsule first: `controller` is the SEAT -- the first-person eye, the boot
       camera's initial focus, the Vessel window's home distance (#573) -- and the rule
       here used to be "the first reaction wheel", which #571 left meaning nothing but
       an actuator: the eye lands on whatever the wheels happen to be bolted to, at that
       part's own pose. With no capsule left on either hull, the part this ship is
       rooted at is the honest guess -- it is at least the part everything about the
       ship is measured from (attitudeRef()). Same rule as ShipDef::controllerIndex(). */
    auto seatOf = [](const std::vector<Part *> &ps, Part *fallback) {
        for(Part *p : ps) { if(p->isCapsule()) { return p; } }
        return fallback;
    };
    nv->controller = seatOf(nvParts, root);

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
    if(controllerDropped) { controller = seatOf(parts, parts[0]); }
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
       the mesh tips the same way its own force bends. A surface hinges along
       its own SPAN, which its part pose already carries (#453), so there is no
       ship frame to hoist and no per-axis selector to consult: a v-tail fin's
       hinge is its own slanted span, and a mirrored mount's is upside down
       along with its deflection. */
    for(Part *p : parts) {
        // One body per ship, so a part's pose is derived, not read off its own body.
        glm::dvec3 pp; glm::dmat3 pr;
        partPoseRelCom(p, pp, pr);
        if(p->def != nullptr && p->def->control_area > 0.0 && p->controlDefl != 0.0) {
            pr = glm::mat3_cast(controlSurfaceVisualRot(p->controlDefl, pr)) * pr;
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

/* Attitude amounts arrive normalized to +-1 of the ship's authority (see
   ShipCmd). Out of range is a sender bug; assert catches it -- no build here
   defines NDEBUG, so that fires in the shipped binary too. The clamp is for what a
   broken sender leaves behind: every consumer gates on `stick[i] != 0.0f`, which a
   NaN PASSES, so a non-finite demand must fail CLOSED (as in drag.h's NaN rule and
   attitudeTorqueAbout's NaN axis) rather than arm NaN torque for the whole tick. */
static float normCmd(float a) {
    assert(a >= -1.0f && a <= 1.0f && "attitude amount must be normalized to +-1");
    if(a < -1.0f) { return -1.0f; }
    if(a > 1.0f)  { return 1.0f; }
    return std::isfinite(a) ? a : 0.0f;
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
            throttleKeyHeld_ = true;    // the player is on the lever THIS tick (#400)
            break;
        case ThrottleDown:
            assert(step > 0.0 && "throttle ramp needs the tick's duration");
            adjustThrottle(-0.6f * (float)step);
            throttleKeyHeld_ = true;
            break;
        /* A SETPOINT, and the only channel that is not the player's (#400 F3): a law
           plans with availableAccel(), so it wants a fraction of rated thrust, not a
           ramp whose landing point depends on how long it has been latched. It does
           NOT set throttleKeyHeld_ -- the flag answers "is the PLAYER on the lever",
           which is what lets a law yield to a held key and resume when it is up. */
        case ThrottleSet:
            setThrottle(cmd.amount);
            break;
        case Thrust:
            ApplyThrust(step);
            break;
        /* Stick components in the ship's frame (see the `stick` member), and a
           NORMALIZED demand rather than a key flip (see ShipCmd). The consumers
           scale with it -- wheel torque and surface deflection (both through the one
           demanded torque in applyRotationForce, #426) and the TVC mixer -- so 0.5 is
           half a demand and 0 arms nothing. Note the old spelling forced +-1 off the
           SIGN of `amount`, so an amount of 0 meant full tilt; that reading is gone.
           #425 spends this number across the actuators instead of giving each one the
           full stick. */
        case Pitch:
            stick[1] = normCmd(cmd.amount);
            break;
        case Yaw:
            stick[2] = normCmd(cmd.amount);
            break;
        case Roll:
            stick[0] = normCmd(cmd.amount);
            break;
        /* RCS translation in the ship's own axes (rcsWorldDir). Sign only: a
           thruster group is on or off. Each key writes one component of the tick's
           REQUEST and then hands the whole request to armRcs(), the named writer, so
           the armed field stays a UNIT demand rather than a corner of a cube.
           rcsWorldDir() normalizes for a SHIP, so the unit invariant changes nothing
           there; what it buys is one rule for both paths, since the suit's
           rcsWorldDir (eva.h) returns the field exactly as it stands.
           The request is what makes three keys an even diagonal: normalizing the
           PREVIOUS answer instead used to rewrite one component to +-1 and rescale the
           rest, so the last key armed kept the full share and earlier ones shrank --
           N+I+L thrust along (.707,.5,.5), not (.577,.577,.577) (measured on
           rcs_box_pair, tmp/aero/n580_rcs.log). Holding a key and its opposite is
           still last-writer-wins, like a centered stick. */
        case RcsNose:
            rcsRequest.z = (cmd.amount >= 0) ? +1.0 : -1.0;
            armRcs(rcsRequest);
            break;
        case RcsUp:
            rcsRequest.y = (cmd.amount >= 0) ? +1.0 : -1.0;
            armRcs(rcsRequest);
            break;
        case RcsRight:
            rcsRequest.x = (cmd.amount >= 0) ? +1.0 : -1.0;
            armRcs(rcsRequest);
            break;
        /* The latched Autopilot mode (#424): mutually exclusive, held until
           toggled again. That rule lives HERE rather than in the Autopilot
           window, the Z/B key handler or --autopilot, so an autopilot that wants
           to engage a mode asks for it the way the player does and gets the same
           semantics. `slewRequest` is the REQUEST; tick.cpp copies it into `slew`
           while it is non-none, which is also what leaves the rails.
           `SlewNone` in the command is a construction bug, not a request to
           disengage: ShipCmd's default would otherwise turn a forgotten `mode`
           into a silent release. Releasing means sending the mode that is latched. */
        case Sas:
            assert(cmd.mode != SlewNone && "ShipCmd(Sas) must name a mode");
            slewRequest = (slewRequest == cmd.mode) ? SlewNone : cmd.mode;
            break;
        /* The aim a SlewFree law steers at (#606). A STATE write like ThrottleSet, NOT a
           latch: it never engages a mode, so a law that never sends ShipCmd(Sas,
           SlewFree) aims at nothing and can be debugged as such. Stored RAW on purpose:
           pointingTo()'s door (#609) fails a zero or NaN ask closed and the [slew] trace
           prints `mode=free no-target`, which is a better record than silently steering
           at the last good direction -- and normalizing here would erase the one state a
           law must be able to express, "I have no target this tick" (prograde of a ship
           sitting still on the pad is not a direction). */
        case Aim:
            slewFreeDir_ = cmd.dir;
            break;
        /* Kill rotation WHILE HELD: the one slew mode that is an override rather
           than a latch, so it writes the engaged copy and dies with clearRotCmd
           when X comes up. The latched spelling is ShipCmd(Sas, SlewKillRot),
           which is what the Autopilot window's "Kill rotation" button and
           --autopilot kill-rot use. */
        case KillRot:
            slew = SlewKillRot;
            break;
    }
}

glm::dvec3 Vehicle::GetVel() {
    return GetVelocity(hull);
}

/* The SETPOINT twin of adjustThrottle (#400 F3). Clamped here rather than asserted
   the way normCmd() asserts an out-of-range attitude demand: an attitude amount is
   normalized to +-1 of the authority BY DEFINITION, whereas a throttle fraction a law
   computes from an acceleration ratio legitimately saturates -- asking for 3 m/s^2 out
   of an engine that can give 1 is asking for 300%, and that is a saturated ask, not a
   sender bug. NaN fails CLOSED to 0 like every other control-state door: thruster_util
   feeds ApplyThrust's fuel rates and comes back as s.throttle in a save file. */
void Vehicle::setThrottle(double frac) {
    thruster_util = (frac >= 1.0) ? 1.0f : ((frac > 0.0) ? (float)frac : 0.0f);
}

void Vehicle::adjustThrottle(float delta) {
    thruster_util += delta;
    if(thruster_util > 1) { thruster_util = 1; }
    if(thruster_util < 0) { thruster_util = 0; }
}

/* One ignited engine's FULL-THROTTLE push (N): rockets at their rated value, jets at
   their PEAK speed (v = v_e/2, where drag.h's ram term tops out) in the air they
   actually have. The air has to be the LOCAL density and not the authored sea level:
   a jet at 15 km cannot produce its sea-level number, and since #400 this is the
   number a throttle law sizes a burn with, so sea level here would make a
   high-altitude jet look several times stronger than it is. Since #630 the gate is
   absolute (the machine's rating intake, constants.h:kAirSeaLevelDensity) and reads
   the body's oxidizer share, so this ceiling is the honest one on a thin or
   oxygen-free world: a jet's ceiling is EXACTLY 0 on Jool, 0.6 % of its rating on
   Eve and 0.007 % on Mars -- the air is there, its oxidizer is not -- and a law that
   sizes a burn with it says so. The peak speed is the honest CEILING -- the ship's
   own speed only ever lowers the ram term from it, which is what producedThrust()
   reports. */
static double partThrustCeiling(const Part *p, double rho, double ox_share) {
    if(p->isJet()) {
        return jetThrust(p->def->exhaust_velocity * 0.5, rho, ox_share,
                         p->def->jet_fan_thrust, p->jetFuelRate(),
                         p->def->exhaust_velocity, p->def->jet_intake_area);
    }
    return p->thrust();
}

double Vehicle::ceilingSum(const glm::dvec3 *want) {
    const int as = activeStage();
    const double rho = airDensityAtCom();
    const double ox_share = (m_parent != nullptr)
        ? m_parent->surface.atmosphere.oxidizer_share : 0.0;
    /* Frame S once. partAxis() re-solves it per part, and a law asks this every tick of
       a 51-part hull; the rest of this file reads it once per substep for the same
       reason (see "Frame S read once" in the aero pass). Only the directional caller
       needs it at all. */
    glm::dvec3 sPos;
    glm::dmat3 sRot(1.0);
    if(want != nullptr) { frameS(sPos, sRot); }
    double t = 0;
    for(Part *p : parts) {
        if(!p->isThruster() || p->stage < as) { continue; }
        double share = 1.0;
        if(want != nullptr) {
            /* How much of this engine's ceiling can be brought to bear along the ask.
               Measured from the MOUNT (localRot[2] = the authored +Z, rotated into
               frame S: exactly partAxis(p, 2)), not from partThrustDir(): the nozzle
               may swing anywhere inside its gimbal cone around that mount, while where
               it points this tick is whatever the steering asked for. Reading the live
               deflection would make a law size its burn with the player's stick and
               would DENY a gimballed engine authority it plainly still has. (The two
               rings together reach a little past the authored +/-gimbal_max stop --
               test_gimbal pins the swing -- so this cone is the shared-lateral clamp
               the gimbals are driven by, not a measured envelope.)
               cos(angle - cone) is the share at the best aim inside the cone: 1
               whenever the ask is reachable, negative past cone + 90 deg. Signed,
               because a thruster that can only push the wrong way is a fact a law
               should see rather than a floor at 0 -- and one lever (thruster_util)
               arms every ignited engine, so the push along dir really is the signed
               sum: an up-firing and a down-firing engine cancel for the rest of this
               burn. What this does NOT price is that swinging every nozzle at the ask
               spends the gimbal range the TVC steers with. #608 landed the other side
               of that bill (AttitudeAuthority::tvcTorqueAvailable, and
               availableTurnTorque about an axis), but the two reads are NOT additive:
               an aim that spends the cone along `want` and a turn that budgets the full
               sin(gimbal_max) double-count one lever. Nothing in the model adds them
               today -- tvcSplit only rotates the vector, which does price the axial cost
               of the deflection -- and pricing the CONE's availability is #419's work
               (circulating the gimbals to roll a cluster), not a sum to bolt on here. */
            const double off = acosUnit(glm::dot(sRot * p->localRot[2], *want));
            share = std::cos(std::max(0.0, off - p->def->gimbal_max));
        }
        t += share * partThrustCeiling(p, rho, ox_share);
    }
    return t * exhaust_scale;
}

double Vehicle::availableThrust() {
    return ceilingSum(nullptr);
}

double Vehicle::availableAccel() {
    /* m/s^2 at full throttle at the CURRENT mass. A burn plan wants this and not the
       thrust: what a stage can do to this hull HERE, so a throttle law can size a burn
       without reading back the throttle it is writing (#400). Draining propellant makes
       it a pessimistic bound -- getMaxTWR() is the optimistic end of that range. */
    return availableThrust() / getMass();
}

double Vehicle::availableAccelAlong(const glm::dvec3 &dir) {
    /* Fail CLOSED on a bad ask, because "bad" is worse than NaN here: glm's own
       normalize of a zero vector is NaN, glm::clamp does not rescue a NaN, and
       std::max(0.0, NaN) is 0.0 -- so the share would come out cos(0) = 1. A bad aim
       would read as FULL authority along nothing, which for a throttle law is a
       full-throttle burn in a direction nobody asked for. Delete this guard and the
       bad-ask pins go red at 10.0, not at NaN. The length covers the whole family:
       NaN for a NaN component, 0 for a subnormal ask whose length underflows, and inf
       for an infinite or overflowingly large one (an inf component normalizes to NaN
       and takes the same road to 1.0, so it must be stopped here too). The
       clamp-then-max shape is not local to this function: #622. */
    const double L = glm::length(dir);
    if(!(L > 0.0) || !std::isfinite(L)) { return 0.0; }
    const glm::dvec3 want = dir * (1.0 / L);   // unit; also skips a second length()
    return ceilingSum(&want) / getMass();
}

bool Vehicle::canBurn() {
    /* The same predicate ApplyThrust sizes each engine's draw with -- s = min over
       propellants of (available / desired), and s <= 0 means no burn -- asked as a
       yes/no rather than a fraction. A fraction would be the wrong shape for the ceiling
       beside it: an engine sizes its last tick by that share (burnShare), so ANY
       propellant at all can still produce thrust for a slice of a second -- rockets have
       always done this and jets do since #638. Availability is asked
       per engine, because availableResourceMass walks THAT engine's fuel groups: a live
       engine walled off by a fuel barrier has no drain layers at all (fuelDrainLayers,
       group -1) and reads false while the stacks are full. */
    const int as = activeStage();
    for(Part *p : parts) {
        if(!p->isThruster() || p->stage < as) { continue; }
        bool has_every = true;
        for(int r = 0; r < (int)ResourceType::Num; r++) {
            if(p->def->propellant_rate[r] <= 0.0) { continue; }
            if(availableResourceMass((ResourceType)r, p) <= 0.0) { has_every = false; break; }
        }
        if(has_every) { return true; }   // isThruster() already means one rate > 0
    }
    return false;
}

void Vehicle::ApplyThrust(double step) {
    if(thruster_util == 0.0f) { return; } /* zero throttle: no burn, no plume */
    const int as = activeStage();
    // Jet state shared by all jet parts this tick: air-relative speed, COM density,
    // and what share of this body's air a turbine can burn against (#630).
    const double v_air = glm::length(airRelativeVel(get_center_of_mass()));
    const double rho = airDensityAtCom();
    const double ox_share = (m_parent != nullptr)
        ? m_parent->surface.atmosphere.oxidizer_share : 0.0;
    /* --dv-log: the ship mass as this tick's drains proceed. getMass() walks
       every part and recurses into containment, so take it ONCE here and walk
       it down by what each drain ACTUALLY took (FuelDraw::drawn) instead of
       re-walking once per engine. The cursor only ever sees what this function
       drains -- rockets and jets; RCS, the EVA suit and cargo ride in other
       calls and are picked up by the next getMass() (that RCS/jet gap in the
       budget itself is the known limitation noted in vehicle.h). */
    double dvMass = dv_log_enabled ? getMass() : 0.0;
    for(Part *p : parts) {
        if(!p->isThruster()) { continue; }
        if(p->stage < as) { continue; } /* not ignited yet */
        if(p->isJet()) {
            /* Air-breathing: momentum balance (drag.h jetThrust). No usable air --
               vacuum, or an air with no oxidizer in it (#630) -- means no thrust AND
               no burn. Draws JET FUEL only. */
            const double T = jetThrust(
                v_air, rho, ox_share, p->def->jet_fan_thrust, p->jetFuelRate(),
                p->def->exhaust_velocity, p->def->jet_intake_area);
            if(T <= 0.0) { continue; }
            /* Burn with the air, not against it: the same gate jetThrust uses, so a
               thin-air jet is weak AND sips fuel (no usable air, no burn). */
            const double flow = p->jetFuelRate() * (double)thruster_util * step
                              * jetAirGate(rho, ox_share); /* kg this tick */
            /* Size the draw the way the rocket branch does (#638). consumeResourceMass is
               drain-or-nothing, so a jet holding less than one tick of flow used to be
               refused EVERY tick: quiet, with fuel still in the Vessel window, for ever.
               s pays for the thrust too -- an unscaled T would bill momentum the fuel
               never paid for, the same debt #630's ram gate closed. */
            const double avail = availableResourceMass(ResourceType::JetFuel, p);
            const double s = burnShare(avail, flow);
            if(s <= 0.0) { continue; }
            double amt = flow * s;
            if(amt > avail) { amt = avail; }   // see the ULP note on consumeResourceMass
            const FuelDraw drew = consumeResourceMass(ResourceType::JetFuel, amt, p);
            /* Sized first and clamped to what the sizing measured, so this cannot
               refuse -- the same invariant the rocket branch asserts below. Before #638
               the refusal was the bug: quietly, forever, with fuel in the window. */
            assert(drew.ok && "s must cover the jet draw too; no build sets NDEBUG");
            p->armedThrust = (float)(T * thruster_util * exhaust_scale * s);
            m_thrust = 1.0;
            /* No delta-v line for a jet -- but the kg it threw out the back IS
               mass the rockets later in parts order no longer push, so the
               --dv-log cursor steps down by it anyway (#531's rule: bill the
               tanks). propSpent_ stays rocket-only, as its name says. */
            if(dv_log_enabled) { dvMass -= drew.drawn; }
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
            const double si = burnShare(avail[r], desired);
            if(si < s) { s = si; }
        }
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
            const FuelDraw drew = consumeResourceMass((ResourceType)r, amt, p);
            assert(drew.ok && "s must cover every draw; no build sets NDEBUG");
            /* Bill the tanks, not the plan (#531): dm goes to --dv-log's rocket
               equation, so it has to be the mass the ship actually lost. `amt` is the
               draw this tick sized, and a pro-rata layer can come a couple of ULP
               under it -- see the FuelDraw note in vehicle.h. */
            dm += drew.drawn;
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

/* WHAT CAN TURN THIS SHIP RIGHT NOW, in one answer: see AttitudeAuthority in
   vehicle.h for the rows and the axis, and why they must be read per substep.
   Three sets of rows today -- drums, control surfaces (#426) and the gimbals' disk --
   and the attitude laws spend all three through the ONE actuatorSolve call in
   actuateAttitude. The surfaces used to be outside the model: the aero step answered
   the stick directly, so until #568 the shipped skiff had no wheel at all and in air
   its fins were its only authority while the laws saw none of it -- and they out-bill
   a pod's 100 N m drum by two orders of magnitude in flight. RCS nozzles are still
   never asked to turn the hull (#595). The Vessel
   window prints THIS answer rather than a parts-list rating, so a dead battery reads
   as 0 in the window and in flight rather than quoting a rating the ship cannot spend
   (#572, #418). The part window prints the part's own per-drum rating and its drum
   count. */
Vehicle::AttitudeAuthority Vehicle::attitudeAuthority() {
    AttitudeAuthority a;
    const Part *ref = attitudeRef();
    if(ref == nullptr) { return a; }   // no hull, no lever arm, no answer
    /* The mixer's frame: applyThrustForce aims in it and tvcSplit splits about it,
       and since #571 so do the slew/kill-rot laws and the manual stick. The budget
       below, actuateAttitude and those callers all read the same part, so none of
       them can be stated in a frame the others do not use. */
    a.stackAxis = partAxis(ref, 2);
    /* Power gate: the wheels are electric -- no EC, no wheel rows (#418 says a
       dead-ship trace used to read as uncontrolled when the engines were fine).
       A wheel is `torque > 0` (Part::isWheel()), so an empty `wheels` is exactly
       "nothing wheel-shaped is spending authority", with no separate has-a-wheel test.
       THE DRUM'S OWN AXES, not the stack: #576. A single drum levers the line it
       is mounted along, so a stacked one rolls the hull and does nothing else;
       a triad housing (PartDef::wheel_axes == 3, #593) is three orthogonal drums
       and so three rows on the part's own X/Y/Z, which is why it still covers
       three lines when a mount rolls or tilts the box. Every shipped wheel part
       is a triad; the single-drum half of this is what a cheap roll-only part
       would be, and what the sideways_wheel fixture exercises. */
    if(wheelsPowered()) {
        /* Two reasons for an empty `wheels`, and they are deliberately the SAME door
           (Vehicle::wheelsPowered, #425 D-2): no EC power (#418), or the hull has the
           drums switched off. Either
           way the hull's per-axis capability drops, the laws see the smaller answer,
           the Vessel window prints it, and a held key is scaled by what is left -- so a
           wheels-off rocket steers on its gimbals rather than pretending to have drums
           it declined to spend. */
        for(const Part *p : parts) {
            if(!p->isWheel()) { continue; }
            const double t = p->wheelTorque();
            /* wheelParts is the row->housing link (#648): the setpoints come back per
               row, and the watt figure to scale them against is per package. */
            if(p->def->wheel_axes == 3) {
                for(int k = 0; k < 3; k++) {
                    a.wheels.push_back({ partAxis(p, k), t });
                    a.wheelParts.push_back(p);
                }
            }
            else {
                a.wheels.push_back({ partAxis(p, 2), t });
                a.wheelParts.push_back(p);
            }
            a.wheelHousings++;
        }
    }
    /* TVC, read off the ARMED gimballed engines so it scales with throttle (and
       falls to zero when the engines quit mid-slew). Small angle (gimbal_max ~ 5
       deg): a deflection of |lat| rad off the stack axis pushes a lateral force
       T*|lat| at lever |h|, so
         heffMag = Sum(T_i*|h_i|)  N m / rad,  tvcTorque = heffMag*sin(gimbal_max).
       The lever is COM height ALONG THE STACK AXIS: an engine at COM height has no
       useful pitch/yaw lever, exactly as the mixer's sign flip is moot for it.
       The same levers are walked a second way into tvcTorqueAvailable (#608): rated
       thrust over the IGNITED stage instead of what the last ApplyThrust armed, so a
       plan can ask whether this hull could turn at all rather than watching its
       steering shrink with its own throttle law -- and read flat zero for the tick
       before a burn arms anything. Air is asked only if an ignited gimballed engine
       is in the loop at all (a jet's ceiling is air-gated, a rocket's is not). */
    const glm::dvec3 com = comPos();
    const int as = activeStage();
    /* ONE atmosphere read for this whole walk, because this function runs every
       substep and both halves below ask the sky: the jet ceilings in the TVC loop
       (partThrustCeiling is air-gated, #630) and the surface rows after it (q, Mach).
       airDensityAtCom() would pay for the profile twice; here the profile -- atm/alt,
       not just the density, because the fin rows need the speed of sound too -- is
       asked at most once and shared. (#594 is the ticket that stops the walk itself
       from running several times a substep.) */
    DragAtmosphere atm; double alt = 0.0;
    bool airAsked = false, airOk = false;
    const auto airProfile = [&]() {
        if(!airAsked) { airAsked = true; airOk = comAirProfile(m_parent, com, atm, alt); }
        return airOk;
    };
    const auto airRho = [&]() {
        return airProfile() ? airDensity(atm, alt) : 0.0;
    };
    const double oxShare = (m_parent != nullptr)
                         ? m_parent->surface.atmosphere.oxidizer_share : 0.0;
    bool anySurface = false;
    for(const Part *p : parts) {
        if(p->def->isControlSurface()) { anySurface = true; }
        if(p->def->gimbal_max <= 0.0) { continue; }
        const double armed = p->armedThrust;
        const bool lit = p->isThruster() && p->stage >= as;
        if(armed == 0.0 && !lit) { continue; }
        /* sin() of the limit is a capability only while the limit is under 90 deg;
           past that the exhaust points forward of the mount and the lateral-force
           fraction FALLS as the authored range grows, so a 120 deg nozzle would be
           credited as weaker than a 60 deg one -- and past 180 sin() is negative,
           which is how a live-wheeled ship ends up with `authMax <= 0`, the laws'
           "this ship cannot turn, do not pretend" early-out. The loader refuses such
           a part, so this is an invariant, and it is asserted rather than clamped:
           clamping the sum would let one silly def hide in a mixed fleet's budget as
           a positive-but-wrong number instead of a loud failure. */
        assert(p->def->gimbal_max <= std::numbers::pi_v<double> * 0.5);
        const double lever = std::fabs(glm::dot(partPos(p) - com, a.stackAxis));
        if(lever < 1e-6) { continue; }
        const double s = std::sin(p->def->gimbal_max);
        if(armed != 0.0) {
            a.heffMag   += armed * lever;
            a.tvcTorque += armed * s * lever;
        }
        if(lit) {
            /* Air is asked for only when a JET is in the loop: a rocket's ceiling does
               not read the sky, and this walk runs every substep (#594). */
            const double rho = p->isJet() ? airRho() : 0.0;
            /* The same per-engine ceiling availableThrust() sums, so the pair cannot
               drift: tvcTorqueAvailable is that number seen through the gimbal lever. */
            a.tvcTorqueAvailable += partThrustCeiling(p, rho, oxShare)
                                  * exhaust_scale * s * lever;
        }
    }
    /* #426: the SURFACE rows, spent by actuateAttitude -- see AttitudeAuthority::
       surfaces for what the mix does and what a setpoint means. q is the COM's, and
       only worth asking for when the hull actually carries a steerable surface -- the
       TVC loop above already walks every part, so it carries that answer too and a
       wheel-only hull pays nothing for the atmosphere here.
       The push direction is the part's own mount normal (partAxis(p,1)), NOT the live
       liftDir -- the rigging-is-a-build-fact rule of #495, which is also what keeps
       this a per-build constant rather than a per-attitude question. The `about` axis
       and targetSign are the part's authored steering axis, so the row's SENSE is the
       command's sense; that is what lets applyAeroForce feed the setpoint straight back
       into controlDeflection without a second sign decision, and it is what kills a
       surface whose push decides nothing about the axis it is rigged for (#495).
       The frame is attitudeRef()'s basis, the same one the stick, the laws and the
       mixer speak (#571); one partRot for the whole set. */
    /* drag_cd is the aero master switch (--drag-cd 0), and applyAeroForce returns
       before its control-surface block when it is off. The profile read answers the
       AUTHORED shell and does not know about the switch, so asking it is not enough:
       without this the budget quotes kN m of fin, actuateAttitude hands the plates
       their setpoints and subtracts their share from what it applies, and no force
       ever arrives -- the hull then gets less than it was promised for a reason no
       readout says. Same lesson as #418's power gate: the budget must answer what the
       actuators will really be able to spend. */
    if(anySurface && drag_cd > 0.0) {
        const glm::dvec3 vrel = airRelativeVel(com);
        const double v2 = glm::dot(vrel, vrel);
        const double rho = airRho();   // the same profile the jet ceilings above asked for
        const double q = 0.5 * rho * v2;
        if(q > 0.0) {
            /* #543 A5: the authority a fin has is the AUTHORED slope corrected for
               the Mach it is actually living at -- the same liftSlopeMach() the lift
               and hinge forces spend, so a hull's aerodynamic rate damping cannot
               disagree with the force those same plates produce mid-flight. Same
               profile read as the density above, so the two cannot straddle a level
               boundary. airA > 0 wherever rho > 0: the loader gates g, gamma and every
               layer's T above zero (atmo.h's invariants), and #632's airless and
               above-the-top reads are the rho 0 branch. */
            const double airA = airSoundSpeed(atm, alt);
            assert(airA > 0.0);
            const double mach = std::sqrt(v2) / airA;
            const glm::dmat3 basis = partRot(ref);   // [0] right, [1] up, [2] nose
            for(const Part *p : parts) {
                if(p->def == nullptr || !p->def->isControlSurface()) { continue; }
                const ControlAxisParams ax = controlAxisParams(p->def->control_axis);
                /* Every PartDef carries a ControlAxis (the loader rejects anything else
                   and the default is pitch), so the index below cannot go out of range --
                   asserted rather than clamped, because a surface with no steering axis
                   would have no command sense and so no row at all. */
                assert(ax.aboutAxis >= 0 && ax.aboutAxis < 3);
                const glm::dvec3 eff = surfaceTorquePerRadian(
                    q, p->def->control_area,
                    liftSlopeMach(controlCl(p->def->cl, p->def->cl_control), mach),
                    partPos(p) - com, partAxis(p, 1),
                    basis[(size_t)ax.aboutAxis], ax.targetSign);
                if(glm::dot(eff, eff) <= 0.0) { continue; }  // degenerate rig (#495)
                a.surfaces.push_back({ eff, p->def->max_deflection });
                a.surfaceParts.push_back(p);
            }
        }
    }
    return a;
}

/* The one directional sum both authority reads share: actuator rows per line
   (actuators.h) plus the TVC disk projected on the ask. `tvc` is the only thing that
   differs between them -- what the ARMED engines lever this substep, or what the
   IGNITED stage could lever at full throttle (#608). Keeping one copy is what stops
   the budget a law plans against and the torque the actuators deliver from
   disagreeing (#576, #426). */
static double torqueAboutRows(const std::vector<ActuatorRow> &wheels,
                              const std::vector<ActuatorRow> &surfaces,
                              const glm::dvec3 &stackAxis, double tvc,
                              const glm::dvec3 &axis) {
    const double ax2 = glm::dot(axis, axis);
    if(!(ax2 > 0.0)) { return 0.0; }   // a zero or NaN axis buys nothing
    /* sin of the angle between `axis` and the stack axis = |z x axis| / |axis| for
       the unit z attitudeAuthority() stores. Clamped at 1 because a near-perpendicular
       axis can round a hair over |axis|. The `> 0.0` test is also the NaN guard:
       std::fmin would hand a NaN stack axis the FULL 1.0, i.e. grant TVC about every
       axis at once, whereas this buys nothing. (A NaN `axis` is caught above.) */
    const double sinToStack = glm::length(glm::cross(stackAxis, axis)) / std::sqrt(ax2);
    const double b = (sinToStack > 0.0) ? std::fmin(1.0, sinToStack) : 0.0;
    /* The wheels' and the surfaces' halves ask actuators.h the same question
       actuateAttitude will answer with the same rows, so a hull with no wheel at all
       still has attitude authority in air (it had none to name before #426), and a
       latched law can fly a plane. */
    return actuatorTorqueAbout(wheels, axis)
         + actuatorTorqueAbout(surfaces, axis) + tvc * b;
}

double Vehicle::attitudeTorqueAbout(const AttitudeAuthority &a,
                                    const glm::dvec3 &axis) const {
    return torqueAboutRows(a.wheels, a.surfaces, a.stackAxis, a.tvcTorque, axis);
}

double Vehicle::availableTurnTorque(const AttitudeAuthority &a,
                                    const glm::dvec3 &axis) const {
    return torqueAboutRows(a.wheels, a.surfaces, a.stackAxis, a.tvcTorqueAvailable, axis);
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
    /* A law named this aim (#606), so it needs no frame dance: it is handed in in the
       same basis the rest of the attitude path uses. Empty or NaN is not filtered here
       -- pointingTo() owns that door, and the trace shows `no-target` rather than
       pretending the ship has a heading. */
    if(slew == SlewFree) { return slewFreeDir_; }
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
       orbit's normal. Then back into this frame -- slewDemand steers in it.
       Matches the drawn markers (render.cpp), so what SAS aims at is what the
       prograde icon shows. */
    Frame *inertial = frame->getNonRotFrame();
    glm::dvec3 p, v;
    comStateIn(inertial, p, v);
    const glm::dmat3 to_frame = glm::transpose(frame->GetOrientRelTo(inertial));
    glm::dvec3 dir = to_frame * (nrm ? glm::cross(p, v) : v);
    return (slew == SlewRetrograde || slew == SlewAntiNormal) ? -dir : dir;
}

/* Route one commanded attitude torque through the available actuators. The
   budget is the caller's AttitudeAuthority; the split and the solve bound it, so
   the delivered total never exceeds what the hull's own actuators can carry
   however hard the law or the stick demanded. (tests/test_slew3d pins the bound
   on the LAW, re-implemented in the test -- it cannot see this function;
   tests/test_authority.cpp is what pins the split and the solve.) Note the
   asymmetry this enforces and attitudeTorqueAbout states: the stack-axis (roll)
   part of the command can only ever reach the wheels. */
void Vehicle::actuateAttitude(glm::dvec3 torque, const AttitudeAuthority &a) {
    glm::dvec3 tauW, lat;
    /* The split does NOT cap the wheel half. A scalar cap preserves the remainder's
       direction, so on a summed demand one saturated line spends the idle ones: with
       a latched kill-rot eating the pitch budget, a pilot's held roll key got 2.29 N m
       of its 200 N m drum. actuatorSolve below is the bound that knows which drums
       cover which axis, so it caps each LINE against its own drums instead. */
    tvcSplit(torque, a.stackAxis, a.tvcTorque, a.heffMag, tauW, lat);
    /* Exactly one call per substep, so the aim below is an assignment and not a
       merge: applyRotationForce clears it and answers one summed demand (#420). Count
       the calls rather than test the aim is still zero -- a pure-roll demand arms no
       aim, so the aim test passes for a second call that should still be illegal, and
       it would go quietly wrong the day the fins (#426) or RCS rows (#595) arrive. */
    assert(++tvcCalls_ <= 1);
    tvcLat_ = lat;
    /* The demand half of the #599 record. What each CHANNEL spent is recorded where it
       is spent -- ApplyTorque below, applyThrustForce and applyAeroForce -- not here,
       because the split and the solve only ROUTE a share: the mixer's per-engine clamp,
       the gimbal rate and the air can all land short of what this function asked for,
       and a readout that quotes the routing says the rating over again (#608). */
    attDel_.asked = torque;
    /* ONE solve over every row the hull can spend this substep: the drums, then the
       control surfaces (#426). They share one vector on purpose. Two solves would
       double-spend a line a drum and a fin both cover -- which is the shape #420's
       double-bill lived in -- and the mix is not a mode you can switch a hull into:
       a plate at 45 deg clock levers pitch and yaw whatever the hull also carries.
       A wheel and a fin on one line split it pro-rata by rating, so in cruise (a fin
       row is kN m per rad) a 100 N m drum is a rounding error on the same line, which
       is the honest ratio between what the air and the drivetrain can do.
       The wheel rows come first so the setpoint indices below are stable. */
    std::vector<ActuatorRow> rows;
    rows.reserve(a.wheels.size() + a.surfaces.size());
    rows.insert(rows.end(), a.wheels.begin(), a.wheels.end());
    rows.insert(rows.end(), a.surfaces.begin(), a.surfaces.end());
    std::vector<double> setpoints;
    const glm::dvec3 delivered = actuatorSolve(rows, tauW, setpoints);
    /* A FIN'S SHARE NEVER TRAVELS THROUGH THIS CALL. actuatorSolve answers with a
       torque, and for a drum that is what actually happens to the hull -- but a
       surface steers by pushing on the AIR, and applyAeroForce bills that force from
       the deflection it is about to be handed below. Applying the abstract torque as
       well would charge the hull twice for one fin: once because the solver said it
       could, once because the air agrees. So subtract the surface rows' contribution
       and hand their setpoints to the air instead (surfaceLat_, read this same
       substep -- the tick runs control before aero for exactly this reason).
       With no surface rows the subtraction is zero, so a wheel-only hull gets the
       same bits it got before #426. */
    assert(a.surfaceParts.size() == a.surfaces.size());
    glm::dvec3 aeroShare(0.0);
    for(size_t k = 0; k < a.surfaces.size(); k++) {
        const double rad = setpoints[a.wheels.size() + k];
        /* The solver bounds each row by its own limit; the slack is the divide by the
           row's length. A setpoint past max_deflection would be clamped away silently
           in applyAeroForce, so say so here where it is the solver's fault. */
        assert(std::fabs(rad) <= a.surfaces[k].limit * (1.0 + 1e-9));
        aeroShare += rad * a.surfaces[k].axis;
        if(rad != 0.0) { surfaceLat_.push_back({a.surfaceParts[k], rad}); }
    }
    /* WHAT THE DRUMS COST FOR THIS SUBSTEP (#648): each housing's package figure scaled
       by how hard its busiest drum was asked to work, read off the setpoints the solve
       just produced. A drum asked for 4 N m of its 4500 N m pays 4/4500 of the figure.
       That is the same split #599 made for torque -- the rating answers what the hull
       OWNS, the bill answers what it SPENT -- and three things fall out of it: an
       autopilot that has arrived (kill-rot on a hull that has stopped spinning, a slew
       that has aligned) stops eating the pool while its key stays held; a demand about a
       line no drum covers costs nothing, because drums that deliver nothing never
       worked; and a hold the gimbals are doing bills the nozzles' propellant, not the
       wheels' EC. What it does NOT move is the EC GATE, which still weighs the whole
       rating (wheelDemandWatts, #642): "could this hull ever work these drums" is a
       build question, and a verdict that tracked the demand would be the stick re-arming
       its own brown-out.
       Per HOUSING, and by its PEAK drum rather than their mean: #593 authored one
       `power_draw` per package exactly so a triad would not be billed as three drums, so
       the only honest denominator for that one figure is one drum at full demand. The
       everyday case stays exact: a held key asks the hull's whole capability about its own
       axis (applyRotationForce), so the drums on that line sit at their limits and the
       bill is the authored figure to the watt. The price of the choice, stated: a triad
       pulling three lines at once bills the same as one pulling one, so the oblique
       sqrt(3) advantage actuators.h documents arrives at no extra EC. Per-drum pricing
       would buy that, and would triple every shipped wheel's figure -- a catalog pass,
       not a solver change.
       These setpoints are also what #579's momentum store will want per wheel. Until that
       store exists there is no wheel speed to multiply, so torque is the best
       proportionality this model can state; tau*omega belongs with #579. */
    assert(a.wheelParts.size() == a.wheels.size());
    double spend = 0.0;
    int runs = 0;
    for(size_t k = 0; k < a.wheels.size(); ) {
        /* The rows are built housing by housing (attitudeAuthority), so today one
           housing's drums are one run of consecutive rows and the grouping needs no map.
           That is a fact about the builder, not about this loop, so it is asserted rather
           than trusted: a reorder or a split would otherwise silently bill one housing
           once per run. */
        const Part *owner = a.wheelParts[k];
        double peak = 0.0;
        size_t m = k;
        while(m < a.wheels.size() && a.wheelParts[m] == owner) {
            /* A wheel row's setpoint is N m along its own line and its limit is its
               rated N m, so the ratio is that drum's utilisation. The solver bounds each
               row by its own limit; clamp it rather than silently over-bill a housing
               past its authored figure, and clamp per row so a row that is not a number
               fails closed (charges nothing) instead of reaching the total. */
            double u = std::fabs(setpoints[m]) / a.wheels[m].limit;
            assert(u <= 1.0 + 1e-9);
            if(u > 1.0) { u = 1.0; }
            if(u > peak) { peak = u; }
            m++;
        }
        spend += owner->powerPackageW() * peak;
        runs++;
        k = m;
    }
    assert(runs == a.wheelHousings);
    wheelSpendWatts_ = spend;
    const glm::dvec3 mechanical = delivered - aeroShare;
    if(glm::length2(mechanical) > 0.0) { ApplyTorque(hull, mechanical); }
    /* The one channel this function can state as fact: whatever lands here is on the
       hull by the end of the substep. The fins' and nozzles' shares are NOT recorded
       here -- they are billed by the air and by the thrust pass, which is where the
       #599 record reads them. */
    attDel_.wheels = mechanical;
}

/* WHAT AIMING AT `dir` MEANS FOR THIS HULL (#400): the error, the axis that closes it,
   the hull's inertia about that axis, and the authority-limited rate the actuators could
   hold along it. The alignment law and --slew-log each used to compute this, about 25
   lines apart and with their own target tests, and the log's test was the looser one: it
   dropped a target the law was steering toward and normalized what was left of it, so E
   and the axis came out NaN and the trace read alpha=0.000 next to a ship actually
   slewing on its engines. One answer now, and guidance is its third reader.
   The ship's frame is attitudeRef() -- not whichever wheel happens to be first in the
   parts list (#571). */
Vehicle::Pointing Vehicle::pointingTo(glm::dvec3 dir, const AttitudeAuthority &auth) {
    Pointing p;
    /* `valid` has to mean what the struct says it means, because guidance is this
       function's third reader and a false `valid` is a NaN travelling into a law's
       arithmetic (#609). The old test only caught a SHORT vector: a NaN dir failed
       `length2 < 1e-12` (NaN compares false) and came back valid with a NaN angle and
       axis; a 1e300 dir OVERFLOWED length2 to inf, normalized to ~0, and read as 90 deg
       off the nose -- wrong rather than NaN, which is worse to notice. Bounded both
       ways, so the normalize below cannot under- or overflow either. */
    const double l2 = glm::length2(dir);
    if(!(l2 > 1e-12) || !(l2 < 1e24)) { return p; }
    dir = glm::normalize(dir);
    Part *ref = attitudeRef();
    if(ref == nullptr) { return p; }
    /* ONE basis solve for both columns. partAxis() re-runs frameS on every call and
       this function runs per hull per substep. */
    const glm::dmat3 basis = partRot(ref);
    p.facing = basis[2];
    p.angle = acosUnit(glm::dot(p.facing, dir));
    p.axis = glm::cross(p.facing, dir);          /* + turns the nose toward dir */
    if(glm::length2(p.axis) < 1e-12) {
        /* nose ~ opposite dir: the axis is FREE, and the convention is the hull's own
           right -- a column of the same orthonormal basis facing came from, so it is
           perpendicular to facing to fp and needs no re-orthogonalising, and the answer
           stops depending on where the hull sits in the world. The world selector this
           replaces -- fabs(facing.y) > 0.9 -- answered the same ask on the same hull
           about a different line as the hull rolled across |y| = 0.9, and 1e-6 rad of
           dither at antipodes alternated it between +Y and -Y on consecutive substeps,
           spending the same torque each time (#609). Pitching over about the hull's
           right is also what "point retrograde" means to a pilot.
           What a FREE choice must not do is land on a line nothing levers:
           attitudeTorqueAbout() is directional (#576), so a hull whose only transverse
           row is along its own up (one drum, or a yaw rudder) buys nothing about its
           right -- alphaMax 0, and slewDemand holds position for ever without saying
           so. So: right if it buys anything, otherwise up. Deliberately one-sided
           rather than "whichever buys more", which would alternate the two lines as
           the wheel budget moves.
           The question here is WHICH LINE the hull owns, not what this substep can
           pay for, so it asks the throttle-free rating (#608). Asked of the armed
           authority, a TVC-only hull's invented axis would flip between right and up
           as its throttle opened and closed -- the same per-tick alternation #609
           closed, arriving through the lever instead of through the world selector. */
        p.axis = basis[0];
        if(availableTurnTorque(auth, p.axis) <= 0.0) { p.axis = basis[1]; }
        assert(std::fabs(glm::dot(p.axis, p.facing)) < 1e-12);
    } else {
        p.axis = glm::normalize(p.axis);
    }
    const glm::dmat3 I = getInertia();
    p.Ieff = glm::dot(p.axis, I * p.axis);       /* kg m^2 about the slew axis */
    /* The rate budget ABOUT THE AXIS THIS AIM NEEDS: wheels (when live) plus whatever
       TVC can lever that way, so a braking curve and the actuator bound scale together
       and with throttle. Nothing available -> alphaMax 0, and the reader holds position:
       a law does not pretend to slew (#401's lesson).
       Note what this can never credit: `axis` is cross(facing, dir) with facing ==
       attitudeRef()'s +Z, so it is transverse to the stack axis by construction and TVC
       counts in full. A law aimed at a ROLL-axis error gets nothing from TVC here -- not
       a new limit, it is the one tvcSplit has always applied when it drops that
       component -- and aiming at a roll error is the allocator's job (#420), not the
       reader's. */
    p.alphaMax = (p.Ieff > 0.0)
        ? attitudeTorqueAbout(auth, p.axis) / p.Ieff : 0.0;
    p.valid = true;
    return p;
}

Vehicle::Pointing Vehicle::pointingTo(glm::dvec3 dir) {
    return pointingTo(dir, attitudeAuthority());
}

/* What the alignment law WANTS this substep, in N m: the braking-curve rate change
   through the hull's own inertia. It does not actuate -- applyRotationForce sums
   this with the stick's demand and asks the actuators once, because two trips
   through the allocator bill the drums twice (#420). */
glm::dvec3 Vehicle::slewDemand(glm::dvec3 dir, double h, const AttitudeAuthority &auth) {
    const Pointing p = pointingTo(dir, auth);
    if(!p.valid) { return glm::dvec3(0.0); }     /* nothing to aim at */
    const double E = p.angle;
    if(E < 1e-9) { return glm::dvec3(0.0); }     /* already aligned */
    if(p.Ieff <= 0.0 || p.alphaMax <= 0.0) { return glm::dvec3(0.0); }
    const double alpha = p.alphaMax;             /* rad/s^2, actuator-limited */
    const double w_des = std::min(std::sqrt(2.0 * alpha * E), E / (2.0 * h));
    /* Drive the FULL transverse angular velocity toward the braking-curve rate: damping
       only the slew-axis component leaves the third-axis spin alive, which is the
       sustained wobble about a prograde/retrograde target. Roll is left free. */
    const glm::dvec3 w_now = hullAngVel();
    const glm::dvec3 w_transverse = w_now - p.facing * glm::dot(w_now, p.facing);
    const glm::dvec3 dW = p.axis * w_des - w_transverse;   /* desired change in rate */
    /* The demand is bounded by the law's own alpha only through w_des; the actuators'
       bound is applied once, by the allocator, to whatever this sums with (#420). The
       tensor is read again because the demand needs the WHOLE tensor (I*dW) while the aim
       measures one scalar off it; getInertia() is three Bullet reads and no allocation,
       so that is cheaper than carrying a 3x3 through a measurement struct. */
    return getInertia() * dW / h;
}

/* What kill-rot WANTS this substep, N m: a full-tensor cancel of the hull's rate.
   Same contract as slewDemand -- it does not actuate. */
glm::dvec3 Vehicle::killRotDemand(double h, const AttitudeAuthority &auth) {
    /* No frame in this law at all: the command is the hull's own rate and its own
       inertia, and the allocator's split decides which axes the actuators can
       deliver it about. */
    const glm::dvec3 w = hullAngVel();
    if(glm::length2(w) == 0.0) { return glm::dvec3(0.0); } /* at rest: nothing to kill */
    /* Kill-rot commands I*(-w)/h, a direction it does not get to choose, so it
       asks only "is there any authority at all" and lets the split decide how
       much of the command survives about which axis (a wheel-less ship keeps no
       roll, and the split drops that component instead of pretending).
       "Any authority at all" means no ROWS at all, and since #426 the surfaces are
       rows: a glider with no drum and no nozzle has kill-rot, and the same solver
       that gives it a slew law gives it this one. Omitting `surfaces` here (as this
       line did when wheels+TVC were the whole set) silences the law completely while
       the capability model quotes tens of kN m -- slewDemand asks the question
       through attitudeTorqueAbout and never had that hole. */
    if(auth.wheels.empty() && auth.surfaces.empty()
       && auth.tvcTorque <= 0.0) { return glm::dvec3(0.0); }  /* nothing to fight with */
    const glm::dmat3 I = getInertia();
    /* Full-tensor cancel, as in slewDemand: tau = I * (-w) / h. Scaling each world axis
       by I[i][i] alone needs a world-aligned principal basis, else the under-cancel
       couples axes into a period-2 limit cycle. */
    return I * (-w) / h;
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
       the battery drives is same-body, so fpos is zero (#161).
       The position and velocity tolerances SCALE, because both sides of those
       checks are ROOT-space quantities (#489): a flat 1e-6 is a bit over one ulp
       at the Kerbin->Kerbol distance, so out there it demanded bit-exactness and
       aborted on one bit of round-off (measured pre-#483: comErr 0, velErr 0,
       hullErr 1.90736318e-06 against an ulp of 1.90734863e-06). The attitude pair
       stays absolute -- a rotation is dimensionless and its round-off floor is
       ~1e-31, nowhere near 1e-20. */
    const double posTol = continuityTol(glm::length(p0));
    const double velTol = continuityTol(glm::length(v0) + glm::length(oldVel)
                                       + glm::length(frame->root_vel)
                                       + glm::length(newFrame->root_vel));
    glm::dvec3 pOrigin; glm::dmat3 pBasis;
    fromBt(principal, pOrigin, pBasis);
    glm::dvec3 hullPos; glm::dmat3 hullRot;
    fromBt(hull->btBody->getCenterOfMassTransform(), hullPos, hullRot);
    const double comErr = glm::length(rootPosIn(newFrame, newCom) - p0);
    const double hullErr = glm::length(rootPosIn(newFrame, hullPos) - p0);
    const double velErr = glm::length(rootVelIn(newFrame, newCom, newVel) - v0);
    if(comErr > 0.0 || hullErr > 0.0 || velErr > 0.0) {
        /* Exactly zero is the normal answer -- these are algebraic identities --
           so anything else is worth seeing even when the scaled tolerance lets it
           through. It is the difference between a bisected bug and a rebuild. */
        printf("[mvf] %s %s -> %s: comErr %.9g hullErr %.9g velErr %.9g"
               " posTol %.3g velTol %.3g\n",
               displayName().c_str(), frame->name.c_str(), newFrame->name.c_str(),
               comErr, hullErr, velErr, posTol, velTol);
        fflush(stdout);
    }
    assert(comErr < posTol);
    assert(velErr < velTol);
    assert(rotDist2(att0, newFrame->root_orient * (forient * sRot)) < 1e-20);
    assert(hullErr < posTol);
    assert(rotDist2(frame->root_orient * (sRot * pBasis),
                    newFrame->root_orient * hullRot) < 1e-20);

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
    if(!atm.physical()) {
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
    /* And the #599 spend record: applyRotationForce, which clears it every substep,
       never runs on rails, so the Vessel window would keep quoting the last live
       hull-substep's steering for as long as the conic runs. The #648 bill rides along
       for the same reason -- the overlay's Active row reads it, and on rails there is no
       substep that spent anything. (wheelDrawWatts() would answer 0 for a railed hull
       anyway through actuatorsEngaged(); this is the readout that has no gate.) */
    attDel_ = AttitudeDelivery{};
    wheelSpendWatts_ = 0.0;

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

RailEnd Vehicle::railAdvance(double t, const double step) {
    if(!onRails) { return RailEndNone; }
    if(railFrozen) { return RailEndNone; }
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
    const bool has_air = atm.physical();
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

/* The EC half of a rail hop (#647). A railed hull is off tick.cpp's substep loop, the
   only other caller of powerTick, so nothing else bills it: a warped transfer used to
   bill NOTHING at all. Measured on `power_test` (298 W of rtg against 600 W of air) --
   charge pinned at 12000.0 Wh for the whole warp, then falling at the printed 301.6 W
   the moment the hull was back under physics. One hop per call: `step` is this tick's
   sim advance, so the Wh is what the substep loop would have spent.
   Bill AFTER the advance, and only while the hull is STILL on rails: a hop that ends on
   air calls leaveRails(), and the substep loop bills the same step for the hull it just
   took on, so billing here as well would charge a re-entry twice. A step of 0 is
   Game::recoverActive re-homing a hull from the UI -- no time passed, so the gate is
   not re-decided by a UI action.
   The wheels' term drops out inside powerTick: `actuatorsEngaged()` asks `mixerRuns()`,
   and there is no control pass out here, so a hull is not charged for drums it never
   turned even with a slew still engaged from before the warp. The GATE still weighs
   their cost, because that is a property of the hull rather than of this step: a plant
   that cannot pay for its drums reads UNPOWERED on rails like anywhere else, and the
   brown-out latch keeps running across the warp instead of freezing mid-cut.
   One honest limit: the gate's verdict is the charge at the hop's START, so at extreme
   warp the readout can call a hull POWERED for one hop after its pool went dry mid-hop. */
RailEnd Vehicle::railsTick(double t, const double step) {
    const RailEnd e = railAdvance(t, step);
    if(onRails && step > 0.0) { powerTick(step); }
    return e;
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
