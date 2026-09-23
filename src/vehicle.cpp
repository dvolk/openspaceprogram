// vehicle.cpp -- the ship: Vehicle's method definitions (declared in
// vehicle.h) + ship building + spawn scenarios.
#include "vehicle.h"

#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <stdexcept>

#include "system.h"    // System (spawn_vehicle resolves the home body's SOI)
#include "mesh.h"      // get_mesh
#include "texture.h"   // get_texture
#include "drag.h"      // the drag law (airDensity / dragForce)

/* Instantiate a ship def: one rigid body per part (mesh + texture from the
   catalog entry), welded parent-first in the def's construction order.
   GL is needed here (shader binding); the JSON parse/validate and the
   attach geometry (attachPose) are GL-free (shipdef.cpp). The catalog must
   outlive the ship (the partDefs point into it). */
void build_ship_structure(Vehicle *ship, const ShipDef &def, Shader *partsshader)
{
    printf("Building ship '%s' (%d parts)\n", def.name.c_str(), (int)def.parts.size());

    /* 0) partition the def's parts into physical parts (those that get a
       Body) and fuel links (virtual -- no Body, no mesh). The fuel links
       are resolved to Part* edges after the physical parts exist. The
       parent indices (set at load time) are indices into def.parts, so I
       remap them to the physical-part indices as I go. A fuel link can
       never be a parent (it is virtual), so the remap is safe. */
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

    /* Remap the controller index to the physical-part index (the controller
       is always a physical part -- a fuel link can't be a controller). */
    int controllerIdx = def.controllerIndex();
    auto cit = physIndex.find((size_t)controllerIdx);
    if(cit == physIndex.end()) {
        throw std::runtime_error("build_ship: controller is a fuel link");
    }
    const size_t n = physical.size();

    /* 1) relative poses in a canonical frame: the root at the origin, +Z =
       the stack axis. Each child is placed off its (earlier) parent by the
       shared attach geometry (shipdef.cpp): a STACK edge mates two named
       nodes (attachNodes); a SURFACE edge places the child's surface node at
       a contact point+normal on the parent (attachSurface). Both funnel
       through the one node solver. */
    std::vector<glm::dvec3> pos(n);
    std::vector<glm::dmat3> rot(n);
    pos[0] = glm::dvec3(0.0);
    rot[0] = glm::dmat3(1.0);
    for(size_t i = 1; i < n; i++) {
        const ShipPart &sp = physical[i];
        const ShipPart &pp = physical[(size_t)sp.parent];
        const glm::dvec3 &pPos = pos[(size_t)sp.parent];
        const glm::dmat3 &pRot = rot[(size_t)sp.parent];
        /* one solver for both edge kinds (and for the VAB build tree), so
           flight and the editor can never disagree; solveEdge throws on a
           missing node (validated at load too). */
        const AttachPose ap = solveEdge(pPos, pRot, *pp.def, *sp.def, sp.attach,
                                        sp.parentNode, sp.childNode,
                                        sp.contactPoint, sp.contactNormal,
                                        sp.angle, sp.roll, sp.offset);
        pos[i] = ap.childPos;
        rot[i] = ap.childRot;
    }

    for(size_t i = 0; i < n; i++) {
        const PartDef &pd = *physical[i].def;

        /* Shared assets (the get_mesh/get_texture registries): one assimp
           import + GPU upload + texture upload per part FILE, so a
           100-part ship built from 10 part types pays 10x, not 100x. */
        Mesh *mesh = get_mesh(std::string("./res/") + pd.mesh);
        Texture *tex = get_texture(std::string("./res/") + pd.texture);

        /* No rigid body and no world pose of its own: the part is a child of
           the ship's one compound body, and its pose is derived from
           pos[i]/rot[i] once the ship is placed as a whole (below). */
        Body *b = create_part_body(mesh, partsshader, tex, (float)pd.mass,
                                   resolveHullMargin(def.hull_margin, pd.hull_margin));

        Part *part = new Part;
        part->body  = b;
        part->def   = &pd;
        part->id    = physical[i].id;
        part->stage = physical[i].stage;

        /* Engine shroud (optional, see PartDef.shroud): the open-cylinder
           wrap drawn OVER this part while a part is attached below it
           (Vehicle::Draw / the VAB draw). Registry-shared like the part
           assets -- one import per shroud file. */
        if(!pd.shroud.empty()) {
            part->shroud = get_mesh(std::string("./res/") + pd.shroud);
            part->shroud_texture =
                get_texture(std::string("./res/") + pd.shroud_texture);
        }

        if(i == 0) {
            ship->setRoot(part);
        } else {
            const ShipPart &sp = physical[i];
            /* pos[i]/rot[i] are already the ship-local (S) transforms -- S is
               the root's frame, pos[0]=0/rot[0]=I, and the pad `shift` and
               the world base/orient are applied uniformly to every part, so
               they cancel in the relative pose. attachPose's geometry is
               pinned numerically by test_shipload. */
            ship->attach(part, (size_t)sp.parent, pos[i], rot[i]);
        }
    }
    /* One-shot shroud snapshot (e2e anchor + debug): each part that
       declares a shroud, and whether it is shrouded as built (a part
       attached below). The state is LIVE -- staging can drop the child
       below and the shroud goes with it; this is the built state. */
    for(size_t i = 0; i < n; i++) {
        Part *p = ship->parts[i];
        if(p->shroud == nullptr) { continue; }
        printf("[shroud] %s: %s (%s over %s)\n", p->id.c_str(),
               ship->hasChildBelow(p) ? "shrouded" : "bare",
               p->def->shroud.c_str(), p->def->mesh.c_str());
        fflush(stdout);
    }

    ship->controller = ship->parts[cit->second];

    /* 4) resolve the fuel links (from/to ids -> Part*). The ids reference
       the physical parts (a fuel link can't reference another fuel link),
       so I build an id -> Part* map and look up each endpoint. Reject link
       cycles (A->B and B->A, or longer) -- the drain model requires a DAG. */
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
        /* cycle check: build the directed graph (from -> to) and do a DFS
           for back-edges. The graph is over fuel GROUPS (not parts), so I
           collapse each endpoint to its fuelGroup first. */
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
                       const glm::dvec3 &base, const glm::dmat3 &orient)
{
    build_ship_structure(ship, def, partsshader);
    /* Seed the tanks full (init) -- a pad ship lifts off with a full load. */
    ship->init();
    /* Place it on the pad: the lowest point at the pad top, lifted by the
       collision margins (terrain 0.5 + hull 0.1) so the inflated shapes just
       touch instead of popping apart on the first solve. For orbit scenarios
       this is only staging -- spawn_vehicle repositions. */
    double lowest = 1e30;
    for(size_t i = 0; i < ship->parts.size(); i++) {
        const Part *p = ship->parts[i];
        lowest = std::min(lowest, p->localPos.z - p->def->height / 2.0);
    }
    const glm::dvec3 shift = glm::dvec3(0.0, 0.0, -lowest + 0.6);
    /* init() built the single rigid body at the origin; this puts frame S
       where the pad staging wants it -- S's origin at base + orient*shift and
       S's axes at `orient` -- and every part's world pose then follows from
       its authored local pose. One write, not one per part. */
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

/*
  Starting scenarios (chosen at the CLI on startup, see main). The pad
  scenarios are already set up in main (the ship is built on the pad); the
  orbit scenarios place the ship in a circular orbit around the home body at
  r = radius + alt_frac * (rotating-frame SOI - radius), in the equatorial
  plane (local +Z) or the polar plane (local +Y), nose prograde.

  The ellipse-* scenarios place the ship on a 10 km x 1000 km ASL orbit in
  the equatorial plane, prograde in the same sense as the circular ones
  (periapsis along world +Z), at periapsis (ell_phase 0), apoapsis (1), or
  90 deg of true anomaly - halfway by angle between the apsides (2).

  The escape scenario places the ship at the rot-orbit radius with
  esc_frac x the local escape velocity, prograde -- periapsis of a
  hyperbola, so it coasts out of the body's SOI on its own.

  As before, the ship's frame is resolved from the innermost SOI containing
  the spawn point (resolve_frame_by_soi), with the stasis-velocity correction
  so a rotating frame still yields the correct inertial orbital velocity.
  ScenarioDef lives in vehicle.h.
*/
static const ScenarioDef kScenarios[] = {
    {"pad",            true,  0.0,  false, -1, 0.0,     0.0, 0.0, 0.0},
    {"pad-polar",      true,  0.0,  true,  -1, 0.0,     0.0, 0.0, 0.0},
    {"rot-orbit",      false, 0.85, false, -1, 0.0,     0.0, 0.0, 0.0},
    {"inertial-orbit", false, 1.25, false, -1, 0.0,     0.0, 0.0, 0.0},
    {"high-orbit",     false, 5.0,  false, -1, 0.0,     0.0, 0.0, 0.0},
    {"high-polar",     false, 5.0,  true,  -1, 0.0,     0.0, 0.0, 0.0},
    {"ellipse-peri",   false, 0.0,  false,  0, 10e3, 1000e3, 0.0, 0.0},
    {"ellipse-apo",    false, 0.0,  false,  1, 10e3, 1000e3, 0.0, 0.0},
    {"ellipse-mid",    false, 0.0,  false,  2, 10e3, 1000e3, 0.0, 0.0},
    {"escape",         false, 0.85, false, -1, 0.0,     0.0, 2.0, 0.0},
    /* the absolute-radius distance ladder (see ScenarioDef): anchored to
       real astronomical distances, so a name means the same distance
       around any body. Precision test beds -- neptune is comfortably
       inside double's range (~1 mm ULP), oort is where it starts to bite
       (~0.22 m), interstellar is where it clearly breaks (~22 m). */
    {"neptune",        false, 0.0,  false, -1, 0.0,     0.0, 0.0, 4.495e12},
    {"oort",           false, 0.0,  false, -1, 0.0,     0.0, 0.0, 1.0e15},
    {"interstellar",   false, 0.0,  false, -1, 0.0,     0.0, 0.0, 1.0e17},
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
    throw std::runtime_error("fleet: unknown scenario '" + name
                             + "' (available: " + avail + ")");
}

size_t scenario_count() {
    return sizeof(kScenarios) / sizeof(kScenarios[0]);
}

const char *scenario_name_at(size_t i) {
    return kScenarios[i].name;   // caller bounds-checks against scenario_count()
}

// Orientation with the nose (local +Z) along `dir`; the roll axis is the
// coordinate axis most orthogonal to dir (never singular for a unit dir).
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

/* slot_offset (m): lateral separation for ships sharing a scenario --
   applied along the orbit binormal (perpendicular to both the radius
   vector and the velocity), so each ship's orbit stays essentially the
   same shape. 0 for a lone ship (and no-op for pad scenarios). */
void spawn_vehicle(Vehicle *ship, const ScenarioDef &sc, TerrainBody *home,
                          System &sys, double slot_offset)
{
    if(sc.on_pad) { return; } // already on the pad, set up in main

    const glm::dvec3 center = home->frame->root_pos;
    glm::dvec3 shipWorldPos, velWorld;

    if(sc.ell_phase >= 0) {
        // Elliptical orbit in the equatorial plane (world X-Z), prograde in
        // the same sense as the circular scenarios: periapsis along +Z, so
        // 90 deg along the travel direction is +X. The apsides are inertial
        // (root-frame) directions, as orbital elements should be.
        const double rp = home->radius + sc.peri_alt;
        const double ra = home->radius + sc.apo_alt;
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
        // Circular orbit around the home body: radius measured from its frame origin.
        // abs_r > 0 pins the radius to an absolute distance (the neptune /
        // oort ladder); otherwise alt_frac scales it against the SOI.
        const double r = sc.abs_r > 0.0
                       ? sc.abs_r
                       : home->radius + sc.alt_frac * (home->rot_frame->soi - home->radius);
        const glm::dvec3 rhat_local = sc.polar ? glm::dvec3(0, 1, 0) : glm::dvec3(0, 0, 1);
        shipWorldPos = center + home->frame->root_orient * (rhat_local * r);

        // Circular orbital speed (vis-viva with semi-major axis == r); the
        // escape scenario reuses the radius but leaves at esc_frac x the
        // local escape velocity, so the ship is on a hyperbola (periapsis
        // at r) and coasts out of the body's SOI on its own.
        const double speed = sc.esc_frac > 0.0
                           ? sc.esc_frac * sqrt(2.0 * home->mu / r)
                           : sqrt(home->mu / r);

        // Prograde: perpendicular to the radius vector, in the system's sense of
        // rotation (+y axis); polar orbits go around the spin axis instead.
        // Normalize: with an inclined body orbit rhat is not orthogonal to
        // the reference axis, and the raw cross product is short by
        // cos(incl) -- the spawn would arrive below circular speed, at the
        // apoapsis of an e = sin^2(incl) ellipse.
        const glm::dvec3 rhat = glm::normalize(shipWorldPos - center);
        const glm::dvec3 vhat = glm::normalize(
            sc.polar ? glm::cross(glm::dvec3(1, 0, 0), rhat)
                     : glm::cross(glm::dvec3(0, 1, 0), rhat));
        velWorld = speed * vhat;
    }

    if(slot_offset != 0.0) {
        const glm::dvec3 rhat = glm::normalize(shipWorldPos - center);
        const glm::dvec3 vhat = glm::normalize(velWorld);
        shipWorldPos += glm::normalize(glm::cross(rhat, vhat)) * slot_offset;
    }

    Frame *frame = resolve_frame_by_soi(sys.root->frame, shipWorldPos);

    // Express the spawn position and velocity in the resolved frame's local
    // coordinates. The invariant (see frame.h) is
    //   R * (vel + stasis(p)) + root_vel == root-frame velocity,
    // so  vel = R^T * (velRoot - root_vel) - stasis(p). velWorld above is the
    // ship's velocity RELATIVE to home, in root-frame axes; the true
    // root-frame velocity is that plus home's own root velocity (nonzero now
    // that home orbits on a Kepler rail). When the resolved frame IS home's,
    // the two root velocities cancel and this reduces to the old R^T*velWorld.
    const glm::dvec3 target = glm::transpose(frame->root_orient) * (shipWorldPos - frame->root_pos);
    const glm::dvec3 vel = glm::transpose(frame->root_orient)
                          * (velWorld + home->frame->root_vel - frame->root_vel)
                          - frame->GetStasisVelocity(target);

    if(frame != ship->frame) {
        ship->moveToFrame(frame);
    }

    // Nose (local +Z) along prograde: rigidly re-orient the whole ship. One
    // body, so this is one pose write -- the COM goes to `target` and frame
    // S's axes to `orient`. Every part's RELATIVE geometry survives by
    // construction (a stacked part stays stacked, a radial part keeps its
    // perpendicular axis) because the parts are rigidly embedded in the
    // compound.
    const glm::dmat3 orient = faceAlong(velWorld);
    ship->placeShipAtCom(target, orient);
    ship->setVelocity(vel);

    printf("Spawn '%s' around %s: frame '%s' @ world (%.0f, %.0f, %.0f), r = %.0f m, |v| = %.1f m/s\n",
           sc.name, home->name.c_str(), frame->name.c_str(),
           shipWorldPos.x, shipWorldPos.y, shipWorldPos.z,
           glm::length(shipWorldPos - center), glm::length(velWorld));
}

/* --spin-log: the ship's rotational state. A ship is ONE rigid body, so
   there is a single angular velocity and nothing internal to compare it
   against -- which is the point of the representation. The old per-part
   spread, and the inter-part contact torque that drove it, measured how far
   the welds were from holding a rigid body; that error no longer exists, and
   Bullet generates no contacts at all between the children of a compound.

   What is left that can still spin a passive ship is the tidal
   (differential-gravity) torque -- the one legitimate external torque, and
   negligible at ship scale -- so that is reported alongside the state it acts
   on. */
void spin_log(Vehicle *ship, double time) {
    if(ship->hull == nullptr) { return; }

    const glm::dvec3 com = ship->get_center_of_mass();
    const glm::dvec3 v = GetVelocity(ship->hull);
    const glm::dvec3 w = GetAngVelocity(ship->hull);
    printf("[spin] t=%.2fs ship=%s parts=%zu mass=%.1f kg com=[%.1f %.1f %.1f]"
           " |v|=%.3f m/s w=[%.3e %.3e %.3e] |w|=%.3e rad/s\n",
           time, ship->name.c_str(), ship->parts.size(), ship->hull->mass,
           com.x, com.y, com.z, glm::length(v),
           w.x, w.y, w.z, glm::length(w));

    const double G = 6.674e-11;
    const double M = ship->m_parent->mass;
    glm::dvec3 tau(0, 0, 0);
    for(size_t i = 0; i < ship->parts.size(); i++) {
        const glm::dvec3 p = ship->partPos(ship->parts[i]);
        const double r = glm::length(p);
        /* phase 3: effectiveMass -- same mass basis as applyGravity. */
        const glm::dvec3 F = -G * M * ship->parts[i]->effectiveMass() * p / (r * r * r);
        tau += glm::cross(p - com, F);
    }
    printf("[spin]   tidal gravity torque |tau|=%.3e\n", glm::length(tau));
    fflush(stdout);
}

/* --- Vehicle method definitions (moved out of vehicle.h, same order) --- */

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
    /* `principal` moves when the masses do, and the body's transform is
       the COM transform, so leaving the transform alone across a rebuild
       would shift the ship by the COM movement and leave it spinning
       about the wrong point. */
    const bool have = (hull != nullptr && hull->btBody != nullptr);
    /* Out of the world before the delete, back in after: the rebuild
       replaces the rigid body, and a registered one cannot just be
       freed. */
    const bool wasInWorld = hullInWorld();
    if(wasInWorld) { RemoveBody(hull); }
    glm::dvec3 sPos(0.0), vCom(0.0), omega(0.0), oldOrigin(0.0);
    glm::dmat3 sRot(1.0);
    /* a rebuild makes a new rigid body, so the state that lives on the
       OLD one and is not part of the compound has to be carried over
       explicitly: the friction. An EVA kerbal's feet are set
       frictionless once, at spawn, and a burn must not undo it. (The
       never-sleep flag needs no carrying -- every ship has it, so it is
       re-asserted unconditionally below.) */
    btScalar friction = 4.0;          // RegisterObject's value
    if(have) {
        frameS(sPos, sRot);
        vCom = GetVelocity(hull);
        omega = GetAngVelocity(hull);
        glm::dmat3 ignore;
        fromBt(principal, oldOrigin, ignore);
        friction = hull->btBody->getFriction();
    }

    delete hull; hull = nullptr;
    compoundParts.clear();
    principal = btTransform::getIdentity();
    if(parts.empty()) { aeroHull.clear(); return; }

    /* ownership back-pointers (part.h): the parts list is the ownership list,
       so every part in it points back at this vehicle. The attach primitives
       (setRoot/attach) and the merge/split set this as parts move; wiring it
       here too keeps the invariant well-defined for ships assembled without
       them (the headless tests build parts by hand), so the assert below
       passes on a correctly-owned ship and fails where ownership is broken. */
    for(Part *p : parts) { p->owner = this; }

    btCompoundShape *inS = new btCompoundShape(true, (int)parts.size());
    std::vector<btScalar> masses(parts.size());
    btScalar total = 0;
    for(size_t i = 0; i < parts.size(); i++) {
        Part *p = parts[i];
        inS->addChildShape(toBt(p->localPos, p->localRot), p->body->shape);
        /* phase 3: the mass a part reports to the compound is its
           effectiveMass -- its own body mass plus whatever is parked inside
           it (the containment edge). A capsule thus carries its crew without
           the crew mass being baked into its body (addPartMass, now gone).
           The crew's mass is smeared over the capsule hull's shape (see
           checkCompoundInvariants); 3.3 refines it to a point mass. */
        masses[i] = (btScalar)p->effectiveMass();
        total += masses[i];
        compoundParts.push_back(p);
    }
    /* calculatePrincipalAxisTransform btAsserts every child mass > 0 */
    if(total <= 0) { delete inS; compoundParts.clear(); aeroHull.clear(); return; }

    btVector3 inertiaDiag(0, 0, 0);
    inS->calculatePrincipalAxisTransform(&masses[0], principal, inertiaDiag);

    /* Re-base the children into the COM/principal frame: leaving them in
       S would give a body whose origin sits at the root part while its
       inertia is diagonal about the principal axes -- an inconsistent
       body that tumbles under any off-axis torque. (Bullet's own
       CompoundBoxes tutorial writes this product the other way round;
       the FractureDemo form below is the correct one.) */
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
    ci.m_friction = friction;
    hull->btBody = new btRigidBody(ci);
    NeverSleep(hull);

    /* Restore frame S under the new principal, and the COM's velocity:
       the COM moved WITHIN S, so its world velocity changed by
       omega x the shift. omega is in world axes and is unchanged. */
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

void Vehicle::rebuildAeroHull() {
    aeroHull.clear();
    size_t cap = 0;
    for(Part *p : parts) {
        if(p->body != nullptr) { cap += p->body->hullVerts.size(); }
    }
    if(cap < 3) { return; }
    /* The union of the parts' hull verts, in frame S (their localPos/localRot
       are authored in S, so no world state is needed -- the silhouette area
       is transform-invariant anyway, see aeroHull). */
    std::vector<glm::dvec3> u;
    u.reserve(cap);
    for(Part *p : parts) {
        if(p->body == nullptr) { continue; }
        for(const glm::dvec3 &v : p->body->hullVerts) {
            u.push_back(p->localRot * v + p->localPos);
        }
    }
    /* Reduce to the extreme points with the same hull Bullet already links
       (optimizeConvexHull): the convex hull -- and so every projected
       silhouette of it -- is unchanged, but the per-substep projectedArea
       sorts tens of points instead of every part's every vert. This runs at
       most once per tick (staging/dock, or a burn's mass-drift rebuild)
       against n substeps per tick (n up to 2000 at warp). */
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

bool Vehicle::checkPartInvariants() const {
    for(Part *p : parts) {
        if(p->owner != this) {
            printf("[part] '%s': part '%s' (uid %llu) is attached to the wrong "
                   "vehicle\n", name.c_str(), p->id.c_str(),
                   (unsigned long long)p->uid);
            return false;
        }
        if(p->container != nullptr) {
            /* phases 2-4: a part in a parts list may only be contained as
               a CREW member -- its owner is the character vehicle (isEva).
               Inventory items are contained too, but they never appear in
               any parts list (the container owns them via ownedContents),
               so they are checked on the container side below. Phase 5
               drops the exception when a contained kerbal stops being a
               Vehicle (design report §2.6/§2.7). */
            if(!isEva()) {
                printf("[part] '%s': part '%s' (uid %llu) is contained, but "
                       "this vehicle is not a character\n",
                       name.c_str(), p->id.c_str(), (unsigned long long)p->uid);
                return false;
            }
            const Part *cap = p->container;
            if(cap->def == nullptr || cap->def->crew_capacity <= 0) {
                printf("[part] '%s': a character is contained in a "
                       "non-capsule part (uid %llu)\n", name.c_str(),
                       (unsigned long long)cap->uid);
                return false;
            }
            bool listed = false;
            for(Part *c : cap->contents) { if(c == p) { listed = true; break; } }
            if(!listed) {
                printf("[part] '%s': part '%s' claims container uid %llu, "
                       "but that part does not list it\n", name.c_str(),
                       p->id.c_str(), (unsigned long long)cap->uid);
                return false;
            }
        }
        /* owned inventory items must be listed for traversal too --
           ownership without traversal would free them in ~Part but silently
           drop their mass from effectiveMass (the two lists must agree). */
        for(Part *c : p->ownedContents) {
            if(c->container != p) {
                printf("[part] '%s': ownedContents holds part uid %llu, but "
                       "its container points elsewhere\n", name.c_str(),
                       (unsigned long long)c->uid);
                return false;
            }
            bool listed = false;
            for(Part *d : p->contents) { if(d == c) { listed = true; break; } }
            if(!listed) {
                printf("[part] '%s': ownedContents holds part uid %llu, but "
                       "contents does not list it\n", name.c_str(),
                       (unsigned long long)c->uid);
                return false;
            }
        }
        for(Part *c : p->contents) {
            if(c->container != p) {
                printf("[part] '%s': contents lists part uid %llu, but its "
                       "container points elsewhere\n", name.c_str(),
                       (unsigned long long)c->uid);
                return false;
            }
            /* phase 4: contents holds BOTH crew members and inventory
               items. An item is owned by the container itself (in its
               ownedContents) and claims no vehicle; everything else in
               contents must be a character's part. */
            if(c->ownedBy(p)) {
                if(p->def == nullptr || p->def->inventory_capacity <= 0) {
                    printf("[part] '%s': part uid %llu is an inventory "
                           "item, but this part is not a container\n",
                           name.c_str(), (unsigned long long)c->uid);
                    return false;
                }
                continue;
            }
            if(c->owner == nullptr || !c->owner->isEva()) {
                printf("[part] '%s': part uid %llu in contents is not a "
                       "character's\n", name.c_str(), (unsigned long long)c->uid);
                return false;
            }
        }
    }
    return true;
}

glm::dvec3 Vehicle::compoundCom() const {
    double total = 0.0;
    glm::dvec3 com(0.0);
    for(size_t i = 0; i < parts.size(); i++) {
        /* phase 3: effectiveMass (body + contained crew), matching
           rebuildCompound -- so refreshCompound's COM comparison sees the
           crew's contribution. */
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
    const double total = (double)getMass();
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
        /* phase 3: effectiveMass (body + contained crew), matching
           rebuildCompound -- the compound's mass/COM carry the crew. */
        const double m = parts[i]->effectiveMass();
        total += m;
        com += m * parts[i]->localPos;
        extent = std::max(extent, glm::length(parts[i]->localPos));
    }
    if(total <= 0.0) { return; }
    com /= total;

    /* the analytic tensor about the authored COM, in S axes. phase 3: each
       part's SHAPE carries its effectiveMass (rebuildCompound passes that
       to calculatePrincipalAxisTransform), so the shape's inertia is scaled
       from the body mass up to the effective mass (inertia is linear in mass
       for a fixed shape -- getInertiaDiag is per-kg mass times body->mass, so
       the ratio restores it at the larger mass) and the parallel-axis term
       uses the effective mass. A part with body mass but crew in it thus
       reports the crew's mass smeared over its own hull; 3.3 moves it to a
       point mass at the child's pose instead. */
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

    /* btMatrix3x3::diagonalize is a Jacobi iteration that stops once
       every off-diagonal is under 1e-5 x the diagonal trace, so the
       eigenvalues it hands back carry that much of the tensor's residue
       -- the tolerance cannot be tighter than that. The COM and the
       child poses are plain arithmetic, so they are held to rounding. */
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
               name.c_str(), comErr, comTol, iErr, iTol,
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
    /* childBelow (shipdef.h) is the shared flight/VAB test: the child is on
       p's exhaust face (below, in p's own frame) and axial, so a child
       glued to the side never counts. */
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
    // principal's origin IS the COM in S (see vehicle.h), so the COM-
    // relative pose is sRot * (localPos - pOrigin) -- small numbers only.
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
           time, name.c_str(), parts.size(), hull->mass,
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
    /* attachPose is relative, so feeding it the parent's ship-local pose
       returns the child's ship-local pose directly. */
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
    /* ::attachSurface is the free solver in shipdef.cpp (same name as this
       method -- qualify it so this isn't a recursive call). */
    const AttachPose ap = ::attachSurface(pp->localPos, pp->localRot, point,
                                          normal, *cn, rollDeg, offset);
    attach(part, parentIdx, ap.childPos, ap.childRot);
}

void Vehicle::init() {
    if(parts.empty()) { return; }
    /* propellant reservoirs: seed each tank part's resources so the
       thrusters can draw from them (they shed mass as they burn). Only
       done at construction -- extractSubtreeAsShip()
       must NOT re-seed (a stage that has been burning keeps what it has
       left). */
    for(size_t i = 0; i < parts.size(); i++) {
        Part *p = parts[i];
        if(!p->isTank()) { continue; }
        for(int r = 0; r < (int)ResourceType::Num; r++) {
            p->resources.capacity[r] = p->def->capacity[r];
            p->resources.current[r]  = p->def->capacity[r];
        }
    }
    finalize();
}

void Vehicle::finalize() {
    if(parts.empty()) { return; }
    if(controller == nullptr) { controller = parts[0]; }
    /* stage bookkeeping: totalStages_ = the highest stage number (the
       counter's start + the "stage X of N" N); minStage_ = the lowest (the
       counter's floor); activeStage_ starts at the HIGHEST stage, so the
       highest-numbered engines fire at t=0 and stage 1 fires last (a ship
       whose first engine is stage N still lifts off). Computed once here. */
    totalStages_ = 1;
    int lowest = parts[0]->stage;
    for(size_t i = 1; i < parts.size(); i++) {
        if(parts[i]->stage > totalStages_) { totalStages_ = parts[i]->stage; }
        if(parts[i]->stage < lowest) { lowest = parts[i]->stage; }
    }
    minStage_ = lowest;
    activeStage_ = totalStages_;
    /* fuel groups: an engine draws from the tanks it's connected to
       (its fuel group), not by stage -- the weld links are known now. */
    buildFuelGroups();
    /* the ship's single rigid body: the part list is complete, so the
       compound can be built, and its mass properties asserted against
       the assembly it came from. ONE body per ship, not one per part --
       and nothing to weld, because a rigid body has no internal degrees
       of freedom to constrain. Registering it is enterWorld()'s job, so
       that a headless caller (the unit tests) can build a ship with no
       physics world in existence. */
    rebuildCompound();
}

void Vehicle::enterWorld() {
    if(hull != nullptr && !hullInWorld()) { AddPhysicsBody(hull); }
}

bool Vehicle::isEva() const { return false; }

bool Vehicle::isCrewAboard() const { return false; }

Part *Vehicle::capsulePart() const { return nullptr; }

void Vehicle::buildFuelGroups() {
    // The group structure is about to change, so any cached drain layers
    // (fuelDrainLayers) are now stale -- drop them. This is the only place
    // the groups are ever rebuilt (construction, docking merges, split/undock,
    // save-load), so the cache can never outlive the structure it describes.
    drainLayers_.clear();
    for(Part *p : parts) { p->fuelGroup = -1; }
    /* undirected adjacency over the part tree (Part::parent; each
       non-root part has exactly one parent edge). */
    std::map<Part *, std::vector<Part *>> adj;
    for(Part *p : parts) {
        if(p->parent == nullptr) { continue; }
        adj[p->parent].push_back(p);
        adj[p].push_back(p->parent);
    }
    int next = 0;
    for(Part *p : parts) {
        if(p->isFuelBarrier()) { continue; }   /* a wall: stays -1 */
        if(p->fuelGroup != -1) { continue; }   /* already grouped */
        int g = next++;
        std::vector<Part *> stack;
        stack.push_back(p);
        p->fuelGroup = g;
        while(!stack.empty()) {
            Part *q = stack.back(); stack.pop_back();
            auto it = adj.find(q);
            if(it == adj.end()) { continue; }
            for(size_t i = 0; i < it->second.size(); i++) {
                Part *r = it->second[i];
                if(r->isFuelBarrier()) { continue; }   /* don't cross a wall */
                if(r->fuelGroup != -1) { continue; }   /* already grouped */
                r->fuelGroup = g;
                stack.push_back(r);
            }
        }
    }
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
    // Cached per group (drainLayers_): the layer structure is static until
    // buildFuelGroups re-runs (construction / a docking merge), so a repeat
    // call for the same group is a map lookup, not a reverse-adjacency BFS
    // plus a dozen map/vector allocs (a hot path while thrusting).
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
    /* bucket the groups by distance, then take the buckets in
       descending order (std::map is ascending, so walk it backwards).
       Within a bucket the group ids are ascending (dist is keyed by
       group id), so the order is deterministic. */
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

bool Vehicle::consumeResourceMass(enum ResourceType type, float amt, Part *engine) {
    const std::vector<std::vector<int> > &layers = fuelDrainLayers(engine);
    if(layers.empty()) { return false; }
    /* Total fuel across all source groups (every layer). */
    float total = 0;
    for(size_t li = 0; li < layers.size(); li++) {
        for(size_t gi = 0; gi < layers[li].size(); gi++) {
            for(size_t i = 0; i < parts.size(); i++) {
                Part *p = parts[i];
                if(!p->isTank()) { continue; }
                if(p->fuelGroup != layers[li][gi]) { continue; }
                total += p->resources.current[(int)type];
            }
        }
    }
    if(total < amt) { return false; }
    /* Drain layer by layer (furthest first), pro-rata across the layer's
       tanks. Single pass (no per-layer scratch vector): total >= amt means
       `remaining` can never exceed a layer's available fuel, so each tank's
       share `take * have / layerTotal` is computed and applied in one loop
       -- identical to the old collect-then-drain, minus the allocation. */
    float remaining = amt;
    for(size_t li = 0; li < layers.size() && remaining > 0.0f; li++) {
        float layerTotal = 0;
        for(size_t gi = 0; gi < layers[li].size(); gi++) {
            const int grp = layers[li][gi];
            for(Part *p : parts) {
                if(!p->isTank() || p->fuelGroup != grp) { continue; }
                if(p->resources.current[(int)type] > 0.0f) {
                    layerTotal += p->resources.current[(int)type];
                }
            }
        }
        if(layerTotal <= 0.0f) { continue; }
        float take = remaining < layerTotal ? remaining : layerTotal;
        for(size_t gi = 0; gi < layers[li].size(); gi++) {
            const int grp = layers[li][gi];
            for(Part *p : parts) {
                if(!p->isTank() || p->fuelGroup != grp) { continue; }
                float have = p->resources.current[(int)type];
                if(have <= 0.0f) { continue; }
                float share = take * have / layerTotal;
                if(share > have) { share = have; }
                p->resources.current[(int)type] = have - share;
                /* No SetMass: a part has no rigid body. The ship's mass
                   properties follow from the parts, and refreshCompound()
                   (once per tick) rebuilds them once the drift matters. */
                p->body->mass -= (double)share;
            }
        }
        remaining -= take;
    }
    return true;
}

float Vehicle::getFuelMass(const std::vector<enum ResourceType>& types) {
    float fuel = 0;
    for(auto&& type : types) {
        for(Part *p : parts) {
            fuel += p->resources.current[(int)type];
        }
    }
    return fuel;
}

float Vehicle::getDeltaV() {
    // Rocket propellant only (H2 + LOX): jet fuel is a SEPARATE type
    // (air-breathing, no onboard oxidizer) and produces no delta-v, so it
    // is correctly absent from this count. The type list is a static (not a
    // brace-init temp): getDeltaV is called every frame by the Vessel window,
    // and a fresh std::vector per call is pure churn.
    static const std::vector<ResourceType> prop =
        { ResourceType::Hydrogen, ResourceType::LOX };
    float remaining_fuel = getFuelMass(prop); /* kg */
    double ve = 0;   // first ROCKET thruster's exhaust velocity (the delta-v estimate).
                     // Jets are skipped: they are air-breathing, so they produce no
                     // thrust in vacuum and their exhaust velocity is not a delta-v.
    for(Part *p : parts) { if(p->isThruster() && !p->isJet()) { ve = p->exhaustVelocity(); break; } }
    // Tsiolkovsky: dv = ve * ln(m_fueled / m_dry). getMass() is the FUELED
    // mass (each tank's body mass is its fueled mass -- hull + propellant, and
    // drainFuel subtracts burned propellant from it), and m_dry = getMass()
    // - remaining_fuel (the empty hulls + dry parts).
    return (float)(ve * exhaust_scale)
         * log(getMass() / (getMass() - remaining_fuel));
}

float Vehicle::getMass() {
    /* phase 3: effectiveMass -- the ship's mass includes what is parked in
       it (the containment edge), so a capsule's crew counts here too. */
    float r = 0;
    for(Part *p : parts) {
        r += (float)p->effectiveMass();
    }
    return r;
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
            ecCapacity += p->resources.capacity[(int)ResourceType::EC];
        }
    }
    // gate: no EC system -> ungated (wheels work as before). Otherwise
    // the wheels need power left over for them after life support --
    // excess generation or stored charge.
    const bool hasEC = (ecCapacity > 0.0) || (totalGen > 0.0) || (constantDraw > 0.0);
    powered_ = hasEC ? ((totalGen > constantDraw) || (ecCharge > 0.0)) : true;
    // active draw: the wheels draw only while they are actually working
    // (powered AND commanding attitude) -- the same condition
    // applyRotationForce uses to apply torque.
    double activeDraw = 0.0;
    const bool wheelsActive = powered_
        && ((stick[0] != 0.0f || stick[1] != 0.0f || stick[2] != 0.0f)
            || slew != SlewNone);
    if(wheelsActive) {
        for(size_t i = 0; i < parts.size(); i++) {
            if(parts[i]->isWheel()) { activeDraw += parts[i]->powerDraw(); }
        }
    }
    // balance: generation charges, the draws drain; clamp to the pool.
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
        float have = p->resources.current[(int)ResourceType::EC];
        if(have <= 0.0f) { continue; }
        float share = (float)(take * have / total);
        if(share > have) { share = have; }
        p->resources.current[(int)ResourceType::EC] = have - share;
    }
}

void Vehicle::chargeEC(double wh) {
    if(wh <= 0.0) { return; }
    double freeCap = 0.0;
    for(Part *p : parts) {
        if(p->isBattery()) {
            freeCap += p->resources.capacity[(int)ResourceType::EC]
                     - p->resources.current[(int)ResourceType::EC];
        }
    }
    if(freeCap <= 0.0) { return; }
    double add = (wh < freeCap) ? wh : freeCap;
    for(Part *p : parts) {
        if(!p->isBattery()) { continue; }
        float cap = p->resources.capacity[(int)ResourceType::EC];
        float have = p->resources.current[(int)ResourceType::EC];
        float free = cap - have;
        if(free <= 0.0f) { continue; }
        float share = (float)(add * free / freeCap);
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
            cap += p->resources.capacity[(int)ResourceType::EC];
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
           time, name.c_str(), (int)powered_, gen, constDraw, charge, capacity);
    fflush(stdout);
}

int Vehicle::activeStage() { return activeStage_; }

void Vehicle::advanceStage() { if(activeStage_ > minStage_) { activeStage_--; } }

int Vehicle::numStages() { return totalStages_; }

float Vehicle::getThrust() {
    /* The ACTUAL thrust at the current throttle and air state. Rockets
       contribute their rated thrust; a jet contributes its air-breathing
       thrust at the current airspeed + density (drag.h jetThrust), so the
       HUD reads the real force, not a rated figure. */
    const int as = activeStage();
    const double v_air = glm::length(GetVel());
    const double rho = airDensityAtCom();
    const double rho_sea = (m_parent != nullptr)
        ? (double)m_parent->surface.atmosphere.sea_level_density : 0.0;
    double t = 0;
    for(Part *p : parts) {
        if(!p->isThruster() || p->stage < as) { continue; }
        if(p->isJet()) {
            t += jetThrust(v_air, rho, rho_sea, p->def->jet_fan_thrust,
                           p->def->fuel_rate, p->def->exhaust_velocity,
                           p->def->jet_intake_area);
        } else {
            t += p->thrust();
        }
    }
    return (float)(t * thruster_util * exhaust_scale);
}

float Vehicle::getTWR() {
    return getThrust() / (getMass() * m_parent->g);
}

float Vehicle::getFullThrustTWR() {
    return GetActiveThrust() / (getMass() * m_parent->g);
}

float Vehicle::getMaxTWR() {
    // ALL burnable propellant (rocket H2 + LOX and jet fuel): max TWR is at
    // the lightest mass, i.e. after all of it has been spent. The type list
    // is a static (getMaxTWR is called every frame by the Vessel window; a
    // brace-init std::vector per call is pure churn).
    static const std::vector<ResourceType> prop =
        { ResourceType::Hydrogen, ResourceType::LOX, ResourceType::JetFuel };
    float remaining_fuel = getFuelMass(prop); /* kg */
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
        /* phase 3: effectiveMass -- the force must carry the same mass as the
           COM and the inertia (both derived from effectiveMass), else the net
           force misses the crew parked in the capsule and acts off the true
           COM, adding a spurious dcom x F torque (the com-torque regression). */
        const double m = p->effectiveMass();
        if(m == 0) { continue; }
        const glm::dvec3 b1b2 = partPos(p);
        const double m1m2 = m * parent_mass;
        const double invrsqr = 1.0 / glm::length2(b1b2);
        const double mag = G * m1m2 * invrsqr;
        const glm::dvec3 f = mag * sqrt(invrsqr) * -b1b2;
        /* AT the part, not at the COM: ApplyForce's rel_pos is what
           delivers the differential (tidal) torque, the one legitimate
           external torque on a rigid ship. Summing the forces and applying
           them at the COM would drop it. */
        ApplyForce(hull, b1b2 - com, f);
        gf += f;
        if(frame->isRotFrame()) {
            // In rotating coordinates the ship additionally feels
            // Coriolis + centrifugal; without these its true inertial
            // orbit is perturbed for as long as it spends in the rotating
            // frame (see GetFictitiousAccel in frame.h).
            const glm::dvec3 a_fict = frame->GetFictitiousAccel(b1b2, partVel(p));
            const glm::dvec3 ff = m * a_fict;
            ApplyForce(hull, b1b2 - com, ff);
            ff_total += ff;
        }
    }
    /* The per-part levers above are referenced to the hull's transform
       origin, which lags the true COM during a burn; a net force through
       that offset adds a spurious (comOffset x F) torque the ship's true
       COM does not feel, so cancel it. (The legitimate tidal torque is
       about the true COM and is unaffected.) */
    const glm::dvec3 dcom = comOffset();
    if(glm::length2(dcom) > 0.0) {
        ApplyTorque(hull, -glm::cross(dcom, gf + ff_total));
    }
    return gf;
}

Vehicle::Vehicle() { }

Vehicle::~Vehicle() {
    // The crew aboard (Kerbals, eva.h) are owned by this ship: Vehicle::crew
    // is the sole owner (the capsule's contents list is a non-owning
    // back-reference, part.h). Delete them before the parts -- each kerbal's
    // container/contents edge points into this ship's capsule part, so the
    // parts must still be alive when the kerbal's edge is torn down.
    for(auto&& k : crew) { delete k; }
    crew.clear();
    if(hullInWorld()) { RemoveBody(hull); }
    delete hull; hull = nullptr;
    compoundParts.clear();
    for(Part *p : parts) { delete p; }   // ~Part deletes the Body
}

glm::dvec3 Vehicle::processGravity() {
    return applyGravity();
}

void Vehicle::applyThrustForce() {
    const glm::dvec3 com = comPos();
    glm::dvec3 ftotal(0.0);
    for(Part *p : parts) {
        if(!p->isThruster()) { continue; }
        if(p->armedThrust == 0.0f) { continue; }
        /* Along the engine's own +Z, applied AT the engine: an off-axis or
           tilted engine torques the ship directly, which is what the weld
           used to have to transmit. */
        const glm::dvec3 ft = partAxis(p, 2) * (double)p->armedThrust;
        ApplyForce(hull, partPos(p) - com, ft);
        ftotal += ft;
    }
    lastThrustForce = ftotal;
    if(glm::length2(ftotal) < 1e-24) { return; }
    /* Same spurious-torque cancellation as applyGravity: the thrust lever
       is referenced to the hull origin, which lags the true COM during a
       burn, so subtract the (comOffset x F) term it introduces. */
    const glm::dvec3 dcom = comOffset();
    if(glm::length2(dcom) > 0.0) {
        ApplyTorque(hull, -glm::cross(dcom, ftotal));
    }
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
    // applyAeroForce: the atmosphere is a spherically-symmetric shell, so
    // the density depends only on the distance from the body's centre.
    const double ref_radius =
        (double)m_parent->radius + (double)m_parent->surface.sea_level;
    const DragAtmosphere da { atm.sea_level_density, atm.scale_height };
    return airDensity(da, r - ref_radius);
}

glm::dvec3 Vehicle::applyAeroForce(double h) {
    (void)h;  // a force (not an impulse); Bullet integrates it over the substep
    // The aero state the --drag-log instrument prints; reset so a substep
    // with no air (no atmo / above it / on the ground) reports zero.
    lastAeroForce = glm::dvec3(0.0);
    lastLiftForce = glm::dvec3(0.0);
    lastAeroTorque = glm::dvec3(0.0);
    lastDragAlt = 0.0;
    lastDragRho = 0.0;
    lastDragAlpha = 0.0;
    lastDragArea = 0.0;
    lastDragCd = 0.0;
    lastControlDeflections.clear();  // a no-air substep reports no steering

    // --drag-cd 0 = no aero at all (the master off switch, the v1 contract).
    // The lift term is gated on it too, so "0 disables aero entirely" holds
    // even for a part that lifts.
    if(drag_cd <= 0.0) { return lastAeroForce; }

    // Only a body with a PHYSICAL atmosphere (a density model) produces
    // aero -- a limb rim alone (render) does not.
    if(m_parent == nullptr) { return lastAeroForce; }
    const AtmosphereParams &atm = m_parent->surface.atmosphere;
    if(atm.sea_level_density <= 0.0 || atm.scale_height <= 0.0) {
        return lastAeroForce;
    }

    // Altitude above SEA LEVEL -- the fixed reference radius, not the local
    // terrain. The atmosphere is a spherically-symmetric shell, so its
    // density depends only on distance from the body's centre: a ship at a
    // given altitude reads the same air whether it is over a peak or a
    // valley (measuring above the terrain would make it read denser over a
    // peak -- backwards). sea_level is 0 for a landlocked body, so this is
    // altitude above the base radius there. (|com| is the distance from the
    // centre; GetTerrainHeight is not needed.)
    const glm::dvec3 com = comPos();
    const double r = glm::length(com);
    if(r <= 0.0) { return lastAeroForce; }
    const double ref_radius =
        (double)m_parent->radius + (double)m_parent->surface.sea_level;
    const double alt = r - ref_radius;
    lastDragAlt = alt;
    if(alt <= 0.0) { return lastAeroForce; }  // at / below sea level (in the sea)

    const DragAtmosphere da { atm.sea_level_density, atm.scale_height };
    const double rho = airDensity(da, alt);
    lastDragRho = rho;
    /* Numerically above the air: below kRhoFloor (drag.h) the drag is
       unmeasurable but exp(-alt/H) stays positive for hundreds more km --
       the 500 km high-orbit perf case read rho = 4e-40 kg/m3 and F = 0.00 N,
       yet every substep still paid the whole silhouette pass. */
    if(rho < kRhoFloor) { return lastAeroForce; }

    // v_rel = the ship's velocity in its (rot) frame -- the air co-rotates
    // with the planet, so this is already air-relative (see drag.h).
    const glm::dvec3 vrel = GetVel();
    const double v2 = glm::length2(vrel);
    if(v2 <= 0.0) { return lastAeroForce; }  // at rest in air

    // Frame S read ONCE: it is constant within a substep, and every part
    // pose below derives from it. (partPos/partRot re-read the Bullet
    // transform per call, and this function used to make 3-4 such calls per
    // part per substep for the identical answer.)
    glm::dvec3 sPos; glm::dmat3 sRot;
    frameS(sPos, sRot);
    const auto partPosS = [&](const Part *p) {
        return sPos + sRot * p->localPos;
    };

    // The flow frame -- shared by every part (the ship is one rigid body).
    // The root part's local axes give the nose (+Z, sets the off-axis term),
    // right (+X) and up (+Y, the wing normal that lift acts along). A ship
    // with no root part (defensive) is treated as prograde and non-lifting.
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
    // The flow in frame S: the aero hulls live in local frames, and both the
    // silhouette area and the axis-flow dots are rotation-invariant, so
    // rotating the flow once replaces transforming any vertices at all.
    const glm::dvec3 vhatS   = glm::transpose(sRot) * vhat;
    const glm::dvec3 liftDir = liftDirection(vrel, right, nose);
    const double q = 0.5 * rho * v2;  // dynamic pressure (shared by all parts)

    glm::dvec3 ftotal(0.0);
    glm::dvec3 lift_total(0.0);
    glm::dvec3 moment(0.0);

    // DRAG (R2: silhouette area x per-part shape). The AREA is the ship's
    // convex-hull silhouette facing the flow -- the convex hull of ALL parts'
    // hull verts, projected onto the plane perpendicular to the flow (drag.h
    // projectedArea). One area for the whole ship, so a stacked rocket
    // presents its true end face (one circle), not N of them, and the
    // prograde->side swing is honest (a long body's side is N x its end).
    // The COEFFICIENT is each part's 3-anchor cd (drag.h partCd: its
    // forward/side/backward anchors blended by the angle its nose axis makes
    // with the flow) -- so a cone is sleek nose-first and blunt base-first,
    // the silhouette alone can't say -- area-weighted by how much area each
    // shows to the flow (a blunt heat shield raises it, a sleek nose lowers it).
    // drag_cd is the global master scale (--drag-cd; 0 = off, handled above).
    // Applied at the center of pressure (the parts' centroid, weighted by
    // each part's projected area -- NOT the silhouette polygon's centroid),
    // so a banked ship still weathervanes the nose into the flow (the moment
    // about the COM). `com` is the hull origin the lever is measured from.
    {
        glm::dvec3 cp(0.0);      // center of pressure (area-weighted centroid)
        double cpArea = 0.0;
        double cdNum = 0.0;      // sum of (partArea x part cd) for the cd mean
        for(Part *p : parts) {
            if(p->body == nullptr || p->body->hullVerts.empty()) { continue; }
            // The part's own silhouette facing the flow: the weight it shows
            // in the center of pressure AND in the area-weighted cd mean.
            // hullVerts are part-local, so the flow is rotated to the part
            // (vhatS is already in S) instead of the verts to the world.
            const double a = projectedArea(p->body->hullVerts,
                                           glm::transpose(p->localRot) * vhatS);
            cp += a * partPosS(p);
            cpArea += a;
            // The part's cd AS IT FACES THE FLOW (drag.h partCd): the angle
            // between the part's nose axis and the flow selects the forward /
            // side / backward anchor (a cone is sleek nose-first, blunt
            // base-first; a thin disc is blunt face-on, sleek edge-on). A
            // part with only the shared `drag` set is symmetric. localRot[2]
            // is the part's nose in S; sRot preserves dots, so this is the
            // world-frame dot(partAxis(p,2), vhat) without building either.
            if(p->def != nullptr) {
                const double c = glm::dot(p->localRot[2], vhatS);
                const double cd = partCd(p->def->drag_forward,
                                         p->def->drag_side,
                                         p->def->drag_backward, c);
                if(cd > 0.0) { cdNum += a * cd; }
            }
        }
        // The ship's silhouette from the precomputed union hull (aeroHull,
        // frame S, rebuilt with the compound): the same convex hull the
        // per-substep union built, but only its extreme points, and only
        // v̂ rotates -- no vertex touches the world frame any more.
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

    // LIFT (per part): each lifting surface generates lift on its OWN area at
    // its own position, so the moment (pitch/yaw stability) comes from the
    // surfaces' distribution (a wing ahead pitches one way, behind the other).
    // 0 for a part with no lift_area / cl (a rocket stays a rocket); the soft
    // stall collapses it past the part's stall_angle.
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

    // Control surfaces (deflection-driven steering): each surface steers ONE
    // axis (its control_axis -- elevator pitches, rudder yaws, aileron rolls)
    // and is driven by that axis's stick alone. The force is the lift law with
    // the deflection in place of the AoA (controlForce) -- linear, bounded by
    // the travel -- applied at the surface, so its OFFSET from the COM is the
    // steering leverage (a tail pitches/yaws the ship, a canard ahead the
    // other way, a laterally-offset pair rolls). Zero in vacuum (q = 0) and
    // at rest (returned above).
    {
        lastControlDeflections.clear();
        // Force directions, out of the flow (mirrors liftDirection): pitch
        // and roll share the "up" plane; yaw uses the ship's right axis. The
        // axis picks the plane AND the stick that drives the surface.
        const glm::dvec3 yawDirRaw = right - glm::dot(right, vhat) * vhat;
        const double yawLen = glm::length(yawDirRaw);
        const glm::dvec3 yawDir = (yawLen > 0.0) ? yawDirRaw / yawLen
                                                 : glm::dvec3(0.0);
        int ctrlIndex = 0;  // the Nth control surface (disambiguates instances)
        for(Part *p : parts) {
            if(p->def == nullptr) { continue; }
            const PartDef *d = p->def;
            if(d->control_area <= 0.0 || d->max_deflection <= 0.0) { continue; }
            // Deflection effectiveness (per radian): the part's dedicated
            // cl_control if it is set (>0), else its lift-curve slope cl
            // (controlCl) -- so a part that only declares cl (no cl_control)
            // keeps the old single-"cl" behaviour.
            const double clc = controlCl(d->cl, d->cl_control);
            // The axis -> (moment axis, force plane, stick, target sign)
            // selection is the PURE controlAxisParams (pinned in
            // test_shipload); here it resolves to the ship's concrete axes.
            const ControlAxisParams ax = controlAxisParams(d->control_axis);
            const float sv = stick[ax.stickIndex];   // 0 when that stick is free
            const glm::dvec3 forceDir = (ax.forceDirKind == 0) ? liftDir : yawDir;
            const glm::dvec3 about =
                (ax.aboutAxis == 0) ? right : (ax.aboutAxis == 1) ? up : nose;
            const glm::dvec3 ri = partPosS(p) - com;
            // The deflection sign is POSITION-DEPENDENT: the steering torque
            // is ri x F, so a tail (behind the CG) and a canard (ahead) need
            // OPPOSITE deflections for the same steering torque, and a
            // laterally-offset pair (an aileron) deflects opposite to roll.
            // controlDeflectionSign picks the sign so the moment about the
            // axis matches the reaction wheel for EITHER position (the B1
            // fix), consistent with applyRotationForce.
            const double sign =
                controlDeflectionSign(ri, forceDir, about, ax.targetSign);
            const double deflection = sign * (double)sv * d->max_deflection;
            // Record the applied deflection for the --drag-log telemetry (0
            // when that stick is released -- the pilot is not steering it).
            // Stored by part reference (no per-substep string copies); the
            // name is resolved when the log prints it.
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

    // Same spurious-torque cancellation as applyThrustForce / applyGravity:
    // the lever is referenced to the hull origin, which lags the true COM
    // during a burn, so subtract the (comOffset × F) term it introduces.
    const glm::dvec3 dcom = comOffset();
    if(glm::length2(dcom) > 0.0) {
        ApplyTorque(hull, -glm::cross(dcom, ftotal));
    }
    return lastAeroForce;
}

void Vehicle::applyControlForces(double h) {
    applyThrustForce();
    applyRotationForce(h);
    applyRcsForce(h);
}

Part * Vehicle::firstWheel() {
    for(Part *p : parts) { if(p->isWheel()) { return p; } }
    return nullptr;
}

void Vehicle::applyRotationForce(double h) {
    if(firstWheel() == nullptr) { return; }
    /* Power gate: the reaction wheels are electric -- with no power the
       ship is uncontrolled (no manual stick AND no autopilot slew).
       powered_ is set by powerTick this substep (a ship with no EC
       system is ungated, so this is a no-op for them). */
    if(!powered_) { return; }
    /* Manual stick: standard aviation mapping, body-relative.
       Pitch (W/S) about the ship's right axis, yaw (A/D) about its
       up axis, roll (Q/E) about the nose. The camera tracks the
       ship's attitude, so these read consistently on screen
       regardless of the ship's world orientation. Each wheel gets
       its rated torque along the combined axis; diagonals (W+A)
       compose as a vector sum. */
    if(stick[0] != 0.0f || stick[1] != 0.0f || stick[2] != 0.0f) {
        Part *rw0 = firstWheel();
        const glm::dvec3 pitchAxis = -partAxis(rw0, 0);  // right (W/S)
        const glm::dvec3 yawAxis   = -partAxis(rw0, 1);  // up    (A/D)
        const glm::dvec3 rollAxis  =  partAxis(rw0, 2);  // nose  (Q/E)
        const glm::dvec3 worldAxis =
            (double)stick[0] * rollAxis
            + (double)stick[1] * pitchAxis
            + (double)stick[2] * yawAxis;
        /* Every wheel turns the same rigid body along the same axis, so
           their torques simply sum -- and maxTorque() is that sum. */
        ApplyTorque(hull, maxTorque() * worldAxis);
    }
    /* Autopilot: one authority-bounded step of the slew/kill-rot law
       (h = this substep's duration, so the law re-evaluates per
       substep -- the stable form of the same law). Every directional
       mode slews the nose toward its target direction (slewTargetDir);
       kill-rot damps the spin directly instead of chasing a direction. */
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

void Vehicle::applyRcsForce(double h) {
    if(glm::length2(rcsDir) < 1e-12) { return; }
    Part *e = firstRcsPart();
    if(e == nullptr) { return; }
    const double F = maxRcsThrust();
    if(F <= 0.0) { return; }
    /* flow this substep (kg) = thrust / (Isp * g0) * h, the same
       monoprop Isp the EVA suit uses (src/eva.cpp kRcsIsp). */
    const double flow = (F / (kRcsIsp * 9.81)) * h;
    if(consumeResourceMass(ResourceType::Hydrazine, (float)flow, e)) {
        ApplyCentralForce(hull, F * rcsWorldDir());
        rcsFiring = true;
    }
}

void Vehicle::slew_log(double time) {
    if(slew == SlewNone) { return; }
    Part *wheel = firstWheel();
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
    const double alpha = (Ieff > 0.0) ? maxTorque() / Ieff : 0.0;
    const double w_des = (alpha > 0.0) ? std::sqrt(2.0 * alpha * E) : 0.0;
    printf("[slew] t=%.3f mode=%s E=%.4f rad (%.2f deg) "
           "w_slew=%+.4f w_roll=%+.4f w_third=%+.4f |w|=%.4f "
           "alpha=%.3f Ieff=%.0f w_des=%+.4f "
           "nose=[%+.3f %+.3f %+.3f] target=[%+.3f %+.3f %+.3f]\n",
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
    /* phase 3: effectiveMass -- this probe re-derives the SAME force that
       applyGravity applies, so it must use the same mass basis, or it
       under-reports |F| and |dcom x F| on a crewed ship. */
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
            F += partAxis(p, 2) * (double)p->armedThrust;
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

void Vehicle::fuel_log(double time) {
    /* distinct group ids, ascending (stable order across ticks). */
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
    /* ONE line per sample (e2e-greppable): each group's current /
       capacity per resource, with the member tanks' own contents in
       brackets (parts order -- a pro-rata drain keeps them equal),
       then the fuel links as group pairs. */
    printf("[fuel] t=%.3fs ship=\"%s\"", time, name.c_str());
    for(size_t gi = 0; gi < groups.size(); gi++) {
        const int g = groups[gi];
        /* the resources this group carries (any member tank has
           capacity > 0), printed in resource order. */
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
            float cur = 0.0f, cap = 0.0f;
            std::vector<float> tankCur;
            for(size_t i = 0; i < parts.size(); i++) {
                Part *p = parts[i];
                if(p->fuelGroup != g || !p->isTank()) { continue; }
                if(p->def->capacity[r] <= 0.0f) { continue; }
                cur += p->resources.current[r];
                cap += p->resources.capacity[r];
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
    /* the fuel links, collapsed to group ids (the same rule
       fuelDrainLayers applies: skip barrier endpoints and self-links). */
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

void Vehicle::drain_log(double time) {
    // Each group's total fuel mass now, in one pass over the parts.
    std::map<int, double> cur;
    double thrust = 0.0;
    for(size_t i = 0; i < parts.size(); i++) {
        Part *p = parts[i];
        thrust += (double)p->armedThrust;
        if(p->fuelGroup < 0 || !p->isTank()) { continue; }
        for(int r = 0; r < (int)ResourceType::Num; r++) {
            if(p->resources.current[r] <= 0.0f) { continue; }
            cur[p->fuelGroup] += p->resources.current[r];
        }
    }
    if(drainPrevTime_ > 0.0 && time > drainPrevTime_) {
        const double dt = time - drainPrevTime_;
        printf("[drainlog] t=%.3fs dt=%.3fs ship=\"%s\" thrust=%.1fN",
               time, dt, name.c_str(), thrust);
        for(std::map<int, double>::iterator it = cur.begin(); it != cur.end(); ++it) {
            const int g = it->first;
            const double mass = it->second;
            const double prev = drainPrevMass_.count(g) ? drainPrevMass_[g] : mass;
            double rate = (prev - mass) / dt;
            if(rate < 0.0) { rate = 0.0; }   /* float noise / a stage that just dropped */
            printf(" g%d=%.3f", g, rate);
        }
        printf(" kg/s\n");
        fflush(stdout);
    }
    drainPrevTime_ = time;
    drainPrevMass_.swap(cur);
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
    /* parent -> children, from Part::parent (each non-root part has
       exactly one parent edge, so this is a tree). */
    std::map<Part *, std::vector<Part *>> children;
    for(Part *p : parts) {
        if(p->parent != nullptr) { children[p->parent].push_back(p); }
    }
    std::set<Part *> dropped;
    for(Part *p : parts) {
        if(!p->isDecoupler() || p->stage != stage) { continue; }
        dropped.insert(p);   // the decoupler flies off with its stage
        /* BFS over the decoupler's child side (its direct children and
           their descendants). */
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

    /* Rigid rebase of B's parts from B's frame into this ship's S:
       with frame S at world (p, R) and S_B at (p_B, R_B),
           x_S  =  R^T R_B x_Sb + R^T (p_B - p)   (position)
       R^T R_B likewise for the rotations. */
    glm::dvec3 pA, pB; glm::dmat3 RA, RB;
    frameS(pA, RA);
    B->frameS(pB, RB);
    const glm::dmat3 T_rot = glm::transpose(RA) * RB;
    const glm::dvec3 T_pos = glm::transpose(RA) * (pB - pA);
    for(Part *q : B->parts) {
        q->localPos = T_rot * q->localPos + T_pos;
        q->localRot = T_rot * q->localRot;
    }

    /* Rigid-body state before the move (B's hull is still live; after,
       it is an empty shell). */
    const double mA = getMass();
    const double mB = B->getMass();
    const glm::dvec3 vA = GetVelocity(hull);
    const glm::dvec3 vB = GetVelocity(B->hull);
    const glm::dvec3 wA = GetAngVelocity(hull);

    /* Topology: B's root hangs off this ship's port part -- one tree
       edge, the two port parts being the joint. The part lists, fuel
       links and crew move into this ship; B is left an empty shell. */
    Part *bRoot = B->rootPart();
    bRoot->parent = portA;

    for(Part *q : B->parts) { q->owner = this; }   // they are OUR parts now
    /* phase 4: the inventory items in B's containers ride B's vehicle via
       the container's owner (inventory.cpp) -- re-point them to this ship
       too, or they dangle when the caller deletes B as a shell. */
    {
        std::vector<Part *> stack;
        for(Part *q : B->parts) { stack.push_back(q); }
        while(!stack.empty()) {
            Part *p = stack.back(); stack.pop_back();
            for(Part *it : p->ownedContents) {
                it->owner = this;
                stack.push_back(it);
            }
        }
    }
    parts.insert(parts.end(), B->parts.begin(), B->parts.end());
    B->parts.clear();
    /* The shell's cached aero hull went with the parts: the caller deletes B
       this same tick (updateDocking), but a parts-empty ship must not carry a
       hull that would still produce drag if it ever survived to a substep. */
    B->aeroHull.clear();
    fuelLinks.insert(fuelLinks.end(), B->fuelLinks.begin(), B->fuelLinks.end());
    B->fuelLinks.clear();

    /* crew: their capsules just moved in and re-pointed to us, and a
       kerbal's capsule is a Part* -- stable through the move -- so the
       kerbals simply join this ship's crew list; each kerbal's
       `aboardPart` already names the right part. */
    for(size_t i = 0; i < B->crew.size(); i++) {
        crew.push_back(B->crew[i]);
    }
    B->crew.clear();

    /* stage counters: the union (the parts keep their baked-in numbers).
       The counter walks down from totalStages_ to minStage_, so "further
       along" is a LOWER counter; the more-advanced ship (the lower counter
       and lower floor) governs the merge. */
    if(B->totalStages_ > totalStages_) { totalStages_ = B->totalStages_; }
    if(B->activeStage_ < activeStage_) { activeStage_ = B->activeStage_; }
    if(B->minStage_ < minStage_) { minStage_ = B->minStage_; }

    clearThrust();
    clearRotCmd();

    /* B's own seams (it may have docked things of its own) move in too; they
       are inserted BEFORE the new seam so the undock order -- pop the last --
       peels the outermost dock (B) first, then B's inner docks. Without this
       the joint B recorded is orphaned when B is deleted as a shell. */
    seams.insert(seams.end(), B->seams.begin(), B->seams.end());
    B->seams.clear();
    seams.push_back(DockSeam{ portA, bRoot, B->name });

    /* Rebuild as the union (carries frame S + the velocity, which the
       inelastic average below then corrects), and regroup the fuel --
       the port parts are fuel barriers, so the two ships' fuel systems
       stay separate groups inside the one body. */
    rebuildCompound();
    buildFuelGroups();
    SetVelocity(hull, (mA * vA + mB * vB) / (mA + mB));
    SetAngVelocity(hull, wA);
}

Vehicle * Vehicle::extractSubtreeAsShip(Part *root, const std::string &name) {
    if(root == nullptr) { return nullptr; }
    bool found = false;
    for(Part *p : parts) { if(p == root) { found = true; break; } }
    if(!found) { return nullptr; }

    /* The subtree: `root` plus its descendants (BFS over the children
       map, the same walk droppedPartsAtStage uses). */
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

    /* Rigid velocity of the dropped side's COM (before the rebase --
       partPos needs the current hull). */
    double M = 0.0;
    glm::dvec3 comDropped(0.0);
    /* phase 3: effectiveMass -- the dropped side's COM velocity must carry
       the same mass basis as the rest of the simulation (a crewed capsule
       undocked out of this ship carries its crew). */
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

    /* New ship: same frame/home/sun (the split is local); no scenario
       (it is a runtime ship, not a def build). */
    Vehicle *nv = new Vehicle();
    nv->name = name;
    nv->defPath = "";
    nv->m_parent = m_parent;
    nv->frame = frame;
    nv->home = home;
    nv->sun = sun;
    nv->scenario = nullptr;

    /* Part list in the original order: parent before child -- the save
       format resolves each part's parent by reference, so a child must
       never precede its parent. (The crew no longer index their capsule:
       a Part* is stable, so the old "stable indices" reason is gone.)
       Rebase the poses into S'. */
    std::vector<Part *> nvParts;
    for(size_t i = 0; i < parts.size(); i++) {
        if(!droppedSet.count(parts[i])) { continue; }
        Part *q = parts[i];
        q->localPos = glm::transpose(RR) * (q->localPos - pR);
        q->localRot = glm::transpose(RR) * q->localRot;
        nvParts.push_back(q);
    }
    root->parent = nullptr;   // root of the new ship
    nv->parts = nvParts;
    for(Part *q : nvParts) { q->owner = nv; }   // the dropped side is ITS ship now
    /* phase 4: the inventory items in the dropped side's containers follow
       their container to the new ship (they are owned by the container and
       traverse its owner for the vehicle). */
    {
        std::vector<Part *> stack;
        for(Part *q : nvParts) { stack.push_back(q); }
        while(!stack.empty()) {
            Part *p = stack.back(); stack.pop_back();
            for(Part *it : p->ownedContents) {
                it->owner = nv;
                stack.push_back(it);
            }
        }
    }
    /* controller: the build rule (the first wheel, else the root). */
    nv->controller = nullptr;
    for(size_t i = 0; i < nvParts.size(); i++) {
        if(nvParts[i]->isWheel()) { nv->controller = nvParts[i]; break; }
    }
    if(nv->controller == nullptr) { nv->controller = root; }

    /* fuel links: both endpoints dropped -> the new ship; both kept ->
       this ship; crossing the cut -> dangling, dropped. */
    std::vector<FuelLink> nvLinks, keepLinks;
    for(size_t k = 0; k < fuelLinks.size(); k++) {
        const bool fIn = droppedSet.count(fuelLinks[k].from) > 0;
        const bool tIn = droppedSet.count(fuelLinks[k].to) > 0;
        if(fIn && tIn) { nvLinks.push_back(fuelLinks[k]); }
        else if(!fIn && !tIn) { keepLinks.push_back(fuelLinks[k]); }
    }
    nv->fuelLinks = nvLinks;
    fuelLinks = keepLinks;

    /* seams: the same containment edge as fuel links. A seam records a
       joint between its port and the docked ship's root, so it stays valid
       only while both ends live in ONE ship. Both ends staged off together
       -> the joint moves with the split-off ship (it can still undock). Both
       ends in the survivor -> stays. Split across the cut (the undock case:
       the port stays, the docked ship leaves) -> the joint no longer exists,
       so the seam is dropped. This is what stops the survivor from dangling a
       seam at a part that has been staged away -- the use-after-free the
       save path hit in the inventory design report (section 1.7b). */
    std::vector<DockSeam> nvSeams, keepSeams;
    for(size_t k = 0; k < seams.size(); k++) {
        const bool portIn = droppedSet.count(seams[k].port) > 0;
        const bool rootIn = droppedSet.count(seams[k].root) > 0;
        if(portIn && rootIn) { nvSeams.push_back(seams[k]); }
        else if(!portIn && !rootIn) { keepSeams.push_back(seams[k]); }
    }
    nv->seams = nvSeams;
    seams = keepSeams;

    /* crew: a kerbal follows its capsule. The capsules' owners were just
       re-pointed (or kept), and each kerbal's capsule is a Part* that was
       stable through the move -- so the side is read straight off the
       capsule's owner; nothing to reindex. capsulePart() is virtual
       (Kerbal returns its aboardPart; a ship returns null), so no cast and
       a headless test can stand in for a Kerbal. */
    {
        std::vector<Vehicle *> movedCrew, keepCrew;
        for(size_t i = 0; i < crew.size(); i++) {
            Vehicle *k = crew[i];
            Part *cap = k->capsulePart();
            if(cap != nullptr && cap->owner == nv) { movedCrew.push_back(k); }
            else { keepCrew.push_back(k); }
        }
        for(size_t i = 0; i < movedCrew.size(); i++) { nv->crew.push_back(movedCrew[i]); }
        crew = keepCrew;
    }

    /* this ship: the survivors (their Part* are valid; no delete order
       to worry about -- the new ship owns the dropped parts). */
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
    drainPrevTime_ = 0.0;
    drainPrevMass_.clear();

    /* The new ship: finalize (no tank re-seed -- the parts carry their
       current contents), place it at the root's world pose, and give it
       the rigid velocity of its COM. The caller enters it into the
       physics world (enterWorld) -- kept out so the split runs headless. */
    nv->finalize();
    /* The new ship was never commanded: its parts left the active ship
       mid-tick, still carrying that tick's armedThrust. Disarm them so the
       split-off vessel coasts instead of firing its inherited thrust (the
       survivor's own parts are disarmed by the clearThrust() above). */
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
    // Light direction at the ship (sun -> ship COM), in the render frame's
    // axes where the part normals end up after the xform below. Using the
    // ship's own position -- not the SOI body's center as SunlightDir does
    // -- is what keeps it defined in the Kerbol SOI, where the SOI center
    // IS the star and sun->center is a zero vector (normalize -> NaN).
    const glm::dvec3 com_root =
        frame->root_orient * get_center_of_mass() + frame->root_pos;
    glm::vec3 sunlightVec =
        glm::vec3(TerrainBody::LightDirFrom(com_root, sun, renderFrame));

    const glm::dmat4 xform = renderXform(renderFrame);

    /* Precision: place the ship with ONE common shift (its absolute COM,
       which DrawModelAt subtracts renderOrigin from -- exactly 0 for the
       active ship, an exact small difference for the others) and build the
       per-part models COM-relative in small coords. Going through absolute
       per-part positions instead would round each part onto the ULP grid of
       the huge coords (~0.125 m at oort, ~22 m at interstellar) and shake
       the ship apart; see reports/precision-scaling2026_09_22. */
    const glm::dmat4 xformShip = xform * glm::translate(get_center_of_mass());

    for(Part *p : parts) {
        // Per-part terrain shadow
        const float shadow =
            ComputeTerrainShadow(m_parent, frame, partPos(p), sun);
        /* Drawn at the part's world pose rather than at a matrix read
           off its own rigid body: a ship is ONE body, so a part's pose
           is derived. (While partPoseRelCom still reads the hull body
           this is the same matrix Draw would have built itself, minus
           the ULP rounding of the absolute coords.) */
        glm::dvec3 pp; glm::dmat3 pr;
        partPoseRelCom(p, pp, pr);
        const glm::dmat4 model = glm::translate(pp) * glm::dmat4(pr);
        p->body->DrawAt(camera, sunlightVec, shadow, model, xformShip);

        /* Engine shroud (see PartDef.shroud): while a part is attached
           on the part's exhaust face (a child below), the plain open
           cylinder hides the engine underneath -- the part's own pose,
           shader and terrain shadow, drawn right after the part so it
           depth-tests against it. */
        if(p->shroud != nullptr && hasChildBelow(p)) {
            DrawModelAt(camera, p->shroud, p->body->shader,
                        p->shroud_texture, model, sunlightVec, shadow, xformShip);
        }
    }
}

void Vehicle::Command(ShipCmd cmd, bool simActive, double step) {
    if(not simActive)
        return;
    switch(cmd.type) {
        case ThrottleUp:
            adjustThrottle(+0.01);
            break;
        case ThrottleDown:
            adjustThrottle(-0.01);
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
        // RCS translation in the ship's own axes (rcsWorldDir): each
        // command arms one body axis; diagonals compose as a vector sum,
        // like the stick.
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

float Vehicle::GetActiveThrust() {
    /* The FULL-THROTTLE thrust the ignited engines can produce. Rockets
       contribute their rated value; a jet contributes its SEA-LEVEL PEAK
       (jetThrust at v = v_e/2, its maximum) so "full TWR" is the best a jet
       can do and stays >= the current (speed/density-dependent) TWR. Dead
       in vacuum (no sea-level air -> peak 0), like ApplyThrust. */
    const int as = activeStage();
    const double rho_sea = (m_parent != nullptr)
        ? (double)m_parent->surface.atmosphere.sea_level_density : 0.0;
    double t = 0;
    for(Part *p : parts) {
        if(!p->isThruster() || p->stage < as) { continue; }
        if(p->isJet()) {
            t += jetThrust(p->def->exhaust_velocity * 0.5, rho_sea, rho_sea,
                           p->def->jet_fan_thrust, p->def->fuel_rate,
                           p->def->exhaust_velocity, p->def->jet_intake_area);
        } else {
            t += p->thrust();
        }
    }
    return (float)(t * exhaust_scale);
}

void Vehicle::ApplyThrust(double step) {
    if(thruster_util == 0.0f) { return; } /* zero throttle: no burn, no plume */
    const int as = activeStage();
    /* Jet state (shared by all jet parts this tick): the air-relative
       speed (the ship's frame velocity -- the air co-rotates with the
       planet, so this IS airspeed, like applyAeroForce) and the local air
       density at the COM (0 in vacuum). The per-part thrust is the
       air-breathing momentum balance jetThrust(v, rho, rho_sea, ...)
       (drag.h). */
    const double v_air = glm::length(GetVel());
    const double rho = airDensityAtCom();
    const double rho_sea = (m_parent != nullptr)
        ? (double)m_parent->surface.atmosphere.sea_level_density : 0.0;
    for(Part *p : parts) {
        if(!p->isThruster()) { continue; }
        if(p->stage < as) { continue; } /* not ignited yet */
        const float flow =
            (float)(p->rate() * (double)thruster_util * step); /* kg this tick, per tank */
        if(p->isJet()) {
            /* Air-breathing: the thrust is the momentum balance
               T = T_fan + ṁ_f·v_e + ρ·A·v·(v_e − v), gated on the local air
               (drag.h jetThrust). In vacuum rho = 0 -> T = 0: no thrust
               AND no burn (a jet cannot run without air). It draws JET
               FUEL only (air is the free oxidizer, no LOX) -- a resource
               SEPARATE from the rocket H2, so a jet and a rocket on the
               same ship do not share a propellant pool. */
            const double T = jetThrust(
                v_air, rho, rho_sea, p->def->jet_fan_thrust, p->def->fuel_rate,
                p->def->exhaust_velocity, p->def->jet_intake_area);
            if(T <= 0.0) { continue; }
            if(consumeResourceMass(ResourceType::JetFuel, flow, p)) {
                p->armedThrust = (float)(T * thruster_util * exhaust_scale);
                m_thrust = 1.0;
            }
            continue;
        }
        if(consumeResourceMass(ResourceType::Hydrogen, flow, p) and
           consumeResourceMass(ResourceType::LOX,      flow, p))
            {
                p->armedThrust =
                    (float)(p->thrust() * thruster_util * exhaust_scale);
                m_thrust = 1.0;
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
    const glm::dvec3 vel = GetVel();
    switch(slew) {
        case SlewPrograde:   return  vel;
        case SlewRetrograde: return -vel;
        case SlewRadialOut:  return  pos;
        case SlewRadialIn:   return -pos;
        case SlewNormal:     return  glm::cross(pos, vel);
        case SlewAntiNormal: return -glm::cross(pos, vel);
        default:             return glm::dvec3(0.0);
    }
}

void Vehicle::slewToward(glm::dvec3 dir, double h) {
    if(glm::length2(dir) < 1e-12) { return; } /* no direction to align to */
    dir = glm::normalize(dir);
    Part *wheel = firstWheel();
    const glm::dvec3 facing = partAxis(wheel, 2);
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
    const double alpha = maxTorque() / Ieff; /* rad/s^2, wheel-limited */
    const double w_des = std::min(std::sqrt(2.0 * alpha * E), E / (2.0 * h));
    /* Drive the FULL transverse angular velocity (the part perpendicular
       to the nose) toward the braking-curve rate about the slew axis.
       The old torque was along the slew axis ONLY, so the perpendicular
       "third-axis" spin was never damped: any residual spin about it at
       engagement persisted (and grew via gyroscopic coupling), and as the
       slew axis rotated that undamped spin coupled into the nose -- the
       sustained wobble around the prograde/retrograde target. Killing it
       is the fix. Roll about the nose is intentionally left free. */
    const glm::dvec3 w_now = partAngVel(wheel);
    const glm::dvec3 w_transverse = w_now - facing * glm::dot(w_now, facing);
    glm::dvec3 dW = axis * w_des - w_transverse; /* desired change in rate */
    glm::dvec3 torque = I * dW / h;
    /* Authority bound: the wheel pushes at most maxTorque() N m, so scale
       the correction down if it would exceed that. Only active while a
       third-axis spin is present; with none, dW is along the slew axis
       and |torque| == maxTorque exactly as before. */
    const double tq = glm::length(torque);
    if(tq > maxTorque()) { torque *= maxTorque() / tq; }
    ApplyTorque(hull, torque);
}

void Vehicle::killRotStep(double h) {
    Part *wheel = firstWheel();
    const glm::dvec3 w = partAngVel(wheel);
    if(glm::length2(w) == 0.0) { return; } /* at rest: nothing to kill */
    const glm::dmat3 I = getInertia();
    /* Full-tensor cancel, same form as slewToward: tau = I * (-w) / h.
       Scaling each world axis by I[i][i] alone is only valid when the
       principal basis is world-aligned; with a rotated tensor the
       under-cancel couples axes and the law settles into a period-2
       limit cycle instead of reaching zero (the post-staging |w|
       oscillation). Authority-bounded like slewToward, so the command
       never exceeds a maxed manual stick. When the bound is active the
       step is a pure scale of w toward 0 (monotonic, no sign flip). */
    glm::dvec3 torque = I * (-w) / h;
    const double tq = glm::length(torque);
    if(tq > maxTorque()) { torque *= maxTorque() / tq; }
    ApplyTorque(hull, torque);
}

glm::dvec3 Vehicle::GetPositionRelTo(const Part *part, Frame *relTo) {
    glm::dvec3 fpos = frame->GetPositionRelTo(relTo);
    glm::dmat3 forient = frame->GetOrientRelTo(relTo);
    return forient * partPos(part) + fpos;
}

void Vehicle::moveToFrame(Frame *newFrame) {
    /* One rigid body, so a frame change is one pose write and one
       velocity write; the transform is rigid, so the COM maps like any
       other point. */
    const glm::dvec3 oldCom = get_center_of_mass();
    const glm::dvec3 oldVel = GetVelocity(hull);
    const glm::dvec3 fpos = frame->GetPositionRelTo(newFrame);
    const glm::dmat3 forient = frame->GetOrientRelTo(newFrame);

    glm::dvec3 sPos; glm::dmat3 sRot;
    frameS(sPos, sRot);
    placeShip(forient * sPos + fpos, forient * sRot);

    // The stored velocity is the frame-coordinate velocity, so a ship's
    // inertial velocity is R*(v + stasis(p)) + V. The OLD frame's stasis
    // term is added here and the NEW frame's SUBTRACTED below, or the
    // ship's inertial velocity is wrong by 2*stasis and the orbit jumps
    // shape at every inertial->rotational switch.
    glm::dvec3 vel = oldVel;
    if(frame != newFrame) { vel += frame->GetStasisVelocity(oldCom); }
    vel = forient * vel + frame->GetVelocityRelTo(newFrame);
    const glm::dvec3 newCom = forient * oldCom + fpos;
    const glm::dvec3 newVel = vel - newFrame->GetStasisVelocity(newCom);

    printf("@@@ %s frame %s -> %s: com (%.0f %.0f %.0f) -> (%.0f %.0f %.0f)"
           " vel (%.0f %.0f %.0f) -> (%.0f %.0f %.0f)\n",
           name.c_str(), frame->name.c_str(), newFrame->name.c_str(),
           oldCom.x, oldCom.y, oldCom.z, newCom.x, newCom.y, newCom.z,
           oldVel.x, oldVel.y, oldVel.z, newVel.x, newVel.y, newVel.z);

    /* after placeShip: proceedToTransform zeroes both velocities */
    SetVelocity(hull, newVel);

    // The ship lives in the ships list of its SOI body (terrain.h):
    // crossing to another body's SoI is a list move, done here so the
    // body lists always agree with m_parent. (Rare -- this runs once
    // per crossing, not per tick.)
    if(m_parent != nullptr && m_parent != newFrame->body) {
        for(auto it = m_parent->ships.begin();
            it != m_parent->ships.end(); it++) {
            if(*it == this) { m_parent->ships.erase(it); break; }
        }
        newFrame->body->ships.push_back(this);
    }
    frame = newFrame;
    m_parent = newFrame->body;
}

void Vehicle::switchFrames() {
    const glm::dvec3 com = get_center_of_mass();
    double ship_r = glm::length(com);
    if(ship_r > frame->soi + 10000) {
        // switching to parent SOI if there is one
        if(frame->parent != NULL) {
            glm::dvec3 pos = partPos(controller);
            printf("@@@ %s switching frame from %s to parent %s\n",
                   name.c_str(), frame->name.c_str(),
                   frame->parent->name.c_str());
            glm::dvec3 offset = frame->GetPositionRelTo(frame->parent);
            printf("@@@ Frame offset: %.0f %.0f %.0f\n", offset.x, offset.y, offset.z);
            printf("@@@@@ OLD position: %.0f %.0f %.0f\n", pos.x, pos.y, pos.z);
            moveToFrame(frame->parent);
            pos = partPos(controller);
            printf("@@@@@ NEW position: %.0f %.0f %.0f\n", pos.x, pos.y, pos.z);
        }
    }
    else {
        // check if we've entered a child SOI
        for(auto&& child : frame->children) {
            double dist = glm::length(GetPositionRelTo(controller, child));
            if(dist < child->soi - 10000) {
                printf("@@@ %s switching frame from %s to child %s, distance: %.0f\n",
                       name.c_str(), frame->name.c_str(),
                       child->name.c_str(), dist);
                moveToFrame(child);
                break;
            }
        }
    }
}

void Vehicle::writeRailPose() {
    /* ONE write, and it is the whole ship. rail_pos is the COM, which is
       exactly the hull's transform origin, and rail_orient * railRot are
       frame S's axes; every part's pose then follows from its authored
       local pose. There is no per-part snapshot to restore and no
       deformation to freeze -- the geometry the rails carry IS the
       authored geometry. */
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
    return el.periapsis <= inertial->body->radius + 3000.0;
}

bool Vehicle::canRail() {
    if(onRails) { return true; }
    if(inTerrainBand()) {
        return frame->isRotFrame();   // grounded: freeze needs the surface frame
    }
    return true;
}

bool Vehicle::goOnRails() {
    if(onRails) { return true; }
    if(!canRail()) { return false; }

    /* The COM state in the body's inertial frame node, where the
       trajectory is a Kepler conic. The velocity is the hull's own -- one
       rigid body, one COM velocity -- and the parked ship's residual spin
       is discarded with its attitude (writeRailPose zeroes it). */
    Frame *oldFrame = frame;
    Frame *inertial = frame->getNonRotFrame();
    const glm::dvec3 com_frame = get_center_of_mass();  // old frame coords
    const glm::dvec3 vel_frame = GetVelocity(hull);     // old frame coords
    glm::dvec3 p, v;
    comStateIn(inertial, p, v);

    const OrbitElements el = computeOrbitElements(p, v, inertial->body->mu);
    const bool grounded = el.periapsis <= inertial->body->radius + 3000.0;

    /* Frame S's axes at park time (== the old frame's axes for a ship
       built in it); rail_orient carries them into the inertial node and
       then holds inertially. Nothing per-part to snapshot: the ship is
       rigid, so the authored local poses ARE the parked geometry. */
    glm::dvec3 sPosPark;
    frameS(sPosPark, railRot);

    if(grounded) {
        /* freeze: the pose is static in the rotating surface frame
           (its transforms already are), so the rail state just holds
           it; the planet's spin carries it via the render transform. */
        rail_pos = com_frame;
        rail_vel = vel_frame;
        rail_orient = glm::dmat3(1.0);
        railFrozen = true;
    } else {
        rail_pos = p;
        rail_vel = v;
        rail_orient = oldFrame->GetOrientRelTo(inertial);
        frame = inertial;   // on rails, ship->frame == its inertial node
        railFrozen = false;
    }

    /* out of the world: the one body */
    if(hullInWorld()) { RemoveBody(hull); }

    onRails = true;
    if(!railFrozen) { writeRailPose(); }
    if(grounded) {
        printf("@@@ %s frozen on rails (grounded around %s)\n",
               name.c_str(), frame->body->name.c_str());
    } else {
        printf("@@@ %s parked on rails around %s: sma=%.6g m ecc=%.4f\n",
               name.c_str(), inertial->body->name.c_str(),
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
           name.c_str(), frame->body->name.c_str());
}

void Vehicle::railsTick(const double step) {
    if(!onRails || railFrozen) { return; }
    propagateKepler(rail_pos, rail_vel, frame->body->mu, step,
                    rail_pos, rail_vel);
    railsSwitchFrames();
    writeRailPose();
}

void Vehicle::railsSwitchFrames() {
    const double r = glm::length(rail_pos);
    if(r > frame->soi + 10000) {
        if(frame->parent != NULL) {
            printf("@@@ %s rails switching frame from %s to parent %s\n",
                   name.c_str(), frame->name.c_str(),
                   frame->parent->name.c_str());
            moveToRailFrame(frame->parent);
        }
    } else {
        for(auto&& child : frame->children) {
            if(child->body == frame->body) { continue; }
            // ship position in the child's coordinates (the same
            // transform GetPositionRelTo(part, child) applies)
            const glm::dvec3 rel = frame->GetOrientRelTo(child) * rail_pos
                                 + frame->GetPositionRelTo(child);
            const double dist = glm::length(rel);
            if(dist < child->soi - 10000) {
                printf("@@@ %s rails switching frame from %s to child %s, distance: %.0f\n",
                       name.c_str(), frame->name.c_str(),
                       child->name.c_str(), dist);
                moveToRailFrame(child);
                break;
            }
        }
    }
}

void Vehicle::moveToRailFrame(Frame *newFrame) {
    const glm::dmat3 O = frame->GetOrientRelTo(newFrame);
    rail_vel = O * rail_vel + frame->GetVelocityRelTo(newFrame);
    rail_pos = O * rail_pos + frame->GetPositionRelTo(newFrame);
    rail_orient = O * rail_orient;
    // Same list move as moveToFrame (the ship follows its SoI body).
    if(m_parent != nullptr && m_parent != newFrame->body) {
        for(auto it = m_parent->ships.begin();
            it != m_parent->ships.end(); it++) {
            if(*it == this) { m_parent->ships.erase(it); break; }
        }
        newFrame->body->ships.push_back(this);
    }
    frame = newFrame;
    m_parent = newFrame->body;
}
