// vehicle.cpp -- ship building + spawn scenarios (declarations in
// vehicle.h; the Vehicle class itself is inline there).
#include "vehicle.h"

#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <stdexcept>

#include "system.h"    // System (spawn_vehicle resolves the home body's SOI)
#include "texture.h"   // load_texture

/* Instantiate a ship def: one rigid body per part (mesh + texture from the
   catalog entry), welded parent-first in the def's construction order.
   GL is needed here (shader binding); the JSON parse/validate and the
   attach geometry (attachPose) are GL-free (shipdef.cpp). The catalog must
   outlive the ship (the partDefs point into it). */
void build_ship(Vehicle *ship, const ShipDef &def, Shader *partsshader,
                       const glm::dvec3 &base, const glm::dmat3 &orient)
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
       the stack axis, each child welded to its (earlier) parent by the
       shared attachPose geometry (shipdef.cpp). */
    std::vector<glm::dvec3> pos(n);
    std::vector<glm::dmat3> rot(n);
    std::vector<glm::dvec3> pAnchor(n), cAnchor(n);
    pos[0] = glm::dvec3(0.0);
    rot[0] = glm::dmat3(1.0);
    for(size_t i = 1; i < n; i++) {
        const ShipPart &sp = physical[i];
        AttachPose ap = attachPose(pos[(size_t)sp.parent], rot[(size_t)sp.parent],
                                   *physical[(size_t)sp.parent].def, *sp.def,
                                   sp.attach, sp.angle, sp.offset);
        pos[i] = ap.childPos;
        rot[i] = ap.childRot;
        pAnchor[i] = ap.parentAnchor;
        cAnchor[i] = ap.childAnchor;
    }

    /* 2) the ship's lowest point along the stack axis: an axis-aligned part
       (down/side) spans h/2 about its center, a radial part spans its
       radius (its cross-section lies across the stack axis). */
    double lowest = 1e30;
    for(size_t i = 0; i < n; i++) {
        const ShipPart &sp = physical[i];
        double extent = (i > 0 && sp.attach == AttachMode::Radial)
                       ? sp.def->radius : sp.def->height / 2.0;
        lowest = std::min(lowest, pos[i].z - extent);
    }

    /* 3) place it on the pad: the lowest point at the pad top, lifted by
       the collision margins (terrain 0.5 + hull 0.1) so the inflated
       shapes just touch instead of popping apart on the first solve. For
       orbit scenarios this is only staging -- spawn_vehicle repositions. */
    const glm::dvec3 shift = glm::dvec3(0.0, 0.0, -lowest + 0.6);

    for(size_t i = 0; i < n; i++) {
        const PartDef &pd = *physical[i].def;

        Mesh *mesh = new Mesh;
        mesh->FromFile((std::string("./res/") + pd.mesh).c_str(), true);
        Texture *tex = load_texture((std::string("./res/") + pd.texture).c_str());
        Model *model = new Model;
        model->FromData(mesh, partsshader, tex);
        model->hull_margin = resolveHullMargin(def.hull_margin, pd.hull_margin);

        Body *b = create_body(model, 0, 0, 0, (float)pd.mass, false);
        setPosRot(b, base + orient * (pos[i] + shift), orient * rot[i]);

        Part *part = new Part;
        part->body  = b;
        part->def   = &pd;
        part->stage = physical[i].stage;

        if(i == 0) {
            ship->setRoot(part);
        } else {
            const ShipPart &sp = physical[i];
            ship->attach(part, (size_t)sp.parent, pAnchor[i], cAnchor[i]);
        }
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

    ship->init();
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
       real solar-system distances, so a name means the same distance
       around any body. Precision test beds -- neptune is comfortably
       inside double's range (~1 mm ULP), oort is where it starts to bite
       (~0.22 m). */
    {"neptune",        false, 0.0,  false, -1, 0.0,     0.0, 0.0, 4.495e12},
    {"oort",           false, 0.0,  false, -1, 0.0,     0.0, 0.0, 1.0e15},
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

    // Nose (local +Z) along prograde: rigidly re-orient the whole ship.
    // Part 0 (the root) takes `orient`; every other part gets the same
    // world rotation (Rrel) about the ship's COM, so its RELATIVE geometry
    // survives -- a stacked part stays stacked, a radial part keeps its
    // perpendicular axis. (The old loop applied `orient` to every part,
    // which silently straightened a radial part into the stack axis.)
    const glm::dmat3 orient = faceAlong(velWorld);
    const glm::dvec3 com0 = ship->get_center_of_mass();
    const glm::dmat3 Rrel = orient * glm::transpose(GetOrient(ship->parts[0]->body));
    for(Part *part : ship->parts) {
        const glm::dvec3 p = GetPosition(part->body);
        const glm::dmat3 R0 = GetOrient(part->body);
        setPosRot(part->body, target + Rrel * (p - com0), Rrel * R0);
        SetVelocity(part->body, vel);
    }

    printf("Spawn '%s' around %s: frame '%s' @ world (%.0f, %.0f, %.0f), r = %.0f m, |v| = %.1f m/s\n",
           sc.name, home->name.c_str(), frame->name.c_str(),
           shipWorldPos.x, shipWorldPos.y, shipWorldPos.z,
           glm::length(shipWorldPos - center), glm::length(velWorld));
}

/* --radial-test spin diagnostics (two-part ship): the per-part angular
   velocities (if they differ, the weld is not holding a rigid body), the
   INTERNAL contact torque between the two parts -- the only way a passive
   welded pair can spin itself -- and the tidal (differential gravity)
   torque, which is the one legitimate external torque and should be
   negligible at ship scale. */
void spin_log(Vehicle *ship, double time) {
    if(ship->parts.size() < 2) { return; }

    const glm::dvec3 com = ship->get_center_of_mass();
    printf("[spin] t=%.2fs ship=%s com=[%.0f %.0f %.0f] parts=%zu\n",
           time, ship->name.c_str(), com.x, com.y, com.z, ship->parts.size());
    for(size_t i = 0; i < ship->parts.size(); i++) {
        const glm::dvec3 w = GetAngVelocity(ship->parts[i]->body);
        const glm::dvec3 p = GetPosition(ship->parts[i]->body);
        printf("[spin]   %-14s pos=[%.1f %.1f %.1f] w=[%.3e %.3e %.3e] |w|=%.3e\n",
               ship->parts[i]->def->name.c_str(),
               p.x, p.y, p.z, w.x, w.y, w.z, glm::length(w));
    }

    for(size_t i = 0; i < ship->parts.size(); i++) {
        for(size_t j = i + 1; j < ship->parts.size(); j++) {
            const ContactPairInfo cp = contact_report(ship->parts[i]->body, ship->parts[j]->body);
            printf("[spin]   contact %-8s-%-8s: manifs=%d (other=%d) pts=%zu |F|=%.3e |T|=%.3e maxImp=%.3e\n",
                   ship->parts[i]->def->name.c_str(), ship->parts[j]->def->name.c_str(),
                   cp.manifolds, cp.otherManifolds, cp.points.size(),
                   glm::length(cp.netForce), glm::length(cp.netTorque), cp.maxImpulse);
            for(size_t k = 0; k < cp.points.size(); k++) {
                const ContactPointInfo &p = cp.points[k];
                printf("[spin]     pt%zu pos=[%.1f %.1f %.1f] pen=%.4f imp=[%.3e %.3e %.3e] |imp|=%.3e\n",
                       k, p.pos.x, p.pos.y, p.pos.z, p.pen,
                       p.impulse.x, p.impulse.y, p.impulse.z, glm::length(p.impulse));
            }
        }
    }

    const double G = 6.674e-11;
    const double M = ship->m_parent->mass;
    glm::dvec3 tau(0, 0, 0);
    for(size_t i = 0; i < ship->parts.size(); i++) {
        const glm::dvec3 p = GetPosition(ship->parts[i]->body);
        const double r = glm::length(p);
        const glm::dvec3 F = -G * M * ship->parts[i]->body->mass * p / (r * r * r);
        tau += glm::cross(p - com, F);
    }
    printf("[spin]   tidal gravity torque |tau|=%.3e\n", glm::length(tau));
    fflush(stdout);
}
