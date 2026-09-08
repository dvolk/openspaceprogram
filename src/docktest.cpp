// docktest.cpp -- the --dock-test pair builder (see docktest.h).
//
// Two single-stage r1.0 ships, nose-to-nose along the station's prograde
// (the docking axis), co-moving on the same circular orbit:
//   station  tank_r1h3 (root) + docking_port_r1 on its REAR end (-Z)
//   probe    docking_port_r1 (root, FRONT) + tank + engine behind it
// The probe is the active ship and its engine thrusts along its +Z, which
// is the direction toward the station (the station is ahead of it along
// prograde) -- so a prograde burn closes the gap. Both ports present the
// face pointing at the other port, exactly along the line between them,
// so the alignment test (cos 15 deg) passes with margin.
#include "docktest.h"

#include <cmath>
#include <stdexcept>

#include "body.h"     // create_part_body
#include "mesh.h"     // Mesh
#include "model.h"    // Model
#include "physics.h"  // GetVelocity
#include "texture.h"  // load_texture

DockTestShips build_dock_test_ships(const std::string &mode,
                                    bool scenario_given,
                                    const std::string &scenario,
                                    const PartsCatalog &part_catalog,
                                    TerrainBody *home,
                                    TerrainBody *sun,
                                    Shader *partsshader,
                                    System &sys)
{
    const PartDef *defPort = part_catalog.find("docking_port_r1");
    const PartDef *defEng  = part_catalog.find("engine");
    const PartDef *defTank = part_catalog.find("tank_r1h3");
    if(defPort == nullptr || defEng == nullptr || defTank == nullptr) {
        throw std::runtime_error("--dock-test: docking_port_r1 / engine / "
                                 "tank_r1h3 missing from the parts catalog");
    }

    /* the starting port-face gap (m): "near" sits inside the kDockCapture
       window (1.5 m) and docks on the first live tick; "approach" (the
       default) starts outside it and must burn the last stretch. */
    double gap = 1.0;
    if(mode == "approach") { gap = 2.0; }
    else if(mode != "near") {
        throw std::runtime_error("--dock-test: unknown mode '" + mode
                                 + "' (near | approach)");
    }

    const ScenarioDef *sc = scenario_by_name(
        scenario_given ? scenario : "rot-orbit");

    /* One Part (a rigid body + render model wrapped with its catalog spec),
       exactly the radialtest builder's makePart. */
    auto makePart = [&](const PartDef *def) -> Part * {
        Mesh *mesh = new Mesh;
        mesh->FromFile((std::string("./res/") + def->mesh).c_str(), true);
        Model *model = new Model;
        model->FromData(mesh, partsshader,
                        load_texture((std::string("./res/") + def->texture).c_str()));
        model->hull_margin = def->hull_margin;
        Body *b = create_part_body(model, (float)def->mass);
        Part *p = new Part;
        p->body  = b;
        p->def   = def;
        p->stage = 1;
        return p;
    };

    /* --- station: tank (root) + port on its rear end (-Z) ---------------
       Both ships are oriented local +Z = prograde (spawn_vehicle's
       faceAlong(vel)). The station's port must face the probe, which sits
       BEHIND it, so the port goes on the tank's -Z side. */
    Vehicle *station = new Vehicle;
    station->m_parent = home;
    station->sun = sun;
    station->frame = home->rot_frame;
    station->name = "dockstation";
    Part *stTank = makePart(defTank);
    Part *stPort = makePart(defPort);
    station->setRoot(stTank);
    station->attachDown(stPort);   // port stacked on the tank's rear (-Z) face
    station->controller = stTank;
    station->init();
    station->enterWorld();
    // Circular orbit, slot 0 (no lateral offset): the station's COM ends up
    // at the orbit radius, oriented nose (+Z) along prograde.
    spawn_vehicle(station, *sc, home, sys, 0.0);

    /* --- probe: port (root, front) + tank + engine behind it -----------
       The engine sits at the rear and thrusts along its +Z = the probe's
       +Z = prograde = toward the station ahead of it. attachDown stacks
       each part face-to-face below the last (port, then tank, then engine),
       so nothing overlaps. */
    Vehicle *probe = new Vehicle;
    probe->m_parent = home;
    probe->sun = sun;
    probe->frame = home->rot_frame;
    probe->name = "dockprobe";
    Part *prPort = makePart(defPort);
    Part *prTank = makePart(defTank);
    Part *prEng  = makePart(defEng);
    probe->setRoot(prPort);
    probe->attachDown(prTank);     // tank below the port
    probe->attachDown(prEng);      // engine below the tank (no overlap)
    probe->controller = prTank;
    probe->init();

    /* --- place the probe on the station's orbit, trailing in phase -----
       The orbit is a Keplerian conic in the inertial node (the station is
       driven by that conic on the rails). The probe is the ACTIVE ship,
       simulated by Bullet in the rotating frame with the fictitious
       forces. To keep the two in step, phase the probe to a point on the
       station's conic in the inertial node -- rotating the station's
       inertial (r, v) about the conic's angular-momentum axis is another
       valid state of that same orbit -- then express that state in the
       rotating frame with the exact inverse of comStateIn, so the Bullet
       sim lands on the conic instead of a neighbouring one. */
    if(probe->frame != station->frame) {
        probe->moveToFrame(station->frame);
    }
    Frame *inert = station->frame->getNonRotFrame();
    glm::dvec3 rS_i, vS_i;
    station->comStateIn(inert, rS_i, vS_i);
    const glm::dvec3 h_i = glm::normalize(glm::cross(rS_i, vS_i));
    /* the docking reference is the port, not the COM: the along-track COM
       spacing must clear both COM-to-port offsets plus the face gap. */
    const glm::dvec3 rS = station->get_center_of_mass();
    const double dS = glm::length(station->partPos(stPort) - rS);
    probe->placeShip(glm::dvec3(0.0), glm::dmat3(1.0)); // COM reads at origin
    const double dP = glm::length(probe->get_center_of_mass());
    const double arc = dP + dS + gap;                 // COM-to-COM spacing
    const double th = -arc / glm::length(rS_i);       // negative = trailing
    auto rotAxis = [&](const glm::dvec3 &v) {
        return v * std::cos(th) + glm::cross(h_i, v) * std::sin(th)
             + h_i * glm::dot(h_i, v) * (1.0 - std::cos(th));
    };
    const glm::dvec3 rP_i = rotAxis(rS_i);
    const glm::dvec3 vP_i = rotAxis(vS_i);
    /* Inertial node -> the ship's rotating frame (the inverse of
       comStateIn): rotate back, drop the frame offset, subtract the
       frame's own velocity and the point's stasis velocity. */
    const glm::dmat3 Rinv = glm::transpose(station->frame->GetOrientRelTo(inert));
    const glm::dvec3 pOff = station->frame->GetPositionRelTo(inert);
    const glm::dvec3 vFrm = station->frame->GetVelocityRelTo(inert);
    const glm::dvec3 rP = Rinv * (rP_i - pOff);
    const glm::dvec3 vP = Rinv * (vP_i - vFrm)
                        - station->frame->GetStasisVelocity(rP);
    /* Face the station: the probe's port is on its +Z, so aim +Z at the
       station (ahead along the orbit) for a clean aligned dock. */
    const glm::dvec3 faceDir = glm::normalize(rS - rP);
    probe->placeShipAtCom(rP, faceAlong(faceDir));
    probe->enterWorld();
    // Co-moving on the same orbit: zero relative speed -> the kDockMaxV gate
    // passes the moment the gap is inside capture.
    probe->setVelocity(vP);

    /* Docking is intent-driven (Game::updateDocking docks only a targeted
       port): arm the probe to mate with the station's port, which is what
       the player does in-game by right-clicking it -> "Target for docking". */
    probe->dockTargetShip = station;
    probe->dockTargetPort = stPort;

    printf("Dock-test (%s): station '%s' + probe '%s', port-face gap %.2f m, "
           "|v| = %.1f m/s\n",
           mode.c_str(), station->name.c_str(), probe->name.c_str(),
           gap, glm::length(GetVelocity(station->hull)));

    DockTestShips d;
    d.probe = probe;
    d.station = station;
    d.gap = gap;
    return d;
}
