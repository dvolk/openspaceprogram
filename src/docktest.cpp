// docktest.cpp -- the --dock-test pair builder (see docktest.h).
// Two single-stage r1.0 ships, nose-to-nose along the station's prograde,
// co-moving on the same circular orbit, each carrying the same seven parts.
// The probe is the active ship; its engine thrusts along +Z toward the station.
#include "docktest.h"

#include <cmath>
#include <numbers>
#include <stdexcept>

#include "part.h"     // make_part
#include "physics.h"  // GetVelocity

DockTestShips build_dock_test_ships(const std::string &mode,
                                    const PartsCatalog &part_catalog,
                                    TerrainBody *home,
                                    TerrainBody *sun,
                                    Shader *partsshader,
                                    System &sys)
{
    const PartDef *defPort = part_catalog.find("docking_port_r1");
    const PartDef *defCap  = part_catalog.find("capsule");
    const PartDef *defRcs  = part_catalog.find("rcs_r1");
    const PartDef *defMono = part_catalog.find("mono_tank_r1");
    const PartDef *defRw   = part_catalog.find("reaction_wheel");
    const PartDef *defTank = part_catalog.find("tank_r1h3");
    const PartDef *defEng  = part_catalog.find("engine");
    if(defPort == nullptr || defCap == nullptr || defRcs == nullptr
       || defMono == nullptr || defRw == nullptr
       || defTank == nullptr || defEng == nullptr) {
        throw std::runtime_error("--dock-test: a part the pair needs is "
                                 "missing from the parts catalog");
    }

    /* The COM-arc spacing that puts the port FACES `gap` metres apart in the aligned
       modes (the arc is built from each hull's COM-to-port distance, so it holds for
       their stacks as built, not for a yawed one -- see the banner below). */
    double gap = 1.0;
    /* #1026: yaw the probe off the line to the station so the two port axes leave the
       kDockAlign cone. The Docking window's end-to-end case needs a pair the gate refuses
       -- with the other two modes it opens at once and the panel has nothing to explain. */
    double yawDeg = 0.0;
    if(mode == "approach") { gap = 2.0; }
    else if(mode == "misaligned") { yawDeg = 40.0; }
    else if(mode != "near") {
        throw std::runtime_error("--dock-test: unknown mode '" + mode
                                 + "' (near | approach | misaligned)");
    }

    const ScenarioDef *sc = scenario_by_name("rot-orbit");

    // One Part: shared render assets + hull + mass.
    auto makePart = [&](const PartDef *def) -> Part * {
        Part *p = make_part(*def, def->hull_margin, nullptr, nullptr,
                            def->name, partsshader);
        p->stage = 1;
        return p;
    };

    /* --- station: the same 7-part stack, mirror-oriented --- */
    Vehicle *station = new Vehicle;
    station->m_parent = home;
    station->sun = sun;
    station->frame = home->rot_frame;
    station->name = "dockstation";
    Part *stEng  = makePart(defEng);
    Part *stTank = makePart(defTank);
    Part *stRw   = makePart(defRw);
    Part *stMono = makePart(defMono);
    Part *stRcs  = makePart(defRcs);
    Part *stCap  = makePart(defCap);
    Part *stPort = makePart(defPort);
    station->setRoot(stEng);
    station->attachDown(stTank);
    station->attachDown(stRw);
    station->attachDown(stMono);
    station->attachDown(stRcs);
    station->attachDown(stCap);
    station->attachDown(stPort);   // port on the tail (-Z), facing the probe
    station->controller = stRw;
    station->init();
    station->enterWorld();
    // Circular orbit, slot 0: the station's COM ends up at the orbit radius,
    // nose (+Z) along prograde.
    spawn_vehicle(station, *sc, home, sys, 0.0, 0.0);

    /* --- probe: port (root, front) ... engine (rear) --- */
    Vehicle *probe = new Vehicle;
    probe->m_parent = home;
    probe->sun = sun;
    probe->frame = home->rot_frame;
    probe->name = "dockprobe";
    Part *prPort = makePart(defPort);
    Part *prCap  = makePart(defCap);
    Part *prRcs  = makePart(defRcs);
    Part *prMono = makePart(defMono);
    Part *prRw   = makePart(defRw);
    Part *prTank = makePart(defTank);
    Part *prEng  = makePart(defEng);
    probe->setRoot(prPort);
    probe->attachDown(prCap);
    probe->attachDown(prRcs);
    probe->attachDown(prMono);
    probe->attachDown(prRw);
    probe->attachDown(prTank);
    probe->attachDown(prEng);      // engine at the rear, thrusting +Z
    probe->controller = prRw;
    probe->init();

    /* --- place the probe on the station's orbit, trailing in phase --- */
    if(probe->frame != station->frame) {
        probe->moveToFrame(station->frame, 0.0);   // t=0: the test boots at epoch
    }
    Frame *inert = station->frame->getNonRotFrame();
    glm::dvec3 rS_i, vS_i;
    station->comStateIn(inert, rS_i, vS_i);
    const glm::dvec3 h_i = glm::normalize(glm::cross(rS_i, vS_i));
    /* The docking reference is the port, not the COM. */
    const glm::dvec3 rS = station->get_center_of_mass();
    const double dS = glm::length(station->partPos(stPort) - rS);
    probe->placeShip(glm::dvec3(0.0), glm::dmat3(1.0)); // COM reads at origin
    const double dP = glm::length(probe->get_center_of_mass());
    const double arc = dP + dS + gap;                 // COM-to-COM spacing
    const double th = -arc / glm::length(rS_i);       // negative = trailing
    /* Rodrigues rotation about the orbit normal, by an arbitrary angle: the phase
       offset below uses it, and so does the "misaligned" yaw -- one spelling. */
    auto rotH = [&](const glm::dvec3 &v, double a) {
        return v * std::cos(a) + glm::cross(h_i, v) * std::sin(a)
             + h_i * glm::dot(h_i, v) * (1.0 - std::cos(a));
    };
    const glm::dvec3 rP_i = rotH(rS_i, th);
    const glm::dvec3 vP_i = rotH(vS_i, th);
    /* Inertial -> the ship's rotating frame (the inverse of comStateIn):
       rotate back, drop the frame offset, subtract the frame's own velocity
       and the point's stasis velocity. */
    const glm::dmat3 Rinv = glm::transpose(station->frame->GetOrientRelTo(inert));
    const glm::dvec3 pOff = station->frame->GetPositionRelTo(inert);
    const glm::dvec3 vFrm = station->frame->GetVelocityRelTo(inert);
    const glm::dvec3 rP = Rinv * (rP_i - pOff);
    const glm::dvec3 vP = Rinv * (vP_i - vFrm)
                        - station->frame->GetStasisVelocity(rP);
    /* Face the station: the probe's port is on its +Z, so aim +Z at the
       station (ahead along the orbit) for a clean aligned dock. */
    const glm::dvec3 faceDir = glm::normalize(rS - rP);
    /* "misaligned" yaws that aim off the line, and takes the yaw AROUND THE PORT rather
       than around the COM: this is a seven-part stack whose port sits metres out from its
       own centre of mass, so a yaw about the COM swings the port more than a metre
       sideways -- which would fail the distance gates as well as the cone one and leave
       the fixture testing two things at once. Place, read where the port went, place
       again shifted by the difference.
       The angle is 40 deg, not the 16 that would do: nothing flies the probe, so the pair
       wanders, and a yaw set just outside the cone is inside it (or far outside it) by the
       time the window is on screen. Measured over the 3 s the end-to-end case runs: the
       face gap the panel first prints is 1.2 m and the last line 3 s later says 4.3 m
       (separating at 1.2 m/s), while the misalignment drifts from 38 to 46 deg. The
       verdict stays `misaligned` through all of it because the gate asks the axis question
       before the distance one -- the same order tests/test_dock.cpp pins. */
    probe->placeShipAtCom(rP, faceAlong(faceDir));      // the aligned pose
    if(yawDeg != 0.0) {
        const glm::dvec3 port0 = probe->partPos(prPort);    // where its port belongs
        const glm::dvec3 aim = rotH(faceDir, yawDeg * std::numbers::pi / 180.0);
        probe->placeShipAtCom(rP, faceAlong(aim));
        const glm::dvec3 port1 = probe->partPos(prPort);
        probe->placeShipAtCom(rP + (port0 - port1), faceAlong(aim));
    }
    probe->enterWorld();
    // Co-moving on the same orbit: zero relative speed -> the kDockMaxV gate
    // passes the moment the gap is inside capture.
    probe->setVelocity(vP);

    /* Docking is intent-driven (Game::updateDocking needs BOTH halves): the
       probe's port is armed, the station's port targeted -- what the player
       does with "Arm for docking" / "Target for docking". */
    probe->dockArmPort = prPort;
    probe->dockTargetShip = station;
    probe->dockTargetPort = stPort;

    /* The banner says what the FIXTURE asked for, not what the ports ended up at: the arc
       is built for the aligned stacks, so a yawed probe's face gap is whatever the yaw
       leaves (the Docking window prints the measured figure, and for `misaligned` it is
       not this number). Naming the yaw here is what keeps the two from looking like a
       contradiction in one log. */
    if(yawDeg == 0.0) {
        printf("Dock-test (%s): station '%s' + probe '%s', arc gap %.2f m, "
               "|v| = %.1f m/s\n",
               mode.c_str(), station->name.c_str(), probe->name.c_str(),
               gap, glm::length(GetVelocity(station->hull)));
    } else {
        printf("Dock-test (%s): station '%s' + probe '%s', arc gap %.2f m, "
               "yaw %.0f deg about the port, |v| = %.1f m/s\n",
               mode.c_str(), station->name.c_str(), probe->name.c_str(),
               gap, yawDeg, glm::length(GetVelocity(station->hull)));
    }

    DockTestShips d;
    d.probe = probe;
    d.station = station;
    return d;
}
