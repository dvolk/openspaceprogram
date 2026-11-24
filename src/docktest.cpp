// docktest.cpp -- the --dock-test pair builder (see docktest.h).
// Two single-stage r1.0 ships, nose-to-nose along the station's prograde,
// co-moving on the same circular orbit, each carrying the same seven parts --
// except `stale-target`, where the station wears two more (a decoupler and a port
// under its tail, and THAT far port is the targeted one, so a stage press on that
// hull walks the targeted port off into a new ship: #1062).
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
    /* rcs_box stacked where the round pod (#1120) used to sit: this fixture only
       needs the pair to carry a ring, and the port arc below is recomputed from
       the built hull's COM-to-port distance, so the box being 5 cm taller than
       the pod it replaces moves the pair rather than skewing it. */
    const PartDef *defRcs  = part_catalog.find("rcs_box");
    const PartDef *defMono = part_catalog.find("mono_tank_r1");
    const PartDef *defRw   = part_catalog.find("reaction_wheel");
    const PartDef *defTank = part_catalog.find("tank_r1h3");
    const PartDef *defEng  = part_catalog.find("engine");
    const PartDef *defDec  = part_catalog.find("decoupler_r1");
    if(defPort == nullptr || defCap == nullptr || defRcs == nullptr
       || defMono == nullptr || defRw == nullptr
       || defTank == nullptr || defEng == nullptr || defDec == nullptr) {
        throw std::runtime_error("--dock-test: a part the pair needs is "
                                 "missing from the parts catalog");
    }

    /* Port CENTRE spacing along the orbit arc. The arc is built from each hull's
       COM-to-port distance, so it holds for their stacks as built, not for a yawed one
       (see the banner below). The FACES sit half a port depth closer on each side: two
       25 cm r1 ports at a 1.0 m centre gap read a 0.75 m face gap, measured.
       #1152 retuned these spacings, because the r1 pair now captures inside 0.10 m of
       face gap rather than 1.5 m: a fixture that is meant to mate has to start inside
       its own gate. The refusal fixtures keep their 1 m spacing on purpose -- at 0.05 m
       a 40 deg yaw would drive the two port meshes through each other and the fixture
       would be testing the contact solver as well. */
    double gap = 0.30;
    /* The scripted closing rate, m/s, added along the line joining the two hulls in
       their shared rotating frame (both sit at rest in it as placed, so this IS the
       relative speed the gate measures). #1152: the approach fixture used to fire the
       50 kN engine for 30 ms, which arrives at 0.51 m/s -- over three times the capture
       cap, and at 0.0091 m/s^2 of RCS authority no player could have flown that legally.
       A scripted rate has no key timing in it, so the case cannot flake on how many
       substeps a wall-clock press happened to cover (#47's old comment). */
    double closeV = 0.0;
    /* #1026: yaw the probe off the line to the station so the two port axes leave the
       kDockAlign cone. The Docking window's end-to-end case needs a pair the gate refuses
       -- with the other two modes it opens at once and the panel has nothing to explain. */
    double yawDeg = 0.0;
    /* #1040: which half of the docking intent goes stale, and how. Same pair, same
       geometry; only the ARMED port moves, onto a part the stage press hands to a new
       vessel. See the probe block below. */
    bool shedArm = false;
    /* #1062: the same trick on the OTHER hull. The station's targeted port rides the
       station's own stage-1 decoupler, so a stage press while the station is the active
       ship hands that port to a new vessel and the probe's intent points at a hull that
       no longer carries it. */
    bool shedTarget = false;
    /* #1042: 4 deg/s of roll about the probe's own bore. A roll about the bore leaves the
       cone and both distance gates exactly as they were, so the case measures spin and
       nothing else. The fixture used to supply that spin by accident -- the authored
       velocity was a few cm/s off, which drove the two port meshes through each other and
       let the contact torques tumble the pair. With the pair now co-moving as the world
       measures it, nothing is spinning unless a mode asks for it. */
    double spinDeg = 0.0;
    if(mode == "approach") { gap = 0.55; closeV = 0.09; }
    /* A pair 8 m apart, aligned and at rest, for a terminal approach flown by hand:
       RCS forward is N, up I, right L, and the Docking window's dial is the instrument.
       No case flies it -- it is the fixture the human test of #1152's numbers runs on. */
    else if(mode == "corridor") { gap = 8.0; }
    else if(mode == "spin") { gap = 8.0; spinDeg = 4.0; }
    else if(mode == "misaligned") { gap = 1.0; yawDeg = 40.0; }
    else if(mode == "stale-arm") { gap = 1.0; yawDeg = 40.0; shedArm = true; }
    else if(mode == "stale-target") { gap = 1.0; yawDeg = 40.0; shedTarget = true; }
    else if(mode != "near") {
        throw std::runtime_error("--dock-test: unknown mode '" + mode
                                 + "' (near | approach | corridor | spin | misaligned"
                                 + " | stale-arm | stale-target)");
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
    /* The port the probe TARGETS. In stale-target the station wears one more stack
       below its own tail port -- decoupler then port -- and that far one is the
       targeted half, so a stage press on THIS hull hands it to a new vessel while the
       station keeps the port the arc spacing below is built from. */
    Part *stTarget = stPort;
    if(shedTarget) {
        Part *stDec  = makePart(defDec);   // stage 1, like every part here
        Part *stTail = makePart(defPort);
        station->attachDown(stDec);
        station->attachDown(stTail);
        stTarget = stTail;
    }
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
    /* #1040: the ARMED half of the intent rides a stage that leaves. A second port, hung
       under a decoupler at the tail, is the port this mode arms -- so one stage press
       hands it to a new vessel (Game::stageVehicle -> extractSubtreeAsShip) while this
       hull keeps the nose port it was built with. Vehicle::dockSituation() then answers
       ArmGone, and what Game::updateDocking() does with that (drop the arm, keep the
       target) is readable on [dockwin]: the rows after the press blame the arm while b=
       still names dockstation.
       The nose port is deliberately left alone: the arc spacing below is built from it,
       and 1026's pair is the fixture this one borrows. */
    Part *prArm = prPort;
    if(shedArm) {
        Part *prDec  = makePart(defDec);   // stage 1, like every part here
        Part *prTail = makePart(defPort);  // armed, and on the side that gets cut
        probe->attachDown(prDec);
        probe->attachDown(prTail);
        prArm = prTail;
    }
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
    /* Inertial -> the ship's rotating frame (the inverse of comStateIn): rotate back
       and drop the frame offset. The probe's VELOCITY is not built the same way -- see
       the setVelocity below. */
    const glm::dmat3 Rinv = glm::transpose(station->frame->GetOrientRelTo(inert));
    const glm::dvec3 pOff = station->frame->GetPositionRelTo(inert);
    glm::dvec3 rP = Rinv * (rP_i - pOff);
    /* The docking reference is the PORT, so the spacing is authored at the port and the
       probe's COM is shifted until its port lands exactly on the station's port axis.
       The arc maths above is only the neighbourhood: it leaves centimetres of slop, and
       #1152's gate has no room for that -- at a 0.10 m capture distance and a 5 deg cone
       the entire lateral budget is 9 mm, and a 4 cm offset reads 16 deg of misalignment by
       the time a 0.2 m gap closes (measured, and the reason the approach fixture used to
       jam on its own port rims instead of mating). Place, read where the port went, place
       again shifted by the difference: COM to port is a translation, so one pass is exact. */
    const glm::dvec3 portB = station->partPos(stPort);
    const glm::dvec3 axisOut = glm::normalize(portB - rS);   // out of the station's tail
    const glm::dvec3 want = portB + gap * axisOut;           // the probe's port belongs here
    probe->placeShipAtCom(rP, faceAlong(-axisOut));
    rP += want - probe->partPos(prPort);
    /* Face the station: the probe's port is on its +Z, so aim +Z down the port axis
       for a clean aligned dock. */
    const glm::dvec3 faceDir = -axisOut;
    /* "misaligned" yaws that aim off the line, and takes the yaw AROUND THE PORT rather
       than around the COM: this is a seven-part stack whose port sits metres out from its
       own centre of mass, so a yaw about the COM swings the port more than a metre
       sideways -- which would fail the distance gates as well as the cone one and leave
       the fixture testing two things at once. Place, read where the port went, place
       again shifted by the difference.
       The angle is 40 deg, not the 16 that would do: nothing flies the probe, so the pair
       wanders, and a yaw set just outside the cone is inside it (or far outside it) by the
       time the window is on screen. Measured across several runs of the 3 s end-to-end
       case: the face gap the panel first prints is 1.2 m, the last line some 2.5 s later
       says 4.2 m (separating at 1.2 m/s), and the misalignment drifts from 38 to 46 deg.
       The verdict stays `misaligned` through all of it because the gate asks the axis
       question before the distance one -- the same order tests/test_dock.cpp pins. */
    probe->placeShipAtCom(rP, faceAlong(faceDir));      // the aligned pose
    if(yawDeg != 0.0) {
        const glm::dvec3 port0 = probe->partPos(prPort);    // where its port belongs
        const glm::dvec3 aim = rotH(faceDir, yawDeg * std::numbers::pi / 180.0);
        probe->placeShipAtCom(rP, faceAlong(aim));
        const glm::dvec3 port1 = probe->partPos(prPort);
        probe->placeShipAtCom(rP + (port0 - port1), faceAlong(aim));
    }
    probe->enterWorld();
    /* Co-moving means what the world measures, not what the arc maths implies: the probe
       takes the station's own hull velocity plus the scripted closing rate down the port
       axis. Deriving it instead as a rotated copy of the station's inertial state -- the
       textbook way onto the same circular orbit one phase angle behind -- left a relative
       velocity of about 0.04 m/s, almost all of it "up": the first panel line reads
       drift=-0.01,-0.04 and 8 s later the ports sit 0.35 m apart sideways at an 8 m range.
       The old 1.5 m gate hid that completely; at 0.10 m and 5 deg it is the entire
       lateral budget, and the approach fixture spent its closure jammed against the
       station's port rim instead of mating. */
    probe->setVelocity(GetVelocity(station->hull) + closeV * faceDir);
    if(spinDeg != 0.0) {
        /* Roll about the bore: that axis IS the probe's port axis, so the cone and both
           distance gates read exactly the figures they would at rest and the only thing the
           dial gains is twist= -- |w_a - w_b|, measured against a station that sits still. */
        SetAngVelocity(probe->hull, faceDir * (spinDeg * std::numbers::pi / 180.0));
    }

    /* Docking is intent-driven (Game::updateDocking needs BOTH halves): the
       probe's port is armed, the station's port targeted -- what the player
       does with "Arm for docking" / "Target for docking". In stale-arm the armed
       one is the tail port on the decoupler (#1040); in stale-target the TARGETED
       one is, and it is the other hull that wears the decoupler (#1062). */
    probe->dockArmPort = prArm;
    probe->dockTargetShip = station;
    probe->dockTargetPort = stTarget;

    /* The banner says what the FIXTURE asked for, not what the ports ended up at: the arc
       is built for the aligned stacks, so a yawed probe's face gap is whatever the yaw
       leaves (the Docking window prints the measured figure, and for `misaligned` it is
       not this number). Naming the yaw here is what keeps the two from looking like a
       contradiction in one log. */
    if(yawDeg == 0.0) {
        printf("Dock-test (%s): station '%s' + probe '%s', arc gap %.2f m, "
               "closing %.2f m/s, |v| = %.1f m/s\n",
               mode.c_str(), station->name.c_str(), probe->name.c_str(),
               gap, closeV, glm::length(GetVelocity(station->hull)));
    } else {
        printf("Dock-test (%s): station '%s' + probe '%s', arc gap %.2f m, "
               "yaw %.0f deg about the port, |v| = %.1f m/s\n",
               mode.c_str(), station->name.c_str(), probe->name.c_str(),
               gap, yawDeg, glm::length(GetVelocity(station->hull)));
    }
    /* A second line for stale-arm, naming the mechanism rather than the geometry: in
       this mode the question is not where the ports are, it is which half of the intent
       a stage press cuts off. What the gate then DOES with the half-left-over is the
       [dockwin] column's business, not this banner's. */
    if(shedArm) {
        printf("Dock-test (stale-arm): the armed port rides a stage-1 decoupler, so one "
               "press cuts it off this hull\n");
    }
    /* stale-target names the same mechanism one hull over, and says which hull has to
       be flying when the press comes: the gate only ever weighs the ACTIVE ship's
       intent, so the probe's rows reappear once control is handed back to it. */
    if(shedTarget) {
        printf("Dock-test (stale-target): the targeted port rides %s's stage-1 decoupler, "
               "so a press on that hull cuts it away from the intent\n",
               station->name.c_str());
    }
    /* And the spin, named the same way: what the FIXTURE asked for. The dial's twist= row
       is the measured half, and a case that wants to know the pair was ever spinning reads
       this line rather than inferring it from a nonzero figure (#1042). */
    if(spinDeg != 0.0) {
        printf("Dock-test (%s): the probe rolls at %.0f deg/s about its own bore\n",
               mode.c_str(), spinDeg);
    }

    DockTestShips d;
    d.probe = probe;
    d.station = station;
    return d;
}
