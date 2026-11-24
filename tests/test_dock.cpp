// Headless test for the docking merge/split primitives (src/vehicle.h):
// Vehicle::absorbShip (one ship rigidly absorbs another through a docking
// port) and Vehicle::extractSubtreeAsShip (the general "a part of this ship
// becomes a ship" split that undock -- and later dropped stages -- use).
//
// What is pinned:
//   - REBASE EXACTNESS: absorbShip is a rigid merge. Every absorbed part
//     keeps its exact WORLD pose (partWorldPose before == after), even though
//     its ship-local pose is re-based into the survivor's frame S and the
//     survivor's COM (hence hull transform) shifts to take on the new mass.
//     The re-base is pinned by value too: Part::localPos lives in the OWNING
//     vehicle's S, so B's root leaves S_B's origin for z = -1.875 in A's S.
//   - MOMENTUM (#366): the capture is inelastic and mass-weighted. The merged hull leaves
//     at (mA*vA + mB*vB)/(mA + mB), read off the hull the game actually flies after
//     rebuildCompound. Nothing else in the suite crosses a merge with velocity, and a
//     capture that braked or flung the pair feels like normal flying while every later
//     delta-v plan is wrong.
//   - ANGULAR MOMENTUM (#1053): the other half of the same sentence. The merge may lose
//     each hull's OWN spin, but not the pair's L about their shared centre -- spin terms
//     AND the r x m v of an off-axis capture -- and the merged hull's re-solved inertia is
//     what divides it. This used to be `SetAngVelocity(hull, wA)`, which deleted the
//     absorbed hull's rotation at contact.
//   - SEAM: the joint is recorded (survivor port, absorbed root, its name),
//     which is what undock splits back apart.
//   - TOPOLOGY: the absorbed ship's root now hangs off the survivor's port;
//     the part list is the union.
//   - STAGE COUNTERS: finalize()'s scan over the part list counts EVERY part's stage
//     label, the root's included (#519). It lands in this file because it needs the
//     body-backed ships (finalize() ends in rebuildCompound), which test_staging.cpp
//     deliberately does not have.
//   - ROUND-TRIP: absorbShip then extractSubtreeAsShip(seam.root) restores
//     BOTH ships' parts to their original world poses -- the split is the
//     exact inverse of the merge on geometry. This is the invariant the
//     future "dropped stage becomes a ship" relies on.
//   - THE MERGED MASS (#583): Vehicle::refuseDockWith, the sentence the dock gate
//     prints instead of folding two totals that stop being one. A range check, not a
//     size limit: the pair that still fits has to merge.
//   - THE CAPTURE DISTANCE (#1152): Vehicle::dockCaptureRange -- how close the faces have
//     to be before the gate closes, scaled off the shallower port's own depth with a floor
//     at the pair's collision standoff. The old global 1.5 m is what made a docked pair
//     look like two ships holding each other at arm's length, because absorbShip welds the
//     pair at whatever gap the capture tick measured.
//   - THE SITUATION OF AN INTENT (#1026): Vehicle::dockRead's figures and
//     Vehicle::dockSituation's verdict -- the one pass that now decides a dock (the gate
//     switches on it) AND feeds the Docking window, so the numbers a pilot reads are the
//     numbers that refused them. Pinned at the cone boundary from the constant, for the
//     signed nudge rows that faceGap alone cannot give, for the ORDER the questions are
//     asked in (axis, then distance, then speed, then the mass door -- a pair that is far
//     away is told to close, not that it is too heavy), and for the rule that a port's
//     speed is its PORT's: a hull spinning about a still centre still offers a port that
//     is moving. And for WHICH FRAME those figures speak in (#1044): the turn and push
//     pairs are resolved on attitudeRef()'s axes, because that is where the keys live, so
//     a port rolled 90 deg off its hull rotates the dial without moving a single axis
//     figure.
//   - IDENTITY: Part::uid is minted per instance and stays distinct across a
//     merge of two ships whose Part::ids collide on EVERY part -- the case
//     two ships built from one def produce, since absorbShip does not rename.
//     `id` is provably ambiguous there (asserted, not assumed) and uid is
//     not, which is what makes uid the key save/load resolves by. Also pins
//     that a split neither remints nor reuses a uid.
//
// No physics world and no GL context: ships are built with init() (which
// runs rebuildCompound -- the one hull rigid body -- but NOT enterWorld) and
// placed with placeShip, exactly like test_inertia/test_fuel.
// extractSubtreeAsShip deliberately leaves enterWorld to its caller so this
// split can run headless.
//
// Runs from the repo root:
//   make test   (or: ./test_dock)

#include "btcommon.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <deque>
#include <numbers>
#include <set>
#include <string>
#include <vector>

#include "vehicle.h"   // Vehicle + Part (inline); pulls in Bullet (btcommon.h)
#include "tfueltable.h"   // the group-table-vs-scan check (#984)
#include "frame.h"     // Frame: the dock gate will not measure across two of them
#include "physics.h"   // SetAngVelocity: a spinning hull, a moving port (#1026)
                           // SetVelocity/GetVelocity: the merge keeps m*v (#366)
#include "tchecks.h"

/* A hand-built ship. `defs` is a deque, not a vector: push_back on a vector
   reallocates and would dangle every Part::def handed out so far. */
struct Ship {
    Vehicle *v;
    std::deque<PartDef> defs;
};

/* Where a part sits in Vehicle::parts. Attaching here is by index, and #544's pins are
   ABOUT that index, so the fixtures must not hard-code a position that adding a part
   shifts. */
static size_t partIndex(Vehicle *v, Part *p) {
    std::vector<Part *>::iterator it = std::find(v->parts.begin(), v->parts.end(), p);
    assert(it != v->parts.end());
    return (size_t)(it - v->parts.begin());
}

/* A box part (no rigid body of its own -- the ship is ONE body and the part
   is a child of its compound, carrying a collision shape, a mass and an
   authored pose). `port` marks a docking port. Parent/local pose are set by
   setRoot/attachDown, not here. */
static Part *mkPart(Ship &s, const char *name, double mass,
                    double hx, double hy, double hz, bool port,
                    bool decoupler = false, double h2 = 0.0) {
    Body *b = new Body;
    // mesh/shader/texture default to null (no GL in a headless test)
    b->btBody = nullptr;                 // a part is not simulated on its own
    b->shape  = new btBoxShape(btVector3(hx, hy, hz));
    b->mass   = mass;

    s.defs.push_back(PartDef());
    PartDef &d = s.defs.back();
    d.name         = name;
    d.mass         = (float)mass;
    d.radius       = (float)hx;
    d.height       = (float)(2.0 * hz);
    d.docking_port = port;
    d.decoupler    = decoupler;
    /* Part::def is const, so a fixture that wants a real tank has to say so here:
       Part::isTank() reads capacity, and the dry default leaves every part in this
       file out of the fuel-group table, checks included (#984). */
    d.capacity[(int)ResourceType::Hydrogen] = h2;
    d.synthesizeNodes();   // hand-built def: give it the axial stack nodes
                           // attachDown mates (a catalog part gets these from
                           // load_parts_catalog)

    Part *p = new Part;
    p->body  = b;
    p->def   = &d;
    p->stage = 1;
    return p;
}

/* tank (root) + docking port stacked on its rear (-Z) face -- the station. */
static const double kTankHz = 1.5;    // half-height (3 m tank)
static const double kPortHz = 0.125;  // half-height (0.25 m port)

static void destroyShip(Vehicle *v) {
    /* onRails keeps ~Vehicle off RemoveBody: nothing here was ever added to
       a physics world (there is none in this test). */
    v->onRails = true;
    delete v;
}

/* A part's world position, via the ship that currently owns it. */
static glm::dvec3 worldPos(Vehicle *owner, Part *p) {
    glm::dvec3 pos; glm::dmat3 rot;
    owner->partWorldPose(p, pos, rot);
    return pos;
}

/* --- the stage counters finalize() computes (#519) -------------------------

   finalize() scans every part to find the highest and lowest stage labels, and the
   high end used to be SEEDED AT 1 while the scan started at index 1: the root part's
   own label was never compared, so a ship whose root carries the top label never had
   that label counted. Here that means droppedPartsAtStage(top) drops nothing, because
   the counter never reaches it -- in flight the same split shows up as a part that is
   live from the first frame (producedThrust skips only `stage < as`, and the counter only
   falls), a decoupler and chute that never fire (both test `stage == stage`), and an
   HUD `Stage: x / y` that disagrees with the part popup's own number.

   It lives here rather than in test_staging.cpp because finalize() ends in
   rebuildCompound(), which needs a real collision shape per part -- test_staging.cpp
   builds ships with body = nullptr on purpose, and segfaults on this path (measured).
   Reachability from that file is one reason extractSubtreeAsShip (vehicle.cpp:3793)
   calls finalize() on its new ship, which is why it needs the shapes too.

   The fixture is deliberately lopsided: the TOP label belongs to the root part ALONE,
   because that is the only arrangement the old scan could miss (it started at index 1).
   A first draft of this test gave the root and its decoupler the same label and stayed
   green with the bug reinstalled -- mutation-checked, as usual. A second draft pinned
   five of its checks against a hard-coded label instead of the live counter, so they
   passed with the bug back in; every check below now asks activeStage() where the label
   lives, and the ladder carries a SECOND decoupler on the LOWEST label so the bottom of
   the sequence is a fact rather than a tautology. */
static void test_the_root_parts_stage_is_counted() {
    printf("== finalize(): every part's stage label is counted (#519) ==\n");
    Ship s; s.v = new Vehicle; s.v->name = "toplabel";
    Part *caps = mkPart(s, "capsuleTop", 800.0, 1.0, 1.0, 1.0, false);
    Part *dec3 = mkPart(s, "dec3",        50.0, 1.0, 1.0, kPortHz, false, true);
    Part *tank = mkPart(s, "tank2",     1000.0, 1.0, 1.0, kTankHz, false);
    Part *eng  = mkPart(s, "engine1",   500.0, 1.0, 1.0, 1.0, false);
    Part *dec1 = mkPart(s, "decLow",     50.0, 1.0, 1.0, kPortHz, false, true);
    Part *pod  = mkPart(s, "podSmall",   300.0, 1.0, 1.0, 1.0, false);
    s.v->setRoot(caps);
    s.v->attachDown(dec3);
    s.v->attachDown(tank);
    s.v->attachDown(eng);
    // dec1 hangs off the capsule too -- the ladder's bottom rung, not inside dec3's
    // subtree, or the label-3 drop would take it and the bottom of the sequence would
    // never be reached.
    s.v->attachMode(dec1, partIndex(s.v, caps), AttachMode::Down);
    s.v->attachDown(pod);          // hangs off dec1, the part just appended
    // 4 / 3 / 2 / 1, with the top label on the ROOT alone and a decoupler at BOTH ends.
    caps->stage = 4;  dec3->stage = 3;  tank->stage = 2;  eng->stage = 1;
    dec1->stage = 1;  pod->stage = 1;
    s.v->init();

    CHECK_TRUE(s.v->numStages() == 4, "the root's label is the top of the counter (#519)");
    CHECK_TRUE(s.v->activeStage() == 4, "the counter starts on the highest label, the root's");
    CHECK_TRUE(s.v->stagedDecouplers(s.v->activeStage()).empty(),
               "the pad lights no decoupler: the top label is the root's own");

    /* Walk the whole sequence by the counter, never by a literal label: with the bug
       reinstalled the counter starts one label low, and a check written against `3`
       still passes while one written against activeStage() cannot. */
    struct Rung { int stage; const char *dec; size_t drop; };
    const Rung rungs[] = {{3, "dec3", 3}, {2, NULL, 0}, {1, "decLow", 2}};
    for(size_t k = 0; k < 3; k++) {
        s.v->advanceStage();
        const int as = s.v->activeStage();
        CHECK_TRUE(as == rungs[k].stage, "the counter walks one label per press");
        const std::vector<Part *> lit = s.v->stagedDecouplers(as);
        if(rungs[k].dec == NULL) {
            CHECK_TRUE(lit.empty(), "this label carries no decoupler, so nothing fires");
        } else {
            CHECK_TRUE(lit.size() == 1 && lit[0]->def->name == std::string(rungs[k].dec),
                       "this label lights exactly its own decoupler");
        }
        const std::vector<Part *> gone = s.v->droppedPartsAtStage(as);
        CHECK_TRUE(gone.size() == rungs[k].drop,
                   "and the drop walk takes that label's subtree, no more and no less");
    }
    s.v->advanceStage();
    CHECK_TRUE(s.v->activeStage() == 1, "the counter stops at minStage_");
    destroyShip(s.v);

    // One part, labelled above 1: it IS the ship, so it is the whole count.
    Ship solo; solo.v = new Vehicle; solo.v->name = "solo";
    Part *only = mkPart(solo, "solo", 500.0, 1.0, 1.0, 1.0, false);
    solo.v->setRoot(only);
    only->stage = 7;
    solo.v->init();
    CHECK_TRUE(solo.v->numStages() == 7 && solo.v->activeStage() == 7,
               "a one-part ship is staged to its own label, whatever it is");
    destroyShip(solo.v);
}

/* --- absorb: rebase exactness + seam + topology -------------------------- */
static void test_absorb() {
    printf("== absorbShip: rigid rebase, seam, topology ==\n");

    /* A: tank (root) + port below it, at the world origin. */
    Ship A; A.v = new Vehicle; A.v->name = "A";
    /* The names promise tanks, and the table check below needs a member to find:
       Part::isTank() reads capacity, so a dry fixture leaves the group table
       legitimately empty on both sides of the merge -- a check that can never fail
       (#984). 100 kg of hydrogen each, which lands in hull->mass (part.h
       effectiveMass) and so in the merged total at the end. */
    Part *tankA = mkPart(A, "tankA", 1000.0, 1.0, 1.0, kTankHz, false, false, 100.0);
    Part *portA = mkPart(A, "portA",   50.0, 1.0, 1.0, kPortHz, true);
    A.v->setRoot(tankA);
    A.v->attachDown(portA);          // portA at local z = -(3.0+0.25)/2 = -1.625
    A.v->controller = tankA;
    A.v->init();
    A.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    /* B: port (root) + tank below it, placed so B's port face meets A's port
       face (A's port rear face is at world z = -1.625 - 0.125 = -1.75; B's
       port front face is at its origin + 0.125, so B sits at z = -1.875). */
    Ship B; B.v = new Vehicle; B.v->name = "B";
    Part *portB = mkPart(B, "portB",   50.0, 1.0, 1.0, kPortHz, true);
    Part *tankB = mkPart(B, "tankB", 1000.0, 1.0, 1.0, kTankHz, false, false, 100.0);
    B.v->setRoot(portB);
    B.v->attachDown(tankB);          // tankB at local z = -1.625
    B.v->controller = portB;
    B.v->init();
    B.v->placeShip(glm::dvec3(0.0, 0.0, -1.875), glm::dmat3(1.0));

    /* Capture B's world poses before the merge; they must survive it. */
    const glm::dvec3 portB0 = worldPos(B.v, portB);
    const glm::dvec3 tankB0 = worldPos(B.v, tankB);
    const glm::dvec3 tankA0 = worldPos(A.v, tankA);
    const glm::dvec3 portA0 = worldPos(A.v, portA);
    /* ...and B's ship-local poses, which must NOT survive it: localPos lives
       in the owning vehicle's S, so B's root sits at the origin of S_B only
       while B owns it. */
    const glm::dvec3 portBlocal0 = portB->localPos;
    const glm::dvec3 tankBlocal0 = tankB->localPos;

    /* THE ORDERS DO NOT CROSS A DOCK, in the merge direction too (#649's law, and
       #768's nozzles ride it). The split direction is pinned below -- staging keeps a
       pilot's switches as left -- and this is the other half: the hull that does the
       absorbing flies the merge, and the absorbed hull's switches die with it. Without
       it, docking a drums-off probe to a drums-on station would shut the station's drums
       on contact. */
    B.v->wheels_enabled = false;
    B.v->surfaces_enabled = false;
    B.v->tvc_enabled = false;
    A.v->absorbShip(B.v, portA);
    CHECK_TRUE(A.v->wheels_enabled && A.v->surfaces_enabled && A.v->tvc_enabled,
               "absorb: the survivor keeps its OWN orders, whatever the absorbed hull had");

    /* Union of the parts; B is left an empty shell. */
    CHECK_TRUE(A.v->parts.size() == 4, "absorb: survivor holds all four parts");
    CHECK_TRUE(B.v->parts.empty(), "absorb: absorbed ship is an empty shell");
    CHECK_TRUE(B.v->memberStart_.size() == 1 && B.v->memberOf_.empty()
               && B.v->drainLayers_.empty(),
               "and the shell keeps no fuel cache whose indices would name parts it no longer has (#984)");

    /* ...and the fuel cache comes out of the merge whole (#984). A dock renumbers the
       groups -- both hulls numbered theirs from 0 -- so the survivor rebuilds the table
       the drain walks. The stale one is A's pre-merge table, which lists only A's tank:
       the absorbed tank would belong to a group nobody walks, so a merged engine could
       not drink from it, and nothing else about the merge would notice. */
    checkFuelTableMatchesScan(A.v, "after the dock");
    CHECK_TRUE(A.v->memberStart_.size() == 2
               && A.v->memberStart_[1] - A.v->memberStart_[0] == 2,
               "and both tanks are on that list: these hand-built ports carry no fuel_barrier, "
               "so the merged hull is ONE group holding both");

    /* Topology: B's root now hangs off A's port. */
    CHECK_TRUE(portB->parent == portA, "absorb: absorbed root hangs off the port");

    /* Seam recorded for undock. */
    CHECK_TRUE(A.v->seams.size() == 1, "absorb: one seam recorded");
    if(A.v->seams.size() == 1) {
        CHECK_TRUE(A.v->seams[0].port == portA, "absorb: seam port is the survivor's");
        CHECK_TRUE(A.v->seams[0].root == portB, "absorb: seam root is the absorbed root");
        CHECK_TRUE(A.v->seams[0].name == "B", "absorb: seam remembers the absorbed name");
    }

    /* Rebase exactness: every part keeps its world pose (the merge is rigid;
       only the ship-local poses and the survivor's COM/hull transform move). */
    const glm::dvec3 tankA1 = worldPos(A.v, tankA);
    const glm::dvec3 portA1 = worldPos(A.v, portA);
    const glm::dvec3 portB1 = worldPos(A.v, portB);
    const glm::dvec3 tankB1 = worldPos(A.v, tankB);
    CHECK_NEAR(glm::length(tankA1 - tankA0), 0.0, 1e-9, "absorb: tankA world pose kept");
    CHECK_NEAR(glm::length(portA1 - portA0), 0.0, 1e-9, "absorb: portA world pose kept");
    CHECK_NEAR(glm::length(portB1 - portB0), 0.0, 1e-9, "absorb: portB world pose kept");
    CHECK_NEAR(glm::length(tankB1 - tankB0), 0.0, 1e-9, "absorb: tankB world pose kept");

    /* The frame convention, by value: localPos is in the OWNING vehicle's S,
       so the merge re-bases B's parts into A's. S_A sits at the world origin
       and S_B at z = -1.875, both identity, so the re-base is a pure -1.875
       shift: B's root -- authored at its own S origin -- lands at z = -1.875,
       and its tank a tank-stack below that. The parent edge (portB->parent ==
       portA) carries no pose of its own. */
    CHECK_NEAR(glm::length(portBlocal0), 0.0, 1e-9,
               "absorb: B's root starts at S_B's origin");
    CHECK_NEAR(glm::length(tankBlocal0 - glm::dvec3(0.0, 0.0, -1.625)), 0.0, 1e-9,
               "absorb: B's tank starts in S_B");
    CHECK_NEAR(glm::length(portB->localPos - glm::dvec3(0.0, 0.0, -1.875)), 0.0, 1e-9,
               "absorb: absorbed root re-based into the survivor's S");
    CHECK_NEAR(glm::length(tankB->localPos - glm::dvec3(0.0, 0.0, -3.5)), 0.0, 1e-9,
               "absorb: absorbed child re-based into the survivor's S");

    /* The merged COM sits between the two tanks' COMs (mass-weighted), and
       the hull mass is the sum -- a sanity check that rebuildCompound ran. The sum
       is effectiveMass (part.h), so it carries the 100 kg each tank holds above. */
    CHECK_NEAR(A.v->hull->mass, 2300.0, 1e-6, "absorb: hull mass is the union, fuel included");

    destroyShip(B.v);   // empty shell (its old compound references, not owns)
    destroyShip(A.v);
}

/* --- the capture keeps the pair's momentum (#366) ------------------------- */
/* Two hulls become one and the one must leave where their masses said, not where either
   of them was going. The merge is inelastic by design -- a dock does not bounce -- but it
   is still a law, and it was unasserted: a merge that kept the survivor's v, or averaged
   the two hulls equally, or wrote the blend before rebuildCompound swapped the hull out
   from under it, all feel identical in flight and corrupt every delta-v plan measured
   from the capture. The angular half has its own test below (#1053). */
static void test_a_dock_keeps_the_pair_s_momentum() {
    printf("== a dock keeps the pair's m*v ==\n");

    /* Unequal masses on purpose. With equal ones, "keep the survivor's v", "split the
       difference" and "mass-weight" differ only in the size of the answer, and a midpoint
       looks plausible enough to ship. */
    Ship A; A.v = new Vehicle; A.v->name = "A";
    Part *tankA = mkPart(A, "tankA", 1000.0, 1.0, 1.0, kTankHz, false);
    Part *portA = mkPart(A, "portA",   50.0, 1.0, 1.0, kPortHz, true);
    A.v->setRoot(tankA);
    A.v->attachDown(portA);
    A.v->controller = tankA;
    A.v->init();
    A.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    Ship B; B.v = new Vehicle; B.v->name = "B";
    Part *portB = mkPart(B, "portB",   50.0, 1.0, 1.0, kPortHz, true);
    Part *tankB = mkPart(B, "tankB",  200.0, 1.0, 1.0, kTankHz, false);
    B.v->setRoot(portB);
    B.v->attachDown(tankB);
    B.v->controller = portB;
    B.v->init();
    B.v->placeShip(glm::dvec3(0.0, 0.0, -1.875), glm::dmat3(1.0));

    const double mA = A.v->getMass();
    const double mB = B.v->getMass();
    CHECK_NEAR(mA, 1050.0, 1e-9, "momentum: the absorbing hull weighs its parts");
    CHECK_NEAR(mB, 250.0, 1e-9, "momentum: the absorbed hull weighs its parts");

    /* Three live components, not one axis: a merge that dropped or rescaled a single
       axis would keep passing an along-one-axis check. The numbers are orbital-scale
       (a pair co-moving near Kerbin's ~7.8 km/s) with the closing rate inside the gate. */
    const glm::dvec3 vA(1200.0, -3.5, 7.25);
    const glm::dvec3 vB(1150.0, 6.0, -4.0);
    SetVelocity(A.v->hull, vA);
    SetVelocity(B.v->hull, vB);

    A.v->absorbShip(B.v, portA);

    const glm::dvec3 want = (mA * vA + mB * vB) / (mA + mB);
    const glm::dvec3 got = GetVelocity(A.v->hull);
    CHECK_NEAR(got.x, want.x, 1e-9, "absorb: x momentum kept");
    CHECK_NEAR(got.y, want.y, 1e-9, "absorb: y momentum kept");
    CHECK_NEAR(got.z, want.z, 1e-9, "absorb: z momentum kept");
    /* Said as values, because the shape of the answer is the point: 1190.3846... is
       nearer A's 1200 than B's 1150 by exactly the mass ratio. "Keep the survivor's"
       would read 1200, "average the hulls" 1175, "weight by part count" something
       else again -- and only the first two look like nothing happened. */
    CHECK_NEAR(want.x, 1547500.0 / 1300.0, 1e-9, "absorb: the blend is mass-weighted");

    destroyShip(B.v);
    destroyShip(A.v);
}

/* --- the capture keeps the pair's angular momentum (#1053) -----------------
   The merge used to end `SetAngVelocity(hull, wA)`: the survivor's spin, the absorbed
   hull's gone. A rigid weld is entitled to lose each body's OWN spin -- that is what
   inelastic means, and it is why the linear half blends instead of bouncing -- but it is
   not entitled to lose the pair's angular momentum. Station-keeping is the player-facing
   case: dock a still probe nose-first to a hull that is turning, and the turn used to be
   deleted on contact, with no torque anywhere to have caused it.

   The pin is CONSERVATION rather than a number, because the merged hull's inertia comes
   out of calculatePrincipalAxisTransform and the test has no business repeating that
   solve: it reads the same tensor the merge divides by. */
static void test_a_dock_keeps_the_pair_s_spin() {
    printf("== a dock keeps the pair's angular momentum ==\n");

    Ship A; A.v = new Vehicle; A.v->name = "A";
    Part *tankA = mkPart(A, "tankA", 1000.0, 1.0, 1.0, kTankHz, false);
    Part *portA = mkPart(A, "portA",   50.0, 1.0, 1.0, kPortHz, true);
    A.v->setRoot(tankA);
    A.v->attachDown(portA);
    A.v->controller = tankA;
    A.v->init();
    A.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    Ship B; B.v = new Vehicle; B.v->name = "B";
    Part *portB = mkPart(B, "portB",   50.0, 1.0, 1.0, kPortHz, true);
    Part *tankB = mkPart(B, "tankB",  200.0, 1.0, 1.0, kTankHz, false);
    B.v->setRoot(portB);
    B.v->attachDown(tankB);
    B.v->controller = portB;
    B.v->init();
    B.v->placeShip(glm::dvec3(0.0, 0.0, -1.875), glm::dmat3(1.0));

    const double mA = A.v->getMass();
    const double mB = B.v->getMass();

    /* Opposite spins about the port axis -- the ticket's repro. The signs are the point:
       a merge that kept the survivor's spin and one that split the difference both look
       plausible inside a same-sign pair, while 0.5 and -0.4 cannot be confused. */
    const glm::dvec3 wA(0.0, 0.0, 0.5);
    const glm::dvec3 wB(0.0, 0.0, -0.4);
    SetAngVelocity(A.v->hull, wA);
    SetAngVelocity(B.v->hull, wB);
    /* And a closing rate with a lateral component, so the capture is not purely head-on:
       that is what makes the orbital term r x m v nonzero about the shared centre, and it
       is the half of L that a spin-only merge would still get wrong. */
    const glm::dvec3 vA(0.0, 0.6, -0.9);
    const glm::dvec3 vB(0.0, -0.4, 0.3);
    SetVelocity(A.v->hull, vA);
    SetVelocity(B.v->hull, vB);

    /* Read everything while there are still two hulls to read it from. */
    const glm::dmat3 iA = A.v->hullInertiaWorld();
    const glm::dmat3 iB = B.v->hullInertiaWorld();
    const glm::dvec3 cA = A.v->comPos();
    const glm::dvec3 cB = B.v->comPos();
    const glm::dvec3 com = (mA * cA + mB * cB) / (mA + mB);
    const glm::dvec3 vOut = (mA * vA + mB * vB) / (mA + mB);
    const glm::dvec3 Lwant = iA * wA + iB * wB
                           + glm::cross(cA - com, mA * (vA - vOut))
                           + glm::cross(cB - com, mB * (vB - vOut));
    CHECK_TRUE(std::fabs(Lwant.z) > 1.0,
               "the pair really does carry spin angular momentum here");

    A.v->absorbShip(B.v, portA);

    const glm::dvec3 w = GetAngVelocity(A.v->hull);
    const glm::dvec3 Lgot = A.v->hullInertiaWorld() * w;
    CHECK_REL(Lgot.x, Lwant.x, 1e-9, "absorb: x angular momentum kept");
    CHECK_REL(Lgot.y, Lwant.y, 1e-9, "absorb: y angular momentum kept");
    CHECK_REL(Lgot.z, Lwant.z, 1e-9, "absorb: z angular momentum kept");

    /* Said as a rate too, because that is what the pilot sees: B's -0.4 has to land in
       the answer, so the merged spin sits between the two and is NOT the survivor's. */
    CHECK_TRUE(w.z < 0.5 && w.z > -0.4,
               "the merged spin lies between the pair's, not at one end of them");
    CHECK_TRUE(std::fabs(w.z - wA.z) > 1e-3,
               "and it is not the survivor's spin, which is what the merge used to write");
    /* The lateral closing has to show up as a turn about a DIFFERENT axis than the spins:
       without the orbital terms this component would be 0 on both hulls' z-axis spin. */
    CHECK_TRUE(std::fabs(w.x) > 1e-4,
               "the off-axis capture starts the merged hull turning about x as well");
    /* The figures, printed: the old merge answered exactly wA for all three. */
    printf("merged spin (%.4f, %.4f, %.4f) rad/s from A (%.1f, %.1f, %.1f) and B (%.1f, %.1f, %.1f)\n",
           w.x, w.y, w.z, wA.x, wA.y, wA.z, wB.x, wB.y, wB.z);
    printf("L about the shared centre: (%.1f, %.1f, %.1f) kg m^2/s, kept to %.4g\n",
           Lwant.x, Lwant.y, Lwant.z, glm::length(Lgot - Lwant));

    destroyShip(B.v);
    destroyShip(A.v);
}

/* --- the orders a dock froze come back with the hull that had them (#657) --- */
/* The ticket's repro, in one merge and one undock: dock a probe whose pilot shut every
   switch to a station that has them all open, undock, and the probe must not come back
   flying on the station's orders. absorbShip pins the merge direction (A keeps A's);
   test_seam_staging pins the other split rule (a decoupler's subtree keeps the flying
   hull's). What was missing is the third case: an UNDOCK, where the hull leaving was a
   vessel in its own right and its orders waited on the seam. */
static void test_orders_return_on_undock() {
    printf("== a docked vessel's orders survive the merge and return on undock ==\n");

    Ship A; A.v = new Vehicle; A.v->name = "A";
    Part *tankA = mkPart(A, "tankA", 1000.0, 1.0, 1.0, kTankHz, false);
    Part *portA = mkPart(A, "portA",   50.0, 1.0, 1.0, kPortHz, true);
    A.v->setRoot(tankA);
    A.v->attachDown(portA);
    A.v->controller = tankA;
    A.v->init();
    A.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    Ship B; B.v = new Vehicle; B.v->name = "B";
    Part *portB = mkPart(B, "portB",   50.0, 1.0, 1.0, kPortHz, true);
    Part *tankB = mkPart(B, "tankB", 1000.0, 1.0, 1.0, kTankHz, false);
    B.v->setRoot(portB);
    B.v->attachDown(tankB);
    B.v->controller = portB;
    B.v->init();
    B.v->placeShip(glm::dvec3(0.0, 0.0, -1.875), glm::dmat3(1.0));

    /* Everything the pilot can shut on that probe, and its dial (#678). */
    B.v->wheels_enabled = false;
    B.v->surfaces_enabled = false;
    B.v->rcs_attitude_enabled = false;   // #595
    B.v->tvc_enabled = false;            // #768
    B.v->sas_turn_rate = 0.75;
    B.v->rcs_thrust_frac = 0.4;    // #1067's dial rides the same rail
    B.v->rcs_attitude_frac = 0.25;  // #1124's second dial, a DIFFERENT figure so a
                                    // swapped copy cannot pass for the right one
    /* A's own dial, DIFFERENT from B's, so the checks on A below can tell a leak from
       B apart from a reset of A: both readings would land on the same figure otherwise. */
    A.v->sas_turn_rate = 1.5;
    A.v->rcs_thrust_frac = 0.9;
    A.v->rcs_attitude_frac = 0.8;

    A.v->absorbShip(B.v, portA);
    CHECK_TRUE(A.v->wheels_enabled && A.v->surfaces_enabled && A.v->rcs_attitude_enabled
               && A.v->tvc_enabled,
               "undock orders: the merged hull is still flown on the survivor's switches");

    /* Frozen ON the seam: not applied to the merge, not lost with the hull. */
    CHECK_TRUE(A.v->seams.size() == 1, "undock orders: one seam recorded");
    if(A.v->seams.size() == 1) {
        const Vehicle::DockSeam &s = A.v->seams[0];
        CHECK_TRUE(!s.wheels_enabled && !s.surfaces_enabled && !s.rcs_attitude_enabled
                   && !s.tvc_enabled, "the seam holds the absorbed hull's orders, not ours");
        CHECK_NEAR(s.sas_turn_rate, 0.75, 1e-9, "and its turn-rate ceiling");
        CHECK_NEAR(s.rcs_thrust_frac, 0.4, 1e-9, "and its RCS authority");
        CHECK_NEAR(s.rcs_attitude_frac, 0.25, 1e-9,
                   "and its RCS pointing dial (#1124), which is a separate figure");
    }

    Vehicle *out = A.v->extractSubtreeAsShip(portB, "B");   // Game::undock's call
    CHECK_TRUE(out != nullptr, "undock orders: the split succeeded");
    if(out == nullptr) { destroyShip(B.v); destroyShip(A.v); return; }
    CHECK_TRUE(!out->wheels_enabled && !out->surfaces_enabled && !out->rcs_attitude_enabled
               && !out->tvc_enabled,
               "the probe comes back with its switches as ITS pilot left them (#657)");
    CHECK_NEAR(out->sas_turn_rate, 0.75, 1e-9, "and comes back capped as it was capped");
    CHECK_NEAR(out->rcs_thrust_frac, 0.4, 1e-9, "and with the RCS authority it was dialed to");
    CHECK_NEAR(out->rcs_attitude_frac, 0.25, 1e-9,
               "and the pointing dial the seam froze for it (#1124)");
    CHECK_TRUE(A.v->wheels_enabled && A.v->surfaces_enabled && A.v->rcs_attitude_enabled
               && A.v->tvc_enabled, "the station keeps its own orders throughout");
    CHECK_NEAR(A.v->sas_turn_rate, 1.5, 1e-9, "and its dial is neither overwritten nor cleared");
    CHECK_NEAR(A.v->rcs_thrust_frac, 0.9, 1e-9, "and its RCS dial neither");
    CHECK_NEAR(A.v->rcs_attitude_frac, 0.8, 1e-9, "and neither its pointing dial");
    CHECK_TRUE(A.v->seams.empty() && out->seams.empty(), "the cut seam is on neither side");

    destroyShip(out);
    destroyShip(B.v);   // the empty shell absorbShip left behind
    destroyShip(A.v);
}

/* --- a stage press whose decoupler IS the seam root (#657) ------------------- */
/* Game::stageVehicle extracts at the DECOUPLER itself, not at its child, so a docked
   vessel whose own root part is the staged decoupler puts a stage press exactly on the
   joint absorbShip recorded. Reaching it needs a vessel rooted on a decoupler: the
   catalog's port and decoupler defs are disjoint (3 and 4 of them), and of the 32
   res/ships hulls none is -- their roots are capsules, tanks, an RCS pod, a kerbal and
   once a docking port -- but the VAB's root part is simply whatever was placed first, so
   a player build can be. The seam-root rule answers it the same way it answers an
   undock, and that is the right answer rather than an accident: same edge, same
   switches. Pinned so the rule is what the code says, not a guess about what today's
   catalog can reach. */
static void test_a_stage_cut_at_a_seam_root() {
    printf("== a stage press whose decoupler IS the seam root ==\n");

    Ship A; A.v = new Vehicle; A.v->name = "A";
    Part *tankA = mkPart(A, "tankA", 1000.0, 1.0, 1.0, kTankHz, false);
    Part *portA = mkPart(A, "portA",   50.0, 1.0, 1.0, kPortHz, true);
    A.v->setRoot(tankA);
    A.v->attachDown(portA);
    A.v->controller = tankA;
    A.v->init();
    A.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    /* B, rooted on its own decoupler: the decoupler is parts[0], so it is what
       absorbShip hangs off A's port and therefore what the seam names as `root`. */
    Ship B; B.v = new Vehicle; B.v->name = "B";
    Part *decopB = mkPart(B, "decoupler", 50.0, 1.0, 1.0, kPortHz, false, true);
    Part *tankB  = mkPart(B, "tankB",   1000.0, 1.0, 1.0, kTankHz, false);
    B.v->setRoot(decopB);
    B.v->attachDown(tankB);
    B.v->controller = decopB;
    B.v->init();
    B.v->placeShip(glm::dvec3(0.0, 0.0, -1.875), glm::dmat3(1.0));

    B.v->tvc_enabled = false;      // the probe's pilot shut its nozzles
    B.v->sas_turn_rate = 0.6;
    A.v->sas_turn_rate = 1.5;      // the station's own, different

    A.v->absorbShip(B.v, portA);
    CHECK_TRUE(A.v->seams.size() == 1 && A.v->seams[0].root == decopB,
               "stage-at-root: the seam's root IS the decoupler");

    Vehicle *out = A.v->extractSubtreeAsShip(decopB, "staged");   // the press's call
    CHECK_TRUE(out != nullptr, "stage-at-root: the split succeeded");
    if(out == nullptr) { destroyShip(B.v); destroyShip(A.v); return; }
    CHECK_TRUE(!out->tvc_enabled,
               "the staged hull leaves with the docked vessel's shut door, not ours");
    CHECK_NEAR(out->sas_turn_rate, 0.6, 1e-9, "and with the ceiling that vessel was capped at");
    CHECK_TRUE(A.v->tvc_enabled, "the survivor keeps its own open door");
    CHECK_NEAR(A.v->sas_turn_rate, 1.5, 1e-9, "and its own ceiling");
    CHECK_TRUE(A.v->seams.empty() && out->seams.empty(), "the cut seam is on neither side");

    destroyShip(out);
    destroyShip(B.v);   // the empty shell absorbShip left behind
    destroyShip(A.v);
}

/* --- what a seam with NO records hands back (#657) --------------------------- */
/* A seam written before #657 carries the constructor defaults, and so does one made by
   a vessel whose pilot had everything open. In both cases the leaving hull comes back
   OPEN even if the hull it was docked to has a door shut. That is a fallback, not a
   restore: the pre-#657 build handed the leaving hull the SURVIVOR's orders, and the
   vessel's own figures were never recorded anywhere, so there is nothing to recover --
   and "as the constructor would have built it" beats "as the station happens to be". */
static void test_a_seam_with_no_records_falls_back_to_defaults() {
    printf("== a seam with no frozen orders falls back to the defaults ==\n");

    Ship A; A.v = new Vehicle; A.v->name = "A";
    Part *tankA = mkPart(A, "tankA", 1000.0, 1.0, 1.0, kTankHz, false);
    Part *portA = mkPart(A, "portA",   50.0, 1.0, 1.0, kPortHz, true);
    A.v->setRoot(tankA);
    A.v->attachDown(portA);
    A.v->controller = tankA;
    A.v->init();
    A.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    Ship B; B.v = new Vehicle; B.v->name = "B";
    Part *portB = mkPart(B, "portB",   50.0, 1.0, 1.0, kPortHz, true);
    Part *tankB = mkPart(B, "tankB", 1000.0, 1.0, 1.0, kTankHz, false);
    B.v->setRoot(portB);
    B.v->attachDown(tankB);
    B.v->controller = portB;
    B.v->init();
    B.v->placeShip(glm::dvec3(0.0, 0.0, -1.875), glm::dmat3(1.0));

    /* Only A has shut anything. */
    A.v->wheels_enabled = false;
    A.v->tvc_enabled = false;

    A.v->absorbShip(B.v, portA);
    CHECK_TRUE(A.v->seams.size() == 1 && A.v->seams[0].wheels_enabled
               && A.v->seams[0].tvc_enabled,
               "no records: the seam froze the donor's OPEN switches");

    Vehicle *out = A.v->extractSubtreeAsShip(portB, "B");
    CHECK_TRUE(out != nullptr, "no records: the split succeeded");
    if(out == nullptr) { destroyShip(B.v); destroyShip(A.v); return; }
    CHECK_TRUE(out->wheels_enabled && out->tvc_enabled,
               "no records: it comes back open, not on the station's shut doors");
    CHECK_TRUE(!A.v->wheels_enabled && !A.v->tvc_enabled, "and the station keeps them shut");

    destroyShip(out);
    destroyShip(B.v);
    destroyShip(A.v);
}

/* --- round-trip: absorb then extract restores both ships' geometry ------- */
static void test_roundtrip() {
    printf("== extractSubtreeAsShip: undock round-trip ==\n");

    Ship A; A.v = new Vehicle; A.v->name = "A";
    Part *tankA = mkPart(A, "tankA", 1000.0, 1.0, 1.0, kTankHz, false);
    Part *portA = mkPart(A, "portA",   50.0, 1.0, 1.0, kPortHz, true);
    A.v->setRoot(tankA);
    A.v->attachDown(portA);
    A.v->controller = tankA;
    A.v->init();
    A.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    Ship B; B.v = new Vehicle; B.v->name = "B";
    Part *portB = mkPart(B, "portB",   50.0, 1.0, 1.0, kPortHz, true);
    Part *tankB = mkPart(B, "tankB", 1000.0, 1.0, 1.0, kTankHz, false);
    B.v->setRoot(portB);
    B.v->attachDown(tankB);
    B.v->controller = portB;
    B.v->init();
    B.v->placeShip(glm::dvec3(0.0, 0.0, -1.875), glm::dmat3(1.0));

    const glm::dvec3 tankA0 = worldPos(A.v, tankA);
    const glm::dvec3 portA0 = worldPos(A.v, portA);
    const glm::dvec3 portB0 = worldPos(B.v, portB);
    const glm::dvec3 tankB0 = worldPos(B.v, tankB);

    A.v->absorbShip(B.v, portA);
    destroyShip(B.v);   // the shell is gone; its parts live on A now
    CHECK_TRUE(A.v->seams.size() == 1, "roundtrip: seam present after absorb");

    /* Undock: split the seam's subtree (B's old root) back into a ship. */
    Vehicle *out = A.v->extractSubtreeAsShip(A.v->seams[0].root, "B");
    CHECK_TRUE(out != nullptr, "roundtrip: extract returned a ship");
    if(out == nullptr) { destroyShip(A.v); return; }

    /* The survivor keeps its own two parts; the new ship has the other two. */
    CHECK_TRUE(A.v->parts.size() == 2, "roundtrip: survivor keeps two parts");
    CHECK_TRUE(out->parts.size() == 2, "roundtrip: extracted ship has two parts");
    CHECK_TRUE(out->rootPart() == portB, "roundtrip: extracted root is B's old root");

    /* Geometry restored: both ships' parts are back at their original world
       poses -- the split is the exact inverse of the merge. */
    CHECK_NEAR(glm::length(worldPos(A.v, tankA) - tankA0), 0.0, 1e-9,
               "roundtrip: tankA world pose restored");
    CHECK_NEAR(glm::length(worldPos(A.v, portA) - portA0), 0.0, 1e-9,
               "roundtrip: portA world pose restored");
    CHECK_NEAR(glm::length(worldPos(out, portB) - portB0), 0.0, 1e-9,
               "roundtrip: portB world pose restored");
    CHECK_NEAR(glm::length(worldPos(out, tankB) - tankB0), 0.0, 1e-9,
               "roundtrip: tankB world pose restored");

    destroyShip(out);
    destroyShip(A.v);
}

/* --- uid: identity that survives a merge which collides every id ---------- */

/* A two-part ship: tank + docking port, stacked face to face. `portIsRoot`
   picks which end is the root, so the caller can build the mated pair (one
   ship port-first, the other tank-first) that test_absorb docks.

   Part::id is set to the SAME strings for every ship built here -- which is
   exactly what two ships built from one def look like, since shipdef.cpp
   auto-generates "<catalog name>_<n>" per file and absorbShip never renames. */
static void mkTankPort(Ship &s, const char *shipName, double z, bool portIsRoot,
                       Part **tankOut, Part **portOut) {
    s.v = new Vehicle;
    s.v->name = shipName;
    Part *tank = mkPart(s, "fuel_tank", 1000.0, 1.0, 1.0, kTankHz, false);
    Part *port = mkPart(s, "docking_port", 50.0, 1.0, 1.0, kPortHz, true);
    tank->id = "fuel_tank_1";
    port->id = "docking_port";
    if(portIsRoot) {
        s.v->setRoot(port);
        s.v->attachDown(tank);
        s.v->controller = port;
    } else {
        s.v->setRoot(tank);
        s.v->attachDown(port);
        s.v->controller = tank;
    }
    s.v->init();
    s.v->placeShip(glm::dvec3(0.0, 0.0, z), glm::dmat3(1.0));
    *tankOut = tank;
    *portOut = port;
}

/* True when every part in `v` has a nonzero uid and no two share one. */
static bool uidsUnique(const Vehicle *v) {
    std::set<uint64_t> seen;
    for(const Part *p : v->parts) {
        if(p->uid == 0) { return false; }
        if(!seen.insert(p->uid).second) { return false; }
    }
    return true;
}

/* True when at least two parts in `v` share a Part::id -- the collision that
   makes `id` unusable as a key once ships merge. */
static bool idsCollide(const Vehicle *v) {
    std::set<std::string> seen;
    for(const Part *p : v->parts) {
        if(!seen.insert(p->id).second) { return true; }
    }
    return false;
}

static void test_uid() {
    printf("== uid: unique per instance, distinct across an id-colliding merge ==\n");

    /* Same geometry as test_absorb: A tank-first at the origin, B port-first
       mated onto A's port face (A's port rear face is at z = -1.75, B's port
       front face is its origin + 0.125, so B sits at z = -1.875). */
    Ship A, B;
    Part *tankA = nullptr, *portA = nullptr, *tankB = nullptr, *portB = nullptr;
    mkTankPort(A, "A", 0.0,     false, &tankA, &portA);
    mkTankPort(B, "B", -1.875,  true,  &tankB, &portB);

    /* Minted, nonzero, and distinct before anything merges -- the two ships
       are independent builds, so a per-ship counter would pass the first and
       fail this one. */
    CHECK_TRUE(uidsUnique(A.v), "uid: ship A's parts are all distinct and nonzero");
    CHECK_TRUE(uidsUnique(B.v), "uid: ship B's parts are all distinct and nonzero");
    {
        std::set<uint64_t> both;
        for(const Part *p : A.v->parts) { both.insert(p->uid); }
        for(const Part *p : B.v->parts) { both.insert(p->uid); }
        CHECK_TRUE(both.size() == A.v->parts.size() + B.v->parts.size(),
                   "uid: distinct ACROSS two independently built ships");
    }

    /* Both ships were authored with the same ids, so the collision is present
       before the merge too -- assert it, or the check below proves nothing. */
    CHECK_TRUE(portA->id == portB->id, "uid: the two ships' port ids collide by construction");
    CHECK_TRUE(tankA->id == tankB->id, "uid: the two ships' tank ids collide by construction");

    A.v->absorbShip(B.v, portA);

    /* The property save/load resolution depends on: after the merge the
       string ids are ambiguous but the uids are not. */
    CHECK_TRUE(A.v->parts.size() == 4, "uid: merged ship holds all four parts");
    CHECK_TRUE(idsCollide(A.v), "uid: merged ship's string ids DO collide (the bug uid avoids)");
    CHECK_TRUE(uidsUnique(A.v), "uid: merged ship's uids are still all distinct");

    /* A split must not remint or reuse: the parts keep the uids they had. */
    const uint64_t portBuid = portB->uid;
    const uint64_t tankBuid = tankB->uid;
    Vehicle *out = A.v->extractSubtreeAsShip(portB, "B");
    CHECK_TRUE(out != nullptr, "uid: split succeeded");
    if(out != nullptr) {
        CHECK_TRUE(portB->uid == portBuid, "uid: split does not remint the root's uid");
        CHECK_TRUE(tankB->uid == tankBuid, "uid: split does not remint a child's uid");
        CHECK_TRUE(uidsUnique(out), "uid: split-off ship's uids are distinct and nonzero");
        CHECK_TRUE(uidsUnique(A.v), "uid: survivor's uids are distinct and nonzero");
        std::set<uint64_t> both;
        for(const Part *p : A.v->parts) { both.insert(p->uid); }
        for(const Part *p : out->parts) { both.insert(p->uid); }
        CHECK_TRUE(both.size() == 4, "uid: survivor and split-off share no uid");
        destroyShip(out);
    }

    destroyShip(B.v);   /* the empty shell absorbShip left behind */
    destroyShip(A.v);
}

/* --- seam maintenance across a stage (the 1.7b use-after-free) ----------- */
/* A: capsule (root) -> decoupler -> docking port P. Dock B onto P, then
   stage the decoupler: everything below it (the decoupler, P, and all of B)
   comes off as a separate ship. The seam {P, B.root} has BOTH ends in that
   split-off subtree, so it must move with it -- leaving the survivor with no
   seam. Before the fix the survivor kept {P, B.root}, both now owned by the
   split-off ship; deleting that ship and then saving the survivor read a
   freed Part (the use-after-free in the report, section 1.7b).

   Two branches now hang off that chain -- sideX off the part that becomes the cut
   root, sideY/sideYkid/sideY2 off the capsule -- each built after everything else.
   They are load-bearing, not clutter: a chain has only one order worth naming, so
   without a branch the #544 order pins below cannot tell "filter the vector" from
   "rebuild it in some other order". */
static void test_seam_staging() {
    printf("== seam maintenance across a stage (1.7b use-after-free) ==\n");

    Ship A; A.v = new Vehicle; A.v->name = "A";
    Part *capsule = mkPart(A, "capsule",   400.0, 1.0, 1.0, kTankHz, false);
    Part *decop   = mkPart(A, "decoupler",  50.0, 1.0, 1.0, kPortHz, false);
    Part *portA   = mkPart(A, "portA",      50.0, 1.0, 1.0, kPortHz, true);
    A.v->setRoot(capsule);
    A.v->attachDown(decop);
    A.v->attachDown(portA);
    A.v->controller = capsule;
    A.v->init();
    A.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    /* #544: index into `parts` is the tie-break that decides which split ship keeps the
       bare name, and it only means "build order" while `parts` keeps that order. Pin the
       three ways the vector changes -- append here, append-across-ships at the absorb,
       filter on both sides of the split. */
    CHECK_TRUE(A.v->parts.size() == 3 && A.v->parts[0] == capsule &&
               A.v->parts[1] == decop && A.v->parts[2] == portA,
               "staging: setRoot and attachDown append, so parts is build order");

    Ship B; B.v = new Vehicle; B.v->name = "B";
    Part *portB = mkPart(B, "portB",   50.0, 1.0, 1.0, kPortHz, true);
    Part *tankB = mkPart(B, "tankB", 1000.0, 1.0, 1.0, kTankHz, false);
    B.v->setRoot(portB);
    B.v->attachDown(tankB);
    B.v->controller = portB;
    B.v->init();
    B.v->placeShip(glm::dvec3(0.0, 0.0, -2.5), glm::dmat3(1.0));

    A.v->absorbShip(B.v, portA);
    /* The path #544 worries about most: it stitches two ships' build orders into one
       vector. Append (not insert, not sort) is what keeps both halves ordered. */
    CHECK_TRUE(A.v->parts.size() == 5 && A.v->parts[0] == capsule &&
               A.v->parts[1] == decop && A.v->parts[2] == portA &&
               A.v->parts[3] == portB && A.v->parts[4] == tankB,
               "staging: absorb keeps the host's order, then appends the donor's");

    /* Hang a branch off the part that will be the cut root, and build it LAST. Every
       other order agrees with build order in a chain -- a walk order would put this part
       third, tree order puts it deep, build order puts it last -- so without it the pins
       below could not tell "filter the vector" from "rebuild it in walk order". */
    Part *sideX = mkPart(A, "sideX",   50.0, 1.0, 1.0, kTankHz, false);
    A.v->attachMode(sideX, partIndex(A.v, decop), AttachMode::Down);   // parent = decop

    /* The same trick on the OTHER side of the cut, because a split filters `parts`
       twice and a one-part survivor has no order to lose. Build order here is
       capsule, sideY, sideYkid, sideY2, while walking the survivor's own tree gives
       capsule, sideY, sideY2, sideYkid -- and they differ only because sideYkid is a
       grandchild whose sibling was built last, which is why two parts would not do.

       A rebuild through an address-ordered list (the idiom droppedPartsAtStage uses
       above) is NOT catchable from here: measured, the allocator hands these four parts
       out in build order, so sorting them by address is not a permutation and no order
       pin can see it. That rebuild is still worth worrying about on a real ship, whose
       addresses do arrive scrambled -- it can put a child before its parent, and the
       save format resolves parts by parent-before-child (save.cpp:899) -- but forcing
       that through a fixture's allocation order is allocator luck, not a test. */
    Part *sideY    = mkPart(A, "sideY",    50.0, 1.0, 1.0, kTankHz, false);
    Part *sideYkid = mkPart(A, "sideYkid", 50.0, 1.0, 1.0, kTankHz, false);
    Part *sideY2   = mkPart(A, "sideY2",   50.0, 1.0, 1.0, kTankHz, false);
    A.v->attachMode(sideY, partIndex(A.v, capsule), AttachMode::Down);
    A.v->attachMode(sideYkid, partIndex(A.v, sideY), AttachMode::Down);
    A.v->attachMode(sideY2, partIndex(A.v, capsule), AttachMode::Down);   // LAST built
    CHECK_TRUE(A.v->seams.size() == 1, "staging: one seam after the dock");
    CHECK_TRUE(A.v->seams[0].port == portA, "staging: seam port is A's port");
    CHECK_TRUE(A.v->seams[0].root == portB, "staging: seam root is B's root");

    /* Stage the decoupler: the decoupler, the port, and all of B come off. */
    /* The pilot's ORDERS ride along with the hull they were given for (#649). An enable
       switch is not a stage's property, so a hull that had its drums or plates off stages
       with them still off; left out, the new Vehicle's constructor defaults silently
       re-open a door the player shut. Nothing could write either mask until #649 put a
       handle on them, which is how this stayed invisible since #425 D-2. */
    A.v->wheels_enabled = false;
    A.v->surfaces_enabled = false;
    A.v->tvc_enabled = false;   // #768's door rides the same way
    Vehicle *out = A.v->extractSubtreeAsShip(decop, "staged");
    CHECK_TRUE(out != nullptr, "staging: the split succeeded");
    if(out == nullptr) { destroyShip(B.v); destroyShip(A.v); return; }
    CHECK_TRUE(!out->wheels_enabled && !out->surfaces_enabled && !out->tvc_enabled,
               "staging: the split-off ship keeps every switch as the pilot left them");
    CHECK_TRUE(!A.v->wheels_enabled && !A.v->surfaces_enabled && !A.v->tvc_enabled,
               "and so does the survivor: the cut splits parts, not orders");

    /* The fix: both ends of the seam are in the split-off subtree, so the
       seam moved WITH it and the survivor is left with none. */
    CHECK_TRUE(A.v->seams.empty(), "staging: survivor holds no seam (both ends left with the split)");
    CHECK_TRUE(out->seams.size() == 1, "staging: the seam followed the split-off ship");
    if(out->seams.size() == 1) {
        CHECK_TRUE(out->seams[0].port == portA, "staging: split-off seam's port");
        CHECK_TRUE(out->seams[0].root == portB, "staging: split-off seam's root");
        /* The seam moved with the ship, and it moved with B's record -- open doors,
           though the hull carrying it is flown on A's shut ones. This is what lets that
           hull undock B later and answer for B (#657). */
        CHECK_TRUE(out->seams[0].wheels_enabled && out->seams[0].tvc_enabled,
                   "staging: a seam that travels keeps what it froze off the donor");
    }

    /* Both sides of a split come from filtering the SAME vector (vehicle.cpp builds the
       new ship's list by walking `parts`, and the survivor's by the complement), so build
       order survives the cut on both sides -- the split-off ship's own ties depend on it
       the next time it stages. */
    CHECK_TRUE(out->parts.size() == 5 && out->parts[0] == decop &&
               out->parts[1] == portA && out->parts[2] == portB &&
               out->parts[3] == tankB && out->parts[4] == sideX,
               "staging: the split-off ship keeps build order, walk order would put "
               "sideX third");

    /* The other half of the same filter: vehicle.cpp rebuilds the survivor by walking
       `parts` and skipping the cut, exactly as it builds the new ship's list above, so
       the cut keeps build order on BOTH sides. The pin above only reaches one side --
       the survivor here used to be a single part, which no order can get wrong. */
    CHECK_TRUE(A.v->parts.size() == 4 && A.v->parts[0] == capsule &&
               A.v->parts[1] == sideY && A.v->parts[2] == sideYkid &&
               A.v->parts[3] == sideY2,
               "staging: the survivor keeps build order too -- walk order puts sideY2 third");

    /* Delete the split-off ship (the report's step 4): this frees portA and
       portB. Any seam the survivor still held would now dangle at them. */
    destroyShip(out);

    /* With the fix the survivor's seam list is empty, so this dereference
       touches nothing; without it, this is the use-after-free (step 5 of the
       report) -- ASan reports it here. */
    for(const Vehicle::DockSeam &s : A.v->seams) {
        (void)s.port->uid;
        (void)s.root->uid;
    }
    CHECK_TRUE(A.v->seams.empty(), "staging: no dangling seam after the split-off is deleted");

    destroyShip(B.v);   /* the empty shell absorbShip left behind */
    destroyShip(A.v);
}

/* --- seam maintenance across an absorb of an already-docked ship --------- */
/* B docks C (so B->seams = {B.port, C.root}). Then A absorbs B. Before the
   fix the joint B recorded was orphaned when B was deleted as a shell (report
   section 1.7c). Now it moves into A, ahead of the new {A.port, B.root} seam,
   so the undock order (pop the last) peels B first; undocking B then takes C
   with it and leaves the B-C seam on the split-off ship. */
static void test_seam_nested_dock() {
    printf("== seam maintenance across an absorb of a docked ship ==\n");

    /* C: port (root) + tank. */
    Ship C; C.v = new Vehicle; C.v->name = "C";
    Part *portC = mkPart(C, "portC",   50.0, 1.0, 1.0, kPortHz, true);
    Part *tankC = mkPart(C, "tankC", 1000.0, 1.0, 1.0, kTankHz, false);
    C.v->setRoot(portC);
    C.v->attachDown(tankC);
    C.v->controller = portC;
    C.v->init();
    C.v->placeShip(glm::dvec3(0.0, 0.0, 2.0), glm::dmat3(1.0));

    /* B: tank (root) + port, docks C. B is the survivor, so B->seams =
       {B.port, C.root, "C"}. */
    Ship B; B.v = new Vehicle; B.v->name = "B";
    Part *tankB = mkPart(B, "tankB", 1000.0, 1.0, 1.0, kTankHz, false);
    Part *portB = mkPart(B, "portB",   50.0, 1.0, 1.0, kPortHz, true);
    B.v->setRoot(tankB);
    B.v->attachDown(portB);
    B.v->controller = tankB;
    B.v->init();
    B.v->placeShip(glm::dvec3(0.0, 0.0, 1.0), glm::dmat3(1.0));

    /* Every vessel gets a DIFFERENT pilot's mark, so each hand-off below can be told
       apart (#657): C shut its wheels, B its plates, A its nozzles, and all three dials
       differ. */
    C.v->wheels_enabled = false;
    C.v->sas_turn_rate = 0.25;
    B.v->absorbShip(C.v, portB);
    CHECK_TRUE(B.v->seams.size() == 1, "nested: B holds the C seam after the dock");
    CHECK_TRUE(B.v->seams[0].port == portB, "nested: B-C seam port is B's port");
    CHECK_TRUE(B.v->seams[0].root == portC, "nested: B-C seam root is C's root");
    CHECK_TRUE(!B.v->seams[0].wheels_enabled, "nested: the B-C seam froze C's shut wheels");
    CHECK_NEAR(B.v->seams[0].sas_turn_rate, 0.25, 1e-9, "nested: and C's ceiling");

    /* A: tank (root) + port, absorbs B (which already holds C). */
    Ship A; A.v = new Vehicle; A.v->name = "A";
    Part *tankA = mkPart(A, "tankA", 1000.0, 1.0, 1.0, kTankHz, false);
    Part *portA = mkPart(A, "portA",   50.0, 1.0, 1.0, kPortHz, true);
    A.v->setRoot(tankA);
    A.v->attachDown(portA);
    A.v->controller = tankA;
    A.v->init();
    A.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    B.v->surfaces_enabled = false;
    B.v->sas_turn_rate = 0.5;
    A.v->tvc_enabled = false;
    A.v->sas_turn_rate = 1.5;
    A.v->absorbShip(B.v, portA);

    /* All six parts in A now. */
    CHECK_TRUE(A.v->parts.size() == 6, "nested: A holds all six parts after the merge");

    /* Both seams survive the merge, with the outer (A-B) last so an undock
       peels B first. */
    CHECK_TRUE(A.v->seams.size() == 2, "nested: both seams moved into A");
    if(A.v->seams.size() == 2) {
        CHECK_TRUE(A.v->seams[1].port == portA && A.v->seams[1].root == tankB,
                   "nested: the A-B seam is the outer one (last)");
        CHECK_TRUE(A.v->seams[0].port == portB && A.v->seams[0].root == portC,
                   "nested: the B-C seam was moved in, not orphaned");
        /* Two seams, two different donors, so a merge that overwrote the moved seam with
           the survivor's figures -- or with the donor it is about to absorb -- shows up. */
        CHECK_TRUE(!A.v->seams[0].wheels_enabled && A.v->seams[0].surfaces_enabled
                   && A.v->seams[0].tvc_enabled,
                   "nested: the moved B-C seam still holds C's orders, not B's or A's");
        CHECK_NEAR(A.v->seams[0].sas_turn_rate, 0.25, 1e-9, "nested: and C's ceiling");
        CHECK_TRUE(!A.v->seams[1].surfaces_enabled && A.v->seams[1].wheels_enabled
                   && A.v->seams[1].tvc_enabled,
                   "nested: the new A-B seam holds B's orders, not C's or A's");
        CHECK_NEAR(A.v->seams[1].sas_turn_rate, 0.5, 1e-9, "nested: and B's ceiling");
    }

    /* Undock B: B and its docked C come off together; the B-C seam follows,
       and the A-B seam (split across the cut) is dropped. */
    Vehicle *out = A.v->extractSubtreeAsShip(tankB, "B");
    CHECK_TRUE(out != nullptr, "nested: undock B succeeded");
    if(out != nullptr) {
        CHECK_TRUE(A.v->seams.empty(), "nested: A holds no seam after undocking B");
        CHECK_TRUE(out->seams.size() == 1, "nested: the B-C seam followed B");
        if(out->seams.size() == 1) {
            CHECK_TRUE(out->seams[0].port == portB && out->seams[0].root == portC,
                       "nested: B can still undock C");
            CHECK_TRUE(!out->seams[0].wheels_enabled && out->seams[0].surfaces_enabled
                       && out->seams[0].tvc_enabled,
                       "nested: and it carries C's record through the merge and the split");
            CHECK_NEAR(out->seams[0].sas_turn_rate, 0.25, 1e-9, "nested: C's ceiling included");
        }
        /* B leaves on B's own orders (its shut plates, its 0.5 ceiling), not A's shut
           nozzles, and A is left with A's. C's turn comes when B undocks it, using the
           record that followed B here. */
        CHECK_TRUE(!out->surfaces_enabled && out->wheels_enabled && out->tvc_enabled,
                   "nested: B comes back on B's orders");
        CHECK_NEAR(out->sas_turn_rate, 0.5, 1e-9, "nested: capped as B was capped");
        CHECK_TRUE(!A.v->tvc_enabled && A.v->wheels_enabled && A.v->surfaces_enabled,
                   "nested: the survivor is untouched by either donor");
        CHECK_NEAR(A.v->sas_turn_rate, 1.5, 1e-9, "nested: including A's own ceiling");
        CHECK_TRUE(A.v->parts.size() == 2, "nested: A keeps its two parts");
        CHECK_TRUE(out->parts.size() == 4, "nested: B+C come off as four parts");
        /* And the inner dock still answers for C, one step later: undock C from the ship
           B came back as. Two nested docks, three hulls, each on its own pilot's
           switches at the end. */
        Vehicle *outC = out->extractSubtreeAsShip(portC, "C");
        CHECK_TRUE(outC != nullptr, "nested: undocking C from B still works");
        if(outC != nullptr) {
            CHECK_TRUE(!outC->wheels_enabled && outC->surfaces_enabled && outC->tvc_enabled,
                       "nested: C comes back on C's own shut wheels");
            CHECK_NEAR(outC->sas_turn_rate, 0.25, 1e-9, "nested: and C's ceiling");
            CHECK_TRUE(!out->surfaces_enabled && out->wheels_enabled && out->tvc_enabled,
                       "nested: B still flies B's orders after handing C back");
            CHECK_NEAR(out->sas_turn_rate, 0.5, 1e-9, "nested: and B's ceiling");
            CHECK_TRUE(out->seams.empty() && outC->seams.empty(),
                       "nested: the inner cut seam is on neither side");
            destroyShip(outC);
        }
        destroyShip(out);
    }

    destroyShip(C.v);   /* empty shells left by the two absorbs */
    destroyShip(B.v);
    destroyShip(A.v);
}

/* --- seam KEPT on the survivor when a stage drops only its far end -------- */
/* A: capsule -> port P. B: bRoot -> decoupler -> tank, docked onto P. Staging
   the decoupler drops only {decoupler, tank}; the seam {P, bRoot} has BOTH
   ends in the survivor, so it must be KEPT on A (the joint is intact -- bRoot
   is still mated to P). This is the one seam branch (both ends kept) the other
   cases never exercise with a non-empty result: a regression that drops kept
   seams would strand the joint -- A toasts "Nothing docked to undock" forever
   while bRoot still hangs off P -- and nothing in the suite would catch it.
   The follow-up undock then drops the seam (now split across the cut). */
static void test_seam_kept_across_stage() {
    printf("== seam kept on the survivor when a stage drops only its far end ==\n");

    Ship A; A.v = new Vehicle; A.v->name = "A";
    Part *capsule = mkPart(A, "capsule",   400.0, 1.0, 1.0, kTankHz, false);
    Part *portA   = mkPart(A, "portA",      50.0, 1.0, 1.0, kPortHz, true);
    A.v->setRoot(capsule);
    A.v->attachDown(portA);
    A.v->controller = capsule;
    A.v->init();
    A.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    Ship B; B.v = new Vehicle; B.v->name = "B";
    Part *bRoot   = mkPart(B, "bRoot",      50.0, 1.0, 1.0, kPortHz, true);
    Part *decop   = mkPart(B, "decoupler",  50.0, 1.0, 1.0, kPortHz, false);
    Part *tankB   = mkPart(B, "tankB",   1000.0, 1.0, 1.0, kTankHz, false);
    B.v->setRoot(bRoot);
    B.v->attachDown(decop);
    B.v->attachDown(tankB);
    B.v->controller = bRoot;
    B.v->init();
    B.v->placeShip(glm::dvec3(0.0, 0.0, -2.0), glm::dmat3(1.0));

    A.v->absorbShip(B.v, portA);
    /* Shut a door on the FLYING hull after the dock (#657's other half): the seam must
       keep what it froze off B -- B's doors are all open -- so the checks below can tell
       a record from a leak. */
    A.v->wheels_enabled = false;
    A.v->sas_turn_rate = 2.0;
    A.v->rcs_thrust_frac = 0.2;    // #1067: the same inheritance, dialed
    A.v->rcs_attitude_frac = 0.6;  // #1124: and its second dial, a different figure
    CHECK_TRUE(A.v->seams.size() == 1, "kept: one seam after the dock");
    CHECK_TRUE(A.v->seams[0].port == portA && A.v->seams[0].root == bRoot,
               "kept: seam is {P, bRoot}");

    /* Stage the decoupler: drops only {decoupler, tank}. bRoot (and the joint
       {P, bRoot}) stays with A. */
    Vehicle *out = A.v->extractSubtreeAsShip(decop, "staged");
    CHECK_TRUE(out != nullptr, "kept: the split succeeded");
    if(out == nullptr) { destroyShip(B.v); destroyShip(A.v); return; }

    /* THE keep branch: both seam ends are in the survivor, so the seam stays. */
    CHECK_TRUE(A.v->seams.size() == 1, "kept: the seam is KEPT on the survivor");
    if(A.v->seams.size() == 1) {
        CHECK_TRUE(A.v->seams[0].port == portA && A.v->seams[0].root == bRoot,
                   "kept: the kept seam is still {P, bRoot}");
        CHECK_TRUE(A.v->seams[0].wheels_enabled,
                   "kept: a kept seam holds the donor's open door, not ours");
    }
    CHECK_TRUE(out->seams.empty(), "kept: the split-off ship holds no seam");
    /* This cut is at a decoupler INSIDE the docked vessel, not at the seam's root end,
       so #649's rule still decides it: the hull leaving is flown on this hull's switches,
       shut ones included. */
    CHECK_TRUE(!out->wheels_enabled, "kept: a stage cut away from the seam root takes our orders");
    CHECK_NEAR(out->sas_turn_rate, 2.0, 1e-9, "kept: and our ceiling");
    CHECK_NEAR(out->rcs_thrust_frac, 0.2, 1e-9, "kept: and our RCS authority");
    CHECK_NEAR(out->rcs_attitude_frac, 0.6, 1e-9,
               "kept: and our pointing dial, which is a separate figure on the same rail");
    CHECK_TRUE(A.v->parts.size() == 3, "kept: survivor keeps capsule, port, bRoot");
    CHECK_TRUE(out->parts.size() == 2, "kept: split-off holds decoupler + tank");

    destroyShip(out);

    /* The joint is still intact, so undocking bRoot still works: the seam is
       now split across the cut (P stays, bRoot goes) and is dropped. */
    Vehicle *out2 = A.v->extractSubtreeAsShip(bRoot, "B");
    CHECK_TRUE(out2 != nullptr, "kept: undock after the stage still works");
    if(out2 != nullptr) {
        CHECK_TRUE(A.v->seams.empty(), "kept: the undock dropped the seam");
        CHECK_TRUE(out2->wheels_enabled, "kept: this cut IS the seam root, so bRoot leaves open");
        CHECK_NEAR(out2->sas_turn_rate, 0.0, 1e-9, "kept: uncapped, as B ever was");
        CHECK_TRUE(!A.v->wheels_enabled, "kept: and the survivor keeps its own shut door");
        CHECK_TRUE(A.v->parts.size() == 2, "kept: survivor keeps capsule + port");
        CHECK_TRUE(out2->parts.size() == 1, "kept: bRoot comes off alone (the stage already took the rest)");
        destroyShip(out2);
    }

    destroyShip(B.v);   /* the empty shell absorbShip left behind */
    destroyShip(A.v);
}

/* --- experiments ride the Part across dock/undock ------------------------- */
/* Part::experiments is payload on the Part instance. absorbShip /
   extractSubtreeAsShip only re-point owner/parent/pose -- they must never
   rebuild or copy Parts -- so science data follows the part. A seated kerbal's
   suit rides along for the same reason: it hangs off the capsule Part (part.h),
   so there is no crew move to get wrong. */
static void test_experiments_ride_parts() {
    printf("== experiments ride Parts across absorb + extract ==\n");

    Experiment eA;
    eA.type = "observation";
    eA.body = "Mun";
    eA.situation = SciSituation::LowOrbit;
    eA.biome = "midlands";
    Experiment eB = eA;
    eB.situation = SciSituation::HighOrbit;

    Ship A; A.v = new Vehicle; A.v->name = "A";
    Part *tankA = mkPart(A, "tankA", 1000.0, 1.0, 1.0, kTankHz, false);
    Part *portA = mkPart(A, "portA",   50.0, 1.0, 1.0, kPortHz, true);
    tankA->experiments.push_back(eA);
    A.v->setRoot(tankA);
    A.v->attachDown(portA);
    A.v->controller = tankA;
    A.v->init();
    A.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    Ship B; B.v = new Vehicle; B.v->name = "B";
    Part *portB = mkPart(B, "portB",   50.0, 1.0, 1.0, kPortHz, true);
    Part *tankB = mkPart(B, "tankB", 1000.0, 1.0, 1.0, kTankHz, false);
    tankB->experiments.push_back(eB);
    B.v->setRoot(portB);
    B.v->attachDown(tankB);
    B.v->controller = portB;
    B.v->init();
    B.v->placeShip(glm::dvec3(0.0, 0.0, -1.875), glm::dmat3(1.0));

    A.v->absorbShip(B.v, portA);
    CHECK_TRUE(tankA->experiments.size() == 1 && tankA->experiments[0] == eA,
               "dock: survivor part keeps its experiment");
    CHECK_TRUE(tankB->experiments.size() == 1 && tankB->experiments[0] == eB,
               "dock: absorbed part keeps its experiment");
    CHECK_TRUE(tankB->owner == A.v, "dock: absorbed part is owned by the survivor");

    Vehicle *out = A.v->extractSubtreeAsShip(A.v->seams[0].root, "B");
    CHECK_TRUE(out != nullptr, "undock: extract returned a ship");
    if(out != nullptr) {
        CHECK_TRUE(tankA->experiments.size() == 1 && tankA->experiments[0] == eA,
                   "undock: survivor part keeps its experiment");
        CHECK_TRUE(tankB->experiments.size() == 1 && tankB->experiments[0] == eB,
                   "undock: split-off part keeps its experiment");
        CHECK_TRUE(tankB->owner == out, "undock: split-off part is owned by the new ship");
        destroyShip(out);
    }

    destroyShip(B.v);   /* empty shell */
    destroyShip(A.v);
}

/* --- the dock's own mass door (#583) --------------------------------------
   Both loaders refuse a ship whose parts add past a double (#547), and both judge
   ONE ship. A dock is the one place in play where two of those totals become ONE
   Body, and an infinite compound mass is #547's symptom, not a refusal: Bullet's
   principal-axis solve answers it with a NaN inertia and a NaN centre, and the
   merged hull cannot be turned. So the gate asks before absorbShip does any part
   surgery. This is a RANGE check -- the pair that still fits must merge, or the
   door would be an invented size limit. */
static void test_dock_mass_door() {
    printf("== refuseDockWith: the merged total, not each half ==\n");

    Ship A; A.v = new Vehicle; A.v->name = "A";
    Part *portA = mkPart(A, "portA", 1.7e308, 1.0, 1.0, kPortHz, true);
    A.v->setRoot(portA);
    A.v->controller = portA;
    A.v->init();
    A.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    Ship B; B.v = new Vehicle; B.v->name = "B";
    Part *portB = mkPart(B, "portB", 1.7e308, 1.0, 1.0, kPortHz, true);
    B.v->setRoot(portB);
    B.v->controller = portB;
    B.v->init();
    B.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz)), glm::dmat3(1.0));

    /* Each half is a hull the game can store: 1.7e308 kg is finite, so init()
       above -- which runs rebuildCompound, and its #547 assert -- got there
       honestly. */
    CHECK_TRUE(std::isfinite(A.v->getMass()) && std::isfinite(B.v->getMass()),
               "mass door: each hull alone fits");
    CHECK_TRUE(!std::isfinite(A.v->getMass() + B.v->getMass()),
               "mass door: the pair does not, and that is the state the door is for");

    const std::string why = A.v->refuseDockWith(B.v);
    CHECK_TRUE(!why.empty(), "mass door: the dock is refused, not merged");
    CHECK_TRUE(why == "cannot dock with \"B\": the two hulls together weigh more"
                      " than the game can store",
               "mass door: the sentence names the other hull and the reason");
    CHECK_TRUE(A.v->parts.size() == 1 && B.v->parts.size() == 1,
               "mass door: refused before any part surgery, both ships intact");
    CHECK_TRUE(A.v->seams.empty(), "mass door: refused, so no seam recorded");
    CHECK_NEAR(glm::length(portB->localPos), 0.0, 1e-12,
               "mass door: B's root was never re-based into A's frame");

    /* And the pair that DOES fit: same hulls at half the mass, so the sum is
       8e307 kg -- finite, therefore a hull the game can hold and a dock it must
       allow. Half is enough headroom to be a real test rather than a rounding
       coincidence; the shipped fleet is 300 orders below either bound. */
    Ship C; C.v = new Vehicle; C.v->name = "C";
    Part *portC = mkPart(C, "portC", 4e307, 1.0, 1.0, kPortHz, true);
    C.v->setRoot(portC);
    C.v->controller = portC;
    C.v->init();
    C.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    Ship D; D.v = new Vehicle; D.v->name = "D";
    Part *portD = mkPart(D, "portD", 4e307, 1.0, 1.0, kPortHz, true);
    D.v->setRoot(portD);
    D.v->controller = portD;
    D.v->init();
    D.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz)), glm::dmat3(1.0));

    CHECK_TRUE(C.v->refuseDockWith(D.v).empty(),
               "mass door: a pair whose sum fits is not refused");
    C.v->absorbShip(D.v, portC);
    CHECK_TRUE(C.v->parts.size() == 2 && D.v->parts.empty(),
               "mass door: and it really merged");
    CHECK_TRUE(C.v->seams.size() == 1, "mass door: the seam is recorded as usual");
    CHECK_NEAR(C.v->getMass(), 8e307, 1e300, "mass door: the merged total is the sum");

    destroyShip(D.v);   /* the empty shell absorbShip left behind */
    destroyShip(C.v);
    destroyShip(B.v);
    destroyShip(A.v);
}

/* --- the docking panel's readout and verdict (#1026) ------------------------ */
/* Vehicle::dockRead and Vehicle::dockSituation are now the ONE pass that decides a dock
   (Game::updateDocking switched on them) and the one the Docking window prints, so the
   numbers a pilot reads and the reason a mate silently failed are both pinned here.
   The fixtures are two single-port hulls: A's port at the world origin facing +Z, B's
   port facing it, faces touching at z = -kPortHz. One part per hull keeps a part's pose
   equal to the hull's, so each row below can be worked out by hand. */
/* #1152. THE CAPTURE DISTANCE IS THE PORTS' OWN SIZE. The gate used to be a global 1.5 m,
   which on a 0.25 m-deep port is six port-depths of daylight: absorbShip welds the pair at
   whatever gap the capture tick measured, so every "docked" hull in the game carried up to
   1.5 m of air between the ports that had just joined (the ticket's screenshot). The law is
   now 0.4 of the SHALLOWER port's depth, floored at 1.25x the pair's collision standoff.
   Pinned by value for the three shipped port sizes: the situation test above asks the law
   for its boundary rather than a literal, so it cannot see the law itself move. These rows
   assume the shipped hull margin, which is what the last one checks. */
static void test_the_capture_distance_is_the_ports_own_size() {
    printf("== dockCaptureRange: the ports set the distance, not a global ==\n");
    Ship s;                              // just owns the PartDefs these Parts point at
    Part *p1   = mkPart(s, "r1",   50.0, 1.0,  1.0,  0.125,   true);   // 0.25 m deep
    Part *p15  = mkPart(s, "r15",  50.0, 1.5,  1.5,  0.1875,  true);   // 0.375 m
    Part *p225 = mkPart(s, "r225", 50.0, 2.25, 2.25, 0.28125, true);   // 0.5625 m
    CHECK_NEAR(effectiveHullMargin(p1->body), 0.0, 1e-12,
               "at the shipped default a part carries no collision margin of its own");
    CHECK_NEAR(Vehicle::dockCaptureRange(p1, p1), 0.10, 1e-12,
               "the shipped r1 pair captures at 10 cm of face gap");
    CHECK_NEAR(Vehicle::dockCaptureRange(p15, p15), 0.15, 1e-12,
               "a 50% deeper port opens 50% wider");
    CHECK_NEAR(Vehicle::dockCaptureRange(p225, p225), 0.225, 1e-12,
               "and the biggest port in the game still captures inside a quarter metre");
    CHECK_NEAR(Vehicle::dockCaptureRange(p1, p225), 0.10, 1e-12,
               "a big port cannot out-reach a small one: the shallower governs");
    CHECK_NEAR(Vehicle::dockCaptureRange(p225, p1), 0.10, 1e-12,
               "whichever of the two is the armed one");

    /* The floor, and the reason it exists. A pair whose colliders stop them 0.2 m apart can
       never be flown inside a 0.10 m gate: the approach ends TooWide with the pilot holding
       every key down, which is the unreachable-gate bug this function would otherwise
       reintroduce the moment a part carries a margin. */
    p1->body->hull_margin = 0.1;
    p225->body->hull_margin = 0.1;
    CHECK_NEAR(Vehicle::dockCaptureRange(p1, p225), 0.25, 1e-12,
               "0.1 m of margin on each: the floor lifts that pair's gate to 0.25 m");
    CHECK_TRUE(Vehicle::dockCaptureRange(p1, p225)
                   > effectiveHullMargin(p1->body) + effectiveHullMargin(p225->body),
               "and a gated pair always stops short of contact, never short of the gate");
    p1->body->hull_margin = -1.0;
    p225->body->hull_margin = -1.0;

    /* A port with no def has no size to scale from, so the answer is 0 -- a capture
       distance of nothing is a gate that never opens, which is the direction a broken
       part should fail in. It is also unreachable from the gate: dockRead() reports
       unmeasurable for a port it cannot read, and dockSituation() answers before the
       distance gates are ever asked. */
    Part bare;
    CHECK_NEAR(Vehicle::dockCaptureRange(p1, &bare), 0.0, 1e-12,
               "no def, no capture distance");

    /* The other two gates' figures, by value, for the same reason as the rows above: the
       situation test asks the law for its boundary, so nothing there notices a retune.
       Both are numbers a pilot's hands are tuned to -- 5 deg of axis, 15 cm/s of closure
       -- and moving either also moves the e2e pins that fly them: flight/47 mates at the
       scripted 0.09 m/s, flight/1026-a refuses at 40 deg outside this cone. */
    CHECK_NEAR(Vehicle::kDockAlign, 0.996195, 1e-6, "the axis gate is a 5 deg cone");
    const double cone = std::acos(Vehicle::kDockAlign) * 180.0 / std::numbers::pi;
    CHECK(cone > 4.99 && cone < 5.01);
    CHECK_NEAR(Vehicle::kDockMaxV, 0.15, 1e-12, "and capture needs 15 cm/s or less");
}

static void test_the_dock_situation_says_why_a_pair_will_not_mate() {
    printf("== dockRead/dockSituation: the gates, and what stopped this dock ==\n");

    Ship A; A.v = new Vehicle; A.v->name = "A";
    Part *portA = mkPart(A, "portA", 50.0, 1.0, 1.0, kPortHz, true);
    A.v->setRoot(portA);
    A.v->controller = portA;
    A.v->init();
    A.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    Ship B; B.v = new Vehicle; B.v->name = "B";
    Part *portB = mkPart(B, "portB", 50.0, 1.0, 1.0, kPortHz, true);
    B.v->setRoot(portB);
    B.v->controller = portB;
    B.v->init();
    B.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz)), glm::dmat3(1.0));

    A.v->dockArmPort = portA;
    A.v->dockTargetShip = B.v;
    A.v->dockTargetPort = portB;
    /* The capture distance THIS pair gets (#1152: it is the ports' own size, not a
       global constant). Asked from the law so the boundary rows below stay boundaries if
       the law is retuned again; the law itself is pinned by value in
       test_the_capture_distance_is_the_ports_own_size(). */
    const double cap = Vehicle::dockCaptureRange(portA, portB);
    CHECK_NEAR(cap, 0.10, 1e-12, "two 0.25 m ports: capture opens at 10 cm of face gap");

    /* The aligned pair, faces touching: every gate open. */
    Vehicle::DockSit s = A.v->dockSituation();
    CHECK_TRUE(s.why == Vehicle::DockSit::Go, "aligned and touching: GO");
    CHECK_TRUE(s.b == B.v && s.arm == portA && s.target == portB,
               "the situation names the pair it measured");
    CHECK_TRUE(s.read.measurable, "a port-centre line exists, so the read is real");
    CHECK_NEAR(s.read.axisCosA, 1.0, 1e-12, "my face points at the other port");
    CHECK_NEAR(s.read.axisCosB, 1.0, 1e-12, "and theirs points back");
    CHECK_NEAR(s.read.faceGap, 0.0, 1e-12, "faces coincide: the capture gate is open");
    CHECK_NEAR(s.read.turnYaw, 0.0, 1e-12, "my bore is on the line: the dial asks no turn");
    CHECK_NEAR(s.read.turnPitch, 0.0, 1e-12, "in either key");
    CHECK_NEAR(s.read.pushRight, 0.0, 1e-12, "and my port sits on their approach line");
    CHECK_NEAR(s.read.pushUp, 0.0, 1e-12, "in either key");
    CHECK_NEAR(s.read.along, 0.0, 1e-12, "nothing short or past to nudge");
    CHECK_NEAR(s.read.closing, 0.0, 1e-12, "and neither port is moving");

    /* Each port presents whichever of its two faces is nearer the other part, so a port
       mounted the other way round still mates. Turn B 180 deg about its own port: its
       authored +Z now points AWAY down the line, and the read must still land on 1.0
       rather than -1.0. This is the ordinary case, not a corner -- two ships meeting
       nose to nose carry ports whose +Z faces out of each hull, away from the meeting.
       Mutation-checked: dropping the face choice on the target's axis passes every other
       check in this file, because the aligned fixture happens to need no flip. */
    B.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz)),
                   glm::mat3_cast(glm::angleAxis(std::numbers::pi,
                                                 glm::dvec3(0.0, 1.0, 0.0))));
    s = A.v->dockSituation();
    CHECK_NEAR(s.read.axisCosB, 1.0, 1e-12,
               "a port facing the other way still presents its near face");
    CHECK_NEAR(s.read.faceGap, 0.0, 1e-12, "and the face it presents is the one measured");
    CHECK_TRUE(s.why == Vehicle::DockSit::Go, "so a flipped port is not a refusal");
    A.v->placeShip(glm::dvec3(0.0),
                   glm::mat3_cast(glm::angleAxis(std::numbers::pi,
                                                 glm::dvec3(0.0, 1.0, 0.0))));
    s = A.v->dockSituation();
    CHECK_NEAR(s.read.axisCosA, 1.0, 1e-12, "the same flip on MY port reads 1.0 too");
    CHECK_TRUE(s.why == Vehicle::DockSit::Go, "two flipped ports still mate");
    A.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    /* The face sits at HALF the part height along the presented axis, not at the part
       centre: move the other hull back 1.5 m and the gap that grows is exactly that 1.5 m,
       because both faces moved with their parts. */
    B.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz) - 1.5), glm::dmat3(1.0));
    s = A.v->dockSituation();
    CHECK_NEAR(s.read.faceGap, 1.5, 1e-12, "the gap is measured face to face");
    CHECK_TRUE(s.read.faceGap >= cap,
               "and 1.5 m is outside the capture gate this pair gets (#1152: 1.5 m used to "
               "BE the gate, six port-depths of daylight on a 0.25 m port)");

    /* Exactly ON the boundary is the row that separates a `>=` from a `>`, and it cannot
       be done with this pair: 0.4 x 0.25 m is 0.1 m, which is not a binary fraction, so the
       read lands a few times 1e-17 off its own threshold and the verdict there is a
       rounding coin flip (measured: the row below reads TooWide at 1.5 m and GO at a
       distance that should have been exactly the limit). The same discrimination with a
       floor that IS representable: give both ports 0.1 m of collision margin, which floors
       their captureRange at 1.25 x 0.2 = 0.25 m, and place the faces exactly 0.5 m apart
       about two 0.125 m port halves. */
    portA->body->hull_margin = portB->body->hull_margin = 0.1;
    CHECK_NEAR(Vehicle::dockCaptureRange(portA, portB), 0.25, 1e-15,
               "the margined pair's floor");
    B.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz) - 0.25), glm::dmat3(1.0));
    s = A.v->dockSituation();
    CHECK_NEAR(s.read.faceGap, 0.25, 1e-15, "faces exactly on that floor");
    CHECK_TRUE(s.read.shutGap, "exactly on the limit is outside it");
    CHECK_TRUE(s.why == Vehicle::DockSit::TooWide, "and the verdict agrees");
    B.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz) - 0.2499999), glm::dmat3(1.0));
    s = A.v->dockSituation();
    CHECK_TRUE(!s.read.shutGap, "1e-7 inside it: the lamp goes green");
    CHECK_TRUE(s.why == Vehicle::DockSit::Go, "so the gate is >=, not >");
    portA->body->hull_margin = portB->body->hull_margin = -1.0;
    B.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz)), glm::dmat3(1.0));

    /* Off-axis: turn B 25 deg about its own port, which is the one rotation that moves
       the FACE without moving either port centre. cos 25 = 0.906 is outside the cone,
       and the face swings off my axis by halfB * sin(25). */
    const double yaw = 25.0 * std::numbers::pi / 180.0;
    B.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz)),
                   glm::mat3_cast(glm::angleAxis(yaw, glm::dvec3(0.0, 1.0, 0.0))));
    s = A.v->dockSituation();
    CHECK_NEAR(s.read.axisCosB, std::cos(yaw), 1e-12, "their face is 25 deg off the line");
    CHECK_NEAR(s.read.axisCosA, 1.0, 1e-12, "mine still points at it");
    /* Their line has swung, so my port centre now sits off the APPROACH path -- and the
       shift the dial can ask for is only the part of that which the L/I keys reach. The
       rest lies along my own nose, where no transverse key goes, which is why the figure
       carries a cos() rather than the full 2*halfB*sin() perpendicular distance. Mine
       still points at the port, so the turn markers stay in the centre: the honest advice
       here is to close and let the cone shrink, not to rotate. Positive pushRight is the
       RcsRight key, and the port-centre separation is 2*halfB straight down -Z. */
    CHECK_NEAR(s.read.pushRight, 2.0 * kPortHz * std::sin(yaw) * std::cos(yaw), 1e-12,
               "off their line, in the key that gets me back on it");
    CHECK_NEAR(s.read.pushUp, 0.0, 1e-12, "and nothing up or down");
    CHECK_NEAR(s.read.turnYaw, 0.0, 1e-12, "my bore is still on the line, so no turn asked");
    CHECK_TRUE(s.read.faceGap < cap,
               "close enough to mate, which is why the axis is the only reason not to");
    CHECK_TRUE(s.why == Vehicle::DockSit::Misaligned, "off-axis: MISALIGNED");

    /* The other half of the dial: MY bore off the line. Yaw A 10 deg about world Y --
       through its own port centre, so neither port moves -- which leaves the two ports
       coaxial (push stays zero) while axisA comes off the line by exactly 10 deg. The
       SIGN is the dial's whole contract: undoing this yaw is a rotation about -keyR[1],
       which is the axis the yaw-right demand turns about (tick.cpp sends YawRight as +1,
       applyRotationForce reads it as -axes[1]), so the figure must read +10 deg --
       positive means press the key beside the marker. Get the sign wrong here and every
       number on the panel tells the pilot to press the one key that makes it worse. */
    B.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz)), glm::dmat3(1.0));
    const double turn = 10.0 * std::numbers::pi / 180.0;
    A.v->placeShip(glm::dvec3(0.0),
                   glm::mat3_cast(glm::angleAxis(turn, glm::dvec3(0.0, 1.0, 0.0))));
    s = A.v->dockSituation();
    CHECK_NEAR(s.read.turnYaw, turn, 1e-9, "10 deg off the line, signed for the yaw-right key");
    CHECK_NEAR(s.read.turnPitch, 0.0, 1e-12, "a yaw error leaves the pitch marker at the centre");
    CHECK_NEAR(s.read.pushRight, 0.0, 1e-12, "and the ports are still coaxial");
    CHECK_NEAR(s.read.portRange, 2.0 * kPortHz, 1e-12, "range is port centre to port centre");
    A.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    /* Cone boundary, from the constant rather than a literal: 14.9 deg fits, 15.1 does
       not. kDockAlign is a cosine, so the test asks in degrees and converts. */
    const double cone = std::acos(Vehicle::kDockAlign);
    B.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz)),
                   glm::mat3_cast(glm::angleAxis(cone * 0.99, glm::dvec3(0.0, 1.0, 0.0))));
    CHECK_TRUE(A.v->dockSituation().why == Vehicle::DockSit::Go,
               "just inside the cone: still GO");
    B.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz)),
                   glm::mat3_cast(glm::angleAxis(cone * 1.01, glm::dvec3(0.0, 1.0, 0.0))));
    CHECK_TRUE(A.v->dockSituation().why == Vehicle::DockSit::Misaligned,
               "just outside it: MISALIGNED, and neither answer is a rounding coin flip");

    /* The capture boundary the same way, since the window prints that row too: 1% inside
       fits, 1% outside does not. Together with the exact-on-limit row above, this pins the
       gate's `>=` against a `>` and against a bare `<`. */
    B.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz) - cap * 0.99),
                   glm::dmat3(1.0));
    s = A.v->dockSituation();
    CHECK_TRUE(!s.read.shutGap, "1% inside capture: the gap lamp stays green");
    CHECK_TRUE(s.why == Vehicle::DockSit::Go, "and the pair mates");
    B.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz) - cap * 1.01),
                   glm::dmat3(1.0));
    s = A.v->dockSituation();
    CHECK_TRUE(s.read.shutGap, "1% outside: the lamp the window lights");
    CHECK_TRUE(s.why == Vehicle::DockSit::TooWide, "and the verdict agrees with the lamp");

    /* Too fast, with everything else open. */
    B.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz)), glm::dmat3(1.0));
    A.v->setVelocity(glm::dvec3(0.0, 5.0, 0.0));
    s = A.v->dockSituation();
    CHECK_NEAR(s.read.relSpeed, 5.0, 1e-12, "the port speed is the two ports' relative speed");
    CHECK_TRUE(s.read.shutSpeed, "too fast, by the gate's own answer");
    CHECK_TRUE(s.why == Vehicle::DockSit::TooFast, "too fast says so too");
    A.v->setVelocity(glm::dvec3(0.0, 0.0, 0.0));

    /* The RELATIVE in relative speed: every other velocity fixture here moves MY hull, so
       a read that subtracted nothing from partVel(pa) -- forgetting the target entirely --
       still measured 5.0 and passed. Move only theirs, and read the sign too: theirs going
       down the line is the two ports separating, not closing. */
    B.v->setVelocity(glm::dvec3(0.0, 0.0, -5.0));
    s = A.v->dockSituation();
    CHECK_NEAR(s.read.relSpeed, 5.0, 1e-12, "their hull moving is the same gate");
    CHECK_NEAR(s.read.closing, -5.0, 1e-12, "and it reads as separating, not approaching");
    CHECK_TRUE(s.why == Vehicle::DockSit::TooFast, "so a fast target is refused as fast");
    B.v->setVelocity(glm::dvec3(0.0, 0.0, 0.0));
    CHECK_TRUE(A.v->dockSituation().why == Vehicle::DockSit::Go,
               "braked: open again, and no leftover velocity in the fixture");

    /* Signed, because faceGap cannot tell "still short" from "already past the socket"
       -- and the capture gate has no lower bound, so past it is a state a pilot reaches.
       B's centre INSIDE my port's half-height crosses the two faces: along goes
       negative while faceGap, being a length, does not. */
    const double q = kPortHz * 0.4;                    // 0.05 m: inside the half-height
    B.v->placeShip(glm::dvec3(0.0, 0.0, -q), glm::dmat3(1.0));
    s = A.v->dockSituation();
    CHECK_TRUE(s.read.measurable, "the two port CENTRES are still apart, so it measures");
    CHECK_NEAR(s.read.along, -(2.0 * kPortHz - q), 1e-12,
               "past the socket reads NEGATIVE along");
    CHECK_NEAR(s.read.faceGap, 2.0 * kPortHz - q, 1e-12, "while faceGap stays unsigned");
    B.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz) - 1.0), glm::dmat3(1.0));
    CHECK_NEAR(A.v->dockSituation().read.along, 1.0, 1e-12, "short of it reads positive");

    /* Closing is signed the other way: + = coming together. */
    B.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz) - 1.0), glm::dmat3(1.0));
    A.v->setVelocity(glm::dvec3(0.0, 0.0, -1.5));
    CHECK_NEAR(A.v->dockSituation().read.closing, 1.5, 1e-12,
               "heading down the line at the other port: +1.5 closing");
    A.v->setVelocity(glm::dvec3(0.0, 0.0, 0.0));

    /* The states that are not geometry, in the gate's order. Each of these used to be
       a tick in which nothing happened and nothing said why. */
    B.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz)), glm::dmat3(1.0));
    CHECK_TRUE(A.v->dockSituation().why == Vehicle::DockSit::Go,
               "back to GO before the state cases, so each one below isolates its reason");
    A.v->onRails = true;
    CHECK_TRUE(A.v->dockSituation().why == Vehicle::DockSit::MyRails,
               "a hull on the rails is not docking, whatever its ports look like");
    A.v->onRails = false;
    B.v->onRails = true;
    CHECK_TRUE(A.v->dockSituation().why == Vehicle::DockSit::TargetRails,
               "a target still on the rails names ITSELF as the reason");
    B.v->onRails = false;

    A.v->dockArmPort = nullptr;
    CHECK_TRUE(A.v->dockSituation().why == Vehicle::DockSit::NoArm, "no port armed");
    A.v->dockArmPort = portA;
    A.v->dockTargetPort = nullptr;
    CHECK_TRUE(A.v->dockSituation().why == Vehicle::DockSit::NoTarget, "no port targeted");
    A.v->dockTargetPort = portB;
    A.v->dockTargetShip = nullptr;
    CHECK_TRUE(A.v->dockSituation().why == Vehicle::DockSit::NoTarget,
               "a target port with no target hull is the same half missing");
    A.v->dockTargetShip = B.v;

    /* A stale half: the part both halves point at still exists but stopped being what
       the intent needs. The gate DROPS these (dockSituation only names them), which is
       what lets a re-arm retry. */
    A.v->dockTargetPort = portA;      // "my own port" -- the target check rules out self
    CHECK_TRUE(A.v->dockSituation().why == Vehicle::DockSit::TargetGone,
               "targeting a port on this same hull is not a target");
    A.v->dockTargetPort = portB;
    A.v->dockArmPort = portA;
    CHECK_TRUE(A.v->dockSituation().why == Vehicle::DockSit::Go,
               "and a clean intent answers GO again, not a stale complaint");

    /* The two guards that ask whether the part is a PORT at all. The A/B pair cannot reach
       them: the "not a port" part an earlier version of this test built was never attached
       to anything, so every row landed in the identity scan instead of the port test, and
       deleting isDockingPort() from either loop kept all 222 checks green. Two-part hulls
       here, so the tanks are genuinely aboard and genuinely not ports. */
    Ship R; Part *tankR, *portR;
    mkTankPort(R, "R", 0.0, true, &tankR, &portR);
    Ship Sq; Part *tankSq, *portSq;
    mkTankPort(Sq, "Sq", -(2.0 * kPortHz), true, &tankSq, &portSq);
    R.v->dockArmPort = portR;
    R.v->dockTargetShip = Sq.v;
    R.v->dockTargetPort = portSq;
    CHECK_TRUE(R.v->dockSituation().why == Vehicle::DockSit::Go,
               "guards: two real ports two port-heights apart, intent accepted");
    R.v->dockTargetPort = tankSq;     // aboard Sq, but not a port
    CHECK_TRUE(R.v->dockSituation().why == Vehicle::DockSit::TargetGone,
               "guards: a targeted part that is aboard and is not a port is gone");
    R.v->dockTargetPort = tankR;      // aboard the WRONG hull
    CHECK_TRUE(R.v->dockSituation().why == Vehicle::DockSit::TargetGone,
               "guards: a targeted part aboard some other hull is gone too");
    R.v->dockTargetPort = portSq;
    R.v->dockArmPort = tankR;         // aboard R, but not a port
    CHECK_TRUE(R.v->dockSituation().why == Vehicle::DockSit::ArmGone,
               "guards: an armed part that is aboard and is not a port is gone");
    R.v->dockArmPort = portR;
    CHECK_TRUE(R.v->dockSituation().why == Vehicle::DockSit::Go,
               "guards: and a clean intent on the same pair answers GO");
    destroyShip(Sq.v);
    destroyShip(R.v);

    /* Yaw MINE, not theirs. The shipped --dock-test misaligned fixture turns the ARMED
       port, while every fixture above turns the target's; the window prints those as two
       separate rows, and a readout that only ever fills one of them cannot show that the
       pilot and the target each have an axis to fix. */
    A.v->placeShip(glm::dvec3(0.0),
                   glm::mat3_cast(glm::angleAxis(cone * 1.01, glm::dvec3(0.0, 1.0, 0.0))));
    s = A.v->dockSituation();
    CHECK_NEAR(s.read.axisCosA, std::cos(cone * 1.01), 1e-12,
               "turning my hull moves the axis-mine row");
    CHECK_NEAR(s.read.axisCosB, 1.0, 1e-12, "and leaves their axis reading straight");
    CHECK_TRUE(s.why == Vehicle::DockSit::Misaligned,
               "either port outside the cone refuses, with the same word either way");
    A.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    /* No line, therefore no numbers. Two port CENTRES at one point have no direction to
       measure along, so the read stays unmeasurable and the window prints no gate rows --
       it would otherwise show a 0.00 m gap and a GO for two ships occupying one spot.
       B is turned 180 deg about its own port to make this fixture worth having: two ports
       at one point FACING the same way present the same face, so faceGap and along read
       0.0 even with the guard gone. Flipped, an unguarded read would invent a 0.25 m gap
       and a -0.25 m nudge out of a division by zero. */
    B.v->placeShip(glm::dvec3(0.0),
                   glm::mat3_cast(glm::angleAxis(std::numbers::pi,
                                                 glm::dvec3(0.0, 1.0, 0.0))));
    s = A.v->dockSituation();
    CHECK_TRUE(!s.read.measurable, "coincident port centres: nothing to measure");
    CHECK_TRUE(s.why == Vehicle::DockSit::Unmeasurable,
               "and it says so rather than inventing a touching pair");
    CHECK_NEAR(s.read.faceGap, 0.0, 1e-12, "no distance invented for an unmeasurable pair");
    CHECK_NEAR(s.read.along, 0.0, 1e-12, "and no nudge direction invented either");
    CHECK_NEAR(s.read.axisCosA, 1.0, 1e-12, "no cosine invented either -- the normals are 0/0");
    CHECK_NEAR(s.read.axisCosB, 1.0, 1e-12, "same on their side");
    CHECK_TRUE(!s.read.shutAxis && !s.read.shutGap && !s.read.shutSpeed,
               "and no lamp lit from a figure that was never measured");
    B.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz)), glm::dmat3(1.0));

    /* Another frame: the gate cannot measure a pair that is not in one place, so it does
       not try. Every fixture above leaves both hulls on the null frame the test never
       sets, so this is the first time that comparison is not trivially equal. */
    Frame other;
    Frame *savedFrame = B.v->frame;
    B.v->frame = &other;
    s = A.v->dockSituation();
    CHECK_TRUE(s.why == Vehicle::DockSit::OtherFrame,
               "a target in another frame waits, whatever its ports look like");
    B.v->frame = savedFrame;
    CHECK_TRUE(A.v->dockSituation().why == Vehicle::DockSit::Go,
               "back in one frame: GO, not a leftover complaint");

    /* A PORT's speed, not its hull's (#1026). dockRead asks partVel() at both ends, so a
       station whose centre of mass is frozen still has to answer for the port it is
       offering, which is sweeping past. Asking the hull instead would let this pair mate
       at more than twice kDockMaxV. The hulls are two-part stacks on purpose: a
       single-port hull puts its port at its own COM, where the cross product vanishes and
       the whole distinction disappears -- which is how this case nearly went untested. */
    Ship E; Part *tankE, *portE;
    mkTankPort(E, "spin", 0.0, false, &tankE, &portE);   // tank root, port hanging below
    Ship F; Part *tankF, *portF;
    mkTankPort(F, "still", 0.0, true, &tankF, &portF);   // port root, tank below it
    E.v->placeShipAtCom(glm::dvec3(0.0), glm::dmat3(1.0));
    const double rE = glm::length(E.v->partPos(portE) - E.v->comPos());
    CHECK_TRUE(rE > 1.0, "spin fixture: the port sits a metre and a half off its COM");
    /* Faces touching, derived rather than hand-worked -- the offset depends on how the
       tank and the port divide the mass. */
    const glm::dvec3 wantPort = E.v->partPos(portE) - glm::dvec3(0.0, 0.0, 2.0 * kPortHz);
    F.v->placeShipAtCom(F.v->comPos() + (wantPort - F.v->partPos(portF)), glm::dmat3(1.0));
    E.v->dockArmPort = portE;
    E.v->dockTargetShip = F.v;
    E.v->dockTargetPort = portF;
    s = E.v->dockSituation();
    CHECK_NEAR(s.read.faceGap, 0.0, 1e-12, "spin fixture: faces touching");
    CHECK_TRUE(s.why == Vehicle::DockSit::Go, "spin fixture: both still, so it mates");

    /* Roll about the docking line: nothing on that line moves, so this is a spin the
       pilot is entitled to be doing while they close. */
    SetAngVelocity(E.v->hull, glm::dvec3(0.0, 0.0, 3.0));
    s = E.v->dockSituation();
    CHECK_NEAR(s.read.relSpeed, 0.0, 1e-9, "spinning about the line moves no port");
    CHECK_TRUE(s.why == Vehicle::DockSit::Go, "so a roll about the axis is not a refusal");

    /* Now about an axis ACROSS it: the hull's COM is exactly where it was and its port is
       not, at 3 rad/s * 1.55 m = 4.6 m/s. */
    SetAngVelocity(E.v->hull, glm::dvec3(3.0, 0.0, 0.0));
    s = E.v->dockSituation();
    CHECK_NEAR(s.read.relSpeed, 3.0 * rE, 1e-9,
               "the port-speed row reads the port, not the still centre of its hull");
    CHECK_NEAR(s.read.closing, 0.0, 1e-9, "a sweep sideways is not a closing rate");
    CHECK_TRUE(s.why == Vehicle::DockSit::TooFast,
               "and a spinning station is refused on speed, not on luck");

    /* Both spinning: what matters is the two ports' DIFFERENCE, so a read that asked only
       the armed port's velocity (or only the target's) reads 4.6 here instead of 3.1. The
       target's rate is NEGATIVE because these two hulls hang their ports on opposite sides
       of their own centres of mass -- E's root is the tank, F's is the port -- so the same
       signed rate sweeps their ports opposite ways, and matching directions takes speed off. */
    const double rF = glm::length(F.v->partPos(portF) - F.v->comPos());
    SetAngVelocity(F.v->hull, glm::dvec3(-1.0, 0.0, 0.0));
    s = E.v->dockSituation();
    CHECK_NEAR(s.read.relSpeed, 3.0 * rE - 1.0 * rF, 1e-9,
               "their port sweeping the same way takes speed OFF, not adds a second number");
    CHECK_TRUE(s.why == Vehicle::DockSit::TooFast, "still over the limit, now by 3.1 m/s");
    SetAngVelocity(E.v->hull, glm::dvec3(0.0));
    SetAngVelocity(F.v->hull, glm::dvec3(0.0));
    CHECK_TRUE(E.v->dockSituation().why == Vehicle::DockSit::Go,
               "spin both off: the pair mates again, nothing left in the body");

    /* THE ORDER the questions are asked in: axis, then distance, then speed, then the mass
       door. The verdict is ONE word, so which gate answers first is the advice the pilot
       gets, and every row below pairs two reasons that could have answered and pins which
       one did. A pair 3 m out and absurdly heavy is too FAR, and only once it is close does
       the mass door (#583) have anything to say: move that door up the list and every
       approach in a heavy game opens with a weight complaint the player cannot act on. */
    Ship G; G.v = new Vehicle; G.v->name = "G";
    Part *portG = mkPart(G, "portG", 1.7e308, 1.0, 1.0, kPortHz, true);
    G.v->setRoot(portG);
    G.v->controller = portG;
    G.v->init();
    G.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));
    Ship H; H.v = new Vehicle; H.v->name = "H";
    Part *portH = mkPart(H, "portH", 1.7e308, 1.0, 1.0, kPortHz, true);
    H.v->setRoot(portH);
    H.v->controller = portH;
    H.v->init();
    G.v->dockArmPort = portG;
    G.v->dockTargetShip = H.v;
    G.v->dockTargetPort = portH;

    H.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz) - 3.0), glm::dmat3(1.0));
    s = G.v->dockSituation();
    CHECK_TRUE(s.why == Vehicle::DockSit::TooWide,
               "far away AND an impossible pair: the distance answers, not the mass");
    G.v->setVelocity(glm::dvec3(0.0, 5.0, 0.0));
    CHECK_TRUE(G.v->dockSituation().why == Vehicle::DockSit::TooWide,
               "far away AND moving: still the distance -- speed is not asked first");
    G.v->setVelocity(glm::dvec3(0.0));
    H.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz) - 3.0),
                   glm::mat3_cast(glm::angleAxis(cone * 1.01, glm::dvec3(0.0, 1.0, 0.0))));
    CHECK_TRUE(G.v->dockSituation().why == Vehicle::DockSit::Misaligned,
               "far away AND off-axis: the axis answers before the distance");
    H.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz)), glm::dmat3(1.0));
    G.v->setVelocity(glm::dvec3(0.0, 5.0, 0.0));
    CHECK_TRUE(G.v->dockSituation().why == Vehicle::DockSit::TooFast,
               "in range but moving: speed answers before the mass door");
    H.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz)),
                   glm::mat3_cast(glm::angleAxis(cone * 1.01, glm::dvec3(0.0, 1.0, 0.0))));
    CHECK_TRUE(G.v->dockSituation().why == Vehicle::DockSit::Misaligned,
               "in range, moving AND off-axis: the axis still answers first");
    G.v->setVelocity(glm::dvec3(0.0));
    H.v->placeShip(glm::dvec3(0.0, 0.0, -(2.0 * kPortHz)), glm::dmat3(1.0));   // unyaw, unshift
    s = G.v->dockSituation();
    CHECK_TRUE(s.read.faceGap < Vehicle::dockCaptureRange(portG, portH),
               "order: now inside capture");
    CHECK_TRUE(s.why == Vehicle::DockSit::Mass,
               "so the mass door is the only gate left standing, and it speaks");
    CHECK_TRUE(!s.massWhy.empty(),
               "and it hands the window #583's own sentence rather than a paraphrase");

    destroyShip(H.v);
    destroyShip(G.v);
    destroyShip(F.v);
    destroyShip(E.v);
    destroyShip(B.v);
    destroyShip(A.v);
}

/* #1044. WHICH FRAME THE KEYS ARE RESOLVED IN. dockRead() answers its turn and push
   figures on attitudeRef()'s axes, because that is the basis applyRotationForce() reads the
   rotation stick in (vehicle.cpp:4870) and rcsWorldDir() the translation keys in
   (vehicle.cpp:4952). Every other fixture in this file is a one-part hull, so there the
   armed port IS the attitude reference and the two bases are the same matrix: spelling
   that line `keyR = paR` keeps all of them green while a shipped sideways port --
   res/ships/sideways_wheel.json mounts one at angle: 90, and src/vehicle.h:1931 says the
   displayed attitude asks this same part -- plots its markers a quarter turn away from the
   keys they name.

   The fixture hangs a port off a tank with 90 deg of roll about the stack axis, through
   attachMode's angleDeg, which IS that roll -- the game's own mount path rather than a
   hand-set localRot. A roll about the bore leaves the bore alone, so every AXIS figure is
   exactly what a square-mounted port would read; only the key-resolved figures move. That
   leaves this fixture pinning the frame choice and nothing else. The roll's sign is not
   pinned, deliberately: the expected figures below are the ROOT's and do not depend on it. */
static void test_a_sideways_port_moves_the_dial_but_not_the_verdict() {
    printf("== dockRead: a rolled port still answers in the hull's keys (#1044) ==\n");

    /* The two offsets the read is built from, named so the expectations below are the
       fixture's own geometry rather than a magic number. */
    const double lat = 1.0;     // m, along the world's +X: the hull's RIGHT
    const double down = 1.0;    // m, further along -Z

    Ship A; A.v = new Vehicle; A.v->name = "A";
    Part *tankA = mkPart(A, "tankA", 800.0, 1.0, 1.0, kTankHz, false);
    A.v->setRoot(tankA);
    A.v->controller = tankA;
    Part *portA = mkPart(A, "portA", 50.0, 1.0, 1.0, kPortHz, true);
    A.v->attachMode(portA, partIndex(A.v, tankA), AttachMode::Down, 90.0);
    A.v->init();
    A.v->placeShip(glm::dvec3(0.0), glm::dmat3(1.0));

    Ship B; B.v = new Vehicle; B.v->name = "B";
    Part *portB = mkPart(B, "portB", 50.0, 1.0, 1.0, kPortHz, true);
    B.v->setRoot(portB);
    B.v->controller = portB;
    B.v->init();
    /* A's port hangs kTankHz + kPortHz under the tank; B's port sits `down` below that and
       `lat` to the world's +X. So the joining line comes 45 deg off both bores, and the
       whole perpendicular offset lies along the one axis the two bases disagree about. */
    B.v->placeShip(glm::dvec3(lat, 0.0, -(kTankHz + kPortHz) - down), glm::dmat3(1.0));

    A.v->dockArmPort = portA;
    A.v->dockTargetShip = B.v;
    A.v->dockTargetPort = portB;

    /* The fixture's own premises first: a test that pins a frame choice is worthless if the
       hull being measured has no frame to choose, and worthless if the roll quietly moved
       the bore -- that would make the rows below about the axis gates instead. */
    CHECK_TRUE(A.v->attitudeRef() == tankA && A.v->attitudeRef() != portA,
               "the hull asks its ROOT part for its axes, and the armed port is not that part");
    CHECK_NEAR(std::fabs(glm::dot(A.v->partAxis(tankA, 2), A.v->partAxis(portA, 2))), 1.0, 1e-12,
               "90 deg of roll about the stack axis leaves the BORE exactly where it was");
    CHECK_NEAR(glm::dot(A.v->partAxis(tankA, 0), A.v->partAxis(portA, 0)), 0.0, 1e-12,
               "and the bases are a quarter turn apart: the hull's right is one of the port's sides");

    Vehicle::DockSit s = A.v->dockSituation();
    CHECK_NEAR(s.read.portRange, std::sqrt(lat * lat + down * down), 1e-12,
               "range is port centre to port centre, and the ports never moved");
    CHECK_NEAR(s.read.axisCosA, down / std::sqrt(lat * lat + down * down), 1e-12,
               "my bore sits 45 deg off the line -- the same as an unrolled port");
    CHECK_NEAR(s.read.axisCosB, down / std::sqrt(lat * lat + down * down), 1e-12,
               "and theirs by the same amount: the roll changes no axis figure at all");
    CHECK_TRUE(s.read.shutAxis, "45 deg is outside the cone");
    CHECK_TRUE(s.why == Vehicle::DockSit::Misaligned, "so that is the one reason this pair refuses");

    /* THE FIGURES THE DOCKING PANEL PLOTS. turnYaw answers the 45 deg with the hull's UP
       axis, and the error lies wholly in that plane, so the pitch key is asked for nothing;
       pushRight answers the whole 1 m with the hull's RIGHT, and pushUp nothing. Resolve
       those same two pairs on the port's basis and each one lands on the OTHER key -- which
       is exactly the lie: the panel would put its marker beside D while the key that centres
       the bore is W, and beside RcsUp while the key that gets onto their path is RcsRight. */
    CHECK_NEAR(s.read.turnYaw, std::atan2(lat, down), 1e-12,
               "45 deg of turn, read along the HULL's up axis rather than the port's side");
    CHECK_NEAR(s.read.turnPitch, 0.0, 1e-12,
               "a yaw-plane error asks no pitch key, even through a rolled port");
    CHECK_NEAR(s.read.pushRight, lat, 1e-12,
               "the offset is 1 m along the hull's right: RcsRight is the key that answers it");
    CHECK_NEAR(s.read.pushUp, 0.0, 1e-12, "and the hull's up axis has no part in that offset");

    destroyShip(B.v);
    destroyShip(A.v);
}

int main() {
    test_absorb();
    test_a_dock_keeps_the_pair_s_momentum();
    printf("\n");
    test_a_dock_keeps_the_pair_s_spin();
    test_orders_return_on_undock();
    test_a_stage_cut_at_a_seam_root();
    test_a_seam_with_no_records_falls_back_to_defaults();
    test_roundtrip();
    test_uid();
    test_seam_staging();
    test_the_root_parts_stage_is_counted();
    test_seam_nested_dock();
    test_seam_kept_across_stage();
    test_experiments_ride_parts();
    test_dock_mass_door();
    test_the_capture_distance_is_the_ports_own_size();
    test_the_dock_situation_says_why_a_pair_will_not_mate();
    test_a_sideways_port_moves_the_dial_but_not_the_verdict();

    printf("%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures) { printf("FAILED\n"); return 1; }
    printf("test_dock: all checks passed\n");
    return 0;
}
