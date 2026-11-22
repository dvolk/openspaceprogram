// THE AERO BENCH (#655): one authored hull, flown through a grid of alpha / beta /
// Mach, printing -- and pinning -- the whole-surface law's answers: lift, side force,
// the three drag bills (hull silhouette, surface profile, induced), the moment, the
// centre of pressure, the lift slope and the static margin.
//
// Why this file exists: every aero law change was judged by FLYING ONE PLANE. The
// law's pieces are pinned one function at a time in tests/test_drag.cpp, and the
// shipped hulls are pinned one flight at a time in e2e/cases/flight/ -- but neither
// is a table over the argument space, so "did this change move the whole surface, or
// only the point I happened to fly through?" had no answer (#655).
//
// What it pins that nothing else did:
//   THE COMPOSITION, not the parts. A surface pays a profile bill, a slope force and
//   an induced bill, all read at its own flow, and only applyAeroForce puts them
//   together. Pinning the sums is what catches a law that double-charges, or stops
//   charging: a pane that falls out of its lifting system, an AR that short-circuits
//   inducedCd, a regime edge that moves.
//   THE CONTROL SURFACE'S AUTHORITY CURVE (#494's droop, #503's fix): force against
//   commanded deflection across the whole travel. It is NOT monotone -- see
//   test_surface_authority_curve -- because the shipped travel stop sits PAST the
//   shipped stall angle, so the outer quarter of the stick's throw pulls weaker. The
//   same sweep prints the tab's own shape bill, which never moves however hard it is
//   turned (#506's other half).
//   THE LIFTING-SYSTEM MERGE (#556/#562): the same tab hung off the wing's trailing
//   edge joins the wing's system, and the wing's induced bill drops 20% for it.
//   THE INDUCED SUM'S SIGNS (#494's fix, untracked half): the system's CL is a sum of
//   |F|, so a tab's download bills exactly what its upload bills -- and a downloading
//   trim that takes a quarter of the lift off the airframe more than doubles its
//   induced bill. See test_induced_sum_is_sign_blind.
//   THE RATE COLUMN (#487, the column #655's header named as missing): the same hull
//   with an angular velocity, and the rate's OWN moment -- the difference against the
//   same hull with no spin. Its sign is the result: damping inside the linear range,
//   a PUMP on the airframe's own rotation inside the stall droop. See test_rate_column.
//   THE ENERGY HOOKS a heating model would read (#553/#554/#728): exact per surface,
//   and since #1090 an accounted credit share for bodies too -- which is what a
//   capsule is made of.
//
// The fixture is a Vehicle, not a re-implementation: init() builds the real compound
// hull (never enterWorld, as in test_authority / test_dock), the parts carry real
// collision hull verts so the silhouette union, the cp centroid and the weathervane arm
// all run, and the numbers come back out of applyAeroForce's own last* readouts -- the
// same fields --drag-log prints. Flow is swept by setting the hull's VELOCITY, never
// its pose, so at fly()'s default rate of 0 the hull is non-rotating and every
// surface's localFlow (#483) equals the COM's: the bench is a steady stream, which is
// what makes the expectations hand-checkable. A rotating hull is pinned for the pure
// math in test_drag.cpp (test_local_flow_sweep) and for the whole-airframe answer here
// (test_rate_column, #655's rate column).
//
// Build & run (from repo root) -- also part of `make test`:
//   see the test_aerosweep rule in the Makefile (TBASE: it constructs a Part and
//   calls btBoxShape, so it needs the Bullet archive and terrain.cpp's TerrainBody).

#include "btcommon.h"

#include <cassert>
#include <cmath>
#include <algorithm>   // std::max/min: which tab sign loads the airframe
#include <cstdio>
#include <deque>
#include <numbers>
#include <utility>   // std::make_pair: the (Induced, net L) pair per trim shot
#include <vector>

#include <glm/glm.hpp>

#include "drag.h"      // flowDirection, liftCurve, liftSlopeMach, waveCd, kSupersonicMach
#include "physics.h"   // SetAngVelocity: the rate column
#include "vehicle.h"   // Vehicle + Part (inline); pulls in Bullet (btcommon.h)
#include "terrain.h"   // TerrainBody: the air this hull flies in
#include "tchecks.h"

/* A body with the fields airDensity / airSoundSpeed read: 600 km radius, a sea-level
   atmosphere, and the gravity that atmosphere implies (#552 A2 -- the loader refuses a
   body with air and no g, and applyAeroForce asserts the sound speed it derives). The
   sound speed here is sqrt(gamma g H) = 331 m/s, so the sweep's speeds straddle
   M 0.15 / 0.30 / 0.76 / 1.51: subsonic, the transonic band's low edge, the band and
   past its high edge. */
static TerrainBody *mkAirBall() {
    TerrainBody *body = new TerrainBody;
    body->name = "airball";
    body->radius = 600000.0f;
    body->surface.sea_level = 0.0f;
    body->g = 9.81;
    /* dragAtm weighs the air in mu at the datum sphere, not in g (#708), so a
       hand-built body carries the mu the loader would have derived. */
    body->mu = body->g * (double)body->radius * (double)body->radius;
    // hand-built: a FILE authors sea_level_pressure and the loader resolves this (#739)
    body->surface.atmosphere.sea_level_density = 1.225;
    body->surface.atmosphere.scale_height = 8000.0;
    return body;
}

/* A plate part: a box collision shape for the mass and inertia, and -- unlike the
   other test rigs -- hand-filled Body::hullVerts, so the silhouette union, the cp
   centroid and the form-drag arm all run for real. `mx/my/mz` are the mesh box's
   HALF-extents, which is what PartDef::meshSpanHalf / meshThickness / meshEdgeArea /
   meshThicknessToChord read; a def with no mesh falls back to the authored cylinder
   and bills a plate's planform as its edge (#559), so a bench about areas has to give
   the box. */
static Part *mkPlate(std::deque<PartDef> &defs, const char *name, double mass,
                     double mx, double my, double mz) {
    Body *b = new Body;
    b->btBody = nullptr;
    b->shape = new btBoxShape(btVector3((float)mx, (float)my, (float)mz));
    b->mass = mass;
    for(int i = 0; i < 8; i++) {   // the shape's 8 corners, part-local
        b->hullVerts.push_back(glm::dvec3(
            (i & 1) ? mx : -mx, (i & 2) ? my : -my, (i & 4) ? mz : -mz));
    }
    defs.push_back(PartDef());
    PartDef &d = defs.back();
    d.name = name;
    d.mass = (float)mass;
    d.radius = (float)std::max(mx, my);
    d.height = (float)(2.0 * mz);
    d.mesh_min = glm::dvec3(-mx, -my, -mz);
    d.mesh_max = glm::dvec3(mx, my, mz);
    d.has_mesh_extents = true;
    d.synthesizeNodes();
    Part *p = new Part;
    p->body = b;
    p->def = &d;
    p->stage = 1;
    return p;
}

/* The shipped plate aero numbers, in one place, so the pins below can name them:
   a 0.01 profile cd chordwise, a blunt 1.0 out of plane, cl 6.0/rad, and -- for a
   steering part -- a 0.26 rad stall angle against a 0.35 rad travel stop. */
static void setSurface(PartDef &d, double area, bool lift) {
    d.cl = 6.0;
    d.drag_forward = 0.01;
    d.drag_side = 1.0;
    d.drag_backward = 0.01;
    if(lift) {
        d.lift_area = area;
        d.stall_angle = 0.35;          // the shipped wing's 20.06 deg
    } else {
        d.control_area = area;
        d.cl_control = 6.0;
        d.stall_angle = 0.26;          // the shipped steering parts' 14.89 deg
        d.max_deflection = 0.35;       // ... and their 20.06 deg travel stop
        d.control_axis = ControlAxis::Pitch;
    }
}

struct Bench {
    Vehicle *v;
    std::deque<PartDef> defs;
    TerrainBody *air;
    Part *pod;
};

/* One 100 kg fuselage box on the ship origin, in air 1000 m up. Aero surfaces hang off
   it below, along -Z, so every lifting surface sits AFT of the COM.
   Part::def is const (a def is shared by every part of a type), so a fixture tunes a
   surface through the deque that owns it; references into a deque survive push_back,
   so the pointer the Part holds stays valid. */
static Bench mkBench() {
    Bench b;
    b.v = new Vehicle;
    b.v->name = "bench";
    b.pod = mkPlate(b.defs, "pod", 100.0, 0.5, 0.5, 1.0);
    PartDef &pod = b.defs.back();
    pod.drag_forward = 0.35;
    pod.drag_side = 1.0;
    pod.drag_backward = 1.3;
    b.v->setRoot(b.pod);
    b.air = mkAirBall();
    b.v->m_parent = b.air;
    return b;
}

/* Finish the part list and put the hull 1000 m up: 1000 m of column, thin enough to
   matter and thick enough to bill. */
static void launch(Bench &b) {
    b.v->init();
    b.v->placeShip(glm::dvec3(0.0, 0.0, 601000.0), glm::dmat3(1.0));
}

static void teardown(Bench &b) {
    delete b.v->m_parent;
    b.v->m_parent = nullptr;
    delete b.v;
}

/* Fly the hull at (speed, alpha, beta) and step the aero pass once. The hull sits at
   the identity pose with no spin, so frame S is the world and the ship axes are x
   right, y up, z nose. flowDirection() is drag.h's own inverse of aeroFrame's two
   angles, so the bench asks for an angle by NAME instead of hand-building the vector
   and hoping. `pitchRate` is rad/s about the ship's RIGHT axis -- the rate column
   (test_rate_column); at 0 it is exactly the steady stream every pin above assumes. */
static void fly(Vehicle *v, double speed, double alphaDeg, double betaDeg,
                double pitchRate = 0.0) {
    const double a = std::numbers::pi / 180.0 * alphaDeg;
    const double bt = std::numbers::pi / 180.0 * betaDeg;
    const glm::dvec3 flow = flowDirection(a, bt, glm::dvec3(1, 0, 0),
                                          glm::dvec3(0, 1, 0), glm::dvec3(0, 0, 1));
    assert(glm::length(flow) > 0.0);
    v->setVelocity(flow * speed);
    // Identity pose, so the world axes ARE the ship axes: x right, y up, z nose.
    SetAngVelocity(v->hull, glm::dvec3(pitchRate, 0.0, 0.0));
    v->applyAeroForce(0.01, nullptr);
}

/* One row of the table, in the shape --drag-log prints so a reader can put the two
   side by side. Printed, not pinned: the pins below name the formulas. */
static void report(Vehicle *v, double speed, double alpha, double beta) {
    char sm[16];
    if(v->lastStaticValid) { snprintf(sm, sizeof sm, "%+.3f", v->lastStaticMargin); }
    else { snprintf(sm, sizeof sm, "-"); }
    printf("[sweep] v %6.1f  alpha %6.1f  beta %5.1f  rho %7.5f  Mach %5.3f"
           "  L %10.2f N  D %10.2f N  Fright %10.2f N  |tau| %10.2f Nm"
           "  A %6.3f Cd %5.3f  Profile %6.4f  Induced %7.4f  CdAtot %7.4f"
           "  CpL %6.3f  FormCp %6.3f  CLa %8.3f  SM %8s  SideCdA %7.3f  SideCp %6.3f"
           "  BodyCdA %+7.3f  BodyCp %+6.3f\n",
           // Fright: the along-+x force component (the sideslip leg; renamed from
           // "Side" so it cannot be read as the SideCdA= bill two columns over).
           speed, alpha, beta, v->lastDragRho,
           v->lastAirV / (v->lastAirA > 0.0 ? v->lastAirA : 1.0),
           v->lastFlowLift, v->lastFlowDrag,
           glm::dot(v->lastAeroForce, glm::dvec3(1, 0, 0)),
           glm::length(v->lastAeroTorque),
           v->lastDragArea, v->lastDragCd, v->lastSurfCdA, v->lastInducedCdA,
           v->lastSurfCdA + v->lastInducedCdA
               + v->drag_cd * v->lastDragCd * v->lastDragArea,
           v->lastCpLever, v->lastFormCpLever, v->lastLiftSlope, sm,
           v->lastSideCdA, v->lastSideCpLever,
           v->lastBodyCdA, v->lastBodyCpLever);
}

/* --- the grid (#655) -------------------------------------------------------
   One 8 m^2 wing panel (span 4 m, chord 2 m, 0.1 m thick: the shipped `wing` def's
   proportions) hung 2 m aft of the pod's centre, plus a 2 m^2 elevator 2 m further aft
   and neutral. The pins are hand-checkable off the def's numbers:
     q       = 0.5 rho v^2, with rho read back off the pass (test_drag pins the profile)
     CL(a,M) = liftSlopeMach(6.0, M) x alpha, drooping past stall_angle 0.35 rad
     Profile = drag_cd x surfaceFormCdA(planform, edge, 0.01+wave, 1.0, 0.01+wave, ...)
     Induced = drag_cd x S x CL^2 / (pi x 0.8 x AR),  AR = span^2/S = 16/8 = 2
*/
static void test_surface_sweep() {
    printf("== the aero bench: one wing over a grid of alpha / beta / Mach ==\n");
    Bench b = mkBench();
    Part *wing = mkPlate(b.defs, "wing", 20.0, 2.0, 0.05, 1.0);
    setSurface(b.defs.back(), 8.0, true);
    b.v->attachMode(wing, 0, AttachMode::Down);
    Part *elev = mkPlate(b.defs, "elevator", 5.0, 1.0, 0.05, 1.0);
    setSurface(b.defs.back(), 2.0, false);
    /* Both plates hang off the POD along -Z, at the same rotation -- but the tab is
       attached with a 3 m offset, so its chord interval clears the wing's and the two
       stay TWO lifting systems (#556). Stack attachments touch by construction, and
       panes whose chord intervals touch join one system: without the offset this hull
       is ONE system of planform 10 m2 and AR 1.6, and the wing's induced bill drops 20%
       -- pinned for itself in test_lift_system_merge. */
    b.v->attachMode(elev, 0, AttachMode::Down, 0.0, 3.0);
    launch(b);

    /* The tab is its own system with liftArea > 0, and it carries control_area, so its
       angle is the stick alone -- the airframe's own angle of attack never loads it
       (#454's other half). At alpha 0 with a neutral tab there is no lift anywhere. */
    const double speeds[4] = { 50.0, 100.0, 250.0, 500.0 };
    const double alphas[7] = { -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 45.0 };
    const double betas[2] = { 0.0, 15.0 };
    for(double sp : speeds) {
        for(double al : alphas) {
            for(double be : betas) {
                fly(b.v, sp, al, be);
                report(b.v, sp, al, be);
            }
        }
    }

    const double cdMul = b.v->drag_cd;   // --drag-cd: the bill multiplier, 1.2 by default
    /* Both plates are 0.1 m thick: the wing is 4 m span x 2 m chord = 8 m2 with 0.2 m2
       of edge, the tab 2 m x 2 m = 2 m2 with another 0.2 m2. PartDef::meshEdgeArea is
       thickness x chord, and meshThicknessToChord is what wave cd reads -- both plates
       here are t/c 0.05, so they charge the same cd per m2. */
    const double S = 8.0, St = 2.0, edge = 0.4;   // planforms, and both edges together

    /* ---- pins at 100 m/s (Mach 0.30: under the transonic band, so no wave drag and
       the slope correction is the Prandtl-Glauert factor alone). */
    fly(b.v, 100.0, 0.0, 0.0);
    const double rho = b.v->lastDragRho;
    CHECK_TRUE(rho > 0.0, "the bench hull is in air");
    const double q = 0.5 * rho * 100.0 * 100.0;
    /* At zero angle of attack a surface pays ONLY its chordwise profile cd: the
       out-of-plane share and the edge share are both zero. cd x (8 + 2) x 0.01,
       exactly, for the two plates together. */
    CHECK_NEAR(b.v->lastSurfCdA, cdMul * (S + St) * 0.01, 1e-12,
               "alpha 0: a surface's shape bill is its profile cd on its planform");
    CHECK_NEAR(glm::length(b.v->lastLiftForce), 0.0, 1e-12,
               "alpha 0 with a neutral tab: no lift anywhere");
    CHECK_NEAR(b.v->lastInducedCdA, 0.0, 0.0, "no lift, so no induced bill");
    /* The hull column: the pod box is 1 x 1 m seen nose-on and its cd is the authored
       forward anchor. The union and the 3-anchor blend both run here, so this is the
       silhouette column's pin (#463/#478). */
    CHECK_NEAR(b.v->lastDragArea, 1.0 * 1.0, 1e-9, "the pod's silhouette nose-on");
    CHECK_NEAR(b.v->lastDragCd, 0.35, 1e-9, "the pod's cd as it faces the flow");

    fly(b.v, 100.0, 10.0, 0.0);
    const double m100 = 100.0 / b.v->lastAirA;
    const double cl100 = 6.0 / std::sqrt(1.0 - m100 * m100);
    const double a10 = 10.0 * std::numbers::pi / 180.0;
    const double cl10 = cl100 * a10;
    CHECK_NEAR(glm::length(b.v->lastLiftForce), q * S * cl10, q * S * 1e-9,
               "lift = q S CL, CL = PG-corrected slope x alpha");
    CHECK_NEAR(b.v->lastInducedCdA,
               cdMul * S * cl10 * cl10 / (std::numbers::pi * 0.8 * 2.0), 1e-9,
               "the induced bill is the SYSTEM's: CL^2/(pi e AR) at AR = span^2/S");
    /* A lifting surface at incidence pays for its lift twice over: once as inducedCd,
       and again as surfaceFormCdA's out-of-plane share, cd_side sin^2(alpha) on its
       planform. Both go as alpha^2, so neither one is a corner case: at 10 deg this
       hull bills 0.36 m2 of sin^2 share beside 1.84 m2 of induced, and on the shipped
       skiff at its 5 deg trim it bills 0.219 m2 against 0.340 m2 of induced --
       two thirds of it. Nothing says the overlap is wrong (a plate at incidence really
       does both), but the split is a choice, and drag.h's comment on it disclaims only
       an alpha-LINEAR term. Pinned so that folding one into the other is a decision
       someone makes rather than a slip. */
    const double sa = std::sin(a10), ca = std::cos(a10);
    CHECK_NEAR(b.v->lastSurfCdA,
               cdMul * ((S + St) * (0.01 * ca * ca + 1.0 * sa * sa)), 1e-12,
               "the alpha^2 shape share rides alongside the induced bill");
    const double overlap = cdMul * (S + St) * sa * sa;
    const double induced = cdMul * S * cl10 * cl10 / (std::numbers::pi * 0.8 * 2.0);
    CHECK_TRUE(overlap > 0.1 * induced && overlap < induced,
               "the sin^2 share is the SAME ORDER as the induced bill it overlaps");

    /* Sideslip costs a HORIZONTAL wing its EDGE, not its planform (#559): at beta 15
       the out-of-plane share is still zero and only the two plates' edges (thickness x
       chord, 0.3 m2 together) get charged. */
    fly(b.v, 100.0, 0.0, 15.0);
    const double cb = std::cos(15.0 * std::numbers::pi / 180.0);
    CHECK_NEAR(b.v->lastSurfCdA,
               cdMul * ((S + St) * 0.01 * cb * cb + edge * 1.0 * (1.0 - cb * cb)),
               1e-12, "beta charges a plate its EDGE, not its planform (#559)");
    fly(b.v, 100.0, 0.0, 60.0);
    const double cb60 = std::cos(60.0 * std::numbers::pi / 180.0);
    CHECK_NEAR(b.v->lastSurfCdA,
               cdMul * ((S + St) * 0.01 * cb60 * cb60 + edge * 1.0 * (1.0 - cb60 * cb60)),
               1e-12, "and still only its edge at beta 60 -- 8 m2 of wing never appears");

    /* Past the stall angle the curve droops by a cosine, and that is also what turns
       the surface's share of the static margin negative (#487's band). alpha 30 deg
       sits inside 0.35..0.70 rad. */
    fly(b.v, 100.0, 30.0, 0.0);
    const double m30 = 100.0 / b.v->lastAirA;
    const double cl30 = 6.0 / std::sqrt(1.0 - m30 * m30);
    const double a30 = 30.0 * std::numbers::pi / 180.0;
    const double droop = (a30 - 0.35) / 0.35;
    const double clStalled = cl30 * 0.35 * std::cos(droop * std::numbers::pi * 0.5);
    CHECK_NEAR(glm::length(b.v->lastLiftForce), q * S * clStalled, q * S * 1e-9,
               "past stall the slope force droops by a cosine");
    fly(b.v, 100.0, 45.0, 0.0);
    CHECK_NEAR(glm::length(b.v->lastLiftForce), 0.0, 1e-12,
               "past twice the stall angle the wing makes no lift at all");

    /* Mach 1.51 is past the band's high edge: the slope becomes thin-airfoil
       4/sqrt(M^2-1), which is NOT a multiple of the authored 6.0 any more, and the
       section starts paying wave cd on top of its 0.01 profile cd. Read back rather
       than derived, because the point of the bench is that the law's regime edges are
       VISIBLE from here. */
    fly(b.v, 500.0, 5.0, 0.0);
    const double m5 = 500.0 / b.v->lastAirA;
    CHECK_TRUE(m5 > kSupersonicMach, "500 m/s on this body is past the band");
    const double a5 = 5.0 * std::numbers::pi / 180.0;
    const double cl5 = 4.0 / std::sqrt(m5 * m5 - 1.0);
    const double q5 = 0.5 * b.v->lastDragRho * 500.0 * 500.0;
    CHECK_NEAR(glm::length(b.v->lastLiftForce), q5 * S * cl5 * a5, q5 * S * 1e-9,
               "supersonic lift uses the thin-airfoil slope, not the authored 6.0");
    /* Wave cd comes from EACH section's own thickness-to-chord, off its own mesh box --
       both plates here are 0.1 m thick on a 2 m chord, so t/c 0.05 and one cd. Nothing
       to author: being thin is the fix (#543 A5b). */
    const double wave = waveCd(0.1 / 2.0, m5);
    CHECK_TRUE(wave > 0.0, "a supersonic section pays wave cd");
    CHECK_NEAR(b.v->lastSurfCdA,
               cdMul * ((S + St) * ((0.01 + wave) * std::cos(a5) * std::cos(a5)
                                    + 1.0 * std::sin(a5) * std::sin(a5))),
               1e-12, "wave cd joins each plate's own profile cd, by its own t/c");
    /* The hull's blunt-body cd never moves with Mach -- the other half of #635. Read at
       alpha 0, because the 3-anchor blend DOES move with alpha: at alpha 5 the same
       hull reads 0.357, and that is the anchors mixing, not the Mach. */
    fly(b.v, 500.0, 0.0, 0.0);
    CHECK_NEAR(b.v->lastDragCd, 0.35, 1e-12,
               "a blunt body's cd is Mach-blind: the same 0.35 at M 1.5 as at M 0.3");

    /* The stability columns: both surfaces sit aft of the COM, so the airframe is
       stable, and the margin is the slope-weighted mean station behind it. */
    fly(b.v, 100.0, 5.0, 0.0);
    CHECK_TRUE(b.v->lastStaticValid, "the bench quotes a static margin at all");
    CHECK_TRUE(b.v->lastStaticMargin > 0.0,
               "an all-aft airframe has a POSITIVE static margin");
    CHECK_TRUE(b.v->lastLiftSlope > 0.0, "and it is stiff inside the linear range");
    fly(b.v, 100.0, 35.0, 0.0);
    CHECK_TRUE(!b.v->lastStaticValid || b.v->lastLiftSlope < 40.0,
               "inside the droop the stiffness collapses, it does not hold");
    teardown(b);
}

/* THE LIFTING-SYSTEM MERGE, and what it does to the induced bill (#556/#562). Two
   near-identical hulls -- same pod, same 8 m2 wing, same 2 m2 tab, same 0.1 m plates.
   The merged one hangs the tab off the wing's trailing edge, where stack attachments
   touch by construction, so the two panes form ONE lifting system of planform 10 m2
   and AR 1.6. The split one stacks the tab on the pod with a 3 m offset, which clears
   the intervals: TWO systems, an 8 m2 wing at AR 2 and a 2 m2 tab at AR 2. The tab is
   neutral in both, so it makes no lift in either, and yet the wing's induced bill is
   20% cheaper for having a tab hung off it: CL_sys divides the wing's lift by the
   MERGED planform and AR_sys uses it too. Pinned as the law now is, because whether
   that is right is a #562 question, and changing the answer should move a test. */
static void test_lift_system_merge() {
    printf("== one tab, merged with the wing versus standing alone ==\n");
    Bench merged = mkBench();
    Part *mw = mkPlate(merged.defs, "wing", 20.0, 2.0, 0.05, 1.0);
    setSurface(merged.defs.back(), 8.0, true);
    merged.v->attachMode(mw, 0, AttachMode::Down);
    Part *mt = mkPlate(merged.defs, "elevator", 5.0, 1.0, 0.05, 1.0);
    setSurface(merged.defs.back(), 2.0, false);
    merged.v->attachMode(mt, 1, AttachMode::Down);          // off the wing: intervals touch
    launch(merged);

    Bench split = mkBench();
    Part *sw = mkPlate(split.defs, "wing", 20.0, 2.0, 0.05, 1.0);
    setSurface(split.defs.back(), 8.0, true);
    split.v->attachMode(sw, 0, AttachMode::Down);
    Part *st = mkPlate(split.defs, "elevator", 5.0, 1.0, 0.05, 1.0);
    setSurface(split.defs.back(), 2.0, false);
    split.v->attachMode(st, 0, AttachMode::Down, 0.0, 3.0);  // 3 m aft: two systems
    launch(split);

    fly(merged.v, 100.0, 10.0, 0.0);
    const double aInd = merged.v->lastInducedCdA;
    fly(split.v, 100.0, 10.0, 0.0);
    const double bInd = split.v->lastInducedCdA;
    printf("[merge] wing + tab as ONE system  Induced %8.4f m2\n", aInd);
    printf("[merge] wing + tab as TWO systems Induced %8.4f m2\n", bInd);
    const double m100 = 100.0 / merged.v->lastAirA;
    const double cl10 = 6.0 / std::sqrt(1.0 - m100 * m100) * (10.0 * std::numbers::pi / 180.0);
    const double clSys = 8.0 * cl10 / 10.0;   // the wing's lift over the merged planform
    const double cdi = clSys * clSys / (std::numbers::pi * 0.8 * (16.0 / 10.0));
    CHECK_NEAR(aInd, merged.v->drag_cd * 8.0 * cdi, 1e-9,
               "a merged system bills the wing CL_sys^2/(pi e AR) at the MERGED planform");
    CHECK_TRUE(aInd < 0.85 * bInd,
               "a neutral tab under a wing makes its induced bill 15%+ cheaper (#562)");
    teardown(merged);
    teardown(split);
}

/* THE INDUCED SUM IS OVER MAGNITUDES. #494 closed by billing a trimmed surface's
   download into the system's induced drag, and the way it went in is a sum of |F|:
   vehicle.h's AeroLiftSample::lift is glm::length(flift), applyInducedDrag adds those
   and squares the total. Squaring a sum of magnitudes has no room for a surface that
   pushes DOWN, so the tab's 12 deg buys the same inflated bill whether it is lifting the
   airframe or unloading it -- and the unloading case is the one a nose-heavy trim is in:
   LESS net lift through the same speed, MORE induced drag than the same airframe with
   the tab neutral. Whether CL_sys should be signed, per-sheet, or this is a decision
   nobody has written down, so the shape of the current answer is pinned here and a
   change to it should move these pins. */
static void test_induced_sum_is_sign_blind() {
    printf("== the same tab lifting, neutral, and downloading ==\n");
    Bench b = mkBench();
    Part *w = mkPlate(b.defs, "wing", 20.0, 2.0, 0.05, 1.0);
    setSurface(b.defs.back(), 8.0, true);
    b.v->attachMode(w, 0, AttachMode::Down);
    Part *t = mkPlate(b.defs, "elevator", 5.0, 1.0, 0.05, 1.0);
    setSurface(b.defs.back(), 2.0, false);
    b.v->attachMode(t, 1, AttachMode::Down);          // off the wing: ONE system
    launch(b);

    const double trim12 = glm::degrees(0.21);         // what res/ships/skiff_trim.json authors
    auto shot = [&](double trim) {
        t->trimDeg = trim;
        fly(b.v, 100.0, 10.0, 0.0);
        return std::make_pair(b.v->lastInducedCdA, glm::length(b.v->lastLiftForce));
    };
    const auto neu = shot(0.0);
    const auto up = shot(trim12);
    const auto dn = shot(-trim12);
    printf("[sigblind] tab neutral   Induced %8.4f m2   net L %10.2f N\n"
           "[sigblind] tab +12 deg   Induced %8.4f m2   net L %10.2f N\n"
           "[sigblind] tab -12 deg   Induced %8.4f m2   net L %10.2f N\n",
           neu.first, neu.second, up.first, up.second, dn.first, dn.second);
    /* Which authored sign lifts is the MOUNT's business (drag.h::controlDeflectionSign
       flips it per rig), so name the two shots by what they do to the airframe. */
    const double lLoad = std::max(up.second, dn.second);
    const double lUnload = std::min(up.second, dn.second);
    printf("[sigblind] the tab's own force %9.2f N; the bill goes %+.1f%% from neutral, "
           "identically in both directions\n", 0.5 * (lLoad - lUnload),
           100.0 * (dn.first / neu.first - 1.0));
    /* Mirror-image deflections, bit-identical bill: the law cannot tell a download from
       an upload, because the sum it squares took the absolute value first. */
    CHECK_NEAR(up.first, dn.first, 0.0, "the induced bill is blind to the tab's sign");
    CHECK_TRUE(lUnload < neu.second && neu.second < lLoad,
               "and yet one sign loads the airframe and the other unloads it");
    /* THE PARADOX, pinned: unloading the airframe costs it induced drag. */
    CHECK_TRUE(lUnload < 0.75 * neu.second,
               "a downloading tab takes a quarter of the airframe's lift off");
    CHECK_TRUE(dn.first > 2.0 * neu.first,
               "and more than DOUBLES its induced bill doing it (#494's cross term)");
    teardown(b);
}

/* THE STICK CURVE. A pod and one elevator, flown at alpha 0 with a standing trim, so
   the only lift anywhere is the tab's deflection force and lastLiftForce IS its
   magnitude. Swept over the whole travel, the force is NOT monotone: it peaks at
   stall_angle and is gone by twice that, and the shipped travel stop (0.35 rad) sits
   PAST the peak (0.26 rad). What a player pays for that is in the pins below. */
static void test_surface_authority_curve() {
    printf("== a control surface's authority over its own travel ==\n");
    Bench b = mkBench();
    Part *elev = mkPlate(b.defs, "elevator", 5.0, 1.0, 0.05, 1.0);
    setSurface(b.defs.back(), 2.0, false);
    b.v->attachMode(elev, 0, AttachMode::Down);
    launch(b);

    fly(b.v, 100.0, 0.0, 0.0);
    const double q = 0.5 * b.v->lastDragRho * 100.0 * 100.0;
    const double slope = liftSlopeMach(6.0, 100.0 / b.v->lastAirA);
    const double St = 2.0;

    printf("[curve] travel stop %6.2f deg, stall angle %6.2f deg, q %8.1f Pa\n",
           glm::degrees(0.35), glm::degrees(0.26), q);
    const int kN = 36;
    for(int i = 0; i <= kN; i++) {
        const double frac = -1.0 + 2.0 * i / kN;
        elev->trimDeg = glm::degrees(frac * 0.35);
        fly(b.v, 100.0, 0.0, 0.0);
        const double f = glm::length(b.v->lastLiftForce);
        const double d = std::fabs(elev->trimDeg) * std::numbers::pi / 180.0;
        CHECK_NEAR(f, q * St * liftCurve(slope, d, 0.26), q * St * 1e-9,
                   "the tab's force is the lift law read on its commanded deflection");
        printf("[curve] defl %+7.2f deg  F %9.2f N\n", elev->trimDeg, f);
    }

    /* Three named points on that curve. The rig sign flips the deflection's sign for
       a mount, not its magnitude, so force magnitudes are what this curve means. */
    auto tabForce = [&](double deflRad) {
        elev->trimDeg = glm::degrees(deflRad);
        fly(b.v, 100.0, 0.0, 0.0);
        return glm::length(b.v->lastLiftForce);
    };
    const double fPeak = tabForce(0.26);
    const double fStop = tabForce(0.35);
    const double fTrim12 = tabForce(glm::radians(12.0));
    /* THE FINDING, pinned: full travel is not full authority. The stop sits past the
       peak, so the last 26% of the stick's throw pulls 14% WEAKER, and a surface
       trimmed at 12 deg -- which is what res/ships/skiff_trim.json authors -- has 6%
       more authority available at the stop. Both halves are the surface's own Cl: the
       bench cannot price what that costs a flying airframe, and no in-game readout
       shows surface lift at all (#655), so the [ctrl] row's deflection columns are the
       only place a pilot or a case can see the difference. */
    CHECK_TRUE(fStop < fPeak, "the travel stop sits PAST peak authority (#503's droop)");
    CHECK_NEAR(fStop / fPeak, 0.8557, 1e-3,
               "at the stop the surface makes 85.6% of its own peak force");
    CHECK_NEAR(fStop / fTrim12, 1.0625, 1e-3,
               "from a 12 deg trim, full travel buys 6.2% more force");

    /* The hinge is invisible to the SHAPE bill as well (#506's other half, and the
       bigger half: the hull-union side is what that issue names). The surface form-drag
       pass reads the flow against the plate's AUTHORED axes -- vehicle.cpp's flowLocal
       against plateNormal -- never against the deflection, so a tab hard over at 20 deg
       still meets the flow chordwise as far as its bill is concerned. Pinned as the law
       now IS, loudly: 15 kN of tab force, and the same 0.024 m2 of shape drag it pays
       sitting neutral. A tab turned 20 deg broadside is turning 2 x sin^2(20) = 0.24 m2
       of plate, which at this q is another ~1.5 kN the airframe never pays. */
    elev->trimDeg = 0.0;
    fly(b.v, 100.0, 0.0, 0.0);
    const double profNeutral = b.v->lastSurfCdA;
    CHECK_NEAR(profNeutral, b.v->drag_cd * St * 0.01, 1e-12,
               "a neutral tab's shape bill is its profile cd on its planform");
    elev->trimDeg = glm::degrees(0.35);
    fly(b.v, 100.0, 0.0, 0.0);
    printf("[curve] tab shape bill: %8.4f m2 neutral, %8.4f m2 hard over, while its "
           "force goes 0 -> %9.2f N\n", profNeutral, b.v->lastSurfCdA, fStop);
    CHECK_NEAR(b.v->lastSurfCdA, profNeutral, 1e-12,
               "a tab hard over bills the SAME shape drag as a neutral one (#506)");

    /* Trim cannot reach the reversal region: controlCommandFrac clamps the command to
       one full travel, so an over-large authored trim saturates at the stop rather
       than driving the tab past twice its stall angle, where liftCurve reverses and
       the stick would run backwards. (The loader also refuses max_deflection >= 2x
       stall_angle for a NEW def -- shipdef.cpp.) */
    CHECK_NEAR(tabForce(glm::degrees(0.55)), fStop, 1e-9,
               "trim past the travel stop saturates at the stop, it does not reverse");
    teardown(b);
}

/* THE ENERGY HOOKS A HEATING MODEL CAN READ (#1090 grew the second one). #553 (no
   reentry heat), #554 (no structural limits / max Q) and #728 (heat as a mechanic,
   parked) are all open, and there is no temperature or damage state anywhere in the
   codebase -- so the question is what the smallest credible version would read.
   There are now two per-part figures the sim keeps every substep:
     - A SURFACE's number is exact, per part, per substep: q and v and CdA are already
       per-surface since #483/#730, so a heat rating could be authored per part today.
       The pin below is the arithmetic itself -- watts = drag_cd x q x CdA x v.
     - A BODY pays an ACCOUNTED share, not a local bill: the hull's form drag is
       billed ONCE on the union at one cp (#463/#514), and #1090 splits that bill by
       the #945 partition -- Part::formVis/formCdA/formDragWatts, refreshed every
       billed substep, chart HITS included. A reentering capsule -- all hull, no
       surfaces -- reports watts through that trio now rather than the 0 W this test
       used to pin as the hole. What the trio does NOT do is turn power into heat:
       the share is an accounting of what the air takes, and depositing some of it in
       a material is still #553's open law. The pins below lock the ACCOUNTING: the
       credits sum to exactly the bill the force pays on.
   The v^3 row is the sizing number. 20 km up on this body (rho = 1.225 e^-2.5 = 0.109),
   the 8 m2 wing's watts run 0.20 MW at 300 m/s, 7.07 MW at 1000 m/s and 48.9 MW at
   2000 m/s -- 25 kW/m2, 0.88 MW/m2 and 6.1 MW/m2. The whole ship drags 2.9 kN, 31.7 kN
   and 122 kN there, and both halves of that power are now booked somewhere: the
   surfaces locally, the hull by credit. Whatever a rating turns out to be, that is the
   order of number it has to answer. */
static void test_heat_hooks() {
    printf("== the energy hook a heating model would read, and what it misses ==\n");
    Bench b = mkBench();
    Part *wing = mkPlate(b.defs, "wing", 20.0, 2.0, 0.05, 1.0);
    setSurface(b.defs.back(), 8.0, true);
    b.v->attachMode(wing, 0, AttachMode::Down);
    Part *elev = mkPlate(b.defs, "elevator", 5.0, 1.0, 0.05, 1.0);
    setSurface(b.defs.back(), 2.0, false);
    b.v->attachMode(elev, 0, AttachMode::Down, 0.0, 3.0);
    launch(b);

    const double speeds[3] = { 300.0, 1000.0, 2000.0 };
    double prevW = -1.0;
    for(double sp : speeds) {
        /* 20 km up on this body: rho = 1.225 x e^-2.5. placeShip again because the
           column is a profile, not one flight. */
        b.v->placeShip(glm::dvec3(0.0, 0.0, 620000.0), glm::dmat3(1.0));
        /* Five substeps at the SAME flow direction: kSilFillNodes=1 spreads the
           cell's four corner fills over substeps 1..4 -- each of which looks the
           chart up BEFORE filling, so it bills its own exact pass -- and substep
           5 HITS. The hull pins below therefore run through the blended columns
           and the Part-copy path, not just the exact one (#1090's other half;
           test_drag's memo pins stop at the blend itself). */
        for(int shot = 0; shot < 5; shot++) { fly(b.v, sp, 0.0, 0.0); }
        const double q = 0.5 * b.v->lastDragRho * sp * sp;
        printf("[heat] v %7.1f  rho %8.5f  Mach %5.2f  q %11.1f Pa  "
               "hull CdA %6.3f m2  ship drag %11.1f N  -> W %11.1f  |  "
               "wing W %11.1f  tab W %10.1f  pod formW %10.1f  "
               "(per m2 of wing %10.1f W/m2)\n",
               sp, b.v->lastDragRho, sp / b.v->lastAirA, q,
               b.v->lastDragArea, b.v->lastFlowDrag, b.v->lastFlowDrag * sp,
               (double)b.v->parts[1]->aeroDragWatts, (double)b.v->parts[2]->aeroDragWatts,
               (double)b.v->parts[0]->formDragWatts,
               (double)b.v->parts[1]->aeroDragWatts / 8.0);
        /* The surface hook, exactly: drag_cd x q x (its own chordwise cd) x v. At
           alpha 0 a surface's whole bill is its chordwise cd -- its profile cd plus,
           past the sound bar, its own wave cd (#543 A5b), and induced is zero. */
        const double wv = waveCd(0.1 / 2.0, sp / b.v->lastAirA);
        CHECK_NEAR((double)b.v->parts[1]->aeroDragWatts,
                   b.v->drag_cd * q * ((0.01 + wv) * 8.0) * sp,
                   b.v->drag_cd * q * 0.08 * sp * 1e-6,
                   "a surface's watts are its own q x CdA x v, ready to integrate");
        /* ... and the hull's share, which used to be the HOLE this test existed to
           show: the ship's drag power (the aero force along the flow times v) is
           0.87 MW at 300 m/s, and the two surfaces booked only 0.25 MW of it while
           the pod -- all the hull -- kept nothing. #1090 closed the books: the
           partition columns ride onto the Part every billed substep, so here the
           pod (the whole hull: the union pass skips the two SURFACE plates) is
           credited the entire silhouette, and its watts are the hull's own bill --
           drag_cd x q x cdNum x v, at the cp's flow (the bench flies unrotating,
           so that flow is the COM's). The fields are float; the tolerance is
           float's, not the law's. */
        CHECK_TRUE(b.v->lastFlowDrag > 0.0, "the ship is definitely paying drag");
        CHECK_NEAR((double)b.v->parts[0]->formVis, b.v->lastDragArea,
                   b.v->lastDragArea * 1e-5,
                   "the pod is all the hull: its credit IS the whole silhouette");
        double sumCdA = 0.0, sumW = 0.0;
        for(Part *p : b.v->parts) { sumCdA += p->formCdA; sumW += p->formDragWatts; }
        CHECK_NEAR(sumCdA, b.v->lastDragCd * b.v->lastDragArea,
                   b.v->lastDragCd * b.v->lastDragArea * 1e-5,
                   "the credits sum to the cd numerator the force pays on (#1090)");
        CHECK_NEAR(sumW, b.v->drag_cd * q * b.v->lastDragCd * b.v->lastDragArea * sp,
                   b.v->drag_cd * q * b.v->lastDragCd * b.v->lastDragArea * sp * 1e-5,
                   "the hull's ALONG-FLOW drag power is fully booked by credit now "
                   "(#1090; this bench hull is exposed, so #947's normal watts are 0 here)");
        CHECK_NEAR(b.v->lastFormWatts, sumW, std::fabs(sumW) * 1e-5 + 1e-9,
                   "#1114: the body-card hold's criterion (the hull's whole bill) IS the "
                   "partition sum -- the product and the columns cannot drift apart");
        CHECK_NEAR((double)b.v->parts[0]->aeroDragWatts, 0.0, 0.0,
                   "a body still folds no SURFACE watts: the hull share is the form trio, "
                   "so #730's sum-over-surfaces reading keeps its meaning");
        if(prevW > 0.0) {
            CHECK_TRUE((double)b.v->parts[1]->aeroDragWatts > 3.0 * prevW,
                       "the hook's number climbs faster than v^2, so a rating must be "
                       "a flux not a force");
        }
        prevW = (double)b.v->parts[1]->aeroDragWatts;
    }
    /* #947's anti-double-count, pinned away from the kernel excuse: at 30 deg
       ANGLE OF ATTACK a body's side strip is genuinely nonzero, so a zero here can
       only come from the discount -- the pod's credit equals its own shadow at
       every node to within the sum order (shoelace vs the union's strip sum,
       ~1e-16 of area), refused falls under the 1e-12 floor, and the crossflow law
       bills the fully exposed hull NOTHING: its flank is already paid once,
       axially. The fifth shot HITS, so this runs through the blended shadow/credit
       columns, where a lying column would show up. If this pin ever reddens,
       either the normal law and the union bill the same flank twice, or the #947
       shadow column is not the same polygon world as the credits. */
    b.v->placeShip(glm::dvec3(0.0, 0.0, 620000.0), glm::dmat3(1.0));
    for(int shot = 0; shot < 5; shot++) { fly(b.v, 800.0, 30.0, 0.0); }
    CHECK_TRUE(b.v->lastDragAlphaValid, "30 deg is a real angle for the pass");
    CHECK_NEAR(b.v->lastSideCdA, 0.0, 0.0,
               "a fully exposed hull pays no crossflow normal force, blended or not");
    teardown(b);
}

/* #1097: BODY LIFT -- the opt-in normal force on a body part's EXPOSED silhouette,
   and the partition that keeps it disjoint from #947. The fixture stacks three body
   parts along the stack axis, all parallel:

       pod   (root, 0.5 x 0.5 x 1.0, body_lift 0.6)  center z =  0.0
       shelter (2.0 x 2.5 x 0.2,    body_lift 0.6)   center z = -1.2
       tail  (0.1 x 0.1 x 1.5,      body_lift 0.6, drag_side 1) center z = -2.9

   At alpha 30 the travel direction is cos30 z - sin30 y, so shelter stands between
   the flow and tail, and tail's small shadow falls INSIDE shelter's (margin ~0.6 m
   at every chart node near 30 deg): tail bills no lift (visible = 0, the #1090
   column) and full side (refused = 1, the #947 block) -- the two laws' DISJOINT
   partition, pinned by counting recorded force vectors by kind at each mount. The
   three axes are parallel, so cos(a) factors out and the bill closes by #1090's
   identity sum(visible) == union:

       BodyCdA = drag_cd x 0.6 x cos(30) x lastDragArea -- EXACT even on memo HITS,
       because both sides ride the same columns.

   The recorded arms sit at the hull body's ORIGIN (Bullet keeps a rigid body's
   COM at its transform origin, and the root pod center is the bench's origin):
   pod 0, shelter -1.2, tail -2.9 -- the tail is both the ONLY side vector and
   the aftmost vector on the hull. And lastForceVecs accumulates across
   applyAeroForce calls (the tick loop clears it per substep, the applier does
   not), so each recording shot clears it by hand first.
   Since #1106 the arms sit at each part's PROJECTED CENTROID, not the bolt --
   but every box here is mount-centred, so its cp equals its mount EXACTLY and
   these pins read the same numbers as before (and stay blind to the arm;
   test_hull_lift_arm_is_the_projected_centroid is the one that is not). */
static void test_body_lift() {
    printf("== #1097: body lift -- sin cos kernel, opt-in, disjoint with #947 ==\n");
    Bench b = mkBench();
    launch(b);
    fly(b.v, 800.0, 30.0, 0.0);
    CHECK_NEAR(b.v->lastBodyCdA, 0.0, 0.0,
               "a hull with no body_lift authored lifts nothing (every flight-pinned "
               "hull; liftbody_4m is the catalog's opt-in)");
    CHECK_NEAR(b.v->lastBodyCpLever, 0.0, 0.0,
               "...and quotes no lever either");

    b.defs.front().body_lift = 0.6;   // the pod def (mkBench filled its drag anchors)
    Part *shelter = mkPlate(b.defs, "shelter", 200.0, 2.0, 2.5, 0.2);
    b.defs.back().body_lift = 0.6;
    b.v->attachMode(shelter, 0, AttachMode::Down);
    Part *tail = mkPlate(b.defs, "tail", 50.0, 0.1, 0.1, 1.5);
    b.defs.back().body_lift = 0.6;
    b.defs.back().drag_side = 1.0;    // #947 bills cn x strip: no anchor, no vector
    b.v->attachMode(tail, 1, AttachMode::Down);

    b.v->recordForceVecs = true;
    for(int shot = 0; shot < 3; shot++) { fly(b.v, 800.0, 30.0, 0.0); }
    b.v->lastForceVecs.clear();   // the counted shot: fresh vectors only
    fly(b.v, 800.0, 30.0, 0.0);
    CHECK_TRUE(b.v->lastDragAlphaValid, "30 deg is a real angle for the pass");
    const double a = 30.0 * std::numbers::pi / 180.0;
    CHECK_NEAR(b.v->lastBodyCdA,
               b.v->drag_cd * 0.6 * std::cos(a) * std::sin(a) * b.v->lastDragArea,
               b.v->drag_cd * 0.6 * b.v->lastDragArea * 1e-6,
               "BodyCdA = drag_cd x cl x cos x sin x union: the column bills the "
               "MAGNITUDE line -- the sin is folded in, like #947's strip -- so "
               "q x BodyCdA is the summed force's size; cos and sin factor out "
               "(parallel axes) and the visible column sums to the union (#1090), "
               "HITS included");
    CHECK_NEAR(b.v->lastSideCdA, b.v->drag_cd * 1.0 * 0.3, b.v->drag_cd * 1e-6,
               "only the hidden tail bills crossflow: refused 1 x strip 0.3 m2");
    // The lever is the FORCE-weighted centroid arm (a bare-coefficient weighting
    // would be the arm of nothing once parts see different flows or axes). Same
    // cl/cos/sin on both lifting mounts here, so the weights reduce to the
    // cached visible areas: visPod x 0 + visShelter x (-1.2) over their sum.
    const double visPod = (double)b.v->parts[0]->formVis;
    const double visShel = (double)b.v->parts[1]->formVis;
    CHECK_TRUE(visPod > 0.0 && visShel > 0.0,
               "pod fully seen, shelter partly -- the middle of the partition");
    CHECK_NEAR(b.v->lastBodyCpLever, -1.2 * visShel / (visPod + visShel), 1e-5,
               "BodyCp = force-weighted arm of the two lifting mounts; the hidden "
               "tail bills no lift and cannot pull the lever");

    /* The kind-count partition, at the mounts: TWO body-lift vectors (pod, shelter),
       ZERO at tail; ONE side vector and it is the aftmost. */
    int nlift = 0, nside = 0;
    double liftRz[2] = { 0.0, 0.0 };
    glm::dvec3 liftSum(0.0);
    for(const Vehicle::ForceVec &fv : b.v->lastForceVecs) {
        if(fv.kind == Vehicle::kForceBodyLift) {
            nlift++;
            if(nlift <= 2) { liftRz[nlift - 1] = fv.pos.z; }
            liftSum += fv.vec;
        } else if(fv.kind == Vehicle::kForceSide) {
            nside++;
            CHECK_NEAR(fv.pos.z, -2.9, 1e-5,
                       "the only crossflow vector sits at the hidden tail's mount");
        }
    }
    CHECK_TRUE(nlift == 2, "body lift at the two exposed mounts, not at the hidden one");
    CHECK_TRUE(nside == 1, "crossflow normal at the hidden mount only: #947 acts "
                           "where the union refused, and on this hull that is the "
                           "tail alone (complementary gates -- see the middle case "
                           "below for both laws on one part)");
    CHECK_NEAR(liftRz[0], 0.0, 1e-5, "pod mount arm (the hull origin)");
    CHECK_NEAR(liftRz[1], -1.2, 1e-5, "shelter mount arm");

    /* Direction and magnitude in one: the two vectors share one ahat (parallel
       axes, same local flow at a non-rotating bench), so their sum must be the
       UNIT ahat x (q x BodyCdA) -- leeward, up-ish at belly-into-flow. flowHat
       is fly()'s velocity direction itself: production's uhat is vp/|vp|, the
       same vector. */
    const glm::dvec3 flowHat = flowDirection(a, 0.0, glm::dvec3(1, 0, 0),
                                             glm::dvec3(0, 1, 0), glm::dvec3(0, 0, 1));
    const glm::dvec3 ahat = glm::dvec3(0.0, 0.0, 1.0) - flowHat * std::cos(a);
    CHECK_NEAR(glm::length(ahat), std::sin(a), 1e-12, "sin lives in ahat");
    const double q = 0.5 * b.v->lastDragRho * 800.0 * 800.0;
    // The column already carries the sin, so the force rides the UNIT direction.
    const glm::dvec3 want = (ahat / std::sin(a)) * (q * b.v->lastBodyCdA);
    CHECK_NEAR(glm::length(liftSum - want), 0.0, q * b.v->lastBodyCdA * 1e-6,
               "sum(body lift vectors) = (ahat/sin) x q x BodyCdA: perpendicular to "
               "the FLOW (not #947's perp), leeward, and |sum| = q x BodyCdA -- the "
               "column is a force meter, like SideCdA");
    CHECK_NEAR(glm::dot(liftSum, flowHat), 0.0, q * b.v->lastBodyCdA * 1e-9,
               "perpendicular to the flow by construction: it does no work at a "
               "non-rotating bench");
    CHECK_TRUE(b.v->lastFlowLift > 0.0,
               "the hull now carries a perpendicular force with no wings aboard "
               "(Ld= picks it up automatically)");

    /* Axial travel: the kernel, not a guard, zeroes BOTH the force and the
       column. ahat = z - z x 1 is EXACTLY zero, and the folded sin makes the
       column read 0.00 there too -- which is the whole point of folding it in:
       the old column read a full bill beside a force of exactly nothing. */
    b.v->lastForceVecs.clear();
    fly(b.v, 800.0, 0.0, 0.0);
    CHECK_NEAR(b.v->lastBodyCdA, 0.0, 0.0,
               "axial: q x BodyCdA sizes the force at 0, honestly, because the "
               "force IS 0 there");
    double axialLift = 0.0;
    for(const Vehicle::ForceVec &fv : b.v->lastForceVecs) {
        if(fv.kind == Vehicle::kForceBodyLift) { axialLift += glm::length(fv.vec); }
    }
    CHECK_NEAR(axialLift, 0.0, 0.0, "zero axial force exactly: the sin lives in ahat");

    /* THE MIDDLE CASE (#1097 review): complementary GATES are not one shared
       area measure -- the #947 strip and the #1097 silhouette are different
       numbers for the same box -- so a HALF-SHIELDED part with both anchors
       authored takes both normal forces at once. That sum is a decision, so
       pin it: author the shelter's drag_side and re-shoot 30 deg. The shot
       now records a side vector AT THE SHELTER MOUNT too (its refused share,
       beside the tail's whole refused shadow), while the two lift vectors
       stand untouched. Neither law hands material to the other. */
    b.defs[1].drag_side = 1.0;   // the shelter def
    b.v->lastForceVecs.clear();
    fly(b.v, 800.0, 30.0, 0.0);
    int nB_lift = 0, nB_side = 0, shelterSide = 0;
    for(const Vehicle::ForceVec &fv : b.v->lastForceVecs) {
        if(fv.kind == Vehicle::kForceBodyLift) { nB_lift++; }
        else if(fv.kind == Vehicle::kForceSide) {
            nB_side++;
            if(std::fabs(fv.pos.z + 1.2) < 1e-5 && glm::length(fv.vec) > 0.0) {
                shelterSide++;
            }
        }
    }
    CHECK_TRUE(nB_side == 2, "refused billing now sits at the tail AND the shelter");
    CHECK_TRUE(shelterSide == 1,
               "the half-shielded shelter bills crossflow ON TOP of its body lift: "
               "the partition gates the fractions, it does not make the two laws "
               "take turns over one part (different area measures, drag.h)");
    CHECK_TRUE(nB_lift == 2, "and its body lift stands: 30 deg visible share lifted");
    CHECK_TRUE(b.v->lastSideCdA > b.v->drag_cd * 0.3,
               "SideCdA grew past the tail-only 0.3 m2 by the shelter's refused "
               "share of its own strip");

    /* Signed anchor: flipping body_lift negates the bill and swings the push to
       the windward side. (Mounting the part INVERTED instead -- flipping the
       axis, not the anchor -- must leave the force alone: cosA and ahat flip
       together. Same shot, negated anchors:) */
    b.defs.front().body_lift = -0.6;
    b.defs[1].body_lift = -0.6;   // the shelter def (Part::def is const; the deque
                                  // that owns it is not, and refs into it are stable)
    b.v->lastForceVecs.clear();
    fly(b.v, 800.0, 30.0, 0.0);
    CHECK_NEAR(b.v->lastBodyCdA,
               -b.v->drag_cd * 0.6 * std::cos(a) * std::sin(a) * b.v->lastDragArea,
               b.v->drag_cd * 0.6 * b.v->lastDragArea * 1e-6,
               "cl is signed: the bill negates");
    glm::dvec3 flipSum(0.0);
    for(const Vehicle::ForceVec &fv : b.v->lastForceVecs) {
        if(fv.kind == Vehicle::kForceBodyLift) { flipSum += fv.vec; }
    }
    CHECK_NEAR(glm::length(flipSum + liftSum), 0.0,
               q * std::fabs(b.v->lastBodyCdA) * 1e-6,
               "and the force negates: the same ahat, the opposite side");
    b.v->recordForceVecs = false;
    teardown(b);
}

/* MOUNT-FLIP INVARIANCE, on its own bench (the review asked: the law's headline
   safety claim was tested only by flipping cl -- a different mechanism).
   Mounting a part INVERTED flips its +z to -z: cosA flips and ahat flips with
   it, so the FORCE is untouched; only the signed COLUMN turns with cosA, an
   inverted shelter's bill cancelling part of the pod's. Same script twice, the
   second hanging the identical box from the raw attach pose with pi of roll
   about x: same arm, same silhouette, opposite axis. */
static void test_body_lift_mount_flip_invariance() {
    printf("== #1097: an inverted mount flips cosA and ahat together, not the force ==\n");
    auto shot = [](Bench &b, glm::dvec3 *shelterLift, double *bcda,
                   double *visPod, double *visShel) {
        b.v->recordForceVecs = true;
        for(int s = 0; s < 3; s++) { fly(b.v, 800.0, 30.0, 0.0); }
        b.v->lastForceVecs.clear();
        fly(b.v, 800.0, 30.0, 0.0);
        *shelterLift = glm::dvec3(0.0);
        int at = 0;
        for(const Vehicle::ForceVec &fv : b.v->lastForceVecs) {
            if(fv.kind == Vehicle::kForceBodyLift
               && std::fabs(fv.pos.z + 1.2) < 1e-5) {
                *shelterLift = fv.vec;
                at++;
            }
        }
        CHECK_TRUE(at == 1, "exactly one body-lift vector at the shelter mount");
        *bcda = b.v->lastBodyCdA;
        *visPod = (double)b.v->parts[0]->formVis;
        *visShel = (double)b.v->parts[1]->formVis;
        b.v->recordForceVecs = false;
    };
    const double a = 30.0 * std::numbers::pi / 180.0;
    Bench up = mkBench();
    launch(up);
    Part *shUp = mkPlate(up.defs, "shelter", 200.0, 2.0, 2.5, 0.2);
    up.defs.front().body_lift = 0.6;
    up.defs.back().body_lift = 0.6;
    up.v->attachMode(shUp, 0, AttachMode::Down);
    glm::dvec3 liftUp; double bcdaUp, visPodUp, visShelUp;
    shot(up, &liftUp, &bcdaUp, &visPodUp, &visShelUp);

    Bench inv = mkBench();
    launch(inv);
    Part *shInv = mkPlate(inv.defs, "shelter", 200.0, 2.0, 2.5, 0.2);
    inv.defs.front().body_lift = 0.6;
    inv.defs.back().body_lift = 0.6;
    // The raw pose, not the node solve: same -1.2 m arm, +z flipped (pi about x).
    inv.v->attach(shInv, 0, glm::dvec3(0.0, 0.0, -1.2),
                  glm::dmat3(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0));
    glm::dvec3 liftInv; double bcdaInv, visPodInv, visShelInv;
    shot(inv, &liftInv, &bcdaInv, &visPodInv, &visShelInv);

    CHECK_NEAR(visShelInv, visShelUp, 1e-4, "same symmetric box, same visible column");
    CHECK_NEAR(glm::length(liftInv - liftUp), 0.0, glm::length(liftUp) * 1e-9,
               "the shelter's lift force is UNCHANGED by the inverted mount: cosA "
               "and ahat flip together; only cl's sign moves the force (drag.h)");
    const double scale = up.v->drag_cd * 0.6;
    CHECK_NEAR(bcdaUp, scale * std::cos(a) * std::sin(a) * (visPodUp + visShelUp),
               scale * 1e-5, "upright: both bills add");
    CHECK_NEAR(bcdaInv, scale * std::cos(a) * std::sin(a) * (visPodInv - visShelInv),
               scale * 1e-5,
               "inverted: the COLUMN is axis-sign-aware even though the force is not "
               "-- the signed sum is a bill, not a vector meter (vehicle.h)");
    teardown(up);
    teardown(inv);
}

/* Body parts must not join the SURFACE machinery: no aeroLiftSamples, so
   Induced= cannot move, and no part carries a #730 aeroQ stamp. CLa used to
   be pinned out too; #1106 reversed THAT half on purpose -- the hull's own
   d(force)/d(alpha) now folds into the pair -- and the pin below names
   exactly how much, off the same frame's BodyCdA bill. */
static void test_body_lift_stays_out_of_the_surface_systems() {
    printf("== #1097/#1106: body lift moves no Induced; its slope joins CLa ==\n");
    Bench b = mkBench();
    Part *wing = mkPlate(b.defs, "wing", 20.0, 2.0, 0.05, 1.0);
    setSurface(b.defs.back(), 8.0, true);
    b.v->attachMode(wing, 0, AttachMode::Down);
    launch(b);
    fly(b.v, 300.0, 10.0, 0.0);
    CHECK_TRUE(b.v->lastInducedCdA > 0.0, "the wing is loaded and billing induced");
    const double ind0 = b.v->lastInducedCdA, cla0 = b.v->lastLiftSlope;
    b.defs.front().body_lift = 0.6;
    fly(b.v, 300.0, 10.0, 0.0);
    CHECK_TRUE(b.v->lastBodyCdA > 0.0, "and the pod is now lifting too");
    CHECK_NEAR(b.v->lastInducedCdA, ind0, ind0 * 1e-12,
               "Induced= is bit-identical: body parts never join a lifting system");
    /* #1113 grew the form* trio into a quintet -- the BILL's flow (lfCp at the cp)
       stamped beside the credits so the part window's rows reconcile -- and with a
       printed q now living on body parts too, the naming contract that comment
       above promised becomes a CHECK: the #730 aeroQ/aeroValid stamp is
       surface-only (tick.cpp's [surface] filter reads aeroValid; flight/730
       counts those rows). */
    Part *pod = b.v->parts[0];
    CHECK_TRUE(pod->aeroQ == 0.0f && !pod->aeroValid,
               "no body part carries the #730 surface stamp, now pinned not just said");
    CHECK_TRUE(b.v->parts[1]->formQ == 0.0f,
               "and the reverse half: a lifting surface is not in the union bill");
    CHECK_TRUE(b.v->lastFormWatts > 0.0,
               "#1114: a billed pass stamps the hull criterion beside the columns");
    /* #1106: CLa moves, by the law's OWN derivative. The frame's BodyCdA
       column is D cl vis sinA cosA (the force meter), so D cl vis reads back
       off it, and d(sin a cos^2 a)/da = cos a (cos^2 a - 2 sin^2 a) does the
       rest. Same alpha, same frame, same visible column the force just billed:
       the wing carries lift_area so it is not in the silhouette set at all,
       and both shots run on a fresh table (exact passes), so whatever the
       pod's visible happens to be cancels between the two sides of this pin
       -- what is left is the fold, not a fudge. (2e-6, not 1e-12: the
       production slope is a central finite difference at eps 1e-3, whose
       truncation measures ~3e-7 relative here and scales as eps^2.) */
    const double a10 = 10.0 * std::numbers::pi / 180.0;
    const double s10 = std::sin(a10), c10 = std::cos(a10);
    const double hullWant = b.v->lastBodyCdA / (s10 * c10)
        * c10 * (c10 * c10 - 2.0 * s10 * s10);
    CHECK_NEAR(b.v->lastLiftSlope - cla0, hullWant, std::fabs(hullWant) * 2e-6,
               "#1106: CLa gains exactly the hull's d(lift)/d(alpha) (drag.h "
               "bodyLiftSlopePerQ -- the finite-differenced bodyLiftForce)");
    CHECK_TRUE(b.v->lastStaticValid, "and the pair still answers, wings and hull together");
    /* #1113 review, run LAST so no extra fly moves lastLiftSlope under the CLa pin:
       formQ/formMach are pinned to the BILL's flow -- the cp's (#486), not the
       COM's. At zero spin the two coincide, so the identity pins the q's magnitude
       (and the pair is unswappable: ~4.6e4 Pa beside ~0.43). At 20 rad/s the sweep
       separates the cp's flow from the COM's: this fixture's cp-COM lever is only
       ~5 cm, so the shift is ~0.6% of q -- comfortably over the 0.4% floor, and an
       exact-COM mutant sits at 0.0. (cp-vs-mount stays with the stamp-site comment
       and the 1090-a row closure -- the bench holds no cp vector to aim at.) */
    CHECK_REL((double)pod->formQ,
              0.5 * b.v->lastDragRho * b.v->lastAirV * b.v->lastAirV, 1e-5,
              "no spin: the bill's q IS the ship's dynamic pressure (float stamp)");
    CHECK_TRUE(pod->formMach > 0.1f && pod->formMach < 1.0f,
               "subsonic Mach beside a ~4.6e4 Pa q: the pair cannot be swapped");
    const double w1 = pod->formDragWatts;
    CHECK_TRUE(w1 > 0.0, "the pod bills a real watt share, or the peak pin is vacuous");
    fly(b.v, 300.0, 10.0, 0.0);
    const double w2 = pod->formDragWatts;
    CHECK_NEAR((double)pod->formDragWattsPeak, (w1 > w2 ? w1 : w2), 0.0,
               "the peak folds in the same single pass as the surface one, and holds "
               "the worst billed run -- not just the last one");
    fly(b.v, 300.0, 10.0, 0.0, 20.0);  // 20 rad/s about the right axis
    const double comq = 0.5 * b.v->lastDragRho * b.v->lastAirV * b.v->lastAirV;
    CHECK_TRUE(std::fabs((double)pod->formQ - comq) > 0.004 * comq,
               "under rotation the bill's flow leaves the COM's: lfCp, not a COM "
               "stamp (the sweep term, #486)");
    teardown(b);
}

/* #1106: A HULL-ONLY AIRFRAME answers the pair, and answers it where the force
   columns are honest zeros. One plate with body_lift hung 1.5 m aft of the pod
   and 1.2 m off the axis (coaxial, the pod would shadow it at alpha 0), no
   lifting surface anywhere: before #1106 this build read CLa 0.00 SM - and
   that was the whole story the VAB told. The mesh stays centred on the mount,
   so cp == mount: the recorded vector sits at its attach offset MINUS the
   compound COM the pass subtracts (attach happens before launch here, so the
   pod's 100 kg and the fairing's 50 kg put that COM at (0, 0.4, -0.5) and the
   fairing's moment arm at (0, 0.8, -1.0)). The 60-degree shots run at NEGATIVE
   alpha, where the fairing stands upwind of the pod and sees the air whole.
   The pair's slope numbers are checked against the law's own derivative; the
   margin against that derivative and the RECORDED arm, which is where the
   fold's lever claim actually lives. */
static void test_hull_only_build_answers_the_pair() {
    printf("== #1106: a hull-only build answers CLa/SM, force-free at alpha 0 ==\n");
    Bench b = mkBench();
    Part *fair = mkPlate(b.defs, "fairing", 50.0, 0.4, 0.4, 0.5);
    b.defs.back().body_lift = 0.6;
    b.v->attach(fair, 0, glm::dvec3(0.0, 1.2, -1.5), glm::dmat3(1.0));
    launch(b);
    fly(b.v, 300.0, 0.0, 0.0);
    CHECK_NEAR(b.v->lastBodyCdA, 0.0, 1e-12, "alpha 0: sin a bills no force yet");
    CHECK_TRUE(b.v->lastStaticValid, "the pair still answers at alpha 0 -- it is a derivative");
    CHECK_REL(b.v->lastLiftSlope, b.v->drag_cd * 0.6 * 0.64, 2e-6,
              "CLa = drag_cd cl vis: the slope of sin a cos a at 0, on the "
              "0.8 x 0.8 m nose-on silhouette (2e-6: the eps 1e-3 finite "
              "difference truncates ~3e-7 here)");
    CHECK_NEAR(b.v->lastStaticMargin, 1.0, 1e-9,
               "SM = the -1.0 m moment arm, flipped: restoring, with no force aboard");
    /* Past 35.3 deg the sin cos law's own slope turns negative -- a hull
       parked there has a NEGATIVE lift slope, and now the pair says so
       instead of dashing. */
    const double a = -60.0 * std::numbers::pi / 180.0;
    const double s = std::sin(a), c = std::cos(a);
    fly(b.v, 300.0, -60.0, 0.0);
    const double vis = b.v->parts[1]->formVis;
    CHECK_TRUE(vis > 0.9, "upwind of the pod, the fairing sees the air whole");
    const double syn = c * (c * c - 2.0 * s * s);   // d(sin a cos^2 a)/dalpha
    const double szp = -s * (s * s - 2.0 * c * c);  // d(cos a sin^2 a)/dalpha
    const double dl = b.v->drag_cd * 0.6 * vis * syn;
    CHECK_REL(b.v->lastLiftSlope, dl, 2e-6,
              "CLa at -60 deg: the drooping slope of the same law, vis read off "
              "the frame's own visible column");
    // The one body-lift vector: arm, and the column that must agree with it.
    b.v->recordForceVecs = true;
    fly(b.v, 300.0, -60.0, 0.0);
    glm::dvec3 fpos(0.0);
    int at = 0;
    for(const Vehicle::ForceVec &fv : b.v->lastForceVecs) {
        if(fv.kind == Vehicle::kForceBodyLift) { fpos = fv.pos; at++; }
    }
    CHECK_TRUE(at == 1, "exactly one body-lift vector");
    CHECK_NEAR(fpos.x, 0.0, 1e-9, "the fixture is in the pitch plane");
    CHECK_NEAR(fpos.y, 0.8, 1e-9, "attach y minus the compound COM's y");
    CHECK_NEAR(fpos.z, -1.0, 1e-9, "attach z minus the compound COM's z: cp == mount");
    CHECK_NEAR(b.v->lastBodyCpLever, fpos.z, 1e-12,
               "single lifting part: the BodyCp column IS this vector's arm");
    const double marginWant = (fpos.y * szp - fpos.z * syn) / syn;
    CHECK_REL(b.v->lastStaticMargin, marginWant, 2e-6,
              "SM carries the full slope VECTOR (the z-term tilts the lever too), "
              "about the same arm the force meter quotes");
    CHECK_TRUE(b.v->lastStaticMargin > 0.0, "and this fixture is still restoring");
    teardown(b);
}

/* #1106: THE ARM ITSELF, on a hull whose MATERIAL sits forward of its bolt.
   Every other fixture in this file is a mount-centred box, so cp == mount and
   the centroid law is invisible -- green AND blind. This one slides the mesh
   +0.8 m along +z while the collision box and pose stay identical, so mass,
   inertia and pivot are identical between the benches and every difference
   below is the cp. The shot flies at NEGATIVE alpha so the fairing stands
   upwind of the pod and sees the air whole (at +60 it hides in the pod's
   shadow -- measured vis 0.17 and, slid forward, 0.0: the wrong fixture to
   talk about arms on). At -60 deg the projection can only recover the
   IN-PLANE part of the slide: sin^2 x 0.8 = +0.6 m of nose arm and
   sin cos x 0.8 = -0.346 m of offset toward the pod's side. The along-flow
   part of the slide is unrecoverable from any silhouette -- that is
   #512's caveat, and pinning the exact sin^2 rather than the full 0.8 is what
   keeps the promise honest (a static mesh-box centre would "pass" here by
   promising an arm the projection never saw). The last pins move the SLOPE
   lever too: SM must shift by the lever ratio across the arm gap, or a fold
   that adds its slope at the bolt while the force acts at the cp keeps half
   the ticket undone with every arm pin green. */
static void test_hull_lift_arm_is_the_projected_centroid() {
    printf("== #1106: the arm is the projected centroid, not the bolt ==\n");
    auto shot = [](Bench &b, double *bodycp, glm::dvec3 *fpos, double *margin) {
        b.v->recordForceVecs = true;
        fly(b.v, 300.0, -60.0, 0.0);  // fresh table: the exact pass, products and all
        b.v->lastForceVecs.clear();
        fly(b.v, 300.0, -60.0, 0.0);
        CHECK_TRUE((double)b.v->parts[1]->formVis > 0.9, "the fairing is exposed");
        *bodycp = b.v->lastBodyCpLever;
        *margin = b.v->lastStaticMargin;
        int at = 0;
        for(const Vehicle::ForceVec &fv : b.v->lastForceVecs) {
            if(fv.kind == Vehicle::kForceBodyLift) { *fpos = fv.pos; at++; }
        }
        CHECK_TRUE(at == 1, "one body-lift vector on this hull");
        b.v->recordForceVecs = false;
    };
    Bench mid = mkBench();
    Part *fMid = mkPlate(mid.defs, "fairing", 50.0, 0.4, 0.4, 0.5);
    mid.defs.back().body_lift = 0.6;
    mid.v->attach(fMid, 0, glm::dvec3(0.0, 1.2, -1.5), glm::dmat3(1.0));
    launch(mid);
    double cpMid = 0.0, mMid = 0.0; glm::dvec3 posMid(0.0);
    shot(mid, &cpMid, &posMid, &mMid);

    Bench fwd = mkBench();
    Part *fFwd = mkPlate(fwd.defs, "fairing", 50.0, 0.4, 0.4, 0.5);
    for(glm::dvec3 &h : fFwd->body->hullVerts) { h += glm::dvec3(0.0, 0.0, 0.8); }
    fwd.defs.back().mesh_min += glm::dvec3(0.0, 0.0, 0.8);
    fwd.defs.back().mesh_max += glm::dvec3(0.0, 0.0, 0.8);
    fwd.defs.back().body_lift = 0.6;
    fwd.v->attach(fFwd, 0, glm::dvec3(0.0, 1.2, -1.5), glm::dmat3(1.0));
    launch(fwd);
    double cpFwd = 0.0, mFwd = 0.0; glm::dvec3 posFwd(0.0);
    shot(fwd, &cpFwd, &posFwd, &mFwd);

    const double a = -60.0 * std::numbers::pi / 180.0;
    const double s = std::sin(a), c = std::cos(a);
    const double syn = c * (c * c - 2.0 * s * s);   // d(sin a cos^2 a)/dalpha
    const double szp = -s * (s * s - 2.0 * c * c);  // d(cos a sin^2 a)/dalpha
    CHECK_NEAR(cpFwd - cpMid, 0.8 * s * s, 1e-6,
               "BodyCp moved by the in-plane part of the slide, sin^2 x 0.8: the "
               "lift lands 0.6 m further forward than the bolt (cf. #947's review: "
               "an arm must be real geometry, never an accident of the mount)");
    CHECK_NEAR(posFwd.z - posMid.z, 0.8 * s * s, 1e-6, "the force vector moved with it");
    CHECK_NEAR(posFwd.y - posMid.y, 0.8 * s * c, 1e-6,
               "and the in-plane offset toward the axis the same projection implies");
    /* THE SLOPE LEVER, the other half of #1106. Everything above moves the FORCE
       arm; a fold that kept adding its slope at the bolt (hullSlope.add(..., ri,
       ...) in vehicle.cpp) would pass every pin above and silently undo half the
       ticket. The slope vector's pattern is common to both benches up to each
       one's frozen visible -- same law, same alpha, fresh table -- and SM is a
       dm/dlabs RATIO, so visible cancels in the gap: the mid->fwd margin move
       must equal the hull-only bench's lever formula on the MEASURED arm gap.
       0.8 m of slide taking nearly half a metre off the restoring margin: a
       bolt-lever fold would leave the gap at zero and redden this. */
    const double dMarginWant = ((posFwd.y - posMid.y) * szp
                                - (posFwd.z - posMid.z) * syn) / syn;
    CHECK_REL(mFwd - mMid, dMarginWant, 2e-6,
              "SM moved with the cp, not with the bolt: the fold's lever is the "
              "same projected centroid the force acts at");
    CHECK_TRUE(std::fabs(dMarginWant) > 0.3,
               "the fixture separates the two levers by a measurable margin gap");
    /* Both benches get the identical two-shot script on a fresh table, so the
       counted shot is the exact pass in each and the difference above is pure
       geometry, not cache state. A HIT blends the product column bilinearly,
       which reproduces a linear function exactly but not sin^2 -- a few mm of
       wobble across a 5 deg cell, the same kind of wobble test_silhouetteMemo
       pins as EXPECTED for cpNum. What must never happen is the arm depending
       on whether this part happened to have been projected before (#1090's
       doctrine): hit or miss, both routes carry a centroid. */
    teardown(mid);
    teardown(fwd);
}

/* THE RATE COLUMN: the same hull with an angular velocity, which is #487's whole
   question and the column #655's header said was missing. A rotating airframe carries
   each surface through the air (drag.h::localFlow, #483), so every surface meets the
   flow at its OWN angle; the two ends of that tilt are the pitch damping a plane needs
   and, inside the stall droop, the pump that feeds the tumble instead of stopping it.
   The steady moment at a fixed alpha is not the measurement -- it is the same
   restoring term the no-rate rows already pay. The rate's own contribution is the
   DIFFERENCE against the same hull at the same alpha with no spin, and the sign of
   that difference dotted with the rate is the answer: negative takes energy out of the
   rotation (damping), positive puts energy in (the pump, #487). */
static void test_rate_column() {
    printf("== the same hull with a pitch rate: damping, and the pump inside the droop ==\n");
    Bench b = mkBench();
    Part *wing = mkPlate(b.defs, "wing", 20.0, 2.0, 0.05, 1.0);
    setSurface(b.defs.back(), 8.0, true);
    b.v->attachMode(wing, 0, AttachMode::Down);
    Part *elev = mkPlate(b.defs, "elevator", 5.0, 1.0, 0.05, 1.0);
    setSurface(b.defs.back(), 2.0, false);
    b.v->attachMode(elev, 0, AttachMode::Down, 0.0, 3.0);
    launch(b);

    const double speeds[] = { 100.0 };
    const double alphas[] = { 0.0, 10.0, 30.0, 45.0 };   // 30 is inside 0.35..0.70 rad
    const double rates[] = { 0.0, 0.5, 1.5, 3.0 };       // rad/s about the ship's right
    for(double sp : speeds) {
        for(double al : alphas) {
            fly(b.v, sp, al, 0.0, 0.0);
            const glm::dvec3 m0 = b.v->lastAeroTorque;
            double prevAbs = -1.0;
            for(double w : rates) {
                fly(b.v, sp, al, 0.0, w);
                const glm::dvec3 dm = b.v->lastAeroTorque - m0;
                const double pw = glm::dot(dm, glm::dvec3(w, 0.0, 0.0));
                printf("[rate] alpha %5.1f  w %+.2f rad/s  hull AoA %6.2f  wing a %+7.2f"
                       "  |tau| %10.2f  dM|w %10.2f  dM.w %11.2f W  CLa %8.2f  SM %s\n",
                       al, w, b.v->lastDragAlpha * 180.0 / std::numbers::pi,
                       b.v->parts[1]->aeroAlphaDeg, glm::length(b.v->lastAeroTorque),
                       dm.x, pw, b.v->lastLiftSlope,
                       b.v->lastStaticValid ? "ok" : "-");
                /* No spin is not "mostly the same", it is the same number: the sweep
                   term is exactly zero, so the rate column cannot shift anything. */
                if(w == 0.0) {
                    CHECK_NEAR(glm::length(dm), 0.0, 0.0, "w 0: the rate changes nothing");
                    continue;
                }
                /* The sign IS the finding. Inside the linear range the air takes the
                   rotation's energy out (pitch damping); past the stall angle and under
                   twice it, the slope is negative and the airframe feeds the tumble
                   instead (#487) -- here +6.6 kW at 1.5 rad/s where alpha 10 takes
                   15.7 kW out. CLa says it in the same breath: -48 m2/rad at alpha 30.
                   Pinning both halves is what makes a droop change a visible event. */
                if(al < 20.0) {
                    CHECK_TRUE(pw < 0.0, "the linear range damps the rate it meets");
                } else if(al >= 30.0 && al < 40.0) {
                    CHECK_TRUE(pw > 0.0, "inside the droop the airframe PUMPS its own rate");
                    CHECK_TRUE(b.v->lastLiftSlope < 0.0,
                               "and the lift slope the log prints is negative there");
                }
                /* Power goes as w^2 (the moment is linear in the rate, and q rises with
                   the sweep on top), so doubling the rate more than triples the bill. */
                if(prevAbs > 0.0) {
                    CHECK_TRUE(std::fabs(pw) > 3.0 * prevAbs,
                               "the rate term grows faster than the rate it opposes");
                }
                prevAbs = std::fabs(pw);
            }
        }
    }

    /* The angle the law reads is the SURFACE's, not the hull's (#483): at hull AoA 0
       with a 3 rad/s pitch rate the wing meets the flow at over 2 deg and the hull
       quotes 0.00. Nothing else in this bench would notice if the sweep went missing,
       and the damping pins above are exactly what depends on it. */
    fly(b.v, 100.0, 0.0, 0.0, 0.0);
    const double flatAlpha = b.v->parts[1]->aeroAlphaDeg;
    fly(b.v, 100.0, 0.0, 0.0, 3.0);
    CHECK_NEAR(b.v->lastDragAlpha, 0.0, 1e-12, "the hull still flies dead along its nose");
    CHECK_NEAR(flatAlpha, 0.0, 1e-12, "with no spin, and so does the wing");
    CHECK_TRUE(b.v->parts[1]->aeroAlphaDeg < -2.0,
               "a 3 rad/s pitch rate tilts the wing's own flow past 2 deg");

    /* Past twice the stall angle the droop has finished: the lifting law is silent, so
       the margin dashes rather than quoting a lever for an airframe that has stopped
       lifting, and the damping that remains is the hull's form drag (#486) -- which is
       why a broadside hull still settles, and why it settles slowly. */
    fly(b.v, 100.0, 45.0, 0.0, 3.0);
    CHECK_NEAR(b.v->lastLiftSlope, 0.0, 1e-12, "no lift slope past twice the stall angle");
    CHECK_TRUE(!b.v->lastStaticValid, "and the static margin dashes instead of lying");
    teardown(b);
}

/* THE NEUTRAL FIN. A steering part's own angle of attack enters its force law only
   while a NON-ZERO command exists -- vehicle.cpp:3771's `cmd == 0.0` early-out skips
   everything up to vehicle.cpp:3892's `aeroAngle = deflection + (systemLifts ? 0 : pa.alpha)`.
   So the side force is DISCONTINUOUS in the command at zero: a 1e-12 rad trim deflects
   nothing (its own q*S*cl*delta is 0.000 N) and still buys the entire weathervane.
   Pinned as the law now is, because #551/#508 are the question and the answer should
   move these pins, not a deleted scratch probe. The beta 40 half says where the gate
   stops mattering: past twice the fin's stall angle its own alpha is dead, so opening
   the gate hands the law nothing. */
static void test_neutral_fin_alpha_gate() {
    printf("== a vertical fin, uncommanded and commanded by nothing ==\n");
    Bench b = mkBench();
    Part *fin = mkPlate(b.defs, "rudder", 5.0, 1.0, 0.05, 1.0);
    setSurface(b.defs.back(), 2.0, false);
    b.defs.back().control_axis = ControlAxis::Yaw;    // what a vertical fin is for
    // 90 deg about the stack axis: the plate stands up, so a sideslip is its angle of attack.
    b.v->attachMode(fin, 0, AttachMode::Down, 90.0, 3.0);
    launch(b);

    auto side = [&](double trimDeg, double beta) {
        fin->trimDeg = trimDeg;
        fly(b.v, 200.0, 0.0, beta);
        return b.v->lastAeroForce.x;
    };
    const double n15 = side(0.0, 15.0);
    const double c15 = side(glm::degrees(1e-12), 15.0);
    printf("[fin] beta 15 at 200 m/s: uncommanded side %10.2f N   1e-12 rad trim %10.2f N"
           "   jump %10.2f N\n", n15, c15, c15 - n15);
    /* Uncommanded, a fin standing in a 15 deg sideslip adds exactly nothing: what is
       left is the pod's own form drag off the nose. */
    CHECK_NEAR(n15, -4946.76, 0.01, "an uncommanded fin makes no side force at all");
    CHECK_NEAR(c15, -96896.19, 0.01, "a 1e-12 rad command buys the whole weathervane");
    CHECK_TRUE(std::fabs(c15) > 19.0 * std::fabs(n15),
               "so the law jumps by 20x the ship's own side force on a command of nothing");

    const double n40 = side(0.0, 40.0);
    const double c40 = side(glm::degrees(1e-12), 40.0);
    printf("[fin] beta 40: uncommanded %10.2f N   commanded %10.2f N"
           "   (past 2x the fin's stall angle the gate opens onto nothing)\n", n40, c40);
    CHECK_NEAR(c40, n40, 1e-9, "and where the fin's own alpha is dead, the gate changes nothing");
    teardown(b);

    /* A second gate nobody names: the alpha term is billed only when the DEFLECTION
       itself is non-zero, and controlDeflection answers 0 for a mount with no lever
       about the surface's AUTHORED control_axis (vehicle.cpp:3871's skip). Same fin,
       same mount, authored Pitch instead of Yaw: it never weathervanes at all, however
       hard it is commanded -- while vehicle.cpp:3751 calls control_axis "the label for
       WHICH KEY asks... not a wall on what the plate is for". */
    Bench pb = mkBench();
    Part *pfin = mkPlate(pb.defs, "rudder", 5.0, 1.0, 0.05, 1.0);
    setSurface(pb.defs.back(), 2.0, false);        // setSurface's axis is Pitch
    pb.v->attachMode(pfin, 0, AttachMode::Down, 90.0, 3.0);
    launch(pb);
    pfin->trimDeg = 0.0;
    fly(pb.v, 200.0, 0.0, 15.0);
    const double pitchNeutral = pb.v->lastAeroForce.x;
    pfin->trimDeg = glm::degrees(1e-12);
    fly(pb.v, 200.0, 0.0, 15.0);
    printf("[fin] the same fin authored PITCH: uncommanded %10.2f N   commanded %10.2f N"
           "   defl %+.3e rad\n", pitchNeutral, pb.v->lastAeroForce.x, pfin->controlDefl);
    CHECK_NEAR(pb.v->lastAeroForce.x, pitchNeutral, 1e-9,
               "a fin with no lever about its authored axis never weathervanes");
    teardown(pb);
}

int main() {
    test_surface_sweep();
    test_lift_system_merge();
    test_induced_sum_is_sign_blind();
    test_surface_authority_curve();
    test_heat_hooks();
    test_body_lift();
    test_body_lift_mount_flip_invariance();
    test_body_lift_stays_out_of_the_surface_systems();
    test_hull_only_build_answers_the_pair();
    test_hull_lift_arm_is_the_projected_centroid();
    test_rate_column();
    test_neutral_fin_alpha_gate();
    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
