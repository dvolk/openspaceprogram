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
//   THE ENERGY HOOK a heating model would read (#553/#554/#728): exact per surface --
//   and zero for a body, which is what a capsule is made of.
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

static int g_failures = 0;
static int g_checks = 0;

#define CHECK_TRUE(cond, msg)                                                 \
    do {                                                                      \
        g_checks++;                                                           \
        if(!(cond)) { g_failures++; printf("FAIL: %s\n", msg); }              \
    } while(0)

#define CHECK_NEAR(actual, expected, tol, msg)                                \
    do {                                                                      \
        g_checks++;                                                           \
        double _a = (actual), _e = (expected);                                \
        /* Both sides guarded: a "diff > tol" test cannot see a NaN, and a    \
           pin written the other way round would pass on one (#604). */       \
        if(!std::isfinite(_a) || !std::isfinite(_e)                           \
           || std::fabs(_a - _e) > (tol)) {                                   \
            g_failures++;                                                     \
            printf("FAIL: %s (got %.9g, want %.9g, tol %.3g)\n",              \
                   msg, _a, _e, (double)(tol));                               \
        }                                                                     \
    } while(0)

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
           "  L %10.2f N  D %10.2f N  Side %10.2f N  |tau| %10.2f Nm"
           "  A %6.3f Cd %5.3f  Profile %6.4f  Induced %7.4f  CdAtot %7.4f"
           "  CpL %6.3f  FormCp %6.3f  CLa %8.3f  SM %8s\n",
           speed, alpha, beta, v->lastDragRho,
           v->lastAirV / (v->lastAirA > 0.0 ? v->lastAirA : 1.0),
           v->lastFlowLift, v->lastFlowDrag,
           glm::dot(v->lastAeroForce, glm::dvec3(1, 0, 0)),
           glm::length(v->lastAeroTorque),
           v->lastDragArea, v->lastDragCd, v->lastSurfCdA, v->lastInducedCdA,
           v->lastSurfCdA + v->lastInducedCdA
               + v->drag_cd * v->lastDragCd * v->lastDragArea,
           v->lastCpLever, v->lastFormCpLever, v->lastLiftSlope, sm);
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

/* THE ONLY ENERGY HOOK A HEATING MODEL COULD READ TODAY. #553 (no reentry heat),
   #554 (no structural limits / max Q) and #728 (heat as a mechanic, parked) are all
   open, and there is no temperature or damage state anywhere in the codebase -- so the
   question is what the smallest credible version would read. Part::aeroDragWatts is
   the one per-part figure the sim already keeps every substep, and this column shows
   both halves of what it can do:
     - A SURFACE's number is exact, per part, per substep: q and v and CdA are already
       per-surface since #483/#730, so a heat rating could be authored per part today.
       The pin below is the arithmetic itself -- watts = drag_cd x q x CdA x v.
     - A BODY pays nothing. The hull's form drag is billed on the union at one cp
       (#463/#514), and no per-part watts are folded for it, so a reentering CAPSULE --
       which is all hull and no surfaces -- reports 0 W through the whole heating. The
       pin below locks that in as the current law, so the day someone folds the hull's
       share in, a test moves.
   The v^3 row is the sizing number. 20 km up on this body (rho = 1.225 e^-2.5 = 0.109),
   the 8 m2 wing's watts run 0.20 MW at 300 m/s, 7.07 MW at 1000 m/s and 48.9 MW at
   2000 m/s -- 25 kW/m2, 0.88 MW/m2 and 6.1 MW/m2. The whole ship drags 2.9 kN, 31.7 kN
   and 122 kN there, and only the two surfaces' share of that power is booked anywhere.
   Whatever a rating turns out to be, that is the order of number it has to answer, and
   the hull is where a capsule would have to put it. */
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
        fly(b.v, sp, 0.0, 0.0);
        const double q = 0.5 * b.v->lastDragRho * sp * sp;
        printf("[heat] v %7.1f  rho %8.5f  Mach %5.2f  q %11.1f Pa  "
               "hull CdA %6.3f m2  ship drag %11.1f N  -> W %11.1f  |  "
               "wing W %11.1f  tab W %10.1f  (per m2 of wing %10.1f W/m2)\n",
               sp, b.v->lastDragRho, sp / b.v->lastAirA, q,
               b.v->lastDragArea, b.v->lastFlowDrag, b.v->lastFlowDrag * sp,
               (double)b.v->parts[1]->aeroDragWatts, (double)b.v->parts[2]->aeroDragWatts,
               (double)b.v->parts[1]->aeroDragWatts / 8.0);
        /* The surface hook, exactly: drag_cd x q x (its own chordwise cd) x v. At
           alpha 0 a surface's whole bill is its chordwise cd -- its profile cd plus,
           past the sound bar, its own wave cd (#543 A5b), and induced is zero. */
        const double wv = waveCd(0.1 / 2.0, sp / b.v->lastAirA);
        CHECK_NEAR((double)b.v->parts[1]->aeroDragWatts,
                   b.v->drag_cd * q * ((0.01 + wv) * 8.0) * sp,
                   b.v->drag_cd * q * 0.08 * sp * 1e-6,
                   "a surface's watts are its own q x CdA x v, ready to integrate");
        /* ... and the hole: the ship's drag power (the aero force along the flow times
           v) is 0.87 MW at 300 m/s, and these two surfaces book only 0.25 MW of it. The
           pod -- all the hull there is -- keeps no energy figure at all. */
        CHECK_TRUE(b.v->lastFlowDrag > 0.0, "the ship is definitely paying drag");
        CHECK_NEAR((double)b.v->parts[0]->aeroDragWatts, 0.0, 0.0,
                   "a BODY folds no watts at all: a capsule reports 0 W reentry (#553)");
        if(prevW > 0.0) {
            CHECK_TRUE((double)b.v->parts[1]->aeroDragWatts > 3.0 * prevW,
                       "the hook's number climbs faster than v^2, so a rating must be "
                       "a flux not a force");
        }
        prevW = (double)b.v->parts[1]->aeroDragWatts;
    }
    teardown(b);
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

int main() {
    test_surface_sweep();
    test_lift_system_merge();
    test_induced_sum_is_sign_blind();
    test_surface_authority_curve();
    test_heat_hooks();
    test_rate_column();
    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
