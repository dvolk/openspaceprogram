// test_stability: shipdef.cpp's buildStability -- the answer to "is this airframe
// stable?" asked of a BUILD, with no atmosphere, no Bullet and no substep (#499).
// Runs from the repo root (needs res/):
//   make test_stability   (or ./build/<arch>/tests/test_stability)
//
// The claim worth pinning is the one the feature makes to a player: the number the
// VAB prints is the number the flight log prints. The shipped skiff measured in the
// air (e2e 454-static-margin, plus --drag-log by hand) is CLa 153.4 m2/rad and SM
// +0.60 m; these cases reproduce both from the parts alone, which is only true if
// every convention lines up -- the launch COM (tanks full, one suit per seat), the
// root's axes, and the same liftSlopePerQ the lift loop bills. A unit pin on
// drag.h::LiftSlopeSum cannot see that composition, and this one broke the first
// time it was written: forgetting the seated kerbal (97.05 kg in the nose) moved
// the skiff from +0.60 m to +0.42 m, 30% of the answer.

#include "shipdef.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

static int failures = 0;
#define CHECK(cond) do { \
        if(!(cond)) { \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++; \
        } \
    } while(0)

static void check_near(const char *what, double got, double want, double tol) {
    if(!(std::fabs(got - want) <= tol)) {
        printf("FAIL %s:%d: %s: got %.4f, want %.4f +/- %.4f\n",
               __FILE__, __LINE__, what, got, want, tol);
        failures++;
    }
}

// Loaded once; every case reads the same catalog the game boots with.
struct Fixture {
    PartsCatalog cat;
    bool ok = false;
    double occupant = 0.0;   // what one seated kerbal weighs

    BuildShip build(const char *path) const {
        ShipDef def = load_ship_def(path, cat);
        return BuildShip::fromShipDef(def);
    }
};

static Fixture load_all() {
    Fixture f;
    try {
        f.cat = load_parts_catalog("res/data/parts.json");
        f.occupant = seatOccupantMass(load_ship_def("res/ships/kerbal.json", f.cat));
        f.ok = true;
    } catch(const std::exception &e) {
        printf("FAIL: could not load res/: %s\n", e.what());
    }
    return f;
}

/* The shipped airframes, against what the same parts measure in the air. The
   bands are the two known differences, not slack: the flight number carries the
   cos(alpha) dip (CLa 153.42 at 0 deg, 152.55 at 6.6) and burns fuel mid-run, so
   it drifts a little BELOW the design value. 0.03 m on the margin is the same
   story -- fuel burn walks the COM. */
static void test_shipped_against_flight(const Fixture &f) {
    struct Row { const char *path; double slope; double margin; };
    const Row rows[] = {
        { "res/ships/skiff.json",  153.42, 0.395 },
    };
    for(const Row &r : rows) {
        const BuildShip bs = f.build(r.path);
        const BuildStability st = buildStability(bs, f.occupant);
        CHECK(st.valid);
        check_near(r.path, st.slope, r.slope, 0.6);
        check_near(r.path, st.margin, r.margin, 0.03);
    }

    /* The COM itself, pinned against the compound mass the flight ship reports
       (626.3 kg for the skiff at launch, fuselage + 96 kg of jet fuel). Every aero number hangs off this pivot,
       and the two routes agree only while "tanks full, one suit per seat" means
       the same thing on both sides. */
    const BuildStability skiff = buildStability(f.build("res/ships/skiff.json"),
                                                f.occupant);
    check_near("skiff launch mass", skiff.mass, 626.3, 0.5);
}

/* A seated kerbal is part of the airframe. Measured on the skiff: its capsule sits
   forward, so the 97.05 kg suit pulls the COM 0.18 m ahead of the parts-only one
   and the margin grows by 0.18 m of restoring. Quoting the parts-only figure for
   a ship that will launch crewed is a 30% error in the number the player builds
   to, which is why the crew term is here rather than in the caller's head. */
static void test_crew_moves_the_pivot(const Fixture &f) {
    const BuildShip bs = f.build("res/ships/skiff.json");
    const BuildStability crewed = buildStability(bs, f.occupant);
    const BuildStability empty = buildStability(bs, 0.0);
    CHECK(crewed.valid);
    CHECK(empty.valid);
    // A suit is the catalog's kerbal part plus its hydrazine, nothing else.
    check_near("occupant mass", f.occupant, 87.05 + 10.0, 1e-6);
    // The fuselage seats two, so a full crew is two suits.
    CHECK(crewed.mass > empty.mass + 2.0 * 90.0);
    CHECK(crewed.mass < empty.mass + 2.0 * 105.0);
    check_near("crewed margin", crewed.margin, 0.395, 0.03);
    check_near("uncrewed margin", empty.margin, 0.254, 0.03);
    // The lift does not change, only where it is measured from.
    check_near("slope unchanged by crew", crewed.slope, empty.slope, 1e-9);
    // The suit sits at the seat, so the COM moves toward the capsule (+z).
    CHECK(crewed.com.z > empty.com.z + 0.10);
}

/* The failure #499 is about: a rig whose lift change sits AHEAD of the pivot. Same
   parts, same areas, same masses -- the rig alone is what a builder gets wrong, and
   this is the case that has to say so before the runway.
   Measured, by moving one pair 3 m forward in the skiff (no recomputePoses: the
   pose IS the edit):
     as shipped          SM +0.60 m  restoring
     tailplanes +3 m     SM +0.20 m  restoring, and the number shows why
     wings +3 m          SM -1.46 m  diverging
   The middle row is the honest one and is pinned so nobody "fixes" it by expecting
   a flip: the skiff's wings are 129.6 of its 153.4 m2/rad, so putting its tail in
   front of them costs two thirds of the stiffness without turning the airframe
   inside out. Moving the WINGS forward does. */
static void test_rig_flips_the_verdict(const Fixture &f) {
    const BuildShip asShipped = f.build("res/ships/skiff.json");
    const BuildStability good = buildStability(asShipped, f.occupant);
    CHECK(good.valid);
    CHECK(good.margin > 0.0);

    BuildShip tailsFwd = f.build("res/ships/skiff.json");
    int moved = 0;
    for(BuildPart &bp : tailsFwd.parts) {
        if(bp.id == "tail_l" || bp.id == "tail_r") {
            bp.localPos.z += 3.0;
            ++moved;
        }
    }
    CHECK(moved == 2);
    const BuildStability mid = buildStability(tailsFwd, f.occupant);
    CHECK(mid.valid);
    check_near("tails forward of the wings", mid.margin, -0.016, 0.03);
    CHECK(mid.margin < good.margin - 0.3);

    BuildShip wingsFwd = f.build("res/ships/skiff.json");
    moved = 0;
    for(BuildPart &bp : wingsFwd.parts) {
        if(bp.id == "wing_l" || bp.id == "wing_r") {
            bp.localPos.z += 3.0;
            ++moved;
        }
    }
    CHECK(moved == 2);
    const BuildStability bad = buildStability(wingsFwd, f.occupant);
    CHECK(bad.valid);           // the surfaces still agree; the rig is the problem
    CHECK(bad.margin < 0.0);    // ... and the readout says which way
    check_near("wings well ahead", bad.margin, -1.722, 0.05);

    /* All three rigs bill the same lift; only where it sits changed. A slope that
       moves under a pose edit means the sum is reading something other than the
       distribution -- which is how #454's first version drifted on one skiff. */
    check_near("slope kept by the tail edit", mid.slope, good.slope, 1e-9);
    check_near("slope kept by the wing edit", bad.slope, good.slope, 1e-9);
}

/* Nothing lifting is not "zero stability": LiftSlopeSum::usable dashes the pair
   rather than quoting where the average of a cancelled sum landed (#454 quoted
   Np=+15.9 m, then +121.5, then -89.7 for one skiff). The caller prints "-" from
   this flag, so a silently-valid zero is what would ship the lie. */
static void test_nothing_lifts_dashes(const Fixture &f) {
    const BuildShip bs = f.build("res/ships/racer.json");
    const BuildStability st = buildStability(bs, f.occupant);
    CHECK(!st.valid);
    CHECK(st.slope == 0.0);
    CHECK(st.margin == 0.0);   // never printed, and never a fake +0.00 "neutral"
}

/* An empty build has no pivot to speak of. Not a player-facing case -- a
   zero-part BuildShip is what a fresh VAB session holds before the first click,
   and the readout runs on it every frame. */
static void test_empty_build(const Fixture &f) {
    BuildShip bs;
    const BuildStability st = buildStability(bs, f.occupant);
    CHECK(!st.valid);
    CHECK(st.mass == 0.0);
}

int main() {
    const Fixture f = load_all();
    if(!f.ok) { return 1; }
    test_shipped_against_flight(f);
    test_crew_moves_the_pivot(f);
    test_rig_flips_the_verdict(f);
    test_nothing_lifts_dashes(f);
    test_empty_build(f);
    if(failures) {
        printf("test_stability: %d failure(s)\n", failures);
        return 1;
    }
    printf("test_stability: all tests passed\n");
    return 0;
}
