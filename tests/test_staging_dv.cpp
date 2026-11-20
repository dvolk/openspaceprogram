// test_staging_dv: VAB staging analysis (src/staging.cpp) -- per-stage
// vacuum delta-v and TWR, including fuel-link asparagus.
//
// Pins:
//   * a plain two-stage rocket: Tsiolkovsky delta-v per stage, min TWR at
//     the start of the burn (heaviest) and max at the end (lightest),
//     against the reference g;
//   * dry mass = catalog mass (the DRY structure) + inert resources:
//     PartDef::mass EXCLUDES propellant -- H2+LOX rides the burnable pool,
//     hydrazine / jet fuel ride the dry;
//   * asparagus (outer -> inner fuel links): the outer booster's tanks
//     empty first (shared with the core engine), the stage drops when
//     THEY are empty, and the core keeps burning afterwards;
//   * decouplers are fuel barriers (the core engine cannot drain a booster's
//     tanks without an explicit link).
//   * drain layers: a lit engine drains the FURTHEST link layer first and each
//     period ends on its own layer, by hop distance rather than part order;
//     the buffers that walk them are reused between calls, so the second ship in
//     a process must answer for its own graph (#963).
//   * the whole simulation snapshot is reused that way too: a small ship planned after
//     a big one carries neither the big one's dead parts nor its engine list, and the
//     big one after it still grows back (#963).
//   * a fuel link's endpoint id resolves to the LAST part that carries it, as the old
//     id map did; with a duplicated id that decides whether the link is real (#963).
//   * stage numbers are LABELS: the walk visits the labels the ship carries, highest
//     first, and a gap between two labels costs nothing (#517).
//   * autoStage assigns those labels from the tree instead of by hand, keeps an authored cut
//     sequence over the tree's depth guess but clamps it so no cut rides a press inside a cut
//     that contains it (#1020), ranks the cuts by the fuel chain instead when the player asks
//     for it (#1031), keeps an engine lit that the build lights from the pad unless that would
//     empty the floor label, and refuses where the tree cannot read the stack at all -- no
//     decoupler, a cut that would drop everything, an engine re-sequenced relative to another
//     (#1012).
//   * a staging edit takes itself back: the change list an edit performed is the undo
//     record, so an undo finds parts by id (the vector renumbers under it), leaves a label
//     that something else moved since, says which parts it could not reach, and moves
//     stagingSignature so the Staging window's cache notices for free (#1013). One step is
//     one decision: a run of number-box retypes on one part merges into a step, and a bulk
//     plan never merges with anything, not even when it moves exactly one part.
//
// Pure math over BuildShip -- no GL, no Bullet.
//
// Runs from the repo root:
//   make test   (or: ./test_staging_dv)

#include "staging.h"
#include "part.h"   // Part's live predicates, which stageRoles() must not drift from (#1009)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>
#include "tchecks.h"

// PartDef lives in a deque so pointers stay valid while the build grows.
// Tanks carry mass + capacity; engines carry thrust (and may hold no
// propellant of their own -- they drain their fuel group).
struct Catalog {
    std::deque<PartDef> defs;

    const PartDef *tank(double dry, double h2, double lox) {
        defs.push_back(PartDef());
        PartDef &d = defs.back();
        d.mass = dry;   // DRY: catalog mass is structure only; the fuel rides capacity
        d.capacity[(int)ResourceType::Hydrogen] = h2;
        d.capacity[(int)ResourceType::LOX] = lox;
        return &d;
    }
    const PartDef *engine(double dry, double fuelRate, double ve) {
        defs.push_back(PartDef());
        PartDef &d = defs.back();
        d.mass = dry;
        // a chemical engine draws H2 + LOX, each at fuelRate, so the total
        // propellant flow (the staging mdot) is 2*fuelRate.
        d.setPropellantRate(ResourceType::Hydrogen, fuelRate);
        d.setPropellantRate(ResourceType::LOX, fuelRate);
        d.exhaust_velocity = ve;
        return &d;
    }
    // A nuclear-thermal engine: draws H2 ONLY (no onboard oxidizer), so the
    // total propellant flow (the staging mdot) is just h2Rate and the thrust
    // is h2Rate * ve.
    const PartDef *nuclearEngine(double dry, double h2Rate, double ve) {
        defs.push_back(PartDef());
        PartDef &d = defs.back();
        d.mass = dry;
        d.setPropellantRate(ResourceType::Hydrogen, h2Rate);
        d.exhaust_velocity = ve;
        return &d;
    }
    const PartDef *decoupler(double mass) {
        defs.push_back(PartDef());
        defs.back().mass = mass;
        defs.back().decoupler = true;
        defs.back().fuel_barrier = true;
        return &defs.back();
    }
    // A parachute: chute_area > 0 is the whole predicate (part.h:216), and the
    // only thing that makes a label mean "deploy" at a press (vehicle.cpp:3136).
    const PartDef *chute(double mass, double area) {
        defs.push_back(PartDef());
        defs.back().mass = mass;
        defs.back().chute_area = area;
        return &defs.back();
    }
    // A fuel wall that is NOT a decoupler: it splits fuel groups but can never be
    // what a stage label fires, so a ship can be walled apart without separating.
    const PartDef *barrier(double mass) {
        defs.push_back(PartDef());
        defs.back().mass = mass;
        defs.back().fuel_barrier = true;
        return &defs.back();
    }
    const PartDef *mono(double dry, double hydrazine) {
        defs.push_back(PartDef());
        PartDef &d = defs.back();
        d.mass = dry;   // DRY: the hydrazine rides capacity (inert mass in staging)
        d.capacity[(int)ResourceType::Hydrazine] = hydrazine;
        return &d;
    }
};

static int addPart(BuildShip &bs, const PartDef *def, const std::string &id,
                   int parent, int stage) {
    BuildPart bp;
    bp.def = def;
    bp.id = id;
    bp.parent = parent;
    bp.stage = stage;
    bs.parts.push_back(bp);
    return (int)bs.parts.size() - 1;
}

static void addLink(BuildShip &bs, const std::string &from, const std::string &to) {
    BuildShip::FuelLink fl;
    fl.def = nullptr;
    fl.id = "link_" + from + "_" + to;
    fl.from = from;
    fl.to = to;
    bs.fuelLinks.push_back(fl);
}

// rate 0.5 per propellant (H2+LOX) -> mdot = 1 kg/s, T = 1*1000 = 1000 N.
static void test_two_stage() {
    Catalog cat;
    BuildShip bs;
    // Upper: dry 50 + fuel 50 = wet 100. Engine dry 10.
    // Lower: dry 50 + fuel 150 = wet 200. Engine dry 10. Decoupler 5.
    // Stage 1 is the LAST to fire (the upper); stage 2 (the lower) fires
    // first and drops.
    const int upTank = addPart(bs, cat.tank(50, 25, 25), "upTank", -1, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "upEng", upTank, 1);
    const int dec = addPart(bs, cat.decoupler(5), "dec", upTank, 2);
    const int loTank = addPart(bs, cat.tank(50, 75, 75), "loTank", dec, 2);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "loEng", loTank, 2);

    // Wet: 100+10+5+200+10 = 325. Dry: 50+10+5+50+10 = 125.
    // The ship's chemical engines burn H2+LOX, so both are the burnable pool
    // and a tank's dry mass is its structure alone.
    bool burn[(int)ResourceType::Num] = {};
    burn[(int)ResourceType::Hydrogen] = true;
    burn[(int)ResourceType::LOX] = true;
    CHECK_REL(partDryMass(*bs.parts[0].def, burn), 50.0, 1e-9, "upper dry");
    CHECK_REL(bs.parts[0].def->mass, 50.0, 1e-9, "upper catalog mass is dry structure");

    const double g = 10.0;
    const std::vector<StageRow> rows = computeStaging(bs, g);
    CHECK_TRUE(rows.size() == 2, "two rows");
    if(rows.size() != 2) { return; }

    // Stage 2 (lower): burns 150 kg of the 200-kg tank while the upper
    // is still attached. m0 = 325, m1 = 325 - 150 = 175 (the empty lower
    // tank dry + engines + upper still hang on until the drop).
    // dv = 1000 * ln(325/175).
    CHECK_TRUE(rows[0].stage == 2, "row0 is stage 2");
    CHECK_TRUE(rows[0].drops, "stage 2 drops");
    CHECK_REL(rows[0].massStart, 325.0, 1e-6, "s1 m0");
    CHECK_REL(rows[0].massEnd, 175.0, 1e-6, "s1 m1 (before drop)");
    CHECK_REL(rows[0].deltaV, 1000.0 * std::log(325.0 / 175.0), 1e-4, "s1 dv");

    // Stage 1 (upper): after dropping dec+lower (5+50+10 = 65 kg dry),
    // m0 = 175 - 65 = 110, burns 50 kg -> m1 = 60.
    // dv = 1000 * ln(110/60).
    CHECK_TRUE(rows[1].stage == 1, "row1 is stage 1");
    CHECK_TRUE(!rows[1].drops, "stage 1 does not drop");
    CHECK_REL(rows[1].massStart, 110.0, 1e-6, "s2 m0");
    CHECK_REL(rows[1].massEnd, 60.0, 1e-6, "s2 m1");
    CHECK_REL(rows[1].deltaV, 1000.0 * std::log(110.0 / 60.0), 1e-4, "s2 dv");

    // TWR against g = 10. Stage 1: T = 1000 N (one engine).
    // min at start: 1000 / (325 * 10); max at end: 1000 / (175 * 10).
    CHECK_REL(rows[0].minTWR, 1000.0 / (325.0 * g), 1e-6, "s1 minTWR");
    CHECK_REL(rows[0].maxTWR, 1000.0 / (175.0 * g), 1e-6, "s1 maxTWR");
    CHECK_TRUE(rows[0].minTWR < rows[0].maxTWR, "s1 min < max");
    CHECK_REL(rows[1].minTWR, 1000.0 / (110.0 * g), 1e-6, "s2 minTWR");
    CHECK_REL(rows[1].maxTWR, 1000.0 / (60.0 * g), 1e-6, "s2 maxTWR");
    CHECK_TRUE(rows[0].engines == 1 && rows[1].engines == 1, "one engine each");
}

// Single stage, no decouplers: one row, burns every drop of propellant.
static void test_single_stage() {
    Catalog cat;
    BuildShip bs;
    const int tank = addPart(bs, cat.tank(100, 50, 50), "tank", -1, 1);
    addPart(bs, cat.engine(20, 0.5, 2000.0), "eng", tank, 1);

    // wet = 200+20 = 220, dry = 120. dv = 2000 * ln(220/120).
    const std::vector<StageRow> rows = computeStaging(bs, 9.8);
    CHECK_TRUE(rows.size() == 1, "one row");
    if(rows.size() != 1) { return; }
    CHECK_TRUE(!rows[0].drops, "nothing to drop");
    CHECK_REL(rows[0].deltaV, 2000.0 * std::log(220.0 / 120.0), 1e-4, "dv");
    CHECK_REL(rows[0].massStart, 220.0, 1e-6, "m0");
    CHECK_REL(rows[0].massEnd, 120.0, 1e-6, "m1");
}

// Hydrazine / jet fuel / life support ride along as INERT mass (vacuum
// never burns them). Only H2+LOX is subtracted from the catalog's wet
// mass, so a mono tank must not vanish from the stack.
static void test_inert_resources() {
    Catalog cat;
    BuildShip bs;
    // full 200 + engine 20 = 220 as before, plus a mono tank (dry 10.45 +
    // 78.54 hydrazine = 88.99 full) whose hydrazine stays on the stack
    // the whole burn.
    const int tank = addPart(bs, cat.tank(100, 50, 50), "tank", -1, 1);
    addPart(bs, cat.engine(20, 0.5, 2000.0), "eng", tank, 1);
    addPart(bs, cat.mono(10.45, 78.54), "mono", tank, 1);

    // full = 220 + 88.99 = 308.99, burnable = 100, inert end = 208.99.
    const std::vector<StageRow> rows = computeStaging(bs, 9.8);
    CHECK_TRUE(rows.size() == 1, "inert: one row");
    if(rows.size() != 1) { return; }
    CHECK_REL(rows[0].massStart, 308.99, 1e-3, "inert m0");
    CHECK_REL(rows[0].massEnd, 208.99, 1e-3, "inert m1 (mono kept)");
    CHECK_REL(rows[0].deltaV, 2000.0 * std::log(308.99 / 208.99), 1e-3, "inert dv");
}

// A nuclear-thermal (H2-ONLY) engine on a tank that also holds LOX: no engine
// burns the LOX, so it is DEAD WEIGHT -- inert mass, not deliverable fuel.
// This is the shared-tank accounting issue #38 flagged: the burnable set is
// derived from the engines actually present, not assumed to be H2+LOX.
static void test_h2_only_engine() {
    Catalog cat;
    BuildShip bs;
    // Tank: dry 100 + H2 50 + LOX 50. Nuclear engine: dry 20, H2 0.5 kg/s,
    // ve 2000 -> thrust 1000 N, mdot 0.5 (H2 only).
    const int tank = addPart(bs, cat.tank(100, 50, 50), "tank", -1, 1);
    addPart(bs, cat.nuclearEngine(20, 0.5, 2000.0), "eng", tank, 1);

    // The engine draws H2 only, so the burnable mask is {H2}: the tank's
    // deliverable propellant is its 50 kg of H2, and its 50 kg of LOX folds
    // into the dry/inert mass.
    bool burn[(int)ResourceType::Num] = {};
    burn[(int)ResourceType::Hydrogen] = true;
    CHECK_REL(partPropellantMass(*bs.parts[tank].def, burn), 50.0, 1e-9,
               "h2-only: tank burnable = H2 only (the LOX is inert)");
    CHECK_REL(partDryMass(*bs.parts[tank].def, burn), 150.0, 1e-9,
               "h2-only: tank dry = structure + the inert LOX");

    // Wet 220, burnable 50 (H2), so the burn ends at 170 (the LOX rides
    // along). dv = ve * ln(220/170) -- LESS than a chemical engine would get
    // from the same tank (it could burn the LOX too), which is the point.
    const std::vector<StageRow> rows = computeStaging(bs, 10.0);
    CHECK_TRUE(rows.size() == 1, "h2-only: one row");
    if(rows.size() != 1) { return; }
    CHECK_REL(rows[0].massStart, 220.0, 1e-6, "h2-only m0");
    CHECK_REL(rows[0].massEnd, 170.0, 1e-6, "h2-only m1 (the LOX is kept)");
    CHECK_REL(rows[0].deltaV, 2000.0 * std::log(220.0 / 170.0), 1e-3, "h2-only dv");
}

// Payload separator: a decoupler subtree with NO propellant. Without the
// drainAll fallback the period would end at zero length and swallow the
// upper stage's burn.
static void test_inert_drop() {
    Catalog cat;
    BuildShip bs;
    // Upper: tank (stage 1, fires last) + engine (stage 2). Then an inert
    // sep (stage 2) with a dummy block below it. Lower booster on stage 3
    // (fires first) with its own engine.
    const int upTank = addPart(bs, cat.tank(50, 25, 25), "upTank", -1, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "upEng", upTank, 2);
    const int sep = addPart(bs, cat.decoupler(5), "sep", upTank, 2);
    addPart(bs, cat.tank(20, 0, 0), "dummy", sep, 2);
    const int dec = addPart(bs, cat.decoupler(5), "dec", upTank, 3);
    const int loTank = addPart(bs, cat.tank(50, 75, 75), "loTank", dec, 3);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "loEng", loTank, 3);

    // wet = 100+10+5+20+5+200+10 = 350. Stage 3 (the highest) lights the
    // lower engine (the upper is stage 2): burns the lower's 150 kg,
    // m 350 -> 200. Drop 65 -> 135. Stage 2 lights the upper, drops the
    // inert sep, and must still burn the upper's 50 kg: 135 -> 85.
    const std::vector<StageRow> rows = computeStaging(bs, 10.0);
    CHECK_TRUE(rows.size() >= 2, "inert-drop: at least two rows");
    if(rows.size() < 2) { return; }
    CHECK_REL(rows[0].deltaV, 1000.0 * std::log(350.0 / 200.0), 1e-3, "id s1 dv");
    // The upper burn must still happen even though the sep holds no
    // propellant (the drainAll fallback).
    CHECK_TRUE(rows[1].deltaV > 100.0, "inert-drop: upper burn survives");
    CHECK_REL(rows[1].deltaV, 1000.0 * std::log(135.0 / 85.0), 1e-3, "id s2 dv");
    CHECK_TRUE(rows[1].drops, "id s2 still records the sep drop");
}

// g = 0: TWR is undefined/skipped but vacuum delta-v must still appear.
static void test_zero_g() {
    Catalog cat;
    BuildShip bs;
    const int tank = addPart(bs, cat.tank(100, 50, 50), "tank", -1, 1);
    addPart(bs, cat.engine(20, 0.5, 2000.0), "eng", tank, 1);
    const std::vector<StageRow> rows = computeStaging(bs, 0.0);
    CHECK_TRUE(rows.size() == 1, "zero-g: one row");
    if(rows.size() != 1) { return; }
    CHECK_REL(rows[0].deltaV, 2000.0 * std::log(220.0 / 120.0), 1e-4, "zero-g dv");
    CHECK_TRUE(rows[0].minTWR == 0.0 && rows[0].maxTWR == 0.0, "zero-g TWR is 0");
}

// Asparagus pair: outer booster feeds the core (fuel link outer -> core).
// Both engines light at launch; the outer drop is on the same (highest)
// stage. The outer tanks empty FIRST (they sit in the core engine's
// furthest drain layer and also feed the outer engine), then the booster
// drops, then the core keeps burning its own tank (stage 1 / final).
//
//   coreTank (stage 1) -- coreEng (stage 2)
//   dec (stage 2) -- outerTank (stage 2) -- outerEng (stage 2)
//   link: outerTank -> coreTank
//
// Both engines sit on stage 2 (the highest, so they fire together from
// launch through the asparagus phase); the core tank is stage 1 (final).
static void test_asparagus() {
    Catalog cat;
    BuildShip bs;
    const int coreTank = addPart(bs, cat.tank(50, 50, 50), "coreTank", -1, 1);
    // Core engine on stage 2: lights at launch beside the booster.
    addPart(bs, cat.engine(10, 0.5, 1000.0), "coreEng", coreTank, 2);
    const int dec = addPart(bs, cat.decoupler(5), "dec", coreTank, 2);
    const int outTank = addPart(bs, cat.tank(50, 50, 50), "outTank", dec, 2);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "outEng", outTank, 2);
    addLink(bs, "outTank", "coreTank");   // outer feeds core (asparagus)

    // wet: coreTank 150 + coreEng 10 + dec 5 + outTank 150 + outEng 10 = 325
    // Both engines: T = 2000 N, mdot = 2 kg/s together.
    // Drain layers of the core engine: [out], [core] -- outer first.
    // Drain layers of the outer engine: [out] only.
    // Combined they empty OUTER first at 2 kg/s. Outer has 100 kg fuel.
    // Stage 2 (the booster stage) stops when outer is empty of drainable
    // fuel: 100 kg burned in 50 s. Mass falls 325 -> 225 (both dry cores
    // remain). Drop outer: 50 dry + 10 eng + 5 dec = 65. Remaining = 160.
    // Stage 1 (final): core engine burns the core 100 kg. 160 -> 60.
    const double g = 10.0;
    const std::vector<StageRow> rows = computeStaging(bs, g);
    CHECK_TRUE(rows.size() == 2, "asparagus: two rows");
    if(rows.size() != 2) { return; }

    CHECK_TRUE(rows[0].drops, "stage 1 drops the booster");
    CHECK_REL(rows[0].massStart, 325.0, 1e-6, "asp m0");
    CHECK_REL(rows[0].massEnd, 225.0, 1e-6, "asp m1 (outer empty, still attached)");
    // dv while BOTH engines fire and the outer empties:
    // F = 2000, mdot = 2, ve_eq = 1000. dv = 1000 * ln(325/225).
    CHECK_REL(rows[0].deltaV, 1000.0 * std::log(325.0 / 225.0), 1e-3, "asp s1 dv");

    CHECK_TRUE(!rows[1].drops, "final does not drop");
    CHECK_REL(rows[1].massStart, 160.0, 1e-6, "asp s2 m0");
    CHECK_REL(rows[1].massEnd, 60.0, 1e-6, "asp s2 m1");
    // Only the core engine remains: T = 1000, mdot = 1, ve = 1000.
    CHECK_REL(rows[1].deltaV, 1000.0 * std::log(160.0 / 60.0), 1e-3, "asp s2 dv");

    // Stage-1 TWR is the pair (T = 2000) at 325 .. 225 kg on g = 10.
    CHECK_REL(rows[0].minTWR, 2000.0 / (325.0 * g), 1e-6, "asp s1 minTWR");
    CHECK_REL(rows[0].maxTWR, 2000.0 / (225.0 * g), 1e-6, "asp s1 maxTWR");
    // After the drop only the core engine burns (T = 1000).
    CHECK_REL(rows[1].minTWR, 1000.0 / (160.0 * g), 1e-6, "asp s2 minTWR");
    CHECK_REL(rows[1].maxTWR, 1000.0 / (60.0 * g), 1e-6, "asp s2 maxTWR");
}

// Decoupler is a fuel barrier: WITHOUT the link the core engine cannot
// see the outer tank. Each engine burns only its own group, so the outer
// empties on its own (the "drop when spent" clock) while the core keeps
// a reserve for the final stage.
static void test_barrier_no_link() {
    Catalog cat;
    BuildShip bs;
    // Core carries a LARGER tank (150 kg fuel) so it outlasts the outer.
    const int coreTank = addPart(bs, cat.tank(50, 75, 75), "coreTank", -1, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "coreEng", coreTank, 2);
    const int dec = addPart(bs, cat.decoupler(5), "dec", coreTank, 2);
    const int outTank = addPart(bs, cat.tank(50, 50, 50), "outTank", dec, 2);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "outEng", outTank, 2);

    const double g = 10.0;
    const std::vector<StageRow> rows = computeStaging(bs, g);
    CHECK_TRUE(rows.size() == 2, "barrier: two rows");
    if(rows.size() != 2) { return; }

    // wet = 200+10+5+150+10 = 375. Outer engine drains only [out] (100
    // kg) at 1 kg/s; core only [core] (150 kg) at 1 kg/s. Stage 1 stops
    // when the OUTER is empty (100 s): 200 kg burned total, m = 175.
    // Drop 65 -> 110 left (core dry 50 + 50 fuel + engine 10).
    CHECK_TRUE(rows[0].drops, "barrier s1 drops");
    CHECK_REL(rows[0].massStart, 375.0, 1e-6, "barrier s1 m0");
    CHECK_REL(rows[0].massEnd, 175.0, 1e-6, "barrier s1 m1");
    // Two engines, mdot = 2, F = 2000, ve_eq = 1000.
    CHECK_REL(rows[0].deltaV, 1000.0 * std::log(375.0 / 175.0), 1e-3, "barrier s1 dv");
    // Core kept 50 kg because the barrier stopped the outer from feeding it.
    CHECK_REL(rows[1].massStart, 110.0, 1e-6, "barrier s2 m0");
    CHECK_REL(rows[1].massEnd, 60.0, 1e-6, "barrier s2 m1");
    CHECK_REL(rows[1].deltaV, 1000.0 * std::log(110.0 / 60.0), 1e-3, "barrier s2 dv");
}

// Exhaust-velocity difficulty scale: thrust (so ve_eq = F/mdot) and TWR
// scale by it, the fuel burn / mass schedule does not -- matching flight
// (Vehicle::exhaust_scale).
static void test_exhaust_scale() {
    Catalog cat;
    BuildShip bs;
    const int tank = addPart(bs, cat.tank(50, 75, 75), "tank", -1, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "eng", tank, 1);

    const double g = 10.0;
    const std::vector<StageRow> a = computeStaging(bs, g, 1.0);
    const std::vector<StageRow> b = computeStaging(bs, g, 2.0);
    CHECK_TRUE(a.size() == 1 && b.size() == 1, "scale: one row each");
    if(a.size() != 1 || b.size() != 1) { return; }

    // Same mass schedule (the burn does not scale).
    CHECK_REL(b[0].massStart, a[0].massStart, 1e-9, "scale m0 unchanged");
    CHECK_REL(b[0].massEnd, a[0].massEnd, 1e-9, "scale m1 unchanged");
    // ve_eq and TWR double; delta-v is ve_eq * ln(m0/m1), so it doubles too.
    CHECK_REL(b[0].deltaV, 2.0 * a[0].deltaV, 1e-6, "scale dv doubles");
    CHECK_REL(b[0].minTWR, 2.0 * a[0].minTWR, 1e-6, "scale minTWR doubles");
    CHECK_REL(b[0].maxTWR, 2.0 * a[0].maxTWR, 1e-6, "scale maxTWR doubles");
}

/* stagingSignature: the Staging window caches its rows against this hash
   (#293), so EVERY field computeStaging reads has to move it. A field the hash
   misses is a frame where the player edits the ship and the table keeps
   showing the old numbers. */
static bool rowsDiffer(const std::vector<StageRow> &a, const std::vector<StageRow> &b) {
    if(a.size() != b.size()) { return true; }
    for(size_t i = 0; i < a.size(); i++) {
        if(a[i].stage != b[i].stage || a[i].drops != b[i].drops
           || a[i].deltaV != b[i].deltaV || a[i].minTWR != b[i].minTWR
           || a[i].maxTWR != b[i].maxTWR || a[i].massStart != b[i].massStart
           || a[i].massEnd != b[i].massEnd || a[i].engines != b[i].engines) {
            return true;
        }
    }
    return false;
}

static void test_signature() {
    Catalog cat;
    BuildShip bs;
    addPart(bs, cat.tank(500.0, 100.0, 600.0), "cap", -1, 1);
    addPart(bs, cat.engine(1000.0, 5.0, 3200.0), "eng", 0, 1);
    addPart(bs, cat.decoupler(100.0), "dec", 0, 2);
    addPart(bs, cat.tank(1000.0, 200.0, 1200.0), "tank", 2, 2);
    addLink(bs, "tank", "eng");

    const uint64_t base = stagingSignature(bs, 9.81, 1.0);
    CHECK_TRUE(stagingSignature(bs, 9.81, 1.0) == base, "stable build: the cache hits");

    // The two scalars.
    CHECK_TRUE(stagingSignature(bs, 1.62, 1.0) != base, "a different launch body");
    CHECK_TRUE(stagingSignature(bs, 9.81, 2.0) != base, "a different exhaust scale");
    // computeStaging clamps a non-positive scale to 1.0, so the cache must too.
    CHECK_TRUE(stagingSignature(bs, 9.81, 0.0) == base, "scale 0 clamps like computeStaging");
    CHECK_TRUE(stagingSignature(bs, 9.81, -3.0) == base, "negative scale clamps too");

    // The stage number: what the table is keyed by.
    bs.parts[3].stage = 1;
    CHECK_TRUE(stagingSignature(bs, 9.81, 1.0) != base, "a stage number moved");
    bs.parts[3].stage = 2;
    CHECK_TRUE(stagingSignature(bs, 9.81, 1.0) == base, "and back: the cache hits again");

    // The tree: a re-parent is a different ship even with the same parts.
    const int p = bs.parts[3].parent;
    bs.parts[3].parent = 1;
    CHECK_TRUE(stagingSignature(bs, 9.81, 1.0) != base, "a re-parented part");
    bs.parts[3].parent = p;

    // The part identity: swapping a tank for an engine keeps the part count.
    const PartDef *tank = bs.parts[3].def;
    bs.parts[3].def = cat.engine(1000.0, 5.0, 3200.0);
    CHECK_TRUE(stagingSignature(bs, 9.81, 1.0) != base, "a part swapped for another def");
    bs.parts[3].def = tank;

    // A fuel link added, and one removed: asparagus is the whole point.
    BuildShip withExtra = bs;
    addLink(withExtra, "cap", "eng");
    CHECK_TRUE(stagingSignature(withExtra, 9.81, 1.0) != base, "a fuel link added");
    BuildShip noLink = bs;
    noLink.fuelLinks.clear();
    CHECK_TRUE(stagingSignature(noLink, 9.81, 1.0) != base, "a fuel link deleted");

    // A part placed or deleted.
    BuildShip bigger = bs;
    addPart(bigger, cat.tank(1000.0, 200.0, 1200.0), "extra", 0, 1);
    CHECK_TRUE(stagingSignature(bigger, 9.81, 1.0) != base, "a part placed");
    BuildShip smaller = bs;
    smaller.parts.pop_back();
    CHECK_TRUE(stagingSignature(smaller, 9.81, 1.0) != base, "a part deleted");

    /* A link's OWN id is not hashed, on purpose: computeStaging resolves a link
       through its endpoints and never reads fl.id, so renaming one cannot
       change a row. Pinned so the hash is not "fixed" into over-invalidating. */
    BuildShip renamed = bs;
    renamed.fuelLinks[0].id = "renamed_link";
    CHECK_TRUE(stagingSignature(renamed, 9.81, 1.0) == base, "a link rename changes no row");

    // A repointed link: same part count, same link count, so this can only be
    // caught by hashing the endpoints themselves.
    BuildShip repointed = bs;
    repointed.fuelLinks[0].to = "cap";
    CHECK_TRUE(stagingSignature(repointed, 9.81, 1.0) != base, "a link repointed, counts equal");

    /* The contract the cache actually rests on is not "different build,
       different hash" but "different ROWS, different hash". This pair is the
       shape that broke it once: identical part order, defs, parents and stages,
       identical link strings -- only the ids of two same-def tanks are swapped,
       and because computeStaging resolves a link by looking its endpoint ids up
       among the parts, that moves the link across the decoupler. Loading one
       over the other has to refresh the table.

       The decoupler has to sit on the SAME stage as the engine: put it on a
       later stage and its branch is gone before anything burns, the link stops
       mattering, and the pair silently stops testing anything (it did once). */
    {
        BuildShip a;
        addPart(a, cat.tank(500.0, 100.0, 600.0), "cap", -1, 1);
        addPart(a, cat.engine(1000.0, 5.0, 3200.0), "eng", 0, 1);
        addPart(a, cat.decoupler(100.0), "dec", 0, 1);
        addPart(a, cat.tank(1000.0, 200.0, 1200.0), "t1", 2, 1);   // behind the wall
        addPart(a, cat.tank(1000.0, 200.0, 1200.0), "t2", 1, 1);   // with the engine
        addLink(a, "t1", "eng");
        BuildShip b = a;
        b.parts[3].id = "t2";
        b.parts[4].id = "t1";

        const std::vector<StageRow> ra = computeStaging(a, 9.81, 1.0);
        const std::vector<StageRow> rb = computeStaging(b, 9.81, 1.0);
        CHECK_TRUE(rowsDiffer(ra, rb), "the id swap does move the numbers");
        CHECK_TRUE(stagingSignature(a, 9.81, 1.0) != stagingSignature(b, 9.81, 1.0),
                   "different rows, so the hash must differ");
    }
}

/* Stage numbers are LABELS, not a count of things to walk over (#517). Numbering the
   same ship 1 and 9000 used to cost 9000 passes over the part list -- one drop walk
   per index -- on the way to exactly the same rows, so the pass billed the SPAN
   between the highest and lowest label instead of the number of
   labels. These checks hold the cheap walk to the same computation: every row matches
   except the label, and the high label still fires FIRST even when the parts arrive in
   the opposite order. */
static BuildShip numbered_ship(Catalog &cat, int upper, int lower) {
    BuildShip bs;
    const int upTank = addPart(bs, cat.tank(50, 25, 25), "upTank", -1, upper);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "upEng", upTank, upper);
    const int dec = addPart(bs, cat.decoupler(5), "dec", upTank, lower);
    const int loTank = addPart(bs, cat.tank(50, 75, 75), "loTank", dec, lower);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "loEng", loTank, lower);
    return bs;
}

/* Three groups down one spine, and BOTH separations carry a label -- so the rows
   depend on which label the walk reaches first. The top group is added first (a
   BuildShip needs every parent earlier), which puts the parts in DESCENDING label
   order: a walk that is not sorted visits the bottom label first and burns the whole
   ship before the separation that should have preceded it. */
static BuildShip stacked_ship(Catalog &cat, int top, int bottom) {
    BuildShip bs;
    const int upTank = addPart(bs, cat.tank(50, 25, 25), "upTank", -1, top);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "upEng", upTank, top);
    const int dec1 = addPart(bs, cat.decoupler(5), "dec1", upTank, top);
    const int midTank = addPart(bs, cat.tank(50, 50, 50), "midTank", dec1, bottom);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "midEng", midTank, bottom);
    const int dec2 = addPart(bs, cat.decoupler(5), "dec2", midTank, bottom);
    addPart(bs, cat.tank(10, 0, 0), "payload", dec2, bottom);
    return bs;
}

// Field-by-field except the label, which the caller checks separately.
static void same_rows(const StageRow &a, const StageRow &b) {
    CHECK_REL(a.deltaV, b.deltaV, 1e-9, "same delta-v");
    CHECK_REL(a.massStart, b.massStart, 1e-9, "same mass start");
    CHECK_REL(a.massEnd, b.massEnd, 1e-9, "same mass end");
    CHECK_REL(a.minTWR, b.minTWR, 1e-9, "same min TWR");
    CHECK_REL(a.maxTWR, b.maxTWR, 1e-9, "same max TWR");
    CHECK_TRUE(a.drops == b.drops, "same separation");
    CHECK_TRUE(a.engines == b.engines, "same lit engine count");
}

static void test_stage_labels_are_labels() {
    const double g = 10.0;
    Catalog cat;
    BuildShip a = numbered_ship(cat, 1, 2);
    BuildShip b = numbered_ship(cat, 1, 9000);

    const std::vector<StageRow> ra = computeStaging(a, g);
    const std::vector<StageRow> rb = computeStaging(b, g);
    CHECK_TRUE(ra.size() == 2 && rb.size() == 2, "two rows either way");
    if(ra.size() != 2 || rb.size() != 2) { return; }

    CHECK_TRUE(ra[0].stage == 2 && rb[0].stage == 9000, "the high label fires first");
    CHECK_TRUE(ra[1].stage == 1 && rb[1].stage == 1, "the low label fires last");
    for(size_t i = 0; i < ra.size(); i++) { same_rows(ra[i], rb[i]); }

    // Same physical ship, but the parts now arrive in DESCENDING label order, so the
    // vector the walk is built from is unsorted-with-respect-to-labels. Both separations
    // carry a label here, so an unsorted walk burns the lower stage first and both rows
    // come out wrong -- which is what pins the sort.
    BuildShip c = stacked_ship(cat, 2, 1);
    BuildShip d = stacked_ship(cat, 9000, 1);
    const std::vector<StageRow> rc = computeStaging(c, g);
    const std::vector<StageRow> rd = computeStaging(d, g);
    CHECK_TRUE(rc.size() == 2 && rd.size() == 2, "two separations, two rows");
    if(rc.size() != 2 || rd.size() != 2) { return; }
    CHECK_TRUE(rc[0].stage == 2 && rd[0].stage == 9000, "the top label still fires first");
    CHECK_TRUE(rc[1].stage == 1 && rd[1].stage == 1, "and the bottom label last");
    CHECK_TRUE(rd[0].deltaV > 0.0, "the top stage does burn something");
    for(size_t i = 0; i < rc.size(); i++) { same_rows(rc[i], rd[i]); }

    // A one-stage ship whose single label is huge: the last row is emitted even where
    // nothing separates, and that must not depend on the label being small.
    BuildShip s;
    const int t = addPart(s, cat.tank(50, 50, 0), "tank", -1, 4000);
    addPart(s, cat.engine(10, 0.5, 1000.0), "eng", t, 4000);
    const std::vector<StageRow> rs = computeStaging(s, g);
    CHECK_TRUE(rs.size() == 1, "a huge label still yields exactly one row");
    if(!rs.empty()) { CHECK_TRUE(rs[0].stage == 4000, "and keeps it"); }
}

/* What the walk is allowed to COST. Rows identical is not enough to keep #517: the old
   per-integer loop passes every check above (bit-identical rows), so nothing else in
   the suite can see the difference. Here a 2-part ship is numbered 1 and 1000000:
   walked sparse it takes ~0.5 us, walked per-integer it measured ~25 ms. The bound of
   5 ms therefore sits four orders above what the work costs and five below what the
   old shape cost -- a complexity pin, not a benchmark, so it stays binary on a loaded
   machine. The number is deliberately beyond kMaxStage: the loaders refuse that, but
   the pass must not care, and the margin is the point. */
static void test_a_gap_costs_nothing() {
    Catalog cat;
    BuildShip bs;
    const int t = addPart(bs, cat.tank(50, 100, 100), "tank", -1, 1000000);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "eng", t, 1);
    computeStaging(bs, 9.81, 1.0);   // warm, so this measures the walk and not the allocator

    const auto t0 = std::chrono::steady_clock::now();
    const std::vector<StageRow> rows = computeStaging(bs, 9.81, 1.0);
    const double us = std::chrono::duration<double, std::micro>(
                          std::chrono::steady_clock::now() - t0).count();
    CHECK_TRUE(rows.size() == 1 && rows[0].stage == 1, "the gap still gives one row");
    printf("  a 1000000-label gap: %.1f us\n", us);
    CHECK_TRUE(us < 5000.0, "a label gap must not cost work per skipped integer");
}

/* The label-dense shape the speed probes never had (#539). Every probe of the preview was
   a spine or an asparagus bundle, and all of them carry a handful of stage labels no
   matter how many parts they hold -- which is how 3fdbf4a (#538) merged looking faster on
   every number in its own table while making a whole class of ship 13-35% slower. Here
   every tank carries its OWN label (what the stage box invites: 1..kMaxStage) and every
   decoupler sits on one shared label, so the per-label work is the same order as the part
   count instead of a constant. Synthetic on purpose: the busiest ship in res/ships/
   carries 7 labels over 51 parts, so nothing shipped looks like this -- it is
   the shape the stage box (1..kMaxStage, one label per part) invites an author to build,
   which is the population a preview cost claim has to survive. Its fuel graph is
   heavy_asp's chain (each string feeds the one inboard) with one extra link per tank;
   only the chain crosses a decoupler barrier, the rest are same-group and fuelDrainLayers
   drops them, so the printed `links=` counts more edges than the pass walks. */
static BuildShip label_dense(const PartsCatalog &cat, int strings, int depth) {
    const PartDef *capsule = cat.find("capsule");
    const PartDef *tankL = cat.find("tank_r2.25h5");
    const PartDef *engL = cat.find("engine_r2.25h4.5");
    const PartDef *dec = cat.find("decoupler_r1.5");
    CHECK_TRUE(capsule && tankL && engL && dec, "catalog defs for the label-dense fixture");
    if(!capsule || !tankL || !engL || !dec) { return BuildShip(); }
    BuildShip bs;
    const int hiLabel = 3 + strings * depth;
    int core = addPart(bs, capsule, "capsule_0000", -1, 1);
    core = addPart(bs, tankL, "tank_core_0000", core, 1);
    addPart(bs, engL, "engine_core_0000", core, hiLabel);
    std::string prevTank = "tank_core_0000";
    char id[40];
    for(int s = 0; s < strings; s++) {
        snprintf(id, sizeof(id), "dec_%04d", s);
        int p = addPart(bs, dec, id, core, 2);                  // one separation label
        for(int k = 0; k < depth; k++) {
            snprintf(id, sizeof(id), "tank_%04d_%02d", s, k);
            p = addPart(bs, tankL, id, p, 3 + s * depth + k);   // every tank its own label
            addLink(bs, id, prevTank);
            prevTank = id;
        }
        snprintf(id, sizeof(id), "engine_%04d", s);
        addPart(bs, engL, id, p, hiLabel);
    }
    return bs;
}

/* Cost, not just correctness: the shape above exists so a per-label hoist cannot merge on
   data where the label count is a constant. Two sizes, because one number cannot show a
   trend. Measured here (one warm pass each, this machine): 171 parts / 147 labels is
   ~100 us and 339 parts / 291 labels is ~290 us -- doubling the parts costs ~3x, so the
   pass is ALREADY superlinear in labels and this guard does not pretend otherwise.

   What the ceilings can see is a gross blowup, and that is all. Each sits ~30x above what
   the shape costs today, and that margin is not caution -- it is what a wall-clock pin
   needs on a box this noisy: three back-to-back runs of this same binary read 95-100 us at
   12x12 and 284-475 us at 24x12, a 1.7x spread with nothing else running. Anything inside
   it is invisible here. A label walk made quadratic in the LABEL count (mutant, measured
   twice: 107.0/107.8 us and 332.1/332.4 us) sits inside that spread -- squarely in the
   13-35% band this shape was built for -- and both ceilings stay green. So this is a
   tripwire for "the pass started doing per-label-quadratic work and then WORSE", not a
   regression detector. The pin that could see 30% counts WORK (labels stamped vs
   drain-layer lookups) instead of seconds; that needs a counter inside the pass, so it is
   a ticket (#549) rather than a scratch probe nobody re-runs. */
static void test_a_label_dense_ship_is_still_cheap_to_number() {
    const PartsCatalog pc = load_parts_catalog("res/data/parts.json");
    const PartDef *engL = pc.find("engine_r2.25h4.5");
    if(!engL) { return; }
    const int shapes[][2] = {{12, 12}, {24, 12}};
    const double ceilings[] = {3000.0, 9000.0};
    for(size_t sh = 0; sh < 2; sh++) {
        const BuildShip bs = label_dense(pc, shapes[sh][0], shapes[sh][1]);
        CHECK_TRUE(!bs.parts.empty(), "label-dense fixture built");
        const std::vector<StageRow> warm = computeStaging(bs, 9.81, 1.0);

        std::vector<int> labels;
        for(size_t i = 0; i < bs.parts.size(); i++) { labels.push_back(bs.parts[i].stage); }
        std::sort(labels.begin(), labels.end());
        labels.erase(std::unique(labels.begin(), labels.end()), labels.end());

        const auto t0 = std::chrono::steady_clock::now();
        const std::vector<StageRow> rows = computeStaging(bs, 9.81, 1.0);
        const double us = std::chrono::duration<double, std::micro>(
                              std::chrono::steady_clock::now() - t0).count();
        printf("  label-dense %2dx%-2d: parts=%4zu labels=%4zu links=%4zu rows=%3zu  %7.1f us (ceiling %.0f)\n",
               shapes[sh][0], shapes[sh][1], bs.parts.size(), labels.size(),
               bs.fuelLinks.size(), rows.size(), us, ceilings[sh]);
        CHECK_TRUE(labels.size() * 4 >= bs.parts.size() * 3,
                   "the fixture really is label-dense (labels within 4:3 of parts)");
        CHECK_TRUE(bs.fuelLinks.size() == (size_t)(shapes[sh][0] * shapes[sh][1]),
                   "the fixture kept its drain graph (one link per string tank)");

        /* Teeth. A timing guard whose rows are never read cannot fail for the right
           reason: with every row's deltaV zeroed and massEnd == massStart, this test was
           the ONLY one in the file that stayed green, and gutting the fixture's links
           dropped its cost ~40% with nothing red anywhere. So the rows are pinned with
           the same invariants test_deep_chain_burn_stays_physical uses -- same single ve
           here (one engine def), same reachable-through-a-link graph -- and the warm pass
           earns its keep by matching the timed pass field by field, not just in size. */
        CHECK_TRUE(engL->exhaust_velocity > 0.0, "the label-dense engine has an exhaust velocity");
        bool burn[(int)ResourceType::Num] = {};
        burn[(int)ResourceType::Hydrogen] = true;
        burn[(int)ResourceType::LOX] = true;
        double fuel = 0.0;
        for(size_t i = 0; i < bs.parts.size(); i++) { fuel += partPropellantMass(*bs.parts[i].def, burn); }
        const double ve = engL->exhaust_velocity;
        double spent = 0.0;
        for(size_t i = 0; i < rows.size(); i++) {
            const StageRow &r = rows[i];
            CHECK_TRUE(std::isfinite(r.deltaV), "label-dense delta-v is finite");
            CHECK_TRUE(std::isfinite(r.massStart) && std::isfinite(r.massEnd)
                       && r.massEnd > 0.0 && r.massStart >= r.massEnd,
                       "label-dense mass columns are finite and shrink");
            CHECK_REL(r.deltaV, ve * std::log(r.massStart / r.massEnd), 1e-9,
                       "label-dense delta-v matches ve ln(m0/m1)");
            spent += r.massStart - r.massEnd;
        }
        CHECK_REL(spent, fuel, 1e-9, "the label-dense rows spend every kg of propellant once");
        bool same = rows.size() == warm.size() && !rows.empty();
        for(size_t i = 0; same && i < rows.size(); i++) {
            same = rows[i].stage == warm[i].stage && rows[i].deltaV == warm[i].deltaV
                && rows[i].massStart == warm[i].massStart && rows[i].massEnd == warm[i].massEnd
                && rows[i].engines == warm[i].engines && rows[i].drops == warm[i].drops;
        }
        CHECK_TRUE(same, "a second pass reads the same rows, not just the same count");
        CHECK_TRUE(us < ceilings[sh],
                   "the label-dense pass stayed inside its complexity guard");
    }
}

/* heavy_asp's shape, scaled: a core stack (capsule, wheel, big tank, big engine)
   plus `boosters` side strings of `stringLen` parts (decoupler, tanks, engine),
   each string's innermost tank fuel-linked to the string inboard of it. Every
   engine burns H2+LOX at the same exhaust velocity, and the strings share three
   stage labels, like the shipped ship. Shipped part defs because the burn's
   step count and its rounding both matter here (#526): this is the shape the VAB
   produces and the numbers the player reads. */
static BuildShip shipped_asparagus(const PartsCatalog &cat, int boosters, int stringLen) {
    const PartDef *capsule = cat.find("capsule");
    const PartDef *wheel = cat.find("reaction_wheel");
    const PartDef *tankS = cat.find("tank_r1.5h3");
    const PartDef *tankL = cat.find("tank_r2.25h5");
    const PartDef *engS = cat.find("engine_r1.5h3");
    const PartDef *engL = cat.find("engine_r2.25h4.5");
    const PartDef *dec = cat.find("decoupler_r1.5");
    CHECK_TRUE(capsule && wheel && tankS && tankL && engS && engL && dec,
               "catalog defs for the asparagus fixture");
    if(!capsule || !wheel || !tankS || !tankL || !engS || !engL || !dec) { return BuildShip(); }

    BuildShip bs;
    const int hiLabel = 2 + boosters;   // every engine fires on the top label
    int core = addPart(bs, capsule, "capsule_0000", -1, 1);
    core = addPart(bs, wheel, "reaction_wheel_0000", core, 1);
    core = addPart(bs, tankL, "tank_core_0000", core, 1);
    addPart(bs, engL, "engine_core_0000", core, hiLabel);
    std::string inboard = "tank_core_0000";
    for(int b = 0; b < boosters; b++) {
        char id[40];
        const int label = 2 + (b % 3);   // three separations, like heavy_asp
        snprintf(id, sizeof(id), "dec_%04d", b);
        int p = addPart(bs, dec, id, core, label);
        std::string innermost;
        for(int k = 0; k < stringLen - 1; k++) {
            snprintf(id, sizeof(id), "tank_%04d_%02d", b, k);
            p = addPart(bs, k == stringLen - 2 ? tankS : tankL, id, p, label);
            innermost = id;
        }
        snprintf(id, sizeof(id), "engine_%04d", b);
        addPart(bs, engS, id, p, hiLabel);
        addLink(bs, innermost, inboard);   // this string feeds the one inboard
        inboard = innermost;
    }
    return bs;
}

/* Two invariants runBurn was free to break, because it kept the tanks
   (sim.parts[i].fuel) and the running mass in the Tsiolkovsky sum (acc.m) as two
   separate representations with nothing forcing them together (#526):

     * a row's delta-v is its exhaust velocity times ln(massStart/massEnd) -- all
       these engines share one ve, and PartDef::fullThrust() is ve * mdot, so
       F/mdot is ve every step and the step ratios telescope;
     * the rows together spend the ship's whole propellant load, once -- a period
       that stops early leaves fuel in a stage that then separates. That one is
       shape-dependent, so the fixture keeps the shape simple: no decoupler sits on
       the LOWEST label (#529: such a ship's walk ends at the separation and the
       survivor's burn never gets a row) and every tank is reachable through a link
       (res/ships/central_radial.json is the shipped counter-example -- a fuel wall
       makes its booster's tanks unreachable, and jettisoning them full is right).

   At HEAD a 6-boosters build (70 parts) read dv 1,801,269 m/s for the middle
   stage and a 12-boosters build read -nan, and both left tens of tonnes of
   propellant unaccounted. The step budget that used to end those burns silently
   is now asserted against, so a cap hit fails this test in a debug build. */
static void test_deep_chain_burn_stays_physical() {
    const double g = 9.81;
    const PartsCatalog pc = load_parts_catalog("res/data/parts.json");
    const PartDef *engS = pc.find("engine_r1.5h3");
    const PartDef *engL = pc.find("engine_r2.25h4.5");
    if(!engS || !engL) { return; }
    CHECK_TRUE(engS->exhaust_velocity == engL->exhaust_velocity,
               "fixture engines share an exhaust velocity");
    const double ve = engS->exhaust_velocity;

    // What computeStaging treats as burnable: the resources its rocket engines draw.
    bool burn[(int)ResourceType::Num] = {};
    burn[(int)ResourceType::Hydrogen] = true;
    burn[(int)ResourceType::LOX] = true;

    /* Which rung covers what -- the earlier version of this note had the two bugs
       swapped. The old bug, skipping near-empty tanks when picking dt, is caught at
       6, 12, 16 AND 24; only the 4-booster rung is clean. At 12 it reads dv 3931.73
       where the fix reads 3170.24, then a -nan row, and spends 884,889 kg of a
       1,059,751 kg load. What genuinely needs the deep rungs is the OTHER one: drop
       the exact-empty rule and 4, 6 and 12 pass it untouched -- only 16 and 24
       report the shortfall (972,320 of 1,409,476, and 1,671,770 of 2,108,926). So a
       rung >= 16 is the sole cover for the tie rule: do not shorten the ladder. */
    for(int boosters : {4, 6, 12, 16, 24}) {
        const std::string tag = std::to_string(boosters) + "-boosters: ";
        BuildShip bs = shipped_asparagus(pc, boosters, 10);
        CHECK_TRUE(!bs.parts.empty(), (tag + "fixture built").c_str());

        double fuel = 0.0;
        for(size_t i = 0; i < bs.parts.size(); i++) {
            fuel += partPropellantMass(*bs.parts[i].def, burn);
        }

        const std::vector<StageRow> rows = computeStaging(bs, g);
        CHECK_TRUE(rows.size() == 4, (tag + "four rows").c_str());
        double spent = 0.0;
        for(size_t i = 0; i < rows.size(); i++) {
            const StageRow &r = rows[i];
            CHECK_TRUE(std::isfinite(r.deltaV), (tag + "delta-v is finite").c_str());
            CHECK_TRUE(std::isfinite(r.massStart) && std::isfinite(r.massEnd)
                       && r.massEnd > 0.0 && r.massStart >= r.massEnd,
                       (tag + "mass columns are finite and shrink").c_str());
            CHECK_REL(r.deltaV, ve * std::log(r.massStart / r.massEnd), 1e-9,
                       (tag + "delta-v matches ve ln(m0/m1)").c_str());
            spent += r.massStart - r.massEnd;
        }
        CHECK_REL(spent, fuel, 1e-9, (tag + "every kg of propellant is spent once").c_str());
    }
}

/* The drop walk reads a parent -> children table, and staging now builds that table
   once per pass (#520) instead of once per stage label. A wrong range in such a table
   does not crash -- it severs or grafts a branch -- so the pin is a shape where both
   mistakes land in the mass columns: the decoupler is a SIBLING of another part under
   the same parent, so the walk has to take its subtree and leave the sibling's, and it
   owns TWO children so both ends of its range matter. It also carries a SECOND
   decoupler inside that subtree on the SAME label, which is a shape a builder can make
   and the walk has to keep its footing on. Numbers by hand (base 50; side 10 dry +
   40 fuel; dec 5; branch 10 dry + 60 fuel; noseCone 10 dry; innerDec 5; innerTank 10
   dry; engines 10 dry, 1 kg/s, ve 1000): label 2 lights the branch engine only and
   drains the branch's 60 kg (220 -> 160; massEnd still carries the separating hardware,
   which the next row pays for), and label 1 then starts 50 kg lighter at 110 and drains
   the sibling's 40 kg: 110 -> 70. Mutation-checked: a walk that forgets the child side
   reads massStart 150 there, a histogram off by one reads 140, and a seed range one
   child short reads 120 -- all fail here and several checks elsewhere. Dropping the
   walk's visited guard instead shows up as a HANG, not a failure, which is what the
   self-parent case below is for. Three mutants are NOT failures and are recorded as such:
   deleting the sort changes no row (every ship in res/ships and 20 000 fuzzed ships
   agree bit-for-bit at full precision, so the sort is insurance rather than behaviour);
   skipping the walk for a seed another decoupler
   already reached changes nothing either, because that walk covered the subtree first; and
   pushing a seed without the dedupe does put duplicate indices into the caller's list,
   which no row notices only because a duplicated entry is always a decoupler -- a fuel
   barrier, so drainableIn skips it and kill is idempotent. */
static void test_drop_walk_takes_only_the_decouplers_side() {
    Catalog cat;
    BuildShip bs;
    const int base = addPart(bs, cat.tank(50, 0, 0), "base", -1, 1);
    addPart(bs, cat.tank(10, 20, 20), "side", base, 1);
    const int dec = addPart(bs, cat.decoupler(5), "dec", base, 2);
    const int branch = addPart(bs, cat.tank(10, 30, 30), "branch", dec, 2);
    const int noseCone = addPart(bs, cat.tank(10, 0, 0), "noseCone", dec, 2);
    const int innerDec = addPart(bs, cat.decoupler(5), "innerDec", noseCone, 2);
    addPart(bs, cat.tank(10, 0, 0), "innerTank", innerDec, 2);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "branchEng", branch, 2);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "coreEng", base, 1);

    const std::vector<StageRow> rows = computeStaging(bs, 10.0);
    CHECK_TRUE(rows.size() == 2, "two labels, two rows");
    if(rows.size() != 2) { return; }
    CHECK_TRUE(rows[0].stage == 2 && rows[0].drops, "the high label separates");
    CHECK_REL(rows[0].massStart, 220.0, 1e-9, "everything is aboard to start");
    CHECK_REL(rows[0].massEnd, 160.0, 1e-9, "the branch burned its own 60 kg");
    CHECK_REL(rows[0].deltaV, 1000.0 * std::log(220.0 / 160.0), 1e-9,
               "and paid for it");
    CHECK_TRUE(rows[1].stage == 1 && !rows[1].drops, "the final row separates nothing");
    CHECK_REL(rows[1].massStart, 110.0, 1e-9, "only the decoupler's side left");
    CHECK_REL(rows[1].massEnd, 70.0, 1e-9, "the sibling's tank is still there to drain");
    CHECK_REL(rows[1].deltaV, 1000.0 * std::log(110.0 / 70.0), 1e-9, "and it drains");
}

/* A part that is its OWN parent is malformed input -- the loaders refuse a parent that is
   not defined earlier (shipdef.cpp) and BuildShip::recomputePoses asserts parent < i --
   but the unit fixtures never call recomputePoses, and the walk's visited guard is the
   only thing between a cycle and an infinite loop. No other case in this file has a
   cycle, so pin one. An OUT-of-range parent cannot be pinned here: partitionFuelGroups
   asserts p < n (shipdef.cpp:1777) from the refreshFuel call inside the walk, so that
   input aborts the process before staging's own guard is reached. */
static void test_a_self_parent_walks_once() {
    Catalog cat;
    BuildShip bs;
    const int t = addPart(bs, cat.tank(50, 50, 50), "tank", -1, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "eng", t, 1);
    const int dec = addPart(bs, cat.decoupler(5), "loopDec", -1, 2);
    bs.parts[dec].parent = dec;   // a cycle: the part is its own parent

    const std::vector<StageRow> rows = computeStaging(bs, 10.0);
    CHECK_TRUE(rows.size() == 2, "two labels, two rows");
    if(rows.size() != 2) { return; }
    CHECK_TRUE(rows[0].drops, "the self-parented decoupler still separates");
    CHECK_REL(rows[0].massStart, 165.0, 1e-9, "the cycle loses nothing");
    CHECK_REL(rows[0].massEnd, 165.0, 1e-9, "nothing is lit at that label, so no burn");
    CHECK_REL(rows[1].massStart, 160.0, 1e-9, "and the decoupler left");
    CHECK_REL(rows[1].massEnd, 60.0, 1e-9, "the tank drains down to its dry mass");
}

/* What makes a drop "ready to leave" is the fuel a LIT ENGINE can reach inside it, not
   the fuel it happens to carry. A tank sealed off by a fuel wall never drains, so a drop
   holding one must not keep the period running: the burn stops when the reachable part of
   the drop runs out, the sealed tank is jettisoned full, and the survivor still has its
   own propellant for the next row. Nothing else in this file has a drop set holding BOTH
   reachable and unreachable burnable fuel, which is why ignoring the reachability filter
   passes every other case here (#536 makes that filter fast; this makes it load-bearing).
   Shape: the core (tank + engine) and the decoupler all carry label 2, so both engines are
   lit when label 2 is walked; the booster string's own tank is reachable, and a second
   wall inside the drop seals 80 kg of propellant that nothing can draw. Label 1 is the
   payload's label, so the walk has a second row to spend the survivor's fuel on. */
static void test_a_sealed_drop_does_not_hold_the_burn_open() {
    Catalog cat;
    BuildShip bs;
    const int coreTank = addPart(bs, cat.tank(50, 100, 100), "coreTank", -1, 2);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "coreEng", coreTank, 2);
    addPart(bs, cat.tank(5, 0, 0), "payload", coreTank, 1);
    const int dec = addPart(bs, cat.decoupler(5), "dec", coreTank, 2);
    const int boostTank = addPart(bs, cat.tank(10, 20, 20), "boostTank", dec, 2);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "boostEng", boostTank, 2);
    const int wall = addPart(bs, cat.decoupler(5), "wall", dec, 2);
    addPart(bs, cat.tank(10, 40, 40), "sealed", wall, 2);

    const std::vector<StageRow> rows = computeStaging(bs, 10.0);
    CHECK_TRUE(rows.size() == 2, "two labels, two rows");
    if(rows.size() != 2) { return; }

    // Label 2 burns while the reachable part of the drop lasts: boostTank's 40 kg at
    // 1 kg/s is 40 s, during which coreEng spends 40 kg of the core's 200 as well.
    CHECK_TRUE(rows[0].stage == 2 && rows[0].drops, "label 2 separates");
    CHECK_REL(rows[0].massStart, 425.0, 1e-9, "everything is aboard");
    CHECK_REL(rows[0].massEnd, 345.0, 1e-9, "80 kg burnt, the sealed tank still aboard");
    // The separation takes the sealed tank's 80 kg of propellant with it, full.
    CHECK_REL(rows[1].massStart, 225.0, 1e-9, "the survivor kept its own 160 kg");
    CHECK_REL(rows[1].massEnd, 65.0, 1e-9, "and burns it in the last row");
    CHECK_TRUE(rows[1].deltaV > 0.0, "the survivor still has something to burn");
    CHECK_REL(rows[1].deltaV, 1000.0 * std::log(225.0 / 65.0), 1e-9,
               "one engine, the fuel that is left");
}

/* #529: a decoupler on the LOWEST label. The walk calls that label the final period and a
   final period ends when the DROP set runs dry, so the survivor's own tank -- lit engine,
   most of the ship's propellant -- is never billed. The table shows one row, a plausible
   number ~5x too small here, and a mass column that agrees with it. 1 is the DEFAULT stage
   label, so this is what a hull looks like when nobody has touched the VAB's stage box, and
   it is why the ticket is High: silent, confident, and the common new-player shape.

   Shape: every part carries label 1, so period 1 is the whole walk. The decoupler is a fuel
   wall, so the booster's 50 kg and the core's 200 kg are separate groups; both engines are
   lit (label 1 == maxStage), so the period runs 50 s at 2 kg/s -- the booster's 50 kg and
   50 kg of the core's -- and the drop is then ready. What follows is the core's remaining
   150 kg, which belongs to this row: after the separation (5 + 50 + 10 = 65 kg leaves) the
   core burns 210 -> 60, so the row is 1000*ln(375/275) + 1000*ln(210/60).

   One row, not two: a row is a stage LABEL and this hull has one label. So the last period
   is "separate, then burn what is still aboard", and its massEnd is the mass AFTER the
   separation it performs -- there is no later row left to say what actually remained. */
static void test_a_decoupler_on_the_last_label_does_not_end_the_accounting() {
    Catalog cat;
    BuildShip bs;
    const int core = addPart(bs, cat.tank(50, 100, 100), "coreTank", -1, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "coreEng", core, 1);
    const int dec = addPart(bs, cat.decoupler(5), "dec", core, 1);
    const int btank = addPart(bs, cat.tank(50, 25, 25), "boostTank", dec, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "boostEng", btank, 1);
    (void)dec;

    const std::vector<StageRow> rows = computeStaging(bs, 10.0);
    CHECK_TRUE(rows.size() == 1, "one label, one row");
    if(rows.size() != 1) { return; }
    CHECK_TRUE(rows[0].stage == 1 && rows[0].drops, "label 1 separates");
    CHECK_REL(rows[0].massStart, 375.0, 1e-9, "all aboard");
    CHECK_REL(rows[0].massEnd, 60.0, 1e-9, "every burnable kg is gone: 50 + 10 of core dry");
    CHECK_REL(rows[0].deltaV, 1000.0 * (std::log(375.0 / 275.0) + std::log(210.0 / 60.0)),
              1e-9, "the drop's burn AND the survivor's burn, in one row");
    CHECK_TRUE(rows[0].engines == 2, "both engines light for the period");
    /* TWR across the merged period: the first period ignites at 2000 N against 375 kg, the
       second at 1000 N against the 210 kg that survived the separation -- and 1000/(210*10)
       is the worst ratio the stage ever flies, so min follows it. Max is the core running
       light at the end. */
    CHECK_REL(rows[0].minTWR, 1000.0 / (210.0 * 10.0), 1e-9, "min TWR = the lighter ignition");
    CHECK_REL(rows[0].maxTWR, 1000.0 / (60.0 * 10.0), 1e-9, "max TWR = the lightest mass");

    /* The follow-on burn invents nothing: the same hull with an empty core tank still pays
       exactly the booster's burn, and no period is spent chasing a tank that is dry. */
    Catalog cat2;
    BuildShip es;
    const int ecore = addPart(es, cat2.tank(50, 0, 0), "coreTank", -1, 1);
    addPart(es, cat2.engine(10, 0.5, 1000.0), "coreEng", ecore, 1);
    const int edec = addPart(es, cat2.decoupler(5), "dec", ecore, 1);
    const int ebt = addPart(es, cat2.tank(50, 25, 25), "boostTank", edec, 1);
    addPart(es, cat2.engine(10, 0.5, 1000.0), "boostEng", ebt, 1);
    (void)edec;
    const std::vector<StageRow> erows = computeStaging(es, 10.0);
    CHECK_TRUE(erows.size() == 1, "empty core: still one row");
    if(erows.size() != 1) { return; }
    CHECK_REL(erows[0].massStart, 175.0, 1e-9, "empty core, full booster");
    CHECK_REL(erows[0].deltaV, 1000.0 * std::log(175.0 / 125.0), 1e-9,
              "empty core: the booster's burn and nothing more");
    CHECK_REL(erows[0].massEnd, 60.0, 1e-9, "empty core: the separation still leaves");
}

/* Two separating periods, which is the shape that makes reachability marks a LIFETIME
   question rather than a per-call one: period 2 inherits the mark buffer from period 1,
   and after period 1's kill the groups are renumbered, so a stale mark can land on a
   group id period 2 cares about. The two fuel sources in period 1's drop are sized to
   empty in the SAME step, so that period makes three reachability calls (one at the
   drainAll decision, one per burn step, and the one that finds the drop dry) -- the third
   is where a leftover stamp from period 1 lines up with period 2's tick. And the engine-less linkedTank
   is reachable only across a fuel_link, so the marking has to cross revLinks (the drop's
   20 kg of linkedTank fuel is part of the burn; see
   test_a_linked_tank_in_a_drop_holds_the_burn_open for the case that pins that on its own).
   Every row is 1000*ln(m0/m1): one or two engines, each 1 kg/s at ve 1000, so F/mdot is
   1000 however many are lit. The teeth here are the STALE MARKS: assign->resize, which
   keeps period 1's stamps and lets them resurface once period 2's tick climbs back to them,
   reads s2 m=360->140 and s1 dv=0. Note that the sizes are what make that visible -- the
   two fuel sources in the first drop empty in the same step, so the period ends on its
   third reachability call, and period 2's third call is where a leftover stamp collides. */
static void test_reachability_survives_a_renumbering() {
    Catalog cat;
    BuildShip bs;
    const int core = addPart(bs, cat.tank(50, 100, 100), "coreTank", -1, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "coreEng", core, 3);
    const int decA = addPart(bs, cat.decoupler(5), "decA", core, 3);
    const int t1 = addPart(bs, cat.tank(10, 10, 10), "boostTank", decA, 3);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "boostEng", t1, 3);
    const int l1 = addPart(bs, cat.tank(10, 10, 10), "linkedTank", decA, 3);
    addLink(bs, "linkedTank", "coreTank");
    const int decB = addPart(bs, cat.decoupler(5), "decB", core, 2);
    const int r = addPart(bs, cat.tank(10, 10, 10), "reachTank", decB, 2);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "reachEng", r, 2);
    const int wall = addPart(bs, cat.decoupler(5), "wall", decB, 2);
    addPart(bs, cat.tank(10, 20, 20), "sealedTank", wall, 2);
    (void)t1; (void)l1; (void)r; (void)wall;

    const std::vector<StageRow> rows = computeStaging(bs, 10.0);
    CHECK_TRUE(rows.size() == 3, "three labels, three rows");
    if(rows.size() != 3) { return; }
    CHECK_TRUE(rows[0].stage == 3 && rows[0].drops, "label 3 separates");
    CHECK_REL(rows[0].massStart, 435.0, 1e-9, "all aboard");
    CHECK_REL(rows[0].massEnd, 395.0, 1e-9, "the drop's REACHABLE fuel burns");
    CHECK_REL(rows[0].deltaV, 1000.0 * std::log(435.0 / 395.0), 1e-9, "s3 dv");

    CHECK_TRUE(rows[1].stage == 2 && rows[1].drops, "label 2 separates");
    CHECK_REL(rows[1].massStart, 360.0, 1e-9, "period 1 left, renumbered");
    CHECK_REL(rows[1].massEnd, 320.0, 1e-9, "and its reachable fuel burns, no more");
    CHECK_REL(rows[1].deltaV, 1000.0 * std::log(360.0 / 320.0), 1e-9, "s2 dv");

    CHECK_REL(rows[2].massStart, 240.0, 1e-9, "the survivor, both drops gone");
    CHECK_REL(rows[2].massEnd, 60.0, 1e-9, "burns everything that is left");
    CHECK_REL(rows[2].deltaV, 1000.0 * std::log(240.0 / 60.0), 1e-9, "s1 dv");
}

/* An engine-less tank in a drop, fed to the core engine by a fuel link, is part of what
   the drop can burn -- so it holds the period open. It is the biggest tank in the drop on
   purpose (100 kg against boostTank's 20): a lit engine drains the furthest link layer
   first, so the period runs until linkedTank is dry (120 kg) instead of stopping when
   boostTank is (20 kg). Marking only each lit engine's OWN group instead of its drain
   layers reads s3 415->375, dv 101 instead of 341; counting no fuel at all reads s3
   dv=0. No ship in res/ships tells these apart -- in theirs every group inside a drop
   carries its own engine, so the own-group shortcut is invisible there. */
static void test_a_linked_tank_in_a_drop_holds_the_burn_open() {
    Catalog cat;
    BuildShip bs;
    const int core = addPart(bs, cat.tank(50, 100, 100), "coreTank", -1, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "coreEng", core, 3);
    const int decA = addPart(bs, cat.decoupler(5), "decA", core, 3);
    const int t1 = addPart(bs, cat.tank(10, 10, 10), "boostTank", decA, 3);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "boostEng", t1, 3);
    const int l1 = addPart(bs, cat.tank(10, 50, 50), "linkedTank", decA, 3);
    addLink(bs, "linkedTank", "coreTank");
    (void)t1; (void)l1;

    const std::vector<StageRow> rows = computeStaging(bs, 10.0);
    CHECK_TRUE(rows.size() == 2, "two labels, two rows");
    if(rows.size() != 2) { return; }
    CHECK_REL(rows[0].massStart, 415.0, 1e-9, "all aboard");
    CHECK_REL(rows[0].massEnd, 295.0, 1e-9, "both drop tanks run dry, the core's does not");
    CHECK_REL(rows[0].deltaV, 1000.0 * std::log(415.0 / 295.0), 1e-9, "s3 dv");
    CHECK_REL(rows[1].massStart, 260.0, 1e-9, "the drop leaves 35 kg of structure");
    CHECK_REL(rows[1].massEnd, 60.0, 1e-9, "the core burns the rest");
    CHECK_REL(rows[1].deltaV, 1000.0 * std::log(260.0 / 60.0), 1e-9, "s1 dv");
}

/* A decoupler an EARLIER separation already took must not start a walk of its own. decB
   carries label 2 and hangs under decA (label 3), so by the time the walk reaches label 2
   decB is dead, and label 2 must separate nothing. A label that separates nothing emits no
   row at all, so the symptom is an EXTRA row: a staging event the ship can no longer
   perform. What keeps the row away is droppedAt() seeding only LIVE decouplers: delete that
   test and this fixture fails. The other half -- droppedAt() listing only live parts -- is
   invisible here and everywhere, because a dead entry adds no mass, no fuel and no kill;
   delete both tests and the phantom row returns. Before #537 the second test sat in
   computeStaging over the returned vector, and that pair hid each other: deleting either
   one read green, which is why the fix was to move the rule, not to keep both copies. */
static void test_a_separation_does_not_relist_an_already_dead_subtree() {
    Catalog cat;
    BuildShip bs;
    const int core = addPart(bs, cat.tank(50, 50, 50), "coreTank", -1, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "coreEng", core, 3);
    const int decA = addPart(bs, cat.decoupler(5), "decA", core, 3);
    const int decB = addPart(bs, cat.decoupler(5), "decB", decA, 2);
    addPart(bs, cat.tank(10, 10, 10), "deadTank", decB, 2);
    (void)decB;

    const std::vector<StageRow> rows = computeStaging(bs, 10.0);
    CHECK_TRUE(rows.size() == 2, "label 2 separates nothing, so it gets no row: 3 and 1 only");
    if(rows.size() != 2) { return; }
    CHECK_TRUE(rows[0].stage == 3 && rows[0].drops, "label 3 separates the whole tail");
    CHECK_REL(rows[0].massStart, 200.0, 1e-9, "all aboard");
    /* The tail is walled off, so this drop is INERT and the row falls back to draining
       everything reachable (computeStaging's inert-drop fallback: drainAll when the drop
       holds nothing drainable) rather than burning nothing. Its own tank's 20 kg stays
       aboard and leaves with the tail. */
    CHECK_REL(rows[0].massEnd, 100.0, 1e-9, "the core's 100 kg burns; the tail's does not");
    CHECK_REL(rows[0].deltaV, 1000.0 * std::log(200.0 / 100.0), 1e-9, "s3 dv");
    CHECK_TRUE(rows[1].stage == 1, "the next row is the survivor's, not label 2's");
    CHECK_REL(rows[1].massStart, 60.0, 1e-9, "the tail left dry, and so did the core");
    CHECK_REL(rows[1].massEnd, 60.0, 1e-9, "nothing left to burn");
    CHECK_REL(rows[1].deltaV, 0.0, 1e-9, "s1 dv");
}

/* Reachability means "a LIT engine can draw this", not "some engine somewhere could
   draw it". secEng is on label 2, so at label 3 it is unlit: linkTank, which feeds only
   secTank's group, is out of reach and must not be credited to label 3's drop. Marking
   every engine's layers regardless of `lit` instead reads s3 520->300, dv 550 -- 200 kg
   of the survivor's own fuel billed at the wrong mass, because nothing drains linkTank
   (its only consumer is unlit) yet the period stays open chasing it. A lit engine two
   hops up an asparagus string already marks everything that unlit engine's group
   reaches -- drainLayersFor() is transitive -- so the discriminator needs the unlit
   engine's group in a DIFFERENT branch of the link graph, which is why port (a fuel
   wall that never separates) sits between the two strings. */
static void test_an_unlit_engine_does_not_open_a_burn() {
    Catalog cat;
    BuildShip bs;
    const int core = addPart(bs, cat.tank(50, 100, 100), "coreTank", -1, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "coreEng", core, 3);
    const int port = addPart(bs, cat.barrier(5), "port", core, 1);
    const int st = addPart(bs, cat.tank(20, 40, 40), "secTank", port, 2);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "secEng", st, 2);   // unlit until label 2
    const int decA = addPart(bs, cat.decoupler(5), "decA", core, 3);
    addPart(bs, cat.tank(10, 10, 10), "dropTank", decA, 3);      // the lit core drains this
    addPart(bs, cat.tank(10, 50, 50), "linkTank", decA, 3);      // feeds secTank ONLY
    addLink(bs, "dropTank", "coreTank");
    addLink(bs, "linkTank", "secTank");
    (void)port; (void)st;

    const std::vector<StageRow> rows = computeStaging(bs, 10.0);
    CHECK_TRUE(rows.size() == 2, "label 2 separates nothing, so rows are 3 and 1");
    if(rows.size() != 2) { return; }
    CHECK_TRUE(rows[0].engines == 1, "secEng is not lit at label 3");
    CHECK_REL(rows[0].massStart, 520.0, 1e-9, "all aboard");
    CHECK_REL(rows[0].massEnd, 500.0, 1e-9, "only dropTank's 20 kg burns");
    CHECK_REL(rows[0].deltaV, 1000.0 * std::log(520.0 / 500.0), 1e-9, "s3 dv");
    CHECK_REL(rows[1].massStart, 375.0, 1e-9, "linkTank leaves with its 100 kg unburnt");
    CHECK_REL(rows[1].massEnd, 95.0, 1e-9, "nothing drainable is left");
    CHECK_TRUE(rows[1].engines == 2, "secEng lights for the last row");
    CHECK_REL(rows[1].deltaV, 1000.0 * std::log(375.0 / 95.0), 1e-9, "s1 dv");
}

/* The reachability marks are stamped once per burn, after that label's engines light
   (#538). This is the second shape that constrains WHERE the stamp sits, and it fails
   differently from test_reachability_survives_a_renumbering: the newly lit engine
   (secEng, label 2) is outside the drop while the tank it newly reaches (linkTank) is
   inside it, so stamping before the lighting reads drainableIn(drop) == 0 and the whole
   period flips to drainAll -- linkTank's 40 kg burns and midTank's 20 goes with it.
   Derived from the model: dry 125, fuel 300, wet 425; every row is ve*ln(m0/m1).
     label 3: coreEng lit, drop {decA, dropTank}; dropTank feeds coreTank -> 425->405.
     label 2: secEng lights; linkTank is now drainable, midTank (no link, no engine)
       is not -> 390->310, with coreEng taking its 40 kg from coreTank.
     label 1: no decoupler left (port is a wall, not a decoupler) -> drainAll burns the
       160 + 20 kg still aboard -> 265->85. */
static void test_a_newly_lit_engine_outside_the_drop_reaches_inside_it() {
    Catalog cat;
    BuildShip bs;
    const int core = addPart(bs, cat.tank(50, 100, 100), "coreTank", -1, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "coreEng", core, 3);
    const int decA = addPart(bs, cat.decoupler(5), "decA", core, 3);
    addPart(bs, cat.tank(10, 10, 10), "dropTank", decA, 3);      // feeds coreTank
    addLink(bs, "dropTank", "coreTank");
    const int port = addPart(bs, cat.barrier(5), "port", core, 1);
    const int st = addPart(bs, cat.tank(10, 10, 10), "secTank", port, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "secEng", st, 2);   // lit from label 2
    const int decB = addPart(bs, cat.decoupler(5), "decB", core, 2);
    addPart(bs, cat.tank(10, 10, 10), "midTank", decB, 2);       // no link, no engine
    addPart(bs, cat.tank(10, 20, 20), "linkTank", decB, 2);      // feeds secTank
    addLink(bs, "linkTank", "secTank");
    (void)decA; (void)port; (void)decB; (void)st;

    const std::vector<StageRow> rows = computeStaging(bs, 10.0);
    CHECK_TRUE(rows.size() == 3, "rows for labels 3, 2 and the final label 1");
    if(rows.size() != 3) { return; }
    CHECK_TRUE(rows[0].drops, "label 3 separates decA and dropTank");
    CHECK_TRUE(rows[1].drops, "label 2 separates decB, midTank and linkTank");
    CHECK_TRUE(!rows[2].drops, "no decoupler is left at label 1, so this is the drainAll row");
    CHECK_TRUE(rows[0].engines == 1, "only coreEng is lit at label 3");
    CHECK_REL(rows[0].massStart, 425.0, 1e-9, "all aboard");
    CHECK_REL(rows[0].massEnd, 405.0, 1e-9, "dropTank's 20 kg burn");
    CHECK_REL(rows[0].deltaV, 1000.0 * std::log(425.0 / 405.0), 1e-9, "s3 dv");
    CHECK_TRUE(rows[1].engines == 2, "secEng lights for label 2");
    CHECK_REL(rows[1].massStart, 390.0, 1e-9, "dropTank left dry");
    CHECK_REL(rows[1].massEnd, 310.0, 1e-9, "linkTank's 40 kg is drainable, midTank's is not");
    CHECK_REL(rows[1].deltaV, 1000.0 * std::log(390.0 / 310.0), 1e-9, "s2 dv: two engines, same ve");
    CHECK_REL(rows[2].massStart, 265.0, 1e-9, "midTank leaves with 20 kg unburnt");
    CHECK_REL(rows[2].massEnd, 85.0, 1e-9, "drainAll burns the 180 kg still aboard");
    CHECK_REL(rows[2].deltaV, 1000.0 * std::log(265.0 / 85.0), 1e-9, "s1 dv");
}

/* A LIVE decoupler sitting above a part an EARLIER separation already killed, where that
   dead part still holds fuel that the LIT engines can now reach. Nothing in the suite had
   that shape: this file had none, and 1,768 of 20,000 fuzzed ships do (the earlier figure
   of 11,751 counted dead walk entries, not ships). It earns its keep on exactly one mutant:
   let kill() unlight every LIT engine rather than only the engines of the parts it dropped,
   and the suite reads 31 failures where every other fixture's share is 30 -- the 31st is
   this file's "secEng lights for label 2". It does NOT catch the aliveness pair: a dead
   part carries group -1 because buildFuelGroups makes dead a wall, so deleting
   droppedAt()'s listing test, drainableIn()'s alive skip, or BOTH reads 292/0. Keep it for
   the shape itself -- the dead tank's 40 kg left at label 3 and label 2 must not adopt it,
   which is where a future per-part action on the drop list (marking, accounting) bites.
   Derived from the model (dry 105, fuel 260, wet 365; ve 1000, 1 kg/s per engine):
     label 3: coreEng lit only. deadTank hangs under decA (label 3) and feeds secTank,
       whose engine is not lit yet, so the drop {decA, deadTank} holds nothing drainable
       -> the inert-drop fallback burns coreTank's 200 kg, 365->165.
     label 2: secEng lights; decTop (label 2) is still alive above the now-dead decA, so
       its drop is {decTop} alone and stays inert -> burns the 20 kg left aboard, 110->90.
       decA and deadTank (with its 40 kg) left the ship at label 3, NOT at label 2.
     label 1: decTop itself left at the end of label 2 (massEnd is sampled before the
       kill), and nothing else is separate or burnable, so the last row is 85->85, dv 0. */
static void test_a_live_decoupler_above_a_dead_tank_does_not_adopt_its_fuel() {
    Catalog cat;
    BuildShip bs;
    const int core = addPart(bs, cat.tank(50, 100, 100), "coreTank", -1, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "coreEng", core, 3);
    const int decTop = addPart(bs, cat.decoupler(5), "decTop", core, 2);
    const int decA = addPart(bs, cat.decoupler(5), "decA", decTop, 3);
    addPart(bs, cat.tank(10, 20, 20), "deadTank", decA, 3);   // dies at label 3, fuel intact
    const int port = addPart(bs, cat.barrier(5), "port", core, 1);
    const int st = addPart(bs, cat.tank(10, 10, 10), "secTank", port, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "secEng", st, 2);   // lit from label 2
    addLink(bs, "deadTank", "secTank");   // its fuel becomes reachable only at label 2
    (void)port; (void)st;

    const std::vector<StageRow> rows = computeStaging(bs, 10.0);
    CHECK_TRUE(rows.size() == 3, "labels 3, 2 and the final label 1");
    if(rows.size() != 3) { return; }
    CHECK_REL(rows[0].massStart, 365.0, 1e-9, "all aboard");
    CHECK_REL(rows[0].massEnd, 165.0, 1e-9, "the inert drop falls back to draining coreTank");
    CHECK_REL(rows[0].deltaV, 1000.0 * std::log(365.0 / 165.0), 1e-9, "s3 dv");
    CHECK_TRUE(rows[1].drops, "decTop is still alive at label 2");
    CHECK_TRUE(rows[1].engines == 2, "secEng lights for label 2");
    CHECK_REL(rows[1].massStart, 110.0, 1e-9, "decA left dry and deadTank left with its 40 kg");
    CHECK_REL(rows[1].massEnd, 90.0, 1e-9, "only the 20 kg aboard is burnable");
    CHECK_REL(rows[2].massStart, 85.0, 1e-9, "decTop left too, so label 1 starts 5 kg lighter");
    CHECK_REL(rows[2].massEnd, 85.0, 1e-9, "and nothing drainable is left");
    CHECK_REL(rows[2].deltaV, 0.0, 1e-9, "s1 dv: the burn already had it");
}

/* Seeding: staging a ship that EXISTS rather than a build being authored (#948). The
   VAB asks "what would this build do, tanks full"; a flown ship has to answer "what do
   I have left", and the only difference is two arrays -- what each part holds of the
   burnable resources, and what it weighs with crew, inventory and any resource no
   engine on this ship can burn. The fixture is test_two_stage's (dry 125, fuel 200,
   wet 325; ve 1000, 1 kg/s per engine, g 10), so the numbers below are the same
   arithmetic with the fuel moved. */
static void test_seed_the_fuel_and_mass_a_live_ship_carries() {
    Catalog cat;
    BuildShip bs;
    const int upTank = addPart(bs, cat.tank(50, 25, 25), "upTank", -1, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "upEng", upTank, 1);
    const int dec = addPart(bs, cat.decoupler(5), "dec", upTank, 2);
    const int loTank = addPart(bs, cat.tank(50, 75, 75), "loTank", dec, 2);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "loEng", loTank, 2);
    (void)dec; (void)loTank;

    const std::vector<StageRow> full = computeStaging(bs, 10.0);
    CHECK_TRUE(full.size() == 2, "the unseeded fixture still has two rows");

    /* The mask is exported so a live ship weighs its resources with THIS rule, not a
       copy of it: both propellants here, because both engines are chemical. */
    bool burn[(int)ResourceType::Num] = {};
    stagingPropellantMask(bs, burn);
    CHECK_TRUE(burn[(int)ResourceType::Hydrogen] && burn[(int)ResourceType::LOX],
               "the build's engines decide what is burnable");
    CHECK_TRUE(!burn[(int)ResourceType::Hydrazine] && !burn[(int)ResourceType::JetFuel],
               "and nothing else joins the pool");

    /* Seeding the def's own figures has to be the identity, bit for bit -- otherwise
       the flight readout and the VAB table are different code paths for the same
       question, which is the bug this seed exists to close. */
    StagingSeed same;
    for(size_t i = 0; i < bs.parts.size(); i++) {
        same.fuelKg.push_back(partPropellantMass(*bs.parts[i].def, burn));
        same.dryKg.push_back(partDryMass(*bs.parts[i].def, burn));
    }
    const std::vector<StageRow> seeded = computeStaging(bs, 10.0, 1.0, same);
    bool identical = seeded.size() == full.size();
    for(size_t i = 0; identical && i < full.size(); i++) {
        identical = seeded[i].deltaV == full[i].deltaV
                 && seeded[i].massStart == full[i].massStart
                 && seeded[i].massEnd == full[i].massEnd
                 && seeded[i].minTWR == full[i].minTWR
                 && seeded[i].maxTWR == full[i].maxTWR
                 && seeded[i].engines == full[i].engines
                 && seeded[i].drops == full[i].drops
                 && seeded[i].stage == full[i].stage;
    }
    CHECK_TRUE(identical, "seeding the def's own figures changes no digit");

    /* Half-full tanks: 125 dry + 100 fuel aboard. */
    StagingSeed half = same;
    for(double &f : half.fuelKg) { f *= 0.5; }
    const std::vector<StageRow> rows = computeStaging(bs, 10.0, 1.0, half);
    CHECK_TRUE(rows.size() == 2, "half fuel still flies the same two labels");
    if(rows.size() != 2) { return; }
    CHECK_REL(rows[0].massStart, 225.0, 1e-9, "the row starts at the mass aboard");
    CHECK_REL(rows[0].massEnd, 150.0, 1e-9, "the lower tank had 75 kg to give, not 150");
    CHECK_REL(rows[0].deltaV, 1000.0 * std::log(225.0 / 150.0), 1e-9, "s2 dv on the fuel that is there");
    CHECK_REL(rows[1].massStart, 85.0, 1e-9, "dec + drained loTank + loEng left");
    CHECK_REL(rows[1].massEnd, 60.0, 1e-9, "the upper tank had 25 kg left to burn");
    CHECK_REL(rows[1].deltaV, 1000.0 * std::log(85.0 / 60.0), 1e-9, "s1 dv likewise");

    /* Crew and inventory: PartDef cannot know a seat weighs anything, so the caller
       folds the extra into that part's dry mass. Same fuel, more kg to move. That is
       how a flown hull counts its seats: mars_explorer_mk1 carries 97.0 kg of crew the
       authored build tree does not, which alone put its flight figure 1.5% under the
       VAB's before it was accounted for (#948). */
    StagingSeed crewed = half;
    crewed.dryKg[0] += 90.0;
    const std::vector<StageRow> flown = computeStaging(bs, 10.0, 1.0, crewed);
    CHECK_REL(flown[0].massStart, 315.0, 1e-9, "the crew rides the part that carries them");
    CHECK_REL(flown[0].deltaV, 1000.0 * std::log(315.0 / 240.0), 1e-9, "same fuel, heavier stack");
    CHECK_REL(flown[1].deltaV, 1000.0 * std::log(175.0 / 150.0), 1e-9, "and the drop still sheds only the drop");

    /* Nothing aboard to burn: the plan is two rows of zero, and the hardware and its
       TWR are still there to explain why the number is not an error. */
    StagingSeed empty = same;
    for(double &f : empty.fuelKg) { f = 0.0; }
    const std::vector<StageRow> dry = computeStaging(bs, 10.0, 1.0, empty);
    CHECK_TRUE(dry.size() == 2, "an empty ship still walks its labels");
    CHECK_REL(dry[0].massStart, 125.0, 1e-9, "dry mass only");
    CHECK_REL(dry[0].deltaV + dry[1].deltaV, 0.0, 0.0, "no fuel, no delta-v");
    CHECK_TRUE(dry[0].engines == 1 && dry[0].drops, "the stage-2 engine is lit and the drop still happens");
}

/* THREE drain layers, so the layer indexing has to be right rather than merely
   two-deep: the core engine's set is [outTank, outTank2] (2 hops), [midTank] (1 hop),
   [coreTank] (0), furthest first, and each period ends on its own layer because the
   drop set is that layer. The hop distance, not the part order, decides what burns:
   outTank2 is built three parts after midTank and hangs two tree levels off it, yet
   drains in the first period, while the core's own tank -- one part from the engine --
   waits for the last. Nearest-first (layer li carrying dist li rather than maxDist -
   li) spends everything in the first period: row 0 reads 215 -> 75 and rows 1 and 2
   burn nothing, because draining midTank and coreTank never lowers the drop set's
   drainable fuel, so the burn chases on until the ship is dry.
   Derived: wet 215 = coreTank 80 + coreEng 10 + decA 5 + midTank 55 + decB 5
   + outTank 40 + outTank2 20. One engine, mdot 1 kg/s, ve 1000. Dead parts leave the
   mass, so each row starts 25 kg / 20 kg under the last one's end -- the dry of the
   separation that just happened, fuel already spent.
     label 3 (decB + its two tanks): the outer layer holds 30 + 10 kg, pro-rated
       0.75 / 0.25, both empty at t = 40 s -> massEnd 175.
     label 2 (decA + midTank): 150 after decB's subtree goes (its 5 + 10 + 10 dry),
       40 kg burns -> 110.
     label 1 (final, drainAll): 90 after decA's and midTank's dry goes, the core's
       60 kg burn -> 30. */
static void test_a_three_hop_string_drains_one_layer_per_period() {
    Catalog cat;
    BuildShip bs;
    const int core = addPart(bs, cat.tank(20, 30, 30), "coreTank", -1, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "coreEng", core, 3);
    const int decA = addPart(bs, cat.decoupler(5), "decA", core, 2);
    const int mid = addPart(bs, cat.tank(15, 20, 20), "midTank", decA, 2);
    const int decB = addPart(bs, cat.decoupler(5), "decB", mid, 3);
    addPart(bs, cat.tank(10, 15, 15), "outTank", decB, 3);
    addPart(bs, cat.tank(10, 5, 5), "outTank2", decB, 3);
    addLink(bs, "midTank", "coreTank");
    addLink(bs, "outTank", "midTank");
    addLink(bs, "outTank2", "midTank");

    const double g = 10.0;
    const std::vector<StageRow> rows = computeStaging(bs, g);
    CHECK_TRUE(rows.size() == 3, "three labels, three rows");
    if(rows.size() != 3) { return; }

    CHECK_TRUE(rows[0].drops && rows[1].drops && !rows[2].drops, "two separations, then the end");
    CHECK_REL(rows[0].massStart, 215.0, 1e-9, "all aboard");
    CHECK_REL(rows[0].massEnd, 175.0, 1e-9, "the 2-hop layer only");
    CHECK_REL(rows[0].deltaV, 1000.0 * std::log(215.0 / 175.0), 1e-9, "label 3 dv");
    CHECK_REL(rows[1].massStart, 150.0, 1e-9, "decB's dry came with its tanks");
    CHECK_REL(rows[1].massEnd, 110.0, 1e-9, "the 1-hop layer");
    CHECK_REL(rows[1].deltaV, 1000.0 * std::log(150.0 / 110.0), 1e-9, "label 2 dv");
    CHECK_REL(rows[2].massStart, 90.0, 1e-9, "the core's own tank is the last layer");
    CHECK_REL(rows[2].massEnd, 30.0, 1e-9, "and it is the last to empty");
    CHECK_REL(rows[2].deltaV, 1000.0 * std::log(90.0 / 30.0), 1e-9, "label 1 dv");
    CHECK_TRUE(rows[0].engines == 1 && rows[1].engines == 1 && rows[2].engines == 1,
               "one engine reaches every layer");

    /* The hop table drainLayersFor reuses is static, so it outlives the call that
       filled it (#963), and this file computes a dozen ships in one process. Dropping
       the walk's exit clear leaves marks standing for the next one: 39 checks go red,
       9 of them in this fixture, starting at the pass above (row 0 reads 215 -> 155
       because the BFS finds midTank already marked and never walks out to the 2-hop
       layer). The same rule's other half -- a table that GROWS must start its new
       entries unseen, since 0 means "this group is the one with the lit engine" -- is
       caught by the deep chain's "a second pass reads the same rows" check further up
       this file; the one-group ship here just keeps the shapes varied, since a smaller
       ship cannot make the table grow. */
    Catalog cat2;
    BuildShip alone;
    const int t1 = addPart(alone, cat2.tank(20, 30, 30), "tank", -1, 1);
    addPart(alone, cat2.engine(10, 0.5, 1000.0), "eng", t1, 1);
    const std::vector<StageRow> single = computeStaging(alone, g);
    CHECK_TRUE(single.size() == 1, "no decoupler, one period");
    CHECK_REL(single[0].deltaV, 1000.0 * std::log(90.0 / 30.0), 1e-9, "alone dv");

    const std::vector<StageRow> again = computeStaging(bs, g);
    CHECK_TRUE(again.size() == 3, "the same ship, again, still three rows");
    if(again.size() != 3) { return; }
    CHECK_REL(again[0].massEnd, 175.0, 1e-9, "the outer layer drained first, again");
    CHECK_REL(again[1].massEnd, 110.0, 1e-9, "the middle layer, again");
    CHECK_REL(again[2].massEnd, 30.0, 1e-9, "the core last, again");
    CHECK_REL(again[0].deltaV + again[1].deltaV + again[2].deltaV,
              rows[0].deltaV + rows[1].deltaV + rows[2].deltaV, 1e-12,
              "the same plan out of a reused hop table");
}

/* One Sim now answers every computeStaging call in the process (#963), so a ship's
   snapshot must not inherit anything from the ship planned before it. BIG is the
   three-hop string above -- seven parts, three labels, three fuel groups, an engine at
   index 1, and it ends with five parts dead. SMALL is deliberately NOT that shape: four
   parts, one label, one fuel group, an inert tank where BIG's engine sat, and a part
   with no def at all where BIG's 55 kg mid tank sat (a legal BuildShip -- the snapshot
   loop has a branch for it). What a missed reset does, measured by deleting it and
   running the suite (an abort is named by the assert that fires, or by its signal):
     sp.alive        SMALL's engine sits where BIG killed decA, i.e. an 85 kg hull with
                     nothing lit. The suite aborts before this fixture, at
                     markReachable's "litSeen == litCount" assert.
     sp.thrust/mdot  SMALL's inert tank sits where BIG's engine sat, so a phantom engine
                     joins the burn: engines reads 2 and both TWRs double AT THE SAME DV.
                     29 checks red, 4 of them here -- the masses alone would not catch
                     it, which is why TWR is pinned.
     sp.dry/fuel     SMALL's def-less part sits where BIG's fueled mid tank sat, so the
                     row bills 150 kg to a 95 kg hull. 7 checks red, all 7 here.
     engines         the previous ship's engine list survives and lights: same abort.
     litCount        ditto.
     layersEpoch     SMALL has one fuel group where BIG had three, so a cached layer set
                     answers with BIG's groups. 105 checks red, 6 here.
     stageLabels     SMALL carries label 1 alone, so BIG's 2 and 3 trip the walk's own
                     "labels end at maxStage" assert.
     revOf           the previous ship's feeders still stand: abort in drainLayersFor.
     buildBuckets' cursor copy, and buildFuelGroups' resize: segfault (the scratch still
                     holds the previous fill / is sized for a smaller ship).
     markLitCount    NOT pinned -- green. Every caller in staging.cpp marks before it
                     asks, so only a future one would notice; the line is restored so a
                     reused Sim keeps a fresh Sim's tripwire, and said out loud rather
                     than counted as a win.
   Some of those land on an assert rather than a number, which is the point: a surviving
   buffer is exactly where a wrong plan rather than a crash is the danger, so the numbers
   are pinned too. SMALL runs before BIG in this fixture, so its rows are the reference
   that the two reused passes are compared against -- and by here the Sim has already
   served a dozen other ships, so even that first pass is a reused one. The last pair is
   the case where parts.resize() GROWS rather than reuses, covering both halves of the
   buffer's life. */
static void test_a_small_ship_planned_after_a_big_one_is_only_itself() {
    const double g = 10.0;

    Catalog catA;
    BuildShip big;
    const int core = addPart(big, catA.tank(20, 30, 30), "coreTank", -1, 1);
    addPart(big, catA.engine(10, 0.5, 1000.0), "coreEng", core, 3);
    const int decA = addPart(big, catA.decoupler(5), "decA", core, 2);
    const int mid = addPart(big, catA.tank(15, 20, 20), "midTank", decA, 2);
    const int decB = addPart(big, catA.decoupler(5), "decB", mid, 3);
    addPart(big, catA.tank(10, 15, 15), "outTank", decB, 3);
    addPart(big, catA.tank(10, 5, 5), "outTank2", decB, 3);
    addLink(big, "midTank", "coreTank");
    addLink(big, "outTank", "midTank");
    addLink(big, "outTank2", "midTank");

    Catalog catB;
    BuildShip small;
    const int stank = addPart(small, catB.tank(20, 30, 30), "tank", -1, 1);
    addPart(small, catB.tank(5, 0, 0), "frame", stank, 1);   // inert, where BIG's engine sat
    addPart(small, catB.engine(10, 0.5, 1000.0), "eng", stank, 1);
    BuildPart loose;                  // no def: legal, and the pass has a branch for it
    loose.id = "noDef";
    loose.parent = stank;
    loose.stage = 1;
    small.parts.push_back(loose);

    /* 95 kg aboard (80 + 5 + 10 + 0), one lit engine at 1000 N, dry at 35 kg. */
    const std::vector<StageRow> solo = computeStaging(small, g);
    CHECK_TRUE(solo.size() == 1, "one label, one period");
    if(solo.size() != 1) { return; }
    CHECK_REL(solo[0].massStart, 95.0, 1e-9, "95 kg of small ship");
    CHECK_REL(solo[0].massEnd, 35.0, 1e-9, "its 60 kg burned");
    CHECK_REL(solo[0].deltaV, 1000.0 * std::log(95.0 / 35.0), 1e-9, "small ship dv");
    CHECK_TRUE(solo[0].engines == 1, "one engine aboard");
    CHECK_REL(solo[0].minTWR, 1000.0 / (95.0 * g), 1e-9, "small ignition TWR");
    CHECK_REL(solo[0].maxTWR, 1000.0 / (35.0 * g), 1e-9, "small cutoff TWR");

    const std::vector<StageRow> bigRows = computeStaging(big, g);
    CHECK_TRUE(bigRows.size() == 3, "the big ship keeps its three periods");
    CHECK_REL(bigRows[0].massEnd, 175.0, 1e-9, "the outer layer, as the fixture above");

    const std::vector<StageRow> again = computeStaging(small, g);
    CHECK_TRUE(again.size() == 1, "still one period after the big ship");
    if(again.size() != 1) { return; }
    CHECK_REL(again[0].massStart, 95.0, 1e-9, "95 kg, not BIG's leftovers");
    CHECK_REL(again[0].massEnd, 35.0, 1e-9, "the same burn");
    CHECK_REL(again[0].deltaV, solo[0].deltaV, 1e-12, "the same dv");
    CHECK_TRUE(again[0].engines == 1, "one engine, not BIG's engine list");
    CHECK_REL(again[0].minTWR, solo[0].minTWR, 1e-12, "one engine's thrust");

    const std::vector<StageRow> bigAgain = computeStaging(big, g);
    CHECK_TRUE(bigAgain.size() == 3, "the big ship again, three periods");
    if(bigAgain.size() != 3) { return; }
    CHECK_REL(bigAgain[0].massStart, bigRows[0].massStart, 1e-12, "same first period");
    CHECK_REL(bigAgain[0].massEnd, bigRows[0].massEnd, 1e-12, "same outer drain");
    CHECK_REL(bigAgain[2].massEnd, bigRows[2].massEnd, 1e-12, "same core at the end");
    CHECK_REL(bigAgain[0].deltaV + bigAgain[1].deltaV + bigAgain[2].deltaV,
              bigRows[0].deltaV + bigRows[1].deltaV + bigRows[2].deltaV, 1e-12,
              "the same plan out of a buffer that grew back");
}

/* Fuel links name their endpoints by part ID, and the loaders turn that name into an
   index. #963 replaced the id -> index std::map with a sorted index vector, and the one
   rule that had to survive is WHICH part wins when an id is duplicated: `idIndex[id] = i`
   over each part in order left the LAST one, and upper_bound + a step back here does the
   same -- but ONLY because the sort tie-breaks on the index. std::sort is not stable, so
   comparing ids alone leaves a duplicate's place inside its block unspecified, and the
   answer then depends on the standard library's sort internals. The loaders refuse
   duplicate ids, so this is a kept rule rather than reachable state; it is pinned because
   a link resolving to the wrong part silently changes which fuel group the engine can
   reach, with no crash:
     last wins  : "dup" is part 17, which shares the engine's group, so the link joins a
                  group to itself and is dropped. 80 kg of the 225 aboard burns, dv 439.
     first wins : "dup" is part 0, one hop across the barrier, so its 30 kg burns too --
                  110 kg, and the pair of checks below reads 115 / 671 instead.
   The ship is deliberately 18 parts, not 5: libstdc++'s introsort sorts 16 or fewer
   elements with insertion sort, which happens to be stable, so a short fixture passes with
   OR without the tie-break and pins nothing. Measured -- an 18-part ship with the tie-break
   dropped fails the two pins below; the same mutant at 5 parts stayed green. */
static void test_a_duplicate_id_resolves_to_the_last_part() {
    Catalog cat;
    BuildShip bs;
    addPart(bs, cat.tank(10, 15, 15), "dup", -1, 1);            // 40 kg, 30 kg fuel
    addPart(bs, cat.barrier(5), "wall", 0, 1);                  // a fuel wall, not a decoupler
    addPart(bs, cat.tank(20, 30, 30), "core", 1, 1);            // 80 kg, 60 kg fuel
    addPart(bs, cat.engine(10, 0.5, 1000.0), "eng", 2, 1);      // in the core's group
    /* Ballast to push the part count over the insertion-sort threshold. Inert (no fuel)
       so it cannot change what drains, only what the hull weighs. */
    char bid[16];
    for(int i = 0; i < 13; i++) {
        std::snprintf(bid, sizeof bid, "ballast%02d", i);
        addPart(bs, cat.tank(5, 0, 0), bid, 2, 1);              // 5 kg each
    }
    addPart(bs, cat.tank(5, 10, 10), "dup", 2, 1);              // 25 kg, 20 kg fuel
    addLink(bs, "dup", "core");
    CHECK_TRUE(bs.parts.size() == 18, "18 parts, past the stable-sort coincidence");

    const double g = 10.0;
    const std::vector<StageRow> rows = computeStaging(bs, g);
    CHECK_TRUE(rows.size() == 1, "no decoupler, one period");
    if(rows.size() != 1) { return; }
    CHECK_REL(rows[0].massStart, 225.0, 1e-9, "all 18 parts aboard");
    CHECK_REL(rows[0].massEnd, 145.0, 1e-9, "only the engine's own group drained");
    CHECK_REL(rows[0].deltaV, 1000.0 * std::log(225.0 / 145.0), 1e-9, "last-wins dv");
    CHECK_TRUE(rows[0].engines == 1, "one engine");
}

/* What a stage label MEANS, derived from the def (shipdef.h, StageRole). #1009's dump
   prints this, and the Staging ladder leaves a part with NONE out of the table altogether
   (#1022) -- which is exactly why the derivation has to be right, since a wrong bit hides a
   part that fires or shows a row for one that does not. A label has three readings in
   flight and no more: a thruster lights when the descending counter reaches
   its label, a decoupler separates and a chute deploys only when it EQUALS the label.

   What is pinned is the DERIVATION, so the dump cannot lie about a part: the two halves
   of the thruster test are checked separately, and the engine half is checked against
   what the staging analysis beside it counts as lit. A fuel wall is deliberately NOT a
   role -- it splits fuel groups but a press can never fire it, so it reads "-" (see
   test_barrier_no_link for what walls do instead). */
static void test_a_stage_label_means_exactly_three_things() {
    Catalog cat;
    constexpr unsigned kEng = (unsigned)StageRole::Engine;
    constexpr unsigned kDec = (unsigned)StageRole::Decoupler;
    constexpr unsigned kChute = (unsigned)StageRole::Chute;

    CHECK_TRUE(stageRoles(nullptr) == 0u, "no def answers nothing");
    CHECK_TRUE(stageRoles(cat.tank(10, 5, 5)) == 0u, "a tank is not fireable");
    CHECK_TRUE(stageRoles(cat.barrier(4)) == 0u, "a fuel wall is not fireable");
    CHECK_TRUE(stageRoles(cat.mono(10, 20)) == 0u, "a hydrazine tank is not a thruster");
    CHECK_TRUE(stageRoles(cat.decoupler(5)) == kDec, "a decoupler separates");
    CHECK_TRUE(stageRoles(cat.chute(3, 20.0)) == kChute, "a chute deploys");
    CHECK_TRUE(stageRoles(cat.engine(10, 0.5, 1000.0)) == kEng, "an engine lights");
    CHECK_TRUE(stageRoles(cat.nuclearEngine(10, 0.5, 800.0)) == kEng,
               "one propellant is still a thruster");

    /* Both halves of the thruster test matter on their own: these are what a mis-typed
       table entry looks like, and both are inert mass that still costs a press. Built
       inline because no shipped part is shaped like either -- the def, not the name in
       the picker, is what decides. */
    PartDef veOnly;
    veOnly.mass = 10.0;
    veOnly.exhaust_velocity = 1000.0;
    CHECK_TRUE(stageRoles(&veOnly) == 0u, "an exhaust velocity with nothing to push");
    PartDef flowOnly;
    flowOnly.mass = 10.0;
    flowOnly.setPropellantRate(ResourceType::Hydrogen, 0.5);
    CHECK_TRUE(stageRoles(&flowOnly) == 0u, "a flow with nowhere to push it out");

    /* The two fakes are inert mass that still costs a press, and the analysis beside the
       dump agrees: they ride at the real engine's label and exactly one engine lights
       there. Note the reading is not identical on either side. Flight's gate is
       Part::isThruster (vehicle.cpp:6451, and the jet branch under it), so a jet engine
       answers "engine" to a press; the row's count is litRocketCount(), which excludes
       jets (staging.cpp:791). The dump follows flight, not the row. (This binary cannot
       instantiate a Part to compare against -- ~Part reaches Body, which would drag
       Bullet into a pure-math test.) */
    {
        BuildShip bs;
        const int tk = addPart(bs, cat.tank(10, 20, 20), "tk", -1, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "eng", tk, 1);
        addPart(bs, &veOnly, "ve_only", tk, 1);
        addPart(bs, &flowOnly, "flow_only", tk, 1);
        const std::vector<StageRow> rows = computeStaging(bs, 10.0);
        CHECK_TRUE(rows.size() == 1, "one label, one burn");
        if(rows.size() == 1) { CHECK_TRUE(rows[0].engines == 1, "only the real engine lit"); }
    }

    /* The words the dump and the window show. Order comes from the mask, not from
       whatever order the bits were set, so one build prints one string twice. */
    CHECK_TRUE(std::string(stageRoleName(0u)) == "-", "no role prints a dash");
    CHECK_TRUE(std::string(stageRoleName(kEng)) == "engine", "engine");
    CHECK_TRUE(std::string(stageRoleName(kDec)) == "decoupler", "decoupler");
    CHECK_TRUE(std::string(stageRoleName(kChute)) == "chute", "chute");
    CHECK_TRUE(std::string(stageRoleName(kEng | kDec)) == "engine+decoupler", "engine+decoupler");
    CHECK_TRUE(std::string(stageRoleName(kEng | kChute)) == "engine+chute", "engine+chute");
    CHECK_TRUE(std::string(stageRoleName(kChute | kDec)) == "decoupler+chute", "decoupler+chute");
    CHECK_TRUE(std::string(stageRoleName(kEng | kDec | kChute)) == "engine+decoupler+chute",
               "all three");
}

/* A stage label is a position in the press order, nothing more.

   Nobody had written this down, and a reader can easily assume the opposite: that a label
   is a height, so a booster labelled 301 instead of 2 carries a number the sim can never
   reach and the booster rides all the way up. It doesn't. The walk visits the labels the
   ship CARRIES (sim.stageLabels, sorted descending), and the setup pass lights every engine
   at maxStage before the first period, so a high label is first in flight order exactly
   like a low one: same two rows, same mass, same burn, same separation, same TWR at the
   pad. #517's sparse-label rule already pins the wide case (1,9000 == 1,2); this pins the
   narrow one -- a high label is not merely cheap, it is not late.

   What a high label DOES cost is a flight-side property, not one of this pass: the counter
   steps down one integer per press, so a gap is a press the player or the ascent law has to
   spend. Measured and ticketed as #1018. */
static void test_a_stage_label_is_order_not_height() {
    Catalog cat;
    const double g = 10.0;

    /* A two-stage stack, one engine under each label. The booster engine is sized to
       clear unity at the pad (4 kg/s * 1000 m/s = 4000 N against 325 kg at g = 10),
       because the last check below states that the TOP row is the pad burn and it lifts. */
    BuildShip dense;
    const int dt = addPart(dense, cat.tank(50, 25, 25), "tank_up", -1, 1);
    addPart(dense, cat.engine(10, 0.5, 1000.0), "eng_up", dt, 1);
    const int dd = addPart(dense, cat.decoupler(5), "dec", dt, 2);
    const int dl = addPart(dense, cat.tank(50, 75, 75), "tank_low", dd, 2);
    addPart(dense, cat.engine(10, 2.0, 1000.0), "eng_low", dl, 2);

    BuildShip tall = dense;   // the same physical ship, one label written 301
    for(BuildPart &p : tall.parts) {
        p.id = "t_" + p.id;
        if(p.stage == 2) { p.stage = 301; }
    }

    const std::vector<StageRow> rd = computeStaging(dense, g);
    const std::vector<StageRow> rt = computeStaging(tall, g);
    CHECK_TRUE(rd.size() == 2 && rt.size() == 2, "two periods either way");
    if(rd.size() != 2 || rt.size() != 2) { return; }

    CHECK_TRUE(rd[0].stage == 2, "the dense ladder's first row is label 2");
    CHECK_TRUE(rt[0].stage == 301, "the tall ladder's first row is label 301, not a skipped row");
    CHECK_TRUE(rt[1].stage == 1, "and the pad stage is still last");
    for(size_t i = 0; i < rd.size(); i++) { same_rows(rd[i], rt[i]); }

    /* The pad row, stated rather than compared: the 301 row is the one that has to lift
       the whole 325 kg, and it does. If a high label ever read as unreachable, this is the
       pair of numbers that would go to zero. */
    CHECK_REL(rt[0].massStart, 325.0, 1e-9, "301 ignites with everything aboard");
    CHECK_TRUE(rt[0].engines == 1 && rt[0].drops, "and it burns and separates");
    CHECK_TRUE(rt[0].minTWR > 1.0, "a 301 label still lifts off the pad");
}

/* validateStaging (#1010): a verdict on the authored ORDER, not on the cost. The
   fixtures below differ from a working ship by exactly one authored thing, and each
   says which press that breaks. Phrases are matched, not whole lines: the numbers in
   the line are the point and a reword should not redden the test, but the count of
   verdicts is pinned -- a checker that starts silent is the failure mode this has to
   catch. */
static bool has_warn(const std::vector<std::string> &w, const char *phrase) {
    for(const std::string &s : w) {
        if(s.find(phrase) != std::string::npos) { return true; }
    }
    return false;
}

static void test_the_staging_checker_names_the_presses_that_do_nothing() {
    Catalog cat;

    /* The control: labels 1 and 2, something answering at each, the decoupler well
       above the lowest label, the root part a tank. Nothing to say. */
    {
        BuildShip ok;
        const int t1 = addPart(ok, cat.tank(50, 25, 25), "up_tank", -1, 1);
        addPart(ok, cat.engine(10, 0.5, 1000.0), "up_eng", t1, 1);
        const int d = addPart(ok, cat.decoupler(5), "dec", t1, 2);
        const int t2 = addPart(ok, cat.tank(50, 75, 75), "lo_tank", d, 2);
        addPart(ok, cat.engine(10, 2.0, 1000.0), "lo_eng", t2, 2);
        std::vector<std::string> w;
        validateStaging(ok, w);
        CHECK_TRUE(w.empty(), "a dense ladder with a job at every label says nothing");
    }

    /* A gap. The same ship with the lower label written 5 instead of 2: the press
       order is unchanged (#1018's pin above), but the counter has to walk 4 down to
       2, and those three presses have nothing on the label that stands there. */
    {
        BuildShip bs;
        const int t1 = addPart(bs, cat.tank(50, 25, 25), "up_tank", -1, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "up_eng", t1, 1);
        const int d = addPart(bs, cat.decoupler(5), "dec", t1, 5);
        const int t2 = addPart(bs, cat.tank(50, 75, 75), "lo_tank", d, 5);
        addPart(bs, cat.engine(10, 2.0, 1000.0), "lo_eng", t2, 5);
        std::vector<std::string> w;
        validateStaging(bs, w);
        CHECK_TRUE(w.size() == 1, "one verdict, for the gap");
        CHECK_TRUE(has_warn(w, "3 of 4 presses do nothing"), "and it counts the presses");
        CHECK_TRUE(has_warn(w, "no part carries 2-4"), "one compressed run, not three lines");
    }

    /* A label carried by parts that answer to nothing. The press stands there, and
       the tank just rides: this is what a player cannot discover by flying. */
    {
        BuildShip bs;
        const int t1 = addPart(bs, cat.tank(50, 25, 25), "up_tank", -1, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "up_eng", t1, 1);
        addPart(bs, cat.tank(20, 10, 10), "rider", t1, 3);
        const int d = addPart(bs, cat.decoupler(5), "dec", t1, 2);
        const int t2 = addPart(bs, cat.tank(50, 75, 75), "lo_tank", d, 2);
        addPart(bs, cat.engine(10, 2.0, 1000.0), "lo_eng", t2, 2);
        std::vector<std::string> w;
        validateStaging(bs, w);
        CHECK_TRUE(w.size() == 1, "one verdict, for the silent label");
        CHECK_TRUE(has_warn(w, "1 of 2 presses do nothing"), "presses 3 and 2, one dead");
        CHECK_TRUE(has_warn(w, "nothing answers at 3"), "and it says the label IS carried");
    }

    /* A decoupler on the lowest label. The counter stops there, so an advancing press
       never stands on it: it needs one press more than stagesLeft() counts (#529). */
    {
        BuildShip bs;
        const int t1 = addPart(bs, cat.tank(50, 25, 25), "up_tank", -1, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "up_eng", t1, 1);
        const int d = addPart(bs, cat.decoupler(5), "dec", t1, 1);
        const int t2 = addPart(bs, cat.tank(50, 75, 75), "lo_tank", d, 2);
        addPart(bs, cat.engine(10, 2.0, 1000.0), "lo_eng", t2, 2);
        std::vector<std::string> w;
        validateStaging(bs, w);
        CHECK_TRUE(w.size() == 1, "one verdict, for the decoupler on the floor");
        CHECK_TRUE(has_warn(w, "a decoupler rides label 1, the lowest"), "and it names the label");
        CHECK_TRUE(has_warn(w, "needs a press past the ladder"), "and says why");
    }

    /* ...and the shape that IS correct authoring, which the rule above must not touch. A chute
       belongs at the end of the sequence: the player cuts the chutes with one press past the
       ladder, and deployChutes fires at whatever label stands (game.cpp:2640). Flagging it
       would put an orange verdict on every normal capsule and train the player to ignore the
       window. */
    {
        BuildShip bs;
        const int c1 = addPart(bs, cat.tank(50, 25, 25), "pod_tank", -1, 1);
        addPart(bs, cat.chute(2, 30.0), "pod_chute", c1, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "pod_eng", c1, 1);
        const int d = addPart(bs, cat.decoupler(5), "dec", c1, 2);
        const int t2 = addPart(bs, cat.tank(50, 75, 75), "lo_tank", d, 2);
        addPart(bs, cat.engine(10, 2.0, 1000.0), "lo_eng", t2, 2);
        std::vector<std::string> w;
        validateStaging(bs, w);
        CHECK_TRUE(w.empty(), "a chute on the lowest label is the way to author one");
    }

    /* A decoupler whose side is the whole ship. The press is refused and the label is
       spent anyway (#541), so it can never do its job. Named, because the part is what
       the player has to move. */
    {
        BuildShip bs;
        const int d = addPart(bs, cat.decoupler(5), "root_dec", -1, 2);
        const int t2 = addPart(bs, cat.tank(50, 75, 75), "lo_tank", d, 2);
        addPart(bs, cat.engine(10, 2.0, 1000.0), "lo_eng", t2, 2);
        std::vector<std::string> w;
        validateStaging(bs, w);
        CHECK_TRUE(w.size() == 1, "one verdict, for the whole-ship decoupler");
        CHECK_TRUE(has_warn(w, "decoupler 'root_dec' at label 2 would have to take the whole "
                               "ship with it"), "named by id, in the extract's own words");
        CHECK_TRUE(has_warn(w, "the label is spent"), "and says what the press costs");
    }

    /* ...and the false positive that rule would have had. A BuildShip is a forest in
       principle -- detachSubtree and graftTree hand out one-root views of a bigger build, and a
       test can build two stacks directly (this one does; a loaded SHIP file cannot, because
       shipdef.cpp:1024 parents each entry to the previous one unless told otherwise). So a
       parentless decoupler is NOT the same as a whole-ship decoupler: its side is only its own
       stack, and that separates fine. The rule therefore counts the subtree, the way
       extractSubtreeAsShip does. */
    {
        BuildShip bs;
        const int t1 = addPart(bs, cat.tank(50, 25, 25), "up_tank", -1, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "up_eng", t1, 1);
        const int d = addPart(bs, cat.decoupler(5), "booster_dec", -1, 2);   // a second tree
        const int t2 = addPart(bs, cat.tank(50, 75, 75), "lo_tank", d, 2);
        addPart(bs, cat.engine(10, 2.0, 1000.0), "lo_eng", t2, 2);
        std::vector<std::string> w;
        validateStaging(bs, w);
        CHECK_TRUE(w.empty(), "a parentless decoupler over one stack says nothing");
    }

    /* A label's roles are OR'd over the parts riding it, not taken from one of them. The
       control's working ship lists an inert part on a label that also carries an engine: if the
       scan kept only the last part's role, or only the first, this label would read inert and
       the player would be told to fix a stage that is fine. */
    {
        BuildShip bs;
        const int t1 = addPart(bs, cat.tank(50, 25, 25), "up_tank", -1, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "up_eng", t1, 1);
        const int d = addPart(bs, cat.decoupler(5), "dec", t1, 2);
        const int t2 = addPart(bs, cat.tank(50, 75, 75), "lo_tank", d, 2);
        addPart(bs, cat.engine(10, 2.0, 1000.0), "lo_eng", t2, 2);
        addPart(bs, cat.tank(20, 0, 0), "lo_strut", t2, 2);   // inert, listed LAST
        std::vector<std::string> w;
        validateStaging(bs, w);
        CHECK_TRUE(w.empty(), "an inert part riding a label that also burns says nothing");
    }

    /* The LOWEST label is never pressed, so parts riding it answer to nothing and that is not a
       fault: it is the payload, or a tank still feeding the engine one label up. Only labels
       above lo can be silent faults. */
    {
        BuildShip bs;
        const int t1 = addPart(bs, cat.tank(50, 25, 25), "pod_tank", -1, 1);   // nothing inert
        addPart(bs, cat.tank(50, 0, 0), "pod_ballast", t1, 1);                 // ... except this
        const int d = addPart(bs, cat.decoupler(5), "dec", t1, 2);
        const int t2 = addPart(bs, cat.tank(50, 75, 75), "lo_tank", d, 2);
        addPart(bs, cat.engine(10, 2.0, 1000.0), "lo_eng", t2, 2);
        std::vector<std::string> w;
        validateStaging(bs, w);
        CHECK_TRUE(w.empty(), "an inert part on the floor label is not a dead press");
    }

    /* Several gaps, listed as runs, and the run list capped so a ladder with a hole in every
       step cannot print a page. Labels 1,3,5,7,9,11: five single-integer runs, of which the
       line names four and admits the rest. */
    {
        BuildShip bs;
        const int t1 = addPart(bs, cat.tank(50, 25, 25), "up_tank", -1, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "up_eng", t1, 1);
        int prev = t1;
        for(int s = 3; s <= 11; s += 2) {
            prev = addPart(bs, cat.engine(10, 0.2, 1000.0), "eng_at_" + std::to_string(s), prev, s);
        }
        std::vector<std::string> w;
        validateStaging(bs, w);
        CHECK_TRUE(w.size() == 1, "one verdict however many runs");
        CHECK_TRUE(has_warn(w, "5 of 10 presses do nothing"), "and it still counts every press");
        CHECK_TRUE(has_warn(w, "no part carries 2, 4, 6, 8, ..."),
                   "four runs named, the rest admitted, not five lines");
    }

    /* The compressed-run wording has to close at the label that IS carried. This is the bug
       the first version shipped with: walking integers left a run open across a carried label,
       so 1/3/7 printed "2-6" and blamed label 3, which carries parts. */
    {
        BuildShip bs;
        const int t1 = addPart(bs, cat.tank(50, 25, 25), "up_tank", -1, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "up_eng", t1, 1);
        const int d = addPart(bs, cat.decoupler(5), "dec_3", t1, 3);
        addPart(bs, cat.engine(10, 0.4, 1000.0), "eng_3", d, 3);
        addPart(bs, cat.engine(10, 0.2, 1000.0), "eng_7", d, 7);
        std::vector<std::string> w;
        validateStaging(bs, w);
        CHECK_TRUE(w.size() == 1, "one verdict, for the two runs");
        CHECK_TRUE(has_warn(w, "no part carries 2, 4-6"),
                   "the run closes before the carried label, not at the top");
    }
}

/* The shared grouping (#1011): one pass that the checker, the Staging ladder (#1022) and
   the auto-stager all read, so the shape of it is pinned once rather than in three readers.
   The order is the load-bearing part -- groups ascending, parts in build order inside a
   group -- because a reader that walks the groups back-to-front is claiming to print the
   PRESS order. */
static void test_the_label_groups_read_as_press_order() {
    Catalog cat;
    BuildShip bs;
    // Deliberately out of label order in the array, so a reader cannot rely on insertion.
    const int t1 = addPart(bs, cat.tank(50, 25, 25), "up_tank", -1, 5);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "up_eng", t1, 5);
    const int d = addPart(bs, cat.decoupler(5), "dec", t1, 3);
    addPart(bs, cat.tank(20, 10, 10), "ballast", t1, 1);
    const int t2 = addPart(bs, cat.tank(50, 75, 75), "lo_tank", d, 1);
    addPart(bs, cat.engine(10, 2.0, 1000.0), "lo_eng", t2, 1);

    std::vector<StagingLabel> g = stagingLabels(bs);
    CHECK_TRUE(g.size() == 3, "three distinct labels");
    CHECK_TRUE(g[0].stage == 1 && g[1].stage == 3 && g[2].stage == 5, "ascending");
    CHECK_TRUE(g[0].parts.size() == 3 && g[1].parts.size() == 1
                   && g[2].parts.size() == 2,
               "every part in exactly one group");
    size_t seen = 0;
    for(const StagingLabel &l : g) { seen += l.parts.size(); }
    CHECK_TRUE(seen == bs.parts.size(), "no part dropped, none twice");
    // Build order holds inside a group, which in a real build is parent before child:
    // lo_tank (index 4) comes before its child lo_eng (5).
    CHECK_TRUE(g[0].parts[1] == 4 && g[0].parts[2] == 5, "build order inside a group");
    CHECK_TRUE((g[0].responds & (unsigned)StageRole::Engine) != 0u,
               "the floor label burns, so it is not dimmed");
    CHECK_TRUE((g[0].responds & (unsigned)StageRole::Decoupler) == 0u,
               "and it carries no decoupler");
    CHECK_TRUE(g[1].responds == (unsigned)StageRole::Decoupler,
               "the middle label carries only the decoupler");
    CHECK_TRUE(g[2].responds == (unsigned)StageRole::Engine, "the top label burns");

    /* And the two readers agree on the same pass: the groups' edges are exactly what the
       checker counts presses over. Labels 1/3/7 here means 4 presses, of which the two
       single-integer holes are dead. */
    std::vector<std::string> w;
    validateStaging(bs, w);
    CHECK_TRUE(w.size() == 1, "one verdict, for the two holes");
    CHECK_TRUE(has_warn(w, "2 of 4 presses do nothing"), "counted off the group edges");
    CHECK_TRUE(has_warn(w, "no part carries 2, 4"), "two runs, not one merged range");

    BuildShip empty;
    CHECK_TRUE(stagingLabels(empty).empty(), "an empty build groups to nothing");

    /* A part with no def still rides a label. Nothing live builds one -- the loader throws
       on an unknown def and the VAB mints parts from the catalog -- but stagingLabels is
       now the single source of the ladder's edges, so pin what a hole in it does rather
       than leave the answer to chance: the group exists, answers to nothing, and the
       checker spends presses on it like any other. */
    {
        BuildShip hole;
        addPart(hole, cat.tank(50, 25, 25), "tank", -1, 1);
        addPart(hole, cat.engine(10, 2.0, 1000.0), "eng", 0, 1);
        BuildPart ghost;
        ghost.id = "ghost";
        ghost.parent = 0;
        ghost.stage = 4;
        hole.parts.push_back(ghost);
        std::vector<StagingLabel> hg = stagingLabels(hole);
        CHECK_TRUE(hg.size() == 2 && hg[1].stage == 4 && hg[1].parts.size() == 1
                       && hg[1].responds == 0u,
                   "a def-less part still opens a label, and answers to nothing");
        std::vector<std::string> w;
        validateStaging(hole, w);
        CHECK_TRUE(w.size() == 1, "and the checker reads that ladder");
        CHECK_TRUE(has_warn(w, "3 of 3 presses do nothing"), "two holes and a silent top");
        CHECK_TRUE(has_warn(w, "no part carries 2-3"), "the run between the real labels");
        CHECK_TRUE(has_warn(w, "nothing answers at 4"), "and the label only a ghost rides");
    }
}

/* stagingPressNote (#1011 review): the few words that say where the counter stands. Nobody
   prints them since #1022 took the prose out of the Staging window -- this test is what keeps
   the arithmetic honest until #1015's number box takes the sentence (or someone deletes it).
   Short by design: whoever prints it prints into a narrow column. The two branches
   that are not "press N of M" are the ones a player cannot derive: the floor, which no
   advancing press ever reaches, and the one-label ship, where the first press DOES stand
   there (which is why #1010 rule 2 stays quiet about it). */
static void test_the_press_note_says_where_the_counter_stands() {
    /* Labels 1..5: the four advancing presses stand at 5, 4, 3, 2. */
    CHECK_TRUE(stagingPressNote(1, 5, 5) == "press 1 of 4", "the top label is press 1");
    CHECK_TRUE(stagingPressNote(1, 5, 4) == "press 2 of 4", "no off-by-one at the top");
    CHECK_TRUE(stagingPressNote(1, 5, 3) == "press 3 of 4", "a carried label in the middle");
    CHECK_TRUE(stagingPressNote(1, 5, 2) == "press 4 of 4", "the last advancing press");
    CHECK_TRUE(stagingPressNote(1, 5, 1) == "the floor",
               "the bottom label, which no advancing press stands on");

    /* A two-label ladder, the common case: one press, and it stands on the top label. */
    CHECK_TRUE(stagingPressNote(1, 2, 2) == "press 1 of 1", "two labels, one press");
    CHECK_TRUE(stagingPressNote(1, 2, 1) == "the floor", "and the bottom waits");

    /* One label: the counter has nowhere to advance, but the first press stands here. */
    CHECK_TRUE(stagingPressNote(1, 1, 1) == "only label: press 1", "a one-label ship");
    CHECK_TRUE(stagingPressNote(4, 4, 4) == "only label: press 1",
               "and the label values do not matter, only the span");
}

/* autoStage (#1012): the tree already knows the press order, so hand-numbering every part is
   the thing that goes wrong. What the plan has to get right, in the order the rules bite: cuts
   are ordered by the pair (the author's number, the tree's depth), the author first only when
   every cut carries one (#1020) and clamped so no cut rides a press inside a cut that contains
   it; a decoupler rides its OWN label (the press drops the decoupler with its stage,
   vehicle.cpp:5577); decouplers at one key are ONE press, not several (#540); what nothing
   separates is the floor, and the floor still has to carry a part after the pad-lit pin; and a
   plan that would re-sequence two engines the player ordered is refused rather than guessed. */
static std::vector<int> labelsOf(const BuildShip &bs) {
    std::vector<int> out;
    for(const BuildPart &p : bs.parts) { out.push_back(p.stage); }
    return out;
}

static bool has_note(const StagingPlan &plan, const char *phrase) {
    for(const std::string &n : plan.notes) {
        if(n.find(phrase) != std::string::npos) { return true; }
    }
    return false;
}

static void test_auto_stager_numbers_the_tree_from_its_decouplers() {
    Catalog cat;
    BuildShip bs;
    /* Placed, not authored: every part still carries the default label 1 (vab.cpp:320). */
    const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 1);
    const int dCore = addPart(bs, cat.decoupler(5), "d_core", pod, 1);
    const int upTank = addPart(bs, cat.tank(50, 25, 25), "up_tank", dCore, 1);
    const int upEng = addPart(bs, cat.engine(10, 0.5, 1000.0), "up_eng", upTank, 1);
    const int dBoost = addPart(bs, cat.decoupler(5), "d_boost", upEng, 1);
    const int boTank = addPart(bs, cat.tank(50, 75, 75), "bo_tank", dBoost, 1);
    const int boEng = addPart(bs, cat.engine(10, 0.5, 1000.0), "bo_eng", boTank, 1);

    StagingPlan plan = autoStage(bs);
    CHECK_TRUE(plan.applied, "a stack with two cuts numbers itself");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 2, 2, 2, 3, 3, 3}),
               "the floor, the core cut, the booster cut");
    CHECK_TRUE(plan.changes.size() == 6, "every part but the pod moved");
    if(plan.changes.size() != 6) { return; }   // CHECK does not bail; indexing it would only segfault
    CHECK_TRUE(plan.changes[0].part == dCore && plan.changes[0].from == 1 && plan.changes[0].to == 2,
               "and the change list says what it did to which part");
    CHECK_TRUE(plan.changes[0].id == "d_core",
               "and carries the id, which survives the part vector moving (#1013's undo)");
    /* The decoupler part rides the press that drops it, not the side it stays on. */
    CHECK_TRUE(bs.parts[dCore].stage == 2 && bs.parts[dBoost].stage == 3,
               "a decoupler separates itself (Vehicle::droppedPartsAtStage)");
    /* The order a press meets them: deepest cut first (label 3), then 2, and the floor is
       never stood on -- which is #1010 rule 2 made structural. */
    CHECK_TRUE(bs.parts[dBoost].stage > bs.parts[dCore].stage, "the deepest cut fires first");
    CHECK_TRUE(bs.parts[pod].stage == 1, "what nothing separates keeps the floor");
    CHECK_TRUE(bs.parts[upTank].stage == 2 && bs.parts[upEng].stage == 2,
               "a part takes the label of the cut that separates it");
    (void)boEng;

    std::vector<std::string> w;
    validateStaging(bs, w);
    CHECK_TRUE(w.empty(), "an auto-staged build carries no staging warning (#1010)");

    StagingPlan again = autoStage(bs);
    CHECK_TRUE(again.applied && again.changes.empty(), "numbered once: a second pass changes nothing");

    /* The engines were labelled together (all at the default), so the plan had no order to
       keep -- and it says which pair it pulled apart, because that IS a behaviour change. */
    CHECK_TRUE(has_note(plan, "light apart"), "it reports re-sequencing engines it was allowed to do");
    CHECK_TRUE(has_note(plan, "'up_eng' at 2") && has_note(plan, "'bo_eng' at 3"),
               "and names the pair with the labels it gave them");
}

static void test_auto_stager_gives_a_ring_one_press() {
    Catalog cat;
    BuildShip bs;
    const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 1);
    const int dA = addPart(bs, cat.decoupler(5), "d_a", pod, 1);
    const int dB = addPart(bs, cat.decoupler(5), "d_b", pod, 1);
    const int aTank = addPart(bs, cat.tank(50, 25, 25), "a_tank", dA, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "a_eng", aTank, 1);
    const int bTank = addPart(bs, cat.tank(50, 25, 25), "b_tank", dB, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "b_eng", bTank, 1);

    StagingPlan plan = autoStage(bs);
    CHECK_TRUE(plan.applied, "two side boosters number");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 2, 2, 2, 2, 2, 2}),
               "one depth is one press: a 2-ring is not two stages (#540)");

    /* A cut nested below one branch: it fires BEFORE the ring, so it takes the higher label,
       and the ring keeps one label between them. Built fresh, because adding it to the build
       above and re-running is a different question -- pinned below. */
    BuildShip nest;
    const int npod = addPart(nest, cat.mono(100, 0), "pod", -1, 1);
    const int nA = addPart(nest, cat.decoupler(5), "d_a", npod, 1);
    const int nB = addPart(nest, cat.decoupler(5), "d_b", npod, 1);
    const int nATank = addPart(nest, cat.tank(50, 25, 25), "a_tank", nA, 1);
    addPart(nest, cat.engine(10, 0.5, 1000.0), "a_eng", nATank, 1);
    const int nBTank = addPart(nest, cat.tank(50, 25, 25), "b_tank", nB, 1);
    addPart(nest, cat.engine(10, 0.5, 1000.0), "b_eng", nBTank, 1);
    const int dIn = addPart(nest, cat.decoupler(5), "d_inner", nBTank, 1);
    const int innerTank = addPart(nest, cat.tank(20, 10, 10), "inner_tank", dIn, 1);
    (void)innerTank;
    StagingPlan deeper = autoStage(nest);
    CHECK_TRUE(deeper.applied, "a nested cut numbers too");
    CHECK_TRUE(nest.parts[dIn].stage == 3, "the innermost cut fires first, so it is the top label");
    CHECK_TRUE(nest.parts[nA].stage == 2 && nest.parts[nB].stage == 2, "the ring stays one press");
    CHECK_TRUE(labelsOf(nest) == std::vector<int>({1, 2, 2, 2, 2, 2, 2, 3, 3}),
               "and everything below the inner cut rides it");

    /* The same edit on the build that was already numbered as a ring. The new cut carries the
       default label, so the ladder is half-numbered and DEPTH orders it (the inner cut first);
       and deepening the ladder takes the ring's engines off the top label they were lit from,
       which the plan now keeps instead of refusing (#1020). */
    const int dIn2 = addPart(bs, cat.decoupler(5), "d_inner", bTank, 1);
    addPart(bs, cat.tank(20, 10, 10), "inner_tank", dIn2, 1);
    (void)dIn2;
    StagingPlan edited = autoStage(bs);
    CHECK_TRUE(edited.applied, "deepening a numbered stack is a renumber, not a refusal");
    CHECK_TRUE(has_note(edited, "kept lit at the pad"),
               "and it says which engines it kept lit at the pad while doing it");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 2, 2, 2, 3, 2, 3, 3, 3}),
               "the ring keeps one press, the inner cut takes the new top label, and the "
               "booster engines ride that top label because the build lights them at the pad");
}

static void test_auto_stager_refuses_a_build_with_no_decoupler() {
    Catalog cat;
    BuildShip bs;
    const int tank = addPart(bs, cat.tank(50, 25, 25), "tank", -1, 3);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "eng", tank, 1);

    StagingPlan plan = autoStage(bs);
    CHECK_TRUE(!plan.applied, "no decoupler: there is nothing to assign");
    CHECK_TRUE(plan.changes.empty(), "and nothing moved");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({3, 1}), "the build is left exactly as it was");
    CHECK_TRUE(has_note(plan, "refuse:") && has_note(plan, "no decoupler"), "and it says why");
    /* The kind is what a one-line banner says with (#1027): this shape is a one-stage rocket,
       not a broken one, so the generic "can't" would be the wrong sentence. */
    CHECK_TRUE(plan.refusal == StagingPlan::Refusal::NoDecoupler,
               "and it names the refusal whose short verdict differs");

    std::vector<std::string> w;
    validateStaging(bs, w);
    CHECK_TRUE(!w.empty(), "refusing hides nothing: the checker still makes the same argument");
}

static void test_auto_stager_refuses_a_cut_that_would_drop_everything() {
    Catalog cat;
    BuildShip bs;
    const int d = addPart(bs, cat.decoupler(5), "d_root", -1, 1);   // the ROOT is a decoupler
    const int tank = addPart(bs, cat.tank(50, 25, 25), "tank", d, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "eng", tank, 1);

    StagingPlan plan = autoStage(bs);
    CHECK_TRUE(!plan.applied, "a decoupler whose side is every part cannot be labelled");
    CHECK_TRUE(has_note(plan, "whole ship"), "named as the reason");
    CHECK_TRUE(has_note(plan, "'d_root'"), "and named by part");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 1, 1}), "untouched");
    /* The default kind: the banner's generic line is the honest answer for a shape a short
       sentence cannot describe (#1027). */
    CHECK_TRUE(plan.refusal == StagingPlan::Refusal::Other,
               "a refusal the short verdict cannot name falls back to the generic one");
    (void)d;
}

static void test_auto_stager_keeps_two_engines_relative_to_each_other() {
    Catalog cat;
    /* Two cuts, so the stack says the low stage burns first and the mid stage after it. The
       build orders those two engines the other way round, which is a fact about intent the
       tree cannot see -- so the plan steps back and names both engines rather than picking. */
    BuildShip bs;
    const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 5);
    const int dUp = addPart(bs, cat.decoupler(5), "d_up", pod, 5);
    const int midTank = addPart(bs, cat.tank(50, 25, 25), "mid_tank", dUp, 5);
    const int midEng = addPart(bs, cat.engine(10, 0.5, 1000.0), "mid_eng", midTank, 3);
    const int dLow = addPart(bs, cat.decoupler(5), "d_low", midEng, 5);
    const int loTank = addPart(bs, cat.tank(50, 75, 75), "low_tank", dLow, 5);
    const int loEng = addPart(bs, cat.engine(10, 0.5, 1000.0), "low_eng", loTank, 1);
    (void)pod; (void)dUp; (void)midTank; (void)dLow; (void)loTank;

    StagingPlan plan = autoStage(bs);
    CHECK_TRUE(!plan.applied, "the build lights mid_eng first; the stack would light it last");
    CHECK_TRUE(has_note(plan, "refuse:") && has_note(plan, "'mid_eng'") && has_note(plan, "'low_eng'"),
               "and the refusal names both engines");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({5, 5, 5, 3, 5, 5, 1}), "untouched");
    CHECK_TRUE(bs.parts[midEng].stage > bs.parts[loEng].stage,
               "authored: mid_eng first, low_eng later -- the stack says the opposite");

    /* The same stack authored the way the stack reads it: accepted, and the core lights last. */
    BuildShip ok;
    const int t2 = addPart(ok, cat.tank(50, 25, 25), "core_tank", -1, 1);
    addPart(ok, cat.engine(10, 0.5, 1000.0), "core_eng", t2, 1);
    const int d2 = addPart(ok, cat.decoupler(5), "d_boost", t2, 4);
    addPart(ok, cat.tank(50, 75, 75), "bo_tank", d2, 4);
    addPart(ok, cat.engine(10, 0.5, 1000.0), "bo_eng", d2, 4);
    StagingPlan good = autoStage(ok);
    CHECK_TRUE(good.applied, "authored in the order the stack implies: the plan agrees");
    CHECK_TRUE(labelsOf(ok) == std::vector<int>({1, 1, 2, 2, 2}), "one cut: the floor and the cut");
    CHECK_TRUE(good.changes.size() == 3, "only the parts the cut takes moved");
    std::vector<std::string> w;
    validateStaging(ok, w);
    CHECK_TRUE(w.empty(), "and the result is clean");

    /* Two engines the build labelled together carry no order to keep -- that is the default a
       placed part gets, and refusing on it would refuse every stack out of the VAB -- so the
       plan may pull them apart, and says so (first test above). The reversal here is what it
       refuses. */
    CHECK_TRUE(good.notes.empty(), "an accepted renumber with no behaviour to warn about says nothing");
}

static void test_auto_stager_says_what_a_chute_on_the_floor_needs() {
    Catalog cat;
    BuildShip bs;
    const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 1);
    addPart(bs, cat.chute(5, 30.0), "chute", pod, 1);
    const int d = addPart(bs, cat.decoupler(5), "d_boost", pod, 1);
    const int boTank = addPart(bs, cat.tank(50, 75, 75), "bo_tank", d, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "bo_eng", boTank, 1);
    (void)boTank;

    StagingPlan plan = autoStage(bs);
    CHECK_TRUE(plan.applied, "a chute does not stop the build numbering");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 1, 2, 2, 2}),
               "the chute rides the hull it stays on, at the floor");
    CHECK_TRUE(has_note(plan, "chute") && has_note(plan, "past the ladder"),
               "and the plan says what a floor chute costs: one press past the ladder");
    std::vector<std::string> w;
    validateStaging(bs, w);
    CHECK_TRUE(w.empty(), "#1010 rule 2 does not call a floor chute a fault, and neither does this");
}

static void test_auto_stager_keeps_a_launch_lit_engine_at_the_pad() {
    Catalog cat;
    /* The shape heavy_asp ships with, miniature: a core that burns the whole flight while a
       booster falls away first. The tree says when a part FALLS and never when an engine
       STARTS, so the stack hands the core engine to a later press while the author lit it at
       launch. That is the one piece of ignition intent the plan cannot derive, so it keeps it
       (#1020) instead of refusing the way it used to. */
    BuildShip bs;
    const int coreTank = addPart(bs, cat.tank(50, 25, 25), "core_tank", -1, 1);
    const int coreEng = addPart(bs, cat.engine(10, 0.5, 1000.0), "core_eng", coreTank, 3);
    const int d = addPart(bs, cat.decoupler(5), "d_boost", coreTank, 2);
    const int boTank = addPart(bs, cat.tank(50, 75, 75), "bo_tank", d, 2);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "bo_eng", boTank, 3);
    (void)coreTank; (void)boTank;

    StagingPlan plan = autoStage(bs);
    CHECK_TRUE(plan.applied, "a launch-lit engine no longer stops the plan (#1020)");
    /* One cut, so hi = 2 and the counter starts there. Both engines the build lit from the pad
       end up on that label: the plan moved their numbers and kept what they MEAN. */
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 2, 2, 2, 2}),
               "the pad-lit engines land on hi and the tanks follow their cut");
    CHECK_TRUE(bs.parts[coreEng].stage == 2, "the core still lights at launch");
    CHECK_TRUE(has_note(plan, "kept lit at the pad"),
               "and the plan says out loud that it kept an ignition it cannot derive");
    std::vector<std::string> w;
    validateStaging(bs, w);
    CHECK_TRUE(w.empty(), "and the re-numbered stack leaves #1010's checker nothing to say");

    /* The same stack with nothing numbered: no intent to keep, so nothing is pinned -- and a
       booster engine moves ONTO launch, which the stack does imply (a group cut on the first
       press has to burn before it, or never burns at all). */
    BuildShip bare;
    const int t2 = addPart(bare, cat.tank(50, 25, 25), "core_tank", -1, 1);
    addPart(bare, cat.engine(10, 0.5, 1000.0), "core_eng", t2, 1);
    const int d2 = addPart(bare, cat.decoupler(5), "d_boost", t2, 1);
    addPart(bare, cat.tank(50, 75, 75), "bo_tank", d2, 1);
    const int boEng = addPart(bare, cat.engine(10, 0.5, 1000.0), "bo_eng", d2, 1);
    StagingPlan open = autoStage(bare);
    CHECK_TRUE(open.applied, "an un-numbered stack numbers");
    CHECK_TRUE(bare.parts[boEng].stage == 2, "the booster engine ends on launch (hi = 2)");
    CHECK_TRUE(labelsOf(bare) == std::vector<int>({1, 1, 2, 2, 2}), "floor and cut");
    CHECK_TRUE(!has_note(open, "kept lit at the pad"),
               "and a build that numbered nothing has no ignition to keep, so no such note");
}

/* #1020: the author's number orders the cuts; the tree's depth orders them only where the
   author said nothing. heavy_asp hangs six boosters off one core tank, all at ONE tree depth,
   and sequences them 6/5/4 so opposite boosters drop together -- a depth sort could not keep
   that and used to resequence it. */
static void test_auto_stager_keeps_an_authored_asparagus_sequence() {
    Catalog cat;
    /* pod / d_core / core tank + core engine / three radial boosters off the core tank: two
       outboard share a press (label 4), one inboard waits a press (3), and every engine is lit
       from the pad (4). Authored densely, so a plan that reads the shape right moves nothing. */
    BuildShip bs;
    const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 1);
    const int dCore = addPart(bs, cat.decoupler(5), "d_core", pod, 2);
    const int coreTank = addPart(bs, cat.tank(50, 25, 25), "core_tank", dCore, 2);
    const int coreEng = addPart(bs, cat.engine(10, 0.5, 1000.0), "core_eng", coreTank, 4);
    const int dOut1 = addPart(bs, cat.decoupler(5), "d_out1", coreTank, 4);
    const int bo1Tank = addPart(bs, cat.tank(50, 25, 25), "bo1_tank", dOut1, 4);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "bo1_eng", bo1Tank, 4);
    const int dOut2 = addPart(bs, cat.decoupler(5), "d_out2", coreTank, 4);
    const int bo2Tank = addPart(bs, cat.tank(50, 25, 25), "bo2_tank", dOut2, 4);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "bo2_eng", bo2Tank, 4);
    const int dIn1 = addPart(bs, cat.decoupler(5), "d_in1", coreTank, 3);
    const int bo3Tank = addPart(bs, cat.tank(50, 25, 25), "bo3_tank", dIn1, 3);
    const int bo3Eng = addPart(bs, cat.engine(10, 0.5, 1000.0), "bo3_eng", bo3Tank, 4);
    (void)pod; (void)dCore; (void)coreTank; (void)dOut1; (void)dOut2;

    StagingPlan plan = autoStage(bs);
    CHECK_TRUE(plan.applied, "an authored asparagus stack is numberable (#1020)");
    CHECK_TRUE(plan.changes.empty(), "and the plan agrees with the author: nothing to move");
    CHECK_TRUE(bs.parts[coreEng].stage == 4 && bs.parts[bo3Eng].stage == 4,
               "the pad-lit engines keep the top of a ladder whose hi the author set");
    CHECK_TRUE(bs.parts[dIn1].stage == 3, "the inboard booster waits a press for its cut");
    CHECK_TRUE(bs.parts[dOut1].stage == 4 && bs.parts[dOut2].stage == 4,
               "the outboard pair shares one press, as authored");
    CHECK_TRUE(has_note(plan, "2 engine(s) kept lit at the pad"),
               "exactly the two the tree would have dimmed, counted once");
    std::vector<std::string> w;
    validateStaging(bs, w);
    CHECK_TRUE(w.empty(), "an authored 4/3/2 ladder is dense and every rung answers");
}

/* Where the author's sequence and the tree's depth disagree, the author wins -- provided neither
   cut sits inside the other. heavy_asp is the case at scale: six boosters at ONE depth, three
   authored numbers. Here the two cuts are in different branches, so no press can swallow either
   one and the authored order is the answer; a depth sort would run the presses the other way
   round. The nested case is the test below, where no authored order is answerable. */
static void test_auto_stager_orders_cuts_by_the_authored_label_not_depth() {
    Catalog cat;
    BuildShip bs;
    const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 1);
    const int dHigh = addPart(bs, cat.decoupler(5), "d_shallow", pod, 5);   // depth 1
    addPart(bs, cat.tank(50, 25, 25), "tank_a", dHigh, 5);
    const int strut = addPart(bs, cat.mono(50, 1), "strut", pod, 3);        // depth 1
    const int dLow = addPart(bs, cat.decoupler(5), "d_deep", strut, 3);     // depth 2, other arm
    addPart(bs, cat.tank(50, 25, 25), "tank_b", dLow, 3);
    (void)pod; (void)strut;

    StagingPlan plan = autoStage(bs);
    CHECK_TRUE(plan.applied, "a numbered stack whose tree disagrees still numbers (#1020)");
    CHECK_TRUE(bs.parts[dHigh].stage == 3 && bs.parts[dLow].stage == 2,
               "the shallower cut fires first because the author put it higher, not last");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 3, 3, 1, 2, 2}),
               "and the strut between them is what nothing separates: the floor");
    std::vector<std::string> w;
    validateStaging(bs, w);
    CHECK_TRUE(w.empty(), "and re-spacing to 1..3 is dense, so the checker stays quiet");
}

/* No cut may ride a press INSIDE the press of a cut that contains it (#1020): the outer press
   takes the inner cut with it, so an authored ladder that runs outer-first across a nested pair
   is a sequence the stack cannot fly. The outer number is read down to the inner one and the
   depth tie-break splits them the only working way. The checker cannot see this shape at all --
   a decoupler does answer at the label it is left on -- so the plan has to. */
static void test_auto_stager_never_puts_a_cut_above_the_cut_that_contains_it() {
    Catalog cat;
    BuildShip bs;
    const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 1);
    const int dOuter = addPart(bs, cat.decoupler(5), "d_outer", pod, 5);      // depth 1
    const int tankA = addPart(bs, cat.tank(50, 25, 25), "tank_a", dOuter, 5);
    const int dInner = addPart(bs, cat.decoupler(5), "d_inner", tankA, 4);    // depth 2, inside
    const int tankB = addPart(bs, cat.tank(50, 25, 25), "tank_b", dInner, 4);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "eng", tankB, 4);
    (void)pod; (void)tankA; (void)tankB;

    StagingPlan plan = autoStage(bs);
    CHECK_TRUE(plan.applied, "an authored inversion is repaired, not refused");
    CHECK_TRUE(bs.parts[dInner].stage > bs.parts[dOuter].stage,
               "the inner cut separates first, whatever the numbers said");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 2, 2, 3, 3, 3}),
               "the outer cut drops to a press below the inner one");
    CHECK_TRUE(has_note(plan, "ordered above a cut they contain"),
               "and the plan says it bent an authored number instead of hiding it");
    std::vector<std::string> w;
    validateStaging(bs, w);
    CHECK_TRUE(w.empty(), "the checker stays quiet about exactly the thing this repairs (#1030)");
}

/* And the clamp reaches PAST the pair it repairs: bending an outer cut's number moves it relative
   to the unrelated cuts it used to sit beside. Here a sibling cut is authored 5, between the outer
   6 and the nested 2, so the unrepaired sort runs outer-sibling-inner and the repair runs
   sibling-inner-outer. The labels tell the two apart, which the previous test's shape does not. */
static void test_auto_stager_clamps_an_outer_cut_against_its_neighbours() {
    Catalog cat;
    BuildShip bs;
    const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 1);
    const int dOut = addPart(bs, cat.decoupler(5), "d_outer", pod, 6);      // depth 1
    const int tankA = addPart(bs, cat.tank(50, 25, 25), "tank_a", dOut, 6);
    const int dIn = addPart(bs, cat.decoupler(5), "d_inner", tankA, 2);     // depth 3, inside
    addPart(bs, cat.tank(50, 25, 25), "tank_b", dIn, 2);
    const int dSib = addPart(bs, cat.decoupler(5), "d_sibling", pod, 5);    // depth 1, other arm
    addPart(bs, cat.tank(50, 25, 25), "tank_c", dSib, 5);
    (void)pod; (void)tankA;

    StagingPlan plan = autoStage(bs);
    CHECK_TRUE(plan.applied, "the stack numbers -- with one authored number bent");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 2, 2, 3, 3, 4, 4}),
               "the sibling takes the top rung, the inner cut the next, the outer cut the last");
    CHECK_TRUE(bs.parts[dSib].stage == 4, "the sibling was never the problem and keeps its press");
    CHECK_TRUE(has_note(plan, "1 cut(s) ordered above a cut they contain"),
               "exactly one number bent, and the plan says which kind of bend it was");
    std::vector<std::string> w;
    validateStaging(bs, w);
    CHECK_TRUE(w.empty(), "three dense rungs, each answering to a cut");
}

/* #1031: the "Fuel link staging" switch -- the drain graph orders the cuts instead of the part
   tree. A core stage with three radial boosters chained into it (a -> b -> c -> core tank) is
   the shape most rockets will have, placed here without a number on anything. Off, the three
   cuts sit at one tree depth and share the first press. On, the chain says which fuel is still
   feeding the engine and from how far off: the outboard booster's is three hops away and burns
   first, so the ring comes apart into three presses -- the ladder heavy_asp's author typed by
   hand, derived instead of typed. */
static void test_auto_stager_orders_cuts_by_the_fuel_chain_when_asked() {
    Catalog cat;
    /* Built twice on purpose: a plan WRITES labels, and the next plan reads whatever labels the
       build carries as its authored intent. Asking the two questions on one build would let the
       second answer be shaped by the first, which is a separate (and filed) quirk, not this. */
    auto build = [&](BuildShip &bs) {
        const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 1);
        const int dCore = addPart(bs, cat.decoupler(5), "dec_core", pod, 1);
        const int coreTank = addPart(bs, cat.tank(50, 25, 25), "core_tank", dCore, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "core_eng", coreTank, 1);
        const int dA = addPart(bs, cat.decoupler(5), "dec_a", coreTank, 1);
        const int aTank = addPart(bs, cat.tank(50, 25, 25), "a_tank", dA, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "a_eng", aTank, 1);
        const int dB = addPart(bs, cat.decoupler(5), "dec_b", coreTank, 1);
        const int bTank = addPart(bs, cat.tank(50, 25, 25), "b_tank", dB, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "b_eng", bTank, 1);
        const int dC = addPart(bs, cat.decoupler(5), "dec_c", coreTank, 1);
        const int cTank = addPart(bs, cat.tank(50, 25, 25), "c_tank", dC, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "c_eng", cTank, 1);
        addLink(bs, "a_tank", "b_tank");
        addLink(bs, "b_tank", "c_tank");
        addLink(bs, "c_tank", "core_tank");
        (void)pod; (void)dCore; (void)coreTank; (void)aTank; (void)bTank; (void)cTank;
        return dCore;
    };

    BuildShip plain;
    build(plain);
    StagingPlan off = autoStage(plain);
    CHECK_TRUE(off.applied, "an un-numbered stack numbers either way");
    CHECK_TRUE(labelsOf(plain) == std::vector<int>({1, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3}),
               "off: the ring's three cuts share one press, and the core's cut sits below");
    CHECK_TRUE(!has_note(off, "ranked by the fuel chain"), "off says nothing about the chain");
    std::vector<std::string> wOff;
    validateStaging(plain, wOff);
    CHECK_TRUE(wOff.empty(), "off leaves the checker quiet (the ring's one press answers)");

    BuildShip bs;
    const int dCore = build(bs);
    StagingPlan on = autoStage(bs, true);
    CHECK_TRUE(on.applied, "the chain's answer is a plan too");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 2, 2, 2, 5, 5, 5, 4, 4, 4, 3, 3, 3}),
               "three hops, three presses, outboard first, and every part of a stage rides it");
    CHECK_TRUE(bs.parts[dCore].stage == 2,
               "the core ranks 0 -- it burns its own fuel -- so it stays a press below");
    CHECK_TRUE(has_note(on, "3 cut(s) ranked by the fuel chain"),
               "and the plan says the chain ordered them, not the tree");
    std::vector<std::string> w;
    validateStaging(bs, w);
    CHECK_TRUE(w.empty(), "four dense rungs, each answering to a cut");
}

/* The links are the only thing the switch can read, so on a stack without one it moves nothing
   and says nothing. That is what makes drawing the checkbox always (rather than only when the
   build has links) honest. */
static void test_auto_stager_fuel_link_switch_is_inert_without_links() {
    Catalog cat;
    BuildShip bs;
    const int tank = addPart(bs, cat.tank(50, 25, 25), "tank", -1, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "eng", tank, 1);
    const int d = addPart(bs, cat.decoupler(5), "d_boost", tank, 1);
    addPart(bs, cat.tank(50, 75, 75), "bo_tank", d, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "bo_eng", d, 1);
    (void)tank;
    StagingPlan ask = autoStage(bs, true);
    CHECK_TRUE(ask.applied, "the switch does not make a stack unnumberable");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 1, 2, 2, 2}), "and moves no label");
    CHECK_TRUE(!has_note(ask, "ranked by the fuel chain"), "nor claims the chain ordered anything");
}

/* A ring where only SOME of the boosters carry a link: the chain covers one of them, so it ranks
   one of them. The linked booster waits for its fuel to run down and rides the top press; the
   unlinked one burns its own tank, so the chain has no opinion and it ranks 0 with the core stage,
   where depth puts it. Recorded because "rank the whole ring off its linked neighbours" is a
   tempting shortcut that would make the unlinked booster wait for a fuel leg it never had. */
static void test_auto_stager_ranks_only_the_boosters_the_links_cover() {
    Catalog cat;
    auto build = [&](BuildShip &bs, bool linkA) {
        const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 1);
        const int dCore = addPart(bs, cat.decoupler(5), "dec_core", pod, 1);
        const int coreTank = addPart(bs, cat.tank(50, 25, 25), "core_tank", dCore, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "core_eng", coreTank, 1);
        const int dA = addPart(bs, cat.decoupler(5), "dec_a", coreTank, 1);
        const int aTank = addPart(bs, cat.tank(50, 25, 25), "a_tank", dA, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "a_eng", aTank, 1);
        const int dB = addPart(bs, cat.decoupler(5), "dec_b", coreTank, 1);
        const int bTank = addPart(bs, cat.tank(50, 25, 25), "b_tank", dB, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "b_eng", bTank, 1);
        if(linkA) { addLink(bs, "a_tank", "core_tank"); }
        (void)dCore; (void)coreTank; (void)dA; (void)aTank; (void)dB; (void)bTank;
    };

    BuildShip plain;
    build(plain, false);
    autoStage(plain);
    CHECK_TRUE(labelsOf(plain) == std::vector<int>({1, 2, 2, 2, 3, 3, 3, 3, 3, 3}),
               "no links: both boosters share one press");

    BuildShip bs;
    build(bs, true);
    StagingPlan on = autoStage(bs, true);
    CHECK_TRUE(on.applied, "a chain over one booster is still a plan");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 2, 2, 2, 4, 4, 4, 3, 3, 3}),
               "only the linked booster moves up; the unlinked one and the core stay put");
    CHECK_TRUE(has_note(on, "1 cut(s) ranked by the fuel chain"), "one opinion, counted");
    std::vector<std::string> w;
    validateStaging(bs, w);
    CHECK_TRUE(w.empty(), "three dense rungs, each answering to a cut");
}

/* The chain ranks a stage by fuel it hands to an engine OUTSIDE itself. Point the chain the
   other way -- the core feeds the boosters, so every engine burns inside the stage it sits on --
   and the switch has nothing to say: no rank, no note, and the same ladder the switch-off rule
   gives. This is the half of the rule the ring tests cannot show, because there every chain runs
   outward. */
static void test_auto_stager_ignores_a_chain_that_hands_fuel_no_further_inboard() {
    Catalog cat;
    BuildShip bs;
    const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 1);
    const int dCore = addPart(bs, cat.decoupler(5), "dec_core", pod, 1);
    const int coreTank = addPart(bs, cat.tank(50, 25, 25), "core_tank", dCore, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "core_eng", coreTank, 1);
    const int dA = addPart(bs, cat.decoupler(5), "dec_a", coreTank, 1);
    const int aTank = addPart(bs, cat.tank(50, 25, 25), "a_tank", dA, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "a_eng", aTank, 1);
    const int dB = addPart(bs, cat.decoupler(5), "dec_b", coreTank, 1);
    const int bTank = addPart(bs, cat.tank(50, 25, 25), "b_tank", dB, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "b_eng", bTank, 1);
    (void)pod; (void)dCore; (void)coreTank; (void)dA; (void)dB;
    addLink(bs, "core_tank", "a_tank");   // outward: the trunk feeds the branches
    addLink(bs, "core_tank", "b_tank");
    StagingPlan ask = autoStage(bs, true);
    CHECK_TRUE(ask.applied, "an outward chain is still a stack");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 2, 2, 2, 3, 3, 3, 3, 3, 3}),
               "and the ring shares one press, exactly as it does without links");
    CHECK_TRUE(!has_note(ask, "ranked by the fuel chain"), "the chain offered no opinion");
}

/* Links AND typed numbers -- the case the whole feature turns on, and the one no fixture had.
   The author numbered the ring 3/4/5 in the order OPPOSITE to the drain chain (which would put
   the outboard booster on the top press), and left the core's cut on 2 because it goes last --
   a ladder that answers to containment, so nothing here gets bent. A typed number beats the
   tree's guesses (#1020), and it beats the chain for the same reason: the author can be wrong
   about physics, but the number is the only statement of intent there is, and ranking over it
   would also split a ring they left on one label deliberately. So the switch is inert here, and
   says so. */
static void test_auto_stager_keeps_typed_cut_numbers_ahead_of_the_fuel_chain() {
    Catalog cat;
    auto build = [&](BuildShip &bs) {
        const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 1);
        const int dCore = addPart(bs, cat.decoupler(5), "dec_core", pod, 2);
        const int coreTank = addPart(bs, cat.tank(50, 25, 25), "core_tank", dCore, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "core_eng", coreTank, 1);
        const int dA = addPart(bs, cat.decoupler(5), "dec_a", coreTank, 3);   // chain: 3 hops
        const int aTank = addPart(bs, cat.tank(50, 25, 25), "a_tank", dA, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "a_eng", aTank, 1);
        const int dB = addPart(bs, cat.decoupler(5), "dec_b", coreTank, 4);   // chain: 2 hops
        const int bTank = addPart(bs, cat.tank(50, 25, 25), "b_tank", dB, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "b_eng", bTank, 1);
        const int dC = addPart(bs, cat.decoupler(5), "dec_c", coreTank, 5);   // chain: 1 hop
        const int cTank = addPart(bs, cat.tank(50, 25, 25), "c_tank", dC, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "c_eng", cTank, 1);
        (void)pod; (void)coreTank; (void)aTank; (void)bTank; (void)cTank;
        addLink(bs, "a_tank", "b_tank");
        addLink(bs, "b_tank", "c_tank");
        addLink(bs, "c_tank", "core_tank");
    };

    BuildShip off;
    build(off);
    autoStage(off);
    const std::vector<int> offLabels = labelsOf(off);
    CHECK_TRUE(offLabels == std::vector<int>({1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5}),
               "the typed order is the ladder: c off first, the core's cut last");

    BuildShip bs;
    build(bs);
    StagingPlan on = autoStage(bs, true);
    CHECK_TRUE(labelsOf(bs) == offLabels, "and the chain does not move a numbered cut");
    CHECK_TRUE(has_note(on, "ranked nothing -- every cut already carries a number"),
               "the switch explains why it did nothing");
    std::vector<std::string> w;
    validateStaging(bs, w);
    CHECK_TRUE(w.empty(), "the ladder stays legal either way");
}

/* "Stage from scratch" (#1035) in the shape that started it: heavy_asp's first engine sits on a
   part nothing ever cuts away, so the only label the tree can give it is 1 -- lit at every press.
   Typing 3 on it says "light it later", which the tree cannot produce, and the default path says so
   rather than overwriting the number. With the switch, the plan reads the build as if it had just
   been placed: the number is replaced, the ladder comes from the tree, and the refusal is gone. */
static void test_auto_stager_from_scratch_drops_numbers_it_cannot_honor() {
    Catalog cat;
    auto build = [&](BuildShip &bs) {
        const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "engine_1", pod, 3);   // lit later, the player says
        const int d = addPart(bs, cat.decoupler(5), "dec_x", pod, 2);
        const int tank = addPart(bs, cat.tank(50, 25, 25), "x_tank", d, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "engine_2", tank, 2);
        (void)pod; (void)d; (void)tank;
    };

    BuildShip bs;
    build(bs);
    StagingPlan keep = autoStage(bs);
    CHECK_TRUE(!keep.applied, "as numbered, the tree and the ignition order cannot both hold");
    CHECK_TRUE(has_note(keep, "lights before"), "and the plan names the pair it would re-sequence");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 3, 2, 1, 2}), "a refusal touches nothing");

    BuildShip fresh;
    build(fresh);
    StagingPlan scratch = autoStage(fresh, false, true);
    CHECK_TRUE(scratch.applied, "asked to start over, it numbers the stack");
    CHECK_TRUE(labelsOf(fresh) == std::vector<int>({1, 1, 2, 2, 2}),
               "the floor engine goes back to the floor, which is all the tree can do with it");
    CHECK_TRUE(has_note(scratch, "staged from scratch"), "and the log says nothing was protected");
    bool engineReverted = false;
    for(const StagingChange &c : scratch.changes) {
        if(c.id == "engine_1" && c.from == 3 && c.to == 1) { engineReverted = true; }
    }
    CHECK_TRUE(engineReverted, "the change record still carries the REAL replaced number, so Undo works");
    std::vector<std::string> w;
    validateStaging(fresh, w);
    CHECK_TRUE(w.empty(), "and the ladder it wrote is legal");

    /* The commit's promise is "one click recovers them", so exercise it rather than infer it from
       the change list: this mode is the one that destroys a ladder on purpose. */
    StagingUndo stack;
    recordStagingEdit(stack, scratch.changes, false);
    std::vector<std::string> un;
    CHECK_TRUE(undoStagingEdit(fresh, stack, un) == scratch.changes.size(),
               "one Undo takes the whole plan back");
    CHECK_TRUE(labelsOf(fresh) == std::vector<int>({1, 3, 2, 1, 2}),
               "and the numbers the mode ignored are the ones that come back");
}

/* The other half: a hand-numbered ladder. With the switch the author's cut order is not kept (that
   is the point of asking), and depth decides -- the whole ring on one press. With BOTH switches the
   two compose: from scratch makes the ladder unread-as-intent, and the chain then orders the ring
   one press apart, which is the ladder #1031 derives for a build nobody numbered. Built three times
   because a plan WRITES labels and the next plan would read them. */
static void test_auto_stager_from_scratch_replaces_a_hand_numbered_ladder() {
    Catalog cat;
    auto build = [&](BuildShip &bs) {
        const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 1);
        const int dCore = addPart(bs, cat.decoupler(5), "dec_core", pod, 2);
        const int coreTank = addPart(bs, cat.tank(50, 25, 25), "core_tank", dCore, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "core_eng", coreTank, 1);
        const int dA = addPart(bs, cat.decoupler(5), "dec_a", coreTank, 3);
        const int aTank = addPart(bs, cat.tank(50, 25, 25), "a_tank", dA, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "a_eng", aTank, 1);
        const int dB = addPart(bs, cat.decoupler(5), "dec_b", coreTank, 4);
        const int bTank = addPart(bs, cat.tank(50, 25, 25), "b_tank", dB, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "b_eng", bTank, 1);
        const int dC = addPart(bs, cat.decoupler(5), "dec_c", coreTank, 5);
        const int cTank = addPart(bs, cat.tank(50, 25, 25), "c_tank", dC, 1);
        addPart(bs, cat.engine(10, 0.5, 1000.0), "c_eng", cTank, 1);
        (void)pod; (void)dCore; (void)coreTank; (void)dA; (void)aTank;
        (void)dB; (void)bTank; (void)dC; (void)cTank;
        addLink(bs, "a_tank", "b_tank");
        addLink(bs, "b_tank", "c_tank");
        addLink(bs, "c_tank", "core_tank");
    };

    BuildShip keep;
    build(keep);
    autoStage(keep);
    CHECK_TRUE(labelsOf(keep) == std::vector<int>({1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5}),
               "unread, the authored ring keeps its three presses");

    BuildShip bs;
    build(bs);
    StagingPlan scratch = autoStage(bs, false, true);
    CHECK_TRUE(scratch.applied, "asked to start over, it numbers the ring from the tree");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3}),
               "one press for the ring -- depth cannot tell three chained boosters apart");
    CHECK_TRUE(!has_note(scratch, "ranked by the fuel chain"), "the chain was not asked");

    BuildShip both;
    build(both);
    StagingPlan bothPlan = autoStage(both, true, true);
    CHECK_TRUE(bothPlan.applied, "and both switches together number it too");
    CHECK_TRUE(labelsOf(both) == std::vector<int>({1, 2, 2, 2, 5, 5, 5, 4, 4, 4, 3, 3, 3}),
               "from scratch lets the chain order a ring the author had ordered the other way");
    CHECK_TRUE(has_note(bothPlan, "3 cut(s) ranked by the fuel chain"), "and says the chain did it");
    std::vector<std::string> w;
    validateStaging(both, w);
    CHECK_TRUE(w.empty(), "four dense rungs, each answering to a cut");
}

/* A chain can rank a stage further from an engine than a cut NESTED inside it, and that order
   cannot fly either -- the outer press takes the inner cut with it. The same repair as the
   authored clamp applies to the rank: the outer rank comes down to the inner one, the depth
   tie-break puts the inner cut first, and the plan counts what it bent. Shape: the outer
   stage's booster feeds an engine ABOVE the outer stage (two hops), while the innermost cut's
   own stage feeds nothing -- so the raw chain would fire the outer stage before the cut inside
   it. */
static void test_auto_stager_clamps_a_fuel_rank_above_a_cut_it_contains() {
    Catalog cat;
    BuildShip bs;
    const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 1);
    const int engTop = addPart(bs, cat.engine(10, 0.5, 1000.0), "eng_top", pod, 1);
    const int decO = addPart(bs, cat.decoupler(5), "dec_outer", engTop, 1);
    const int tankO = addPart(bs, cat.tank(50, 25, 25), "tank_o", decO, 1);
    const int decA = addPart(bs, cat.decoupler(5), "dec_a", tankO, 1);
    const int tankA = addPart(bs, cat.tank(50, 25, 25), "a_tank", decA, 1);
    const int decI = addPart(bs, cat.decoupler(5), "dec_inner", tankA, 1);
    addPart(bs, cat.tank(50, 25, 25), "inner_tank", decI, 1);
    addLink(bs, "a_tank", "tank_o");
    addLink(bs, "tank_o", "pod");   // the leg that leaves the outer stage, up to eng_top's group
    (void)pod; (void)engTop; (void)tankO; (void)tankA;

    StagingPlan plan = autoStage(bs, true);
    CHECK_TRUE(plan.applied, "the shape numbers -- with the chain bent");
    CHECK_TRUE(bs.parts[decI].stage == 4 && bs.parts[decA].stage == 3 && bs.parts[decO].stage == 2,
               "inner first, then its container, then the outer stage -- depth, not the chain");
    CHECK_TRUE(has_note(plan, "2 cut(s) ordered above a cut they contain"),
               "the two ranks the clamp had to bring down");
    std::vector<std::string> w;
    validateStaging(bs, w);
    CHECK_TRUE(w.empty(), "and a ladder with no dead press in it");
}

/* The one protection "Stage from scratch" removes that no label vector shows on its own: the
   pad-lit pin. Same shape as the test below -- a root engine and one under a cut, both authored at
   the top label so the build lights them at the pad. The default path lifts the safe one; the mode
   lifts neither, because "this engine lights at the pad" is itself one of the numbers the player
   said not to read. Pinned so a future edit that reads ship.parts[e].stage in the pin loop instead
   of authored[e] cannot pass the suite while quietly changing what the mode means. */
static void test_auto_stager_from_scratch_declines_the_pad_lit_pin() {
    Catalog cat;
    auto build = [&](BuildShip &bs) {
        const int topEng = addPart(bs, cat.engine(10, 0.5, 1000.0), "top_eng", -1, 5);
        const int dA = addPart(bs, cat.decoupler(5), "d_a", topEng, 3);
        const int tankA = addPart(bs, cat.tank(50, 25, 25), "tank_a", dA, 3);
        const int safeEng = addPart(bs, cat.engine(10, 0.5, 1000.0), "safe_eng", tankA, 5);
        const int dB = addPart(bs, cat.decoupler(5), "d_b", tankA, 4);
        addPart(bs, cat.tank(50, 25, 25), "tank_b", dB, 4);
        (void)topEng; (void)dA; (void)tankA; (void)safeEng; (void)dB;
    };

    BuildShip keep;
    build(keep);
    StagingPlan pin = autoStage(keep);
    CHECK_TRUE(pin.applied && labelsOf(keep) == std::vector<int>({1, 2, 2, 3, 3, 3}),
               "read as intent, the engine under the cut is lifted to the pad label");
    CHECK_TRUE(has_note(pin, "kept lit at the pad"), "and the plan says it did that");

    BuildShip bs;
    build(bs);
    StagingPlan scratch = autoStage(bs, false, true);
    CHECK_TRUE(scratch.applied, "the mode numbers the same stack");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 2, 2, 2, 3, 3}),
               "and leaves that engine where its own stage puts it -- it lights at 2, not at the pad");
    CHECK_TRUE(!has_note(scratch, "kept lit at the pad"), "no pin, so no note about one");
    std::vector<std::string> w;
    validateStaging(bs, w);
    CHECK_TRUE(w.empty(), "a different ladder, still a legal one");
}

/* The pad-lit pin gives up rather than empty the floor -- and gives up only for the engines
   standing there. A part reaches the floor only when no cut is above it, so the root engine below
   is all label 1 carries; lifting it would leave the lowest label the build carries being a cut,
   which is #1010 rule 2 -- the fault the plan refuses to number into existence wherever else it
   meets it. The other pad-lit engine hangs under a cut, cannot hurt the floor, and is pinned
   anyway. */
static void test_auto_stager_gives_up_the_pad_pin_rather_than_empty_the_floor() {
    Catalog cat;
    BuildShip bs;
    const int topEng = addPart(bs, cat.engine(10, 0.5, 1000.0), "top_eng", -1, 5);  // the ROOT
    const int dA = addPart(bs, cat.decoupler(5), "d_a", topEng, 3);
    const int tankA = addPart(bs, cat.tank(50, 25, 25), "tank_a", dA, 3);
    const int safeEng = addPart(bs, cat.engine(10, 0.5, 1000.0), "safe_eng", tankA, 5);
    const int dB = addPart(bs, cat.decoupler(5), "d_b", tankA, 4);
    addPart(bs, cat.tank(50, 25, 25), "tank_b", dB, 4);
    (void)dA; (void)tankA; (void)dB;

    StagingPlan plan = autoStage(bs);
    CHECK_TRUE(plan.applied, "the plan runs -- it just declines one pin");
    CHECK_TRUE(bs.parts[topEng].stage == 1, "the root engine stays on the floor, where it is needed");
    CHECK_TRUE(bs.parts[safeEng].stage == 3, "and the engine that cannot hurt the floor is pinned");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 2, 2, 3, 3, 3}),
               "one floor part, two rungs, and the pinning that was safe about it");
    CHECK_TRUE(has_note(plan, "stay on label 1"), "it says which engines it declined to lift");
    CHECK_TRUE(has_note(plan, "a press past the ladder"),
               "and what that costs: as numbered, they light only past the last rung");
    std::vector<std::string> w;
    validateStaging(bs, w);
    CHECK_TRUE(w.empty(), "which is the point: an empty label 1 would put a cut on the floor");
}

/* The plan walks the parent GRAPH rather than the part index, and refuses when the graph
   cannot be read. Each shape below is one the extract or the checker already has to survive;
   an index-based walk gets them wrong quietly. */
static void test_auto_stager_reads_the_graph_not_the_part_order() {
    Catalog cat;
    /* Built in the wrong order on purpose: the booster engine is listed BEFORE its parent,
       which an index pass would read as a second root -- and then it would sit on the floor
       while the cut that flight drops it with sits one press above. */
    BuildShip bs;
    const int eng = addPart(bs, cat.engine(10, 0.5, 1000.0), "bo_eng", 2, 1);   // parent = index 2
    const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 1);
    const int d = addPart(bs, cat.decoupler(5), "d_boost", pod, 1);
    (void)pod;

    StagingPlan plan = autoStage(bs);
    CHECK_TRUE(plan.applied, "a part listed before its parent still numbers");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({2, 1, 2}),
               "the graph decides who hangs off whom, not the list order");
    CHECK_TRUE(plan.changes.size() == 2 && plan.changes[0].id == "bo_eng",
               "and the change list names what it moved");
    (void)eng; (void)d;
}

static void test_auto_stager_refuses_a_parent_chain_that_loops() {
    Catalog cat;
    BuildShip bs;
    const int d = addPart(bs, cat.decoupler(5), "d_loop", 0, 1);   // its own parent
    addPart(bs, cat.tank(50, 25, 25), "tank", d, 1);
    (void)d;

    StagingPlan plan = autoStage(bs);
    CHECK_TRUE(!plan.applied, "a chain that loops cannot be walked, so it cannot be numbered");
    CHECK_TRUE(has_note(plan, "parent chain that loops"), "and it says so in those words");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 1}), "untouched");

    std::vector<std::string> w;
    validateStaging(bs, w);
    CHECK_TRUE(has_warn(w, "parent chain that loops"),
               "the checker sees the same thing: one graph, one answer, no assert in between");
}

static void test_auto_stager_refuses_a_forest_whose_every_root_is_a_cut() {
    Catalog cat;
    /* Two independent stacks, each entered through a decoupler, one with a nested cut. No
       part would carry label 1, so the LOWEST label the build carries would be a cut -- the
       exact thing #1010 rule 2 reports. A plan that applies has to leave the checker quiet,
       so this shape is refused rather than numbered into a ship whose bottom cut needs a
       press past the ladder. */
    BuildShip bs;
    const int dA = addPart(bs, cat.decoupler(5), "d_a", -1, 1);
    addPart(bs, cat.tank(50, 25, 25), "a_tank", dA, 1);
    const int dB = addPart(bs, cat.decoupler(5), "d_b", -1, 1);
    const int bTank = addPart(bs, cat.tank(50, 25, 25), "b_tank", dB, 1);
    const int dIn = addPart(bs, cat.decoupler(5), "d_inner", bTank, 1);
    addPart(bs, cat.tank(20, 10, 10), "inner_tank", dIn, 1);
    (void)dB; (void)bTank; (void)dIn;

    StagingPlan plan = autoStage(bs);
    CHECK_TRUE(!plan.applied, "nothing would hold the floor, so the plan steps back");
    CHECK_TRUE(has_note(plan, "every root"), "and names the shape");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 1, 1, 1, 1, 1}), "untouched");

    /* The labels it would have produced, set by hand, are what the checker then objects to:
       proof the refusal is about a fault a player would fly, not a tidy rule. */
    for(BuildPart &p : bs.parts) { p.stage = 2; }
    bs.parts[4].stage = 3;
    bs.parts[5].stage = 3;
    std::vector<std::string> w;
    validateStaging(bs, w);
    CHECK_TRUE(has_warn(w, "a decoupler rides label 2, the lowest"),
               "the shape it refuses is the shape rule 2 reports");
}

static void test_auto_stager_refuses_more_cuts_than_labels_exist() {
    Catalog cat;
    /* A non-decoupler root and then one decoupler per depth: the ladder wants
       hi = kMaxStage + 1, and labels run 1..kMaxStage on both sides of a save
       (shipdef.cpp:1192-1201 throws for the rest). Also a check that the tree walk is
       iterative -- a recursive one dies here instead of answering. */
    BuildShip bs;
    int par = addPart(bs, cat.tank(10, 0, 0), "root_tank", -1, 1);
    for(int i = 0; i < kMaxStage; i++) { par = addPart(bs, cat.decoupler(1), "d", par, 1); }
    CHECK_TRUE((int)bs.parts.size() == kMaxStage + 1, "the build is the size it says it is");

    StagingPlan plan = autoStage(bs);
    CHECK_TRUE(!plan.applied, "the label range runs out before the cuts do");
    CHECK_TRUE(has_note(plan, "labels a ship can carry"), "and says the range is the reason");
    CHECK_TRUE(par >= 0, "the chain was built");
}

/* undoStagingEdit + recordStagingEdit (#1013): the VAB cannot undo placing or deleting a
   part, and does not try here. It CAN undo a LABEL, because a staging edit is nothing but
   the list of (part, from, to) it performed -- which is the shape autoStage already returns.
   What has to hold for that to be safe at a mouse button: an undo lands every label back
   where it was; it reaches parts by ID, because placing, deleting and detaching renumber the
   vector under a change list recorded earlier; a part that has gone, or whose label moved
   since, is reported instead of guessed for; and the edit's record moves stagingSignature,
   which is the ticket's "verify the window refreshes rather than assume it". */
static bool has_line(const std::vector<std::string> &v, const char *phrase) {
    for(const std::string &s : v) {
        if(s.find(phrase) != std::string::npos) { return true; }
    }
    return false;
}

/* Stand-in for BuildShip::removePart deleting a LEAF: the same index remap, without the
   pose solve that function ends in (which needs real attach-node names, and these
   hand-built parts have none). What the undo cares about is that the indices moved. */
static void eraseLeaf(BuildShip &bs, int idx) {
    bs.parts.erase(bs.parts.begin() + idx);
    for(BuildPart &p : bs.parts) {
        if(p.parent > idx) { p.parent--; }
    }
}

static void test_undo_takes_an_auto_stage_back_to_the_authored_labels() {
    Catalog cat;
    BuildShip bs;
    const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 1);
    addPart(bs, cat.decoupler(5), "d_core", pod, 1);
    addPart(bs, cat.tank(50, 25, 25), "up_tank", 1, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "up_eng", 2, 1);
    addPart(bs, cat.decoupler(5), "d_boost", 3, 1);
    addPart(bs, cat.tank(50, 75, 75), "bo_tank", 4, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "bo_eng", 5, 1);

    const std::vector<int> authored = labelsOf(bs);
    const uint64_t sigBefore = stagingSignature(bs, 9.81, 1.0);

    StagingPlan plan = autoStage(bs);
    CHECK_TRUE(plan.applied && plan.changes.size() == 6, "a stack numbered from the tree, to take back");
    StagingUndo stack;
    recordStagingEdit(stack, plan.changes, false);
    CHECK_TRUE(stack.size() == 1, "a plan is ONE undo step, not one per part");

    std::vector<std::string> notes;
    const size_t back = undoStagingEdit(bs, stack, notes);
    CHECK_TRUE(back == 6, "six parts came back");
    CHECK_TRUE(labelsOf(bs) == authored, "each to the label it had before the plan ran");
    CHECK_TRUE(stack.empty(), "one step, one undo -- the same edit cannot be undone twice");
    CHECK_TRUE(notes.empty(), "and nothing had to be reported on the way back");

    /* The Staging window caches its rows, its verdicts, its label groups and its id map
       against this hash (#293). An undo that left the hash alone would keep drawing the
       build the player just left. */
    CHECK_TRUE(stagingSignature(bs, 9.81, 1.0) == sigBefore,
               "stagingSignature notices an undo, so the window refreshes without extra work");

    /* Why there is no redo stack: the tree did not change when the plan was taken back, so
       the same button pressed again reproduces it. */
    StagingPlan again = autoStage(bs);
    bool same = again.applied && again.changes.size() == plan.changes.size();
    for(size_t i = 0; same && i < plan.changes.size(); i++) {
        same = again.changes[i].id == plan.changes[i].id
            && again.changes[i].from == plan.changes[i].from
            && again.changes[i].to == plan.changes[i].to;
    }
    CHECK_TRUE(same, "pressing Auto-stage again IS the redo");
}

static void test_undo_reaches_a_part_by_id_not_by_the_index_it_had() {
    Catalog cat;
    BuildShip bs;
    const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 4);
    addPart(bs, cat.decoupler(5), "d_core", pod, 1);
    /* A leaf sitting in the middle of the array, so deleting it shifts every later part up
       by one -- exactly what a delete does to a change list that remembered indices. */
    addPart(bs, cat.chute(10, 500), "chute_1", pod, 4);
    addPart(bs, cat.tank(50, 25, 25), "up_tank", 1, 2);
    addPart(bs, cat.decoupler(5), "d_boost", 3, 3);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "bo_eng", 4, 2);

    StagingPlan plan = autoStage(bs);
    /* The authored labels are deliberately all different (4,1,4,2,3,2), so a change that
       lands on the wrong part shows up in the label row at the end. */
    CHECK_TRUE(plan.applied && plan.changes.size() == 4,
               "the tree moves four of the six labels here");
    StagingUndo stack;
    recordStagingEdit(stack, plan.changes, false);

    eraseLeaf(bs, 2);
    CHECK_TRUE(bs.parts.size() == 5, "the mid-array leaf is gone, shifting every part below it up by one");
    CHECK_TRUE(bs.parts[2].id == "up_tank" && bs.parts[4].id == "bo_eng",
               "which is exactly what a delete does to a change list that remembered indices");

    std::vector<std::string> notes;
    const size_t back = undoStagingEdit(bs, stack, notes);
    CHECK_TRUE(back == 3, "three of the four come back; the fourth part is gone");
    /* {4,1,2,3,2}: pod, d_core, up_tank, d_boost, bo_eng. Distinct values on purpose -- if
       the undo aimed by index it would land d_boost's or bo_eng's label on the wrong hull
       member and this row would read differently. */
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({4, 1, 2, 3, 2}),
               "each survivor keeps the label the authored build gave it");
    CHECK_TRUE(has_line(notes, "chute_1") && has_line(notes, "no longer in the build"),
               "and the player is told which part the undo could not reach");
}

static void test_undo_leaves_a_label_that_moved_since_alone() {
    Catalog cat;
    BuildShip bs;
    const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 1);
    addPart(bs, cat.decoupler(5), "d_core", pod, 1);
    addPart(bs, cat.tank(50, 25, 25), "up_tank", 1, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "up_eng", 2, 1);

    StagingPlan plan = autoStage(bs);
    CHECK_TRUE(plan.applied && !plan.changes.empty(), "something to take back");
    StagingUndo stack;
    recordStagingEdit(stack, plan.changes, false);

    /* Something newer than the entry touched this part (a hand edit the stack never saw, or
       a part re-added under the same id). The entry describes a build that no longer
       exists, so it gets a say about the parts it still recognises and none about this. */
    bs.parts[2].stage = 7;

    std::vector<std::string> notes;
    const size_t back = undoStagingEdit(bs, stack, notes);
    CHECK_TRUE(bs.parts[2].stage == 7, "the newer value is left alone, not overwritten");
    CHECK_TRUE(has_line(notes, "'up_tank' now carries label 7"), "and the disagreement is named");
    CHECK_TRUE(back == plan.changes.size() - 1, "the rest of the edit still takes effect");
}

static void test_a_run_of_same_part_retyping_is_one_undo() {
    StagingUndo stack;
    recordStagingEdit(stack, {}, true);
    CHECK_TRUE(stack.empty(), "a box edit with nothing in it is not a step");
    recordStagingEdit(stack, {{0, "pod", 3, 3}}, true);
    CHECK_TRUE(stack.empty(), "nor is one whose two labels are equal -- an entry that cannot "
                              "move anything is an Undo button that lies");

    /* The number box fires once per frame while the arrow is held, so one decision arrives
       as several edits. They have to be one step: the player's question afterwards is "what
       did this part say before I started", not "how many frames did my finger stay down". */
    recordStagingEdit(stack, {{0, "pod", 1, 2}}, true);
    recordStagingEdit(stack, {{0, "pod", 2, 3}}, true);
    recordStagingEdit(stack, {{0, "pod", 3, 4}}, true);
    CHECK_TRUE(stack.size() == 1, "a held arrow is one step");
    CHECK_TRUE(stack[0].changes[0].from == 1 && stack[0].changes[0].to == 4, "and it remembers where it started");

    recordStagingEdit(stack, {{1, "tank_1", 1, 2}}, true);
    CHECK_TRUE(stack.size() == 2, "a different part is a different step");
    recordStagingEdit(stack, {{1, "tank_1", 2, 1}}, true);
    CHECK_TRUE(stack.size() == 1, "typing it back to where it started is not a step either");

    Catalog cat;
    BuildShip bs;
    addPart(bs, cat.mono(100, 0), "pod", -1, 4);
    addPart(bs, cat.tank(50, 25, 25), "tank_1", 0, 1);
    std::vector<std::string> notes;
    CHECK_TRUE(undoStagingEdit(bs, stack, notes) == 1, "one undo puts the one part back");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 1}), "back to the authored floor");
    CHECK_TRUE(stack.empty(), "and the stack is empty again");
}

/* Two retypes on DIFFERENT parts whose values happen to chain (A 1->2, then B 2->3) are two
   steps. The merge rule reads the part id, and nothing here would notice if it stopped: the
   second edit would be folded into the first, and Ctrl+Z would move A while B's edit
   vanished -- the shape that makes this its own test. */
static void test_two_retys_on_different_parts_are_two_steps() {
    StagingUndo stack;
    recordStagingEdit(stack, {{0, "pod", 1, 2}}, true);
    recordStagingEdit(stack, {{1, "tank_1", 2, 3}}, true);
    // Read the ids through a bounds check: if the merge stops looking at the id it eats this
    // entry, and indexing stack[1] off the end crashes the binary before it can print which
    // decision was swallowed.
    auto idAt = [&](size_t i) -> std::string {
        return i < stack.size() && !stack[i].changes.empty() ? stack[i].changes[0].id
                                                             : std::string();
    };
    CHECK_TRUE(stack.size() == 2, "the values chain but the parts do not: two steps");
    CHECK(idAt(0) == "pod", "step 1 names the pod");
    CHECK(idAt(1) == "tank_1", "step 2 names the tank");

    Catalog cat;
    BuildShip bs;
    addPart(bs, cat.mono(100, 0), "pod", -1, 2);
    addPart(bs, cat.tank(50, 25, 25), "tank_1", 0, 3);
    std::vector<std::string> notes;
    CHECK_TRUE(undoStagingEdit(bs, stack, notes) == 1, "one undo, one part");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({2, 2}), "the later retype goes back, the earlier stays");
    CHECK_TRUE(undoStagingEdit(bs, stack, notes) == 1, "and the earlier one is still there to undo");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 2}), "in order");
}

/* A plan that happens to move ONE part is still a bulk edit. Keying the merge on how many
   parts moved -- rather than on who made the edit -- folds it into the hand edit above it,
   and one Ctrl+Z then rewinds two decisions: the toast promised "Ctrl+Z to take it back"
   and what comes back is the plan AND the player's own retype, with nothing left under the
   button. */
static void test_a_one_part_plan_is_its_own_undo_step() {
    Catalog cat;
    BuildShip bs;
    /* Already numbered except for the core cut: the plan has exactly one part to move. */
    const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 1);
    const int dCore = addPart(bs, cat.decoupler(5), "d_core", pod, 1);
    const int upTank = addPart(bs, cat.tank(50, 25, 25), "up_tank", dCore, 2);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "up_eng", upTank, 2);

    StagingPlan plan = autoStage(bs);
    CHECK_TRUE(plan.applied && plan.changes.size() == 1, "a plan with one part in it");
    CHECK_TRUE(bs.parts[dCore].stage == 2, "which moved d_core onto the cut");

    StagingUndo stack;
    recordStagingEdit(stack, plan.changes, false);
    CHECK_TRUE(stack.size() == 1, "the plan is a step");

    /* The player then nudges that same part by hand. */
    bs.parts[dCore].stage = 3;
    recordStagingEdit(stack, {{dCore, "d_core", 2, 3}}, true);
    CHECK_TRUE(stack.size() == 2, "and does not merge into the plan below it");

    std::vector<std::string> notes;
    CHECK_TRUE(undoStagingEdit(bs, stack, notes) == 1, "one undo");
    CHECK_TRUE(bs.parts[dCore].stage == 2, "takes back the hand nudge, leaving the plan's label");
    CHECK_TRUE(undoStagingEdit(bs, stack, notes) == 1, "and the plan is still there to take back");
    CHECK_TRUE(bs.parts[dCore].stage == 1, "back to what was authored");
    CHECK_TRUE(stack.empty(), "two decisions, two undos");
}

static void test_undo_with_nothing_recorded_changes_nothing() {
    Catalog cat;
    BuildShip bs;
    addPart(bs, cat.mono(100, 0), "pod", -1, 1);
    addPart(bs, cat.decoupler(5), "d_core", 0, 1);
    const std::vector<int> before = labelsOf(bs);

    StagingUndo stack;
    std::vector<std::string> notes;
    CHECK_TRUE(undoStagingEdit(bs, stack, notes) == 0, "an empty stack undoes nothing");
    CHECK_TRUE(labelsOf(bs) == before, "and leaves the build alone");
    CHECK_TRUE(notes.empty(), "saying so is the editor's job (vabUndoStaging), not this one's");
}

/* The Staging ladder's step buttons (#1022), tested as data rather than as widgets. The
   window's job is to put the right button next to the right row; what these two own is what
   a click does to the build, and the part a player cannot see going wrong: a rung shift that
   took only the parts the ladder SHOWS would strand a tank or a capsule on the label its
   neighbours left, and the ladder draws no row for those parts to notice it in. */
static void test_a_rung_step_moves_every_part_carrying_the_label() {
    Catalog cat;
    BuildShip bs;
    const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 1);        // answers nothing
    addPart(bs, cat.tank(50, 25, 25), "tank_1", pod, 1);                // answers nothing
    addPart(bs, cat.decoupler(5), "d_core", pod, 1);                    // answers the press
    addPart(bs, cat.engine(10, 0.5, 1000.0), "up_eng", 2, 3);           // a different rung
    addPart(bs, cat.decoupler(5), "d_boost", 3, 3);

    StagingUndo stack;
    const size_t moved = rungStepStage(bs, stack, 1, +1);
    CHECK_TRUE(moved == 3, "three parts carry rung 1");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({2, 2, 2, 3, 3}),
               "the whole rung moves, the parts that answer nothing included, and the rung "
               "above is untouched");
    CHECK_TRUE(stack.size() == 1, "the rung is ONE undo step, not one per part");
    CHECK_TRUE(stack[0].changes.size() == 3, "and its entry names all three");
    CHECK_TRUE(!stack[0].typed,
               "a rung shift is a bulk edit: it must not chain onto a part's own steps");

    std::vector<std::string> notes;
    CHECK_TRUE(undoStagingEdit(bs, stack, notes) == 3, "one Ctrl+Z takes the whole rung back");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 1, 1, 3, 3}), "back where they stood");

    /* A rung that no part carries is not a rung the ladder drew, so this is the caller's
       mistake rather than the player's -- but it must still be a no-op, not an empty entry
       that makes the Undo button answer "done" while the build sits still. */
    CHECK_TRUE(rungStepStage(bs, stack, 7, +1) == 0, "a label nothing carries moves nothing");
    CHECK_TRUE(stack.empty(), "and records nothing");
}

static void test_the_ladder_steps_clamp_at_the_ends_of_the_save_range() {
    Catalog cat;
    BuildShip bs;
    const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 1);
    addPart(bs, cat.decoupler(5), "d_core", pod, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "up_eng", 1, kMaxStage);

    StagingUndo stack;
    /* The floor: a fresh build is ONE rung at label 1, so the ladder's first rung has
       nowhere to go DOWN -- and holding a button at the end must not fill the stack. */
    CHECK_TRUE(rungStepStage(bs, stack, 1, -1) == 0, "the floor rung cannot step down");
    CHECK_TRUE(partStepStage(bs, stack, pod, -1) == 0, "nor can a part on it");
    CHECK_TRUE(stack.empty(), "a step that cannot move is not an undo step");
    /* The ceiling, and it is kMaxStage rather than the rungs the build carries: separating a
       stack by hand has to work when autoStage refuses to read it, so a part has to be able
       to travel past the labels that exist today. */
    CHECK_TRUE(rungStepStage(bs, stack, kMaxStage, +1) == 0, "the top rung cannot step up");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 1, kMaxStage}), "the build is untouched");
    CHECK_TRUE(rungStepStage(bs, stack, kMaxStage, -1) == 1, "but it can come back down");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 1, kMaxStage - 1}), "one label at a time");
}

static void test_a_part_step_is_one_typed_decision() {
    Catalog cat;
    BuildShip bs;
    const int pod = addPart(bs, cat.mono(100, 0), "pod", -1, 1);
    addPart(bs, cat.decoupler(5), "d_core", pod, 1);
    const int eng = addPart(bs, cat.engine(10, 0.5, 1000.0), "up_eng", 1, 1);

    StagingUndo stack;
    /* Holding the button repeats the call on the same part, and "where was this engine
       before I started nudging it" has one answer. */
    for(int i = 0; i < 3; i++) {
        CHECK_TRUE(partStepStage(bs, stack, eng, +1) == 1, "one label per click");
    }
    CHECK_TRUE(stack.size() == 1, "three clicks on one part are one undo step");
    CHECK_TRUE(stack[0].typed, "and it is the typed kind, so it chains, not accumulates");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 1, 4}), "and the part is three labels up");

    /* Out-of-range index: the window builds these from its own cached groups, so a bad index
       means the cache and the build disagree (#527's tripwire) -- but the step must not be
       the thing that reads past the vector. */
    CHECK_TRUE(partStepStage(bs, stack, -1, +1) == 0, "no part before the build");
    CHECK_TRUE(partStepStage(bs, stack, (int)bs.parts.size(), +1) == 0, "and none past its end");
    CHECK_TRUE(stack.size() == 1, "neither records an edit");

    /* A rung step lands on top of a typed entry and must NOT merge into it: the player who
       nudged one engine and then moved a whole rung wants two undoes, not one confused one. */
    CHECK_TRUE(rungStepStage(bs, stack, 1, +1) == 2, "the parts still on rung 1 move together");
    CHECK_TRUE(stack.size() == 2, "as their own step");
    std::vector<std::string> notes;
    CHECK_TRUE(undoStagingEdit(bs, stack, notes) == 2, "one undo takes the rung back");
    CHECK_TRUE(labelsOf(bs) == std::vector<int>({1, 1, 4}),
               "and leaves the nudged engine where the player left it");
}

int main() {
    test_two_stage();
    test_single_stage();
    test_asparagus();
    test_barrier_no_link();
    test_exhaust_scale();
    test_inert_resources();
    test_h2_only_engine();
    test_inert_drop();
    test_zero_g();
    test_signature();
    test_stage_labels_are_labels();
    test_a_gap_costs_nothing();
    test_a_label_dense_ship_is_still_cheap_to_number();
    test_deep_chain_burn_stays_physical();
    test_drop_walk_takes_only_the_decouplers_side();
    test_a_self_parent_walks_once();
    test_a_sealed_drop_does_not_hold_the_burn_open();
    test_a_decoupler_on_the_last_label_does_not_end_the_accounting();
    test_reachability_survives_a_renumbering();
    test_a_linked_tank_in_a_drop_holds_the_burn_open();
    test_an_unlit_engine_does_not_open_a_burn();
    test_a_newly_lit_engine_outside_the_drop_reaches_inside_it();
    test_a_live_decoupler_above_a_dead_tank_does_not_adopt_its_fuel();
    test_a_separation_does_not_relist_an_already_dead_subtree();
    test_a_three_hop_string_drains_one_layer_per_period();
    test_a_small_ship_planned_after_a_big_one_is_only_itself();
    test_a_duplicate_id_resolves_to_the_last_part();
    test_seed_the_fuel_and_mass_a_live_ship_carries();
    test_a_stage_label_means_exactly_three_things();
    test_a_stage_label_is_order_not_height();
    test_the_staging_checker_names_the_presses_that_do_nothing();
    test_the_label_groups_read_as_press_order();
    test_the_press_note_says_where_the_counter_stands();
    test_auto_stager_numbers_the_tree_from_its_decouplers();
    test_auto_stager_gives_a_ring_one_press();
    test_auto_stager_refuses_a_build_with_no_decoupler();
    test_auto_stager_refuses_a_cut_that_would_drop_everything();
    test_auto_stager_keeps_two_engines_relative_to_each_other();
    test_auto_stager_keeps_a_launch_lit_engine_at_the_pad();
    test_auto_stager_keeps_an_authored_asparagus_sequence();
    test_auto_stager_orders_cuts_by_the_authored_label_not_depth();
    test_auto_stager_never_puts_a_cut_above_the_cut_that_contains_it();
    test_auto_stager_clamps_an_outer_cut_against_its_neighbours();
    test_auto_stager_orders_cuts_by_the_fuel_chain_when_asked();
    test_auto_stager_fuel_link_switch_is_inert_without_links();
    test_auto_stager_ranks_only_the_boosters_the_links_cover();
    test_auto_stager_ignores_a_chain_that_hands_fuel_no_further_inboard();
    test_auto_stager_keeps_typed_cut_numbers_ahead_of_the_fuel_chain();
    test_auto_stager_from_scratch_drops_numbers_it_cannot_honor();
    test_auto_stager_from_scratch_replaces_a_hand_numbered_ladder();
    test_auto_stager_from_scratch_declines_the_pad_lit_pin();
    test_auto_stager_clamps_a_fuel_rank_above_a_cut_it_contains();
    test_auto_stager_gives_up_the_pad_pin_rather_than_empty_the_floor();
    test_auto_stager_says_what_a_chute_on_the_floor_needs();
    test_auto_stager_reads_the_graph_not_the_part_order();
    test_auto_stager_refuses_a_parent_chain_that_loops();
    test_auto_stager_refuses_a_forest_whose_every_root_is_a_cut();
    test_auto_stager_refuses_more_cuts_than_labels_exist();
    test_undo_takes_an_auto_stage_back_to_the_authored_labels();
    test_undo_reaches_a_part_by_id_not_by_the_index_it_had();
    test_undo_leaves_a_label_that_moved_since_alone();
    test_a_run_of_same_part_retyping_is_one_undo();
    test_two_retys_on_different_parts_are_two_steps();
    test_a_one_part_plan_is_its_own_undo_step();
    test_undo_with_nothing_recorded_changes_nothing();
    test_a_rung_step_moves_every_part_carrying_the_label();
    test_the_ladder_steps_clamp_at_the_ends_of_the_save_range();
    test_a_part_step_is_one_typed_decision();

    printf("test_staging_dv: %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
