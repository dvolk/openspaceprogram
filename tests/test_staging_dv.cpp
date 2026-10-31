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
//   * decouplers are fuel barriers (the core engine cannot drain a
//     booster's tanks without an explicit link).
//   * stage numbers are LABELS: the walk visits the labels the ship carries, highest
//     first, and a gap between two labels costs nothing (#517).
//
// Pure math over BuildShip -- no GL, no Bullet.
//
// Runs from the repo root:
//   make test   (or: ./test_staging_dv)

#include "staging.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

static int g_failures = 0;
static int g_checks = 0;

#define CHECK_TRUE(cond, msg)                                                  \
    do {                                                                       \
        g_checks++;                                                            \
        if (!(cond)) {                                                         \
            g_failures++;                                                      \
            printf("FAIL: %s\n", msg);                                         \
        }                                                                      \
    } while (0)

#define CHECK_NEAR(actual, expected, rel_tol, msg)                             \
    do {                                                                       \
        g_checks++;                                                            \
        double _a = (actual), _e = (expected);                                 \
        /* Relative, except at an expectation of exactly zero (#510):              \
           rel_tol * 0.0 would demand bit-exactness, which is not what a           \
           caller writing 1e-12 means. 0.0 AS a tolerance stays exact, so          \
           a bit-identical pin keeps its teeth. */                                 \
        const double _tol = (_e == 0.0) ? rel_tol                                  \
                                        : rel_tol * std::fabs(_e);                 \
        if (!std::isfinite(_a) || std::fabs(_a - _e) > _tol) {                    \
            g_failures++;                                                      \
            printf("FAIL: %s (got %.9g, want %.9g)\n", msg, _a, _e);           \
        }                                                                      \
    } while (0)

// PartDef lives in a deque so pointers stay valid while the build grows.
// Tanks carry mass + capacity; engines carry thrust (and may hold no
// propellant of their own -- they drain their fuel group).
struct Catalog {
    std::deque<PartDef> defs;

    const PartDef *tank(double dry, double h2, double lox) {
        defs.push_back(PartDef());
        PartDef &d = defs.back();
        d.mass = dry;   // DRY: catalog mass is structure only; the fuel rides capacity
        d.capacity[(int)ResourceType::Hydrogen] = (float)h2;
        d.capacity[(int)ResourceType::LOX] = (float)lox;
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
        d.capacity[(int)ResourceType::Hydrazine] = (float)hydrazine;
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
    CHECK_NEAR(partDryMass(*bs.parts[0].def, burn), 50.0, 1e-9, "upper dry");
    CHECK_NEAR(bs.parts[0].def->mass, 50.0, 1e-9, "upper catalog mass is dry structure");

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
    CHECK_NEAR(rows[0].massStart, 325.0, 1e-6, "s1 m0");
    CHECK_NEAR(rows[0].massEnd, 175.0, 1e-6, "s1 m1 (before drop)");
    CHECK_NEAR(rows[0].deltaV, 1000.0 * std::log(325.0 / 175.0), 1e-4, "s1 dv");

    // Stage 1 (upper): after dropping dec+lower (5+50+10 = 65 kg dry),
    // m0 = 175 - 65 = 110, burns 50 kg -> m1 = 60.
    // dv = 1000 * ln(110/60).
    CHECK_TRUE(rows[1].stage == 1, "row1 is stage 1");
    CHECK_TRUE(!rows[1].drops, "stage 1 does not drop");
    CHECK_NEAR(rows[1].massStart, 110.0, 1e-6, "s2 m0");
    CHECK_NEAR(rows[1].massEnd, 60.0, 1e-6, "s2 m1");
    CHECK_NEAR(rows[1].deltaV, 1000.0 * std::log(110.0 / 60.0), 1e-4, "s2 dv");

    // TWR against g = 10. Stage 1: T = 1000 N (one engine).
    // min at start: 1000 / (325 * 10); max at end: 1000 / (175 * 10).
    CHECK_NEAR(rows[0].minTWR, 1000.0 / (325.0 * g), 1e-6, "s1 minTWR");
    CHECK_NEAR(rows[0].maxTWR, 1000.0 / (175.0 * g), 1e-6, "s1 maxTWR");
    CHECK_TRUE(rows[0].minTWR < rows[0].maxTWR, "s1 min < max");
    CHECK_NEAR(rows[1].minTWR, 1000.0 / (110.0 * g), 1e-6, "s2 minTWR");
    CHECK_NEAR(rows[1].maxTWR, 1000.0 / (60.0 * g), 1e-6, "s2 maxTWR");
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
    CHECK_NEAR(rows[0].deltaV, 2000.0 * std::log(220.0 / 120.0), 1e-4, "dv");
    CHECK_NEAR(rows[0].massStart, 220.0, 1e-6, "m0");
    CHECK_NEAR(rows[0].massEnd, 120.0, 1e-6, "m1");
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
    CHECK_NEAR(rows[0].massStart, 308.99, 1e-3, "inert m0");
    CHECK_NEAR(rows[0].massEnd, 208.99, 1e-3, "inert m1 (mono kept)");
    CHECK_NEAR(rows[0].deltaV, 2000.0 * std::log(308.99 / 208.99), 1e-3, "inert dv");
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
    CHECK_NEAR(partPropellantMass(*bs.parts[tank].def, burn), 50.0, 1e-9,
               "h2-only: tank burnable = H2 only (the LOX is inert)");
    CHECK_NEAR(partDryMass(*bs.parts[tank].def, burn), 150.0, 1e-9,
               "h2-only: tank dry = structure + the inert LOX");

    // Wet 220, burnable 50 (H2), so the burn ends at 170 (the LOX rides
    // along). dv = ve * ln(220/170) -- LESS than a chemical engine would get
    // from the same tank (it could burn the LOX too), which is the point.
    const std::vector<StageRow> rows = computeStaging(bs, 10.0);
    CHECK_TRUE(rows.size() == 1, "h2-only: one row");
    if(rows.size() != 1) { return; }
    CHECK_NEAR(rows[0].massStart, 220.0, 1e-6, "h2-only m0");
    CHECK_NEAR(rows[0].massEnd, 170.0, 1e-6, "h2-only m1 (the LOX is kept)");
    CHECK_NEAR(rows[0].deltaV, 2000.0 * std::log(220.0 / 170.0), 1e-3, "h2-only dv");
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
    CHECK_NEAR(rows[0].deltaV, 1000.0 * std::log(350.0 / 200.0), 1e-3, "id s1 dv");
    // The upper burn must still happen even though the sep holds no
    // propellant (the drainAll fallback).
    CHECK_TRUE(rows[1].deltaV > 100.0, "inert-drop: upper burn survives");
    CHECK_NEAR(rows[1].deltaV, 1000.0 * std::log(135.0 / 85.0), 1e-3, "id s2 dv");
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
    CHECK_NEAR(rows[0].deltaV, 2000.0 * std::log(220.0 / 120.0), 1e-4, "zero-g dv");
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
    CHECK_NEAR(rows[0].massStart, 325.0, 1e-6, "asp m0");
    CHECK_NEAR(rows[0].massEnd, 225.0, 1e-6, "asp m1 (outer empty, still attached)");
    // dv while BOTH engines fire and the outer empties:
    // F = 2000, mdot = 2, ve_eq = 1000. dv = 1000 * ln(325/225).
    CHECK_NEAR(rows[0].deltaV, 1000.0 * std::log(325.0 / 225.0), 1e-3, "asp s1 dv");

    CHECK_TRUE(!rows[1].drops, "final does not drop");
    CHECK_NEAR(rows[1].massStart, 160.0, 1e-6, "asp s2 m0");
    CHECK_NEAR(rows[1].massEnd, 60.0, 1e-6, "asp s2 m1");
    // Only the core engine remains: T = 1000, mdot = 1, ve = 1000.
    CHECK_NEAR(rows[1].deltaV, 1000.0 * std::log(160.0 / 60.0), 1e-3, "asp s2 dv");

    // Stage-1 TWR is the pair (T = 2000) at 325 .. 225 kg on g = 10.
    CHECK_NEAR(rows[0].minTWR, 2000.0 / (325.0 * g), 1e-6, "asp s1 minTWR");
    CHECK_NEAR(rows[0].maxTWR, 2000.0 / (225.0 * g), 1e-6, "asp s1 maxTWR");
    // After the drop only the core engine burns (T = 1000).
    CHECK_NEAR(rows[1].minTWR, 1000.0 / (160.0 * g), 1e-6, "asp s2 minTWR");
    CHECK_NEAR(rows[1].maxTWR, 1000.0 / (60.0 * g), 1e-6, "asp s2 maxTWR");
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
    CHECK_NEAR(rows[0].massStart, 375.0, 1e-6, "barrier s1 m0");
    CHECK_NEAR(rows[0].massEnd, 175.0, 1e-6, "barrier s1 m1");
    // Two engines, mdot = 2, F = 2000, ve_eq = 1000.
    CHECK_NEAR(rows[0].deltaV, 1000.0 * std::log(375.0 / 175.0), 1e-3, "barrier s1 dv");
    // Core kept 50 kg because the barrier stopped the outer from feeding it.
    CHECK_NEAR(rows[1].massStart, 110.0, 1e-6, "barrier s2 m0");
    CHECK_NEAR(rows[1].massEnd, 60.0, 1e-6, "barrier s2 m1");
    CHECK_NEAR(rows[1].deltaV, 1000.0 * std::log(110.0 / 60.0), 1e-3, "barrier s2 dv");
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
    CHECK_NEAR(b[0].massStart, a[0].massStart, 1e-9, "scale m0 unchanged");
    CHECK_NEAR(b[0].massEnd, a[0].massEnd, 1e-9, "scale m1 unchanged");
    // ve_eq and TWR double; delta-v is ve_eq * ln(m0/m1), so it doubles too.
    CHECK_NEAR(b[0].deltaV, 2.0 * a[0].deltaV, 1e-6, "scale dv doubles");
    CHECK_NEAR(b[0].minTWR, 2.0 * a[0].minTWR, 1e-6, "scale minTWR doubles");
    CHECK_NEAR(b[0].maxTWR, 2.0 * a[0].maxTWR, 1e-6, "scale maxTWR doubles");
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
       and because computeStaging resolves a link through an id -> index map,
       that moves the link across the decoupler. Loading one over the other has
       to refresh the table.

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
    CHECK_NEAR(a.deltaV, b.deltaV, 1e-9, "same delta-v");
    CHECK_NEAR(a.massStart, b.massStart, 1e-9, "same mass start");
    CHECK_NEAR(a.massEnd, b.massEnd, 1e-9, "same mass end");
    CHECK_NEAR(a.minTWR, b.minTWR, 1e-9, "same min TWR");
    CHECK_NEAR(a.maxTWR, b.maxTWR, 1e-9, "same max TWR");
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

    /* The ladder matters. At 12 boosters the old bug -- skipping near-empty tanks
       when picking dt -- produces rows BIT-IDENTICAL to the fix (mutation-tested:
       tmp/n526b_a), so the small rungs only pin that the fix does not disturb a
       working burn. It is the deep rungs where the drift becomes a row: at 16 the
       same mutation reads dv -nan, and removing the exact-empty rule or cutting the
       step budget shows up there too. */
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
            CHECK_NEAR(r.deltaV, ve * std::log(r.massStart / r.massEnd), 1e-9,
                       (tag + "delta-v matches ve ln(m0/m1)").c_str());
            spent += r.massStart - r.massEnd;
        }
        CHECK_NEAR(spent, fuel, 1e-9, (tag + "every kg of propellant is spent once").c_str());
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
   deleting the sort changes no row (the 29 ships in res/ships and 20 000 fuzzed ships
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
    CHECK_NEAR(rows[0].massStart, 220.0, 1e-9, "everything is aboard to start");
    CHECK_NEAR(rows[0].massEnd, 160.0, 1e-9, "the branch burned its own 60 kg");
    CHECK_NEAR(rows[0].deltaV, 1000.0 * std::log(220.0 / 160.0), 1e-9,
               "and paid for it");
    CHECK_TRUE(rows[1].stage == 1 && !rows[1].drops, "the final row separates nothing");
    CHECK_NEAR(rows[1].massStart, 110.0, 1e-9, "only the decoupler's side left");
    CHECK_NEAR(rows[1].massEnd, 70.0, 1e-9, "the sibling's tank is still there to drain");
    CHECK_NEAR(rows[1].deltaV, 1000.0 * std::log(110.0 / 70.0), 1e-9, "and it drains");
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
    CHECK_NEAR(rows[0].massStart, 165.0, 1e-9, "the cycle loses nothing");
    CHECK_NEAR(rows[0].massEnd, 165.0, 1e-9, "nothing is lit at that label, so no burn");
    CHECK_NEAR(rows[1].massStart, 160.0, 1e-9, "and the decoupler left");
    CHECK_NEAR(rows[1].massEnd, 60.0, 1e-9, "the tank drains down to its dry mass");
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
    CHECK_NEAR(rows[0].massStart, 425.0, 1e-9, "everything is aboard");
    CHECK_NEAR(rows[0].massEnd, 345.0, 1e-9, "80 kg burnt, the sealed tank still aboard");
    // The separation takes the sealed tank's 80 kg of propellant with it, full.
    CHECK_NEAR(rows[1].massStart, 225.0, 1e-9, "the survivor kept its own 160 kg");
    CHECK_NEAR(rows[1].massEnd, 65.0, 1e-9, "and burns it in the last row");
    CHECK_TRUE(rows[1].deltaV > 0.0, "the survivor still has something to burn");
    CHECK_NEAR(rows[1].deltaV, 1000.0 * std::log(225.0 / 65.0), 1e-9,
               "one engine, the fuel that is left");
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
    CHECK_NEAR(rows[0].massStart, 435.0, 1e-9, "all aboard");
    CHECK_NEAR(rows[0].massEnd, 395.0, 1e-9, "the drop's REACHABLE fuel burns");
    CHECK_NEAR(rows[0].deltaV, 1000.0 * std::log(435.0 / 395.0), 1e-9, "s3 dv");

    CHECK_TRUE(rows[1].stage == 2 && rows[1].drops, "label 2 separates");
    CHECK_NEAR(rows[1].massStart, 360.0, 1e-9, "period 1 left, renumbered");
    CHECK_NEAR(rows[1].massEnd, 320.0, 1e-9, "and its reachable fuel burns, no more");
    CHECK_NEAR(rows[1].deltaV, 1000.0 * std::log(360.0 / 320.0), 1e-9, "s2 dv");

    CHECK_NEAR(rows[2].massStart, 240.0, 1e-9, "the survivor, both drops gone");
    CHECK_NEAR(rows[2].massEnd, 60.0, 1e-9, "burns everything that is left");
    CHECK_NEAR(rows[2].deltaV, 1000.0 * std::log(240.0 / 60.0), 1e-9, "s1 dv");
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
    CHECK_NEAR(rows[0].massStart, 415.0, 1e-9, "all aboard");
    CHECK_NEAR(rows[0].massEnd, 295.0, 1e-9, "both drop tanks run dry, the core's does not");
    CHECK_NEAR(rows[0].deltaV, 1000.0 * std::log(415.0 / 295.0), 1e-9, "s3 dv");
    CHECK_NEAR(rows[1].massStart, 260.0, 1e-9, "the drop leaves 35 kg of structure");
    CHECK_NEAR(rows[1].massEnd, 60.0, 1e-9, "the core burns the rest");
    CHECK_NEAR(rows[1].deltaV, 1000.0 * std::log(260.0 / 60.0), 1e-9, "s1 dv");
}

/* A decoupler an EARLIER separation already took must not start a walk of its own. decB
   carries label 2 and hangs under decA (label 3), so by the time the walk reaches label 2
   decB is dead, and label 2 must separate nothing. A label that separates nothing emits no
   row at all, so the symptom is an EXTRA row: a staging event the ship can no longer
   perform. Two alive checks keep that row away -- droppedAt seeds only live decouplers, and
   computeStaging trims dead entries off the list. Either alone suffices: deleting one leaves
   every check in this file passing, deleting both fails right here, and that redundancy is
   the thing worth a look (#537). */
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
    CHECK_NEAR(rows[0].massStart, 200.0, 1e-9, "all aboard");
    /* The tail is walled off, so this drop is INERT and the row falls back to draining
       everything reachable (computeStaging's inert-drop fallback: drainAll when the drop
       holds nothing drainable) rather than burning nothing. Its own tank's 20 kg stays
       aboard and leaves with the tail. */
    CHECK_NEAR(rows[0].massEnd, 100.0, 1e-9, "the core's 100 kg burns; the tail's does not");
    CHECK_NEAR(rows[0].deltaV, 1000.0 * std::log(200.0 / 100.0), 1e-9, "s3 dv");
    CHECK_TRUE(rows[1].stage == 1, "the next row is the survivor's, not label 2's");
    CHECK_NEAR(rows[1].massStart, 60.0, 1e-9, "the tail left dry, and so did the core");
    CHECK_NEAR(rows[1].massEnd, 60.0, 1e-9, "nothing left to burn");
    CHECK_NEAR(rows[1].deltaV, 0.0, 1e-9, "s1 dv");
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
    CHECK_NEAR(rows[0].massStart, 520.0, 1e-9, "all aboard");
    CHECK_NEAR(rows[0].massEnd, 500.0, 1e-9, "only dropTank's 20 kg burns");
    CHECK_NEAR(rows[0].deltaV, 1000.0 * std::log(520.0 / 500.0), 1e-9, "s3 dv");
    CHECK_NEAR(rows[1].massStart, 375.0, 1e-9, "linkTank leaves with its 100 kg unburnt");
    CHECK_NEAR(rows[1].massEnd, 95.0, 1e-9, "nothing drainable is left");
    CHECK_TRUE(rows[1].engines == 2, "secEng lights for the last row");
    CHECK_NEAR(rows[1].deltaV, 1000.0 * std::log(375.0 / 95.0), 1e-9, "s1 dv");
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
    test_deep_chain_burn_stays_physical();
    test_drop_walk_takes_only_the_decouplers_side();
    test_a_self_parent_walks_once();
    test_a_sealed_drop_does_not_hold_the_burn_open();
    test_reachability_survives_a_renumbering();
    test_a_linked_tank_in_a_drop_holds_the_burn_open();
    test_an_unlit_engine_does_not_open_a_burn();
    test_a_separation_does_not_relist_an_already_dead_subtree();

    printf("test_staging_dv: %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
