// test_staging_dv: VAB staging analysis (src/staging.cpp) -- per-stage
// vacuum delta-v and TWR, including fuel-link asparagus.
//
// Pins:
//   * a plain two-stage rocket: Tsiolkovsky delta-v per stage, min TWR at
//     the start of the burn (heaviest) and max at the end (lightest),
//     against the reference g;
//   * dry mass = wet - propellant (PartDef::mass INCLUDES full tanks);
//   * asparagus (outer -> inner fuel links): the outer booster's tanks
//     empty first (shared with the core engine), the stage drops when
//     THEY are empty, and the core keeps burning afterwards;
//   * decouplers are fuel barriers (the core engine cannot drain a
//     booster's tanks without an explicit link).
//
// Pure math over BuildShip -- no GL, no Bullet.
//
// Runs from the repo root:
//   make test   (or: ./test_staging_dv)

#include "staging.h"

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
        if (!std::isfinite(_a) || std::fabs(_a - _e) > rel_tol * std::fabs(_e)) { \
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
        d.mass = dry + h2 + lox;   // WET: catalog mass includes full tanks
        d.capacity[(int)ResourceType::Hydrogen] = (float)h2;
        d.capacity[(int)ResourceType::LOX] = (float)lox;
        return &d;
    }
    const PartDef *engine(double dry, double fuelRate, double ve) {
        defs.push_back(PartDef());
        PartDef &d = defs.back();
        d.mass = dry;
        d.fuel_rate = fuelRate;
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
    const PartDef *mono(double wet, double hydrazine) {
        defs.push_back(PartDef());
        PartDef &d = defs.back();
        d.mass = wet;
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

// rate = 0.5 -> mdot = 1 kg/s, T = 2*0.5*1000 = 1000 N.
static void test_two_stage() {
    Catalog cat;
    BuildShip bs;
    // Upper: dry 50 + fuel 50 = wet 100. Engine dry 10.
    // Lower: dry 50 + fuel 150 = wet 200. Engine dry 10. Decoupler 5.
    const int upTank = addPart(bs, cat.tank(50, 25, 25), "upTank", -1, 2);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "upEng", upTank, 2);
    const int dec = addPart(bs, cat.decoupler(5), "dec", upTank, 1);
    const int loTank = addPart(bs, cat.tank(50, 75, 75), "loTank", dec, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "loEng", loTank, 1);

    // Wet: 100+10+5+200+10 = 325. Dry: 50+10+5+50+10 = 125.
    CHECK_NEAR(partDryMass(*bs.parts[0].def), 50.0, 1e-9, "upper dry");
    CHECK_NEAR(bs.parts[0].def->mass, 100.0, 1e-9, "upper wet");

    const double g = 10.0;
    const std::vector<StageRow> rows = computeStaging(bs, g);
    CHECK_TRUE(rows.size() == 2, "two rows");
    if(rows.size() != 2) { return; }

    // Stage 1 (lower): burns 150 kg of the 200-kg tank while the upper
    // is still attached. m0 = 325, m1 = 325 - 150 = 175 (the empty lower
    // tank dry + engines + upper still hang on until the drop).
    // dv = 1000 * ln(325/175).
    CHECK_TRUE(rows[0].stage == 1, "row0 is stage 1");
    CHECK_TRUE(rows[0].drops, "stage 1 drops");
    CHECK_NEAR(rows[0].massStart, 325.0, 1e-6, "s1 m0");
    CHECK_NEAR(rows[0].massEnd, 175.0, 1e-6, "s1 m1 (before drop)");
    CHECK_NEAR(rows[0].deltaV, 1000.0 * std::log(325.0 / 175.0), 1e-4, "s1 dv");

    // Stage 2 (upper): after dropping dec+lower (5+50+10 = 65 kg dry),
    // m0 = 175 - 65 = 110, burns 50 kg -> m1 = 60.
    // dv = 1000 * ln(110/60).
    CHECK_TRUE(rows[1].stage == 2, "row1 is stage 2");
    CHECK_TRUE(!rows[1].drops, "stage 2 does not drop");
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
    // wet 200 + engine 20 = 220 as before, plus a mono tank whose 78 kg
    // of hydrazine stays on the stack the whole burn.
    const int tank = addPart(bs, cat.tank(100, 50, 50), "tank", -1, 1);
    addPart(bs, cat.engine(20, 0.5, 2000.0), "eng", tank, 1);
    addPart(bs, cat.mono(88.99, 78.54), "mono", tank, 1);

    // wet = 220 + 88.99 = 308.99, burnable = 100, inert end = 208.99.
    const std::vector<StageRow> rows = computeStaging(bs, 9.8);
    CHECK_TRUE(rows.size() == 1, "inert: one row");
    if(rows.size() != 1) { return; }
    CHECK_NEAR(rows[0].massStart, 308.99, 1e-3, "inert m0");
    CHECK_NEAR(rows[0].massEnd, 208.99, 1e-3, "inert m1 (mono kept)");
    CHECK_NEAR(rows[0].deltaV, 2000.0 * std::log(308.99 / 208.99), 1e-3, "inert dv");
}

// Payload separator: a decoupler subtree with NO propellant. Without the
// drainAll fallback the period would end at zero length and swallow the
// upper stage's burn.
static void test_inert_drop() {
    Catalog cat;
    BuildShip bs;
    // Upper: tank + engine (stage 2). Then an inert sep (stage 2) with a
    // dummy block below it. Lower booster on stage 1 with its own engine.
    const int upTank = addPart(bs, cat.tank(50, 25, 25), "upTank", -1, 3);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "upEng", upTank, 2);
    const int sep = addPart(bs, cat.decoupler(5), "sep", upTank, 2);
    addPart(bs, cat.tank(20, 0, 0), "dummy", sep, 2);
    const int dec = addPart(bs, cat.decoupler(5), "dec", upTank, 1);
    const int loTank = addPart(bs, cat.tank(50, 75, 75), "loTank", dec, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "loEng", loTank, 1);

    // wet = 100+10+5+20+5+200+10 = 350. Stage 1 lights only the lower
    // engine (upper is stage 2): burns the lower's 150 kg, m 350 -> 200.
    // Drop 65 -> 135. Stage 2 lights the upper, drops the inert sep, and
    // must still burn the upper's 50 kg: 135 -> 85.
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
// Both engines light at stage 1; the outer drop is stage 1. The outer
// tanks empty FIRST (they sit in the core engine's furthest drain layer
// and also feed the outer engine), then the booster drops, then the core
// keeps burning its own tank (stage 2 / final).
//
//   coreTank (stage 2) -- coreEng (stage 2)
//   dec (stage 1) -- outerTank (stage 1) -- outerEng (stage 1)
//   link: outerTank -> coreTank
//
// Engines both fire at launch (stage 1), including the core (stage 2
// lights when the counter reaches 2 -- so for this test put BOTH engines
// on stage 1 so they burn together through the asparagus phase).
static void test_asparagus() {
    Catalog cat;
    BuildShip bs;
    const int coreTank = addPart(bs, cat.tank(50, 50, 50), "coreTank", -1, 2);
    // Core engine on stage 1: lights at launch beside the booster.
    addPart(bs, cat.engine(10, 0.5, 1000.0), "coreEng", coreTank, 1);
    const int dec = addPart(bs, cat.decoupler(5), "dec", coreTank, 1);
    const int outTank = addPart(bs, cat.tank(50, 50, 50), "outTank", dec, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "outEng", outTank, 1);
    addLink(bs, "outTank", "coreTank");   // outer feeds core (asparagus)

    // wet: coreTank 150 + coreEng 10 + dec 5 + outTank 150 + outEng 10 = 325
    // Both engines: T = 2000 N, mdot = 2 kg/s together.
    // Drain layers of the core engine: [out], [core] -- outer first.
    // Drain layers of the outer engine: [out] only.
    // Combined they empty OUTER first at 2 kg/s. Outer has 100 kg fuel.
    // Stage 1 stops when outer is empty of drainable fuel: 100 kg burned
    // in 50 s. Mass falls 325 -> 225 (both dry cores remain).
    // Drop outer: 50 dry + 10 eng + 5 dec = 65. Remaining = 160.
    // Stage 2 (final): core engine burns the core 100 kg. 160 -> 60.
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
    const int coreTank = addPart(bs, cat.tank(50, 75, 75), "coreTank", -1, 2);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "coreEng", coreTank, 1);
    const int dec = addPart(bs, cat.decoupler(5), "dec", coreTank, 1);
    const int outTank = addPart(bs, cat.tank(50, 50, 50), "outTank", dec, 1);
    addPart(bs, cat.engine(10, 0.5, 1000.0), "outEng", outTank, 1);

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

int main() {
    test_two_stage();
    test_single_stage();
    test_asparagus();
    test_barrier_no_link();
    test_inert_resources();
    test_inert_drop();
    test_zero_g();

    printf("test_staging_dv: %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
