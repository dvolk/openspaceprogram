//
// Headless test for Vehicle::droppedPartsAtStage and Vehicle::stagedDecouplers
// (src/vehicle.h).
//
// Staging is a TREE operation: firing a decoupler drops the decoupler plus
// its whole child-side subtree and nothing else -- in particular not a
// sibling branch that happens to share the same stage number. That is the
// heavy_two rule: its central engine shares a stage with the radial boosters
// but hangs off the central tank, so it survives while the boosters go.
//
// droppedPartsAtStage is pure graph logic over Part::parent, Part::stage and
// PartDef::decoupler. It reads no Bullet state, so the ships here are built
// with body = nullptr and nothing is registered in a physics world; the
// destructor is kept off RemoveBody with onRails = true (see destroyShip).
//
// The topology is Part::parent, the part tree. It used to be derived from
// Vehicle::constraintLinks (the weld handles); it is now the authoritative
// source, which is what lets the welds go away -- and what a stage-as-ship
// split will later hand to a new Vehicle.
//
// Runs from the repo root:
//   make test   (or: ./test_staging)

#include <algorithm>
#include <cstdio>
#include <deque>
#include <set>
#include <string>
#include <vector>

#include "vehicle.h"   // Vehicle + Part (inline); pulls in Bullet (btcommon.h)

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

/* A hand-built ship. `defs` is a deque, not a vector: push_back on a vector
   reallocates and would dangle every Part::def handed out so far, while a
   deque never invalidates references to existing elements. */
struct Ship {
    Vehicle *v;
    std::deque<PartDef> defs;
};

/* Add one part under `parent` (nullptr = the root). No Body: nothing here
   touches Bullet. */
static Part *addPart(Ship &s, Part *parent, int stage, bool decoupler,
                     const char *name) {
    s.defs.push_back(PartDef());
    PartDef &d = s.defs.back();
    d.name = name;
    d.decoupler = decoupler;
    d.mass = 100.0f;
    d.radius = 1.0f;
    d.height = 2.0f;

    Part *p = new Part;
    p->body   = nullptr;
    p->def    = &d;
    p->stage  = stage;
    p->parent = parent;
    s.v->parts.push_back(p);
    return p;
}

static void destroyShip(Ship &s) {
    /* onRails keeps ~Vehicle off RemoveBody: these Parts have no
       Body, and there is no physics world here. */
    s.v->onRails = true;
    delete s.v;
}

/* The dropped set as part names, so a failure says WHICH parts were wrong
   rather than just how many. */
static std::set<std::string> droppedNames(Ship &s, int stage) {
    std::set<std::string> out;
    const std::vector<Part *> d = s.v->droppedPartsAtStage(stage);
    for(size_t i = 0; i < d.size(); i++) { out.insert(d[i]->def->name); }
    return out;
}

static std::set<std::string> want(const char *a, const char *b = NULL,
                                  const char *c = NULL, const char *d = NULL,
                                  const char *e = NULL) {
    std::set<std::string> s;
    s.insert(a);
    if(b) { s.insert(b); }
    if(c) { s.insert(c); }
    if(d) { s.insert(d); }
    if(e) { s.insert(e); }
    return s;
}

static void checkDrop(Ship &s, int stage, const std::set<std::string> &expected,
                      const char *msg) {
    const std::set<std::string> got = droppedNames(s, stage);
    g_checks++;
    if(got != expected) {
        g_failures++;
        printf("FAIL: %s\n  want {", msg);
        for(std::set<std::string>::const_iterator it = expected.begin();
            it != expected.end(); ++it) {
            printf("%s%s", it == expected.begin() ? "" : ", ", it->c_str());
        }
        printf("}\n  got  {");
        for(std::set<std::string>::const_iterator it = got.begin();
            it != got.end(); ++it) {
            printf("%s%s", it == got.begin() ? "" : ", ", it->c_str());
        }
        printf("}\n");
    }
}

/* A stage with no decoupler on it drops nothing -- the stage counter
   advances but the ship keeps every part. */
static void test_no_decoupler() {
    Ship s; s.v = new Vehicle;
    Part *root = addPart(s, NULL, 2, false, "capsule");
    Part *mid  = addPart(s, root, 2, false, "tank");
    addPart(s, mid, 2, false, "engine");

    checkDrop(s, 1, std::set<std::string>(), "no decoupler anywhere: empty");
    checkDrop(s, 2, std::set<std::string>(),
              "parts on the stage but no decoupler: empty");
    destroyShip(s);
}

/* The decoupler flies off with its own stage, taking its whole child-side
 * subtree with it -- three levels deep here -- while its parent survives. */
static void test_subtree() {
    Ship s; s.v = new Vehicle;
    Part *root = addPart(s, NULL,  3, false, "capsule");
    Part *dec  = addPart(s, root,  1, true,  "decoupler");
    Part *a    = addPart(s, dec,   1, false, "tank");
    Part *b    = addPart(s, a,     1, false, "engine");
    (void)b;

    checkDrop(s, 1, want("decoupler", "tank", "engine"),
              "decoupler + its whole subtree, parent survives");
    checkDrop(s, 3, std::set<std::string>(), "other stages: empty");
    destroyShip(s);
}

/* THE heavy_two rule: a sibling branch that shares the stage NUMBER is not
   a child of the decoupler, so it stays. Same stage, different parent. */
static void test_sibling_survives() {
    Ship s; s.v = new Vehicle;
    Part *root = addPart(s, NULL, 2, false, "capsule");
    /* branch 1: the booster, under a stage-1 decoupler */
    Part *dec     = addPart(s, root, 1, true,  "decoupler");
    Part *booster = addPart(s, dec,  1, false, "booster");
    /* branch 2: the central engine, also stage 1 but attached to the root */
    Part *central = addPart(s, root, 1, false, "central_engine");
    (void)booster; (void)central;

    checkDrop(s, 1, want("decoupler", "booster"),
              "same-stage sibling branch survives (the heavy_two rule)");
    destroyShip(s);
}

/* Two decouplers on different stages: firing one leaves the other (and its
   subtree) alone. */
static void test_different_stages() {
    Ship s; s.v = new Vehicle;
    Part *root  = addPart(s, NULL,  3, false, "capsule");
    Part *decA  = addPart(s, root,  2, true,  "dec_upper");
    Part *midA  = addPart(s, decA,  2, false, "tank_upper");
    Part *decB  = addPart(s, midA,  1, true,  "dec_lower");
    Part *midB  = addPart(s, decB,  1, false, "tank_lower");
    (void)midB;

    checkDrop(s, 2, want("dec_upper", "tank_upper", "dec_lower", "tank_lower"),
              "outer decoupler swallows the nested inner one");
    checkDrop(s, 1, want("dec_lower", "tank_lower"),
              "inner decoupler drops only its own subtree");
    destroyShip(s);
}

/* Two decouplers on the SAME stage (heavy_two's two radial boosters): the
   drop is the union of both subtrees, and the shared parent survives. */
static void test_two_on_one_stage() {
    Ship s; s.v = new Vehicle;
    Part *root  = addPart(s, NULL,  2, false, "core");
    Part *decL  = addPart(s, root,  1, true,  "dec_left");
    Part *tankL = addPart(s, decL,  1, false, "tank_left");
    Part *decR  = addPart(s, root,  1, true,  "dec_right");
    Part *tankR = addPart(s, decR,  1, false, "tank_right");
    (void)tankL; (void)tankR;

    checkDrop(s, 1, want("dec_left", "tank_left", "dec_right", "tank_right"),
              "two decouplers on one stage: union of both subtrees");
    destroyShip(s);
}

/* Same-depth decouplers, in split order (#540). The order decides which split ship
   keeps the bare name and which becomes "#2", and which one the next-ship handoff
   picks, so it must not come from pointer addresses. The parts are swapped in
   `parts` after being added: allocation order -- which is what the std::set<Part*>
   walk hands back -- stays left-to-right, while build order becomes right-to-left.
   An implementation that falls through to the set's order fails here instead of
   passing by coincidence. */
static void test_staged_decouplers_tie() {
    Ship s; s.v = new Vehicle;
    Part *root  = addPart(s, NULL,  3, false, "core");
    Part *decL  = addPart(s, root,  1, true,  "dec_left");
    Part *tankL = addPart(s, decL,  1, false, "tank_left");
    Part *decR  = addPart(s, root,  1, true,  "dec_right");
    Part *tankR = addPart(s, decR,  1, false, "tank_right");
    (void)tankL; (void)tankR;
    std::swap(s.v->parts[1], s.v->parts[3]);   // dec_right now BUILDS before dec_left
    const std::vector<Part *> got = s.v->stagedDecouplers(1);
    CHECK_TRUE(got.size() == 2, "tie: two decouplers on stage 1");
    CHECK_TRUE(got.size() == 2 && got[0]->def->name == std::string("dec_right")
            && got[1]->def->name == std::string("dec_left"),
               "same-depth decouplers split in build order, not address order");
    /* Ask again with the build order put back. An implementation that answers in the
       set's order gives the SAME first ship both times, whatever the allocator did
       with the two Part addresses -- so this pair of checks cannot be passed by an
       unlucky allocation happening to make the swap above coincide with address
       order, which is the one way the single check above could go vacuous. */
    std::swap(s.v->parts[1], s.v->parts[3]);
    const std::vector<Part *> back = s.v->stagedDecouplers(1);
    CHECK_TRUE(back.size() == 2 && back[0]->def->name == std::string("dec_left")
            && back[1]->def->name == std::string("dec_right"),
               "tie: with build order restored, the left booster splits first");
    CHECK_TRUE(got.size() == 2 && back.size() == 2 && got[0] != back[0],
               "tie: the two build orders must give different first ships");
    destroyShip(s);
}

/* Nested decouplers on the SAME stage: outer first. The inner one is still listed
   (it is a seed of the walk too) and game.cpp's extract then finds it already gone
   with its parent's subtree, which is the only reason the split does not mint two
   ships from one cut. */
static void test_staged_decouplers_nested() {
    Ship s; s.v = new Vehicle;
    Part *core  = addPart(s, NULL,  3, false, "core");
    Part *outer = addPart(s, core,  1, true,  "dec_outer");
    Part *mid   = addPart(s, outer, 3, false, "collar");
    Part *inner = addPart(s, mid,   1, true,  "dec_inner");
    (void)mid; (void)inner;
    const std::vector<Part *> got = s.v->stagedDecouplers(1);
    CHECK_TRUE(got.size() == 2, "nested: both same-stage decouplers are listed");
    CHECK_TRUE(got.size() == 2 && got[0]->def->name == std::string("dec_outer")
            && got[1]->def->name == std::string("dec_inner"),
               "nested same-stage decouplers split outer first");
    destroyShip(s);
}

/* A decoupler of ANOTHER stage that rides along in a dropped subtree is not a split
   root for this press: it stays aboard the ship that just split off and separates on
   its own label. Offering it is what lets one press fire a stage nobody asked for. */
static void test_staged_decouplers_other_stage() {
    Ship s; s.v = new Vehicle;
    Part *core  = addPart(s, NULL, 4, false, "core");
    Part *outer = addPart(s, core, 1, true,  "dec_pressed");
    Part *ride  = addPart(s, outer, 2, true, "dec_other_stage");
    (void)ride;
    const std::vector<Part *> got = s.v->stagedDecouplers(1);
    CHECK_TRUE(got.size() == 1, "a stage-2 decoupler is not a root for a stage-1 press");
    CHECK_TRUE(got.size() == 1 && got[0]->def->name == std::string("dec_pressed"),
               "the press roots its splits at that stage's decouplers only");
    destroyShip(s);
}

/* Depth outranks build order: a deep decoupler built FIRST still splits after a
   shallow one built last, because the outer cut has to happen before the inner one
   is even reachable. */
static void test_staged_decouplers_depth_first() {
    Ship s; s.v = new Vehicle;
    Part *core = addPart(s, NULL, 4, false, "core");
    Part *mid  = addPart(s, core, 4, false, "collar");
    Part *up   = addPart(s, mid,  4, false, "upper");
    Part *deep = addPart(s, up,   1, true,  "dec_deep");     // depth 3, built first
    Part *low  = addPart(s, core, 1, true,  "dec_shallow");  // depth 1, built last
    (void)deep; (void)low;
    const std::vector<Part *> got = s.v->stagedDecouplers(1);
    CHECK_TRUE(got.size() == 2, "depth: two decouplers on stage 1");
    CHECK_TRUE(got.size() == 2 && got[0]->def->name == std::string("dec_shallow")
            && got[1]->def->name == std::string("dec_deep"),
               "a shallower decoupler splits first whatever its build order");
    destroyShip(s);
}

/* Why a split would be refused (#541), which is what lets the staging press name the
   decoupler that cannot separate instead of printing "nothing left to separate". */
static void test_split_block_reason() {
    Ship s; s.v = new Vehicle;
    Part *core = addPart(s, NULL, 2, false, "core");
    Part *dec  = addPart(s, core, 1, true,  "decoupler");
    Part *kid  = addPart(s, dec,  1, false, "tank");
    (void)kid;
    CHECK_TRUE(s.v->splitBlockReason(dec) == Vehicle::SplitBlock::None,
               "a decoupler with a parent and a real subtree is splittable");
    CHECK_TRUE(s.v->splitBlockReason(core) == Vehicle::SplitBlock::WholeShip,
               "the root part's side is the whole ship");
    /* A decoupler that already went off inside an outer cut: same stage, same press,
       ordinary rather than an error -- and the one reason the old game.cpp comment
       named even though it could not apply to a root from this ship's own walk. */
    Part *gone = new Part;
    gone->body = nullptr; gone->def = nullptr; gone->parent = nullptr;
    CHECK_TRUE(s.v->splitBlockReason(gone) == Vehicle::SplitBlock::NotAboard,
               "a part not aboard this ship cannot split it");
    delete gone;
    destroyShip(s);
}

/* A part is dropped because of WHERE it hangs, not how far down: a deep
   chain under one decoupler all goes, and a deep chain outside it all stays.
   Also pins that the result carries no duplicates (it is a set). */
static void test_deep_chain() {
    Ship s; s.v = new Vehicle;
    Part *root = addPart(s, NULL, 2, false, "capsule");
    Part *dec  = addPart(s, root, 1, true,  "decoupler");
    Part *p = dec;
    char nm[32];
    std::set<std::string> expected = want("decoupler");
    for(int i = 0; i < 6; i++) {
        snprintf(nm, sizeof(nm), "link%d", i);
        p = addPart(s, p, 1, false, nm);
        expected.insert(nm);
    }
    /* a bystander deep in the tree but on the other side of the decoupler */
    Part *side = addPart(s, root, 1, false, "bystander");
    (void)side;

    const std::vector<Part *> got = s.v->droppedPartsAtStage(1);
    CHECK_TRUE(got.size() == expected.size(),
               "deep chain: every descendant dropped exactly once");
    checkDrop(s, 1, expected, "deep chain under one decoupler");
    destroyShip(s);
}

/* The shroud condition (Vehicle::hasChildBelow): a child on the part's
   EXHAUST face (below, in the part's OWN frame) shrouds it; a child above
   or on the side does not. The rotated case pins that "below" is judged
   in the part's axes, not the ship frame. */
static void test_child_below() {
    {   // a child below (a down-stack edge)
        Ship s; s.v = new Vehicle;
        Part *eng = addPart(s, NULL, 1, false, "engine");
        Part *dec = addPart(s, eng, 1, false, "decoupler");
        dec->localPos = glm::dvec3(0, 0, -3);
        CHECK_TRUE(s.v->hasChildBelow(eng), "child below shrouds");
        CHECK_TRUE(!s.v->hasChildBelow(dec), "the child has no child of its own");
        destroyShip(s);
    }
    {   // a child above (an up-stack edge)
        Ship s; s.v = new Vehicle;
        Part *eng  = addPart(s, NULL, 1, false, "engine");
        Part *tank = addPart(s, eng, 1, false, "tank");
        tank->localPos = glm::dvec3(0, 0, 3);
        CHECK_TRUE(!s.v->hasChildBelow(eng), "child above does not shroud");
        destroyShip(s);
    }
    {   // a child on the side (a surface edge)
        Ship s; s.v = new Vehicle;
        Part *eng = addPart(s, NULL, 1, false, "engine");
        Part *w   = addPart(s, eng, 1, false, "wheel");
        w->localPos = glm::dvec3(2, 0, 0);
        CHECK_TRUE(!s.v->hasChildBelow(eng), "side child does not shroud");
        destroyShip(s);
    }
    {   // a surface child on the LOWER half of the side: its center is below
        // the parent's midplane, but it is on the wall, not the exhaust
        // face -- the axial guard keeps the shroud off
        Ship s; s.v = new Vehicle;
        Part *eng = addPart(s, NULL, 1, false, "engine");
        Part *w   = addPart(s, eng, 1, false, "wheel");
        w->localPos = glm::dvec3(2, 0, -0.5);
        CHECK_TRUE(!s.v->hasChildBelow(eng),
                   "side-below-midplane does not shroud");
        destroyShip(s);
    }
    {   // above AND below: the below child shrouds
        Ship s; s.v = new Vehicle;
        Part *eng   = addPart(s, NULL, 1, false, "engine");
        Part *upper = addPart(s, eng, 1, false, "tank_upper");
        upper->localPos = glm::dvec3(0, 0, 3);
        Part *dec = addPart(s, eng, 1, false, "decoupler");
        dec->localPos = glm::dvec3(0, 0, -3);
        CHECK_TRUE(s.v->hasChildBelow(eng), "above + below: the below shrouds");
        destroyShip(s);
    }
    {   // rotated parent: "below" is in the part's OWN frame (a +90 deg
        // roll about X sends the exhaust face, local -Z, to world +Y)
        Ship s; s.v = new Vehicle;
        Part *eng = addPart(s, NULL, 1, false, "engine");
        eng->localRot = glm::dmat3(1.0, 0.0, 0.0,
                                   0.0, 0.0, -1.0,
                                   0.0, 1.0, 0.0);
        Part *dec   = addPart(s, eng, 1, false, "decoupler");
        dec->localPos = eng->localRot * glm::dvec3(0, 0, -3);
        Part *upper = addPart(s, eng, 1, false, "tank_upper");
        upper->localPos = eng->localRot * glm::dvec3(0, 0, +3);
        CHECK_TRUE(s.v->hasChildBelow(eng),
                   "rotated: the own-frame-below child shrouds");
        CHECK_TRUE(!s.v->hasChildBelow(upper), "rotated: the up child is bare");
        destroyShip(s);
    }
}

int main() {
    printf("== droppedPartsAtStage (src/vehicle.h) ==\n");
    test_no_decoupler();
    test_subtree();
    test_sibling_survives();
    test_different_stages();
    test_two_on_one_stage();
    test_staged_decouplers_tie();
    test_staged_decouplers_depth_first();
    test_staged_decouplers_nested();
    test_staged_decouplers_other_stage();
    test_split_block_reason();
    test_deep_chain();
    test_child_below();

    printf("%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures) { printf("FAILED\n"); return 1; }
    printf("test_staging: all checks passed\n");
    return 0;
}
