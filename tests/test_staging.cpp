//
// Headless test for Vehicle::droppedPartsAtStage (src/vehicle.h).
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

#include <cstdio>
#include <deque>
#include <set>
#include <string>
#include <vector>

#include "vehicle.h"   // Vehicle + Part (inline); body.h pulls in Bullet

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

int main() {
    printf("== droppedPartsAtStage (src/vehicle.h) ==\n");
    test_no_decoupler();
    test_subtree();
    test_sibling_survives();
    test_different_stages();
    test_two_on_one_stage();
    test_deep_chain();

    printf("%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures) { printf("FAILED\n"); return 1; }
    printf("test_staging: all checks passed\n");
    return 0;
}
