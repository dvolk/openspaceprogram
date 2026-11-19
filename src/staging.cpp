// staging.cpp -- per-stage vacuum delta-v / TWR with fuel-link asparagus.
// See staging.h. Pure math over BuildShip.
#include "staging.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <string_view>

#include "hash.h"   // the staging cache key's FNV-1a mixer
#include "fuelgraph.h"  // the drain-layer walk, shared with the live drain (#965)

namespace {

struct SimPart {
    double dry = 0.0;      // inert mass (structure + non-burnable resources)
    double fuel = 0.0;     // kg of the burnable pool
    double thrust = 0.0;   // N, rocket rated (0 = not an engine)
    double mdot = 0.0;     // kg/s of fuel at full throttle
    bool isDec = false;
    bool isBarrier = false;
    int stage = 1;
    int parent = -1;
    int group = -1;
    bool alive = true;
};

struct SimEngine {
    int part = -1;
    double thrust = 0.0;
    double mdot = 0.0;
    bool lit = false;
    bool fed = false;
};

/* A fuel link resolved to part indices (both ends alive when the snapshot was taken;
   refreshFuel() re-checks liveness every renumbering, since a separation can kill an
   endpoint). */
struct LinkRef { int from, to; };

// Groups the parts, returning the group count (ids are dense from 0).
int buildFuelGroups(std::vector<SimPart> &parts) {
    /* The one partition (shipdef.h, #292). A DEAD part is a wall too, keeping
       the file's standing invariant that a dead part carries group -1. For a
       def the parser produces that is belt-and-braces: Sim::kill only ever
       kills a decoupler's subtree, and the parser forces fuel_barrier on every
       decoupler (shipdef.cpp:272), so the subtree's own root already separates
       it. No reachable state needs this term -- it is here so a hand-built def
       (or a future non-decoupler way to kill a part) cannot leave a dead part
       sharing a live group. */
    /* These three ride between calls (#963): this runs once per fuel renumbering --
       ~940 times an ascent -- and each entry is written before it is read, so
       resize()-then-write is the whole reset. partitionFuelGroups() assigns `group` in
       full (its contract, shipdef.h), so that one needs no clear of its own. */
    static thread_local std::vector<int> parent;
    static thread_local std::vector<bool> wall;
    static thread_local std::vector<int> group;
    parent.resize(parts.size());
    wall.resize(parts.size());
    for(size_t i = 0; i < parts.size(); i++) {
        parent[i] = parts[i].parent;
        wall[i] = parts[i].isBarrier || !parts[i].alive;
    }
    const int n = partitionFuelGroups(parent, wall, group);
    for(size_t i = 0; i < parts.size(); i++) { parts[i].group = group[i]; }
    return n;
}

double totalMass(const std::vector<SimPart> &parts) {
    double m = 0.0;
    for(size_t i = 0; i < parts.size(); i++) {
        if(parts[i].alive) { m += parts[i].dry + parts[i].fuel; }
    }
    return m;
}

/* The one bucketed index this file needs (a CSR): bucket b owns
   items[start[b] .. start[b+1]). `key(i)` says which bucket item i belongs to, or -1 to
   leave it out. Both passes go through the SAME key, which is the whole point of the
   helper: a histogram that accepts an item the fill rejects (or the reverse) writes
   `items` past its end, and nothing else in here would notice. #534 folded the two
   hand-rolled copies (group members, part children) into this one. */
template <class Key>
void buildBuckets(int nBuckets, int nItems, const Key &key,
                  std::vector<int> &start, std::vector<int> &items) {
    start.assign((size_t)nBuckets + 1, 0);
    for(int i = 0; i < nItems; i++) {
        const int b = key(i);
        assert(b < nBuckets);            // a bucket id outside the table is a bug
        if(b >= 0) { start[(size_t)b + 1]++; }
    }
    for(int b = 0; b + 1 < (int)start.size(); b++) {
        start[(size_t)b + 1] += start[(size_t)b];
    }
    items.assign((size_t)nItems, 0);
    /* The write cursor is a copy of `start`, so it used to be a fresh container per call;
       this runs twice a fuel renumbering and both sizes are the part or group count
       (#963). Assigned rather than rebuilt, so only a growing bucket/item count costs
       anything. Note that this static belongs to the INSTANTIATION, not to the thread:
       the two call sites pass different key closures, so they already have two separate
       cursors. That is not what makes nesting safe -- a third caller that reused one
       key type and nested inside a buildBuckets call would share this buffer and corrupt
       the outer fill. The body calls nothing but key(), asserts and vector ops, so the
       only way to nest it is a caller whose key() calls buildBuckets; that would need its
       own cursor passed in. */
    static thread_local std::vector<int> cursor;
    cursor = start;
    for(int i = 0; i < nItems; i++) {
        const int b = key(i);
        if(b < 0) { continue; }
        items[(size_t)cursor[(size_t)b]++] = i;
    }
    // Per bucket: its cursor ran exactly to where its count said it would. A key that
    // answers differently in the two passes shows up here whenever that change moves
    // an item in or out of a bucket's COUNT; a straight swap between two buckets keeps
    // every count and cannot be caught this way (nor by anything cheaper than
    // re-running key() over the result). Both call sites pass a pure key -- a part's
    // group id, a clamped parent -- so this is a tripwire for a future caller rather
    // than a check the current ones could fail.
    for(int b = 0; b < nBuckets; b++) {
        assert(cursor[(size_t)b] == start[(size_t)b + 1]);
    }
}

struct Sim {
    std::vector<SimPart> parts;
    std::vector<SimEngine> engines;
    /* group -> the groups that feed it, indexed by group id (#963). Group ids are dense
       from 0, and this is rewritten from scratch at every refreshFuel(); the std::map it
       replaces billed a node per linked group and a vector per key, per renumbering.
       `resize` keeps the inner vectors' capacity, so only a growing group count costs
       anything; every one is cleared before the links are written. */
    std::vector<std::vector<int> > revOf;
    /* Drain layers per group, indexed by group id and generation-stamped (#963) -- the
       same epoch idiom as reachEpoch below. layersOf[g].epoch == layersEpoch means
       "THIS numbering's", and a renumbering costs one integer bump instead of freeing
       every cached layer set: the FuelLayers' two buffers then carry their capacity across
       renumberings and across passes, where the old map<int, FuelLayers> billed a node plus
       a vector per layer per cached group (15,126 allocations an ascent).
       A returned `const FuelLayers *` outlives nothing that could move it: the only holders
       are computeRates() (one engine's layer walk) and markReachable() (one marking
       loop), and refreshFuel() -- the only resize of this table, and it runs BOTH before
       the walk and again after a mid-pass kill() -- is called only after both have
       returned. It has to be that way: a vector element that is reallocated MOVES, so an
       outstanding FuelLayers * would dangle outright rather than read stale data, unlike the
       map whose nodes never moved.
       The table grows only, so an entry past nGroups keeps the previous numbering's
       layers; layersFor() is only ever asked for a group id below nGroups, and asserts as
       much. */
    struct LayerCache {
        uint32_t epoch = 0;
        FuelLayers layers;
    };
    std::vector<LayerCache> layersOf;
    /* 1, not 0: epoch 0 means "no generation yet", so a LayerCache that resize() just
       default-constructed is never current however the two lines in refreshFuel() are
       ordered. The assert there turns a counter wrap into a crash rather than a cache
       that reads the pre-wrap numbering (#963). */
    uint32_t layersEpoch = 1;
    /* group -> its parts, flat: group g owns memberOf[memberStart[g] ..
       memberStart[g+1]). Built once per refreshFuel (#293): the drain loop used
       to rescan the whole ship for every group in every layer, which made a
       240-group ship 5.6x the cost of a one-group ship of the same part count.
       Alive parts only (a dead part has group -1), so the drain loop needs no
       liveness test. */
    int nGroups = 0;
    std::vector<int> memberStart;
    std::vector<int> memberOf;

    /* One Sim now serves every pass (#963): a flight rebuilds the plan 755 times and
       each container here is sized to the part or group count, so the last pass's
       capacity is exactly what the next one wants. The bargain this strikes is the one
       the epoch buffers above already strike -- a stale entry here is a wrong PLAN
       rather than a crash, so the reset list has to be complete. What is covered where:
         parts        resize() keeps capacity; the snapshot loop writes dry, fuel,
                      thrust, mdot and alive itself (resize() only initializes slots
                      ADDED since the last pass). `group` is the exception: it belongs
                      to buildFuelGroups(), which writes every entry.
         engines      cleared here (the pass push_backs, and appending to the previous
                      ship's engines would light engines that are not aboard);
         litCount     zeroed here; markLitCount back to its fresh-Sim -1, so a
                      drainableIn() that ran without this pass's markReachable() still
                      trips. That last one is insurance a test cannot reach (dropping it
                      leaves all 383 checks green, measured): the reused Sim would
                      otherwise quietly lose a fresh one's tripwire, so it is restored
                      and stated as unpinnable rather than dropped.
         revOf        cleared at refreshFuel(), then refilled from the links;
         layersOf     generation bump at refreshFuel(), no free (see above);
         memberStart/memberOf, childStart/childOf, seenEpoch and reachEpoch are
                      assigned over their whole range by rebuildMembers(),
                      buildChildren() and refreshFuel().
       Not reset, because nothing can read the old value before it is rewritten: nGroups
       (refreshFuel() assigns it before the walk, and the link block above reads only
       ids) and dropStack (droppedAt() drains it on every path and asserts it is empty on
       entry, which is the assert that guards the use -- repeating it here would only put
       the same invariant in two places).
       buildChildren() is what restarts seenTick, and refreshFuel() reachTick and
       layersEpoch, because those are where the sizes are decided. */
    void resetFor(size_t nParts) {
        parts.resize(nParts);
        engines.clear();
        litCount = 0;
        markLitCount = -1;
        stageLabels.clear();   // the label loop below push_backs into it
        linkRefs.clear();      // so does the fuel-link lookup (#963)
    }

    /* The stage labels this pass walks (#517), the drop list, the burn buffers and the
       resolved fuel links: one set per Sim instead of one per call site (#963). Each is
       cleared where it is filled -- droppedAt() clears the drop list, computeRates()
       overwrites the rate buffer in full, resetFor() empties the label list and the link
       list -- so none of them can carry an answer from the previous pass. */
    std::vector<int> stageLabels;
    std::vector<int> dropList;
    std::vector<double> burnRate;
    std::vector<double> burnRatio;
    std::vector<LinkRef> linkRefs;

    void rebuildMembers() {
        buildBuckets(nGroups, (int)parts.size(),
                     [this](int i) { return parts[(size_t)i].group; },   // -1 = not in a group
                     memberStart, memberOf);
    }

    /* part -> its children, flat: part p owns childOf[childStart[p] ..
       childStart[p+1]). Built once from the snapshot (#520), NOT per droppedAt()
       call: the stage walk calls it for every label the ship carries, and the old
       per-call map rebuild billed the label count times the part count for a
       graph that never changes. Listed here are ALL the parts, alive or not: the
       table is built before any kill(), and filtering by liveness now would freeze
       build-time liveness into a structure the walk reads after parts die (the
       group member table above is the opposite case -- it is rebuilt per refresh,
       so it can filter). Parent links never change afterwards (kill() only flips a
       flag), so one build per pass is enough. */
    std::vector<int> childStart;
    std::vector<int> childOf;
    /* Scratch for one drop walk (#533): a stamp per part standing in for a visited SET,
       because the walk only ever needs "have I seen this part" and "what did I collect".
       Stamping an epoch instead of setting and clearing bytes means there is nothing to
       undo: no clear pass, no marks left standing if the walk throws, and no cross-call
       invariant to remember. Sized with the snapshot in buildChildren(); 0 is never a
       live epoch, so a fresh vector starts clean. */
    std::vector<uint32_t> seenEpoch;
    uint32_t seenTick = 0;
    /* The drop walk's LIFO (#963). Part of the Sim rather than a local because the walk
       runs once per stage label; the walk drains it, so it is empty whenever it is next
       entered -- asserted, not cleared, so a walk that leaves a seed's children standing
       shows up instead of leaking into the next label. */
    std::vector<int> dropStack;

    /* The same epoch trick for drainableIn (#536): which groups a LIT engine can draw
       from, stamped once per burn instead of rebuilt into a std::set per query (#538).
       Sized and cleared in refreshFuel, because that is where buildFuelGroups
       renumbers the groups. markReachable() owns the ordering rule. */
    std::vector<uint32_t> reachEpoch;
    uint32_t reachTick = 0;
    /* litCount is how many engines are lit; markLitCount is what it was when the marks
       were stamped. drainableIn asserts they match, which states #538's precondition
       outright. The count belongs to the only three writers of engines[].lit (the
       pre-loop pass, the label's lighting, kill()); markReachable() checks it against
       what its own walk finds lit. */
    int litCount = 0;
    int markLitCount = -1;

    void buildChildren() {
        const int n = (int)parts.size();
        /* ONE predicate for "usable parent": the two passes inside buildBuckets go
           through it, so they cannot disagree by one test and write childOf past its
           end. (Malformed parents cannot reach a game -- partitionFuelGroups asserts
           p < n from refreshFuel -- but that runs after this, so this is the first
           code to touch them.) */
        auto parentOf = [this, n](int i) {
            const int p = parts[(size_t)i].parent;
            return (p >= 0 && p < n) ? p : -1;
        };
        buildBuckets(n, n, parentOf, childStart, childOf);
        seenEpoch.assign(parts.size(), 0);
        seenTick = 0;   // 0 means "no walk has run", so the fresh vector starts clean
    }

    /* Decoupler-on-stage drop: the decouplers carrying this label plus everything
       on their child side (Vehicle::droppedPartsAtStage). Ascending part order, which
       is what the std::set this used to return gave: drainableIn sums fuel in iteration
       order, so keeping the order keeps the rows bit-identical (#533).

       Fills `dropped` (cleared first) rather than returning it, and walks on the Sim's
       own stack (#963): one walk per stage label inside a pass that runs 755 times an
       ascent, and the caller declares its vector outside the label loop, so both
       buffers' capacity is what gets reused instead of being remade per label. */
    void droppedAt(int stage, std::vector<int> &dropped) {
        /* Only proves the tables match the part count -- that buildChildren() ran
           at all and nothing resized parts afterwards. A table built BEFORE the
           snapshot has the right size and the wrong edges, so keep the call in
           computeStaging where the snapshot loop ends. */
        assert(childStart.size() == parts.size() + 1);
        assert(seenEpoch.size() == parts.size());
        /* One epoch per walk. A pass runs at most one walk per stage label, and labels
           never outnumber parts, so the counter cannot reach its range inside a Sim --
           buildChildren() restarts it with the snapshot. Asserted, not handled. */
        ++seenTick;
        assert(seenTick != 0);
        dropped.clear();
        assert(dropStack.empty());   // the walk below drains it; nothing else touches it
        for(size_t i = 0; i < parts.size(); i++) {
            if(!parts[i].alive || !parts[i].isDec || parts[i].stage != stage) { continue; }
            if(seenEpoch[i] != seenTick) { seenEpoch[i] = seenTick; dropped.push_back((int)i); }
            for(int k = childStart[i]; k < childStart[i + 1]; k++) { dropStack.push_back(childOf[k]); }
            while(!dropStack.empty()) {
                const int q = dropStack.back();
                dropStack.pop_back();
                if(seenEpoch[(size_t)q] == seenTick) { continue; }
                seenEpoch[(size_t)q] = seenTick;
                // The child table is built before any kill(), so a live seed's subtree
                // can still list parts an earlier separation took. List only the alive
                // ones (#537). Descend either way: no rule says a dead part's child is
                // dead, and pruning dead subtrees costs a live part its whole burn the
                // moment that rule breaks (dv 136 against 1435 on a shape built to force
                // it). Cycles are not the reason either: a part has one parent, so a cycle
                // cannot be entered from outside, and a self-parented seed is turned away
                // by the seed's own stamp.
                if(parts[(size_t)q].alive) { dropped.push_back(q); }
                for(int k = childStart[(size_t)q]; k < childStart[(size_t)q + 1]; k++) {
                    dropStack.push_back(childOf[k]);
                }
            }
        }
        /* Ascending, i.e. the iteration order the std::set this used to return. It is
           what drainableIn sums fuel in; every ship in res/ships reads identically
           either way (measured), so this is insurance for a big drop set with inexact
           sums, not a live difference. */
        std::sort(dropped.begin(), dropped.end());
    }

    /* Drain layers for one group, computed on first use (#293). Only a group
       that holds a lit engine is ever asked for, and computing them for every
       group -- a BFS each, over a graph as big as the group set -- was the bulk
       of a deep asparagus build's cost once the drain loop stopped rescanning.
       The returned pointer has to outlive later writes to the cache: the drain
       loop holds it across a whole layer walk while later engines insert. A dense
       LayerCache does NOT give that for free -- a resize moves elements -- it holds
       only because no resize happens while a pointer is outstanding (the layersOf
       field above names the holders and when). What the dense table does buy is the
       end of the node-per-group and vector-per-layer the map billed (#963). */
    const FuelLayers *layersFor(int g) {
        assert(g >= 0 && g < nGroups && g < (int)layersOf.size());
        LayerCache &c = layersOf[(size_t)g];
        if(c.epoch == layersEpoch) { return &c.layers; }
        c.epoch = layersEpoch;
        drainLayersFor(revOf, g, nGroups, c.layers);
        return &c.layers;
    }

    // Per-tank drain rates (kg/s) from every lit engine with fuel in its layers.
    // Each engine fills its mdot from the furthest non-empty bucket, pro-rata by
    // remaining fuel.
    void computeRates(std::vector<double> &rate) {
        rate.assign(parts.size(), 0.0);
        for(size_t ei = 0; ei < engines.size(); ei++) {
            SimEngine &e = engines[ei];
            e.fed = false;
            if(!e.lit || e.mdot <= 0.0) { continue; }
            if(e.part < 0 || e.part >= (int)parts.size() || !parts[e.part].alive) { continue; }
            const int g = parts[e.part].group;
            if(g < 0) { continue; }
            const FuelLayers *lit = layersFor(g);
            double need = e.mdot;
            for(size_t li = 0; li + 1 < lit->layerStart.size() && need > 0.0; li++) {
                const int b0 = lit->layerStart[li];
                const int b1 = lit->layerStart[li + 1];
                double layerFuel = 0.0;
                for(int k = b0; k < b1; k++) {
                    const int b = lit->groupOf[(size_t)k];
                    // Every groupOf entry is a revLinks key or value: a real group id.
                    assert(b >= 0 && b < nGroups);
                    for(size_t m = (size_t)memberStart[(size_t)b];
                        m < (size_t)memberStart[(size_t)b + 1]; m++) {
                        layerFuel += parts[(size_t)memberOf[m]].fuel;
                    }
                }
                if(layerFuel <= 0.0) { continue; }
                for(int k = b0; k < b1; k++) {
                    const int b = lit->groupOf[(size_t)k];
                    assert(b >= 0 && b < nGroups);
                    for(size_t m = (size_t)memberStart[(size_t)b];
                        m < (size_t)memberStart[(size_t)b + 1]; m++) {
                        const size_t pi = (size_t)memberOf[m];
                        const double have = parts[pi].fuel;
                        if(have <= 0.0) { continue; }
                        rate[pi] += need * (have / layerFuel);
                    }
                }
                e.fed = true;
                need = 0.0;
            }
        }
    }

    /* computeRates' buffer for fedThrust alone (#963), which runs at both ends of a
       burn rather than once a pass. runBurn's drain loop keeps its own rate vector, so
       the two hold separate rows: fedThrust answers off engines[].fed, which
       computeRates writes, and no caller of either reads a rate after it returns. That
       separation is a convention, so runBurn asserts its buffers are not this one. */
    std::vector<double> fedRate;

    double fedThrust() {
        computeRates(fedRate);   // the Sim's buffer: fedThrust runs per burn sample (#963)
        double F = 0.0;
        for(size_t ei = 0; ei < engines.size(); ei++) {
            if(engines[ei].fed) { F += engines[ei].thrust; }
        }
        return F;
    }

    int litRocketCount() const {
        int n = 0;
        for(size_t ei = 0; ei < engines.size(); ei++) {
            const SimEngine &e = engines[ei];
            if(e.lit && e.part >= 0 && e.part < (int)parts.size() && parts[e.part].alive) {
                n++;
            }
        }
        return n;
    }

    bool anyFuel() const {
        for(size_t i = 0; i < parts.size(); i++) {
            if(parts[i].alive && parts[i].fuel > 0.0) { return true; }
        }
        return false;
    }

    /* Which groups a LIT engine can draw from, stamped into an epoch buffer once per
       burn instead of rebuilt into a set per query (#536, #538). What gets marked
       depends only on the lit set, and after this point in the label iteration nothing
       lights an engine again while the marks are read; the only un-lighting is kill(),
       which runs after the burn. Marking BEFORE that label's lighting marks the previous
       period's lit set instead -- test_reachability_survives_a_renumbering reads
       s2 m=360->140 when that happens, and the markLitCount assert says so outright.
       Not const: layersFor fills its cache. */
    void markReachable() {
        assert(reachEpoch.size() == (size_t)nGroups);   // refreshFuel() sizes this
        ++reachTick;
        assert(reachTick != 0);
        markLitCount = litCount;
        int litSeen = 0;
        for(size_t ei = 0; ei < engines.size(); ei++) {
            const SimEngine &e = engines[ei];
            if(!e.lit || e.part < 0 || e.part >= (int)parts.size() || !parts[e.part].alive) {
                continue;
            }
            litSeen++;
            const int g = parts[e.part].group;
            if(g < 0) { continue; }
            const FuelLayers *layers = layersFor(g);
            /* The flat list IS the union of the layers, so marking needs no layer
               boundaries -- and order does not matter here, unlike in computeRates. */
            for(size_t k = 0; k < layers->groupOf.size(); k++) {
                const int rg = layers->groupOf[k];   // a GROUP, not a part
                assert(rg >= 0 && rg < (int)reachEpoch.size());
                reachEpoch[(size_t)rg] = reachTick;
            }
        }
        /* litCount is the writers' running count, litSeen is what this loop actually
           finds lit. They agree only if every site that flips engines[].lit stepped the
           count exactly once -- free to check, since the loop is already here. */
        assert(litSeen == litCount);
    }

    // Propellant in `list` that a lit engine can still reach via its drain
    // layers. The "ready to drop" test. Reads the marks markReachable() stamped,
    // so it is only meaningful after that call within the same period -- the
    // litCount assert is what makes "the same period" checkable.
    double drainableIn(const std::vector<int> &list) {
        if(list.empty()) { return 0.0; }
        assert(reachEpoch.size() == (size_t)nGroups);   // refreshFuel() sizes this
        assert(markLitCount == litCount);   // the marks must describe THIS lit set
        double m = 0.0;
        for(size_t k = 0; k < list.size(); k++) {
            const int pi = list[k];
            /* Two things already exclude a dead part: buildFuelGroups marks dead as a
               wall so it carries group -1, and droppedAt() stopped listing it (#537). So
               this skip is a tripwire -- deleting it changes no row anywhere. The rule
               that a dead tank's fuel is not drainable lives in the group split. */
            if(pi < 0 || pi >= (int)parts.size() || !parts[pi].alive) { continue; }
            const int pg = parts[pi].group;
            if(pg < 0) { continue; }
            assert(pg < (int)reachEpoch.size());
            if(reachEpoch[(size_t)pg] != reachTick) { continue; }
            m += parts[pi].fuel;
        }
        return m;
    }

    /* Indices are in range and unique: they come from droppedAt(), whose child table
       rejects out-of-range parents (#520) and stamps each part once per walk, so this
       write is unchecked. drainableIn re-reads the parts for its own reasons -- see the
       note there. */
    void kill(const std::vector<int> &list) {
        for(size_t k = 0; k < list.size(); k++) {
            parts[(size_t)list[k]].alive = false;
        }
        for(size_t ei = 0; ei < engines.size(); ei++) {
            SimEngine &e = engines[ei];
            /* Only a LIT engine may step litCount. Without the lit test this also steps
               for engines that never lit -- labels go high to low, so a part under a
               decoupler an earlier period took dies before its own label ever lights it
               -- and it steps again for every engine an earlier kill() already stepped,
               since this loop scans all engines rather than this drop. That drifts to
               -18 on res/ships/heavy_asp.json while litRocketCount() reads 1, after which
               markReachable compares a counter that stopped meaning "lit". */
            if(e.lit && e.part >= 0 && e.part < (int)parts.size()
               && !parts[(size_t)e.part].alive) {
                e.lit = false;
                litCount--;
            }
        }
    }
};

// One burn stretch at a (piecewise) constant engine set. Vacuum delta-v
// from rockets; min TWR is pinned to the ignition sample (heaviest).
struct BurnAcc {
    double dv = 0.0;
    double m0 = 0.0;
    double m = 0.0;
    double minTWR = 0.0;
    double maxTWR = 0.0;
    bool anyTWR = false;

    void sampleTWR(double thrust, double mass, double g, bool pinMin) {
        if(thrust <= 0.0 || mass <= 0.0 || g <= 0.0) { return; }
        const double twr = thrust / (mass * g);
        if(!anyTWR) {
            minTWR = maxTWR = twr;
            anyTWR = true;
        } else {
            if(pinMin && twr < minTWR) { minTWR = twr; }
            if(twr > maxTWR) { maxTWR = twr; }
        }
    }
};

// Drain until `watch` is empty of drainable fuel, or -- when `drainAll`
// -- until no lit engine can draw more. Events are tank-empty boundaries.
/* `rate` and `ratio` come from the caller (#963): a pass runs one burn per stage
   label and both are sized to the part count, so declaring them outside the label loop
   is what keeps that capacity. computeRates overwrites rate in full; ratio is written
   only where rate > 0 and read only there, so neither can carry a stale answer. What
   they must NOT be is the Sim's own fedThrust buffer: the ignition sample below calls
   computeRates(sim.fedRate) from inside this loop, so an aliased rate would be zeroed
   mid-burn and the period would end early with plausible-looking rows. Asserted rather
   than left to the comment at fedRate. */
void runBurn(Sim &sim, double g, bool drainAll,
             const std::vector<int> &watch, BurnAcc &acc,
             std::vector<double> &rate, std::vector<double> &ratio) {
    assert(&rate != &sim.fedRate && &ratio != &sim.fedRate);
    acc.m0 = totalMass(sim.parts);
    acc.m = acc.m0;
    acc.dv = 0.0;
    acc.anyTWR = false;

    auto ready = [&]() -> bool {
        return drainAll ? sim.anyFuel() : sim.drainableIn(watch) > 1e-12;
    };

    // Ignition sample: min TWR is this one (heaviest mass).
    acc.sampleTWR(sim.fedThrust(), acc.m, g, true);

    const int kMaxSteps = (int)sim.parts.size() * 8 + 32;
    int step = 0;
    for(; step < kMaxSteps && ready(); step++) {
        sim.computeRates(rate);
        double F = 0.0, mdot = 0.0;
        for(size_t ei = 0; ei < sim.engines.size(); ei++) {
            if(!sim.engines[ei].fed) { continue; }
            F += sim.engines[ei].thrust;
            mdot += sim.engines[ei].mdot;
        }
        if(mdot <= 0.0 || F <= 0.0) { break; }

        /* The step runs to the first tank that empties, over EVERY part this plan
           bills -- residues included. computeRates pro-rates a fed engine's whole
           mdot over the parts it drains (skipping only the ones holding literally
           nothing), so a part with a rate holds fuel and is spent first. Skipping
           near-empty ones -- what this loop used to do -- left dt set by the OTHER
           tanks while mdot*dt still charged all of every fed engine: the mass in the
           Tsiolkovsky sum ran ahead of the fuel that had actually left the tanks. On
           a deep fuel-link chain (six strings is enough) that over-charge compounded
           until the table read dv 1.8e6 m/s, and further until it read -nan (#526). */
        ratio.resize(rate.size());
        double dt = 1e300;
        for(size_t i = 0; i < rate.size(); i++) {
            if(rate[i] <= 0.0) { continue; }
            const double t = sim.parts[i].fuel / rate[i];
            ratio[i] = t;
            if(t < dt) { dt = t; }
        }
        if(!(dt < 1e299) || dt <= 0.0) { break; }

        double drained = 0.0;
        for(size_t i = 0; i < sim.parts.size(); i++) {
            if(rate[i] <= 0.0) { continue; }
            /* A tank whose emptying time IS this step comes out EXACTLY empty.
               Pro-rata spreading ties a whole fuel layer on that instant, so
               subtracting instead would leave every tank in it a rounding residue:
               the layer would still read as drainable, the same engines would be fed
               against dust, and the burn would spend steps chasing residues down
               until they went subnormal and a step changed nothing. "IS this step"
               means bit-equal to the ratio computed above -- the same division, so
               the boundary tanks are exactly the argmin. A tank that misses the tie
               by an ulp keeps an ulp of fuel and becomes the next step's boundary:
               one extra step, not a chase. */
            const double have = sim.parts[i].fuel;
            const double take = (ratio[i] == dt) ? have : std::min(rate[i] * dt, have);
            sim.parts[i].fuel = have - take;
            drained += take;
        }
        /* The two halves of the comment above, pinned: a step spends what it bills.
           Only fp slack (sum(rate) against the fed mdot is a rounding sum) -- and
           that slack is why the charge below reads `drained` and not `mdot * dt`. */
        assert(std::fabs(drained - mdot * dt) <= 1e-9 * (mdot * dt));

        /* Bill the drain, not the plan (#531, in the preview). The rates pro-rate each
           fed engine's mdot over the tanks it reaches, so sum(rate) is a rounding sum
           against mdot and `mdot * dt` is a PLAN: on the suite's own fixtures 105 of
           218 burn steps charge a mass that differs from what their tanks gave up (up
           to 7e-11 kg, 6e-16 relative). That is fp-scale noise today -- it survived
           precisely because it is -- but the Tsiolkovsky cursor is the same ledger
           #531 found drifting in flight, and charging the drain makes it the parts'
           mass by construction instead of by tolerance (runBurn's last line still
           re-reads the parts, since summing there and summing here walk in different
           orders). Note what keeps m1 positive: the DRY mass. The one def with no mass
           is a fuel_link marker ("fuel_link": true skips the mass block,
           shipdef.cpp:330), and no loader lets one become a Part: the ship file answers
           `a fuel link needs "from" and "to"` (shipdef.cpp:1092) and the save refuses
           it for parts, seats and contents alike (refuseVirtualPartDef, save.cpp:121).
           So this is a corrupt-state trap, not a report an author would ever see (#530). */
        const double m1 = acc.m - drained;
        assert(m1 > 0.0);
        acc.dv += (F / mdot) * std::log(acc.m / m1);
        acc.m = m1;
        // End-of-step TWR with THIS step's thrust (an engine that flames
        // out on the emptying tank still pushed through the whole step).
        acc.sampleTWR(F, acc.m, g, false);
    }
    /* Not a safety net: every step takes at least one tank from full-empty to
       exactly empty and nothing refuels, so a period cannot outlive its parts. If
       this fires, a burn is being cut short and its rows silently under-report. */
    assert(step < kMaxSteps);
    acc.m = totalMass(sim.parts);
    if(!acc.anyTWR) {
        // Zero-length burn: still report ignition TWR once.
        acc.sampleTWR(sim.fedThrust(), acc.m, g, true);
    }
}

} // namespace

double partPropellantMass(const PartDef &def, const bool *burn) {
    // EC is Wh, not kg; JetFuel / hydrazine / life support ride along as inert.
    double m = 0.0;
    for(size_t r = 0; r < def.capacity.size(); r++) {
        if(burn[r]) { m += (double)def.capacity[r]; }
    }
    return m;
}

/* The resources the ship's ROCKET engines draw (jets excluded: they need air,
   so no vacuum delta-v). A tank resource no engine burns is inert mass.
   LIMITATION: a ship-wide UNION, and a part's propellant is one pool -- a ship
   MIXING chemical (H2+LOX) and nuclear-thermal (H2 only) engines credits every
   tank's LOX as burnable even where only the H2 engine can draw it. Exact for
   a homogeneous engine set; mixed ships need a per-fuel-group mask (#41).
   Split out of computeStaging because a live ship's caller must weigh its
   current contents with the same rule (#948). */
void stagingPropellantMask(const BuildShip &ship, bool *burn) {
    for(int r = 0; r < (int)ResourceType::Num; r++) { burn[r] = false; }
    for(const BuildPart &bp : ship.parts) {
        const PartDef *def = bp.def;
        if(def == nullptr || def->jet) { continue; }
        if(!(def->totalPropellantRate() > 0.0 && def->exhaust_velocity > 0.0)) { continue; }
        for(int r = 0; r < (int)ResourceType::Num; r++) {
            if(def->propellant_rate[r] > 0.0) { burn[r] = true; }
        }
    }
}

uint64_t stagingSignature(const BuildShip &ship, double g, double exhaust_scale) {
    /* Same normalization computeStaging applies, so two builds that would
       produce identical rows share one cache entry. */
    if(!(exhaust_scale > 0.0)) { exhaust_scale = 1.0; }

    uint64_t h = fnv::start();
    fnv::mix(h, (uint64_t)ship.parts.size());
    fnv::mix(h, (uint64_t)ship.fuelLinks.size());
    for(size_t i = 0; i < ship.parts.size(); i++) {
        const BuildPart &bp = ship.parts[i];
        /* The def POINTER, not its contents: the catalog is loaded once
           (main.cpp's load_parts_catalog, moved into Game::parts) and no
           PartDef is written after load, so identity is the whole state
           computeStaging reads from it (mass, capacity, rates,
           barriers). If a parts hot-reload ever lands, this assumption breaks
           -- a freed catalog's addresses get reused by the next one and the
           cache goes silently stale. Mix a catalog epoch in then. */
        fnv::mix(h, (uint64_t)(uintptr_t)bp.def);
        fnv::mix(h, (uint64_t)(bp.parent + 1));
        fnv::mix(h, (uint64_t)bp.stage);
        /* The ID, not just the shape: computeStaging resolves every fuel link by
           looking its endpoint ids up among the parts (only for ships that author
           links), so swapping the ids of two same-def parts can move a link across a
           fuel barrier and change the rows. Loading design B over design A must
           not show A's numbers -- so ids stay mixed in even for link-free ships,
           where they cost a hash apiece and nothing else. */
        fnv::mixText(h, bp.id);
    }
    for(size_t k = 0; k < ship.fuelLinks.size(); k++) {
        const BuildShip::FuelLink &fl = ship.fuelLinks[k];
        /* Only the ENDPOINTS: a link's own id is never read, and an id that
           appears in no link cannot change a row. */
        fnv::mixText(h, fl.from);
        fnv::mixText(h, fl.to);
    }
    fnv::mixDouble(h, g);
    fnv::mixDouble(h, exhaust_scale);
    return h;
}

std::vector<StageRow> computeStaging(const BuildShip &ship, double g,
                                     double exhaust_scale,
                                     const StagingSeed &seed) {
    std::vector<StageRow> rows;
    if(ship.parts.empty()) { return rows; }
    if(!(exhaust_scale > 0.0)) { exhaust_scale = 1.0; }

    const size_t n = ship.parts.size();
    assert(seed.fuelKg.empty() || seed.fuelKg.size() == n);
    assert(seed.dryKg.empty() || seed.dryKg.size() == n);
    /* One Sim for every pass (#963). The plan is recomputed ~755 times a flight and the
       Sim is the biggest set of containers in the path -- parts, engines, child and
       member tables, epoch marks, cached drain layers -- so letting it die at `return`
       threw away exactly the capacity the next pass wanted. thread_local because the
       cache is only sound if the resets in resetFor() pair with the epoch resets, and
       only staging.cpp reads any of it; a per-Sim scratch held by Vehicle::stagedRows
       would put the same bargain behind a public type for one caller. See resetFor()
       for the field-by-field story of what survives and what cannot. */
    static thread_local Sim sim;
    sim.resetFor(n);
    int minStage = ship.parts[0].stage;
    int maxStage = minStage;

    bool burn[(int)ResourceType::Num];
    stagingPropellantMask(ship, burn);

    for(size_t i = 0; i < n; i++) {
        const BuildPart &bp = ship.parts[i];
        SimPart &sp = sim.parts[i];
        /* BuildShip::recomputePoses asserts every parent is an earlier part, so
           the snapshot cannot be torn here (partitionFuelGroups asserts it too). */
        sp.parent = bp.parent;
        sp.stage = bp.stage;
        /* The same wall rule flight uses (PartDef::isFuelBarrier). The parser
           forces fuel_barrier on every decoupler and docking port (shipdef.cpp),
           so spelling out `decoupler` here is a second rule free to drift (#292). */
        sp.isBarrier = (bp.def != nullptr) && bp.def->isFuelBarrier();
        sp.isDec = (bp.def != nullptr) && bp.def->decoupler;
        /* Written here rather than left to SimPart's defaults, because a Sim now rides
           between passes (#963): resize() only value-initializes parts ADDED since the
           last pass, so this slot still holds the previous ship's verdict. `alive` is
           what kill() clears; dry, fuel, thrust and mdot are only assigned below for
           parts that have a def (or qualify as engines), so a slot that carried mass
           last pass must be zeroed BEFORE those tests, not after. `group` is the one
           field NOT reset here: buildFuelGroups() writes every entry of it before
           anything can read one, which is why removing this write changes no check in
           the suite -- the mutation table says so rather than claiming a reset that is
           not load-bearing. */
        sp.alive = true;
        sp.dry = 0.0;
        sp.fuel = 0.0;
        sp.thrust = 0.0;
        sp.mdot = 0.0;
        if(bp.def != nullptr) {
            /* Seeded figures are already the ship's own accounting (a live part's
               effectiveMass split by the burn mask), so they override the def's
               rather than adjust it. Clamped at 0: a negative dry mass would make
               the burn's ln(m0/m1) see a mass the sim never had. */
            sp.dry = std::max(0.0, seed.dryKg.empty() ? partDryMass(*bp.def, burn)
                                                      : seed.dryKg[i]);
            sp.fuel = std::max(0.0, seed.fuelKg.empty() ? partPropellantMass(*bp.def, burn)
                                                        : seed.fuelKg[i]);
            if(bp.def->totalPropellantRate() > 0.0 && bp.def->exhaust_velocity > 0.0
               && !bp.def->jet) {
                // Scale thrust only (not mdot): ve_eff = F/mdot scales, so
                // delta-v and TWR track the difficulty knob like flight.
                sp.thrust = bp.def->fullThrust() * exhaust_scale;
                sp.mdot = bp.def->totalPropellantRate();
            }
        }
        if(sp.stage < minStage) { minStage = sp.stage; }
        if(sp.stage > maxStage) { maxStage = sp.stage; }
    }
    sim.buildChildren();   // the parent links are fixed by the snapshot (#520)

    /* Resolving a fuel link means turning an authored part id into an index. The
       std::map that used to do it held one node PER PART, built on every pass that has
       any link at all: 975 of the 978 allocations a 1,060-part linked build pays, and
       253 of 256 for its 268-part little brother (#963 measured; the same shape without
       links costs 3). A sorted vector of string_views into the parts' own ids costs one
       scratch buffer that is reused between passes and no node per part.
       The rule that had to survive is WHICH part wins when an id is duplicated:
       `idIndex[id] = i` over the parts in order left the LAST one, so the lookup uses
       upper_bound and steps back one. That is only equivalent because the sort below
       tie-breaks on the index -- std::sort is NOT stable, so ordering the ids alone
       leaves a duplicate's position inside its block unspecified, and a 22-entry probe
       of that version picked the FIRST duplicate: the opposite rule, and silently, since
       it changes which fuel link crosses a barrier rather than crashing (#963's review).
       With the tie-break the order is total, so the last entry of a block IS the highest
       index. The loaders reject duplicate ids (shipdef.cpp), so this is a kept rule
       rather than reachable state.
       The views dangle between passes -- cleared before use, never read between calls,
       like the other scratch here. A ship that authors no fuel link (every shipped hull
       but heavy_asp and heavy_two) never sorts at all. */
    if(!ship.fuelLinks.empty()) {
        static thread_local std::vector<std::pair<std::string_view, int> > byId;
        byId.clear();
        byId.reserve(n);
        for(size_t i = 0; i < n; i++) {
            byId.push_back(std::make_pair(std::string_view(ship.parts[i].id), (int)i));
        }
        std::sort(byId.begin(), byId.end(),
                  [](const std::pair<std::string_view, int> &a,
                     const std::pair<std::string_view, int> &b) {
                      /* compare() rather than != then <: it walks the bytes once. */
                      const int c = a.first.compare(b.first);
                      return c < 0 || (c == 0 && a.second < b.second);
                  });
        /* The last entry whose id equals `id`, or -1 if no part carries it. No capture:
           byId is a static, not an automatic variable. */
        auto findLast = [](const std::string &id) -> int {
            const std::string_view sv(id);
            auto hi = std::upper_bound(
                byId.begin(), byId.end(), sv,
                [](const std::string_view &v, const std::pair<std::string_view, int> &e) {
                    return v < e.first;
                });
            if(hi == byId.begin()) { return -1; }
            --hi;
            return hi->first == sv ? hi->second : -1;
        };
        sim.linkRefs.reserve(ship.fuelLinks.size());   // an upper bound; free after pass 1
        for(size_t k = 0; k < ship.fuelLinks.size(); k++) {
            const BuildShip::FuelLink &fl = ship.fuelLinks[k];
            const int f = findLast(fl.from);
            const int t = findLast(fl.to);
            if(f < 0 || t < 0) { continue; }
            LinkRef lr;
            lr.from = f;
            lr.to = t;
            sim.linkRefs.push_back(lr);
        }
    }

    auto refreshFuel = [&]() {
        sim.nGroups = buildFuelGroups(sim.parts);
        sim.rebuildMembers();
        /* The group ids just changed, so the reachability marks are meaningless --
           restart the epoch over the new group count (#536). */
        sim.reachEpoch.assign((size_t)sim.nGroups, 0);
        sim.reachTick = 0;
        /* Group ids are dense, so the reverse-hop table is indexed rather than keyed
           (#963). Grow only: resize() keeps each inner vector's capacity, which is what
           the next pass's links want, and a mid-pass kill can leave the table wider
           than the new numbering -- rows past nGroups are stale, and nothing reads them
           because every index the walk can form is below nGroups. The entries it CAN
           reach are exactly the ones cleared here. */
        if(sim.revOf.size() < (size_t)sim.nGroups) { sim.revOf.resize((size_t)sim.nGroups); }
        for(int g = 0; g < sim.nGroups; g++) { sim.revOf[(size_t)g].clear(); }
        for(size_t k = 0; k < sim.linkRefs.size(); k++) {
            if(!sim.parts[sim.linkRefs[k].from].alive || !sim.parts[sim.linkRefs[k].to].alive) { continue; }
            const int a = sim.parts[sim.linkRefs[k].from].group;
            const int b = sim.parts[sim.linkRefs[k].to].group;
            if(a < 0 || b < 0 || a == b) { continue; }
            sim.revOf[(size_t)b].push_back(a);
        }
        /* The layers follow the groups, so every cached set is stale. Bump the
           generation instead of clearing: the FuelLayers keep their capacity, which is the
           half of the win (#963). An entry that survived a counter wrap would be read
           as THIS numbering's, i.e. a silently wrong plan, hence the assert. */
        ++sim.layersEpoch;
        assert(sim.layersEpoch != 0);
        if(sim.layersOf.size() < (size_t)sim.nGroups) { sim.layersOf.resize((size_t)sim.nGroups); }
    };

    for(size_t i = 0; i < n; i++) {
        const SimPart &sp = sim.parts[i];
        if(sp.mdot <= 0.0 || sp.thrust <= 0.0) { continue; }
        SimEngine e;
        e.part = (int)i;
        e.thrust = sp.thrust;
        e.mdot = sp.mdot;
        sim.engines.push_back(e);
    }

    refreshFuel();

    for(size_t ei = 0; ei < sim.engines.size(); ei++) {
        if(sim.parts[sim.engines[ei].part].stage >= maxStage) {
            sim.engines[ei].lit = true;
            sim.litCount++;
        }
    }

    /* Walk the stage NUMBERS that exist rather than every integer between the highest
       and the lowest (#517). An index no part carries can only produce an empty drop
       set and no engine to light, so skipping it cannot change a row -- but visiting
       it was not free: the drop walk scans every part looking for a decoupler on that
       label, so the dense loop billed the SPAN between the highest and lowest labels
       instead of the number of labels. (#520 later took the per-call child-map
       rebuild out of that scan; the scan itself remains.) Measured on a 51-part
       probe, best of 5: 35 us with its labels in 1..6, 19.4 ms once one part carries
       10000, 60.8 ms once it carries 30000; walked sparse, all three cost 35-39 us.
       (Labels past kMaxStage are unreachable through the loaders, but the pass should
       not care.) Stage numbers are labels, so a ship numbered 1 and 9000 now costs
       what a ship numbered 1 and 2 costs. */
    sim.stageLabels.reserve(n);   // free after the first pass; capacity rides along (#963)
    for(size_t i = 0; i < n; i++) { sim.stageLabels.push_back(sim.parts[i].stage); }
    std::sort(sim.stageLabels.begin(), sim.stageLabels.end());
    sim.stageLabels.erase(std::unique(sim.stageLabels.begin(), sim.stageLabels.end()),
                          sim.stageLabels.end());
    /* The whole equivalence argument is that these are minStage/maxStage's only
       members, visited in the same order the dense loop visited its integers. */
    assert(sim.stageLabels.size() >= 1 && sim.stageLabels.front() == minStage
           && sim.stageLabels.back() == maxStage);

    /* Reused across the label loop, which is the only reason they are cheap: one drop
       list and one pair of burn buffers per PASS, not per label (#963) -- and on the Sim,
       which is what keeps that capacity between passes too. `dropList` is dead by the
       time the next label refills it: kill() reads it and refreshFuel() runs before the
       loop turns over. */
    std::vector<int> &drop = sim.dropList;
    std::vector<double> &burnRate = sim.burnRate;
    std::vector<double> &burnRatio = sim.burnRatio;

    for(std::vector<int>::const_reverse_iterator it = sim.stageLabels.rbegin();
        it != sim.stageLabels.rend(); ++it) {
        const int s = *it;
        if(s < maxStage) {
            for(size_t ei = 0; ei < sim.engines.size(); ei++) {
                SimEngine &e = sim.engines[ei];
                if(!e.lit && e.part >= 0 && sim.parts[e.part].alive
                   && sim.parts[e.part].stage == s) {
                    e.lit = true;
                    sim.litCount++;
                }
            }
        }

        /* Aliveness of the drop list is decided in droppedAt(), not here, and its two
           tests are not equal. Seeding on ALIVE decouplers (a decoupler an earlier
           separation took must not start a walk) is load-bearing: delete it and
           test_a_separation_does_not_relist_an_already_dead_subtree fails. Listing only
           ALIVE parts states the vector's contract -- a dead part is already a wall in
           buildFuelGroups (group -1), its links are out of revLinks, and totalMass and
           kill skip it, so no row can tell. It stays so that everything reading `drop`
           reads only parts the separation really takes. #537: this used to be a second
           filter HERE, over the returned vector, and the two copies hid each other. */
        sim.droppedAt(s, drop);

        // A stage that separates nothing and is not the last only lights
        // engines (the counter steps through it); no burn of its own.
        const bool isFinal = (s == minStage);
        if(!isFinal && drop.empty()) { continue; }

        /* Reachability for the period, stamped once per burn (#538): drainableIn used to
           re-walk every lit engine's drain layers on every call, and it is called once
           per BURN STEP. Below the lighting above, because marking first marks the
           previous period's lit set; below the no-drop `continue`, because a label that
           separates nothing never calls drainableIn -- the drainAll test short-circuits
           on !row.drops and the burn uses anyFuel(); droppedAt() (between the two) reads
           neither the lit set nor the groups. */
        sim.markReachable();

        StageRow row;
        row.stage = s;
        row.drops = !drop.empty();
        row.massStart = totalMass(sim.parts);

        // Burn until the to-be-dropped parts hold nothing the lit engines can draw
        // (asparagus: outer tanks first). An inert drop (payload separation --
        // nothing drainable in the set) would end a zero-length period and swallow
        // the survivor's burn, so fall back to draining everything reachable.
        const bool drainAll = !row.drops || sim.drainableIn(drop) <= 1e-12;
        BurnAcc burn;
        runBurn(sim, g, drainAll, drop, burn, burnRate, burnRatio);
        row.engines = sim.litRocketCount();   // the lit set THIS row opens with (#529)

        /* A separation on the LOWEST label does not end the accounting (#529). The period
           above stops when the DROP set runs dry, which is right for every label but this
           one: no later label exists to bill what survived that separation, so a decoupler
           at minStage deletes the survivor's burn from the plan -- the row keeps a number,
           just a much too small one, and its mass column agrees with the lie. 1 is the
           DEFAULT label (the VAB stage box takes 1..10000 and starts there), so the common
           trigger is a hull nobody has staged at all.
           So the last period is "separate, then burn what is still aboard", folded into the
           SAME row: rows are stage LABELS, and this hull has one. That also changes what
           massEnd means for this shape alone -- it is now the mass AFTER the separation the
           row performs, because there is no later row left to report what remained. Every
           other row keeps the old shape: massEnd pre-drop, the next row's massStart carries
           the drop.
           The follow-on period is unconditional rather than guarded on "fuel left": runBurn's
           ready() already answers that, and an unconditional kill-then-drain is what keeps
           this row's massEnd meaning one thing for every separating final label. */
        bool finalDrop = false;
        if(isFinal && !drainAll) {
            sim.kill(drop);
            refreshFuel();
            finalDrop = true;
            BurnAcc rest;
            runBurn(sim, g, true, drop, rest, burnRate, burnRatio);
            burn.dv += rest.dv;
            /* TWR of the whole stage: the worst ratio either half flies, and the best. The
               second half ignites lighter but with fewer engines, so its opening sample can
               be the stage's worst ratio -- which is the number a pilot wants under min. */
            if(rest.anyTWR) {
                if(!burn.anyTWR) {
                    burn.minTWR = rest.minTWR;
                    burn.maxTWR = rest.maxTWR;
                    burn.anyTWR = true;
                } else {
                    if(rest.minTWR < burn.minTWR) { burn.minTWR = rest.minTWR; }
                    if(rest.maxTWR > burn.maxTWR) { burn.maxTWR = rest.maxTWR; }
                }
            }
        }

        row.deltaV = burn.dv;
        row.massEnd = totalMass(sim.parts);
        row.minTWR = burn.minTWR;
        row.maxTWR = burn.maxTWR;
        rows.push_back(row);

        if(!drop.empty() && !finalDrop) {
            sim.kill(drop);
            refreshFuel();
        }
    }

    /* markReachable()'s assert cannot see a bookkeeping slip in the last period's kill(),
       since no stamp follows it. This is the only place that can. */
    assert(sim.litCount == sim.litRocketCount());

    return rows;
}

/* What a build's authored labels DO, as opposed to what computeStaging says they COST.

   Everything here is about the press counter, because a press is the only thing that ever
   reads a label. finalize() starts the counter at the highest label the parts carry and
   advanceStage() steps it down ONE integer per press (vehicle.cpp:2458, :3128), so the
   presses a ship can make are the integers hi, hi-1, ... lo+1: a decoupler or chute fires on
   the press that stands at its label (the == tests at :5573 and :3136), and an engine is
   thrusting from the moment the counter comes down to its label and stays lit (:6451, lit
   here by the same test at the max label). Nothing anywhere compares a label to a height --
   a big label is first in order, not late (pinned by test_a_stage_label_is_order_not_height)
   -- but a wide ladder is presses the player or the ascent law has to spend, which is #1018.

   Deliberately warn-only. A refuse today would mostly punish content: --fleet-report calls 7
   of 32 hulls GROUNDED and 25 of the 32 are "testship" fixtures hidden from the picker
   (#950). The one rule that could become a refuse is the root-part decoupler: it can never
   split, and game.cpp already refuses it at the press and spends the label anyway (#541), so
   the launch is not ruined -- but once #1011's window shows the part, a refuse there is the
   better answer, and it is noted rather than decided here. */
std::vector<StagingLabel> stagingLabels(const BuildShip &ship) {
    /* Fewer entries than parts, ascending by label. A sorted vector probed in place beats a
       map here: this runs on an editor edit and in an offline report, never per tick, and the
       label count is small enough that the probe is a handful of compares. Pushing the index
       rather than the part keeps the build's parent-before-child order inside a group, which
       is what makes reading a group read like the rocket. */
    std::vector<StagingLabel> labels;
    for(size_t i = 0; i < ship.parts.size(); i++) {
        const BuildPart &p = ship.parts[i];
        auto it = std::lower_bound(labels.begin(), labels.end(), p.stage,
                                   [](const StagingLabel &l, int s) { return l.stage < s; });
        if(it == labels.end() || it->stage != p.stage) {
            it = labels.insert(it, StagingLabel{});
            it->stage = p.stage;
        }
        it->parts.push_back((int)i);
        /* No null-def guard: stageRoles() answers 0 for a null def (shipdef.h:729), and a
           part with no def still rides a label -- it just answers to nothing, which is
           exactly what the group's `responds` is for. Labels themselves are 1..kMaxStage
           for anything the loader or the VAB produces (shipdef.cpp:1192-1201 refuses the
           rest), so the group edges need no clamping here. */
        it->responds |= stageRoles(p.def);
    }
    return labels;
}

std::string stagingPressNote(int lo, int hi, int stage) {
    if(hi == lo) { return "only label: press 1"; }
    if(stage > lo) {
        return "press " + std::to_string(hi - stage + 1) + " of " + std::to_string(hi - lo);
    }
    return "the floor";
}

/* The parts under each part, keyed by index. A BuildShip is one tree for anything the loader
   produces (shipdef.cpp:1023 defaults a part's parent to the entry before it) but a forest in
   principle -- graftTree/detachSubtree hand out one-root views, the VAB grafts a loaded hull
   into a bigger build, and tests build stacks directly -- so an out-of-range parent is simply
   no edge, which is what makes a parentless part a root. */
static void childIndex(const BuildShip &ship, std::vector<std::vector<int>> &children) {
    const size_t n = ship.parts.size();
    children.assign(n, {});
    for(size_t i = 0; i < n; i++) {
        if(ship.parts[i].parent >= 0 && (size_t)ship.parts[i].parent < n) {
            children[(size_t)ship.parts[i].parent].push_back((int)i);
        }
    }
}

/* Every part in parent-before-child order, entering from each part no edge points at. The
   GRAPH decides who hangs off whom, not the part index: an index pass would quietly call a
   part a root because its parent happens to sit later in the list, and the answer would then
   disagree with the drop walk (Vehicle::droppedPartsAtStage walks this same graph). Each part
   has one parent field, so each is pushed once and `order` holds each reachable part exactly
   once. A part the walk never reaches sits on a parent chain that loops -- a self-parent, or
   two parts parenting each other -- and is left out, which is how both callers spot one. The
   stack is a vector rather than recursion because a 10000-deep chain is legal here. */
static void treeOrder(const std::vector<std::vector<int>> &children, std::vector<int> &order,
                      std::vector<int> *depth) {
    const size_t n = children.size();
    std::vector<char> hasParent(n, 0);
    for(size_t i = 0; i < n; i++) {
        for(int c : children[i]) { hasParent[(size_t)c] = 1; }
    }
    if(depth != nullptr) { depth->assign(n, 0); }
    order.clear();
    order.reserve(n);
    std::vector<std::pair<int, int>> stack;
    for(size_t i = 0; i < n; i++) {
        if(hasParent[i]) { continue; }
        stack.push_back({(int)i, 0});
        while(!stack.empty()) {
            const auto [p, d] = stack.back();
            stack.pop_back();
            order.push_back(p);
            if(depth != nullptr) { (*depth)[(size_t)p] = d; }
            for(int c : children[(size_t)p]) { stack.push_back({c, d + 1}); }
        }
    }
}

/* Parts under each part, itself included, filled by reading that order back-to-front. 0 marks
   a part the order never reached. This is what a decoupler's press takes: the press drops the
   decoupler WITH its stage (Vehicle::droppedPartsAtStage), so the subtree is rooted at the
   decoupler. validateStaging's rule 3 and autoStage's refusal both ask the question the
   extract itself asks (vehicle.cpp:5803-5810), so they read one shared definition. */
static void subtreeSizes(const std::vector<int> &order,
                         const std::vector<std::vector<int>> &children,
                         std::vector<size_t> &sizes) {
    sizes.assign(children.size(), 0);
    for(size_t k = order.size(); k-- > 0;) {
        const int p = order[k];
        size_t s = 1;
        for(int c : children[(size_t)p]) { s += sizes[(size_t)c]; }
        sizes[(size_t)p] = s;
    }
}

void validateStaging(const BuildShip &ship, std::vector<std::string> &warns) {
    validateStaging(ship, stagingLabels(ship), warns);
}

void validateStaging(const BuildShip &ship, const std::vector<StagingLabel> &labels,
                     std::vector<std::string> &warns) {
    if(labels.empty()) { return; }

    char buf[512];
    const int lo = labels.front().stage, hi = labels.back().stage;
    const int presses = hi > lo ? hi - lo : 0;

    /* 1. Presses that do nothing: a label no part carries (a gap), or a label carried only by
       parts that answer to nothing. Both cost a press, and #517's "a gap costs nothing" is
       only about the preview's cost -- see #1018 for what the presses cost in flight. Listed
       to a cap so a 1..10000 ladder cannot print 9998 lines. */
    if(presses > 0) {
        /* Two shapes of dead press, and they want different cures, so they are counted apart:
           an integer NO PART carries (a gap in the ladder -- the author skipped a number), and
           a label carried only by parts that answer to nothing (the author meant something
           there and picked parts that do not respond). Runs of the first are compressed: a
           ladder numbered 2..300 has 299 of them, and what a reader needs is the shape. */
        /* Read the gaps off the labels that EXIST, not off a walk over every integer between
           lo and hi. #517 deleted that walk from computeStaging for the same reason (#695's
           10000-label ship), and it is also where the ranges went wrong: an integer walk left
           a run open across a label that DOES carry parts, so a ship labelled 1/3/7 printed
           "no part carries 2-6" and told the author to fix a label that was already right.
           Two carried labels a < b with b > a + 1 delimit exactly one run, a + 1 .. b - 1. */
        constexpr int kRuns = 4;      // gap runs listed before "..."
        int gapCount = 0, gapRuns = 0;
        std::string gapText;
        for(size_t k = 0; k + 1 < labels.size(); k++) {
            const int a = labels[k].stage, b = labels[k + 1].stage;
            if(b <= a + 1) { continue; }
            gapCount += b - a - 1;
            gapRuns++;
            if(gapRuns <= kRuns) {
                if(gapRuns > 1) { gapText += ", "; }
                if(b == a + 2) { gapText += std::to_string(a + 1); }
                else { gapText += std::to_string(a + 1) + "-" + std::to_string(b - 1); }
            }
        }
        if(gapRuns > kRuns) { gapText += ", ..."; }

        constexpr int kSilent = 4;    // silent labels listed before "..."
        int silentCount = 0;
        std::string silentText;
        for(const StagingLabel &l : labels) {
            if(l.stage <= lo || l.responds != 0u) { continue; }
            silentCount++;
            if(silentCount <= kSilent) {
                if(silentCount > 1) { silentText += ", "; }
                silentText += std::to_string(l.stage);
            }
        }
        if(silentCount > kSilent) { silentText += ", ..."; }

        const int dead = gapCount + silentCount;
        if(dead > 0) {
            std::string at;
            if(gapCount > 0) { at = "no part carries " + gapText; }
            if(silentCount > 0) {
                if(!at.empty()) { at += "; "; }
                at += "nothing answers at " + silentText;
            }
            std::snprintf(buf, sizeof buf,
                          "warn: %d of %d presses do nothing -- %s", dead, presses, at.c_str());
            warns.push_back(buf);
        }
    }

    /* 2. A DECOUPLER riding the LOWEST label of a multi-label ship. The press acts at the
       counter's current value and only THEN steps, so a ladder hi..lo spends its advancing
       presses at hi, hi-1, ... lo+1 and the label at lo waits for one more press that no longer
       moves anything. The player can still make that press; the ascent law cannot -- guidance
       presses only while stagesLeft() > 0 (guidance.cpp:336-339) and stageIsDue() refuses to say
       a stage is due there (vehicle.cpp:6373). So this is "the autopilot stops one short", not
       "can never fire": flown by hand it works, flown by the law the stage below rides along.

       A CHUTE on the floor label is deliberately NOT a warning. The chute is the part you want
       at the end of the sequence, deployChutes fires at whatever label stands (game.cpp:2640),
       and a human pressing once past the ladder to cut the chutes is normal play -- a
       capsule-and-booster ship with the chute on the capsule's label is the correct way to
       author it, and an orange verdict on correct work trains the player to ignore the window.
       A single-label ship (hi == lo) is skipped for the same reason: there the FIRST press
       stands on that label. #529 covers the preview side, where lo is the final period. */
    if(hi > lo) {
        for(const StagingLabel &l : labels) {
            if(l.stage != lo || !(l.responds & (unsigned)StageRole::Decoupler)) { continue; }
            std::snprintf(buf, sizeof buf,
                          "warn: a decoupler rides label %d, the lowest: the counter never "
                          "stands there, so it needs a press past the ladder", l.stage);
            warns.push_back(buf);
            break;
        }
    }

    /* 3. A decoupler whose side is the WHOLE ship. extractSubtreeAsShip refuses exactly that
       (droppedSet.size() == parts.size(), vehicle.cpp:5803-5810) while the press still steps
       the counter, so the label is spent and the part is inert for the rest of the flight
       (#541, e2e 541-booster-root-decoupler).

       Count the side instead of asking "does it have a parent". They agree for every loaded
       ship -- shipdef.cpp:1024 defaults a part's parent to the entry before it, so a file is
       one tree and a --stage-dump of the fleet shows exactly one root row -- but a BuildShip is
       a forest in principle (graftTree/detachSubtree hand out one-root views, the VAB grafts a
       loaded hull into a bigger build, and tests build stacks directly), and the question that
       stays true there is the one the extract asks: does THIS decoupler's side reach every
       part? Counting also keeps the checker and the press from drifting apart, which is what
       vehicle.cpp:5808's assert is for. */
    {
        const size_t n = ship.parts.size();
        std::vector<std::vector<int>> children;
        childIndex(ship, children);
        std::vector<int> order;
        treeOrder(children, order, nullptr);
        std::vector<size_t> sizes;
        subtreeSizes(order, children, sizes);
        if(order.size() != n) {
            /* A part no walk reaches is on a chain that loops, so nothing can ever separate
               it. No writer produces one today (shipdef.cpp:1021-1038 requires a parent that
               is already defined), and tests build self-parented shapes for other functions,
               so say it in words rather than assert -- the shipped binary asserts. */
            std::snprintf(buf, sizeof buf,
                          "warn: %d part(s) sit on a parent chain that loops -- nothing can "
                          "separate them", (int)(n - order.size()));
            warns.push_back(buf);
        }
        for(size_t i = 0; i < n; i++) {
            const BuildPart &p = ship.parts[i];
            if(p.def == nullptr) { continue; }
            if(!(stageRoles(p.def) & (unsigned)StageRole::Decoupler)) { continue; }
            if(sizes[i] != n) { continue; }
            std::snprintf(buf, sizeof buf,
                          "warn: decoupler '%s' at label %d would have to take the whole ship "
                          "with it -- the press is refused and the label is spent",
                          p.id.c_str(), p.stage);
            warns.push_back(buf);
        }
    }
}

StagingPlan autoStage(BuildShip &ship) {
    StagingPlan plan;
    const size_t n = ship.parts.size();
    char buf[512];
    if(n == 0) {
        plan.notes.push_back("refuse: nothing to stage -- the build has no parts");
        return plan;
    }

    std::vector<std::vector<int>> children;
    childIndex(ship, children);
    std::vector<int> order;
    std::vector<int> depth;
    treeOrder(children, order, &depth);
    std::vector<size_t> sizes;
    subtreeSizes(order, children, sizes);

    /* authored[] is the build's own numbering, kept for the two questions that compare
       against it: whether re-numbering would re-sequence the engines, and which engines the
       plan pulls apart. The rest of the function never reads a label again. */
    std::vector<int> authored(n);
    std::vector<char> isDec(n, 0), isEng(n, 0), isChute(n, 0);
    std::vector<int> decs;
    for(size_t i = 0; i < n; i++) {
        authored[i] = ship.parts[i].stage;
        const unsigned roles = stageRoles(ship.parts[i].def);
        if(roles & (unsigned)StageRole::Decoupler) { isDec[i] = 1; decs.push_back((int)i); }
        if(roles & (unsigned)StageRole::Engine) { isEng[i] = 1; }
        if(roles & (unsigned)StageRole::Chute) { isChute[i] = 1; }
    }

    if(order.size() != n) {
        /* Parts on a chain that loops were skipped by the walk, so any label here would be a
           guess about a stack that cannot be read. Said rather than asserted: the shipped
           binary asserts, and a self-parented BuildShip is a shape the test suite builds for
           other functions -- no loader produces one (shipdef.cpp:1021-1038 requires a parent
           that is already defined), but autoStage now sits behind a mouse button (#1012). */
        std::snprintf(buf, sizeof buf,
                      "refuse: %d part(s) sit on a parent chain that loops -- the stack cannot "
                      "be read, let alone numbered", (int)(n - order.size()));
        plan.notes.push_back(buf);
        return plan;
    }

    if(decs.empty()) {
        /* One label, and the checker has nothing to say about it. Assigning anything else
           would invent presses that separate nothing. */
        std::snprintf(buf, sizeof buf,
                      "refuse: no decoupler in the build -- one label is the answer, so there "
                      "is nothing to assign");
        plan.notes.push_back(buf);
        return plan;
    }

    /* A decoupler that would take every part is refused by the press itself, so no label
       could make it mean anything (#541). Numbering it would hide the shape the player has
       to fix in the stack view instead. */
    for(int d : decs) {
        if(sizes[(size_t)d] != n) { continue; }
        std::snprintf(buf, sizeof buf,
                      "refuse: '%s' would have to take the whole ship with it, so no label "
                      "would do anything -- the press itself refuses (#541)",
                      ship.parts[(size_t)d].id.c_str());
        plan.notes.push_back(buf);
        return plan;
    }

    /* Distinct decoupler depths, ascending: the deepest cut is the one the FIRST press
       makes, and the counter starts high, so depth order maps straight onto labels --
       level j from the top of the ladder is j + 2, which puts the deepest cut at
       hi = levels + 1 and the innermost at 2. The floor, 1, is therefore never a cut:
       that is #1010 rule 2 made structural rather than warned about. Decouplers at one
       depth share their label, which is how a ring is authored (#540 counts them on 4 of
       29 shipped hulls), and the labels come out contiguous, so the result can never
       trip rule 1's gap check either. */
    std::vector<int> levels;
    levels.reserve(decs.size());
    for(int d : decs) { levels.push_back(depth[d]); }
    std::sort(levels.begin(), levels.end());
    levels.erase(std::unique(levels.begin(), levels.end()), levels.end());
    const int hi = (int)levels.size() + 1;
    if(hi > kMaxStage) {
        /* Labels are 1..kMaxStage on both sides of the save (shipdef.cpp:1192-1201 throws
           for anything else), so a build with this many cuts cannot be numbered here. */
        std::snprintf(buf, sizeof buf,
                      "refuse: %d decoupler levels, more than the %d labels a ship can carry",
                      (int)levels.size(), kMaxStage);
        plan.notes.push_back(buf);
        return plan;
    }
    std::vector<int> decLabel(n, 0);
    for(size_t j = 0; j < levels.size(); j++) {
        for(int d : decs) {
            if(depth[d] == levels[j]) { decLabel[d] = (int)j + 2; }
        }
    }

    /* Every other part takes the label of the decoupler that separates it, which is the
       nearest one walking up its parent chain. Reading the walk's ORDER makes that one pass
       and correct whatever order the parts happen to be in: a parent is always earlier in
       `order` than its child, which an index pass can only assume. A part with no decoupler
       above it lands on the floor, with the hull it stays on to the end. */
    std::vector<int> label(n, 1);
    bool floorUsed = false;
    for(int p : order) {
        if(isDec[p]) { label[p] = decLabel[p]; continue; }
        const int par = ship.parts[p].parent;
        label[p] = (par >= 0 && (size_t)par < n) ? label[(size_t)par] : 1;
        if(label[p] == 1) { floorUsed = true; }
    }
    for(int d : decs) { assert(label[d] >= 2); }   // the floor never carries a cut
    if(!floorUsed) {
        /* Every root is a decoupler, so no part would carry label 1 and the LOWEST label in
           the build would be a cut -- which is what #1010 rule 2 reports, the counter never
           standing there. A plan that applies must leave the checker with nothing to say, so
           this shape is refused rather than numbered into a ship whose bottom cut needs a
           press past the ladder. */
        std::snprintf(buf, sizeof buf,
                      "refuse: every root of this build is a decoupler, so the lowest label "
                      "would carry a cut -- the counter never stands there (#1010 rule 2)");
        plan.notes.push_back(buf);
        return plan;
    }

    /* The tree says WHO separates and nothing about WHEN an engine was meant to light: an
       engine lights when the counter steps down onto its label (vehicle.cpp:6451), so its
       label IS its ignition. Where the build ordered two engines -- distinct labels -- that
       order is authored intent, and if the tree implies the opposite the two readings of the
       build disagree, so autoStage steps back rather than choose. Engines the build labelled
       the same carry no order to keep: that is the default a placed part gets, not a
       statement, and refusing on it would refuse every stack straight out of the VAB. What
       the plan does to them is reported below instead. */
    std::vector<int> engs;
    for(size_t i = 0; i < n; i++) { if(isEng[i]) { engs.push_back((int)i); } }

    /* The sharper version of the same objection, and the one a real hull walks into: an
       engine the build lights FROM THE PAD burns while it is still attached, and the tree
       says only when it FALLS, never when it starts. Measured over res/ships, heavy_asp
       lights 'engine_central' at its top label and burns the core through every separation,
       which is not a serial stage waiting its turn -- the depth rule hands it to a later
       press. So a plan that takes a launch-lit engine off launch is refused, unless the build
       was never numbered at all (every part one label), which is the case autoStage exists
       for and which carries no intent to contradict. Moving an engine the OTHER way -- onto
       launch -- is implied by the tree: a group cut on the first press has to burn before
       that press or it never burns at all. */
    int authoredHi = authored[0], authoredLo = authored[0];
    for(int s : authored) {
        authoredHi = std::max(authoredHi, s);
        authoredLo = std::min(authoredLo, s);
    }
    if(authoredHi > authoredLo) {
        for(int e : engs) {
            if(authored[e] != authoredHi || label[e] == hi) { continue; }
            std::snprintf(buf, sizeof buf,
                          "refuse: '%s' lights at launch as built and the stack would light it "
                          "at label %d -- a core that burns the whole flight is not something "
                          "the tree can see",
                          ship.parts[(size_t)e].id.c_str(), label[e]);
            plan.notes.push_back(buf);
            return plan;
        }
    }

    /* Engines, both remaining questions in one pass over the pairs. (a) A pair the build
       ORDERS -- distinct labels -- is authored intent, and if the tree implies the opposite
       then the two readings of the build disagree and the plan steps back. (b) A pair the
       build labelled TOGETHER carries no order to keep (that is the default a placed part
       gets, and refusing on it would refuse every stack straight out of the VAB), but pulling
       it apart IS a behaviour change, so it is counted and reported rather than hidden. */
    int splitA = -1, splitB = -1;
    size_t apart = 0;
    for(size_t a = 0; a < engs.size(); a++) {
        for(size_t b = a + 1; b < engs.size(); b++) {
            const int ea = engs[a], eb = engs[b];
            if(authored[ea] == authored[eb]) {
                if(label[ea] != label[eb]) {
                    if(splitA < 0) { splitA = ea; splitB = eb; }
                    apart++;
                }
                continue;
            }
            const bool authoredFirst = authored[ea] > authored[eb];
            const bool plannedFirst = label[ea] > label[eb];
            if(authoredFirst == plannedFirst) { continue; }
            if(label[ea] == label[eb]) {
                /* Worded apart because "it would light after" is the wrong sentence for a plan
                   that would light the two together. */
                std::snprintf(buf, sizeof buf,
                              "refuse: '%s' lights before '%s' as built; the stack would light "
                              "them together -- fix the stack, or number the labels by hand",
                              ship.parts[(size_t)ea].id.c_str(),
                              ship.parts[(size_t)eb].id.c_str());
            } else {
                std::snprintf(buf, sizeof buf,
                              "refuse: as built, '%s' lights before '%s'; the stack would light "
                              "it after -- fix the stack, or number the labels by hand",
                              ship.parts[(size_t)ea].id.c_str(),
                              ship.parts[(size_t)eb].id.c_str());
            }
            plan.notes.push_back(buf);
            return plan;
        }
    }

    for(size_t i = 0; i < n; i++) {
        if(label[i] == ship.parts[i].stage) { continue; }
        plan.changes.push_back({(int)i, ship.parts[i].id, ship.parts[i].stage, label[i]});
        ship.parts[i].stage = label[i];
    }
    plan.applied = true;

    /* Two caveats, said once instead of left for the player to notice mid-flight. Neither is
       a refusal: the numbering is right for the stack, and the stack is what the tree says
       it is. */
    if(apart > 0) {
        std::snprintf(buf, sizeof buf,
                      "note: %d engine pair(s) carried one label and would now light apart -- "
                      "'%s' at %d, '%s' at %d; an engine lights when the counter steps down "
                      "onto its label", (int)apart, ship.parts[(size_t)splitA].id.c_str(),
                      label[splitA], ship.parts[(size_t)splitB].id.c_str(), label[splitB]);
        plan.notes.push_back(buf);
    }
    for(size_t i = 0; i < n; i++) {
        if(!isChute[i] || label[i] > 1) { continue; }
        std::snprintf(buf, sizeof buf,
                      "note: a chute rides label %d, the floor: it deploys on a press past the "
                      "ladder", label[i]);
        plan.notes.push_back(buf);
        break;
    }
    return plan;
}

void recordStagingEdit(StagingUndo &undo, std::vector<StagingChange> changes, bool typed) {
    /* A change that did not change anything is not a step. The box guards this itself (it
       only records when the clamped value differs), so this is the belt: an entry that
       cannot move a label makes the Undo button answer "done" while the build sits still. */
    changes.erase(std::remove_if(changes.begin(), changes.end(),
                                 [](const StagingChange &c) { return c.from == c.to; }),
                  changes.end());
    if(changes.empty()) { return; }

    if(typed && changes.size() == 1 && !undo.empty()) {
        StagingEdit &top = undo.back();
        /* Only a TYPED entry above, and only one part on each side: the number box's
           arrow-repeat, or a retype of a retype. A plan entry below stays a plan entry. */
        if(top.typed && top.changes.size() == 1
           && top.changes[0].id == changes[0].id && top.changes[0].to == changes[0].from) {
            top.changes[0].to = changes[0].to;
            if(top.changes[0].from == top.changes[0].to) { undo.pop_back(); }  // back where it started
            return;
        }
    }
    undo.push_back({std::move(changes), typed});
}

size_t undoStagingEdit(BuildShip &ship, StagingUndo &undo,
                       std::vector<std::string> &notes) {
    if(undo.empty()) { return 0; }
    const StagingEdit edit = std::move(undo.back());
    undo.pop_back();
    char buf[256];
    size_t done = 0;
    for(const StagingChange &c : edit.changes) {
        /* By id, not c.part: the vector renumbers under the player whenever a part is
           placed, deleted or detached, and a stale index would put the label back on a
           different part. The recorded index is tried first because when it still names the
           same id it is exact, where a scan is a search; the scan is what makes the entry
           survive the renumbering, and it takes the LAST match so this function and
           computeStaging cannot point at different parts of the same build. Linear is fine
           here -- this runs on a button press, not a frame, and the editor's id->index memo
           is keyed to a signature that the edit itself invalidates. */
        size_t at = ship.parts.size();
        if(c.part >= 0 && (size_t)c.part < ship.parts.size()
           && ship.parts[(size_t)c.part].id == c.id) {
            at = (size_t)c.part;
        } else {
            for(size_t i = 0; i < ship.parts.size(); i++) {
                if(ship.parts[i].id == c.id) { at = i; }   // last wins
            }
        }
        if(at == ship.parts.size()) {
            std::snprintf(buf, sizeof buf,
                          "'%s' is no longer in the build -- its label went with it",
                          c.id.c_str());
            notes.push_back(buf);
            continue;
        }
        BuildPart &bp = ship.parts[at];
        if(bp.stage != c.to) {
            /* Something newer than this entry changed it (an edit the stack never saw, or a
               part re-added under the same id). Forcing `from` over it would undo the
               player's later work rather than this edit. */
            std::snprintf(buf, sizeof buf,
                          "'%s' now carries label %d, not the %d this edit left -- left as it "
                          "is", c.id.c_str(), bp.stage, c.to);
            notes.push_back(buf);
            continue;
        }
        bp.stage = c.from;
        done++;
    }
    return done;
}
