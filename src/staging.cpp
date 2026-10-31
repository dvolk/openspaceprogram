// staging.cpp -- per-stage vacuum delta-v / TWR with fuel-link asparagus.
// See staging.h. Pure math over BuildShip.
#include "staging.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <map>

#include "hash.h"   // the staging cache key's FNV-1a mixer

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

typedef std::vector<std::vector<int> > Bucket;

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
    std::vector<int> parent(parts.size());
    std::vector<bool> wall(parts.size());
    for(size_t i = 0; i < parts.size(); i++) {
        parent[i] = parts[i].parent;
        wall[i] = parts[i].isBarrier || !parts[i].alive;
    }
    std::vector<int> group;
    const int n = partitionFuelGroups(parent, wall, group);
    for(size_t i = 0; i < parts.size(); i++) { parts[i].group = group[i]; }
    return n;
}

// Group buckets by reverse-hop distance, furthest first: outer groups drain
// first, as in Vehicle::fuelDrainLayers.
Bucket drainLayersFor(const std::map<int, std::vector<int> > &rev, int group) {
    std::map<int, int> dist;
    dist[group] = 0;
    std::vector<int> queue;
    queue.push_back(group);
    for(size_t qi = 0; qi < queue.size(); qi++) {
        const int u = queue[qi];
        std::map<int, std::vector<int> >::const_iterator it = rev.find(u);
        if(it == rev.end()) { continue; }
        for(size_t i = 0; i < it->second.size(); i++) {
            const int v = it->second[i];
            if(dist.count(v)) { continue; }
            dist[v] = dist[u] + 1;
            queue.push_back(v);
        }
    }
    std::map<int, std::vector<int> > byDist;
    for(std::map<int, int>::const_iterator it = dist.begin(); it != dist.end(); ++it) {
        byDist[it->second].push_back(it->first);
    }
    Bucket layers;
    for(std::map<int, std::vector<int> >::const_reverse_iterator it = byDist.rbegin();
        it != byDist.rend(); ++it) {
        layers.push_back(it->second);
    }
    return layers;
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
    std::vector<int> cursor = start;
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
    std::map<int, std::vector<int> > revLinks;
    std::map<int, Bucket> layersOf;
    /* group -> its parts, flat: group g owns memberOf[memberStart[g] ..
       memberStart[g+1]). Built once per refreshFuel (#293): the drain loop used
       to rescan the whole ship for every group in every layer, which made a
       240-group ship 5.6x the cost of a one-group ship of the same part count.
       Alive parts only (a dead part has group -1), so the drain loop needs no
       liveness test. */
    int nGroups = 0;
    std::vector<int> memberStart;
    std::vector<int> memberOf;

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
       order, so keeping the order keeps the rows bit-identical (#533). */
    std::vector<int> droppedAt(int stage) {
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
        std::vector<int> dropped;
        std::vector<int> stack;   // one allocation per walk: the drain below empties it
        for(size_t i = 0; i < parts.size(); i++) {
            if(!parts[i].alive || !parts[i].isDec || parts[i].stage != stage) { continue; }
            if(seenEpoch[i] != seenTick) { seenEpoch[i] = seenTick; dropped.push_back((int)i); }
            for(int k = childStart[i]; k < childStart[i + 1]; k++) { stack.push_back(childOf[k]); }
            while(!stack.empty()) {
                const int q = stack.back();
                stack.pop_back();
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
                    stack.push_back(childOf[k]);
                }
            }
        }
        /* Ascending, i.e. the iteration order the std::set this used to return. It is
           what drainableIn sums fuel in; the 29 ships in res/ships read identically
           either way (measured), so this is insurance for a big drop set with inexact
           sums, not a live difference. */
        std::sort(dropped.begin(), dropped.end());
        return dropped;
    }

    /* Drain layers for one group, computed on first use (#293). Only a group
       that holds a lit engine is ever asked for, and computing them for every
       group -- a BFS each, over a graph as big as the group set -- was the bulk
       of a deep asparagus build's cost once the drain loop stopped rescanning.
       layersOf must stay a node-stable container: the drain loop holds the
       returned pointer across a whole layer walk while later engines insert. */
    const Bucket *layersFor(int g) {
        std::map<int, Bucket>::const_iterator it = layersOf.find(g);
        if(it != layersOf.end()) { return &it->second; }
        Bucket b = drainLayersFor(revLinks, g);
        return &layersOf.insert(it, std::make_pair(g, std::move(b)))->second;
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
            const Bucket *lit = layersFor(g);
            double need = e.mdot;
            for(size_t li = 0; li < lit->size() && need > 0.0; li++) {
                const std::vector<int> &bucket = (*lit)[li];
                double layerFuel = 0.0;
                for(size_t gi = 0; gi < bucket.size(); gi++) {
                    const int b = bucket[gi];
                    // Every bucket entry is a revLinks key or value: a real group id.
                    assert(b >= 0 && b < nGroups);
                    for(size_t k = (size_t)memberStart[(size_t)b];
                        k < (size_t)memberStart[(size_t)b + 1]; k++) {
                        layerFuel += parts[(size_t)memberOf[k]].fuel;
                    }
                }
                if(layerFuel <= 0.0) { continue; }
                for(size_t gi = 0; gi < bucket.size(); gi++) {
                    const int b = bucket[gi];
                    assert(b >= 0 && b < nGroups);
                    for(size_t k = (size_t)memberStart[(size_t)b];
                        k < (size_t)memberStart[(size_t)b + 1]; k++) {
                        const size_t pi = (size_t)memberOf[k];
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

    double fedThrust() {
        std::vector<double> tmp;
        computeRates(tmp);
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
            const Bucket *layers = layersFor(g);
            for(size_t li = 0; li < layers->size(); li++) {
                for(size_t gi = 0; gi < (*layers)[li].size(); gi++) {
                    const int rg = (*layers)[li][gi];   // a GROUP, not a part
                    assert(rg >= 0 && rg < (int)reachEpoch.size());
                    reachEpoch[(size_t)rg] = reachTick;
                }
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
void runBurn(Sim &sim, double g, bool drainAll,
             const std::vector<int> &watch, BurnAcc &acc) {
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
    std::vector<double> rate;    // this plan's per-part drain rates
    std::vector<double> ratio;   // part fuel / part rate = when that tank empties
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

        double m1 = acc.m - mdot * dt;
        /* One plan, one mass: the charge is exactly the fuel this plan billed (the
           rates sum to the fed mdot, and each rate[i]*dt is one tank's own fuel), so
           what is left is every part's dry mass plus the fuel still in the tanks.
           Note what keeps that positive: the DRY mass. A part with mass 0 -- which a
           hand-written parts.json can still mint, because "fuel_link": true skips the
           mass block at shipdef.cpp:330 (#530) -- leaves rounding as the only thing
           above zero, so this assert is where such a part gets reported. */
        assert(m1 > 0.0);
        acc.dv += (F / mdot) * std::log(acc.m / m1);
        acc.m = m1;
        // End-of-step TWR with THIS step's thrust (an engine that flames
        // out on the emptying tank still pushed through the whole step).
        acc.sampleTWR(F, acc.m, g, false);

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
        // The two halves of the comment above, pinned: a step spends what it bills.
        // Only fp slack (sum(rate) against the fed mdot is a rounding sum).
        assert(std::fabs(drained - mdot * dt) <= 1e-9 * (mdot * dt));
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
        /* The ID, not just the shape: computeStaging resolves every fuel link
           through an id -> index map (staging.cpp:452), so swapping the ids of
           two same-def parts can move a link across a fuel barrier and change
           the rows. Loading design B over design A must not show A's numbers. */
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
                                     double exhaust_scale) {
    std::vector<StageRow> rows;
    if(ship.parts.empty()) { return rows; }
    if(!(exhaust_scale > 0.0)) { exhaust_scale = 1.0; }

    const size_t n = ship.parts.size();
    Sim sim;
    sim.parts.resize(n);
    std::map<std::string, int> idIndex;
    int minStage = ship.parts[0].stage;
    int maxStage = minStage;

    // The resources the ship's ROCKET engines draw (jets excluded: they need air,
    // so no vacuum delta-v). A tank resource no engine burns is inert mass.
    // LIMITATION: a ship-wide UNION, and a part's propellant is one pool -- a ship
    // MIXING chemical (H2+LOX) and nuclear-thermal (H2 only) engines credits every
    // tank's LOX as burnable even where only the H2 engine can draw it. Exact for
    // a homogeneous engine set; mixed ships need a per-fuel-group mask (#41).
    bool burn[(int)ResourceType::Num];
    for(int r = 0; r < (int)ResourceType::Num; r++) { burn[r] = false; }
    for(size_t i = 0; i < n; i++) {
        const PartDef *def = ship.parts[i].def;
        if(def == nullptr || def->jet) { continue; }
        if(!(def->totalPropellantRate() > 0.0 && def->exhaust_velocity > 0.0)) { continue; }
        for(int r = 0; r < (int)ResourceType::Num; r++) {
            if(def->propellant_rate[r] > 0.0) { burn[r] = true; }
        }
    }

    for(size_t i = 0; i < n; i++) {
        const BuildPart &bp = ship.parts[i];
        SimPart &sp = sim.parts[i];
        idIndex[bp.id] = (int)i;
        /* BuildShip::recomputePoses asserts every parent is an earlier part, so
           the snapshot cannot be torn here (partitionFuelGroups asserts it too). */
        sp.parent = bp.parent;
        sp.stage = bp.stage;
        /* The same wall rule flight uses (PartDef::isFuelBarrier). The parser
           forces fuel_barrier on every decoupler and docking port (shipdef.cpp),
           so spelling out `decoupler` here is a second rule free to drift (#292). */
        sp.isBarrier = (bp.def != nullptr) && bp.def->isFuelBarrier();
        sp.isDec = (bp.def != nullptr) && bp.def->decoupler;
        if(bp.def != nullptr) {
            sp.dry = std::max(0.0, partDryMass(*bp.def, burn));
            sp.fuel = partPropellantMass(*bp.def, burn);
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

    struct LinkRef { int from, to; };
    std::vector<LinkRef> links;
    for(size_t k = 0; k < ship.fuelLinks.size(); k++) {
        const BuildShip::FuelLink &fl = ship.fuelLinks[k];
        std::map<std::string, int>::const_iterator f = idIndex.find(fl.from);
        std::map<std::string, int>::const_iterator t = idIndex.find(fl.to);
        if(f == idIndex.end() || t == idIndex.end()) { continue; }
        LinkRef lr;
        lr.from = f->second;
        lr.to = t->second;
        links.push_back(lr);
    }

    auto refreshFuel = [&]() {
        sim.nGroups = buildFuelGroups(sim.parts);
        sim.rebuildMembers();
        /* The group ids just changed, so the reachability marks are meaningless --
           restart the epoch over the new group count (#536). */
        sim.reachEpoch.assign((size_t)sim.nGroups, 0);
        sim.reachTick = 0;
        sim.revLinks.clear();
        for(size_t k = 0; k < links.size(); k++) {
            if(!sim.parts[links[k].from].alive || !sim.parts[links[k].to].alive) { continue; }
            const int a = sim.parts[links[k].from].group;
            const int b = sim.parts[links[k].to].group;
            if(a < 0 || b < 0 || a == b) { continue; }
            sim.revLinks[b].push_back(a);
        }
        sim.layersOf.clear();   // the layers follow the groups, so they are stale
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
    std::vector<int> walk;
    walk.reserve(n);
    for(size_t i = 0; i < n; i++) { walk.push_back(sim.parts[i].stage); }
    std::sort(walk.begin(), walk.end());
    walk.erase(std::unique(walk.begin(), walk.end()), walk.end());
    /* The whole equivalence argument is that these are minStage/maxStage's only
       members, visited in the same order the dense loop visited its integers. */
    assert(walk.size() >= 1 && walk.front() == minStage && walk.back() == maxStage);

    for(std::vector<int>::const_reverse_iterator it = walk.rbegin();
        it != walk.rend(); ++it) {
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
        const std::vector<int> drop = sim.droppedAt(s);

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
        runBurn(sim, g, drainAll, drop, burn);

        row.deltaV = burn.dv;
        row.massEnd = totalMass(sim.parts);
        row.minTWR = burn.minTWR;
        row.maxTWR = burn.maxTWR;
        row.engines = sim.litRocketCount();
        rows.push_back(row);

        if(!drop.empty()) {
            sim.kill(drop);
            refreshFuel();
        }
    }

    /* markReachable()'s assert cannot see a bookkeeping slip in the last period's kill(),
       since no stamp follows it. This is the only place that can. */
    assert(sim.litCount == sim.litRocketCount());

    return rows;
}
