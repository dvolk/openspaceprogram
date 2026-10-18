// staging.cpp -- per-stage vacuum delta-v / TWR with fuel-link asparagus.
// See staging.h. Pure math over BuildShip.
#include "staging.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <set>

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

// Decoupler-on-stage drop: the decoupler + its child-side subtree
// (Vehicle::droppedPartsAtStage).
std::set<int> droppedParts(const std::vector<SimPart> &parts, int stage) {
    std::map<int, std::vector<int> > children;
    for(size_t i = 0; i < parts.size(); i++) {
        const int p = parts[i].parent;
        if(p >= 0 && p < (int)parts.size()) { children[p].push_back((int)i); }
    }
    std::set<int> dropped;
    for(size_t i = 0; i < parts.size(); i++) {
        if(!parts[i].alive || !parts[i].isDec || parts[i].stage != stage) { continue; }
        dropped.insert((int)i);
        std::vector<int> stack;
        std::map<int, std::vector<int> >::const_iterator it = children.find((int)i);
        if(it != children.end()) { stack = it->second; }
        while(!stack.empty()) {
            const int q = stack.back();
            stack.pop_back();
            if(dropped.count(q)) { continue; }
            dropped.insert(q);
            std::map<int, std::vector<int> >::const_iterator ct = children.find(q);
            if(ct != children.end()) {
                for(size_t k = 0; k < ct->second.size(); k++) { stack.push_back(ct->second[k]); }
            }
        }
    }
    return dropped;
}

double totalMass(const std::vector<SimPart> &parts) {
    double m = 0.0;
    for(size_t i = 0; i < parts.size(); i++) {
        if(parts[i].alive) { m += parts[i].dry + parts[i].fuel; }
    }
    return m;
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
        memberStart.assign((size_t)nGroups + 1, 0);
        for(size_t i = 0; i < parts.size(); i++) {
            const int g = parts[i].group;
            if(g >= 0) { memberStart[(size_t)g + 1]++; }
        }
        for(size_t g = 0; g + 1 < memberStart.size(); g++) {
            memberStart[g + 1] += memberStart[g];
        }
        memberOf.assign(parts.size(), 0);
        std::vector<int> cursor = memberStart;
        for(size_t i = 0; i < parts.size(); i++) {
            const int g = parts[i].group;
            if(g < 0) { continue; }
            memberOf[(size_t)cursor[(size_t)g]++] = (int)i;
        }
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

    // Propellant in `set` that a lit engine can still reach via its drain
    // layers. The "ready to drop" test. Not const: layersFor fills its cache.
    double drainableIn(const std::set<int> &set) {
        if(set.empty()) { return 0.0; }
        std::set<int> reachable;
        for(size_t ei = 0; ei < engines.size(); ei++) {
            const SimEngine &e = engines[ei];
            if(!e.lit || e.part < 0 || e.part >= (int)parts.size() || !parts[e.part].alive) {
                continue;
            }
            const int g = parts[e.part].group;
            if(g < 0) { continue; }
            const Bucket *lit = layersFor(g);
            for(size_t li = 0; li < lit->size(); li++) {
                for(size_t gi = 0; gi < (*lit)[li].size(); gi++) {
                    reachable.insert((*lit)[li][gi]);
                }
            }
        }
        double m = 0.0;
        for(std::set<int>::const_iterator it = set.begin(); it != set.end(); ++it) {
            const int pi = *it;
            if(pi < 0 || pi >= (int)parts.size() || !parts[pi].alive) { continue; }
            if(parts[pi].group < 0 || !reachable.count(parts[pi].group)) { continue; }
            m += parts[pi].fuel;
        }
        return m;
    }

    void kill(const std::set<int> &set) {
        for(std::set<int>::const_iterator it = set.begin(); it != set.end(); ++it) {
            parts[*it].alive = false;
        }
        for(size_t ei = 0; ei < engines.size(); ei++) {
            if(engines[ei].part >= 0 && !parts[engines[ei].part].alive) {
                engines[ei].lit = false;
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
             const std::set<int> &watch, BurnAcc &acc) {
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
    for(int step = 0; step < kMaxSteps && ready(); step++) {
        std::vector<double> rate;
        sim.computeRates(rate);
        double F = 0.0, mdot = 0.0;
        for(size_t ei = 0; ei < sim.engines.size(); ei++) {
            if(!sim.engines[ei].fed) { continue; }
            F += sim.engines[ei].thrust;
            mdot += sim.engines[ei].mdot;
        }
        if(mdot <= 0.0 || F <= 0.0) { break; }

        double dt = 1e300;
        for(size_t i = 0; i < rate.size(); i++) {
            if(rate[i] <= 0.0) { continue; }
            const double have = sim.parts[i].fuel;
            if(have <= 1e-12) { continue; }
            dt = std::min(dt, have / rate[i]);
        }
        if(!(dt < 1e299) || dt <= 0.0) { break; }

        double m1 = acc.m - mdot * dt;
        if(m1 < acc.m * 1e-12) {
            m1 = acc.m * 0.5;
            dt = acc.m * 0.5 / mdot;
        }
        acc.dv += (F / mdot) * std::log(acc.m / m1);
        acc.m = m1;
        // End-of-step TWR with THIS step's thrust (an engine that flames
        // out on the emptying tank still pushed through the whole step).
        acc.sampleTWR(F, acc.m, g, false);

        for(size_t i = 0; i < sim.parts.size(); i++) {
            if(rate[i] <= 0.0) { continue; }
            sim.parts[i].fuel -= rate[i] * dt;
            if(sim.parts[i].fuel < 0.0) { sim.parts[i].fuel = 0.0; }
        }
    }
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

namespace {

// FNV-1a, byte at a time so the caller need not care about endianness.
inline void sigMix(uint64_t &h, uint64_t v) {
    for(int i = 0; i < 8; i++) {
        h = (h ^ ((v >> (i * 8)) & 0xffull)) * 1099511628211ull;
    }
}

// The length goes in first so "ab","c" and "a","bc" are not one byte stream.
inline void sigMixText(uint64_t &h, const std::string &s) {
    sigMix(h, (uint64_t)s.size());
    for(size_t i = 0; i < s.size(); i++) {
        h = (h ^ (uint64_t)(unsigned char)s[i]) * 1099511628211ull;
    }
}

} // namespace

uint64_t stagingSignature(const BuildShip &ship, double g, double exhaust_scale) {
    /* Same normalization computeStaging applies, so two builds that would
       produce identical rows share one cache entry. */
    if(!(exhaust_scale > 0.0)) { exhaust_scale = 1.0; }

    uint64_t h = 1469598103934665603ull;
    sigMix(h, (uint64_t)ship.parts.size());
    sigMix(h, (uint64_t)ship.fuelLinks.size());
    for(size_t i = 0; i < ship.parts.size(); i++) {
        const BuildPart &bp = ship.parts[i];
        /* The def POINTER, not its contents: the catalog is loaded once
           (main.cpp's load_parts_catalog, moved into Game::parts) and no
           PartDef is written after load, so identity is the whole state
           computeStaging reads from it (mass, capacity, rates,
           barriers). If a parts hot-reload ever lands, this assumption breaks
           -- a freed catalog's addresses get reused by the next one and the
           cache goes silently stale. Mix a catalog epoch in then. */
        sigMix(h, (uint64_t)(uintptr_t)bp.def);
        sigMix(h, (uint64_t)(bp.parent + 1));
        sigMix(h, (uint64_t)bp.stage);
        /* The ID, not just the shape: computeStaging resolves every fuel link
           through an id -> index map (staging.cpp:452), so swapping the ids of
           two same-def parts can move a link across a fuel barrier and change
           the rows. Loading design B over design A must not show A's numbers. */
        sigMixText(h, bp.id);
    }
    for(size_t k = 0; k < ship.fuelLinks.size(); k++) {
        const BuildShip::FuelLink &fl = ship.fuelLinks[k];
        /* Only the ENDPOINTS: a link's own id is never read, and an id that
           appears in no link cannot change a row. */
        sigMixText(h, fl.from);
        sigMixText(h, fl.to);
    }
    uint64_t bits = 0;
    memcpy(&bits, &g, sizeof(bits));             sigMix(h, bits);
    memcpy(&bits, &exhaust_scale, sizeof(bits)); sigMix(h, bits);
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
        }
    }

    for(int s = maxStage; s >= minStage; s--) {
        if(s < maxStage) {
            for(size_t ei = 0; ei < sim.engines.size(); ei++) {
                SimEngine &e = sim.engines[ei];
                if(!e.lit && e.part >= 0 && sim.parts[e.part].alive
                   && sim.parts[e.part].stage == s) {
                    e.lit = true;
                }
            }
        }

        std::set<int> drop;
        {
            const std::set<int> raw = droppedParts(sim.parts, s);
            for(std::set<int>::const_iterator it = raw.begin(); it != raw.end(); ++it) {
                if(sim.parts[*it].alive) { drop.insert(*it); }
            }
        }

        // A stage that separates nothing and is not the last only lights
        // engines (the counter steps through it); no burn of its own.
        const bool isFinal = (s == minStage);
        if(!isFinal && drop.empty()) { continue; }

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

    return rows;
}
