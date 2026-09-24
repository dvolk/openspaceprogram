// staging.cpp -- per-stage vacuum delta-v / TWR with fuel-link asparagus.
// See staging.h. Pure math over BuildShip.
#include "staging.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace {

struct SimPart {
    double dry = 0.0;      // inert mass (structure + non-burnable resources)
    double fuel = 0.0;     // H2+LOX, kg (the only burnable pool)
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

void buildFuelGroups(std::vector<SimPart> &parts) {
    std::vector<std::vector<int> > adj(parts.size());
    for(size_t i = 0; i < parts.size(); i++) {
        const int p = parts[i].parent;
        if(p < 0 || p >= (int)parts.size()) { continue; }
        adj[i].push_back(p);
        adj[p].push_back((int)i);
    }
    for(size_t i = 0; i < parts.size(); i++) { parts[i].group = -1; }
    int next = 0;
    for(size_t i = 0; i < parts.size(); i++) {
        if(parts[i].isBarrier || parts[i].group != -1 || !parts[i].alive) { continue; }
        const int g = next++;
        std::vector<int> stack;
        stack.push_back((int)i);
        parts[i].group = g;
        while(!stack.empty()) {
            const int q = stack.back();
            stack.pop_back();
            for(size_t k = 0; k < adj[q].size(); k++) {
                const int r = adj[q][k];
                if(parts[r].isBarrier || parts[r].group != -1 || !parts[r].alive) { continue; }
                parts[r].group = g;
                stack.push_back(r);
            }
        }
    }
}

// Group buckets by reverse-hop distance, furthest first -- the same
// order as Vehicle::fuelDrainLayers (asparagus empties outer groups first).
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

    void rebuildLayers() {
        layersOf.clear();
        for(size_t i = 0; i < parts.size(); i++) {
            const int g = parts[i].group;
            if(g < 0 || layersOf.count(g)) { continue; }
            layersOf[g] = drainLayersFor(revLinks, g);
        }
    }

    // Per-tank drain rates (kg/s) from every lit engine that still has
    // fuel in its layers. Each engine fills its mdot from the furthest
    // non-empty bucket, pro-rata by remaining fuel (the same distribution
    // consumeResourceMass applies within a call).
    void computeRates(std::vector<double> &rate) {
        rate.assign(parts.size(), 0.0);
        for(size_t ei = 0; ei < engines.size(); ei++) {
            SimEngine &e = engines[ei];
            e.fed = false;
            if(!e.lit || e.mdot <= 0.0) { continue; }
            if(e.part < 0 || e.part >= (int)parts.size() || !parts[e.part].alive) { continue; }
            const int g = parts[e.part].group;
            if(g < 0) { continue; }
            std::map<int, Bucket>::const_iterator lit = layersOf.find(g);
            if(lit == layersOf.end()) { continue; }
            double need = e.mdot;
            for(size_t li = 0; li < lit->second.size() && need > 0.0; li++) {
                const std::vector<int> &bucket = lit->second[li];
                double layerFuel = 0.0;
                for(size_t gi = 0; gi < bucket.size(); gi++) {
                    for(size_t pi = 0; pi < parts.size(); pi++) {
                        if(parts[pi].alive && parts[pi].group == bucket[gi]) {
                            layerFuel += parts[pi].fuel;
                        }
                    }
                }
                if(layerFuel <= 0.0) { continue; }
                for(size_t gi = 0; gi < bucket.size(); gi++) {
                    for(size_t pi = 0; pi < parts.size(); pi++) {
                        if(!parts[pi].alive || parts[pi].group != bucket[gi]) { continue; }
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
    // layers. The "ready to drop" test: once this hits 0, every atom those
    // engines would take from the set is gone.
    double drainableIn(const std::set<int> &set) const {
        if(set.empty()) { return 0.0; }
        std::set<int> reachable;
        for(size_t ei = 0; ei < engines.size(); ei++) {
            const SimEngine &e = engines[ei];
            if(!e.lit || e.part < 0 || e.part >= (int)parts.size() || !parts[e.part].alive) {
                continue;
            }
            std::map<int, Bucket>::const_iterator lit = layersOf.find(parts[e.part].group);
            if(lit == layersOf.end()) { continue; }
            for(size_t li = 0; li < lit->second.size(); li++) {
                for(size_t gi = 0; gi < lit->second[li].size(); gi++) {
                    reachable.insert(lit->second[li][gi]);
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
// -- until no lit engine can draw more. Events are tank-empty boundaries
// (the drop condition is itself one).
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

double partPropellantMass(const PartDef &def) {
    // Only H2+LOX is burned in vacuum. JetFuel / hydrazine / life support
    // ride along as inert mass; EC is Wh, not kg.
    return (double)def.capacity[(int)ResourceType::Hydrogen]
         + (double)def.capacity[(int)ResourceType::LOX];
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

    for(size_t i = 0; i < n; i++) {
        const BuildPart &bp = ship.parts[i];
        SimPart &sp = sim.parts[i];
        idIndex[bp.id] = (int)i;
        sp.parent = (bp.parent >= 0 && bp.parent < (int)n) ? bp.parent : -1;
        sp.stage = bp.stage;
        sp.isBarrier = (bp.def != nullptr) && (bp.def->decoupler || bp.def->fuel_barrier);
        sp.isDec = (bp.def != nullptr) && bp.def->decoupler;
        if(bp.def != nullptr) {
            sp.dry = std::max(0.0, partDryMass(*bp.def));
            sp.fuel = partPropellantMass(*bp.def);
            // Rocket only (fuel_rate + ve, not a jet). Jets need air, so
            // they contribute no vacuum thrust and burn no vacuum fuel.
            if(bp.def->fuel_rate > 0.0 && bp.def->exhaust_velocity > 0.0
               && !bp.def->jet) {
                // Scale thrust only (not mdot): ve_eff = F/mdot scales, so
                // delta-v and TWR track the difficulty knob like flight.
                sp.thrust = bp.def->fullThrust() * exhaust_scale;
                sp.mdot = 2.0 * bp.def->fuel_rate;
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
        buildFuelGroups(sim.parts);
        sim.revLinks.clear();
        for(size_t k = 0; k < links.size(); k++) {
            if(!sim.parts[links[k].from].alive || !sim.parts[links[k].to].alive) { continue; }
            const int a = sim.parts[links[k].from].group;
            const int b = sim.parts[links[k].to].group;
            if(a < 0 || b < 0 || a == b) { continue; }
            sim.revLinks[b].push_back(a);
        }
        sim.rebuildLayers();
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

        // Burn until the to-be-dropped parts are empty of anything the
        // lit engines can draw (asparagus: outer tanks first). An inert
        // drop (payload sep -- nothing drainable in the set) would end a
        // zero-length period and swallow the survivor's burn, so fall
        // back to draining everything the engines can reach.
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
