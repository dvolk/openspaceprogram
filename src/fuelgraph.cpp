// fuelgraph.cpp -- the drain-layer walk shared by the live fuel drain and the staging
// preview (#965). See fuelgraph.h.

#include "fuelgraph.h"

#include <algorithm>
#include <cassert>
#include <vector>

/* Group buckets by reverse-hop distance, furthest first: outer groups drain first.

   The group ids are dense from 0 (partitionFuelGroups' contract), so the hop table is
   a vector indexed by group rather than a std::map keyed by it, and the layers are a
   count-then-fill CSR rather than a map of vectors. Both used to bill a tree node per
   reachable group and per layer: 35,280 allocations over an ascent's 755 staging
   passes -- ~47 a pass, on a hull with three fuel groups (#963). A 1,204-part /
   240-group hull went 124,614 allocations per walk to 3,123.

   Listing each layer in ASCENDING group id is what the map-of-vectors gave, and
   computeRates sums a layer's fuel in that order. The order is insurance rather than a
   live difference: it can only reach that fp sum, and filling in BFS order instead
   changed no row on any shipped hull and failed no check (measured, #963's review). It
   costs a sort of the reachable set, which is never larger than the walk already did. */
void drainLayersFor(const std::vector<std::vector<int> > &rev, int group,
                    int nGroups, FuelLayers &out) {
    /* These three ride between calls (#963): this runs once per group that holds a lit
       engine, and every one is overwritten before anything reads it. The body calls
       nothing but vector ops and asserts, so there is no reentrant caller to worry
       about; one that wanted the hop table WHILE this ran would need its own. */
    static thread_local std::vector<int> dist;   // group -> hops, -1 = not reachable
    static thread_local std::vector<int> queue;  // reachable groups, ascending on use
    static thread_local std::vector<int> cnt;    // per layer: count, then write cursor
    assert(group >= 0 && group < nGroups);
    assert(rev.size() >= (size_t)nGroups);   // the caller grows it to the numbering
    /* The exit below leaves dist ALL -1 again, so there is no clear pass here: the walk
       touches only entries it marked, which keeps this function O(reachable) rather
       than O(nGroups) -- a ship with many groups and short reach (a ring of unlinked
       boosters) asks for one layer set per group, and scanning the whole group set for
       each was 40% slower than the map it replaced. A grown tail starts unseen, or 0
       would leak into the next call on a bigger ship reading as "this group IS the one
       with the lit engine". */
    const size_t had = dist.size();
    if(had < (size_t)nGroups) {
        dist.resize((size_t)nGroups);
        std::fill(dist.begin() + had, dist.end(), -1);
    }
    queue.clear();
    dist[group] = 0;
    queue.push_back(group);
    int maxDist = 0;
    for(size_t qi = 0; qi < queue.size(); qi++) {
        const int u = queue[qi];
        const std::vector<int> &feed = rev[(size_t)u];   // empty unless something feeds u
        for(size_t i = 0; i < feed.size(); i++) {
            const int v = feed[i];
            assert(v >= 0 && v < nGroups);
            if(dist[v] != -1) { continue; }
            dist[v] = dist[u] + 1;
            if(dist[v] > maxDist) { maxDist = dist[v]; }
            queue.push_back(v);
        }
    }
    /* Every hop count from 0 to maxDist has a group (a group at distance d was reached
       through one at d - 1), so there are exactly maxDist + 1 layers and none is empty.
       Layer li carries the groups maxDist - li hops out, so li 0 is the outermost. */
    const size_t nLayers = (size_t)maxDist + 1;
    std::sort(queue.begin(), queue.end());
    cnt.assign(nLayers, 0);
    for(size_t k = 0; k < queue.size(); k++) { cnt[(size_t)(maxDist - dist[queue[k]])]++; }
    out.layerStart.resize(nLayers + 1);
    out.layerStart[0] = 0;
    for(size_t li = 0; li < nLayers; li++) {
        out.layerStart[li + 1] = out.layerStart[li] + cnt[li];
    }
    out.groupOf.resize(queue.size());
    assert(out.layerStart[nLayers] == (int)queue.size());   // each reachable group once
    for(size_t li = 0; li < nLayers; li++) { cnt[li] = out.layerStart[li]; }
    for(size_t k = 0; k < queue.size(); k++) {
        const int g = queue[k];                       // ascending: the layer's order
        const size_t li = (size_t)(maxDist - dist[g]);
        out.groupOf[(size_t)cnt[li]++] = g;
    }
    for(size_t k = 0; k < queue.size(); k++) { dist[queue[k]] = -1; }
}
