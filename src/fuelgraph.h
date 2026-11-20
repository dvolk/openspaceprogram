// fuelgraph.h -- the ONE drain-layer walk over a fuel-group graph: which groups feed
// an engine's group, bucketed by hop distance, furthest first.
//
// Both halves of the game ask this question. Flight drains a live hull through it
// (Vehicle::fuelDrainLayers -> consumeResourceMass), and the staging preview prices a
// burn through it (staging.cpp). They used to be two ~50-line copies of the same
// ordering rule, which is how two views start disagreeing about what a tank is
// reachable from (#965; #963 had already made the preview's copy flat). It also holds
// the one CSR builder the tree has (#534, and #984 made the flight drain use it too).

#pragma once

#include <cassert>
#include <vector>

/* One group's drain layers, flat: layer li owns groupOf[layerStart[li] ..
   layerStart[li+1]), furthest layer first. It used to be a vector of vectors, which
   pays an allocation per layer on top of the outer one, for every cached group in
   every staging pass (#963). The layer COUNT stays data rather than structure, so the
   cache's per-group cost is two buffers whatever the shape of the ship. */
struct FuelLayers {
    std::vector<int> groupOf;
    std::vector<int> layerStart;   // nLayers + 1 entries, [0] = 0
};

/* Fill `out` with the layers reachable from `group`.
   `rev[Y] = { X : X feeds Y }`, indexed by group id, at least nGroups long. Group ids
   are dense from 0 -- partitionFuelGroups' contract on both sides.
   `out` is the caller's (a cache slot) and is fully overwritten. */
void drainLayersFor(const std::vector<std::vector<int> > &rev, int group,
                    int nGroups, FuelLayers &out);

/* The one bucketed index (a CSR) in the tree: bucket b owns
   items[start[b] .. start[b+1]), and `key(i)` says which bucket item i belongs to, or -1
   to leave it out. Both passes go through the SAME key, which is the whole point of the
   helper: a histogram that accepts an item the fill rejects (or the reverse) writes
   `items` past its end, and nothing else in the caller would notice. #534 folded the
   staging preview's two hand-rolled copies (group members, part children) into this one;
   #984 folds the flight drain's copy in as well, so the rule -- and its asserts -- live
   in one place instead of three. `items` is sized to nItems rather than to the member
   count, so a two-pass disagreement can mislay an entry but cannot write outside it. */
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
       each call site passes its own key closure, so they already have separate cursors.
       That is not what makes nesting safe -- a caller that reused one key type and nested
       inside another buildBuckets call would share this buffer and corrupt the outer
       fill. The body calls nothing but key(), asserts and vector ops, so the only way to
       nest it is a key() that calls buildBuckets, which would need its own cursor. */
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
    // re-running key() over the result). Every call site passes a pure key -- a part's
    // group id, a clamped parent -- so this is a tripwire for a future caller rather
    // than a check the current ones could fail.
    for(int b = 0; b < nBuckets; b++) {
        assert(cursor[(size_t)b] == start[(size_t)b + 1]);
    }
}
