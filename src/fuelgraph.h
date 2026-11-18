// fuelgraph.h -- the ONE drain-layer walk over a fuel-group graph: which groups feed
// an engine's group, bucketed by hop distance, furthest first.
//
// Both halves of the game ask this question. Flight drains a live hull through it
// (Vehicle::fuelDrainLayers -> consumeResourceMass), and the staging preview prices a
// burn through it (staging.cpp). They used to be two ~50-line copies of the same
// ordering rule, which is how two views start disagreeing about what a tank is
// reachable from (#965; #963 had already made the preview's copy flat).

#pragma once

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
