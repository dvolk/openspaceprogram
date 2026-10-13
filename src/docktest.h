// docktest.h -- the --dock-test pair builder (see docktest.cpp): a nose-to-nose
// pair from the parts catalog; the probe is the ACTIVE ship.
#pragma once

#include <string>

#include "shader.h"   // Shader
#include "vehicle.h"  // Vehicle, ScenarioDef, PartDef, PartsCatalog, TerrainBody

struct System;

/* The built --dock-test pair (the caller owns both ships) plus the starting
   port-face gap. */
struct DockTestShips {
    Vehicle *probe;    // the active ship (root = its docking port)
    Vehicle *station;  // the target (root = its engine, port at the tail)
    double gap;        // port-face gap at start (m)
};

/* Build the pair for the given --dock-test mode (near | approach).
   Both start co-moving on the same circular orbit. Throws if parts are missing. */
DockTestShips build_dock_test_ships(const std::string &mode,
                                    const PartsCatalog &part_catalog,
                                    TerrainBody *home,
                                    TerrainBody *sun,
                                    Shader *partsshader,
                                    System &sys);
