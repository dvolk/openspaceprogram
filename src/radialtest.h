// radialtest.h -- the --radial-test spin-test ship builder (see radialtest.cpp).
#pragma once

#include <string>

#include "shader.h"   // Shader
#include "vehicle.h"  // Vehicle, ScenarioDef, PartDef, PartsCatalog, TerrainBody

/* A built --radial-test ship (the caller owns it) plus the starting scenario
   to register it with. The builder places it, so there is no slot. */
struct RadialTestShip {
    Vehicle *v;
    const ScenarioDef *sc;
};

/* Build a passive-tank spin-test ship for a --radial-test mode ("parallel",
   "stacked", "stacks", "parstacks"). It starts in "rot-orbit": no pad contact
   or terrain noise in the spin measurement. Throws if the two tank parts are
   missing from the catalog. */
RadialTestShip build_radial_test_ship(const std::string &mode,
                                      const PartsCatalog &part_catalog,
                                      TerrainBody *home,
                                      TerrainBody *sun,
                                      Shader *partsshader);
