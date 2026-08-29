// radialtest.h -- the --radial-test spin-test ship builder (see
// radialtest.cpp). Builds a passive-tank test ship straight from the parts
// catalog so the spin diagnostics have a known, thruster-free geometry.
#pragma once

#include <string>

#include "shader.h"   // Shader
#include "vehicle.h"  // Vehicle, ScenarioDef, PartDef, PartsCatalog, TerrainBody

/* The result of building one --radial-test ship: the ship itself (the
   caller owns it and pushes it into its fleet) plus the bookkeeping main()
   needs to slot it -- the resolved starting scenario and its pad slot
   (always 0: the radial-test ship is the only one on its body+scenario). */
struct RadialTestShip {
    Vehicle *v;
    const ScenarioDef *sc;
    int slot;
};

/* Build one passive-tank spin-test ship for the given --radial-test mode
   ("radial", "parallel", "stacked", "stacks", "parstacks"). If
   scenario_given is false the ship starts in a "rot-orbit" scenario (no pad
   contact, no terrain noise in the spin measurement); otherwise it honors
   the caller's scenario. Throws std::runtime_error if the two tank parts it
   needs are missing from the catalog. */
RadialTestShip build_radial_test_ship(const std::string &mode,
                                      bool scenario_given,
                                      const std::string &scenario,
                                      const PartsCatalog &part_catalog,
                                      TerrainBody *home,
                                      TerrainBody *sun,
                                      Shader *partsshader);
