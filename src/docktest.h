// docktest.h -- the --dock-test pair builder (see docktest.cpp). Builds a
// nose-to-nose docking pair straight from the parts catalog so the
// docking check + the undock round trip have a known, aligned geometry.
// Each ship carries the same seven parts (port to engine):
// docking_port_r1, capsule, rcs_r1, mono_tank_r1, reaction_wheel, tank_r1h3,
// engine -- the probe port-first, the station its mirror (port-last). The
// probe is the ACTIVE ship (it is returned first and main() makes it the
// player's); its engine thrusts along its +Z, toward the station.
#pragma once

#include <string>

#include "shader.h"   // Shader
#include "vehicle.h"  // Vehicle, ScenarioDef, PartDef, PartsCatalog, TerrainBody

struct System;

/* The result of building the --dock-test pair: both ships (the caller owns
   them and pushes them into the fleet) + the starting port-face gap. */
struct DockTestShips {
    Vehicle *probe;    // the active ship (root = its docking port)
    Vehicle *station;  // the target (root = its engine, port at the tail)
    double gap;        // port-face gap at start (m)
};

/* Build the pair for the given --dock-test mode:
     near      probe's port face 1.0 m from the station's (inside the
               kDockCapture window: it docks on the first live tick, no burn)
     approach  probe's port face 2.0 m out (outside capture: the player
               burns prograde to close the last 0.5 m)
   Both start co-moving on the same circular orbit (the station is placed
   by spawn_vehicle; the probe is placed in front of it, prograde, at the
   mode's gap). Throws std::runtime_error if the parts it needs are missing
   from the catalog. */
DockTestShips build_dock_test_ships(const std::string &mode,
                                    bool scenario_given,
                                    const std::string &scenario,
                                    const PartsCatalog &part_catalog,
                                    TerrainBody *home,
                                    TerrainBody *sun,
                                    Shader *partsshader,
                                    System &sys);
