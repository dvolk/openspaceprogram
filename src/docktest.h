// docktest.h -- the --dock-test pair builder (see docktest.cpp): a nose-to-nose
// pair from the parts catalog; the probe is the ACTIVE ship.
#pragma once

#include <string>

#include "shader.h"   // Shader
#include "vehicle.h"  // Vehicle, ScenarioDef, PartDef, PartsCatalog, TerrainBody

struct System;

/* The built --dock-test pair (the caller owns both ships). */
struct DockTestShips {
    Vehicle *probe;    // the active ship (root = its docking port)
    Vehicle *station;  // the target (root = its engine, port at the tail)
};

/* Build the pair for the given --dock-test mode (near | approach | misaligned |
   stale-arm | stale-target). Both start co-moving on the same circular orbit. Throws
   if parts are missing. stale-arm is misaligned's geometry plus a second port hung
   under a stage-1 decoupler, and THAT port is the armed one, so one stage press makes
   the arm stale while the target stays (#1040). stale-target hangs the same
   decoupler-and-port stack under the STATION and targets that far port, so a press
   while the station is the active ship makes the TARGET stale -- which is the route
   Vehicle::dockSituation()'s TargetGone branch needs, and it is a player's route:
   Ship List -> the station -> SPACE -> back to the probe (#1062). */
DockTestShips build_dock_test_ships(const std::string &mode,
                                    const PartsCatalog &part_catalog,
                                    TerrainBody *home,
                                    TerrainBody *sun,
                                    Shader *partsshader,
                                    System &sys);
