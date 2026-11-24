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

/* Build the pair for the given --dock-test mode (near | approach | tilt | corridor | spin |
   misaligned | stale-arm | stale-target). Both start co-moving on the same circular
   orbit; `approach` also gets a scripted closing rate down the port axis, because a
   wall-clock engine press cannot land under #1152's 0.15 m/s capture cap. `tilt` is
   approach with the probe's bore 4 deg off the joining line -- still inside the cone, so it
   mates, and it is the only fixture whose dock has an angle for #1153's snap to take up.
   `corridor` is a
   wide (8 m) aligned pair at rest for a terminal approach flown by hand, and `spin` is the
   same pair with the probe rolling 4 deg/s about its own bore (#1042's dial figure).
   Throws if parts are missing. stale-arm is misaligned's geometry plus a second
   port hung under a stage-1 decoupler, and THAT port is the armed one, so one stage
   press makes the arm stale while the target stays (#1040). stale-target hangs the
   same decoupler-and-port stack under the STATION and targets that far port, so a press
   while the station is the active ship makes the TARGET stale -- which is the route
   Vehicle::dockSituation()'s TargetGone branch needs, and it is a player's route:
   Ship List -> the station -> SPACE -> back to the probe (#1062). */
DockTestShips build_dock_test_ships(const std::string &mode,
                                    const PartsCatalog &part_catalog,
                                    TerrainBody *home,
                                    TerrainBody *sun,
                                    Shader *partsshader,
                                    System &sys);
