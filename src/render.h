// render.h -- the 3D render pass (draw3d): the world draw (pads, ships,
// planets, atmospheres), the active ship's per-frame state (ShipView),
// and the overlays (starbox, engine plume, the maneuver indicators, the
// reference skylines, the physics debug).
//
// This was the render section of main's loop (the block between the frame
// setup and the ImGui pass). It moved out verbatim: main's locals became
// Game members (the ship, the camera, the render resources, the draw
// toggles), and the per-frame state now lands in Game::view (ShipView)
// instead of main's locals, so the UI readouts read one snapshot. The
// transfer planner's per-frame update keeps its exact position in the
// pass (between the plume and the burn indicator) -- it is a parameter,
// not a Game member. The ImGui pass stays in main (the next extraction).
#pragma once

#include "game.h"            // Game (ShipView, the borrowed subsystems)
#include "transferplanner.h" // TransferPlanner (its update is part of the pass)

// Draw one 3D frame for g, computing g.view (the active ship's state)
// along the way. planner.update() runs inside the pass where it always
// has (the burn indicator draws from its result).
void draw3d(Game &g, TransferPlanner &planner);

// The VAB scene's 3D pass: draw the physics-free build tree (Game::vab) at
// its solved poses via DrawModelAt -- no Bullet bodies, no world/terrain.
// The orbit camera is re-aimed at the build ship (Game::vab_center) and the
// render frame is centered on it. Hovered/selected parts are tinted.
void drawVab(Game &g);
