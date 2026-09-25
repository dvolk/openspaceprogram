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
// pass (between the plume and the burn indicator); the planner is a Game
// member (g.xferPlanner). The ImGui pass stays in main (the next
// extraction).
#pragma once

#include "game.h"   // Game (ShipView, the borrowed subsystems, xferPlanner)

// Draw one 3D frame for g, computing g.view (the active ship's state)
// along the way. g.xferPlanner.update() runs inside the pass where it
// always has (the burn indicator draws from its result).
void draw3d(Game &g);

// Compute the active ship's per-frame state snapshot (Game::view, the
// ShipView the HUD / VESSEL / orbital map / Tracking Station read) from the
// ship's live physics + frame state. Split out of draw3d so a scene that runs
// the sim WITHOUT drawing the world (the live Tracking Station) can still
// refresh it each frame; draw3d calls it in the same place the pass always
// has. No-op when there is no ship (the orbit-view state).
void updateShipView(Game &g);

// The VAB scene's 3D pass: draw the physics-free build tree (Game::vab) at
// its solved poses via DrawModelAt -- no Bullet bodies, no world/terrain.
// The orbit camera is re-aimed at the build ship (Game::vab.center) and the
// render frame is centered on it. Hovered/selected parts are tinted.
void drawVab(Game &g);
