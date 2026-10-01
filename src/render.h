// render.h -- the 3D render pass (draw3d) and overlays.
#pragma once

#include "game.h"   // Game

// Draw one 3D frame for g, computing g.view along the way.
void draw3d(Game &g);

// Compute Game::view from the active ship's live state. Split out so scenes
// that run the sim without drawing the world can refresh it. No-op w/o ship.
void updateShipView(Game &g);

// The VAB scene's 3D pass: the physics-free build tree at its solved poses.
void drawVab(Game &g);
