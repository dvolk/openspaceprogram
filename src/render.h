// render.h -- the 3D pass, split into its state half and its paint half (#212).
// The state half is not optional: it owns the terrain LOD (hence the ground's
// collision bodies) and the Game::hud.view snapshot the UI reads.
#pragma once

#include "game.h"   // Game

// Per-frame world state, no draw calls in it (the LOD deletes GL buffers when
// patches collapse).
void refreshWorld(Game &g);

// The world's pixels. Reads what refreshWorld left this frame; never call it
// without refreshWorld.
void paintWorld(Game &g);

// VAB halves: camera re-pose plus ship state (no LOD -- its camera is parked
// at the build tree), then the physics-free build tree at its solved poses.
void refreshVab(Game &g);
void paintVab(Game &g);

// The Game::hud.view snapshot on its own (no camera, no LOD). The UI takes it when
// the active ship changed during the imgui pass and this frame's snapshot
// describes the previous one (#235).
void updateShipView(Game &g);
