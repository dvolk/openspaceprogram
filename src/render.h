// render.h -- the 3D pass, split into its state half and its paint half (#212).
// A scene runs refresh every frame and paints only what it puts on screen;
// the state half is not optional -- it owns the terrain LOD (hence the
// ground's collision bodies) and the Game::view snapshot the UI reads.
#pragma once

#include "game.h"   // Game

// Per-frame world state, and no draw calls in it (the LOD does delete GL
// buffers when patches collapse): camera re-pose + renderOrigin + ComputeView,
// each body's draw transform, the terrain LOD, the Game::view snapshot and the
// transfer planner.
void refreshWorld(Game &g);

// The world's pixels: skybox, pads/ships/planets, ocean/clouds/atmosphere/
// rings, engine + RCS plumes, the flight markers, skylines, debug draw.
// Reads what refreshWorld left this frame; never call it without refreshWorld.
void paintWorld(Game &g);

// The VAB scene's halves: the editor camera re-pose plus the ship state (no
// LOD -- its camera is parked at the build tree), then the physics-free build
// tree at its solved poses.
void refreshVab(Game &g);
void paintVab(Game &g);
