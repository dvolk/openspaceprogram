// tick.h -- the game's fixed-timestep logic tick (was the "LOGIC" section
// of main's loop):
//
//   tick(g)  advances the frame accumulator by the measured frame time,
//            then runs as many fixed ticks (g.dt) as fit: arm thrust and
//            rotation from the held keys (plus the free-camera WASD),
//            advance the sim clock (g.time), tick every ship (rails or
//            substepped Bullet) and fire the --spin-log / --orbit-log /
//            --dbg-log output. Marks the frame for a redraw (g.redraw).
//
// Everything reads/writes state through Game, so this file touches nothing
// outside it. The EVENTS section is in events.cpp; RENDER stays in main.
#pragma once

#include "game.h"   // Game

void tick(Game &g);
