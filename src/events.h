// events.h -- the game's input-event dispatch (was the "EVENTS" section of
// main's loop):
//
//   emit_sim_events(g)  pushes the --sim-press / --sim-mouse events that
//                       fell due this frame into the SDL queue (they are
//                       polled in the same frame, below).
//   poll_events(g)      drains the SDL queue and dispatches each event:
//                       quit, resize, the keybinds, the RMB camera-look and
//                       the wheel. Sets g.running = false on QUIT.
//
// Everything reads/writes state through Game, so this file touches nothing
// outside it. The LOGIC section (thrust / rotation commands) and the RENDER
// section stay in main.
#pragma once

#include "game.h"   // Game

void emit_sim_events(Game &g);
void poll_events(Game &g);
