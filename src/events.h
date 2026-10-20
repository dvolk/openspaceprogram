// events.h -- input-event dispatch (see events.cpp).
#pragma once

#include "game.h"   // Game

void emit_sim_events(Game &g);
void poll_events(Game &g);

// Per-scene key maps (SceneDef::keys). Scene-neutral slots run first.
void flightKeyActions(Game &g, SDL_Scancode ksc, Uint16 kmod, bool repeat);
void vabKeyActions(Game &g, SDL_Scancode ksc, Uint16 kmod, bool repeat);
void hubKeyActions(Game &g, SDL_Scancode ksc, Uint16 kmod, bool repeat);
// The excursions on top of the hub (tracking, lab, lounge): Esc pops back.
void popKeyActions(Game &g, SDL_Scancode ksc, Uint16 kmod, bool repeat);
