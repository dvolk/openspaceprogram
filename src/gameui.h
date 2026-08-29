// gameui.h -- the ImGui UI pass: the readout windows (drawUIReadouts) and
// the fixed main menu (drawMainMenu), drawn in main's loop after the 3D
// pass (render.cpp).
//
// This was the ImGui section of main's loop. It moved out verbatim: main's
// locals became Game members (the per-window options, the Settings state,
// the big font, the draw toggles), and the per-frame state the readouts
// show is the ShipView snapshot render.cpp computes. The orbital map --
// the last window, extracted separately -- draws between the readouts and
// the main menu, keeping the original window order / z-order.
#pragma once

#include "game.h"            // Game (ShipView, the UI state)
#include "transferplanner.h" // TransferPlanner (the TRANSFER window state)

// Draw the readout windows (HUD, Windows, Settings, TRANSFER, Game Debug
// Info, ORBITAL, TELEMETRY, SURFACE, SHIPS, VESSEL, SHIP PARTS, Controls,
// Autopilot, RESOURCES) for g. planner feeds the TRANSFER window (its
// solution is computed in the 3D pass).
void drawUIReadouts(Game &g, TransferPlanner &planner);

// Draw the fixed main menu (Esc toggles it). Drawn last so it sits on top.
void drawMainMenu(Game &g);
