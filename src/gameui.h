// gameui.h -- the ImGui UI pass: the readout windows (drawUIReadouts),
// the orbital map (drawUIMap) and the fixed main menu (drawMainMenu),
// drawn in main's loop after the 3D pass (render.cpp) in that order.
//
// This was the ImGui section of main's loop. It moved out verbatim: main's
// locals became Game members (the per-window options, the Settings state,
// the big font, the draw toggles, the map state), and the per-frame state
// the readouts show is the ShipView snapshot render.cpp computes.
#pragma once

#include "game.h"            // Game (ShipView, the UI state)
#include "transferplanner.h" // TransferPlanner (the TRANSFER window state)

// Draw the readout windows (HUD, Windows, Settings, TRANSFER, Game Debug
// Info, ORBITAL, TELEMETRY, SURFACE, SHIPS, VESSEL, SHIP PARTS, Controls,
// Autopilot, RESOURCES) for g. planner feeds the TRANSFER window (its
// solution is computed in the 3D pass).
void drawUIReadouts(Game &g, TransferPlanner &planner);

// Draw the open part windows: one plain imgui window per g.part_sels
// entry (the parts the player right-clicked in the 3D view). Not part of
// the slot layout -- they are user-placed popups, several may be open at
// once. Closing a window (X) or staging/removing its part drops the
// entry.
void drawPartWindows(Game &g);

// Draw the orbital map window for g. planner feeds it (the transfer
// conic + the selected target's highlight).
void drawUIMap(Game &g, TransferPlanner &planner);

// Draw the fixed main menu (Esc toggles it). Drawn last so it sits on top.
void drawMainMenu(Game &g);

// Draw the one-shot on-screen messages (g.toast): the last kToastVisible
// that are still alive, stacked and centered on the screen. A bare
// foreground-draw-list overlay (no imgui window): the messages are
// non-interactive and must float above everything, so main draws this
// after drawMainMenu.
void drawToasts(Game &g);
