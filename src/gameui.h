// gameui.h -- the ImGui UI pass: the readout windows (drawUIReadouts),
// the orbital map (drawUIMap) and the two menus (drawTitleMenu /
// drawSpaceCenterMenu), drawn in main's loop after the 3D pass (render.cpp).
//
// This was the ImGui section of main's loop. It moved out verbatim: main's
// locals became Game members (the per-window options, the Settings state,
// the big font, the draw toggles, the map state), and the per-frame state
// the readouts show is the ShipView snapshot render.cpp computes.
#pragma once

#include "game.h"            // Game (ShipView, the UI state)
#include "transferplanner.h" // TransferPlanner (the TRANSFER window state)

// Draw the readout windows (HUD, Windows, Settings, TRANSFER, Game Debug
// Info, ORBITAL, TELEMETRY, SURFACE, SHIPS, VESSEL, Controls, Autopilot,
// RESOURCES) for g. planner feeds the TRANSFER window (its
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

/* The menus and title overlays: the shared menu shell (drawTitleMenu /
   drawSpaceCenterMenu) and the New Game setup sheet (drawNewGame: system +
   exhaust-velocity difficulty). The shell's heading + navigation block
   differ per scene; the standard items (Save/Load, Settings, Controls,
   Quit game) are shared, and the hub adds a confirmed "Return to title".
   The menus are Root windows -- forced open, no X -- because the title
   screen and the Space Center hub ARE their menus; New Game is Transient
   (opened by the title's "New Game"). The other scenes (flight, VAB,
   tracking) have no menu of their own: Esc walks up the tree and the hub
   is the only in-game menu. */
void drawTitleMenu(Game &g);
void drawSpaceCenterMenu(Game &g);

/* The New Game setup sheet (W_NewGame), opened by the title menu's "New
   Game": pick which star system to load and the exhaust-velocity scale
   (difficulty). Start applies both and calls startNewGame; Cancel closes.
   Title-only (it is in kTitleWins). */
void drawNewGame(Game &g);

/* The title screen's README panel (W_Readme), docked left of the title
   menu. Shows the player-facing readme text (release/README.md in the
   source tree, README.md beside the assets in a package) as plain text.
   Open by default; the menu's "Readme" toggles it. Title-only. */
void drawReadme(Game &g);

/* The Tracking Station's widgets: a full-screen, chrome-less orbital map and a
   ship list, each a COPY of the flight window's draw code into its own window id
   (W_TrackingMap / W_TrackingShipList) so the two scenes' versions can diverge
   without touching each other. drawTrackingMap takes the planner for the same
   transfer-conic overlay the flight map draws. */
void drawTrackingMap(Game &g, TransferPlanner &planner);
void drawTrackingShipList(Game &g);

// Draw the in-game Save/Load window (a name to save into + the list of
// existing saves to load / delete). Opened from the main menu; drawn with
// the other UI (main-menu group). Saving/loading the live fleet + clock is
// the save_game / load_game pair in save.cpp.
void drawSaveLoad(Game &g);

// The VAB editor scene's widgets: the build-tree part list (select), and
// hints. Drawn instead of the flight readouts when Game::scene == Vab.
void drawVabUI(Game &g);

// Draw the one-shot on-screen messages (g.toast): the last kToastVisible
// that are still alive, stacked and centered on the screen. A bare
// foreground-draw-list overlay (no imgui window): the messages are
// non-interactive and must float above everything, so main draws this
// after the scene's menu.
void drawToasts(Game &g);
