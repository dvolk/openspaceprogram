// gameui.h -- the ImGui UI pass: the readout windows (drawUIReadouts),
// the orbital map (drawUIMap) and the two menus (drawTitleMenu /
// drawSpaceCenterMenu), drawn in main's loop after the 3D pass (render.cpp).
//
// This was the ImGui section of main's loop. It moved out verbatim: main's
// locals became Game members (the per-window options, the Settings state,
// the big font, the draw toggles, the map state), and the per-frame state
// the readouts show is the ShipView snapshot render.cpp computes.
#pragma once

#include "game.h"   // Game (ShipView, the UI state, xferPlanner)

// Draw the readout windows (HUD, Windows, Settings, TRANSFER, Game Debug
// Info, ORBITAL, TELEMETRY, SURFACE, SHIPS, VESSEL, Controls, Autopilot,
// RESOURCES) for g. g.xferPlanner feeds the TRANSFER window (its
// solution is computed in the 3D pass).
void drawUIReadouts(Game &g);

// Draw the open part windows: one plain imgui window per g.part_sels
// entry (the parts the player right-clicked in the 3D view). Not part of
// the slot layout -- they are user-placed popups, several may be open at
// once. Closing a window (X) or staging/removing its part drops the
// entry.
void drawPartWindows(Game &g);

// Draw the orbital map window for g. g.xferPlanner feeds it (the transfer
// conic + the selected target's highlight).
void drawUIMap(Game &g);

/* The menus and title overlays: the shared menu shell (drawTitleMenu /
   drawSpaceCenterMenu) and the New Game setup sheet (drawNewGame: system +
   exhaust-velocity difficulty). The shell's heading + navigation block
   differ per scene; the standard items (Save/Load, Settings, Controls,
   Quit game) are shared, and the hub adds a "Return to title".
   The menus are Root windows -- forced open, no X -- because the title
   screen and the Space Center hub ARE their menus; New Game is Transient
   (opened by the title's "New Game"). The other scenes (flight, VAB,
   tracking, research lab) have no menu of their own: Esc walks up the tree
   and the hub is the only in-game menu. */
void drawTitleMenu(Game &g);
void drawSpaceCenterMenu(Game &g);

/* The hub's top bar (W_SpaceCenterTopBar): the career-level readouts --
   the home calendar clock, the science recovered so far, and how many
   vessels are out there. Chrome like the VAB's bar (fixed, top-center,
   no titlebar, hidden by TAB). Space-Center-only. */
void drawSpaceCenterTopBar(Game &g);

/* The New Game setup sheet (W_NewGame), opened by the title menu's "New
   Game": name the game (the <stamp>-<name> dir its saves land in under
   saves/), pick the star system and the exhaust-velocity scale (difficulty).
   Start applies all and calls startNewGame; Cancel closes. Title-only (it
   is in kTitleWins). */
void drawNewGame(Game &g);

/* The Flight Summary window (W_FlightSummary), opened by the hub's
   "Recover Vessel" (Game::recoverActive): the successful-flight sheet
   (vessel, calendar duration, SoI enter/leave journal).
   Space-Center-only (it is in kSpaceCenterWins). */
void drawFlightSummary(Game &g);

/* The title screen's README panel (W_Readme), docked left of the title
   menu. Shows the player-facing readme text (release/README.md in the
   source tree, README.md beside the assets in a package) as plain text.
   Open by default; the menu's "Readme" toggles it. Title-only. */
void drawReadme(Game &g);

/* The Tracking Station's widgets: a full-screen, chrome-less orbital map and a
   ship list, each a COPY of the flight window's draw code into its own window id
   (W_TrackingMap / W_TrackingShipList) so the two scenes' versions can diverge
   without touching each other. drawTrackingMap reads g.xferPlanner for the
   same transfer-conic overlay the flight map draws. */
void drawTrackingMap(Game &g);
void drawTrackingShipList(Game &g);

/* The Research Lab's widget: the archive of recovered experiments
   (Game::science.recovered, named via experimentName) plus the career science
   score. Read-only -- recovery (Game::recoverActive) is what adds entries.
   The window is the scene's Root (forced open by researchLabDrawUi); "Back
   to Space Center" + Esc are the exits. Research-lab-only. */
void drawResearchLab(Game &g);
/* The Lab's scene entry (the scene table's enter hook): build g.labRows from
   the log + home calendar (labEntries, issue #87) so the per-frame render
   walks pre-built strings. Called by pushScene when the Lab is entered. */
void researchLabEnter(Game &g);

// Draw the in-game Save/Load window: a slot name to save the CURRENT game
// into (its <stamp>-<name> dir under saves/) + the list of existing games
// and their slots to load / delete. Opened from the main menu; drawn with
// the other UI (main-menu group). Saving/loading the live fleet + clock is
// the save_game / load_game pair in save.cpp (a load also adopts that
// game's identity, so the next save lands in its dir).
void drawSaveLoad(Game &g);

// The VAB editor scene's widgets: the selected-part panel (the part is picked
// by clicking it in the 3D view), the fuel-link list, the subassemblies, and
// hints. Drawn instead of the flight readouts when Game::scene == Vab.
void drawVabUI(Game &g);

// Draw the one-shot on-screen messages (g.toast): the last kToastVisible
// that are still alive, stacked and centered on the screen. A bare
// foreground-draw-list overlay (no imgui window): the messages are
// non-interactive and must float above everything, so main draws this
// after the scene's menu.
void drawToasts(Game &g);
