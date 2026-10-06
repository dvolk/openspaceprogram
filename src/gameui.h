// gameui.h -- the ImGui UI pass: readout windows, orbital map, menus.
#pragma once

#include "game.h"

// Draw the flight readout windows. Transfer solution is computed in the 3D pass.
void drawUIReadouts(Game &g);

// User-placed popups (not slot layout) for right-clicked parts.
void drawPartWindows(Game &g);

// Draw the orbital map window for g.
void drawUIMap(Game &g);

/* Shared menu shell. Title/hub menus are Root (the scene IS the menu);
   New Game is Transient. */
void drawTitleMenu(Game &g);
void drawSpaceCenterMenu(Game &g);

// The hub's career top bar (Chrome, Space-Center-only).
void drawSpaceCenterTopBar(Game &g);

// The New Game setup sheet (Title-only).
void drawNewGame(Game &g);

// The post-recovery flight summary (Space-Center-only).
void drawFlightSummary(Game &g);

// The title screen's README panel (Title-only).
void drawReadme(Game &g);

/* Tracking Station widgets (copies of the flight windows so the two can
   diverge). */
void drawTrackingMap(Game &g);
void drawTrackingShipList(Game &g);

// The Research Lab archive (Root, Research-lab-only).
void drawResearchLab(Game &g);
// The System Atlas (Research-lab-only): the system as a tree, each body's
// research value + approach Δv + science found, and (selected) a dossier of
// its physical + orbital numbers.
void drawResearchAtlas(Game &g);
// --atlas-dump MS: print every Atlas row + dossier to stdout (test hook /
// terminal readout; the same strings the window draws).
void dumpAtlas(Game &g);
// Scene enter hook: pre-build g.labRows and g.atlasRows so the frame walk is cheap.
void researchLabEnter(Game &g);

// Draw the in-game Save/Load window. A load adopts the game's identity
// so the next save lands in its dir.
void drawSaveLoad(Game &g);

// The VAB editor's widgets.
void drawVabUI(Game &g);

// Draw live toasts on the foreground list (must float above all windows).
void drawToasts(Game &g);
