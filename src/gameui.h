// gameui.h -- the ImGui UI pass: readout windows, orbital map, menus. Each
// window's doc lives with its definition in gameui.cpp; its role (Root /
// Chrome / Transient) lives in the window table (uiwins.cpp).
#pragma once

#include "game.h"

void drawUIReadouts(Game &g);
void drawPartWindows(Game &g);
void drawUIMap(Game &g);
// The flight recorder's window (#972). Its own entry point rather than a block in
// drawUIReadouts because it anchors on the map, so it must draw AFTER drawUIMap.
void drawUIRecorder(Game &g);
/* The docking panel (#1026): the three gates a mate is decided by, in numbers. Its own
   entry point because it anchors under the resource bars, and because its readout comes
   from Vehicle::dockSituation() -- the same pass the dock gate runs -- not from ShipView. */
void drawUIDock(Game &g);

void drawTitleMenu(Game &g);
void drawSpaceCenterMenu(Game &g);
void drawSpaceCenterTopBar(Game &g);
void drawNewGame(Game &g);
void drawFlightSummary(Game &g);
void drawReadme(Game &g);

void drawTrackingMap(Game &g);
void drawTrackingShipList(Game &g);

void drawResearchLab(Game &g);
void drawResearchAtlas(Game &g);
void dumpAtlas(Game &g);
void dumpMapBasis(Game &g);
void researchLabEnter(Game &g);

void drawStaffLounge(Game &g);
void dumpStaff(Game &g);

/* --stage-dump: every part of the VAB build (or, with no build, the active
   hull) with its stage label and what a press does to it (#1009). */
void dumpStageTree(Game &g);

// A load adopts the game's identity, so the next save lands in its dir.
void drawSaveLoad(Game &g);

void drawVabUI(Game &g);
void drawQuestion(Game &g);

// Toasts must float above every window.
void drawToasts(Game &g);

// F4 force overlay: thrust + RCS arrows on the active ship (foreground).
void drawForceOverlay(Game &g);
