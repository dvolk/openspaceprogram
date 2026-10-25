// gameui.h -- the ImGui UI pass: readout windows, orbital map, menus. Each
// window's doc lives with its definition in gameui.cpp; its role (Root /
// Chrome / Transient) lives in the window table (uiwins.cpp).
#pragma once

#include "game.h"

void drawUIReadouts(Game &g);
void drawPartWindows(Game &g);
void drawUIMap(Game &g);

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

// A load adopts the game's identity, so the next save lands in its dir.
void drawSaveLoad(Game &g);

void drawVabUI(Game &g);
void drawQuestion(Game &g);

// Toasts must float above every window.
void drawToasts(Game &g);

// F4 force overlay: thrust + RCS arrows on the active ship (foreground).
void drawForceOverlay(Game &g);
