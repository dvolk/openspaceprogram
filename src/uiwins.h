#pragma once

/* The window table: one entry per imgui window (layout + scene membership). */

#include <cstddef>   // size_t
#include <utility>   // std::forward

#include "ui.h"

struct Game;

/* Bulk-op role. Transitions close nothing; a TAB-hide does not survive a
   scene entry. Nav code closes its own menu before transitioning.

   Root        the scene's identity; forced open every frame, excluded from
               TAB and the panel. closable alone does NOT give this.
   Chrome      scene furniture; hidden by TAB, not a TAB toggle or panel row.
   Transient   modal-ish (Save/Load); open state is the nav code's job.
   Persistent  everything else; TAB and the panel toggle it. */
enum class WinRole : unsigned char { Root, Chrome, Transient, Persistent };

// One id per window, game-wide. Draw order is the call order in gameui.cpp.
enum Win : int {
    // shared across scenes (settings / info / the save slots)
    W_Settings, W_Controls, W_Debug, W_Telemetry, W_SaveLoad,
    // flight
    W_Hud, W_Windows, W_Orbital, W_Surface, W_Resources, W_OrbitalMap,
    W_SurfaceMap, W_VesselInfo, W_ShipList, W_Autopilot, W_Transfer,
    W_Porkchop,
    // title
    W_TitleMenu, W_NewGame, W_Readme,
    // space center hub
    W_SpaceCenterMenu, W_FlightSummary, W_SpaceCenterTopBar,
    // tracking station (copies of the map + ship list, free to diverge)
    W_TrackingMap, W_TrackingShipList,
    // research lab
    W_ResearchLab, W_ResearchAtlas,
    // editor
    W_VabTopBar,
    W_Staging,
    W_Count
};

struct WinDef {
    const char *name;     // the imgui window id (unique game-wide)
    const char *label;    // the Windows-panel row (unused when !inList)
    ui::Options opts;
    WinRole role;
    bool inList;          // a checkbox row in the Windows panel
};

extern const WinDef kWins[W_Count];

// The windows one scene owns. One WinDef per window game-wide.
struct WinSet { const Win *ids; size_t n; };

extern const WinSet kFlightWins, kTitleWins, kVabWins, kSpaceCenterWins,
    kTrackingWins, kResearchWins;

// Does `w` belong to the live scene's window set?
bool winInScene(const Game &g, Win w);

/* Is `w` suppressed by TAB? Everything except a scene's Root window -- a
   title screen whose only UI can be hidden is a dead end. Open state of a
   hidden window is untouched. Defined in uiwins.cpp (Game is incomplete here). */
bool hiddenByTab(const Game &g, Win w);

// Draw `w` under its own name/options, iff the live scene owns it and it is open.
template<class F>
inline bool drawWin(const Game &g, Win w, F &&body) {
    if(!winInScene(g, w) || hiddenByTab(g, w)) { return false; }
    return ui::Window(kWins[w].name, kWins[w].opts, std::forward<F>(body));
}

// Same, with a per-frame options override (the orbital map's chrome-less mode).
template<class F>
inline bool drawWin(const Game &g, Win w, const ui::Options &opts, F &&body) {
    if(!winInScene(g, w) || hiddenByTab(g, w)) { return false; }
    return ui::Window(kWins[w].name, opts, std::forward<F>(body));
}

// Open-state access without drawing.
bool winOpen(Win w);
void setWinOpen(Win w, bool open);
