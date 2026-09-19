#pragma once

/* The window table: one entry per imgui window in the game, holding its layout
   and saying which scenes it belongs to.

   Before this, a window's identity was split across three places that nothing
   kept in sync: a string literal at the draw site, a loose `ui::Options o_*`
   field on Game, and a row in Game::ui_windows that COPIED the options. And no
   layer knew which windows belonged to which mode, so the only way to keep the
   flight readouts off the game-start screen was a `ship == nullptr` guard
   inside each of their bodies.

   Now the draw sites say `drawWin(g, W_Orbital, ...)`: the name and the layout
   come from one table entry and cannot be paired wrongly, and the scene
   membership check is built into the call. A window that is not in the live
   scene's set is not drawn -- by construction, not by guarding.

   See reports/ui-scenes2026_09_17 (ui-scenes.md §3.3, verification.md §A1-A2). */

#include <cstddef>   // size_t
#include <utility>   // std::forward

#include "ui.h"

struct Game;

/* What the bulk operations (TAB, the Windows panel, a layout reset) are
   allowed to do to a window. Scene transitions close NOTHING -- push/pop/
   enterTitle never touch open state, with one exception: a TAB-hide
   (ui_visible=false) does not survive a transition, every scene entry
   restores visibility (Game::ensure_ui_visible), so a hidden flight scene
   cannot hand the player a blue VAB. What keeps a scene's menus honest is the
   nav code closing its own menu before it transitions (gameui.cpp's nav*
   callbacks), and the WinSet gate (winInScene) keeping a stale one from
   drawing where it is not owned.

   Root        the scene's identity -- the title and hub menus, the tracking
               map. Forced open every frame by the scene's drawUi, and
               excluded from TAB and the panel, so no key combination can
               leave a scene with no UI at all. ui::Options::closable alone
               does NOT give this: it only hides the X button, ui::SetOpen
               still closes the window and ui::Window then early-returns.
   Chrome      scene furniture -- the Windows panel, the VAB top bar. Drawn
               with the scene and hidden by TAB, but not a TAB toggle and not
               a panel row (a panel that can close itself is a dead end).
   Transient   modal-ish -- the Save/Load window. Not a TAB toggle and not a
               panel row; its open state is the nav code's responsibility (see
               above). (The old per-scene menus were Transient; they are gone
               now that the hub is the only in-game menu.)
   Persistent  everything else. TAB and the panel toggle it; a transition
               leaves its open state alone (a TAB-hide is the one state a
               scene entry resets, see above), so an excursion to the VAB and
               back restores the flight layout exactly as you left it. */
enum class WinRole : unsigned char { Root, Chrome, Transient, Persistent };

/* One id per window, game-wide. The order is irrelevant (the draw order is
   the call order in gameui.cpp); the enum exists so the table, the scene
   window sets and the draw sites all refer to the same thing by name. */
enum Win : int {
    // shared across scenes (settings / info / the save slots)
    W_Settings, W_Controls, W_Debug, W_Telemetry, W_SaveLoad,
    // flight
    W_Hud, W_Windows, W_Orbital, W_Surface, W_Resources, W_OrbitalMap,
    W_SurfaceMap, W_VesselInfo, W_ShipList, W_Autopilot, W_Transfer,
    W_Porkchop,
    // title
    W_TitleMenu,
    // space center hub
    W_SpaceCenterMenu,
    // tracking station (its own copies of the map + ship list, so they can
    // diverge from the flight ones)
    W_TrackingMap, W_TrackingShipList,
    // editor
    W_VabTopBar,
    W_Count
};

struct WinDef {
    const char *name;     // the imgui window id (unique game-wide)
    const char *label;    // the Windows-panel row (unused when !inList)
    ui::Options opts;     // THE layout: one home, nothing copies it
    WinRole role;
    bool inList;          // a checkbox row in the Windows panel
};

extern const WinDef kWins[W_Count];

/* The windows one scene owns. `ids`/`n` drive the Windows panel's rows and
   the TAB toggle; winInScene scans it to decide whether a window may be drawn
   at all. A window in two scenes' sets is the same entry in both -- there is
   exactly one WinDef per window game-wide, so its ui::Options has one home and
   cannot be given two conflicting layouts. */
struct WinSet { const Win *ids; size_t n; };

extern const WinSet kFlightWins, kTitleWins, kVabWins, kSpaceCenterWins,
    kTrackingWins;

// Does `w` belong to the live scene's window set?
bool winInScene(const Game &g, Win w);

/* Is `w` suppressed by TAB (Game::ui_visible)? Everything goes except a
   scene's Root window (its menu / identity) -- that is the whole point of
   TAB, a clean screenshot in one key, and a title screen whose only UI can be
   hidden is the original bug back again. A hidden-but-still-open menu would
   make the next Esc CLOSE it (toggle of open state, which TAB does not touch),
   so the player would press Esc twice and see nothing -- the Root exemption
   is what stops that. Centralised here rather than as an early-return in each
   draw function, which is how two of them (Save/Load, the editor) came to
   honour TAB while the flight windows did not. Note the open STATE of a hidden
   window is untouched: toggle_windows only flips Persistent ones, so Chrome
   and Transient come back exactly as they were. */
bool hiddenByTab(const Game &g, Win w);   // defined in uiwins.cpp (Game is
                                           // incomplete in this header)

/* Draw window `w` under its own name and options, iff the live scene owns it.
   Returns false (and draws nothing) when the window is not in this scene's set
   or the player has closed it. */
template<class F>
inline bool drawWin(const Game &g, Win w, F &&body) {
    if(!winInScene(g, w) || hiddenByTab(g, w)) { return false; }
    return ui::Window(kWins[w].name, kWins[w].opts, std::forward<F>(body));
}

/* The same, with a per-frame override of the table's options -- for the one
   window whose flags depend on runtime state (the orbital map's chrome-less
   mode 2). The table stays const and shared; the caller copies and adjusts. */
template<class F>
inline bool drawWin(const Game &g, Win w, const ui::Options &opts, F &&body) {
    if(!winInScene(g, w) || hiddenByTab(g, w)) { return false; }
    return ui::Window(kWins[w].name, opts, std::forward<F>(body));
}

// Open-state access for code that needs to ask or set without drawing (the
// panel's own rows, the menu buttons that open Settings / Save-Load, ...).
bool winOpen(Win w);
void setWinOpen(Win w, bool open);
