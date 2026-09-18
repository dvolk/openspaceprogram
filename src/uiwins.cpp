// The window table. Every value here was transcribed from the old
// Game::setup_ui_windows() verbatim -- slots, anchors, sizes, fixed/closable
// flags and default-open states are unchanged, so the flight layout comes up
// exactly as it did. What is new is the role and the per-scene membership.

#include "uiwins.h"

#include "game.h"    // Game
#include "scene.h"   // curScene

/* Designated array initializers ([W_Orbital] = ...) so an entry cannot drift
   away from its enum value, and designated struct initializers in ui::Options'
   declaration order (slot, offset, left_of, right_of, below, initial_size,
   fixed_width, fixed, closable, default_open, flags) so an omitted field keeps
   the default it had before.

   `closable = true` on nearly every window is the old info_opts() preset: an X
   on the title bar. `inList` is a row in the Windows panel -- the ones that are
   false are toggled from their own context instead (the pause menu, the
   Transfer window's Porkchop button) or are furniture. */
/* The designated ARRAY initializers below ([W_Orbital] = {...}) are what stop
   an entry drifting away from its enum value when a window is added or the enum
   is reordered -- a plain positional list would silently mis-align every
   window after the insertion. ISO C++20 standardised designated initializers
   for aggregates but not array indices, so GCC flags these under -Wpedantic;
   they are a long-standing GCC/Clang extension and the safety is worth the
   scoped suppression. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
const WinDef kWins[W_Count] = {
    // --- shared across scenes: settings, the key map, the debug/telemetry
    // --- readouts and the save slots. Same entry in every scene that lists
    // --- them, so their layout has exactly one home.
    [W_Settings] = {
        .name = "Settings", .label = "Settings",
        .opts = { .slot = ui::Slot::BottomCenter, .closable = true,
                  .default_open = false },
        .role = WinRole::Persistent, .inList = false,
    },
    [W_Controls] = {
        .name = "Controls", .label = "Controls",
        .opts = { .slot = ui::Slot::BottomCenter, .closable = true,
                  .default_open = false },
        .role = WinRole::Persistent, .inList = false,
    },
    [W_Debug] = {
        .name = "Game Debug Info", .label = "Game Debug Info",
        .opts = { .slot = ui::Slot::TopCenter, .closable = true,
                  .default_open = false },
        .role = WinRole::Persistent, .inList = false,
    },
    [W_Telemetry] = {
        .name = "Telemetry", .label = "Telemetry",
        // 2x2 grid of plots: wider than the old two-stacked-plots layout so
        // the two columns have room (each cell is ~half this width).
        .opts = { .slot = ui::Slot::MiddleLeft, .initial_size = ImVec2(880.0f, 620.0f),
                  .closable = true, .default_open = false },
        .role = WinRole::Persistent, .inList = false,
    },
    [W_SaveLoad] = {
        .name = "Save/Load", .label = "Save / Load",
        .opts = { .slot = ui::Slot::Center, .initial_size = ImVec2(380.0f, 360.0f),
                  .closable = true, .default_open = false },
        // Transient: it captures the live fleet, so it must not survive a
        // transition into a scene where that fleet is not what is on screen.
        .role = WinRole::Transient, .inList = false,
    },

    // --- flight ----------------------------------------------------------
    [W_Hud] = {
        .name = "HUD", .label = "Top HUD",
        // Docked panel: no title bar, not user-movable/resizable; it still
        // re-fits and re-places on a relayout (F10 / "Reset windows"). Closed
        // by default -- it is opt-in from the Windows panel.
        .opts = { .slot = ui::Slot::TopCenter, .fixed = true,
                  .default_open = false, .flags = ImGuiWindowFlags_NoTitleBar },
        .role = WinRole::Persistent, .inList = true,
    },
    [W_Windows] = {
        .name = "Windows", .label = "Windows",
        .opts = { .slot = ui::Slot::MiddleRight, .fixed = true, .closable = true,
                  .flags = ImGuiWindowFlags_NoTitleBar },
        // Chrome: this IS the panel, so it is not a row in itself and TAB does
        // not toggle it (a panel that can close itself is a dead end -- TAB
        // still hides it, for a clean screenshot).
        .role = WinRole::Chrome, .inList = false,
    },
    // Layout: top left ORBITAL + SURFACE, top right RESOURCES, middle right
    // the window list, bottom right VESSEL, bottom left the orbit map.
    [W_Orbital] = {
        .name = "Orbital", .label = "Orbit Info",
        .opts = { .slot = ui::Slot::TopLeft, .closable = true },
        .role = WinRole::Persistent, .inList = true,
    },
    [W_Surface] = {
        .name = "Surface", .label = "Surface Info",
        .opts = { .slot = ui::Slot::TopLeft, .right_of = "Orbital", .closable = true },
        .role = WinRole::Persistent, .inList = true,
    },
    [W_Resources] = {
        .name = "Resources", .label = "Resources",
        // bars have no width of their own; pin it to a 7-resource column so
        // the width doesn't shrink with the number of resources shown (in
        // font-size units, so it tracks the font size and the DPI scale)
        .opts = { .slot = ui::Slot::TopRight, .fixed_width = 2.0f * 7.0f,
                  .closable = true },
        .role = WinRole::Persistent, .inList = true,
    },
    [W_OrbitalMap] = {
        .name = "Orbital Map", .label = "Orbit Map",
        .opts = { .slot = ui::Slot::BottomLeft, .initial_size = ImVec2(480.0f, 480.0f),
                  .closable = true },   // orbit drawn at (200,200)
        .role = WinRole::Persistent, .inList = true,
    },
    [W_SurfaceMap] = {
        .name = "Surface Map", .label = "Surface Map",
        // The body's 2-D surface (equirectangular) with the ship's position +
        // orbit overlaid. 256 x 128 default map + the combo + readouts fits in
        // 520 x 430. Sits under Surface Info, mirroring Orbit Info -> Map.
        .opts = { .slot = ui::Slot::Center, .initial_size = ImVec2(520.0f, 430.0f),
                  .closable = true, .default_open = false },
        .role = WinRole::Persistent, .inList = true,
    },
    [W_VesselInfo] = {
        .name = "Vessel Info", .label = "Vessel Info",
        .opts = { .slot = ui::Slot::BottomRight, .closable = true },
        .role = WinRole::Persistent, .inList = true,
    },
    [W_ShipList] = {
        .name = "Ship List", .label = "Ship List",
        // In the list regardless of fleet size: the window is always drawn, so
        // it always needs the toggle + checkbox.
        .opts = { .slot = ui::Slot::TopCenter, .closable = true,
                  .default_open = false },
        .role = WinRole::Persistent, .inList = true,
    },
    [W_Autopilot] = {
        .name = "Autopilot", .label = "Autopilot",
        .opts = { .slot = ui::Slot::Center, .left_of = "Windows", .closable = true,
                  .default_open = false },   // docked left of the window list
        .role = WinRole::Persistent, .inList = true,
    },
    [W_Transfer] = {
        .name = "Transfer", .label = "Transfer",
        .opts = { .slot = ui::Slot::Center, .closable = true, .default_open = false },
        .role = WinRole::Persistent, .inList = true,
    },
    [W_Porkchop] = {
        .name = "Porkchop", .label = "Porkchop",
        // The 2-D launch-window heatmap. initial_size fits the full content
        // (420px heatmap + colorbar + the readouts + captions) so the image is
        // not clipped; the window stays user-movable/resizable. Toggled from
        // the Transfer window, not from the panel.
        .opts = { .slot = ui::Slot::Center, .initial_size = ImVec2(520.0f, 660.0f),
                  .closable = true, .default_open = false },
        .role = WinRole::Persistent, .inList = false,
    },
    [W_PauseMenu] = {
        .name = "Pause Menu", .label = "Pause Menu",
        // Fixed + centered: it is a menu, not a panel to arrange. Closable
        // (the X and Esc both resume), and Transient so pushing the VAB from
        // here cannot leave it open to reappear over a running sim.
        .opts = { .slot = ui::Slot::Center, .fixed = true, .closable = true,
                  .default_open = false },
        .role = WinRole::Transient, .inList = false,
    },

    // --- title -----------------------------------------------------------
    [W_TitleMenu] = {
        .name = "Title Menu", .label = "Title Menu",
        // Root: the title screen IS this window, so it is forced open by
        // titleDrawUi and no bulk operation may close it. closable stays false
        // (no X) -- but that alone would not be enough, see WinRole::Root.
        .opts = { .slot = ui::Slot::Center, .fixed = true, .default_open = true },
        .role = WinRole::Root, .inList = false,
    },

    // --- editor ----------------------------------------------------------
    [W_VabTopBar] = {
        .name = "VAB TopBar", .label = "VAB TopBar",
        // The same fixed / top-center / no-titlebar treatment as the HUD, but
        // it is core editor chrome, so it opens by default.
        .opts = { .slot = ui::Slot::TopCenter, .fixed = true, .default_open = true,
                  .flags = ImGuiWindowFlags_NoTitleBar },
        .role = WinRole::Chrome, .inList = false,
    },
};
#pragma GCC diagnostic pop

// --- the per-scene sets ----------------------------------------------------
// A window may appear in more than one set; there is still exactly one WinDef
// for it, so the sets cannot disagree about its layout.

static const Win kFlightWinIds[] = {
    W_Hud, W_Windows, W_Orbital, W_Surface, W_Resources, W_OrbitalMap,
    W_SurfaceMap, W_VesselInfo, W_ShipList, W_Autopilot, W_Transfer, W_Porkchop,
    W_Settings, W_Controls, W_Debug, W_Telemetry, W_SaveLoad, W_PauseMenu,
};
// The title screen gets the shared windows and nothing else -- in particular
// no flight readouts, which is the whole point: there is no vessel, and the
// set is what says so rather than a guard in each window's body.
static const Win kTitleWinIds[] = {
    W_TitleMenu, W_Settings, W_Controls, W_Debug, W_Telemetry, W_SaveLoad,
};
static const Win kVabWinIds[] = {
    W_VabTopBar,
};

const WinSet kFlightWins = { kFlightWinIds, sizeof(kFlightWinIds) / sizeof(Win) };
const WinSet kTitleWins  = { kTitleWinIds,  sizeof(kTitleWinIds)  / sizeof(Win) };
const WinSet kVabWins    = { kVabWinIds,    sizeof(kVabWinIds)    / sizeof(Win) };

bool winInScene(const Game &g, Win w) {
    const WinSet &set = curScene(g).wins;
    for(size_t i = 0; i < set.n; i++) {
        if(set.ids[i] == w) { return true; }
    }
    return false;
}

bool winOpen(Win w) { return ui::IsOpen(kWins[w].name); }

void setWinOpen(Win w, bool open) { ui::SetOpen(kWins[w].name, open); }
