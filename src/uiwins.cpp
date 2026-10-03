// The window table: one WinDef per imgui window (layout + scene membership).

#include "uiwins.h"

#include <algorithm>   // std::max

#include "game.h"    // Game
#include "scene.h"   // curScene

/* Designated array initializers ([W_Orbital] = ...) so an entry cannot drift
   away from its enum value. C++20 standardises designated initializers for
   aggregates but not array indices, so GCC flags these under -Wpedantic
   (a long-standing extension; the safety is worth the suppression). */

// The Surface Map is 2:1 and fills the content width, so the min height is
// a function of the proposed width (keeps the bottom caption from clipping).
// The chrome count mirrors the window body's rows -- keep in sync if a row
// is added there.
static void surfaceMapMinSize(ImGuiSizeCallbackData *d) {
    const ImGuiStyle &s = ImGui::GetStyle();
    const float frame = ImGui::GetFrameHeight();   // a framed row (the title bar too)
    const float text = ImGui::GetTextLineHeight();
    const float isp = s.ItemSpacing.y;
    const float map_h =
        std::max(0.0f, d->DesiredSize.x - 2.0f * s.WindowPadding.x) * 0.5f;
    const float min_h = s.WindowPadding.y * 2.0f
        + frame                 // title bar (FontSize + 2*FramePadding.y)
        + 3.0f * (frame + isp)  // body, button, checkbox rows
        + (text + isp)          // the map-size row
        + (map_h + isp)         // the map itself
        + 4.0f * (text + isp);  // caption rows: busy / hover / SOI / equirect
    if(d->DesiredSize.y < min_h) { d->DesiredSize.y = min_h; }
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
const WinDef kWins[W_Count] = {
    // --- shared across scenes ---------------------------------------------
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
        // F1 (Slot::DebugInfo): flight-only diagnostics overlay.
        .role = WinRole::Persistent, .inList = false,
    },
    [W_Telemetry] = {
        .name = "Telemetry", .label = "Telemetry",
        // 2x2 grid of plots: wider than the old two-stacked-plots layout so
        // the two columns have room (each cell is ~half this width).
        .opts = { .slot = ui::Slot::MiddleLeft, .initial_size = ImVec2(880.0f, 620.0f),
                  .closable = true, .default_open = false },
        // F2 (Slot::Telemetry): flight overlay.
        .role = WinRole::Persistent, .inList = false,
    },
    [W_SaveLoad] = {
        .name = "Save/Load", .label = "Save / Load",
        .opts = { .slot = ui::Slot::Center, .initial_size = ImVec2(380.0f, 360.0f),
                  .closable = true, .default_open = false },
        // Transient (not Root): open state is the nav code's job (WinRole).
        .role = WinRole::Transient, .inList = false,
    },

    // --- flight ----------------------------------------------------------
    [W_Hud] = {
        .name = "HUD", .label = "Top HUD",
        // Docked panel: fixed chrome; still re-fits on a relayout (F10).
        .opts = { .slot = ui::Slot::TopCenter, .fixed = true,
                  .default_open = false, .flags = ImGuiWindowFlags_NoTitleBar },
        .role = WinRole::Persistent, .inList = true,
    },
    [W_Windows] = {
        .name = "Windows", .label = "Windows",
        .opts = { .slot = ui::Slot::MiddleRight, .fixed = true, .closable = true,
                  .flags = ImGuiWindowFlags_NoTitleBar },
        // Chrome: this IS the panel, so it is not a row in itself (a panel
        // that can close itself is a dead end). TAB still hides it.
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
                  .closable = true },   // the map fills the window
        .role = WinRole::Persistent, .inList = true,
    },
    [W_SurfaceMap] = {
        .name = "Surface Map", .label = "Surface Map",
        // 2:1 equirectangular map filling the content width; surfaceMapMinSize
        // keeps the height tall enough as the window is widened. Sits under
        // Surface Info, mirroring Orbit Info -> Map.
        .opts = { .slot = ui::Slot::Center, .initial_size = ImVec2(520.0f, 450.0f),
                  .size_cb = &surfaceMapMinSize,
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
        // Fixed starting size (user-resizable); not auto-fit, so it stays
        // where the user puts it and doesn't jump as the solution appears.
        .opts = { .slot = ui::Slot::Center, .initial_size = ImVec2(420.0f, 350.0f),
                  .closable = true, .default_open = false },
        .role = WinRole::Persistent, .inList = true,
    },
    [W_Porkchop] = {
        .name = "Porkchop", .label = "Porkchop",
        // The 2-D launch-window heatmap. initial_size fits the full content
        // so the image is not clipped. Toggled from the Transfer window.
        .opts = { .slot = ui::Slot::Center, .initial_size = ImVec2(480.0f, 700.0f),
                  .closable = true, .default_open = false },
        .role = WinRole::Persistent, .inList = false,
    },
    // --- title -----------------------------------------------------------
    [W_TitleMenu] = {
        .name = "Title Menu", .label = "Title Menu",
        // Root: the title screen IS this window (forced open; no bulk op may
        // close it). closable alone is not enough -- see WinRole::Root.
        .opts = { .slot = ui::Slot::Center, .fixed = true, .default_open = true },
        .role = WinRole::Root, .inList = false,
    },
    [W_NewGame] = {
        .name = "New Game", .label = "New Game",
        // The New Game setup sheet (system + difficulty). Transient like
        // Save/Load. Docked right of the title menu so the two do not stack.
        .opts = { .slot = ui::Slot::Center, .right_of = "Title Menu",
                  .initial_size = ImVec2(400.0f, 280.0f),
                  .closable = true, .default_open = false },
        .role = WinRole::Transient, .inList = false,
    },
    [W_Readme] = {
        .name = "Readme", .label = "Readme",
        // The title screen's README panel, docked LEFT of the title menu
        // (the mirror of New Game on the right). Open by default.
        .opts = { .slot = ui::Slot::Center, .left_of = "Title Menu",
                  .initial_size = ImVec2(470.0f, 520.0f),
                  .closable = true, .default_open = true },
        .role = WinRole::Persistent, .inList = false,
    },

    // --- space center hub ------------------------------------------------
    [W_SpaceCenterMenu] = {
        .name = "Game Menu", .label = "Game Menu",
        // Root: the hub IS this window (forced open; no bulk op may close it).
        .opts = { .slot = ui::Slot::Center, .fixed = true, .default_open = true },
        .role = WinRole::Root, .inList = false,
    },
    [W_FlightSummary] = {
        .name = "Flight Summary", .label = "Flight Summary",
        // The "successful flight" dialog opened by Recover Vessel. Transient
        // like New Game. Docked right of the hub menu so they do not stack.
        .opts = { .slot = ui::Slot::Center, .right_of = "Game Menu",
                  .initial_size = ImVec2(475.0f, 355.0f),
                  .closable = true, .default_open = false },
        .role = WinRole::Transient, .inList = false,
    },

    [W_SpaceCenterTopBar] = {
        .name = "Space Center TopBar", .label = "Space Center TopBar",
        // Fixed / top-center / no-titlebar chrome (like the HUD). TAB hides
        // it; the Windows panel does not offer a row for it.
        .opts = { .slot = ui::Slot::TopCenter, .fixed = true, .default_open = true,
                  .flags = ImGuiWindowFlags_NoTitleBar },
        .role = WinRole::Chrome, .inList = false,
    },

    // --- tracking station ------------------------------------------------
    [W_TrackingMap] = {
        .name = "Tracking Map", .label = "Tracking Map",
        // Root: the map IS the Tracking Station view. drawTrackingMap
        // overrides these options per frame (full-screen, chrome-less).
        .opts = { .slot = ui::Slot::TopLeft, .fixed = true, .default_open = true,
                  .flags = ImGuiWindowFlags_NoDecoration },
        .role = WinRole::Root, .inList = false,
    },
    [W_TrackingShipList] = {
        .name = "Tracking Ship List", .label = "Tracking Ship List",
        // A copy of the flight Ship List, overlaid on the map's right side.
        .opts = { .slot = ui::Slot::TopRight, .closable = true,
                  .default_open = true },
        .role = WinRole::Persistent, .inList = false,
    },
    // --- research lab ----------------------------------------------------
    [W_ResearchLab] = {
        .name = "Research Lab", .label = "Research Lab",
        // Root: the lab IS this window (forced open; no bulk op may close
        // it). Top-left corner; initial_size gives the archive list room to
        // scroll. The System Atlas (below) matches this width.
        .opts = { .slot = ui::Slot::TopLeft, .initial_size = ImVec2(545.0f, 520.0f),
                  .default_open = true },
        .role = WinRole::Root, .inList = false,
    },
    [W_ResearchAtlas] = {
        .name = "System Atlas", .label = "System Atlas",
        // The system as a tree (star -> planets -> moons); click a body for
        // its research weight + approach Δv + science found. Docked right of
        // the lab at the SAME width; closable and draggable, so a small
        // screen can move it aside.
        .opts = { .slot = ui::Slot::TopLeft, .right_of = "Research Lab",
                  .initial_size = ImVec2(545.0f, 520.0f),
                  .closable = true, .default_open = true },
        .role = WinRole::Persistent, .inList = false,
    },
    // --- editor ----------------------------------------------------------
    [W_VabTopBar] = {
        .name = "VAB TopBar", .label = "VAB TopBar",
        // Fixed / top-center / no-titlebar chrome (like the HUD), but it is
        // core editor chrome, so it opens by default.
        .opts = { .slot = ui::Slot::TopCenter, .fixed = true, .default_open = true,
                  .flags = ImGuiWindowFlags_NoTitleBar },
        .role = WinRole::Chrome, .inList = false,
    },
    [W_Staging] = {
        .name = "Staging", .label = "Staging",
        // The VAB staging table (per-stage delta-v / TWR). Sits bottom-left
        // under the build list; the table is 5 columns and fits ~400px.
        .opts = { .slot = ui::Slot::BottomLeft, .initial_size = ImVec2(520.0f, 280.0f),
                  .closable = true, .default_open = true },
        .role = WinRole::Persistent, .inList = false,
    },
};
#pragma GCC diagnostic pop

// --- the per-scene sets ----------------------------------------------------
// A window may appear in more than one set; there is still exactly one WinDef
// for it, so the sets cannot disagree about its layout.

static const Win kFlightWinIds[] = {
    W_Hud, W_Windows, W_Orbital, W_Surface, W_Resources, W_OrbitalMap,
    W_SurfaceMap, W_VesselInfo, W_ShipList, W_Autopilot, W_Transfer, W_Porkchop,
    W_Settings, W_Controls, W_Debug, W_Telemetry, W_SaveLoad,
};
// The title screen: shared windows and nothing else -- no flight readouts
// (the set is what says so, not a guard in each window's body).
static const Win kTitleWinIds[] = {
    W_TitleMenu, W_Readme, W_NewGame, W_Settings, W_Controls, W_SaveLoad,
};
// The editor: its top-bar chrome plus the shared windows. The VAB has no
// menu of its own.
static const Win kVabWinIds[] = {
    W_VabTopBar, W_Staging, W_Settings, W_Controls, W_SaveLoad,
};
// The Space Center hub: its root menu + top bar, plus the shared windows.
// No flight readouts -- the top bar is career state, not a vessel's.
static const Win kSpaceCenterWinIds[] = {
    W_SpaceCenterMenu, W_SpaceCenterTopBar, W_FlightSummary, W_Settings,
    W_Controls, W_SaveLoad,
};
// The Tracking Station: its own map + ship list (copies of the flight
// windows, free to diverge) and the shared windows. No menu of its own.
static const Win kTrackingWinIds[] = {
    W_TrackingMap, W_TrackingShipList, W_Settings, W_Controls,
    W_SaveLoad,
};
// The Research Lab: its root window + the System Atlas, plus the shared
// windows. No menu of its own.
static const Win kResearchWinIds[] = {
    W_ResearchLab, W_ResearchAtlas, W_Settings, W_Controls, W_SaveLoad,
};

const WinSet kFlightWins = { kFlightWinIds, sizeof(kFlightWinIds) / sizeof(Win) };
const WinSet kTitleWins  = { kTitleWinIds,  sizeof(kTitleWinIds)  / sizeof(Win) };
const WinSet kVabWins    = { kVabWinIds,    sizeof(kVabWinIds)    / sizeof(Win) };
const WinSet kSpaceCenterWins = { kSpaceCenterWinIds,
                                  sizeof(kSpaceCenterWinIds) / sizeof(Win) };
const WinSet kTrackingWins = { kTrackingWinIds,
                               sizeof(kTrackingWinIds) / sizeof(Win) };
const WinSet kResearchWins = { kResearchWinIds,
                               sizeof(kResearchWinIds) / sizeof(Win) };

bool winInScene(const Game &g, Win w) {
    const WinSet &set = curScene(g).wins;
    for(size_t i = 0; i < set.n; i++) {
        if(set.ids[i] == w) { return true; }
    }
    return false;
}

bool hiddenByTab(const Game &g, Win w) {
    return !g.ui_visible && kWins[w].role != WinRole::Root;
}

bool winOpen(Win w) { return ui::IsOpen(kWins[w].name); }

void setWinOpen(Win w, bool open) { ui::SetOpen(kWins[w].name, open); }
