// game.cpp -- the control transitions of the running game (declared in
// game.h): switching / removing the active ship, entering rails warp, the
// UI window toggle and the orbit-camera focus resolution. These were
// lambdas in main(); they touch the fleet (Ships) and the runtime state
// (Game), so they live with the state.
#include "game.h"

#include <cstdarg>
#include <cstdio>
#include <string>

glm::dvec3 Game::focusWorldPos(int i) const {
    if (focusTargets[i].body == nullptr) {
        return ship->get_center_of_mass();
    }
    return focusTargets[i].body->frame->GetPositionRelTo(ship->frame);
}

/* Build the per-window UI options + the window registry. This was a block
   in main(): one options block per window (src/ui.h) plus the table the
   main-menu checkboxes, the F10 toggle and a UI reset all share. The main
   menu itself and the HUD (one "Top HUD" checkbox) are handled
   separately. Slots/offsets: left column stacks under the menu, right
   column under ORBITAL, the rest spread over the edges so the center
   stays clear for the 3D view. */
void Game::setup_ui_windows() {
    auto info_opts = [](ui::Slot slot) {
        ui::Options o;
        o.slot = slot;
        return o;
    };
    // Layout: top left ORBITAL + SURFACE, top right RESOURCES,
    // middle right the menu, bottom right VESSEL, bottom left Orbital map.
    o_orbit     = info_opts(ui::Slot::TopLeft);
    o_surface   = info_opts(ui::Slot::TopLeft);
    o_surface.right_of = "Orbital";
    o_resources = info_opts(ui::Slot::TopRight);
    o_resources.width_ratio = 1.5f; // bars have no width of their own
    o_menu      = info_opts(ui::Slot::MiddleRight);
    o_vessel    = info_opts(ui::Slot::BottomRight);
    o_parts     = info_opts(ui::Slot::BottomRight);
    o_parts.below = "Vessel Info";
    o_parts.default_open = false;
    o_map = info_opts(ui::Slot::BottomLeft);
    o_map.default_open = true;
    o_map.initial_size = ImVec2(480.0f, 480.0f); // orbit drawn at (200,200)
    // The rest stay out of the way of the above (all closed by default).
    o_ships = info_opts(ui::Slot::TopCenter);
    o_ships.default_open = false;
    o_autopilot = info_opts(ui::Slot::Center);
    o_autopilot.default_open = false;
    o_controls  = info_opts(ui::Slot::BottomCenter);
    o_controls.default_open = false;
    o_controls.closable = true;  // X close button (menu window)
    o_debug     = info_opts(ui::Slot::TopCenter);
    o_debug.default_open = false;
    o_telemetry = info_opts(ui::Slot::MiddleLeft);
    o_telemetry.default_open = false;
    o_telemetry.initial_size = ImVec2(460.0f, 680.0f);
    o_settings  = info_opts(ui::Slot::BottomCenter);
    o_settings.default_open = false;
    o_settings.closable = true;  // X close button (menu window)
    // Transfer planner: target selection + dv readouts.
    o_transfer = info_opts(ui::Slot::Center);
    o_transfer.default_open = false;
    // Porkchop plot: the 2-D launch-window heatmap (on-demand compute).
    // initial_size fits the full content (420px heatmap + colorbar + the
    // readouts + captions) so the image isn't clipped; the window stays
    // user-movable/resizable (a refit-on-content-change would let it auto-
    // fit the short pre-compute state too -- noted for later).
    o_porkchop = info_opts(ui::Slot::Center);
    o_porkchop.initial_size = ImVec2(520.0f, 660.0f);
    o_porkchop.default_open = false;
    o_hud.fixed = true;
    o_hud.default_open = false;
    o_hud.flags |= ImGuiWindowFlags_NoTitleBar;
    o_hud.slot = ui::Slot::TopCenter;
    o_mainmenu = info_opts(ui::Slot::Center);
    o_mainmenu.fixed = true;
    o_mainmenu.default_open = false;

    // The registry (ui_windows): the TAB toggle and the main-menu
    // "Toggle windows" read it.
    auto add_ui_window = [&](const char *name, const char *label,
                             const ui::Options &o) {
        ui_windows.push_back(UiWin{name, label, o});
    };
    add_ui_window("Resources", "Resources", o_resources);
    add_ui_window("Orbital", "Orbit Info", o_orbit);
    add_ui_window("Orbital Map", "Orbit Map", o_map);
    add_ui_window("Surface", "Surface Info", o_surface);
    add_ui_window("Vessel Info", "Vessel Info", o_vessel);
    add_ui_window("Ship Parts", "Vessel Parts", o_parts);
    // Registered regardless of fleet size: the window (ship list + spawn)
    // is always drawn, so it always needs the toggle + checkbox.
    add_ui_window("Ship List", "Ship List", o_ships);
    add_ui_window("Autopilot", "Autopilot", o_autopilot);
    // Controls and Settings are main-menu only, so they're deliberately
    // NOT in this list (and thus not affected by the TAB toggle).
    add_ui_window("Game Debug Info", "Game Debug Info", o_debug);
    add_ui_window("Telemetry", "Telemetry", o_telemetry);
    add_ui_window("Transfer", "Transfer", o_transfer);
    add_ui_window("Porkchop", "Porkchop", o_porkchop);
    // The TAB toggle + the main-menu "Toggle windows" button call
    // toggle_windows(), which flips ui_visible and re-opens every registry
    // window (plus the HUD) from their defaults.
}

void Game::toggle_windows() {
    ui_visible = !ui_visible;
    for(auto &w : ui_windows) {
        ui::SetOpen(w.name, ui_visible && w.opts.default_open);
    }
    ui::SetOpen("HUD", ui_visible && o_hud.default_open);
}

/* Rebuild the imgui style from the Settings window state. A fresh
   ImGuiStyle every time: ScaleAllSizes() is lossy (it rounds every value
   to an integer), so it must scale the unscaled defaults, not the
   previous scale. The fresh default is already dark, so only light and
   classic need their colors applied. FontScaleDpi scales the fonts too;
   imgui 1.92+ sizes fonts dynamically, so no atlas rebuild is needed.
   The themes ship semi-transparent window surfaces (WindowBg alpha
   0.94/0.85), so make those solid -- otherwise the 3D scene still
   seethroughs at full transparency and the slider can't reach opaque. */
void Game::apply_ui_style() {
    ImGuiStyle style;
    if(ui_style == 1) { ImGui::StyleColorsLight(&style); }
    else if(ui_style == 2) { ImGui::StyleColorsClassic(&style); }
    style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    style.Colors[ImGuiCol_PopupBg].w = 1.0f;
    style.ScaleAllSizes(ui_scale);
    style.FontScaleDpi = ui_scale;
    style.WindowRounding = window_rounding;
    style.Alpha = ui_alpha;
    ImGui::GetStyle() = style;
}

/* Push a one-shot on-screen message. The queue is bounded: expired
   entries are dropped lazily here (a push is the only place the queue
   grows), and anything beyond the cap falls off the front. */
void Game::toast(const char *fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    const double now = SDL_GetTicks() * 0.001;
    while(!toasts.empty() && now - toasts.front().born >= kToastLife) {
        toasts.erase(toasts.begin());
    }
    toasts.push_back(ToastMsg{buf, now});
    if(toasts.size() > (size_t)kToastVisible * 2) {
        toasts.erase(toasts.begin());
    }
}

/* Switch the active (controlled) ship. The ship being left is released:
   throttle zeroed, armed thrust + rotation commands cleared, and it parks
   on rails (coasting or frozen) if it can. The ship being taken re-enters
   physics. Taking control during rails warp drops the warp to 10 (the top
   physics warp -- anything above is a rails warp) so the active ship is
   integrated. The orbit camera recenters on the ship being taken. */
void Game::select_ship(int idx) {
    if(idx < 0 || idx >= (int)ships.size() || idx == activeIdx) { return; }
    ships[activeIdx]->releaseControl();
    ships[activeIdx]->goOnRails();
    activeIdx = idx;
    ships[activeIdx]->leaveRails();
    ship = ships[activeIdx];
    if(time_accel >= kRailsWarp) {
        time_accel = 10;
        toast("Active ship: %s, warp 10x", ship->name.c_str());
    }
    focusBody = 0;   // back to the "ship" focus target
    if(camera->mode == CAM_ORBIT) {
        camera->Follow(ship->get_center_of_mass());
        camera->distance = 50.0;
    }
    printf("Active ship %d of %d: %s\n",
           activeIdx + 1, (int)ships.size(), ship->name.c_str());
}

/* Enter rails warp: park every ship (flying ones coast on their conic,
   grounded ones freeze on the ground). Refuses -- and keeps the current
   accel -- if any ship is not rail-eligible, e.g. a suborbital descent in
   progress. */
bool Game::enter_rails_warp() {
    for(auto *s : ships) {
        if(!s->canRail()) {
            printf("Rails warp refused: '%s' is neither in free fall nor "
                   "grounded (warp stays %d)\n", s->name.c_str(), time_accel);
            toast("Rails warp refused: '%s' is not in free fall",
                  s->name.c_str());
            return false;
        }
    }
    for(auto *s : ships) { s->goOnRails(); }
    return true;
}

/* Remove a ship + its bookkeeping. The Vehicle dtor detaches the welds
   and unregisters the bodies (skipped when the ship is already parked on
   rails), so this is safe in any state. Refuses to remove the last ship.
   If the removed ship was active, control hands off to the next ship in
   the list (or the last one). */
void Game::remove_ship(int idx) {
    if(idx < 0 || idx >= (int)ships.size()) { return; }
    if(ships.size() <= 1) {
        printf("Refusing to remove the last ship\n");
        return;
    }
    Vehicle *v = ships[idx];
    const bool wasActive = (idx == activeIdx);
    const std::string removedName = v->name;
    if(wasActive) { v->releaseControl(); }

    ships.erase_ship((size_t)idx);   // erase the 4 fleet vectors + delete v

    if(wasActive) {
        activeIdx = (idx >= (int)ships.size()) ? (int)ships.size() - 1 : idx;
        ships[activeIdx]->leaveRails();
        ship = ships[activeIdx];
        if(time_accel >= kRailsWarp) {
            time_accel = 10;
            toast("Active ship: %s, warp 10x", ship->name.c_str());
        }
        focusBody = 0;
        if(camera->mode == CAM_ORBIT) {
            camera->Follow(ship->get_center_of_mass());
            camera->distance = 50.0;
        }
        printf("Removed '%s'; active ship %d of %d: %s\n",
               removedName.c_str(), activeIdx + 1, (int)ships.size(),
               ship->name.c_str());
    } else {
        if(idx < activeIdx) { activeIdx--; }
        printf("Removed '%s' (active unchanged: %s)\n",
               removedName.c_str(), ship->name.c_str());
    }
}
