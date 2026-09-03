// game.cpp -- the control transitions of the running game (declared in
// game.h): switching / removing the active ship, entering rails warp, the
// UI window toggle and the orbit-camera focus resolution. These were
// lambdas in main(); they touch the fleet (Ships) and the runtime state
// (Game), so they live with the state.
#include "game.h"

#include <algorithm>
#include <cstdarg>
#include <cmath>
#include <cstdio>
#include <string>

#include "eva.h"      // Kerbal (the crew characters)
#include "physics.h"  // SetMass, AddPhysicsBody, RemoveBody, setPosRot, GetPosition
#include "pick.h"     // pickShipPart (pickAt)
#include "shipdef.h"  // PartDef (crew_capacity)

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
    // 2x2 grid of plots: wider than the old two-stacked-plots layout so the
    // two columns have room (each cell is ~half this width).
    o_telemetry.initial_size = ImVec2(880.0f, 620.0f);
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
    // Surface Map: the body's 2-D surface (equirectangular) with the ship's
    // position + orbit overlaid and (optionally) the terminator baked in.
    // 256 x 128 default map + the combo + readouts fits in 520 x 430.
    o_surfmap = info_opts(ui::Slot::Center);
    o_surfmap.initial_size = ImVec2(520.0f, 430.0f);
    o_surfmap.default_open = false;
    // --surfmap-noshade (CLI) mirrors the window's "Sun shading" box.
    surfmap_shade = !args.surfmap_noshade;
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
        ui_windows.push_back(UiWin{name, label, o, true});
    };
    // In the registry (the TAB toggle covers it) but out of the Windows
    // list: it's toggled from its own context (the Transfer window, the
    // main menu) instead.
    auto add_ui_window_hidden = [&](const char *name, const char *label,
                                    const ui::Options &o) {
        ui_windows.push_back(UiWin{name, label, o, false});
    };
    add_ui_window("Resources", "Resources", o_resources);
    add_ui_window("Orbital", "Orbit Info", o_orbit);
    add_ui_window("Orbital Map", "Orbit Map", o_map);
    add_ui_window("Surface", "Surface Info", o_surface);
    // Surface Map sits under Surface Info, mirroring Orbit Info -> Orbit Map.
    add_ui_window("Surface Map", "Surface Map", o_surfmap);
    add_ui_window("Vessel Info", "Vessel Info", o_vessel);
    add_ui_window("Ship Parts", "Vessel Parts", o_parts);
    // Registered regardless of fleet size: the window (ship list + spawn)
    // is always drawn, so it always needs the toggle + checkbox.
    add_ui_window("Ship List", "Ship List", o_ships);
    add_ui_window("Autopilot", "Autopilot", o_autopilot);
    add_ui_window("Game Debug Info", "Game Debug Info", o_debug);
    add_ui_window("Telemetry", "Telemetry", o_telemetry);
    add_ui_window("Transfer", "Transfer", o_transfer);
    // Porkchop: toggled from the Transfer window (pick a target, then
    // open the plot for it).
    add_ui_window_hidden("Porkchop", "Porkchop", o_porkchop);
    // Settings and Controls: main-menu only.
    add_ui_window_hidden("Settings", "Settings", o_settings);
    add_ui_window_hidden("Controls", "Controls", o_controls);
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

/* Part windows: open (or focus) the window for a picked part. Picking
   the same part again just re-focuses the existing window -- the player
   wants ONE window per part (several parts can be open at once). */
void Game::openPartWindow(Vehicle *ship, size_t part, const glm::dvec3 &point) {
    for(auto &sel : part_sels) {
        if(sel.ship == ship && sel.part == part) { return; }
    }
    PartSel sel;
    sel.ship = ship;
    sel.part = part;
    sel.t = time;
    sel.point = point;
    part_sels.push_back(sel);
}

void Game::dropPartWindowsFor(Vehicle *ship) {
    part_sels.erase(std::remove_if(part_sels.begin(), part_sels.end(),
                   [ship](const PartSel &p) { return p.ship == ship; }),
                   part_sels.end());
}

/* The RMB-click entry point: pick the part under the cursor, open its
   window, and log it. The [pick] line doubles as the e2e assertion. */
void pickAt(Game &g, int px, int py) {
    Vehicle *ship = nullptr;
    size_t part = 0;
    PickBodyHit hit;
    if(pickShipPart(g, px, py, ship, part, hit)) {
        g.openPartWindow(ship, part, hit.point);
        printf("[pick] t=%.1f ship=%s part=%zu (%s)\n",
               g.time, ship->name.c_str(), part,
               ship->parts[part]->def->name.c_str());
        fflush(stdout);
    } else {
        printf("[pick] t=%.1f miss px=%d py=%d\n", g.time, px, py);
        fflush(stdout);
    }
}

/* Switch the active (controlled) ship. The ship being left is released:
   throttle zeroed, armed thrust + rotation commands cleared, and it parks
   on rails (coasting or frozen) if it can. The ship being taken re-enters
   physics. Taking control during rails warp drops the warp to 10 (the top
   physics warp -- anything above is a rails warp) so the active ship is
   integrated. The orbit camera recenters on the ship being taken. */
void Game::select_ship(Vehicle *v) {
    if(v == nullptr || v == ship) { return; }
    // An EVA character aboard a ship is not directly controllable: it is
    // parked inside a capsule (out of physics) with its mass folded into that
    // part, so un-parking it here would double-count its mass. EVA it from
    // the capsule part window (or the V key) first.
    if(v->isCrewAboard()) {
        toast("%s is aboard -- EVA it from its capsule first",
              v->name.c_str());
        return;
    }
    ship->releaseControl();
    ship->goOnRails();
    v->leaveRails();
    ship = v;
    if(time_accel >= kRailsWarp) {
        time_accel = 10;
        toast("Active ship: %s, warp 10x", ship->name.c_str());
    }
    focusBody = 0;   // back to the "ship" focus target
    if(camera->mode == CAM_ORBIT) {
        camera->Follow(ship->get_center_of_mass());
        // a kerbal is 1.7 m tall; 50 m would lose it
        camera->distance = ship->isEva() ? 10.0 : 50.0;
    }
    // "N of M" in the canonical order (collectVehicles, ships.h) -- the
    // same order F6 and the Ship List window walk. N = v's position, M = the
    // whole fleet (ships + aboard crew).
    int n = 0, i = 0;
    for(auto *x : collectVehicles(sys)) {
        n++;
        if(x == v) { i = n; }
    }
    printf("Active ship %d of %d: %s\n", i, n, ship->name.c_str());
}

/* --- crew (characters aboard ships; decls at the bottom of game.h). The
   aboard crew live on their ship (Vehicle::crew), so the queries read it
   directly; the free kerbals are the isEva ships in the bodies' lists.
   The transitions move the kerbal's mass onto/off the capsule part and
   park/restore its body (parked = out of the physics world, the same
   railFrozen convention as a grounded railed ship), and move it between
   ship->crew and its SoI body's ship list. */

std::vector<Kerbal *> shipCrew(Vehicle *ship) {
    std::vector<Kerbal *> out;
    for(auto *k : ship->crew) { out.push_back(static_cast<Kerbal *>(k)); }
    return out;
}

std::vector<Kerbal *> partCrew(Vehicle *ship, size_t part) {
    std::vector<Kerbal *> out;
    for(auto *k : ship->crew) {
        Kerbal *kb = static_cast<Kerbal *>(k);
        if(kb->aboardPart == part) { out.push_back(kb); }
    }
    return out;
}

std::vector<Kerbal *> freeKerbals(System &sys) {
    std::vector<Kerbal *> out;
    for(auto *s : collectVehicles(sys)) {
        if(!s->isEva()) { continue; }
        Kerbal *k = static_cast<Kerbal *>(s);
        if(k->aboard == nullptr) { out.push_back(k); }
    }
    return out;
}

/* Take `k` out of its capsule: move its mass off the capsule (the ship
   gets lighter), place it standing / hovering just beside the capsule
   (relative to the capsule part), restore its body to the physics world,
   and hand the player control of it. The kerbal moves from ship->crew to
   the ship's SoI body's ship list (its frame follows the ship's, so its
   pose -- set in the ship's frame -- is integrated in the right frame).
   The capsule part keeps the rest of the ship; the kerbal is now a live
   body the player can fly / walk. */
void Game::kerbalEVA(Kerbal *k) {
    if(!k->isAboard()) {
        toast("EVA: %s is not aboard a ship", k->name.c_str());
        return;
    }
    Vehicle *ship = k->aboard;
    const size_t part = k->aboardPart;
    if(part >= ship->parts.size()) { return; }
    const PartDef *capDef = ship->parts[part]->def;
    Body *cap = ship->parts[part]->body;
    Body *kb = k->parts[0]->body;
    const double kerbalMass = k->parts[0]->body->mass;

    /* move the crew mass off the capsule (the ship gets lighter) */
    cap->mass -= kerbalMass;
    SetMass(cap, cap->mass);

    /* the standing / hover pose beside the capsule: on a surface stand on
       the same floor (the capsule's bottom) just outside its side, in free
       fall hover beside it co-moving. */
    const glm::dvec3 capCom = GetPosition(cap);
    const glm::dvec3 upDir = glm::normalize(capCom);
    const glm::dvec3 refs[3] = { {1,0,0}, {0,1,0}, {0,0,1} };
    int best = 0;
    for(int i = 1; i < 3; i++) {
        if(fabs(glm::dot(refs[i], upDir)) < fabs(glm::dot(refs[best], upDir))) { best = i; }
    }
    const glm::dvec3 tangent =
        glm::normalize(refs[best] - glm::dot(refs[best], upDir) * upDir);
    const glm::dvec3 right = glm::cross(tangent, upDir);
    const glm::dmat3 orient = glm::dmat3(right, tangent, upDir);
    const double offset = capDef->radius + 2.0;
    if(ship->frame->isRotFrame()) {
        const double floorR = std::max(glm::length(capCom) - capDef->height / 2.0,
            (double)ship->m_parent->GetTerrainHeight(glm::vec3(upDir)));
        setPosRot(kb, upDir * (floorR + k->restAlt()) + tangent * offset, orient);
        SetVelocity(kb, glm::dvec3(0.0));
    } else {
        setPosRot(kb, capCom + tangent * offset, orient);
        SetVelocity(kb, GetVelocity(cap));   // co-moving beside the ship
    }

    /* the kerbal now lives beside the ship: same SoI body (its ship list)
       and same frame as the ship. While aboard its frame was set once at
       build time and the pose was bookkeeping; the ship may have moved on
       (or changed SoI) since, so both follow the ship now. */
    k->frame = ship->frame;
    k->m_parent = ship->m_parent;
    for(auto it = ship->crew.begin(); it != ship->crew.end(); it++) {
        if(*it == k) { ship->crew.erase(it); break; }
    }
    if(ship->m_parent != nullptr) { ship->m_parent->ships.push_back(k); }
    k->aboard = nullptr;

    /* back into the physics world (it was parked while aboard) */
    AddPhysicsBody(kb);
    k->onRails = false;
    k->railFrozen = false;

    kerbal = k;
    lastShip = ship;
    select_ship(k);
    toast("EVA: %s", k->name.c_str());
    printf("[crew] t=%.1f EVA: '%s' out of '%s' part %zu\n",
           time, k->name.c_str(), ship->name.c_str(), part);
}

/* Put a free kerbal `k` into the capsule (ship, part): move its mass
   onto the capsule (the ship gets heavier), park its body inside at the
   capsule's COM (out of the physics world), and set its aboard state --
   it moves from its SoI body's ship list to ship->crew (frame/m_parent
   follow the ship, so the parked pose is consistent). Refuses a full
   capsule or a non-capsule part. If the player was controlling the kerbal,
   hand control to the ship it entered. */
void Game::kerbalBoard(Kerbal *k, Vehicle *ship, size_t part) {
    if(k->isAboard()) {
        toast("Board: %s is already aboard", k->name.c_str());
        return;
    }
    if(part >= ship->parts.size()) { return; }
    const PartDef *capDef = ship->parts[part]->def;
    if(capDef->crew_capacity <= 0) {
        toast("Board: part %zu is not a capsule", part);
        return;
    }
    if((int)partCrew(ship, part).size() >= capDef->crew_capacity) {
        toast("Board: capsule full (%d)", capDef->crew_capacity);
        return;
    }
    Body *cap = ship->parts[part]->body;
    Body *kb = k->parts[0]->body;
    const double kerbalMass = k->parts[0]->body->mass;

    /* move the crew mass onto the capsule (the ship gets heavier) */
    cap->mass += kerbalMass;
    SetMass(cap, cap->mass);

    /* park the kerbal inside the capsule (at its COM, out of the world) */
    setPosRot(kb, GetPosition(cap), GetOrient(cap));
    RemoveBody(kb);
    k->onRails = true;
    k->railFrozen = true;
    // Leave its body's ship list (it was a free kerbal there), then follow
    // the ship's SoI + frame for the parked state.
    if(k->m_parent != nullptr) {
        for(auto it = k->m_parent->ships.begin();
            it != k->m_parent->ships.end(); it++) {
            if(*it == k) { k->m_parent->ships.erase(it); break; }
        }
    }
    k->frame = ship->frame;
    k->m_parent = ship->m_parent;
    k->aboard = ship;
    k->aboardPart = part;
    ship->crew.push_back(k);
    if(kerbal == k) { kerbal = nullptr; }
    toast("Board: %s -> %s", k->name.c_str(), ship->name.c_str());
    printf("[crew] t=%.1f Board: '%s' into '%s' part %zu\n",
           time, k->name.c_str(), ship->name.c_str(), part);

    /* the player was controlling the kerbal that just boarded: hand control
       to the ship it entered (it is no longer controllable). */
    if(this->ship == k) {
        lastShip = ship;
        select_ship(ship);
    }
}

/* V: toggle EVA. From a ship: EVA one of its aboard kerbals (the first) and
   take control. From the kerbal: hand control back to the ship the player
   came from. The kerbal stays free either way -- it boards back in via the
   capsule part window's Board button (kerbalBoard). Both directions go
   through select_ship, so the old controller parks on rails and the new
   one re-enters physics. */
void Game::toggle_eva() {
    if(ship->isEva()) {
        if(lastShip != nullptr && !lastShip->isEva()) {
            select_ship(lastShip);
        } else {
            toast("EVA: no ship to return to");
        }
        return;
    }
    // controlling a regular ship: EVA its first aboard kerbal
    std::vector<Kerbal *> crew = shipCrew(ship);
    if(crew.empty()) {
        toast("EVA: no crew aboard %s", ship->name.c_str());
        return;
    }
    kerbalEVA(crew.front());
}

/* Enter rails warp: park every ship (flying ones coast on their conic,
   grounded ones freeze on the ground). Refuses -- and keeps the current
   accel -- if any ship is not rail-eligible, e.g. a suborbital descent in
   progress. */
bool Game::enter_rails_warp() {
    std::vector<Vehicle *> all = collectVehicles(sys);
    for(auto *s : all) {
        if(!s->canRail()) {
            printf("Rails warp refused: '%s' is neither in free fall nor "
                   "grounded (warp stays %d)\n", s->name.c_str(), time_accel);
            toast("Rails warp refused: '%s' is not in free fall",
                  s->name.c_str());
            return false;
        }
    }
    for(auto *s : all) { s->goOnRails(); }
    return true;
}

/* Remove a ship + its bookkeeping. The Vehicle dtor detaches the welds
   and unregisters the bodies (skipped when the ship is already parked on
   rails), so this is safe in any state. Refuses to remove the last ship.
   If the removed ship was active, control hands off to the next ship in
   the canonical order (or the last one). */
void Game::remove_ship(Vehicle *v) {
    if(v == nullptr) { return; }
    std::vector<Vehicle *> all = collectVehicles(sys);
    if(all.size() <= 1) {
        printf("Refusing to remove the last ship\n");
        return;
    }
    // A ship that still carries crew (or the crew themselves, whose mass is
    // folded into a capsule part) can't be removed cleanly: deleting it would
    // dangle their `aboard` pointer or leak the folded mass. EVA the crew
    // out first.
    if(v->isCrewAboard() || !shipCrew(v).empty()) {
        toast("Cannot remove %s -- EVA its crew out first", v->name.c_str());
        return;
    }
    const bool wasActive = (v == ship);
    const std::string removedName = v->name;
    if(wasActive) { v->releaseControl(); }

    dropPartWindowsFor(v);   // its part windows would dangle

    // It is not aboard (guarded above), so it is in its SoI body's ship
    // list: take it out, then delete (the Vehicle dtor detaches the welds
    // + unregisters the bodies + deletes its crew).
    if(v->m_parent != nullptr) {
        for(auto it = v->m_parent->ships.begin();
            it != v->m_parent->ships.end(); it++) {
            if(*it == v) { v->m_parent->ships.erase(it); break; }
        }
    }
    delete v;

    // drop any selection references that dangled off the removed ship
    if(kerbal == v) { kerbal = nullptr; }
    if(lastShip == v) { lastShip = nullptr; }

    if(wasActive) {
        // hand off to the next ship in the canonical order (or the last
        // one if v was last); the aboard crew are not controllable
        Vehicle *next = nullptr;
        bool seen = false;
        for(size_t i = 0; i < all.size(); i++) {
            Vehicle *x = all[i];
            if(x->isCrewAboard()) { continue; }
            if(x == v) { seen = true; continue; }
            if(seen) { next = x; break; }
        }
        if(next == nullptr) {
            for(size_t i = all.size(); i-- > 0; ) {
                if(all[i] == v || all[i]->isCrewAboard()) { continue; }
                next = all[i];
                break;
            }
        }
        if(next != nullptr) {
            next->leaveRails();
            ship = next;
            if(time_accel >= kRailsWarp) {
                time_accel = 10;
                toast("Active ship: %s, warp 10x", ship->name.c_str());
            }
            focusBody = 0;
            if(camera->mode == CAM_ORBIT) {
                camera->Follow(ship->get_center_of_mass());
                camera->distance = 50.0;
            }
            // N/M in the remaining fleet (the canonical order minus v):
            // N = next's position, M = the whole remaining fleet.
            int n = 0, i = 0;
            for(size_t j = 0; j < all.size(); j++) {
                if(all[j] == v) { continue; }
                n++;
                if(all[j] == next) { i = n; }
            }
            printf("Removed '%s'; active ship %d of %d: %s\n",
                   removedName.c_str(), i, n, ship->name.c_str());
        }
    } else {
        printf("Removed '%s' (active unchanged: %s)\n",
               removedName.c_str(), ship->name.c_str());
    }
}
