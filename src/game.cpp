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
#include <fstream>
#include <random>
#include <string>

#include "eva.h"      // Kerbal (the crew characters)
#include "inventory.h" // inventoryRemove / inventoryAdd (phase 4.3/4.4)
#include "physics.h"  // AddPhysicsBody, RemoveBody, setPosRot
#include "pick.h"     // pickShipPart (pickAt)
#include "save.h"     // load_game (Game::loadFrom)
#include "settings.h" // SettingsData + the settings.json JSON mapping
#include "datadir.h"  // settings.json's location (the data directory)
#include "shipdef.h"  // PartDef (crew_capacity)

glm::dvec3 Game::focusWorldPos(int i) const {
    // Render frame: the ship's frame, or the home body's frame when there is
    // no ship (the orbit-view state). A body focus resolves into it.
    Frame *rf = ship ? ship->frame : home->frame;
    if (focusTargets[i].body == nullptr) {
        return ship->get_center_of_mass();   // the "ship" target only exists with a ship
    }
    return focusTargets[i].body->frame->GetPositionRelTo(rf);
}

void Game::apply_ui_visible() {
    const WinSet &set = curScene(*this).wins;
    for(size_t i = 0; i < set.n; i++) {
        const WinDef &w = kWins[set.ids[i]];
        if(w.role != WinRole::Persistent) { continue; }
        ui::SetOpen(w.name, ui_visible && w.opts.default_open);
    }
}

/* TAB: hide the chrome for a clean screenshot, then put it back. Only the
   LIVE scene's Persistent windows are touched. Root is left alone -- that is
   what stops TAB blanking the title screen, which ui::Options::closable alone
   would not do, since it only hides the X button while ui::SetOpen still
   closes the window. Chrome and the Transient windows are skipped here and
   hidden by ui_visible at draw time instead, so their open state survives the
   toggle; the Root menus are not hidden at draw time either. The HUD is an
   ordinary Persistent entry now and no longer needs the special case it had. */
void Game::toggle_windows() {
    ui_visible = !ui_visible;
    apply_ui_visible();
}

void Game::ensure_ui_visible() {
    if(ui_visible) { return; }
    ui_visible = true;
    printf("[ui] TAB-hide dropped at scene entry\n");
    fflush(stdout);
    apply_ui_visible();
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

/* Settings persistence (the JSON mapping is in settings.cpp).
   collect_settings reads the live state (args + Game + PostFX);
   apply_settings writes it back, honoring the CLI mask (args.cli_given:
   the command line beats the file, field by field). load starts from
   collect, so a field the file does not mention keeps its current value. */
static SettingsData collect_settings(Game &g) {
    SettingsData s;
    s.window_mode = (int)g.args.window_mode;
    s.screen_width = g.args.screen_width;
    s.screen_height = g.args.screen_height;
    s.msaa_samples = g.args.msaa_samples;
    s.physics_debug_drawing = g.physics_debug_drawing;
    s.world_drawing = g.world_drawing;
    s.draw_starfield = g.draw_starfield;
    s.draw_skylines = g.draw_skylines;
    for(const std::string &fx : PostFX::Available()) {
        if(g.postfx->IsEnabled(fx)) { s.postfx_enabled.push_back(fx); }
        std::vector<FXParam> params = PostFX::Params(fx);
        if(params.empty()) { continue; }
        for(const FXParam &p : params) {
            s.postfx_params[fx][p.name] = g.postfx->GetParam(fx, p.name);
        }
    }
    s.ui_style = g.ui_style;
    s.window_rounding = g.window_rounding;
    s.ui_alpha = g.ui_alpha;
    s.ui_scale = g.ui_scale;
    s.sfx_volume = g.sfx_volume;
    s.music_volume = g.music_volume;
    s.camFovDeg = g.args.camFovDeg;
    s.terrain_px = g.args.terrain_px;
    s.exhaust_scale = g.args.exhaust_scale;
    s.cam_shake = g.args.cam_shake;
    s.flip_pitch = g.flip_pitch;
    s.flip_yaw = g.flip_yaw;
    s.flip_roll = g.flip_roll;
    s.keybinds = g.binds;
    return s;
}

static void apply_settings_args(const SettingsData &s, GameArgs &args) {
    if(!args.cli_given.window_mode) {
        args.window_mode = static_cast<WindowMode>(s.window_mode);
    }
    if(!args.cli_given.width)  { args.screen_width  = s.screen_width; }
    if(!args.cli_given.height) { args.screen_height = s.screen_height; }
    if(!args.cli_given.msaa)   { args.msaa_samples  = s.msaa_samples; }
    if(!args.cli_given.fov)           { args.camFovDeg     = s.camFovDeg; }
    if(!args.cli_given.terrain_px)    { args.terrain_px    = s.terrain_px; }
    if(!args.cli_given.exhaust_scale){ args.exhaust_scale = s.exhaust_scale; }
    if(!args.cli_given.cam_shake)    { args.cam_shake     = s.cam_shake; }
}

static void apply_settings_game(Game &g, const SettingsData &s) {
    if(!g.args.cli_given.postfx) {
        // The effect set is one CLI unit (--postfx): apply the file's
        // list. Unknown names are never touched -- only Available() names
        // are iterated.
        for(const std::string &fx : PostFX::Available()) {
            const bool on =
                std::find(s.postfx_enabled.begin(), s.postfx_enabled.end(),
                          fx) != s.postfx_enabled.end();
            g.postfx->SetEnabled(fx, on);
        }
        // The effects all exist now (SetEnabled creates them), so the
        // param values land; unknown effect/param names are skipped.
        for(const auto &fx : s.postfx_params) {
            for(const auto &p : fx.second) {
                g.postfx->SetParam(fx.first, p.first, p.second);
            }
        }
    }
    g.physics_debug_drawing = s.physics_debug_drawing;
    g.world_drawing = s.world_drawing;
    g.draw_starfield = s.draw_starfield;
    g.draw_skylines = s.draw_skylines;
    g.ui_style = s.ui_style;
    g.window_rounding = s.window_rounding;
    g.ui_alpha = s.ui_alpha;
    g.ui_scale = s.ui_scale;
    g.sfx_volume = s.sfx_volume;
    g.music_volume = s.music_volume;
    // Push to the Audio module (no-op while it is disabled). Runs at boot,
    // after audio.init() + setMusic(): the music track starts at the saved
    // level instead of the default 0.5 and then fading to it.
    g.audio.setSfxVolume(s.sfx_volume);
    g.audio.setMusicVolume(s.music_volume);
    g.flip_pitch = s.flip_pitch;
    g.flip_yaw = s.flip_yaw;
    g.flip_roll = s.flip_roll;
    g.binds = s.keybinds;
}

/* Startup phase 1 (main, before the Renderer): the file's args fields
   must reach the window creation (the display mode/size + the MSAA count
   are fixed in the GLX visual then), so they apply over the CLI defaults
   here. SettingsData's defaults ARE the CLI defaults, so a field the
   file does not mention lands back on the same value. No-op when the
   file is absent. */
void load_settings_args(GameArgs &args) {
    SettingsData s;
    if(!settings_load_file(s)) { return; }
    apply_settings_args(s, args);
}

/* "Save" (the Settings window): the current Settings state to
   settings.json in the data directory (datadir.h). */
bool Game::save_settings() {
    nlohmann::json j;
    settings_write(collect_settings(*this), j);
    datadir::make_dir(datadir::dir());   // a --data-dir whose parents are missing
    std::ofstream f(datadir::settings_file());
    if(!f) { return false; }
    f << j.dump(2) << "\n";
    f.flush();
    return (bool)f;
}

/* Startup phase 2 (main, once the Game exists): the Game + PostFX
   fields. Started from the live state (collect_settings), so a field
   the file does not mention keeps its current value. No-op when the
   file is absent. */
void Game::load_settings() {
    SettingsData s = collect_settings(*this);
    if(!settings_load_file(s)) { return; }
    apply_settings_game(*this, s);
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
void Game::openPartWindow(Vehicle *ship, size_t part, const glm::dvec3 &point,
                          int mx, int my) {
    for(auto &sel : part_sels) {
        if(sel.ship == ship && sel.part == part) { return; }
    }
    PartSel sel;
    sel.ship = ship;
    sel.part = part;
    sel.t = time;
    sel.point = point;
    sel.mx = mx;
    sel.my = my;
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
        g.openPartWindow(ship, part, hit.point, px, py);
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

/* Keep the "ship" focus entry in sync with the active ship and point the
   camera focus at it -- or at a random non-star body (the title backdrop)
   when there is none. select_ship enters the ship state; load_game can enter
   OR leave it (a save may carry no active ship), so both route through this.

   The orbit camera follows the state change: it keeps its old distance,
   so re-centering must re-scale too (50 m around a ship is space; 2 radii
   around a planet centre frames the title backdrop). Home sits at its own
   frame's origin -- the render frame in the no-ship state -- so a backdrop
   body resolves into it via focusWorldPos. */
void Game::syncShipFocus() {
    if(ship != nullptr) {
        if(focusTargets.empty() || focusTargets[0].body != nullptr) {
            focusTargets.insert(focusTargets.begin(), { "ship", nullptr });
        }
        focusBody = 0;   // the "ship" focus target
        // camera is null on a --load boot (main.cpp creates it after
        // load_game), and a free camera is the pilot's own pose.
        if(camera != nullptr && camera->mode == CAM_ORBIT) {
            camera->Follow(ship->get_center_of_mass());
            // a kerbal is 0.75 m tall; 50 m would lose it
            camera->distance = ship->isEva() ? 5.0 : 50.0;
        }
    } else {
        if(!focusTargets.empty() && focusTargets[0].body == nullptr) {
            focusTargets.erase(focusTargets.begin());
        }
        // The shipless orbit view (the title backdrop): a random non-star
        // body, not the home planet.
        parkTitleCamera();
    }
}

/* The title-screen backdrop: park the orbit camera on a random non-star
   body, 2 radii out. Purely the menu backdrop -- the gameplay home (ship
   spawn, HUD time, saves) still anchors to `home`. focusTargets must be
   seeded; at every real call site it is (boot after the list is built, and
   at runtime after load_game), but a --load no-ship pass through load_game
   can reach it first, so an empty list is a no-op and the boot call parks
   it for real. */
void Game::parkTitleCamera() {
    if(focusTargets.empty()) { return; }
    int idx = -1;
    bool pinned = false;
    // A --title-body pin selects that body deterministically (a test /
    // visual-regression hook); otherwise the pick is a random non-star body.
    if(!args.title_body.empty()) {
        for(int i = 0; i < (int)focusTargets.size(); i++) {
            if(focusTargets[i].body != nullptr &&
               focusTargets[i].body->name == args.title_body) {
                idx = i; pinned = true; break;
            }
        }
        if(idx < 0) {
            printf("[title] backdrop pin '%s' not found; picking randomly\n",
                   args.title_body.c_str());
            fflush(stdout);
        }
    }
    if(idx < 0) {
        std::vector<int> cand;
        for(int i = 0; i < (int)focusTargets.size(); i++) {
            TerrainBody *b = focusTargets[i].body;
            if(b != nullptr && b != sys.root) { cand.push_back(i); }
        }
        if(cand.empty()) {   // degenerate: no non-star body -- fall back to any
            for(int i = 0; i < (int)focusTargets.size(); i++) {
                if(focusTargets[i].body != nullptr) { cand.push_back(i); break; }
            }
            if(cand.empty()) { return; }   // no body at all: nothing to park on
        }
        static std::mt19937 rng(std::random_device{}());
        std::uniform_int_distribution<size_t> pick(0, cand.size() - 1);
        idx = cand[pick(rng)];
    }
    focusBody = idx;
    TerrainBody *b = focusTargets[idx].body;
    if(camera != nullptr) {
        // Deliberate: the title backdrop is always an orbit view, so this
        // overrides a free-cam pose (same as quitToTitle does for the ship).
        camera->mode = CAM_ORBIT;
        camera->Follow(focusWorldPos(idx));
        // 2 radii from the centre frames the body at ~53 deg for ANY size
        // (Kerbin and Pol look the same scale) -- don't "scale" it further.
        camera->distance = 2.0 * (double)b->radius;
        camera->ComputeView();
    }
    printf("[title] backdrop: %s%s, %.0f m out\n", b->name.c_str(),
           pinned ? " (pinned)" : "", 2.0 * (double)b->radius);
    fflush(stdout);
}

bool Game::newGame() {
    // Any vehicle in the world (not just the active one) means a game is
    // running: a spawned-but-unselected ship, or a crew member aboard a
    // capsule, would otherwise linger in the fresh world.
    if(!collectVehicles(sys).empty()) {
        toast("A game is already running");
        return false;
    }
    // A new game starts in the Space Center with NO ship: the player then goes
    // to the VAB to build and launch the first vessel (vabLaunch ->
    // enterFlight). Nothing to build here -- the fleet stays empty until that
    // launch -- so this is just "the Space Center is now the floor".
    enterSpaceCenter(*this);
    printf("[game] new game: Space Center, no ship\n");
    fflush(stdout);
    toast("New game -- Space Center");
    return true;
}

bool Game::loadFrom(const std::string &dir) {
    try {
        load_game(*this, dir);
    } catch(const std::exception &e) {
        printf("[load] refused %s: %s\n", dir.c_str(), e.what());
        fflush(stdout);
        toast("Load failed: %s", e.what());
        // load_game is transactional, so the fleet is intact and there is
        // nothing to do but stay put. The test is belt and braces: if a future
        // change ever lets a load fail after the fleet is gone, this routes to
        // the title screen instead of leaving a Flight scene with no vessel.
        if(ship == nullptr) { enterTitle(*this); }
        return false;
    }
    // A load is a fresh game: the thrust latch is per-active-ship and is not in
    // the save, so clear it -- a latch engaged elsewhere (the Title shares the
    // flight key map) must not light the loaded ship's engine on the first tick.
    thrust_latched = false;
    if(ship != nullptr) { enterFlight(*this); } else { enterTitle(*this); }
    return true;
}

void Game::unloadGame() {
    /* Tear the running fleet down to the shipless-boot state. ~Vehicle
       detaches the welds, unregisters the physics bodies and deletes the crew
       aboard, so walking each body's ship list and deleting is the whole
       teardown -- the same pattern load_game's commit path uses. Aboard crew
       are NOT in those lists (their ship owns them), so there is no double
       free; a free EVA kerbal IS a top-level entry and is deleted like any
       other vehicle. part_sels holds Part* into the fleet, so it goes first.

       No job drain is needed: every background continuation (surface map,
       porkchop grid, terrain) publishes into Game- or body-level state and
       none dereferences the fleet, and the bodies and the Game both outlive
       this -- so tearing the fleet down cannot dangle an in-flight job. */
    part_sels.clear();
    for(TerrainBody *b : sys.bodies) {
        for(Vehicle *v : b->ships) { delete v; }
        b->ships.clear();
    }
    ship = nullptr;
    kerbal = nullptr;
    lastShip = nullptr;
    // The title screen is an orbit view: force orbit mode (a pilot quitting
    // from free-cam would otherwise keep the free pose) before syncShipFocus
    // drops the "ship" focus entry and re-aims at the title backdrop (a
    // random non-star body).
    if(camera != nullptr) { camera->mode = CAM_ORBIT; }
    syncShipFocus();
    printf("[game] unloaded: fleet torn down, no active vessel\n");
    fflush(stdout);
}

void Game::quitToTitle() {
    unloadGame();
    enterTitle(*this);
}

void Game::settleFleet(Vehicle *active) {
    /* Apply each ship's scenario. Ships sharing a body+scenario group get
       their own slot (20 m apart along the orbit binormal for an orbit start,
       along the pad for a ground one) so they do not spawn on top of each
       other; ships placed with a null scenario are skipped. */
    ships.apply_scenarios(sys);
    /* Idle ships park on rails: flying ones coast on their conic, pad ships
       freeze in the surface frame (their pose rides the planet's spin via the
       render transform). Ships that are neither in free fall nor grounded
       refuse and stay in the physics world. */
    for(auto *b : sys.bodies) {
        for(auto *s : b->ships) {
            if(s != active) { s->goOnRails(); }
        }
    }
}

void Game::select_ship(Vehicle *v) {
    if(v == nullptr || v == ship) { return; }
    // An EVA character aboard a ship is not directly controllable: it is
    // parked inside a capsule (out of the physics world, rail-frozen) and is
    // owned by that ship (its Vehicle::crew), so it is not a free vehicle to
    // drive. EVA it from the capsule part window (or the V key) first.
    // (phase 3: the old "its mass is folded into that part" reason is gone --
    // the mass is derived through the containment edge -- but a parked,
    // ship-owned kerbal still cannot be selected directly.)
    if(v->isCrewAboard()) {
        toast("%s is aboard -- EVA it from its capsule first",
              v->name.c_str());
        return;
    }
    // The old active ship (null when launching from the orbit-view state,
    // where there was no ship to release).
    if(ship != nullptr) {
        ship->releaseControl();
        ship->goOnRails();
    }
    v->leaveRails();
    ship = v;
    // The thrust latch is per-active-ship: a new ship starts with thrust
    // released (the user re-latches if they want it on this ship).
    thrust_latched = false;
    if(time_accel >= kRailsWarp) {
        time_accel = 10;
        toast("Active ship: %s, warp 10x", ship->name.c_str());
    }
    // A ship is active now: the "ship" focus target may be absent (an
    // orbit-view boot), so sync it in at index 0 -- and re-center + re-scale
    // the orbit camera onto the new ship.
    syncShipFocus();
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

std::vector<Kerbal *> partCrew(Part *capPart) {
    /* step 2.4: read the containment edge (capPart->contents) instead of
       scanning the ship's crew by aboardPart. A crew member's owner is the
       character (checkPartInvariants guarantees isEva), so the cast is safe.
       phase 4: contents may also hold inventory items -- they are in the
       container's ownedContents and their owner is the carrier, NOT a
       Kerbal, so they are skipped. */
    std::vector<Kerbal *> out;
    for(Part *p : capPart->contents) {
        if(p->ownedBy(capPart)) { continue; }
        out.push_back(static_cast<Kerbal *>(p->owner));
    }
    return out;
}

std::vector<Kerbal *> freeKerbals(System &sys) {
    std::vector<Kerbal *> out;
    for(auto *s : collectVehicles(sys)) {
        if(!s->isEva()) { continue; }
        Kerbal *k = static_cast<Kerbal *>(s);
        if(k->aboard() == nullptr) { out.push_back(k); }
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
    Part *capPart = k->aboardPart;
    Vehicle *ship = capPart->owner;
    const PartDef *capDef = capPart->def;
    Body *kb = k->hull;

    /* the standing / hover pose beside the capsule: on a surface stand on
       the same floor (the capsule's bottom) just outside its side, in free
       fall hover beside it co-moving. */
    const glm::dvec3 capCom = ship->partPos(capPart);
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
        k->placeShipAtCom(upDir * (floorR + k->restAlt()) + tangent * offset, orient);
        SetVelocity(kb, glm::dvec3(0.0));
    } else {
        k->placeShipAtCom(capCom + tangent * offset, orient);
        SetVelocity(kb, ship->partVel(capPart));   // co-moving beside the ship
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
    k->aboardPart = nullptr;
    /* step 2.4: clear the containment edge (the kerbal leaves the capsule) --
       both directions, so checkPartInvariants still holds. */
    for(auto it = capPart->contents.begin(); it != capPart->contents.end(); it++) {
        if(*it == k->parts[0]) { capPart->contents.erase(it); break; }
    }
    k->parts[0]->container = nullptr;
    /* phase 3: the ship's mass no longer comes from a baked capsule body
       (addPartMass is gone) -- it is the capsule's effectiveMass, which just
       lost the kerbal through the edge. Rebuild now (one-off, large) so the
       next physics step sees the lighter ship; the edge is cleared first, so
       checkPartInvariants inside the rebuild still holds. */
    ship->rebuildCompound();

    /* back into the physics world (it was parked while aboard) */
    AddPhysicsBody(kb);
    k->onRails = false;
    k->railFrozen = false;

    kerbal = k;
    lastShip = ship;
    select_ship(k);
    toast("EVA: %s", k->name.c_str());
    /* log the capsule's slot in the ship's part list (e2e 29 pins "part 0"):
       the state is a Part* now, so map it back to its index for the line. */
    int capIdx = 0;
    for(size_t i = 0; i < ship->parts.size(); i++) {
        if(ship->parts[i] == capPart) { capIdx = (int)i; break; }
    }
    printf("[crew] t=%.1f EVA: '%s' out of '%s' part %d\n",
           time, k->name.c_str(), ship->name.c_str(), capIdx);
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
    Part *capPart = ship->parts[part];
    const PartDef *capDef = capPart->def;
    if(capDef->crew_capacity <= 0) {
        toast("Board: part %zu is not a capsule", part);
        return;
    }
    if((int)partCrew(capPart).size() >= capDef->crew_capacity) {
        toast("Board: capsule full (%d)", capDef->crew_capacity);
        return;
    }
    Body *kb = k->hull;

    /* park the kerbal inside the capsule (at its COM, out of the world) */
    glm::dvec3 capPos; glm::dmat3 capRot;
    ship->partWorldPose(capPart, capPos, capRot);
    k->placeShipAtCom(capPos, capRot);
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
    k->aboardPart = capPart;
    ship->crew.push_back(k);
    /* step 2.4: register the containment edge (the kerbal's part is parked in
       the capsule, both directions). Vehicle::crew stays the sole owner;
       contents is a non-owning back-reference (2.1). */
    capPart->contents.push_back(k->parts[0]);
    k->parts[0]->container = capPart;
    /* phase 3: the ship's mass is the capsule's effectiveMass, which just
       gained the kerbal through the edge (no more addPartMass bake). Rebuild
       now (one-off, large) so the next physics step sees the heavier ship. */
    ship->rebuildCompound();
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

/* phase 4.4: drop an inventory item -- it becomes a free 1-part ship. */
Vehicle *Game::dropItem(Part *item) {
    if(item == nullptr || item->container == nullptr) { return nullptr; }
    Vehicle *carrier = item->container->owner;
    if(carrier == nullptr) { return nullptr; }
    /* a parked carrier (on rails, or an aboard kerbal's frozen hull) has a
       stale hull state, and its pose may sit inside another ship's hull --
       the item would spawn in collision or with the wrong velocity. Only a
       live carrier can shed items. */
    if(carrier->onRails) { return nullptr; }

    /* the item's world pose = its container's pose (it sits at the
       container's COM). Rigid velocity: v + w x r (the same derivation
       as extractSubtreeAsShip's vOut, for a 1-part drop where root == COM). */
    glm::dvec3 itemPos; glm::dmat3 itemRot;
    carrier->partWorldPose(item->container, itemPos, itemRot);
    const glm::dvec3 com = carrier->comPos();
    const glm::dvec3 v = GetVelocity(carrier->hull)
                       + glm::cross(GetAngVelocity(carrier->hull),
                                   itemPos - com);

    /* remove from the container (ownership + traversal) */
    inventoryRemove(item);

    /* build the 1-part ship */
    Vehicle *nv = new Vehicle();
    nv->name = item->def->display_name.empty() ? item->def->name : item->def->display_name;
    nv->defPath = "";
    nv->m_parent = carrier->m_parent;
    nv->frame = carrier->frame;
    nv->home = carrier->home;
    nv->sun = carrier->sun;
    nv->parts.push_back(item);
    /* the whole inventory subtree (a crate in a crate) now rides the new
       ship, not just the top item */
    inventorySetOwner(item, nv);
    nv->finalize();
    nv->placeShip(itemPos, itemRot);
    nv->setVelocity(v);
    SetAngVelocity(nv->hull, GetAngVelocity(carrier->hull));
    nv->enterWorld();
    if(nv->m_parent != nullptr) { nv->m_parent->ships.push_back(nv); }
    toast("Dropped %s", item->def->name.c_str());
    return nv;
}

/* phase 4.4: pick up a dropped item -- re-parent it into the container. */
bool Game::pickUpItem(Vehicle *itemShip, Part *dest) {
    if(itemShip == nullptr || dest == nullptr) { return false; }
    if(itemShip->parts.size() != 1) { return false; }
    Part *item = itemShip->parts[0];
    if(!dest->isContainer()) { return false; }

    /* re-parent into the destination container */
    if(!inventoryAdd(item, dest)) {
        toast("Pick up: %s is full", dest->def->name.c_str());
        return false;
    }

    /* remove the item ship from the fleet + destroy it. The item itself is
       now owned by dest (in its ownedContents) and still in itemShip's
       parts list -- so clear the list BEFORE the delete: ~Vehicle deletes
       its parts, and leaving the item in would free it here AND again when
       ~Part(dest) runs. */
    if(itemShip->m_parent != nullptr) {
        for(auto it = itemShip->m_parent->ships.begin();
            it != itemShip->m_parent->ships.end(); it++) {
            if(*it == itemShip) { itemShip->m_parent->ships.erase(it); break; }
        }
    }
    // Clear every bookkeeping reference that points at the doomed ship so the
    // next tick doesn't dereference it (mirror remove_ship's pre-delete
    // guards). Picking up the ship you are flying leaves orbit-view; handing
    // control to the carrier is the pickup UI's job (4.4 wiring).
    if(ship == itemShip) { ship->releaseControl(); ship = nullptr; }
    if(lastShip == itemShip) { lastShip = nullptr; }
    if(kerbal == itemShip) { kerbal = nullptr; }
    dropPartWindowsFor(itemShip);
    for(auto *s : collectVehicles(sys)) {
        if(s->dockTargetShip == itemShip) { s->dockTargetShip = nullptr; s->dockTargetPort = nullptr; }
    }
    itemShip->parts.clear();
    RemoveBody(itemShip->hull);
    delete itemShip;
    /* phase 3: the carrier's compound gains the item's mass */
    if(dest->owner != nullptr) { dest->owner->rebuildCompound(); }
    toast("Picked up %s", item->def->name.c_str());
    return true;
}

/* V: toggle EVA. From a ship: EVA one of its aboard kerbals (the first) and
   take control. From the kerbal: hand control back to the ship the player
   came from. The kerbal stays free either way -- it boards back in via the
   capsule part window's Board button (kerbalBoard). Both directions go
   through select_ship, so the old controller parks on rails and the new
   one re-enters physics. */
void Game::toggle_eva() {
    if(ship == nullptr) {
        // orbit-view state: there is no ship (hence no kerbal) to EVA.
        toast("EVA: no active ship");
        return;
    }
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
    // On rails the engines are off (the ships coast on their conics), so the
    // thrust latch -- which keeps the active ship's engines lit -- no longer
    // applies. Clear it so it doesn't re-engage thrust the moment the warp is
    // dropped back to physics.
    thrust_latched = false;
    return true;
}

/* Proximity activation: keep the ships close to the active ship live (in
   the physics world) so they can interact, and park the ones that are not.
   The active ship is either grounded or flying (inTerrainBand); the two
   regimes use very different radii -- a few tens of metres on the pad, a few
   kilometres in orbit. An engaged ship that is parked while the active ship
   is on rails (rails warp) wakes the active ship and caps the accel, so a
   close approach always drops out of warp into live physics. A ground
   engage radius of 0 disables auto-waking grounded neighbors (they wake only
   when you switch to them). */
void Game::updateProximity() {
    Vehicle *a = ship;
    if(a == nullptr) { return; }

    const bool grounded = a->inTerrainBand();
    const double r_on  = grounded ? args.prox_ground_on  : args.prox_fly_on;
    const double r_off = grounded ? args.prox_ground_off : args.prox_fly_off;

    // Reused scratch (collectVehiclesInto): this runs every tick.
    static thread_local std::vector<Vehicle *> all;
    collectVehiclesInto(sys, all);
    bool any_engaged = false;
    for(auto *s : all) {
        if(s == a) { continue; }
        if(s->isCrewAboard()) { continue; }   // crew riding in a ship is not a target
        const double d = a->distanceTo(s);
        if(s->onRails && r_on > 0.0 && d < r_on) {
            s->releaseControl();   // no armed commands once it's live
            s->leaveRails();
            /* Re-express the woken ship in its SOI's rotating frame NOW,
               while the frame transforms are still at last tick's epoch
               (updateProximity runs before UpdateOrbitRails). leaveRails
               leaves it in the inertial node holding its stale rail_pos --
               this tick's railsTick has not run for it -- and the per-ship
               switchFrames below runs AFTER the frame has advanced, which
               would rotate that stale pose into the new frame and land the
               ship frame_velocity*dt (~4 m at LEO) off the live ships. Doing
               the conversion here keeps every ship's pose on one epoch. */
            s->switchFrames();
            if(args.prox_log) { printf("[prox] t=%.3f %s ENGAGED at %.1f m (< %.1f m, %s)\n",
                                       time, s->name.c_str(), d, r_on,
                                       grounded ? "ground" : "fly"); }
        } else if(!s->onRails && d > r_off) {
            s->goOnRails();
            if(args.prox_log) { printf("[prox] t=%.3f %s RELEASED at %.1f m (> %.1f m, %s)\n",
                                       time, s->name.c_str(), d, r_off,
                                       grounded ? "ground" : "fly"); }
        }
        if(!s->onRails) { any_engaged = true; }
    }

    if(any_engaged && a->onRails) {
        a->leaveRails();
        a->switchFrames();   // same epoch-consistency as the neighbor wake
        if(args.prox_log) { printf("[prox] t=%.3f active %s WOKEN from rails\n",
                                   time, a->name.c_str()); }
    }
    if(any_engaged && time_accel > args.prox_warp) {
        time_accel = args.prox_warp;
        toast("Close approach: time accel limited to %dx", (int)args.prox_warp);
        if(args.prox_log) { printf("[prox] t=%.3f warp CAPPED to %d\n",
                                   time, (int)args.prox_warp); }
    }

    if(args.prox_log) {
        static double last_snap = -1e30;
        if(time - last_snap >= 0.5) {
            last_snap = time;
            printf("[prox] t=%.3f regime=%s r_on=%.0f r_off=%.0f\n",
                   time, grounded ? "ground" : "fly", r_on, r_off);
            for(auto *s : all) {
                if(s == a) { continue; }
                printf("[prox]   %s dist=%.1f m %s\n", s->name.c_str(),
                       a->distanceTo(s), s->onRails ? "on-rails" : "LIVE");
            }
        }
    }
}

/* Docking: INTENT-driven. The active ship mates only when the player has
   BOTH armed one of its own docking ports (right-click the port -> "Arm for
   docking", Vehicle::dockArmPort) AND targeted a port on another ship (->
   "Target for docking", Vehicle::dockTarget*). Checked once per tick at the
   boundary (tick.cpp, after the physics substeps): the armed port against
   the targeted port -- the two port face-centres within kDockCapture, each
   port axis within kDockAlign of the line between the ports, and the port
   points' relative speed under kDockMaxV at capture. The armed port is the
   one that mates (no best-fit scan -- the player picked which of the ship's
   ports this dock uses). The active ship is the survivor (it absorbs the
   target's ship); the joint is recorded as a seam on it. The intent is
   consumed (cleared) on success, so an undock cannot immediately re-dock --
   the player has to re-arm and re-target. At most one dock per tick. */
void Game::updateDocking() {
    Vehicle *a = ship;
    if(a == nullptr || a->isEva() || a->onRails) { return; }
    /* No intent, no dock: the ship must have a targeted port AND an armed
       port of its own (the two halves of the intent). */
    if(a->dockTargetPort == nullptr || a->dockTargetShip == nullptr
       || a->dockArmPort == nullptr) { return; }
    Vehicle *b = a->dockTargetShip;
    /* Validate the intent: the target must be another live ship that still
       carries that docking port (else it went stale -- the ship was removed
       or the port staged away -- so drop it). A ship is only ever deleted
       through updateDocking / remove_ship, both of which drop targets that
       point at it, so b is live here. */
    bool targetOk = (b != a && !b->isEva() && !b->isCrewAboard());
    if(targetOk) {
        targetOk = false;
        for(Part *p : b->parts) {
            if(p == a->dockTargetPort && p->isDockingPort()) { targetOk = true; break; }
        }
    }
    if(!targetOk) { a->dockTargetShip = nullptr; a->dockTargetPort = nullptr; return; }
    /* Parked on rails or in another frame: cannot mate this tick. Keep the
       intent and wait -- proximity wakes the target when it is near, and a
       frame switch brings it into this ship's frame. */
    if(b->onRails || b->frame != a->frame) { return; }

    Part *pb = a->dockTargetPort;
    /* The armed port is the one that mates -- the player picked which of
       this ship's docking ports this dock uses, so there is no best-fit
       scan. */
    Part *bestA = a->dockArmPort;
    /* Validate the arm: it must still be a live docking port on this ship
       (else it went stale -- staged away -- and the intent can't be
       fulfilled). Drop the dangling arm; keep the target so a re-arm is all
       the player needs to retry. */
    bool armOk = false;
    for(Part *p : a->parts) {
        if(p == bestA && p->isDockingPort()) { armOk = true; break; }
    }
    if(!armOk) { a->dockArmPort = nullptr; return; }

    glm::dvec3 paP, pbP;
    glm::dmat3 paR, pbR;
    a->partWorldPose(bestA, paP, paR);
    b->partWorldPose(pb, pbP, pbR);
    const glm::dvec3 dir = pbP - paP;
    const double d = glm::length(dir);
    if(d < 1e-9) { return; }
    const glm::dvec3 n = dir / d;
    /* Each port presents the face closest to the other part:
       its +Z face if the other is on that side, else its -Z. */
    const double sA = (glm::dot(paR[2], dir) >= 0.0) ? 1.0 : -1.0;
    const double sB = (glm::dot(pbR[2], paP - pbP) >= 0.0) ? 1.0 : -1.0;
    const glm::dvec3 axisA = paR[2] * sA;
    const glm::dvec3 axisB = pbR[2] * sB;
    if(glm::dot(axisA, n) < kDockAlign) { return; }
    if(glm::dot(axisB, -n) < kDockAlign) { return; }
    const glm::dvec3 faceA = paP + axisA * (bestA->def->height * 0.5);
    const glm::dvec3 faceB = pbP + axisB * (pb->def->height * 0.5);
    const glm::dvec3 rv = a->partVel(bestA) - b->partVel(pb);
    const double bestD = glm::length(faceA - faceB);
    if(bestD >= kDockCapture) { return; }
    const double bestV = glm::length(rv);
    if(bestV > kDockMaxV) { return; }

    const std::string bName = b->name;
    a->absorbShip(b, bestA);
    a->dockTargetShip = nullptr;   // the intent is consumed by the dock
    a->dockTargetPort = nullptr;   // (both halves: re-arm and re-target on
    a->dockArmPort = nullptr;      //  a redock)
    dropPartWindowsFor(b);
    if(b->m_parent != nullptr) {
        for(auto it = b->m_parent->ships.begin(); it != b->m_parent->ships.end(); it++) {
            if(*it == b) { b->m_parent->ships.erase(it); break; }
        }
    }
    if(kerbal == b) { kerbal = nullptr; }
    if(lastShip == b) { lastShip = nullptr; }
    /* Any other ship that had targeted b now dangles -- drop its intent. */
    for(auto *s : collectVehicles(sys)) {
        if(s->dockTargetShip == b) { s->dockTargetShip = nullptr; s->dockTargetPort = nullptr; }
    }
    toast("Docked with %s", bName.c_str());
    printf("[dock] t=%.3f a=\"%s\" b=\"%s\" d=%.3f m v=%.3f m/s\n",
           time, a->name.c_str(), bName.c_str(), bestD, bestV);
    delete b;
}

/* Undock: split the most recent seam off the active ship. The other side
   (the subtree under the seam's root) is extracted into a new ship via the
   general Vehicle::extractSubtreeAsShip primitive (the same call a future
   "dropped stage becomes a ship" will make) and returned to the fleet.
   One-shot (the handler in events.cpp). */
void Game::undock() {
    Vehicle *a = ship;
    if(a == nullptr || a->isEva()) { return; }
    if(a->seams.empty()) {
        toast("Nothing docked to undock");
        return;
    }
    /* Undock needs the parts in the physics world: wake a ship parked on
       rails first (like staging does). */
    if(a->onRails) {
        a->leaveRails();
        if(time_accel >= kRailsWarp) {
            time_accel = 1;
            toast("Undock: left the rails, warp 1x");
        }
    }
    Vehicle::DockSeam seam = a->seams.back();
    Vehicle *out = a->extractSubtreeAsShip(seam.root, seam.name);
    if(out == nullptr) {
        toast("Cannot undock");
        return;
    }
    out->enterWorld();   // the split leaves world registration to the caller
    /* The undocked seam is already gone: extractSubtreeAsShip drops a seam
       split across the cut (the port stays on this ship, the docked ship
       leaves), so there is nothing left to pop -- popping here would drop the
       WRONG seam (the one just before it). */
    /* part windows on the survivor address parts by index, which just
       shifted -- drop them rather than dangle. */
    dropPartWindowsFor(a);
    if(out->m_parent != nullptr) {
        out->m_parent->ships.push_back(out);
    }
    toast("Undocked %s", seam.name.c_str());
    printf("[undock] t=%.3f a=\"%s\" b=\"%s\"\n", time, a->name.c_str(), seam.name.c_str());
}

/* Stage: fire the active stage's decouplers. Each decoupler's child-side
   subtree comes off as a SEPARATE ship (the same extractSubtreeAsShip
   primitive undock uses -- not a delete) and is returned to the fleet, so
   the dropped stages keep flying rather than vanishing (like KSP). The
   survivor stays the active ship and its stage counter steps down so the
   next stage's engines light. One-shot (the SPACE handler in events.cpp).

   A decoupler nested inside another's subtree on the same stage is absorbed
   by the outer one (extracted with it), so the decouplers are fired
   shallowest-first and a decoupler already absorbed into a previous
   extraction (extract returns null) is skipped. */
void Game::stage() {
    Vehicle *a = ship;
    if(a == nullptr || a->isEva()) { return; }
    const int st = a->activeStage();

    /* Staging needs the parts in the physics world: wake a ship parked on
       rails first (like undock). */
    if(a->onRails) {
        a->leaveRails();
        if(time_accel >= kRailsWarp) {
            time_accel = 1;
            toast("Staging: left the rails, warp 1x");
        }
    }

    /* The parts that WOULD come off (the decouplers on this stage plus their
       child-side subtrees, unioned). Refuse if any carries a crewed capsule
       -- the crew is locked to the vessel, so EVA them out first. (The
       survivor keeps its crew; a sibling branch sharing the stage is not in
       this set.) */
    const std::vector<Part *> dropped = a->droppedPartsAtStage(st);
    bool crewOnStage = false;
    for(Part *p : dropped) {
        if(p->def == nullptr || p->def->crew_capacity <= 0) { continue; }
        if(!partCrew(p).empty()) { crewOnStage = true; }
    }
    if(crewOnStage) {
        printf("Stage: refused -- crew aboard the capsule (EVA them first)\n");
        toast("Cannot stage -- EVA the capsule's crew out first");
        return;
    }

    /* The decouplers on this stage (the roots of the dropped subtrees),
       shallowest-first. */
    std::vector<Part *> decs;
    for(Part *p : dropped) { if(p->isDecoupler()) { decs.push_back(p); } }
    int ships = 0, parts = 0;
    if(!decs.empty()) {
        auto depth = [&](Part *p) {
            int d = 0;
            while(p->parent != nullptr) { d++; p = p->parent; }
            return d;
        };
        std::sort(decs.begin(), decs.end(),
                  [&](Part *x, Part *y) { return depth(x) < depth(y); });

        /* De-duplicate a name against the live fleet (first keeps the bare
           name, later ones get #2, #3 ..) -- the same rule Ships::dedupName
           uses, inlined so the new ships register as they split. */
        auto dedup = [&](const std::string &base) -> std::string {
            std::string nm = base;
            int n = 2;
            for(;;) {
                bool taken = false;
                for(Vehicle *s : collectVehicles(sys)) {
                    if(s->name == nm) { taken = true; break; }
                }
                if(!taken) { return nm; }
                nm = base + " #" + std::to_string(n);
                n++;
            }
        };

        for(Part *d : decs) {
            /* Name: the parent ship's name, qualified by the decoupler part
               so several ships from one staging stay distinguishable. */
            std::string base = a->name;
            const std::string qual =
                d->def->display_name.empty() ? d->def->name : d->def->display_name;
            if(!qual.empty()) { base += " " + qual; }
            // The pop: a one-shot "slam" as the part separates. balance 0.4
            // pulls the file's full-scale transient down to sit with the engine
            // hum (the file peaks at 0 dB, and a transient reads louder than a
            // steady loop at the same gain).
            Vehicle *out = a->extractSubtreeAsShip(d, dedup(base));
            if(out == nullptr) { continue; }   // already absorbed into an outer ship
            audio.playOnce("res/qubodup-crash.wav", 0.4f);
            out->enterWorld();
            if(out->m_parent != nullptr) { out->m_parent->ships.push_back(out); }
            ships++;
            parts += (int)out->parts.size();
        }
    }
    /* Step the counter on every press, even a decoupler-less stage (KSP
       semantics, and what the VAB table assumes): a stage number with no
       decoupler must not wedge the lower decouplers out of reach.
       advanceStage() clamps at the lowest stage. */
    a->advanceStage();
    if(ships > 0) {
        /* part windows on the survivor address parts by index, which just
           shifted -- drop them rather than dangle. */
        dropPartWindowsFor(a);
        printf("Stage: dropped %d ship(s) / %d part(s); now on stage %d of %d\n",
               ships, parts, a->activeStage(), a->numStages());
        toast("Staged -- dropped %d ship(s)", ships);
    } else {
        printf("Stage: nothing left to separate\n");
    }
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
    // A ship that still carries crew -- or a crew member themselves -- can't
    // be removed here: an aboard kerbal is owned by its ship (its
    // Vehicle::crew) and parked out of the world, not in this body's ship
    // list, so the removal below would not find it and delete would leave the
    // ship's crew + the capsule's contents dangling; and a crewed ship owns
    // its crew, so deleting it would delete the crew too (kill them) -- a
    // game action we don't support. EVA the crew out first.
    // (phase 3: the old "folded mass / aboard pointer" reasons are gone; the
    // ownership rule above is what keeps this guard.)
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
    // Ships that had v targeted for docking now dangle -- drop their intent
    // (pointer compare only, so it is safe once v is off the lists).
    for(auto *s : collectVehicles(sys)) {
        if(s->dockTargetShip == v) { s->dockTargetShip = nullptr; s->dockTargetPort = nullptr; }
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
            // The pointer compare MUST come first: v was deleted above and
            // `all` still holds it, and isCrewAboard() is virtual -- calling
            // it on the freed entry reads its vptr out of freed memory.
            if(x == v) { seen = true; continue; }
            if(x->isCrewAboard()) { continue; }
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
        } else {
            /* Nothing left to control. Unreachable with today's fleet model
               (a surviving ship or a free kerbal is selectable, and an
               aboard crew character's carrier ship is always in `all` too),
               but the guard above means `ship` would otherwise keep pointing
               at the deleted vehicle: enter the no-ship state instead (the
               same one load_game enters -- syncShipFocus drops the "ship"
               focus entry and re-aims the orbit camera at the title backdrop,
               a random non-star body). */
            ship = nullptr;
            syncShipFocus();
            // Flight's "there is an active vessel" invariant: with nothing
            // left to control the floor is the title screen, not an empty
            // flight scene.
            enterTitle(*this);
            printf("Removed '%s'; nothing left to control -- no active vessel\n",
                   removedName.c_str());
        }
    } else {
        printf("Removed '%s' (active unchanged: %s)\n",
               removedName.c_str(), ship->name.c_str());
    }
}
