// game.cpp -- the control transitions of the running game (declared in game.h).
#include "game.h"

#include <algorithm>
#include <cassert>
#include <cstdarg>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <random>
#include <string>

#include "eva.h"      // Kerbal
#include "inventory.h" // inventoryRemove / inventoryAdd
#include "part.h"     // make_suit (the crew pool's suits)
#include "physics.h"  // AddPhysicsBody, RemoveBody, setPosRot
#include "pick.h"     // pickShipPart (pickAt)
#include "save.h"     // load_game (Game::loadFrom)
#include "settings.h" // SettingsData
#include "datadir.h"  // settings.json's location
#include "shipdef.h"  // PartDef (crew_capacity)
#include "buildings.h" // place_world_buildings
#include "shader.h"   // get_shader
#include "skybox.h"   // Skybox::load
#include "terragen.h" // biomeAt / biomeName
#include "uiwins.h"   // setWinOpen (W_FlightSummary)
#include "calendar.h" // fmt_cal_duration

/* Register a headless test hook (see HookPhase). Boot-time only: fireHooks
   walks the table by index, so an act() that added a hook would either fire
   early or never -- refuse it instead. */
void Game::addHook(HookPhase phase, int ms,
                   std::function<bool(Game &)> gate,
                   std::function<void(Game &)> act) {
    assert((int)phase >= 0 && (int)phase < kHookPhaseCount);
    assert(!hooks_polled && "addHook is boot-time only");
    hooks[(size_t)phase].push_back({ms, false, std::move(gate), std::move(act)});
    hooks_pending[(size_t)phase]++;
}

/* Headless test hooks: each fires at most once, at/after its ms from loop
   start, gate permitting. Registration order within a phase IS the firing
   order (see GameHook). Index walk, not range-for: act() must not add hooks
   (addHook asserts), and the index form stays defined if one ever does. */
void Game::fireHooks(HookPhase phase) {
    if(hooks_pending[(size_t)phase] == 0) { return; }
    hooks_polled = true;
    const int ms = (int)(SDL_GetTicks() - loop_start_ms);
    std::vector<GameHook> &vec = hooks[(size_t)phase];
    for(size_t i = 0; i < vec.size(); i++) {
        GameHook &h = vec[i];
        if(h.fired || ms < h.ms) { continue; }
        if(h.gate && !h.gate(*this)) { continue; }
        h.fired = true;
        hooks_pending[(size_t)phase]--;
        h.act(*this);
    }
}

glm::dvec3 Game::focusWorldPos(int i) const {
    // Render frame: ship's frame, or home's when there is no ship (orbit view).
    Frame *rf = ship ? ship->frame : home->frame;
    if (focusTargets[i].body == nullptr) {
        // A stale "ship" entry with no ship focuses the origin, not null.
        return ship ? ship->get_center_of_mass() : glm::dvec3(0.0);
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

/* TAB: hide the chrome for a clean screenshot. Only the LIVE scene's
   Persistent windows; Root is untouched so TAB cannot blank the title. */
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

/* Fresh ImGuiStyle every time: ScaleAllSizes() is lossy, so it must scale the
   unscaled defaults. WindowBg is forced solid or the alpha slider tops out
   translucent. */
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

/* Settings persistence (JSON mapping in settings.cpp). A field the file omits
   keeps its value; the CLI mask (args.cli_given) beats the file. */
static SettingsData collect_settings(Game &g) {
    SettingsData s;
    s.window_mode = (int)g.args.window_mode;
    s.screen_width = g.args.screen_width;
    s.screen_height = g.args.screen_height;
    s.msaa_samples = g.args.msaa_samples;
    s.physics_debug_drawing = g.physics_debug_drawing;
    s.world_drawing = g.world_drawing;
    s.draw_starfield = g.draw_starfield;
    s.draw_ref_circles = g.draw_ref_circles;
    s.draw_orbit_plane = g.draw_orbit_plane;
    s.sky_dim = g.sky_dim;
    s.sky_dim_cone = g.sky_dim_cone;
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
    if(!args.cli_given.cam_shake)    { args.cam_shake     = s.cam_shake; }
    // Star-field fade lives on the Game but main.cpp seeds it from args.
    if(!args.cli_given.sky_dim)       { args.sky_dim       = s.sky_dim; }
    if(!args.cli_given.sky_dim_cone)  { args.sky_dim_cone  = s.sky_dim_cone; }
}

static void apply_settings_game(Game &g, const SettingsData &s) {
    if(!g.args.cli_given.postfx) {
        // The effect set is one CLI unit (--postfx): apply the file's list.
        for(const std::string &fx : PostFX::Available()) {
            const bool on =
                std::find(s.postfx_enabled.begin(), s.postfx_enabled.end(),
                          fx) != s.postfx_enabled.end();
            g.postfx->SetEnabled(fx, on);
        }
        // SetEnabled creates the effects, so params land; unknown names skip.
        for(const auto &fx : s.postfx_params) {
            for(const auto &p : fx.second) {
                g.postfx->SetParam(fx.first, p.first, p.second);
            }
        }
    }
    g.physics_debug_drawing = s.physics_debug_drawing;
    g.world_drawing = s.world_drawing;
    g.draw_starfield = s.draw_starfield;
    g.draw_ref_circles = s.draw_ref_circles;
    g.draw_orbit_plane = s.draw_orbit_plane;
    g.ui_style = s.ui_style;
    g.window_rounding = s.window_rounding;
    g.ui_alpha = s.ui_alpha;
    g.ui_scale = s.ui_scale;
    g.sfx_volume = s.sfx_volume;
    g.music_volume = s.music_volume;
    // Push to the Audio module (no-op while it is disabled).
    g.audio.setSfxVolume(s.sfx_volume);
    g.audio.setMusicVolume(s.music_volume);
    g.flip_pitch = s.flip_pitch;
    g.flip_yaw = s.flip_yaw;
    g.flip_roll = s.flip_roll;
    g.binds = s.keybinds;
}

/* Startup phase 1 (main, before the Renderer): the file's args fields must
   reach window creation. */
void load_settings_args(GameArgs &args) {
    SettingsData s;
    if(!settings_load_file(s)) { return; }
    apply_settings_args(s, args);
}

// Settings window "Save": live state to settings.json in the data dir.
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

/* Startup phase 2 (main, once the Game exists): the Game + PostFX fields. */
void Game::load_settings() {
    SettingsData s = collect_settings(*this);
    if(!settings_load_file(s)) { return; }
    apply_settings_game(*this, s);
}

/* One-shot on-screen message. The queue is bounded: expired entries drop
   lazily, overflow falls off the front. */
void Game::toast(const char *fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    // Mirror to stdout: the on-screen toast is transient, the log is not.
    printf("[toast] %s\n", buf);
    fflush(stdout);

    const double now = SDL_GetTicks() * 0.001;
    while(!toasts.empty() && now - toasts.front().born >= kToastLife) {
        toasts.erase(toasts.begin());
    }
    toasts.push_back(ToastMsg{buf, now});
    if(toasts.size() > (size_t)kToastVisible * 2) {
        toasts.erase(toasts.begin());
    }
}

// toast() without the on-screen part: console line only, a log/e2e anchor.
void Game::toastLog(const char *fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    printf("[toast] %s\n", buf);
    fflush(stdout);
}

// "A | B | C" for the [ask] line, so a test can assert the choices.
static std::string joinLabels(const std::vector<std::string> &labels) {
    std::string out;
    for(size_t i = 0; i < labels.size(); i++) {
        if(i) { out += " | "; }
        out += labels[i];
    }
    return out;
}

// The log line must not wrap, or an e2e EXPECT can't match the choices.
static std::string oneLine(const std::string &s) {
    std::string out = s;
    for(char &c : out) {
        if(c == '\n' || c == '\r') { c = ' '; }
    }
    return out;
}

/* The card is drawn by the UI pass (drawQuestion); this holds the pending one.
   Mirrored to stdout so tests can see it too. */
void Game::ask(std::string text, std::vector<std::string> labels,
               std::function<void(int choice)> onAnswer) {
    /* A card with no buttons can never be answered: drawQuestion draws the text
       and returns, and ESC has no last choice to pick. */
    assert(!labels.empty());
    question = std::make_unique<Question>();
    question->text = std::move(text);
    question->labels = std::move(labels);
    question->onAnswer = std::move(onAnswer);
    printf("[ask] %s  [%s]\n", oneLine(question->text).c_str(),
           joinLabels(question->labels).c_str());
    fflush(stdout);
}

void Game::answerQuestion(int choice) {
    if(!question) { return; }
    printf("[ask] answered %d\n", choice);
    fflush(stdout);
    // Close before firing: an answer may open the next question.
    auto fn = std::move(question->onAnswer);
    question.reset();
    if(fn) { fn(choice); }
}

// One window per part: re-picking an already-open part is a no-op.
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

void Game::removeVehicle(Vehicle *v) {
    if(std::erase(vehicles, v) == 0) {
        /* retire() only ever gets a vehicle the list holds, and the bulk
           teardowns clear the list wholesale rather than calling this. A miss
           means the list and the world already disagree -- which is the bug,
           not the erase. */
        assert(!"removeVehicle: the vehicle is not in Game::vehicles");
    }
}

/* retire()'s second half: every reference the Game and the transfer planner
   hold to a vehicle that is about to die. A missed one is a use-after-free in a
   later frame -- the planner holds its target list across frames (render.cpp
   rebuilds it, updateShipView reads the previous frame's), so a dangling
   Vehicle* there is read by the map plane and the burn marker long before
   anything notices. List membership is removeVehicle's -- only ui_remove_vehicle
   needs the list intact while it scans for the next ship to hand control to. */
void Game::dropVehicleRefs(Vehicle *v) {
    dropPartWindowsFor(v);
    // Ships that had v targeted for docking now dangle -- drop their intent.
    for(auto *s : vehicles) {
        if(s->dockTargetShip == v) { s->dockTargetShip = nullptr; s->dockTargetPort = nullptr; }
    }
    /* The planner holds v's name pointer + handle until its next rebuild: if v
       was the target, drop the selection so nothing reads it in between. The
       name goes with the index -- xfer_target is derived from it, so leaving it
       would have the next rebuild re-select a vessel that no longer exists. */
    if(xferPlanner.xfer_target >= 0
       && xferPlanner.xfer_target < (int)xferPlanner.xferTargets.size()
       && xferPlanner.xferTargets[xferPlanner.xfer_target].ship == v) {
        xferPlanner.xfer_sel = TargetRef();
        xferPlanner.xfer_target = -1;
        xferPlanner.xfer.valid = false;
        xferPlanner.xfer.burn_dir = glm::dvec3(0.0);
    }
    /* The list holds v's name pointer in its own row, so the row cannot outlive
       it either: anything that scans the list before the next rebuild (the
       --select-target hook does) would read freed memory. Erasing shifts every
       later row, and the index is only this frame's address, so drop it too and
       let the next rebuild re-derive it from the name. */
    const size_t rowsBefore = xferPlanner.xferTargets.size();
    std::erase_if(xferPlanner.xferTargets, [v](const TransferPlanner::XferTarget &t) { return t.ship == v; });
    if(xferPlanner.xferTargets.size() != rowsBefore) { xferPlanner.xfer_target = -1; }
    if(kerbal == v) { kerbal = nullptr; }
    if(lastShip == v) { lastShip = nullptr; }
    /* The active pointer needs the same treatment: a caller that is about to
       hand control to another vessel reads its own saved `wasActive` flag, not
       this, so nulling it here is what keeps select_vehicle from releasing control
       back to a corpse. */
    if(ship == v) { ship = nullptr; }
}

/* The one way a live Vehicle dies (#297).

   This used to be three calls at five sites -- erase, drop refs, delete -- in an
   order each site had to remember. A missed dropVehicleRefs is a use-after-free
   a frame later, and deleting in the middle of an imgui handler frees memory
   that a row of the SAME frame still points at. So the free moves to one sweep
   point and everything else happens here, in one order, once.

   The hull leaves the Bullet world NOW rather than at the sweep: a retired ship
   that stayed registered would still collide on the next substep with the ship
   that just absorbed it or staged it away. ~Vehicle's hullInWorld() guard then
   makes the deferred delete skip the unregister. Nothing else about a retired
   vehicle is reachable -- it is out of Game::vehicles, and every pointer the
   Game and the planner kept across frames was just cleared. */
void Game::retire(Vehicle *v) {
    removeVehicle(v);
    dropVehicleRefs(v);
    // A railed ship is already out of the world; RemoveBody on an unregistered
    // body is UB.
    if(v->hullInWorld()) { RemoveBody(v->hull); }
    /* One line per death, at the moment it stops being a world object rather
       than at the sweep a frame later: this is the point a bug would get wrong,
       and the count is what makes "did every ship actually die?" assertable
       (recover-esc-multi and the dock cases read it). Flushed because a death
       near a crash is exactly the line worth having. */
    printf("[vehicle] retired: '%s' (%zu vehicle(s) left in the world)\n",
           v->name.c_str(), vehicles.size());
    fflush(stdout);
    retired.push_back(v);
}

void Game::sweepRetired() {
    for(Vehicle *v : retired) { delete v; }
    retired.clear();
}

void Game::clearFlightSummary() {
    flightSummary = FlightSummary{};
    setWinOpen(W_FlightSummary, false);
}

/* The named crew pool (Game::roster) becomes standing people: one suit Part per
   profile, parked on the home system's crew pad (crew.h). The pad holds the
   pool, so a system switch empties it and a new game refills it. A suit whose
   def will not load skips its profile rather than ending the mint. */
void Game::mintCrewPool() {
    StaticBuilding *pad = homeCrewPad(sys);
    if(pad == nullptr) {
        printf("[crew] no VAB_LAUNCH_PAD: nowhere to stand the crew pool\n");
        return;
    }
    for(const KerbalProfile &p : roster.profiles) {
        Part *suit = nullptr;
        try {
            suit = make_suit(kKerbalShipDef, parts, partsshader);
        } catch(const std::exception &e) {
            printf("Crew: %s\n", e.what());
            continue;
        }
        suit->kerbal = std::make_unique<KerbalProfile>(p);
        pad->crew.push_back(suit);
    }
    printf("[crew] %zu kerbal(s) standing at '%s'\n",
           pad->crew.size(), pad->id.c_str());
}

/* Situation experiment (identity + provenance) for a science run: localPart's
   COM on poseVehicle (null = the vehicle's own COM) in the body's rotating
   frame. Stars and banded gas giants have no biome. `runnerName` is who ran it
   and `shipName` what they were in ("" = free EVA): a seat occupant has no
   Kerbal object, so provenance travels as names, not pointers. */
bool Game::situationExperiment(TerrainBody *body, Vehicle *poseVehicle,
                               Part *localPart, const std::string &type,
                               const std::string &runnerName,
                               const std::string &shipName, Experiment &out) {
    if(body == nullptr) {
        toast("No experiment: nothing to observe here");
        return false;
    }
    /* A seat occupant has no body of its own: read the live capsule it sits in. */
    const glm::dvec3 localCom = (localPart != nullptr)
        ? poseVehicle->partPos(localPart)
        : poseVehicle->get_center_of_mass();
    Frame *rot = body->frame->getRotFrame();
    const glm::dvec3 sp = poseVehicle->frame->GetOrientRelTo(rot) * localCom
                        + poseVehicle->frame->GetPositionRelTo(rot);
    const double r = glm::length(sp);
    if(r < 1e-9) {
        toast("No experiment: bad position");
        return false;
    }
    const glm::vec3 dir = glm::vec3(sp / r);
    // ASL: above sea level -- same datum as the aero altitude and atmo top.
    const double altAsl = r - ((double)body->radius
                               + (double)body->surface.sea_level);
    // Biome + situation: the two halves of the finding's key.
    const PoseSituation ps = poseSituation(body, dir, altAsl, poseVehicle->isGrounded());
    // A surface body's biome needs measured terrain (max_height).
    if(body->hasClassifiableSurface() && !body->ready) {
        toast("No experiment: terrain not ready");
        return false;
    }
    out = Experiment{};
    out.type = type;
    out.body = body->name;
    out.situation = ps.situation;
    out.biome = (ps.biome != Biome::None) ? biomeName(ps.biome) : "";
    // Availability (ExperimentDef.valid_in): refuse with where it DOES work.
    if(!experimentValidIn(type, out.situation)) {
        std::string where;
        if(const ExperimentDef *d = defFor(type)) {
            for(SciSituation s : d->valid_in) {
                if(!where.empty()) { where += ", "; }
                where += situationName(s);
            }
        }
        if(where.empty()) { where = "no situation (never)"; }   // broken def
        toast("%s only works in: %s", type.c_str(), where.c_str());
        return false;
    }
    // Provenance (the identity key above is what dedups).
    out.ran_at = time;                                    // when the runner ran it
    out.kerbal = runnerName;
    out.ship = shipName;                                  // "" = free EVA
    return true;
}

/* One person runs an observation and stores it in their suit. The walking and
   seated entries below differ only in where the suit, the pose and the name
   come from -- a seated kerbal has no vehicle, so its capsule is the pose
   (part.h). The toast/printf pair lives here so the two cannot drift. */
void Game::runSuitExperiment(Part *suit, Vehicle *poseVehicle, Part *localPart,
                             const std::string &runner,
                             const std::string &shipName) {
    Experiment e;
    if(!situationExperiment(poseVehicle->m_parent, poseVehicle, localPart,
                            "observation", runner, shipName, e)) {
        return;
    }
    if(!suit->addExperiment(e)) {
        // The cap is per-type, not per-biome: say the type.
        toast("Already holding one %s (the suit holds one of each) -- store it first",
              e.type.c_str());
        printf("[science] t=%.1f '%s' already held '%s'\n", time,
               runner.c_str(), experimentName(e).c_str());
        fflush(stdout);
        return;
    }
    // A key the career already recovered is a repeat (scores less on recovery).
    const bool repeat = holdsExperiment(science.recovered, e);
    toast(repeat ? "Experiment (repeat): %s -- scores less"
                 : "Experiment: %s",
          experimentName(e).c_str());
    printf("[science] t=%.1f '%s' recorded%s '%s'\n", time, runner.c_str(),
           repeat ? " REPEAT" : "", experimentName(e).c_str());
    fflush(stdout);
}

void Game::runExperiment(Kerbal *k) {
    if(k == nullptr || k->parts.empty()) {
        toast("No experiment: no kerbal");
        return;
    }
    runSuitExperiment(k->parts[0], k, nullptr, k->name, "");
}

/* A SEATED kerbal runs one: the suit Part is the storage, and the live pose is
   the capsule it sits in -- there is no body of its own (part.h). */
void Game::runSeatExperiment(Part *suit) {
    if(suit == nullptr || !suit->isSeat() || suit->container == nullptr) {
        toast("No experiment: no seat occupant");
        return;
    }
    Vehicle *ship = suit->container->owner;
    runSuitExperiment(suit, ship, suit->container, seatName(suit),
                      ship->name);
}

/* The pod's runner is whoever sits in the pod's ship: a seat occupant or a
   walking kerbal, so the runner arrives as a name. */
void Game::runPodExperiment(Part *pod, const std::string &runner) {
    if(pod == nullptr || pod->def == nullptr || pod->owner == nullptr) {
        toast("No experiment: no pod on a ship");
        return;
    }
    if(runner.empty()) {
        toast("No experiment: no kerbal to run it");
        return;
    }
    Vehicle *ship = pod->owner;
    Experiment e;
    if(!situationExperiment(ship->m_parent, ship, pod,
                            pod->def->experiment_family, runner, ship->name, e)) {
        return;
    }
    if(!pod->addExperiment(e)) {
        // Same per-type cap (the pod holds one of its own family).
        toast("Already holding one %s (the pod holds one) -- take it out first",
              e.type.c_str());
        printf("[science] t=%.1f '%s' already held '%s' (pod)\n", time,
               runner.c_str(), experimentName(e).c_str());
        fflush(stdout);
        return;
    }
    const bool repeat = holdsExperiment(science.recovered, e);
    toast(repeat ? "Experiment (repeat): %s -- scores less"
                 : "Experiment: %s",
          experimentName(e).c_str());
    printf("[science] t=%.1f '%s' recorded%s '%s' (pod)\n", time,
           runner.c_str(), repeat ? " REPEAT" : "", experimentName(e).c_str());
    fflush(stdout);
}

void Game::moveExperiment(Part *from, Part *to, size_t which) {
    if(from == nullptr || to == nullptr || from == to) {
        toast("No transfer: invalid parts");
        return;
    }
    if(which >= from->experiments.size()) {
        toast("No data to move");
        return;
    }
    if(!to->canReceive()) {
        toast("Can't store here: %s (not a holder)",
              to->def ? to->def->label().c_str() : "?");
        return;
    }
    // Copy by value: the erase below invalidates references into
    // from->experiments (latent UB when `from` holds a single finding).
    const Experiment e = from->experiments[which];
    if(!to->canHold(e)) {
        toast("Already holding: %s", experimentName(e).c_str());
        return;
    }
    const std::string name = experimentName(e);
    const std::string fromName = from->def ? from->def->label() : "?";
    const std::string toName = to->def ? to->def->label() : "?";
    from->experiments.erase(from->experiments.begin() + (std::ptrdiff_t)which);
    to->experiments.push_back(e);
    toast("Moved: %s  (%s -> %s)", name.c_str(), fromName.c_str(), toName.c_str());
    printf("[science] t=%.1f moved '%s' %s -> %s\n", time,
           name.c_str(), fromName.c_str(), toName.c_str());
    fflush(stdout);
}

// RMB click: pick the part, open its window. The [pick] line is the e2e hook.
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

/* Switching the active ship: the left one parks on rails, the taken one
   re-enters physics; rails warp drops to 10. */

// Keeps the "ship" focus entry in sync with the active ship; else the backdrop.
void Game::syncShipFocus() {
    if(ship != nullptr) {
        if(focusTargets.empty() || focusTargets[0].body != nullptr) {
            focusTargets.insert(focusTargets.begin(), { "ship", nullptr });
        }
        focusBody = 0;   // the "ship" focus target
        // camera is null on a --load boot (main.cpp creates it after load_game).
        if(camera != nullptr && camera->mode == CAM_ORBIT) {
            camera->Follow(ship->get_center_of_mass());
            // a kerbal is 0.75 m tall; 50 m would lose it
            camera->distance = ship->isEva() ? 5.0 : 50.0;
        }
    } else {
        if(!focusTargets.empty() && focusTargets[0].body == nullptr) {
            focusTargets.erase(focusTargets.begin());
        }
        // The shipless orbit view (the title backdrop).
        parkTitleCamera();
    }
}

/* The title-screen backdrop body + its camera park. The pick happens BEFORE
   the boot heavy phase, so that body is the synchronous one. Purely the menu
   backdrop -- gameplay anchors to `home`. */
TerrainBody *Game::pickTitleBody() {
    // --title-body pins the pick deterministically (test hook); else random.
    if(!args.title_body.empty()) {
        if(TerrainBody *b = sys.find(args.title_body)) {
            return (titleBody = b);
        }
        printf("[title] backdrop pin '%s' not found; picking randomly\n",
               args.title_body.c_str());
        fflush(stdout);
    }
    std::vector<TerrainBody *> cand;
    for(TerrainBody *b : sys.bodies) {
        if(b != sys.root) { cand.push_back(b); }
    }
    if(cand.empty()) {   // degenerate: no non-star body -- fall back to any
        if(sys.bodies.empty()) { return nullptr; }
        cand.push_back(sys.bodies[0]);
    }
    static std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<size_t> pick(0, cand.size() - 1);
    return (titleBody = cand[pick(rng)]);
}

void Game::parkTitleCamera() {
    if(focusTargets.empty()) { return; }
    // A system that never picked a backdrop gets one now.
    if(titleBody == nullptr) { pickTitleBody(); }
    int idx = -1;
    for(int i = 0; i < (int)focusTargets.size(); i++) {
        if(focusTargets[i].body == titleBody) { idx = i; break; }
    }
    if(idx < 0) {
        // focusTargets and sys.bodies disagree: pick fresh, don't park nowhere.
        pickTitleBody();
        for(int i = 0; i < (int)focusTargets.size(); i++) {
            if(focusTargets[i].body == titleBody) { idx = i; break; }
        }
        if(idx < 0) { return; }   // no body at all: nothing to park on
    }
    focusBody = idx;
    TerrainBody *b = focusTargets[idx].body;
    if(camera != nullptr) {
        // The title backdrop is always an orbit view: overrides a free cam.
        camera->mode = CAM_ORBIT;
        camera->Follow(focusWorldPos(idx));
        // 2 radii frames any body size -- don't "scale" it further.
        camera->distance = 2.0 * (double)b->radius;
        camera->ComputeView();
    }
    printf("[title] backdrop: %s%s, %.0f m out\n", b->name.c_str(),
           (b->name == args.title_body) ? " (pinned)" : "",
           2.0 * (double)b->radius);
    fflush(stdout);
}

// Basename compare: robust to "./" or absolute prefixes on either side.
static std::string baseName(const std::string &p) {
    const size_t i = p.find_last_of('/');
    return (i == std::string::npos) ? p : p.substr(i + 1);
}

/* The system file a save records (dir/save.json's "system"), or "" when the
   file is absent or predates the field. `format_ok` (optional)
   says whether its format is readable. Must not throw, and is needed before
   switchSystem: a switch tears the game down and the load may still refuse. */
static std::string saveSystemFile(const std::string &dir, bool *format_ok = nullptr) {
    if(format_ok != nullptr) { *format_ok = true; }
    std::ifstream f(dir + "/save.json");
    if(!f) { return ""; }
    nlohmann::json j;
    // Permissive parse, as in load_game (comments allowed).
    try { j = nlohmann::json::parse(f, nullptr, true); }
    catch(const std::exception &) { return ""; }
    // Peek the keys directly: saveMetaFromJson REFUSES a foreign format.
    if(j.contains("format")) {
        if(format_ok != nullptr) {
            *format_ok = j["format"].is_number_integer()
                         && j["format"].get<int>() == kSaveFormat;
        }
    }
    if(j.contains("system") && j["system"].is_string()) {
        return j["system"].get<std::string>();
    }
    return "";
}

bool Game::newGame() {
    // Any vehicle in the world, not just the active one, means a game runs.
    // A retire queued this frame does NOT count: retire() already took it out of
    // `vehicles`, and sweepRetired() runs at the top of the loop, never from a
    // game action (game.h) -- so the corpse is freed on its own schedule.
    if(!vehicles.empty()) {
        toast("A game is already running");
        return false;
    }
    /* A game with no identity cannot be saved: quicksave() writes into whatever
       gameId names, and after a quit-to-title that is the PREVIOUS game's
       directory (#299). startNewGame mints before calling this; every other way
       in (--new-game, a bare newGame) mints here. Mint after the guard above so
       a refused new game leaves no orphan directory. */
    if(gameId.empty() && !mintGameDir()) {
        toast("Too many same-named games this second; try again");
        return false;
    }
    /* A new career starts at t=0. Nothing else writes `time` -- only boot
       --start-time and a load -- so a New Game on a system that had been
       running opened at the old game's epoch, and the science stamps and Flight
       Summary inherited it (#299). setTime also re-derives the rails and drops
       the world-stamped caches, which a fresh world wants anyway. An explicit
       --start-time is a deliberate epoch (daylight, a body alignment), so it
       wins: resetting it would silently neuter the knob. */
    if(!args.cli_given.start_time) { setTime(0.0); }
    // A new game starts in the Space Center with NO ship, paused.
    time_accel = 0;
    clearFlightSummary();   // a prior recover's summary is not this game's
    science.reset();        // a fresh career: no score, nothing recovered
    /* A fresh career gets the full cast standing again: deaths shrink the pool
       (death does not return a suit) and a system switch empties it (new
       buildings, new pad), and only the boot mint ever restored it. */
    if(StaticBuilding *home = homeCrewPad(sys)) {
        for(Part *p : home->crew) { delete p; }
        home->crew.clear();
    }
    mintCrewPool();
    enterSpaceCenter(*this);
    printf("[game] new game: Space Center, no ship, paused (t=%.1f)\n", time);
    fflush(stdout);
    toastLog("New game -- Space Center");
    return true;
}

// The "home" body name a system JSON records ("" when absent). The New Game
// path syncs it: the hub frames the new system's home, so it must be solid.
static std::string systemHomeName(const std::string &path) {
    std::ifstream f(path);
    if(!f) { return ""; }
    nlohmann::json j;
    try { j = nlohmann::json::parse(f, nullptr, true); }
    catch(const std::exception &) { return ""; }
    if(j.contains("home") && j["home"].is_string()) {
        return j["home"].get<std::string>();
    }
    return "";
}

bool Game::startNewGame(const std::string &name, const std::string &sysPath,
                        float exhaustScale) {
    if(!vehicles.empty()) {
        toast("A game is already running");
        return false;
    }
    // Switch first (transactional: a bad file throws with the world untouched).
    // Same basename = already on it. Sync set: the new system's home.
    if(!sysPath.empty() && baseName(sysPath) != baseName(systemPath)) {
        std::vector<std::string> syncNames;
        std::string hn = systemHomeName(sysPath);
        if(!hn.empty()) { syncNames.push_back(hn); }
        try {
            switchSystem(sysPath, syncNames);
        } catch(const std::exception &e) {
            printf("[game] new game: cannot load system '%s': %s\n",
                   sysPath.c_str(), e.what());
            fflush(stdout);
            toast("Cannot load system: %s", e.what());
            return false;
        }
    }
    // Difficulty: scales every engine's exhaust velocity (thrust + delta-v).
    if(exhaustScale < 0.5f) { exhaustScale = 0.5f; }
    if(exhaustScale > 5.0f) { exhaustScale = 5.0f; }
    args.exhaust_scale = exhaustScale;
    // The game's identity is minted AFTER the switch, so a failed switch
    // leaves no orphan dir; minting here, not at first save, keeps the
    // same-second collision check sound. gameName first: mintGameDir names the
    // directory after it.
    gameName = name;
    if(!mintGameDir()) {
        toast("Too many same-named games this second; try again");
        return false;
    }
    const std::string gdir = datadir::saves() + "/" + gameId;
    /* The difficulty rides in the line because it changes every engine's
       thrust for the whole game: a save whose burns look wrong is read
       before the save file is opened. */
    printf("[game] new game '%s' (exhaust %.2fx) -> %s\n", name.c_str(),
           exhaustScale, gdir.c_str());
    fflush(stdout);
    return newGame();
}

bool Game::mintGameDir() {
    const std::string dir = newGameDir(datadir::saves(), gameName);
    if(dir.empty()) { return false; }
    ensure_dir(dir);
    gameId = baseName(dir);
    return true;
}

std::string Game::ensureGameDir() {
    if(!gameId.empty()) { return datadir::saves() + "/" + gameId; }
    /* Minting is for a game that EXISTS. quitToTitle clears the identity with
       the rest of the game, so a save from the title afterwards would invent a
       fresh game dir named after the dead one -- a phantom entry in the games
       list holding an empty game. events.cpp:492 closes exactly this hole for
       F5/F9; the Save window and --save/--resave reach the same mint ungated
       (#299). */
    if(!gameRunning(*this)) {
        toast("Save failed: no game is running");
        return "";
    }
    if(!mintGameDir()) {
        toast("Save failed: too many same-named games this second");
        return "";
    }
    return datadir::saves() + "/" + gameId;
}

/* If the save's system (meta.system) differs from the running one, switch into
   it FIRST so the ships load onto the right bodies. False = the switch failed. */
bool Game::ensureSystemForSave(const std::string &dir, bool *switched) {
    if(switched != nullptr) { *switched = false; }
    bool format_ok = true;
    const std::string sysfile = saveSystemFile(dir, &format_ok);
    if(!format_ok) {
        /* Refuse HERE, not only in load_game: a cross-system load switches
           first, and switchSystem tears the game down -- by the time load_game
           throws, the career is already lost. */
        printf("[load] refused %s: its \"format\" is not this build's format %d\n",
               dir.c_str(), kSaveFormat);
        fflush(stdout);
        toast("Load failed: this save's format is not this build's");
        return false;
    }
    if(sysfile.empty() || baseName(sysfile) == baseName(systemPath)) {
        return true;   // same system (or none recorded) -- nothing to switch
    }
    /* Load the new system, validate the save against it, and only then install.
       installSystem is the half that tears the running game down, so everything
       worth refusing is refused with the career still intact (#302). That
       includes the save's body references, which need the NEW world to resolve:
       a name the new system lacks used to fall back to another body and re-read
       the saved position in the wrong frame -- a ship 540 km underground,
       reported as "flying low" (#303). */
    // Sync the bodies the save's ships sit on, then the new system's home --
    // last, so the cap drops it first.
    std::vector<std::string> syncNames = saveShipBodies(dir);
    if(std::string hn = systemHomeName(sysfile); !hn.empty()) {
        syncNames.push_back(hn);
    }
    try {
        const WorldShaders world = get_world_shaders();
        System newSys = load_system(sysfile.c_str(), world.terrain, world.sun,
                                    nullptr);
        if(std::string why = preflightSave(dir, parts, &newSys); !why.empty()) {
            printf("[load] refused %s: %s (before the system switch)\n",
                   dir.c_str(), why.c_str());
            fflush(stdout);
            toast("Load failed: %s", why.c_str());
            /* Nothing adopted these bodies and System owns no destructor, so
               they are ours to free -- the same delete switchSystem does. */
            for(TerrainBody *b : newSys.bodies) { delete b; }
            return false;
        }
        installSystem(std::move(newSys), sysfile, syncNames);
    } catch(const std::exception &e) {
        printf("[load] cannot switch to system '%s': %s\n",
               sysfile.c_str(), e.what());
        fflush(stdout);
        toast("Load failed: %s", e.what());
        return false;   // stay on the current system
    }
    if(switched != nullptr) { *switched = true; }
    printf("[load] save is from system '%s'; switched before loading\n",
           sysfile.c_str());
    fflush(stdout);
    return true;
}

bool Game::loadFrom(const std::string &dir) {
    bool switched = false;
    if(!ensureSystemForSave(dir, &switched)) { return false; }
    /* load_game moves the clock BEFORE anything can fail: setTime re-derives
       every body's frame tree to the save's epoch (a ship restored into a
       ROTATING frame captures its rail state from those transforms), and
       time_accel and exhaust_scale come from the file too. A refusal that left
       them there handed the surviving game a paused clock at the SAVE's time
       with its bodies parked at the SAVE's epoch (#301). Rails are a pure
       function of `time`, so setting it back puts the frames back exactly. */
    const double prev_time = time;
    const int prev_accel = time_accel;
    const float prev_exhaust = args.exhaust_scale;
    try {
        load_game(*this, dir);
    } catch(const std::exception &e) {
        printf("[load] refused %s: %s\n", dir.c_str(), e.what());
        /* The two scalars are world-independent, so restore them either way --
           after a failed cross-system load the title's backdrop would otherwise
           sit frozen at the save's warp 0. Only the clock is switch-dependent:
           its epoch belongs to bodies that no longer exist. (switchSystem never
           writes `time`, so re-setting it there would be a harmless no-op --
           it is skipped for honesty, not necessity.) */
        time_accel = prev_accel;
        args.exhaust_scale = prev_exhaust;
        if(!switched) {
            setTime(prev_time);
            printf("[load] clock restored: t=%.1f, warp %d, exhaust %.2fx\n",
                   time, time_accel, args.exhaust_scale);
        }
        // A same-system refusal leaves science intact (load_game adopts it only
        // on success); a cross-system one already reset it.
        if(switched) {
            printf("[load] science reset by the system switch "
                   "(score=%d, %zu recovered)\n",
                   science.score, science.recovered.size());
        } else {
            printf("[load] science score=%d recovered=%zu (unchanged)\n",
                   science.score, science.recovered.size());
        }
        fflush(stdout);
        toast("Load failed: %s", e.what());
        // A switch tore the old ships down: a failed load leaves no game, so
        // title. Without a switch the old game and scene are intact: stay.
        if(switched) { enterTitle(*this); }
        return false;
    }
    // The thrust latch is per-active-ship and not in the save: clear it.
    thrust_latched = false;
    // A shipless save is still a game: land on the hub, not the title.
    if(ship != nullptr) { enterFlight(*this); } else { enterSpaceCenter(*this); }
    return true;
}

void Game::quicksave() {
    const std::string gamedir = ensureGameDir();
    if(gamedir.empty()) { return; }   // ensureGameDir already toasted
    const std::string slot = nextQuicksave(gamedir);
    try {
        save_game(*this, gamedir + "/" + slot);
    } catch(const std::exception &e) {
        toast("Quicksave failed: %s", e.what());
        return;
    }
    toast("Quicksaved %s (game '%s')", slot.c_str(), gameName.c_str());
}

void Game::quickload() {
    if(gameId.empty()) {
        toast("No quicksaves to load (no game yet)");
        return;
    }
    const std::string slot = latestQuicksave(datadir::saves() + "/" + gameId);
    if(slot.empty()) {
        toast("No quicksaves to load");
        return;
    }
    if(loadFrom(datadir::saves() + "/" + gameId + "/" + slot)) {
        toast("Loaded %s", slot.c_str());
    }
}

void Game::unloadGame() {
    /* Tear the vehicles down to the shipless-boot state. ~Vehicle detaches the
       welds, unregisters physics and deletes its parts -- and a seat occupant
       is OWNED by its capsule Part, so ~Part frees it with the ship (part.h).
       A walking kerbal, by contrast, IS a list entry. Deleting the vehicles is
       therefore the whole teardown, and it must happen BEFORE any body dies:
       a Vehicle holds m_parent and its frame, both owned by the body, so a
       live one outliving its body is a dangling pointer the tick and the
       renderer would follow. switchSystem honours that (unloadGame, then the
       old bodies). part_sels holds Part* into them: it goes first. No job drain
       needed -- no continuation touches a Vehicle. */
    part_sels.clear();
    /* A transfer row names a Vehicle and points at it, so the rows go with the
       vehicles. update() rebuilds them each refresh, but quit-to-title leaves
       frames between here and the next world where the TARGET window, the map's
       plane and --select-target would read freed names. */
    xferPlanner.dropTargets();
    clearFlightSummary();   // a summary from THIS game must not leak into the next
    science.reset();        // ...and neither must the career score / archive
    /* Retired vehicles are not in `vehicles`, so this is the other half of the
       teardown -- and it has to come first, because the bodies they still
       reference die below. */
    sweepRetired();
    for(Vehicle *v : vehicles) { delete v; }
    vehicles.clear();
    ship = nullptr;
    kerbal = nullptr;
    lastShip = nullptr;
    /* The game's identity belongs to the game that just ended. Leaving gameId
       set meant a later quicksave wrote into the PREVIOUS game's saves/<id>/
       directory (#299); newGame() mints a fresh one when this is empty. A load
       sets it from its own directory afterwards, so the switch path is
       unaffected. */
    gameId.clear();
    // The title screen is an orbit view: a quitting free-cam pilot keeps no pose.
    if(camera != nullptr) { camera->mode = CAM_ORBIT; }
    syncShipFocus();
    printf("[game] unloaded: ships torn down, no active vessel\n");
    fflush(stdout);
}

void Game::quitToTitle() {
    unloadGame();
    enterTitle(*this);
}

void Game::switchSystem(const std::string &path,
                        const std::vector<std::string> &syncNames) {
    /* Shaders are registry singletons: re-fetch the boot's files (cache hit, not
       a recompile). load_system throwing here is survivable precisely because
       installSystem is what destroys things. */
    const WorldShaders world = get_world_shaders();
    installSystem(load_system(path.c_str(), world.terrain, world.sun, nullptr),
                  path, syncNames);
}

/* The install half of a system switch: adopt an already-loaded system and land on
   the title. Transactional by construction -- the new system is loaded by the
   caller, so a bad system file throws before this runs and nothing is lost. It
   re-points sys / home / sun / focusTargets at the new system before the old
   bodies are deleted, so nothing ever points at freed memory. Split out for the
   same reason: a caller can validate a save against the new world before the
   teardown (#302). jobs.abort waits for the in-flight snapshot read (what makes
   the delete safe) and is terminal, hence jobs.restart. */
void Game::installSystem(System &&newSys, const std::string &path,
                         const std::vector<std::string> &syncNames) {
    const WorldShaders world = get_world_shaders();
    unloadGame();
    std::vector<TerrainBody *> oldBodies = sys.bodies;
    sys = std::move(newSys);
    // The star field belongs to the system, so it swaps with the system.
    assert(skybox != nullptr);   // render.cpp draws with it unguarded
    skybox->load(sys.skybox_faces);
    systemPath = path;   // the running system is now this one
    home = sys.home;
    sun = sys.root;
    titleBody = nullptr;   // the old system's pick dangles with oldBodies;
                           // the sync-set block re-picks it
    surfmap.body = nullptr;   // a combo pick names an old-system body deleted
                              // below; invalidateClockStampedCaches resets the
                              // map's cache, not this pointer.
    /* load_system propagated the new tree at t=0 but the clock did not move:
       bring the bodies to `time` or a paused game renders t=0 and snaps later. */
    syncRails();
    focusTargets.clear();
    for(TerrainBody *b : sys.bodies) {
        focusTargets.push_back({ b->name.c_str(), b });
    }
    focusBody = 0;
    // The planner's plan + grid and the surface map were built for the OLD system.
    invalidateClockStampedCaches();
    // Old bodies are reachable only from oldBodies now: abort the terrain
    // stream (it captures them), then delete.
    jobs.abort();
    for(TerrainBody *b : oldBodies) { delete b; }
    jobs.restart();
    // The sync set resolves against the NEW system; unknown names drop out. A
    // bare switch lands on the title, so the backdrop must be the solid one.
    std::vector<TerrainBody *> sync;
    for(const std::string &n : syncNames) {
        if(TerrainBody *b = sys.find(n)) { sync.push_back(b); }
    }
    if(sync.empty()) {
        titleBody = pickTitleBody();
        if(titleBody != nullptr) { sync.push_back(titleBody); }
    }
    postHeavyPhase(sys, sync, jobs, world.atmosphere, world.cloud,
                   world.ocean, world.ring, args.cloud_mesh);
    // World buildings travel with the system: re-place the new file's set.
    place_world_buildings(sys, buildings, partsshader, sun);
    // Launch-site ids are per-system; drop a sticky pick.
    vab.siteName.clear();
    vab.bodyName.clear();

    enterTitle(*this);
    parkTitleCamera();
    std::string syncList;
    for(TerrainBody *b : sync) {
        if(!syncList.empty()) { syncList += ", "; }
        syncList += b->name;
    }
    printf("[game] switched system -> %s (home %s, sync %s)\n",
           path.c_str(), home->name.c_str(), syncList.c_str());
    fflush(stdout);
}

void Game::settleVehicles(Vehicle *active) {
    // Ships sharing an orbit scenario are spaced by slot.
    apply_scenarios(*this);
    /* Idle ships park on rails: flying ones coast on their conic, pad ships
       freeze. Neither free fall nor grounded refuses and stays in physics. */
    for(auto *s : vehicles) {
        if(s != active) { s->goOnRails(); }
    }
}

void Game::select_vehicle(Vehicle *v) {
    if(v == nullptr || v == ship) { return; }
    // Every Vehicle is controllable: a seated kerbal is not one (part.h).
    // The old active ship (null when launching from the orbit-view state).
    if(ship != nullptr) {
        ship->releaseControl();
        ship->goOnRails();
    }
    v->leaveRails();
    ship = v;
    // The thrust latch is per-active-ship: a new ship starts with thrust released.
    thrust_latched = false;
    if(time_accel >= kRailsWarp) {
        time_accel = 10;
        toast("Active ship: %s, warp 10x", ship->name.c_str());
    }
    // First person is a cockpit view: drop it on an EVA handoff BEFORE the sync.
    if(camera != nullptr && camera->mode == CAM_FIRST && v->isEva()) {
        camera->mode = CAM_ORBIT;
    }
    syncShipFocus();
    // "N of M" in spawn order.
    int n = 0, i = 0;
    for(auto *x : vehicles) {
        n++;
        if(x == v) { i = n; }
    }
    printf("Active ship %d of %d: %s\n", i, n, ship->name.c_str());
}

/* --- crew. A seated kerbal is a seat on its capsule Part (part.h); a walking
   one is a Kerbal vehicle in Game::vehicles. There is no third state. */

std::vector<Part *> vehicleSeats(Vehicle *v) {
    std::vector<Part *> out;
    for(Part *p : v->parts) {
        for(Part *suit : p->seats) {
            if(suit != nullptr) { out.push_back(suit); }
        }
    }
    return out;
}

std::string seatName(const Part *suit) {
    if(suit == nullptr || suit->kerbal == nullptr) { return ""; }
    return suit->kerbal->name;
}

void freeKerbalsInto(const std::vector<Vehicle *> &vehicles,
                     std::vector<Kerbal *> &out) {
    out.clear();
    /* Buffered because this runs twice per part-window frame. Every kerbal the
       list yields is FREE -- a seated one is not a vehicle at all. */
    for(auto *s : vehicles) {
        if(!s->isEva()) { continue; }
        out.push_back(static_cast<Kerbal *>(s));
    }
}

std::vector<Kerbal *> freeKerbals(const std::vector<Vehicle *> &vehicles) {
    std::vector<Kerbal *> out;
    freeKerbalsInto(vehicles, out);
    return out;
}

/* Reach check: `k` must be within kBoardingRange of `part`. A raw position
   subtraction is only a distance within one frame. */
bool Game::kerbalInRange(Kerbal *k, Part *part) {
    if(k == nullptr || part == nullptr || part->owner == nullptr) {
        return false;
    }
    const glm::dvec3 partCom =
        part->owner->GetPositionRelTo(part, k->frame);
    const double dist = glm::length(k->get_center_of_mass() - partCom);
    return dist <= kBoardingRange;
}

/* Put the person in `suit`'s seat onto the surface beside its capsule. This
   MINTS the Kerbal vehicle: the suit Part leaves the capsule's seat and becomes
   the vehicle's one part, keeping its uid, fuel and pockets. */
void Game::kerbalEVA(Part *suit) {
    if(suit == nullptr || !suit->isSeat()) {
        toast("EVA: no seat occupant");
        return;
    }
    Part *capPart = suit->container;
    Vehicle *ship = capPart->owner;
    const PartDef *capDef = capPart->def;
    const std::string who = seatName(suit);

    /* Standing / hover pose beside the capsule. The branch is the ship's
       GROUNDED state, not its frame's rotation: zero velocity in a rotating
       frame is co-rotating free fall. Computed BEFORE the seat is emptied,
       because it reads the capsule's live pose. */
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
    const glm::dvec3 capVel = ship->partVel(capPart);

    /* Empty the seat: the suit stops being held, and the capsule loses the
       mass (checkPartInvariants holds on both sides of this). The suit keeps
       its owned KerbalProfile -- identity travels with the person. */
    if(capPart->removeSeat(suit) == nullptr) {
        toast("EVA: seat not on this capsule");
        return;
    }
    ship->rebuildCompound();

    // Mint the walker around the suit it just left.
    Kerbal *k = new Kerbal;
    k->name = who;
    /* The walker's own def, not the seat's: a crew entry writes defPath (a SHIP
       def) while a seat writes a catalog part name (SavePart::part). Both resolve
       to the same one-part kerbal def, which is what buildKerbalFromSave now
       requires of this path. */
    k->defPath = kKerbalShipDef;
    k->home = ship->home;
    k->scenario = ship->scenario;
    k->m_parent = ship->m_parent;
    k->sun = sun;
    k->frame = ship->frame;
    k->setRoot(suit);      // parts list is the ownership list (part.h)
    /* setRoot re-points the suit only. Its pockets were wired with the SEAT's
       vehicle as their Part::owner, and dropItem / addToContainer both resolve a
       carrier through that pointer -- leave them at the ship and dropping a
       picked-up crate spawns it at the capsule the person just left. */
    inventorySetOwner(suit, k);
    /* finalize, NOT init: init re-seeds tanks, and this suit may be partly
       burned. finalize builds the hull (rebuildCompound) and the fuel groups. */
    k->finalize();

    if(ship->isGrounded()) {
        const double floorR = std::max(glm::length(capCom) - capDef->height / 2.0,
            (double)ship->m_parent->GetTerrainHeight(glm::vec3(upDir)));
        k->placeShipAtCom(upDir * (floorR + k->restAlt()) + tangent * offset, orient);
        k->setVelocity(glm::dvec3(0.0));
    } else {
        k->placeShipAtCom(capCom + tangent * offset, orient);
        k->setVelocity(capVel);   // co-moving beside the ship
    }
    // Re-home and join the list: starts the flight journal.
    k->setSoi(ship->frame, time);
    vehicles.push_back(k);
    k->enterWorld();

    kerbal = k;
    lastShip = ship;
    select_vehicle(k);
    toast("EVA: %s", who.c_str());
    /* Log the capsule's slot in the ship's part list (e2e pins "part 0"). */
    int capIdx = 0;
    for(size_t i = 0; i < ship->parts.size(); i++) {
        if(ship->parts[i] == capPart) { capIdx = (int)i; break; }
    }
    printf("[crew] t=%.1f EVA: '%s' out of '%s' part %d\n",
           time, who.c_str(), ship->name.c_str(), capIdx);
}

/* Put a free kerbal `k` into the capsule (ship, part): the Kerbal vehicle is
   DELETED and its suit Part becomes a seat occupant (part.h). The suit keeps
   its uid, fuel and pockets, so a part selection pointing at it stays valid. */
void Game::kerbalBoard(Kerbal *k, Vehicle *ship, size_t part) {
    if(part >= ship->parts.size()) { return; }
    /* A kerbal cannot board itself. The capsule guard below already refuses it
       (a suit is not a capsule), but the delete further down would otherwise
       leave a capsule containing its own pointer -- effectiveMass recursing
       forever is not a failure mode worth depending on a def never changing. */
    if(ship == k) { return; }
    Part *capPart = ship->parts[part];
    const PartDef *capDef = capPart->def;
    if(capDef->crew_capacity <= 0) {
        toast("Board: part %zu is not a capsule", part);
        return;
    }
    if(capPart->seatCount() >= capDef->crew_capacity) {
        toast("Board: capsule full (%d)", capDef->crew_capacity);
        return;
    }
    Part *suit = k->parts.empty() ? nullptr : k->parts[0];
    if(suit == nullptr) { return; }
    /* A walker IS its suit: one Part. The delete below clears the parts list
       first so ~Vehicle does not free the suit, so anything else in there would
       leak silently. */
    assert(k->parts.size() == 1);
    const std::string who = k->name;
    const bool wasActive = (this->ship == k);
    /* The suit outlives the vehicle: take it off the parts list FIRST so
       ~Vehicle does not free it. The walker is in Game::vehicles (kerbalEVA
       added it), so it leaves the list too or the next walk reads freed
       memory. */
    k->parts.clear();
    retire(k);

    /* Park the suit as a seat occupant (part.h). `owner` is the vehicle the
       capsule sits on. Identity (Part::kerbal) stays on the suit. */
    // Capacity was checked above; a failure here is a bug, not a recoverable state.
    assert(capPart->addSeat(suit));
    /* The suit AND whatever its pockets hold now ride the ship: a walker picks
       items up with itself as their Part::owner, so leaving that pointer at the
       deleted Kerbal is a dangling Vehicle* in the containment tree -- the same
       edge absorbShip / extractSubtreeAsShip walk recursively. */
    inventorySetOwner(suit, ship);
    // Rebuild so the next physics step sees the heavier ship.
    ship->rebuildCompound();
    toast("Board: %s -> %s", who.c_str(), ship->name.c_str());
    printf("[crew] t=%.1f Board: '%s' into '%s' part %zu\n",
           time, who.c_str(), ship->name.c_str(), part);

    // The kerbal is no longer controllable: hand control to the ship it entered.
    if(wasActive) {
        lastShip = ship;
        select_vehicle(ship);
    }
}

/* Drop an inventory item -- it becomes a free 1-part ship. */
Vehicle *Game::dropItem(Part *item) {
    if(item == nullptr || item->container == nullptr) { return nullptr; }
    Vehicle *carrier = item->container->owner;
    if(carrier == nullptr) { return nullptr; }
    /* A parked carrier's hull state is stale: the item would spawn in
       collision or with the wrong velocity. Only a live carrier can shed. */
    if(carrier->onRails) {
        toast("Drop: %s is parked", carrier->name.c_str());
        return nullptr;
    }

    // Item pose = container pose; rigid velocity v + w x r.
    glm::dvec3 itemPos; glm::dmat3 itemRot;
    carrier->partWorldPose(item->container, itemPos, itemRot);
    const glm::dvec3 com = carrier->comPos();
    const glm::dvec3 v = GetVelocity(carrier->hull)
                       + glm::cross(GetAngVelocity(carrier->hull),
                                   itemPos - com);

    /* The seat guard (inventory.cpp): a person held by a capsule cannot be
       shed as cargo, and spawning it as a 1-part ship while the capsule still
       owns it would free the suit twice. */
    if(!inventoryRemove(item)) { return nullptr; }

    Vehicle *nv = new Vehicle();
    // The part label is the same for every crate of a kind, so two drops of one
    // part type would land in the ship list under one indistinguishable name.
    nv->name = dedupVehicleName(vehicles, item->def->label(), &roster);
    nv->defPath = "";
    nv->setSoi(carrier->frame, time);   // journal starts at the drop
    vehicles.push_back(nv);
    nv->home = carrier->home;
    nv->sun = carrier->sun;
    nv->parts.push_back(item);
    // The whole inventory subtree (a crate in a crate) rides the new ship.
    inventorySetOwner(item, nv);
    nv->finalize();
    nv->placeShip(itemPos, itemRot);
    nv->setVelocity(v);
    SetAngVelocity(nv->hull, GetAngVelocity(carrier->hull));
    nv->enterWorld();
    // The carrier's compound still carries the item's mass + inertia.
    carrier->rebuildCompound();
    toast("Dropped %s", item->def->name.c_str());
    return nv;
}

/* Pick up a dropped item -- re-parent it into the container. */
bool Game::pickUpItem(Vehicle *itemShip, Part *dest) {
    if(itemShip == nullptr || dest == nullptr) { return false; }
    if(itemShip->parts.size() != 1) { return false; }
    /* A walking kerbal is a person, not a crate. Its suit is a kerbal-type
       Part, and addToContainer takes anything with a null container -- so the
       stow would succeed and checkContainmentTree would then refuse the ship
       ("a person is not cargo"), which rebuildCompound asserts on. */
    if(itemShip->isEva()) {
        toast("Pick up: %s is a crew member, not cargo", itemShip->name.c_str());
        return false;
    }
    /* A crewed ship is not cargo: the capsule owns its seat occupants, and the
       suit part would dangle in the destination container. */
    if(!vehicleSeats(itemShip).empty()) {
        toast("Pick up: %s has crew aboard -- EVA them out first",
              itemShip->name.c_str());
        return false;
    }
    Part *item = itemShip->parts[0];
    if(!dest->isContainer()) { return false; }

    if(!inventoryAdd(item, dest)) {
        toast("Pick up: %s is full", dest->def->name.c_str());
        return false;
    }

    /* Clear itemShip->parts BEFORE the retire: ~Vehicle deletes its parts, and
       the item is already owned by dest -- leaving it in double-frees. */
    if(ship == itemShip) { ship->releaseControl(); }
    itemShip->parts.clear();
    retire(itemShip);
    // The carrier's compound gains the item's mass.
    if(dest->owner != nullptr) { dest->owner->rebuildCompound(); }
    toast("Picked up %s", item->def->name.c_str());
    return true;
}

/* V: toggle EVA. From a ship: put its first seat occupant out and take control;
   from the kerbal: hand control back to the ship the player came from. */
void Game::toggle_eva() {
    if(ship == nullptr) {
        // Orbit-view state: no ship, hence no kerbal, to EVA.
        toast("EVA: no active ship");
        return;
    }
    if(ship->isEva()) {
        if(lastShip != nullptr && !lastShip->isEva()) {
            select_vehicle(lastShip);
        } else {
            toast("EVA: no ship to return to");
        }
        return;
    }
    std::vector<Part *> seats = vehicleSeats(ship);
    if(seats.empty()) {
        toast("EVA: no crew aboard %s", ship->name.c_str());
        return;
    }
    kerbalEVA(seats.front());
}

/* Enter rails warp: park every ship. Refuses -- keeping the current accel --
   if any ship is not rail-eligible (e.g. a suborbital descent). */
bool Game::enter_rails_warp() {
    for(auto *s : vehicles) {
        if(!s->canRail()) {
            printf("Rails warp refused: '%s' is neither in free fall nor "
                   "grounded (warp stays %d)\n", s->name.c_str(), time_accel);
            toast("Rails warp refused: '%s' is not in free fall",
                  s->name.c_str());
            return false;
        }
    }
    for(auto *s : vehicles) { s->goOnRails(); }
    // On rails the engines are off: clear the latch so it can't re-engage
    // on the way out.
    thrust_latched = false;
    return true;
}

/* Proximity: keep the ships near the active one live, park the rest. The
   ground/fly regime comes from inTerrainBand(), not isGrounded(): its speed
   term would flip a still-settling lander to the wide in-air radii. */
void Game::updateProximity() {
    Vehicle *a = ship;
    if(a == nullptr) { return; }

    const bool grounded = a->inTerrainBand();
    const double r_on  = grounded ? args.prox_ground_on  : args.prox_fly_on;
    const double r_off = grounded ? args.prox_ground_off : args.prox_fly_off;

    // Waking a ship re-homes it but never moves it between containers, so the
    // live list is safe to walk here.
    bool any_engaged = false;
    for(auto *s : vehicles) {
        if(s == a) { continue; }
        const double d = a->distanceTo(s);
        if(s->onRails && r_on > 0.0 && d < r_on) {
            s->releaseControl();   // no armed commands once it's live
            s->leaveRails();
            /* Re-express the woken ship in its SOI's rotating frame NOW, while
               the transforms are still at last tick's epoch (this runs before
               UpdateOrbitRails). Later would rotate a stale rail_pos. */
            s->switchFrames(time);
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
        a->switchFrames(time);   // same epoch-consistency as the neighbor wake
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
            for(auto *s : vehicles) {
                if(s == a) { continue; }
                printf("[prox]   %s dist=%.1f m %s\n", s->name.c_str(),
                       a->distanceTo(s), s->onRails ? "on-rails" : "LIVE");
            }
        }
    }
}

/* Docking is INTENT-driven: the active ship mates only when the player has
   armed one of its own ports AND targeted a port on another ship. The intent
   is consumed on success, so an undock cannot immediately re-dock. */
void Game::updateDocking() {
    Vehicle *a = ship;
    if(a == nullptr || a->isEva() || a->onRails) { return; }
    // Both halves of the intent are required.
    if(a->dockTargetPort == nullptr || a->dockTargetShip == nullptr
       || a->dockArmPort == nullptr) { return; }
    Vehicle *b = a->dockTargetShip;
    /* The target must be another live ship still carrying that port. Every
       death goes through retire(), which drops this intent before the hull
       leaves the world, so a pointer that survives here is one that was never
       dropped -- the belt, not the braces. */
    bool targetOk = (b != a && !b->isEva());
    if(targetOk) {
        targetOk = false;
        for(Part *p : b->parts) {
            if(p == a->dockTargetPort && p->isDockingPort()) { targetOk = true; break; }
        }
    }
    if(!targetOk) { a->dockTargetShip = nullptr; a->dockTargetPort = nullptr; return; }
    /* On rails or in another frame: cannot mate this tick. Keep the intent;
       proximity wakes the target. */
    if(b->onRails || b->frame != a->frame) { return; }

    Part *pb = a->dockTargetPort;
    Part *bestA = a->dockArmPort;
    /* The arm must still be a live port here (else it was staged away). Drop
       the arm, keep the target so a re-arm retries. */
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
    // Each port presents the face closest to the other part: +Z or -Z.
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
    /* b's parts are already a's, so this frees the empty vessel -- and its hull
       leaves the Bullet world here, not at the sweep, or the ship that just
       swallowed it spends the next substep colliding with a ghost. */
    retire(b);
    toast("Docked with %s", bName.c_str());
    printf("[dock] t=%.3f a=\"%s\" b=\"%s\" d=%.3f m v=%.3f m/s\n",
           time, a->name.c_str(), bName.c_str(), bestD, bestV);
}

/* Undock: split the most recent seam off the active ship into its own vessel. */
void Game::undock() {
    Vehicle *a = ship;
    if(a == nullptr || a->isEva()) { return; }
    if(a->seams.empty()) {
        toast("Nothing docked to undock");
        return;
    }
    // Undock needs the parts in the physics world: wake a railed ship first.
    if(a->onRails) {
        a->leaveRails();
        if(time_accel >= kRailsWarp) {
            time_accel = 1;
            toast("Undock: left the rails, warp 1x");
        }
    }
    Vehicle::DockSeam seam = a->seams.back();
    /* The seam carries the partner's name, which another live vessel may already
       hold -- two ships docked to the same class, or the partner still flying as
       its own vessel. Undocking would then mint a second vehicle under one name,
       and every name-only readout (ship list, toasts, this line) can't tell them
       apart. */
    const std::string nm = dedupVehicleName(vehicles, seam.name, &roster);
    Vehicle *out = a->extractSubtreeAsShip(seam.root, nm, time);
    if(out == nullptr) {
        toast("Cannot undock");
        return;
    }
    vehicles.push_back(out);   // the split ship joins the list at the cut
    out->enterWorld();   // the split leaves world registration to the caller
    /* No seam pop here: extractSubtreeAsShip already drops a seam split across
       the cut, so popping would drop the WRONG one. */
    // Part windows address parts by index, which just shifted: drop them.
    dropPartWindowsFor(a);
    toast("Undocked %s", out->name.c_str());
    printf("[undock] t=%.3f a=\"%s\" b=\"%s\"\n", time, a->name.c_str(), out->name.c_str());
}

/* Stage: fire the active stage's decouplers. Each decoupler's child-side
   subtree comes off as a SEPARATE ship (extractSubtreeAsShip, not a delete)
   and joins the vehicle list. Decouplers fire shallowest-first. */
void Game::stage() {
    Vehicle *a = ship;
    if(a == nullptr || a->isEva()) { return; }
    const int st = a->activeStage();

    // Staging needs the parts in the physics world: wake a railed ship first.
    if(a->onRails) {
        a->leaveRails();
        if(time_accel >= kRailsWarp) {
            time_accel = 1;
            toast("Staging: left the rails, warp 1x");
        }
    }

    /* Refuse if any dropped part carries a crewed capsule -- crew are locked
       to the vessel. */
    const std::vector<Part *> dropped = a->droppedPartsAtStage(st);
    bool crewOnStage = false;
    for(Part *p : dropped) {
        if(p->def == nullptr || p->def->crew_capacity <= 0) { continue; }
        if(p->seatCount() > 0) { crewOnStage = true; }
    }
    if(crewOnStage) {
        printf("Stage: refused -- crew aboard the capsule (EVA them first)\n");
        toast("Cannot stage -- EVA the capsule's crew out first");
        return;
    }

    /* Chutes deploy BEFORE the decoupler extractions, while every part is
       still on this ship, so a chute riding a subtree about to drop deploys
       with it. One-shot. */
    const int chutes = a->deployChutes(st);
    if(chutes > 0) {
        printf("[chute] t=%.1fs deployed %d chute(s) on stage %d\n",
               time, chutes, st);
        toast("Chute deployed (%d)", chutes);
    }

    // The decouplers on this stage (roots of the dropped subtrees), shallowest-first.
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

        /* De-dup a name against the live vehicles: first keeps the bare name,
           later get #2, #3. One rule for every ship-naming site (this one was
           a verbatim copy of it as a lambda, which is how the others drifted). */

        for(Part *d : decs) {
            // Name: the parent ship's name, qualified by the decoupler part.
            std::string base = a->name;
            const std::string qual = d->def->label();
            if(!qual.empty()) { base += " " + qual; }
            // A one-shot "slam" as the part separates; 0.4 sits it with the
            // engine hum (a transient reads louder than a steady loop).
            const std::string nm = dedupVehicleName(vehicles, base, &roster);
            Vehicle *out = a->extractSubtreeAsShip(d, nm, time);
            if(out == nullptr) { continue; }   // already absorbed into an outer ship
            vehicles.push_back(out);           // the split subtree is its own vessel
            audio.playOnce("res/audio/qubodup-crash.wav", 0.4f);
            out->enterWorld();
            ships++;
            parts += (int)out->parts.size();
        }
    }
    /* Step the counter on every press, even a decoupler-less stage: a stage
       number with no decoupler must not wedge the lower ones out of reach. */
    a->advanceStage();
    if(ships > 0) {
        // Part windows index parts, which just shifted: drop them.
        dropPartWindowsFor(a);
        printf("Stage: dropped %d ship(s) / %d part(s); now on stage %d of %d\n",
               ships, parts, a->activeStage(), a->numStages());
        toast("Staged -- dropped %d ship(s)", ships);
    } else {
        printf("Stage: nothing left to separate\n");
    }
}

/* Remove a ship + its bookkeeping. Refuses to remove the last ship. If it was
   active, control hands off to the next ship in spawn order. */
bool Game::ui_remove_vehicle(Vehicle *v) {
    if(v == nullptr) { return false; }
    if(vehicles.size() <= 1) {
        printf("Refusing to remove the last ship\n");
        return false;
    }
    /* Neither a crewed ship nor a person can be removed here: a seat occupant
       is owned by its capsule and would be deleted along with it (part.h), and
       a walking kerbal is not a vessel to discard. */
    if(v->isEva()) {
        // A walking kerbal is a vehicle and passes both tests below.
        toast("Cannot remove %s -- a crew member can't be deleted",
              v->name.c_str());
        return false;
    }
    if(!vehicleSeats(v).empty()) {
        toast("Cannot remove %s -- EVA its crew out first", v->name.c_str());
        return false;
    }
    const bool wasActive = (v == ship);
    const std::string removedName = v->name;
    if(wasActive) { v->releaseControl(); }

    /* Pick the successor BEFORE the erase, while the order is still intact: the
       next entry, wrapping past the end back to the previous one. */
    Vehicle *next = nullptr;
    if(wasActive) {
        const auto at = std::find(vehicles.begin(), vehicles.end(), v);
        if(at != vehicles.end()) {
            next = (at + 1 != vehicles.end()) ? *(at + 1) : *(at - 1);
        }
    }

    retire(v);

    if(wasActive) {
        if(next != nullptr) {
            next->leaveRails();
            ship = next;
            if(time_accel >= kRailsWarp) {
                time_accel = 10;
                toast("Active ship: %s, warp 10x", ship->name.c_str());
            }
            focusBody = 0;
            if(camera != nullptr && camera->mode == CAM_ORBIT) {
                camera->Follow(ship->get_center_of_mass());
                camera->distance = 50.0;
            }
            // N/M in the remaining ships.
            int n = 0, i = 0;
            for(auto *x : vehicles) {
                n++;
                if(x == next) { i = n; }
            }
            printf("Removed '%s'; active ship %d of %d: %s\n",
                   removedName.c_str(), i, n, ship->name.c_str());
        } else {
            /* Unreachable with today's model, but without this guard
               `ship` would keep pointing at the deleted vehicle. */
            ship = nullptr;
            syncShipFocus();
            // A shipless game is still a game: the floor is the hub.
            enterSpaceCenter(*this);
            printf("Removed '%s'; nothing left to control -- no active vessel\n",
                   removedName.c_str());
        }
    } else {
        /* A game can hold vessels with NONE of them active: recoverActive()
           detaches the last one without a handoff, and the tracking list still
           lists the rest. Reading `ship` blind here segfaults. */
        printf("Removed '%s' (active unchanged: %s)\n",
               removedName.c_str(), ship ? ship->name.c_str() : "(none)");
    }
    return true;
}

/* Recover the active vessel -- the end of a flight. Unlike ui_remove_vehicle this
   allows the last vessel (the hub is a legal shipless floor) and absorbs the
   crew. Lands on the hub with the Flight Summary. */
void Game::recoverActive() {
    Vehicle *v = ship;
    if(v == nullptr) {
        toast("Recover: no active ship");
        return;
    }
    const std::string name = v->name;
    if(v->isEva()) {
        // A free kerbal is its own vehicle, not a mission.
        toast("Cannot recover %s -- a crew member can't be recovered",
              name.c_str());
        return;
    }
    // Re-run SoI detection first: switchFrames/railsTick run before the physics
    // substeps, so a crossing in those (or while paused) leaves m_parent stale.
    if(v->onRails) { v->railsTick(time, 0.0); } else { v->switchFrames(time); }
    v->flog.observe(time, v->m_parent ? v->m_parent->name : "");
    // A flight ends only when the ship is HOME: grounded in the home body's
    // SoI. --recover-anywhere skips the check.
    if(!args.recover_anywhere && (v->m_parent != home || !v->isGrounded())) {
        toast("Cannot recover %s -- it must be grounded on the home body",
              name.c_str());
        return;
    }
    flightSummary.shipName = name;
    flightSummary.log = v->flog;
    flightSummary.end_t = time;
    /* Harvest every experiment aboard -- the ship's parts and each seat's suit
       -- BEFORE the delete below frees them. recoverMany de-dups by key, so a
       finding held by N kerbals banks once. */
    std::vector<Experiment> loot;
    for(Part *p : v->parts) {
        loot.insert(loot.end(), p->experiments.begin(), p->experiments.end());
    }
    for(Part *suit : vehicleSeats(v)) {
        loot.insert(loot.end(), suit->experiments.begin(), suit->experiments.end());
    }
    std::map<std::string, double> bodyMult;
    for(TerrainBody *b : sys.bodies) {
        bodyMult[b->name] = b->science_mult;
    }
    const RecoverSummary rec = recoverMany(science, loot, bodyMult, time);
    flightSummary.scienceGained = rec.gained;
    flightSummary.repeatScience = rec.repeat;
    flightSummary.newExperiments = rec.fresh;

    /* The crew come home: suits peel off to the launch pad before the ship
       dies (part.h: a kerbal is a Part at one place). */
    {
        std::vector<Part *> home = vehicleSeats(v);
        for(Part *suit : home) {
            Part *cap = suit->container;
            if(cap != nullptr) { cap->removeSeat(suit); }
            // returnCrew unowns the suit and whatever its pockets hold.
            returnCrew(sys, suit);
        }
    }

    // Part windows on the ship would dangle with it, and the crew are already
    // home: out of the list, out of every reference, out of the world.
    retire(v);

    // No handoff to a neighbour: the flight is over.
    syncShipFocus();
    enterSpaceCenter(*this);
    setWinOpen(W_FlightSummary, true);

    printf("[recover] t=%.1f '%s' recovered\n", time, name.c_str());
    if(flightSummary.scienceGained > 0) {
        printf("[science] recovered +%d (total %d): %zu new, +%d repeat\n",
               flightSummary.scienceGained, science.score,
               flightSummary.newExperiments.size(),
               flightSummary.repeatScience);
        for(const Experiment &e : flightSummary.newExperiments) {
            printf("[science]   new: %s\n", experimentName(e).c_str());
        }
    }
    {
        char dur[32];
        fmt_cal_duration(sys.home ? sys.home->cal : Calendar{},
                         time - flightSummary.log.start_t, dur, sizeof dur);
        printf("[flight] vessel='%s' duration=%s events=%zu\n",
               flightSummary.shipName.c_str(), dur,
               flightSummary.log.events.size());
        for(const FlightEvent &e : flightSummary.log.events) {
            char ts[48];
            if(!fmt_cal_compact(sys.home ? sys.home->cal : Calendar{},
                                e.t, ts, sizeof ts)) {
                snprintf(ts, sizeof ts, "t=%.0f", e.t);
            }
            printf("[flight] %s %s %s\n", ts,
                   e.enter ? "entered" : "left", e.body.c_str());
        }
    }
    fflush(stdout);
    toast("Recovered %s", name.c_str());
}
