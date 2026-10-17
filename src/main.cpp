// Open Space Program

#include <stdio.h>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <ctime>
#include <cstdlib>
#include <filesystem>
#include <vector>
#include <string>
#include <cmath>
#include <numbers>
#include <fstream>
#include <map>
#include <set>

#include <SDL3/SDL.h>
#include <SDL3/SDL_keycode.h>

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/gtc/noise.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/projection.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/polar_coordinates.hpp>

#include "display.h"
#include "mesh.h"
#include "shader.h"
#include "camera.h"
#include "body.h"
#include "physics.h"
#include "gldebug.h"
#include "frame.h"
#include "shipdef.h"
#include "crew.h"
#include <nlohmann/json.hpp>
#include "billboard.h"
#include "texture.h"
#include "skybox.h"
#include "postfx.h"
#include "ui.h"
#include "siminput.h"
#include "terrain.h"
#include "system.h"
#include "vehicle.h"
#include "radialtest.h"
#include "docktest.h"
#include "ships.h"
#include "buildings.h"
#include "game.h"
#include "events.h"
#include "tick.h"
#include "render.h"
#include "gameui.h"
#include "vab.h"
#include "save.h"
#include "datadir.h"
#include "resdir.h"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "cli.h"

#include "../middleware/imgui/imgui.h"
#include "../middleware/imgui/backends/imgui_impl_sdl3.h"
#include "../middleware/imgui/backends/imgui_impl_opengl3.h"
#include "../middleware/implot/implot.h"


/* ResourceType / ResourceContent / PartDef live in shipdef.h. */

// Pump the SDL queue during pre-loop load so a window-close is honoured
// promptly (poll_events() only runs in the main loop). True = quit asked.
static bool pumpLoadingQuit() {
    SDL_Event ev;
    while(SDL_PollEvent(&ev)) {
        ImGui_ImplSDL3_ProcessEvent(&ev);
        if(ev.type == SDL_EVENT_QUIT) { return true; }
    }
    return false;
}

// Centered "loading..." label on black, so the window never sits blank.
static void drawLoadingFrame(Renderer &display, ImFont *font, const char *text) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();

    display.Clear(0, 0, 0, 1);

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
    ImGui::Begin("loading", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings |
                 ImGuiWindowFlags_NoBringToFrontOnFocus);
    ImGui::PushFont(font);
    const ImVec2 ts = ImGui::CalcTextSize(text);
    ImGui::SetCursorPos(ImVec2((ImGui::GetWindowWidth()  - ts.x) * 0.5f,
                               (ImGui::GetWindowHeight() - ts.y) * 0.5f));
    ImGui::TextUnformatted(text);
    ImGui::PopFont();
    ImGui::End();

    // Leave GL state as the main loop's ImGui pass expects (no bound program).
    glUseProgram(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    display.SwapBuffers();
}

// The heavy phase (max_height + root terrain + shells): `sync` builds inline
// (solid from frame one), the rest defer to the worker. The star is always sync,
// at most two planets are.
void postHeavyPhase(System &sys, const std::vector<TerrainBody *> &sync,
                    JobRunner &jobs, Shader *atmosphereshader,
                    Shader *cloudshader, Shader *oceanshader,
                    Shader *ringshader, int cloudres) {
    std::vector<TerrainBody *> isSync;
    isSync.push_back(sys.root);   // the star, always solid
    for(TerrainBody *b : sync) {
        if(b == nullptr || b == sys.root) { continue; }
        bool dup = false;
        for(TerrainBody *e : isSync) { if(e == b) { dup = true; break; } }
        if(!dup && isSync.size() < 3) { isSync.push_back(b); }
    }
    auto inSync = [&](TerrainBody *b) {
        for(TerrainBody *e : isSync) { if(e == b) { return true; } }
        return false;
    };
    {   // e2e anchor: the sync set is solid from frame one
        std::string names;
        for(TerrainBody *e : isSync) { if(!names.empty()) { names += ", "; }
            names += e->name; }
        printf("[heavy] sync: %s (%zu deferred)\n", names.c_str(),
               sys.bodies.size() - isSync.size());
        fflush(stdout);
    }
    for(TerrainBody *b : sys.bodies) {
        if(inSync(b)) {
            b->Finish(b->BuildRootGeoms(), atmosphereshader, cloudshader,
                      cloudres, oceanshader, ringshader, jobs);
        } else {
            const std::string label = std::string("Terrain (") + b->name + ")";
            // The worker body uses only `b` (BuildRootGeoms is pure); the rest
            // feed the main-thread continuation.
            jobs.post(label,
                [b, atmosphereshader, cloudshader, cloudres, oceanshader,
                 ringshader, &jobs]() -> std::function<void()> {
                auto r = b->BuildRootGeoms();   // worker: pure math
                return [b, r, atmosphereshader, cloudshader, cloudres,
                        oceanshader, ringshader, &jobs]() {
                    b->Finish(r, atmosphereshader, cloudshader, cloudres,
                              oceanshader, ringshader, jobs);
                };
            });
        }
    }
}

// Split "name,def,body,scenario[,site]" into four required non-empty fields
// plus the optional pad id; false if the shape is wrong.
static bool splitStartship(const std::string &spec, DebugStartShip &out) {
    std::vector<std::string> f;
    std::string cur;
    for(char c : spec) {
        if(c == ',') { f.push_back(cur); cur.clear(); }
        else { cur += c; }
    }
    f.push_back(cur);
    if(f.size() != 4 && f.size() != 5) { return false; }
    for(const std::string &x : f) { if(x.empty()) { return false; } }
    out.name = f[0];
    out.ship = f[1];
    out.body = f[2];
    out.scenario = f[3];
    if(f.size() == 5) { out.site = f[4]; }
    return true;
}

/* Register the CLI headless hooks (cli.h). Only armed ones are registered;
   within a phase the registration order IS the firing order, and the e2e
   cases lean on it (e.g. --eva before --take). Each act() runs at fire time,
   against that frame's state. The VAB hooks join the same pre-scene phase
   (vabInstallHooks, called right after this). */
static void installHeadlessHooks(Game &game) {
    GameArgs &args = game.args;

    // --reload: before the scene is read -- a load decides the scene.
    if(!args.reload_dir.empty() && args.reload_ms >= 0) {
        const std::string dir = args.reload_dir;
        game.addHook(kHookPreScene, args.reload_ms, nullptr,
                     [dir](Game &g) { g.loadFrom(dir); });
    }
    // --new-game: Game::newGame from the title screen, before the scene read.
    if(args.new_game_ms >= 0) {
        game.addHook(kHookPreScene, args.new_game_ms, nullptr,
                     [](Game &g) { g.newGame(); });
    }
    // --quit-title: before the scene is read (it tears the fleet down and
    // lands on Title).
    if(args.quit_title_ms >= 0) {
        game.addHook(kHookPreScene, args.quit_title_ms, nullptr,
                     [](Game &g) { g.quitToTitle(); });
    }
    // --space-center: only from Flight (the hub is an excursion above a
    // running game).
    if(args.space_center_ms >= 0) {
        game.addHook(kHookPreScene, args.space_center_ms, nullptr, [](Game &g) {
            if(sceneIs(g, SceneId::Flight)) { pushScene(g, SceneId::SpaceCenter); }
        });
    }
    // --recover: fired after --space-center.
    if(args.recover_ms >= 0) {
        game.addHook(kHookPreScene, args.recover_ms, nullptr, [](Game &game) {
            // The active ship may be a free EVA kerbal, which recoverActive
            // refuses -- recover the ship holding the findings.
            if(game.ship != nullptr && game.ship->isEva()) {
                for(auto *s : game.vehicles) {
                    if(s->isEva()) { continue; }
                    bool holdsFinding = false;
                    for(Part *p : s->parts) {
                        if(!p->experiments.empty()) { holdsFinding = true; break; }
                    }
                    if(holdsFinding) { game.select_ship(s); break; }
                }
            }
            game.recoverActive();
        });
    }
    // --experiment: mirrors the UI -- a walking kerbal runs it itself, else
    // the active ship's first seat.
    if(args.experiment_ms >= 0) {
        game.addHook(kHookPreScene, args.experiment_ms, nullptr, [](Game &game) {
            if(game.ship == nullptr) {
                printf("[hook] --experiment: no kerbal, ignored\n");
            } else if(game.ship->isEva()) {
                game.runExperiment(static_cast<Kerbal *>(game.ship));
            } else {
                std::vector<Part *> seats = shipSeats(game.ship);
                if(seats.empty()) { printf("[hook] --experiment: no kerbal, ignored\n"); }
                else { game.runSeatExperiment(seats.front()); }
            }
        });
    }
    // --pod-experiment: the active ship's first experiment-family part + its
    // first seat.
    if(args.pod_experiment_ms >= 0) {
        game.addHook(kHookPreScene, args.pod_experiment_ms, nullptr, [](Game &game) {
            Part *pod = nullptr;
            std::string runner;
            if(game.ship != nullptr) {
                for(Part *p : game.ship->parts) {
                    if(p->def != nullptr && !p->def->experiment_family.empty()) {
                        pod = p; break;
                    }
                }
                std::vector<Part *> seats = shipSeats(game.ship);
                if(!seats.empty()) { runner = seatName(seats.front()); }
            }
            if(pod != nullptr && !runner.empty()) { game.runPodExperiment(pod, runner); }
            else { printf("[hook] --pod-experiment: no pod+kerbal, ignored\n"); }
        });
    }
    // --eva: Game::kerbalEVA on the active ship's first seat. Fired before
    // --take / --store in the e2e.
    if(args.eva_ms >= 0) {
        game.addHook(kHookPreScene, args.eva_ms, nullptr, [](Game &game) {
            std::vector<Part *> seats = game.ship != nullptr ? shipSeats(game.ship)
                                                             : std::vector<Part *>{};
            if(!seats.empty()) { game.kerbalEVA(seats.front()); }
            else { printf("[hook] --eva: no crew aboard, ignored\n"); }
        });
    }
    // --board: the reverse of --eva, so one process round-trips a person
    // through their seat.
    if(args.board_ms >= 0) {
        game.addHook(kHookPreScene, args.board_ms, nullptr, [](Game &game) {
            Vehicle *ship = nullptr;
            size_t part = 0;
            Kerbal *k = nullptr;
            /* Any vessel with a free seat, not just the active one: after an
               EVA the walker stands beside the ship it left, and the e2e may
               have switched control since. The walking crew are collected once:
               freeKerbals walks the whole fleet, so calling it per candidate
               part is quadratic in a fleet that is only getting bigger. */
            const std::vector<Kerbal *> freeK = freeKerbals(game.vehicles);
            for(auto *s : game.vehicles) {
                for(size_t i = 0; i < s->parts.size(); i++) {
                    Part *cap = s->parts[i];
                    if(cap->def == nullptr || cap->def->crew_capacity <= 0) { continue; }
                    if(cap->seatCount() >= cap->def->crew_capacity) { continue; }
                    for(Kerbal *f : freeK) {
                        if(game.kerbalInRange(f, cap)) {
                            ship = s; part = i; k = f; break;
                        }
                    }
                    if(k != nullptr) { break; }
                }
                if(k != nullptr) { break; }
            }
            if(k != nullptr) { game.kerbalBoard(k, ship, part); }
            else { printf("[hook] --board: no kerbal in reach of a free seat, ignored\n"); }
        });
    }
    // --take: move the first held finding off its instrument onto its courier
    // (the kerbal's suit).
    if(args.take_ms >= 0) {
        game.addHook(kHookPreScene, args.take_ms, nullptr, [](Game &game) {
            Part *from = nullptr;   // the instrument holding a finding to take
            Part *to = nullptr;     // the courier (the kerbal's suit)
            // After an EVA the instrument sits on a DIFFERENT ship than the
            // active one: search the whole fleet.
            for(auto *s : game.vehicles) {
                for(Part *p : s->parts) {
                    if(p->def != nullptr && !p->experiments.empty()
                       && p->def->experiment_storage == ExpStorage::Instrument) {
                        from = p; break;
                    }
                }
                if(from != nullptr) { break; }
            }
            // Needs a FREE (EVA) kerbal in reach of the part -- the Take
            // button's gate (kerbalInRange).
            if(from != nullptr) {
                for(Kerbal *k : freeKerbals(game.vehicles)) {
                    if(game.kerbalInRange(k, from) && !k->parts.empty()
                       && k->parts[0]->canHold(from->experiments[0])) {
                        to = k->parts[0]; break;
                    }
                }
            }
            if(from != nullptr && to != nullptr) { game.moveExperiment(from, to, 0); }
            else { printf("[hook] --take: no instrument/free-kerbal-in-reach, ignored\n"); }
        });
    }
    // --store: move the courier's first held finding onto its container
    // (the capsule).
    if(args.store_ms >= 0) {
        game.addHook(kHookPreScene, args.store_ms, nullptr, [](Game &game) {
            Part *from = nullptr;   // the courier (the kerbal's suit)
            Part *to = nullptr;     // the container (the capsule)
            // After an EVA the container sits on a DIFFERENT ship: search the
            // whole fleet.
            for(auto *s : game.vehicles) {
                for(Part *p : s->parts) {
                    if(p->def != nullptr
                       && p->def->experiment_storage == ExpStorage::Container) {
                        to = p; break;
                    }
                }
                if(to != nullptr) { break; }
            }
            // A FREE (EVA) kerbal in reach of the capsule, carrying a finding --
            // the Store button's gate.
            if(to != nullptr) {
                for(Kerbal *k : freeKerbals(game.vehicles)) {
                    if(game.kerbalInRange(k, to) && !k->parts.empty()
                       && !k->parts[0]->experiments.empty()) {
                        from = k->parts[0]; break;
                    }
                }
            }
            if(from != nullptr && to != nullptr) { game.moveExperiment(from, to, 0); }
            else { printf("[hook] --store: no free-kerbal-in-reach/container, ignored\n"); }
        });
    }
    // --tracking: fired after --space-center.
    if(args.tracking_ms >= 0) {
        game.addHook(kHookPreScene, args.tracking_ms, nullptr,
                     [](Game &g) { pushScene(g, SceneId::TrackingStation); });
    }
    // --tracking-close: popScene, fired after --tracking.
    if(args.tracking_close_ms >= 0) {
        game.addHook(kHookPreScene, args.tracking_close_ms, nullptr, [](Game &g) {
            if(sceneIs(g, SceneId::TrackingStation)) { popScene(g); }
            else { printf("[hook] --tracking-close: not in the tracking station, ignored\n"); }
        });
    }
    // --research: fired after --space-center.
    if(args.research_ms >= 0) {
        game.addHook(kHookPreScene, args.research_ms, nullptr,
                     [](Game &g) { pushScene(g, SceneId::ResearchLab); });
    }
    // --research-close: popScene, fired after --research.
    if(args.research_close_ms >= 0) {
        game.addHook(kHookPreScene, args.research_close_ms, nullptr, [](Game &g) {
            if(sceneIs(g, SceneId::ResearchLab)) { popScene(g); }
            else { printf("[hook] --research-close: not in the research lab, ignored\n"); }
        });
    }
    /* --atlas-dump: the System Atlas as text; reads the system, not the
       window, so it works from any scene. The e2e cover for the dossier
       (imgui Text rows are not clickable, so --ui-list cannot see them). */
    if(args.atlas_dump_ms >= 0) {
        game.addHook(kHookPreScene, args.atlas_dump_ms, nullptr,
                     [](Game &g) { dumpAtlas(g); });
    }
    /* --switch-system: Game::switchSystem. Before the scene is read -- it
       tears the current system down and lands on Title. */
    if(!args.switch_system_path.empty() && args.switch_system_ms >= 0) {
        const std::string path = args.switch_system_path;
        game.addHook(kHookPreScene, args.switch_system_ms, nullptr, [path](Game &g) {
            // A bad path throws from load_system: catch it, keep running on the
            // current system.
            try {
                g.switchSystem(path);
            } catch(const std::exception &e) {
                printf("[switch] cannot switch to '%s': %s\n",
                       path.c_str(), e.what());
                fflush(stdout);
                g.toast("Switch failed: %s", e.what());
            }
        });
    }
    /* --select-target: the Target window's combo, headless. The phase keeps
       the old poll site (after sc.update): a vessel deleted earlier in the
       frame has already lost its xferTargets rows (Game::dropVehicleRefs),
       which is what keeps the name lookup off a borrowed name pointer (it
       points into the Vehicle). */
    for(const GameArgs::SelectTargetArg &a : args.select_targets) {
        const std::string name = a.name;
        game.addHook(kHookPostUpdate, a.ms, nullptr,
                     [name](Game &game) {
                game.xferPlanner.selectTargetByName(name == "none" ? std::string()
                                                                   : name);
                // Say what the planner actually took: a name that is not a
                // target now is dropped, not queued, so the case author sees
                // a typo.
                const int idx = game.xferPlanner.xfer_target;
                const std::string where = idx >= 0
                    ? " (index " + std::to_string(idx) + ")"
                    : " (not in the target list)";
                printf("[target] hook: selected \"%s\"%s\n",
                       name.c_str(), where.c_str());
                fflush(stdout);
            });
    }
    /* --map-dump: the orbit map's plane basis per combo slot. Its phase is
       AFTER the state pass -- sc.refresh is what fills Game::hud.view, so
       dumping before it silently reports a plausible-but-wrong basis. Needs
       an active ship. */
    if(args.map_dump_ms >= 0) {
        game.addHook(kHookPostRefresh, args.map_dump_ms, nullptr,
                     [](Game &g) { dumpMapBasis(g); });
    }
}

int main(int argc, char **argv)
{
    const auto prog_start = std::chrono::steady_clock::now();

    GameArgs args;
    int exit_code = 1;
    if(!parse_cli(argc, argv, args, &exit_code)) { return exit_code; }

    // saves/ + settings.json live in the per-OS data dir (datadir.h,
    // --data-dir overrides). Must run before the settings load.
    datadir::init(args.data_dir);

    // settings.json phase 1: these fields must reach window creation -- the
    // mode/size and the MSAA count are fixed in the GLX visual, and setWindowMode
    // can't change the MSAA after. CLI flags win field by field.
    load_settings_args(args);

    /* The glGetError drains (gldebug.h) are opt-in: --gl-debug turns them on with
       the debug callback, --gl-error-checks turns on ONLY the drains -- the only
       producer of the "GL_<ENUM> - file:line" lines the e2e GL_-forbidding cases
       assert on. Set before the Renderer so its own sites are covered. */
    set_gl_error_checks(args.gl_debug || args.gl_error_checks);

    Renderer display(args.screen_width, args.screen_height, args.window_mode,
                     args.msaa_samples, args.gl_debug, args.vsync);
    check_gl_error();
    const Uint32 sim_win_id = SDL_GetWindowID(display.get_display());
    /* --sim-press: keycodes -> scancodes now SDL is up (the CLI parse ran before
       the video subsystem existed). */
    for(auto &p : args.sim_presses) {
        p.sc = SDL_GetScancodeFromKey(p.key, nullptr);
        if(p.sc == SDL_SCANCODE_UNKNOWN) {
            printf("warning: --sim-press key %d has no scancode in the "
                   "current keyboard layout: one-shot actions fire, held "
                   "commands for it do not\n", (int)p.key);
        }
    }
    ImGuiContext* ctx1 = ImGui::CreateContext();
    ImGui::SetCurrentContext(ctx1);
    // ImPlot keeps its own state per imgui context.
    ImPlot::CreateContext();
    ImGui_ImplSDL3_InitForOpenGL(display.get_display(), SDL_GL_GetCurrentContext());
    ImGui_ImplOpenGL3_Init("#version 430");
    check_gl_error();

    ImGuiIO& io = ImGui::GetIO();
    // No imgui.ini: a saved layout would clobber the per-frame layout.
    io.IniFilename = nullptr;
    // Normal and big faces are the same font, the big one at 2x.
    // GlyphExtraAdvanceX is letter tracking (map-label clearance is a separate
    // knob in gameui.cpp).
    const float glyph_extra_advance_x = 0.0f;
    ImFontConfig font_cfg;
    font_cfg.GlyphExtraAdvanceX = glyph_extra_advance_x;
    /* Check --font here: imgui answers a missing TTF with an assert + SIGABRT
       (#16). */
    {
        const std::string fontfile = resdir::path(args.font_path);
        if(!std::filesystem::exists(fontfile)) {
            printf("error: --font '%s' not found (looked for %s)\n",
                   args.font_path.c_str(), fontfile.c_str());
            return 1;
        }
    }
    io.Fonts->AddFontFromFileTTF(resdir::path(args.font_path).c_str(),
                                 args.font_size, &font_cfg);
    // Big face (2x) for the HUD + main menu.
    ImFont *bigger = io.Fonts->AddFontFromFileTTF(
        resdir::path(args.font_path).c_str(), 2.0f * args.font_size,
        &font_cfg);
    check_gl_error();

    // The presented frame persists through the init steps that don't present.
    if(pumpLoadingQuit()) { return 1; }
    drawLoadingFrame(display, bigger, "loading...");

    // start bullet; see physics.cpp
    create_physics();
    check_gl_error();

    /* get_shader registry owns these: compiled once, shared, never deleted */
    Shader *partsshader = get_shader("res/shaders/partsShader",
                                     { "position", "uv", "normal" },
                                     { "MVP", "Normal", "lightDirection", "shadow",
                                       "alpha", "tint", "flatLight" });

    // Specified once in shader.cpp: switchSystem needs the same programs, and
    // get_shader's FIRST registration wins -- a second copy could silently
    // disagree about uniform indices.
    const WorldShaders world = get_world_shaders();

    Shader *skyboxshader = get_shader("res/shaders/skyboxShader",
                                      { "position" },
                                      { "projectionview", "gain", "skybox" });
    /* The cubemap sampler unit is program state, so pin it once here instead of
       resolving the location every frame (a linked sampler defaults to 0, so this
       buys explicitness, not a fix). glUniform needs the program CURRENT: bind for
       the write and restore, or Mesa answers GL_INVALID_OPERATION (e2e 14-postfx
       catches it). */
    {
        GLint prev_program = 0;
        glGetIntegerv(GL_CURRENT_PROGRAM, &prev_program);
        skyboxshader->Bind();
        skyboxshader->setUniform_i("skybox", 0);
        glUseProgram(prev_program);
    }
    // Skybox::Draw binds the texture to unit 0.

    Shader *lineshader = get_shader("res/shaders/lineShader2",
                                    { "position" },
                                    { "MVP", "color" });

    PostFX *postfx = new PostFX;
    // Create every effect up front: no mid-frame shader compilation when
    // Settings toggles one.
    for(const std::string &name : PostFX::Available()) {
        postfx->AddEffect(name);
    }
    // Each --postfx value may itself be comma-separated.
    std::vector<std::string> fx_names;
    for(const std::string &spec : args.postfx_spec) {
        size_t start = 0;
        while(start <= spec.size()) {
            size_t comma = spec.find(',', start);
            std::string name = spec.substr(start, comma == std::string::npos
                                           ? std::string::npos
                                           : comma - start);
            size_t b = name.find_first_not_of(" \t");
            size_t e = name.find_last_not_of(" \t");
            name = (b == std::string::npos) ? "" : name.substr(b, e - b + 1);
            if(!name.empty()) {
                if(!postfx->SetEnabled(name, true)) {
                    printf("error: unknown --postfx effect '%s' (available: ",
                           name.c_str());
                    const std::vector<std::string> &avail = PostFX::Available();
                    for(size_t i = 0; i < avail.size(); i++) {
                        printf("%s%s", i ? ", " : "", avail[i].c_str());
                    }
                    printf(")\n");
                    return 1;
                }
                fx_names.push_back(name);
            }
            if(comma == std::string::npos) break;
            start = comma + 1;
        }
    }
    if(!fx_names.empty()) {
        printf("postfx: %s", fx_names[0].c_str());
        for(size_t i = 1; i < fx_names.size(); i++) {
            printf(" -> %s", fx_names[i].c_str());
        }
        printf("\n");
    }
    postfx->Resize(display.get_width(), display.get_height());

    // Redraws the "loading..." label after each body's LIGHT phase, throttled:
    // each draw ends in a SwapBuffers that blocks on vsync.
    auto last_loading_draw = std::chrono::steady_clock::time_point{};
    System sys;
    try {
        sys = load_system(args.system_file.c_str(), world.terrain, world.sun,
            [&](size_t i, size_t total, const std::string &name) {
                if(pumpLoadingQuit()) { std::exit(1); }   // window closed mid-load
                auto now = std::chrono::steady_clock::now();
                if(now - last_loading_draw < std::chrono::milliseconds(100)
                   && i + 1 < total) {
                    return;   // not time for another frame yet
                }
                last_loading_draw = now;
                char buf[160];
                snprintf(buf, sizeof(buf), "loading %s  %zu / %zu",
                         name.c_str(), i + 1, total);
                drawLoadingFrame(display, bigger, buf);
            });
    } catch(const std::exception &e) {
        // The loader throws on data bugs: report and leave rather than let the
        // throw reach std::terminate.
        printf("error: %s\n", e.what());
        fflush(stdout);
        return 1;
    }
    TerrainBody *sun = sys.root;
    TerrainBody *home = sys.home;   // VAB launch body + title backdrop

    // The star field comes from the system JSON, so this precedes the boot
    // --load path, whose ensureSystemForSave may switch systems.
    Skybox skybox;
    skybox.init();
    skybox.load(sys.skybox_faces);

    /* Experiment family table (res/data/experiments.json). Loaded before the
       ships so the cross-check below has it. */
    loadExperimentDefs(resdir::path(args.experiments_file).c_str());

    /* The kerbal roster (res/data/kerbals.json): the named people a capsule
       seat draws from. A data bug is reported like the system loader's rather
       than allowed to reach std::terminate. */
    CrewRoster roster;
    try {
        roster = load_crew_roster(resdir::path(args.kerbals_file).c_str());
    } catch(const std::exception &e) {
        printf("error: %s\n", e.what());
        fflush(stdout);
        return 1;
    }
    printf("Roster: %zu kerbal(s) from %s\n", roster.profiles.size(),
           args.kerbals_file.c_str());

    /* parts.json gives mass + behavior, the defs give stack order + offsets.
       Ships sharing a (body, scenario) get spaced apart; each pad ship stands
       on its named site, else the body's default. */
    Ships ships(args.parts_file, partsshader, sun, roster);

    /* World buildings: placed once physics + the part shader are live, before
       any ship stands on a pad. */
    BuildingsCatalog buildings =
        load_buildings_catalog(resdir::path("res/data/buildings.json").c_str());
    place_world_buildings(sys, buildings, partsshader, sun);

    /* Unassigned kerbals stand on the home pad (the building pool). A --load
       replaces this with the save's crew_pool. */
    if(args.load_name.empty()) {
        ships.mintCrewPool(roster, sys);
    }

    /* A family with no def silently falls back to generic behavior -- a
       parts.json typo should not hide. The suit's family is hard-coded in
       game.cpp, so check it too. */
    for(const PartDef &p : ships.catalog().parts) {
        if(!p.experiment_family.empty() && defFor(p.experiment_family) == nullptr) {
            printf("warning: part '%s' has experiment_family '%s' with no def in %s\n",
                   p.name.c_str(), p.experiment_family.c_str(),
                   args.experiments_file.c_str());
        }
    }
    if(defFor("observation") == nullptr) {
        printf("warning: the kerbal suit's hard-coded family 'observation' has no def in %s\n",
               args.experiments_file.c_str());
    }

    // The running game: borrows the subsystems above, owns the runtime state.
    Game game(display, postfx, ships, roster, sys, sun, home, args, sim_win_id);
    game.bigger = bigger;   // the UI pass draws with it
    game.skybox = &skybox;  // set before any boot --load can switch systems
    game.buildings = std::move(buildings);   // switchSystem re-places pads
    // Set before the boot --load so ensureSystemForSave sees it.
    game.systemPath = args.system_file;

    // Silent no-op with no device/assets. --audio-log must be set before
    // init(), where the trace prints.
    game.audio.setDebug(args.audio_log);
    if(game.audio.init()) {
        game.audio.setMusic("res/audio/ville_seppanen-1_g.ogg");
    }

    // settings.json phase 2: the Game + PostFX state, before apply_ui_style.
    game.load_settings();
    game.apply_ui_style();

    // --start-time: must happen before the fleet spawns -- the orbit scenarios
    // read the home body's frame state. (--load overwrites it.)
    game.setTime(args.start_time);
    if(args.start_time > 0.0) {
        printf("Starting at sim time t = %.0f s\n", args.start_time);
    }

    // Aliases into game's members for the loop body below.
    Vehicle *&ship = game.ship;
    int &time_accel = game.time_accel;
    int &cam_speed = game.cam_speed;
    bool &poly_mode = game.poly_mode;
    bool &screenshot_requested = game.screenshot_requested;
    bool &running = game.running;

    std::vector<DebugStartShip> start_ships;
    if(!args.startships_file.empty()) {
        // file form: a JSON list (loadDebugStartShips validates the fields)
        try {
            start_ships =
                loadDebugStartShips(resdir::path(args.startships_file).c_str())
                    .ships;
        } catch(const std::exception &e) {
            printf("error: %s\n", e.what());
            return 1;
        }
    } else if(!args.startship.empty()) {
        // inline form: each --startship is "name,def,body,scenario[,site]"
        for(size_t i = 0; i < args.startship.size(); i++) {
            DebugStartShip e;
            if(!splitStartship(args.startship[i], e)) {
                printf("error: --startship #%zu '%s': expected "
                       "name,def,body,scenario[,site] (four or five non-empty "
                       "fields)\n",
                       i + 1, args.startship[i].c_str());
                return 1;
            }
            start_ships.push_back(e);
        }
    }
    // Neither -> no start ships: boots to the title screen.

    /* Split the heavy phase: build the bodies the player is ON synchronously so
       the first frame is solid, defer the rest to the worker. The sync set is
       the player's bodies, NOT the system home -- see the branches below. */
    Vehicle *first = nullptr;
    if(!args.load_name.empty()) {
        // --load: the saved fleet replaces the built one, each ship in the world
        // state it was saved in (live or railed).
        //
        // A bare slot name resolves under the data dir's saves/ when the given
        // path has no save.json.
        std::string load_dir = args.load_name;
        if(!std::filesystem::exists(load_dir + "/save.json")) {
            load_dir = find_slot(datadir::saves(), load_dir);
            if(load_dir.empty()) {
                printf("Load: no save named '%s' under %s (or ambiguous "
                       "across games)\n",
                       args.load_name.c_str(), datadir::saves().c_str());
                exit(1);
            }
            printf("Load: using saves slot '%s'\n", load_dir.c_str());
        }
        game.partsshader = partsshader;   // load_game builds parts with it
        // Honor the save's system: a solar save loaded into KSP must land on
        // Earth, not Kerbin. A switch runs its own heavy phase.
        bool switched = false;
        if(!game.ensureSystemForSave(load_dir, &switched)) {
            exit(1);   // reason already printed + toasted by ensureSystemForSave
        }
        if(!switched) {
            // Boot, not a runtime reload: sync the save's ship bodies present
            // in THIS system, plus home.
            std::vector<TerrainBody *> sync;
            for(const std::string &n : saveShipBodies(load_dir)) {
                if(TerrainBody *b = sys.find(n)) { sync.push_back(b); }
            }
            sync.push_back(home);
            postHeavyPhase(sys, sync, game.jobs, world.atmosphere, world.cloud,
                           world.ocean, world.ring, args.cloud_mesh);
        }
        try {
            load_game(game, load_dir);
        } catch(const std::exception &e) {
            printf("Load failed: %s\n", e.what());
            exit(1);
        }
        first = game.ship;
        check_gl_error();
        ship = first;
        // switchSystem DELETED the old bodies, so the boot-time `sun`/`home`
        // locals dangle -- re-point at the live system.
        sun = game.sun;
        home = game.home;
        // switchSystem landed on the Title: a loaded fleet resumes in flight,
        // a shipless save at the hub.
        if(ship != nullptr) { enterFlight(game); }
        else { enterSpaceCenter(game); }
    } else {
        std::vector<TerrainBody *> sync;
        if(!args.radial_test.empty() || !args.dock_test.empty()) {
            sync.push_back(home);   // the test ships sit here
        } else if(!args.vab.empty() || args.vab_empty) {
            TerrainBody *vb = args.vab_body.empty()
                             ? home : sys.find(args.vab_body);
            sync.push_back(vb != nullptr ? vb : home);   // the launch body
        } else if(!start_ships.empty()) {
            for(const DebugStartShip &fe : start_ships) {
                sync.push_back(sys.find(fe.body));   // required, non-empty
            }   // postHeavyPhase dedupes the sync set and caps it at two
        } else {
            sync.push_back(game.pickTitleBody());   // the backdrop, below
            sync.push_back(home);   // the hub: the first launch leaves from it
        }
        postHeavyPhase(sys, sync, game.jobs, world.atmosphere, world.cloud,
                       world.ocean, world.ring, args.cloud_mesh);

        if(!args.radial_test.empty()) {
            RadialTestShip rts = build_radial_test_ship(
                args.radial_test, ships.catalog(), home, sun, partsshader);
            ships.add_ship(game.vehicles, rts.v, home, rts.sc, game.time);
            first = rts.v;
        } else if(!args.dock_test.empty()) {
            DockTestShips dts = build_dock_test_ships(
                args.dock_test, ships.catalog(), home, sun, partsshader, sys);
            /* Both are already placed (the builder ran spawn_vehicle), so the null
               scenario means apply_scenarios skips them. The probe is active; the
               station rides on rails. */
            ships.add_ship(game.vehicles, dts.probe, home, nullptr, game.time);
            ships.add_ship(game.vehicles, dts.station, home, nullptr, game.time);
            first = dts.probe;
        } else {
            // A typo in any field throws deep in here: report it, don't core
            // dump.
            try {
                first = ships.buildDebugStartShips(start_ships, sys,
                                                   game.vehicles, game.time);
            } catch(const std::exception &e) {
                printf("error: %s\n", e.what());
                return 1;
            }
        }
    }

    if(args.load_name.empty()) {
        check_gl_error();
        // Scenarios first (they position the ships), then park the idle ones --
        // both before the camera exists, so it focuses on the spawn.
        game.settleFleet(first);
        /* The active ship: the first built. Assigned directly, not via
           select_ship: the camera does not exist yet. */
        ship = first;
        if(first != nullptr) {
            printf("[dbg-dv] getMass=%.2f kg  getDeltaV=%.1f m/s\n",
                   (double)first->getMass(), (double)first->getDeltaV());
        }
        /* --autopilot: engage a slew mode (test hook for the Autopilot toggles).
           slewRequest is held until toggled. */
        if(!args.autopilot.empty() && first != nullptr) {
            int m = 0;  // SlewMode (vehicle.h)
            if(args.autopilot == "prograde")        { m = 1; }
            else if(args.autopilot == "retrograde") { m = 2; }
            else if(args.autopilot == "radial-out") { m = 3; }
            else if(args.autopilot == "radial-in")  { m = 4; }
            else if(args.autopilot == "normal")     { m = 5; }
            else if(args.autopilot == "anti-normal"){ m = 6; }
            else if(args.autopilot == "kill-rot")   { m = 7; }
            first->setSlewRequest((SlewMode)m);
            printf("Autopilot: %s engaged at startup\n", args.autopilot.c_str());
        }
    }

    Mesh *engine_plume_mesh = get_mesh("res/meshes/engine_plume.obj");
    Texture *engine_plume_texture = get_texture("res/textures/engine_plume.png");

    Shader *billboardshader = get_shader("res/shaders/billboardshader",
                                         { "position", "texcoord", "normal" },
                                         { "MVP", "color_uniform" });

    // Billboard icons opt out of mip chains: alpha cutouts bleed into the
    // neighbouring level when minified.
    Texture * front_indicator_texture = get_texture("res/textures/front_crosshair.png", false);
    Texture * prograde_indicator_texture = get_texture("res/textures/prograde_icon.png", false);
    Texture * retrograde_indicator_texture = get_texture("res/textures/retrograde_icon.png", false);
    Texture * radial_in_indicator_texture = get_texture("res/textures/radial_in_icon.png", false);
    Texture * radial_out_indicator_texture = get_texture("res/textures/radial_out_icon.png", false);
    Texture * normal_plus_indicator_texture = get_texture("res/textures/normal_plus_icon.png", false);
    Texture * normal_minus_indicator_texture = get_texture("res/textures/normal_minus_icon.png", false);

    // Marker hues: one per direction family, so the ring around the ship reads by
    // colour. billboardshader.fs tints with this uniform and ignores the texture
    // RGB, so the colour lives here, not in the PNGs. Prograde takes the orbit
    // map's green (col_ship) so world and map agree.
    const glm::vec4 front_color       = glm::vec4(1.00f, 1.00f, 1.00f, 1.0f);
    const glm::vec4 prograde_color    = glm::vec4(0.20f, 0.80f, 0.40f, 1.0f);
    const glm::vec4 retrograde_color  = glm::vec4(1.00f, 0.25f, 0.25f, 1.0f);
    const glm::vec4 radial_color      = glm::vec4(0.75f, 0.35f, 1.00f, 1.0f);
    const glm::vec4 normal_color      = glm::vec4(0.30f, 0.55f, 1.00f, 1.0f);

    Billboard *front_indicator =
        mk_billboard(billboardshader, front_indicator_texture, 1.0, 1.0, front_color);
    Billboard *prograde_indicator =
        mk_billboard(billboardshader, prograde_indicator_texture, 1.0, 1.0, prograde_color);
    Billboard *retrograde_indicator =
        mk_billboard(billboardshader, retrograde_indicator_texture, 1.0, 1.0, retrograde_color);
    Billboard *radial_in_indicator =
        mk_billboard(billboardshader, radial_in_indicator_texture, 1.0, 1.0, radial_color);
    Billboard *radial_out_indicator =
        mk_billboard(billboardshader, radial_out_indicator_texture, 1.0, 1.0, radial_color);
    Billboard *normal_plus_indicator =
        mk_billboard(billboardshader, normal_plus_indicator_texture, 1.0, 1.0, normal_color);
    Billboard *normal_minus_indicator =
        mk_billboard(billboardshader, normal_minus_indicator_texture, 1.0, 1.0, normal_color);
    // Transfer burn direction: the prograde icon in amber, where the departure
    // burn should point. Not blue -- blue belongs to normal now.
    Billboard *burn_indicator =
        mk_billboard(billboardshader, prograde_indicator_texture, 1.0, 1.0,
                     glm::vec4(0.95f, 0.70f, 0.15f, 1.0f));
    // Target relative velocity, two cyan markers: prograde icon for you −
    // target, retrograde icon for target − you.
    const glm::vec4 relvelcolor = glm::vec4(0.15f, 0.85f, 1.0f, 1.0f);
    Billboard *relvel_indicator =
        mk_billboard(billboardshader, prograde_indicator_texture, 1.0, 1.0, relvelcolor);
    Billboard *relvel_retro_indicator =
        mk_billboard(billboardshader, retrograde_indicator_texture, 1.0, 1.0, relvelcolor);

    const float camFov = (float)glm::radians(args.camFovDeg);
    // The drawable size the Renderer actually got (the WM may have clamped it).
    const float camAspect = (float)display.get_width() / (float)display.get_height();
    /* zNear = 1 m clips anything nearer than that, which CAM_FIRST hits in anger:
       the eye sits on the controller's top face (render.cpp), so a part stacked
       above the pod vanishes. Lowering it is not free on the 24-bit unorm window
       buffer -- window depth is zNear/d, so it multiplies the far-field collapse
       by the same factor. Pairs with #263. */
    const float camZNear = 1.0f;
    // Stored only: the projection is infinite-far, and no shader reads a far
    // limit (camera.cpp reverseZInfinitePerspective ignores zFar).
    const float camZFar = 1e13;

    // One camera, two modes (orbit + free); the terrain LOD reads the live one.
    const glm::dvec3 camFocus = ship ? ship->partPos(ship->controller)
                                     : home->frame->root_pos;
    Camera *cam = new Camera(camFocus, camFov, camAspect, camZNear, camZFar);
    cam->setViewport(display.get_width(), display.get_height());
    game.camera = cam;
    // Orbit-camera targets. Seeded BEFORE the --vab / --free-cam framing below
    // so that framing stays the last word. A --load boot already synced the
    // "ship" entry, so insert only when absent.
    if(game.ship != nullptr &&
       (game.focusTargets.empty() || game.focusTargets[0].body != nullptr)) {
        game.focusTargets.push_back({ "ship", nullptr });
    }
    for (TerrainBody *b : sys.bodies) {
        game.focusTargets.push_back({ b->name.c_str(), b });
    }
    if(ship == nullptr && args.load_name.empty()) {
        /* No vessel and no loaded game: the floor scene is the TITLE screen.
           Decided before the --vab entry below so an editor opened with nothing
           to fly sits on [title, vab]. */
        enterTitle(game);
        game.parkTitleCamera();
        printf("[boot] no vessel: title screen\n");
        fflush(stdout);
    }

    /* --vab: open the editor with a def loaded as a physics-free build tree;
       paintVab draws only that. */
    if(!args.vab.empty()) {
        /* A bad --vab def is a harness typo: load_ship_def's throw must not reach
           the top of main as SIGABRT (#16). */
        try {
            ShipDef vdef = load_ship_def(resdir::path(args.vab).c_str(),
                                         ships.catalog());
            game.vab.build = BuildShip::fromShipDef(vdef);
        } catch(const std::exception &e) {
            printf("error: --vab: %s\n", e.what());
            return 1;
        }
        game.vab.armed = args.vab_arm;   // test hook: pre-arm a palette part
        vabOpen(game);
    } else if(args.vab_empty) {
        // --vab-empty: headless entry to the same editor as the menu's VAB.
        game.vab.armed = args.vab_arm;   // test hook: pre-arm a palette part
        vabOpen(game);
    }
    if(sceneIs(game, SceneId::Vab)) {
        /* --vab-scenario / --vab-body / --vab-site override the launch config.
           Pins a test relies on, so a miss is a hard error (#16): a silent
           fallback measures a launch nobody asked for. */
        if(!args.vab_scenario.empty()) {
            try {
                scenario_by_name(args.vab_scenario);
            } catch(const std::exception &e) {
                printf("error: --vab-scenario: %s\n", e.what());
                return 1;
            }
            game.vab.scenarioName = args.vab_scenario;
        }
        TerrainBody *vab_body = home;
        if(!args.vab_body.empty()) {
            TerrainBody *b = sys.find(args.vab_body);
            if(b == nullptr) {
                printf("error: --vab-body '%s' is not a body in %s\n",
                       args.vab_body.c_str(), game.systemPath.c_str());
                return 1;
            }
            vab_body = b;
            game.vab.bodyName = args.vab_body;
        }
        if(!args.vab_site.empty()) {
            // nullptr means "not a VAB_LAUNCH_PAD on the launch body":
            // find_launch_site defaults only for an empty id.
            if(vab_body == nullptr
               || find_launch_site(vab_body, args.vab_site) == nullptr) {
                printf("error: --vab-site '%s' is not a launch pad on %s\n",
                       args.vab_site.c_str(),
                       vab_body ? vab_body->name.c_str() : "<no launch body>");
                return 1;
            }
            game.vab.siteName = args.vab_site;
        }
    }

    // --surfmap-body: pin the Surface Map's body. Applied AFTER the --load
    // branch so it resolves against the system the boot runs. Hard error on a
    // miss, not a silent fallback (#72).
    if(!args.surfmap_body.empty()) {
        if(TerrainBody *b = sys.find(args.surfmap_body)) {
            game.surfmap.body = b;
        } else {
            printf("error: --surfmap-body '%s' is not a body in %s\n",
                   args.surfmap_body.c_str(), game.systemPath.c_str());
            return 1;
        }
    }

    if(args.use_free_cam) {
        // Default free pose = the orbit camera's current view, per-axis
        // overridable by --free-cam-pos/-fwd/-up.
        glm::dvec3 p = cam->GetPos();
        glm::dvec3 f = cam->GetForward();
        glm::dvec3 u = cam->up;
        if(args.free_cam_pos.size() == 3) {
            p = glm::dvec3(args.free_cam_pos[0], args.free_cam_pos[1], args.free_cam_pos[2]);
        }
        if(args.free_cam_fwd.size() == 3) {
            f = glm::dvec3(args.free_cam_fwd[0], args.free_cam_fwd[1], args.free_cam_fwd[2]);
        }
        if(args.free_cam_up.size() == 3) {
            u = glm::dvec3(args.free_cam_up[0], args.free_cam_up[1], args.free_cam_up[2]);
        }
        cam->setFreePose(p, f, u);
    }

    int screenshot_count = 0;
    SDL_SetWindowRelativeMouseMode(display.get_display(), false);

    // kRailsWarp: see game.h. A load starts paused (load_game sets 0); an
    // explicit --time-accel applies on both paths.
    if(args.load_name.empty() || args.cli_given.time_accel) {
        time_accel = args.initial_time_accel;
    }

    /* --time-accel is not bound by the warp ladder: clamp it, so a typo cannot
       dump an absurd amount of sim on one tick. */
    if(time_accel > kMaxWarp) {
        printf("Time accel %dx is above the warp ceiling; clamping to %dx\n",
               time_accel, kMaxWarp);
        time_accel = kMaxWarp;
    }

    /* Boot straight into rails warp: the active ship parks too (works on the pad
       -- the frozen mode), unless some ship is not rail-eligible, then clamp to
       the top physics warp. */
    if(time_accel >= kRailsWarp) {
        bool all_eligible = true;
        for(auto *s : game.vehicles) {
            if(!s->canRail()) {
                printf("Rails warp refused at start: '%s' is neither in free "
                       "fall nor grounded; clamping time accel to 10\n",
                       s->name.c_str());
                all_eligible = false;
                break;
            }
        }
        if(all_eligible) {
            for(auto *s : game.vehicles) { s->goOnRails(); }
        } else {
            time_accel = 10;
        }
    }
    cam_speed = 1;

    // Game owns the transfer planner: it holds sim-clock state, so the clock
    // hook can invalidate it.

    /* Reference overlays, all radius 1000 in the render frame's units and drawn
       with the line shader (render.cpp): XZ (normal +Y, equatorial) and XY
       (normal +Z, meridian) for the render frame's own planes, plus two ticks at
       the XY ring's +/-X. The XY ring and the ticks are also the orbital-plane
       overlay's ring and node markers, which rotate them by a basis built from
       the orbit normal -- so the XY ring's normal (+Z) is the axis the basis maps
       onto the plane normal. The markers are ticks, not a diameter: a line
       through the ring's centre points AT the camera (the rings are camera-
       centred), so it projects to a point.
       Two tick meshes, not one, because the overlay pins its basis to the node
       line: +X is the ASCENDING side, -X the descending one, and the orbit map
       colors those two green and red -- so the overlay says the same thing in
       the same colors. */
    Mesh *ref_circle_xz = new Mesh;
    Mesh *ref_circle_xy = new Mesh;
    Mesh *ref_node_an = new Mesh;
    Mesh *ref_node_dn = new Mesh;
    {
        const float r = 1000;
        const float tick = 40;
        const int n = 128;
        PosInterface xzinterface;
        PosInterface xyinterface;
        PosInterface aninterface;
        PosInterface dninterface;
        for(int i = 0; i < n; i++) {
            const double a = (2 * std::numbers::pi) * float(i)/float(n);
            xzinterface.positions.push_back(glm::vec3(r * cos(a), 0, r * sin(a)));
            xyinterface.positions.push_back(glm::vec3(r * cos(a), r * sin(a), 0));
        }
        // Span the ring's +Z, which the basis maps onto the plane normal: the
        // tick then sticks out of the ring line instead of hiding along it.
        aninterface.positions.push_back(glm::vec3(r, 0, -tick));
        aninterface.positions.push_back(glm::vec3(r, 0, tick));
        dninterface.positions.push_back(glm::vec3(-r, 0, -tick));
        dninterface.positions.push_back(glm::vec3(-r, 0, tick));
        ref_circle_xz->InitMesh(xzinterface);
        ref_circle_xy->InitMesh(xyinterface);
        ref_node_an->InitMesh(aninterface);
        ref_node_dn->InitMesh(dninterface);
    }

    game.skyboxshader = skyboxshader;
    game.lineshader = lineshader;
    game.partsshader = partsshader;
    game.engine_plume_mesh = engine_plume_mesh;
    game.engine_plume_texture = engine_plume_texture;
    game.ref_circle_xz = ref_circle_xz;
    game.ref_circle_xy = ref_circle_xy;
    game.ref_node_an = ref_node_an;
    game.ref_node_dn = ref_node_dn;
    game.front_indicator = front_indicator;
    game.prograde_indicator = prograde_indicator;
    game.retrograde_indicator = retrograde_indicator;
    game.radial_in_indicator = radial_in_indicator;
    game.radial_out_indicator = radial_out_indicator;
    game.normal_plus_indicator = normal_plus_indicator;
    game.normal_minus_indicator = normal_minus_indicator;
    game.burn_indicator = burn_indicator;
    game.relvel_indicator = relvel_indicator;
    game.relvel_retro_indicator = relvel_retro_indicator;

    // --timeout: wall-clock budget for the run (0 = run until closed).
    // events.cpp and the timeout check below both measure from this.
    game.loop_start_ms = SDL_GetTicks();

    game.sky_dim = args.sky_dim;
    game.sky_dim_cone = args.sky_dim_cone;

    // Headless hooks: registered once from the CLI args; the loop polls the
    // phases (Game::fireHooks) and the bodies live next to the registration.
    installHeadlessHooks(game);
    vabInstallHooks(game);
    if(args.map_plane >= 0) { game.map.plane = args.map_plane; }
    const double startup_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - prog_start).count();
    printf("Main loop starting: startup took %.3f s", startup_s);
    if(args.timeout_seconds > 0.0) {
        printf(" | auto-exit after %.1f s (wall clock)", args.timeout_seconds);
    }
    printf("\n");
    fflush(stdout);

    // --frame-cap: budget per loop iteration (0 = uncapped). The logic tick
    // runs off the wall clock regardless.
    const int cap_ms = (args.frame_cap > 0) ? (int)(1000.0 / (double)args.frame_cap) : 0;
    if (cap_ms > 0) {
        printf("frame cap: %d fps\n", args.frame_cap);
    } else {
        printf("frame cap: off (uncapped)\n");
    }
    printf("logic tick: %.0f Hz (%.1f ms)\n",
           args.physics_hz, 1000.0 / args.physics_hz);

    /* --idle-fps: while the window is occluded or minimized the loop idles
       instead of drawing into a swap that goes nowhere. Throttled, NOT skipped:
       the 3D pass is not only pixels -- sc.refresh re-poses the camera and runs
       the terrain LOD that owns the ground's collision bodies, so dropping a draw
       would change the simulation. #212 split that state from the paint so a
       later gate CAN drop pixels; --idle-fps does not use it. The logic tick is
       untouched, so a minimized window still warps at full rate; only one-shot
       presses quantise to the idle interval.
       Floor: tick.cpp's spiral-of-death clamp takes at most 10 ticks per frame,
       so an idle frame slower than 10/dt silently loses sim time -- hence the
       physics_hz/8 floor. This bounds IDLE_MS only: a --frame-cap slower than
       physics_hz/10 loses sim time hidden or not. */
    const double idle_fps = std::max((double)args.idle_fps,
                                     args.physics_hz / 8.0);
    const int idle_ms = (args.idle_fps > 0)
        ? std::max(1, (int)(1000.0 / idle_fps)) : 0;
    if(args.idle_fps > 0) {
        printf("idle: %d ms/frame while the window is occluded/minimized",
               idle_ms);
        if(idle_fps > (double)args.idle_fps) {
            printf(" (--idle-fps %d raised to %.1f: the 10-tick catch-up "
                   "clamp at %.0f Hz allows no slower frame)",
                   args.idle_fps, idle_fps, args.physics_hz);
        }
        printf("\n");
    } else {
        printf("idle: off (--idle-fps 0)\n");
    }

    // Per-frame phase timing. Pushed into Game::hud.perf_* EVERY frame; --perf
    // only gates the console output.
    const bool perf_on = args.perf;
    double p_events = 0.0, p_logic = 0.0, p_jobs = 0.0, p_refresh = 0.0,
           p_render = 0.0, p_present = 0.0, p_total = 0.0;                     // cumulative ms
    long long p_frames = 0, p_steps = 0;                                       // cumulative counts
    long long p_idled = 0;                                    // frames run hidden
    double w_events = 0.0, w_logic = 0.0, w_jobs = 0.0, w_refresh = 0.0,
           w_render = 0.0, w_present = 0.0;                                    // rolling-window ms
    long long w_frames = 0, w_steps = 0;                                       // rolling-window counts
    // This frame's marks: "refresh" = the 3D state pass (camera re-pose, terrain
    // LOD, g.hud.view), "render" = issuing GL draw commands, "present" = SwapBuffers
    // (blocks on vsync). Value-init: an indeterminate time_point makes
    // perf_ms() UB.
    std::chrono::steady_clock::time_point pf_iter{}, pf_a{}, pf_b{}, pf_c{},
                                        pf_refresh{}, pf_swap{}, pf_d{};
    const std::chrono::steady_clock::time_point perf_loop_start =
        std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point perf_w_start = perf_loop_start;

    const auto perf_ms = [](std::chrono::steady_clock::time_point a,
                            std::chrono::steady_clock::time_point b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };
    const auto perf_roll = [&]() {
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - perf_w_start).count();
        if(dt < 1.0 || w_frames == 0) { return; }
        printf("perf  logic=%7.3fms  refresh=%7.3fms  render=%7.3fms"
               "  events=%6.3fms  jobs=%6.3fms  present=%7.3fms"
               "   %6.1ffps   %d logic ticks (%.3fms/tick)\n",
               w_logic / (double)w_frames, w_refresh / (double)w_frames,
               w_render / (double)w_frames,
               w_events / (double)w_frames, w_jobs / (double)w_frames,
               w_present / (double)w_frames,
               (double)w_frames / dt, (int)w_steps,
               (w_steps > 0) ? (w_logic / (double)w_steps) : 0.0);
        w_events = w_logic = w_jobs = w_refresh = w_render = w_present = 0.0;
        w_frames = 0; w_steps = 0;
        perf_w_start = now;
    };
    const auto perf_summary = [&]() {
        if(p_frames == 0) { return; }
        const double wall = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - perf_loop_start).count();
        printf("\n=== perf summary (%lld frames over %.2f s, %.1f fps) ===\n",
               p_frames, wall, (wall > 0.0) ? (double)p_frames / wall : 0.0);
        const char *names[6] = {"events", "logic", "jobs", "refresh", "render",
                                "present"};
        const double vals[6] = {p_events, p_logic, p_jobs, p_refresh, p_render,
                                p_present};
        for(int i = 0; i < 6; i++) {
            printf("  %-7s avg %9.4f ms  (%5.1f%%)\n", names[i],
                   vals[i] / (double)p_frames,
                   (p_total > 0.0) ? (vals[i] * 100.0 / p_total) : 0.0);
        }
        printf("  %-7s avg %9.4f ms  (frame work only, EXCLUDES the "
               "--frame-cap sleep)\n", "total",
               p_total / (double)p_frames);
        if(p_idled > 0) {
            // No interval here: a --frame-cap slower than the idle budget paced
            // those frames.
            printf("  idled   %lld of those frames (window "
                   "occluded/minimized)\n", p_idled);
        }
        printf("  (refresh = the 3D state pass: camera re-pose, terrain LOD,\n"
               "   g.hud.view; render = issuing GL draw commands; present =\n"
               "   SwapBuffers, which blocks on vsync -- display pacing,\n"
               "   not render cost)\n");
        if(p_steps > 0) {
            printf("  logic %lld ticks, %.4f ms/tick (the logic phase)\n",
                   p_steps, p_logic / (double)p_steps);
        }
    };

    /* main loop timing from
       http://gafferongames.com/game-physics/fix-your-timestep/
    */
    bool idled_last = false;   // so the [idle] line prints on transitions only
    while (running == true) {
        const Uint32 iter_start_ms = SDL_GetTicks();
        pf_iter = std::chrono::steady_clock::now();

        /* Read visibility from the window flags every frame rather than latching
           SDL_EVENT_WINDOW_OCCLUDED / _MINIMIZED: a window that starts behind
           others is occluded from frame one, and a missed event cannot wedge the
           loop in either state. --force-occluded is the headless stand-in (a
           compositor never occludes an offscreen window). */
        const bool idling = idle_ms > 0
            && (display.isHidden()
                || (args.force_occluded_ms >= 0
                    && (int)(iter_start_ms - game.loop_start_ms)
                       >= args.force_occluded_ms));
        // Only the frame RATE changes. iter_cap_ms is a budget floor for the
        // iteration, not a period -- a blocking swap already fills it.
        const int iter_cap_ms = idling ? std::max(cap_ms, idle_ms) : cap_ms;
        if(idling != idled_last) {
            idled_last = idling;
            if(idling) {
                // The ceiling this budget can hold, not a promise about
                // --physics-hz.
                printf("[idle] window hidden: %d ms/frame budget "
                       "(sim ceiling %.0f Hz at 10 ticks/frame)\n",
                       iter_cap_ms, 10000.0 / (double)iter_cap_ms);
            } else {
                printf("[idle] window visible: drawing every frame\n");
            }
            fflush(stdout);
        }

        if(args.timeout_seconds > 0.0) {
            const double elapsed_s = (SDL_GetTicks() - game.loop_start_ms) * 0.001;
            if(elapsed_s >= args.timeout_seconds) {
                printf("Timeout reached (%.1f s); exiting main loop.\n", elapsed_s);
                fflush(stdout);
                /* --save / --resave: capture the live state before exiting. A bare
                   name is a slot of the CURRENT game under saves/; a path is used
                   as-is. --resave skips the exclusion so one process can load, fly
                   and write back for a round-trip diff (#51). */
                const std::string save_name = !args.save_name.empty()
                                            ? args.save_name : args.resave_name;
                if(!save_name.empty()) {
                    std::string save_dir = save_name;
                    if(save_dir.find('/') == std::string::npos) {
                        const std::string gamedir = game.ensureGameDir();
                        if(gamedir.empty()) {
                            printf("Save failed: could not create the game dir\n");
                            exit(1);
                        }
                        save_dir = gamedir + "/" + save_dir;
                    }
                    try {
                        save_game(game, save_dir);
                    } catch(const std::exception &e) {
                        printf("Save failed: %s\n", e.what());
                        exit(1);
                    }
                    fflush(stdout);
                }
                running = false;
            }
        }

        /*
          EVENTS
        */
        emit_sim_events(game);
        poll_events(game);
        pf_a = std::chrono::steady_clock::now();

        /*
          LOGIC
        */
        // The fixed-timestep loop lives in tick.cpp.
        /* Headless hooks first, then the LIVE scene's per-frame step. A
           launch collapses the stack to Flight, so the scene is read after
           them. The bodies + registration order (the e2e relies on it) are
           in installHeadlessHooks / vabInstallHooks. */
        game.fireHooks(kHookPreScene);
        {
            const SceneDef &sc = curScene(game);
            sc.update(game);
        }
        /* --select-target hooks keep the old poll site: a vessel removed
           earlier this frame has already had its xferTargets rows erased
           (Game::dropVehicleRefs), so the name lookup cannot hit a borrowed
           name pointer to a dead Vehicle. The list itself is rebuilt by the
           state pass below, so its orbits are one frame old -- fine for a
           name match (see installHeadlessHooks). */
        game.fireHooks(kHookPostUpdate);
        pf_b = std::chrono::steady_clock::now();

        // Run the finished jobs' continuations BEFORE the UI reads the state
        // they wrote.
        game.jobs.poll();
        // Engine hum: pilot scenes only. Gain is the throttle; the sound is ON
        // only while the ship actually produces thrust.
        {
            const SceneDef &sc = curScene(game);
            if(sc.pilot && ship != nullptr) {
                // Firing = an engine armed THIS tick. getThrust() is the
                // POTENTIAL at the current throttle, so it would hum at the
                // default 100% with the thrust key never touched.
                bool firing = false;
                int armedN = 0;
                for(const Part *p : ship->parts) {
                    if(p->armedThrust > 0.0f) { firing = true; armedN++; }
                }
                static const bool audDbg = (getenv("AUDIO_DEBUG") != nullptr);
                static bool lastFiring = false;
                if(audDbg && firing != lastFiring) {
                    printf("[hook] firing %d -> %d (armed=%d, thrust=%.0f N, throttle=%.2f)\n",
                           (int)lastFiring, (int)firing, armedN,
                           (double)ship->getThrust(), (double)ship->thruster_util);
                    fflush(stdout);
                    lastFiring = firing;
                }
                // Match the engine loop to the device's sample rate so the
                // real-time callback never resamples.
                static const char *engineFile = nullptr;
                if(engineFile == nullptr) {
                    engineFile = (game.audio.deviceRate() == 48000)
                        ? "res/audio/rocket_engine.001.wav"
                        : "res/audio/rocket_engine_44k.wav";
                }
                game.audio.setLoop(engineFile, firing, ship->thruster_util);
            } else {
                game.audio.setLoop("", false, 0.0f);
            }
        }
        game.audio.update();
        pf_c = std::chrono::steady_clock::now();

        /*
          RENDERING

          Two halves (#212): the state pass (sc.refresh) and the paint pass
          (sc.paint). The state pass is unconditional and issues no draw calls
          (though the LOD does delete GL buffers): it re-poses the camera, runs
          the terrain LOD that owns the ground's Bullet collision bodies, and
          fills the g.hud.view snapshot the UI reads. Every scene runs it (the VAB
          without the LOD).
          No scene opts out of the paint yet. Whoever adds a paint gate must force
          a paint when screenshot_requested is set: display.SaveScreenshot
          glReadPixels'es the back buffer, so a shot on a skipped paint is garbage.
        */
        {
            // The scene cannot change inside the render block, so read it once.
            const SceneDef &sc = curScene(game);
            sc.refresh(game);
            pf_refresh = std::chrono::steady_clock::now();

            /* --map-dump: AFTER the state pass -- sc.refresh is what fills
               Game::hud.view (see installHeadlessHooks). */
            game.fireHooks(kHookPostRefresh);

            check_gl_error();
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplSDL3_NewFrame();
            ImGui::NewFrame();
            check_gl_error();

            if(poly_mode == true) {
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                check_gl_error();
            }

            postfx->Begin();  // no-op unless --postfx effects are active
            // The VAB gets a studio backdrop (no skybox there); flight clears
            // to black.
            if(sc.backdrop == Backdrop::Studio) {
                display.Clear(0.55f, 0.62f, 0.68f, 1.0f);
            } else {
                display.Clear(0, 0, 0, 1);
            }

            // Begin/End stay together: Begin binds the offscreen target, End
            // blits it back, and the clear must land between them (#212).
            sc.paint(game);
            postfx->End();  // no-op unless --postfx effects are active

            /*
              ImGui stuff below
            */

            glUseProgram(0);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            if(poly_mode == true) {
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            }

            sc.drawUi(game);

            // The pending question over every window: deliberately outside the
            // scene's widget set, so a card cannot be hidden by scene membership
            // or TAB.
            drawQuestion(game);

            // Toasts: above everything, including the menu.
            drawToasts(game);

            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            if(screenshot_requested == true) {
                // A user feature, so shots live under the data dir. UTC stamp +
                // shot count: unique by construction.
                time_t now = ::time(nullptr);
                char stamp[40];
                strftime(stamp, sizeof(stamp), "%Y-%m-%dT%H-%M-%SZ", gmtime(&now));
                const std::string shot_dir = datadir::screenshots();
                {
                    std::error_code ec;   // non-throwing: a failure just fails the shot
                    std::filesystem::create_directories(shot_dir, ec);
                }
                char fname[256];
                snprintf(fname, sizeof(fname), "%s/osp_%s_%03d.png", shot_dir.c_str(),
                         stamp, screenshot_count + 1);   // 1-indexed (count = shots so far)
                /* F12 answers with a toast: a silent shot is indistinguishable
                   from a dead key. The toast never lands in a shot -- a
                   screenshot frame draws no toasts, so a burst of shots cannot
                   photograph the previous one's message. */
                if(display.SaveScreenshot(fname)) {
                    screenshot_count++;
                    game.toast("Screenshot saved");
                } else {
                    // SaveScreenshot already printed the path and SDL_GetError.
                    game.toast("Screenshot failed");
                }
                screenshot_requested = false;
            }

            // SwapBuffers is where the vsync block lives.
            pf_swap = std::chrono::steady_clock::now();
            display.SwapBuffers();
            check_gl_error();
        }

        pf_d = std::chrono::steady_clock::now();
        const double f_events  = perf_ms(pf_iter, pf_a);
        const double f_logic   = perf_ms(pf_a, pf_b);
        const double f_jobs    = perf_ms(pf_b, pf_c);
        const double f_refresh = perf_ms(pf_c, pf_refresh);  // 3D state pass
        const double f_render  = perf_ms(pf_refresh, pf_swap);  // issue GL cmds
        const double f_present = perf_ms(pf_swap, pf_d);  // SwapBuffers (vsync)
        const double perf_t =
            std::chrono::duration<double>(pf_d - perf_loop_start).count();
        game.hud.perf_events.push(perf_t, f_events);
        game.hud.perf_logic.push(perf_t, f_logic);
        game.hud.perf_jobs.push(perf_t, f_jobs);
        game.hud.perf_refresh.push(perf_t, f_refresh);
        game.hud.perf_render.push(perf_t, f_render);
        game.hud.perf_present.push(perf_t, f_present);
        if(perf_on) {
            p_events += f_events; p_logic += f_logic; p_jobs += f_jobs;
            p_refresh += f_refresh; p_render += f_render; p_present += f_present;
            p_total += perf_ms(pf_iter, pf_d);
            p_frames++; p_steps += game.logic_ticks;
            if(idling) { p_idled++; }
            w_events += f_events; w_logic += f_logic; w_jobs += f_jobs;
            w_refresh += f_refresh; w_render += f_render; w_present += f_present;
            w_frames++; w_steps += game.logic_ticks;
            perf_roll();
        }
        game.logic_ticks = 0;   // tick() re-arms it next frame

        /* --frame-cap: burn the rest of the frame budget. Without it the iteration
           spins at full speed whenever the swap isn't vsync-gated -- the offscreen
           EGL path reports swap interval 1 and then never waits.
           used_ms is sampled AFTER SwapBuffers, so a blocking swap already counts
           against the budget. A cap BELOW a faster panel's retrace grid cannot be
           met by sleeping -- that needs a swap interval > 1 (--vsync N). While the
           window is hidden this sleep IS the pacer. */
        if (iter_cap_ms > 0) {
            const Uint32 used_ms = SDL_GetTicks() - iter_start_ms;
            if (used_ms < (Uint32)iter_cap_ms) {
                SDL_Delay(iter_cap_ms - used_ms);
            }
        }
    }

    if(perf_on) { perf_summary(); }

    // The ships + pads are owned by the bodies (TerrainBody::ships / ::pads):
    // freed when the bodies are deleted below.

    // Stop the worker BEFORE the bodies it may still hold (a job captures a
    // TerrainBody* and may be sampling it off-thread).
    game.jobs.abort();

    for(auto&& body : sys.bodies) { delete body; }

    // Shaders + textures + plume mesh are registry-owned (get_*): they outlive
    // this scope and must NOT be deleted here.
    delete postfx;   // owns its own per-effect shaders (unique programs)

    delete front_indicator;
    delete prograde_indicator;
    delete retrograde_indicator;
    delete radial_in_indicator;
    delete radial_out_indicator;
    delete normal_plus_indicator;
    delete normal_minus_indicator;
    delete burn_indicator;
    delete relvel_indicator;
    delete relvel_retro_indicator;

    game.audio.shutdown();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();

    return 0;
}
