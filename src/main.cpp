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
#include "game.h"
#include "events.h"
#include "tick.h"
#include "render.h"
#include "gameui.h"
#include "vab.h"
#include "save.h"
#include "datadir.h"
#include "resdir.h"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include "cli.h"

#include "../middleware/imgui/imgui.h"
#include "../middleware/imgui/backends/imgui_impl_sdl3.h"
#include "../middleware/imgui/backends/imgui_impl_opengl3.h"
#include "../middleware/implot/implot.h"


/* ResourceType / ResourceContent / PartDef live in shipdef.h (shared with
   the JSON loaders and the headless tests). */

// Pump the SDL event queue during pre-loop load so a window-close is
// honoured promptly (poll_events() only drains it inside the main loop).
// Returns true if the user asked to quit.
static bool pumpLoadingQuit() {
    SDL_Event ev;
    while(SDL_PollEvent(&ev)) {
        ImGui_ImplSDL3_ProcessEvent(&ev);
        if(ev.type == SDL_EVENT_QUIT) { return true; }
    }
    return false;
}

// Draw a centered "loading..." label on a black background. Called before
// the slow init and from load_system's per-body progress hook so the window
// never sits blank.
static void drawLoadingFrame(Renderer &display, ImFont *font, const char *text) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();

    display.Clear(0, 0, 0, 1);

    // Fullscreen, undecorated window; the label is centered in it.
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

    // Leave GL state where the main loop's ImGui pass expects it (no bound
    // program / VAO).
    glUseProgram(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    display.SwapBuffers();
}

// The heavy phase (max_height + root terrain + shells) per body, split:
// `sync` builds synchronously (solid from the first frame), the rest defer
// to the worker. The star is always synchronous; at most two planets are.
// Declared in system.h (shared by the boot and the in-process system switch).
void postHeavyPhase(System &sys, const std::vector<TerrainBody *> &sync,
                    JobRunner &jobs, Shader *atmosphereshader,
                    Shader *cloudshader, Shader *oceanshader,
                    Shader *ringshader, int cloudres) {
    std::vector<TerrainBody *> isSync;
    isSync.push_back(sys.root);   // the star: the light source, always solid
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
    {   // e2e anchor: the sync set is "the player's bodies are solid from
        // frame one".
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
            // The worker body only uses `b` (BuildRootGeoms is pure); the
            // rest are carried so the main-thread continuation can capture
            // them.
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

// Split one --startship value "name,def,body,scenario" into its four fields
// (all required, non-empty); returns false if the shape is wrong.
static bool splitStartship(const std::string &spec, DebugStartShip &out) {
    std::vector<std::string> f;
    std::string cur;
    for(char c : spec) {
        if(c == ',') { f.push_back(cur); cur.clear(); }
        else { cur += c; }
    }
    f.push_back(cur);
    if(f.size() != 4) { return false; }
    for(const std::string &x : f) { if(x.empty()) { return false; } }
    out.name = f[0];
    out.ship = f[1];
    out.body = f[2];
    out.scenario = f[3];
    return true;
}

int main(int argc, char **argv)
{
    const auto prog_start = std::chrono::steady_clock::now();

    GameArgs args;
    int exit_code = 1;
    if(!parse_cli(argc, argv, args, &exit_code)) { return exit_code; }

    // Data directory (datadir.h): saves/ + settings.json live in the per-OS
    // user data directory (--data-dir overrides). Must run before the
    // settings load below.
    datadir::init(args.data_dir);

    // settings.json phase 1: the file's args fields must reach the window
    // creation -- the display mode/size and the MSAA count are fixed in the
    // GLX visual, and setWindowMode can't change the MSAA after. Explicit
    // CLI flags win field by field. Phase 2 (game.load_settings) applies the
    // Game + PostFX fields once the Game exists.
    load_settings_args(args);

    Renderer display(args.screen_width, args.screen_height, args.window_mode,
                     args.msaa_samples, args.gl_debug);
    check_gl_error();
    const Uint32 sim_win_id = SDL_GetWindowID(display.get_display());
    /* --sim-press: resolve keycodes to scancodes now that SDL is initialized
       (the CLI parse ran before the Renderer created the video subsystem). */
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
    // No imgui.ini: window layout must not survive between runs or clobber
    // the layout the code sets up each frame.
    io.IniFilename = nullptr;
    // Normal and big faces are the same font (the big one at 2x size).
    // --font / --font-size pick which + how big. GlyphExtraAdvanceX is the
    // letter-tracking knob (map-label clearance is a separate knob in
    // gameui.cpp).
    const float glyph_extra_advance_x = 0.0f;
    ImFontConfig font_cfg;
    font_cfg.GlyphExtraAdvanceX = glyph_extra_advance_x;
    io.Fonts->AddFontFromFileTTF(resdir::path(args.font_path).c_str(),
                                 args.font_size, &font_cfg);
    // The big face (2x size) for the HUD + main menu (gameui.cpp).
    ImFont *bigger = io.Fonts->AddFontFromFileTTF(
        resdir::path(args.font_path).c_str(), 2.0f * args.font_size,
        &font_cfg);
    check_gl_error();

    // First thing the user sees: a "loading..." label before the slow init.
    // The presented frame persists through the init steps that don't present.
    if(pumpLoadingQuit()) { return 1; }
    drawLoadingFrame(display, bigger, "loading...");

    // start bullet; see physics.cpp
    void create_physics(void);
    create_physics();
    check_gl_error();

    /* data init (the get_shader registry owns these: compiled once,
       shared, never deleted) */
    Shader *partsshader = get_shader("res/shaders/partsShader",
                                     { "position", "uv", "normal" },
                                     { "MVP", "Normal", "lightDirection", "shadow",
                                       "alpha", "tint", "flatLight" });

    Shader *terrainshader = get_shader("res/shaders/terrainShader",
                                       { "position", "normal", "color" },
                                       { "MVP", "Normal", "lightDirection", "color",
                                         "anchor" });

    Shader *sunshader = get_shader("res/shaders/sunShader",
                                   { "position", "normal", "color" },
                                   { "MVP", "Normal", "lightDirection", "color" });

    // Atmosphere shell: Fresnel limb glow from orbit, interior sky dome
    // from the surface (the `inside` flag). See reports/atmosphere2026_08_25.
    Shader *atmosphereshader = get_shader("res/shaders/atmosphereShader",
                                          { "position", "normal" },
                                          { "MVP", "Normal", "cameraPos",
                                            "color", "intensity", "power",
                                            "lightDirection", "inside",
                                            "planetCenter" });

    // Cloud deck: a shell between the terrain and the atmosphere rim.
    // Coverage is baked (BuildClouds) on the job worker. "uvParam" binds
    // the mesh's color slot (attrib location 2): the unwrapped sphere
    // params the deck UV is built from.
    Shader *cloudshader = get_shader("res/shaders/cloudShader",
                                     { "position", "normal", "uvParam" },
                                     { "MVP", "Normal", "cameraPos", "color",
                                       "lightDirection", "drift", "planetCenter",
                                       "coverage_tex" });

    // Ocean surface: a transparent shell at sea level with animated wave
    // normals, Fresnel reflection and a specular sun glint. Land pokes
    // through via the depth test.
    Shader *oceanshader = get_shader("res/shaders/oceanShader",
                                     { "position", "normal" },
                                     { "MVP", "Normal", "cameraPos", "seaColor",
                                       "lightDirection", "time", "planetCenter" });

    // Planetary rings: flat annuli in the body's equatorial plane, drawn
    // over the opaque terrain. Two-sided Lambert (the sun can be above or
    // below the plane).
    Shader *ringshader = get_shader("res/shaders/ringShader",
                                    { "position", "normal" },
                                    { "MVP", "Normal", "lightDirection",
                                      "albedo", "opacity", "planetRadius" });

    Shader *skyboxshader = get_shader("res/shaders/skyboxShader",
                                      { "position" },
                                      { "projectionview", "gain" });

    Shader *lineshader = get_shader("res/shaders/lineShader2",
                                    { "position" },
                                    { "MVP", "color" });

    PostFX *postfx = new PostFX;
    // Create every built-in effect up front (no mid-frame shader
    // compilation) so the Settings window can toggle them at runtime.
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

    // The progress hook redraws the "loading..." label after each body's
    // LIGHT phase. Throttled to ~10 fps: each draw ends in a SwapBuffers
    // that blocks on vsync, so drawing every body would add pure UI pacing
    // to the load. The epoch initial value lets the first body (and the
    // final one, via the i+1<total guard) draw immediately.
    auto last_loading_draw = std::chrono::steady_clock::time_point{};
    System sys;
    try {
        sys = load_system(args.system_file.c_str(), terrainshader, sunshader,
            [&](size_t i, size_t total, const std::string &name) {
                if(pumpLoadingQuit()) { std::exit(1); }   // window closed mid-load
                auto now = std::chrono::steady_clock::now();
                if(now - last_loading_draw < std::chrono::milliseconds(100)
                   && i + 1 < total) {
                    return;   // not time for another frame yet (still responsive to quit)
                }
                last_loading_draw = now;
                char buf[160];
                snprintf(buf, sizeof(buf), "loading %s  %zu / %zu",
                         name.c_str(), i + 1, total);
                drawLoadingFrame(display, bigger, buf);
            });
    } catch(const std::exception &e) {
        // The loader throws on data bugs (a bad field, a missing skybox
        // face): name the problem and leave, rather than letting the throw
        // reach std::terminate.
        printf("error: %s\n", e.what());
        fflush(stdout);
        return 1;
    }
    TerrainBody *sun = sys.root;
    TerrainBody *home = sys.home;   // the system home: the VAB launch body,
                                    // the title backdrop, and the body the
                                    // --radial-test / --dock-test ships use.

    // The star field comes from the system JSON, so the Skybox is built here
    // -- before the boot --load path, whose ensureSystemForSave may switch
    // systems -- and Game::switchSystem reloads its cubemap from there.
    Skybox skybox;
    skybox.init();
    skybox.load(sys.skybox_faces);

    /* The experiment family table (res/data/experiments.json): the
       instruments' balance data (base value, runnable situations,
       biome-specificity). Loaded before the ships so the cross-check below
       has the table. */
    loadExperimentDefs(resdir::path(args.experiments_file).c_str());

    /* The ships are built from JSON: the parts catalog (res/data/parts.json)
       supplies each part's mass + behavior, the ship defs supply the stack
       order + offsets, and the start-ship list supplies one entry per ship.
       Ships sharing a (body, scenario) pair are slotted. */
    Ships ships(args.parts_file, partsshader, sun);

    /* An instrument naming a family with no def would silently fall back to
       the base-10 generic behavior -- a parts.json typo should not hide.
       The kerbal suit's own family is hard-coded in game.cpp (runExperiment,
       "observation"), not in parts.json, so check it here too. */
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

    // The running game: borrows the subsystems above and owns the runtime
    // state (camera, clock, active ship, input/UI flags, the orbit-camera
    // focus targets, the UI window registry) plus the control transitions.
    Game game(display, postfx, ships, sys, sun, home, args, sim_win_id);
    game.bigger = bigger;   // the UI pass (gameui.cpp) draws with it
    game.skybox = &skybox;  // set before any boot --load can switch systems
    // The running system (what save_game records + the load path compares
    // against). Set before the boot --load so ensureSystemForSave sees it.
    game.systemPath = args.system_file;

    // Sound (audio.h): a silent no-op when there is no playback device or
    // the assets are missing. Music starts here and loops the whole session.
    if(game.audio.init()) {
        game.audio.setMusic("res/audio/ville_seppanen-1_g.ogg");
    }

    // settings.json phase 2: the Game + PostFX state, before apply_ui_style
    // reads the ui knobs.
    game.load_settings();
    game.apply_ui_style();  // the Settings defaults (dark theme, scale 1.0)

    // --start-time: start the analytic clock (and every body's orbit and
    // spin) at a later instant. Must happen before the fleet spawns -- the
    // orbit scenarios read the home body's frame state. setTime propagates
    // the frames for the paused start. (--load overwrites it with the
    // save's clock.)
    game.setTime(args.start_time);
    if(args.start_time > 0.0) {
        printf("Starting at sim time t = %.0f s\n", args.start_time);
    }

    // The runtime state lives in `game`. These local references alias
    // game's members so the loop body reads as before. (screenshot_count
    // is pure loop bookkeeping and stays local.)
    Vehicle *&ship = game.ship;
    int &time_accel = game.time_accel;
    int &cam_speed = game.cam_speed;
    bool &poly_mode = game.poly_mode;
    bool &screenshot_requested = game.screenshot_requested;
    bool &running = game.running;

    std::vector<DebugStartShip> start_ships;
    if(!args.startships_file.empty()) {
        // file form: a JSON list (loadDebugStartShips validates all four
        // fields per entry)
        try {
            start_ships =
                loadDebugStartShips(resdir::path(args.startships_file).c_str())
                    .ships;
        } catch(const std::exception &e) {
            printf("error: %s\n", e.what());
            return 1;
        }
    } else if(!args.startship.empty()) {
        // inline form: each --startship is "name,def,body,scenario"
        for(size_t i = 0; i < args.startship.size(); i++) {
            DebugStartShip e;
            if(!splitStartship(args.startship[i], e)) {
                printf("error: --startship #%zu '%s': expected "
                       "name,def,body,scenario (four non-empty fields)\n",
                       i + 1, args.startship[i].c_str());
                return 1;
            }
            start_ships.push_back(e);
        }
    }
    // Neither -> no start ships, which boots to the title screen.

    /* The heavy phase (max_height + root terrain + shells) is the part that
       made a big system take ~7s. Split it: build the bodies the player is
       ON synchronously so the first frame is solid, and defer the rest to
       the worker. The sync set is the player's bodies, NOT the system home.
       Which bodies:
         --load         the save's ship bodies
         --radial/--dock the home body
         --vab          the launch body
         fleet          the fleet's bodies
         otherwise      the title backdrop + home
       Shared with the in-process system switch (Game::switchSystem). */
    Vehicle *first = nullptr;
    if(!args.load_name.empty()) {
        // --load: the saved fleet replaces the one that would be built.
        // load_game builds every ship + kerbal and puts each in the world
        // state it was saved in (live or railed).
        //
        // A bare slot name is a UI save for the data dir's saves/; if the
        // given path has no save.json, resolve the slot there instead.
        // Precedence: the given path as-is first, then the data-dir slot
        // (legacy flat before the unique saves/<game>/<slot>).
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
        // Earth, not Kerbin. Same check the UI/CLI reload uses (loadFrom). A
        // successful switch runs the heavy phase with the save's ship bodies
        // as the sync set; when no switch happens the boot heavy phase below
        // does.
        bool switched = false;
        if(!game.ensureSystemForSave(load_dir, &switched)) {
            exit(1);   // reason already printed + toasted by ensureSystemForSave
        }
        if(!switched) {
            // The heavy phase has not run yet (boot, not a runtime reload):
            // sync the save's ship bodies where they exist in THIS system,
            // plus home (the load's find-else-fallback landing).
            std::vector<TerrainBody *> sync;
            for(const std::string &n : saveShipBodies(load_dir)) {
                if(TerrainBody *b = sys.find(n)) { sync.push_back(b); }
            }
            sync.push_back(home);
            postHeavyPhase(sys, sync, game.jobs, atmosphereshader, cloudshader,
                           oceanshader, ringshader, args.cloud_mesh);
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
        // A cross-system save switched the running system just now
        // (ensureSystemForSave -> switchSystem DELETED the old bodies), so
        // the boot-time `sun`/`home` locals dangle. Re-point them at the
        // live system.
        sun = game.sun;
        home = game.home;
        // switchSystem landed on the Title; a loaded fleet resumes in flight,
        // and a shipless save lands on the Space Center hub (a meaningful
        // game, just no active vessel) rather than the title front door.
        if(ship != nullptr) { enterFlight(game); }
        else { enterSpaceCenter(game); }
    } else {
        std::vector<TerrainBody *> sync;
        if(!args.radial_test.empty() || !args.dock_test.empty()) {
            sync.push_back(home);   // the test ships sit on the home body
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
            sync.push_back(home);   // the hub frames it and the first launch
                                    // leaves from it
        }
        postHeavyPhase(sys, sync, game.jobs, atmosphereshader, cloudshader,
                       oceanshader, ringshader, args.cloud_mesh);

        if(!args.radial_test.empty()) {
            RadialTestShip rts = build_radial_test_ship(
                args.radial_test, ships.catalog(), home, sun, partsshader);
            ships.add_ship(rts.v, home, rts.sc, rts.slot, game.time);
            first = rts.v;
        } else if(!args.dock_test.empty()) {
            DockTestShips dts = build_dock_test_ships(
                args.dock_test, ships.catalog(), home, sun, partsshader, sys);
            /* Both are placed already (the builder ran spawn_vehicle), so
               null scenario: apply_scenarios skips them. The probe is the
               ACTIVE ship; the station parks on rails until proximity wakes
               it. */
            ships.add_ship(dts.probe, home, nullptr, 0, game.time);
            ships.add_ship(dts.station, home, nullptr, 1, game.time);
            first = dts.probe;
        } else {
            // A typo in any field (unknown body / scenario, an unreadable
            // def) throws deep in here; catch it the way the --load path
            // does so it is a clean error, not a core dump.
            try {
                first = ships.buildDebugStartShips(start_ships, sys, game.time);
            } catch(const std::exception &e) {
                printf("error: %s\n", e.what());
                return 1;
            }
        }
    }

    if(args.load_name.empty()) {
        check_gl_error();
        // Scenarios first (they are what position the ships), then park the
        // idle ones -- both before the camera is constructed, so it focuses
        // on the spawn point. Shared with the title screen's New Game.
        game.settleFleet(first);
        /* The active (player-controlled) ship: the first one built; F6 / the
           SHIPS window switch it. Assigned directly rather than through
           select_ship: the camera does not exist yet here. */
        ship = first;
        if(first != nullptr) {
            printf("[dbg-dv] getMass=%.2f kg  getDeltaV=%.1f m/s\n",
                   (double)first->getMass(), (double)first->getDeltaV());
        }
        /* --autopilot: engage a slew mode on the active ship (a test hook;
           the Autopilot window is the only in-game way to engage these).
           slewRequest is applied every tick and held until toggled. */
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

    // Billboard icons opt out of mip chains: their alpha cutouts bleed
    // into the neighbouring level when minified.
    Texture * front_indicator_texture = get_texture("res/textures/front_crosshair.png", false);
    Texture * prograde_indicator_texture = get_texture("res/textures/prograde_icon.png", false);
    Texture * retrograde_indicator_texture = get_texture("res/textures/retrograde_icon.png", false);
    Texture * radial_in_indicator_texture = get_texture("res/textures/radial_in_icon.png", false);
    Texture * radial_out_indicator_texture = get_texture("res/textures/radial_out_icon.png", false);
    Texture * normal_plus_indicator_texture = get_texture("res/textures/normal_plus_icon.png", false);
    Texture * normal_minus_indicator_texture = get_texture("res/textures/normal_minus_icon.png", false);

    // Marker hues: one per direction family, so a glance at the ring around
    // the ship says which marker is which. billboardshader.fs tints with this
    // uniform and ignores the texture RGB, so the colour lives here, not in
    // the icon PNGs. Prograde takes the orbit map's green (gameui.cpp
    // col_ship) so the world and the map read the same; the others stay clear
    // of it. Each pair (radial in/out, normal +/-) shares a hue and is told
    // apart by its glyph; prograde/retrograde get separate hues because they
    // are the two you slew to by name.
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
    // Transfer burn direction (TRANSFER window): the prograde icon in amber,
    // pointing where the departure burn should point. Not the KSP blue it
    // used -- that hue belongs to normal now, and a blue marker in the ring
    // is ambiguous exactly when a transfer is plotted.
    Billboard *burn_indicator =
        mk_billboard(billboardshader, prograde_indicator_texture, 1.0, 1.0,
                     glm::vec4(0.95f, 0.70f, 0.15f, 1.0f));
    // Target ship's relative velocity, two cyan markers: the prograde
    // (diamond) icon for you − target, the retrograde (X) icon for
    // target − you. Cyan, not the old pink: magenta sat next door to radial
    // purple while reusing the prograde/retrograde glyphs.
    const glm::vec4 relvelcolor = glm::vec4(0.15f, 0.85f, 1.0f, 1.0f);
    Billboard *relvel_indicator =
        mk_billboard(billboardshader, prograde_indicator_texture, 1.0, 1.0, relvelcolor);
    Billboard *relvel_retro_indicator =
        mk_billboard(billboardshader, retrograde_indicator_texture, 1.0, 1.0, relvelcolor);

    /* camera init */
    const float camFov = (float)glm::radians(args.camFovDeg);
    // The drawable size the Renderer actually got (the WM may have clamped
    // it, or fullscreen may have used the display mode).
    const float camAspect = (float)display.get_width() / (float)display.get_height();
    const float camZNear = 1.0f;
    // zFar must exceed the farthest visible body. The log-depth shaders
    // define the hard far limit as `far = 1e13` m -- keep zFar consistent
    // with that.
    const float camZFar = 1e13;

    // One camera, two modes (orbit + free). The terrain LOD reads the
    // live one.
    const glm::dvec3 camFocus = ship ? ship->partPos(ship->controller)
                                     : home->frame->root_pos;
    Camera *cam = new Camera(camFocus, camFov, camAspect, camZNear, camZFar);
    cam->setViewport(display.get_width(), display.get_height());
    game.camera = cam;
    // Bodies the orbit camera can target. Seeded BEFORE the --vab /
    // --free-cam framing below, so that framing stays the LAST word on the
    // camera. With a ship the ship is index 0. A --load boot already ran
    // load_game (which syncs the "ship" entry), so insert only when absent.
    if(game.ship != nullptr &&
       (game.focusTargets.empty() || game.focusTargets[0].body != nullptr)) {
        game.focusTargets.push_back({ "ship", nullptr });
    }
    for (TerrainBody *b : sys.bodies) {
        game.focusTargets.push_back({ b->name.c_str(), b });
    }
    if(ship == nullptr && args.load_name.empty()) {
        /* No vessel and no loaded game: the floor scene is the TITLE screen,
           not an empty flight one. Decided before the --vab entry below so an
           editor opened with nothing to fly sits on [title, vab]. The backdrop
           camera parks before the --vab / --free-cam framing below. A --load
           already chose its floor (flight or the Space Center hub) above, so a
           shipless save keeps the hub instead of being pulled to the title. */
        enterTitle(game);
        game.parkTitleCamera();
        printf("[boot] no vessel: title screen\n");
        fflush(stdout);
    }

    /* --vab: open the editor scene with a ship def loaded as a physics-free
       build tree. The flight ships still exist in the world but drawVab
       draws only the build tree. vabOpen parks the (boot) camera and aims
       the orbit at the build. */
    if(!args.vab.empty()) {
        ShipDef vdef = load_ship_def(resdir::path(args.vab).c_str(),
                                     ships.catalog());
        game.vab.build = BuildShip::fromShipDef(vdef);
        game.vab.armed = args.vab_arm;   // test hook: pre-arm a palette part
        vabOpen(game);
    } else if(args.vab_empty) {
        // --vab-empty: the main menu's "Go to VAB" (an empty build) -- the
        // headless entry to the same editor.
        game.vab.armed = args.vab_arm;   // test hook: pre-arm a palette part
        vabOpen(game);
    }
    if(sceneIs(game, SceneId::Vab)) {
        // --vab-scenario / --vab-body: override the launch config the top-bar
        // dropdowns show (vabOpen already seeded the defaults).
        if(!args.vab_scenario.empty()) { game.vab.scenarioName = args.vab_scenario; }
        if(!args.vab_body.empty())     { game.vab.bodyName = args.vab_body; }
    }

    // --surfmap-body: pin the Surface Map's body so M / Refresh map it
    // without a combo click. Applied AFTER the --load branch so it resolves
    // against the system the boot actually runs. A hard error on a miss: a
    // silent fallback would leave the map on the default body and defeat the
    // test that relies on the pin (issue #72).
    if(!args.surfmap_body.empty()) {
        if(TerrainBody *b = sys.find(args.surfmap_body)) {
            game.surfmap_body = b;
        } else {
            printf("error: --surfmap-body '%s' is not a body in %s\n",
                   args.surfmap_body.c_str(), game.systemPath.c_str());
            return 1;
        }
    }

    if(args.use_free_cam) {
        // Default free pose = the orbit camera's current view, overridable
        // per-axis via --free-cam-pos / --free-cam-fwd / --free-cam-up.
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

    // kRailsWarp is defined in game.h (the rails-warp threshold).
    // New game / load game start paused: load_game sets 0, and a --load
    // leaves it there. An explicit --time-accel still applies on both paths.
    // A non-load boot without the flag keeps the CLI default (1x).
    if(args.load_name.empty() || args.cli_given.time_accel) {
        time_accel = args.initial_time_accel;
    }

    /* The CLI is not the ladder: --time-accel can ask for more than the
       ceiling the warp keys stop at. Clamp it, so a typo cannot put 1e12 s of
       sim on the clock in one tick. */
    if(time_accel > kMaxWarp) {
        printf("Time accel %dx is above the warp ceiling; clamping to %dx\n",
               time_accel, kMaxWarp);
        time_accel = kMaxWarp;
    }

    /* Starting the game directly in rails warp (accel > 10): the active
       ship parks too (works on the pad -- that is the frozen mode), unless
       some ship is not rail-eligible, in which case clamp to the top
       physics warp (10). */
    if(time_accel >= kRailsWarp) {
        bool all_eligible = true;
        for(auto *s : collectVehicles(sys)) {
            if(!s->canRail()) {
                printf("Rails warp refused at start: '%s' is neither in free "
                       "fall nor grounded; clamping time accel to 10\n",
                       s->name.c_str());
                all_eligible = false;
                break;
            }
        }
        if(all_eligible) {
            for(auto *s : collectVehicles(sys)) { s->goOnRails(); }
        } else {
            time_accel = 10;
        }
    }
    cam_speed = 1;

    // The per-window UI options + the window registry (game.cpp) live on
    // the game; the transfer planner (the TRANSFER window) too -- it holds
    // sim-clock state, so Game owns it and the clock hook can invalidate it.

    // Two reference circles in the render frame's local axes. Each is its
    // own mesh so it can be drawn a distinct colour: the XZ plane (y=0, the
    // "flat" orbital/equatorial reference) and the XY plane (z=0, the
    // "vertical" meridian reference).
    Mesh *skyline_xz = new Mesh;
    Mesh *skyline_xy = new Mesh;
    {
        float r = 1000;
        int n = 128;
        PosInterface xzinterface;
        PosInterface xyinterface;
        for(int i = 1; i < 128; i++) {
            const double a = (2 * std::numbers::pi) * float(i-1)/float(n);
            xzinterface.positions.push_back(glm::vec3(r * cos(a), 0, r * sin(a)));  // y=0 -> XZ plane
            xyinterface.positions.push_back(glm::vec3(r * cos(a), r * sin(a), 0));  // z=0 -> XY plane
        }
        skyline_xz->InitMesh(xzinterface);
        skyline_xy->InitMesh(xyinterface);
    }

    // Hand the render resources to the game (render.cpp draws with them).
    game.skyboxshader = skyboxshader;
    game.lineshader = lineshader;
    game.partsshader = partsshader;
    game.engine_plume_mesh = engine_plume_mesh;
    game.engine_plume_texture = engine_plume_texture;
    game.skyline_xz = skyline_xz;
    game.skyline_xy = skyline_xy;
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

    /* Runtime spawn: Ships::spawn_ship (ships.cpp) -- place + apply the
       scenario + park on rails; appended at the end so it is never the
       active one. */

    /* --selftest-spawn: exercise the runtime spawn/remove path. Spawn a
       copy of the active ship, remove it, then spawn-select-remove the
       active one. Each step is checked against the expected fleet size +
       active index. */
    int spawn_test_ticks = 0;
    if(args.selftest_spawn) {
        /* A free kerbal has a def but remove_ship refuses it (a crew member
           is not deletable), so the spawn-copy-then-remove steps cannot run. */
        if(ship->defPath.empty() || ship->isEva()) {
            printf("selftest-spawn: SKIP (%s)\n",
                   ship->isEva() ? "active ship is a crew member"
                                 : "active ship has no def: test ship");
            running = false;
        } else {
            const size_t base = collectVehicles(sys).size();
            Vehicle *origShip = ship;
            bool ok = true;
            printf("== selftest-spawn: %zu ships at start, active: %s ==\n",
                   base, ship->name.c_str());

            // 1) spawn a copy of the active ship -> appended at the end;
            //    the active ship must be untouched
            Vehicle *sp = ships.spawn_ship(ship->defPath, "", ship->home,
                                           ship->scenario, sys, game.time);
            printf("spawn 1: size=%zu active=%s\n",
                   collectVehicles(sys).size(), ship->name.c_str());
            if(collectVehicles(sys).size() != base + 1 || ship != origShip) { ok = false; }

            // 2) remove the ship we just spawned -> size back to base,
            //    active unchanged
            game.remove_ship(sp);
            printf("remove 1: size=%zu active=%s\n",
                   collectVehicles(sys).size(), ship->name.c_str());
            if(collectVehicles(sys).size() != base || ship != origShip) { ok = false; }

            // 3) spawn again, select it, remove it (the active one) -> the
            //    control must hand off and the size return to base
            Vehicle *sp2 = ships.spawn_ship(ship->defPath, "", ship->home,
                                            ship->scenario, sys, game.time);
            game.select_ship(sp2);
            printf("spawn 2 + select: active=%s size=%zu\n",
                   ship->name.c_str(), collectVehicles(sys).size());
            if(ship != sp2) { ok = false; }
            game.remove_ship(sp2);
            /* The handoff must be checked BEFORE the printf below reads
               ship->name: sp2 is deleted, so a failed handoff would leave
               `ship` pointing at freed memory. */
            if(ship == sp2 || ship == nullptr) { ok = false; }
            printf("remove 2 (active): active=%s size=%zu\n",
                   ship ? ship->name.c_str() : "(none)",
                   collectVehicles(sys).size());
            if(collectVehicles(sys).size() != base) { ok = false; }

            if(ok) {
                printf("selftest-spawn: all checks passed; running 30 ticks for stability\n");
                spawn_test_ticks = 30;
            } else {
                printf("selftest-spawn: FAIL (bookkeeping mismatch)\n");
                running = false;
            }
        }
    }

    // --timeout: wall-clock budget for the whole run (0 = run until closed).
    // Stamped on the game: the sim-event emitter (events.cpp) and the
    // timeout check below both measure "ms since the loop started" from it.
    game.loop_start_ms = SDL_GetTicks();

    // Star-field exposure prototype: the CLI seeds it, the Settings window
    // edits it live, render.cpp's skyGain reads it per frame.
    game.sky_dim = args.sky_dim;
    game.sky_dim_cone = args.sky_dim_cone;

    // The headless VAB hooks are stamped on the game too, so the code that
    // fires them lives with the editor (vabFireHooks / vabUpdate) rather
    // than in this loop.
    game.newGameMs = args.new_game_ms;
    game.reloadDir = args.reload_dir;
    game.reloadMs = args.reload_ms;
    game.quitTitleMs = args.quit_title_ms;
    game.spaceCenterMs = args.space_center_ms;
    game.recoverMs = args.recover_ms;
    game.experimentMs = args.experiment_ms;
    game.podExperimentMs = args.pod_experiment_ms;
    game.evaMs = args.eva_ms;
    game.takeMs = args.take_ms;
    game.storeMs = args.store_ms;
    game.trackingMs = args.tracking_ms;
    game.trackingCloseMs = args.tracking_close_ms;
    game.researchMs = args.research_ms;
    game.researchCloseMs = args.research_close_ms;
    game.atlasDumpMs = args.atlas_dump_ms;
    game.mapDumpMs = args.map_dump_ms;
    if(args.map_plane >= 0) { game.map_plane = args.map_plane; }
    game.switchSystemPath = args.switch_system_path;
    game.switchSystemMs = args.switch_system_ms;
    game.vabHooks.placeMs = args.vab_place_ms;
    game.vabHooks.loadMs = args.vab_load_ms;
    game.vabHooks.loadPath = args.vab_load;
    game.vabHooks.launchMs = args.vab_launch_ms;
    game.vabHooks.closeMs = args.vab_close_ms;
    game.vabHooks.detachIdx = args.vab_detach_idx;
    game.vabHooks.detachMs = args.vab_detach_ms;
    game.vabHooks.savePath = args.vab_save;
    game.vabHooks.saveMs = args.vab_save_ms;
    const double startup_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - prog_start).count();
    printf("Main loop starting: startup took %.3f s", startup_s);
    if(args.timeout_seconds > 0.0) {
        printf(" | auto-exit after %.1f s (wall clock)", args.timeout_seconds);
    }
    printf("\n");
    fflush(stdout);

    // --frame-cap: budget per loop iteration (0 = uncapped). Physics stays
    // at its fixed 50 Hz off the wall clock regardless of this.
    const int cap_ms = (args.frame_cap > 0) ? (int)(1000.0 / (double)args.frame_cap) : 0;
    if (cap_ms > 0) {
        printf("frame cap: %d fps\n", args.frame_cap);
    } else {
        printf("frame cap: off (uncapped)\n");
    }

    // Per-frame phase timing. The push into the Game::perf_* series runs
    // EVERY frame (the Telemetry window reads them); --perf only controls
    // the console output. The "logic" phase is where the Part*/Body*
    // indirection lives (tick -> physics_tick -> ships -> parts -> bodies);
    // the per-substep number is the one to compare across refactors.
    const bool perf_on = args.perf;
    double p_events = 0.0, p_logic = 0.0, p_jobs = 0.0, p_render = 0.0,
           p_present = 0.0, p_total = 0.0;                                     // cumulative ms
    long long p_frames = 0, p_steps = 0;                                       // cumulative counts
    double w_events = 0.0, w_logic = 0.0, w_jobs = 0.0, w_render = 0.0,
           w_present = 0.0;                                                    // rolling-window ms
    long long w_frames = 0, w_steps = 0;                                       // rolling-window counts
    // This frame's marks. pf_swap sits between the last draw call and the
    // SwapBuffers, so "render" = issuing the GL commands and "present" =
    // the SwapBuffers (which blocks on vsync -- display pacing, not render
    // cost).
    std::chrono::steady_clock::time_point pf_iter, pf_a, pf_b, pf_c, pf_swap, pf_d;
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
        printf("perf  logic=%7.3fms  render=%7.3fms  events=%6.3fms  jobs=%6.3fms"
               "  present=%7.3fms   %6.1ffps   %d phys steps (%.3fms/step)\n",
               w_logic / (double)w_frames, w_render / (double)w_frames,
               w_events / (double)w_frames, w_jobs / (double)w_frames,
               w_present / (double)w_frames,
               (double)w_frames / dt, (int)w_steps,
               (w_steps > 0) ? (w_logic / (double)w_steps) : 0.0);
        w_events = w_logic = w_jobs = w_render = w_present = 0.0;
        w_frames = 0; w_steps = 0;
        perf_w_start = now;
    };
    const auto perf_summary = [&]() {
        if(p_frames == 0) { return; }
        const double wall = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - perf_loop_start).count();
        printf("\n=== perf summary (%lld frames over %.2f s, %.1f fps) ===\n",
               p_frames, wall, (wall > 0.0) ? (double)p_frames / wall : 0.0);
        const char *names[5] = {"events", "logic", "jobs", "render", "present"};
        const double vals[5] = {p_events, p_logic, p_jobs, p_render, p_present};
        for(int i = 0; i < 5; i++) {
            printf("  %-7s avg %9.4f ms  (%5.1f%%)\n", names[i],
                   vals[i] / (double)p_frames,
                   (p_total > 0.0) ? (vals[i] * 100.0 / p_total) : 0.0);
        }
        printf("  %-7s avg %9.4f ms  (frame total, incl. frame-cap sleep)\n",
               "total", p_total / (double)p_frames);
        printf("  (render = issuing GL draw commands; present = SwapBuffers,\n"
               "   which blocks on vsync -- display pacing, not render cost)\n");
        if(p_steps > 0) {
            printf("  physics %lld substeps, %.4f ms/substep (the logic phase)\n",
                   p_steps, p_logic / (double)p_steps);
        }
    };

    /* main loop timing from
       http://gafferongames.com/game-physics/fix-your-timestep/
    */
    while (running == true) {
        const Uint32 iter_start_ms = SDL_GetTicks();
        pf_iter = std::chrono::steady_clock::now();

        // --timeout: auto-exit once the wall-clock budget is spent.
        if(args.timeout_seconds > 0.0) {
            const double elapsed_s = (SDL_GetTicks() - game.loop_start_ms) * 0.001;
            if(elapsed_s >= args.timeout_seconds) {
                printf("Timeout reached (%.1f s); exiting main loop.\n", elapsed_s);
                fflush(stdout);
                // --save: capture the live game state into the save directory
                // before the loop exits. A bare name is a slot of the CURRENT
                // game under the data dir's saves/; a path is used as-is.
                if(!args.save_name.empty()) {
                    std::string save_dir = args.save_name;
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

        // --selftest-spawn: a few post spawn/remove physics ticks, then exit.
        if(spawn_test_ticks > 0) {
            spawn_test_ticks--;
            if(spawn_test_ticks == 0) {
                printf("selftest-spawn: 30 ticks after spawn/remove, no crash; OK\n");
                fflush(stdout);
                running = false;
            }
        }

        /*
          EVENTS
        */
        // Emit the synthetic (sim) input that fell due this frame, then
        // drain the SDL queue and dispatch it. Both live in events.cpp.
        emit_sim_events(game);
        poll_events(game);
        pf_a = std::chrono::steady_clock::now();

        /*
          LOGIC
        */
        // The fixed-timestep loop lives in tick.cpp.
        /* The VAB's headless transition hooks, then the LIVE scene's
           per-frame step. The hooks run first and a launch collapses the
           stack to Flight, so the scene is read after them. */
        /* --reload: the headless runtime load. Before the scene is read,
           since a load decides the scene. */
        if(!game.reloadDir.empty() && game.reloadMs >= 0 && !game.reloadFired
           && (int)(SDL_GetTicks() - game.loop_start_ms) >= game.reloadMs) {
            game.reloadFired = true;
            game.loadFrom(game.reloadDir);
        }
        /* --new-game: the headless hook for starting a game from the title
           screen (Game::newGame). Before the scene is read. */
        if(game.newGameMs >= 0 && !game.newGameFired
           && (int)(SDL_GetTicks() - game.loop_start_ms) >= game.newGameMs) {
            game.newGameFired = true;
            game.newGame();
        }
        /* --quit-title: the headless hook for the flight pause menu's
           "Quit to title". Before the scene is read (it tears the fleet
           down and lands on Title). */
        if(game.quitTitleMs >= 0 && !game.quitTitleFired
           && (int)(SDL_GetTicks() - game.loop_start_ms) >= game.quitTitleMs) {
            game.quitTitleFired = true;
            game.quitToTitle();
        }
        /* --space-center: the headless hook for the pause menu's "Space
           Center". Only from Flight (the hub is an excursion above a running
           game). */
        if(game.spaceCenterMs >= 0 && !game.spaceCenterFired
           && (int)(SDL_GetTicks() - game.loop_start_ms) >= game.spaceCenterMs) {
            game.spaceCenterFired = true;
            if(sceneIs(game, SceneId::Flight)) { pushScene(game, SceneId::SpaceCenter); }
        }
        /* --recover: the headless hook for the hub's "Recover Vessel". Fired
           after --space-center. */
        if(game.recoverMs >= 0 && !game.recoverFired
           && (int)(SDL_GetTicks() - game.loop_start_ms) >= game.recoverMs) {
            game.recoverFired = true;
            // The active ship may be a free EVA kerbal (a crew member),
            // which recoverActive refuses -- recover the ship that holds
            // the findings instead.
            if(game.ship != nullptr && game.ship->isEva()) {
                for(auto *s : collectVehicles(game.sys)) {
                    if(s->isEva()) { continue; }
                    bool holdsFinding = false;
                    for(Part *p : s->parts) {
                        if(!p->experiments.empty()) { holdsFinding = true; break; }
                    }
                    if(holdsFinding) { game.select_ship(s); break; }
                }
            }
            game.recoverActive();
        }
        /* --experiment: the headless hook for the part window's "Run
           Experiment". Mirrors the UI: a free EVA kerbal runs it itself,
           else the active ship's first aboard crew. */
        if(game.experimentMs >= 0 && !game.experimentFired
           && (int)(SDL_GetTicks() - game.loop_start_ms) >= game.experimentMs) {
            game.experimentFired = true;
            Kerbal *k = nullptr;
            if(game.ship != nullptr) {
                if(game.ship->isEva()) {
                    k = static_cast<Kerbal *>(game.ship);
                } else if(!game.ship->crew.empty()) {
                    k = static_cast<Kerbal *>(game.ship->crew.front());
                }
            }
            if(k != nullptr) { game.runExperiment(k); }
            else { printf("[hook] --experiment: no kerbal, ignored\n"); }
        }
        /* --pod-experiment: the headless hook for a science pod's "Run
           Experiment". Finds the active ship's first experiment-family part
           + its first aboard crew. */
        if(game.podExperimentMs >= 0 && !game.podExperimentFired
           && (int)(SDL_GetTicks() - game.loop_start_ms) >= game.podExperimentMs) {
            game.podExperimentFired = true;
            Part *pod = nullptr;
            Kerbal *k = nullptr;
            if(game.ship != nullptr) {
                for(Part *p : game.ship->parts) {
                    if(p->def != nullptr && !p->def->experiment_family.empty()) {
                        pod = p; break;
                    }
                }
                if(!game.ship->crew.empty()) {
                    k = static_cast<Kerbal *>(game.ship->crew.front());
                }
            }
            if(pod != nullptr && k != nullptr) { game.runPodExperiment(pod, k); }
            else { printf("[hook] --pod-experiment: no pod+kerbal, ignored\n"); }
        }
        /* --eva: the headless hook for the part window's "EVA" button
           (Game::kerbalEVA) -- take the active ship's first crew kerbal out
           of its capsule. Fired before --take / --store in the e2e. */
        if(game.evaMs >= 0 && !game.evaFired
           && (int)(SDL_GetTicks() - game.loop_start_ms) >= game.evaMs) {
            game.evaFired = true;
            if(game.ship != nullptr && !game.ship->crew.empty()) {
                Kerbal *k = static_cast<Kerbal *>(game.ship->crew.front());
                game.kerbalEVA(k);
            } else {
                printf("[hook] --eva: no crew aboard, ignored\n");
            }
        }
        /* --take: the headless hook for the take/store dance -- move the
           active ship's first held finding off its instrument onto its
           courier (the kerbal's suit). */
        if(game.takeMs >= 0 && !game.takeFired
           && (int)(SDL_GetTicks() - game.loop_start_ms) >= game.takeMs) {
            game.takeFired = true;
            Part *from = nullptr;   // the instrument holding a finding to take
            Part *to = nullptr;     // the courier (the kerbal's suit)
            // The instrument (pod) is on the ship's part tree, which after
            // an EVA is a DIFFERENT ship than the active one -- search the
            // whole fleet.
            for(auto *s : collectVehicles(game.sys)) {
                for(Part *p : s->parts) {
                    if(p->def != nullptr && !p->experiments.empty()
                       && p->def->experiment_storage == ExpStorage::Instrument) {
                        from = p; break;
                    }
                }
                if(from != nullptr) { break; }
            }
            // The dance needs a FREE (EVA) kerbal in reach of the part --
            // the same gate the Take button uses (kerbalInRange).
            if(from != nullptr) {
                for(Kerbal *k : freeKerbals(game.sys)) {
                    if(game.kerbalInRange(k, from) && !k->parts.empty()
                       && k->parts[0]->canHold(from->experiments[0])) {
                        to = k->parts[0]; break;
                    }
                }
            }
            if(from != nullptr && to != nullptr) { game.moveExperiment(from, to, 0); }
            else { printf("[hook] --take: no instrument/free-kerbal-in-reach, ignored\n"); }
        }
        /* --store: the headless hook for the take/store dance -- move the
           active ship's courier's first held finding onto its container
           (the capsule). */
        if(game.storeMs >= 0 && !game.storeFired
           && (int)(SDL_GetTicks() - game.loop_start_ms) >= game.storeMs) {
            game.storeFired = true;
            Part *from = nullptr;   // the courier (the kerbal's suit)
            Part *to = nullptr;     // the container (the capsule)
            // The container (capsule) is on the ship's part tree, a
            // DIFFERENT ship than the active one after an EVA.
            for(auto *s : collectVehicles(game.sys)) {
                for(Part *p : s->parts) {
                    if(p->def != nullptr
                       && p->def->experiment_storage == ExpStorage::Container) {
                        to = p; break;
                    }
                }
                if(to != nullptr) { break; }
            }
            // A FREE (EVA) kerbal in reach of the capsule, carrying a
            // finding -- the same gate the Store button uses.
            if(to != nullptr) {
                for(Kerbal *k : freeKerbals(game.sys)) {
                    if(game.kerbalInRange(k, to) && !k->parts.empty()
                       && !k->parts[0]->experiments.empty()) {
                        from = k->parts[0]; break;
                    }
                }
            }
            if(from != nullptr && to != nullptr) { game.moveExperiment(from, to, 0); }
            else { printf("[hook] --store: no free-kerbal-in-reach/container, ignored\n"); }
        }
        /* --tracking: the headless hook for the hub's "Tracking Station".
           Fired after --space-center. */
        if(game.trackingMs >= 0 && !game.trackingFired
           && (int)(SDL_GetTicks() - game.loop_start_ms) >= game.trackingMs) {
            game.trackingFired = true;
            pushScene(game, SceneId::TrackingStation);
        }
        /* --tracking-close: the headless hook for the tracking menu's "Back
           to Space Center" (popScene). Fired after --tracking. */
        if(game.trackingCloseMs >= 0 && !game.trackingCloseFired
           && (int)(SDL_GetTicks() - game.loop_start_ms) >= game.trackingCloseMs) {
            game.trackingCloseFired = true;
            if(sceneIs(game, SceneId::TrackingStation)) { popScene(game); }
            else { printf("[hook] --tracking-close: not in the tracking station, ignored\n"); }
        }
        /* --research: the headless hook for the hub's "Research Lab". Fired
           after --space-center. */
        if(game.researchMs >= 0 && !game.researchFired
           && (int)(SDL_GetTicks() - game.loop_start_ms) >= game.researchMs) {
            game.researchFired = true;
            pushScene(game, SceneId::ResearchLab);
        }
        /* --research-close: the headless hook for the lab's "Back to Space
           Center" (popScene). Fired after --research. */
        if(game.researchCloseMs >= 0 && !game.researchCloseFired
           && (int)(SDL_GetTicks() - game.loop_start_ms) >= game.researchCloseMs) {
            game.researchCloseFired = true;
            if(sceneIs(game, SceneId::ResearchLab)) { popScene(game); }
            else { printf("[hook] --research-close: not in the research lab, ignored\n"); }
        }
        /* --atlas-dump: the System Atlas (rows + dossiers) as text. Reads the
           system, not the window, so it works from any scene -- the terminal
           sanity check on the loaded data, and the e2e cover for the dossier
           (imgui Text rows are not clickable items, so --ui-list cannot see
           them). */
        if(game.atlasDumpMs >= 0 && !game.atlasDumpFired
           && (int)(SDL_GetTicks() - game.loop_start_ms) >= game.atlasDumpMs) {
            game.atlasDumpFired = true;
            dumpAtlas(game);
        }
        /* --switch-system: the headless hook for the in-process system switch
           (Game::switchSystem). Before the scene is read, since it tears the
           current system down and lands on the Title screen -- the only
           automated cover for a live system swap. */
        if(!game.switchSystemPath.empty() && game.switchSystemMs >= 0
           && !game.switchSystemFired
           && (int)(SDL_GetTicks() - game.loop_start_ms) >= game.switchSystemMs) {
            game.switchSystemFired = true;
            // A bad path throws from load_system -- catch it like the load
            // path does and keep running on the current system.
            try {
                game.switchSystem(game.switchSystemPath);
            } catch(const std::exception &e) {
                printf("[switch] cannot switch to '%s': %s\n",
                       game.switchSystemPath.c_str(), e.what());
                fflush(stdout);
                game.toast("Switch failed: %s", e.what());
            }
        }
        vabFireHooks(game);
        {
            const SceneDef &sc = curScene(game);
            sc.update(game);
        }
        pf_b = std::chrono::steady_clock::now();

        // Background jobs (the porkchop grid, the surface map, and terrain
        // patch subdivision): run the finished jobs' main-thread
        // continuations. Once per frame, BEFORE the UI reads the state those
        // continuations wrote.
        game.jobs.poll();
        // The engine hum: the active ship's thrust state, pilot scenes
        // only. Gain is the throttle; the sound is ON only while the ship
        // actually produces thrust.
        {
            const SceneDef &sc = curScene(game);
            if(sc.pilot && ship != nullptr) {
                // Firing = an engine armed THIS tick (getThrust() alone is
                // the POTENTIAL at the current throttle -- it would hum at
                // the default 100% with the thrust key never touched).
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
                // real-time callback never has to resample (backends
                // negotiate differently: Pulse/PipeWire ~48 kHz, ALSA the
                // hardware's 44.1 kHz).
                static const char *engineFile = nullptr;
                if(engineFile == nullptr) {
                    engineFile = (game.audio.deviceRate() == 48000)
                        ? "res/audio/rocket_engine.001.wav"    // 48 kHz original
                        : "res/audio/rocket_engine_44k.wav";   // converted for 44.1 kHz
                }
                game.audio.setLoop(engineFile, firing, ship->thruster_util);
            } else {
                game.audio.setLoop("", false, 0.0f);
            }
        }
        // Audio: reap finished one-shots and complete any engine stop-fade.
        // No-op when audio is unavailable.
        game.audio.update();
        // pf_swap defaults to pf_c so a frame that skips the render block
        // (redraw false) records render = present = 0.
        pf_c = std::chrono::steady_clock::now(); pf_swap = pf_c;

        /*
          RENDERING
        */
        if(game.redraw == true) {
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
            // The scene cannot change inside the render block, so read it once.
            const SceneDef &sc = curScene(game);
            // The VAB gets a desaturated steel-blue studio backdrop (no
            // skybox is drawn there); flight clears to black under the skybox.
            if(sc.backdrop == Backdrop::Studio) {
                display.Clear(0.55f, 0.62f, 0.68f, 1.0f);
            } else {
                display.Clear(0, 0, 0, 1);
            }

            // The 3D pass: the world + active ship (flight), or the
            // physics-free build tree (Vab).
            sc.draw3d(game);

            /* --map-dump: the orbit map's plane basis per combo slot, AFTER
               the 3D pass -- draw3d is what fills Game::view (updateShipView),
               and the Orbital slot is the ship's own plane. Dumped before it,
               the view is still zero and the slot silently reports the rail
               normal with a perfectly plausible sweep sign. Needs a ship, so
               unlike the atlas dump it is a no-op in the VAB. */
            if(game.mapDumpMs >= 0 && !game.mapDumpFired
               && (int)(SDL_GetTicks() - game.loop_start_ms) >= game.mapDumpMs) {
                game.mapDumpFired = true;
                dumpMapBasis(game);
            }

            /*
              ImGui stuff below
            */

            glUseProgram(0);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            if(poly_mode == true) {
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            }

            // The imgui pass: the live scene's widget set.
            sc.drawUi(game);

            // One-shot messages (g.toast): above everything, including the
            // menu (drawToasts, gameui.cpp).
            drawToasts(game);

            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            if(screenshot_requested == true) {
                // A user feature, so shots live under the data dir (like
                // saves/). UTC stamp + the shot count: unique by
                // construction, portable (gmtime is standard C). Main-thread
                // only.
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
                if(display.SaveScreenshot(fname)) {
                    screenshot_count++;
                }
                screenshot_requested = false;
            }

            // Mark the render/present boundary: everything above is issuing
            // GL commands; SwapBuffers is where the vsync block lives.
            pf_swap = std::chrono::steady_clock::now();
            display.SwapBuffers();
            check_gl_error();
        }

        // Close the frame's timing: fold this frame's phase times into the
        // Telemetry series (always) and, with --perf, the console running
        // totals.
        pf_d = std::chrono::steady_clock::now();
        const double f_events  = perf_ms(pf_iter, pf_a);
        const double f_logic   = perf_ms(pf_a, pf_b);
        const double f_jobs    = perf_ms(pf_b, pf_c);
        const double f_render  = perf_ms(pf_c, pf_swap);  // issue GL cmds
        const double f_present = perf_ms(pf_swap, pf_d);  // SwapBuffers (vsync)
        // Always push into the Telemetry window's series (wall-clock x-axis,
        // s since loop start).
        const double perf_t =
            std::chrono::duration<double>(pf_d - perf_loop_start).count();
        game.perf_events.push(perf_t, f_events);
        game.perf_logic.push(perf_t, f_logic);
        game.perf_jobs.push(perf_t, f_jobs);
        game.perf_render.push(perf_t, f_render);
        game.perf_present.push(perf_t, f_present);
        // --perf: fold into the console running totals + print the rolling line.
        if(perf_on) {
            p_events += f_events; p_logic += f_logic; p_jobs += f_jobs;
            p_render += f_render; p_present += f_present;
            p_total += perf_ms(pf_iter, pf_d);
            p_frames++; p_steps += game.phys_steps;
            w_events += f_events; w_logic += f_logic; w_jobs += f_jobs;
            w_render += f_render; w_present += f_present;
            w_frames++; w_steps += game.phys_steps;
            perf_roll();
        }
        game.phys_steps = 0;   // tick() re-arms it next frame

        // --frame-cap: burn the rest of the frame budget. Without this the
        // iteration spins at full speed whenever the swap isn't vsync-gated.
        if (cap_ms > 0) {
            const Uint32 used_ms = SDL_GetTicks() - iter_start_ms;
            if (used_ms < (Uint32)cap_ms) {
                SDL_Delay(cap_ms - used_ms);
            }
        }
    }

    // --perf: the final breakdown (the rolling lines are the live view).
    if(perf_on) { perf_summary(); }

    // The ships + space pads are owned by the bodies (TerrainBody::ships /
    // ::pads), so they are freed when the bodies are deleted below.

    // Stop the background worker BEFORE the bodies it may still hold (a
    // job captures a TerrainBody* and may be sampling it off-thread).
    // abort() drops the queued jobs and waits only for the in-flight one.
    game.jobs.abort();

    for(auto&& body : sys.bodies) { delete body; }

    // The shaders + textures + plume mesh are registry-owned (get_*):
    // they outlive this scope on purpose and must NOT be deleted here.
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
