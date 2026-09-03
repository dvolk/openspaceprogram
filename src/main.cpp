// Open Space Program

#include <stdio.h>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <ctime>
#include <cstdlib>
#include <sys/stat.h>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <map>
#include <set>

#include "SDL2/SDL.h"
#include "SDL_keycode.h"

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/gtc/noise.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/projection.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/polar_coordinates.hpp>

#define BT_USE_DOUBLE_PRECISION true
#include <bullet/btBulletDynamicsCommon.h>

#include "display.h"
#include "mesh.h"
#include "shader.h"
#include "camera.h"
#include "model.h"
#include "body.h"
#include "physics.h"
#include "gldebug.h"
#include "frame.h"
#include "transferplanner.h"
#include "shipdef.h"
#include "fleet.h"
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
#include "ships.h"
#include "game.h"
#include "events.h"
#include "tick.h"
#include "render.h"
#include "gameui.h"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include "cli.h"

#include "../middleware/imgui/imgui.h"
#include "../middleware/imgui/backends/imgui_impl_sdl2.h"
#include "../middleware/imgui/backends/imgui_impl_opengl3.h"
#include "../middleware/implot/implot.h"


/* ResourceType / ResourceContent / PartDef live in shipdef.h (the GL-free
   ship/part data model), shared with the JSON loaders and the headless
   tests. */

int main(int argc, char **argv)
{
    const auto prog_start = std::chrono::steady_clock::now();

    GameArgs args;
    int exit_code = 1;
    if(!parse_cli(argc, argv, args, &exit_code)) { return exit_code; }

    Renderer display(args.screen_width, args.screen_height, args.window_mode,
                     args.gl_debug);
    check_gl_error();
    const Uint32 sim_win_id = SDL_GetWindowID(display.get_display());
    /* --sim-press: resolve keycodes to scancodes now that SDL is initialized
       (SDL_GetScancodeFromKey needs SDL_Init; the CLI parse ran before the
       Renderer above created the video subsystem). */
    for(auto &p : args.sim_presses) {
        p.sc = SDL_GetScancodeFromKey(p.key);
        if(p.sc == SDL_SCANCODE_UNKNOWN) {
            printf("warning: --sim-press key %d has no scancode in the "
                   "current keyboard layout: one-shot actions fire, held "
                   "commands for it do not\n", (int)p.key);
        }
    }
    ImGuiContext* ctx1 = ImGui::CreateContext();
    ImGui::SetCurrentContext(ctx1);
    // ImPlot keeps its own state per imgui context (v1.0 requires an
    // explicit context; it is bound to the current one at creation).
    ImPlot::CreateContext();
    ImGui_ImplSDL2_InitForOpenGL(display.get_display(), SDL_GL_GetCurrentContext());
    ImGui_ImplOpenGL3_Init("#version 430");
    check_gl_error();

    ImGuiIO& io = ImGui::GetIO();
    // No imgui.ini: window layout must not survive between runs or clobber
    // the layout the code sets up each frame.
    io.IniFilename = nullptr;
    // Normal and big faces are the same font (the big one at 2x size), so
    // the whole UI is one typeface; --font picks which.
    io.Fonts->AddFontFromFileTTF(args.font_path.c_str(), args.font_size);
    // The big face (2x size) for the HUD + main menu; the UI pass
    // (gameui.cpp) draws with it via the game.
    ImFont *bigger = io.Fonts->AddFontFromFileTTF(args.font_path.c_str(), 2.0f * args.font_size);
    check_gl_error();

    // start bullet; see physics.cpp
    void create_physics(void);
    create_physics();
    check_gl_error();

    /* data init */
    Shader *partsshader = new Shader;
    partsshader->registerAttribs({ "position", "uv", "normal" });
    partsshader->registerUniforms({ "MVP", "Normal", "lightDirection", "shadow" });
    partsshader->FromFile("./res/partsShader");

    Shader *terrainshader = new Shader;
    terrainshader->registerAttribs({ "position", "normal", "color" });
    terrainshader->registerUniforms({ "MVP", "Normal", "lightDirection", "color" });
    terrainshader->FromFile("./res/terrainShader");

    Shader *sunshader = new Shader;
    sunshader->registerAttribs({ "position", "normal", "color" });
    sunshader->registerUniforms({ "MVP", "Normal", "lightDirection", "color" });
    sunshader->FromFile("./res/sunShader");

    // Atmosphere rim shell (Fresnel limb glow). See reports/atmosphere2026_08_25.
    Shader *atmosphereshader = new Shader;
    atmosphereshader->registerAttribs({ "position", "normal" });
    atmosphereshader->registerUniforms({ "MVP", "Normal", "cameraPos",
                                         "color", "intensity", "power",
                                         "lightDirection" });
    atmosphereshader->FromFile("./res/atmosphereShader");

    Shader *skyboxshader = new Shader;
    skyboxshader->registerAttribs({ "position" });
    skyboxshader->registerUniforms({ "projectionview" });
    skyboxshader->FromFile("./res/skyboxShader");

    Shader *lineshader = new Shader;
    lineshader->registerAttribs({ "position" });
    lineshader->registerUniforms({ "MVP", "color" });
    lineshader->FromFile("./res/lineShader2");

    PostFX *postfx = new PostFX;
    // Create every built-in effect up front (no mid-frame shader
    // compilation) so the Settings window can toggle them at runtime;
    // they all start disabled and the --postfx selection enables a subset.
    for(const std::string &name : PostFX::Available()) {
        postfx->AddEffect(name);
    }
    // Each --postfx value may itself be comma-separated, so both
    // --postfx crt,grain and --postfx crt --postfx grain work.
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

    System sys = load_system(args.system_file.c_str(), terrainshader, sunshader);
    TerrainBody *sun = sys.root;
    TerrainBody *home;
    if(args.body_name.empty()) {
        home = sys.home;
    } else {
        home = sys.find(args.body_name);
        if(home == nullptr) {
            std::string avail;
            for(size_t i = 0; i < sys.bodies.size(); i++) {
                if(i) avail += ", ";
                avail += sys.bodies[i]->name;
            }
            printf("error: unknown body '%s' (available: %s)\n",
                   args.body_name.c_str(), avail.c_str());
            return 1;
        }
    }

    // Build the atmosphere rim shells now that the bodies + shader exist.
    // Bodies without an atmosphere are no-ops (no mesh, no draw cost).
    for(auto&& b : sys.bodies) {
        b->BuildAtmosphere(atmosphereshader);
    }

    /* The ships are built from JSON: the parts catalog (res/parts.json)
       supplies each part's mass + behavior, the ship defs supply the stack
       order + offsets, and the fleet supplies one entry per ship: its def,
       name, body and scenario. The fleet comes from --fleet (res/fleet.json)
       or, when that is not given, from the --ship flags as a uniform fleet
       (all entries share the --body/--scenario). Omitted entry body/scenario
       fall back to the CLI values. Ships sharing a (body, scenario) pair are
       slotted: pad slots 20 m apart along the pad, orbit slots 100 m apart
       along the orbit binormal. */
    Ships ships(args.parts_file, partsshader, sun);

    // The running game: borrows the subsystems above and owns the runtime
    // state (camera, clock, active ship, input/UI flags, the orbit-camera
    // focus targets, the UI window registry) plus the control transitions
    // (select/remove a ship, enter rails warp, toggle the windows -- see
    // game.cpp). The event dispatch (events.cpp) and the loop below drive
    // it through this, so the state has a single home.
    Game game(display, postfx, ships, sys, sun, home, args, sim_win_id);
    game.bigger = bigger;   // the UI pass (gameui.cpp) draws with it
    game.apply_ui_style();  // the Settings defaults (dark theme, scale 1.0)

    // --start-time: start the analytic clock (and every body's orbit and
    // spin, which are functions of it) at a later instant. Must happen
    // before the fleet spawns -- the orbit scenarios read the home body's
    // frame state -- and the tick only re-propagates frames while
    // unpaused, so a paused start has to be propagated here or the first
    // frame renders the t=0 system.
    game.time = args.start_time;
    sun->frame->UpdateOrbitRails(args.start_time);
    if(args.start_time > 0.0) {
        printf("Starting at sim time t = %.0f s\n", args.start_time);
    }

    // The runtime state lives in `game`. These local references keep the
    // loop body reading exactly as before; they alias game's members, so
    // the writes here and the control transitions in game.cpp hit the same
    // storage. (screenshot_count is pure loop bookkeeping and stays local.)
    Vehicle *&ship = game.ship;
    int &time_accel = game.time_accel;
    int &cam_speed = game.cam_speed;
    bool &rmbCam = game.rmbCam;
    bool &poly_mode = game.poly_mode;
    bool &screenshot_requested = game.screenshot_requested;
    bool &running = game.running;
    int &focusBody = game.focusBody;
    double &time = game.time;

    std::vector<FleetEntry> fleet_entries;
    if(!args.fleet_file.empty()) {
        fleet_entries = load_fleet(args.fleet_file.c_str()).ships;
    } else {
        if(args.ship_files.empty()) { args.ship_files.push_back("res/ships/racer.json"); }
        for(size_t i = 0; i < args.ship_files.size(); i++) {
            FleetEntry e;
            e.ship = args.ship_files[i];
            fleet_entries.push_back(e);
        }
    }

    Vehicle *first = nullptr;
    if(!args.radial_test.empty()) {
        RadialTestShip rts = build_radial_test_ship(
            args.radial_test, args.scenario_given, args.scenario,
            ships.catalog(), home, sun, partsshader);
        ships.add_ship(rts.v, home, rts.sc, rts.slot);
        first = rts.v;
    } else {
        first = ships.build_fleet(fleet_entries, sys, home, args.scenario);
    }
    check_gl_error();

    /* Apply each ship's scenario (before the camera is constructed,
       so the camera focuses on the spawn point). Ships sharing a
       body+scenario group get their own orbit slot (100 m apart along the
       orbit binormal) so they don't spawn on top of each other. */
    ships.apply_scenarios(sys);

    /* the active (player-controlled) ship: the first one built; Tab / the
       SHIPS window switch it. game.ship always points at it, so the HUD,
       camera, input and draw code follow the active ship without special
       cases. */
    ship = first;

    /* Idle ships park on rails: flying ones coast on their conic, pad
       ships freeze in the surface frame (their pose rides the planet's
       spin via the render transform). Ships that are neither in free
       fall nor grounded refuse and stay in the physics world. */
    for(auto *b : sys.bodies) {
        for(auto *s : b->ships) {
            if(s != ship) { s->goOnRails(); }
        }
    }

    Mesh *engine_plume_mesh = new Mesh;
    engine_plume_mesh->FromFile("./res/engine_plume.obj", false);
    Texture *engine_plume_texture = load_texture("res/engine_plume.png");
    Model *engine_plume_model = new Model;
    engine_plume_model->FromData(engine_plume_mesh, partsshader, engine_plume_texture);

    Shader *billboardshader = new Shader;
    billboardshader->registerAttribs({ "position", "texcoord", "normal" });
    billboardshader->registerUniforms({ "MVP", "color_uniform" });
    billboardshader->FromFile("./res/billboardshader");

    // Billboard icons opt out of mip chains: their alpha cutouts bleed
    // into the neighbouring level when minified.
    Texture * front_indicator_texture = load_texture("res/front_crosshair.png", false);
    Texture * prograde_indicator_texture = load_texture("res/prograde_icon.png", false);
    Texture * retrograde_indicator_texture = load_texture("res/retrograde_icon.png", false);
    Texture * radial_in_indicator_texture = load_texture("res/radial_in_icon.png", false);
    Texture * radial_out_indicator_texture = load_texture("res/radial_out_icon.png", false);
    Texture * normal_plus_indicator_texture = load_texture("res/normal_plus_icon.png", false);
    Texture * normal_minus_indicator_texture = load_texture("res/normal_minus_icon.png", false);

    glm::vec4 billboardcolor = glm::vec4(1, 1, 1, 1.0); // TODO should these be different colors?

    Billboard *front_indicator =
        mk_billboard(billboardshader, front_indicator_texture, 1.0, 1.0, billboardcolor);
    Billboard *prograde_indicator =
        mk_billboard(billboardshader, prograde_indicator_texture, 1.0, 1.0, billboardcolor);
    Billboard *retrograde_indicator =
        mk_billboard(billboardshader, retrograde_indicator_texture, 1.0, 1.0, billboardcolor);
    Billboard *radial_in_indicator =
        mk_billboard(billboardshader, radial_in_indicator_texture, 1.0, 1.0, billboardcolor);
    Billboard *radial_out_indicator =
        mk_billboard(billboardshader, radial_out_indicator_texture, 1.0, 1.0, billboardcolor);
    Billboard *normal_plus_indicator =
        mk_billboard(billboardshader, normal_plus_indicator_texture, 1.0, 1.0, billboardcolor);
    Billboard *normal_minus_indicator =
        mk_billboard(billboardshader, normal_minus_indicator_texture, 1.0, 1.0, billboardcolor);
    // Transfer burn direction (TRANSFER window): the prograde icon in
    // KSP blue, pointing where the departure burn should point.
    Billboard *burn_indicator =
        mk_billboard(billboardshader, prograde_indicator_texture, 1.0, 1.0,
                     glm::vec4(0.2f, 0.45f, 1.0f, 1.0f));

    /* camera init */
    const float camFov = (float)glm::radians(args.camFovDeg);
    // The drawable size the Renderer actually got (the WM may have clamped
    // it, or fullscreen may have used the display mode).
    const float camAspect = (float)display.get_width() / (float)display.get_height();
    const float camZNear = 1.0f;
    // zFar must exceed the farthest visible body. The log-depth shaders
    // (res/*Shader.vs) define the hard far limit as `far = 1e13` m, which
    // covers the real solar system (Pluto at ~5.9e12 m) and KSP-style
    // AU scales (~1.4e10 m). Keep zFar consistent with that.
    const float camZFar = 1e13;

    // One camera, two modes (orbit + free): starts in Orbit mode focused on
    // the ship; --free-cam-* / use_free_cam drops it into free flight at the
    // (possibly overridden) pose. The terrain LOD reads the live one.
    Camera *cam = new Camera(GetPosition(ship->controller->body),
                             camFov, camAspect, camZNear, camZFar);
    cam->setViewport(display.get_width(), display.get_height());
    game.camera = cam;
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

    // Bodies the orbit camera can target (the ship is the default). Built from
    // the loaded system: the ship, then every body in the system (in file
    // order), so G cycles through all of them. game.focusWorldPos() resolves
    // one to a ship-frame position.
    game.focusTargets.push_back({ "ship", nullptr });
    for (TerrainBody *b : sys.bodies) {
        game.focusTargets.push_back({ b->name.c_str(), b });
    }
    game.numFocusTargets = (int)game.focusTargets.size();

    int screenshot_count = 0;
    SDL_SetRelativeMouseMode(SDL_FALSE);

    // kRailsWarp is defined in game.h (the rails-warp threshold).
    time_accel = args.initial_time_accel;

    /* Starting the game directly in rails warp (accel > 10): the active
       ship parks too (works on the pad -- that is the frozen mode),
       unless some ship is not rail-eligible, in which case clamp to the
       top physics warp (10). */
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

    // The per-window UI options + the window registry (game.cpp): the
    // layout slots, the default-open states and the TAB-toggle table all
    // live on the game; the UI pass (gameui.cpp) draws with them.
    game.setup_ui_windows();

    // Transfer planner (the TRANSFER window + the map's transfer conic +
    // the blue burn-direction icon): the state (targets, selection, solver
    // cache) and the per-frame rebuild / solve / --xfer-log live in the
    // TransferPlanner (transferplanner.cpp). The UI pass (gameui.cpp)
    // reads it through its own aliases.
    TransferPlanner xferPlanner(game);

    Skybox skybox;
    skybox.init();

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
            const double a = (2 * M_PI) * float(i-1)/float(n);
            xzinterface.positions.push_back(glm::vec3(r * cos(a), 0, r * sin(a)));  // y=0 -> XZ plane
            xyinterface.positions.push_back(glm::vec3(r * cos(a), r * sin(a), 0));  // z=0 -> XY plane
        }
        skyline_xz->InitMesh(xzinterface);
        skyline_xy->InitMesh(xyinterface);
    }

    // Hand the render resources to the game (render.cpp draws with them;
    // main still owns their lifetime, the teardown below).
    game.skybox = &skybox;
    game.skyboxshader = skyboxshader;
    game.lineshader = lineshader;
    game.engine_plume_model = engine_plume_model;
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

    /* Runtime spawn: Ships::spawn_ship (ships.cpp) -- place + apply the
       scenario + park on rails; appended at the end so it is never the
       active one. Called as ships.spawn_ship(def, name, home, sc, sys). */

    /* --selftest-spawn: exercise the runtime spawn/remove path. Spawn a copy
       of the active ship, remove it, then spawn-select-remove the active one
       (exercising the control handoff). Each step is checked against the
       expected fleet size + active index. Runs before the loop; the loop
       then takes a few physics ticks to prove the world is stable and exits. */
    int spawn_test_ticks = 0;
    if(args.selftest_spawn) {
        if(ship->defPath.empty()) {
            printf("selftest-spawn: SKIP (active ship has no def: test ship)\n");
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
                                           ship->scenario, sys);
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
                                            ship->scenario, sys);
            game.select_ship(sp2);
            printf("spawn 2 + select: active=%s size=%zu\n",
                   ship->name.c_str(), collectVehicles(sys).size());
            if(ship != sp2) { ok = false; }
            game.remove_ship(sp2);
            printf("remove 2 (active): active=%s size=%zu\n",
                   ship->name.c_str(), collectVehicles(sys).size());
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

    // Per-frame phase timing. The steady_clock reads and the push into the
    // Game::perf_* series run EVERY frame (the Telemetry window reads them);
    // the cost is a handful of vDSO clock reads + five ring writes, negligible.
    // --perf only controls the console output: when set, a rolling line prints
    // ~every second (perf_roll) and a full summary prints at exit
    // (perf_summary). The "logic" phase is where the Part*/Body* indirection
    // lives (tick -> physics_tick -> ships -> parts -> bodies); the per-substep
    // number is the one to compare across refactors.
    const bool perf_on = args.perf;
    double p_events = 0.0, p_logic = 0.0, p_jobs = 0.0, p_render = 0.0,
           p_present = 0.0, p_total = 0.0;                                     // cumulative ms
    long long p_frames = 0, p_steps = 0;                                       // cumulative counts
    double w_events = 0.0, w_logic = 0.0, w_jobs = 0.0, w_render = 0.0,
           w_present = 0.0;                                                    // rolling-window ms
    long long w_frames = 0, w_steps = 0;                                       // rolling-window counts
    // This frame's marks. pf_swap sits between the last draw call and the
    // SwapBuffers, so "render" = issuing the GL commands and "present" = the
    // SwapBuffers (which blocks on vsync -- that's the display pacing, not
    // render cost; keeping the two apart is why the breakdown is honest).
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
        // drain the SDL queue and dispatch it (quit, resize, keybinds, the
        // RMB camera-look and the wheel). Both live in events.cpp and drive
        // state through the game.
        emit_sim_events(game);
        poll_events(game);
        pf_a = std::chrono::steady_clock::now();

        /*
          LOGIC
        */
        // The fixed-timestep loop (command arming, the substepped physics,
        // the spin/orbit/dbg logs) lives in tick.cpp: it advances the
        // game's clock and marks the frame for a redraw.
        tick(game);
        pf_b = std::chrono::steady_clock::now();

        // Background jobs (the porkchop grid, the surface map, and
        // terrain patch subdivision): run the finished jobs' main-thread
        // continuations, which publish their results into game state.
        // Once per frame, BEFORE the UI reads the state those
        // continuations wrote. (Per-job "working on it"
        // state lives in the window that owns the job, e.g. the Porkchop's
        // "sweeping ..." -- not a global HUD line.)
        game.jobs.poll();
        // pf_swap defaults to pf_c so a frame that skips the render block
        // (redraw false) records render = present = 0, not a stale window.
        pf_c = std::chrono::steady_clock::now(); pf_swap = pf_c;

        /*
          RENDERING
        */
        if(game.redraw == true) {
            check_gl_error();
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplSDL2_NewFrame();
            ImGui::NewFrame();
            check_gl_error();

            if(poly_mode == true) {
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                check_gl_error();
            }

            postfx->Begin();  // no-op unless --postfx effects are active
            display.Clear(0, 0, 0, 1);

            // The 3D pass (render.cpp): the world, the active ship's
            // per-frame state (game.view) and the overlays.
            draw3d(game, xferPlanner);

            /*
              ImGui stuff below
            */

            glUseProgram(0);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            if(poly_mode == true) {
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            }

            // The readout windows (HUD .. RESOURCES) live in gameui.cpp.
            drawUIReadouts(game, xferPlanner);

            // The orbital map (gameui.cpp): the transfer conic and
            // the target highlight come from the planner. Drawn after
            // the readouts, before the main menu.
            drawUIMap(game, xferPlanner);

            // The open part windows (gameui.cpp): one per part the
            // player right-clicked in the 3D view.
            drawPartWindows(game);

            // The main menu (gameui.cpp): drawn last so it sits on top.
            drawMainMenu(game);

            // One-shot messages (g.toast): above everything, including the
            // menu (drawToasts, gameui.cpp).
            drawToasts(game);

            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            if(screenshot_requested == true) {
                char fname[256];
                time_t now = ::time(nullptr);
                struct tm tm;
                localtime_r(&now, &tm);
                char stamp[32];
                strftime(stamp, sizeof(stamp), "%Y_%m_%d_%H_%M_%S", &tm);
                snprintf(fname, sizeof(fname), "./tmp/osp_%s.png", stamp);
                mkdir("./tmp", 0755);
                if(display.SaveScreenshot(fname)) {
                    screenshot_count++;
                }
                screenshot_requested = false;
            }

            // Mark the render/present boundary: everything above is issuing
            // GL commands (the real render cost); SwapBuffers is where the
            // vsync block lives (the present cost).
            pf_swap = std::chrono::steady_clock::now();
            display.SwapBuffers();
            check_gl_error();
        }

        // Close the frame's timing: fold this frame's phase times into the
        // Telemetry series (always) and, with --perf, the console running
        // totals (the rolling line prints when a second has passed).
        pf_d = std::chrono::steady_clock::now();
        const double f_events  = perf_ms(pf_iter, pf_a);
        const double f_logic   = perf_ms(pf_a, pf_b);
        const double f_jobs    = perf_ms(pf_b, pf_c);
        const double f_render  = perf_ms(pf_c, pf_swap);  // issue GL cmds
        const double f_present = perf_ms(pf_swap, pf_d);  // SwapBuffers (vsync)
        // Always push into the Telemetry window's series (wall-clock x-axis,
        // s since loop start; the ring dedups on the last sample's time).
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
        // iteration spins at full speed whenever the swap isn't vsync-gated
        // (paused VAB, headless, vsync off) -- 100% of a core doing nothing.
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
    // ::pads), so they are freed when the bodies are deleted below -- no
    // separate ships.clear() here (the bodies would dangle).

    // Drain the background worker BEFORE the bodies it may still hold
    // (the Surface Map job captures a TerrainBody* and samples its
    // surface off-thread): game's destructor would join the worker only
    // on the way out of main, AFTER the deletes below.
    game.jobs.join();

    for(auto&& body : sys.bodies) { delete body; }

    delete partsshader;
    delete sunshader;
    delete terrainshader;
    delete atmosphereshader;
    delete billboardshader;
    delete skyboxshader;
    delete postfx;

    delete front_indicator;
    delete prograde_indicator;
    delete retrograde_indicator;

    delete front_indicator_texture;
    delete prograde_indicator_texture;
    delete retrograde_indicator_texture;

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    return 0;
}
