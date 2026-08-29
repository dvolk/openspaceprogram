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
#include "calendar.h"
#include "orbit.h"
#include "orbitmap.h"
#include "orbitsample.h"
#include "transfer.h"
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
                if(!postfx->AddEffect(name)) {
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

    std::vector<TerrainBody *> planets = sys.bodies;

    // Build the atmosphere rim shells now that the bodies + shader exist.
    // Bodies without an atmosphere are no-ops (no mesh, no draw cost).
    for(auto&& b : planets) {
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

    // The runtime state lives in `game`. These local references keep the
    // loop body reading exactly as before; they alias game's members, so
    // the writes here and the control transitions in game.cpp hit the same
    // storage. (screenshot_count is pure loop bookkeeping and stays local.)
    Vehicle *&ship = game.ship;
    int &activeIdx = game.activeIdx;
    Camera *&camera = game.camera;
    int &camMode = game.camMode;
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

    if(!args.radial_test.empty()) {
        RadialTestShip rts = build_radial_test_ship(
            args.radial_test, args.scenario_given, args.scenario,
            ships.catalog(), home, sun, partsshader);
        ships.add_ship(rts.v, home, rts.sc, rts.slot);
    } else {
        ships.build_fleet(fleet_entries, sys, home, args.scenario);
    }
    check_gl_error();

    /* Apply each ship's scenario (before the camera is constructed,
       so the camera focuses on the spawn point). Ships sharing a
       body+scenario group get their own orbit slot (100 m apart along the
       orbit binormal) so they don't spawn on top of each other. */
    ships.apply_scenarios(sys);

    /* the active (player-controlled) ship: Tab / the SHIPS window switch
       it; game.ship always points at it (activeIdx starts at 0, the first
       ship), so the HUD, camera, input and draw code follow the active
       ship without special cases. */
    ship = ships[0];

    /* Idle ships park on rails: flying ones coast on their conic, pad
       ships freeze in the surface frame (their pose rides the planet's
       spin via the render transform). Ships that are neither in free
       fall nor grounded refuse and stay in the physics world. */
    for(size_t i = 0; i < ships.size(); i++) {
        if((int)i != activeIdx) { ships[i]->goOnRails(); }
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

    OrbitCamera *orbitCam = new OrbitCamera(GetPosition(ship->controller),
                                            camFov, camAspect, camZNear, camZFar);
    glm::dvec3 freeCamPos  = orbitCam->GetPos();
    glm::dvec3 freeCamFwd  = orbitCam->GetForward();
    glm::dvec3 freeCamUp   = orbitCam->up;
    if(args.free_cam_pos.size() == 3) { // TODO do we need these guards?
        freeCamPos = glm::dvec3(args.free_cam_pos[0], args.free_cam_pos[1], args.free_cam_pos[2]);
    }
    if(args.free_cam_fwd.size() == 3) {
        freeCamFwd = glm::dvec3(args.free_cam_fwd[0], args.free_cam_fwd[1], args.free_cam_fwd[2]);
    }
    if(args.free_cam_up.size() == 3) {
        freeCamUp = glm::dvec3(args.free_cam_up[0], args.free_cam_up[1], args.free_cam_up[2]);
    }
    FreeCamera *freeCam = new FreeCamera(freeCamPos, freeCamFwd, freeCamUp,
                                         camFov, camAspect, camZNear, camZFar);
    game.orbitCam = orbitCam;
    game.freeCam = freeCam;
    camera = orbitCam;   // active camera
    camMode = CAM_ORBIT;
    if(args.use_free_cam) {
        camMode = CAM_FREE;
        camera = freeCam;
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

    /* Starting the game directly in rails warp: the active ship parks
       too (works on the pad -- that is the frozen mode), unless some
       ship is not rail-eligible, in which case clamp to physics warp. */
    if(time_accel >= kRailsWarp) {
        bool all_eligible = true;
        for(auto *s : ships) {
            if(!s->canRail()) {
                printf("Rails warp refused at start: '%s' is neither in free "
                       "fall nor grounded; clamping time accel to 1000\n",
                       s->name.c_str());
                all_eligible = false;
                break;
            }
        }
        if(all_eligible) {
            for(auto *s : ships) { s->goOnRails(); }
        } else {
            time_accel = 1000;
        }
    }
    cam_speed = 1;

    // The per-window UI options + the window registry (game.cpp): the
    // layout slots, the default-open states and the TAB-toggle table all
    // live on the game; the UI pass (gameui.cpp) draws with them.
    game.setup_ui_windows();

    // Orbital map: meters per pixel (the "Scale" slider) + the chosen map
    // plane (0 = equatorial, 1 = ecliptic, 2 = orbital). Persistent UI state,
    // like the transfer planner's below.
    float map_scale = 6000.0f;
    int map_plane = 0;
    // Pan offset from the window center, in pixels (P4 navigation): the focus
    // no longer has to sit dead-center. Wheel zooms to the cursor, a left
    // drag pans, and "Reset view" zeros this (and the scale).
    ImVec2 map_pan = ImVec2(0.0f, 0.0f);
    // Optional overlays, toggleable from the map's controls.
    bool map_show_soi = true;   // spheres-of-influence rings
    bool map_show_vel = true;   // the ship's velocity (prograde) arrow
    // Right-clicking the map window cycles its chrome: 0 = full window with
    // the control widgets (plane, scale, checkboxes, legend), 1 = window
    // with only the bare map, 2 = no window chrome at all (title bar,
    // border and background hidden -- the map just floats over the 3D view).
    // Modes 1 and 2 keep pan/zoom working.
    int map_mode = 0;
    // Cached orbit samplings, one entry per orbiting object (keyed on its
    // pointer -- the ship for the ship's orbit, a body for a child's; a given
    // ship always has exactly one entry, overwritten on each sample). Reuse
    // is only trusted while the orbiting object is on a fixed Keplerian conic:
    // the ship passes its onRails flag, terrain bodies are always on theirs.
    // See OrbitSampleCache.
    std::map<const void *, OrbitSampleCache> orbit_caches;

    // Transfer planner (TRANSFER window + the blue burn-direction icon):
    // the state (targets, selection, solver cache) and the per-frame
    // rebuild / solve / --xfer-log live in the TransferPlanner
    // (transferplanner.cpp). main talks to it through these aliases, so
    // the TRANSFER window and the orbital map read exactly as before.
    TransferPlanner xferPlanner(game);
    std::vector<TransferPlanner::XferTarget> &xferTargets = xferPlanner.xferTargets;
    int &xfer_target = xferPlanner.xfer_target;
    bool &xfer_auto = xferPlanner.xfer_auto;
    float &xfer_tof_log = xferPlanner.xfer_tof_log;
    auto &xfer = xferPlanner.xfer;

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

    // The 3D pass (render.cpp) emits the active ship's per-frame state
    // into game.view; the orbital map (still here -- the last UI
    // extraction) names the few locals it reads.
    OrbitElements &o = game.view.o;
    double &mu = game.view.mu;
    glm::dvec3 &orbit_pos = game.view.orbit_pos;
    glm::dvec3 &orbit_vel = game.view.orbit_vel;

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
            const size_t base = ships.size();
            const int origActive = activeIdx;
            bool ok = true;
            printf("== selftest-spawn: %zu ships at start, active %d (%s) ==\n",
                   base, origActive, ship->name.c_str());

            // 1) spawn a copy of the active ship -> appended at the end;
            //    the active ship must be untouched
            int sp = ships.spawn_ship(ship->defPath, "", ships.homeOf(activeIdx),
                                      ships.scenarioOf(activeIdx), sys);
            printf("spawn 1: new idx=%d size=%zu activeIdx=%d\n",
                   sp, ships.size(), activeIdx);
            if(ships.size() != base + 1 || sp != (int)base || activeIdx != origActive) { ok = false; }

            // 2) remove the ship we just spawned -> size back to base,
            //    active unchanged
            game.remove_ship(sp);
            printf("remove 1: size=%zu activeIdx=%d\n", ships.size(), activeIdx);
            if(ships.size() != base || activeIdx != origActive) { ok = false; }

            // 3) spawn again, select it, remove it (the active one) -> the
            //    control must hand off and the size return to base
            int sp2 = ships.spawn_ship(ship->defPath, "", ships.homeOf(activeIdx),
                                       ships.scenarioOf(activeIdx), sys);
            game.select_ship(sp2);
            printf("spawn 2 + select: activeIdx=%d size=%zu\n", activeIdx, ships.size());
            if(activeIdx != sp2) { ok = false; }
            game.remove_ship(activeIdx);
            printf("remove 2 (active): activeIdx=%d size=%zu\n", activeIdx, ships.size());
            if(ships.size() != base) { ok = false; }

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

    /* main loop timing from
       http://gafferongames.com/game-physics/fix-your-timestep/
    */
    while (running == true) {
        const Uint32 iter_start_ms = SDL_GetTicks();

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

        /*
          LOGIC
        */
        // The fixed-timestep loop (command arming, the substepped physics,
        // the spin/orbit/dbg logs) lives in tick.cpp: it advances the
        // game's clock and marks the frame for a redraw.
        tick(game);

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

            // Mode 2 strips the window chrome entirely (see map_mode): the
            // window is invisible but still hit-tested, so the map below
            // keeps pan/zoom and the right-click cycle.
            if(map_mode == 2) {
                game.o_map.flags |= ImGuiWindowFlags_NoDecoration |
                                    ImGuiWindowFlags_NoBackground;
            } else {
                game.o_map.flags = 0;
            }
            ui::Window("Orbital map", game.o_map, [&] {
                // Right-click anywhere in the window cycles the chrome:
                // full window -> bare map -> no window -> full window.
                // Over the map this is safe: imgui owns the mouse here, so
                // the RMB camera orbit (gated on !WantCaptureMouse) never
                // fires.
                if(ImGui::IsWindowHovered() &&
                   ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
                    map_mode = (map_mode + 1) % 3;
                }
                const float kMapSize = 360.0f;   // the map square (layout below)
                // The ship's trajectory around the focus: a closed ellipse
                // (a coasting Kepler orbit) or, when the ship is escaping or
                // flying by (ecc >= 1 -- e.g. right after switching SOI to a
                // body you are approaching), an open hyperbolic/parabolic arc.
                // Both draw the same way (a projected polyline); only the
                // sampling differs. Top-down view in the focus's inertial
                // frame, so the trajectory's true 3D orientation shows through
                // the projection.
                const int N = 64;
                const bool closed = (o.ecc < 1.0);
                std::vector<glm::dvec3> traj_pts;
                if(closed) {
                    // Sampled through a per-ship cache, trusted only while the
                    // ship is on rails (coasting on its Keplerian conic). Off
                    // rails -- Bullet-integrated, or right after a burn /
                    // staging / SOI switch / crash (all of which clear onRails)
                    // -- the orbit is moving, so re-sample every frame. See
                    // OrbitSampleCache.
                    traj_pts = orbit_caches[(const void *)ship].sample(
                        orbit_pos, orbit_vel, mu, N, ship->onRails);
                } else {
                    // Open trajectory: an arc around periapsis, truncated where
                    // it would run off to infinity. r_cap is the current view
                    // extent (the map square's width in world units) so the
                    // curve reaches the edge of the view, but never smaller
                    // than a few periapsis radii or the ship's current radius
                    // (so the ship itself lies on the arc).
                    const double r_cap = std::max<double>(
                        kMapSize * (double)map_scale,
                        std::max(4.0 * o.periapsis, o.distance));
                    traj_pts = sampleOpenTrajectory(orbit_pos, orbit_vel, mu, N, r_cap);
                }

                // Periapsis (both cases) and apoapsis (closed only). A closed
                // orbit propagates to each apsis (exact); an open arc has no
                // apoapsis, and its periapsis point is radius o.periapsis
                // along the eccentricity vector (which points to periapsis) --
                // no propagation needed.
                glm::dvec3 peri_p, apo_p, tmp;
                bool have_peri = false, have_apo = false;
                if(closed) {
                    if(o.time_to_peri > 0.0) {
                        propagateKepler(orbit_pos, orbit_vel, mu, o.time_to_peri, peri_p, tmp);
                        have_peri = true;
                    }
                    if(o.time_to_apo > 0.0) {
                        propagateKepler(orbit_pos, orbit_vel, mu, o.time_to_apo, apo_p, tmp);
                        have_apo = true;
                    }
                } else {
                    const glm::dvec3 h = glm::cross(orbit_pos, orbit_vel);
                    const double hl = glm::length(h);
                    if(hl > 1e-9) {
                        const glm::dvec3 evec =
                            glm::cross(orbit_vel, h)/mu - orbit_pos/o.distance;
                        const double el = glm::length(evec);
                        if(el > 1e-9) {
                            peri_p = (o.periapsis / el) * evec;
                            have_peri = true;
                        }
                    }
                }

                // The focus body (the ship's parent) and the map plane.
                // The plane is a normal in the focus's inertial frame;
                // OrbitMap derives an in-plane basis from it. All three
                // candidates live in that frame:
                //   equatorial = the focus's reference plane (normal +Y);
                //   ecliptic   = the system reference plane (root XZ) expressed
                //                in the focus's frame;
                //   orbital    = the ship's own orbital plane (h = r x v).
                TerrainBody *focus = ship->m_parent;
                glm::dvec3 plane_n(0.0, 1.0, 0.0);
                if(map_plane == 1) {
                    plane_n = glm::transpose(focus->frame->root_orient) *
                              glm::dvec3(0.0, 1.0, 0.0);
                } else if(map_plane == 2) {
                    const glm::dvec3 h = glm::cross(orbit_pos, orbit_vel);
                    const double hl = glm::length(h);
                    if(hl > 1e-9) { plane_n = h / hl; }
                }

                // The map is a kMapSize square at the top of the window; the
                // focus (parent body) sits at its center plus the pan offset;
                // the controls go below. (kMapSize is defined at the top of
                // the block, where the open-trajectory radius cap uses it.)
                const ImVec2 p0 = ImGui::GetCursorScreenPos();
                const float center_x = p0.x + kMapSize * 0.5f;
                const float center_y = p0.y + kMapSize * 0.5f;

                // Reserve the map square with an invisible button. It captures
                // the mouse, so a left-drag over the map pans the map instead
                // of moving the window (imgui otherwise treats a drag on the
                // window background as a window move). Wheel-zoom and drag-pan
                // both apply only while the mouse is over this square.
                ImGui::InvisibleButton("##mapnav", ImVec2(kMapSize, kMapSize));
                const bool over_map = ImGui::IsItemHovered();
                const ImGuiIO &g_io = ImGui::GetIO();
                if(over_map && g_io.MouseWheel != 0.0f) {
                    // Wheel zooms to the cursor (the world point under the
                    // mouse stays put). Reversed per preference: wheel UP zooms
                    // IN (scale = meters/pixel goes down), wheel OUT zooms out.
                    const float factor = (g_io.MouseWheel > 0.0f) ? 0.8f : 1.25f;
                    const float old_scale = map_scale;
                    float new_scale = old_scale * factor;
                    // Clamp to the same range the Scale slider spans (10^3..10^9.5).
                    const float min_scale = 1000.0f;
                    const float max_scale = powf(10.0f, 9.5f);
                    if(new_scale < min_scale) { new_scale = min_scale; }
                    if(new_scale > max_scale) { new_scale = max_scale; }
                    const ImVec2 mouse = ImGui::GetMousePos();
                    const float u = mouse.x - (center_x + map_pan.x);
                    const float v = mouse.y - (center_y + map_pan.y);
                    map_pan.x = (mouse.x - u * old_scale / new_scale) - center_x;
                    map_pan.y = (mouse.y - v * old_scale / new_scale) - center_y;
                    map_scale = new_scale;
                }
                if(ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                    map_pan.x += g_io.MouseDelta.x;
                    map_pan.y += g_io.MouseDelta.y;
                }

                OrbitMap map;
                map.cx = center_x + map_pan.x;
                map.cy = center_y + map_pan.y;
                map.scale = map_scale;
                map.setPlane(plane_n);

                // KSP-inspired palette (P4): your orbit is green, the transfer
                // is blue, other bodies are gray. The focus body, ship dot and
                // labels use a near-black/white ink that contrasts with the
                // current style's window background, so they stay readable in
                // both the light and dark themes. The selected transfer target
                // is highlighted brighter than the other children.
                const ImVec4 bg = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
                const ImU32 ink       = contrastingColor(bg);
                const ImU32 col_ship  = ImGui::GetColorU32(ImVec4(0.20f, 0.80f, 0.40f, 1.0f));
                const ImU32 col_apsis = col_ship;  // periapsis / apoapsis: part of your orbit
                const ImU32 col_xfer  = ImGui::GetColorU32(ImVec4(0.35f, 0.55f, 1.00f, 1.0f));
                const ImU32 col_vessel = ImGui::GetColorU32(ImVec4(1.00f, 0.62f, 0.22f, 1.0f));
                const ImU32 col_child = ImGui::GetColorU32(ImVec4(0.55f, 0.55f, 0.55f, 1.0f));
                const ImU32 col_body  = ink;
                const ImU32 col_sel   = ImGui::GetColorU32(ImVec4(0.90f, 0.90f, 0.90f, 1.0f));
                const ImU32 soi_col   = ImGui::GetColorU32(ImVec4(0.50f, 0.50f, 0.50f, 0.30f));
                ImDrawList *dl = ImGui::GetWindowDrawList();
                const ImVec2 focus_px = map.px(glm::dvec3(0.0, 0.0, 0.0));

                // The body selected in the TRANSFER window (a child of the
                // focus), highlighted on the map; nullptr for a ship target or
                // no selection.
                TerrainBody *sel_body = nullptr;
                if(xfer_target >= 0 && xfer_target < (int)xferTargets.size() &&
                   xferTargets[xfer_target].body) {
                    sel_body = xferTargets[xfer_target].body;
                }

                // A body's sphere-of-influence ring, faint. Skipped when
                // sub-pixel or far off-view (a huge circle is both useless and
                // expensive to tessellate).
                auto draw_soi = [&](const glm::dvec3 &center, double soi_m) {
                    if(!map_show_soi || soi_m <= 0.0) { return; }
                    const float r_px = (float)(soi_m / map_scale);
                    if(r_px < 1.0f || r_px > 4000.0f) { return; }
                    map.drawRing(dl, center, soi_m, soi_col, 1.0f);
                };

                // The orbits of the bodies orbiting the focus (the ship's
                // parent) -- moons around a planet, planets around the star --
                // in the same view/scale/plane. Each child's state is taken in
                // the focus's inertial frame; its ellipse is sampled through
                // the same cache as the ship's orbit (a child never burns, so
                // its points are propagated once). Drawn first, so the ship's
                // orbit sits on top.
                for(auto *b : planets) {
                    if(!(b->frame && b->frame->parent == focus->frame)) continue;
                    const double mu_c = b->frame->parent_mu;
                    if(mu_c <= 0.0) continue;
                    const glm::dvec3 cpos = b->frame->GetPositionRelTo(focus->frame);
                    const glm::dvec3 cvel = b->frame->GetVelocityRelTo(focus->frame);
                    const std::vector<glm::dvec3> &cpts =
                        orbit_caches[(const void *)b].sample(cpos, cvel, mu_c, N);
                    if(cpts.empty()) continue;
                    const bool selected = (b == sel_body);
                    const ImU32 ccol = selected ? col_sel : col_child;
                    map.drawOrbit(dl, cpts, ccol, selected ? 2.0f : 1.0f);
                    const ImVec2 cpx = map.px(cpos);
                    dl->AddCircleFilled(cpx, selected ? 5.0f : 3.0f, ccol);
                    if(selected) { dl->AddCircle(cpx, 8.0f, ccol, 0, 1.0f); }
                    dl->AddText(ImVec2(cpx.x + 4.0f, cpx.y - 12.0f), ink,
                                b->name.c_str());
                    draw_soi(cpos, b->frame->soi);
                }
                // The focus body's own SOI -- the boundary of the current
                // gravitational regime the ship is inside.
                draw_soi(glm::dvec3(0.0, 0.0, 0.0), focus->frame->soi);

                // closed=true for the ellipse (it is a closed loop); false for
                // the open arc (a chord would otherwise close it).
                map.drawOrbit(dl, traj_pts, col_ship, 1.0f, closed);
                map.drawBody(dl, ship->m_parent->radius, col_body);
                // The ship: a bright dot (you are here) with a green ring, on
                // the line from the focus.
                const ImVec2 ship_px = map.px(orbit_pos);
                dl->AddLine(focus_px, ship_px, ink, 1.0f);
                dl->AddCircleFilled(ship_px, 5.0f, ink);
                dl->AddCircle(ship_px, 8.0f, col_ship, 0, 1.5f);
                // Prograde (velocity) arrow, along the ship's velocity.
                if(map_show_vel) {
                    map.drawArrow(dl, orbit_pos, orbit_vel, 24.0f, col_ship, 1.5f);
                }
                // Apside markers are only meaningful for a non-circular orbit;
                // an open arc has periapsis but no apoapsis.
                if(o.ecc > 1e-3) {
                    if(have_peri) { map.drawDot(dl, peri_p, 4.0f, col_apsis); }
                    if(have_apo)  { map.drawDot(dl, apo_p,  4.0f, col_apsis); }
                }

                // Every other ship in this body: its orbit (when closed) plus
                // a dot + label at its current position, so the whole traffic
                // pattern shows, not just you and the target. The player's own
                // ship is already drawn above in green (skipped here). Ships
                // on an escape trajectory (ecc >= 1) have no closed orbit to
                // draw -- sample() returns empty -- so only their position
                // marker shows.
                {
                    Frame *inertial = ship->frame->getNonRotFrame();
                    for(auto *s : ships) {
                        if(s == ship || !s->frame || s->frame->body != focus) {
                            continue;
                        }
                        Frame *tsf = s->frame;
                        const glm::dvec3 tcom = s->get_center_of_mass();
                        const glm::dmat3 O = tsf->GetOrientRelTo(inertial);
                        const glm::dvec3 r2 = O * tcom + tsf->GetPositionRelTo(inertial);
                        const glm::dvec3 v2 = O * (s->GetVel()
                                                  + tsf->GetStasisVelocity(tcom))
                                            + tsf->GetVelocityRelTo(inertial);
                        const std::vector<glm::dvec3> &tpts =
                            orbit_caches[(const void *)s].sample(
                                r2, v2, mu, N, s->onRails);
                        if(!tpts.empty()) {
                            map.drawOrbit(dl, tpts, col_vessel, 1.0f);
                        }
                        const ImVec2 tpx = map.px(r2);
                        dl->AddCircleFilled(tpx, 3.0f, col_vessel);
                        dl->AddText(ImVec2(tpx.x + 4.0f, tpx.y - 11.0f),
                                    col_vessel, s->name.c_str());
                    }
                }

                // P3: the transfer conic to the selected target (planner has a
                // valid solution). It is a Kepler orbit under the focus's mu,
                // starting at the ship (r1 = orbit_pos) with velocity
                // sol.v_departure and propagated over sol.tof -- the same
                // frame as the rest of the map, so it projects through the
                // same plane. The arc's end is the arrival / intercept point
                // (where the ship meets the target at t + tof); the departure
                // point is the ship dot already drawn above.
                if(xfer.valid) {
                    const TransferSolution &sol = xfer.sol;
                    std::vector<glm::dvec3> xfer_pts;
                    xfer_pts.reserve(N + 1);
                    for(int i = 0; i <= N; i++) {
                        glm::dvec3 p, v;
                        propagateKepler(orbit_pos, sol.v_departure, mu,
                                        sol.tof * i / N, p, v);
                        xfer_pts.push_back(p);
                    }
                    map.drawOrbit(dl, xfer_pts, col_xfer, 1.5f, /*closed=*/false);
                    const glm::dvec3 &arrival = xfer_pts.back();
                    map.drawDot(dl, arrival, 4.0f, col_xfer);
                    char xfer_label[96];
                    snprintf(xfer_label, sizeof(xfer_label), "%s  %.0f m/s",
                             xferTargets[xfer_target].name, sol.total_dv);
                    const ImVec2 apx = map.px(arrival);
                    dl->AddText(ImVec2(apx.x + 5.0f, apx.y + 4.0f), col_xfer,
                                xfer_label);
                }

                // (The invisible map-nav button above already reserved the map
                // square, so the controls land below it.)
                // Bare-map (mode 1) and chrome-less (mode 2) draws only the
                // map -- the controls below are skipped in both.
                if(map_mode != 0) {
                    return;
                }
                // Which plane to project onto (see the plane_n selection above).
                // "Orbital" aligns the view with the ship's orbit, so a polar
                // orbit reads as a full ellipse instead of collapsing to a line.
                static const char *kPlanes[] = { "Equatorial", "Ecliptic", "Orbital" };
                ImGui::Combo("Map plane", &map_plane, kPlanes, 3);
                // The map spans ~8 orders of magnitude (a ~70 km low orbit up
                // to a ~90,000 Mm interplanetary orbit), so the scale is edited
                // on a log10 axis -- a linear slider couldn't reach the moons.
                // Wheel-zoom / drag-pan (above) edit the same values; this is
                // the coarse control, and "Reset view" restores the default.
                {
                    float log_scale = log10f(map_scale);
                    if(ImGui::SliderFloat("Scale", &log_scale, 3.0f, 9.5f, "%.1f")) {
                        map_scale = powf(10.0f, log_scale);
                    }
                    ImGui::SameLine();
                    ImGui::Text("%.0f m/px", (double)map_scale);
                    ImGui::SameLine();
                    if(ImGui::Button("Reset view")) {
                        map_pan = ImVec2(0.0f, 0.0f);
                        map_scale = 6000.0f;
                    }
                }
                ImGui::Checkbox("SOI rings", &map_show_soi);
                ImGui::SameLine();
                ImGui::Checkbox("Velocity", &map_show_vel);
                ImGui::Text("nu %.2f   E %.2f   inc %.2f deg",
                            o.true_anomaly, o.ecc_anomaly,
                            o.inclination * 180.0 / M_PI);
                // Legend: a compact color key (one line).
                ImGui::Spacing();
                auto legend = [&](const char *label, ImU32 col, bool dot) {
                    const ImVec2 p = ImGui::GetCursorScreenPos();
                    const float s = 10.0f;
                    if(dot) {
                        dl->AddCircleFilled(ImVec2(p.x + 5.0f, p.y + 8.0f), 4.0f, col);
                    } else {
                        dl->AddRectFilled(ImVec2(p.x, p.y + 3.0f),
                                         ImVec2(p.x + s, p.y + 13.0f), col);
                    }
                    ImGui::Dummy(ImVec2(s, 16.0f));
                    ImGui::SameLine();
                    ImGui::TextUnformatted(label);
                };
                legend("your orbit", col_ship, false);
                ImGui::SameLine();
                legend("other ships", col_vessel, false);
                ImGui::SameLine();
                legend("transfer", col_xfer, false);
                ImGui::SameLine();
                legend("other bodies", col_child, false);
                ImGui::SameLine();
                legend("apsides", col_apsis, true);
            });

            // The main menu (gameui.cpp): drawn last so it sits on top.
            drawMainMenu(game);

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

            display.SwapBuffers();
            check_gl_error();
        }

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

    ships.clear();   // ships + space pads (BEFORE the System bodies/shaders they reference)

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
