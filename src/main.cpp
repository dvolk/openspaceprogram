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

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include "cli.h"

#include "../middleware/imgui/imgui.h"
#include "../middleware/imgui/backends/imgui_impl_sdl2.h"
#include "../middleware/imgui/backends/imgui_impl_opengl3.h"
#include "../middleware/implot/implot.h"


ImFont *bigger;

/* ResourceType / ResourceContent / PartDef live in shipdef.h (the GL-free
   ship/part data model), shared with the JSON loaders and the headless
   tests. */

glm::dvec3 projectVecOntoPlane(const glm::dvec3 & vec, const glm::dvec3 & normal) {
    return vec - glm::dot(vec, normal) * normal;
}

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
    bigger = io.Fonts->AddFontFromFileTTF(args.font_path.c_str(), 2.0f * args.font_size);
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

    // The runtime state lives in `game`. These local references keep the
    // loop body reading exactly as before; they alias game's members, so
    // the writes here and the control transitions in game.cpp hit the same
    // storage. (redraw / screenshot_count / dt / currentTime / accumulator
    // are pure loop bookkeeping and stay local.)
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

    bool redraw = false;
    int screenshot_count = 0;
    SDL_SetRelativeMouseMode(SDL_FALSE);

    const double dt = 1.0/50.0; // TODO explain why 50

    // kRailsWarp is defined in game.h (the rails-warp threshold).
    double currentTime = 0.001 * (double)(SDL_GetTicks());
    double accumulator = 0.0;
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
    bool physics_debug_drawing = false;
    bool world_drawing = true;
    bool draw_starfield = true;
    bool draw_skylines = false;
    // 0=dark 1=light 2=classic (classic = imgui's default palette)
    int ui_style = 2;
    float window_rounding = 0.0f; // imgui default

    /* UI windows (src/ui.h): one options block per window, plus a table the
       main-menu checkboxes, the F10 toggle and a UI reset all share. The
       main menu itself and the HUD trio (one "Top HUD" checkbox) are
       handled separately. Slots/offsets: left column stacks under the
       menu, right column under ORBITAL, the rest spread over the edges so
       the center stays clear for the 3D view. */
    auto info_opts = [](ui::Slot slot) {
        ui::Options o;
        o.slot = slot;
        return o;
    };
    // Layout: top left ORBITAL + SURFACE, top right RESOURCES,
    // middle right the menu, bottom right VESSEL, bottom left Orbital map.
    ui::Options o_orbit     = info_opts(ui::Slot::TopLeft);
    ui::Options o_surface   = info_opts(ui::Slot::TopLeft);
    o_surface.right_of = "ORBITAL";
    ui::Options o_resources = info_opts(ui::Slot::TopRight);
    o_resources.width_ratio = 1.5f; // bars have no width of their own
    ui::Options o_menu      = info_opts(ui::Slot::MiddleRight);
    o_menu.closable = false;
    ui::Options o_vessel    = info_opts(ui::Slot::BottomRight);
    ui::Options o_parts     = info_opts(ui::Slot::BottomRight);
    o_parts.below = "VESSEL";
    o_parts.default_open = false;
    ui::Options o_map       = info_opts(ui::Slot::BottomLeft);
    o_map.initial_size = ImVec2(480.0f, 480.0f); // orbit drawn at (200,200)
    // The rest stay out of the way of the above (all closed by default).
    ui::Options o_ships     = info_opts(ui::Slot::TopCenter);
    ui::Options o_autopilot = info_opts(ui::Slot::Center);
    o_autopilot.default_open = false;
    ui::Options o_controls  = info_opts(ui::Slot::BottomCenter);
    o_controls.default_open = false;
    ui::Options o_debug     = info_opts(ui::Slot::TopCenter);
    o_debug.default_open = false;
    ui::Options o_telemetry = info_opts(ui::Slot::MiddleLeft);
    o_telemetry.default_open = false;
    o_telemetry.initial_size = ImVec2(460.0f, 680.0f);
    ui::Options o_settings = info_opts(ui::Slot::BottomCenter);
    o_settings.default_open = false;
    // Transfer planner: target selection + dv readouts.
    ui::Options o_transfer = info_opts(ui::Slot::Center);
    o_transfer.default_open = false;
    game.o_hud.fixed = true;
    game.o_hud.closable = false;
    game.o_hud.default_open = true;
    game.o_hud.flags |= ImGuiWindowFlags_NoTitleBar;
    game.o_hud.slot = ui::Slot::TopCenter;
    ui::Options o_mainmenu = info_opts(ui::Slot::Center);
    o_mainmenu.fixed = true;
    o_mainmenu.closable = false;
    o_mainmenu.default_open = false;

    // The registry lives on the game (game.ui_windows): the TAB toggle and
    // the main-menu "Toggle windows" read it there.
    auto add_ui_window = [&](const char *name, const char *label,
                             const ui::Options &o) {
        game.ui_windows.push_back(Game::UiWin{name, label, o});
    };
    add_ui_window("RESOURCES", "Resources", o_resources);
    add_ui_window("ORBITAL", "Orbit Info", o_orbit);
    add_ui_window("Orbital map", "Orbit Map", o_map);
    add_ui_window("SURFACE", "Surface Info", o_surface);
    add_ui_window("VESSEL", "Vessel Info", o_vessel);
    add_ui_window("SHIP PARTS", "Vessel Parts", o_parts);
    if(ships.size() > 1) {
        add_ui_window("SHIPS", "Ship List", o_ships);
    }
    add_ui_window("Autopilot", "DUMB-ASS", o_autopilot);
    // Controls and Settings are main-menu only, so they're deliberately
    // NOT in this list (and thus not affected by the TAB toggle).
    add_ui_window("Game Debug Info", "Game Debug Info", o_debug);
    add_ui_window("TELEMETRY", "Telemetry", o_telemetry);
    add_ui_window("TRANSFER", "Transfer", o_transfer);
    // The TAB toggle + the main-menu "Toggle windows" button call
    // game.toggle_windows() (game.cpp), which flips game.ui_visible and
    // re-opens every registry window (plus the HUD) from their defaults.

    double time = 0;

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

    // Transfer planner (TRANSFER window + the blue burn-direction icon).
    // Targets: child bodies of the ship's current body (parent->child
    // transfers, with a capture burn) + other ships in the same body
    // (intercept only). The list is rebuilt every frame; the solution is
    // recomputed only on input change or every 30 frames (the plan changes
    // slowly relative to the ToF scale, and the sweep is the cost center).
    struct XferTarget {
        const char *name;
        TerrainBody *body;   // body target (capture available)
        Vehicle *ship;       // ship target (intercept only)
    };
    std::vector<XferTarget> xferTargets;
    int xfer_target = -1;
    bool xfer_auto = true;                    // auto min-dv ToF vs pinned
    float xfer_tof_log = (float)std::log10(3600.0); // log10(s), the pinned ToF
    struct {
        int target = -2;        // target index at last compute
        bool auto_tof = true;
        double tof_log = -1.0;
        int frame = 0;          // per-frame counter while a target is set
        int solved_frame = -1000000; // xfer.frame at last recompute
        bool valid = false;
        TransferSolution sol;
        glm::dvec3 burn_dir = glm::dvec3(0.0); // render-frame burn direction
    } xfer;

    // Telemetry plots (active ship, sampled once per rendered frame).
    TimeSeries energy_series;
    TimeSeries angmom_series;

    // --orbit-log: print at most once per wall-clock interval
    const Uint32 orbit_log_interval_ms = (Uint32)(args.orbit_interval * 1000.0);
    Uint32 orbit_log_last_ms = 0;
    /* Separate timestamp: the two logs share --orbit-interval but must not
       share the "last fired" time, or the earlier block in the loop always
       wins and the other never fires (and one alone spews every tick). */
    Uint32 dbg_log_last_ms = 0;
    Uint32 xfer_log_last_ms = 0;

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
    const Uint32 loop_start_ms = SDL_GetTicks();
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
            const double elapsed_s = (SDL_GetTicks() - loop_start_ms) * 0.001;
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
        /* --sim-press: emit the synthetic key events that fell due this
           frame, in down-then-up order per press. They are polled below in
           the same frame, so one-shot actions fire in the frame the press
           is due. */
        if(!args.sim_presses.empty()) {
            const Uint32 now = SDL_GetTicks() - loop_start_ms;
            auto push_key = [&](SDL_EventType type, const SimKeyPress &p) {
                SDL_Event kev = {0};
                kev.type = type;
                kev.key.windowID = sim_win_id;
                kev.key.state = (type == SDL_KEYDOWN) ? SDL_PRESSED : SDL_RELEASED;
                kev.key.repeat = 0;
                kev.key.keysym.sym = p.key;
                kev.key.keysym.scancode = p.sc;
                SDL_PushEvent(&kev);
            };
            for(auto &p : args.sim_presses) {
                if(!p.down_sent && now >= p.down_ms) {
                    push_key(SDL_KEYDOWN, p);
                    p.down_sent = true;
                }
                if(p.down_sent && !p.up_sent && now >= p.up_ms) {
                    push_key(SDL_KEYUP, p);
                    p.up_sent = true;
                }
            }
        }

        /* --sim-mouse: emit the synthetic mouse events that fell due this
           frame, in the order each gesture needs. A drag (button + held)
           presses the button BEFORE moving so the camera-look handler
           (gated on rmbCam) sees the button down first; a click moves the
           cursor into place then presses + releases in place; BTN==0 just
           repositions. The motion carries the delta from the previous
           simulated position (args.sim_mouse_x/y), which the camera consumes. */
        if(!args.sim_mouse_actions.empty()) {
            const Uint32 now = SDL_GetTicks() - loop_start_ms;
            auto push_motion = [&](int x, int y) {
                SDL_Event mev = {0};
                mev.type = SDL_MOUSEMOTION;
                mev.motion.windowID = sim_win_id;
                mev.motion.which = 0;
                mev.motion.x = x;
                mev.motion.y = y;
                mev.motion.xrel = x - args.sim_mouse_x;
                mev.motion.yrel = y - args.sim_mouse_y;
                mev.motion.state = 0;
                SDL_PushEvent(&mev);
                args.sim_mouse_x = x;
                args.sim_mouse_y = y;
            };
            auto push_btn = [&](SDL_EventType type, int button, int x, int y) {
                SDL_Event bev = {0};
                bev.type = type;
                bev.button.windowID = sim_win_id;
                bev.button.which = 0;
                bev.button.button = (Uint8)button;
                bev.button.state = (type == SDL_MOUSEBUTTONDOWN) ? SDL_PRESSED
                                                                 : SDL_RELEASED;
                bev.button.x = x;
                bev.button.y = y;
                SDL_PushEvent(&bev);
            };
            for(auto &a : args.sim_mouse_actions) {
                if(!a.started && now >= a.time_ms) {
                    if(a.button != 0 && a.up_ms > a.time_ms) {
                        // drag: press, then move (release comes at up_ms)
                        push_btn(SDL_MOUSEBUTTONDOWN, a.button, a.x, a.y);
                        push_motion(a.x, a.y);
                    } else if(a.button != 0) {
                        // click: move into place, press, release (same frame)
                        push_motion(a.x, a.y);
                        push_btn(SDL_MOUSEBUTTONDOWN, a.button, a.x, a.y);
                        push_btn(SDL_MOUSEBUTTONUP, a.button, a.x, a.y);
                        a.released = true;
                    } else {
                        // move only (no button)
                        push_motion(a.x, a.y);
                    }
                    a.started = true;
                }
                // release a held button at up_ms
                if(a.button != 0 && a.started && !a.released
                   && now >= a.up_ms) {
                    push_btn(SDL_MOUSEBUTTONUP, a.button, a.x, a.y);
                    a.released = true;
                }
            }
        }

        SDL_Event ev;

        while (SDL_PollEvent(&ev)) {
            ImGui_ImplSDL2_ProcessEvent(&ev);
            if (ev.type == SDL_QUIT) {
                running = false;
            }

            if (ev.type == SDL_WINDOWEVENT) {
                if(ev.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
                    display.onResize(ev.window.data1, ev.window.data2);
                    check_gl_error();

                    postfx->Resize(ev.window.data1, ev.window.data2);
                    check_gl_error();

                    camera->setAspect((float)ev.window.data1 / (float)ev.window.data2);
                    check_gl_error();
                }
            }
            if(ev.type == SDL_KEYDOWN) {
                if(ev.key.keysym.sym == SDLK_PERIOD) {
                    if(time_accel < 1000) {
                        time_accel *= 10;
                        if(time_accel == 0) {
                            time_accel = 1;
                        }
                    } else if(time_accel < 100000) {
                        // 1000 -> 10000 -> 100000: rails warp. Every ship
                        // coasts (or freezes on the ground) and the physics
                        // world stops stepping; refuses if any ship is not
                        // rail-eligible.
                        if(game.enter_rails_warp()) {
                            time_accel *= 10;
                            printf("Rails warp: time accel %d (ships on rails)\n",
                                   time_accel);
                        }
                    }
                }
                if(ev.key.keysym.sym == SDLK_COMMA) {
                    if(time_accel > 1) {
                        const bool leaving_rails_warp =
                            (time_accel >= kRailsWarp) && (time_accel / 10 < kRailsWarp);
                        time_accel /= 10;
                        if(leaving_rails_warp) {
                            // dropped out of rails warp: the active ship
                            // re-enters physics (idle ships stay parked)
                            ship->leaveRails();
                            printf("Rails warp: exited, time accel %d\n", time_accel);
                        }
                    }
                    else if(time_accel == 1) {
                        time_accel = 0;
                    }
                }
                if(ev.key.keysym.sym == SDLK_l) {
                    if(cam_speed < 10000000) {
                        cam_speed *= 4;
                    }
                }
                if(ev.key.keysym.sym == SDLK_k) {
                    if(cam_speed > 1) {
                        cam_speed /= 4;
                    }
                }
                if(ev.key.keysym.sym == SDLK_c) {
                    // Toggle between the body-orbit camera and the free camera.
                    if(camMode == CAM_ORBIT) {
                        freeCam->pos = orbitCam->pos;
                        freeCam->forward = orbitCam->forward;
                        freeCam->up = orbitCam->up;
                        camMode = CAM_FREE;
                        camera = freeCam;
                    } else {
                        glm::dvec3 focus = game.focusWorldPos(focusBody);
                        orbitCam->Follow(focus);
                        double dist = glm::length(freeCam->pos - focus);
                        if(dist < 10.0) { dist = 10.0; }
                        orbitCam->distance = dist;
                        camMode = CAM_ORBIT;
                        camera = orbitCam;
                        printf("Camera: orbiting %s (G = switch body, C = free)\n",
                               game.focusTargets[focusBody].name);
                    }
                }
                if(ev.key.keysym.sym == SDLK_g) {
                    // Cycle the orbit camera's target body.
                    if(camMode == CAM_ORBIT) {
                        focusBody = (focusBody + 1) % game.numFocusTargets;
                        orbitCam->Follow(game.focusWorldPos(focusBody));
                        double d = (game.focusTargets[focusBody].body == nullptr)
                            ? 50.0
                            : (double)game.focusTargets[focusBody].body->radius * 3.0;
                        orbitCam->distance = d;
                        printf("Orbit camera targeting %s\n",
                               game.focusTargets[focusBody].name);
                    } else {
                        printf("In free flight; press C to go to orbit, then G to switch body.\n");
                    }
                }
                if(ev.key.keysym.sym == SDLK_TAB) {
                    // toggle the info windows (one-shot; auto-repeat would
                    // just keep flipping)
                    if(!ev.key.repeat) {
                        game.toggle_windows();
                    }
                }
                if(ev.key.keysym.sym == SDLK_F6) {
                    // advance to the next ship in the fleet, wrapping around
                    // (one-shot; auto-repeat would keep cycling). No-op with a
                    // single ship: the next index is the current one.
                    if(!ev.key.repeat && ships.size() > 1) {
                        game.select_ship((activeIdx + 1) % (int)ships.size());
                    }
                }
                if(ev.key.keysym.sym == SDLK_SPACE) {
                    // separate the active stage (one-shot; auto-repeat would
                    // keep dropping stages). Only while flying a ship with
                    // time running (a paused separation would leave the
                    // survivors frozen mid-air).
                    if(!ev.key.repeat && camMode == CAM_ORBIT && time_accel > 0) {
                        // staging needs the parts in the physics world:
                        // wake a ship parked on rails first
                        if(ship->onRails) {
                            ship->leaveRails();
                            if(time_accel >= kRailsWarp) { time_accel = 1; }
                        }
                        int dropped = ship->separateStage(ship->activeStage());
                        if(dropped > 0) {
                            printf("Stage: dropped %d part(s); now on stage %d of %d\n",
                                   dropped, ship->activeStage(), ship->numStages());
                        } else {
                            printf("Stage: nothing left to separate\n");
                        }
                    }
                }
                if(ev.key.keysym.sym == SDLK_F12) {
                    screenshot_requested = true;
                }
                if(ev.key.keysym.sym == SDLK_F11) {
                    if(poly_mode == false) {
                        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                        poly_mode = true;
                    } else {
                        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                        poly_mode = false;
                    }
                }
                if(ev.key.keysym.sym == SDLK_F10) {
                    // Reset the window layout to defaults (same as the
                    // main menu's "Reset windows" button).
                    ui::ResetGui();
                }
                if(ev.key.keysym.sym == SDLK_ESCAPE) {
                    // Toggle the main menu.
                    ui::SetOpen("Main Menu", !ui::IsOpen("Main Menu"));
                }
            }
            if(ev.type == SDL_MOUSEBUTTONDOWN) {
                // holding RMB over 3D (not over a UI window) moves the camera.
                if(ev.button.button == SDL_BUTTON_RIGHT &&
                   !ImGui::GetIO().WantCaptureMouse) {
                    rmbCam = true;
                }
            }
            if(ev.type == SDL_MOUSEBUTTONUP) {
                if(ev.button.button == SDL_BUTTON_RIGHT) {
                    rmbCam = false;
                }
            }
            if(ev.type == SDL_MOUSEMOTION) {
                if(rmbCam && !ImGui::GetIO().WantCaptureMouse) {
                    camera->RotateY(-ev.motion.xrel / 200.0f);
                    camera->Pitch(ev.motion.yrel / 200.0f);
                }
            }
            if(ev.type == SDL_MOUSEWHEEL) {
                // Zoom when the wheel is not scrolling a UI window.
                if(!ImGui::GetIO().WantCaptureMouse) {
                    camera->wheel(ev.wheel.y);
                }
            }
        }

        /*
          LOGIC
        */
        double newTime = (double)(SDL_GetTicks()) * 0.001;
        double frameTime = newTime - currentTime;
        currentTime = newTime;
        accumulator += frameTime;

        glm::dvec3 com;

        if(accumulator > 10 * dt) {
            accumulator = 10 * dt;
        }

        // clear stats and stuff
        for(auto *s : ships) { s->m_thrust = 0.0; }

        while (accumulator >= dt) {
            // is this logic? ;_;
            // Thrust and rotation are armed once per tick (if the keys are
            // held, below) and then re-applied before every substep; clear
            // them first so a tick without the keys doesn't keep pushing or
            // slewing from the last one.
            ship->clearThrust();
            ship->clearRotCmd();

            const Uint8* key = SDL_GetKeyboardState(NULL);
            /* --sim-press: a synthetic key is "down" from its down time to
               its up time. SDL_PushEvent does not update the state array
               above (verified on this SDL), so held commands OR in each
               entry's window instead. */
            auto isDown = [&](SDL_Scancode sc) -> bool {
                if(key[sc]) { return true; }
                for(size_t i = 0; i < args.sim_presses.size(); i++) {
                    if(args.sim_presses[i].sc == sc && args.sim_presses[i].down_sent
                       && !args.sim_presses[i].up_sent) {
                        return true;
                    }
                }
                return false;
            };

            if (camMode == CAM_FREE) {
                if (isDown(SDL_SCANCODE_W)) { camera->MoveForward(cam_speed); }
                else if (isDown(SDL_SCANCODE_S)) { camera->MoveForward(-cam_speed); }

                if (isDown(SDL_SCANCODE_A)) { camera->MoveRight(-cam_speed); }
                else if (isDown(SDL_SCANCODE_D)) { camera->MoveRight(cam_speed); }

                if (isDown(SDL_SCANCODE_E)) { camera->Roll(-0.05); }
                else if (isDown(SDL_SCANCODE_Q)) { camera->Roll(0.05); }

                if (isDown(SDL_SCANCODE_LSHIFT) || isDown(SDL_SCANCODE_RSHIFT)) { camera->MoveUp(cam_speed); }
                else if (isDown(SDL_SCANCODE_LCTRL) || isDown(SDL_SCANCODE_RCTRL)) { camera->MoveUp(-cam_speed); }
            }

            if (camMode == CAM_ORBIT) {
                bool game_running = (time_accel > 0);
                /* touching the controls wakes a ship parked on rails: it
                   re-enters physics and rails warp drops to 1x (you cannot
                   maneuver on rails). */
                if(ship->onRails && time_accel >= kRailsWarp) {
                    if(isDown(SDL_SCANCODE_W) || isDown(SDL_SCANCODE_S) ||
                       isDown(SDL_SCANCODE_A) || isDown(SDL_SCANCODE_D) ||
                       isDown(SDL_SCANCODE_Q) || isDown(SDL_SCANCODE_E) ||
                       isDown(SDL_SCANCODE_I) || isDown(SDL_SCANCODE_X) ||
                       isDown(SDL_SCANCODE_B) || isDown(SDL_SCANCODE_N) ||
                       isDown(SDL_SCANCODE_R) || isDown(SDL_SCANCODE_F)) {
                        ship->leaveRails();
                        time_accel = 1;
                        printf("Control input: '%s' left the rails, warp -> 1\n",
                               ship->name.c_str());
                    }
                }
                // pitch
                if (isDown(SDL_SCANCODE_W)) { ship->Command(ShipCmd(Pitch, +1.0f), game_running); }
                if (isDown(SDL_SCANCODE_S)) { ship->Command(ShipCmd(Pitch, -1.0f), game_running); }
                // yaw
                if (isDown(SDL_SCANCODE_A)) { ship->Command(ShipCmd(Yaw, +1.0f), game_running); }
                if (isDown(SDL_SCANCODE_D)) { ship->Command(ShipCmd(Yaw, -1.0f), game_running); }
                // roll
                if (isDown(SDL_SCANCODE_Q)) { ship->Command(ShipCmd(Roll, +1.0f), game_running); }
                if (isDown(SDL_SCANCODE_E)) { ship->Command(ShipCmd(Roll, -1.0f), game_running); }

                if (isDown(SDL_SCANCODE_I)) { ship->Command(ShipCmd(Thrust), game_running, dt * time_accel); }
                if (isDown(SDL_SCANCODE_X)) { ship->Command(ShipCmd(KillRot), game_running); }

                if (isDown(SDL_SCANCODE_B)) { ship->Command(ShipCmd(Prograde), game_running); }
                if (isDown(SDL_SCANCODE_N)) { ship->Command(ShipCmd(Retrograde), game_running); }

                if (isDown(SDL_SCANCODE_R)) { ship->Command(ShipCmd(ThrottleUp), game_running); }
                if (isDown(SDL_SCANCODE_F)) { ship->Command(ShipCmd(ThrottleDown), game_running); }
            }

            void physics_tick(float timeStep);

            // Advance the analytic sim clock by exactly the physics timestep
            // (dt * time_accel), matching physics_tick(dt * time_accel) below.
            // The frame tree's analytic motion must run on the same clock as
            // the ship's integration. The old 1/60.0 constant disagreed with
            // dt (1/50), so the physics clock ran 20% faster than the analytic
            // body positions and the ship systematically outran the planets.
            time += dt * time_accel;

            if(time_accel != 0) {
                sun->frame->UpdateOrbitRails(time);

                if(time_accel >= kRailsWarp) {
                    /* Rails warp: every ship coasts analytically (or sits
                       frozen on the ground) and the Bullet world is not
                       stepped at all -- O(ships) per tick instead of a
                       substep count that explodes with the accel. */
                    for(auto *s : ships) { s->railsTick(dt * time_accel); }
                } else {

                // per-ship SOI bookkeeping: each ship tracks its own
                // position in the shared frame tree (an idle ship can
                // cross a boundary while we fly another one). Railed
                // ships advance their analytic conic here instead --
                // exact for any step size, at any time accel.
                for(auto *s : ships) {
                    if(s->onRails) { s->railsTick(dt * time_accel); }
                    else { s->switchFrames(); }
                }

                // Integrate the (time-accelerated) step in substeps,
                // re-applying gravity + the rotating-frame fictitious forces
                // + the engine thrust + the armed rotation commands before
                // EACH substep. Two reasons:
                //  1. Bullet clears accumulated forces at the end of every
                //     stepSimulation call, so applying gravity once and then
                //     stepping multiple substeps would leave the ship
                //     force-free for all but the first substep.
                //  2. Re-applying per substep keeps the central-force
                //     direction and the velocity-dependent Coriolis term
                //     accurate across the step instead of frozen at the
                //     step's start.
                // Keep >=3 substeps so low-accel behavior matches the old
                // 3-substep step, and grow the count so the substep stays
                // <= kMaxSubStep at high time-accel.
                const double step = dt * time_accel;
                const double kMaxSubStep = 0.1;
                int n = 3;
                int need = (int)(step / kMaxSubStep + 0.5);
                if (need > n) { n = need; }
                if (n > 2000) { n = 2000; }
                const double h = step / n;
                for (int i = 0; i < n; i++) {
                    // every NON-RAILED ship feels its own gravity/thrust/
                    // rotation each substep; physics_tick then steps the
                    // shared Bullet world all of them at once. Railed ships
                    // have no bodies in the world -- their conic already
                    // advanced this tick in railsTick.
                    for(auto *s : ships) {
                        if(s->onRails) { continue; }
                        s->processGravity();
                        s->applyThrustForce();
                        s->applyRotationForce(h);
                    }
                    physics_tick(h);
                }

                } // end physics-warp branch (time_accel < kRailsWarp)

                /* --spin-log (or --radial-test): spin diagnostics, once per
                   0.5 s of sim time (after the last substep's solve, so the
                   reported impulses are that solve's). */
                if(args.spin_log_enabled || !args.radial_test.empty()) {
                    static double last_spin_log = -1e30;
                    if(time - last_spin_log >= 0.5) {
                        last_spin_log = time;
                        spin_log(ship, time);
                    }
                }
            }

            // --orbit-log: orbital elements, fit in the body's inertial
            // frame, where the ship's trajectory is a Kepler conic.
            if(args.orbit_log) {
                const Uint32 now_ms = SDL_GetTicks();
                if(now_ms - orbit_log_last_ms >= orbit_log_interval_ms) {
                    orbit_log_last_ms = now_ms;

                    const double mu = ship->m_parent->mu;
                    glm::dvec3 o_pos = ship->get_center_of_mass();
                    glm::dvec3 o_vel = ship->GetVel();
                    if(ship->frame->isRotFrame()) {
                        Frame *inertial = ship->frame->getNonRotFrame();
                        o_vel += ship->frame->GetStasisVelocity(o_pos);
                        o_vel = ship->frame->GetOrientRelTo(inertial) * o_vel
                              + ship->frame->GetVelocityRelTo(inertial);
                        o_pos = ship->frame->GetOrientRelTo(inertial) * o_pos
                              + ship->frame->GetPositionRelTo(inertial);
                    }
                    OrbitElements o = computeOrbitElements(o_pos, o_vel, mu);
                    printf("[orbitlog] t=%.1fs frame=\"%s\" r=%.6g m v=%.6g m/s "
                           "sma=%.6g m ecc=%.6g peri=%.6g m apo=%.6g m "
                           "inc=%.4f deg T=%.6g s ttAp=%.6g s ttPe=%.6g s "
                           "|h|=%.6f m2/s E=%.6f J/kg\n",
                           time, ship->frame->name.c_str(), o.distance, o.speed,
                           o.semi_major, o.ecc, o.periapsis, o.apoapsis,
                           glm::degrees(o.inclination), o.period,
                           o.time_to_apo, o.time_to_peri,
                           o.ang_momentum, o.energy);
                    fflush(stdout);
                }
            }

            // --dbg-log: ship pos/alt/vel in its own frame
            if(args.dbg_log) {
                const Uint32 now_ms = SDL_GetTicks();
                if(now_ms - dbg_log_last_ms >= orbit_log_interval_ms) {
                    dbg_log_last_ms = now_ms;
                    glm::dvec3 p = ship->get_center_of_mass();
                    glm::dvec3 v = ship->GetVel();
                    double r = glm::length(p);
                    double alt = r - ship->m_parent->GetTerrainHeight(glm::normalize(p));
                    printf("[dbg] t=%.1fs pos=[%.1f %.1f %.1f] alt=%.1f m "
                           "vel=[%.2f %.2f %.2f] |v|=%.2f m/s\n",
                           time, p.x, p.y, p.z, alt, v.x, v.y, v.z,
                           glm::length(v));
                    fflush(stdout);
                }
            }

            accumulator -= dt;
        }

        redraw = true;

        /*
          RENDERING
        */
        if(redraw == true) {
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

            com = ship->get_center_of_mass();

            if(camMode == CAM_ORBIT) {
                camera->Follow(game.focusWorldPos(focusBody));
            }

            // Render frame origin = the active ship's COM (both are in
            // ship->frame, the render frame). The view is built there and
            // the Draw sites shift geometry by -renderOrigin, so the
            // float32 cast works on ship-relative numbers.
            camera->renderOrigin = com;
            camera->ComputeView();

            /*
              standard 3d stuff drawn here
            */

            if(world_drawing == true) {
                // one per home body; StaticBuilding::Draw culls itself when
                // the active ship is not on that body
                for(auto &kv : ships.pads()) {
                    kv.second->Draw(camera, ship->m_parent, ship->frame);
                }
                // render frame = the active ship's frame; idle ships in a
                // different frame are transformed into it in Vehicle::Draw
                for(auto *s : ships) { s->Draw(camera, ship->frame); }
            }

            for(auto&& planet : planets) {
                if(planet == ship->m_parent) {
                    //this is the planet we're on. This means its position is always 0, 0, 0

                    if(ship->frame->isRotFrame()) {
                        // we're in its rotational frame
                        planet->transform = glm::dmat4(1.0);
                    }
                    else {
                        // we're in its inertial frame
                        planet->transform = glm::dmat4(planet->frame->getRotFrame()->orient);
                    }
                }
                else {
                    // other planets
                    glm::dvec3 translate = planet->frame->GetPositionRelTo(ship->frame);
                    planet->transform = glm::translate(translate) * glm::dmat4(planet->frame->getRotFrame()->orient);
                }
            }

            for(auto&& planet : planets) {
                planet->Update(camera);
                if(world_drawing == true) {
                    planet->Draw(camera, sun, ship->frame);
                }
            }

            /*
              end 3d stuff drawn here
            */

            const double mu = ship->m_parent->mu;

            glm::dvec3 getRelAxis_(Body *body, int n);
            // surf pos??
            const glm::dvec3 pos = com;
            /* orbital velocity */
            glm::dvec3 vel = ship->GetVel();

            // The orbit is a Kepler conic in the body's INERTIAL (non-rotating)
            // frame — that is the frame the spawn/switching code targets and
            // the frame in which the ship's trajectory is a conic.
            glm::dvec3 orbit_pos = pos;
            glm::dvec3 orbit_vel = vel;
            if(ship->frame->isRotFrame() == true) {
                Frame *inertial = ship->frame->getNonRotFrame();
                orbit_vel += ship->frame->GetStasisVelocity(orbit_pos);
                orbit_vel = ship->frame->GetOrientRelTo(inertial) * orbit_vel + ship->frame->GetVelocityRelTo(inertial);
                orbit_pos = ship->frame->GetOrientRelTo(inertial) * orbit_pos + ship->frame->GetPositionRelTo(inertial);
            }

            // Surface-relative state: the ship's position/velocity in the
            // ROTATING frame (i.e. relative to the ground).
            glm::dvec3 surf_pos = pos;
            glm::dvec3 surf_vel = vel;

            if(ship->frame->isRotFrame() == false and
               ship->frame->hasRotFrame() == true) {
                Frame *rot = ship->frame->getRotFrame();
                surf_pos = ship->frame->GetOrientRelTo(rot) * pos;
                surf_vel = ship->frame->GetOrientRelTo(rot) * vel
                         - rot->GetStasisVelocity(surf_pos);
            }

            const OrbitElements o = computeOrbitElements(orbit_pos, orbit_vel, mu);
            const double distance = o.distance;
            const double speed = o.speed;

            // Telemetry: e and |h| are the two conserved 2-body constants,
            // so a drifting plot = integrator drift; steps = burns/staging.
            energy_series.push(time, o.energy);
            angmom_series.push(time, o.ang_momentum);

            const glm::dvec3 up = getRelAxis_(ship->controller, 1);
            const glm::dvec3 facing = getRelAxis_(ship->controller, 2);
            const glm::dvec3 other = getRelAxis_(ship->controller, 0);

            const glm::dvec3 facing_dir = glm::normalize(facing);
            const glm::dvec3 vel_dir = glm::normalize(vel);

            const glm::dvec3 _up = glm::normalize(pos);
            const glm::dvec3 _north = glm::normalize(projectVecOntoPlane(glm::dvec3(0, 1, 0), _up));
            const glm::dvec3 _east = glm::cross(_up, _north);

            const double ver_speed = glm::length(glm::proj(surf_vel, pos)); // m/s
            const double hor_speed2 = glm::length(projectVecOntoPlane(surf_vel, _up)); // m/s

            const glm::dvec3 groundHed = glm::normalize(projectVecOntoPlane(facing, _up));

            const double hedNorth = glm::dot(groundHed, _north);
            const double hedEast = glm::dot(groundHed, _east);
            const double heading = wrapAngleToPositive(atan2(hedEast, hedNorth));

            const double yaw = heading;
            const double pitch = asin(glm::dot(_up, facing));
            const double roll =
                glm::orientedAngle(glm::normalize(projectVecOntoPlane(-pos, glm::normalize(facing))),
                                   glm::normalize(-up),
                                   glm::normalize(facing));

            // ImGui::Text("pos: %.2f %.2f %.2f", pos.x, pos.y, pos.z);
            // ImGui::Text("facing: %.2f %.2f %.2f", facing.x, facing.y, facing.z);
            // ImGui::Text("up: %.2f %.2f %.2f", up.x, up.y, up.z);
            // ImGui::Text("other: %.2f %.2f %.2f", other.x, other.y, other.z);
            // ImGui::Text("Ground hed: %.2f %.2f %.2f", groundHed.x, groundHed.y, groundHed.z);
            // ImGui::Text("Pitch: %.2f", glm::degrees(pitch));
            // ImGui::Text("Heading: %.2f", glm::degrees(heading));
            // ImGui::Text("up: %.2f, %.2f, %.2f", up.x, up.y, up.z);
            // ImGui::Text("facing: %.2f, %.2f, %.2f", facing.x, facing.y, facing.z);

            const glm::dvec3 dir = glm::normalize(surf_pos);

            const double longitude = atan2(dir.x, dir.z);
            const double latitude = asin(dir.y);

            if(draw_starfield) {
                skybox.Draw(camera, skyboxshader, sun->frame->GetOrientRelTo(ship->frame));
            }

            // Atmosphere rims: transparent Fresnel shells, drawn after the
            // skybox (the starfield is the background) so the rim ring blends
            // over it and the horizon haze blends over the already-drawn
            // terrain. Depth-write off; no-ops for bodies without an
            // atmosphere. See reports/atmosphere2026_08_25.
            if(world_drawing == true) {
                for(auto&& planet : planets) {
                    planet->DrawAtmosphere(camera, sun, ship->frame);
                }
            }

            /* draw engine plume */
            glm::dmat4 View = camera->GetView();
            glm::mat4 Projection = camera->GetProjection();
            if(ship->m_thrust > 0) {
                for(size_t t = 0; t < ship->m_thrusters.size(); t++) {
                    /* the plume mesh is authored for the base part
                       (radius 1 m, height 2 m): scale it to this thruster's
                       size so the tail lands on the engine tail (-h/2) */
                    const glm::dvec2 &dim = ship->m_thrusterDims[t];
                    glm::dmat4 Model = ship->m_thrusters[t]->model_matrix
                        * glm::dmat4(glm::dmat3(dim.x, 0.0, 0.0,
                                                 0.0, dim.x, 0.0,
                                                 0.0, 0.0, dim.y / 2.0));
                    // shifted into the render frame, like the view
                    glm::mat4 ModelViewFloat = View * glm::translate(-camera->GetRenderOrigin()) * Model;
                    engine_plume_model->shader->Bind();
                    engine_plume_model->shader->setUniform_mat4(0, Projection * ModelViewFloat);
                    engine_plume_model->shader->setUniform_mat4(1, glm::mat4(1.0)); // identity (GLM 1.0.0+: default ctor is zero)
                    engine_plume_model->shader->setUniform_vec3(2, glm::vec3(1, 1, 1));

                    glActiveTexture(GL_TEXTURE0);
                    glBindTexture(GL_TEXTURE_2D, engine_plume_model->texture->id);
                    glEnable(GL_BLEND);
                    glBlendFunc(GL_ONE, GL_ONE);
                    glDisable(GL_CULL_FACE);
                    engine_plume_model->mesh->Draw();
                    glEnable(GL_CULL_FACE);
                    glDisable(GL_BLEND);
                    glBindTexture(GL_TEXTURE_2D, 0);
                }
            }
            /* end draw engine plume */

            /* Transfer planner: rebuild the target list, then recompute
               the solution on input change or every 30 frames. */
            xferTargets.clear();
            {
                TerrainBody *pb = ship->frame->body;
                for(auto *b : planets) {
                    if(b->frame && b->frame->parent == pb->frame) {
                        xferTargets.push_back({b->name.c_str(), b, nullptr});
                    }
                }
                for(auto *s : ships) {
                    if(s != ship && s->frame && s->frame->body == pb) {
                        xferTargets.push_back({s->name.c_str(), nullptr, s});
                    }
                }
                // --transfer-target: explicit selection (e2e / scripting);
                // wins over the window's combo on every rebuild.
                if(!args.transfer_target.empty()) {
                    for(int i = 0; i < (int)xferTargets.size(); i++) {
                        if(xferTargets[i].name == args.transfer_target) {
                            xfer_target = i;
                        }
                    }
                }
            }
            if(xfer_target >= (int)xferTargets.size()) { xfer_target = -1; }

            if(xfer_target < 0) {
                xfer.valid = false;
                xfer.burn_dir = glm::dvec3(0.0);
            } else {
                xfer.frame++;
                const bool dirty = xfer.target != xfer_target
                    || xfer.auto_tof != xfer_auto
                    || std::fabs(xfer.tof_log - xfer_tof_log) > 1e-12
                    || xfer.frame - xfer.solved_frame >= 30;
                if(dirty) {
                    // Ship state in the parent's INERTIAL frame — the frame
                    // the transfer conic lives in (same idiom as the ORBITAL
                    // readout above).
                    Frame *sf = ship->frame;
                    Frame *inertial = sf->getNonRotFrame();
                    const glm::dvec3 r1 = sf->GetOrientRelTo(inertial) * com
                                         + sf->GetPositionRelTo(inertial);
                    const glm::dvec3 v1 = sf->GetOrientRelTo(inertial)
                                         * (vel + sf->GetStasisVelocity(com))
                                         + sf->GetVelocityRelTo(inertial);
                    const double mu_parent = inertial->body->mu;

                    // Target state in the same frame.
                    const XferTarget &t = xferTargets[xfer_target];
                    glm::dvec3 r2, v2;
                    double mu_target = 0.0, r_cap = 0.0;
                    double tof_max = 3.0 * 86400.0;
                    if(t.body) {
                        Frame *tf = t.body->frame;
                        r2 = tf->GetPositionRelTo(inertial);
                        // The body's orbital velocity in the parent frame,
                        // straight from the frame tree (nonzero now that the
                        // rails are Kepler orbits; for the current circular
                        // data this equals the old omega x r construction).
                        v2 = tf->GetVelocityRelTo(inertial);
                        mu_target = t.body->mu;
                        r_cap = t.body->radius + 100e3; // 100 km capture orbit
                        if(tf->orb_ang_speed > 0.0) {
                            // 3 full target periods covers the min-dv point
                            // (near the Hohmann ToF) with margin on both sides.
                            tof_max = 3.0 * (2.0 * M_PI / tf->orb_ang_speed);
                        }
                    } else {
                        // Ship in the same body: transform its state over to
                        // our inertial frame (stasis of the non-rotating
                        // frame is zero).
                        Frame *tsf = t.ship->frame;
                        const glm::dvec3 tcom = t.ship->get_center_of_mass();
                        const glm::dmat3 O = tsf->GetOrientRelTo(inertial);
                        r2 = O * tcom + tsf->GetPositionRelTo(inertial);
                        v2 = O * (t.ship->GetVel() + tsf->GetStasisVelocity(tcom))
                            + tsf->GetVelocityRelTo(inertial);
                    }

                    const bool capture = (t.body != nullptr);
                    TransferSolution sol;
                    if(xfer_auto) {
                        sol = planTransfer(r1, v1, r2, v2, mu_parent,
                                           mu_target, r_cap,
                                           60.0, tof_max, 150, capture);
                    } else {
                        const double tof = std::pow(10.0, xfer_tof_log);
                        sol = planTransfer(r1, v1, r2, v2, mu_parent,
                                           mu_target, r_cap,
                                           tof, tof, 1, capture);
                    }

                    xfer.sol = sol;
                    xfer.valid = sol.valid;
                    if(sol.valid) {
                        // Burn direction at the ship, in the render frame
                        // (ship->frame). A dv delta carries no stasis term:
                        // the same stasis applies before and after the burn
                        // at the same position, so it cancels in the
                        // difference.
                        const glm::dmat3 O = sf->GetOrientRelTo(inertial);
                        xfer.burn_dir = glm::transpose(O) * (sol.v_departure - v1);
                    } else {
                        xfer.burn_dir = glm::dvec3(0.0);
                    }
                    xfer.target = xfer_target;
                    xfer.auto_tof = xfer_auto;
                    xfer.tof_log = xfer_tof_log;
                    xfer.solved_frame = xfer.frame;
                }
            }

            // --xfer-log: the planner's current solution (render pass, since
            // that is where the computation lives).
            if(args.xfer_log && xfer_target >= 0) {
                const Uint32 now_ms = SDL_GetTicks();
                if(now_ms - xfer_log_last_ms >= orbit_log_interval_ms) {
                    xfer_log_last_ms = now_ms;
                    const char *tn = xferTargets[xfer_target].name;
                    if(xfer.valid) {
                        printf("[xferlog] t=%.1fs target=\"%s\" dv_dep=%.6g m/s "
                               "dv_cap=%.6g m/s total=%.6g m/s tof=%.6g s "
                               "v_inf=%.6g m/s r_cap=%.6g m "
                               "burn=[%.4f %.4f %.4f]\n",
                               time, tn, xfer.sol.dv_departure,
                               xfer.sol.dv_capture, xfer.sol.total_dv,
                               xfer.sol.tof, xfer.sol.v_inf, xfer.sol.r_cap,
                               xfer.burn_dir.x, xfer.burn_dir.y,
                               xfer.burn_dir.z);
                    } else {
                        printf("[xferlog] t=%.1fs target=\"%s\" no-solution\n",
                               time, tn);
                    }
                    fflush(stdout);
                }
            }

            glDisable(GL_DEPTH_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            front_indicator->pos = facing;
            front_indicator->Draw(camera, M_PI /* <- ?? */ + roll);
            prograde_indicator->pos = vel;
            prograde_indicator->Draw(camera, M_PI);
            retrograde_indicator->pos = - vel;
            retrograde_indicator->Draw(camera, M_PI);
            radial_in_indicator->pos = - pos;
            radial_in_indicator->Draw(camera, M_PI);
            radial_out_indicator->pos = pos;
            radial_out_indicator->Draw(camera, M_PI);
            normal_plus_indicator->pos = glm::cross(pos, vel);
            normal_plus_indicator->Draw(camera, M_PI);
            normal_minus_indicator->pos = -glm::cross(pos, vel);
            normal_minus_indicator->Draw(camera, M_PI);
            // Transfer burn direction (TRANSFER window target selected):
            // KSP-blue prograde icon pointing where the departure burn goes.
            if(xfer.valid && glm::length(xfer.burn_dir) > 0.0) {
                burn_indicator->pos = xfer.burn_dir;
                burn_indicator->Draw(camera, M_PI);
            }
            // horizon_indicator->pos = groundHed;
            // horizon_indicator->Draw(camera, M_PI);

            if(draw_skylines) {
                glLineWidth(4);
                lineshader->Bind();
                lineshader->setUniform_mat4(0, glm::dmat4(camera->GetProjection()) * glm::dmat4(glm::dmat3(camera->GetView())));
                // XZ plane (flat / orbital-equatorial reference): green
                lineshader->setUniform_vec4(1, glm::vec4(0, 1, 0, 0.5));
                skyline_xz->Draw(GL_LINE_LOOP);
                // XY plane (vertical / meridian reference): magenta
                lineshader->setUniform_vec4(1, glm::vec4(1, 0, 1, 0.5));
                skyline_xy->Draw(GL_LINE_LOOP);
            }

            glDisable(GL_BLEND);
            glEnable(GL_DEPTH_TEST);

            if(physics_debug_drawing == true) {
                glDisable(GL_DEPTH_TEST);
                debug_draw(camera);
                glEnable(GL_DEPTH_TEST);
            }

            postfx->End();  // no-op unless --postfx effects are active

            /*
              ImGui stuff below
            */

            glUseProgram(0);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            if(poly_mode == true) {
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            }

            /* Top bar: one fixed window (no move, no resize, re-placed every
               frame so it tracks the viewport). Row 1: speed + altitude
               (big font) — orbital (ASL + orbital speed) when in the
               inertial frame or above 30km ASL, else surface (terrain
               altitude + ground speed) in the rotating frame.
               Row 2: Kerbin clock (regular font, centered). */
            ui::Window("HUD", game.o_hud, [&] {
                const double asl = distance - ship->m_parent->radius;
                const double agl = distance - ship->m_parent->GetTerrainHeight(glm::normalize(pos));
                const bool surface_mode = ship->frame->isRotFrame() && asl < 30000.0;
                const double alt = surface_mode ? agl : asl;
                const double spd = surface_mode ? glm::length(surf_vel) : speed;
                ImGui::PushFont(bigger);
                ImGui::Text("%06dm/s   %08dm", (int)spd, (int)alt);
                ImGui::PopFont();
                if(sys.home && sys.home->cal.valid()) {
                    CalTime ct = sys.home->cal.at(time);
                    char line[64];
                    if(ct.has_year) {
                        // CalTime only exposes month + day-of-month, so the
                        // day-of-year is day + the days in the earlier months.
                        int doy = ct.day;
                        for(int m = 0; m < ct.month - 1; m++) {
                            doy += sys.home->cal.month_days[m];
                        }
                        snprintf(line, sizeof(line),
                                 "Year %04d   Day %d/%d   %02d:%02d:%02d",
                                 ct.year, doy, sys.home->cal.days_per_year,
                                 ct.hh, ct.mm, ct.ss);
                    } else {
                        snprintf(line, sizeof(line),
                                 "Day %d   %02d:%02d:%02d",
                                 ct.day, ct.hh, ct.mm, ct.ss);
                    }
                    ImGui::SetCursorPosX((ImGui::GetWindowWidth() - ImGui::CalcTextSize(line).x) * 0.5f);
                    ImGui::TextUnformatted(line);
                }
            });

            /* Window list (single source of truth: the game.ui_windows
               table) plus the Top-HUD group switch. */
            ui::Window("Windows", o_menu, [&] {
                ImGui::Spacing();
                for(auto &w : game.ui_windows) {
                    bool open = ui::IsOpen(w.name);
                    if(ImGui::Checkbox(w.label, &open)) {
                        ui::SetOpen(w.name, open);
                    }
                }
                bool hud = ui::IsOpen("HUD");
                if(ImGui::Checkbox("Top HUD", &hud)) {
                    ui::SetOpen("HUD", hud);
                }
            });

            // Settings: the render/physics debug toggles (moved out of
            // Game Debug Info, which is now read-only diagnostics).
            ui::Window("Settings", o_settings, [&] {
                ImGui::Checkbox("Physics debug draw", &physics_debug_drawing);
                ImGui::Checkbox("World draw", &world_drawing);
                ImGui::Checkbox("Starfield", &draw_starfield);
                ImGui::Checkbox("Reference circles", &draw_skylines);
                if(ImGui::Combo("UI style", &ui_style, "Dark\0Light\0Classic\0")) {
                    if(ui_style == 0) ImGui::StyleColorsDark();
                    else if(ui_style == 1) ImGui::StyleColorsLight();
                    else ImGui::StyleColorsClassic();
                }
                if(ImGui::SliderFloat("Window rounding", &window_rounding,
                                     0.0f, 50.0f, "%.0f")) {
                    ImGui::GetStyle().WindowRounding = window_rounding;
                }
                if(ImGui::SliderFloat("FOV", &args.camFovDeg, 10.0f, 120.0f, "%.0f°")) {
                    const float f = (float)glm::radians(args.camFovDeg);
                    orbitCam->setFov(f);
                    freeCam->setFov(f);
                }
            });

            // Transfer planner: parent->child body transfers (with capture)
            // and same-body ship intercepts. The solution is computed in the
            // render pass (xfer), so this window is pure readout + inputs.
            ui::Window("TRANSFER", o_transfer, [&] {
                if(xferTargets.empty()) {
                    ImGui::Text("No transfer targets: no child bodies or ships here.");
                    return;
                }
                const char *cur = (xfer_target >= 0)
                                 ? xferTargets[xfer_target].name : "none";
                if(ImGui::BeginCombo("Target", cur)) {
                    for(int i = 0; i < (int)xferTargets.size(); i++) {
                        if(ImGui::Selectable(xferTargets[i].name,
                                             i == xfer_target)) {
                            xfer_target = i;
                        }
                    }
                    ImGui::EndCombo();
                }
                if(xfer_target < 0) {
                    ImGui::Text("Select a target body or ship.");
                    return;
                }
                const bool isShip = xferTargets[xfer_target].ship != nullptr;
                if(isShip) {
                    ImGui::TextDisabled("ship target: intercept only, no capture burn");
                }
                ImGui::Checkbox("Auto ToF (min dv)", &xfer_auto);
                if(!xfer_auto) {
                    ImGui::SliderFloat("log10(ToF s)", &xfer_tof_log,
                                       1.8, 7.5, "%.2f");
                    ImGui::Text("ToF: %s",
                                fmt_time(std::pow(10.0, xfer_tof_log)).c_str());
                }
                if(!xfer.valid) {
                    ImGui::Text("No transfer solution for this target / ToF.");
                    return;
                }
                const TransferSolution &sol = xfer.sol;
                ImGui::Text("dv depart:  %08.1f m/s", sol.dv_departure);
                if(!isShip) {
                    ImGui::Text("dv capture: %08.1f m/s @ %.0f km",
                                sol.dv_capture, sol.r_cap / 1000.0);
                    if(sol.capture_orbit_period > 0.0) {
                        ImGui::Text("capture P:  %s",
                                    fmt_time(sol.capture_orbit_period).c_str());
                    }
                }
                ImGui::Text("total dv:   %08.1f m/s", sol.total_dv);
                ImGui::Text("ToF:        %s", fmt_time(sol.tof).c_str());
                ImGui::Text("v_inf:      %08.1f m/s", sol.v_inf);
                if(sol.transfer_semi_major > 0.0) {
                    ImGui::Text("transfer:   ellipse  a=%.6g m  e=%.3f",
                                sol.transfer_semi_major, sol.transfer_ecc);
                } else if(sol.transfer_semi_major < 0.0) {
                    ImGui::Text("transfer:   hyperbolic  a=%.6g m  e=%.3f",
                                sol.transfer_semi_major, sol.transfer_ecc);
                } else {
                    ImGui::Text("transfer:   parabolic  e=%.3f",
                                sol.transfer_ecc);
                }
            });

            ui::Window("Game Debug Info", o_debug, [&] {
                ImGui::Text("Time: %f", time);
                if(sys.home && sys.home->cal.valid()) {
                    CalTime ct = sys.home->cal.at(time);
                    if(ct.has_year) {
                        ImGui::Text("Clock:  Yr %d  Mo %d  Day %d  %02d:%02d:%02d  (%s time)",
                                    ct.year, ct.month, ct.day, ct.hh, ct.mm, ct.ss,
                                    sys.home->name.c_str());
                    } else {
                        ImGui::Text("Clock:  Day %d  %02d:%02d:%02d  (%s time)",
                                    ct.day, ct.hh, ct.mm, ct.ss,
                                    sys.home->name.c_str());
                    }
                }
                // Local date + time on the body the ship is currently in,
                // when it's not the home planet (e.g. the Moon's own day).
                TerrainBody *local_body = (ship && ship->frame)
                                        ? ship->frame->body : nullptr;
                if(local_body && local_body != sys.home &&
                   local_body->cal.valid()) {
                    CalTime lt = local_body->cal.at(time);
                    if(lt.has_year) {
                        ImGui::Text("Local:  %s  Yr %d  Mo %d  Day %d  %02d:%02d:%02d",
                                    local_body->name.c_str(),
                                    lt.year, lt.month, lt.day,
                                    lt.hh, lt.mm, lt.ss);
                    } else {
                        ImGui::Text("Local:  %s  Day %d  %02d:%02d:%02d",
                                    local_body->name.c_str(),
                                    lt.day, lt.hh, lt.mm, lt.ss);
                    }
                }
                ImGui::Text("Patches: %d", ship->m_parent->CountPatches());
                ImGui::Text("Cam speed: %d", cam_speed);
                ImGui::Text("Time Accel: %d%s", time_accel,
                            time_accel >= kRailsWarp ? " (rails)" : "");
                ImGui::Text("Camera altitude: %0.f",
                            glm::length(camera->GetPos()) - ship->m_parent->GetTerrainHeight(glm::normalize(camera->GetPos())));
                ImGui::Text("Camera ASL: %0.f", glm::length(camera->GetPos()) - ship->m_parent->radius);
                ImGui::Text("Camera Pos: %.0f %.0f %0.f", camera->GetPos().x, camera->GetPos().y, camera->GetPos().z);
                ImGui::Text("Cam forward: %.2f %.2f %.2f",
                            camera->forward.x, camera->forward.y, camera->forward.z);
                if(ImGui::Button("Print camera pose (CLI args)")) {
                    // Copy-paste the printed line after ./osp to relaunch at this view.
                    // pos/forward/up are world / ship-frame coordinates.
                    printf("Camera pose:\n");
                    printf("--free-cam-pos %.9g %.9g %.9g "
                           "--free-cam-fwd %.9g %.9g %.9g "
                           "--free-cam-up %.9g %.9g %.9g "
                           "--fov %.0f\n",
                           camera->pos.x, camera->pos.y, camera->pos.z,
                           camera->forward.x, camera->forward.y, camera->forward.z,
                           camera->up.x, camera->up.y, camera->up.z,
                           args.camFovDeg);
                }
                ImGui::Text("Home distance: %f",
                            glm::length(ship->GetPositionRelTo(ship->controller,
                                                                ships.homeOf(activeIdx)->frame)));
                ImGui::Text("Pos: %.3fkm", distance / 1000);
                ImGui::Text("xyz(%0.f, %0.f, %0.f)", pos.x, pos.y, pos.z);
                ImGui::Text("Vel: %.3fm/s", speed);
                ImGui::Text("xyz(%0.f, %0.f, %0.f)", vel.x, vel.y, vel.z);
            });

            // Labels are abbreviated to <= 3 chars and right-padded to the
            // same width so the values start at a tidy column.
            ui::Window("ORBITAL", o_orbit, [&] {
                ImGui::Text("Vel: %.1fm/s", speed);
                ImGui::Text("Alt: %.1fm", distance);
                /* Every line below is always present; "-" = the quantity
                   does not exist for this orbit class. Escape trajectories
                   have no apoapsis and no period; a near-circular orbit
                   (apsides within 10 km) has no apsis line, so the
                   countdowns to one are numerically meaningless. */
                const bool circular = o.ecc < 1.0
                                    && (o.apoapsis - o.periapsis) < 10e3;
                if(o.ecc < 1.0) { ImGui::Text("ApA: %.1fm", o.apoapsis); }
                else { ImGui::Text("ApA: -"); }
                if(o.ecc < 1.0 && !circular) { ImGui::Text("ApT: %.1fs", o.time_to_apo); }
                else { ImGui::Text("ApT: -"); }
                ImGui::Text("PeA: %.1fm", o.periapsis);
                if(!circular && o.time_to_peri >= 0.0) { ImGui::Text("PeT: %.1fs", o.time_to_peri); }
                else { ImGui::Text("PeT: -"); }
                if(o.period > 0.0) { ImGui::Text("  T: %.1fs", o.period); }
                else { ImGui::Text("  T: -"); }
                ImGui::Text("Inc: %.2f", glm::degrees(o.inclination));
                ImGui::Text("Ecc: %f", o.ecc);
                ImGui::Text("SMa: %.1fm", o.semi_major);
                ImGui::Text("LAN: %.2f", glm::degrees(o.raan));
                ImGui::Text("LPe: %.2f", glm::degrees(o.arg_periapsis));
                double prograde_angle = glm::angle(facing_dir, vel_dir);
                double retrograde_angle = glm::angle(facing_dir, - vel_dir);
                ImGui::Text("Prg: %.2f", glm::degrees(prograde_angle));
                ImGui::Text("Rtg: %.2f", glm::degrees(retrograde_angle));
                ImGui::Text("Eng: %.2f J", o.energy);
            });

            // Initial size comes from o_telemetry.initial_size (the plots
            // need real estate; content-fit would clip them).
            ui::Window("TELEMETRY", o_telemetry, [&] {
                // Two separate plots: e (~1e5) and |h| (~1e11) differ by
                // ~6 orders of magnitude, so sharing one axis would flatten
                // e to a line and hide the drift we're looking for.
                if(energy_series.count > 1) {
                    const int n = energy_series.stage();
                    ImPlot::SetNextAxesToFit();  // live auto-fit, both axes
                    if(ImPlot::BeginPlot("specific orbital energy (J/kg)")) {
                        ImPlot::SetupAxis(ImAxis_X1, "t (s)");
                        ImPlot::PlotLine("e", energy_series.t_arr(), energy_series.v_arr(), n);
                        ImPlot::EndPlot();
                    }
                }
                if(angmom_series.count > 1) {
                    const int n = angmom_series.stage();
                    ImPlot::SetNextAxesToFit();  // live auto-fit, both axes
                    if(ImPlot::BeginPlot("angular momentum (m^2/s)")) {
                        ImPlot::SetupAxis(ImAxis_X1, "t (s)");
                        ImPlot::PlotLine("|h|", angmom_series.t_arr(), angmom_series.v_arr(), n);
                        ImPlot::EndPlot();
                    }
                }
            });

            // Labels right-padded to 3 chars, same as ORBITAL.
            ui::Window("SURFACE", o_surface, [&] {
                ImGui::Text("Alt: %.1fm", distance - ship->m_parent->GetTerrainHeight(glm::normalize(pos)));
                ImGui::Text("ASL: %.1fm", distance - ship->m_parent->radius);
                ImGui::Text(" Vs: %.2fm/s", ver_speed);
                ImGui::Text(" Hs: %.2fm/s", hor_speed2);
                ImGui::Text("Lat: %.4f", glm::degrees(latitude));
                ImGui::Text("Lon: %.4f", glm::degrees(longitude));
                ImGui::Text(" Pt: %.2f", glm::degrees(pitch));
                ImGui::Text("  R: %.2f", glm::degrees(roll));
                ImGui::Text("Hdg: %.2f", glm::degrees(yaw));
            });

            ui::Window("SHIPS", o_ships, [&] {
            // Buttons (natural width) + SameLine, the same pattern as the
            // map controls: a full-width Selectable in this auto-resize window
            // would swallow the line and push the "x" off it (or collapse the
            // window), so each name is its own sized button. The active ship
            // is highlighted with a pushed color.
            for(size_t i = 0; i < ships.size(); i++) {
                const bool active = ((int)i == activeIdx);
                ImGui::PushID((int)i);
                if(active) {
                    ImGui::PushStyleColor(ImGuiCol_Button,
                                         ImVec4(0.30f, 0.45f, 0.70f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                                         ImVec4(0.35f, 0.50f, 0.75f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive,
                                         ImVec4(0.40f, 0.55f, 0.80f, 1.0f));
                }
                if(ImGui::Button(ships[i]->name.c_str())) {
                    game.select_ship((int)i);
                }
                if(active) {
                    ImGui::PopStyleColor(3);
                }
                ImGui::SameLine();
                if(ImGui::SmallButton("x")) {
                    game.remove_ship((int)i);
                    i = ships.size();   // the list shrank; stop iterating
                }
                ImGui::PopID();
            }
            ImGui::Separator();
            if(ImGui::Button("Spawn a copy of the active ship")) {
                if(!ship->defPath.empty()) {
                    ships.spawn_ship(ship->defPath, "", ships.homeOf(activeIdx),
                                     ships.scenarioOf(activeIdx), sys);
                } else {
                    printf("Spawn: active ship has no def (test ship)\n");
                }
            }
            ImGui::Text("click name - select    x - remove");
            });

            ui::Window("VESSEL", o_vessel, [&] {
                ImGui::Text("Ship: %s", ship->name.c_str());
                ImGui::Text("Stage: %d / %d  (SPACE to drop)",
                            ship->activeStage(), ship->numStages());
                ImGui::Text("Reference frame: %s", ship->frame->name.c_str());
                ImGui::Text("Reference frame type: %s", ship->frame->isRotFrame() ? "Rotational" : "Inertial");
                ImGui::Text("Mass: %.3fkg", ship->getMass());
                ImGui::Text("Delta-v: %.1fm/s", ship->getDeltaV());
                ImGui::Text("Thrust Util: %.0f%%", ship->thruster_util * 100);
                ImGui::Text("Thrust: %.2fN", ship->getThrust());
                ImGui::Text("Current TWR: %.2f/%.2f", ship->getTWR(), ship->getFullThrustTWR());
                ImGui::Text("Max TWR: %.2f", ship->getMaxTWR());
                ImGui::Text("Wheel torque: %.0fN m", ship->GetWheelTorque());
                ImGui::Text("Angular rate: %.2fdeg/s",
                            glm::degrees(glm::length(GetAngVelocity(ship->controller))));
            });
            ui::Window("SHIP PARTS", o_parts, [&] {
                int i = 0;
                for(auto&& part : ship->parts) {
                    ImGui::Text("Part #%d  (stage %d)", i, ship->partStages[i]);
                    ImGui::Separator();
                    ImGui::Text("Name: %s", ship->partDefs[i]->name.c_str());
                    ImGui::Text("Mass: %.3fkg", part->mass);
                    ImGui::Text("Hydrogen: %.3fkg/%.3fkg",
                                ship->partResources[i].current[(int)ResourceType::Hydrogen],
                                ship->partResources[i].capacity[(int)ResourceType::Hydrogen]);
                    ImGui::Text("LOX: %.3fkg/%.3fkg",
                                ship->partResources[i].current[(int)ResourceType::LOX],
                                ship->partResources[i].capacity[(int)ResourceType::LOX]);
                    ImGui::Spacing();
                    i++;
                }
            });

            ui::Window("Controls", o_controls, [&] {
                ImGui::Text("Game");
                ImGui::Separator();
                ImGui::Text("p - toggle wireframe mode");
                ImGui::Text(", - decrease time acceleration");
                ImGui::Text(". - increase time acceleration");
                ImGui::Text("k - decrease camera speed");
                ImGui::Text("l - increase camera speed");
                ImGui::Text("c - switch mode: orbit (flying) <-> free (exploring)");
                ImGui::Text("g - orbit mode: cycle target (ship/sun/planet/moon)");
                ImGui::Text("tab - toggle windows");
                ImGui::Text("f6 - next ship (fleet)");
                ImGui::Text("SHIPS window - select a ship, spawn a copy, remove one (x)");
                ImGui::Text("f10 - reset windows");
                ImGui::Text("esc - main menu");
                ImGui::Text("mouse - UI (hold RMB over 3D to look, both modes)");
                ImGui::Text("RMB (orbital map) - cycle window -> bare map -> no window");
                ImGui::Text("wheel - zoom (orbit mode)");
                ImGui::Spacing();
                ImGui::Text("Orbit mode (flying the ship)");
                ImGui::Separator();
                ImGui::Text("w/s - pitch up/down");
                ImGui::Text("a/d - yaw left/right");
                ImGui::Text("q/e - roll left/right");
                ImGui::Text("i - fire ship engines");
                ImGui::Text("x - kill rotation");
                ImGui::Text("b - align prograde");
                ImGui::Text("n - align retrograde");
                ImGui::Text("r/f - throttle up/down");
                ImGui::Text("SPACE - separate the active stage");
                ImGui::Spacing();
                ImGui::Text("Free mode (exploring)");
                ImGui::Separator();
                ImGui::Text("w/s - forward/back");
                ImGui::Text("a/d - strafe");
                ImGui::Text("q/e - roll");
                ImGui::Text("shift/ctrl - up/down");
            });

            ui::Window("Autopilot", o_autopilot, [&] {
                ImGui::Button("Prograde");
                ImGui::Button("Retrograde");
                ImGui::Button("Radial-in");
                ImGui::Button("Radial-out");
                ImGui::Button("Normal");
                ImGui::Button("Anti-normal");
            });

            ui::Window("RESOURCES", o_resources, [&] {
                // aggregate across the active ship's parts (any ship layout)
                float h_cur = 0, h_cap = 0, l_cur = 0, l_cap = 0;
                for(size_t i = 0; i < ship->partResources.size(); i++) {
                    h_cur += ship->partResources[i].current[(int)ResourceType::Hydrogen];
                    h_cap += ship->partResources[i].capacity[(int)ResourceType::Hydrogen];
                    l_cur += ship->partResources[i].current[(int)ResourceType::LOX];
                    l_cap += ship->partResources[i].capacity[(int)ResourceType::LOX];
                }
                const float hydrogen_frac = (h_cap > 0) ? h_cur / h_cap : 0.0f;
                const float lox_frac = (l_cap > 0) ? l_cur / l_cap : 0.0f;

                ImGui::ProgressBar(hydrogen_frac, ImVec2(-1, 0), "Hydrogen");
                ImGui::ProgressBar(lox_frac, ImVec2(-1, 0), "LOX");
                ImGui::ProgressBar(0.13, ImVec2(-1, 0), "Hydrazine");
                ImGui::ProgressBar(0.45, ImVec2(-1, 0), "Electric charge");
                ImGui::ProgressBar(0.75, ImVec2(-1, 0), "Oxygen");
                ImGui::ProgressBar(0.83, ImVec2(-1, 0), "Water");
                ImGui::ProgressBar(0.94, ImVec2(-1, 0), "Food");
            });

            // Mode 2 strips the window chrome entirely (see map_mode): the
            // window is invisible but still hit-tested, so the map below
            // keeps pan/zoom and the right-click cycle.
            if(map_mode == 2) {
                o_map.flags |= ImGuiWindowFlags_NoDecoration |
                               ImGuiWindowFlags_NoBackground;
            } else {
                o_map.flags = 0;
            }
            ui::Window("Orbital map", o_map, [&] {
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

            // Main menu: Esc toggles it. Fixed, so it stays centered and
            // tracks viewport resizes. Drawn last so it sits on top.
            // Buttons have an explicit width: -1 (full width) inside an
            // auto-resizing window collapses the window to a sliver.
            ui::Window("Main Menu", o_mainmenu, [&] {
                ImGui::PushFont(bigger);
                if(ImGui::Button("Toggle windows", ImVec2(240.0f, 0.0f))) {
                    game.toggle_windows();
                }
                if(ImGui::Button("Reset windows", ImVec2(240.0f, 0.0f))) {
                    ui::ResetGui();
                }
                if(ImGui::Button("Settings", ImVec2(240.0f, 0.0f))) {
                    ui::SetOpen("Settings", true);
                }
                if(ImGui::Button("Controls", ImVec2(240.0f, 0.0f))) {
                    ui::SetOpen("Controls", true);
                }
                if(ImGui::Button("Quit game", ImVec2(240.0f, 0.0f))) {
                    running = false;
                }
                ImGui::PopFont();
            });

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
