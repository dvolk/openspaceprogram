// game.h -- the running game: the long-lived subsystems (borrowed from
// main) plus the runtime state -- the cameras, the clock, the active ship,
// the input/UI flags, the orbit-camera focus targets, the UI window
// registry -- and the control transitions (select/remove a ship, enter
// rails warp, toggle the windows).
//
// main() creates the subsystems, builds the Game once they exist, and then
// drives everything through it: the event dispatch (events.cpp), the logic
// tick (tick.cpp) and the render section of the main loop read and write
// the state here, so there is a single source of truth for it. Game owns
// none of the subsystems (main still creates and deletes them); it only
// borrows them.
// The small runtime state (clock, selection, flags, focus, UI registry)
// is owned by Game.
#pragma once

#include <SDL2/SDL.h>   // Uint32

#include <string>
#include <vector>

#include "camera.h"   // Camera, CameraMode
#include "cli.h"      // GameArgs
#include "display.h"  // Renderer
#include "orbit.h"    // OrbitElements (the ShipView state)
#include "postfx.h"   // PostFX
#include "ships.h"    // Ships
#include "siminput.h" // TimeSeries (the ShipView telemetry)
#include "system.h"   // System
#include "terrain.h"  // TerrainBody
#include "ui.h"       // ui::Options
#include "vehicle.h"  // Vehicle

// Render resources (render.cpp draws with them; main owns their lifetime).
// Forward-declared so Game can hold them by pointer without pulling their
// headers into every includer.
struct Billboard;
struct Mesh;
struct Model;
struct Shader;
struct Skybox;

// Rails warp threshold: at accel > 10 nobody is integrated -- every ship
// coasts on rails (or sits frozen on the ground) and the Bullet world is
// not stepped, so a tick costs O(ships). At 10 and below the active ship
// is in the physics world. 11 == "the first accel above 10" for the
// power-of-10 warps (1, 10, 100, ...). Shared by the event dispatch, the
// logic tick and the startup clamp (was a local const in main).
static const int kRailsWarp = 11;

// One-shot on-screen messages (g.toast), drawn centered by gameui.cpp.
// Wall-clock lifetimes: sim time is paused or warped, so a UI message must
// not live or die with the sim clock.
static const double kToastLife = 3.0;   // seconds a toast stays up
static const int kToastVisible = 3;    // the last N toasts shown (stacked)
struct ToastMsg {
    std::string text;
    double born;   // wall-clock seconds (SDL_GetTicks() * 0.001)
};

// The active ship's per-frame state in the render frame (ship->frame):
// computed once per drawn frame by the 3D pass (render.cpp) and read by
// the UI readouts (HUD / ORBITAL / SURFACE / TELEMETRY / the map) and the
// TRANSFER planner. Was the local cluster in main's render section (com /
// vel / o / the orbit + surface state / the attitude and lat-lon scalars /
// the telemetry series).
struct ShipView {
    // render-frame (ship->frame) state
    glm::dvec3 pos;      // the ship's COM
    glm::dvec3 vel;
    // the surface frame (the body's rotating frame, when the ship is on one)
    glm::dvec3 surf_pos;
    glm::dvec3 surf_vel;
    // the orbit conic, in the ship's non-rotating (inertial) frame
    glm::dvec3 orbit_pos;
    glm::dvec3 orbit_vel;
    OrbitElements o;
    double distance = 0;  // |orbit_pos|
    double speed = 0;     // |orbit_vel|
    double mu = 0;        // the parent body's mu (the conic's)
    // attitude readouts (render frame)
    glm::dvec3 up;
    glm::dvec3 facing;
    glm::dvec3 other;
    glm::dvec3 facing_dir;
    glm::dvec3 vel_dir;
    // surface + orientation scalars (the SURFACE window)
    double ver_speed = 0;
    double hor_speed2 = 0;
    double latitude = 0;
    double longitude = 0;
    double heading = 0;
    double pitch = 0;
    double roll = 0;
    // telemetry (sampled once per drawn frame)
    TimeSeries energy_series;
    TimeSeries angmom_series;
};

struct Game {
    // --- borrowed subsystems (main creates + deletes) ---------------------
    Renderer &display;
    PostFX *postfx;
    Ships &ships;
    System &sys;
    TerrainBody *sun;
    TerrainBody *home;
    GameArgs &args;
    Uint32 sim_win_id;
    Uint32 loop_start_ms = 0;   // set once the main loop is about to start

    // --- cameras -----------------------------------------------------------
    Camera *camera = nullptr;   // one object: orbit + free, `camera->mode` picks
    int cam_speed = 1;

    // --- the clock ----------------------------------------------------------
    int time_accel = 0;
    double time = 0;   // the analytic sim clock (s), advanced by the tick

    // --- one-shot on-screen messages (gameui.cpp draws the last N) ----------
    std::vector<ToastMsg> toasts;

    // --- the fixed-timestep loop (tick.cpp) ---------------------------------
    // The loop adds the measured frame time to the accumulator each frame
    // and burns off whole physics steps (dt) from it.
    double currentTime = 0.001 * (double)(SDL_GetTicks());
    double accumulator = 0.0;
    const double dt = 1.0/50.0;   // TODO explain why 50
    bool redraw = false;         // a frame of logic ran: RENDER should draw

    // --- the wall-clock log gates (--orbit-interval; tick.cpp + xfer-log) ---
    const Uint32 orbit_log_interval_ms = (Uint32)(args.orbit_interval * 1000.0);
    Uint32 orbit_log_last_ms = 0;
    /* Separate timestamp: the two logs share --orbit-interval but must not
       share the "last fired" time, or the earlier block in the loop always
       wins and the other never fires (and one alone spews every tick). */
    Uint32 dbg_log_last_ms = 0;

    // --- input / selection state -------------------------------------------
    bool running = true;
    bool rmbCam = false;            // RMB held over 3D: camera look
    bool poly_mode = false;         // F11 wireframe
    bool screenshot_requested = false;
    bool porkchop_compute_requested = false;  // P: one-shot compute the plot
    int activeIdx = 0;             // the player-controlled ship
    Vehicle *ship = nullptr;       // always ships[activeIdx]

    // --- render resources (render.cpp draws with them) ---------------------
    // main creates + deletes them (the teardown at the end of main); they
    // are handed over here once they exist.
    Skybox *skybox = nullptr;
    Shader *skyboxshader = nullptr;
    Shader *lineshader = nullptr;
    Model *engine_plume_model = nullptr;
    Mesh *skyline_xz = nullptr;
    Mesh *skyline_xy = nullptr;
    Billboard *front_indicator = nullptr;
    Billboard *prograde_indicator = nullptr;
    Billboard *retrograde_indicator = nullptr;
    Billboard *radial_in_indicator = nullptr;
    Billboard *radial_out_indicator = nullptr;
    Billboard *normal_plus_indicator = nullptr;
    Billboard *normal_minus_indicator = nullptr;
    Billboard *burn_indicator = nullptr;

    // --- the draw toggles (the Settings window writes, render.cpp reads) --
    bool physics_debug_drawing = false;
    bool world_drawing = true;
    bool draw_starfield = true;
    bool draw_skylines = false;

    // --- the active ship's per-frame state (render.cpp writes it) ----------
    ShipView view;

    // --- orbit camera focus targets (the ship + every body; G cycles) ------
    struct FocusTarget { const char *name; TerrainBody *body; };
    std::vector<FocusTarget> focusTargets;
    int focusBody = 0;             // index into focusTargets
    int numFocusTargets = 0;

    // --- UI window registry (the TAB toggle + the main-menu button) --------
    struct UiWin { const char *name; const char *label; ui::Options opts; };
    std::vector<UiWin> ui_windows;
    ui::Options o_hud;
    bool ui_visible = true;

    // --- UI window options (gameui.cpp draws with them) ---------------------
    // The per-window layout blocks (slot, the right_of / below chain, size,
    // default-open state), set up once by setup_ui_windows(); the registry
    // above copies the ones the TAB toggle controls.
    ui::Options o_orbit;
    ui::Options o_surface;
    ui::Options o_resources;
    ui::Options o_menu;
    ui::Options o_vessel;
    ui::Options o_parts;
    ui::Options o_map;
    ui::Options o_ships;
    ui::Options o_autopilot;
    ui::Options o_controls;
    ui::Options o_debug;
    ui::Options o_telemetry;
    ui::Options o_settings;
    ui::Options o_transfer;
    ui::Options o_mainmenu;

    // The big face (2x the UI font), created by main at ImGui init.
    ImFont *bigger = nullptr;

    // The Settings window state (gameui.cpp writes it; apply_ui_style()
    // rebuilds the imgui style from it).
    int ui_style = 0;              // 0=dark (imgui default) 1=light 2=classic
    float window_rounding = 0.0f;  // imgui default
    float ui_alpha = 1.0f;         // global imgui alpha (window transparency)
    float ui_scale = 1.0f;         // DPI scale: fonts + style sizes

    // --- Orbital map state (gameui.cpp draws with them) ---------------------
    // Orbital map: meters per pixel (the "Scale" slider) + the chosen map
    // plane (0 = equatorial, 1 = ecliptic, 2 = orbital).
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

    // --- control transitions (events + the SHIPS window + selftest) --------
    // World (ship-frame) position of a focus target, to point the orbit
    // camera at it.
    glm::dvec3 focusWorldPos(int i) const;
    // Build the per-window UI options + the window registry (game.cpp).
    void setup_ui_windows();
    // TAB: toggle the info windows (the main menu's "Toggle windows" too).
    void toggle_windows();
    // Rebuild the imgui style from the Settings state (theme, DPI scale,
    // rounding, transparency).
    void apply_ui_style();
    // Take control of a ship (release + park the current one, recenter the
    // orbit camera, drop rails warp).
    void select_ship(int idx);
    // Enter rails warp (park every ship); false + keeps the accel if any
    // ship is not rail-eligible.
    bool enter_rails_warp();
    // Remove a ship + its bookkeeping (refuses the last one; hands control
    // off if the active one is removed).
    void remove_ship(int idx);
    // Push a one-shot on-screen message (printf-style), shown for
    // kToastLife wall-clock seconds (the last kToastVisible stack).
    void toast(const char *fmt, ...);

    Game(Renderer &display, PostFX *postfx, Ships &ships, System &sys,
         TerrainBody *sun, TerrainBody *home, GameArgs &args,
         Uint32 sim_win_id)
        : display(display), postfx(postfx), ships(ships), sys(sys),
          sun(sun), home(home), args(args), sim_win_id(sim_win_id) {}
};
