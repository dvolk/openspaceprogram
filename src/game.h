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

#include <SDL3/SDL.h>   // Uint32

#include <string>
#include <vector>

#include "camera.h"   // Camera, CameraMode
#include "cli.h"      // GameArgs
#include "display.h"  // Renderer
#include "job.h"      // JobRunner (background jobs: the porkchop grid, ...)
#include "orbit.h"    // OrbitElements (the ShipView state)
#include "postfx.h"   // PostFX
#include "scene.h"    // SceneId, SceneFrame (the scene stack), Backdrop
#include "ships.h"    // Ships
#include "siminput.h" // TimeSeries (the ShipView telemetry)
#include "system.h"   // System
#include "terrain.h"  // TerrainBody
#include "ui.h"       // ui::Options
#include "eva.h"      // Kerbal (the crew characters, the aboard state)
#include "vehicle.h"  // Vehicle
#include "keys.h"     // KeyBindings (the rebindable key map)

// Render resources (render.cpp draws with them; main owns their lifetime).
// Forward-declared so Game can hold them by pointer without pulling their
// headers into every includer.
struct Billboard;
struct Mesh;
struct Shader;
struct Skybox;
struct Texture;

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

// RMB click (pick a part) vs RMB drag (camera look): a press that moves
// less than this much and lasts under this long is a click (events.cpp).
static const int kPickClickPx = 6;     // total cursor motion, px
static const int kPickClickMs = 400;   // press duration, ms

// Docking capture (Game::updateDocking, once per tick at the boundary):
// the two port face-centres must be within kDockCapture, each port axis
// within kDockAlign (cos of the max misalignment) of the line between the
// ports, and the port points' relative speed under kDockMaxV at capture.
// The hulls are 0.1 m inflated (0.2 m contact gap), so a slow aligned
// approach locks before the hulls touch; a fast one bounces instead.
static const double kDockCapture = 1.5;   // m, port face-centre distance
static const double kDockAlign   = 0.966; // cos(15 deg) axis misalignment
static const double kDockMaxV    = 2.0;   // m/s relative speed at capture

struct ToastMsg {
    std::string text;
    double born;   // wall-clock seconds (SDL_GetTicks() * 0.001)
};

// One open part window: a part the player right-clicked in the 3D view.
// Each entry is a plain imgui window (gameui.cpp draws one per entry,
// closable -- several can be open at once, e.g. two tanks for a fuel
// transfer). Closing the window drops the entry; removing the ship drops
// its entries (dropPartWindowsFor, or they dangle).
struct PartSel {
    Vehicle *ship = nullptr;
    size_t part = 0;    // index into ship->parts
    double t = 0;       // sim seconds at selection
    glm::dvec3 point;   // hit point, ship frame
    int mx = 0, my = 0; // mouse at pick, window pixels (window placement)
    bool placed = false;  // first window placement done
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

/* The VAB editor's session state: the physics-free build tree, the LAUNCH
   config, the hover/selection, the placement ghost, the symmetry + snap
   modifiers, the fuel-link authoring and the detached subassemblies. Was ~30
   loose vab_* fields on Game; grouped so the editor's state has one home
   (Game::vab) and the scene work can hand it over wholesale.

   All of it is EDITOR SESSION state, not save state: the build tree
   round-trips through the ship-def files (vabSave / vabLoad), and the
   subassemblies deliberately outlive the build they came from. The camera
   parked across a VAB session is NOT here -- that is transition state, and it
   rides on the scene stack's frame (scene.h SceneFrame). */
struct VabState {
    /* The physics-free build tree the editor edits (shipdef.h BuildShip).
       Empty unless --vab loaded a ship (or the editor started one). Poses are
       in the build ship's own frame S; the VAB scene draws it and re-aims the
       orbit camera at `center`. */
    BuildShip build;
    glm::dvec3 center = glm::dvec3(0.0);   // bbox center of the parts (S frame)

    /* LAUNCH config (the VAB top bar's body/scenario dropdowns). Names, not
       pointers, so they survive and are easy to inspect; vabOpen seeds the
       body to g.home and the scenario to "pad". vabLaunch resolves them
       (g.sys.find / scenario_by_name), falling back to those defaults. */
    std::string bodyName;      // body to launch from ("" -> g.home)
    std::string scenarioName;  // scenario to launch ("" -> "pad")

    int hover = -1;        // build-part index under the mouse; -1 = none
    int selected = -1;     // build-part index selected (click); -1 = none
    std::string armed;     // catalog part name armed from the palette ("" = none)
    int hoverNode = -1;    // stack-node index on the hover parent under the mouse
    int hoverParent = -1;  // build-part index the ghost would attach to

    // --- the placement ghost (the armed part / subassembly's preview) ------
    bool ghostValid = false;
    bool ghostSurface = false;              // ghost = surface attach (vs stack node)
    glm::dvec3 ghostPos;                    // ghost pose (S frame) when valid
    glm::dmat3 ghostRot;
    glm::dvec3 ghostPoint, ghostNormal;     // parent-local surface contact
    std::string ghostParentNode, ghostChildNode;   // stack ghost's mated ids
    /* Pending roll for the armed part (deg): Q/E spin it about the attach
       axis while the ghost previews (stack edge: the mating axis; surface:
       the contact normal). Stored on the placed part's angle/roll, then
       reset to 0. */
    double ghostRoll = 0.0;
    double ghostRollUsed = 0.0;  // the effective (snap-rounded) roll the
                                 // current ghost solves + placement stores
    bool ghostRoot = false;      // the ghost is the ROOT of an empty build
                                 // (placed at the S origin by a plain click)
    int ghostAssembly = -1;      // the subassembly the current ghost previews
                                 // (-1 = the armed catalog part)
    std::vector<SymClone> ghostClones;   // the extra symmetric ghosts
                                         // (radialSymmetryClones output)

    // --- placement modifiers ---------------------------------------------
    int symmetry = 1;   // radial copies for SURFACE placing (1 = single,
                        // up to 8): clones ring the hovered parent's axis
    bool snapLen = true;   // distance snap: the contact's height along
                           // the parent axis (10 cm grid)
    bool snapAng = true;   // angle snap: the contact's clock angle + the
                           // part roll (10 deg grid)
    // holding Alt bypasses BOTH snaps while pressed

    /* Fuel-link authoring (the VAB window's "Add fuel link"): link mode
       arms a two-click pick -- the source part, then the destination --
       which appends a BuildShip::FuelLink (fuel flows from -> to). */
    bool linkMode = false;
    std::string linkFromId;   // the clicked source ("" = not picked yet)
    int linkSel = -1;         // selected fuel-link index (-1 = none)

    /* Detached subtrees (the VAB window's Subassemblies list): session
       editor state -- NOT part of the ship file, and they outlive the
       build they came from (usable across ships). Arming one makes the
       placement ghost solve its ROOT like any part; placing grafts a
       COPY and does not consume the entry (copy-paste). */
    struct Subassembly {
        std::string name;   // display: "<ship> > <root part id>"
        BuildShip ship;     // its own tree, root at its own frame's identity
    };
    std::vector<Subassembly> subassemblies;
    int armedAsm = -1;       // armed subassembly (exclusive with `armed`)

    bool lmbPrev = false;    // LMB edge detect for click-to-place
};


struct Game {
    // --- borrowed subsystems (main creates + deletes) ---------------------
    Renderer &display;
    PostFX *postfx;
    Ships &ships;   // the ship builder (ships.h); the ships themselves live
                    // in the bodies' lists (TerrainBody::ships)
    System &sys;
    TerrainBody *sun;
    TerrainBody *home;
    GameArgs &args;
    Uint32 sim_win_id;
    Uint32 loop_start_ms = 0;   // set once the main loop is about to start

    // --- cameras -----------------------------------------------------------
    Camera *camera = nullptr;   // one object: orbit + free, `camera->mode` picks
    int cam_speed = 1;
    // Camera shake at high acceleration (render.cpp, --cam-shake scale):
    // the smoothed jitter applied to the chase cam while the ship's
    // proper acceleration (thrust + aero over mass) is high. Two state
    // vectors -- a translation offset (m) and a basis wobble (rad) --
    // each low-passed toward fresh random targets per frame, so the
    // rumble is correlated (a shake) instead of a per-frame strobe, and
    // both decay to zero when the engine goes quiet. shake_last_ms is
    // the previous frame's wall clock, for the fps-independent low-pass
    // time constant.
    glm::dvec3 shake_off = glm::dvec3(0.0);
    glm::dvec3 shake_ang = glm::dvec3(0.0);
    Uint32 shake_last_ms = 0;

    // --- scene + VAB editor state ------------------------------------------
    /* The scene stack: back() is the live scene, the frames below are
       suspended. Seeded with [Flight] at boot and never empty -- see scene.h
       for the transitions and why this replaced a single Scene field. Each
       frame carries the camera pose to hand back when it is popped. */
    std::vector<SceneFrame> sceneStack;
    /* The VAB editor's session state (VabState, above): `vab.build` is the
       physics-free tree, plus the hover / ghost / snap / link / subassembly
       state that goes with it. */
    VabState vab;

    /* One-shot headless test hooks (the cli.h --vab-* options). The timings
       are copied from GameArgs at boot so the code that FIRES them can live
       next to the code it drives -- the place hook inside vabUpdate, the three
       transition hooks in vabFireHooks -- instead of in main's loop, and so
       both keep the uniform Game&-only signature the scene table needs.
       A negative time means "never"; each `fired` latches so a hook fires at
       most once per run. */
    struct VabHooks {
        int placeMs = -1, loadMs = -1, launchMs = -1, closeMs = -1;
        std::string loadPath;
        bool placeFired = false, loadFired = false, launchFired = false,
             closeFired = false;
    };
    VabHooks vabHooks;

    // --new-game: the headless hook for the title screen's New Game button
    // (Game::newGame). Same shape as the VAB hooks: a loop time, a latch.
    int newGameMs = -1;
    bool newGameFired = false;

    // --reload DIR / --reload-at MS: the headless runtime-load hook, i.e. the
    // Save/Load window's Load button without the click. It is the only
    // automated cover for load_game running against a LIVE game (the --load
    // boot path starts from nothing, so it cannot show a load preserving one).
    std::string reloadDir;
    int reloadMs = -1;
    bool reloadFired = false;

    // --quit-title MS: the headless hook for the flight pause menu's "Quit to
    // title" (Game::quitToTitle). The only automated cover for unloadGame
    // tearing down a LIVE fleet and landing on the title screen.
    int quitTitleMs = -1;
    bool quitTitleFired = false;

    // --space-center MS: the headless hook for the flight pause menu's "Space
    // Center" (pushes SceneId::SpaceCenter over the running flight). Mirrors
    // --quit-title; the only automated way into the hub.
    int spaceCenterMs = -1;
    bool spaceCenterFired = false;

    // --tracking MS: the headless hook for the Space Center hub's "Tracking
    // Station" (pushes SceneId::TrackingStation). Mirrors --space-center.
    int trackingMs = -1;
    bool trackingFired = false;

    // --- the clock ----------------------------------------------------------
    int time_accel = 1;
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
    // Physics substeps executed by tick() (the while(accumulator>=dt) loop).
    // The --perf breakdown (main.cpp) reads and resets this once per frame.
    long long phys_steps = 0;

    // --- background jobs (job.h) --------------------------------------------
    // Long computations run off the main thread so the frame stays
    // responsive: the porkchop grid, the surface map, and terrain patch
    // subdivision (GeoPatch::requestSubdivide). The main loop calls
    // jobs.poll() once per frame, which runs the finished jobs'
    // main-thread continuations (which publish the result into game
    // state). Each job's own window shows its "working on it" state
    // (e.g. the Porkchop's "sweeping ..."), so there
    // is no global job label here.
    JobRunner jobs;

    // --- the wall-clock log gates (--orbit-interval; tick.cpp + xfer-log) ---
    const Uint32 orbit_log_interval_ms = (Uint32)(args.orbit_interval * 1000.0);
    Uint32 orbit_log_last_ms = 0;
    /* Separate timestamp: the two logs share --orbit-interval but must not
       share the "last fired" time, or the earlier block in the loop always
       wins and the other never fires (and one alone spews every tick). */
    Uint32 dbg_log_last_ms = 0;
    /* Same gate, independent clock (the --att-log cadence is --orbit-interval). */
    Uint32 att_log_last_ms = 0;
    /* Same gate, independent clock (--shake-log: the cam-shake state). */
    Uint32 shake_log_last_ms = 0;
    /* Same gate, independent clock (--eva-log: the kerbal's mode/pos/vel). */
    Uint32 eva_log_last_ms = 0;
    /* Same gate, independent clock (--drag-log: the active ship's drag). */
    Uint32 drag_log_last_ms = 0;

    // --- input / selection state -------------------------------------------
    bool running = true;
    bool rmbCam = false;            // RMB held over 3D: camera look
    // The in-progress RMB gesture (events.cpp): down position + time and
    // the motion accumulated while held. At release, a press with less
    // than kPickClickPx of motion under kPickClickMs is a CLICK (it
    // picks a part, pickAt); otherwise it was the camera drag.
    int rmbDownX = 0, rmbDownY = 0;
    Uint32 rmbDownMs = 0;
    int rmbMoved = 0;               // |xrel| + |yrel| while held
    bool poly_mode = false;         // F11 wireframe
    bool screenshot_requested = false;
    bool porkchop_compute_requested = false;  // P: one-shot compute the plot
    bool surfmap_compute_requested = false;   // M: one-shot compute the map
    // The player-controlled ship (a pointer: the ships live in the bodies'
    // lists -- TerrainBody::ships -- so an index into a flat fleet is no
    // longer a thing).
    Vehicle *ship = nullptr;
    // Crew (src/eva.h, the transitions in game.cpp): kerbals start ABOARD
    // the starting ships' capsules (ships::spawn_crew, on Vehicle::crew).
    // V EVA's one out of the active ship (kerbalEVA) and hands control to
    // it; the part window boards a nearby free kerbal back in
    // (kerbalBoard). `kerbal` is the kerbal the player most recently
    // EVA'd (for the V toggle-back); `lastShip` is the ship they came
    // from.
    Kerbal *kerbal = nullptr;
    Vehicle *lastShip = nullptr;

    // --- part windows (pickAt opens one per right-clicked part) -----------
    // Drawn by gameui.cpp (drawPartWindows); one plain imgui window per
    // entry, so several can be open at once.
    std::vector<PartSel> part_sels;

    // --- render resources (render.cpp draws with them) ---------------------
    // Handed over here once they exist. The FILE assets (the shaders, the
    // plume mesh/texture, the billboard icons) are registry-owned
    // (get_*, shared); the billboards' quads are freed with the billboards
    // (the teardown at the end of main).
    Skybox *skybox = nullptr;
    Shader *skyboxshader = nullptr;
    Shader *lineshader = nullptr;
    Shader *partsshader = nullptr;
    Mesh *engine_plume_mesh = nullptr;
    Texture *engine_plume_texture = nullptr;
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
    Billboard *relvel_indicator = nullptr;      // you − target (prograde icon)
    Billboard *relvel_retro_indicator = nullptr; // target − you (retrograde icon)

    // --- the draw toggles (the Settings window writes, render.cpp reads) --
    bool physics_debug_drawing = false;
    bool world_drawing = true;
    bool draw_starfield = true;
    bool draw_skylines = false;

    // --- control-axis flips (the Settings window writes, tick.cpp reads) --
    // Each inverts one manual attitude axis away from the default. The
    // default baseline (see tick.cpp) already bakes in the preferred
    // orientation -- viewed from the front the ship's left/right are
    // mirrored, so yaw and roll are pre-flipped to respond in your screen
    // direction (pitch is not mirrored). All false = the default; set one
    // to invert that axis (Settings -> Controls).
    bool flip_pitch = false;
    bool flip_yaw = false;
    bool flip_roll = false;

    // --- the key map (keys.h) --------------------------------------------
    // The rebindable key bindings. Default-constructed to the game's default
    // key assignments; load_settings() merges settings.json over it, and the
    // Controls window edits it live. events.cpp (one-shot), tick.cpp and
    // eva.cpp (held) all read through this table.
    KeyBindings binds;
    // While the Controls window is capturing a new binding: the Slot index
    // being rebound (>=0), or -1 when not capturing. events.cpp swallows
    // the next non-modifier key-down (the new binding) and sets it back to -1.
    int rebind_capture_slot = -1;
    // Thrust latch (events.cpp: the ThrustLatch slot toggles it, a plain
    // Thrust press clears it). While true, tick.cpp keeps commanding the
    // active ship's thrust each tick even with the thrust key released, so
    // the engines stay lit until it is undone.
    bool thrust_latched = false;

    // --- the active ship's per-frame state (render.cpp writes it) ----------
    ShipView view;

    // Per-frame timing samples for the Telemetry window. main.cpp pushes
    // every frame (always, not gated on --perf, so the window can show them);
    // the --perf console breakdown reads the same per-frame values. x-axis is
    // wall-clock seconds since loop start (frame timings are real-time, not
    // sim time); y-axis is ms/frame.
    TimeSeries perf_events;
    TimeSeries perf_logic;
    TimeSeries perf_jobs;
    TimeSeries perf_render;
    TimeSeries perf_present;
    // Which series each of the 4 Telemetry cells is showing (0..6, see the
    // kSeries table in gameui.cpp). Defaults to the first four.
    int telemetry_sel[4] = {0, 1, 2, 3};

    // --- orbit camera focus targets (the ship + every body; G cycles) ------
    struct FocusTarget { const char *name; TerrainBody *body; };
    std::vector<FocusTarget> focusTargets;
    int focusBody = 0;             // index into focusTargets

    /* TAB hides the chrome for a clean screenshot. The window table itself
       -- names, layout, roles, and which scene owns which -- lives in
       uiwins.h; it used to be 19 loose option fields here plus a parallel
       registry that copied them and could silently drift. */
    bool ui_visible = true;

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

    // --- Surface Map state (gameui.cpp draws it; surfmap.cpp fills it) ---
    // The mapped body: the combo pick; null = the active ship's parent
    // (the body the ship is orbiting / landed on), else the system home.
    TerrainBody *surfmap_body = nullptr;
    bool surfmap_shade = true;    // bake the terminator (the "Sun shading" box)
    // The last computed map (surfmapCompute publishes it atomically; the
    // window re-uploads the texture on surfmap_rev changes).
    std::vector<unsigned char> surfmap_px;  // RGBA8, w*h
    int surfmap_w = 0;
    int surfmap_h = 0;
    std::string surfmap_body_name;
    double surfmap_computed_at = -1.0;  // sim seconds of the last compute
    bool surfmap_valid = false;
    int surfmap_rev = 0;
    // Sweep jobs posted but not yet landed (surfmapCompute posts one; the
    // main-thread continuation lands it): the window shows "mapping ..."
    // and keeps its buttons disabled until the new map replaces the old
    // one (the same pc_in_flight pattern as the Porkchop grid).
    int surfmap_in_flight = 0;

    // --- control transitions (events + the SHIPS window + selftest) --------
    // V: toggle EVA (src/eva.h) -- from a ship, EVA one of its aboard kerbals
    // and take control; from the kerbal, hand control back to the last ship.
    void toggle_eva();
    // Crew transitions (the capsule part window buttons + the V key):
    //   kerbalEVA    take `k` out of its capsule -- move its mass off the
    //                capsule, un-park its body beside the capsule (it joins
    //                the ship's SoI body's ship list), and hand the player
    //                control of it.
    //   kerbalBoard  put a free kerbal `k` into the capsule (ship, part) --
    //                move its mass onto the capsule, park its body inside,
    //                set its aboard state (it moves to ship->crew). Refuses
    //                a full capsule.
    void kerbalEVA(Kerbal *k);
    void kerbalBoard(Kerbal *k, Vehicle *ship, size_t part);
    // World (ship-frame) position of a focus target, to point the orbit
    // camera at it.
    glm::dvec3 focusWorldPos(int i) const;
    // TAB: hide / restore the live scene's Persistent windows (the pause
    // menu's "Toggle windows" button calls this too).
    void toggle_windows();
    // Rebuild the imgui style from the Settings state (theme, DPI scale,
    // rounding, transparency).
    void apply_ui_style();
    // Settings persistence (settings.h): the window's "Save" button writes
    // the current Settings state to ./settings.json; startup (main.cpp)
    // restores it in two phases, split by what must exist to apply it --
    // the args fields before the Renderer (the display mode/size + the
    // MSAA count are fixed at window creation) and the Game + PostFX
    // fields once the Game exists. A field the CLI set explicitly
    // (args.cli_given) beats the file, in both phases.
    bool save_settings();
    void load_settings();
    // Take control of `v` (release + park the current one, recenter the
    // orbit camera, drop rails warp).
    void select_ship(Vehicle *v);
    /* Start a fresh game from the title screen: the default vessel on the home
       body's pad, then hand over to Flight. False (plus a toast) if a game is
       already running or the def fails to build. This is the runtime twin of
       main's CLI boot path; the two should merge into one startGame()
       (reports/ui-scenes2026_09_17 stage 4), which is why it stays small
       instead of growing fleet/scenario options of its own. */
    bool newGame();
    /* Settle a freshly built fleet into the world: apply every ship's
       scenario (which is what positions them), then park on rails every ship
       except `active`. Does NOT make `active` the player's ship -- the caller
       does, because the two callers need different things: main assigns
       Game::ship directly at boot (the camera does not exist yet and there is
       no previous ship to hand off from), while newGame goes through
       select_ship so the camera follows.

       --load skips all of this: a save records each ship's live-or-railed
       state and the clock, so re-scenarioing would move them. */
    void settleFleet(Vehicle *active);
    /* Load a save over the running game, then move to the scene the result
       implies: Flight when it has a vessel, Title when it does not. False if
       the load was refused, in which case the running game is untouched --
       load_game reads and builds everything before it clears the fleet.
       Shared by the Save/Load window and the --reload hook, so the headless
       path exercises the real one rather than a parallel implementation. */
    bool loadFrom(const std::string &dir);
    /* Tear the running game down to the shipless-boot state: delete the fleet,
       drop part_sels and the active ship/kerbal/lastShip refs, and re-aim the
       camera at home (orbit view). ~Vehicle does the physics/weld/crew cleanup
       per ship, so walking the bodies' ship lists is the whole teardown. No
       job drain: no background continuation dereferences the fleet. Leaves the
       game exactly as a no-vessel boot does. */
    void unloadGame();
    /* unloadGame + enterTitle: the flight pause menu's "Quit to title" and the
       --quit-title hook, shared so the headless path exercises the real
       teardown. */
    void quitToTitle();
    // Keep the "ship" focus entry in sync with the active ship and point
    // the camera focus at it -- or at home (the orbit view) when there is
    // none. select_ship and load_game both enter/leave the no-ship state.
    void syncShipFocus();
    // Enter rails warp (park every ship); false + keeps the accel if any
    // ship is not rail-eligible.
    bool enter_rails_warp();
    // Proximity activation: keep ships near the active ship live (in
    // physics) so they can interact, park the rest; wake the active ship and
    // cap the warp on a close approach. Ground/fly radii differ; a ground
    // engage radius of 0 never auto-wakes grounded neighbors.
    void updateProximity();
    // Docking: once per tick at the boundary (after the physics substeps),
    // the active ship's port against every other live ship's port -- close,
    // aligned and slow enough, the two merge (the active ship survives) and
    // the joint is recorded as a seam on the survivor.
    void updateDocking();
    // Undock: split the most recent seam off the active ship -- the other
    // side's subtree is extracted into a new ship (the general
    // Vehicle::extractSubtreeAsShip primitive) and returned to the fleet.
    void undock();
    // Stage: fire the active stage's decouplers -- each one's child-side
    // subtree comes off as a SEPARATE ship (the same extractSubtreeAsShip
    // primitive undock uses, not a delete) and is returned to the fleet, so
    // the dropped stages fly off and keep coasting like KSP. Refuses a stage
    // that still carries a crewed capsule (EVA them out first), wakes a
    // railed ship first, and advances the survivor's stage counter.
    void stage();
    // Remove a ship + its bookkeeping (refuses the last one; hands control
    // off if the active one is removed).
    void remove_ship(Vehicle *v);
    // Push a one-shot on-screen message (printf-style), shown for
    // kToastLife wall-clock seconds (the last kToastVisible stack).
    void toast(const char *fmt, ...);
    // Open (or focus) the part window for (ship, part) -- picking the
    // same part again does not open a second one (pickAt). (mx,my) is
    // the mouse at pick, window pixels; the window opens near it.
    void openPartWindow(Vehicle *ship, size_t part, const glm::dvec3 &point,
                        int mx, int my);
    // Drop every part window of a ship (remove_ship); its entries would
    // dangle the moment the Vehicle is deleted.
    void dropPartWindowsFor(Vehicle *ship);

    Game(Renderer &display, PostFX *postfx, Ships &ships, System &sys,
         TerrainBody *sun, TerrainBody *home, GameArgs &args,
         Uint32 sim_win_id)
        : display(display), postfx(postfx), ships(ships), sys(sys),
          sun(sun), home(home), args(args), sim_win_id(sim_win_id)
    {
        // The stack is never empty: Flight is the floor scene that every
        // excursion returns to. Seeded here rather than in main so no code
        // path can observe it empty (curSceneId reads back() unguarded). A
        // shipless boot replaces it with Title (main.cpp).
        sceneStack.reserve(4);
        sceneStack.push_back(SceneFrame{});   // Flight, no parked camera
        // --surfmap-noshade (CLI) mirrors the Surface Map window's "Sun
        // shading" box. This used to be set in setup_ui_windows(), which the
        // window table replaced -- it is game state, not window layout.
        surfmap_shade = !args.surfmap_noshade;
    }
};

// RMB-click entry point (events.cpp calls it when a short, still RMB
// press lands over the 3D view): pick the part under (px,py), open its
// window, and log the outcome (the [pick] line is what the e2e cases
// assert). Misses log a miss and leave the existing windows alone.
void pickAt(Game &g, int px, int py);

// settings.json launch phase (main.cpp, BEFORE the Renderer is built):
// apply the file's args fields (display mode/size, the MSAA count -- the
// GLX visual is fixed at window creation -- the FOV/terrain/exhaust
// knobs) over the CLI defaults, honoring args.cli_given (the CLI wins).
// No-op when settings.json is absent. The Game + PostFX fields are the
// second phase (Game::load_settings), run once the Game exists.
void load_settings_args(GameArgs &args);

// Crew queries (defined in game.cpp). The aboard crew live on their ship
// (Vehicle::crew), so these read it directly; the free kerbals are the
// isEva ships in the bodies' lists.
//   shipCrew    every kerbal aboard `ship` (any capsule)
//   partCrew    the kerbals sitting in the specific capsule (ship, part)
//   freeKerbals every kerbal not aboard any ship (on EVA)
std::vector<Kerbal *> shipCrew(Vehicle *ship);
std::vector<Kerbal *> partCrew(Vehicle *ship, size_t part);
std::vector<Kerbal *> freeKerbals(System &sys);
