// game.h -- the running game: subsystems (borrowed from main), runtime
// state, and the control transitions.
#pragma once

#include <SDL3/SDL.h>   // Uint32

#include <string>
#include <vector>

#include "audio.h"
#include "camera.h"   // Camera, CameraMode
#include "cli.h"      // GameArgs
#include "display.h"  // Renderer
#include "flightlog.h"
#include "job.h"      // JobRunner
#include "orbit.h"    // OrbitElements
#include "postfx.h"   // PostFX
#include "scene.h"    // SceneId, SceneFrame, Backdrop
#include "science.h"  // Experiment
#include "ships.h"    // Ships
#include "siminput.h" // TimeSeries
#include "system.h"   // System
#include "terrain.h"  // TerrainBody
#include "transferplanner.h"
#include "ui.h"       // ui::Options
#include "eva.h"      // Kerbal
#include "vehicle.h"  // Vehicle
#include "keys.h"     // KeyBindings

// Render resources (render.cpp draws with them; main owns their lifetime).
struct Billboard;
struct Mesh;
struct Shader;
struct Skybox;
struct Texture;

// Rails warp threshold: above this accel every ship coasts on rails and
// the Bullet world is not stepped. 11 == first accel above 10.
static const int kRailsWarp = 11;

// Toast lifetimes are wall-clock (sim time is paused or warped).
static const double kToastLife = 3.0;
static const int kToastVisible = 3;

// RMB click (pick a part) vs RMB drag (camera look).
static const int kPickClickPx = 6;
static const int kPickClickMs = 400;

// Docking capture thresholds (Game::updateDocking). Hulls are 0.1 m
// inflated, so a slow aligned approach locks before the hulls touch.
static const double kDockCapture = 1.5;   // m, port face-centre distance
static const double kDockAlign   = 0.966; // cos(15 deg) axis misalignment
static const double kDockMaxV    = 2.0;   // m/s relative speed at capture

struct ToastMsg {
    std::string text;
    double born;   // wall-clock seconds (SDL_GetTicks() * 0.001)
};

// One open part window (right-clicked part). Closing drops the entry;
// dropPartWindowsFor when the ship is removed.
struct PartSel {
    Vehicle *ship = nullptr;
    size_t part = 0;    // index into ship->parts
    double t = 0;       // sim seconds at selection
    glm::dvec3 point;   // hit point, ship frame
    int mx = 0, my = 0; // mouse at pick, window pixels (window placement)
    bool placed = false;  // first window placement done
};

// The active ship's per-frame state (render frame), written by render.cpp
// and read by the UI readouts and the transfer planner.
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

/* The VAB editor's session state (Game::vab). EDITOR SESSION state, not
   save state: the build tree round-trips through the ship-def files. */
struct VabState {
    // The physics-free build tree (shipdef.h BuildShip).
    BuildShip build;
    glm::dvec3 center = glm::dvec3(0.0);   // bbox center of the parts (S frame)

    // LAUNCH config. Names, not pointers, so they survive and are easy to
    // inspect; "" falls back to g.home / "pad".
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
    // Pending roll for the armed part (deg); stored on the placed part then reset.
    double ghostRoll = 0.0;
    double ghostRollUsed = 0.0;  // the effective (snap-rounded) roll the
                                 // current ghost solves + placement stores
    bool ghostRoot = false;      // the ghost is the ROOT of an empty build
    int ghostAssembly = -1;      // the subassembly the current ghost previews
                                 // (-1 = the armed catalog part)
    std::vector<SymClone> ghostClones;   // the extra symmetric ghosts

    // --- placement modifiers ---------------------------------------------
    int symmetry = 1;   // radial copies for SURFACE placing (1 = single,
                        // up to 8): clones ring the hovered parent's axis
    bool snapLen = true;   // distance snap (10 cm grid)
    bool snapAng = true;   // angle snap (10 deg grid)
    // holding Alt bypasses BOTH snaps while pressed

    // Fuel-link authoring: two-click pick (source, then destination).
    bool linkMode = false;
    std::string linkFromId;   // the clicked source ("" = not picked yet)
    int linkSel = -1;         // selected fuel-link index (-1 = none)

    // Detached subtrees (Subassemblies): session state, NOT part of the
    // ship file; they outlive the build they came from. Placing grafts a COPY.
    struct Subassembly {
        std::string name;   // display: "<ship> > <root part id>"
        BuildShip ship;     // its own tree, root at its own frame's identity
    };
    std::vector<Subassembly> subassemblies;
    int armedAsm = -1;       // armed subassembly (exclusive with `armed`)

    bool lmbPrev = false;    // LMB edge detect for click-to-place
};

// Science identity of a pose (biome + situation). Biome::None when the
// body has no classifiable surface or terrain is not ready; the situation
// is still valid then.
struct PoseSituation {
    Biome biome = Biome::None;
    SciSituation situation = SciSituation::Landed;
};
inline PoseSituation poseSituation(const TerrainBody *body, const glm::vec3 &dir,
                                   double altAsl, bool grounded) {
    const double atmoTop = body->surface.atmosphere.top();
    const SciSituation situation =
        situationFor(grounded, altAsl, atmoTop,
                     orbitCutAlt(body->rot_frame->soi, (double)body->radius,
                                 (double)body->surface.sea_level));
    if(!body->hasClassifiableSurface() || !body->ready) {
        return { Biome::None, situation };
    }
    return { biomeAt(dir, body->params()), situation };
}

struct Game {
    // --- borrowed subsystems (main creates + deletes) ---------------------
    Renderer &display;
    PostFX *postfx;
    Ships &ships;   // the ship builder; the ships themselves live in the
                    // bodies' lists (TerrainBody::ships)
    System &sys;
    TerrainBody *sun;
    TerrainBody *home;
    // Title-screen backdrop body: chosen ONCE per system so the boot can
    // build it synchronously (it is what the first frame shows).
    TerrainBody *titleBody = nullptr;
    GameArgs &args;
    Uint32 sim_win_id;
    Uint32 loop_start_ms = 0;   // set once the main loop is about to start

    // --- the transfer planner (TRANSFER window + porkchop cache) -----------
    // A Game member so the clock hook can reach and invalidate its state.
    TransferPlanner xferPlanner;

    // --- sound (audio.h; a silent no-op without a device) -------------------
    Audio audio;

    // --- cameras -----------------------------------------------------------
    Camera *camera = nullptr;   // orbit + free + first person (`camera->mode`)
    int cam_speed = 1;
    // Chase-cam rumble under high proper acceleration (see render.cpp).
    glm::dvec3 shake_off = glm::dvec3(0.0);
    glm::dvec3 shake_ang = glm::dvec3(0.0);
    Uint32 shake_last_ms = 0;

    // --- scene + VAB editor state ------------------------------------------
    /* The scene stack: back() is the live scene, never empty (see scene.h).
       Each frame carries the camera pose to hand back when it is popped. */
    std::vector<SceneFrame> sceneStack;
    VabState vab;

    /* One-shot headless test hooks (cli.h --vab-* etc). Negative time means
       "never"; each `fired` latches so a hook fires at most once per run. */
    struct VabHooks {
        int placeMs = -1, loadMs = -1, launchMs = -1, closeMs = -1;
        int detachIdx = -1, detachMs = -1;   // --vab-detach part index + time
        int saveMs = -1;
        std::string loadPath, savePath;
        bool placeFired = false, loadFired = false, launchFired = false,
             closeFired = false, detachFired = false, saveFired = false;
    };
    VabHooks vabHooks;

    // --new-game: headless hook for the title screen's New Game button.
    int newGameMs = -1;
    bool newGameFired = false;

    // --reload DIR / --reload-at MS: headless runtime-load hook.
    std::string reloadDir;
    int reloadMs = -1;
    bool reloadFired = false;

    // --quit-title MS: headless "Quit to title" hook.
    int quitTitleMs = -1;
    bool quitTitleFired = false;

    // --space-center MS: headless "Space Center" hook.
    int spaceCenterMs = -1;
    bool spaceCenterFired = false;

    // --recover MS: headless "Recover Vessel" hook.
    int recoverMs = -1;
    bool recoverFired = false;

    // --experiment MS: headless suit-observation hook.
    int experimentMs = -1;
    bool experimentFired = false;

    // --pod-experiment MS: headless science-pod experiment hook.
    int podExperimentMs = -1;
    bool podExperimentFired = false;

    // --eva MS: headless EVA hook.
    int evaMs = -1;
    bool evaFired = false;

    // --take MS: headless take/store dance (move finding to a courier).
    int takeMs = -1;
    bool takeFired = false;

    // --store MS: headless take/store dance (move finding to a container).
    int storeMs = -1;
    bool storeFired = false;

    // --tracking MS: headless Tracking Station hook.
    int trackingMs = -1;
    bool trackingFired = false;

    // --tracking-close MS: headless Tracking Station exit hook.
    int trackingCloseMs = -1;
    bool trackingCloseFired = false;

    // --research MS: headless Research Lab hook.
    int researchMs = -1;
    bool researchFired = false;

    // --research-close MS: headless Research Lab exit hook.
    int researchCloseMs = -1;
    bool researchCloseFired = false;

    // --switch-system FILE / --switch-at MS: headless in-process system swap.
    std::string switchSystemPath;
    int switchSystemMs = -1;
    bool switchSystemFired = false;

    // The system file this game is running (updated by switchSystem). This --
    // not args.system_file -- is what save_game records.
    std::string systemPath;

    // The game (one playthrough) this state belongs to. gameId is the dir
    // under saves/ (<YYYYMMDD_HHMMSS>-<name>, see save.h).
    std::string gameName = "game1";
    std::string gameId;   // "" until New Game Start / a bare boot's first save / a load

    // --- the clock ----------------------------------------------------------
    int time_accel = 1;
    double time = 0;   // the analytic sim clock (s), advanced by the tick

    /* Bodies' orbits/spin are a pure function of `time`. Anything that moves
       the clock OUTSIDE a tick must call these, or a paused game renders the
       epoch the previous state left behind. */
    void syncRails() { sun->frame->UpdateOrbitRails(time); }
    /* Drop every cache stamped against the current sim state. A job captures
       cache_epoch at post time and skips its publish if it has changed. */
    void invalidateClockStampedCaches() {
        xferPlanner.invalidateClockState();
        surfmap_valid = false;
        surfmap_computed_at = -1.0;
        cache_epoch++;
        // In-flight counters are only decremented by their continuations;
        // on the switch path those are discarded (jobs.restart).
        xferPlanner.pc_in_flight = 0;
        surfmap_in_flight = 0;
    }
    /* The clock moves OUTSIDE a tick (a load, the boot --start-time): re-derive
       the bodies and invalidate clock-stamped caches. A system swap calls
       invalidateClockStampedCaches() directly instead. */
    void setTime(double t) {
        time = t;
        syncRails();
        invalidateClockStampedCaches();
    }

    // --- one-shot on-screen messages (gameui.cpp draws the last N) ----------
    std::vector<ToastMsg> toasts;

    // --- the fixed-timestep loop (tick.cpp) ---------------------------------
    double currentTime = 0.001 * (double)(SDL_GetTicks());
    double accumulator = 0.0;
    const double dt = 1.0/50.0;   // TODO explain why 50
    bool redraw = false;         // a frame of logic ran: RENDER should draw
    // Physics substeps executed by tick(); --perf reads/resets once per frame.
    long long phys_steps = 0;

    // --- background jobs (job.h) --------------------------------------------
    // Porkchop, surface map, terrain subdivision. jobs.poll() runs the
    // finished jobs' main-thread continuations.
    JobRunner jobs;

    // --- the wall-clock log gates (--orbit-interval; tick.cpp + xfer-log) ---
    // Each log needs its own "last fired" clock, or only the first one fires.
    const Uint32 orbit_log_interval_ms = (Uint32)(args.orbit_interval * 1000.0);
    Uint32 orbit_log_last_ms = 0;
    Uint32 dbg_log_last_ms = 0;
    Uint32 info_log_last_ms = 0;
    Uint32 att_log_last_ms = 0;
    Uint32 shake_log_last_ms = 0;
    Uint32 eva_log_last_ms = 0;
    Uint32 drag_log_last_ms = 0;
    Uint32 terrain_log_last_ms = 0;

    // --- input / selection state -------------------------------------------
    bool running = true;
    bool rmbCam = false;            // RMB held over 3D: camera look
    // The in-progress RMB gesture (events.cpp): a short still press is a click.
    int rmbDownX = 0, rmbDownY = 0;
    Uint32 rmbDownMs = 0;
    int rmbMoved = 0;               // |xrel| + |yrel| while held
    bool poly_mode = false;         // F11 wireframe
    bool screenshot_requested = false;
    bool porkchop_compute_requested = false;  // P: one-shot compute the plot
    bool surfmap_compute_requested = false;   // M: one-shot compute the map
    // The player-controlled ship (ships live in the bodies' lists).
    Vehicle *ship = nullptr;
    // Crew: `kerbal` is the kerbal the player most recently EVA'd (for the
    // V toggle-back); `lastShip` is the ship they came from.
    Kerbal *kerbal = nullptr;
    Vehicle *lastShip = nullptr;

    // --- part windows (pickAt opens one per right-clicked part) -----------
    std::vector<PartSel> part_sels;

    // --- render resources (render.cpp draws with them) ---------------------
    // FILE assets are registry-owned (get_*, shared); the billboards' quads
    // are freed with the billboards (the teardown at the end of main).
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
    // Each inverts one manual attitude axis away from the default (see tick.cpp).
    bool flip_pitch = false;
    bool flip_yaw = false;
    bool flip_roll = false;

    // --- the key map (keys.h) --------------------------------------------
    KeyBindings binds;
    // While the Controls window is capturing a new binding: the Slot index
    // being rebound (>=0), or -1 when not capturing.
    int rebind_capture_slot = -1;
    // Thrust latch (events.cpp toggles; tick.cpp keeps thrust on while set).
    bool thrust_latched = false;

    // The Flight Summary window payload, written by recoverActive.
    struct FlightSummary {
        std::string shipName;
        FlightLog log;
        double end_t = 0.0;
        int scienceGained = 0;
        int repeatScience = 0;
        std::vector<Experiment> newExperiments;
    };
    FlightSummary flightSummary;

    // --- science (career score + the unique experiments recovered) --------
    Career science;

    // Pre-built Research Lab rows (see researchLabEnter). labRowsVersion is
    // the science.version they were built from; the render rebuilds if it
    // differs, so the cache self-heals if the log changes while live.
    std::vector<LabEntry> labRows;
    std::size_t labRowsVersion = std::size_t(-1);   // no cache built yet

    // Pre-built System Atlas rows (one per body, tree-ordered). Same
    // self-heal: built from science.version, rebuilt on the first render if
    // it has moved (a recovery while the lab was open).
    std::vector<AtlasRow> atlasRows;
    std::size_t atlasRowsVersion = std::size_t(-1);

    // --- the active ship's per-frame state (render.cpp writes it) ----------
    ShipView view;

    // Per-frame timing samples for the Telemetry window (wall-clock x-axis).
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

    // TAB hides the chrome for a clean screenshot. Window table: uiwins.h.
    bool ui_visible = true;

    // The big face (2x the UI font), created by main at ImGui init.
    ImFont *bigger = nullptr;

    // The Settings window state (apply_ui_style() rebuilds the style from it).
    int ui_style = 0;              // 0=dark (imgui default) 1=light 2=classic
    float window_rounding = 0.0f;  // imgui default
    float ui_alpha = 1.0f;         // global imgui alpha (window opacity)
    float ui_scale = 1.0f;         // DPI scale: fonts + style sizes

    // Audio master levels in [0,1] (Settings sliders; applied live).
    float sfx_volume = 1.0f;       // one-shots + the engine loop
    float music_volume = 0.5f;     // ambient music (background, not the star)

    // --- Orbital map state (gameui.cpp draws with them) ---------------------
    float map_scale = 6000.0f;   // meters per pixel
    int map_plane = 0;           // 0 = equatorial, 1 = ecliptic, 2 = orbital
    // Pan offset from the window center, in pixels (P4 navigation).
    ImVec2 map_pan = ImVec2(0.0f, 0.0f);
    // Right-clicking the map cycles chrome: 0 = full window, 1 = bare map,
    // 2 = no window chrome (map floats over the 3D view). 1 and 2 keep pan/zoom.
    int map_mode = 0;

    // --- world-stamped caches ------------------------------------------------
    // Bumped by invalidateClockStampedCaches(); a background job drops its
    // result on landing if it has moved since.
    int cache_epoch = 0;

    // --- Surface Map state (gameui.cpp draws it; surfmap.cpp fills it) ---
    // The mapped body; null = the active ship's parent, else the system home.
    TerrainBody *surfmap_body = nullptr;
    bool surfmap_shade = true;    // bake the terminator (the "Sun shading" box)
    bool surfmap_sea = false;     // paint the sea (the "Ocean" box)
    // The last computed map (surfmapCompute publishes it atomically).
    std::vector<unsigned char> surfmap_px;  // RGBA8, w*h
    int surfmap_w = 0;
    int surfmap_h = 0;
    std::string surfmap_body_name;
    double surfmap_computed_at = -1.0;  // sim seconds of the last compute
    bool surfmap_valid = false;
    int surfmap_rev = 0;
    // Sweep jobs posted but not yet landed (the window shows "mapping ...").
    int surfmap_in_flight = 0;

    // --- control transitions (events + the SHIPS window + selftest) --------
    // V: toggle EVA -- EVA one of the ship's aboard kerbals, or hand control
    // back to the last ship.
    void toggle_eva();
    // Crew transitions. kerbalEVA hands the player control of the kerbal;
    // kerbalBoard parks a free kerbal into a capsule (refuses a full one).
    void kerbalEVA(Kerbal *k);
    void kerbalBoard(Kerbal *k, Vehicle *ship, size_t part);
    // Inventory drop/pickup: dropItem makes a free 1-part ship; pickUpItem
    // re-parents it into `dest`.
    Vehicle *dropItem(Part *item);
    bool pickUpItem(Vehicle *itemShip, Part *dest);
    // World (ship-frame) position of a focus target (for the orbit camera).
    glm::dvec3 focusWorldPos(int i) const;
    // Choose the title backdrop body (stored in titleBody).
    TerrainBody *pickTitleBody();
    // Park the orbit camera on titleBody (no-op until focusTargets is seeded).
    void parkTitleCamera();
    // TAB: hide / restore the live scene's Persistent windows.
    void toggle_windows();
    // Set the live scene's Persistent windows to match ui_visible.
    void apply_ui_visible();
    // A scene entry always shows the UI (a TAB-hide must not carry over).
    void ensure_ui_visible();
    // Rebuild the imgui style from the Settings state.
    void apply_ui_style();
    // Settings persistence (settings.json). CLI beats the file field-by-field
    // (args.cli_given). Two load phases: args before the Renderer, Game+PostFX after.
    bool save_settings();
    void load_settings();
    // Take control of `v` (release + park the current one, recenter the camera).
    void select_ship(Vehicle *v);
    /* Start a fresh game from the title screen: Space Center, no ship, paused.
       False (plus a toast) if a game is already running. */
    bool newGame();
    /* The New Game sheet's Start: switch system if needed, apply difficulty,
       mint the game identity, then newGame(). False if a game is running or
       the switch failed (running world untouched). */
    bool startNewGame(const std::string &name, const std::string &sysPath,
                      float exhaustScale);
    /* Ensure this game has a dir under saves/ (minting one if needed). "" on
       a mint failure (a toast already fired). */
    std::string ensureGameDir();
    /* Settle a freshly built fleet: apply scenarios, park on rails every ship
       except `active`. Does NOT make `active` the player's ship -- the caller
       does (main assigns Game::ship directly; newGame uses select_ship).
       --load skips all of this (a save records live-or-railed state). */
    void settleFleet(Vehicle *active);
    /* Load a save over the running game, then enter Flight (a ship) or the
       Space Center hub (shipless). False if refused: a same-system refusal
       leaves the running game and its scene untouched; a cross-system one
       already switched systems, so it lands on the title. Shared by the
       Save/Load window and the --reload hook. */
    bool loadFrom(const std::string &dir);
    // Quicksave (F5): save into the game dir's next quicksave-NN pool slot.
    void quicksave();
    // Quickload (F9): load the newest quicksave of this game by mtime.
    void quickload();
    /* Ensure the running system matches the save at `dir` (switching if
       different). `switched` is set true iff the switch actually ran (so the
       caller must not run the heavy phase). Shared by loadFrom and boot --load. */
    bool ensureSystemForSave(const std::string &dir, bool *switched = nullptr);
    /* Tear the running game down to the shipless-boot state. No job drain: no
       background continuation dereferences the fleet. */
    void unloadGame();
    // unloadGame + enterTitle: the flight pause menu's "Quit to title".
    void quitToTitle();
    /* In-process system switch: tear down the running system and load a
       different one, landing on the title screen. Transactional (see
       game.cpp). `syncNames` are body NAMES to build synchronously; empty =
       the title backdrop. */
    void switchSystem(const std::string &path,
                      const std::vector<std::string> &syncNames = {});
    // Keep the "ship" focus entry in sync with the active ship (or a random
    // non-star title backdrop when there is none).
    void syncShipFocus();
    // Enter rails warp (park every ship); false + keeps the accel if any ship
    // is not rail-eligible.
    bool enter_rails_warp();
    // Keep ships near the active ship live (in physics), park the rest.
    void updateProximity();
    // Docking: once per tick at the boundary, the active ship's port against
    // every other live ship's port -- close, aligned and slow enough, merge.
    void updateDocking();
    // Undock: split the most recent seam off the active ship into a new ship.
    void undock();
    // Stage: fire the active stage's decouplers; each child-side subtree comes
    // off as a separate ship. Refuses a stage that still carries a crewed
    // capsule (EVA them out first).
    void stage();
    // Remove a ship + its bookkeeping (refuses the last one; hands control off
    // if the active one is removed).
    void remove_ship(Vehicle *v);
    // Recover the active vessel (the successful end of a flight). Refuses
    // unless the ship is grounded on the home body (--recover-anywhere lifts
    // that). Unlike remove_ship this allows the last vessel.
    void recoverActive();
    /* Run an observation experiment with kerbal `k` at its current pose.
       Stores on the kerbal's suit; refuses a duplicate already held. */
    void runExperiment(Kerbal *k);
    /* The situation experiment (identity + provenance) for a part riding
       `poseVehicle`, recorded by `runner`. Shared by runExperiment and
       runPodExperiment. */
    bool situationExperiment(TerrainBody *body, Vehicle *poseVehicle,
                             Part *localPart, const std::string &type,
                             Kerbal *runner, Experiment &out);
    /* Run a science-instrument experiment on part `pod` with aboard kerbal `k`.
       Refuses a duplicate the pod already holds. */
    void runPodExperiment(Part *pod, Kerbal *k);
    /* The take/store dance: move the `which`-th finding held on part `from`
       over to part `to` (must be a transfer destination with a free slot). */
    void moveExperiment(Part *from, Part *to, size_t which);
    /* Is free kerbal `k` close enough to part `part` to interact with it
       (kBoardingRange). `k` must be free (on EVA, not aboard). */
    bool kerbalInRange(Kerbal *k, Part *part);
    // Push a one-shot on-screen message (printf-style).
    void toast(const char *fmt, ...);
    // toast() minus the on-screen part: just the "[toast] ..." console line.
    // Scene transitions use this (the move is visible; the line stays an anchor).
    void toastLog(const char *fmt, ...);
    // Open (or focus) the part window for (ship, part). (mx,my) is the mouse
    // at pick; the window opens near it.
    void openPartWindow(Vehicle *ship, size_t part, const glm::dvec3 &point,
                        int mx, int my);
    // Drop every part window of a ship (its entries would dangle after delete).
    void dropPartWindowsFor(Vehicle *ship);
    // Close the Flight Summary and drop its payload. Every game teardown
    // calls this (scene transitions close nothing).
    void clearFlightSummary();

    Game(Renderer &display, PostFX *postfx, Ships &ships, System &sys,
         TerrainBody *sun, TerrainBody *home, GameArgs &args,
         Uint32 sim_win_id)
        : display(display), postfx(postfx), ships(ships), sys(sys),
          sun(sun), home(home), args(args), sim_win_id(sim_win_id),
          xferPlanner(*this)   // borrows the Game it is a member of
    {
        // The stack is never empty: Flight is the floor scene. Seeded here so
        // no code path can observe it empty (curSceneId reads back() unguarded).
        sceneStack.reserve(4);
        sceneStack.push_back(SceneFrame{});   // Flight, no parked camera
        // --surfmap-noshade (CLI) mirrors the Surface Map "Sun shading" box.
        surfmap_shade = !args.surfmap_noshade;
    }
};

// RMB-click entry point (events.cpp): pick the part under (px,py), open its
// window, and log the outcome (the [pick] line is what the e2e cases assert).
void pickAt(Game &g, int px, int py);

// settings.json launch phase (main.cpp, BEFORE the Renderer is built): apply
// the file's args fields over the CLI defaults. No-op when settings.json is
// absent. See also Game::load_settings (the second phase).
void load_settings_args(GameArgs &args);

// Crew queries (defined in game.cpp). The aboard crew live on their ship
// (Vehicle::crew); the free kerbals are the isEva ships in the bodies' lists.
std::vector<Kerbal *> shipCrew(Vehicle *ship);
std::vector<Kerbal *> partCrew(Part *capPart);
std::vector<Kerbal *> freeKerbals(System &sys);

// Capsule-boarding / take-store reach [m]. One constant so the reach the UI
// shows and the gate it enforces can't drift.
constexpr double kBoardingRange = 10.0;
