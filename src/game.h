// game.h -- the running game: subsystems, runtime state, control transitions.
#pragma once

#include <SDL3/SDL.h>   // Uint32

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "audio.h"
#include "camera.h"   // Camera, CameraMode
#include "cli.h"      // GameArgs
#include "constants.h" // kMapDefaultScale
#include "crew.h"     // CrewRoster (the named crew pool)
#include "display.h"  // Renderer
#include "flightlog.h"
#include "job.h"      // JobRunner
#include "orbit.h"    // OrbitElements, RefPlane, PlaneAngles
#include "postfx.h"   // PostFX
#include "scene.h"    // SceneId, SceneFrame, Backdrop
#include "science.h"  // Experiment
#include "shipdef.h"  // PartsCatalog (the part types the world is built from)
#include "siminput.h" // TimeSeries
#include "staging.h"  // StageRow (the Staging window's cached table)
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

// Above this accel every ship coasts on rails and the Bullet world is not stepped.
// 11 == first accel above 10.
static const int kRailsWarp = 11;

/* Top of the decimal warp ladder (1, 10, then rails at 100 ... kMaxWarp): the last
   WarpUp allowed. The cost is SoI resolution, not accuracy: railsTick tests the SoI
   only at the END of a tick, so a tick longer than a body's sphere flies past it. */
static const int kMaxWarp = 10000000;

// Toast lifetimes are wall-clock (sim time is paused or warped).
static const double kToastLife = 3.0;
static const int kToastVisible = 3;

// RMB click (pick a part) vs RMB drag (camera look).
static const int kPickClickPx = 6;
static const int kPickClickMs = 400;

// Docking capture thresholds. Hulls are 0.1 m inflated, so a slow aligned
// approach locks before the hulls touch.
static const double kDockCapture = 1.5;   // m, port face-centre distance
static const double kDockAlign   = 0.966; // cos(15 deg) axis misalignment
static const double kDockMaxV    = 2.0;   // m/s relative speed at capture

struct ToastMsg {
    std::string text;
    double born;   // wall-clock seconds (SDL_GetTicks() * 0.001)
};

// One open part window (right-clicked part); dropPartWindowsFor drops them
// with the ship.
struct PartSel {
    Vehicle *ship = nullptr;
    size_t part = 0;    // index into ship->parts
    double t = 0;       // sim seconds at selection
    glm::dvec3 point;   // hit point, ship frame
    int mx = 0, my = 0; // mouse at pick, window pixels (window placement)
    /* Window layout, written by drawPartWindows. Placement is NOT one-off: the
       size is unknown until imgui finishes auto-fitting (two frames), and the
       window is AlwaysAutoResize so its size follows its content -- Board /
       Take / Store rows appear when a crew member walks into reach -- so a
       window placed clear of its neighbour can grow into it. ask* is where WE
       asked for it, so any other Pos is the player's own drag. winW/winH are
       imgui's measured size, 0 until known. */
    bool placed = false;
    float askX = 0, askY = 0;
    float winW = 0, winH = 0;
    bool userMoved = false;
};

/* The active ship's per-frame state (render frame): render.cpp writes it, the UI
   readouts and the transfer planner read it. */
struct ShipView {
    /* Which ship this describes (nullptr = none). The state pass takes it once per
       frame, but the imgui pass changes the active ship itself, so later windows
       read a stale snapshot; on a shipless boot it is never taken at all (#235). */
    Vehicle *ship = nullptr;
    // render-frame (ship->frame) state; an unfilled snapshot reads zeros
    glm::dvec3 pos = glm::dvec3(0.0);
    glm::dvec3 vel = glm::dvec3(0.0);
    // the surface frame (the body's rotating frame, when the ship is on one)
    glm::dvec3 surf_pos = glm::dvec3(0.0);
    glm::dvec3 surf_vel = glm::dvec3(0.0);
    // the orbit conic, in the ship's non-rotating (inertial) frame
    glm::dvec3 orbit_pos = glm::dvec3(0.0);
    glm::dvec3 orbit_vel = glm::dvec3(0.0);
    /* orbit_vel brought back into the ship's OWN frame. The flight markers
       (prograde / retrograde / normal) and the prograde angle are properties of
       the conic, so they read this rather than vel: inside the rotating surface
       frame vel is the ground-relative velocity, missing the ground's own
       motion, which tilts the markers off the plane the orbital-plane overlay
       draws. Radial needs no such lift -- pos is already this frame's. */
    glm::dvec3 marker_vel = glm::dvec3(0.0);
    OrbitElements o;
    // Inc/LAN/LPe measured in the plane the orbit map shows (Game::map.plane), so
    // the numbers describe the picture (#171). The plane itself is kept as well:
    // the 3D orbital-plane overlay draws THIS one, not a second opinion.
    PlaneAngles plane;
    RefPlane ref_plane;
    // The transfer target's orbit normal about the same focus body, in the same
    // inertial frame -- what the map's "Target orbit" plane slot measures
    // against. Zero + target_plane_ok false when no target is picked (or it is
    // not in orbit), which is also what uiRefPlane expects.
    glm::dvec3 target_h_hat = glm::dvec3(0.0);
    bool target_plane_ok = false;
    double distance = 0;  // |orbit_pos|
    double speed = 0;     // |orbit_vel|
    double mu = 0;        // the parent body's mu (the conic's)
    // attitude readouts (render frame)
    glm::dvec3 up = glm::dvec3(0.0);
    glm::dvec3 facing = glm::dvec3(0.0);
    glm::dvec3 other = glm::dvec3(0.0);
    glm::dvec3 facing_dir = glm::dvec3(0.0);
    glm::dvec3 vel_dir = glm::dvec3(0.0);
    // surface + orientation scalars (the SURFACE window)
    double ver_speed = 0;
    double hor_speed2 = 0;
    double latitude = 0;
    double longitude = 0;
    double heading = 0;
    double pitch = 0;
    double roll = 0;
    // telemetry, sampled once per drawn frame
    TimeSeries energy_series;
    TimeSeries angmom_series;
};

/* Flight HUD SESSION state (Game::hud), not save state: the active ship's
   per-frame snapshot (ShipView) that render.cpp writes and the readouts +
   transfer planner read, plus the Telemetry window's series and per-cell
   selections. */
struct HudState {
    ShipView view;

    // Per-frame timing samples for the Telemetry window (wall-clock x-axis).
    TimeSeries perf_events;
    TimeSeries perf_logic;
    TimeSeries perf_jobs;
    TimeSeries perf_refresh;   // the 3D state pass (render.cpp refreshWorld)
    TimeSeries perf_render;    // the paint pass + imgui (GL command issue)
    TimeSeries perf_present;
    // Which series each Telemetry cell shows (0..7, see kSeries in gameui.cpp).
    int telemetry_sel[4] = {0, 1, 2, 3};
};

/* VAB editor SESSION state (Game::vab), not save state: the build tree round-trips
   through the ship-def files. */
struct VabState {
    BuildShip build;                       // physics-free (shipdef.h)
    glm::dvec3 center = glm::dvec3(0.0);   // bbox center of the parts (S frame)

    // Names, not pointers, so they survive; "" falls back to g.home / "pad" / the
    // default launch site.
    std::string bodyName;
    std::string scenarioName;
    std::string siteName;      // launch building id ("" -> first VAB_LAUNCH_PAD)

    // Build-part indices; -1 = none.
    int hover = -1;
    int selected = -1;
    std::string armed;     // catalog part name armed from the palette ("" = none)
    int hoverNode = -1;    // stack-node index on the hover parent
    int hoverParent = -1;  // the build-part the ghost would attach to

    // --- the placement ghost (the armed part / subassembly's preview) ------
    bool ghostValid = false;
    bool ghostSurface = false;              // surface attach (vs stack node)
    glm::dvec3 ghostPos;                    // ghost pose (S frame) when valid
    glm::dmat3 ghostRot;
    glm::dvec3 ghostPoint, ghostNormal;     // parent-local surface contact
    std::string ghostParentNode, ghostChildNode;   // stack ghost's mated ids
    // Pending roll for the armed part (deg); stored on the placed part then reset.
    double ghostRoll = 0.0;
    double ghostRollUsed = 0.0;  // the effective (snap-rounded) roll the
                                 // current ghost solves + placement stores
    bool ghostRoot = false;      // the ghost is the ROOT of an empty build
    int ghostAssembly = -1;      // subassembly previewed (-1 = the armed part)
    std::vector<SymClone> ghostClones;   // the extra symmetric ghosts

    // --- placement modifiers ---------------------------------------------
    int symmetry = 1;   // radial copies for SURFACE placing (1..8)
    bool snapLen = true;   // distance snap (10 cm grid)
    bool snapAng = true;   // angle snap (10 deg grid)
    // holding Alt bypasses BOTH snaps while pressed

    // Fuel-link authoring: two-click pick (source, then destination).
    bool linkMode = false;
    std::string linkFromId;   // the clicked source ("" = not picked yet)
    int linkSel = -1;         // selected fuel-link index (-1 = none)

    // NOT part of the ship file; they outlive the build they came from. Placing
    // grafts a COPY.
    struct Subassembly {
        std::string name;   // display: "<ship> > <root part id>"
        BuildShip ship;     // its own tree, root at its own frame's identity
    };
    std::vector<Subassembly> subassemblies;
    int armedAsm = -1;       // armed subassembly (exclusive with `armed`)

    /* --- seat assignments (the seat window: right-click a capsule) ---------
       Who sits where, chosen in the editor instead of drawn FIFO off the pad at
       launch. Keyed by BuildPart::id, NOT the part index: the tree renumbers
       under the player's feet. Session-only like the rest of VabState -- a ship
       file on disk holds no people (crew stands on the pad, crew.h).

       Launch rule: ANY entry here means the player chose the crew, so nothing
       auto-fills; an unassigned seat flies empty.

       profileId 0 is a CHOICE, not an absence: "nobody in this seat". Emptying
       a seat keeps the entry at 0 instead of erasing it, so a player who empties
       every seat launches unmanned rather than hearing "I never said anything"
       and getting the pad's line (issue #287). An entry whose part leaves the
       build is dropped -- and the player is told, because the fallback to the
       line is otherwise invisible (issue #288). */
    struct SeatAssign {
        std::string partId;
        int slot = 0;            // 0-based seat within the capsule
        uint64_t profileId = 0;  // roster id worn by the standing suit; 0 = empty
    };
    std::vector<SeatAssign> seatAssigns;

    // The seat window: the build part it shows ("" = closed). An ID, not an
    // index -- the tree renumbers under the player's feet.
    std::string seatPartId;
    int seatMx = 0, seatMy = 0;   // where it was opened (first-draw placement)
    bool seatPlaced = false;

    /* The Staging window's table, cached against a content hash of the build
       (#293). The window redraws every frame; the build only changes when the
       player edits it, and a burn simulation costs 84-473 us on a real ship
       where revalidating the hash costs about a microsecond. 0 is "no cache":
       a real signature is FNV-1a over the whole build, so colliding with 0 is
       not a practical concern, and a stale entry could only ever show a table
       one edit behind -- never a wrong launch. */
    uint64_t stagingSig = 0;
    std::vector<StageRow> stagingRows;
};

/* Orbital map SESSION state (Game::map), not save state: a fresh boot starts
   on the defaults. map.plane is NOT just chrome: ShipView's Inc/LAN/LPe are
   measured in it (see ShipView), so the readouts follow the picture. */
struct MapState {
    float scale = kMapDefaultScale;   // meters per pixel
    // RefPlaneMode (frame.h). ImGui::Combo needs an int*.
    int plane = kRefEquator;
    // Pan offset from the window center, in pixels (P4 navigation).
    ImVec2 pan = ImVec2(0.0f, 0.0f);
    // Right-click cycles chrome: 0 = full window, 1 = bare map, 2 = no chrome.
    int mode = 0;
};

/* Surface Map SESSION state (Game::surfmap), not save state: the pixels are
   re-baked from the body on demand. valid / computed_at / in_flight are also
   the clock-stamped cache's: invalidateClockStampedCaches() resets them, so
   the window reads state the clock owns. */
struct SurfmapState {
    // null = the active ship's parent, else the system home.
    TerrainBody *body = nullptr;
    bool shade = true;    // bake the terminator (the "Sun shading" box)
    bool sea = false;     // paint the sea (the "Ocean" box)
    // Overlay-only, so unlike the two above it needs no re-bake.
    bool site_labels = true;
    // The last computed map (surfmapCompute publishes it atomically).
    // px is the unshaded albedo; shade_px is the terminator overlay (black,
    // alpha = 1-shade) drawn over it -- toggling "Sun shading" is a free
    // show/hide, no re-bake.
    std::vector<unsigned char> px;       // RGBA8, w*h
    std::vector<unsigned char> shade_px; // RGBA8 overlay, w*h
    int w = 0;
    int h = 0;
    std::string body_name;
    double computed_at = -1.0;  // sim seconds of the last compute
    bool valid = false;
    int rev = 0;
    // Jobs posted but not landed (the window shows "mapping ...").
    int in_flight = 0;
    // M: one-shot compute request (events.cpp sets, the state pass consumes).
    bool compute_requested = false;
};

// Biome::None when the body has no classifiable surface or terrain is not ready.
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

/* The question card (drawQuestion, gameui.cpp): answered on a LATER frame, so
   onAnswer must capture IDENTITY (a name, an id), never a pointer -- the subject
   can be gone by then. */
struct Question {
    std::string text;
    std::vector<std::string> labels;   // one button each, in order
    std::function<void(int choice)> onAnswer;
};

/* Wall-clock log gates: each throttled log needs its own "last fired" clock,
   or only the first one fires. All share --orbit-interval (Game::
   orbit_log_interval_ms) except kGateSkyDim, whose interval is fixed where it
   is polled. */
enum LogGate { kGateOrbit, kGateDbg, kGateInfo, kGateAtt, kGateShake,
               kGateCam, kGateEva, kGateDrag, kGateTerrain, kGatePlane,
               kGateSkyDim, kGateXfer, kGateCount };

/* A headless test hook (cli.h --vab-* etc): fires act() once, at/after ms
   from loop start (Game::loop_start_ms). Registered at boot from the CLI
   args -- installHeadlessHooks (main.cpp) and vabInstallHooks (vab.cpp) --
   and polled by Game::fireHooks at the phase points below. Registration
   order within a phase IS the firing order, and the e2e cases depend on it
   (e.g. --vab-place before --vab-launch). gate is an extra per-frame
   condition: the VAB hooks only fire while the VAB is the live scene, and
   --vab-place also waits for the mouse to be off the UI. */
struct GameHook {
    int ms;
    bool fired = false;
    std::function<bool(Game &)> gate;   // empty = always eligible
    std::function<void(Game &)> act;
};

/* Where in the frame a phase's hooks are polled. The split is semantic: a
   --reload decides the scene, so it runs before the scene is read; the
   --select-target hook keeps the old poll site (after sc.update, so this
   frame's removals have already erased their xferTargets rows -- see
   Game::dropVehicleRefs); --map-dump needs the g.hud.view that sc.refresh
   fills. */
enum HookPhase { kHookPreScene,    // before the live scene's update
                 kHookPostUpdate,  // after sc.update
                 kHookPostRefresh, // after sc.refresh
                 kHookPhaseCount };

struct Game {
    // --- borrowed subsystems (main creates + deletes) ---------------------
    Renderer &display;
    PostFX *postfx;
    /* The named crew pool (crew.h). Borrowed like the rest: main loads it from
       --kerbals, a save replaces its contents. A minted capsule seat takes its
       name from here. */
    CrewRoster &roster;
    System &sys;
    TerrainBody *sun;
    TerrainBody *home;
    TerrainBody *titleBody = nullptr;   // chosen ONCE per system, built sync at boot
    GameArgs &args;
    Uint32 sim_win_id;
    Uint32 loop_start_ms = 0;   // set once the main loop is about to start

    // A member so the clock hook can invalidate its state.
    TransferPlanner xferPlanner;

    // A silent no-op without a device (audio.h).
    Audio audio;

    // --- cameras -----------------------------------------------------------
    Camera *camera = nullptr;   // orbit + free + first person (`camera->mode`)
    int cam_speed = 1;
    // Chase-cam rumble under high proper acceleration (see render.cpp).
    glm::dvec3 shake_off = glm::dvec3(0.0);
    glm::dvec3 shake_ang = glm::dvec3(0.0);
    Uint32 shake_last_ms = 0;

    /* The scene stack: back() is the live scene, never empty (see scene.h). Each
       frame carries the camera pose to hand back when it is popped. */
    std::vector<SceneFrame> sceneStack;
    VabState vab;

    /* Headless test hooks (cli.h --vab-* etc): registered at boot by
       installHeadlessHooks (main.cpp) + vabInstallHooks (vab.cpp), fired
       once each by fireHooks at the phase points. A SINGLE hook's ms is
       -1 = never (the registration guards skip it); repeatables must pass
       ms >= 0. Each hook's `fired` latches so it fires at most once per run. */
    std::array<std::vector<GameHook>, kHookPhaseCount> hooks;
    // Unlatched hooks per phase; lets fireHooks skip a phase with none.
    std::array<int, kHookPhaseCount> hooks_pending = {};
    // Set by the first fireHooks; addHook asserts boot-time registration only.
    bool hooks_polled = false;
    void addHook(HookPhase phase, int ms,
                 std::function<bool(Game &)> gate,
                 std::function<void(Game &)> act);
    void fireHooks(HookPhase phase);

    // What save_game records -- not args.system_file. Updated by switchSystem.
    std::string systemPath;

    std::string gameName = "game1";
    std::string gameId;   // the saves/ dir, <YYYYMMDD_HHMMSS>-<name> (see save.h)

    // --- the clock ----------------------------------------------------------
    int time_accel = 1;
    double time = 0;   // the analytic sim clock (s), advanced by the tick

    /* Bodies' orbits/spin are a pure function of `time`. Anything moving the clock
       OUTSIDE a tick must call these, or a paused game renders a stale epoch. */
    void syncRails() { sun->frame->UpdateOrbitRails(time); }
    /* A job captures cache_epoch at post time and skips its publish if it changed. */
    void invalidateClockStampedCaches() {
        xferPlanner.invalidateClockState();
        surfmap.valid = false;
        surfmap.computed_at = -1.0;
        cache_epoch++;
        // In-flight counters are only decremented by their continuations; on the
        // switch path those are discarded (jobs.restart).
        xferPlanner.pc_in_flight = 0;
        surfmap.in_flight = 0;
    }
    /* A clock move OUTSIDE a tick (a load, --start-time); a system swap invalidates
       directly. */
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
    const double dt = 1.0 / args.physics_hz;   // --physics-hz (60 default)
    // Counts TICKS, not the Bullet substeps inside each one.
    long long logic_ticks = 0;

    // jobs.poll() runs the finished jobs' main-thread continuations (job.h).
    JobRunner jobs;

    // --- the wall-clock log gates (--orbit-interval; tick.cpp + xfer-log) --
    // Each log needs its own "last fired" clock, or only the first one fires.
    const Uint32 orbit_log_interval_ms = (Uint32)(args.orbit_interval * 1000.0);
    // True once per interval_ms, stamping the gate as it fires.
    bool logDue(LogGate gate, Uint32 now_ms, Uint32 interval_ms) {
        assert(gate >= 0 && gate < kGateCount);
        Uint32 &last = log_last_ms[(size_t)gate];
        if(now_ms - last < interval_ms) { return false; }
        last = now_ms;
        return true;
    }
    std::array<Uint32, kGateCount> log_last_ms = {};

    // --- --pose-jitter (render.cpp) -----------------------------------------
    /* The previous DRAWN sample plus what it was taken against. Re-armed whenever
       the ship, its Frame or the wall-clock gap changes, so a handoff / ship switch
       never reads as a jitter spike. prev_* compare as identities, never
       dereferenced. */
    bool pose_jitter_armed = false;
    Vehicle *pose_jitter_prev_ship = nullptr;
    Frame *pose_jitter_prev_frame = nullptr;
    std::chrono::steady_clock::time_point pose_jitter_prev_t;
    glm::dvec3 pose_jitter_prev = glm::dvec3(0.0);
    glm::dvec3 pose_jitter_prev_nose = glm::dvec3(0.0, 0.0, 1.0);

    // --- input / selection state -------------------------------------------
    bool running = true;
    bool rmbCam = false;            // RMB held over 3D: camera look
    // The in-progress RMB gesture (events.cpp): a short still press is a click.
    int rmbDownX = 0, rmbDownY = 0;
    Uint32 rmbDownMs = 0;
    int rmbMoved = 0;               // |xrel| + |yrel| while held
    /* Cursor position and the LMB edge, fed by the EVENT queue (events.cpp)
       rather than polled from SDL_GetMouseState. The poll reads the DEVICE,
       which a pushed event never updates, so --sim-mouse could move the camera
       (that reads the events) while the VAB's hover/place/select -- also a poll
       -- sat frozen. One source for both, so synthetic input drives the editor
       the way it drives everything else.
       lmbPressed is an edge, not the held state: --sim-mouse issues a
       zero-duration click as DOWN then UP in the same poll, so a held flag
       would be back to false before the editor ever looked. RMB clicks already
       work that way, off the UP event. poll_events clears it on entry, so it
       lives exactly one frame; vabClickAfterUI is its only consumer. */
    int mposX = 0, mposY = 0;
    bool lmbPressed = false;    // one-frame edge; cleared by poll_events
    bool poly_mode = false;         // F11 wireframe
    bool screenshot_requested = false;
    /* Every Vehicle in the world, in spawn order, OWNED here (a TerrainBody
       owns only its pads + terrain). This is the one authoritative list -- the
       Ship List, F6, "ship N of M", the tick pass, the save's v0/v1/.. slugs
       and the transfer planner's target list all read it, so there is exactly
       one order for all of them. Spawn order is stable across an SoI crossing;
       a same-body query filters Vehicle::m_parent.

       Ordering invariant: these die BEFORE the bodies they reference
       (resetWorld / switchSystem). A Vehicle holds m_parent and its frame, both
       owned by the body, so a live one outliving its body leaves the tick and
       the renderer following freed memory. */
    std::vector<Vehicle *> vehicles;

    // The player-controlled ship (every vessel is in `vehicles` above).
    Vehicle *ship = nullptr;
    // `kerbal` is the one most recently EVA'd (the V toggle-back); `lastShip` the
    // ship they came from.
    Kerbal *kerbal = nullptr;
    Vehicle *lastShip = nullptr;

    // --- part windows (pickAt opens one per right-clicked part) -----------
    std::vector<PartSel> part_sels;

    // --- render resources (render.cpp draws with them) ---------------------
    // FILE assets are registry-owned (get_*, shared); billboard quads free with them.
    Skybox *skybox = nullptr;
    Shader *skyboxshader = nullptr;
    Shader *lineshader = nullptr;
    Shader *partsshader = nullptr;
    BuildingsCatalog buildings;   // world-building types (res/data/buildings.json)
    /* The part types the world is built from (res/data/parts.json, or
       --parts). main loads it and moves it in; a save records only the file
       name, so a load resolves part names against whatever is live here. */
    PartsCatalog parts;
    Mesh *engine_plume_mesh = nullptr;
    Texture *engine_plume_texture = nullptr;
    Mesh *ref_circle_xz = nullptr;
    Mesh *ref_circle_xy = nullptr;
    // Ring's +/-X ticks: the plane overlay's node markers. +X is the ascending
    // side (the overlay pins its basis to the node line), so the two meshes can
    // carry the orbit map's green/red node colors.
    Mesh *ref_node_an = nullptr;
    Mesh *ref_node_dn = nullptr;
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
    bool draw_ref_circles = false;   // the render frame's XZ + XY rings
    bool draw_orbit_plane = false;   // orbital plane + reference plane + node line

    // render.cpp skyGain: the gain the sky fades to when the sun is in view and the
    // camera is in sunlight. sky_dim = 1 disables the fade.
    float sky_dim = 0.25f;
    float sky_dim_cone = 25.0f;   // deg: half-angle of the "sun in view" cone
    // Low-passed in wall-clock seconds: the inputs are binary, so without this the
    // sky steps instead of ramping.
    double sky_fade = 0.0;
    Uint32 sky_fade_last_ms = 0;

    // --- control-axis flips (the Settings window writes, tick.cpp reads) --
    bool flip_pitch = false;
    bool flip_yaw = false;
    bool flip_roll = false;

    // --- the key map (keys.h) --------------------------------------------
    KeyBindings binds;
    // Slot the Controls window is rebinding; -1 = not capturing.
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

    // Built from science.version; a version mismatch rebuilds, so it self-heals.
    std::vector<LabEntry> labRows;
    std::size_t labRowsVersion = std::size_t(-1);   // no cache built yet

    // Pre-built System Atlas rows (one per body, tree-ordered); same self-heal.
    std::vector<AtlasRow> atlasRows;
    std::size_t atlasRowsVersion = std::size_t(-1);

    // --- the active ship's per-frame state + the Telemetry window ---------
    HudState hud;

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
    MapState map;

    // --- world-stamped caches ------------------------------------------------
    // Bumped by invalidateClockStampedCaches(); a job drops its result on landing
    // if it has moved since.
    int cache_epoch = 0;

    // --- Surface Map state (gameui.cpp draws it; surfmap.cpp fills it) ---
    SurfmapState surfmap;

    /* --- control transitions (a keypress, a part/crew window button, or the
       --eva load-probe hook) --- */
    // V: put a seat occupant out, or hand control back to the last ship.
    void toggle_eva();
    /* kerbalEVA MINTS the Kerbal vehicle from a seat's suit Part; kerbalBoard
       deletes it and turns the suit back into a seat. Both refuse as noted. */
    void kerbalEVA(Part *suit);
    void kerbalBoard(Kerbal *k, Vehicle *ship, size_t part);
    // dropItem makes a free 1-part ship; pickUpItem re-parents it into `dest`.
    Vehicle *dropItem(Part *item);
    bool pickUpItem(Vehicle *itemShip, Part *dest);
    // World (ship-frame) position of a focus target.
    glm::dvec3 focusWorldPos(int i) const;
    TerrainBody *pickTitleBody();   // stores it in titleBody
    void parkTitleCamera();         // no-op until focusTargets is seeded
    // TAB: hide / restore the live scene's Persistent windows.
    void toggle_windows();
    void apply_ui_visible();
    // A scene entry always shows the UI (a TAB-hide must not carry over).
    void ensure_ui_visible();
    void apply_ui_style();
    // Settings persistence (settings.json). CLI beats the file field-by-field
    // (args.cli_given). Two load phases: args before the Renderer, Game+PostFX after.
    bool save_settings();
    void load_settings();
    // Takes control of `v` (releases + parks the current one).
    void select_vehicle(Vehicle *v);
    /* Space Center, no ship, paused. False (plus a toast) if a game is running. */
    bool newGame();
    /* False if a game is running or the system switch failed (world untouched). */
    bool startNewGame(const std::string &name, const std::string &sysPath,
                      float exhaustScale);
    std::string ensureGameDir();    // "" on a mint failure (a toast already fired)
    /* Claim a fresh saves/<stamp>-<gameName>/ dir unconditionally: this STARTS a
       game, so it is not gated on one already being in charge the way
       ensureGameDir is. Sets gameId. False = the name was taken this second. */
    bool mintGameDir();
    /* Does NOT make `active` the player's ship -- the caller does. --load skips
       this (a save records live-or-railed state). */
    void settleVehicles(Vehicle *active);
    /* Refusal costs nothing (#302, #305): the whole save is built -- against the
       INCOMING system when it differs -- before installSystem tears anything
       down. Only a failure AFTER the switch (a mesh place_world_buildings needs
       but validate_world_buildings does not check) costs the career, and that
       path says so on stdout and lands on the title. */
    bool loadFrom(const std::string &dir);
    // Quicksave (F5): the game dir's next quicksave-NN pool slot.
    void quicksave();
    // Quickload (F9): the newest quicksave of this game by mtime.
    void quickload();
    /* True iff the switch ran, so the caller must not run the heavy phase. */
    bool ensureSystemForSave(const std::string &dir, bool *switched = nullptr);
    /* The one list of state that belongs to a game and dies with it (#304).
       Three callers only: resetWorld, a SUCCESSFUL load, and newGame. The bug
       class is OMISSION -- every teardown used to clear its own subset, so
       whatever was unlisted survived the reset. The definition carries the
       rule and the two deliberate exceptions. */
    void clearEphemeralState();
    /* The only teardown. resetWorld = clearEphemeralState + the fleet, in the
       one order that is safe (refs first, then the vehicles they borrow, then
       the bodies' owners get to die). No job drain: no background
       continuation dereferences a Vehicle. */
    void resetWorld();
    /* Queue a world change (load / new game / quit to title) for the top of the
       NEXT frame. Only the UI pass needs it: a teardown deletes Vehicles
       immediately where retire() defers, so one that runs inside drawUi frees
       what every LATER widget in the same frame borrows -- safe today only
       because drawSaveLoad happens to be last in each scene's widget list,
       which makes widget ORDER load-bearing state (#304). The events and hook
       phases need no deferral: they already run before refresh and paint. */
    void requestWorldOp(std::function<void()> op);
    /* The frame loop only, next to sweepRetired(): run what the UI queued
       before anything reads the world this frame. */
    void runWorldOps();
    // resetWorld + enterTitle: the flight pause menu's "Quit to title".
    void resetToTitle();
    /* Transactional (see game.cpp). `syncNames` are body NAMES to build
       synchronously; empty = the title backdrop. */
    void switchSystem(const std::string &path,
                      const std::vector<std::string> &syncNames = {});
    /* The install half of switchSystem, on an already-loaded system. Call it
       directly only to validate something against the new world between the load
       and the teardown (Game::ensureSystemForSave, #302). */
    void installSystem(System &&newSys, const std::string &path,
                       const std::vector<std::string> &syncNames);
    // Keeps the "ship" focus entry on the active ship (else a random non-star
    // backdrop).
    void syncShipFocus();
    // False + keeps the accel if any ship is not rail-eligible.
    bool enter_rails_warp();
    void updateProximity();
    /* The active ship's port against every other live ship's port -- close, aligned
       and slow enough, merge. */
    void updateDocking();
    // Splits the most recent seam off the active ship.
    void undock();
    // Refuses a stage that still carries a crewed capsule.
    void stage();
    /* The tracking list's "x" and the VAB's occupied-pad refusal (the ui_
       prefix: a player action, not a teardown). Refuses the last ship; hands
       control off if the active one is removed.
       False = refused (also the null case), so a caller drawing a delete
       button knows whether the row actually went away. */
    bool ui_remove_vehicle(Vehicle *v);
    /* Refuses unless grounded on the home body (--recover-anywhere lifts that).
       Unlike ui_remove_vehicle this allows the last vessel. */
    void recoverActive();
    /* Stores on the walking kerbal's suit; refuses a duplicate already held. */
    void runExperiment(Kerbal *k);
    /* Same, for a SEATED occupant: the suit Part is the storage and the capsule
       is the live pose (there is no Kerbal object for a seated one). */
    void runSeatExperiment(Part *suit);
    /* The half both of those share: `suit` stores the finding, the live pose is
       `poseVehicle`'s (`localPart` = the part the runner sits in, null = the
       vehicle's own COM), and the provenance is `runner` aboard `shipName`. */
    void runSuitExperiment(Part *suit, Vehicle *poseVehicle, Part *localPart,
                           const std::string &runner,
                           const std::string &shipName);
    /* The situation experiment (identity + provenance) for a part riding
       `poseVehicle`, recorded by `runnerName` aboard `shipName` ("" = free
       EVA). Provenance is names, not pointers: a seat has no Kerbal object. */
    bool situationExperiment(TerrainBody *body, Vehicle *poseVehicle,
                             Part *localPart, const std::string &type,
                             const std::string &runnerName,
                             const std::string &shipName, Experiment &out);
    /* Refuses a duplicate the pod already holds. */
    void runPodExperiment(Part *pod, const std::string &runner);
    /* `to` must be a transfer destination with a free slot. */
    void moveExperiment(Part *from, Part *to, size_t which);
    bool kerbalInRange(Kerbal *k, Part *part);
    /* The gates the crew buttons apply, in ONE place. Both are about a WALKING
       kerbal acting as a courier. canMoveExperiment moves a finding to or from
       that courier's OWN suit (parts[0]) and refuses anything else: a
       capsule-to-capsule or ship-to-ship transfer is not a courier move, so a
       future transfer of that shape needs its own gate rather than this one.
       Reach is owed only to the end the courier is NOT carrying, because the suit
       they carry is their root part and so is trivially in reach. Every button
       that offers such a move asks these first, so a button cannot offer what the
       move would refuse. */
    bool canMoveExperiment(Part *from, Part *to, Kerbal *courier);
    bool canBoard(Kerbal *courier, Vehicle *ship, size_t part);
    // Push a one-shot on-screen message (printf-style).
    void toast(const char *fmt, ...);
    /* Console line only. Scene transitions use this -- the move is visible, the
       "[toast] ..." line stays an e2e anchor. */
    void toastLog(const char *fmt, ...);
    /* onAnswer runs on a later frame with the chosen index (see Question). */
    void ask(std::string text, std::vector<std::string> labels,
             std::function<void(int choice)> onAnswer);
    /* Closes the card first, so a callback may ask() again. No-op with none up. */
    void answerQuestion(int choice);
    std::unique_ptr<Question> question;   // null = no card up
    // Opens (or focuses) the part window for (ship, part), near (mx,my).
    void openPartWindow(Vehicle *ship, size_t part, const glm::dvec3 &point,
                        int mx, int my);
    // Its entries would dangle after delete.
    void dropPartWindowsFor(Vehicle *ship);
    /* The one way a live Vehicle dies: out of Game::vehicles, out of every
       reference the Game and the transfer planner hold, and out of the physics
       world -- then parked in `retired` until the next sweep frees it. After
       this only the caller's own surgery may touch `v` (handing a suit off its
       parts list, peeling crew to the pad); the object stays VALID so a pointer
       captured earlier in the frame cannot dangle under it. */
    void retire(Vehicle *v);
    /* Free everything retire() parked since the last sweep. Called once per
       frame at the top of the loop (main.cpp) and before any bulk teardown
       (resetWorld) -- never from a game action, which is what makes the
       deferral worth having. */
    void sweepRetired();
    /* retire()'s deferred half. Nothing walks or reads it: a retired Vehicle is
       out of the world and waiting to be freed, not a container of state. */
    std::vector<Vehicle *> retired;
    /* The same deferral, one level up: world changes the UI asked for while it
       drew, run at the top of the next frame (requestWorldOp). */
    std::vector<std::function<void()>> worldOps;
    /* retire()'s two halves, public only so the bulk teardowns (resetWorld,
       load_game) can read what they are clearing. A game action calls retire(). */
    void removeVehicle(Vehicle *v);
    // Clears every reference the Game and the transfer planner hold to a
    // vehicle that is about to die.
    void dropVehicleRefs(Vehicle *v);
    // Every game teardown calls this (scene transitions close nothing).
    void clearFlightSummary();
    /* The crew pool becomes the world: one suit Part per roster profile,
       standing at the home system's crew pad (crew.h). Called at boot and on
       every new game, once the parts catalog and the pad exist. */
    void mintCrewPool();

    Game(Renderer &display, PostFX *postfx, CrewRoster &roster,
         System &sys,
         TerrainBody *sun, TerrainBody *home, GameArgs &args,
         Uint32 sim_win_id)
        : display(display), postfx(postfx), roster(roster),
          sys(sys),
          sun(sun), home(home), args(args), sim_win_id(sim_win_id),
          xferPlanner(*this)   // borrows the Game it is a member of
    {
        // Never empty: Flight is the floor scene, and curSceneId reads back()
        // unguarded.
        sceneStack.reserve(4);
        sceneStack.push_back(SceneFrame{});   // Flight, no parked camera
        // --surfmap-noshade (CLI) mirrors the Surface Map "Sun shading" box.
        surfmap.shade = !args.surfmap_noshade;
    }
};

// RMB-click entry point (events.cpp). The [pick] log line is what the e2e cases
// assert.
void pickAt(Game &g, int px, int py);

// settings.json phase 1 (phase 2 is Game::load_settings). No-op if absent.
void load_settings_args(GameArgs &args);

/* Every seat occupant on the vehicle, in part order (part.h: a seat is a Part
   on a capsule). Empty = the vehicle has no crew. */
std::vector<Part *> vehicleSeats(Vehicle *v);
/* Display name for a person: their owned profile, else empty. */
std::string seatName(const Part *suit);
/* Free kerbals are the isEva entries of Game::vehicles; a seated crew member is
   not a vehicle at all (part.h). Walked into a reused buffer (cleared here), for
   per-frame callers. */
void freeKerbalsInto(const std::vector<Vehicle *> &vehicles,
                     std::vector<Kerbal *> &out);

// One constant so the boarding / take-store reach the UI shows and the gate it
// enforces can't drift.
constexpr double kBoardingRange = 10.0;
