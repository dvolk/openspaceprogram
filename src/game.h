// game.h -- the running game: subsystems, runtime state, control transitions.
#pragma once

#include <SDL3/SDL.h>   // Uint32

#include <array>
#include <cassert>     // assert() is used below; the header states what it uses (#691
                       // found atmo.h failing the same way -- SDL3 includes it today)
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <unordered_map>
#include <string>
#include <utility>
#include <vector>

#include "audio.h"
#include "surfmap.h"   // SurfMapType / SurfLevel: the map state names the picture it holds
#include "camera.h"   // Camera, CameraMode
#include "cli.h"      // GameArgs
#include "constants.h" // kMapMinScale (MapState's boot scale)
#include "crew.h"     // CrewRoster (the named crew pool)
#include "display.h"  // Renderer
#include "flightlog.h"
#include "guidance.h" // AscentLaw (the ascent autopilot)
#include "job.h"      // JobRunner
#include "orbit.h"    // OrbitElements, RefPlane, PlaneAngles
#include "postfx.h"   // PostFX
#include "recorder.h"  // RecSample / recRow (the flight recorder's row grammar)
#include "scene.h"    // SceneId, SceneFrame, Backdrop
#include "science.h"  // Experiment
#include "shipdef.h"  // PartsCatalog (the part types the world is built from)
#include "siminput.h" // TimeSeries
#include "staging.h"  // StageRow (the Staging window's cached table)
#include "timestep.h"  // scriptedClockMs (--sim-steps)
#include "system.h"   // System
#include "terrain.h"  // TerrainBody
#include "transferplanner.h"
#include "question.h"   // Question, dropPendingQuestion, answerPendingQuestion
#include "seatplan.h"   // SeatAssign, pruneSeatPlan, seatPlanProblem
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
    /* THE 2 Hz INSTRUMENT CADENCE (#734), per WINDOW rather than per program.
       drawPartWindows loops over g.part_sels, so state kept in a static there is
       SHARED by every open window: with two open (right-clicking wing_l then
       wing_r is the natural way to use this panel) each iteration would mismatch
       the other's identity and publish every frame, and the hold would silently
       stop existing -- measured at 2.0 Hz with one window, 60.0 Hz with two. It
       lives here instead because a PartSel is erased with its window and with its
       ship, so a held number cannot outlive either -- the same rule this file
       states for cached paths: "the slot can be deleted between frames". */
    double nextRead = 0.0;    // wall seconds (ImGui::GetTime) of the next publish
    /* The worst LOADED frame since the last publish, kept as one sample rather
       than four maxima: q, Mach and AoA came from the same instant as the watts,
       so the published rows are internally consistent and never understate a
       spike. Frame-sampled -- see the caveat in gameui.cpp. */
    double holdWatts = 0.0, holdQ = 0.0, holdMach = 0.0, holdAlpha = 0.0;
    char flowRow[64] = "";    // the two published rows, formatted once per publish
    char wattsRow[64] = "";
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
    /* The ship's velocity relative to the AIR, in this same (render) frame
       (#498). marker_vel is the CONIC's velocity, so inside an atmosphere on a
       rotating body it is mostly the GROUND's own motion: Kerbin carries the
       sea-level ground ~175 m/s, which swamps an 80 m/s airframe and puts the
       green prograde 16 deg off a nose the aero says is dead on the flow
       (Beta = 0.0 the whole flight). This vector comes from the law the forces
       themselves use (Vehicle::airRelativeVel), so a marker and a drag force
       cannot describe different directions. air_dir_valid is the aero's own flow
       floor (kAoAFlowFloor) plus "there is air": a parked ship's ~1e-4 m/s of
       integrator jitter has no direction to point at, so the marker is absent
       rather than random -- the same dash rule as the AoA readout. */
    glm::dvec3 air_vel = glm::dvec3(0.0);
    glm::dvec3 air_dir = glm::dvec3(0.0);
    bool air_dir_valid = false;
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
    glm::dvec3 facing_dir = glm::dvec3(0.0);
    glm::dvec3 vel_dir = glm::dvec3(0.0);
    // surface + orientation scalars (the SURFACE window)
    /* The altitude PAIR, computed once for every reader instead of at each one: the
       SURFACE window, [surfinfo] and the flight recorder (#674) all quote it, and the
       pair is the only honest way to print a height -- #626 is the mess that happens
       when logs spell one name two datums. alt_asl is above the body's sea-level
       datum and reads NEGATIVE exactly where the air is thickest (#245); alt_agl is
       above the ground actually under the hull. */
    double alt_asl = 0;
    double alt_agl = 0;
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
    /* The top bar's save-name buffer, seeded from the build on first entry.
       It lives here, not in a drawVabUI static, so load and clear keep it in
       step with the tree they replace. */
    char saveName[128] = {0};
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

    /* --- seat assignments (the VAB part window: right-click a part) --------
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
    using SeatAssign = ::SeatAssign;   // seatplan.h: the pure rules live there
    std::vector<SeatAssign> seatAssigns;

    // The VAB part window (dossier + seats): the build part it shows ("" =
    // closed). An ID, not an index -- the tree renumbers under the player's feet.
    std::string partWinId;
    int partWinMx = 0, partWinMy = 0;   // where it was opened (first-draw placement)
    bool partWinPlaced = false;

    /* The Staging window's table, cached against a content hash of the build
       (#293). The window redraws every frame; the build only changes when the
       player edits it, and a burn simulation costs 84-473 us on a real ship
       where revalidating the hash costs about a microsecond. 0 is "no cache":
       a real signature is FNV-1a over the whole build, so colliding with 0 is
       not a practical concern, and a stale entry could only ever show a table
       one edit behind -- never a wrong launch. */
    uint64_t stagingSig = 0;
    std::vector<StageRow> stagingRows;
    /* id -> index into vab.build.parts, rebuilt with the rows above and keyed by the
       same signature (#527). The fuel-link overlay draws every frame and used to
       resolve both ends of every link by scanning every part and comparing std::string
       ids: 11.8 us/frame at 268 parts and 24 links, now 1.3 (unordered_map; std::map
       costs about twice that on both arms, and nothing iterates this). Rebuilding
       costs 23.7 us and only happens when the signature moves, i.e. once per edit.
       Indices rather than pointers, because the parts vector reallocates as the player
       builds, and the frame pays one signature for both readers -- see
       ensureStagingCache's caller in gameui.cpp. */
    std::unordered_map<std::string, int> partIndex;
    /* Bit k: vab.build's part i, def node k, is used by a stack edge. This vector
       describes vab.build and nothing else, which is what the name says: VabState
       also owns subassemblies, whose ports are NOT in here. Filled by
       BuildShip::fillNodeTaken at the top of each reader -- the free-port gizmos
       and pickVabNode -- so unlike the cache above it carries no key and cannot be
       read stale: nobody asks it a question in a call that did not fill it. Both
       readers share it because a green dot and the click it invites must agree
       about which ports are free (#542). */
    std::vector<uint64_t> buildNodeTaken;
};

/* Orbital map SESSION state (Game::map), not save state: a fresh boot starts
   on the defaults. map.plane is NOT just chrome: ShipView's Inc/LAN/LPe are
   measured in it (see ShipView), so the readouts follow the picture. */
struct MapState {
    // Meters per pixel. A boot seed only: the first draw fits it to the focus.
    float scale = kMapMinScale;
    // RefPlaneMode (frame.h). ImGui::Combo needs an int*.
    int plane = kRefEquator;
    // Pan offset from the window center, in pixels (P4 navigation).
    ImVec2 pan = ImVec2(0.0f, 0.0f);
    // Right-click cycles chrome: 0 = full window, 1 = bare map, 2 = no chrome.
    int mode = 0;
    /* The body and the ship this view was last fitted to, which latch the fit:
       it only re-runs when the focus changes -- a new body (SOI crossing) or a
       new ship (F6, a launch), even around the same body -- or "Reset view"
       forces it, so a maneuver does not drag the camera. The scale itself comes
       from that ship's orbit, floored at the body's radius: arriving at Earth
       or Gilly opens at the same screen size as Kerbin, and a low orbit opens
       framed on the orbit. Both are compared only, never dereferenced;
       clearEphemeralState nulls them with the world they belong to, so the next
       draw re-fits. */
    TerrainBody *fit_body = nullptr;
    Vehicle *fit_ship = nullptr;
    /* The player has wheeled or panned since the last fit. A focus change
       then leaves their framing alone -- the fit is the opening view, and
       "Reset view" asks for it back. */
    bool posed = false;
};

/* Surface Map SESSION state (Game::surfmap), not save state: the pixels are
   re-baked from the body on demand. Everything but the three toggles and the
   two counters (rev, in_flight) belongs to the world the map was baked in, so a
   teardown replaces the struct wholesale -- see resetForNewWorld. valid is also
   the clock-stamped cache's: invalidateClockStampedCaches() clears it on a clock
   jump, which is the only invalidation that route gets. in_flight is NOT: its
   job survives a clock jump and its continuation clears the count, so only
   installSystem zeroes it -- next to the jobs.abort() that discards those
   continuations. */
struct SurfmapState {
    /* null = the active ship's parent, else the system home. A pick of a body
       in THIS system, so it outlives a same-system teardown -- those are the
       same TerrainBody objects -- and dies where the bodies do (installSystem). */
    TerrainBody *body = nullptr;
    bool shade = true;    // bake the terminator (the "Sun shading" box)
    // Which picture the sweep paints, and at what level (#745). The old "Ocean"
    // box is gone: painting the sea is a PICTURE (kMapGroundOcean), not a coat
    // over one picture, and the two map types say so. The terminator stays a box
    // because it reads on top of any of them.
    int map_type = kMapGround;
    int level = kLevelGround;
    // Overlay-only, so unlike the two above it needs no re-bake.
    bool site_labels = true;
    // The last computed map (surfmapCompute publishes it atomically).
    // px is the base for whatever type was baked (the albedo, or the temperature
    // field); shade_px is the terminator overlay (black, alpha = 1-shade) drawn
    // over it. A TYPE change re-bakes the base; toggling shading stays a free
    // show/hide.
    std::vector<unsigned char> px;       // RGBA8, w*h
    std::vector<unsigned char> shade_px; // RGBA8 overlay, w*h
    int w = 0;
    int h = 0;
    std::string body_name;
    // What the published buffers actually ARE, so switching map type re-bakes
    // instead of showing the previous type's pixels under the new label (#745).
    int baked_type = kMapGround;
    /* The temperature layer's legend, published with the buffers it belongs to.
       temp_ok=false means this world cannot be painted as a temperature at all,
       and temp_note carries the reason -- the window shows it rather than
       greying a box out, because "no data" and "no ground to have data about"
       are different answers (#726's rule that an absent thing stays a state).
       temp_lo_K / temp_hi_K are the model's OWN ends (the same two figures the
       dossier prints), which is what the colour ramp spans; temp_min/max/mean_K
       are what the sweep actually painted, logged so the two can be compared, and
       temp_mean4_K is that sweep's mean taken in T^4 -- the quantity the model
       conserves -- written back as a temperature, which is the number that must
       land on temp_anchor_K. */
    bool temp_ok = false;
    double temp_anchor_K = 0.0, temp_lo_K = 0.0, temp_hi_K = 0.0;
    double temp_min_K = 0.0, temp_max_K = 0.0, temp_mean_K = 0.0, temp_mean4_K = 0.0;
    double temp_knob = kEvenShareUnauthored;
    /* The star direction the published pixels were baked with. The cursor readout needs it:
       the number beside a pixel has to be THAT pixel's number, so it is the bake's sun, not
     the sim's sun now (the map is a snapshot and frames move underneath it -- #687's
     "measured or recomputed, never both at different instants"). */
    glm::dvec3 baked_sun_dir = glm::dvec3(0.0);
    bool baked_sun_valid = false;
    /* Three words naming WHICH absence, for the cursor row (`ground - (no swing)`): the
       sentence lives in the log, the word lives on the line, and both are the same decision
       (surfmap.h:TempLayer). A bare dash would merge the different absences back into one
       (#726). */
    std::string temp_block;
    bool valid = false;
    int rev = 0;
    // Jobs posted but not landed (the window shows "mapping ...").
    int in_flight = 0;
    // M: one-shot compute request (events.cpp sets, the state pass consumes).
    bool compute_requested = false;

    /* The world was replaced -- quit-to-title, a new game, a load. Build the
       replacement and move the survivors across, so a field added here resets
       by default rather than leaking the dead game's pixels into the next one
       (#346). Same shape as TransferPlanner::resetForNewWorld, and called from
       Game::clearEphemeralState.

       What survives, and why:
         shade / map_type / level / site_labels   the player's boxes in the window
         body       a pick of a body in THIS system. A same-system teardown keeps
             the very TerrainBody objects it names, so a pinned --surfmap-body or
             combo pick stays put; installSystem nulls it where the bodies die.
         rev        a monotonic stamp, not world state. The window's two
             textures are program-lifetime statics keyed by rev (gameui.cpp), so
             restarting it at 0 makes the next bake look unchanged and leaves
             the old body's pixels on screen.
         in_flight  a bake posted before the teardown still runs its
             continuation -- only installSystem discards those -- so it still
             owns a slot.

       What this fixes: a New Game started with --start-time -- newGame skips its
       setTime(0.0) for that knob, so nothing invalidates the cache -- kept a map
       that was valid, correctly named and baked for the game that had just
       ended, and the window's auto-rebake (gameui.cpp) cannot fire while
       valid && body_name still matches the body on screen. */
    void resetForNewWorld() {
        SurfmapState fresh;
        fresh.body = body;
        fresh.shade = shade;
        fresh.map_type = map_type;
        fresh.level = level;
        fresh.site_labels = site_labels;
        fresh.rev = rev;
        fresh.in_flight = in_flight;
        *this = std::move(fresh);
    }
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
                     orbitCutAlt(body->rot_frame->soi, body->seaLevelRadius()));
    if(!body->hasClassifiableSurface() || !body->ready) {
        return { Biome::None, situation };
    }
    return { biomeAt(dir, body->params()), situation };
}

/* The question card (drawQuestion, gameui.cpp). The pure rules live in
   question.h: answered on a LATER frame, so onAnswer must capture IDENTITY
   (a name, an id), never a pointer -- the subject can be gone by then. */

/* Log gates: each throttled log needs its own "last fired" clock,
   or only the first one fires. Wall-clock clocks unless --sim-steps replaces
   them (logClockMs, below). All share --orbit-interval (Game::
   orbit_log_interval_ms) except the ones whose cadence is fixed where they are
   polled: kGateSkyDim, and the tick.cpp state logs (spin/mass/fuel/drain/
   power/slew), which keep the rate each was written to sample at.
   Wall clock is what makes a log readable at any instant the player can see:
   a SIM-clock gate prints nothing while paused and floods at warp (#374).
   --sim-steps swaps in the tick clock for a scripted run (#472), which fixes the
   CADENCE of the rows only -- see the scope note on sim_steps_left below. */
enum LogGate { kGateOrbit, kGateDbg, kGateInfo, kGateAtt, kGateShake,
               kGateCam, kGateEva, kGateDrag, kGateTerrain, kGatePlane,
               kGateSkyDim, kGateXfer, kGateDv, kGateCompound, kGateProxSnap,
               kGateSpin, kGateMass, kGateFuel, kGateDrain, kGatePower,
               kGateSteer, kGateSlew, kGateAscent, kGateRec, kGateSolar, kGateCount };

/* A headless test hook (cli.h --atlas-dump / --map-dump): fires act() once,
   at/after ms on the schedule clock (Game::schedClockMs(): loop_start_ms normally,
   the tick clock under --sim-steps). Registered at boot from
   the CLI args (installHeadlessHooks, main.cpp) and polled by Game::fireHooks
   at the phase points below. Registration order within a phase IS the firing
   order. gate is an extra per-frame condition.
   The phases sit on different sides of tick() (kHookPreScene before it,
   kHookPostRefresh after), so under --sim-steps two hooks given the SAME ms fire
   in phase order but sample clocks one tick apart -- a dump line's t= can differ
   by one dt between the phases. Order between phases is not stable across modes. */
struct GameHook {
    int ms;
    bool fired = false;
    std::function<bool(Game &)> gate;   // empty = always eligible
    std::function<void(Game &)> act;
};

/* Where in the frame a phase's hooks are polled. The split is semantic: a
   hook that tears the world down runs before the scene is read; --map-dump
   needs the g.hud.view that sc.refresh fills. */
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
    /* The rumble's own random stream, fixed at boot: std::rand() is already
       deterministic per process, so a seed buys nothing -- what made a scripted
       run irreproducible was the NUMBER of draws (one pair per render frame) and
       the wall-clock dt feeding the smoothing. Its own stream also keeps the
       camera pose from drifting if anything else ever starts randomising.
       Deterministic here matters beyond the log: focusPoint += shake_off feeds
       the terrain LOD, which owns the ground's collision bodies (#475). */
    uint32_t shake_rng = 0x9e3779b9u;
    double shakeRand() {   // [-1, 1)
        shake_rng = shake_rng * 1664525u + 1013904223u;   // LCG (numeric recipes)
        return (double)shake_rng * (2.0 / 4294967296.0) - 1.0;
    }

    /* The scene stack: back() is the live scene, never empty (see scene.h). Each
       frame carries the camera pose to hand back when it is popped. */
    std::vector<SceneFrame> sceneStack;
    VabState vab;

    /* Headless test hooks (cli.h --atlas-dump / --map-dump): registered at
       boot by installHeadlessHooks (main.cpp), fired once each by fireHooks
       at the phase points. A SINGLE hook's ms is -1 = never (the registration
       guards skip it); repeatables must pass ms >= 0. Each hook's `fired`
       latches so it fires at most once per run. */
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
    /* The stored form of THIS game: the saves/ dir it writes into, named
       <YYYYMMDD_HHMMSS>-<gameName> (save.h). Empty = never claimed one. */
    std::string gameId;
    /* Extra trees the Load sheet lists games from, beyond the player's saves/
       dir -- which is never in here and never needs to be. Writing a save still
       goes to saves/<gameId>/ only; these are read-only sources for the sheet.
       Game Debug's fixture checkbox toggles kFixtureRoot (save.h) here. */
    std::vector<std::string> extra_save_roots;

    /* Game Debug's dev-vehicles switch, and the only thing that sets it. The
       VAB's Load picker shows the defs marked "testship": true and lists
       kFixtureVehicleRoot (shipdef.h) alongside the stock and player trees;
       vabLoad resolves a bare name from that tree too. Read-only -- the editor
       still saves to the data dir's ships/. */
    bool dev_vehicles = false;

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
        cache_epoch++;
        /* NOT the in_flight counters. A queued or running job still owes its
           continuation, and that continuation is what clears them; zeroing here
           says "nothing is running" while something is, which lets a second
           sweep post on top of the first. installSystem zeroes them next to
           jobs.abort() -- the only place the continuations are discarded. */
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
    // Both conversions go through gateIntervalMs (cli.h) rather than casting
    // here: the gates count Uint32 ms, so the range is the law and belongs with
    // the flag that fills the field (#673).
    const Uint32 orbit_log_interval_ms = gateIntervalMs(args.orbit_interval);
    // The recorder's cadence is its own knob (--record-interval, #662): a trace
    // at the log rate would be a 3 MB file for one ascent, and at the tick rate
    // it is one for a cruise.
    const Uint32 record_interval_ms = gateIntervalMs(args.record_interval);

    /* --- flight recorder (#662) -------------------------------------------
       A trace's identity sits HERE, not on a Vehicle: staging hands control to a
       different Vehicle and #297 retires the half that was dropped, so a
       recorder hung on the vessel would close its own file mid-flight. The
       per-vessel journal (Vehicle::flog, flightlog.h) starts again for each stage
       for the same reason. A row's t= counts the log clock (see logClockMs),
       which never runs backwards: g.time does, on a load (#304). */
    struct Recorder {
        std::FILE *f = nullptr;    // null = not recording
        bool failed = false;       // the open failed: say it once, then stay quiet
        std::string path;
        Uint32 started_ms = 0;     // the log clock when the trace opened
        long rows = 0;
    };
    Recorder rec;
    /* Open the trace (game.cpp): called from the tick, the first time a row is
       due, so the game dir it lands in is whatever the run has claimed by then.
       now_ms is that tick's logClockMs reading -- the rows count from it, so the
       first one reads t=0. False = the file could not be opened, which is printed
       once and then left: a recorder that cannot write must not spam the log. */
    bool openRecorder(Uint32 now_ms);

    /* --sim-steps (cli.cpp) makes a scripted run reproducible (#472): exactly one
       tick per frame and a fixed number of ticks. The step count alone is not
       enough -- a --sim-press on the wall clock lands on a different tick on a
       faster machine, and a wall-gated log row samples a different substep -- so
       the two clocks below switch together when the mode is on, and stay exactly
       as they were (process-relative for the logs, loop-relative for the
       schedules) when it is off. Off is the player's choice for good reasons: a
       sim-clock log gate goes quiet while paused and fires on every tick at warp
       (#374).
       Scope, honestly: this makes the CADENCE of inputs and logs deterministic.
       A scripted run also replays its CAMERA: the chase-cam rumble and the sky
       fade step on the tick under --sim-steps (smoothingDt) and the rumble draws
       from its own fixed stream (shakeRand), so focusPoint is the same number on
       the same tick. What still differs run to run is the terrain tree's FILL --
       the LOD requests the same patches, but the geometry and the ground
       collision bodies arrive on job continuations whenever the worker finishes,
       so a grounded ship can still settle differently (#475). Flights that never
       touch the ground collision tree (the 458 envelope cases) are reproducible
       today.
       Pin the CADENCE of [shakelog]/[camlog]/[terrain]/[skydim] in a scripted
       run: their payloads replay under --sim-steps now, but in the wall-clock
       mode they follow frame rate by design. [porkchop] depends on job timing
       rather than the camera and is not pinn-able either way. --pose-jitter
       prints a wall-clock frame dt too.
       The scripted clock counts TICKS, not g.time: g.time can rewind (a load or a
       New Game mid-run) and stops dead while paused, and either would freeze or
       reverse a schedule. One tick always moves it dt. */
    long long sim_steps_left = -1;     // < 0: off. Else ticks still owed.
    long long sim_ticks_run = 0;       // spent so far: the scripted clock's base
    double sim_seconds_ran = 0.0;      // sum of dt*time_accel: what the run simulated
    /* #402: the sim time tick.cpp's 10-tick catch-up cap threw away, valued at the
       warp of the frame that dropped it. Wall-clock stepping only -- a scripted run
       cannot lose ticks. Non-zero means this run is shorter than its
       --timeout x --time-accel asks for, by an amount that depends on how fast this
       machine drew frames, which is exactly what makes a "sim seconds" assertion
       lie on one box and hold on another. */
    double sim_seconds_dropped = 0.0;

    /* The clock --sim-press / --sim-mouse / --sim-mode / --ui-* schedules are
       keyed to. Always run-relative ms (t=0 when the main loop starts), so a case
       file's delays keep their meaning; scripted mode just measures that elapsed
       run time by ticks instead of the wall clock. */
    Uint32 schedClockMs() const {
        return (sim_steps_left >= 0) ? scriptedClockMs(sim_ticks_run, dt)
                                     : (Uint32)(SDL_GetTicks() - loop_start_ms);
    }
    // The clock the periodic state logs gate on: process-relative ms when the
    // wall clock is in charge (that is what --orbit-interval has always meant).
    Uint32 logClockMs() const {
        return (sim_steps_left >= 0) ? scriptedClockMs(sim_ticks_run, dt)
                                     : (Uint32)SDL_GetTicks();
    }

    /* dt for a low-pass filter a PLAYER must feel in real seconds (camera rumble,
       sky fade) but a SCRIPTED run must replay: under --sim-steps the frame rate
       is not part of the run, so the tick is its step and one draw per frame
       becomes one draw per tick. The wall-clock branch is exactly what it always
       was. #475. */
    double smoothingDt(Uint32 &last_ms) const {
        if(sim_steps_left >= 0) { return dt; }
        const Uint32 now_ms = SDL_GetTicks();
        const double d = (now_ms - last_ms) * 0.001;
        last_ms = now_ms;
        return d;
    }

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
    /* Air-relative prograde/retrograde (#498): the direction the AIR sees you
       moving, so the one the aero law actually answers to. Drawn only while there
       is air and the air-relative speed clears the flow floor, which is why a
       plane in flight shows a second diamond well off the green one -- at these
       speeds the green one is the ground's direction, not the plane's. */
    Billboard *air_prograde_indicator = nullptr;
    Billboard *air_retrograde_indicator = nullptr;

    // --- the draw toggles (the Settings window writes, render.cpp reads) --
    bool physics_debug_drawing = false;
    bool world_drawing = true;
    bool draw_starfield = true;
    bool draw_ref_circles = false;   // the render frame's XZ + XY rings
    bool draw_orbit_plane = false;   // orbital plane + reference plane + node line
    /* F4: force arrows on the active ship -- engine, RCS and the aero law
       (gameui.cpp drawForceOverlay). Debug-facing, so not a Settings
       checkbox and not saved: it should be off by default for a fresh
       session. */
    bool show_force_overlay = false;

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

    /* PILOT vs SAS on one axis (#421, #425 D-2): the Settings window's Controls
       section writes it, --sas-policy overrides settings.json inside
       apply_settings_game, tick.cpp mirrors it onto every ship each tick, and
       applyRotationForce applies it where the pilot's demand and the law's demand are
       still separate vectors. It lives here rather than on the Vehicle because it is
       one control preference the player owns, not a property of a build (#600 asks
       whether that is the right scope). */
    SasPolicy sas_policy = SasStickOwnsAxis;

    // --- the key map (keys.h) --------------------------------------------
    KeyBindings binds;
    // Slot the Controls window is rebinding; -1 = not capturing.
    int rebind_capture_slot = -1;
    // Thrust latch (events.cpp toggles; tick.cpp keeps thrust on while set).
    bool thrust_latched = false;

    /* The ascent autopilot (#400 -> AUTOPILOT v1): a law that flies a ship to a
       circular orbit. Empty until the Guidance window's Go (or --ascent) arms
       it; tick.cpp steps it after the key input and before the substeps. Held
       here rather than in Vehicle because it is a mission, not a control
       surface: it owns an aim and a cutoff, and it talks to the ship only
       through the public reads and Command, like any other hand on the controls.
       The law itself is not bound to anything -- it takes a state and returns an
       aim -- so the hull it was armed on lives next to it. Without this,
       switching ships mid-flight handed the autopilot to the new one, which
       launched itself out from under the player (#610 review). dropVehicleRefs
       drops both together: a law with no ship is not a mission, it is a landmine
       at a recycled address. */
    std::optional<AscentLaw> ascent;
    Vehicle *ascent_ship = nullptr;
    /* The Guidance window's fields, kept out of the law on purpose (#612): a
       mission ends and the player wants to see what they had asked for, tweak
       it and re-fly. armAscent() is the one door from these to a flying law, so
       --ascent and a Go click arm the same way. */
    AscentAsk ascent_ask;
    /* Which law the Guidance window is offering to arm next: the index into its
       "type" combo (GuidanceType / kGuidanceTypeItems in gameui.cpp), like the
       Telemetry cells' series indices in HudState. Game-owned rather than a
       static in the window body, so clearEphemeralState can put it back with the
       world it was chosen against: a stale "Transfer" selection opens the window
       with no apsides fields and a greyed Go, which reads as a broken window
       rather than as a choice made in some earlier game (#612 review). */
    int guidance_type = 0;
    /* Arm the ascent law from ascent_ask on this hull. Nothing may be flying
       when this is called: a second law would steer the same aim setpoint and
       the same throttle lever as the first, and arming one over the other would
       leave the first owning a hull nobody is driving any more. The Guidance
       window enforces that at Go; the assert in armAscent is the contract for
       the next caller, which is the descent law (#612's arbiter). The caller has
       also decided the ask is worth flying -- the window's guards, --ascent's
       parse -- because this is the action, not the judgement. */
    void armAscent(Vehicle *v);
    /* Put the hull back in the player's hands: let go of both channels the law
       was holding (releaseAscent -- the SlewFree latch and the throttle lever),
       then drop the law. The order matters and so does the first half: tick.cpp
       stops calling flyAscent the moment Game::ascent is empty, so its
       going-away pass never runs for a Cancel, and a mission cancelled in the
       middle of a burn would leave the hull steering at a dead law's aim with
       the SAS keys toggling a mode the player never set (#610 review). */
    void cancelAscent();

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

    /* --- control transitions (a keypress or a part/crew window button) --- */
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
    /* Space Center, no ship, paused. Cannot fail: startNewGame is the only
       caller and it refuses a running game first (asserted here). */
    void newGame();
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
    /* The only teardown. resetWorld = clearEphemeralState + the fleet + the
       warp (the title it lands on is at rest), in the one order that is safe
       (refs first, then the vehicles they borrow, then the bodies' owners get to
       die). No job drain: no background continuation dereferences a Vehicle. */
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
    // resetWorld + enterTitle: the hub's "Return to title".
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
    void openPartWindow(Vehicle *ship, size_t part, int mx, int my);
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
        /* Never empty: curSceneId reads back() unguarded. The seed is the Title
           because no game is in charge until main.cpp puts one in charge, which
           is what gameRunning() reports. So a ship boot logs its way in
           ("title -> flight") and a shipless one simply stays: no phantom
           flight to unwind, and every floor change is a logged decision. */
        sceneStack.reserve(4);
        sceneStack.push_back(SceneFrame{.id = SceneId::Title});
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
/* The first occupied seat, or nullptr -- the form for a caller that only needs
   to know whether anyone is aboard, so it does not allocate a vector to answer
   a yes/no question. */
Part *firstVehicleSeat(Vehicle *v);
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
