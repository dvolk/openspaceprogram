#pragma once

#include <string>
#include <vector>

#include "display.h"   // WindowMode
#include "siminput.h"  // SimKeyPress, SimMouseAction

/* Everything the CLI flags configure, filled by parse_cli (cli.cpp).
   The sim_* state doubles as the live state for synthetic input; the *_ms
   fields are e2e test hooks. */
struct GameArgs {
    std::string title_body;   // pin the title-screen backdrop
    std::string surfmap_body;  // pin the Surface Map's body at boot
    std::string surfmap_map;   // pin the Surface Map's picture at boot (#745)

    // Settings the command line set explicitly: the CLI beats settings.json
    // field by field. A flag not listed here means the file may apply.
    struct {
        bool window_mode = false;    // --fullscreen / --borderless / --exclusive
        bool width = false;
        bool height = false;
        bool msaa = false;
        bool postfx = false;         // the whole effect set
        bool fov = false;
        bool terrain_px = false;
        bool exhaust_scale = false;
        bool start_time = false;   // an explicit epoch: a new game keeps it
        bool cam_shake = false;
        bool time_accel = false;     // overrides a paused load
        bool sky_dim = false;
        bool sky_dim_cone = false;
        bool frame_cap = false;    // --sim-steps uncaps the frame unless asked
        bool idle_fps = false;     // ...and skips the occluded-window throttle
        bool sas_policy = false;   // --sas-policy beats settings.json
    } cli_given;

    std::string system_file = "res/systems/ksp_system.json";
    std::string parts_file = "res/data/parts.json";
    std::string experiments_file = "res/data/experiments.json";
    std::string kerbals_file = "res/data/kerbals.json";
    /* Start ships: --startship is one inline entry "name,def,body,scenario" plus
       optional named params (site=, lat=, lon=); --startships is the same list as a
       JSON file. main.cpp's splitStartship is the parser; DebugStartShip (shipdef.h)
       is what it fills. */
    std::vector<std::string> startship;
    std::string startships_file;

    // --load replaces the ships at startup. Bare names resolve under saves/.
    std::string load_name;
    std::string data_dir;   // user data directory; empty = per-OS default
    /* --allow-part-geom-drift: load a ship whose recorded part geometry no longer
       matches the parts file instead of refusing it (#456). The poses are what
       the save solved; the parts are today's shapes, so the ship may be reshaped
       in ways the sim will happily fly. */
    bool allow_part_geom_drift = false;

    std::string radial_test;
    std::string autopilot;  // engage a slew mode at startup
    /* Fly the active ship to a circular orbit: "ALT[,INC]", metres and degrees.
       The ascent autopilot -- an aim, a throttle and a cutoff, all through the
       same command door a key uses (#424). No panel for it yet. */
    std::string ascent;
    std::string sas_policy;  // pilot-vs-SAS on one axis: "axis" | "keep" (#421)
    /* #678's dial from the command line, deg/s: the ceiling on how fast an attitude
       LAW may swing the active hull. 0 = unlimited (the default). It sets the same
       field the Systems window's slider writes, so a case and a click cannot disagree. */
    double sas_rate = 0.0;
    bool recover_anywhere = false;  // default: home body only
    bool dev_scenarios = false;  // --dev-scenarios: the VAB dropdown's dev beds
    bool fleet_report = false;   // --fleet-report: print the airworthiness table and exit
    std::string dock_test;
    int initial_time_accel = 1;
    double start_time = 0.0;   // the sim clock's t0 (s)
    double timeout_seconds = 0.0;
    /* --sim-steps: run exactly this many logic ticks, then exit (-1 = off). A
       scripted e2e flight's length in TICKS rather than wall seconds, so two runs
       step identically and the clocks the run reads switch to sim time (#472). */
    long long sim_steps = -1;
    float exhaust_scale = 1.0f;  // difficulty: scales ve (thrust + delta-v)
    float cam_shake = 1.0f;   // camera shake at high accel (0 = off)
    double drag_cd = 1.2;       // the drag BILL multiplier: the hull's form drag,
                                // the surfaces' own shape drag (Profile=) and the
                                // induced drag that lift pays -- never the lift
                                // itself (#502, #455). 0 = off
    bool drag_log = false;      // the active ship's drag per tick
    /* #687: the star's light per tick -- distance, flux, elevation, and what the
       body underneath asserts (authored albedo / surface temperature, and the
       black-body temperature the flux implies). Read-only: nothing in the physics
       consumes it yet (#553's part heat is the first buyer). */
    bool solar_log = false;

    std::vector<SimKeyPress> sim_presses;
    std::vector<SimMouseAction> sim_mouse_actions;
    int sim_mouse_x = 0;   // simulated cursor; each motion carries the delta from here
    int sim_mouse_y = 0;
    std::vector<UiFocus> ui_focuses;   // park that cursor on a named part
    std::vector<SimModeChange> sim_mode_changes;

    std::vector<UiClick> ui_clicks;  // imgui clicks by "Window/Label"
    std::vector<UiScroll> ui_scrolls;  // imgui wheel scrolls by window
    std::vector<int> ui_list_ms;     // dump the clickable items, at each time given
    int atlas_dump_ms = -1;          // print the System Atlas rows + dossiers
    int staff_dump_ms = -1;          // print the Staff Lounge roster + locations
    int map_plane = -1;              // the orbit map's plane combo (0 equ, 1 ecl, 2 orb, 3 tgt)
    int map_dump_ms = -1;            // print the map's plane basis per slot

    bool orbit_log = false;
    bool ascent_log = false;   // the ascent law's phase, apsides and cutoff
    bool plane_log = false;      // the drawn orbital/reference plane normals
    double orbit_interval = 1.0;
    /* The flight recorder (#662): one file per run, beside the save slots when
       the game has claimed a game dir and under the data dir's flights/ when it
       has not. Rows land on the recorder's own gate, not --orbit-interval's. */
    bool record = false;
    double record_interval = 0.25;   // seconds of sim between recorded rows
    bool dbg_log = false;
    bool info_log = false;       // dump Orbit/Surface Info to stdout
    bool debug_accel = false;   // per-substep thrust/velocity dump
    bool xfer_log = false;
    bool porkchop_log = false;   // the launch-window grid min
    int porkchop_n = 256;
    std::vector<int> porkchop_bench; // sweep these sizes off one snapshot
    std::string porkchop_dump;       // write each grid as CSV

    bool eva_log = false;        // the kerbal's mode + pos/vel
    bool surfmap_log = false;    // the map's albedo/shaded means
    int surfmap_n = 256;        // the map width (h = n/2)
    bool surfmap_noshade = false; // skip the terminator
    std::string transfer_target;
    bool spin_log_enabled = false;
    bool slew_log_enabled = false;  // the prograde/retrograde autopilot state
    bool att_log = false;          // the ship's frame axes + angular velocity
    bool shake_log = false;       // the cam-shake state per interval
    bool cam_log = false;         // the live camera pose + who ate the input
    bool audio_log = false;       // the [aud] audio-event trace
    bool terrain_log = false;     // the local body's patch-tree LOD
    bool tq_log = false;           // COM-lag x net-force spurious-torque probe
    bool fuel_log = false;        // per-fuel-group fuel mass + links
    bool drain_log = false;      // per-fuel-group drain rate (kg/s)
    bool dv_log = false;         // the ship's delta-v budget (spent vs delivered)
    bool mass_log = false;       // every vehicle's mass, active or not
    bool power_log = false;      // power balance (gen/draw/pool/gate)
    bool steer_log = false;      // attitude bill: what it may spend, asked, spent
    bool prox_log = false;       // proximity engage/release + distances
    double prox_fly_on = 2000.0;    // engage radius while the active ship flies (m)
    double prox_fly_off = 10000.0;  // release radius while flying (m)
    double prox_ground_on = 10.0;   // engage radius while grounded (m; 0 = never auto-wake)
    double prox_ground_off = 20.0;  // release radius while grounded (m)
    double prox_warp = 1.0;         // max time accel while a ship is engaged
    bool compound_check = false;   // the ship-as-one-rigid-body gate

    std::vector<std::string> postfx_spec;
    bool gl_debug = false;
    // The glGetError drains without the debug callback (gldebug.h). The e2e
    // runner sets it for cases that FORBID GL_.
    bool gl_error_checks = false;
    int msaa_samples = 4;   // the scene target's sample count (0 = none)

    int screen_width = 1920;
    int screen_height = 1080;
    WindowMode window_mode = WindowMode::Windowed;

    // Cloud deck sphere resolution (lat = lon = res rings); the detail lives
    // in the baked coverage map.
    int cloud_mesh = 128;

    // Render toggles (debug knobs for isolating terrain gaps / depth issues).
    bool no_clouds = false;
    bool no_atmosphere = false;
    bool no_ocean = false;
    bool no_rings = false;
    bool no_skirt = false;

    // Star-field exposure (render.cpp skyGain): the gain the sky fades to with
    // the sun in view (1 = off), and the half-angle of that cone in degrees.
    float sky_dim = 0.25f;
    float sky_dim_cone = 25.0f;
    bool sky_dim_log = false;     // the fade's inputs, 1/s

    std::string font_path = "res/fonts/DejaVuSansMono.ttf";
    float font_size = 14.0f;
    int frame_cap = 60;
    /* Skip the 3D paint pass (#212's paint half): the state pass, the logic
       tick and the UI still run, so only pixels go away. For runs that read
       logs, not the window (#666: a scripted frame is ~95% render). */
    bool no_paint = false;
    // Render rate while the window is occluded/minimized (0 = never idle).
    int idle_fps = 10;
    // Pretend the window is hidden from MS on (-1 = off).
    int force_occluded_ms = -1;
    int vsync = 1;           // 0 = off, 1 = one refresh per swap
    double physics_hz = 60.0;  // the fixed logic tick rate
    bool perf = false;   // print a per-frame phase timing breakdown
    /* --perf-gpu: pay for the GPU's own completion at the end of the 3D pass, so
       --perf's paint half says what the frame cost to DRAW and not just what it
       cost to submit. Without it every phase is a submission time: the GPU runs
       behind, and on a real window its work hides in the frame cap's sleep, where
       nothing measures it. The stall breaks the CPU/GPU overlap, so the frame rate
       drops -- it is a ruler for A/B-ing a render change, not a mode to fly in. */
    bool perf_gpu = false;
    /* Seconds of run time between [perf] rows (#676). The rolling window was
       prose on a hard-coded 1 s: easy to read, impossible to diff. A tagged row
       per gate is what makes "did this build make the frame cheaper" a question
       with an answer, and unlike the prose line it carries render's halves, which
       until now only the exit summary broke down. */
    double perf_interval = 1.0;
    bool pose_jitter = false;  // the drawn pose per render frame
    float camFovDeg = 60.0f;
    // terrain LOD: a patch subdivides while it projects wider than this [px].
    int terrain_px = 512;

    std::vector<double> free_cam_pos;
    std::vector<double> free_cam_fwd;
    std::vector<double> free_cam_up;
    bool use_free_cam = false;
};

/* Fills args from the command line. Returns true on success; false with the
   process exit code in *exit_code for --help / --version or invalid input. */
bool parse_cli(int argc, char **argv, GameArgs &args, int *exit_code);

/* Cadence of the periodic state logs, in WALL-clock ms: 2Hz for the books
   (--spin-log, --mass-log, --fuel-log, --drain-log, --power-log), 10Hz for
   --slew-log, whose error curve is undersampled slower than that. They keep
   their own rate rather than following --orbit-interval. tick.cpp gates on
   these; --help states them; tests/test_cli.cpp fails if the two drift apart. */
inline constexpr int kStateLogMs = 500;
inline constexpr int kSlewLogMs = 100;

/* The longest wait a cadence knob may ask for, in seconds of run time (#673).
   Every periodic log counts its interval in Uint32 MILLISECONDS, so a wait past
   ~49.7 days is UB and lands on an arbitrary rate. Measured on this box with the
   cast the code used to do inline: 4294968 s (49.7 days) became 704 ms, and 1e18 s
   became 0 ms -- every throttled log firing on every poll. An hour between rows is
   already a strange request, so the law sits two orders of magnitude under the
   wrap. parse_cli refuses past it; the clamp below is the belt for a caller that
   did not come through the CLI. */
inline constexpr double kIntervalCeilingS = 3600.0;

/* Seconds of run time -> the ms a log gate waits. One helper for every interval
   knob, so none of them re-derives the cast and its range (#673). A non-positive
   or NaN interval is 0 ms, which is what the gates already treat as "no throttle";
   an infinite one saturates at the ceiling rather than wrapping. */
inline unsigned gateIntervalMs(double seconds) {
    if(!(seconds > 0.0)) { return 0u; }
    if(seconds > kIntervalCeilingS) { seconds = kIntervalCeilingS; }
    return (unsigned)(seconds * 1000.0);
}
