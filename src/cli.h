#pragma once

#include <string>
#include <vector>

#include "display.h"   // WindowMode
#include "siminput.h"  // SimKeyPress, SimMouseAction

/* Everything the CLI flags configure, filled by parse_cli (cli.cpp),
   which returns 0 on success or the process exit code on failure (help,
   a parse error, malformed --sim-press / --sim-mouse). The sim_* state
   doubles as the live state for the synthetic input the event loop emits. */
struct GameArgs {
    std::string body_name;
    std::string scenario = "pad";
    bool scenario_given = false;    // --scenario was passed explicitly

    // The settings (Settings window) the command line set explicitly
    // (filled in parse_cli from CLI11 ->count()): Game::load_settings()
    // must not overwrite these -- the CLI beats settings.json field by
    // field. A flag not listed here (absent) means the file may apply.
    struct {
        bool window_mode = false;    // --fullscreen / --borderless / --exclusive
        bool width = false;          // --width
        bool height = false;         // --height
        bool msaa = false;           // --msaa
        bool postfx = false;         // --postfx (the whole effect set)
        bool fov = false;            // --fov
        bool terrain_px = false;     // --terrain-px
        bool exhaust_scale = false;  // --exhaust-scale
    } cli_given;

    std::string system_file = "res/ksp_system.json";
    std::string parts_file = "res/parts.json";
    std::vector<std::string> ship_files;
    std::string fleet_file;

    std::string radial_test;
    std::string dock_test;
    int initial_time_accel = 0;
    double start_time = 0.0;   // --start-time: the sim clock's t0 (s)
    double timeout_seconds = 0.0;
    float exhaust_scale = 1.0f;  // test knob: scales ve (thrust + delta-v)
    double drag_cd = 1.2;       // --drag-cd: the drag coefficient (0 = off)
    bool drag_log = false;      // --drag-log: the active ship's drag per tick

    std::vector<SimKeyPress> sim_presses;
    std::vector<SimMouseAction> sim_mouse_actions;
    int sim_mouse_x = 0;   // simulated cursor (window pixels); each motion
    int sim_mouse_y = 0;   // carries the delta from here for the camera look
    std::vector<SimModeChange> sim_mode_changes;   // --sim-mode

    bool selftest_spawn = false;
    bool orbit_log = false;
    double orbit_interval = 1.0;
    bool dbg_log = false;
    bool debug_accel = false;   // --debug-accel: per-substep thrust/velocity dump
    bool xfer_log = false;
    bool porkchop_log = false;   // --porkchop-log: the launch-window grid min
    int porkchop_n = 40;        // --porkchop-n: the plot grid (porkchop_n x
                                // porkchop_n); the size knob (a Settings-
                                // window hook later). 40 x 40 is ~15 ms.

    bool eva_log = false;        // --eva-log: the kerbal's mode + pos/vel
    bool surfmap_log = false;    // --surfmap-log: the map's albedo/shaded means
    int surfmap_n = 256;        // --surfmap-n: the map width (h = n/2)
    bool surfmap_noshade = false; // --surfmap-noshade: skip the terminator
    std::string transfer_target;
    bool spin_log_enabled = false;
    bool slew_log_enabled = false;  // log the prograde/retrograde autopilot state
    bool att_log = false;          // log the ship's nose + angular velocity
    bool tq_log = false;           // --tq-log: COM-lag x net-force spurious-torque probe
    bool fuel_log = false;        // --fuel-log: per-fuel-group fuel mass + links
    bool drain_log = false;      // --drain-log: per-fuel-group drain rate (kg/s)
    bool power_log = false;      // --power-log: power balance (gen/draw/pool/gate)
    bool prox_log = false;       // --prox-log: proximity engage/release + distances
    double prox_fly_on = 2000.0;    // --prox-fly-on: engage radius while the active ship flies (m)
    double prox_fly_off = 10000.0;  // --prox-fly-off: release radius while flying (m)
    double prox_ground_on = 10.0;   // --prox-ground-on: engage radius while grounded (m; 0 = never auto-wake)
    double prox_ground_off = 20.0;  // --prox-ground-off: release radius while grounded (m)
    double prox_warp = 1.0;         // --prox-warp: max time accel while a ship is engaged
    bool compound_check = false;   // --compound-check: the ship-as-one-rigid-body gate

    std::vector<std::string> postfx_spec;
    bool gl_debug = false;
    int msaa_samples = 4;   // --msaa: the window's sample count (0 = none)

    int screen_width = 1920;
    int screen_height = 1080;
    WindowMode window_mode = WindowMode::Windowed;

    // Cloud deck sphere resolution (lat = lon = res rings), applied at
    // load by BuildClouds. The detail lives in the baked coverage map,
    // so this only changes the deck's silhouette smoothness at the rim.
    int cloud_mesh = 128;

    std::string font_path = "./res/DejaVuSansMono.ttf";
    float font_size = 14.0f;
    int frame_cap = 60;
    bool perf = false;   // --perf: print a per-frame phase timing breakdown
    float camFovDeg = 60.0f;
    // terrain LOD: a patch subdivides while it projects wider than this
    // [px]. 1024 = coarsest (default; fastest startup, the e2e env
    // renders in software), 256 = visual sweet spot, 32 = finest.
    int terrain_px = 512;

    std::vector<double> free_cam_pos;
    std::vector<double> free_cam_fwd;
    std::vector<double> free_cam_up;
    bool use_free_cam = false;
};

/* Fills args from the command line. Returns true on a successful parse.
   For --help / --version or any invalid input, prints the message and
   returns false with the process exit code in *exit_code (0 for help).
   Note the asymmetry: a successful parse returns true, NOT 0, because
   CLI11's help path also "returns" 0. */
bool parse_cli(int argc, char **argv, GameArgs &args, int *exit_code);
