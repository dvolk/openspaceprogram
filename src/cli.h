#pragma once

#include <string>
#include <vector>

#include "display.h"   // WindowMode
#include "siminput.h"  // SimKeyPress, SimMouseAction

/* Everything the CLI flags configure, filled by parse_cli (cli.cpp).
   The sim_* state doubles as the live state for synthetic input. */
struct GameArgs {
    std::string title_body;   // --title-body: pin the title-screen backdrop (test hook)
    std::string surfmap_body;  // --surfmap-body: pin the Surface Map's body at boot

    // Settings the command line set explicitly; the CLI beats settings.json
    // field by field. A flag not listed here means the file may apply.
    struct {
        bool window_mode = false;    // --fullscreen / --borderless / --exclusive
        bool width = false;          // --width
        bool height = false;         // --height
        bool msaa = false;           // --msaa
        bool postfx = false;         // --postfx (the whole effect set)
        bool fov = false;            // --fov
        bool terrain_px = false;     // --terrain-px
        bool exhaust_scale = false;  // --exhaust-scale
        bool cam_shake = false;      // --cam-shake
        bool time_accel = false;     // --time-accel (overrides a paused load)
    } cli_given;

    std::string system_file = "res/systems/ksp_system.json";
    std::string parts_file = "res/data/parts.json";
    std::string experiments_file = "res/data/experiments.json";
    // Start ships: --startship is one inline entry "name,def,body,scenario";
    // --startships is the same list as a JSON file.
    std::vector<std::string> startship;
    std::string startships_file;

    // --save captures the game into a directory when --timeout is spent;
    // --load replaces the fleet at startup. Bare names resolve under saves/.
    std::string save_name;
    std::string load_name;
    std::string data_dir;   // --data-dir: user data directory; empty = per-OS default

    std::string radial_test;
    std::string autopilot;  // --autopilot: engage a slew mode at startup (test hook)
    std::string vab;   // ship def to open in the VAB editor scene (empty = flight)
    bool vab_empty = false;  // --vab-empty: open the VAB with an EMPTY build
    std::string vab_arm;   // catalog part to arm in the VAB palette at startup (test hook)
    int vab_place_ms = -1; // --vab-place: fire vabPlace once at this loop time (test hook)
    std::string vab_load;   // --vab-load: ship def to load into the VAB (test hook)
    int vab_load_ms = -1;   // --vab-load-at: loop time for --vab-load (test hook)
    int vab_launch_ms = -1;  // --vab-launch: fire the VAB's LAUNCH (test hook)
    int vab_close_ms = -1;   // --vab-close: fire vabClose (camera park/restore round trip)
    int vab_detach_idx = -1; // --vab-detach: select this build part and fire vabDetachSelected
    int vab_detach_ms = -1;  // --vab-detach-at: loop time for --vab-detach
    std::string vab_save;   // --vab-save: save the build under this name (test hook)
    int vab_save_ms = -1;   // --vab-save-at: loop time for --vab-save (test hook)
    std::string reload_dir; // --reload: save dir to load over the running game
    int reload_ms = -1;     // --reload-at: loop time for --reload
    int new_game_ms = -1;   // --new-game: fire Game::newGame (test hook)
    int quit_title_ms = -1; // --quit-title: fire Game::quitToTitle (test hook)
    int space_center_ms = -1; // --space-center: push the Space Center hub (test hook)
    int recover_ms = -1;    // --recover: fire Game::recoverActive (test hook)
    bool recover_anywhere = false;  // --recover-anywhere: recover anywhere (default: home body only)
    int experiment_ms = -1; // --experiment: fire Game::runExperiment (test hook)
    int pod_experiment_ms = -1; // --pod-experiment: fire Game::runPodExperiment (test hook)
    int eva_ms = -1;          // --eva: fire Game::kerbalEVA (test hook)
    int take_ms = -1;         // --take: fire the take/store dance's "Take" (test hook)
    int store_ms = -1;        // --store: fire the take/store dance's "Store" (test hook)
    int tracking_ms = -1;   // --tracking: push the Tracking Station (test hook)
    int tracking_close_ms = -1; // --tracking-close: pop the Tracking Station (test hook)
    int research_ms = -1;   // --research: push the Research Lab (test hook)
    int research_close_ms = -1; // --research-close: pop the Research Lab (test hook)
    std::string switch_system_path; // --switch-system: system JSON to swap to (Game::switchSystem)
    int switch_system_ms = -1;      // --switch-at: loop time for --switch-system
    std::string vab_scenario;  // --vab-scenario: seed the VAB launch scenario (empty = "pad")
    std::string vab_body;      // --vab-body: seed the VAB launch body (empty = home body)
    std::string dock_test;
    int initial_time_accel = 1;
    double start_time = 0.0;   // --start-time: the sim clock's t0 (s)
    double timeout_seconds = 0.0;
    float exhaust_scale = 1.0f;  // difficulty: scales ve (thrust + delta-v)
    float cam_shake = 1.0f;   // camera shake at high accel (0 = off)
    double drag_cd = 1.2;       // --drag-cd: the drag coefficient (0 = off)
    bool drag_log = false;      // --drag-log: the active ship's drag per tick

    std::vector<SimKeyPress> sim_presses;
    std::vector<SimMouseAction> sim_mouse_actions;
    int sim_mouse_x = 0;   // simulated cursor; each motion carries the delta from here
    int sim_mouse_y = 0;
    std::vector<SimModeChange> sim_mode_changes;   // --sim-mode

    std::vector<UiClick> ui_clicks;  // --ui-click: imgui clicks by "Window/Label"
    int ui_list_ms = -1;             // --ui-list: dump the clickable items at this loop time

    bool selftest_spawn = false;
    bool orbit_log = false;
    double orbit_interval = 1.0;
    bool dbg_log = false;
    bool info_log = false;       // --info-log: dump Orbit/Surface Info to stdout
    bool debug_accel = false;   // --debug-accel: per-substep thrust/velocity dump
    bool xfer_log = false;
    bool porkchop_log = false;   // --porkchop-log: the launch-window grid min
    int porkchop_n = 40;        // --porkchop-n: the plot grid size

    bool eva_log = false;        // --eva-log: the kerbal's mode + pos/vel
    bool surfmap_log = false;    // --surfmap-log: the map's albedo/shaded means
    int surfmap_n = 256;        // --surfmap-n: the map width (h = n/2)
    bool surfmap_noshade = false; // --surfmap-noshade: skip the terminator
    std::string transfer_target;
    bool spin_log_enabled = false;
    bool slew_log_enabled = false;  // log the prograde/retrograde autopilot state
    bool att_log = false;          // log the ship's nose + angular velocity
    bool shake_log = false;       // --shake-log: the cam-shake state per interval
    bool terrain_log = false;     // --terrain-log: the local body's patch-tree LOD
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

    // Cloud deck sphere resolution (lat = lon = res rings). The detail lives
    // in the baked coverage map; this only changes the silhouette at the rim.
    int cloud_mesh = 128;

    // Render toggles (debug knobs for isolating terrain gaps / depth issues).
    bool no_clouds = false;       // --no-clouds
    bool no_atmosphere = false;   // --no-atmosphere
    bool no_ocean = false;        // --no-ocean
    bool no_rings = false;        // --no-rings

    std::string font_path = "res/fonts/DejaVuSansMono.ttf";
    float font_size = 14.0f;
    int frame_cap = 60;
    bool perf = false;   // --perf: print a per-frame phase timing breakdown
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
