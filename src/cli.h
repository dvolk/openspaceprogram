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
        bool cam_shake = false;
        bool time_accel = false;     // overrides a paused load
        bool sky_dim = false;
        bool sky_dim_cone = false;
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
    // Same capture as --save but legal with --load: the load -> fly -> save
    // round trip in one process (#51).
    std::string resave_name;
    std::string data_dir;   // user data directory; empty = per-OS default

    std::string radial_test;
    std::string autopilot;  // engage a slew mode at startup
    std::string vab;   // ship def to open in the VAB scene (empty = flight)
    bool vab_empty = false;
    std::string vab_arm;
    int vab_place_ms = -1;
    std::string vab_load;
    int vab_load_ms = -1;   // --vab-load-at
    int vab_launch_ms = -1;
    int vab_close_ms = -1;   // camera park/restore round trip
    int vab_detach_idx = -1;
    int vab_detach_ms = -1;  // --vab-detach-at
    std::string vab_save;
    int vab_save_ms = -1;   // --vab-save-at
    std::string reload_dir; // save dir to load over the running game
    int reload_ms = -1;     // --reload-at
    int new_game_ms = -1;
    int quit_title_ms = -1;
    int space_center_ms = -1;
    int recover_ms = -1;
    bool recover_anywhere = false;  // default: home body only
    int experiment_ms = -1;
    int pod_experiment_ms = -1;
    int eva_ms = -1;
    int take_ms = -1;         // the take/store dance's "Take"
    int store_ms = -1;        // the take/store dance's "Store"
    int tracking_ms = -1;
    int tracking_close_ms = -1;
    int research_ms = -1;
    int research_close_ms = -1;
    std::string switch_system_path;
    int switch_system_ms = -1;      // --switch-at
    std::string vab_scenario;  // VAB launch scenario (empty = "pad")
    std::string vab_body;      // VAB launch body (empty = home body)
    std::string vab_site;      // VAB launch site (building id)
    bool dev_scenarios = false;  // --dev-scenarios: the VAB dropdown's dev beds
    std::string dock_test;
    int initial_time_accel = 1;
    double start_time = 0.0;   // the sim clock's t0 (s)
    double timeout_seconds = 0.0;
    float exhaust_scale = 1.0f;  // difficulty: scales ve (thrust + delta-v)
    float cam_shake = 1.0f;   // camera shake at high accel (0 = off)
    double drag_cd = 1.2;       // the drag coefficient (0 = off)
    bool drag_log = false;      // the active ship's drag per tick

    std::vector<SimKeyPress> sim_presses;
    std::vector<SimMouseAction> sim_mouse_actions;
    int sim_mouse_x = 0;   // simulated cursor; each motion carries the delta from here
    int sim_mouse_y = 0;
    std::vector<SimModeChange> sim_mode_changes;

    std::vector<UiClick> ui_clicks;  // imgui clicks by "Window/Label"
    int ui_list_ms = -1;             // dump the clickable items
    int atlas_dump_ms = -1;          // print the System Atlas rows + dossiers
    int map_plane = -1;              // the orbit map's plane combo (0 equ, 1 ecl, 2 orb, 3 tgt)
    int map_dump_ms = -1;            // print the map's plane basis per slot

    bool orbit_log = false;
    bool plane_log = false;      // the drawn orbital/reference plane normals
    double orbit_interval = 1.0;
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
    bool att_log = false;          // the ship's nose + angular velocity
    bool shake_log = false;       // the cam-shake state per interval
    bool cam_log = false;         // the live camera pose + who ate the input
    bool audio_log = false;       // the [aud] audio-event trace
    bool terrain_log = false;     // the local body's patch-tree LOD
    bool tq_log = false;           // COM-lag x net-force spurious-torque probe
    bool fuel_log = false;        // per-fuel-group fuel mass + links
    bool drain_log = false;      // per-fuel-group drain rate (kg/s)
    bool power_log = false;      // power balance (gen/draw/pool/gate)
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
    int msaa_samples = 4;   // the window's sample count (0 = none)

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

    // Star-field exposure (render.cpp skyGain): the gain the sky fades to with
    // the sun in view (1 = off), and the half-angle of that cone in degrees.
    float sky_dim = 0.25f;
    float sky_dim_cone = 25.0f;
    bool sky_dim_log = false;     // the fade's inputs, 1/s

    std::string font_path = "res/fonts/DejaVuSansMono.ttf";
    float font_size = 14.0f;
    int frame_cap = 60;
    // Render rate while the window is occluded/minimized (0 = never idle).
    int idle_fps = 10;
    // Pretend the window is hidden from MS on (-1 = off).
    int force_occluded_ms = -1;
    int vsync = 1;           // 0 = off, 1 = one refresh per swap
    double physics_hz = 60.0;  // the fixed logic tick rate
    bool perf = false;   // print a per-frame phase timing breakdown
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
