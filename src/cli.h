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

    std::string system_file = "res/ksp_system.json";
    std::string parts_file = "res/parts.json";
    std::vector<std::string> ship_files;
    std::string fleet_file;

    std::string radial_test;
    int initial_time_accel = 0;
    double timeout_seconds = 0.0;
    float exhaust_scale = 1.0f;  // test knob: scales ve (thrust + delta-v)
    // Control scheme (also a Settings-window option): 0 = screen-aligned
    // (pitch/yaw follow the screen, roll about the nose), 1 = heading-
    // aligned (all axes follow the ship). See Vehicle::applyRotationForce.
    int control_scheme = 0;

    std::vector<SimKeyPress> sim_presses;
    std::vector<SimMouseAction> sim_mouse_actions;
    int sim_mouse_x = 0;   // simulated cursor (window pixels); each motion
    int sim_mouse_y = 0;   // carries the delta from here for the camera look

    bool selftest_spawn = false;
    bool orbit_log = false;
    double orbit_interval = 1.0;
    bool dbg_log = false;
    bool xfer_log = false;
    std::string transfer_target;
    bool spin_log_enabled = false;

    std::vector<std::string> postfx_spec;
    bool gl_debug = false;

    int screen_width = 1920;
    int screen_height = 1080;
    WindowMode window_mode = WindowMode::Windowed;

    std::string font_path = "./res/DejaVuSansMono.ttf";
    float font_size = 14.0f;
    int frame_cap = 60;
    float camFovDeg = 60.0f;
    // terrain LOD: a patch subdivides while it projects wider than this
    // [px]. 1024 = coarsest (default; fastest startup, the e2e env
    // renders in software), 256 = visual sweet spot, 32 = finest.
    int terrain_px = 1024;

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
