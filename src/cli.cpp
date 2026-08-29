// cli.cpp -- command-line parsing (CLI11) + the --sim-press / --sim-mouse
// folding. Fills GameArgs (cli.h); returns true on a successful parse,
// false with the process exit code in *exit_code otherwise (help, a
// parse error, malformed sim input).
#include "cli.h"

#include <cstdio>
#include <cstdlib>

#include <CLI11/CLI11.hpp>

bool parse_cli(int argc, char **argv, GameArgs &args, int *exit_code)
{
    CLI::App app{"Open Space Program"};

    app.add_option("--body", args.body_name,
        "Body the ship starts on / orbits (default: the system's home body)");

    app.add_option("--scenario", args.scenario,
        "Starting scenario: pad, pad-polar, rot-orbit, inertial-orbit, "
        "high-orbit, high-polar, ellipse-peri, ellipse-apo, ellipse-mid, "
        "escape (the ellipse-* scenarios are a 10x1000 km ASL orbit started "
        "at periapsis, apoapsis, or halfway by angle between them; escape "
        "is 2x escape velocity at the rot-orbit radius, coasting out of "
        "the body's SOI on its own; default: pad)")
        ->check(CLI::IsMember({"pad", "pad-polar", "rot-orbit",
                               "inertial-orbit", "high-orbit", "high-polar",
                               "ellipse-peri", "ellipse-apo", "ellipse-mid",
                               "escape"}));

    app.add_option("--system", args.system_file,
                   "Star-system JSON file to load (default: res/ksp_system.json; "
                   "try res/old_system.json for the Eerbon system)");

    app.add_option("--parts", args.parts_file,
                   "Parts catalog JSON (default: res/parts.json)");

    app.add_option("--ship", args.ship_files,
                   "Ship def JSON to build; repeat the flag to build more "
                   "ships (they share the body/scenario, each getting its "
                   "own pad slot / orbit slot). A uniform-fleet shorthand "
                   "-- --fleet overrides it. Default: res/ships/racer.json");

    app.add_option("--fleet", args.fleet_file,
                   "Fleet JSON (default: none; then --ship applies). One "
                   "entry per ship, each with its own ship def, name, body "
                   "and scenario; omitted body/scenario fall back to "
                   "--body/--scenario. Ships sharing a body+scenario get "
                   "their own pad slot / orbit slot. Try res/fleet.json");

    /* Spin-instrumentation mode: build a test ship (no JSON ship def)
       and log its spin + the internal contact torque each tick.
       radial     = part B welded to part A's side, axes PERPENDICULAR
       parallel   = part B welded to part A's side, axes PARALLEL
                    (side by side, off-axis anchor)
       stacked    = part B welded on A's axis (known-good baseline)
       stacks     = two 2-part stacks side by side, 2nd stack PERPENDICULAR
       parstacks  = two 2-part stacks side by side, ALL axes PARALLEL
       All parts are passive tanks (no wheels/thrusters), so any spin
       is self-inflicted. */
    app.add_option("--radial-test", args.radial_test,
                   "Build the spin-test ship(s) instead of a fleet: "
                   "radial | parallel | stacked | stacks | parstacks")
        ->check(CLI::IsMember({"radial", "parallel", "stacked", "stacks",
                               "parstacks"}));

    app.add_option("-t,--time-accel", args.initial_time_accel,
                   "Initial time acceleration (0 = paused, default 0)")
        ->check(CLI::NonNegativeNumber);

    app.add_option("--exhaust-scale", args.exhaust_scale,
                   "Scale the engines' exhaust velocity: thrust and delta-v "
                   "scale by it, the fuel burn does not (0.5-5.0, default 1; "
                   "adjustable in the Settings window)")
        ->check(CLI::Range(0.5f, 5.0f));

    app.add_option("--timeout", args.timeout_seconds,
                   "Auto-exit the main loop after this many wall-clock "
                   "seconds (0 = run until closed; default: 0)")
        ->check(CLI::NonNegativeNumber);

    std::vector<std::string> sim_press;
    app.add_option("--sim-press", sim_press,
                   "Synthetic keypresses for e2e testing: a flat list of "
                   "START_MS,DURATION_MS,KEY triples (e.g. 500,200,SPACE, "
                   "1500,100,I; spaces also separate values). KEY is an SDL "
                   "key name (A..Z, SPACE, TAB, F1-F12, ...) or a decimal "
                   "SDL keycode. The key is pressed START_MS after the main "
                   "loop starts and held for DURATION_MS. Repeat the flag "
                   "to append more triples.")
        ->delimiter(',');

    std::vector<std::string> sim_mouse;
    app.add_option("--sim-mouse", sim_mouse,
                   "Synthetic mouse input for e2e testing: a flat list of "
                   "TIME_MS,DURATION_MS,X,Y,BTN quintuples (e.g. "
                   "500,0,400,300,1 = click LMB at (400,300) after 500ms; "
                   "500,600,900,500,RMB = RMB-drag to (900,500) over 600ms "
                   "to orbit the camera; spaces also separate values). X,Y "
                   "are absolute window pixels (the cursor moves there; the "
                   "delta from the previous position drives the camera look: "
                   "yaw = -dx/200 rad, pitch = +dy/200 rad, 200px ~= 1 rad). "
                   "BTN is an SDL button code (1=LEFT, 2=MIDDLE, 3=RIGHT) or "
                   "name (L/LEFT/LMB, M/MIDDLE/MMB, R/RIGHT/RMB); 0/NONE = "
                   "move only. DURATION_MS>0 with a button = a drag (held); "
                   "0 = a quick click. Repeat the flag to append more "
                   "quintuples.")
        ->delimiter(',');

    app.add_flag("--selftest-spawn", args.selftest_spawn,
                 "Exercise the runtime spawn/remove path: spawn a copy of "
                 "the active ship, remove it, then spawn-select-remove the "
                 "active one (handoff), checking bookkeeping each step, and "
                 "exit after a few physics ticks");

    app.add_flag("--orbit-log", args.orbit_log,
                 "Periodically print the ship's orbital elements to stdout "
                 "(for measuring orbital stability)");

    app.add_option("--orbit-interval", args.orbit_interval,
                   "Wall-clock seconds between --orbit-log lines (default: 1)")
        ->check(CLI::PositiveNumber);

    app.add_flag("--dbg-log", args.dbg_log,
                 "Periodically print ship position/altitude/velocity "
                 "(surface-level companion to --orbit-log)");

    app.add_flag("--xfer-log", args.xfer_log,
                 "Periodically print the transfer planner's solution to "
                 "stdout (needs a target; --transfer-target selects one)");

    app.add_option("--transfer-target", args.transfer_target,
                   "Transfer planner target: a child body of the ship's "
                   "current body, or another ship in the same body");

    app.add_flag("--spin-log", args.spin_log_enabled,
                 "Periodically print the ship's spin diagnostics (per-part "
                 "angular velocities, inter-part contact impulses, tidal "
                 "torque) to stdout; also implied by --radial-test");

    app.add_option("--postfx", args.postfx_spec,
                   "Post-processing effect, in the order given; repeatable "
                   "and/or comma-separated (e.g. --postfx cas,grain). "
                   "Available: crt (retro tube look), grain (animated film "
                   "grain), cas (adaptive-contrast sharpening, 'sharpen' "
                   "also accepted). Omit for direct output (default)");

    app.add_flag("--gl-debug", args.gl_debug,
                 "Enable the OpenGL debug output callback (GL_DEBUG_* "
                 "messages print as they occur)");

    app.add_option("--width", args.screen_width,
                   "Window width in pixels (used with --borderless and "
                   "--exclusive; ignored with --fullscreen)")
        ->check(CLI::PositiveNumber);
    app.add_option("--height", args.screen_height,
                   "Window height in pixels (used with --borderless and "
                   "--exclusive; ignored with --fullscreen)")
        ->check(CLI::PositiveNumber);
    bool fullscreen = false;
    auto fs_opt = app.add_flag("--fullscreen", fullscreen,
                               "Start in borderless fullscreen at the "
                               "display's native resolution");
    bool borderless = false;
    auto bl_opt = app.add_flag("--borderless", borderless,
                               "Start as a borderless window (no title bar) "
                               "at --width/--height");
    bool exclusive = false;
    auto ex_opt = app.add_flag("--exclusive", exclusive,
                               "Exclusive fullscreen: change the display "
                               "mode to --width/--height (low latency, the "
                               "only way to go non-native on X11). Note: "
                               "SDL 2.32's X11 driver never restores the "
                               "previous mode on exit (X11_QuitModes is a "
                               "no-op), so restore it yourself with xrandr "
                               "if it matters");
    fs_opt->excludes(bl_opt);
    fs_opt->excludes(ex_opt);
    bl_opt->excludes(ex_opt);

    app.add_option("--font", args.font_path,
                   "TTF font file for all UI text; the normal and big faces "
                   "are the same font (the big one at twice --font-size; "
                   "default ./res/DejaVuSansMono.ttf)");

    app.add_option("--font-size", args.font_size,
                   "UI font size in pixels (the big HUD readout font is "
                   "twice this; default 14)")
        ->check(CLI::PositiveNumber);

    app.add_option("--frame-cap", args.frame_cap,
                   "Max render frames per second (0 = uncapped; default 60). "
                   "Without a cap the loop busy-spins between vsyncs, "
                   "idling a CPU core at 100% even while paused")
        ->check(CLI::NonNegativeNumber);

    app.add_option("--fov", args.camFovDeg,
                   "Camera vertical field of view in degrees (default 60; "
                   "adjustable in the Settings window)")
        ->check(CLI::Range(10.0f, 120.0f));

    app.add_option("--terrain-px", args.terrain_px,
                   "Terrain LOD: a patch subdivides while it projects "
                   "wider than this [screen px] (1024 = coarsest, default; "
                   "256 = good balance; 32 = finest, ~1px per mesh edge; "
                   "adjustable in the Settings window)")
        ->check(CLI::Range(32, 1024));

    // it's like a google maps link
    app.add_option("--free-cam-pos", args.free_cam_pos,
                   "Start in the free camera at this world position: X Y Z "
                   "(ship-frame coordinates)")
        ->expected(3);

    app.add_option("--free-cam-fwd", args.free_cam_fwd,
                   "Initial free camera forward direction: X Y Z "
                   "(normalised)")
        ->expected(3);

    app.add_option("--free-cam-up", args.free_cam_up,
                   "Initial free camera up direction: X Y Z (default: 0 1 0)")
        ->expected(3);

    // Call app.parse() directly, NOT the CLI11_PARSE macro: that macro does
    // its own `return (app).exit(e)`, which returns the *int* exit code and
    // would be implicitly converted to bool here (0=false, non-zero=true),
    // inverting help vs. error. Our own catch sets *exit_code and returns
    // false so main() can exit with the right code.
    try {
        app.parse(argc, argv);
    } catch(const CLI::ParseError &e) {
        *exit_code = app.exit(e);
        return false;
    }

    /* --sim-press: fold the flat START_MS,DURATION_MS,KEY list into press
       entries. */
    if(!sim_press.empty()) {
        if(sim_press.size() % 3 != 0) {
            printf("error: --sim-press expects START_MS,DURATION_MS,KEY "
                   "triples; got %zu value(s)\n", sim_press.size());
            *exit_code = 1;
            return false;
        }
        for(size_t i = 0; i < sim_press.size(); i += 3) {
            char *end = nullptr;
            const unsigned long t = strtoul(sim_press[i].c_str(), &end, 10);
            if(end == sim_press[i].c_str() || *end != '\0') {
                printf("error: --sim-press start time '%s' is not an "
                       "integer ms\n", sim_press[i].c_str());
                *exit_code = 1;
                return false;
            }
            const unsigned long d =
                strtoul(sim_press[i + 1].c_str(), &end, 10);
            if(end == sim_press[i + 1].c_str() || *end != '\0') {
                printf("error: --sim-press duration '%s' is not an "
                       "integer ms\n", sim_press[i + 1].c_str());
                *exit_code = 1;
                return false;
            }
            const SDL_Keycode k = sim_parse_key(sim_press[i + 2]);
            if(k == 0) {
                printf("error: --sim-press key '%s' is not a known SDL "
                       "keycode or name\n", sim_press[i + 2].c_str());
                *exit_code = 1;
                return false;
            }
            SimKeyPress p;
            p.down_ms = (Uint32)t;
            p.up_ms = (Uint32)t + (Uint32)d;
            p.key = k;
            p.sc = SDL_SCANCODE_UNKNOWN; // resolved in main once the video is up
            p.down_sent = false;
            p.up_sent = false;
            args.sim_presses.push_back(p);
        }
    }

    /* --sim-mouse: fold the flat TIME_MS,DURATION_MS,X,Y,BTN list into
       actions. X,Y are signed (the cursor can move up/left from where it
       was), so they parse as strtol, unlike the unsigned times above. */
    if(!sim_mouse.empty()) {
        if(sim_mouse.size() % 5 != 0) {
            printf("error: --sim-mouse expects TIME_MS,DURATION_MS,X,Y,BTN "
                   "quintuples; got %zu value(s)\n", sim_mouse.size());
            *exit_code = 1;
            return false;
        }
        for(size_t i = 0; i < sim_mouse.size(); i += 5) {
            char *end = nullptr;
            unsigned long v;
            v = strtoul(sim_mouse[i].c_str(), &end, 10);
            if(end == sim_mouse[i].c_str() || *end != '\0') {
                printf("error: --sim-mouse time '%s' is not an "
                       "integer ms\n", sim_mouse[i].c_str());
                *exit_code = 1;
                return false;
            }
            const unsigned long t = v;
            v = strtoul(sim_mouse[i + 1].c_str(), &end, 10);
            if(end == sim_mouse[i + 1].c_str() || *end != '\0') {
                printf("error: --sim-mouse duration '%s' is not an "
                       "integer ms\n", sim_mouse[i + 1].c_str());
                *exit_code = 1;
                return false;
            }
            const unsigned long d = v;
            v = (unsigned long)strtol(sim_mouse[i + 2].c_str(), &end, 10);
            if(end == sim_mouse[i + 2].c_str() || *end != '\0') {
                printf("error: --sim-mouse X '%s' is not an "
                       "integer pixel\n", sim_mouse[i + 2].c_str());
                *exit_code = 1;
                return false;
            }
            const int x = (int)v;
            v = (unsigned long)strtol(sim_mouse[i + 3].c_str(), &end, 10);
            if(end == sim_mouse[i + 3].c_str() || *end != '\0') {
                printf("error: --sim-mouse Y '%s' is not an "
                       "integer pixel\n", sim_mouse[i + 3].c_str());
                *exit_code = 1;
                return false;
            }
            const int y = (int)v;
            const int b = sim_parse_button(sim_mouse[i + 4]);
            if(b < 0) {
                printf("error: --sim-mouse button '%s' is not a known SDL "
                       "button code or name (0=none, 1=LEFT, 2=MIDDLE, "
                       "3=RIGHT)\n", sim_mouse[i + 4].c_str());
                *exit_code = 1;
                return false;
            }
            SimMouseAction a;
            a.time_ms = (Uint32)t;
            a.up_ms = (Uint32)t + (Uint32)d;
            a.x = x;
            a.y = y;
            a.button = (Uint8)b;
            a.started = false;
            a.released = false;
            args.sim_mouse_actions.push_back(a);
        }
    }

    // Any of the --free-cam-* options opts in to starting in free-cam mode.
    args.use_free_cam = !args.free_cam_pos.empty() || !args.free_cam_fwd.empty()
                            || !args.free_cam_up.empty();

    args.window_mode =
        exclusive  ? WindowMode::Exclusive
        : fullscreen ? WindowMode::Fullscreen
        : borderless ? WindowMode::Borderless
                     : WindowMode::Windowed;

    /* Whether --scenario was passed explicitly: the --radial-test block
       honors it, and otherwise defaults the test ship to an orbit. */
    args.scenario_given = app.get_option("--scenario") != nullptr
                        && app.get_option("--scenario")->count() > 0;

    return true;
}
