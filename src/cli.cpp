// cli.cpp -- command-line parsing (CLI11) + the --sim-press / --sim-mouse
// folding. Fills GameArgs (cli.h); returns true on a successful parse,
// false with the process exit code in *exit_code otherwise (help, a
// parse error, malformed sim input).
#include "cli.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>

#include <CLI11/CLI11.hpp>

bool parse_cli(int argc, char **argv, GameArgs &args, int *exit_code)
{
    CLI::App app{"Open Space Program"};

    app.add_option("--body", args.body_name,
        "Body the ship starts on / orbits (default: the system's home body)");

    /* The name list is duplicated below (the help text and the IsMember
       validator) and again in vehicle.cpp's kScenarios, which is the real
       source of truth -- scenario_by_name() lists them when it throws.
       Unifying would mean exposing the names from vehicle.h, and that pulls
       body.h (Bullet + GL) into this deliberately dependency-free
       translation unit. So: add a scenario in ALL THREE places. */
    app.add_option("--scenario", args.scenario,
        "Starting scenario: pad, pad-polar, rot-orbit, inertial-orbit, "
        "high-orbit, high-polar, ellipse-peri, ellipse-apo, ellipse-mid, "
        "escape, neptune, oort (the ellipse-* scenarios are a 10x1000 km "
        "ASL orbit started at periapsis, apoapsis, or halfway by angle "
        "between them; escape is 2x escape velocity at the rot-orbit "
        "radius, coasting out of the body's SOI on its own; neptune / oort "
        "are circular orbits at an absolute 4.495e12 / 1e15 m from the body "
        "centre -- real-solar-system distances, for precision testing; use "
        "them with --body Kerbol, since around a planet the ship inherits "
        "that planet's orbital velocity and is hyperbolic w.r.t. the star; "
        "default: pad)")
        ->check(CLI::IsMember({"pad", "pad-polar", "rot-orbit",
                               "inertial-orbit", "high-orbit", "high-polar",
                               "ellipse-peri", "ellipse-apo", "ellipse-mid",
                               "escape", "neptune", "oort"}));

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

    /* Docking pair: probe (active) + station, nose-to-nose in the same
       orbit (see src/docktest.cpp). near = inside the capture window,
       it docks on the first live tick; approach = outside it, the player
       burns prograde to close the last stretch. */
    app.add_option("--dock-test", args.dock_test,
                   "Build the docking pair (probe + station) instead of a "
                   "fleet: near | approach")
        ->check(CLI::IsMember({"near", "approach"}));

    app.add_option("-t,--time-accel", args.initial_time_accel,
                   "Initial time acceleration (0 = paused, default 0)")
        ->check(CLI::NonNegativeNumber);

    app.add_option("--start-time", args.start_time,
                   "Initial sim time in seconds: start the clock (and every "
                   "body's orbit and spin, which are functions of it) at a "
                   "later instant, e.g. to put a pad in daylight or line up "
                   "a transfer window (default 0)")
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
                   "move only; 4/WHEEL_UP and 5/WHEEL_DOWN = one wheel notch "
                   "(zoom in / out in orbit camera; X,Y and DURATION "
                   "ignored, one notch per entry). DURATION_MS>0 with a "
                   "button = a drag (held); 0 = a quick click. Repeat the "
                   "flag to append more quintuples.")
        ->delimiter(',');

    std::vector<std::string> sim_mode;
    app.add_option("--sim-mode", sim_mode,
                   "Synthetic display-mode change for e2e testing: a flat "
                   "list of TIME_MS,MODE,WIDTH,HEIGHT quadruples (e.g. "
                   "2000,borderless,1024,768,5000,exclusive,640,480; "
                   "spaces also separate values). MODE is "
                   "windowed|borderless|fullscreen|exclusive; WIDTH / "
                   "HEIGHT of 0 keep the launch --width/--height (native "
                   "fullscreen ignores both). Each change is applied "
                   "TIME_MS after the main loop starts, through the same "
                   "Renderer::setWindowMode path the Settings dropdowns "
                   "use. Repeat the flag to append more quadruples.")
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

    app.add_flag("--porkchop-log", args.porkchop_log,
                 "Print the porkchop plot's best cell (min dv, its departure "
                 "delay + time of flight) to stdout whenever the plot is "
                 "computed (P key or the window's button)");
    app.add_option("--porkchop-n", args.porkchop_n,
                   "Porkchop plot grid size (porkchop-n x porkchop-n); the "
                   "size knob, a Settings-window hook later (default 40, "
                   "~15 ms at 40 x 40)");

    app.add_flag("--surfmap-log", args.surfmap_log,
                 "Print the surface map's albedo + shaded RGB means to stdout "
                 "whenever the map is computed (M key or the window's button)");
    app.add_option("--surfmap-n", args.surfmap_n,
                   "Surface map width in pixels (height = surfmap-n / 2); "
                   "default 256 (a 256 x 128 sweep is tens of ms)");
    app.add_flag("--surfmap-noshade", args.surfmap_noshade,
                 "Skip the terminator (day/night shading) when computing the "
                 "surface map (the window's 'Sun shading' box does the same "
                 "at runtime)");

    app.add_flag("--spin-log", args.spin_log_enabled,
                 "Periodically print the ship's spin diagnostics (per-part "
                 "angular velocities, inter-part contact impulses, tidal "
                 "torque) to stdout; also implied by --radial-test");

    app.add_flag("--slew-log", args.slew_log_enabled,
                 "Periodically print the prograde/retrograde autopilot "
                 "state (slew error angle, angular velocity split into the "
                 "slew/roll/third axes, braking-curve rate) to stdout; "
                 "the instrument for hunting the prograde wobble");

    app.add_flag("--att-log", args.att_log,
                 "Periodically print the ship's nose direction and the "
                 "hull's angular velocity (world coords) to stdout; the "
                 "instrument for the attitude-physics e2e test");

    app.add_flag("--tq-log", args.tq_log,
                 "Print the spurious-torque probe once per tick: the hull "
                 "origin's COM lag (|dcom|), the net force (|F|), the "
                 "|dcom x F| torque the fix cancels, and |w| -- the "
                 "instrument for the com-torque e2e test");

    app.add_flag("--fuel-log", args.fuel_log,
                 "Periodically print each fuel group's fuel mass (per "
                 "resource, plus each member tank) and the ship's fuel "
                 "links to stdout; the instrument for the fuel-link "
                 "drain-rate bug (symmetric radial groups must stay equal)");

    app.add_flag("--drain-log", args.drain_log,
                 "Periodically print each fuel group's drain rate (kg/s) "
                 "to stdout -- the change in a group's fuel mass between "
                 "samples; the instrument for how fuel is flowing (which "
                 "groups feed the engines and at what rate)");

    app.add_flag("--power-log", args.power_log,
                 "Periodically print the ship's electrical balance to "
                 "stdout -- generation (W), constant draw (W), stored "
                 "charge (Wh) and the wheel gate -- the instrument for "
                 "'is the ship losing power?' (life support + the active "
                 "wheels drain the battery; an RTG charges it)");

    app.add_flag("--prox-log", args.prox_log,
                 "Print proximity engage/release events, the active-ship "
                 "wake, the warp cap, and a throttled per-ship distance "
                 "snapshot -- the instrument for the proximity e2e tests");

    app.add_option("--prox-fly-on", args.prox_fly_on,
                   "Engage radius (m) while the active ship is flying: a "
                   "non-active ship within it leaves the rails and gets "
                   "physics (default 2000 = 2 km)");
    app.add_option("--prox-fly-off", args.prox_fly_off,
                   "Release radius (m) while the active ship is flying: a "
                   "proximity-activated ship beyond it returns to the rails "
                   "(default 10000 = 10 km)");
    app.add_option("--prox-ground-on", args.prox_ground_on,
                   "Engage radius (m) while the active ship is grounded: a "
                   "non-active ship within it leaves the rails (default "
                   "10 = 10 m; 0 disables auto-waking grounded neighbors)");
    app.add_option("--prox-ground-off", args.prox_ground_off,
                   "Release radius (m) while the active ship is grounded: a "
                   "proximity-activated ship beyond it returns to the rails "
                   "(default 20 = 20 m)");
    app.add_option("--prox-warp", args.prox_warp,
                   "Max time accel while any non-active ship is engaged "
                   "(default 1)");

    app.add_flag("--compound-check", args.compound_check,
                 "Periodically print, for every ship, how far the part poses "
                 "derived from its single compound rigid body are from the "
                 "parts' own live poses (max position + angle error, and the "
                 "centre-of-mass drift) -- the gate on the "
                 "ship-as-one-rigid-body migration; a nonzero error is the "
                 "weld wobble the compound removes");

    app.add_flag("--eva-log", args.eva_log,
                 "Periodically print the EVA kerbal's mode (ground/space), "
                 "grounded state, position and velocity to stdout; the "
                 "instrument for the EVA e2e tests");

    app.add_option("--postfx", args.postfx_spec,
                   "Post-processing effect, in the order given; repeatable "
                   "and/or comma-separated (e.g. --postfx cas,grain). "
                   "Available: crt (retro tube look), grain (animated film "
                   "grain), cas (adaptive-contrast sharpening, 'sharpen' "
                   "also accepted), color (gamma/brightness/black level/"
                   "saturation; 'gamma' also accepted, slider per knob in "
                   "Settings). Omit for direct output (default)");

    app.add_flag("--gl-debug", args.gl_debug,
                 "Enable the OpenGL debug output callback (GL_DEBUG_* "
                 "messages print as they occur)");

    std::string msaa;
    app.add_option("--msaa", msaa,
                   "Multisample antialiasing: none | 2x | 4x | 8x "
                   "(default 4x). Fixed at window creation (the GLX visual "
                   "is chosen then), so it applies at launch/restart")
        ->check(CLI::IsMember({"none", "2x", "4x", "8x"}));

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

    app.add_flag("--perf", args.perf,
                 "Print a per-frame phase timing breakdown (events / logic / "
                 "jobs / render + fps + physics substeps): a rolling line "
                 "every second and a summary at exit. Off by default; the "
                 "timing overhead is negligible when off");

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

    app.add_option("--cloud-mesh", args.cloud_mesh,
                   "Cloud deck sphere resolution (lat = lon = N rings; "
                   "default 128). The cloud detail lives in the baked "
                   "coverage map, so this only changes the deck's rim "
                   "smoothness and its vertex cost -- try 32..256")
        ->check(CLI::Range(8, 512));

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

    /* --msaa: map the label to a sample count (unspecified keeps the
       4x default in GameArgs). */
    if(msaa == "none") { args.msaa_samples = 0; }
    else if(msaa == "2x") { args.msaa_samples = 2; }
    else if(msaa == "4x") { args.msaa_samples = 4; }
    else if(msaa == "8x") { args.msaa_samples = 8; }

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

    /* --sim-mode: fold the flat TIME_MS,MODE,WIDTH,HEIGHT list into
       changes. WIDTH / HEIGHT of 0 keep the launch --width/--height. */
    if(!sim_mode.empty()) {
        if(sim_mode.size() % 4 != 0) {
            printf("error: --sim-mode expects TIME_MS,MODE,WIDTH,HEIGHT "
                   "quadruples; got %zu value(s)\n", sim_mode.size());
            *exit_code = 1;
            return false;
        }
        auto parse_mode = [](const std::string &s, WindowMode &out) -> bool {
            std::string t;
            for(char c : s) { t += (char)std::tolower((unsigned char)c); }
            if(t == "windowed" || t == "window") { out = WindowMode::Windowed; return true; }
            if(t == "borderless") { out = WindowMode::Borderless; return true; }
            if(t == "fullscreen") { out = WindowMode::Fullscreen; return true; }
            if(t == "exclusive") { out = WindowMode::Exclusive; return true; }
            return false;
        };
        for(size_t i = 0; i < sim_mode.size(); i += 4) {
            char *end = nullptr;
            const unsigned long t =
                strtoul(sim_mode[i].c_str(), &end, 10);
            if(end == sim_mode[i].c_str() || *end != '\0') {
                printf("error: --sim-mode time '%s' is not an "
                       "integer ms\n", sim_mode[i].c_str());
                *exit_code = 1;
                return false;
            }
            WindowMode mode;
            if(!parse_mode(sim_mode[i + 1], mode)) {
                printf("error: --sim-mode mode '%s' is not one of "
                       "windowed, borderless, fullscreen, exclusive\n",
                       sim_mode[i + 1].c_str());
                *exit_code = 1;
                return false;
            }
            const unsigned long w =
                strtoul(sim_mode[i + 2].c_str(), &end, 10);
            if(end == sim_mode[i + 2].c_str() || *end != '\0') {
                printf("error: --sim-mode width '%s' is not an "
                       "integer pixel\n", sim_mode[i + 2].c_str());
                *exit_code = 1;
                return false;
            }
            const unsigned long h =
                strtoul(sim_mode[i + 3].c_str(), &end, 10);
            if(end == sim_mode[i + 3].c_str() || *end != '\0') {
                printf("error: --sim-mode height '%s' is not an "
                       "integer pixel\n", sim_mode[i + 3].c_str());
                *exit_code = 1;
                return false;
            }
            SimModeChange m;
            m.at_ms = (Uint32)t;
            m.mode = mode;
            m.width = (w == 0) ? args.screen_width : (int)w;
            m.height = (h == 0) ? args.screen_height : (int)h;
            m.done = false;
            args.sim_mode_changes.push_back(m);
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

    /* Which settings the CLI set explicitly (CLI11 ->count()): the
       settings.json load honors this mask -- the command line beats the
       saved file, field by field. */
    args.cli_given.window_mode =
        (fs_opt->count() + bl_opt->count() + ex_opt->count()) > 0;
    args.cli_given.width = app.get_option("--width")->count() > 0;
    args.cli_given.height = app.get_option("--height")->count() > 0;
    args.cli_given.msaa = !msaa.empty();
    args.cli_given.postfx = !args.postfx_spec.empty();
    args.cli_given.fov = app.get_option("--fov")->count() > 0;
    args.cli_given.terrain_px = app.get_option("--terrain-px")->count() > 0;
    args.cli_given.exhaust_scale =
        app.get_option("--exhaust-scale")->count() > 0;

    return true;
}
