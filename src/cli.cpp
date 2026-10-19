// cli.cpp -- command-line parsing (CLI11) + the --sim-press / --sim-mouse
// folding. Fills GameArgs (cli.h).
#include "cli.h"
#include "settings.h"   // kTerrainPxMin/Max (the --terrain-px range)
#include "version.h"   // VERSION

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <algorithm>   // stable_sort on the --ui-* queue

#include <CLI11/CLI11.hpp>

/* Parses the leading "AT_MS," of a --ui-* value, leaving everything after the
   comma in `rest`. Prints the error and returns false; the callers are the
   --ui-click and --ui-type loops, which share the time token exactly. */
static bool parse_ui_time(const std::string &spec, const char *flag,
                          Uint32 &at_ms, std::string &rest) {
    const size_t comma = spec.find(',');
    if(comma == std::string::npos) {
        printf("error: %s expects AT_MS,... (e.g. %s 1500,\"Title Menu/"
               "New Game\"); got '%s'\n", flag, flag, spec.c_str());
        return false;
    }
    const std::string ts = spec.substr(0, comma);
    char *end = nullptr;
    const unsigned long t = strtoul(ts.c_str(), &end, 10);
    // The end-pointer test alone would accept "-5" (strtoul wraps); the range
    // test also keeps the Uint32 cast honest.
    if(ts.empty() || end != ts.c_str() + ts.size() || t > 0xFFFFFFFFUL) {
        printf("error: %s time '%s' is not a loop time in ms "
               "(0..4294967295)\n", flag, ts.c_str());
        return false;
    }
    at_ms = (Uint32)t;
    rest = spec.substr(comma + 1);
    return true;
}

bool parse_cli(int argc, char **argv, GameArgs &args, int *exit_code)
{
    CLI::App app{"Open Space Program"};

    // CLI11 throws CallForVersion on parse; the catch below routes it through
    // app.exit().
    app.set_version_flag("--version",
                         std::string("Open Space Program ") + VERSION,
                         "Prints the version and exits");

    app.add_option("--title-body", args.title_body,
        "Pin the title-screen backdrop to this body instead of a random "
        "non-star one (a test / visual-regression hook; default: random)");

    app.add_option("--system", args.system_file,
                   "Star-system JSON file to load (default: "
                   "res/systems/ksp_system.json; try res/systems/old_system.json "
                   "for the Eerbon system)");

    app.add_option("--parts", args.parts_file,
                   "Parts catalog JSON (default: res/data/parts.json)");

    app.add_option("--experiments", args.experiments_file,
                   "Experiment family JSON (default: res/data/experiments.json)");

    app.add_option("--kerbals", args.kerbals_file,
                   "Kerbal roster JSON: the named crew the ships draw from, "
                   "each with id, name and piloting/science/engineering 1..5 "
                   "(default: res/data/kerbals.json)");

    app.add_option("--startship", args.startship,
                   "One start ship: name,def,body,scenario[,site] -- the "
                   "first four required, the fifth an optional pad id (e.g. "
                   "racer,res/ships/racer.json,Kerbin,pad,polar_pad). "
                   "Repeat the flag for more ships; ships sharing a "
                   "body+scenario are spaced apart in orbit but default to "
                   "the same pad. --startships is the same list as a JSON "
                   "file (and overrides it)");

    app.add_option("--startships", args.startships_file,
                   "Start-ship-list JSON: one entry per ship, each naming all "
                   "four of name, ship def, body and scenario (the game "
                   "errors if any is missing), plus an optional \"site\" pad "
                   "id. Ships sharing a body+scenario are spaced apart in "
                   "orbit but default to the same pad. Overrides --startship. "
                   "Try e2e/fixtures/startships.json");

    app.add_option("--save", args.save_name,
                   "Save the game (its ships + crew + clock) when the "
                   "--timeout budget is spent, then exit (the headless/e2e "
                   "path). A bare name is a slot of the current game under "
                   "the data dir's saves/ (<stamp>-<gameName>/<slot>, the "
                   "dir minted at this first save); a path is used as-is. "
                   "Mutually exclusive with --load");
    app.add_option("--load", args.load_name,
                   "Load the game from this saved directory at startup "
                   "instead of building the start ships (--startship/--startships are ignored). "
                   "A bare name resolves to the data dir's saves/<name> "
                   "(legacy flat) or the unique saves/<game>/<name>; a path "
                   "is used as-is. Mutually exclusive with --save");
    app.add_option("--resave", args.resave_name,
                   "Write this save dir when the --timeout budget is spent -- "
                   "the same capture as --save, but legal alongside --load, "
                   "which is the whole point: load a save, run it, write it "
                   "back, and diff the two trees. That is how a save/load "
                   "drift (a field the writer drops or the reader misreads) "
                   "gets caught in one process instead of two runs plus a "
                   "committed fixture. Same name rules as --save");
    app.add_option("--data-dir", args.data_dir,
                   "User data directory (saves/ + settings.json). Default: "
                   "the per-OS user data directory (on Linux: "
                   "~/.local/share/openspaceprogram/)");

    // Spin-test ship (no JSON def); all parts are passive tanks, so any spin is
    // self-inflicted.
    app.add_option("--radial-test", args.radial_test,
                   "Build the spin-test ship(s) instead of the start ships: "
                   "parallel | stacked | stacks | parstacks")
        ->check(CLI::IsMember({"parallel", "stacked", "stacks",
                               "parstacks"}));
    app.add_option("--autopilot", args.autopilot,
                   "Engage a slew mode on the active ship at startup: "
                   "prograde | retrograde | radial-out | radial-in | normal | "
                   "anti-normal | kill-rot. A test hook -- in-game these are "
                   "the Autopilot window's toggles (clickable headless with "
                   "--ui-click) plus Z/B for prograde/retrograde "
                   "(default: none)")
        ->check(CLI::IsMember({"prograde", "retrograde", "radial-out",
                               "radial-in", "normal", "anti-normal",
                               "kill-rot"}));
    app.add_option("--vab", args.vab,
                   "Open the VAB editor scene with this ship def loaded "
                   "(physics-free build tree) instead of flying");
    app.add_flag("--vab-empty", args.vab_empty,
                 "Open the VAB editor scene with an EMPTY build (the main "
                 "menu's \"Go to VAB\") instead of flying (test hook)");
    app.add_option("--vab-arm", args.vab_arm,
                   "Arm this catalog part in the VAB palette at startup "
                   "(so sim-input can place it without clicking the palette)");
    app.add_option("--vab-place", args.vab_place_ms,
                   "Fire the VAB's place (vabPlace) once at this loop time in "
                   "ms (headless test hook; -1 = never)");
    app.add_option("--vab-launch", args.vab_launch_ms,
                   "Fire the VAB's LAUNCH once at this loop time in ms "
                   "(headless test hook for the launch path; -1 = never)");
    app.add_option("--vab-close", args.vab_close_ms,
                   "Fire the VAB's \"Back to game\" (vabClose) once at this loop "
                   "time in ms -- the headless path into the camera "
                   "park/restore round trip (test hook; -1 = never)");
    app.add_option("--vab-detach", args.vab_detach_idx,
                   "Select this build part (index) and fire the VAB's detach "
                   "(vabDetachSelected) once -- the headless stand-in for "
                   "Del/X on a selected part (test hook; -1 = never)");
    app.add_option("--vab-detach-at", args.vab_detach_ms,
                   "Fire the --vab-detach hook once at this loop time in ms "
                   "(test hook; -1 = never)");
    std::vector<std::string> vab_assign;
    app.add_option("--vab-assign", vab_assign,
                   "Put a named kerbal in a VAB seat at this loop time: a flat "
                   "list of MS,PART_IDX,SLOT,KERBAL groups (e.g. 4000,0,0,Ace "
                   "= at 4s put Ace in seat 1 of build part 0). KERBAL '-' "
                   "empties that seat, which is a choice: the launch then leaves "
                   "it empty instead of drawing the pad's line. Repeat the flag "
                   "for more seats. The headless stand-in for the seat window "
                   "(right-click a capsule in the VAB), which it opens as it "
                   "assigns (test hook)")
        ->delimiter(',');
    std::vector<std::string> vab_link;
    app.add_option("--vab-link", vab_link,
                   "Click a fuel link at this loop time: a flat list of "
                   "MS,FROM_ID,TO_ID groups (e.g. 4000,capsule_1,fuel_tank_1 "
                   "= at 4s link capsule_1 into fuel_tank_1). Repeat the flag "
                   "for more links. The headless stand-in for the VAB's "
                   "\"Add fuel link\" + two part clicks (test hook)")
        ->delimiter(',');
    app.add_option("--reload", args.reload_dir,
                   "Load this save dir over the running game at --reload-at "
                   "(the Save/Load window's Load button, headless)");
    app.add_option("--reload-at", args.reload_ms,
                   "Loop time in ms at which --reload fires (-1 = never)");
    app.add_option("--quit-title", args.quit_title_ms,
                   "Jump to the title screen at this loop time in ms. No button "
                   "does this: from flight the hub is in between, and the hub's "
                   "\"Return to title\" asks first (case 260 walks that). From "
                   "the Tracking Station or the Research Lab there is no title "
                   "exit at all -- issue #310 (headless test hook; -1 = never)");
    app.add_option("--recover", args.recover_ms,
                   "Fire the Space Center menu's \"Recover Vessel\" once at "
                   "this loop time in ms (Game::recoverActive, headless; "
                   "-1 = never)");
    app.add_flag("--recover-anywhere", args.recover_anywhere,
                 "Let 'Recover Vessel' recover the active ship anywhere "
                 "(the default requires it grounded on the home body)");
    std::vector<std::string> select_target;
    app.add_option("--select-target", select_target,
                   "Choose a transfer target at this loop time: MS,NAME groups "
                   "(e.g. 4000,Mun = at 4s pick Mun in the Target window). "
                   "Repeat the flag for more picks; NAME 'none' clears the "
                   "selection. The combo it stands in for is clickable now "
                   "(#251), and cases 241/242 use it; this survives for case "
                   "243 alone, which needs a selection INSIDE the state pass, "
                   "in the window where a removed ship's name still backs a "
                   "target row. --transfer-target, if given, re-asserts itself "
                   "on every rebuild and wins (test hook)")
        ->delimiter(',');
    app.add_option("--experiment", args.experiment_ms,
                   "Fire the part window's \"Run Experiment\" once at this "
                   "loop time in ms (Game::runExperiment, headless; "
                   "-1 = never)");
    app.add_option("--pod-experiment", args.pod_experiment_ms,
                   "Fire a science pod's \"Run Experiment\" once at this loop "
                   "time in ms (Game::runPodExperiment on the active ship's "
                   "first experiment-family part, headless; -1 = never)");
    app.add_option("--eva", args.eva_ms,
                   "Fire the part window's \"EVA\" once at this loop time in "
                   "ms (Game::kerbalEVA on the active ship's first crew "
                   "kerbal, headless; -1 = never). The take/store dance needs "
                   "a FREE kerbal in reach, so the e2e EVAs before --take/--store.");
    app.add_option("--board", args.board_ms,
                   "Fire the part window's \"Board\" once at this loop time in "
                   "ms (Game::kerbalBoard: the first walking kerbal in reach of "
                   "the first capsule with a free seat, headless; -1 = never). "
                   "The reverse of --eva, so --eva then --board round-trips a "
                   "person through their seat.");
    app.add_option("--take", args.take_ms,
                   "Fire the take/store dance's \"Take\" once at this loop "
                   "time in ms (Game::moveExperiment from the active ship's "
                   "first held finding to its first free kerbal in reach, "
                   "headless; -1 = never)");
    app.add_option("--store", args.store_ms,
                   "Fire the take/store dance's \"Store\" once at this loop "
                   "time in ms (Game::moveExperiment from the active ship's "
                   "courier to its first container, headless; -1 = never)");
    app.add_option("--switch-system", args.switch_system_path,
                   "Swap to this system JSON over the running game at "
                   "--switch-at (the in-process system switch, "
                   "Game::switchSystem; lands on the Title screen)");
    app.add_option("--switch-at", args.switch_system_ms,
                   "Loop time in ms at which --switch-system fires "
                   "(headless test hook; -1 = never)");
    app.add_option("--vab-scenario", args.vab_scenario,
                   "Seed the VAB's launch scenario (the top-bar dropdown's "
                   "value; empty = \"pad\")");
    app.add_option("--vab-body", args.vab_body,
                   "Seed the VAB's launch body (the top-bar dropdown's "
                   "value; empty = the home body)");
    app.add_option("--vab-site", args.vab_site,
                   "Seed the VAB's launch site (the top-bar site dropdown's "
                   "world-building id; empty = the first VAB_LAUNCH_PAD)");
    app.add_flag("--dev-scenarios", args.dev_scenarios,
                 "Add the dev-only start beds (orbit, ellipse, escape, "
                 "distance-ladder, heliocentric) to the VAB's scenario "
                 "dropdown. Without it the dropdown offers only the "
                 "player-facing beds (currently just \"pad\"); the dev beds "
                 "still resolve by name here and in --startship, so this "
                 "gates the UI only");

    // Docking pair: probe (active) + station, nose-to-nose (see src/docktest.cpp).
    app.add_option("--dock-test", args.dock_test,
                   "Build the docking pair (probe + station) instead of the "
                   "start ships: near | approach")
        ->check(CLI::IsMember({"near", "approach"}));

    app.add_option("-t,--time-accel", args.initial_time_accel,
                   "Initial time acceleration (0 = paused, default 1)")
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
                   "the New Game sheet sets this per game and it is stored "
                   "in the save)")
        ->check(CLI::Range(0.5f, 5.0f));

    app.add_option("--cam-shake", args.cam_shake,
                   "Camera shake at high acceleration (the chase cam "
                   "rumbles with the crew's felt g's): 0 disables it, "
                   "higher scales it up (0-3.0, default 1; adjustable in "
                   "the Settings window)")
        ->check(CLI::Range(0.0f, 3.0f));

    app.add_option("--drag-cd", args.drag_cd,
                   "The atmospheric drag coefficient (src/drag.h): F = "
                   "-v*0.5*rho*Cd*A*v^2. Default 1.2; 0 disables drag "
                   "entirely (a test / A-B knob)")
        ->check(CLI::NonNegativeNumber);

    app.add_flag("--drag-log", args.drag_log,
                 "Log the active ship's drag per tick (altitude, density, "
                 "speed, force) -- the 'is drag acting?' instrument");

    app.add_option("--timeout", args.timeout_seconds,
                   "Auto-exit the main loop after this many wall-clock "
                   "seconds (0 = run until closed; default: 0)")
        ->check(CLI::NonNegativeNumber);

    std::vector<std::string> sim_press;
    app.add_option("--sim-press", sim_press,
                   "Synthetic keypresses for e2e testing: a flat list of "
                   "START_MS,DURATION_MS,KEY triples (e.g. 500,200,SPACE, "
                   "1500,100,I; spaces also separate values). KEY is a name in "
                   "the Controls window's vocabulary (A..Z, 0..9, SPACE, TAB, "
                   "ESCAPE, BACKSPACE, F1-F12, LEFT/RIGHT/UP/DOWN, LSHIFT/"
                   "RSHIFT/LCTRL/RCTRL/LALT/RALT, and the punctuation the "
                   "Controls list shows: - = [ ] \\ ; ' ` / .) or a decimal SDL "
                   "keycode. The key is pressed START_MS after the main loop "
                   "starts and held for DURATION_MS. Repeat the flag to append "
                   "more triples.")
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
                   "button = a drag (held); 0 = a quick click. CAUTION in "
                   "flight: a 0-duration RMB press is a part PICK -- it opens "
                   "that part's window, and a window under the cursor then "
                   "captures the mouse, so the NEXT RMB drag is silently "
                   "truncated mid-gesture. Give a camera drag its own "
                   "DURATION_MS>0 and check it with --cam-log (yaw/pitch plus "
                   "the rmb=/cap= gates). Repeat the flag to append more "
                   "quintuples.")
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

    /* No ->delimiter: the path may contain commas, so each value stays one
       string and the split at the first comma happens below (src/uiinput.cpp). */
    std::vector<std::string> ui_click;
    app.add_option("--ui-click", ui_click,
                   "Synthetic imgui click for e2e testing: AT_MS,WINDOW/LABEL "
                   "(e.g. 1500,\"Title Menu/New Game\", 3000,\"Game Menu/"
                   "Tracking Station\"). The click lands on the first frame "
                   "from AT_MS on whose previous imgui frame drew a matching "
                   "item, through imgui's own input queue -- so the widget "
                   "must exist and be on screen, unlike the --*-at hooks that "
                   "call the action directly. WINDOW may be omitted to match "
                   "the label in any window; labels are exact, minus the ## "
                   "suffix. Repeat the flag, or space-separate the values, "
                   "for more clicks.")
        ->expected(-1);

    app.add_option("--ui-list", args.ui_list_ms,
                   "Dump every clickable imgui item (window, label, rect) at "
                   "this loop time in ms, for writing --ui-click paths "
                   "(test hook; -1 = never)");

    std::vector<std::string> ui_type;
    app.add_option("--ui-type", ui_type,
                   "Synthetic imgui text entry for e2e testing: "
                   "AT_MS,WINDOW/LABEL,TEXT. Clicks the widget to focus it, "
                   "selects all, then queues TEXT through imgui's own character "
                   "input -- so the widget's filters and length cap run, and a "
                   "pre-filled field (\"game1\", \"save1\") is replaced rather "
                   "than appended to. Fails like --ui-click if the widget is "
                   "not there. TEXT may contain commas; a path that does cannot "
                   "be named here. Repeat the flag, or space-separate the "
                   "values.")
        ->expected(-1);

    app.add_option("--atlas-dump", args.atlas_dump_ms,
                   "Print the System Atlas at this loop time in ms: one "
                   "[atlas] line per body (tree order, research weight, "
                   "approach Δv, science found) plus its dossier -- radius, "
                   "mass, gravity, escape velocity, day length, axial tilt, "
                   "air, and the orbit's semi-major axis / eccentricity / "
                   "apsides / period / plane tilt (ecl, parent, eq.) / node "
                   "(ecl, parent) / periapsis longitude. "
                   "The same strings the "
                   "Research Lab's Atlas draws, so the terminal and the "
                   "window cannot disagree (test hook; -1 = never)");

    app.add_option("--map-plane", args.map_plane,
                   "Test hook: start the orbit map in this plane slot "
                   "(0 equatorial, 1 ecliptic, 2 orbital, 3 target orbit -- "
                   "needs --transfer-target, else it measures against the "
                   "ecliptic and says so). The flight "
                   "readout measures Inc/LAN/LPe in the plane the map shows, "
                   "so this pins those branches without driving the combo "
                   "widget (default -1 = leave it alone)")
        ->check(CLI::Range(-1, 3));

    app.add_option("--map-dump", args.map_dump_ms,
                   "Test hook: print the orbit map's in-plane basis (n, e1, "
                   "e2 and the prograde sweep sign) plus the AN/DN crossings "
                   "it marks [m, focus at the origin, in the focus's inertial "
                   "frame; \"-\" where the orbit does not cross that plane], "
                   "for every plane slot it "
                   "can resolve -- the target-orbit slot needs a transfer "
                   "target -- "
                   "at this loop time in ms: one [mapdump] line per slot, "
                   "from the same code the map windows use (test hook; "
                   "-1 = never)");

    app.add_flag("--orbit-log", args.orbit_log,
                 "Periodically print the ship's orbital elements to stdout "
                 "(for measuring orbital stability)");

    app.add_flag("--plane-log", args.plane_log,
                 "Test hook: draw the Orbital plane overlay (the Settings "
                 "checkbox otherwise leaves it off, and the flag would log "
                 "nothing) and periodically print what it drew -- the plane "
                 "normals, the ascending node, the dihedral and the "
                 "prograde/normal flight markers, all render-frame, so a wrong "
                 "frame transform shows up as a number");

    app.add_option("--orbit-interval", args.orbit_interval,
                   "Wall-clock seconds between --orbit-log / --plane-log lines "
                   "(default: 1)")
        ->check(CLI::PositiveNumber);

    app.add_flag("--dbg-log", args.dbg_log,
                 "Periodically print ship position/altitude/velocity "
                 "(surface-level companion to --orbit-log)");

    app.add_flag("--info-log", args.info_log,
                 "Periodically print the Orbit Info + Surface Info window "
                 "values (body, vel, alt, apsides, elements, attitude, "
                 "surface coords) to stdout");

    app.add_flag("--debug-accel", args.debug_accel,
                 "Per-substep dump of every ship's thrust force + velocity "
                 "(diagnosing spurious forces on dropped ships)");

    app.add_flag("--xfer-log", args.xfer_log,
                 "Periodically print the transfer planner's solution to "
                 "stdout (needs a target -- --transfer-target selects one -- "
                 "and the TARGET window's \"Compute transfer\" on, which it "
                 "starts as)");

    app.add_option("--transfer-target", args.transfer_target,
                   "Transfer planner target: a child body of the ship's "
                   "current body, or another ship in the same body");

    app.add_flag("--porkchop-log", args.porkchop_log,
                 "Print the porkchop plot's best cell (min dv, its departure "
                 "delay + time of flight) to stdout whenever the plot is "
                 "computed (P key or the window's button)");
    app.add_option("--porkchop-n", args.porkchop_n,
                   "Porkchop plot grid size (porkchop-n x porkchop-n); the "
                   "size knob, a Settings-window hook later. Cost is n^2: "
                   "~25 ms at 40 x 40, ~0.9 s at the default 256 x 256. The "
                   "sweep runs on a worker thread, so the frame rate is "
                   "unaffected and the last grid stays on screen")
        ->check(CLI::Range(1, 2048));
    app.add_option("--porkchop-bench", args.porkchop_bench,
                   "On the plot trigger (P), sweep these grid sizes back to "
                   "back off ONE ship/target snapshot and log each, so the "
                   "grids differ only in resolution. Swept ascending; the live "
                   "plot keeps the largest, e.g. --porkchop-bench 40,128,256")
        // CLI11's number lexer otherwise reads "40,128" as one integer, 40128.
        ->delimiter(',')
        // A sweep is ONE job, so an unbounded list is unbounded worker time the
        // process must wait out before it can exit or switch system.
        ->expected(1, 8)
        // Looser than --porkchop-n's cap: an offline sweep, not the live plot.
        ->check(CLI::Range(1, 4096));
    app.add_option("--porkchop-dump", args.porkchop_dump,
                   "Directory to write each computed porkchop grid into as "
                   "CSV (one row per ToF sample, NaN = no solution); implies "
                   "--porkchop-bench's per-grid logging");

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

    app.add_option("--surfmap-body", args.surfmap_body,
                   "Pin the Surface Map's body to NAME at boot (the window's "
                   "combo pick), so M / the Refresh button map it. Unknown "
                   "body names are an error. A test hook (issue #72): lets "
                   "e2e set the map's body without driving the combo click");

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

    app.add_flag("--shake-log", args.shake_log,
                 "Periodically print the camera shake state (the felt "
                 "acceleration, the amplitude, the offset) to stdout; the "
                 "instrument for the cam-shake e2e test");

    app.add_flag("--cam-log", args.cam_log,
                 "Periodically print the live camera to stdout: mode, the "
                 "orbit turntable (yaw/pitch/dist), the view direction, and "
                 "the two flags that decide what a --sim-mouse gesture "
                 "actually does -- rmb= whether a right-button drag is armed, "
                 "cap= whether imgui ate the mouse (a window under the cursor "
                 "swallows the drag). --sim-mouse is documented to move the "
                 "camera, yet nothing else in stdout proves it moved: without "
                 "this line a mouse case whose drag never reached the camera "
                 "still passes");

    app.add_flag("--audio-log", args.audio_log,
                 "Trace every audio event to stdout as [aud] lines: the "
                 "device that opened, each clip loaded (format + length), "
                 "each one-shot played with its gain, and the loop fades -- "
                 "the instrument for 'did that part's sound fire?'. Same "
                 "trace the AUDIO_DEBUG=1 environment variable turns on, "
                 "which nothing in --help used to mention");

    app.add_flag("--terrain-log", args.terrain_log,
                 "Periodically print the local body's terrain LOD to stdout: "
                 "the alive patch count, the deepest leaf depth, how many "
                 "leaves carry collision, and how far the nearest deepest "
                 "leaf is from the camera (deep_off) -- the instrument for "
                 "the terrain-LOD e2e cases");

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
                 "messages print as they occur) and the glGetError drains");

    app.add_flag("--gl-error-checks", args.gl_error_checks,
                 "Drain glGetError at every check_gl_error() site (printing "
                 "GL_<ENUM> - file:line) without the debug callback. Off by "
                 "default because the drain can flush the command buffer; the "
                 "e2e runner sets it for any case that FORBIDs GL_");

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
                               "the vendored SDL3's X11 driver never "
                               "restores the previous mode on exit "
                               "(X11_QuitModes is a no-op), so restore it "
                               "yourself with xrandr if it matters");
    fs_opt->excludes(bl_opt);
    fs_opt->excludes(ex_opt);
    bl_opt->excludes(ex_opt);

    app.add_option("--font", args.font_path,
                   "TTF font file for all UI text; the normal and big faces "
                   "are the same font (the big one at twice --font-size; "
                   "default res/fonts/DejaVuSansMono.ttf)");

    app.add_option("--font-size", args.font_size,
                   "UI font size in pixels (the big HUD readout font is "
                   "twice this; default 14)")
        ->check(CLI::PositiveNumber);

    app.add_option("--frame-cap", args.frame_cap,
                   "Max render frames per second (0 = uncapped; default 60). "
                   "Without a cap the loop busy-spins between vsyncs, "
                   "idling a CPU core at 100% even while paused")
        ->check(CLI::NonNegativeNumber);

    app.add_option("--idle-fps", args.idle_fps,
                   "Render frames per second while the window is occluded or "
                   "minimized (default 10; 0 = never idle). The compositor "
                   "says nobody can see the surface, so the loop idles instead "
                   "of issuing ~1ms of GL commands on the title screen and "
                   "2-3ms in flight, plus a swap that goes nowhere. It is "
                   "throttled, NOT skipped: the 3D state pass also re-poses "
                   "the camera, runs the terrain LOD that owns the ground's "
                   "collision bodies, and refreshes the g.hud.view snapshot the UI "
                   "reads, so skipping a draw would change the simulation and "
                   "not just the image. The logic tick keeps running at "
                   "--physics-hz off the wall clock, so a minimized window "
                   "still warps at full "
                   "rate. The idle rate is never slower than physics_hz/8 (80% "
                   "of tick.cpp's 10-tick catch-up clamp); a slower request is "
                   "raised to it and startup says so. On X11 SDL reports "
                   "occlusion only for an iconified window, so there this "
                   "fires on minimize")
        ->check(CLI::NonNegativeNumber);

    app.add_option("--force-occluded", args.force_occluded_ms,
                   "Pretend the window is occluded from this many ms after the "
                   "main loop starts (-1 = off). Headless cover for --idle-fps: "
                   "a compositor never occludes an offscreen window, so without "
                   "this the idle path has no automated test")
        ->check(CLI::Range(-1, 1000000000));

    app.add_option("--vsync", args.vsync,
                   "Swap interval: 1 = wait one display refresh per swap "
                   "(default), 0 = immediate. The game used to leave this at "
                   "the driver default, so the render clock was whatever the "
                   "driver felt like -- and the fixed physics tick can only "
                   "phase-lock to a render clock that is a known rate. With "
                   "vsync on, SwapBuffers blocks the retrace, so --frame-cap "
                   "only bites when the panel is FASTER than the cap (a cap "
                   "below a faster panel's retrace grid needs an interval > 1, "
                   "not a sleep); with it off the --frame-cap sleep is the "
                   "pacer, at SDL_Delay's accuracy. N > 1 blocks N retraces per "
                   "swap (interval 2 on a 120 Hz panel = 60 fps); -1 asks for "
                   "adaptive. Startup prints what the driver actually granted")
        ->check(CLI::Range(-1, 4));

    app.add_option("--physics-hz", args.physics_hz,
                   "Fixed logic/physics tick rate in Hz (default 60). The "
                   "tick is what quantizes every drawn pose, so a tick rate "
                   "the render clock cannot land on gives a beat: at 50 Hz "
                   "against a 60 Hz panel one render frame in five runs no "
                   "tick at all and redraws an identical ship pose (the "
                   "camera and its shake still move). 60 locks to "
                   "60/120/180/240 Hz panels because display refreshes are "
                   "multiples of 60, not 50. Raising it costs proportional "
                   "logic time and changes every simulated trajectory; the "
                   "10-tick catch-up clamp and the warp substep cap both "
                   "scale with it. 5..1000")
        ->check(CLI::Range(5.0, 1000.0));

    app.add_flag("--perf", args.perf,
                 "Print a per-frame phase timing breakdown (events / logic / "
                 "jobs / refresh (the 3D state pass) / render (the paint pass) "
                 "+ fps + logic ticks): a rolling line "
                 "every second and a summary at exit. Off by default; the "
                 "timing overhead is negligible when off");

    app.add_flag("--pose-jitter", args.pose_jitter,
                 "Print one line per RENDER frame: the sim time (t) and wall "
                 "frame time (fdt), the physics ticks that ran, the tick-window "
                 "position (alpha), the ship's distance from the local body's "
                 "centre (r), how far that offset MOVED since the previous drawn "
                 "frame (gap) and the same for attitude in degrees (dang; -1 = "
                 "unmeasurable, no controller part). Separates logic-tick pose "
                 "quantization (gap swings 0..2x while fdt is steady) from "
                 "frame-pacing jitter (fdt varies). Diagnostic only: nothing "
                 "here feeds physics");

    app.add_option("--fov", args.camFovDeg,
                   "Camera vertical field of view in degrees (default 60; "
                   "adjustable in the Settings window)")
        ->check(CLI::Range(10.0f, 120.0f));

    app.add_option("--terrain-px", args.terrain_px,
                   "Terrain LOD: a patch subdivides while it projects "
                   "wider than this [REAL screen px] (512 = default; "
                   "256 = finer, ~2x the patches; 1024 = coarsest). "
                   "The fine end is expensive: the budget is per patch, so "
                   "the low end asks for tens of thousands of them and the "
                   "single async builder never catches up (the tree stays "
                   "shallow under the camera) -- and since collision meshes "
                   "only exist at max depth, that means no ground under the "
                   "ship. Adjustable in the Settings window")
        ->check(CLI::Range(kTerrainPxMin, kTerrainPxMax));

    app.add_option("--cloud-mesh", args.cloud_mesh,
                   "Cloud deck sphere resolution (lat = lon = N rings; "
                   "default 128). The cloud detail lives in the baked "
                   "coverage map, so this only changes the deck's rim "
                   "smoothness and its vertex cost -- try 32..256")
        ->check(CLI::Range(8, 512));

    app.add_flag("--no-clouds", args.no_clouds,
                 "Skip the cloud deck spheres (debug: see the terrain and "
                 "its depth without the opaque cloud layer)");
    app.add_flag("--no-atmosphere", args.no_atmosphere,
                 "Skip the atmosphere rim/haze shells (debug: see the "
                 "terrain and its depth without the transparent limb glow)");
    app.add_flag("--no-ocean", args.no_ocean,
                 "Skip the ocean surface shells (debug: see the sea floor "
                 "terrain without the water layer; only \"ocean\": \"mesh\" "
                 "bodies have a shell)");
    app.add_flag("--no-rings", args.no_rings,
                 "Skip the planetary ring annuli (debug: see the planet and "
                 "its depth without the rings)");

    app.add_option("--sky-dim", args.sky_dim,
                   "Star-field exposure prototype: the gain the sky fades to "
                   "while the sun is in view and the camera is in sunlight "
                   "(0..1; 1 = no fade, 0 = the stars vanish)")
        ->check(CLI::Range(0.0f, 1.0f));
    app.add_option("--sky-dim-cone", args.sky_dim_cone,
                   "Half-angle [deg] of the \"sun in view\" cone behind "
                   "--sky-dim; the fade runs out by 2.5x that angle")
        ->check(CLI::Range(1.0f, 179.0f));
    app.add_flag("--sky-dim-log", args.sky_dim_log,
                 "Print the star-field fade's inputs once a second "
                 "(sun angle off the view axis, in-view and lit factors, gain)");

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

    // app.parse(), NOT the CLI11_PARSE macro: the macro does `return (app).exit(e)`,
    // an int implicitly converted to bool here (0=false), inverting help vs.
    // error. Our catch sets *exit_code and returns false for main() to act on.
    try {
        app.parse(argc, argv);
    } catch(const CLI::ParseError &e) {
        *exit_code = app.exit(e);
        return false;
    }

    /* Unspecified keeps the 4x default in GameArgs. */
    if(msaa == "none") { args.msaa_samples = 0; }
    else if(msaa == "2x") { args.msaa_samples = 2; }
    else if(msaa == "4x") { args.msaa_samples = 4; }
    else if(msaa == "8x") { args.msaa_samples = 8; }

    /* CLI11's range validators are `val < min || val > max` -- both false for
       NaN, and its float lexer parses "nan". A NaN dt never satisfies the tick
       loop's `accumulator >= dt` (silent freeze) and a NaN --timeout never
       expires (hang), so every numeric knob gets a finiteness check here. */
    {
        const std::pair<const char *, double> numeric[] = {
            {"--physics-hz", args.physics_hz},
            {"--start-time", args.start_time},
            {"--timeout", args.timeout_seconds},
            {"--exhaust-scale", args.exhaust_scale},
            {"--cam-shake", args.cam_shake},
            {"--drag-cd", args.drag_cd},
            {"--orbit-interval", args.orbit_interval},
            {"--prox-fly-on", args.prox_fly_on},
            {"--prox-fly-off", args.prox_fly_off},
            {"--prox-ground-on", args.prox_ground_on},
            {"--prox-ground-off", args.prox_ground_off},
            {"--prox-warp", args.prox_warp},
            {"--sky-dim", args.sky_dim},
            {"--sky-dim-cone", args.sky_dim_cone},
            {"--font-size", args.font_size},
            {"--fov", args.camFovDeg},
        };
        for(const auto &[name, value] : numeric) {
            if(!std::isfinite(value)) {
                printf("error: %s must be a finite number (got %g)\n", name, value);
                *exit_code = 1;
                return false;
            }
        }
        const std::pair<const char *, const std::vector<double> &> vectors[] = {
            {"--free-cam-pos", args.free_cam_pos},
            {"--free-cam-fwd", args.free_cam_fwd},
            {"--free-cam-up", args.free_cam_up},
        };
        for(const auto &[name, values] : vectors) {
            for(double v : values) {
                if(!std::isfinite(v)) {
                    printf("error: %s must be finite numbers (got %g)\n", name, v);
                    *exit_code = 1;
                    return false;
                }
            }
        }
    }

    /* --vab-assign: group the flat list into MS,PART_IDX,SLOT,KERBAL quads. */
    if(!vab_assign.empty()) {
        if(vab_assign.size() % 4 != 0) {
            printf("error: --vab-assign expects MS,PART_IDX,SLOT,KERBAL "
                   "groups; got %zu value(s)\n", vab_assign.size());
            *exit_code = 1;
            return false;
        }
        for(size_t i = 0; i < vab_assign.size(); i += 4) {
            long v[3] = {0, 0, 0};
            static const char *kFields[3] = { "time", "part index", "slot" };
            for(int f = 0; f < 3; f++) {
                const std::string &s = vab_assign[i + (size_t)f];
                char *end = nullptr;
                v[f] = strtol(s.c_str(), &end, 10);
                if(end == s.c_str() || *end != '\0') {
                    printf("error: --vab-assign %s '%s' is not an integer\n",
                           kFields[f], s.c_str());
                    *exit_code = 1;
                    return false;
                }
            }
            /* "-" is the seat window's "Empty": it records the choice that
               nobody sits in this seat (issue #287), and the dropdown offers it
               as a real row now. --vab-assign survives for what a click cannot
               do: naming the SAME person twice, which the dropdown greys out
               (gameui.cpp:3663) so the launch-refusal guard has no other way
               in, and reaching a seat on a multi-part build, where a fixed
               pixel pick lands on whatever part is under it (#311). CLI11 drops
               a trailing empty field, so the sentinel has to be a token rather
               than an empty KERBAL. */
            if(vab_assign[i + 3].empty()) {
                printf("error: --vab-assign needs a kerbal name, or '-' for an "
                       "empty seat\n");
                *exit_code = 1;
                return false;
            }
            args.vab_assigns.push_back({(int)v[0], (int)v[1], (int)v[2],
                                        vab_assign[i + 3]});
        }
    }

    /* --vab-link: group the flat list into MS,FROM_ID,TO_ID triples. CLI11
       drops an empty delimited field, so a missing id lands here as a short
       group and the count check names it. */
    if(!vab_link.empty()) {
        if(vab_link.size() % 3 != 0) {
            printf("error: --vab-link expects MS,FROM_ID,TO_ID groups; "
                   "got %zu value(s)\n", vab_link.size());
            *exit_code = 1;
            return false;
        }
        for(size_t i = 0; i < vab_link.size(); i += 3) {
            const std::string &s = vab_link[i];
            char *end = nullptr;
            const long ms = strtol(s.c_str(), &end, 10);
            if(end == s.c_str() || *end != '\0') {
                printf("error: --vab-link time '%s' is not an integer\n",
                       s.c_str());
                *exit_code = 1;
                return false;
            }
            args.vab_links.push_back({(int)ms, vab_link[i + 1], vab_link[i + 2]});
        }
    }

    /* --select-target: group the flat list into MS,NAME pairs. */
    if(!select_target.empty()) {
        if(select_target.size() % 2 != 0) {
            printf("error: --select-target expects MS,NAME groups; "
                   "got %zu value(s)\n", select_target.size());
            *exit_code = 1;
            return false;
        }
        for(size_t i = 0; i < select_target.size(); i += 2) {
            const std::string &s = select_target[i];
            char *end = nullptr;
            long ms = strtol(s.c_str(), &end, 10);
            if(end == s.c_str() || *end != '\0') {
                printf("error: --select-target time '%s' is not an integer\n",
                       s.c_str());
                *exit_code = 1;
                return false;
            }
            if(select_target[i + 1].empty()) {
                printf("error: --select-target needs a target name, or 'none'\n");
                *exit_code = 1;
                return false;
            }
            args.select_targets.push_back({(int)ms, select_target[i + 1]});
        }
    }

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

    /* X,Y are signed (the cursor can move up/left from where it was), so they
       parse as strtol, unlike the unsigned times above. */
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

    /* Split each AT_MS,WINDOW/LABEL value at the FIRST comma -- the path itself
       may contain commas. */
    for(const std::string &spec : ui_click) {
        Uint32 at_ms = 0;
        std::string path;
        if(!parse_ui_time(spec, "--ui-click", at_ms, path)) {
            *exit_code = 1;
            return false;
        }
        if(path.empty()) {
            printf("error: --ui-click '%s' has an empty WINDOW/LABEL path\n",
                   spec.c_str());
            *exit_code = 1;
            return false;
        }
        UiClick c;
        c.at_ms = at_ms;
        c.path = path;
        c.down_sent = false;
        c.up_sent = false;
        c.done = false;
        args.ui_clicks.push_back(c);
    }

    /* --ui-type is a --ui-click that keeps going, so it lands in the same
       queue. The path ends at the SECOND comma, which means a path containing a
       comma cannot be named here; TEXT may contain anything. */
    for(const std::string &spec : ui_type) {
        Uint32 at_ms = 0;
        std::string rest;
        if(!parse_ui_time(spec, "--ui-type", at_ms, rest)) {
            *exit_code = 1;
            return false;
        }
        const size_t comma = rest.find(',');
        if(comma == std::string::npos) {
            printf("error: --ui-type expects AT_MS,WINDOW/LABEL,TEXT (e.g. "
                   "2000,\"##newgame_name\",mygame); got '%s'\n", spec.c_str());
            *exit_code = 1;
            return false;
        }
        const std::string path = rest.substr(0, comma);
        const std::string text = rest.substr(comma + 1);
        if(path.empty()) {
            printf("error: --ui-type '%s' has an empty WINDOW/LABEL path\n",
                   spec.c_str());
            *exit_code = 1;
            return false;
        }
        if(text.empty()) {
            printf("error: --ui-type '%s' has empty TEXT\n", spec.c_str());
            *exit_code = 1;
            return false;
        }
        UiClick c;
        c.at_ms = at_ms;
        c.path = path;
        c.down_sent = false;
        c.up_sent = false;
        c.done = false;
        c.type_after = text;
        args.ui_clicks.push_back(c);
    }

    /* The harness serves the queue in order, one press per frame, and skips an
       entry whose widget is not drawn yet rather than blocking on it -- so a
       later op could otherwise fire before an earlier one settles. --ui-click
       and --ui-type are parsed into one vector, so a --ui-type written after a
       --ui-click sits behind it regardless of its timestamp. Order by when they
       are asked for; stable, so equal timestamps keep the author's order. */
    std::stable_sort(args.ui_clicks.begin(), args.ui_clicks.end(),
                     [](const UiClick &a, const UiClick &b) {
                         return a.at_ms < b.at_ms;
                     });

    // Any of the --free-cam-* options opts in to starting in free-cam mode.
    args.use_free_cam = !args.free_cam_pos.empty() || !args.free_cam_fwd.empty()
                            || !args.free_cam_up.empty();

    args.window_mode =
        exclusive  ? WindowMode::Exclusive
        : fullscreen ? WindowMode::Fullscreen
        : borderless ? WindowMode::Borderless
                     : WindowMode::Windowed;

    /* Which settings the CLI set explicitly (CLI11 ->count()): the settings.json
       load honors this mask -- the command line beats the saved file, field by
       field. */
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
    args.cli_given.start_time =
        app.get_option("--start-time")->count() > 0;
    args.cli_given.cam_shake =
        app.get_option("--cam-shake")->count() > 0;
    args.cli_given.time_accel =
        app.get_option("--time-accel")->count() > 0;
    args.cli_given.sky_dim =
        app.get_option("--sky-dim")->count() > 0;
    args.cli_given.sky_dim_cone =
        app.get_option("--sky-dim-cone")->count() > 0;

    if(!args.save_name.empty() && !args.load_name.empty()) {
        printf("error: --save and --load are mutually exclusive\n");
        *exit_code = 1;
        return false;
    }
    if(!args.vab.empty() && args.vab_empty) {
        printf("error: --vab and --vab-empty are mutually exclusive\n");
        *exit_code = 1;
        return false;
    }
    if(!args.save_name.empty() && args.timeout_seconds <= 0.0) {
        printf("error: --save needs --timeout (the save is taken when the "
               "wall-clock budget is spent)\n");
        *exit_code = 1;
        return false;
    }
    if(!args.resave_name.empty() && args.timeout_seconds <= 0.0) {
        printf("error: --resave needs --timeout (the save is taken when the "
               "wall-clock budget is spent)\n");
        *exit_code = 1;
        return false;
    }

    return true;
}
