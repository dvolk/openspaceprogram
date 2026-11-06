// test_cli.cpp -- cli.cpp: parse_cli's short-circuit paths (--version,
// --help) and the basic parse contract (a valid flag set succeeds, an
// unknown flag fails with a nonzero exit code). CLI11 prints help and
// version to std::cout, so we capture that (rdbuf swap, restored before
// returning) and check the exit code + content. No SDL: cli.cpp is
// CLI11 + plain C++ only.
#include <cassert>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

#include "cli.h"
#include "timestep.h"   // scriptedClockMs: the --sim-steps clock
#include "version.h"

/* Run parse_cli with stdout captured. `ok` is its return, `code` its
   exit code; returns what it printed. */
static std::string run(int argc, char **argv, GameArgs &args, bool &ok, int &code) {
    std::stringstream ss;
    std::streambuf *old = std::cout.rdbuf(ss.rdbuf());
    ok = parse_cli(argc, argv, args, &code);
    std::cout.rdbuf(old);
    return ss.str();
}

int main() {
    // 1) --version: short-circuits the parse (returns false, like --help),
    //    exits 0, and prints the embedded version string (so the output
    //    always matches the main menu footer's VERSION).
    {
        const char *argv[] = {"osp", "--version"};
        GameArgs args;
        bool ok = true;
        int code = -1;
        std::string out = run(2, (char **)argv, args, ok, code);
        assert(!ok);
        assert(code == 0);
        assert(out.find(VERSION) != std::string::npos);
    }

    // 2) --help: short-circuits, exits 0, prints the flag list (a couple
    //    of well-known flags must be in there).
    {
        const char *argv[] = {"osp", "--help"};
        GameArgs args;
        bool ok = true;
        int code = -1;
        std::string out = run(2, (char **)argv, args, ok, code);
        assert(!ok);
        assert(code == 0);
        assert(out.find("--startship") != std::string::npos);
        assert(out.find("--version") != std::string::npos);
    }

    // 2b) The state logs' --help text states the cadence tick.cpp gates them
    //     at. That number lives in cli.h (kStateLogMs / kSlewLogMs) AND in the
    //     help string, so this is what stops --help silently lying when one
    //     side moves. CLI11 re-wraps descriptions to the terminal width, so
    //     match the short token rather than a whole phrase.
    {
        const char *argv[] = {"osp", "--help"};
        GameArgs args;
        bool ok = true;
        int code = -1;
        std::string out = run(2, (char **)argv, args, ok, code);
        char state_s[16], slew_s[16];
        snprintf(state_s, sizeof state_s, "%.1fs", kStateLogMs / 1000.0);
        snprintf(slew_s, sizeof slew_s, "%.1fs", kSlewLogMs / 1000.0);
        /* The flag's own description: CLI11 puts the entry on its own line and
           wraps the text as a hanging indent. Anchor on the ENTRY, not the
           first mention -- --orbit-interval's description names all six of
           these flags, and its wrap can start a line with one of them. */
        auto block = [&](const char *flag) {
            const size_t n = strlen(flag);
            std::string b;
            bool in = false;
            std::istringstream ls(out);
            std::string line;
            while(std::getline(ls, line)) {
                const size_t s = line.find_first_not_of(" \t");
                if(s == std::string::npos) { continue; }
                const std::string t = line.substr(s);
                const bool entry = t.compare(0, n, flag) == 0
                                && t.size() > n && (t[n] == ' ' || t[n] == '\t');
                if(entry) { b = t; in = true; continue; }
                if(in) {
                    if(t.compare(0, 2, "--") == 0) { break; }
                    b += " " + t;
                }
            }
            assert(in);
            return b;
        };
        for(const char *flag : {"--spin-log", "--mass-log", "--fuel-log",
                                "--drain-log", "--power-log", "--steering-log"}) {
            assert(block(flag).find(state_s) != std::string::npos);
        }
        assert(block("--slew-log").find(slew_s) != std::string::npos);
    }

    // 2c) The log gate's floor. An --orbit-interval under one physics tick
    //     passes CLI11's finiteness and positivity checks and then truncates
    //     to 0ms in the gate, so every throttled log fires on every tick. A
    //     log cannot sample faster than the sim steps, so refuse it. (The
    //     refusal message goes through printf, not std::cout, so it is not in
    //     the captured output -- same as the --ui-scroll refusals above.)
    {
        const char *argv[] = {"osp", "--orbit-interval", "0.0001"};
        GameArgs args;
        bool ok = true;
        int code = -1;
        run(3, (char **)argv, args, ok, code);
        assert(!ok);
        assert(code == 1);
    }
    // The floor tracks --physics-hz rather than being a magic constant: the
    // SAME 0.02 s is accepted at 100 Hz (one tick = 0.01 s) and refused at
    // 20 Hz (one tick = 0.05 s).
    {
        const char *argv[] = {"osp", "--physics-hz", "100", "--orbit-interval", "0.02"};
        GameArgs args;
        bool ok = false;
        int code = -1;
        run(5, (char **)argv, args, ok, code);
        assert(ok);
    }
    {
        const char *argv[] = {"osp", "--physics-hz", "20", "--orbit-interval", "0.02"};
        GameArgs args;
        bool ok = true;
        int code = -1;
        run(5, (char **)argv, args, ok, code);
        assert(!ok);
        assert(code == 1);
    }

    // 2d) The log gate's CEILING (#673). Every periodic log counts its interval
    //     in Uint32 MILLISECONDS, so a wait past ~49.7 days is UB and lands on an
    //     arbitrary rate: 1e12 s asks for a quieter log and gets a spammy one.
    //     Refuse at the ceiling rather than wrap.
    {
        const char *argv[] = {"osp", "--orbit-interval", "1e12"};
        GameArgs args;
        bool ok = true;
        int code = -1;
        run(3, (char **)argv, args, ok, code);
        assert(!ok);
        assert(code == 1);
    }
    {
        const char *argv[] = {"osp", "--record", "--record-interval", "1e12"};
        GameArgs args;
        bool ok = true;
        int code = -1;
        run(4, (char **)argv, args, ok, code);
        assert(!ok);
        assert(code == 1);
    }
    // An unused knob is not worth refusing, same as the floor's rule (#662): the
    // ceiling belongs to the recorder, so without --record the value parses. The
    // helper below is what keeps even that case out of UB.
    {
        const char *argv[] = {"osp", "--record-interval", "1e12"};
        GameArgs args;
        bool ok = false;
        int code = -1;
        run(3, (char **)argv, args, ok, code);
        assert(ok);
    }
    // --help states the ceiling for both knobs, in the number cli.h holds. The
    // same drift rule as the state-log cadences in 2b: the help and the law are
    // written twice, so this is what makes them move together.
    {
        const char *argv[] = {"osp", "--help"};
        GameArgs args;
        bool ok = true;
        int code = -1;
        std::string out = run(2, (char **)argv, args, ok, code);
        char ceil_s[16];
        snprintf(ceil_s, sizeof ceil_s, "%.0fs", kIntervalCeilingS);
        size_t hits = 0;
        for(size_t p = out.find(ceil_s); p != std::string::npos; p = out.find(ceil_s, p + 1)) { hits++; }
        assert(hits == 2);
    }
    // 2e) The conversion itself: one law for every interval knob (#673), so the
    //     saturation is pinned at the end that used to be UB, and the 0 that the
    //     gates read as "no throttle" at the other.
    assert(gateIntervalMs(1.0) == 1000u);
    assert(gateIntervalMs(0.25) == 250u);
    assert(gateIntervalMs(kIntervalCeilingS) == 3600000u);
    assert(gateIntervalMs(1e12) == 3600000u);
    assert(gateIntervalMs(std::numeric_limits<double>::infinity()) == 3600000u);
    assert(gateIntervalMs(0.0) == 0u);
    assert(gateIntervalMs(-5.0) == 0u);
    assert(gateIntervalMs(std::numeric_limits<double>::quiet_NaN()) == 0u);

    // 3) a valid flag set parses (true), filling the field.
    {
        const char *argv[] = {"osp", "--timeout", "5"};
        GameArgs args;
        bool ok = false;
        int code = -1;
        run(3, (char **)argv, args, ok, code);
        assert(ok);
        assert(args.timeout_seconds == 5.0);
    }

    // 3b) --recover-anywhere: off by default, on when passed. It is a
    //     permission, not a way to act -- the cases pair it with a click on
    //     the hub's Recover Vessel.
    {
        GameArgs args;
        assert(!args.recover_anywhere);
        const char *argv[] = {"osp", "--recover-anywhere"};
        bool ok = false;
        int code = -1;
        run(2, (char **)argv, args, ok, code);
        assert(ok);
        assert(args.recover_anywhere);
    }

    // 3b') --allow-part-geom-drift: off by default, because the default for a
    //      save whose parts changed shape is to refuse it (#456). On, it loads
    //      the ship reshaped and says so -- so the flag's spelling is part of
    //      the contract the refusal message points at.
    {
        GameArgs args;
        assert(!args.allow_part_geom_drift);
        const char *argv[] = {"osp", "--allow-part-geom-drift"};
        bool ok = false;
        int code = -1;
        run(2, (char **)argv, args, ok, code);
        assert(ok);
        assert(args.allow_part_geom_drift);
    }

    // 3c) --ui-click: one value per click, split at the FIRST comma so the
    //     path keeps its own commas (and its spaces, which the shell / the
    //     e2e ARGS line quote away). --ui-list takes one time per flag and is
    //     sorted like the click queue, so a case may write its dumps in any
    //     order and still get them in time order.
    {
        GameArgs args;
        const char *argv[] = {"osp",
                              "--ui-click", "1500,Title Menu/New Game",
                              "--ui-click", "3000,Game Menu/Tracking Station",
                              "--ui-click", "2500,Save/Load,slot 1",
                              "--ui-list", "5000",
                              "--ui-list", "1200"};
        bool ok = false;
        int code = -1;
        run(11, (char **)argv, args, ok, code);
        assert(ok);
        assert(args.ui_list_ms.size() == 2);
        assert(args.ui_list_ms[0] == 1200);
        assert(args.ui_list_ms[1] == 5000);
        assert(args.ui_clicks.size() == 3);
        // Given out of time order on purpose: the queue is served in order and
        // an entry whose widget is not drawn yet is skipped rather than waited
        // on, so a later op could otherwise fire before an earlier one settles.
        assert(args.ui_clicks[0].at_ms == 1500);
        assert(args.ui_clicks[0].path == "Title Menu/New Game");
        assert(args.ui_clicks[1].at_ms == 2500);
        assert(args.ui_clicks[1].path == "Save/Load,slot 1");
        assert(args.ui_clicks[2].at_ms == 3000);
        assert(args.ui_clicks[2].path == "Game Menu/Tracking Station");
        // Only the first comma splits: the path keeps the rest of them.
        assert(args.ui_clicks[0].type_after.empty());
    }

    // 3c4) --ui-scroll: TARGET may carry commas, NOTCHES is the last field.
    //       Entries are sorted by time like the click queue.
    {
        GameArgs args;
        const char *argv[] = {"osp",
                              "--ui-scroll", "2000,##Combo_00,-1",
                              "--ui-scroll", "1500,Save/Load,slot 1,3"};
        bool ok = false;
        int code = -1;
        run(5, (char **)argv, args, ok, code);
        assert(ok);
        assert(args.ui_scrolls.size() == 2);
        assert(args.ui_scrolls[0].at_ms == 1500);
        assert(args.ui_scrolls[0].target == "Save/Load,slot 1");
        assert(args.ui_scrolls[0].notches == 3);
        assert(args.ui_scrolls[1].at_ms == 2000);
        assert(args.ui_scrolls[1].target == "##Combo_00");
        assert(args.ui_scrolls[1].notches == -1);
    }

    // 3c5) --ui-scroll needs both commas, a target and nonzero notches.
    for(const char *bad : {"2000,##Combo_00", "2000,,1", "2000,##Combo_00,0",
                           "2000,##Combo_00,101"}) {
        GameArgs args;
        const char *argv[] = {"osp", "--ui-scroll", bad};
        bool ok = true;
        int code = 0;
        run(3, (char **)argv, args, ok, code);
        assert(!ok);
        assert(code == 1);
        assert(args.ui_scrolls.empty());
    }

    // 3c2) --ui-type: the path ends at the SECOND comma, so TEXT keeps the rest
    //      of them, and the entries share --ui-click's queue and its sort.
    {
        GameArgs args;
        const char *argv[] = {"osp",
                              "--ui-type", "4000,##newslot,slot, one",
                              "--ui-click", "3000,Save/Load"};
        bool ok = false;
        int code = -1;
        run(5, (char **)argv, args, ok, code);
        assert(ok);
        assert(args.ui_clicks.size() == 2);
        assert(args.ui_clicks[0].at_ms == 3000);
        assert(args.ui_clicks[0].path == "Save/Load");
        assert(args.ui_clicks[0].type_after.empty());
        assert(args.ui_clicks[1].at_ms == 4000);
        assert(args.ui_clicks[1].path == "##newslot");
        assert(args.ui_clicks[1].type_after == "slot, one");
    }

    // 3c3) --ui-type needs both commas, a path and non-empty TEXT: an empty
    //      TEXT would be a click that silently types nothing.
    for(const char *bad : {"4000,##newslot", "4000,,name", "4000,##newslot,"}) {
        GameArgs args;
        const char *argv[] = {"osp", "--ui-type", bad};
        bool ok = true;
        int code = 0;
        run(3, (char **)argv, args, ok, code);
        assert(!ok);
        assert(code == 1);
        assert(args.ui_clicks.empty());
    }

    // 3d) --ui-click without the AT_MS comma fails (the path alone is not a
    //     time). The message goes to stdout through printf, which the cout
    //     capture above does not see, so only the outcome is checked.
    {
        GameArgs args;
        const char *argv[] = {"osp", "--ui-click", "Title Menu/New Game"};
        bool ok = true;
        int code = 0;
        run(3, (char **)argv, args, ok, code);
        assert(!ok);
        assert(code == 1);
        assert(args.ui_clicks.empty());
    }

    // 3e) a negative or oversized AT_MS fails rather than wrapping into a
    //     click at some surprising loop time (strtoul accepts "-5").
    {
        GameArgs args;
        const char *argv[] = {"osp", "--ui-click", "-5,Title Menu/New Game"};
        bool ok = true;
        int code = 0;
        run(3, (char **)argv, args, ok, code);
        assert(!ok);
        assert(code == 1);
        assert(args.ui_clicks.empty());
    }

    // 4) an unknown flag fails with a nonzero exit code (main() exits with
    //    it) and prints nothing to stdout (CLI11 routes errors to stderr).
    {
        const char *argv[] = {"osp", "--no-such-flag"};
        GameArgs args;
        bool ok = false;
        int code = 0;
        std::string out = run(2, (char **)argv, args, ok, code);
        assert(!ok);
        assert(code != 0);
        assert(out.empty());
    }

    // 5) sim_parse_key: the --sim-press key vocabulary. Names must beat the
    //    decimal escape (#108: "3" parsed as keycode 3, so the digit keys were
    //    unreachable from --sim-press), and every key the Controls window can
    //    bind has to be nameable -- "[" and "]" are the DEFAULT camera-speed
    //    keys and had no name at all, leaving a raw keycode as the only way to
    //    press one.
    {
        assert(sim_parse_key("3") == SDLK_3);
        assert(sim_parse_key("32") == SDLK_SPACE);      // the decimal escape
        assert(sim_parse_key("[") == SDLK_LEFTBRACKET);
        assert(sim_parse_key("LEFTBRACKET") == SDLK_LEFTBRACKET);
        assert(sim_parse_key("]") == SDLK_RIGHTBRACKET);
        assert(sim_parse_key("RIGHTBRACKET") == SDLK_RIGHTBRACKET);
        assert(sim_parse_key("MINUS") == SDLK_MINUS);
        assert(sim_parse_key("EQUALS") == SDLK_EQUALS);
        assert(sim_parse_key("SLASH") == SDLK_SLASH);
        assert(sim_parse_key("BACKSPACE") == SDLK_BACKSPACE);
        assert(sim_parse_key("LALT") == SDLK_LALT);
        assert(sim_parse_key("nope") == 0);
    }

    // 3f) --sim-steps: off by default (-1), and a positive tick count. Zero is
    //     refused: it exits before the first tick, so the run could not observe
    //     anything -- which is exactly the silent no-op the mode must not become.
    {
        GameArgs args;
        assert(args.sim_steps == -1);
        const char *argv[] = {"osp", "--sim-steps", "840"};
        bool ok = false;
        int code = -1;
        run(3, (char **)argv, args, ok, code);
        assert(ok);
        assert(args.sim_steps == 840);
    }
    for(const char *bad : {"0", "-1", "abc"}) {
        GameArgs args;
        const char *argv[] = {"osp", "--sim-steps", bad};
        bool ok = true;
        int code = 0;
        run(3, (char **)argv, args, ok, code);
        assert(!ok);
        // CLI11's own validator reports these, so the code is its ParseError
        // rather than the 1 our hand-written checks use. Only nonzero matters:
        // main() exits with it.
        assert(code != 0);
    }

    // 3g) The scripted clock (#472): ms counted from TICKS. It must be monotone
    //     and land exactly on a whole ms boundary when N ticks meet the log
    //     interval -- 600 ticks at 60 Hz = exactly 10000 ms, so a 1 s log gate
    //     fires its last row inside the run instead of one tick too late. And it
    //     must survive nonsense rather than hit the UB of casting a negative
    //     double to an integer type.
    {
        assert(scriptedClockMs(0, 1.0 / 60.0) == 0);
        assert(scriptedClockMs(600, 1.0 / 60.0) == 10000);
        assert(scriptedClockMs(840, 1.0 / 60.0) == 14000);
        assert(scriptedClockMs(-5, 1.0 / 60.0) == 0);
        // A long run saturates instead of wrapping, or a schedule could land at
        // the start of the run again.
        assert(scriptedClockMs(4000000000LL, 1.0 / 60.0) == 4294967295U);
        bool monotone = true;
        for(long long n = 1; n <= 2000; ++n) {
            if(scriptedClockMs(n, 1.0 / 60.0) <= scriptedClockMs(n - 1, 1.0 / 60.0)) {
                monotone = false;
            }
        }
        assert(monotone);
        // --physics-hz is range-checked 5..1000 in cli.cpp, so one tick is
        // always >= 1 ms: that is what keeps the clock strictly increasing (a
        // sub-ms tick would make repeated stamps fall due on the same ms) at
        // either allowed extreme.
        assert(scriptedClockMs(5, 0.2) == 1000);       // 5 Hz
        assert(scriptedClockMs(1000, 0.001) == 1000);  // 1000 Hz
        // And the rounding DIRECTION, which is the whole reason a scripted run
        // keeps its last log row: one tick at 60 Hz is 16.67 ms, and a truncated
        // ladder slips ~16 ms per gate -- 840 steps then prints 13 [drag] rows
        // instead of 14.
        assert(scriptedClockMs(1, 1.0 / 60.0) == 17);
    }

    // 3h) cli_given.frame_cap / cli_given.idle_fps: what a scripted run reads to
    //     know the case ASKED for a frame rate or the idle throttle. Get these
    //     wrong and a case's own --frame-cap is silently lifted (its slow frames
    //     were the thing under test), or the idle path stops being exercised by
    //     the --force-occluded cases the moment they move to --sim-steps.
    {
        GameArgs args;
        const char *argv[] = {"osp", "--sim-steps", "840"};
        bool ok = false;
        int code = -1;
        run(3, (char **)argv, args, ok, code);
        assert(ok);
        assert(!args.cli_given.frame_cap);
        assert(!args.cli_given.idle_fps);
        assert(args.frame_cap == 60);   // the default still stands; the MODE lifts it
    }
    {
        GameArgs args;
        const char *argv[] = {"osp", "--sim-steps", "840",
                              "--frame-cap", "30", "--idle-fps", "5"};
        bool ok = false;
        int code = -1;
        run(7, (char **)argv, args, ok, code);
        assert(ok);
        assert(args.cli_given.frame_cap);
        assert(args.cli_given.idle_fps);
        assert(args.frame_cap == 30);
        assert(args.idle_fps == 5);
    }

    printf("test_cli: all checks passed\n");
    return 0;
}
