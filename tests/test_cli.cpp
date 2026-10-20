// test_cli.cpp -- cli.cpp: parse_cli's short-circuit paths (--version,
// --help) and the basic parse contract (a valid flag set succeeds, an
// unknown flag fails with a nonzero exit code). CLI11 prints help and
// version to std::cout, so we capture that (rdbuf swap, restored before
// returning) and check the exit code + content. No SDL: cli.cpp is
// CLI11 + plain C++ only.
#include <cassert>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include "cli.h"
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

    // 3c) --ui-click: one value per click, split at the FIRST comma so the
    //     path keeps its own commas (and its spaces, which the shell / the
    //     e2e ARGS line quote away). --ui-list is the one-shot dump time.
    {
        GameArgs args;
        const char *argv[] = {"osp",
                              "--ui-click", "1500,Title Menu/New Game",
                              "--ui-click", "3000,Game Menu/Tracking Station",
                              "--ui-click", "2500,Save/Load,slot 1",
                              "--ui-list", "1200"};
        bool ok = false;
        int code = -1;
        run(9, (char **)argv, args, ok, code);
        assert(ok);
        assert(args.ui_list_ms == 1200);
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

    printf("test_cli: all checks passed\n");
    return 0;
}
