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

    // 3b) --recover-anywhere: off by default, on when passed (with the
    //     --recover hook it travels with).
    {
        GameArgs args;
        assert(!args.recover_anywhere);
        const char *argv[] = {"osp", "--recover-anywhere", "--recover", "2000"};
        bool ok = false;
        int code = -1;
        run(4, (char **)argv, args, ok, code);
        assert(ok);
        assert(args.recover_anywhere);
        assert(args.recover_ms == 2000);
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
        assert(args.ui_clicks[0].at_ms == 1500);
        assert(args.ui_clicks[0].path == "Title Menu/New Game");
        assert(args.ui_clicks[1].at_ms == 3000);
        assert(args.ui_clicks[1].path == "Game Menu/Tracking Station");
        // Only the first comma splits: the path keeps the rest of them.
        assert(args.ui_clicks[2].at_ms == 2500);
        assert(args.ui_clicks[2].path == "Save/Load,slot 1");
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

    printf("test_cli: all checks passed\n");
    return 0;
}
