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
        assert(out.find("--body") != std::string::npos);
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
