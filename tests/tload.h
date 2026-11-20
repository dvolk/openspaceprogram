#pragma once
/* Reading a SHIPPED system file in a test (#720).

   system.cpp refuses malformed generated data by throwing, which is the right call for the
   game: a file it cannot honour should stop the boot rather than fly on wrong air. In a test
   the same throw is SIGABRT, and an abort takes every later check in that binary with it --
   one body with `"wind": []` in a generated file left `make test` with

     terminate called after throwing an instance of 'std::runtime_error'
     what():  system: Venus: atmosphere.wind is empty -- omit the key ...

   and no list of what else was broken. A file the game refuses to start IS a test failure,
   so it should be reported as one and let the rest of the run finish.

   Use this where the file is an INPUT the suite does not own -- the shipped catalogue. A test
   that mutates a system to prove the loader rejects it keeps its own try/catch, because the
   throw is the assertion there; see expect_reject / expect_refuses in test_belts.cpp,
   test_skybox.cpp and test_scenarios.cpp, which already reported instead of aborting. */

#include <exception>
#include <cstdio>

#include "system.h"
#include "tchecks.h"

// Loads `path`, counting a refusal as one failed check. Returns false in that case, so the
// caller skips the body of its assertions instead of poking a half-built System -- a default
// System has a null `root`, and the first rail call would trade SIGABRT for SIGSEGV.
inline bool load_shipped(const char *path, System &sys) {
    try {
        sys = load_system(path, nullptr, nullptr);
        return true;
    } catch(const std::exception &e) {
        g_checks++;
        g_failures++;
        printf("FAIL: the loader refused a shipped file: %s: %s\n", path, e.what());
        return false;
    }
}
