#pragma once

#include <cmath>
#include <cstdint>

/* How the tick loop (src/tick.cpp) splits one tick of `step` simulated seconds
   into Bullet substeps: each <= kMaxSubStep, never fewer than 3 (the low-accel
   baseline), never more than kMaxSubSteps (a hitch must not stall the frame).

   The unit tests mirror the tick loop through this, so it is the only
   definition -- a test that re-derives it drifts. This header must stay free
   of other declarations so the standalone math tests can include it. */
constexpr double kMaxSubStep = 0.1;
constexpr int kMaxSubSteps = 2000;

inline int substepCount(double step) {
    int n = (int)(step / kMaxSubStep + 0.5);
    if (n < 3) { n = 3; }
    if (n > kMaxSubSteps) { n = kMaxSubSteps; }
    return n;
}

/* --sim-steps (cli.cpp) keys a scripted run's input schedules and log gates to
   TICKS RUN, not to the wall clock and not to g.time.

   Not g.time, even though that is the sim clock: a mid-run save load or New Game
   rewrites g.time (save.cpp's setTime, game.cpp's newGame), so a base captured at
   boot can end up behind it -- and converting a negative double to uint32_t is
   undefined behaviour, which in practice quietly guts the log gates. A paused sim
   also freezes g.time while the step budget keeps draining, so the scripted
   presses never fall due at all. Both were reproduced in review.
   Ticks run is monotone by construction, and it matches what the option
   promises: N steps at --physics-hz is N/hz seconds of RUN time. Sim seconds are
   a different number -- at --time-accel 100 one tick is 100xdt of sim but still
   only dt of run -- and this clock deliberately tracks the run. */
inline uint32_t scriptedClockMs(long long ticksRun, double dt) {
    if(ticksRun < 0) { ticksRun = 0; }
    const double ms = (double)ticksRun * dt * 1000.0;
    // Round rather than truncate: N steps at 60 Hz must land exactly on the log
    // interval, or the run's last row falls off the end.
    double whole = std::llround(ms);
    if(whole > 4294967295.0) { whole = 4294967295.0; }   // ~49.7 days of run
    return (uint32_t)whole;
}
