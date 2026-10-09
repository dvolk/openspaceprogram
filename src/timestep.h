#pragma once

/* How the tick loop (src/tick.cpp) splits one tick of `step` simulated seconds
   into Bullet substeps: each substep stays <= kMaxSubStep, never fewer than 3
   (the low-accel baseline), never more than kMaxSubSteps (a hitch must not
   stall the frame).

   The unit tests mirror the tick loop through this, so it is the only
   definition -- a test that re-derives it drifts when the loop moves. Kept
   free of any other declaration so the standalone math tests can include it
   without pulling in the physics engine (or colliding with their own local
   `Body`). */
constexpr double kMaxSubStep = 0.1;
constexpr int kMaxSubSteps = 2000;

inline int substepCount(double step) {
    int n = (int)(step / kMaxSubStep + 0.5);
    if (n < 3) { n = 3; }
    if (n > kMaxSubSteps) { n = kMaxSubSteps; }
    return n;
}
