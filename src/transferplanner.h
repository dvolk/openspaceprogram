// transferplanner.h -- the game-side transfer planner: the TRANSFER window's
// target selection + the min-dv solver cache.
//
// The pure-math solver (Lambert + planTransfer) stays in transfer.h,
// header-only, so it can be pinned without the game. This class owns the
// game-side state that was main() locals -- xferTargets / xfer_target /
// xfer_auto / xfer_tof_log / the solver cache `xfer` / the --xfer-log
// gate -- and the per-frame work that was a render-pass block in main:
// rebuild the target list, recompute the solution on input change or
// every 30 frames, and fire the --xfer-log. The implementation is in
// transferplanner.cpp; everything reads and writes state through the
// planner's own members or the borrowed Game.
#pragma once

#include <vector>

#include "game.h"       // Game (Ships / System / GameArgs / the sim clock)
#include "transfer.h"   // TransferSolution

class TransferPlanner {
public:
    struct XferTarget {
        const char *name;
        TerrainBody *body;   // body target (capture available)
        Vehicle *ship;       // ship target (intercept only)
    };

    explicit TransferPlanner(Game &g) : g(g) {}

    /* Per-frame (render pass): rebuild the target list from the ship's
       parent's children + the sibling ships, recompute the solution on
       input change or every 30 frames (the plan changes slowly relative
       to the ToF scale, and the sweep is the cost center), and fire the
       --xfer-log. com / vel are the active ship's render-frame COM and
       velocity -- the render pass's scratch -- so the solver sees exactly
       what the readouts use. */
    void update(const glm::dvec3 &com, const glm::dvec3 &vel);

    /* On-demand (the P key / the window's button): build the porkchop
       plot for the current target from the ship's current state and cache
       it in pc until the next call. No-op when there is no target. */
    void porkchopCompute();

    std::vector<XferTarget> xferTargets;
    int xfer_target = -1;
    bool xfer_auto = true;                     // auto min-dv ToF vs pinned
    float xfer_tof_log = (float)std::log10(3600.0); // log10(s), the pinned ToF
    struct {
        int target = -2;        // target index at last compute
        bool auto_tof = true;
        double tof_log = -1.0;
        int frame = 0;          // per-frame counter while a target is set
        int solved_frame = -1000000; // xfer.frame at last recompute
        bool valid = false;
        TransferSolution sol;
        glm::dvec3 burn_dir = glm::dvec3(0.0); // render-frame burn direction
    } xfer;
    // --xfer-log: its own "last fired" timestamp (it shares the game's
    // orbit_log_interval_ms, like the orbit/dbg logs).
    Uint32 xfer_log_last_ms = 0;

    /* Porkchop plot (on-demand): the last porkchopCompute() result, for the
       window to render. The grid size is g.args.porkchop_n (the size knob;
       a Settings-window hook later). */
    PorkchopResult pc;   // valid when pc.valid

private:
    Game &g;
};
