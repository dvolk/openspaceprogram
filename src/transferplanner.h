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
    // Departure-delay (x-axis) range, in s. When pcCustomDep is off the
    // sweep uses the auto range (0 .. one target period); when on it uses
    // these slider values (see the Porkchop window's checkbox).
    bool   pcCustomDep = false;
    float  pcDepLo = 0.0f;
    float  pcDepHi = 0.0f;
    // Sim time (s) the last porkchop grid was swept (porkchopCompute). The
    // best cell's t_dep_min is a DELAY relative to that moment, so an
    // absolute departure time = pc_computed_at + pc.t_dep_min.
    double pc_computed_at = 0.0;
    // A porkchop "Send best" applied to this target: xfer_t_dep is the
    // absolute departure time to count down to (and the ToF is pinned to
    // the best cell's). At t_dep the live "depart now" solution IS the best
    // cell, so that is when you burn. xfer_plan_target is the target index
    // it was sent for; if the target changes, the countdown is dropped.
    bool   xfer_from_porkchop = false;
    double xfer_t_dep = 0.0;   // s (sim clock)
    int    xfer_plan_target = -1;

private:
    Game &g;
};
