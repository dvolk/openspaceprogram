// transferplanner.h -- the game-side transfer planner: the TRANSFER window's
// target selection + the min-dv solver cache. Pure math is in transfer.h.
#pragma once

#include <cmath>
#include <vector>

#include <SDL3/SDL.h>
#include <glm/glm.hpp>

#include "transfer.h"   // TransferSolution, PorkchopResult

// Planner is a MEMBER of Game; forward-declare to avoid a cycle.
struct Game;
struct TerrainBody;
class Vehicle;

class TransferPlanner {
public:
    struct XferTarget {
        const char *name;
        TerrainBody *body;   // body target (capture available)
        Vehicle *ship;       // ship target (intercept only)
    };

    explicit TransferPlanner(Game &g) : g(g) {}

    // Per-frame: rebuild the target list, recompute the solution on input
    // change or every 30 frames, fire the --xfer-log.
    void update(const glm::dvec3 &com, const glm::dvec3 &vel);

    // On-demand (P key / button): snapshot inputs and post the grid sweep
    // to the background worker; never blocks the frame.
    void porkchopCompute();

    // Drop a "Send best" plan and restore the prior ToF mode. No-op if none.
    void clearPorkchopPlan();

    // Clock jump or system swap: drop every stamp against the old world.
    void invalidateClockState();

    std::vector<XferTarget> xferTargets;
    int xfer_target = -1;
    bool xfer_auto = true;                     // auto min-dv ToF vs pinned
    float xfer_tof_log = (float)std::log10(3600.0); // log10(s), the pinned ToF
    struct {
        int target = -2;        // target index at last compute (the solve's
                                // owner: xfer.valid can outlive xfer_target
                                // by a frame -- UI passes edit xfer_target
                                // between update()s, so pair the solution
                                // with THIS index, never the live one)
        bool auto_tof = true;
        double tof_log = -1.0;
        int frame = 0;          // per-frame counter while a target is set
        int solved_frame = -1000000; // xfer.frame at last recompute
        bool valid = false;
        TransferSolution sol;
        glm::dvec3 burn_dir = glm::dvec3(0.0); // render-frame burn direction
    } xfer;
    Uint32 xfer_log_last_ms = 0;

    // Last porkchopCompute() result. pc_in_flight counts jobs posted but not
    // yet landed (the window shows "sweeping"). pc_target is the target the
    // grid was swept for; a target change invalidates pc.
    PorkchopResult pc;   // valid when pc.valid
    int pc_target = -1;  // target index the current pc grid was swept for
    int pc_in_flight = 0;
    // Bumped whenever pc is replaced (the ONLY writer is the sweep's
    // continuation). The Porkchop window redraws its heatmap off this, so a
    // grid that is merely still on screen costs nothing per frame. Covers the
    // grid's CONTENTS only: the sites that merely invalidate pc (target change,
    // clock jump) leave rev alone, which is safe because the window returns
    // before its texture block when !pc.valid. NOT pc_computed_at: that is
    // g.time at POST time, so two sweeps posted in the same sim instant (clock
    // paused, headless) share a stamp and the second grid would never reach the
    // screen -- and it already means "the departure epoch of this grid" for
    // "Send best".
    int pc_rev = 0;
    // Custom dep/ToF ranges; off = auto range (see Porkchop window).
    bool   pcCustomDep = false;
    float  pcDepLo = 0.0f;
    float  pcDepHi = 0.0f;
    bool   pcCustomTof = false;
    float  pcTofLo = 60.0f;
    float  pcTofHi = 0.0f;
    double pc_computed_at = 0.0;
    // "Send best": absolute departure time to count down to. Dropped on target change.
    bool   xfer_from_porkchop = false;
    double xfer_t_dep = 0.0;   // s (sim clock)
    int    xfer_plan_target = -1;
    // ToF mode saved before "Send best" pinned the ToF; clearPorkchopPlan restores it.
    bool   xfer_prev_auto = true;
    float  xfer_prev_tof_log = (float)std::log10(3600.0);

private:
    Game &g;
};
