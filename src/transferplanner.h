// transferplanner.h -- the game-side transfer planner: the TARGET window's
// target selection + the min-dv solver cache. Pure math is in transfer.h.
#pragma once

#include <cmath>
#include <string>
#include <vector>

#include <SDL3/SDL.h>
#include <glm/glm.hpp>

#include "transfer.h"   // TransferSolution, PorkchopResult

// Planner is a MEMBER of Game; forward-declare to avoid a cycle.
struct Game;
struct TerrainBody;
struct Frame;
class Vehicle;

/* A target the player holds on to, by identity rather than by row: xferTargets
   is rebuilt from scratch every frame, so a row number is only this frame's
   address -- a fleet change silently retargets the window, the swept grid and an
   armed plan (issue #276). The kind is part of the identity because a vessel may
   be named after a body, and the list shows both. */
struct TargetRef {
    std::string name;
    bool isShip = false;
    bool empty() const { return name.empty(); }
    bool operator==(const TargetRef &o) const { return name == o.name && isShip == o.isShip; }
    bool operator!=(const TargetRef &o) const { return !(*this == o); }
};

class TransferPlanner {
public:
    struct XferTarget {
        const char *name;
        TerrainBody *body;   // body target (capture available)
        Vehicle *ship;       // ship target (intercept only)
    };

    explicit TransferPlanner(Game &g) : g(g) {}

    // Per-frame: rebuild the target list, recompute on input change or every
    // 30 frames, fire the --xfer-log.
    void update(const glm::dvec3 &com, const glm::dvec3 &vel);

    // Post the grid sweep to the background worker; never blocks the frame.
    void porkchopCompute();

    /* The TARGET window's "Compute transfer". Off, NOTHING solves: the target
       stays selected (its orbit is on the map, and its plane is one of the
       map's planes) but the burn marker, the map arc, the solution block and
       --xfer-log all go quiet, and no Lambert run happens. A porkchop grid is
       still an explicit request in its own window, so it sweeps regardless;
       "Send best" turns planning back on because the plan needs it. */
    bool xfer_compute = true;

    /* Unit normal of the selected target's orbit about the body the ship
       orbits, in that body's INERTIAL frame -- the frame the orbit map and the
       ship's own conic live in, so it feeds uiRefPlane directly. False (n_hat
       untouched) with no target, or one that is not in orbit: a grounded vessel
       traces a surface circle, not an orbit, and passing its plane off as the
       target's would be a lie. */
    bool targetPlaneNormal(Frame *focus_inertial, glm::dvec3 &n_hat) const;

    // Drop a "Send best" plan and restore the prior ToF mode. No-op if none.
    void clearPorkchopPlan();

    // Clock jump or system swap: drop every stamp against the old world.
    void invalidateClockState();

    /* Select by IDENTITY ("" clears), resolving against the current list so the
       caller sees this frame's index. `isShip` is part of the identity: the list
       can hold a body and a vessel under one name, and the combo shows a row for
       each. */
    void selectTarget(const std::string &name, bool isShip);

    /* Select by name alone, taking the LAST match -- a vessel wins over a body
       of the same name. This is the CLI's shape, where there is no row to click. */
    void selectTargetByName(const std::string &name);

    std::vector<XferTarget> xferTargets;
    /* The selection is an IDENTITY, and xfer_target is derived from it on every
       rebuild (update). xferTargets is rebuilt from scratch each refresh, so an
       index alone silently re-points at another vessel when a sibling docks,
       undocks, is removed or boards -- the map's transfer plane, the burn
       marker and the TARGET block all follow it with no player action. A name
       is only stable while names are, but dedupVehicleName's " #N" suffix is a
       rename the player caused, not a reshuffle they did not. */
    TargetRef xfer_sel;
    int xfer_target = -1;
    bool xfer_auto = true;                     // auto min-dv ToF vs pinned
    float xfer_tof_log = (float)std::log10(3600.0); // log10(s), the pinned ToF
    struct {
        int target = -2;        // target index at last compute (the solve's
                                // owner: xfer.valid can outlive xfer_target
                                // by a frame, so pair the solution with THIS
                                // index, never the live one)
        bool auto_tof = true;
        double tof_log = -1.0;
        int frame = 0;          // per-frame counter while a target is set
        int solved_frame = -1000000; // xfer.frame at last recompute
        /* xfer.frame at the last pc_period lift. pc_period describes the TARGET
           (the porkchop range sliders scale by it), not a plan, so it keeps
           refreshing while planning is off -- on the same 30-frame cadence a
           solve uses, rather than a state lift every frame. */
        int period_frame = -1000000;
        bool valid = false;
        TransferSolution sol;
        glm::dvec3 burn_dir = glm::dvec3(0.0); // render-frame burn direction
    } xfer;

    // Last porkchopCompute() result.
    PorkchopResult pc;   // valid when pc.valid
    /* The identity the current grid was swept for -- an IDENTITY, not the index
       it happened to hold: the index moves when the list reshuffles, and treating
       that as a target change would throw away a grid for the same vessel. It has
       to be as exact as the selection itself, or a deleted vessel hands its grid
       to a body of the same name. */
    TargetRef pc_target;
    // P: one-shot compute request (events.cpp sets, the state pass consumes).
    bool pc_compute_requested = false;
    int pc_in_flight = 0; // jobs posted but not landed (window shows "sweeping")
    // Bumped whenever pc is replaced (only writer: the sweep's continuation).
    // The window redraws its heatmap off this, so a grid merely still on screen
    // costs nothing per frame. Covers CONTENTS only: target change / clock jump
    // invalidate pc but leave rev alone, safe because the window returns before
    // its texture block when !pc.valid. NOT pc_computed_at: that is g.time at
    // POST time, so sweeps posted in the same sim instant share a stamp and the
    // second grid never reaches the screen.
    int pc_rev = 0;
    // Custom dep/ToF ranges; off = auto range (see Porkchop window). Seconds.
    bool   pcCustomDep = false;
    float  pcDepLo = 0.0f;
    float  pcDepHi = 0.0f;
    bool   pcCustomTof = false;
    float  pcTofLo = 60.0f;
    float  pcTofHi = 0.0f;
    // The current target's own period about the shared parent (s), 0 if it is
    // not on a closed orbit. The porkchop range sliders scale by this: the
    // interesting window for a planet is months, for a co-orbital phasing leg
    // longer still, and no fixed ceiling fits both.
    double pc_period = 0.0;
    /* The ToF ceiling the UI offers for the current target: nine target periods,
       never below the week the range sliders used to stop at. The auto sweep and
       a default grid search three of them (tof_max), and the porkchop range
       sliders let a grid go to nine, so nine covers every cell a plan can be
       armed from. One definition because the Target window's manual ToF slider and
       the porkchop range sliders must not disagree (issue #255). */
    double xfer_tof_ceiling() const {
        const double period = pc_period > 0.0 ? pc_period : 86400.0;
        const double nine = 9.0 * period;
        return nine > 604800.0 ? nine : 604800.0;
    }
    double pc_computed_at = 0.0;
    // "Send best": absolute departure time to count down to. Dropped on target change.
    bool   xfer_from_porkchop = false;
    double xfer_t_dep = 0.0;   // s (sim clock)
    // Which target the armed plan is for -- see pc_target.
    TargetRef xfer_plan_target;
    // Saved before "Send best" pinned the ToF; clearPorkchopPlan restores it.
    bool   xfer_prev_auto = true;
    float  xfer_prev_tof_log = (float)std::log10(3600.0);

private:
    Game &g;

    /* Point xfer_target at xfer_sel in the CURRENT xferTargets, or clear the
       selection when it is not there: a target that left the fleet takes it with
       it, and keeping the index would name whichever vessel landed in that slot.
       Called on every rebuild. */
    void resolveTarget();
};
