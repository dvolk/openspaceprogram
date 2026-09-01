// transferplanner.cpp -- the game-side transfer planner (declared in
// transferplanner.h).
//
// This was the "Transfer planner" block of main's render pass (the
// per-frame target-list rebuild, the min-dv solve and the --xfer-log) plus
// the planner's state locals (xferTargets / xfer_target / xfer_auto /
// xfer_tof_log / xfer / xfer_log_last_ms). It moved out as-is: main's
// locals became the planner's members, and the remaining globals (ship,
// ships, bodies, args, the sim clock, the log interval) come from the
// borrowed Game. The pure-math solver (planTransfer) is in transfer.h.
#include "transferplanner.h"

#include <cmath>
#include <cstdio>
#include <functional>   // std::function (the job's main-thread continuation)
#include <memory>       // shared_ptr (the cross-thread result handoff)

namespace {
/* The frame-independent transforms update() and porkchopCompute() both
   need: the ship's state and the target's state, lifted from their own
   (possibly rotating) frames into the parent's INERTIAL frame -- the frame
   the transfer conic lives in (the same idiom as the ORBITAL readout). */
struct InertialShip {
    Frame *inertial = nullptr;
    glm::dvec3 r = glm::dvec3(0.0), v = glm::dvec3(0.0);
    double mu_parent = 0.0;
};
InertialShip shipInertial(Game &g, const glm::dvec3 &com, const glm::dvec3 &vel) {
    Frame *sf = g.ship->frame;
    Frame *inertial = sf->getNonRotFrame();
    InertialShip s;
    s.inertial = inertial;
    s.r = sf->GetOrientRelTo(inertial) * com + sf->GetPositionRelTo(inertial);
    s.v = sf->GetOrientRelTo(inertial) * (vel + sf->GetStasisVelocity(com))
        + sf->GetVelocityRelTo(inertial);
    s.mu_parent = inertial->body->mu;
    return s;
}

struct InertialTarget {
    glm::dvec3 r = glm::dvec3(0.0), v = glm::dvec3(0.0);
    double mu = 0.0;
    double r_cap = 0.0;
    double tof_max = 3.0 * 86400.0;
    bool capture = false;
};
InertialTarget targetInertial(const TransferPlanner::XferTarget &t,
                              Frame *inertial) {
    InertialTarget d;
    if(t.body) {
        Frame *tf = t.body->frame;
        d.r = tf->GetPositionRelTo(inertial);
        // The body's orbital velocity in the parent frame, straight from the
        // frame tree (nonzero now that the rails are Kepler orbits; for the
        // current circular data this equals the old omega x r construction).
        d.v = tf->GetVelocityRelTo(inertial);
        d.mu = t.body->mu;
        d.r_cap = t.body->radius + 100e3; // 100 km capture orbit
        if(tf->orb_ang_speed > 0.0) {
            // 3 full target periods covers the min-dv point (near the
            // Hohmann ToF) with margin on both sides.
            d.tof_max = 3.0 * (2.0 * M_PI / tf->orb_ang_speed);
        }
        d.capture = true;
    } else {
        // Ship in the same body: transform its state over to our inertial
        // frame (stasis of the non-rotating frame is zero).
        Frame *tsf = t.ship->frame;
        const glm::dvec3 tcom = t.ship->get_center_of_mass();
        const glm::dmat3 O = tsf->GetOrientRelTo(inertial);
        d.r = O * tcom + tsf->GetPositionRelTo(inertial);
        d.v = O * (t.ship->GetVel() + tsf->GetStasisVelocity(tcom))
            + tsf->GetVelocityRelTo(inertial);
    }
    return d;
}
} // namespace

void TransferPlanner::update(const glm::dvec3 &com, const glm::dvec3 &vel) {
    /* Rebuild the target list, then recompute the solution on input
       change or every 30 frames. */
    xferTargets.clear();
    {
        TerrainBody *pb = g.ship->frame->body;
        for(auto *b : g.sys.bodies) {
            if(b->frame && b->frame->parent == pb->frame) {
                xferTargets.push_back({b->name.c_str(), b, nullptr});
            }
        }
        for(auto *s : g.ships) {
            if(s != g.ship && s->frame && s->frame->body == pb) {
                xferTargets.push_back({s->name.c_str(), nullptr, s});
            }
        }
        // --transfer-target: explicit selection (e2e / scripting);
        // wins over the window's combo on every rebuild.
        if(!g.args.transfer_target.empty()) {
            for(int i = 0; i < (int)xferTargets.size(); i++) {
                if(xferTargets[i].name == g.args.transfer_target) {
                    xfer_target = i;
                }
            }
        }
    }
    if(xfer_target >= (int)xferTargets.size()) { xfer_target = -1; }

    // A porkchop grid is only valid for the target it was swept for: drop it
    // on a target change so the old target's launch window never shows under
    // the new target's label (the "Send best" plan is dropped just below, for
    // the same reason). Idempotent once pc is cleared.
    if(pc.valid && pc_target != xfer_target) {
        pc.valid = false;
    }

    // A "Send best" plan is for the target it was sent for. Drop it if the
    // target changed so a stale countdown doesn't linger on the new target
    // (and restore the ToF mode the user had before sending).
    if(xfer_from_porkchop && xfer_plan_target != xfer_target) {
        clearPorkchopPlan();
    }

    if(xfer_target < 0) {
        xfer.valid = false;
        xfer.burn_dir = glm::dvec3(0.0);
    } else {
        xfer.frame++;
        const bool dirty = xfer.target != xfer_target
            || xfer.auto_tof != xfer_auto
            || std::fabs(xfer.tof_log - xfer_tof_log) > 1e-12
            || xfer.frame - xfer.solved_frame >= 30;
        if(dirty) {
            // Ship + target state in the parent's INERTIAL frame (the
            // shared transform helpers above).
            const InertialShip s1 = shipInertial(g, com, vel);
            const XferTarget &t = xferTargets[xfer_target];
            const InertialTarget d = targetInertial(t, s1.inertial);

            TransferSolution sol;
            if(xfer_auto) {
                sol = planTransfer(s1.r, s1.v, d.r, d.v, s1.mu_parent,
                                   d.mu, d.r_cap, 60.0, d.tof_max, 150,
                                   d.capture);
            } else {
                const double tof = std::pow(10.0, xfer_tof_log);
                sol = planTransfer(s1.r, s1.v, d.r, d.v, s1.mu_parent,
                                   d.mu, d.r_cap, tof, tof, 1, d.capture);
            }

            xfer.sol = sol;
            xfer.valid = sol.valid;
            if(sol.valid) {
                // Burn direction at the ship, in the render frame
                // (ship->frame). A dv delta carries no stasis term:
                // the same stasis applies before and after the burn
                // at the same position, so it cancels in the
                // difference.
                const glm::dmat3 O = g.ship->frame->GetOrientRelTo(s1.inertial);
                xfer.burn_dir = glm::transpose(O) * (sol.v_departure - s1.v);
            } else {
                xfer.burn_dir = glm::dvec3(0.0);
            }
            xfer.target = xfer_target;
            xfer.auto_tof = xfer_auto;
            xfer.tof_log = xfer_tof_log;
            xfer.solved_frame = xfer.frame;
        }
    }

    // --xfer-log: the planner's current solution (render pass, since
    // that is where the computation lives).
    if(g.args.xfer_log && xfer_target >= 0) {
        const Uint32 now_ms = SDL_GetTicks();
        if(now_ms - xfer_log_last_ms >= g.orbit_log_interval_ms) {
            xfer_log_last_ms = now_ms;
            const char *tn = xferTargets[xfer_target].name;
            if(xfer.valid) {
                printf("[xferlog] t=%.1fs target=\"%s\" dv_dep=%.6g m/s "
                       "dv_cap=%.6g m/s total=%.6g m/s tof=%.6g s "
                       "v_inf=%.6g m/s r_cap=%.6g m "
                       "burn=[%.4f %.4f %.4f]\n",
                       g.time, tn, xfer.sol.dv_departure,
                       xfer.sol.dv_capture, xfer.sol.total_dv,
                       xfer.sol.tof, xfer.sol.v_inf, xfer.sol.r_cap,
                       xfer.burn_dir.x, xfer.burn_dir.y,
                       xfer.burn_dir.z);
            } else {
                printf("[xferlog] t=%.1fs target=\"%s\" no-solution\n",
                       g.time, tn);
            }
            fflush(stdout);
        }
    }
}

void TransferPlanner::clearPorkchopPlan() {
    if(!xfer_from_porkchop) { return; }
    xfer_from_porkchop = false;
    xfer_t_dep = 0.0;
    xfer_plan_target = -1;
    xfer_auto = xfer_prev_auto;
    xfer_tof_log = xfer_prev_tof_log;
}

void TransferPlanner::porkchopCompute() {
    if(xfer_target < 0 || xfer_target >= (int)xferTargets.size()) { return; }
    const XferTarget &t = xferTargets[xfer_target];

    // The ship's + the target's state at t = 0 (now) in the parent's
    // INERTIAL frame (the shared transform helpers). The ship's state is
    // this render pass's snapshot (g.view), so the grid matches what the
    // readouts show. This snapshot step is the ONLY part that reads game
    // state; the grid sweep itself is pure, so it runs on the background
    // worker (g.jobs) and the frame stays responsive (see job.h).
    const InertialShip s1 = shipInertial(g, g.view.pos, g.view.vel);
    const InertialTarget d = targetInertial(t, s1.inertial);

    // Windows. Each axis defaults to the auto range -- departure delay:
    // 0 .. one full target orbit (covers every relative phase), ToF: 60 s
    // .. three target periods (the same span update() sweeps) -- unless
    // the window's checkbox for that axis is on, in which case the slider
    // values (seconds) are used. porkchopGrid swaps a reversed pair.
    double t_dep_lo, t_dep_hi;
    if(pcCustomDep) {
        t_dep_lo = pcDepLo; t_dep_hi = pcDepHi;
    } else {
        t_dep_lo = 0.0; t_dep_hi = d.tof_max / 3.0;
    }
    double tof_lo, tof_hi;
    if(pcCustomTof) {
        tof_lo = pcTofLo; tof_hi = pcTofHi;
    } else {
        tof_lo = 60.0; tof_hi = d.tof_max;
    }

    // Snapshot everything the worker needs (pure values, no game refs) so
    // the off-thread body stays safe to run. The old grid (pc) stays on
    // screen while this runs; it is replaced when the job lands.
    const glm::dvec3 r1 = s1.r, v1 = s1.v, r2 = d.r, v2 = d.v;
    const double mu_p = s1.mu_parent, mu_t = d.mu, r_cap = d.r_cap;
    const int n = g.args.porkchop_n;
    const bool capture = d.capture;
    const bool log = g.args.porkchop_log;
    const std::string tname = t.name;
    const double t_now = g.time;
    const int target_idx = xfer_target;   // the grid is for this target

    pc_in_flight++;   // the window's "sweeping..." state (main thread)
    g.jobs.post("Porkchop grid", [r1,v1,r2,v2,mu_p,mu_t,r_cap,
                                  t_dep_lo,t_dep_hi,tof_lo,tof_hi,
                                  n,capture,log,tname,t_now,target_idx,this]()
                -> std::function<void()> {
        // Worker thread: PURE. Sweep the grid (porkchopGrid is header-only
        // math) and fire the log; no game state, GL or imgui is touched
        // here. The result is handed to the main thread through the returned
        // continuation; a shared_ptr lets it outlive this body (a C++11-safe
        // way to move a large result across the thread handoff -- lambda
        // capture initializers are C++14).
        std::shared_ptr<PorkchopResult> res =
            std::make_shared<PorkchopResult>(porkchopGrid(
                r1,v1,r2,v2,mu_p,mu_t,r_cap,
                t_dep_lo,t_dep_hi,tof_lo,tof_hi,
                n,n,capture));
        if(log) {
            if(res->valid) {
                printf("[porkchop] t=%.1fs target=\"%s\" %dx%d dv_min=%.6g m/s "
                       "dv_hi=%.6g m/s t_dep_min=%.6g s tof_min=%.6g s\n",
                       t_now, tname.c_str(), res->n_dep, res->n_tof,
                       res->dv_min, res->dv_hi, res->t_dep_min, res->tof_min);
            } else {
                printf("[porkchop] t=%.1fs target=\"%s\" no-solution\n",
                       t_now, tname.c_str());
            }
            fflush(stdout);
        }
        // Main-thread continuation (JobRunner::poll): publish the result +
        // clear the "sweeping" state. Runs on the main thread, so writing
        // the planner's state is safe. pc_computed_at = t_now so a "Send
        // best" departure = t_now + the best cell's delay.
        return [this, res, t_now, tname, target_idx]() {
            // Only publish if the target is still the one this grid was
            // swept for: a target switch mid-flight would otherwise leave a
            // grid for the OLD target showing under the new target's label.
            const bool still_target = (xfer_target >= 0
                && xfer_target < (int)xferTargets.size()
                && xferTargets[xfer_target].name == tname);
            if(still_target) {
                pc = std::move(*res);
                pc_computed_at = t_now;
                pc_target = target_idx;   // remember whose grid this is
            }
            if(pc_in_flight > 0) { pc_in_flight--; }
        };
    });
}
