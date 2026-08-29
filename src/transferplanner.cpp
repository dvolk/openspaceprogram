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
            // Ship state in the parent's INERTIAL frame — the frame
            // the transfer conic lives in (same idiom as the ORBITAL
            // readout).
            Frame *sf = g.ship->frame;
            Frame *inertial = sf->getNonRotFrame();
            const glm::dvec3 r1 = sf->GetOrientRelTo(inertial) * com
                                 + sf->GetPositionRelTo(inertial);
            const glm::dvec3 v1 = sf->GetOrientRelTo(inertial)
                                 * (vel + sf->GetStasisVelocity(com))
                                 + sf->GetVelocityRelTo(inertial);
            const double mu_parent = inertial->body->mu;

            // Target state in the same frame.
            const XferTarget &t = xferTargets[xfer_target];
            glm::dvec3 r2, v2;
            double mu_target = 0.0, r_cap = 0.0;
            double tof_max = 3.0 * 86400.0;
            if(t.body) {
                Frame *tf = t.body->frame;
                r2 = tf->GetPositionRelTo(inertial);
                // The body's orbital velocity in the parent frame,
                // straight from the frame tree (nonzero now that the
                // rails are Kepler orbits; for the current circular
                // data this equals the old omega x r construction).
                v2 = tf->GetVelocityRelTo(inertial);
                mu_target = t.body->mu;
                r_cap = t.body->radius + 100e3; // 100 km capture orbit
                if(tf->orb_ang_speed > 0.0) {
                    // 3 full target periods covers the min-dv point
                    // (near the Hohmann ToF) with margin on both sides.
                    tof_max = 3.0 * (2.0 * M_PI / tf->orb_ang_speed);
                }
            } else {
                // Ship in the same body: transform its state over to
                // our inertial frame (stasis of the non-rotating
                // frame is zero).
                Frame *tsf = t.ship->frame;
                const glm::dvec3 tcom = t.ship->get_center_of_mass();
                const glm::dmat3 O = tsf->GetOrientRelTo(inertial);
                r2 = O * tcom + tsf->GetPositionRelTo(inertial);
                v2 = O * (t.ship->GetVel() + tsf->GetStasisVelocity(tcom))
                    + tsf->GetVelocityRelTo(inertial);
            }

            const bool capture = (t.body != nullptr);
            TransferSolution sol;
            if(xfer_auto) {
                sol = planTransfer(r1, v1, r2, v2, mu_parent,
                                   mu_target, r_cap,
                                   60.0, tof_max, 150, capture);
            } else {
                const double tof = std::pow(10.0, xfer_tof_log);
                sol = planTransfer(r1, v1, r2, v2, mu_parent,
                                   mu_target, r_cap,
                                   tof, tof, 1, capture);
            }

            xfer.sol = sol;
            xfer.valid = sol.valid;
            if(sol.valid) {
                // Burn direction at the ship, in the render frame
                // (ship->frame). A dv delta carries no stasis term:
                // the same stasis applies before and after the burn
                // at the same position, so it cancels in the
                // difference.
                const glm::dmat3 O = sf->GetOrientRelTo(inertial);
                xfer.burn_dir = glm::transpose(O) * (sol.v_departure - v1);
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
