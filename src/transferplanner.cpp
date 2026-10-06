// transferplanner.cpp -- game-side transfer planner (declared in
// transferplanner.h). The pure-math solver is in transfer.h.
#include "transferplanner.h"
#include "game.h"   // the complete Game (transferplanner.h only forward-declares it)
#include "bodylimits.h"  // shellEdge (the capture-orbit radius)

#include <cmath>
#include <numbers>
#include <cstdio>
#include <functional>
#include <memory>

namespace {
// Ship/target state lifted into the parent's INERTIAL frame -- the frame the
// transfer conic lives in.
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
        d.v = tf->GetVelocityRelTo(inertial);
        d.mu = t.body->mu;
        // Capture "orbit" at the near-body shell edge (bodylimits.h): the
        // LowOrbit/HighOrbit boundary -- above the air on every body (the
        // old flat radius+100 km sat inside the atmosphere of Jool and the
        // RSS giants) and on the sea-level datum.
        d.r_cap = t.body->radius + t.body->surface.sea_level
                + shellEdge(t.body->surface.atmosphere.top());
        if(tf->orb_ang_speed > 0.0) {
            // 3 target periods covers the min-dv point with margin.
            d.tof_max = 3.0 * (2.0 * std::numbers::pi / tf->orb_ang_speed);
        }
        d.capture = true;
    } else {
        // Ship in the same body: transform its state to our inertial frame.
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
    xferTargets.clear();
    if(g.ship == nullptr) {
        // No active ship: drop any stale target / plan (orbit-view boot).
        xfer_target = -1;
        xfer.valid = false;
        xfer.burn_dir = glm::dvec3(0.0);
        return;
    }
    {
        TerrainBody *pb = g.ship->frame->body;
        for(auto *b : g.sys.bodies) {
            if(b->frame && b->frame->parent == pb->frame) {
                xferTargets.push_back({b->name.c_str(), b, nullptr});
            }
        }
        // Sibling ships (free ships + EVA; not aboard crew).
        for(auto *s : pb->ships) {
            if(s != g.ship) {
                xferTargets.push_back({s->name.c_str(), nullptr, s});
            }
        }
        // --transfer-target wins over the window's combo on every rebuild.
        if(!g.args.transfer_target.empty()) {
            for(int i = 0; i < (int)xferTargets.size(); i++) {
                if(xferTargets[i].name == g.args.transfer_target) {
                    xfer_target = i;
                }
            }
        }
    }
    if(xfer_target >= (int)xferTargets.size()) { xfer_target = -1; }

    // Drop a grid / plan swept for a different target (stale label).
    if(pc.valid && pc_target != xfer_target) {
        pc.valid = false;
    }

    if(xfer_from_porkchop && xfer_plan_target != xfer_target) {
        clearPorkchopPlan();
    }

    if(xfer_target < 0) {
        xfer.valid = false;
        xfer.burn_dir = glm::dvec3(0.0);
        // Forget the solved target too: xfer.frame only advances while a
        // target is set, so without this, re-picking the SAME index clears
        // no dirty term and leaves the solution blank for up to 30 frames.
        xfer.target = -2;
    } else {
        xfer.frame++;
        const bool dirty = xfer.target != xfer_target
            || xfer.auto_tof != xfer_auto
            || std::fabs(xfer.tof_log - xfer_tof_log) > 1e-12
            || xfer.frame - xfer.solved_frame >= 30;
        if(dirty) {
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
                // A dv delta carries no stasis term: it cancels in the difference.
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

    // --xfer-log: the planner's current solution.
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

void TransferPlanner::invalidateClockState() {
    // Clock jump or system swap: everything stamped against the old world is
    // stale. Clearing xferTargets also frees the old system's pointers.
    clearPorkchopPlan();
    pc.valid = false;
    xfer.valid = false;
    xferTargets.clear();
    xfer_target = -1;
}

void TransferPlanner::porkchopCompute() {
    if(xfer_target < 0 || xfer_target >= (int)xferTargets.size()) { return; }
    const XferTarget &t = xferTargets[xfer_target];

    // Snapshot ship/target state at t = 0 in the parent's INERTIAL frame.
    // This is the ONLY part that reads game state; the grid sweep is pure
    // and runs on the background worker.
    const InertialShip s1 = shipInertial(g, g.view.pos, g.view.vel);
    const InertialTarget d = targetInertial(t, s1.inertial);

    // Windows: auto range unless the axis checkbox is on (then slider values).
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

    // Snapshot pure values only (no game refs) so the worker is safe to run.
    const glm::dvec3 r1 = s1.r, v1 = s1.v, r2 = d.r, v2 = d.v;
    const double mu_p = s1.mu_parent, mu_t = d.mu, r_cap = d.r_cap;
    const int n = g.args.porkchop_n;
    const bool capture = d.capture;
    const bool log = g.args.porkchop_log;
    const std::string tname = t.name;
    const double t_now = g.time;
    const int target_idx = xfer_target;
    const int epoch = g.cache_epoch;      // drop the result if the world changes

    pc_in_flight++;
    g.jobs.post("Porkchop grid", [r1,v1,r2,v2,mu_p,mu_t,r_cap,
                                  t_dep_lo,t_dep_hi,tof_lo,tof_hi,
                                  n,capture,log,tname,t_now,target_idx,epoch,this]()
                -> std::function<void()> {
        // Worker: PURE (no game state, GL, or imgui). shared_ptr because the
        // std::function continuation capture must be copyable, not moved.
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
        // Main-thread continuation: publish the result, clear "sweeping".
        return [this, res, t_now, tname, target_idx, epoch]() {
            // Epoch bumped after we posted: the grid is for the old world
            // (load does NOT abort jobs). Drop it before touching the target
            // list -- a SHIP target list was freed by the load (UAF risk).
            if(epoch != this->g.cache_epoch) {
                if(pc_in_flight > 0) { pc_in_flight--; }
                return;
            }
            // Publish only if the target is still the one this grid was for.
            const bool still_target = (xfer_target >= 0
                && xfer_target < (int)xferTargets.size()
                && xferTargets[xfer_target].name == tname);
            if(still_target) {
                pc = std::move(*res);
                pc_computed_at = t_now;
                pc_target = target_idx;
            }
            if(pc_in_flight > 0) { pc_in_flight--; }
        };
    });
}
