// transferplanner.cpp -- game-side transfer planner (declared in
// transferplanner.h). The pure-math solver is in transfer.h.
#include "transferplanner.h"
#include "game.h"   // the complete Game (transferplanner.h only forward-declares it)
#include "bodylimits.h"  // shellEdge (the capture-orbit radius)

#include <algorithm>
#include <cmath>
#include <numbers>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>

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

// One [porkchop] line per swept grid (the shape e2e/run.py parses).
void logPorkchop(double t_now, const std::string &target,
                 const PorkchopResult &res, double sweep_ms) {
    if(res.valid) {
        printf("[porkchop] t=%.1fs target=\"%s\" %dx%d dv_min=%.6g m/s "
               "dv_hi=%.6g m/s t_dep_min=%.6g s tof_min=%.6g s sweep=%.1f ms\n",
               t_now, target.c_str(), res.n_dep, res.n_tof,
               res.dv_min, res.dv_hi, res.t_dep_min, res.tof_min, sweep_ms);
    } else {
        printf("[porkchop] t=%.1fs target=\"%s\" no-solution sweep=%.1f ms\n",
               t_now, target.c_str(), sweep_ms);
    }
    fflush(stdout);
}

/* The grid as CSV for offline analysis: a comment header carrying the grid
   shape, the axis ranges and the argmin, then one row per ToF sample (the
   ImPlot ToF-major layout), NaN where no conic solved. The sim time is in the
   filename because re-sweeping is the whole point of --porkchop-bench, and a
   fixed name would destroy the previous dump. */
void dumpPorkchop(const std::string &dir, const std::string &target,
                  double t_now, const PorkchopResult &res) {
    std::string safe;
    for(const char c : target) {
        safe += (std::isalnum((unsigned char)c) || c == '-' || c == '_') ? c : '_';
    }
    char tb[32];
    std::snprintf(tb, sizeof tb, "t%.0f", t_now);
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    const std::string path = dir + "/porkchop_" + safe + "_" + tb + "_"
        + std::to_string(res.n_dep) + "x" + std::to_string(res.n_tof) + ".csv";
    std::ofstream f(path);
    if(!f) {
        // Deliberately NOT tagged "[porkchop]": an e2e EXPECT on that tag must
        // not be satisfied by a failure line.
        printf("[porkchop-dump] FAILED: %s\n", path.c_str());
        fflush(stdout);
        return;
    }
    f.precision(9);   // set BEFORE the header, or the ranges print at 6 digits
    f << "# n_dep,n_tof,t_dep_lo,t_dep_hi,tof_lo,tof_hi,dv_min,t_dep_min,tof_min (s; m/s)\n"
      << "# " << res.n_dep << "," << res.n_tof << ","
      << res.t_dep_lo << "," << res.t_dep_hi << "," << res.tof_lo << ","
      << res.tof_hi << "," << res.dv_min << "," << res.t_dep_min << ","
      << res.tof_min << "\n";
    for(int j = 0; j < res.n_tof; j++) {
        for(int i = 0; i < res.n_dep; i++) {
            if(i) { f << ','; }
            f << res.total_dv[(size_t)j * res.n_dep + i];
        }
        f << '\n';
    }
    f.close();
    if(!f) {
        // Truncated (disk full, EIO): say so rather than leave a quiet partial.
        printf("[porkchop-dump] TRUNCATED: %s\n", path.c_str());
        fflush(stdout);
    }
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
    // A sweep interrupted by the swap never runs its continuation (abort()
    // drops the queued ones, restart() drops the landed one), and that
    // continuation is the only other place pc_in_flight is cleared -- without
    // this the window shows "sweeping" and disables Compute forever.
    pc_in_flight = 0;
}

void TransferPlanner::porkchopCompute() {
    if(xfer_target < 0 || xfer_target >= (int)xferTargets.size()) { return; }
    // One sweep at a time. The window's button is disabled while a sweep is in
    // flight but the P key is not, and a --porkchop-bench list can queue
    // minutes of worker time that abort() would then have to wait out.
    if(pc_in_flight > 0) { return; }
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
    /* --porkchop-bench sweeps several sizes off THIS one snapshot, so the
       grids differ only in resolution (separate runs would also move the
       snapshot with the wall clock, since P fires on a frame boundary).
       Swept ascending; the live plot keeps the largest. */
    std::vector<int> sizes = g.args.porkchop_bench;
    if(sizes.empty()) { sizes.push_back(g.args.porkchop_n); }
    // A repeated size would redo identical work and rewrite the same CSV.
    std::sort(sizes.begin(), sizes.end());
    sizes.erase(std::unique(sizes.begin(), sizes.end()), sizes.end());
    const bool dump = !g.args.porkchop_dump.empty();
    // A bench sweep exists to be read, so it implies --porkchop-log.
    const bool log = g.args.porkchop_log || dump || !g.args.porkchop_bench.empty();
    const std::string dump_dir = g.args.porkchop_dump;
    const bool capture = d.capture;
    const std::string tname = t.name;
    const double t_now = g.time;
    const int epoch = g.cache_epoch;      // drop the result if the world changes
    JobRunner *jobs = &g.jobs;           // stopping() is thread-safe (job.h)

    pc_in_flight++;
    g.jobs.post("Porkchop grid", [r1,v1,r2,v2,mu_p,mu_t,r_cap,
                                  t_dep_lo,t_dep_hi,tof_lo,tof_hi,
                                  sizes,capture,log,tname,t_now,
                                  epoch,dump,dump_dir,jobs,this]()
                -> std::function<void()> {
        // Worker: PURE (no game state, GL, or imgui). shared_ptr because the
        // std::function continuation capture must be copyable, not moved.
        std::shared_ptr<PorkchopResult> res;
        for(const int n : sizes) {
            // abort()/join() (load, system swap, exit) drops QUEUED jobs but
            // still joins the IN-FLIGHT body, so a multi-grid sweep must notice
            // the stop between grids or it holds the main thread for its whole
            // remaining runtime.
            if(jobs->stopping()) { break; }
            res.reset();   // the previous grid is done with; don't peak at 2
            const auto t0 = std::chrono::steady_clock::now();
            try {
                res = std::make_shared<PorkchopResult>(porkchopGrid(
                    r1,v1,r2,v2,mu_p,mu_t,r_cap,
                    t_dep_lo,t_dep_hi,tof_lo,tof_hi,
                    n,n,capture));
            } catch(const std::exception &e) {
                // Swallowed by JobRunner, which then skips the continuation --
                // and the continuation is what clears pc_in_flight. Catching
                // here keeps the "sweeping" indicator from wedging the window.
                printf("[porkchop] sweep FAILED (%dx%d): %s\n", n, n, e.what());
                fflush(stdout);
                break;
            }
            const double ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
            if(log) { logPorkchop(t_now, tname, *res, ms); }
            if(dump) { dumpPorkchop(dump_dir, tname, t_now, *res); }
        }
        // Main-thread continuation: publish the result, clear "sweeping".
        return [this, res, t_now, tname, epoch]() {
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
            if(still_target && res) {
                pc = std::move(*res);
                pc_computed_at = t_now;
                pc_rev++;   // the heatmap texture is a cache of pc; this is its
                           // invalidation (see pc_rev)
                // The index just validated, not the one captured at post: the
                // target list is rebuilt every frame, so a sibling ship joining
                // or leaving shifts it, and a pc_target that disagrees with
                // xfer_target makes update() drop this grid on the next frame.
                pc_target = xfer_target;
            }
            if(pc_in_flight > 0) { pc_in_flight--; }
        };
    });
}
