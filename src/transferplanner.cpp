// transferplanner.cpp -- game-side transfer planner (declared in
// transferplanner.h). The pure-math solver is in transfer.h.
#include "transferplanner.h"
#include "game.h"   // complete Game (the header only forward-declares it)
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

// Period of the conic through (r, v) under mu (s); 0 if it is not closed.
// Works for a body or a ship, since both are Keplerian about the same parent.
double orbitalPeriod(const glm::dvec3 &r, const glm::dvec3 &v, double mu) {
    const double rl = glm::length(r);
    if(mu <= 0.0 || rl <= 0.0) { return 0.0; }
    const double energy = 0.5 * glm::dot(v, v) - mu / rl;
    if(energy >= 0.0) { return 0.0; }
    const double a = -mu / (2.0 * energy);
    return 2.0 * std::numbers::pi * std::sqrt(a * a * a / mu);
}
InertialTarget targetInertial(const TransferPlanner::XferTarget &t,
                              Frame *inertial) {
    InertialTarget d;
    if(t.body) {
        Frame *tf = t.body->frame;
        d.r = tf->GetPositionRelTo(inertial);
        d.v = tf->GetVelocityRelTo(inertial);
        d.mu = t.body->mu;
        // Capture "orbit" at the near-body shell edge (bodylimits.h): the
        // LowOrbit/HighOrbit boundary -- above the air on every body, and on
        // the sea-level datum.
        d.r_cap = t.body->radius + t.body->surface.sea_level
                + shellEdge(t.body->surface.atmosphere.top());
        if(tf->orb_ang_speed > 0.0) {
            // 3 target periods covers the min-dv point with margin.
            d.tof_max = 3.0 * (2.0 * std::numbers::pi / tf->orb_ang_speed);
        }
        d.capture = true;
    } else {
        Frame *tsf = t.ship->frame;
        const glm::dvec3 tcom = t.ship->get_center_of_mass();
        const glm::dmat3 O = tsf->GetOrientRelTo(inertial);
        d.r = O * tcom + tsf->GetPositionRelTo(inertial);
        d.v = O * (t.ship->GetVel() + tsf->GetStasisVelocity(tcom))
            + tsf->GetVelocityRelTo(inertial);
        // A ship has no orb_ang_speed to read, so scale the search by its own
        // conic. The 3-day default is nonsense for a 1-hour orbit (150 ToF
        // samples spread over days) and too tight for a co-orbital phasing
        // window, which is measured in target periods (#136).
        const double T = orbitalPeriod(d.r, d.v, inertial->body->mu);
        if(T > 0.0) { d.tof_max = 3.0 * T; }
    }
    return d;
}

// One [porkchop] line per swept grid (the shape e2e/run.py parses). The swept
// range travels with the result: dv_min means nothing without the window it
// was the best inside.
void logPorkchop(double t_now, const std::string &target,
                 const PorkchopResult &res, double sweep_ms) {
    if(res.valid) {
        printf("[porkchop] t=%.1fs target=\"%s\" %dx%d "
               "dep=[%.0f,%.0f]s tof=[%.0f,%.0f]s "
               "dv_min=%.6g m/s dv_hi=%.6g m/s t_dep_min=%.6g s tof_min=%.6g s "
               "sweep=%.1f ms\n",
               t_now, target.c_str(), res.n_dep, res.n_tof,
               res.t_dep_lo, res.t_dep_hi, res.tof_lo, res.tof_hi,
               res.dv_min, res.dv_hi, res.t_dep_min, res.tof_min, sweep_ms);
    } else {
        printf("[porkchop] t=%.1fs target=\"%s\" no-solution "
               "dep=[%.0f,%.0f]s tof=[%.0f,%.0f]s sweep=%.1f ms\n",
               t_now, target.c_str(), res.t_dep_lo, res.t_dep_hi,
               res.tof_lo, res.tof_hi, sweep_ms);
    }
    fflush(stdout);
}

/* The grid as CSV for offline analysis: header of grid shape, axis ranges and
   argmin, then one row per ToF sample (ImPlot's ToF-major layout), NaN where no
   conic solved. Sim time is in the filename so a re-sweep cannot clobber the
   previous dump. */
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
        // Truncated (disk full, EIO): say so, don't leave a quiet partial.
        printf("[porkchop-dump] TRUNCATED: %s\n", path.c_str());
        fflush(stdout);
    }
}
} // namespace

/* Point xfer_target at the (name, is_ship) selection in the CURRENT list. A
   selection that is no longer there means the target was deleted, and the
   selection goes with it -- keeping the index instead would have it name
   whichever vessel happened to land in that slot (issue #276). */
void TransferPlanner::resolveTarget() {
    xfer_target = -1;
    if(xfer_sel.empty()) { return; }
    for(int i = 0; i < (int)xferTargets.size(); i++) {
        const XferTarget &t = xferTargets[i];
        if(t.name != xfer_sel.name) { continue; }
        // A body and a vessel can share a name; the combo lists both, so the
        // row clicked is part of what the player chose.
        if((t.ship != nullptr) != xfer_sel.isShip) { continue; }
        xfer_target = i;
        return;
    }
    xfer_sel = TargetRef();
}

void TransferPlanner::selectTarget(const std::string &name, bool isShip) {
    xfer_sel = TargetRef{name, isShip};
    resolveTarget();
}

void TransferPlanner::selectTargetByName(const std::string &name) {
    // Last name match wins, as --transfer-target always resolved it: the list
    // is bodies then vessels, so a vessel named after a body is the one a test
    // naming that string means to aim at.
    for(int i = 0; i < (int)xferTargets.size(); i++) {
        if(xferTargets[i].name == name) {
            xfer_sel = TargetRef{name, xferTargets[i].ship != nullptr};
        }
    }
    // A name-only request cannot know the kind, so an unresolved one is dropped
    // rather than left pending on a guess.
    if(xfer_sel.name != name) { xfer_sel = TargetRef(); }
    resolveTarget();
}

void TransferPlanner::update(const glm::dvec3 &com, const glm::dvec3 &vel) {
    xferTargets.clear();
    if(g->ship == nullptr) {
        // No active ship: drop any stale target / plan (orbit-view boot).
        xfer_sel = TargetRef();
        xfer_target = -1;
        xfer.valid = false;
        xfer.burn_dir = glm::dvec3(0.0);
        pc_period = 0.0;
        return;
    }
    {
        TerrainBody *pb = g->ship->frame->body;
        for(auto *b : g->sys.bodies) {
            if(b->frame && b->frame->parent == pb->frame) {
                xferTargets.push_back({b->name.c_str(), b, nullptr});
            }
        }
        /* Sibling vessels: Game::vehicles filtered to the subject's SoI body.
           Every entry is flyable -- a seated kerbal is a seat on its capsule's
           Part (part.h), never a target here. */
        for(auto *s : g->vehicles) {
            if(s != g->ship && s->m_parent == pb) {
                xferTargets.push_back({s->name.c_str(), nullptr, s});
            }
        }
        // --transfer-target wins over the window's combo on every rebuild.
        if(!g->args.transfer_target.empty()) {
            selectTargetByName(g->args.transfer_target);
        } else {
            resolveTarget();
        }
    }

    // Drop a grid / plan swept for a DIFFERENT target. By identity: the index
    // moves whenever the list reshuffles, and treating that as a target change
    // would discard a grid the player swept for the very same ship. The identity
    // has to carry the kind too, or deleting the vessel "Mun" hands its grid and
    // its armed plan to the moon Mun.
    if(pc.valid && pc_target != xfer_sel) {
        pc.valid = false;
    }

    if(xfer_from_porkchop && xfer_plan_target != xfer_sel) {
        clearPorkchopPlan();
    }

    if(xfer_target < 0) {
        xfer.valid = false;
        xfer.burn_dir = glm::dvec3(0.0);
        // Forget the solved target too: xfer.frame only advances while a target
        // is set, so re-picking the SAME index would clear no dirty term and
        // leave the solution blank for up to 30 frames.
        xfer.target = -2;
    } else if(!xfer_compute) {
        /* A target is also just something to look at: its orbit is on the map
           and its plane is one of the map's planes. With planning off nothing
           solves, so the HUD burn marker, the map arc and the window's solution
           block all go quiet together -- and xfer.target = -2 means a stale
           solution cannot be drawn for a frame on the way back, nor a solve be
           skipped on the way forward (that term is what makes it dirty). */
        xfer.valid = false;
        xfer.burn_dir = glm::dvec3(0.0);
        xfer.target = -2;
        // The target's own period still feeds the porkchop range sliders, which
        // describe the target rather than a plan. Same 30-frame cadence as a
        // live solve: a scalar the sliders read is not worth a state lift on
        // every frame.
        xfer.frame++;
        if(xfer.frame - xfer.period_frame >= 30) {
            xfer.period_frame = xfer.frame;
            const InertialShip s1 = shipInertial(*g, com, vel);
            pc_period = targetInertial(xferTargets[xfer_target], s1.inertial)
                            .tof_max / 3.0;
        }
    } else {
        xfer.frame++;
        const bool dirty = xfer.target != xfer_target
            || xfer.auto_tof != xfer_auto
            || std::fabs(xfer.tof_log - xfer_tof_log) > 1e-12
            || xfer.frame - xfer.solved_frame >= 30;
        if(dirty) {
            const InertialShip s1 = shipInertial(*g, com, vel);
            const XferTarget &t = xferTargets[xfer_target];
            const InertialTarget d = targetInertial(t, s1.inertial);
            // Scale for the porkchop range sliders: one target period, the
            // same quantity the auto range is built from (tof_max = 3 of
            // them), so the sliders' ends and the auto window cannot disagree.
            pc_period = d.tof_max / 3.0;
            xfer.period_frame = xfer.frame;   // the same lift, done

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
                const glm::dmat3 O = g->ship->frame->GetOrientRelTo(s1.inertial);
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
    if(g->args.xfer_log && xfer_target >= 0 && xfer_compute) {
        if(g->logDue(kGateXfer, SDL_GetTicks(), g->orbit_log_interval_ms)) {
            const char *tn = xferTargets[xfer_target].name;
            if(xfer.valid) {
                printf("[xferlog] t=%.1fs target=\"%s\" dv_dep=%.6g m/s "
                       "dv_cap=%.6g m/s total=%.6g m/s tof=%.6g s "
                       "v_inf=%.6g m/s r_cap=%.6g m "
                       "burn=[%.4f %.4f %.4f] retrograde=%d rev=%d\n",
                       g->time, tn, xfer.sol.dv_departure,
                       xfer.sol.dv_capture, xfer.sol.total_dv,
                       xfer.sol.tof, xfer.sol.v_inf, xfer.sol.r_cap,
                       xfer.burn_dir.x, xfer.burn_dir.y,
                       xfer.burn_dir.z, xfer.sol.retrograde ? 1 : 0,
                       xfer.sol.revolutions);
            } else {
                printf("[xferlog] t=%.1fs target=\"%s\" no-solution\n",
                       g->time, tn);
            }
            fflush(stdout);
        }
    }
}

void TransferPlanner::clearPorkchopPlan() {
    if(!xfer_from_porkchop) { return; }
    xfer_from_porkchop = false;
    xfer_t_dep = 0.0;
    xfer_plan_target = TargetRef();
    xfer_auto = xfer_prev_auto;
    xfer_tof_log = xfer_prev_tof_log;
}

void TransferPlanner::dropTargets() {
    xferTargets.clear();
    xfer_sel = TargetRef();
    xfer_target = -1;
}

void TransferPlanner::invalidateClockState() {
    // Clock jump or system swap: everything stamped against the old world is
    // stale. Clearing xferTargets also frees the old system's pointers.
    clearPorkchopPlan();
    pc.valid = false;
    xfer.valid = false;
    dropTargets();
    /* Deliberately NOT pc_in_flight. A sweep that is queued or running still
       owes its continuation, and that continuation is the only other place the
       count clears -- so zeroing it here (which this used to) tells the window
       the sweep is over mid-flight: its "sweeping" line vanishes, Compute
       re-enables, and the player posts a SECOND grid on top of the first. The
       one caller that really does discard continuations -- installSystem's
       jobs.abort() -- zeroes it where that happens (game.cpp). */
}

void TransferPlanner::resetForNewWorld() {
    /* Replace the whole planner, then move the survivors onto the replacement.
       Building the fresh one first keeps the survivor list a single run of
       assignments: a field added to the class resets by default, and a field
       that must survive appears exactly once -- a save/restore pair would have
       to be edited in two places that nothing checks against each other.

       What survives, and why:
         xfer_compute, pcCustomDep/pcDepLo/Hi, pcCustomTof/pcTofLo/Hi
             the player's setup in the TARGET and Porkchop windows. The bounds
             are what the window seeded them with rather than what a sweep
             produced, so they are the player's, not the world's.
         pc_rev   a monotonic stamp, not world state. The window caches its
             heatmap in a program-lifetime static keyed by rev (gameui.cpp), so
             restarting at 0 makes the new game's first sweep look unchanged:
             no rebuild, the old game's pixels, and the assert after that block
             fires as soon as the new grid is a different size.
         pc_in_flight
             a sweep posted before the teardown still runs its continuation --
             only installSystem aborts jobs -- so it still owns a slot.

       Everything else names a Vehicle, a body, or a clock stamp in the world
       that is going away. */
    TransferPlanner fresh(*g);
    fresh.xfer_compute = xfer_compute;
    fresh.pcCustomDep = pcCustomDep;
    fresh.pcDepLo = pcDepLo;
    fresh.pcDepHi = pcDepHi;
    fresh.pcCustomTof = pcCustomTof;
    fresh.pcTofLo = pcTofLo;
    fresh.pcTofHi = pcTofHi;
    fresh.pc_rev = pc_rev;
    fresh.pc_in_flight = pc_in_flight;
    *this = std::move(fresh);
}

bool TransferPlanner::targetPlaneNormal(Frame *focus_inertial,
                                        glm::dvec3 &n_hat) const {
    if(focus_inertial == nullptr
       || xfer_target < 0 || xfer_target >= (int)xferTargets.size()) {
        return false;
    }
    const XferTarget &t = xferTargets[xfer_target];
    // A grounded vessel's "conic" about the focus is the circle the ground draws
    // under it -- the body's own equator plane, which the Equatorial slot
    // already offers under an honest name.
    if(t.ship && t.ship->isGrounded()) { return false; }
    // The same state lift the solver uses, so the plane the map shows cannot
    // disagree with the transfer planned to this target.
    const InertialTarget d = targetInertial(t, focus_inertial);
    const glm::dvec3 h = glm::cross(d.r, d.v);
    const double hl = glm::length(h);
    if(hl <= 1e-9) { return false; }
    // An escaping ship target has no orbit to share a plane with. A body target
    // is on rails, so its conic is closed by construction.
    if(t.ship && orbitalPeriod(d.r, d.v, focus_inertial->body->mu) <= 0.0) {
        return false;
    }
    n_hat = h / hl;
    return true;
}

void TransferPlanner::porkchopCompute() {
    if(xfer_target < 0 || xfer_target >= (int)xferTargets.size()) { return; }
    // One sweep at a time: the window's button is disabled mid-sweep but the P
    // key is not, and a --porkchop-bench list can queue minutes of worker time
    // that abort() would then have to wait out.
    if(pc_in_flight > 0) { return; }
    const XferTarget &t = xferTargets[xfer_target];

    // Snapshot ship/target state in the parent's INERTIAL frame -- the ONLY game
    // state read; the grid sweep is pure and runs on the background worker.
    const InertialShip s1 = shipInertial(*g, g->hud.view.pos, g->hud.view.vel);
    const InertialTarget d = targetInertial(t, s1.inertial);

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
    /* --porkchop-bench sweeps several sizes off THIS one snapshot, so the grids
       differ only in resolution. Swept ascending; the live plot keeps the largest. */
    std::vector<int> sizes = g->args.porkchop_bench;
    // assign, not push_back: GCC 15 loses the buffer origin through the
    // unique/erase below and warns -Wfree-nonheap-object on push_back's growth
    // path -- which cannot run here, since sizes is empty so capacity is 0.
    // False positive (ASan-clean), but it was the tree's only warning.
    if(sizes.empty()) { sizes.assign(1, g->args.porkchop_n); }
    // A repeated size would redo identical work and rewrite the same CSV.
    std::sort(sizes.begin(), sizes.end());
    sizes.erase(std::unique(sizes.begin(), sizes.end()), sizes.end());
    const bool dump = !g->args.porkchop_dump.empty();
    // A bench sweep exists to be read, so it implies --porkchop-log.
    const bool log = g->args.porkchop_log || dump || !g->args.porkchop_bench.empty();
    const std::string dump_dir = g->args.porkchop_dump;
    const bool capture = d.capture;
    const std::string tname = t.name;
    // The kind travels with the name: a grid swept for the VESSEL "Mun" must not
    // be handed to the moon Mun if the vessel is deleted mid-sweep.
    const bool t_is_ship = t.ship != nullptr;
    const double t_now = g->time;
    const int epoch = g->cache_epoch;      // drop the result if the world changes
    JobRunner *jobs = &g->jobs;           // stopping() is thread-safe (job.h)

    pc_in_flight++;
    g->jobs.post("Porkchop grid", [r1,v1,r2,v2,mu_p,mu_t,r_cap,
                                  t_dep_lo,t_dep_hi,tof_lo,tof_hi,
                                  sizes,capture,log,tname,t_is_ship,t_now,
                                  epoch,dump,dump_dir,jobs,this]()
                -> std::function<void()> {
        // Worker: PURE (no game state, GL, or imgui). shared_ptr because the
        // std::function continuation capture must be copyable, not moved.
        std::shared_ptr<PorkchopResult> res;
        for(const int n : sizes) {
            // abort()/join() drops QUEUED jobs but still joins the IN-FLIGHT
            // body, so a multi-grid sweep must notice the stop between grids or
            // it holds the main thread for its whole remaining runtime.
            if(jobs->stopping()) { break; }
            res.reset();   // the previous grid is done with; don't peak at 2
            const auto t0 = std::chrono::steady_clock::now();
            try {
                res = std::make_shared<PorkchopResult>(porkchopGrid(
                    r1,v1,r2,v2,mu_p,mu_t,r_cap,
                    t_dep_lo,t_dep_hi,tof_lo,tof_hi,
                    n,n,capture));
            } catch(const std::exception &e) {
                // JobRunner swallows it and skips the continuation -- which is
                // what clears pc_in_flight. Catching here keeps the "sweeping"
                // indicator from wedging the window.
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
        return [this, res, t_now, tname, t_is_ship, epoch]() {
            // Epoch bumped since posting: the grid is for the old world (load
            // does NOT abort jobs). Drop before touching the target list -- a
            // SHIP target list was freed by the load (UAF risk).
            if(epoch != this->g->cache_epoch) {
                if(pc_in_flight > 0) { pc_in_flight--; }
                return;
            }
            const bool still_target = (xfer_target >= 0
                && xfer_target < (int)xferTargets.size()
                && xferTargets[xfer_target].name == tname
                && (xferTargets[xfer_target].ship != nullptr) == t_is_ship);
            if(still_target && res) {
                pc = std::move(*res);
                pc_computed_at = t_now;
                pc_rev++;   // the heatmap texture is a cache of pc; this invalidates it
                // Stamp the IDENTITY the sweep was for, not the index it landed
                // on: the list is rebuilt every frame, so a sibling ship joining
                // or leaving shifts the index while the vessel is the same, and
                // update() would drop a grid that is still exactly right.
                pc_target = TargetRef{tname, t_is_ship};
            }
            if(pc_in_flight > 0) { pc_in_flight--; }
        };
    });
}
