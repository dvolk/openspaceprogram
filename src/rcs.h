#pragma once

// rcs.h -- the RCS nozzle allocator as pure math (glm only), so tests/ can
// pin it without the physics chain (#428). Translation only: attitude
// authority is #425's territory.
//
// Model: every RCS nozzle is ONE-WAY (fires along its direction, never
// pulls) with a rated thrust. Given a demanded force F (world frame), pick
// per-nozzle strengths so the ship does what the player asked:
//   sum c_i * thrust_i * dir_i      ~= F          (the commanded move)
//   sum c_i * thrust_i * (arm_i x dir_i) ~= 0     (no unwanted spin)
// Force wins when they conflict (the player asked for a move; a spin is the
// honest physical cost of an off-center pod); exact cancellation is taken
// when free. c is solved as a fraction of each nozzle's rating, so c in
// [0, 1] means "within rating"; a clipped c_i > 1 is an honest shortfall the
// caller reports ([rcs] log), not silently free thrust.

#include <cassert>
#include <cmath>
#include <string>
#include <vector>

#include <glm/glm.hpp>

struct RcsNozzle {
    glm::dvec3 dir = glm::dvec3(0.0);  // unit, world frame: pushes the ship this way
    glm::dvec3 arm = glm::dvec3(0.0);  // nozzle pos - COM, world frame
    double thrust = 0.0;               // rated N per nozzle (> 0)
    std::string tag;                   // label for logs (e.g. "+Z")
};

namespace rcs {

/* Every work buffer the allocator uses, as flat row-major vectors sized to
   the biggest solve seen. applyRcsForce runs every physics SUBSTEP, so it
   holds one of these and the hot path allocates nothing after warmup;
   rcsAllocate() below wraps it for tests. A Scratch belongs to ONE solve at
   a time (not reentrant); the Into functions own their buffers' contents
   and fully overwrite them, so stale capacity is never stale data. */
struct Scratch {
    std::vector<double> c;     // solution: per-nozzle rating fractions
    std::vector<double> W;     // 6 x N allocator matrix
    std::vector<double> bd;    // demand: 3 force + 3 weighted torque rows
    std::vector<double> bdF;   // demand left after pinned nozzles
    std::vector<double> Wf;    // 6 x k free-nozzle columns of W
    std::vector<double> x;     // nnls solution
    std::vector<double> Wx;    // W x (m rows)
    std::vector<char> P;       // nnls passive set: 1 = in the solution
    std::vector<size_t> idx;   // nnls passive column indices
    std::vector<double> A;     // m x |idx| column subset for solveLS
    std::vector<double> z;     // solveLS solution
    std::vector<double> ATA;   // solveLS normal equations: |idx| x (|idx|+1)
    std::vector<char> pinned;  // allocator: nozzle held at rating
    std::vector<size_t> freeIdx;  // allocator: not-yet-pinned columns
};

/* Least squares on the first n columns of a flat row-major m x n A; x is
   the solution. Same normal-equations + Gaussian elimination with partial
   pivoting as before: rank-deficient subsets (duplicate or opposed
   columns) are carried by the pivot-skip + back-sub-zero paths, and the
   tiny ridge is only insurance against exactly-zero pivots. n <= ~6
   columns, O(n^3) noise. */
inline void solveLS(const double *A, size_t m, size_t n, const double *b,
                    Scratch &s) {
    std::vector<double> &ATA = s.ATA;
    std::vector<double> &x = s.z;
    ATA.assign(n * (n + 1), 0.0);
    x.assign(n, 0.0);
    for(size_t i = 0; i < n; i++) {
        for(size_t j = 0; j < n; j++) {
            double t = 0.0;
            for(size_t r = 0; r < m; r++) { t += A[r * n + i] * A[r * n + j]; }
            if(i == j) { t += 1e-12; }
            ATA[i * (n + 1) + j] = t;
        }
        double t = 0.0;
        for(size_t r = 0; r < m; r++) { t += A[r * n + i] * b[r]; }
        ATA[i * (n + 1) + n] = t;
    }
    for(size_t col = 0; col < n; col++) {
        size_t piv = col;
        for(size_t r = col + 1; r < n; r++) {
            if(std::fabs(ATA[r * (n + 1) + col]) > std::fabs(ATA[piv * (n + 1) + col])) { piv = r; }
        }
        if(std::fabs(ATA[piv * (n + 1) + col]) < 1e-15) { continue; }  // ridge covers it
        if(piv != col) {
            for(size_t k = 0; k <= n; k++) { std::swap(ATA[col * (n + 1) + k], ATA[piv * (n + 1) + k]); }
        }
        for(size_t r = col + 1; r < n; r++) {
            const double f = ATA[r * (n + 1) + col] / ATA[col * (n + 1) + col];
            for(size_t k = col; k <= n; k++) { ATA[r * (n + 1) + k] -= f * ATA[col * (n + 1) + k]; }
        }
    }
    for(int row = (int)n - 1; row >= 0; row--) {
        double t = ATA[row * (n + 1) + n];
        for(size_t k = (size_t)row + 1; k < n; k++) { t -= ATA[row * (n + 1) + k] * x[k]; }
        x[row] = std::fabs(ATA[row * (n + 1) + row]) < 1e-15 ? 0.0 : t / ATA[row * (n + 1) + row];
    }
}

/* Lawson-Hanson NNLS on the flat row-major m x n W: min ||W c - b||^2
   subject to c >= 0; the solution lands in s.x. */
inline void nnls(const double *W, size_t m, size_t n, const double *b, Scratch &s) {
    std::vector<double> &x = s.x;
    std::vector<char> &P = s.P;
    x.assign(n, 0.0);
    P.assign(n, 0);  // 1 = passive (in the solution)
    /* Enter only on gradients meaningful FOR THIS SYSTEM: columns are scaled
       by rating x force weight (1e5..1e6), so an ABSOLUTE KKT tolerance lets
       float-noise gains re-enter dropped columns forever (enter/drop limit
       cycle -- the #428 review's B2). x itself is a fraction of rating, so
       the drop tol stays absolute. */
    double wScale = 0.0;
    for(size_t r = 0; r < m * n; r++) { wScale = std::max(wScale, std::fabs(W[r])); }
    double bScale = 0.0;
    for(size_t r = 0; r < m; r++) { bScale = std::max(bScale, std::fabs(b[r])); }
    const double tolEnter = 1e-9 * wScale * bScale;
    const double tolX = 1e-10;
    for(size_t iter = 0; iter <= n; iter++) {
        // Gradient w = W^T (b - W x); pick the largest positive outside P.
        std::vector<double> &Wx = s.Wx;
        Wx.assign(m, 0.0);
        for(size_t r = 0; r < m; r++) {
            double t = 0.0;
            for(size_t j = 0; j < n; j++) { t += W[r * n + j] * x[j]; }
            Wx[r] = t;
        }
        int best = -1;
        double bestw = 0.0;
        for(size_t j = 0; j < n; j++) {
            if(P[j]) { continue; }
            double wj = 0.0;
            for(size_t r = 0; r < m; r++) { wj += W[r * n + j] * (b[r] - Wx[r]); }
            if(wj > bestw) { bestw = wj; best = (int)j; }
        }
        if(best < 0 || bestw <= tolEnter) { return; }  // KKT: nothing improves
        P[best] = 1;
        // Solve LS on the passive columns; loop dropping non-positives.
        std::vector<size_t> &idx = s.idx;
        idx.clear();
        for(size_t j = 0; j < n; j++) { if(P[j]) { idx.push_back(j); } }
        for(size_t inner = 0; inner <= n; inner++) {
            std::vector<double> &A = s.A;
            A.assign(m * idx.size(), 0.0);
            for(size_t r = 0; r < m; r++) {
                for(size_t k = 0; k < idx.size(); k++) { A[r * idx.size() + k] = W[r * n + idx[k]]; }
            }
            solveLS(A.data(), m, idx.size(), b, s);
            const double *z = s.z.data();
            double alpha = 1.0;
            for(size_t k = 0; k < idx.size(); k++) {
                if(z[k] <= 0.0) {
                    const double old = x[idx[k]];
                    const double t = old / (old - z[k]);
                    if(t < alpha) { alpha = t; }
                }
            }
            bool dropped = false;
            for(size_t k = 0; k < idx.size(); k++) {
                const double nx = x[idx[k]] + alpha * (z[k] - x[idx[k]]);
                if(nx <= tolX) { x[idx[k]] = 0.0; P[idx[k]] = 0; dropped = true; }
                else { x[idx[k]] = nx; }
            }
            if(!dropped) { break; }
            idx.clear();
            for(size_t j = 0; j < n; j++) { if(P[j]) { idx.push_back(j); } }
            assert(!idx.empty());
        }
    }
    // iteration cap: whatever the active set settled on
}

}  // namespace rcs

/* Allocate nozzle strengths for the demanded force Fdem (world frame, N).
   c_i lands in scratch.c, in [0, 1]: a fraction of each nozzle's rating.
   Rows: force is scaled by forceWeight = 1e3, torque rows are
   nondimensionalized by the longest arm and weighted by torqueWeight --
   the commanded force wins any conflict (it is the move the player asked
   for; the spin of an off-center pod is the honest physical cost), while
   spin cancellation that costs the force NOTHING is taken. Ratings are
   BOUNDS, not a post-hoc clip: a solved c above 1 pins that nozzle at full
   rating and the leftover demand is re-solved across the idle nozzles
   (each pass finishes or pins at least one nozzle, so at most N passes;
   post-clip-without-re-solve silently loses thrust -- the #428 review's
   B1). What stays unmet after that is a real shortfall the caller reports
   ([rcs] log), never free thrust. */
inline void rcsAllocateInto(const std::vector<RcsNozzle> &noz,
                            const glm::dvec3 &Fdem,
                            rcs::Scratch &s,
                            double torqueWeight = 0.05) {
    assert(torqueWeight >= 0.0);
    const size_t N = noz.size();
    std::vector<double> &c = s.c;
    c.assign(N, 0.0);
    if(N == 0) { return; }
    const double forceWeight = 1000.0;
    double Lref = 1.0;  // nondimensionalize torque: arm x dir has [m], F has [N]
    for(const RcsNozzle &nz : noz) { Lref = std::max(Lref, glm::length(nz.arm)); }
    // Rows: 3 force + 3 (weighted, nondim) torque. Column i is scaled by the
    // nozzle rating, so the solution is a fraction of rating. Column-major
    // would serve the per-column writes below, but every reader wants rows.
    std::vector<double> &W = s.W;
    W.assign(6 * N, 0.0);
    std::vector<double> &bd = s.bd;
    bd.assign(6, 0.0);
    for(size_t i = 0; i < N; i++) {
        const RcsNozzle &nz = noz[i];
        assert(nz.thrust > 0.0);
        const glm::dvec3 tq = glm::cross(nz.arm, nz.dir) * (torqueWeight / Lref);
        W[0 * N + i] = nz.dir.x * nz.thrust * forceWeight; W[1 * N + i] = nz.dir.y * nz.thrust * forceWeight; W[2 * N + i] = nz.dir.z * nz.thrust * forceWeight;
        W[3 * N + i] = tq.x * nz.thrust;                  W[4 * N + i] = tq.y * nz.thrust;                  W[5 * N + i] = tq.z * nz.thrust;
    }
    bd[0] = Fdem.x * forceWeight; bd[1] = Fdem.y * forceWeight; bd[2] = Fdem.z * forceWeight;

    std::vector<char> &pinned = s.pinned;
    pinned.assign(N, 0);
    std::vector<size_t> &freeIdx = s.freeIdx;
    freeIdx.clear();
    for(size_t j = 0; j < N; j++) { freeIdx.push_back(j); }
    for(size_t pass = 0; pass <= N; pass++) {
        // Demand left once the pinned-at-rating nozzles have had their say.
        std::vector<double> &bdF = s.bdF;
        bdF = bd;
        for(size_t j = 0; j < N; j++) {
            if(!pinned[j]) { continue; }
            for(size_t r = 0; r < 6; r++) { bdF[r] -= W[r * N + j]; }
        }
        if(!freeIdx.empty()) {
            // nnls writes its solution to s.x; alias it as cf for the pass.
            std::vector<double> &Wf = s.Wf;
            Wf.assign(6 * freeIdx.size(), 0.0);
            for(size_t r = 0; r < 6; r++) {
                for(size_t k = 0; k < freeIdx.size(); k++) { Wf[r * freeIdx.size() + k] = W[r * N + freeIdx[k]]; }
            }
            rcs::nnls(Wf.data(), 6, freeIdx.size(), bdF.data(), s);
        }
        const std::vector<double> &cf = freeIdx.empty() ? c : s.x;  // empty freeIdx: loop below is empty
        bool pinnedMore = false;
        for(size_t k = 0; k < freeIdx.size(); k++) {
            if(cf[k] > 1.0 + 1e-9) {
                c[freeIdx[k]] = 1.0;
                pinned[freeIdx[k]] = 1;
                pinnedMore = true;
            }
        }
        if(!pinnedMore) {
            for(size_t k = 0; k < freeIdx.size(); k++) { c[freeIdx[k]] = cf[k]; }
            return;
        }
        freeIdx.clear();
        for(size_t j = 0; j < N; j++) { if(!pinned[j]) { freeIdx.push_back(j); } }
    }
    // every nozzle at rating: the demand simply exceeds the ship
}

// Test-side wrapper: one-shot Scratch, solution returned. The game's hot
// path calls rcsAllocateInto with a held Scratch (Vehicle::rcsScratch).
inline std::vector<double> rcsAllocate(const std::vector<RcsNozzle> &noz,
                                       const glm::dvec3 &Fdem,
                                       double torqueWeight = 0.05) {
    rcs::Scratch s;
    rcsAllocateInto(noz, Fdem, s, torqueWeight);
    return s.c;
}
