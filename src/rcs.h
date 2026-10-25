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

/* Small dense least squares via normal equations + Gaussian elimination with
   partial pivoting. Rank-deficient subsets (duplicate or opposed columns)
   are carried by the pivot-skip + back-sub-zero paths; the tiny ridge is
   only insurance against exactly-zero pivots. n <= ~6 columns, O(n^3) noise. */
inline std::vector<double> solveLS(const std::vector<std::vector<double> > &A,
                                   const std::vector<double> &b) {
    const size_t n = A.empty() ? 0 : A[0].size();
    std::vector<std::vector<double> > ATA(n, std::vector<double>(n + 1, 0.0));
    for(size_t i = 0; i < n; i++) {
        for(size_t j = 0; j < n; j++) {
            double s = 0.0;
            for(size_t r = 0; r < A.size(); r++) { s += A[r][i] * A[r][j]; }
            if(i == j) { s += 1e-12; }
            ATA[i][j] = s;
        }
        double s = 0.0;
        for(size_t r = 0; r < A.size(); r++) { s += A[r][i] * b[r]; }
        ATA[i][n] = s;
    }
    for(size_t col = 0; col < n; col++) {
        size_t piv = col;
        for(size_t r = col + 1; r < n; r++) {
            if(std::fabs(ATA[r][col]) > std::fabs(ATA[piv][col])) { piv = r; }
        }
        if(std::fabs(ATA[piv][col]) < 1e-15) { continue; }  // ridge covers it
        std::swap(ATA[col], ATA[piv]);
        for(size_t r = col + 1; r < n; r++) {
            const double f = ATA[r][col] / ATA[col][col];
            for(size_t k = col; k <= n; k++) { ATA[r][k] -= f * ATA[col][k]; }
        }
    }
    std::vector<double> x(n, 0.0);
    for(int row = (int)n - 1; row >= 0; row--) {
        double s = ATA[row][n];
        for(size_t k = (size_t)row + 1; k < n; k++) { s -= ATA[row][k] * x[k]; }
        x[row] = std::fabs(ATA[row][row]) < 1e-15 ? 0.0 : s / ATA[row][row];
    }
    return x;
}

/* Lawson-Hanson NNLS: min ||W c - b||^2 subject to c >= 0. */
inline std::vector<double> nnls(const std::vector<std::vector<double> > &W,
                                const std::vector<double> &b) {
    const size_t m = W.size();
    const size_t n = m ? W[0].size() : 0;
    std::vector<double> x(n, 0.0);
    std::vector<char> P(n, 0);  // 1 = passive (in the solution)
    /* Enter only on gradients meaningful FOR THIS SYSTEM: columns are scaled
       by rating x force weight (1e5..1e6), so an ABSOLUTE KKT tolerance lets
       float-noise gains re-enter dropped columns forever (enter/drop limit
       cycle -- the #428 review's B2). x itself is a fraction of rating, so
       the drop tol stays absolute. */
    double wScale = 0.0;
    for(const std::vector<double> &row : W) {
        for(double v : row) { wScale = std::max(wScale, std::fabs(v)); }
    }
    double bScale = 0.0;
    for(double v : b) { bScale = std::max(bScale, std::fabs(v)); }
    const double tolEnter = 1e-9 * wScale * bScale;
    const double tolX = 1e-10;
    for(size_t iter = 0; iter <= n; iter++) {
        // Gradient w = W^T (b - W x); pick the largest positive outside P.
        std::vector<double> Wx(m, 0.0);
        for(size_t r = 0; r < m; r++) {
            double s = 0.0;
            for(size_t j = 0; j < n; j++) { s += W[r][j] * x[j]; }
            Wx[r] = s;
        }
        int best = -1;
        double bestw = 0.0;
        for(size_t j = 0; j < n; j++) {
            if(P[j]) { continue; }
            double wj = 0.0;
            for(size_t r = 0; r < m; r++) { wj += W[r][j] * (b[r] - Wx[r]); }
            if(wj > bestw) { bestw = wj; best = (int)j; }
        }
        if(best < 0 || bestw <= tolEnter) { return x; }  // KKT: nothing improves
        P[best] = 1;
        // Solve LS on the passive columns; loop dropping non-positives.
        std::vector<size_t> idx;
        for(size_t j = 0; j < n; j++) { if(P[j]) { idx.push_back(j); } }
        for(size_t inner = 0; inner <= n; inner++) {
            std::vector<std::vector<double> > A(m, std::vector<double>(idx.size()));
            for(size_t r = 0; r < m; r++) {
                for(size_t k = 0; k < idx.size(); k++) { A[r][k] = W[r][idx[k]]; }
            }
            const std::vector<double> s = solveLS(A, b);
            double alpha = 1.0;
            for(size_t k = 0; k < idx.size(); k++) {
                if(s[k] <= 0.0) {
                    const double old = x[idx[k]];
                    const double t = old / (old - s[k]);
                    if(t < alpha) { alpha = t; }
                }
            }
            bool dropped = false;
            for(size_t k = 0; k < idx.size(); k++) {
                const double nx = x[idx[k]] + alpha * (s[k] - x[idx[k]]);
                if(nx <= tolX) { x[idx[k]] = 0.0; P[idx[k]] = 0; dropped = true; }
                else { x[idx[k]] = nx; }
            }
            if(!dropped) { break; }
            idx.clear();
            for(size_t j = 0; j < n; j++) { if(P[j]) { idx.push_back(j); } }
            assert(!idx.empty());
        }
    }
    return x;  // iteration cap: whatever the active set settled on
}

}  // namespace rcs

/* Allocate nozzle strengths for the demanded force Fdem (world frame, N).
   Returns c_i in [0, 1]: a fraction of each nozzle's rating. Rows: force is
   scaled by forceWeight = 1e3, torque rows are nondimensionalized by the
   longest arm and weighted by torqueWeight -- the commanded force wins any
   conflict (it is the move the player asked for; the spin of an off-center
   pod is the honest physical cost), while spin cancellation that costs the
   force NOTHING is taken. Ratings are BOUNDS, not a post-hoc clip: a solved
   c above 1 pins that nozzle at full rating and the leftover demand is
   re-solved across the idle nozzles (each pass finishes or pins at least one
   nozzle, so at most N passes; post-clip-without-re-solve silently loses
   thrust -- the #428 review's B1). What stays unmet after that is a real
   shortfall the caller reports ([rcs] log), never free thrust. */
inline std::vector<double> rcsAllocate(const std::vector<RcsNozzle> &noz,
                                       const glm::dvec3 &Fdem,
                                       double torqueWeight = 0.05) {
    assert(torqueWeight >= 0.0);
    const size_t N = noz.size();
    std::vector<double> c(N, 0.0);
    if(N == 0) { return c; }
    const double forceWeight = 1000.0;
    double Lref = 1.0;  // nondimensionalize torque: arm x dir has [m], F has [N]
    for(const RcsNozzle &nz : noz) { Lref = std::max(Lref, glm::length(nz.arm)); }
    // Rows: 3 force + 3 (weighted, nondim) torque. Column i is scaled by the
    // nozzle rating, so the solution is a fraction of rating.
    std::vector<std::vector<double> > W(6, std::vector<double>(N));
    std::vector<double> bd(6, 0.0);
    for(size_t i = 0; i < N; i++) {
        const RcsNozzle &nz = noz[i];
        assert(nz.thrust > 0.0);
        const glm::dvec3 tq = glm::cross(nz.arm, nz.dir) * (torqueWeight / Lref);
        W[0][i] = nz.dir.x * nz.thrust * forceWeight; W[1][i] = nz.dir.y * nz.thrust * forceWeight; W[2][i] = nz.dir.z * nz.thrust * forceWeight;
        W[3][i] = tq.x * nz.thrust;                  W[4][i] = tq.y * nz.thrust;                  W[5][i] = tq.z * nz.thrust;
    }
    bd[0] = Fdem.x * forceWeight; bd[1] = Fdem.y * forceWeight; bd[2] = Fdem.z * forceWeight;

    std::vector<char> pinned(N, 0);
    std::vector<size_t> freeIdx;
    for(size_t j = 0; j < N; j++) { freeIdx.push_back(j); }
    for(size_t pass = 0; pass <= N; pass++) {
        // Demand left once the pinned-at-rating nozzles have had their say.
        std::vector<double> bdF = bd;
        for(size_t j = 0; j < N; j++) {
            if(!pinned[j]) { continue; }
            for(size_t r = 0; r < 6; r++) { bdF[r] -= W[r][j]; }
        }
        std::vector<double> cf;
        if(!freeIdx.empty()) {
            std::vector<std::vector<double> > Wf(6, std::vector<double>(freeIdx.size()));
            for(size_t r = 0; r < 6; r++) {
                for(size_t k = 0; k < freeIdx.size(); k++) { Wf[r][k] = W[r][freeIdx[k]]; }
            }
            cf = rcs::nnls(Wf, bdF);
        }
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
            return c;
        }
        freeIdx.clear();
        for(size_t j = 0; j < N; j++) { if(!pinned[j]) { freeIdx.push_back(j); } }
    }
    return c;  // every nozzle at rating: the demand simply exceeds the ship
}
