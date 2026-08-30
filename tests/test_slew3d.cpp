//
// Regression test for the prograde/retrograde slew law in 3 DOF
// (Vehicle::slewToward, src/vehicle.h). Pure C++ -- no Bullet/GL -- so it
// runs anywhere `make test` does.
//
// THE BUG this pins (fixed 2026-08-30):
//   The slew drives the nose (local +Z) toward a target (prograde/retrograde).
//   The OLD law applied torque along the instantaneous SLEW AXIS only
//   (a = nose x target), damping only the angular-velocity component about a.
//   The perpendicular transverse component -- the "third axis" b = a x nose --
//   was NEVER damped. Any residual spin about b at engagement (from the player's
//   last stick input, or developed as the orbit target curves) therefore
//   persisted, grew via gyroscopic coupling, and as the slew axis rotated that
//   undamped spin coupled into the nose: a sustained wobble around the target
//   (E ringing, the slew-axis rate sign-flipping).
//
//   The FIX drives the FULL transverse angular velocity (the part
//   perpendicular to the nose) toward the braking-curve rate about the slew
//   axis, authority-bounded. That kills the third-axis spin while preserving
//   the braking curve on the slew axis. Roll about the nose stays free.
//
// This test simulates an axisymmetric rigid body (the racer) under both laws,
// from a residual third-axis spin, across a grid of spin magnitudes and warps,
// and asserts:
//   1. THE FIX: the NEW law damps the third-axis spin (end rate well below its
//      initial value) in EVERY configuration, and converges to the target.
//      That damping is the core observable difference from the OLD law.
//   2. Authority bound: the NEW law's torque never exceeds the wheel rating,
//      so the autopilot is never more forceful than a maxed manual stick.
//   3. Settling: in the realistic spin range (w3 <= 0.30, well under a hard
//      roll) the NEW law settles to a small E-ring -- no sustained wobble.
//      (The extreme w3=0.50, a 28 deg/s roll, may overshoot before converging;
//      it still damps the spin and reaches the target per check 1.)
//   4. Non-vacuous guard: the OLD law leaves the third-axis spin undamped in
//      at least one configuration -- proving the harness can catch the bug.
//
// Build & run (from repo root) -- also part of `make test`:
//   see the test_slew3d rule in the Makefile.

#include <cmath>
#include <cstdio>
#include <algorithm>

static int g_failures = 0;
static int g_checks = 0;

#define CHECK_TRUE(cond, msg)                                                  \
    do {                                                                       \
        g_checks++;                                                            \
        if (!(cond)) {                                                         \
            g_failures++;                                                      \
            printf("FAIL: %s\n", msg);                                         \
        }                                                                      \
    } while (0)

// ---- minimal 3-vector / 3x3 math (no external deps) ----------------------
struct V3 { double x, y, z; };
static V3 v3(double x, double y, double z) { return V3{x, y, z}; }
static V3 operator+(V3 a, V3 b) { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static V3 operator-(V3 a, V3 b) { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static V3 operator*(V3 a, double s) { return v3(a.x*s, a.y*s, a.z*s); }
static V3 operator*(double s, V3 a) { return v3(a.x*s, a.y*s, a.z*s); }
static V3 operator/(V3 a, double s) { return v3(a.x/s, a.y/s, a.z/s); }
static double dot(V3 a, V3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static V3 cross(V3 a, V3 b) {
    return v3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}
static double vlen2(V3 a) { return dot(a, a); }
static double vlen(V3 a) { return std::sqrt(dot(a, a)); }
static V3 vnorm(V3 a) { double l = vlen(a); return (l > 1e-15) ? (a * (1.0/l)) : V3{0,0,0}; }

struct M3 { double m[3][3]; };
static V3 matvec(M3 a, V3 v) {
    return v3(a.m[0][0]*v.x+a.m[0][1]*v.y+a.m[0][2]*v.z,
              a.m[1][0]*v.x+a.m[1][1]*v.y+a.m[1][2]*v.z,
              a.m[2][0]*v.x+a.m[2][1]*v.y+a.m[2][2]*v.z);
}
static M3 matmul(M3 a, M3 b) {
    M3 r; for(int i=0;i<3;i++)for(int j=0;j<3;j++){
        double s=0; for(int k=0;k<3;k++)s+=a.m[i][k]*b.m[k][j]; r.m[i][j]=s; }
    return r;
}
static M3 skewm(V3 w) {
    M3 r;
    r.m[0][0]=0; r.m[0][1]=-w.z; r.m[0][2]=w.y;
    r.m[1][0]=w.z; r.m[1][1]=0; r.m[1][2]=-w.x;
    r.m[2][0]=-w.y; r.m[2][1]=w.x; r.m[2][2]=0;
    return r;
}
static M3 m3scale(M3 a, double s) { M3 r; for(int i=0;i<3;i++)for(int j=0;j<3;j++)r.m[i][j]=a.m[i][j]*s; return r; }
static M3 m3add(M3 a, M3 b) { M3 r; for(int i=0;i<3;i++)for(int j=0;j<3;j++)r.m[i][j]=a.m[i][j]+b.m[i][j]; return r; }
// world-frame inertia of the axisymmetric body (Ix=Iy): Ix*I + (Iz-Ix) nn^T
static M3 world_inertia(double Ix, double Iz, V3 n) {
    M3 r;
    double nd[3] = {n.x, n.y, n.z};
    for(int i=0;i<3;i++)for(int j=0;j<3;j++)
        r.m[i][j] = (i==j) ? Ix : 0.0;
    for(int i=0;i<3;i++)for(int j=0;j<3;j++)
        r.m[i][j] += (Iz - Ix) * nd[i] * nd[j];
    return r;
}
// re-orthonormalize R via Gram-Schmidt (keeps column 2 = the nose)
static M3 reorthonormalize(M3 R) {
    V3 c0 = v3(R.m[0][0], R.m[1][0], R.m[2][0]);
    V3 c2 = v3(R.m[0][2], R.m[1][2], R.m[2][2]);
    V3 e2 = vnorm(c2);
    V3 e0 = vnorm(c0 - e2 * dot(c0, e2));
    V3 e1 = cross(e2, e0);
    M3 r;
    r.m[0][0]=e0.x; r.m[1][0]=e0.y; r.m[2][0]=e0.z;
    r.m[0][1]=e1.x; r.m[1][1]=e1.y; r.m[2][1]=e1.z;
    r.m[0][2]=e2.x; r.m[1][2]=e2.y; r.m[2][2]=e2.z;
    return r;
}

// ---- axisymmetric body + the two slew laws ------------------------------
struct Body { double Ix, Iz, maxTorque; };

// Euler step for the axisymmetric body: Iw dW/dt = tau - (Iz-Ix)(W.n)(W x n)
static V3 euler_dW(const Body &b, V3 n, V3 W, V3 tau) {
    M3 Iw = world_inertia(b.Ix, b.Iz, n);
    V3 rhs = tau - ((b.Iz - b.Ix) * dot(W, n)) * cross(W, n);
    // solve Iw x = rhs (3x3 inverse)
    double det = Iw.m[0][0]*(Iw.m[1][1]*Iw.m[2][2]-Iw.m[1][2]*Iw.m[2][1])
               - Iw.m[0][1]*(Iw.m[1][0]*Iw.m[2][2]-Iw.m[1][2]*Iw.m[2][0])
               + Iw.m[0][2]*(Iw.m[1][0]*Iw.m[2][1]-Iw.m[1][1]*Iw.m[2][0]);
    if(std::fabs(det) < 1e-30) return v3(0,0,0);
    M3 inv;
    inv.m[0][0]=( Iw.m[1][1]*Iw.m[2][2]-Iw.m[1][2]*Iw.m[2][1])/det;
    inv.m[0][1]=(-Iw.m[0][1]*Iw.m[2][2]+Iw.m[0][2]*Iw.m[2][1])/det;
    inv.m[0][2]=( Iw.m[0][1]*Iw.m[1][2]-Iw.m[0][2]*Iw.m[1][1])/det;
    inv.m[1][0]=(-Iw.m[1][0]*Iw.m[2][2]+Iw.m[1][2]*Iw.m[2][0])/det;
    inv.m[1][1]=( Iw.m[0][0]*Iw.m[2][2]-Iw.m[0][2]*Iw.m[2][0])/det;
    inv.m[1][2]=(-Iw.m[0][0]*Iw.m[1][2]+Iw.m[0][2]*Iw.m[1][0])/det;
    inv.m[2][0]=( Iw.m[1][0]*Iw.m[2][1]-Iw.m[1][1]*Iw.m[2][0])/det;
    inv.m[2][1]=(-Iw.m[0][0]*Iw.m[2][1]+Iw.m[0][1]*Iw.m[2][0])/det;
    inv.m[2][2]=( Iw.m[0][0]*Iw.m[1][1]-Iw.m[0][1]*Iw.m[1][0])/det;
    return matvec(inv, rhs);
}

struct SlewResult {
    bool bad;         // NaN/Inf
    double final_E;   // rad, nose-target angle at the end of the run
    double E_ring;    // peak-to-peak of E after the first local min (rad)
    double end_W3;    // |third-axis rate| at the end
    double max_torque_ratio; // max |torque| / maxTorque over the run
};

// Simulate the axisymmetric body slewing to a FIXED target, from an initial
// third-axis spin, under the given law.
//   law 0 (old): torque along the slew axis only (the pre-fix law).
//   law 1 (new): drive the full transverse rate toward the braking-curve
//                direction, authority-bounded (the fix).
static SlewResult run3d(const Body &b, double h, double E0, double w3_init,
                        int law, int steps) {
    V3 t = v3(1.0, 0.0, 0.0);
    V3 n0 = v3(std::cos(E0), std::sin(E0), 0.0);
    // orthonormal frame, column 2 = nose
    V3 e2 = n0;
    V3 e0 = vnorm(v3(1.0, 0.0, 0.0) - e2 * dot(v3(1,0,0), e2));
    if(vlen2(e0) < 1e-12) e0 = v3(1,0,0);
    V3 e1 = cross(e2, e0);
    M3 R;
    R.m[0][0]=e0.x; R.m[1][0]=e0.y; R.m[2][0]=e0.z;
    R.m[0][1]=e1.x; R.m[1][1]=e1.y; R.m[2][1]=e1.z;
    R.m[0][2]=e2.x; R.m[1][2]=e2.y; R.m[2][2]=e2.z;

    // initial third-axis spin about b = a x n
    V3 a0 = vnorm(cross(n0, t));
    V3 b0 = vnorm(cross(a0, n0));
    V3 W = b0 * w3_init;

    double max_torque_ratio = 0.0;
    double end_W3 = 0.0;
    double prevE = 1e9, prevE2 = 1e9;
    bool saw_min = false;
    double min_at = 1e9, ring_max_after = 0.0;
    bool bad = false;
    double final_E = 1e9;

    for(int k = 0; k < steps; k++) {
        V3 n = v3(R.m[0][2], R.m[1][2], R.m[2][2]);
        double c = std::max(-1.0, std::min(1.0, dot(n, t)));
        double E = std::acos(c);
        final_E = E;
        V3 tau = v3(0,0,0);
        if(E > 1e-9) {
            V3 a = vnorm(cross(n, t));
            // Ieff = a . (Iw a)
            V3 Iwa = a * b.Ix + (n * dot(a, n)) * (b.Iz - b.Ix);
            double Ieff = dot(a, Iwa);
            double alpha = b.maxTorque / Ieff;
            double w_des = std::min(std::sqrt(2.0 * alpha * E), E / (2.0 * h));
            if(law == 0) {
                double w = dot(W, a);
                double A = alpha * h;
                double dw = std::max(-A, std::min(A, w_des - w));
                tau = a * (Ieff * dw / h);
            } else {
                V3 w_trans = W - n * dot(W, n);
                V3 dW = a * w_des - w_trans;
                V3 IwdW = dW * b.Ix + (n * dot(dW, n)) * (b.Iz - b.Ix);
                tau = IwdW / h;
                double tl = vlen(tau);
                if(tl > b.maxTorque) tau = tau * (b.maxTorque / tl);
            }
        }
        double ratio = vlen(tau) / b.maxTorque;
        if(ratio > max_torque_ratio) max_torque_ratio = ratio;

        V3 dW = euler_dW(b, n, W, tau);
        if(!std::isfinite(dW.x) || !std::isfinite(dW.y) || !std::isfinite(dW.z)) bad = true;
        W = W + dW * h;
        if(!std::isfinite(W.x) || !std::isfinite(W.y) || !std::isfinite(W.z)) bad = true;
        R = m3add(m3scale(matmul(skewm(W), R), h), R);
        R = reorthonormalize(R);

        // track the third-axis rate + the E-ring
        V3 n2 = v3(R.m[0][2], R.m[1][2], R.m[2][2]);
        double c2 = std::max(-1.0, std::min(1.0, dot(n2, t)));
        double E2 = std::acos(c2);
        V3 a2 = (E2 > 1e-9) ? vnorm(cross(n2, t)) : V3{0,0,0};
        V3 b2 = (E2 > 1e-9) ? vnorm(cross(a2, n2)) : V3{0,0,0};
        double w3 = std::fabs(dot(W, b2));
        end_W3 = w3;
        if(saw_min) {
            if(E2 > ring_max_after) ring_max_after = E2;
        } else if(E2 < prevE && E2 < prevE2) {
            saw_min = true; min_at = E2;
        }
        prevE2 = prevE; prevE = E2;
    }

    double E_ring = std::max(0.0, ring_max_after - min_at);
    return SlewResult{bad, final_E, E_ring, end_W3, max_torque_ratio};
}

int main() {
    // The racer, approx axisymmetric: Ix ~ 9400, Iz ~ 1300 kg m^2, one
    // 2000 N m wheel.
    Body b{9400.0, 1300.0, 2000.0};
    const double E0 = 0.30; // ~17 deg off prograde at engagement
    const double dt = 1.0/50.0;

    printf("== Slew 3-DOF: full-transverse law damps the third-axis spin ==\n");

    const double w3_grid[] = {0.05, 0.10, 0.20, 0.30, 0.50};
    const double warp_grid[] = {1.0, 10.0, 100.0};
    const int nw3 = (int)(sizeof(w3_grid)/sizeof(double));
    const int nwp = (int)(sizeof(warp_grid)/sizeof(double));

    // Helper: the per-config substep grid (mirrors how the game splits a
    // warp step into substeps), so the law sees the same h it sees in-game.
    // Returns h for a given warp; `steps` = 12 s of sim time.
    auto setup = [&](double warp, double &h, int &steps) {
        const double step = dt * warp;
        int n = 3; int need = (int)(step/0.1 + 0.5); if(need > n) n = need; if(n > 2000) n = 2000;
        h = step / n;
        steps = (int)(12.0 / h);
    };

    // 1. THE FIX: the NEW law damps the third-axis spin (end_W3 well below its
    //    initial value) in EVERY configuration, and converges to the target.
    //    This is the core observable difference from the OLD law, which leaves
    //    the spin undamped (it persists or grows). Threshold is relative
    //    (0.5x initial) so it is robust across the spin/warp grid.
    int new_runs = 0, new_damped = 0, new_converged = 0;
    double worst_ratio = 0.0;
    for(int i = 0; i < nw3; i++) {
        for(int j = 0; j < nwp; j++) {
            double w3 = w3_grid[i], warp = warp_grid[j];
            double h; int steps; setup(warp, h, steps);
            SlewResult r = run3d(b, h, E0, w3, 1, steps);
            char buf[192];
            snprintf(buf, sizeof buf,
                     "NEW law w3=%.2f warp=%gx: bad=%d finalE=%.2fdeg "
                     "endW3=%.4f (init %.2f)",
                     w3, warp, (int)r.bad, r.final_E*57.2958, r.end_W3, w3);
            CHECK_TRUE(!r.bad, buf);
            CHECK_TRUE(r.end_W3 < 0.5 * w3,
                       "NEW law must damp the third-axis spin (the fix)");
            if(r.end_W3 < 0.5 * w3) new_damped++;
            CHECK_TRUE(r.final_E < 0.05, "NEW law must converge to the target");
            if(r.final_E < 0.05) new_converged++;
            worst_ratio = std::max(worst_ratio, r.max_torque_ratio);
            new_runs++;
        }
    }
    printf("   (%d NEW-law configs: spin damped in %d, converged in %d)\n",
           new_runs, new_damped, new_converged);

    // 2. Authority bound: the NEW law never commands more than the wheel, so
    //    the autopilot is never more forceful than a maxed manual stick.
    CHECK_TRUE(worst_ratio <= 1.0 + 1e-9,
               "NEW-law torque must never exceed the wheel rating "
               "(autopilot <= maxed manual stick)");

    // 3. Settling in the REALISTIC range: the user's complaint is a visible
    //    wobble around the indicator. Stick-input residuals are well under
    //    0.5 rad/s, so for w3 <= 0.30 the NEW law must settle to a small
    //    E-ring (no sustained oscillation). The extreme w3=0.50 (a hard 28
    //    deg/s roll) is allowed to overshoot before converging -- it still
    //    damps the spin and reaches the target (covered by check 1).
    int realistic = 0, realistic_settled = 0;
    for(int i = 0; i < nw3; i++) {
        if(w3_grid[i] > 0.30) continue;
        for(int j = 0; j < nwp; j++) {
            double w3 = w3_grid[i], warp = warp_grid[j];
            double h; int steps; setup(warp, h, steps);
            SlewResult r = run3d(b, h, E0, w3, 1, steps);
            char buf[192];
            snprintf(buf, sizeof buf,
                     "NEW law w3=%.2f warp=%gx: E-ring=%.2fdeg (should settle)",
                     w3, warp, r.E_ring*57.2958);
            CHECK_TRUE(r.E_ring < 0.03, buf); // ~1.7 deg
            if(r.E_ring < 0.03) realistic_settled++;
            realistic++;
        }
    }
    printf("   (%d realistic-range configs settled: %d)\n",
           realistic, realistic_settled);

    // 4. Non-vacuous guard: the OLD law (slew-axis only) must NOT damp the
    //    third-axis spin in at least one configuration of the same grid --
    //    otherwise this harness cannot distinguish the two laws and the test
    //    is vacuous. (In practice the OLD law leaves it undamped in ALL of
    //    them; requiring >=1 keeps the guard robust to grid tweaks.)
    int old_no_damp = 0;
    for(int i = 0; i < nw3; i++) {
        for(int j = 0; j < nwp; j++) {
            double w3 = w3_grid[i], warp = warp_grid[j];
            double h; int steps; setup(warp, h, steps);
            SlewResult r = run3d(b, h, E0, w3, 0, steps);
            // "not damped" = the spin persists (>= half its initial value).
            if(r.end_W3 >= 0.5 * w3) old_no_damp++;
        }
    }
    printf("   (OLD law left the spin undamped in %d of %d configs)\n",
           old_no_damp, nw3 * nwp);
    CHECK_TRUE(old_no_damp > 0,
               "OLD law must leave the third-axis spin undamped in at least "
               "one configuration (if this passes, the harness cannot catch "
               "the third-axis bug)");

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if(g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("TESTS FAILED\n");
    return 1;
}
