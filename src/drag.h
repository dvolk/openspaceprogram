#pragma once

// drag.h -- the atmospheric-drag law as pure math (glm + <cmath> only), so tests/
// can pin it without the render chain.
//
// The air co-rotates with the planet: it is at rest in the body's ROTATING frame,
// so there the ship's frame velocity IS the air-relative velocity. In any other
// frame Vehicle::airRelativeVel subtracts the co-rotation term (#60/#97).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>  // controlSurfaceVisualRot

#include "constants.h"  // kRhoFloor

/* The physical half of a body's atmosphere (render half: AtmosphereParams,
   terragen.h). Both zero means "no drag". */
struct DragAtmosphere {
    double sea_level_density = 0.0;  // kg/m^3 at alt 0; 0 = no drag
    double scale_height = 0.0;       // [m]; density /e-fold altitude
    /* The hard top [m above sea level]: at or above it the air is vacuum.
       0 = no cutoff (the exponential tail runs down to kRhoFloor). */
    double height = 0.0;
    DragAtmosphere() {}
    DragAtmosphere(double rho0, double H, double top = 0.0)
        : sea_level_density(rho0), scale_height(H), height(top) {}
};

/* rho(alt) = sea_level_density · exp(−alt / scale_height), [kg/m^3], alt in m above
   the sea-level datum. At or above `height` the air stops dead; at or below 0 it is
   vacuum. */
inline double airDensity(const DragAtmosphere &a, double alt) {
    if(a.sea_level_density <= 0.0 || a.scale_height <= 0.0) { return 0.0; }
    if(alt <= 0.0) { return 0.0; }
    if(a.height > 0.0 && alt >= a.height) { return 0.0; }
    return a.sea_level_density * std::exp(-alt / a.scale_height);
}

/* F = −v̂ · ½ · rho(alt) · cd · area · |v_rel|²   (area [m^2], v_rel [m/s]).
   Zero for any degenerate input so callers need no guards. */
inline glm::dvec3 dragForce(const DragAtmosphere &a, double cd, double area,
                            double alt, const glm::dvec3 &v_rel) {
    const double rho = airDensity(a, alt);
    const double v = glm::length(v_rel);
    if(rho <= 0.0 || v <= 0.0 || cd <= 0.0 || area <= 0.0) {
        return glm::dvec3(0.0);
    }
    return -glm::normalize(v_rel) * (0.5 * rho * cd * area * v * v);
}

/* The aerodynamic flow frame: the ship's air-relative velocity decomposed
   against its body axes (x right, y up, z nose).

   ALPHA IS THE CONVENTIONAL ANGLE OF ATTACK: positive when the NOSE sits above
   the flight path. That is what every downstream law assumes -- liftCurve()
   signs the lift by alpha, so the sign chosen here decides whether a pitched-up
   wing lifts or drives the airframe into the ground. The flow comes AT the nose
   from ahead, so v_rel points BACKWARD: a nose above the flow has v_rel leaning
   DOWN, i.e. dot(v_rel, up) < 0, hence the negated up term. BETA needs no such
   flip -- v_rel leaning toward +right already means the wind arrives from the
   ship's right, which IS the conventional positive sideslip. */
struct AeroFrame {
    double v = 0.0;         // speed |v_rel|
    double alpha = 0.0;     // angle of attack, rad; + = nose above flight path
    double beta = 0.0;      // sideslip, rad; + = flow from the ship's right
    bool valid = false;     // false when v=0 or the nose axis is degenerate
};

inline AeroFrame aeroFrame(const glm::dvec3 &v_rel,
                           const glm::dvec3 &right, const glm::dvec3 &up,
                           const glm::dvec3 &nose) {
    AeroFrame f;
    const double v = glm::length(v_rel);
    if(v <= 0.0 || glm::length(nose) <= 0.0) { return f; }
    f.v = v;
    f.alpha = std::atan2(-glm::dot(v_rel, up), glm::dot(v_rel, nose));
    f.beta  = std::atan2(glm::dot(v_rel, right), glm::dot(v_rel, nose));
    f.valid = true;
    return f;
}

/* The inverse of those two angles: the unit flow direction that aeroFrame()
   would report as (alpha, beta). Exact everywhere except the corner where both
   reach +-90: the direction is built from their TANGENTS, and at the rim both are
   infinite -- only their ratio says which broadside direction it is, so that one
   point is ambiguous (SilhouetteMemo keeps its chart inside +-75 for the same
   reason, and answers the rim exactly). Zero for a degenerate frame. */
inline glm::dvec3 flowDirection(double alpha, double beta,
                                const glm::dvec3 &right, const glm::dvec3 &up,
                                const glm::dvec3 &nose) {
    if(glm::length(nose) <= 0.0) { return glm::dvec3(0.0); }
    // Taking dot(v, nose) = 1 fixes the scale, leaving the two tangents as the
    // other components; normalizing then undoes the choice.
    const glm::dvec3 v = nose + right * std::tan(beta) - up * std::tan(alpha);
    const double len = glm::length(v);
    return (len > 0.0) ? v / len : glm::dvec3(0.0);
}

/* q = 0.5 · rho(alt) · |v_rel|² -- the factor common to the lift and drag laws.
   Zero for any degenerate input. */
inline double dynamicPressure(const DragAtmosphere &a, double alt,
                              const glm::dvec3 &v_rel) {
    const double rho = airDensity(a, alt);
    if(rho <= 0.0) { return 0.0; }
    return 0.5 * rho * glm::dot(v_rel, v_rel);
}

/* Lift direction: the ship's up axis with its flow component removed
   (perpendicular to v_rel, on the ship's "up" side). */
inline glm::dvec3 liftDirection(const glm::dvec3 &v_rel,
                                const glm::dvec3 &right,
                                const glm::dvec3 &nose) {
    const double v = glm::length(v_rel);
    if(v <= 0.0 || glm::length(nose) <= 0.0 || glm::length(right) <= 0.0) {
        return glm::dvec3(0.0);
    }
    const glm::dvec3 up   = glm::cross(nose, right);   // the ship's +Y
    const glm::dvec3 vhat = v_rel / v;
    const glm::dvec3 l    = up - glm::dot(up, vhat) * vhat;  // up, out of flow
    const double len = glm::length(l);
    if(len <= 0.0) { return glm::dvec3(0.0); }
    return l / len;
}

/* CL(α) of a symmetric section, soft stall: linear to the stall angle, then a
   cosine droop to zero by 2·stallAngle. A = 0 disables the stall. CL stays
   continuous (no force jump). */
inline double liftCurve(double cl, double alpha, double stallAngle) {
    if(cl <= 0.0) { return 0.0; }
    const double a = std::fabs(alpha);
    const double A = stallAngle;
    double c;
    if(A <= 0.0 || a <= A) {
        c = cl * a;                                          // linear (up to stall)
    } else {
        const double t = (a - A) / A;                        // 0 at A, 1 at 2A
        c = (t >= 1.0) ? 0.0 : cl * A * std::cos(t * std::numbers::pi * 0.5);  // droop
    }
    return (alpha < 0.0) ? -c : c;                           // sign follows α
}

/* The lift force on one part: L = q · S · CL(α) · liftDir. */
inline glm::dvec3 liftForce(double q, double S, double cl, double alpha,
                            const glm::dvec3 &liftDir, double stallAngle = 0.0) {
    if(q <= 0.0 || S <= 0.0 || cl <= 0.0) { return glm::dvec3(0.0); }
    return liftDir * (q * S * liftCurve(cl, alpha, stallAngle));
}

/* The 2-D cross product (z-component) of (b-a) x (c-a). */
inline double cross2(const glm::dvec2 &a, const glm::dvec2 &b,
                     const glm::dvec2 &c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

/* The plane perpendicular to a view direction, as an orthonormal 2-D basis.
   Project a WHOLE ship through ONE of these when its parts' silhouettes are to
   be combined: if every part picks its own basis from its own direction, the
   polygons land in different planes and cannot be unioned (#463). */
struct FlowPlane {
    glm::dvec3 u{0.0}, w{0.0};
    bool valid = false;
};

inline FlowPlane flowPlane(const glm::dvec3 &dir) {
    const double len = glm::length(dir);
    if(len <= 0.0) { return FlowPlane(); }
    const glm::dvec3 d = dir / len;
    const glm::dvec3 ref = (std::fabs(d.x) < 0.9) ? glm::dvec3(1, 0, 0)
                                                  : glm::dvec3(0, 1, 0);
    FlowPlane pl;
    pl.u = glm::normalize(glm::cross(d, ref));
    pl.w = glm::cross(d, pl.u);
    pl.valid = true;
    return pl;
}

/* The silhouette POLYGON of a convex-hull vertex set seen edge-on: the hull of
   the projected vertices, which IS the silhouette -- so this stays right for a
   NON-convex mesh. Symmetric in the view direction. `out` is caller-owned
   scratch (this runs per part per physics substep); it ends up a CCW polygon,
   empty for a degenerate (collinear) projection. The output is CONVEX by
   construction -- unionProjectedArea is only correct for convex rings, because
   inside one slab it assumes two active edges cannot cross. */
inline void projectHull(const std::vector<glm::dvec3> &hullVerts,
                        const FlowPlane &pl, std::vector<glm::dvec2> &out) {
    out.clear();
    if(!pl.valid || hullVerts.size() < 3) { return; }
    out.reserve(hullVerts.size());
    for(const glm::dvec3 &v : hullVerts) {
        out.push_back(glm::dvec2(glm::dot(v, pl.u), glm::dot(v, pl.w)));
    }
    // Andrew's monotone chain: convex hull of the projected points (CCW,
    // collinear dropped). glm::dvec2 has no operator<, hence the explicit
    // x-then-y comparator.
    std::sort(out.begin(), out.end(),
              [](const glm::dvec2 &a, const glm::dvec2 &b) {
                  return (a.x < b.x) || (a.x == b.x && a.y < b.y);
              });
    static thread_local std::vector<glm::dvec2> h;
    h.clear();
    for(size_t i = 0; i < out.size(); i++) {
        while(h.size() >= 2
              && cross2(h[h.size() - 2], h[h.size() - 1], out[i]) <= 0.0) {
            h.pop_back();
        }
        h.push_back(out[i]);
    }
    const size_t lowerSize = h.size() + 1;
    for(int i = (int)out.size() - 1; i >= 0; i--) {
        while(h.size() >= lowerSize
              && cross2(h[h.size() - 2], h[h.size() - 1], out[i]) <= 0.0) {
            h.pop_back();
        }
        h.push_back(out[i]);
    }
    h.pop_back();                              // drop the duplicated start point
    out.swap(h);                               // out keeps the hull, h the buffer
}

/* One PART's silhouette in a flow plane shared by the whole ship: its collision
   verts through its MOUNT (rotation and position) into that one 2-D basis.
   A part's area alone is translation-invariant, which is why the mount position
   used to be ignorable; it stops being ignorable the moment two parts' polygons
   are unioned (#463), so this is the only projection a ship should use. */
inline void projectPartHull(const std::vector<glm::dvec3> &hullVerts,
                            const glm::mat3 &localRot, const glm::dvec3 &localPos,
                            const FlowPlane &pl, std::vector<glm::dvec2> &out) {
    // Rotating the basis into the part costs two mat3-vec multiplies and keeps
    // the ship's basis shared; the alternative is rotating every vert (2x the
    // parts' vert count) into the ship frame.
    const glm::mat3 toLocal = glm::transpose(localRot);
    projectHull(hullVerts, FlowPlane{toLocal * pl.u, toLocal * pl.w, pl.valid}, out);
    const glm::dvec2 off{glm::dot(localPos, pl.u), glm::dot(localPos, pl.w)};
    for(glm::dvec2 &q : out) { q += off; }
}

/* Shoelace area of a simple polygon (either winding). Zero for a degenerate
   (fewer than three points, or collinear) one. */
inline double polygonArea2(const std::vector<glm::dvec2> &poly) {
    if(poly.size() < 3) { return 0.0; }
    double a = 0.0;
    for(size_t i = 0; i < poly.size(); i++) {
        const size_t j = (i + 1) % poly.size();
        a += poly[i].x * poly[j].y - poly[j].x * poly[i].y;
    }
    return 0.5 * std::fabs(a);
}

/* The projected (silhouette) area of ONE body seen along `dir`, from its CONVEX
   HULL vertices. */
inline double projectedArea(const std::vector<glm::dvec3> &hullVerts,
                            const glm::dvec3 &dir) {
    static thread_local std::vector<glm::dvec2> poly;
    projectHull(hullVerts, flowPlane(dir), poly);
    return polygonArea2(poly);
}

/* The area of the UNION of a set of convex polygons -- the honest silhouette of
   a ship made of separated solids (#463). Convex-hulling every part's vertices
   together instead draws a skin over the gaps: on the shipped jet that turns two
   0.4 m^2 plates into 12.2 m^2 of wing that is not there, and the tank-to-nose
   stack into more fuselage than there is.

   Exact by vertical decomposition: cut the plane at every vertex x and every
   place two polygons' edges cross. Within one such slab the active edges are
   fixed and no two of them cross, so each merged interval's height is linear in
   x, which makes the merged measure linear in x too -- its value at the slab
   midpoint times the slab width is therefore exact, not a sample.

   Costs edge-pair work per direction, so it belongs at cache-node time, not in
   the substep loop; see SilhouetteMemo. */
inline double unionProjectedArea(const std::vector<std::vector<glm::dvec2>> &polys) {
    struct Span { size_t poly; double lo, hi; };
    static thread_local std::vector<Span> spans;
    static thread_local std::vector<double> xs;
    static thread_local std::vector<std::pair<double, double>> bands;
    spans.clear();
    xs.clear();
    for(size_t i = 0; i < polys.size(); i++) {
        if(polys[i].size() < 3) { continue; }
        Span sp{i, polys[i][0].x, polys[i][0].x};
        for(const glm::dvec2 &v : polys[i]) {
            if(v.x < sp.lo) { sp.lo = v.x; }
            if(v.x > sp.hi) { sp.hi = v.x; }
        }
        spans.push_back(sp);
        for(const glm::dvec2 &v : polys[i]) { xs.push_back(v.x); }
    }
    if(spans.empty()) { return 0.0; }
    // Polygon-pair edge crossings: the only places the merged measure can change
    // curvature. The x-range tests reject most pairs of a ship's hulls cheaply.
    for(size_t a = 0; a < spans.size(); a++) {
        for(size_t b = a + 1; b < spans.size(); b++) {
            const std::vector<glm::dvec2> &pa = polys[spans[a].poly];
            const std::vector<glm::dvec2> &pb = polys[spans[b].poly];
            const double xlo = std::max(spans[a].lo, spans[b].lo);
            const double xhi = std::min(spans[a].hi, spans[b].hi);
            if(!(xlo < xhi)) { continue; }
            for(size_t i = 0; i < pa.size(); i++) {
                const glm::dvec2 &p = pa[i], &q = pa[(i + 1) % pa.size()];
                const double alo = std::min(p.x, q.x), ahi = std::max(p.x, q.x);
                if(ahi <= xlo || alo >= xhi) { continue; }
                for(size_t j = 0; j < pb.size(); j++) {
                    const glm::dvec2 &r = pb[j], &s = pb[(j + 1) % pb.size()];
                    // Prune against THIS edge's x-range, not just the two
                    // polygons' overlap -- an order of magnitude more pairs die here.
                    if(std::max(r.x, s.x) <= alo || std::min(r.x, s.x) >= ahi) { continue; }
                    const double d1x = q.x - p.x, d1y = q.y - p.y;
                    const double d2x = s.x - r.x, d2y = s.y - r.y;
                    const double den = d1x * d2y - d1y * d2x;
                    if(den == 0.0) { continue; }      // parallel: no crossing
                    const double t = ((r.x - p.x) * d2y - (r.y - p.y) * d2x) / den;
                    if(t < 0.0 || t > 1.0) { continue; }
                    const double u = ((r.x - p.x) * d1y - (r.y - p.y) * d1x) / den;
                    if(u < 0.0 || u > 1.0) { continue; }
                    xs.push_back(p.x + t * d1x);
                }
            }
        }
    }
    std::sort(xs.begin(), xs.end());
    // Unique to a relative epsilon. Every vertex x is already in the list
    // exactly, so this only merges a crossing that landed a rounding hair off an
    // endpoint. Scaled by the coordinate magnitude rather than the whole ship's
    // span, so a small part's own breakpoints cannot be swallowed by the ship
    // being large next to it.
    const double scale = std::max(std::fabs(xs.front()), std::fabs(xs.back()));
    const double eps = 1e-12 * (scale > 1.0 ? scale : 1.0);
    size_t n = 0;
    for(size_t i = 0; i < xs.size(); i++) {
        if(n == 0 || xs[i] - xs[n - 1] > eps) { xs[n++] = xs[i]; }
    }
    double area = 0.0;
    for(size_t k = 0; k + 1 < n; k++) {
        const double x0 = xs[k], x1 = xs[k + 1];
        if(!(x1 > x0)) { continue; }
        const double xm = 0.5 * (x0 + x1);
        // Where the vertical line at xm cuts each polygon that spans it.
        bands.clear();
        for(const Span &sp : spans) {
            if(xm < sp.lo || xm > sp.hi) { continue; }
            const std::vector<glm::dvec2> &poly = polys[sp.poly];
            double lo = 0.0, hi = 0.0;
            bool hit = false;
            for(size_t i = 0; i < poly.size(); i++) {
                const glm::dvec2 &p = poly[i], &q = poly[(i + 1) % poly.size()];
                // Half-open so a vertex shared by two edges counts once. An edge
                // lying ON xm cannot reach here: xm is a slab MIDPOINT and every
                // vertex x is a breakpoint, so no vertex sits at xm -- which also
                // means a straddling edge has p.x != q.x and the divide is safe.
                if((p.x <= xm && q.x > xm) || (q.x <= xm && p.x > xm)) {
                    const double t = (xm - p.x) / (q.x - p.x);
                    const double y = p.y + t * (q.y - p.y);
                    if(!hit) { lo = hi = y; hit = true; }
                    else {
                        if(y < lo) { lo = y; }
                        if(y > hi) { hi = y; }
                    }
                }
            }
            if(hit && hi > lo) { bands.push_back({lo, hi}); }
        }
        if(bands.empty()) { continue; }
        std::sort(bands.begin(), bands.end());
        double measure = 0.0;
        double clo = bands[0].first, chi = bands[0].second;
        for(size_t i = 1; i < bands.size(); i++) {
            if(bands[i].first > chi) {
                measure += chi - clo;
                clo = bands[i].first;
                chi = bands[i].second;
            } else if(bands[i].second > chi) {
                chi = bands[i].second;
            }
        }
        measure += chi - clo;
        area += measure * (x1 - x0);
    }
    return area;
}

/* The silhouette area as a function of the flow DIRECTION only (#463). Speed and
   density multiply the force, never its shape, so the union above is worth
   computing once per direction rather than once per substep: measured on the
   shipped jet, a union pass costs about 0.5 ms over its 13 hulls (3.3k verts),
   and the convex-hull balloon it replaces cost 1.7 ms at the 33k-vert balloon.

   The table is a grid over the two angles AeroFrame already produces -- alpha
   (flow off the nose, pitch plane) and beta (the same, yaw plane) -- read
   bilinear between its four surrounding nodes. Nodes are NOT computed up front:
   they fill the first time a ship's flow enters the cell, so a parked or
   cruising ship pays only for the handful of directions it actually flies, and
   a direction sweep spreads across the flight instead of landing in one
   rebuild. The whole table goes stale by bumping a generation counter; nothing
   is cleared.

   A cell whose corners are not all filled reports a MISS and the caller uses the
   exact area for the direction it is already flying -- which the per-part
   projections it computes anyway give for free. The cache is therefore never a
   cost: the worst case equals the uncached law, and filling is pure saving. */
struct SilhouetteMemo {
    // 5 deg nodes over +-75 deg in both angles. kFoldHalf is the atan2 chart's
    // own bound: past +-90 the flow comes from behind, and a silhouette is
    // symmetric in the view direction, so those directions fold onto this chart
    // (see fold). The cached range stops short of that bound on purpose -- see
    // the note on kHalf.
    static constexpr int kSteps = 31;
    static constexpr double kFoldHalf = std::numbers::pi_v<double> / 2.0;
    // The angles parameterise a direction as tan(alpha), tan(beta), which blows up
    // at 90 deg, so cells near +-90 span an enormous slice of direction space and
    // blending across them is poor -- at the rim the parameterisation is not merely
    // coarse but ambiguous (see flowDirection). Measured over the folded sphere,
    // 31 nodes: worst blend error 0.7% for a smooth silhouette, 5.0% for a
    // plate-dominated one (a wing-led ship is the sharp case) with this +-75
    // range; the +-90 range Stage 1 first shipped measures 6.9%. So the cached
    // range stops at 75 deg: it holds the blend error to a few percent and refuses
    // the rim, where the extreme attitudes MISS and the caller's exact union --
    // which it can compute anyway -- answers them instead.
    static constexpr double kHalf = 75.0 * std::numbers::pi_v<double> / 180.0;
    static constexpr double kStep = (2.0 * kHalf) / (kSteps - 1);

    std::vector<float> area;       // [m^2]; node (i, j) at j * kSteps + i
    std::vector<uint32_t> stamp;   // the generation the node was filled at
    uint32_t gen = 0;              // bump to invalidate every node at once

    void reset() {
        area.assign(kSteps * kSteps, 0.0f);
        stamp.assign(kSteps * kSteps, 0u);
        gen = 1;                   // 0 means "never filled"
    }
    // Drop the table without allocating. A ship with no parts has no silhouette
    // to remember: a filled node outliving its hull would charge drag for a body
    // that is not there, and the old balloon law could not do that because an
    // empty hull projected to zero area.
    void clear() {
        area.clear();
        stamp.clear();
        gen = 0;
    }
    void invalidate() {
        if(area.empty()) { reset(); }
        else { gen++; }
    }
    bool empty() const { return area.empty(); }

    static double alphaAt(int i) { return -kHalf + i * kStep; }
    static double betaAt(int j) { return -kHalf + j * kStep; }

    /* AeroFrame's angles come from atan2 over the SAME denominator, dot(v, nose),
       so a flow from behind the nose lands past +-90 in BOTH at once. A silhouette
       is symmetric in the view direction, so the back hemisphere folds onto this
       chart by REVERSING the flow: each angle wraps by 180 under its own sign.
       That reversal is the exact symmetry (a projected point set seen from the
       other side is its mirror image -- same area, same overlaps). Note it is not
       180 - alpha, which would mirror the ship's left and right and charge a
       broadside for a quartering one. Wrapping only alpha keys a straight-tail
       flow as a quartering one, and leaving beta un-wrapped was the bug: a
       straight-down flow landed in the straight-up row and a straight-tail flow
       in the quartering one. The clamps are insurance for a hand-fed pair. */
    static void fold(double &alpha, double &beta) {
        if(alpha > kFoldHalf) { alpha -= std::numbers::pi; }
        else if(alpha < -kFoldHalf) { alpha += std::numbers::pi; }
        if(beta > kFoldHalf) { beta -= std::numbers::pi; }
        else if(beta < -kFoldHalf) { beta += std::numbers::pi; }
        if(alpha > kFoldHalf) { alpha = kFoldHalf; }
        else if(alpha < -kFoldHalf) { alpha = -kFoldHalf; }
        if(beta > kFoldHalf) { beta = kFoldHalf; }
        else if(beta < -kFoldHalf) { beta = -kFoldHalf; }
    }

    // The cell index whose lower corner is at or below `angle` (angle folded).
    static int cellOf(double angle) {
        int i = (int)std::floor((angle + kHalf) / kStep);
        if(i < 0) { i = 0; }
        if(i > kSteps - 2) { i = kSteps - 2; }
        return i;
    }
    static double weight(double angle, int i0) {
        const double f = (angle + kHalf) / kStep - i0;
        if(f < 0.0) { return 0.0; }
        if(f > 1.0) { return 1.0; }
        return f;
    }
    // Inside the cached range (both angles already folded).
    static bool charted(double alpha, double beta) {
        return std::fabs(alpha) <= kHalf && std::fabs(beta) <= kHalf;
    }

    /* A hit: all four corners of the cell were filled at this generation, so
       bilinear between them. Anything else (no table, a NaN angle, a direction
       outside the cached range, a corner not yet filled) is a MISS and leaves
       `out` alone. */
    bool lookup(double alpha, double beta, double &out) const {
        if(area.empty() || !std::isfinite(alpha) || !std::isfinite(beta)) { return false; }
        fold(alpha, beta);
        if(!charted(alpha, beta)) { return false; }
        const size_t k00 = (size_t)cellOf(beta) * kSteps + cellOf(alpha);
        if(stamp[k00] != gen || stamp[k00 + 1] != gen
           || stamp[k00 + kSteps] != gen || stamp[k00 + kSteps + 1] != gen) {
            return false;
        }
        const int i0 = cellOf(alpha), j0 = cellOf(beta);
        const double wa = weight(alpha, i0), wb = weight(beta, j0);
        const double a00 = area[k00], a10 = area[k00 + 1];
        const double a01 = area[k00 + kSteps], a11 = area[k00 + kSteps + 1];
        out = (a00 * (1.0 - wa) + a10 * wa) * (1.0 - wb)
            + (a01 * (1.0 - wa) + a11 * wa) * wb;
        return true;
    }

    /* Fill this cell's missing corners, at most `budget` of them -- the caller
       spreads the cost over substeps. `exact(alpha, beta)` computes the area at
       a node; the caller holds the parts, the ship frame and the union. Returns
       how many nodes were filled. A direction outside the cached range fills
       nothing: it is answered by the caller's exact union instead. */
    template <class ExactFn>
    int fillAround(double alpha, double beta, int budget, ExactFn exact) {
        if(budget <= 0 || area.empty()
           || !std::isfinite(alpha) || !std::isfinite(beta)) {
            return 0;
        }
        fold(alpha, beta);
        if(!charted(alpha, beta)) { return 0; }
        const int i0 = cellOf(alpha), j0 = cellOf(beta);
        int filled = 0;
        for(int dj = 0; dj <= 1; dj++) {
            for(int di = 0; di <= 1; di++) {
                if(budget <= 0) { return filled; }
                const int i = i0 + di, j = j0 + dj;
                const size_t k = (size_t)j * kSteps + i;
                if(stamp[k] == gen) { continue; }
                area[k] = (float)exact(alphaAt(i), betaAt(j));
                stamp[k] = gen;
                filled++;
                budget--;
            }
        }
        return filled;
    }
};

/* The drag that LIFT costs (#462): induced drag coefficient = CL^2 / (pi e AR),
   with AR = span^2 / planform area. Nothing charged for it before: a surface
   earned its lift free, paying only whatever blunt form drag its shape happened
   to draw. `span` must be the SPAN, not the thickness -- for a plate part,
   PartDef::radius is half the largest x/y extent of the mesh, which for a wing
   IS its half-span (#457 wants an honest per-part box; until then that is the
   only span the catalog states).

   Known overcharge until #457: AR is computed PER PART, so a lifting surface
   modelled as two half-plates (the shipped jet's wings, mounted at angle 0 and
   180) pays two AR-2 systems instead of one 8 m AR-4 system -- twice the induced
   drag. Half-wings are genuinely separate lifting bodies only if they are
   separated; co-planar halves share one span. The extents #457 emits are what
   tells those cases apart. */
inline double inducedCd(double cl, double span, double area, double oswald = 0.8) {
    if(!(area > 0.0) || !(span > 0.0) || !(oswald > 0.0)) { return 0.0; }
    const double ar = (span * span) / area;
    if(!(ar > 0.0)) { return 0.0; }
    return (cl * cl) / (std::numbers::pi * oswald * ar);
}

/* The per-part DRAG COEFFICIENT as the part faces the flow: a blend of the three
   declared anchors (nose / side / base) by c = cos(nose axis, flow):
     cd(c) = drag_side·(1−c²) + drag_forward·(max(c,0)²) + drag_backward·(max(−c,0)²)
   Convex blend: hits each anchor exactly, stays within the part's min..max cd. */
inline double partCd(double cdForward, double cdSide, double cdBackward,
                     double cosAxisFlow) {
    double c = cosAxisFlow;
    if(c > 1.0) { c = 1.0; }
    else if(c < -1.0) { c = -1.0; }
    const double c2 = c * c;
    const double cw = (c > 0.0) ? c2 : 0.0;   // forward weight (nose-in)
    const double bw = (c < 0.0) ? c2 : 0.0;   // backward weight (base-in)
    return cdSide * (1.0 - c2) + cdForward * cw + cdBackward * bw;
}

/* The force on one DEFLECTED control surface: the lift law with the DEFLECTION
   in place of the angle of attack (LINEAR -- no stall; travel bounds it). The
   steering moment is NOT here: it comes from applying this force AT the surface
   (Vehicle::applyAeroForce). */
inline glm::dvec3 controlForce(double q, double S, double cl, double delta,
                               const glm::dvec3 &dir) {
    if(q <= 0.0 || S <= 0.0 || cl <= 0.0 || delta == 0.0) { return glm::dvec3(0.0); }
    return dir * (q * S * cl * delta);
}

/* The control-surface DEFLECTION EFFECTIVENESS: the part's dedicated cl_control
   if set (>0), else its lift-curve slope cl. */
inline double controlCl(double cl, double clControl) {
    return (clControl > 0.0) ? clControl : cl;
}

/* The jet AIR GATE shared by thrust and fuel flow: available air as a fraction
   of the body's own sea level, capped at 1. The tank drains with it -- thin air
   gives less thrust AND less burn, and vacuum gives neither. */
inline double jetAirGate(double rho, double rho_sea) {
    if(!(rho > 0.0) || !(rho_sea > 0.0)) { return 0.0; }  // NaN fails closed
    const double d = rho / rho_sea;
    return (d >= 1.0) ? 1.0 : d;
}

/* JET ENGINE (air-breathing) thrust, newtons:
     T(v, rho) = T_fan·d  +  ṁ_f·v_e·d  +  rho·A·v·(v_e − v)   d = jetAirGate(rho, rho_sea)
   d gates the fan + fuel terms on available air; rho = 0 kills every term. The net
   is clamped at 0 (a jet never reverses). */
inline double jetThrust(double v, double rho, double rho_sea,
                        double T_fan, double m_f, double v_e, double A) {
    const double d = jetAirGate(rho, rho_sea);
    if(d <= 0.0) { return 0.0; }   /* no air -> no thrust */
    const double speed = (v < 0.0) ? 0.0 : v;
    const double T = T_fan * d + m_f * v_e * d + rho * A * speed * (v_e - speed);
    return (T < 0.0) ? 0.0 : T;
}

/* The control-surface DEFLECTION SIGN for a surface at `ri` (rel. to the COM). The
   torque is ri x F, so a tail (behind the CG) and a canard (ahead) need OPPOSITE
   deflections for the same torque; this matches the reaction wheel's convention
   either way. `targetSign` is the SIGN of the wheel's torque about that axis for a
   positive stick (pitch/yaw = -1, roll = +1). */
inline double controlDeflectionSign(const glm::dvec3 &ri,
                                    const glm::dvec3 &forceDir,
                                    const glm::dvec3 &aboutAxis,
                                    double targetSign = -1.0) {
    const double d = glm::dot(glm::cross(ri, forceDir), aboutAxis);
    return (d >= 0.0) ? targetSign : -targetSign;
}

/* The VISUAL hinge of a control-surface mesh at commanded deflection defl
   (signed rad, the control law's sign; Vehicle::Draw only). The visual hinge
   is NOT the torque axis (aboutAxis): a surface hinged at angle t produces
   force along t x nose, so to bend the same way the law's defl does --
   - a rudder (Yaw) tips +defl about UP,
   - an elevator or aileron tips -defl about RIGHT (aboutAxis 2 hinges on
     right, not nose: ailerons FLAP, they do not sweep).
   controlDeflectionSign already gives left/right surfaces opposite defl, so
   this yields differential aileron flapping for free. Assumes the mesh's
   chord runs along the part's nose axis, which is the surface-mount frame
   gen_wing.py authors plates in (res/meshes/fin_r1h2.obj). */
inline glm::dquat controlSurfaceVisualRot(int aboutAxis, double defl,
                                          const glm::dmat3 &rootRot) {
    return (aboutAxis == 1) ? glm::angleAxis(defl, rootRot[1])
                            : glm::angleAxis(-defl, rootRot[0]);
}
