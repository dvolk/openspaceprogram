#pragma once
// orbitmap.h -- orthographic top-down projection + drawing for the orbital map.
// Drops the out-of-plane component, so inclined orbits look squashed.

#include <algorithm>
#include <cassert>
#include <cmath>

#include <glm/glm.hpp>

#include <vector>

#include <imgui.h>

struct OrbitMap {
    double cx = 200.0, cy = 200.0;  // screen position of the focus
    double scale = 6000.0;          // meters per pixel
    // Map plane in the focus's inertial frame; default is the body-rail plane.
    glm::dvec3 n  = glm::dvec3(0.0, 1.0, 0.0);
    glm::dvec3 e1 = glm::dvec3(1.0, 0.0, 0.0);
    glm::dvec3 e2 = glm::dvec3(0.0, 0.0, 1.0);

    // `x_axis` (optional) pins the screen-x direction INSIDE the plane: every
    // combo slot passes the focus frame's +X, so flipping the plane combo
    // tilts the picture instead of rotating it (#173, #185), and a normal
    // approaching the rail normal keeps a stable basis instead of handing over
    // to the near-polar branch below. Zero length means derive it from the node
    // line (only the no-ship default and the tests do that).
    // A reference direction ALONG the normal cannot define an east at all: its
    // in-plane part is zero, so the floor below is a NaN guard, not a
    // conditioning choice.
    // No choice of screen-x escapes the singularity itself -- e1 has to flip
    // sign as the normal crosses the reference axis -- but pinning to +X keeps
    // it off the near-equatorial orbits the node line's circle crosses (#185).
    // Handedness, the same on all three paths below: e2 = e1 x n, so e1 x e2
    // = -n and a prograde body (which moves along n x r_hat) sits at +e1
    // moving along -e2 -- counter-clockwise on screen, whichever plane is
    // showing. Pinned by test_orbitmap; #181 broke it.
    void setPlane(const glm::dvec3 &normal,
                  const glm::dvec3 &x_axis = glm::dvec3(0.0)) {
        n = glm::normalize(normal);
        const glm::dvec3 t = x_axis - glm::dot(x_axis, n) * n;  // in-plane part
        // t == 0 when x_axis is collinear with n (including the zero default),
        // where normalize would return NaN; those go to the branches below.
        if(glm::length(t) > 1e-9) {
            e1 = glm::normalize(t);
            e2 = glm::cross(e1, n);
            return;
        }
        if(glm::abs(glm::dot(n, glm::dvec3(0.0, 1.0, 0.0))) > 0.99) {
            // Only reachable with no usable x_axis. The derived basis below
            // degenerates here (|Y x n| tends to 0), so keep a fixed screen
            // basis, with e2 following the sign of n so that e1 x e2 = -n
            // still holds: e1 x n = X x (0,+-1,0) = +-Z. That sign matters for
            // any normal below the rail plane (a retrograde rail, or a
            // retrograde orbit) -- #181.
            // Safe to branch on the sign: a normal cannot get from +Y to -Y
            // without passing |n.y| < 0.99, which takes the derived branch, so
            // this never mirrors a picture mid-slew.
            e1 = glm::dvec3(1.0, 0.0, 0.0);
            e2 = n.y < 0.0 ? glm::dvec3(0.0, 0.0, -1.0)
                           : glm::dvec3(0.0, 0.0, 1.0);
        } else {
            // Derived basis: the node line between the plane and the rail
            // plane as screen-x. Same e2 = e1 x n as the paths above (#181).
            e1 = glm::normalize(glm::cross(glm::dvec3(0.0, 1.0, 0.0), n));
            e2 = glm::cross(e1, n);
        }
    }

    glm::dvec2 project(const glm::dvec3 &p) const {
        return glm::dvec2(cx + glm::dot(p, e1) / scale,
                          cy + glm::dot(p, e2) / scale);
    }

    ImVec2 px(const glm::dvec3 &p) const {
        const glm::dvec2 q = project(p);
        return ImVec2(float(q.x), float(q.y));
    }

    // `start` rotates the point order so a caller can begin/end at a marker
    // between two samples.
    void drawOrbit(ImDrawList *dl, const std::vector<glm::dvec3> &pts,
                   ImU32 col, float thickness = 1.0f, bool closed = true,
                   size_t start = 0) const {
        const size_t n = pts.size();
        if(n < 2) { return; }
        // Reused: hundreds of orbits per frame made the per-call vector
        // measurable allocator traffic.
        thread_local std::vector<ImVec2> sp;
        sp.clear();
        sp.reserve(n);
        for(size_t j = 0; j < n; j++) { sp.push_back(px(pts[(start + j) % n])); }
        const ImDrawFlags flags =
            closed ? ImDrawFlags_Closed : ImDrawFlags_None;
        dl->AddPolyline(sp.data(), (int)sp.size(), col, thickness, flags);
    }

    void drawDot(ImDrawList *dl, const glm::dvec3 &p, float r_px,
                 ImU32 col) const {
        dl->AddCircleFilled(px(p), r_px, col);
    }

    // Floored at min_px so a body stays visible when zoomed out to sub-pixel.
    float bodyRadiusPx(double radius_m, float min_px) const {
        return fmaxf(min_px, (float)(radius_m / scale));
    }

    void drawBody(ImDrawList *dl, const glm::dvec3 &pos, double radius_m,
                  ImU32 col, float min_px = 0.0f) const {
        dl->AddCircleFilled(px(pos), bodyRadiusPx(radius_m, min_px), col);
    }

    // A sphere projects to a circle of the same radius in any plane.
    void drawRing(ImDrawList *dl, const glm::dvec3 &center, double radius_m,
                  ImU32 col, float thickness = 1.0f) const {
        dl->AddCircle(px(center), float(radius_m / scale), col, 0, thickness);
    }

    // Direction arrow of fixed pixel length; out-of-plane component dropped.
    // Nothing drawn if the direction is purely out of plane.
    void drawArrow(ImDrawList *dl, const glm::dvec3 &origin,
                   const glm::dvec3 &dir, float len_px, ImU32 col,
                   float thickness = 1.5f) const {
        const glm::dvec3 dp = dir - glm::dot(dir, n) * n;
        const double dl_len = glm::length(dp);
        if(dl_len < 1e-9) { return; }
        const glm::dvec2 d2(glm::dot(dp, e1) / dl_len,
                            glm::dot(dp, e2) / dl_len);
        const ImVec2 from = px(origin);
        const ImVec2 to(from.x + d2.x * len_px, from.y + d2.y * len_px);
        dl->AddLine(from, to, col, thickness);
        const float ah = 6.0f;                       // arrowhead size (px)
        const float a  = atan2f(d2.y, d2.x);
        dl->AddLine(to, ImVec2(to.x - ah * cosf(a + 0.5f),
                               to.y - ah * sinf(a + 0.5f)), col, thickness);
        dl->AddLine(to, ImVec2(to.x - ah * cosf(a - 0.5f),
                               to.y - ah * sinf(a - 0.5f)), col, thickness);
    }
};

// Near-black or near-white that contrasts with the window background
// (Rec. 709 luminance, 0.5 threshold).
inline ImU32 contrastingColor(const ImVec4 &bg,
                              const ImVec4 &dark  = ImVec4(0.08f, 0.08f, 0.08f, 1.0f),
                              const ImVec4 &light = ImVec4(0.95f, 0.95f, 0.95f, 1.0f)) {
    const float lum = 0.2126f * bg.x + 0.7152f * bg.y + 0.0722f * bg.z;
    const ImVec4 c = (lum > 0.5f) ? dark : light;
    // Pack 0xAABBGGRR directly so this stays usable in the link-free unit test.
    const int r = (int)(c.x * 255.0f + 0.5f);
    const int g = (int)(c.y * 255.0f + 0.5f);
    const int b = (int)(c.z * 255.0f + 0.5f);
    const int a = (int)(c.w * 255.0f + 0.5f);
    return (ImU32)(((a & 0xFF) << 24) | ((b & 0xFF) << 16) |
                   ((g & 0xFF) << 8) | (r & 0xFF));
}

// A gray `v` of luminance OFF THE CANVAS, always away from it: bg + v on a dark
// canvas, bg - v on a light one, so a quiet rail is equally quiet on either theme --
// a hardcoded dark gray vanishes on the light style. The canvases this game actually
// draws on are 0.00 (the Tracking map pushes black, and the Classic style's WindowBg),
// 0.06 (the dark theme) and 0.94 (the light style), so the v that gameui.cpp calls
// kMinorQuiet lands on one gray per canvas -- the same offset, not the same gray. Same
// Rec. 709 test on gamma-encoded style values as contrastingColor(), and the same
// fiction in the flight map's chrome-less mode 2, where the rails sit on the 3D scene.
// Returns an ImVec4 instead of packing it, so the caller's GetColorU32 applies
// style.Alpha as it does for every other map colour.
inline ImVec4 quietRgba(const ImVec4 &bg, float v) {
    assert(v >= 0.0f);
    const float lum = 0.2126f * bg.x + 0.7152f * bg.y + 0.0722f * bg.z;
    const float d = (lum > 0.5f) ? -v : v;   // away from the canvas: down on light, up on dark
    const auto ch = [d](float c) {
        const float x = c + d;
        return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
    };
    return ImVec4(ch(bg.x), ch(bg.y), ch(bg.z), 1.0f);
}

// --- Map names ------------------------------------------------------------------
// A name waiting to be placed: where its marker is, how big the text is (measured by
// the caller -- there is no font here), and how much it matters when space is
// contested. Separated from the drawing so the rule below is testable without a
// window, a font atlas or a Game.
struct MapLabelCand {
    ImVec2 dot;       // the marker the name belongs to
    ImVec2 size;      // text extent
    float r_px;       // that marker's radius, for the air gap
    unsigned char prio;   // lower claims space first
};
struct MapLabelPlaced { int cand; ImVec2 at; };   // index into items; top-left to draw at

// Where a name may sit around its marker, in preference order: right (how map names
// read today), then left -- which is also what keeps a name whole for a body against
// the right edge -- then below and above, for a body whose east and west are taken.
// The vertical slots start clear of the horizontal row, not just clear of the marker:
// a 12 px name centred on the dot reaches 6 px below it, so a below slot measured only
// from the marker would sit on top of the right-hand name.
inline void mapLabelSlots(const MapLabelCand &c, float pad, ImVec2 out[4]) {
    const float g = c.r_px + pad;
    out[0] = ImVec2(c.dot.x + g,              c.dot.y - c.size.y * 0.5f);
    out[1] = ImVec2(c.dot.x - g - c.size.x,   c.dot.y - c.size.y * 0.5f);
    out[2] = ImVec2(c.dot.x - c.size.x * 0.5f, c.dot.y + g + c.size.y * 0.5f);
    out[3] = ImVec2(c.dot.x - c.size.x * 0.5f, c.dot.y - g - c.size.y * 1.5f);
}

// Only one name may sit on a patch of map, and no name may sit on a body: candidates
// claim space in priority order -- whole, inside `view` (l,t,r,b), at most max_drawn of
// them -- and the rest are dropped. Without this the shipped catalogue prints every
// body on screen at once: ~440 names overlapping at a planning zoom, where the belts
// and the big planets' moons sit on each other. And a name printed across a dot makes
// that dot unidentifiable, so every marker (whether its name survives or not) plus
// anything in `blocks` reserves its own space first.
//
// A loser is dropped, not slid to a free row: a name a line away from its marker can
// read as belonging to whichever dot is nearest, which is a worse map than one with
// fewer names. `items` may be any type carrying a `.cand` member (the game's label pass
// keeps colour and text beside it); the result indexes `items`.
//
// `cell` bounds the overlap tests with a coarse occupancy grid: the catalogue is 670
// bodies and the plan is to scale that up, where testing every box against every other
// is ~0.4 M comparisons per window per frame.
class LabelGrid {
public:
    void reset(const ImVec4 &view, float cell) {
        x0_ = view.x; y0_ = view.y; cell_ = cell;
        cols_ = (int)((view.z - view.x) / cell) + 2;
        rows_ = (int)((view.w - view.y) / cell) + 2;
        heads_.assign((size_t)cols_ * rows_, -1);
        nodes_.clear();
    }
    // True if no claimed box overlaps this one. Only the cells the box touches are
    // walked, so the cost follows how crowded that corner of the map is.
    bool freeBox(const ImVec4 &b) const {
        const int cx0 = clampCol(b.x), cx1 = clampCol(b.z);
        const int cy0 = clampRow(b.y), cy1 = clampRow(b.w);
        for(int cy = cy0; cy <= cy1; cy++) {
            for(int cx = cx0; cx <= cx1; cx++) {
                for(int i = heads_[(size_t)cy * cols_ + cx]; i >= 0; i = nodes_[i].next) {
                    const ImVec4 &t = nodes_[i].box;
                    if(b.z > t.x && b.x < t.z && b.w > t.y && b.y < t.w) { return false; }
                }
            }
        }
        return true;
    }
    void claim(const ImVec4 &b) {
        // Fully off the map: nothing that can be drawn overlaps it. (A partially off
        // box is clamped, which is what keeps an edge marker's dot free of names.)
        if(b.z < x0_ || b.x > x0_ + (float)cols_ * cell_ ||
           b.w < y0_ || b.y > y0_ + (float)rows_ * cell_) { return; }
        const int cx0 = clampCol(b.x), cx1 = clampCol(b.z);
        const int cy0 = clampRow(b.y), cy1 = clampRow(b.w);
        for(int cy = cy0; cy <= cy1; cy++) {
            for(int cx = cx0; cx <= cx1; cx++) {
                const size_t h = (size_t)cy * cols_ + cx;
                nodes_.push_back({ b, heads_[h] });
                heads_[h] = (int)nodes_.size() - 1;
            }
        }
    }
private:
    struct Node { ImVec4 box; int next; };
    int clampCol(float x) const {
        const int c = (int)((x - x0_) / cell_);
        return c < 0 ? 0 : (c >= cols_ ? cols_ - 1 : c);
    }
    int clampRow(float y) const {
        const int r = (int)((y - y0_) / cell_);
        return r < 0 ? 0 : (r >= rows_ ? rows_ - 1 : r);
    }
    int cols_ = 0, rows_ = 0;
    float x0_ = 0.0f, y0_ = 0.0f, cell_ = 1.0f;
    std::vector<int> heads_;      // cell -> first node index, -1 when empty
    std::vector<Node> nodes_;     // one node per (box, cell) pair
};

template <class Item>
inline std::vector<MapLabelPlaced> placeMapLabels(const std::vector<Item> &items,
                                                  const std::vector<MapLabelCand> &blocks,
                                                  const ImVec4 &view,
                                                  size_t max_drawn,
                                                  float gap = 1.0f, float pad = 4.0f,
                                                  float cell = 32.0f) {
    // Index order: the caller's items stay where they are because the result indexes
    // them. stable_sort keeps bodies in catalogue order inside a priority tier, so
    // which name wins a contest does not chatter between frames as bodies drift.
    std::vector<int> order(items.size());
    for(size_t i = 0; i < order.size(); i++) { order[i] = (int)i; }
    std::stable_sort(order.begin(), order.end(),
                     [&](int a, int b) { return items[a].cand.prio < items[b].cand.prio; });

    // Reused across frames and both map windows (they draw in turn on one thread);
    // reset() is what re-fits it to this window.
    static thread_local LabelGrid grid;
    grid.reset(view, cell);
    // Markers first, before any name: a body keeps its own dot even when the map has
    // no room for its name.
    for(const Item &it : items) {
        const MapLabelCand &c = it.cand;
        grid.claim(ImVec4(c.dot.x - c.r_px - gap, c.dot.y - c.r_px - gap,
                          c.dot.x + c.r_px + gap, c.dot.y + c.r_px + gap));
    }
    for(const MapLabelCand &b : blocks) {
        grid.claim(ImVec4(b.dot.x - b.r_px - gap, b.dot.y - b.r_px - gap,
                          b.dot.x + b.r_px + gap, b.dot.y + b.r_px + gap));
    }

    std::vector<MapLabelPlaced> out;
    out.reserve(max_drawn < items.size() ? max_drawn : items.size());
    ImVec2 slot[4];
    for(int idx : order) {
        if(out.size() >= max_drawn) { break; }
        const MapLabelCand &c = items[(size_t)idx].cand;
        mapLabelSlots(c, pad, slot);
        for(const ImVec2 &p : slot) {
            const ImVec4 box(p.x - gap, p.y - gap,
                             p.x + c.size.x + gap, p.y + c.size.y + gap);
            // A name the window's clip rect would cut must not claim space -- it would
            // cost the map a name that fits. (A body at the very edge keeps its dot
            // and loses its name.)
            if(box.x < view.x || box.z > view.z || box.y < view.y || box.w > view.w) {
                continue;
            }
            if(!grid.freeBox(box)) { continue; }
            grid.claim(box);
            out.push_back({idx, p});
            break;
        }
    }
    return out;
}
