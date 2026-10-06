#pragma once
// orbitmap.h -- orthographic top-down projection + drawing for the orbital map.
// Drops the out-of-plane component, so inclined orbits look squashed.

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

    // Near-polar normals keep the canonical X/Z basis (stable equatorial view).
    // `x_axis` (optional) pins the screen-x direction INSIDE the plane: pass the
    // focus frame's +X for an equatorial plane, so flipping the plane combo
    // tilts the picture instead of rotating it (issue #173). Zero length means
    // derive it, the old way.
    // A reference direction that lies nearly ALONG the normal cannot define a
    // stable east: Uranus's 97.8 deg tilt puts its pole within 9 deg of the
    // system +X, where the in-plane part is only 0.15 long and swings on
    // rounding noise. Such an x_axis is ignored in favour of the node line.
    // Handedness of the pinned path: e2 = e1 x n, so a prograde body (which
    // moves along n x r_hat) sweeps counter-clockwise on screen -- the same way
    // the canonical X/Z case below already draws it.
    void setPlane(const glm::dvec3 &normal,
                  const glm::dvec3 &x_axis = glm::dvec3(0.0)) {
        n = glm::normalize(normal);
        const glm::dvec3 t = x_axis - glm::dot(x_axis, n) * n;  // in-plane part
        // |t| is sin(angle from the normal): insist the reference lies at least
        // ~26 deg inside the plane.
        if(glm::length(t) > 0.44) {
            e1 = glm::normalize(t);
            e2 = glm::cross(e1, n);
            return;
        }
        if(glm::abs(glm::dot(n, glm::dvec3(0.0, 1.0, 0.0))) > 0.99) {
            e1 = glm::dvec3(1.0, 0.0, 0.0);
            e2 = glm::dvec3(0.0, 0.0, 1.0);
        } else {
            // Historical basis. Note it ends up with the OPPOSITE handedness to
            // the two paths above (e2 = n x e1), so Ecliptic/Orbital views draw
            // prograde the other way from Equatorial -- issue #181. Left alone
            // here, where flipping it would change two views silently.
            e1 = glm::normalize(glm::cross(glm::dvec3(0.0, 1.0, 0.0), n));
            e2 = glm::cross(n, e1);
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
    // between two samples (the body-on-orbit chord fix).
    void drawOrbit(ImDrawList *dl, const std::vector<glm::dvec3> &pts,
                   ImU32 col, float thickness = 1.0f, bool closed = true,
                   size_t start = 0) const {
        const size_t n = pts.size();
        if(n < 2) { return; }
        // Reused: with hundreds of orbits per frame the per-call vector was
        // measurable allocator traffic.
        thread_local std::vector<ImVec2> sp;
        sp.clear();
        sp.reserve(n);
        for(size_t j = 0; j < n; j++) { sp.push_back(px(pts[(start + j) % n])); }
        // imgui 1.92.8+: closed is ImDrawFlags_Closed, not a bool param.
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
// (Rec. 709 luminance, 0.5 threshold). Colored accents are left fixed.
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
