#pragma once
// orbitmap.h -- projection + drawing for the "Orbital map" window.
//
// The map is an orthographic top-down view of the FOCUS body's inertial
// frame, projected onto its reference plane (XZ, normal +Y -- the plane the
// bodies orbit in; see railStateFromElements, rhat = (cos phi, 0, sin phi)).
// A 3D point (x, y, z) in that frame maps to
//   (cx, cy) + (x, z) / scale
// so y (height above the reference plane) is dropped -- that is what makes an
// inclined orbit look squashed. The focus body sits at the origin, which is
// the ellipse focus, so an orbit drawn from this frame has its central body
// at the right spot for free.
//
// project() is pure math (glm only) so it can be unit-tested without
// rendering; the draw* helpers are thin ImDrawList calls. Orbit sampling
// (propagateKepler) stays in the caller, so this header stays physics-free
// and later phases (child orbits, the transfer conic) just project more 3D
// points through the same helpers.

#include <glm/glm.hpp>

#include <vector>

#include <imgui.h>

struct OrbitMap {
    double cx = 200.0, cy = 200.0;  // screen position of the focus
    double scale = 6000.0;          // meters per pixel

    // 3D point in the focus's inertial frame -> 2D map coordinates.
    // (cx, cy) + (p.x, p.z) / scale; p.y (the map normal) is dropped.
    glm::dvec2 project(const glm::dvec3 &p) const {
        return glm::dvec2(cx + p.x / scale, cy + p.z / scale);
    }

    ImVec2 px(const glm::dvec3 &p) const {
        const glm::dvec2 q = project(p);
        return ImVec2(float(q.x), float(q.y));
    }

    // A closed orbit from sampled 3D points (focus's inertial frame).
    void drawOrbit(ImDrawList *dl, const std::vector<glm::dvec3> &pts,
                   ImU32 col, float thickness = 1.0f) const {
        std::vector<ImVec2> sp;
        sp.reserve(pts.size());
        for(const glm::dvec3 &p : pts) { sp.push_back(px(p)); }
        if(sp.size() >= 3) {
            // imgui 1.92.8+ signature: (points, count, col, thickness, flags)
            // -- 'closed' is no longer a bool param, it is the ImDrawFlags_Closed
            // flag (the old (.., bool closed, float thickness) order now trips
            // the "Did you swap thickness and flags?" assert).
            dl->AddPolyline(sp.data(), (int)sp.size(), col, thickness,
                            ImDrawFlags_Closed);
        }
    }

    // Filled dot at a 3D position.
    void drawDot(ImDrawList *dl, const glm::dvec3 &p, float r_px,
                 ImU32 col) const {
        dl->AddCircleFilled(px(p), r_px, col);
    }

    // The focus body: a circle of its true radius (meters) at the center.
    void drawBody(ImDrawList *dl, double radius_m, ImU32 col) const {
        dl->AddCircleFilled(ImVec2(float(cx), float(cy)),
                            float(radius_m / scale), col);
    }
};

// A near-black or near-white that contrasts with the given (window) background,
// so the orbit / ship markers stay readable under any ImGui style (light or
// dark). Rec. 709 perceptual luminance, 0.5 threshold: light bg -> dark ink,
// dark bg -> light ink. (The colored accents -- body, apsides -- are left
// fixed; only the near-white/near-black elements need this.)
inline ImU32 contrastingColor(const ImVec4 &bg,
                              const ImVec4 &dark  = ImVec4(0.08f, 0.08f, 0.08f, 1.0f),
                              const ImVec4 &light = ImVec4(0.95f, 0.95f, 0.95f, 1.0f)) {
    const float lum = 0.2126f * bg.x + 0.7152f * bg.y + 0.0722f * bg.z;
    const ImVec4 c = (lum > 0.5f) ? dark : light;
    // Pack to imgui's 0xAABBGGRR ImU32 layout directly (no imgui call), so
    // this stays usable in the link-free header-only unit test.
    const int r = (int)(c.x * 255.0f + 0.5f);
    const int g = (int)(c.y * 255.0f + 0.5f);
    const int b = (int)(c.z * 255.0f + 0.5f);
    const int a = (int)(c.w * 255.0f + 0.5f);
    return (ImU32)(((a & 0xFF) << 24) | ((b & 0xFF) << 16) |
                   ((g & 0xFF) << 8) | (r & 0xFF));
}
