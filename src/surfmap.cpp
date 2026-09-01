// surfmap.cpp -- the Surface Map's pixel buffer (the projection + shading
// math is surfmap.h, pure and tested; this part needs the game: the
// body's terrain colors, the sun's position, the sim clock).
//
// One pixel per (lon, lat) of the equirectangular grid over the body's
// ROTATING frame (the surface's own frame): the body's analytic surface
// color at that direction (TerrainBody::SurfaceColor, the SAME function
// the terrain mesh bakes, so the map matches the rendered surface),
// optionally multiplied by the terminator at the compute instant (the
// sun's direction in the body's rotating frame). The map is the surface,
// fixed; gameui.cpp draws the ship's dot (sub-satellite point) and orbit
// (ground track) on it in this same frame. The sweep is one-shot (M key
// / the window's button), like the porkchop grid: a 256 x 128 map is ~33k
// simplex samples, so it runs on the background worker (g.jobs) and the
// frame stays responsive (see job.h).

#include "surfmap.h"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <vector>

#include "game.h"
#include "terrain.h"

namespace {
// Direction TOWARD the star, in `body`'s rotating frame (the frame the
// map's pixel directions live in). False when there is no terminator --
// the mapped body IS the star (it maps itself, fully lit) or the system
// has no sun -- in which case the caller skips the shading.
bool sunDirRot(const TerrainBody *body, const TerrainBody *sun,
               glm::dvec3 &dir) {
    if(sun == nullptr || body == sun) {
        return false;
    }
    const glm::dvec3 to_sun =
        sun->frame->root_pos - body->frame->root_pos;
    const double l = glm::length(to_sun);
    if(l < 1e-9) {
        return false;
    }
    // root axes -> body's rotating frame (a direction: rotate by the
    // inverse orientation; v*M is M^T*v in glm).
    const glm::dmat3 to_rot =
        glm::transpose(body->frame->getRotFrame()->root_orient);
    dir = to_rot * (to_sun / l);
    return true;
}
} // namespace

void surfmapCompute(Game &g) {
    // Default body = the active ship's parent (the ship is orbiting /
    // landed on it); a combo pick pins the map to another body.
    TerrainBody *body = g.surfmap_body
        ? g.surfmap_body
        : (g.ship ? g.ship->m_parent : g.sys.home);
    if(body == nullptr) { return; }
    const TerrainBody *sun = g.sun;

    int w = std::max(16, g.args.surfmap_n);
    const int h = std::max(8, w / 2);   // 2:1 equirectangular

    // The terminator at the REQUEST instant: the frames move with the
    // sim, so this read stays on the main thread. False when there is no
    // terminator -- the mapped body IS the star (it maps itself, fully
    // lit) or the system has no sun -- in which case no shading is baked.
    glm::dvec3 sun_dir;
    const bool baked = g.surfmap_shade && sunDirRot(body, sun, sun_dir);

    // Snapshot the rest (values only) and hand the sweep to the worker
    // (g.jobs), like the porkchop grid: the frame stays responsive and
    // the last map stays on screen until the job lands. The worker calls
    // body->SurfaceColor per pixel: that state (the terrain params, the
    // palette, the radius) is set once in load_system and never written
    // after, and the bodies outlive every job (main.cpp joins g.jobs
    // before freeing them), so the off-thread READ is safe -- the same
    // const data create_grid_mesh baked into the mesh.
    const bool log = g.args.surfmap_log;
    const std::string body_name = body->name;
    const double t_now = g.time;

    g.surfmap_in_flight++;   // the window's "mapping ..." state
    // `g` is captured by REFERENCE only so the returned continuation (below)
    // can name it; the worker body itself never touches game state. A
    // by-value capture would copy the non-copyable Game (the JobRunner
    // member forbids it).
    g.jobs.post("Surface map", [&g, body, sun_dir, w, h, baked, log,
                                body_name, t_now]()
                -> std::function<void()> {
        // Worker thread: build the pixel buffer. No game-state WRITE, GL
        // or imgui here. The result is handed to the main thread through
        // the returned continuation; a shared_ptr lets it outlive this
        // body (a C++11-safe way to move a large result across the thread
        // handoff -- lambda capture initializers are C++14).
        const size_t npx = (size_t)w * (size_t)h;
        std::vector<unsigned char> px(npx * 4);
        double ar = 0.0, ag = 0.0, ab = 0.0;   // albedo (unshaded) means
        double sr = 0.0, sg = 0.0, sb = 0.0;   // stored (shaded) means
        for(int j = 0; j < h; j++) {
            for(int i = 0; i < w; i++) {
                const glm::dvec3 d = surfmapDir(i, j, w, h);
                const glm::vec3 c = body->SurfaceColor((glm::vec3)d);
                const float f = baked ? surfmapShade(d, sun_dir) : 1.0f;
                unsigned char *q = &px[((size_t)j * w + i) * 4];
                q[0] = (unsigned char)(std::min(1.0f, f * c.r) * 255.0f + 0.5f);
                q[1] = (unsigned char)(std::min(1.0f, f * c.g) * 255.0f + 0.5f);
                q[2] = (unsigned char)(std::min(1.0f, f * c.b) * 255.0f + 0.5f);
                q[3] = 255;
                ar += c.r; ag += c.g; ab += c.b;
                sr += f * c.r; sg += f * c.g; sb += f * c.b;
            }
        }
        if(log) {
            // albedo = the map before the terminator; shaded = as stored.
            // shade=on means a terminator was baked (the e2e battery checks
            // shaded < albedo on the sun-facing body).
            printf("[surfmap] t=%.1fs body=\"%s\" %dx%d "
                   "albedo=[%.4f %.4f %.4f] shaded=[%.4f %.4f %.4f] shade=%s\n",
                   t_now, body_name.c_str(), w, h,
                   ar / npx, ag / npx, ab / npx,
                   sr / npx, sg / npx, sb / npx,
                   baked ? "on" : "off");
            fflush(stdout);
        }
        // Main-thread continuation (JobRunner::poll): publish the cache
        // (the window uploads it to a texture on a rev change;
        // surfmap_body_name is the staleness check when the ship's SOI /
        // the combo pick changes) + clear the "mapping" state. `g` is
        // captured BY REFERENCE (a by-value capture would copy the
        // non-copyable Game -- the JobRunner member forbids it); it is the
        // main()'s object, which outlives every job (main.cpp joins the
        // worker before the game is torn down).
        std::shared_ptr<std::vector<unsigned char> > ppx =
            std::make_shared<std::vector<unsigned char> >(std::move(px));
        return [&g, ppx, w, h, body_name, t_now]() {
            g.surfmap_px = std::move(*ppx);
            g.surfmap_w = w;
            g.surfmap_h = h;
            g.surfmap_body_name = body_name;
            g.surfmap_computed_at = t_now;
            g.surfmap_valid = true;
            g.surfmap_rev++;
            if(g.surfmap_in_flight > 0) { g.surfmap_in_flight--; }
        };
    });
}
