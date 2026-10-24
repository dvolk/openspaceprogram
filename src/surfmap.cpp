// surfmap.cpp -- the Surface Map's pixel buffer (the projection + shading
// math is in surfmap.h). One pixel per (lon, lat) of the equirectangular
// grid over the body's ROTATING frame. Runs on the background worker.

#include "surfmap.h"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <vector>

#include "game.h"
#include "terrain.h"

namespace {
// Direction TOWARD the star in `body`'s rotating frame. False when there
// is no terminator (the body IS the star, or the system has no sun).
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
    // Default body = the active ship's parent; a combo pick pins the map.
    TerrainBody *body = g.surfmap.body
        ? g.surfmap.body
        : (g.ship ? g.ship->m_parent : g.sys.home);
    if(body == nullptr) { return; }
    const TerrainBody *sun = g.sun;

    int w = std::max(16, g.args.surfmap_n);
    const int h = std::max(8, w / 2);   // 2:1 equirectangular

    // The terminator at the REQUEST instant (frames move with the sim).
    // has_sun = there IS a terminator to bake; baked = it is actually applied
    // (the "Sun shading" box is on). The overlay is always the true terminator;
    // `baked` only drives the log's shade on/off + shaded means.
    glm::dvec3 sun_dir;
    const bool has_sun = sunDirRot(body, sun, sun_dir);
    const bool baked = g.surfmap.shade && has_sun;

    // Snapshot terrain params and hand the sweep to the worker. The worker
    // colours from THIS snapshot: surface.max_height is applied LATER by
    // AttachRoot on the main thread, so a live per-pixel read would race it.
    const TerrainParams tp = body->params();
    // Hoisted datum: the sea test runs per pixel, and a hand-rolled
    // radius + sea_level there is one more place to get the datum wrong (#406).
    const double sea_r = tp.seaLevelRadius();
    const bool ocean = g.surfmap.sea;     // the "Ocean" box (per-pixel below)
    /* Launch sites the window's overlay marks. Counted here because the worker
       never touches game state, and logged so e2e can pin the marker filter -- a
       mistyped flag name draws no sites and still compiles clean. */
    int n_sites = 0;
    for(const StaticBuilding *p : body->pads) {
        if(p->hasFlag("VAB_LAUNCH_PAD")) { n_sites++; }
    }
    const bool log = g.args.surfmap_log;
    const std::string body_name = body->name;
    const double t_now = g.time;
    const int epoch = g.cache_epoch;      // drop the map if the world changes

    g.surfmap.in_flight++;   // the window's "mapping ..." state
    // `g` is captured by REFERENCE only for the returned continuation; the
    // worker body never touches game state (a by-value capture would copy
    // the non-copyable Game).
    g.jobs.post("Surface map", [&g, tp, sea_r, sun_dir, w, h, baked, has_sun, ocean,
                                log, n_sites, body_name, t_now, epoch]()
                -> std::function<void()> {
        // Worker: build the pixel buffers (pure). shared_ptr because the
        // std::function continuation capture must be copyable.
        // px  = unshaded albedo (the expensive part, computed once).
        // shpx = the terminator as a black overlay, alpha = (1 - shade); drawn
        //         over px with normal alpha blend it multiplies px by shade.
        //         Always the true terminator (has_sun), independent of the
        //         "Sun shading" box, so toggling the box is a free show/hide.
        const size_t npx = (size_t)w * (size_t)h;
        std::vector<unsigned char> px(npx * 4);
        std::vector<unsigned char> shpx(npx * 4);
        double ar = 0.0, ag = 0.0, ab = 0.0;   // albedo (unshaded) means
        double sr = 0.0, sg = 0.0, sb = 0.0;   // displayed (shaded) means
        for(int j = 0; j < h; j++) {
            for(int i = 0; i < w; i++) {
                const glm::dvec3 d = equirectDirAt(i, j, w, h);
                // Ocean box on and at/below sea level: flat sea covers the floor
                // (same coverage as the 3D ocean shell, and the same predicate as
                // a Flat-sea body's painted sea_color floor -- issue #193).
                const bool under_sea = ocean && tp.surface.has_sea
                    && terrainHeight((glm::vec3)d, tp) <= sea_r;
                const glm::vec3 c = under_sea
                    ? tp.surface.sea_color
                    : terrainSurfaceColor((glm::vec3)d, tp);
                unsigned char *q = &px[((size_t)j * w + i) * 4];
                q[0] = (unsigned char)(std::min(1.0f, c.r) * 255.0f + 0.5f);
                q[1] = (unsigned char)(std::min(1.0f, c.g) * 255.0f + 0.5f);
                q[2] = (unsigned char)(std::min(1.0f, c.b) * 255.0f + 0.5f);
                q[3] = 255;
                // Overlay: black, alpha = (1 - shade). Transparent when there is
                // no terminator (has_sun false -> shade 1 -> alpha 0).
                const float f_ov = has_sun ? surfmapShade(d, sun_dir) : 1.0f;
                unsigned char *sq = &shpx[((size_t)j * w + i) * 4];
                sq[0] = sq[1] = sq[2] = 0;
                sq[3] = (unsigned char)((1.0f - f_ov) * 255.0f + 0.5f);
                ar += c.r; ag += c.g; ab += c.b;
                // Log's shaded means reflect what is DISPLAYED (box on + sun).
                const float f_log = baked ? surfmapShade(d, sun_dir) : 1.0f;
                sr += f_log * c.r; sg += f_log * c.g; sb += f_log * c.b;
            }
        }
        if(log) {
            // albedo = before the terminator; shaded = as displayed.
            printf("[surfmap] t=%.1fs body=\"%s\" %dx%d "
                   "albedo=[%.4f %.4f %.4f] shaded=[%.4f %.4f %.4f] "
                   "shade=%s ocean=%s sites=%d\n",
                   t_now, body_name.c_str(), w, h,
                   ar / npx, ag / npx, ab / npx,
                   sr / npx, sg / npx, sb / npx,
                   baked ? "on" : "off", ocean ? "on" : "off", n_sites);
            fflush(stdout);
        }
        // Main-thread continuation (JobRunner::poll): publish the cache (the
        // window uploads it to a texture on a rev change; surfmap.body_name is the
        // staleness check) + clear the "mapping" state.
        std::shared_ptr<std::vector<unsigned char> > ppx =
            std::make_shared<std::vector<unsigned char> >(std::move(px));
        std::shared_ptr<std::vector<unsigned char> > pshpx =
            std::make_shared<std::vector<unsigned char> >(std::move(shpx));
        return [&g, ppx, pshpx, w, h, body_name, epoch]() {
            // Epoch bumped after we posted: this map is for the old world.
            // Drop it -- quit-to-title, a new game and a same-system load never
            // abort jobs (only installSystem does, game.cpp).
            if(epoch != g.cache_epoch) {
                if(g.surfmap.in_flight > 0) { g.surfmap.in_flight--; }
                return;
            }
            g.surfmap.px = std::move(*ppx);
            g.surfmap.shade_px = std::move(*pshpx);
            g.surfmap.w = w;
            g.surfmap.h = h;
            g.surfmap.body_name = body_name;
            g.surfmap.valid = true;
            g.surfmap.rev++;
            if(g.surfmap.in_flight > 0) { g.surfmap.in_flight--; }
        };
    });
}
