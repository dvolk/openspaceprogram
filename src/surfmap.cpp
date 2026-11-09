// surfmap.cpp -- the Surface Map's pixel buffer (the projection + shading
// math is in surfmap.h). One pixel per (lon, lat) of the equirectangular
// grid over the body's ROTATING frame. Runs on the background worker.

#include "surfmap.h"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "game.h"
#include "groundtemp.h"
#include "terrain.h"
#include "insolation.h"
#include "solar.h"

namespace {
// Direction TOWARD the star in `body`'s rotating frame, from whatever the frames hold
// right now (the map bakes the terminator at the request instant). False when there is
// no terminator (the body IS the star, or the system has no sun). The arithmetic itself
// is insolation.h's, shared with the calendar's subsolar row and the solar log (#724).
bool sunDirRot(const TerrainBody *body, const TerrainBody *sun,
               glm::dvec3 &dir) {
    if(sun == nullptr || body == sun || body->frame == nullptr
       || sun->frame == nullptr) {
        return false;
    }
    const StarFromBody sv = starFromBody(body->frame->root_pos,
                                        sun->frame->root_pos,
                                        body->frame->getRotFrame()->root_orient);
    if(!sv.valid) { return false; }
    dir = sv.to_sun;
    return true;
}
}   // namespace

// Map `body` (default: the active ship's parent): snapshot the inputs on the main thread,
// sweep on the worker, publish on the main thread. The sweep is pure and per-pixel cheap,
// but 512x256 of terrainSurfaceColor is far too much to do inside a frame, hence the job.
void surfmapCompute(Game &g) {
    // Default body = the active ship's parent; a combo pick pins the map.
    TerrainBody *body = g.surfmap.body
        ? g.surfmap.body
        : (g.ship ? g.ship->m_parent : g.sys.home);
    if(body == nullptr) { return; }
    const TerrainBody *sun = g.sun;

    const int w = std::max(16, g.args.surfmap_n);
    const int h = std::max(8, w / 2);   // 2:1 equirectangular

    /* Which PICTURE to paint, and at what level (#745). The old "Ocean" box is gone:
       painting the sea is a picture of its own (kMapGroundOcean), not a coat over one
       picture. The terminator stays a box because it reads on top of any of them. */
    const int type = g.surfmap.map_type;
    const bool ocean = (type == kMapGroundOcean);
    const bool want_temp = (type == kMapTemperature);

    /* The terminator at the REQUEST instant (frames move with the sim), read BEFORE the
       temperature layer: a world with no star to light it has no day side to price, and the
       layer has to know that rather than paint a field from a direction that was never
       measured (sun_dir starts at zero, so a stray vector could never reach the law).
       has_sun = there IS a terminator to bake; baked = it is actually applied (the "Sun
       shading" box). The overlay itself is always the true terminator and is baked for every
       map type; `baked` only drives the log's shade on/off and the shaded means. */
    glm::dvec3 sun_dir = glm::dvec3(0.0);
    const bool has_sun = sunDirRot(body, sun, sun_dir);
    const bool baked = g.surfmap.shade && has_sun;

    /* The third anchor rung needs LIGHT, and the map prices the body's ACTUAL rail -- the
       whole picture is one instant, and this is the same live sample --solar-log prices
       (solar.h). The atlas prices the mean orbit instead, so on an eccentric rail the two
       dashboards can differ by the rail's own spread; that is why the e2e case pins exact
       numbers on bodies whose anchor involves no light at all (an air datum, or a stated
       ground: Duna, Kerbin, Laythe). */
    double derived_bb = 0.0;
    if(want_temp && has_sun) {
        const SunSample ss = sunSample(sun->star_luminosity, sun->frame->root_pos,
                                       body->frame->root_pos);
        derived_bb = derivedBlackbody_K(ss.flux, body->surface.bond_albedo);
    }
    // has_sun goes in as an input, so the refusal is one decision in one place rather than a
    // patch applied afterwards where a reader has to find both halves.
    const TempLayer tl = want_temp ? tempLayerFor(body->surface, derived_bb, has_sun)
                                   : TempLayer{};

    // Snapshot terrain params and hand the sweep to the worker. The worker
    // colours from THIS snapshot: surface.max_height is applied LATER by
    // AttachRoot on the main thread, so a live per-pixel read would race it.
    const TerrainParams tp = body->params();
    // Hoisted datum: the sea test runs per pixel, and a hand-rolled
    // radius + sea_level there is one more place to get the datum wrong (#406).
    const double sea_r = tp.seaLevelRadius();
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
                                type, level = g.surfmap.level, want_temp, tl,
                                log, n_sites, body_name, t_now, epoch]()
                -> std::function<void()> {
        // Worker: build the pixel buffers (pure). shared_ptr because the
        // std::function continuation capture must be copyable.
        // px  = the BASE picture for this map type: the unshaded albedo, or the
        //       temperature field coloured by surfmapRamp.
        // shpx = the terminator as a black overlay, alpha = (1 - shade); drawn
        //         over px with normal alpha blend it multiplies px by shade.
        //         Always the true terminator (has_sun), independent of the
        //         "Sun shading" box, so toggling the box is a free show/hide.
        const size_t npx = (size_t)w * (size_t)h;
        std::vector<unsigned char> px(npx * 4);
        std::vector<unsigned char> shpx(npx * 4);
        double ar = 0.0, ag = 0.0, ab = 0.0;   // base colour means (unshaded)
        double sr = 0.0, sg = 0.0, sb = 0.0;   // displayed (shaded) means
        /* Temperature stats, only meaningful when the base IS a temperature. mean4 is the
           mean taken in T^4 -- the quantity the model conserves -- written back as a
           temperature, so it must land on the anchor.
           The MEANS are area-weighted, not pixel-counted: an equirectangular grid is uniform
           in (lon, lat), so a pixel near a pole covers far less ground than one on the
           equator, and a plain average over pixels over-weights the poles. Not pedantry: on
           Duna the unweighted mean came out kelvin below the anchor the model provably
           conserves, purely from the projection -- 8 K under its old 0.300 swing, 3 K under the
           0.708 it carries now (e2e/render/745 re-measures it, because the offset is a function
           of the swing and a stale number here would be a lie with a decimal point on it). cos(lat) is the xz length of the unit
           direction, so the weight is free. min/max stay unweighted -- extremes, not means. */
        double tmin = 1e30, tmax = -1e30, tsum = 0.0, t4sum = 0.0, tw = 0.0;

        for(int j = 0; j < h; j++) {
            for(int i = 0; i < w; i++) {
                const glm::dvec3 d = equirectDirAt(i, j, w, h);
                glm::vec3 c;                 // the base pixel's colour
                if(want_temp && tl.ok) {                    /* The SAME cosine the terminator overlay uses, through the SAME law the
                       dossier and the solar log use -- nothing is invented at the pixel. */
                    const double T = surfmapGroundT(tl.anchor_K, tl.knob, d, sun_dir);
                    // Ramp span = the model's own ends. With no swing lo == hi, so the span
                    // is 0 and every pixel takes the midpoint: one honest flat colour rather
                    // than a gradient the eye invents.
                    const double t01 = (tl.hi_K > tl.lo_K)
                        ? (T - tl.lo_K) / (tl.hi_K - tl.lo_K) : 0.5;
                    unsigned char r = 0, gg = 0, bb = 0;
                    surfmapRamp(t01, r, gg, bb);
                    c = glm::vec3(r / 255.0f, gg / 255.0f, bb / 255.0f);
                    if(T < tmin) { tmin = T; }
                    if(T > tmax) { tmax = T; }
                    const double cw = std::sqrt(d.x * d.x + d.z * d.z);   // cos(latitude)
                    tw += cw;
                    tsum += cw * T;
                    t4sum += cw * T * T * T * T;
                } else if(want_temp) {
                    /* Asked for a temperature this world does not have, so the picture says
                       so: a flat neutral field, with the reason in the window's legend row.
                       Falling back to the albedo here would put a ground picture under a
                       "temperature" label, which is the map lying about what it is (#727's
                       law, applied to a picture rather than a number). */
                    c = glm::vec3(0.34f, 0.34f, 0.34f);
                } else {
                    // Ocean map on and at/below sea level: flat sea covers the floor
                    // (same coverage as the 3D ocean shell, and the same predicate as
                    // a Flat-sea body's painted sea_color floor -- issue #193).
                    const bool under_sea = ocean && tp.surface.has_sea
                        && terrainHeight((glm::vec3)d, tp) <= sea_r;
                    c = under_sea
                        ? tp.surface.sea_color
                        : terrainSurfaceColor((glm::vec3)d, tp);
                }
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
            // Base colour means before/after the terminator, then the map's identity and,
            // for a temperature map, what the sweep actually priced. A dash is an absent
            // reading, never a zero (#727): a ground map has no temperatures to report and
            // a world the model cannot price reports none either.
            const bool have_t = want_temp && tl.ok;
            /* A dash is an absent reading, never a zero (#727). Under a temperature map the
               base picture's colours are RAMP COLOURS, not albedos, so the albedo columns
               dash rather than reporting a meaningless mean of the palette. */
            char alb_s[64], shd_s[64];
            if(want_temp) {
                snprintf(alb_s, sizeof alb_s, "-");
                snprintf(shd_s, sizeof shd_s, "-");
            } else {
                snprintf(alb_s, sizeof alb_s, "[%.4f %.4f %.4f]",
                         ar / npx, ag / npx, ab / npx);
                snprintf(shd_s, sizeof shd_s, "[%.4f %.4f %.4f]",
                         sr / npx, sg / npx, sb / npx);
            }
            printf("[surfmap] t=%.1fs body=\"%s\" %dx%d map=%s level=%s "
                   "albedo=%s shaded=%s shade=%s ocean=%s sites=%d",
                   t_now, body_name.c_str(), w, h, surfMapTypeName(type),
                   surfLevelName((SurfLevel)level), alb_s, shd_s,
                   baked ? "on" : "off", ocean ? "on" : "off", n_sites);
            if(want_temp) {
                if(have_t) {
                    /* anchor_src names which rung the anchor came from -- the same decision the
                       cursor's provenance and the atlas' `ground anchor (...)` row report, so a
                       log line can be compared against either without guessing. It matters most
                       for the DERIVED rung: the map prices it on the body's LIVE rail and the
                       atlas on its mean orbit, so on an eccentric world the two anchors differ
                       by the rail's own spread, and the row says which one it printed (#687). */
                    printf(" temp=[%.2f %.2f %.2f] temp_ends=[%.2f %.2f] "
                           "temp_mean4=%.2f anchor=%.2f anchor_src=\"%s\"",
                           tmin, tsum / tw, tmax, tl.lo_K, tl.hi_K,
                           std::pow(t4sum / tw, 0.25), tl.anchor_K,
                           tl.legend.c_str());
                } else {
                    printf(" temp=- note=\"%s\"", tl.legend.c_str());
                }
            }
            printf("\n");
            fflush(stdout);
        }
        // Main-thread continuation (JobRunner::poll): publish the cache (the
        // window uploads it to a texture on a rev change; surfmap.body_name is the
        // staleness check) + clear the "mapping" state.
        std::shared_ptr<std::vector<unsigned char> > ppx =
            std::make_shared<std::vector<unsigned char> >(std::move(px));
        std::shared_ptr<std::vector<unsigned char> > pshpx =
            std::make_shared<std::vector<unsigned char> >(std::move(shpx));
        return [&g, ppx, pshpx, w, h, body_name, type, level, want_temp, tl, sun_dir, has_sun,
                have_t = (want_temp && tl.ok),
                epoch]() {
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
            // What these buffers ARE, as one pair: the window re-bakes when either disagrees
            // with what the player asked for, and its cursor readout refuses to quote authored
            // numbers of the body on screen while the picture still belongs to another (#745).
            g.surfmap.baked_type = type;
            g.surfmap.baked_level = level;
            // The star this picture was baked under, so the cursor's number is the picture's
            // number rather than a fresh sample of a sun that has since moved.
            g.surfmap.baked_sun_dir = sun_dir;
            g.surfmap.baked_sun_valid = has_sun;
            /* Only what the cursor readout reads, and only when there IS a temperature to
               read: leaving a plausible anchor (Moho's 440 K) published beside temp_ok=false is
               how a later readout ends up quoting a number this decision refused (#726). The
               sweep's own statistics are not published at all -- --surfmap-log carries them,
               which keeps one copy (#687). */
            g.surfmap.temp_ok = have_t;
            g.surfmap.temp_anchor_K = have_t ? tl.anchor_K : 0.0;
            g.surfmap.temp_knob = have_t ? tl.knob : kEvenShareUnauthored;
            g.surfmap.temp_block = want_temp ? tl.block : std::string();
            g.surfmap.valid = true;
            g.surfmap.rev++;
            if(g.surfmap.in_flight > 0) { g.surfmap.in_flight--; }
        };
    });
}
