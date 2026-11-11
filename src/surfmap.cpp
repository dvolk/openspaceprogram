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
    const bool want_wind = (type == kMapWind);

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
    /* The wind profile (#769). Decided here, once, and COPIED into the job: the worker never
       touches a body pointer, exactly like the TerrainParams snapshot below. A wind needs no
       sun and no clock -- it is authored data about the air, not a sample of the sky right
       now -- so unlike the temperature layer it does not care what instant the map is baked at. */
    const WindLayer wl = want_wind ? windLayerFor(body->surface) : WindLayer{};

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
                                want_wind, wl,
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
        /* Wind stats, same shape and the same area weighting (cos lat) as the temperature
           block above: what the sweep actually painted, so the log can show the ground's reach
           against the table's ends rather than assuming they meet. */
        double wmin = 1e30, wmax = -1e30, wsum = 0.0, wwt = 0.0;
        /* Arrow cells drawn by the pass below, counted where they are drawn. A wind map whose
           grid silently draws nothing (a stride bug, a refusal that painted anyway) is otherwise
           invisible to every reader but one standing in front of the screen. */
        int wind_cells = 0;

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
                } else if(want_wind && wl.ok) {
                    /* The scalar half of the field: how fast the air moves OVER THIS PIXEL.
                       The altitude windAt() wants is this pixel's ground elevation above the
                       sea-level datum -- the same coordinate the Surface window's Wind: row
                       asks with and the aero law reads -- so a mountain pokes into faster air
                       and shows up brighter, and a basin does not get credited with the wind
                       of the ridge above it. The arrows go on afterwards; a scalar alone is not
                       a vector field (#769). */
                    const double elev = (double)terrainHeight((glm::vec3)d, tp) - sea_r;
                    const glm::dvec2 wv = windAt(wl.rows, elev);
                    const double spd = glm::length(wv);
                    unsigned char r = 0, gg = 0, bb = 0;
                    surfmapRamp(surfmapWindT01(spd, wl.lo_ms, wl.hi_ms), r, gg, bb);
                    c = glm::vec3(r / 255.0f, gg / 255.0f, bb / 255.0f);
                    if(spd < wmin) { wmin = spd; }
                    if(spd > wmax) { wmax = spd; }
                    const double cw = std::sqrt(d.x * d.x + d.z * d.z);   // cos(latitude)
                    wwt += cw;
                    wsum += cw * spd;
                } else if(want_temp || want_wind) {
                    /* Asked for a temperature this world does not have, so the picture says
                       so: a flat neutral field, with the reason in the window's legend row.
                       Falling back to the albedo here would put a ground picture under a
                       "temperature" label, which is the map lying about what it is (#727's
                       law, applied to a picture rather than a number). A wind map refuses the
                       same way -- and the two refusals say WHICH absence in the log's note= and
                       on the cursor row, not here (#726). */
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
        /* THE ARROWS (#769) -- the vector half. A speed-coloured field says where the air
           moves HARDER and nothing about WHICH WAY, and which way is the half a player asks
           for. They go on afterwards rather than inside the loop above: a base pixel has to be
           one colour, and an arrow is a stroke across several of them.
           On a coarse grid, because the field is sampled -- not a streamline integration. The
           grid is what makes the flat parts visible: every body authored today has one wind
           direction and one altitude axis (#767), so the honest picture is a grid of parallel
           arrows whose length changes where the ground changes height.
           The base mean (ar/ag/ab) above is taken BEFORE this: those columns describe the
           field, not the ink drawn on it. (They dash for a wind map anyway.) */
        if(want_wind && wl.ok) {
            const int stride = std::max(8, w / 32);   // ~32 cells across, whatever the bake size
            const double half = stride * 0.42;        // arrow half-length, kept inside one cell
            const unsigned char kr = 245, kg = 248, kb = 255;   // near-white: reads over any ramp colour
            auto put = [&](int xi, int yj, unsigned char r, unsigned char g, unsigned char b) {
                if(xi < 0 || yj < 0 || xi >= w || yj >= h) { return; }
                unsigned char *q = &px[((size_t)yj * w + xi) * 4];
                q[0] = r; q[1] = g; q[2] = b; q[3] = 255;
            };
            auto stroke = [&](double x0, double y0, double x1, double y1) {
                const int n = std::max(1, (int)std::ceil(std::hypot(x1 - x0, y1 - y0)));
                for(int k = 0; k <= n; k++) {
                    const double f = (double)k / (double)n;
                    put((int)std::llround(x0 + (x1 - x0) * f),
                        (int)std::llround(y0 + (y1 - y0) * f), kr, kg, kb);
                }
            };
            for(int j = stride / 2; j < h; j += stride) {
                for(int i = stride / 2; i < w; i += stride) {
                    const glm::dvec3 d = equirectDirAt(i, j, w, h);
                    const double elev = (double)terrainHeight((glm::vec3)d, tp) - sea_r;
                    const glm::dvec2 wv = windAt(wl.rows, elev);
                    const double spd = glm::length(wv);
                    const double cl = std::sqrt(d.x * d.x + d.z * d.z);   // cos(latitude)
                    const glm::dvec2 pv = surfmapWindPixelVec(wv.x, wv.y, cl);
                    const double pvl = glm::length(pv);
                    wind_cells++;   // every cell draws something: an arrow, or a dot for calm air
                    if(spd <= 0.0 || pvl <= 0.0) {
                        /* Calm air gets a dot, not an arrow. A direction drawn for a wind of
                           zero -- or a headless dash for one that has no horizontal component at
                           all -- is a reading the file never gave (#727). */
                        put(i, j, kr, kg, kb);
                        continue;
                    }
                    const glm::dvec2 u = pv * (1.0 / pvl);
                    const double L = half * (0.35 + 0.65 * surfmapWindT01(spd, wl.lo_ms, wl.hi_ms));
                    const double x1 = i + u.x * L, y1 = j + u.y * L;
                    stroke(i - u.x * L, j - u.y * L, x1, y1);
                    /* A head at the tip: two strokes back at 150 deg, so the arrow reads as
                       pointing somewhere rather than as a dash lying across the cell. */
                    const double ca = std::cos(2.618), sa = std::sin(2.618), hl = L * 0.45;
                    stroke(x1, y1, x1 + (u.x * ca - u.y * sa) * hl, y1 + (u.x * sa + u.y * ca) * hl);
                    stroke(x1, y1, x1 + (u.x * ca + u.y * sa) * hl, y1 + (-u.x * sa + u.y * ca) * hl);
                }
            }
        }
        if(log) {
            // Base colour means before/after the terminator, then the map's identity and,
            // for a temperature map, what the sweep actually priced. A dash is an absent
            // reading, never a zero (#727): a ground map has no temperatures to report and
            // a world the model cannot price reports none either.
            const bool have_t = want_temp && tl.ok;
            /* A dash is an absent reading, never a zero (#727). Under a temperature or a wind
               map the base picture's colours are RAMP COLOURS, not albedos, so the albedo
               columns dash rather than reporting a meaningless mean of the palette. */
            char alb_s[64], shd_s[64];
            if(want_temp || want_wind) {
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
            if(want_wind) {
                if(wl.ok) {
                    /* wind= is what the sweep painted (min, area-weighted mean, max) at the
                       GROUND's altitudes; wind_ends= is what the file states at its two rows.
                       The gap between them is the point: on Earth the deck row lives at 11 km
                       while the ground tops out around 6, so the ramp spans a span the picture
                       only partly reaches -- and a reader can see that here instead of being
                       shown a colour with no scale. wind_rows= says how much profile the body
                       authored, which is the difference between a two-row datum and a real
                       layered claim (#767 is what would add the other axis). wind_cells= is how
                       many grid cells the arrow pass drew: the vector half has no other readout,
                       and a field that paints its colours but forgets its arrows looks fine. */
                    printf(" wind=[%.2f %.2f %.2f] wind_ends=[%.2f %.2f] wind_rows=%d"
                           " wind_cells=%d",
                           wmin, wwt > 0.0 ? wsum / wwt : 0.0, wmax,
                           wl.lo_ms, wl.hi_ms, (int)wl.rows.size(), wind_cells);
                } else {
                    printf(" wind=- note=\"%s\"", wl.legend.c_str());
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
        return [&g, ppx, pshpx, w, h, body_name, type, level, want_temp, tl, want_wind, wl,
                sun_dir, has_sun, t_now,
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
            /* WHEN the picture is from (#746). The request instant, not the landing instant --
               the same stamp --surfmap-log prints as t=, and the same instant the terminator
               above was measured at, so the number on the window and the number in the log
               describe one bake. */
            g.surfmap.baked_t = t_now;
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
            /* The wind, published the same way and for the same reason: the cursor's m/s has to
               be the number THIS picture's pixels were coloured from, so it reads the profile
               that was baked, not the body now on screen. Copying the rows rather than a pointer
               is also what keeps the worker honest -- nothing on that thread dereferences a
               TerrainBody. Empty rows and ok=false go together, so windAt() never sees the empty
               vector its assert refuses. */
            g.surfmap.wind_ok = want_wind && wl.ok;
            g.surfmap.wind_rows = (want_wind && wl.ok) ? wl.rows : std::vector<WindLevel>();
            g.surfmap.wind_block = want_wind ? wl.block : std::string();
            g.surfmap.valid = true;
            g.surfmap.rev++;
            if(g.surfmap.in_flight > 0) { g.surfmap.in_flight--; }
        };
    });
}
