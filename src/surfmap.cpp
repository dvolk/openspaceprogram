// surfmap.cpp -- the Surface Map's pixel buffer (the projection + shading
// math is in surfmap.h). One pixel per (lon, lat) of the equirectangular
// grid over the body's ROTATING frame. Runs on the background worker.

#include "surfmap.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
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
    /* Is this picture a coloured scalar field at all? Read from the table rather than written
       as "temp or wind", so the refusal paint and the log's albedo columns follow #745's promise
       that a new picture is one row in kSurfMapTypes, not a new pair of conditions here. */
    const bool want_field = kSurfMapTypes[type].scalar_field;

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
    /* Does this picture need the ground's HEIGHT at all? Decided here, on the main thread,
       because #839's cache lookup is main-thread work too and both must agree about it: a wind
       map needs the heights the arrows stand on, and a temperature map needs them only where the
       air profile actually lapses (tl.relief). The worker takes the answer, never the question.
       #841 adds the GROUND picture itself. It has always sampled the height -- once inside
       terrainSurfaceColor for the palette, and again per pixel for the under-sea test -- so it
       was the one picture paying the noise twice and caching nothing, while a temperature bake
       of the same hill reused its grid. Asking for the grid costs no extra SAMPLE on a miss (the
       height was already being sampled once per pixel) -- what is new is a 4-byte-per-pixel store
       and its min/max -- and it buys back a whole sampling pass on a hit.
       A banded body is excluded because its colour answers from the band table. That also drops
       the under-sea test for a banded body that authors a datum, which is authorable and shipped
       nowhere: terrainHeight answers a giant's datum radius, so the old test was true in every
       direction and its ground+ocean picture was one flat sphere in sea_color. Painting its
       bands instead is the deliberate choice here. */
    const bool want_elev = (want_wind && wl.ok) || (want_temp && tl.ok && tl.relief)
                        || (!want_field && !tp.surface.bands);
    /* #839: the heights the previous bake sampled, if they were for this body at this size.
       The ground does not move between bakes and terrainHeight() is the expensive half of the
       whole sweep (the cli.h table: AltoFrio 110 / 481 / 1914 ms at 512 / 1024 / 2048 against
       Meridian's 6.4 / 98.9 ms at 512 / 2048 -- a relief world costs ~17x a flat one), so
       re-sampling it for a temperature picture of the hill the ground picture already
       measured -- or for the same hill under a different knob -- is the 17x, not the law.
       A shared_ptr copy out, an immutable grid in, and the slot is only ever read or written
       here and in the continuation: no lock, and a job still running holds its own grid while
       the slot moves on. Keyed by name (see SurfmapState's #839 block for why not a pointer). */
    std::shared_ptr<const std::vector<float> > elev_hit;
    double elev_hit_lo = 0.0, elev_hit_hi = 0.0;
    if(want_elev && g.surfmap.elev_grid && g.surfmap.elev_w == w && g.surfmap.elev_h == h
       && g.surfmap.elev_name == body_name) {
        elev_hit = g.surfmap.elev_grid;
        elev_hit_lo = g.surfmap.elev_lo;
        elev_hit_hi = g.surfmap.elev_hi;
    }
    const double t_now = g.time;
    const int epoch = g.cache_epoch;      // drop the map if the world changes

    g.surfmap.in_flight++;   // the window's "mapping ..." state
    // `g` is captured by REFERENCE only for the returned continuation; the
    // worker body never touches game state (a by-value capture would copy
    // the non-copyable Game).
    g.jobs.post("Surface map", [&g, tp, sea_r, sun_dir, w, h, baked, has_sun, ocean,
                                type, level = g.surfmap.level, want_temp, tl,
                                want_wind, wl, want_field, want_elev,
                                elev_hit, elev_hit_lo, elev_hit_hi,
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
        /* What the sweep costs, printed as sweep_ms=. The bake size is a user-facing knob
           (--surfmap-n) and the only honest way to choose its default is to measure it: the
           picture is w*h = n^2/2 pixels of terrain sample plus a windAt each, so the cost
           scales with n^2 and a default picked by feel will age badly. */
        const auto t_sweep0 = std::chrono::steady_clock::now();
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
           of the swing and a stale number here would be a lie with a decimal point on it). The
           weight is cos(lat), which is one of the grid's per-row table values (EquirectTrig,
           #802) rather than something the pixel recomputes, so it costs nothing. min/max stay
           unweighted -- extremes, not means. */
        double tmin = 1e30, tmax = -1e30, tsum = 0.0, t4sum = 0.0, tw = 0.0;
        /* [deg] Hour angle of the hottest pixel, east of the picture's subsolar meridian (#794).
           With no lag the model's answer for this is 0, so the number is a check on the geometry;
           with one it IS the feature -- and its SIGN is what no still picture of the law alone can
           check, because a flipped hour angle mirrors the pattern instead of moving it. On a body
           with relief this is the PAINTED peak, which the ground's own warmth can pull away from
           noon; --surfmap-log's relief= column says when that is possible. */
        double peak_deg = 0.0;
        /* The T^4 mean of the SAME field with the relief term taken back off (#793), which is the
           accumulator the anchor theorem owns: <T^4> = anchor^4 is a statement about the datum
           ground, and it has to stay checkable now that the painted field is not that field. */
        double t4sum_datum = 0.0;
        /* Wind stats, same shape and the same area weighting (cos lat) as the temperature
           block above: what the sweep actually painted, so the log can show the ground's reach
           against the table's ends rather than assuming they meet. */
        double wmin = 1e30, wmax = -1e30, wsum = 0.0, wwt = 0.0;
        /* Arrow cells drawn by the pass below, counted where they are drawn. A wind map whose
           grid silently draws nothing (a stride bug, a refusal that painted anyway) is otherwise
           invisible to every reader but one standing in front of the screen. */
        int wind_cells = 0;
        /* How many distinct LATITUDE bands the table has, how many rows of arrows the pass below
           drew, and which of the two row sets that was. All three go to the log because they are
           meant to differ: a banded table draws one ring per band, and one whose bands are too
           many or too close -- or too FEW -- falls back to the uniform grid. The two counts cannot
           say which fallback fired (too few and too many both end at the grid's row count), so
           wind_rows_from= names the path out loud. */
        int wind_bands = 0, wind_arrow_rows = 0;
        bool wind_from_bands = false;
        /* The wind ramp's ends, and the ground's floor and ceiling they are priced from
           (#769). The file's rows are not the picture's scale: Venus states 1 m/s at the datum
           and 100 m/s at 65 km, and its highest ground is 2.7 km, so colouring against the rows
           spends the whole palette on air nobody can stand in and paints one flat colour where
           the interesting variation is. So the sweep takes the elevations it is about to colour
           first, and the ramp spans what they can reach.
           Storing them costs 4 bytes a pixel and buys the sweep back the work: terrainHeight()
           -- the expensive half, it is noise -- then runs exactly once per pixel, and the arrow
           pass below reads this grid instead of re-sampling. windAt() is a scan over a handful
           of authored rows, so running it twice is nothing next to that.
           Since #839 the grid may not be THIS bake's at all: `elev_hit` is the previous bake's
           sampling of the same body at the same size, handed over as const. `elev_mine` is what
           this bake sampled and is the only thing the continuation publishes, so a hit never
           republishes and a miss publishes exactly once. */
        std::shared_ptr<std::vector<float> > elev_mine;
        const std::vector<float> *elevs = nullptr;   // whichever grid this bake reads
        double rlo = 0.0, rhi = 0.0;
        /* The temperature ramp's ends, and what the relief term actually cost this picture.
           Seeded with the LAW's ends, not 0: a body with no profile gradient never samples a height
           for the sake of a zero, and it must still paint its real span. Seeding 0 there was a
           live bug -- every airless world came out `temp_ramp=[0.00 0.00]`, so `trhi > trlo` was
           false for every pixel and the whole sphere took the midpoint colour while the cursor
           printed real kelvin (Moho: temp=[137.88 295.67 565.97], one flat green). The invariant
           below the grid pass is what stops it coming back. */
        double trlo = tl.lo_K, trhi = tl.hi_K;
        double rel_at_lo = 0.0, rel_at_hi = 0.0;
        /* The grid's trig, once for every pass below (#802). Each pass visits every column of
           every row, so the equirect grid needs w + h transcendental evaluations in total, not
           w*h per pass; equirect.h owns both spellings. The bit-for-bit guard on this is
           tests/test_surfmap.cpp, which compares the two spellings at every index of several
           grids -- e2e/render/745 checks this picture's aggregates (its coldest pixel against
           the night end, the cos(lat)-weighted T^4 mean against the anchor) at 0.02..0.05 K,
           which is ~10^11 ulps of slack and would not notice a re-baseline. */
        const EquirectTrig trig = equirectTrig(w, h);
        /* #793 added a second reason to want this grid: a relief-aware ground. The two uses share
           one pass because terrainHeight() -- the noise -- is the expensive half of the whole
           sweep, and a pixel that needs its height twice pays for it once. */
        double elo = 0.0, ehi = 0.0;
        if(want_elev) {
            if(elev_hit) {
                /* #839: some earlier bake sampled this body at this size. The floor and ceiling
                   travel WITH the grid, because the palette ends are priced from them just below
                   -- a hit that forgot them would paint the same ground on a different scale. */
                elevs = elev_hit.get();
                elo = elev_hit_lo; ehi = elev_hit_hi;
            } else {
                auto grid = std::make_shared<std::vector<float> >();
                grid->resize(npx);
                elo = 1e300; ehi = -1e300;
                for(int j = 0; j < h; j++) {
                    for(int i = 0; i < w; i++) {
                        /* #841: the grid stores the height the sampler ANSWERS (m from the body
                           centre), not the elevation above the datum. Every reader subtracts the
                           datum itself, so a cached grid and a fresh sampling are the same float
                           in the same place -- the colour pass below compares this value to
                           sea_r and feeds it to the palette, and re-deriving an absolute height
                           from a stored above-datum float would round twice and could move a
                           coastline or a palette band between a hit and a miss.
                           The cost of that choice is honest to name: the relief and wind readers
                           now subtract in double from a float sitting near the body's radius, so
                           their elevation is quantised at THAT float's ulp (0.0625 m at 600 km)
                           rather than at the ulp of a small above-datum number. Through relief_K
                           that is ~3e-4 K and through the ramp ~1e-7 of a stop -- under every
                           printed digit, and the floor/ceiling they price are unaffected because
                           both are computed in double from this same value. */
                        const float hgt = terrainHeight(
                            (glm::vec3)equirectDirAt(trig, i, j), tp);
                        (*grid)[(size_t)j * w + i] = hgt;
                        const double e = (double)hgt - sea_r;
                        if(e < elo) { elo = e; }
                        if(e > ehi) { ehi = e; }
                    }
                }
                elev_mine = grid;
                elevs = grid.get();
            }
            assert(elevs != nullptr && (size_t)elevs->size() == npx);
            if(want_wind && wl.ok) { windRampEnds(wl.rows, elo, ehi, rlo, rhi); }
            if(want_temp && tl.ok) {
                /* Computed for every relief-bearing body, lagged or not: it is the law's span
                   moved by the ground's reach, and its asserts are the ones that catch a profile
                   pointed the wrong way. A LAGGED body's palette comes from the measured field
                   below and overwrites these -- the pairing here would price a cold end at "the
                   darkest light on the tallest mountain", and on a lagged world those two never
                   stand in the same place. */
                tempRampEnds(tl, elo, ehi, trlo, trhi);
                rel_at_lo = tl.relief_K(elo);  // 0 at or below the datum (drag.h's flat column)
                rel_at_hi = tl.relief_K(ehi);  // <= 0: the cost lives at the top
            }
        }
        /* #794: a body whose ground LAGS its star needs its LIGHT before it needs its colour.
           A lag is a filter along longitude, so the picture's hot end is no longer the law's day
           end and its palette cannot be priced from cos_inc 0 and 1; and the per-pixel answer is
           no longer a cosine at all. So: one GroundLagRow per latitude (its terminator, its
           endpoint constants and 1/(1+tau^2) are all latitude-only), one at() per column, and the
           ANSWER -- kelvin, relief already folded in -- kept for the colour pass below.
           Keeping kelvin rather than the forcing is what makes the ramp honest on a body that has
           BOTH a lag and relief. Pricing the cold end as "coldest light at the highest ground"
           (what the forcing extremes alone would say) invents a place: a lagged world's minimum
           forcing lives only in its dark polar rows, which are not where its tallest mountain
           stands. Measured on a lapse-bearing scratch body, that pairing left 8.0 K of palette no
           pixel could reach (temp=[107.19 ...] against temp_ramp=[99.18 ...]), which is the #793
           bug wearing the new knob's clothes: a labelled colour the world never wears.
           It runs ONLY for a body that authored a tau -- seven in ksp_system.json, and no body
           in the four solar corpora, which the pricing pass has not looked at yet. Everywhere
           else the sweep stays the single pass it always was, so nothing that has ever been
           painted moves by one bit. Measured at --surfmap-n 2048, the pre-pass costs nothing
           worth naming (Dres 98.2 ms against Moho's 102.3 ms, unlagged): the row is solved once
           per latitude band, not once per pixel. */
        const bool lag = want_temp && tl.ok && lagTauActive(tl.lag_tau) && has_sun;
        /* has_sun is part of the gate, not decoration: the row model needs a declination, and a
           body with no star has none. Such a body takes the unlagged path, whose floored cosine
           answers 0 in every direction, so its whole sphere reads night -- which is the right
           picture, where a lag solved against a declination of nowhere would paint a day arc no
           star is causing. */
        std::vector<float> tlag;
        double sun_lon = 0.0, sun_dec = 0.0, ylo = 0.0, yhi = 0.0;
        if(want_temp && tl.ok && has_sun) {
            /* Where the baked picture's noon is -- the same reading the calendar's solar row
               takes, on the same vector the terminator was baked from. Read for every
               temperature map, lagged or not: the log prints it, and `peak_deg` below is an
               hour angle against it. No star means no meridian, so the pair stays unset and
               peak_deg prints as an absence rather than a bearing measured from nowhere. */
            subsolarLonLat(sun_dir, sun_lon, sun_dec);
        }
        if(lag) {
            tlag.resize(npx);
            ylo = 1e30; yhi = -1e30;
            trlo = 1e30; trhi = -1e30;
            for(int j = 0; j < h; j++) {
                /* trig.lat is the grid's own row angle -- the same expression equirectDirAt
                   builds from -- so the row the picture paints is defined against the picture
                   rather than against a latitude the sweep only approximates. */
                const GroundLagRow row = groundLagRow(trig.lat[(size_t)j], sun_dec, tl.lag_tau);
                for(int i = 0; i < w; i++) {
                    const size_t idx = (size_t)j * w + i;
                    const double y = row.at(trig.lon[(size_t)i] - sun_lon);
                    if(y < ylo) { ylo = y; }
                    if(y > yhi) { yhi = y; }
                    const double off = tl.relief_K(want_elev ? (double)(*elevs)[idx] - sea_r
                                                             : 0.0);
                    const float T = (float)groundTemperatureAt(tl.anchor_K, tl.knob, y, off);
                    tlag[idx] = T;
                    /* Extremes taken from the STORED value, so the ramp's ends are bit-for-bit
                       the kelvin the colour pass below reads: `the picture reaches both of its
                       own ends` is then a fact about the same numbers the log prints, not a
                       statement about two paths that round differently. */
                    const double Tv = (double)T;
                    if(Tv < trlo) { trlo = Tv; }
                    if(Tv > trhi) { trhi = Tv; }
                }
            }
            /* A unit-mass positive filter of a forcing that never exceeds 1 stays in [0,1]. Out
               of that box is the closed form going wrong, not a bright day. */
            assert(yhi >= ylo && ylo >= 0.0 && yhi <= 1.0 + 1e-9);
        }
        if(want_temp && tl.ok) {
            assert(trhi >= trlo);
            /* A body with NEITHER a height term NOR a lag MUST paint exactly the law's span:
               that is the #793 bug's shape -- the ends were seeded with 0, so every airless
               world came out `temp_ramp=[0.00 0.00]` and its whole sphere took the midpoint
               colour, on the one path no e2e case baked. Relief moves the ends by the ground's
               reach; a lag moves them inward. Either way they must still be a span.
               (The containment that the ramp exists to guarantee -- every painted pixel inside
               it -- is asserted where the pixels are counted, below, because that is the only
               place that knows what was painted.) */
            assert(tl.relief || lag || (trlo == tl.lo_K && trhi == tl.hi_K));
        }

        for(int j = 0; j < h; j++) {
            for(int i = 0; i < w; i++) {
                const glm::dvec3 d = equirectDirAt(trig, i, j);
                glm::vec3 c;                 // the base pixel's colour
                if(want_temp && tl.ok) {                    /* The SAME cosine the terminator overlay uses, through the SAME law the
                       dossier and the solar log use -- nothing is invented at the pixel. */
                    const double elev = want_elev ? (double)(*elevs)[(size_t)j * w + i] - sea_r
                                                  : 0.0;
                    const double off = tl.relief_K(elev);   // 0.0 whenever relief is off
                    /* A lagged body reads the kelvin the pass above solved; an unlagged one
                       reads solar.h's cosine of the direction, exactly as it always has. Both go
                       through groundTemperatureAt, so the lag is the ONLY difference between the
                       two paths -- and the cursor readout in gameui.cpp takes the same pair,
                       which is what keeps a colour and the kelvin beside it one world. */
                    const double T = lag ? (double)tlag[(size_t)j * w + i]
                                         : surfmapGroundT(tl.anchor_K, tl.knob, d, sun_dir, off);
                    // Ramp span: the model's ends moved to the ground the sweep is colouring
                    // (tempRampEnds above) and, for a lagged body, to the kelvin this grid can
                    // actually reach (measured in the pass above). Only a body with no swing at
                    // all -- no relief, no lag, and an even share that flattens the field -- has
                    // lo == hi, and then the span is 0 and every pixel takes the midpoint: one
                    // honest flat colour rather than a gradient the eye invents (#726's "no
                    // swing" is that state, and the assert above keeps a relief-free body from
                    // reaching it by accident).
                    const double t01 = (trhi > trlo) ? (T - trlo) / (trhi - trlo) : 0.5;
                    unsigned char r = 0, gg = 0, bb = 0;
                    surfmapRamp(t01, r, gg, bb);
                    c = glm::vec3(r / 255.0f, gg / 255.0f, bb / 255.0f);
                    if(T < tmin) { tmin = T; }
                    if(T > tmax) {
                        tmax = T;
                        peak_deg = std::remainder(trig.lon[(size_t)i] - sun_lon,
                                                  2.0 * std::numbers::pi) * 180.0
                               / std::numbers::pi;
                    }
                    const double cw = trig.cos_lat[(size_t)j];   // cos(latitude)
                    tw += cw;
                    tsum += cw * T;
                    t4sum += cw * T * T * T * T;
                    /* The same mean with the relief term taken back off (#793). The anchor theorem
                       is about the datum field, so this is the accumulator that must land on the
                       anchor; temp_mean4 is allowed to sit below it, and the log prints both so
                       the gap is a measurement rather than a drift nobody named. */
                    if(off != 0.0) {
                        const double Td = T - off;
                        t4sum_datum += cw * Td * Td * Td * Td;
                    } else {
                        t4sum_datum += cw * T * T * T * T;
                    }
                } else if(want_wind && wl.ok) {
                    /* The scalar half of the field: how fast the air moves OVER THIS PIXEL.
                       The altitude windAt() wants is this pixel's ground elevation above the
                       sea-level datum -- the same coordinate the Surface window's Wind: row
                       asks with and the aero law reads -- so a mountain pokes into faster air
                       and shows up brighter, and a basin does not get credited with the wind
                       of the ridge above it. The arrows go on afterwards; a scalar alone is not
                       a vector field (#769). And since #767 the pixel's own place matters too:
                       a row can answer for one band of the body and not this pixel. */
                    const double elev = (double)(*elevs)[(size_t)j * w + i] - sea_r;
                    double lon_deg = 0.0, lat_deg = 0.0;
                    equirectLonLatDeg(d, lon_deg, lat_deg);
                    const glm::dvec3 wv = windAt(wl.rows, elev, lat_deg, lon_deg);
                    /* The picture is a HORIZONTAL field: the arrows point along the ground and
                       the colour says how fast the air crosses it. The rising component (#767)
                       is deliberately not folded into the speed -- a cm/s downdraft painted as
                       wind would read as air moving sideways, which is the wrong news. */
                    const double spd = std::hypot(wv.x, wv.y);
                    unsigned char r = 0, gg = 0, bb = 0;
                    surfmapRamp(surfmapWindT01(spd, rlo, rhi), r, gg, bb);
                    c = glm::vec3(r / 255.0f, gg / 255.0f, bb / 255.0f);
                    if(spd < wmin) { wmin = spd; }
                    if(spd > wmax) { wmax = spd; }
                    const double cw = trig.cos_lat[(size_t)j];   // cos(latitude)
                    wwt += cw;
                    wsum += cw * spd;
                } else if(want_field) {
                    /* Asked for a temperature this world does not have, so the picture says
                       so: a flat neutral field, with the reason in the window's legend row.
                       Falling back to the albedo here would put a ground picture under a
                       "temperature" label, which is the map lying about what it is (#727's
                       law, applied to a picture rather than a number). A wind map refuses the
                       same way -- and the two refusals say WHICH absence in the log's note= and
                       on the cursor row, not here (#726). */
                    c = glm::vec3(0.34f, 0.34f, 0.34f);
                } else if(want_elev) {
                    /* Ocean map on and at/below sea level: flat sea covers the floor
                       (same coverage as the 3D ocean shell, and the same predicate as
                       a Flat-sea body's painted sea_color floor -- issue #193).
                       #841: the height the grid pass already measured does both jobs. The sea
                       test is a comparison against the same hoisted datum the grid's floor and
                       ceiling were taken from, and the palette reads the very float
                       terrainSurfaceColor would have sampled for this direction -- the same
                       direction table, the same captured params, the same jitter expression -- so
                       this picture is bit-for-bit what it was, at one height sample per pixel
                       instead of two, and NO height sample at all on a cache hit (the colour
                       jitter still runs per pixel). elevs is non-null exactly when want_elev is,
                       and the assert below the grid pass says so. */
                    const float hgt = (*elevs)[(size_t)j * w + i];
                    const bool under_sea = ocean && tp.surface.has_sea
                        && (double)hgt <= sea_r;
                    c = under_sea
                        ? tp.surface.sea_color
                        : terrainSurfaceColorH((glm::vec3)d, tp, hgt);
                } else {
                    // Only a banded body reaches here (see want_elev above): its colour comes
                    // from the band table and never reads a height, so there is no sample to
                    // hand over.
                    c = terrainSurfaceColor((glm::vec3)d, tp);
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
        if(want_temp && tl.ok) {
            /* THE RAMP SPANS THE PICTURE. Not a tautology on the lagged path: there the span is
               measured and the pixels are painted from the same numbers, and on the relief path
               it depends on relief_K() getting no warmer as the ground rises -- a monotonicity
               tempRampEnds has always assumed and now says out loud. If this ever fires, a colour
               on the legend is a colour no pixel wears (or a pixel clamps to one off the end of
               its own scale), which is the #793/#769 bug class rather than a rounding argument.
               The slack is for the float the elevation grid and the lagged kelvin are stored in,
               not for the maths: 1e-3 K against a 4-digit print. */
            assert(tw > 0.0);
            assert(tmin >= trlo - 1e-3 && tmax <= trhi + 1e-3);
        }
        /* THE ARROWS (#769) -- the vector half. A speed-coloured field says where the air
           moves HARDER and nothing about WHICH WAY, and which way is the half a player asks
           for. They go on afterwards rather than inside the loop above: a base pixel has to be
           one colour, and an arrow is a stroke across several of them.
           On a coarse grid, because the field is sampled -- not a streamline integration. The
           grid is what makes the flat parts visible: every body authored today has one wind
           direction and one altitude axis -- #767 gave the rows a lat/lon band, but no file
           uses it yet -- so the honest picture is a grid of parallel arrows whose length
           changes where the ground changes height.
           The base mean (ar/ag/ab) above is taken BEFORE this: those columns describe the
           field, not the ink drawn on it. (They dash for a wind map anyway.) */
        if(want_wind && wl.ok) {
            const int stride = std::max(8, w / 32);   // ~32 cells across, whatever the bake size
            const double half = stride * 0.42;        // arrow half-length (SHAFT, not reach)
            /* The HEAD is a fixed fraction of the CELL, not of the shaft. Scaling it with the
               shaft (this used to be L * 0.45) puts it under a pixel at the default bake: at
               --surfmap-n 256 the stride is 8, so the head was 0.53..1.51 px and the two 150-deg
               strokes landed ON the shaft or one pixel beside it -- a crossbar at the tip rather
               than a head, missing altogether on the slowest air. A constant head costs some of
               the length cue, and buys the direction cue that the arrows exist for (#780). The
               cell then holds an arrow whose REACH is half + head = 0.72 * stride, so on an
               oblique wind neighbouring arrows' ink can touch; that is the price of a head big
               enough to see, and zonal ink (the common case) stays well inside. */
            const double head = stride * 0.30;
            /* ONE ink for the whole picture, decided here rather than per cell. The per-cell
               version sampled the ramp under the cell and switched to near-black ink where the
               field was bright, because a white stroke on the yellow stop is weak -- and the
               yellow stop is where the fastest air is. It cost more than it bought: the ink then
               varied TWICE across the ramp (luma crosses the switch at t01 0.468 and back at
               0.884, since the hot end is a dark red), so a black arrow read as a different KIND
               of arrow rather than as the same arrow over a bright patch. If the yellow stop ever
               does swallow an arrow, darken the STOP -- the ramp is ours -- rather than switch
               ink mid-map. */
            const unsigned char kr = 245, kg = 248, kb = 255;
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
            /* WHICH rows draw. One row per latitude band when the table has the structure for
               it, the uniform grid otherwise; the rules and the two bounds that fight over it
               (too few bands / too many, with the minimum asked of rows DRAWN) live in
               surfmapArrowRows (surfmap.h) where a test can reach them without a game. This
               pass just draws what it returns, and the log says which of the two drew. */
            const ArrowRowPlan plan = surfmapArrowRows(wl.rows, h, stride, half, head);
            const std::vector<int> &use_j = plan.from_bands ? plan.rings : plan.grid;
            wind_bands = plan.bands;
            wind_arrow_rows = (int)use_j.size();
            wind_from_bands = plan.from_bands;
            const double ha = 5.0 * std::numbers::pi / 6.0;   // 150 deg back from the tip
            const double ca = std::cos(ha), sa = std::sin(ha);
            for(int j : use_j) {
                for(int i = stride / 2; i < w; i += stride) {
                    const glm::dvec3 d = equirectDirAt(trig, i, j);
                    const double elev = (double)(*elevs)[(size_t)j * w + i] - sea_r;  // sampled once
                    double lon_deg = 0.0, lat_deg = 0.0;
                    equirectLonLatDeg(d, lon_deg, lat_deg);
                    const glm::dvec3 wv = windAt(wl.rows, elev, lat_deg, lon_deg);
                    const double spd = std::hypot(wv.x, wv.y);   // the arrow is a ground vector
                    const double cl = trig.cos_lat[(size_t)j];   // cos(latitude)
                    const glm::dvec2 pv = surfmapWindPixelVec(wv.x, wv.y, cl);
                    const double pvl = glm::length(pv);
                    const double t01 = surfmapWindT01(spd, rlo, rhi);
                    wind_cells++;   // every cell draws something: an arrow, or a dot for calm air
                    if(spd <= 0.0 || pvl <= 0.0) {
                        /* Calm air gets a dot, not an arrow. A direction drawn for a wind of
                           zero -- or a headless dash for one that has no horizontal component at
                           all -- is a reading the file never gave (#727). */
                        put(i, j, kr, kg, kb);
                        continue;
                    }
                    const glm::dvec2 u = pv * (1.0 / pvl);
                    /* The SHAFT still scales with speed -- that is the field's magnitude cue --
                       with a floor. The barbs reach back head * cos(30 deg) = 0.87 * head from
                       the tip, so the invariant "the barbs cannot run past the tail and make the
                       arrow point away from itself" only needs L >= 0.43 * head; the floor is
                       0.8 * head, deliberately stricter, because below that the head is the
                       whole arrow and a calm cell reads as a blob rather than a direction. The
                       cost is the length cue: at --surfmap-n 256 the shaft spans 3.84..6.72 px
                       instead of 2.35..6.72, so the heatmap carries the speed instead. */
                    const double L = std::max(half * (0.35 + 0.65 * t01), head * 0.8);
                    const double x1 = i + u.x * L, y1 = j + u.y * L;
                    stroke(i - u.x * L, j - u.y * L, x1, y1);
                    /* A head at the tip: two strokes back at 150 deg, so the arrow reads as
                       pointing somewhere rather than as a dash lying across the cell. */
                    stroke(x1, y1, x1 + (u.x * ca - u.y * sa) * head,
                               y1 + (u.x * sa + u.y * ca) * head);
                    stroke(x1, y1, x1 + (u.x * ca + u.y * sa) * head,
                               y1 + (-u.x * sa + u.y * ca) * head);
                }
            }
        }
        if(log) {
            /* What the sweep cost, printed LAST on the line. The log parser in e2e/run.py reads
               the optional temp= and wind= tails by position, so anything appended in the middle
               of the line hides them; sweep_ms= therefore goes after every tail. */
            const double sweep_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_sweep0).count();
            // Base colour means before/after the terminator, then the map's identity and,
            // for a temperature map, what the sweep actually priced. A dash is an absent
            // reading, never a zero (#727): a ground map has no temperatures to report and
            // a world the model cannot price reports none either.
            const bool have_t = want_temp && tl.ok;
            /* A dash is an absent reading, never a zero (#727). Under a temperature or a wind
               map the base picture's colours are RAMP COLOURS, not albedos, so the albedo
               columns dash rather than reporting a meaningless mean of the palette. */
            char alb_s[64], shd_s[64];
            if(want_field) {
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
                    printf(" temp=[%.2f %.2f %.2f] temp_ends=[%.2f %.2f] temp_ramp=[%.2f %.2f]",
                           tmin, tsum / tw, tmax, tl.lo_K, tl.hi_K, trlo, trhi);
                    /* #794, on every temperature map. `lag_tau` is the authored knob in phase
                       radians and DASHES when no file stated one (#727: an absence, not a zero)
                       -- which is what the four solar corpora and everything under
                       LAG_AUTHOR_FLOOR print, and what the ksp case beside this one checks a
                       number against. `peak_deg` is where the picture's hottest pixel actually
                       sits -- degrees east of the subsolar meridian of this bake -- so the claim
                       "the hot spot is under the star" is a number a case can hold the engine
                       to, and the sign of it is the temporal half of the hour-angle convention
                       that the law's own tests cannot see (a flipped h mirrors the pattern
                       rather than moving it). `sun_lon` is that meridian in the grid's own
                       coordinates, so two bakes at two instants can be compared in absolute
                       longitude instead of only against each other.
                       Both dash when the question has no answer: no star means no meridian, and a
                       picture with NO span at all -- no relief, and an even share that flattens the
                       field -- has its "hottest pixel" decided by scan order rather than by physics,
                       so printing +90.07 deg for it would be a lag-sized number invented out of
                       nothing. A relief-bearing body with a flat star field (Venus: even_share 1.0,
                       a 13.5 K lapse) still has a span and still prints a number; there it is the
                       hottest ground, which happens to be the lowest ground, and reading it as an
                       afternoon would be the reader's error rather than the log's. */
                    char lag_s[16], peak_s[24], sun_s[24];
                    if(lagTauAuthored(tl.lag_tau)) {
                        snprintf(lag_s, sizeof lag_s, "%.4f", tl.lag_tau);
                    } else {
                        snprintf(lag_s, sizeof lag_s, "-");
                    }
                    if(has_sun && trhi > trlo) {
                        snprintf(peak_s, sizeof peak_s, "%+.2f", peak_deg);
                    } else {
                        snprintf(peak_s, sizeof peak_s, "-");
                    }
                    if(has_sun) {
                        snprintf(sun_s, sizeof sun_s, "%.2f",
                                 sun_lon * 180.0 / std::numbers::pi);
                    } else {
                        snprintf(sun_s, sizeof sun_s, "-");
                    }
                    printf(" lag_tau=%s peak_deg=%s sun_lon=%s", lag_s, peak_s, sun_s);
                    /* #793, only when there IS relief to price: the grounds this grid actually
                       sampled [m above the datum] and what the profile charges at each end (K).
                       This is the number that says a map's cold end is a mountain and not a longer
                       night. */
                    if(tl.relief) {
                        printf(" relief=[%.0f %.0f %.2f %.2f]", elo, ehi, rel_at_lo, rel_at_hi);
                    }
                    printf(" temp_mean4=%.2f datum_mean4=%.2f anchor=%.2f anchor_src=\"%s\"",
                           std::pow(t4sum / tw, 0.25), std::pow(t4sum_datum / tw, 0.25),
                           tl.anchor_K, tl.legend.c_str());
                } else {
                    printf(" temp=- note=\"%s\"", tl.legend.c_str());
                }
            }
            if(want_wind) {
                if(wl.ok) {
                    /* wind= is what the sweep painted (min, area-weighted mean, max) at the
                       GROUND's altitudes; wind_ends= is what the FILE states at its rows;
                       wind_ramp= is the span the PICTURE's colours and arrow lengths run
                       over, priced from the ground's own floor and ceiling. The gap between
                       the second and the third is the point: Earth's deck row lives at 11 km
                       while its ground tops out nearer 9, and Venus reaches 5% of its own
                       stated span, so a reader can see how much of the file this picture can
                       show instead of being handed a colour with no scale. wind_rows= says how
                       much profile the body authored, which is the difference between a
                       two-row datum and a layered claim (#767 is what would add the other
                       axis). wind_cells= is how many grid cells the arrow pass drew: the
                       vector half has no other readout, and a field that paints its colours
                       but forgets its arrows looks fine. wind_bands= and wind_arrow_rows= come
                       in as a pair: the first is how many latitude bands the table actually has,
                       the second how many rows of arrows were drawn. They cannot say WHICH fallback
                       fired -- a 3-band table and a 31-band table at a coarse bake both end at the
                       grid's row count -- so wind_rows_from= says it: "bands" means the rings landed
                       one per band, "grid" means the uniform lattice drew. */
                    assert(wwt > 0.0);   // every pixel is weighted, and h >= 8 by construction
                    printf(" wind=[%.2f %.2f %.2f] wind_ends=[%.2f %.2f] wind_ramp=[%.2f %.2f]"
                           " wind_rows=%d wind_cells=%d wind_bands=%d wind_arrow_rows=%d"
                           " wind_rows_from=%s",
                           wmin, wsum / wwt, wmax, wl.auth_lo_ms, wl.auth_hi_ms, rlo, rhi,
                           (int)wl.rows.size(), wind_cells, wind_bands, wind_arrow_rows,
                           wind_from_bands ? "bands" : "grid");
                } else {
                    printf(" wind=- note=\"%s\"", wl.legend.c_str());
                }
            }
            /* #839: whether this bake SAMPLED the ground's heights or reused the previous bake's
               grid for the same body at the same size. It goes here, after every tail and before
               sweep_ms=, for the same reason sweep_ms= does: the harness reads the tails by
               position, so a column in the middle hides them. `-` is an absent QUESTION rather
               than a miss: the picture reads no height at all -- a banded body's colour comes
               from its band table, a refused field paints flat grey instead of the ground, and a
               temperature map on air whose profile does not lapse has no relief term to price.
               #841 moved the ground and ground+ocean pictures OUT of that list: their palette and
               their sea test now read one sample per pixel, cached like any other picture. */
            printf(" elev_cache=%s", want_elev ? (elev_hit ? "hit" : "miss") : "-");
            printf(" sweep_ms=%.1f\n", sweep_ms);
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
                elev_mine, elo_pub = elo, ehi_pub = ehi,
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
            /* ... and the lag those pixels were painted with (#794). The cursor's kelvin has to
               be the colour's kelvin: a lagged picture read through the unlagged law answers a
               noon question about an afternoon picture. */
            g.surfmap.temp_lag_tau = have_t ? tl.lag_tau : kLagTauUnauthored;
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
            /* #839: the heights this bake sampled join the slot, so the next bake of this body at
               this size paints from them instead of asking the noise 2M more times. Only a bake
               that SAMPLED publishes: a hit holds the grid that is already there, and
               republishing it would be a shared-pointer assignment that changes nothing.
               elo/ehi travel with it because the palette ends are priced from them, not
               recomputed from the grid.
               Publish AFTER the epoch check above: a map for a world that no longer exists must
               not leave its hills in the slot for the next one to paint. */
            if(elev_mine) {
                g.surfmap.elev_grid = elev_mine;
                g.surfmap.elev_w = w;
                g.surfmap.elev_h = h;
                g.surfmap.elev_name = body_name;
                g.surfmap.elev_lo = elo_pub;
                g.surfmap.elev_hi = ehi_pub;
            }
            g.surfmap.valid = true;
            g.surfmap.rev++;
            if(g.surfmap.in_flight > 0) { g.surfmap.in_flight--; }
        };
    });
}
