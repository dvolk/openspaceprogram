#pragma once
// surfmap.h -- the "Surface Map" window's 2-D projection + shading.
// EQUIRECTANGULAR over the body's ROTATING frame; the direction <-> (lon, lat)
// <-> pixel math lives in equirect.h.

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cctype>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "groundtemp.h"   // the temperature layer's law, and the anchor's rungs
#include "terragen.h"     // Surface: the data the layer decides from
#include "solar.h"        // cosIncidence: the law's ONE floored cosine (#724)

#include "equirect.h"

/* The terminator's light factor, NOT a cosine of the law: a picture wants a soft edge over
   ~0.2 of the cosine with 30 % ambient left in the night, where the temperature law wants a
   hard floor at 0 (groundtemp.h). Same dot product, different purpose -- do not merge them. */
inline float surfmapShade(const glm::dvec3 &n, const glm::dvec3 &sun) {
    const double d = glm::dot(n, sun);
    const double t = glm::clamp((d + 0.05) / 0.20, 0.0, 1.0);
    return 0.30f + 0.70f * (float)t;
}

// Consecutive lon samples cross the antimeridian when their difference
// exceeds half a turn (the orbit line must break there).
inline bool surfmapWraps(double lon_prev, double lon_cur) {
    return std::fabs(lon_prev - lon_cur) > std::numbers::pi;
}

/* A MAP TYPE, and the LEVEL of the world it draws (#745). The sweep used to paint one thing
   -- the albedo, under a terminator overlay -- and the window had two boxes hard-wired to
   that one picture. The temperature field (#724) is the second thing worth drawing, and a
   wind vector field is the third in reach, so the seam goes in now: the table below is what
   the window's two combos read, and a new type is one row here plus whatever buffer it
   publishes in SurfmapState.

   ONE level enum, shared by every type, and its name is "ground": a body's surface and the
   ground under it are one place, and two enum values for that would be two spellings of it
   (#636). Air levels (where the #724 rung D coupling lives) slot in when there is more than
   one to offer. */
enum SurfMapType {
    kMapGround = 0,         // the albedo sweep: bare ground, no sea painted
    kMapGroundOcean,        // the same sweep with the sea painted over the floor
    kMapTemperature,        // the ground's modelled temperature at this instant (#724)
    kMapWind,               // the air's own motion over this ground (#688's data, #769's picture)
    kMapTypeCount
};
enum SurfLevel {
    kLevelGround = 0,     // the ground itself -- today's only level
    kLevelCount
};

struct SurfMapTypeSpec {
    const char *name;             // what the combo offers
    SurfLevel levels[4];          // the levels this type can draw, in menu order
    int n_levels;
    /* Is this picture a SCALAR FIELD coloured by surfmapRamp, rather than the ground's own
       colours? The sweep and the log both have to know, and neither should ask by listing the
       types that happen to be fields today: a new picture is one row here (#745's promise), so
       "paint grey when the field is absent" and "the albedo columns dash" both read this. */
    bool scalar_field;
};

/* A type with ONE level renders that level as a label rather than as a menu offering a
   single choice: a dropdown that cannot be taken up is a widget lying about a choice. */
inline constexpr SurfMapTypeSpec kSurfMapTypes[kMapTypeCount] = {
    { "ground",         { kLevelGround }, 1, false },
    { "ground+ocean",   { kLevelGround }, 1, false },
    { "temperature",    { kLevelGround }, 1, true  },
    { "wind",           { kLevelGround }, 1, true  },
};
inline constexpr const char *kSurfLevelNames[kLevelCount] = { "ground" };

inline const char *surfMapTypeName(int t) {
    assert(t >= 0 && t < kMapTypeCount);
    return kSurfMapTypes[t].name;
}
inline const char *surfLevelName(SurfLevel l) {
    assert(l >= 0 && l < kLevelCount);
    return kSurfLevelNames[l];
}

/* The map type by the name the combo (and --surfmap-map) uses, or -1. Case-insensitive on
   purpose: a CLI name the UI shows with different casing would be a second spelling with a
   second set of typos, so the name in the table is the ONE name and the matcher accepts a
   sloppy case, not a different word. */
inline int surfMapTypeFromName(const std::string &want) {
    std::string w = want;
    for(char &ch : w) { ch = (char)tolower((unsigned char)ch); }
    for(int i = 0; i < kMapTypeCount; i++) {
        std::string n = kSurfMapTypes[i].name;
        for(char &ch : n) { ch = (char)tolower((unsigned char)ch); }
        if(n == w) { return i; }
    }
    return -1;
}

/* The colour ramp for a scalar field: t in [0,1], cold to hot. Deliberately NOT a hue
   rotation -- five hand-picked stops, dark blue at the cold end to dark red at the hot one,
   with the mid stops far enough apart that the ramp is still readable where the field is
   FLAT (a body with no swing paints one honest colour, not a gradient the eye invents).
   Endpoints and the "does not stall" property are pinned in tests/test_surfmap.cpp. The two
   temperatures the ends stand for are NOT on this window -- a row there costs the map its own
   aspect ratio -- they are --surfmap-log's temp_ramp= and the atlas' `ground swing` row (the
   law's ends at the datum) / `ground at peak` row (the same law where the relief puts it), and
   what the window does print is the value at the point under the cursor. */
inline void surfmapRamp(double t, unsigned char &r, unsigned char &g, unsigned char &b) {
    struct Stop { double t; unsigned char r, g, b; };
    static const Stop kStops[] = {
        { 0.00,  30,  48, 120 },
        { 0.28,  52, 140, 205 },
        { 0.52,  70, 190, 120 },
        { 0.76, 235, 195,  70 },
        { 1.00, 200,  45,  35 },
    };
    const int n = (int)(sizeof(kStops) / sizeof(kStops[0]));
    double c = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
    int i = 0;
    while(i < n - 2 && c > kStops[i + 1].t) { i++; }
    const Stop &a = kStops[i], &b_stop = kStops[i + 1];
    const double span = b_stop.t - a.t;
    const double f = span > 0.0 ? (c - a.t) / span : 0.0;
    r = (unsigned char)(a.r + f * (b_stop.r - a.r));
    g = (unsigned char)(a.g + f * (b_stop.g - a.g));
    b = (unsigned char)(a.b + f * (b_stop.b - a.b));
}


/* What the temperature layer can do for a body (#745), decided ONCE per bake on the main
   thread -- the worker gets these numbers, never body pointers, exactly like the albedo sweep
   gets a TerrainParams snapshot instead of the live body.

   The anchor and the ramp's ends come from the SAME helpers the dossier and the solar log use
   (groundtemp.h), so a map and a dossier cannot price one world two ways (#687). The sweep
   then reports both the ends aimed for and what it actually painted, and the two closing is a
   fact an e2e case checks on real pixels rather than on a quadrature.

   `derived_bb` is passed in rather than looked up, because only the caller knows WHICH
   instant's light to price: the map prices the body's actual rail (the whole picture is one
   instant), the atlas prices its mean orbit. On an eccentric rail those differ by the rail's
   own spread, and that belongs at the call site, not hidden in a policy function. */
struct TempLayer {
    bool ok = false;
    double anchor_K = 0.0, knob = kEvenShareUnauthored;
    /* [phase rad] #794: how far this body's ground lags its star, as authored
       (Surface::lag_tau). The sentinel and a stated 0.0 both mean "the ground tracks the light",
       which is what every shipped body does today; only lagTauActive() makes the sweep trade its
       per-pixel cosine for a lagged row. It rides on the layer because the picture and the
       number beside it are one decision: a map painted with a lag and read out without one is
       #793's mistake pointed the other way. */
    double lag_tau = kLagTauUnauthored;
    /* [K] The model's own night / day ends AT THE DATUM. These are the two figures the atlas
       prints as `ground swing` and the log prints as temp_ends=, and they are relief-free on
       purpose: they are the law's ends, tied to the anchor and the knob by an identity a case
       checks (e2e science/724). The PICTURE's ends, which include relief, are temp_ramp= and come
       from the sweep, which is the only place that knows the ground's reach (#769 set this
       precedent for the wind ramp). A lagged body moves the PICTURE's ends inwards as well
       (#794: a first-order low-pass flattens the row), which is exactly why these stay what they
       are -- they are what the file's one knob forces, and the row that checks that identity must
       not be the row that describes a palette. */
    double lo_K = 0.0, hi_K = 0.0;
    /* #793: the body's OWN air profile, copied for the worker exactly like WindLayer copies the
       wind rows -- the worker gets numbers and tables, never body pointers. The ground's height is
       read against this, so a map pixel's temperature and the kelvin beside it price the same
       mountain the same way (#687). */
    std::vector<AtmoLayer> profile;
    double air_top = 0.0;
    /* Is relief worth anything here at all? False for no physical air and for a profile that never
       lapses (an isothermal column has no height term), which is also the answer for an airless
       body -- and it is what lets the sweep skip sampling a heightfield it would only multiply by
       zero. */
    bool relief = false;
    /* [K] The relief term for one ground elevation [m above the sea-level datum]. ONE spelling,
       used by the sweep and the tests: the worker gets this copy of the profile, while the cursor
       and the other readouts go through Surface::relief_K (same arithmetic, one copy of the air in
       the body). */
    double relief_K(double elev_asl) const {
        return relief ? airProfileOffset_K(profile, air_top, elev_asl) : 0.0;
    }
    /* The sentence for --surfmap-log: the anchor's PROVENANCE when the layer paints (printed
       as anchor_src=, "the air's datum", "derived: sun + albedo" -- #722's rule that
       a computed figure announces itself at the readout), and the REASON it does not when it
       does not ("no authored swing", "a banded giant", "no sun"). `block` is the same decision
       in three words, for the cursor line, which has room for a word and not a sentence. */
    std::string legend;
    /* The same refusal in THREE WORDS, for a readout that has no room for the sentence: the
     cursor row says `ground - (no swing)` rather than a bare dash, so #726's rule -- that the
     different absences stay different states -- survives the trip to a cramped line. Empty when
     the layer paints. */
    std::string block;
};

inline TempLayer tempLayerFor(const Surface &s, double derived_bb, bool has_sun) {
    TempLayer tl;
    tl.knob = s.even_share;
    tl.lag_tau = s.lag_tau;
    const double airT = airDatum_K(s.atmosphere);
    const GroundAnchorRung rung = groundAnchorRung(s.temperature, airT, derived_bb);
    tl.anchor_K = groundAnchor_K(s.temperature, airT, derived_bb);
    // One reason, in the order a reader would ask for it, as a sentence rather than a code:
    // the window shows this where the map would be, so "no data" and "no ground for there to
    // be data about" have to read apart (#726). The giant test matches the atlas' own
    // (gameui.cpp): the file gives a banded world air at levels and no ground at all, so a
    // ground temperature is not an absent reading, it is a question with no subject (#738).
    if(!has_sun) {
        // Ahead of the others: with no star there is no lit side, so there is no cosine to put
        // through the law at all -- the field is grey and the line says "no sun" rather than
        // agreeing with the other absences (#726).
        tl.legend = "no star in range to light this world";
        tl.block = "no sun";
    } else if(rung == kAnchorNone) {
        tl.legend = "nothing anchors this world's ground: no stated temperature, no air "
                    "datum, no albedo to derive one from";
        tl.block = "no anchor";
    } else if(!evenShareAuthored(tl.knob)) {
        // Reachable now only for a hand-authored file: #749's passes price a knob for every
        // ground they can anchor, and a body with no anchor is refused one rung earlier. So a
        // dash here is a DECLINED ground, and the sentence says which of the two it is not.
        tl.legend = "no authored swing: this ground has an anchor and no knob, which is a "
                    "decision rather than a gap";
        tl.block = "no swing";
    } else {
        tl.ok = true;
        // A banded world gets a temperature like every other world, and the legend says which
        // level it is. The file authors its surface AT the 1 bar (#689 makes sea_level that
        // datum), so the anchor the ladder picks IS a level of that air, and "the ground" here
        // means the level the file calls the surface -- not a sea floor nobody can find. The
        // branch that used to sit above this one dashed five worlds with the words "no ground",
        // which read as a hole in the data on the best-measured airs in either catalogue.
        tl.legend = s.bands
            ? std::string("the 1 bar level, not a sea floor: ")
                  + groundAnchorSource(rung, s.temperature_src)
            : groundAnchorSource(rung, s.temperature_src);
    }
    if(tl.ok) {
        tl.lo_K = groundTemperature(tl.anchor_K, tl.knob, 0.0);
        tl.hi_K = groundTemperature(tl.anchor_K, tl.knob, 1.0);
        /* #793: the profile this body's ground stands in. Copied for the worker (numbers and
           tables, never body pointers), and gated by Surface::reliefOn() so the sweep does not
           sample a heightfield for a term that would be zero: no physical air (the limb-only rims
           of Io and Triton), a column that never lapses, or a banded giant. */
        tl.relief = s.reliefOn();
        if(tl.relief) {
            tl.profile = s.atmosphere.layers;
            tl.air_top = s.atmosphere.top();
        }
    }
    return tl;
}

/* [K] The temperature ramp's ends for one body's GROUND (#793): the law's two ends moved by what
   the profile says at the highest and lowest ground the sweep will colour. Same shape as
   windRampEnds (#769), for the same reason -- a span that does not cover what the picture can
   paint puts pixels off the end of the palette, and a relief-aware field can reach BELOW the
   analytic night end by lapse x height: Earth's 6.5 K/km against 8 km of relief is 52 K, so half
   the palette would paint nothing.
   The extremes are analytic, not sampled: the coldest pixel is the night end standing on the
   highest ground, the hottest is the day end on the lowest, because the relief term does not
   depend on the star. relief_K clamps below the datum to 0 (drag.h's flat sub-datum column), so
   on a body whose lowest ground is under the sea level the hot end is the analytic one. */
inline void tempRampEnds(const TempLayer &tl, double elev_lo, double elev_hi,
                         double &ramp_lo_K, double &ramp_hi_K) {
    assert(elev_lo <= elev_hi);
    /* The pairing above assumes it: the coldest reachable pixel is the night end on the HIGHEST
       ground, which is only true if the profile never gets warmer as the ground rises. Every
       shipped lapse is negative below its first inversion and terrainHeightFade keeps ground
       under the layer tops, so it holds -- but the sweep's containment assert reads the same way,
       and an authored positive-lapse layer near the datum would make both wrong at once. */
    assert(tl.relief_K(elev_hi) <= tl.relief_K(elev_lo));
    ramp_lo_K = tl.lo_K + tl.relief_K(elev_hi);
    ramp_hi_K = tl.hi_K + tl.relief_K(elev_lo);
    // >= 0, not > 0: a body that authors even_share 0 has a 0 K night end from the bare law, and
    // the sweep divides by (hi - lo) only when the two differ. A NEGATIVE end is the data guard.
    assert(ramp_lo_K <= ramp_hi_K && ramp_lo_K >= 0.0);
}

/* [K] The temperature the map paints in ONE direction: the same arithmetic the sweep runs per
   pixel and the window's cursor readout runs for the point under the mouse. Shared because the
   number beside a pixel has to BE that pixel's number -- and because the sun it uses is the
   one the picture was BAKED with, not the sim's sun now (the map is a snapshot; frames move
   underneath it). #687's rule that a figure is measured or recomputed, never both at different
   instants.
   The cosine is solar.h's: groundtemp.h says cos_inc comes from that floored cosine and nothing
   re-derives it, and this was the one place that did. Both vectors go onto the unit sphere
   first, because a direction is a BEARING and its length must not reach the law. It used to:
   the old clamp sat on the DOT PRODUCT, so a direction 3x too long priced as three times the
   cosine of its angle -- saturating to the day end inside ~70 deg of the star, reading 80 deg
   as 59 outside it. Normalising also absorbs the cursor's float round-trip (gameui.cpp widens
   a glm::vec3, unit to ~1e-7, worth ~4e-6 K) and keeps an over-long sun vector away from
   groundTemperature's cos_inc assert. A zero-length vector stays zero: no bearing to normalise,
   and the floor's night end is the honest answer for it. Only the unit test feeds a direction
   that is deliberately off the sphere -- the sweep (equirectDirAt) and the baked sun
   (starFromBody) are unit by construction.
   `relief_K` is TempLayer::relief_K at this point's elevation above the datum (#793): the number
   beside a pixel has to be that pixel's number, so the cursor passes the same term the sweep used
   for the pixel under it, and a caller that has no height to offer passes 0.0 -- the datum, which
   is what every map before #793 painted. */
inline double surfmapGroundT(double anchor_K, double even_share, glm::dvec3 dir,
                             glm::dvec3 sun_dir, double relief_K) {
    const double dir_len2 = glm::dot(dir, dir);    // length2 would need glm/gtx/norm.hpp
    if(dir_len2 > 0.0) { dir *= 1.0 / std::sqrt(dir_len2); }
    const double sun_len2 = glm::dot(sun_dir, sun_dir);
    if(sun_len2 > 0.0) { sun_dir *= 1.0 / std::sqrt(sun_len2); }
    return groundTemperatureAt(anchor_K, even_share, cosIncidence(dir, sun_dir), relief_K);
}

/* What the WIND layer can do for a body (#769), decided once per bake on the main thread for
   the same reason TempLayer is: the worker gets numbers and a table, never body pointers.

   The ramp's ends are NOT here, and that is the one thing this struct is careful about. The
   authored rows say how fast the air blows at the altitudes the FILE names; the picture is
   coloured over the altitudes the GROUND reaches, and those are not the same span. Venus
   states 1 m/s at the datum and 100 m/s at 65 km, and its highest ground is 2.7 km, so a ramp
   spanning the rows paints its whole world in the bottom 3% of the palette -- one colour and a
   grid of identical arrows, which is the picture failing at exactly the place it is most
   interesting. So the rows travel here and windRampEnds() below prices the span once the sweep
   knows the ground's own floor and ceiling. Both spans go to --surfmap-log: wind_ends= is the
   file's, wind_ramp= is the picture's, and the gap between them is news rather than an
   assumption. */
struct WindLayer {
    bool ok = false;
    // The authored profile, copied for the worker. windAt() asserts non-empty, so this is
    // only ever read when ok is true.
    std::vector<WindLevel> rows;
    double auth_lo_ms = 0.0, auth_hi_ms = 0.0;   // slowest / fastest ROW (|east, north|)
    std::string legend;                 // the sentence for --surfmap-log's note=
    /* The same refusal in three words for the cursor row (`wind - (no air)`), because the two
       absences are different news: a world with no sky and a world whose file states a still
       sky (#726, and the same pair hasSurfaces/hasRcs keep apart in the capability model). */
    std::string block;
};

inline WindLayer windLayerFor(const Surface &s) {
    WindLayer wl;
    /* "Is there air?" is AtmosphereParams::physical(), NOT enabled: enabled means "draw a limb
       glow", and Io and Triton ship an atmosphere object with render fields and no profile at
       all. Asking the rim would let those two print "its air is authored still" -- the #726
       merge, a world with nothing to blow and a world whose air stands still wearing the same
       words. The dossier already asks the right question ("none (rim only)", gameui.cpp). */
    if(!s.atmosphere.physical()) {
        wl.legend = s.atmosphere.enabled
            ? "no air on this world: the file gives it a render rim and nothing to blow"
            : "no air on this world: there is nothing to blow";
        wl.block = "no air";
    } else if(!s.atmosphere.has_wind()) {
        // Reachable, and since #772 narrow: of the nine airs the shipped solar system files
        // write, only Pluto's states no wind (1.3 Pa of N2, and no fact page gives it one).
        // Kerbin-side airs and the playground files still take this path, so it has to stay
        // honest -- "no wind authored" is different news from the no-air branch above. The
        // roster of bodies that DO author rows is tests/test_scenarios.cpp:kWindsAuthored.
        wl.legend = "this world's file states no wind: its air is authored still";
        wl.block = "no wind";
    } else {
        wl.ok = true;
        wl.rows = s.atmosphere.wind;
        wl.auth_lo_ms = wl.auth_hi_ms = std::hypot(wl.rows.front().east,
                                                   wl.rows.front().north);
        for(const WindLevel &r : wl.rows) {
            const double sp = std::hypot(r.east, r.north);
            if(sp < wl.auth_lo_ms) { wl.auth_lo_ms = sp; }
            if(sp > wl.auth_hi_ms) { wl.auth_hi_ms = sp; }
        }
    }
    /* A refusal with an empty reason is the silent half of #726: the log would print note=""
       and the cursor "(unavailable)", a dash wearing the shape of an explanation. Authoring
       bug, so it crashes rather than paints. */
    assert(wl.ok || (!wl.legend.empty() && !wl.block.empty()));
    return wl;
}

/* The speed ramp's ends for one body's GROUND (#769): the slowest and fastest windAt() can
   answer anywhere between the lowest and highest ground the sweep will colour, in metres above
   the sea-level datum. Exact for the profile windAt() is -- between two rows the vector is
   linear in altitude, so |v| can only turn inside a segment where v . dv/de = 0, which is the
   closest approach to calm and is evaluated too (rows of (5,0) -> (0,5) dip to 3.5 m/s between
   their two ends, and a ramp spanning the rows would paint that dip off the cold end).
   Consequence worth stating: every pixel the sweep colours lies inside [lo_ms, hi_ms] by
   construction, so surfmapWindT01() below never extrapolates.

   Banded rows (#767) interleave in altitude -- a whole-sphere column and a jet band are two
   profiles, not one stretch of the same one -- so the walk goes per BAND: sort by band and
   then by alt, price each band's own profile over the ground's reach, take the union. That
   union is EXACT because windAt() answers a place with the narrowest band covering it and
   nothing else: each pixel's value is some one band's profile at that pixel's elevation, so
   pricing every band over the ground's whole reach covers every pixel and no others. A table
   with no bands, which is every table authored so far, is one band and exactly the old walk. */
inline void windRampEnds(const std::vector<WindLevel> &rows, double elev_lo, double elev_hi,
                         double &lo_ms, double &hi_ms) {
    assert(!rows.empty() && elev_lo <= elev_hi);
    lo_ms = 1e300; hi_ms = -1e300;
    std::vector<int> idx(rows.size());
    for(size_t i = 0; i < rows.size(); i++) { idx[i] = (int)i; }
    std::sort(idx.begin(), idx.end(), [&](int a, int b) {
        const WindLevel &A = rows[a], &B = rows[b];
        if(A.lat0 != B.lat0) { return A.lat0 < B.lat0; }
        if(A.lat1 != B.lat1) { return A.lat1 < B.lat1; }
        if(A.lon0 != B.lon0) { return A.lon0 < B.lon0; }
        if(A.lon1 != B.lon1) { return A.lon1 < B.lon1; }
        return A.alt < B.alt;
    });
    auto consider = [&](const glm::dvec2 &v) {
        const double sp = glm::length(v);
        if(sp < lo_ms) { lo_ms = sp; }
        if(sp > hi_ms) { hi_ms = sp; }
    };
    /* One band's own profile at one altitude: the same hold-outside / lerp-inside law
       windAt() runs, restricted to the run of rows answering for that band. */
    auto at_alt = [&](size_t s, size_t e, double alt) {
        if(alt <= rows[idx[s]].alt) {
            return glm::dvec2(rows[idx[s]].east, rows[idx[s]].north);
        }
        if(alt >= rows[idx[e - 1]].alt) {
            return glm::dvec2(rows[idx[e - 1]].east, rows[idx[e - 1]].north);
        }
        for(size_t k = s + 1; k < e; k++) {
            const double a0 = rows[idx[k - 1]].alt, a1 = rows[idx[k]].alt;
            if(alt > a0 && alt <= a1) {
                const double f = (a1 > a0) ? (alt - a0) / (a1 - a0) : 0.0;
                const glm::dvec2 v0(rows[idx[k - 1]].east, rows[idx[k - 1]].north);
                const glm::dvec2 v1(rows[idx[k]].east, rows[idx[k]].north);
                return v0 + f * (v1 - v0);
            }
        }
        assert(false);   // bracketed above, so unreachable
        return glm::dvec2(0.0);
    };
    for(size_t s = 0; s < idx.size(); ) {
        size_t e = s + 1;
        while(e < idx.size() && windSameBand(rows[idx[s]], rows[idx[e]])) { e++; }
        consider(at_alt(s, e, elev_lo));
        consider(at_alt(s, e, elev_hi));
        for(size_t k = s; k + 1 < e; k++) {
            const double a0 = rows[idx[k]].alt, a1 = rows[idx[k + 1]].alt;
            if(a1 <= elev_lo || a0 >= elev_hi) { continue; }   // this segment is out of reach
            if(a0 > elev_lo) { consider(glm::dvec2(rows[idx[k]].east, rows[idx[k]].north)); }
            const glm::dvec2 v0(rows[idx[k]].east, rows[idx[k]].north);
            const glm::dvec2 v1(rows[idx[k + 1]].east, rows[idx[k + 1]].north);
            const glm::dvec2 dv = v1 - v0;
            const double dd = glm::dot(dv, dv);
            if(dd > 0.0) {
                /* |v| is convex along a segment, so its low end is where v . dv/de = 0 --
                   but only if that altitude is one the ground can actually stand at. A dip
                   40 km up on a body whose highest ground is 3 km must not set the cold end
                   of the ramp, or the coldest colour paints nothing. */
                const double t = -glm::dot(v0, dv) / dd;
                if(t > 0.0 && t < 1.0) {
                    const double dip = a0 + t * (a1 - a0);
                    const double seg_lo = a0 > elev_lo ? a0 : elev_lo;
                    const double seg_hi = a1 < elev_hi ? a1 : elev_hi;
                    const double at = dip < seg_lo ? seg_lo : (dip > seg_hi ? seg_hi : dip);
                    consider(v0 + ((at - a0) / (a1 - a0)) * dv);
                }
            }
        }
        s = e;
    }
}

/* A wind (east, north) [m/s] as a pixel delta on the equirect map (#769). Row 0 is the north
   pole and column 0 is lon 0 (equirect.h), so east is +x and north is -y.

   The east component carries a 1/cos(lat): the projection spends the same pixels on a degree
   of longitude wherever it is, while the ground gets cos(lat) as much of it. A hull drifting
   10 m/s east at 60 deg N covers TWICE the map distance of one drifting 10 m/s north, so an
   arrow built from the raw components tilts that wind toward the pole. This is the ANGLE the
   divide fixes; an arrow's LENGTH comes from the speed through surfmapWindT01, not from this
   vector's magnitude, because the ramp is a scale over speeds and not over map distance. The
   divide is floored at kMapPoleCos because inside ~85 deg the map is degenerate anyway (every
   direction from there is south) and an unbounded divide would put an arrow's head off the
   picture at the exact place a reader is most likely to look. */
inline constexpr double kMapPoleCos = 0.08;
inline glm::dvec2 surfmapWindPixelVec(double east, double north, double cos_lat) {
    const double cl = cos_lat < kMapPoleCos ? kMapPoleCos : cos_lat;
    return glm::dvec2(east / cl, -north);
}

/* The [0,1] position of a ground speed in a body's own ramp. lo == hi (a table that states one
   speed, or a calm world) spans nothing, so the picture takes the midpoint: one honest flat
   colour rather than a gradient the eye invents. Same shape as the temperature ramp above. */
inline double surfmapWindT01(double speed_ms, double lo_ms, double hi_ms) {
    return (hi_ms > lo_ms) ? (speed_ms - lo_ms) / (hi_ms - lo_ms) : 0.5;
}

/* WHICH pixel rows get a row of wind arrows (#780). A uniform grid is right for a table that
   is one column: a whole-sphere row -- what Venus, Earth, Mars and Titan still author -- answers
   for the body and has no centre worth ringing. A table with real LATITUDE structure reads
   better as bands -- a generated belt layout (#775) or an authored jet band (#767), which is
   what #772 put on Jupiter and Saturn -- one row at each band's own centre latitude, so the
   alternation and the BELT COUNT come out as rings rather than as a grid someone has to scan
   and count by eye.

   Two bounds pull opposite ways, and the grid is the honest fallback for both:

     too FEW -- ringing needs at least kMinArrowRings rows, or a 3-band table draws 3 rows where
       the grid drew 16 and the map loses the coverage wind_cells= exists to guard. The bound is
       on rows DRAWN, not on bands found: bands whose centres round to the same pixel row are one
       ring, so 20 bands sitting in two clumps would draw 2 and must take the grid.

     too MANY -- rings must sit `need` px apart, where need is an arrow's VERTICAL footprint:
       the head's spread for zonal air (the barbs reach +-head/2 above and below the row), and at
       the other extreme the shaft's whole length for a meridional field. That is decided from
       the table's most meridional ROW, never from a bake's pixels, so a coarse bake cannot fool
       it into drawing overlapping rings. surfmapWindPixelVec's 1/cos(lat) east stretch only
       pushes a vector away from vertical, so the raw component ratio is the conservative one.

   A pole row is never drawn at all, by either set: at j == 0 or h - 1 every direction is south,
   so the whole row would show one degenerate arrow along the map's edge. The rings drop a band
   centre that rounds there, and the grid skips it too -- it usually cannot reach h - 1 (a
   multiple of stride from stride / 2 is even, and h - 1 is odd whenever stride = w / 32), but an
   odd bake size lets it: --surfmap-n 26 has h = 13 and rows 4 and 12.

   The loader guarantees every place is covered by SOME band; distinct (lat0, lat1)
   pairs can still overlap across altitudes -- a whole-sphere column and a jet band are two
   profiles, not a partition -- so `bands` counts distinct pairs, and that is the number the
   player's belt count is, not the number of rows the table has. */
inline constexpr size_t kMinArrowRings = 10;

struct ArrowRowPlan {
    std::vector<int> grid;      // the uniform rows, ascending
    std::vector<int> rings;     // one row per band centre, ascending; empty if ringing lost
    bool from_bands = false;    // which of the two the caller must draw
    int bands = 0;              // distinct (lat0, lat1) pairs in the table
};

inline ArrowRowPlan surfmapArrowRows(const std::vector<WindLevel> &rows, int h, int stride,
                                     double half, double head) {
    assert(!rows.empty() && stride >= 1 && h >= 2);
    assert(half > 0.0 && head > 0.0);
    ArrowRowPlan plan;
    for(int j = stride / 2; j < h; j += stride) {
        if(j == 0 || j == h - 1) { continue; }   // a pole, reached by an odd bake size
        plan.grid.push_back(j);
    }

    std::vector<std::pair<double, double>> bands;
    bands.reserve(rows.size());
    for(const WindLevel &r : rows) { bands.push_back({r.lat0, r.lat1}); }
    std::sort(bands.begin(), bands.end());
    bands.erase(std::unique(bands.begin(), bands.end()), bands.end());
    plan.bands = (int)bands.size();
    if(bands.size() < kMinArrowRings) { return plan; }

    double merid = 0.0;   // most meridional row in the table, as a fraction of itself
    for(const WindLevel &r : rows) {
        const double sp = std::hypot(r.east, r.north);
        if(sp > 0.0) { merid = std::max(merid, std::fabs(r.north) / sp); }
    }
    const int need = (int)std::ceil(2.0 * merid * half + head + 1.0);

    for(const auto &b : bands) {
        // equirectDirAt inverts to j = (90 - lat) / 180 * (h - 1); a band centre is in range.
        const int j = (int)std::llround((90.0 - 0.5 * (b.first + b.second))
                                        / 180.0 * (double)(h - 1));
        assert(j >= 0 && j <= h - 1);
        if(j == 0 || j == h - 1) { continue; }   // a pole: one degenerate arrow, all the way along
        plan.rings.push_back(j);
    }
    /* Into pixel order, which runs the OTHER way from latitude: the band list above sorts south
       to north while row 0 is the north pole. Comparing unsorted gaps compares negatives, which
       is how every body silently fell back to the grid the first time this was written. Two
       bands whose centres round to the same row are one ring, so unique BEFORE the spacing and
       the minimum are asked -- a duplicate would fail the spacing check for the wrong reason and
       a clump would pass the minimum for the wrong one. */
    std::sort(plan.rings.begin(), plan.rings.end());
    plan.rings.erase(std::unique(plan.rings.begin(), plan.rings.end()), plan.rings.end());
    if(plan.rings.size() < kMinArrowRings) { plan.rings.clear(); return plan; }
    for(size_t k = 1; k < plan.rings.size(); k++) {
        if(plan.rings[k] - plan.rings[k - 1] < need) { plan.rings.clear(); return plan; }
    }
    plan.from_bands = true;
    return plan;
}

// Request the surface map: snapshot inputs on the main thread, post the
// sweep to the background worker (g.jobs).
struct Game;
void surfmapCompute(Game &g);
