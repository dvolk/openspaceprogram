#pragma once
// surfmap.h -- the "Surface Map" window's 2-D projection + shading.
// EQUIRECTANGULAR over the body's ROTATING frame; the direction <-> (lon, lat)
// <-> pixel math lives in equirect.h.

#include <cassert>
#include <cmath>
#include <cctype>
#include <numbers>
#include <string>

#include <glm/glm.hpp>

#include "groundtemp.h"   // the temperature layer's law, and the anchor's rungs
#include "terragen.h"     // Surface: the data the layer decides from

#include "equirect.h"

// Terminator shading: light factor in [0.3, 1] (night keeps 30% ambient).
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
};

/* A type with ONE level renders that level as a label rather than as a menu offering a
   single choice: a dropdown that cannot be taken up is a widget lying about a choice. */
inline constexpr SurfMapTypeSpec kSurfMapTypes[kMapTypeCount] = {
    { "ground",         { kLevelGround },          1 },
    { "ground+ocean",   { kLevelGround },          1 },
    { "temperature",    { kLevelGround },          1 },
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
   aspect ratio -- they are --surfmap-log's temp_ends= and the atlas' `ground swing` row, and
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
    double lo_K = 0.0, hi_K = 0.0;      // the model's own night / day ends
    /* The sentence for --surfmap-log: the anchor's PROVENANCE when the layer paints (printed
       as anchor_src=, "the air's datum", "DERIVED from this sun and albedo" -- #722's rule that
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
    } else if(s.bands) {
        tl.legend = "a banded giant: the file gives this world air at levels, no ground";
        tl.block = "no ground";
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
        tl.legend = groundAnchorRungName(rung);
    }
    if(tl.ok) {
        tl.lo_K = groundTemperature(tl.anchor_K, tl.knob, 0.0);
        tl.hi_K = groundTemperature(tl.anchor_K, tl.knob, 1.0);
    }
    return tl;
}

/* [K] The temperature the map paints in ONE direction: the same arithmetic the sweep runs per
   pixel and the window's cursor readout runs for the point under the mouse. Shared because the
   number beside a pixel has to BE that pixel's number -- and because the sun it uses is the
   one the picture was BAKED with, not the sim's sun now (the map is a snapshot; frames move
   underneath it). #687's rule that a figure is measured or recomputed, never both at different
   instants. */
inline double surfmapGroundT(double anchor_K, double even_share, const glm::dvec3 &dir,
                             const glm::dvec3 &sun_dir) {
    const double c = std::max(0.0, std::min(1.0, glm::dot(dir, sun_dir)));
    return groundTemperature(anchor_K, even_share, c);
}

// Request the surface map: snapshot inputs on the main thread, post the
// sweep to the background worker (g.jobs).
struct Game;
void surfmapCompute(Game &g);
