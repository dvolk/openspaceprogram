// test_surfmap.cpp -- unit tests for the equirectangular projection
// (src/equirect.h) plus the Surface Map's shade/wrap helpers
// (src/surfmap.h). Header-only pure math (glm + <cmath>). Pins the
// lon/lat convention (lon 0 = +X = rail lon 0, +90 deg = -Z east, north
// = +Y -- see equirect.h / #200), the pixel <-> direction round-trip, the
// cloud-deck mesh-phi UV bridge (cloudShader.vs), a heightmap column
// round-trip, the terminator's shade range, and the antimeridian wrap
// detection. Links no imgui / Bullet / GL.

#include "equirect.h"
#include "surfmap.h"
#include "terragen.h"

#include <cmath>
#include <cstdio>
#include <numbers>
#include <initializer_list>
#include <cstring>
#include <memory>

static int g_failures = 0;

static void check(bool cond, const char *what) {
    if(!cond) {
        std::printf("FAIL %s\n", what);
        ++g_failures;
    }
}

static bool near_d(double a, double b, double tol) {
    return std::fabs(a - b) <= tol;
}

static bool near_v(const glm::dvec3 &a, const glm::dvec3 &b, double tol) {
    return near_d(a.x, b.x, tol) && near_d(a.y, b.y, tol)
        && near_d(a.z, b.z, tol);
}

/* #745: the map-type/level chooser, the colour ramp, and which worlds the temperature layer
   can paint. These belong in this file because it is already the home of surfmap.h's pure
   parts, and they are the invariants the WINDOW assumes and cannot check itself: it builds its
   two combos by iterating the table, so a new map type that ships half-wired -- a level its
   type does not offer, two rows under one name, a CLI name matching nothing -- compiles clean
   and shows up as a widget that does nothing. */
static Surface makeDuna() {
    Surface duna;
    duna.atmosphere.enabled = true;
    AtmoLayer ground_level;
    ground_level.base_T = 214.81;
    duna.atmosphere.layers.push_back(ground_level);
    duna.even_share = 0.300;
    return duna;
}

static void mapLayers() {
    // ---- the table -------------------------------------------------------------
    check(kMapTypeCount >= 3,
          "the shipped system offers at least ground, ground+ocean and temperature");
    for(int t = 0; t < kMapTypeCount; t++) {
        const SurfMapTypeSpec &s = kSurfMapTypes[t];
        check(s.n_levels >= 1 && s.n_levels <= kLevelCount,
              "every map type offers at least one level");
        for(int k = 0; k < s.n_levels; k++) {
            check(s.levels[k] >= 0 && s.levels[k] < kLevelCount,
                  "a map type's level list holds real levels");
            for(int m = k + 1; m < s.n_levels; m++) {
                check(s.levels[k] != s.levels[m],
                      "a map type's level list has no duplicates");
            }
        }
        check(s.name != nullptr && s.name[0] != '\0', "every map type has a name");
        // The name is what --surfmap-map matches, so a duplicate makes that flag ambiguous
        // and the combo offer one choice twice.
        for(int u = t + 1; u < kMapTypeCount; u++) {
            check(std::string(s.name) != std::string(kSurfMapTypes[u].name),
                  "map type names are unique");
        }
        // The window applies levels[0] when the type changes, so today every type offers
        // exactly the ground: a new level appearing here is a decision, not a typo.
        check(s.n_levels == 1, "every map type has exactly one level today");
        check(s.levels[0] == kLevelGround, "and that level is the ground");
    }

    // ---- the name the CLI uses -------------------------------------------------
    check(surfMapTypeFromName("ground") == kMapGround, "\"ground\" is a map type");
    check(surfMapTypeFromName("ground+ocean") == kMapGroundOcean,
          "\"ground+ocean\" is a map type");
    check(surfMapTypeFromName("temperature") == kMapTemperature,
          "\"temperature\" is a map type");
    for(int t = 0; t < kMapTypeCount; t++) {
        check(surfMapTypeFromName(kSurfMapTypes[t].name) == t,
              "every combo label is also a CLI name (#636: one spelling)");
    }
    check(surfMapTypeFromName("TEMPERATURE") == kMapTemperature,
          "a map type name matches whatever its case");
    // A wrong WORD is refused rather than guessed: silently drawing 'ground' for
    // --surfmap-map=temp would give the caller a picture they did not ask for.
    check(surfMapTypeFromName("temp") == -1, "an unknown map type is refused, not guessed");
    check(surfMapTypeFromName("ocean") == -1, "a level word is not a map type");
    check(surfMapTypeFromName("") == -1, "the empty name is not a map type");
    check(std::string(surfLevelName(kLevelGround)) == "ground",
          "the one level is 'ground' -- surface and ground are one place (#636)");

    // ---- the colour ramp -------------------------------------------------------
    // Not the exact palette -- the properties the picture depends on. A temperature a hair
    // over the day end (float dust at the subsolar pixel) must not come back as the cold end.
    unsigned char r0, g0, b0, r1, g1, b1, ra, ga, ba, rb, gb, bb;
    surfmapRamp(0.0, r0, g0, b0);
    surfmapRamp(1.0, r1, g1, b1);
    check(b0 > r0, "the ramp's cold end is blue");
    check(r1 > b1, "the ramp's hot end is red");
    surfmapRamp(-3.0, ra, ga, ba);
    surfmapRamp(0.0, rb, gb, bb);
    check(ra == rb && ga == gb && ba == bb, "the ramp clamps below its cold end");
    surfmapRamp(9.0, ra, ga, ba);
    surfmapRamp(1.0, rb, gb, bb);
    check(ra == rb && ga == gb && ba == bb, "the ramp clamps above its hot end");
    /* What "hotness" means for THIS palette. Channel-by-channel monotonicity is a property the
       palette deliberately does not have (the yellow stop is redder than the red one, which is
       what makes the hot end read hot rather than merely dark), so pin the reading a player
       brings instead: warmth = red minus blue rises from end to end, and the ramp never stalls
       on one colour across its middle. */
    check((int)r1 - (int)b1 > (int)r0 - (int)b0,
          "the hot end reads warmer than the cold end (red minus blue)");
    bool distinct = true;
    for(int k = 0; k <= 10; k++) {
        for(int m = k + 1; m <= 10; m++) {
            unsigned char ka, kb, kc, la, lb, lc;
            surfmapRamp(k / 10.0, ka, kb, kc);
            surfmapRamp(m / 10.0, la, lb, lc);
            if(ka == la && kb == lb && kc == lc) { distinct = false; }
        }
    }
    check(distinct, "the ramp does not stall: ten samples across it are ten colours");
    /* ...and the ORDER of the middle matters, not just the ends: the ramp walks blue -> cyan ->
       green -> yellow -> red. Swap the two mid stops and the ends and the distinctness both
       still pass, while a mid-range field renders green-colder-than-blue. */
    unsigned char c15r, c15g, c15b, c55r, c55g, c55b, c85r, c85g, c85b;
    surfmapRamp(0.15, c15r, c15g, c15b);
    surfmapRamp(0.55, c55r, c55g, c55b);
    surfmapRamp(0.85, c85r, c85g, c85b);
    check(c15b > c15r && c15b > c15g, "a quarter up the ramp is still blue");
    check(c55g > c55r && c55g > c55b, "the middle of the ramp is green, not red or blue");
    check(c85r > c85g && c85r > c85b, "three quarters up is red");
    unsigned char rm, gm, bm;
    surfmapRamp(0.5, rm, gm, bm);
    check(!(rm == r0 && gm == g0 && bm == b0) && !(rm == r1 && gm == g1 && bm == b1),
          "the ramp's midpoint is neither end (a no-swing world reads as one flat colour)");

    /* ---- which worlds the temperature layer can paint --------------------------
       tempLayerFor's policy is what the window shows and the log prints, so it is pinned here
       rather than by paying for four more game runs: an e2e case can afford one refusal, but
       the DIFFERENT refusals are what #726's rule is about -- a reader deserves to know which
       one they are looking at. */
    Surface kerbin;
    kerbin.temperature = 288.15;
    kerbin.even_share = 0.9323;
    TempLayer tk = tempLayerFor(kerbin, 0.0, true);
    check(tk.ok, "a stated ground with an authored swing paints");
    check(groundAnchorRung(kerbin.temperature, 0.0, 0.0) == kAnchorStated,
          "and says its anchor was stated");
    check(near_d(tk.lo_K, groundTemperature(288.15, 0.9323, 0.0), 1e-12),
          "the ramp's cold end is the model's night end");
    check(near_d(tk.hi_K, groundTemperature(288.15, 0.9323, 1.0), 1e-12),
          "and its hot end the subsolar end");
    check(near_d(tk.lo_K, 283.15, 2e-4 * 283.15),
          "Kerbin's night end is the 283 K the pair states");
    check(near_d(tk.hi_K, 301.77, 2e-4 * 301.77),
          "and its day end the 302 K the law implies");

    // Venus: a swing of exactly one. The ends COLLAPSE, which is why the caller's ramp spans
    // by a guarded division and paints one honest colour rather than a gradient.
    Surface venus;
    venus.temperature = 737.0;
    venus.even_share = 1.0;
    TempLayer tv = tempLayerFor(venus, 0.0, true);
    check(tv.ok, "a stated swing of one still paints");
    check(near_d(tv.hi_K - tv.lo_K, 0.0, 1e-9), "a no-swing world's ramp span is zero");

    // Mercury/Moho: an anchor with nothing stated about night and day (#742's refusal).
    Surface moho;
    moho.temperature = 440.0;
    TempLayer tm = tempLayerFor(moho, 0.0, true);
    check(!tm.ok, "an unauthored swing does not paint");
    check(tm.legend.find("no authored swing") != std::string::npos
              && tm.legend.find("decision") != std::string::npos,
          "and the reason names the missing swing, not a missing anchor");
    check(tm.lo_K == 0.0, "a refused layer has no ramp ends");

    // Mun/Moon: nobody states a ground, so the anchor is DERIVED and the legend has to say so
    // beside the picture (#722).
    Surface mun;
    mun.bond_albedo = 0.11;
    mun.even_share = 0.0153;
    TempLayer tn = tempLayerFor(mun, 208.0, true);
    check(tn.ok, "a derived anchor still paints");
    check(tn.legend.find("DERIVED") != std::string::npos,
          "and its legend says DERIVED, not 'stated'");

    // Minmus: no reading of any kind. Not the same sentence as the two above.
    Surface minmus;
    TempLayer tg = tempLayerFor(minmus, 0.0, true);
    check(!tg.ok, "a world with no readings does not paint");
    check(tg.legend.find("nothing anchors") != std::string::npos,
          "and the reason says there is no anchor -- not that a swing is missing");

    // Jool: asked about a ground temperature, this world has no ground to have one about
    // (#738). Said FIRST, ahead of any missing number.
    Surface jool;
    jool.bands = true;
    jool.temperature = 165.0;
    jool.even_share = 0.5;
    jool.bond_albedo = 0.34;
    TempLayer tb = tempLayerFor(jool, 100.0, true);
    check(!tb.ok, "a banded giant never paints as a ground");
    check(tb.legend.find("banded giant") != std::string::npos,
          "and the reason says it has no ground, not that a number is missing");

    // Duna: the air's datum anchors it (rung two -- how most of the shipped system is
    // anchored), and outranks the derived black-body handed to it.
    Surface duna = makeDuna();
    TempLayer td = tempLayerFor(duna, 999.0, true);
    check(td.ok, "a world with air and a swing paints");
    check(near_d(td.anchor_K, 214.81, 1e-9),
          "the air's datum anchors it, and outranks the derived black-body");
    check(near_d(td.lo_K, 158.98, 2e-4 * 158.98),
          "Duna's night end follows from that anchor and its knob");
    check(near_d(td.hi_K, 285.03, 2e-4 * 285.03), "and so does its day end");

    /* Which ABSENCE, in the three words the cursor row prints. The sentence goes to the log and
       the word to the line, and they are the same decision -- so the words have to stay as
       distinct as the states they name (#726): a bare dash would fold "this world has no
       ground" and "nobody stated a night/day pair" back into one. */
    check(tempLayerFor(jool, 0.0, true).block == "no ground", "a giant's cursor row says no ground");
    check(tempLayerFor(moho, 0.0, true).block == "no swing", "a refused pair says no swing");
    check(tempLayerFor(minmus, 0.0, true).block == "no anchor", "an unpriced world says no anchor");
    check(td.block.empty() && tk.block.empty(), "a world that DOES paint has no block word");
    check(tempLayerFor(jool, 0.0, true).block != tempLayerFor(moho, 0.0, true).block,
          "the two refusals do not read alike on the cursor line");

    /* The fourth absence: no star. The whole field would be night, so the layer refuses rather
       than painting a picture in which everything is the cold end -- which is exactly what a
       naive "cos = 0 everywhere" fallback would do, and it would look like a frozen world. */
    const TempLayer no_sun = tempLayerFor(makeDuna(), 0.0, /*has_sun=*/false);
    check(!no_sun.ok, "a world with no star in range does not paint");
    check(no_sun.block == "no sun", "and says no sun, which is not no swing");
    check(no_sun.lo_K == 0.0 && no_sun.hi_K == 0.0, "a no-sun layer has no ramp ends");
    // The star outranks the other reasons: Duna has an anchor and a swing and STILL refuses.
    check(tempLayerFor(makeDuna(), 0.0, false).block == "no sun",
          "no sun is said ahead of a world that would otherwise paint");
}

/* #745: the number the cursor row prints. This is the SAME call the sweep makes per pixel, and
   it is handed the star the picture was BAKED with rather than a fresh sample -- a map is a
   snapshot and the frames move underneath it, so a cursor reading taken from today's sun would
   be a different world's number than the colour under the mouse (#687: measured or recomputed,
   never both at different instants). */
static void cursorTemperature() {
    const double anchor = 214.81, knob = 0.300;      // Duna
    const TempLayer duna = tempLayerFor(makeDuna(), 0.0, true);
    const glm::dvec3 sun(1.0, 0.0, 0.0);

    // Under the star: the day end the atlas prints.
    check(near_d(surfmapGroundT(anchor, knob, sun, sun), duna.hi_K, 1e-9),
          "the point under the star reads the map's day end");
    // Antipodal to it, and anywhere on the terminator: the night end. The law has no night/day
    // distinction past cos = 0, and a cursor on the seam must not invent one.
    check(near_d(surfmapGroundT(anchor, knob, -sun, sun), duna.lo_K, 1e-9),
          "the antipode reads the night end");
    check(near_d(surfmapGroundT(anchor, knob, glm::dvec3(0.0, 0.0, 1.0), sun), duna.lo_K, 1e-9),
          "the terminator reads the night end too");
    // A direction NOT on the unit sphere (a cursor un-normalised somewhere) is clamped, not
    // extrapolated: the readout cannot exceed the map's own day end.
    check(surfmapGroundT(anchor, knob, 3.0 * sun, sun) <= duna.hi_K + 1e-9,
          "an un-normalised direction cannot read hotter than the picture's day end");
    // Three places a third of a turn apart on the equator: at least one is in night and at
    // least one is lit, whatever the star's bearing -- which is why an e2e case could ask for
    // this spread without knowing where local noon is. Pinned here instead of there, because
    // the pixels a hover needs are window-layout coordinates.
    double t[3];
    for(int k = 0; k < 3; k++) {
        const double a = k * 2.0 * std::numbers::pi / 3.0;
        t[k] = surfmapGroundT(anchor, knob, glm::dvec3(std::cos(a), 0.0, std::sin(a)), sun);
    }
    const double lo = std::min(std::min(t[0], t[1]), t[2]);
    const double hi = std::max(std::max(t[0], t[1]), t[2]);
    check(near_d(lo, duna.lo_K, 1e-9), "of three places 120 deg apart, the coldest is in night");
    check(hi > lo + 1.0, "and the warmest is lit by more than rounding");
}

/* #769: the wind map. Same division of labour as mapLayers() -- the window builds its combo
   from the table and paints the field per pixel, so what belongs HERE is the policy the window
   cannot check (which worlds can be drawn, and which absence a refusal names) and the geometry
   the sweep cannot see (what a wind looks like as a direction on an equirectangular map). */
static void windLayers() {
    // ---- the name the CLI and the combo share ---------------------------------
    check(surfMapTypeFromName("wind") == kMapWind, "\"wind\" is a map type");
    check(surfMapTypeFromName("Wind") == kMapWind, "and matches whatever its case");
    check(surfMapTypeFromName("winds") == -1, "a near-miss word is refused, not guessed");

    // ---- the states a body can be in ------------------------------------------
    /* "Is there air?" is AtmosphereParams::physical() (a profile), NOT enabled (a render rim).
       Io and Triton ship an atmosphere object with rim fields and no profile at all, so the
       rim-only fixture below is a shipped shape and not a hypothetical: asking the rim would
       let those two say "its air is authored still", which is #726 merging the two absences
       back into one. */
    // 1. No air at all (the Moon).
    Surface moon;
    WindLayer wm = windLayerFor(moon);
    check(!wm.ok, "a body with no air paints no wind");
    check(wm.rows.empty(), "and publishes no rows to read");
    check(wm.block == "no air", "naming which absence: no air");
    check(!wm.legend.empty(), "with a sentence for the log");

    // 1b. A render rim and no profile (Io, Triton): the same NEWS, a different sentence.
    Surface io;
    io.atmosphere.enabled = true;
    WindLayer wio = windLayerFor(io);
    check(!wio.ok, "a rim is not air");
    check(wio.block == "no air", "and the cursor still says no air, not no wind");
    check(wio.legend != wm.legend, "while the log can tell a rim world from a bare one");

    // 2. Air the file states no wind for (every body but Venus, Earth, Mars).
    Surface kerbin;
    kerbin.atmosphere.enabled = true;
    kerbin.atmosphere.sea_level_density = 1.225;
    kerbin.atmosphere.scale_height = 8000.0;
    WindLayer wk = windLayerFor(kerbin);
    check(!wk.ok, "a body with air and no wind rows paints no wind");
    check(wk.block == "no wind", "and says so differently from a body with no air");
    check(wk.block != wm.block, "the two absences do not collapse into one (#726)");

    // 3. Authored wind (Earth's two rows).
    Surface earth;
    earth.atmosphere.enabled = true;
    earth.atmosphere.sea_level_density = 1.225;
    earth.atmosphere.scale_height = 8000.0;
    earth.atmosphere.wind = { {0.0, 5.0, 0.0}, {11000.0, 40.0, 0.0} };
    WindLayer we = windLayerFor(earth);
    check(we.ok, "a body that authors wind paints it");
    check(we.rows.size() == 2, "carrying the profile for the sweep and the cursor alike");
    check(near_d(we.auth_lo_ms, 5.0, 1e-12) && near_d(we.auth_hi_ms, 40.0, 1e-12),
          "the FILE's ends travel with it, for --surfmap-log's wind_ends=");

    /* Ends from the MAGNITUDE, not from one component: a row blowing hard north-west is the
       fastest row on this world, and a ramp that read only `east` would paint it cold. */
    Surface skew;
    skew.atmosphere.enabled = true;
    skew.atmosphere.sea_level_density = 0.02;
    skew.atmosphere.scale_height = 11000.0;
    skew.atmosphere.wind = { {0.0, 3.0, 0.0}, {20000.0, -30.0, -40.0} };
    WindLayer ws = windLayerFor(skew);
    check(near_d(ws.auth_lo_ms, 3.0, 1e-12), "the slow row still sets the cold end");
    check(near_d(ws.auth_hi_ms, 50.0, 1e-12), "and the fast one its hot end, whichever way it blows");

    // A one-row body (Mars): the span collapses, and the ramp must not invent a gradient.
    Surface mars;
    mars.atmosphere.enabled = true;
    mars.atmosphere.sea_level_density = 0.02;
    mars.atmosphere.scale_height = 11000.0;
    mars.atmosphere.wind = { {0.0, 10.0, 0.0} };
    WindLayer wa = windLayerFor(mars);
    check(wa.ok && near_d(wa.auth_lo_ms, wa.auth_hi_ms, 1e-12), "one row spans nothing");
    check(near_d(surfmapWindT01(10.0, wa.auth_lo_ms, wa.auth_hi_ms), 0.5, 1e-12),
          "so every pixel takes the midpoint: one honest flat colour");
    check(near_d(surfmapWindT01(40.0, we.auth_lo_ms, we.auth_hi_ms), 1.0, 1e-12)
          && near_d(surfmapWindT01(5.0, we.auth_lo_ms, we.auth_hi_ms), 0.0, 1e-12),
          "the ends of an Earth profile map to the ends of the ramp");
    check(surfmapWindT01(20.0, we.auth_lo_ms, we.auth_hi_ms)
          > surfmapWindT01(10.0, we.auth_lo_ms, we.auth_hi_ms),
          "and the ramp does not stall between them");

    /* ---- the ramp spans the GROUND, not the file (#769) ------------------------
       This is the difference between a picture and a swatch. Venus states 1 m/s at the datum
       and 100 m/s at 65 km; the highest Venus ground is ~2.7 km. Colouring against the rows
       puts every pixel of that world inside the bottom 3% of the palette -- one colour, and a
       grid of arrows all the same length -- so the sweep prices its own span from the floor and
       ceiling it is about to colour instead. */
    {
        const std::vector<WindLevel> venus = { {0.0, 1.0, 0.0}, {65000.0, 100.0, 0.0} };
        double lo, hi;
        windRampEnds(venus, -100.0, 2700.0, lo, hi);
        check(near_d(lo, 1.0, 1e-12), "the datum row still sets the floor");
        check(near_d(hi, 1.0 + (2700.0 / 65000.0) * 99.0, 1e-9),
              "and the highest GROUND its hot end, not the 65 km row the ground never reaches");
        check(hi < 6.0, "which is to say: this world's picture spans 5% of its own file");
    }
    /* A ROW the ground never stands at must not set the ramp. Earth's deck row lives at 11 km,
       so on a body whose ground stays below 50 m the picture is one speed, and the ramp spans
       5..5.2 -- the slice of the segment the ground actually enters -- not 5..40. */
    {
        const std::vector<WindLevel> earth = { {0.0, 5.0, 0.0}, {11000.0, 40.0, 0.0} };
        double lo, hi;
        windRampEnds(earth, -80.0, 50.0, lo, hi);
        check(near_d(lo, 5.0, 1e-12) && near_d(hi, 5.0 + (50.0 / 11000.0) * 35.0, 1e-9),
              "a row the ground never stands in does not stretch the ramp to 40 m/s");
        check(hi - lo < 0.2, "so a flat-shore world paints one speed and a ramp that agrees");
        windRampEnds(earth, -8000.0, 8848.0, lo, hi);
        check(near_d(lo, 5.0, 1e-12), "a sea floor below the datum row holds, it does not extrapolate");
        check(near_d(hi, 5.0 + (8848.0 / 11000.0) * 35.0, 1e-9),
              "and Everest pokes into the faster air the ramp then shows");
    }
    /* |v| can dip BELOW both rows between them -- (5,0) to (0,5) passes through 3.54 m/s at
       the midpoint -- and a ramp taken from the rows would paint that calm off its cold end,
       which is a reading the file never gave (#727). */
    {
        const std::vector<WindLevel> veer = { {0.0, 5.0, 0.0}, {1000.0, 0.0, 5.0} };
        double lo, hi;
        windRampEnds(veer, 0.0, 1000.0, lo, hi);
        check(near_d(lo, std::sqrt(12.5), 1e-9), "the closest approach to calm sets the floor");
        check(near_d(hi, 5.0, 1e-12), "and the rows themselves its ceiling");
    }

    // ---- what a wind LOOKS like on the map ------------------------------------
    /* The corner #769 is really about. An equirectangular map spends the same pixels on a
       degree of longitude wherever it is, while the ground gets cos(lat) as much of it, so the
       east component needs 1/cos(lat) or every wind tilts toward the pole. */
    {
        const glm::dvec2 e = surfmapWindPixelVec(10.0, 0.0, 1.0);      // equator
        check(e.x > 0.0 && near_d(e.y, 0.0, 1e-12), "east points along +column");
        const glm::dvec2 n = surfmapWindPixelVec(0.0, 10.0, 1.0);
        check(near_d(n.x, 0.0, 1e-12) && n.y < 0.0, "north points up the map (row 0 is north)");
        const glm::dvec2 s = surfmapWindPixelVec(0.0, -10.0, 1.0);
        check(s.y > 0.0, "and south down it");
    }
    {
        // 60 deg N: cos = 0.5, so an eastward wind spans twice the map distance.
        const double cl = 0.5;
        const glm::dvec2 e60 = surfmapWindPixelVec(10.0, 0.0, cl);
        check(near_d(e60.x, 20.0, 1e-9), "at 60 deg an east wind is stretched by 1/cos(lat)") ;
        check(near_d(e60.y, 0.0, 1e-12), "without tipping off the parallel");
        /* A wind that is 45 deg on the GROUND reads shallower than 45 deg on the map at 60 deg
           N -- that is the whole point, and the naive (east, north) drawing fails it. */
        const glm::dvec2 ne = surfmapWindPixelVec(10.0, 10.0, cl);
        check(std::fabs(ne.x) > std::fabs(ne.y),
              "a ground-NE wind is more eastward than northward on the map");
        check(near_d(std::atan2(ne.y, ne.x), std::atan2(-10.0, 20.0), 1e-12),
              "by exactly the projection's factor");
    }
    {
        /* Near a pole the divide would run away (cos -> 0) and put an arrow's head off the
           picture. The floor keeps the direction sane where the map is degenerate anyway. */
        const glm::dvec2 p1 = surfmapWindPixelVec(10.0, 0.0, 0.0);
        const glm::dvec2 p2 = surfmapWindPixelVec(10.0, 0.0, kMapPoleCos);
        check(near_d(p1.x, p2.x, 1e-12), "a pole does not divide by zero");
        check(std::isfinite(p1.x) && p1.x > 0.0, "and stays a finite eastward arrow");
        check(kMapPoleCos > 0.0 && kMapPoleCos < 0.2,
              "the floor is small (a real latitude) without being negligible");
    }
}

int main() {
    const int w = 16, h = 8;

    // 1. The pixel direction convention (equirect.h, the same pair
    //    render.cpp reads off the ship's position).
    //    lon 0 is +X (the left edge), +90 deg is -Z (east), north is +Y.
    {
        // A non-pole row: the direction is
        //   (cos lat . sin lon, sin lat, cos lat . cos lon)
        // with lat = pi/2 - pi * j / (h-1). (An even h has no exact
        // equator row -- j = (h-1)/2 is half-integer -- so the expected
        // lat is computed, not assumed.)
        const int j = h / 2;
        const double lat = M_PI * 0.5 - M_PI * (double)j / (double)(h - 1);
        const double cl = std::cos(lat), sl = std::sin(lat);
        check(near_v(equirectDirAt(0, j, w, h), glm::dvec3(cl, sl, 0), 1e-9),
              "dir: lon 0 -> +X hemisphere");
        check(near_v(equirectDirAt(w / 4, j, w, h), glm::dvec3(0, sl, -cl), 1e-9),
              "dir: lon 90 -> -Z hemisphere");
        check(near_v(equirectDirAt(w / 2, j, w, h), glm::dvec3(-cl, sl, 0), 1e-9),
              "dir: lon 180 -> -X hemisphere");
        check(near_v(equirectDirAt(3 * w / 4, j, w, h), glm::dvec3(0, sl, cl), 1e-9),
              "dir: lon 270 -> +Z hemisphere");
    }
    {
        // Poles: every column of row 0 is the north pole (lat +pi/2),
        // row h-1 the south.
        check(near_v(equirectDirAt(0, 0, w, h), glm::dvec3(0, 1, 0), 1e-9),
              "dir: row 0 -> north (+Y)");
        check(near_v(equirectDirAt(w / 3, 0, w, h), glm::dvec3(0, 1, 0), 1e-9),
              "dir: row 0, any column -> north");
        check(near_v(equirectDirAt(0, h - 1, w, h), glm::dvec3(0, -1, 0), 1e-9),
              "dir: row h-1 -> south (-Y)");
    }
    {
        // Latitude at a mid-column (lon 0, so the direction is (x, y, 0)):
        // row 1 of 8 -> lat = pi/2 - pi/7.
        const glm::dvec3 d = equirectDirAt(0, 1, w, h);
        const double lat = M_PI * 0.5 - M_PI * (double)1 / (double)(h - 1);
        check(near_v(d, glm::dvec3(std::cos(lat), std::sin(lat), 0), 1e-9),
              "dir: latitude at row 1");
    }

    // 2. equirectLonLat: the inverse (render.cpp's atan2(x, z), asin(y),
    //    with lon unwrapped to [0, 2pi)).
    {
        double lon, lat;
        equirectLonLat(glm::dvec3(1, 0, 0), lon, lat);
        check(near_d(lon, 0.0, 1e-12) && near_d(lat, 0.0, 1e-12),
              "lonlat: +X -> (0, 0)");
        equirectLonLat(glm::dvec3(0, 0, -1), lon, lat);
        check(near_d(lon, M_PI / 2, 1e-12) && near_d(lat, 0.0, 1e-12),
              "lonlat: -Z -> (pi/2, 0)");
        equirectLonLat(glm::dvec3(-1, 0, 0), lon, lat);
        check(near_d(lon, M_PI, 1e-12) && near_d(lat, 0.0, 1e-12),
              "lonlat: -X -> (pi, 0)");
        equirectLonLat(glm::dvec3(0, 0, 1), lon, lat);
        check(near_d(lon, 3.0 * M_PI / 2, 1e-12) && near_d(lat, 0.0, 1e-12),
              "lonlat: +Z -> (3pi/2, 0) (unwrapped, not -pi/2)");
        equirectLonLat(glm::dvec3(0, 1, 0), lon, lat);
        check(near_d(lat, M_PI / 2, 1e-12), "lonlat: north -> +pi/2");
        equirectLonLat(glm::dvec3(0, -1, 0), lon, lat);
        check(near_d(lat, -M_PI / 2, 1e-12), "lonlat: south -> -pi/2");
        equirectLonLat(glm::dvec3(0, 0.5, 0.8660254037844387), lon, lat);
        check(near_d(lat, M_PI / 6, 1e-9), "lonlat: latitude 30 deg N");
    }

    // 3. Round-trip: direction -> (lon, lat) -> pixel -> direction
    //    recovers the original (within one pixel cell).
    {
        const double cell_ang = 2.0 * M_PI / (double)w;  // one cell in lon
        bool ok = true;
        for(const glm::dvec3 d : {
                glm::dvec3(0.577, 0.577, 0.577),     // an octant direction
                glm::dvec3(0, -0.7071, 0.7071),      // south, lon 180
                glm::dvec3(-0.7071, 0.3827, 0.5946), // a generic direction
                glm::dvec3(1, 0, 0),                 // lon 0, equator
        }) {
            double lon, lat;
            equirectLonLat(d, lon, lat);
            double u, v;
            equirectPixel(lon, lat, w, h, u, v);
            const int i = (int)(u + 0.5);
            const int j = (int)(v + 0.5);
            const glm::dvec3 back = equirectDirAt(i, j, w, h);
            // Angle between the two (acos of the dot; both are unit).
            const double ang = std::acos(glm::clamp(
                glm::dot(glm::dvec3(d), back), -1.0, 1.0));
            if(ang > 2.0 * cell_ang) { ok = false; }
        }
        check(ok, "roundtrip: dir -> pixel -> dir (within a cell)");
    }

    // 4. equirectPixel: the (lon, lat) -> (u, v) mapping.
    {
        double u, v;
        equirectPixel(0.0, M_PI / 2, w, h, u, v);
        check(near_d(u, 0.0, 1e-12) && near_d(v, 0.0, 1e-12),
              "pixel: (lon 0, north) -> (0, 0) top-left");
        equirectPixel(M_PI, 0.0, w, h, u, v);
        check(near_d(u, w / 2.0, 1e-9) && near_d(v, (h - 1) / 2.0, 1e-9),
              "pixel: (lon 180, equator) -> center");
        equirectPixel(0.0, -M_PI / 2, w, h, u, v);
        check(near_d(u, 0.0, 1e-12) && near_d(v, (double)(h - 1), 1e-9),
              "pixel: (lon 0, south) -> bottom-left");
    }

    // 4b. Grid round-trip: for every non-pole pixel (i, j), the pixel
    //     direction -> (lon, lat) -> pixel is EXACTLY (i, j). (The pole
    //     rows are excluded: every column of a pole row is the same
    //     direction, so lon reads 0 there by convention -- test 1 pins it.)
    {
        bool ok = true;
        for(int j = 1; j < h - 1 && ok; j++) {
            for(int i = 0; i < w; i++) {
                const glm::dvec3 d = equirectDirAt(i, j, w, h);
                double lon, lat;
                equirectLonLat(d, lon, lat);
                double u, v;
                equirectPixel(lon, lat, w, h, u, v);
                if(!near_d(u, (double)i, 1e-9)
                   || !near_d(v, (double)j, 1e-9)) {
                    ok = false;
                    break;
                }
            }
        }
        check(ok, "grid: pixel -> dir -> lonlat -> pixel is exact");
    }

    // 5. surfmapShade: the terminator's light factor.
    {
        const glm::dvec3 n(1, 0, 0);
        check(near_d(surfmapShade(n, glm::dvec3(1, 0, 0)), 1.0, 1e-6),
              "shade: facing the sun -> 1.0");
        check(near_d(surfmapShade(n, glm::dvec3(-1, 0, 0)), 0.30, 1e-6),
              "shade: facing away -> 0.30 (ambient floor)");
        // n . sun = 0: t = clamp(0.05 / 0.20) = 0.25 -> 0.30 + 0.70 * 0.25.
        check(near_d(surfmapShade(n, glm::dvec3(0, 1, 0)),
                     0.30 + 0.70 * 0.25, 1e-6),
              "shade: terminator band at n . sun = 0");
        // Range: always within [0.30, 1.00].
        bool in_range = true;
        for(int k = 0; k < 24 && in_range; k++) {
            const double a = 2.0 * M_PI * k / 24;
            const float s = surfmapShade(glm::dvec3(1, 0, 0),
                                         glm::dvec3(std::cos(a), std::sin(a), 0));
            if(!(s >= 0.30f - 1e-6 && s <= 1.0f + 1e-6)) { in_range = false; }
        }
        check(in_range, "shade: range [0.30, 1.00]");
    }

    // 6. surfmapWraps: consecutive lons cross the antimeridian when their
    //    difference exceeds half a turn.
    {
        check(!surfmapWraps(0.1, 0.2), "wrap: small step -> false");
        check(!surfmapWraps(1.0, 1.1), "wrap: small step mid-map -> false");
        check(surfmapWraps(0.1, 0.9 * 2.0 * M_PI), "wrap: 0.1 -> 324 deg -> true");
        check(surfmapWraps(0.9 * 2.0 * M_PI, 0.1), "wrap: 324 -> 0.1 deg -> true");
        check(!surfmapWraps(0.0, M_PI), "wrap: exactly half a turn -> false");
        check(surfmapWraps(0.0, M_PI + 0.1), "wrap: half a turn + -> true");
    }

    // 7. Cloud-deck UV bridge (cloudShader.vs vs create_atmosphere_mesh).
    //    Mesh param phi = atan2(z, x) from +X toward +Z; surface lon is
    //    pi/2 - phi; bake u = lon/2pi (lon 0 at the LEFT edge); the shader
    //    therefore samples u = 0.25 - phi/2pi. Pin the full chain: the
    //    direction the mesh builds at phi MUST be the direction the bake
    //    writes into the column the shader's u names.
    {
        const int W = 64, H = 32;
        bool ok = true;
        for(int k = 0; k < 16 && ok; k++) {
            const float phi = 2.0f * (float)M_PI * (float)k / 16.0f;
            const float theta = 1.1f;   // mesh COLATITUDE (0 = north)
            // create_atmosphere_mesh's vertex direction at (theta, phi).
            const glm::vec3 mesh_dir(std::sin(theta) * std::cos(phi),
                                     std::cos(theta),
                                     std::sin(theta) * std::sin(phi));
            // cloudShader.vs: u = -phi/2pi
            const double u = -(double)phi / (2.0 * M_PI);
            // Bake column at that u (REPEAT wrap) -> pixel-centre lon.
            // Mesh theta is colatitude; equirectDir wants latitude.
            const double u01 = u - std::floor(u);
            const int pxi = (int)(u01 * W) % W;
            const double lon_bake = ((pxi + 0.5) / (double)W) * 2.0 * M_PI;
            const double lat_bake = 0.5 * M_PI - (double)theta;
            const glm::vec3 bake_dir = equirectDir((float)lon_bake,
                                                   (float)lat_bake);
            // One bake column is 2pi/W in lon; allow a full cell of slack
            // (the bake samples pixel centres, the mesh does not).
            if(glm::length(mesh_dir - bake_dir) > 2.0f * (float)M_PI / (float)W) {
                ok = false;
                break;
            }
            // And the closed form: lon = -phi (mod 2pi).
            double lon, lat;
            equirectLonLat(glm::dvec3(mesh_dir), lon, lat);
            double want = std::fmod(-(double)phi, 2.0 * M_PI);
            if(want < 0.0) { want += 2.0 * M_PI; }
            if(std::fabs(lon - want) > 1e-5) { ok = false; }
        }
        check(ok, "cloud UV: mesh phi -> shader u -> bake column -> dir");
        // The shader constant is NOT the formula above -- a leftover 0.25
        // or 0.75 in cloudShader.vs would not fail the chain. Pin the file.
        {
            FILE *f = std::fopen("res/shaders/cloudShader.vs", "r");
            check(f != nullptr, "cloud UV: cloudShader.vs readable");
            if(f) {
                char buf[4096];
                const size_t n = std::fread(buf, 1, sizeof(buf) - 1, f);
                buf[n] = '\0';
                std::fclose(f);
                check(std::strstr(buf, "uv0 = vec2(-uvParam.x, uvParam.y);")
                      != nullptr,
                      "cloud UV: cloudShader.vs samples u = -phi/2pi");
            }
        }
    }

    // 8. Heightmap column round-trip (the layout Heightmap::sample uses).
    //    A synthetic 4x3 grid: sample(equirectDir(lon, lat)) at the grid
    //    points must read that cell exactly (bilinear is exact there).
    {
        auto hm = std::make_shared<Heightmap>();
        hm->w = 4;
        hm->h = 3;
        hm->m = { 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120 };
        hm->recomputeRange();
        bool ok = true;
        // Skip pole rows: no unique lon there (test 1 pins them), and the
        // round-trip would only pass via float dust in cos(pi/2).
        for(int j = 1; j < hm->h - 1 && ok; j++) {
            for(int i = 0; i < 4 && ok; i++) {
                const glm::dvec3 d = equirectDirAt(i, j, 4, 3);
                const float s = hm->sample(glm::vec3(d));
                const float want = (float)hm->m[(size_t)j * 4 + i];
                if(std::fabs(s - want) > 0.5f) { ok = false; }
            }
        }
        check(ok, "heightmap: sample(equirectDir(i, j)) reads cell (i, j)");
    }

    mapLayers();
    cursorTemperature();
    windLayers();

    if(g_failures == 0) {
        std::printf("test_surfmap: all checks passed\n");
        return 0;
    }
    std::printf("test_surfmap: %d check(s) failed\n", g_failures);
    return 1;
}
