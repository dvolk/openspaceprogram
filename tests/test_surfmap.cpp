// test_surfmap.cpp -- unit tests for the equirectangular projection
// (src/equirect.h) plus the Surface Map's shade/wrap helpers
// (src/surfmap.h). Header-only pure math (glm + <cmath>). Pins the
// lon/lat convention (lon 0 = +X = rail lon 0, +90 deg = -Z east, north
// = +Y -- see equirect.h / #200), the pixel <-> direction round-trip, the
// cloud-deck mesh-phi UV bridge (cloudShader.vs), a heightmap column
// round-trip, the terminator's shade range, and the antimeridian wrap
// detection. Links no imgui / Bullet / GL.

#include "equirect.h"
#include "insolation.h"   // subsolarLonLat: the same noon the sweep and the cursor price with
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

    /* Jool: the banded refusal is gone (surface-temp plan 2026_11_12 §2.2). The file authors a
       banded world's surface AT the 1 bar, so a temperature there is a temperature and the map
       paints it like any other. What the readout must still never do is let that read as a sea
       floor nobody can find. */
    Surface jool;
    jool.bands = true;
    jool.temperature = 165.0;
    jool.even_share = 0.5;
    jool.bond_albedo = 0.34;
    TempLayer tb = tempLayerFor(jool, 100.0, true);
    check(tb.ok, "a banded world paints, at the level the file calls the surface");
    check(tb.legend.find("1 bar level") != std::string::npos
              && tb.legend.find("not a sea floor") != std::string::npos,
          "and the legend says WHICH level, rather than refusing over a missing number");
    check(near_d(tb.anchor_K, 165.0, 1e-12), "the stated deck temperature anchors it");

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
       distinct as the states they name (#726): a bare dash would fold "nobody stated a night/day
       pair" and "nothing prices this world at all" back into one. */
    check(tempLayerFor(moho, 0.0, true).block == "no swing", "a refused pair says no swing");
    check(tempLayerFor(minmus, 0.0, true).block == "no anchor", "an unpriced world says no anchor");
    check(td.block.empty() && tk.block.empty() && tb.block.empty(),
          "a world that DOES paint has no block word -- the banded one included");
    check(tempLayerFor(moho, 0.0, true).block != tempLayerFor(minmus, 0.0, true).block,
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
    check(near_d(surfmapGroundT(anchor, knob, sun, sun, 0.0), duna.hi_K, 1e-9),
          "the point under the star reads the map's day end");
    // Antipodal to it, and anywhere on the terminator: the night end. The law has no night/day
    // distinction past cos = 0, and a cursor on the seam must not invent one.
    check(near_d(surfmapGroundT(anchor, knob, -sun, sun, 0.0), duna.lo_K, 1e-9),
          "the antipode reads the night end");
    check(near_d(surfmapGroundT(anchor, knob, glm::dvec3(0.0, 0.0, 1.0), sun, 0.0), duna.lo_K, 1e-9),
          "the terminator reads the night end too");
    // A direction NOT on the unit sphere cannot read hotter than the map's own day end. This is
    // the ceiling only: it passed under the OLD spelling too, which clamped the DOT PRODUCT and
    // so priced a 3x-long direction at three times the cosine of its angle -- a point 70 deg off
    // the star came back as the subsolar one, 80 deg came back as 59. The pair below is what the
    // clamp could not see: off the star, a longer or shorter vector must read the SAME place,
    // because a direction is a bearing and its length is not part of where it points.
    check(surfmapGroundT(anchor, knob, 3.0 * sun, sun, 0.0) <= duna.hi_K + 1e-9,
          "an un-normalised direction cannot read hotter than the picture's day end");
    const glm::dvec3 off = glm::normalize(glm::dvec3(std::cos(1.4), 0.0, std::sin(1.4)));
    const double off_T = surfmapGroundT(anchor, knob, off, sun, 0.0);
    check(near_d(surfmapGroundT(anchor, knob, 3.0 * off, sun, 0.0), off_T, 1e-9),
          "a longer direction off the star is the same place, not a hotter one");
    check(near_d(surfmapGroundT(anchor, knob, 0.4 * off, sun, 0.0), off_T, 1e-9),
          "and so is a shorter one");
    // Three places a third of a turn apart on the equator: at least one is in night and at
    // least one is lit, whatever the star's bearing -- which is why an e2e case could ask for
    // this spread without knowing where local noon is. Pinned here instead of there, because
    // the pixels a hover needs are window-layout coordinates.
    double t[3];
    for(int k = 0; k < 3; k++) {
        const double a = k * 2.0 * std::numbers::pi / 3.0;
        t[k] = surfmapGroundT(anchor, knob, glm::dvec3(std::cos(a), 0.0, std::sin(a)), sun, 0.0);
    }
    const double lo = std::min(std::min(t[0], t[1]), t[2]);
    const double hi = std::max(std::max(t[0], t[1]), t[2]);
    check(near_d(lo, duna.lo_K, 1e-9), "of three places 120 deg apart, the coldest is in night");
    check(hi > lo + 1.0, "and the warmest is lit by more than rounding");
}

/* #793: the ground's OWN height. Every file already states how fast its air cools with height
   (Earth -0.0065 K/m to 11 km, Mars -0.004 to 16.5), yet the ground law priced every pixel at the
   sea-level datum, so a summit read as warm as a beach. What belongs HERE is what neither the
   window nor the sweep can check: the term's SHAPE (additive, outside the fourth-power bracket),
   its edges (0 at and below the datum, holding above the air's top), the gate (which worlds have a
   height term at all), and the span the sweep colours with. A full bake of it -- the map's cold
   end moving -- is e2e render/745's job. */
static Surface makeEarthish() {
    Surface e;
    e.atmosphere.enabled = true;
    // A profile needs a density to be PHYSICAL (terragen.h:physical()), not just a table: Io and
    // Triton ship layers-less rims, and the gate that keeps them relief-free is this one.
    e.atmosphere.sea_level_density = 1.225;
    AtmoLayer trop;
    trop.base_alt = 0.0; trop.base_T = 288.15; trop.lapse = -0.00648878;
    AtmoLayer strat;
    // Continuous with the troposphere at the seam, because atmo.h:resolveAtmoLevels refuses a
    // table that steps there -- and airProfileOffset_K picks its level by base altitude, so a
    // hand-authored step here would be a file the loader would have rejected.
    strat.base_alt = 11000.0; strat.base_T = 216.77; strat.lapse = 0.001;
    e.atmosphere.layers.push_back(trop);
    e.atmosphere.layers.push_back(strat);
    e.atmosphere.height = 60000.0;
    e.even_share = 0.300;
    return e;
}

/* #794: a body that AUTHORS a lag, shaped like the lag_playground.json fixture -- a STATED anchor
   (so no rail enters the numbers), a near-bare-rock knob (so the swing is wide and the lag is
   tens of kelvin rather than a rounding hair), no air (so relief cannot be mistaken for the lag),
   and one radian of phase. */
static Surface makeLagged() {
    Surface s;
    s.temperature = 240.0;
    s.even_share = 0.05;
    s.lag_tau = 1.0;
    return s;
}

static void groundRelief() {
    const Surface earth = makeEarthish();
    check(earth.reliefOn(), "a body whose air lapses has a height term");
    check(earth.relief_K(0.0) == 0.0, "the datum is the datum: a coast pays no height term");
    check(earth.relief_K(-4000.0) == 0.0,
          "a sea bed reads the datum's air, not level 0's lapse run backwards into the hottest "
          "ground in the game (drag.h's flat sub-datum column, #245)");
    check(near_d(earth.relief_K(8000.0), -0.00648878 * 8000.0, 1e-9),
          "an 8 km summit costs exactly the file's own lapse times its height");
    check(earth.relief_K(11000.0) < earth.relief_K(8000.0),
          "and it is still colder across the seam into the next level");
    check(near_d(earth.relief_K(9.0e6), earth.relief_K(earth.atmosphere.top()), 1e-9),
          "above the air's top the term HOLDS at the top: airTemperature's 0 K up there is a "
          "statement about density, and a 22 km volcano on a 16.5 km profile does not get a "
          "free 143 K");

    // ---- which worlds have one at all ---------------------------------------
    check(!makeDuna().reliefOn(), "a column that never lapses states no gradient, so no term");
    Surface bare;
    bare.even_share = 0.3;
    check(!bare.reliefOn(), "airless: a summit reads what its coast reads, for free");
    Surface giant = makeEarthish();
    giant.bands = true;
    check(!giant.reliefOn(), "a banded giant has no ground to raise");
    check(Surface{}.relief_K(9000.0) == 0.0, "and asking anyway answers 0 rather than a NaN");

    // ---- the SHAPE, which is the design decision ----------------------------
    const TempLayer tl = tempLayerFor(makeEarthish(), 0.0, true);
    check(tl.relief, "a relief-aware layer carries the profile, exactly as a wind layer carries "
                     "its rows: numbers and tables to the worker, never body pointers");
    const glm::dvec3 sun(1.0, 0.0, 0.0);
    const double climb = earth.relief_K(8000.0);
    const double coast = surfmapGroundT(tl.anchor_K, tl.knob, sun, sun, 0.0);
    const double summit = surfmapGroundT(tl.anchor_K, tl.knob, sun, sun, climb);
    check(near_d(summit, coast + climb, 1e-9),
          "the height term is ADDITIVE: the fourth-power bracket is insolation, and a mountain "
          "is not more sunlight");
    check(summit < coast - 50.0, "so the same place at 8 km reads more than 50 K colder");
    check(near_d(tl.lo_K, groundTemperature(tl.anchor_K, tl.knob, 0.0), 1e-9)
             && near_d(tl.hi_K, groundTemperature(tl.anchor_K, tl.knob, 1.0), 1e-9),
          "the layer's own ends stay relief-free: they are the law's ends, which the identity "
          "e2e science/724 checks against the anchor");

    // ---- the span the sweep colours with (#769's precedent for the wind ramp) -
    double rlo = 0.0, rhi = 0.0;
    tempRampEnds(tl, 0.0, 8848.0, rlo, rhi);
    check(near_d(rlo, tl.lo_K + earth.relief_K(8848.0), 1e-9),
          "the cold end is the law's night end standing on the highest ground");
    check(near_d(rhi, tl.hi_K, 1e-9),
          "the hot end does not move, because the lowest ground IS the datum and the term is 0 there");
    check(rlo > 0.0 && rhi > rlo,
          "and a relief-aware palette still spans positive kelvin -- the assert in tempRampEnds "
          "is a data guard, not a live abort");
    // Off the datum the sweep's pixel and the cursor's kelvin are ONE call (surfmapGroundT), so
    // a colour and the number beside it cannot drift apart; pinned above for the datum case.
    check(near_d(surfmapGroundT(tl.anchor_K, tl.knob, sun, sun, climb),
                 tl.hi_K + climb, 1e-9),
          "under the star at 8 km: the day end minus the climb, same term as the cold end");
}

/* #794: the skin allowed to lag the light. The law itself -- groundtemp.h's closed form, its
   endpoint constants, its anchor theorem -- has its own battery in tests/test_groundtemp.cpp, RK4
   cross-check and all. What belongs HERE is the wiring that header cannot see: that the hour angle
   a row is evaluated at is THIS grid's longitude less the star's subsolar meridian, and that the
   palette the sweep colours with spans what a lagged field can actually paint. A flipped sign in
   that wiring mirrors the picture instead of shifting it, and nothing about a still frame of the
   law alone would notice -- cos h is even in h. */
static void groundLag() {
    const double anchor = 240.0, knob = 0.05, tau = 1.0;   // the lag_playground fixture
    const double d2r = std::numbers::pi / 180.0;

    // ---- the hour angle is the grid's own longitude less the star's meridian ----
    // tau 0 answers the FORCING itself, so this is the one place the lag's coordinate system is
    // pinned against solar.h's cosine of two real directions: same latitude, same longitude, same
    // star. It catches a row built from the wrong declination, a lon/lat convention flipped
    // (equirect.h's lon +90 = -Z is easy to get backwards), and a subsolar point read from the
    // wrong frame.
    const double dec_deg = -20.0, lon_ss_deg = 130.0;
    const glm::dvec3 sun = equirectDir(lon_ss_deg * d2r, dec_deg * d2r);
    double slon = 0.0, sdec = 0.0;
    subsolarLonLat(sun, slon, sdec);
    check(near_d(sdec, dec_deg * d2r, 1e-12), "subsolarLonLat reads the star's declination");
    check(near_d(slon, lon_ss_deg * d2r, 1e-12), "and its subsolar longitude");
    // An un-normalised sun vector is the same noon: a direction is a bearing.
    double rlsl = 0.0, rsdc = 0.0;
    subsolarLonLat(4.0 * sun, rlsl, rsdc);
    check(near_d(rlsl, slon, 1e-12) && near_d(rsdc, sdec, 1e-12),
          "and reads it off a direction of any length");

    double worst = 0.0;
    for(int latdeg = -80; latdeg <= 80; latdeg += 10) {
        const double lat = latdeg * d2r;
        const GroundLagRow flat = groundLagRow(lat, sdec, 0.0);
        for(int londeg = 0; londeg < 360; londeg += 15) {
            const double lon = londeg * d2r;
            const double via_row = flat.at(lon - slon);
            const double via_dot = cosIncidence(equirectDir(lon, lat), sun);
            worst = std::max(worst, std::fabs(via_row - via_dot));
        }
    }
    check(worst < 1e-12,
          "a lag-free row at (lon - lon_ss) IS solar.h's incidence at that place (#794's "
          "coordinate system, pinned against the sphere the engine actually paints)");

    // ---- the peak sits EAST of noon, and only because of the lag ----
    // Same row, tau 0 against tau 1: the forcing's own peak is due noon (the hot spot sits under
    // the star, which is the complaint #794 exists to fix), and the lagged field's peak has to
    // move EAST -- the sign of that is the temporal half of the convention, and a mirror image
    // would score perfectly on every other test in this file.
    struct RowScan { double peak_h_deg; double lo, hi; };
    auto scan = [&](double lat, double t) {
        const GroundLagRow row = groundLagRow(lat, sdec, t);
        double lo = 1e30, hi = -1e30, peak_h = 0.0;
        for(int k = 0; k < 720; k++) {
            const double h = -std::numbers::pi + (double)k * std::numbers::pi / 360.0;
            const double y = row.at(h);
            if(y < lo) { lo = y; }
            if(y > hi) { hi = y; peak_h = h; }
        }
        return RowScan{ std::remainder(peak_h, 2.0 * std::numbers::pi) / d2r, lo, hi };
    };
    const RowScan now = scan(0.0, 0.0);
    const RowScan later = scan(0.0, tau);
    check(std::fabs(now.peak_h_deg) < 0.5, "with no lag the hottest ground is due noon");
    check(later.peak_h_deg > 20.0 && later.peak_h_deg < 60.0,
          "a lag of one radian puts it in the afternoon, east of the star");
    check(later.hi < now.hi && later.hi > 0.0,
          "and pays for the shift with amplitude: the row's peak share falls");
    check(scan(0.0, 2.0 * std::numbers::pi).lo > now.lo,
          "at a lag of a whole rotation the night floor RISES too -- the legend lie is two-sided, "
          "so a palette priced from the law's 0 and 1 misses both ends");

    // ---- the palette spans what a lagged picture can paint ----
    const Surface lag_surf = makeLagged();
    const TempLayer tl = tempLayerFor(lag_surf, 0.0, true);
    check(near_d(tl.lag_tau, tau, 1e-12),
          "the layer carries the authored knob to the sweep (and to the cursor, which has to read "
          "the same world the pixels came from)");
    check(tl.lo_K == groundTemperature(anchor, knob, 0.0)
              && tl.hi_K == groundTemperature(anchor, knob, 1.0),
          "the LAW's ends stay the law's: they are what the one knob forces, which is the row "
          "e2e science/724 checks -- the picture is allowed to stop reaching them, the identity "
          "is not allowed to move with the palette");

    /* ---- what a lagged picture can actually paint ----
       The sweep no longer PAIRS the extremes ("the darkest light standing on the tallest ground")
       to price a lagged palette; it measures the field. The pairing over-spans, because a lagged
       world's darkest forcing lives in its dark polar rows and its tallest ground does not: on a
       lapse-bearing scratch body that left 8.0 K of palette no pixel could reach
       (temp=[107.19 ...] against temp_ramp=[99.18 ...]), which is the #793 bug in the new knob's
       clothes. The reach itself is a sweep property and lives in e2e
       render/794-a-lagged-world-with-mountains-still-prices-its-ends; what belongs HERE is what
       the field is bounded by, which the sweep's own assert leans on. */
    double ylo = 1e30, yhi = -1e30, tlo = 1e30, thi = -1e30;
    for(int latdeg = -85; latdeg <= 85; latdeg += 5) {
        const GroundLagRow row = groundLagRow(latdeg * d2r, sdec, tl.lag_tau);
        for(int londeg = 0; londeg < 360; londeg += 3) {
            const double y = row.at(londeg * d2r - slon);
            if(y < ylo) { ylo = y; }
            if(y > yhi) { yhi = y; }
            const double T = groundTemperatureAt(anchor, knob, y, 0.0);
            if(T < tlo) { tlo = T; }
            if(T > thi) { thi = T; }
        }
    }
    check(ylo >= 0.0 && yhi <= 1.0 + 1e-9,
          "a lagged forcing stays inside the box the law is defined on -- the assert the sweep "
          "puts on its own pre-pass is for the wiring, not for the maths");
    check(thi < tl.hi_K, "the hottest paintable pixel sits INSIDE the law's day end -- that is the "
                         "#794 flattening, and the reason temp_ramp= is not temp_ends= for a "
                         "lagged body");
    check(tlo >= tl.lo_K, "and the coldest sits inside the law's night end");
    check(thi > tlo + 10.0, "and the palette is not degenerate: this body's swing is wide");

    // ---- the cursor and the sweep are one spelling ----
    /* #829 took the cursor off a surfmap.h helper of its own and onto groundtemp.h's
       groundForcingLagged(), the same call the --solar-log column and the air under a hull now
       make. What this pins is that the call answers the row: same place, same hour angle, same
       height term, to the last bit. A cursor priced from a different spelling is a number that
       does not match the colour beside it. */
    const double clim = -12.0;
    const glm::dvec3 at = equirectDir(100.0 * d2r, 20.0 * d2r);   // lon 100, lat 20
    check(near_d(groundTemperatureAt(anchor, knob, groundForcingLagged(tau, at, sun), clim),
                 groundTemperatureAt(anchor, knob,
                                     groundLagRow(20.0 * d2r, sdec, tau).at(100.0 * d2r - slon),
                                     clim),
                 0.0),
          "the number beside a lagged pixel IS that pixel's number: same row, same hour angle, "
          "same height term");
    // And it is NOT the unlagged answer -- a lagged map read through surfmapGroundT would print a
    // noon kelvin under an afternoon colour. Pinned on a place where the two differ loudly.
    const glm::dvec3 dawn = equirectDir(40.0 * d2r, 0.0);         // lon 40, lat 0: sunrise there
    const double unlagged = surfmapGroundT(anchor, knob, dawn, sun, 0.0);
    const double lagged = groundTemperatureAt(anchor, knob,
                                              groundForcingLagged(tau, dawn, sun), 0.0);
    check(lagged > unlagged + 5.0,
          "at an hour angle the lag has already paid for, the lagged ground is warmer than the "
          "instantaneous one -- so a cursor that forgot the lag would be measurably wrong");
    /* The other half of #829: the two readers that used to ignore tau hold a cosine they measured
       for a frame-invariant reason (the dot does not care which axes it is read in). This says
       the shared helper agrees with that cosine when there is nothing to lag, which is what lets
       both call sites keep the plain cosine for an unlagged world instead of paying for a row --
       and it is the pin that says the rotating-frame step the lag needs is the ONLY thing the lag
       changes about the pair. */
    double worst_flat = 0.0;
    for(int latdeg = -80; latdeg <= 80; latdeg += 10) {
        for(int londeg = 0; londeg < 360; londeg += 15) {
            const glm::dvec3 z = equirectDir(londeg * d2r, latdeg * d2r);
            worst_flat = std::max(worst_flat, std::fabs(groundForcingLagged(1e-9, z, sun)
                                                        - cosIncidence(z, sun)));
        }
    }
    check(worst_flat < 1e-6,
          "at tau -> 0 the shared forcing IS that cosine: the lag adds a phase, not a new cosine");
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

    // 2. Air the file states no wind for. In the shipped solar system that is Pluto alone
    // since #772; ksp_system.json's six Kerbin-side airs all take this path too.
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
    /* BANDED rows (#767): the sweep paints the whole sphere, so the ramp spans what the
       ground sees WHEREVER it stands -- the union over bands, never a blend of them. Here a
       base column says 5 m/s at the datum and 25 m/s at 20 km, and a tropical belt answers
       for lat [0,10) with 100 m/s at 10 km and 200 m/s at 30 km. Interpolating the four rows
       as one column would price 52.5 m/s at 5 km -- a speed no pixel on that world shows. */
    {
        std::vector<WindLevel> banded = {
            {0.0, 5.0, 0.0}, {20000.0, 25.0, 0.0},
            {10000.0, 100.0, 0.0, 0.0, 0.0, 10.0}, {30000.0, 200.0, 0.0, 0.0, 0.0, 10.0},
        };  // alt, east, north, up, lat0, lat1 -- the whole-sphere lon pair is the default
        double lo, hi;
        windRampEnds(banded, -100.0, 5000.0, lo, hi);
        check(near_d(lo, 5.0, 1e-12), "the base column's deck still sets the floor");
        check(near_d(hi, 100.0, 1e-12),
              "the belt holds at its own low row over the same ground, so it sets the ceiling");
        check(hi < 200.0, "and the belt's 30 km row stays out of a ramp the ground never reaches");
        const std::vector<WindLevel> base_only = { banded[0], banded[1] };
        windRampEnds(base_only, -100.0, 5000.0, lo, hi);
        check(near_d(hi, 10.0, 1e-9),
              "without the belt the same ground spans 5..10 -- the belt adds a hot end, not a blend");
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
    /* ---- which ROWS of the map get arrows (#780) --------------------------------
       The choice is a pure function of the table and the bake, so it is pinned here rather than
       only in front of a screen. What it decides is whether the belt structure of a banded
       table reads as rings of arrows, or whether the uniform lattice draws because ringing would
       be worse. h = 256, stride = 16, half = 6.72, head = 4.8 is the shipped --surfmap-n 512. */
    {
        // A one-column table -- a whole-sphere column, which is what Venus, Earth, Mars and
        // Titan still author -- has no band centre worth ringing.
        const std::vector<WindLevel> one = { {0.0, 5.0, 0.0}, {11000.0, 40.0, 0.0} };
        const ArrowRowPlan g = surfmapArrowRows(one, 256, 16, 16 * 0.42, 16 * 0.30);
        check(g.bands == 1 && !g.from_bands && g.rings.empty(),
              "a whole-sphere column keeps the uniform grid and says so");
        check(g.grid.size() == 16 && g.grid.front() == 8 && g.grid.back() == 248,
              "16 rows, the first at half a stride in and none of them a pole row");
        /* An ODD bake size lets the lattice reach a map's last row: --surfmap-n 26 has h = 13,
           so its rows are 4 and 12, and 12 is the south pole. Skipped, not drawn. */
        const ArrowRowPlan odd = surfmapArrowRows(one, 13, 8, 8 * 0.42, 8 * 0.30);
        check(odd.grid.size() == 1 && odd.grid[0] == 4,
              "the grid skips a bake's last row, which is its south pole");
    }
    {
        // 13 even bands: rings, one per band, ASCENDING in pixel order -- which is descending in
        // latitude, the order the band list arrives sorted in. The first cut compared the unsorted
        // gaps, found them negative, and every body silently took the grid.
        std::vector<WindLevel> belts;
        for(int k = 0; k < 13; k++) {
            belts.push_back({0.0, 10.0, 0.0, 0.0,
                             -90.0 + 180.0 * (double)k / 13.0,
                             -90.0 + 180.0 * (double)(k + 1) / 13.0});
        }
        const ArrowRowPlan p = surfmapArrowRows(belts, 256, 16, 16 * 0.42, 16 * 0.30);
        check(p.bands == 13 && p.from_bands && p.rings.size() == 13,
              "13 bands ring 13 rows");
        bool asc = true;
        for(size_t k = 1; k < p.rings.size(); k++) { asc = asc && p.rings[k] > p.rings[k - 1]; }
        check(asc, "and the rings come back in pixel order, north pole first");
        check(p.rings.front() > 0 && p.rings.back() < 255, "with no ring on a pole row");
    }
    {
        // The minimum is asked of rows DRAWN, not of bands found. 20 distinct pairs in two clumps
        // are 20 belts on paper and a couple of rows of ink: a picture sparser than the grid it
        // replaced has lost the coverage wind_cells= exists to guard.
        std::vector<WindLevel> clumps;
        for(int k = 0; k < 10; k++) {
            const double a = (double)k * 0.2, b = 60.0 + (double)k * 0.2;
            clumps.push_back({0.0, 10.0, 0.0, 0.0, a, a + 0.2});
            clumps.push_back({0.0, 10.0, 0.0, 0.0, b, b + 0.2});
        }
        const ArrowRowPlan p = surfmapArrowRows(clumps, 256, 16, 16 * 0.42, 16 * 0.30);
        check(p.bands == 20, "20 distinct bands on paper");
        check(!p.from_bands && p.rings.empty(), "and the grid draws, because they ring ~6 rows");
    }
    {
        // The spacing guard reads the TABLE, not the bake. The same 16 even bands ring when the
        // air blows along the parallels, where an arrow's vertical footprint is the head's spread,
        // and must not ring meridionally, where it is the shaft's whole length and the rings would
        // draw on top of each other -- the one smear the guard exists to prevent.
        std::vector<WindLevel> zon, mer;
        for(int k = 0; k < 16; k++) {
            const double lat0 = -90.0 + 180.0 * (double)k / 16.0;
            const double lat1 = -90.0 + 180.0 * (double)(k + 1) / 16.0;
            zon.push_back({0.0, 10.0, 0.0, 0.0, lat0, lat1});
            mer.push_back({0.0, 0.0, 10.0, 0.0, lat0, lat1});
        }
        const ArrowRowPlan pz = surfmapArrowRows(zon, 256, 16, 16 * 0.42, 16 * 0.30);
        const ArrowRowPlan pm = surfmapArrowRows(mer, 256, 16, 16 * 0.42, 16 * 0.30);
        check(pz.from_bands && pz.rings.size() == 16,
              "a zonal field of 16 belts rings all 16");
        check(!pm.from_bands,
              "a meridional one of the same bands does not: its arrows are taller than the gaps");
    }
    {
        // A thin polar cap centres ON the pole, where every direction is south and the row would
        // draw one degenerate arrow the whole width of the map. It drops out; the rest ring.
        std::vector<WindLevel> capped = { {0.0, 10.0, 0.0, 0.0, 89.6, 90.0} };
        for(int k = 0; k < 11; k++) {
            capped.push_back({0.0, 10.0, 0.0, 0.0,
                              -90.0 + 179.0 * (double)k / 11.0,
                              -90.0 + 179.0 * (double)(k + 1) / 11.0});
        }
        const ArrowRowPlan p = surfmapArrowRows(capped, 256, 16, 16 * 0.42, 16 * 0.30);
        check(p.bands == 12 && p.rings.size() == 11 && p.from_bands,
              "the cap counts as a band but not as a row");
        for(int j : p.rings) {
            check(j != 0 && j != 255, "and no ring ever lands on a pole row");
        }
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
    {
        /* The hoisted per-row/per-column trig (#802) is the SAME grid, not an approximation
           of it. The sweep takes this form for speed, so the speed had better not be bought
           with pixels: every index pair has to give the bits equirectDirAt(i, j, w, h) gave
           before, exactly -- hence == and not near_v, which here would make the check
           vacuous. Several grids rather than the one 16x8 this file otherwise uses: the two
           spellings divide by w and by h-1 in different places, so an even width, an odd one,
           the 2x2 degenerate and a real bake size are different paths through it. {28, 14}
           is here because h=14 is the first row count whose computed south latitude misses
           -pi/2 (#844): if the pin ever survives in one spelling but not the other, this is
           the grid that says so. If this ever fails, the perf change has become a
           re-baseline. */
        const int kGrids[][2] = { {2, 2}, {3, 2}, {16, 8}, {17, 8}, {26, 13},
                                  {255, 127}, {512, 256}, {28, 14} };
        bool exact = true;
        for(auto &g : kGrids) {
            const EquirectTrig t = equirectTrig(g[0], g[1]);
            if((int)t.cos_lon.size() != g[0] || (int)t.cos_lat.size() != g[1]) {
                exact = false;   // the tables are not the grid they were asked for
                continue;
            }
            for(int j = 0; j < g[1] && exact; j++) {
                for(int i = 0; i < g[0] && exact; i++) {
                    const glm::dvec3 a = equirectDirAt(i, j, g[0], g[1]);
                    const glm::dvec3 b = equirectDirAt(t, i, j);
                    exact = a.x == b.x && a.y == b.y && a.z == b.z;
                    /* The ANGLES are table entries too, and they are the same expressions --
                       not a second derivation the lag (#794) is free to disagree with. A row
                       built from t.lat and an hour angle built from t.lon are the latitude and
                       longitude this picture is actually coloured at. */
                    const double lon = 2.0 * std::numbers::pi * (double)i / (double)g[0];
                    /* Re-spelled here rather than calling equirectRowLat(), so this stays an
                       independent reference and not a tautology -- including its pinned south
                       endpoint, which is part of the law now (#844: the computed
                       pi*(h-1)/(h-1) is not pi, and the row that lands under -pi/2 gets a
                       negative cos and a mirrored direction). */
                    const double lt = (j == g[1] - 1)
                                    ? -std::numbers::pi * 0.5
                                    : std::numbers::pi * 0.5
                                      - std::numbers::pi * (double)j / (double)(g[1] - 1);
                    if(t.lon[i] != lon || t.lat[j] != lt) { exact = false; }
                    if(std::cos(t.lon[i]) != t.cos_lon[i]
                       || std::sin(t.lat[j]) != t.sin_lat[j]) { exact = false; }
                }
            }
        }
        check(exact, "trig tables: bit-for-bit the same directions and angles at every pixel");
    }
    {
        /* The DEGREES tables (#824), which a place-keyed sweep (windAt) reads instead of
           recovering a lat/lon per pixel with asin + atan2 + fmod. Pinned per pixel rather
           than by trusting the builder, because the claim being made is that the sweep and
           the one-off inverse ask the SAME question of the same grid.
           LATITUDE is bit-for-bit the per-pixel inverse at every pixel: that half runs
           through the direction's y, and y is the row's sin_lat in every column. Not an
           empirical accident -- both spellings evaluate asinUnit on the same bits.
           LONGITUDE is not, and the whole gap is an ulp or two -- interior rows AND the south
           row, since #844 pinned that endpoint. 6-7% of pixels sit that far from their
           column's number (worst 5.684e-14 deg at 1024x512), because the inverse scales BOTH
           atan2 arguments by the row's cos(lat) and scaling two doubles rounds differently per
           row. That is one column answered differently in different rows, which is what the
           table ends; equirect.h says what a band edge inside the envelope means. Pinned at
           1e-12 deg, 18x the worst gap seen.
           The south row used to be a different story: a computed pi/2 - pi*(h-1)/(h-1) lands
           below -pi/2 at some heights, its cos comes out NEGATIVE, the row's direction points
           at the antipodal meridian and the inverse answers exactly 180 deg off. {28, 14} and
           {1440, 720} stay in the list as the shapes that used to trip it; the endpoint check
           below is what keeps that from coming back. */
        const int kGrids[][2] = { {2, 2}, {3, 2}, {16, 8}, {17, 8}, {26, 13},
                                  {255, 127}, {512, 256}, {1024, 512},
                                  {28, 14}, {1440, 720} };
        bool lat_exact = true;
        bool sizes_ok = true;
        bool poles_pinned = true;    // rows 0 and h-1 really are the poles
        double worst_lon = 0.0;
        for(auto &g : kGrids) {
            const int w = g[0], h = g[1];
            const EquirectTrig t = equirectTrig(w, h);
            if((int)t.lon_deg.size() != w || (int)t.lat_deg.size() != h) {
                sizes_ok = false;   // the tables are not the grid they were asked for
                continue;
            }
            /* By value, not by construction, and at BOTH ends: a grid that misses the pin
               reads a latitude a hair off -pi/2 (a hair UNDER means a negative cos and a
               mirrored row, #844), and the north end is exact only by argument. == on t.lat
               compares the pinned constant; the trig that comes out of it gets a tolerance,
               because sin(-pi/2) == -1 and (pi/2)*(180/pi) == 90 are libm courtesies rather
               than laws. */
            if(t.lat[0] != std::numbers::pi * 0.5
               || t.lat[(size_t)h - 1] != -std::numbers::pi * 0.5) { poles_pinned = false; }
            if(t.cos_lat[0] <= 0.0 || t.cos_lat[(size_t)h - 1] <= 0.0) { poles_pinned = false; }
            if(!near_d(t.sin_lat[0], 1.0, 1e-12)
               || !near_d(t.sin_lat[(size_t)h - 1], -1.0, 1e-12)
               || !near_d(t.lat_deg[0], 90.0, 1e-12)
               || !near_d(t.lat_deg[(size_t)h - 1], -90.0, 1e-12)) { poles_pinned = false; }
            for(int j = 0; j < h; j++) {
                for(int i = 0; i < w; i++) {
                    double lo = 0.0, la = 0.0;
                    equirectLonLatDeg(equirectDirAt(t, i, j), lo, la);
                    if(la != t.lat_deg[(size_t)j]) { lat_exact = false; }
                    const double gap = std::fabs(lo - t.lon_deg[(size_t)i]);
                    if(gap > worst_lon) { worst_lon = gap; }
                }
            }
        }
        check(lat_exact, "degrees tables: latitude bit-for-bit the per-pixel inverse");
        check(sizes_ok, "degrees tables: one entry per column and per row");
        check(worst_lon <= 1e-12, "degrees tables: longitude within 1e-12 deg of it");
        check(poles_pinned, "degrees tables: both end rows are the poles, cos and all (#844)");
        std::printf("test_surfmap: lon_deg worst gap against the per-pixel inverse:"
                    " %.3e deg over %d grids\n",
                    worst_lon, (int)(sizeof(kGrids) / sizeof(kGrids[0])));
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

    /* 7b. The cloud bake's OWN layout tables (#825). The deck is baked on the TILEABLE
       layout -- pixel centres, v scaled by H -- not the poles-on-rows grid the surface map
       uses, and its UV has to keep matching cloudShader.vs. Being handed the OTHER layout is a
       half-pixel slide of the whole cover that no log would show, and a perf rewrite is exactly
       the change that silently re-baselines pixels, so the hoisted tables are pinned bit-for-bit
       to the per-pixel spelling they replaced. (Not because the deck could show one ulp -- the
       coverage map is R8 -- but because there is nothing else watching.) The formulas are
       re-spelled inline below so this stays an independent reference rather than a call to the
       thing it pins -- the same trade the #802 pin makes for the other layout, which sweeps
       every index of its grids too. */
    {
        const int kDeck[][2] = { {4, 2}, {7, 3}, {64, 32},
                                 {kCloudBakeW, kCloudBakeH} };
        bool exact = true;
        for(auto &d : kDeck) {
            const int W = d[0], H = d[1];
            const EquirectTrigCentres t = equirectTrigCentres(W);
            if((int)t.cos_lon.size() != W || (int)t.sin_lon.size() != W) {
                exact = false;   // not the grid it was asked for
                continue;
            }
            /* EVERY pixel of the shipped deck: measured, the full 2,097,152 comparisons cost
               ~11 ms, which is not worth trading coverage of a column for. */
            for(int j = 0; j < H; j++) {
                /* The row's trig computed once, the way the bake computes it -- equirectDir
                   below derives cos/sin from the same latitude itself, so what is being
                   compared is the same bits arriving from a table instead of a register. */
                const float lat = (0.5f - ((float)j + 0.5f) / (float)H)
                                * (float)std::numbers::pi;
                const float cl = std::cos(lat), sl = std::sin(lat);
                for(int i = 0; i < W; i++) {
                    const float lon = ((float)i + 0.5f) / (float)W
                                    * 2.0f * (float)std::numbers::pi;
                    const glm::vec3 a = equirectDir(lon, lat);
                    const glm::vec3 b = equirectDirAt(t, i, cl, sl);
                    if(a.x != b.x || a.y != b.y || a.z != b.z) { exact = false; }
                }
            }
            /* The row angles never reach the loop above's tables -- the bake hands the row's
               cos/sin in from its own arithmetic -- so they are pinned here against the inline
               spelling rather than trusting the header to agree with itself. A centres grid has
               NO pole row: row 0 sits a half-row south of north and the last row a half-row
               north of south. If the centre law ever became the poles-on-rows law (row 0 IS the
               pole), the first clause fires; if only the table builder drifted, the pixel loop
               above does. */
            const float pi_f = (float)std::numbers::pi;
            const float north = (0.5f - 0.5f / (float)H) * pi_f;
            const float south = (0.5f - ((float)H - 0.5f) / (float)H) * pi_f;
            if(equirectCentreLat(0, H) != north
               || equirectCentreLat(H - 1, H) != south) { exact = false; }
            if(!(equirectCentreLat(0, H) < pi_f * 0.5f)
               || !(equirectCentreLat(H - 1, H) > -pi_f * 0.5f)) { exact = false; }
        }
        check(exact, "cloud bake: centre tables bit-for-bit the per-pixel bake, no pole rows");
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
    groundRelief();
    groundLag();
    windLayers();

    if(g_failures == 0) {
        std::printf("test_surfmap: all checks passed\n");
        return 0;
    }
    std::printf("test_surfmap: %d check(s) failed\n", g_failures);
    return 1;
}
