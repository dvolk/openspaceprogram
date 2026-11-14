#pragma once
// equirect.h -- the game's ONE equirectangular (lon, lat) <-> direction
// convention over a body's ROTATING frame, and the shared map layout.
//
// Axes: north = +Y, lon 0 = +X (the rail longitude zero, #146/#200),
// lon +pi/2 = -Z (east). Inverse is lon = atan2(-z, x), unwrapped to
// [0, 2pi). Forward is (cos lat * cos lon, sin lat, -cos lat * sin lon).
// Surface lon 0 is the body's prime meridian (rot-frame +X = WGCCRE /
// spin_phase0), so map, HUD readout and rail longitudes share one azimuth.
//
// Map layout (surfmap pixels, heightmaps, cloud coverage): lon 0 at the
// LEFT edge (column 0), north up (row 0). Heightmap / surfmap grids put
// the poles ON the first/last row (v scaled by h-1); a tileable bake
// (cloud coverage) instead samples pixel CENTRES and may use v in [0,1]
// scaled by H. Same direction formulas -- only the grid parametrisation
// differs.
//
// Everything that maps a surface direction to a map column (or back)
// MUST go through these. No local atan2 / sin/cos lon formulas.
//
// create_atmosphere_mesh (terrain.cpp) carries unwrapped phi = atan2(z, x)
// in the vertex colour slot (phi = 0 at +X toward +Z). Surface lon is
// -phi (mod 2pi), so the cloud deck's u = lon/2pi is -phi/2pi
// (cloudShader.vs; texture REPEAT closes the seam).

#include <cassert>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include <glm/glm.hpp>

#include "angmath.h"   // asinUnit: the bounded-ratio spelling (#622)

/* "No coordinate was given", for the DEGREES spelling the authored coordinates
   below use: a world_buildings pad's lat_deg/lon_deg, a start ship's lat/lon
   (--startship / --startships). It lives here rather than in constants.h because
   it changes nothing in the game -- it is the ABSENCE of one of these coordinates.
   NaN and not a number outside the range: every pair in [-90,90] x (any angle) is
   a real place, so a range sentinel is a value someone could type into the live
   path. NaN never equals itself, so test it with isnan(), never `!= kNoCoord`. A
   NaN that reaches a spawn is a half-filled pair, which the spawn asserts on
   rather than guesses. */
inline constexpr double kNoCoord = std::numeric_limits<double>::quiet_NaN();

/* The lon half of the authored-coordinate law: wrap once, at the parse, into
   [0, 360). Wrapping there is what makes the angle the trig sees and the number a
   readout echoes the SAME number: a raw 1e308 fed to cos/sin is an arbitrary
   meridian with no precision left to say which, while an echo that fmod'd it
   afterwards would print a tidy place the ship is not at. A wrapped 1e308 is
   still an arbitrary meridian -- it is just the honest one. Same law as the
   inverse above, which unwraps lon into [0, 2pi). */
inline double wrapLonDeg(double lon_deg) {
    const double w = std::fmod(lon_deg, 360.0);
    return w < 0.0 ? w + 360.0 : w;
}

// (lon, lat) -> unit direction. lon 0 = +X, +pi/2 = -Z, north = +Y.
inline glm::dvec3 equirectDir(double lon, double lat) {
    const double cl = std::cos(lat);
    return glm::dvec3(cl * std::cos(lon), std::sin(lat), -cl * std::sin(lon));
}

inline glm::vec3 equirectDir(float lon, float lat) {
    const float cl = std::cos(lat);
    return glm::vec3(cl * std::cos(lon), std::sin(lat), -cl * std::sin(lon));
}

// Unit direction -> (lon, lat), lon unwrapped to [0, 2pi).
inline void equirectLonLat(const glm::dvec3 &d, double &lon, double &lat) {
    lat = asinUnit(d.y);
    lon = std::atan2(-d.z, d.x);
    if(lon < 0.0) { lon += 2.0 * std::numbers::pi; }
}

/* The same inverse in the DEGREES spelling an authored table is keyed by -- a wind row's
   lat/lon band (#767), a pad's lat_deg/lon_deg. Spelled once here rather than as a
   rad2deg at each call site, so the number a file is keyed by and the number a direction
   unwraps to cannot drift apart. wrapLonDeg() runs AFTER the scale: 2*pi rounds to a 360
   that no half-open band covers, and a place outside every band is not a place. */
inline void equirectLonLatDeg(const glm::dvec3 &d, double &lon_deg, double &lat_deg) {
    double lon = 0.0, lat = 0.0;
    equirectLonLat(d, lon, lat);
    constexpr double rad2deg = 180.0 / std::numbers::pi;
    lon_deg = wrapLonDeg(lon * rad2deg);
    lat_deg = lat * rad2deg;
}

/* Local horizontal unit vectors at a surface direction, in the same rotating-frame
   axes as equirectDir: east points along INCREASING lon, north along increasing lat.
   Both are perpendicular to `up`, which is what makes them the basis a horizontal
   velocity (the wind, #688) is authored in -- a wind is a surface direction that is
   not a place, and this is the only place that basis is spelled.

   east = normalize(Y x up) is exact off the poles: for up = equirectDir(lon, lat),
   Y x up = (up.z, 0, -up.x) = cos(lat) * (the unit east). EXACTLY at a pole the
   meridian is undefined and that cross product is zero; east then falls back to
   lon 0's east, which is the limit of approaching the pole along that meridian --
   arbitrary, but never NaN and never off the horizontal plane. north = up x east
   is unit whenever up and east are unit and perpendicular. */
inline glm::dvec3 equirectEast(const glm::dvec3 &up) {
    const glm::dvec3 e(up.z, 0.0, -up.x);
    const double l2 = glm::dot(e, e);
    if(l2 < 1e-24) { return glm::dvec3(0.0, 0.0, -1.0); }   // a pole: lon 0's east
    return e * (1.0 / std::sqrt(l2));
}

inline glm::dvec3 equirectNorth(const glm::dvec3 &up, const glm::dvec3 &east) {
    return glm::cross(up, east);
}

inline void equirectLonLat(const glm::vec3 &d, float &lon, float &lat) {
    lat = std::asin(glm::clamp(d.y, -1.0f, 1.0f));
    lon = std::atan2(-d.z, d.x);
    if(lon < 0.0f) { lon += 2.0f * std::numbers::pi_v<float>; }
}

// Fractional pixel (u, v) of a w x h poles-on-rows map (surfmap /
// heightmap): u = lon/2pi * w (lon 0 at column 0), v = (pi/2 - lat)/pi
// * (h-1) (row 0 = north pole, row h-1 = south).
inline void equirectPixel(double lon, double lat, int w, int h,
                          double &u, double &v) {
    assert(w >= 2 && h >= 2);
    u = lon / (2.0 * std::numbers::pi) * (double)w;
    v = (std::numbers::pi * 0.5 - lat) / std::numbers::pi * (double)(h - 1);
}

inline void equirectPixel(float lon, float lat, int w, int h,
                          float &u, float &v) {
    assert(w >= 2 && h >= 2);
    u = lon / (2.0f * std::numbers::pi_v<float>) * (float)w;
    v = (std::numbers::pi_v<float> * 0.5f - lat) / std::numbers::pi_v<float>
      * (float)(h - 1);
}

/* Row j's latitude on a w x h poles-on-rows grid [rad]. Spelled once because both grid
   lookups below need it and the #802 pin in tests/test_surfmap.cpp compares their bits.
   The SOUTH endpoint is pinned rather than computed: pi*(h-1)/(h-1) is not pi, so the
   computed last row lands a hair off -pi/2 -- BELOW at 538 of the row counts up to 8192
   (h=14 the first), ABOVE at 545 (h=12 the first), exact at the other 7108. A hair below
   makes cos come out NEGATIVE, which mirrors the whole row's direction to the antipodal
   meridian and makes its cos(lat) area weight negative; a hair above leaves the sign alone
   but reads a cos 8x the pole's true 6.123e-17. Either way the inverse of that latitude
   lands a fraction past row h-1, so equirectPixel hands back an off-the-end row fraction
   (#844; the widths --surfmap-n accepts: 266 of 4081, 28 the first). The north end needs no
   pin: j == 0 multiplies by zero, so pi/2 - 0 is exactly pi/2 -- verified for every h in
   2..8192, and checked in the test anyway. */
inline double equirectRowLat(int j, int h) {
    assert(h >= 2 && j >= 0 && j < h);
    if(j == h - 1) { return -std::numbers::pi * 0.5; }
    return std::numbers::pi * 0.5 - std::numbers::pi * (double)j / (double)(h - 1);
}

/* Pixel (i, j) -> unit direction on a w x h poles-on-rows map (the grid sample at i, j --
   NOT the pixel centre). Two spellings of one lookup, on purpose: this one derives lon/lat
   from the indices, and is the reference the tables below are pinned against as well as what
   a one-off lookup wants; the EquirectTrig overload answers the same question with no
   transcendental, which is what a full-map sweep wants. Nothing in src/ calls this form any
   more -- every sweep builds tables -- so the tests are what keep it from drifting. */
inline glm::dvec3 equirectDirAt(int i, int j, int w, int h) {
    assert(w >= 2 && h >= 2);
    const double lon = 2.0 * std::numbers::pi * (double)i / (double)w;
    const double lat = equirectRowLat(j, h);
    return equirectDir(lon, lat);
}

/* The same grid's trig, paid for once per COLUMN and once per ROW instead of once per
   pixel (#802). A full-map sweep walks every column of every row, so `lon` repeats one
   cycle and `lat` is constant along a row: at --surfmap-n 1024 that is 524,288 sincos
   pairs to say w + h = 1,536 things.
   The tables live HERE rather than in whichever sweep wants them, because "column i is lon
   2pi*i/w, row j is lat pi/2 - pi*j/(h-1)" is this file's law -- a sweep with its own
   hoisted tables would be a second spelling free to drift, which is what the comment at the
   top of this file forbids. The directions below are BIT-FOR-BIT the ones equirectDirAt
   builds: same expressions, evaluated earlier. Pinned in tests/test_surfmap.cpp.
   POLES-ON-ROWS only -- the layout at the top of this file whose first and last rows are the
   poles. A tileable bake that samples pixel CENTRES (cloud coverage, terrain.cpp) scales v
   by H instead of h-1, so it wants its own tables; do not hand it these and let the matching
   w/h do the thinking. */
struct EquirectTrig {
    std::vector<double> cos_lon, sin_lon;   // [w], indexed by column
    std::vector<double> cos_lat, sin_lat;   // [h], indexed by row
    /* [rad] The grid's OWN angles, not only their trig. A sweep that just colours pixels never
       reads these; the lagged ground (#794) does -- its hour angle is `lon - lon_ss` and its row
       is built from the latitude. They live here rather than in that sweep because these are the
       same expressions equirectDirAt above uses, so a lag cannot be defined against a grid the
       engine does not paint (the spatial half of what tests/test_surfmap.cpp pins). */
    std::vector<double> lon, lat;
    /* [deg] The same grid's PLACES in the degrees spelling an authored table is keyed by
       (#824, and #767's wind bands): longitude per COLUMN, latitude per ROW. A sweep that
       asks a place-keyed question per pixel (windAt) needs these, and paying asin + atan2 +
       fmod per pixel for two numbers that take w + h evaluations is the mistake #802 removed
       on the forward side. Both are built by running equirectLonLatDeg on this file's own
       forward law, so a table entry is that law read at a place -- the one spelling a
       one-off inverse (the aero path reads a hull's place at src/vehicle.cpp:504) agrees with.
       lat_deg is BIT-FOR-BIT what the per-pixel inverse answers at every pixel: that half
       runs through the direction's y, and y is the row's sin_lat in every column. lon_deg is
       NOT, and the gap is worth knowing before it is read as noise:
       - Interior rows: 6-7% of pixels sit an ulp or two off their column's own number, worst
         5.684e-14 deg. Both atan2 arguments carry the row's cos(lat), and scaling two doubles
         rounds differently per row -- so the OLD per-pixel spelling answered the SAME COLUMN
         DIFFERENTLY IN DIFFERENT ROWS. The table makes a column one answer. What that can
         move: a pixel within ~1e-13 deg of an authored longitude band edge picks the other
         band, where before which band it picked depended on the row. Only
         res/systems/wind_playground.json authors a longitude band (its edges are 0 and 300).
       - The LAST row used to be worse than an ulp, and that is what equirectRowLat's pinned
         south endpoint is for: pi/2 - pi*(h-1)/(h-1) is not exactly -pi/2, and at 266 of the
         4081 widths --surfmap-n accepts (28 the first; the row count h is what decides it) it
         landed a hair below, so cos_lat[h-1] came out negative, the row's direction pointed
         at the antipodal meridian, and the per-pixel inverse answered a longitude exactly
         180 deg off the column's (#844). With the endpoint pinned the envelope above covers
         every row.
       Pinned in tests/test_surfmap.cpp. */
    std::vector<double> lon_deg, lat_deg;
};

/* equirectDirAt for a grid whose trig is already in hand: same direction, no
   transcendentals. Bounds asserted rather than assumed -- the tables carry their own w and
   h, so a caller that built them for a different bake would otherwise read past them
   instead of failing the way the (i, j, w, h) form fails. The assert is load-bearing in a
   way that form's is not: past-the-end vector indexing answers a plausible heap neighbour,
   i.e. silently wrong pixels rather than a crash, and the (i, j, w, h) form has no indexing
   to get wrong at all. No build configuration here defines NDEBUG today, so the debug builds
   that would catch it are the ones we run.

   Defined ABOVE equirectTrig because that builder calls it: the per-row degrees table is
   built by inverting this function's column-0 direction, so the latitude a wind row is keyed
   by is literally the latitude the sweep paints that row at. Row j's cos_lat/sin_lat are
   filled just before the call and every lon table entry is filled, so the asserts hold while
   the grid is still under construction. */
inline glm::dvec3 equirectDirAt(const EquirectTrig &t, int i, int j) {
    assert(i >= 0 && i < (int)t.cos_lon.size()
       && j >= 0 && j < (int)t.cos_lat.size());
    const double cl = t.cos_lat[(size_t)j];
    return glm::dvec3(cl * t.cos_lon[(size_t)i], t.sin_lat[(size_t)j],
                      -cl * t.sin_lon[(size_t)i]);
}

inline EquirectTrig equirectTrig(int w, int h) {
    assert(w >= 2 && h >= 2);
    EquirectTrig t;
    t.cos_lon.resize((size_t)w);
    t.sin_lon.resize((size_t)w);
    t.cos_lat.resize((size_t)h);
    t.sin_lat.resize((size_t)h);
    t.lon.resize((size_t)w);
    t.lat.resize((size_t)h);
    t.lon_deg.resize((size_t)w);
    t.lat_deg.resize((size_t)h);
    for(int i = 0; i < w; i++) {
        const double lon = 2.0 * std::numbers::pi * (double)i / (double)w;
        t.lon[(size_t)i] = lon;
        t.cos_lon[(size_t)i] = std::cos(lon);
        t.sin_lon[(size_t)i] = std::sin(lon);
        /* The column's OWN place: the forward law at lat 0, inverted. w evaluations of
           cos/sin more than reading the entries back, against w*h pixels that stop paying
           for it. */
        double ld = 0.0, ad = 0.0;
        equirectLonLatDeg(equirectDir(lon, 0.0), ld, ad);
        t.lon_deg[(size_t)i] = ld;
    }
    for(int j = 0; j < h; j++) {
        const double lat = equirectRowLat(j, h);
        t.lat[(size_t)j] = lat;
        t.cos_lat[(size_t)j] = std::cos(lat);
        t.sin_lat[(size_t)j] = std::sin(lat);
        /* The row's OWN place: this grid's column 0, by the same function a pixel uses. */
        double ld = 0.0, ad = 0.0;
        equirectLonLatDeg(equirectDirAt(t, 0, j), ld, ad);
        t.lat_deg[(size_t)j] = ad;
    }
    return t;
}

/* The TILEABLE layout at the top of this file, spelled once (#825): pixel CENTRES, with v
   scaled by H, so row 0 sits a half-row south of the north pole and no row is a pole. That is
   the layout the cloud coverage deck is baked on, and its UV has to keep matching
   cloudShader.vs, so the parametrisation belongs here and not in the bake.
   A separate TYPE rather than a flag on EquirectTrig: the two layouts answer DIFFERENT
   directions for the same (i, j) -- a poles-on-rows row 0 is the pole, a centres row 0 is not
   -- and handing the deck the wrong table would slide it half a pixel with nothing to see in
   the log. Making the caller name the layout is the cheapest way to make that impossible.
   FLOAT because the bake and the shader are float (the coverage map itself is R8, so the tables
   are not the last rounding anyway): double tables would add a second rounding between the
   noise and what the deck draws. */
struct EquirectTrigCentres {
    std::vector<float> cos_lon, sin_lon;   // [W], the column's centre
};

/* The centre formulas themselves, spelled once -- this is what the tileable grid IS. */
inline float equirectCentreLon(int i, int W) {
    assert(W >= 1 && i >= 0 && i < W);
    return ((float)i + 0.5f) / (float)W * 2.0f * (float)std::numbers::pi;
}

inline float equirectCentreLat(int j, int H) {
    assert(H >= 1 && j >= 0 && j < H);
    return (0.5f - ((float)j + 0.5f) / (float)H) * (float)std::numbers::pi;
}

/* The column trig, once per COLUMN. There is deliberately NO per-row table, and that is a
   measurement rather than an oversight (tmp/825_bench4.cpp, project flags; the ranges are what
   four reps spread over, so read them as a few percent, not as digits). At the shipped
   kCloudBakeW x kCloudBakeH deck:
     per-pixel lon trig (what the bake did)        213-226 ms
     this column table, row trig computed per row  185-193 ms   <- shipped, ~13% under
   A per-row lat table on top of that is never faster: 184-196 ms in one build of that arm and
   213-237 ms in another (the arm swings with codegen; the conclusion does not). Kept in
   registers the row's cos/sin are free; re-read from memory per row they cost exactly what the
   saved sincos bought. Asserts cost nothing here -- the same binary built -DNDEBUG lands in the
   same band, and reading the tables through the accessor measures the same as raw pointer
   indexing.
   What the ~13% must not be confused with: with cloudCover removed the whole loop runs in under
   2 ms, so the FBM noise in cloudCover is the ~200 ms half of this bake. That is the part worth
   attacking next, and the bake's own bake_ms= print (terrain.h) is where to watch it. */
inline EquirectTrigCentres equirectTrigCentres(int W) {
    assert(W >= 1);
    EquirectTrigCentres t;
    t.cos_lon.resize((size_t)W);
    t.sin_lon.resize((size_t)W);
    for(int i = 0; i < W; i++) {
        const float lon = equirectCentreLon(i, W);
        t.cos_lon[(size_t)i] = std::cos(lon);
        t.sin_lon[(size_t)i] = std::sin(lon);
    }
    return t;
}

/* equirectDir(lon, lat) at a centre-sampled column, with the ROW's cos/sin latitude handed in
   (see above: that is the shape the bake measures fast). Bit-for-bit equirectDir at the same
   centre, pinned in tests/test_surfmap.cpp -- not because the deck could show a one-ulp
   difference (the coverage map is R8, and the mip-filtered shell averages 255 steps anyway),
   but because a perf rewrite is exactly the change that silently re-baselines pixels, and a
   half-pixel slide against cloudShader.vs is a bug no log would show.
   Bounds asserted like the other overload: past-the-end vector indexing answers a plausible
   heap neighbour. The handed row trig is asserted unit-length for a narrower reason: cos_lat and
   sin_lat are two bare floats, so handing this the row's INDICES compiles clean and paints a
   non-unit direction; cl*cl+sl*sl is 1 to within float rounding, and catching a swapped pair is
   out of reach -- that one is what the pin is for. */
inline glm::vec3 equirectDirAt(const EquirectTrigCentres &t, int i,
                               float cos_lat, float sin_lat) {
    assert(i >= 0 && i < (int)t.cos_lon.size());
    assert(std::fabs(cos_lat * cos_lat + sin_lat * sin_lat - 1.0f) < 1e-4f);
    return glm::vec3(cos_lat * t.cos_lon[(size_t)i], sin_lat,
                     -cos_lat * t.sin_lon[(size_t)i]);
}
