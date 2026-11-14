#pragma once
/* insolation.h -- ONE answer to "where is the star from this body, and what does its
   light do at this place" (#724 slice 0).

   WHY A MODULE. The same eight lines -- star root_pos minus body root_pos, normalise,
   transpose the body's orientation, multiply -- were written four times over:
   surfmap.cpp's sunDirRot (the map's terminator), system.cpp's subsolarLonAt (the solar
   day, frames advanced to a time), gameui.cpp's "Solar:" clock row, and -- in root axes
   rather than rotating ones -- tick.cpp's --solar-log elevation. Each was correct and
   each was a chance to disagree. That is #687's meta-law on the GEOMETRY axis: the
   subsolar point the calendar prints, the terminator the surface map bakes and the
   elevation the log reports are one fact about one vector.

   WHAT THIS IS. Geometry: a unit direction to the star, and the cosine/elevation at a
   zenith. The FLUX stays in solar.h (sunSample, solarFlux, incidentFlux), and the cosine
   and elevation here ARE solar.h's cosIncidence and sunElevation rather than new ones --
   a part's sun-facing skin and a named place must not hold two versions of one sun. The
   temperature field this is being built for (#724) multiplies the flux by the cosine.

   FRAMES, because this is where the bug would live. A direction's COMPONENTS are not
   frame-invariant -- that is exactly what starFromBody()'s transpose is for. What is
   invariant is the DOT PRODUCT, and only when both of its vectors are in the same frame.
   starFromBody() hands back the body's ROTATING frame, which is the frame equirect.h's
   (lon, lat) live in; that pairing is the reason the rotating-frame step lives here at
   all. A caller that already holds a root-axes pair (the solar log, which has a hull
   position and a SunSample) may fill the struct itself and stay in root axes -- but both
   vectors must move together, never one.

   WHAT THIS IS *NOT*, said out loud so nobody re-files it as a bug.
    * NO atmosphere, NO extinction, NO eclipse. solar.h owns that law and its absences; a
      top-of-atmosphere beam is what the cosine here multiplies.
    * NO temperature. A cosine is not a ground temperature. The surface field (roadmap
      rung C, #724) is the next step and will call this.
    * NOT the renderer's light vector. terrain.h:LightDirFrom answers star -> object in the
      render frame's axes for a shader: opposite sign, different frame, different
      degenerate fallback. It consumes the same offset; routing it through here would put a
      physics header inside a draw call for no gain, so it stays put and this comment
      records that the remaining duplication is known and bounded. */

#include <cassert>
#include <cmath>

#include <glm/glm.hpp>

#include "equirect.h"   // the (lon, lat) <-> direction convention
#include "solar.h"      // kSunMinDistance_m, cosIncidence, sunElevation

/* [m] A zenith shorter than this is not a horizon. A hull at a body's exact centre (a
   fixture, never a flight) has no local vertical to measure the star against, and dividing
   noise by noise would answer with a confident direction. Named rather than written at the
   test and the call (#707), and deliberately NOT kSunMinDistance_m: that one is "how far
   may I be from a star", this one is "how short may a direction be", two questions that
   happen to want similar numbers and would drift apart the moment one of them moves. */
inline constexpr double kNoHorizon_m = 1e-9;

/* Where the star is, as seen from a body's centre: a unit direction, and whether there is
   a direction to name at all. The distance is NOT carried here -- solar.h's SunSample
   already reports it for the same pair of points, and a second copy of that number is a
   second spelling of one fact (#636). */
struct StarFromBody {
    glm::dvec3 to_sun = glm::dvec3(0.0);   // unit; same frame as the caller's zenith
    bool valid = false;
};

/* The frame arithmetic in one place: root positions plus the body frame's root_orient ->
   the unit direction from the body's CENTRE to the star, in the body's ROTATING frame
   (i.e. comparable with equirectDir(lon, lat)). `body_root_orient` is
   body->frame->getRotFrame()->root_orient, which maps rotating axes -> root axes, hence
   the transpose. Invalid when the body and the star are at one point -- the body IS the
   star, or a degenerate system -- and callers then dash rather than invent a direction.
   The floor is solar.h's kSunMinDistance_m on purpose: the flux and the direction must not
   disagree about what counts as a distance, which is #707's one-constant rule applied to a
   geometry axis. */
inline StarFromBody starFromBody(const glm::dvec3 &body_root,
                                 const glm::dvec3 &sun_root,
                                 const glm::dmat3 &body_root_orient) {
    StarFromBody sv;
    const glm::dvec3 d = sun_root - body_root;
    const double len = glm::length(d);
    if(len <= kSunMinDistance_m) { return sv; }
    sv.to_sun = glm::transpose(body_root_orient) * (d * (1.0 / len));
    sv.valid = true;
    return sv;
}

/* The place where the star stands at the zenith: the row the calendar prints, the hour
   the clock is built from, and the anchor a seasonal law is written against. `lat` is the
   declination, and while the star stays in the body's orbital plane it never leaves
   [-axial_tilt, +axial_tilt] -- the rails today are all in one plane, so that bound holds;
   give a rail an inclination and the bound becomes tilt + inclination, which is the day a
   season law written against this sentence needs revisiting. Requires a valid sv: the
   callers above branch on validity to decide what to dash, and answering (0, 0) here would
   report a real place -- the equator at the prime meridian -- for a body with no star. */
inline void subsolarLonLat(const glm::dvec3 &to_sun, double &lon, double &lat) {
    /* The reading, not the frame arithmetic: a direction is a bearing, so it gets
       normalised here rather than at every caller (surfmap.h keeps its own sun vector
       unit by construction, but a caller that hand-filled the struct may not). */
    const double l2 = glm::dot(to_sun, to_sun);
    assert(l2 > 0.0);   // a zero direction has no bearing; the caller dashes instead
    glm::dvec3 d = to_sun;
    d *= 1.0 / std::sqrt(l2);
    equirectLonLat(d, lon, lat);
    assert(std::isfinite(lon) && std::isfinite(lat));
}

inline void subsolarLonLat(const StarFromBody &sv, double &lon, double &lat) {
    assert(sv.valid);
    subsolarLonLat(sv.to_sun, lon, lat);
}

/* What the star's light does at one zenith: the cosine a flat plate collects and the
   elevation a radiometer reports. `zenith` is the local vertical in the SAME frame as
   sv.to_sun -- equirectDir(lon, lat) for a named place, a hull offset for a ship, and
   either may be unnormalised because this normalises once. cos_inc is solar.h's floor (a
   night side collects nothing, and a negative cosine must not read as light coming from
   below), while elev keeps its sign so a readout can tell dusk from night. valid is false
   when there is no star to point at or no zenith to measure against; the caller then dashes
   both columns (#727's law). "Is it day" is not a field here: cos_inc > 0 says it, and a
   second spelling of the same question is one that can disagree. */
struct SkyAtZenith {
    double cos_inc = 0.0;    // [1]
    double elev_rad = 0.0;   // [rad] + overhead, - below the horizon
    bool valid = false;
};

inline SkyAtZenith skyAtZenith(const StarFromBody &sv, const glm::dvec3 &zenith) {
    SkyAtZenith sk;
    if(!sv.valid) { return sk; }
    const double len = glm::length(zenith);
    if(len <= kNoHorizon_m) { return sk; }   // no horizon: invalid, not asin(0)
    const glm::dvec3 up = zenith * (1.0 / len);
    sk.valid = true;
    sk.cos_inc = cosIncidence(up, sv.to_sun);
    sk.elev_rad = sunElevation(up, sv.to_sun);
    return sk;
}

/* The same question for a named place -- the entry point the surface temperature field
   (#724) is built on, which is why it exists before any caller in src/ does. Note
   equirect.h's argument order: lon first. Nothing in src/ calls it today: the sweep builds its
   direction from pixel indices through equirectDirAt rather than from a place name.
   tests/test_solar.cpp does call it, so the wrapper is exercised rather than dead weight. */
inline SkyAtZenith skyAtPlace(const StarFromBody &sv, double lon, double lat) {
    return skyAtZenith(sv, equirectDir(lon, lat));
}

/* [s since local midnight] What a clock reads at the body's prime meridian, from the
   subsolar longitude: that point is local noon, and east-positive longitude (#200) means
   every degree east of it is later. Kept geometric on purpose (#202): this is the subsolar
   point read as hours, not a mean-time table, so it is exactly the quantity the terminator
   and the solar log agree with.
   The FACE IS 24 HOURS FOR EVERY BODY -- the 15 deg/hour below is 360/24, the dial's own
   graduation, not a rotation rate. On Earth that graduation matches the day; on Kerbin
   (5:59:59) one dial hour is ~15 minutes of sim time and a dial minute is ~15 sim seconds,
   so noon still sits at 12:00 and local midnight at 00:00 whatever the world's spin. That
   is a display convention with a pinned case (e2e ui/741), not physics: nothing in the
   simulation ticks in these hours. The clamp at the end of the day is the "23:59:59 rather
   than 24:00:00" of a display, not a law. */
inline int solarSecOfDay(double subsolar_lon) {
    double sod = 12.0 - glm::degrees(subsolar_lon) / 15.0;
    while(sod < 0.0) { sod += 24.0; }
    while(sod >= 24.0) { sod -= 24.0; }
    int sec = static_cast<int>(std::lround(sod * 3600.0));
    if(sec >= 86400) { sec = 86399; }
    return sec;
}
