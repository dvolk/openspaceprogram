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

   WHAT THIS IS. Geometry: a unit direction to the star, the distance, and the
   cosine/elevation at a zenith. The FLUX stays in solar.h (sunSample, solarFlux,
   incidentFlux) -- one flux with many buyers -- so this header takes it as an argument
   rather than re-deriving it, and its cosine and elevation ARE solar.h's cosIncidence and
   sunElevation, not new ones.

   FRAMES, because this is where the bug would live. `StarFromBody::to_sun` is a direction
   and directions are frame-invariant under rotation, so what matters is that the caller's
   zenith is in the SAME frame. starFromBody() hands back the body's ROTATING frame, which
   is the frame equirect.h's (lon, lat) live in -- that is the combination a place needs,
   and the reason the rotating-frame step is in this header at all. A caller that already
   holds a root-axes sample (the solar log, which has a hull position and a SunSample) can
   fill the struct itself and stay in root axes.

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

#include <glm/glm.hpp>

#include "equirect.h"   // the (lon, lat) <-> direction convention
#include "solar.h"      // kSunMinDistance_m, cosIncidence, sunElevation

/* Where the star is, as seen from a point: a unit direction and the distance to it. */
struct StarFromBody {
    glm::dvec3 to_sun = glm::dvec3(0.0);   // unit; share a frame with the zenith
    double dist_m = 0.0;                   // [m] centre to centre
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
    sv.dist_m = glm::length(d);
    if(sv.dist_m <= kSunMinDistance_m) { return sv; }
    sv.to_sun = glm::transpose(body_root_orient) * (d * (1.0 / sv.dist_m));
    sv.valid = true;
    return sv;
}

/* The place where the star stands at the zenith. This is the row the calendar prints and
   the clock is built from, and it is the anchor a seasonal law is written against: lat is
   the declination, and |declination| never exceeds the body's axial tilt. */
inline void subsolarLonLat(const StarFromBody &sv, double &lon, double &lat) {
    lon = 0.0; lat = 0.0;
    if(!sv.valid) { return; }
    equirectLonLat(sv.to_sun, lon, lat);
}

/* What the star's light does at one zenith: the cosine a flat plate collects and the
   elevation a radiometer reports. `zenith` is the local vertical in the SAME frame as
   sv.to_sun -- equirectDir(lon, lat) for a named place, a rotated hull direction for a
   ship. cos_inc is solar.h's floor (a night side collects nothing, and a negative cosine
   must not read as light coming from below), while elev keeps its sign so a readout can
   tell dusk from night. valid is false when there is no star to point at or no zenith to
   measure against; the caller then dashes both columns (#727's law). */
struct SkyAtZenith {
    double cos_inc = 0.0;    // [1]
    double elev_rad = 0.0;   // [rad] + overhead, - below the horizon
    bool lit = false;        // above the horizon at all
    bool valid = false;
};

inline SkyAtZenith skyAtZenith(const StarFromBody &sv, const glm::dvec3 &zenith) {
    SkyAtZenith sk;
    if(!sv.valid) { return sk; }
    const double len = glm::length(zenith);
    if(len <= 0.0) { return sk; }   // no horizon: stay invalid rather than asin(0)
    const glm::dvec3 up = zenith * (1.0 / len);
    sk.valid = true;
    sk.lit = glm::dot(up, sv.to_sun) > 0.0;
    sk.cos_inc = cosIncidence(up, sv.to_sun);
    sk.elev_rad = sunElevation(up, sv.to_sun);
    return sk;
}

/* The same question for a named place, which is the form a surface temperature field
   wants. Note equirect.h's argument order: lon first. */
inline SkyAtZenith skyAtPlace(const StarFromBody &sv, double lon, double lat) {
    return skyAtZenith(sv, equirectDir(lon, lat));
}
