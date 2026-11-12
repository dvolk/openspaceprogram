#pragma once
/* groundtemp.h -- the ground's temperature under this star, on ONE knob (#724 rung C).

    T(lat, lon, t) = T0 * [ f + (1 - f) * 4 * cos_inc ] ^ (1/4)

   WHAT IT IS FOR. Until now a body had exactly ONE temperature: the authored
   Surface::temperature, the same at midnight and at noon, at the pole and under the star.
   This turns that one number into a field the star's light moves -- and moves the mean of
   not at all.

   WHY THE ANCHOR IS EXACT RATHER THAN A FUDGE TERM. Average T^4 over the sphere:
       <T^4> = T0^4 * ( f + (1 - f) * 4 * <cos_inc> ),      <cos_inc> = 1/4 exactly
   because the lit hemisphere's integral of cos over solid angle is pi, and the sphere's
   whole solid angle is 4 pi, so the mean is pi / 4 pi = 1/4 -- the same identity
   tests/test_solar.cpp pins as the 1/pi daily-mean cosine at the equator, and
   utils/ground_model.py pins by quadrature. So <T^4> = T0^4 for EVERY f: the surface
   radiates exactly the power the star delivers whatever the swing, and the swing lives
   entirely in the day/night shape.

   WHICH MEAN IS ANCHORED, because "the mean stays T0" is ambiguous and the two answers are
   not the same number. What is conserved is the RADIATION mean, <T^4>^(1/4). The arithmetic
   mean of T is not conserved -- measured at the knobs this corpus authors it sits 0.07 %
   below (Earth, f = 0.932: 0.2 K), 26 % below at the Moon's f = 0.014 (0.738 T0), and 43 %
   below at f = 0. That mean is AREA-weighted, which is the same distinction the surface map
   makes a point of: averaging an equirectangular grid's pixels without the cos(lat) weight
   puts it a further 4 % lower (30 % at the Moon's f). Do NOT read either as the gap the
   dossier prints between a body's airless
   black-body and its authored ground (Earth's 255 K against its 288 K, 33 K): that one is a
   POWER imbalance this model does not have -- the unmodelled greenhouse (#728, #738). This
   swing neither explains it nor closes it, which is why this file anchors the radiation mean
   and leaves that gap exactly where it was.

   THE KNOB f, [1]: the EVEN SHARE -- what fraction of the surface's own emission is spread
   evenly instead of falling where the star lights it. f = 1 is a uniform surface, which is
   what the RSS sheet STATES for Venus's ground ("Diurnal temperature range: ~0"); f = 0 is
   local radiative balance, an airless rock: the subsolar point reaches 4^(1/4) T0 and the
   anti-solar point radiates away. The two ends are not free dials -- the anchor forces
   (T_day/T0)^4 = 4 - 3 (T_night/T0)^4 -- which is what makes a corpus that states both ends
   of a swing a TEST of this model rather than a second knob to turn. utils/ground_model.py
   does that fitting and refuses the pairs that refuse to fit (Mars's lander-site row among
   them), and prints the residuals either way.

   THE LAW IS SPELLED TWICE, ON PURPOSE. utils/ground_model.py holds the same law for the
   generators that author f, and tests/test_groundtemp.cpp pins this header against the
   numbers that module produced -- the atmo.h / atmo_units.py arrangement, which is the only
   way two spellings of one law can be caught drifting (#636).

   WHAT THIS IS *NOT*, said out loud so nobody files it as a bug.
    * NOT an air temperature. What a hull flies through is still atmo.h's datum plus lapse
      profile; making the air follow the ground it sits on is the next rung, not this one.
    * NOT thermal inertia, regolith, or heat flow. An airless body's night end goes toward 0 K
      here because a bare surface in shadow has nothing in this model to keep it warm; the
      corpus's Moon row says 95 K, and the gap IS the missing inertia. The authored f damps
      that empirically rather than explaining it.
    * NOT a greenhouse. The unmodelled gap between an air's authored datum and the black-body
      its sunlight implies (#738's Laythe question) is untouched: this swings around the
      authored number, so it cannot close it, hide it, or make it worse.
    * NO eclipse, NO extinction, NO albedo physics of its own: cos_inc comes from solar.h's
      floored cosine and the albedo only enters through the anchor when one is derived. */

#include <algorithm>
#include <cassert>
#include <cmath>
#include <string>

#include "authored.h"   // kEvenShareUnauthored, kThermalUnauthored + the authored?() pairs

/* Absence for the knob is spelled in src/authored.h (kEvenShareUnauthored,
   evenShareAuthored) beside the albedo and temperature sentinels, because the three share one
   law and each has its own reason: Venus states a diurnal range of about zero while Mercury
   states NOTHING about its swing, and any default is a reading the file does not have -- "no
   swing" prints a constant hot ground under a midnight sun (least defensible exactly where the
   body is airless), "bare rock" invents a swing no source stated. So there is no default here:
   an unpriced knob DASHES the ground, and the swing is a claim only where a source's night/day
   pair priced it (#727's law, #726's state, #729's sentinel). */

/* [K] The surface temperature at one place. `cos_inc` is solar.h's floored cosine of the
   star's elevation (so [0,1]: 1 at the subsolar point, 0 anywhere in night), and
   `even_share` must be authored. Passing an anchor of 0 is a caller bug, not a cold world:
   every anchor path (groundtemp's own groundAnchor_K) says "dash" instead. */
inline double groundTemperature(double anchor_K, double even_share, double cos_inc) {
    assert(anchor_K > 0.0 && std::isfinite(anchor_K));
    assert(evenShareAuthored(even_share) && even_share <= 1.0);
    assert(cos_inc >= 0.0 && cos_inc <= 1.0 + 1e-9);
    const double c = std::min(1.0, std::max(0.0, cos_inc));
    const double shape = even_share + (1.0 - even_share) * 4.0 * c;
    return anchor_K * std::pow(shape, 0.25);
}

/* [K] The air at one altitude when the air nearest the ground is allowed to follow the
   ground it sits on (#724 rung D):

       T_air(alt) = T_profile(alt) + k * w(alt) * (T_ground - T_profile(alt))

   `k` is the body's authored follow (AtmosphereParams::air_follow, [0,1]); `w` is the
   WEIGHT this altitude gets, which the caller takes as the air's own pressure fraction
   p(alt)/p(0) -- "the air follows the ground where the air is thick", one fewer invented
   number than naming a boundary-layer depth, and it lands at 1 at the datum, ~0.37 one scale
   height up, and 0 above the profile's top because that is what the column itself says.

   WHAT THIS IS NOT. It is not an energy balance: nothing here conserves heat, moves heat
   between layers, or has a time constant (a hull at noon feels a noon ground immediately --
   the ground model has no thermal inertia either, #728 is parked on exactly that question).
   It is one authored number on an authored profile, and the reason it is allowed to exist at
   all is that the corpus already states both ends: the air's datum and the ground's swing.
   `T_profile` is the profile's own answer and `T_ground` comes from groundTemperature() off an
   ANCHORED reading -- never from this function's output, which is the loop #724's review
   flagged. */
inline double followAirTemperature(double profile_T, double ground_T, double k, double w) {
    assert(followAuthored(k) && k <= 1.0);
    assert(w >= 0.0 && w <= 1.0);
    assert(profile_T > 0.0 && ground_T > 0.0);
    return profile_T + k * w * (ground_T - profile_T);
}

/* [1] The weight an altitude gets in the law above: the air's own pressure fraction. Takes
   0.0 for "no air here" (including above the profile's top) so the caller can pass whatever
   drag.h:airPressure answers and get "no coupling" out of a vacuum for free. */
inline double followWeight(double p_at, double p_sea_level) {
    if(!(p_sea_level > 0.0) || !(p_at >= 0.0)) { return 0.0; }   // fail closed, drag.h's doctrine
    const double w = p_at / p_sea_level;
    return w < 0.0 ? 0.0 : (w > 1.0 ? 1.0 : w);
}

/* [K] The air at one hull, composed: the profile moved toward the ground under the hull.
   This is the entry point callers should use, because every "nothing to couple" case answers
   the PROFILE rather than asserting: no knob, no weight (vacuum, or above the top), no
   authored swing, no anchor -- and a ground that answers 0.0, which is not a bug but the bare
   rock's own night end (even_share 0 at midnight, pinned in tests/test_groundtemp.cpp). That
   last one is the reason the composition lives here instead of at the call site: a
   loader-legal pair of knobs (even_share 0.0 and air_follow 0.5) would otherwise hand
   followAirTemperature's assert a 0 K ground and abort the game over night. */
inline double coupledAirTemperature(double profile_T, double anchor_K, double even_share,
                                    double follow, double w, double cos_inc) {
    if(!(profile_T > 0.0) || !followActive(follow) || !(w > 0.0)
       || !evenShareAuthored(even_share) || !(anchor_K > 0.0)) { return profile_T; }
    const double ground_T = groundTemperature(anchor_K, even_share, cos_inc);
    if(!(ground_T > 0.0)) { return profile_T; }
    return followAirTemperature(profile_T, ground_T, follow, w);
}

/* [K] Which temperature this body's ground swings around, in the corpus's own order of
   authority; pass the field's own sentinel for any rung that is absent (kThermalUnauthored for
   a stated ground, 0.0 for a derived black-body), so a blank never reads as 0 K.
     1. a stated ground temperature (Surface::temperature);
     2. the datum of the air sitting on that ground -- after #625 that datum IS the published
        surface temperature, so reading it is the same statement under a different key and
        never a new number (which is how most of the shipped system is anchored);
     3. the black-body the starlight and this body's albedo imply (solar.h, #687), which is
        DERIVED and every readout that uses it has to say so (#722 keeps that visible).
   0.0 means none of the three, and the caller dashes. */
/* WHICH rung answers, so a readout can name its source without restating the order (#636:
   the dossier, the surface map's legend and the generator each spelled the if-chain, and a
   fourth spelling would eventually disagree with the third). The rank IS the order below. */
enum GroundAnchorRung {
    kAnchorNone = 0,      // nothing prices this world's ground
    kAnchorStated = 1,    // a source states a surface temperature
    kAnchorAirDatum = 2,  // the datum of the air sitting on that ground (#625's column)
    kAnchorDerived = 3    // DERIVED from sunlight + albedo: an estimate, and says so (#722)
};
inline GroundAnchorRung groundAnchorRung(double stated_T, double air_base_T,
                                         double derived_bb) {
    if(thermalAuthored(stated_T)) { return kAnchorStated; }
    if(air_base_T > 0.0) { return kAnchorAirDatum; }
    if(derived_bb > 0.0) { return kAnchorDerived; }
    return kAnchorNone;
}

// [K] The anchor the rung picks, or 0.0 when the rung is "nothing" -- and the caller dashes.
inline double groundAnchor_K(double stated_T, double air_base_T, double derived_bb) {
    switch(groundAnchorRung(stated_T, air_base_T, derived_bb)) {
        case kAnchorStated:   return stated_T;
        case kAnchorAirDatum:  return air_base_T;
        case kAnchorDerived:   return derived_bb;
        default:               return 0.0;
    }
}

/* A readout's source word. "DERIVED" is capitalised on purpose: #722's rule is that a
   computed figure announces itself at the readout, so it must not read like a measurement. */
inline const char *groundAnchorRungName(GroundAnchorRung r) {
    switch(r) {
        case kAnchorStated:   return "stated ground temperature";
        case kAnchorAirDatum: return "the air's datum";
        case kAnchorDerived:  return "DERIVED: sun + albedo";
        default:               return "no anchor";
    }
}

/* [std::string] What a readout should CALL this body's anchor. The file's own provenance
   string wins when it carries one: the generators SOLVE an anchor when a page states a range
   and no mean -- the Moon's 95-390 K and Pluto's 24-38 K price both the anchor and the knob
   (utils/ground_model.py:pair_solve) -- and the result lands in the same key a stated average
   uses. Calling that "stated ground temperature" at the readout would credit the corpus with a
   number it never stated, which is #722's rule running in the other direction. Takes the
   string rather than a Surface so this header stays out of the loader's way (#636: one
   spelling of the order, here; the strings travel with the values that carry them). */
inline std::string groundAnchorSource(GroundAnchorRung r, const std::string &temperature_src) {
    return temperature_src.empty() ? std::string(groundAnchorRungName(r)) : temperature_src;
}
