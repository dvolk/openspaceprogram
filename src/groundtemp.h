#pragma once
/* groundtemp.h -- the ground's temperature under this star, on ONE knob (#724 rung C).

    T(lat, lon, t) = T0 * [ f + (1 - f) * 4 * cos_inc ] ^ (1/4)

   WHAT IT IS FOR. Until now a body had exactly ONE temperature: the authored
   Surface::temperature, the same at midnight and at noon, at the pole and under the star.
   This turns that one number into a field the star's light moves -- and moves the mean of
   not at all.

   WHY THE ANCHOR IS EXACT RATHER THAN A FUDGE TERM. Average T^4 over the sphere:
       <T^4> = T0^4 * ( f + (1 - f) * 4 * <cos_inc> ),      <cos_inc> = 1/4 exactly
   because the lit hemisphere's integral of cos over the sphere's whole 4 pi is pi/4 -- the
   same identity tests/test_solar.cpp pins as the 1/pi daily-mean cosine at the equator, and
   utils/ground_model.py pins by quadrature. So <T^4> = T0^4 for EVERY f: the surface
   radiates exactly the power the star delivers whatever the swing, and the swing lives
   entirely in the day/night shape. "The global mean stays T0" therefore means the RADIATION
   mean <T^4>^(1/4), not the arithmetic mean of T. Those two differ by the same factor that
   separates Earth's 255 K black-body from its 288 K ground, and the game already prints both
   figures side by side (#687), which is why this file anchors the first and leaves the gap
   alone.

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

/* [1] An UNAUTHORED even share: NOT 1.0. Venus states a diurnal range of about zero and
   Mercury states nothing at all; defaulting absence into 1.0 would print the same number for
   both and lose the difference between "the source says this ground does not swing" and "the
   source says nothing" (#726's sentinel, #727's dash law). */
inline constexpr double kEvenShareUnauthored = -1.0;
inline bool evenShareAuthored(double f) { return f >= 0.0; }

/* [1] The knob as a caller should use it when the file states no swing: a uniform surface.
   The BEHAVIOUR coincides with a stated f = 1 on purpose -- the air over the ground decides
   that, not the readout -- but anything PRINTING the knob must print an em-dash for the
   unauthored case, via evenShareAuthored(), not this default. */
inline double evenShareOrDefault(double f) { return evenShareAuthored(f) ? f : 1.0; }

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

/* [K] Which temperature this body's ground swings around, in the corpus's own order of
   authority; pass 0.0 for any of the three that is absent.
     1. a stated ground temperature (Surface::temperature);
     2. the datum of the air sitting on that ground -- after #625 that datum IS the published
        surface temperature, so reading it is the same statement under a different key and
        never a new number (which is how most of the shipped system is anchored);
     3. the black-body the starlight and this body's albedo imply (solar.h, #687), which is
        DERIVED and every readout that uses it has to say so (#722 keeps that visible).
   0.0 means none of the three, and the caller dashes. */
inline double groundAnchor_K(double stated_T, double air_base_T, double derived_bb) {
    if(stated_T > 0.0) { return stated_T; }
    if(air_base_T > 0.0) { return air_base_T; }
    return derived_bb > 0.0 ? derived_bb : 0.0;
}
