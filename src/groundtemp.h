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
#include <numbers>
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
    /* The quarter root is two square roots, not a general pow (#802): this is the law's
       hottest call -- once per pixel of a map bake -- and the asserts above leave `shape` in
       [0, 4 - 3f + eps] (f = 0 at midnight is exactly 0, and cos_inc's 1e-9 tolerance rides
       along), so there is no domain case for pow to guard.
       NOT bit-for-bit pow, and it would be dishonest to imply otherwise: measured across the
       reachable (f, cos_inc) domain, ~12 % of arguments land 1 ulp apart. That is ~6e-14 K --
       it cannot move a ramp byte, a dossier row, or one of the 1e-6..1e-12 pins this header
       shares with utils/ground_model.py, which still writes ** 0.25. The two spellings of
       this law (#636) have never agreed to the bit and still do not: what is shared is the
       LAW. */
    return anchor_K * std::sqrt(std::sqrt(shape));
}

/* ---------------------------------------------------------------------------------------------
   #794: THE SAME LAW WITH THE SKIN ALLOWED TO LAG THE LIGHT.

   The law above is INSTANTANEOUS: the ground answers the star's cosine at the same instant, so
   the hottest place on a map is always due south/north of the subsolar point and the coldest is
   local midnight. Real surfaces peak in the afternoon and bottom out after sunrise -- which is
   what this lets the model say, for one authored number.

   THE EQUATION. Along a latitude, write the star's hour angle as `h` (longitude measured from
   the subsolar meridian, EAST positive, so h grows with time), and the forcing the instantaneous
   law uses as g(h) = max(a + b cos h, 0), with a = sin lat sin dec and b = cos lat cos dec.
   "EAST positive, growing with time" is a convention, not a fact about the maths: it holds here
   because this engine encodes a retrograde body as a tilt past pi/2 with a non-negative
   rot_ang_speed, so a surface point's longitude always increases. tests/test_groundtemp.cpp pins
   the SPATIAL half of it -- a + b cos h against the equirect directions -- but only the map wiring
   can pin the temporal half, and a flipped sign is invisible in a picture.
   The lagged skin is the periodic steady state of

       tau dy/dh + y = g(h),      tau in PHASE RADIANS (one rotation = 2 pi)

   which is the usual tau_time dT/dt = ... read against phase instead of seconds, i.e.
   tau_phase = tau_time * omega_solar. The ANSWER inside a row does not care how fast the body
   turns -- but the authored number is precisely the thing that does, so the same ground on a body
   spinning twice as fast has half the phase lag. A generator pricing tau from a stated afternoon
   peak therefore converts with that body's MEASURED mean solar day (system.h:measureSolarDay),
   never the sidereal rotation, and the substitution quietly assumes dh/dt is constant over the
   lag's memory -- true on a circular orbit, slightly false on an eccentric one, where the subsolar
   longitude rate drifts over the year.

   WHY THE ANCHOR STILL HOLDS -- and this is the part worth stating, because it looked like the
   lag would break the exactness that makes T0 an anchor rather than a fudge term. Average the
   equation over one rotation: the derivative term averages to zero (a periodic function has zero
   mean derivative), so <y> = <g> for EVERY tau. The forcing's sphere mean is still 1/4, so
   <T^4> = T0^4 exactly as before. The lag moves the swing around; it cannot change its size.
   tests/test_groundtemp.cpp pins this on the sphere, cos(lat)-weighted over a full rotation.

   THE CLOSED FORM (no time stepping, no sample table). With the lit arc |h| <= h0,
   h0 = acos(-a/b), and yp(h) = a + b(cos h + tau sin h)/(1 + tau^2) the particular solution
   (so tau yp' + yp = a + b cos h exactly):
       lit    y = yp(h) + C e^{-h/tau}
       night  y = N e^{-(h_n - h0)/tau},     h_n = phase since sunset, in [h0, 2 pi - h0]
   with kset = yp(h0) and crise = yp(-h0) -- the particular solution at SUNSET and at SUNRISE,
   tau(a*tau +/- b sin h0)/(1 + tau^2). NOTE the +/-: the two ends of the day arc are NOT
   symmetric once dec != 0, and the symmetric shortcut b*tau*sin(h0)/(1+tau^2) -- which is what
   one writes down first -- is exact only at equinox. That error hides at dec 0, shows up as a
   step at the terminator everywhere else, and puts heat where the model has none; the RK4
   cross-check in tests/test_groundtemp.cpp is what caught it.

   NUMERICS, and the trap that cost a rewrite. Continuity at both terminators fixes C:
       x = e^{-h0/tau},  z = e^{-(2 pi - h0)/tau},  u = e^{-(2 pi - 2 h0)/tau},  q = x*z
       C = (kset z - crise x) / (1 - q)      N = kset + C x
   and every exponential in the builder has a non-positive argument. The lit term does not:
   on the morning half h is negative, so e^{-h/tau} has a POSITIVE exponent reaching e^{+h0/tau},
   which overflows once h0/tau > ~709. Folding it as x * e^{-(h0 - h)/tau} to dodge that -- the
   obvious move, and my first attempt -- is simply a different function (x^2 e^{h/tau}); it breaks
   continuity at sunrise by 0.3-0.5 of a share, which is tens of kelvin on an airless body and it
   does not trip an assert. So the term is anchored at whichever terminator it is read from:
       hom_noon = C                                (its value at h = 0)
       hom_rise = (kset u - crise) / (1 - q)       (= C e^{+h0/tau}, never forming e^{+h0/tau})
   and each half of the day evaluates from its own anchor, again with non-positive exponents.
   Small tau then behaves rather than exploding: C -> 0, N -> kset, hom_rise -> -crise, and the
   row answers the forcing itself.

   WHAT THIS IS STILL *NOT*. It is a phase shift, not thermal inertia: it does not damp the
   swing's AMPLITUDE the way a regolith skin does (that stays the job of the authored even share,
   which is why the two trade against each other and a body that authors tau needs its f re-fit),
   and it is not heat flow, not a subsurface profile, not a weather system. It is one authored
   number on the skin the law already has, and the field it produces is still a radiative skin --
   sigma T^4 -- not a measured air temperature.

   Feeding this to groundTemperature() is deliberate: y is a unit-mass positive filter of g, and
   g <= max(a + b) = cos(lat - dec) <= 1, so y stays in [0,1] and lands in the same currency as
   cos_inc. Its parameter is still named cos_inc; a lagged value is not a cosine, and this is the
   sentence that says so.
   --------------------------------------------------------------------------------------------- */

/* One latitude's lagged answer, built once per latitude and evaluated per longitude (#802's
   hoists are what make this affordable on a map bake: everything that varies only with latitude
   -- a, b, the terminator, the two exponential constants -- is resolved in the builder). */
struct GroundLagRow {
    enum Mode : unsigned char {
        kInstant = 0,     // tau absent or 0: the unlagged forcing (today's ground)
        kDayNight = 1,    // an arc of day and an arc of night
        kPolarDay = 2,    // lit for the whole rotation
        kPolarNight = 3   // dark for the whole rotation
    };
    Mode mode = kInstant;
    double tau = 0.0;      // [phase rad]
    double a = 0.0;        // forcing g(h) = a + b cos h
    double b = 0.0;
    double h0 = 0.0;
    /* [rad] Half the day arc: |h| <= h0 is lit. Filled in EVERY mode -- pi for a row lit all
       rotation, 0 for one dark all rotation, acos(-a/b) in between -- because the follow-up wants
       to read it as "where this latitude's sun rises and sets", and a field that means something
       in only one of four modes is a bug waiting to be drawn. */
    double inv_den = 1.0;  // 1/(1 + tau^2), hoisted: at() runs once per map column
    double hom_noon = 0.0; // the lit arc's homogeneous term at h = 0
    double hom_rise = 0.0; // ... at sunrise (h = -h0), so e^{+h0/tau} is never formed
    double night_at_sunset = 0.0;  // the night decay's value AT sunset (continuous with the day)

    /* [1] The lagged forcing at hour angle `h` [rad, any real -- wrapped here]. */
    double at(double h) const {
        /* A row is not meant to be assembled by hand, but a sweep will copy these fields between
           structs, and a kDayNight row with tau 0 answers -nan rather than failing. */
        assert(mode != kDayNight || (tau > 0.0 && std::isfinite(tau)));
        const double w = std::remainder(h, 2.0 * std::numbers::pi);   // [-pi, pi]
        if(mode == kPolarNight) { return 0.0; }
        if(mode == kInstant)    { return std::max(a + b * std::cos(w), 0.0); }
        if(mode == kPolarDay) {
            // The forcing never breaks, so this is the smooth answer and its peak sits exactly
            // atan(tau) of phase east of noon -- the only case where atan(omega tau) is the whole
            // story, which is why tests/test_groundtemp.cpp pins it there and nowhere else.
            return a + b * (std::cos(w) + tau * std::sin(w)) * inv_den;
        }
        // The particular solution is only needed where there IS a day; a night column pays for
        // two transcendentals and a divide otherwise, and this bake calls it per column.
        if(w >= 0.0 && w <= h0) {
            const double yp = a + b * (std::cos(w) + tau * std::sin(w)) * inv_den;
            return yp + hom_noon * std::exp(-w / tau);
        }
        if(w < 0.0 && w >= -h0) {
            const double yp = a + b * (std::cos(w) + tau * std::sin(w)) * inv_den;
            return yp + hom_rise * std::exp(-(w + h0) / tau);
        }
        const double hn = (w > 0.0) ? w : w + 2.0 * std::numbers::pi;  // since sunset
        return night_at_sunset * std::exp(-(hn - h0) / tau);
    }
};

/* [1] Build one latitude's row. `lat` and `dec` are RADIANS within [-pi/2, pi/2] (dec = the
   star's declination in the body's own frame, the same quantity the map's equirect grid already
   knows; a retrograde world's tilt past pi/2 is folded by the latitude that geometry returns, so
   the bound holds for every shipped body). `tau` is the authored lag in phase radians or
   kLagTauUnauthored; either non-positive answer takes the kInstant branch, which is the unlagged
   forcing and therefore cannot change a picture. */
inline GroundLagRow groundLagRow(double lat, double dec, double tau) {
    assert(std::isfinite(lat) && std::isfinite(dec));
    /* Handing DEGREES to a radians function is the exact mistake this knob exists to catch, and
       unasserted it answers a plausible-looking row: groundLagRow(45.0, -23.44, .42) is quietly a
       "polar day". Same doctrine as groundTemperature() asserting its cosine. */
    assert(std::fabs(lat) <= 0.5 * std::numbers::pi + 1e-9
           && std::fabs(dec) <= 0.5 * std::numbers::pi + 1e-9);
    /* The sentinel is a legal argument on purpose: a bake walks the whole system and passes the
       field it finds, and "nobody authored a lag" answers the instantaneous row rather than
       costing every caller a pre-check. Any OTHER negative -- or a NaN -- is a bug upstream, and
       the ceiling is the same kind of guard: measured, the row starts returning NEGATIVE shares
       near tau 1e16 and answers NaN past ~1e18, so a caller that lands there has a unit bug, not
       a slowly-turning world. */
    assert(tau == kLagTauUnauthored || (tau >= 0.0 && tau <= 1e6));
    GroundLagRow r;
    r.a = std::sin(lat) * std::sin(dec);
    r.b = std::cos(lat) * std::cos(dec);
    r.h0 = (r.a >= r.b) ? std::numbers::pi
         : (r.a <= -r.b) ? 0.0 : std::acos(-r.a / r.b);
    if(!lagTauActive(tau)) { return r; }
    r.tau = tau;
    r.inv_den = 1.0 / (1.0 + tau * tau);
    if(r.a >= r.b) { r.mode = GroundLagRow::kPolarDay; return r; }
    if(r.a <= -r.b) { r.mode = GroundLagRow::kPolarNight; return r; }
    r.mode = GroundLagRow::kDayNight;
    const double kset = tau * (r.a * tau + r.b * std::sin(r.h0)) * r.inv_den;   // yp at sunset
    const double crise = tau * (r.a * tau - r.b * std::sin(r.h0)) * r.inv_den;  // ... at sunrise
    const double x = std::exp(-r.h0 / tau);
    const double z = std::exp(-(2.0 * std::numbers::pi - r.h0) / tau);
    const double u = std::exp(-2.0 * (std::numbers::pi - r.h0) / tau);
    const double both = 1.0 - x * z;
    assert(both > 0.0);     // x*z = e^{-2 pi/tau}; see the tau ceiling on the argument above
    r.hom_noon = (kset * z - crise * x) / both;
    r.hom_rise = (kset * u - crise) / both;
    r.night_at_sunset = kset + r.hom_noon * x;
    assert(std::isfinite(r.hom_noon) && std::isfinite(r.hom_rise)
           && std::isfinite(r.night_at_sunset));
    return r;
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
