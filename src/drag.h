#pragma once

// drag.h -- the atmospheric-drag law as pure math (glm + <cmath> only), so tests/
// can pin it without the render chain.
//
// The air co-rotates with the planet: it is at rest in the body's ROTATING frame,
// so there the ship's frame velocity IS the air-relative velocity. In any other
// frame Vehicle::airRelativeVel subtracts the co-rotation term (#60/#97).

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>  // controlSurfaceVisualRot

#include "atmo.h"       // AtmoLayer: a body's authored altitude/temperature levels
#include "constants.h"  // kRhoFloor

/* The physical half of a body's atmosphere (render half: AtmosphereParams,
   terragen.h). Both zero means "no drag". */
struct DragAtmosphere {
    double sea_level_density = 0.0;  // kg/m^3 at alt 0; 0 = no drag
    double scale_height = 0.0;       // [m]; density /e-fold altitude
    /* The hard top [m above sea level]: at or above it the air is vacuum.
       0 = no cutoff (the exponential tail runs down to kRhoFloor). */
    double height = 0.0;
    /* What the thermodynamics below needs (terragen.h:AtmosphereParams has the
       authored halves; vehicle.cpp:dragAtm is the one place they are copied).
       gravity has NO default world: an atmosphere that does not say what planet it
       is on reads 0 K / 0 Pa / 0 m/s, which is the LOUD answer -- inheriting Earth's
       9.81 would be the plausible wrong one on Eve or Jool. The gas defaults are a
       different matter: they say what the air is made of, not where it is. */
    double gravity = 0.0;              // [m/s^2] at the body's sea-level DATUM
    /* The sea-level DATUM radius [m] this air sits on -- terrain.h:seaLevelRadius(),
       the same sphere airDensity's `alt` is measured from. It is not decoration:
       the column is weighed in the field the hull actually falls in,
       g(z) = gravity radius^2/(radius+z)^2, and on a KSP-sized world that is the
       difference between 3.6e-6 and 1.4e-5 kg/m^3 at Kerbin's air top (#683).
       0 = integrate a UNIFORM field: the honest reading of a hand-built atmosphere
       that does not say what planet it is on, and what every pre-#683 fixture in
       tests/ asks for. A body loaded from a system file always carries it, and
       test_scenarios checks that dragAtm() copies it for exactly the same reason it
       checks g and the composition -- a lost copy reads plausible, not broken. */
    double radius = 0.0;
    double molar_mass = kAirMolarMass; // [kg/mol]
    double gamma = kAirGamma;          // cp/cv
    /* Kilograms of oxidizer per kilogram of air, off the body's authored composition
       (atmo.h:resolveMix, #630). It sits here rather than beside the gate as a loose
       argument because it is a fact about THIS air, the same fact molar_mass and gamma
       are: a caller that reads the density under a hull already holds the profile it
       came from, and fetching the share off TerrainBody by hand lets the two halves
       describe different worlds (#637). Defaults to dry air for the same reason the
       two gas constants above do -- it says what the air is made of, not where it is;
       a body loaded from a file gets whatever its mix resolves to. */
    double oxidizer_share = dryAirOxidizerShare();
    /* The layer table (#552 A4), when the body has one: a pointer INTO the body's
       own AtmosphereParams, so this stays a cheap copy -- the aero pass builds one
       of these per hull per substep and a vector here would allocate on every one.
       Non-null means the profile IS the levels and not scale_height: the loader
       resolves each level's base pressure and refuses a body that authors both. */
    const AtmoLayer *layers = nullptr;
    int layer_count = 0;
    DragAtmosphere() {}
    DragAtmosphere(double rho0, double H, double top = 0.0)
        : sea_level_density(rho0), scale_height(H), height(top) {}
};

/* Which level `alt` falls in: the last one whose base_alt is at or below it, or -1
   for at/below the datum, where the profile stays FLAT (#245's floor, which the
   level form keeps rather than replaces -- the datum is level 0's base_alt by the
   loader's rule). A LINEAR scan on purpose: a layer table is a handful of levels
   (the real 1976 standard atmosphere uses seven to 85 km), the profile is read once
   per hull per substep (applyAeroForce samples it at the COM, not per part), and a
   binary search over seven entries cannot pay for its own branches. */
inline int atmoLevel(const DragAtmosphere &a, double alt) {
    if(a.layers == nullptr || a.layer_count <= 0) { return -1; }
    if(alt <= a.layers[0].base_alt) { return -1; }
    int i = 0;
    for(int k = 1; k < a.layer_count; ++k) {
        if(a.layers[k].base_alt > alt) { break; }
        i = k;
    }
    return i;
}


/* rho(alt) = sea_level_density · exp(−Z(alt) / scale_height), [kg/m^3], alt in m above
   the sea-level DATUM (terrain.h:seaLevelRadius(), i.e. radius + the authored
   sea_level -- not the surface, and not the bare radius: #412's fixture is what
   distinguishes them), where Z is the geopotential depth of that altitude in the
   body's own field (atmo.h:geopotentialDepth, #683) -- with no radius on the
   atmosphere, Z is just alt and this is the single exponential it always was.
   At or above `height` the air stops dead.
   At or below the datum the air is sea_level_density, FLAT. The exponential's base
   IS the datum, so a sub-datum point -- a hull skimming the sea, a dry basin below
   a sea_level that sits above the body's relief -- used to read as vacuum and lose
   drag, q and jet thrust together (#245). Sub-datum terrain is not exotic: relief is
   SIGNED about the radius (terragen.h:terrainRelief) and only a Flat-sea body clamps
   its floor, so Kerbin's sea beds sit under its datum and so do the valleys of dry
   air bodies -- Eve (amplitude 5000, 1.7 kg/m3), Mars, Titan. The noise's own floor
   is -amplitude/3 (the continents term bottoms at -0.7, the mountain term is >= 0,
   and the pair scales by amplitude/2.1), i.e. -1.7 km on Eve, -17 km on Titan.
   Not the exponential continued downward: 1.30 kg/m3 at -336 m already, and
   115 kg/m3 at -25 km -- the deepest sea would be the densest air in the game,
   without bound. What the air really does down there is a profile question, and a
   hull under the sea of a has_sea body arguably wants WATER rather than denser air
   (the game has none: no buoyancy, no splashdown, no drag law for it).
   A body with a LAYER TABLE (#552 A4, `layers`) answers this from the levels
   instead: T comes from the level it is in and rho from p / (Rs T) with p
   integrated hydrostatically. That is the profile a lapse rate makes, and it is
   what lets a body state a temperature AND its sea-level density at once -- one
   exponential cannot (see "THE THREE ARE LOCKED" below). The flat floor below the
   datum is the SAME law there, not an exception bolted on: below level 0's
   base_alt the game still hands out thick air rather than water (#618). */
inline double airDensity(const DragAtmosphere &a, double alt) {
    if(!(a.sea_level_density > 0.0)) { return 0.0; }
    if(a.layers != nullptr) {
        if(!(a.gravity > 0.0) || a.layer_count <= 0) { return 0.0; }
        if(a.height > 0.0 && alt >= a.height) { return 0.0; }
        const int i = atmoLevel(a, alt);
        if(i < 0) { return a.sea_level_density; }   // flat at/below the datum
        assert(a.molar_mass > 0.0);  // system.cpp refuses air with a bad molar mass
        const double Rs = kGasR / a.molar_mass;
        const double T = levelTemperature(a.layers[i], alt);
        if(!(T > 0.0)) { return 0.0; }
        const double p = levelPressure(a.layers[i], alt, a.gravity, Rs, a.radius);
        return p / (Rs * T);
    }
    if(!(a.scale_height > 0.0)) { return 0.0; }
    if(alt <= 0.0) { return a.sea_level_density; }
    if(a.height > 0.0 && alt >= a.height) { return 0.0; }
    return a.sea_level_density * std::exp(-geopotentialDepth(a.radius, alt)
                                          / a.scale_height);
}

/* THE AIR'S THERMODYNAMICS (#552 A2). One exponential IS an isothermal
   atmosphere -- H = R T / (M g) -- so the authored rho0 / scale_height plus the
   body's g already ASSERT a temperature, a pressure and a sound speed:
       T = g H M / R                   [K]   constant with altitude
       p = rho R T / M = rho g H       [Pa]  M cancels
       a = sqrt(gamma R T / M) = sqrt(gamma g H)  [m/s]   M cancels
   Pressure and sound speed need NOTHING authored -- the scale height alone sets
   them, which is why molar_mass is a composition question only for the TEMPERATURE
   line. Read that as content, not trivia: Laythe (H 5500, g 7.848, rho0 1.225) means
   150.4 K, 52.9 kPa, 246.0 m/s, so a hull at 300 m/s at sea level is already
   Mach 1.22 there; an Earth-shaped exponential (H 8500) means 290.8 K, 102.3 kPa,
   341.9 m/s -- within ~1% of USSA-1976, with no temperature authored anywhere.
   (Both use dry air's resolved gamma 1.4015 rather than the typed 1.4; the shipped
   bodies' gammas come off their authored mixes, #630, and a hand-built DragAtmosphere
   still defaults to constants.h's 1.4. Sound speed goes as sqrt(gamma), so it is 0.05%.)
   THE COLUMN IS WEIGHED IN THE BODY'S OWN FIELD (#683), which is the one input these
   formulas do not show: the hull is pulled by g radius^2/(radius+z)^2, so the exponent
   is the geopotential depth Z(alt), not alt. At the datum the read is unchanged -- H is
   defined there -- and only the aloft numbers move: on Earth by 0.3% at 11 km, at
   Kerbin's 85 km top by 4.6x (2.26 Pa of column weighed, 0.50 Pa uniform), across the
   shipped catalogue 1.04x (Jupiter's top) to 5.4x (Duna's). What stays GEOMETRIC on purpose is everything the rest of the game
   measures altitude in: the `alt >= height` vacuum check above, the science bands, the
   shell (bodylimits.h), the spawn beds and the atlas disk. Z is arithmetic inside the
   column, not a unit the game speaks.
   COMPOSITION IS NOT FREE EITHER, and since #630 it is a MIX authored per body: T is
   linear in M, so a hydrogen giant left at dry air's mix reads absurd -- Jool reads
   547 K on dry air where its authored H2/He/CH4 mix (M 0.002364) reads 44.6 K.
   p and a do not care,
   so this is a readout-truth question, not a flight-model one. The "isothermal"
   label on the readouts warns about the missing lapse rate; it does NOT excuse a
   wrong gas.
   THE THREE ARE LOCKED on ONE exponential: rho0, p0 and H cannot all be Earth's
   numbers at once. rho0 1.225 with p0 101.3 kPa wants T0 288 K, which wants H 8.43 km:
   against the H = 5.5 km Laythe ships, that exponential is 1.37x thicker at 5 km and
   3.54x at 20 km -- and the REAL atmosphere is 1.5x / 2.7x thicker there, because a
   lapse rate makes density fall faster than pressure. Having both means a LAPSE RATE,
   and that is what a LAYER TABLE is (#552 A4): a body that authors one gets a
   temperature that varies with altitude, a pressure integrated hydrostatically and a
   density that follows from the gas law, all three consistent with its authored
   sea_level_density. A body that does not (every KSP-derived world except Kerbin,
   which #686 gave the same table Earth wears) still gets the isothermal read of its
   authored scale height, and the atlas labels it
   "(isothermal)" rather than dressing it up as weather. Earth is the body that
   authors the real table, both because it is the only one with published numbers to
   copy and because it is the fixture that keeps the profile honest (test_drag pins it
   within 1% of the standard atmosphere from 0 to 85 km), and Kerbin has worn a
   converted copy of it since #686 -- so the Earth row above is
   the EXERCISE, not the shipped body: Earth now reads 288.1 K / 101.3 kPa / 340.5 m/s
   at its datum because it says so, not because H 8500 implies it.
   ALL THREE STOP AT `height`, layered or not. Above the authored air top the air is
   vacuum: rho and p have always said so, and so must T and a -- a hull at 120 km over
   Kerbin has no air to have a temperature IN, and a nonzero sound speed there is a
   Mach number divided by nothing (#543). An exponential with no authored `height`
   (height 0) has no such cutoff, which is what "the tail runs down to kRhoFloor" means.
   Below the datum airDensity() is FLAT (#245), so p is flat too: the column under
   the datum is not in hydrostatic equilibrium. Deliberate -- under a has_sea body
   the game gives thick air, not water, and has no buoyancy, splashdown or water
   drag anywhere (#618, closed as designed) -- and integrating the column downward
   would invent a pressure no law uses.
   Guards fail CLOSED in the !(x > 0.0) shape (src/angmath.h: every relational test
   against NaN is false, so `x <= 0.0` lets a NaN walk straight into the multiply and
   out the other side, and a NaN sound speed becomes a NaN Mach several steps later).
   Same shape as airDensity above. A g-less or H-less atmosphere therefore reads 0.0
   for all three, never a negative pressure or a NaN. `alt` is NOT guarded: it comes
   from a live position, and a NaN there is a broken hull, not a bad def. */
inline double airTemperature(const DragAtmosphere &a, double alt) {
    if(!(a.sea_level_density > 0.0) || !(a.gravity > 0.0)) { return 0.0; }
    assert(a.molar_mass > 0.0);
    if(a.layers != nullptr) {
        if(a.layer_count <= 0) { return 0.0; }
        if(a.height > 0.0 && alt >= a.height) { return 0.0; }
        const int i = atmoLevel(a, alt);
        // At/below the datum the profile is FLAT at the datum's own values, exactly
        // as airDensity() is flat at sea_level_density (#245). Extrapolating level 0's
        // lapse downward instead would make p = rho Rs T false at the boundary the
        // two functions share, and would put the densest air in the game under the
        // sea floor.
        if(i < 0) { return a.layers[0].base_T; }
        return levelTemperature(a.layers[i], alt);
    }
    if(!(a.scale_height > 0.0)) { return 0.0; }
    if(a.height > 0.0 && alt >= a.height) { return 0.0; }   // vacuum, layered or not
    return a.gravity * a.scale_height * a.molar_mass / kGasR;
}

/* [Pa], alt in m above the sea-level DATUM -- the same alt airDensity takes. */
inline double airPressure(const DragAtmosphere &a, double alt) {
    if(!(a.gravity > 0.0)) { return 0.0; }
    if(a.layers != nullptr) {
        /* p and rho come out of the SAME integration here (this is the branch where
           the law is p-first: p = hydrostatics, rho = p / (Rs T)), so the two
           cannot drift into describing different columns of air. */
        if(!(a.sea_level_density > 0.0) || a.layer_count <= 0) { return 0.0; }
        if(a.height > 0.0 && alt >= a.height) { return 0.0; }
        const int i = atmoLevel(a, alt);
        assert(a.molar_mass > 0.0);
        // Flat at the datum's own pressure at/below it, the same floor airDensity()
        // hands out. p0 is resolved at load to rho0 Rs T0, so rho Rs T == p holds
        // exactly across that boundary rather than only above it.
        if(i < 0) { return a.layers[0].base_p; }
        return levelPressure(a.layers[i], alt, a.gravity, kGasR / a.molar_mass,
                             a.radius);
    }
    if(!(a.scale_height > 0.0) || !(a.sea_level_density > 0.0)) { return 0.0; }
    /* Not the column -- the equation of state. p = rho Rs T with the T this profile
       implies (the line above), so it holds at every altitude whether the field is
       uniform or falling off: the falloff moves rho, and rho carries it (#683). */
    return airDensity(a, alt) * a.gravity * a.scale_height;
}

/* [m/s] -- the speed the Mach number is a ratio to, at `alt` (#543 divides by this,
   which is why it is asked for at the altitude it applies to and not once per body).
   An isothermal body's value is CONSTANT with altitude -- that is what isothermal
   means -- and a layer table is what makes it vary: on Earth it falls from 340.5 m/s
   at the datum to 295.1 m/s at 11 km, so the same 300 m/s airspeed is Mach 0.88 at
   sea level and Mach 1.02 up there. Above `height` it is 0 on either profile: there is
   no air left to carry sound, so a Mach number up there has nothing to be a ratio to. */
inline double airSoundSpeed(const DragAtmosphere &a, double alt) {
    if(!(a.sea_level_density > 0.0) || !(a.gravity > 0.0)) { return 0.0; }
    assert(a.gamma >= 1.0);  // system.cpp refuses an authored gamma below 1
    if(a.layers != nullptr) {
        const double T = airTemperature(a, alt);
        if(!(T > 0.0)) { return 0.0; }
        assert(a.molar_mass > 0.0);
        return std::sqrt(a.gamma * (kGasR / a.molar_mass) * T);
    }
    if(!(a.scale_height > 0.0)) { return 0.0; }
    if(a.height > 0.0 && alt >= a.height) { return 0.0; }   // vacuum, layered or not
    return std::sqrt(a.gamma * a.gravity * a.scale_height);
}

/* F = −v̂ · ½ · rho(alt) · cd · area · |v_rel|²   (area [m^2], v_rel [m/s]).
   Zero for any degenerate input so callers need no guards. */
inline glm::dvec3 dragForce(const DragAtmosphere &a, double cd, double area,
                            double alt, const glm::dvec3 &v_rel) {
    const double rho = airDensity(a, alt);
    const double v = glm::length(v_rel);
    if(rho <= 0.0 || v <= 0.0 || cd <= 0.0 || area <= 0.0) {
        return glm::dvec3(0.0);
    }
    return -glm::normalize(v_rel) * (0.5 * rho * cd * area * v * v);
}

/* The aerodynamic flow frame: the ship's air-relative velocity decomposed
   against its body axes (x right, y up, z nose).

   ALPHA IS THE CONVENTIONAL ANGLE OF ATTACK: positive when the NOSE sits above
   the flight path. That is what every downstream law assumes -- liftCurve()
   signs the lift by alpha, so the sign chosen here decides whether a pitched-up
   wing lifts or drives the airframe into the ground. The flow comes AT the nose
   from ahead, so v_rel points BACKWARD: a nose above the flow has v_rel leaning
   DOWN, i.e. dot(v_rel, up) < 0, hence the negated up term. BETA needs no such
   flip -- v_rel leaning toward +right already means the wind arrives from the
   ship's right, which IS the conventional positive sideslip. */
struct AeroFrame {
    double v = 0.0;         // speed |v_rel|
    double alpha = 0.0;     // angle of attack, rad; + = nose above flight path
    double beta = 0.0;      // sideslip, rad; + = flow from the ship's right
    bool valid = false;     // false when v=0 or the nose axis is degenerate
};

inline AeroFrame aeroFrame(const glm::dvec3 &v_rel,
                           const glm::dvec3 &right, const glm::dvec3 &up,
                           const glm::dvec3 &nose) {
    AeroFrame f;
    const double v = glm::length(v_rel);
    if(v <= 0.0 || glm::length(nose) <= 0.0) { return f; }
    f.v = v;
    f.alpha = std::atan2(-glm::dot(v_rel, up), glm::dot(v_rel, nose));
    f.beta  = std::atan2(glm::dot(v_rel, right), glm::dot(v_rel, nose));
    f.valid = true;
    return f;
}

/* The inverse of those two angles: the unit flow direction that aeroFrame()
   would report as (alpha, beta). Exact everywhere except the corner where both
   reach +-90: the direction is built from their TANGENTS, and at the rim both are
   infinite -- only their ratio says which broadside direction it is, so that one
   point is ambiguous (SilhouetteMemo keeps its chart inside +-75 for the same
   reason, and answers the rim exactly). Zero for a degenerate frame. */
inline glm::dvec3 flowDirection(double alpha, double beta,
                                const glm::dvec3 &right, const glm::dvec3 &up,
                                const glm::dvec3 &nose) {
    if(glm::length(nose) <= 0.0) { return glm::dvec3(0.0); }
    // Taking dot(v, nose) = 1 fixes the scale, leaving the two tangents as the
    // other components; normalizing then undoes the choice.
    const glm::dvec3 v = nose + right * std::tan(beta) - up * std::tan(alpha);
    const double len = glm::length(v);
    return (len > 0.0) ? v / len : glm::dvec3(0.0);
}

/* Lift direction: the ship's up axis with its flow component removed
   (perpendicular to v_rel, on the ship's "up" side). */
inline glm::dvec3 liftDirection(const glm::dvec3 &v_rel,
                                const glm::dvec3 &right,
                                const glm::dvec3 &nose) {
    const double v = glm::length(v_rel);
    if(v <= 0.0 || glm::length(nose) <= 0.0 || glm::length(right) <= 0.0) {
        return glm::dvec3(0.0);
    }
    const glm::dvec3 up   = glm::cross(nose, right);   // the ship's +Y
    const glm::dvec3 vhat = v_rel / v;
    const glm::dvec3 l    = up - glm::dot(up, vhat) * vhat;  // up, out of flow
    const double len = glm::length(l);
    if(len <= 0.0) { return glm::dvec3(0.0); }
    return l / len;
}

/* THE SLOPE BELONGS TO THE AIR, NOT THE PART (#543 A5). Until now every surface
   flew the same CL(α) at Mach 0.01 and Mach 2: a skiff at 85 m/s (M 0.31 on Kerbin)
   and a rocket at M 3 paid the same slope per radian. Compressibility is the biggest
   single omission in the flight model -- it is why a real airframe's wing "goes soft"
   approaching M 1 and why a rocket's fins bite far less at M 3 than at rollout.
   THREE REGIMES, one continuous function:
     M <= kTransonicMach   Prandtl-Glauert: cl / sqrt(1 - M^2). Applied from M = 0,
                           where it is exactly 1 -- no threshold to choose, and at the
                           game's cruise Mach (0.3) it is a 4.8% answer, not noise.
                           Textbook validity runs to ~0.7-0.8; past that the sqrt is
                           extrapolation, which is where the band starts.
     M >= kSupersonicMach  thin-airfoil supersonic slope: 4 / sqrt(M^2 - 1) per radian
                           for a FLAT 2-D SECTION. NOTE what this says: above the band
                           the AUTHORED cl stops mattering at all. Read that as the
                           limit of the theory it comes from, not as a fact about any
                           part: the game's cl is a section slope applied to a planform
                           with no lifting-line correction, so 4/sqrt(M^2-1) is the
                           right absolute slope for a section whose incompressible slope
                           is near 2 pi -- which is what every shipped part authors (6.0)
                           -- and a windfall for one that is not: a cl 1 body is 5.3x its
                           own slope by M 1.25 and still 2x at M 2.2. If that ever needs
                           fixing, the fix is a second authored number, not a min() here.
     between               the two theories do not meet (PG(0.75) = 1.512 cl, thin-air
                           at 1.25 = 5.33 absolute), so the band joins them with a
                           straight line in ABSOLUTE slope, continuous at both edges.
                           At M 1 the join sits halfway between the edges -- 0.756 cl +
                           2.667, which is 7.20 (1.200 cl) for a cl 6 section. Finite at
                           M 1, and for a section above cl 3.53 it is on its way DOWN
                           through the band -- the qualitative fact the old law got
                           backwards. Below that crossover the join still RISES to its
                           high edge, so the curve's peak sits on the band's low edge
                           only for the steep sections the game actually ships.
   A5b (wave drag, the cd rise in the same band) reuses these two edges -- see waveCd
   below. Call sites correct the slope ONCE per surface and pass the result as `cl` to
   the laws below, so #487's rate damping cannot carry its own constant -- liftCurve(),
   liftForce(), liftSlopePerQ(), controlForce() and surfaceTorquePerRadian() all read
   the same corrected number. */
inline constexpr double kTransonicMach = 0.75;
inline constexpr double kSupersonicMach = 1.25;

inline double liftSlopeMach(double cl, double mach) {
    if(!(cl > 0.0) || !(mach > 0.0)) { return cl; }   // no lift, no air, no correction
    if(mach <= kTransonicMach) { return cl / std::sqrt(1.0 - mach * mach); }
    const double subSlope = cl / std::sqrt(1.0 - kTransonicMach * kTransonicMach);
    const double supSlope = 4.0 / std::sqrt(kSupersonicMach * kSupersonicMach - 1.0);
    if(mach < kSupersonicMach) {
        // Weights, not `sub + (sup - sub) * t`: with a non-finite authored cl the
        // difference form answers NaN out of inf - inf, where this one stays inf.
        const double t = (mach - kTransonicMach) / (kSupersonicMach - kTransonicMach);
        return subSlope * (1.0 - t) + supSlope * t;
    }
    return 4.0 / std::sqrt(mach * mach - 1.0);
}

/* THE SAME BAND BILLS WAVE DRAG (#543 A5b). A5 corrected what a section LENDS the air;
   this is what the air charges for the section's THICKNESS, and it lives between the
   same two edges so the two halves of the law cannot disagree about where the regime
   starts. Linear (Ackeret) supersonic theory, zero lift, a thin 2-D SECTION:
       cd_wave = 4 (t/c)^2 / sqrt(M^2 - 1)
   which is the same 1/sqrt(M^2-1) the slope branch above carries -- and it is not a
   multiplier on anything: it is an absolute cd added to the part's own profile cd.
   THREE REGIMES, same shape as liftSlopeMach:
     M <= kTransonicMach   zero. There is no wave drag to linearise subsonically; the
                           term would be pure invention, and PG has none in it.
     M >= kSupersonicMach  Ackeret above, which DECAYS with speed: a 5% section pays
                           0.0133 at M 1.25, 0.0076 at M 1.65, 0.0035 at M 3.
     between               straight join in ABSOLUTE cd, 0 at 0.75 to the Ackeret value
                           at 1.25. Linear theory says nothing here, and the join is the
                           same device A5 used, with the same known cost: the real curve
                           HUMPS just past M 1 (a shock walks onto the wing and the bill
                           peaks ABOVE the far-supersonic value), which a join to zero
                           cannot do. It is a shortfall in the middle of the band, not a
                           wrong sign, and the band is 0.5 Mach wide.
   WHAT THIS DOES NOT TOUCH: a blunt body's cd. The hull's form drag comes from
   surfaceFormCdA / partCd on projected silhouette, which has no chord and no thickness
   ratio to put into the formula. The deeper reason not to extrapolate is that the
   blunt-body curve is not this curve: a pod's cd barely moves from subsonic to
   hypersonic, and a blunt nose can even bill LESS once a shock stands off it, so any
   multiplier on the body anchors would be a guess dressed as a law. Nothing in the
   game can measure that today -- partCd reads no Mach at all, so a hull prints one Cd
   at M 0.2 and the same at M 4.4, and flight/151's flat column is the omission, not
   evidence about pods. Getting it right means authored supersonic anchors per body
   part (real numbers, not a multiplier) -- filed as #635 rather than guessed at here.
   t/c comes off the mesh box the edge-area law already reads (PartDef::meshThickness-
   ToChord): every shipped surface is 0.044-0.086, so at the game's wing chord cd_wave
   lands beside the 0.01 profile cd those parts author -- at M 1.65 it is 0.0076, i.e.
   a 76% rise in what a wing's own shape costs. A section thicker than kWaveThickness-
   Limit is outside linear theory (a part with NO mesh reads 2 x radius / height, which
   is >= 1 for the plate defs that fallback exists for); cap it there so a malformed
   def bills a lot rather than absurdly.
   Call sites correct ONCE and pass the result as the existing cd argument, exactly as
   A5 did with cl: surfaceFormCdA() takes cd_forward + wave and cd_backward + wave and
   knows nothing about Mach. The edge and broadside shares of a plate's shape drag get
   nothing -- they are not sections meeting the flow chordwise. Neither does the LIFT
   half of Ackeret: a deflected section's extra wave drag is priced by inducedCd off the
   force the slope law just made, so it stays charged once rather than twice. */
inline constexpr double kWaveThicknessLimit = 0.3;

inline double waveCd(double thicknessToChord, double mach) {
    if(!(thicknessToChord > 0.0) || !(mach > kTransonicMach)) { return 0.0; }
    const double tc = (thicknessToChord < kWaveThicknessLimit)
                    ? thicknessToChord : kWaveThicknessLimit;
    // The join's high edge IS the M 1.25 Ackeret value, so the two branches meet.
    const double supEdge = 4.0 * tc * tc / std::sqrt(kSupersonicMach * kSupersonicMach - 1.0);
    if(mach < kSupersonicMach) {
        return supEdge * (mach - kTransonicMach) / (kSupersonicMach - kTransonicMach);
    }
    return 4.0 * tc * tc / std::sqrt(mach * mach - 1.0);
}

/* CL(α) of a symmetric section, soft stall: linear to the stall angle, then a
   cosine droop to zero by 2·stallAngle. A = 0 disables the stall. CL stays
   continuous (no force jump).
   `cl` is the slope PER RADIAN AT THE FLOW'S MACH -- get it from liftSlopeMach().
   The droop and the peak scale with that slope, which means the section keeps ONE
   authored stall angle at every Mach and its CL_max climbs 51% by Mach 0.75. Both of
   those are the wrong shape near the sound bar: a real section stalls at a LOWER angle
   as a shock walks onto the wing and its CL_max COLLAPSES toward M 1.2-1.4 while its
   linear slope is still rising. That is the one part of the curve A5 did NOT make
   compressible -- tracked as #633, and pinned as a deliberate simplification in
   tests/test_drag.cpp::test_lift_slope_mach until it lands. Do not "fix" it here with a
   cap on the peak: the droop is a cosine of the same slope, so a cap would break the
   continuity the curve has today. */
inline double liftCurve(double cl, double alpha, double stallAngle) {
    if(cl <= 0.0) { return 0.0; }
    const double a = std::fabs(alpha);
    const double A = stallAngle;
    double c;
    if(A <= 0.0 || a <= A) {
        c = cl * a;                                          // linear (up to stall)
    } else {
        const double t = (a - A) / A;                        // 0 at A, 1 at 2A
        c = (t >= 1.0) ? 0.0 : cl * A * std::cos(t * std::numbers::pi * 0.5);  // droop
    }
    return (alpha < 0.0) ? -c : c;                           // sign follows α
}

/* The lift force on one part: L = q · S · CL(α) · liftDir. */
inline glm::dvec3 liftForce(double q, double S, double cl, double alpha,
                            const glm::dvec3 &liftDir, double stallAngle = 0.0) {
    if(q <= 0.0 || S <= 0.0 || cl <= 0.0) { return glm::dvec3(0.0); }
    return liftDir * (q * S * liftCurve(cl, alpha, stallAngle));
}

/* THE NEUTRAL POINT (#454): where an angle-of-attack change puts the CHANGE in lift.
   Not where the lift is -- that question is already answered per part, since #453
   applies every surface's force at its own mount. What was missing is the derivative:
   turn the nose one degree into the flow and every surface adds lift, and where that
   extra lift sits says whether the airframe fights the overshoot or feeds it. That is
   what a player means by "it flies": a released stick settles because one number here
   is positive.
   Feed it one surface at a time from the lift loop -- see liftSlopePerQ for the
   increment vector, which is where the subtlety is -- then read point()/margin().
   q is absent on purpose: it multiplies the lift change and the moment change
   equally, so it cancels out of the lever arm and everything stays in area units
   (m^2/rad, m^3/rad), comparable with the [drag] line's A= and Induced=.
   Only lifting surfaces belong here. A control surface has no angle term in its law
   at all (controlForce is pure deflection), so it contributes no margin and cannot
   stabilise anything -- it trims, which is a different job (#492). And nothing here
   involves the hull's FORM drag: that restores by weathervaning, which is the CpL=
   lever, a separate source of the same judgement, and in a real airframe the two add.
   Each surface's INDUCED drag (vehicle.cpp, right below the lift loop) also leans on
   pitch -- it grows with CL^2 and is applied at the same mount -- and is outside this
   sum, so near the top of the polar the true margin is a little further aft than the
   one printed. #494 put a SECOND such term on the very surfaces this paragraph says
   contribute nothing: a HELD deflection bills induced drag at its own mount, so on a
   trimmed airframe it is the larger of the two (measured on the pre-#568 skiff_trim:
   0.18 of its 0.45 m2 of Induced=, on a ~2 m lever; the shipped stack trims to 0.33 m2
   of Induced= with its elevators 2.8 m aft of the CG) and it grows with the trim the
   player sets, as delta^2. It is a force along the flow, so it pitches one way from a
   mount behind the CG and exactly the other from one ahead of it -- a canard's trim
   would push this sum's true margin the
   wrong way. Neither term is in the printed SM=, which is a lift-curve margin and
   nothing more; that is a limit of the readout, not of the law. */
struct LiftSlopeSum {
    double dl = 0.0;    // sum dL/dalpha . liftAxis        -- m^2/rad
    double dlabs = 0.0; // the same surfaces, unsigned     -- m^2/rad
    double dm = 0.0;    // sum (ri x dL/dalpha) . latAxis  -- m^3/rad
    void add(const glm::dvec3 &dLdA, const glm::dvec3 &ri,
             const glm::dvec3 &liftAxis, const glm::dvec3 &latAxis) {
        // Both axes have to be unit or the metres of point()/margin() stop being
        // metres: dl and dm would carry different scale factors and their ratio
        // would still print as a plausible lever. Production passes frame columns
        // and partAero's liftDir, which are.
        assert(std::abs(glm::dot(liftAxis, liftAxis) - 1.0) < 1e-9);
        assert(std::abs(glm::dot(latAxis, latAxis) - 1.0) < 1e-9);
        const double c = glm::dot(dLdA, liftAxis);
        dl += c;
        dlabs += std::fabs(c);
        dm += glm::dot(glm::cross(ri, dLdA), latAxis);
    }
    /* Quote the pair or dash it -- never quote a lever out of a sum that cancelled.
       Two reasons to dash: nothing lifting (no lifting surface at all, or all of them
       stalled flat or mounted edge-on, so dlabs itself is ~0), and surfaces that
       DISAGREE (dl a small difference of large numbers, so where the average lands is
       arithmetic noise). The second is the one that used to lie: the first version of
       this sum weighted each surface by its own slope signed against ship-up, which
       makes a clock-0/clock-180 twin pair cancel EXACTLY -- the lift direction flips
       and the angle of attack flips with it -- and the log happily quoted
       "Np=+15.87 m" for the shipped skiff, drifting to +51.74, +121.54 and -89.73 on
       other frames of the same plane. The pair now dashes instead, and the unit pin in
       test_drag.cpp::test_neutral_point exists because a mutant of exactly that
       weighting left the whole rest of the suite green.
       The 0.1 is agreement, not magnitude: at cruise the skiff is 95% agreed (its fin
       is in dlabs and 0 in dl) and quotes +0.55 m; crossing its own stall peak it drops
       under it (measured on the pre-#568 hull at AoA 20.10: dl 8.2 of 161.8, which
       would otherwise print SM=+2.22 m --
       an airframe that has just stopped resisting, reading as MORE stable) and dashes.
       Everything drooping TOGETHER keeps the ratio and keeps quoting: that case is
       weak, not incoherent, and slope() is the field that says so. */
    bool usable() const {
        return std::isfinite(dl) && std::isfinite(dm) && dlabs > 1e-9
            && std::fabs(dl) >= 0.1 * dlabs;
    }
    /* How much lift an angle-of-attack change buys at all, m^2/rad (CL·A per radian,
       so multiply by q for the force). Print it NEXT TO the margin: a stalled
       airframe can hold a healthy-looking margin -- if every surface droops together
       their slopes scale by the same factor and the weighted AVERAGE of their
       positions does not move -- while its stiffness collapses towards zero and
       nothing pushes the nose back any more. Measured on the jet at AoA 28 and -35
       (both inside the 20.1-40.1 droop): SM stayed +0.55 m, and it is CLa that says
       why the airframe had stopped caring. #487's band, in other words, is a
       stiffness story, not a sign story.
       Measured along the SHIP's up axis, so it carries a cos(alpha) dip: the shipped
       skiff quotes 167.82-167.84 across its 2.5-2.8 deg of cruise AoA, inside 0.05 of
       168.0 x cos(alpha); on the pre-#568 hull the same reading ran 153.60 at 0.4 deg,
       153.34 at 3.5, 152.75 at 5.9 and 149.95 at 12.7, each within 0.2 of 153.6 x
       cos(alpha). That is the right number for a pitch stiffness (the nose rotates
       about the ship), but it is why a steady-airframe
       CLa is not a constant. */
    double slope() const { return dl; }
    /* Metres along the nose axis from the pivot, + = ahead of it. From the root
       frame's right x up = nose (so nose x up = -right), a lift increment L' acting
       at offset x·nose makes moment -x·L' about the lateral axis, hence M' = -x·L'. */
    double point() const { return -dm / dl; }
    /* The STATIC MARGIN: how far behind the pivot that lift change sits, so + =
       raising the nose makes more lift BEHIND the COM, which pushes the nose back
       down = restoring. This is the number to watch while building: the shipped
       airframes all measure positive (skiff +0.55 m -- the skiff's wings are 0.32 m
       aft of its CG and its tailplanes 1.97 m, so the wings themselves are already
       restoring and the tail adds the other half), and a
       negative one is an airframe that diverges on its own -- tail ahead of the
       wings, or a mount angle that puts a surface on the far side of its own stall.
       It is a WEIGHTED AVERAGE of where the lift change sits, in metres, so it does
       not compare between airframes of different size; compare it against a chord
       when you want a figure of merit. */
    double margin() const { return dm / dl; }
};

/* One surface's OWN view of the flow (#453): its angle of attack and its own lift
   direction, both out of the part's frame (columns = right, up, nose) instead of
   the ship's. A wing mounted with incidence, dihedral, or upside down then lifts
   the way it is bolted rather than the way the pod is pointed -- the difference
   between an airframe that can be trimmed and one where the editor's wing angle
   is decoration. Only the angles a surface mount can express reach here:
   attachSurface keeps the child's +Z along the parent's, so a mount tilts and
   rolls a chord but cannot sweep it in plan view.
   Note what this does NOT do: a part turned 180 deg about its own nose flips up
   AND right, so alpha flips sign and liftDir flips with it, and the force comes
   out the SAME as the ship-wide frame's. That is why the shipped airframes --
   wing_1 at attach angle 0 and wing_2 at 180 on one parent, rudders at 90 and
   270 -- keep flying exactly as they do today once each part reads its own frame:
   float noise, ~1e-11 N and N m on the totals at 5 deg AoA. Pinned part by part
   over jet.json and glider.json in test_shipload, and as pure geometry in
   tests/test_drag.cpp, because a flight log cannot tell "nothing moved" from
   "the law is broken".
   That claim is about the FRAMES alone. v_rel here is a per-surface quantity now
   (#483): the aero pass hands each part v_rel + w x r measured at that part's own
   offset, so a rotating airframe's surfaces meet a flow the COM does not. That is
   NOT a no-op on shipped content -- the glider's tumble decays to under 2 deg and
   it keeps 50% more of its speed. Two claims, two pins: test_shipload for the
   frames, flight/458-glider for the flow. A body that is not spinning still gets
   bit-identical numbers, because w x r is exactly zero. */
struct PartAero {
    double alpha = 0.0;                 // rad, this part's own angle of attack
    glm::dvec3 liftDir = glm::dvec3(0.0);   // unit, this part's own; zero if unusable
    bool valid = false;                 // false -> no usable frame or no lift axis
};

inline PartAero partAero(const glm::dmat3 &partRot, const glm::dvec3 &v_rel) {
    PartAero pa;
    const AeroFrame fr = aeroFrame(v_rel, partRot[0], partRot[1], partRot[2]);
    if(!fr.valid) { return pa; }
    pa.liftDir = liftDirection(v_rel, partRot[0], partRot[2]);
    /* Negated on purpose: a NaN flow gives a NaN axis, and `dot <= 0` is FALSE for
       NaN, so the plain form certifies a garbage frame as usable (vehicle.cpp's
       kAoAFlowFloor comment has been saying as much). Asking for a positive squared
       length refuses both the zero residue and the NaN one. */
    if(!(glm::dot(pa.liftDir, pa.liftDir) > 0.0)) { return pa; }
    pa.alpha = fr.alpha;
    pa.valid = true;
    return pa;
}

/* THE CHANGE IN A SURFACE'S LIFT when the SHIP's angle of attack changes, per unit q
   (so m^2/rad along its own lift axis) -- the increment #454's neutral point is made
   of. The mount turns WITH the ship, which is the same relative wind as turning the
   flow the other way about the ship's lateral axis, so the derivative is taken by
   tilting the flow and asking the surface's own law. That "ask the law" choice is the
   whole point: nothing here gets to decide which way a mount faces, because the
   shortcuts do so and fail silently. Weighting by slope*area*dot(liftDir, up) -- which
   is what I wrote first -- makes a twin pair at clock 0 and 180 cancel EXACTLY, since
   its liftDir and its alpha are both flipped, and the resulting denominator of ~0 had
   the drag log quoting Np=+15.9 m, then +121.5, then -89.7 for the same skiff. A v-tail
   at 45 deg answers to only part of a pitch, and a fin to almost none of one; tilting
   the flow gets both out of the laws that already make the force, stall droop included:
   a surface inside the droop contributes NEGATIVELY, which is an airframe diverging
   past the stall (#487's band) rather than a display glitch.
   The sign of the tilt only sets which way the SLOPE prints -- reversing it flips dl
   and dm together and margin() is their ratio -- so it is picked for the reader: tilting
   the flow +eps about the ship's lateral axis makes a surface in its linear range report
   a POSITIVE slope, which is what "more angle of attack, more lift" should look like.
   The AXIS is not free: tilting about the ship's lift axis is yaw, and the number would
   then quietly describe a different instability.
   Pass the PartAero the force loop already computed rather than recomputing it: the
   mount, flow and eps are the same, so a second sample of `here` is 44 ns per lifting
   part per substep for nothing. Centered (+-eps/2), not forward: liftCurve's soft-stall
   peak is a kink, and a forward difference reports only the alpha-increasing side -- at
   the skiff's onset the printed CLa moved 5x with eps (1e-5 gave 144, 1e-3 gave 29,
   1e-2 gave -1.2). Only the MAGNITUDE is differentiated; the lift direction swinging
   with the flow is dropped, which is 2-4% of the slope at AoA 8-12 and none of the
   margin (both are measured projections on the SHIP's up axis, hence the cos alpha dip).
   An analytic slope would be ~10x cheaper -- liftCurve's derivative is closed form --
   but then a change to the law would not show up in the derivative, and the whole point
   of differencing is that the number keeps describing whatever the force loop does.
   `cl` is the slope AT THE FLOW'S MACH (drag.h::liftSlopeMach), not the authored one.
   The tilt is a ROTATION of `flow`, so |flow| -- and with it the Mach -- is identical
   at lo and hi, which is what lets the correction ride through this difference as the
   constant it is, rather than being re-derived per sample. */
inline glm::dvec3 liftSlopePerQ(const PartAero &here, const glm::dmat3 &partRot,
                                const glm::dvec3 &flow, double S, double cl,
                                double stallAngle, const glm::dvec3 &latAxis,
                                double eps = 1e-3) {
    if(!here.valid) { return glm::dvec3(0.0); }
    const PartAero lo = partAero(partRot, glm::angleAxis(-eps * 0.5, latAxis) * flow);
    const PartAero hi = partAero(partRot, glm::angleAxis(eps * 0.5, latAxis) * flow);
    if(!lo.valid || !hi.valid) { return glm::dvec3(0.0); }
    const double dcl = (liftCurve(cl, hi.alpha, stallAngle)
                      - liftCurve(cl, lo.alpha, stallAngle)) / eps;
    return here.liftDir * (S * dcl);
}

/* THE FLOW A SURFACE STANDS IN (#483). A part bolted at offset r from the centre
   of mass is carried through the air by the body's rotation, so the air it meets
   is the COM-relative flow plus its own sweep w x r, and its dynamic pressure
   follows THAT flow: a rotating airframe has no single q to take. One function so
   the lift loop, the control loop and the chute harness cannot each re-inline the
   composition and drift apart, and so the unit test pins production rather than a
   copy of it.
   vp2 is |vp|^2: a caller that needs the flow's DIRECTION (the induced term)
   takes one sqrt here instead of paying a second magnitude on top of q's.
   The frame of w must be the frame of vrel and of the hull body being pushed;
   in updateAerodynamics all three are S. */
struct LocalFlow {
    glm::dvec3 vp;   // the flow at this surface
    double q;        // 0.5 * rho * |vp|^2 -- this surface's dynamic pressure
    double vp2;      // |vp|^2
};

inline LocalFlow localFlow(const glm::dvec3 &vrel, const glm::dvec3 &w,
                           const glm::dvec3 &r, double rho) {
    const glm::dvec3 vp = vrel + glm::cross(w, r);
    /* glm::dot, not glm::length2: in the vendored glm, length2 is a gtx/norm.hpp
       extra that drag.h does not (and should not) pull in. Same expression --
       length2(x) is dot(x, x) -- so the number is unchanged. */
    const double vp2 = glm::dot(vp, vp);
    return { vp, 0.5 * rho * vp2, vp2 };
}

/* The Mach the flow at one point stands in. One spelling so a call site cannot divide
   by a different sound speed than its neighbour did: `invAirA` is 1 / a at the hull's
   COM (vehicle.cpp), and a LocalFlow carries |v|^2 AT THE SURFACE (#483), which is the
   speed that surface's slope correction (#543 A5) and its wave drag (A5b) use. */
inline double flowMach(const LocalFlow &lf, double invAirA) {
    return std::sqrt(lf.vp2) * invAirA;
}

/* The 2-D cross product (z-component) of (b-a) x (c-a). */
inline double cross2(const glm::dvec2 &a, const glm::dvec2 &b,
                     const glm::dvec2 &c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

/* The plane perpendicular to a view direction, as an orthonormal 2-D basis.
   Project a WHOLE ship through ONE of these when its parts' silhouettes are to
   be combined: if every part picks its own basis from its own direction, the
   polygons land in different planes and cannot be unioned (#463). */
struct FlowPlane {
    glm::dvec3 u{0.0}, w{0.0};
    bool valid = false;
};

inline FlowPlane flowPlane(const glm::dvec3 &dir) {
    const double len = glm::length(dir);
    if(len <= 0.0) { return FlowPlane(); }
    const glm::dvec3 d = dir / len;
    const glm::dvec3 ref = (std::fabs(d.x) < 0.9) ? glm::dvec3(1, 0, 0)
                                                  : glm::dvec3(0, 1, 0);
    FlowPlane pl;
    pl.u = glm::normalize(glm::cross(d, ref));
    pl.w = glm::cross(d, pl.u);
    pl.valid = true;
    return pl;
}

/* The silhouette POLYGON of a convex-hull vertex set seen edge-on: the hull of
   the projected vertices, which IS the silhouette -- so this stays right for a
   NON-convex mesh. Symmetric in the view direction. `out` is caller-owned
   scratch (this runs per part per physics substep); it ends up a CCW polygon,
   empty for a degenerate (collinear) projection. The output is CONVEX by
   construction -- unionProjectedArea is only correct for convex rings, because
   inside one slab it assumes two active edges cannot cross. */
inline void projectHull(const std::vector<glm::dvec3> &hullVerts,
                        const FlowPlane &pl, std::vector<glm::dvec2> &out) {
    out.clear();
    if(!pl.valid || hullVerts.size() < 3) { return; }
    out.reserve(hullVerts.size());
    for(const glm::dvec3 &v : hullVerts) {
        out.push_back(glm::dvec2(glm::dot(v, pl.u), glm::dot(v, pl.w)));
    }
    // Andrew's monotone chain: convex hull of the projected points (CCW,
    // collinear dropped). glm::dvec2 has no operator<, hence the explicit
    // x-then-y comparator.
    std::sort(out.begin(), out.end(),
              [](const glm::dvec2 &a, const glm::dvec2 &b) {
                  return (a.x < b.x) || (a.x == b.x && a.y < b.y);
              });
    static thread_local std::vector<glm::dvec2> h;
    h.clear();
    for(size_t i = 0; i < out.size(); i++) {
        while(h.size() >= 2
              && cross2(h[h.size() - 2], h[h.size() - 1], out[i]) <= 0.0) {
            h.pop_back();
        }
        h.push_back(out[i]);
    }
    const size_t lowerSize = h.size() + 1;
    for(int i = (int)out.size() - 1; i >= 0; i--) {
        while(h.size() >= lowerSize
              && cross2(h[h.size() - 2], h[h.size() - 1], out[i]) <= 0.0) {
            h.pop_back();
        }
        h.push_back(out[i]);
    }
    h.pop_back();                              // drop the duplicated start point
    out.swap(h);                               // out keeps the hull, h the buffer
}

/* One PART's silhouette in a flow plane shared by the whole ship: its collision
   verts through its MOUNT (rotation and position) into that one 2-D basis.
   A part's area alone is translation-invariant, which is why the mount position
   used to be ignorable; it stops being ignorable the moment two parts' polygons
   are unioned (#463), so this is the only projection a ship should use. */
inline void projectPartHull(const std::vector<glm::dvec3> &hullVerts,
                            const glm::dmat3 &localRot, const glm::dvec3 &localPos,
                            const FlowPlane &pl, std::vector<glm::dvec2> &out) {
    // Rotating the basis into the part costs two mat3-vec multiplies and keeps
    // the ship's basis shared; the alternative is rotating every vert (2x the
    // parts' vert count) into the ship frame. dmat3 throughout: Part::localRot is
    // a dmat3, and narrowing it costs the projected coords ~1e-7 relative.
    const glm::dmat3 toLocal = glm::transpose(localRot);
    projectHull(hullVerts, FlowPlane{toLocal * pl.u, toLocal * pl.w, pl.valid}, out);
    const glm::dvec2 off{glm::dot(localPos, pl.u), glm::dot(localPos, pl.w)};
    for(glm::dvec2 &q : out) { q += off; }
}

/* Shoelace area of a simple polygon (either winding). Zero for a degenerate
   (fewer than three points, or collinear) one. */
inline double polygonArea2(const std::vector<glm::dvec2> &poly) {
    if(poly.size() < 3) { return 0.0; }
    double a = 0.0;
    for(size_t i = 0; i < poly.size(); i++) {
        const size_t j = (i + 1) % poly.size();
        a += poly[i].x * poly[j].y - poly[j].x * poly[i].y;
    }
    return 0.5 * std::fabs(a);
}

/* The centroid of a simple polygon, either winding, from the same shoelace sum
   as polygonArea2. Exact for a polygon (not a sample) -- NOT the vertex mean,
   which only coincides with it for point-symmetric rings and triangles. `origin`
   is subtracted from every vertex before the sums, which does not change the
   answer mathematically and buys a lot of conditioning: a part's silhouette can
   sit hundreds of metres from the plane's origin (a 200 m station, or simply a
   hull far along the ship), where the moment sums would otherwise cancel to
   ~1e-6 m of noise. Degenerate for a collinear or too-few-vertex ring, where `c`
   is left as passed in. */
inline void polygonCentroid2(const std::vector<glm::dvec2> &poly, glm::dvec2 &c,
                             const glm::dvec2 &origin = glm::dvec2(0.0)) {
    if(poly.size() < 3) { return; }
    double a2 = 0.0, mx = 0.0, my = 0.0;
    for(size_t i = 0; i < poly.size(); i++) {
        const glm::dvec2 p = poly[i] - origin;
        const glm::dvec2 q = poly[(i + 1) % poly.size()] - origin;
        const double cr = p.x * q.y - q.x * p.y;
        a2 += cr;
        mx += (p.x + q.x) * cr;
        my += (p.y + q.y) * cr;
    }
    if(a2 == 0.0) { return; }
    c = origin + glm::dvec2(mx, my) / (3.0 * a2);
}

/* WHERE a part's drag actually pushes, relative to its own mount: the centroid of
   its projected silhouette, measured IN THE SAME PLANE as the mount's own
   projection, so the answer is a free 2-D vector (#512). A part's mount says
   nothing about where its silhouette is: a cone+cylinder capsule seen broadside
   has its area a third of its length from the bolt, a plate authored at its root
   half a span away, and a ship built out of those gets a weathervane arm that is
   simply wrong. Exactly zero for a hull that is POINT-sYMMETRIC about its mount
   (box, cylinder, a plate centred on its bolt -- most shipped parts, which is why
   the shipped beds barely move); a cone is not, even when its mount sits at the
   middle of its bounding box.
   Zero for a degenerate polygon. In production that guard is a formality -- the
   monotone-chain projection of a collinear hull comes back with two points, so it
   never reaches a >=3-point zero-area ring -- but the helper's contract says so
   and the pin says so.
   Multiply by the plane basis and rotate by the ship's rotation to get a 3-D
   offset. The component along the FLOW stays with the mount, because a projection
   cannot say where along its own normal the area sits. That is an approximation
   with a known size, not a free choice: the cp's along-flow error lambda is
   exactly inert while the force runs along vrel, but #486 charges the hull at the
   LOCAL flow, which the ship's rotation tilts by w x (cp - com), so a moment
   error of order |w| lambda / |v| slips in -- about 0.1% for the shipped hulls
   (lambda under 0.4 m at a few rad/s) and about 10% for the 4 m plate at 1 rad/s.
   Nothing recovers that coordinate from a silhouette, so it stays with the mount
   and the size of the lie is written down. */
/* Joining that offset to a point: off2 lives in pl's 2-D basis, which is a
   SHIP-LOCAL plane, while partPosS is in the frame the ship pose lives in. So the
   offset goes through pl's basis and through sRot; the plane basis is not the
   ship's axes and sRot is not optional. Exposed as a function for exactly this
   reason -- the whole fix is this composition, and identity-aligned unit pins
   cannot see it. */
inline glm::dvec3 silhouetteCpPointS(const glm::dmat3 &sRot, const glm::dvec3 &partPosS,
                                     const glm::dvec2 &off2, const FlowPlane &pl) {
    return partPosS + sRot * (off2.x * pl.u + off2.y * pl.w);
}

inline glm::dvec2 silhouetteCpOffset2(const std::vector<glm::dvec2> &poly,
                                      const glm::dvec2 &mount2) {
    if(poly.size() < 3) { return glm::dvec2(0.0); }
    glm::dvec2 c = mount2;
    polygonCentroid2(poly, c, mount2);   // relative to the mount, so no cancellation
    return c - mount2;
}

/* The projected (silhouette) area of ONE body seen along `dir`, from its CONVEX
   HULL vertices. */
inline double projectedArea(const std::vector<glm::dvec3> &hullVerts,
                            const glm::dvec3 &dir) {
    static thread_local std::vector<glm::dvec2> poly;
    projectHull(hullVerts, flowPlane(dir), poly);
    return polygonArea2(poly);
}

/* The area of the UNION of a set of convex polygons -- the honest silhouette of
   a ship made of separated solids (#463). Convex-hulling every part's vertices
   together instead draws a skin over the gaps: on the shipped jet that turns two
   0.4 m^2 plates into 12.2 m^2 of wing that is not there, and the tank-to-nose
   stack into more fuselage than there is.

   Exact by vertical decomposition: cut the plane at every vertex x and every
   place two polygons' edges cross. Within one such slab the active edges are
   fixed and no two of them cross, so each merged interval's height is linear in
   x, which makes the merged measure linear in x too -- its value at the slab
   midpoint times the slab width is therefore exact, not a sample.

   Costs edge-pair work per direction, so it belongs at cache-node time, not in
   the substep loop; see SilhouetteMemo. */
inline double unionProjectedArea(const std::vector<std::vector<glm::dvec2>> &polys) {
    struct Span { size_t poly; double lo, hi; };
    static thread_local std::vector<Span> spans;
    static thread_local std::vector<double> xs;
    static thread_local std::vector<std::pair<double, double>> bands;
    spans.clear();
    xs.clear();
    for(size_t i = 0; i < polys.size(); i++) {
        if(polys[i].size() < 3) { continue; }
        Span sp{i, polys[i][0].x, polys[i][0].x};
        for(const glm::dvec2 &v : polys[i]) {
            if(v.x < sp.lo) { sp.lo = v.x; }
            if(v.x > sp.hi) { sp.hi = v.x; }
        }
        spans.push_back(sp);
        for(const glm::dvec2 &v : polys[i]) { xs.push_back(v.x); }
    }
    if(spans.empty()) { return 0.0; }
    // Polygon-pair edge crossings: the only places the merged measure can change
    // curvature. The x-range tests reject most pairs of a ship's hulls cheaply.
    for(size_t a = 0; a < spans.size(); a++) {
        for(size_t b = a + 1; b < spans.size(); b++) {
            const std::vector<glm::dvec2> &pa = polys[spans[a].poly];
            const std::vector<glm::dvec2> &pb = polys[spans[b].poly];
            const double xlo = std::max(spans[a].lo, spans[b].lo);
            const double xhi = std::min(spans[a].hi, spans[b].hi);
            if(!(xlo < xhi)) { continue; }
            for(size_t i = 0; i < pa.size(); i++) {
                const glm::dvec2 &p = pa[i], &q = pa[(i + 1) % pa.size()];
                const double alo = std::min(p.x, q.x), ahi = std::max(p.x, q.x);
                if(ahi <= xlo || alo >= xhi) { continue; }
                for(size_t j = 0; j < pb.size(); j++) {
                    const glm::dvec2 &r = pb[j], &s = pb[(j + 1) % pb.size()];
                    // Prune against THIS edge's x-range, not just the two
                    // polygons' overlap -- an order of magnitude more pairs die here.
                    if(std::max(r.x, s.x) <= alo || std::min(r.x, s.x) >= ahi) { continue; }
                    const double d1x = q.x - p.x, d1y = q.y - p.y;
                    const double d2x = s.x - r.x, d2y = s.y - r.y;
                    const double den = d1x * d2y - d1y * d2x;
                    if(den == 0.0) { continue; }      // parallel: no crossing
                    const double t = ((r.x - p.x) * d2y - (r.y - p.y) * d2x) / den;
                    if(t < 0.0 || t > 1.0) { continue; }
                    const double u = ((r.x - p.x) * d1y - (r.y - p.y) * d1x) / den;
                    if(u < 0.0 || u > 1.0) { continue; }
                    xs.push_back(p.x + t * d1x);
                }
            }
        }
    }
    std::sort(xs.begin(), xs.end());
    // Unique to a relative epsilon. Every vertex x is already in the list
    // exactly, so this only merges a crossing that landed a rounding hair off an
    // endpoint. Scaled by the coordinate magnitude rather than the whole ship's
    // span, so a small part's own breakpoints cannot be swallowed by the ship
    // being large next to it.
    const double scale = std::max(std::fabs(xs.front()), std::fabs(xs.back()));
    const double eps = 1e-12 * (scale > 1.0 ? scale : 1.0);
    size_t n = 0;
    for(size_t i = 0; i < xs.size(); i++) {
        if(n == 0 || xs[i] - xs[n - 1] > eps) { xs[n++] = xs[i]; }
    }
    double area = 0.0;
    for(size_t k = 0; k + 1 < n; k++) {
        const double x0 = xs[k], x1 = xs[k + 1];
        if(!(x1 > x0)) { continue; }
        const double xm = 0.5 * (x0 + x1);
        // Where the vertical line at xm cuts each polygon that spans it.
        bands.clear();
        for(const Span &sp : spans) {
            if(xm < sp.lo || xm > sp.hi) { continue; }
            const std::vector<glm::dvec2> &poly = polys[sp.poly];
            double lo = 0.0, hi = 0.0;
            bool hit = false;
            for(size_t i = 0; i < poly.size(); i++) {
                const glm::dvec2 &p = poly[i], &q = poly[(i + 1) % poly.size()];
                // Half-open so a vertex shared by two edges counts once. An edge
                // lying ON xm cannot reach here: xm is a slab MIDPOINT and every
                // vertex x is a breakpoint, so no vertex sits at xm -- which also
                // means a straddling edge has p.x != q.x and the divide is safe.
                if((p.x <= xm && q.x > xm) || (q.x <= xm && p.x > xm)) {
                    const double t = (xm - p.x) / (q.x - p.x);
                    const double y = p.y + t * (q.y - p.y);
                    if(!hit) { lo = hi = y; hit = true; }
                    else {
                        if(y < lo) { lo = y; }
                        if(y > hi) { hi = y; }
                    }
                }
            }
            if(hit && hi > lo) { bands.push_back({lo, hi}); }
        }
        if(bands.empty()) { continue; }
        std::sort(bands.begin(), bands.end());
        double measure = 0.0;
        double clo = bands[0].first, chi = bands[0].second;
        for(size_t i = 1; i < bands.size(); i++) {
            if(bands[i].first > chi) {
                measure += chi - clo;
                clo = bands[i].first;
                chi = bands[i].second;
            } else if(bands[i].second > chi) {
                chi = bands[i].second;
            }
        }
        measure += chi - clo;
        area += measure * (x1 - x0);
    }
    return area;
}

/* The silhouette area as a function of the flow DIRECTION only (#463). Speed and
   density multiply the force, never its shape, so the union above is worth
   computing once per direction rather than once per substep: measured on the
   shipped jet, a union pass costs about 0.5 ms over its 13 hulls (3.3k verts),
   and the convex-hull balloon it replaces cost 1.7 ms at the 33k-vert balloon.

   The table is a grid over the two angles AeroFrame already produces -- alpha
   (flow off the nose, pitch plane) and beta (the same, yaw plane) -- read
   bilinear between its four surrounding nodes. Nodes are NOT computed up front:
   they fill the first time a ship's flow enters the cell, so a parked or
   cruising ship pays only for the handful of directions it actually flies, and
   a direction sweep spreads across the flight instead of landing in one
   rebuild. The whole table goes stale by bumping a generation counter; nothing
   is cleared.

   A cell whose corners are not all filled reports a MISS and the caller uses the
   exact area for the direction it is already flying -- which the per-part
   projections it computes anyway give for free. So a MISS is never wrong, but it
   is not free either: the substep pays its own exact pass PLUS up to `budget` node
   evaluations. Filling trades this substep's time for later hits; the caller keeps
   the budget small. Cost of one node pass, measured on a pod-plus-two-plates hull:
   0.009 ms at 13 parts, 0.18 ms at 100, 0.58 ms at 200 -- the overlap prune is
   pair-wise, so it grows quadratically with the part count. */
struct SilhouetteMemo {
    // 5 deg nodes over +-75 deg in both angles. kFoldHalf is the atan2 chart's
    // own bound: past +-90 the flow comes from behind, and both angles enter a
    // direction through tan(), whose period folds those directions onto this chart
    // (see fold). The cached range stops short of that bound on purpose -- see
    // the note on kHalf.
    static constexpr int kSteps = 31;
    static constexpr double kFoldHalf = std::numbers::pi_v<double> / 2.0;
    // The angles parameterise a direction as tan(alpha), tan(beta), which blows up
    // at 90 deg, so cells near +-90 span an enormous slice of direction space and
    // blending across them is poor -- at the rim the parameterisation is not merely
    // coarse but ambiguous (see flowDirection). Measured over the folded sphere on
    // a jet-shaped hull (pod + two 8 m plates, mounts honoured): worst 8.3%, mean
    // 0.22% in-chart (tmp/review_20261028/probe_chart.cpp). Widening this to +-90
    // with 37 nodes keeps the same 5 deg cells and the same error, so the narrower
    // range buys nothing in resolution -- it exists to refuse the rim, where the
    // blend is unbounded. The 8% lives in the middle of the chart instead: the
    // union JUMPS where a part slides in or out of another part's shadow, and no
    // smooth interpolation of neighbouring nodes follows a jump. #473.
    static constexpr double kHalf = 75.0 * std::numbers::pi_v<double> / 180.0;
    static constexpr double kStep = (2.0 * kHalf) / (kSteps - 1);
    static_assert(kHalf <= kFoldHalf, "the chart may not exceed the fold's own chart");

    std::vector<float> area;       // [m^2]; node (i, j) at j * kSteps + i
    std::vector<uint32_t> stamp;   // the generation the node was filled at
    uint32_t gen = 0;              // bump to invalidate every node at once

    void reset() {
        area.assign(kSteps * kSteps, 0.0f);
        stamp.assign(kSteps * kSteps, 0u);
        gen = 1;                   // 0 means "never filled"
    }
    /* Free the table. A ship that stops having parts stops having a silhouette:
       a node outliving its hull would report an area for a body that is not
       there, and the old balloon law could not do that because an empty hull
       projected to zero area. Frees the buffer on purpose -- this is called when
       a ship is absorbed, and the shell may live on. */
    void clear() {
        std::vector<float>().swap(area);
        std::vector<uint32_t>().swap(stamp);
        gen = 0;
    }
    // Geometry changed: nothing in the table is trustworthy. No allocation here --
    // a parked or orbital ship that never flies in a direction should not pay
    // 7.7 kB for a table it never consults (fillAround allocates on first use).
    void invalidate() {
        gen++;
        if(gen == 0) { gen = 1; }   // skip 0: it means "never filled"
    }
    bool empty() const { return area.empty(); }

    static double alphaAt(int i) { return -kHalf + i * kStep; }
    static double betaAt(int j) { return -kHalf + j * kStep; }

    /* AeroFrame's two angles come from atan2 over the SAME denominator, dot(v,
       nose), so a flow from behind the nose lands past +-90 in BOTH at once. Both
       angles only ever build a direction through tan() (see flowDirection), and tan
       has period 180, so re-expressing an angle by -+180 names the SAME flow line:
       the fold is exact, not an approximation. (The other symmetry, area(d) ==
       area(-d), is real but is not what this leans on -- a half turn in one angle
       is not a flow reversal.) Each angle wraps under ITS OWN sign, because that is
       what atan2 emitted: (-135, +157) wraps to (+45, -23) and both name one
       direction. Wrapping only alpha -- the bug this pins -- left beta at the clamp
       below, so a straight-tail flow keyed the straight-up cell: 9% off on a
       jet-shaped hull (tmp/review_20261028/probe_chart.cpp), and the error grows
       with how far the ship has yawed off its tail. Note it is NOT 180 - alpha,
       which would mirror the ship's left and right and charge a broadside for a
       quartering one. The clamps are insurance for hand-fed pairs only: one angle
       cannot cross the bound without the other, since they share a denominator. */
    static void fold(double &alpha, double &beta) {
        if(alpha > kFoldHalf) { alpha -= std::numbers::pi; }
        else if(alpha < -kFoldHalf) { alpha += std::numbers::pi; }
        if(beta > kFoldHalf) { beta -= std::numbers::pi; }
        else if(beta < -kFoldHalf) { beta += std::numbers::pi; }
        if(alpha > kFoldHalf) { alpha = kFoldHalf; }
        else if(alpha < -kFoldHalf) { alpha = -kFoldHalf; }
        if(beta > kFoldHalf) { beta = kFoldHalf; }
        else if(beta < -kFoldHalf) { beta = -kFoldHalf; }
    }

    // The cell index whose lower corner is at or below `angle` (angle folded).
    static int cellOf(double angle) {
        int i = (int)std::floor((angle + kHalf) / kStep);
        if(i < 0) { i = 0; }
        if(i > kSteps - 2) { i = kSteps - 2; }
        return i;
    }
    static double weight(double angle, int i0) {
        const double f = (angle + kHalf) / kStep - i0;
        if(f < 0.0) { return 0.0; }
        if(f > 1.0) { return 1.0; }
        return f;
    }
    // Inside the cached range (both angles already folded).
    static bool charted(double alpha, double beta) {
        return std::fabs(alpha) <= kHalf && std::fabs(beta) <= kHalf;
    }

    /* A hit: all four corners of the cell were filled at this generation, so
       bilinear between them. Anything else (no table, a NaN angle, a direction
       outside the cached range, a corner not yet filled) is a MISS and leaves
       `out` alone. */
    bool lookup(double alpha, double beta, double &out) const {
        if(area.empty() || !std::isfinite(alpha) || !std::isfinite(beta)) { return false; }
        fold(alpha, beta);
        if(!charted(alpha, beta)) { return false; }
        const size_t k00 = (size_t)cellOf(beta) * kSteps + cellOf(alpha);
        if(stamp[k00] != gen || stamp[k00 + 1] != gen
           || stamp[k00 + kSteps] != gen || stamp[k00 + kSteps + 1] != gen) {
            return false;
        }
        const int i0 = cellOf(alpha), j0 = cellOf(beta);
        const double wa = weight(alpha, i0), wb = weight(beta, j0);
        const double a00 = area[k00], a10 = area[k00 + 1];
        const double a01 = area[k00 + kSteps], a11 = area[k00 + kSteps + 1];
        out = (a00 * (1.0 - wa) + a10 * wa) * (1.0 - wb)
            + (a01 * (1.0 - wa) + a11 * wa) * wb;
        return true;
    }

    /* Fill this cell's missing corners, at most `budget` of them -- the caller
       spreads the cost over substeps. `exact(alpha, beta)` computes the area at
       a node; the caller holds the parts, the ship frame and the union. Returns
       how many nodes were filled. A direction outside the cached range fills
       nothing: it is answered by the caller's exact union instead. */
    template <class ExactFn>
    int fillAround(double alpha, double beta, int budget, ExactFn exact) {
        if(budget <= 0 || !std::isfinite(alpha) || !std::isfinite(beta)) { return 0; }
        fold(alpha, beta);
        if(!charted(alpha, beta)) { return 0; }
        /* The table appears the first time a ship actually asks for a direction.
           A ship that never flies in one never pays for it (7.7 kB each). */
        if(area.empty()) {
            area.assign(kSteps * kSteps, 0.0f);
            stamp.assign(kSteps * kSteps, 0u);
            if(gen == 0) { gen = 1; }
        }
        const int i0 = cellOf(alpha), j0 = cellOf(beta);
        int filled = 0;
        for(int dj = 0; dj <= 1; dj++) {
            for(int di = 0; di <= 1; di++) {
                if(budget <= 0) { return filled; }
                const int i = i0 + di, j = j0 + dj;
                const size_t k = (size_t)j * kSteps + i;
                if(stamp[k] == gen) { continue; }
                area[k] = (float)exact(alphaAt(i), betaAt(j));
                stamp[k] = gen;
                filled++;
                budget--;
            }
        }
        return filled;
    }
};

/* The drag that LIFT costs (#462): induced drag coefficient = CL^2 / (pi e AR),
   with AR = span^2 / planform area. Nothing charged for it before: a surface
   earned its lift free, paying only whatever blunt form drag its shape happened
   to draw. `span` must be the SPAN, not the thickness, so callers pass
   PartDef::meshSpanHalf() -- the widest extent across the stack axis, read off
   the mesh (#457). The authored radius gives the same number for every wing in
   this catalog only because they are all authored with their span along X.

   Callers pass the SPAN OF THE LIFTING SYSTEM, not of one panel (#556): AR is a
   property of the whole wing, because that is the width of the vortex sheet. Two
   co-planar panels bolted together are one 8 m AR-4 wing, not two AR-2 wings --
   the per-part version billed double the drag for a shape whose wake is one sheet.
   classifyLiftingSystems below is how a set of panels becomes a system. */
inline double inducedCd(double cl, double span, double area, double oswald = 0.8) {
    if(!(area > 0.0) || !(span > 0.0) || !(oswald > 0.0)) { return 0.0; }
    const double ar = (span * span) / area;
    if(!(ar > 0.0)) { return 0.0; }
    return (cl * cl) / (std::numbers::pi * oswald * ar);
}

/* ONE LIFTING PANE (#556): a surface's planform, placed in the airframe. The axes
   are the part's span and chord carried into ship frame (PartDef::meshSpanAxis(),
   localRot[2]), `spanHalf`/`chordHalf` off the same mesh box #457/#559 use, and
   `area` the planform the author declared. This is everything the grouping needs;
   the lift law's own numbers (cl, stall_angle) stay with the part, because two
   panels of one wing can legitimately differ. */
struct LiftingPane {
    glm::dvec3 spanAxis = glm::dvec3(1.0, 0.0, 0.0);
    glm::dvec3 chordAxis = glm::dvec3(0.0, 0.0, 1.0);
    glm::dvec3 mount = glm::dvec3(0.0);   // ship frame
    double spanHalf = 0.0;
    double chordHalf = 0.0;
    double area = 0.0;
};

/* ONE LIFTING SYSTEM (#556): the wing the panes make together. `span` is the
   projected width of their union along the airframe's lateral axis, `area` the sum
   of their planforms. AR = span^2/area, and that is the number both the lift slope
   and the induced bill want.

   `span` is the width of the lifting MATERIAL this model knows about -- the panels
   themselves. A gap bridged by a fuselage merges the panels into one system but is
   not counted as span, because the fuselage's own share of the planform is not in
   `area` either: b and S stay self-consistent. */
struct LiftingSystem {
    double span = 0.0;
    double area = 0.0;      // panes' planforms: lift_area OR control_area
    /* The part of `area` that is a LIFTING planform. 0 means the system is only
       steering surfaces -- a freestanding fin -- which is the case that must make
       its own side force from sideslip (#454's other half). A flap sitting on a
       wing is in a system with liftArea > 0: the wing already lifts from that
       angle, so the flap must not. */
    double liftArea = 0.0;
    double oswald = 0.8;   // the shipped value; per-part e would feed min() here
    int members = 0;
};

/* A lateral band occupied by a NON-LIFTING part -- a fuselage, a tank, a stack --
   and therefore the place where a wing panel can continue through structure the
   catalogue does not model as lifting. The shipped jets mount their wings on the
   hull's sides, roots 2 m apart, and they are one wing: this is that fact, read at
   the fuselage's own width instead of authored as a number. */
struct LiftingBridge {
    double lo = 0.0;
    double hi = 0.0;
};

/* GROUP CO-PLANAR PANELS INTO WINGS (#556). Two panes are one wing when, in ship
   frame:

     1. their spans run along the airframe's LATERAL axis (within kLiftAlignCos of
        it). That is what keeps a vertical fin out -- and it is measured on the
        PROJECTION, so a panel with a few degrees of dihedral still groups;
     2. their chords run fore/aft (within kLiftAlignCos of the reference axis);
     3. their span intervals, projected on the lateral axis, overlap, come within
        kLiftTouch of each other, OR are separated only by a `bridges` band -- a
        non-lifting part (the fuselage) whose width carries the wing through. And
        their chord intervals overlap or touch.

   Three is what tells a wing from a tailplane: same plane, same span, different
   chord station -> two systems, because a tail's wake is not the wing's. A plain
   flap hinged at the trailing edge touches in chord, so it merges with the wing it
   is part of. Touch, not "close enough": a visible gap that no structure fills is
   two wings, and the rule the player can see is the rule the law uses.

   Returns one system index per pane (empty input -> empty), and fills `systems`
   with the aggregates in the order the systems were first met. Transitive by
   construction (a chain of touching panels is one wing), O(n^2) on a handful of
   surfaces, and it runs where the geometry cache is rebuilt, not per substep. */
inline std::vector<int> classifyLiftingSystems(const std::vector<LiftingPane> &panes,
                                               const std::vector<LiftingBridge> &bridges,
                                               const glm::dvec3 &lateral,
                                               const glm::dvec3 &foreAft,
                                               std::vector<LiftingSystem> &systems) {
    /* cos 20 deg. Wide enough for a dihedral panel or a few degrees of sweep,
       narrow enough that a fin (90 deg off) never joins a wing. */
    constexpr double kLiftAlignCos = 0.9396926207859084;
    /* 1 cm: float error in a mount, or the hairline a builder leaves between two
       panels that are meant to touch. Plus 2% of the panels' own half-extent, which
       is what a DIHEDRAL break costs in projection: two 5-deg panels of a 2 m
       half-span have roots 1.5 cm apart in the lateral projection while being one
       continuous wing. So the rule is "touching, within the sagitta of the tilt we
       already allow" -- smooth in the tilt, and a deliberately visible slot still
       reads as two wings. */
    constexpr double kLiftTouch = 0.01;
    constexpr double kLiftTouchFrac = 0.02;

    const size_t n = panes.size();
    systems.clear();
    std::vector<int> id(n, -1);
    if(n == 0) { return id; }

    std::vector<size_t> parent(n);
    for(size_t i = 0; i < n; i++) { parent[i] = i; }
    const auto find = [&](size_t i) {
        while(parent[i] != i) { i = parent[i] = parent[parent[i]]; }
        return i;
    };
    const auto join = [&](size_t a, size_t b) {
        const size_t ra = find(a), rb = find(b);
        if(ra != rb) { parent[rb] = ra; }
    };
    struct Interval { double lo, hi; };
    /* Both helpers normalise the axis: PartDef rotations are unit by construction,
       but a test (or a future caller) may hand in a scaled direction, and a span is
       a length -- it must not scale with the vector it was measured along. */
    const auto interval = [](const LiftingPane &p, const glm::dvec3 &axis,
                             bool span) {
        const double L = glm::length(axis);
        const glm::dvec3 a = (L > 0.0) ? axis / L : glm::dvec3(0.0);
        const glm::dvec3 dir = span ? p.spanAxis : p.chordAxis;
        const double half = span ? p.spanHalf : p.chordHalf;
        const double c = glm::dot(p.mount, a);
        const double h = std::fabs(glm::dot(dir, a)) * half;
        return Interval{c - h, c + h};
    };
    const auto touches = [](Interval x, Interval y, double hx, double hy) {
        const double gap = kLiftTouch + kLiftTouchFrac * (hx + hy);
        return !(x.hi + gap < y.lo || y.hi + gap < x.lo);
    };
    /* The gap between two intervals (empty when they overlap), and whether some
       non-lifting part spans it. Min/max rather than ordering the pair: a caller
       never promises x is inboard of y. */
    const auto bridged = [&bridges](Interval x, Interval y) {
        const double lo = std::min(x.hi, y.hi);
        const double hi = std::max(x.lo, y.lo);
        if(!(hi > lo)) { return false; }   // no gap to bridge
        for(const LiftingBridge &b : bridges) {
            if(b.lo <= lo + kLiftTouch && b.hi >= hi - kLiftTouch) { return true; }
        }
        return false;
    };
    const auto aligned = [](const glm::dvec3 &v, const glm::dvec3 &axis) {
        const double L = glm::length(v) * glm::length(axis);
        return L > 0.0 && std::fabs(glm::dot(v, axis)) / L >= kLiftAlignCos;
    };

    for(size_t i = 0; i < n; i++) {
        const LiftingPane &a = panes[i];
        /* A pane whose span does not run across the airframe is not part of any
           wing -- a vertical fin, a canard mounted on its edge. It is still its own
           system (the induced bill has to charge it), it just never merges. */
        const bool aLateral = aligned(a.spanAxis, lateral) && aligned(a.chordAxis, foreAft);
        for(size_t j = i + 1; j < n; j++) {
            const LiftingPane &b = panes[j];
            const bool bLateral = aligned(b.spanAxis, lateral) && aligned(b.chordAxis, foreAft);
            if(!aLateral || !bLateral) { continue; }
            const Interval spanA = interval(a, lateral, true);
            const Interval spanB = interval(b, lateral, true);
            if(!touches(spanA, spanB, a.spanHalf, b.spanHalf)
               && !bridged(spanA, spanB)) {
                continue;
            }
            if(!touches(interval(a, foreAft, false), interval(b, foreAft, false),
                        a.chordHalf, b.chordHalf)) {
                continue;
            }
            join(i, j);
        }
    }

    /* Aggregate each root: area sum, and the span as the UNION of the projected
       intervals, so a hairline gap inside a merged wing is not counted as lift.
       The projection axis is the SYSTEM's: the airframe's lateral axis for a wing,
       and the panel's OWN span axis for anything that never merges (a vertical fin
       still bills its real span, it just has no neighbour to share it with -- its
       lateral projection is zero and would wrongly cancel its induced drag). */
    std::vector<size_t> rootToSystem(n, SIZE_MAX);
    std::vector<glm::dvec3> sysAxis;
    for(size_t i = 0; i < n; i++) {
        const size_t r = find(i);
        if(rootToSystem[r] == SIZE_MAX) {
            rootToSystem[r] = systems.size();
            systems.push_back(LiftingSystem{});
            const bool isLateral = aligned(panes[i].spanAxis, lateral) &&
                                   aligned(panes[i].chordAxis, foreAft);
            sysAxis.push_back(isLateral ? lateral : panes[i].spanAxis);
        }
        const int s = (int)rootToSystem[r];
        id[i] = s;
        systems[s].area += panes[i].area;
        systems[s].members++;
    }
    std::vector<std::vector<Interval>> spans(systems.size());
    for(size_t i = 0; i < n; i++) {
        spans[(size_t)id[i]].push_back(interval(panes[i], sysAxis[(size_t)id[i]], true));
    }
    for(size_t s = 0; s < spans.size(); s++) {
        std::vector<Interval> &v = spans[s];
        std::sort(v.begin(), v.end(),
                  [](Interval x, Interval y) { return x.lo < y.lo; });
        double total = 0.0;
        double lo = 0.0, hi = 0.0;
        bool open = false;
        for(Interval iv : v) {
            if(!open) { lo = iv.lo; hi = iv.hi; open = true; continue; }
            if(iv.lo <= hi) {
                hi = std::max(hi, iv.hi);
            } else {
                total += hi - lo;
                lo = iv.lo;
                hi = iv.hi;
            }
        }
        if(open) { total += hi - lo; }
        systems[s].span = total;
    }
    return id;
}

/* Which parts the hull's silhouette union may charge for the air it stops (#455).

   A part that declares an aerodynamic AREA is a SURFACE, not body, and pays its own
   bill at its own place. This is the line between the two area conventions that live
   in one catalog (and #478 asks for exactly this sentence, where the next author
   will look):

     BODY part  -- the union projects the part's MESH against the flow and charges
                   partCd on THAT projected area. The authored cd is a blunt-body
                   coefficient (capsule 0.35, tank 1.0): "how much air does this
                   shape stop".
     SURFACE    -- the PLANFORM the author declared (lift_area / control_area) times
                   its own shape drag, read at the flow the surface itself stands in:
                   chordwise that is the PROFILE cd the author declared (0.03 for
                   every shipped wing and fin); out of the plate's plane the surface
                   pays its planform as the plate it is; spanwise it pays its edge.
                   surfaceFormCdA below is that split.

   What #455 actually removed is the COEFFICIENT the projection was billed at. In the
   union a wing's sliver of silhouette was charged at the SHIP's area-weighted mean cd
   (~0.77 on the jet), so a wing paid a blunt body's shape drag on top of the induced
   drag its lift had already bought: ~15.5 m2 of flat plate for two wings earning 4.2
   of CLmax, which is the number that made the airframe's L/D unquotable. The momentum
   the surface turns downward is still charged exactly once, as inducedCd above.

   A part is one or the other, never both -- shipdef.cpp refuses a part carrying both
   lift_area and control_area. #551 wants THAT split gone (one fin, one law); it does
   not want this one gone with it. */
inline bool isAeroSurface(double liftArea, double controlArea) {
    return liftArea > 0.0 || controlArea > 0.0;
}

/* The per-part DRAG COEFFICIENT as the part faces the flow: a blend of the three
   declared anchors (nose / side / base) by c = cos(nose axis, flow):
     cd(c) = drag_side·(1−c²) + drag_forward·(max(c,0)²) + drag_backward·(max(−c,0)²)
   Convex blend: hits each anchor exactly, stays within the part's min..max cd. */
inline double partCd(double cdForward, double cdSide, double cdBackward,
                     double cosAxisFlow) {
    double c = cosAxisFlow;
    if(c > 1.0) { c = 1.0; }
    else if(c < -1.0) { c = -1.0; }
    const double c2 = c * c;
    const double cw = (c > 0.0) ? c2 : 0.0;   // forward weight (nose-in)
    const double bw = (c < 0.0) ? c2 : 0.0;   // backward weight (base-in)
    return cdSide * (1.0 - c2) + cdForward * cw + cdBackward * bw;
}

/* The shape drag of one SURFACE part (#455, split by #559). A plate meets the flow
   three ways and they are three different AREAS:

     chordwise   -> the section's PROFILE cd on the planform (0.03 x 8 m2 = 0.24 m2)
     out of plane-> the planform as a flat plate (1.0 x 8 m2): incidence, a tumble,
                    a wing seen from below
     spanwise    -> the plate's EDGE, thickness x chord (1.0 x 0.2 m2), blunt

   #455 billed the last two as one term on the planform, keyed on the chord axis
   alone, so a horizontal wing in sideslip paid 8 m2 of plate for the 0.1 m of
   aluminium it presents there: the 453 bed billed 15.6 m2 of CdA for two 8 m2 wings
   whose silhouette at Beta 64 deg is under 1 m2, which was most of the yaw damping
   that case pinned. The three weights are the squared cosines of the flow against
   the chord, the plate normal and the span, so they sum to 1 and no share of the
   flow is charged twice (or missed). cd_forward/cd_backward keep partCd's
   nose/base convention on the chordwise share; cd_side is the blunt coefficient for
   both out-of-plane and edge flow, which is what it already meant. */
inline double surfaceFormCdA(double planform, double edgeArea,
                             double cdForward, double cdSide, double cdBackward,
                             double cosChordFlow, double cosNormalFlow) {
    if(!(planform > 0.0)) { return 0.0; }
    double c = cosChordFlow;
    double n = cosNormalFlow;
    if(c > 1.0) { c = 1.0; } else if(c < -1.0) { c = -1.0; }
    if(n > 1.0) { n = 1.0; } else if(n < -1.0) { n = -1.0; }
    const double c2 = c * c;
    const double n2 = n * n;
    double s2 = 1.0 - c2 - n2;              // the spanwise share, by the identity
    if(!(s2 > 0.0)) { s2 = 0.0; }
    const double wf = (c > 0.0) ? c2 : 0.0;
    const double wb = (c < 0.0) ? c2 : 0.0;
    const double edge = (edgeArea > 0.0) ? edgeArea : 0.0;
    return planform * (cdForward * wf + cdBackward * wb + cdSide * n2)
         + edge * cdSide * s2;
}

/* The force on one DEFLECTED control surface: the lift law with the DEFLECTION
   in place of the angle of attack, through the SAME liftCurve a wing uses (#487's
   shape, authored here as the surface's own stall_angle). A plate turned 20 deg into
   the flow has separated: it makes a FRACTION of its peak side force, not the 2.1 an
   infinite-aspect 2-D airfoil would claim (cl 6.0/rad x 0.35 rad), which is what the
   law said while it had no stall term at all. Peak authority is at stall_angle and the
   force is GONE by twice that, so the loader refuses a travel stop at or past 2x the
   stall angle: past the zero the last of the stick's throw pulls the wrong way. A
   stall_angle ABOVE the travel stop is legal and simply leaves the surface linear over
   its whole travel -- and that is the case no loader check can catch, which is how a
   plane ends up with 2x the authority it was tuned for. The number is required on any
   steering part and there is deliberately NO default argument here, so a new call site
   cannot ask for the un-stalled law by omission. `cl` is the slope at the flow's MACH
   (liftSlopeMach), same as a wing's: a fin's authority moves with the air exactly as
   its lift does, which is why the flight loop corrects it once per surface rather than
   carrying a second Mach rule here. The steering moment is NOT here: it comes from
   applying this force AT the surface (Vehicle::applyAeroForce). */
inline glm::dvec3 controlForce(double q, double S, double cl, double delta,
                               const glm::dvec3 &dir, double stallAngle) {
    if(q <= 0.0 || S <= 0.0 || cl <= 0.0 || delta == 0.0) { return glm::dvec3(0.0); }
    return dir * (q * S * liftCurve(cl, delta, stallAngle));
}

/* The control-surface DEFLECTION EFFECTIVENESS: the part's dedicated cl_control
   if set (>0), else its lift-curve slope cl. */
inline double controlCl(double cl, double clControl) {
    return (clControl > 0.0) ? clControl : cl;
}

/* The CONTROL COMMAND (#492): how far a surface travels for a stick at `stick`
   (units of FULL travel, so -1..+1; the shipped keys are binary +-1) holding a
   standing `trim` in the same units. Trim joins the COMMAND rather than the mount,
   which is what makes it come out right on its own: controlDeflectionSign resolves
   the sign per instance, so a mirrored elevator pair at clock 0/180 and a canard
   forward of the CG all take the SAME trim number and still torque the same way --
   exactly as they do for the stick today. (A mount rotation, which is what shipdef
   "roll" is, would need the opposite sign on each half of a mirrored pair: fine for
   a builder who sees the mesh, wrong for a slider.)
   The sum is clamped to +-1 because a surface cannot travel past its own stop no
   matter how the player holds the stick and sets the trim at once. Zero command is
   exactly zero: the caller skips the whole surface on that test, so an unset trim
   must not shift it (0.0 + 0.0 is exact here -- and -0.0 == 0.0). */
inline double controlCommandFrac(double stick, double trimFrac) {
    const double c = stick + trimFrac;
    return (c > 1.0) ? 1.0 : (c < -1.0) ? -1.0 : c;
}

/* The OXIDIZER half of the jet gate, on its own: how much of a rated intake's oxidizer
   MASS this air can feed a burner, in [0, 1], whatever the density. A turbine does not
   burn "air", it burns fuel against the oxidizer dissolved in it, so a world can be at
   full sea-level density and still be a dead engine: Eve's 1.7 kg/m3 and Duna's 0.12
   are CO2, Jool's 2.0 is H2 and He -- H2 is the fuel, and there is nothing here to burn
   it with. oxidizer_share (kilograms of oxidizer per kilogram of air) comes off the
   body's authored composition via atmo.h:resolveMix, which is the only reason the game
   can say so. Richer-than-dry-air gets 1, not more: extra oxygen does not make a fan
   push harder. Fails CLOSED on NaN or a share of 0. */
inline double jetOxGate(double oxidizer_share) {
    const double rated = dryAirOxidizerShare();
    if(!(oxidizer_share > 0.0) || !(rated > 0.0)) { return 0.0; }  // NaN fails closed
    const double g = oxidizer_share / rated;
    return (g > 1.0) ? 1.0 : g;
}

/* The jet AIR GATE shared by thrust and fuel flow: the share of its intake an
   air-breathing engine can actually use, in [0, 1]. Two limits, and the engine is
   whichever of them runs out FIRST -- min, not a product, because on an oxygen-world
   they fall together and multiplying them would bill thin air twice:

     the air itself:  rho / kAirSeaLevelDensity, capped at 1. An intake swallows a
       VOLUME per second, so the kilograms it moves -- and therefore the fan thrust
       and the fuel it can inject with them -- are set by absolute intake density.
       The reference is the machine's rating condition (constants.h), not the body's
       own sea level: until #630 this divided by rho_sea, so a jet standing on Mars
       (0.02 kg/m3 of CO2) ran its FULL authored fan thrust, because 0.02 is 1.0 of
       Mars. That is the bug the second limit below makes impossible to miss.

     the oxidizer:  rho x oxidizer_share, against the same rating density times dry
       air's oxidizer share (0.231 kg of O2 per kg of air). A turbine does not burn
       "air", it burns fuel against the oxidizer IN the air, so a world can be at
       full sea-level density and still be a dead engine: Eve's 1.7 kg/m3 and Duna's
       0.12 are CO2, Jool's 2.0 is H2 and He -- H2 is the fuel, and there is nothing
       here to burn it with. oxidizer_share comes off the body's authored composition
       (atmo.h:resolveMix, terragen.h), which is the only reason the game can say so.

   Both halves fail CLOSED on NaN or a non-positive input: no air, no oxidizer, no
   thrust and no burn. A jet is the only engine in the game whose thrust depends on
   the world it is standing on, so this is where that dependence lives. */
inline double jetAirGate(double rho, double oxidizer_share) {
    const double oxg = jetOxGate(oxidizer_share);
    if(oxg <= 0.0) { return 0.0; }              // no oxidizer to burn against
    if(!(rho > 0.0)) { return 0.0; }            // NaN or vacuum fails closed
    const double air = (rho >= kAirSeaLevelDensity) ? 1.0
                                                    : rho / kAirSeaLevelDensity;
    const double ox = (rho / kAirSeaLevelDensity) * oxg;
    return (ox < air) ? ox : air;
}

/* The same gate from the AIR ITSELF: the profile carries the oxidizer share, so a
   caller that has read the density under a hull passes the thing that density came
   from and cannot pair one body's rho with another body's composition (#637). The
   two-argument form above stays the law -- it is pure math, it is what test_jet drives
   by sweeping density and share, and a DragAtmosphere cannot express a NaN share or a
   negative one as neatly. This is the door the sim calls through. */
inline double jetAirGate(const DragAtmosphere &a, double rho) {
    return jetAirGate(rho, a.oxidizer_share);
}

/* JET ENGINE (air-breathing) thrust, newtons:
     T(v, rho) = T_fan·d  +  ṁ_f·v_e·d  +  jetOxGate·rho·A·v·(v_e − v)
                                            d = jetAirGate(rho, oxidizer_share)
   d gates the fan + fuel terms on the usable intake. The ram term is the air the
   intake captures (rho·A·v) thrown out the back at v_e, so it scales with the raw
   density the hull is moving through -- NOT with d, which would bill thin air twice
   and quietly halve Kerbin cruise thrust at altitude.
   It is still NOT free: the momentum has to come from somewhere, and the only source
   is fuel burned against this air's oxidizer, so jetOxGate -- the oxidizer half of the
   gate on its own, without the density term the rho already carries -- pays for it.
   Without that the law contradicts itself on a CO2 world: Mars (0.02 kg/m3, 0.1 % of
   its air is O2) would bill the fan 6 N and hand back 530 N of ram for a fuel flow the
   gate has already cut to 8e-5 kg/s, and Eve's 1.7 kg/m3 of CO2 would be a full-power
   engine at the exact moment its fan goes dead. An intake that cannot burn is a hole,
   not an engine. */
inline double jetThrust(double v, double rho, double oxidizer_share,
                        double T_fan, double m_f, double v_e, double A) {
    const double d = jetAirGate(rho, oxidizer_share);
    if(d <= 0.0) { return 0.0; }   /* no usable air -> no thrust */
    const double speed = (v < 0.0) ? 0.0 : v;
    const double T = T_fan * d + m_f * v_e * d
                     + jetOxGate(oxidizer_share) * rho * A * speed * (v_e - speed);
    return (T < 0.0) ? 0.0 : T;
}

/* The same engine under the same air, asked through the profile (#637): the share of
   this air a turbine can burn against comes from the profile the density came from,
   so thrust and fuel flow cannot be gated on two different worlds. */
inline double jetThrust(double v, double rho, const DragAtmosphere &a,
                        double T_fan, double m_f, double v_e, double A) {
    return jetThrust(v, rho, a.oxidizer_share, T_fan, m_f, v_e, A);
}

/* The control-surface DEFLECTION SIGN for a surface at `ri` (rel. to the COM). The
   torque is ri x F, so a tail (behind the CG) and a canard (ahead) need OPPOSITE
   deflections for the same torque; this matches the reaction wheel's convention
   either way. `targetSign` is the SIGN of the wheel's torque about that axis for a
   positive stick (pitch/yaw = -1, roll = +1).
   The push direction is the MOUNT's own -- a plate pushes along its normal (mount[1])
   at zero angle of attack, which is also the axis partAero's liftDir is measured
   from and the hinge line drag.h::controlSurfaceVisualRot turns. So the rigging is a
   build fact and the flow does not get a vote (#495). That is why this takes the mount
   matrix rather than a direction: handed the LIVE liftDir, the leverage below can
   change sign mid-flight, which reverses the travel -- and with it the whole force, at
   whatever q the ship is flying -- and promises the stick a torque about an axis the
   surface has no reliable authority over. tests/test_drag.cpp::test_control_rigging
   pins both halves. It assumes a proper rotation (det +1), which is what every
   localRot is built from; a reflected mount would quietly invert the rigging. */
inline double controlRigLever(const glm::dvec3 &ri, const glm::dmat3 &mount,
                              const glm::dvec3 &aboutAxis) {
    return glm::dot(glm::cross(ri, mount[1]), aboutAxis);
}

inline double controlDeflectionSign(double rigLever, double targetSign = -1.0) {
    return (rigLever >= 0.0) ? targetSign : -targetSign;
}

/* HOW FAR A COMMAND TAKES one control surface, in rad, and whether it takes it at
   all. `cmdFrac` is the signed command fraction (controlCommandFrac: stick + trim,
   clamped to +-1) and `maxDefl` the part's travel stop. This is the whole decision the
   aero step makes about travel -- frame, rig sign, degeneracy, travel -- so that the
   wiring is the thing under test rather than a call site nobody pins
   (tests/test_drag.cpp::test_control_rigging).
   An unusable flow frame (pa.valid, #469) asks for no travel: there is no force to
   point, so moving the plate only moves the ship once the frame comes back.
   A surface whose ZERO-ALPHA push line runs through the pivot (rig lever ~0 against
   its own mount distance -- a fin turned to the wrong clock, a rudder mounted inline
   with the flow, a steering part sitting on the COM) is ALSO asked for NO TRAVEL.
   Being precise about why: the rig cannot decide WHICH WAY it would push that axis,
   so a commanded travel has no meaning; actuating it anyway is worse than dead,
   because the force is still q*S*cl*defl and its direction is whatever the flow says,
   so the airframe pays a real moment whose sign the builder never asked for. Measured,
   a skiff with its rudder at clock 0: the rudder sat at its +20.1 deg stop and the
   airframe peaked at |tau| = 2560 N m, above the 1539 N m of the same ship rigged
   properly; asked for no travel the peak is 246 N m. Note what that also means: such a
   surface now bills NO control force and NO induced bill, so a mis-rigged airframe
   flies cleaner than its geometry says. Dead-and-invisible beats random-and-loud, and
   telling the builder is the load-time check #509.
   The cutoff is a degeneracy test on the GEOMETRY, not a threshold on what counts as
   weak: |lever|^2 <= 1e-12 * |ri|^2, i.e. 1e-6 of the mount distance. The shipped
   surfaces sit 3.18 m (skiff rudder, hung off the fin's trailing edge by #570), 2.96 m
   (its elevators) and 6.56 m (its ailerons, out on the span) from the COM, so their
   cutoff is 3.0-6.6 um of lever; a builder with a real lever of even 1 mm gets full
   travel.
   test_control_rigging pins the SCALING -- a 3 m arm tilted 1e-4 rad off the degenerate
   line has 3e-4 m of leverage (100x that arm's cutoff) and travels fully, the same tilt
   on a 3 mm arm has 3e-7 m and still travels (an absolute floor would have gated it),
   and a tilt of 1e-8 rad is degenerate. */
inline double controlDeflection(const PartAero &pa, const glm::dvec3 &ri,
                                const glm::dmat3 &mount, const glm::dvec3 &aboutAxis,
                                double targetSign, double cmdFrac, double maxDefl) {
    if(!pa.valid) { return 0.0; }
    const double lever = controlRigLever(ri, mount, aboutAxis);
    /* Negated so a NaN fails CLOSED: `lever*lever <= k` is false when lever or ri is
       NaN, which would hand full travel to a rig whose geometry is garbage. The
       squared form keeps the sqrt off the hot path, and ri == 0 (a steering part on
       the COM) lands on the degenerate side either way, which is the answer we want. */
    if(!(lever * lever > 1e-12 * glm::dot(ri, ri))) { return 0.0; }
    return controlDeflectionSign(lever, targetSign) * cmdFrac * maxDefl;
}

/* #426: the surface as an ACTUATOR ROW (actuators.h) -- the torque one radian of
   this surface's COMMAND buys, N m/rad, pointed the way a POSITIVE command pushes and
   limited by its own travel. controlForce pushes along the plate's own zero-alpha
   normal (`pushDir`, the mount fact #495 chose over the live flow) and the steering
   moment is that force AT the surface, so the row is ri x pushDir scaled by q*S*cl --
   the same arithmetic the force law bills, stated as a derivative instead of a force.
   `cl` therefore means the same number it means there: the authored slope corrected to
   the flow's Mach (liftSlopeMach), which is how #487's rate damping inherits
   compressibility instead of carrying its own constant (#543 A5).
   It is the SAME SIGN and the SAME degeneracy test Vehicle::applyAeroForce applies to
   the travel (controlRigLever / controlDeflectionSign just above), and that is the
   whole point of putting them next to each other: a rig that cannot be commanded buys
   no authority in the budget either, and a positive setpoint means a positive COMMAND
   whichever way the plate is bolted. So the degeneracy test is on the lever PROJECTED
   ON THE AUTHORED CONTROL AXIS (`aboutAxis`), not on the raw cross product: the raw one
   may well lever some other axis, and the mix credits that (see the returned VECTOR
   below), but a rig whose push decides nothing about the axis it is rigged for cannot
   be commanded at all -- which is #495's rule, unchanged.
   What they cannot share is the FLOW. controlDeflection also refuses when partAero
   finds no usable frame at the part (a plate flying at exactly zero alpha has no lift
   axis), and a row built from the mount alone cannot see that; it sees the airspeed the
   caller measured, which is why the caller has to do the seeing (q > 0, and --drag-cd,
   which airDensityAtCom() ignores).
   The returned vector is the raw cross product, NOT projected: this row levers whatever
   it physically levers, so a plate at 45 deg clock levers pitch and yaw at once and
   actuatorSolve can spend it either way. Only the SENSE is the authored axis's.
   Two deliberate simplifications, both budget-not-delivery and both named:
   - the LINEAR slope. Past stall_angle the force law droops (#494), so a surface
     worked near or past its stall delivers LESS than this says; the delivered number
     is what the caller bills, as in #599 for the wheels.
   - the COM's q, not the surface's local flow. A turning hull sweeps its surfaces
     (#483: 0.93-1.07 of the COM's q on the jet), which is exactly what makes a ring's
     two halves unequal -- a difference this budget does not see and the force law does.
   A zero row also keeps actuators.h honest: it divides a line's share by the row's
   length, so a hair-length row would be asked for a near-infinite deflection. */
inline glm::dvec3 surfaceTorquePerRadian(double q, double S, double cl,
                                         const glm::dvec3 &ri,
                                         const glm::dvec3 &pushDir,
                                         const glm::dvec3 &aboutAxis,
                                         double targetSign = -1.0) {
    if(q <= 0.0 || S <= 0.0 || cl <= 0.0) { return glm::dvec3(0.0); }
    const double lever = glm::dot(glm::cross(ri, pushDir), aboutAxis);
    if(!(lever * lever > 1e-12 * glm::dot(ri, ri))) { return glm::dvec3(0.0); }
    return controlDeflectionSign(lever, targetSign) * glm::cross(ri, pushDir)
         * (q * S * cl);
}

/* WHICH SHIP AXIS a control-surface readout is quoted against, and which way that
   surface's own rigging pushes along it (#495 review). Both answers come from how the part
   is bolted on, so ASK THEM IN THE BODY FRAME -- the part's own mount against the root
   part's frame, both fixed at load -- and they are per-part CONSTANTS. Asked in the world
   frame they are per-substep questions: over a 6-degree grid of 9300 attitudes, a mount
   exactly 45 deg out of vertical has no clear winner (its two dots differ by 1e-16 of
   rounding) and the answer changes with attitude on 1578 of those attitudes, while a mount
   off the tie never flips. Pinned in test_surface_alpha_readout, which pins both framings
   so nobody "simplifies" the call site back.
   Tie-break goes to UP, the conventional pitch/roll readout. rigSign is the mount's own
   push projected on the chosen axis: the readout uses it ONLY where the wind offers no
   sign at all (a surface exactly broadside to its own flow), and a mirrored pair's rigSign
   and alpha flip together, which is what keeps such a pair printing one number. A mount
   pointing along the ship's nose projects on neither axis (rigSign 0); such a part has no
   readout worth quoting and is what #509 flags. */
struct SurfaceReadout {
    bool axisUp;        // quote against the ship's up (true) or its right (false)
    double rigSign;     // the mount's own push along that axis (a build fact)
};

inline SurfaceReadout surfaceReadoutFrame(const glm::dvec3 &rigPush,
                                          const glm::dvec3 &bodyUp,
                                          const glm::dvec3 &bodyRight) {
    const double du = glm::dot(rigPush, bodyUp), dr = glm::dot(rigPush, bodyRight);
    const bool axisUp = std::fabs(du) >= std::fabs(dr);
    return { axisUp, axisUp ? du : dr };
}

/* #485: a surface's angle of attack, signed into SHIP terms for a READOUT
   (degrees). partAero measures alpha in the surface's own mount frame, and a twin
   pair mounted at clock angles 0 and 180 has its face normal flipped, so the sign
   convention flips with it. The law never notices -- sin(alpha) and liftDir flip
   together, which is exactly how a mirrored fin pair pushes the same way (#453) --
   but a readout that quotes +2.6 and -2.6 for two wings at the same physical angle
   is worse than no readout: it hides the one thing worth seeing, a pair whose
   angles actually DIFFER (a yawing airframe sweeps one half into the flow and the
   other out of it, #483).
   THREE INPUTS, each from the place that owns it (surfaceReadoutFrame above supplies the
   first two, the flow supplies the third):
   - WHICH SHIP AXIS to quote against: the MOUNT's. Take it from the live liftDir instead
     and the axis wanders -- over an 810-direction flow sweep on a mount 45 deg out of
     vertical the live direction picks the other axis in 459 of the flows.
   - which way that axis runs for the rig: the MOUNT's, and only a fallback (below).
   - WHICH WAY ALONG the axis the angle points: the WIND's, from the surface's own LIVE
     force projected on the axis -- the direction controlForce bills along and the F4
     overlay draws an arrow along (#469). Sign it off the rig instead and the number
     disagrees with the force the law actually bills: 284 of those 810 flows on a v-tail
     mount, 64 of them with a non-zero force to disagree with (at |alpha| 0.7-28.9 deg,
     well before the droop completes) and the other 220 past twice the stall angle, where
     liftCurve answers exactly zero so there is no force and no arrow -- that half is
     #508's corner, not this one.
   A mirrored pair reads alike either way, because its alpha and its liftDir flip together
   -- #453's identity -- while its mount picks the same axis for both halves.
   A mount EXACTLY 45 deg out of vertical ties the axis choice, where the tie-break is a
   real decision rather than a rounding detail: for one half of a +-45 v-tail the force
   projects oppositely on the two ship axes, so quoting it against RIGHT rather than UP
   flips the printed sign. With the tie-break fixed in the body frame the two halves print
   the SAME number (measured: +7.107 both). Shipped mounts sit at 0/90/180/270 and never
   come near the tie.
   Display arithmetic; nothing in the law calls this. */
inline double surfaceAlphaDegForReadout(const PartAero &pa, const SurfaceReadout &rf,
                                        const glm::dvec3 &up, const glm::dvec3 &right) {
    const double along = glm::dot(pa.liftDir, rf.axisUp ? up : right);
    /* along == 0: the live force has no component along the axis being quoted, so the wind
       offers no sign. Fall back to the mount's own convention -- alpha is measured against
       the plate's normal, so the pair (rigSign, alpha) is the rig's answer, and it keeps a
       mirrored pair agreeing -- and keep the magnitude either way. Returning 0.0 would
       invent an angle: a fin mounted fore-aft in a dead-ahead flow sits at 90 deg of its
       own incidence, and quoting 0.0 for it is a lie of the same shape as the ones this
       line of work keeps finding. Note this is NOT controlDeflection's gate firing: that
       one is a build-fact lever about the pivot and never fires on a v-tail half, which
       reaches here with a full-size lever.
       pa.liftDir is unit length whenever pa.valid (partAero refuses a NaN axis too), so a
       zero here is a zero component, not a zero vector. */
    if(along == 0.0) { return (rf.rigSign >= 0.0 ? 1.0 : -1.0) * glm::degrees(pa.alpha); }
    return (along > 0.0 ? 1.0 : -1.0) * glm::degrees(pa.alpha);
}

/* The VISUAL hinge of a control-surface mesh at commanded deflection defl
   (signed rad, the control law's sign; Vehicle::Draw only). ONE rule, whatever
   axis the surface steers: a surface hinges along its own SPAN. A surface mesh
   is authored as a plate -- span along the part's +X, plate normal +Y, chord
   +Z (fin_r1h2.obj: 2 m span, 0.1 m thick, 2 m chord) -- so the hinge axis is
   partRot[0] and nothing else. Rotating the chord (partRot[2]) by t about
   partRot[0] moves its tip along -t * partRot[1] (col0 x col2 = -col1), and the
   law pushes along the SURFACE's own normal with the sign of defl (controlForce
   = defl * partAero(...).liftDir), so t = -defl tips the mesh the way its own
   force pushes (#453).
   This is also why the torque axis (ControlAxisParams::aboutAxis) is NOT the
   hinge axis and has no business here: it names the SHIP axis the stick promises
   to turn, and controlDeflectionSign resolves the per-instance sign -- left and
   right surfaces get opposite defl (differential aileron flapping for free), and
   a mirrored mount, whose span is reversed along with its defl, gets the same
   world-space swing as its upright twin. A v-tail fin hinges on its own slanted
   span, which a ship-frame hinge could not express. */
inline glm::dquat controlSurfaceVisualRot(double defl, const glm::dmat3 &partRot) {
    return glm::angleAxis(-defl, partRot[0]);
}
