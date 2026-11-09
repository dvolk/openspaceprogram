#!/usr/bin/env python3
"""ground_model.py -- the ONE spelling of the game's surface temperature law (#724 rung C).

    T(lat, lon, t) = T0 * [ f + (1 - f) * 4 * cos_inc ] ** (1/4)

  T0      [K]  the body's ANCHOR: the surface temperature the file states. For a body the
               corpus leaves without one, the black-body its starlight and albedo imply
               (#687's arithmetic, already in the dossier) -- and the generator prints that
               anchor as DERIVED, because "nobody stated a ground temperature" and "the
               ground is the equilibrium value" are different claims (#722).
  f       [1]  the EVEN SHARE: what fraction of the surface's own emission is spread evenly
               rather than falling where the star lights it. f = 1 is a uniform surface --
               which is what the RSS sheet states for Venus's ground, "Diurnal temperature
               range: ~0"; f = 0 is local radiative balance, an airless rock under a star.
  cos_inc [1]  the floored cosine of the star's elevation, from solar.h / insolation.h --
               not a second cosine.

WHY THE ANCHOR IS EXACT RATHER THAN A FUDGE TERM. Averaging T^4 over the sphere:

    <T^4> = T0^4 * ( f + (1 - f) * 4 * <cos_inc> ),      <cos_inc> = 1/4 exactly

because the lit hemisphere's integral of cos(theta) over the whole sphere's 4 pi is pi/4.
So <T^4> = T0^4 for EVERY f: the ground radiates exactly the power the star delivers,
whatever the swing, and no fudge constant is anywhere in sight. The energy anchor is what
"the global mean stays exactly T0" has to mean -- the mean of T, unlike the mean of T^4, is
not conserved by any choice of f, and the difference is the whole reason a black-body
temperature and a weather-report temperature are different numbers (Earth: 255 K against
288 K, the 33 K the game does not model).

ONE KNOB, SO A STATED PAIR IS A TEST AND NOT A FIT. With a = (T_night/T0)^4, the day end is
forced to (T_day/T0)^4 = 4 - 3a. So a corpus that states BOTH ends of a swing is giving an
over-determined claim, and the disagreement is information: fit the knob on the night end
(which the model gets structurally right -- it IS the anti-solar floor) and predict the day
end. See fit_even_share for what that comparison has to say about our sources.
"""

import math
import re
import sys

# [1] How far the PREDICTED day end may sit from a stated one before this module refuses to
# author the swing. Not a physics tolerance: the model's day end is the SUBSOLAR point, the
# hottest place a body can have, while a fact sheet's maximum is a station's afternoon at
# one latitude and season (Mars's is explicitly "Viking 1 Lander site"). Comparing those is
# comparing two different places, so a fit is authored only while the two agree to within
# this share of T0. Bodies outside it are printed with their residual and left without a
# swing, which the dossier then dashes (#727).
DAY_PREDICTION_GATE = 0.08

# [1] The default when a body states no diurnal range at all: no swing, a uniform surface at
# the anchor. The behaviour is Venus's (isothermal) on purpose -- the AIR mass decides that
# -- but the STATE is kept apart: a body that authors f prints it, a body that does not
# dashes the row, so "the source says the range is zero" and "the source says nothing" stay
# two different facts (#726/#727's law, which a shared default value erases only if the
# presence of the number is not also visible).
EVEN_SHARE_NO_SWING = 1.0


# [kPa] "Thick enough that a bare-rock swing is a contradiction." Chosen at the order of
# Earth's sea level: a body at or above that is not an airless rock whatever its source's
# night number says. This is a printed flag, not a refusal -- the pair and the air are two
# sources making two claims, and the file keeps both visible.
THICK_AIR_KPA = 50.0


def even_share_air_note(name, f, sea_level_pa):
    """[] The sentence for a pair that implies an almost un-redistributed surface under a
    dense sky -- Eve's 160 K night under a thousand kilopascals, for instance. Empty when
    there is nothing to flag."""
    if f is None or f >= 0.5 or not sea_level_pa:
        return ''
    if sea_level_pa / 1000.0 < THICK_AIR_KPA:
        return ''
    return ("NOTE %s: this pair implies an almost UNredistributed surface (f = %.3f) under "
            "a %.0f kPa sky -- the pair and this body's own air argue"
            % (name, f, sea_level_pa / 1000.0))


def night_T(T0, f):
    """[K] The model's global floor: the anti-solar point, cos_inc = 0."""
    assert T0 > 0.0 and 0.0 <= f <= 1.0
    return T0 * f ** 0.25


def day_T(T0, f):
    """[K] The model's global ceiling: the subsolar point, cos_inc = 1."""
    assert T0 > 0.0 and 0.0 <= f <= 1.0
    return T0 * (4.0 - 3.0 * f) ** 0.25


def even_share_from_night(T0, T_night):
    """[1] The knob, read off the night end alone. Inverting the DAY end instead would be a
    different answer whenever the pair is not consistent with the anchor, and this way the
    disagreement lands in the printed residual rather than in the authored number."""
    assert T0 > 0.0 and T_night >= 0.0
    return (T_night / T0) ** 4.0


def fit_even_share(T0, T_night, T_day, gate=DAY_PREDICTION_GATE):
    """Fit one knob on a stated (night, day) pair and grade it against the pair.

    Returns (f, note). f is None when the swing should NOT be authored: the ends are the
    wrong way round, the pair implies no room to swing, or the predicted day end misses by
    more than `gate` * T0. The note is written to be printed as the generator's verdict, so
    it names the residual rather than hiding a bool."""
    if T0 <= 0.0 or T_night < 0.0 or T_day < T_night:
        return None, "the pair is not a pair (night %s > day %s at T0 %s)" % (
            T_night, T_day, T0)
    if T_day == T_night:
        return EVEN_SHARE_NO_SWING, "the pair states NO range -> f = 1.000 (stated, not defaulted)"
    f = even_share_from_night(T0, T_night)
    if f > 1.0:
        return None, "the night end sits above the anchor: f would be %.3f" % f
    pred = day_T(T0, f)
    resid = pred - T_day
    verdict = "AUTHOR" if abs(resid) <= gate * T0 else "REFUSE"
    note = ("f = %.4f from the night end; model %.1f-%.1f K vs stated %.1f-%.1f K "
            "(day end %+0.1f K = %+.1f%% of T0) -> %s"
            % (f, night_T(T0, f), pred, T_night, T_day, resid,
               100.0 * resid / T0, verdict))
    if verdict == "REFUSE":
        return None, note
    return f, note


def parse_diurnal_pair(text):
    """[(K, K) or None] The RSS sheets' "Diurnal temperature range" row, whose spellings
    vary: "283 K to 293 K (10 to 20 C)", "95 K to 390 K (~ -290 F to +240 F)", and Venus's
    bare "~0". Two kelvin ends become a pair; a single small number becomes a zero range
    (a stated NO range, which is data, not a blank); anything else becomes None."""
    if not text:
        return None
    kels = [float(m.replace(",", "")) for m in
            re.findall(r"(\d[\d,.]*)\s*K\b", text)]
    kels = [k for k in kels if 1.0 < k < 3000.0]     # kelvin ends, not unit noise
    if len(kels) >= 2:
        return (min(kels[:2]), max(kels[:2]))
    if len(kels) == 1:
        return (kels[0], kels[0])
    m = re.search(r"~?\s*0\b", text)
    if m:
        return (0.0, 0.0)                             # "Diurnal range: ~0" -- stated zero
    return None


def anchor_note(stated_T0, derived_T0):
    """[] The one-line provenance the print needs: whose number is the anchor."""
    if stated_T0 is not None:
        return "anchor: the sheet's stated average %.2f K" % stated_T0
    return ("anchor: DERIVED black-body %.2f K -- this sheet states no ground temperature, "
            "so the equilibrium is what the starlight and albedo give (#722)"
            % derived_T0)


def _self_test():
    bad = 0

    def check(cond, msg):
        nonlocal bad
        if not cond:
            bad += 1
            print("FAIL: %s" % msg)

    # The anchor, by quadrature rather than by the closed form: whatever f is, the sphere
    # radiates T0. A wrong 1/4, a wrong exponent or a ramp spelled with cos instead of
    # cos^4 all move this.
    for f in (0.0, 0.3, 0.5, 0.9, 1.0):
        n, s = 400000, 0.0
        for i in range(n):
            th = math.pi * (i + 0.5) / n
            c = math.cos(th)
            t4 = f + (1.0 - f) * 4.0 * max(0.0, c)
            s += t4 * math.sin(th)
        mean = 0.5 * (math.pi / n) * s      # <X> = (1/2) int X sin th dth
        check(abs(mean - 1.0) < 2e-5, "area-mean of T^4 is T0^4 at f=%.1f (got %.6f)" % (f, mean))
    # <cos_inc> = 1/4 exactly, the identity the anchor rests on.
    n, s = 400000, 0.0
    for i in range(n):
        th = math.pi * (i + 0.5) / n
        s += max(0.0, math.cos(th)) * math.sin(th)
    cmean = 0.5 * (math.pi / n) * s
    check(abs(cmean - 0.25) < 1e-5, "<cos_inc> over the sphere is 1/4 (got %.6f)" % cmean)

    # The forced relation between the two ends -- one knob means a pair over-determines it.
    for T0, f in ((288.0, 0.95), (270.4, 0.015), (151.43, 0.5)):
        a = (night_T(T0, f) / T0) ** 4
        check(abs((day_T(T0, f) / T0) ** 4 - (4.0 - 3.0 * a)) < 1e-9,
              "day end is forced: (Tday/T0)^4 = 4 - 3a at f=%.3f" % f)
    check(night_T(288.0, 1.0) == day_T(288.0, 1.0), "f = 1 is a uniform surface")
    check(abs(day_T(270.4, 0.0) / 270.4 - 4.0 ** 0.25) < 1e-12,
          "f = 0 is the local-balance subsolar point, 4^1/4 T0")

    # The corpus's own rows, through the parser and the fit. These are the numbers the
    # sheets print; if RSS wording moves, these fail and the generator's verdict moves with
    # it visibly rather than silently.
    check(parse_diurnal_pair("283 K to 293 K (10 to 20 C)") == (283.0, 293.0),
          "Earth's pair parses")
    check(parse_diurnal_pair("95 K to 390 K (~ -290 F to +240 F)") == (95.0, 390.0),
          "the Moon's pair parses")
    check(parse_diurnal_pair("~0") == (0.0, 0.0), "Venus's stated no-range parses as a pair")
    check(parse_diurnal_pair("") is None and parse_diurnal_pair(None) is None,
          "an absent row stays absent rather than a zero")
    f, note = fit_even_share(270.4, 95.0, 390.0)
    check(f is not None and abs(night_T(270.4, f) - 95.0) < 1e-6,
          "the Moon's swing is authored from its night end: " + note)
    check(f is not None and day_T(270.4, f) > 370.0,
          "and its predicted day end is near the lunar noon: " + note)
    f2, note2 = fit_even_share(288.0, 283.0, 293.0)
    check(f2 is not None and f2 > 0.9, "Earth's thin swing is authored: " + note2)
    f3, note3 = fit_even_share(214.0, 184.0, 242.0)
    check(f3 is None, "Mars's lander-site pair is refused, not swallowed: " + note3)
    f4, note4 = fit_even_share(288.15, 186.95, 288.15)
    check(f4 is None, "a pair whose day end IS the anchor cannot set a swing: " + note4)
    f5, note5 = fit_even_share(737.0, 0.0, 0.0)
    check(f5 == EVEN_SHARE_NO_SWING, "a stated zero range authors f = 1: " + note5)
    check(fit_even_share(200.0, 250.0, 100.0)[0] is None,
          "a night end above the anchor is refused")

    check(even_share_air_note("Eve", 0.09, 1013000.0) != "",
          "a bare-rock swing under a thousand kPa gets flagged")
    check(even_share_air_note("Earth", 0.93, 101325.0) == "",
          "a damped surface under the same sky says nothing")
    check(even_share_air_note("Mun", 0.0, None) == "",
          "no air, no note")
    print("ground_model self-test: %s" % ("OK" if bad == 0 else "%d FAILURES" % bad))
    return 1 if bad else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(_self_test())
    print(__doc__)
