// fmt.h -- UI formatting helpers: header-only, buffer-fill (the no-alloc
// guarantee is by construction). For HUMAN readouts; instrument logs stay raw
// SI for the e2e CHECK harness.

#pragma once

#include <cstddef>
#include <cstdio>
#include <numbers>

// A distance with a human-readable unit (m / km / Mm / Gm / AU / ly); sign kept.
inline char *fmt_dist(double meters, char *buf, size_t n) {
    const double a = meters < 0.0 ? -meters : meters;
    const char *unit;
    double v;
    if(a < 1e3)                { v = meters;                     unit = "m";  }
    else if(a < 1e6)           { v = meters / 1e3;               unit = "km"; }
    else if(a < 1e9)           { v = meters / 1e6;               unit = "Mm"; }
    else if(a < 1e12)          { v = meters / 1e9;               unit = "Gm"; }
    else if(a < 1e16)          { v = meters / 1.495978707e11;    unit = "AU"; }
    else                       { v = meters / 9.4607304725808e15; unit = "ly"; }
    snprintf(buf, n, "%.1f %s", v, unit);
    return buf;
}

// A speed: m/s below 1 km/s, km/s above.
inline char *fmt_speed(double m_s, char *buf, size_t n) {
    const double a = m_s < 0.0 ? -m_s : m_s;
    if(a < 1e3) { snprintf(buf, n, "%.0f m/s", m_s); }
    else        { snprintf(buf, n, "%.2f km/s", m_s / 1e3); }
    return buf;
}

// A surface gravity. A fixed "%.2f" prints every small moon as "0.00 m/s2" --
// 151 of the 240 bodies in solar_system_full -- which reads as "no gravity"
// about a body whose lander still answers to it. Two decades down the ladder is
// mm/s2, and the cut sits at 1e-1 rather than where the digits run out: 0.0123
// m/s2 is one significant figure where 12.30 mm/s2 is four. Below 1e-5 mm needs
// three digits, so it goes scientific -- as does above 1e6, which keeps the whole
// ladder inside the 32-char buffers every call site passes. A NaN prints "-", not
// "0": "no gravity" is a fact about a world and "could not be computed" is a bug in
// this program, and the dossier must not print the first about the second.
inline char *fmt_accel(double m_s2, char *buf, size_t n) {
    const double a = m_s2 < 0.0 ? -m_s2 : m_s2;
    if(!(a > 0.0))      { snprintf(buf, n, a == 0.0 ? "0 m/s2" : "-"); }
    else if(a < 1e-5)   { snprintf(buf, n, "%.2e m/s2", m_s2); }
    else if(a < 1e-1)   { snprintf(buf, n, "%.2f mm/s2", m_s2 * 1e3); }
    else if(a < 1e6)    { snprintf(buf, n, "%.2f m/s2", m_s2); }
    else                { snprintf(buf, n, "%.2e m/s2", m_s2); }
    return buf;
}

// An angle in degrees (internal angles are radians).
inline char *fmt_deg(double radians, char *buf, size_t n) {
    snprintf(buf, n, "%.2f deg", radians * 180.0 / std::numbers::pi);
    return buf;
}

// A mass in scientific notation: a fixed unit ladder never reads well across
// body classes.
inline char *fmt_mass(double kg, char *buf, size_t n) {
    snprintf(buf, n, "%.3e kg", kg);
    return buf;
}

// "1d 04:03:02" or "04:03:02" — ToF / orbit-period readouts.
// long long, not int: interstellar ToFs exceed INT_MAX days (UB cast).
inline char *fmt_time(double s, char *buf, size_t n) {
    if(s < 0.0) { s = 0.0; }
    const long long d = (long long)(s / 86400.0);
    const long long h = (long long)(s / 3600.0) % 24;
    const long long m = (long long)(s / 60.0) % 60;
    const long long sec = (long long)s % 60;
    if(d > 0) { snprintf(buf, n, "%lldd %02lld:%02lld:%02lld", d, h, m, sec); }
    else     { snprintf(buf, n, "%02lld:%02lld:%02lld", h, m, sec); }
    return buf;
}
