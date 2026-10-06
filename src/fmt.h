// fmt.h -- UI formatting helpers (header-only, zero-alloc buffer-fill).
// For HUMAN readouts; instrument logs stay raw SI (the e2e CHECK harness
// parses them). Buffer-fill, not std::string: the no-alloc guarantee is
// by construction.

#pragma once

#include <cstddef>
#include <cstdio>
#include <numbers>

// A distance with a human-readable unit (m / km / Mm / Gm / AU / ly).
// The sign is kept. Returns buf.
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

// A speed: m/s below 1 km/s, km/s above (the readouts a player compares
// against escape velocity and the transfer planner's legs). Returns buf.
inline char *fmt_speed(double m_s, char *buf, size_t n) {
    const double a = m_s < 0.0 ? -m_s : m_s;
    if(a < 1e3) { snprintf(buf, n, "%.0f m/s", m_s); }
    else        { snprintf(buf, n, "%.2f km/s", m_s / 1e3); }
    return buf;
}

// An angle in degrees (the authored orbital/spin angles are radians; the
// JSON reader wants the familiar unit). Returns buf.
inline char *fmt_deg(double radians, char *buf, size_t n) {
    snprintf(buf, n, "%.2f deg", radians * 180.0 / std::numbers::pi);
    return buf;
}

// A mass in scientific notation: body masses span two decades per body
// class, so a fixed unit ladder never reads well. Returns buf.
inline char *fmt_mass(double kg, char *buf, size_t n) {
    snprintf(buf, n, "%.3e kg", kg);
    return buf;
}

// "1d 04:03:02" or "04:03:02" — ToF / orbit-period readouts.
// long long, not int: interstellar ToFs exceed INT_MAX days (UB cast). Returns buf.
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
