// fmt.h -- UI formatting helpers (header-only, no dependencies, zero-alloc).
//
// These format for HUMAN readouts (the HUD, the Orbital / Surface /
// Transfer windows). The instrument logs (--orbit-log / --dbg-log / ...)
// deliberately stay raw SI: the e2e CHECK harness parses them, and raw
// meters is the honest unit there.
//
// Buffer-fill, not a returned std::string: the no-allocation guarantee is
// then by construction, not an accident of std::string's small-string
// buffer (whose capacity is a library implementation detail the outputs
// would silently outgrow as distances scale).

#pragma once

#include <cstddef>
#include <cstdio>

// A distance with a human-readable unit. The ladder:
//   m < 1 km < 1 Mm < 1e12 m < 1e16 m < ly
// so the distance scenarios read neptune = 30.0 AU, oort = 6684.6 AU,
// interstellar = 10.6 ly, instead of 16-digit meter counts. The sign is
// kept (a sub-meter negative altitude prints "-0.4 m"). Returns buf.
inline char *fmt_dist(double meters, char *buf, size_t n) {
    const double a = meters < 0.0 ? -meters : meters;
    const char *unit;
    double v;
    if(a < 1e3)                { v = meters;                     unit = "m";  }
    else if(a < 1e6)           { v = meters / 1e3;               unit = "km"; }
    else if(a < 1e12)          { v = meters / 1e6;               unit = "Mm"; }
    else if(a < 1e16)          { v = meters / 1.495978707e11;    unit = "AU"; }
    else                       { v = meters / 9.4607304725808e15; unit = "ly"; }
    snprintf(buf, n, "%.1f %s", v, unit);
    return buf;
}

// "1d 04:03:02" or "04:03:02" — ToF / orbit-period readouts.
// long long, not int: oort/interstellar-class ToFs exceed INT_MAX days,
// and the out-of-range double->int cast is UB (INT_MIN on x86-64).
// Returns buf.
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
