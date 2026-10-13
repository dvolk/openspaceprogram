#pragma once
// Home-planet calendar from a body's day and year lengths (#201), or a
// proleptic-Gregorian civil calendar (#202, "epoch_utc" in the system JSON).
//
// Derived day unit D is the SOLAR day, measured by the caller (system.cpp's
// measureSolarDay) -- never 2*pi/spin_rate: sidereal slides off the sun and is
// wrong for Venus / Triton / the tidally locked Moon. The year is snapped to
// round(Y/D) whole days so boundaries fall on local midnight; a body with a
// year shorter than 12 days gets just a day count.
//
// Civil mode runs fixed 86400 s days -- never the measured solar day (Earth's
// is 86400.101 s).
//
// Not a bug: the game year is sidereal, Gregorian is tropical, so seasons
// drift against calendar dates. `year_seconds` is the seasonal handle.

#include <cmath>
#include <cstdio>
#include <cstddef>

struct CalTime {
    int year = 0;
    int month = 1;   // 1..12
    int day = 1;     // 1..days-in-month (or running count without a year)
    int hh = 0, mm = 0, ss = 0;  // 24-hour dial: 1 hour = day_seconds/24
    bool has_year = true;
    bool civil = false;  // Gregorian/UTC fields (#202)
};

// Proleptic Gregorian <-> day count (days since 1970-01-01); y may be negative.
inline long days_from_civil(int y, unsigned m, unsigned d) {
    y -= (m <= 2);
    const long era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);            // [0, 399]
    const unsigned doy =
        (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;        // [0, 365]
    const unsigned doe =
        yoe * 365 + yoe / 4 - yoe / 100 + doy;                 // [0, 146096]
    return era * 146097 + (long)doe - 719468;
}

inline void civil_from_days(long z, int &y, unsigned &m, unsigned &d) {
    z += 719468;
    const long era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = (unsigned)(z - era * 146097);         // [0, 146096]
    const unsigned yoe =
        (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365; // [0, 399]
    const long yll = (long)yoe + era * 400;
    const unsigned doy =
        doe - (365 * yoe + yoe / 4 - yoe / 100);               // [0, 365]
    const unsigned mp = (5 * doy + 2) / 153;                   // [0, 11]
    d = doy - (153 * mp + 2) / 5 + 1;                          // [1, 31]
    m = mp + (mp < 10 ? 3 : (unsigned)-9);                     // [1, 12]
    y = (int)yll + (m <= 2);
}

// Days in a proleptic-Gregorian month (1..12).
inline int civil_month_days(int y, int m) {
    static const int md[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if(m == 2) {
        const bool leap = (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
        return leap ? 29 : 28;
    }
    return md[m - 1];
}

struct Calendar {
    // Clock's day unit (sim seconds). Derived: the measured solar day.
    // Civil: exactly 86400 -- NEVER the solar day (Earth's is 86400.101).
    double day_seconds = 0.0;
    // Measured solar day (#201), body panel only; == day_seconds when derived.
    double solar_day_seconds = 0.0;
    double year_seconds = 0.0;  // Y (TRUE orbital period); seasons handle
    int days_per_year = 0;      // round(Y / D); 0 = no year. Civil: 365
    int month_days[12] = {0};   // derived: 12 months. Civil: unused
    int epoch_year = 1;         // the year number at t == 0 (derived)
    bool civil = false;         // Gregorian/UTC (#202)
    long epoch_days = 0;        // civil: days_from_civil of the epoch
    double epoch_sod = 0.0;     // civil: seconds-of-day at t=0 (THH:MM:SS)

    bool valid() const { return day_seconds > 0.0; }
    bool has_year() const { return civil || days_per_year >= 12; }

    // Derived 12-month calendar (#201).
    static Calendar make(double D, double Y, int epoch_year) {
        Calendar c;
        c.day_seconds = D;
        c.solar_day_seconds = D;
        c.year_seconds = Y;
        c.epoch_year = epoch_year;
        if(D > 0.0 && Y > 0.0) {
            c.days_per_year = (int)std::lround(Y / D);
            if(c.days_per_year < 1) { c.days_per_year = 1; }
        }
        if(c.days_per_year >= 12) {
            // Prefer round(N/12); fall back to floor-split so every month >= 1.
            int base = (int)std::lround((double)c.days_per_year / 12.0);
            if(base < 1) { base = 1; }
            if(c.days_per_year - 11 * base < 1) {
                base = c.days_per_year / 12;
                if(base < 1) { base = 1; }
            }
            for(int m = 0; m < 11; m++) { c.month_days[m] = base; }
            c.month_days[11] = c.days_per_year - 11 * base;
        }
        return c;
    }

    // Proleptic-Gregorian civil calendar (#202). `solar_day` is body-panel
    // only; the clock always runs 86400 s days. `epoch_sod`: seconds of day
    // at t=0 (from epoch_utc's THH:MM:SS).
    static Calendar makeCivil(long epoch_days, double epoch_sod,
                              double Y, double solar_day) {
        Calendar c;
        c.civil = true;
        c.epoch_days = epoch_days;
        c.epoch_sod = epoch_sod;
        c.day_seconds = 86400.0;
        c.solar_day_seconds = solar_day;
        c.year_seconds = Y;
        // Only splits fmt_cal_duration's "1y .."; 365 approximates a civil year.
        c.days_per_year = 365;
        int y = 0;
        unsigned m = 1, d = 1;
        civil_from_days(epoch_days, y, m, d);
        c.epoch_year = y;
        return c;
    }

    CalTime at(double t) const {
        CalTime h;
        h.civil = civil;
        if(day_seconds <= 0.0 || t < 0.0) { return h; }

        if(civil) {
            // t runs from the civil epoch instant (epoch_sod into that date),
            // so a non-midnight epoch_utc works.
            double el = epoch_sod + t;
            long day_count = (long)std::floor(el / 86400.0);
            long total = (long)std::lround((el - (double)day_count * 86400.0)
                                           * 1.0);
            if(total >= 86400) { total = 86399; day_count += 1; }
            int y = 0;
            unsigned m = 1, d = 1;
            civil_from_days(epoch_days + day_count, y, m, d);
            h.year = y;
            h.month = (int)m;
            h.day = (int)d;
            h.has_year = true;
            h.hh = (int)(total / 3600);
            h.mm = (int)((total % 3600) / 60);
            h.ss = (int)(total % 60);
            return h;
        }

        const long day_count = (long)std::floor(t / day_seconds);
        long total = (long)std::lround(std::fmod(t, day_seconds)
                                       * 86400.0 / day_seconds);
        if(total >= 86400) { total = 86399; }

        if(has_year()) {
            h.year = epoch_year + (int)(day_count / days_per_year);
            int doy = (int)(day_count % days_per_year);  // 0-based day of year
            h.month = 12;
            for(int m = 0; m < 12; m++) {
                if(doy < month_days[m]) { h.month = m + 1; h.day = doy + 1; break; }
                doy -= month_days[m];
            }
        } else {
            h.has_year = false;
            h.year = epoch_year;
            h.month = 1;
            h.day = (int)day_count + 1;
        }

        h.hh = (int)(total / 3600);
        h.mm = (int)((total % 3600) / 60);
        h.ss = (int)(total % 60);
        return h;
    }
};

// Day-of-year (1-based) from a CalTime that has a year.
inline int cal_day_of_year(const Calendar &cal, const CalTime &ct) {
    if(cal.civil) {
        int doy = ct.day;
        for(int m = 1; m < ct.month; m++) {
            doy += civil_month_days(ct.year, m);
        }
        return doy;
    }
    int doy = ct.day;
    for(int m = 0; m < ct.month - 1; m++) { doy += cal.month_days[m]; }
    return doy;
}

// Zero-alloc buffer-fill; false when there is no calendar line.
inline bool fmt_cal_time(const Calendar &cal, double t, char *buf, size_t n) {
    if(!cal.valid() || t < 0.0) { buf[0] = '\0'; return false; }
    const CalTime ct = cal.at(t);
    if(cal.civil) {
        snprintf(buf, n, "%04d-%02d-%02d  %02d:%02d UTC",
                 ct.year, ct.month, ct.day, ct.hh, ct.mm);
    } else if(ct.has_year) {
        snprintf(buf, n, "Year %04d   Day %d/%d   %02d:%02d",
                 ct.year, cal_day_of_year(cal, ct), cal.days_per_year,
                 ct.hh, ct.mm);
    } else {
        snprintf(buf, n, "Day %d   %02d:%02d", ct.day, ct.hh, ct.mm);
    }
    return true;
}

// Compact stamp for event lists. Same false/empty contract as fmt_cal_time.
inline bool fmt_cal_compact(const Calendar &cal, double t, char *buf, size_t n) {
    if(!cal.valid() || t < 0.0) { buf[0] = '\0'; return false; }
    const CalTime ct = cal.at(t);
    if(cal.civil) {
        snprintf(buf, n, "%04d-%02d-%02d  %02d:%02d",
                 ct.year, ct.month, ct.day, ct.hh, ct.mm);
    } else if(ct.has_year) {
        snprintf(buf, n, "Yr %04d Day %d  %02d:%02d",
                 ct.year, cal_day_of_year(cal, ct), ct.hh, ct.mm);
    } else {
        snprintf(buf, n, "Day %d  %02d:%02d", ct.day, ct.hh, ct.mm);
    }
    return true;
}

/* Elapsed sim seconds as a home-calendar duration ("1y 2d 3h 04m"). Returns buf. */
inline char *fmt_cal_duration(const Calendar &cal, double dt, char *buf, size_t n) {
    if(dt < 0.0) { dt = 0.0; }
    if(!cal.valid() || cal.day_seconds <= 0.0) {
        snprintf(buf, n, "%.0fs", dt);
        return buf;
    }
    // Time-of-day maps as in Calendar::at; lround can hit 86400 at the
    // boundary -- clamp.
    const long day_count = (long)(dt / cal.day_seconds);
    long total = (long)std::lround(std::fmod(dt, cal.day_seconds)
                                   * 86400.0 / cal.day_seconds);
    if(total >= 86400) { total = 86399; }
    const int hh = (int)(total / 3600);
    const int mm = (int)((total % 3600) / 60);
    const int ss = (int)(total % 60);

    int years = 0, days = (int)day_count;
    if(cal.has_year()) {
        years = (int)(day_count / cal.days_per_year);
        days = (int)(day_count % cal.days_per_year);
    }

    if(years > 0) {
        if(days > 0 || hh > 0) {
            snprintf(buf, n, "%dy %dd %dh %02dm", years, days, hh, mm);
        } else {
            snprintf(buf, n, "%dy %02dm", years, mm);
        }
    } else if(days > 0) {
        if(hh > 0) {
            snprintf(buf, n, "%dd %dh %02dm", days, hh, mm);
        } else {
            snprintf(buf, n, "%dd %02dm", days, mm);
        }
    } else if(hh > 0) {
        snprintf(buf, n, "%dh %02dm", hh, mm);
    } else if(mm > 0) {
        snprintf(buf, n, "%dm %02ds", mm, ss);
    } else {
        snprintf(buf, n, "%ds", ss);
    }
    return buf;
}
