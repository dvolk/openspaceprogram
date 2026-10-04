#pragma once
// A home-planet calendar from a body's spin + orbital rates.
// day D = 2*pi / spin_rate; year Y = 2*pi / orbital_rate (0 if no orbit).
// The calendar year is SNAPPED to a whole number of days round(Y/D) so
// boundaries fall on local midnight. 12 months (first 11 get
// round(Y/D/12) days, the 12th the remainder). A body whose year is
// shorter than 12 days gets no year/months -- just a day count.
// Pure math: a function of (D, Y, epoch, t) only.

#include <cmath>
#include <cstdio>
#include <cstddef>

struct CalTime {
    int year = 0;
    int month = 1;   // 1..12
    int day = 1;     // 1..days-in-month (or running count without a year)
    int hh = 0, mm = 0, ss = 0;  // 24-hour dial: 1 hour = D/24 sim seconds
    bool has_year = true;
};

struct Calendar {
    double day_seconds = 0.0;   // D, sim seconds; 0 = body doesn't spin
    double year_seconds = 0.0;  // Y (TRUE orbital period), sim seconds; 0 = none
    int days_per_year = 0;      // round(Y / D); 0 = no year
    int month_days[12] = {0};   // 12 months, summing to days_per_year
    int epoch_year = 1;         // the year number at t == 0

    bool valid() const { return day_seconds > 0.0; }
    bool has_year() const { return days_per_year >= 12; }

    static Calendar make(double D, double Y, int epoch_year) {
        Calendar c;
        c.day_seconds = D;
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

    CalTime at(double t) const {
        CalTime h;
        if(day_seconds <= 0.0 || t < 0.0) { return h; }

        // Whole days elapsed; day/month/year only ever change at midnight.
        const long day_count = (long)std::floor(t / day_seconds);
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

        // Time of day on a 24-hour dial. Clamp at 23:59:59 so rounding
        // never crosses into the next day.
        long total = (long)std::lround(std::fmod(t, day_seconds)
                                       * 86400.0 / day_seconds);
        if(total >= 86400) { total = 86399; }
        h.hh = (int)(total / 3600);
        h.mm = (int)((total % 3600) / 60);
        h.ss = (int)(total % 60);
        return h;
    }
};

// Day-of-year (1-based) from a CalTime that has a year.
inline int cal_day_of_year(const Calendar &cal, const CalTime &ct) {
    int doy = ct.day;
    for(int m = 0; m < ct.month - 1; m++) { doy += cal.month_days[m]; }
    return doy;
}

// "Year 2000   Day 12/427   08:14" -- the HUD / Transfer stamp.
// Zero-alloc buffer-fill. Returns false when there is no calendar line.
inline bool fmt_cal_time(const Calendar &cal, double t, char *buf, size_t n) {
    if(!cal.valid() || t < 0.0) { buf[0] = '\0'; return false; }
    const CalTime ct = cal.at(t);
    if(ct.has_year) {
        snprintf(buf, n, "Year %04d   Day %d/%d   %02d:%02d",
                 ct.year, cal_day_of_year(cal, ct), cal.days_per_year,
                 ct.hh, ct.mm);
    } else {
        snprintf(buf, n, "Day %d   %02d:%02d", ct.day, ct.hh, ct.mm);
    }
    return true;
}

// "Yr 4724 Day 12  08:14" -- compact stamp for event lists. Same
// false/empty contract as fmt_cal_time.
inline bool fmt_cal_compact(const Calendar &cal, double t, char *buf, size_t n) {
    if(!cal.valid() || t < 0.0) { buf[0] = '\0'; return false; }
    const CalTime ct = cal.at(t);
    if(ct.has_year) {
        snprintf(buf, n, "Yr %04d Day %d  %02d:%02d",
                 ct.year, cal_day_of_year(cal, ct), ct.hh, ct.mm);
    } else {
        snprintf(buf, n, "Day %d  %02d:%02d", ct.day, ct.hh, ct.mm);
    }
    return true;
}

/* Elapsed sim seconds as a home-calendar duration: "1y 2d 3h 04m", etc.
   Returns buf. */
inline char *fmt_cal_duration(const Calendar &cal, double dt, char *buf, size_t n) {
    if(dt < 0.0) { dt = 0.0; }
    if(!cal.valid() || cal.day_seconds <= 0.0) {
        snprintf(buf, n, "%.0fs", dt);
        return buf;
    }
    // Whole days, then the dial time-of-day within the day (same mapping
    // as Calendar::at). lround can hit 86400 at the boundary -- clamp.
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

    // Drop leading zero components; always show at least seconds.
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
