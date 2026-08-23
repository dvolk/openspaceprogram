#pragma once
// A home-planet calendar derived from a body's spin + orbital rates.
//
// The sim clock is one number (seconds since game start); this turns it
// into a date on a 24-hour dial:
//
//   day   D = 2*pi / spin_rate       (one rotation of the body)
//   year  Y = 2*pi / orbital_rate    (one orbit; 0 if the body doesn't orbit)
//
// The calendar year is SNAPPED to a whole number of days, round(Y/D), so
// that day / month / year boundaries all fall exactly on local midnight
// (tod == 0) -- a non-integer day count would make the month roll over
// mid-day. For Eerbon/Kerbin (D = 21,549 s, Y = 9,203,545 s) that is 427
// days, and the snap shortens the year by 4,122 s (0.045%) -- the true
// orbital period stays in year_seconds for anything that needs the real
// orbit (e.g. seasons).
//
// Months: 12. The first 11 get round(days_per_year/12) days each, the
// 12th the remainder. Eerbon/Kerbin: 11 x 36-day months + a 31-day 12th.
// A body whose year is shorter than 12 days (the Moon: day == orbit,
// tidally locked) gets no year/months -- just a running day count.
//
// Pure math, no game state: the calendar is a function of (D, Y, epoch, t)
// only, so it freezes with the sim clock when time_accel is 0 and needs
// nothing extra persisted.

#include <cmath>

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
            const int base = (int)std::lround((double)c.days_per_year / 12.0);
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

        // Time of day on a 24-hour dial (86,400 dial-seconds per day).
        // Clamp at 23:59:59 so rounding can never cross into the next day
        // (the day number comes from day_count above, which hasn't rolled).
        long total = (long)std::lround(std::fmod(t, day_seconds)
                                       * 86400.0 / day_seconds);
        if(total >= 86400) { total = 86399; }
        h.hh = (int)(total / 3600);
        h.mm = (int)((total % 3600) / 60);
        h.ss = (int)(total % 60);
        return h;
    }
};
