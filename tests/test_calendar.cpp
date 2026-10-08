// test_calendar: the home-planet calendar (src/calendar.h).
// Runs from the repo root:
//   make test   (or: g++ -O2 -std=c++20 -I./src tests/test_calendar.cpp -o test_calendar && ./test_calendar)
//
// D here is the SOLAR day (what system.cpp feeds Calendar::make after
// #201's measureSolarDay), pinned for Eerbon / KSP-Kerbin rates:
//   home spin   rot_ang_speed = 2.9157090303706880702966723086e-4 rad/s
//   home orbit  orb_ang_speed = 6.8269186570822291594437651e-7 rad/s
// -> solar day = 21,600.0 s (the canonical 6 h; the sidereal 21,549 s is
//    NOT what the clock uses -- see #201). measureSolarDay on those rates
//    reads 21600.0; this file is pure Calendar::make and takes D as input.
// -> year = 9,203,545 s = 426.1 solar days  -> snapped to 426 days
// -> 11 x 36-day months + a 30-day 12th month
// -> epoch year 4724, so t = 0 is Yr 4724 Mo 1 Day 1 00:00:00
#include "calendar.h"

#include <cmath>
#include <cstdio>
#include <string>

static int failures = 0;
#define CHECK(cond) do { \
        if(!(cond)) { \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++; \
        } \
    } while(0)

#define CHECK_NEAR(a, b, tol) do { \
        double _a = (a), _b = (b), _t = (tol); \
        if(std::fabs(_a - _b) > _t) { \
            printf("FAIL %s:%d: %s = %g, want %g +- %g\n", \
                   __FILE__, __LINE__, #a, _a, _b, _t); \
            failures++; \
        } \
    } while(0)

int main() {
    // Solar day of the Eerbon/Kerbin rates (measureSolarDay, #201).
    const double D = 21600.0;                    // 6 h solar, not 21549 sidereal
    const double TWOPI = 6.2831853071795864765;
    const double Y = TWOPI / 6.8269186570822291594437651e-7; // Eerbon orbit
    const int EPOCH = 4724;

    // --- derived constants ---------------------------------------------------
    Calendar cal = Calendar::make(D, Y, EPOCH);
    CHECK(cal.valid());
    CHECK(cal.has_year());
    CHECK_NEAR(D, 21600.0, 1.0);                 // 6 h solar home day
    CHECK_NEAR(Y, 9203545.0, 1.0);               // 106.5 real-day home year
    CHECK(cal.days_per_year == 426);             // round(9203545 / 21599.6)
    int sum = 0;
    for(int m = 0; m < 12; m++) { sum += cal.month_days[m]; }
    CHECK(sum == cal.days_per_year);
    for(int m = 0; m < 11; m++) { CHECK(cal.month_days[m] == 36); }
    CHECK(cal.month_days[11] == 30);             // 11*36 + 30 = 426

    // Tiny years: lround(N/12) can empty the 12th month; the floor-split
    // fallback keeps every month >= 1 and the sum exact.
    Calendar tiny = Calendar::make(D, 22.0 * D, EPOCH);
    CHECK(tiny.days_per_year == 22);
    int tsum = 0;
    for(int m = 0; m < 12; m++) {
        CHECK(tiny.month_days[m] >= 1);
        tsum += tiny.month_days[m];
    }
    CHECK(tsum == 22);
    CHECK(cal.year_seconds == Y);                // true orbit kept for seasons

    // --- epoch ----------------------------------------------------------------
    CalTime t0 = cal.at(0.0);
    CHECK(t0.year == EPOCH);
    CHECK(t0.month == 1 && t0.day == 1);
    CHECK(t0.hh == 0 && t0.mm == 0 && t0.ss == 0);

    // --- rollovers land on local midnight -------------------------------------
    // Boundaries are tested just AFTER the exact multiple: the sim clock
    // accumulates dt*time_accel and never lands exactly on k*D, and at the
    // exact double either side of the boundary is representable.
    CalTime d2 = cal.at(D + 1.0);                // 1 s past midnight of day 2
    CHECK(d2.year == EPOCH && d2.month == 1 && d2.day == 2);
    CHECK(d2.hh == 0 && d2.mm == 0 && d2.ss <= 5); // 1 sim s = ~4 dial s

    CalTime just_before = cal.at(D - 1.0);       // still the previous day
    CHECK(just_before.day == 1 && just_before.hh == 23 && just_before.mm == 59);

    CalTime noon = cal.at(D / 2.0);              // midday of day ONE
    CHECK(noon.day == 1 && noon.hh == 12 && noon.mm == 0 && noon.ss == 0);

    CalTime m2 = cal.at(36.0 * D + 1.0);         // month 2 starts at midnight
    CHECK(m2.year == EPOCH && m2.month == 2 && m2.day == 1);
    CHECK(m2.hh == 0 && m2.mm == 0 && m2.ss <= 5); // 1 sim s = ~4 dial s

    CalTime m12 = cal.at(396.0 * D + 1.0);       // the short 12th month
    CHECK(m12.month == 12 && m12.day == 1);

    CalTime ny = cal.at(426.0 * D + 1.0);        // new year at midnight
    CHECK(ny.year == EPOCH + 1 && ny.month == 1 && ny.day == 1);
    CHECK(ny.hh == 0 && ny.mm == 0 && ny.ss <= 5); // 1 sim s = ~4 dial s

    // --- time-of-day never crosses into the next day ---------------------------
    CalTime late = cal.at(D - 0.1);
    CHECK(late.day == 1 && late.hh <= 23);
    CalTime verylate = cal.at(D - 0.0001);
    CHECK(verylate.day == 1);
    CHECK(verylate.hh == 23 && verylate.mm == 59 && verylate.ss == 59);

    // --- mid-month sanity -------------------------------------------------------
    CalTime mid = cal.at(3.0 * D + D * 0.5);
    CHECK(mid.month == 1 && mid.day == 4 && mid.hh == 12);

    // --- tidally locked body (Moon): day == orbit -> no year/months ------------
    Calendar moon = Calendar::make(138984.4, 138984.5, EPOCH);
    CHECK(moon.valid());
    CHECK(!moon.has_year());
    CalTime mt = moon.at(5.25 * 138984.4);
    CHECK(mt.has_year == false);
    CHECK(mt.day == 6 && mt.hh == 6 && mt.mm == 0 && mt.ss == 0);

    // --- star: no spin at all -> invalid calendar -------------------------------
    Calendar star = Calendar::make(0.0, 0.0, EPOCH);
    CHECK(!star.valid());
    CalTime st = star.at(1000.0);
    CHECK(st.year == 0);

    // --- fmt_cal_time / fmt_cal_compact -----------------------------------------
    char buf[64];
    CHECK(fmt_cal_time(cal, 0.0, buf, sizeof buf));
    CHECK(std::string(buf) == "Year 4724   Day 1/426   00:00");
    CHECK(fmt_cal_time(cal, 3.0 * D + D * 0.5, buf, sizeof buf));
    CHECK(std::string(buf) == "Year 4724   Day 4/426   12:00");
    CHECK(fmt_cal_compact(cal, 0.0, buf, sizeof buf));
    CHECK(std::string(buf) == "Yr 4724 Day 1  00:00");
    CHECK(fmt_cal_compact(cal, 3.0 * D + D * 0.5, buf, sizeof buf));
    CHECK(std::string(buf) == "Yr 4724 Day 4  12:00");
    CHECK(!fmt_cal_time(cal, -1.0, buf, sizeof buf));
    CHECK(buf[0] == '\0');
    CHECK(fmt_cal_time(moon, 5.25 * 138984.4, buf, sizeof buf));
    CHECK(std::string(buf) == "Day 6   06:00");

    // --- fmt_cal_duration: home-calendar y/d + 24h dial h/m/s --------------------
    // Every unit is the home calendar's: dial hour = D/24 sim seconds,
    // dial second = D/86400. 30 real seconds on Kerbin is ~2 dial minutes.
    const double dial_h = D / 24.0;
    const double dial_m = D / 1440.0;
    const double dial_s = D / 86400.0;
    fmt_cal_duration(cal, 0.0, buf, sizeof buf);
    CHECK(std::string(buf) == "0s");
    fmt_cal_duration(cal, 30.0 * dial_s, buf, sizeof buf);
    CHECK(std::string(buf) == "30s");
    fmt_cal_duration(cal, 5.0 * dial_m + 12.0 * dial_s, buf, sizeof buf);
    CHECK(std::string(buf) == "5m 12s");
    fmt_cal_duration(cal, 3.0 * dial_h + 4.0 * dial_m, buf, sizeof buf);
    CHECK(std::string(buf) == "3h 04m");
    fmt_cal_duration(cal, 2.0 * D + 3.0 * dial_h + 4.0 * dial_m, buf, sizeof buf);
    CHECK(std::string(buf) == "2d 3h 04m");
    // 1 year + 2 days + 3h 04m (the snapped 426-day year).
    fmt_cal_duration(cal,
                     (double)cal.days_per_year * D + 2.0 * D
                         + 3.0 * dial_h + 4.0 * dial_m,
                     buf, sizeof buf);
    CHECK(std::string(buf) == "1y 2d 3h 04m");
    // Invalid calendar: fall back to raw seconds.
    fmt_cal_duration(star, 42.0, buf, sizeof buf);
    CHECK(std::string(buf) == "42s");

    // --- #202: proleptic-Gregorian civil calendar ---------------------------
    {
        // days_from_civil / civil_from_days round-trip + known pins.
        CHECK(days_from_civil(1970, 1, 1) == 0);
        CHECK(days_from_civil(2000, 1, 1) == 10957);
        CHECK(days_from_civil(2000, 3, 1) == 10957 + 60);   // 2000 is a leap year
        {
            int y = 0; unsigned m = 0, d = 0;
            civil_from_days(10957, y, m, d);
            CHECK(y == 2000 && m == 1 && d == 1);
            civil_from_days(0, y, m, d);
            CHECK(y == 1970 && m == 1 && d == 1);
        }
        CHECK(civil_month_days(2000, 2) == 29);   // /400
        CHECK(civil_month_days(1900, 2) == 28);   // /100
        CHECK(civil_month_days(2001, 2) == 28);
        CHECK(civil_month_days(2004, 2) == 29);   // /4

        const double CIV = 86400.0;
        Calendar g = Calendar::makeCivil(days_from_civil(2000, 1, 1),
                                         365.256 * CIV, 86400.101);
        CHECK(g.valid() && g.civil);
        CHECK(g.day_seconds == 86400.0);          // civil day, NOT the solar day
        CHECK_NEAR(g.solar_day_seconds, 86400.101, 0.001);
        CalTime g0 = g.at(0.0);
        CHECK(g0.year == 2000 && g0.month == 1 && g0.day == 1
              && g0.hh == 0 && g0.mm == 0 && g0.ss == 0 && g0.civil);
        // 2000-01-31 -> 2000-02-01 (31-day month)
        CalTime feb = g.at(31.0 * CIV + 1.0);
        CHECK(feb.year == 2000 && feb.month == 2 && feb.day == 1);
        // Leap day exists in 2000.
        CalTime mar = g.at(60.0 * CIV + 1.0);
        CHECK(mar.month == 3 && mar.day == 1);
        CalTime leap = g.at(59.0 * CIV + 12.0 * 3600.0);
        CHECK(leap.month == 2 && leap.day == 29 && leap.hh == 12);
        // Century rule: 2100-02-28 -> 2100-03-01 (no Feb 29).
        Calendar c100 = Calendar::makeCivil(days_from_civil(2100, 1, 1), 0.0, 86400.0);
        CalTime mar100 = c100.at(59.0 * CIV + 1.0);   // 31 + 28 = 59
        CHECK(mar100.year == 2100 && mar100.month == 3 && mar100.day == 1);
        // June solstice 2000-06-21 01:38.
        const double t_sol = 172.0 * CIV + 1.0 * 3600.0 + 38.0 * 60.0;
        CalTime sol = g.at(t_sol);
        CHECK(sol.year == 2000 && sol.month == 6 && sol.day == 21
              && sol.hh == 1 && sol.mm == 38);
        CHECK(cal_day_of_year(g, sol) == 173);
        char gbuf[64];
        CHECK(fmt_cal_time(g, t_sol, gbuf, sizeof gbuf));
        CHECK(std::string(gbuf) == "2000-06-21  01:38 UTC");
        CHECK(fmt_cal_compact(g, t_sol, gbuf, sizeof gbuf));
        CHECK(std::string(gbuf) == "2000-06-21  01:38");
        // Derived path is untouched (byte-identical pins above).
        CHECK(!cal.civil);
    }

    if(failures == 0) {
        printf("test_calendar: all checks passed\n");
        return 0;
    }
    printf("test_calendar: %d FAILURE(S)\n", failures);
    return 1;
}
