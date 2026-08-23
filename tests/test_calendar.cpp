// test_calendar: the home-planet calendar (src/calendar.h).
// Runs from the repo root:
//   make test   (or: g++ -O2 -std=c++11 -I./src tests/test_calendar.cpp -o test_calendar && ./test_calendar)
//
// Pinned against the Eerbon system data (system.json), whose home planet's
// rates are the same as KSP's Kerbin:
//   home spin   rot_ang_speed = 2.9157090303706880702966723086e-4 rad/s
//   home orbit  orb_ang_speed = 6.8269186570822291594437651e-7 rad/s
// -> day  = 21,549 s (5 h 59 m)
// -> year = 9,203,545 s = 427.09 days  -> snapped to 427 days
// -> 11 x 36-day months + a 31-day 12th month
// -> epoch year 4724, so t = 0 is Yr 4724 Mo 1 Day 1 00:00:00
#include "calendar.h"

#include <cmath>
#include <cstdio>

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
    const double TWOPI = 6.2831853071795864765;
    const double D = TWOPI / 2.9157090303706880702966723086e-4; // Eerbon spin
    const double Y = TWOPI / 6.8269186570822291594437651e-7; // Eerbon orbit
    const int EPOCH = 4724;

    // --- derived constants ---------------------------------------------------
    Calendar cal = Calendar::make(D, Y, EPOCH);
    CHECK(cal.valid());
    CHECK(cal.has_year());
    CHECK_NEAR(D, 21549.0, 1.0);                 // 5 h 59 m home day
    CHECK_NEAR(Y, 9203545.0, 1.0);               // 106.5 real-day home year
    CHECK(cal.days_per_year == 427);             // round(9203545 / 21549)
    int sum = 0;
    for(int m = 0; m < 12; m++) { sum += cal.month_days[m]; }
    CHECK(sum == cal.days_per_year);
    for(int m = 0; m < 11; m++) { CHECK(cal.month_days[m] == 36); }
    CHECK(cal.month_days[11] == 31);             // 11*36 + 31 = 427
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

    CalTime ny = cal.at(427.0 * D + 1.0);        // new year at midnight
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

    if(failures == 0) {
        printf("test_calendar: all checks passed\n");
        return 0;
    }
    printf("test_calendar: %d FAILURE(S)\n", failures);
    return 1;
}
