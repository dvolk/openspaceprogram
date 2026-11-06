// test_recorder: the flight recorder's row grammar (src/recorder.h, #662, #674).
// Runs from the repo root:
//   make test   (or: g++ -O2 -std=c++20 -I./src tests/test_recorder.cpp -o test_recorder && ./test_recorder)
//
// The recorder is the instrument, not the analysis, so the exact row string is
// the contract and this pins it:
//
//   One file, two halves: a `#rec v<n> channels=<list>` header and `[rec]` rows
//   whose keys are that list IN ORDER. Pinned against each other, because the
//   whole design claim is that a reader needs no per-channel table -- if a
//   channel is added to the list and the row, and the order drifts, the trace
//   silently mislabels every later column.
//
//   The dash rules are --drag-log's, copied rather than re-invented: a column
//   that has no value prints `-`. Dashing a zero is forbidden -- zero is a
//   reading (no lift, no chute, no airspeed), and an instrument that blinks
//   zeros loses the difference between "off" and "absent".
//
//   v2 (#674) added the frame group: the ten columns the SURFACE and ORBITAL
//   windows show. They dash as a GROUP, on one test, because they come from one
//   snapshot -- and an unfilled snapshot reads zeros (#235), which is exactly the
//   0-that-isn't-a-reading the law above exists to refuse.
//
//   A row that does not fit its buffer is refused (returns -1) rather than
//   written short: a half-written row is a parse error in someone else's script.
#include "recorder.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

static int failures = 0;

#define CHECK_ROW(sample, want) do { \
        char _b[512]; \
        const int _n = recRow((sample), _b, sizeof _b); \
        if(_n < 0 || std::strcmp(_b, (want)) != 0) { \
            printf("FAIL %s:%d: recRow wrote %s, want \"%s\"\n", \
                   __FILE__, __LINE__, (_n < 0 ? "(refused)" : _b), (want)); \
            failures++; \
        } \
    } while(0)

// A reading from a cruise frame: air, flow and flight all present.
static RecSample cruise() {
    RecSample s;
    s.t = 12.50; s.alt_asl = 700.0; s.v = 78.00; s.rho = 1.225;
    s.F = 210.50; s.L = 12.00; s.Cd = 0.950; s.A = 3.20;
    s.aoa = 2.30; s.beta = -1.00; s.cpl = 0.250; s.ind = 0.0200;
    s.chute = 0.00;
    s.T = 288.15; s.p = 101325.0; s.a = 340.292; s.mach = 0.100;
    s.mass = 6000.0; s.thr = 0.75; s.stg = 2;
    // The windows' group: the three speeds differ because they are three
    // different frames, not three opinions of one number.
    s.alt_agl = 698.5; s.v_surf = 77.75; s.v_orb = 252.75;
    s.vs = -0.25; s.hs = 77.75;
    s.lat = -0.0943; s.lon = 30.5;
    s.pitch = 2.5; s.roll = -0.5; s.hdg = 91.5;
    s.dv = 1450.5; s.acc = 3.75;
    return s;
}

int main() {
    // --- a full reading, exactly as printed -------------------------------
    // The precisions are the logs' (--drag-log for the aero group, the SURFACE
    // window's rows for the frame group), so the same reading looks the same in
    // the trace, in the log and in the window.
    CHECK_ROW(cruise(),
              "t=12.50 alt_asl=700.0 alt_agl=698.5 v=78.00 v_surf=77.75 "
              "v_orb=252.75 vs=-0.25 hs=77.75 lat=-0.094300 lon=30.500000 "
              "pitch=2.5 roll=-0.5 hdg=91.5 dv=1450.5 acc=3.75 "
              "rho=1.225 F=210.50 L=12.00 Cd=0.950 "
              "A=3.20 aoa=2.3 beta=-1.0 cpl=0.25 ind=0.020 chute=0.00 "
              "T=288.1 p=101325 a=340.3 mach=0.100 mass=6000.0 thr=0.75 stg=2");

    // --- on the ground: no air, so no flow group either -------------------
    // The air-relative readings are meaningless with rho = 0, and the flow
    // geometry needs a flow to be measured against. Everything else keeps its
    // zero: mass, throttle, stage, altitude, speed. Note the frame group keeps
    // printing v_orb while v and v_surf read 0: a hull parked on a rotating body
    // IS moving at 175 m/s over the ground's own frame, and a recorder that
    // called that "not a reading" would hide the difference between the two.
    RecSample pad = cruise();
    pad.t = 0.0; pad.alt_asl = 0.0; pad.v = 0.0; pad.rho = 0.0;
    pad.F = 0.0; pad.L = 0.0; pad.aoa = pad.beta = pad.cpl = pad.ind = NAN;
    pad.T = pad.p = pad.a = pad.mach = NAN;
    pad.thr = 0.0; pad.stg = 0;
    pad.alt_agl = 0.0; pad.v_surf = 0.0; pad.v_orb = 175.0;
    pad.vs = 0.0; pad.hs = 0.0;
    pad.lat = 0.0; pad.lon = 0.0;
    pad.pitch = 0.0; pad.roll = 0.0; pad.hdg = 0.0;
    pad.acc = 0.0;
    CHECK_ROW(pad,
              "t=0.00 alt_asl=0.0 alt_agl=0.0 v=0.00 v_surf=0.00 "
              "v_orb=175.00 vs=0.00 hs=0.00 lat=0.000000 lon=0.000000 "
              "pitch=0.0 roll=0.0 hdg=0.0 dv=1450.5 acc=0.00 "
              "rho=0 F=0.00 L=0.00 Cd=0.950 A=3.20 "
              "aoa=- beta=- cpl=- ind=- chute=0.00 T=- p=- a=- mach=- "
              "mass=6000.0 thr=0.00 stg=0");

    // --- air, but no lifting surfaces to define the flow frame ------------
    RecSample ball = cruise();
    ball.aoa = ball.beta = ball.cpl = ball.ind = NAN;
    ball.L = 0.0;
    CHECK_ROW(ball,
              "t=12.50 alt_asl=700.0 alt_agl=698.5 v=78.00 v_surf=77.75 "
              "v_orb=252.75 vs=-0.25 hs=77.75 lat=-0.094300 lon=30.500000 "
              "pitch=2.5 roll=-0.5 hdg=91.5 dv=1450.5 acc=3.75 "
              "rho=1.225 F=210.50 L=0.00 Cd=0.950 "
              "A=3.20 aoa=- beta=- cpl=- ind=- chute=0.00 "
              "T=288.1 p=101325 a=340.3 mach=0.100 mass=6000.0 thr=0.75 stg=2");

    // --- a zero is a reading, not a gap ----------------------------------
    RecSample nosurf = cruise();
    nosurf.L = 0.0; nosurf.ind = 0.0; nosurf.beta = 0.0;
    CHECK_ROW(nosurf,
              "t=12.50 alt_asl=700.0 alt_agl=698.5 v=78.00 v_surf=77.75 "
              "v_orb=252.75 vs=-0.25 hs=77.75 lat=-0.094300 lon=30.500000 "
              "pitch=2.5 roll=-0.5 hdg=91.5 dv=1450.5 acc=3.75 "
              "rho=1.225 F=210.50 L=0.00 Cd=0.950 "
              "A=3.20 aoa=2.3 beta=0.0 cpl=0.25 ind=0.000 chute=0.00 "
              "T=288.1 p=101325 a=340.3 mach=0.100 mass=6000.0 thr=0.75 stg=2");

    // --- no snapshot for this hull: the frame group dashes, together ------
    // The physics group is untouched: lastDrag* belongs to the hull whatever the
    // render pass has been doing. This is the row a recorder writes in the tick
    // before the first refresh, or the one for a hull the view never described.
    RecSample stale = cruise();
    stale.alt_agl = stale.v_surf = stale.v_orb = stale.vs = stale.hs = NAN;
    stale.lat = stale.lon = stale.pitch = stale.roll = stale.hdg = NAN;
    CHECK_ROW(stale,
              "t=12.50 alt_asl=700.0 alt_agl=- v=78.00 v_surf=- "
              "v_orb=- vs=- hs=- lat=- lon=- pitch=- roll=- hdg=- "
              "dv=1450.5 acc=3.75 "
              "rho=1.225 F=210.50 L=12.00 Cd=0.950 "
              "A=3.20 aoa=2.3 beta=-1.0 cpl=0.25 ind=0.020 chute=0.00 "
              "T=288.1 p=101325 a=340.3 mach=0.100 mass=6000.0 thr=0.75 stg=2");

    // --- the header's channel list and the row's keys are the same list ----
    {
        char buf[512];
        const int n = recRow(cruise(), buf, sizeof buf);
        if(n < 0) {
            printf("FAIL %s:%d: a full row was refused\n", __FILE__, __LINE__);
            failures++;
        } else {
            const std::string row(buf);
            size_t after = 0;
            for(const char *key = recChannels(); *key; ) {
                const char *sp = std::strchr(key, ' ');
                const std::string k = sp ? std::string(key, sp) : std::string(key);
                const std::string want = k + "=";
                const size_t at = row.find(want, after);
                if(at == std::string::npos) {
                    printf("FAIL %s:%d: channel \"%s\" is not in the row (out of "
                           "order or missing): %s\n", __FILE__, __LINE__, k.c_str(), buf);
                    failures++;
                    break;
                }
                after = at + want.size();
                key = sp ? sp + 1 : key + strlen(key);
            }
            // No row key the header does not promise. Every field carries
            // exactly one '=', and no value contains one, so counting the
            // separators is the whole check.
            const size_t keys = std::count(row.begin(), row.end(), '=');
            const size_t channels = 1 + std::count(recChannels(),
                                                   recChannels() + strlen(recChannels()), ' ');
            if(keys != channels) {
                printf("FAIL %s:%d: %zu channels in the header, %zu keys in the row\n",
                       __FILE__, __LINE__, channels, keys);
                failures++;
            }
        }
    }

    // --- a row that does not fit is refused, not truncated ----------------
    {
        char small[32];
        if(recRow(cruise(), small, sizeof small) >= 0) {
            printf("FAIL %s:%d: a 32-byte buffer accepted a full row\n", __FILE__, __LINE__);
            failures++;
        }
    }

    if(failures == 0) {
        printf("test_recorder: all checks passed\n");
        return 0;
    }
    printf("test_recorder: %d FAILURE(S)\n", failures);
    return 1;
}
