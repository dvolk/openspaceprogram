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
//   v3 (#675) added the ascent profile: q, gamma, and the four burn totals. q and
//   the totals are the physics' own readings and never dash -- 0 with no air and
//   no burn IS the reading. gamma dashes ALONE, on its own NaN, and that is the
//   case pinned below: a hull with no velocity above orbit.h's conic floor has no
//   flight path even in a frame the windows describe perfectly, and dashing the
//   group with it would throw away ten good columns to say one thing.
//
//   v4 (#972) added seams: the hull's own dock count, which is where a dock or an
//   undock leaves its mark. Those events never change WHICH hull is recorded, so they
//   are not gaps -- they are one hull whose parts jumped, and mass/area/Cd/dv move
//   with nothing saying where. Like the totals, it is a count and never dashes.
//
//   A row that does not fit its buffer is refused (returns -1) rather than
//   written short: a half-written row is a parse error in someone else's script.
//   The buffer below is the SAME size tick.cpp gives the writer, so this also
//   pins that the row still fits the game's own buffer.
//
//   The ring (#972) is the other sink for the same sample: the Recorder
//   window's history. It is pinned here rather than in a UI test because its
//   contract is storage, not pixels -- the cap and wrap order, a NaN staying a
//   gap, a same-x push refreshing instead of appending, and a stage change being
//   data inside one span rather than a reset.
//
//   Step 2 of #972 puts the window's DRAWING inputs in this header too, so they
//   get pinned here rather than in a UI test: kRecChans (the toggles, the legend
//   rows and the plot item ids all come from it, so a key must be a real trace
//   column and unique) and the ring's three read-outs -- recExtent (the min..max
//   each line is scaled by, since one graph carries thirteen units), spans() (the
//   one or two runs the plot draws, oldest-first), recTimeExtent (the x range the
//   axis pins to) and recMarks (where stg= or seams= changed, drawn as vertical
//   lines, bounded so a docking port does not become a black wall).
#include "recorder.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include "tchecks.h"

#define CHECK_ROW(sample, want) do { \
        char _b[1024];   /* tick.cpp's row buffer: if the grammar outgrows the \
                            writer, this test is where it says so. */ \
        const int _n = recRow((sample), _b, sizeof _b); \
        if(_n < 0 || std::strcmp(_b, (want)) != 0) { \
            printf("FAIL %s:%d: recRow wrote %s, want \"%s\"\n", \
                   __FILE__, __LINE__, (_n < 0 ? "(refused)" : _b), (want)); \
            g_failures++; \
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
    // #675: the ascent profile. q is the hull's dynamic pressure and the totals
    // are what its three burns have cost -- all readings, none of them derived
    // here. gamma is degrees, and 12.50 deg up means the hull is climbing.
    s.q = 3720.5; s.gamma = 12.50;
    s.dv_exp = 2380.5; s.grav = 612.4; s.drag = 140.7; s.steer = 38.5;
    s.burns = 3;
    s.seams = 0;   // a single hull, mated to nothing
    return s;
}

// The v4 tail of a full reading, spelled once so the six rows below cannot drift
// apart from each other by a digit and hide a real ordering bug.
static const char *kTail = " q=3720.5 gamma=12.50 dv_exp=2380.5 grav=612.4"
                           " drag=140.7 steer=38.5 burns=3 seams=0";

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
              "T=288.1 p=101325 a=340.3 mach=0.100 mass=6000.0 thr=0.75 stg=2"
              " q=3720.5 gamma=12.50 dv_exp=2380.5 grav=612.4 drag=140.7"
              " steer=38.5 burns=3 seams=0");

    // --- on the ground: no air, so no flow group either -------------------
    // The air-relative readings are meaningless with rho = 0, and the flow
    // geometry needs a flow to be measured against. Everything else keeps its
    // zero: mass, throttle, stage, altitude, speed. Note the frame group keeps
    // printing v_orb while v and v_surf read 0: a hull parked on a rotating body
    // IS moving at 175 m/s over the ground's own frame, and a recorder that
    // called that "not a reading" would hide the difference between the two.
    // q reads 0 with rho 0, and the totals read 0 because nothing has burned --
    // both are measurements, not gaps. gamma dashes: parked on a body, this
    // sample has no flight path of its own to report.
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
    pad.q = 0.0; pad.gamma = NAN;
    pad.dv_exp = pad.grav = pad.drag = pad.steer = 0.0; pad.burns = 0;
    CHECK_ROW(pad,
              "t=0.00 alt_asl=0.0 alt_agl=0.0 v=0.00 v_surf=0.00 "
              "v_orb=175.00 vs=0.00 hs=0.00 lat=0.000000 lon=0.000000 "
              "pitch=0.0 roll=0.0 hdg=0.0 dv=1450.5 acc=0.00 "
              "rho=0 F=0.00 L=0.00 Cd=0.950 A=3.20 "
              "aoa=- beta=- cpl=- ind=- chute=0.00 T=- p=- a=- mach=- "
              "mass=6000.0 thr=0.00 stg=0"
              " q=0.0 gamma=- dv_exp=0.0 grav=0.0 drag=0.0 steer=0.0 burns=0 seams=0");

    // --- seams= marks where the hull itself changed size (#972) ----------
    /* A dock or an undock does not change which hull is sampled: updateDocking's
       absorber IS the flown ship and retire() takes the target, and undock mints a
       new vessel with extractSubtreeAsShip while leaving Game::ship alone. So unlike
       a hand-off of control, this is not a gap in the recorder's history -- it is
       one hull with different parts, and mass, A, Cd, chute and dv jump without
       anything in the row saying where. seams= is that mark, and it is a count, so
       it never dashes: a free hull reads 0 and 0 IS the reading. */
    {
        RecSample docked = cruise();
        docked.seams = 2;   // two joints: a truss mated at both ends counts twice
        char buf[1024];
        const int n = recRow(docked, buf, sizeof buf);
        if(n < 0 || std::strstr(buf, " burns=3 seams=2") == nullptr) {
            printf("FAIL %s:%d: seams= did not print the hull's dock count: %s\n",
                   __FILE__, __LINE__, (n < 0 ? "(refused)" : buf));
            g_failures++;
        }
    }

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
              "T=288.1 p=101325 a=340.3 mach=0.100 mass=6000.0 thr=0.75 stg=2"
              " q=3720.5 gamma=12.50 dv_exp=2380.5 grav=612.4 drag=140.7"
              " steer=38.5 burns=3 seams=0");

    // --- a zero is a reading, not a gap ----------------------------------
    // The same rule the v3 columns follow: a hull with nothing spent prints 0 for
    // the totals, not "-".
    RecSample nosurf = cruise();
    nosurf.L = 0.0; nosurf.ind = 0.0; nosurf.beta = 0.0;
    CHECK_ROW(nosurf,
              "t=12.50 alt_asl=700.0 alt_agl=698.5 v=78.00 v_surf=77.75 "
              "v_orb=252.75 vs=-0.25 hs=77.75 lat=-0.094300 lon=30.500000 "
              "pitch=2.5 roll=-0.5 hdg=91.5 dv=1450.5 acc=3.75 "
              "rho=1.225 F=210.50 L=0.00 Cd=0.950 "
              "A=3.20 aoa=2.3 beta=0.0 cpl=0.25 ind=0.000 chute=0.00 "
              "T=288.1 p=101325 a=340.3 mach=0.100 mass=6000.0 thr=0.75 stg=2"
              " q=3720.5 gamma=12.50 dv_exp=2380.5 grav=612.4 drag=140.7"
              " steer=38.5 burns=3 seams=0");

    /* --- a flow with AoA but no yaw (#787) --------------------------------
       Beta's floor is not AoA's: a hull parked nose-up in wind has plenty of flow
       and no sideslip to name. If the row dashed beta only when AoA dashed, this
       sample would print "nan" -- a word in a column where every other missing
       value is "-", which is precisely what one format string per row buys. */
    RecSample broadside = cruise();
    broadside.beta = NAN;
    CHECK_ROW(broadside,
              "t=12.50 alt_asl=700.0 alt_agl=698.5 v=78.00 v_surf=77.75 "
              "v_orb=252.75 vs=-0.25 hs=77.75 lat=-0.094300 lon=30.500000 "
              "pitch=2.5 roll=-0.5 hdg=91.5 dv=1450.5 acc=3.75 "
              "rho=1.225 F=210.50 L=12.00 Cd=0.950 "
              "A=3.20 aoa=2.3 beta=- cpl=0.25 ind=0.020 chute=0.00 "
              "T=288.1 p=101325 a=340.3 mach=0.100 mass=6000.0 thr=0.75 stg=2"
              " q=3720.5 gamma=12.50 dv_exp=2380.5 grav=612.4 drag=140.7"
              " steer=38.5 burns=3 seams=0");

    /* --- a flight path to report, and none (#675) --------------------------
       gamma's floor is not the snapshot's. A hull sitting on a body that does not
       turn has a full set of windows -- position, attitude, the three speeds -- and
       no velocity above orbit.h's conic floor to have an angle THROUGH. Dashing it
       with the frame group would throw away ten good columns; printing 0.00 would
       claim a level flight that nothing measured. This is the same shape as #787's
       beta rule, one struct over. */
    RecSample noPath = cruise();
    noPath.gamma = NAN;
    CHECK_ROW(noPath,
              "t=12.50 alt_asl=700.0 alt_agl=698.5 v=78.00 v_surf=77.75 "
              "v_orb=252.75 vs=-0.25 hs=77.75 lat=-0.094300 lon=30.500000 "
              "pitch=2.5 roll=-0.5 hdg=91.5 dv=1450.5 acc=3.75 "
              "rho=1.225 F=210.50 L=12.00 Cd=0.950 "
              "A=3.20 aoa=2.3 beta=-1.0 cpl=0.25 ind=0.020 chute=0.00 "
              "T=288.1 p=101325 a=340.3 mach=0.100 mass=6000.0 thr=0.75 stg=2"
              " q=3720.5 gamma=- dv_exp=2380.5 grav=612.4 drag=140.7"
              " steer=38.5 burns=3 seams=0");

    // --- no snapshot for this hull: the frame group dashes, together ------
    // The physics group is untouched: lastDrag* belongs to the hull whatever the
    // render pass has been doing. This is the row a recorder writes in the tick
    // before the first refresh, or the one for a hull the view never described.
    // gamma rides that dash because it comes from the same snapshot (tick.cpp), and
    // the q/totals do not because they do not.
    RecSample stale = cruise();
    stale.alt_agl = stale.v_surf = stale.v_orb = stale.vs = stale.hs = NAN;
    stale.lat = stale.lon = stale.pitch = stale.roll = stale.hdg = NAN;
    stale.gamma = NAN;
    CHECK_ROW(stale,
              "t=12.50 alt_asl=700.0 alt_agl=- v=78.00 v_surf=- "
              "v_orb=- vs=- hs=- lat=- lon=- pitch=- roll=- hdg=- "
              "dv=1450.5 acc=3.75 "
              "rho=1.225 F=210.50 L=12.00 Cd=0.950 "
              "A=3.20 aoa=2.3 beta=-1.0 cpl=0.25 ind=0.020 chute=0.00 "
              "T=288.1 p=101325 a=340.3 mach=0.100 mass=6000.0 thr=0.75 stg=2"
              " q=3720.5 gamma=- dv_exp=2380.5 grav=612.4 drag=140.7"
              " steer=38.5 burns=3 seams=0");

    // --- the header's channel list and the row's keys are the same list ----
    {
        char buf[1024];
        const int n = recRow(cruise(), buf, sizeof buf);
        if(n < 0) {
            printf("FAIL %s:%d: a full row was refused\n", __FILE__, __LINE__);
            g_failures++;
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
                    g_failures++;
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
                g_failures++;
            }
            // The tail spelled at the top of this file is the same tail the row
            // prints -- so a column added to the header without the row, or in a
            // different order, fails here as well as above.
            if(row.find(kTail) == std::string::npos) {
                printf("FAIL %s:%d: the row does not end in the pinned v4 tail"
                       " (\"%s\"): %s\n", __FILE__, __LINE__, kTail, buf);
                g_failures++;
            }
        }
    }

    // --- a row that does not fit is refused, not truncated ----------------
    {
        char small[32];
        if(recRow(cruise(), small, sizeof small) >= 0) {
            printf("FAIL %s:%d: a 32-byte buffer accepted a full row\n", __FILE__, __LINE__);
            g_failures++;
        }
    }

    // --- the Recorder window's ring (#972) --------------------------------
    // The plot reads this ring and the rows above come from the same sample, so
    // the ring has to hold the same NaN-dashes and the same stage marks.
    {
        // Cap is a MISSION-TIME decision: the ring has to hold a whole ascent or
        // the window drops the launch while the hull is still climbing. 480 s is
        // what test_guidance's own ascent takes to a 100 km periapsis, at the
        // 0.25 s default of --record-interval. Pinned as a span rather than only
        // as bytes, because a Cap that quietly halves is invisible in a bytes
        // check. The byte half still matters: RecRing lives on Game for the whole
        // run, and the eight TimeSeries HudState carries are the yardstick (2 MB).
        CHECK(RecRing::Cap * 0.25 >= 480.0, "the ring no longer spans an ascent");
        CHECK(sizeof(RecPoint) * RecRing::Cap <= 2 * 1024 * 1024,
              "the recorder ring blew its memory budget");

        RecRing r;
        CHECK(r.count == 0, "a fresh ring holds nothing");

        // Fill past the cap, one mission second apart, with a stage press
        // half-way through the span.
        for(int i = 0; i < RecRing::Cap + 7; i++) {
            RecPoint pt;
            pt.sim_t = 10.0 + i;
            pt.s.t = i * 0.25;                        // the run-clock column
            pt.s.alt_asl = 1000.0 + i;
            pt.s.stg = (i < RecRing::Cap / 2) ? 1 : 2;
            r.push(pt);
        }
        CHECK(r.count == RecRing::Cap, "the ring stops at Cap");
        // push() must not grow the vector after the first one: step 2 will hand
        // the plot pointers taken from at(), and a reallocation dangles them.
        CHECK(r.p.capacity() == (size_t)RecRing::Cap, "push() never reallocates");
        // The whole sample travels, not only its x.
        CHECK_NEAR(r.at(0).s.t, 7 * 0.25, 1e-12, "the sample's own columns survive");
        CHECK_NEAR(r.at(0).s.alt_asl, 1007.0, 1e-12, "channel values survive the wrap");
        CHECK_NEAR(r.at(0).sim_t, 17.0, 1e-12, "the 7 oldest fell off");
        CHECK_NEAR(r.at(RecRing::Cap - 1).sim_t, 10.0 + RecRing::Cap + 6, 1e-12,
                   "at() ends on the newest sample");
        bool mono = true;
        for(int i = 1; i < r.count; i++) {
            if(!(r.at(i).sim_t > r.at(i - 1).sim_t)) { mono = false; break; }
        }
        CHECK(mono, "at() is oldest-first across the wrap");
        // The stage press is DATA inside the span, not a reset and not a split:
        // an ascent that goes through staging is one story. The ring is blind to
        // stg by design; this is the pin for whoever is tempted to make it
        // stage-aware because "a new hull needs a new plot" (it does not --
        // staging keeps Game::ship, see game.cpp's select_vehicle comment).
        int stage_edges = 0;
        for(int i = 1; i < r.count; i++) {
            if(r.at(i).s.stg != r.at(i - 1).s.stg) { stage_edges++; }
        }
        CHECK(stage_edges == 1, "a stage change neither clears nor splits the ring");

        // A NaN survives as a NaN -- it is how a channel says "no reading" (aoa
        // waits for a flow worth reading), and the plot needs the gap rather than
        // a zero, which is what recRow's dashes do for the file.
        RecPoint dash;
        dash.sim_t = 5000.0;
        dash.s.aoa = NAN; dash.s.gamma = NAN; dash.s.beta = NAN;
        dash.s.alt_asl = 0.0;
        r.push(dash);
        CHECK(std::isnan(r.at(r.count - 1).s.aoa), "a dashed channel keeps its gap");
        CHECK(std::isnan(r.at(r.count - 1).s.gamma), "gamma keeps its own gap");
        CHECK(!std::isnan(r.at(r.count - 1).s.alt_asl), "a plain 0 stays a reading");
        CHECK(r.count == RecRing::Cap, "the ring never exceeds Cap");

        // Paused: the mission clock does not advance, so the last point is
        // refreshed rather than duplicated -- what TimeSeries does (siminput.h),
        // and what keeps a long pause from eating the ring.
        const int before = r.count;
        RecPoint held;
        held.sim_t = r.at(before - 1).sim_t;
        held.s.alt_asl = 4242.0;
        r.push(held);
        CHECK(r.count == before, "a paused tick adds no point at the same x");
        CHECK_NEAR(r.at(r.count - 1).s.alt_asl, 4242.0, 1e-12,
                   "a same-x push refreshes rather than appends");

        // A load clears: g.time runs backwards across one (#304), so samples kept
        // from the dead world would leave a non-monotonic x axis.
        r.clear();
        CHECK(r.count == 0, "clear empties the ring");
        RecPoint fresh;
        fresh.sim_t = 0.5; fresh.s.alt_asl = 5.0;
        r.push(fresh);
        CHECK(r.count == 1, "pushing after clear starts again at the oldest slot");
        CHECK_NEAR(r.at(0).sim_t, 0.5, 1e-12);

        /* A backwards sim_t is not refused, so pin what it DOES instead: the point
           appends and the ring holds a non-monotonic x (recorder.h says this is
           deliberate -- no assert). It should be unreachable: g.time moves only at
           tick.cpp's `g.time += g.dt * g.time_accel` and Game::setTime, and every
           setTime caller is followed by clearEphemeralState. So this is the pin for
           that invariant, written from the ring's side -- if a future refactor does
           move the clock under a live ring, the window sees a stroke back over
           itself, which is loud, rather than a silently re-sorted plot. */
        r.clear();
        RecPoint up;
        up.sim_t = 5.0; up.s.alt_asl = 1.0;
        RecPoint back;
        back.sim_t = 4.0; back.s.alt_asl = 2.0;
        r.push(up);
        r.push(back);
        CHECK(r.count == 2, "a backwards push still appends");
        CHECK_NEAR(r.at(1).sim_t, 4.0, 1e-12, "at() stays write order, never x order");
    }

    /* --- the channels the window can draw (#972 step 2) -------------------
       kRecChans is what the Recorder window's toggles, legend rows and scaling
       are built from, so a table entry that is not a trace column is a graph
       showing a number the file cannot back up. The keys must also be distinct:
       one key is the toggle label, the legend row and the plot item's label at
       once, and implot identifies an item by hashing that whole string -- a
       duplicate would quietly draw two channels as one line. */
    {
        const std::string chans = recChannels();
        for(int i = 0; i < kRecNumCh; i++) {
            const std::string key(kRecChans[i].key);
            /* Whole-word, not substring: "v" lives inside "v_surf" and "dv"
               inside "dv_exp", so a bare find() would pass an entry naming no
               column of the trace at all. */
            bool whole = false;
            for(size_t p = chans.find(key); p != std::string::npos; p = chans.find(key, p + 1)) {
                const bool left_ok = p == 0 || chans[p - 1] == ' ';
                const size_t e = p + key.size();
                if(left_ok && (e == chans.size() || chans[e] == ' ')) { whole = true; break; }
            }
            CHECK(whole, "every kRecChans key is a recChannels() column");
            for(int j = i + 1; j < kRecNumCh; j++) {
                CHECK(key != std::string(kRecChans[j].key), "channel keys are unique");
            }
            /* Non-empty, because the whole-word scan above passes an empty key: ""
               is "found" at position 0 of every string. It is a checkbox label and
               a plot item label, so an empty one is a widget nobody can read. */
            CHECK(!key.empty(), "a channel key is a readable label");
            /* And no '#' anywhere: the key goes verbatim to ImGui::Checkbox and to
               ImPlot::PlotLineG, and both cut display text at "##"
               (imgui.cpp:3880 FindRenderedTextEnd), so a key carrying one would
               render a blank toggle and an item with no legend row. */
            CHECK(key.find('#') == std::string::npos,
                  "a channel key survives imgui's and implot's label grammar");
        }
        /* Curated on purpose (recorder.h): the trace has more columns than the
           window offers, and a table that grew to all of them would be a list of
           38 checkboxes no pilot reads. */
        CHECK(kRecNumCh < 20, "the window's channel list stays a subset of the trace");
    }

    /* --- recExtent: the range the plot scales by, and the legend's numbers --
       One graph over thirteen units means every line is drawn against its own
       min..max, so this is the arithmetic the graph is made of -- and the two
       traps are the ones the row grammar already refuses: a dash is not a zero,
       and a flat reading has no range to divide by. */
    {
        double mn = 0.0, mx = 0.0;
        int nread = 0;

        RecRing r;
        const double vals[3] = {10.0, -3.5, 7.25};
        for(int i = 0; i < 3; i++) {
            RecPoint p;
            p.sim_t = (double)i; p.s.vs = vals[i];
            r.push(p);
        }
        recExtent(r, &RecSample::vs, &mn, &mx, &nread);
        CHECK_NEAR(mn, -3.5, 1e-12, "the extent starts at the smallest reading");
        CHECK_NEAR(mx, 10.0, 1e-12, "the extent ends at the largest");
        CHECK(nread == 3, "every number counts");

        // A dash must never become the minimum: 0 is a reading and NaN is not.
        RecPoint dash;
        dash.sim_t = 3.0; dash.s.vs = NAN;
        r.push(dash);
        recExtent(r, &RecSample::vs, &mn, &mx, &nread);
        CHECK_NEAR(mn, -3.5, 1e-12, "a NaN reading never sets the min");
        CHECK(nread == 3, "a dash is not a reading");

        // Nor is an Inf. It is worse than a dash if it gets through: one Inf makes
        // the RANGE itself NaN, and a NaN range normalises the whole line away
        // (k = 0) while nread still reports readings -- so the graph would draw
        // nothing and the window would say nothing either.
        RecPoint inf_pt;
        inf_pt.sim_t = 4.0; inf_pt.s.vs = INFINITY;
        r.push(inf_pt);
        recExtent(r, &RecSample::vs, &mn, &mx, &nread);
        CHECK_NEAR(mx, 10.0, 1e-12, "an Inf reading never sets the max");
        CHECK(mx - mn == 13.5, "the range stays a number with an Inf in the ring");
        CHECK(nread == 3, "an Inf is not a reading either");

        // A channel that has never been readable says so rather than plotting a
        // flat line at 0 (the window counts these and prints how many under the
        // plot -- it cannot say it in the legend row, where the label has to stay
        // static text).
        r.clear();
        RecPoint only_dash;
        only_dash.sim_t = 1.0; only_dash.s.gamma = NAN;
        r.push(only_dash);
        recExtent(r, &RecSample::gamma, &mn, &mx, &nread);
        CHECK(nread == 0, "an all-dash channel has no extent");

        // A flat channel has no range: the window must not divide by it, and the
        // legend still has to print the value it is flat at.
        RecRing flat;
        for(int i = 0; i < 3; i++) {
            RecPoint p;
            p.sim_t = (double)i; p.s.thr = 1.0;
            flat.push(p);
        }
        recExtent(flat, &RecSample::thr, &mn, &mx, &nread);
        CHECK_NEAR(mx - mn, 0.0, 1e-12, "a flat channel has no range");
        CHECK_NEAR(mn, 1.0, 1e-12, "and its extent still names the value");

        /* Across the wrap both runs count, and each held sample counts exactly
           once: the window scales the two runs by the SAME range (recorder.h), so
           an extent that read only one of them would bend the line at the seam. */
        RecRing w;
        for(int i = 0; i < RecRing::Cap + 5; i++) {
            RecPoint p;
            p.sim_t = (double)i; p.s.q = (double)i;
            w.push(p);
        }
        CHECK(w.count == RecRing::Cap, "the wrap test filled the ring");
        recExtent(w, &RecSample::q, &mn, &mx, &nread);
        CHECK(nread == RecRing::Cap, "a wrapped ring counts each held sample once");
        CHECK_NEAR(mn, 5.0, 1e-12, "the aged-out samples leave the extent with the ring");
        CHECK_NEAR(mx, (double)RecRing::Cap + 4, 1e-12, "the newest sample sets the max");
    }

    /* --- spans(): the runs the plot actually draws (the window plots each one
       separately, so the contract is index math, not pixels) --------------- */
    {
        RecRing r;
        int idx[2], len[2];
        CHECK(r.spans(idx, len) == 0, "an empty ring hands back no runs");

        for(int i = 0; i < 3; i++) {
            RecPoint p;
            p.sim_t = (double)i; p.s.q = (double)i;
            r.push(p);
        }
        CHECK(r.spans(idx, len) == 1, "an unwrapped ring is one run");
        CHECK(idx[0] == 0 && len[0] == 3, "the run is the whole ring, oldest first");

        /* Filled exactly to Cap is NOT wrapped: head has come round to 0 and the
           ring is one contiguous run again. Getting this wrong draws the newest
           sample first, which is a stroke back across the flight. */
        while(r.count < RecRing::Cap) {
            RecPoint p;
            p.sim_t = (double)r.count; p.s.q = (double)r.count;
            r.push(p);
        }
        CHECK(r.spans(idx, len) == 1, "a full but unwrapped ring stays one run");
        CHECK(idx[0] == 0 && len[0] == RecRing::Cap, "and it holds every slot");

        for(int i = 0; i < 5; i++) {
            RecPoint p;
            p.sim_t = (double)(RecRing::Cap + i); p.s.q = (double)(RecRing::Cap + i);
            r.push(p);
        }
        CHECK(r.spans(idx, len) == 2, "past Cap the ring is two runs");
        CHECK(len[0] + len[1] == r.count, "the two runs are the ring, with no gap or overlap");
        CHECK(idx[0] == 5 && len[0] == RecRing::Cap - 5, "the older run starts where the next write lands");
        CHECK(idx[1] == 0 && len[1] == 5, "the newer run is the head of the vector");
        // Reading the runs in order must agree with at(), which is the definition
        // of oldest-first the window relies on.
        int k = 0;
        bool same = true;
        for(int s = 0; s < 2; s++) {
            for(int i = 0; i < len[s]; i++) {
                if(r.p[(size_t)idx[s] + (size_t)i].sim_t != r.at(k).sim_t) { same = false; }
                k++;
            }
        }
        CHECK(same && k == r.count, "spans walk == at() walk");
    }

    /* --- recMarks(): where the flight's chapters turn over ----------------
       The window draws these as vertical lines. They are data logic, not pixels,
       so they belong with the ring: a mark is a CHANGE in stg= or seams=, which is
       the same rule that made v4 append seams= to the row in the first place. */
    {
        auto push_at = [](RecRing &r, double t, int stg, int seams) {
            RecPoint p;
            p.sim_t = t; p.s.stg = stg; p.s.seams = seams;
            r.push(p);
        };
        double xs[8];

        RecRing r;
        push_at(r, 0.0, 1, 0);
        push_at(r, 1.0, 1, 0);
        push_at(r, 2.0, 2, 0);      // stage about to fire moved: a mark at t=2
        push_at(r, 3.0, 2, 1);      // hull grew a joint: a seam mark at t=3
        push_at(r, 4.0, 2, 0);      // and shrank one: another seam mark at t=4
        CHECK(recMarks(r, false, xs, 8) == 1, "one stage change makes one mark");
        CHECK_NEAR(xs[0], 2.0, 1e-12, "and it lands on the sample that changed");
        CHECK(recMarks(r, true, xs, 8) == 2, "a mate and a separation are both marks");
        CHECK_NEAR(xs[0], 3.0, 1e-12, "seam marks ignore the stage change");
        CHECK_NEAR(xs[1], 4.0, 1e-12, "seams go down as well as up, so a set of values would miss it");

        // Nothing changed: no marks, and the ring is not misread as one big change.
        RecRing flat;
        for(int i = 0; i < 6; i++) { push_at(flat, (double)i, 2, 1); }
        CHECK(recMarks(flat, false, xs, 8) == 0, "a steady stage draws no line");
        CHECK(recMarks(flat, true, xs, 8) == 0, "a steady hull size draws no line");

        /* The cap: it bounds the drawing, and returning max is how the caller knows
           to admit that later chapter changes were dropped. */
        RecRing many;
        for(int i = 0; i < 20; i++) { push_at(many, (double)i, i, 0); }
        CHECK(recMarks(many, false, xs, 4) == 4, "the cap bounds the marks");
        CHECK_NEAR(xs[0], 1.0, 1e-12, "and keeps the earliest ones");

        // Across the wrap the marks are still oldest-first, and the transition that
        // straddles the seam is still found.
        RecRing w;
        for(int i = 0; i < RecRing::Cap + 3; i++) { push_at(w, (double)i, i / 100, 0); }
        double big[256];
        const int n = recMarks(w, false, big, 256);
        CHECK(n > 0, "a wrapped ring still has its marks");
        bool asc = true;
        for(int i = 1; i < n; i++) { if(big[i] <= big[i - 1]) { asc = false; } }
        CHECK(asc, "and they come out in mission-time order");
        CHECK(recMarks(w, true, big, 256) == 0, "seams that never moved draw nothing across a wrap");

        // Empty ring: no marks, and nothing is read out of the vector.
        RecRing empty;
        CHECK(recMarks(empty, false, xs, 8) == 0, "an empty ring has no marks");
    }

    /* --- recTimeExtent(): the x range the plot pins its axis to ----------- */
    {
        RecRing r;
        double t0 = 0.0, t1 = 0.0;
        for(int i = 0; i < 4; i++) {
            RecPoint p;
            p.sim_t = 10.0 + (double)i;
            r.push(p);
        }
        recTimeExtent(r, &t0, &t1);
        CHECK_NEAR(t0, 10.0, 1e-12, "the axis starts at the oldest sample");
        CHECK_NEAR(t1, 13.0, 1e-12, "and ends at the newest");

        /* min/max rather than ends: the ring keeps write order, and a load moves
           the mission clock underneath it, so reading the ends would clip half the
           flight instead of showing the stroke back (recorder.h says the ring does
           not refuse a backwards push). */
        RecPoint back;
        back.sim_t = 8.0;
        r.push(back);
        recTimeExtent(r, &t0, &t1);
        CHECK_NEAR(t0, 8.0, 1e-12, "a backwards sample still sets the low end");
        CHECK_NEAR(t1, 13.0, 1e-12, "and the high end stays");

        // Across the wrap both runs count.
        RecRing w;
        for(int i = 0; i < RecRing::Cap + 7; i++) {
            RecPoint p;
            p.sim_t = (double)i;
            w.push(p);
        }
        recTimeExtent(w, &t0, &t1);
        CHECK_NEAR(t0, 7.0, 1e-12, "the run that aged out leaves the range");
        CHECK_NEAR(t1, (double)RecRing::Cap + 6, 1e-12, "the newest run sets the top");
    }

    if(g_failures == 0) {
        printf("test_recorder: all checks passed\n");
        return 0;
    }
    printf("test_recorder: %d FAILURE(S)\n", g_failures);
    return 1;
}
