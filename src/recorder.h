// recorder.h -- the flight recorder's channel table and row grammar (#662, #674).
//
// A recorder row is one line in the flight logs' grammar ("t=... key=value ..."),
// so the harness that already parses [drag]/[orbit]/[ascent] rows parses a trace
// too, and the version plus the channel names ride in the file's header line -- the
// file stays readable after the code that wrote it is gone.
//
// Every channel is a number the game ALREADY publishes, at one of two cadences:
// the physics pass's own readouts (lastDrag*, lastAir*, feltAccel) and the frame
// readouts in ShipView, which are what the SURFACE and ORBITAL windows show. The
// recorder computes nothing, which is the point: a recorded value and a logged one
// cannot disagree -- the same one-seam law #665 put on the instrument parts.

#pragma once

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <initializer_list>
#include <vector>

// v2 (#674) added the frame readouts: the altitude pair, the three speeds, position,
// attitude, delta-v and felt acceleration. Nothing was renamed or removed, so a v1
// reader keeps every key it knew.
// v3 (#675) added the ascent-profile numbers the physics did not publish until now:
// q, the flight-path angle, and the four burn totals -- all of them a reading the
// flight model keeps, none of them computed here.
// v4 (#972) added seams: how many dock joints the hull carries. A recorder row has
// always said WHICH stage was about to fire, and never said when the hull itself
// changed size -- so a dock or an undock shows up as a cliff in mass, area, Cd and dv
// with nothing marking it. seams= is the game's own count (Vehicle::seams), appended
// like every earlier version did: nothing renamed, nothing removed, so a v3 reader
// keeps every key it knew.
static const int kRecVersion = 4;

// The channels, in row order: the header line's contract with a reader.
inline const char *recChannels() {
    return "t alt_asl alt_agl v v_surf v_orb vs hs lat lon pitch roll hdg dv acc"
           " rho F L Cd A aoa beta cpl ind chute T p a mach mass thr stg"
           " q gamma dv_exp grav drag steer burns seams";
}

// The dash rules come from where a number comes from, not from how sure the pass
// sounded (#560):
//   * aoa/cpl/ind wait for a flow worth reading (lastDragAlphaValid, #560); beta waits for
//     a flow whose YAW is worth reading (lastDragBetaValid, #787), which is the stricter
//     test -- a flow up a hull's mast gives AoA -90 and no yaw at all. T/p/a/mach wait for
//     air, because drag.h answers 0.0 wherever there is no air and that is not a
//     temperature to record (00db083).
//   * The frame group -- alt_agl, the three speeds, lat/lon, pitch/roll/hdg -- waits
//     for a snapshot that describes THIS hull. An unfilled ShipView reads zeros
//     (#235), and a 0 in a log gets treated as a measurement, which is the same
//     mistake kAoAFlowFloor exists to stop.
//   * A 0 elsewhere is a reading and stays printed: a parked chute IS 0 m2, a
//     coasting hull IS 0 m/s2, a hull with no rocket HAS 0 m/s.
// A NaN in a RecSample field means "dash this column"; only the channels named above are
// ever NaN on purpose, so a plain channel keeps printing whatever was published --
// if that is a NaN, the trace should show it rather than hide it.
struct RecSample {
    double t = 0.0;        // run seconds: the log clock since THIS trace began
    double alt_asl = 0.0;  // metres above the datum (lastDragAlt)
    double alt_agl = 0.0;  // metres above the ground under the hull (ShipView)
    double v = 0.0;        // |velocity in the physics frame| -- [drag]'s |v|
    double v_surf = 0.0;   // surface frame: what the ground sees go past
    double v_orb = 0.0;    // the conic's speed -- ORBITAL window's Speed
    double vs = 0.0;       // vertical speed, + = climbing
    double hs = 0.0;       // horizontal (surface) speed
    double lat = 0.0, lon = 0.0;
    double pitch = 0.0, roll = 0.0, hdg = 0.0;   // surface-relative (render.cpp)
    /* The coarse one-burn figure (Vehicle::getDeltaV), NOT the Vessel Info window's staged plan:
       a telemetry row is one part in ~2000 of a recording, and #948 is about the readout a pilot
       watches, not this column. Note it can rise when a stage separates (#948's diagnosis), so a
       script reading this column should not assume monotonicity. */
    double dv = 0.0;
    double acc = 0.0;      // felt acceleration: |thrust + aero| / mass
    double rho = 0.0;      // air density the flight model stood in, kg/m3
    double F = 0.0;        // |aero force| (lift + drag), N
    double L = 0.0;        // |lift force|, N
    double Cd = 0.0;
    double A = 0.0;        // body silhouette facing the flow, m2
    double aoa = 0.0, beta = 0.0, cpl = 0.0, ind = 0.0;
    double chute = 0.0;    // always a number: a parked chute IS 0 m2
    double T = 0.0, p = 0.0, a = 0.0, mach = 0.0;
    double mass = 0.0;     // kg, fuel and crew included
    double thr = 0.0;      // throttle, 0..1
    int stg = 0;           // the stage about to fire: the timeline's chapter marks
    /* #675: the ascent profile, all of it published by the flight model rather than
       worked out here. q is the whole-hull dynamic pressure (lastDynPressure: 1/2 rho
       v^2 at the COM, v against the AIR, not the ground or the conic), so it reads 0
       where there is no air -- that IS the reading. Reconstructing it from this row's
       v or v_orb is a different number whenever the air moves. gamma is ShipView's
       flight-path angle in degrees, and the one field in this struct that arrives
       NaN on purpose: a hull with no velocity above orbit.h's conic floor has no
       angle, and plotting the contact solver's residue as level flight is worse.
       The four totals are the hull's running sums since it existed (losses.h), and
       burns= is how many burns they cover -- the per-burn identity, so a reader can
       split a total by differencing across a burns= boundary. */
    double q = 0.0;
    double gamma = 0.0;
    double dv_exp = 0.0;
    double grav = 0.0;
    double drag = 0.0;
    double steer = 0.0;
    int burns = 0;
    /* #972: how many dock joints the hull carries (Vehicle::seams). Always a number,
       never dashed: a hull mated to nothing IS 0 joints, and that is a reading. A
       change in it is where the hull itself grew or shrank, which is the mark stg=
       gives a stage press and a dock or undock has never had: mass, A, Cd, chute and
       dv all jump there, and a plot showing that jump cannot say why it jumped. */
    int seams = 0;
};

// One row, no newline, into buf. Returns the row's length, or -1 when it did not
// fit: snprintf does not write past the buffer, but it DOES report the length it
// wanted, and a half-written row is a parse error in someone else's script. One
// format string on purpose: the row grammar is one thing to read and one thing
// to change.
inline int recRow(const RecSample &s, char *buf, size_t n) {
    if(buf == nullptr || n == 0) { return -1; }
    char aoa_txt[32], beta_txt[32], cpl_txt[32], ind_txt[32];
    char t_txt[32], p_txt[32], a_txt[32], mach_txt[32];
    const bool flow = !std::isnan(s.aoa);   // one test, as --drag-log does
    if(flow) {
        snprintf(aoa_txt, sizeof aoa_txt, "%.1f", s.aoa);
        snprintf(cpl_txt, sizeof cpl_txt, "%.2f", s.cpl);
        snprintf(ind_txt, sizeof ind_txt, "%.3f", s.ind);
    } else {
        snprintf(aoa_txt, sizeof aoa_txt, "-");
        snprintf(cpl_txt, sizeof cpl_txt, "-");
        snprintf(ind_txt, sizeof ind_txt, "-");
    }
    /* beta is NOT gated by `flow`: a flow can be fast enough to have an angle of
       attack and still have no yaw to report, because it lies along the hull's up
       axis (#787). Reusing the AoA test here would print "nan" into a column
       whose every other missing value is "-", and a NaN in a numeric column is
       exactly the parse error this row's one-format-string rule exists to avoid. */
    if(!std::isnan(s.beta)) {
        snprintf(beta_txt, sizeof beta_txt, "%.1f", s.beta);
    } else {
        snprintf(beta_txt, sizeof beta_txt, "-");
    }
    if(!std::isnan(s.T)) {
        snprintf(t_txt, sizeof t_txt, "%.1f", s.T);
        snprintf(p_txt, sizeof p_txt, "%.0f", s.p);
        snprintf(a_txt, sizeof a_txt, "%.1f", s.a);
        snprintf(mach_txt, sizeof mach_txt, "%.3f", s.mach);
    } else {
        snprintf(t_txt, sizeof t_txt, "-");
        snprintf(p_txt, sizeof p_txt, "-");
        snprintf(a_txt, sizeof a_txt, "-");
        snprintf(mach_txt, sizeof mach_txt, "-");
    }
    /* gamma dashes on its own NaN, the same shape as aoa's rule (#675): "this state
       has no flight path" is not a claim that the hull is flying level. It does NOT
       ride the frame group's shared test below, because it can be absent while the
       snapshot describes this hull -- a parked ship on a body that does not turn has
       a full set of windows and no angle. */
    char gam_txt[32];
    if(!std::isnan(s.gamma)) { snprintf(gam_txt, sizeof gam_txt, "%.2f", s.gamma); }
    else { snprintf(gam_txt, sizeof gam_txt, "-"); }
    // The frame group dashes as a group: one snapshot describes all ten or none.
    char agl_txt[32], vsurf_txt[32], vorb_txt[32], vs_txt[32], hs_txt[32];
    char lat_txt[32], lon_txt[32], pit_txt[32], rol_txt[32], hdg_txt[32];
    if(!std::isnan(s.alt_agl)) {
        snprintf(agl_txt, sizeof agl_txt, "%.1f", s.alt_agl);
        snprintf(vsurf_txt, sizeof vsurf_txt, "%.2f", s.v_surf);
        snprintf(vorb_txt, sizeof vorb_txt, "%.2f", s.v_orb);
        snprintf(vs_txt, sizeof vs_txt, "%.2f", s.vs);
        snprintf(hs_txt, sizeof hs_txt, "%.2f", s.hs);
        snprintf(lat_txt, sizeof lat_txt, "%.6f", s.lat);
        snprintf(lon_txt, sizeof lon_txt, "%.6f", s.lon);
        snprintf(pit_txt, sizeof pit_txt, "%.1f", s.pitch);
        snprintf(rol_txt, sizeof rol_txt, "%.1f", s.roll);
        snprintf(hdg_txt, sizeof hdg_txt, "%.1f", s.hdg);
    } else {
        for(char *txt : {agl_txt, vsurf_txt, vorb_txt, vs_txt, hs_txt,
                         lat_txt, lon_txt, pit_txt, rol_txt, hdg_txt}) {
            txt[0] = '-';
            txt[1] = '\0';
        }
    }
    const int wrote = snprintf(buf, n,
                               "t=%.2f alt_asl=%.1f alt_agl=%s v=%.2f v_surf=%s"
                               " v_orb=%s vs=%s hs=%s lat=%s lon=%s pitch=%s roll=%s"
                               " hdg=%s dv=%.1f acc=%.2f rho=%.5g F=%.2f L=%.2f"
                               " Cd=%.3f A=%.2f aoa=%s beta=%s cpl=%s ind=%s chute=%.2f"
                               " T=%s p=%s a=%s mach=%s mass=%.1f thr=%.2f stg=%d"
                               " q=%.1f gamma=%s dv_exp=%.1f grav=%.1f drag=%.1f"
                               " steer=%.1f burns=%d seams=%d",
                               s.t, s.alt_asl, agl_txt, s.v, vsurf_txt, vorb_txt,
                               vs_txt, hs_txt, lat_txt, lon_txt, pit_txt, rol_txt,
                               hdg_txt, s.dv, s.acc, s.rho, s.F, s.L, s.Cd, s.A,
                               aoa_txt, beta_txt, cpl_txt, ind_txt, s.chute,
                               t_txt, p_txt, a_txt, mach_txt, s.mass, s.thr, s.stg,
                               s.q, gam_txt, s.dv_exp, s.grav, s.drag, s.steer,
                               s.burns, s.seams);
    if(wrote < 0 || (size_t)wrote >= n) { return -1; }
    return wrote;
}

/* --- the Recorder window's history (#972) --------------------------------
   The same samples the rows above are made of: ONE sampler feeds both sinks,
   because this file's rule is that a recorded number and a logged number cannot
   disagree. A second sampling path for a plot is therefore not on the table --
   the plot reads this ring, and the trace prints the sample that went into it.

   sim_t is the mission clock (g.time) at the sample. The window plots against
   it, so a warp coast stays proportional; the SAMPLING GATE stays run time
   (--record-interval, and #987 is the comment that misstates it), which is why
   the points are sparser under warp rather than more numerous. RecSample::t
   keeps the run-clock meaning the file's rows have always had, so nothing here
   changes the file's grammar.

   PLOT sim_t, NEVER s.t. s.t is the column the FILE prints, counted from the
   trace's opening (tick.cpp rewrites it per row); a graph that reads it is
   silently back on the run clock, which is the one property this field exists to
   buy. Nothing at compile time keeps a step-2 author from making that mistake,
   hence the emphasis. */
struct RecPoint {
    RecSample s;
    double sim_t = 0.0;
};

struct RecRing {
    /* Cap is a MISSION-TIME decision, not a byte one: the ring has to hold a
       whole ascent, or the window drops the launch while the hull is still
       climbing. test_guidance's own ascent runs ~480 s to a 100 km periapsis, so
       at the default 0.25 s gate 4096 samples is ~17 min of mission time -- one
       ascent plus its descent, with headroom for a warped coast (which thins the
       samples rather than adding them). 4096 x sizeof(RecPoint) = 4096 x 320 B =
       1.25 MB on the heap: the same order as the eight TimeSeries HudState already
       carries (2 MB).
       constexpr, not const: a class static with an in-class initializer needs it
       the first time someone odr-uses Cap (std::min(count, RecRing::Cap)). */
    static constexpr int Cap = 4096;
    std::vector<RecPoint> p;   // sized on the first push
    int head = 0;              // next write slot
    int count = 0;             // samples held, never above Cap

    void push(const RecPoint &pt) {
        if(p.empty()) { p.resize(Cap); }
        const int last = (head - 1 + Cap) % Cap;
        /* A paused tick does not advance the mission clock. Refreshing the last
           point rather than storing a second one at the same x is what
           TimeSeries does (siminput.h), and it keeps a long pause from eating
           the ring. Not an assert on a backwards sim_t: a load clears the ring
           (Game::clearEphemeralState), and anything else that moves the clock
           is the plot's problem, not a reason to stop the sim. */
        if(count > 0 && p[last].sim_t == pt.sim_t) { p[last] = pt; return; }
        p[head] = pt;
        head = (head + 1) % Cap;
        if(count < Cap) { count++; }
    }

    // Oldest first: at(0) is the oldest sample still held.
    const RecPoint &at(int i) const {
        // count > 0 implies p was sized by the push that set it, so there is
        // nothing separate to guard here.
        assert(i >= 0 && i < count);
        return p[(head - count + i + Cap) % Cap];
    }

    // Keeps the allocation; count is the only reader, as in TimeSeries.
    void clear() { head = 0; count = 0; }
};
