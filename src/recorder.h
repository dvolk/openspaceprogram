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
// v5 (#947) added the crossflow normal bill (side=, m^2) and its lever (sidecp=, m):
// the reading --drag-log prints as SideCdA=/SideCp=, the push the sides of hidden
// bodies pay that the axial silhouette never billed. Same append law: a v4 reader
// keeps every key it knew.
static const int kRecVersion = 5;

// The channels, in row order: the header line's contract with a reader.
inline const char *recChannels() {
    return "t alt_asl alt_agl v v_surf v_orb vs hs lat lon pitch roll hdg dv acc"
           " rho F L Cd A aoa beta cpl ind side sidecp chute T p a mach mass thr stg"
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
    /* #947: the crossflow normal bill (m^2) and its lever (+ ahead of the COM),
       the reading --drag-log spells SideCdA=/SideCp=. They ride the aoa gate: no
       usable flow, no normal force to quote. */
    double side = 0.0, sidecp = 0.0;
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
    char side_txt[32], sidecp_txt[32];
    char t_txt[32], p_txt[32], a_txt[32], mach_txt[32];
    const bool flow = !std::isnan(s.aoa);   // one test, as --drag-log does
    if(flow) {
        snprintf(aoa_txt, sizeof aoa_txt, "%.1f", s.aoa);
        snprintf(cpl_txt, sizeof cpl_txt, "%.2f", s.cpl);
        snprintf(ind_txt, sizeof ind_txt, "%.3f", s.ind);
        snprintf(side_txt, sizeof side_txt, "%.2f", s.side);
        snprintf(sidecp_txt, sizeof sidecp_txt, "%+.2f", s.sidecp);
    } else {
        snprintf(aoa_txt, sizeof aoa_txt, "-");
        snprintf(cpl_txt, sizeof cpl_txt, "-");
        snprintf(ind_txt, sizeof ind_txt, "-");
        snprintf(side_txt, sizeof side_txt, "-");
        snprintf(sidecp_txt, sizeof sidecp_txt, "-");
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
                               " Cd=%.3f A=%.2f aoa=%s beta=%s cpl=%s ind=%s"
                               " side=%s sidecp=%s chute=%.2f"
                               " T=%s p=%s a=%s mach=%s mass=%.1f thr=%.2f stg=%d"
                               " q=%.1f gamma=%s dv_exp=%.1f grav=%.1f drag=%.1f"
                               " steer=%.1f burns=%d seams=%d",
                               s.t, s.alt_asl, agl_txt, s.v, vsurf_txt, vorb_txt,
                               vs_txt, hs_txt, lat_txt, lon_txt, pit_txt, rol_txt,
                               hdg_txt, s.dv, s.acc, s.rho, s.F, s.L, s.Cd, s.A,
                               aoa_txt, beta_txt, cpl_txt, ind_txt,
                               side_txt, sidecp_txt, s.chute,
                               t_txt, p_txt, a_txt, mach_txt, s.mass, s.thr, s.stg,
                               s.q, gam_txt, s.dv_exp, s.grav, s.drag, s.steer,
                               s.burns, s.seams);
    if(wrote < 0 || (size_t)wrote >= n) { return -1; }
    return wrote;
}

/* --- the channels the Recorder window can draw (#972) --------------------
   The keys are the trace's own channel names, so a legend row and a log column
   cannot mean different things: the same one-seam law as the sampler itself.
   `get` is a pointer to the member instead of a switch, so a channel is one row
   here (plus its slot in recRow's format string) and the two cannot drift apart
   silently -- test_recorder checks every key below is one of recChannels()'s.

   There is deliberately NO axis assignment here. One graph was the design, and
   these channels sit three decades apart in unit terms -- an ascent touches
   100 km of altitude, 30 kPa of q and 2.5 km/s of v -- so any shared unit axis
   (one, two, or three of them) draws two of those three flat against the floor.
   The window therefore plots each visible channel against its OWN min..max: the
   shape and its timing read true, the value does not, and recovering the value
   is #990. Which axis a channel belongs to stops being a fact anywhere.

   ONE string per channel, used verbatim as the toggle label, the legend row and
   the plot item's label, and it must stay static text. ImPlot keys a plot item
   by hashing the whole label (implot_items.cpp:410 RegisterOrGetItem ->
   ImPlotItemGroup::GetItemID), and "##" only truncates what gets DISPLAYED
   (imgui.cpp:3880 FindRenderedTextEnd) -- it is not an id override. The first
   version of this window put live numbers in the label ("q Pa 20..2.3e3"), which
   registered a fresh item every frame: each one was just_created, so it took the
   next colormap colour and the graph changed hue mid-flight, while the plot's
   item pool and legend buffer grew by an entry per channel per frame for as long
   as the window stayed open. Units go with the numbers: on a normalised axis
   "m/s" names a scale the axis does not carry, and it is what stopped the 13
   toggles fitting in two rows. A key must never contain '#', since both imgui and
   implot cut display text at "##" -- test_recorder pins that.

   Curated, not all 40 columns: this is what an ascent or a re-entry is read
   from. A channel that is in the trace and not here is still in the file. */
struct RecChan {
    const char *key;                 // the trace channel name; the toggle, the legend row, and the plot item's label -- the same static string three times over
    double RecSample::*get;          // the reading itself
    bool dflt;                       // visible when the window first opens
    /* The unit, for the readout ONLY (#990). Never with the key: the toggles and
       the legend row stay the bare key, because on a normalised axis a unit names
       a scale the axis does not carry, and it is what stopped the 13 labels
       fitting two rows. The readout is different -- there the number is absolute,
       so a bare "45210" asks which of metres, seconds or pascals it is.
       "" for a dimensionless ratio (throttle). */
    const char *unit;
};

static const RecChan kRecChans[] = {
    {"alt_asl", &RecSample::alt_asl, true,  "m"},
    {"v",       &RecSample::v,       true,  "m/s"},
    {"q",       &RecSample::q,       true,  "Pa"},
    {"aoa",     &RecSample::aoa,     true,  "deg"},
    {"vs",      &RecSample::vs,      false, "m/s"},
    {"gamma",   &RecSample::gamma,   false, "deg"},
    {"acc",     &RecSample::acc,     false, "m/s2"},
    {"dv",      &RecSample::dv,      false, "m/s"},
    {"thr",     &RecSample::thr,     false, ""},
    {"dv_exp",  &RecSample::dv_exp,  false, "m/s"},
    {"grav",    &RecSample::grav,    false, "m/s"},
    {"drag",    &RecSample::drag,    false, "m/s"},
    {"steer",   &RecSample::steer,   false, "m/s"},
};
static const int kRecNumCh = (int)(sizeof(kRecChans) / sizeof(kRecChans[0]));

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

    /* The samples as the vector actually holds them: ONE run of slots,
       oldest-first, while the ring has not wrapped -- and TWO runs once it has,
       because then the oldest sits at `head`, where the next write lands. A plot
       that walked at() across the wrap would join the newest sample to the oldest
       and streak one long line straight across the flight, so this hands back the
       runs to draw separately. Fills idx[]/len[] (at most two) and returns how
       many: 0 empty, 1 unwrapped, 2 wrapped. Drawing them separately has a cost
       worth naming: the one segment between the last sample of run 0 and the
       first of run 1 goes undrawn, so a wrapped ring has a gap exactly one sample
       wide -- 0.25 s of mission time, which at a 560 px plot and a 17 min ring is
       well under a pixel. Worth knowing before someone "fixes" it by joining them. */
    int spans(int *idx, int *len) const {
        const int o = count ? (head - count + Cap) % Cap : 0;
        if(o + count <= Cap) {
            idx[0] = o;
            len[0] = count;
            return count ? 1 : 0;
        }
        idx[0] = o;   len[0] = Cap - o;
        idx[1] = 0;   len[1] = count - (Cap - o);
        return 2;
    }

    // Keeps the allocation; count is the only reader, as in TimeSeries.
    void clear() { head = 0; count = 0; }
};

/* The extent of one channel across the ring: min/max over the readings that are
   numbers, with *nread counting them so a caller can tell "flat at 0" from
   "nothing was ever readable here" (a dash is not a zero, which is the same
   distinction the row grammar spends its dash rules on).
   Whole ring rather than per span, because the window scales both runs of a
   wrapped ring by the same range -- scale each run by its own and the line
   breaks at the wrap. */
inline void recExtent(const RecRing &ring, double RecSample::*get,
                      double *mn, double *mx, int *nread) {
    int idx[2], len[2];
    const int nspans = ring.spans(idx, len);
    double lo = 0.0, hi = 0.0;
    int n = 0;
    for(int s = 0; s < nspans; s++) {
        const RecPoint *b = ring.p.data() + idx[s];
        for(int i = 0; i < len[s]; i++) {
            const double v = b[i].s.*(get);
            /* An Inf is not a number either: one in the ring makes the range itself
               NaN, and a NaN range normalises the whole line away while nread still
               reports readings. */
            if(std::isnan(v) || std::isinf(v)) { continue; }
            if(n == 0) { lo = hi = v; }
            else {
                if(v < lo) { lo = v; }
                if(v > hi) { hi = v; }
            }
            n++;
        }
    }
    *mn = lo;
    *mx = hi;
    *nread = n;
}

/* The mission-time extent of what the ring holds. Spelled out rather than left to
   implot's fitter because implot fits an axis ONCE, on the frame the plot first
   appears (implot.cpp:2584, CanInitFit): a live recorder would otherwise stay
   pinned to however much flight the ring held when the window opened. min/max
   rather than first/last: the ring keeps WRITE order, and a load can move the
   mission clock underneath it (see RecRing::push), which read as ends would pass
   while clipping half the flight. */
inline void recTimeExtent(const RecRing &ring, double *t0, double *t1) {
    int idx[2], len[2];
    const int nspans = ring.spans(idx, len);
    double lo = 0.0, hi = 0.0;
    int n = 0;
    for(int s = 0; s < nspans; s++) {
        const RecPoint *b = ring.p.data() + idx[s];
        for(int i = 0; i < len[s]; i++) {
            const double t = b[i].sim_t;
            if(n == 0) { lo = hi = t; }
            else {
                if(t < lo) { lo = t; }
                if(t > hi) { hi = t; }
            }
            n++;
        }
    }
    *t0 = lo;
    *t1 = hi;
}

/* The sample nearest a mission time (#990: the readout has to answer "what was
   q WHEN", and the graph only carries x). at()-order is chronological, so a walk
   finds the nearest; it runs once per frame, not once per point, so the 4096
   sample worst case is noise beside the ~50k vertices the plot submits.
   Clamped to the nearest END rather than returning -1 when the time is outside
   what the ring holds (the pilot panned past it, or the ring rolled): the readout
   then says what the closest reading IS, and the marker sits on a real sample
   instead of floating in a gap. Only an empty ring gives -1. */
inline int recNearSample(const RecRing &ring, double t) {
    if(ring.count == 0) { return -1; }
    int best = 0;
    double bestD = -1.0;
    for(int i = 0; i < ring.count; i++) {
        const double d = std::fabs(ring.at(i).sim_t - t);
        if(bestD < 0.0 || d < bestD) { bestD = d; best = i; }
    }
    return best;
}

/* One reading in the readout's format. %.6g because a pilot comparing two samples
   needs to tell 45210 from 45211, and a fixed %.1f would print 123456.7 next to
   0.0 for two channels decades apart. A non-finite reading gets the trace's dash:
   recExtent() already refuses to build a range out of one, and a window showing
   "inf" where the file prints "-" is the disagreement this file's dash rules exist
   to prevent. Both the drawn rows and the [recwin] pin line go through here, so
   the picture and the log cannot report different numbers for one sample. */
inline void recFmtReading(const RecChan &c, const RecSample &s, char *buf, size_t n) {
    assert(buf != nullptr && n > 0);
    const double v = s.*(c.get);
    if(!std::isfinite(v)) { snprintf(buf, n, "-"); return; }
    snprintf(buf, n, "%.6g", v);
}

/* Chapter marks: the times at which the flight's own story turned a page.
   seams = true asks where the dock count changed (v4's seams=), false where the
   stage about to fire changed (stg=). "Differs from the sample before it" is the
   whole test, because both counters go DOWN as well as up: a fresh hull restarts
   the stage index and an undock removes a seam, so a set of the values seen would
   mark one chapter twice and miss the transitions that matter.
   Bounded at max: a hull that mates and separates two hundred times would
   otherwise draw a black wall instead of a graph. Returns how many it stored, so a
   caller that means to admit the truncation has something to compare against.
   Walks the same runs spans() hands back rather than at(), because the caller is
   about to plot those runs anyway. */
static const int kRecMarkMax = 64;

inline int recMarks(const RecRing &ring, bool seams, double *xs, int max) {
    int idx[2], len[2];
    const int nspans = ring.spans(idx, len);
    if(nspans == 0) { return 0; }
    const RecPoint *first = ring.p.data() + idx[0];
    int prev = seams ? first[0].s.seams : first[0].s.stg;
    int n = 0;
    for(int s = 0; s < nspans; s++) {
        const RecPoint *b = ring.p.data() + idx[s];
        // The oldest sample is the first one's neighbour, not a transition.
        for(int i = (s == 0 ? 1 : 0); i < len[s]; i++) {
            const int now = seams ? b[i].s.seams : b[i].s.stg;
            if(now != prev) {
                if(n < max) { xs[n++] = b[i].sim_t; }
                prev = now;
            }
        }
    }
    return n;
}
