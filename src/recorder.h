// recorder.h -- the flight recorder's channel table and row grammar (#662).
//
// A recorder row is one line in the flight logs' grammar ("t=... key=value ..."),
// so the harness that already parses [drag]/[orbit]/[ascent] rows parses a trace
// too, and the version plus the channel names ride in the file's header line --
// the file stays readable after the code that wrote it is gone.
//
// Every channel is a number the physics ALREADY publishes for the tick it is
// sampled in. The recorder computes nothing, which is the point: a recorded value
// and a logged value cannot disagree -- the same one-seam law #665 put on the
// instrument parts.

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdio>

static const int kRecVersion = 1;

// The channels, in row order: the header line's contract with a reader.
inline const char *recChannels() {
    return "t alt_asl v rho F L Cd A aoa beta cpl ind chute T p a mach mass thr stg";
}

// The dash rules are --drag-log's, unchanged: aoa/beta/cpl/ind wait for a flow
// worth reading (the same test that sets lastDragAlphaValid, #560) and T/p/a/mach
// wait for air, because drag.h answers 0.0 wherever there is no air and that is
// not a temperature to record (00db083). A NaN in a RecSample field means "dash
// this column"; only those eight can be NaN on purpose, so a plain channel keeps
// printing whatever the physics published -- if that is a NaN, the trace should
// show it rather than hide it.
struct RecSample {
    double t = 0.0;        // run seconds: the log clock since THIS trace began
    double alt_asl = 0.0;  // metres above the datum (lastDragAlt)
    double v = 0.0;        // hull speed, m/s
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
        snprintf(beta_txt, sizeof beta_txt, "%.1f", s.beta);
        snprintf(cpl_txt, sizeof cpl_txt, "%.2f", s.cpl);
        snprintf(ind_txt, sizeof ind_txt, "%.3f", s.ind);
    } else {
        snprintf(aoa_txt, sizeof aoa_txt, "-");
        snprintf(beta_txt, sizeof beta_txt, "-");
        snprintf(cpl_txt, sizeof cpl_txt, "-");
        snprintf(ind_txt, sizeof ind_txt, "-");
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
    const int wrote = snprintf(buf, n,
                               "t=%.2f alt_asl=%.1f v=%.2f rho=%.5g F=%.2f L=%.2f"
                               " Cd=%.3f A=%.2f aoa=%s beta=%s cpl=%s ind=%s chute=%.2f"
                               " T=%s p=%s a=%s mach=%s mass=%.1f thr=%.2f stg=%d",
                               s.t, s.alt_asl, s.v, s.rho, s.F, s.L, s.Cd, s.A,
                               aoa_txt, beta_txt, cpl_txt, ind_txt, s.chute,
                               t_txt, p_txt, a_txt, mach_txt, s.mass, s.thr, s.stg);
    if(wrote < 0 || (size_t)wrote >= n) { return -1; }
    return wrote;
}
