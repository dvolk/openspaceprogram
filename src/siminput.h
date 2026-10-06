// siminput.h -- synthetic input + telemetry types for the e2e tests.

#pragma once

#include <SDL3/SDL.h>
#include <SDL3/SDL_keycode.h>

#include <string>

#include "display.h"   // WindowMode (the --sim-mode entry)

// Circular buffer of (sim time, value) samples. Fixed size, preallocated.
struct TimeSeries {
    enum { N = 8192 };
    double t[N];
    double v[N];
    double st_t[N];  // staging copies, filled oldest-first by stage()
    double st_v[N];
    int head = 0;    // next write slot
    int count = 0;   // samples stored (capped at N)
    void push(double tt, double vv) {
        const int last = (head - 1 + N) % N;
        if(count > 0 && t[last] == tt) {
            // sim time didn't advance (paused): refresh the last sample
            v[last] = vv;
            return;
        }
        t[head] = tt;
        v[head] = vv;
        head = (head + 1) % N;
        if(count < N) { count++; }
    }
    // Copies the samples oldest-first into the staging buffers (a wrapped
    // ring is not contiguous) and returns their count.
    int stage() {
        const int start = (head - count + N) % N;
        for(int i = 0; i < count; i++) {
            const int idx = (start + i) % N;
            st_t[i] = t[idx];
            st_v[i] = v[idx];
        }
        return count;
    }
    const double *t_arr() const { return st_t; }
    const double *v_arr() const { return st_v; }
};

/* --sim-press: synthetic key input for e2e testing. One entry = one key
   press. Held commands read SDL_GetKeyboardState (which SDL_PushEvent does
   NOT update), so the loop ORs in each entry's down..up window. */
struct SimKeyPress {
    Uint32 down_ms;
    Uint32 up_ms;
    SDL_Keycode key;
    SDL_Scancode sc;
    bool down_sent;
    bool up_sent;
};

/* --sim-mouse: synthetic mouse input for e2e testing.
   Semantics by (button, duration): drag (button!=0, dur>0), click
   (button!=0, dur==0), move (button==0), wheel notch (button 4/5).
   (x,y) are absolute window pixels; MOUSEMOTION carries the delta. */
struct SimMouseAction {
    Uint32 time_ms;   // when the action starts (after the loop starts)
    Uint32 up_ms;     // time_ms + duration; the button release time
    int x;            // target position, window pixels
    int y;
    Uint8 button;     // SDL button code (1=LEFT,2=MIDDLE,3=RIGHT); 0 = move only
    bool started;     // start events (button-down + motion) already emitted
    bool released;    // button-up already emitted
};

/* --ui-click: a synthetic imgui click for e2e testing, addressed by the
   widget's "Window/Label" path rather than by window pixels (uiinput.h).
   The click lands on the first frame, from at_ms on, whose PRECEDING imgui
   pass drew a matching on-screen item. */
struct UiClick {
    Uint32 at_ms;      // when to start looking (after the loop starts)
    std::string path;  // "Window/Label"; with no '/', the label in any window
    bool down_sent;    // the press was queued (the item was found)
    bool up_sent;      // the release was queued
    bool done;         // clicked, or gave up (a diagnostic was printed)
};

/* --sim-mode: a scripted runtime display-mode change for e2e testing. */
struct SimModeChange {
    Uint32 at_ms;     // when the change applies (after the loop starts)
    WindowMode mode;
    int width;
    int height;
    bool done;        // already applied
};

// Unknown key: returns 0.
SDL_Keycode sim_parse_key(const std::string &s);
// Unknown button: returns -1.
int sim_parse_button(const std::string &s);
