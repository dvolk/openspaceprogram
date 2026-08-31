// siminput.h -- synthetic input + telemetry types for the e2e tests.
//
//   TimeSeries         fixed-size (sim time, value) ring for the plots.
//   SimKeyPress        one parsed --sim-press entry.
//   SimMouseAction     one parsed --sim-mouse entry.
//   sim_parse_key      SDL key name (SPACE, A, F11, ...) or decimal keycode.
//   sim_parse_button   SDL button name (L/R/M...) or decimal code.
//   fmt_time           "1d 04:03:02" / "04:03:02" ToF / period readouts.

#pragma once

#include "SDL2/SDL.h"
#include "SDL_keycode.h"

#include <string>

#include "display.h"   // WindowMode (the --sim-mode entry)

// Circular buffer of (sim time, value) samples for the telemetry plots.
// Fixed size and preallocated, so sampling in the render loop never
// allocates; once full, push() overwrites the oldest sample.
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

/* --sim-press: synthetic key input for e2e testing.
   One entry = one key press: down `down_ms` after the main loop starts,
   up `up_ms` after. It feeds two input channels:
   - one-shot actions (SPACE, TAB, G, C, ...) fire from the synthetic
     SDL_KEYDOWN event SDL_PushEvent()ed into the queue by the loop;
   - held commands (WASDQE, I, X, B, N, R, F, ESC) read
     SDL_GetKeyboardState(), which SDL_PushEvent does NOT update (verified
     on this system's SDL 2.32), so the loop additionally ORs in each
     entry's down_sent..up_sent window (see isDown below).
   Keys are given by SDL key name (SPACE, A, F11, ...) or decimal SDL
   keycode (32, 105, 1073741911). */
struct SimKeyPress {
    Uint32 down_ms;
    Uint32 up_ms;
    SDL_Keycode key;
    SDL_Scancode sc;
    bool down_sent;
    bool up_sent;
};

/* --sim-mouse: synthetic mouse input for e2e testing.
   One entry = one mouse action: it starts `time_ms` after the main loop
   starts. Semantics by (button, duration):
   - button != 0 && duration > 0  -> a DRAG: press the button, then move
     the cursor to (x,y); release at time_ms + duration. Used to orbit the
     camera (RMB) -- the look code reads the motion delta, and the button
     must be down before the motion event.
   - button != 0 && duration == 0 -> a CLICK: move the cursor to (x,y),
     then press + release in place (same frame). Used to click UI buttons.
   - button == 0                  -> a MOVE: just reposition the cursor.
   (x,y) are absolute window pixels; the emitted MOUSEMOTION carries the
   delta from the previous simulated position, which is what the camera
   consumes (yaw = -dx/200 rad, pitch = +dy/200 rad; 200px ~= 1 rad). */
struct SimMouseAction {
    Uint32 time_ms;   // when the action starts (after the loop starts)
    Uint32 up_ms;     // time_ms + duration; the button release time
    int x;            // target position, window pixels
    int y;
    Uint8 button;     // SDL button code (1=LEFT,2=MIDDLE,3=RIGHT); 0 = move only
    bool started;     // start events (button-down + motion) already emitted
    bool released;    // button-up already emitted
};

/* --sim-mode: a scripted runtime display-mode change for e2e testing
   (the same Renderer::setWindowMode path the Settings dropdowns use).
   One entry = one change at `at_ms` after the main loop starts, to
   `mode` at `width` x `height` (native "fullscreen" ignores the size). */
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
std::string fmt_time(double s);
