// siminput.h -- synthetic input + telemetry types for the e2e tests.

#pragma once

#include <SDL3/SDL.h>
#include <SDL3/SDL_keycode.h>

#include <string>

#include "display.h"   // WindowMode

// Circular buffer of (sim time, value) samples.
struct TimeSeries {
    enum { N = 8192 };
    double t[N];
    double v[N];
    double st_t[N];  // staging copy for stage()
    double st_v[N];
    int head = 0;    // next write slot
    int count = 0;   // samples stored
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
    /* A load moves sim `time` BACKWARDS, so samples kept across a teardown
       leave a non-monotonic x-axis spanning the game that just died (#304).
       The ring needs no zeroing: `count` is the only reader. */
    void clear() { head = 0; count = 0; }
    // Oldest-first copy into the staging buffers (a wrapped ring is not
    // contiguous).
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

/* --sim-press: one synthetic key press. Held commands read
   SDL_GetKeyboardState, which SDL_PushEvent does NOT update, so the loop ORs
   in each entry's down..up window. */
struct SimKeyPress {
    Uint32 down_ms;
    Uint32 up_ms;
    SDL_Keycode key;
    SDL_Scancode sc;
    bool down_sent;
    bool up_sent;
};

/* --sim-mouse: drag (button!=0, dur>0), click (button!=0, dur==0), move
   (button==0), wheel notch (button 4/5). (x,y) are absolute window pixels, or
   the literal "CUR" to use wherever the simulated cursor already stands --
   which is how a case clicks a part it aimed at with --ui-focus instead of
   hardcoding a pixel that only means something in one camera framing. */
static const int kSimCurPos = -0x7fffffff;   // "not a pixel, use the cursor"
struct SimMouseAction {
    Uint32 time_ms;   // action start
    Uint32 up_ms;     // button release time
    int x;            // window pixels; kSimCurPos = "wherever the cursor is"
    int y;
    Uint8 button;     // SDL code (1=LEFT,2=MIDDLE,3=RIGHT); 0 = move only
    bool started;     // button-down + motion emitted
    bool released;    // button-up emitted
};

/* --ui-focus: park the simulated cursor on a named part. The test says which
   PART it means; the game projects it to a pixel with the same math the pick
   uses. It is a teleport rather than a drag, but it lands on the real cursor
   position, so the real raycast pick still runs -- if another part is in
   front, that one gets selected and the case sees it. */
struct UiFocus {
    Uint32 at_ms;      // when to aim
    std::string part;  // build part id
    bool done;         // aimed, or gave up (no such part / not the editor)
};

/* --ui-click: synthetic imgui click addressed by the widget's "Window/Label"
   path rather than by pixels. Lands on the first frame at or after at_ms (see
   uiinput.h). */
struct UiClick {
    Uint32 at_ms;      // start looking
    std::string path;  // "Window/Label"; with no '/', the label in any window
    bool down_sent;    // press queued (item found)
    bool up_sent;      // release queued
    bool done;         // clicked, or gave up
    /* --ui-type: text to enter once the click has focused this widget. Empty
       for a plain --ui-click. It goes through imgui's own input queue (a
       select-all, then the characters), so the widget's own filters, length cap
       and commit path are what run -- only the OS-level key-to-text translation
       is skipped. */
    std::string type_after;
};

/* --sim-mode: scripted runtime display-mode change. */
struct SimModeChange {
    Uint32 at_ms;     // when it applies
    WindowMode mode;
    int width;
    int height;
    bool done;        // already applied
};

// Unknown key: returns 0.
SDL_Keycode sim_parse_key(const std::string &s);
// Unknown button: returns -1.
int sim_parse_button(const std::string &s);
