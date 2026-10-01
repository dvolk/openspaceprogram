// keys.h -- the rebindable key map. A Slot is a logical action; a KeyBind is
// a scancode + modifiers that must be EXACTLY present (plain binding = no
// modifiers held). The same physical key may back several slots (one per mode).
// Pure logic (SDL headers only), headless-testable.
#pragma once

#include <SDL3/SDL_scancode.h>  // SDL_Scancode
#include <SDL3/SDL_keycode.h>   // SDL_KMOD_SHIFT / SDL_KMOD_CTRL / SDL_KMOD_ALT
#include <SDL3/SDL_stdinc.h>    // Uint8, Uint16

#include <array>
#include <string>
#include <vector>

// LShift/RShift (and L/R Ctrl, Alt) are DISTINCT: a binding names the exact
// side. SDL_KMOD_SHIFT is L|R -- a value no single press produces.
static const Uint16 KMOD_RELEVANT = SDL_KMOD_SHIFT | SDL_KMOD_CTRL | SDL_KMOD_ALT;

// One rebindable control. Grouped for the UI. SLOT_COUNT is the array bound.
enum class Slot {
    // --- Game: one-shot actions (events.cpp, SDL_KEYDOWN edges) ----------
    WarpUp,        // '.'  warp one step up (10x)
    WarpDown,      // ','  warp one step down
    CamSpeedUp,    // ']'  camera speed x4
    CamSpeedDown,  // '['  camera speed /4
    ToggleCamMode, // 'c'  orbit (flying) <-> free (exploring)
    CycleTarget,   // 'g'  orbit mode: cycle the target body/ship
    ToggleWindows, // TAB  toggle the info windows
    DebugInfo,     // F1   toggle the Game Debug Info window
    Telemetry,     // F2   toggle the TELEMETRY plots
    NextShip,      // F6   advance to the next selectable ship
    ToggleEva,     // 'v'  EVA out of the ship / back in
    Space,         // SPACE stage (ship) / jump (EVA)
    Undock,        // 'u'  split the most recent docked seam (active ship)
    Screenshot,    // F12
    Quicksave,     // F5   save the fleet to the next quicksave-NN pool slot
    Quickload,     // F9   load the newest quicksave of the running game
    Porkchop,      // 'p'  compute the porkchop plot
    SurfaceMap,    // 'm'  compute the surface map
    Wireframe,     // F11  toggle wireframe
    ResetWindows,  // F10  reset the window layout
    Menu,          // ESC  toggle the main menu
    GoSpaceCenter, // '1'  jump to the Space Center hub (the in-game menu)
    GoFlight,      // '2'  jump to the flight (the cockpit)
    GoTracking,    // '3'  jump to the Tracking Station
    GoVab,         // '4'  jump to the VAB (the editor)
    // --- Flight: held commands in orbit mode (tick.cpp) ------------------
    PitchUp,       // 'w'
    PitchDown,     // 's'
    YawLeft,       // 'a'
    YawRight,      // 'd'
    RollLeft,      // 'q'
    RollRight,     // 'e'
    Thrust,        // 't'
    ThrustLatch,   // 'LShift+t' latch thrust to fire (toggle; a plain 't' release)
    KillRot,       // 'x'
    ThrottleUp,    // 'r'
    ThrottleDown,  // 'f'
    // RCS translation (ship-relative):
    RcsForward,    // 'n'  along the ship's nose
    RcsBack,       // 'h'  astern
    RcsUp,         // 'i'  ship up
    RcsDown,       // 'k'  ship down
    RcsLeft,       // 'j'  ship left
    RcsRight,      // 'l'  ship right
    // --- Camera: held commands in free mode (tick.cpp) -------------------
    CamForward,    // 'w'
    CamBack,       // 's'
    CamStrafeLeft, // 'a'
    CamStrafeRight,// 'd'
    CamRollLeft,   // 'q'
    CamRollRight,  // 'e'
    CamUp,         // 'r'
    CamDown,       // 'f'
    // --- Eva: held commands on the kerbal (eva.cpp) ----------------------
    EvaForward,    // 'w'
    EvaBack,       // 's'
    EvaLeft,       // 'a'
    EvaRight,      // 'd'
    EvaUp,         // 'r'
    EvaDown,       // 'f'
    EvaYawLeft,    // 'q'
    EvaYawRight,   // 'e'
    SLOT_COUNT
};

// One key binding: scancode + the modifiers that must be exactly present.
struct KeyBind {
    SDL_Scancode sc;
    Uint16 mods;   // masked to KMOD_RELEVANT; 0 = plain
};

// slot -> the keys that back it.
struct KeyBindings {
    std::array<std::vector<KeyBind>, (size_t)Slot::SLOT_COUNT> perSlot;
    KeyBindings();
    void resetDefaults();
};

// --- lookup: exact-modifier match ------------------------------------------
bool bindingMatches(const KeyBind &b, SDL_Scancode sc, Uint16 mods);

// Edge path (events.cpp): did THIS press fire slot s?
bool slotFired(Slot s, SDL_Scancode sc, Uint16 mods, const KeyBindings &kb);

// Held path (tick.cpp / eva.cpp): is slot s currently armed?
bool slotHeld(Slot s, const bool *keyState, Uint16 mods, const KeyBindings &kb);

// --sim-press: a synthetic key has no modifier state, so only plain bindings match.
bool slotSimKey(Slot s, SDL_Scancode sc, const KeyBindings &kb);

// --- naming (stable identifiers for the UI + settings.json) ----------------
const char *slotName(Slot s);
Slot        slotFromName(const char *name);   // SLOT_COUNT if unknown
const char *slotLabel(Slot s);
enum class SlotGroup { Game, Flight, Camera, Eva, GROUP_COUNT };
SlotGroup   slotGroup(Slot s);

// A binding as a label: "W", "Shift+W", "Ctrl+Shift+W".
std::string bindLabel(const KeyBind &b);
