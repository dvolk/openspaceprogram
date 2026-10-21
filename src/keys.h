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

// One rebindable control. SLOT_COUNT is the array bound.
enum class Slot {
    // --- Game: one-shot actions (events.cpp, SDL_KEYDOWN edges) ----------
    WarpUp,        // '.'  warp one step up (10x)
    WarpDown,      // ','  warp one step down
    CamSpeedUp,    // ']'  camera speed x4
    CamSpeedDown,  // '['  camera speed /4
    ToggleCamMode, // 'c'  ship: orbit <-> first-person; kerbal: orbit <-> free
    ToggleFreeCam, // 'LShift+c' free-cam on/off
    CycleTarget,   // 'g'  orbit mode: cycle the target body/ship
    ToggleWindows, // TAB  the info windows
    DebugInfo,     // F1   the GAME DEBUG window (readouts + dev options)
    Telemetry,     // F2   the TELEMETRY plots
    NextShip,      // F6   next selectable ship
    ToggleEva,     // 'v'  EVA out of the ship / back in
    Space,         // SPACE stage (ship) / jump (EVA)
    Undock,        // 'u'  split the most recent docked seam (active ship)
    Screenshot,    // F12
    Quicksave,     // F5   game -> next quicksave-NN pool slot
    Quickload,     // F9   newest quicksave of the running game
    Porkchop,      // 'p'  the porkchop plot
    SurfaceMap,    // 'm'  the surface map
    Wireframe,     // F11  wireframe
    ToggleOrbitPlane, // F3  the orbital plane overlay (Settings checkbox mirror)
    ResetWindows,  // F10  reset the window layout
    Menu,          // ESC  the main menu
    GoSpaceCenter, // '1'  Space Center hub
    GoFlight,      // '2'  flight (the cockpit)
    GoTracking,    // '3'  Tracking Station
    GoVab,         // '4'  VAB (the editor)
    // --- Flight: held commands while piloting (tick.cpp) -------------------
    PitchUp,
    PitchDown,
    YawLeft,
    YawRight,
    RollLeft,
    RollRight,
    Thrust,
    ThrustLatch,   // latch thrust to fire (toggle; a plain thrust release)
    KillRot,
    ThrottleUp,
    ThrottleDown,
    // One-key attitude-to-velocity (the Autopilot window's toggles, bound).
    SlewPrograde,  // hold the nose along the ORBIT's velocity vector
    SlewRetrograde,// hold the nose against the ORBIT's velocity vector
    // RCS translation (ship-relative):
    RcsForward,    // along the ship's nose
    RcsBack,       // astern
    RcsUp,         // ship up
    RcsDown,       // ship down
    RcsLeft,       // ship left
    RcsRight,      // ship right
    // --- Camera: held commands in free mode (tick.cpp) -------------------
    CamForward,
    CamBack,
    CamStrafeLeft,
    CamStrafeRight,
    CamRollLeft,
    CamRollRight,
    CamUp,
    CamDown,
    // --- Eva: held commands on the kerbal (eva.cpp) ----------------------
    EvaForward,
    EvaBack,
    EvaLeft,
    EvaRight,
    EvaUp,
    EvaDown,
    EvaYawLeft,
    EvaYawRight,
    SLOT_COUNT
};

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

bool bindingMatches(const KeyBind &b, SDL_Scancode sc, Uint16 mods);

// Edge path (events.cpp): did THIS press fire slot s?
bool slotFired(Slot s, SDL_Scancode sc, Uint16 mods, const KeyBindings &kb);

// Held path (tick.cpp / eva.cpp): is slot s currently armed?
bool slotHeld(Slot s, const bool *keyState, Uint16 mods, const KeyBindings &kb);

// A synthetic key (--sim-press) has no modifier state: only plain bindings match.
bool slotSimKey(Slot s, SDL_Scancode sc, const KeyBindings &kb);

// Stable identifiers for the UI + settings.json.
const char *slotName(Slot s);
Slot        slotFromName(const char *name);   // SLOT_COUNT if unknown
const char *slotLabel(Slot s);
enum class SlotGroup { Game, Flight, Camera, Eva, GROUP_COUNT };
SlotGroup   slotGroup(Slot s);

// A binding as a label: "W", "Shift+W", "Ctrl+Shift+W".
std::string bindLabel(const KeyBind &b);
