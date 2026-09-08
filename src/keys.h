// keys.h -- the rebindable key map.
//
// A control is a named Slot (PitchUp, Thrust, WarpUp, ...) -- a logical
// action, not a physical key. Each Slot backs a list of KeyBindings, and a
// KeyBinding is a physical scancode plus the modifier bits that must be
// EXACTLY present (Shift/Ctrl/Alt). A plain binding (mods == 0) fires only
// when no modifier is held. The same physical key may back several slots
// (W = PitchUp and CamForward and EvaForward); the call sites only ever
// read the slot their mode uses, so the shared defaults never collide.
//
// This is the single source of truth for "which key does what". It replaces
// the two hardcoded key models that used to live in the input paths:
//   - the one-shot (edge) actions in events.cpp, matched by SDL_Keycode;
//   - the held commands in tick.cpp / eva.cpp, matched by SDL_Scancode.
// Both now go through this table, keyed by scancode (layout-independent:
// it is the physical key, so a non-US layout just rebinds).
//
// Pure logic: SDL headers only (no SDL calls), so the lookup/isDown core is
// headless-testable without a video context.
#pragma once

#include <SDL2/SDL_scancode.h>   // SDL_Scancode
#include <SDL2/SDL_keycode.h>   // KMOD_SHIFT / KMOD_CTRL / KMOD_ALT
#include <SDL2/SDL_stdinc.h>    // Uint8, Uint16

#include <array>
#include <string>
#include <vector>

// The modifier bits a binding may require. Anything else (NumLock, CapsLock,
// GUI/Cmd, ...) is ignored -- a binding is Shift/Ctrl/Alt or none.
//
// LShift and RShift (and the L/R Ctrl, Alt pairs) are DISTINCT modifier keys:
// a binding names the exact side, and a press matches only the side it used.
// KMOD_SHIFT is LShift|RShift -- a value no single press produces -- so a
// binding always stores one concrete side (KMOD_LSHIFT or KMOD_RSHIFT).
static const Uint16 KMOD_RELEVANT = KMOD_SHIFT | KMOD_CTRL | KMOD_ALT;

// One rebindable control. The enum value is its index into KeyBindings.
// Grouped for the UI: Game (one-shot), Flight (orbit mode), Camera (free
// mode), Eva (the kerbal). SLOT_COUNT is the sentinel / array bound.
enum class Slot {
    // --- Game: one-shot actions (events.cpp, SDL_KEYDOWN edges) ----------
    WarpUp,        // '.'  warp one step up (10x)
    WarpDown,      // ','  warp one step down
    CamSpeedUp,    // ']'  camera speed x4
    CamSpeedDown,  // '['  camera speed /4
    ToggleCamMode, // 'c'  orbit (flying) <-> free (exploring)
    CycleTarget,   // 'g'  orbit mode: cycle the target body/ship
    ToggleWindows, // TAB  toggle the info windows
    NextShip,      // F6   advance to the next selectable ship
    ToggleEva,     // 'v'  EVA out of the ship / back in
    Space,         // SPACE stage (ship) / jump (EVA)
    Undock,        // 'u'  split the most recent docked seam (active ship)
    Screenshot,    // F12
    Porkchop,      // 'p'  compute the porkchop plot
    SurfaceMap,    // 'm'  compute the surface map
    Wireframe,     // F11  toggle wireframe
    ResetWindows,  // F10  reset the window layout
    Menu,          // ESC  toggle the main menu
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
    // RCS translation (camera-relative, KSP-style; the ship's analogue of
    // the EVA suit's WASD/R/F -- the kerbal keeps its own keys):
    RcsForward,    // 'n'  along the view direction (into the screen)
    RcsBack,       // 'h'  out of the screen
    RcsUp,         // 'i'  screen up
    RcsDown,       // 'k'  screen down
    RcsLeft,       // 'j'  screen left
    RcsRight,      // 'l'  screen right
    // --- Camera: held commands in free mode (tick.cpp) -------------------
    CamForward,    // 'w'
    CamBack,       // 's'
    CamStrafeLeft, // 'a'
    CamStrafeRight,// 'd'
    CamRollLeft,   // 'q'
    CamRollRight,  // 'e'
    CamUp,         // 'r'  (was LShift/RShift)
    CamDown,       // 'f'  (was LCtrl/RCtrl)
    // --- Eva: held commands on the kerbal (eva.cpp) ----------------------
    EvaForward,    // 'w'
    EvaBack,       // 's'
    EvaLeft,       // 'a'
    EvaRight,      // 'd'
    EvaUp,         // 'r'  (was LShift)
    EvaDown,       // 'f'  (was LCtrl)
    EvaYawLeft,    // 'q'
    EvaYawRight,   // 'e'
    SLOT_COUNT
};

// One key binding: a physical key (scancode) + the modifiers that must be
// exactly present. mods is always stored masked to KMOD_RELEVANT; 0 = plain.
struct KeyBind {
    SDL_Scancode sc;
    Uint16 mods;
};

// The binding table: slot -> the keys that back it. Default-constructed to
// the game's default key map (see defaultBindings in keys.cpp).
struct KeyBindings {
    std::array<std::vector<KeyBind>, (size_t)Slot::SLOT_COUNT> perSlot;
    KeyBindings();                      // = defaultBindings()
    void resetDefaults();               // back to the game's default map
};

// --- lookup ----------------------------------------------------------------
// Exact-modifier match: the press's relevant modifier bits equal the
// binding's (a plain binding needs no modifier held; a combo needs exactly
// its modifiers, no more, no less).
bool bindingMatches(const KeyBind &b, SDL_Scancode sc, Uint16 mods);

// Edge path (events.cpp): did THIS press (sc, mods) fire slot s? True if any
// of s's bindings matches it. At most one Game slot fires per press in the
// default map (the one-shot keys are distinct).
bool slotFired(Slot s, SDL_Scancode sc, Uint16 mods, const KeyBindings &kb);

// Held path (tick.cpp / eva.cpp): is slot s currently armed? True if any of
// s's keys is down (keyState = the SDL_GetKeyboardState array) with exactly
// its modifiers (mods = the current SDL_GetModState).
bool slotHeld(Slot s, const Uint8 *keyState, Uint16 mods, const KeyBindings &kb);

// --sim-press compatibility: a synthetic key carries a scancode but no
// modifier state, so it can only back a PLAIN binding. True if slot s has a
// mods==0 binding on scancode sc (tick.cpp ORs this into its held check).
bool slotSimKey(Slot s, SDL_Scancode sc, const KeyBindings &kb);

// --- naming (stable identifiers for the UI + settings.json) ---------------
// slotName is the persistent identifier (snake_case); slotLabel is the
// human display string; slotGroup is the UI category.
const char *slotName(Slot s);
Slot        slotFromName(const char *name);   // SLOT_COUNT if unknown
const char *slotLabel(Slot s);
enum class SlotGroup { Game, Flight, Camera, Eva, GROUP_COUNT };
SlotGroup   slotGroup(Slot s);

// A binding as a label: "W", "Shift+W", "Ctrl+Shift+W". (For the UI.)
std::string bindLabel(const KeyBind &b);
