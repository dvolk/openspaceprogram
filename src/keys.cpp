// keys.cpp -- the rebindable key map (declared in keys.h).
//
// Pure logic over SDL's scancode/modifier constants: no SDL calls, so the
// lookup / isDown core runs headless (no video context). See keys.h for the
// Slot / KeyBind / KeyBindings model and the exact-modifier matching rule.
#include "keys.h"

#include <cstring>   // strcmp (slotFromName)

// ---------------------------------------------------------------------------
// Default map: the game's current key assignments, as scancodes (the
// physical keys -- on a US layout these are exactly the keys the input
// paths used to hardcode, and --sim-press resolves the same names to the
// same scancodes, so the e2e suite is unaffected). Cam/Eva up-down moved
// off LShift/LCtrl to R/F so that Shift/Ctrl are free to be pure modifiers.
// ---------------------------------------------------------------------------
void KeyBindings::resetDefaults() {
    for (auto &v : perSlot) { v.clear(); }
    auto add = [&](Slot s, SDL_Scancode sc) {
        perSlot[(size_t)s].push_back(KeyBind{sc, 0});
    };

    // Game (one-shot, events.cpp)
    add(Slot::WarpUp,        SDL_SCANCODE_PERIOD);
    add(Slot::WarpDown,      SDL_SCANCODE_COMMA);
    add(Slot::CamSpeedUp,    SDL_SCANCODE_L);
    add(Slot::CamSpeedDown,  SDL_SCANCODE_K);
    add(Slot::ToggleCamMode, SDL_SCANCODE_C);
    add(Slot::CycleTarget,   SDL_SCANCODE_G);
    add(Slot::ToggleWindows, SDL_SCANCODE_TAB);
    add(Slot::NextShip,      SDL_SCANCODE_F6);
    add(Slot::ToggleEva,     SDL_SCANCODE_V);
    add(Slot::Space,         SDL_SCANCODE_SPACE);
    add(Slot::Screenshot,    SDL_SCANCODE_F12);
    add(Slot::Porkchop,      SDL_SCANCODE_P);
    add(Slot::SurfaceMap,    SDL_SCANCODE_M);
    add(Slot::Wireframe,     SDL_SCANCODE_F11);
    add(Slot::ResetWindows,  SDL_SCANCODE_F10);
    add(Slot::Menu,          SDL_SCANCODE_ESCAPE);

    // Flight (orbit mode, tick.cpp)
    add(Slot::PitchUp,       SDL_SCANCODE_W);
    add(Slot::PitchDown,     SDL_SCANCODE_S);
    add(Slot::YawLeft,       SDL_SCANCODE_A);
    add(Slot::YawRight,      SDL_SCANCODE_D);
    add(Slot::RollLeft,      SDL_SCANCODE_Q);
    add(Slot::RollRight,     SDL_SCANCODE_E);
    add(Slot::Thrust,        SDL_SCANCODE_I);
    // Latch is a modifier combo (toggles; a plain 'i' press -- the Thrust
    // slot -- releases it). LShift and RShift are distinct modifier keys, so
    // the default names one concrete side: LShift+I. (KMOD_SHIFT is
    // LShift|RShift -- a value no single press produces -- and would never
    // match; a rebind to RShift+I stores KMOD_RSHIFT instead.)
    perSlot[(size_t)Slot::ThrustLatch].push_back(KeyBind{SDL_SCANCODE_I, KMOD_LSHIFT});
    add(Slot::KillRot,       SDL_SCANCODE_X);
    add(Slot::ThrottleUp,    SDL_SCANCODE_R);
    add(Slot::ThrottleDown,  SDL_SCANCODE_F);

    // Camera (free mode, tick.cpp)
    add(Slot::CamForward,    SDL_SCANCODE_W);
    add(Slot::CamBack,       SDL_SCANCODE_S);
    add(Slot::CamStrafeLeft, SDL_SCANCODE_A);
    add(Slot::CamStrafeRight,SDL_SCANCODE_D);
    add(Slot::CamRollLeft,   SDL_SCANCODE_Q);
    add(Slot::CamRollRight,  SDL_SCANCODE_E);
    add(Slot::CamUp,         SDL_SCANCODE_R);
    add(Slot::CamDown,       SDL_SCANCODE_F);

    // Eva (the kerbal, eva.cpp)
    add(Slot::EvaForward,    SDL_SCANCODE_W);
    add(Slot::EvaBack,       SDL_SCANCODE_S);
    add(Slot::EvaLeft,       SDL_SCANCODE_A);
    add(Slot::EvaRight,      SDL_SCANCODE_D);
    add(Slot::EvaUp,         SDL_SCANCODE_R);
    add(Slot::EvaDown,       SDL_SCANCODE_F);
    add(Slot::EvaYawLeft,    SDL_SCANCODE_Q);
    add(Slot::EvaYawRight,   SDL_SCANCODE_E);
}

KeyBindings::KeyBindings() { resetDefaults(); }

// ---------------------------------------------------------------------------
// Lookup. Exact-modifier rule: (pressed modifiers & relevant) == binding.
// A plain binding (mods 0) therefore fires only with no Shift/Ctrl/Alt held.
// ---------------------------------------------------------------------------
bool bindingMatches(const KeyBind &b, SDL_Scancode sc, Uint16 mods) {
    return b.sc == sc && (mods & KMOD_RELEVANT) == b.mods;
}

bool slotFired(Slot s, SDL_Scancode sc, Uint16 mods, const KeyBindings &kb) {
    for (const auto &b : kb.perSlot[(size_t)s]) {
        if (bindingMatches(b, sc, mods)) { return true; }
    }
    return false;
}

bool slotHeld(Slot s, const Uint8 *keyState, Uint16 mods, const KeyBindings &kb) {
    for (const auto &b : kb.perSlot[(size_t)s]) {
        if (keyState[b.sc] && (mods & KMOD_RELEVANT) == b.mods) { return true; }
    }
    return false;
}

bool slotSimKey(Slot s, SDL_Scancode sc, const KeyBindings &kb) {
    for (const auto &b : kb.perSlot[(size_t)s]) {
        if (b.sc == sc && b.mods == 0) { return true; }
    }
    return false;
}

// ---------------------------------------------------------------------------
// Naming. slotName is the persistent identifier (settings.json + the
// --bind CLI); slotLabel the display string; slotGroup the UI category.
// Switches (not a parallel table) so a new Slot can't silently misalign.
// ---------------------------------------------------------------------------
const char *slotName(Slot s) {
    switch (s) {
        case Slot::WarpUp:         return "warp_up";
        case Slot::WarpDown:       return "warp_down";
        case Slot::CamSpeedUp:     return "cam_speed_up";
        case Slot::CamSpeedDown:   return "cam_speed_down";
        case Slot::ToggleCamMode:  return "toggle_cam_mode";
        case Slot::CycleTarget:    return "cycle_target";
        case Slot::ToggleWindows:  return "toggle_windows";
        case Slot::NextShip:       return "next_ship";
        case Slot::ToggleEva:      return "toggle_eva";
        case Slot::Space:          return "space";
        case Slot::Screenshot:     return "screenshot";
        case Slot::Porkchop:       return "porkchop";
        case Slot::SurfaceMap:     return "surface_map";
        case Slot::Wireframe:      return "wireframe";
        case Slot::ResetWindows:   return "reset_windows";
        case Slot::Menu:           return "menu";
        case Slot::PitchUp:        return "pitch_up";
        case Slot::PitchDown:      return "pitch_down";
        case Slot::YawLeft:        return "yaw_left";
        case Slot::YawRight:       return "yaw_right";
        case Slot::RollLeft:       return "roll_left";
        case Slot::RollRight:      return "roll_right";
        case Slot::Thrust:         return "thrust";
        case Slot::ThrustLatch:    return "thrust_latch";
        case Slot::KillRot:        return "kill_rot";
        case Slot::ThrottleUp:     return "throttle_up";
        case Slot::ThrottleDown:   return "throttle_down";
        case Slot::CamForward:     return "cam_forward";
        case Slot::CamBack:        return "cam_back";
        case Slot::CamStrafeLeft:  return "cam_strafe_left";
        case Slot::CamStrafeRight: return "cam_strafe_right";
        case Slot::CamRollLeft:    return "cam_roll_left";
        case Slot::CamRollRight:   return "cam_roll_right";
        case Slot::CamUp:          return "cam_up";
        case Slot::CamDown:        return "cam_down";
        case Slot::EvaForward:     return "eva_forward";
        case Slot::EvaBack:        return "eva_back";
        case Slot::EvaLeft:        return "eva_left";
        case Slot::EvaRight:       return "eva_right";
        case Slot::EvaUp:          return "eva_up";
        case Slot::EvaDown:        return "eva_down";
        case Slot::EvaYawLeft:     return "eva_yaw_left";
        case Slot::EvaYawRight:    return "eva_yaw_right";
        case Slot::SLOT_COUNT:     break;
    }
    return nullptr;
}

Slot slotFromName(const char *name) {
    if (name == nullptr) { return Slot::SLOT_COUNT; }
    for (size_t i = 0; i < (size_t)Slot::SLOT_COUNT; i++) {
        const char *n = slotName((Slot)i);
        if (n != nullptr && std::strcmp(n, name) == 0) { return (Slot)i; }
    }
    return Slot::SLOT_COUNT;
}

const char *slotLabel(Slot s) {
    switch (s) {
        case Slot::WarpUp:         return "Warp up";
        case Slot::WarpDown:       return "Warp down";
        case Slot::CamSpeedUp:     return "Camera speed up";
        case Slot::CamSpeedDown:   return "Camera speed down";
        case Slot::ToggleCamMode:  return "Toggle camera mode";
        case Slot::CycleTarget:    return "Cycle orbit target";
        case Slot::ToggleWindows:  return "Toggle windows";
        case Slot::NextShip:       return "Next ship";
        case Slot::ToggleEva:      return "Toggle EVA";
        case Slot::Space:          return "Stage / jump";
        case Slot::Screenshot:     return "Screenshot";
        case Slot::Porkchop:       return "Compute porkchop";
        case Slot::SurfaceMap:     return "Compute surface map";
        case Slot::Wireframe:      return "Toggle wireframe";
        case Slot::ResetWindows:   return "Reset windows";
        case Slot::Menu:           return "Main menu";
        case Slot::PitchUp:        return "Pitch up";
        case Slot::PitchDown:      return "Pitch down";
        case Slot::YawLeft:        return "Yaw left";
        case Slot::YawRight:       return "Yaw right";
        case Slot::RollLeft:       return "Roll left";
        case Slot::RollRight:      return "Roll right";
        case Slot::Thrust:         return "Thrust";
        case Slot::ThrustLatch:    return "Thrust latch (hold thrust)";
        case Slot::KillRot:        return "Kill rotation";
        case Slot::ThrottleUp:     return "Throttle up";
        case Slot::ThrottleDown:   return "Throttle down";
        case Slot::CamForward:     return "Camera forward";
        case Slot::CamBack:        return "Camera back";
        case Slot::CamStrafeLeft:  return "Camera strafe left";
        case Slot::CamStrafeRight: return "Camera strafe right";
        case Slot::CamRollLeft:    return "Camera roll left";
        case Slot::CamRollRight:   return "Camera roll right";
        case Slot::CamUp:          return "Camera up";
        case Slot::CamDown:        return "Camera down";
        case Slot::EvaForward:     return "Walk forward";
        case Slot::EvaBack:        return "Walk back";
        case Slot::EvaLeft:        return "Walk left";
        case Slot::EvaRight:       return "Walk right";
        case Slot::EvaUp:          return "EVA up";
        case Slot::EvaDown:        return "EVA down";
        case Slot::EvaYawLeft:     return "EVA yaw left";
        case Slot::EvaYawRight:    return "EVA yaw right";
        case Slot::SLOT_COUNT:     break;
    }
    return "?";
}

SlotGroup slotGroup(Slot s) {
    switch (s) {
        case Slot::WarpUp: case Slot::WarpDown:
        case Slot::CamSpeedUp: case Slot::CamSpeedDown:
        case Slot::ToggleCamMode: case Slot::CycleTarget: case Slot::ToggleWindows:
        case Slot::NextShip: case Slot::ToggleEva: case Slot::Space:
        case Slot::Screenshot: case Slot::Porkchop: case Slot::SurfaceMap:
        case Slot::Wireframe: case Slot::ResetWindows: case Slot::Menu:
            return SlotGroup::Game;
        case Slot::PitchUp: case Slot::PitchDown:
        case Slot::YawLeft: case Slot::YawRight:
        case Slot::RollLeft: case Slot::RollRight:
        case Slot::Thrust: case Slot::ThrustLatch: case Slot::KillRot:
        case Slot::ThrottleUp: case Slot::ThrottleDown:
            return SlotGroup::Flight;
        case Slot::CamForward: case Slot::CamBack:
        case Slot::CamStrafeLeft: case Slot::CamStrafeRight:
        case Slot::CamRollLeft: case Slot::CamRollRight:
        case Slot::CamUp: case Slot::CamDown:
            return SlotGroup::Camera;
        case Slot::EvaForward: case Slot::EvaBack:
        case Slot::EvaLeft: case Slot::EvaRight:
        case Slot::EvaUp: case Slot::EvaDown:
        case Slot::EvaYawLeft: case Slot::EvaYawRight:
            return SlotGroup::Eva;
        case Slot::SLOT_COUNT: break;
    }
    return SlotGroup::GROUP_COUNT;
}

// ---------------------------------------------------------------------------
// Display: a binding as "Ctrl+Shift+W" (modifiers in a fixed order, then the
// key). Unknown scancodes fall back to "key<scancode>" so the UI never shows
// an empty slot.
// ---------------------------------------------------------------------------
namespace {
std::string keyName(SDL_Scancode sc) {
    switch (sc) {
        case SDL_SCANCODE_A: return "A";
        case SDL_SCANCODE_B: return "B";
        case SDL_SCANCODE_C: return "C";
        case SDL_SCANCODE_D: return "D";
        case SDL_SCANCODE_E: return "E";
        case SDL_SCANCODE_F: return "F";
        case SDL_SCANCODE_G: return "G";
        case SDL_SCANCODE_H: return "H";
        case SDL_SCANCODE_I: return "I";
        case SDL_SCANCODE_J: return "J";
        case SDL_SCANCODE_K: return "K";
        case SDL_SCANCODE_L: return "L";
        case SDL_SCANCODE_M: return "M";
        case SDL_SCANCODE_N: return "N";
        case SDL_SCANCODE_O: return "O";
        case SDL_SCANCODE_P: return "P";
        case SDL_SCANCODE_Q: return "Q";
        case SDL_SCANCODE_R: return "R";
        case SDL_SCANCODE_S: return "S";
        case SDL_SCANCODE_T: return "T";
        case SDL_SCANCODE_U: return "U";
        case SDL_SCANCODE_V: return "V";
        case SDL_SCANCODE_W: return "W";
        case SDL_SCANCODE_X: return "X";
        case SDL_SCANCODE_Y: return "Y";
        case SDL_SCANCODE_Z: return "Z";
        case SDL_SCANCODE_1: return "1";
        case SDL_SCANCODE_2: return "2";
        case SDL_SCANCODE_3: return "3";
        case SDL_SCANCODE_4: return "4";
        case SDL_SCANCODE_5: return "5";
        case SDL_SCANCODE_6: return "6";
        case SDL_SCANCODE_7: return "7";
        case SDL_SCANCODE_8: return "8";
        case SDL_SCANCODE_9: return "9";
        case SDL_SCANCODE_0: return "0";
        case SDL_SCANCODE_SPACE:     return "Space";
        case SDL_SCANCODE_TAB:       return "Tab";
        case SDL_SCANCODE_RETURN:    return "Return";
        case SDL_SCANCODE_ESCAPE:    return "Esc";
        case SDL_SCANCODE_BACKSPACE: return "Backspace";
        case SDL_SCANCODE_MINUS:     return "-";
        case SDL_SCANCODE_EQUALS:    return "=";
        case SDL_SCANCODE_LEFTBRACKET:  return "[";
        case SDL_SCANCODE_RIGHTBRACKET: return "]";
        case SDL_SCANCODE_BACKSLASH:   return "\\";
        case SDL_SCANCODE_SEMICOLON:   return ";";
        case SDL_SCANCODE_APOSTROPHE:  return "'";
        case SDL_SCANCODE_GRAVE:       return "`";
        case SDL_SCANCODE_COMMA:       return ",";
        case SDL_SCANCODE_PERIOD:      return ".";
        case SDL_SCANCODE_SLASH:       return "/";
        case SDL_SCANCODE_LCTRL:  return "LCtrl";
        case SDL_SCANCODE_LSHIFT: return "LShift";
        case SDL_SCANCODE_LALT:   return "LAlt";
        case SDL_SCANCODE_RCTRL:  return "RCtrl";
        case SDL_SCANCODE_RSHIFT: return "RShift";
        case SDL_SCANCODE_RALT:   return "RAlt";
        case SDL_SCANCODE_F1:  return "F1";
        case SDL_SCANCODE_F2:  return "F2";
        case SDL_SCANCODE_F3:  return "F3";
        case SDL_SCANCODE_F4:  return "F4";
        case SDL_SCANCODE_F5:  return "F5";
        case SDL_SCANCODE_F6:  return "F6";
        case SDL_SCANCODE_F7:  return "F7";
        case SDL_SCANCODE_F8:  return "F8";
        case SDL_SCANCODE_F9:  return "F9";
        case SDL_SCANCODE_F10: return "F10";
        case SDL_SCANCODE_F11: return "F11";
        case SDL_SCANCODE_F12: return "F12";
        default: {
            char buf[24];
            std::snprintf(buf, sizeof buf, "key%d", (int)sc);
            return std::string(buf);
        }
    }
}
}

std::string bindLabel(const KeyBind &b) {
    std::string s;
    // Conventional left-to-right modifier order (Shift, Ctrl, Alt). LShift and
    // RShift (and the L/R Ctrl, Alt pairs) are distinct modifier keys, so each
    // side is shown on its own -- a binding names the exact side it used.
    if (b.mods & KMOD_LSHIFT) { s += "LShift+"; }
    if (b.mods & KMOD_RSHIFT) { s += "RShift+"; }
    if (b.mods & KMOD_LCTRL)  { s += "LCtrl+"; }
    if (b.mods & KMOD_RCTRL)  { s += "RCtrl+"; }
    if (b.mods & KMOD_LALT)   { s += "LAlt+"; }
    if (b.mods & KMOD_RALT)   { s += "RAlt+"; }
    s += keyName(b.sc);
    return s;
}
