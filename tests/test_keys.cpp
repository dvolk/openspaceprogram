// test_keys.cpp -- keys.h/.cpp: the rebindable key map. The exact-modifier
// lookup (a plain binding fires only with no Shift/Ctrl/Alt held; a combo
// fires only with exactly its modifiers), the default map (the previously
// hardcoded keys land on the right scancodes, cam/eva up-down on R/F), the
// --sim-press plain-key compatibility, and the naming round-trip.
// Pure logic: compiles src/keys.cpp with no SDL link (no SDL calls).
#include <cassert>
#include <cstdio>
#include <string>
#include <vector>

#include "keys.h"

int main() {
    KeyBindings kb;   // default map

    // 1) defaults: the previously-hardcoded keys land on the expected
    //    physical keys (US layout), plain (no modifier). Every slot has at
    //    least one binding. Cam/Eva up-down moved off LShift/LCtrl to R/F.
    auto hasPlain = [&](Slot s, SDL_Scancode sc) {
        for (const auto &b : kb.perSlot[(size_t)s]) {
            if (b.sc == sc && b.mods == 0) { return true; }
        }
        return false;
    };
    assert(hasPlain(Slot::Thrust, SDL_SCANCODE_T));
    assert(hasPlain(Slot::WarpUp, SDL_SCANCODE_PERIOD));
    assert(hasPlain(Slot::WarpDown, SDL_SCANCODE_COMMA));
    assert(hasPlain(Slot::PitchUp, SDL_SCANCODE_W));
    assert(hasPlain(Slot::ThrottleUp, SDL_SCANCODE_R));
    assert(hasPlain(Slot::ThrottleDown, SDL_SCANCODE_F));
    assert(hasPlain(Slot::CamUp, SDL_SCANCODE_R));    // was LShift/RShift
    assert(hasPlain(Slot::CamDown, SDL_SCANCODE_F));  // was LCtrl/RCtrl
    assert(hasPlain(Slot::EvaUp, SDL_SCANCODE_R));    // was LShift
    assert(hasPlain(Slot::EvaDown, SDL_SCANCODE_F));  // was LCtrl
    for (size_t i = 0; i < (size_t)Slot::SLOT_COUNT; i++) {
        assert(!kb.perSlot[i].empty());
    }

    // 1b) The thrust latch is the one combo default: LShift+T (not plain T).
    //     LShift and RShift are distinct modifier keys, so the default names
    //     one concrete side (LShift) -- a RShift press is a different combo and
    //     must not fire it, nor must a plain T (that's the Thrust slot, the
    //     plain key that releases the latch).
    auto hasCombo = [&](Slot s, SDL_Scancode sc, Uint16 mods) {
        for (const auto &b : kb.perSlot[(size_t)s]) {
            if (b.sc == sc && b.mods == mods) { return true; }
        }
        return false;
    };
    assert(hasCombo(Slot::ThrustLatch, SDL_SCANCODE_T, SDL_KMOD_LSHIFT));
    assert(!hasPlain(Slot::ThrustLatch, SDL_SCANCODE_T));      // not a plain T
    assert(slotFired(Slot::ThrustLatch, SDL_SCANCODE_T, SDL_KMOD_LSHIFT, kb));
    assert(!slotFired(Slot::ThrustLatch, SDL_SCANCODE_T, 0, kb));   // plain T
    assert(!slotFired(Slot::ThrustLatch, SDL_SCANCODE_T, SDL_KMOD_RSHIFT, kb)); // other side
    assert(slotFired(Slot::Thrust, SDL_SCANCODE_T, 0, kb));        // plain T
    assert(!slotFired(Slot::Thrust, SDL_SCANCODE_T, SDL_KMOD_LSHIFT, kb));
    assert(slotGroup(Slot::ThrustLatch) == SlotGroup::Flight);

    // 2) slotFired: exact-modifier match. A plain binding fires with no
    //    modifier and NOT with one held; a different key does not fire.
    assert(slotFired(Slot::Thrust, SDL_SCANCODE_T, 0, kb));
    assert(!slotFired(Slot::Thrust, SDL_SCANCODE_T, SDL_KMOD_LSHIFT, kb));
    assert(!slotFired(Slot::Thrust, SDL_SCANCODE_T, SDL_KMOD_LCTRL, kb));
    assert(!slotFired(Slot::Thrust, SDL_SCANCODE_J, 0, kb));

    // 3) a combo binding fires only with exactly its modifier (and the exact
    //    side -- L/R Ctrl are distinct keys).
    KeyBindings kb2;
    kb2.perSlot[(size_t)Slot::Thrust].clear();
    kb2.perSlot[(size_t)Slot::Thrust].push_back(KeyBind{SDL_SCANCODE_I, SDL_KMOD_LCTRL});
    assert(slotFired(Slot::Thrust, SDL_SCANCODE_I, SDL_KMOD_LCTRL, kb2));
    assert(!slotFired(Slot::Thrust, SDL_SCANCODE_I, 0, kb2));                          // plain press
    assert(!slotFired(Slot::Thrust, SDL_SCANCODE_I, SDL_KMOD_RCTRL, kb2));                 // other side
    assert(!slotFired(Slot::Thrust, SDL_SCANCODE_I, SDL_KMOD_LSHIFT, kb2));                // wrong modifier
    assert(!slotFired(Slot::Thrust, SDL_SCANCODE_I, SDL_KMOD_LCTRL | SDL_KMOD_LSHIFT, kb2));   // too many
    // An irrelevant modifier bit (NumLock) is masked out -> still fires.
    assert(slotFired(Slot::Thrust, SDL_SCANCODE_I, SDL_KMOD_LCTRL | SDL_KMOD_NUM, kb2));

    // 4) slotHeld: the same rule through a key-state array. (SDL3's
    // SDL_GetKeyboardState array is bool-indexed, so the stand-in is bool
    // too -- std::vector<bool> has no .data(), hence the plain array.)
    bool key[SDL_SCANCODE_COUNT] = {false};
    key[SDL_SCANCODE_T] = true;
    assert(slotHeld(Slot::Thrust, key, 0, kb));
    assert(!slotHeld(Slot::Thrust, key, SDL_KMOD_LSHIFT, kb));
    key[SDL_SCANCODE_T] = false;
    assert(!slotHeld(Slot::Thrust, key, 0, kb));

    // 5) slotSimKey: a synthetic (modifier-less) --sim-press key backs a
    //    plain binding, but not a combo one.
    assert(slotSimKey(Slot::Thrust, SDL_SCANCODE_T, kb));
    assert(!slotSimKey(Slot::Thrust, SDL_SCANCODE_J, kb));
    assert(!slotSimKey(Slot::Thrust, SDL_SCANCODE_I, kb2));

    // 6) naming round-trip: every slot name maps back to itself.
    for (size_t i = 0; i < (size_t)Slot::SLOT_COUNT; i++) {
        Slot s = (Slot)i;
        const char *n = slotName(s);
        assert(n != nullptr);
        assert(slotFromName(n) == s);
        assert(slotLabel(s) != nullptr);
    }
    assert(slotFromName("no_such_slot") == Slot::SLOT_COUNT);
    assert(slotFromName(nullptr) == Slot::SLOT_COUNT);

    // 7) bindLabel: modifiers (each L/R side is a distinct modifier key) in
    //    Shift, Ctrl, Alt order, then the key.
    assert(bindLabel(KeyBind{SDL_SCANCODE_W, 0}) == "W");
    assert(bindLabel(KeyBind{SDL_SCANCODE_W, SDL_KMOD_LSHIFT}) == "LShift+W");
    assert(bindLabel(KeyBind{SDL_SCANCODE_W, SDL_KMOD_RSHIFT}) == "RShift+W");
    assert(bindLabel(KeyBind{SDL_SCANCODE_W, SDL_KMOD_LCTRL}) == "LCtrl+W");
    assert(bindLabel(KeyBind{SDL_SCANCODE_W, SDL_KMOD_LSHIFT | SDL_KMOD_LCTRL})
           == "LShift+LCtrl+W");
    assert(bindLabel(KeyBind{SDL_SCANCODE_W, SDL_KMOD_LSHIFT | SDL_KMOD_LCTRL | SDL_KMOD_LALT})
           == "LShift+LCtrl+LAlt+W");
    assert(bindLabel(KeyBind{SDL_SCANCODE_UNKNOWN, 0}) == "key0");

    printf("test_keys: all checks passed\n");
    return 0;
}
