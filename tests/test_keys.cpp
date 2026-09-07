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
    assert(hasPlain(Slot::Thrust, SDL_SCANCODE_I));
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

    // 2) slotFired: exact-modifier match. A plain binding fires with no
    //    modifier and NOT with one held; a different key does not fire.
    assert(slotFired(Slot::Thrust, SDL_SCANCODE_I, 0, kb));
    assert(!slotFired(Slot::Thrust, SDL_SCANCODE_I, KMOD_SHIFT, kb));
    assert(!slotFired(Slot::Thrust, SDL_SCANCODE_I, KMOD_CTRL, kb));
    assert(!slotFired(Slot::Thrust, SDL_SCANCODE_J, 0, kb));

    // 3) a combo binding fires only with exactly its modifier.
    KeyBindings kb2;
    kb2.perSlot[(size_t)Slot::Thrust].clear();
    kb2.perSlot[(size_t)Slot::Thrust].push_back(KeyBind{SDL_SCANCODE_I, KMOD_CTRL});
    assert(slotFired(Slot::Thrust, SDL_SCANCODE_I, KMOD_CTRL, kb2));
    assert(!slotFired(Slot::Thrust, SDL_SCANCODE_I, 0, kb2));                 // plain press
    assert(!slotFired(Slot::Thrust, SDL_SCANCODE_I, KMOD_SHIFT, kb2));        // wrong modifier
    assert(!slotFired(Slot::Thrust, SDL_SCANCODE_I, KMOD_CTRL | KMOD_SHIFT, kb2)); // too many
    // An irrelevant modifier bit (NumLock) is masked out -> still fires.
    assert(slotFired(Slot::Thrust, SDL_SCANCODE_I, KMOD_CTRL | KMOD_NUM, kb2));

    // 4) slotHeld: the same rule through a key-state array.
    std::vector<Uint8> key(SDL_NUM_SCANCODES, 0);
    key[SDL_SCANCODE_I] = 1;
    assert(slotHeld(Slot::Thrust, key.data(), 0, kb));
    assert(!slotHeld(Slot::Thrust, key.data(), KMOD_SHIFT, kb));
    key[SDL_SCANCODE_I] = 0;
    assert(!slotHeld(Slot::Thrust, key.data(), 0, kb));

    // 5) slotSimKey: a synthetic (modifier-less) --sim-press key backs a
    //    plain binding, but not a combo one.
    assert(slotSimKey(Slot::Thrust, SDL_SCANCODE_I, kb));
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

    // 7) bindLabel: modifiers in Shift+Ctrl+Alt order, then the key.
    assert(bindLabel(KeyBind{SDL_SCANCODE_W, 0}) == "W");
    assert(bindLabel(KeyBind{SDL_SCANCODE_W, KMOD_SHIFT}) == "Shift+W");
    assert(bindLabel(KeyBind{SDL_SCANCODE_W, KMOD_CTRL}) == "Ctrl+W");
    assert(bindLabel(KeyBind{SDL_SCANCODE_W, KMOD_SHIFT | KMOD_CTRL}) == "Shift+Ctrl+W");
    assert(bindLabel(KeyBind{SDL_SCANCODE_W, KMOD_SHIFT | KMOD_CTRL | KMOD_ALT})
           == "Shift+Ctrl+Alt+W");
    assert(bindLabel(KeyBind{SDL_SCANCODE_UNKNOWN, 0}) == "key0");

    printf("test_keys: all checks passed\n");
    return 0;
}
