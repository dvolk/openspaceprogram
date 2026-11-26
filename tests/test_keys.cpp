// test_keys.cpp -- keys.h/.cpp: the rebindable key map. The exact-modifier
// lookup (a plain binding fires only with no Shift/Ctrl held; a combo
// fires only with exactly its modifiers; Alt is stripped from the mask because
// it backs the #1164 precision dim as a plain key of its own), the default map
// (the previously hardcoded keys land on the right scancodes, cam/eva up-down on
// R/F, the #1148 brackets on the fleet and -/= on camera speed), the --sim-press
// plain-key compatibility, and the naming round-trip.
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
    /* F3 is the one free function key beside the F1/F2 diagnostics, and the
       orbital-plane overlay is the toggle that wants it: the flag is a Settings
       checkbox, three clicks away from the cockpit it is needed in. */
    assert(hasPlain(Slot::ToggleOrbitPlane, SDL_SCANCODE_F3));
    assert(slotGroup(Slot::ToggleOrbitPlane) == SlotGroup::Game);
    assert(slotSimKey(Slot::ToggleOrbitPlane, SDL_SCANCODE_F3, kb));
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

    // 1c) Free cam is Shift+C (both sides). Plain C is ToggleCamMode only.
    assert(hasCombo(Slot::ToggleFreeCam, SDL_SCANCODE_C, SDL_KMOD_LSHIFT));
    assert(hasCombo(Slot::ToggleFreeCam, SDL_SCANCODE_C, SDL_KMOD_RSHIFT));
    assert(!hasPlain(Slot::ToggleFreeCam, SDL_SCANCODE_C));
    assert(hasPlain(Slot::ToggleCamMode, SDL_SCANCODE_C));
    assert(slotFired(Slot::ToggleCamMode, SDL_SCANCODE_C, 0, kb));
    assert(!slotFired(Slot::ToggleCamMode, SDL_SCANCODE_C, SDL_KMOD_LSHIFT, kb));
    assert(slotFired(Slot::ToggleFreeCam, SDL_SCANCODE_C, SDL_KMOD_LSHIFT, kb));
    assert(slotFired(Slot::ToggleFreeCam, SDL_SCANCODE_C, SDL_KMOD_RSHIFT, kb));
    assert(!slotFired(Slot::ToggleFreeCam, SDL_SCANCODE_C, 0, kb));
    assert(slotGroup(Slot::ToggleFreeCam) == SlotGroup::Game);

    // 1d) The VAB's staging undo (#1013) is Ctrl+Z, both ctrl sides, and NOT plain Z. Plain
    //     Z is SlewPrograde in flight, so a plain binding here would collide with it -- and
    //     it would make the undo reachable from --sim-press, which it is not: the e2e drives
    //     the Staging window's Undo button instead. Nothing else pins this slot, so without
    //     these lines a plain-Z binding would pass the suite silently.
    assert(hasCombo(Slot::UndoStaging, SDL_SCANCODE_Z, SDL_KMOD_LCTRL));
    assert(hasCombo(Slot::UndoStaging, SDL_SCANCODE_Z, SDL_KMOD_RCTRL));
    assert(!hasPlain(Slot::UndoStaging, SDL_SCANCODE_Z));
    assert(slotFired(Slot::UndoStaging, SDL_SCANCODE_Z, SDL_KMOD_LCTRL, kb));
    assert(slotFired(Slot::UndoStaging, SDL_SCANCODE_Z, SDL_KMOD_RCTRL, kb));
    assert(!slotFired(Slot::UndoStaging, SDL_SCANCODE_Z, 0, kb));
    assert(!slotSimKey(Slot::UndoStaging, SDL_SCANCODE_Z, kb));
    assert(slotGroup(Slot::UndoStaging) == SlotGroup::Game);

    /* 1e) Fleet cycling (#1148): the brackets belong to the fleet -- '[' back, ']'
       forward, both wrapping -- and F6 still steps forward next to ']'. Camera speed
       moved to -/= to free them; the reason it was put on the brackets in the first
       place (no double-fire with the held RCS keys) holds just as well on -/=.
       A slot backs a LIST of keys, so NextShip having two is not a special case --
       but that list is exactly what the Controls row now prints, so a binding sliding
       off it would be invisible to a player and only noticed as a dead key. */
    assert(hasPlain(Slot::NextShip, SDL_SCANCODE_F6));
    assert(hasPlain(Slot::NextShip, SDL_SCANCODE_RIGHTBRACKET));
    assert(hasPlain(Slot::PrevShip, SDL_SCANCODE_LEFTBRACKET));
    assert(kb.perSlot[(size_t)Slot::NextShip].size() == 2);
    assert(kb.perSlot[(size_t)Slot::PrevShip].size() == 1);
    assert(!hasPlain(Slot::CamSpeedUp, SDL_SCANCODE_RIGHTBRACKET));
    assert(!hasPlain(Slot::CamSpeedDown, SDL_SCANCODE_LEFTBRACKET));
    assert(hasPlain(Slot::CamSpeedUp, SDL_SCANCODE_EQUALS));
    assert(hasPlain(Slot::CamSpeedDown, SDL_SCANCODE_MINUS));
    // Each bracket does exactly one thing: '[' never steps forward.
    assert(slotFired(Slot::PrevShip, SDL_SCANCODE_LEFTBRACKET, 0, kb));
    assert(!slotFired(Slot::NextShip, SDL_SCANCODE_LEFTBRACKET, 0, kb));
    assert(slotFired(Slot::NextShip, SDL_SCANCODE_RIGHTBRACKET, 0, kb));
    assert(!slotFired(Slot::PrevShip, SDL_SCANCODE_RIGHTBRACKET, 0, kb));
    // --sim-press carries no modifiers, so an e2e case can only reach plain keys.
    assert(slotSimKey(Slot::PrevShip, SDL_SCANCODE_LEFTBRACKET, kb));
    assert(slotSimKey(Slot::NextShip, SDL_SCANCODE_RIGHTBRACKET, kb));
    // The Controls window lists slots BY GROUP, so a mis-grouped slot is a row that
    // does not exist: the player could never rebind what the ticket just added.
    assert(slotGroup(Slot::PrevShip) == SlotGroup::Game);
    assert(std::string(slotName(Slot::PrevShip)) == "prev_ship");
    assert(slotFromName("prev_ship") == Slot::PrevShip);

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

    /* 4b) #1164: ALT IS NOT A MODIFIER, it is a key. The exact-modifier rule used to
       make a held Alt disarm every plain flight binding -- the ticket's "Alt parks the
       vessel keys", which is the only reason anyone could park a hull one handed. Now
       Alt is stripped from the pressed mask (relevantMods) and reaches its OWN slot as
       a plain binding, so the pilot keeps WASD/T/N/I and the rings get dimmed instead.
       These pins are the whole contract: the dim arms, the plain keys survive it, the
       Shift and Ctrl chords are untouched by it, and an Alt chord is dead on purpose. */
    assert(hasPlain(Slot::PrecisionDim, SDL_SCANCODE_LALT));
    assert(hasPlain(Slot::PrecisionDim, SDL_SCANCODE_RALT));
    assert(slotGroup(Slot::PrecisionDim) == SlotGroup::Flight);
    assert(std::string(slotName(Slot::PrecisionDim)) == "precision_dim");
    assert(slotFromName("precision_dim") == Slot::PrecisionDim);
    // --sim-press carries no modifiers, so a plain binding is what makes the dim
    // driveable from an e2e case at all.
    assert(slotSimKey(Slot::PrecisionDim, SDL_SCANCODE_LALT, kb));

    // relevantMods: Alt out, everything else exactly as pressed (side included).
    assert(relevantMods(0) == 0);
    assert(relevantMods(SDL_KMOD_LALT) == 0);
    assert(relevantMods(SDL_KMOD_RALT) == 0);
    assert(relevantMods(SDL_KMOD_ALT) == 0);           // the L|R union goes too
    assert(relevantMods(SDL_KMOD_LALT | SDL_KMOD_LSHIFT) == SDL_KMOD_LSHIFT);
    assert(relevantMods(SDL_KMOD_LALT | SDL_KMOD_LCTRL) == SDL_KMOD_LCTRL);

    key[SDL_SCANCODE_LALT] = true;
    // The dim itself arms -- on its own pressed mods, and with the union bit.
    assert(slotHeld(Slot::PrecisionDim, key, SDL_KMOD_LALT, kb));
    assert(slotHeld(Slot::PrecisionDim, key, SDL_KMOD_ALT, kb));
    // The plain flight keys stay armed under it (this is the fix).
    key[SDL_SCANCODE_T] = true;
    assert(slotHeld(Slot::Thrust, key, SDL_KMOD_LALT, kb));
    assert(slotHeld(Slot::Thrust, key, SDL_KMOD_ALT, kb));
    // A chord keeps working with Alt held: the latch is LShift+T, and Alt must not
    // count as an extra modifier against it.
    assert(slotFired(Slot::ThrustLatch, SDL_SCANCODE_T, SDL_KMOD_LSHIFT | SDL_KMOD_LALT, kb));
    assert(slotFired(Slot::ToggleFreeCam, SDL_SCANCODE_C, SDL_KMOD_RSHIFT | SDL_KMOD_RALT, kb));
    assert(slotFired(Slot::UndoStaging, SDL_SCANCODE_Z, SDL_KMOD_LCTRL | SDL_KMOD_LALT, kb));
    // And a Shift chord still does NOT become a plain slot just because Alt is down.
    assert(!slotFired(Slot::Thrust, SDL_SCANCODE_T, SDL_KMOD_LSHIFT | SDL_KMOD_LALT, kb));
    key[SDL_SCANCODE_LALT] = false;
    key[SDL_SCANCODE_T] = false;

    /* An Alt chord can never fire: relevantMods takes Alt out of the value a binding is
       compared against, so nothing matches it -- not the Alt press, and not the plain
       press either (the chord's mods are not 0). settings_read drops such an entry
       rather than leaving a slot bound to a key that will never answer, which
       tests/test_settings.cpp 5c pins; this is the matching rule's own half. The key
       state has to be LIVE for the held asserts to mean anything -- with T up,
       slotHeld is false whatever the modifier rule does. */
    KeyBindings kbAlt;
    kbAlt.perSlot[(size_t)Slot::Thrust].clear();
    kbAlt.perSlot[(size_t)Slot::Thrust].push_back(KeyBind{SDL_SCANCODE_T, SDL_KMOD_LALT});
    key[SDL_SCANCODE_T] = true;
    assert(slotHeld(Slot::Thrust, key, 0, kb));                    // the plain map arms T
    assert(!slotHeld(Slot::Thrust, key, SDL_KMOD_LALT, kbAlt));    // not with Alt down
    assert(!slotHeld(Slot::Thrust, key, 0, kbAlt));                // nor with nothing down
    assert(!slotFired(Slot::Thrust, SDL_SCANCODE_T, SDL_KMOD_LALT, kbAlt));
    assert(!slotFired(Slot::Thrust, SDL_SCANCODE_T, 0, kbAlt));
    assert(!slotSimKey(Slot::Thrust, SDL_SCANCODE_T, kbAlt));
    key[SDL_SCANCODE_T] = false;

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
