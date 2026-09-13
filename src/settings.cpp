// settings.cpp -- SettingsData <-> settings.json (nlohmann). The reader
// is deliberately permissive: it only touches a field when the key is
// present AND of the expected type, so an absent (newer/older file) or
// mistyped (hand-edited) field leaves s's current value in place.
#include "settings.h"

#include <cstdio>
#include <fstream>
#include <iterator>

namespace {
const char *mode_name(int m) {
    switch(m) {
        case 1: return "borderless";
        case 2: return "fullscreen";
        case 3: return "exclusive";
        default: return "windowed";
    }
}
}

void settings_write(const SettingsData &s, nlohmann::json &j) {
    j["version"] = 1;
    j["window_mode"] = mode_name(s.window_mode);
    j["screen_width"] = s.screen_width;
    j["screen_height"] = s.screen_height;
    j["msaa"] = s.msaa_samples;
    j["physics_debug_drawing"] = s.physics_debug_drawing;
    j["world_drawing"] = s.world_drawing;
    j["starfield"] = s.draw_starfield;
    j["reference_circles"] = s.draw_skylines;
    j["postfx"] = s.postfx_enabled;   // enabled effect names, pass order
    nlohmann::json params = nlohmann::json::object();
    for(const auto &fx : s.postfx_params) {
        nlohmann::json inner = nlohmann::json::object();
        for(const auto &p : fx.second) {
            inner[p.first] = p.second;
        }
        params[fx.first] = inner;
    }
    j["postfx_params"] = params;
    j["ui_style"] = s.ui_style;
    j["window_rounding"] = s.window_rounding;
    j["ui_alpha"] = s.ui_alpha;
    j["ui_scale"] = s.ui_scale;
    j["fov"] = s.camFovDeg;
    j["terrain_px"] = s.terrain_px;
    j["exhaust_scale"] = s.exhaust_scale;
    j["flip_pitch"] = s.flip_pitch;
    j["flip_yaw"] = s.flip_yaw;
    j["flip_roll"] = s.flip_roll;
    // keybinds: slot name -> list of {sc, mods}. Every slot is written (a
    // cleared slot as an empty list), so a save->load round-trips exactly,
    // including bindings the user unbound.
    nlohmann::json kb = nlohmann::json::object();
    for (size_t i = 0; i < (size_t)Slot::SLOT_COUNT; i++) {
        const char *name = slotName((Slot)i);
        if (name == nullptr) { continue; }
        nlohmann::json arr = nlohmann::json::array();
        for (const auto &b : s.keybinds.perSlot[i]) {
            nlohmann::json e = nlohmann::json::object();
            e["sc"] = (int)b.sc;
            e["mods"] = (int)b.mods;
            arr.push_back(e);
        }
        kb[name] = arr;
    }
    j["keybinds"] = kb;
}

void settings_read(const nlohmann::json &j, SettingsData &s) {
    if(!j.is_object()) { return; }

    // window_mode: a name (the same words --sim-mode / the e2e cases use);
    // an unknown word keeps the current value.
    if(j.contains("window_mode") && j["window_mode"].is_string()) {
        const std::string m = j["window_mode"].get<std::string>();
        if(m == "windowed" || m == "window") { s.window_mode = 0; }
        else if(m == "borderless") { s.window_mode = 1; }
        else if(m == "fullscreen") { s.window_mode = 2; }
        else if(m == "exclusive") { s.window_mode = 3; }
    }
    if(j.contains("screen_width") && j["screen_width"].is_number_integer()) {
        s.screen_width = j["screen_width"].get<int>();
    }
    if(j.contains("screen_height") && j["screen_height"].is_number_integer()) {
        s.screen_height = j["screen_height"].get<int>();
    }
    if(j.contains("msaa") && j["msaa"].is_number_integer()) {
        s.msaa_samples = j["msaa"].get<int>();
    }
    if(j.contains("physics_debug_drawing") &&
       j["physics_debug_drawing"].is_boolean()) {
        s.physics_debug_drawing = j["physics_debug_drawing"].get<bool>();
    }
    if(j.contains("world_drawing") && j["world_drawing"].is_boolean()) {
        s.world_drawing = j["world_drawing"].get<bool>();
    }
    if(j.contains("starfield") && j["starfield"].is_boolean()) {
        s.draw_starfield = j["starfield"].get<bool>();
    }
    if(j.contains("reference_circles") &&
       j["reference_circles"].is_boolean()) {
        s.draw_skylines = j["reference_circles"].get<bool>();
    }
    // postfx: a list of effect names. Unknown names are ignored here and
    // again at apply time (only PostFX::Available() names are toggled).
    if(j.contains("postfx") && j["postfx"].is_array()) {
        s.postfx_enabled.clear();
        for(const auto &e : j["postfx"]) {
            if(e.is_string()) { s.postfx_enabled.push_back(e.get<std::string>()); }
        }
    }
    // postfx_params: effect -> param name -> value; non-object entries and
    // mistyped values are skipped, the rest lands as-is (unknown effect or
    // param names are harmless -- SetParam ignores them at apply time).
    if(j.contains("postfx_params") && j["postfx_params"].is_object()) {
        s.postfx_params.clear();
        for(auto it = j["postfx_params"].begin();
            it != j["postfx_params"].end(); ++it) {
            if(!it.value().is_object()) { continue; }
            for(auto pit = it.value().begin(); pit != it.value().end(); ++pit) {
                if(pit.value().is_number()) {
                    s.postfx_params[it.key()][pit.key()] =
                        pit.value().get<float>();
                }
            }
        }
    }
    if(j.contains("ui_style") && j["ui_style"].is_number_integer()) {
        s.ui_style = j["ui_style"].get<int>();
    }
    if(j.contains("window_rounding") && j["window_rounding"].is_number()) {
        s.window_rounding = j["window_rounding"].get<float>();
    }
    if(j.contains("ui_alpha") && j["ui_alpha"].is_number()) {
        s.ui_alpha = j["ui_alpha"].get<float>();
    }
    if(j.contains("ui_scale") && j["ui_scale"].is_number()) {
        s.ui_scale = j["ui_scale"].get<float>();
    }
    if(j.contains("fov") && j["fov"].is_number()) {
        s.camFovDeg = j["fov"].get<float>();
    }
    if(j.contains("terrain_px") && j["terrain_px"].is_number_integer()) {
        s.terrain_px = j["terrain_px"].get<int>();
    }
    if(j.contains("exhaust_scale") && j["exhaust_scale"].is_number()) {
        s.exhaust_scale = j["exhaust_scale"].get<float>();
    }
    if(j.contains("flip_pitch") && j["flip_pitch"].is_boolean()) {
        s.flip_pitch = j["flip_pitch"].get<bool>();
    }
    if(j.contains("flip_yaw") && j["flip_yaw"].is_boolean()) {
        s.flip_yaw = j["flip_yaw"].get<bool>();
    }
    if(j.contains("flip_roll") && j["flip_roll"].is_boolean()) {
        s.flip_roll = j["flip_roll"].get<bool>();
    }
    // keybinds: object of slot name -> array of {sc, mods}. Per-slot merge
    // (a slot the file does not mention keeps its current bindings), and
    // mistyped entries are skipped, matching the rest of the reader. A
    // present-but-empty list clears that slot (the user unbound it).
    if(j.contains("keybinds") && j["keybinds"].is_object()) {
        const nlohmann::json &kbo = j["keybinds"];
        for(auto it = kbo.begin(); it != kbo.end(); ++it) {
            Slot slot = slotFromName(it.key().c_str());
            if(slot == Slot::SLOT_COUNT) { continue; }   // unknown slot: skip
            if(!it.value().is_array()) { continue; }     // mistyped: skip
            const nlohmann::json &arr = it.value();
            if(arr.empty()) {
                // An explicit empty list clears the slot (the user unbound it).
                s.keybinds.perSlot[(size_t)slot].clear();
                continue;
            }
            std::vector<KeyBind> binds;
            for(const auto &e : arr) {
                if(!e.is_object()) { continue; }
                int sc = -1, mods = 0;
                if(e.contains("sc") && e["sc"].is_number_integer()) {
                    sc = e["sc"].get<int>();
                }
                if(e.contains("mods") && e["mods"].is_number_integer()) {
                    mods = e["mods"].get<int>();
                }
                if(sc < 0 || sc >= SDL_SCANCODE_COUNT) { continue; }
                binds.push_back(KeyBind{(SDL_Scancode)sc,
                                        (Uint16)(mods & KMOD_RELEVANT)});
            }
            // Non-empty list but no valid entry: a hand-edited mistake, not
            // an unbind -- keep the slot's current bindings.
            if(binds.empty()) { continue; }
            s.keybinds.perSlot[(size_t)slot] = binds;
        }
    }
}

bool settings_load_file(SettingsData &s) {
    std::ifstream f(kSettingsFile);
    if(!f) { return false; }   // no settings.json: the caller's values stand
    std::string text((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    nlohmann::json j;
    try {
        j = nlohmann::json::parse(text);
    } catch(const std::exception &e) {
        printf("settings.json: %s (ignored)\n", e.what());
        return false;
    }
    settings_read(j, s);
    return true;
}
