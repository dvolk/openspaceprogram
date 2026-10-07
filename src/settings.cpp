// settings.cpp -- SettingsData <-> settings.json (nlohmann). Permissive reader:
// only touches a field when the key is present AND of the expected type.
#include "settings.h"
#include "datadir.h"

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
    j["sky_dim"] = s.sky_dim;
    j["sky_dim_cone"] = s.sky_dim_cone;
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
    j["sfx_volume"] = s.sfx_volume;
    j["music_volume"] = s.music_volume;
    j["fov"] = s.camFovDeg;
    j["terrain_px"] = s.terrain_px;
    j["cam_shake"] = s.cam_shake;
    j["flip_pitch"] = s.flip_pitch;
    j["flip_yaw"] = s.flip_yaw;
    j["flip_roll"] = s.flip_roll;
    // keybinds: every slot is written (cleared slots as empty lists) so
    // save->load round-trips exactly.
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

    // window_mode: a name (same words --sim-mode uses); unknown keeps the current value.
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
    // Clamp to the --sky-dim / --sky-dim-cone ranges: a hand-edited file
    // must not put the fade outside the range the sliders can show.
    if(j.contains("sky_dim") && j["sky_dim"].is_number()) {
        s.sky_dim = j["sky_dim"].get<float>();
        if(s.sky_dim < 0.0f) { s.sky_dim = 0.0f; }
        if(s.sky_dim > 1.0f) { s.sky_dim = 1.0f; }
    }
    if(j.contains("sky_dim_cone") && j["sky_dim_cone"].is_number()) {
        s.sky_dim_cone = j["sky_dim_cone"].get<float>();
        if(s.sky_dim_cone < 1.0f) { s.sky_dim_cone = 1.0f; }
        if(s.sky_dim_cone > 179.0f) { s.sky_dim_cone = 179.0f; }
    }
    // postfx: unknown names are ignored here and at apply time.
    if(j.contains("postfx") && j["postfx"].is_array()) {
        s.postfx_enabled.clear();
        for(const auto &e : j["postfx"]) {
            if(e.is_string()) { s.postfx_enabled.push_back(e.get<std::string>()); }
        }
    }
    // postfx_params: mistyped entries are skipped.
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
    if(j.contains("sfx_volume") && j["sfx_volume"].is_number()) {
        s.sfx_volume = j["sfx_volume"].get<float>();
    }
    if(j.contains("music_volume") && j["music_volume"].is_number()) {
        s.music_volume = j["music_volume"].get<float>();
    }
    if(j.contains("fov") && j["fov"].is_number()) {
        s.camFovDeg = j["fov"].get<float>();
    }
    if(j.contains("terrain_px") && j["terrain_px"].is_number_integer()) {
        s.terrain_px = j["terrain_px"].get<int>();
    }
    if(j.contains("cam_shake") && j["cam_shake"].is_number()) {
        // Clamp: a hand-edited file bypasses the CLI's 0-3 range.
        s.cam_shake = j["cam_shake"].get<float>();
        if(s.cam_shake < 0.0f) { s.cam_shake = 0.0f; }
        if(s.cam_shake > 3.0f) { s.cam_shake = 3.0f; }
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
    // keybinds: per-slot merge; a present-but-empty list clears that slot.
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
    const std::string path = datadir::settings_file();
    std::ifstream f(path);
    if(!f) { return false; }   // no settings.json: the caller's values stand
    std::string text((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    nlohmann::json j;
    try {
        j = nlohmann::json::parse(text);
    } catch(const std::exception &e) {
        printf("%s: %s (ignored)\n", path.c_str(), e.what());
        return false;
    }
    settings_read(j, s);
    return true;
}
