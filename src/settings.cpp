// settings.cpp -- SettingsData <-> settings.json (nlohmann). The reader
// is deliberately permissive: it only touches a field when the key is
// present AND of the expected type, so an absent (newer/older file) or
// mistyped (hand-edited) field leaves s's current value in place.
#include "settings.h"

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
    j["gamma"] = s.gamma;
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
    if(j.contains("gamma") && j["gamma"].is_number()) {
        s.gamma = j["gamma"].get<float>();
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
}
