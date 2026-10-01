// settings.h -- the Settings window state as plain data + its JSON
// serialization. One field per Settings-window control. The CLI beats the
// file field by field (GameArgs::cli_given).
#pragma once

#include <map>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "keys.h"   // KeyBindings (the rebindable key map)

struct SettingsData {
    // display
    int window_mode = 0;          // WindowMode index (0 = windowed)
    int screen_width = 1920;
    int screen_height = 1080;
    int msaa_samples = 4;
    // draw toggles
    bool physics_debug_drawing = false;
    bool world_drawing = true;
    bool draw_starfield = true;
    bool draw_skylines = false;
    // postfx: enabled effect names (pass order) + param values (stored even
    // when the effect is off, so re-enabling restores them).
    std::vector<std::string> postfx_enabled;
    std::map<std::string, std::map<std::string, float>> postfx_params;
    // ui
    int ui_style = 0;             // 0 dark, 1 light, 2 classic
    float window_rounding = 0.0f;
    float ui_alpha = 1.0f;
    float ui_scale = 1.0f;
    // audio: master levels in [0,1]
    float sfx_volume = 1.0f;
    float music_volume = 0.5f;
    // camera / terrain / test knobs
    float camFovDeg = 60.0f;
    int terrain_px = 512;
    float cam_shake = 1.0f;
    // control flips
    bool flip_pitch = false;
    bool flip_yaw = false;
    bool flip_roll = false;
    KeyBindings keybinds;
};

// Fill j with s; overwrite s's fields from j, skipping absent/mistyped keys.
void settings_write(const SettingsData &s, nlohmann::json &j);
void settings_read(const nlohmann::json &j, SettingsData &s);

// Read settings.json over s if it exists and parses; false when missing or invalid.
bool settings_load_file(SettingsData &s);
