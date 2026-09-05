// settings.h -- the Settings window state as plain data + its JSON (de)
// serialization (nlohmann). One field per Settings-window control, so the
// file format has a single home: Game (game.cpp) maps its members +
// GameArgs + PostFX onto SettingsData to save (the window's "Save" button)
// and load (startup). A field the CLI set explicitly (GameArgs::cli_given)
// beats the file, field by field.
#pragma once

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

// The settings file (relative to the working directory; the game runs
// from the repo root).
static const char *kSettingsFile = "settings.json";

// One field per Settings window control. Defaults mirror the launch
// defaults (GameArgs + Game). The load path starts a SettingsData from
// the live state and parses the file over it, so a field the file does
// not mention keeps its current value.
struct SettingsData {
    // display (GameArgs)
    int window_mode = 0;          // WindowMode index (0 = windowed)
    int screen_width = 1920;
    int screen_height = 1080;
    int msaa_samples = 4;
    // draw toggles (Game)
    bool physics_debug_drawing = false;
    bool world_drawing = true;
    bool draw_starfield = true;
    bool draw_skylines = false;
    // postfx (PostFX): the enabled effect names (pass order) + the gamma
    // strength (stored even when gamma is off, so re-enabling restores it)
    std::vector<std::string> postfx_enabled;
    float gamma = 1.0f;
    // ui (Game)
    int ui_style = 0;             // 0 dark, 1 light, 2 classic
    float window_rounding = 0.0f;
    float ui_alpha = 1.0f;
    float ui_scale = 1.0f;
    // camera / terrain / test knobs (GameArgs)
    float camFovDeg = 60.0f;
    int terrain_px = 512;
    float exhaust_scale = 1.0f;
    // control flips (Game)
    bool flip_pitch = false;
    bool flip_yaw = false;
    bool flip_roll = false;
};

// Fill j with s; overwrite s's fields from j, skipping absent keys and
// mistyped ones (a hand-edited or newer/older file must not crash the
// load -- the current value stands for anything unrecognized).
void settings_write(const SettingsData &s, nlohmann::json &j);
void settings_read(const nlohmann::json &j, SettingsData &s);
