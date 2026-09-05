// test_settings.cpp -- settings.h/.cpp: the SettingsData <-> settings.json
// mapping. Round-trip stability, absent-key tolerance (a field the file
// does not mention keeps the current value -- the load path starts a
// SettingsData from the live state), mistyped-key tolerance, and the
// window-mode name mapping (unknown words keep the current value).
#include <cassert>
#include <cstdio>
#include <string>

#include "settings.h"

int main() {
    // 1) round trip: every field survives write -> parse -> read.
    SettingsData s;
    s.window_mode = 3;
    s.screen_width = 640;
    s.screen_height = 480;
    s.msaa_samples = 0;
    s.physics_debug_drawing = true;
    s.world_drawing = false;
    s.draw_starfield = true;
    s.draw_skylines = true;
    s.postfx_enabled.push_back("crt");
    s.postfx_enabled.push_back("color");
    s.postfx_params["color"]["gamma"] = 1.25f;
    s.postfx_params["color"]["brightness"] = 1.5f;
    s.ui_style = 2;
    s.window_rounding = 12.0f;
    s.ui_alpha = 0.5f;
    s.ui_scale = 1.5f;
    s.camFovDeg = 77.0f;
    s.terrain_px = 256;
    s.exhaust_scale = 2.5f;
    s.flip_pitch = true;
    s.flip_yaw = false;
    s.flip_roll = true;

    nlohmann::json j;
    settings_write(s, j);
    const nlohmann::json parsed =
        nlohmann::json::parse(j.dump());   // the text a real file holds

    SettingsData r;   // defaults
    settings_read(parsed, r);

    assert(r.window_mode == 3);
    assert(r.screen_width == 640);
    assert(r.screen_height == 480);
    assert(r.msaa_samples == 0);
    assert(r.physics_debug_drawing == true);
    assert(r.world_drawing == false);
    assert(r.draw_starfield == true);
    assert(r.draw_skylines == true);
    assert(r.postfx_enabled.size() == 2);
    assert(r.postfx_enabled[0] == "crt");
    assert(r.postfx_enabled[1] == "color");
    assert(r.postfx_params["color"]["gamma"] == 1.25f);
    assert(r.postfx_params["color"]["brightness"] == 1.5f);
    assert(r.ui_style == 2);
    assert(r.window_rounding == 12.0f);
    assert(r.ui_alpha == 0.5f);
    assert(r.ui_scale == 1.5f);
    assert(r.camFovDeg == 77.0f);
    assert(r.terrain_px == 256);
    assert(r.exhaust_scale == 2.5f);
    assert(r.flip_pitch == true);
    assert(r.flip_yaw == false);
    assert(r.flip_roll == true);

    // 2) absent keys keep the current value (the load path's starting
    //    state, mimicking collect_settings): only the file's fields move.
    SettingsData cur;
    cur.window_mode = 1;
    cur.postfx_params["color"]["gamma"] = 1.7f;
    cur.camFovDeg = 65.0f;
    cur.postfx_enabled.push_back("grain");
    nlohmann::json partial = nlohmann::json::parse(
        R"({"window_mode": "exclusive", "fov": 90})");
    settings_read(partial, cur);
    assert(cur.window_mode == 3);
    assert(cur.camFovDeg == 90.0f);
    assert(cur.postfx_params["color"]["gamma"] == 1.7f);   // untouched
    assert(cur.postfx_enabled.size() == 1);
    assert(cur.postfx_enabled[0] == "grain");

    // 3) mistyped keys are skipped, not fatal.
    nlohmann::json bad = nlohmann::json::parse(
        R"({"screen_width": "six-hundred", "postfx": 42,
            "postfx_params": [1], "fov": [1, 2]})");
    SettingsData t;
    settings_read(bad, t);
    assert(t.screen_width == 1920);   // the default stands
    assert(t.postfx_enabled.empty());
    assert(t.postfx_params.empty());
    assert(t.camFovDeg == 60.0f);

    // 4) an unknown window-mode word keeps the current value.
    nlohmann::json badmode = nlohmann::json::parse(
        R"({"window_mode": "ultrawide"})");
    SettingsData m;
    m.window_mode = 2;
    settings_read(badmode, m);
    assert(m.window_mode == 2);

    printf("test_settings: all checks passed\n");
    return 0;
}
