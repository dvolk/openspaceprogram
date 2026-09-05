// siminput.cpp -- see siminput.h.

#include "siminput.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <map>

SDL_Keycode sim_parse_key(const std::string &s) {
    // decimal SDL keycode, e.g. 32 = SPACE
    if(!s.empty()) {
        char *end = nullptr;
        const unsigned long v = strtoul(s.c_str(), &end, 10);
        if(end != s.c_str() && *end == '\0') {
            return (SDL_Keycode)v;
        }
    }
    // This SDL header defines no uppercase letter aliases (SDLK_a..z only).
    static const std::map<std::string, SDL_Keycode> names = {
        {"A", SDLK_a}, {"B", SDLK_b}, {"C", SDLK_c}, {"D", SDLK_d},
        {"E", SDLK_e}, {"F", SDLK_f}, {"G", SDLK_g}, {"H", SDLK_h},
        {"I", SDLK_i}, {"J", SDLK_j}, {"K", SDLK_k}, {"L", SDLK_l},
        {"M", SDLK_m}, {"N", SDLK_n}, {"O", SDLK_o}, {"P", SDLK_p},
        {"Q", SDLK_q}, {"R", SDLK_r}, {"S", SDLK_s}, {"T", SDLK_t},
        {"U", SDLK_u}, {"V", SDLK_v}, {"W", SDLK_w}, {"X", SDLK_x},
        {"Y", SDLK_y}, {"Z", SDLK_z},
        {"SPACE", SDLK_SPACE}, {"TAB", SDLK_TAB},
        {"RETURN", SDLK_RETURN}, {"ENTER", SDLK_RETURN},
        {"ESCAPE", SDLK_ESCAPE},
        {"PERIOD", SDLK_PERIOD}, {"COMMA", SDLK_COMMA},
        {"LSHIFT", SDLK_LSHIFT}, {"RSHIFT", SDLK_RSHIFT},
        {"LCTRL", SDLK_LCTRL}, {"RCTRL", SDLK_RCTRL},
        {"LEFT", SDLK_LEFT}, {"RIGHT", SDLK_RIGHT},
        {"UP", SDLK_UP}, {"DOWN", SDLK_DOWN},
        {"F1", SDLK_F1}, {"F2", SDLK_F2}, {"F3", SDLK_F3}, {"F4", SDLK_F4},
        {"F5", SDLK_F5}, {"F6", SDLK_F6}, {"F7", SDLK_F7}, {"F8", SDLK_F8},
        {"F9", SDLK_F9}, {"F10", SDLK_F10}, {"F11", SDLK_F11},
        {"F12", SDLK_F12},
    };
    std::string up;
    up.reserve(s.size());
    for(size_t i = 0; i < s.size(); i++) {
        up.push_back((char)toupper((unsigned char)s[i]));
    }
    std::map<std::string, SDL_Keycode>::const_iterator it = names.find(up);
    if(it != names.end()) {
        return it->second;
    }
    return 0; // unknown
}

int sim_parse_button(const std::string &s) {
    // decimal SDL button code (1 = LEFT, 2 = MIDDLE, 3 = RIGHT; 0 = none),
    // or a name for readability.
    if(!s.empty()) {
        char *end = nullptr;
        const unsigned long v = strtoul(s.c_str(), &end, 10);
        if(end != s.c_str() && *end == '\0') {
            return (int)v;
        }
    }
    std::string up;
    up.reserve(s.size());
    for(size_t i = 0; i < s.size(); i++) {
        up.push_back((char)toupper((unsigned char)s[i]));
    }
    static const std::map<std::string, int> names = {
        {"L", 1}, {"LEFT", 1}, {"LMB", 1},
        {"M", 2}, {"MIDDLE", 2}, {"MMB", 2},
        {"R", 3}, {"RIGHT", 3}, {"RMB", 3},
        {"WHEEL_UP", 4}, {"WHEELDOWN", 5}, {"WHEEL_DOWN", 5},
        {"NONE", 0},
    };
    std::map<std::string, int>::const_iterator it = names.find(up);
    if(it != names.end()) {
        return it->second;
    }
    return -1; // unknown
}

// "1d 04:03:02" or "04:03:02" — ToF / orbit-period readouts.
std::string fmt_time(double s) {
    if(s < 0.0) s = 0.0;
    const int d = (int)(s / 86400.0);
    const int h = (int)(s / 3600.0) % 24;
    const int m = (int)(s / 60.0) % 60;
    const int sec = (int)s % 60;
    char buf[32];
    if(d > 0) { snprintf(buf, sizeof buf, "%dd %02d:%02d:%02d", d, h, m, sec); }
    else     { snprintf(buf, sizeof buf, "%02d:%02d:%02d", h, m, sec); }
    return buf;
}
