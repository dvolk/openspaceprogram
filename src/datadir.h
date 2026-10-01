#pragma once

// datadir.h -- where the game keeps its per-user data (saves/ + screenshots/
// + settings.json). Uses the per-OS user data directory (SDL_GetPrefPath);
// --data-dir overrides. Header-only, no game state.

#include <SDL3/SDL.h>

#include <filesystem>
#include <cstdio>
#include <string>

namespace datadir {

// The app name for the default directory (SDL_GetPrefPath's `app` argument).
static const char *kAppName = "openspaceprogram";

// Create dir (and parents) if missing. Non-throwing (a failure surfaces later
// at the first write, not as an uncaught exception at startup).
inline void make_dir(const std::string &dir) {
    if(dir.empty()) { return; }
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
}

// The data directory (trailing '/'). Empty until init(); accessors fall back
// to cwd-relative names (for headless unit tests that never init).
inline std::string &dir() {
    static std::string d;
    return d;
}

// Store the data directory, normalized to a trailing '/', creating it.
inline void set(const std::string &d) {
    std::string n = d;
    while(n.size() > 1 && (n.back() == '/' || n.back() == '\\')) {
        n.pop_back();
    }
    if(n.empty()) { return; }   // nothing to set (an empty dir would be "/")
    dir() = n + "/";
    make_dir(dir());
}

// Resolve + store the data directory. `override` (--data-dir) wins when
// non-empty; otherwise the per-OS default.
inline const std::string &init(const std::string &override) {
    std::string base = override;
    if(base.empty()) {
        char *p = SDL_GetPrefPath(nullptr, kAppName);
        if(p != nullptr) {
            base = p;
            SDL_free(p);
        } else {
            base = ".";   // no HOME / XDG_DATA_HOME (bare container): cwd
        }
    }
    set(base);
    printf("Data directory: %s\n", dir().c_str());
    return dir();
}

// saves/, screenshots/, ships/, and settings.json -- all under the data
// directory (user content never lands in the install tree).
inline const std::string saves() { return dir() + "saves"; }
inline const std::string screenshots() { return dir() + "screenshots"; }
inline const std::string ships() { return dir() + "ships"; }
inline const std::string settings_file() { return dir() + "settings.json"; }

}   // namespace datadir
