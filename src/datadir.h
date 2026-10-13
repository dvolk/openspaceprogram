#pragma once

// datadir.h -- per-user data (saves/, screenshots/, ships/, settings.json).
// Default is the per-OS user data dir (SDL_GetPrefPath); --data-dir overrides.

#include <SDL3/SDL.h>

#include <filesystem>
#include <cstdio>
#include <string>

namespace datadir {

// SDL_GetPrefPath's `app` argument.
static const char *kAppName = "openspaceprogram";

// Create dir (and parents). Non-throwing: a failure surfaces at the first
// write, not as an uncaught exception at startup.
inline void make_dir(const std::string &dir) {
    if(dir.empty()) { return; }
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
}

// The data directory (trailing '/'). Empty until init(); accessors then fall
// back to cwd-relative names.
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

// Resolve + store: `override` (--data-dir) wins, else the per-OS default.
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

// User content never lands in the install tree.
inline const std::string saves() { return dir() + "saves"; }
inline const std::string screenshots() { return dir() + "screenshots"; }
inline const std::string ships() { return dir() + "ships"; }
inline const std::string settings_file() { return dir() + "settings.json"; }

}   // namespace datadir
