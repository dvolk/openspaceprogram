#pragma once

// datadir.h -- where the game keeps its per-user data (saves/ + screenshots/
// + settings.json).
//
// The game has historically dropped all of it in the working directory (the
// repo root): fine on a dev box, but it assumes the cwd is writable and the
// same on every machine, and it pollutes the source tree. The per-OS user
// data directory is the portable home instead (SDL_GetPrefPath supplies it):
//   Linux:   $XDG_DATA_HOME/openspaceprogram/   (default ~/.local/share/...)
//   Windows: %APPDATA%\openspaceprogram  (the Roaming AppData folder)
//   macOS:   ~/Library/Application Support/openspaceprogram
// A --data-dir override (empty = default) redirects it -- portable installs,
// tests, CI.
//
// Header-only, no game state: the same "plain file-system ops" stance as
// save.h's directory helpers. Both use std::filesystem, so this whole layer
// is portable (the path resolution via SDL_GetPrefPath and the directory
// ops alike).

#include <SDL3/SDL.h>

#include <filesystem>
#include <cstdio>
#include <string>

namespace datadir {

// The app name for the default directory (SDL_GetPrefPath's `app` argument).
static const char *kAppName = "openspaceprogram";

// Create dir (and any missing parents) if it does not exist. No-op if it
// does. (std::filesystem, so portable -- the same as save.h's ensure_dir.)
// Non-throwing: a creation failure surfaces later at the settings save or
// the first write (which names the file), not as an uncaught exception at
// startup -- matching the old mkdir(2) behavior of ignoring the error.
inline void make_dir(const std::string &dir) {
    if(dir.empty()) { return; }
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
}

// The data directory (trailing '/'). Empty until init() runs; the accessors
// then fall back to cwd-relative names (the old behavior) -- a safe default
// for anything that never inits (the headless unit tests). Main-thread only
// (every settings/saves caller is; set once at startup, never concurrently).
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

// Resolve + store the data directory (main.cpp calls this once at startup,
// before anything reads or writes settings/saves). `override` (--data-dir)
// wins when non-empty; otherwise the per-OS default. Returns the stored
// directory (trailing '/').
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

// The saves/ directory (slots live under it), the screenshots/ directory
// (F12 shots land there), the user ship-defs/ directory (VAB Save), and
// the settings.json path -- all under the data directory. User-authored
// content never lands in the install tree (an AppImage's res/ is read-only).
inline const std::string saves() { return dir() + "saves"; }
inline const std::string screenshots() { return dir() + "screenshots"; }
inline const std::string ships() { return dir() + "ships"; }
inline const std::string settings_file() { return dir() + "settings.json"; }

}   // namespace datadir
