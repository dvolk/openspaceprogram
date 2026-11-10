#pragma once

// datadir.h -- per-user data (saves/, screenshots/, ships/, settings.json).
// Default is the per-OS user data dir (SDL_GetPrefPath); --data-dir overrides.

#include <SDL3/SDL.h>

#include <nlohmann/json.hpp>

#include <filesystem>
#include <cstdio>
#include <fstream>
#include <stdexcept>
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

/* The one way the game writes a JSON file, and it is write-then-rename (#304):
   an ofstream truncates its target the moment it opens, so a crash or a full
   disk partway through leaves a half-written file behind that everything else
   still points at. Writing <path>.tmp and renaming means a reader sees either
   the whole old file or the whole new one -- rename within one filesystem is
   atomic. Every failure path unlinks the temp (save_game's orphan sweep also
   collects a .tmp a killed process left). Throws with the reason; the caller
   turns it into a sentence the player sees. */
inline void writeJson(const std::string &path, const nlohmann::json &j) {
    /* Serialize BEFORE opening anything. dump() throws on invalid UTF-8 in any
       string it walks -- and names arrive from UI text or a hand-edited file --
       so a throw with the temp already open would leave it behind. */
    const std::string text = j.dump(2) + "\n";
    const std::string tmp = path + ".tmp";
    {
        std::ofstream f(tmp.c_str());
        if(!f.is_open()) { throw std::runtime_error("cannot write " + path); }
        f << text;
        /* close() is where a full disk or an NFS error actually surfaces, and
           the destructor closes after this scope -- i.e. after the rename. An
           unchecked close failure would install a truncated temp, which is the
           exact outcome this function exists to prevent. */
        f.close();
        if(!f.good()) {
            std::error_code ec;
            std::filesystem::remove(tmp, ec);   // best effort: the throw is the answer
            throw std::runtime_error("error writing " + path);
        }
    }
    std::error_code ec;
    std::filesystem::rename(tmp, path, ec);
    if(ec) {
        std::error_code rm;
        std::filesystem::remove(tmp, rm);
        throw std::runtime_error("cannot install " + path + ": " + ec.message());
    }
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
/* Flight recorder traces (#662). A run that has claimed a game dir writes beside
   its save slots instead; this is where the ones that have not (a --startship
   boot, every scripted run) put the trace, so the artifact has one obvious home
   either way. */
inline const std::string flights() { return dir() + "flights"; }
inline const std::string settings_file() { return dir() + "settings.json"; }

}   // namespace datadir
