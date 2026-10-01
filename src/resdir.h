#pragma once

// resdir.h -- where the game finds its read-only assets (the res/ tree).
// Game code names assets as "res/..."; path() is the one place that turns
// such a name into a filesystem path at the point of I/O. root() walks up
// from SDL_GetBasePath() accepting <dir>/res or <dir>/share/openspaceprogram/res.
// Keep passing "res/..." as identity; resolve with path() only when opening.

#include <SDL3/SDL.h>

#include <filesystem>
#include <string>

namespace resdir {

// The install root that contains res/ (trailing '/'). Resolved once.
inline const std::string &root() {
    static const std::string r = []() -> std::string {
        namespace fs = std::filesystem;
        // Directory of the running binary (trailing separator). SDL caches
        // this -- do not free the pointer.
        fs::path p = ".";
        if(const char *base = SDL_GetBasePath()) {
            p = fs::path(base);
        }
        // At each level, accept <dir>/res (tarball / AppImage / dev tree) or
        // <dir>/share/openspaceprogram/res (FHS).
        for(int i = 0; i < 8; i++) {
            if(fs::is_directory(p / "res")) {
                std::string s = p.string();
                if(s.empty() || s.back() != '/') { s += '/'; }
                return s;
            }
            const fs::path share = p / "share" / "openspaceprogram";
            if(fs::is_directory(share / "res")) {
                std::string s = share.string();
                if(s.empty() || s.back() != '/') { s += '/'; }
                return s;
            }
            const fs::path parent = p.parent_path();
            if(parent.empty() || parent == p) { break; }
            p = parent;
        }
        return std::string("./");   // bare cwd (a test harness, a lost binary)
    }();
    return r;
}

// Map a game asset name onto a filesystem path. Absolute passes through;
// "res/..." is rewritten to root(); other relative paths stay cwd-relative.
inline std::string path(const std::string &p) {
    if(p.empty()) { return p; }
    // Absolute (unix, or a Windows drive): a user path, use as-is.
    if(p[0] == '/' || (p.size() >= 2 && p[1] == ':')) { return p; }
    // One spelling of a game asset: "res/...". A leading "./" (old saves /
    // fixtures) is tolerated and stripped so those keep loading.
    std::string rel = (p.compare(0, 2, "./") == 0) ? p.substr(2) : p;
    if(rel == "res" || rel.compare(0, 4, "res/") == 0) {
        return root() + rel;
    }
    return p;
}

}   // namespace resdir
