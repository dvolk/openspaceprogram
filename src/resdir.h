#pragma once

// resdir.h -- read-only assets (the res/ tree). Game code names assets
// "res/..." and keeps that identity; path() resolves it only at the point of
// I/O.
//
// The search itself is std::filesystem only, so a data-model TU that links
// without SDL or GL (shipdef.cpp's part geometry fingerprint, #456) can resolve
// a res path the same way the renderer does. main.cpp hands in the executable's
// directory as the hint before anything loads; with no hint -- a test harness, a
// lost binary -- the search starts at cwd.

#include <cassert>
#include <filesystem>
#include <string>

namespace resdir {

// The hint, empty until set. A separate function-local static because root()'s
// first call freezes it.
inline std::string &hint() {
    static std::string h;
    return h;
}

// Set once root() has searched: after that a hint can never take effect, and a
// silent no-op is how an install-tree launch (cwd far from res/) starts loading
// the wrong tree -- invisible in the dev tree, where cwd IS the repo.
inline bool &frozen() {
    static bool f = false;
    return f;
}

// Call once at startup with SDL_GetBasePath()'s answer (SDL owns the pointer --
// do not free it). It must precede the first path(): that ordering is the whole
// contract, so it asserts rather than degrades.
inline void setHint(const char *base) {
    assert(!frozen());
    if(base == nullptr) { return; }
    if(!hint().empty()) { return; }
    hint() = base;
}

// The install root that contains res/ (trailing '/'). Resolved once.
inline const std::string &root() {
    static const std::string r = []() -> std::string {
        frozen() = true;
        namespace fs = std::filesystem;
        fs::path p = hint().empty() ? fs::path(".") : fs::path(hint());
        // Each level: <dir>/res (dev tree) or
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

// Absolute paths pass through; "res/..." is rewritten onto root(); other
// relative paths stay cwd-relative.
inline std::string path(const std::string &p) {
    if(p.empty()) { return p; }
    // Unix or Windows-drive absolute: a user path, use as-is.
    if(p[0] == '/' || (p.size() >= 2 && p[1] == ':')) { return p; }
    // A leading "./" (old saves / fixtures) is tolerated and stripped.
    std::string rel = (p.compare(0, 2, "./") == 0) ? p.substr(2) : p;
    if(rel == "res" || rel.compare(0, 4, "res/") == 0) {
        return root() + rel;
    }
    return p;
}

}   // namespace resdir
