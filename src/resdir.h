#pragma once

// resdir.h -- where the game finds its read-only assets (the res/ tree).
//
// Game code names assets as "res/..." (parts.json, ships/*.json, meshes,
// shaders, audio). That spelling is LOGICAL: it is what CLI defaults, save
// metadata, fleet JSON and the e2e EXPECT strings use, and it is what the
// logs print. path() is the one place that turns such a name into a
// filesystem path at the point of I/O.
//
// root() is the install directory that CONTAINS res/. Today that is
// SDL_GetBasePath() (the directory of the running binary) with a short
// walk-up -- the portable layout (release tarball, AppImage) keeps res/
// next to the binary, and the walk-up covers the dev tree's ./osp symlink
// into build/<os>-<march>-<tune>/<config>/. The walk also accepts an FHS
// split (binary in .../bin, assets in .../share/openspaceprogram/res), so
// a future deb only has to land the tree there -- this file is the sole
// place that knows the layout.
//
// Header-only, no game state: the same "plain file-system ops" stance as
// datadir.h (datadir = per-user writable state; resdir = shipped assets).
//
// Call-site contract: keep passing "res/..." around as identity (args,
// save.json, toasts, logs). Resolve with path() only when opening the
// file. An already-absolute path (a --parts/--system/--font the user
// pointed somewhere else) passes through unchanged; a relative path that
// is NOT under res/ (e2e fixtures, a user's ./my_fleet.json) stays
// cwd-relative.

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
        // At each level, accept either
        //   <dir>/res                          -- tarball / AppImage / dev tree
        //   <dir>/share/openspaceprogram/res   -- FHS (/usr/bin + /usr/share/...)
        // The walk-up finds the repo root from the dev tree's build/...
        // binary and finds /usr/share/... from a /usr/bin/osp install.
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

// Map a game asset name onto a filesystem path. See the header comment for
// what is rewritten and what passes through.
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
