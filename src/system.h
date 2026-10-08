// system.h -- the loaded star system and its JSON loader.
#pragma once

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

#include "terrain.h"
#include "frame.h"
#include "shader.h"
#include "job.h"

// One debris belt (an entry in the root "belts" array): a named annulus
// orbiting the STAR, inner/outer [m] from it. Deliberately the same shape as
// a body's "surface.rings" band (terragen.h) -- a belt is that object with a
// different centre -- so name/inner/outer is one vocabulary. The name is
// documentation, as RingParams.name is; the array's order is the draw order.
struct BeltParams {
    std::string name;
    double inner = 0.0;   // [m] from the star
    double outer = 0.0;   // [m] from the star
};

struct System {
    std::vector<TerrainBody *> bodies;
    TerrainBody *root;      // the star (frame-tree root)
    TerrainBody *home;      // calendar + default spawn body (JSON "home")

    // The star field: six cubemap face names in GL order (+X,-X,+Y,-Y,+Z,-Z),
    // derived by the loader from the JSON "skybox" directory -- it names the
    // six files, so the order is the loader's, not the data's. Required:
    // every shipped system names its own sky (see src/system.cpp).
    std::vector<std::string> skybox_faces;

    // Debris belts from the root "belts" array, drawn on the orbital maps
    // (gameui.cpp). Optional: absent means no bands. Belts orbit the star's
    // equator; a body's own annuli are surface.rings, not this.
    std::vector<BeltParams> belts;

    TerrainBody *find(const std::string &name) {
        for(auto&& b : bodies) {
            if(b->name == name) { return b; }
        }
        return nullptr;
    }
};

// Read a star-system JSON (see res/systems/) and build each TerrainBody with
// its inertial + rotating frame tree. Light phase only (surface params +
// frames + home); the heavy phase is postHeavyPhase. Caller must run
// create_physics() first (heavy phase builds Bullet collision).
// `progress` is called on the caller's thread after each body's light phase.
System load_system(const char *path, Shader *terrainshader, Shader *sunshader,
                   std::function<void(size_t i, size_t total,
                                      const std::string &name)> progress = nullptr);

// Geometric MEAN solar day (#201): T * 2pi / |unwrapped subsolar-lon
// sweep over T|, where T is the heliocentric year (the period of the
// frame that directly orbits the star) so moons average out the parent's
// true-anomaly rate. Handles prograde and retrograde spin/orbit and the
// tidally locked case -- do NOT use 2*pi/(spin-orb), which is wrong for
// those encodings and divides by ~0 when they match. Returns 0 when the
// body does not spin. Falls back to the sidereal day when there is no
// sun or the sweep is degenerate. Sizes the per-body Calendar.
// Call at load: the measure walks frame chains and leaves them at t=T
// (load_system re-runs UpdateOrbitRails(0) afterwards).
double measureSolarDay(Frame *root, TerrainBody *body, TerrainBody *sun);

// The heavy phase per body: `sync` builds synchronously (the player's bodies
// + the star, at most two planets), the rest stream in on the JobRunner.
// Shared by boot and Game::switchSystem. Defined in main.cpp.
void postHeavyPhase(System &sys, const std::vector<TerrainBody *> &sync,
                    JobRunner &jobs, Shader *atmosphereshader,
                    Shader *cloudshader, Shader *oceanshader,
                    Shader *ringshader, int cloudres);

// Star-system slugs in `dir` (json stems, sorted); empty if missing/none.
inline std::vector<std::string> list_systems(const std::string &dir) {
    std::vector<std::string> names;
    namespace fs = std::filesystem;
    std::error_code ec;
    auto it = fs::directory_iterator(dir, ec);
    if(ec) { return names; }
    for(const auto &entry : it) {
        const fs::path &p = entry.path();
        if(p.extension() != ".json") { continue; }
        names.push_back(p.stem().string());
    }
    std::sort(names.begin(), names.end());
    return names;
}
