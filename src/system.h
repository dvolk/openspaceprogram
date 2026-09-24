// system.h -- the loaded star system and the JSON loader that builds it.
//
//   System        the body list + root/home/moon shortcuts.
//   load_system() reads a star-system JSON and builds each TerrainBody
//                  with its inertial + rotating frame tree.

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

//  A loaded star system
struct System {
    std::vector<TerrainBody *> bodies;
    TerrainBody *root;      // the star (frame-tree root)
    TerrainBody *home;      // the planet the ship starts on
    TerrainBody *moon;      // home's first moon, or NULL

    TerrainBody *find(const std::string &name) {
        for(auto&& b : bodies) {
            if(b->name == name) { return b; }
        }
        return nullptr;
    }
};

// TODO are these detailed docs needed?
/*
  load_system() reads a star-system description from a JSON file and builds,
  together, each TerrainBody (terrain mesh + physics constants) and the
  reference frames that belong to it:

    * every body gets an INERTIAL (non-rotating) frame, carrying the body's
      true sphere of influence and its orbital angular speed;
    * every body ALSO gets a ROTATING (near-body) frame, a child of its
      inertial frame in the frame tree, carrying the small "near body" SOI;
      bodies without a "rotating" JSON section (e.g. the star) get a dummy
      one with zero spin and soi = radius + 100 km;
    * the parent/child frame tree the per-tick SOI logic walks.

  JSON layout (see res/systems/old_system.json and res/systems/ksp_system.json):
    {
      "home": "<name of the planet the ship starts on>",
      "bodies": [
        {
          "name":  "...",
          "type":  "star" | "planet" | "moon",
          "orbits": "<parent body name>" | (omitted/null for the star),
          "radius":  metres,
          "mass":    kg,
          "g":       m/s^2 (surface gravity, used for TWR),
          "seed":         terrain-noise seed (noise-domain offset; 0 = legacy pattern),
          "has_sea":      bool,              // legacy: implies surface.sea_level = 0
          "surface": {                       // optional; every field optional
            "amplitude":   m,                // tallest relief above the
                                             // base radius            (2500)
            "octaves":     int,              // noise octaves (continents
                                             // + mountains)              (9)
            "persistence": 0..1,             // octave falloff           (0.5)
            "frequency":   float,            // feature-size multiplier  (1.0)
            "sea_level":   m,                // key present => has ocean (0)
            "sea_color":   [r,g,b],          //                         (0.1,0.1,0.8)
            "palette":     [ [t, [r,g,b]], ... ]   // land-color stops, t in 0..1
            "bands":       bool,             // gas giant: smooth sphere,
                                             // latitude bands, no terrain
            "band_count":  int,              // stripes pole to pole (9);
                                             // odd => bright band at equator
          },
          "inertial": { "soi": m, "pos": [x,y,z], "orb_ang_speed": rad/s,
                        "orb_incl": rad,       // optional; inclination of the
                                               // orbital plane; 0 = coplanar
                        "lon_asc_node": rad,   // optional; longitude of the
                                               // ascending node from the
                                               // parent's +X; 0 = node at +X
                        "ecc": 0..1,           // optional; eccentricity,
                                               // default 0 (circular)
                        "arg_peri": rad,       // optional; in-plane angle of
                                               // periapsis from +X, default 0
                        "true_anomaly0": rad },// optional; anomaly at epoch,
                                               // default: the in-plane angle
                                               // of pos (orbit starts there)
          "rotating": { "soi": m, "rot_ang_speed": rad/s,
                        "axial_tilt": rad }  // optional; lean of the spin axis
                                             // from the orbital normal toward
                                             // +X; 0 = pole on the orbit normal
        },   // "rotating" absent => dummy (zero spin, soi = radius + 100 km)
        ...
      ]
    }

  `mu` is derived from `mass` (mu = G * mass) to keep the files minimal. The
  bodies must list parents before children is NOT required — the frame tree is
  wired in a second pass, so the order in the file does not matter.

  load_system does the LIGHT phase only: surface params + the frame tree
  (orbital/physical values) + home/moon resolution. The heavy phase (max_height
  + root terrain + the atmosphere/cloud/ocean shells) is deferred by the caller
  to the JobRunner worker (TerrainBody::Finish) so the title can appear before
  every body's terrain is built; a body simply isn't drawn until its heavy
  phase lands (TerrainBody::ready). The caller must still run create_physics()
  first, because the deferred heavy phase builds Bullet terrain collision.

  `progress` (optional) is called on the caller's thread after each body's
  light phase, with (index, total, body name). The game uses it to keep
  drawing a "loading..." frame during the (now fast) light phase. Pass
  nullptr (the default) to load silently.
*/
System load_system(const char *path, Shader *terrainshader, Shader *sunshader,
                   std::function<void(size_t i, size_t total,
                                      const std::string &name)> progress = nullptr);

// The heavy phase per body (the load_system light phase's counterpart): build
// the boot-critical bodies (home, its moon, the star) synchronously so the
// title + ship are solid from the first frame, and defer the rest to the
// JobRunner worker so they stream in while the game runs (a body isn't drawn
// until its heavy phase lands). BuildClouds still posts its coverage bake, so
// that per-body cost never stalls anything. Shared by the boot (main) and the
// in-process system switch (Game::switchSystem): one "build this system's
// bodies" path. Defined in main.cpp.
void postHeavyPhase(System &sys, TerrainBody *home, TerrainBody *sun,
                    JobRunner &jobs, Shader *atmosphereshader,
                    Shader *cloudshader, Shader *oceanshader, int cloudres);

/* List the star-system slugs in `dir` (e.g. "res/systems") -- the file base
   names with the ".json" extension stripped -- sorted; empty if the directory
   is missing or holds no .json files. Only the .json entries are kept, so the
   New Game setup window offers exactly the files load_system reads
   (res/systems/<slug>.json). Header-only (std::filesystem ops), the analog of
   list_ship_defs in shipdef.h. */
inline std::vector<std::string> list_systems(const std::string &dir) {
    std::vector<std::string> names;
    namespace fs = std::filesystem;
    std::error_code ec;
    auto it = fs::directory_iterator(dir, ec);
    if(ec) { return names; }   // dir missing or not a directory
    for(const auto &entry : it) {
        const fs::path &p = entry.path();
        if(p.extension() != ".json") { continue; }
        names.push_back(p.stem().string());
    }
    std::sort(names.begin(), names.end());
    return names;
}
