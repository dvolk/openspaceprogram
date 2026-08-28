// system.h -- the loaded star system and the JSON loader that builds it.
//
//   System        the body list + root/home/moon shortcuts.
//   load_system() reads a star-system JSON and builds each TerrainBody
//                  with its inertial + rotating frame tree.

#pragma once

#include <string>
#include <vector>

#include "terrain.h"
#include "frame.h"
#include "shader.h"

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

  JSON layout (see res/old_system.json and res/ksp_system.json):
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
          "power_scaler": int,               // legacy: default for surface.power
          "surface": {                       // optional; every field optional
            "amplitude":   m,                // peak noise height        (2500)
            "octaves":     int,              // simplex octaves          (12)
            "persistence": 0..1,             // octave falloff           (0.6)
            "frequency":   float,            // noise-domain scale       (1.0)
            "power":       int,              // height-distribution exp  (3)
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

  Must be called after create_physics(), because Create() builds Bullet terrain
  collision.
*/
System load_system(const char *path, Shader *terrainshader, Shader *sunshader);
