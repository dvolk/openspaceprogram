#pragma once

#include <string>
#include <vector>

/* Ship/part data model: the JSON-backed description of what a ship is made
   of. This file is GL-free (no rendering, no Bullet) so the parse/validate
   path can be unit-tested headless; only the build step (main.cpp's
   build_ship) needs GL, for shader binding.

   JSON files (see res/parts.json and res/ships/basic.json):

   parts catalog:
     {
       "parts": [
         { "name": "engine",
           "type": "engine",              // capsule | reaction_wheel | engine
           "mesh": "engine.obj",          // file in res/
           "texture": "engine.png",       // file in res/
           "mass": 3000,                  // kg
           "torque": 2000,                // reaction_wheel only, N m
           "fuel_rate": 1.0,              // engine only, kg/s per tank
           "exhaust_velocity": 40492,     // engine only, m/s
           "capacity": { "hydrogen": 1000, "lox": 1000 }  // engine only, kg
         }, ...
       ]
     }

   ship def (a linear stack, root/nose first -- the same chain the ship is
   glued in, see Vehicle::setRoot/attachDown):
     {
       "name": "Basic",
       "controller": 2,                  // part index; omitted = last part
       "parts": [
         { "part": "capsule",      "offset": 10.5 },
         { "part": "reaction_wheel", "offset": 8.5 },
         { "part": "engine",       "offset": 6.5 }
       ]
     }
   `offset` is metres from the ship base (the pad top) along the stack axis.
*/

enum class ResourceType {
    Hydrogen,
    LOX,
    EC,
    Oxygen,
    Water,
    Food,
    Num
};

struct ResourceContent {
    float current[(int)ResourceType::Num];
    float capacity[(int)ResourceType::Num];

    ResourceContent() {
        for(int i = 0; i < (int)ResourceType::Num; i++) {
            current[i] = 0;
            capacity[i] = 0;
        }
    }
};

enum class VesselPartType {
    Capsule,
    ReactionWheel,
    Engine
};

const char *VesselPartTypeStr(VesselPartType& p);

/* One part TYPE (a catalog entry; ship defs reference it by name).
   Behavior fields are only meaningful for the matching type: torque for
   ReactionWheel, fuel_rate/exhaust_velocity/capacity for Engine. */
struct PartDef {
    std::string name;
    VesselPartType type;
    std::string mesh;     // file in res/
    std::string texture;  // file in res/
    double mass;          // kg

    double torque;            // N m (ReactionWheel)
    double fuel_rate;         // kg/s per tank at full throttle (Engine)
    double exhaust_velocity;  // m/s (Engine)
    std::vector<float> capacity; // kg per ResourceType (Engine)

    PartDef();

    /* full thrust of one engine: T = (H2 + LOX flow) x ve -- both
       propellants end up in the plume, so the flow is 2 tanks */
    double fullThrust() const { return 2.0 * fuel_rate * exhaust_velocity; }
};

/* One part INSTANCE in a ship def, in stack order (index 0 = root/nose). */
struct ShipPart {
    std::string part;      // catalog name
    const PartDef *def;    // resolved at load time (points into the catalog)
    double offset;         // m from the ship base along the stack axis
};

struct ShipDef {
    std::string name;
    std::vector<ShipPart> parts;
    int controller;        // part index; -1 = default (last part)

    int controllerIndex() const {
        if(controller < 0) { return (int)parts.size() - 1; }
        return controller;
    }
};

struct PartsCatalog {
    std::vector<PartDef> parts;

    const PartDef *find(const std::string &name) const;
};

/* Parse + validate, in the load_system() style: throws std::runtime_error
   with the file and the offending field on any bad/missing data. The
   catalog must outlive any ShipDef (the parts point into it). */
PartsCatalog load_parts_catalog(const char *path);
ShipDef load_ship_def(const char *path, const PartsCatalog &catalog);
