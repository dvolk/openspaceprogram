#pragma once

#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

/* Ship/part data model: the JSON-backed description of what a ship is made
   of. This file is GL-free (no rendering, no Bullet) so the parse/validate
   path can be unit-tested headless; only the build step (main.cpp's
   build_ship) needs GL, for shader binding.

   JSON files (see res/parts.json and res/ships/racer.json):

   parts catalog:
     {
       "parts": [
         { "name": "engine",
           "type": "engine",              // free-form label (display only)
           "mesh": "engine.obj",          // file in res/
           "texture": "engine.png",       // file in res/
           "mass": 12500,                 // kg (dry mass of the part)
           "radius": 5.0,                 // optional, m; cross-section (x/y extent 2r), default 1.0
           "height": 2.0,                 // optional, m; stack-axis length (z extent), default 2.0
           "torque": 5000,                // optional, N m -> contributes as a reaction wheel
           "fuel_rate": 142.0,            // optional, kg/s; with exhaust_velocity -> a thruster
           "exhaust_velocity": 4400,      // optional, m/s; with fuel_rate -> a thruster (H2/LOX, Isp ~450s)
           "capacity": { "hydrogen": 26100, "lox": 26100 }, // optional, kg -> a propellant tank
           "hull_margin": 0.0             // optional, m; collision convex-hull margin
         }, ...
       ]
     }

   Behavior is driven by the PRESENCE of the optional fields, not by the
   type label: any part with torque adds to the ship's reaction-wheel
   authority; any part with fuel_rate + exhaust_velocity is a thruster;
   any part with capacity is a propellant tank (engines draw from the
   tanks; a tank's mass INCLUDES the propellant it holds, so it sheds
   mass as the engines burn -- the residual is its dry/structural mass).
   Fields combine freely -- e.g. a capsule can carry a small reaction
   wheel, or an engine can carry its own tank -- so new part kinds are
   added by editing parts.json alone, no source changes.

   ship def (a tree of parts, in CONSTRUCTION order -- every parent must be
   defined before the parts attached to it; part 0 is the root, see
   Vehicle::setRoot/attach):
     {
       "name": "Booster",
       "controller": "capsule_1",        // part id; omitted = last part
       "hull_margin": 0.0,               // optional, m; collision convex-hull margin
                                        // for EVERY part of this ship (see below)
       "parts": [
         { "part": "capsule" },
         { "part": "reaction_wheel" },
         { "part": "fuel_tank" },
         { "part": "engine" },
         { "part": "tank_r3h2", "attach": "radial", "parent": "fuel_tank_1" },
         { "part": "engine_r5h10", "attach": "down", "parent": "tank_r3h2_1" }
       ]
     }
   Per-part fields (all optional unless noted):
     part     catalog name (required)
     id       instance id; omitted -> auto "<catalog name>_<n>" (n = 1, 2, ..
              per catalog name). Must be unique within the ship.
     parent   id of the part to weld to; must already be defined (this is
              what makes cycles impossible). Omitted -> the previous part,
              so a plain linear stack is a bare part list.
     attach   "down" (default)  face-to-face on the parent's -Z face
              "up"              face-to-face on the parent's +Z face (stack
                                outward from a radially attached part, or a
                                nose part above the root)
              "radial"          child axis perpendicular, child's base face
                                on the parent's side (like a KSP booster)
              "side"            child axis parallel, side by side
     angle    degrees around the parent's stack axis (0 = parent +X);
              rotates the radial/side direction (and, for down, the child
              about the shared axis)
     offset   metres of GAP along the attach axis, beyond the touching
              faces (default 0); the weld anchors sit at the gap, so the
              solver holds it (see attachPose)
     stage    positive int, default 1. RESERVED for staging (separable
              stages); parsed + validated, no runtime effect yet.
   The absolute pad-relative offsets of the old schema are gone: the geometry
   is fully determined by the part sizes + attach mode, and build_ship
   places the ship's lowest point on the pad top.
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

/* One part TYPE (a catalog entry; ship defs reference it by name).
   `type` is a free-form display label. Behavior comes from the optional
   fields below (see the header comment): torque makes it a reaction
   wheel, fuel_rate + exhaust_velocity make it a thruster, capacity makes
   it a propellant tank. They combine freely. */
struct PartDef {
    std::string name;
    std::string type;         // free-form label (display only)
    std::string mesh;     // file in res/
    std::string texture;  // file in res/
    double mass;          // kg

    /* Physical size in metres; the .obj is authored to match (origin
       centered, +Z = stack axis): radius = cross-section (x/y extent 2r),
       height = extent along the stack axis (z extent h). Defaults are the
       legacy 2 m cube, so pre-size parts keep working. attachPose welds
       stack faces at +-h/2 and radial/side ports at r from the axis; a
       future staging cut lands on a face. */
    double radius;
    double height;

    double torque;            // N m; > 0 -> contributes as a reaction wheel
    double fuel_rate;         // kg/s at full throttle; with exhaust_velocity -> thruster
    double exhaust_velocity;  // m/s; with fuel_rate -> thruster
    std::vector<float> capacity; // kg per ResourceType; > 0 -> propellant tank

    /* Collision convex-hull margin (m), the catalog default for this
       part. -1 = not set -> the physics engine's default applies
       (OSP_HULL_MARGIN / 0.1). A ship def's hull_margin (see ShipDef)
       overrides this when set (see resolveHullMargin). */
    double hull_margin;

    PartDef();

    /* full thrust of one engine: T = (H2 + LOX flow) x ve -- both
       propellants end up in the plume, so the flow is 2 tanks */
    double fullThrust() const { return 2.0 * fuel_rate * exhaust_velocity; }
};

/* How a part is welded to its parent (see the header schema comment).
   Down/Up are the two stack faces (child axis parallel to the parent's);
   Radial/Side attach to the parent's side. */
enum class AttachMode {
    Down,    // face-to-face on the parent's -Z face (a plain stack)
    Up,      // face-to-face on the parent's +Z face (stacking OUTWARD from a
             // radially attached part, or a nose part above the root)
    Radial,  // child axis perpendicular; child's base face on the parent's side
    Side     // parallel axes, side by side
};

/* One part INSTANCE in a ship def, in construction order (index 0 = root).
   `parent` is resolved to an index at load time; it must point at an
   earlier part (that rule is what keeps the parts a tree). */
struct ShipPart {
    std::string part;      // catalog name
    std::string id;        // instance id (explicit, or auto "<name>_<n>")
    const PartDef *def;    // resolved at load time (points into the catalog)
    int parent;            // part index of the weld parent; -1 = root (part 0)
    AttachMode attach;     // how it is welded to the parent (root: unused)
    double angle;          // degrees around the parent's stack axis (0 = parent +X)
    double offset;         // m of gap along the attach axis, beyond touching faces
    int stage;             // reserved for staging; 1 = single stage
};

struct ShipDef {
    std::string name;
    std::vector<ShipPart> parts;
    int controller;        // part index; -1 = default (last part)

    /* Collision convex-hull margin (m) for EVERY part of this ship;
       -1 = not set -> fall back to the part catalog value, then the
       physics default. Ship-level (not part-level) because the welded-
       hull overlap problem depends on the SHIP'S layout: the same part
       is stable in one arrangement and not in another. */
    double hull_margin;

    int controllerIndex() const {
        if(controller < 0) { return (int)parts.size() - 1; }
        return controller;
    }
};

/* The resolved pose + weld anchors for one attachment (GL-free math; the
   same function the future VAB snap uses). All inputs/outputs are in the
   SAME world frame (parent given in world coords).

   The anchor points COINCIDE in world space at the returned child pose --
   required, because the 6DOF weld (this Bullet 2.x) enforces zero relative
   linear offset: anchors apart by the gap, not at the surfaces. */
struct AttachPose {
    glm::dvec3 childPos;
    glm::dmat3  childRot;
    glm::dvec3 parentAnchor;   // local to the parent
    glm::dvec3 childAnchor;    // local to the child
};

AttachPose attachPose(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                      const PartDef &parentDef, const PartDef &childDef,
                      AttachMode mode, double angleDeg, double offset);

/* Result of a stage split (see computeStageSplit): which parts stay vs. go,
   which constraints must be removed, and the survivors' links remapped into
   the compressed index space. GL/Bullet-free. */
struct StageSplit {
    std::vector<size_t> keptParts;      // surviving part indices (ascending)
    std::vector<size_t> droppedParts;   // part indices to delete (ascending)
    std::vector<long> newIndexOf;       // size nParts; newIndexOf[old] = new
                                        // index of a kept part, -1 if dropped
    std::vector<size_t> cutConstraints; // constraint indices to remove
    std::vector<std::pair<size_t,size_t>> keptLinks;
                                        // surviving links (both ends kept),
                                        // remapped, in original constraint order
};

/* Collision hull margin (m) resolution: the ship def value wins over the
   part catalog value; either may be unset (-1), in which case the other
   applies; both unset -> -1, and the physics engine applies its own
   default (OSP_HULL_MARGIN / 0.1). Pure, so the precedence is
   unit-testable headless. */
double resolveHullMargin(double shipMargin, double partMargin);

/* Pure stage-split bookkeeping (unit-testable headless). Given nParts, the
   (a,b) part-index link of each constraint (parallel to the constraints),
   and drop[i] = "part i is being dropped", returns:
     keptParts / droppedParts : the two complementary index sets
     cutConstraints           : every constraint with AT LEAST ONE dropped
                               end (both "cut" links and the drop-set's
                               internal links -- any constraint touching a
                               deleted body must be removed from the world)
     keptLinks                : the constraints whose BOTH ends are kept, with
                               their endpoints remapped to the compressed
                               (kept-only) index space
   Precondition: every link endpoint is a valid part index [0, nParts). */
StageSplit computeStageSplit(size_t nParts,
                             const std::vector<std::pair<size_t,size_t>> &links,
                             const std::vector<bool> &drop);

struct PartsCatalog {
    std::vector<PartDef> parts;

    const PartDef *find(const std::string &name) const;
};

/* Parse + validate, in the load_system() style: throws std::runtime_error
   with the file and the offending field on any bad/missing data. The
   catalog must outlive any ShipDef (the parts point into it). */
PartsCatalog load_parts_catalog(const char *path);
ShipDef load_ship_def(const char *path, const PartsCatalog &catalog);
