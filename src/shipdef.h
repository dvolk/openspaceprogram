#pragma once

#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

/* Ship/part data model: the JSON-backed description of what a ship is made
   of. This file is GL-free (no rendering, no Bullet) so the parse/validate
   path can be unit-tested headless; only the build step (vehicle.cpp's
   build_ship) needs GL, for shader binding.

   JSON files (see res/parts.json and res/ships/racer.json):

   parts catalog:
     {
       "parts": [
         { "name": "engine",
           "type": "engine",              // free-form label (display only)
           "display_name": "Engine (2m)", // human-readable name (display only);
                                          //   optional; empty -> fall back to name
           "mesh": "engine.obj",          // file in res/
           "texture": "engine.png",       // file in res/
           "mass": 12500,                 // kg (dry mass of the part)
           "radius": 5.0,                 // optional, m; cross-section (x/y extent 2r), default 1.0
           "height": 2.0,                 // optional, m; stack-axis length (z extent), default 2.0
           "torque": 5000,                // optional, N m -> contributes as a reaction wheel
           "fuel_rate": 142.0,            // optional, kg/s; with exhaust_velocity -> a thruster
           "exhaust_velocity": 4400,      // optional, m/s; with fuel_rate -> a thruster (H2/LOX, Isp ~450s)
           "rcs_thrust": 5000,            // optional, N; > 0 -> RCS translation authority (burns hydrazine mono)
           "power_draw": 1000,            // optional, W; > 0 -> draws EC while active (a reaction wheel)
           "power_draw_constant": 100,    // optional, W; > 0 -> a CONSTANT EC draw, on all the time (capsule life support)
           "power_gen": 300,             // optional, W; > 0 -> a constant EC source (an RTG)
           "capacity": { "hydrogen": 26100, "lox": 26100 }, // optional, kg -> a propellant tank
           "capacity": { "ec": 157079 },  // optional, Wh -> a battery (EC storage)
           "crew_capacity": 3,             // optional, int; > 0 -> a capsule (holds that many EVA characters)
           "hull_margin": 0.0,            // optional, m; collision convex-hull margin
           "fuel_barrier": true,          // optional, bool; true -> fuel does not flow across this part (splits fuel groups)
           "docking_port": true           // optional, bool; true -> a docking port (an end face that can lock to another port)
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
       "controller": "capsule_1",        // part id; omitted = first reaction wheel
       "hull_margin": 0.0,               // optional, m; collision convex-hull margin
                                        // for EVERY part of this ship (see below)
       "parts": [
         { "part": "capsule" },
         { "part": "reaction_wheel" },
         { "part": "fuel_tank" },
         { "part": "engine" },
         { "part": "tank_r1.5h2", "attach": "radial", "parent": "fuel_tank_1" },
         { "part": "engine_r2.25h4.5", "attach": "down", "parent": "tank_r1.5h2_1" }
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
    Hydrazine,   // monopropellant (the EVA kerbal's RCS suit)
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
    std::string display_name; // human-readable name (display only); empty -> fall back to name
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
    double rcs_thrust;        // N; > 0 -> RCS translation authority (hydrazine mono, the EVA suit's propellant)
    /* Electrical (KSP-style EC), independent of each other:
       power_draw (W) > 0        -> a part that draws EC only while ACTIVE
                                    (a reaction wheel; off when not torquing);
       power_draw_constant (W) > 0 -> a part that draws EC ALL THE TIME
                                    (capsule life support; on whether or not
                                    anything else is active);
       power_gen (W) > 0        -> a constant EC source (an RTG). A battery
       is neither draw -- it is EC STORAGE, marked by capacity[EC] > 0 (see
       Part::isBattery); a capsule carries BOTH a constant draw and its own
       small built-in battery. The per-tick balance (gen vs draw, charging
       / draining the batteries, gating the wheels when EC runs out) is
       Vehicle's job. */
    double power_draw;
    double power_draw_constant;
    double power_gen;
    std::vector<float> capacity; // kg per ResourceType; > 0 -> propellant tank

    /* Crew capacity: how many EVA characters (src/eva.h) this part can hold
       aboard. > 0 -> a capsule (a character can EVA out of it and board it,
       see the crew transitions in game.cpp); 0 -> not a capsule. The
       occupant's mass is added to / removed from this part's mass as they
       board / leave (the "capsule weight changes with crew" rule). */
    int crew_capacity;

    /* true -> a decoupler: a staging boundary. When the ship's stage
       counter reaches this part's stage, the weld to its parent is cut and
       the decoupler plus its child-side subtree (the parts attached below
       it, away from the root) are dropped -- it flies off with the stage
       like a KSP separator, not dangling under the survivor. See
       Vehicle::extractSubtreeAsShip. The part carries no other behavior (no thrust
       / wheel / tank) -- it is the separation point. */
    bool decoupler;

    /* true -> a docking port: the part's end face (whichever of its +-Z
       faces points at the other port) can lock to another docking port of a
       different ship, joining the two ships into one rigid body (see
       Vehicle::absorbShip / Game::updateDocking). Like a decoupler it
       carries no other behavior (no thrust / wheel / tank) -- it is a
       connection point. It is a fuel barrier by definition, for the same
       reason a decoupler is: a boundary between two ships' fuel systems. */
    bool docking_port;

    /* true -> a fuel barrier: propellant does NOT flow across this part, so
       it splits the parts on either side into separate fuel groups (an
       engine cannot draw fuel from across it). Decouplers are fuel barriers;
       most parts (tanks, engines, adapters, wheels, capsules) conduct fuel
       and leave it false. See Vehicle::buildFuelGroups. */
    bool fuel_barrier;

    /* true -> a fuel link: a virtual (no-mesh) one-way connection between
       fuel groups. It carries no physics (no body, no mass) -- it only
       declares that fuel may flow one way between two parts' groups (see
       Vehicle::fuelLinks). It is removed from def.parts at the top of
       build_ship, so the rest of build_ship never sees it. See ShipPart's
       from/to for the endpoints. */
    bool fuel_link;

    /* Collision convex-hull margin (m), the catalog default for this
       part. -1 = not set -> the physics engine's default applies
       (OSP_HULL_MARGIN / 0.1). A ship def's hull_margin (see ShipDef)
       overrides this when set (see resolveHullMargin). */
    double hull_margin;

    /* Aerodynamics (src/drag.h, reports/aerodynamics2026_09_11). All
       optional. Drag terms: 0 = "use the ship's global default for that
       term" (a part that sets none keeps exactly the v1 behaviour). Lift
       terms: 0 = no lift (a rocket stays a rocket). A part sets one to
       override the global for itself:
         drag_area  m^2; the part's drag cross-section. 0 = fall back to the
                    silhouette 2*radius*height (the v1 area).
         cd         the part's baseline (parasite) drag coefficient. 0 = use
                    the ship's global --drag-cd.
         k_drag     the part's off-axis (weathervane) coefficient: the part's
                    drag grows by k* (1 - (v^nose)^2) as the ship turns off
                    its nose. 0 = use the ship's global --drag-k.
         lift_area  m^2; the part's lift reference area. 0 = no lift (the
                    default; a lifting surface -- a wing/fin, added with its
                    part asset -- sets this).
         cl         the lift-curve slope (dimensionless, per radian):
                    CL = cl * alpha, so the lift force is q * lift_area *
                    cl * alpha (src/drag.h liftForce). 0 = no lift.
         stall_angle rad; the angle of attack where lift peaks and the flow
                    stalls. The lift curve is linear (CL = cl * alpha) up to
                    it, then drops to zero by 2 * stall_angle (src/drag.h
                    liftCurve). 0 = no stall (pure linear, the Phase 2 law).
                    The peak CL is cl * stall_angle. A lifting surface sets
                    it (a wing/fin, with its part asset). */
    double drag_area;
    double cd;
    double k_drag;
    double lift_area;
    double cl;
    double stall_angle;

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
   earlier part (that rule is what keeps the parts a tree). A fuel link
   (see PartDef.fuel_link) is a virtual part: it has no parent/attach --
   instead `from`/`to` name the two parts (by instance id) whose groups
   it connects, fuel flowing from -> to. The ids are resolved to Part*
   at build time (see build_ship). */
struct ShipPart {
    std::string part;      // catalog name
    std::string id;        // instance id (explicit, or auto "<name>_<n>")
    const PartDef *def;    // resolved at load time (points into the catalog)
    int parent;            // part index of the weld parent; -1 = root (part 0)
    AttachMode attach;     // how it is welded to the parent (root: unused)
    double angle;          // degrees around the parent's stack axis (0 = parent +X)
    double offset;         // m of gap along the attach axis, beyond touching faces
    int stage;             // reserved for staging; 1 = single stage
    std::string from;      // fuel link only: source part id (fuel flows out of)
    std::string to;        // fuel link only: destination part id (fuel flows into)

    bool isFuelLink() const { return def != nullptr && def->fuel_link; }
};

struct ShipDef {
    std::string name;
    std::vector<ShipPart> parts;
    int controller;        // part index; -1 = default (first reaction wheel)

    /* Collision convex-hull margin (m) for EVERY part of this ship;
       -1 = not set -> fall back to the part catalog value, then the
       physics default. Ship-level (not part-level) because the welded-
       hull overlap problem depends on the SHIP'S layout: the same part
       is stable in one arrangement and not in another. */
    double hull_margin;

    int controllerIndex() const {
        if(controller >= 0) { return controller; }
        /* default: the first part with reaction-wheel authority -- the
           SAME part Vehicle::applyRotationForce() uses for the stick
           frame, so the camera basis (built from the controller's local
           axes) and the controls agree by default. The old default (the
           LAST part) was a footgun for ships ending in a radial/side
           attachment: that tank's local frame is rotated about the nose
           by its attach angle, so the screen axes landed between the
           ship's right/up and the stick commands read as swapped or
           mixed. A ship with no wheel at all falls back to the root. */
        for(size_t i = 0; i < parts.size(); i++) {
            if(parts[i].def && parts[i].def->torque > 0.0) { return (int)i; }
        }
        return 0;
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

/* Collision hull margin (m) resolution: the ship def value wins over the
   part catalog value; either may be unset (-1), in which case the other
   applies; both unset -> -1, and the physics engine applies its own
   default (OSP_HULL_MARGIN / 0.1). Pure, so the precedence is
   unit-testable headless. */
double resolveHullMargin(double shipMargin, double partMargin);

struct PartsCatalog {
    std::vector<PartDef> parts;

    const PartDef *find(const std::string &name) const;
};

/* Parse + validate, in the load_system() style: throws std::runtime_error
   with the file and the offending field on any bad/missing data. The
   catalog must outlive any ShipDef (the parts point into it). */
PartsCatalog load_parts_catalog(const char *path);
ShipDef load_ship_def(const char *path, const PartsCatalog &catalog);
