#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <dirent.h>

#include <glm/glm.hpp>
#include <nlohmann/json.hpp>

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
           "shroud": "engine_shroud.obj",          // optional, file in res/;
           "shroud_texture": "engine_shroud.png",  //   an OPEN-cylinder wrap
                                                   //   drawn OVER the part when
                                                   //   a part is attached on its
                                                   //   exhaust face (a child
                                                   //   below) -- hides an
                                                   //   engine's nozzle (see
                                                   //   PartDef.shroud)
           "mass": 12500,                 // kg (dry mass of the part)
           "radius": 5.0,                 // optional, m; cross-section (x/y extent 2r), default 1.0
           "height": 2.0,                 // optional, m; stack-axis length (z extent), default 2.0
           "torque": 5000,                // optional, N m -> contributes as a reaction wheel
           "fuel_rate": 142.0,            // optional, kg/s; with exhaust_velocity -> a thruster
           "exhaust_velocity": 4400,      // optional, m/s; with fuel_rate -> a thruster (H2/LOX, Isp ~450s)
           "jet": true,                   // optional, bool; with fuel_rate + exhaust_velocity
                                          //   -> an AIR-BREATHING thruster (a jet engine,
                                          //   see below); burns jet fuel (air is the free
                                          //   oxidizer, so no LOX / no delta-v)
           "jet_fan_thrust": 32000,       // optional, N; the jet's STATIC (fan) thrust at sea
                                          //   level -- the VTOL floor (no runways/wheels yet,
                                          //   so a stationary jet still pushes); default 0;
                                          //   ignored for non-jet parts
           "jet_intake_area": 0.9,        // optional, m^2; effective intake/capture area --
                                          //   the ram term rho·A·v·(v_e − v); default 0;
                                          //   ignored for non-jet parts
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
         { "part": "tank_r1.5h2", "attach": "surface", "parent": "fuel_tank_1", "angle": 0 },
         { "part": "engine_r2.25h4.5", "attach": "down", "parent": "tank_r1.5h2_1" }
       ]
     }
   Per-part fields (all optional unless noted):
     part     catalog name (required)
     id       instance id; omitted -> auto "<catalog name>_<n>" (n = 1, 2, ..
              per catalog name). Must be unique within the ship.
     parent   id of the part to attach to; must already be defined (this is
              what makes cycles impossible). Omitted -> the previous part,
              so a plain linear stack is a bare part list.

   An edge is either a STACK edge (mates two named nodes) or a SURFACE edge
   (places the child's surface node at a contact point on the parent):

     attach   "down" (default)  stack edge on the parent's -Z face
              "up"              stack edge on the parent's +Z face
              "surface"         surface edge: the child's surface node is
                                placed at a contact point + normal on the
                                parent (KSP srfAttach)
     STACK edge node selection:
     parentNode  node id on the parent to mate (default: "bottom" for down,
                 "top" for up)
     childNode   node id on the child to mate (default: "top" for down,
                 "bottom" for up)
     angle       stack edge: roll (deg) about the mating axis
     SURFACE edge contact (one of):
     point/normal  explicit contact point + outward normal in the PARENT's
                   local frame (what the editor's raycast writes)
     angle [+ z]   cylinder shorthand: contact at clock `angle` (deg, 0 =
                   parent +X), height `z` (default 0) on the parent's
                   radius; normal points radially outward
     childNode     the child's surface node id (default: its surface node)
     roll          surface edge: rotation (deg) about the contact normal
     offset   metres of GAP along the attach axis / contact normal (default 0)
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
    JetFuel,     // the onboard fuel of an air-breathing jet (air is the free oxidizer)
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

/* The steering axis a control surface acts on (its deflection produces a
   moment about exactly this axis). A surface is one or the other, like a
   real control surface: an elevator pitches, a rudder yaws, an aileron
   rolls. The moment still comes from the surface's OFFSET from the COM
   (Vehicle::applyAeroForce); the axis only picks which stick drives it and
   the force plane (pitch/yaw/roll -> up/right/up out of the flow). */
enum class ControlAxis { Pitch, Yaw, Roll };

/* The per-axis steering parameters for a control surface (I1): the abstract
   selection that Vehicle::applyAeroForce resolves to concrete vectors. Pure
   (no glm) so tests/ can pin the mapping without Bullet/GL.
   aboutAxis     which ship frame axis the moment is about: 0=right, 1=up,
                 2=nose (this must equal the reaction wheel's axis for the
                 axis, so the surface steers the way the stick expects).
   forceDirKind  the flow-plane the force acts along: 0=up (liftDir),
                 1=right (yawDir) -- both are the ship axis projected out of
                 the flow (Vehicle::applyAeroForce resolves them).
   stickIndex    the stick component that drives it: 0=roll(Q/E), 1=pitch
                 (W/S), 2=yaw(A/D).
   targetSign    the SIGN of the wheel's torque about that axis for a
                 positive stick: pitch (W/S) -> -right, yaw (A/D) -> -up,
                 roll (Q/E) -> +nose. So pitch/yaw are -1, roll is +1. */
struct ControlAxisParams {
    int aboutAxis = -1;
    int forceDirKind = -1;
    int stickIndex = -1;
    double targetSign = -1.0;
};
inline ControlAxisParams controlAxisParams(ControlAxis axis) {
    ControlAxisParams p;
    switch(axis) {
        case ControlAxis::Pitch:
            p.aboutAxis = 0; p.forceDirKind = 0; p.stickIndex = 1; p.targetSign = -1.0; break;
        case ControlAxis::Yaw:
            p.aboutAxis = 1; p.forceDirKind = 1; p.stickIndex = 2; p.targetSign = -1.0; break;
        case ControlAxis::Roll:
            p.aboutAxis = 2; p.forceDirKind = 0; p.stickIndex = 0; p.targetSign = +1.0; break;
    }
    return p;
}
/* The axis's display label (the --drag-log control-surface telemetry names
   the surface's axis by this). */
inline const char *controlAxisName(ControlAxis axis) {
    switch(axis) {
        case ControlAxis::Pitch: return "pitch";
        case ControlAxis::Yaw:   return "yaw";
        case ControlAxis::Roll:  return "roll";
    }
    return "?";
}

/* A named attachment node on a part (KSP-style stack node). `pos` is the
   node position in part-local metres (+Z = stack axis, origin at the part
   centre); `dir` is its outward direction (normalized at load). Two nodes
   mate when their world directions are anti-parallel and their positions
   coincide -- see attachNodes(). A part with no explicit nodes in JSON gets
   `top`/`bottom` synthesized from radius/height (see load_parts_catalog), so
   axis-aligned cylinder parts need no node authoring. */
struct Node {
    std::string id;
    glm::dvec3 pos;
    glm::dvec3 dir;
    /* true -> the part's surface-attach node (the KSP node_attach equivalent):
       a surface edge places THIS node at a contact point on the parent, with
       `dir` pointing inward (toward the parent). A part has at most one. Stack
       edges mate the non-surface nodes by id. */
    bool surface = false;
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
    /* Engine shroud (optional; empty = none, most parts): an open-cylinder
       mesh + texture drawn OVER the part when a part is attached on its
       exhaust face (a child below) -- a plain light-gray wrap that hides an
       engine's nozzle under the part (Vehicle::Draw / the VAB draw). Both
       fields must be set. Author it at the part's height and just inside
       its radius (0.98x here), so its rim never sits coplanar with the part
       above or the engine's top disc edge -- no z-fighting. */
    std::string shroud;          // shroud mesh file in res/
    std::string shroud_texture;  // shroud texture file in res/
    double mass;          // kg

    /* Physical size in metres; the .obj is authored to match (origin
       centered, +Z = stack axis): radius = cross-section (x/y extent 2r),
       height = extent along the stack axis (z extent h). Defaults are the
       legacy 2 m cube, so pre-size parts keep working. These seed the
       synthesized nodes (top/bottom stack faces at +-h/2, and a side surface
       node at -radius) and still drive the collision/aero extent; attachment
       itself is node-based (see `nodes`). */
    double radius;
    double height;

    /* Attachment nodes (KSP-style). Empty in JSON -> load_parts_catalog
       synthesizes `top` (+h/2, +Z) and `bottom` (-h/2, -Z) from the size
       above, so an axis-aligned cylinder part needs no authoring. A part
       declares explicit nodes only when its stack faces aren't at +-h/2 on
       the axis (an off-axis hub port; a surface node -- Phase 2). */
    std::vector<Node> nodes;

    /* Look up a node by id; nullptr if absent. Ship-def stack edges name the
       parent/child node ids they mate. */
    const Node *findNode(const std::string &id) const {
        for(size_t i = 0; i < nodes.size(); i++) {
            if(nodes[i].id == id) { return &nodes[i]; }
        }
        return nullptr;
    }

    /* The part's surface-attach node (surface == true); nullptr if it has
       none. A surface edge places this node at the parent contact. */
    const Node *findSurfaceNode() const {
        for(size_t i = 0; i < nodes.size(); i++) {
            if(nodes[i].surface) { return &nodes[i]; }
        }
        return nullptr;
    }

    /* Fill the default nodes from the size above when `nodes` is empty:
       the two axial stack faces (top +h/2, bottom -h/2) plus a side surface
       node at (-radius, 0, 0) pointing inward, so an axis-aligned cylinder
       part can both stack and surface-attach (at any clock angle, via roll)
       with no authoring. load_parts_catalog calls it for every physical part;
       hand-built defs (tests) call it directly. A part with explicit nodes is
       left alone -- it must declare its own surface node if it needs one. */
    void synthesizeNodes() {
        if(!nodes.empty()) { return; }
        Node top;
        top.id  = "top";
        top.pos = glm::dvec3(0.0, 0.0,  height / 2.0);
        top.dir = glm::dvec3(0.0, 0.0, 1.0);
        Node bottom;
        bottom.id  = "bottom";
        bottom.pos = glm::dvec3(0.0, 0.0, -height / 2.0);
        bottom.dir = glm::dvec3(0.0, 0.0, -1.0);
        Node srf;
        srf.id      = "srf";
        srf.pos     = glm::dvec3(-radius, 0.0, 0.0);
        srf.dir     = glm::dvec3(-1.0, 0.0, 0.0);
        srf.surface = true;
        nodes.push_back(top);
        nodes.push_back(bottom);
        nodes.push_back(srf);
    }

    double torque;            // N m; > 0 -> contributes as a reaction wheel
    double fuel_rate;         // kg/s at full throttle; with exhaust_velocity -> thruster
    double exhaust_velocity;  // m/s; with fuel_rate -> thruster. For a JET this is
                              // the REAL exhaust velocity (~500-600 m/s), used in
                              // drag.h jetThrust -- NOT a thrust-encoding knob.
    /* Jet engine (air-breathing) modifier on a thruster (see drag.h
       jetThrust). jet = true makes the thruster AIR-BREATHING: it burns H2
       against FREE air (no LOX), and its thrust is the momentum balance
       T = T_fan + ṁ_f·v_e + ρ·A·v·(v_e − v), gated on the local air (so a
       jet is dead in vacuum: no thrust, no burn). Ignored unless the part
       is also a thruster (fuel_rate + exhaust_velocity). */
    bool jet;
    double jet_fan_thrust;    // N; static (fan) thrust at sea level -- the VTOL floor
    double jet_intake_area;   // m^2; effective intake/capture area (the ram term)
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

    /* Aerodynamics (src/drag.h, reports/projected-drag). All optional.
       Drag: the ship's facing area is its convex-hull SILHOUETTE (the hull
       of all parts' collision vertices, see Body::hullVerts) -- a stacked
       rocket presents its true end face, a long body more side-on than
       end-on. The drag coefficient is the ship's global --drag-cd (a single
       knob; no per-part area or coefficient to author). Lift terms:
       0 = no lift (a rocket stays a rocket). A part sets one to override
       the global for itself:
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
                    it (a wing/fin, with its part asset).
   Control surfaces (deflection-driven steering authority): 0 = no control
   surface (the default; a control surface -- a rudder/elevator, added with
   its part asset -- sets these). The force is the lift law with the
   deflection in place of the angle of attack (src/drag.h controlForce):
         control_area  m^2; the part's control reference area. 0 = none.
         control_axis  pitch|yaw|roll; the ONE steering axis this surface
                       acts on (default pitch; a rudder is yaw, an aileron
                       roll). Only the matching stick (W/S, A/D, Q/E) drives
                       it -- a surface no longer responds to every axis.
         cl_control    the deflection effectiveness (per radian) for this
                       surface. 0 = fall back to the lift-curve slope `cl`
                       (a part that only declares `cl` keeps the old
                       behaviour). Set it to give a part that is BOTH a wing
                       and a control surface a lift-slope and a deflection
                       effectiveness that differ (src/drag.h controlCl).
         max_deflection rad; the surface's travel limit (the deflection is
                       bounded by the player input x this). 0 = none.
   The steering moment about the COM comes from the surface's POSITION
   (Vehicle::applyAeroForce): a tail behind the CG pitches/yaws the ship, a
   canard ahead pitches it the other way. Air gives the authority --
   zero in vacuum. */
    double lift_area;
    double cl;
    double stall_angle;
    double control_area;
    ControlAxis control_axis;
    double cl_control;
    double max_deflection;

    PartDef();

    /* full thrust of one engine: T = (H2 + LOX flow) x ve -- both
       propellants end up in the plume, so the flow is 2 tanks */
    double fullThrust() const { return 2.0 * fuel_rate * exhaust_velocity; }
};

/* Engine-shroud condition, shared by the flight draw (Vehicle::
   hasChildBelow) and the VAB draw: `child` sits on `parent`'s exhaust face
   -- below its centre in the PARENT'S own frame (so it follows a rotated
   parent) AND on the parent's axis. A down-stack edge is exactly axial
   (rel.xy = 0); a surface-attached child lands at the summed radii laterally,
   so it never shrouds no matter where on the wall it is attached. */
inline bool childBelow(double parentRadius,
                       const glm::dvec3 &parentPos,
                       const glm::dmat3 &parentRot,
                       const glm::dvec3 &childPos) {
    const glm::dvec3 rel = glm::transpose(parentRot) * (childPos - parentPos);
    return rel.z < 0.0 && std::hypot(rel.x, rel.y) <= parentRadius * 0.5;
}

/* How a part attaches to its parent (see the header schema comment).
   Down/Up are stack edges (node mating); Surface places the child's surface
   node at a contact point + normal on the parent. */
enum class AttachMode {
    Down,    // stack edge: face-to-face on the parent's -Z face
    Up,      // stack edge: face-to-face on the parent's +Z face (stacking
             // OUTWARD from a surface-attached part, or a nose above the root)
    Surface  // surface edge: child surface node at a parent contact point+normal
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
    int parent;            // part index of the attach parent; -1 = root (part 0)
    AttachMode attach;     // down/up = stack edge (node mating); surface =
                           // surface edge (child surface node at a parent contact)
    double angle;          // stack edge: roll about the mating axis. surface
                           //   edge: consumed into contactNormal (cylinder shorthand).
    double offset;         // m of gap along the attach axis / contact normal
    /* Stack edges (attach down/up) mate two named nodes. Resolved at load: an
       explicit "parentNode"/"childNode" wins, else down defaults to parent
       "bottom" / child "top" and up to parent "top" / child "bottom" (the
       synthesized cylinder faces), so a bare attach:down keeps working. */
    std::string parentNode;
    std::string childNode;   // surface edge: the child's surface node id (empty
                             //   -> the part's surface node)
    /* Surface edges (attach surface): the contact is resolved at load into a
       point + outward normal in the PARENT's local frame -- either from an
       explicit "point"/"normal", or from the "angle"[+"z"] cylinder shorthand
       on the parent's radius. `roll` then spins the child about that normal. */
    glm::dvec3 contactPoint;
    glm::dvec3 contactNormal;
    double roll;
    int stage;             // reserved for staging; 1 = single stage
    std::string from;      // fuel link only: source part id (fuel flows out of)
    std::string to;        // fuel link only: destination part id (fuel flows into)

    bool isFuelLink() const { return def != nullptr && def->fuel_link; }
    /* A stack edge mates two named nodes; a surface edge places the child's
       surface node at a parent contact point. */
    bool isStackEdge() const {
        return attach == AttachMode::Down || attach == AttachMode::Up;
    }
    bool isSurfaceEdge() const { return attach == AttachMode::Surface; }
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

/* ---- VAB build tree (physics-free authoring representation) -------------
   The editor edits THIS, not the flight Vehicle: a mutable tree of part
   instances with their attach edges and solved ship-local poses, no Bullet.
   Launch converts it to a ShipDef/Vehicle via build_ship. Poses come from
   solveEdge (the same solver flight uses), in the root part's frame S. */
struct BuildPart {
    const PartDef *def = nullptr;
    std::string id;
    int parent = -1;              // index into BuildShip::parts; -1 = root
    AttachMode attach = AttachMode::Down;
    std::string parentNode, childNode;          // stack edge node ids
    glm::dvec3 contactPoint, contactNormal;     // surface edge contact (parent-local)
    double angle = 0.0;   // stack edge: roll about the mating axis
    double roll  = 0.0;   // surface edge: roll about the contact normal
    double offset = 0.0;
    int stage = 1;
    glm::dvec3 localPos;  // solved, ship-local frame S (root frame)
    glm::dmat3 localRot;
};

struct BuildShip {
    std::string name;
    std::vector<BuildPart> parts;   // construction order; parts[0] = root

    /* Round-trip extras the tree itself does not edit: fromShipDef keeps
       them, save_ship_def writes them back, toShipDef re-appends them.
       Fuel links are virtual parts (a catalog def for the "part" name +
       the endpoint ids); a link whose endpoint part was deleted is
       dropped at convert/save. controllerId is the explicit controller's
       part id ("" = the default rule); if that part is deleted the
       controller falls back to the default. */
    struct FuelLink { const PartDef *def; std::string id, from, to; };
    std::vector<FuelLink> fuelLinks;
    std::string controllerId;
    double hull_margin = -1.0;

    /* Re-solve every part's localPos/localRot off its parent in construction
       order (root at identity). Call after any add/remove/re-orient. */
    void recomputePoses();

    /* True when part `partIdx`'s stack node `nodeId` is consumed by an
       existing stack edge -- as the parent-side port a child mated to, OR
       as the child-side port mated onto a parent (a mating consumes the
       node on BOTH parts, KSP-style, so the child's mated node is not a
       free port: attaching there would land inside the parent). Surface
       edges never consume a stack port. */
    bool nodeOccupied(int partIdx, const std::string &nodeId) const;

    /* Remove part `idx` AND its whole subtree (a KSP delete takes the
       descendants with it), remap the surviving parent indices and
       re-solve the poses. The root refuses (it is the S-frame anchor);
       invalid indices refuse too. true = the tree changed. */
    bool removePart(int idx);

    /* Detach the subtree at `idx` as a standalone BuildShip (the VAB's
       Subassemblies list -- the non-destructive delete): the detached part
       becomes the new tree's root (its edge cleared -- the graft re-makes
       it), descendants keep theirs, so the assembly re-solves to the same
       relative shape. Fuel links with both endpoints inside MOVE into the
       returned tree; boundary-crossing links are dropped from this one (a
       detached pipe feeds nothing). An explicit controller follows its
       part. The root refuses (returns an empty tree). Both trees come
       back re-solved. */
    BuildShip detachSubtree(int idx);

    /* Append a COPY of `sub` (a subassembly): `root` is the new edge for
       the assembly's root part (its `parent` must be set; the edge fields
       come from the attach solve, like any placement). Every grafted part
       keeps its id when free, else gets a unique "<id>_<n>" suffix (the
       copy-paste case: the same assembly placed twice), descendants keep
       their edges (parents remapped), and sub's fuel links follow with
       their endpoints remapped to the new ids. Returns the grafted root's
       part index; the tree comes back re-solved. */
    size_t graftTree(const BuildShip &sub, const BuildPart &root);

    /* Spin part `idx` about its attach axis by deltaDeg (stack edge: the
       roll about the mating axis; surface edge: the roll about the contact
       normal) and re-solve the subtree poses. The root has no edge:
       no-op. */
    void rotatePart(int idx, double deltaDeg);

    /* The inverse of fromShipDef: a ShipDef build_ship can consume (fuel
       links re-appended after the physical parts, the controller resolved
       from its id). An empty tree converts to an empty def -- callers
       check parts.empty(). */
    ShipDef toShipDef() const;

    /* Copy the physical parts of a loaded ShipDef into a build tree (fuel
       links are virtual and are dropped; parent indices are remapped). */
    static BuildShip fromShipDef(const ShipDef &def);
};

/* Write the build tree as a ship-def JSON file that load_ship_def reads
   back (the same schema: parent by id, explicit node ids, surface
   contacts as point+normal arrays). Round-trip contract: loading a saved
   tree reproduces its ids, edges and poses exactly. false = empty tree
   or the file could not be written. */
bool save_ship_def(const BuildShip &bs, const char *path);

/* The resolved child pose for one attachment (GL-free math; the same
   function the future VAB snap uses). attachPose is purely relative: the
   parent is given in some frame and the child pose comes back in that same
   frame, so callers can work in world coords or in the ship-local frame S
   interchangeably (build_ship feeds it S-frame parent poses; a VAB would
   feed it world poses). */
struct AttachPose {
    glm::dvec3 childPos;
    glm::dmat3 childRot;
};

AttachPose attachPose(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                      const PartDef &parentDef, const PartDef &childDef,
                      AttachMode mode, double angleDeg, double offset);

/* Mate childNode (on the child) onto parentNode (on the already-posed parent):
   the node positions coincide (pushed apart by `offset` along the parent node
   direction) and the node directions are anti-parallel. This is the stack-edge
   solver and the single source of stack-attach geometry -- attachPose(down/up)
   is a shim over it. Purely relative, like attachPose.

   Roll about the mating axis is unconstrained by the two direction vectors
   alone (the KSP "secondaryOrientation" problem); it is resolved here by
   taking the minimal arc that aligns the child node dir, then applying
   `rollDeg` about the mating axis. For the synthesized axial top/bottom nodes
   the minimal arc is the identity, so the child inherits the parent's full
   orientation -- exactly the old attachPose(down/up) behaviour, which is what
   makes the synthesis a drop-in migration. */
AttachPose attachNodes(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                       const Node &parentNode, const Node &childNode,
                       double rollDeg = 0.0, double offset = 0.0);

/* Surface attach (KSP srfAttach): place the child's surface node at a contact
   point on the parent, oriented to the surface normal. This is attachNodes
   with a synthetic parent node built from the contact -- the SAME solver, so
   stack and surface attach never drift. `point` is the contact in the parent's
   local frame and `normal` the outward surface normal there; `childNode` is the
   child's surface node (its dir points inward, toward the parent). `rollDeg`
   spins the child about the normal, `offset` pushes it out along the normal. */
AttachPose attachSurface(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                         const glm::dvec3 &point, const glm::dvec3 &normal,
                         const Node &childNode, double rollDeg = 0.0,
                         double offset = 0.0);

/* A surface edge's authoring data: the contact in the PARENT's local frame
   plus the child's roll about it (the fields a BuildPart/ShipPart stores). */
struct SurfaceEdge {
    glm::dvec3 point;
    glm::dvec3 normal;
    double rollDeg = 0.0;
};

/* The editor's placement snap grids (the VAB's Snap toggles): 10 cm along
   the parent's axis, 10 deg around it and for the part roll. */
static const double kSnapLenM = 0.1;     // m, contact height grid
static const double kSnapAngDeg = 10.0;  // deg, clock angle + roll grid

double snapAngleDeg(double deg);   // round onto the kSnapAngDeg grid

/* The next kSnapAngDeg grid point in the direction of `delta` -- aligned
   even when `cur` is off-grid (fine-tuned with snap off), so snapped
   stepping never strands a value between grid points. */
double gridStepDeg(double cur, double delta);

/* Snap a surface contact in the PARENT's local frame (the VAB's snap
   toggles; either may be off). Distance: the height along the parent's
   long Z to the 10 cm grid (the radius is untouched, so the contact
   stays on a cylindrical side). Angle: the clock angle about Z to the
   10 deg grid -- for the contact point AND the normal, whose azimuths
   are unified on parts (surfaces of revolution, where the true normal
   azimuth equals the contact azimuth) so a hull facet's quantized
   normal cannot cant the part against the snapped position; a normal
   whose azimuth genuinely diverges from the point's (a non-revolution
   surface, e.g. a wing plate) snaps its own azimuth instead, keeping
   its polar tilt. Near the axis (a cap hit) only the height snaps. */
void snapSurfaceContact(glm::dvec3 &point, glm::dvec3 &normal,
                        bool snapLen, bool snapAng);

/* One radial-symmetry copy: the sibling's edge data (what the tree stores)
   + its solved pose (what the ghost draws). */
struct SymClone {
    SurfaceEdge edge;
    AttachPose pose;
};

/* The N-1 radial-symmetry clones of a surface attachment (the VAB's
   symmetry placing): each is the primary placement rotated by k*360/N
   about the parent's own long axis (its local +Z through its position),
   expressed as edge data attachSurface re-solves EXACTLY to that rotated
   pose -- the clone's roll absorbs the minimal-arc holonomy, which is
   zero for purely radial contacts (the booster case) and nonzero on
   tilted ones. Rotating about the parent's own axis keeps every clone's
   contact on the parent's surface, whatever the parent's pose. */
std::vector<SymClone> radialSymmetryClones(const glm::dvec3 &parentPos,
                                           const glm::dmat3 &parentRot,
                                           const Node &childNode,
                                           const glm::dvec3 &point,
                                           const glm::dvec3 &normal,
                                           double rollDeg, double offset,
                                           int n);

/* Solve one parent->child edge into the child's pose in the parent's frame,
   dispatching on the edge kind: a STACK edge (Down/Up) mates the named
   parentNode/childNode via attachNodes (angleDeg is the roll about the mating
   axis); a SURFACE edge places the child's childNode surface node at
   contactPoint/contactNormal via attachSurface (rollDeg about the normal).
   This is the single source of edge geometry -- build_ship and the VAB build
   tree both use it, so the editor and flight can never disagree. Throws if a
   referenced node is missing. */
AttachPose solveEdge(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                     const PartDef &parentDef, const PartDef &childDef,
                     AttachMode mode,
                     const std::string &parentNode, const std::string &childNode,
                     const glm::dvec3 &contactPoint, const glm::dvec3 &contactNormal,
                     double angleDeg, double rollDeg, double offset);

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

/* Parse a ship def from an already-parsed JSON document (the body of
   load_ship_def, split out so the save/load code can build a ShipDef from
   in-memory data -- a loaded ship's parts + hull_margin -- without a file).
   Same contract as load_ship_def: throws std::runtime_error on bad data. */
ShipDef shipDefFromJson(const nlohmann::json &doc, const PartsCatalog &catalog,
                        const std::string &path);

/* List the ship-def slugs in `dir` (e.g. "res/ships") -- the file base names
   with the ".json" extension stripped -- sorted; empty if the directory is
   missing or holds no .json files. Only the .json entries are kept, so the
   VAB's Load picker offers exactly the files the VAB Save writes and --vab
   loads (res/ships/<slug>.json). Header-only (plain POSIX file-system ops),
   so it is unit-testable headless -- the analog of list_saves in save.h. */
inline std::vector<std::string> list_ship_defs(const std::string &dir) {
    std::vector<std::string> names;
    DIR *d = opendir(dir.c_str());
    if(d == nullptr) { return names; }
    struct dirent *e;
    while((e = readdir(d)) != nullptr) {
        const std::string name = e->d_name;
        // strictly longer than ".json" (a bare ".json" would give an empty slug)
        if(name.size() <= 5 || name.compare(name.size() - 5, 5, ".json") != 0) {
            continue;
        }
        names.push_back(name.substr(0, name.size() - 5));
    }
    closedir(d);
    std::sort(names.begin(), names.end());
    return names;
}
