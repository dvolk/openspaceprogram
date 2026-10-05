#pragma once

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <nlohmann/json.hpp>

#include "science.h"   // ExpStorage (PartDef.experiment_storage) + the science helpers

/* Ship/part data model: the JSON-backed description of what a ship is made
   of. GL-free (no rendering, no Bullet) so the parse/validate path can be
   unit-tested headless. Behavior is field-driven (presence of optional
   fields, not the type label). See load_parts_catalog / shipDefFromJson for
   the schema and res/data/parts.json + res/ships JSON files for examples. */

enum class ResourceType {
    Hydrogen,
    LOX,
    EC,
    Oxygen,
    Water,
    Food,
    Hydrazine,   // monopropellant (the EVA kerbal's RCS suit)
    JetFuel,     // the onboard fuel of an air-breathing jet
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

/* The steering axis a control surface acts on. One or the other, like a
   real control surface: an elevator pitches, a rudder yaws, an aileron
   rolls. The moment still comes from the surface's OFFSET from the COM. */
enum class ControlAxis { Pitch, Yaw, Roll };

/* The per-axis steering parameters for a control surface. Pure (no glm) so
   tests/ can pin the mapping without Bullet/GL. */
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
/* The axis's display label (the --drag-log control-surface telemetry). */
inline const char *controlAxisName(ControlAxis axis) {
    switch(axis) {
        case ControlAxis::Pitch: return "pitch";
        case ControlAxis::Yaw:   return "yaw";
        case ControlAxis::Roll:  return "roll";
    }
    return "?";
}

/* A named attachment node on a part (KSP-style stack node). Two nodes mate
   when their world directions are anti-parallel and their positions
   coincide -- see attachNodes(). */
struct Node {
    std::string id;
    glm::dvec3 pos;
    glm::dvec3 dir;
    /* true -> the part's surface-attach node: a surface edge places THIS
       node at a contact point on the parent, with `dir` pointing inward.
       A part has at most one. */
    bool surface = false;
};

/* One part TYPE (a catalog entry; ship defs reference it by name).
   Behavior comes from the optional fields: torque makes it a reaction
   wheel, propellant + exhaust_velocity make it a thruster, capacity makes
   it a propellant tank. They combine freely. */
struct PartDef {
    std::string name;
    std::string type;         // free-form label (display only)
    std::string display_name; // human-readable name (display only); empty -> fall back to name
    std::string mesh;     // subpath under res/
    std::string texture;  // subpath under res/
    /* Engine shroud (optional): an open-cylinder wrap drawn OVER the part
       when a child is attached on its exhaust face. Both fields must be
       set together. */
    std::string shroud;          // shroud mesh subpath under res/
    std::string shroud_texture;  // shroud texture subpath under res/
    double mass;          // kg

    /* Physical size in metres; the .obj is authored to match (origin
       centered, +Z = stack axis). Defaults are the legacy 2 m cube. */
    double radius;
    double height;

    /* Attachment nodes. Empty -> load_parts_catalog synthesizes `top`/`bottom`
       from the size above (so an axis-aligned cylinder needs no authoring). */
    std::vector<Node> nodes;

    /* Look up a node by id; nullptr if absent. */
    const Node *findNode(const std::string &id) const {
        for(size_t i = 0; i < nodes.size(); i++) {
            if(nodes[i].id == id) { return &nodes[i]; }
        }
        return nullptr;
    }

    /* The part's surface-attach node (surface == true); nullptr if none. */
    const Node *findSurfaceNode() const {
        for(size_t i = 0; i < nodes.size(); i++) {
            if(nodes[i].surface) { return &nodes[i]; }
        }
        return nullptr;
    }

    /* Fill the default nodes from the size when `nodes` is empty (top/bottom
       axial faces + a side surface node). A part with explicit nodes is left
       alone. */
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
    std::vector<double> propellant_rate; // kg/s per ResourceType at full throttle;
                              // all-zero -> not a thruster
    double exhaust_velocity;  // m/s; with propellant_rate -> thruster. For a JET
                              // this is the REAL exhaust velocity (drag.h jetThrust).
    /* Jet engine (air-breathing) modifier on a thruster (drag.h jetThrust):
       burns jet fuel against FREE air. Ignored unless also a thruster. */
    bool jet;
    double jet_fan_thrust;    // N; static (fan) thrust at sea level -- the VTOL floor
    double jet_intake_area;   // m^2; effective intake/capture area (the ram term)
    double rcs_thrust;        // N; > 0 -> RCS translation authority (hydrazine mono)
    /* Electrical (KSP-style EC), independent of each other:
       power_draw (W) > 0           -> draws EC only while ACTIVE (a reaction wheel);
       power_draw_constant (W) > 0  -> draws EC ALL THE TIME (capsule life support);
       power_gen (W) > 0            -> a constant EC source (an RTG).
       A battery is EC STORAGE (capacity[EC] > 0). */
    double power_draw;
    double power_draw_constant;
    double power_gen;
    std::vector<float> capacity; // kg per ResourceType; > 0 -> propellant tank

    /* Crew capacity: how many EVA characters this part can hold. > 0 -> a
       capsule. The occupant's mass is DERIVED (Part::effectiveMass). */
    int crew_capacity;

    /* Inventory capacity: how many inventory items this part can hold.
       > 0 -> a container (Part::isContainer). */
    int inventory_capacity;

    /* The experiment family this part runs (a science instrument). Empty ->
       not a science part. */
    std::string experiment_family;

    /* How this part stores science findings (Part::experiments). None ->
       holds nothing; Instrument -> 1 of its own family; Courier -> 1 per
       family (a kerbal's suit); Container -> unlimited per family. */
    ExpStorage experiment_storage = ExpStorage::None;

    /* true -> a decoupler: a staging boundary. When the stage counter
       reaches this part's stage, the decoupler + its child-side subtree
       are dropped (like a KSP separator). */
    bool decoupler;

    /* true -> a docking port: an end face that can lock to another port,
       joining the two ships into one rigid body. It is a fuel barrier by
       definition (a boundary between two ships' fuel systems). */
    bool docking_port;

    /* true -> a fuel barrier: propellant does NOT flow across this part
       (splits fuel groups). Decouplers and docking ports are fuel barriers. */
    bool fuel_barrier;

    /* true -> a fuel link: a virtual (no-mesh) one-way connection between
       fuel groups. No physics (no body, no mass). See ShipPart's from/to. */
    bool fuel_link;

    /* Collision convex-hull margin (m). -1 = not set -> physics default.
       A ship def's hull_margin overrides this when set (resolveHullMargin). */
    double hull_margin;

    /* Aerodynamics (src/drag.h). All optional. `drag` is the SYMMETRIC
       default coefficient; drag_forward/side/backward override it per
       orientation. Lift terms: 0 = no lift. */
    double drag;          // the part's drag COEFFICIENT (dimensionless)
    double drag_forward;  // cd with the part's NOSE into the flow. Defaults to `drag`.
    double drag_side;     // cd broadside. Defaults to `drag`.
    double drag_backward; // cd with the part's BASE into the flow. Defaults to `drag`.
    // applyAeroForce blends these three by the axis-flow angle (drag.h partCd).
    double lift_area;
    double cl;
    double stall_angle;
    double control_area;
    ControlAxis control_axis;
    double cl_control;
    double max_deflection;

    PartDef();

    /* Total propellant mass flow (kg/s) at full throttle. */
    double totalPropellantRate() const {
        double s = 0.0;
        for(size_t r = 0; r < propellant_rate.size(); r++) { s += propellant_rate[r]; }
        return s;
    }
    /* Full thrust of one engine: T = (total propellant flow) x ve. Jets do
       NOT use this (their thrust is drag.h jetThrust). */
    double fullThrust() const { return totalPropellantRate() * exhaust_velocity; }
    /* Convenience setter (keeps propellant_rate sized). */
    void setPropellantRate(ResourceType r, double rate) {
        propellant_rate[(int)r] = rate;
    }
};

/* Engine-shroud condition, shared by the flight draw (Vehicle::
   hasChildBelow) and the VAB draw: `child` sits on `parent`'s exhaust face
   -- below its centre in the PARENT'S own frame AND on the parent's axis. */
inline bool childBelow(double parentRadius,
                       const glm::dvec3 &parentPos,
                       const glm::dmat3 &parentRot,
                       const glm::dvec3 &childPos) {
    const glm::dvec3 rel = glm::transpose(parentRot) * (childPos - parentPos);
    return rel.z < 0.0 && std::hypot(rel.x, rel.y) <= parentRadius * 0.5;
}

/* How a part attaches to its parent. Down/Up are stack edges (node
   mating); Surface places the child's surface node at a contact point. */
enum class AttachMode {
    Down,    // stack edge: face-to-face on the parent's -Z face
    Up,      // stack edge: face-to-face on the parent's +Z face
    Surface  // surface edge: child surface node at a parent contact point+normal
};

/* One part INSTANCE in a ship def, in construction order (index 0 = root).
   `parent` must point at an earlier part (that rule keeps the parts a tree).
   A fuel link is a virtual part: `from`/`to` name the two parts whose
   groups it connects. */
struct ShipPart {
    std::string part;      // catalog name
    std::string id;        // instance id (explicit, or auto "<name>_<n>")
    const PartDef *def;    // resolved at load time (points into the catalog)
    int parent;            // part index of the attach parent; -1 = root
    AttachMode attach;
    double angle;          // stack edge: roll about the mating axis. surface
                           //   edge: consumed into contactNormal (cylinder shorthand).
    double offset;         // m of gap along the attach axis / contact normal
    /* Stack edges mate two named nodes (defaults: parent "bottom"/"top" and
       child "top"/"bottom" for down/up). */
    std::string parentNode;
    std::string childNode;   // surface edge: the child's surface node id
    /* Surface edges: contact point + outward normal in the PARENT's local
       frame (explicit "point"/"normal" or the "angle"[+"z"] cylinder shorthand). */
    glm::dvec3 contactPoint;
    glm::dvec3 contactNormal;
    double roll;
    int stage;             // reserved for staging; 1 = single stage
    std::string from;      // fuel link only: source part id
    std::string to;        // fuel link only: destination part id

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
       physics default. Ship-level because the welded-hull overlap problem
       depends on the SHIP'S layout. */
    double hull_margin;

    int controllerIndex() const {
        if(controller >= 0) { return controller; }
        /* default: the first part with reaction-wheel authority (the same
           part the stick frame uses), so the camera basis and the controls
           agree. Falls back to the root. */
        for(size_t i = 0; i < parts.size(); i++) {
            if(parts[i].def && parts[i].def->torque > 0.0) { return (int)i; }
        }
        return 0;
    }
};

/* ---- VAB build tree (physics-free authoring representation) -------------
   The editor edits THIS, not the flight Vehicle. Launch converts it to a
   ShipDef/Vehicle via build_ship. Poses come from solveEdge (the same
   solver flight uses). */
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

    /* Round-trip extras the tree itself does not edit: fuel links (virtual
       parts with endpoint ids) and the explicit controller's part id. */
    struct FuelLink { const PartDef *def; std::string id, from, to; };
    std::vector<FuelLink> fuelLinks;
    std::string controllerId;
    double hull_margin = -1.0;

    /* Re-solve every part's localPos/localRot off its parent in construction
       order (root at identity). Call after any add/remove/re-orient. */
    void recomputePoses();

    /* True when part `partIdx`'s stack node `nodeId` is consumed by an
       existing stack edge (a mating consumes the node on BOTH parts). */
    bool nodeOccupied(int partIdx, const std::string &nodeId) const;

    /* Remove part `idx` AND its whole subtree, remap the surviving parent
       indices and re-solve the poses. The root refuses. */
    bool removePart(int idx);

    /* Detach the subtree at `idx` as a standalone BuildShip (the VAB's
       Subassemblies list). Fuel links with both endpoints inside MOVE;
       boundary-crossing links are dropped. The root refuses (returns empty). */
    BuildShip detachSubtree(int idx);

    /* Append a COPY of `sub` (a subassembly). `root` is the new edge for the
       assembly's root part. Returns the grafted root's part index. */
    size_t graftTree(const BuildShip &sub, const BuildPart &root);

    /* Spin part `idx` about its attach axis by deltaDeg and re-solve the
       subtree poses. The root has no edge: no-op. */
    void rotatePart(int idx, double deltaDeg);

    /* The inverse of fromShipDef: a ShipDef build_ship can consume. */
    ShipDef toShipDef() const;

    /* Copy the physical parts of a loaded ShipDef into a build tree (fuel
       links are dropped; parent indices are remapped). */
    static BuildShip fromShipDef(const ShipDef &def);
};

/* Write the build tree as a ship-def JSON file that load_ship_def reads
   back. Round-trip contract: loading a saved tree reproduces its ids, edges
   and poses exactly. */
bool save_ship_def(const BuildShip &bs, const char *path);

/* The resolved child pose for one attachment (GL-free math). Purely
   relative: the parent is given in some frame and the child pose comes back
   in that same frame. */
struct AttachPose {
    glm::dvec3 childPos;
    glm::dmat3 childRot;
};

AttachPose attachPose(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                      const PartDef &parentDef, const PartDef &childDef,
                      AttachMode mode, double angleDeg, double offset);

/* Mate childNode onto parentNode: node positions coincide (pushed apart by
   `offset`) and node directions are anti-parallel. Roll about the mating
   axis is unconstrained by the two direction vectors alone (the KSP
   "secondaryOrientation" problem); it is resolved by the minimal arc that
   aligns the child node dir, then `rollDeg` about the mating axis. */
AttachPose attachNodes(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                       const Node &parentNode, const Node &childNode,
                       double rollDeg = 0.0, double offset = 0.0);

/* Surface attach (KSP srfAttach): place the child's surface node at a
   contact point on the parent, its node dir opposing the contact normal.
   The POSITION is node mating exactly as for a stack port (a synthetic
   parent node at the contact), so stack and surface attach never drift.
   The ROLL is the one difference: a stack port's roll is authored, while a
   surface contact has no authored port to reference, so its roll is pinned
   to the stack axes -- the child's +Z stays parallel to the parent's +Z
   projected into the contact plane -- instead of left to the shortest arc.
   See surfaceMatingRot in the .cpp for why the shortest arc is not enough. */
AttachPose attachSurface(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                         const glm::dvec3 &point, const glm::dvec3 &normal,
                         const Node &childNode, double rollDeg = 0.0,
                         double offset = 0.0);

/* A surface edge's authoring data: the contact in the PARENT's local frame
   plus the child's roll about it. */
struct SurfaceEdge {
    glm::dvec3 point;
    glm::dvec3 normal;
    double rollDeg = 0.0;
};

/* The editor's placement snap grids (the VAB's Snap toggles). */
static const double kSnapLenM = 0.1;     // m, contact height grid
static const double kSnapAngDeg = 10.0;  // deg, clock angle + roll grid

double snapAngleDeg(double deg);   // round onto the kSnapAngDeg grid

/* The next kSnapAngDeg grid point in the direction of `delta` -- aligned
   even when `cur` is off-grid. */
double gridStepDeg(double cur, double delta);

/* Snap a surface contact in the PARENT's local frame. Distance: the height
   along Z to the 10 cm grid. Angle: the clock angle about Z to the 10 deg
   grid -- for the contact point AND the normal, whose azimuths are unified
   on parts (surfaces of revolution) so a hull facet's quantized normal
   cannot cant the part against the snapped position. */
void snapSurfaceContact(glm::dvec3 &point, glm::dvec3 &normal,
                        bool snapLen, bool snapAng);

/* One radial-symmetry copy: the sibling's edge data + its solved pose. */
struct SymClone {
    SurfaceEdge edge;
    AttachPose pose;
};

/* The N-1 radial-symmetry clones of a surface attachment: each is the
   primary placement rotated by k*360/N about the parent's own long axis,
   carrying the SAME roll -- the surface frame is referenced to that axis,
   so re-solving the rotated contact lands on the rotated pose by
   construction (no holonomy to absorb). */
std::vector<SymClone> radialSymmetryClones(const glm::dvec3 &parentPos,
                                           const glm::dmat3 &parentRot,
                                           const Node &childNode,
                                           const glm::dvec3 &point,
                                           const glm::dvec3 &normal,
                                           double rollDeg, double offset,
                                           int n);

/* Solve one parent->child edge into the child's pose in the parent's frame,
   dispatching on the edge kind (stack: attachNodes; surface: attachSurface).
   Single source of edge geometry -- build_ship and the VAB both use it.
   Throws if a referenced node is missing. */
AttachPose solveEdge(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                     const PartDef &parentDef, const PartDef &childDef,
                     AttachMode mode,
                     const std::string &parentNode, const std::string &childNode,
                     const glm::dvec3 &contactPoint, const glm::dvec3 &contactNormal,
                     double angleDeg, double rollDeg, double offset);

/* Collision hull margin (m) resolution: ship def wins over part catalog;
   either may be unset (-1); both unset -> -1 (physics default applies). */
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
   in-memory data). Same contract as load_ship_def. */
ShipDef shipDefFromJson(const nlohmann::json &doc, const PartsCatalog &catalog,
                        const std::string &path);

/* List the ship-def slugs in `dir` (file base names with ".json" stripped),
   sorted. Header-only (std::filesystem ops). */
inline std::vector<std::string> list_ship_defs(const std::string &dir) {
    std::vector<std::string> names;
    namespace fs = std::filesystem;
    std::error_code ec;
    auto it = fs::directory_iterator(dir, ec);
    if(ec) { return names; }   // dir missing or not a directory
    for(const auto &entry : it) {
        const fs::path &p = entry.path();
        if(p.extension() != ".json") { continue; }   // a bare ".json" has no extension
        names.push_back(p.stem().string());
    }
    std::sort(names.begin(), names.end());
    return names;
}

/* True if the ship-def file at `path` is marked "testship": true -- an
   e2e/scenario ship the VAB Load picker hides. Fail-open on bad data. */
inline bool shipDefIsTestship(const std::string &path) {
    std::ifstream f(path);
    if(!f.is_open()) { return false; }
    nlohmann::json doc;
    try { doc = nlohmann::json::parse(f); }
    catch(const std::exception &) { return false; }
    if(!doc.is_object() || !doc.contains("testship") || !doc["testship"].is_boolean()) {
        return false;
    }
    return doc["testship"].get<bool>();
}

/* The VAB Load picker's list for `dir`: list_ship_defs minus the testships. */
inline std::vector<std::string> list_vab_ship_defs(const std::string &dir) {
    std::vector<std::string> names;
    for(const std::string &s : list_ship_defs(dir)) {
        if(shipDefIsTestship(dir + "/" + s + ".json")) { continue; }
        names.push_back(s);
    }
    return names;
}
