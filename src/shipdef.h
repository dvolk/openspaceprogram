#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <nlohmann/json.hpp>

#include "science.h"   // ExpStorage (PartDef.experiment_storage) + the science helpers

/* Ship/part data model: the JSON-backed description of what a ship is made of.
   GL-free, so the parse/validate path is unit-testable headless. Behavior is
   field-driven (optional-field presence, not the type label). */

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

/* Only what a part HOLDS. Capacity is not per-part state: nothing ever mutates
   it and the save format has no field for it, so it lives on the PartDef and
   Part::capacityOf() reads it (#383). Keeping a copy cost 64 bytes per part once
   these went double (#379) -- a substep's worth of EC on a big battery is ~5e-4
   Wh, under half a float ULP at 12 kWh, so drainEC's `have - share` rounded
   straight back to `have` and the ship never lost charge. The save format is
   already double (save.h), so this removes a narrowing rather than adding one. */
struct ResourceContent {
    double current[(int)ResourceType::Num];

    ResourceContent() {
        for(int i = 0; i < (int)ResourceType::Num; i++) { current[i] = 0; }
    }
};

/* The steering axis a control surface acts on (one only). The moment still comes
   from the surface's OFFSET from the COM. */
enum class ControlAxis { Pitch, Yaw, Roll };

/* Per-axis steering parameters. Pure (no glm) so tests can pin the mapping. */
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
inline const char *controlAxisName(ControlAxis axis) {
    switch(axis) {
        case ControlAxis::Pitch: return "pitch";
        case ControlAxis::Yaw:   return "yaw";
        case ControlAxis::Roll:  return "roll";
    }
    return "?";
}

/* A named attachment node on a part (KSP-style stack node). Two nodes mate when
   their world directions are anti-parallel and their positions coincide. */
struct Node {
    std::string id;
    glm::dvec3 pos;
    glm::dvec3 dir;
    /* true -> the part's surface-attach node: a surface edge places THIS node at
       a contact point on the parent, with `dir` pointing inward. At most one. */
    bool surface = false;
};

/* One part TYPE (a catalog entry; ship defs reference it by name). Behavior comes
   from the optional fields, which combine freely. */
struct PartDef {
    std::string name;
    std::string type;         // free-form label (display only)
    std::string display_name; // human-readable name (display only); empty -> fall back to name
    std::string mesh;     // subpath under res/
    std::string texture;  // subpath under res/
    /* Engine shroud (optional): an open-cylinder wrap drawn over the part when a
       child sits on its exhaust face. Both fields must be set together. */
    std::string shroud;          // shroud mesh subpath under res/
    std::string shroud_texture;  // shroud texture subpath under res/
    double mass;          // kg

    /* Physical size [m]; the .obj is authored to match (origin centered,
       +Z = stack axis). Defaults are the legacy 2 m cube. */
    double radius;
    double height;

    /* Attachment nodes. Empty -> load_parts_catalog synthesizes them from the
       size above. */
    std::vector<Node> nodes;

    const Node *findNode(const std::string &id) const {
        for(size_t i = 0; i < nodes.size(); i++) {
            if(nodes[i].id == id) { return &nodes[i]; }
        }
        return nullptr;
    }

    const Node *findSurfaceNode() const {
        for(size_t i = 0; i < nodes.size(); i++) {
            if(nodes[i].surface) { return &nodes[i]; }
        }
        return nullptr;
    }

    /* Default nodes from the size when `nodes` is empty: top/bottom faces plus a
       side surface node. */
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
    /* Air-breathing modifier on a thruster (drag.h jetThrust): burns jet fuel
       against FREE air. Ignored unless also a thruster. */
    bool jet;
    double jet_fan_thrust;    // N; static (fan) thrust at sea level -- the VTOL floor
    double jet_intake_area;   // m^2; effective intake/capture area (the ram term)
    double rcs_thrust;        // N; > 0 -> RCS translation authority (hydrazine mono).
                              // Rated thrust PER ACTUATOR (per nozzle; for a
                              // nozzle-less legacy pod, per axis).
    /* Nozzle thrust directions in the part's LOCAL frame, unit-normalized
       (src/rcs.h allocator, #428): a nozzle is one-way, and a 5-direction
       box omits its mount face. For a surface-mounted part the mount side is
       the SURFACE NODE's side -- the synthesized srf node points -X, so the
       omitted dir is -X (child +X faces away from the hull). Empty -> legacy
       omnidirectional pod: six directions along the part's own axes. */
    std::vector<glm::dvec3> rcs_nozzles;
    /* The ONE nozzle enumeration (parsed order capped at kMaxRcsNozzles):
       the sim builds its allocator columns in this order and render draws a
       plume per slot off Part::rcsFired, so a plume always lands on a nozzle
       that fired. */
    static constexpr size_t kMaxRcsNozzles = 6;  // == sizeof Part::rcsFired
    size_t rcsNozzleCount() const { return rcs_nozzles.empty() ? kMaxRcsNozzles : rcs_nozzles.size(); }
    glm::dvec3 rcsNozzleDir(size_t i) const {
        assert(i < rcsNozzleCount());
        if(!rcs_nozzles.empty()) { return rcs_nozzles[i]; }
        // Legacy pod: +X,-X,+Y,-Y,+Z,-Z along the part's own axes.
        glm::dvec3 d(0.0);
        d[i / 2] = (i % 2 == 0) ? 1.0 : -1.0;
        return d;
    }
    /* Engine gimbal / TVC (src/gimbal.h): the nozzle may swing +/-gimbal_max
       (rad) at gimbal_rate (rad/s). gimbal_max = 0 -> the thrust line stays
       locked to the part's +Z. */
    double gimbal_max;
    double gimbal_rate;
    /* Electrical (KSP-style EC), independent of each other:
       power_draw (W) > 0           -> draws EC only while ACTIVE (reaction wheel);
       power_draw_constant (W) > 0  -> draws EC ALL THE TIME (capsule life support);
       power_gen (W) > 0            -> a constant EC source (an RTG);
       capacity[EC] > 0             -> a battery. */
    double power_draw;
    double power_draw_constant;
    double power_gen;
    std::vector<float> capacity; // kg per ResourceType; > 0 -> propellant tank

    /* > 0 -> a capsule. The occupant's mass is DERIVED (Part::effectiveMass). */
    int crew_capacity;

    /* > 0 -> a container (Part::isContainer). */
    int inventory_capacity;

    /* The experiment family this part runs; empty -> not a science part. */
    std::string experiment_family;

    /* How this part stores findings: None -> nothing; Instrument -> 1 of its own
       family; Courier -> 1 per family (a kerbal's suit); Container -> unlimited. */
    ExpStorage experiment_storage = ExpStorage::None;

    /* true -> a decoupler: when the stage counter reaches this part's stage, it
       and its child-side subtree are dropped. */
    bool decoupler;

    /* true -> a docking port: an end face that locks to another port, joining two
       ships into one rigid body. A fuel barrier by definition. */
    bool docking_port;

    /* true -> propellant does NOT flow across this part (splits fuel groups);
       decouplers and docking ports are ones. */
    bool fuel_barrier;

    /* true -> a virtual (no-mesh) one-way connection between fuel groups. No
       physics (no body, no mass). */
    bool fuel_link;

    /* Hull margin (m). -1 = not set -> physics default; a ship def's hull_margin
       overrides it (resolveHullMargin). */
    double hull_margin;

    /* Aerodynamics (drag.h). All optional; drag_forward/side/backward override
       the symmetric `drag`. Lift terms: 0 = no lift. */
    double drag;          // the part's drag COEFFICIENT (dimensionless)
    double drag_forward;  // cd with the part's NOSE into the flow. Defaults to `drag`.
    double drag_side;     // cd broadside. Defaults to `drag`.
    double drag_backward; // cd with the part's BASE into the flow. Defaults to `drag`.
    double lift_area;
    double cl;
    double stall_angle;
    double control_area;
    ControlAxis control_axis;
    double cl_control;
    double max_deflection;

    /* Parachute: the deployed canopy's effective Cd·A [m^2]; > 0 -> a chute part.
       The canopy is NOT a collision shape -- its drag is analytic, applied at the
       canopy's anchor (its offset from the COM is the weathervane moment). */
    double chute_area;
    std::string chute_mesh;     // canopy mesh subpath under res/
    std::string chute_texture;  // canopy texture subpath under res/

    /* The canopy's anchor along the part's +Z (the stack axis): half the part off
       its face, then a suspension gap scaling with the canopy's size. SHARED by
       the aero force point and the draw. */
    static double chuteCanopyOffset(const PartDef &d) {
        const double side = std::sqrt(d.chute_area);
        return d.height / 2.0 + 0.75 * side;
    }

    PartDef();

    /* The human-readable name: display_name, else the machine id (catalogs that
       predate the field) (#37). */
    const std::string &label() const {
        return display_name.empty() ? name : display_name;
    }

    /* Total propellant mass flow (kg/s) at full throttle. */
    double totalPropellantRate() const {
        double s = 0.0;
        for(size_t r = 0; r < propellant_rate.size(); r++) { s += propellant_rate[r]; }
        return s;
    }
    /* Full thrust of one engine: T = (total propellant flow) x ve. Jets do
       NOT use this (their thrust is drag.h jetThrust). */
    double fullThrust() const { return totalPropellantRate() * exhaust_velocity; }
    /* The ONE wall rule: what splits a fuel group (see partitionFuelGroups).
       Part::isFuelBarrier, the staging snapshot and the VAB's loop check all go
       through here, so flight and the editor cannot disagree about what a wall
       is (#292). */
    bool isFuelBarrier() const { return fuel_barrier; }
    /* Convenience setter (keeps propellant_rate sized). */
    void setPropellantRate(ResourceType r, double rate) {
        propellant_rate[(int)r] = rate;
    }
};

/* Engine-shroud condition: `child` sits on `parent`'s exhaust face -- below its
   centre in the PARENT'S own frame AND on the parent's axis. */
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
   `parent` must point at an earlier part -- that keeps the parts a tree. A fuel
   link is a virtual part (`from`/`to`). */
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
    bool isStackEdge() const {
        return attach == AttachMode::Down || attach == AttachMode::Up;
    }
    bool isSurfaceEdge() const { return attach == AttachMode::Surface; }
};

struct ShipDef {
    std::string name;
    std::vector<ShipPart> parts;
    /* Defaults so a ShipDef declared before it is filled is never read with
       indeterminate scalars: mintCrewPool and the load path both declare one and
       then assign it, and make_suit -> resolveHullMargin reads hull_margin. */
    int controller = -1;   // part index; -1 = default (first reaction wheel)

    /* Hull margin (m) for EVERY part of this ship; -1 = not set -> the catalog
       value, then the physics default. Ship-level because the welded-hull overlap
       problem depends on the SHIP'S layout. */
    double hull_margin = -1.0;

    int controllerIndex() const {
        if(controller >= 0) { return controller; }
        /* Default: the first part with reaction-wheel authority (the stick
           frame's), so camera basis and controls agree. Falls back to the root. */
        for(size_t i = 0; i < parts.size(); i++) {
            if(parts[i].def && parts[i].def->torque > 0.0) { return (int)i; }
        }
        return 0;
    }
};

/* Fuel groups: the partition of a part tree into the sets that share their
   fuel. `wall[i]` is a wall -- it joins no group and the flood never crosses
   it -- so it keeps -1. `parent` is the tree (-1 = root; any index order, the
   adjacency is built in a full pass first), and `group` comes back sized to
   parent.size(), numbered from 0 in the order the seeds are reached. The group
   count is returned.

   One flood, one wall rule (PartDef::isFuelBarrier), and the consumers MUST
   agree: the live ship (Vehicle::buildFuelGroups, which build_ship's cycle
   check also runs through), the staging snapshot, and the editor's loop check
   (BuildShip::fuelLinkLoop) -- that last is only worth having because it
   predicts what build_ship will do (#292). */
int partitionFuelGroups(const std::vector<int> &parent,
                        const std::vector<bool> &wall, std::vector<int> &group);

/* ---- VAB build tree (physics-free authoring representation) -------------
   The editor edits THIS, not the flight Vehicle; launch converts it via
   build_ship. Poses come from solveEdge, the same solver flight uses. */
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

    /* Round-trip extras the tree itself does not edit: fuel links and the
       explicit controller's part id. */
    struct FuelLink { const PartDef *def; std::string id, from, to; };
    std::vector<FuelLink> fuelLinks;
    std::string controllerId;
    double hull_margin = -1.0;

    /* Re-solve every part's localPos/localRot off its parent in construction
       order (root at identity). Call after any add/remove/re-orient. */
    void recomputePoses();

    /* True when part `partIdx`'s stack node `nodeId` is consumed by an existing
       stack edge (a mating consumes it on BOTH parts). */
    bool nodeOccupied(int partIdx, const std::string &nodeId) const;

    /* Remove part `idx` AND its whole subtree, remap the surviving parent
       indices and re-solve the poses. The root refuses. */
    bool removePart(int idx);

    /* Detach the subtree at `idx` as a standalone BuildShip. Fuel links with both
       endpoints inside MOVE; boundary-crossing links are dropped. The root
       refuses (returns empty). */
    BuildShip detachSubtree(int idx);

    /* Append a COPY of `sub`; `root` is the new edge for the assembly's root
       part. Returns the grafted root's part index. */
    size_t graftTree(const BuildShip &sub, const BuildPart &root);

    /* Spin part `idx` about its attach axis by deltaDeg and re-solve the
       subtree poses. The root has no edge: no-op. */
    void rotatePart(int idx, double deltaDeg);

    /* The drain model needs a DAG, so build_ship throws on a fuel-link loop
       (vehicle.cpp). This is the same test over the build tree -- the loop as a
       chain of part ids, or "" when the links are acyclic. `extra` is a link
       that is NOT yet in fuelLinks, so the editor can ask before adding one:
       a throw at launch is a toast the player cannot undo, while a refusal in
       the editor is just a click (#291). Endpoints are grouped the way
       buildFuelGroups groups them, so the ids name the loop's ends, not every
       hop (parts inside one group share their fuel anyway). */
    std::string fuelLinkLoop(const FuelLink *extra = nullptr) const;

    /* The inverse of fromShipDef: a ShipDef build_ship can consume. */
    ShipDef toShipDef() const;

    /* Copy the physical parts of a loaded ShipDef into a build tree (fuel
       links are dropped; parent indices are remapped). */
    static BuildShip fromShipDef(const ShipDef &def);
};

/* Write the build tree as a ship-def JSON file. Round-trip contract: loading a
   saved tree reproduces its ids, edges and poses exactly. */
bool save_ship_def(const BuildShip &bs, const char *path);

/* The resolved child pose for one attachment. Purely relative: parent in some
   frame, child back in that same frame. */
struct AttachPose {
    glm::dvec3 childPos;
    glm::dmat3 childRot;
};

AttachPose attachPose(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                      const PartDef &parentDef, const PartDef &childDef,
                      AttachMode mode, double angleDeg, double offset);

/* Mate childNode onto parentNode: positions coincide (pushed apart by `offset`)
   and directions are anti-parallel. Roll about the mating axis is unconstrained
   by the two directions alone; it is resolved by the minimal arc aligning the
   child node dir, then `rollDeg`. */
AttachPose attachNodes(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                       const Node &parentNode, const Node &childNode,
                       double rollDeg = 0.0, double offset = 0.0);

/* Surface attach (KSP srfAttach): the child's surface node at a contact point on
   the parent, its node dir opposing the contact normal. POSITION is node mating
   exactly as for a stack port (a synthetic parent node at the contact), so stack
   and surface attach never drift. ROLL is the difference: with no authored port
   to reference it is pinned to the stack axes -- the child's +Z parallel to the
   parent's +Z projected into the contact plane -- not the shortest arc. */
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

/* The editor's placement snap grids. */
static const double kSnapLenM = 0.1;     // m, contact height grid
static const double kSnapAngDeg = 10.0;  // deg, clock angle + roll grid

double snapAngleDeg(double deg);   // round onto the kSnapAngDeg grid

/* The next kSnapAngDeg grid point in the direction of `delta` -- aligned
   even when `cur` is off-grid. */
double gridStepDeg(double cur, double delta);

/* Snap a surface contact in the PARENT's local frame: height along Z and clock
   angle about Z, for the contact point AND the normal -- their azimuths unified,
   so a hull facet's quantized normal cannot cant the part. */
void snapSurfaceContact(glm::dvec3 &point, glm::dvec3 &normal,
                        bool snapLen, bool snapAng);

/* One radial-symmetry copy: the sibling's edge data + its solved pose. */
struct SymClone {
    SurfaceEdge edge;
    AttachPose pose;
};

/* The N-1 radial-symmetry clones: the primary placement rotated by k*360/N about
   the parent's long axis, carrying the SAME roll -- the surface frame is
   referenced to that axis, so the rotated contact re-solves to the rotated pose. */
std::vector<SymClone> radialSymmetryClones(const glm::dvec3 &parentPos,
                                           const glm::dmat3 &parentRot,
                                           const Node &childNode,
                                           const glm::dvec3 &point,
                                           const glm::dvec3 &normal,
                                           double rollDeg, double offset,
                                           int n);

/* Solve one parent->child edge into the child's pose in the parent's frame
   (stack: attachNodes; surface: attachSurface). Throws if a node is missing. */
AttachPose solveEdge(const glm::dvec3 &parentPos, const glm::dmat3 &parentRot,
                     const PartDef &parentDef, const PartDef &childDef,
                     AttachMode mode,
                     const std::string &parentNode, const std::string &childNode,
                     const glm::dvec3 &contactPoint, const glm::dvec3 &contactNormal,
                     double angleDeg, double rollDeg, double offset);

/* Hull margin resolution: ship def wins over part catalog; either may be unset
   (-1); both unset -> physics default. */
double resolveHullMargin(double shipMargin, double partMargin);

struct PartsCatalog {
    std::vector<PartDef> parts;

    const PartDef *find(const std::string &name) const;
};

/* Parse + validate: throws std::runtime_error naming the file and offending
   field. The catalog must outlive any ShipDef (the parts point into it). */
PartsCatalog load_parts_catalog(const char *path);
ShipDef load_ship_def(const char *path, const PartsCatalog &catalog);

/* Parse a ship def from an already-parsed JSON document. Same contract as
   load_ship_def. */
ShipDef shipDefFromJson(const nlohmann::json &doc, const PartsCatalog &catalog,
                        const std::string &path);

/* A def whose ROOT part is kerbal-typed describes a PERSON, and a person is
   exactly one part (part.h). True when `def` is consistent with that: no
   kerbal root, or exactly one part. The spawn path enforces it so the game
   can never write a save its loader then refuses -- a Kerbal minted from a
   multi-part def loses its extra parts on the way out (the writer reads a
   walker's part 0 only), and Game::kerbalBoard asserts one part the moment
   anyone boards. Each site keeps its own message: the e2e cases pin them. */
bool kerbalDefConsistent(const ShipDef &def);

/* One vehicle the game spawns at boot. The first four fields are required;
   `site` is an optional pad id, meaningful only for an on_pad scenario. */
struct DebugStartShip {
    std::string ship;      // ship def path
    std::string name;      // display name
    std::string body;      // the body it sits on (a name in the system)
    std::string scenario;  // the spawn scenario (a --startship field)
    std::string site;      // pad id ("" = the body's default site)
};

// The parsed "ships" array.
struct DebugStartShips {
    std::vector<DebugStartShip> ships;
};

// Parse + validate the debug-start-ships JSON; throws std::runtime_error naming
// the file + the offending entry.
DebugStartShips loadDebugStartShips(const char *path);

/* The ship-def slugs in `dir` (file base names with ".json" stripped), sorted. */
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

/* True if the ship-def file is marked "testship": true -- an e2e/scenario ship
   the VAB Load picker hides unless --dev-vehicles. Fail-open on bad data. */
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

/* The dev ship-def tree: Game Debug's dev-vehicles switch (game.h) lists it in
   the VAB picker and vabLoad resolves bare names from it. cwd-relative on
   purpose, like kFixtureRoot (save.h) -- it is the repo, not installed content,
   so a packaged build simply has nothing there. */
inline const std::string kFixtureVehicleRoot = "e2e/vehicles";

/* The VAB Load picker's list for `dir`: list_ship_defs, minus the testships
   unless the caller asks for them (Game::dev_vehicles). */
inline std::vector<std::string> list_vab_vehicle_defs(const std::string &dir,
                                                      bool include_testships) {
    std::vector<std::string> names;
    for(const std::string &s : list_ship_defs(dir)) {
        if(!include_testships && shipDefIsTestship(dir + "/" + s + ".json")) {
            continue;
        }
        names.push_back(s);
    }
    return names;
}
