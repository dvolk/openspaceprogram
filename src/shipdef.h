#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <nlohmann/json.hpp>

#include "equirect.h"  // kNoCoord (DebugStartShip's optional lat/lon pair)
#include "science.h"   // ExpStorage (PartDef.experiment_storage) + the science helpers

/* Ship/part data model: the JSON-backed description of what a ship is made of.
   GL-free, so the parse/validate path is unit-testable headless. Behavior is
   field-driven (optional-field presence, not the type label). */

/* The largest stage number a ship file or a save may ask for. Not a physics limit,
   and since #517 not a cost limit either: computeStaging walks the stage NUMBERS the
   ship carries, so a lone big number buys no work. It is an authoring bound because a
   number outside the ship's own sequence rewrites how flight runs: the stage counter
   starts at the highest stage any part carries (vehicle.cpp:1650-1658), so an outlier
   becomes the top of the sequence -- its decoupler fires on the first stage press, the
   stages the ship was built with sit thousands of presses away, and producedThrust()
   cuts
   every engine below the counter (vehicle.cpp:1985) until that counter grinds down to
   them. The VAB stage box clamps to this so the editor cannot create what the loaders
   refuse. */
inline constexpr int kMaxStage = 10000;

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

/* What each resource IS called, and whether it weighs anything -- one row per
   ResourceType (#334). Four vocabularies survive on purpose -- a window's bar
   says "Electric charge", --fuel-log says "EC", a quantity needs a unit, and a
   parts file keys it as "ec" -- but they were four edits across three files, and
   adding a ninth type meant half-remembering them. The asserts below are what
   make the table safe to edit: row count, enum order and distinct file keys are
   all compile-time facts, so a row that is missing, inserted in the wrong place,
   or sharing a key stops the build instead of quietly mislabelling every drum.
   'fileKey' is the spelling res/data/parts.json uses (the only file that carries
   these words; a save stores fuel as a positional array -- see save.h), so
   changing one is a parts-catalog edit that the catalog's own loader reports.
   hasMass is the other one-fact-per-type column: EC is stored energy, so it
   carries no weight, and every mass sum now asks the table instead of naming EC. */
struct ResourceInfo {
    ResourceType type;      // which enum value this row describes
    const char *label;      // what a window's row says
    const char *unit;       // what a quantity of it is measured in
    const char *shortName;  // what a log line says
    const char *fileKey;    // what res/data/parts.json calls it
    bool hasMass;
};
inline constexpr ResourceInfo kResources[] = {
    { ResourceType::Hydrogen,  "Hydrogen",        "kg", "H2",   "hydrogen",  true  },
    { ResourceType::LOX,       "LOX",             "kg", "LOX",  "lox",       true  },
    { ResourceType::EC,        "Electric charge", "Wh", "EC",   "ec",        false },
    { ResourceType::Oxygen,    "Oxygen",          "kg", "O2",   "oxygen",    true  },
    { ResourceType::Water,     "Water",           "kg", "H2O",  "water",     true  },
    { ResourceType::Food,      "Food",            "kg", "food", "food",      true  },
    { ResourceType::Hydrazine, "Hydrazine",       "kg", "N2H4", "hydrazine", true  },
    { ResourceType::JetFuel,   "Jet fuel",        "kg", "JF",   "jetfuel",   true  },
};
static_assert(sizeof(kResources) / sizeof(kResources[0])
              == (int)ResourceType::Num, "one spelling row per ResourceType");

/* Counting rows is not enough: a row added in the wrong place shifts every name
   AND the mass rule one type over (LOX would become massless), and two rows
   sharing a file key make one type unparsable -- resource_index_from_string
   takes the first match. Both are cheap to refuse. */
static_assert([] {
    for(int i = 0; i < (int)ResourceType::Num; i++) {
        if(kResources[i].type != (ResourceType)i) { return false; }
        for(int j = i + 1; j < (int)ResourceType::Num; j++) {
            if(std::string_view(kResources[i].fileKey)
               == std::string_view(kResources[j].fileKey)) { return false; }
        }
    }
    return true;
}(), "kResources rows must follow enum order, with a distinct fileKey each");

/* Bounds-checked on purpose: Vehicle::resourceName used to answer "?" for a type
   it did not know, and indexing off the end of the table is worse than a typo'd
   label. */
inline const ResourceInfo &resourceInfo(ResourceType t) {
    const int i = (int)t;
    assert(i >= 0 && i < (int)ResourceType::Num);
    return kResources[i];
}
inline const char *resourceLabel(ResourceType t) { return resourceInfo(t).label; }
inline const char *resourceUnit(ResourceType t) { return resourceInfo(t).unit; }
inline const char *resourceShort(ResourceType t) { return resourceInfo(t).shortName; }
inline const char *resourceFileKey(ResourceType t) { return resourceInfo(t).fileKey; }
inline bool resourceHasMass(ResourceType t) { return resourceInfo(t).hasMass; }

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

/* Per-axis steering parameters. Pure (no glm) so tests can pin the mapping.
   aboutAxis is a SHIP axis only, and since #426 it is NOT a promise about which key
   turns the hull: the solver spends a surface on whatever its rig levers. What this
   axis decides is the COMMAND SENSE -- controlDeflectionSign measures the surface's
   leverage about it, and a surface that levers nothing about its own authored axis is
   uncommandable and buys no authority either (#495). There used to be a stickIndex
   beside it, wiring one key to one part; that wire is what #426 removed, so the field
   went with it rather than waiting to be re-wired into a second actuator path.
   The force itself follows the surface's own normal and its mesh hinges on its own
   span (#453), so there is no per-axis force direction or hinge axis here. */
struct ControlAxisParams {
    int aboutAxis = -1;
    double targetSign = -1.0;
};
inline ControlAxisParams controlAxisParams(ControlAxis axis) {
    ControlAxisParams p;
    switch(axis) {
        case ControlAxis::Pitch: p.aboutAxis = 0; p.targetSign = -1.0; break;
        case ControlAxis::Yaw:   p.aboutAxis = 1; p.targetSign = -1.0; break;
        case ControlAxis::Roll:  p.aboutAxis = 2; p.targetSign = +1.0; break;
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
       +Z = stack axis). Defaults are the legacy 2 m cube.
       These describe a CYLINDER, which is what the stack ports are minted from.
       For a plate they are not its shape: the wing's radius 2.0 is half its 4 m
       SPAN while the plate is 0.1 m thick, so `radius` says where a bolt sits
       and says nothing about where the surface is (#457). */
    double radius;
    double height;

    /* The mesh's OWN bounding box, in the part's local frame (+Z = the stack
       axis), read off the mesh file when the catalog is parsed. Anything that
       asks where this part's surface is, or how wide it actually is, reads this
       rather than the cylinder above.
       Derived from the .obj's `v` lines: the same corners geom_fingerprint hashes
       (#1182) and the same vertices mesh.cpp hands Bullet to convex-hull (physics.cpp
       BuildHull), whose aiProcess flags transform nothing, so this IS the hull's
       box before Bullet grows it by the shape's collision margin. Missing mesh
       or no vertices -> has_mesh_extents false, and the helpers below fall back
       to the authored size rather than reporting a zero-size part.
       The box is symmetrised about the part origin, because the part origin IS
       the stack axis: a mesh modelled off-centre would get the larger half-span
       on both sides, exactly the lie the authored radius tells. Every mesh in
       this catalog has its box centred on the axis to <1e-6 m, pinned in
       tests/test_shipload.cpp, so nothing shipped depends on that yet. */
    glm::dvec3 mesh_min = glm::dvec3(0.0);
    glm::dvec3 mesh_max = glm::dvec3(0.0);
    bool has_mesh_extents = false;

    /* Half-extent of the mesh across the stack axis, per side. A surface attach
       puts its contact on the ELLIPSE these two bound: identical to the authored
       cylinder when the parent is round, and honest at a plate's tip AND its
       face instead of half a span out in the air. */
    void meshHalfSpan(double &hx, double &hy) const {
        if(!has_mesh_extents) { hx = radius; hy = radius; return; }
        hx = std::max(std::fabs(mesh_min.x), std::fabs(mesh_max.x));
        hy = std::max(std::fabs(mesh_min.y), std::fabs(mesh_max.y));
    }

    /* Half the widest extent across the stack axis -- a wing's SPAN, whatever
       axis its planform is authored along (drag.h inducedCd, #462). The authored
       radius only happens to match today because every wing in this catalog is
       authored with its span along X. */
    double meshSpanHalf() const {
        double hx, hy;
        meshHalfSpan(hx, hy);
        return std::max(hx, hy);
    }

    /* Half-extent along the stack axis: the top of the actual mesh, which is
       where a surface contact at height `z` stops being on the part. */
    double meshHalfLength() const {
        if(!has_mesh_extents) { return height / 2.0; }
        return std::max(std::fabs(mesh_min.z), std::fabs(mesh_max.z));
    }

    /* Which part-frame axis is a SURFACE part's plate NORMAL: the thinner of the two
       axes across the stack axis. The chord is authored along +Z for every shipped
       wing and fin (mesh Z = chord, X = span, Y = thickness), so the normal is always
       X or Y here. A surface authored with its thickness along Z is not a plate in
       this convention and simply reads as the pre-#559 over-charge. */
    int meshNormalAxis() const {
        double hx, hy;
        meshHalfSpan(hx, hy);
        return (hx <= hy) ? 0 : 1;
    }

    /* The plate's own EDGE area, thickness x chord, off the same mesh box: 0.1 x 2 m
       for wing_r2h2, against the 8 m2 of PLANFORM the aero law charges it everywhere
       else. A surface turned spanwise to the flow presents this and nothing else
       (#559). No mesh -> the authored cylinder's box, which for a plate def equals
       its planform: no discount, i.e. the honest fallback is the old over-charge
       rather than an invisible zero-area plate. */
    double meshEdgeArea() const {
        return meshThickness() * (2.0 * meshHalfLength());
    }

    /* The plate's THICKNESS off the same box: the thinner of the two axes across the
       stack axis. chord x thickness is the edge area above; this is the half of it a
       2-D SECTION law needs (drag.h::waveCd, #543 A5b). */
    double meshThickness() const {
        double hx, hy;
        meshHalfSpan(hx, hy);
        return 2.0 * std::min(hx, hy);
    }

    /* The section's thickness-to-chord ratio, the number Ackeret wave drag wants.
       Every shipped surface is 0.044 (wing_r3h1.8) to 0.086 (elevator_r0.8h0.7) --
       these parts are plates, which is why the wave bill lands beside the profile cd
       they author rather than under it. A def with NO mesh falls back to the authored
       cylinder's box, so its ratio is 2 x radius / height -- >= 1 for the plate defs
       that fallback exists for, not 1: wing_r3h1.8 would read 3.33. drag.h caps that
       rather than trusting linear theory with it. */
    double meshThicknessToChord() const {
        const double chord = 2.0 * meshHalfLength();
        return (chord > 0.0) ? meshThickness() / chord : 0.0;
    }

    /* Which part-frame axis a SURFACE part's planform runs along: the WIDER of the
       two axes across the stack axis (the complement of meshNormalAxis). Every
       shipped wing and fin authors span along X, but a plate authored along Y is
       legal and #556's grouping must read its span where it actually is. A part
       with no mesh is squarish, and X is as good an answer as any -- it is the
       convention the rest of the catalogue uses. */
    int meshSpanAxis() const {
        double hx, hy;
        meshHalfSpan(hx, hy);
        return (hx >= hy) ? 0 : 1;
    }

    /* Attachment nodes. Empty -> load_parts_catalog synthesizes them from the
       size above. */
    std::vector<Node> nodes;

    /* What this part's SHAPE was, hashed: its mesh's corner positions, radius, height
       and the attach nodes (partGeomFingerprint). load_parts_catalog fills it
       and nothing writes it afterwards. A save records it per part (save.h,
       #456) because a save keeps SOLVED part poses rather than the attach spec
       that produced them, so if any of this moves after the save the ship it
       rebuilds is not the ship that was flown. 0 = no body to shape (a virtual
       fuel_link def, which refuseVirtualPartDef never lets into a save anyway);
       an absent key in a save is its own thing -- an empty string, "we do not
       know", not this zero. */
    uint64_t geom_fingerprint = 0;

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
    /* How many drums that rating is spread over, and so how many LINES the
       housing levers (AttitudeAuthority::wheels, actuators.h -- #576, #593):
       1 = a single drum along the part's own +Z, which levers the line it is
       mounted along and nothing else; 3 = an orthogonal triad, one drum per
       axis on the part's own X/Y/Z at `torque` each, which covers three lines
       whatever the mount rolls or tilts the box to. Every shipped wheel part is
       a triad: with single drums only, a coasting pod can roll and cannot point
       itself. A count other than 1 or 3 is a layout this model cannot place
       (2 would not say WHICH two axes; a real 4-drum pyramid wants per-drum
       axes), so the loaders refuse it. */
    int wheel_axes = 1;
    std::vector<double> propellant_rate; // kg/s per ResourceType at full throttle;
                              // all-zero -> not a thruster
    double exhaust_velocity;  // m/s; with propellant_rate -> thruster. For a JET
                              // this is the REAL exhaust velocity (drag.h jetThrust).
    /* Air-breathing modifier on a thruster (drag.h jetThrust): burns jet fuel
       against FREE air. Ignored unless also a thruster. */
    bool jet;
    double jet_fan_thrust;    // N; static (fan) thrust at the RATED INTAKE (1.225
                              // kg/m3, constants.h) -- the VTOL floor. Not "sea level":
                              // a body's own sea level is not the rating (#630).
    double jet_intake_area;   // m^2; effective intake/capture area (the ram term)
    double rcs_thrust;        // N; > 0 -> RCS translation authority (hydrazine mono).
                              // Rated thrust PER ACTUATOR (per bell). A rating with
                              // no rcs_nozzles is refused at load (#1120).
    /* Nozzle BELL (exhaust-ray) directions in the part's LOCAL frame,
       unit-normalized (src/rcs.h allocator, #428): a bell throws gas along
       its dir and the ship is pushed OPPOSITE it (reaction), so dirs are NOT
       force directions. Bells face out of the pod's faces; a 5-direction
       box omits its mount face (a bell there would fire into the hull).
       For a surface-mounted part the mount side is the SURFACE NODE's side
       -- the synthesized srf node points -X, so the omitted bell is -X
       (child +X faces away from the hull).
       An RCS-rated part AUTHORS its bells: the parts loader refuses a rating with
       none (#1120). An empty list used to stand for a magical omnidirectional
       pod -- six dirs along the part's own axes -- which invented hardware no
       author placed: the ring's reach, its arms and its plumes all came from
       the model, so a pod bolted flat to a hull still pushed the hull from
       its CoM in all six directions. */
    std::vector<glm::dvec3> rcs_nozzles;
    /* The ONE nozzle enumeration (parsed order capped at kMaxRcsNozzles):
       the sim builds its allocator columns in this order and render draws a
       plume per slot off Part::rcsFired, so a plume always lands on a nozzle
       that fired. kMaxRcsNozzles is the authoring cap, not a count a part
       gets for free. */
    static constexpr size_t kMaxRcsNozzles = 6;  // == sizeof Part::rcsFired
    size_t rcsNozzleCount() const { return rcs_nozzles.size(); }
    glm::dvec3 rcsNozzleDir(size_t i) const {
        assert(i < rcsNozzleCount());
        return rcs_nozzles[i];
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
    /* The TERMS of a hull's electrical balance, defined once here (#644). The VAB sums
       these over the assembly on the buildPower() row and Vehicle::powerTick() sums the
       same ones over the live parts, so what a part costs or makes has one spelling
       (#575's one-owner rule: two spellings of a term is how a readout starts lying).
       Which parts are on the hull, the EC gate and what the drums actually bill stay
       the sim's.
       The ACTIVE term carries the gate the bill uses rather than the raw field: a housing
       bills when it IS a wheel (`torque > 0`, the same test Part::isWheel() asks), not
       whenever someone happened to author a watt figure. That is what makes a command
       pod's built-in drums count exactly like a bought wheel block (#596), and what keeps
       a part with a stray power_draw on no drum off the bill. */
    double powerGenW() const { return power_gen; }
    double powerConstW() const { return power_draw_constant; }
    double powerActiveW() const { return torque > 0.0 ? power_draw : 0.0; }
    /* capacity[EC] is watt-hours, not kilograms (see the capacity note below). */
    double batteryWh() const { return capacity[(int)ResourceType::EC]; }
    /* Per ResourceType: kilograms for every resource EXCEPT capacity[EC], which is
       watt-hours -- Vehicle prints `capacity=%.1fWh` and partLoadMass skips EC as
       massless energy. Worth spelling out because the loader's error messages quoted
       "(kg)" for a battery until #496's review noticed. > 0 -> propellant tank. */
    /* DOUBLE, like the contents it bounds (ResourceContent::current went double in
       #379). Keeping this float was not just a lost byte: JSON cannot carry an infinity
       (nlohmann refuses 1e999 and the NaN/Infinity literals), but (float)1e308 IS one, so
       a legal number narrowed into +inf at the store, loaded past the >= 0 and total > 0
       checks, and made every pro-rata fuel share nan downstream (#530). */
    std::vector<double> capacity;

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
       the symmetric `drag`. Lift terms: 0 = no lift.

       TWO AREA CONVENTIONS, and drag.h::isAeroSurface is the line between them
       (#455, and the sentence #478 asks for):
         A BODY part (no lift_area, no control_area) is charged on the silhouette
         its MESH throws at the flow, so its cd is a blunt-body coefficient:
         capsule 0.35, tank 1.0, engine 0.9.
         A SURFACE (lift_area or control_area > 0) is charged on the PLANFORM it
         declares, and drag.h::surfaceFormCdA splits how that planform meets the
         flow: chordwise it pays its PROFILE cd (drag_forward = 0.01 on every
         shipped wing and fin -- a faceted section's profile cd; a smooth one is
         0.006-0.013, and #557's yardstick pass is where that gets settled), and
         out of its plane it pays the planform as a flat plate at drag_side = 1.0.
         Flow along its SPAN pays the plate's EDGE instead (thickness x chord, off
         the mesh box -- PartDef::meshEdgeArea), which is #559: keyed on the chord
         axis alone, #455 charged the whole planform for that too, so a wing in
         sideslip paid 8 m2 of plate for 0.1 m of aluminium. Authoring a blunt
         anchor (0.2) chordwise instead of a profile cd makes a surface pay a
         plate's worth of shape drag on top of the induced drag its lift bought.
       The chordwise anchors are not the last word on them: past the sound bar
       drag.h::waveCd adds an absolute cd to BOTH of them, from the section's own
       thickness-to-chord ratio off the same mesh box (meshThicknessToChord -- the
       shipped parts are 0.044-0.086 plates), so a supersonic wing pays for being
       thick as well as for being there (#543 A5b). Nothing to author: a part that
       is thin pays less. The broadside and edge anchors read none of it.
       `drag` stays the fallback for the three anchors on both kinds. Chute Cd·A is
       its own term and reads none of these. */
    double drag;          // the part's drag COEFFICIENT (dimensionless)
    double drag_forward;  // cd with the part's NOSE into the flow. Defaults to `drag`.
    double drag_side;     // cd broadside. Defaults to `drag`.
    double drag_backward; // cd with the part's BASE into the flow. Defaults to `drag`.
    /* #635: the SAME three anchors, priced in the supersonic regime, joined to the
       ones above by drag.h::regimeMix across the same 0.75..1.25 band the slope
       law (#543 A5) and the surface wave law (A5b) already own. An absent one IS the
       subsonic anchor beside it (see the accessors below), not a law: a blunt body's
       cd curve is NOT a wing's. A wing's wave drag comes off its thickness and DECAYS
       above the band; a pod's
       barely moves from subsonic to hypersonic, a blunt nose can even bill LESS once
       a shock stands off it, and a flat base bills more than anything else on the
       hull. So there is no multiplier here to invent -- the catalogue authors numbers
       and the join is a seam to measure them against. What each one is FOR: the
       broadside anchor is the one Newtonian theory gets right for a blunt shoulder
       (tmp/n1099/faceprobe2 measures 1.17-1.33 across the shipped meshes, where the
       subsonic anchors sit 0.8-1.1); the axial anchors are where a heat shield lives,
       and Newtonian's 2.0 for a flat end has no lee suction behind it, so a real
       shield authors nearer 1.1-1.7. A part that declares nothing is exactly as
       Mach-blind as it was before this field existed.
       BODY PARTS ONLY: these three are read in the hull silhouette pass and #947's
       crossflow, both of which skip a part whose planform makes it a surface
       (drag.h::isAeroSurface). A LIFTING or STEERING part's regime cd is waveCd off its
       own thickness-to-chord, not a second anchor set -- so the loader refuses these
       keys on a surface rather than filing them unread. A surface's blunt broadside and
       edge anchors stay regime-blind; whether they should move too is a separate call,
       not an accident of this field. */
    double drag_forward_sup;   // cd nose-in above the band. 0 = inherit drag_forward.
    double drag_side_sup;      // cd broadside above the band. 0 = inherit drag_side.
    double drag_backward_sup;  // cd base-in above the band. 0 = inherit drag_backward.
    /* Read them through these, never bare: a catalogue that predates #635, a scratch
       --parts file, or a bench that builds a PartDef by hand leaves the fields at 0,
       and 0 there means UNAUTHORED, not zero drag -- the part pays its subsonic
       anchor at every Mach, exactly as it did before the fields existed. A part
       cannot author a supersonic anchor of exactly 0 (nothing in this catalogue is
       dragless nose-on, and the fallback would silently swallow it); anything above
       0 is honoured. One rule, in one place, so the read path cannot disagree with
       the loader about what an absent anchor means. */
    double dragForwardSup() const {
        return drag_forward_sup > 0.0 ? drag_forward_sup : drag_forward;
    }
    double dragSideSup() const {
        return drag_side_sup > 0.0 ? drag_side_sup : drag_side;
    }
    double dragBackwardSup() const {
        return drag_backward_sup > 0.0 ? drag_backward_sup : drag_backward;
    }
    /* #1097: signed BODY-lift anchor (drag.h::bodyLiftCdA) — the cl a flat plate
       at angle of attack carries across its own upwind silhouette. The lift lies
       in the plane of the part's +z axis and the flow, so a hull family authors
       its chord along +Z (the same mesh convention meshNormalAxis reads for
       surfaces, with the same silent-90-degree-rotation failure mode if the
       thickness is authored along Z instead). Default 0: no hull lifts until a
       family opts in WITH re-fitted drag anchors, because the part's drag_side
       already bills a crossflow pressure share (partCd's drag_side x (1-c^2)
       weight) and opting in without re-fitting over-charges pressure. The
       catalog's one opt-in so far, liftbody_4m, IS fitted as a pair (#1107):
       drag_cd x drag_side = 1.20 sits inside a real flat plate's 1.17-1.28
       broadside, and body_lift / (2 sqrt(drag_side x drag_forward)) = 1.03 keeps
       the pair's best glide at the ~1:1 a plate can do. tests/test_aerosweep
       sweeps every opted-in catalog part over alpha and pins both halves -- which
       guards the OVER-lifted pair, the one #1107 found. An under-lifting pair
       passes quietly, and a part outside the shipped catalog is checked by nothing
       but the +-3.0 bound here (#1140). Mutually exclusive
       with the surface laws: a part with lift_area or control_area lifts
       through liftCurve/controlForce and must not pay twice. */
    double body_lift = 0.0;
    double lift_area;
    double cl;
    /* Where the surface's own curve stops being linear, and droops to nothing by 2x
       it (drag.h::liftCurve). ONE field, TWO readings, because it is one law: on a
       LIFTING part it is the angle of attack it stalls at; on a STEERING part
       (#494) it is the DEFLECTION where its authority peaks, past which more travel
       buys less force. The loader requires it on anything that steers. */
    double stall_angle;
    double control_area;
    ControlAxis control_axis;
    double cl_control;
    double max_deflection;

    /* Does this part steer? Four places ask (the aero law, the ship-def loader,
       the part window, the tick's per-surface recording arming) and they drifted
       apart the first time I wrote them by hand, so the test lives here (#492). */
    bool isControlSurface() const {
        return control_area > 0.0 && max_deflection > 0.0;
    }

    /* TRIM (#492): force a requested standing deflection (deg) inside this
       surface's travel; 0 for a part that cannot steer, so a trim left on the
       wrong part cannot hide in a save forever. Refusing instead of clamping is
       right at ship-def load time -- an author who wrote 90 into a 20 deg surface
       wrote the wrong number -- but a SAVE is player data, and losing a ship over
       a bad number is worse than clipping it. shipdef.cpp throws; save.cpp calls
       this. */
    double clampTrim(double trimDeg) const {
        if(!isControlSurface()) { return 0.0; }
        const double lim = glm::degrees(max_deflection);
        return (trimDeg > lim) ? lim : (trimDeg < -lim) ? -lim : trimDeg;
    }

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

    /* LANDING GEAR: a wheel that rolls on the ground. wheel_radius > 0 makes this
       a gear wheel; the ground law (src/wheel.h, Vehicle::applyGroundForces) reads
       these. This is NOT a reaction wheel -- isWheel() above is torque. Part frame
       convention: axle along X, rolling direction +Y, mount strut up +Z, origin at
       the axle centre. */
    double wheel_radius;   // m; > 0 -> a gear wheel
    double wheel_travel;   // m, suspension travel
    double wheel_spring;   // N/m, suspension stiffness
    double wheel_damp;     // N s/m, suspension damping
    double wheel_steer;    // deg, max nose-wheel deflection (0 = fixed)

    bool isGearWheel() const { return wheel_radius > 0.0; }

    /* LANDING LEG: a footpad on a splayed strut, surface-mounted on a tank's
       side. leg_foot > 0 makes this a leg; the ground law (src/wheel.h, the leg
       loop in Vehicle::applyGroundForces) stands it with the brake held on.
       NOT a gear wheel: type "gear" drives the horizontal runway/air-platform
       spawn, and a legged rocket stays vertical. Part frame convention: origin
       at the footpad centre, +Z up (parallel to the tank axis when mounted),
       splay baked into the mesh toward -X, mount plate at the top with the
       surface node on its hull-side face (dir -X). */
    double leg_foot;     // m, origin to pad contact face along -Z; > 0 -> a leg
    double leg_travel;   // m, crush stroke
    double leg_spring;   // N/m, suspension stiffness
    double leg_damp;     // N s/m, suspension damping

    bool isLeg() const { return leg_foot > 0.0; }

    /* LADDER: a climbable segment the EVA kerbal hangs off (src/eva.h). The
       field's PRESENCE makes the part a ladder. a/b are the segment's bottom
       and top ends in the part's LOCAL frame (+Z = stack axis); out is the
       unit direction the kerbal hangs off, away from the hull. Capsules carry
       an invisible hatch ladder along their slope (b = the hatch, 2/3 up);
       crewed fuselages one on the midbody; the ladder part one on its outer
       (+X) face. All authored by utils/art/gen_parts.py off the mesh. */
    bool has_ladder = false;
    glm::dvec3 ladder_a = glm::dvec3(0.0);
    glm::dvec3 ladder_b = glm::dvec3(0.0);
    glm::dvec3 ladder_out = glm::dvec3(0.0);

    bool isLadder() const { return has_ladder; }

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
                           //   edge: clock angle about the parent's stack axis,
                           //   turned into a contact by ellipseContact().
    double offset;         // m of gap along the attach axis / contact normal
    /* Stack edges mate two named nodes (defaults: parent "bottom"/"top" and
       child "top"/"bottom" for down/up). */
    std::string parentNode;
    std::string childNode;   // surface edge: the child's surface node id
    /* Surface edges: contact point + outward normal in the PARENT's local
       frame (explicit "point"/"normal", or the "angle"[+"z"] shorthand -- "z"
       has to land within the parent's mesh along its stack axis). */
    glm::dvec3 contactPoint;
    glm::dvec3 contactNormal;
    double roll;
    /* TRIM (degrees, #492): the standing angle a CONTROL SURFACE of this part sits
       at when the stick is free. Authored here, edited in flight in the part's own
       window, saved with the ship. It is NOT incidence: `roll` bolts a surface to
       the airframe at an angle, which is a build choice; trim is what the pilot sets
       so the airframe flies level untouched. See Part::trimDeg for the law. */
    double trim = 0.0;
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
    int controller = -1;   // part index; -1 = default seat (first capsule)

    /* Hull margin (m) for EVERY part of this ship; -1 = not set -> the catalog
       value, then the physics default. Ship-level because the welded-hull overlap
       problem depends on the SHIP'S layout. */
    double hull_margin = -1.0;

    /* Pad attitude (deg): how far the hull's nose is tilted off the launch site's
       local vertical, and which way about it. A pad stands every hull on its tail
       -- the site adds a yaw about the local vertical, nothing can tilt the
       airframe -- so a winged def has no way to ask for a level start, and its
       wings spend the whole vertical climb pointing sideways (#461).
       Pitch 0 (the default, and the rocket convention) leaves the pad pose as it
       was. With a tilt, the roll stops being faceAlong's axis pick and comes from
       the local vertical instead: +Y as close to "up" as the nose allows, i.e.
       wings level (#460). Yaw adds to the site's heading.
       A PAD field: an orbital bed aims the nose along prograde by design, so a
       non-pad spawn ignores it rather than fighting the bed. */
    bool has_attitude = false;
    double attitude_pitch = 0.0;
    double attitude_yaw = 0.0;

    int controllerIndex() const {
        if(controller >= 0) { return controller; }
        /* Default seat: the part the first-person eye sits on, the boot camera aims its
           initial focus at, and the Vessel window's home distance measures to. So ask for
           where the CREW is: the first capsule. It used to be the first part with
           reaction-wheel authority -- a pod is where the controls are, and on shipped
           defs that usually IS row 0 -- but #571 made a wheel an actuator and nothing
           else, so as a seat it puts the eye on whatever the wheels happen to be bolted
           to (res/ships/sideways_wheel.json: a wheel surface-mounted at angle: 90, i.e.
           rolled 90 deg about the stack, which is now the only shipped def this fallback
           moves -- to its root tank).
           It is NOT the ship's attitude frame and must not become one: the laws, the
           stick and every displayed heading/pitch/roll ask Vehicle::attitudeRef() (the
           root, #571) and read it through attitudeBasis() (#573). Those two agreed with
           this lookup by construction before #571, which is how the bug got in.
           A def that wants the view somewhere else says so with "controller". */
        for(size_t i = 0; i < parts.size(); i++) {
            if(parts[i].def && parts[i].def->crew_capacity > 0) { return (int)i; }
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

/* What a stage press DOES to a part -- derived from the def, and deliberately
   NOT an authored `stageable` flag (#970). The label only ever means one of
   these three things, because these are the only places the sim reads it: a
   thruster lights when the descending counter reaches its label and stays lit
   (`if(p->stage < as) continue;`, vehicle.cpp:6451), a decoupler separates and a
   chute deploys only when the counter EQUALS the label (vehicle.cpp:5573 and
   :3136). Anything else carries a label that fires nothing at a press, so the Staging
   window's ladder leaves those parts out (#1022) -- their rows were noise. "Fires nothing
   at a press" is not the same as "does nothing", though: such a label still moves where
   the press counter starts, and that is #1023. A part can hold two roles at once (a chute
   riding a decoupler), hence flags. */
enum class StageRole : unsigned { None = 0, Engine = 1u, Decoupler = 2u, Chute = 4u };

inline unsigned stageRoles(const PartDef *def) {
    if(def == nullptr) { return 0u; }
    unsigned r = 0u;
    /* The same predicate flight uses (Part::isThruster, part.h:202): a thruster
       is a propellant flow and an exhaust velocity, never a part whose name
       happens to contain "engine". */
    if(def->totalPropellantRate() > 0.0 && def->exhaust_velocity > 0.0) {
        r |= (unsigned)StageRole::Engine;
    }
    if(def->decoupler) { r |= (unsigned)StageRole::Decoupler; }
    if(def->chute_area > 0.0) { r |= (unsigned)StageRole::Chute; }
    return r;
}

/* The word the dumps print: "engine", "chute+decoupler", "-". The 0 case ("--") lost its
   window row when #1022 left non-responders out of the ladder, so what reads this now is
   --stage-dump and --fleet-report. Order is fixed by the mask, so two runs of one build
   print one string. */
inline const char *stageRoleName(unsigned roles) {
    constexpr unsigned kEng = (unsigned)StageRole::Engine;
    constexpr unsigned kDec = (unsigned)StageRole::Decoupler;
    constexpr unsigned kChute = (unsigned)StageRole::Chute;
    switch(roles) {
        case 0u:              return "-";
        case kEng:            return "engine";
        case kDec:            return "decoupler";
        case kChute:          return "chute";
        case kEng | kDec:     return "engine+decoupler";
        case kEng | kChute:   return "engine+chute";
        case kDec | kChute:   return "decoupler+chute";
        default:              return "engine+decoupler+chute";
    }
}

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
    /* ShipPart::trim. The editor has no control for it yet: it round-trips, it is
       not editable (same arrangement as BuildShip's attitude fields below, and for
       the same reason -- dropping it here would silently delete an authored or
       in-flight trim the moment a ship goes back through the VAB). */
    double trim = 0.0;
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
    /* ShipDef::attitude, carried so a def loaded into the editor survives a
       launch and a save (save_ship_def writes it back). The editor has no
       control for it yet: it round-trips, it is not editable. */
    bool has_attitude = false;
    double attitude_pitch = 0.0;
    double attitude_yaw = 0.0;

    /* Re-solve every part's localPos/localRot off its parent in construction
       order (root at identity). Call after any add/remove/re-orient. */
    void recomputePoses();

    /* True when part `partIdx`'s stack node `nodeId` is consumed by an existing
       stack edge (a mating consumes it on BOTH parts). This is the readable form and
       the reference tests/test_shipload.cpp checks fillNodeTaken against; it is NOT
       a form to call from a draw loop, since one answer walks the whole build. */
    bool nodeOccupied(int partIdx, const std::string &nodeId) const;

    /* Every port's answer at once: bit k of out[i] is set when parts[i]'s node k
       (counted in def->nodes order) is consumed by a stack edge. Same rule as
       nodeOccupied -- the equivalence is pinned in tests/test_shipload.cpp -- for
       one pass over parts instead of one pass per port (#542). out is REASSIGNED by
       the callee: a caller that keeps the vector across frames must not be able to
       inherit bits from the build that came before it. One uint64_t per part because
       a part def carries a handful of nodes (three, synthesized, for everything in
       the catalog today); the parse refuses a def over 64 nodes, so the assert in
       fillNodeTaken is a tripwire behind that rule, not the thing enforcing it. */
    void fillNodeTaken(std::vector<uint64_t> &out) const;

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

/* #499: is this BUILD aerodynamically stable, asked without a physics pass? The
   flight answer (Vehicle::lastLiftSlope / lastStaticMargin) needs air, an
   atmosphere and a substep, so a player in the VAB cannot see whether the airframe
   they just assembled will diverge until they have flown it. This asks the same
   law (drag.h::LiftSlopeSum, the same liftSlopePerQ that bills the flight force)
   at the one flow that needs no telemetry: an airframe flying straight ahead, at
   its design angle of attack. What it CANNOT see is the stall droop (#487's band)
   and the rotation sweep (#483) -- an airframe that reads +0.6 m here can still
   stop resisting mid-tumble, and only the flight number shows that.
   #543 A5 put a Mach correction on the flight slope, and this screen cannot have
   one: a design has no air and no speed, so there is no sound speed to divide by.
   The number below is therefore the Mach-0 LIMIT of the flight one -- the same
   answer at rest, BELOW the flight number while the air is still subsonic (a skiff
   at Mach 0.3 flies 1.05x it) and ABOVE it once the section is supersonic (0.52x at
   Mach 1.6). The static margin, a ratio of two slopes, does not move with Mach at
   all while one airframe stands at one Mach, which is why this screen is still
   worth showing.
   Conventions, because they are the numbers a player will compare:
   - the COM is the LAUNCH COM: PartDef::mass plus every resource but EC at full
     capacity, which is what Vehicle::init seeds and Part::effectiveMass then
     carries (minus what gets burned). Masses inside contents are not walked; a
     build tree has them as ordinary parts or not at all. A filled seat costs
     `seatOccupantMass` at the seat part's position, which is how the flight COM
     sees it -- pass 0 to ask about an uncrewed ship.
   - the ship axes are the ROOT's: BuildShip::recomputePoses leaves parts[0] at the
     identity, so nose = +z, up = +y, right = +x, exactly the frame the root's own
     axes give in flight.
   - control surfaces (control_area, no lift_area) contribute nothing, as in flight
     (#492 trims, it does not stabilise).
   `valid` is LiftSlopeSum::usable(): nothing lifting, or surfaces that cancel each
   other out, dash the pair rather than quoting the average of a cancelled sum. */
struct BuildStability {
    bool valid = false;
    double slope = 0.0;     // m^2/rad: what one radian of AoA buys
    double margin = 0.0;    // m BEHIND the COM; + = restoring, - = diverging
    double mass = 0.0;      // kg, the launch-convention COM mass
    glm::dvec3 com = glm::dvec3(0.0);   // ship-local (root frame)
};

/* `occupantMass` = what one seated kerbal weighs (the suit def's part mass plus
   its non-EC capacity: 97.05 kg as shipped). It is a parameter rather than a
   constant because the answer belongs to the loaded catalog, not to this header. */
BuildStability buildStability(const BuildShip &bs, double occupantMass = 0.0);

/* WHAT THIS ASSEMBLY COSTS THE POOL (#644) -- the electrical counterpart of
   buildStability(): the same four terms the flight balance prints (the Systems window's
   Power block and --power-log, #643/#649), asked of the parts on the pad with no sim at
   all. That is what the VAB cannot currently answer: mass, fuel, thrust and drag are on
   the build screen, and the watt figures that decide whether a hull can keep its
   instruments and its steering are not.
   `wheelRatingW` is the drums' RATED draw -- the number the EC gate weighs (#642) -- not
   what they will bill. Since #648 a hull bills what its wheels DELIVER, which no build
   screen can know: it depends on what the pilot asks. So this row answers the build
   question ("could this hull ever work these drums, and can it afford to keep trying"),
   which is exactly the question a player is asking while building.
   `storeWh` is the sum of the battery capacities, i.e. how much of the balance is banked
   rather than generated: a hull with gen < const draw survives on that number alone, and
   #652's silent black-out is what happens when it runs out mid-warp. */
struct BuildPower {
    double genW = 0.0;         // W, constant sources (RTGs)
    double constW = 0.0;       // W, always-on draws (life support)
    double wheelRatingW = 0.0; // W, the wheel housings' rated draw (#642's gate figure)
    double storeWh = 0.0;      // Wh of battery on board
};
BuildPower buildPower(const BuildShip &bs);

/* What one occupant of a seat weighs, counted the way Part::effectiveMass counts
   a seated suit: the whole suit ship's mass plus every resource but EC. Pass the
   def from load_suit_def (which is what makes it a suit at all). */
double seatOccupantMass(const ShipDef &suitDef);

/* A part's mass as a freshly built ship carries it: the dry def mass plus every
   resource but EC at full capacity, which is what Vehicle::init seeds and
   Part::effectiveMass then reports (minus whatever gets burned). */
double partLoadMass(const PartDef &def);

/* The same for a part a SAVE builds: dry mass plus the contents the file names.
   A part that names fewer slots than the game has leaves the rest empty, which is
   what make_part fills them with, so an old save weighs what it says rather than
   what the catalog could hold. EC stays massless. Split from partLoadMass because
   a save is player data and the two quantities differ: #547's ship-total door has
   to sum the one the load will actually build. It takes the dry mass rather than
   the PartDef so the rule stays testable from a header (test_save links nothing). */
inline double partSavedMass(double dryMass, const std::vector<double> &fuel) {
    double m = dryMass;
    for(size_t r = 0; r < fuel.size() && r < (size_t)ResourceType::Num; r++) {
        if(!resourceHasMass((ResourceType)r)) { continue; }
        m += fuel[r];
    }
    return m;
}

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

/* Where the surface "angle" shorthand puts its contact on a parent whose
   cross-section across the stack axis is the ELLIPSE bounded by (hx, hy), and
   the outward normal there. A round parent (hx == hy) is the circle the rule
   always used.
   Exposed for the unit test rather than left inline in the parser: every parent
   in the shipped catalog is round, so nothing else in the suite can tell an X/Y
   swap here from a fix, and a plate parent is exactly what the rule is for. */
struct EllipseContact {
    glm::dvec3 point;    // parent-local, at height z along the stack axis
    glm::dvec3 normal;   // unit, out of the ellipse
};
EllipseContact ellipseContact(double hx, double hy, double azimuthRad, double z);

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

/* A part's mesh file read ONCE: the bounding box of its vertices (#457) and the
   corner set the fingerprint hashes (#1182). load_parts_catalog calls this per
   part and feeds both, so a part costs one file read rather than two. */
struct MeshGeom {
    bool readable = false;     // false -> no file (the catalog never required one)
    bool hasExtents = false;   // false -> no vertex line in it
    glm::dvec3 lo = glm::dvec3(0.0);
    glm::dvec3 hi = glm::dvec3(0.0);
    std::string bytes;         // exact contents; the scan reads them, the fingerprint does not
    size_t cornerCount = 0;    // DISTINCT corners, as JoinIdenticalVertices leaves them
    uint64_t vertDigest = 0;   // a hash of that corner set: per-corner hashes, summed
};
MeshGeom readMeshGeom(const std::string &meshSubpath);

/* The vertex scan readMeshGeom runs over the file it read, split out so a test can
   feed text directly and say what the corner set is blind to (precision, order, a
   repeated corner) and what it is not (a corner that moved). `g.bytes` must already
   hold the text. */
void scanMeshGeom(MeshGeom &g);

/* FNV-1a over everything that decides where a part's solved poses sit: the mesh's
   VERTEX POSITIONS (not the file's bytes -- #1182: a normals-only or comment-only
   edit moved every fingerprint and a save then refused while claiming the geometry
   had changed), its radius and height, and every attach node (id, pos, dir,
   surface). load_parts_catalog computes it once per part; a save records it (save.h,
   #456) so a parts-file or mesh change that would reshape a saved ship is refused
   instead of rebuilding the ship silently wrong.
   Components are hashed as float, which is what assimp stores and what the collision
   hull is built from: a re-export that adds digits without changing the float leaves
   the fingerprint alone, which is the point of #1182. The corners go in as a SET,
   because assimp runs JoinIdenticalVertices and Bullet hulls what comes out, and six
   shipped meshes list a corner twice (barometer.obj and decoupler_radial.obj carry 28
   duplicate lines of 36) -- a re-export that deduplicates has not reshaped anything.
   What that gives up on purpose:
   a re-triangulation or a winding flip keeps the vertex set and so keeps the
   fingerprint, while it does change how the part draws -- a saved pose does not care.
   A mesh file that cannot be read folds in one stable value rather than throwing:
   the catalog never required the file to exist, and a file that shows up later moves
   the fingerprint -- which is the point.
   Not covered: what a part's geometry DOES rather than where it sits -- its
   drag and lift areas, and the chute canopy anchor (shipdef.h puts it at
   height/2 + 0.75*sqrt(chute_area), so retuning chute_area visibly moves the
   canopy on an already-saved ship). #389 made those catalog-only, and a saved
   pose does not depend on them. */
uint64_t partGeomFingerprint(const PartDef &d, const MeshGeom &mg);

/* The fingerprint as 16 lowercase hex digits -- what a save writes and reads
   (a 64-bit value as a decimal is 20 digits nobody can eyeball in a diff). */
std::string geomFingerprintHex(uint64_t v);

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
   `site` is an optional pad id, meaningful only for an on_pad scenario, and
   `lat_deg`/`lon_deg` are an optional surface coordinate to place an AIR spawn
   over -- same names, same convention (equirect.h) as a world_buildings entry.
   Either both are given or neither; the CLI refuses a half pair, so a NaN here
   means the parse skipped validation. `alt_m` is an optional altitude [m above
   mean sea level] for a CIRCULAR bed, overriding whatever height the scenario
   authors (its fraction of the shell or the air top, or a pinned radius); it is
   refused on a pad bed, whose height is the ground's, and on an ellipse bed,
   which asks for its two apsides instead. */
struct DebugStartShip {
    std::string ship;      // ship def path
    std::string name;      // display name
    std::string body;      // the body it sits on (a name in the system)
    std::string scenario;  // the spawn scenario (a --startship field)
    std::string site;      // pad id ("" = the body's default site)
    double lat_deg = kNoCoord;
    double lon_deg = kNoCoord;
    double alt_m = kNoCoord;
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
