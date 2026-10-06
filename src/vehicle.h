// vehicle.h -- the ship: Vehicle + its command types.

#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "flightlog.h" // FlightLog (the per-vessel mission journal)

// length2 (used by the inline methods) is a gtx function; quat_cast /
// mat3_cast (the btTransform helpers below) come from gtc/quaternion.
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/norm.hpp>

// Complete Bullet types come from btcommon.h (the single precision-settled
// include); body.h itself stays Bullet-include-free.
#include "btcommon.h"
#include "body.h"
#include "physics.h"
#include "shipdef.h"
#include "part.h"
#include "terrain.h"
#include "frame.h"
#include "orbit.h"

// One ship-control command. Vehicle::Command() is the only path from control
// to physics, so rules that apply to all controls live in one place.
enum ShipCmdType {
    ThrottleUp,
    ThrottleDown,
    Thrust,
    // Ship-relative (standard aviation mapping): see applyRotationForce.
    Pitch,    // W/S: about the ship's right axis
    Yaw,      // A/D: about the ship's up axis
    Roll,     // Q/E: about the ship's nose
    KillRot,
    Prograde,    // align nose with velocity
    Retrograde,  // align nose against velocity
    // RCS translation (ship-relative, KSP-style): see applyRcsForce.
    RcsNose,   // N/H: along the ship's nose
    RcsUp,     // I/K: along the ship's up
    RcsRight,  // J/L: along the ship's right
};

struct ShipCmd {
    ShipCmdType type;
    float amount;
    ShipCmd(ShipCmdType t, float a = 0.0f) : type(t), amount(a) { }
};

/* Autopilot slew targets -- mutually exclusive (last one wins). The manual
   stick is separate from these and composes with them. */
enum SlewMode {
    SlewNone = 0,
    SlewPrograde,    /* align nose with velocity */
    SlewRetrograde,  /* align nose against velocity */
    SlewRadialOut,   /* align nose away from the SOI body (radius vector) */
    SlewRadialIn,    /* align nose toward the SOI body */
    SlewNormal,      /* align nose with the orbital-plane normal (r x v) */
    SlewAntiNormal,  /* align nose against it */
    SlewKillRot      /* kill the spin */
};

struct ScenarioDef;  // the starting-scenario table (end of this file)

class Vehicle {
public:
    std::string name;   // display name (def name, disambiguated in main)
    std::string defPath; // the ship def file it was built from ("" = test ship)
    std::vector<Part *> parts;   // each Part owns its Body (see part.h)

    // SoI placement (setSoi is the one writer that re-homes a live vessel).
    // Null until placed: a fresh `new Vehicle` is off-map.
    TerrainBody *m_parent = nullptr;
    Frame *frame = nullptr;
    // Ownership bookkeeping: a ship lives in the ships list of its SOI body
    // (terrain.h). `home` is the body the ship was built on (fixed),
    // `scenario` its starting scenario, `slot` its pad/orbit slot.
    // `crew` are the characters aboard THIS ship (their capsule slot is
    // Kerbal::aboardPart, eva.h).
    TerrainBody *home = nullptr;
    const ScenarioDef *scenario = nullptr;
    int slot = 0;
    std::vector<Vehicle *> crew;
    TerrainBody *sun = nullptr; // the star (light source); set in main
    float m_thrust;

    /* Mission journal: flight start + SoI enter/leave. Started when the
       vessel is placed (setSoi). Not save-persisted. */
    FlightLog flog;

    glm::dvec3 m_com;

    /* the controller part (the cockpit, or the first reaction wheel by
       default): the camera basis and the stick frame are built from its
       local axes. */
    Part *controller;
    /* --- the ship as ONE rigid body (btCompoundShape) -------------------

       A ship is a SINGLE btRigidBody whose collision shape is a compound of
       the part hulls, each at its authored ship-local pose. There are no
       per-part rigid bodies and no welds: Part::body carries the render
       model, the collision hull and the mass, and nothing else. A part's
       pose is always DERIVED (partWorldPose) and every force goes to the
       one body, at the part's offset from the COM.

       Frames. S is the ship-local frame the authored poses live in (part.h):
       the root part's frame at build time. A btRigidBody's transform is its
       CENTRE-OF-MASS transform and its inertia is stored DIAGONAL, so the
       compound's children are re-based into the principal frame, and
       `principal` is the transform between the two. Hence

           hull transform      =  frameS() * principal
           a part's world pose =  hull transform * principal^-1 * T_S_part

       rebuildCompound() rebuilds the compound from the part list. */

    /* The ship's one rigid body, wrapped in a Body so the existing physics
       API works on it unchanged. Its MASS is not written directly: the
       ship's mass follows from the parts, through rebuildCompound. hull
       OWNS both btBody and shape. hull's render assets are null: the ship
       is drawn part by part. */
    Body *hull = nullptr;
    btTransform principal = btTransform::getIdentity();
    /* the parts in the compound, in child-index order. A child index is
       rebuild-scoped, so anything mapping a collision hit back to a Part
       goes through this. It is always `parts` in order. */
    std::vector<Part *> compoundParts;

    /* The ship's convex hull -- the union of every part's collision-hull
       verts -- in frame S, reduced to its extreme points. Rebuilt with the
       compound. applyAeroForce projects it along the flow for the ship
       silhouette (drag): a projected AREA is invariant under rigid
       transforms, so the substep only rotates v̂ into S. */
    std::vector<glm::dvec3> aeroHull;
    void rebuildAeroHull();

    /* Is the hull in the physics world? A registered collision object has a
       broadphase handle and an unregistered one does not -- asking Bullet
       rather than tracking a flag that could drift out of step. */
    bool hullInWorld() const;

    btCompoundShape *compoundShape() const;

    /* btTransform <-> (glm position, glm rotation). Always through a
       quaternion: btMatrix3x3 is row-major (m_el[i] = row i) and glm is
       column-major, so copying elements silently transposes. */
    static btTransform toBt(const glm::dvec3 &pos, const glm::dmat3 &rot);
    static void fromBt(const btTransform &t, glm::dvec3 &pos, glm::dmat3 &rot);

    /* Frame S in world coordinates: the hull's COM frame mapped back through
       `principal`. */
    void frameS(glm::dvec3 &pos, glm::dmat3 &rot) const;

    /* The COM in world coordinates, straight off the hull -- its transform
       origin IS the COM. O(1). */
    glm::dvec3 comPos() const;

    /* Place the whole ship: frame S at (sPos, sRot). setPosRot zeroes both
       velocities (proceedToTransform), so callers that care set them after. */
    void placeShip(const glm::dvec3 &sPos, const glm::dmat3 &sRot);

    /* The same, addressed by the COM instead of by frame S's origin. */
    void placeShipAtCom(const glm::dvec3 &com, const glm::dmat3 &sRot);

    /* (Re)build the compound + the hull from the CURRENT part list. Frame S
       and the velocity are carried across, so a rebuild neither teleports
       nor stops the ship. */
    void rebuildCompound();

    /* The centre of mass of the CURRENT part masses, in frame S. */
    glm::dvec3 compoundCom() const;

    /* The true mass COM minus the hull's transform origin, in world axes.
       Generally nonzero during a burn (fuel leaves the tanks while the
       origin stays put until the next refreshCompound). The force laws
       subtract the spurious (comOffset x F) torque it would introduce. */
    glm::dvec3 comOffset() const;

    /* Rebuild once the mass distribution has moved enough to matter. */
    void refreshCompound();
    static constexpr double kComRebuildTol = 0.01;
    static constexpr double kMassRebuildFrac = 1e-3;

    /* The compound must reproduce the assembly it was built from (mass
       properties + child poses), recomputed independently from the same
       authored data. Runs on every build, staging event and burn-triggered
       refresh -- in the unit tests and in the game. */
    void checkCompoundInvariants() const;

    /* The containment invariant (part.h): every part of this vehicle is
       attached to exactly this vehicle, and the container/contents/
       ownedContents edges agree. False -- with a [part] diagnostic -- on
       any violation. */
    bool checkPartInvariants() const;

    /* The part frame S is anchored to: the one with no parent edge. */
    Part *rootPart() const;

    /* The shroud condition (see PartDef.shroud): true when a part is
       attached on p's EXHAUST face. Read live, not cached: staging drops
       the child below and the shroud must go with it. */
    bool hasChildBelow(const Part *p) const;

    /* --- part state accessors -------------------------------------------

       The one route to a part's pose, axes and velocity. A part has no
       rigid body of its own: the pose is derived from the hull's transform
       through `principal` and the part's authored local pose. */
    void partWorldPose(const Part *p, glm::dvec3 &pos, glm::dmat3 &rot) const;
    /* The same pose, relative to the hull COM, computed purely from
       ship-local quantities. It never materializes the huge absolute frame
       coords, so it stays exact at any distance. */
    void partPoseRelCom(const Part *p, glm::dvec3 &pos, glm::dmat3 &rot) const;
    glm::dvec3 partPos(const Part *p) const;
    glm::dmat3 partRot(const Part *p) const;
    /* the part's local axis n (0 = right, 1 = up, 2 = nose) in world axes */
    glm::dvec3 partAxis(const Part *p, int n) const;
    glm::dvec3 partVel(const Part *p) const;
    glm::dvec3 partAngVel(const Part *p) const;

    /* --compound-check: rebuild + report the body the game is simulating. */
    void compoundCheck(double time);

    /* Fuel links (see PartDef.fuel_link): one-way fuel connections between
       fuel groups. `from` -> `to` means fuel flows from `from`'s group to
       `to`'s group. Virtual -- no physics. */
    struct FuelLink { Part *from; Part *to; };
    std::vector<FuelLink> fuelLinks;

    /* Cached fuel-drain layers, per fuel group. The layer structure depends
       ONLY on the group graph, so it is static until the groups change.
       buildFuelGroups clears it, so the cache can never outlive the
       structure it describes. */
    mutable std::map<int, std::vector<std::vector<int> > > drainLayers_;

    /* --drain-log state: the last sample's per-group total fuel mass +
       time, so the next sample can print the drain rate (kg/s). */
    std::map<int, double> drainPrevMass_;
    double drainPrevTime_ = 0.0;

    /* Electrical state (powerTick, per substep): the gate for the reaction
       wheels. Default true so a ship with no EC system is ungated. */
    bool powered_ = true;

    /* Rails: an idle ship in free fall coasts analytically on its two-body
       conic instead of being integrated: its rigid body is parked out of
       the Bullet world and its pose is re-derived from the conic every
       tick. While coasting, ship->frame is the SOI body's INERTIAL frame
       node. A grounded ship instead FREEZES: pose static in the rotating
       surface frame (railFrozen). */
    bool onRails = false;
    bool railFrozen = false;    // grounded park: no conic, pose fixed in the
                                // (rotating) frame
    /* Initialised, not just declared: three sites set onRails + railFrozen
       directly instead of going through goOnRails() (game.cpp, ships.cpp,
       save.cpp), so a railed-but-frozen vehicle can exist with the rail pose
       never written. Harmless while railFrozen short-circuits railsTick, but
       leaveRails() -> writeRailPose() would place the hull at garbage. */
    glm::dvec3 rail_pos = glm::dvec3(0.0);   // m, cluster COM in ship->frame coords
    glm::dvec3 rail_vel = glm::dvec3(0.0);   // m/s, inertial, ship->frame coords
    glm::dmat3 rail_orient = glm::dmat3(1.0); // cluster axes -> frame axes
    /* Frame S's axes at park time. The ship is rigid, so there is nothing
       per-part left to snapshot. */
    glm::dmat3 railRot = glm::dmat3(1.0);
    /* The instant rail_pos/rail_vel describe. It must equal frame->rail_time
       whenever the two are composed -- moveToRailFrame() asserts it. The
       frame tree is re-snapshotted once per tick, at the END of the tick, so
       a rail state from earlier in the tick lands the ship where its parent
       will be rather than where it was. */
    double rail_epoch = 0.0;

    /* Per-part catalog spec / stage / tank contents / armed thrust all live
       on each Part now (see part.h). */

    float thruster_util = 1.0;
    double exhaust_scale = 1.0;  // difficulty (New Game / save / --exhaust-scale):
                                 // scales rocket ve and the whole jet thrust;
                                 // synced per tick
    double drag_cd = 1.2;        // test knob (--drag-cd): the drag coefficient
                                 // (src/drag.h); 0 = no drag; synced per tick

    /* The last substep's aero (applyAeroForce), for the --drag-log. */
    glm::dvec3 lastAeroForce = glm::dvec3(0.0);   // total (lift + drag)
    glm::dvec3 lastLiftForce = glm::dvec3(0.0);   // the lift part
    glm::dvec3 lastAeroTorque = glm::dvec3(0.0);  // moment about the COM
    double lastDragAlt = 0.0;
    double lastDragRho = 0.0;
    double lastDragAlpha = 0.0;  // pitch angle of attack (rad) of the last substep
    double lastDragArea = 0.0;  // the ship's silhouette facing the flow (m^2)
    double lastDragCd = 0.0;  // area-weighted mean of the parts' cds

    /* The last substep's control-surface deflections (applyAeroForce), for
       the --drag-log. Stored as a PartDef* (no per-substep string copies). */
    struct ControlDeflection {
        const PartDef *def;  // the part type (name + control_axis)
        int index;           // the Nth control surface in the ship (0-based)
        double deflection;   // rad, signed (the applied hinge angle)
    };
    std::vector<ControlDeflection> lastControlDeflections;

    /* Rotation is armed once per tick (Command) and executed per SUBSTEP
       (applyRotationForce) -- like thrust, because Bullet clears the
       accumulated torque on each stepSimulation. stick: the manual command.
       slew: the autopilot target (exclusive). */
    float stick[3] = {0.0f, 0.0f, 0.0f};
    int slew = SlewNone;
    glm::dvec3 lastThrustForce{};  // debug: total thrust force applied last substep
    /* The autopilot mode the Autopilot window has engaged. Persistent across
       ticks, unlike `slew` (cleared each tick). */
    SlewMode slewRequest = SlewNone;
    void setSlewRequest(SlewMode m);

    void setRoot(Part *part);

    /* Hang `part` off the part at `parentIdx` and record its authored pose in
       the ship-local frame S. There is nothing to weld: the ship is ONE
       rigid body. This is the low-level primitive -- the pose is already
       solved; most callers want attachMode(). */
    void attach(Part *part, size_t parentIdx,
                const glm::dvec3 &localPos, const glm::dmat3 &localRot);

    /* Solve `part`'s ship-local pose off the part at `parentIdx` with the
       shared attachPose() geometry. For the stack modes (Down/Up); surface
       edges use attachSurface() below. */
    void attachMode(Part *part, size_t parentIdx, AttachMode mode,
                    double angleDeg = 0.0, double offset = 0.0);

    /* attachMode() against the most recently added part. */
    void attachDown(Part *part);

    /* Surface-attach `part` (by its surface node) at a contact `point` with
       an outward `normal`, both in the parent's local frame. */
    void attachSurface(Part *part, size_t parentIdx,
                       const glm::dvec3 &point, const glm::dvec3 &normal,
                       double rollDeg = 0.0, double offset = 0.0);

    void init();

    /* init() minus the tank re-seed: the bookkeeping that finalizes a ship
       whose part list is already final. init() calls it after seeding;
       extractSubtreeAsShip() calls it directly. */
    void finalize();

    /* Put the ship's one rigid body into the physics world. Kept apart from
       init(), which builds it, so a headless caller can build without a
       physics world. */
    void enterWorld();

    /* True of the EVA kerbal (src/eva.h). */
    virtual bool isEva() const;

    /* True while an EVA character is ABOARD a ship (parked inside a capsule).
       Overridden in src/eva.h. */
    virtual bool isCrewAboard() const;

    /* The capsule Part this vehicle is parked in (its single source of
       truth for WHERE an aboard character sits). Kerbal overrides it
       (src/eva.h) to return its aboardPart. */
    virtual Part *capsulePart() const;

    /* Assign each part a fuel-group id (Part::fuelGroup). A fuel group is a
       connected component of the part tree across the parts that CONDUCT
       fuel; a part with PartDef::fuel_barrier is a WALL. Barrier parts keep
       fuelGroup = -1. Recompute after a stage split. */
    void buildFuelGroups();

    /* The tanks an engine may draw fuel from: the tanks in its fuel group.
       This is the single place a fuel link will later change. */
    std::vector<Part *> fuelPool(Part *engine) const;

    /* The fuel groups an engine can draw from, in LAYERS by hop distance,
       furthest layer first. Groups in a layer drain TOGETHER (pro-rata) --
       that is what keeps a symmetric star draining symmetrically. Cached
       per group (drainLayers_); the caller must not mutate the reference. */
    const std::vector<std::vector<int> > &fuelDrainLayers(Part *engine) const;

    /* Draw `amt` kg of `type` from the engine's fuel sources, LAYER by
       LAYER and pro-rata across ALL the tanks in a layer. Pro-rata, NOT
       first-tank-first: draining one tank to empty before its siblings
       shifts the ship's mass distribution and torques it under thrust.
       Returns true if the total covers amt (else the thruster doesn't fire). */
    bool consumeResourceMass(enum ResourceType type, float amt /* kg */, Part *engine);

    /* Total kg of `type` available to `engine` across its drain layers.
       Lets ApplyThrust size the burn before draining, so it never drains one
       propellant and leaks it because another ran short. */
    float availableResourceMass(enum ResourceType type, Part *engine) const;

    /* Fill burn[ResourceType::Num] with the resources the ship's engines
       draw (propellant_rate > 0). includeJets also counts jet fuel. */
    void enginePropellantMask(bool *burn, bool includeJets) const;

    /* Current mass (kg) of the resources flagged in `burn`, over all parts. */
    float fuelMassMasked(const bool *burn) const;

    float getDeltaV();

    /* TODO should be cached per frame */
    float getMass();

    /* The crew's felt acceleration [m/s^2]: |thrust + aero| / mass (gravity
       excluded). */
    double feltAccel();

    /* Test-only accessor: drives the real ApplyThrust so its invariants can
       be pinned headlessly. Do not call it from game code. */
    void ApplyThrust_TESTONLY(double step) { ApplyThrust(step); }

    /* --- electrical (KSP-style EC) ---------------------------------------
       The ship's EC is a shared pool across its battery parts. powerTick
       runs once per substep, BEFORE applyControlForces: it gates the
       reaction wheels on power, and balances generation (RTGs) against the
       constant draw (life support) and the active draw (the wheels).
       Units: power in W, charge in Wh. EC has no mass. */
    void powerTick(double h);

    /* Drain up to `wh` of EC from the pool, pro-rata across the batteries. */
    void drainEC(double wh);

    /* Charge the pool by up to `wh`, pro-rata across the batteries. */
    void chargeEC(double wh);

    /* Total EC charge / capacity across the pool (the HUD + --power-log). */
    void getPower(double *gen, double *constDraw, double *charge, double *capacity);

    /* --power-log: the ship's power balance + pool + gate, one line per
       sample. */
    void power_log(double time);

    /* Staging state. `activeStage_` is a monotonic stage COUNTER (the stage
       about to be triggered): it starts at the HIGHEST stage number and
       steps down by one on each stage press. An engine fires once the
       counter has reached its stage (stage >= activeStage_) and then stays
       lit; a decoupler triggers when the counter is at its stage.
       `totalStages_` is the highest stage number at build time;
       `minStage_` is the lowest (the counter's floor). */
    int activeStage_ = 1;
    int totalStages_ = 1;
    int minStage_ = 1;

    /* The active stage (the counter, see above). */
    int activeStage();
    /* Step to the previous stage (clamped at the lowest one). */
    void advanceStage();

    /* Total number of stages on the ship (the highest stage number at build
       time), for the "stage X of N" readout. */
    int numStages();

    float getThrust();

    // current Thrust-to-weight ratio
    float getTWR();

    // full throttle TWR
    float getFullThrustTWR();

    // empty TWR
    float getMaxTWR();

    void setVelocity(glm::dvec3 vel);

    /* The COM is the hull's transform origin -- a rigid body's transform IS
       its centre-of-mass transform -- so this is O(1). */
    const glm::dvec3& get_center_of_mass(void);

    glm::dvec3 applyGravity();

public:
    Vehicle();
    /* Tear the ship down in a safe order: unregister the one rigid body,
       delete it, then delete the parts. The onRails guard is LOAD-BEARING:
       Bullet's removeCollisionObject is not idempotent -- a second remove
       on an absent body can evict the WRONG collision object. */
    virtual ~Vehicle();

    glm::dvec3 processGravity();

    // Bullet clears all accumulated forces on every stepSimulation, so the
    // thrust must be re-applied before EVERY substep.
    void applyThrustForce();

    /* Aero (v2, reports/aerodynamics2026_09_11): the lift + drag the air
       exerts on the ship. LIFT acts PART BY PART at each part's position
       and DRAG acts SHIP-LEVEL at the center of pressure. The drag
       coefficient is the area-weighted mean of the parts' 3-anchor
       directional cds (src/drag.h partCd); the area is the ship's convex-
       hull silhouette (src/drag.h projectedArea). Applied at the center of
       pressure so the pitch-stability (weathervane) torque is preserved.
       Re-applied before EVERY substep (Bullet clears forces). */
    glm::dvec3 applyAeroForce(double h);

    /* The local air density (kg/m^3) at the ship's COM. Deliberately has NO
       kRhoFloor gate: its consumer (jetThrust) scales by rho/rho_sea, so a
       sub-floor density already reads as vacuum. */
    double airDensityAtCom() const;

    /* The ship's velocity relative to the AIR at `com` (its COM in the
       current frame): GetVel() minus the atmosphere's co-rotation when
       outside the body's rotating frame (issues #60/#97). Consumers:
       applyAeroForce (drag/lift/control) and the jet intake speed. */
    glm::dvec3 airRelativeVel(const glm::dvec3 &com);

    /* The armed control forces, re-applied before EVERY substep. The EVA
       kerbal overrides with its own laws (src/eva.h). */
    virtual void applyControlForces(double h);

    /* the first reaction-wheel PART (nullptr if none): the stick / slew /
       kill-rot laws all use it as the ship's attitude reference. */
    Part *firstWheel();

    // The armed rotation commands are re-applied before EVERY substep.
    void applyRotationForce(double h);

    /* --- RCS translation (hydrazine mono, KSP-style) --------------------
       Field-driven (Part::isRcs). The armed direction rcsDir is
       SHIP-RELATIVE. The force is applied AT the COM (ApplyCentralForce):
       the COM-translation approximation. */
    glm::dvec3 rcsDir = glm::dvec3(0.0);  // armed dir in ship axes (right, up, nose); 0 = off
    /* burned this tick (applyRcsForce drew hydrazine): the render pass
       draws the COM plume off this. */
    bool rcsFiring = false;
    void clearRcs();
    /* The armed RCS direction in WORLD axes (unit; (0,0,0) when disarmed).
       Resolved at the point of use, so the direction tracks the ship's live
       attitude substep by substep. */
    glm::dvec3 rcsWorldDir() const;
    Part *firstRcsPart();
    double maxRcsThrust();
    void applyRcsForce(double h);
    /* s, monopropellant (hydrazine) efficiency -- the EVA suit's value. */
    static constexpr double kRcsIsp = 220.0;

    /* Autopilot diagnostic (throttled; --slew-log). */
    void slew_log(double time);

    /* --att-log: the ship's nose (local +Z) and its angular velocity. */
    void att_log(double time);

    /* --tq-log: the spurious-torque bug class on one line (|dcom x F| vs
       |w|). Stateless. */
    void tq_log(double time);

    /* --fuel-log: each fuel group's fuel mass (per resource) and the fuel
       links -- the instrument for the fuel-link drain-rate bug. */
    static const char *resourceName(int r);

    void fuel_log(double time);

    /* --drain-log: the thrust delivered this tick (N) + each fuel group's
       drain rate (kg/s). The first sample only records the baseline. */
    void drain_log(double time);

    /* the largest wheel's rated torque (N m) -- the per-wheel rating for
       the HUD; the ship's TOTAL wheel authority is maxTorque() (the sum) */
    float GetWheelTorque();


    /* disarm the armed thrust (called once per tick, like clearRotCmd) */
    void clearThrust();

    /* disarm the armed rotation commands (called once per tick) */
    void clearRotCmd();

    /* called when control moves to ANOTHER ship: zero the throttle and
       clear the armed thrust + rotation commands. */
    void releaseControl();

    /* The parts that WOULD be dropped if `stage` is triggered: each
       decoupler on that stage plus the child-side subtree it anchors. The
       decoupler itself IS dropped (it flies off with the stage). Empty if
       no decoupler is on that stage. */
    std::vector<Part *> droppedPartsAtStage(int stage);

    /* --- docking ----------------------------------------------------------

       A dock joins two ships into ONE rigid body: this ship (the survivor)
       absorbs the other. The absorbed ship's parts are rebased into this
       ship's S, its root is reparented under this ship's port part, and
       this ship is rebuilt as the union. The joint is recorded as a seam,
       so an undock (extractSubtreeAsShip) undoes exactly this.
       After the call the absorbed ship is an empty shell. */

    /* One dock seam: the tree edge that joins the two ships. `port` is
       THIS ship's port part, `root` the absorbed ship's root part. */
    struct DockSeam {
        Part *port;
        Part *root;
        std::string name;
    };
    /* The docks this ship has absorbed, innermost first (the most recent /
       outermost dock is last -- the one undock selects). */
    std::vector<DockSeam> seams;

    /* Docking INTENT (held PER SHIP). A dock needs BOTH halves set:
       dockTargetShip/dockTargetPort (the port on ANOTHER ship) and
       dockArmPort (which of THIS ship's ports does the mating). */
    Vehicle *dockTargetShip = nullptr;
    Part *dockTargetPort = nullptr;
    Part *dockArmPort = nullptr;   // this ship's port to mate with (mandatory)

    /* Dock-absorb: fold B into this ship at the mated port. B's shell is
       consumed and deleted by the caller; B's CREW ride along. B's
       ship-level flight journal is dropped by design (v1) -- see #49. */
    void absorbShip(Vehicle *B, Part *portA);

    /* Extract a connected subtree (rooted at `root`) into a new Vehicle.
       The dropped parts keep their exact relative geometry, rebased into a
       new frame S' = the root's old frame. The new ship inherits this
       ship's frame/home/sun, is placed at the root's current world pose,
       and given the rigid velocity of the dropped side's COM.
       Returns nullptr if `root` would drop the whole ship. Seams are
       maintained like fuel links. The new ship IS in the fleet list but
       NOT yet in the physics world -- the caller enters it (enterWorld).
       Undock and staging's "dropped stage becomes a ship" are the users. */
    Vehicle *extractSubtreeAsShip(Part *root, const std::string &name,
                                  double t = 0.0);

    /* This ship's part frame -> renderFrame. Usually the identity. */
    glm::dmat4 renderXform(Frame *renderFrame) const;

    void Draw(const Camera* camera, Frame *renderFrame);

    // Single place to control the ship. While paused (simActive == false)
    // every command is dropped.
    /* step = the tick's simulated duration (dt * time_accel); only Thrust
       uses it (to scale this tick's fuel flow). */
    void Command(ShipCmd cmd, bool simActive, double step = 0.0);

    /* The COM velocity. */
    glm::dvec3 GetVel();

protected:
    // Control implementation: applies forces/torques to the Bullet bodies
    // directly, so it is reachable only through Command() above.
    // (protected, not private: the EVA kerbal reuses the rotation-model
    // helpers below for its own attitude law.)
    void adjustThrottle(float delta);

    /* the ship's full-throttle thrust RIGHT NOW (N) = the sum of every
       ignited engine's full thrust (jets contribute their sea-level peak),
       scaled by exhaust_scale. */
    float GetActiveThrust();

    /* Called once per physics tick. Consumes the tick's fuel and arms the
       per-thruster thrust; the force itself is applied by applyThrustForce()
       before EVERY substep. A thruster that can't consume its flow this
       tick doesn't thrust. Stage gates WHEN it ignites; the fuel group
       (connection) decides WHAT it burns. */
    void ApplyThrust(double step);

    // --- physical rotation model (private law implementation) -------------
    // alpha = maxTorque() / I (rad/s^2). Stick, prograde/retrograde slew and
    // kill-rot all work within that authority.

    double maxTorque();

    /* The ship's moment-of-inertia tensor about its COM, in world axes:
       the one rigid body's own (Bullet stores it DIAGONAL in the principal
       frame). O(1). */
    glm::dmat3 getInertia();

    /* The target direction (in the ship's frame) for the current directional
       slew mode. Radial / normal reference the SOI body. */
    glm::dvec3 slewTargetDir();

    /* Slew the nose (local +Z) toward `dir` within the wheel's authority:
       the target rate is the braking curve sqrt(2*alpha*E), capped at
       E/(2h), and the per-substep rate change is bounded by alpha*h. */
    void slewToward(glm::dvec3 dir, double h);

    /* Kill the spin within the wheel's authority: drive the full angular
       velocity to zero in one substep (tau = I * (-w) / h), scaled down to
       |tau| <= maxTorque(). Monotonic, no sign flip. Must use the FULL
       inertia tensor (the world-axis diagonal alone limit-cycles when the
       principal basis is rotated relative to world). */
    void killRotStep(double h);

public:

    /* A part's position in another frame's coordinates. */
    glm::dvec3 GetPositionRelTo(const Part *part, Frame *relTo);

    /* Re-express the ship's physics state (pose + velocity) in newFrame
       and re-home it there (setSoi). */
    void moveToFrame(Frame *newFrame, double t);

    /* The ONE SoI writer: every live (frame, m_parent, ships-list, journal)
       change goes through here. Idempotent list membership -- a free vessel
       is in its body's ships list, an aboard crew character is not. */
    void setSoi(Frame *newFrame, double t);

    /* Out of the SoI body's ships list, if in it (the removal sites:
       recover, remove, dock-absorb, pick-up). */
    void detachSoiList();

    /* Per-tick SOI bookkeeping for THIS ship (the frame tree is shared;
       each ship tracks its own position in it). */
    void switchFrames(double t);

    /* Write the rail state into the ship's body (once per tick). Angular
       velocity is zeroed: a parked ship is torque-free. */
    void writeRailPose();

    /* The ship's COM state in `inertial` -- the frame node where its
       trajectory is a Kepler conic. The ordering matters: the OLD frame's
       stasis velocity is added before rotating, and the result is offset
       by the frame's own velocity in the inertial node. */
    void comStateIn(Frame *inertial, glm::dvec3 &p, glm::dvec3 &v);

    // Separation between this ship's COM and another's, in the universe (root)
    // frame (frame-invariant distance).
    double distanceTo(Vehicle *o);

    /* The COM's osculating orbit dips into the terrain band (periapsis
       within 3 km of the surface). This is a PERIAPSIS test, not a
       proximity one -- ask isGrounded()/isOrbiting() for where the ship is
       RIGHT NOW. */
    bool inTerrainBand();

    /* Coast-clear of the ground: the COM's conic does not intersect the
       body (periapsis above the terrain band). */
    bool isOrbiting();

    /* Resting on the surface NOW: in the rotating surface frame, near-static
       in it, and within a generous band of the analytic terrain. Both terms
       are load-bearing: the band alone calls a low hover "landed", and the
       speed alone calls a hovering ship landed at any altitude. */
    bool isGrounded();

    /* Rails classification: an ORBITING ship coasts on its conic; a GROUNDED
       one freezes in its rotating surface frame. Anything else is not
       rail-eligible. */
    bool canRail();

    /* Park this ship out of the physics world and coast it analytically.
       Refuses (returns false) if the ship is not rail-eligible. */
    bool goOnRails();

    /* Re-enter physics from rails: rebuild the Bullet state and hand the
       ship back to the integrator. */
    void leaveRails();

    /* Per-tick rail advance: propagate the conic, test the END of the
       advance for an SoI boundary, hand off there, refresh the parked
       transforms. A frozen (grounded) ship has nothing to propagate. The
       test only sees the endpoint, so a step longer than a body's whole
       sphere flies clean past it -- see railsTick. */
    void railsTick(double t, const double step);

    /* Re-anchor the rail state on another frame and re-home it (setSoi).
       Preconditions it asserts: the rail state and BOTH frames describe the
       same instant, and the target contains the ship at that instant (the
       endpoint soiTarget test guarantees the latter for a child handoff). */
    void moveToRailFrame(Frame *newFrame, double t);

private:
    /* The rail state in universe-root axes, compared before/after a frame
       switch by moveToRailFrame()'s continuity asserts. See the comment there
       for what those asserts can and cannot detect. */
    void railRootState(glm::dvec3 &p, glm::dvec3 &v) const;

    /* The SoI boundary test shared by switchFrames (physics) and the rails
       path. kSoiMargin (constants.h) of hysteresis on both sides keeps a
       ship loitering at a boundary from flapping. */
    Frame *soiTarget(const glm::dvec3 &posInFrame, bool skipSameBody);
};

// Forward declaration (system.h defines it); spawn_vehicle resolves the
// home body's SOI through the system's frame tree.
struct System;

/* Build a ship's part tree (structure only): create the physical parts
   + the attach edges + the controller + the fuel links. Does NOT seed the
   tanks, place the ship, or enter the physics world. GL is needed (shader
   binding); the catalog must outlive the ship. */
void build_ship_structure(Vehicle *ship, const ShipDef &def, Shader *partsshader);

/* Instantiate a ship def on a pad: build the part tree, seed the tanks full,
   place the ship's lowest point on the pad top, and enter the physics world. */
void build_ship(Vehicle *ship, const ShipDef &def, Shader *partsshader,
                const glm::dvec3 &base, const glm::dmat3 &orient);

/* Starting scenario (chosen at the CLI on startup). The pad scenarios are
   already set up in main (the ship is built on the pad); the orbit scenarios
   place the ship in a circular orbit around the home body. The ellipse-*
   scenarios place the ship on a 10 km x 1000 km ASL orbit. The escape
   scenario places the ship at the circular-orbit radius with esc_frac x the
   local escape velocity (a hyperbola). The distance scenarios (neptune,
   oort, interstellar) set abs_r instead: a circular orbit at an ABSOLUTE
   radius, for precision testing. */
struct ScenarioDef {
    const char *name;
    bool on_pad;
    double alt_frac; // circular: fraction of the near-body shell (rot SOI -
                     // radius), or of the atmosphere top when atmo_frac
    bool polar;
    int ell_phase;   // -1: circular; 0: at periapsis; 1: at apoapsis; 2: at 90 deg
    double peri_alt; // ellipse: periapsis altitude above the body radius (m)
    double apo_alt;  // ellipse: apoapsis altitude above the body radius (m)
    double esc_frac; // escape: launch speed in local escape velocities (0 = not escape)
    double abs_r;    // > 0: absolute circular-orbit radius from the body
                     // centre (m), overriding alt_frac
    bool atmo_frac = false; // alt_frac scales the atmosphere top() (the
                     // flying beds: science bands are top() fractions, so
                     // the beds always land in their band); airless bodies
                     // fall back to the shell fraction
};

/* Look up a scenario by name; throws listing the available names if
   unknown. */
const ScenarioDef *scenario_by_name(const std::string &name);

// Enumerate the scenario table (the VAB's scenario dropdown). Names are
// stable C strings valid for the process lifetime; index < scenario_count().
size_t scenario_count();
const char *scenario_name_at(size_t i);

// Orientation with the nose (local +Z) along `dir`; the roll axis is the
// coordinate axis most orthogonal to dir (never singular for a unit dir).
glm::dmat3 faceAlong(const glm::dvec3 &dir);

/* slot_offset (m): lateral separation for ships sharing a scenario --
   applied along the orbit binormal so each ship's orbit stays essentially
   the same shape. 0 for a lone ship (and no-op for pad scenarios). */
void spawn_vehicle(Vehicle *ship, const ScenarioDef &sc, TerrainBody *home,
                   System &sys, double slot_offset, double t);

/* --radial-test spin diagnostics: the ship's rotational state and the
   tidal gravity torque. */
void spin_log(Vehicle *ship, double time);
