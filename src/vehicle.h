// vehicle.h -- the ship: Vehicle + its command types.
//
//   ShipCmdType / ShipCmd  one control command (throttle, thrust, pitch, ...).
//   SlewMode               autopilot slew target (prograde/retro/kill-rot).
//   Vehicle                a built ship: its parts, controls, staging, and
//                          the rails coasting state machine.
//
// Declarations only: Vehicle's method bodies live in vehicle.cpp.

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

// length2 (used by the inline methods) is a gtx function; quat_cast /
// mat3_cast (the btTransform helpers below) come from gtc/quaternion.
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/norm.hpp>

// body.h must come before physics.h: body.h sets the bullet
// double-precision define and includes the complete bullet types, and
// physics.h names btDefaultCollisionConfiguration in a member (only
// forward-declared there), so it needs the complete type.
#include "body.h"
#include "physics.h"
#include "shipdef.h"
#include "part.h"
#include "terrain.h"
#include "frame.h"
#include "orbit.h"

// One ship-control command. The input layer (keyboard, UI, or a future
// autopilot) emits these; Vehicle::Command() is the only path from control
// to physics, so rules that apply to all controls (e.g. "no commands while
// paused") live in one place instead of at every call site.
enum ShipCmdType {
    ThrottleUp,
    ThrottleDown,
    Thrust,
    // Ship-relative (standard aviation mapping): each key drives the
    // ship's own body axis -- see applyRotationForce for the exact map.
    Pitch,    // W/S: about the ship's right axis
    Yaw,      // A/D: about the ship's up axis
    Roll,     // Q/E: about the ship's nose
    KillRot,
    Prograde,    // align nose with velocity
    Retrograde,  // align nose against velocity
    // RCS translation (ship-relative, KSP-style): each command drives the
    // ship's own body axis -- see applyRcsForce for the exact map.
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
    std::string defPath; // the ship def file it was built from ("" = test ship);
                         // lets a runtime spawn duplicate this ship's design
    std::vector<Part *> parts;   // each Part owns its Body (see part.h)

    TerrainBody *m_parent;
    Frame *frame;
    // Ownership bookkeeping: a ship lives in the ships list of its SOI
    // body (terrain.h) -- that is m_parent, which changes on a SoI
    // crossing (moveToFrame moves the ship between the lists). `home` is
    // the body the ship was built on (fixed), `scenario` its starting
    // scenario and `slot` its pad/orbit slot within the (home, scenario)
    // group. `crew` are the characters aboard THIS ship (their capsule
    // slot is Kerbal::aboardPart, eva.h); they are in no body's list
    // while aboard.
    TerrainBody *home = nullptr;
    const ScenarioDef *scenario = nullptr;
    int slot = 0;
    std::vector<Vehicle *> crew;
    TerrainBody *sun = nullptr; // the star (light source); set in main
    float m_thrust;

    glm::dvec3 m_com;

    /* the controller part (the cockpit, or the first reaction wheel by
       default): the camera basis and the stick frame are built from its
       local axes. build_ship() resolves it from def.controllerIndex(). */
    Part *controller;
    /* --- the ship as ONE rigid body (btCompoundShape) -------------------

       A ship is a SINGLE btRigidBody whose collision shape is a compound of
       the part hulls, each at its authored ship-local pose. There are no
       per-part rigid bodies and no welds between them: Part::body carries the
       render model, the collision hull and the mass, and nothing else. So a
       part's pose is always DERIVED (partWorldPose) and every force goes to
       the one body, at the part's offset from the COM -- which is what
       delivers an off-axis engine's torque, and the tide's.

       Frames. S is the ship-local frame the authored poses live in (part.h):
       the root part's frame at build time, so the root's authored pose is the
       identity. A btRigidBody's transform is its CENTRE-OF-MASS transform and
       its inertia is stored DIAGONAL, so the compound's children cannot stay
       in S: they are re-based into the principal frame, and `principal` --
       btCompoundShape::calculatePrincipalAxisTransform's output -- is the
       transform between the two. It maps the body's COM frame ONTO S (origin
       = the COM in S, basis = the principal inertia axes in S; Bullet's
       diagonalize documents tensor_S = basis * tensor_body * basis^T). Hence

           hull transform      =  frameS() * principal
           a part's world pose =  hull transform * principal^-1 * T_S_part

       The compound is a pure function of the part list (authored geometry +
       current masses), so rebuildCompound() is the whole of it: staging, a
       runtime spawn and a burn that has moved the mass distribution far
       enough (refreshCompound) all just call it again, preserving frame S and
       the velocity across the rebuild. */

    /* The ship's one rigid body, wrapped in a Body so the existing physics
       API works on it unchanged -- GetPosition, GetVelocity, ApplyForce,
       ApplyTorque, SetFriction, NeverSleep, BodyInContact, setPosRot and
       Remove/AddPhysicsBody all take a Body*. Its MASS is not written
       directly: the ship's mass follows from the parts, through
       rebuildCompound. hull->btBody is the rigid body
       and hull->shape the compound, and hull OWNS both, so nothing else frees
       them. hull->model is null: the ship is drawn part by part. */
    Body *hull = nullptr;
    btTransform principal = btTransform::getIdentity();
    /* the parts in the compound, in child-index order. A child index is
       rebuild-scoped (only meaningful for the current compound), so anything
       mapping a collision hit back to a Part goes through this. It is always
       `parts` in order -- test_inertia pins that, and picking relies on it. */
    std::vector<Part *> compoundParts;

    /* Is the hull in the physics world? A registered collision object has a
       broadphase handle and an unregistered one does not, so this asks Bullet
       rather than tracking a flag that could drift out of step with the world
       -- which is how a rebuild once deleted a REGISTERED body and left a
       dangling proxy that crashed the next updateSingleAabb. */
    bool hullInWorld() const;

    btCompoundShape *compoundShape() const;

    /* btTransform <-> (glm position, glm rotation). Always through a
       quaternion: btMatrix3x3 is row-major (m_el[i] = row i) and glm is
       column-major, so copying elements silently transposes (the trap
       physics.cpp's GetOrient / setPosRot comments warn about). */
    static btTransform toBt(const glm::dvec3 &pos, const glm::dmat3 &rot);
    static void fromBt(const btTransform &t, glm::dvec3 &pos, glm::dmat3 &rot);

    /* Frame S in world coordinates: the hull's COM frame mapped back through
       `principal`. */
    void frameS(glm::dvec3 &pos, glm::dmat3 &rot) const;

    /* The COM in world coordinates, straight off the hull -- its transform
       origin IS the COM. O(1), where a mass-weighted walk over the parts is
       O(n), and it is the point Bullet actually rotates the ship about. */
    glm::dvec3 comPos() const;

    /* Place the whole ship: frame S at (sPos, sRot). One write to the one
       body, and every part's pose follows from its authored local pose.
       setPosRot zeroes both velocities (proceedToTransform), so callers that
       care set them after. */
    void placeShip(const glm::dvec3 &sPos, const glm::dmat3 &sRot);

    /* The same, addressed by the COM instead of by frame S's origin: the two
       differ by the COM's offset within S, which the caller should not have
       to know. */
    void placeShipAtCom(const glm::dvec3 &com, const glm::dmat3 &sRot);

    /* (Re)build the compound + the hull from the CURRENT part list: one child
       per part -- its own collision hull, referenced not copied (the parts
       outlive the compound) -- at its authored pose in S, then re-based into
       the principal frame. Frame S and the velocity are carried across, so a
       rebuild neither teleports nor stops the ship. */
    void rebuildCompound();

    /* The centre of mass of the CURRENT part masses, in frame S -- i.e. what
       principal.getOrigin() would be after a rebuild. One pass over the
       parts and no allocation, so it is cheap enough to ask every tick. */
    glm::dvec3 compoundCom() const;

    /* The true mass COM minus the hull's transform origin, in world axes.
       The two coincide right after a rebuild, but a burn shifts the true COM
       (fuel leaves the tanks) while the origin stays put until the next
       refreshCompound threshold trips -- so this is generally nonzero during
       a burn. Bullet rotates the hull about its transform origin, so any net
       force acting through that offset origin adds a spurious
       (comOffset x F) torque the ship's true COM does not feel; the force
       laws subtract it (see applyGravity / applyThrustForce). */
    glm::dvec3 comOffset() const;

    /* Rebuild once the mass distribution has moved enough to matter. A burn
       shifts the COM and the total mass continuously, and a rebuild walks
       every part's hull inertia, so one per tank draw per tick would be pure
       waste -- but never rebuilding is wrong twice over: the children are
       re-based through principal, so a stale COM displaces every hull that
       picking and collision read, and a stale total mass is a stale
       acceleration. Called once per ship per tick, so one call site covers
       every source of a mass change (a burn, crew aboard, crew out). */
    void refreshCompound();
    /* metres in frame S, and a fraction of the ship's mass: both far under
       anything the game reads a pose or an acceleration to, and far over what
       one tick of a burn produces. */
    static constexpr double kComRebuildTol = 0.01;
    static constexpr double kMassRebuildFrac = 1e-3;

    /* The compound must reproduce the assembly it was built from. Two
       invariants, both recomputed here independently from the same authored
       data (the analytic parallel-axis form test_inertia pins getInertia()
       against):

       a) MASS PROPERTIES -- the centre of mass, and the inertia tensor about
          it, against what Bullet's calculatePrincipalAxisTransform produced.
          A transposed principal basis, or a COM taken about the wrong point,
          fails here.
       b) CHILD POSES -- each re-based child, taken back out to S through
          `principal`, is that part's authored pose. A re-base written the
          wrong way round, or skipped, leaves the collision hulls displaced
          from where the game thinks the parts are -- and `principal` by
          itself is consistent either way, because it is computed from the
          shape BEFORE the re-base. This also pins compoundParts[i] to child
          i, the mapping a collision hit is resolved through.

       Neither needs live physics state, so both run on every build, every
       staging event and every burn-triggered refresh -- in the unit tests and
       in the game. */
    void checkCompoundInvariants() const;

    /* The part frame S is anchored to: the one with no parent edge. That is
       build_ship's setRoot, and staging never drops it (a decoupler takes
       its child-side subtree, and dropping the root would drop the whole
       ship, which staging refuses). */
    Part *rootPart() const;

    /* --- part state accessors -------------------------------------------

       The one route to a part's pose, axes and velocity, and the only place
       that knows a part has no rigid body of its own. The pose is derived
       from the hull's transform through `principal` and the part's authored
       local pose; the velocity from the hull's -- the COM velocity plus
       omega x the offset, omega being the whole ship's, since a rigid body
       has one. Forces, mass and the render model still go through the Body. */
    void partWorldPose(const Part *p, glm::dvec3 &pos, glm::dmat3 &rot) const;
    glm::dvec3 partPos(const Part *p) const;
    glm::dmat3 partRot(const Part *p) const;
    /* the part's local axis n (0 = right, 1 = up, 2 = nose) in world axes */
    glm::dvec3 partAxis(const Part *p, int n) const;
    glm::dvec3 partVel(const Part *p) const;
    glm::dvec3 partAngVel(const Part *p) const;

    /* --compound-check: the ship's single-body state, per ship. Rebuilds the
       compound, which re-asserts that it still reproduces the part assembly
       it came from -- the invariant the whole representation rests on -- then
       reports the body the game is actually simulating: mass, COM, speed,
       spin, and the principal inertia diagonal -- the denominator of a
       reaction wheel's authority and of the autopilot slew law, so the number
       worth watching when a ship stops turning the way it used to. There is
       no derived-vs-live pose error to report any more: a part has no rigid
       body of its own to disagree with, which is the point of the change. */
    void compoundCheck(double time);

    /* Fuel links (see PartDef.fuel_link): one-way fuel connections between
       fuel groups. `from` -> `to` means fuel flows from `from`'s group to
       `to`'s group (the engine in `to`'s group can draw fuel from `from`'s
       group). Virtual -- no physics. Populated in build_ship (from the
       def's fuel_link parts), dropped when a stage splits (when either
       endpoint is removed). */
    struct FuelLink { Part *from; Part *to; };
    std::vector<FuelLink> fuelLinks;

    /* --drain-log state: the last sample's per-group total fuel mass +
       time, so the next sample can print the drain rate (kg/s) -- the
       change in a group's mass between two samples. */
    std::map<int, double> drainPrevMass_;
    double drainPrevTime_ = 0.0;

    /* Electrical state (powerTick, per substep): the gate for the reaction
       wheels (attitude control) -- true = the ship can draw power. Set by
       powerTick before applyControlForces each substep; default true so a
       ship with no EC system is ungated until its first tick. */
    bool powered_ = true;

    /* Rails: an idle ship in free fall coasts analytically on its two-body
       conic instead of being integrated: its welds and rigid bodies are
       parked out of the Bullet world and the rigid cluster's pose is
       re-derived from the conic every tick (attitude frozen inertially,
       like a torque-free body). Exact at any time accel, zero solver
       cost. While coasting, ship->frame is the SOI body's INERTIAL frame
       node (where the trajectory is a conic). A grounded ship instead
       FREEZES: same parking, but the pose stays static in the rotating
       surface frame (railFrozen) -- that is what enables rails warp with
       pad ships aboard. */
    bool onRails = false;
    bool railFrozen = false;    // grounded park: no conic, pose fixed in the
                                // (rotating) frame -- the planet's spin is
                                // carried by the render-frame transform
    glm::dvec3 rail_pos;      // m, cluster COM in ship->frame coords
    glm::dvec3 rail_vel;      // m/s, inertial, ship->frame coords
    glm::dmat3 rail_orient = glm::dmat3(1.0); // cluster axes -> frame axes
    /* Frame S's axes at park time. rail_pos is the COM and rail_orient
       carries these into the frame the pose is written in, so the hull's
       orientation is rail_orient * railRot and every part's pose follows
       from its authored local pose. The ship is rigid, so there is nothing
       per-part left to snapshot. */
    glm::dmat3 railRot = glm::dmat3(1.0);

    /* Per-part catalog spec / stage / tank contents / armed thrust / behavior
       all live on each Part now (see part.h). The old partDefs / partStages /
       partResources / m_thruster* / m_reaction_wheels / m_wheel* / m_armed*
       vectors -- all "parallel to parts" -- are gone: a Part carries its own
       def + stage + resources, and thruster/wheel behavior is derived from
       the def, so there is nothing to keep in sync or rebuild. */

    float thruster_util = 1.0;
    double exhaust_scale = 1.0;  // test knob (Settings / --exhaust-scale):
                                 // scales ve, so thrust and delta-v scale with
                                 // it (the fuel burn does not); synced per tick

    /* Rotation is armed once per tick (Command) and executed per SUBSTEP
       (applyRotationForce, before every stepSimulation) -- like thrust,
       because Bullet clears the accumulated torque on each stepSimulation.
       stick: the manual command -- x = Q/E (roll about the ship's nose),
       y = W/S (pitch), z = A/D (yaw); +-1 per axis, diagonals allowed
       (e.g. W+A). Each component drives the ship's own body axis -- see
       applyRotationForce for the exact mapping. slew: the autopilot target
       (exclusive). */
    float stick[3] = {0.0f, 0.0f, 0.0f};
    int slew = SlewNone;
    glm::dvec3 lastThrustForce{};  // debug: total thrust force applied last substep
    /* The autopilot mode the Autopilot window has engaged (its toggle
       buttons). Persistent across ticks, unlike `slew` (cleared each tick):
       the logic tick re-applies it after clearRotCmd(), so the ship keeps
       slewing toward the target and holding until the mode is toggled off. */
    SlewMode slewRequest = SlewNone;
    void setSlewRequest(SlewMode m);

    void setRoot(Part *part);

    /* Hang `part` off the part at `parentIdx` and record its authored pose in
       the ship-local frame S (Part::localPos/localRot; S is the root's frame,
       so these are pure geometry -- see part.h). `parent` is the topology edge
       the staging + fuel-group walks use. There is nothing to weld: the ship
       is ONE rigid body, and the local pose is what puts this part's hull at
       the right place inside it. */
    void attach(Part *part, size_t parentIdx,
                const glm::dvec3 &localPos, const glm::dmat3 &localRot);

    /* The three convenience attach modes (used by the --radial-test ship
       builder; build_ship goes through attachPose + attach directly). Each
       derives the child's ship-local pose from the parent's. */

    void attachDown(Part *part);

    void attachRadial(Part *part);

    void attachSide(Part *part);

    void init();

    /* init() minus the tank re-seed: the bookkeeping that finalizes a ship
       whose part list is already final -- the controller fallback, the stage
       readout, the fuel groups and the compound. init() calls it after
       seeding; extractSubtreeAsShip() calls it directly (the extracted
       parts carry their current tank contents). */
    void finalize();

    /* Put the ship's one rigid body into the physics world. Kept apart from
       init(), which builds it, for the reason above. */
    void enterWorld();

    /* True of the EVA kerbal (src/eva.h): control input, the camera and
       the event dispatch branch on this. Everything else is inherited --
       a kerbal is a one-part ship as far as frames, rails, gravity, the
       fleet and the HUD are concerned. */
    virtual bool isEva() const;

    /* True while an EVA character is ABOARD a ship (parked inside a capsule,
       out of the physics world) -- the render pass skips it (it is inside
       the capsule, not a visible body). Overridden in src/eva.h; a regular
       ship never carries this, so it is false by default. */
    virtual bool isCrewAboard() const;

    /* The crew's capsule slot is an index into the ship's part list, and a
       merge (absorbShip) or a split (extractSubtreeAsShip) reindexes the
       list. A crew member whose capsule moved with `dest` applies the
       old->new index map to its slot and returns true (so the caller moves
       it into dest's crew); one whose capsule stayed returns false. Only a
       Kerbal (eva.h) has a slot to reindex, so the base is a no-op. */
    virtual bool crewRebase(Vehicle *dest, const std::map<size_t, size_t> &reindex);

    /* Assign each part a fuel-group id (Part::fuelGroup). A fuel group is a
       connected component of the part tree across the parts that CONDUCT
       fuel; a part with PartDef::fuel_barrier (a decoupler) is a WALL that
       splits the groups, so an engine never draws fuel from across it.
       Barrier parts keep fuelGroup = -1 (they are in no group). This is the
       base undirected grouping; a fuel link, when added, will bridge groups
       one-way inside fuelPool(), leaving this the same. Recompute after
       a stage split -- the tree shrinks when parts drop. */
    void buildFuelGroups();

    /* The tanks an engine may draw fuel from: the tanks in its fuel group
       (buildFuelGroups) -- its connected neighbours, never across a fuel
       barrier (a decoupler). This is the single place a fuel link will later
       change (to "tanks reachable via directed fuel edges"), so the drain
       logic below stays put. */
    std::vector<Part *> fuelPool(Part *engine) const;

    /* The fuel groups an engine can draw from, in LAYERS by hop distance,
       furthest layer first. layers[0] holds the furthest groups,
       layers[last] is the engine's own group (distance 0); the groups
       WITHIN a layer are all the same distance out. With no fuel links
       there is a single layer {G} -- identical to the old behavior.
       Groups in a layer drain TOGETHER (pro-rata, consumeResourceMass):
       that is what keeps a symmetric star (two radial arms, both one hop
       out) draining symmetrically instead of one arm before the other. It
       generalises the chain rule -- C->B->A, D->E->A drains {C,D}
       together, then {B,E}, then A -- because the layers are exactly the
       hop-distance levels. */
    std::vector<std::vector<int> > fuelDrainLayers(Part *engine) const;

    /* Draw `amt` kg of `type` from the engine's fuel sources, LAYER by
       LAYER (fuelDrainLayers: furthest layer first) and pro-rata across
       ALL the tanks in a layer. Pro-rata, NOT first-tank-first or
       first-group-first: draining one tank (or one arm's tanks) to empty
       before its siblings shifts the ship's mass distribution and torques
       it under thrust (the radial-tank spin); shares proportional to each
       tank's contents keep a symmetric cluster draining together. The
       layering is the symmetry that matters: the two radial arms of
       heavy_two are one layer and split the flow, and a chain C->B->A,
       D->E->A is three layers {C,D}, {B,E}, {A} -- C,D first, then B,E,
       then A. Returns true if the total covers amt (else the thruster
       doesn't fire this tick). amt is the kg consumed THIS tick (the
       caller scales the kg/s flow by the tick's simulated time). */
    bool consumeResourceMass(enum ResourceType type, float amt /* kg */, Part *engine);

    float getFuelMass(const std::vector /* eh */ <enum ResourceType>& types);

    float getDeltaV();

    /* TODO should be cached per frame */
    float getMass();

    /* --- electrical (KSP-style EC) ---------------------------------------
       The ship's EC is a shared pool across its battery parts (a part is a
       battery when capacity[EC] > 0 -- the capsule and the battery parts);
       the charge lives in their resources.current[EC], like propellant.
       powerTick runs once per substep, BEFORE applyControlForces, and does
       two things:
         1. gate  -- the reaction wheels (attitude control) draw power, so
            they work only if the ship can supply it. Life support (the
            constant draw) has priority: the wheels need power left over
            (excess generation) or stored charge. A ship with NO EC system
            at all is ungated -- its wheels work as before (no regression).
         2. balance -- generation (RTGs) charges the pool; the constant draw
            (life support) and the active draw (the wheels, only while they
            are commanded) drain it.
       Units: power in W, charge in Wh (1 Wh = 3600 J), so W over h seconds
       is W*h/3600 Wh. EC has no mass -- draining/charging never touches a
       part's mass (unlike propellant). */
    void powerTick(double h);

    /* Drain up to `wh` of EC from the pool, pro-rata across the batteries
       by their current charge. Clamped to what is stored (never below 0).
       No mass change: EC is energy, not a substance. */
    void drainEC(double wh);

    /* Charge the pool by up to `wh`, pro-rata across the batteries by their
       free capacity. Clamped to the capacity (never above it). No mass
       change. */
    void chargeEC(double wh);

    /* Total EC charge / capacity across the pool (the HUD + --power-log). */
    void getPower(double *gen, double *constDraw, double *charge, double *capacity);

    /* --power-log: the ship's power balance + pool + gate, one line per
       sample (the "is the ship losing power?" instrument). */
    void power_log(double time);

    /* Staging state. `activeStage_` is a monotonic stage COUNTER (the stage
       about to be triggered): it starts at 1 and advances by one on each
       stage press, whether or not that stage had a decoupler. This is what
       lets a part on a lower stage (the central engine) keep firing after a
       HIGHER-numbered part above it has already been triggered -- the old
       "lowest stage number on the ship" rule got stuck at 1 forever in that
       case. An engine fires once the counter has reached its stage
       (stage <= activeStage_) and then stays lit; a decoupler triggers
       (drops its child-side subtree) when the counter is at its stage.
       `totalStages_` is the highest stage number on the ship at build time,
       for the "stage X of N" readout. */
    int activeStage_ = 1;
    int totalStages_ = 1;

    /* The active stage (the counter, see above). */
    int activeStage();
    /* Advance to the next stage (clamped at the last one). Called once per
       stage press, after the current stage's decouplers have fired. */
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

    /* A part's mass changed OUTSIDE a burn (crew aboard, crew out). The burn
       path is picked up by refreshCompound's per-tick threshold; these are
       one-off and large, so rebuild now rather than wait for it. */
    void addPartMass(Part *p, double delta);

    /* The COM is the hull's transform origin -- a rigid body's transform IS
       its centre-of-mass transform -- so this is O(1), and it is exactly the
       point Bullet rotates the ship about. */
    const glm::dvec3& get_center_of_mass(void);

    glm::dvec3 applyGravity();

public:
    Vehicle();
    /* Tear the ship down in a safe order: unregister the one rigid body from
       the world, delete it (which frees the compound it carries), then delete
       the parts (which free their models and their collision hulls -- the
       compound only referenced those). goOnRails() already unregistered, so
       for a railed ship only the deletes remain. The onRails guard is
       LOAD-BEARING: Bullet's removeCollisionObject is not idempotent -- it
       reads the object's world-array index (which remove never resets to -1),
       so a second remove on an absent body can evict the WRONG collision
       object. */
    virtual ~Vehicle();

    glm::dvec3 processGravity();

    // Bullet clears all accumulated forces on every stepSimulation, so the
    // thrust -- like gravity -- must be re-applied before EVERY substep.
    // Applied once per tick it would only act during the first substep's
    // h seconds of the tick's n*h, cutting the delivered thrust to 1/n
    // (and n grows with time acceleration, so it got worse at warp).
    void applyThrustForce();

    /* The armed control forces, re-applied before EVERY substep (Bullet
       clears forces per stepSimulation). Ships deliver thrust + rotation +
       RCS translation; the EVA kerbal overrides with its own laws
       (src/eva.h) and does not call this. */
    virtual void applyControlForces(double h);

    /* the first reaction-wheel PART (nullptr if the ship has none): the
       stick / slew / kill-rot laws all use it as the ship's attitude
       reference. Replaces the old m_reaction_wheels.front(). A Part, not a
       Body, so the reads below go through the part accessors like every
       other consumer of a part's state. */
    Part *firstWheel();

    // The armed rotation commands -- like thrust -- are re-applied before
    // EVERY substep (h = that substep's duration); applied once per tick
    // they would act only during the first substep, cutting the delivered
    // authority to 1/n and making it worse at warp.
    void applyRotationForce(double h);

    /* --- RCS translation (hydrazine mono, KSP-style) --------------------
       Field-driven (Part::isRcs): a part with rcs_thrust > 0 contributes
       that many newtons of translation authority; maxRcsThrust() is the
       ship's total. The armed direction rcsDir is SHIP-RELATIVE: components
       in the ship's own body axes (x = right, y = up, z = nose), armed once
       per tick through Command (the held RCS slots, tick.cpp) -- the same
       local-axes pattern as the stick -- and consumed before every substep:
       a fixed thrust for as long as the key is held AND the ship can draw
       this substep's flow of hydrazine (consume-then-arm, the EVA suit's
       pattern -- no speed cap, the propellant is the limiter). Diagonals
       (two axes held) compose as a vector sum and are normalized at the
       point of use (rcsWorldDir), so the authority is the same however many
       axes are held. The force is applied AT the COM (ApplyCentralForce):
       the net force accelerates the whole ship regardless of where the
       thrusters sit, which is the COM-translation approximation (a real
       positioned-thruster build swaps this one line for ApplyForce at each
       part -- rcsDir and the rcs_thrust field stay the same). */
    glm::dvec3 rcsDir = glm::dvec3(0.0);  // armed dir in ship axes (right, up, nose); 0 = off
    /* burned this tick (applyRcsForce drew hydrazine): the render pass
       draws the COM plume off this, same armed/disarmed pattern as
       m_thrust + armedThrust for the engines (cleared by clearRcs, the
       per-tick disarm in tick.cpp). */
    bool rcsFiring = false;
    void clearRcs();
    /* The armed RCS direction in WORLD axes (unit; (0,0,0) when disarmed):
       rcsDir's ship-body components mapped through the root part's axes.
       The root's local frame IS the ship frame S, and its nose is the same
       axis the attitude law slews (att_log). Resolved at the point of use,
       so the direction tracks the ship's live attitude, substep by
       substep -- not a camera basis sampled once per tick. */
    glm::dvec3 rcsWorldDir() const;
    Part *firstRcsPart();
    double maxRcsThrust();
    void applyRcsForce(double h);
    /* s, monopropellant (hydrazine) efficiency -- the EVA suit's value. */
    static constexpr double kRcsIsp = 220.0;

    /* Autopilot diagnostic (throttled; called from the tick when
       --slew-log is set). Prints the slew error angle, the ship's angular
       velocity DECOMPOSED into the slew axis / nose-roll / the third axis
       (so an uncontrolled spin shows up as nonzero roll/third even while
       the slew-axis rate is being driven to zero), and the braking-curve
       rate the law wants right now. This is the instrument for hunting the
       prograde wobble: watch E and w_slew for a sustained oscillation, and
       roll/third for a residual spin the law is not killing. */
    void slew_log(double time);

    /* --att-log: the ship's nose (local +Z of the hull, world coords) and
       its angular velocity (world coords), for the attitude-physics e2e test.
       One rigid body, so there is exactly one of each: the root part's +Z is
       frame S's, and the spin is the hull's. (A reaction wheel used to have
       to be avoided here -- under the welds it spun relative to the hull.) */
    void att_log(double time);

    /* --tq-log: the spurious-torque bug class on one line, once per tick.
       dcom is how far the hull's transform origin (Bullet's rotation
       centre) lags the true mass COM; F is the net force the ship feels
       right now (per-part gravity + rotating-frame fictitious + armed
       thrust -- the same terms applyGravity / applyThrustForce apply);
       |dcom x F| is the torque those origin-referenced levers would add
       if uncorrected -- the amount the fix cancels. Expect |dcom x F| to
       be sizable around any burn: the COM drifts between refreshCompound
       re-centers (here |dcom| ~1e-4..7e-4 m, |dcom x F| up to ~15 N m),
       and after the burn the last lag stays frozen (no more mass change
       -> no more recentering) and keeps acting -- in the rot-orbit
       scenario it sits at ~5e-4 m x 2.4e4 N = ~13 N m indefinitely. The
       discriminating signal is |w|: healthy stays ~1e-6 rad/s while
       |dcom x F| is sizable; the regression is |w| ramping (pre-fix, the
       frozen lag x the ~24 kN net force grew it ~1.3e-3 rad/s per second).
       Stateless: re-derives the forces from the current state, so there
       is nothing to reset. */
    void tq_log(double time);

    /* --fuel-log: each fuel group's fuel mass (per resource), the
       per-tank breakdown, and the fuel links -- the instrument for the
       fuel-link drain-rate bug: two symmetric radial groups must show
       equal mass at every sample, so a one-before-the-other drain shows
       up as the two group lines diverging while the ship spins. */
    static const char *resourceName(int r);

    void fuel_log(double time);

    /* --drain-log: the thrust delivered this tick (N) + each fuel group's
       drain rate (kg/s) -- the change in the group's total fuel mass (sum
       over its tanks' resources) between consecutive samples. The "how is
       the fuel flowing" instrument: a symmetric asparagus shows the two
       outer groups draining at the same rate and every inner group at 0,
       so a serial or lopsided drain shows up as the rates diverging (or a
       sink touched early). The first sample only records the baseline (no
       rate); from the second on the rate is the interval average. The
       thrust is the sum of the parts' armedThrust -- ApplyThrust arms a
       part only if its flow was covered this tick, so it is the thrust
       actually delivered (an engine whose layers ran dry is 0, not its
       rating). */
    void drain_log(double time);

    /* the largest wheel's rated torque (N m) -- the per-wheel rating for
       the HUD; the ship's TOTAL wheel authority is maxTorque() (the sum) */
    float GetWheelTorque();


    /* disarm the armed thrust (called once per tick, like clearRotCmd,
       so a tick without the keys doesn't keep firing) */
    void clearThrust();

    /* disarm the armed rotation commands (called once per tick, so a tick
       without the keys doesn't keep rotating) */
    void clearRotCmd();

    /* called when control moves to ANOTHER ship: zero the throttle and
       clear the armed thrust + rotation commands, so this ship just
       coasts under its own physics from here on (no residual forces,
       no fuel flow). Control input reaches only the active ship. */
    void releaseControl();

    /* The parts that WOULD be dropped if `stage` is triggered: each
       decoupler on that stage plus the child-side subtree it anchors. The
       child side is the parts attached BELOW the decoupler (away from the
       root/capsule) -- its direct children and their subtrees. The
       decoupler itself IS dropped (it flies off with the stage, like a KSP
       separator -- otherwise it dangles under the surviving engine). This
       is what makes staging scope the deletion to ONE side of an
       attachment: a sibling branch (e.g. the central engine, a child of the
       central tank) is untouched even though it shares a stage with the
       booster the decoupler drops. Empty if no decoupler is on that stage. */
    std::vector<Part *> droppedPartsAtStage(int stage);

    /* --- docking ----------------------------------------------------------

       A dock joins two ships into ONE rigid body: this ship (the survivor,
       always the active one -- Game::updateDocking) absorbs the other.
       The absorbed ship's parts are rebased from its frame S_B into this
       ship's S, its root part is reparented under this ship's port part
       (the part-tree edge), and this ship is rebuilt as the union. The
       joint is recorded as a seam, so an undock (extractSubtreeAsShip)
       undoes exactly this.

       Both ships are rigid bodies, so the merge is a pure rigid rebase:
       the absorbed parts keep their exact relative geometry in S -- only
       their coordinates in S and their one tree parent change. The merged
       velocity is the inelastic (mass-weighted) average of the two, and
       the angular velocity is the survivor's (a rigid body has one).

       After the call the absorbed ship is an empty shell: its parts, fuel
       links and crew have moved into this ship (its hull is left, so its
       dtor can unregister it from the world). The caller (the Game layer)
       removes it from the fleet list, nulls any selection pointing at it,
       and deletes it.

       Precondition (Game::updateDocking's job): both ships are live (not
       on rails), in the same frame, and their ports are close, aligned
       and slow (the capture test). */

    /* One dock seam: the tree edge that joins the two ships. `port` is
       THIS ship's port part (the parent of the joint), `root` the absorbed
       ship's root part (the child). `name` is the absorbed ship's display
       name, restored when the seam is undone. */
    struct DockSeam {
        Part *port;
        Part *root;
        std::string name;
    };
    /* The docks this ship has absorbed, in order (undock pops the last). */
    std::vector<DockSeam> seams;

    /* Docking INTENT (held PER SHIP, not on Game, so a future AI-controlled
       ship can dock under its own steam). A dock needs BOTH halves set:
       - dockTargetShip / dockTargetPort: the port on ANOTHER ship this ship
         wants to mate with (right-click it -> "Target for docking").
       - dockArmPort: which of THIS ship's own docking ports does the mating
         (right-click it -> "Arm for docking"). Mandatory, so a ship with
         several ports uses the one the player picked.
       Game::updateDocking only docks when both are set and clears them on
       success -- so an undock cannot immediately re-dock (the player has to
       re-arm and re-target). The target pointers are validated and dropped
       when the target ship/port goes away (see updateDocking and the cleanups
       where a ship is deleted); dockArmPort points at this ship's own part
       (no cross-ship dangle) and is validated the same way. */
    Vehicle *dockTargetShip = nullptr;
    Part *dockTargetPort = nullptr;
    Part *dockArmPort = nullptr;   // this ship's port to mate with (mandatory)

    void absorbShip(Vehicle *B, Part *portA);

    /* Extract a connected subtree (rooted at `root`) into a new Vehicle:
       the general "a part of this ship becomes a ship" primitive.

       The dropped parts keep their exact relative geometry, rebased into a
       new frame S' = the root's old frame (origin at the root's position in
       S, axes the root's orientation in S -- the same root-frame rule
       build_ship uses, so the new ship's root part has identity pose). The
       new ship inherits this ship's frame/home/sun (the split is local),
       is placed at the root's current world pose, and given the rigid
       velocity of the dropped side's COM (this ship is one rigid body, so
       that point moves as v + w x r). This ship is left with the
       survivors, rebuilt.

       Returns nullptr if `root` is not part of this ship or would drop the
       whole ship (callers refuse that). The new ship is NOT yet in the
       fleet list NOR the physics world -- the caller enters it into the
       world (enterWorld) and adds it to the SoI body's ships and, if it
       came from a seam, pops that seam. Keeping the world registration in
       the caller lets the split run headless (no physics world), like the
       fuel/power tests build ships without enterWorld.

       Undock (Game::undock) is the first user: the dropped side is the
       subtree under the most recent seam's root. Staging's "dropped stage
       becomes a ship" is the second: the same call with the stage's
       subtree (its decoupler root), instead of deleting it. */
    Vehicle *extractSubtreeAsShip(Part *root, const std::string &name);

    /* This ship's part frame -> renderFrame. Usually the identity
       (renderFrame is this ship's own frame); an idle ship that switched
       SOI while another ship was being controlled lives in a different
       frame, so transform its parts into the render frame first. Draw
       uses it to bring the parts into the view; picking (src/pick.cpp)
       inverts it to bring the ray into the parts' frame. */
    glm::dmat4 renderXform(Frame *renderFrame) const;

    void Draw(const Camera* camera, Frame *renderFrame);

    // Single place to control the ship. While paused (simActive == false)
    // every command is dropped, so nothing accumulates in the rigid bodies
    // (a force/torque left in Bullet would dump out as a velocity kick on
    // resume) and settings like throttle stay frozen.
    /* step = the tick's simulated duration (dt * time_accel); only Thrust
       uses it (to scale this tick's fuel flow). */
    void Command(ShipCmd cmd, bool simActive, double step = 0.0);

    /* The COM velocity -- get_center_of_mass() is the COM position, so the
       orbit elements are now fitted to one point's state rather than the
       controller part's velocity at the cluster's COM. */
    glm::dvec3 GetVel();

protected:
    // Control implementation: applies forces/torques to the Bullet bodies
    // directly, so it is reachable only through Command() above.
    // (protected, not private: the EVA kerbal (src/eva.h) reuses the
    // rotation-model helpers below for its own attitude law.)
    void adjustThrottle(float delta);

    /* the ship's full-throttle thrust RIGHT NOW (N) = the sum of every
       engine that has already been ignited (stage <= the stage counter) of
       its full thrust (each T = (H2 + LOX flow) x ve = 2 x fuel_rate x ve,
       both propellants end up in the plume), scaled by exhaust_scale (the
       test knob). Engines stay lit once ignited, so this is the sum of all
       lit engines on the ship; for a single-stage ship it equals the grand
       total. */
    float GetActiveThrust();

    /* Called once per physics tick (step = the tick's simulated duration).
       Consumes the tick's fuel and arms the per-thruster thrust; the force
       itself is applied by applyThrustForce() before EVERY substep below.
       A thruster that can't consume its flow this tick doesn't thrust.
       Every engine that has already been ignited (stage <= the stage
       counter) fires, and each draws its OWN fuel group's tanks (see
       fuelPool) -- so an engine keeps burning from its connected propellant
       until it runs dry or its tanks are dropped. Stage gates WHEN it
       ignites; the fuel group (connection) decides WHAT it burns. */
    void ApplyThrust(double step);

    // --- physical rotation model (private law implementation) -------------
    // The reaction wheel is rated at GetWheelTorque() N m -- the most torque
    // it can apply to the ship -- so the ship's angular authority is
    // alpha = maxTorque() / I (rad/s^2) with I the ship's total moment of
    // inertia (kg m^2, from Bullet). Stick, prograde/retrograde slew and
    // kill-rot all work within that authority, so no command can be more
    // forceful than a maxed manual stick. (The thrust analogue: T = mdot*ve.)

    double maxTorque();

    /* The ship's moment-of-inertia tensor about its COM, in world axes: the
       one rigid body's own. Bullet stores it DIAGONAL in the principal frame,
       so rotating it out by the body's basis gives the tensor -- the same
       parallel-axis assembly this used to be built from by hand, already done
       by calculatePrincipalAxisTransform and held against that assembly by
       checkCompoundInvariants. O(1) rather than O(parts), and this is read
       every substep by the slew and kill-rot laws. */
    glm::dmat3 getInertia();

    /* The target direction (in the ship's frame) for the current directional
       slew mode. Radial / normal reference the SOI body: its center is the
       frame origin, so `pos` is the radius vector and `vel` the velocity --
       radial is the radius vector, normal the orbital angular-momentum
       direction r x v. The same convention as the navball indicators in
       render.cpp. KillRot / None return zero (slewToward refuses a
       zero-length direction). */
    glm::dvec3 slewTargetDir();

    /* Slew the nose (local +Z) toward `dir` within the wheel's authority:
       the target rate is the braking curve sqrt(2*alpha*E) -- the fastest
       rate from which the ship can still stop exactly at the target
       (E = the error angle) -- capped at E/(2h) so no substep can cross
       the target, and the per-substep rate change is bounded by alpha*h,
       so the command never exceeds a maxed manual stick. */
    void slewToward(glm::dvec3 dir, double h);

    /* Kill the spin within the wheel's authority: each axis' rate drops by
       min(|w|, alpha*h) per substep -- monotonic, no sign flip, never more
       forceful than a maxed manual stick. No deadband: the law is
       proportional, so it converges to exact zero. A fixed |w| cutoff would
       strand a residual spin whenever the per-substep authority alpha*h is
       smaller than the cutoff -- heavy ships (e.g. docked stacks) damp
       linearly into the cutoff and then keep drifting forever. */
    void killRotStep(double h);

public:

    /* A part's position in another frame's coordinates. Takes the Part, not
       its Body, so the read goes through the part accessors like every other
       consumer of a part's state. */
    glm::dvec3 GetPositionRelTo(const Part *part, Frame *relTo);

    void moveToFrame(Frame *newFrame);

    /* Per-tick SOI bookkeeping for THIS ship: if the ship is outside the
       current frame's SOI, move to the parent frame; else if it has
       entered a child's SOI, move to the nearest such child. Called once
       per tick, per ship (the frame tree is shared; each ship tracks its
       own position in it). */
    void switchFrames();

    /* Write the rail state into the ship's body (once per tick). Draw,
       get_center_of_mass and everything else that reads the body then sees
       the railed ship's current pose even though it is not in the world.
       Angular velocity is zeroed: a parked ship is torque-free, and readers
       like --orbit-log and the HUD fit their elements to consistent data. */
    void writeRailPose();

    /* The ship's COM state in `inertial` -- the frame node where its
       trajectory is a Kepler conic (the same transform the HUD uses). The
       ordering matters: the OLD frame's stasis (rotation) velocity is added
       before rotating, and the result is offset by the frame's own velocity
       in the inertial node. Getting it wrong biases every conic fitted from
       here. */
    void comStateIn(Frame *inertial, glm::dvec3 &p, glm::dvec3 &v);

    // Separation between this ship's COM and another's, in the universe (root)
    // frame. The root is shared by every body in the system, so expressing
    // both COMs there gives a frame-invariant distance, independent of the SOI
    // each ship is currently tracking.
    double distanceTo(Vehicle *o);

    /* The COM's osculating orbit dips into the terrain band (periapsis
       within 3 km of the surface): sitting on / skimming the ground rather
       than coasting clear of it. */
    bool inTerrainBand();

    /* Rails classification: a FLYING ship (periapsis clear of the terrain
       band) coasts on its conic; a GROUNDED one (periapsis inside the
       band) can only freeze in its rotating surface frame. Anything else
       -- e.g. a suborbital descent -- is not rail-eligible. */
    bool canRail();

    /* Park this ship out of the physics world and coast it analytically.
       Refuses (returns false) and changes nothing if the ship is not
       rail-eligible (see canRail). Flying ships follow their conic in the
       body's inertial node; grounded ships freeze in the rotating surface
       frame. */
    bool goOnRails();

    /* Re-enter physics from rails: rebuild the Bullet state from the rail
       state and hand the ship back to the integrator. Pose and velocity
       already track the rail state (writeRailPose), so this is just
       re-register -- nothing to re-weld, because a rigid body has no internal
       degrees of freedom and the parked geometry IS the authored geometry. */
    void leaveRails();

    /* Per-tick rail advance: propagate the conic by the tick's simulated
       duration (exact for any step size), check SOI boundaries, refresh
       the parked transforms. A frozen (grounded) ship has nothing to
       propagate: its pose is static in the rotating frame. */
    void railsTick(const double step);

    /* SOI bookkeeping for a railed ship (the switchFrames() analog): the
       rail conic is only valid around frame->body while the ship stays in
       that SOI. The rotating child frame is the same body -- never a
       switch candidate; physics ships drop into it after the handoff. */
    void railsSwitchFrames();

    /* Re-anchor the rail state on another frame (moveToFrame's math for
       the analytic state; the new frame is inertial, so no stasis). */
    void moveToRailFrame(Frame *newFrame);
};

// Forward declaration (system.h defines it); spawn_vehicle resolves the
// home body's SOI through the system's frame tree.
struct System;

/* Instantiate a ship def: one rigid body per part (mesh + texture from
   the catalog entry), welded parent-first in the def's construction
   order. GL is needed here (shader binding); the catalog must outlive
   the ship (the partDefs point into it). */
void build_ship(Vehicle *ship, const ShipDef &def, Shader *partsshader,
                const glm::dvec3 &base, const glm::dmat3 &orient);

/* Starting scenario (chosen at the CLI on startup, see main). The pad
   scenarios are already set up in main (the ship is built on the pad);
   the orbit scenarios place the ship in a circular orbit around the
   home body at r = radius + alt_frac * (rotating-frame SOI - radius),
   in the equatorial plane (local +Z) or the polar plane (local +Y),
   nose prograde. The ellipse-* scenarios place the ship on a 10 km x
   1000 km ASL orbit in the equatorial plane, prograde, at periapsis
   (ell_phase 0), apoapsis (1), or 90 deg of true anomaly (2). The escape
   scenario places the ship at the circular-orbit radius with esc_frac x
   the local escape velocity, prograde -- a hyperbolic trajectory that
   coasts out of the body's SOI on its own (no thrusting).

   The distance scenarios (neptune, oort) set abs_r instead: a circular
   orbit at an ABSOLUTE radius from the body centre, anchored to a real
   solar-system distance rather than a multiple of the home body's SOI, so
   the same name means the same distance around any body. Use them with
   --body Kerbol: around a planet the radius is still exact, but the spawn
   inherits the planet's own orbital velocity, so the ship is hyperbolic
   with respect to the star (ecc ~1.8 at neptune around Kerbin) rather than
   circular. They exist as precision test beds -- Kerbol's SOI runs out to
   1e16 m, and double precision (BT_USE_DOUBLE_PRECISION) degrades with
   distance:
     4.495e12 m (neptune)  ULP ~1.0 mm     float32 would be ~536 km
     1.000e15 m (oort)     ULP ~0.22 m     float32 would be ~1.2e5 km
   i.e. oort is roughly where a floating origin would start to matter for
   the physics itself, and neptune is comfortably inside double's range. */
struct ScenarioDef {
    const char *name;
    bool on_pad;
    double alt_frac; // circular: fraction of (rot-frame SOI - radius)
    bool polar;
    int ell_phase;   // -1: circular; 0: at periapsis; 1: at apoapsis; 2: at 90 deg
    double peri_alt; // ellipse: periapsis altitude above the body radius (m)
    double apo_alt;  // ellipse: apoapsis altitude above the body radius (m)
    double esc_frac; // escape: launch speed in local escape velocities (0 = not escape)
    double abs_r;    // > 0: absolute circular-orbit radius from the body
                     // centre (m), overriding alt_frac -- for scenarios
                     // anchored to a real distance (see above)
};

/* Look up a scenario by name; throws listing the available names if
   unknown. */
const ScenarioDef *scenario_by_name(const std::string &name);

// Orientation with the nose (local +Z) along `dir`; the roll axis is the
// coordinate axis most orthogonal to dir (never singular for a unit dir).
glm::dmat3 faceAlong(const glm::dvec3 &dir);

/* slot_offset (m): lateral separation for ships sharing a scenario --
   applied along the orbit binormal (perpendicular to both the radius
   vector and the velocity), so each ship's orbit stays essentially the
   same shape. 0 for a lone ship (and no-op for pad scenarios). */
void spawn_vehicle(Vehicle *ship, const ScenarioDef &sc, TerrainBody *home,
                   System &sys, double slot_offset = 0.0);

/* --radial-test spin diagnostics (two-part ship): per-part angular
   velocities, the INTERNAL contact torque between the two parts, and the
   (tidal) torque. */
void spin_log(Vehicle *ship, double time);
