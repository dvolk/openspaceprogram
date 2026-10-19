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

#include "flightlog.h" // FlightLog

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/norm.hpp>

#include "btcommon.h"
#include "body.h"
#include "physics.h"
#include "shipdef.h"
#include "part.h"
#include "terrain.h"
#include "frame.h"
#include "orbit.h"

// One ship-control command. Command() is the only control -> physics path.
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

/* Autopilot slew targets -- mutually exclusive (last one wins); the stick
   composes with them. */
enum SlewMode {
    SlewNone = 0,
    SlewPrograde,
    SlewRetrograde,
    SlewRadialOut,   /* align nose away from the SOI body (radius vector) */
    SlewRadialIn,    /* align nose toward the SOI body */
    SlewNormal,      /* align nose with the orbital-plane normal (r x v) */
    SlewAntiNormal,  /* align nose against it */
    SlewKillRot      /* kill the spin */
};

struct ScenarioDef;

class Vehicle {
public:
    std::string name;   // display name
    std::string defPath; // ship def it was built from ("" = test ship)
    std::vector<Part *> parts;   // each Part owns its Body (see part.h)

    // SoI placement: setSoi is the one writer. Null until placed (off-map).
    // m_parent is the AUTHORITATIVE "which body": the vehicles are one flat list
    // (Game::vehicles), so a same-body query filters this rather than reading a
    // body-side index that would be a second copy of the same fact.
    TerrainBody *m_parent = nullptr;
    Frame *frame = nullptr;
    // `home` is fixed at build; `slot` pads ships sharing a body+scenario.
    // Every Vehicle in Game::vehicles is FLYABLE: a seated kerbal is not in it
    // (and not a Vehicle at all) -- it is a seat on its capsule's Part (part.h).
    TerrainBody *home = nullptr;
    const ScenarioDef *scenario = nullptr;
    int slot = 0;
    TerrainBody *sun = nullptr; // the star (light source)
    float m_thrust;

    /* Mission journal (started by setSoi). Save-persisted: the load restores it
       before setSoi runs, so re-entering the same body is a no-op. */
    FlightLog flog;

    glm::dvec3 m_com;

    /* The cockpit (else the first wheel): the camera basis and stick frame come
       from its local axes. finalize() sets it, so it must start null -- the
       camera readers use it unguarded. */
    Part *controller = nullptr;
    /* --- the ship as ONE rigid body (btCompoundShape) -------------------

       One btRigidBody; collision shape a compound of the part hulls at their
       authored ship-local poses. No per-part bodies, no welds: Part::body is
       render model + collision hull + mass, nothing else. A part's pose is
       always DERIVED (partWorldPose); forces go to the one body.

       S is the ship-local frame of the authored poses (part.h): the root part's
       frame at build time. A rigid body's transform is its COM transform and its
       inertia is DIAGONAL, so the children are re-based into the principal frame
       and `principal` maps the two:

           hull transform      =  frameS() * principal
           a part's world pose =  hull transform * principal^-1 * T_S_part */

    /* The one rigid body, wrapped in a Body for the physics API. Its MASS is
       never written directly -- it follows from the parts via rebuildCompound.
       hull OWNS btBody and shape; its render assets are null. */
    Body *hull = nullptr;
    /* Coulomb friction, written by rebuildCompound() into every body it mints --
       other grip (eva.h) sets it once. */
    double hullFriction = kDefaultHullFriction;
    btTransform principal = btTransform::getIdentity();
    /* The compound's parts in child-index order (always `parts`). Indices are
       rebuild-scoped: map a hit back through this. */
    std::vector<Part *> compoundParts;

    /* The union of the part collision hulls, in frame S, reduced to extreme
       points; rebuilt with the compound. applyAeroForce projects it for the drag
       silhouette -- AREA is rotation-invariant, so only v̂ rotates into S. */
    std::vector<glm::dvec3> aeroHull;
    void rebuildAeroHull();

    /* In the physics world? Asked of Bullet, not a flag. */
    bool hullInWorld() const;

    btCompoundShape *compoundShape() const;

    /* Always through a quaternion: btMatrix3x3 is row-major and glm
       column-major, so copying elements silently transposes. */
    static btTransform toBt(const glm::dvec3 &pos, const glm::dmat3 &rot);
    static void fromBt(const btTransform &t, glm::dvec3 &pos, glm::dmat3 &rot);

    /* Frame S in world coordinates: the hull's COM frame back through
       `principal`. */
    void frameS(glm::dvec3 &pos, glm::dmat3 &rot) const;

    /* The COM: the hull transform's origin, in world coordinates. O(1). */
    glm::dvec3 comPos() const;

    /* Place the ship: frame S at (sPos, sRot). Leaves BOTH velocities alone. */
    void placeShip(const glm::dvec3 &sPos, const glm::dmat3 &sRot);

    void placeShipAtCom(const glm::dvec3 &com, const glm::dmat3 &sRot);

    /* (Re)build from the CURRENT part list. Frame S and the velocity carry
       across. */
    void rebuildCompound();

    /* COM of the CURRENT part masses, in frame S. */
    glm::dvec3 compoundCom() const;

    /* True mass COM minus the hull's transform origin, world axes. Nonzero in a
       burn (the origin lags the draining tanks). */
    glm::dvec3 comOffset() const;

    void refreshCompound();
    static constexpr double kComRebuildTol = 0.01;
    static constexpr double kMassRebuildFrac = 1e-3;

    /* The compound must reproduce the assembly it was built from (mass
       properties + child poses), recomputed independently. */
    void checkCompoundInvariants() const;

    /* The containment invariant (part.h): each part is attached to exactly this
       vehicle and the container/contents/seats edges agree. */
    bool checkPartInvariants() const;

    /* One attached part's containment subtree, depth-first -- the walk
       checkPartInvariants runs per part, because effectiveMass and ~Part recurse
       without limit and a violation below depth 1 is just as fatal. */
    bool checkContainmentTree(Part *p, int depth) const;

    /* The part frame S is anchored to: the one with no parent edge. */
    Part *rootPart() const;

    /* The shroud condition (PartDef.shroud): a part on p's EXHAUST face. Live,
       not cached: staging drops the child. */
    bool hasChildBelow(const Part *p) const;

    /* --- part state accessors: derived through `principal` from the hull's
       transform (a part has no body of its own). */
    void partWorldPose(const Part *p, glm::dvec3 &pos, glm::dmat3 &rot) const;
    /* The same pose relative to the hull COM: ship-local quantities only, so
       exact at any distance. */
    void partPoseRelCom(const Part *p, glm::dvec3 &pos, glm::dmat3 &rot) const;
    glm::dvec3 partPos(const Part *p) const;
    glm::dmat3 partRot(const Part *p) const;
    /* the part's local axis n (0 = right, 1 = up, 2 = nose) in world axes */
    glm::dvec3 partAxis(const Part *p, int n) const;
    glm::dvec3 partVel(const Part *p) const;
    glm::dvec3 partAngVel(const Part *p) const;

    /* --compound-check: rebuild + report the simulated body. */
    void compoundCheck(double time);

    /* Fuel links (PartDef.fuel_link): one-way group -> group. Virtual. */
    struct FuelLink { Part *from; Part *to; };
    std::vector<FuelLink> fuelLinks;

    /* Cached per group; buildFuelGroups clears them. */
    mutable std::map<int, std::vector<std::vector<int> > > drainLayers_;

    /* --drain-log state: last sample's mass + time, for the rate. */
    std::map<int, double> drainPrevMass_;
    double drainPrevTime_ = 0.0;

    /* EC gate for the wheels (powerTick). Default true: no EC system = ungated. */
    bool powered_ = true;

    /* Rails: an idle ship coasts on its two-body conic instead of being
       integrated -- its body is parked out of the Bullet world, its pose
       re-derived from the conic each tick. While coasting ship->frame is the SOI
       body's INERTIAL node; a grounded ship FREEZES (railFrozen). */
    bool onRails = false;
    bool railFrozen = false;    // grounded park: no conic, static in the
                                // rotating frame
    /* Initialised, not just declared: game.cpp and save.cpp set
       onRails + railFrozen without goOnRails(), so the rail pose can be
       unwritten -- leaveRails() would then place the hull at garbage. */
    glm::dvec3 rail_pos = glm::dvec3(0.0);   // m, cluster COM in ship->frame coords
    glm::dvec3 rail_vel = glm::dvec3(0.0);   // m/s, inertial, ship->frame coords
    glm::dmat3 rail_orient = glm::dmat3(1.0); // cluster axes -> frame axes
    /* Frame S's axes at park time (the ship is rigid: nothing per-part). */
    glm::dmat3 railRot = glm::dmat3(1.0);
    /* The instant rail_pos/rail_vel describe: must equal frame->rail_time when
       composed. */
    double rail_epoch = 0.0;

    float thruster_util = 1.0;
    double exhaust_scale = 1.0;  // --exhaust-scale: scales ve and all jet
                                 // thrust; synced per tick
    double drag_cd = 1.2;        // --drag-cd: drag coefficient (drag.h); 0 = no
                                 // drag. HULL silhouette only: chute Cd·A
                                 // (below) is a separate term.

    /* Last substep's aero (applyAeroForce), for the --drag-log. */
    glm::dvec3 lastAeroForce = glm::dvec3(0.0);   // total (lift + drag)
    glm::dvec3 lastLiftForce = glm::dvec3(0.0);   // the lift part
    glm::dvec3 lastAeroTorque = glm::dvec3(0.0);  // moment about the COM
    double lastDragAlt = 0.0;
    double lastDragRho = 0.0;
    double lastDragAlpha = 0.0;  // pitch angle of attack (rad)
    double lastDragArea = 0.0;  // silhouette facing the flow (m^2)
    double lastDragCd = 0.0;  // area-weighted mean of the parts' cds
    /* Deployed chutes' EFFECTIVE Cd·A (m^2) last substep (0 = none): nominal
       chute_area shrunk by kChuteAccelLimit at high q. NOT in lastDragArea. */
    double lastChuteCdA = 0.0;

    /* Last substep's control-surface deflections (--drag-log). */
    struct ControlDeflection {
        const PartDef *def;  // part type + control_axis
        int index;           // Nth control surface, 0-based
        double deflection;   // rad, signed hinge angle
    };
    std::vector<ControlDeflection> lastControlDeflections;

    /* Armed once per tick (Command), run per SUBSTEP -- Bullet clears the
       accumulated torque each stepSimulation. stick: manual; slew: autopilot. */
    float stick[3] = {0.0f, 0.0f, 0.0f};
    int slew = SlewNone;
    glm::dvec3 lastThrustForce{};  // debug: total thrust force applied last substep
    /* The Autopilot-window mode: persistent, unlike `slew`. */
    SlewMode slewRequest = SlewNone;
    void setSlewRequest(SlewMode m);

    void setRoot(Part *part);

    /* Hang `part` off `parentIdx` with its authored pose in frame S. Low-level:
       the pose is already solved -- see attachMode(). */
    void attach(Part *part, size_t parentIdx,
                const glm::dvec3 &localPos, const glm::dmat3 &localRot);

    /* Solve `part`'s ship-local pose off `parentIdx` with attachPose() (stack
       modes; surfaces use attachSurface()). */
    void attachMode(Part *part, size_t parentIdx, AttachMode mode,
                    double angleDeg = 0.0, double offset = 0.0);

    /* attachMode() against the most recently added part. */
    void attachDown(Part *part);

    /* Surface-attach `part` at a contact `point` and outward `normal`, in the
       parent's local frame. */
    void attachSurface(Part *part, size_t parentIdx,
                       const glm::dvec3 &point, const glm::dvec3 &normal,
                       double rollDeg = 0.0, double offset = 0.0);

    void init();

    /* init() minus the tank re-seed, for a ship whose part list is final
       (extractSubtreeAsShip() calls it directly). */
    void finalize();

    /* Enter the physics world. Separate from init() so a headless caller can
       build without one. */
    void enterWorld();

    /* True of the EVA kerbal (src/eva.h). A kerbal that is SEATED is not a
       Vehicle at all, so every Vehicle here is free: there is no aboard state to
       ask about (part.h: the seat lives on the capsule). */
    virtual bool isEva() const;

    /* Assign Part::fuelGroup: a connected component of the part tree across the
       fuel-CONDUCTING parts; PartDef::fuel_barrier parts are WALLS and keep -1.
       Recompute after a stage split. */
    void buildFuelGroups();

    std::vector<Part *> fuelPool(Part *engine) const;

    /* The engine's fuel groups in LAYERS by hop distance, furthest first. A layer
       drains TOGETHER (pro-rata). Cached; do not mutate the reference. */
    const std::vector<std::vector<int> > &fuelDrainLayers(Part *engine) const;

    /* Draw `amt` kg of `type`, LAYER by LAYER and pro-rata -- first-tank-first
       would torque the ship. False if the total cannot cover amt. */
    bool consumeResourceMass(enum ResourceType type, float amt /* kg */, Part *engine);

    /* Total kg of `type` available to `engine`, so a burn can be sized before
       draining. */
    float availableResourceMass(enum ResourceType type, Part *engine) const;

    /* includeJets counts jet fuel too. */
    void enginePropellantMask(bool *burn, bool includeJets) const;

    float fuelMassMasked(const bool *burn) const;

    float getDeltaV();

    /* TODO should be cached per frame */
    float getMass();

    /* Felt acceleration [m/s^2]: |thrust + aero| / mass, no gravity. */
    double feltAccel();

    /* Test-only: not for game code. */
    void ApplyThrust_TESTONLY(double step) { ApplyThrust(step); }

    /* --- electrical (KSP-style EC) ---------------------------------------
       A shared pool across the batteries. powerTick runs per substep, BEFORE
       applyControlForces: gates the wheels, balances RTG generation against the
       life-support and wheel draws. Units: W and Wh. EC has no mass. */
    void powerTick(double h);

    /* Drain / charge up to `wh` of EC, pro-rata across the batteries. */
    void drainEC(double wh);

    void chargeEC(double wh);

    void getPower(double *gen, double *constDraw, double *charge, double *capacity);

    /* --power-log. */
    void power_log(double time);

    /* Staging. `activeStage_` is a monotonic COUNTER (the stage about to fire):
       it starts at the HIGHEST stage and steps down one per press. An engine
       lights once the counter reaches its stage (stage >= activeStage_) and stays
       lit; a decoupler triggers at its stage. */
    int activeStage_ = 1;
    int totalStages_ = 1;
    int minStage_ = 1;

    int activeStage();
    /* Step to the previous stage (clamped at the lowest one). */
    void advanceStage();

    /* Deploy every STOWED chute at `stage`. Game::stage calls it before the
       decoupler extractions, so chutes on dropped subtrees deploy too. One-shot. */
    int deployChutes(int stage);

    int numStages();

    float getThrust();

    // current Thrust-to-weight ratio
    float getTWR();

    // full throttle TWR
    float getFullThrustTWR();

    // empty TWR
    float getMaxTWR();

    void setVelocity(glm::dvec3 vel);

    /* The hull's transform origin. O(1). */
    const glm::dvec3& get_center_of_mass(void);

    glm::dvec3 applyGravity();

public:
    Vehicle();
    /* Teardown order: unregister the rigid body, delete it, then the parts. The
       onRails guard is LOAD-BEARING -- removeCollisionObject is not idempotent
       and can evict the WRONG object. */
    virtual ~Vehicle();

    glm::dvec3 processGravity();

    // Bullet clears the accumulated forces every stepSimulation: re-apply the
    // thrust before EVERY substep.
    void applyThrustForce();

    /* Lift + drag (reports/aerodynamics2026_09_11). LIFT acts PART BY PART;
       DRAG acts SHIP-LEVEL at the center of pressure, which preserves the
       weathervane torque. Cd is the area-weighted mean of the parts' cds, the
       area the hull silhouette (drag.h). */
    glm::dvec3 applyAeroForce(double h);

    /* Air density (kg/m^3) at the COM. Deliberately NO kRhoFloor gate:
       jetThrust scales by rho/rho_sea, so sub-floor reads vacuum. */
    double airDensityAtCom() const;

    /* Velocity relative to the AIR at `com`: GetVel() minus the air's
       co-rotation outside the body's rotating frame (#60/#97). */
    glm::dvec3 airRelativeVel(const glm::dvec3 &com);

    /* The armed control forces, per SUBSTEP. The EVA kerbal overrides (eva.h). */
    virtual void applyControlForces(double h);

    /* The first reaction-wheel PART (nullptr if none): the attitude reference. */
    Part *firstWheel();

    void applyRotationForce(double h);

    /* --- RCS translation (hydrazine mono, KSP-style) --------------------
       Field-driven (Part::isRcs). rcsDir is armed in whatever frame the control
       law uses -- ship axes for a ship, the render frame for the EVA suit. The
       force goes AT the COM: the COM-translation approximation. */
    glm::dvec3 rcsDir = glm::dvec3(0.0);  // armed dir (see above); 0 = off
    /* Burned this tick: the render pass draws the plume off this. */
    bool rcsFiring = false;
    void clearRcs();
    /* The armed RCS direction in WORLD axes (unit; (0,0,0) disarmed). Resolved at
       the point of use, so it tracks the live attitude. Virtual: its frame is the
       control law's business. */
    virtual glm::dvec3 rcsWorldDir() const;
    Part *firstRcsPart();
    double maxRcsThrust();
    void applyRcsForce(double h);
    /* s, hydrazine efficiency -- the EVA suit's value. */
    static constexpr double kRcsIsp = 220.0;

    /* Autopilot diagnostic (throttled; --slew-log). */
    void slew_log(double time);

    /* --att-log: nose (local +Z) + angular velocity. */
    void att_log(double time);

    /* --tq-log: the spurious-torque bug class, |dcom x F| vs |w|. */
    void tq_log(double time);

    /* --fuel-log: each group's fuel mass + the fuel links. */
    static const char *resourceName(int r);

    /* What a quantity of that resource is measured in. EC is stored energy, so
       it is the one type without a mass (Part::effectiveMass skips it for the
       same reason); everything else is kilograms. */
    static const char *resourceUnit(int r);

    void fuel_log(double time);

    /* --drain-log: thrust this tick (N) + each group's drain rate (kg/s). */
    void drain_log(double time);

    /* The largest wheel's rated torque (N m) for the HUD; the total is
       maxTorque() (the sum). */
    float GetWheelTorque();


    /* Disarm the armed thrust (once per tick). */
    void clearThrust();

    /* Likewise the armed rotation commands. */
    void clearRotCmd();

    /* Control moving to ANOTHER ship: zero the throttle, clear the armed cmds. */
    void releaseControl();

    /* Parts WOULD be dropped if `stage` fires: each decoupler there plus the
       subtree it anchors. The decoupler itself IS dropped. */
    std::vector<Part *> droppedPartsAtStage(int stage);

    /* --- docking ----------------------------------------------------------

       A dock joins two ships into ONE rigid body: this ship absorbs the other,
       rebasing its parts into this ship's S and reparenting its root under this
       ship's port part. The joint is a seam, so an undock (extractSubtreeAsShip)
       undoes exactly this. */

    /* The tree edge joining the ships: `port` is THIS ship's port, `root` the
       absorbed root. */
    struct DockSeam {
        Part *port;
        Part *root;
        std::string name;
    };
    /* Absorbed docks, innermost first -- the LAST is the one undock selects. */
    std::vector<DockSeam> seams;

    /* Docking INTENT (per ship): a dock needs BOTH halves -- target ship + port,
       and THIS ship's port that mates with it. */
    Vehicle *dockTargetShip = nullptr;
    Part *dockTargetPort = nullptr;
    Part *dockArmPort = nullptr;   // this ship's port to mate with (mandatory)

    /* Fold B into this ship at the mated port. B's shell is deleted by the
       caller, B's journal is dropped (#49). B's crew ride along because a seat
       belongs to its capsule Part, which moves with the part tree. */
    void absorbShip(Vehicle *B, Part *portA);

    /* Extract the subtree rooted at `root` into a new Vehicle, rebased into a new
       frame S' = the root's old frame. The new ship inherits frame/home/sun, takes
       the root's current world pose and the dropped side's COM velocity. nullptr
       if `root` is the whole ship. Seams maintained like fuel links. NOT in the
       physics world -- the caller enters it. */
    Vehicle *extractSubtreeAsShip(Part *root, const std::string &name,
                                  double t = 0.0);

    /* This ship's part frame -> renderFrame. Usually the identity. */
    glm::dmat4 renderXform(Frame *renderFrame) const;

    void Draw(const Camera* camera, Frame *renderFrame);

    // Single place to control the ship. While paused every command is dropped.
    /* step = the tick's simulated duration (dt * time_accel): fuel flow and the
       throttle ramp scale by it, so both no-op at step == 0 -- pass the real dt. */
    void Command(ShipCmd cmd, bool simActive, double step = 0.0);

    /* The COM velocity. */
    glm::dvec3 GetVel();

protected:
    // Forces/torques straight on the Bullet bodies, reachable only through
    // Command(). Protected: the EVA kerbal reuses the rotation-model helpers.
    void adjustThrottle(float delta);

    /* Full-throttle thrust RIGHT NOW (N): every ignited engine, jets at
       sea-level peak. */
    float GetActiveThrust();

    /* Once per tick: consumes the fuel and arms the per-thruster thrust
       (applyThrustForce applies it per substep). A thruster that can't consume
       its flow doesn't thrust. Stage gates WHEN it ignites, the fuel group WHAT
       it burns. */
    void ApplyThrust(double step);

    // --- physical rotation model -------------
    // alpha = maxTorque() / I (rad/s^2). Stick, slew and kill-rot all work
    // within that authority.

    double maxTorque();

    /* The inertia tensor about the COM in world axes (Bullet stores it DIAGONAL
       in the principal frame). O(1). */
    glm::dmat3 getInertia();

    /* The target direction in the ship's frame for the current slew mode; radial
       / normal reference the SOI body. */
    glm::dvec3 slewTargetDir();

    /* Slew the nose (local +Z) toward `dir`: target rate is the braking curve
       sqrt(2*alpha*E) capped at E/(2h), rate change bounded by alpha*h. */
    void slewToward(glm::dvec3 dir, double h);

    /* Kill the spin in one substep: tau = I * (-w) / h, scaled to
       |tau| <= maxTorque(). Needs the FULL inertia tensor, not the world-axis
       diagonal. */
    void killRotStep(double h);

public:

    glm::dvec3 GetPositionRelTo(const Part *part, Frame *relTo);

    /* Re-express the ship's state in newFrame and re-home it (setSoi). */
    void moveToFrame(Frame *newFrame, double t);

    /* The ONE SoI writer: every live (frame, m_parent, journal) change goes
       through here. Which list a Vehicle is in is the owner's business
       (Game::vehicles), not touched here -- so an SoI crossing cannot
       invalidate a walk of that list. */
    void setSoi(Frame *newFrame, double t);

    /* Per-tick SOI bookkeeping for THIS ship. */
    void switchFrames(double t);

    /* Write the rail state into the body (once per tick). Spin is zeroed. */
    void writeRailPose();

    /* The COM state in `inertial`, the frame node where the trajectory is a
       Kepler conic. */
    void comStateIn(Frame *inertial, glm::dvec3 &p, glm::dvec3 &v);

    // COM separation in the universe (root) frame: frame-invariant.
    double distanceTo(Vehicle *o);

    /* The COM's osculating orbit dips into the terrain band (periapsis within
       3 km of the surface). A PERIAPSIS test, not proximity. */
    bool inTerrainBand();

    /* The conic misses the body (periapsis above the terrain band). */
    bool isOrbiting();

    /* Resting on the surface NOW: in the rotating surface frame, near-static in
       it, within a generous band of the analytic terrain. Both are needed:
       either alone calls a hover "landed". */
    bool isGrounded();

    /* Rail-eligible: ORBITING coasts, GROUNDED freezes in the rotating surface
       frame. */
    bool canRail();

    /* Park out of the world and coast analytically. False if not rail-eligible. */
    bool goOnRails();

    /* Re-enter physics from rails: rebuild the Bullet state. */
    void leaveRails();

    /* Per-tick rail advance: propagate the conic, test the END of the advance for
       an SoI boundary, hand off there, refresh the parked transforms. The test
       only sees the endpoint, so a long step can fly past a whole sphere. */
    void railsTick(double t, const double step);

    /* Re-anchor the rail state on another frame and re-home it (setSoi). Asserts
       that the rail state and BOTH frames share one instant, and that the target
       contains the ship then. */
    void moveToRailFrame(Frame *newFrame, double t);

private:
    /* The rail state in universe-root axes, for the continuity asserts. */
    void railRootState(glm::dvec3 &p, glm::dvec3 &v) const;

    /* The SoI boundary test, shared by the physics and rails paths. kSoiMargin
       hysteresis stops a loitering ship from flapping. */
    Frame *soiTarget(const glm::dvec3 &posInFrame, bool skipSameBody);
};

struct System;  // system.h defines it

/* The part tree only: parts, edges, controller, fuel links. Does NOT seed the
   tanks, place the ship or enter the world. The catalog must outlive the ship. */
void build_ship_structure(Vehicle *ship, const ShipDef &def, Shader *partsshader);

/* Build a def on a pad: parts, full tanks, the inflated hull bottom on the pad
   top, into the world. `supportMargin` (m) is the surface's collision margin at
   `base`, plus the ship's; 0 if the caller re-places. */
void build_ship(Vehicle *ship, const ShipDef &def, Shader *partsshader,
                const glm::dvec3 &base, const glm::dmat3 &orient,
                double supportMargin);

/* Starting scenario (a CLI choice). Pad beds are set up in main; orbit beds are
   circular about the home body; ellipse-* and escape-* set their own apsides
   and speed; abs_r pins an ABSOLUTE radius. */
struct ScenarioDef {
    const char *name;
    /* Dev-only beds stay out of the VAB dropdown unless --dev-scenarios. The
       name resolves either way: --vab-scenario, --startship, saves and the
       e2e fixtures all name dev beds, so this gates the picker, not the
       scenario. No default: adding a bed means deciding this. */
    bool dev_only;
    bool on_pad;
    double alt_frac; // circular: fraction of the near-body shell (rot SOI -
                     // radius), or of the atmosphere top if atmo_frac
    bool polar;
    int ell_phase;   // -1: circular; 0: periapsis; 1: apoapsis; 2: 90 deg
    double peri_alt; // ellipse: periapsis altitude above the body radius (m)
    double apo_alt;  // ellipse: apoapsis altitude (m)
    double esc_frac; // escape: launch speed in local escape velocities (0 = not escape)
    double abs_r;    // > 0: absolute circular-orbit radius from the body
                     // centre (m), overriding alt_frac
    bool atmo_frac = false; // alt_frac scales the atmosphere top() instead, so
                     // the flying beds land in their science band
    /* Which plane a non-polar orbit bed sits in (#182). False (default) is the
       body's RAIL plane, where its system-file rails live; true is its EQUATOR. */
    bool eq_plane = false;
};

/* Look up a scenario by name; throws if unknown. */
const ScenarioDef *scenario_by_name(const std::string &name);

// Enumerate the scenario table (the VAB dropdown); index < scenario_count().
size_t scenario_count();
const char *scenario_name_at(size_t i);
bool scenario_dev_only_at(size_t i);

// Nose (local +Z) along `dir`; roll axis = the axis most orthogonal to it.
glm::dmat3 faceAlong(const glm::dvec3 &dir);

/* slot_offset (m): lateral separation for ships sharing an orbit bed, along the
   binormal so each orbit keeps its shape. Pads ignore it. */
void spawn_vehicle(Vehicle *ship, const ScenarioDef &sc, TerrainBody *home,
                   System &sys, double slot_offset, double t);

struct Game;   // game.h: the parts catalog, the part shader, the star, the roster

/* A name made unique against every live vehicle: first use keeps the bare name,
   later ones get " #2", " #3" .. Names are display text -- the save keys a
   vehicle by its slug (#266), so a clash no longer mis-resolves a load. Worth
   preventing anyway: the ship list, the tracking window, toasts and the
   [undock] line all show names, and two ships called 'racer' are unreadable
   there. Name-keyed resolution also still lives in the transfer planner, which
   looks a --transfer-target up by display name (transferplanner.cpp) -- a clash
   there does pick the wrong vessel, so this dedup is not only cosmetic.

   `roster` (nullable) additionally reserves the kerbals' names. A kerbal's
   display name IS its roster name and is never deduped, so without this a ship
   could be named 'Ash' while a kerbal named 'Ash' flies. */
std::string dedupVehicleName(const std::vector<Vehicle *> &vehicles,
                             const std::string &base,
                             const CrewRoster *roster = nullptr);

/* --- bringing a vehicle into the world -------------------------------------
   place_vehicle_def builds the vehicle and Game::vehicles adopts it. The
   journal is stamped with Game::time. `site`: the pad to stand on (on_pad
   scenarios); nullptr -> the body's default site, its first VAB_LAUNCH_PAD.
   Scenario is NOT applied -- spawn_vehicle is the caller's job. */
Vehicle *place_vehicle_def(Game &g, const ShipDef &def, const std::string &defPath,
                           const std::string &wantName, TerrainBody *hb,
                           const ScenarioDef *sc,
                           const StaticBuilding *site = nullptr);

/* Register an out-of-band vehicle; its builder already placed it, so it takes
   no orbit slot. */
void add_vehicle(Game &g, Vehicle *v, TerrainBody *home, const ScenarioDef *sc);

/* Apply every vehicle's scenario (the startup spawn_vehicle pass). */
void apply_scenarios(Game &g);

/* Build the --startship / --startships-file entries. Returns the first vehicle
   built (the natural active one), nullptr if empty. */
Vehicle *build_start_vehicles(Game &g, const std::vector<DebugStartShip> &entries);

/* --- the home pad's crew pool ----------------------------------------------
   The pool is the cast: launch takes, recover returns, death does not. A suit
   standing on the pad is a Part with no container and no vehicle (part.h). */

// The pad unassigned kerbals stand on (the body's default VAB_LAUNCH_PAD).
StaticBuilding *homeCrewPad(const System &sys);

// Take a suit from the home pad (nullptr if the pool is empty). Caller owns it.
Part *takeCrew(System &sys);

/* Take ONE named kerbal off the home pad by roster id (nullptr if nobody
   standing there wears that id). The VAB's seat assignments name people, so
   the pad is searched rather than served FIFO. Caller owns the result. */
Part *takeCrewById(System &sys, uint64_t profileId);

// Park a suit back on the home pad (recover). Takes ownership.
void returnCrew(System &sys, Part *suit);

/* Fill one seat of `ship` (part). False if the part is no capsule or the pad
   has no kerbal left -- an empty seat is a legal state, not a failure. The
   occupant is a suit Part parked in the capsule (part.h); no Kerbal vehicle
   exists until someone goes EVA.
   `suit` = the person to park (nullptr: draw FIFO from the pad);
   `slot` = which seat (0-based; -1 = the first free one). */
bool fill_seat(Vehicle *ship, size_t part, System &sys,
               Part *suit = nullptr, int slot = -1);

/* Every seat of every capsule part of `ship`, drawn FIFO from the home pad's
   crew pool: a 3-berth capsule launches with three people while the pad can
   supply them, and the rest fly empty once it cannot. */
void spawn_crew(Vehicle *ship, System &sys);

/* --radial-test spin diagnostics: rotational state + tidal torque. */
void spin_log(Vehicle *ship, double time);
