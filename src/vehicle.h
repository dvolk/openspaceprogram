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
#include "sas_policy.h" // SasPolicy: pilot vs. law (#421)

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/norm.hpp>

#include "btcommon.h"
#include "actuators.h"  // ActuatorRow (the wheel rows of AttitudeAuthority)
#include "body.h"
#include "physics.h"
#include "drag.h"     // SilhouetteMemo (the rest of the drag law: vehicle.cpp)
#include "rcs.h"      // the RCS nozzle allocator + per-substep Scratch
#include "shipdef.h"
#include "part.h"
#include "terrain.h"
#include "frame.h"
#include "orbit.h"

/* Autopilot slew targets -- mutually exclusive (last one wins); the stick
   composes with them. Declared above ShipCmd because a ShipCmd now carries one. */
enum SlewMode {
    SlewNone = 0,
    SlewPrograde,
    SlewRetrograde,
    SlewRadialOut,   /* align nose away from the SOI body (radius vector) */
    SlewRadialIn,    /* align nose toward the SOI body */
    SlewNormal,      /* align nose with the orbital-plane normal (r x v) */
    SlewAntiNormal,  /* align nose against it */
    /* Steer at a direction a LAW named, rather than one the class invented (#606).
       The six above are names this class resolves itself; this one is resolved from
       the aim vector that ShipCmd(Aim, dir) carries, because a guidance law can
       MEASURE a turn through pointingTo() but until now could not ASK for one --
       slewDemand()/slewTargetDir() are protected, so the only channels were the six
       names plus the stick, and a pitch-program law had to re-implement the slew law
       outside the class that owns it. */
    SlewFree,
    SlewKillRot      /* kill the spin */
};


// One ship-control command. Command() is the only control -> physics path.
enum ShipCmdType {
    ThrottleUp,
    ThrottleDown,
    ThrottleSet,   // a SETPOINT 0..1: a throttle law's channel; the keys are the RATE
    /* The attitude twin of ThrottleSet (#606): `dir` IS the aim, in the same basis as
       partAxis()/GetPos(). Like the throttle setpoint it is a STATE write, not a
       latch -- it never engages a mode. The law latches ShipCmd(Sas, SlewFree) once
       (Command's toggle rule, so re-sending it every tick would flicker it off) and
       then re-states the aim here as its own target moves. */
    Aim,
    Thrust,
    // Ship-relative (standard aviation mapping): see applyRotationForce.
    Pitch,    // W/S: about the ship's right axis
    Yaw,      // A/D: about the ship's up axis
    Roll,     // Q/E: about the ship's nose
    Sas,       // latch an Autopilot mode; ShipCmd::mode says which (#424)
    KillRot,   // X: hold-to-kill-the-spin, NOT the latched mode (see Command())
    // RCS translation (ship-relative, KSP-style): see applyRcsForce.
    RcsNose,   // N/H: along the ship's nose
    RcsUp,     // I/K: along the ship's up
    RcsRight,  // J/L: along the ship's right
};

/* What the sender asks for -- `amount`/`mode`, not a key flip.
   Pitch/Yaw/Roll: a NORMALIZED demand, -1..1 of the attitude authority the ship
   has (AttitudeAuthority, #425). Every consumer already multiplies through it --
   wheel torque, surface deflection, the TVC mixer -- so half an amount means half
   a demand. It stays normalized rather than a demanded rad/s^2 because the latter
   makes the allocator's job "deliver it no matter the hull", i.e. a 200 t stack
   rotating like a 5 t one; mass-dependent handling is the physics we have.
   Sas: `amount` unused, `mode` carries the SlewMode.
   Rcs*: only the sign is used -- a thruster group is on or off (#428 adds the
   per-pod rows that would make a magnitude mean something).
   ThrottleSet: `amount` IS the throttle, 0..1 of rated thrust (#400 F3). A rate is
   the wrong shape for a law: it plans with availableAccel(), so what it wants is a
   fraction, and ramping toward it through ThrottleUp would make the law's own output
   a function of how long it has been latched. */
struct ShipCmd {
    ShipCmdType type;
    float amount;
    SlewMode mode;
    glm::dvec3 dir = glm::dvec3(0.0);   // ShipCmd(Aim, dir): the free-aim vector (#606)
    ShipCmd(ShipCmdType t, float a = 0.0f) : type(t), amount(a), mode(SlewNone) { }
    ShipCmd(ShipCmdType t, SlewMode m) : type(t), amount(0.0f), mode(m) { }
    ShipCmd(ShipCmdType t, glm::dvec3 d) : type(t), amount(0.0f), mode(SlewNone), dir(d) { }
};

/* Why a rails tick stopped a coasting ship: the conic reached something a
   two-body coast cannot model (drag, or the ground). */
enum RailEnd {
    RailEndNone = 0,
    RailEndAtmosphere,
    RailEndGround
};

struct ScenarioDef;

class Vehicle {
public:
    /* A ship's own name. A Kerbal leaves this empty: its display name is the
       roster profile the suit owns (eva.h), so there is no second spelling to
       keep in step. Read displayName(), not this, wherever a player or a log
       sees a vehicle name. */
    std::string name;
    std::string defPath; // ship def it was built from ("" = test ship)
    /* Each Part owns its Body (see part.h). BUILD-ORDER throughout: setRoot and attach
       append, absorbShip appends a donor in its own order, and a split filters this same
       vector on both sides -- nothing sorts it, and nothing needs to, because
       stagedDecouplers breaks same-depth split ties by index into here. That index decides
       which booster keeps the bare name and which becomes "Booster #2", so a refactor that
       reshuffles `parts` reshuffles ship names with every test green (#544). The appends
       and BOTH halves of the split filter are pinned in tests/test_dock.cpp -- the
       survivor side needs a branch of its own to have an order to lose, which is why that
       fixture looks over-built. */
    std::vector<Part *> parts;

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

    /* THE SEAT: where the crew sits. The first-person eye is put on this part's top
       face (render.cpp), the boot camera aims its initial focus here (main.cpp), and
       the Vessel window's home distance measures to it. It is NOT the attitude frame:
       the laws, the stick and every displayed heading/pitch/roll ask attitudeRef() and
       attitudeBasis() (#571, #573) -- the orbit camera follows the COM, not this part.
       finalize() sets it, so it must start null. */
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

    /* What the ship presents to the air (#463). AREA is rotation-invariant, so
       the whole ship is projected through ONE flow plane in frame S;
       `aeroSilParts` holds each part's silhouette polygon for the CURRENT flow
       in that shared 2-D basis, reused between substeps (their areas also give
       the centre of pressure and the cd mean, so nothing projects twice). The
       union of those polygons -- not a convex hull drawn over the merged
       vertices, which invents airframe between the parts -- is the drag area;
       SilhouetteMemo caches that area per flow DIRECTION, filling nodes lazily
       and going stale by generation counter when the geometry changes.
       Geometry = the part set + their mounts: mass (fuel burn, the usual reason
       refreshCompound rebuilds) does NOT change the silhouette, so the cache
       invalidation is fingerprint-gated. */
    std::vector<std::vector<glm::dvec2>> aeroSilParts;
    // Scratch for filling cache nodes: those polygons belong to the cell
    // corners' flow directions, not the substep's, so they need their own set.
    std::vector<std::vector<glm::dvec2>> aeroSilNode;
    SilhouetteMemo aeroSil;
    uint64_t aeroGeomKey = 0;      // fingerprint the silhouette cache is keyed to
    bool aeroGeomValid = false;    // false = no geometry cached yet
    void refreshAeroGeom();

    /* #556/#555: the airframe's LIFTING SYSTEMS -- co-planar panels grouped into
       wings by drag.h::classifyLiftingSystems -- and each lifting part's index into
       them (`Part::aeroSystem`). Built wherever the geometry cache is rebuilt, off
       the same fingerprint, so docking, undocking, staging, part damage and an
       in-field fit-out re-group for free; the aero pass only reads. */
    std::vector<LiftingSystem> aeroSystems;
    void groupLiftingSystems();

    /* Per-substep scratch for the induced bill (#556): the lift each surface made,
       held until every surface has contributed, because the price is the SYSTEM's
       CL^2/(pi e AR) and the system's CL needs the whole sum. One reused vector --
       no allocation in steady state, and the sample count is the number of surfaces
       that actually made lift this substep. */
    struct AeroLiftSample {
        glm::dvec3 r = glm::dvec3(0.0);   // mount offset from the COM, frame S
        glm::dvec3 vp = glm::dvec3(0.0);  // the flow at the surface, frame S
        double q = 0.0;                   // that flow's dynamic pressure
        double area = 0.0;                // lift_area or control_area
        double lift = 0.0;                // |F| the law just made, newtons
        int system = -1;                  // index into aeroSystems
    };
    std::vector<AeroLiftSample> aeroLiftSamples;
    /* Per-system scratch for that bill, sized by groupLiftingSystems (which runs on
       a geometry change), so the per-substep pass only fills and reads. */
    std::vector<double> aeroSysLiftOverQ;
    std::vector<double> aeroSysCdi;
    void applyInducedDrag(Body *forceHull, double dragCd, glm::dvec3 &ftotal,
                          glm::dvec3 &moment, bool record);

    /* In the physics world? Asked of Bullet, not a flag. */
    bool hullInWorld() const;

    btCompoundShape *compoundShape() const;

    /* The part shapes at their authored poses in frame S -- the input to
       Bullet's principal-axis solve, and the source of the child poses the
       hull compound is re-based from. Held between rebuilds so a burn, which
       changes only the MASSES, can re-solve in place instead of minting a
       compound, a rigid body and their broadphase proxies every time the mass
       drifts past kMassRebuildFrac. Never registered with a world and its
       child poses only change at a rebuild, so it wants no AABB tree. */
    btCompoundShape *geomS = nullptr;
    /* The per-child masses fed to that solve (kept out of the hot path so the
       re-solve allocates nothing). */
    std::vector<btScalar> geomMasses;

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

    /* Fuel burn moves MASSES but not geometry, so it does not need a new
       compound or rigid body: re-solve the principal frame against the shapes
       in `geomS`, re-base the live compound's children, and re-set the body's
       mass props and pose. Bit-identical to what rebuildCompound() would have
       produced for the same parts, and it allocates nothing. False = the
       geometry moved under us (or there is nothing to refresh), so the caller
       must do the full rebuild. */
    bool refreshMassProps();

    /* The compound must reproduce the assembly it was built from (mass
       properties + child poses), recomputed independently. */
    void checkCompoundInvariants() const;

    /* The cheap half of those invariants -- the geomS mirror and the world
       inverse-inertia tensor -- so refreshMassProps() can check itself instead
       of waiting for a full rebuild to notice. */
    void checkHullMirrorInvariants() const;

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
    /* The direction a thruster on this part pushes RIGHT NOW: the authored +Z
       swung by the part's live gimbal state (src/gimbal.h). Zero gimbal is
       exactly partAxis(p, 2). */
    glm::dvec3 partThrustDir(const Part *p) const;
    glm::dvec3 partVel(const Part *p) const;
    /* The compound is ONE rigid body, so there is no per-part rate to ask for: every
       part of a ship shares the hull's angular velocity (#574). */
    glm::dvec3 hullAngVel() const;

    /* --compound-check: rebuild + report the simulated body. */
    void compoundCheck(double time);

    /* Fuel links (PartDef.fuel_link): one-way group -> group. Virtual. */
    struct FuelLink { Part *from; Part *to; };
    std::vector<FuelLink> fuelLinks;

    /* Cached per group; buildFuelGroups clears them. */
    mutable std::map<int, std::vector<std::vector<int> > > drainLayers_;

    /* --drain-log state. drainSample() runs on every tick the sim steps this
       ship, so the interval a rate describes is SIM time and means the same at
       any warp (#376). What accumulates is everything since the last PRINT --
       the samples tile the timeline, so a burn shorter than the print gap is
       still counted (measuring a fixed 0.5s window and printing only the last
       one before the gate covered 13% of the sim at 10x and read a real burn
       as 0.000). Each group's rate divides by the time THAT group spent, so a
       brief burn reports its flow rate rather than a duty-cycle average.
       The step comes from the caller rather than from this clock: a ship that
       stops being sampled (the player switches to another) would otherwise
       resume with one step spanning its whole inactive stretch and dilute its
       next rate. */
    std::map<int, double> drainPrevMass_;   // group -> kg at the last sample
    std::map<int, double> drainSpent_;      // group -> kg spent since the print
    std::map<int, double> drainSpentSpan_;  // group -> s of sim it spent in
    double drainSpan_ = 0.0;                // s of sim covered since the print
    bool drainPrimed_ = false;              // a sample ran on this part tree

    /* --dv-log state: the running delta-v budget, in m/s since the ship
       existed. Three routes to the same number:
         dvDelivered_ integrates the force the integrator was actually handed
         (|F| * h / hull mass, per substep -- applyThrustForce);
         dvAlong_ is that impulse projected onto the velocity direction, so
         dvDelivered_ - dvAlong_ is what the burn's pointing threw away. Times
         |v| it is the orbital energy the burn bought, but only for a constant
         |v|: a case brackets it between min|v| and max|v| instead;
         dvRocket_ integrates the Tsiolkovsky budget over the propellant the
         engines actually drained (ve * ln(m_before/m_after), per engine per
         tick -- ApplyThrust).
       dvDelivered_ and dvRocket_ share the part table's ve and propellant rate
       (PartDef::fullThrust() IS rate * ve), so their agreement does NOT prove
       the part table -- it proves the two sides agree on the MASS basis (the
       hull Bullet accelerates vs the analytic getMass()), on the CLOCK basis
       (the substep h summed vs the tick's step), and on the throttle-down
       factor s and exhaust_scale, each of which must land on both sides.
       LIMITATION: this is the MAIN-ENGINE budget. RCS pushes go through
       applyRcsForce() and jets drain their own fuel without a vacuum dv, so
       neither reaches it and dv_del reads 0 for an RCS-only nudge (#368).
       dvAlong_ also stays put while the inertial speed is 0: no direction to
       project onto. */
    bool dv_log_enabled = false;   // synced from args per tick (tick.cpp)

    /* PILOT vs SAS on one axis (#421, #425 D-2): who gets an axis when a held key and
       a latched law both ask for it. Synced from the Game per tick like dv_log_enabled
       -- it is a control preference the player owns, not ship state, so it is NOT in
       the ship save and cannot go stale. The mask is applied in applyRotationForce,
       where both demands are still separate vectors. */
    SasPolicy sas_policy = SasStickOwnsAxis;
    /* The wheels' ENABLE mask (#425 D-2): switched-off drums are absent from the
       capability model exactly as unpowered ones are (#418), so no law, readout or
       actuator row can promise a maneuver they will not do. Saved with the ship
       (save.h) because it is a property of THIS vessel, unlike sas_policy. There is no
       player-facing switch yet -- that is the next slice; today the mask exists so the
       allocator is honest and a toggle has somewhere to land. */
    bool wheels_enabled = true;

    double dvDelivered_ = 0.0;
    double dvAlong_ = 0.0;
    double dvRocket_ = 0.0;
    double propSpent_ = 0.0;       // kg of rocket propellant drained
    double dvVeLast_ = 0.0;        // last ve used, exhaust_scale applied (m/s)

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
    glm::dvec3 rail_vel = glm::dvec3(0.0);   // m/s, ship->frame coords
    glm::dmat3 rail_orient = glm::dmat3(1.0); // park frame -> ship->frame axes
    /* Frame S's axes at park time (the ship is rigid: nothing per-part).
       rail_orient * railRot is the hull's S -> frame rotation, so railRot stays
       valid across a frame hop that re-expresses rail_orient. */
    glm::dmat3 railRot = glm::dmat3(1.0);
    /* The instant rail_pos/rail_vel describe: must equal frame->rail_time when
       composed. */
    double rail_epoch = 0.0;

    double exhaust_scale = 1.0;  // --exhaust-scale: scales ve and all jet
                                 // thrust; synced per tick
    double drag_cd = 1.2;        // --drag-cd: the drag BILL multiplier (drag.h).
                                 // Scales the hull silhouette's Cd*A, every
                                 // lifting/steering surface's own shape bill
                                 // (Profile=) and the induced bill (#462, #497,
                                 // #455) -- never the lift or the control force
                                 // itself, so it decides how far an airframe
                                 // glides, not what its wing does (#502).
                                 // 0 = no drag. Chute Cd*A (below) is separate.

    /* Last substep's aero (applyAeroForce), for the --drag-log. */
    glm::dvec3 lastAeroForce = glm::dvec3(0.0);   // total (lift + drag)
    glm::dvec3 lastLiftForce = glm::dvec3(0.0);   // the lift part
    /* lastLiftForce . local vertical (away from the body centre): + = the wings
       are holding the airframe UP. Signed so the --drag-log can pin the lift
       DIRECTION, not just its magnitude -- a flipped AoA sign hid exactly that
       (#448), and |L| alone cannot see it. */
    double lastLiftVert = 0.0;
    /* The airframe's own up axis . local vertical: +1 upright, 0 on its side,
       -1 inverted. The --drag-log's guard rail -- L_up only means "the wings
       hold the ship up" while this is positive. */
    double lastUpVert = 0.0;
    glm::dvec3 lastAeroTorque = glm::dvec3(0.0);  // moment about the COM
    double lastDragAlt = 0.0;
    double lastDragRho = 0.0;
    double lastDragAlpha = 0.0;  // AoA (rad); + = nose above the flow (drag.h)
    /* The flow's angle off the nose in the YAW plane (rad), AeroFrame's other
       angle. Not a control readout (#450 wants it bundled with a yaw law) -- it
       is the silhouette cache's second key, so the drag log prints it. */
    double lastDragBeta = 0.0;
    /* Whether lastDragAlpha measured a flow worth reading: AeroFrame::valid AND
       the air-relative speed clears kAoAFlowFloor. False alpha is not "flying
       straight down the flow" -- it is "there is no flow to have an angle to". */
    bool lastDragAlphaValid = false;
    double lastDragArea = 0.0;  // BODY silhouette facing the flow (m^2) -- since
                                // #455 the union refuses any part with lift_area or
                                // control_area, so this is what the hull stops, not
                                // what the airframe presents
    double lastDragCd = 0.0;    // area-weighted mean of the BODY parts' cds
    /* The hull's weathervane lever: (cp - com) . nose, metres; + = cp AHEAD of the
       COM. #486 made hull drag answer to the flow AT cp, and two different things
       ride on that. The sweep of the flow OPPOSES a rotation whatever the sign of
       this; what the sign decides is the other half -- hull drag acting AT AN ANGLE
       of attack sends the nose back into the flow only with cp BEHIND the COM, and
       diverges it with cp ahead. Printed as CpL= in the drag log because it is the
       quantity #454's static margin is made of and nothing else shows it. */
    double lastCpLever = 0.0;
    /* #455's companion to CpL: the lever of the WHOLE airframe's form drag -- the
       hull union AND every surface's own planform bill, weighted by the CdA each one
       actually pays. CpL on its own went narrow the moment the surfaces left the
       union: on the jet the hull's own cp sits 0.09 m AHEAD of the COM (destabilizing)
       while its rudders and elevators, 3 m aft, are the biggest form-drag areas the
       airframe has. A tuner reading CpL alone would call that airframe unstable in
       yaw when the thing holding it straight is the tail. 0 when the pass paid no
       form drag at all. */
    double lastFormCpLever = 0.0;
    /* #454, the LIFT half of the same judgement (drag.h::LiftSlopeSum).
       lastLiftSlope is how much lift an angle-of-attack change buys this substep,
       CL·A in m^2/rad (multiply by q for the force) -- it collapses towards zero as
       the surfaces droop past the stall, which is the airframe stopping caring about
       its angle of attack. lastStaticMargin is where that change lands: metres BEHIND
       the COM along the nose axis, so + = an overshoot makes more lift behind the COM
       and the airframe pushes the nose back down by itself. A ship with no lifting
       slope at all -- stalled flat, mounted edge-on, no wings -- or whose surfaces
       cancel each other, has no margin to quote, and the drag log dashes it the way it
       dashes CpL. */
    double lastLiftSlope = 0.0;
    double lastStaticMargin = 0.0;
    bool lastStaticValid = false;
    /* Deployed chutes' EFFECTIVE Cd·A (m^2) last substep (0 = none): nominal
       chute_area shrunk by kChuteAccelLimit at high q. NOT in lastDragArea. */
    double lastChuteCdA = 0.0;
    /* The induced Cd·A (m^2) owed for the last substep's turning of the flow
       (#462, 0 = none): every LIFTING part's share plus, since #494, every DEFLECTED
       control surface's share -- a steered surface lifts sideways and pays for it,
       which is what makes a held trim cost something instead of buying altitude for
       free. It rides in lastAeroForce along with the rest of the drag; this is what
       lets the drag log tell blunt form drag from the price of lift. */
    double lastInducedCdA = 0.0;
    /* The PROFILE Cd·A (m^2) the lifting and steering surfaces owed last substep
       (#455): planform area x the authored cd read at the surface's own incidence.
       It is what those parts pay instead of joining the hull union above, so a wing
       is charged once for its shape (here) and once for its lift (Induced= above),
       never twice for the same planform. */
    double lastSurfCdA = 0.0;

    /* Last substep's control-surface deflections (--drag-log). */
    struct ControlDeflection {
        const PartDef *def;  // part type + control_axis
        int index;           // Nth control surface, 0-based
        double deflection;   // rad, signed hinge angle
    };
    std::vector<ControlDeflection> lastControlDeflections;

    /* THE STICK AND THE AUTOPILOT (#575). Command() arms them once per tick and the
       laws read them per SUBSTEP -- Bullet clears the accumulated torque each
       stepSimulation. The fields live under "THE CONTROL STATE" below, where every
       writer is named; these are the readers:
         stickAxis(0/1/2)   manual demand, roll / pitch / yaw, normalized -1..1
         slewMode()         what is turning the ship THIS tick -- the ENGAGED copy, so
                            a held override (ShipCmd(KillRot), the X key) writes there
                            and dies with clearRotCmd(). A window asking "what mode does
                            the player have on" wants slewRequestMode(): the engaged copy
                            says KillRot every time X is held, which is not what the
                            Autopilot window shows. The laws read `slew` from inside the
                            class; this is the reader for the pins and the traces.
         slewRequestMode()  what the player has latched on the Autopilot window */
    float stickAxis(int n) const {
        assert(n >= 0 && n < 3 && "stick axis is roll / pitch / yaw");
        return stick[n];
    }
    SlewMode slewMode() const { return slew; }
    SlewMode slewRequestMode() const { return slewRequest; }
    /* What a law last asked SlewFree to steer at (#606), in the same basis as
       partAxis()/GetPos(). Raw: it is NOT normalized here, because a zero or NaN ask
       is a real answer about the law's state and pointingTo()'s door already fails it
       closed -- the [slew] trace then reads `mode=free no-target` instead of quietly
       steering at last tick's direction. */
    glm::dvec3 slewFreeDir() const { return slewFreeDir_; }
    /* The save loader's path, like restoreSlewRequest(): a stored aim is a legitimate
       writer and the sim may be paused when it runs, so it cannot come through
       Command(). Loading SlewFree without its vector would latch a mode that steers
       nowhere. */
    void restoreSlewFreeDir(glm::dvec3 dir) { slewFreeDir_ = dir; }
    /* The once-per-tick copy from the latched mode to the engaged one. It carries its
       own gate now: a paused sim engages nothing, and a latched SlewNone engages
       nothing. tick.cpp used to test the gate and write the copy by hand after
       clearRotCmd() had already dropped the engaged copy, so the OLD order was not wrong
       -- the point of moving it inside is that the rule deciding what drives the ship no
       longer lives outside the class that owns the state (#575). Being total (it writes
       on every call, not only when a mode is on) means a caller cannot forget the clear.
       Returns whether a mode is driving this tick, which is what tick.cpp's "leave the
       rails" test asks. The HELD override (the X key,
       ShipCmd(KillRot)) still comes after this in tick.cpp, so it keeps winning. */
    bool applySlewRequest(bool simActive) {
        slew = (simActive && slewRequest != SlewNone) ? slewRequest : SlewNone;
        return slew != SlewNone;
    }
    /* May the wheels be spent at all: electrical pressure AND an enable order.
       One call because there must not be a second spelling (#425 step D): it is the
       door attitudeAuthority() builds its wheel rows behind, so the per-axis
       capability, the laws, the stick's own scaling, the Vessel window's row and the
       EC bill all say the same thing. Note that a capsule's built-in triad is a wheel
       too (Part::isWheel() is `torque > 0`), so `wheels_enabled` switches that off as
       well as any bought reaction_wheel part (#600). */
    bool wheelsPowered() const { return powered_ && wheels_enabled; }
    /* Are the attitude actuators doing work THIS tick? powerTick() bills the wheels'
       active draw on exactly this, and the Vessel window's Power row asks it too: an
       unpowered or switched-off hull's wheels are not drawing even with the stick
       pinned, and a second spelling of the rule is how those two rows drift apart. */
    bool actuatorsEngaged() const {
        return wheelsPowered() && (stick[0] != 0.0f || stick[1] != 0.0f
                                   || stick[2] != 0.0f || slew != SlewNone);
    }
    /* W the actuators are billing right now: the wheels' draw, summed over every wheel
       HOUSING and zero when they are not engaged. A triad housing (#593) bills its
       one `power_draw`, not three: the watt figure is the package's, while `torque`
       is per drum -- deliberately, since rating the drums individually would triple
       every shipped wheel's power bill with no author asking for it. powerTick() pays
       this and the Vessel window's Power row shows it, so the row cannot disagree with
       the bill (#575) -- it used to re-derive the sum, and dropped the power gate
       while doing it. */
    double wheelDrawWatts() const;
    /* The TVC aim handed to the mixer: one shared lateral deflection vector
       (world axes, rad) armed by gimbal.h tvcSplit and consumed by applyThrustForce.
       applyRotationForce clears it at the top of the substep and actuateAttitude
       ASSIGNS it exactly once -- tvcCalls_ counts the calls and asserts the bound.
       The stick and any latched law reach the allocator as ONE summed demand (#420),
       so there is one split per substep and no shares to merge. The mixer's per-engine
       clamp against gimbal_max is what keeps the deflection inside the hardware.
       Cleared when nothing commands it, so a released autopilot and a released stick
       both recentre. */
    glm::dvec3 tvcLat_{};
    int tvcCalls_ = 0;
    /* #426: the deflection the ONE actuatorSolve asked each control surface for, in
       radians of COMMAND (so the sign convention is the authored one: +this means the
       travel controlDeflection would take for a +command, whichever way the plate is
       bolted). actuateAttitude fills it from the solve's per-row setpoints;
       Vehicle::applyAeroForce is the only consumer and turns each entry into travel for
       THIS substep -- which is why the tick runs control before aero (see tick.cpp).
       Same clear-and-assign discipline as tvcLat_: applyRotationForce clears it, the
       solve refills it, so a released key and a quiet law both recentre the plates.
       A surface with no entry is commanded to nothing, which is what a hull at rest or
       in vacuum gets: no q, no rows, no travel to show. */
    std::vector<std::pair<const Part *, double>> surfaceLat_;
    /* WHAT CAN TURN THIS SHIP RIGHT NOW (#425). One answer to "how hard, and
       about which axis", for every consumer: the slew law, the kill-rot law, the
       [slew] trace, and later the RCS and aero rows. Before this each caller
       re-derived the budget and none of them asked about the axis. Build one by
       asking attitudeAuthority(); the fields are a snapshot of the hull, not
       something to author.
         wheels:      one ActuatorRow per powered DRUM, axis = that drum's own
           axis and limit = its rated N m. The row shape is what #576 settled: a
           drum levers torque ALONG ITS AXIS, so N drums on one line buy N x along
           that line only and nothing about the axes no drum is on. PartDef::wheel_axes
           says how many lines a housing covers (#593): a triad -- every shipped
           wheel part -- buys three lines at its rating, a single drum buys the one
           it is mounted along. There is still no momentum store and nothing
           saturates (#579).
           Empty with no wheel fitted or no EC power (the wheels are electric,
           #418); the Vessel window prints what these rows buy per axis, which is
           the same gated answer the laws spend (#572: one answer, not the one the
           largest wheel happens to give).
         tvcTorque:   N m, but it is a DISK in the plane perpendicular to the
           mixer's stack axis, not a ball: see attitudeTorqueAbout.
         heffMag:     N m per rad of the SHARED lateral deflection, Sum(T_i*|h_i|),
           which is what turns a torque to gimbal angles (gimbal.h tvcSplit).
         stackAxis:   that disk's normal -- the root part's +Z, i.e. the frame
           applyThrustForce and tvcSplit steer in. attitudeTorqueAbout and
           actuateAttitude read it from here so they cannot talk past each other, and
           since #571 the laws ask the same part for their facing (attitudeRef()), so
           the budget, the mixer and the stick are all stated in one frame.
       TVC follows throttle because it is read off the ARMED thrust, which is a
       per-TICK transient (ApplyThrust arms it, the tick clears it), so ask once per
       substep: what actually moves between substeps is the power gate and the COM
       these levers are measured from. */
    struct AttitudeAuthority {
        /* #576: a wheel is a rating AND the lines it levers. One row per powered
           DRUM: a single-drum housing (PartDef::wheel_axes == 1) is one row along
           the part's own +Z, so mounted on the stack it rolls this hull and cannot
           pitch or yaw it -- which is what a real single-drum package does. A triad
           (== 3, #593) is three orthogonal drums and so three rows, one per axis of
           the part's frame, at the same rating: it covers three lines however the
           mount rolls it. Two consequences worth having: drums stacked on one line
           no longer buy authority about all three axes, and a hull that wants
           attitude authority about all three has to carry drums that cover all
           three, which is a build question the player can see answered. Momentum
           saturation is NOT part of this model (#579).
           The rows are empty when no wheel is fitted or there is no EC power (the
           wheels are electric, #418), and the Vessel window prints this same
           gated answer per axis (#572: one answer, not the one the largest wheel
           happens to give). */
        std::vector<ActuatorRow> wheels;
        /* Sum of the row ratings, deliberately NOT called a capacity: for a triad
           housing it is 3x what any one of its lines can carry, so it answers
           "how much wheel does this hull own" and not "how much about some axis".
           The tests use it to pin how the rows were built (#576, #593); since #420
           nothing in the game spends it, and it must NOT become a budget again --
           a scalar cap on the wheel share preserves the demand's direction, so one
           saturated line spends the idle ones. The axis-specific number is
           actuatorTorqueAbout(wheels, axis), and the bound the allocator obeys is
           actuators.h actuatorSolve, per line. */
        double wheelRatingSum() const {
            double t = 0.0;
            for(const ActuatorRow &w : wheels) { t += w.limit; }
            return t;
        }
        /* #426: the control surfaces as actuator rows -- torque per radian of the
           COMMAND along the row, travel stop as the limit (actuators.h says a row's axis
           is SCALED by what a unit of its demand buys, so a fin's setpoint comes out in
           RADIANS). Built from build facts (mount normal, not the live liftDir -- #495's
           rule -- offset from the COM, control_area, cl_control, max_deflection) times
           the COM's dynamic pressure, so a hull in vacuum or at rest has no aero rows,
           which is the right answer rather than a missing one.
           SPENT, since slice 2: actuateAttitude solves these in the SAME actuatorSolve
           call as the wheels, and what it asks each one for reaches the part through
           surfaceLat_ and thence applyAeroForce. That is #426's mix -- one demand can
           reach any surface whose rig levers part of it -- and two things follow that
           neither the row nor the solver says on its own:
           - PartDef::control_axis is a COMMAND label (which key asks for this surface,
             and which axis decides its rig sign), not a wall on what it may be spent
             for: a plate at 45 deg clock levers pitch and yaw at once. A plate whose
             push decides nothing ABOUT ITS OWN AUTHORED AXIS is dead in the rig and in
             the budget alike (#495).
           - a surface's share of the solved vector is NOT applied as torque here. It
             arrives next door as an aero FORCE; applying both would bill the air twice.
           One row per surface, in `surfaceParts` order: actuatorSolve's setpoints come
           back indexed by row, so the rows have to be able to name the part they came
           from (and the Vessel window reads the same rows, which is why #599's
           budget-not-delivery caveat applies to fins as much as to drums). */
        std::vector<ActuatorRow> surfaces;
        std::vector<const Part *> surfaceParts;
        double tvcTorque = 0.0;
        double heffMag = 0.0;
        glm::dvec3 stackAxis = glm::dvec3(0.0);
    };
    AttitudeAuthority attitudeAuthority();
    /* N m of that authority the actuators can bring to bear ABOUT `axis` (any
       length; it is the direction that counts). Each wheel contributes its rating
       times cos of the angle between its own drum axis and `axis` (actuators.h,
       #576); each control surface its row's cosine (#426 -- in cruise a fin out-bills a
       pod's drum by two orders of magnitude, so this is the term that gives a hull with
       no wheel any attitude authority at all, and the reason a latched law flies a
       plane); TVC contributes tvcTorque * sin(angle between the axis and the stack
       axis), because a lateral nozzle deflection levers only torque perpendicular
       to the stack. So the pair is NOT a single number: about the stack axis
       (roll) the mixer delivers wheels only -- today. #419 would roll a cluster by
       circulating the gimbals, and when it does this model gains that row rather
       than arguing with it.
       gimbal.h tvcSplit already routes torque this way -- the stack-axis component
       goes to the wheels, and with no wheel rows the solve applies nothing -- so
       this is the same physics stated once, on the side that decides how fast the
       law may aim to go. */
    double attitudeTorqueAbout(const AttitudeAuthority &auth,
                               const glm::dvec3 &axis) const;
    /* WHAT AIMING AT `dir` MEANS FOR THIS HULL (#400) -- one answer, so the laws, the
       --slew-log trace and guidance cannot each derive their own and drift. (They had:
       slewDemand() and slew_log() both computed the error, the axis, the inertia and the
       authority-limited rate about 25 lines apart, and the log's target test was looser
       than the law's, which is how a trace came to read alpha=0 next to a ship actually
       slewing.)
       It is a MEASUREMENT, not a law: the braking curve (w_des = sqrt(2*alpha*E), capped
       by the substep) stays with whoever aims, because the cap belongs to the caller's h.
       `valid` is false when there is nothing to aim at -- no frame, or a zero-length
       target -- which is also the NaN door: a normalized zero vector used to poison E
       and the axis. An aligned hull is valid with angle 0, not invalid: "already there"
       and "no target" are different answers a guidance law has to tell apart.
       FRAME: `dir`, `facing` and `axis` are all in the hull's Bullet-world basis --
       the same one partAxis() and GetPos() use -- not an inertial one. Prograde and
       the orbit normal belong to the CONIC, so slewTargetDir() reads them from the
       non-rotating frame and rotates them back. That is not ceremony: inside a
       rotating surface frame GetVel() is the ground-relative velocity, which for a
       ship flying slowly points nowhere near its orbit's prograde. A reader outside
       the funnel owes itself the same conversion. */
    struct Pointing {
        bool valid = false;
        glm::dvec3 facing = glm::dvec3(0.0);    // the nose of attitudeRef() [unit]
        glm::dvec3 axis = glm::dvec3(0.0);      // + turns facing toward dir [unit]
        double angle = 0.0;                     // E [rad]; 0 == aligned
        double Ieff = 0.0;                      // kg m^2 about axis
        double alphaMax = 0.0;                  // rad/s^2 the actuators could hold here
    };
    /* The authority comes in as a parameter because the laws already hold this
       substep's copy -- building a second one per substep is #594's churn. The
       one-argument form is for readers outside the funnel (guidance, the log). */
    Pointing pointingTo(glm::dvec3 dir, const AttitudeAuthority &auth);
    Pointing pointingTo(glm::dvec3 dir);
    /* Send one commanded attitude torque to the actuators: roll about the
       stack axis is wheels-only, transverse goes to TVC first up to tvcTorque,
       the remainder to the wheels AND the control surfaces (gimbal.h tvcSplit).
       Both of those go through ONE actuators.h::actuatorSolve call over the wheel
       rows then the surface rows (#426), so what reaches the hull is what the
       actuators' own axes can carry: a demand about an axis nothing covers is
       dropped there rather than billed to a drum that cannot answer it (#576).
       The surface rows' share is deliberately NOT applied as torque -- their
       setpoints go to the air (surfaceLat_) and the force law bills them there;
       applying the abstract torque too would charge the hull twice for one fin.
       Stashes the TVC half in tvcLat_ for applyThrustForce this same substep.
       applyRotationForce answers ONE summed demand and calls this once, so that aim
       is the whole answer rather than one consumer's share of it (#420). */
    void actuateAttitude(glm::dvec3 torque, const AttitudeAuthority &auth);
    glm::dvec3 lastThrustForce{};  // debug: total thrust force applied last substep
    /* Restoring the latched Autopilot mode when a save loads. Input never calls this:
       a key press or a window click goes through Command(ShipCmd(Sas, mode)), where the
       toggle/replace rule lives (#424). It stays public because the save loader is not a
       Vehicle, so this is one door that names its caller rather than locking it out. The
       range check is its own: a save is user data, and an out-of-range value there would
       otherwise wake the ship off the rails and then leave slewTargetDir() with no target
       for it -- a ship that runs at 1x and refuses to turn, forever. */
    void restoreSlewRequest(SlewMode m);

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

    /* The one name a player or a log should show. A ship stores it in `name`;
       a walking kerbal derives it from the suit's roster profile, the profile
       that also owns the spelling a seat reads. Returns a reference: the
       transfer planner keeps a pointer into this string. */
    virtual const std::string &displayName() const { return name; }

    /* Assign Part::fuelGroup: a connected component of the part tree across the
       fuel-CONDUCTING parts; PartDef::fuel_barrier parts are WALLS and keep -1.
       Recompute after a stage split. */
    void buildFuelGroups();

    std::vector<Part *> fuelPool(Part *engine) const;

    /* The engine's fuel groups in LAYERS by hop distance, furthest first. A layer
       drains TOGETHER (pro-rata). Cached; do not mutate the reference. */
    const std::vector<std::vector<int> > &fuelDrainLayers(Part *engine) const;

    /* What a draw DID, as opposed to what it was asked for (#531). `drawn` is the kg
       that actually left resources.current; a refused draw moved nothing, so it drew 0.
       Bill `drawn`, never the request: a pro-rata layer can give up a hair less than it
       was asked -- a couple of ULP of the request, 2.00 ULP on the fixture pinned in
       tests/test_fuel.cpp -- and #526 was exactly this habit, charge the plan and move
       the tanks separately, at a scale where it produced delta-v of 1.8e6 and then
       -nan. Every src/ caller spells `.ok`; the bool cast is so `if(!consumeResourceMass
       (...))` keeps reading as a question at the test sites that predate the struct. */
    struct FuelDraw {
        bool   ok;      // false: the layers cannot cover the request; nothing moved
        double drawn;   // kg that left resources.current
        explicit operator bool() const { return ok; }
    };

    /* Draw `amt` kg of `type`, LAYER by LAYER and pro-rata -- first-tank-first
       would torque the ship. Refused (ok = false, nothing moved) if the total cannot cover
       amt, so a caller that sized a draw from availableResourceMass must clamp it:
       recomputing it as desired * (avail / desired) can land an ULP above avail
       (#382). Double like the store it draws from. */
    FuelDraw consumeResourceMass(enum ResourceType type, double amt /* kg */, Part *engine);

    /* Total kg of `type` available to `engine`, so a burn can be sized before
       draining. */
    double availableResourceMass(enum ResourceType type, Part *engine) const;

    /* includeJets counts jet fuel too. */
    void enginePropellantMask(bool *burn, bool includeJets) const;

    double fuelMassMasked(const bool *burn) const;

    /* The figures a player plans a burn with, in double: the fuel they are made
       of is stored in double (#379), and narrowing on the way to the readout
       throws that precision away -- ULP(0.0078kg) at 97t, ~1kg at 10kt (#382). */
    double getDeltaV();

    /* TODO should be cached per frame */
    double getMass();

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

    /* THE TWO THRUST NUMBERS, and why both exist (#400).
       producedThrust() is what the engines are doing RIGHT NOW: rated thrust times the
       current throttle times the air state. Anything that COMMANDS the throttle and
       then reads this back reads its own zero -- the prototype in
       reports/autopilots2026_10_24 computed a_max from it while coasting, got 0.00 every
       tick, took its own "coast" branch and never reignited (silent: a burn that never
       happens, not a crash).
       availableThrust() is what the ignited stage COULD do at full throttle, and
       availableAccel() is that over the CURRENT mass: the a_max a throttle law, a
       gravity turn or a landing predictor needs. It does not move with the throttle, so
       a law can plan with it and act on the throttle without a feedback loop through its
       own output. It does include the current air state and the stage gate, so "available"
       means available to THIS stage HERE -- a jet in vacuum says zero, and a jettisoned
       stage is not available at all. */
    double producedThrust();
    double availableThrust();
    double availableAccel();

    /* THE LEVER ITSELF (#400 F3). The throttle used to be a public float, so a law
       could only write it behind the player's back and could not tell whether the
       throttle it read next was its own answer or the player's last key -- the same
       invisible fight #425 D-2 solved for the stick, where a held key owns its axis.
       So: one read, one named write, and a per-tick answer to "is the player on the
       throttle right now", which is what lets a law stand aside and resume. A law that
       wants it latched sends Command(ShipCmd(ThrottleSet, frac)) each tick, exactly as
       the slew law re-states its ask every substep. */
    float throttle() const { return thruster_util; }
    void setThrottle(double frac);
    bool throttleKeyHeld() const { return throttleKeyHeld_; }

    // current Thrust-to-weight ratio (at the current throttle)
    double getTWR();

    // full throttle TWR
    double getFullThrustTWR();

    // empty TWR
    double getMaxTWR();

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

    /* Bullet clears the accumulated forces every stepSimulation: re-apply the
       thrust before EVERY substep. `h` is that substep's duration: --dv-log
       integrates |F|*h/m over it, so the budget needs the same clock the
       integrator advanced by. */
    void applyThrustForce(double h);

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

    /* THE FRAME THE CONTROLS, THE ATTITUDE LAWS AND THE AERO PASS SPEAK (#571):
       the root part -- the pitch/yaw/roll axes the stick drives, the TVC disk's
       normal, the nose the autopilot aligns, the frame AoA and beta are measured
       in. The root, not "a part on the stack", because the ship is BUILT in the
       root's frame: build_ship solves every part's pose from its parent's with the
       root at the origin and its +Z as the stack axis, so the root's +Z is the
       axis the compound itself was authored in.
       Wheels are actuators, never a frame. A surface mount keeps a child's +Z
       parallel to the parent's +Z projected into the contact plane -- exactly
       parallel for a transverse contact, which is what the "angle" shorthand always
       gives -- and rolls the part about the stack. So a wheel bolted to a tank's
       side keeps the ship's nose and has the ship's UP axis as its "right" one:
       read the stick in that part's frame and W/S yaws while A/D pitches
       (res/ships/sideways_wheel.json, e2e flight/571). An authored "point"/"normal"
       contact can tilt a part outright, which is another reason to ask for the root
       rather than for whichever part carries the wheels.
       This is NOT the seat the view sits in, which still follows `controller`. */
    Part *attitudeRef() const;

    /* THE AXES A DISPLAYED ATTITUDE SPEAKS (#573). Same part as attitudeRef(), so a
       heading/pitch/roll readout, the chase basis and the orbit camera's up describe the
       rotation the keys and the laws actually applied. Reading them in another part's
       axes used to be the same lookup the laws used before #571, so they agreed by
       construction; after #571 they agree only while that part keeps the stack's axes.
       On res/ships/sideways_wheel.json the wheel is surface-mounted at angle: 90, which
       rolls it 90 deg about the stack: --att-log shows holding S pitching the hull while
       a heading taken from the wheel's frame counts a yaw.
       Columns are [right, up, nose], the same order as partAxis(ref, 0/1/2); the orbit
       camera wants them as [back, right, up] and reorders them at the call.
       `controller` stays the SEAT: the first-person eye sits on that part's top face, the
       boot camera aims its initial focus there, and the Vessel window's home distance
       measures to it. Where you sit and which way the ship points are different questions. */
    glm::dmat3 attitudeBasis() const;   // == partRot(attitudeRef()), columns [right, up, nose]

    void applyRotationForce(double h);

    /* --- RCS translation (hydrazine mono, KSP-style) --------------------
       Field-driven (Part::isRcs). The armed direction is stored in whatever frame
       the control law uses -- ship axes for a ship, the render frame for the EVA
       suit -- and lives under THE CONTROL STATE below (#575). Force is allocated
       per NOZZLE at the pod's position (src/rcs.h, #428): an off-center pod
       translates AND spins the ship, as reality demands.
         armRcs()          the named writer for the armed demand, and the one that keeps
                           it a DIRECTION -- declared under THE CONTROL STATE below,
                           because its callers are Command's own Rcs* cases and the EVA
                           suit, which IS-A Vehicle (a suit has no RCS parts to Command).
                           Arming means a DIRECTION: the demand is stored unit (or zero),
                           because the suit multiplies its fixed thrust by this vector
                           directly, so a magnitude-2 demand there would be twice the
                           thrust the player asked for. rcsWorldDir() normalizes for the
                           ship anyway; doing it here means the two paths cannot disagree.
         markRcsBurned()   a nozzle actually fired this substep. Both burn paths call
                           it -- applyRcsForce for pods, the suit's jets in eva.cpp --
                           so the flag has one setter and step D has one place to look.
         rcsBurned()       burned at some point since this tick's clearRcs(): the flag
                           is set per substep and cleared once per tick. What the
                           per-nozzle plumes draw and the [eva] trace log.
         rcsTranslating()  burned AND a demand with a direction in it -- what the
                           nozzle-less COM puff (the suit) draws. */
    bool rcsBurned() const { return rcsFiring; }
    bool rcsTranslating() const { return rcsFiring && glm::length2(rcsDir) > 1e-12; }
    /* The last airless-or-not substep's allocation, for the [rcs] log (same
       idiom as lastControlDeflections: empty = no translation requested). */
    struct RcsNozzleLog {
        std::string part;   // the pod's def name
        std::string tag;    // the BELL that fired (part-local, "+Z"); its
                            // force acts along the OPPOSITE of the tag
        double thrust;      // N actually fired
    };
    std::vector<RcsNozzleLog> lastRcsNozzles;
    glm::dvec3 lastRcsCmd = glm::dvec3(0.0);  // demanded force (world, N)
    glm::dvec3 lastRcsAch = glm::dvec3(0.0);  // achieved force (world, N)
    void clearRcs();
    /* The armed RCS direction in WORLD axes (unit; (0,0,0) disarmed). Resolved at
       the point of use, so it tracks the live attitude. Virtual: its frame is the
       control law's business. */
    virtual glm::dvec3 rcsWorldDir() const;
    Part *firstRcsPart();
    double maxRcsThrust();
    void applyRcsForce(double h);
    /* Reused solve + enumeration buffers: applyRcsForce runs EVERY sim
       substep, so these persist (capacity only grows) and the hot path
       allocates nothing after warmup. Only applyRcsForce touches them. */
    rcs::Scratch rcsScratch;
    std::vector<RcsNozzle> rcsNoz;        // this substep's nozzles
    std::vector<size_t> rcsNozPart;      // nozzle -> index into parts
    std::vector<size_t> rcsNozLocal;     // nozzle -> slot on that part (Part::rcsFired)
    std::vector<glm::dvec3> rcsFpart;    // per-part summed force, parts-sized
    /* s, hydrazine efficiency -- the EVA suit's value. */
    static constexpr double kRcsIsp = 220.0;

    /* --- force overlay (F4, gameui.cpp drawForceOverlay) -----------------
       What the LAST recorded substep actually applied to the hull, per force
       site -- aero (drag at the center of pressure, chute at the canopy
       anchor, lift + steering at each surface) AND control (engine, RCS
       pod). The tick.cpp substep loop owns the per-substep clear. pos is
       the application point rel COM AT RECORD TIME (the same arm Bullet
       got): the drawer adds the ship's CURRENT COM, so the ship's travel
       between the recorded substep and the paint pass cancels (an absolute
       pos would trail the ship by up to a tick of velocity -- tens of
       meters in orbit). The attitude lag over that window is negligible.
       pos/vec are in the ship's Bullet-world basis -- the same basis as
       partPos/partAxis, NOT frame S -- and the active ship's frame IS the
       render frame, so they point the same way the ship draws. Gravity is
       deliberately not recorded: it always acts at the COM, so its arrow
       would be a constant.
       Recorded ONLY while recordForceVecs is set, so the sim hot path pays
       nothing when the overlay is off. */
    struct ForceVec {
        glm::dvec3 pos;  // application point, rel COM at record time
        glm::dvec3 vec;  // the force itself (N, ship Bullet-world basis)
        int kind;        // kForce* below; the drawer colors by kind
        /* #485: the angle of attack THIS surface read on the substep that made
           this force, degrees -- carried by value rather than by a part index
           because a stage in the same frame can reshuffle parts between recording
           and painting. Only meaningful for the surface kinds (lift, control). */
        float alphaDeg = 0.0f;
        bool alphaValid = false;
    };
    /* kForceDrag is the hull's FORM drag at the center of pressure; the induced
       drag a lifting or steering surface pays for turning the flow (#462, #494)
       hangs at THAT surface instead, so it is its own kind (#505) -- sharing one
       kind made the legend call a flap's induced bill "drag (at center of
       pressure)", which is the one place a player can ask what the arrow is.
       kForceSurfDrag is the same treatment for a surface's own PROFILE bill -- the
       shape of the plate, not the lift it earns (#455): planform x cd, at the
       surface. Three kinds of drag on the overlay because there are three places it
       pushes, and a legend that says "drag" for all three hides where the
       weathervane is coming from. */
    static constexpr int kForceThrust = 0, kForceRcs = 1, kForceDrag = 2,
                         kForceLift = 3, kForceChute = 4, kForceCtrl = 5,
                         kForceInduced = 6, kForceSurfDrag = 7;
    static constexpr int kForceKindCount = 8;
    std::vector<ForceVec> lastForceVecs;
    bool recordForceVecs = false;
    /* #485: record each surface's own alpha/q (Part::aeroAlphaDeg, aeroQ). The
       LIFT loop already computes them, so recording there is free; the CONTROL
       loop's neutral early-out exists to NOT build the flow and mount frame for a
       surface nobody touched, which at shipped part counts is most of its cost --
       so a neutral surface's angle is looked up only while a readout is open (the
       F4 overlay or --drag-log). Set per ship per frame in tick.cpp, same shape as
       recordForceVecs. */
    bool recordSurfaceAero = false;
    /* The per-substep boundary: the tick.cpp loop that runs the appliers
       opens every substep with this (named, so tests run the real thing). */
    void beginForceRecordSubstep() {
        if(recordForceVecs) { lastForceVecs.clear(); }
    }

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

    /* --drain-log: thrust this tick (N) + each group's drain rate (kg/s) over
       the sim time drainSample() has covered since the last print. Prints
       nothing when nothing has stepped, so it cannot repeat a stale row. */
    void drain_log(double time);

    /* --drain-log's sampler: call it EVERY tick the sim steps this ship, with
       how much that tick advanced sim time, so the rate's interval is sim time
       whatever the warp (#376). */
    void drainSample(double step);

    /* --drain-log: forget the baseline and whatever accumulated. Call it
       whenever buildFuelGroups() has just renumbered the groups (a decouple or
       a dock), because the previous sample then describes different tanks. */
    void dropDrainWindow();

    /* Per-group fuel mass (kg) summed over the ship's tanks, EC excluded. A
       group whose tanks are all empty still appears, holding 0.0. */
    std::map<int, double> groupFuelMasses() const;

    /* --dv-log: the running delta-v budget (see the dvDelivered_ block). */
    void dv_log(double time);

    /* Disarm the armed thrust (once per tick). */
    void clearThrust();

    /* Likewise the armed rotation commands. */
    void clearRotCmd();

    /* Control moving to ANOTHER ship: zero the throttle, clear the armed cmds. */
    void releaseControl();

    /* Parts WOULD be dropped if `stage` fires: each decoupler there plus the
       subtree it anchors. The decoupler itself IS dropped. */
    std::vector<Part *> droppedPartsAtStage(int stage);

    /* The decouplers a stage press separates, in the order their split ships get
       made: shallowest first, and ties in build order (index in `parts`). Not a
       detail -- the first one keeps the bare name and the rest get " #2", " #3",
       and ship-list order is what the next-ship handoff follows, so the tie has to
       come from something stable. See the note on the definition (#540). */
    std::vector<Part *> stagedDecouplers(int stage);

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

    /* The dock gate's own door (#583): "" if folding `B` into this ship gives a hull
       the game can hold, else the sentence to say instead. A ship's mass total is
       refused where it is built -- a def at load (#547), a save at load -- and a
       dock is the ONE place in play that ADDS two of those totals into ONE Body.
       Both hulls fit on their own and can still sum past a double, and the merged
       compound then hands Bullet an infinite mass: the NaN inertia and NaN centre of
       mass that #547 refuses a save over, reached by playing. Ask this BEFORE
       absorbShip: the alternative is the assert in rebuildCompound, which stops a
       debug build mid-flight and leaves a release build with an unrotatable hull.
       `B` is another live ship (the caller's gate has already ruled out null, self
       and a dead target). */
    std::string refuseDockWith(Vehicle *B);

    /* Fold B into this ship at the mated port. B's shell is deleted by the
       caller, B's journal is dropped (#49). B's crew ride along because a seat
       belongs to its capsule Part, which moves with the part tree. */
    void absorbShip(Vehicle *B, Part *portA);

    /* Extract the subtree rooted at `root` into a new Vehicle, rebased into a new
       frame S' = the root's old frame. The new ship inherits frame/home/sun, takes
       the root's current world pose and the dropped side's COM velocity. nullptr if
       `root` is null, is not aboard this ship, or reaches every part (a split needs
       two sides). The whole-ship case is decided by counting the subtree the walk
       reached, not by reading one parent edge off the root, so a caller that has to
       explain a refusal asks this function instead of answering it twice (#541). Seams
       maintained like fuel links. NOT in the physics world -- the caller enters it. */
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
    /* THE CONTROL STATE (#575). Command() is the only thing that writes this from
       input, and the rules it owns -- SAS engage/toggle, the normalized attitude
       demand, the throttle ramp -- are exactly why it is the funnel. The other writers
       each have a name, so a new one is visible rather than merely legal:
         applySlewRequest()    tick.cpp's latched -> engaged copy, once a tick, with its
                               "paused sim engages nothing" gate
         restoreSlewRequest()  a save loading a latched mode: a fact, not a key press
         armRcs()              the armed translation demand, unit: Kerbal::armInput()
                               arms it for a suit (no RCS parts to Command) and
                               Command's own Rcs* cases set their signs in rcsRequest
                               and route that whole request back through it
         markRcsBurned()       a thruster that actually fired (pods, the suit's jets)
         clearRotCmd()         the per-tick release of the stick and of held KillRot
         clearRcs()            the per-tick release of the translation demand
         releaseControl()      handing the ship to another vessel: it releases the
                               demand too, because the per-tick clearRcs() only runs
                               for the ACTIVE ship (#580)
       Protected, not private, because Kerbal IS-A Vehicle and reuses the rotation and
       RCS laws (eva.h) -- so this stops a stranger, not a subclass, and the suit writes
       through the doors above rather than the fields. That is the honest limit of the
       change: the fields no longer advertise "assign me", but the rule that keeps a
       subclass honest is still the one written here. Leaving them public is what cost
       #424, where the Autopilot window, the Z/B keys and --autopilot each re-implemented
       the toggle rule because nothing said they could not. #425 step D is putting an
       allocator in this same corner, clamping once against the capability model, and a
       writer that bypasses it is a ship that turns harder than the player asked for. */
    float stick[3] = {0.0f, 0.0f, 0.0f};   // manual demand: roll / pitch / yaw, -1..1
    SlewMode slew = SlewNone;              // ENGAGED this tick -- see slewMode()
    SlewMode slewRequest = SlewNone;       // latched on the Autopilot window
    /* The aim a law named for SlewFree (#606). Per-STATE, not per-tick intent like
       stick[]: it outlives the tick so a law that stops re-stating keeps the last aim,
       exactly as a latched mode keeps running -- and it is saved with the latch, because
       a loaded SlewFree with no vector is a mode that steers nowhere. */
    glm::dvec3 slewFreeDir_ = glm::dvec3(0.0);
    glm::dvec3 rcsDir = glm::dvec3(0.0);   // armed translation demand, unit; 0 = off
    /* The RAW request the keys describe, before it is made a direction: one axis each,
       so holding Nose+Up+Right asks for (1,1,1). rcsDir is armRcs's answer to THIS on
       the ship path -- renormalizing the previous answer instead hands the LAST key
       the full share and shrinks the ones armed before it, so three keys thrust along
       (.707,.5,.5) instead of the even diagonal the keys describe (#580). Same
       request/answer split as slewRequest above. The suit is not in this pair:
       Kerbal::armInput arms a render-frame direction straight, because its keys are
       screen axes and it has no RCS parts to Command (eva.cpp). */
    glm::dvec3 rcsRequest = glm::dvec3(0.0);
    bool rcsFiring = false;                // burned since this tick's clearRcs()
    void markRcsBurned() { rcsFiring = true; }
    /* Arm the translation demand from the whole request: a direction, never a
       magnitude -- the rating is the thrusters', not the keys'. */
    void armRcs(const glm::dvec3 &bodyDir) {
        assert(std::isfinite(bodyDir.x) && std::isfinite(bodyDir.y)
               && std::isfinite(bodyDir.z) && "RCS demand is a direction");
        const double l = glm::length(bodyDir);
        rcsDir = (l > 1e-9) ? bodyDir * (1.0 / l) : glm::dvec3(0.0);
    }

    /* The throttle, 0..1 (#400 F3). The FIELD has no other writer: the only writes go
       through adjustThrottle() (Command's ThrottleUp/Down -- the player's rate) and
       setThrottle(). The setter is public because two honest writers live outside the
       class -- the save loader (a saved value is a legitimate value) and
       releaseControl() (a handover leaves the lever at zero) -- but a per-tick writer
       should send ShipCmd(ThrottleSet), because Command's door is also where a paused
       tick writes nothing. Nothing else, which is the point: a public field here is how
       a law's setpoint and the player's keys ended up writing the same float with
       nobody able to tell which one won. */
    float thruster_util = 1.0;
    /* Did a throttle KEY fire this tick? Per-tick intent, like stick[]: cleared by
       clearThrust() at the top of every tick and set by Command's ThrottleUp/Down.
       Deliberately NOT set by ThrottleSet, which is the law's channel. */
    bool throttleKeyHeld_ = false;

    // Forces/torques straight on the Bullet bodies, reachable only through
    // Command(). Protected: the EVA kerbal reuses the rotation-model helpers.
    void adjustThrottle(float delta);

    /* Once per tick: consumes the fuel and arms the per-thruster thrust
       (applyThrustForce applies it per substep). A thruster that can't consume
       its flow doesn't thrust. Stage gates WHEN it ignites, the fuel group WHAT
       it burns. */
    void ApplyThrust(double step);

    // --- physical rotation model -------------
    /* The inertia tensor about the COM in world axes (Bullet stores it DIAGONAL
       in the principal frame). O(1). */
    glm::dmat3 getInertia();

    /* The target direction in the ship's frame for the current slew mode; radial
       / normal reference the SOI body. */
    glm::dvec3 slewTargetDir();

    /* What the slew law DEMANDS, N m, for one substep of aligning the nose (local
       +Z) with `dir`: target rate is the braking curve sqrt(2*alpha*E) capped at
       E/(2h), rate change bounded by alpha*h. It does not actuate:
       applyRotationForce sums this with the stick's demand and runs the allocator
       ONCE, because a second trip through it bills the wheels twice (#420). The
       capability comes from the caller's AttitudeAuthority -- one answer per
       substep, shared by the stick and both laws (#594's first half; the [slew]
       trace still builds its own, which is the half still open). */
    glm::dvec3 slewDemand(glm::dvec3 dir, double h, const AttitudeAuthority &auth);

    /* What kill-rot demands for one substep, N m: tau = I * (-w) / h, which the
       allocator bounds against the caller's AttitudeAuthority. Needs the FULL
       inertia tensor, not the world-axis diagonal. */
    glm::dvec3 killRotDemand(double h, const AttitudeAuthority &auth);

public:

    // --- physical rotation model -------------
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

    /* Inside the body's PHYSICAL atmosphere NOW (a density model about sea
       level, drag.h). A visual-only rim does not count. */
    bool inAtmosphere();

    /* Resting on the surface NOW: in the rotating surface frame, near-static in
       it, within a generous band of the analytic terrain. Both are needed:
       either alone calls a hover "landed". */
    bool isGrounded();

    /* Rail-eligible: GROUNDED freezes in the rotating surface frame; anything
       clear of the air coasts on its conic. Periapsis is deliberately NOT a gate
       -- a departure burn can drop it below the surface while the ship is high
       and climbing. railsTick stops a conic at the air/ground instead. */
    bool canRail();

    /* Park out of the world and coast analytically. False if not rail-eligible. */
    bool goOnRails();

    /* Re-enter physics from rails: rebuild the Bullet state. */
    void leaveRails();

    /* Per-tick rail advance: propagate the conic, test the END of the advance for
       an SoI boundary, hand off there, refresh the parked transforms. The test
       only sees the endpoint, so a long step can fly past a whole sphere.
       A coast that reaches the air or the ground leaves the rails here, so the
       caller owns the warp drop. */
    RailEnd railsTick(double t, const double step);

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
       name resolves either way: --startship, saves and the
       e2e fixtures all name dev beds, so this gates the picker, not the
       scenario. No default: adding a bed means deciding this. */
    bool dev_only;
    bool on_pad;
    double alt_frac; // circular: fraction of the near-body shell (rot SOI -
                     // TerrainBody::seaLevelRadius()), or of the atmosphere
                     // top if atmo_frac
    bool polar;
    int ell_phase;   // -1: circular; 0: periapsis; 1: apoapsis; 2: 90 deg
    double peri_alt; // ellipse: periapsis ABOVE MEAN SEA LEVEL (m), i.e. a
                     // radius minus TerrainBody::seaLevelRadius() (#404)
    double apo_alt;  // ellipse: apoapsis above mean sea level (m)
    double esc_frac; // escape: launch speed in local escape velocities (0 = not escape)
    double abs_r;    // > 0: absolute circular-orbit radius from the body
                     // centre (m), overriding alt_frac
    bool atmo_frac = false; // alt_frac scales the atmosphere top() instead, so
                     // the flying beds land in their science band
    /* Which plane a non-polar orbit bed sits in (#182). False (default) is the
       body's RAIL plane, where its system-file rails live; true is its EQUATOR. */
    bool eq_plane = false;
    /* Circular beds only: the bed's speed as a fraction of circular speed
       MEASURED IN THE RESOLVED FRAME, so on a body's rotational frame it is a
       fraction of AIRSPEED -- the air co-rotates with the ground (drag.h), and
       applying the fraction to the INERTIAL circular speed instead leaves
       Kerbin's slow bed with 16 m/s of air: a drop, not a start. 0 (default) =
       circular, which is what makes an in-atmosphere bed hold its altitude band
       with no lift at all (#99) -- and why every flying bed enters at km/s, i.e.
       a re-entry slam rather than flight (~700 g of drag on the shipped jet at
       the instant it spawns on Kerbin flying-low). A slow fraction instead starts
       level, in air, at a fraction of that body's orbital speed -- scale-free, so
       it never means escape anywhere, but the same fraction sits above stall on a
       thick atmosphere and under it on a thin one (a glide). Which body to spawn
       it over is the dev's call: it is a bed, not a mission (#451). It ignores the
       ship: a bed is a position + velocity, and lift is the ship's problem. */
    double speed_frac = 0.0;
    /* NB: there is deliberately no authored angle of attack here. A bed is a
       position + velocity, and an airframe's attitude is not a launch condition it
       can keep: a stable hull rotates to its own zero-moment AoA inside a second.
       Measured on flying-cruise: tipping the launch attitude 8 deg nose-up leaves
       1.2 deg at t=1 and merges with the untipped trace's 1.6 deg by t=2 (#569).
       What decides whether a flight bed is trimmed is the airframe's incidence or
       trim, not the bed's spawn attitude -- res/ships/README.md step 3 does it with
       12 deg of elevator. The Spawn line prints the launch AoA anyway: it reads 0.0
       BY CONSTRUCTION (spawn_vehicle points the nose along the velocity with
       faceAlong()), so it is a canary for wind or for a bed that does author an
       attitude, never a trim readout. */
};

/* Look up a scenario by name; throws if unknown. */
const ScenarioDef *scenario_by_name(const std::string &name);

// Enumerate the scenario table (the VAB dropdown); index < scenario_count().
size_t scenario_count();
const char *scenario_name_at(size_t i);
bool scenario_dev_only_at(size_t i);

// Nose (local +Z) along `dir`; roll axis = the axis most orthogonal to it.
glm::dmat3 faceAlong(const glm::dvec3 &dir);

/* Same, but the ship's +Y takes `up_ref` (projected out of `dir`) instead of a
   picked axis. `up_ref` must not be parallel to `dir` -- asserted. Use this
   whenever the roll means something: the axis pick above is DISCRETE, so a
   degree or two of tilt in `dir` can flip it a full 90 deg (#451). */
glm::dmat3 faceAlong(const glm::dvec3 &dir, const glm::dvec3 &up_ref);

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

/* --mass-log: one line per VEHICLE, so a fleet's mass can be added up. The
   active ship alone cannot: a decoupler hands its subtree to a NEW Vehicle
   (extractSubtreeAsShip), so unless that survivor prints its own mass there is
   no way to check a split moved the mass it moved (#367). */
void mass_log(Vehicle *ship, double time);
