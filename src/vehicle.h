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
#include "fuelgraph.h" // FuelLayers: the drain layers the live drain walks (#965)
#include "sas_policy.h" // SasPolicy: pilot vs. law (#421)
#include "losses.h"    // AscentLosses: what a burn spent, not what it did (#675)

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/norm.hpp>

#include "btcommon.h"
#include "actuators.h"  // ActuatorRow (the wheel rows of AttitudeAuthority)
#include "inlinevec.h"  // InlineVec: the storage the authority rows live in (#668)
#include "body.h"
#include "physics.h"
#include "drag.h"     // SilhouetteMemo (the rest of the drag law: vehicle.cpp)
#include "equirect.h" // kNoCoord (SpawnPlace's lat/lon pair); the law itself: vehicle.cpp
#include "rcs.h"      // the RCS nozzle allocator + per-substep Scratch
#include "shipdef.h"
#include "staging.h"  // StageRow / StagingSeed: the staged plan for a live ship (#948)
#include "part.h"
#include "terrain.h"
#include "frame.h"
#include "orbit.h"

/* The authored atmosphere as drag.h's view of it -- the ONE conversion, shared by
   the aero pass and any readout that wants the air's T/p/a (#552 A2). It takes the
   BODY, not a gravity, because the air needs two facts about where it sits: the
   field at its datum (`g`) and the radius that datum sits at
   (seaLevelRadius(), which is what the column is weighed in -- #683). Handing them
   over separately is how you get a profile resolved against one planet and read
   against another, and the pair has no legal mixed spelling: the column's altitude
   is measured from the same sphere its g belongs to.
   On an EXPONENTIAL body rho(alt) and top() do not use g, so passing a different g
   moves a readout and never a drag number -- but with #683 both bodies' rho aloft DO
   follow the radius, since the column thins more slowly the closer its top sits to
   the surface. A LAYER body weighs its column rather than authoring it (#552 A4), so
   its rho aloft follows g too (pinned in tests/test_drag.cpp: same sea-level air,
   half the gravity, thicker air at 20 km).
   The layer table is BORROWED, not copied: DragAtmosphere::layers points into a's own
   vector, so `a` must outlive whatever this returns. It does everywhere -- the levels
   are resolved once by the loader and live in System::bodies for the whole world, and
   no caller stores a DragAtmosphere (the aero pass builds one per hull per substep).
   Caching one across a system reload would be a dangling read, not a stale copy.
   It copies fields; it does not transform any of them -- including the composition
   the loader resolved (molar_mass, gamma, oxidizer_share), so the jet gate reads them
   off the profile the density came from instead of going back to AtmosphereParams for
   half the pair. That fold is #637, and tests/test_scenarios.cpp refuses a body whose
   dragAtm lost one of those four. */
DragAtmosphere dragAtm(const AtmosphereParams &a, const TerrainBody &body);

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

/* One spelling of what a mode is CALLED (#616): the [slew] trace's mode= field and the
   line Game::select_vehicle prints when a mission parks were keeping separate word
   lists for the same state, which is how a log and the CHECK that reads it drift apart
   without anything noticing. Keep these words -- e2e cases match on them. */
inline const char *slewModeName(SlewMode m) {
    switch(m) {
        case SlewNone:       return "none";
        case SlewPrograde:   return "prograde";
        case SlewRetrograde: return "retrograde";
        case SlewRadialOut:  return "radial-out";
        case SlewRadialIn:   return "radial-in";
        case SlewNormal:     return "normal";
        case SlewAntiNormal: return "anti-normal";
        /* Named, not the fallback: an e2e CHECK on a free-aim law reads these rows,
           and a nameless row cannot be told apart from a construction bug (#606). */
        case SlewFree:       return "free";
        case SlewKillRot:    return "killrot";
    }
    return "slew";   // an int outside the enum, which Command() cannot produce
}


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

/* What a start ship asked about its AIR bed's place: a map point (lat/lon) and/or an
   altitude. Both or neither for the pair; kNoCoord in any slot means "not asked" and is
   tested with isnan(), never with == (a half pair is a parse bug, asserted).
   Degrees, the same pair and the same convention as a world_buildings pad entry:
   equirect.h is the game's ONE (lat,lon) -> direction law and it speaks the body's
   ROTATING frame, so the point means "here on the map, now", not "here in the rail
   frame". It places a bed, not a pose: the bed keeps its altitude and speed, turns
   so its circle passes over the point, and carries its own velocity direction with
   it (bedTurnTo) -- the heading stays prograde in the tilted plane, it is not a
   ground bearing. A ground bed already HAS a place -- the pad site's lat/lon -- so an
   on_pad entry refuses the pair rather than overriding a building.
   `alt_m` [m above mean sea level, the datum every spawn log's alt_asl speaks] replaces
   the height the scenario authors -- its fraction of the near-body shell, of the air top
   (atmo_frac), or its pinned abs_r -- because the asked number is the one the caller
   measured, and the bed's own height is what they are changing. It takes the SAME law as
   the fraction it replaces: the bed stays circular, so its speed follows from the radius
   (and a load-keyed bed's airspeed from the air AT that height, which is why asking
   above a body's air top refuses there rather than here). build_start_vehicles refuses it
   on a pad bed (the ground sets that height) and on an ellipse bed (which authors two
   apsides, so one number cannot place it). */
struct SpawnPlace {
    double lat_deg = kNoCoord;
    double lon_deg = kNoCoord;
    double alt_m = kNoCoord;
    bool asked() const { return !std::isnan(lat_deg); }
    bool altAsked() const { return !std::isnan(alt_m); }
};

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
    /* Where a start ship asked its AIR bed to sit (--startship lat=/lon=,
       --startships "lat_deg"/"lon_deg"). Empty for every other route into the
       world -- a VAB launch, a load, a dock test -- because those either stand on
       a pad or already carry a placed world state. Read once, by apply_scenarios. */
    SpawnPlace place;
    TerrainBody *sun = nullptr; // the star (light source)
    /* Per-tick "an engine fired" flag (ApplyThrust sets it; tick.cpp clears it). It has
       to START at zero: nothing between a ship's construction and the first tick's clear
       may read an indeterminate value through isThrusting() -- the deck carry and any
       movement-intent law read it, and #849's test rig caught it reading garbage. */
    float m_thrust = 0.0f;

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

    /* What the ship presents to the air (#463, #945). AREA is rotation-invariant, so
       the whole ship is projected through ONE flow plane in frame S;
       `aeroSilParts` holds each part's silhouette polygon in that shared 2-D basis,
       reused between substeps. Not "the CURRENT flow's" polygons, though: a chart
       HIT writes nothing here, so on a hitting substep these are whatever the last
       MISS projected, and only the cached bill is current. `aeroSilPartAtt` is the
       parallel array of what each of those polygons brings to the bill -- its cd as
       it faces this flow, how far upwind it reaches, and where it pushes -- because
       #945 charges the coefficient and the centre of pressure on the area the air
       can SEE, not on each part's own. The union of the polygons -- not a convex hull
       drawn over the merged vertices, which invents airframe between the parts -- is
       the drag area, and it is the SAME number the coefficient is averaged over.
       SilhouetteMemo caches the whole bill per flow DIRECTION, filling nodes lazily
       and going stale by generation counter when the geometry changes, so a hitting
       substep projects nothing at all.
       Geometry = the part set + their mounts + their drag anchors: mass (fuel burn,
       the usual reason refreshCompound rebuilds) does NOT change the silhouette, so
       the cache invalidation is fingerprint-gated. */
    std::vector<std::vector<glm::dvec2>> aeroSilParts;
    std::vector<SilhouettePart> aeroSilPartAtt;
    /* The bill's PARTITION, polygon-for-polygon with aeroSilParts: each part's
       visible (unoccluded) area in the union, and that area x its cd as the flow
       faced it (#945's audit columns, #1090's live ones). Current on a HIT as well
       as a MISS -- blended from the chart nodes -- while the polygons beside them
       are not; that is what lets a body part's Form row (and a future heating or
       stress law) read per-part figures at any cadence. `aeroSilPartPtr` is the
       third positional twin, filled by projectSilhouette, so the copy onto the
       Parts cannot drift from the order that built the columns. `aeroSilPartProj`
       is the fourth (#947): each part's OWN projected shadow area beside its
       credits, so `1 - vis/proj` says how much of that shadow the union refused
       -- and cancels to exactly zero for an exposed part, hit or miss.
       `aeroSilPartCp` is the fifth, sixth and seventh (#1106): the part's credit
       times its own projected centroid, in the same frame S cpNum lives in, so
       cp == aeroSilPartCp[i] / aeroSilPartVis[i] is where THAT part's share of
       the union's pressure sits -- and the weighted sum of the parts' copies IS
       the cpNum the axial force acts at, hit or miss (products blend linearly). */
    std::vector<double> aeroSilPartVis;
    std::vector<double> aeroSilPartCdA;
    std::vector<double> aeroSilPartProj;
    std::vector<glm::dvec3> aeroSilPartCp;
    std::vector<Part *> aeroSilPartPtr;
    // Scratch for filling cache nodes: those polygons belong to the cell
    // corners' flow directions, not the substep's, so they need their own set.
    std::vector<std::vector<glm::dvec2>> aeroSilNode;
    std::vector<SilhouettePart> aeroSilNodeAtt;
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

    /* LANDING GEAR: the hull's gear wheels (Part::isGearWheel), rebuilt with the
       geometry cache so staging/docking re-group for free; the ground pass only
       reads. Empty for a wheel-less hull, which makes applyGroundForces a no-op. */
    std::vector<Part*> gearParts;
    void groupGearParts();

    /* LANDING LEGS: the hull's footpads (Part::isLeg), same rebuild discipline
       as gearParts. The ground pass stands them with the brake held on. */
    std::vector<Part*> legParts;
    void groupLegParts();

    /* Per-substep scratch for the induced bill (#556): the lift each surface made,
       held until every surface has contributed, because the price is the SYSTEM's
       CL^2/(pi e AR) and the system's CL needs the whole sum. One reused vector --
       no allocation in steady state, and the sample count is the number of surfaces
       that actually made lift this substep. */
    struct AeroLiftSample {
        Part *part = nullptr;           // the surface that made it (#730 bills it per part)
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

    /* The compound's inertia tensor in WORLD axes (#1053). Bullet keeps the solve's
       diagonal in the hull's PRINCIPAL frame plus the pose that carries it -- the same
       pair refreshMassProps() audits against Bullet's cached inverse (#438) -- so this is
       two rotates, not a solve. Public because anything that moves angular momentum
       through a merge has to know what each hull carried into it. */
    glm::dmat3 hullInertiaWorld() const;

    /* The spin this hull would have if it carried the world angular momentum `L` about
       its own COM: w = I^-1 L, done in the principal frame (two rotates and three
       divides) instead of solving a 3x3. An axis whose diagonal is zero is immovable and
       answers no spin for it, which is what Bullet does with such a body too. */
    glm::dvec3 spinForAngularMomentum(const glm::dvec3 &L) const;

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

    /* Drain layers per fuel group, dense by group id, walked on first use and dropped
       by buildFuelGroups. The map this replaces billed a tree node per cached group;
       the shape is the staging preview's CSR, from fuelgraph.h, so the live drain and
       the preview walk the SAME code (#965). An empty layerStart means "not walked
       yet". A caller may hold the returned reference across its own loop because no
       consumer does it across a buildFuelGroups() -- the hazard is not vector growth
       but a silent drop, so a future caller must re-ask rather than keep a slot
       through anything that could renumber. */
    mutable std::vector<FuelLayers> drainLayers_;
    /* rev[Y] = { X : X feeds Y }, from fuelLinks, built once per renumbering by the
       first layer ask rather than once per group as the map version did. That is
       only sound because every writer of fuelLinks (spawn, save load, the VAB build
       record, absorbShip, extractSubtreeAsShip, stagedRows' hidden Sim) renumbers
       before that ship drains, so a numbering never sees a link edit. The map
       version partly self-healed by rebuilding rev per missed group; if a live link
       edit ever becomes legal, this graph wants the same version counter #973 is
       debating for hull mass. */
    mutable std::vector<std::vector<int> > drainRev_;

    /* Which parts of each fuel group are TANKS, as a CSR (#974): group g owns
       memberOf_[memberStart_[g] .. memberStart_[g+1]], holding indices into `parts`.
       The drain loops used to sweep EVERY part of the hull once per reachable group to
       find them -- O(parts x groups) per ask, so a 286-part stack with ten groups
       looked at 2,860 slots to touch the ~40 tanks it wanted, again per engine per
       substep. The staging preview has kept this table since before #965; flight now
       keeps its own, built by buildFuelGroups() through the shared CSR helper
       (#984, #534). Members stay in `parts` order within a group, so the drain sums land
       bit-for-bit where they did with the scan. tests/test_fuel.cpp pins the table
       against a plain scan, including after a renumbering, and tests/test_dock.cpp and
       tests/test_contain.cpp pin it again after a merge and a split -- the two places a
       renumbering is easy to forget.
       `Part::fuelGroup` has exactly one writer in src/, buildFuelGroups() (a test stamps
       one by hand, and says so: tests/test_staging.cpp); what has many writers is `parts`
       ORDER, and every one of them renumbers before the ship drains again: the loader and
       spawn, init/finalize, absorbShip, extractSubtreeAsShip, the save load, an
       EVA-minted hull, a dropped item. A hull emptied to nothing takes an empty cache
       with it (finalize()'s early return, and the shell absorbShip leaves behind), so
       these indices never outlive the parts they name. */
    std::vector<int> memberStart_;   // nGroups + 1 boundaries
    std::vector<int> memberOf_;      // part indices, grouped, `parts` order within a group

    /* The group's slice of that table, with the one bounds rule (#984). The group id is
       not the hazard: it comes from FuelLayers::groupOf, and drainLayers_ and memberStart_
       are both rewritten by buildFuelGroups(), so an id outside the table cannot be
       reached. What the table ADDS over the old hull scan is an index into `parts` that
       outlives a shrinking part list if a writer ever forgets to renumber -- so that is
       what gets checked. memberOf_ ascends within a group, so its last entry covers the
       whole slice: one compare per group, at the top of the loop that walks it. */
    void groupSlice(int grp, int &begin, int &end) const {
        assert(grp >= 0 && grp + 1 < (int)memberStart_.size());
        begin = memberStart_[(size_t)grp];
        end = memberStart_[(size_t)grp + 1];
        assert(begin == end || memberOf_[(size_t)end - 1] < (int)parts.size());
    }

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

    /* --dv-log state: the TSIOLKOVSKY side of the budget, in m/s since the ship
       existed. The delivered side is not stored here. The impulse the integrator
       was handed, and its share along the velocity line, live in losses_ below:
       lossTick() writes them once per substep, and losses_.dv - losses_.steer IS
       the along-velocity figure. Two members integrating the same force in the
       same file were two owners of one integral, which is how [dv], [ascent] and
       [rec] would start pricing one flight differently the day either changed its
       mass basis, frame or burn gate (#933). Times |v| the along figure is the
       orbital energy the burn bought, but only for a constant |v|: a case brackets
       it between min|v| and max|v| instead.
         dvRocket_ integrates the Tsiolkovsky budget over the propellant the
         engines actually drained (ve * ln(m_before/m_after), per engine per
         tick -- ApplyThrust).
       dvRocket_ and losses_.dv share the part table's ve and propellant rate
       (PartDef::fullThrust() IS rate * ve), so their agreement does NOT prove
       the part table -- it proves the two sides agree on the MASS basis (the
       hull Bullet accelerates vs the analytic getMass()), on the CLOCK basis
       (the substep h summed vs the tick's step), and on the throttle-down
       factor s and exhaust_scale, each of which must land on both sides.
       LIMITATION: this is the MAIN-ENGINE budget. RCS pushes go through
       applyRcsForce() and jets drain their own fuel without a vacuum dv, so
       neither reaches it and dv_del reads 0 for an RCS-only nudge (#368). */
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
       (save.h) because it is a property of THIS vessel, unlike sas_policy. The handle is
       the Systems window's checkbox (#649); the mask existed first so that the allocator
       would be honest and the toggle would have somewhere to land. */
    bool wheels_enabled = true;
    /* The same door for the control surfaces (#649): switched-off fins are absent from
       the capability model exactly as an unpowered drum is, so the laws, the slew budget
       and the Vessel rows all see the smaller answer instead of a plan whose setpoints
       go nowhere. It does NOT deflect the plates by force: a closed door emits no rows,
       so `surfaceLat_` carries nothing for applyAeroForce to spend. Trim still applies
       (a trimmed plate is a build fact, not a steering command).
       Saved with the ship for the same reason as wheels_enabled. */
    bool surfaces_enabled = true;
    /* The same door on the RCS ring's ATTITUDE half (#595, and the switch #649 could
       not build until there was an authority figure to print beside it): with it shut
       the bells are absent from the capability model, so SAS and the stick point the
       hull on wheels and gimbals only and never drink hydrazine to turn it.
       It gates ATTITUDE only -- the translation keys still fire the ring whatever this
       says, because they are what the player pressed and #595 is not allowed to
       switch off the maneuver they asked for. The rows carry the difference rather
       than a scattering of ifs: no rows means no law, no readout and no setpoint can
       promise a turn the player has declined to pay for (#418's lesson, again).
       Saved with the ship, like the other two. */
    bool rcs_attitude_enabled = true;
    /* The same door on the nozzles (#768): with it shut the gimballed stage emits no
       TVC authority in attitudeAuthorityRows(), so the mixer aims nothing sideways
       (gimbal.h tvcSplit sees tvcMax == 0, takes 0 N m and returns a zero lateral
       vector), the laws size a turn on wheels and fins alone, and the nozzles walk back
       to neutral at their own gimbal_rate instead of freezing wherever the stick left
       them -- which is what an actuator does when you stop commanding it.
       It gates STEERING only, never thrust: the throttle still does what the player
       asked, and an engine mounted off the COM still torques the hull while it burns.
       That is rigging, not steering, and switching it off would be switching off the
       build. A mechanical gimbal needs no EC power, so unlike wheels this door has no
       second "no power" reason to read empty. Saved with the ship, like the others. */
    bool tvc_enabled = true;

    /* How fast an attitude LAW may swing this hull, rad/s; 0 = no ceiling (#678).
       It clamps what the law WANTS -- slewDemand's w_des -- and never the actuator
       rows, because alpha/alpha_full are the honest capability answer (#572, #608)
       and the stick stays full-authority by #425. Clamping the want is what buys the
       coast: once the hull is at the ceiling the law asks for nothing, so the
       actuators let go and only the wind-up and the brake burn cost anything. The
       brake point stays the law's own (sqrt(2*alpha*E) falling under the ceiling), so
       it moves whenever the authority moves under it -- wheels saturating, air thinning,
       the throttle closing, #595 taking the bells for a translation key, or #1124's
       attitude dial dimming what the ring may spend on pointing.
       Out of reach on purpose: kill-rot (no w_des there -- it pushes as hard as the
       rate it is killing, so it is already self-limiting) and the translation keys
       (the player's own hands, #595's precedent).
       Saved with the ship like the other doors. 0 means UNLIMITED, not "the smallest
       setting"; the floor and the top of the range belong to the slider
       (gameui.cpp), not to this field -- the sim takes any non-negative figure. */
    double sas_turn_rate = 0.0;

    /* How hard this hull's RCS KEYS may push, as a fraction of what this ring can
       deliver ALONG the direction being asked for -- rcsReachableAlong() (rcs.h),
       the sum of every bell's forward projection onto that demand (#1119: the
       bells are one-way and bounded one at a time, so no hull has one ceiling in
       newtons). The referent is therefore per-direction: the same dial position
       buys more newtons on a diagonal where several bells project, fewer on an
       axis the ring is thin on. Read it as a dial position, never as newtons;
       lim=NN% rides the [rcs] debug row so a dialed hull is visible there.
       1.0 = full authority (the old behaviour), 0.0 = the keys fire nothing -- the
       translation OFF switch rcs_attitude_enabled deliberately never was (that
       door buys pointing, not pushing, gameui.cpp). It caps the
       DEMAND in rcsPlan only: the bells keep their ratings, so the attitude solve
       gets MORE room beside a light nudge, not less, and the fuel draw follows
       the firing pro rata (applyRcsForce derives flow from the same fractions).
       The handle is the Systems window's slider; --rcs-frac is its CLI twin.
       Saved with the ship and carried across seams, like the other doors. */
    double rcs_thrust_frac = 1.0;

    /* #1124: the same handle for the ring's POINTING half. The dial above only
       dims the keys, and #595's priority means dimming them makes steering
       STRONGER (the bell the keys no longer spend is left to the attitude walk),
       so neither door could say "steer with half this ring".
       It multiplies what each bell has LEFT for pointing in
       attitudeAuthorityRows() -- (1 - c_i) * frac -- which is one expression
       covering the live authority figures, what a law plans against, the stick's
       own ask (the #425 switch's model, NOT #678's, which never touches the
       stick) and what applyRcsForce spends, all at once and all in proportion.
       The bells' RATINGS stay untouched: attitude authority is priced from them,
       and a dial that shrank them would re-price the hardware promise too --
       attitudeAuthorityHardware() ignores this field, so the Systems row and
       [steerlog]'s rcs_hw= still say what the ring IS worth. att_lim=NN% rides
       [steerlog] to say what it is being allowed to do.
       0 is the dimmer's end, not the switch: the rows stay PRESENT with a zero limit,
       so it is still a ring the hull has. How visibly that reads depends on the law.
       kill-rot asks whenever any actuator rows exist at all (its guard is every row
       set being empty), so there the shortfall reads tau_ask>0 / tau_spent=0 --
       flight/1124-d pins that. An
       alignment law budgets its ask from alpha, and alpha is 0 with nothing to lever,
       so at 0 % it asks nothing and looks exactly like the switch; that is honest, the
       torque really is unavailable either way. What must NOT collapse is the rows
       themselves: dropping them at 0 % would silence kill-rot too, and the switch is
       the only thing that should do that. */
    double rcs_attitude_frac = 1.0;

    double dvRocket_ = 0.0;
    double propSpent_ = 0.0;       // kg of rocket propellant drained
    double dvVeLast_ = 0.0;        // last ve used, exhaust_scale applied (m/s)

    /* What the burns this hull has flown COST, in m/s (#675). This is also the
       only record of what they DELIVERED: losses_.dv is the thrust impulse the
       integrator was handed, and losses_.dv - losses_.steer the part of it along
       the velocity line, which is what --dv-log prints as dv_del and dv_along
       (#933). dvRocket_ beside it is the tank's side of the same transaction.
       lossTick() is the only writer, once per substep (losses.h says why the
       arithmetic lives there and not at each reader). It is NOT in the ship save
       and there is nothing to restore: a load builds new Vehicles, so the counters
       start at zero with the hull they describe (#304). */
    AscentLosses losses_;
    const AscentLosses &ascentLosses() const { return losses_; }

    /* EC gate for the wheels (powerTick). Default true: no EC system = ungated. */
    bool powered_ = true;
    /* The watts powerTick struck its last balance on: gen - constantDraw -
       activeDraw, the sum behind netWh. The Systems window and --power-log print this
       instead of adding the three terms up themselves (#659): a bill term that
       powerTick pays but getPower() does not report would otherwise make the number
       the pilot reads wrong in silence, which is #642's mistake shape.
       It is a RECORD, not a live sum, and the two are not the same animal:
       drainEC/chargeEC clamp at empty and full, so a hull parked dry prints this
       rate while its pool no longer moves. Staleness is bounded by one powerTick --
       every substep off rails, once per warp hop (railsTick), and once a frame while
       the world is stopped (tick.cpp) -- EXCEPT that a Vehicle which has never
       ticked at all carries 0.0. Docking is the one part change that lands after
       the substep loop (tick.cpp's updateDocking), so on that frame the terms can
       be post-dock while this is pre-dock. ui/601's identity CHECK between this and
       the printed terms is what would notice, and it can only notice at a dump. */
    double net_power_w_ = 0.0;
    /* The brown-out latch (#642). A hull whose generator cannot pay the wheels' bill
       drains its pool, the wheels cut, the unspent surplus recharges the pool, and
       they re-arm -- free-running, that is a relaxation oscillator at the SUBSTEP rate,
       whose duty cycle depends on how finely the sim happens to step rather than on
       anything in the game. So the cut LATCHES: once the pool cannot pay a short burst
       of commanding the drums stay cut until it banks enough Wh to command for
       kWheelsRecoverS -- or, on a store too small to ever bank a whole maneuver, until
       it is half full, so the margin can never strand a hull. Both thresholds are
       durations, never the caller's step: off rails the step is bounded by kMaxSubStep
       so a step-scaled threshold looks harmless, but a railed hull is billed one hop at
       a time (#647) -- a minute-long hop at this rating is ~41 Wh, a max-warp hop is
       ~46 h and ~100 kWh -- so a step-scaled cut would brown out every warped hull the
       moment it left Kerbin. kWheelsCutS is exactly that old bound, kMaxSubStep: the rule
       is now stated instead of inherited from the timestep, and it moved the off-rails
       cut point 1.8x earlier at the largest legal substep (18x at the 1x default), which
       in Wh is 0.069 rather than 0.004 for 2475 W of drums. The cycle's period is the
       power balance's, not the timestep's: on the probe hull (a 1007 W rtg against 600 W
       of air and 2475 W of drums) that is about 12 s of steering per minute of waiting,
       which reads as "this generator cannot sustain it" -- the honest answer, and one
       the player can answer by letting go.
       Both figures above are the SATURATED case, and #648 made that worth saying: the
       drums now bill what they deliver, so a latched law that has arrived spends ~1 W
       while this latch still weighs the whole 2475 W rating. That is deliberate -- the
       gate is a build question and must not read the bill (#642) -- and #654 asked whether
       it therefore strands a hull that is only asking a watt. Measured: not on any hull
       that has a store. The burst the cut wants is one max substep of the rating, so the
       latch closes at 0.069 Wh for 2475 W of drums -- 0.0625 Wh for the biggest shipped
       wheel package, 0.135 Wh for the worst shipped hull (the heavy_* builds, 4850 W) --
       and the worst ratio in the shipped roster is 6.7e-5 of a store, 0.0067% of a 2 kWh
       capsule battery. The Systems row prints those shut charges as `0.1 Wh` (its format
       is %.1f); flight/647 reads 0.0 because a warp hop overshoots to dry, which is the
       hop quantum rather than this threshold. The ASK is what decides how long a hull
       flies to get there (test_power: 720 s of authority for an arrived law against 65 s
       for a pinned key, on one battery). What the rating does cost is the re-arm bank --
       100x the burst: 6.9 Wh, which against the 407 W surplus of that probe hull is a 62 s
       hold, and 13.5 Wh / 121 s for the heavy_* builds. Note that the half-store
       backstop below never fires at shipped scale (it wants 360 kW of drums on a 2 kWh
       store), so the bank is the whole answer until the catalog grows into it (#895); and
       on a hull whose generator does not even cover its air, a dry pool never re-arms at
       all -- which is #652's problem rather than this latch's. */
    bool wheels_brown_out_ = false;
    static constexpr double kWheelsRecoverS = 10.0;   // s of commanding banked before re-arm
    static constexpr double kWheelsCutS = 0.1;        // s of commanding the pool must cover
    /* #652: armed by powerTick() on the substep or the warp hop that empties the pool,
       consumed by tick.cpp's watcher, which puts the words on it. Transient like
       powered_: a loaded hull re-derives its balance on the next tick, and a notice for
       a crossing the player already watched would be a false alarm about a hull they
       chose to park empty. */
    bool ec_dry_notice_ = false;

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
    /* The spin, in the SAME frame the rail state lives in, so a rails round trip is a
       no-op for it (#1042). It used to be DROPPED: writeRailPose() zeroed the hull, so
       parking and waking kept the attitude and destroyed the angular momentum -- most
       visibly at a body-fixed dock, where a hull that had stopped rotating relative to the
       dock holds a real 0.126 deg/s of inertial spin, and zeroing it leaves the hull
       counter-rotating against the dock it is trying to close on. The rails integrate
       the ATTITUDE with it (#1056), so a coasting hull keeps turning rather than waking
       at the heading it fell asleep at. What they still do not model is precession --
       for a hull whose three principal moments differ, omega itself wanders about the
       conserved L -- so the hull keeps the axis it parked with. A frozen park reads zero
       by construction -- its model pins the pose to the node -- as does the warp landing
       that parks one. */
    glm::dvec3 rail_omega = glm::dvec3(0.0);  // rad/s, ship->frame coords
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
    /* The lifting-surface forces: every lifting surface's slope force AND every control
       surface's DEFLECTION force (#699 -- a deflection is perpendicular to its own
       surface's flow and #494 bills it induced drag like any lift, so leaving it out made
       this the lift LAW's sum rather than the lift the air applies. A settled skiff glide
       used to print |L| 6042 N against a total aero force of 5402 N: more lift than the
       air's entire force, because the 12 deg tab trimming that hull pulls DOWN and only
       the control loop saw it). NOT a component of lastAeroForce: each surface feels its
       own flow (#483), so the two agree only as far as the hull's surfaces agree with
       each other -- on the yardstick skiff they now land 0.02 N apart. */
    glm::dvec3 lastLiftForce = glm::dvec3(0.0);   // slope force + deflection, vector sum
    /* The sum is DELIBERATELY a vector sum, where the induced bill below it is a scalar
       one (applyInducedDrag adds |L_i|/q_i, so a wing's upload and a tab's download add
       instead of cancelling -- that inconsistency is open as #562). For "how much is the
       air holding this hull up", cancelling is the right arithmetic. */
    /* The aero force SPLIT ABOUT THE AIR-RELATIVE FLOW -- L and D in the textbook
       sense, halves of lastAeroForce. lastFlowDrag is -(ftotal . flowHat), N (+ = the
       air is holding the hull back; it goes negative when a co-rotating frame or a
       parachute jink makes the air PUSH, which makes a ratio out of it meaningless
       rather than large), and lastFlowLift is the perpendicular remainder -- the FULL
       remainder, so with sideslip a fin's side force sits in it too: this is the
       flow-plane split of the force, not the wing's lift. The --drag-log's Ld= is
       lastFlowLift / lastFlowDrag (#694) because that is the ratio a glide ratio
       equals: in the shipped skiff's settled 200 s glide it prints 7.19 while the
       hull's own ground track says 41.20 m over 5.66 m of sink, 7.28.
       It also agrees with the row's other ways of saying the same thing, which is what
       #699 was for. On that glide: lastFlowLift 5350.2 N, |lastLiftForce| 5350.18 N,
       and q x (1.2 CdA + Induced + Profile) 745.0 N against this row's 744.0 N of
       lastFlowDrag. On flight/492's held trim, |lastLiftForce| over the CHARGED bills
       reads 7.17 median -- this row's Ld, median 7.17. The word charged matters: A= and
       Cd= print the RAW hull product while the force charges drag_cd x them (1.2 by
       default, as tick.cpp's legend says; Induced= and Profile= already carry it), so a
       ratio built from the printed numbers without that multiplier sits ~6 % high
       (7.66 median there). res/ships/README.md's skiff L/D of 8.6 was built that way
       AND on the pre-#699 lift sum -- stale, and which ratio it should lead with is
       still #694's open call. */
    double lastFlowDrag = 0.0;
    double lastFlowLift = 0.0;
    /* One gate for both readouts of the flow split (#694): --drag-log's Ld= and the F4
       aero window. False means dash, and it dashes on three counts -- no usable flow
       last pass, no along-flow force to divide by, and an along-flow force under a
       thousandth of the total (which makes the quotient meaningless rather than large).
       The pair alone is NOT safe to divide: applyAeroForce skips its AoA measurement
       below kAoAFlowFloor but still applies and records the force, so only this gate's
       lastDragAlphaValid term keeps the quotient off a hull with no flow to score. */
    bool flowLd(double &ld) const {
        if(!lastDragAlphaValid || lastFlowDrag <= 1e-6) return false;
        if(lastFlowDrag * 1000.0 <= glm::length(lastAeroForce)) return false;
        ld = lastFlowLift / lastFlowDrag;
        return true;
    }
    /* lastLiftForce . local vertical (away from the body centre): + = the airframe's
       lifting SURFACES are holding it UP -- wings and deflected tabs together since #699,
       so a hull trimmed by a heavy download reads its NET lift here, which is the honest
       figure and can in principle go negative on an airframe the tail is pushing down.
       Signed so the --drag-log can pin the lift DIRECTION, not just its magnitude -- a
       flipped AoA sign hid exactly that (#448), and |L| alone cannot see it. That test
       survives the fold-in: 0 of flight/492's 50 rows change sign, they only shrink. */
    double lastLiftVert = 0.0;
    /* The airframe's own up axis . local vertical: +1 upright, 0 on its side,
       -1 inverted. The --drag-log's guard rail -- L_up only means "the surfaces
       hold the ship up" while this is positive. */
    double lastUpVert = 0.0;
    glm::dvec3 lastAeroTorque = glm::dvec3(0.0);  // moment about the COM
    /* The aero pass's own read, last substep. lastDragAlt is metres above the
       sea-level DATUM (r - seaLevelRadius()), NOT above the ground and not above
       the bare radius, so it goes negative exactly where lastDragRho is at its
       maximum (#245). The F4 overlay and --drag-log print this pair; [surfinfo]'s
       alt_agl/alt_asl is the pair that says where the hull is. applyAeroForce
       zeroes the pair before its early-outs, so "no air", "no atmosphere" and
       "--drag-cd 0" all read 0/0 rather than last substep's value. */
    double lastDragAlt = 0.0;
    double lastDragRho = 0.0;
    /* The dynamic pressure the pass stood in, 1/2 rho v^2 at the COM with v the
       AIR-relative speed (#675) -- the ship-wide counterpart of the chute
       limiter's qpt, which is the same expression at the canopy's own flow
       (vehicle.cpp:3942), and of each surface's localFlow q. It is the number an
       ascent profile wants for its max-Q moment: the bill has always been paid
       per surface, and nobody stored the whole-ship figure. Zeroed with the pair
       above, so no air reads 0 rather than last substep's value. */
    double lastDynPressure = 0.0;
    /* The air the pass stood in, last substep (#552 A2/A3): what the body's profile
       says at this hull's altitude -- the temperature [K], the pressure [Pa] and the
       sound speed [m/s] that a Mach number is a ratio to. On an exponential body
       those are IMPLIED by its scale height and constant with altitude; on a layer
       body they are the profile (#552 A4) and all four go 0 at/above the air top,
       where there is no air to have a temperature in (#632). Plus lastAirV, the
       AIR-relative velocity's speed AT THE COM (the
       [drag] line's |v|= is the FRAME speed, which on a spinning body is not the
       same number, #97). It is the base vector every surface's flow is built from,
       not the flow itself: a surface's own q= and mach= divide localFlow's speed,
       which adds that part's omega x r. applyAeroForce zeroes all four with the
       pair above, before its early-outs, so "no air", "no atmosphere" and
       "--drag-cd 0" read 0 rather than last substep's value -- on the substeps this
       pass runs. A hull on the rails never runs it (tick.cpp skips the physics for
       a railed ship), so these four and the pair above hold their last pre-rails
       values while --drag-log keeps printing its 1 s rows (#627). */
    double lastAirT = 0.0;
    /* The profile's own temperature at the hull BEFORE any ground coupling (#724 D), so
       --solar-log prints measured numbers for both sides of the blend instead of
       recomputing one of them. */
    double lastAirProfileT = 0.0;
    double lastAirP = 0.0;
    double lastAirA = 0.0;
    double lastAirV = 0.0;
    double lastDragAlpha = 0.0;  // AoA (rad); + = nose above the flow (drag.h)
    /* The flow's angle off the nose in the YAW plane (rad), AeroFrame's other
       angle. Not a control readout (#450 wants it bundled with a yaw law) -- it
       is the silhouette cache's second key, so the drag log prints it. */
    double lastDragBeta = 0.0;
    /* Whether lastDragAlpha measured a flow worth reading: AeroFrame::valid AND
       the air-relative speed clears kAoAFlowFloor. False alpha is not "flying
       straight down the flow" -- it is "there is no flow to have an angle to". */
    bool lastDragAlphaValid = false;
    /* Whether lastDragBeta measured one too -- a DIFFERENT test from alpha's, not a
       copy. The speed floor that gates alpha says nothing about beta, whose yaw
       needs a projection onto the hull's nose-right plane to exist at all: a parked
       hull in wind clears the floor by miles and still has no sideslip, because its
       flow lies along the up axis (AeroFrame::betaValid, constants.h:kBetaIllCondDeg,
       #787). Nothing that prints or records beta asks both questions today, which is
       the bug; the forces ask neither, since no force reads beta directly. */
    bool lastDragBetaValid = false;
    double lastDragArea = 0.0;  // BODY silhouette facing the flow (m^2) -- since
                                // #455 the union refuses any part with lift_area or
                                // control_area, so this is what the hull stops, not
                                // what the airframe presents
    double lastDragCd = 0.0;    // the parts' cds, meaned over the area each one CONTRIBUTES
                                // to the union (#945) -- a part behind another does not vote
    /* The hull's whole along-flow bill THIS billed substep [W]: drag_cd x q x |vp|
       at the cp x (lastDragCd x lastDragArea) -- the same product the #945
       partition splits per part, so it EQUALS the sum of the formDragWatts columns
       (pinned by test_aerosweep). #1114: the part-window body cards share ONE
       worst-frame publish per hull (HullFlowPub below), and this is that frame's
       measure. lastFormQ/lastFormMach are the flow the bill is struck in -- lfCp
       at the cp (#486) -- at their ship-level home: #1113 first repeated them per
       part, and two cards then held two frames of "one" hull flow. Zeroed at pass
       start and on rails with the form columns. */
    double lastFormWatts = 0.0;
    double lastFormQ = 0.0;      // [Pa] dynamic pressure at the cp, this substep.
                                 // Exactly 0 while the flow row lives is the
                                 // rotation sweep cancelling the COM flow at the
                                 // cp (#486) -- a real zero at the billing point.
    double lastFormMach = 0.0;   // Mach in that same flow
    /* #1114: the hull's flow row is ONE figure, so the worst-frame window, the
       publish tick and the published sample live on the hull, not per card. Card
       holds produced two q's for one hull (572 vs 455 Pa on ui/1090-a's skiff),
       and a card opened mid-window can never see frames that predate it -- a
       per-ship sample is the only thing both cards can always agree on, and the
       per-part columns of the SAME frame travel with it so each card's rows keep
       multiplying back. UI-cadence state on the model has precedent (stagedRows_,
       #948): drawPartWindows is this struct's only reader and writer, and the
       model merely stamps the candidates above and the per-part form* columns.
       It dies with the hull; with no part window ever open it is a few doubles
       nobody touches. */
    struct HullFlowPub {
        double nextRead = 0.0;            // ImGui::GetTime() of the next publish
        unsigned long long updFrame = 0;  // last ImGui frame this was refreshed
        double worstW = 0.0;              // the window's best hull bill so far
        double worstAlpha = 0.0, worstQ = 0.0, worstMach = 0.0;
        /* the parts' columns OF that best frame, indexed into parts; a size
           change (pickup, drop) invalidates the window's arrays */
        std::vector<double> winVis, winCdA, winWatts;
        /* #1105: the crossflow pair joins the published frame so the card's Form row and
           its flow row quote ONE substep, not a bill from one frame and a weathervane
           reading from another. Note WHICH frame: "best" ranks the AXIAL bill
           (worstW <- lastFormWatts), so a card's side figure is the axial-worst frame's,
           not the frame where SideCdA peaked. One hull, one frame, one row triple -- the
           ship's SideCdA= column is where the peak lives. */
        std::vector<double> winSideRefused, winSideCdA;
        /* the published sample every card prints until the next publish */
        double pubAlpha = 0.0, pubQ = 0.0, pubMach = 0.0;
        std::vector<double> pubVis, pubCdA, pubWatts;
        std::vector<double> pubSideRefused, pubSideCdA;
    } hflow;
    /* The hull's weathervane lever: (cp - com) . nose, metres; + = cp AHEAD of the
       COM. #486 made hull drag answer to the flow AT cp, and two different things
       ride on that. The sweep of the flow OPPOSES a rotation whatever the sign of
       this; what the sign decides is the other half -- hull drag acting AT AN ANGLE
       of attack sends the nose back into the flow only with cp BEHIND the COM, and
       diverges it with cp ahead. Printed as CpL= in the drag log because it is the
       quantity #454's static margin is made of and nothing else shows it.
       #945 made this lever longer: the push now lands on the parts the air can see,
       so on a nose-first stack it sits at the nose caps rather than at an area blend
       that included what was hidden behind them. On the ascending rocket that is
       +0.80 -> +3.30 m. A stack of frontal discs really does push at its front; what
       is missing is the SIDE of a hidden body paying for the crossflow over it,
       which is the aft push this wants and what #947 is for. */
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
    /* #947: the hull's CROSSFLOW NORMAL force -- what the side of a body pays
       when the axial union billed it nothing. A part's share is its mesh-box side
       strip (drag.h::sideStripCdA, 2 L D sin(alpha) for a round body) discounted
       by the shadow the axial partition already credited it, times the authored
       drag_side as the normal coefficient, at the part's own mount and its own
       flow. lastSideCdA is the whole bill (m^2); a zero reads honestly three ways
       -- every part exposed, the flow along the parts' axes (the sin lives in the
       strip), or no body parts at all -- so SideCp only means something beside a
       nonzero SideCdA;
       lastSideCpLever is the arm of its centroid along the nose (+ ahead, like
       CpL) -- on a stacked rocket the hidden tanks and engine sit it AFT, which
       is the aft push #945's cp never had. Deliberately NOT folded into
       CpL/FormCp: normal force and axial drag are perpendicular forces, and one
       CdA-weighted lever over two lines of action is a number that cannot
       reconstruct a torque. */
    double lastSideCdA = 0.0;
    double lastSideCpLever = 0.0;
    /* #1097: the hull's BODY LIFT -- the perpendicular force body parts make on
       their own EXPOSED silhouette (drag.h::bodyLiftCdA; the law and its sin cos
       kernel live there; the #947 split is a complementary GATE -- this law acts
       on the visible fraction, #947 on the refused -- not a shared area
       measure). lastBodyCdA folds the sin in (signed sum of drag_cd x body_lift
       x cos(a) x sin(a) x visible) so q x BodyCdA is the summed force's SIZE the
       way q x SideCdA is #947's: it reads 0 at alpha 0 because the force IS 0
       there, and negative reads honestly as net downforce -- but two parts with
       opposite cos(a) can cancel it to 0.00 while forces still act, the same
       cancellation the SideCp floor guards. It is 0 for every hull whose parts
       leave body_lift unauthored -- every flight-pinned hull -- though the
       catalog's liftbody_4m lets a VAB player opt in. lastBodyCpLever is the
       force-weighted arm (+ ahead, like CpL/SideCp); beside a near-cancelled
       sum it means nothing. Since #1106 the force acts AT the arm: each part's
       lift lands on its PROJECTED CENTROID (aeroSilPartCp / aeroSilPartVis --
       cached per chart node, hit or miss, so the line of action never depends
       on cache history), and the lever weights by that same point. What the
       projection cannot recover is the along-flow part of the offset (#512's
       caveat). Still NOT in CpL/FormCp/Induced: body parts never join the
       SURFACE lift systems force-side and never stamp aeroValid/aeroQ (#730).
       The SLOPE half changed with #1106 though: a part's d(force)/d(alpha)
       joins CLa/SM (vehicle.cpp hullSlope -> LiftSlopeSum::add), so "SM -" no
       longer means "this lifting hull says nothing about trim". */
    double lastBodyCdA = 0.0;
    double lastBodyCpLever = 0.0;

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
    bool wheelsPowered() const { return ecPowersWheels() && wheels_enabled; }
    /* The ELECTRICAL half of that door on its own (#649). The row builder asks the two
       halves separately: the live model asks this plus the enable order, the hardware
       model the Systems window quotes asks ONLY this -- a dead battery is not something
       a switch gives up, so it has to still close the hardware rows. Named rather than
       written as `powered_` at the call site so the power half keeps one spelling even
       where the handle does not apply. */
    bool ecPowersWheels() const { return powered_; }
    /* What the sim balanced on its last powerTick, in W (#659). Read this rather than
       subtracting gen/const draw/active in a window: the readout then cannot drift from
       the bill, and a new term the sim pays without reporting reddens ui/601 instead of
       making the pilot's number wrong in silence. */
    double netPowerW() const { return net_power_w_; }
    /* Is anyone ASKING for attitude authority this tick? Note what this does NOT
       answer: since #648 the wheels' EC bill is what the mixer DELIVERED, so this says
       a command is live, not that a command costs anything -- a converged law with the
       key still down is engaged and spends ~nothing. powerTick() gates its bill on
       exactly this, and the Systems window's Active row prints it too (#649 took the
       balance out of the Game Debug overlay, which is how #646 got fixed). An unpowered
       or switched-off hull's wheels are not drawing even with the stick pinned, and a
       second spelling of the rule is how those two drift apart. `mixerRuns()` is in it
       because a hull coasting on rails has no mixer pass at all (#647): tick.cpp's
       substep loop skips railed hulls, so their wheels turn nothing however hard the
       stick is held. That is not merely defensive either -- Game::updateProximity parks a
       distant hull without releasing its control state, so a railed hull really can be
       carrying a live slew. The bill and `--power-log active=` have to say the same
       thing the physics did. */
    bool actuatorsEngaged() const {
        return wheelsPowered() && mixerRuns() && rotCommanded();
    }
    /* Is there a control pass this step? Exactly "not coasting on rails" -- the rails
       path advances a conic and applies no forces, so nothing that spends authority
       runs. Its own name because the EC bill has to ask it too, and a `!onRails`
       written inline there is how a railed hull ends up charged for drums that never
       turned. */
    bool mixerRuns() const { return !onRails; }
    /* Is a rotation command live this tick: the stick or a latched mode. One
       spelling because the bill and the gate both have to ask it, and a second
       spelling is how they drift. */
    bool rotCommanded() const {
        return stick[0] != 0.0f || stick[1] != 0.0f
            || stick[2] != 0.0f || slew != SlewNone;
    }
    /* W the actuators are billing right now: what the wheel housings DELIVERED, and
       zero when they are not engaged. Not their rating (#648) -- a drum asked for 4 N m
       of its 4500 N m pays 4/4500 of its package figure, so the EC cost tracks the
       turning rather than the keypress. An autopilot that has arrived, a demand about a
       line no drum covers, a hold the gimbals are doing: all of those used to eat the
       pool at full rating. The figure is per HOUSING still: a triad (#593) pays its one
       `power_draw` when its busiest drum is at full demand, not three times it, because
       the catalog's watt figure is the package's while `torque` is per drum -- pricing
       the drums individually would triple every shipped wheel with no author asking.
       powerTick() pays this and the Systems window's Active row shows it, so the row
       cannot disagree with the bill (#575) -- it used to re-derive the sum, and dropped
       the power gate while doing it. */
    double wheelDrawWatts() const;
    /* What the wheels COST this hull: the same rating sum behind the enable mask only,
       whether or not anyone is steering (#642), and whether or not anyone can PAY for
       that much at once. powerTick's gate asks THIS and never wheelDrawWatts(), because
       that figure is the gate's own verdict: a shut gate bills 0, so a gate reading the
       billed sum re-arms the substep after cutting the wheels. The live command stays
       out of the GATE for the same kind of reason -- a hull that cannot afford its drums
       is short of power whether or not anyone is holding the stick, and a verdict that
       moves with the stick is how the Power row starts promising authority that the next
       keypress refuses. #648 moved the DELIVERED figure onto the bill, not onto the
       gate: the gate answers the build question "could this hull ever work these drums",
       the bill answers "what did this substep cost". */
    double wheelDemandWatts() const;
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
    /* WHAT THE STEERING ACTUALLY SPENT THIS SUBSTEP (#599). Every attitude number the
       player could read is a RATING -- the wheel rows, the fin rows, #608's gimbal
       pair -- which answers the build question, "is this drum enough?". The question
       that reads the same rows is "what is turning the hull right now". The N m is in
       world axes throughout, so the channels sum and each projects onto a ship axis:
         asked:    what the stick and the latched laws summed before the split. Not what
                   a law WOULD like: killRotDemand/slewDemand answer 0 when the hull has
                   no authority at all, and a law's demand is sized off the RATE error
                   (I*dw/h), so it is warp- and substep-scaled, not a hardware wish.
         wheels:   the N m ApplyTorque got.
         tvc:      the torque the gimballed engines' thrust produced about the COM,
                   summed in applyThrustForce off the same force it bills. Measured, so
                   it follows the throttle, the gimbal RATE (a ring a held key has only
                   just started to swing reads low) and a mount's own tilt.
         surfaces: the torque applyAeroForce's commanded plates returned, from THE AIR:
                   0 in vacuum, drooped past stall, swept by the hull's own turn (#426's
                   budget-not-delivery caveat), and including whatever trim is holding.
       asked minus the sum of the three is the torque the HARDWARE refused: #577's
       over-subscribed multi-key hold, a demand about a line no drum covers, a law
       driving a saturated actuator. It is not a guaranteed positive, and that is a fact
       about the actuators rather than a bug in the record: a gimbal KEEPS its aim between
       substeps, so a hull that has come round (a law asking almost nothing) is still
       being turned by the nozzle the last substep left deflected -- flight/96 measures
       2.95x the ask on two such rows. The channels come from three different passes of
       one substep (control, thrust, aero), so a pass that did not run leaves ITS channel
       at zero, which is the honest answer -- a hull at drag_cd 0 spent nothing on fins.
       Cleared by applyRotationForce with tvcLat_ and surfaceLat_ and by goOnRails beside
       the F4 overlay's clears: a railed hull runs no substeps, so it shows nothing spent
       rather than the last live hull's numbers. A PAUSED sim runs no clear either, so the
       Vessel window keeps the last simulated substep -- the edge tick.cpp documents. */
    struct AttitudeDelivery {
        glm::dvec3 asked = glm::dvec3(0.0);
        glm::dvec3 wheels = glm::dvec3(0.0);
        glm::dvec3 tvc = glm::dvec3(0.0);
        glm::dvec3 surfaces = glm::dvec3(0.0);
        /* The RCS ring's share, since #595. Billed in applyRcsForce off the ATTITUDE
           fraction of each bell's firing -- the translation fraction a key asked for is
           lastRcsAch's business, not this record's. */
        glm::dvec3 rcs = glm::dvec3(0.0);
    };
    /* The last actuated substep's record. Read it, do not write it: actuateAttitude,
       applyThrustForce, applyAeroForce and (since #595) applyRcsForce own the numbers,
       applyRotationForce the clear. */
    const AttitudeDelivery &attitudeDelivery() const { return attDel_; }

private:
    AttitudeDelivery attDel_;
    /* W the wheel housings are billing for the LAST ACTUATED SUBSTEP (#648): each
       housing's package `power_draw` times how hard its busiest drum was asked to work.
       actuateAttitude fills it from the same solve that produced the setpoints;
       applyRotationForce clears it with tvcLat_, surfaceLat_ and attDel_, so a substep
       nobody asked for pays for nothing. powerTick reads it one substep LATE -- the tick
       bills power before control forces so the EC gate is current (#647), so the Wh paid
       this substep are what the previous one delivered. At kMaxSubStep that lag is 0.1 s
       of sim, and it is the same "last actuated substep" reading the #599 record uses. */
    double wheelSpendWatts_ = 0.0;
    /* The solve's work buffers (#645) -- the same bargain rcsScratch makes below. An
       actuated substep used to pay ten fresh vectors for a handful of rows: 91,830
       allocations in a 42,000-tick Kerbin ascent (10.0 x 9,183 actuateAttitude calls).
       actuatorSolveInto resizes these instead, so only the capacity grows. attRows_ is
       the wheels-then-surfaces concatenation the ONE solve walks: #426 needs one index
       space over both, and the wheel rows stay first so the setpoint offsets in
       actuateAttitude stay stable. Both belong to actuateAttitude alone, and its two
       solves -- drums and fins, then #595's bell mop-up -- take the Scratch in turn, so
       nothing may carry a drum or fin setpoint past the mop-up. */
    actuators::Scratch attScratch_;
    std::vector<ActuatorRow> attRows_;

public:
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
           #418); the Game Debug window prints what these rows buy per axis, which
           is the same gated answer the laws spend (#572: one answer, not the one the
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
       these levers are measured from.
       That makes tvcTorque the SPENDABLE half of the answer, and it has a twin:
       tvcTorqueAvailable is the same disk with the ignited stage at full throttle
       (#608). A law that sizes a turn with the spendable number finds it shrinking
       under its own throttle law -- 2 % of the lever on a TVC-only hull is 2 % of the
       steering, and before a burn's first ApplyThrust it is flat zero -- which is real
       behaviour, but is not the question "could this hull turn at all". Both numbers
       are in this one snapshot so the pair cannot be derived twice (#400's rule). */
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
           wheels are electric, #418), and the Game Debug window prints this same
           gated answer per axis (#572: one answer, not the one the largest wheel
           happens to give). */
        /* The row sets are small-buffer vectors (#668), not std::vector, because this
           struct is a VALUE: attitudeAuthorityRows() rebuilds it every actuation
           substep and again once per frame per open window, and a heap buffer owned by
           a value dies with every one of those calls. heaptrack 2026_11_06 counted
           85,386 such allocations per 20 s of atmospheric flight -- 65% of that run's
           whole steady-state total, and still 24,216 once they were pre-sized with
           reserve(). kInlineRows is the room a shipped hull fits in, so the common
           rebuild allocates nothing at all; past it the set moves to the heap and keeps
           growing, so a hull with 30 fins gets the same physics for one malloc per
           rebuild rather than a truncated row set. It is not roomy: the shipped fleet's
           largest wheel set is EXACTLY 12 rows -- three drums each on four housings,
           res/ships/heavy_one.json &co -- and its largest surface set is 5 (the skiff
           family), so one more drum block puts `wheels` on the heap. */
        static constexpr size_t kInlineRows = 12;
        /* RCS needs its own size because it counts a different population: one row per
           NOZZLE, and rcs_box authors 5 face bells (res/data/parts.json), so the four
           pods of res/ships/rcs_test.json -- the shipped fleet's biggest ring -- is 20
           rows. 24 covers that with a pod of margin. Larger is not free: a row is 40
           bytes and the buffer lives INSIDE the struct, so sizeof(AttitudeAuthority)
           is 1,824 at inline 12, 2,304 at 24 and 2,944 at 40 (compiled and read out of
           the type), and every hull in the game pays that much authority object whether
           or not it carries a ring. What the bytes cost is footprint, not a memset --
           the -O2 code for attitudeAuthorityRows() initializes the vectors'
           bookkeeping, not the whole inline region, as the disassembly shows -- so 24
           buys the shipped fleet's ring and stops there. 40 fits only the 8-pod rigs
           under tmp/, which are not content.
           Overflow is not a cap (InlineVec grows onto the heap, #668), and the price of
           overflow is measured: the 40-bell rig pays the same ~4,500 allocations per 25 s
           of RCS flight at inline 12 and at 24 -- one malloc per rebuild, exactly as
           #668 promises -- while rcs_test's 20 bells go 4,503 -> 0. tests/ pins both
           row counts and both capacities. */
        static constexpr size_t kInlineRcsRows = 24;
        InlineVec<ActuatorRow, kInlineRows> wheels;
        /* The housing each wheel row came from, in `wheels` order. Same reason the
           surfaces carry surfaceParts: actuatorSolve's setpoints come back indexed by
           row, and #648 bills a HOUSING -- PartDef::power_draw is one package figure
           for a housing's drums, not a per-drum figure (#593) -- so paying the delivered
           setpoint needs to know which rows share one figure. A hull that bought two
           wheel blocks has two groups, and they bill separately because they are two
           packages. #579's momentum store wants this row->part link per drum too. */
        InlineVec<const Part *, kInlineRows> wheelParts;
        /* How many HOUSINGS those rows came from, counted where they are counted as
           rows. actuateAttitude groups `wheels` into runs of one owner and asserts this
           equals the run count, so the grouping is checked rather than trusted (#648). */
        int wheelHousings = 0;
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
           from (and the Game Debug window reads the same rows, which is why #599's
           budget-not-delivery caveat applies to fins as much as to drums). */
        InlineVec<ActuatorRow, kInlineRows> surfaces;
        InlineVec<const Part *, kInlineRows> surfaceParts;
        /* Whether the hull CARRIES a steerable surface at all, whatever the sky, the
           airspeed or the aero master switch are doing (#649). An empty `surfaces` has
           two different reasons and a player needs the difference: no plates fitted is a
           build fact, no air is a flight fact. The row set alone cannot say which -- it is
           empty either way -- so the row builder records the fitting while it walks the
           parts, rather than a readout walking them a second time. */
        bool hasSurfaces = false;
        /* #595: the RCS ring as actuator rows -- the fourth actuator, and the only one
           that works everywhere: vacuum, no wheels, no burn. Until now a bell could
           only ever push the hull sideways, and the ring's torque rows in rcs.h entered
           at torqueWeight 0.05 with no demand behind them -- trim, "fire and try not to
           spin" -- so a wheel-less pod coasting past its own prograde was credited with
           no turning authority at all while its thrusters fired on the same ship.
           ONE ROW PER NOZZLE, at the same index as the enumeration buffers (rcsNoz,
           rcsNozPart, rcsNozLocal), because the setpoint that comes back has to find
           its bell again: actuateAttitude writes a per-nozzle firing fraction, and
           applyRcsForce spends it. A nozzle whose arm x dir is degenerate keeps its
           slot with a zero axis -- the solver skips a zero-axis row, and the index
           stays honest -- which is why there is no parallel `rcsParts` vector here.
           The rows are ONE-WAY (actuators.h): a bell winds a turn up and cannot brake
           one the other way, so its limit is a fraction of ITS OWN rating and the
           credit is cosine-clamped.
           THE PRIORITY, which is the part a merged solve would get wrong (#595 fix
           idea 2): the player's translation keys own the bells FIRST -- rcsPlan() runs
           before the attitude walk and spends them -- and a row's `limit` here is the
           fraction the translation solve LEFT, not the ring's full rating. So pointing
           the hull can never cost the player a translation they asked for, in the same
           spirit as tvcSplit handing TVC's transverse remainder to the wheels. In
           hardware mode (the Systems switch row) the limit is the whole rating: the
           question there is what the ring is WORTH, not what the held key left over.
           A nozzle is credited only when the hull could actually fire it: propellant in
           the tank, the switch open, and -- unlike wheels and gimbals -- it is asked
           LAST, after the wheels and plates have had their share, so a hull that can
           turn for free turns for free and hydrazine pays only for what they cannot do
           (actuateAttitude). */
        InlineVec<ActuatorRow, kInlineRcsRows> rcs;
        /* Whether the hull CARRIES an RCS pod at all, whatever the tank or the switch
           say (#649's lesson, one level up from hasSurfaces): "no ring fitted" and "the
           ring is empty" are different sentences to a player, and an empty row set
           answers neither. */
        bool hasRcs = false;
        /* Whether the hull CARRIES a gimballed engine at all, whatever the switch and
           the stage say (#768, the same fitting hasSurfaces/hasRcs record): "no
           gimballed nozzles on this hull" and "no thrust to vector with" are different
           sentences to a player, and a zero tvcTorque answers neither. */
        bool hasGimbals = false;
        /* N m the ARMED gimballed engines can lever this substep. */
        double tvcTorque = 0.0;
        /* N m the IGNITED gimballed stage could lever at FULL throttle (#608): the same
           levers and the same sin(gimbal_max), rated thrust (partThrustCeiling, so a jet
           is air-gated) instead of what the last ApplyThrust armed. A RATING in the
           available* family: it does not consult the tanks (#607) and it does not follow
           the lever. It is also a SUM over the ignited engines, so it inherits #639's
           open question: a stage with one fed and one unfed engine is rated as if both
           could hold the lever open.
           Wheels and control surfaces need no twin -- neither is throttled. What has no
           twin is heffMag, the N m / rad that turns a torque into gimbal angles: a plan
           can ask what a turn is WORTH at full throttle but not yet what it COSTS in
           degrees of deflection, which is the cone-availability question #419 owns. */
        double tvcTorqueAvailable = 0.0;
        double heffMag = 0.0;
        glm::dvec3 stackAxis = glm::dvec3(0.0);
    };
    AttitudeAuthority attitudeAuthority();
    /* The same rows asked of the HARDWARE rather than of what is currently allowed
       (#649): the player's enable switches are ignored, everything else -- EC power,
       air, the aero master switch -- still applies. A switch row has to say what
       flipping it gives up, and attitudeAuthority()'s own answer is already zero once
       the door is shut, so it cannot quote itself. One row builder behind both
       (attitudeAuthorityRows), so the two answers cannot drift apart. Callers want one
       of these two wrappers, not the parameter. */
    AttitudeAuthority attitudeAuthorityHardware();
    /* One row builder behind both wrappers above, and private on purpose: the boolean is
       a mode, and a caller that picks it by eye is how the two answers start meaning
       the same thing in one place and something else in another (#575). */
private:
    AttitudeAuthority attitudeAuthorityRows(bool switchesApply);
public:
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
    /* The same sum with tvcTorqueAvailable in place of tvcTorque (#608): N m this hull
       could bring about `axis` with the IGNITED stage at full throttle. It is the rating
       a plan would size a turn with, beside the availableAccel() it sizes a burn with --
       a law that trims the lever to keep q down otherwise finds most of its steering
       trimmed with it, and finds it gone outright before a burn's first ApplyThrust arms
       anything. No law plans a turn rate yet, so today its readers are the [slew] trace,
       the Vessel window, and pointingTo()'s free-axis choice (which line the hull owns is
       a build question, not a this-tick one). It is NOT spendable -- actuateAttitude
       answers to tvcTorque, because a nozzle that is not pushing levers nothing. */
    double availableTurnTorque(const AttitudeAuthority &auth,
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
        /* + turns facing toward dir [unit]. At E == pi the cross product vanishes and
           the axis becomes a CONVENTION -- the hull's own right, or its up if nothing
           levers the right -- rather than a measurement, so a reader must not compare
           or blend axes across ticks near antipodes (#609). */
        glm::dvec3 axis = glm::dvec3(0.0);
        double angle = 0.0;                     // E [rad]; 0 == aligned
        double Ieff = 0.0;                      // kg m^2 about axis
        /* rad/s^2 the actuators can hold about `axis` THIS TICK: the TVC term rides the
           thrust the last ApplyThrust armed, so it is 0 on a coasting TVC-only hull and
           it shrinks with the lever (#608). The laws brake against exactly that, on
           purpose -- a nozzle that is not pushing levers nothing. Ask
           Vehicle::availableTurnTorque for the hull's turn authority rather than this
           tick's. */
        double alphaMax = 0.0;
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
       It also owns two records that outlive the call: the #599 spend record (the
       wheels channel) and the wheels' EC bill for this substep (#648) -- powerTick
       pays that figure, so this function moves the battery even though it looks like
       a pure actuator call. Both are reasons it stays the ONE call per substep.
       applyRotationForce answers ONE summed demand and calls this once, so that aim
       is the whole answer rather than one consumer's share of it (#420). */
    void actuateAttitude(glm::dvec3 torque, const AttitudeAuthority &auth);
    glm::dvec3 lastThrustForce{};  // debug: total thrust force applied last substep
    /* The gravity force the same substep applied (applyGravity's answer, kept so the
       loss split in losses.h can read one substep's three forces from one place).
       NOT the fictitious terms applyGravity also pushes: those are the frame's, not
       the body's pull, and gravity loss means the body's pull. */
    glm::dvec3 lastGravityForce{};
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

    /* The engine's fuel groups in LAYERS by hop distance, furthest first (src/fuelgraph.h).
       A layer drains TOGETHER (pro-rata). Cached; do not mutate the reference. An
       engine walled off by a fuel barrier (fuelGroup -1) gets an empty FuelLayers: no
       layers, no walk, and every layer loop below runs zero times. */
    const FuelLayers &fuelDrainLayers(Part *engine) const;

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

    /* The same sums for SEVERAL resources in one walk (#934). Each
       availableResourceMass call re-sweeps the engine's layers, groups and the whole
       part list to total one resource, and a rocket asks for two -- so a caller that
       wants the whole propellant set asks once. `want` and `out` are
       ResourceType::Num long; `out` is zeroed here and only the wanted slots are
       written. The per-tank order is the same as availableResourceMass', so a sum here
       is bit-for-bit the sum that function would build. */
    void layerFuelTotals(Part *engine, const bool *want, double *out) const;

    /* How much of ONE TICK's draw the pools can cover -- the share both ApplyThrust
       branches size their burn with (#638). 0 means no burn this tick: nothing available,
       or a zero-length/zero-desired tick (which is what the rocket branch has always
       treated as no burn). 1 means the pools cover it, and in between is the last sliver:
       an engine throttles its own final tick instead of being refused outright, which a
       drain-or-nothing consumeResourceMass needs its caller to do. NaN fails CLOSED --
       an availability that is not a number is not a burn. */
    static double burnShare(double available, double desired);

    /* includeJets counts jet fuel too. */
    void enginePropellantMask(bool *burn, bool includeJets) const;

    double fuelMassMasked(const bool *burn) const;

    /* The figures a player plans a burn with, in double: the fuel they are made
       of is stored in double (#379), and narrowing on the way to the readout
       throws that precision away -- ULP(0.0078kg) at 97t, ~1kg at 10kt (#382).
       ONE ideal burn of everything the tanks hold, at the ve of the FIRST rocket
       in part order: cheap, and wrong in two ways on a mixed-Isp ship (#41, #948)
       -- it prices chemical propellant at the nuclear engine's Isp, and it treats
       fuel in tanks about to be jettisoned as propellant while their dry mass rides
       along as dead weight, so pressing a stage can make the number rise. The
       Vessel Info readout uses stagedRows() instead; this stays the coarse figure
       the recorder and the ascent heuristic budget with. */
    double getDeltaV();

    /* #948: the staged plan for the ship AS IT IS -- the same burn model the VAB's
       Staging window runs (staging.h), seeded from the fuel actually in the tanks
       and the mass the parts actually carry (crew and inventory ride the hull, and
       PartDef knows neither). Rows are the stage labels still aboard, in the order
       the stage counter will fly them, so their sum is what a pilot has LEFT.
       Not a law that a press lowers it: a press deletes one priced row AND re-prices
       the rows under it, because the deleted period was modelled as having spent fuel
       that engines on surviving tanks may still hold. On mars_explorer_mk1 the deleted
       row is worth 3022 m/s and the re-pricing about 1500, so the sum falls; a stage
       where that contest flips still jumps. #952.
       Cost is the VAB's: 84us for a small ship, 473us for a 240-group monster, so it
       is cached and recomputed only when the tree or the ship's totals move (see the
       definition -- and note the figure there lags a burn by up to ve x 0.2%). Never
       call it per tick from a new site without reading that gate. */
    const std::vector<StageRow> &stagedRows();

    /* How many times that plan has actually been walked (#966). The gate in
       stagedRows() is the difference between ~755 walks a flight and one per frame
       (~42,000), and a widened gate is invisible in the numbers -- it just costs. The
       count makes it testable instead of heaptrack-only, like ApplyThrust_TESTONLY. */
    uint64_t stagedRebuildCount() const { return stagedRebuilds_; }

    /* Sum of stagedRows(): the ship's remaining delta-v, staged. */
    double stagedDeltaV() {
        double dv = 0.0;
        for(const StageRow &r : stagedRows()) { dv += r.deltaV; }
        return dv;
    }

    /* TODO should be cached per frame */
    double getMass();

    /* Felt acceleration [m/s^2]: |thrust + aero| / mass, no gravity. */
    double feltAccel();

    /* Test-only: not for game code. */
    void ApplyThrust_TESTONLY(double step) { ApplyThrust(step); }

    /* --- electrical (KSP-style EC) ---------------------------------------
       A shared pool across the batteries. powerTick runs per substep, BEFORE
       applyControlForces: gates the wheels, balances RTG generation against the
       life-support and wheel draws. Units: W and Wh. EC has no mass.
       The order is not free and is deliberate: the gate's verdict is what
       applyControlForces spends this substep, so it has to be current -- and the
       price is that the wheels' term is the PREVIOUS substep's delivery (#648 pays
       what actuateAttitude solved, not a rating), i.e. at most kMaxSubStep stale.

       It is called twice a frame in spirit, once per path: per substep for a hull under
       physics, and once per warp hop from railsTick for a hull coasting on its conic
       (#647 -- before that, a warped transfer billed nothing at all). Nothing here
       needs to know which: the wheels' term is 0 on rails because `actuatorsEngaged()`
       knows the mixer is not running out there, while the GATE still weighs the drums'
       cost, because that is a property of the hull and not of the step. */
    void powerTick(double h);

    /* Drain / charge up to `wh` of EC, pro-rata across the batteries. */
    void drainEC(double wh);

    void chargeEC(double wh);

    void getPower(double *gen, double *constDraw, double *charge, double *capacity);

    /* Did this hull's EC pool just cross from charge to empty? Read-and-clear, so the
       tick layer toasts once per dry event rather than once per substep (#652). The
       crossing matters more than the state: a hull that HAS been dry since a warp hop
       needs no reminder every frame, while the hop that empties the pool can land in the
       middle of a 100000x transfer, seconds before the player tries to point the hull at
       a brake burn and finds the drums shut. powerTick() sets it -- the only place that
       knows the balance -- and says nothing itself, because Vehicle has no Game handle
       and a toast is the UI's word for a warning. */
    bool takeEcDryNotice() {
        const bool n = ec_dry_notice_;
        ec_dry_notice_ = false;
        return n;
    }

    /* Does this hull have an electrical SYSTEM, as opposed to an EC bar that happens to
       read zero? One spelling because three places ask it: powerTick() leaves the wheels
       ungated when the answer is no (#418), the Systems window says "nothing to pay,
       nothing to cut" rather than printing a column of zeroes that reads like a fault
       (#649), and the VAB prices an assembly before launch (#644). The caller hands over
       the figures it has ALREADY summed -- getPower() for a live hull, buildPower() for a
       build on the pad -- so the rule has one home without adding a part walk. */
    static bool ecSystemPresent(double gen, double constDraw, double capacity) {
        return capacity > 0.0 || gen > 0.0 || constDraw > 0.0;
    }

    /* --power-log. */
    void power_log(double time);

    /* --steering-log: the attitude bill as numbers, not as a window. The flight
       windows print these same figures for the player; this is the channel for a
       test or a bug report, so a case does not have to read a UI row to know what
       the hull may spend (2026_11_05 e2e policy). R/P/Y about the attitude basis,
       and every column comes from the same answers the windows ask:
       attitudeAuthority() (live: power, switches, air), attitudeAuthorityHardware()
       (enable switches ignored -- what a switch gives up) and attitudeDelivery()
       (what the last actuated substep asked and actually spent). */
    void steering_log(double time);

    /* Staging. `activeStage_` is a monotonic COUNTER (the stage about to fire):
       it starts at the HIGHEST stage and steps down one per press. An engine
       lights once the counter reaches its stage (stage >= activeStage_) and stays
       lit; a decoupler triggers at its stage. */
    int activeStage_ = 1;
    int totalStages_ = 1;
    int minStage_ = 1;
    /* Which of the two press targets this stage burns against, decided the once per
       stage that stageIsDue() explains -- it is the live form of computeStaging's
       `drainAll`, which staging.cpp strikes at the START of a row's burn. Re-reading
       it per tick is what kept mars_explorer_mk1's dry boosters aboard (#928
       follow-up), because an asparagus drop set that has just emptied looks exactly
       like a drop set that never held fuel. Re-stamped when the counter moves or
       when THIS STAGE'S DROP SET changes size. Not the whole-hull part count: a
       crate ejected or a kerbal stepped aboard changes the hull, not the drop set,
       and re-stamping then re-derives the target from a drop set that has already
       run dry -- which is the #928 bug reintroduced mid-burn. */
    struct StageStamp {
        int stage = -1;
        size_t nDropParts = 0;
        bool drainAll = false;
    } stageStamp_;

    /* #948: the cache behind stagedRows(). Held as a BuildShip because that is what
       computeStaging reads; the parallel pointer arrays are how a redraw answers
       "does this snapshot still describe this ship?" without hashing (a ship is tens
       of parts, and one pass over pointers and labels is cheaper than an FNV over
       synthesized ids). stagedFuel_/stagedMass_/stagedG_/stagedExhaust_ are the
       figures the cached rows were computed on: while a burn runs the totals move
       every tick, and re-running the burn simulation per frame is what the VAB's own
       cache note bills at 473us. stagedSeed_ lives here so the per-frame refill costs
       no allocation. */
    BuildShip stagedSnap_;
    StagingSeed stagedSeed_;
    std::vector<Part *> stagedOrder_;        // parts in snapshot order
    std::vector<Part *> stagedParent_;       // per snapshot part, as seen then
    std::vector<std::pair<Part *, Part *> > stagedLinks_;
    std::vector<StageRow> stagedRows_;
    double stagedFuel_ = -1.0, stagedMass_ = -1.0;
    double stagedG_ = -1.0, stagedExhaust_ = -1.0;
    /* Walks performed, not asks answered (#966). One counter on the cache itself, so
       the pin reads the same `moved` branch that decides the cost. */
    uint64_t stagedRebuilds_ = 0;

    /* droppedPartsAtStage()'s bookkeeping arrays (#1131). The walk stopped being a
       press-time ask when #928 made stageIsDue() run it every logic tick an ascent law
       is armed: heaptrack over an autopilot ascent (mars_explorer_mk1 off the Kerbin
       pad, 30 parts, --ascent 150e3 --time-accel 10; the steady-state window is the
       delta of a 15 s and a 40 s run of that one command) counted 7,746 allocations
       from this one walk per 25 s of flight. They persist and only grow, so the same
       window after the change counts 1,505 -- one per call that actually drops
       something, which is the result vector: out.reserve() allocates nothing when
       nothing came off. Nothing outside the function reads these, and what they hold
       is ~33 bytes per part, retained after the last walk. */
    std::vector<std::pair<Part *, Part *>> dropKids_;
    std::vector<Part *> dropByAddr_, dropStack_;
    std::vector<char> dropFlags_;
    /* stageIsDue()'s "which groups can the lit engines drink" list, same reason:
       rebuilt per tick, and its sortedness is rebuilt too, so clear() is enough. */
    std::vector<int> dueReach_;

    int activeStage();
    /* Step to the previous stage (clamped at the lowest one). */
    void advanceStage();

    /* How many presses this hull still has: the counter steps one per press and
       stops at minStage_, so this is what an autopilot has left to ask for (#928). */
    int stagesLeft();

    /* Has this stage finished its burn, by the VAB's OWN plan? Same question
       staging.cpp's computeStaging asks when it draws the staging table, on live
       tanks -- see the definition beside canBurn(), which it walks like. The
       autopilot asks it before it presses (#928). */
    bool stageIsDue();

    /* Deploy every STOWED chute at `stage`. Game::stage calls it before the
       decoupler extractions, so chutes on dropped subtrees deploy too. One-shot. */
    int deployChutes(int stage);

    int numStages();

    /* THE THRUST NUMBERS, and why there is more than one (#400, #607, #849). There are
       FOUR, and three of them say "yes" for a ship sitting parked on the pad with its
       engines off, which is how the same bug has landed more than once:

       1. throttle() / thruster_util -- the LEVER, 0..1. A demand, not an output, and it
          DEFAULTS TO 1.0, so a freshly spawned ship reads full throttle with nothing
          burning.
       2. availableThrust() -- what the ignited stage COULD do at full throttle.
          Potential. Non-zero for any live engine with fuel, firing or not.
       3. producedThrust() -- rated thrust x the lever x the air state, summed over the
          ignited stage. It is NOT "what the engines are doing right now": it never asks
          whether an engine was armed this tick, so on a parked ship with the lever at its
          default it is the same number as at full burn. Its older comment claimed
          "RIGHT NOW" and #849 corrects it. Anything that COMMANDS the throttle and then
          reads this back reads its own zero -- the prototype in
          reports/autopilots2026_10_24 computed a_max from it while coasting, got 0.00
          every tick, took its own "coast" branch and never reignited (silent: a burn that
          never happens, not a crash).
       4. isThrusting() below -- did an engine ACTUALLY FIRE this tick. The one honest
          answer to "is this hull moving under power".

       availableAccel() is availableThrust() over the CURRENT mass: the a_max a throttle
       law, a gravity turn or a landing predictor needs. It does not move with the throttle,
       so a law can plan with it and act on the throttle without a feedback loop through its
       own output. It does include the current air state and the stage gate, so "available"
       means available to THIS stage HERE -- a jet in vacuum says zero, and a jettisoned
       stage is not available at all. Neither of them has a direction, which is what
       availableAccelAlong() below is for. */
    double producedThrust();
    double availableThrust();
    double availableAccel();
    /* "Did an engine fire this tick?" -- the per-tick "fired" flag, set only inside
       ApplyThrust, which only runs while the Thrust key is held or thrust is latched
       (tick.cpp), and cleared at the top of every tick with the stick state.
       GATE MOVEMENT INTENT ON THIS, never on the lever or the potential: a parking hold,
       a deck carry, an autopilot handover or ground handling that reads "thrust is
       available" releases on a parked ship (#849 -- the balloon-platform deck carry did
       exactly that, letting go of a hull that had never asked to move). The trap is
       silent because the other three numbers all look plausible.
       This is per-TICK, so it must be read after ApplyThrust and before the next
       clearThrust(), which is where the substep loop stands; the per-part armedThrust is
       the same fact per engine (main.cpp reads it for the engine audio). */
    bool isThrusting() const { return m_thrust != 0.0f; }
    /* How hard this ship can accelerate ALONG dir (m/s^2) at full throttle: the
       availableAccel() sum again, each engine weighted by how far its MOUNT and gimbal
       cone let it push along dir at all (#607).
       * dir is in frame S, the same axes partAxis()/GetPos()/attitudeBasis() speak. A
         law that thinks in a non-rotating frame must rotate its ask first, exactly as
         guidance.cpp does for its aim; pass the unrotated one and the answer is wrong
         by the body's rotation, silently.
       * A zero-length, NaN or infinite ask reads 0.0 -- fail CLOSED, because the NaN
         road through acos and std::max ends at "100% of your thrust".
       * The answer is SIGNED: an engine that can only push the wrong way counts
         negative, not 0. One lever (thruster_util) arms every ignited engine, so the
         push along dir really is the signed sum -- up-firing and down-firing engines
         cancel for the rest of that burn, and saying so is the honest number.
       * It moves with the hull's POSE (the mount axes are live) while ignoring the
         throttle and the gimbal deflection. That is not a "cannot thrust" verdict: a
         ship that simply has not turned yet reads 0 or negative along an ask it could
         reach in a second. A law that gates on it (can_thrust, dv/a) must gate on
         availableAccel() and use this for the direction cost, not the reverse.
       * Only thrusters count. An RCS-pods-only hull reads 0.0 in every direction even
         though rcs.h can translate it (#623), and no propellant is consulted, so a dry
         stage still reads healthy -- that is what canBurn() below is for. */
    double availableAccelAlong(const glm::dvec3 &dir);

    /* CAN THIS STAGE ACTUALLY BURN, right now, in the tanks it actually has? The gated
       twin of the three reads above, kept SEPARATE because they answer different
       questions and a read's name is the only thing telling a caller which (#607). The
       thrust family is POTENTIALS on purpose: tests/test_thrust.cpp section 4 builds its
       rig with no tank at all and says why in the comment ("these reads are POTENTIALS
       -- they never consult the fuel"), and the Vessel window prints them as ratings
       (gameui.cpp:2483-2485, "Thrust: produced/available" and "Max TWR") -- that block's
       own comment calls the pair what tells a builder the engine is fine and the throttle
       is not, which is a question about the hardware, not about this tick's tanks.
       A law dividing its wanted accel by availableAccel() wants the other question: a dry
       stage's potential is a number you must never divide by, and until now the only code
       that knew the gated answer was guidance.cpp's own mask pair, for itself.
       Per ENGINE, the way ApplyThrust sizes its draw: an ignited thruster counts only if
       EVERY resource it drains is available to it through ITS fuel groups
       (availableResourceMass). So a rocket with fuel and no oxidizer burns nothing, an
       engine walled off by a fuel barrier reads false with the stacks full, and fuel
       parked inside a cargo bay is invisible here exactly as it is to the draw (contents
       are not in `parts`, though getMass() counts them). The stage gate applies.
       Two edges that belong to the PAIR rather than to this read:
       * EXISTS, not ALL: one fed engine answers true, which is the right shape for a
         yes/no -- a dry engine should not veto a stage that can fly. The ceiling above
         is a SUM over every ignited engine, so on a stage with one fed and one unfed
         engine the law gates on this and sizes the burn with that; #639 asks whether the
         magnitude wants gating too.
       * Jets are engines here (they burn, they thrust, and availableAccel() already
         bills their ceiling), but this read never asks about the AIR -- a jet on Jool can
         "burn" and produce exactly nothing, which is the ceiling's zero for saying. It
         asks about the RESOURCE and the reach; whether one tick can be FUNDED is
         burnShare()'s question, asked where the draw happens. */
    bool canBurn();

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

    /* Variants for a caller that already walked the hull this frame (#384). Mass,
       produced thrust and available thrust are each a pass over every part and what
       is parked inside it, and the Vessel Info window reads all three a line apart --
       so it hands them down rather than asking three more times. NOT a store: the
       window takes its own fresh figures every frame. attitudeAuthorityRows() writes
       down why the same kind of readout walk stays uncached (#649's review pass); the
       mass behind these ratios moves inside a tick too, so a held copy would be last
       substep's hull wearing this one's label. */
    double getTWR(double thrustN, double mass);
    double getFullThrustTWR(double availThrustN, double mass);
    double getMaxTWR(double availThrustN, double mass);

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
    /* `star` (nullable) is the system's star, and it is ONLY what lets the air under this
       hull know which way the sun is (#724 rung D). A nullptr -- a fixture, a test, a
       system with no star -- means no coupling, and the profile stays as authored. */
    glm::dvec3 applyAeroForce(double h, const TerrainBody *star = nullptr);

    /* Charge this substep to the hull's loss totals (#675). Call it AFTER the force
       appliers, with the same `h` the integrator advanced, so it reads the three
       forces of one substep -- thrust, gravity, air -- rather than three substeps'
       opinions of itself. Cheap: two dots, a cross-free normalize and a divide. */
    void lossTick(double h);

    /* Air density (kg/m^3) at the COM. Deliberately NO kRhoFloor gate:
       jetAirGate's density half divides by the engine's rating intake
       (constants.h:kAirSeaLevelDensity), so a sub-floor read is vacuum either way. */
    double airDensityAtCom() const;

    /* The same read, handed over WHOLE: the body's air profile at the COM plus the
       altitude it was sampled at. A caller that needs the density AND anything about
       what the air IS (the jet gate's oxidizer share) takes this instead of
       airDensityAtCom() plus a hand-fetch off TerrainBody -- the two halves of that
       pair come from one profile, and splitting them is how a fourth call site could
       gate one body's air on another body's composition (#637). False = no physical
       air, in which case airDensity(atm, alt) would be 0 anyway. */
    bool airProfileAtCom(DragAtmosphere &atm, double &alt) const;

    /* ONE body-frame read at one part (`p` null = the hull's own COM): where it
       sits over the near body and what the air there is (#665, #667). The part
       window's live gauge and a science run's recorded measurement both come
       through here, so a barometer cannot show one pressure while the finding it
       just wrote says another.
       posRot is the near body's ROTATING frame -- the map/terrain frame, so
       equirect.h turns it into the lon/lat a map wants. press [Pa] and temp [K]
       are 0 wherever the profile says there is no air (a body without one, or
       above its top): that is what a gauge reads up there, not a shrug -- the
       same law airPressure/airTemperature already keep for --drag-log.
       Not const: the frame conversions are not (frame.h). False = no near body. */
    struct AirReading {
        glm::dvec3 posRot;      // m, in the near body's rotating frame
        double r = 0.0;         // m from the body centre
        double altAsl = 0.0;    // m above the body's sea-level datum
        double press = 0.0;     // Pa at altAsl
        double temp = 0.0;      // K at altAsl
    };
    bool airReadingAt(const Part *p, AirReading &out);

    /* Velocity relative to the AIR at `com`, i.e. GetVel() minus the AIR's own
       velocity: its co-rotation with the body (zero in the rotating frame, #60/#97)
       minus the authored wind at this point (atmo.h:windAt, #688). The ONE place the
       air moves -- wind is a velocity, never a force term, so subtracting here is what
       makes a headwind a headwind for the drag law, Mach, the jet gate and #498's
       air-relative prograde all at once. */
    glm::dvec3 airRelativeVel(const glm::dvec3 &com);

    /* The armed control forces, per SUBSTEP. The EVA kerbal overrides (eva.h). */
    virtual void applyControlForces(double h);

    /* The ground reaction on the gear wheels and landing legs, per SUBSTEP
       (src/wheel.h). A no-op for a hull with neither. Runs beside the aero pass;
       nothing integrates until physics_tick, so it reads the same fresh pose
       and velocity the aero does. */
    void applyGroundForces(double h);

    /* The contact find both ground loops share: ray from `from` along `down`
       to `reach`, analytic terrain fall-through outside a building bubble,
       AIRBORNE (false) on a missed ray inside one (#850). True writes the
       distance, hit point and unit normal. */
    bool groundContact(const glm::dvec3 &from, const glm::dvec3 &down,
                       double reach, double &dist, glm::dvec3 &hit,
                       glm::dvec3 &n);

    /* --leg-log: one row per gear wheel / leg footpad, printed from the tick
       (not the substep loop) by re-running the same contact ray. gap= is the
       distance from the part's DRAWN mesh bottom -- the lowest vertex of its
       convex hull, which today agrees with the def face it contacts (leg_foot
       0.05, wheel_radius 0.35) -- to the ground; that is the float the player
       sees (#1158). A no-op for a hull with neither. */
    void logLegContacts(double t);

    /* Brakes: the parking latch. Spawn sets it, O toggles it, and it travels in
       the save as the player's control state. (A ship parked on a balloon
       platform rests at v=0 in the building's bubble frame, #850 -- parking no
       longer needs a special latch to be observable.) */
    bool brakesOn = false;

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
    /* The ring's substep is two halves now (#595), and the order is the priority:
       rcsPlan() enumerates the bells and spends them against the player's TRANSLATION
       demand, the attitude walk then builds its rows from whatever capacity that left,
       and applyRcsForce() fires the sum -- one fuel draw and one force application for
       both demands, so an attitude ask can never buy back a translation the keys asked
       for, and never drains hydrazine twice. */
    void rcsPlan();
    /* Fill rcsNoz / rcsNozPart / rcsNozLocal from the live part geometry. Called by
       rcsPlan every substep, and by attitudeAuthorityRows when nothing fresher is
       standing (a window asking a railed hull, a test building an authority answer by
       hand). rcsNozFresh_ is invalidated wherever the part set or its shape changes
       (rebuildCompound), because a stale enumeration would silently price pointing with
       bells a decoupler just left behind. */
    void enumerateRcsNozzles();
    void applyRcsForce(double h);
    /* Reused solve + enumeration buffers: this path runs EVERY sim substep, so these
       persist (capacity only grows) and the hot path allocates nothing after warmup.
       Only rcsPlan / applyRcsForce / attitudeAuthorityRows touch them, and all three
       run inside one substep, so a caller must not hold a row index across substeps. */
    rcs::Scratch rcsScratch;
    std::vector<RcsNozzle> rcsNoz;        // this substep's nozzles
    std::vector<size_t> rcsNozPart;      // nozzle -> index into parts
    std::vector<size_t> rcsNozLocal;     // nozzle -> slot on that part (Part::rcsFired)
    std::vector<glm::dvec3> rcsFpart;    // per-part summed force, parts-sized
    /* The ATTITUDE ask per nozzle, as a fraction of its rating (#595): actuateAttitude
       fills it from the same actuatorSolve that fills surfaceLat_, applyRotationForce
       clears it with the other per-substep asks, and applyRcsForce adds it to the
       translation fraction rcsScratch.c already holds. Index-parallel to rcsNoz, which
       is what AttitudeAuthority::rcs rows are indexed against. */
    std::vector<double> rcsAttFrac;
    bool rcsNozFresh_ = false;
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
           and painting. Only meaningful for the surface kinds (lift, control).
           #1058 asked whether this second copy of the angle could go away and read
           the part it points at instead; it cannot, since the part index is exactly
           what staging invalidates. So the copy stays and the equivalence is what
           gets pinned: tests/test_authority.cpp checks that an arrow's pair equals
           Part::aeroAlphaDeg/aeroAngleValid for the substep that recorded it, which
           is the only thing keeping the overlay and the part window honest with each
           other. */
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
                         kForceInduced = 6, kForceSurfDrag = 7, kForceGround = 8,
                         kForceSide = 9, kForceBodyLift = 10;
    static constexpr int kForceKindCount = 11;
    // The drawer gates fv.kind < kForceKindCount; a kind past the count is
    // silently never drawn. The list and its count have drifted once (#947's
    // review caught it) -- this fires instead.
    static_assert(kForceBodyLift == kForceKindCount - 1, "kForceKindCount must cover every kForce* kind");
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

    /* --fuel-log: each group's fuel mass + the fuel links. Its resource words
       come from kResources (shipdef.h, #334). */
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
        /* The absorbed vessel's own ORDERS, frozen the tick it stopped being a hull
           (#657). A dock does not carry orders across into the merged hull -- that one
           is flown from the survivor's switches -- but nothing else can address them
           afterwards either, because the vessel that had them no longer exists. So they
           are neither applied nor thrown away: they wait here, and cutting this seam at
           its root end -- an undock, or a stage press that puts a decoupler exactly at
           that joint -- hands them back to the hull they belong to. Saved with the seam
           (save.h). A seam from a save written before #657 carries the constructor
           defaults, which is a FALLBACK rather than a restore: that build handed the
           leaving hull the survivor's orders, and this vessel's own figures were never
           recorded anywhere to recover. */
        bool wheels_enabled = true;
        bool surfaces_enabled = true;
        bool rcs_attitude_enabled = true;
        bool tvc_enabled = true;
        double sas_turn_rate = 0.0;   // #678's dial; 0 = unlimited
        /* #1067's dial travels with the same rule: the merged hull keeps the ROOT's
           RCS authority and the absorbed side's value waits here for the cut that
           gives that hull back its own. */
        double rcs_thrust_frac = 1.0;
        /* #1124's second dial, same rule and for the same reason: a hull that was
           told to steer with half its ring should not get a full one back from a
           dock, nor lose it to one. */
        double rcs_attitude_frac = 1.0;
    };
    /* Absorbed docks, innermost first -- the LAST is the one undock selects. */
    std::vector<DockSeam> seams;

    /* Docking INTENT (per ship): a dock needs BOTH halves -- target ship + port,
       and THIS ship's port that mates with it. */
    Vehicle *dockTargetShip = nullptr;
    Part *dockTargetPort = nullptr;
    Part *dockArmPort = nullptr;   // this ship's port to mate with (mandatory)

    /* Docking capture thresholds (#1152). They live here rather than in Game because
       Vehicle::dockRead is the pass that measures what they judge, and the two readers
       of that measurement -- the dock gate in Game::updateDocking and the Docking window
       (#1026) -- must quote the same numbers.

       WHAT THEY WERE: 1.5 m, cos 15 deg, 2 m/s, none of which anyone derived. A docking
       port is 0.25 m deep (nodes at +-0.125 m), so 1.5 m of face gap is six port-depths
       of daylight -- and since absorbShip() was a rigid rebase that keeps every part's
       world pose, that gap became part of the hull: measured 0.750 m for the fixture
       pair parked nose-to-nose, 1.495 m after an approach burn. (#1153 pulls the seam
       shut inside absorbShip now, so a capture inside these gates leaves the faces at the
       pair's own standoff. What these numbers still decide is how far the mated FACES
       slide and how much the pair TURNS; how far the donor's far end swings is that turn
       times its distance from the seam, which is the figure that reads as a jump.)
       The old numbers were also inconsistent with this game's own authority: an rcs_box is
       60 N, so on the 6560 kg fixture pair translation is 0.0091 m/s^2 and stopping from
       2 m/s needs 219 m of corridor. No player could have obeyed it.

       WHAT THEY ARE NOW, and the anchor for each:
         capture  a fraction of the SHALLOWER port's own depth (dockCaptureRange below).
                  Real capture geometry is decided by the thing being bolted together,
                  not by a round number, so a 4.5 m port is easier than a 2 m one.
         align    5 deg. A hard mate of two flat rings takes single-digit degrees; the
                  wider cones belong to funnelled probe systems, which these are not.
                  The lateral allowance falls out of the same cone: range * tan(5 deg) is
                  ~9 mm at the r1 capture distance, i.e. a real RMS MATE tolerance.
         speed    0.15 m/s. Crewed contact speeds are ~0.1 m/s and robotic berthing is
                  ~0.01-0.02; the fixture pair is 6560 kg, so this cap is ~1000 kg m/s
                  of capture momentum against a Progress's ~700, into a weld with no
                  damper. It is also reachable: 0.15 m/s needs 1.2 m and 16 s of one
                  rcs_box, which is the corridor the Docking window's dial shows. */
    static constexpr double kDockCaptureFrac = 0.4;  // of the shallower port's depth
    /* How close THESE two parts can sit before their collision hulls touch, m measured
       between the presented faces: each hull is inflated by effectiveHullMargin(), so the
       floor is the sum of the two -- 0 with the shipped margins (physics.cpp:58 defaults
       OSP_HULL_MARGIN to 0), metres under that knob. Both dockCaptureRange's floor and
       #1153's seam snap are stated in terms of it, so a mated seam closes to the tightest
       distance this game's own collider can hold rather than to a geometric ideal. */
    static double dockStandoff(const Part *pa, const Part *pb);
    /* The distance at which THESE two ports capture, m between the presented faces.
       A function and not a constant because it is a property of the pair; the gate,
       the dial's ring and the unit tests all call it, so nobody invents a boundary.
       The floor matters as much as the fraction: a collision hull is inflated by
       effectiveHullMargin(), so two parts cannot be pushed closer than twice that.
       max() keeps the gate OUTSIDE the standoff -- a capture distance the hulls can
       never reach is a gate that silently never opens. */
    static double dockCaptureRange(const Part *pa, const Part *pb);
    static constexpr double kDockAlign   = 0.996195; // cos(5 deg) axis misalignment
    static constexpr double kDockMaxV    = 0.15;     // m/s relative speed at capture

    /* Which face a port presents toward a point, and its outward normal there: the port's
       +Z or -Z, whichever faces `toward`, at +-def->height/2 off its centre. Three passes
       need that choice and have to make it the same way -- dockRead() for both halves of a
       pair, absorbShip()'s snap (#1153) for the seam it closes, and the approach ladder
       (#1146) for the mouth it runs out of -- so a rung on screen cannot disagree with the
       gap the gate measured. All three vectors are in ONE frame, whichever the caller is
       already working in; nothing here lifts between frames. */
    static void dockFace(const Part *p, const glm::dvec3 &pP, const glm::dmat3 &pR,
                         const glm::dvec3 &toward, glm::dvec3 &axis, glm::dvec3 &face);

    /* The approach ladder (#1146): squares normal out of the port a hull has targeted, so
       the line to fly and the port's own roll are visible in the world and not only as
       numbers in the Docking window. Their size is a tuning number for the eye, not
       physics -- nothing in the docking law reads them -- but they are named here so the
       drawn ladder and its log line cannot disagree about what a rung is. */
    static constexpr double kDockRungEdge = 2.0;   // m across each square
    static constexpr double kDockRungStep = 2.0;   // m between rung centres
    static constexpr int kDockRungCount = 10;      // -> 20 m of ladder
    struct DockLadder {
        bool valid = false;
        /* Everything in the PORT's own ship's frame; the painter lifts once. Rung n sits
           at face + axis * (kDockRungStep * n) for n = 1..kDockRungCount -- the first rung
           is a step out, not on the face itself, so it never lies in the port's own skin.
           ru/rv are the port's own in-plane axes: the squares ride the port's roll rather
           than the screen's, which is what "oriented with the port" means. */
        glm::dvec3 face;
        glm::dvec3 axis;   // out of that face: the rung plane's normal
        glm::dvec3 ru, rv;
    };
    /* The ladder for one of THIS ship's ports, measured toward `approachFrom` -- the point
       the mating hull is coming from, in the same frame. Not a DockRead: the ladder is one
       port's geometry, drawn whether or not a second port is anywhere near it. */
    DockLadder dockLadder(const Part *port, const glm::dvec3 &approachFrom) const;

    /* What these two ports look like to each other (#1026). ONE pass, shared by the
       dock gate and the Docking window, so a row a pilot reads cannot disagree with the
       rule that actually mates the hulls: the figures AND the three gate verdicts come
       out of here, and the thresholds they are judged against are the constants just
       above, which the window prints beside the figures. */
    struct DockRead {
        /* False when the two port centres coincide: there is no line to measure along,
           so nothing below means anything yet. */
        bool measurable = false;
        /* A port presents whichever of its two faces is nearer the other part (+Z or
           -Z of its own frame). These are the cosines between each presented face's
           axis and the line joining the port centres -- what kDockAlign is tested
           against, and why they run near 1 rather than near 0. */
        double axisCosA = 1.0, axisCosB = 1.0;
        double faceGap = 0.0;   // m between the two presented faces
        /* The pair's own capture distance, m: dockCaptureRange(pa, pb), carried here so
           the Docking window draws the SAME ring the gate compares against rather than
           re-deriving a threshold the ports do not agree with. */
        double captureRange = 0.0;
        double relSpeed = 0.0;  // m/s at the two ports (kDockMaxV)
        /* The corrections the Docking window plots, signed so that POSITIVE MEANS THE
           NAMED KEY: turn D, turn W, push L, push I. They are resolved on the axes the
           KEYS move -- attitudeRef()'s frame, which is where applyRotationForce reads the
           stick -- so a port mounted sideways rotates the dial rather than lying about the
           keys. (#1026. A parallel-error readout was tried first and dropped: it compares
           axis to axis, but the gate compares each axis to the line joining the ports, and
           with a lateral offset those disagree by atan(offset/range) per side -- up to 30
           deg at the capture distance, which is the whole cone.)
           `turn*` is the rotation that brings my presented face onto the joining line,
           which is what shutAxis tests for MY face, so the dial draws it as a circle at
           acos(kDockAlign) on its degree scale. `push*` compared against
           range*sin(kDockAlign) is the same gate for THEIR face, drawn as the cone circle
           on the metre scale: two circles, both halves of shutAxis. Both pairs are
           PROJECTIONS onto the two keys that act on them -- the third component, along the
           nose, has no key -- so a marker's bearing comes from these figures while its
           radius comes from the angle and distance the gate compares.
           `push*` is how far my port centre sits off the OTHER port's bore line -- the
           approach path. It has to be their line: the joining line runs through both ports,
           so by construction it has no offset to report.
           `drift*` is which way that offset is moving, m/s. A pair that reads centred and
           is sliding 0.5 m/s sideways was never centred.
           All three are ill-conditioned when the faces are far from facing each other (a
           two-component projection has no unique meaning there), which the dial handles by
           pinning to its rim rather than pretending. */
        double turnYaw = 0.0, turnPitch = 0.0;      // rad
        double pushRight = 0.0, pushUp = 0.0;       // m
        double driftRight = 0.0, driftUp = 0.0;     // m/s
        double portRange = 0.0;                     // m between the port centres
        /* The three gates as THIS pass judged them, so nobody downstream re-compares the
           figures against a threshold of their own. Each boundary is a different operator
           -- an axis fails BELOW the cosine, a gap AT or ABOVE it, a speed strictly ABOVE
           -- and a window that guessed would disagree with the gate right at the edge,
           which is the only kind of drift this struct exists to prevent. dockSituation()
           and the Docking window's v_rel colour both read these. */
        bool shutAxis = false, shutGap = false, shutSpeed = false;
        /* In the ARMED port's frame, +Z out of its presented face. NOT gates: they say
           which way to nudge once a gate is shut, which is the part a yes/no verdict
           cannot tell a pilot. The sideways half of that nudge is push* above; `along`
           and `closing` have no drawing yet, which is a hole -- see #1026. */
        double along = 0.0;     // m: the other face along this port's axis
        double closing = 0.0;   // m/s: + = the two ports coming together
    };
    DockRead dockRead(const Part *pa, const Vehicle *b, const Part *pb) const;

    /* The whole docking situation of THIS ship's intent (#1026): which half of the
       intent is missing or stale, whether the two hulls are even in a state to mate,
       and what the three gates measure. Game::updateDocking() acts on `why` and the
       Docking window says it out loud, so a refusal that used to be silent has one
       sentence and one token. The order below IS the gate's order -- it matters, because
       a stale intent half is DROPPED where a rails state is merely waited out, and a
       window that asked in a different order would promise a mate the gate never
       considered. */
    struct DockSit {
        enum Why : unsigned char {
            MyRails,       // this hull is on the rails (warp >= kRailsWarp)
            NoArm,         // nothing armed on this hull
            NoTarget,      // no target ship/port set
            TargetGone,    // that hull no longer carries the port that was targeted
            TargetRails,   // the target is on the rails; proximity has to wake it
            OtherFrame,    // the two hulls are in different frames
            ArmGone,       // the armed port left with a stage press
            Unmeasurable,  // the port centres are at one point: no axis to compare
            Misaligned,    // a face is outside the kDockAlign cone
            TooWide,       // the faces are further than the pair's captureRange
            TooFast,       // the ports move relative to each other past kDockMaxV
            Mass,          // refuseDockWith: the pair would not fit in one Body (#583)
            Go,            // every gate open: the next tick mates them
        };
        Why why = NoArm;
        Vehicle *b = nullptr;    // the targeted hull, null when there is none
        Part *arm = nullptr;     // this ship's armed port, null when absent/stale
        Part *target = nullptr;  // the targeted port, ditto
        DockRead read;
        std::string massWhy;     // the #583 sentence, "" when the pair fits
    };
    DockSit dockSituation();
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

    /* How far a dock had to pull the donor onto the survivor, out of #1153's snap. The
       play path prints it on [dock] so a case can pin the snap happened rather than infer
       it from a post-dock screenshot. Both figures describe the MATED PORT: the donor is
       rigid, so a part d metres from the seam swings about d * sin(turn) further than
       `shift` -- that far-end figure, not this one, is what the eye catches. */
    struct DockSnap {
        double shift = 0.0;   // m the donor's mated port travelled
        double turn = 0.0;    // deg the donor turned (degrees; DockRead speaks radians)
    };

    /* Fold B into this ship at the mated port. B's shell is deleted by the
       caller, B's journal is dropped (#49). B's crew ride along because a seat
       belongs to its capsule Part, which moves with the part tree.
       `portB` is B's half of the mated pair, the port the gate measured (see
       absorbShip's snap block, #1153): given it, the seam is pulled shut -- the two
       presented faces coincident, the bores anti-parallel -- before the weld, and
       `snap` comes back with how far that moved the donor. Pass neither and the merge
       is what it always was: a weld at whatever pose the capture tick found. */
    void absorbShip(Vehicle *B, Part *portA, Part *portB = nullptr,
                    DockSnap *snap = nullptr);

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
       nobody able to tell which one won.
       IT STARTS AT 1.0, not 0: a freshly spawned hull reads full throttle while sitting
       on the pad with nothing burning, so this is a demand and never an answer to "is
       the ship moving under power" -- that one is isThrusting(), and #849 is the bug that
       came from confusing the two. Whether the default should be 0 is its own decision
       (it moves the spawn behaviour of every hull and the save format's meaning), so it
       is not taken here. */
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

    /* The part's full pose (position + orientation) in `relTo`'s frame --
       GetPositionRelTo's sibling for when the axes matter too (the EVA ladder
       law reads a part-local segment through it, #1073). One spelling, same
       frame transform; non-const for the same reason as GetPositionRelTo. */
    void partPoseRelTo(const Part *part, Frame *relTo,
                       glm::dvec3 &pos, glm::dmat3 &rot);

    /* The hull's centre of mass in UNIVERSE-ROOT (star) axes (#687). Every other
       frame math here is local to a body; insolation is the one reading that
       needs the distance to the star, and asking for it through a chain of
       local frames is how it would end up spelled three ways. New callers should
       prefer THIS over hand-composing GetPositionRelTo(star): one spelling. It is
       non-const because get_center_of_mass() is -- a hull's mass distribution is
       mutable (staging, docking, a part burning away), so "where is the middle"
       is a question about a thing that can change. */
    glm::dvec3 GetRootPos();

    /* Re-express the ship's state in newFrame and re-home it (setSoi). */
    void moveToFrame(Frame *newFrame, double t);

    /* The ONE SoI writer: every live (frame, m_parent, journal) change goes
       through here. Which list a Vehicle is in is the owner's business
       (Game::vehicles), not touched here -- so an SoI crossing cannot
       invalidate a walk of that list. */
    void setSoi(Frame *newFrame, double t);

    /* Per-tick SOI bookkeeping for THIS ship. */
    void switchFrames(double t);

    /* Write the rail state into the body (once per tick): pose from the conic and
       rail_orient, velocity from rail_vel, spin from rail_omega (#1042). */
    void writeRailPose();

    /* The COM state in `inFrame` -- any frame node. Most callers pass the
       non-rotating node where the trajectory is a Kepler conic; render.cpp also
       passes the body's ROTATING frame for the surface readouts, so this carries
       the whole switch law (old frame's stasis added, new frame's subtracted),
       not just the conic half. */
    void comStateIn(Frame *inFrame, glm::dvec3 &p, glm::dvec3 &v);

    /* A point of this ship (frame coords) in the body's ROTATING frame -- the
       axes terrain, wind and biomes live in. Identity when already there.
       Works from ANY frame, including a building bubble (#850): altitude,
       ground and air reads must go through this, not |com|, because a
       bubble's origin is not the body centre. */
    glm::dvec3 posInBodyRot(const glm::dvec3 &p) const;

    // COM separation in the universe (root) frame: frame-invariant.
    double distanceTo(Vehicle *o);
    /* The same walk, and how fast the gap is changing: rate > 0 means the two
       hulls are separating. Both numbers come from one pair of comStateIn calls
       in the shared root frame, so a caller that wants both does not pay the
       frame walk twice (#937: the proximity pass needs the rate to tell an
       approach from a departure). */
    void gapTo(Vehicle *o, double &dist, double &rate);

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
       and climbing. railsTick stops a conic at the air/ground instead.
       Virtual because a suit that is hanging on someone else's hull has no park
       of its own: see Kerbal::canRail (#1162). */
    virtual bool canRail();

    /* Park out of the world and coast analytically. False if not rail-eligible. */
    bool goOnRails();

    /* Re-enter physics from rails: rebuild the Bullet state. */
    void leaveRails();

    /* Per-tick rail advance: propagate the conic, test the END of the advance for
       an SoI boundary, hand off there, refresh the parked transforms. The test
       only sees the endpoint, so a long step can fly past a whole sphere.
       A coast that reaches the air or the ground leaves the rails here, so the
       caller owns the warp drop. */
    RailEnd railAdvance(double t, const double step);

    /* railAdvance plus the hop's EC bill (#647): a railed hull is off the substep
       loop that bills everything else, so this is the only door through which a warp
       spends electricity. Only a hull still on rails when the hop ends is billed --
       the one that handed off to physics is billed by the loop that took it. */
    RailEnd railsTick(double t, const double step);

    /* Re-anchor the rail state on another frame and re-home it (setSoi). Asserts
       that the rail state and BOTH frames share one instant, and that the target
       contains the ship then. */
    void moveToRailFrame(Frame *newFrame, double t);

private:
    /* One loop behind the two ceiling reads: the ignited stages' full-throttle push
       in newtons, exhaust_scale applied. `want == nullptr` counts every engine at full
       weight (what availableThrust() wants); a unit `want` counts each engine only as
       far as its mount and gimbal cone reach toward it (availableAccelAlong). Shared so
       the ceiling a plan sizes a burn with and the ceiling it steers against cannot
       drift apart (#607). */
    double ceilingSum(const glm::dvec3 *want);

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
   `base`, plus the ship's; 0 if the caller re-places. `supportUp` is the local
   vertical the hull rests against -- the hull's lowest point ALONG THAT
   direction lands on base + margin, so a tilted or horizontal hull rests on
   the right corner (#846), not on its own axis's lowest. */
void build_ship(Vehicle *ship, const ShipDef &def, Shader *partsshader,
                const glm::dvec3 &base, const glm::dmat3 &orient,
                double supportMargin, const glm::dvec3 &supportUp);

/* Starting scenario (a CLI choice). Pad beds are set up in main; orbit beds are
   circular about the home body; ellipse-* and escape-* set their own apsides
   and speed; abs_r pins an ABSOLUTE radius.
   The three fields WITHOUT a default member initializer -- name, dev_only,
   ell_phase -- are the ones a kScenarios row must state (#715). Omitting one is
   now the only -Wmissing-field-initializers this table can emit, where it used to
   print 266 lines and bury it (#991). Two caveats before trusting that arm: g++
   repeats each omission TWICE and blames the table's closing brace, not the row,
   and `make test` compiles src without -Wall at all. The runtime nets are the
   durable ones -- check_vab_gate() and check_ellipse_beds() ask about rows off the
   table, and spawn_vehicle() asserts what a row must mean. Everything else
   defaults to what omitting it already meant in the table's static storage, so no
   bed's numbers changed. */
struct ScenarioDef {
    const char *name;
    /* Dev-only beds stay out of the VAB dropdown unless --dev-scenarios. The
       name resolves either way: --startship, saves and the
       e2e fixtures all name dev beds, so this gates the picker, not the
       scenario. WRITE IT ON EVERY ROW (#715): it has NO default member
       initializer on purpose, so a row that omits it -- handing a dev bed to the
       player -- is the warning this struct still emits. check_vab_gate() is the
       net -- it fails if any bed other than "pad" is not dev-only. */
    bool dev_only;
    bool on_pad = false;
    double alt_frac = 0.0; // circular: fraction of the near-body shell (rot SOI -
                           // TerrainBody::seaLevelRadius()), or of the atmosphere
                           // top if atmo_frac
    bool polar = false;
    int ell_phase;   // -1: circular; 0: periapsis; 1: apoapsis; 2: 90 deg.
                     // Deliberately no default: 0 is periapsis, not circular, so
                     // an omitted one is a wrong bed rather than a plain one.
    double peri_alt = 0.0; // ellipse: periapsis ABOVE MEAN SEA LEVEL (m), i.e. a
                           // radius minus TerrainBody::seaLevelRadius() (#404)
    double apo_alt = 0.0;  // ellipse: apoapsis above mean sea level (m)
    double esc_frac = 0.0; // escape: launch speed in local escape velocities (0 = not escape)
    double abs_r = 0.0;    // > 0: absolute circular-orbit radius from the body
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
    /* Circular beds only, INSTEAD of speed_frac: the bed's AIR-RELATIVE speed set by
       the AIRFRAME's lift rather than by the body's gravity (#693). The authored number
       is a LOAD -- q / g [kg/m^2, equivalently Pa per m/s^2] -- and the bed's dynamic
       pressure is load x airGravity(body, alt), so the speed is sqrt(2 q / rho(alt)).
       Why a LOAD and not a q: a wing's trim condition is q = W/(S CL) = m g/(S CL), so q
       carries the PLANET's g and only q/g is the airframe's own number. One authored q
       would be right near Kerbin and off by g_body/9.81 everywhere -- 2x under trim lift
       on Eve, 1.8x over on Duna (both measured). With the g put back at the bed, the same
       load lands the hull near its trim on each of them.
       That is what a gravity fraction cannot do: it survives neither an air-profile change
       (#686 moved Kerbin's rho 1.44x at bed height and left flying-cruise 1.9x over its
       airframe's trim) nor a bed-height change (#682 moved the flying beds 7 km -> 8.5 km).
       Height invariance is real in this model and was measured (same hull, q 488.7 Pa at
       8069 m vs 486.1 Pa at 7661 m); it is only true SUBSONIC -- liftSlopeMach and waveCd
       both bite hard past M ~1 (Duna's bed reads M 0.8), so this is a low-speed keying.
       It is still a bed, not a flight computer: the load belongs to ONE reference hull
       (the bed ignores which ship it is given, as speed_frac's law says), and even two
       shipped hulls of the same family differ 2.1x in it. A hull-derived load needs a
       trim AoA no code has yet (#569). Needs air at the bed radius; a body without it
       refuses rather than dividing by zero. */
    double speed_load = 0.0;
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

/* Empty when this bed can be used on this body, or the reason it cannot (#693).
   Two front doors ask it: the --startship entry parse and the VAB launch. Both must ask
   BEFORE anything is built or placed, because the alternative is a throw from
   spawn_vehicle arriving in an imgui handler -- a SIGABRT and a lost game (#291) -- or
   escaping the CLI as an abort rather than an `error:` line.
   `alt_m` is the altitude an air bed was asked for (#817), kNoCoord when the bed takes
   the height its scenario authors: a load-keyed bed needs air AT the height it flies at,
   so the asked number belongs in the question rather than in one door only. */
std::string bed_refusal(const ScenarioDef &sc, const TerrainBody &home,
                        double alt_m = kNoCoord);

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
   binormal so each orbit keeps its shape. Pads ignore it.
   `place` (nullable): a map point the bed must pass over; null = the bed's own
   geometry. Only the start-ship path fills one -- see SpawnPlace. */
void spawn_vehicle(Vehicle *ship, const ScenarioDef &sc, TerrainBody *home,
                   System &sys, double slot_offset, double t,
                   const SpawnPlace *place = nullptr);

/* Turn a circular bed so it passes over a surface direction, changing its plane as
   little as it can. All three arguments are WORLD axes unit vectors: `rhat` the bed's
   position direction, `vhat` its velocity direction (so the plane normal is
   rhat x vhat), `want` the surface direction to fly over.
   Two moves, each the smallest available: the plane tilts by exactly the angle
   between itself and `want` -- reported in `tilt_deg`, 0 when the bed already
   reaches that latitude -- then the bed turns within the tilted plane until its
   position direction IS `want`. The return value is where the velocity direction
   goes under the same two turns, so the bed is still a circular bed of the same
   radius and speed, in the tilted plane, with a prograde heading in it.
   A point over the bed's own pole has no node line to tilt about: there the plane
   turns 90 deg about `rhat`, which keeps the meridian the bed already sat in.
   Degenerate input (rhat parallel vhat, or a non-unit vector) is a caller bug --
   asserted, not returned. Pure, so tests/test_scenarios.cpp can pin it by hand. */
glm::dvec3 bedTurnTo(double &tilt_deg, const glm::dvec3 &rhat,
                     const glm::dvec3 &vhat, const glm::dvec3 &want);

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
