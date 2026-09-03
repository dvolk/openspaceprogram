// vehicle.h -- the ship: Vehicle + its command types.
//
//   ShipCmdType / ShipCmd  one control command (throttle, thrust, pitch, ...).
//   SlewMode               autopilot slew target (prograde/retro/kill-rot).
//   Vehicle                a built ship: its parts, controls, staging, and
//                          the rails coasting state machine.
//
// Every method is defined inline: main.cpp is the only user, so keeping
// the definitions here makes the move out of main.cpp a pure extraction.

#pragma once

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// length2 (used by the inline methods) is a gtx function.
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
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
    std::vector<void *> constraints;
    /* which parts each constraint joins (parent, child) BY POINTER, parallel
       to `constraints`. A Part* is stable for the ship's lifetime, so staging
       just drops the constraints that touch a removed part -- no index
       remapping (the old pair<size_t,size_t> links + computeStageSplit are
       gone). */
    std::vector<std::pair<Part *, Part *>> constraintLinks;
    /* the weld's LOCAL anchor points (parentAnchor, childAnchor), parallel
       to `constraints`. The rails handoff detaches every weld and re-glues
       from these when the ship re-enters physics. */
    std::vector<std::pair<glm::dvec3, glm::dvec3>> constraintAnchors;

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
    /* the per-part parked pose (relative to the cluster COM, in cluster axes)
       lives on each Part now (Part::railRelPos / Part::railRelRot). */

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
    /* The autopilot mode the Autopilot window has engaged (its toggle
       buttons). Persistent across ticks, unlike `slew` (cleared each tick):
       the logic tick re-applies it after clearRotCmd(), so the ship keeps
       slewing toward the target and holding until the mode is toggled off. */
    SlewMode slewRequest = SlewNone;
    void setSlewRequest(SlewMode m) { slewRequest = m; }

    void setRoot(Part *part) {
        parts.push_back(part);
    }

    /* Weld `part` to the part at `parentIdx` at the given LOCAL anchor
       points (which must coincide in world space -- the 6DOF weld enforces
       zero relative linear offset). Records the link (by pointer) for
       staging. The caller has already setPosRot-ed the child to the matching
       pose. */
    void attach(Part *part, size_t parentIdx,
                const glm::dvec3 &parentAnchor, const glm::dvec3 &childAnchor) {
        Part *parent = parts[parentIdx];
        void *constraint = GlueTogether(parent->body, part->body,
                                        parentAnchor, childAnchor);
        parts.push_back(part);
        constraints.push_back(constraint);
        constraintLinks.push_back(std::make_pair(parent, part));
        constraintAnchors.push_back(std::make_pair(parentAnchor, childAnchor));
    }

    void attachDown(Part *part) {
        /* weld at the part faces: parent bottom (-h/2) to child top (+h/2);
           generalizes the old hardcoded +-1 m (2 m parts). The parent is the
           last part pushed. */
        const PartDef *parent = parts.back()->def;
        attach(part, parts.size() - 1,
               glm::dvec3(0.0, 0.0, -parent->height / 2.0),
               glm::dvec3(0.0, 0.0,  part->def->height / 2.0));
    }

    void attachRadial(Part *part) {
        /* weld the part to the parent's SIDE: the part's local +Z axis is
           rotated to the parent's local +X (call site), so the part's
           bottom face (local -h/2) touches the parent's side at +radius.
           The anchors coincide in world space at the tangent point
           (parent local (r,0,0) == part local (0,0,-h/2)). */
        const PartDef *parent = parts.back()->def;
        attach(part, parts.size() - 1,
               glm::dvec3(parent->radius, 0.0, 0.0),
               glm::dvec3(0.0, 0.0, -part->def->height / 2.0));
    }

    void attachSide(Part *part) {
        /* weld the part to the parent's SIDE with PARALLEL axes: the part
           keeps the parent's local +Z axis (no rotation at the call site),
           sits along the parent's local +X, and its cylindrical surface
           touches the parent's at +radius. The anchors coincide in world
           space at the tangent point (parent local (r,0,0) == part local
           (-r,0,0)). Unlike attachRadial the child is NOT rotated, so this
           is the "side by side, parallel axes" case. */
        const PartDef *parent = parts.back()->def;
        attach(part, parts.size() - 1,
               glm::dvec3(parent->radius, 0.0, 0.0),
               glm::dvec3(-part->def->radius, 0.0, 0.0));
    }

    void init() {
        if(parts.empty()) { return; }
        if(controller == nullptr) { controller = parts[0]; }
        NeverSleep(controller->body);
        /* propellant reservoirs: seed each tank part's resources so the
           thrusters can draw from them (they shed mass as they burn). Only
           done at construction -- separateStage() must NOT re-seed. */
        for(size_t i = 0; i < parts.size(); i++) {
            Part *p = parts[i];
            if(!p->isTank()) { continue; }
            for(int r = 0; r < (int)ResourceType::Num; r++) {
                p->resources.capacity[r] = p->def->capacity[r];
                p->resources.current[r]  = p->def->capacity[r];
            }
        }
    }

    /* True of the EVA kerbal (src/eva.h): control input, the camera and
       the event dispatch branch on this. Everything else is inherited --
       a kerbal is a one-part ship as far as frames, rails, gravity, the
       fleet and the HUD are concerned. */
    virtual bool isEva() const { return false; }

    /* True while an EVA character is ABOARD a ship (parked inside a capsule,
       out of the physics world) -- the render pass skips it (it is inside
       the capsule, not a visible body). Overridden in src/eva.h; a regular
       ship never carries this, so it is false by default. */
    virtual bool isCrewAboard() const { return false; }

    /* Draw `amt` kg of `type` from the ACTIVE stage's tanks, pro-rata
       across every tank that still holds it (stages are self-contained: a
       stage burns its own propellant, so the booster's engines never drain
       the upper stage's tanks). Pro-rata, NOT first-tank-first: draining
       one tank to empty before its siblings shifts the ship's mass
       distribution and torques it under thrust (the radial-tank spin);
       shares proportional to each tank's contents keep a symmetric cluster
       draining together, and empty the tanks simultaneously. Returns true
       if the stage's total covers amt (else the thruster doesn't fire
       this tick). amt is the kg consumed THIS tick (the caller scales the
       kg/s flow by the tick's simulated time). */
    bool consumeResourceMass(enum ResourceType type, float amt /* kg */) {
        const int as = activeStage();
        float total = 0;
        for(Part *p : parts) {
            if(p->stage != as) { continue; }
            total += p->resources.current[(int)type];
        }
        if(total < amt) { return false; }
        for(Part *p : parts) {
            if(p->stage != as) { continue; }
            const float have = p->resources.current[(int)type];
            if(have <= 0.0f) { continue; }
            float take = amt * have / total; /* pro-rata share */
            if(take > have) { take = have; } /* float rounding */
            p->resources.current[(int)type] = have - take;
            p->body->mass -= (double)take;
            SetMass(p->body, p->body->mass);
        }
        return true;
    }

    float getFuelMass(const std::vector /* eh */ <enum ResourceType>& types) {
        float fuel = 0;
        for(auto&& type : types) {
            for(Part *p : parts) {
                fuel += p->resources.current[(int)type];
            }
        }
        return fuel;
    }

    float getDeltaV() {
        float remaining_fuel = getFuelMass({ ResourceType::Hydrogen, ResourceType::LOX }); /* kg */
        double ve = 0;   // first thruster's exhaust velocity (the delta-v estimate)
        for(Part *p : parts) { if(p->isThruster()) { ve = p->exhaustVelocity(); break; } }
        return (float)(ve * exhaust_scale)
             * log(getMass() / (getMass() - remaining_fuel));
    }

    /* TODO should be cached per frame */
    float getMass() {
        float r = 0;
        for(Part *p : parts) {
            r += p->body->mass;
        }
        return r;
    }

    /* The active (currently live) stage: the lowest stage number still on
       the ship. Stages are dropped low-to-high, so this is what fires and
       what separateStage() detaches next. */
    int activeStage() {
        if(parts.empty()) { return 1; }
        int s = parts[0]->stage;
        for(size_t i = 1; i < parts.size(); i++) {
            if(parts[i]->stage < s) { s = parts[i]->stage; }
        }
        return s;
    }

    /* Total number of stages on the ship (the highest stage number --
       stages are labelled 1..N from the booster up, so the max label IS the
       total). Deliberately the max, not the count of *remaining* stages, so
       the "stage X of N" readout stays stable after the active stage is
       dropped (a remaining-count would read e.g. "stage 2 of 1"). */
    int numStages() {
        int n = 0;
        for(Part *p : parts) {
            if(p->stage > n) { n = p->stage; }
        }
        return n;
    }

    float getThrust() {
        return GetActiveThrust() * thruster_util;
    }

    // current Thrust-to-weight ratio
    float getTWR() {
        return (thruster_util * GetActiveThrust()) / (getMass() * m_parent->g);
    }

    // full throttle TWR
    float getFullThrustTWR() {
        return GetActiveThrust() / (getMass() * m_parent->g);
    }

    // empty TWR
    float getMaxTWR() {
        float remaining_fuel = getFuelMass({ ResourceType::Hydrogen, ResourceType::LOX }); /* kg */
        return GetActiveThrust() / ((getMass() - remaining_fuel) * m_parent->g);
    }

    void setVelocity(glm::dvec3 vel) {
        for(Part *p : parts) {
            SetVelocity(p->body, vel);
        }
    }

    void setPosition(glm::dvec3 pos) {
        void SetPosition(Body *b, glm::dvec3 com, glm::dvec3 pos);
        for(Part *p : parts) {
            SetPosition(p->body, get_center_of_mass(), pos);
        }
    }

    const glm::dvec3& get_center_of_mass(void) {
        double total_mass = 0;
        for(Part *p : parts) {
            total_mass += p->body->mass;
        }
        m_com = glm::dvec3(0, 0, 0);
        for(Part *p : parts) {
            m_com += GetPosition(p->body) * (p->body->mass / total_mass);
        }
        return m_com;
    }

    glm::dvec3 applyGravity() {
        const double& parent_mass = m_parent->mass;
        const double G = 6.674e-11;
        glm::dvec3 gf;
        for(Part *p : parts) {
            if(p->body->mass == 0) { continue; }
            const glm::dvec3& b1b2 = GetPosition(p->body);
            const double m1m2 = p->body->mass * parent_mass;
            const double invrsqr = 1.0 / glm::length2(b1b2);
            const double mag = G * m1m2 * invrsqr;
            gf = mag * sqrt(invrsqr) * -b1b2;
            ApplyCentralForce(p->body, gf, mag);
            if(frame->isRotFrame()) {
                // In rotating coordinates the ship additionally feels
                // Coriolis + centrifugal; without these its true inertial
                // orbit is perturbed for as long as it spends in the rotating
                // frame (see GetFictitiousAccel in frame.h).
                const glm::dvec3 a_fict = frame->GetFictitiousAccel(b1b2, GetVelocity(p->body));
                ApplyCentralForce(p->body, p->body->mass * a_fict);
            }
        }
        return gf;
    }

public:
    Vehicle() { }
    /* Tear the ship down in a safe order: welds first (they reference the
       bodies), then unregister each body from the world, then delete the
       bodies (which free their model + rigid body). goOnRails() already did
       the first two steps and cleared `constraints`, so for a railed ship
       only the deletes remain. The onRails guard is LOAD-BEARING: Bullet's
       removeCollisionObject is not idempotent -- it reads the object's
       world-array index (which remove never resets to -1), so a second
       remove on an absent body can evict the WRONG collision object. The
       old body of this dtor (bare `delete part`) also deleted each rigid
       body while it was still registered in the world -- UB that only got
       away with it because the process exits right after. */
    virtual ~Vehicle() {
        // The crew aboard (Kerbals, eva.h) are owned by this ship: delete
        // them before their parts would outlive their `aboard` target.
        for(auto&& k : crew) { delete k; }
        crew.clear();
        if(!onRails) {
            for(auto&& c : constraints) { Detach(c); }
            constraints.clear();
            for(Part *p : parts) { RemoveBody(p->body); }
        }
        for(Part *p : parts) { delete p; }   // ~Part deletes the Body
    }

    glm::dvec3 processGravity() {
        return applyGravity();
    }

    // Bullet clears all accumulated forces on every stepSimulation, so the
    // thrust -- like gravity -- must be re-applied before EVERY substep.
    // Applied once per tick it would only act during the first substep's
    // h seconds of the tick's n*h, cutting the delivered thrust to 1/n
    // (and n grows with time acceleration, so it got worse at warp).
    void applyThrustForce() {
        for(Part *p : parts) {
            if(!p->isThruster()) { continue; }
            if(p->armedThrust == 0.0f) { continue; }
            ApplyCentralForceForward(p->body, (double)p->armedThrust);
        }
    }

    /* The armed control forces, re-applied before EVERY substep (Bullet
       clears forces per stepSimulation). Ships deliver thrust + rotation;
       the EVA kerbal overrides with its walking/RCS laws (src/eva.h). */
    virtual void applyControlForces(double h) {
        applyThrustForce();
        applyRotationForce(h);
    }

    /* the first reaction-wheel body (nullptr if the ship has none): the
       stick / slew / kill-rot laws all use it as the ship's attitude
       reference. Replaces the old m_reaction_wheels.front(). */
    Body *firstWheel() {
        for(Part *p : parts) { if(p->isWheel()) { return p->body; } }
        return nullptr;
    }

    // The armed rotation commands -- like thrust -- are re-applied before
    // EVERY substep (h = that substep's duration); applied once per tick
    // they would act only during the first substep, cutting the delivered
    // authority to 1/n and making it worse at warp.
    void applyRotationForce(double h) {
        if(firstWheel() == nullptr) { return; }
        /* Manual stick: standard aviation mapping, body-relative.
           Pitch (W/S) about the ship's right axis, yaw (A/D) about its
           up axis, roll (Q/E) about the nose. The camera tracks the
           ship's attitude, so these read consistently on screen
           regardless of the ship's world orientation. Each wheel gets
           its rated torque along the combined axis; diagonals (W+A)
           compose as a vector sum. */
        if(stick[0] != 0.0f || stick[1] != 0.0f || stick[2] != 0.0f) {
            Body *rw0 = firstWheel();
            const glm::dvec3 pitchAxis = -getRelAxis_(rw0, 0);  // right (W/S)
            const glm::dvec3 yawAxis   = -getRelAxis_(rw0, 1);  // up    (A/D)
            const glm::dvec3 rollAxis  =  getRelAxis_(rw0, 2);  // nose  (Q/E)
            const glm::dvec3 worldAxis =
                (double)stick[0] * rollAxis
                + (double)stick[1] * pitchAxis
                + (double)stick[2] * yawAxis;
            for(Part *p : parts) {
                if(!p->isWheel()) { continue; }
                ApplyTorque(p->body, (double)p->wheelTorque() * worldAxis);
            }
        }
        /* Autopilot: one authority-bounded step of the slew/kill-rot law
           (h = this substep's duration, so the law re-evaluates per
           substep -- the stable form of the same law). Every directional
           mode slews the nose toward its target direction (slewTargetDir);
           kill-rot damps the spin directly instead of chasing a direction. */
        if(slew == SlewKillRot) {
            killRotStep(h);
        }
        else if(slew != SlewNone) {
            slewToward(slewTargetDir(), h);
        }
    }

    /* Autopilot diagnostic (throttled; called from the tick when
       --slew-log is set). Prints the slew error angle, the ship's angular
       velocity DECOMPOSED into the slew axis / nose-roll / the third axis
       (so an uncontrolled spin shows up as nonzero roll/third even while
       the slew-axis rate is being driven to zero), and the braking-curve
       rate the law wants right now. This is the instrument for hunting the
       prograde wobble: watch E and w_slew for a sustained oscillation, and
       roll/third for a residual spin the law is not killing. */
    void slew_log(double time) {
        if(slew == SlewNone) { return; }
        Body *wheel = firstWheel();
        if(wheel == nullptr) { return; }
        const glm::dvec3 facing = getRelAxis_(wheel, 2);
        if(slew == SlewKillRot) {
            const glm::dvec3 w = GetAngVelocity(wheel);
            printf("[slew] t=%.3f mode=killrot |w|=%.4f rad/s "
                   "w=[%+.4f %+.4f %+.4f]\n",
                   time, glm::length(w), w.x, w.y, w.z);
            fflush(stdout);
            return;
        }
        glm::dvec3 target = slewTargetDir();
        const char *mode;
        switch(slew) {
            case SlewPrograde:   mode = "prograde"; break;
            case SlewRetrograde: mode = "retrograde"; break;
            case SlewRadialOut:  mode = "radial-out"; break;
            case SlewRadialIn:   mode = "radial-in"; break;
            case SlewNormal:     mode = "normal"; break;
            case SlewAntiNormal: mode = "anti-normal"; break;
            default:             mode = "slew"; break;
        }
        if(glm::length2(target) < 0.5) { target = glm::dvec3(0.0); }
        target = glm::normalize(target);
        const double E = glm::acos(glm::clamp(glm::dot(facing, target), -1.0, 1.0));
        glm::dvec3 axis = glm::cross(facing, target);
        if(glm::length2(axis) < 1e-12) {
            axis = (std::fabs(facing.y) > 0.9) ? glm::dvec3(1, 0, 0) : glm::dvec3(0, 1, 0);
            axis = glm::normalize(axis - facing * glm::dot(axis, facing));
        }
        axis = glm::normalize(axis);
        const glm::dvec3 rollAxis  = glm::normalize(facing);
        const glm::dvec3 thirdAxis = glm::cross(axis, rollAxis);
        const glm::dvec3 w = GetAngVelocity(wheel);
        const double w_slew  = glm::dot(w, axis);
        const double w_roll  = glm::dot(w, rollAxis);
        const double w_third = glm::dot(w, thirdAxis);
        const glm::dmat3 I = getInertia();
        const double Ieff = glm::dot(axis, I * axis);
        const double alpha = (Ieff > 0.0) ? maxTorque() / Ieff : 0.0;
        const double w_des = (alpha > 0.0) ? std::sqrt(2.0 * alpha * E) : 0.0;
        printf("[slew] t=%.3f mode=%s E=%.4f rad (%.2f deg) "
               "w_slew=%+.4f w_roll=%+.4f w_third=%+.4f |w|=%.4f "
               "alpha=%.3f Ieff=%.0f w_des=%+.4f "
               "nose=[%+.3f %+.3f %+.3f] target=[%+.3f %+.3f %+.3f]\n",
               time, mode, E, glm::degrees(E),
               w_slew, w_roll, w_third, glm::length(w),
               alpha, Ieff, w_des,
               facing.x, facing.y, facing.z, target.x, target.y, target.z);
        fflush(stdout);
    }

    /* --att-log: the ship's nose (local +Z of the hull, world coords) and
       the hull's angular velocity (world coords), for the attitude-physics
       e2e test. The angular velocity must come from a HULL part, not the
       reaction wheel: the wheel spins relative to the hull, so its own
       GetAngVelocity is the wheel's spin, not the ship's. Every hull part
       shares the ship's rigid-body angular velocity, so any one works. */
    void att_log(double time) {
        if(parts.empty()) { return; }
        Body *hull = nullptr;
        for(Part *p : parts) {
            if(!p->isWheel()) { hull = p->body; break; }
        }
        if(!hull) { hull = parts[0]->body; }
        const glm::dvec3 nose = getRelAxis_(hull, 2);
        const glm::dvec3 w = GetAngVelocity(hull);
        printf("[attlog] t=%.3fs nose=[%+.4f %+.4f %+.4f] "
               "w=[%+.4f %+.4f %+.4f] |w|=%.4f rad/s\n",
               time, nose.x, nose.y, nose.z,
               w.x, w.y, w.z, glm::length(w));
        fflush(stdout);
    }

    /* the largest wheel's rated torque (N m) -- the per-wheel rating for
       the HUD; the ship's TOTAL wheel authority is maxTorque() (the sum) */
    float GetWheelTorque() {
        double t = 0.0;
        for(Part *p : parts) {
            if(p->isWheel() && p->wheelTorque() > t) { t = p->wheelTorque(); }
        }
        return (float)t;
    }

    /* number of reaction-wheel parts (the torque in slewToward / killRotStep
       is distributed evenly across them). Replaces m_reaction_wheels.size(). */
    size_t numWheels() {
        size_t n = 0;
        for(Part *p : parts) { if(p->isWheel()) { n++; } }
        return n;
    }

    /* disarm the armed thrust (called once per tick, like clearRotCmd,
       so a tick without the keys doesn't keep firing) */
    void clearThrust() {
        for(Part *p : parts) { p->armedThrust = 0.0f; }
    }

    /* disarm the armed rotation commands (called once per tick, so a tick
       without the keys doesn't keep rotating) */
    void clearRotCmd() {
        stick[0] = stick[1] = stick[2] = 0.0f;
        slew = SlewNone;
    }

    /* called when control moves to ANOTHER ship: zero the throttle and
       clear the armed thrust + rotation commands, so this ship just
       coasts under its own physics from here on (no residual forces,
       no fuel flow). Control input reaches only the active ship. */
    void releaseControl() {
        thruster_util = 0.0f;
        clearThrust();
        clearRotCmd();
    }

    /* Separate `stage`: cut the welds joining it to the rest of the ship,
       remove its parts from the Bullet world and from this ship's part
       set, and delete them. The surviving parts keep their relative
       geometry (their internal welds are untouched). Refuses to drop the
       whole ship. Returns the number of parts dropped (0 = no-op). Call at
       a tick boundary, not mid-substep. */
    int separateStage(int stage) {
        const size_t n = parts.size();
        if(n < 2) { return 0; }   // single-part ship: nothing to separate

        /* a part is "dropped" iff its stage == `stage` (no separate
           bookkeeping; each Part carries its own stage). */
        size_t dropped = 0;
        for(Part *p : parts) { if(p->stage == stage) { dropped++; } }
        if(dropped == 0) { return 0; }   // nothing on this stage
        if(dropped == n) { return 0; }   // can't drop the whole ship
        const bool controllerDropped =
            (controller != nullptr && controller->stage == stage);

        /* a constraint is cut if EITHER end is on the dropping stage (the
           stage interface AND the stage's internal welds). A Part* link is
           stable, so the survivors keep their (still-alive) endpoints -- no
           index remapping (the old computeStageSplit is gone). */
        std::vector<bool> isCut(constraints.size(), false);
        for(size_t c = 0; c < constraints.size(); c++) {
            if(constraintLinks[c].first->stage == stage ||
               constraintLinks[c].second->stage == stage) { isCut[c] = true; }
        }

        /* collect the survivors FIRST (their Part* stay valid), then free the
           dropped parts -- so no freed pointer is ever dereferenced. */
        std::vector<Part *> keepParts;
        for(Part *p : parts) { if(p->stage != stage) { keepParts.push_back(p); } }

        /* 1) Detach the cut constraints (they reference the dropped bodies). */
        for(size_t c = 0; c < constraints.size(); c++) {
            if(isCut[c]) { Detach(constraints[c]); }
        }
        /* 2) Unregister + delete the dropped parts (~Part frees the Body). */
        for(Part *p : parts) {
            if(p->stage != stage) { continue; }
            RemoveBody(p->body);
            delete p;
        }
        parts.swap(keepParts);
        /* 3) Keep only the surviving constraints / links / anchors (every
           kept link's endpoints are still-alive Parts). */
        std::vector<void *> keepCons;
        std::vector<std::pair<Part *, Part *>> keepLinks;
        std::vector<std::pair<glm::dvec3, glm::dvec3>> keepAnchors;
        for(size_t c = 0; c < constraints.size(); c++) {
            if(isCut[c]) { continue; }
            keepCons.push_back(constraints[c]);
            keepLinks.push_back(constraintLinks[c]);
            keepAnchors.push_back(constraintAnchors[c]);
        }
        constraints.swap(keepCons);
        constraintLinks.swap(keepLinks);
        constraintAnchors.swap(keepAnchors);
        /* 4) If the controller was on the dropped stage, fall back to the
           first survivor (a Part* is stable -- no index remapping). */
        if(controllerDropped) { controller = parts[0]; }
        NeverSleep(controller->body);
        /* 5) Disarm any thrust (the split just happened). */
        clearThrust();
        return (int)dropped;
    }

    /* This ship's part frame -> renderFrame. Usually the identity
       (renderFrame is this ship's own frame); an idle ship that switched
       SOI while another ship was being controlled lives in a different
       frame, so transform its parts into the render frame first. Draw
       uses it to bring the parts into the view; picking (src/pick.cpp)
       inverts it to bring the ray into the parts' frame. */
    glm::dmat4 renderXform(Frame *renderFrame) const {
        if(frame == renderFrame) { return glm::dmat4(1.0); }
        return glm::translate(frame->GetPositionRelTo(renderFrame))
             * glm::dmat4(frame->GetOrientRelTo(renderFrame));
    }

    void Draw(const Camera* camera, Frame *renderFrame) {
        // Light direction at the ship (sun -> ship COM), in the render frame's
        // axes where the part normals end up after the xform below. Using the
        // ship's own position -- not the SOI body's center as SunlightDir does
        // -- is what keeps it defined in the Kerbol SOI, where the SOI center
        // IS the star and sun->center is a zero vector (normalize -> NaN).
        const glm::dvec3 com_root =
            frame->root_orient * get_center_of_mass() + frame->root_pos;
        glm::vec3 sunlightVec =
            glm::vec3(TerrainBody::LightDirFrom(com_root, sun, renderFrame));

        const glm::dmat4 xform = renderXform(renderFrame);

        for(Part *p : parts) {
            // Per-part terrain shadow
            const float shadow =
                ComputeTerrainShadow(m_parent, frame, GetPosition(p->body), sun);
            p->body->Draw(camera, sunlightVec, shadow, xform);
        }
    }

    // Single place to control the ship. While paused (simActive == false)
    // every command is dropped, so nothing accumulates in the rigid bodies
    // (a force/torque left in Bullet would dump out as a velocity kick on
    // resume) and settings like throttle stay frozen.
    /* step = the tick's simulated duration (dt * time_accel); only Thrust
       uses it (to scale this tick's fuel flow). */
    void Command(ShipCmd cmd, bool simActive, double step = 0.0) {
        if(not simActive)
            return;
        switch(cmd.type) {
            case ThrottleUp:
                adjustThrottle(+0.01);
                break;
            case ThrottleDown:
                adjustThrottle(-0.01);
                break;
            case Thrust:
                ApplyThrust(step);
                break;
            // Stick components in the camera frame (see the `stick` member).
            case Pitch:
                stick[1] = (cmd.amount >= 0) ? +1.0f : -1.0f;
                break;
            case Yaw:
                stick[2] = (cmd.amount >= 0) ? +1.0f : -1.0f;
                break;
            case Roll:
                stick[0] = (cmd.amount >= 0) ? +1.0f : -1.0f;
                break;
            case KillRot:
                slew = SlewKillRot;
                break;
            case Prograde:
                slew = SlewPrograde;
                break;
            case Retrograde:
                slew = SlewRetrograde;
                break;
        }
    }

    glm::dvec3 GetVel() {
        return GetVelocity(controller->body);
    }

protected:
    // Control implementation: applies forces/torques to the Bullet bodies
    // directly, so it is reachable only through Command() above.
    // (protected, not private: the EVA kerbal (src/eva.h) reuses the
    // rotation-model helpers below for its own attitude law.)
    void adjustThrottle(float delta) {
        thruster_util += delta;
        if(thruster_util > 1) { thruster_util = 1; }
        if(thruster_util < 0) { thruster_util = 0; }
    }

    /* the ship's full-throttle thrust RIGHT NOW (N) = the sum of the ACTIVE
       stage's engines' full thrust (each T = (H2 + LOX flow) x ve = 2 x
       fuel_rate x ve, both propellants end up in the plume), scaled by
       exhaust_scale (the test knob). Only the active stage fires, so this
       is the usable thrust; for a single-stage ship it equals the grand
       total. */
    float GetActiveThrust() {
        const int as = activeStage();
        double t = 0;
        for(Part *p : parts) {
            if(p->isThruster() && p->stage == as) { t += p->thrust(); }
        }
        return (float)(t * exhaust_scale);
    }

    /* Called once per physics tick (step = the tick's simulated duration).
       Consumes the tick's fuel and arms the per-thruster thrust; the force
       itself is applied by applyThrustForce() before EVERY substep below.
       A thruster that can't consume its flow this tick doesn't thrust. Only
       the ACTIVE stage's thrusters fire (and they draw the active stage's
       tanks -- see consumeResourceMass), so a non-active engine is skipped
       before any fuel is spent. */
    void ApplyThrust(double step) {
        if(thruster_util == 0.0f) { return; } /* zero throttle: no burn, no plume */
        const int as = activeStage();
        for(Part *p : parts) {
            if(!p->isThruster()) { continue; }
            if(p->stage != as) { continue; } /* not the live stage */
            const float flow =
                (float)(p->rate() * (double)thruster_util * step); /* kg this tick, per tank */
            if(consumeResourceMass(ResourceType::Hydrogen, flow) and
               consumeResourceMass(ResourceType::LOX,      flow))
                {
                    p->armedThrust =
                        (float)(p->thrust() * thruster_util * exhaust_scale);
                    m_thrust = 1.0;
                }
        }
    }

    // --- physical rotation model (private law implementation) -------------
    // The reaction wheel is rated at GetWheelTorque() N m -- the most torque
    // it can apply to the ship -- so the ship's angular authority is
    // alpha = maxTorque() / I (rad/s^2) with I the ship's total moment of
    // inertia (kg m^2, from Bullet). Stick, prograde/retrograde slew and
    // kill-rot all work within that authority, so no command can be more
    // forceful than a maxed manual stick. (The thrust analogue: T = mdot*ve.)

    double maxTorque() {
        double t = 0;
        for(Part *p : parts) {
            if(p->isWheel()) { t += p->wheelTorque(); }
        }
        return t;
    }

    /* The ship's total moment of inertia about its COM (kg m^2): each
       part's local inertia (as Bullet has it) in world coordinates, plus
       the parallel-axis term for its offset from the ship's COM. This is
       the inertia a torque actually moves -- the denominator of the
       wheel's authority. */
    glm::dmat3 getInertia() {
        const glm::dvec3 com = get_center_of_mass();
        glm::dmat3 I = glm::dmat3(0.0);
        for(Part *p : parts) {
            Body *b = p->body;
            const glm::dvec3 d = GetPosition(b) - com;
            const glm::dmat3 R = GetOrient(b);
            const glm::dvec3 il = getInertiaDiag(b);
            /* part's local inertia is diagonal (Bullet stores it that way);
               build the diagonal matrix explicitly -- GLM has no
               vec -> diagonal-mat constructor */
            const glm::dmat3 il_diag(
                il.x, 0.0, 0.0,
                0.0, il.y, 0.0,
                0.0, 0.0, il.z);
            I += R * il_diag * glm::transpose(R);
            I += b->mass * (glm::dot(d, d) * glm::dmat3(1.0) - glm::outerProduct(d, d));
        }
        return I;
    }

    /* The target direction (in the ship's frame) for the current directional
       slew mode. Radial / normal reference the SOI body: its center is the
       frame origin, so `pos` is the radius vector and `vel` the velocity --
       radial is the radius vector, normal the orbital angular-momentum
       direction r x v. The same convention as the navball indicators in
       render.cpp. KillRot / None return zero (slewToward refuses a
       zero-length direction). */
    glm::dvec3 slewTargetDir() {
        const glm::dvec3 pos = get_center_of_mass();
        const glm::dvec3 vel = GetVel();
        switch(slew) {
            case SlewPrograde:   return  vel;
            case SlewRetrograde: return -vel;
            case SlewRadialOut:  return  pos;
            case SlewRadialIn:   return -pos;
            case SlewNormal:     return  glm::cross(pos, vel);
            case SlewAntiNormal: return -glm::cross(pos, vel);
            default:             return glm::dvec3(0.0);
        }
    }

    /* Slew the nose (local +Z) toward `dir` within the wheel's authority:
       the target rate is the braking curve sqrt(2*alpha*E) -- the fastest
       rate from which the ship can still stop exactly at the target
       (E = the error angle) -- capped at E/(2h) so no substep can cross
       the target, and the per-substep rate change is bounded by alpha*h,
       so the command never exceeds a maxed manual stick. */
    void slewToward(glm::dvec3 dir, double h) {
        if(glm::length2(dir) < 1e-12) { return; } /* no direction to align to */
        dir = glm::normalize(dir);
        Body *wheel = firstWheel();
        const glm::dvec3 facing = getRelAxis_(wheel, 2);
        const double E = glm::acos(glm::clamp(glm::dot(facing, dir), -1.0, 1.0));
        if(E < 1e-9) { return; } /* already aligned */
        glm::dvec3 axis = glm::cross(facing, dir); /* + turns the nose toward dir */
        if(glm::length2(axis) < 1e-12) {
            /* nose ~ opposite dir: any axis perpendicular to facing works */
            axis = (std::fabs(facing.y) > 0.9) ? glm::dvec3(1, 0, 0) : glm::dvec3(0, 1, 0);
            axis = glm::normalize(axis - facing * glm::dot(axis, facing));
        }
        axis = glm::normalize(axis);
        const glm::dmat3 I = getInertia();
        const double Ieff = glm::dot(axis, I * axis); /* kg m^2 about the slew axis */
        if(Ieff <= 0.0) { return; }
        const double alpha = maxTorque() / Ieff; /* rad/s^2, wheel-limited */
        const double w_des = std::min(std::sqrt(2.0 * alpha * E), E / (2.0 * h));
        /* Drive the FULL transverse angular velocity (the part perpendicular
           to the nose) toward the braking-curve rate about the slew axis.
           The old torque was along the slew axis ONLY, so the perpendicular
           "third-axis" spin was never damped: any residual spin about it at
           engagement persisted (and grew via gyroscopic coupling), and as the
           slew axis rotated that undamped spin coupled into the nose -- the
           sustained wobble around the prograde/retrograde target. Killing it
           is the fix. Roll about the nose is intentionally left free. */
        const glm::dvec3 w_now = GetAngVelocity(wheel);
        const glm::dvec3 w_transverse = w_now - facing * glm::dot(w_now, facing);
        glm::dvec3 dW = axis * w_des - w_transverse; /* desired change in rate */
        glm::dvec3 torque = I * dW / h;
        /* Authority bound: the wheel pushes at most maxTorque() N m, so scale
           the correction down if it would exceed that. Only active while a
           third-axis spin is present; with none, dW is along the slew axis
           and |torque| == maxTorque exactly as before. */
        const double tq = glm::length(torque);
        if(tq > maxTorque()) { torque *= maxTorque() / tq; }
        for(Part *p : parts) {
            if(!p->isWheel()) { continue; }
            ApplyTorque(p->body, torque / (double)numWheels());
        }
    }

    /* Kill the spin within the wheel's authority: each axis' rate drops by
       min(|w|, alpha*h) per substep -- monotonic, no sign flip, never more
       forceful than a maxed manual stick. */
    void killRotStep(double h) {
        Body *wheel = firstWheel();
        const glm::dvec3 w = GetAngVelocity(wheel);
        if(glm::length(w) < 0.001) { return; } /* at rest: nothing to kill */
        const glm::dmat3 I = getInertia();
        glm::dvec3 torque(0.0);
        for(int i = 0; i < 3; i++) {
            const double Iii = I[i][i];
            if(Iii <= 0.0 || w[i] == 0.0) { continue; }
            const double A = (maxTorque() / Iii) * h; /* max |dw| on this axis */
            const double dw = -w[i] * std::min(1.0, A / std::fabs(w[i]));
            torque[i] = Iii * dw / h; /* |torque[i]| <= maxTorque() */
        }
        for(Part *p : parts) {
            if(!p->isWheel()) { continue; }
            ApplyTorque(p->body, torque / (double)numWheels());
        }
    }

public:

    glm::dmat3 GetOrientRelTo(Body *part, Frame *relTo)
    {
        glm::dmat3 GetOrient(Body *b);
        glm::dmat3 forient = frame->GetOrientRelTo(relTo);
        return forient * GetOrient(part);
    }

    glm::dvec3 GetPositionRelTo(Body *part, Frame *relTo) {
        glm::dvec3 fpos = frame->GetPositionRelTo(relTo);
        glm::dmat3 forient = frame->GetOrientRelTo(relTo);
        return forient * GetPosition(part) + fpos;
    }

    glm::dvec3 GetVelocityRelTo(Body *part, Frame *relTo) {
        glm::dmat3 forient = frame->GetOrientRelTo(relTo);
        glm::dvec3 vel = GetVelocity(part);
        glm::dvec3 pos = GetPosition(part);
        if(frame != relTo) vel += frame->GetStasisVelocity(pos);
        return forient * vel + frame->GetVelocityRelTo(relTo);
    }

    void moveToFrame(Frame *newFrame) {
        void setPosRot(Body *b, glm::dvec3 pos, glm::dmat3 rot);
        glm::dmat3 GetOrient(Body *b);

        for(Part *p : parts) {
            const char *name = p->def->name.c_str();

            glm::dvec3 oldVel = GetVelocity(p->body);
            glm::dvec3 vel = GetVelocityRelTo(p->body, newFrame);
            glm::dvec3 fpos = frame->GetPositionRelTo(newFrame);
            glm::dmat3 forient = frame->GetOrientRelTo(newFrame);

            glm::dvec3 newPos = forient * GetPosition(p->body) + fpos;
            glm::dmat3 newOrient = forient * GetOrient(p->body);

            glm::dvec3 pos = GetPosition(p->body);
            printf("@@@ %s OLD position: %.0f %.0f %.0f\n", name, pos.x, pos.y, pos.z);
            printf("@@@ %s NEW position: %.0f %.0f %.0f\n", name, newPos.x, newPos.y, newPos.z);

            setPosRot(p->body, newPos, newOrient);

            pos = GetPosition(p->body);
            // The stored velocity is the frame-coordinate velocity, so a ship's
            // inertial velocity is R*(v + stasis(p)) + V.  GetVelocityRelTo
            // already added the OLD frame's stasis term; the NEW frame's term
            // must be SUBTRACTED, or the ship's inertial velocity is wrong by
            // 2*stasis and the orbit jumps shape at every inertial->rotational
            // switch.
            glm::dvec3 newVel = vel - newFrame->GetStasisVelocity(pos);
            printf("@@@ %s OLD velocity: %.0f %.0f %.0f\n", name, oldVel.x, oldVel.y, oldVel.z);
            printf("@@@ %s NEW velocity: %.0f %.0f %.0f\n", name, newVel.x, newVel.y, newVel.z);

            SetVelocity(p->body, newVel);
        }
        // The ship lives in the ships list of its SOI body (terrain.h):
        // crossing to another body's SoI is a list move, done here so the
        // body lists always agree with m_parent. (Rare -- this runs once
        // per crossing, not per tick.)
        if(m_parent != nullptr && m_parent != newFrame->body) {
            for(auto it = m_parent->ships.begin();
                it != m_parent->ships.end(); it++) {
                if(*it == this) { m_parent->ships.erase(it); break; }
            }
            newFrame->body->ships.push_back(this);
        }
        frame = newFrame;
        m_parent = newFrame->body;
    }

    /* Per-tick SOI bookkeeping for THIS ship: if the ship is outside the
       current frame's SOI, move to the parent frame; else if it has
       entered a child's SOI, move to the nearest such child. Called once
       per tick, per ship (the frame tree is shared; each ship tracks its
       own position in it). */
    void switchFrames() {
        const glm::dvec3 com = get_center_of_mass();
        double ship_r = glm::length(com);
        if(ship_r > frame->soi + 10000) {
            // switching to parent SOI if there is one
            if(frame->parent != NULL) {
                glm::dvec3 pos = GetPosition(controller->body);
                printf("@@@ %s switching frame from %s to parent %s\n",
                       name.c_str(), frame->name.c_str(),
                       frame->parent->name.c_str());
                glm::dvec3 offset = frame->GetPositionRelTo(frame->parent);
                printf("@@@ Frame offset: %.0f %.0f %.0f\n", offset.x, offset.y, offset.z);
                printf("@@@@@ OLD position: %.0f %.0f %.0f\n", pos.x, pos.y, pos.z);
                moveToFrame(frame->parent);
                pos = GetPosition(controller->body);
                printf("@@@@@ NEW position: %.0f %.0f %.0f\n", pos.x, pos.y, pos.z);
            }
        }
        else {
            // check if we've entered a child SOI
            for(auto&& child : frame->children) {
                double dist = glm::length(GetPositionRelTo(controller->body, child));
                if(dist < child->soi - 10000) {
                    printf("@@@ %s switching frame from %s to child %s, distance: %.0f\n",
                           name.c_str(), frame->name.c_str(),
                           child->name.c_str(), dist);
                    moveToFrame(child);
                    break;
                }
            }
        }
    }

    /* Write the rail state into the parked part transforms (once per
       tick). Draw, get_center_of_mass and everything else that reads
       Bullet transforms then sees the railed ship's current pose even
       though its bodies are not in the world. The velocities are kept in
       sync too -- the cluster is rigid and torque-free, so every part
       shares the rail velocity with zero spin, and readers like
       --orbit-log and the HUD fit their elements to consistent data. */
    void writeRailPose() {
        for(Part *p : parts) {
            setPosRot(p->body,
                      rail_pos + rail_orient * p->railRelPos,
                      rail_orient * p->railRelRot);
            SetVelocity(p->body, rail_vel);
            SetAngVelocity(p->body, glm::dvec3(0.0));
        }
    }

    /* The COM's osculating orbit dips into the terrain band (periapsis
       within 3 km of the surface): sitting on / skimming the ground rather
       than coasting clear of it. */
    bool inTerrainBand() {
        Frame *inertial = frame->getNonRotFrame();
        glm::dvec3 p = get_center_of_mass();
        glm::dvec3 v(0.0);
        double mtot = 0.0;
        for(Part *p2 : parts) {
            v += GetVelocity(p2->body) * p2->body->mass;
            mtot += p2->body->mass;
        }
        v /= mtot;
        if(frame != inertial) {
            v += frame->GetStasisVelocity(p);
            v = frame->GetOrientRelTo(inertial) * v + frame->GetVelocityRelTo(inertial);
            p = frame->GetOrientRelTo(inertial) * p + frame->GetPositionRelTo(inertial);
        }
        const OrbitElements el = computeOrbitElements(p, v, inertial->body->mu);
        return el.periapsis <= inertial->body->radius + 3000.0;
    }

    /* Rails classification: a FLYING ship (periapsis clear of the terrain
       band) coasts on its conic; a GROUNDED one (periapsis inside the
       band) can only freeze in its rotating surface frame. Anything else
       -- e.g. a suborbital descent -- is not rail-eligible. */
    bool canRail() {
        if(onRails) { return true; }
        if(inTerrainBand()) {
            return frame->isRotFrame();   // grounded: freeze needs the surface frame
        }
        return true;
    }

    /* Park this ship out of the physics world and coast it analytically.
       Refuses (returns false) and changes nothing if the ship is not
       rail-eligible (see canRail). Flying ships follow their conic in the
       body's inertial node; grounded ships freeze in the rotating surface
       frame. */
    bool goOnRails() {
        if(onRails) { return true; }
        if(!canRail()) { return false; }

        /* COM state in the body's inertial frame node, where the
           trajectory is a Kepler conic (same transform the HUD uses).
           Cluster velocity = mass-weighted mean of the part velocities
           (the rigid cluster coasts as one body; residual spin is
           discarded with the attitude). */
        Frame *oldFrame = frame;
        Frame *inertial = frame->getNonRotFrame();
        glm::dvec3 p = get_center_of_mass();
        const glm::dvec3 com_frame = p;   // pre-transform, old frame coords
        glm::dvec3 v(0.0);
        double mtot = 0.0;
        for(Part *p2 : parts) {
            v += GetVelocity(p2->body) * p2->body->mass;
            mtot += p2->body->mass;
        }
        v /= mtot;
        const glm::dvec3 vel_frame = v;   // pre-transform, old frame coords
        if(frame != inertial) {
            v += frame->GetStasisVelocity(p);
            v = frame->GetOrientRelTo(inertial) * v + frame->GetVelocityRelTo(inertial);
            p = frame->GetOrientRelTo(inertial) * p + frame->GetPositionRelTo(inertial);
        }

        const OrbitElements el = computeOrbitElements(p, v, inertial->body->mu);
        const bool grounded = el.periapsis <= inertial->body->radius + 3000.0;

        /* the cluster pose at park time, relative to its COM (cluster
           axes == old frame axes; rail_orient carries them into the
           inertial node and then holds inertially) */
        for(Part *p2 : parts) {
            p2->railRelPos = GetPosition(p2->body) - com_frame;
            p2->railRelRot = GetOrient(p2->body);
        }

        if(grounded) {
            /* freeze: the pose is static in the rotating surface frame
               (its transforms already are), so the rail state just holds
               it; the planet's spin carries it via the render transform. */
            rail_pos = com_frame;
            rail_vel = vel_frame;
            rail_orient = glm::dmat3(1.0);
            railFrozen = true;
        } else {
            rail_pos = p;
            rail_vel = v;
            rail_orient = oldFrame->GetOrientRelTo(inertial);
            frame = inertial;   // on rails, ship->frame == its inertial node
            railFrozen = false;
        }

        /* out of the world: welds first (they reference the bodies) */
        for(size_t c = 0; c < constraints.size(); c++) {
            Detach(constraints[c]);
        }
        constraints.clear();
        for(Part *p2 : parts) { RemoveBody(p2->body); }

        onRails = true;
        if(!railFrozen) { writeRailPose(); }
        if(grounded) {
            printf("@@@ %s frozen on rails (grounded around %s)\n",
                   name.c_str(), frame->body->name.c_str());
        } else {
            printf("@@@ %s parked on rails around %s: sma=%.6g m ecc=%.4f\n",
                   name.c_str(), inertial->body->name.c_str(),
                   el.semi_major, el.ecc);
        }
        return true;
    }

    /* Re-enter physics from rails: rebuild the Bullet state from the rail
       state and hand the cluster back to the integrator. Pose and velocity
       already track the rail state (writeRailPose), so this is just
       re-register and re-weld. */
    void leaveRails() {
        if(!onRails) { return; }
        writeRailPose();
        for(Part *p : parts) {
            AddPhysicsBody(p->body);
        }
        /* GlueTogether locks the CURRENT relative pose, which is exactly
           the parked geometry, so the stored anchors reproduce the welds. */
        for(size_t c = 0; c < constraintLinks.size(); c++) {
            constraints.push_back(GlueTogether(constraintLinks[c].first->body,
                                               constraintLinks[c].second->body,
                                               constraintAnchors[c].first,
                                               constraintAnchors[c].second));
        }
        NeverSleep(controller->body);
        onRails = false;
        railFrozen = false;
        printf("@@@ %s left the rails around %s\n",
               name.c_str(), frame->body->name.c_str());
    }

    /* Per-tick rail advance: propagate the conic by the tick's simulated
       duration (exact for any step size), check SOI boundaries, refresh
       the parked transforms. A frozen (grounded) ship has nothing to
       propagate: its pose is static in the rotating frame. */
    void railsTick(const double step) {
        if(!onRails || railFrozen) { return; }
        propagateKepler(rail_pos, rail_vel, frame->body->mu, step,
                        rail_pos, rail_vel);
        railsSwitchFrames();
        writeRailPose();
    }

    /* SOI bookkeeping for a railed ship (the switchFrames() analog): the
       rail conic is only valid around frame->body while the ship stays in
       that SOI. The rotating child frame is the same body -- never a
       switch candidate; physics ships drop into it after the handoff. */
    void railsSwitchFrames() {
        const double r = glm::length(rail_pos);
        if(r > frame->soi + 10000) {
            if(frame->parent != NULL) {
                printf("@@@ %s rails switching frame from %s to parent %s\n",
                       name.c_str(), frame->name.c_str(),
                       frame->parent->name.c_str());
                moveToRailFrame(frame->parent);
            }
        } else {
            for(auto&& child : frame->children) {
                if(child->body == frame->body) { continue; }
                // ship position in the child's coordinates (the same
                // transform GetPositionRelTo(part, child) applies)
                const glm::dvec3 rel = frame->GetOrientRelTo(child) * rail_pos
                                     + frame->GetPositionRelTo(child);
                const double dist = glm::length(rel);
                if(dist < child->soi - 10000) {
                    printf("@@@ %s rails switching frame from %s to child %s, distance: %.0f\n",
                           name.c_str(), frame->name.c_str(),
                           child->name.c_str(), dist);
                    moveToRailFrame(child);
                    break;
                }
            }
        }
    }

    /* Re-anchor the rail state on another frame (moveToFrame's math for
       the analytic state; the new frame is inertial, so no stasis). */
    void moveToRailFrame(Frame *newFrame) {
        const glm::dmat3 O = frame->GetOrientRelTo(newFrame);
        rail_vel = O * rail_vel + frame->GetVelocityRelTo(newFrame);
        rail_pos = O * rail_pos + frame->GetPositionRelTo(newFrame);
        rail_orient = O * rail_orient;
        // Same list move as moveToFrame (the ship follows its SoI body).
        if(m_parent != nullptr && m_parent != newFrame->body) {
            for(auto it = m_parent->ships.begin();
                it != m_parent->ships.end(); it++) {
                if(*it == this) { m_parent->ships.erase(it); break; }
            }
            newFrame->body->ships.push_back(this);
        }
        frame = newFrame;
        m_parent = newFrame->body;
    }
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
   coasts out of the body's SOI on its own (no thrusting). */
struct ScenarioDef {
    const char *name;
    bool on_pad;
    double alt_frac; // circular: fraction of (rot-frame SOI - radius)
    bool polar;
    int ell_phase;   // -1: circular; 0: at periapsis; 1: at apoapsis; 2: at 90 deg
    double peri_alt; // ellipse: periapsis altitude above the body radius (m)
    double apo_alt;  // ellipse: apoapsis altitude above the body radius (m)
    double esc_frac; // escape: launch speed in local escape velocities (0 = not escape)
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
