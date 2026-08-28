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
    Pitch,    // amount: +1 = W, -1 = S
    Yaw,      // amount: +1 = A, -1 = D
    Roll,     // amount: +1 = Q, -1 = E
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
    SlewKillRot      /* kill the spin */
};

class Vehicle {
public:
    std::string name;   // display name (def name, disambiguated in main)
    std::string defPath; // the ship def file it was built from ("" = test ship);
                         // lets a runtime spawn duplicate this ship's design
    std::vector<Body *> parts;
    std::vector<ResourceContent> partResources;

    TerrainBody *m_parent;
    Frame *frame;
    TerrainBody *sun = nullptr; // the star (light source); set in main
    float m_thrust;

    glm::dvec3 m_com;

    Body *controller;
    int controllerIndex = -1;   // part index; -1 = last part (set by build_ship)
    std::vector<Body *> m_thrusters;
    std::vector<Body *> m_reaction_wheels;
    std::vector<void *> constraints;
    /* which parts each constraint joins: (parentIdx, childIdx) into
       `parts`, parallel to `constraints`. Staging splits on these links. */
    std::vector<std::pair<size_t, size_t>> constraintLinks;
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
    std::vector<glm::dvec3> rail_rel_pos;     // parked part pose, cluster axes
    std::vector<glm::dmat3> rail_rel_rot;

    /* Per-part catalog specs (parallel to parts; set by build_ship() before
       init()). init() copies the behavior values into the per-thruster /
       per-wheel vectors below, so the catalog may be freed afterwards. */
    std::vector<const PartDef *> partDefs;
    /* per-part stage number (parallel to parts; set by build_ship from the
       ship def). 1 = single stage. Drives engine/fuel gating + separation. */
    std::vector<int> partStages;
    std::vector<double> m_thrusterThrust;  // N at full throttle (T = 2*rate*ve)
    std::vector<double> m_thrusterRate;    // kg/s per tank at full throttle
    std::vector<glm::dvec2> m_thrusterDims;  // (radius, height) m, parallel to m_thrusters
    std::vector<int> m_thrusterStage;     // stage of each thruster (parallel to m_thrusters)
    std::vector<double> m_wheelTorque;     // N m, rated
    double m_exhaustVel = 0;               // m/s (first engine; delta-v estimate)
    /* the armed per-thruster force for THIS tick (N at current throttle);
       armed by ApplyThrust, applied per substep, disarmed by clearThrust */
    std::vector<float> m_armedThrust;

    float thruster_util = 1.0;
    float thrust_mag = 0.0;   /* N this tick (sum of m_armedThrust); armed by ApplyThrust, cleared by clearThrust */

    /* Rotation is armed once per tick (Command) and executed per SUBSTEP
       (applyRotationForce, before every stepSimulation) -- like thrust,
       because Bullet clears the accumulated torque on each stepSimulation.
       stick: +-1 per commanded body axis (diagonals allowed, e.g. W+A);
       slew:  the autopilot target (exclusive). */
    float stick[3] = {0.0f, 0.0f, 0.0f};
    int slew = SlewNone;

    void setRoot(Body *part) {
        parts = { part };
    }

    /* Weld `part` to the part at `parentIdx` at the given LOCAL anchor
       points (which must coincide in world space -- the 6DOF weld enforces
       zero relative linear offset). Records the link for staging. The
       caller has already setPosRot-ed the child to the matching pose. */
    void attach(Body *part, size_t parentIdx,
                const glm::dvec3 &parentAnchor, const glm::dvec3 &childAnchor) {
        void *constraint = GlueTogether(parts[parentIdx], part,
                                        parentAnchor, childAnchor);
        parts.push_back(part);
        constraints.push_back(constraint);
        constraintLinks.push_back(std::make_pair(parentIdx, parts.size() - 1));
        constraintAnchors.push_back(std::make_pair(parentAnchor, childAnchor));
    }

    void attachDown(Body *part, const PartDef *def) {
        /* weld at the part faces: parent bottom (-h/2) to child top (+h/2);
           generalizes the old hardcoded +-1 m (2 m parts). partDefs is
           parallel to parts, so the parent's def is the last one pushed. */
        const PartDef *parent = partDefs.back();
        attach(part, parts.size() - 1,
               glm::dvec3(0.0, 0.0, -parent->height / 2.0),
               glm::dvec3(0.0, 0.0,  def->height / 2.0));
    }

    void attachRadial(Body *part, const PartDef *def) {
        /* weld the part to the parent's SIDE: the part's local +Z axis is
           rotated to the parent's local +X (call site), so the part's
           bottom face (local -h/2) touches the parent's side at +radius.
           The anchors coincide in world space at the tangent point
           (parent local (r,0,0) == part local (0,0,-h/2)).
           Same partDefs convention as attachDown. */
        const PartDef *parent = partDefs.back();
        attach(part, parts.size() - 1,
               glm::dvec3(parent->radius, 0.0, 0.0),
               glm::dvec3(0.0, 0.0, -def->height / 2.0));
    }

    void attachSide(Body *part, const PartDef *def) {
        /* weld the part to the parent's SIDE with PARALLEL axes: the part
           keeps the parent's local +Z axis (no rotation at the call site),
           sits along the parent's local +X, and its cylindrical surface
           touches the parent's at +radius. The anchors coincide in world
           space at the tangent point (parent local (r,0,0) == part local
           (-r,0,0)). Unlike attachRadial the child is NOT rotated, so this
           is the "side by side, parallel axes" case.
           Same partDefs convention as attachDown. */
        const PartDef *parent = partDefs.back();
        attach(part, parts.size() - 1,
               glm::dvec3(parent->radius, 0.0, 0.0),
               glm::dvec3(-def->radius, 0.0, 0.0));
    }

    void init() {
        partResources.resize(parts.size());
        /* the cockpit part; build_ship() sets controllerIndex from the ship
           def (default -1 = last part, the old behavior) */
        int ci = controllerIndex < 0 ? (int)parts.size() - 1 : controllerIndex;
        controller = parts[(size_t)ci];
        NeverSleep(controller);
        if(partDefs.size() != parts.size()) {
            throw std::runtime_error("Vehicle::init: partDefs must be parallel to parts");
        }
        if(partStages.size() != parts.size()) {
            throw std::runtime_error("Vehicle::init: partStages must be parallel to parts");
        }
        /* propellant reservoirs: seed each tank part's resources so the
           thrusters can draw from them (they shed mass as they burn). Only
           done at construction -- separateStage() must NOT re-seed, so this
           stays here and out of rebuildBehavior(). */
        for(size_t i = 0; i < parts.size(); i++) {
            const PartDef *d = partDefs[i];
            bool has_capacity = false;
            for(size_t r = 0; r < d->capacity.size(); r++) {
                if(d->capacity[r] > 0.0f) { has_capacity = true; }
            }
            if(has_capacity) {
                for(int r = 0; r < (int)ResourceType::Num; r++) {
                    partResources[i].capacity[r] = d->capacity[r];
                    partResources[i].current[r] = d->capacity[r];
                }
            }
        }
        rebuildBehavior();
    }

    /* (Re)build the per-thruster / per-wheel behavior vectors from partDefs.
       Safe to call again after separateStage() has shrunk the part set: it
       touches ONLY the m_* vectors -- never partResources (fuel is never
       re-seeded) and never the parts/constraints themselves. */
    void rebuildBehavior() {
        m_thrusters.clear();
        m_reaction_wheels.clear();
        m_thrusterThrust.clear();
        m_thrusterRate.clear();
        m_thrusterDims.clear();
        m_thrusterStage.clear();
        m_wheelTorque.clear();
        m_exhaustVel = 0;
        for(size_t i = 0; i < parts.size(); i++) {
            const PartDef *d = partDefs[i];
            /* behavior is field-driven (see shipdef.h): a part may carry
               any combination of these, so the checks are independent */
            if(d->torque > 0.0) {
                m_reaction_wheels.push_back(parts[i]);
                m_wheelTorque.push_back(d->torque);
            }
            if(d->fuel_rate > 0.0 && d->exhaust_velocity > 0.0) {
                m_thrusters.push_back(parts[i]);
                m_thrusterThrust.push_back(d->fullThrust());
                m_thrusterRate.push_back(d->fuel_rate);
                m_thrusterDims.push_back(glm::dvec2(d->radius, d->height));
                m_thrusterStage.push_back(partStages[i]);
                if(m_exhaustVel == 0.0) { m_exhaustVel = d->exhaust_velocity; }
            }
        }
        /* no thrust may be armed across a rebuild (a split just happened) */
        m_armedThrust.assign(m_thrusters.size(), 0.0f);
    }

    /* Draw `amt` kg of `type` from a tank on the ACTIVE stage only (stages
       are self-contained: a stage burns its own propellant, so the booster's
       engines never drain the upper stage's tanks). Returns true if fuel was
       consumed. amt is the kg consumed THIS tick (the caller scales the kg/s
       flow by the tick's simulated time). */
    bool consumeResourceMass(enum ResourceType type, float amt /* kg */) {
        const int as = activeStage();
        for(size_t i = 0; i < parts.size(); i++) {
            if(partStages[i] != as) { continue; }
            if(partResources[i].current[(int)type] >= amt) {
                partResources[i].current[(int)type] -= amt;
                parts[i]->mass -= amt; /* why does Body have mass at all? */
                void SetMass(Body *body, double newMass);
                SetMass(parts[i], parts[i]->mass);
                return true;
            }
        }
        return false;
    }

    float getFuelMass(const std::vector /* eh */ <enum ResourceType>& types) {
        float fuel = 0;
        for(auto&& type : types) {
            for(auto&& partResource : partResources) {
                fuel += partResource.current[(int)type];
            }
        }
        return fuel;
    }

    float getDeltaV() {
        float remaining_fuel = getFuelMass({ ResourceType::Hydrogen, ResourceType::LOX }); /* kg */
        return (float)m_exhaustVel * log(getMass() / (getMass() - remaining_fuel));
    }

    /* TODO should be cached per frame */
    float getMass() {
        float r = 0;
        for(auto&& part : parts) {
            r += part->mass;
        }
        return r;
    }

    /* The active (currently live) stage: the lowest stage number still on
       the ship. Stages are dropped low-to-high, so this is what fires and
       what separateStage() detaches next. */
    int activeStage() {
        if(partStages.empty()) { return 1; }
        int s = partStages[0];
        for(size_t i = 1; i < partStages.size(); i++) {
            if(partStages[i] < s) { s = partStages[i]; }
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
        for(size_t i = 0; i < partStages.size(); i++) {
            if(partStages[i] > n) { n = partStages[i]; }
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
        for(auto&& part : parts) {
            SetVelocity(part, vel);
        }
    }

    void setPosition(glm::dvec3 pos) {
        void SetPosition(Body *b, glm::dvec3 com, glm::dvec3 pos);
        for(auto&& part : parts) {
            SetPosition(part, get_center_of_mass(), pos);
        }
    }

    const glm::dvec3& get_center_of_mass(void) {
        double total_mass = 0;
        for(auto&& part : parts) {
            total_mass += part->mass;
        }
        m_com = glm::dvec3(0, 0, 0);
        for(auto&& part : parts) {
            m_com += GetPosition(part) * (part->mass / total_mass);
        }
        return m_com;
    }

    glm::dvec3 applyGravity() {
        const double& parent_mass = m_parent->mass;
        const double G = 6.674e-11;
        glm::dvec3 gf;
        for(auto&& part : parts) {
            if(part->mass == 0) { continue; }
            const glm::dvec3& b1b2 = GetPosition(part);
            const double m1m2 = part->mass * parent_mass;
            const double invrsqr = 1.0 / glm::length2(b1b2);
            const double mag = G * m1m2 * invrsqr;
            gf = mag * sqrt(invrsqr) * -b1b2;
            ApplyCentralForce(part, gf, mag);
            if(frame->isRotFrame()) {
                // In rotating coordinates the ship additionally feels
                // Coriolis + centrifugal; without these its true inertial
                // orbit is perturbed for as long as it spends in the rotating
                // frame (see GetFictitiousAccel in frame.h).
                const glm::dvec3 a_fict = frame->GetFictitiousAccel(b1b2, GetVelocity(part));
                ApplyCentralForce(part, part->mass * a_fict);
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
        if(!onRails) {
            for(auto&& c : constraints) { Detach(c); }
            constraints.clear();
            for(auto&& part : parts) { RemoveBody(part); }
        }
        for(auto&& part : parts) { delete part; }
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
        for(size_t i = 0; i < m_thrusters.size(); i++) {
            if(m_armedThrust[i] == 0.0f) { continue; }
            ApplyCentralForceForward(m_thrusters[i], (double)m_armedThrust[i]);
        }
    }

    // The armed rotation commands -- like thrust -- are re-applied before
    // EVERY substep (h = that substep's duration); applied once per tick
    // they would act only during the first substep, cutting the delivered
    // authority to 1/n and making it worse at warp.
    void applyRotationForce(double h) {
        if(m_reaction_wheels.empty()) { return; }
        /* Manual stick: the full rated torque along each commanded body
           axis (each wheel at its rating; diagonals compose). */
        if(stick[0] != 0.0f || stick[1] != 0.0f || stick[2] != 0.0f) {
            for(size_t wi = 0; wi < m_reaction_wheels.size(); wi++) {
                Body *wheel = m_reaction_wheels[wi];
                for(int a = 0; a < 3; a++) {
                    if(stick[a] == 0.0f) { continue; }
                    ApplyTorque(wheel,
                               (double)stick[a] * m_wheelTorque[wi] * getRelAxis_(wheel, a));
                }
            }
        }
        /* Autopilot: one authority-bounded step of the slew/kill-rot law
           (h = this substep's duration, so the law re-evaluates per
           substep -- the stable form of the same law). */
        if(slew == SlewPrograde) {
            slewToward(GetVel(), h);
        }
        else if(slew == SlewRetrograde) {
            slewToward(-GetVel(), h);
        }
        else if(slew == SlewKillRot) {
            killRotStep(h);
        }
    }

    /* the largest wheel's rated torque (N m) -- the per-wheel rating for
       the HUD; the ship's TOTAL wheel authority is maxTorque() (the sum) */
    float GetWheelTorque() {
        double t = 0.0;
        for(size_t i = 0; i < m_wheelTorque.size(); i++) {
            if(m_wheelTorque[i] > t) { t = m_wheelTorque[i]; }
        }
        return (float)t;
    }

    /* disarm the armed thrust (called once per tick, like clearRotCmd,
       so a tick without the keys doesn't keep firing) */
    void clearThrust() {
        thrust_mag = 0.0f;
        for(size_t i = 0; i < m_armedThrust.size(); i++) { m_armedThrust[i] = 0.0f; }
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

        /* drop set = every part on this stage */
        std::vector<bool> drop(n, false);
        size_t dropped = 0;
        for(size_t i = 0; i < n; i++) {
            if(partStages[i] == stage) { drop[i] = true; dropped++; }
        }
        if(dropped == 0) { return 0; }   // nothing on this stage
        if(dropped == n) { return 0; }   // can't drop the whole ship

        /* the controller part's OLD index (to remap, or replace if dropped) */
        const int oldCi = controllerIndex < 0 ? (int)n - 1 : controllerIndex;

        StageSplit split = computeStageSplit(n, constraintLinks, drop);

        /* 1) Remove every constraint touching a dropped part (the stage
           interface AND the dropped set's internal welds). */
        for(size_t c : split.cutConstraints) {
            Detach(constraints[c]);   // out of the world + deleted
        }
        /* 2) Unregister + delete the dropped parts' bodies. */
        for(size_t i : split.droppedParts) {
            RemoveBody(parts[i]);     // unregister from the world
            delete parts[i];          // frees model + rigid body
        }
        /* 3) Rebuild the part-set vectors from the kept parts. */
        std::vector<Body *>          keepParts;
        std::vector<const PartDef *> keepDefs;
        std::vector<ResourceContent> keepRes;
        std::vector<int>             keepStages;
        for(size_t i : split.keptParts) {
            keepParts.push_back(parts[i]);
            keepDefs.push_back(partDefs[i]);
            keepRes.push_back(partResources[i]);
            keepStages.push_back(partStages[i]);
        }
        parts.swap(keepParts);
        partDefs.swap(keepDefs);
        partResources.swap(keepRes);
        partStages.swap(keepStages);
        /* 4) Keep only the surviving constraints; their links are already
           remapped into the new (kept-only) index space. */
        std::vector<bool> isCut(constraints.size(), false);
        for(size_t c : split.cutConstraints) { isCut[c] = true; }
        std::vector<void *> keepCons;
        std::vector<std::pair<glm::dvec3, glm::dvec3>> keepAnchors;
        for(size_t c = 0; c < constraints.size(); c++) {
            if(!isCut[c]) {
                keepCons.push_back(constraints[c]);
                keepAnchors.push_back(constraintAnchors[c]);
            }
        }
        constraints.swap(keepCons);
        constraintAnchors.swap(keepAnchors);
        constraintLinks = split.keptLinks;
        /* 5) Remap the controller (or, if it was itself dropped, fall back
           to the first surviving part). */
        if(split.newIndexOf[(size_t)oldCi] >= 0) {
            controllerIndex = (int)split.newIndexOf[(size_t)oldCi];
        } else {
            controllerIndex = 0;
        }
        controller = parts[(size_t)controllerIndex];
        NeverSleep(controller);
        /* 6) Rebuild the thruster/wheel vectors (also disarms any thrust). */
        rebuildBehavior();
        return (int)dropped;
    }

    /* renderFrame: the frame the camera view is built in. Usually this
       ship's own frame (identity transform); an idle ship that switched
       SOI while another ship was being controlled lives in a different
       frame, so transform its parts into the render frame first. */
    void Draw(const Camera* camera, Frame *renderFrame) {
        // Light direction in this frame's axes
        glm::vec3 sunlightVec = glm::vec3(TerrainBody::SunlightDir(m_parent, sun, frame));

        glm::dmat4 xform = glm::dmat4(1.0);
        if(frame != renderFrame) {
            xform = glm::translate(frame->GetPositionRelTo(renderFrame))
                  * glm::dmat4(frame->GetOrientRelTo(renderFrame));
        }

        for(auto&& part : parts) {
            // Per-part terrain shadow
            const float shadow = ComputeTerrainShadow(m_parent, frame, GetPosition(part), sun);
            part->Draw(camera, sunlightVec, shadow, xform);
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
            case Pitch:
                stick[0] = (cmd.amount >= 0) ? +1.0f : -1.0f;
                break;
            case Yaw:
                stick[1] = (cmd.amount >= 0) ? +1.0f : -1.0f;
                break;
            case Roll:
                stick[2] = (cmd.amount >= 0) ? +1.0f : -1.0f;
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
        return GetVelocity(controller);
    }

private:
    // Control implementation: applies forces/torques to the Bullet bodies
    // directly, so it is reachable only through Command() above.
    void adjustThrottle(float delta) {
        thruster_util += delta;
        if(thruster_util > 1) { thruster_util = 1; }
        if(thruster_util < 0) { thruster_util = 0; }
    }

    /* the ship's full-throttle thrust RIGHT NOW (N) = the sum of the ACTIVE
       stage's engines' full thrust (each T = (H2 + LOX flow) x ve = 2 x
       fuel_rate x ve, both propellants end up in the plume). Only the active
       stage fires, so this is the usable thrust; for a single-stage ship it
       equals the grand total. */
    float GetActiveThrust() {
        const int as = activeStage();
        double t = 0;
        for(size_t i = 0; i < m_thrusterThrust.size(); i++) {
            if(m_thrusterStage[i] == as) { t += m_thrusterThrust[i]; }
        }
        return (float)t;
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
        thrust_mag = 0.0f;
        const int as = activeStage();
        for(size_t i = 0; i < m_thrusters.size(); i++) {
            if(m_thrusterStage[i] != as) { continue; } /* not the live stage */
            const float flow =
                (float)(m_thrusterRate[i] * (double)thruster_util * step); /* kg this tick, per tank */
            if(consumeResourceMass(ResourceType::Hydrogen, flow) and
               consumeResourceMass(ResourceType::LOX,      flow))
                {
                    m_armedThrust[i] = (float)(m_thrusterThrust[i] * thruster_util);
                    thrust_mag += m_armedThrust[i];
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
        for(size_t i = 0; i < m_wheelTorque.size(); i++) { t += m_wheelTorque[i]; }
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
        for(auto&& part : parts) {
            const glm::dvec3 d = GetPosition(part) - com;
            const glm::dmat3 R = GetOrient(part);
            const glm::dvec3 il = getInertiaDiag(part);
            /* part's local inertia is diagonal (Bullet stores it that way);
               build the diagonal matrix explicitly -- GLM has no
               vec -> diagonal-mat constructor */
            const glm::dmat3 il_diag(
                il.x, 0.0, 0.0,
                0.0, il.y, 0.0,
                0.0, 0.0, il.z);
            I += R * il_diag * glm::transpose(R);
            I += part->mass * (glm::dot(d, d) * glm::dmat3(1.0) - glm::outerProduct(d, d));
        }
        return I;
    }

    /* Slew the nose (local +Z) toward `dir` within the wheel's authority:
       the target rate is the braking curve sqrt(2*alpha*E) -- the fastest
       rate from which the ship can still stop exactly at the target
       (E = the error angle) -- capped at E/(2h) so no substep can cross
       the target, and the per-substep rate change is bounded by alpha*h,
       so the command never exceeds a maxed manual stick. */
    void slewToward(glm::dvec3 dir, double h) {
        dir = glm::normalize(dir);
        if(glm::length2(dir) < 0.5) { return; } /* no velocity to align to */
        Body *wheel = m_reaction_wheels.front();
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
        const double A = alpha * h;              /* max |domega| this substep */
        const double w = glm::dot(GetAngVelocity(wheel), axis);
        const double w_des = std::min(std::sqrt(2.0 * alpha * E), E / (2.0 * h));
        double dw = w_des - w;
        if(dw > A) { dw = A; }
        if(dw < -A) { dw = -A; }
        const glm::dvec3 torque = axis * (Ieff * dw / h); /* |torque| <= maxTorque() */
        for(auto&& rw : m_reaction_wheels) {
            ApplyTorque(rw, torque / (double)m_reaction_wheels.size());
        }
    }

    /* Kill the spin within the wheel's authority: each axis' rate drops by
       min(|w|, alpha*h) per substep -- monotonic, no sign flip, never more
       forceful than a maxed manual stick. */
    void killRotStep(double h) {
        Body *wheel = m_reaction_wheels.front();
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
        for(auto&& rw : m_reaction_wheels) {
            ApplyTorque(rw, torque / (double)m_reaction_wheels.size());
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

        int i = 0;
        for(auto&& part : parts) {
            const char *name = partDefs[i]->name.c_str();
            i++;

            glm::dvec3 oldVel = GetVelocity(part);
            glm::dvec3 vel = GetVelocityRelTo(part, newFrame);
            glm::dvec3 fpos = frame->GetPositionRelTo(newFrame);
            glm::dmat3 forient = frame->GetOrientRelTo(newFrame);

            glm::dvec3 newPos = forient * GetPosition(part) + fpos;
            glm::dmat3 newOrient = forient * GetOrient(part);

            glm::dvec3 pos = GetPosition(part);
            printf("@@@ %s OLD position: %.0f %.0f %.0f\n", name, pos.x, pos.y, pos.z);
            printf("@@@ %s NEW position: %.0f %.0f %.0f\n", name, newPos.x, newPos.y, newPos.z);

            setPosRot(part, newPos, newOrient);

            pos = GetPosition(part);
            // The stored velocity is the frame-coordinate velocity, so a ship's
            // inertial velocity is R*(v + stasis(p)) + V.  GetVelocityRelTo
            // already added the OLD frame's stasis term; the NEW frame's term
            // must be SUBTRACTED, or the ship's inertial velocity is wrong by
            // 2*stasis and the orbit jumps shape at every inertial->rotational
            // switch.
            glm::dvec3 newVel = vel - newFrame->GetStasisVelocity(pos);
            printf("@@@ %s OLD velocity: %.0f %.0f %.0f\n", name, oldVel.x, oldVel.y, oldVel.z);
            printf("@@@ %s NEW velocity: %.0f %.0f %.0f\n", name, newVel.x, newVel.y, newVel.z);

            SetVelocity(part, newVel);
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
                glm::dvec3 pos = GetPosition(controller);
                printf("@@@ %s switching frame from %s to parent %s\n",
                       name.c_str(), frame->name.c_str(),
                       frame->parent->name.c_str());
                glm::dvec3 offset = frame->GetPositionRelTo(frame->parent);
                printf("@@@ Frame offset: %.0f %.0f %.0f\n", offset.x, offset.y, offset.z);
                printf("@@@@@ OLD position: %.0f %.0f %.0f\n", pos.x, pos.y, pos.z);
                moveToFrame(frame->parent);
                pos = GetPosition(controller);
                printf("@@@@@ NEW position: %.0f %.0f %.0f\n", pos.x, pos.y, pos.z);
            }
        }
        else {
            // check if we've entered a child SOI
            for(auto&& child : frame->children) {
                double dist = glm::length(GetPositionRelTo(controller, child));
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
        for(size_t i = 0; i < parts.size(); i++) {
            setPosRot(parts[i],
                      rail_pos + rail_orient * rail_rel_pos[i],
                      rail_orient * rail_rel_rot[i]);
            SetVelocity(parts[i], rail_vel);
            SetAngVelocity(parts[i], glm::dvec3(0.0));
        }
    }

    /* Rails classification: a FLYING ship (periapsis clear of the terrain
       band) coasts on its conic; a GROUNDED one (periapsis inside the
       band) can only freeze in its rotating surface frame. Anything else
       -- e.g. a suborbital descent -- is not rail-eligible. */
    bool canRail() {
        if(onRails) { return true; }
        Frame *inertial = frame->getNonRotFrame();
        glm::dvec3 p = get_center_of_mass();
        glm::dvec3 v(0.0);
        double mtot = 0.0;
        for(auto&& part : parts) {
            v += GetVelocity(part) * part->mass;
            mtot += part->mass;
        }
        v /= mtot;
        if(frame != inertial) {
            v += frame->GetStasisVelocity(p);
            v = frame->GetOrientRelTo(inertial) * v + frame->GetVelocityRelTo(inertial);
            p = frame->GetOrientRelTo(inertial) * p + frame->GetPositionRelTo(inertial);
        }
        const OrbitElements el = computeOrbitElements(p, v, inertial->body->mu);
        if(el.periapsis <= inertial->body->radius + 3000.0) {
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
        for(auto&& part : parts) {
            v += GetVelocity(part) * part->mass;
            mtot += part->mass;
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
        rail_rel_pos.resize(parts.size());
        rail_rel_rot.resize(parts.size());
        for(size_t i = 0; i < parts.size(); i++) {
            rail_rel_pos[i] = GetPosition(parts[i]) - com_frame;
            rail_rel_rot[i] = GetOrient(parts[i]);
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
        for(auto&& part : parts) { RemoveBody(part); }

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
        for(auto&& part : parts) {
            AddPhysicsBody(part);
        }
        /* GlueTogether locks the CURRENT relative pose, which is exactly
           the parked geometry, so the stored anchors reproduce the welds. */
        for(size_t c = 0; c < constraintLinks.size(); c++) {
            constraints.push_back(GlueTogether(parts[constraintLinks[c].first],
                                               parts[constraintLinks[c].second],
                                               constraintAnchors[c].first,
                                               constraintAnchors[c].second));
        }
        NeverSleep(controller);
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
   (ell_phase 0), apoapsis (1), or 90 deg of true anomaly (2). */
struct ScenarioDef {
    const char *name;
    bool on_pad;
    double alt_frac; // circular: fraction of (rot-frame SOI - radius)
    bool polar;
    int ell_phase;   // -1: circular; 0: at periapsis; 1: at apoapsis; 2: at 90 deg
    double peri_alt; // ellipse: periapsis altitude above the body radius (m)
    double apo_alt;  // ellipse: apoapsis altitude above the body radius (m)
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
