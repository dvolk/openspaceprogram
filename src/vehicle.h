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
    bool hullInWorld() const {
        return hull != nullptr && hull->btBody != nullptr
            && hull->btBody->getBroadphaseHandle() != nullptr;
    }

    btCompoundShape *compoundShape() const {
        return (hull != nullptr && hull->shape != nullptr)
             ? static_cast<btCompoundShape *>(hull->shape) : nullptr;
    }

    /* btTransform <-> (glm position, glm rotation). Always through a
       quaternion: btMatrix3x3 is row-major (m_el[i] = row i) and glm is
       column-major, so copying elements silently transposes (the trap
       physics.cpp's GetOrient / setPosRot comments warn about). */
    static btTransform toBt(const glm::dvec3 &pos, const glm::dmat3 &rot) {
        btTransform t;
        t.setIdentity();
        t.setOrigin(btVector3(pos.x, pos.y, pos.z));
        const glm::dquat q = glm::quat_cast(rot);
        t.setRotation(btQuaternion(q.x, q.y, q.z, q.w));
        return t;
    }
    static void fromBt(const btTransform &t, glm::dvec3 &pos, glm::dmat3 &rot) {
        const btVector3 &o = t.getOrigin();
        pos = glm::dvec3(o.getX(), o.getY(), o.getZ());
        btQuaternion q;
        t.getBasis().getRotation(q);
        rot = glm::mat3_cast(glm::dquat(q.w(), q.x(), q.y(), q.z()));
    }

    /* Frame S in world coordinates: the hull's COM frame mapped back through
       `principal`. */
    void frameS(glm::dvec3 &pos, glm::dmat3 &rot) const {
        glm::dvec3 bodyPos; glm::dmat3 bodyRot;
        fromBt(hull->btBody->getCenterOfMassTransform(), bodyPos, bodyRot);
        glm::dvec3 pOrigin; glm::dmat3 pBasis;
        fromBt(principal, pOrigin, pBasis);
        rot = bodyRot * glm::transpose(pBasis);
        pos = bodyPos - rot * pOrigin;
    }

    /* The COM in world coordinates, straight off the hull -- its transform
       origin IS the COM. O(1), where a mass-weighted walk over the parts is
       O(n), and it is the point Bullet actually rotates the ship about. */
    glm::dvec3 comPos() const {
        const btVector3 &o = hull->btBody->getCenterOfMassPosition();
        return glm::dvec3(o.getX(), o.getY(), o.getZ());
    }

    /* Place the whole ship: frame S at (sPos, sRot). One write to the one
       body, and every part's pose follows from its authored local pose.
       setPosRot zeroes both velocities (proceedToTransform), so callers that
       care set them after. */
    void placeShip(const glm::dvec3 &sPos, const glm::dmat3 &sRot) {
        glm::dvec3 pOrigin; glm::dmat3 pBasis;
        fromBt(principal, pOrigin, pBasis);
        setPosRot(hull, sPos + sRot * pOrigin, sRot * pBasis);
    }

    /* The same, addressed by the COM instead of by frame S's origin: the two
       differ by the COM's offset within S, which the caller should not have
       to know. */
    void placeShipAtCom(const glm::dvec3 &com, const glm::dmat3 &sRot) {
        glm::dvec3 pOrigin; glm::dmat3 pBasis;
        fromBt(principal, pOrigin, pBasis);
        placeShip(com - sRot * pOrigin, sRot);
    }

    /* (Re)build the compound + the hull from the CURRENT part list: one child
       per part -- its own collision hull, referenced not copied (the parts
       outlive the compound) -- at its authored pose in S, then re-based into
       the principal frame. Frame S and the velocity are carried across, so a
       rebuild neither teleports nor stops the ship. */
    void rebuildCompound() {
        /* `principal` moves when the masses do, and the body's transform is
           the COM transform, so leaving the transform alone across a rebuild
           would shift the ship by the COM movement and leave it spinning
           about the wrong point. */
        const bool have = (hull != nullptr && hull->btBody != nullptr);
        /* Out of the world before the delete, back in after: the rebuild
           replaces the rigid body, and a registered one cannot just be
           freed. */
        const bool wasInWorld = hullInWorld();
        if(wasInWorld) { RemoveBody(hull); }
        glm::dvec3 sPos(0.0), vCom(0.0), omega(0.0), oldOrigin(0.0);
        glm::dmat3 sRot(1.0);
        /* a rebuild makes a new rigid body, so the state that lives on the
           OLD one and is not part of the compound has to be carried over
           explicitly: the friction. An EVA kerbal's feet are set
           frictionless once, at spawn, and a burn must not undo it. (The
           never-sleep flag needs no carrying -- every ship has it, so it is
           re-asserted unconditionally below.) */
        btScalar friction = 4.0;          // RegisterObject's value
        if(have) {
            frameS(sPos, sRot);
            vCom = GetVelocity(hull);
            omega = GetAngVelocity(hull);
            glm::dmat3 ignore;
            fromBt(principal, oldOrigin, ignore);
            friction = hull->btBody->getFriction();
        }

        delete hull; hull = nullptr;
        compoundParts.clear();
        principal = btTransform::getIdentity();
        if(parts.empty()) { return; }

        btCompoundShape *inS = new btCompoundShape(true, (int)parts.size());
        std::vector<btScalar> masses(parts.size());
        btScalar total = 0;
        for(size_t i = 0; i < parts.size(); i++) {
            Part *p = parts[i];
            inS->addChildShape(toBt(p->localPos, p->localRot), p->body->shape);
            masses[i] = (btScalar)p->body->mass;
            total += masses[i];
            compoundParts.push_back(p);
        }
        /* calculatePrincipalAxisTransform btAsserts every child mass > 0 */
        if(total <= 0) { delete inS; compoundParts.clear(); return; }

        btVector3 inertiaDiag(0, 0, 0);
        inS->calculatePrincipalAxisTransform(&masses[0], principal, inertiaDiag);

        /* Re-base the children into the COM/principal frame: leaving them in
           S would give a body whose origin sits at the root part while its
           inertia is diagonal about the principal axes -- an inconsistent
           body that tumbles under any off-axis torque. (Bullet's own
           CompoundBoxes tutorial writes this product the other way round;
           the FractureDemo form below is the correct one.) */
        btCompoundShape *nc = new btCompoundShape(true, inS->getNumChildShapes());
        const btTransform toBody = principal.inverse();
        for(int i = 0; i < inS->getNumChildShapes(); i++) {
            nc->addChildShape(toBody * inS->getChildTransform(i),
                              inS->getChildShape(i));
        }
        delete inS;

        hull = new Body;
        hull->model = nullptr;      // drawn part by part, not as one mesh
        hull->shape = nc;
        hull->mass  = (double)total;
        btRigidBody::btRigidBodyConstructionInfo ci(total, nullptr, nc,
                                                    inertiaDiag);
        ci.m_friction = friction;
        hull->btBody = new btRigidBody(ci);
        NeverSleep(hull);

        /* Restore frame S under the new principal, and the COM's velocity:
           the COM moved WITHIN S, so its world velocity changed by
           omega x the shift. omega is in world axes and is unchanged. */
        glm::dvec3 pOrigin; glm::dmat3 pBasis;
        fromBt(principal, pOrigin, pBasis);
        hull->btBody->setWorldTransform(toBt(sPos + sRot * pOrigin, sRot * pBasis));
        if(have) {
            SetVelocity(hull, vCom + glm::cross(omega, sRot * (pOrigin - oldOrigin)));
            SetAngVelocity(hull, omega);
        }
        if(wasInWorld) { AddPhysicsBody(hull); }

        checkCompoundInvariants();
    }

    /* The centre of mass of the CURRENT part masses, in frame S -- i.e. what
       principal.getOrigin() would be after a rebuild. One pass over the
       parts and no allocation, so it is cheap enough to ask every tick. */
    glm::dvec3 compoundCom() const {
        double total = 0.0;
        glm::dvec3 com(0.0);
        for(size_t i = 0; i < parts.size(); i++) {
            const double m = parts[i]->body->mass;
            total += m;
            com += m * parts[i]->localPos;
        }
        return (total > 0.0) ? com / total : com;
    }

    /* The true mass COM minus the hull's transform origin, in world axes.
       The two coincide right after a rebuild, but a burn shifts the true COM
       (fuel leaves the tanks) while the origin stays put until the next
       refreshCompound threshold trips -- so this is generally nonzero during
       a burn. Bullet rotates the hull about its transform origin, so any net
       force acting through that offset origin adds a spurious
       (comOffset x F) torque the ship's true COM does not feel; the force
       laws subtract it (see applyGravity / applyThrustForce). */
    glm::dvec3 comOffset() const {
        if(hull == nullptr || parts.empty()) { return glm::dvec3(0.0); }
        glm::dvec3 sPos; glm::dmat3 sRot;
        frameS(sPos, sRot);
        return sPos + sRot * compoundCom() - comPos();
    }

    /* Rebuild once the mass distribution has moved enough to matter. A burn
       shifts the COM and the total mass continuously, and a rebuild walks
       every part's hull inertia, so one per tank draw per tick would be pure
       waste -- but never rebuilding is wrong twice over: the children are
       re-based through principal, so a stale COM displaces every hull that
       picking and collision read, and a stale total mass is a stale
       acceleration. Called once per ship per tick, so one call site covers
       every source of a mass change (a burn, crew aboard, crew out). */
    void refreshCompound() {
        if(hull == nullptr) { rebuildCompound(); return; }
        glm::dvec3 pOrigin; glm::dmat3 pBasis;
        fromBt(principal, pOrigin, pBasis);
        const double total = (double)getMass();
        if(glm::length(compoundCom() - pOrigin) > kComRebuildTol
           || std::fabs(total - hull->mass) > kMassRebuildFrac * hull->mass) {
            rebuildCompound();
        }
    }
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
    void checkCompoundInvariants() const {
        btCompoundShape *cs = compoundShape();
        if(cs == nullptr) { return; }
        double total = 0.0, extent = 0.0;
        glm::dvec3 com(0.0);
        for(size_t i = 0; i < parts.size(); i++) {
            const double m = parts[i]->body->mass;
            total += m;
            com += m * parts[i]->localPos;
            extent = std::max(extent, glm::length(parts[i]->localPos));
        }
        if(total <= 0.0) { return; }
        com /= total;

        /* the analytic tensor about the authored COM, in S axes */
        glm::dmat3 want(0.0);
        for(size_t i = 0; i < parts.size(); i++) {
            Part *p = parts[i];
            const glm::dvec3 il = getInertiaDiag(p->body);
            const glm::dmat3 d(il.x, 0.0, 0.0,
                               0.0, il.y, 0.0,
                               0.0, 0.0, il.z);
            want += p->localRot * d * glm::transpose(p->localRot);
            const glm::dvec3 o = p->localPos - com;
            want += p->body->mass
                  * (glm::dot(o, o) * glm::dmat3(1.0) - glm::outerProduct(o, o));
        }

        /* Bullet's, rotated out of the principal frame back into S */
        glm::dvec3 pOrigin; glm::dmat3 pBasis;
        fromBt(principal, pOrigin, pBasis);
        const btVector3 &bi = hull->btBody->getLocalInertia();
        const glm::dmat3 diag(bi.getX(), 0.0, 0.0,
                              0.0, bi.getY(), 0.0,
                              0.0, 0.0, bi.getZ());
        const glm::dmat3 got = pBasis * diag * glm::transpose(pBasis);

        /* btMatrix3x3::diagonalize is a Jacobi iteration that stops once
           every off-diagonal is under 1e-5 x the diagonal trace, so the
           eigenvalues it hands back carry that much of the tensor's residue
           -- the tolerance cannot be tighter than that. The COM and the
           child poses are plain arithmetic, so they are held to rounding. */
        double trace = 0.0;
        for(int c = 0; c < 3; c++) { trace += std::fabs(want[c][c]); }
        const double iTol = 1e-5 * std::max(1.0, trace);
        double iErr = 0.0;
        for(int c = 0; c < 3; c++) {
            for(int r = 0; r < 3; r++) {
                iErr = std::max(iErr, std::fabs(got[c][r] - want[c][r]));
            }
        }
        const double comErr = glm::length(pOrigin - com);
        const double comTol = 1e-9 * std::max(1.0, glm::length(com));

        double childPosErr = 0.0, childRotErr = 0.0;
        if((size_t)cs->getNumChildShapes() != compoundParts.size()) {
            childPosErr = 1e30;   // the mapping is broken; report it as such
        } else {
            for(size_t i = 0; i < compoundParts.size(); i++) {
                const Part *p = compoundParts[i];
                glm::dvec3 cp; glm::dmat3 cr;
                fromBt(principal * cs->getChildTransform((int)i), cp, cr);
                childPosErr = std::max(childPosErr, glm::length(cp - p->localPos));
                for(int c = 0; c < 3; c++) {
                    for(int r = 0; r < 3; r++) {
                        childRotErr = std::max(childRotErr,
                            std::fabs(cr[c][r] - p->localRot[c][r]));
                    }
                }
            }
        }
        const double childTol = 1e-9 * std::max(1.0, extent);

        if(iErr > iTol || comErr > comTol
           || childPosErr > childTol || childRotErr > 1e-12) {
            printf("[compound] '%s': does NOT reproduce the part assembly "
                   "(com err %.4g m, tol %.4g; inertia err %.4g kg m^2, tol "
                   "%.4g; child pos err %.4g m, tol %.4g; child rot err "
                   "%.4g)\n",
                   name.c_str(), comErr, comTol, iErr, iTol,
                   childPosErr, childTol, childRotErr);
            fflush(stdout);
            assert(false && "compound must reproduce the part assembly");
        }
    }

    /* The part frame S is anchored to: the one with no parent edge. That is
       build_ship's setRoot, and staging never drops it (a decoupler takes
       its child-side subtree, and dropping the root would drop the whole
       ship, which separateStage refuses). */
    Part *rootPart() const {
        for(size_t i = 0; i < parts.size(); i++) {
            if(parts[i]->parent == nullptr) { return parts[i]; }
        }
        return parts.empty() ? nullptr : parts[0];
    }

    /* --- part state accessors -------------------------------------------

       The one route to a part's pose, axes and velocity, and the only place
       that knows a part has no rigid body of its own. The pose is derived
       from the hull's transform through `principal` and the part's authored
       local pose; the velocity from the hull's -- the COM velocity plus
       omega x the offset, omega being the whole ship's, since a rigid body
       has one. Forces, mass and the render model still go through the Body. */
    void partWorldPose(const Part *p, glm::dvec3 &pos, glm::dmat3 &rot) const {
        glm::dvec3 sPos; glm::dmat3 sRot;
        frameS(sPos, sRot);
        pos = sPos + sRot * p->localPos;
        rot = sRot * p->localRot;
    }
    glm::dvec3 partPos(const Part *p) const {
        glm::dvec3 pos; glm::dmat3 rot;
        partWorldPose(p, pos, rot);
        return pos;
    }
    glm::dmat3 partRot(const Part *p) const {
        glm::dvec3 pos; glm::dmat3 rot;
        partWorldPose(p, pos, rot);
        return rot;
    }
    /* the part's local axis n (0 = right, 1 = up, 2 = nose) in world axes */
    glm::dvec3 partAxis(const Part *p, int n) const { return partRot(p)[n]; }
    glm::dvec3 partVel(const Part *p) const {
        return GetVelocity(hull)
             + glm::cross(GetAngVelocity(hull), partPos(p) - comPos());
    }
    glm::dvec3 partAngVel(const Part *p) const { return GetAngVelocity(hull); }

    /* --compound-check: the ship's single-body state, per ship. Rebuilds the
       compound, which re-asserts that it still reproduces the part assembly
       it came from -- the invariant the whole representation rests on -- then
       reports the body the game is actually simulating: mass, COM, speed,
       spin, and the principal inertia diagonal -- the denominator of a
       reaction wheel's authority and of the autopilot slew law, so the number
       worth watching when a ship stops turning the way it used to. There is
       no derived-vs-live pose error to report any more: a part has no rigid
       body of its own to disagree with, which is the point of the change. */
    void compoundCheck(double time) {
        rebuildCompound();
        if(hull == nullptr) { return; }
        const glm::dvec3 com = comPos();
        const btVector3 &I = hull->btBody->getLocalInertia();
        printf("[compound] t=%.2fs ship=%s parts=%zu mass=%.1f kg "
               "com=[%.1f %.1f %.1f] |v|=%.3f m/s |w|=%.5f rad/s "
               "I=[%.4g %.4g %.4g] kg m^2\n",
               time, name.c_str(), parts.size(), hull->mass,
               com.x, com.y, com.z,
               glm::length(GetVelocity(hull)),
               glm::length(GetAngVelocity(hull)),
               I.getX(), I.getY(), I.getZ());
        fflush(stdout);
    }

    /* Fuel links (see PartDef.fuel_link): one-way fuel connections between
       fuel groups. `from` -> `to` means fuel flows from `from`'s group to
       `to`'s group (the engine in `to`'s group can draw fuel from `from`'s
       group). Virtual -- no physics. Populated in build_ship (from the
       def's fuel_link parts), dropped in separateStage (when either
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
    /* The autopilot mode the Autopilot window has engaged (its toggle
       buttons). Persistent across ticks, unlike `slew` (cleared each tick):
       the logic tick re-applies it after clearRotCmd(), so the ship keeps
       slewing toward the target and holding until the mode is toggled off. */
    SlewMode slewRequest = SlewNone;
    void setSlewRequest(SlewMode m) { slewRequest = m; }

    void setRoot(Part *part) {
        part->parent   = nullptr;
        part->localPos = glm::dvec3(0.0);
        part->localRot = glm::dmat3(1.0);
        parts.push_back(part);
    }

    /* Hang `part` off the part at `parentIdx` and record its authored pose in
       the ship-local frame S (Part::localPos/localRot; S is the root's frame,
       so these are pure geometry -- see part.h). `parent` is the topology edge
       the staging + fuel-group walks use. There is nothing to weld: the ship
       is ONE rigid body, and the local pose is what puts this part's hull at
       the right place inside it. */
    void attach(Part *part, size_t parentIdx,
                const glm::dvec3 &localPos, const glm::dmat3 &localRot) {
        part->parent   = parts[parentIdx];
        part->localPos = localPos;
        part->localRot = localRot;
        parts.push_back(part);
    }

    /* The three convenience attach modes (used by the --radial-test ship
       builder; build_ship goes through attachPose + attach directly). Each
       derives the child's ship-local pose from the parent's. */

    void attachDown(Part *part) {
        /* weld at the part faces: parent bottom (-h/2) to child top (+h/2);
           generalizes the old hardcoded +-1 m (2 m parts). The parent is the
           last part pushed. The child sits straight below it, axes unchanged,
           so the child's +hC/2 anchor lands on the parent's -hP/2 anchor. */
        const Part *pp = parts.back();
        const PartDef *parent = pp->def;
        const double dz = -(parent->height + part->def->height) / 2.0;
        attach(part, parts.size() - 1,
               pp->localPos + pp->localRot * glm::dvec3(0.0, 0.0, dz),
               pp->localRot);
    }

    void attachRadial(Part *part) {
        /* hang the part off the parent's SIDE: the part's local +Z axis is
           rotated to the parent's local +X, so the part's bottom face
           (local -h/2) touches the parent's side at +radius. */
        const Part *pp = parts.back();
        const PartDef *parent = pp->def;
        /* columns are the images of X, Y, Z: takes the child's +Z onto the
           parent's +X (the same rotZtoX the --radial-test call site uses). */
        const glm::dmat3 rotZtoX(glm::dvec3(0, 0, -1),
                                 glm::dvec3(0, 1, 0),
                                 glm::dvec3(1, 0, 0));
        const glm::dvec3 off(parent->radius + part->def->height / 2.0, 0.0, 0.0);
        attach(part, parts.size() - 1,
               pp->localPos + pp->localRot * off,
               pp->localRot * rotZtoX);
    }

    void attachSide(Part *part) {
        /* hang the part off the parent's SIDE with PARALLEL axes: the part
           keeps the parent's local +Z axis, sits along the parent's local
           +X, and its cylindrical surface touches the parent's at +radius.
           Unlike attachRadial the child is NOT rotated, so this is the "side
           by side, parallel axes" case. */
        const Part *pp = parts.back();
        const PartDef *parent = pp->def;
        const glm::dvec3 off(parent->radius + part->def->radius, 0.0, 0.0);
        attach(part, parts.size() - 1,
               pp->localPos + pp->localRot * off,
               pp->localRot);
    }

    void init() {
        if(parts.empty()) { return; }
        /* propellant reservoirs: seed each tank part's resources so the
           thrusters can draw from them (they shed mass as they burn). Only
           done at construction -- separateStage() and extractSubtreeAsShip()
           must NOT re-seed (a stage that has been burning keeps what it has
           left). */
        for(size_t i = 0; i < parts.size(); i++) {
            Part *p = parts[i];
            if(!p->isTank()) { continue; }
            for(int r = 0; r < (int)ResourceType::Num; r++) {
                p->resources.capacity[r] = p->def->capacity[r];
                p->resources.current[r]  = p->def->capacity[r];
            }
        }
        finalize();
    }

    /* init() minus the tank re-seed: the bookkeeping that finalizes a ship
       whose part list is already final -- the controller fallback, the stage
       readout, the fuel groups and the compound. init() calls it after
       seeding; extractSubtreeAsShip() calls it directly (the extracted
       parts carry their current tank contents). */
    void finalize() {
        if(parts.empty()) { return; }
        if(controller == nullptr) { controller = parts[0]; }
        /* stage bookkeeping: totalStages_ = the highest stage number (the
           "stage X of N" readout); activeStage_ starts at the LOWEST stage
           on the ship, so the lowest engines fire at t=0 (a ship whose first
           engine is stage 2 still lifts off). Both computed once here. */
        totalStages_ = 1;
        int lowest = parts[0]->stage;
        for(size_t i = 1; i < parts.size(); i++) {
            if(parts[i]->stage > totalStages_) { totalStages_ = parts[i]->stage; }
            if(parts[i]->stage < lowest) { lowest = parts[i]->stage; }
        }
        activeStage_ = lowest;
        /* fuel groups: an engine draws from the tanks it's connected to
           (its fuel group), not by stage -- the weld links are known now. */
        buildFuelGroups();
        /* the ship's single rigid body: the part list is complete, so the
           compound can be built, and its mass properties asserted against
           the assembly it came from. ONE body per ship, not one per part --
           and nothing to weld, because a rigid body has no internal degrees
           of freedom to constrain. Registering it is enterWorld()'s job, so
           that a headless caller (the unit tests) can build a ship with no
           physics world in existence. */
        rebuildCompound();
    }

    /* Put the ship's one rigid body into the physics world. Kept apart from
       init(), which builds it, for the reason above. */
    void enterWorld() {
        if(hull != nullptr && !hullInWorld()) { AddPhysicsBody(hull); }
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

    /* The crew's capsule slot is an index into the ship's part list, and a
       merge (absorbShip) or a split (extractSubtreeAsShip) reindexes the
       list. A crew member whose capsule moved with `dest` applies the
       old->new index map to its slot and returns true (so the caller moves
       it into dest's crew); one whose capsule stayed returns false. Only a
       Kerbal (eva.h) has a slot to reindex, so the base is a no-op. */
    virtual bool crewRebase(Vehicle *dest, const std::map<size_t, size_t> &reindex) {
        return false;
    }

    /* Assign each part a fuel-group id (Part::fuelGroup). A fuel group is a
       connected component of the part tree across the parts that CONDUCT
       fuel; a part with PartDef::fuel_barrier (a decoupler) is a WALL that
       splits the groups, so an engine never draws fuel from across it.
       Barrier parts keep fuelGroup = -1 (they are in no group). This is the
       base undirected grouping; a fuel link, when added, will bridge groups
       one-way inside fuelPool(), leaving this the same. Recompute after
       separateStage() -- the tree shrinks when parts drop. */
    void buildFuelGroups() {
        for(Part *p : parts) { p->fuelGroup = -1; }
        /* undirected adjacency over the part tree (Part::parent; each
           non-root part has exactly one parent edge). */
        std::map<Part *, std::vector<Part *>> adj;
        for(Part *p : parts) {
            if(p->parent == nullptr) { continue; }
            adj[p->parent].push_back(p);
            adj[p].push_back(p->parent);
        }
        int next = 0;
        for(Part *p : parts) {
            if(p->isFuelBarrier()) { continue; }   /* a wall: stays -1 */
            if(p->fuelGroup != -1) { continue; }   /* already grouped */
            int g = next++;
            std::vector<Part *> stack;
            stack.push_back(p);
            p->fuelGroup = g;
            while(!stack.empty()) {
                Part *q = stack.back(); stack.pop_back();
                auto it = adj.find(q);
                if(it == adj.end()) { continue; }
                for(size_t i = 0; i < it->second.size(); i++) {
                    Part *r = it->second[i];
                    if(r->isFuelBarrier()) { continue; }   /* don't cross a wall */
                    if(r->fuelGroup != -1) { continue; }   /* already grouped */
                    r->fuelGroup = g;
                    stack.push_back(r);
                }
            }
        }
    }

    /* The tanks an engine may draw fuel from: the tanks in its fuel group
       (buildFuelGroups) -- its connected neighbours, never across a fuel
       barrier (a decoupler). This is the single place a fuel link will later
       change (to "tanks reachable via directed fuel edges"), so the drain
       logic below stays put. */
    std::vector<Part *> fuelPool(Part *engine) const {
        std::vector<Part *> pool;
        const int g = engine->fuelGroup;
        for(size_t i = 0; i < parts.size(); i++) {
            Part *p = parts[i];
            if(!p->isTank()) { continue; }
            if(p->fuelGroup != g) { continue; }
            pool.push_back(p);
        }
        return pool;
    }

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
    std::vector<std::vector<int> > fuelDrainLayers(Part *engine) const {
        const int g = engine->fuelGroup;
        std::vector<std::vector<int> > layers;
        if(g < 0) { return layers; }
        /* reverse adjacency over fuel groups: rev[Y] = { X : X feeds Y }. */
        std::map<int, std::vector<int> > rev;
        for(size_t k = 0; k < fuelLinks.size(); k++) {
            int a = fuelLinks[k].from->fuelGroup;
            int b = fuelLinks[k].to->fuelGroup;
            if(a < 0 || b < 0 || a == b) { continue; }
            rev[b].push_back(a);
        }
        /* BFS from G in the reverse graph; dist[X] = hops from X to G. */
        std::map<int, int> dist;
        dist[g] = 0;
        std::vector<int> queue;
        queue.push_back(g);
        for(size_t qi = 0; qi < queue.size(); qi++) {
            int u = queue[qi];
            std::map<int, std::vector<int> >::const_iterator it = rev.find(u);
            if(it == rev.end()) { continue; }
            for(size_t i = 0; i < it->second.size(); i++) {
                int v = it->second[i];
                if(dist.count(v)) { continue; }
                dist[v] = dist[u] + 1;
                queue.push_back(v);
            }
        }
        /* bucket the groups by distance, then take the buckets in
           descending order (std::map is ascending, so walk it backwards).
           Within a bucket the group ids are ascending (dist is keyed by
           group id), so the order is deterministic. */
        std::map<int, std::vector<int> > byDist;
        for(std::map<int, int>::const_iterator it = dist.begin(); it != dist.end(); ++it) {
            byDist[it->second].push_back(it->first);
        }
        for(std::map<int, std::vector<int> >::const_reverse_iterator it = byDist.rbegin();
            it != byDist.rend(); ++it) {
            layers.push_back(it->second);
        }
        return layers;
    }

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
    bool consumeResourceMass(enum ResourceType type, float amt /* kg */, Part *engine) {
        const std::vector<std::vector<int> > layers = fuelDrainLayers(engine);
        if(layers.empty()) { return false; }
        /* Total fuel across all source groups (every layer). */
        float total = 0;
        for(size_t li = 0; li < layers.size(); li++) {
            for(size_t gi = 0; gi < layers[li].size(); gi++) {
                for(size_t i = 0; i < parts.size(); i++) {
                    Part *p = parts[i];
                    if(!p->isTank()) { continue; }
                    if(p->fuelGroup != layers[li][gi]) { continue; }
                    total += p->resources.current[(int)type];
                }
            }
        }
        if(total < amt) { return false; }
        /* Drain layer by layer (furthest first), pro-rata across the
           layer's tanks. */
        float remaining = amt;
        for(size_t li = 0; li < layers.size() && remaining > 0.0f; li++) {
            std::vector<Part *> tanks;
            float layerTotal = 0;
            for(size_t gi = 0; gi < layers[li].size(); gi++) {
                const int grp = layers[li][gi];
                for(size_t i = 0; i < parts.size(); i++) {
                    Part *p = parts[i];
                    if(!p->isTank()) { continue; }
                    if(p->fuelGroup != grp) { continue; }
                    float have = p->resources.current[(int)type];
                    if(have <= 0.0f) { continue; }
                    tanks.push_back(p);
                    layerTotal += have;
                }
            }
            if(layerTotal <= 0.0f) { continue; }
            float take = remaining < layerTotal ? remaining : layerTotal;
            for(size_t ti = 0; ti < tanks.size(); ti++) {
                Part *p = tanks[ti];
                float have = p->resources.current[(int)type];
                float share = take * have / layerTotal;
                if(share > have) { share = have; }
                p->resources.current[(int)type] = have - share;
                /* No SetMass: a part has no rigid body. The ship's mass
                   properties follow from the parts, and refreshCompound()
                   (once per tick) rebuilds them once the drift matters. */
                p->body->mass -= (double)share;
            }
            remaining -= take;
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
    void powerTick(double h) {
        double totalGen = 0.0, constantDraw = 0.0;
        double ecCharge = 0.0, ecCapacity = 0.0;
        for(size_t i = 0; i < parts.size(); i++) {
            Part *p = parts[i];
            totalGen += p->powerGen();
            constantDraw += p->powerDrawConstant();
            if(p->isBattery()) {
                ecCharge += p->resources.current[(int)ResourceType::EC];
                ecCapacity += p->resources.capacity[(int)ResourceType::EC];
            }
        }
        // gate: no EC system -> ungated (wheels work as before). Otherwise
        // the wheels need power left over for them after life support --
        // excess generation or stored charge.
        const bool hasEC = (ecCapacity > 0.0) || (totalGen > 0.0) || (constantDraw > 0.0);
        powered_ = hasEC ? ((totalGen > constantDraw) || (ecCharge > 0.0)) : true;
        // active draw: the wheels draw only while they are actually working
        // (powered AND commanding attitude) -- the same condition
        // applyRotationForce uses to apply torque.
        double activeDraw = 0.0;
        const bool wheelsActive = powered_
            && ((stick[0] != 0.0f || stick[1] != 0.0f || stick[2] != 0.0f)
                || slew != SlewNone);
        if(wheelsActive) {
            for(size_t i = 0; i < parts.size(); i++) {
                if(parts[i]->isWheel()) { activeDraw += parts[i]->powerDraw(); }
            }
        }
        // balance: generation charges, the draws drain; clamp to the pool.
        const double netWh = (totalGen - constantDraw - activeDraw) * h / 3600.0;
        if(netWh < 0.0) { drainEC(-netWh); }
        else if(netWh > 0.0) { chargeEC(netWh); }
    }

    /* Drain up to `wh` of EC from the pool, pro-rata across the batteries
       by their current charge. Clamped to what is stored (never below 0).
       No mass change: EC is energy, not a substance. */
    void drainEC(double wh) {
        if(wh <= 0.0) { return; }
        double total = 0.0;
        for(Part *p : parts) {
            if(p->isBattery()) { total += p->resources.current[(int)ResourceType::EC]; }
        }
        if(total <= 0.0) { return; }
        double take = (wh < total) ? wh : total;
        for(Part *p : parts) {
            if(!p->isBattery()) { continue; }
            float have = p->resources.current[(int)ResourceType::EC];
            if(have <= 0.0f) { continue; }
            float share = (float)(take * have / total);
            if(share > have) { share = have; }
            p->resources.current[(int)ResourceType::EC] = have - share;
        }
    }

    /* Charge the pool by up to `wh`, pro-rata across the batteries by their
       free capacity. Clamped to the capacity (never above it). No mass
       change. */
    void chargeEC(double wh) {
        if(wh <= 0.0) { return; }
        double freeCap = 0.0;
        for(Part *p : parts) {
            if(p->isBattery()) {
                freeCap += p->resources.capacity[(int)ResourceType::EC]
                         - p->resources.current[(int)ResourceType::EC];
            }
        }
        if(freeCap <= 0.0) { return; }
        double add = (wh < freeCap) ? wh : freeCap;
        for(Part *p : parts) {
            if(!p->isBattery()) { continue; }
            float cap = p->resources.capacity[(int)ResourceType::EC];
            float have = p->resources.current[(int)ResourceType::EC];
            float free = cap - have;
            if(free <= 0.0f) { continue; }
            float share = (float)(add * free / freeCap);
            if(share > free) { share = free; }
            p->resources.current[(int)ResourceType::EC] = have + share;
        }
    }

    /* Total EC charge / capacity across the pool (the HUD + --power-log). */
    void getPower(double *gen, double *constDraw, double *charge, double *capacity) {
        double g = 0.0, c = 0.0, q = 0.0, cap = 0.0;
        for(Part *p : parts) {
            g += p->powerGen();
            c += p->powerDrawConstant();
            if(p->isBattery()) {
                q += p->resources.current[(int)ResourceType::EC];
                cap += p->resources.capacity[(int)ResourceType::EC];
            }
        }
        if(gen) { *gen = g; }
        if(constDraw) { *constDraw = c; }
        if(charge) { *charge = q; }
        if(capacity) { *capacity = cap; }
    }

    /* --power-log: the ship's power balance + pool + gate, one line per
       sample (the "is the ship losing power?" instrument). */
    void power_log(double time) {
        double gen, constDraw, charge, capacity;
        getPower(&gen, &constDraw, &charge, &capacity);
        printf("[powerlog] t=%.1fs ship=\"%s\" powered=%d gen=%.1fW "
               "const_draw=%.1fW charge=%.1fWh capacity=%.1fWh\n",
               time, name.c_str(), (int)powered_, gen, constDraw, charge, capacity);
        fflush(stdout);
    }

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
    int activeStage() { return activeStage_; }
    /* Advance to the next stage (clamped at the last one). Called once per
       stage press, after the current stage's decouplers have fired. */
    void advanceStage() { if(activeStage_ < totalStages_) { activeStage_++; } }

    /* Total number of stages on the ship (the highest stage number at build
       time), for the "stage X of N" readout. */
    int numStages() { return totalStages_; }

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
        SetVelocity(hull, vel);
    }

    /* A part's mass changed OUTSIDE a burn (crew aboard, crew out). The burn
       path is picked up by refreshCompound's per-tick threshold; these are
       one-off and large, so rebuild now rather than wait for it. */
    void addPartMass(Part *p, double delta) {
        p->body->mass += delta;
        rebuildCompound();
    }

    /* The COM is the hull's transform origin -- a rigid body's transform IS
       its centre-of-mass transform -- so this is O(1), and it is exactly the
       point Bullet rotates the ship about. */
    const glm::dvec3& get_center_of_mass(void) {
        m_com = comPos();
        return m_com;
    }

    glm::dvec3 applyGravity() {
        const double& parent_mass = m_parent->mass;
        const double G = 6.674e-11;
        const glm::dvec3 com = comPos();
        glm::dvec3 gf(0.0);
        glm::dvec3 ff_total(0.0);
        for(Part *p : parts) {
            if(p->body->mass == 0) { continue; }
            const glm::dvec3 b1b2 = partPos(p);
            const double m1m2 = p->body->mass * parent_mass;
            const double invrsqr = 1.0 / glm::length2(b1b2);
            const double mag = G * m1m2 * invrsqr;
            const glm::dvec3 f = mag * sqrt(invrsqr) * -b1b2;
            /* AT the part, not at the COM: ApplyForce's rel_pos is what
               delivers the differential (tidal) torque, the one legitimate
               external torque on a rigid ship. Summing the forces and applying
               them at the COM would drop it. */
            ApplyForce(hull, b1b2 - com, f);
            gf += f;
            if(frame->isRotFrame()) {
                // In rotating coordinates the ship additionally feels
                // Coriolis + centrifugal; without these its true inertial
                // orbit is perturbed for as long as it spends in the rotating
                // frame (see GetFictitiousAccel in frame.h).
                const glm::dvec3 a_fict = frame->GetFictitiousAccel(b1b2, partVel(p));
                const glm::dvec3 ff = p->body->mass * a_fict;
                ApplyForce(hull, b1b2 - com, ff);
                ff_total += ff;
            }
        }
        /* The per-part levers above are referenced to the hull's transform
           origin, which lags the true COM during a burn; a net force through
           that offset adds a spurious (comOffset x F) torque the ship's true
           COM does not feel, so cancel it. (The legitimate tidal torque is
           about the true COM and is unaffected.) */
        const glm::dvec3 dcom = comOffset();
        if(glm::length2(dcom) > 0.0) {
            ApplyTorque(hull, -glm::cross(dcom, gf + ff_total));
        }
        return gf;
    }

public:
    Vehicle() { }
    /* Tear the ship down in a safe order: unregister the one rigid body from
       the world, delete it (which frees the compound it carries), then delete
       the parts (which free their models and their collision hulls -- the
       compound only referenced those). goOnRails() already unregistered, so
       for a railed ship only the deletes remain. The onRails guard is
       LOAD-BEARING: Bullet's removeCollisionObject is not idempotent -- it
       reads the object's world-array index (which remove never resets to -1),
       so a second remove on an absent body can evict the WRONG collision
       object. */
    virtual ~Vehicle() {
        // The crew aboard (Kerbals, eva.h) are owned by this ship: delete
        // them before their parts would outlive their `aboard` target.
        for(auto&& k : crew) { delete k; }
        crew.clear();
        if(hullInWorld()) { RemoveBody(hull); }
        delete hull; hull = nullptr;
        compoundParts.clear();
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
        const glm::dvec3 com = comPos();
        glm::dvec3 ftotal(0.0);
        for(Part *p : parts) {
            if(!p->isThruster()) { continue; }
            if(p->armedThrust == 0.0f) { continue; }
            /* Along the engine's own +Z, applied AT the engine: an off-axis or
               tilted engine torques the ship directly, which is what the weld
               used to have to transmit. */
            const glm::dvec3 ft = partAxis(p, 2) * (double)p->armedThrust;
            ApplyForce(hull, partPos(p) - com, ft);
            ftotal += ft;
        }
        if(glm::length2(ftotal) < 1e-24) { return; }
        /* Same spurious-torque cancellation as applyGravity: the thrust lever
           is referenced to the hull origin, which lags the true COM during a
           burn, so subtract the (comOffset x F) term it introduces. */
        const glm::dvec3 dcom = comOffset();
        if(glm::length2(dcom) > 0.0) {
            ApplyTorque(hull, -glm::cross(dcom, ftotal));
        }
    }

    /* The armed control forces, re-applied before EVERY substep (Bullet
       clears forces per stepSimulation). Ships deliver thrust + rotation +
       RCS translation; the EVA kerbal overrides with its own laws
       (src/eva.h) and does not call this. */
    virtual void applyControlForces(double h) {
        applyThrustForce();
        applyRotationForce(h);
        applyRcsForce(h);
    }

    /* the first reaction-wheel PART (nullptr if the ship has none): the
       stick / slew / kill-rot laws all use it as the ship's attitude
       reference. Replaces the old m_reaction_wheels.front(). A Part, not a
       Body, so the reads below go through the part accessors like every
       other consumer of a part's state. */
    Part *firstWheel() {
        for(Part *p : parts) { if(p->isWheel()) { return p; } }
        return nullptr;
    }

    // The armed rotation commands -- like thrust -- are re-applied before
    // EVERY substep (h = that substep's duration); applied once per tick
    // they would act only during the first substep, cutting the delivered
    // authority to 1/n and making it worse at warp.
    void applyRotationForce(double h) {
        if(firstWheel() == nullptr) { return; }
        /* Power gate: the reaction wheels are electric -- with no power the
           ship is uncontrolled (no manual stick AND no autopilot slew).
           powered_ is set by powerTick this substep (a ship with no EC
           system is ungated, so this is a no-op for them). */
        if(!powered_) { return; }
        /* Manual stick: standard aviation mapping, body-relative.
           Pitch (W/S) about the ship's right axis, yaw (A/D) about its
           up axis, roll (Q/E) about the nose. The camera tracks the
           ship's attitude, so these read consistently on screen
           regardless of the ship's world orientation. Each wheel gets
           its rated torque along the combined axis; diagonals (W+A)
           compose as a vector sum. */
        if(stick[0] != 0.0f || stick[1] != 0.0f || stick[2] != 0.0f) {
            Part *rw0 = firstWheel();
            const glm::dvec3 pitchAxis = -partAxis(rw0, 0);  // right (W/S)
            const glm::dvec3 yawAxis   = -partAxis(rw0, 1);  // up    (A/D)
            const glm::dvec3 rollAxis  =  partAxis(rw0, 2);  // nose  (Q/E)
            const glm::dvec3 worldAxis =
                (double)stick[0] * rollAxis
                + (double)stick[1] * pitchAxis
                + (double)stick[2] * yawAxis;
            /* Every wheel turns the same rigid body along the same axis, so
               their torques simply sum -- and maxTorque() is that sum. */
            ApplyTorque(hull, maxTorque() * worldAxis);
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
    void clearRcs() { rcsDir = glm::dvec3(0.0); rcsFiring = false; }
    /* The armed RCS direction in WORLD axes (unit; (0,0,0) when disarmed):
       rcsDir's ship-body components mapped through the root part's axes.
       The root's local frame IS the ship frame S, and its nose is the same
       axis the attitude law slews (att_log). Resolved at the point of use,
       so the direction tracks the ship's live attitude, substep by
       substep -- not a camera basis sampled once per tick. */
    glm::dvec3 rcsWorldDir() const {
        if(glm::length2(rcsDir) < 1e-12) { return glm::dvec3(0.0); }
        const Part *ref = rootPart();
        if(ref == nullptr) { return glm::dvec3(0.0); }
        const glm::dvec3 d = rcsDir.x * partAxis(ref, 0)
                           + rcsDir.y * partAxis(ref, 1)
                           + rcsDir.z * partAxis(ref, 2);
        return glm::normalize(d);
    }
    Part *firstRcsPart() {
        for(Part *p : parts) { if(p->isRcs()) { return p; } }
        return nullptr;
    }
    double maxRcsThrust() {
        double t = 0.0;
        for(Part *p : parts) { if(p->isRcs()) { t += p->rcsThrust(); } }
        return t;
    }
    void applyRcsForce(double h) {
        if(glm::length2(rcsDir) < 1e-12) { return; }
        Part *e = firstRcsPart();
        if(e == nullptr) { return; }
        const double F = maxRcsThrust();
        if(F <= 0.0) { return; }
        /* flow this substep (kg) = thrust / (Isp * g0) * h, the same
           monoprop Isp the EVA suit uses (src/eva.cpp kRcsIsp). */
        const double flow = (F / (kRcsIsp * 9.81)) * h;
        if(consumeResourceMass(ResourceType::Hydrazine, (float)flow, e)) {
            ApplyCentralForce(hull, F * rcsWorldDir());
            rcsFiring = true;
        }
    }
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
    void slew_log(double time) {
        if(slew == SlewNone) { return; }
        Part *wheel = firstWheel();
        if(wheel == nullptr) { return; }
        const glm::dvec3 facing = partAxis(wheel, 2);
        if(slew == SlewKillRot) {
            const glm::dvec3 w = partAngVel(wheel);
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
        const glm::dvec3 w = partAngVel(wheel);
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
       its angular velocity (world coords), for the attitude-physics e2e test.
       One rigid body, so there is exactly one of each: the root part's +Z is
       frame S's, and the spin is the hull's. (A reaction wheel used to have
       to be avoided here -- under the welds it spun relative to the hull.) */
    void att_log(double time) {
        if(hull == nullptr || parts.empty()) { return; }
        const glm::dvec3 nose = partAxis(rootPart(), 2);
        const glm::dvec3 w = GetAngVelocity(hull);
        printf("[attlog] t=%.3fs nose=[%+.4f %+.4f %+.4f] "
               "w=[%+.4f %+.4f %+.4f] |w|=%.4f rad/s\n",
               time, nose.x, nose.y, nose.z,
               w.x, w.y, w.z, glm::length(w));
        fflush(stdout);
    }

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
    void tq_log(double time) {
        if(hull == nullptr || parts.empty()) { return; }
        const double G = 6.674e-11;
        const double& parent_mass = m_parent->mass;
        glm::dvec3 F(0.0);
        for(Part *p : parts) {
            if(p->body->mass == 0) { continue; }
            const glm::dvec3 b1b2 = partPos(p);
            const double r2 = glm::length2(b1b2);
            F += G * p->body->mass * parent_mass * (-b1b2) / (r2 * sqrt(r2));
            if(frame->isRotFrame()) {
                F += p->body->mass *
                     frame->GetFictitiousAccel(b1b2, partVel(p));
            }
        }
        for(Part *p : parts) {
            if(p->isThruster() && p->armedThrust != 0.0f) {
                F += partAxis(p, 2) * (double)p->armedThrust;
            }
        }
        const glm::dvec3 dcom = comOffset();
        const glm::dvec3 w = GetAngVelocity(hull);
        printf("[tqlog] t=%.3fs |dcom|=%.3e m F=[%.4e %.4e %.4e] "
               "|F|=%.4e N |dxF|=%.3e N m |w|=%.3e rad/s\n",
               time, glm::length(dcom), F.x, F.y, F.z, glm::length(F),
               glm::length(glm::cross(dcom, F)), glm::length(w));
        fflush(stdout);
    }

    /* --fuel-log: each fuel group's fuel mass (per resource), the
       per-tank breakdown, and the fuel links -- the instrument for the
       fuel-link drain-rate bug: two symmetric radial groups must show
       equal mass at every sample, so a one-before-the-other drain shows
       up as the two group lines diverging while the ship spins. */
    static const char *resourceName(int r) {
        switch((ResourceType)r) {
            case ResourceType::Hydrogen:  return "H2";
            case ResourceType::LOX:       return "LOX";
            case ResourceType::EC:        return "EC";
            case ResourceType::Oxygen:    return "O2";
            case ResourceType::Water:     return "H2O";
            case ResourceType::Food:      return "food";
            case ResourceType::Hydrazine: return "N2H4";
            case ResourceType::Num:       break;   /* count, not a resource */
        }
        return "?";
    }

    void fuel_log(double time) {
        /* distinct group ids, ascending (stable order across ticks). */
        std::vector<int> groups;
        for(size_t i = 0; i < parts.size(); i++) {
            const int g = parts[i]->fuelGroup;
            if(g < 0) { continue; }   /* a fuel barrier: in no group */
            bool seen = false;
            for(size_t k = 0; k < groups.size(); k++) {
                if(groups[k] == g) { seen = true; break; }
            }
            if(!seen) { groups.push_back(g); }
        }
        std::sort(groups.begin(), groups.end());
        /* ONE line per sample (e2e-greppable): each group's current /
           capacity per resource, with the member tanks' own contents in
           brackets (parts order -- a pro-rata drain keeps them equal),
           then the fuel links as group pairs. */
        printf("[fuel] t=%.3fs ship=\"%s\"", time, name.c_str());
        for(size_t gi = 0; gi < groups.size(); gi++) {
            const int g = groups[gi];
            /* the resources this group carries (any member tank has
               capacity > 0), printed in resource order. */
            std::vector<int> res;
            for(int r = 0; r < (int)ResourceType::Num; r++) {
                for(size_t i = 0; i < parts.size(); i++) {
                    Part *p = parts[i];
                    if(p->fuelGroup != g || !p->isTank()) { continue; }
                    if(p->def->capacity[r] > 0.0f) { res.push_back(r); break; }
                }
            }
            printf(" g%d=", g);
            for(size_t ri = 0; ri < res.size(); ri++) {
                if(ri > 0) { printf(" "); }
                const int r = res[ri];
                float cur = 0.0f, cap = 0.0f;
                std::vector<float> tankCur;
                for(size_t i = 0; i < parts.size(); i++) {
                    Part *p = parts[i];
                    if(p->fuelGroup != g || !p->isTank()) { continue; }
                    if(p->def->capacity[r] <= 0.0f) { continue; }
                    cur += p->resources.current[r];
                    cap += p->resources.capacity[r];
                    tankCur.push_back(p->resources.current[r]);
                }
                printf("%s:%.1f/%.1f[", resourceName(r), cur, cap);
                for(size_t ti = 0; ti < tankCur.size(); ti++) {
                    if(ti > 0) { printf(","); }
                    printf("%.1f", tankCur[ti]);
                }
                printf("]");
            }
        }
        /* the fuel links, collapsed to group ids (the same rule
           fuelDrainLayers applies: skip barrier endpoints and self-links). */
        if(!fuelLinks.empty()) {
            printf(" links=");
            bool first = true;
            for(size_t k = 0; k < fuelLinks.size(); k++) {
                const int a = fuelLinks[k].from->fuelGroup;
                const int b = fuelLinks[k].to->fuelGroup;
                if(a < 0 || b < 0 || a == b) { continue; }
                if(!first) { printf(","); }
                printf("g%d->g%d", a, b);
                first = false;
            }
        }
        printf("\n");
        fflush(stdout);
    }

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
    void drain_log(double time) {
        // Each group's total fuel mass now, in one pass over the parts.
        std::map<int, double> cur;
        double thrust = 0.0;
        for(size_t i = 0; i < parts.size(); i++) {
            Part *p = parts[i];
            thrust += (double)p->armedThrust;
            if(p->fuelGroup < 0 || !p->isTank()) { continue; }
            for(int r = 0; r < (int)ResourceType::Num; r++) {
                if(p->resources.current[r] <= 0.0f) { continue; }
                cur[p->fuelGroup] += p->resources.current[r];
            }
        }
        if(drainPrevTime_ > 0.0 && time > drainPrevTime_) {
            const double dt = time - drainPrevTime_;
            printf("[drainlog] t=%.3fs dt=%.3fs ship=\"%s\" thrust=%.1fN",
                   time, dt, name.c_str(), thrust);
            for(std::map<int, double>::iterator it = cur.begin(); it != cur.end(); ++it) {
                const int g = it->first;
                const double mass = it->second;
                const double prev = drainPrevMass_.count(g) ? drainPrevMass_[g] : mass;
                double rate = (prev - mass) / dt;
                if(rate < 0.0) { rate = 0.0; }   /* float noise / a stage that just dropped */
                printf(" g%d=%.3f", g, rate);
            }
            printf(" kg/s\n");
            fflush(stdout);
        }
        drainPrevTime_ = time;
        drainPrevMass_.swap(cur);
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
    std::vector<Part *> droppedPartsAtStage(int stage) {
        /* parent -> children, from Part::parent (each non-root part has
           exactly one parent edge, so this is a tree). */
        std::map<Part *, std::vector<Part *>> children;
        for(Part *p : parts) {
            if(p->parent != nullptr) { children[p->parent].push_back(p); }
        }
        std::set<Part *> dropped;
        for(Part *p : parts) {
            if(!p->isDecoupler() || p->stage != stage) { continue; }
            dropped.insert(p);   // the decoupler flies off with its stage
            /* BFS over the decoupler's child side (its direct children and
               their descendants). */
            std::vector<Part *> stack;
            for(size_t i = 0; i < children[p].size(); i++) { stack.push_back(children[p][i]); }
            while(!stack.empty()) {
                Part *q = stack.back(); stack.pop_back();
                if(dropped.count(q)) { continue; }
                dropped.insert(q);
                auto it = children.find(q);
                if(it != children.end()) {
                    for(size_t i = 0; i < it->second.size(); i++) { stack.push_back(it->second[i]); }
                }
            }
        }
        return std::vector<Part *>(dropped.begin(), dropped.end());
    }

    /* Separate `stage`: for every decoupler on that stage, cut the weld to
       its parent and drop the decoupler plus its child-side subtree (see
       droppedPartsAtStage). The dropped parts are removed from the Bullet
       world and this ship's part set and deleted; the survivors keep their
       relative geometry (their internal welds are untouched). Refuses to
       drop the whole ship. Returns the number of parts dropped (0 = no
       decoupler on this stage, a no-op). Call at a tick boundary, not
       mid-substep. */
    int separateStage(int stage) {
        std::vector<Part *> dropped = droppedPartsAtStage(stage);
        if(dropped.empty()) { return 0; }   // no decoupler on this stage
        if(dropped.size() == parts.size()) { return 0; }   // can't drop the whole ship
        std::set<Part *> droppedSet(dropped.begin(), dropped.end());
        const bool controllerDropped =
            (controller != nullptr && droppedSet.count(controller) > 0);

        /* collect the survivors FIRST (their Part* stay valid), then free the
           dropped parts -- so no freed pointer is ever dereferenced. */
        std::vector<Part *> keepParts;
        for(Part *p : parts) { if(!droppedSet.count(p)) { keepParts.push_back(p); } }

        parts.swap(keepParts);
        /* The survivors keep their authored poses in the ORIGINAL frame S
           (the root never drops -- a decoupler takes its child-side subtree),
           so S is still well defined and rebuildCompound carries the ship's
           pose and velocity across untouched.

           Rebuild the ship's single body from the survivors BEFORE freeing
           the dropped parts: the old compound still references their
           collision hulls as children, and ~Part frees those. Rebuilding is
           also what takes the dropped hulls out of the collision world, the
           compound being the only thing registered -- a part has no rigid
           body of its own to unregister, and nothing to un-weld. */
        rebuildCompound();
        for(Part *p : dropped) { delete p; }
        /* 3b) Drop fuel links whose endpoint is in the dropped set (a link
           touching a removed part is dangling). A Part* is stable, so the
           surviving links keep their (still-alive) endpoints. */
        std::vector<FuelLink> keepFuelLinks;
        for(size_t k = 0; k < fuelLinks.size(); k++) {
            if(droppedSet.count(fuelLinks[k].from) ||
               droppedSet.count(fuelLinks[k].to)) { continue; }
            keepFuelLinks.push_back(fuelLinks[k]);
        }
        fuelLinks.swap(keepFuelLinks);
        /* 4) If the controller was dropped, fall back to the first survivor
           (a Part* is stable -- no index remapping). */
        if(controllerDropped) { controller = parts[0]; }
        /* 5) Disarm any thrust (the split just happened). */
        clearThrust();
        /* 6) Recompute fuel groups. The survivors' pools are unchanged in
           practice (the decoupler that fired was the separator, so the two
           sides were already separate groups) -- recompute so the ids stay
           fresh after the tree shrank. */
        buildFuelGroups();
        /* 7) Reset the --drain-log baseline: a rate spanning this split
           would mix the two phases (and its mass map is about to lose the
           dropped groups). The next drain_log call re-baselines silently,
           so every printed rate covers one phase only. */
        drainPrevTime_ = 0.0;
        drainPrevMass_.clear();
        return (int)dropped.size();
    }

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

    /* Docking INTENT: the port (on another ship) this ship wants to mate
       with, set by right-clicking that port -> "Target for docking".
       Game::updateDocking only docks a ship that has a target, and clears
       it on success -- so an undock cannot immediately re-dock (the intent
       is gone; the player must re-target). Held PER SHIP (not on Game) so
       every ship carries its own intent, which is what a future AI-controlled
       ship needs to dock under its own steam. The pointers are validated and
       dropped when the target ship/port goes away (see updateDocking and the
       cleanups where a ship is deleted). */
    Vehicle *dockTargetShip = nullptr;
    Part *dockTargetPort = nullptr;

    void absorbShip(Vehicle *B, Part *portA) {
        if(B == nullptr || B == this || B->parts.empty()) { return; }
        if(portA == nullptr || !portA->isDockingPort()) { return; }

        /* Rigid rebase of B's parts from B's frame into this ship's S:
           with frame S at world (p, R) and S_B at (p_B, R_B),
               x_S  =  R^T R_B x_Sb + R^T (p_B - p)   (position)
           R^T R_B likewise for the rotations. */
        glm::dvec3 pA, pB; glm::dmat3 RA, RB;
        frameS(pA, RA);
        B->frameS(pB, RB);
        const glm::dmat3 T_rot = glm::transpose(RA) * RB;
        const glm::dvec3 T_pos = glm::transpose(RA) * (pB - pA);
        for(Part *q : B->parts) {
            q->localPos = T_rot * q->localPos + T_pos;
            q->localRot = T_rot * q->localRot;
        }

        /* Rigid-body state before the move (B's hull is still live; after,
           it is an empty shell). */
        const double mA = getMass();
        const double mB = B->getMass();
        const glm::dvec3 vA = GetVelocity(hull);
        const glm::dvec3 vB = GetVelocity(B->hull);
        const glm::dvec3 wA = GetAngVelocity(hull);

        /* Topology: B's root hangs off this ship's port part -- one tree
           edge, the two port parts being the joint. The part lists, fuel
           links and crew move into this ship; B is left an empty shell. */
        const size_t aSize = parts.size();
        Part *bRoot = B->rootPart();
        bRoot->parent = portA;

        /* crew: their capsules are B's parts, now at aSize + (old index). */
        std::map<size_t, size_t> reindex;
        for(size_t i = 0; i < B->parts.size(); i++) { reindex[i] = aSize + i; }
        for(size_t i = 0; i < B->crew.size(); i++) {
            B->crew[i]->crewRebase(this, reindex);
            crew.push_back(B->crew[i]);
        }
        B->crew.clear();

        parts.insert(parts.end(), B->parts.begin(), B->parts.end());
        B->parts.clear();
        fuelLinks.insert(fuelLinks.end(), B->fuelLinks.begin(), B->fuelLinks.end());
        B->fuelLinks.clear();

        /* stage counters: the union (the parts keep their baked-in numbers). */
        if(B->totalStages_ > totalStages_) { totalStages_ = B->totalStages_; }
        if(B->activeStage_ > activeStage_) { activeStage_ = B->activeStage_; }

        clearThrust();
        clearRotCmd();

        seams.push_back(DockSeam{ portA, bRoot, B->name });

        /* Rebuild as the union (carries frame S + the velocity, which the
           inelastic average below then corrects), and regroup the fuel --
           the port parts are fuel barriers, so the two ships' fuel systems
           stay separate groups inside the one body. */
        rebuildCompound();
        buildFuelGroups();
        SetVelocity(hull, (mA * vA + mB * vB) / (mA + mB));
        SetAngVelocity(hull, wA);
    }

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
    Vehicle *extractSubtreeAsShip(Part *root, const std::string &name) {
        if(root == nullptr) { return nullptr; }
        bool found = false;
        for(Part *p : parts) { if(p == root) { found = true; break; } }
        if(!found) { return nullptr; }

        /* The subtree: `root` plus its descendants (BFS over the children
           map, the same walk droppedPartsAtStage uses). */
        std::map<Part *, std::vector<Part *>> children;
        for(Part *p : parts) {
            if(p->parent != nullptr) { children[p->parent].push_back(p); }
        }
        std::set<Part *> droppedSet;
        std::vector<Part *> dropped;
        droppedSet.insert(root);
        dropped.push_back(root);
        for(size_t i = 0; i < dropped.size(); i++) {
            auto it = children.find(dropped[i]);
            if(it == children.end()) { continue; }
            for(size_t k = 0; k < it->second.size(); k++) {
                Part *q = it->second[k];
                if(droppedSet.count(q)) { continue; }
                droppedSet.insert(q);
                dropped.push_back(q);
            }
        }
        if(droppedSet.size() == parts.size()) { return nullptr; }  // can't split the whole ship

        /* New frame S' = the root's old frame: x' = R_r^T (x - p_r),
           R' = R_r^T R. */
        const glm::dvec3 pR = root->localPos;
        const glm::dmat3 RR = root->localRot;

        /* Rigid velocity of the dropped side's COM (before the rebase --
           partPos needs the current hull). */
        double M = 0.0;
        glm::dvec3 comDropped(0.0);
        for(Part *q : dropped) { M += q->body->mass; comDropped += q->body->mass * partPos(q); }
        comDropped /= M;
        const glm::dvec3 vOut = GetVelocity(hull)
                              + glm::cross(GetAngVelocity(hull), comDropped - comPos());
        const glm::dvec3 w = GetAngVelocity(hull);
        glm::dvec3 rootWorldPos; glm::dmat3 rootWorldRot;
        partWorldPose(root, rootWorldPos, rootWorldRot);

        /* New ship: same frame/home/sun (the split is local); no scenario
           (it is a runtime ship, not a def build). */
        Vehicle *nv = new Vehicle();
        nv->name = name;
        nv->defPath = "";
        nv->m_parent = m_parent;
        nv->frame = frame;
        nv->home = home;
        nv->sun = sun;
        nv->scenario = nullptr;

        /* Part list in the original order (stable indices for the crew
           reindex below); rebase the poses into S'. */
        std::vector<Part *> nvParts;
        std::map<size_t, size_t> goReindex;   // old index -> new ship index
        for(size_t i = 0; i < parts.size(); i++) {
            if(!droppedSet.count(parts[i])) { continue; }
            Part *q = parts[i];
            q->localPos = glm::transpose(RR) * (q->localPos - pR);
            q->localRot = glm::transpose(RR) * q->localRot;
            nvParts.push_back(q);
            goReindex[i] = nvParts.size() - 1;
        }
        root->parent = nullptr;   // root of the new ship
        nv->parts = nvParts;
        /* controller: the build rule (the first wheel, else the root). */
        nv->controller = nullptr;
        for(size_t i = 0; i < nvParts.size(); i++) {
            if(nvParts[i]->isWheel()) { nv->controller = nvParts[i]; break; }
        }
        if(nv->controller == nullptr) { nv->controller = root; }

        /* fuel links: both endpoints dropped -> the new ship; both kept ->
           this ship; crossing the cut -> dangling, dropped. */
        std::vector<FuelLink> nvLinks, keepLinks;
        for(size_t k = 0; k < fuelLinks.size(); k++) {
            const bool fIn = droppedSet.count(fuelLinks[k].from) > 0;
            const bool tIn = droppedSet.count(fuelLinks[k].to) > 0;
            if(fIn && tIn) { nvLinks.push_back(fuelLinks[k]); }
            else if(!fIn && !tIn) { keepLinks.push_back(fuelLinks[k]); }
        }
        nv->fuelLinks = nvLinks;
        fuelLinks = keepLinks;

        /* crew: a kerbal follows its capsule (crewRebase tells which side
           it is on); the survivors' slots reindex too, because erasing the
           dropped parts shifts the indices before them. */
        std::map<size_t, size_t> stayReindex;
        {
            size_t j = 0;
            for(size_t i = 0; i < parts.size(); i++) {
                if(!droppedSet.count(parts[i])) { stayReindex[i] = j++; }
            }
        }
        std::vector<Vehicle *> movedCrew;
        for(size_t i = 0; i < crew.size(); i++) {
            Vehicle *k = crew[i];
            if(k->crewRebase(nv, goReindex)) { movedCrew.push_back(k); }
            else { k->crewRebase(this, stayReindex); }
        }
        for(size_t i = 0; i < movedCrew.size(); i++) { nv->crew.push_back(movedCrew[i]); }
        {
            std::vector<Vehicle *> keepCrew;
            for(size_t i = 0; i < crew.size(); i++) {
                bool gone = false;
                for(size_t j = 0; j < movedCrew.size(); j++) { if(crew[i] == movedCrew[j]) { gone = true; break; } }
                if(!gone) { keepCrew.push_back(crew[i]); }
            }
            crew = keepCrew;
        }

        /* this ship: the survivors (their Part* are valid; no delete order
           to worry about -- the new ship owns the dropped parts). */
        {
            std::vector<Part *> keep;
            for(Part *p : parts) { if(!droppedSet.count(p)) { keep.push_back(p); } }
            parts.swap(keep);
        }
        const bool controllerDropped = (controller != nullptr && droppedSet.count(controller) > 0);
        rebuildCompound();
        if(controllerDropped) { controller = parts[0]; }
        clearThrust();
        buildFuelGroups();
        drainPrevTime_ = 0.0;
        drainPrevMass_.clear();

        /* The new ship: finalize (no tank re-seed -- the parts carry their
           current contents), place it at the root's world pose, and give it
           the rigid velocity of its COM. The caller enters it into the
           physics world (enterWorld) -- kept out so the split runs headless. */
        nv->finalize();
        nv->placeShip(rootWorldPos, rootWorldRot);
        nv->setVelocity(vOut);
        SetAngVelocity(nv->hull, w);
        return nv;
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
                ComputeTerrainShadow(m_parent, frame, partPos(p), sun);
            /* Drawn at the part's world pose rather than at a matrix read
               off its own rigid body: a ship is ONE body, so a part's pose
               is derived. (While partWorldPose still reads the per-part
               body this is the same matrix Draw would have built itself --
               measured to 1.7e-18 on the engine plume, which is built the
               same way.) */
            glm::dvec3 pp; glm::dmat3 pr;
            partWorldPose(p, pp, pr);
            p->body->DrawAt(camera, sunlightVec, shadow,
                            glm::translate(pp) * glm::dmat4(pr), xform);
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
            // RCS translation in the ship's own axes (rcsWorldDir): each
            // command arms one body axis; diagonals compose as a vector sum,
            // like the stick.
            case RcsNose:
                rcsDir.z = (cmd.amount >= 0) ? +1.0 : -1.0;
                break;
            case RcsUp:
                rcsDir.y = (cmd.amount >= 0) ? +1.0 : -1.0;
                break;
            case RcsRight:
                rcsDir.x = (cmd.amount >= 0) ? +1.0 : -1.0;
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

    /* The COM velocity -- get_center_of_mass() is the COM position, so the
       orbit elements are now fitted to one point's state rather than the
       controller part's velocity at the cluster's COM. */
    glm::dvec3 GetVel() {
        return GetVelocity(hull);
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

    /* the ship's full-throttle thrust RIGHT NOW (N) = the sum of every
       engine that has already been ignited (stage <= the stage counter) of
       its full thrust (each T = (H2 + LOX flow) x ve = 2 x fuel_rate x ve,
       both propellants end up in the plume), scaled by exhaust_scale (the
       test knob). Engines stay lit once ignited, so this is the sum of all
       lit engines on the ship; for a single-stage ship it equals the grand
       total. */
    float GetActiveThrust() {
        const int as = activeStage();
        double t = 0;
        for(Part *p : parts) {
            if(p->isThruster() && p->stage <= as) { t += p->thrust(); }
        }
        return (float)(t * exhaust_scale);
    }

    /* Called once per physics tick (step = the tick's simulated duration).
       Consumes the tick's fuel and arms the per-thruster thrust; the force
       itself is applied by applyThrustForce() before EVERY substep below.
       A thruster that can't consume its flow this tick doesn't thrust.
       Every engine that has already been ignited (stage <= the stage
       counter) fires, and each draws its OWN fuel group's tanks (see
       fuelPool) -- so an engine keeps burning from its connected propellant
       until it runs dry or its tanks are dropped. Stage gates WHEN it
       ignites; the fuel group (connection) decides WHAT it burns. */
    void ApplyThrust(double step) {
        if(thruster_util == 0.0f) { return; } /* zero throttle: no burn, no plume */
        const int as = activeStage();
        for(Part *p : parts) {
            if(!p->isThruster()) { continue; }
            if(p->stage > as) { continue; } /* not ignited yet */
            const float flow =
                (float)(p->rate() * (double)thruster_util * step); /* kg this tick, per tank */
            if(consumeResourceMass(ResourceType::Hydrogen, flow, p) and
               consumeResourceMass(ResourceType::LOX,      flow, p))
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

    /* The ship's moment-of-inertia tensor about its COM, in world axes: the
       one rigid body's own. Bullet stores it DIAGONAL in the principal frame,
       so rotating it out by the body's basis gives the tensor -- the same
       parallel-axis assembly this used to be built from by hand, already done
       by calculatePrincipalAxisTransform and held against that assembly by
       checkCompoundInvariants. O(1) rather than O(parts), and this is read
       every substep by the slew and kill-rot laws. */
    glm::dmat3 getInertia() {
        glm::dvec3 com; glm::dmat3 R;
        fromBt(hull->btBody->getCenterOfMassTransform(), com, R);
        const btVector3 &bi = hull->btBody->getLocalInertia();
        const glm::dmat3 diag(bi.getX(), 0.0, 0.0,
                              0.0, bi.getY(), 0.0,
                              0.0, 0.0, bi.getZ());
        return R * diag * glm::transpose(R);
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
        Part *wheel = firstWheel();
        const glm::dvec3 facing = partAxis(wheel, 2);
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
        const glm::dvec3 w_now = partAngVel(wheel);
        const glm::dvec3 w_transverse = w_now - facing * glm::dot(w_now, facing);
        glm::dvec3 dW = axis * w_des - w_transverse; /* desired change in rate */
        glm::dvec3 torque = I * dW / h;
        /* Authority bound: the wheel pushes at most maxTorque() N m, so scale
           the correction down if it would exceed that. Only active while a
           third-axis spin is present; with none, dW is along the slew axis
           and |torque| == maxTorque exactly as before. */
        const double tq = glm::length(torque);
        if(tq > maxTorque()) { torque *= maxTorque() / tq; }
        ApplyTorque(hull, torque);
    }

    /* Kill the spin within the wheel's authority: each axis' rate drops by
       min(|w|, alpha*h) per substep -- monotonic, no sign flip, never more
       forceful than a maxed manual stick. No deadband: the law is
       proportional, so it converges to exact zero. A fixed |w| cutoff would
       strand a residual spin whenever the per-substep authority alpha*h is
       smaller than the cutoff -- heavy ships (e.g. docked stacks) damp
       linearly into the cutoff and then keep drifting forever. */
    void killRotStep(double h) {
        Part *wheel = firstWheel();
        const glm::dvec3 w = partAngVel(wheel);
        if(glm::length2(w) == 0.0) { return; } /* at rest: nothing to kill */
        const glm::dmat3 I = getInertia();
        glm::dvec3 torque(0.0);
        for(int i = 0; i < 3; i++) {
            const double Iii = I[i][i];
            if(Iii <= 0.0 || w[i] == 0.0) { continue; }
            const double A = (maxTorque() / Iii) * h; /* max |dw| on this axis */
            const double dw = -w[i] * std::min(1.0, A / std::fabs(w[i]));
            torque[i] = Iii * dw / h; /* |torque[i]| <= maxTorque() */
        }
        ApplyTorque(hull, torque);
    }

public:

    /* A part's position in another frame's coordinates. Takes the Part, not
       its Body, so the read goes through the part accessors like every other
       consumer of a part's state. */
    glm::dvec3 GetPositionRelTo(const Part *part, Frame *relTo) {
        glm::dvec3 fpos = frame->GetPositionRelTo(relTo);
        glm::dmat3 forient = frame->GetOrientRelTo(relTo);
        return forient * partPos(part) + fpos;
    }

    void moveToFrame(Frame *newFrame) {
        /* One rigid body, so a frame change is one pose write and one
           velocity write; the transform is rigid, so the COM maps like any
           other point. */
        const glm::dvec3 oldCom = get_center_of_mass();
        const glm::dvec3 oldVel = GetVelocity(hull);
        const glm::dvec3 fpos = frame->GetPositionRelTo(newFrame);
        const glm::dmat3 forient = frame->GetOrientRelTo(newFrame);

        glm::dvec3 sPos; glm::dmat3 sRot;
        frameS(sPos, sRot);
        placeShip(forient * sPos + fpos, forient * sRot);

        // The stored velocity is the frame-coordinate velocity, so a ship's
        // inertial velocity is R*(v + stasis(p)) + V. The OLD frame's stasis
        // term is added here and the NEW frame's SUBTRACTED below, or the
        // ship's inertial velocity is wrong by 2*stasis and the orbit jumps
        // shape at every inertial->rotational switch.
        glm::dvec3 vel = oldVel;
        if(frame != newFrame) { vel += frame->GetStasisVelocity(oldCom); }
        vel = forient * vel + frame->GetVelocityRelTo(newFrame);
        const glm::dvec3 newCom = forient * oldCom + fpos;
        const glm::dvec3 newVel = vel - newFrame->GetStasisVelocity(newCom);

        printf("@@@ %s frame %s -> %s: com (%.0f %.0f %.0f) -> (%.0f %.0f %.0f)"
               " vel (%.0f %.0f %.0f) -> (%.0f %.0f %.0f)\n",
               name.c_str(), frame->name.c_str(), newFrame->name.c_str(),
               oldCom.x, oldCom.y, oldCom.z, newCom.x, newCom.y, newCom.z,
               oldVel.x, oldVel.y, oldVel.z, newVel.x, newVel.y, newVel.z);

        /* after placeShip: proceedToTransform zeroes both velocities */
        SetVelocity(hull, newVel);

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
                glm::dvec3 pos = partPos(controller);
                printf("@@@ %s switching frame from %s to parent %s\n",
                       name.c_str(), frame->name.c_str(),
                       frame->parent->name.c_str());
                glm::dvec3 offset = frame->GetPositionRelTo(frame->parent);
                printf("@@@ Frame offset: %.0f %.0f %.0f\n", offset.x, offset.y, offset.z);
                printf("@@@@@ OLD position: %.0f %.0f %.0f\n", pos.x, pos.y, pos.z);
                moveToFrame(frame->parent);
                pos = partPos(controller);
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

    /* Write the rail state into the ship's body (once per tick). Draw,
       get_center_of_mass and everything else that reads the body then sees
       the railed ship's current pose even though it is not in the world.
       Angular velocity is zeroed: a parked ship is torque-free, and readers
       like --orbit-log and the HUD fit their elements to consistent data. */
    void writeRailPose() {
        /* ONE write, and it is the whole ship. rail_pos is the COM, which is
           exactly the hull's transform origin, and rail_orient * railRot are
           frame S's axes; every part's pose then follows from its authored
           local pose. There is no per-part snapshot to restore and no
           deformation to freeze -- the geometry the rails carry IS the
           authored geometry. */
        glm::dvec3 pOrigin; glm::dmat3 pBasis;
        fromBt(principal, pOrigin, pBasis);
        setPosRot(hull, rail_pos, (rail_orient * railRot) * pBasis);
        SetVelocity(hull, rail_vel);
        SetAngVelocity(hull, glm::dvec3(0.0));
    }

    /* The ship's COM state in `inertial` -- the frame node where its
       trajectory is a Kepler conic (the same transform the HUD uses). The
       ordering matters: the OLD frame's stasis (rotation) velocity is added
       before rotating, and the result is offset by the frame's own velocity
       in the inertial node. Getting it wrong biases every conic fitted from
       here. */
    void comStateIn(Frame *inertial, glm::dvec3 &p, glm::dvec3 &v) {
        p = get_center_of_mass();
        v = GetVelocity(hull);
        if(frame != inertial) {
            v += frame->GetStasisVelocity(p);
            v = frame->GetOrientRelTo(inertial) * v + frame->GetVelocityRelTo(inertial);
            p = frame->GetOrientRelTo(inertial) * p + frame->GetPositionRelTo(inertial);
        }
    }

    // Separation between this ship's COM and another's, in the universe (root)
    // frame. The root is shared by every body in the system, so expressing
    // both COMs there gives a frame-invariant distance, independent of the SOI
    // each ship is currently tracking.
    double distanceTo(Vehicle *o) {
        Frame *root = frame;
        while(root->parent) { root = root->parent; }
        glm::dvec3 p, v, q, w;
        comStateIn(root, p, v);
        o->comStateIn(root, q, w);
        return glm::length(p - q);
    }

    /* The COM's osculating orbit dips into the terrain band (periapsis
       within 3 km of the surface): sitting on / skimming the ground rather
       than coasting clear of it. */
    bool inTerrainBand() {
        Frame *inertial = frame->getNonRotFrame();
        glm::dvec3 p, v;
        comStateIn(inertial, p, v);
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

        /* The COM state in the body's inertial frame node, where the
           trajectory is a Kepler conic. The velocity is the hull's own -- one
           rigid body, one COM velocity -- and the parked ship's residual spin
           is discarded with its attitude (writeRailPose zeroes it). */
        Frame *oldFrame = frame;
        Frame *inertial = frame->getNonRotFrame();
        const glm::dvec3 com_frame = get_center_of_mass();  // old frame coords
        const glm::dvec3 vel_frame = GetVelocity(hull);     // old frame coords
        glm::dvec3 p, v;
        comStateIn(inertial, p, v);

        const OrbitElements el = computeOrbitElements(p, v, inertial->body->mu);
        const bool grounded = el.periapsis <= inertial->body->radius + 3000.0;

        /* Frame S's axes at park time (== the old frame's axes for a ship
           built in it); rail_orient carries them into the inertial node and
           then holds inertially. Nothing per-part to snapshot: the ship is
           rigid, so the authored local poses ARE the parked geometry. */
        glm::dvec3 sPosPark;
        frameS(sPosPark, railRot);

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

        /* out of the world: the one body */
        if(hullInWorld()) { RemoveBody(hull); }

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
       state and hand the ship back to the integrator. Pose and velocity
       already track the rail state (writeRailPose), so this is just
       re-register -- nothing to re-weld, because a rigid body has no internal
       degrees of freedom and the parked geometry IS the authored geometry. */
    void leaveRails() {
        if(!onRails) { return; }
        writeRailPose();
        if(!hullInWorld()) { AddPhysicsBody(hull); }
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
