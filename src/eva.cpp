// eva.cpp -- the EVA kerbal's control laws (declarations in eva.h).

#include "eva.h"

#include <cassert>
#include <cstdio>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>      // length2

#include "camera.h"    // Camera (the arm reads the view basis)
#include "game.h"      // Game (the active kerbal, the clock)
#include "inventory.h" // inventoryDrain (the pocket draw, phase 4.5)
#include "physics.h"   // BodyInContact, ApplyCentralForce, ApplyTorque, ...

// --- tuning -----------------------------------------------------------
static const double kWalkSpeed   = 2.5;     // m/s
static const double kWalkAccel   = 10.0;    // m/s^2 toward walkSpeed (damping)
static const double kJumpSpeed   = 2.5;     // m/s radial kick
// RCS: fixed thrust (N), not fixed accel -- no speed cap; propellant is the limiter.
static const double kRcsIsp      = 220.0;   // s, monopropellant efficiency
static const double kRcsForce    = 194.1;   // N; ~2 m/s^2 on the wet suit
static const double kRcsFlow     = 0.090;   // kg/s = kRcsForce / (kRcsIsp * 9.81)
static const double kEvaTorque   = 100.0;   // N m attitude authority (space)
static const double kGroundTorque = 300.0;  // N m attitude authority (ground)
static const double kMaxRate     = 6.0;     // rad/s attitude slew cap
static const double kYawRate     = 1.5;     // rad/s QE yaw about the view axis
static const double kGroundBand  = 0.25;    // m above restAlt still "grounded"
static const double kFloorDrop   = 0.4;     // m below restAlt -> snap back up

bool Kerbal::onDeck() const {
    return frame != nullptr && frame->building != nullptr && frame->building->air;
}

double Kerbal::restAlt() const {
    /* The surface margin must be the one the contact actually uses: the
       terrain's on the ground, the DECK's own effective margin in a platform
       bubble. Reading terrainMargin there rests the suit a fraction of a
       metre above the contact, and the floor guard fights the solver in a
       bounce limit cycle (#850). Both helpers respect OSP_*_MARGIN. */
    const double surf = onDeck() ? effectiveHullMargin(frame->building->body)
                                 : terrainMargin();
    return parts[0]->def->height / 2.0 + hullMargin() + surf;
}

bool Kerbal::inStationBubble() const {
    return frame != nullptr && frame->building != nullptr && !frame->building->air;
}

glm::dvec3 Kerbal::groundUp() const {
    if(onDeck()) { return glm::dvec3(0.0, 0.0, 1.0); }
    return glm::normalize(comPos());
}

double Kerbal::groundAltAt(const glm::dvec3 &pos) const {
    if(onDeck()) { return pos.z; }   // deck top authored at local z=0
    return glm::length(pos)
        - (double)m_parent->GetTerrainHeight(glm::vec3(glm::normalize(pos)));
}

bool Kerbal::overDeck() const {
    /* True when a ray straight down from the suit hits the platform deck:
       over the footprint, not off the edge. The target sits a little below
       the deck plane so a standing suit always reaches it; a suit fallen
       past it is off the deck and must keep falling -- the floor is the
       deck, not an infinite plane at z=0 (#850). */
    if(!onDeck()) { return false; }
    const glm::dvec3 pos = comPos();
    glm::dvec3 hit, n;
    Body *gb = nullptr;
    return groundRay(pos, glm::dvec3(pos.x, pos.y, -(kFloorDrop + 1.0)),
                     hit, n, hull, &gb)
        && gb == frame->building->body;
}

/* The analytic half of armInput's contact test, without the Bullet query:
   a hull freshly added by enterWorld has no manifolds to find, and the pose
   already says everything. A kerbal loaded standing on a structure that rises
   above the analytic surface reads "space" until the first tick's
   BodyInContact flips it back -- except on a platform deck, whose bubble
   knows its own floor (#850). */
void Kerbal::settleContact() {
    assert(m_parent != nullptr && !parts.empty());
    if(inStationBubble()) {
        grounded = false;   // weightless: no deck to stand on (#850)
        mode = EVA_SPACE;
        return;
    }
    const double alt = groundAltAt(comPos());
    grounded = alt < restAlt() + kGroundBand && (!onDeck() || overDeck());
    mode = grounded ? EVA_GROUND : EVA_SPACE;
}

void Kerbal::armInput(Game &g, const SlotProbe &active) {
    const Camera *cam = g.camera;

    // Camera positions/directions live in the render frame (= the kerbal's frame).
    const glm::dvec3 fwd = glm::normalize(cam->forward);
    const glm::dvec3 up = glm::normalize(evaOntoPlane(cam->up, fwd));
    const glm::dvec3 sright = evaScreenRight(fwd, up);
    camBasis = evaCamBasis(fwd, up);

    const glm::dvec3 radial = groundUp();
    const double alt = groundAltAt(get_center_of_mass());
    const double rest = restAlt();
    grounded = !inStationBubble()
        && (BodyInContact(hull)
            || (alt < rest + kGroundBand && (!onDeck() || overDeck())));
    if(jumping) {
        /* Release once the jump's outward kick is spent. Releasing on
           contact-clear never fires for a hop that never leaves contact;
           releasing on altitude never fires when the apex is under the band.
           Either way the latch sticks and the kerbal is stuck in space mode,
           where W burns suit RCS instead of walking. Gravity spends the kick
           on any body. */
        if(glm::dot(GetVel(), radial) > 0.0) { grounded = false; }
        else { jumping = false; }
    }
    mode = grounded ? EVA_GROUND : EVA_SPACE;
    /* Spend the jump edge whichever mode is running (#578). It used to be read only on
       the ground branch, so a press during a spacewalk survived the tick and fired on
       the next landing: SPACE is the staging key for every ship, so tapping it out of
       habit while flying the suit collected an unasked 2.5 m/s hop on touchdown. */
    const bool jumpWanted = jumpPressed;
    jumpPressed = false;

    if(mode == EVA_GROUND) {
        glm::dvec3 w(0.0);
        if(active.held(Slot::EvaForward)) { w += fwd; }
        if(active.held(Slot::EvaBack)) { w -= fwd; }
        if(active.held(Slot::EvaRight)) { w += sright; }
        if(active.held(Slot::EvaLeft)) { w -= sright; }
        w = evaOntoPlane(w, radial);   // walk along the tangent
        walkDir = (glm::length2(w) > 1e-9) ? glm::normalize(w)
                                           : glm::dvec3(0.0);
        armRcs(glm::dvec3(0.0));   // walking, not flying: no translation demand
        if(jumpWanted) {           // EVA_GROUND means grounded (two lines above)
            jumpRequested = true;
            jumping = true;
        }
    } else {
        glm::dvec3 t(0.0);
        if(active.held(Slot::EvaForward)) { t += fwd; }
        if(active.held(Slot::EvaBack)) { t -= fwd; }
        if(active.held(Slot::EvaRight)) { t += sright; }
        if(active.held(Slot::EvaLeft)) { t -= sright; }
        if(active.held(Slot::EvaUp)) { t += up; }
        if(active.held(Slot::EvaDown)) { t -= up; }
        armRcs((glm::length2(t) > 1e-9)
            ? glm::normalize(t) : glm::dvec3(0.0));
        walkDir = glm::dvec3(0.0);
        double yaw = 0.0;
        if(active.held(Slot::EvaYawLeft)) { yaw += 1.0; }
        if(active.held(Slot::EvaYawRight)) { yaw -= 1.0; }
        viewYaw += yaw * kYawRate * g.dt;
    }
}

void Kerbal::applyEva(double h) {
    Body *b = hull;
    const glm::dvec3 pos = partPos(controller);
    const glm::dvec3 radial = groundUp();
    const double rest = restAlt();

    // Analytic floor guard: snap back to standing height where terrain meshes
    // are not loaded. In a platform bubble the floor is the DECK, not an
    // infinite plane at z=0: off the edge the suit keeps falling (#850). In a
    // STATION bubble there is no floor -- weightless, and snapping would
    // teleport the suit a planet away.
    if(!inStationBubble() && groundAltAt(pos) < rest - kFloorDrop) {
        if(onDeck()) {
            if(!overDeck()) { return; }   // off the edge: free fall
            placeShipAtCom(glm::dvec3(pos.x, pos.y, rest), partRot(controller));
        } else {
            placeShipAtCom(radial * ((double)m_parent->GetTerrainHeight(
                                         glm::vec3(glm::normalize(pos))) + rest),
                           partRot(controller));
        }
        const glm::dvec3 v = partVel(controller);
        const double vr = glm::dot(v, radial);
        if(vr < 0.0) { SetVelocity(b, v - radial * vr); }
        return;
    }

    if(mode == EVA_GROUND) {
        if(jumpRequested) {
            jumpRequested = false;
            SetVelocity(b, partVel(controller) + radial * kJumpSpeed);
        }
        const glm::dvec3 v = partVel(controller);
        /* The ground is stationary in the kerbal's frame: terrain in the rot
           frame, a platform deck in its bubble (#850). The feet are
           frictionless, so this law is the kerbal's only horizontal grip. */
        const glm::dvec3 vh = evaOntoPlane(v, radial);
        glm::dvec3 a = (walkDir * kWalkSpeed - vh) / h;
        const double amax = kWalkAccel;
        const double alen = glm::length(a);
        if(alen > amax) { a *= amax / alen; }
        ApplyCentralForce(b, b->mass * a);

        // Feet are frictionless (Kerbal ctor) so the COM steering force cannot
        // tip the capsule; overwriting the pose would stall translation instead.
        const glm::dvec3 faceHint = (glm::length2(walkDir) > 0.0)
            ? walkDir : partAxis(controller, 1);
        slewTo(evaStandTarget(radial, faceHint), h, kGroundTorque);
    } else {
        // RCS: fixed thrust while held AND hydrazine is available (consume-then-arm).
        if(glm::length2(rcsDir) > 0.0) {
            const double flow = kRcsFlow * h;
            // Suit tank first, then pocket inventory (not in this ship's fuel groups).
            bool haveFuel = consumeResourceMass(ResourceType::Hydrazine, flow, parts[0]).ok;
            if(!haveFuel) {
                haveFuel = inventoryDrain(parts[0], (int)ResourceType::Hydrazine, flow);
            }
            if(haveFuel) {
                ApplyCentralForce(b, kRcsForce * rcsWorldDir());
                // The render pass draws the COM puff off this (render.cpp).
                markRcsBurned();
            }
        }
        // Upright on screen, facing the camera direction, plus QE yaw.
        const glm::dvec3 fwd = camBasis[2];
        slewTo(rotAbout(fwd, viewYaw) * evaSpaceTarget(camBasis), h,
               kEvaTorque);
    }
}

void Kerbal::slewTo(const glm::dmat3 &target, double h, double authority) {
    Body *b = hull;
    const glm::dmat3 R = partRot(controller);
    glm::dvec3 axis;
    const double ang = evaRotAxisAngle(target * glm::transpose(R), axis);
    const glm::dvec3 w = hullAngVel();
    const glm::dmat3 I = getInertia();
    glm::dvec3 tq(0.0);
    if(ang < 1e-9) {
        if(glm::length(w) < 1e-4) { return; }   // aligned: kill residual spin
        tq = -(I * w) / h;
    } else {
        // Braking curve: fastest rate that can still stop exactly on target.
        // A plain linear rate law overshoots and oscillates under the torque cap.
        const double Ieff = glm::dot(axis, I * axis);
        const double alpha = (Ieff > 0.0) ? authority / Ieff : 0.0;
        const double w_des = glm::min(glm::min(std::sqrt(2.0 * alpha * ang),
                                               ang / (2.0 * h)), kMaxRate);
        tq = I * (axis * w_des - w) / h;
    }
    const double m = glm::length(tq);
    if(m > authority) { tq *= authority / m; }
    ApplyTorque(b, tq);
}
