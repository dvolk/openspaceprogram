// eva.cpp -- the EVA kerbal's control laws (declarations in eva.h).

#include "eva.h"

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
static const double kRcsForce    = 194.1;   // N; ~2.0 m/s^2 at the 97.05 kg wet suit
static const double kRcsFlow     = 0.090;   // kg/s = kRcsForce / (kRcsIsp * 9.81); ~111 s burn for 10 kg
static const double kEvaTorque   = 100.0;   // N m attitude authority (space)
static const double kGroundTorque = 300.0;  // N m attitude authority (ground)
static const double kMaxRate     = 6.0;     // rad/s attitude slew cap
static const double kYawRate     = 1.5;     // rad/s QE yaw about the view axis
static const double kGroundBand  = 0.25;    // m above restAlt still "grounded"
static const double kFloorDrop   = 0.4;     // m below restAlt -> snap back up

void evaArmCommands(Game &g, const std::function<bool(Slot)> &active) {
    Kerbal *k = static_cast<Kerbal *>(g.ship);
    const Camera *cam = g.camera;

    // Camera positions/directions live in the render frame (= the kerbal's frame).
    const glm::dvec3 fwd = glm::normalize(cam->forward);
    const glm::dvec3 up = glm::normalize(evaOntoPlane(cam->up, fwd));
    const glm::dvec3 sright = evaScreenRight(fwd, up);
    k->camBasis = evaCamBasis(fwd, up);

    // Grounded: Bullet contact OR analytic terrain (meshes only exist at max
    // LOD under the camera, so the analytic side is load-bearing).
    const glm::dvec3 pos = k->get_center_of_mass();
    const glm::dvec3 radial = glm::normalize(pos);
    const double alt = glm::length(pos)
        - (double)k->m_parent->GetTerrainHeight(glm::vec3(radial));
    const double rest = k->restAlt();
    k->grounded = BodyInContact(k->hull) || alt < rest + kGroundBand;
    if(k->jumping) {
        // Stay ungrounded until floor contact clears (altitude release never
        // fires when the jump apex is below the band, as on high-gravity terrain).
        if(BodyInContact(k->hull)) { k->grounded = false; }
        else { k->jumping = false; }
    }
    k->mode = k->grounded ? EVA_GROUND : EVA_SPACE;

    if(k->mode == EVA_GROUND) {
        glm::dvec3 w(0.0);
        if(active(Slot::EvaForward)) { w += fwd; }
        if(active(Slot::EvaBack)) { w -= fwd; }
        if(active(Slot::EvaRight)) { w += sright; }
        if(active(Slot::EvaLeft)) { w -= sright; }
        w = evaOntoPlane(w, radial);   // walk along the tangent
        k->walkDir = (glm::length2(w) > 1e-9) ? glm::normalize(w)
                                              : glm::dvec3(0.0);
        k->rcsDir = glm::dvec3(0.0);
        if(k->jumpPressed) {
            k->jumpPressed = false;
            if(k->grounded) {
                k->jumpRequested = true;
                k->jumping = true;
            }
        }
    } else {
        glm::dvec3 t(0.0);
        if(active(Slot::EvaForward)) { t += fwd; }
        if(active(Slot::EvaBack)) { t -= fwd; }
        if(active(Slot::EvaRight)) { t += sright; }
        if(active(Slot::EvaLeft)) { t -= sright; }
        if(active(Slot::EvaUp)) { t += up; }
        if(active(Slot::EvaDown)) { t -= up; }
        k->rcsDir = (glm::length2(t) > 1e-9)
            ? glm::normalize(t) : glm::dvec3(0.0);
        k->walkDir = glm::dvec3(0.0);
        double yaw = 0.0;
        if(active(Slot::EvaYawLeft)) { yaw += 1.0; }
        if(active(Slot::EvaYawRight)) { yaw -= 1.0; }
        k->viewYaw += yaw * kYawRate * g.dt;
    }
}

void Kerbal::applyEva(double h) {
    Body *b = hull;
    const glm::dvec3 pos = partPos(controller);
    const glm::dvec3 radial = glm::normalize(pos);
    const double surfR = (double)m_parent->GetTerrainHeight(glm::vec3(radial));
    const double rest = restAlt();

    // Analytic floor guard: snap back to standing height where terrain meshes
    // are not loaded (fires only without Bullet contact).
    const double alt = glm::length(pos) - surfR;
    if(alt < rest - kFloorDrop) {
        placeShipAtCom(radial * (surfR + rest), partRot(controller));
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
        // Walk steering: drive tangent velocity toward walkDir * walkSpeed.
        const glm::dvec3 v = partVel(controller);
        const glm::dvec3 vh = evaOntoPlane(v, radial);
        glm::dvec3 a = (walkDir * kWalkSpeed - vh) / h;
        const double amax = kWalkAccel;
        const double alen = glm::length(a);
        if(alen > amax) { a *= amax / alen; }
        ApplyCentralForce(b, b->mass * a);

        // Feet are frictionless (ships.cpp) so the COM steering force cannot
        // tip the capsule; overwriting the pose stalls translation instead.
        const glm::dvec3 faceHint = (glm::length2(walkDir) > 0.0)
            ? walkDir : partAxis(controller, 1);
        slewTo(evaStandTarget(radial, faceHint), h, kGroundTorque);
    } else {
        // RCS: fixed thrust while held AND hydrazine is available (consume-then-arm).
        if(glm::length2(rcsDir) > 0.0) {
            const float flow = (float)(kRcsFlow * h);
            // Suit tank first, then pocket inventory (not in this ship's fuel groups).
            bool haveFuel = consumeResourceMass(ResourceType::Hydrazine, flow, parts[0]);
            if(!haveFuel) {
                haveFuel = inventoryDrain(parts[0], (int)ResourceType::Hydrazine, flow);
            }
            if(haveFuel) {
                ApplyCentralForce(b, kRcsForce * rcsDir);
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
    const glm::dvec3 w = partAngVel(controller);
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
