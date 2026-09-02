// eva.cpp -- the EVA kerbal's control laws (declarations in eva.h).

#include "eva.h"

#include <cstdio>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>      // length2

#include "camera.h"   // Camera (the arm reads the view basis)
#include "game.h"     // Game (the active kerbal, the clock)
#include "physics.h"  // BodyInContact, ApplyCentralForce, ApplyTorque, ...

// --- tuning (debug scope; see the design notes) ------------------------
static const double kWalkSpeed   = 2.5;     // m/s
static const double kWalkAccel   = 10.0;    // m/s^2 toward walkSpeed (damping)
static const double kJumpSpeed   = 2.5;     // m/s radial kick
static const double kRcsAccel    = 2.0;     // m/s^2 translation
static const double kRcsMaxSpeed = 3.0;     // m/s soft cap (along the input)
static const double kEvaTorque   = 100.0;   // N m attitude authority
static const double kMaxRate     = 6.0;     // rad/s attitude slew cap
static const double kYawRate     = 1.5;     // rad/s QE yaw about the view axis
static const double kGroundBand  = 0.25;    // m above restAlt still "grounded"
static const double kClearBand   = 0.7;     // m a jump must clear (contact margins)
static const double kFloorDrop   = 0.4;     // m below restAlt -> snap back up

void evaArmCommands(Game &g, const std::function<bool(SDL_Scancode)> &isDown) {
    Kerbal *k = static_cast<Kerbal *>(g.ship);
    const Camera *cam = g.camera;

    // The camera's pose snapshot: camera positions/directions live in the
    // render frame, which IS the active kerbal's frame.
    const glm::dvec3 fwd = glm::normalize(cam->forward);
    const glm::dvec3 up = glm::normalize(evaOntoPlane(cam->up, fwd));
    const glm::dvec3 sright = evaScreenRight(fwd, up);
    k->camBasis = evaCamBasis(fwd, up);

    // Grounded: Bullet contact (works on the pad mesh too) OR the analytic
    // terrain within the standing band. Terrain collision leaves only exist
    // at max LOD under the camera, so the analytic side is load-bearing.
    const glm::dvec3 pos = k->get_center_of_mass();
    const glm::dvec3 radial = glm::normalize(pos);
    const double alt = glm::length(pos)
        - (double)k->m_parent->GetTerrainHeight(glm::vec3(radial));
    const double rest = k->restAlt();
    k->grounded = BodyInContact(k->controller) || alt < rest + kGroundBand;
    if(k->jumping) {
        // still rising through the contact-margin band: stay ungrounded
        if(alt > rest + kClearBand) { k->jumping = false; }
        else { k->grounded = false; }
    }
    k->mode = k->grounded ? EVA_GROUND : EVA_SPACE;

    if(k->mode == EVA_GROUND) {
        glm::dvec3 w(0.0);
        if(isDown(SDL_SCANCODE_W)) { w += fwd; }
        if(isDown(SDL_SCANCODE_S)) { w -= fwd; }
        if(isDown(SDL_SCANCODE_D)) { w += sright; }
        if(isDown(SDL_SCANCODE_A)) { w -= sright; }
        // camera-relative -> surface-relative (walk along the tangent)
        w = evaOntoPlane(w, radial);
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
        if(isDown(SDL_SCANCODE_W)) { t += fwd; }
        if(isDown(SDL_SCANCODE_S)) { t -= fwd; }
        if(isDown(SDL_SCANCODE_D)) { t += sright; }
        if(isDown(SDL_SCANCODE_A)) { t -= sright; }
        k->rcsDir = (glm::length2(t) > 1e-9) ? glm::normalize(t)
                                             : glm::dvec3(0.0);
        k->walkDir = glm::dvec3(0.0);
        double yaw = 0.0;
        if(isDown(SDL_SCANCODE_Q)) { yaw += 1.0; }
        if(isDown(SDL_SCANCODE_E)) { yaw -= 1.0; }
        k->viewYaw += yaw * kYawRate * g.dt;
    }
}

void Kerbal::applyEva(double h) {
    Body *b = controller;
    const glm::dvec3 pos = GetPosition(b);
    const glm::dvec3 radial = glm::normalize(pos);
    const double surfR = (double)m_parent->GetTerrainHeight(glm::vec3(radial));
    const double rest = restAlt();

    /* Analytic floor guard: terrain collision meshes exist only at the
       max-LOD leaves under the camera, so where the leaves are not loaded
       there is nothing to stand on. Snap back to standing height instead
       of falling through the body (fires only without collision -- Bullet
       contact holds the kerbal near restAlt otherwise). */
    const double alt = glm::length(pos) - surfR;
    if(alt < rest - kFloorDrop) {
        setPosRot(b, radial * (surfR + rest), GetOrient(b));
        const glm::dvec3 v = GetVelocity(b);
        const double vr = glm::dot(v, radial);
        if(vr < 0.0) { SetVelocity(b, v - radial * vr); }
        return;
    }

    if(mode == EVA_GROUND) {
        if(jumpRequested) {
            jumpRequested = false;
            SetVelocity(b, GetVelocity(b) + radial * kJumpSpeed);
        }
        /* Walk steering: drive the tangent-plane velocity toward
           walkDir * walkSpeed; no input -> damp to a stand. */
        const glm::dvec3 v = GetVelocity(b);
        const glm::dvec3 vh = evaOntoPlane(v, radial);
        glm::dvec3 a = (walkDir * kWalkSpeed - vh) / h;
        const double amax = kWalkAccel;
        const double alen = glm::length(a);
        if(alen > amax) { a *= amax / alen; }
        ApplyCentralForce(b, b->mass * a);

        /* Stand along the local vertical, facing the walk direction
           (last heading while standing still). */
        const glm::dvec3 faceHint = (glm::length2(walkDir) > 0.0)
            ? walkDir : getRelAxis_(b, 1);
        slewTo(evaStandTarget(radial, faceHint), h);
    } else {
        /* RCS translation along the camera axes, soft-capped along the
           input direction. */
        if(glm::length2(rcsDir) > 0.0) {
            const glm::dvec3 v = GetVelocity(b);
            if(glm::dot(rcsDir, v) < kRcsMaxSpeed) {
                ApplyCentralForce(b, b->mass * kRcsAccel * rcsDir);
            }
        }
        /* Upright on screen, facing the camera direction, plus the
           accumulated QE yaw about the view axis. */
        const glm::dvec3 fwd = camBasis[2];
        slewTo(rotAbout(fwd, viewYaw) * evaSpaceTarget(camBasis), h);
    }
}

void Kerbal::slewTo(const glm::dmat3 &target, double h) {
    Body *b = controller;
    const glm::dmat3 R = GetOrient(b);
    glm::dvec3 axis;
    const double ang = evaRotAxisAngle(target * glm::transpose(R), axis);
    const glm::dvec3 w = GetAngVelocity(b);
    const glm::dmat3 I = getInertia();
    glm::dvec3 tq(0.0);
    if(ang < 1e-9) {
        // aligned: just kill the residual spin
        if(glm::length(w) < 1e-4) { return; }
        tq = -(I * w) / h;
    } else {
        // braking curve (the ship's slew law): the fastest rate from
        // which the EVA authority can still stop exactly on target,
        // capped so no substep crosses it. A plain linear rate law
        // overshoots and oscillates under the torque cap.
        const double Ieff = glm::dot(axis, I * axis);
        const double alpha = (Ieff > 0.0) ? kEvaTorque / Ieff : 0.0;
        const double w_des = glm::min(glm::min(std::sqrt(2.0 * alpha * ang),
                                               ang / (2.0 * h)), kMaxRate);
        tq = I * (axis * w_des - w) / h;
    }
    const double m = glm::length(tq);
    if(m > kEvaTorque) { tq *= kEvaTorque / m; }
    ApplyTorque(b, tq);
}
