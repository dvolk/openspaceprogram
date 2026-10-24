#pragma once

// gimbal.h -- the engine-gimbal (TVC) law as pure math (glm only), so tests/
// can pin it without the render/physics chain (like drag.h, orbit.h).
//
// A universal-joint gimbal rotates the thrust vector off the part's authored
// +Z: pitch about the part's local X, yaw about its local Y (the OUTER ring,
// applied after the inner one). The deviation force applied at the engine's
// offset from the COM is what torques the ship -- see Vehicle::applyThrustForce.
//
// Sign convention (pinned in tests/test_gimbal.cpp): the actuator targets for
// a whole ship come from ONE lateral direction in the ship's root frame
// (gimbalTargets), so a ring of surface-mounted engines ADDS authority rather
// than rolling the ship, and engines keep steering the way the pilot commands
// even when mounted rolled. That shared lateral answers the stick the same way
// the reaction-wheel torque in Vehicle::applyRotationForce does -- for the
// ordinary case of engines levers BELOW the COM. An engine on the far side of
// the COM (reversed engines in a docked pair, COM travel mid-burn) pushes the
// nose the other way; a torque-aware mixer is the autopilot stage's job.

#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

/* The part's pose SWUNG by its gimbal angles: the authored frame rotated by
   the same q that gimbalDir applies, so anything that draws the engine --
   plume now, nozzle later -- swings with the thrust vector. Orthonormality
   holds: q is a rotation. */
inline glm::dmat3 gimbalRot(const glm::dmat3 &partRot, double pitch, double yaw) {
    if(pitch == 0.0 && yaw == 0.0) { return partRot; }
    const glm::dquat q = glm::angleAxis(yaw, partRot[1])
                       * glm::angleAxis(pitch, partRot[0]);
    return glm::mat3_cast(q) * partRot;
}

/* The gimballed thrust direction (unit) in WORLD axes: the part's authored +Z
   rotated by `pitch` about its local X, then `yaw` about its local Y (rad).
   Zero angles return the authored axis untouched. */
inline glm::dvec3 gimbalDir(const glm::dmat3 &partRot, double pitch, double yaw) {
    return gimbalRot(partRot, pitch, yaw)[2];
}

/* One actuator step: move each angle toward its target at <= rate * h (rad,
   rad/s, s). Exact arrival when close; no overshoot. Callers keep the angles
   inside +/-gimbal_max by aiming at targets inside it. */
inline void gimbalStep(double cur[2], const double target[2], double rate, double h) {
    const double max_step = rate * h;
    for(int i = 0; i < 2; i++) {
        const double d = target[i] - cur[i];
        cur[i] += (std::fabs(d) <= max_step) ? d : (d > 0.0 ? max_step : -max_step);
    }
}

/* The actuator angles that aim this engine's thrust at a SHARED lateral
   offset `lateral` (world axes; magnitude = the commanded deflection, rad):
   project it onto the engine's gimbal plane. To first order the achieved
   deviation is exactly that projection, so every engine on the ship pushes
   the SAME way instead of around its own ring. */
inline void gimbalTargets(const glm::dmat3 &partRot, const glm::dvec3 &lateral,
                          double out[2]) {
    out[0] = -glm::dot(lateral, partRot[1]);   // pitch: deviates along -local Y
    out[1] =  glm::dot(lateral, partRot[0]);   // yaw:   deviates along +local X
}

/* The attitude-authority term of a gimballed engine: the torque its thrust
   would produce at lever `rel` (from the COM). Staged: the autopilot's
   authority budget (mixing TVC with wheel torque) is its first consumer. */
inline glm::dvec3 gimbalTorque(const glm::dvec3 &rel, double thrust,
                               const glm::dvec3 &dir) {
    return glm::cross(rel, dir * thrust);
}
