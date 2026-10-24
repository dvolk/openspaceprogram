#pragma once

// gimbal.h -- the engine-gimbal (TVC) law as pure math (glm only), so tests/
// can pin it without the render/physics chain (like drag.h, orbit.h).
//
// A universal-joint gimbal rotates the thrust vector off the part's authored
// +Z: pitch about the part's local X, yaw about its local Y (the OUTER ring,
// applied after the inner one). The deviation force applied at the engine's
// offset from the COM is what torques the ship -- see Vehicle::applyThrustForce.
//
// Sign convention (pinned in tests/test_gimbal.cpp): for an engine stacked
// along the ship's axis, target angles {stick[1], stick[2]} * gimbal_max give
// the SAME nose rotation sign as the reaction-wheel torque in
// Vehicle::applyRotationForce -- W/S and A/D feel the same with wheels, TVC,
// or both.

#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

/* The gimballed thrust direction (unit) in WORLD axes: the part's authored +Z
   rotated by `pitch` about its local X, then `yaw` about its local Y (rad).
   Zero angles return the authored axis untouched. */
inline glm::dvec3 gimbalDir(const glm::dmat3 &partRot, double pitch, double yaw) {
    if(pitch == 0.0 && yaw == 0.0) { return partRot[2]; }
    const glm::dquat q = glm::angleAxis(yaw, partRot[1])
                       * glm::angleAxis(pitch, partRot[0]);
    return glm::normalize(q * partRot[2]);
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

/* The attitude-authority term of a gimballed engine: the torque its thrust
   would produce at lever `rel` (from the COM). Exposed so the attitude law
   (autopilot slews) can budget TVC alongside wheel torque. */
inline glm::dvec3 gimbalTorque(const glm::dvec3 &rel, double thrust,
                               const glm::dvec3 &dir) {
    return glm::cross(rel, dir * thrust);
}
