#include "frame.h"

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/gtx/transform.hpp>

glm::dvec3 Frame::GetVelocityRelTo(Frame *relTo)
{
    if (this == relTo) return glm::dvec3(0, 0, 0);
    /* root_vel lives in UNIVERSE axes; the result must be in relTo's OWN
       axes, so rotate by relTo->root_orient^-1 (glm's v*M is M^T*v). This
       applies whether relTo spins (root_orient carries the spin) or is
       inertial (root_orient carries the ancestors' accumulated orbital
       tilts -- identity only while every ancestor orbit is uninclined). */
    return (root_vel - relTo->root_vel) * relTo->root_orient;
}

glm::dvec3 Frame::GetPositionRelTo(Frame *relTo)
{
    /* Universe-axis difference expressed in relTo's own axes -- see
       GetVelocityRelTo for why the rotation is unconditional. */
    return (root_pos - relTo->root_pos) * relTo->root_orient;
}

glm::dmat3 Frame::GetOrientRelTo(Frame *relTo)
{
    if (this == relTo) return glm::dmat3(1.0);
    return glm::transpose(relTo->root_orient) * root_orient;
}

void Frame::UpdateRootRelative(double time, double timestep) { // TODO unused params
    if(parent == NULL) {
        return;
    }

    root_pos = parent->root_orient * orient * pos + parent->root_pos;
    root_vel = parent->root_orient * orient * vel + parent->root_vel;
    root_orient = parent->root_orient * orient;
}

void Frame::UpdateOrbitRails(double time, double timestep) {
    if(parent != NULL and body != NULL and not rotating) {
        // translate body in orbit
        if(orb_ang_speed != 0) {
            pos = glm::dmat3(glm::rotate(orb_ang_speed * timestep, glm::dvec3(0, 1, 0))) * pos;
        }
    }

    if(rotating) {
        // total angle as a function of accumulated sim time. Must NOT scale
        // with the current timestep, or the frame (and everything in it)
        // snaps when the time acceleration changes.
        ang = fmod(rot_ang_speed * time, 2 * M_PI);
        if(ang != 0) {
            orient = initial_orient * glm::dmat3(glm::rotate(-ang, spin_axis));
        }
    }

    UpdateRootRelative(time, timestep);

    for(Frame *child : children) {
        child->UpdateOrbitRails(time, timestep);
    }
}
