#include <numbers>
#include "frame.h"
#include "orbit.h"

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/gtx/transform.hpp>

glm::dvec3 Frame::GetVelocityRelTo(Frame *relTo)
{
    if (this == relTo) return glm::dvec3(0, 0, 0);
    /* root_vel lives in UNIVERSE axes; the result must be in relTo's OWN axes. */
    return (root_vel - relTo->root_vel) * relTo->root_orient;
}

glm::dvec3 Frame::GetPositionRelTo(Frame *relTo)
{
    /* Universe-axis difference expressed in relTo's own axes. */
    return (root_pos - relTo->root_pos) * relTo->root_orient;
}

glm::dmat3 Frame::GetOrientRelTo(Frame *relTo)
{
    if (this == relTo) return glm::dmat3(1.0);
    return glm::transpose(relTo->root_orient) * root_orient;
}

glm::dmat4 Frame::GetBodyDrawTransform(Frame *relTo)
{
    Frame *rot = getRotFrame();
    return glm::translate(rot->GetPositionRelTo(relTo))
         * glm::dmat4(rot->GetOrientRelTo(relTo));
}

void Frame::UpdateRootRelative() {
    if(parent == NULL) {
        return;
    }

    root_pos = parent->root_orient * orient * pos + parent->root_pos;
    root_vel = parent->root_orient * orient * vel + parent->root_vel;
    root_orient = parent->root_orient * orient;
}

void Frame::UpdateOrbitRails(double time) {
    if(parent != NULL and not rotating) {
        // Propagate the epoch state on the true Kepler conic. Absolute sim
        // time -- must NOT scale with the timestep or the frame snaps when
        // time accel changes.
        if(orb_ang_speed != 0) {
            propagateKepler(orbit_pos0, orbit_vel0, parent_mu,
                            time, pos, vel);
        }
    }

    if(rotating) {
        // Spin angle from accumulated sim time (not the current timestep).
        // Unconditional rebuild: orient is a pure function of time; skipping
        // at ang == 0 would leave a stale epoch.
        const double ang = fmod(rot_ang_speed * time, 2 * std::numbers::pi);
        orient = initial_orient * glm::dmat3(glm::rotate(-ang, spin_axis));
    }

    UpdateRootRelative();

    for(Frame *child : children) {
        child->UpdateOrbitRails(time);
    }
}
