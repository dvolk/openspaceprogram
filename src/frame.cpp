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

glm::dvec3 Frame::spinAxisRelTo(Frame *relTo)
{
    Frame *rf = getRotFrame();
    return glm::normalize(rf->GetOrientRelTo(relTo) * rf->spin_axis);
}

RefPlane uiRefPlane(Frame *focus, const glm::dvec3 &h_hat, int mode)
{
    RefPlane r;
    if(mode == kRefEcliptic) {
        // The system plane, in the focus's axes -- the same normal the
        // Ecliptic map view projects onto.
        r.n_hat = glm::transpose(focus->root_orient) * glm::dvec3(0.0, 1.0, 0.0);
        r.x_hat0 = glm::transpose(focus->root_orient) * glm::dvec3(1.0, 0.0, 0.0);
    } else if(mode == kRefOrbit) {
        // The ship's own plane. Inc is 0 by construction and the node is
        // undefined; and there is no zero longitude either -- every in-plane
        // direction is equally arbitrary, so keeping the default x_hat0 here
        // would print a LPe measured from an axis nobody chose.
        r.n_hat = h_hat;
        r.has_zero = false;
    } else {
        // The focus's EQUATOR: its spin axis, with longitude from the
        // equator frame's +X -- the direction an "incl_ref": "equator" rail
        // measures its lon_asc_node from (#147), so the readout and the
        // authored data agree.
        r.n_hat = focus->spinAxisRelTo(focus);
        r.x_hat0 = focus->getRotFrame()->equator_orient * glm::dvec3(1.0, 0.0, 0.0);
    }
    return r;
}

const char *refPlaneName(int mode)
{
    if(mode == kRefEcliptic) { return "ecl"; }
    if(mode == kRefOrbit) { return "orb"; }
    return "equ";
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
    rail_time = time;   // everything below, and every child, is f(time)
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
        orient = initial_orient * glm::dmat3(glm::rotate(ang, spin_axis));
    }

    UpdateRootRelative();

    for(Frame *child : children) {
        child->UpdateOrbitRails(time);
    }
}
