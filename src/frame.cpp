#include <cassert>
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

RefPlane uiRefPlane(Frame *focus, const glm::dvec3 &h_hat,
                    const glm::dvec3 &target_hat, int mode)
{
    RefPlane r;
    r.mode = mode;
    // The system plane in the focus's axes (the Ecliptic map's normal). Also the
    // fallback for a slot that cannot resolve, which is why it stamps the mode.
    auto ecliptic = [&](RefPlane &p) {
        p.n_hat = glm::transpose(focus->root_orient) * glm::dvec3(0.0, 1.0, 0.0);
        p.x_hat0 = glm::transpose(focus->root_orient) * glm::dvec3(1.0, 0.0, 0.0);
        p.mode = kRefEcliptic;
    };
    if(mode == kRefEcliptic) {
        ecliptic(r);
    } else if(mode == kRefOrbit) {
        // The ship's own plane: Inc is 0 by construction, the node undefined.
        // No zero longitude either -- a default x_hat0 would print a LPe
        // measured from an axis nobody chose.
        r.n_hat = h_hat;
        r.has_zero = false;
    } else if(mode == kRefTarget) {
        /* Producers pass either a unit normal or nothing at all; the 0.25 (i.e.
           |n| > 0.5) is a "was one supplied?" test with slack for a scaled
           vector, not a conditioning choice. */
        if(glm::dot(target_hat, target_hat) > 0.25) {
            /* The TARGET's plane about the same focus: Inc here is the plane
               change to the target's orbit and the node line is where to spend
               it. No zero longitude, same as kRefOrbit -- the target's own
               ascending node is not an axis this ship's longitude should
               measure from, and a plane change needs the node, not a longitude. */
            r.n_hat = glm::normalize(target_hat);
            r.has_zero = false;
        } else {
            // No target (or one with no orbit): measure against the ecliptic and
            // say so, rather than printing an ecliptic angle labelled "tgt".
            ecliptic(r);
        }
    } else {
        // The focus's EQUATOR, longitude from the equator frame's +X -- what an
        // "incl_ref": "equator" rail measures lon_asc_node from (#147), so
        // readout and authored data agree.
        r.n_hat = focus->spinAxisRelTo(focus);
        r.x_hat0 = focus->getRotFrame()->equator_orient * glm::dvec3(1.0, 0.0, 0.0);
    }
    return r;
}

const char *refPlaneName(int mode)
{
    if(mode == kRefEcliptic) { return "ecl"; }
    if(mode == kRefOrbit) { return "orb"; }
    if(mode == kRefTarget) { return "tgt"; }
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
        omega_total = omega();
        return;
    }

    root_pos = parent->root_orient * orient * pos + parent->root_pos;
    /* The origin rides the parent's spin too: a point at orient*pos in the
       parent moves with omega_parent x (orient*pos). Zero for every body
       frame (orbiting frames have inertial parents; rotating frames sit at
       pos = 0), load-bearing for a building frame under a spinning body --
       without it a bubble handoff is off by the parent's surface speed. */
    root_vel = parent->root_vel
             + parent->root_orient * (orient * vel
                 + glm::cross(parent->omega_total, orient * pos));
    root_orient = parent->root_orient * orient;
    omega_total = omega() + glm::transpose(orient) * parent->omega_total;
    /* Body frames: patched conics -- the origin's own orbit is not felt, and
       the origin IS the body centre. A frame parented UNDER a building bubble
       would need the composed accel/centre instead; none exists yet, and the
       driven branch recurses into children, so say so loudly (#850). */
    assert(parent->building == nullptr
           && "a frame under a building bubble must compose accel/gravity centre");
    root_accel = glm::dvec3(0.0);
    gravity_center = glm::dvec3(0.0);
}

void Frame::SetDrivenState(const glm::dvec3 &p_parent, const glm::dvec3 &v_parent,
                           const glm::dvec3 &a_parent, const glm::dmat3 &orient_local,
                           const glm::dvec3 &omega_own_local) {
    assert(parent != nullptr);
    orient = orient_local;
    pos = glm::transpose(orient) * p_parent;
    vel = glm::transpose(orient) * v_parent;
    root_pos = parent->root_pos + parent->root_orient * p_parent;
    root_vel = parent->root_vel
             + parent->root_orient * (v_parent
                 + glm::cross(parent->omega_total, p_parent));
    root_orient = parent->root_orient * orient;
    omega_total = omega_own_local + glm::transpose(orient) * parent->omega_total;
    /* The origin's acceleration: the law's own, plus the parent-spin
       transport of its relative motion (Coriolis + centrifugal). The parent
       Euler term is zero: body spin rates are constant. */
    root_accel = parent->root_accel
               + parent->root_orient * (a_parent
                   + 2.0 * glm::cross(parent->omega_total, v_parent)
                   + glm::cross(parent->omega_total,
                                glm::cross(parent->omega_total, p_parent)));
    /* The parent is one of the body's own frames, origin at the body centre:
       in local axes the centre sits at -pos. */
    gravity_center = -pos;
}

void Frame::UpdateOrbitRails(double time) {
    rail_time = time;   // everything below, and every child, is f(time)
    if(driven_tick) {
        /* Building frame: the pose law (terrain.cpp) writes the state at this
           instant through SetDrivenState. The walk is parent-first, so the
           parent is already clocked. */
        driven_tick(this, time);
        for(Frame *child : children) {
            child->UpdateOrbitRails(time);
        }
        return;
    }
    if(parent != NULL and not rotating) {
        // Propagate the epoch state on the true Kepler conic. Absolute sim
        // time -- must NOT scale with the timestep, or the frame snaps when
        // time accel changes.
        if(orb_ang_speed != 0) {
            propagateKepler(orbit_pos0, orbit_vel0, parent_mu,
                            time, pos, vel);
        }
    }

    if(rotating) {
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

void switchFrameState(Frame *from, Frame *to, const glm::dvec3 &p,
                      const glm::dvec3 &v, glm::dvec3 &pt, glm::dvec3 &vt) {
    /* The new frame's stasis term needs the RESULTING position, so pt is written
       before vt finishes -- an input that aliases an output would then be read
       too late. Callers keep them separate; assert rather than comment. */
    assert(&p != &pt && &v != &vt);
    const glm::dmat3 Fo = from->GetOrientRelTo(to);
    pt = Fo * p + from->GetPositionRelTo(to);
    if(from == to) { vt = v; return; }
    vt = Fo * (v + from->GetStasisVelocity(p)) + from->GetVelocityRelTo(to)
       - to->GetStasisVelocity(pt);
}

glm::dvec3 switchFrameSpin(Frame *from, Frame *to, const glm::dvec3 &w) {
    return from->GetOrientRelTo(to) * (w + from->omega_total) - to->omega_total;
}
