#pragma once

#include <string>
#include <vector>

#include <glm/glm.hpp>

struct TerrainBody;

struct Frame {
    std::string name;

    Frame *parent; /* NULL if root */
    TerrainBody *body;
    std::vector<Frame *> children;
    bool rotating;
    Frame *rot_frame = nullptr; /* the body's spin frame; null if none. */

    double soi; // sphere of influence

    /* relative to parent */
    glm::dvec3 pos;
    glm::dvec3 vel;
    glm::dmat3 initial_orient = glm::dmat3(1.0);
    glm::dmat3 orient = glm::dmat3(1.0);
    /* Epoch (t = 0) orbital state relative to the parent, in the local
       orbital plane (XZ, prograde +Y x r_hat -- the body-rail convention;
       see railStateFromElements). UpdateOrbitRails derives pos/vel from
       it with propagateKepler. Zero for a non-orbiting frame, where
       `pos` is its fixed offset instead. */
    glm::dvec3 orbit_pos0;
    glm::dvec3 orbit_vel0;
    double orb_ang_speed;
    double parent_mu; // gravitational parameter of the body orbited (0 =
                      // non-orbiting); the rail propagates under this
    double rot_ang_speed;
    // Spin axis in local (body) frame. Always (0,1,0): a tilted body carries
    // the tilt in initial_orient so the pole stays the spin axis.
    glm::dvec3 spin_axis = glm::dvec3(0.0, 1.0, 0.0);

    // Non-rotating frame: `orient` holds the orbital-plane tilt (identity =
    // coplanar). Rotating frame: `orient` is the spin, `pos` is 0.
    /* relative to universe root (i.e. the sun) */
    glm::dvec3 root_pos;
    glm::dvec3 root_vel;
    glm::dmat3 root_orient = glm::dmat3(1.0);

    void UpdateRootRelative();
    void UpdateOrbitRails(double time);

    /* Origin state of THIS frame in relTo's OWN local axes. */
    glm::dvec3 GetVelocityRelTo(Frame *relTo);
    glm::dvec3 GetPositionRelTo(Frame *relTo);
    glm::dmat3 GetOrientRelTo(Frame *relTo);

    /* Model matrix for a body's meshes (authored in getRotFrame() axes).
       Both halves must be relativized to relTo (issue #27). */
    glm::dmat4 GetBodyDrawTransform(Frame *relTo);

    bool isRotFrame() { return rotating; }
    bool hasRotFrame() { return rot_frame != nullptr; }
    Frame *getNonRotFrame() {
        if(isRotFrame() == true) {
            return parent;
        } else {
            return this;
        }
    }

    /* The body's own rotating frame (the spin frame). Returns `this` when
       the frame has no spin frame of its own. */
    Frame *getRotFrame() { return rot_frame ? rot_frame : this; }

    // A ship at (pos, vel) in this frame has inertial velocity
    //   root_orient * (vel + GetStasisVelocity(pos)) + root_vel.
    // omega = rot_ang_speed * spin_axis (positive spin is prograde, the
    // +Y x r_hat orbital sense -- issue #101). Frame switching F -> N:
    //   v_N = O(F,N) * (v_F + stasis_F(p_F)) + Vrel(F,N) - stasis_N(p_N)
    // (old frame's stasis added, new frame's subtracted).
    glm::dvec3 GetStasisVelocity(const glm::dvec3& pos) {
        return glm::cross(rot_ang_speed * spin_axis, pos);
    }

    // Fictitious (Coriolis + centrifugal) acceleration for a ship integrated
    // IN this rotating frame so its INERTIAL trajectory stays a Kepler orbit:
    //   v' = gravity - 2*omega x v - omega x (omega x p).
    // Zero for non-rotating frames.
    glm::dvec3 GetFictitiousAccel(const glm::dvec3 &pos, const glm::dvec3 &vel) {
        const glm::dvec3 omega = rot_ang_speed * spin_axis;
        return -2.0 * glm::cross(omega, vel) - glm::cross(omega, glm::cross(omega, pos));
    }
};
