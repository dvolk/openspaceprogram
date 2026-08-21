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
    bool has_rot_frame;

    double soi; // sphere of influence

    /* relative to parent */
    glm::dvec3 pos;
    glm::dvec3 initial_pos;
    // GLM 1.0.0+: default-constructed matrices are zero, so default these to
    // the identity explicitly (matches the pre-1.0 glm behaviour these
    // members relied on).
    glm::dmat3 initial_orient = glm::dmat3(1.0);
    glm::dmat3 orient = glm::dmat3(1.0);
    glm::dvec3 vel;
    double orb_ang_speed;
    double rot_ang_speed;
    // Spin axis in this frame's local (body) frame. (0,1,0) = no axial tilt
    // (the body spins about the orbital normal — the historical convention).
    // Overridden from the axial tilt when the system is loaded; drives the
    // spin, stasis velocity and fictitious forces. Irrelevant for a
    // non-spinning frame. Defaulted so any Frame created without an explicit
    // value keeps the old pure-Y behaviour.
    glm::dvec3 spin_axis = glm::dvec3(0.0, 1.0, 0.0);

    // For a non-rotating (inertial) frame, `orient` holds the orbital-plane
    // tilt relative to the parent (line of nodes along the parent's X axis;
    // identity = coplanar). `pos` then lives in the local orbital plane and
    // UpdateRootRelative carries it into the parent via `orient`. For a
    // rotating frame `orient` is the spin and `pos` is 0, so `orient` has no
    // effect on root_pos there.
    /* relative to universe root (i.e. the sun) */
    glm::dvec3 root_pos;
    glm::dvec3 root_vel;
    glm::dmat3 root_orient = glm::dmat3(1.0);

    double ang;
    double orb_ang;

    void UpdateRootRelative(double time, double timestep);
    void UpdateOrbitRails(double time, double timestep);

    glm::dvec3 GetVelocityRelTo(Frame *relTo);
    glm::dvec3 GetPositionRelTo(Frame *relTo);
    glm::dmat3 GetOrientRelTo(Frame *relTo);

    bool isRotFrame() { return rotating; }
    bool hasRotFrame() { return has_rot_frame; }
    Frame *getNonRotFrame() {
        if(isRotFrame() == true) {
            return parent;
        } else {
            return this;
        }
    }

    Frame *getRotFrame() {
        if(hasRotFrame() == true) {
            return children.front();
        }
        else {
            return this;
        }
    }
    // A ship at (pos, vel) in this frame has inertial (root-frame) velocity
    //   root_orient * (vel + GetStasisVelocity(pos)) + root_vel
    // (verified against the frame rotation in UpdateOrbitRails:
    // orient = initial_orient * rotate(-ang, spin_axis)).
    // The frame's angular velocity in its own local frame is
    //   omega = -rot_ang_speed * spin_axis
    // (spin_axis is the spin axis in local coords; (0,1,0) for no axial tilt,
    // which reproduces the old pure-Y convention). Consequences for frame
    // switching F -> N:
    //   v_N = O(F,N) * (v_F + stasis_F(p_F)) + Vrel(F,N) - stasis_N(p_N)
    // i.e. the OLD frame's stasis term is added, the NEW frame's subtracted.
    // (A ship needing velocity -GetStasisVelocity(pos) to be inertially
    // stationary is a useful mnemonic for the signs.)

    glm::dvec3 GetStasisVelocity(const glm::dvec3& pos) {
        return glm::cross(-rot_ang_speed * spin_axis, pos);
    }

    // Fictitious (Coriolis + centrifugal) acceleration that a ship integrated
    // IN this (rotating) frame must additionally feel, on top of gravity, so
    // that its INERTIAL trajectory stays exactly the Kepler orbit the identity
    // above describes.  With omega = -rot_ang_speed * spin_axis
    // (i.e. stasis(p) == omega x p), differentiating
    //   v_root = R * (v + stasis(p)) + V
    // gives  v_root' = R * (v' + 2*omega x v + omega x (omega x p)),
    // so for v_root' == R * gravity the frame integration must use
    //   v' = gravity - 2*omega x v - omega x (omega x p).
    // Without this, time spent in a rotating frame perturbs the true orbit
    // (Coriolis is ~2*w*v, up to ~20% of gravity at low orbits).
    // Zero for non-rotating frames.
    glm::dvec3 GetFictitiousAccel(const glm::dvec3 &pos, const glm::dvec3 &vel) {
        const glm::dvec3 omega = -rot_ang_speed * spin_axis;
        return -2.0 * glm::cross(omega, vel) - glm::cross(omega, glm::cross(omega, pos));
    }
};
