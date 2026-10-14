#pragma once

#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "orbit.h"

struct TerrainBody;

struct Frame {
    std::string name;

    /* Defaults, not loader-set (#179): nothing ever deletes these, and a frame
       whose rails were never walked must report the origin, not heap garbage --
       getNonRotFrame follows parent, the relativize helpers read root_pos/root_vel. */
    Frame *parent = nullptr;   // null if root
    TerrainBody *body = nullptr;
    std::vector<Frame *> children;
    bool rotating = false;
    Frame *rot_frame = nullptr; /* the body's spin frame; null if none. */

    double soi = 0.0; // sphere of influence

    /* The instant `pos`/`vel`/`orient`/`root_*` describe: the time of the last
       UpdateOrbitRails() on the TREE ROOT -- a subtree call would desync the
       tree. Rails are a pure function of the clock, so this is the epoch they
       came from, written down so a caller composing a ship's rail state can CHECK
       it matches (Vehicle::rail_epoch). */
    double rail_time = 0.0;

    /* relative to parent */
    glm::dvec3 pos = glm::dvec3(0.0);
    glm::dvec3 vel = glm::dvec3(0.0);
    glm::dmat3 initial_orient = glm::dmat3(1.0);
    glm::dmat3 orient = glm::dmat3(1.0);
    /* Epoch (t = 0) orbital state relative to the parent, in the local orbital
       plane (XZ, prograde +Y x r_hat -- the body-rail convention; see
       railStateFromElements). UpdateOrbitRails derives pos/vel from it with
       propagateKepler. Zero for a non-orbiting frame, where `pos` is a fixed
       offset instead. */
    glm::dvec3 orbit_pos0 = glm::dvec3(0.0);
    glm::dvec3 orbit_vel0 = glm::dvec3(0.0);
    /* Magnitudes, always >= 0 (load_system rejects negative rates, #139): the
       retrograde sense lives in the orientation -- orb_incl > pi/2 flips the
       orbital plane, axial_tilt > pi/2 flips the pole. */
    double orb_ang_speed = 0.0;
    double parent_mu = 0.0; // gravitational parameter of the body orbited (0 =
                            // non-orbiting); the rail propagates under this
    double rot_ang_speed = 0.0;
    // Spin axis in local (body) axes. Always (0,1,0): a tilted body carries the
    // tilt in initial_orient, so the pole stays the spin axis.
    glm::dvec3 spin_axis = glm::dvec3(0.0, 1.0, 0.0);

    // Loader-set on ROTATING frames: the tilt part of initial_orient
    // (railAz(tilt_azimuth) * Rz(axial_tilt), WITHOUT the spin_phase0
    // pre-rotation) -- the body's equator frame. #147: equator-referred child
    // rails compose under THIS, not initial_orient: a moon's node/anomaly
    // origin is an inertial direction, not the parent's prime meridian.
    glm::dmat3 equator_orient = glm::dmat3(1.0);

    // Non-rotating frame: `orient` holds the orbital-plane tilt (identity =
    // coplanar). Rotating frame: `orient` is the spin, `pos` is 0.
    /* relative to universe root (i.e. the sun) */
    glm::dvec3 root_pos = glm::dvec3(0.0);
    glm::dvec3 root_vel = glm::dvec3(0.0);
    glm::dmat3 root_orient = glm::dmat3(1.0);

    void UpdateRootRelative();
    void UpdateOrbitRails(double time);

    /* Origin state of THIS frame in relTo's OWN local axes. */
    glm::dvec3 GetVelocityRelTo(Frame *relTo);
    glm::dvec3 GetPositionRelTo(Frame *relTo);
    glm::dmat3 GetOrientRelTo(Frame *relTo);

    /* Model matrix for a body's meshes (authored in getRotFrame() axes).
       Both halves must be relativized to relTo (#27). */
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

    /* The body's spin frame. Returns `this` when it has none of its own. */
    Frame *getRotFrame() { return rot_frame ? rot_frame : this; }

    /* The body's spin axis (normal of its equatorial plane) in relTo's axes. The
       ONE spelling of "the body's pole": an Equatorial map plane is this, NOT the
       frame's +Y, which is the normal of the plane the body's own RAIL lives in
       (#173). `orient` and `initial_orient` agree; we take orient because
       GetOrientRelTo already relativizes. An untilted rot frame returns relTo's
       +Y (the system plane) -- every shipped star lands here. */
    glm::dvec3 spinAxisRelTo(Frame *relTo);

    // Spin angular velocity in LOCAL (body) axes. Positive rot_ang_speed is
    // prograde (the +Y x r_hat orbital sense, #101); spin_axis is unit. The ONE
    // place the sense is encoded: orient integration, stasis, fictitious accel
    // and air co-rotation must agree.
    glm::dvec3 omega() const { return rot_ang_speed * spin_axis; }

    // A ship at (pos, vel) in this frame has inertial velocity
    //   root_orient * (vel + GetStasisVelocity(pos)) + root_vel.
    // Frame switching F -> N:
    //   v_N = O(F,N) * (v_F + stasis_F(p_F)) + Vrel(F,N) - stasis_N(p_N)
    // (old frame's stasis added, new frame's subtracted).
    glm::dvec3 GetStasisVelocity(const glm::dvec3& pos) {
        return glm::cross(omega(), pos);
    }

    // Fictitious (Coriolis + centrifugal) acceleration for a ship integrated
    // IN this rotating frame so its INERTIAL trajectory stays a Kepler orbit:
    //   v' = gravity - 2*omega x v - omega x (omega x p).
    // Zero for non-rotating frames.
    glm::dvec3 GetFictitiousAccel(const glm::dvec3 &pos, const glm::dvec3 &vel) {
        const glm::dvec3 w = omega();
        return -2.0 * glm::cross(w, vel) - glm::cross(w, glm::cross(w, pos));
    }
};

/* Which plane the UI measures Inc/LAN/LPe against (#171). The SAME choice the
   orbit map draws (Game::map_plane), so the numbers describe the picture;
   kRefOrbit is view-only, where Inc is 0 by construction and the node is
   undefined; kRefTarget is the transfer target's orbit, where Inc is the plane
   change to the target and the node line is where to burn for it. */
enum RefPlaneMode { kRefEquator = 0, kRefEcliptic = 1, kRefOrbit = 2,
                    kRefTarget = 3 };

/* The reference plane of `focus` for a ship orbiting it, in the focus's
   INERTIAL frame -- the frame a ship's orbit_pos/orbit_vel live in.
   `h_hat` = normalize(orbit_pos x orbit_vel), used only by kRefOrbit.
   `target_hat` = the transfer target's orbit normal about the SAME focus body
   (the target list is bodies orbiting the body you orbit, plus other vessels in
   its SOI), used only by kRefTarget.
   With no target -- a degenerate vector -- kRefTarget measures against the
   ecliptic instead and records that in RefPlane::mode, so nothing prints "tgt"
   next to an ecliptic angle. */
RefPlane uiRefPlane(Frame *focus, const glm::dvec3 &h_hat,
                    const glm::dvec3 &target_hat, int mode);

// Display name of a RefPlaneMode: an inclination without its plane is a guess,
// not a number. Three letters to fit the ORBITAL window's column.
const char *refPlaneName(int mode);

