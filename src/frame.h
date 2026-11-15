#pragma once

#include <functional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "orbit.h"

struct TerrainBody;
struct StaticBuilding;

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

    /* Total angular velocity relative to the ROOT, in LOCAL axes: own spin
       plus every ancestor's, composed in UpdateRootRelative. Equals omega()
       for body frames (their ancestors are inertial); a building frame under
       a spinning body carries both. Every stasis / angular-velocity /
       fictitious reader uses THIS -- omega() alone is only the spin relative
       to the parent, which was the whole story only while the tree was two
       levels deep (#850). */
    glm::dvec3 omega_total = glm::dvec3(0.0);
    /* The origin's inertial acceleration, ROOT axes. Zero for body frames
       (patched conics: the body's own orbit about its parent is not felt --
       the same convention as gravity ignoring the sun). A building frame
       carries its pose law's centripetal acceleration plus the parent-spin
       transport; the -A term it feeds is what makes a ship at rest in an
       orbital station's frame weightless. */
    glm::dvec3 root_accel = glm::dvec3(0.0);
    /* The body's centre in LOCAL axes. Zero for body frames (origin ==
       centre); a building frame's gravity points HERE, not at the origin. */
    glm::dvec3 gravity_center = glm::dvec3(0.0);

    /* Non-null for a BUILDING frame (#850): the building whose pose law
       drives this node. The node is a child of one of the body's own frames
       (rot for air, inertial for orbital), body points at the parent body,
       and rot_frame at the body's spin frame -- so getNonRotFrame() walks to
       the body's inertial node and every body-centred consumer (conics,
       terrain bands, the HUD) keeps working. */
    StaticBuilding *building = nullptr;
    /* The pose law hook, called by UpdateOrbitRails so a building clocks at
       the tree's instant for EVERY caller. std::function keeps frame.cpp
       free of terrain.cpp's dependency chain: the headless frame tests link
       frame.o alone. Set iff building != nullptr. */
    std::function<void(Frame *, double)> driven_tick;

    void UpdateRootRelative();
    void UpdateOrbitRails(double time);
    /* Driven-frame state from the pose law, in PARENT axes (the law's own
       output): compose root_* and the accumulated quantities. pos/vel are
       stored in local axes for direct readers; root_* is authoritative.
       The parent must already be clocked at the same instant. */
    void SetDrivenState(const glm::dvec3 &p_parent, const glm::dvec3 &v_parent,
                        const glm::dvec3 &a_parent, const glm::dmat3 &orient_local,
                        const glm::dvec3 &omega_own_local);

    /* Origin state of THIS frame in relTo's OWN local axes. */
    glm::dvec3 GetVelocityRelTo(Frame *relTo);
    glm::dvec3 GetPositionRelTo(Frame *relTo);
    glm::dmat3 GetOrientRelTo(Frame *relTo);

    /* Model matrix for a body's meshes (authored in getRotFrame() axes).
       Both halves must be relativized to relTo (#27). */
    glm::dmat4 GetBodyDrawTransform(Frame *relTo);

    bool isRotFrame() { return rotating; }
    bool hasRotFrame() { return rot_frame != nullptr; }
    /* True when a ship in this frame needs fictitious forces: a body spin
       frame, or a driven building frame -- which accelerates even when its
       total omega is zero (a sky-fixed station is in free fall), so gating
       on omega alone would be wrong. */
    bool isNonInertial() const { return rotating || building != nullptr; }
    Frame *getNonRotFrame() {
        if(isRotFrame() == true) {
            return parent;
        }
        if(building != nullptr) {
            /* A building's parent is one of its body's own frames: walk to
               the body's inertial node, so conic/band consumers lift state
               about the BODY, not the building (#850). */
            return parent->getNonRotFrame();
        }
        return this;
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
    // The stasis uses the TOTAL omega: valid for a moving origin too, since
    // root_vel already carries the origin's own motion (#850).
    glm::dvec3 GetStasisVelocity(const glm::dvec3& pos) {
        return glm::cross(omega_total, pos);
    }

    // Fictitious acceleration for a ship integrated IN this non-inertial
    // frame so its INERTIAL trajectory stays physical:
    //   a_fict = -A - 2*Omega x v - Omega x (Omega x p)
    // with A the origin's inertial acceleration (root_accel, in local axes)
    // and Omega the total spin. -A is what makes a ship at rest in an orbital
    // station's frame weightless: the station's centripetal acceleration
    // cancels gravity at the origin. For a body spin frame A = 0 and this is
    // the old Coriolis + centrifugal pair. Zero for inertial frames.
    // The Euler term (dOmega/dt x p) is absent: every driven pose law holds
    // omega_total CONSTANT in local axes (circular LVLH, fixed-axis spin,
    // zonal drift at fixed latitude). An eccentric body-fixed station would
    // vary its orbit rate -- parse refuses that combination (#850).
    glm::dvec3 GetFictitiousAccel(const glm::dvec3 &pos, const glm::dvec3 &vel) {
        const glm::dvec3 w = omega_total;
        const glm::dvec3 a = glm::transpose(root_orient) * root_accel;
        return -a - 2.0 * glm::cross(w, vel) - glm::cross(w, glm::cross(w, pos));
    }
};

/* How much slack a frame-switch identity gets (#489). Both sides of the
   Vehicle::moveToFrame checks are ROOT-space quantities, so their absolute error
   grows with the distance from the star: at the Kerbin->Kerbol offset of
   1.36e10 m one ulp is 1.907e-6 m, which is finer than the flat 1e-6 these checks
   used to demand. Out there a flat epsilon is not "a micron of slack", it is
   bit-identical-or-abort, and the hull check aborted on exactly one bit.
   The scale handed in is |p0|, the root distance, so kContinuityAbs -- what the
   checks always demanded -- only binds within ~1e6 m of the tree root, which in a
   star-rooted system is nowhere a ship ever is. That is deliberate, because none
   of these identities is exactly zero in the field (HEAD, a same-body Kerbin
   switch: comErr 0, velErr 0, hullErr 1.16415322e-10; pre-#483, the Kerbol switch:
   hullErr 1.90736318e-06 against an ulp of 1.90734863e-06). kContinuityRel = 1e-12
   leaves ~4.5e3 ulp for a single rounding (7160 measured at 1.36e10 m) while a
   Kerbol-distance switch still only buys 1.4 cm of slack -- every defect worth
   catching (a dropped stasis term, a wrong frame or part offset, a drifted
   root_orient) misses by metres or kilometres, never centimetres. */
constexpr double kContinuityAbs = 1e-6;
constexpr double kContinuityRel = 1e-12;
inline double continuityTol(double scale) {
    const double rel = kContinuityRel * scale;
    return rel > kContinuityAbs ? rel : kContinuityAbs;
}

/* Which plane the UI measures Inc/LAN/LPe against (#171). The SAME choice the
   orbit map draws (Game::map.plane), so the numbers describe the picture;
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

