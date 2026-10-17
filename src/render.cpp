// render.cpp -- the 3D pass: state half, then paint half. See render.h.
#include "render.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <numbers>
#include <cstdlib>
#include <GL/glew.h>   // glBlendFunc / glLineWidth / the GL enums

#include "billboard.h"
#include "constants.h"  // kNodeAnRgb/kNodeDnRgb (the plane overlay's node ticks)
#include "mesh.h"
#include "orbit.h"
#include "physics.h"   // debug_draw
#include "shader.h"
#include "skybox.h"
#include "surfmap.h"   // surfmapCompute (the M key's surface map)
#include "texture.h"

// GLM's gtx extensions hard-error without this.
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/projection.hpp>    // glm::proj
#include <glm/gtx/vector_angle.hpp>  // glm::orientedAngle

// Small-angle rotation (a is ~1e-3 rad): R ~= I + [a]x.
static glm::dmat3 shakeRot(const glm::dvec3 &a) {
    return glm::dmat3(1.0, a.z, -a.y,
                      -a.z, 1.0, a.x,
                      a.y, -a.x, 1.0);
}

/* Camera shake: the felt g's set an amplitude, low-passed toward random
   targets. Orbit camera on the ship only; --cam-shake scales it. */
static void camShakeStep(Game &g, Vehicle *ship) {
    double a = 0.0;
    if(g.time_accel > 0 && !ship->onRails) {
        a = ship->feltAccel();
    }
    // Steady below the threshold; above it the amplitude grows and saturates.
    const bool active = a > 1.0
        && g.args.cam_shake > 0.0
        && g.camera->mode == CAM_ORBIT
        && g.focusTargets[g.focusBody].body == nullptr;
    const double target = active
        ? (double)g.args.cam_shake * std::min(0.15, (a - 1.0) * 0.008)
        : 0.0;
    const Uint32 now_ms = SDL_GetTicks();
    // Wall-clock time constant: the rumble rate must not scale with render fps.
    const double frame_dt = (now_ms - g.shake_last_ms) * 0.001;
    g.shake_last_ms = now_ms;
    const double tau = 0.05;    // s
    const double alpha = frame_dt > 0.0 ? 1.0 - std::exp(-frame_dt / tau) : 1.0;
    const double wobble = 0.02;  // rad of basis jitter per metre of offset
    for(int i = 0; i < 3; i++) {
        const double ro = ((double)std::rand() / RAND_MAX) * 2.0 - 1.0;
        const double ra = ((double)std::rand() / RAND_MAX) * 2.0 - 1.0;
        g.shake_off[i] += alpha * (ro * target - g.shake_off[i]);
        g.shake_ang[i] += alpha * (ra * target * wobble - g.shake_ang[i]);
    }
    // --shake-log: felt accel + live amplitude at the --orbit-interval cadence.
    if(g.args.shake_log) {
        if(g.logDue(kGateShake, now_ms, g.orbit_log_interval_ms)) {
            printf("[shakelog] t=%.3fs a=%.3f m/s2 amp=%.4f m "
                   "off=[%+.4f %+.4f %+.4f]\n",
                   g.time, a, target,
                   g.shake_off.x, g.shake_off.y, g.shake_off.z);
            fflush(stdout);
        }
    }
}

// No camera state -- every scene can fill it, even the ones that paint no
// world (#235).
void updateShipView(Game &g) {
    Vehicle *ship = g.ship;
    if(ship == nullptr) { g.view.ship = nullptr; return; }
    ShipView &view = g.view;
    view.ship = ship;
    double &mu = view.mu;
    glm::dvec3 &pos = view.pos;
    glm::dvec3 &vel = view.vel;
    glm::dvec3 &orbit_pos = view.orbit_pos;
    glm::dvec3 &orbit_vel = view.orbit_vel;
    glm::dvec3 &marker_vel = view.marker_vel;
    glm::dvec3 &surf_pos = view.surf_pos;
    glm::dvec3 &surf_vel = view.surf_vel;
    OrbitElements &o = view.o;
    PlaneAngles &plane = view.plane;
    double &distance = view.distance;
    double &speed = view.speed;
    glm::dvec3 &up = view.up;
    glm::dvec3 &facing = view.facing;
    glm::dvec3 &other = view.other;
    glm::dvec3 &facing_dir = view.facing_dir;
    glm::dvec3 &vel_dir = view.vel_dir;
    double &ver_speed = view.ver_speed;
    double &hor_speed2 = view.hor_speed2;
    double &heading = view.heading;
    double &pitch = view.pitch;
    double &roll = view.roll;
    double &latitude = view.latitude;
    double &longitude = view.longitude;
    TimeSeries &energy_series = view.energy_series;
    TimeSeries &angmom_series = view.angmom_series;

    const glm::dvec3 com = ship->get_center_of_mass();

    mu = ship->m_parent->mu;
    pos = com;
    vel = ship->GetVel();

    // The orbit conic lives in the body's INERTIAL (non-rotating) frame -- what
    // the spawn/switching code targets.
    orbit_pos = pos;
    orbit_vel = vel;
    marker_vel = vel;
    if(ship->frame->isRotFrame() == true) {
        Frame *inertial = ship->frame->getNonRotFrame();
        const glm::dmat3 to_inertial = ship->frame->GetOrientRelTo(inertial);
        orbit_vel += ship->frame->GetStasisVelocity(orbit_pos);
        orbit_vel = to_inertial * orbit_vel + ship->frame->GetVelocityRelTo(inertial);
        orbit_pos = to_inertial * orbit_pos + ship->frame->GetPositionRelTo(inertial);
        // And back into this frame, so the flight markers can point along the
        // conic while being drawn in the frame they live in.
        marker_vel = glm::transpose(to_inertial) * orbit_vel;
    }

    // Surface-relative state: position/velocity in the ROTATING frame.
    surf_pos = pos;
    surf_vel = vel;

    if(ship->frame->isRotFrame() == false and
       ship->frame->hasRotFrame() == true) {
        Frame *rot = ship->frame->getRotFrame();
        surf_pos = ship->frame->GetOrientRelTo(rot) * pos;
        surf_vel = ship->frame->GetOrientRelTo(rot) * vel
                 - rot->GetStasisVelocity(surf_pos);
    }

    o = computeOrbitElements(orbit_pos, orbit_vel, mu);
    /* Plane angles measured in the plane the orbit map is drawing, so the
       numbers match the picture (#171). The plane itself is stored too: the 3D
       orbital-plane overlay draws this one, so the rings and the readout cannot
       describe different planes. The target's normal comes from the planner, so
       the map's "Target orbit" slot is the same target the transfer is aimed
       at -- one target, one plane, everywhere. */
    view.target_h_hat = glm::dvec3(0.0);
    view.target_plane_ok = g.xferPlanner.targetPlaneNormal(ship->m_parent->frame,
                                                           view.target_h_hat);
    view.ref_plane = uiRefPlane(ship->m_parent->frame, o.h_hat,
                                view.target_h_hat, g.map_plane);
    plane = orbitPlaneAngles(o, view.ref_plane);
    distance = o.distance;
    speed = o.speed;

    // Telemetry: conserved 2-body constants (drift = integrator error,
    // steps = burns/staging).
    energy_series.push(g.time, o.energy);
    angmom_series.push(g.time, o.ang_momentum);

    up = ship->partAxis(ship->controller, 1);
    facing = ship->partAxis(ship->controller, 2);
    other = ship->partAxis(ship->controller, 0);

    facing_dir = glm::normalize(facing);
    /* The ORBITAL window's Prg/Rtg is the angle to the CONIC's prograde, like
       the marker it sits beside -- and a grounded ship has one (the ground's own
       motion), where normalize(vel) would divide by a near-zero length. */
    vel_dir = glm::normalize(marker_vel);

    // heading/pitch/roll are SURFACE-relative: measured against the ground's
    // north/east in the rotating frame.
    glm::dvec3 surf_facing = facing;
    glm::dvec3 surf_up_axis = up;
    if(ship->frame->isRotFrame() == false and ship->frame->hasRotFrame() == true) {
        Frame *rot = ship->frame->getRotFrame();
        surf_facing = ship->frame->GetOrientRelTo(rot) * facing;
        surf_up_axis = ship->frame->GetOrientRelTo(rot) * up;
    }

    const glm::dvec3 surf_up = glm::normalize(surf_pos);
    const glm::dvec3 surf_north = glm::normalize(projectVecOntoPlane(glm::dvec3(0, 1, 0), surf_up));
    const glm::dvec3 surf_east = glm::cross(surf_up, surf_north);

    ver_speed = glm::dot(surf_vel, surf_up); // m/s, + = climbing
    hor_speed2 = glm::length(projectVecOntoPlane(surf_vel, surf_up)); // m/s

    const glm::dvec3 groundHed = glm::normalize(projectVecOntoPlane(surf_facing, surf_up));

    const double hedNorth = glm::dot(groundHed, surf_north);
    const double hedEast = glm::dot(groundHed, surf_east);
    heading = wrapAngleToPositive(atan2(hedEast, hedNorth));

    pitch = asin(glm::dot(surf_up, surf_facing));
    roll =
        glm::orientedAngle(glm::normalize(projectVecOntoPlane(-surf_pos, glm::normalize(surf_facing))),
                           glm::normalize(-surf_up_axis),
                           glm::normalize(surf_facing));

    const glm::dvec3 dir = glm::normalize(surf_pos);

    equirectLonLat(dir, longitude, latitude);
}

/* Fake star-field exposure: no HDR headroom, so the brightest stars land at the
   sun's own value. Fade the sky toward g.sky_dim while the sun is inside a cone
   of half-angle g.sky_dim_cone around the view axis (a fixed angle, not the
   frustum) AND the ship is lit. g.sky_dim >= 1 disables it. */
static float skyGain(Game &g, Frame *rf, TerrainBody *sun, Camera *camera,
                     TerrainBody *localBody, const glm::dvec3 &lit_at) {
    if(g.sky_dim >= 1.0f) { return 1.0f; }
    const glm::dvec3 cam_p = camera->GetPos();
    const glm::dvec3 sun_p = sun->frame->GetPositionRelTo(rf);
    const double sun_d = glm::length(sun_p - cam_p);
    if(sun_d <= 0.0) { return 1.0f; }   // inside the star: nothing to fade to
    const glm::dvec3 sun_dir = (sun_p - cam_p) / sun_d;

    // Smooth 1 -> 0 as x rises from lo to hi.
    auto fade = [](double lo, double hi, double x) {
        const double t = std::clamp((hi - x) / (hi - lo), 0.0, 1.0);
        return t * t * (3.0 - 2.0 * t);
    };
    // Angle to the sun's NEAR limb: a wide sun is "in view" before its centre
    // reaches the cone.
    const double axl = std::acos(std::clamp(glm::dot(sun_dir,
                                                    glm::normalize(camera->GetForward())),
                                            -1.0, 1.0));
    const double limb = axl - std::asin(std::clamp(sun->radius / sun_d, 0.0, 1.0));
    // Game stores the cone in degrees (the CLI and Settings units).
    const double cone = glm::radians((double)g.sky_dim_cone);
    const double in_view = fade(cone, cone * 2.5, limb);

    // "Lit" is the hull's own shadow ray, so the sky fades when the parts flip
    // to night. A centre-hemisphere test is NOT the visible horizon. Sample at
    // the ship's COM, not the frame origin (off-rails it sits at the parent's
    // centre).
    double lit = 1.0;
    if(localBody && localBody->frame) {
        lit = ComputeTerrainShadow(localBody, rf, lit_at, sun) > 0.5f ? 1.0 : 0.0;
    }
    // Both inputs are hard tests, so low-pass the fade. Wall-clock seconds: a
    // sim-time tau would race at high time accel.
    const double target = in_view * lit;
    const Uint32 now_ms = SDL_GetTicks();
    const double frame_dt = (now_ms - g.sky_fade_last_ms) * 0.001;
    g.sky_fade_last_ms = now_ms;
    const double tau = 0.4;
    const double alpha = frame_dt > 0.0 ? 1.0 - std::exp(-frame_dt / tau) : 1.0;
    g.sky_fade += alpha * (target - g.sky_fade);
    const double t = g.sky_fade;
    const float gain = (float)(1.0 + (g.sky_dim - 1.0) * t);
    // --sky-dim-log: the fade's inputs, tunable from the command line.
    if(g.args.sky_dim_log && g.logDue(kGateSkyDim, now_ms, 200)) {
        std::printf("[skydim] t=%.1fs sun_off=%.1f limb=%.1f in_view=%.2f "
                    "lit=%.2f fade=%.2f gain=%.3f\n", g.time,
                    axl * 180.0 / std::numbers::pi,
                    limb * 180.0 / std::numbers::pi, in_view, lit, t, gain);
        std::fflush(stdout);
    }
    return gain;
}

/* The ship-side half of the state pass: g.view, the transfer planner, the P/M
   one-shots. `com` is passed in: get_center_of_mass walks the part tree. */
static void refreshShipState(Game &g, const glm::dvec3 &com) {
    updateShipView(g);

    g.xferPlanner.update(com, g.view.vel);

    // P one-shot: set in poll_events, run here where the planner and this
    // frame's snapshot both exist.
    if(g.porkchop_compute_requested) {
        g.xferPlanner.porkchopCompute();
        g.porkchop_compute_requested = false;
    }

    // M one-shot, same pattern as P; the sweep runs on the background worker.
    if(g.surfmap_compute_requested) {
        surfmapCompute(g);
        g.surfmap_compute_requested = false;
    }
}

/* State half of the 3D pass (#212). The terrain LOD owns the ground's Bullet
   collision bodies, so skipping it changes collision outcomes, not just
   pixels (#24). */
void refreshWorld(Game &g) {
    Vehicle *ship = g.ship;
    // Render frame: the ship's frame, or the home body's when there is no ship.
    Frame *rf = ship ? ship->frame : g.home->frame;
    // The body "we're on": the LOD and the --terrain-log key on it.
    TerrainBody *localBody = ship ? ship->m_parent : g.home;
    const std::vector<TerrainBody *> &planets = g.sys.bodies;
    Camera *camera = g.camera;

    /* The orbit basis takes the ship's attitude whenever the focus is the
       "ship" entry, which exists only while a ship is active
       (Game::syncShipFocus). Invariants, not inputs to handle (#208). */
    assert(g.focusBody >= 0 && g.focusBody < (int)g.focusTargets.size());
    assert(g.focusTargets[g.focusBody].body != nullptr || ship != nullptr);

    const glm::dvec3 com = ship ? ship->get_center_of_mass() : glm::dvec3(0.0);
    // First person: eye at the controller's top (stack +Z face), along the
    // nose.
    if(camera->mode == CAM_FIRST) {
        if(ship && !ship->isEva() && ship->controller != nullptr
           && ship->controller->def != nullptr) {
            const Part *c = ship->controller;
            const glm::dvec3 nose = ship->partAxis(c, 2);
            camera->pos = ship->partPos(c) + nose * (c->def->height * 0.5);
            camera->forward = nose;
            camera->up = ship->partAxis(c, 1);
        } else {
            // No cockpit to ride (EVA / shipless): fall back to orbit.
            camera->toOrbit(g.focusWorldPos(g.focusBody));
        }
    }

    if(camera->mode == CAM_ORBIT) {
        camera->Follow(g.focusWorldPos(g.focusBody));
        // The orbit offset lives in this orientation: the ship's attitude when
        // focused on the ship, the body's rotating frame when on a body.
        if(g.focusTargets[g.focusBody].body == nullptr) {
            if(ship->isEva()) {
                // North-up on the kerbal's radial, tangent plane from the WORLD
                // axes: the frame's orient spins with the planet, and the
                // kerbal's own axes would feed back into its attitude law.
                const glm::dvec3 radial = glm::normalize(com);
                // Tangent "north": +Y, or +X near +/-Y so the projection never
                // vanishes.
                const glm::dvec3 ref_dir = (std::fabs(radial.y) > 0.9)
                    ? glm::dvec3(1.0, 0.0, 0.0) : glm::dvec3(0.0, 1.0, 0.0);
                const glm::dvec3 right = glm::normalize(
                    ref_dir - radial * glm::dot(ref_dir, radial));
                const glm::dvec3 back = glm::cross(right, radial);
                camera->ref = glm::dmat3(back, right, radial);
            } else {
            // The orbit camera expects ref as [back, right, up]; the ship's
            // axes 0/1/2 are [right, up, nose]. Feed the columns in the
            // camera's order: nose-as-up swaps the pitch/yaw responses.
            camera->ref = glm::dmat3(ship->partAxis(ship->controller, 2),   // back = nose
                                     ship->partAxis(ship->controller, 0),   // right
                                     ship->partAxis(ship->controller, 1));  // up
            }
        } else {
            TerrainBody *b = g.focusTargets[g.focusBody].body;
            camera->ref = b->frame->getRotFrame()->GetOrientRelTo(rf);
        }
    }

    // Camera shake: shift the focus point and wobble the orbit basis.
    if(ship) { camShakeStep(g, ship); }
    if(camera->mode == CAM_ORBIT && ship &&
       g.focusTargets[g.focusBody].body == nullptr) {
        camera->focusPoint += g.shake_off;
        camera->ref = shakeRot(g.shake_ang) * camera->ref;
    }

    // Render frame origin = the ship's COM, so the float32 cast works on
    // ship-relative numbers. Draw sites shift by -renderOrigin.
    camera->renderOrigin = com;
    camera->ComputeView();

    // The cache GeoPatch::Draw composes into its modelview (terrain.cpp).
    for(auto&& planet : planets) {
        planet->transform = planet->frame->GetBodyDrawTransform(rf);
    }

    for(auto&& planet : planets) {
        // LOD may post async subdivision jobs; their continuations run in
        // jobs.poll() BEFORE the paint pass draws.
        planet->Update(camera, g.args.terrain_px, g.jobs);

        /* --terrain-log: the local body's tree as the LOD just left it.
           deep_off is the camera-to-nearest-deepest-leaf distance; collision
           counts leaves with a Bullet body (#24). */
        if(g.args.terrain_log && planet == localBody && planet->ready) {
            const Uint32 now_ms = SDL_GetTicks();
            if(g.logDue(kGateTerrain, now_ms, g.orbit_log_interval_ms)) {
                const TerrainLodStats s =
                    planet->lodStats(planet->CameraInBodyFrame(camera));
                printf("[terrain] t=%.1fs body=\"%s\" patches=%d deepest=%d "
                       "max_depth=%d collision=%d deep_off=%.6g m "
                       "cam_r=%.6g m\n",
                       g.time, planet->name.c_str(), s.patches, s.deepest,
                       planet->max_depth, s.collision, s.deep_off, s.cam_r);
                fflush(stdout);
            }
        }
    }

    refreshShipState(g, com);

    /* --cam-log: the camera as this frame left it, so a --sim-mouse drag that
       never reached the camera cannot pass silently. rmb= and cap= are the
       gates in events.cpp's mouse handlers: a right-button drag only looks when
       rmb is armed AND imgui is not capturing the mouse. */
    if(g.args.cam_log) {
        const Uint32 now_ms = SDL_GetTicks();
        if(g.logDue(kGateCam, now_ms, g.orbit_log_interval_ms)) {
            const char *mode = camera->mode == CAM_ORBIT ? "orbit"
                             : camera->mode == CAM_FREE  ? "free" : "first";
            printf("[camlog] t=%.3fs mode=%s focus=\"%s\" "
                   "yaw=%+.5f rad pitch=%+.5f rad dist=%.3f m "
                   "fwd=[%+.4f %+.4f %+.4f] rmb=%d cap=%d\n",
                   g.time, mode, g.focusTargets[g.focusBody].name,
                   camera->orbitYaw, camera->orbitPitch, camera->distance,
                   camera->forward.x, camera->forward.y, camera->forward.z,
                   g.rmbCam ? 1 : 0,
                   ImGui::GetIO().WantCaptureMouse ? 1 : 0);
            fflush(stdout);
        }
    }
}

/* The orbital-plane overlay (Settings "Orbital plane"): a ring in the ship's
   orbital plane, a ring in the orbit map's reference plane (Game::map_plane, so
   the dihedral here is the Inc the map and HUD print), and a tick at each point
   where the two cross -- the node line, where a plane change is cheapest (both
   nodes get the same white: for a symmetric plane change either is fine). Like
   the reference circles these are CAMERA-CENTRED: the view's translation is
   dropped, so each plane shows as its horizon line and the rings fix
   orientation only, not where the plane sits relative to the ground. The caller
   binds g.lineshader and passes proj * dmat3(view). */
static void drawOrbitPlane(Game &g, const glm::dmat4 &VP) {
    Vehicle *ship = g.ship;
    /* The orbit state and the stored reference plane both live in the focus
       body's INERTIAL frame; the render frame may be the body's ROTATING one, in
       which case the rings turn with the ground -- which is the point when
       flying. Assert that pairing, not just non-nullness: a mismatch skews the
       rings silently. */
    assert(ship->m_parent != nullptr && ship->m_parent->frame != nullptr
           && ship->m_parent->frame == ship->frame->getNonRotFrame());
    Frame *inertial = ship->m_parent->frame;
    const OrbitElements &o = g.view.o;
    const glm::dvec3 &h_hat = o.h_hat;
    if(glm::dot(h_hat, h_hat) < 0.5) { return; }   // radial / degenerate state
    /* A ship resting on the ground has no conic to take a plane from: its h_hat
       is the circle the ground draws under it, so the ring would be the equator
       wearing an orbit's name. FLYING is not the same case -- a suborbital arc
       has a real plane, and refusing it here left the Map plane combo looking
       dead exactly when a plane change is on the table. */
    if(ship->isGrounded()) { return; }
    const glm::dmat3 to_inertial = ship->frame->GetOrientRelTo(inertial);
    const glm::dvec3 n_orb = glm::transpose(to_inertial) * h_hat;
    const glm::dvec3 n_ref = glm::transpose(to_inertial) * g.view.ref_plane.n_hat;
    // Same convention as orbitPlaneAngles: n_ref x h_hat is the ASCENDING node.
    const glm::dvec3 node = glm::cross(n_ref, n_orb);
    /* Coplanar -- map_plane == kRefOrbit (the reference IS the orbit plane), a
       genuinely equatorial orbit, or an escape the other way round -- means one
       ring and no node to point at. Reuse the HUD's own gate rather than a
       second threshold: rotation preserves |node|, so the ticks and the LAN dash
       can never disagree. */
    const bool coplanar = !g.view.plane.node_ok;
    glm::dvec3 u, v;
    planeTangentBasis(n_orb, coplanar ? glm::dvec3(0.0) : node, u, v);
    // The ring mesh is the XY circle, so the basis columns map +X -> u, +Y -> v,
    // +Z -> the plane normal.
    g.lineshader->setUniform_mat4(0, VP * glm::dmat4(glm::dmat3(u, v, n_orb)));
    g.lineshader->setUniform_vec4(1, glm::vec4(0.0f, 1.0f, 1.0f, 0.5f));  // orbit: cyan
    g.ref_circle_xy->Draw(GL_LINE_LOOP);
    if(!coplanar) {
        // Pin both bases to the node line: each ring then has a vertex on the
        // crossings, and the ticks sit at the ring's +/-X -- the same u for both
        // planes, so one marker mesh serves either basis.
        planeTangentBasis(n_ref, node, u, v);
        g.lineshader->setUniform_mat4(0, VP * glm::dmat4(glm::dmat3(u, v, n_ref)));
        g.lineshader->setUniform_vec4(1, glm::vec4(1.0f, 1.0f, 0.0f, 0.5f));  // reference: yellow
        g.ref_circle_xy->Draw(GL_LINE_LOOP);
        // The basis is pinned to the node line, so the +X tick is the ascending
        // side: the same green/red pair the orbit map's markers use, from the
        // same constants (constants.h), with this overlay's translucent alpha.
        g.lineshader->setUniform_vec4(1, glm::vec4(kNodeAnRgb[0], kNodeAnRgb[1],
                                                   kNodeAnRgb[2], 0.8f));
        g.ref_node_an->Draw(GL_LINES);
        g.lineshader->setUniform_vec4(1, glm::vec4(kNodeDnRgb[0], kNodeDnRgb[1],
                                                   kNodeDnRgb[2], 0.8f));
        g.ref_node_dn->Draw(GL_LINES);
    }
    /* --plane-log: the two normals as this frame drew them, in the RENDER frame.
       inc is rotation-invariant, so it must equal the [orbitlog] inc measured in
       the inertial frame; the components only match [mapdump]'s when the render
       frame IS the inertial one. In the ROTATING frame they must instead match
       the body's own axes: the equator's normal is exactly +Y there. */
    if(g.args.plane_log) {
        const Uint32 now_ms = SDL_GetTicks();
        if(g.logDue(kGatePlane, now_ms, g.orbit_log_interval_ms)) {
            const double inc = std::acos(glm::clamp(glm::dot(n_orb, n_ref),
                                                    -1.0, 1.0));
            /* The flight markers as paintWorld places them, so a marker that
               drifts off the plane it belongs on is visible here rather than
               only on screen: prg must be IN the plane and nrm must BE the
               plane normal. Both in the render frame, like n_orb. */
            const glm::dvec3 prg = glm::normalize(g.view.marker_vel);
            const glm::dvec3 nrm = glm::normalize(
                glm::cross(g.view.pos, g.view.marker_vel));
            printf("[planelog] t=%.1fs frame=%s plane=%s inc=%.4f deg "
                   "n_orb=(%.6f,%.6f,%.6f) n_ref=(%.6f,%.6f,%.6f) "
                   "node=(%.6f,%.6f,%.6f) ticks=%s "
                   "prg=(%.6f,%.6f,%.6f) nrm=(%.6f,%.6f,%.6f)\n",
                   g.time, ship->frame->isRotFrame() ? "rot" : "inertial",
                   refPlaneName(g.view.ref_plane.mode), inc * 180.0 / std::numbers::pi,
                   n_orb.x, n_orb.y, n_orb.z, n_ref.x, n_ref.y, n_ref.z,
                   node.x, node.y, node.z, coplanar ? "-" : "drawn",
                   prg.x, prg.y, prg.z, nrm.x, nrm.y, nrm.z);
            fflush(stdout);
        }
    }
}

/* Paint half of the 3D pass (#212). The postfx Begin/End pair lives in main.cpp
   around the clear and this call: no path leaves an FBO bound with no blit. */
void paintWorld(Game &g) {
    TransferPlanner &planner = g.xferPlanner;
    Vehicle *ship = g.ship;
    Frame *rf = ship ? ship->frame : g.home->frame;
    TerrainBody *localBody = ship ? ship->m_parent : g.home;
    const std::vector<TerrainBody *> &planets = g.sys.bodies;
    TerrainBody *sun = g.sun;
    Camera *camera = g.camera;

    ShipView &view = g.view;
    glm::dvec3 &pos = view.pos;
    glm::dvec3 &vel = view.vel;
    glm::dvec3 &facing = view.facing;

    const glm::dvec3 com = ship ? ship->get_center_of_mass() : glm::dvec3(0.0);

    // Starfield first as pure background: depth test and write off, so every
    // body draws over it -- even far ones, whose reverse-Z depth collapsed to
    // the 0.0 background.
    if(g.draw_starfield) {
        glDepthMask(GL_FALSE);
        glDisable(GL_DEPTH_TEST);
        g.skybox->Draw(camera, g.skyboxshader, sun->frame->GetOrientRelTo(rf),
                       skyGain(g, rf, sun, camera, localBody,
                               ship ? com : camera->GetPos()));
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
    }

    if(g.world_drawing == true) {
        // one per home body; StaticBuilding::Draw culls itself off-body
        for(auto *b : planets) {
            for(auto *p : b->pads) {
                p->Draw(camera, localBody, rf);
            }
        }
        // Ships in another frame are transformed into the render frame by
        // Vehicle::Draw. Body lists hold the free ships + EVA characters.
        for(auto *b : planets) {
            for(auto *s : b->ships) {
                s->Draw(camera, rf);
            }
        }
    }

    /* --pose-jitter: is the DRAWN relative pose quantized to the logic tick?
       The ship is pinned at renderOrigin, so the drawn offset is the COM alone
       -- do NOT subtract frame->root_pos: that is universe-root axes, and would
       measure the planet's own orbital motion. Steady gap with a ragged fdt is
       frame PACING, which interpolation cannot fix. Wall time is steady_clock,
       not SDL_GetTicks (1 ms ticks is too coarse for a 60 Hz frame). */
    if(g.args.pose_jitter && ship && localBody && localBody->frame) {
        const auto now = std::chrono::steady_clock::now();
        /* Re-arm on any discontinuity, or its jump reads as a jitter spike: a
           different ship, a Frame handoff, or a gap over a few frames. */
        const bool stale =
            !g.pose_jitter_armed
            || g.pose_jitter_prev_ship != ship
            || g.pose_jitter_prev_frame != ship->frame
            || std::chrono::duration<double>(
                   now - g.pose_jitter_prev_t).count() > 0.25;
        if(!stale) {
            const double fdt = std::chrono::duration<double>(
                now - g.pose_jitter_prev_t).count();
            const double gap = glm::length(com - g.pose_jitter_prev);
            // Attitude steps the same way, and the orbit basis comes from the
            // ship's axes, so a stepped attitude rotates the whole image.
            double dang = -1.0;
            if(ship->controller) {
                const glm::dvec3 nose = ship->partAxis(ship->controller, 2);
                dang = glm::angle(nose, g.pose_jitter_prev_nose);
            }
            printf("[jitter] t=%9.3f fdt=%6.2f ticks=%2d alpha=%.3f "
                   "r=%12.1f gap=%10.4f dang=%9.6f\n",
                   g.time, 1000.0 * fdt, (int)g.logic_ticks, g.accumulator / g.dt,
                   glm::length(com), gap, dang);
            fflush(stdout);
        }
        g.pose_jitter_prev = com;
        if(ship->controller) {
            g.pose_jitter_prev_nose = ship->partAxis(ship->controller, 2);
        }
        g.pose_jitter_prev_ship = ship;
        g.pose_jitter_prev_frame = ship->frame;
        g.pose_jitter_prev_t = now;
        g.pose_jitter_armed = true;
    }

    if(g.world_drawing == true) {
        for(auto&& planet : planets) {
            planet->Draw(camera, sun, rf);
        }
    }

    // Transparent shells over the opaque bodies, depth-write off; cloud decks
    // draw first (under the rim).
    if(g.world_drawing == true) {
        for(auto&& planet : planets) {
            if(!g.args.no_ocean) {
                planet->DrawOcean(camera, sun, rf, g.time);
            }
            if(!g.args.no_clouds) {
                planet->DrawClouds(camera, sun, rf, g.time);
            }
            if(!g.args.no_atmosphere) {
                planet->DrawAtmosphere(camera, sun, rf);
            }
            /* Rings last: the depth test is what hides the far arc -- but only
               while the planet's own depth survives quantization, and on the
               24-bit unorm window buffer it does not past ~1.7e7 m (#263). There
               the whole ring draws over its own planet. */
            if(!g.args.no_rings) {
                planet->DrawRings(camera, sun, rf);
            }
        }
    }

    /* draw engine plume */
    glm::dmat4 View = camera->GetView();
    glm::mat4 Projection = camera->GetProjection();
    if(ship && ship->m_thrust > 0) {
        for(Part *p : ship->parts) {
            if(!p->isThruster()) { continue; }
            // Air-breathing engines have no rocket plume.
            if(p->isJet()) { continue; }
            /* Only engines armed THIS tick (ApplyThrust): m_thrust is a coarse
               "something fired" flag, so a lit stage would otherwise paint
               plumes on un-ignited higher stages. */
            if(p->armedThrust <= 0.0f) { continue; }
            // The plume mesh is authored for the base part (radius 1 m,
            // height 2 m): scale it to this thruster's size.
            const double radius = p->def->radius;
            const double height = p->def->height;
            /* The part pose is COM-relative: the ACTIVE ship's COM IS the
               renderOrigin, so no huge absolute coord is materialized
               (precision). */
            glm::dvec3 plumePos; glm::dmat3 plumeRot;
            ship->partPoseRelCom(p, plumePos, plumeRot);
            glm::dmat4 Model = glm::translate(plumePos) * glm::dmat4(plumeRot)
                * glm::dmat4(glm::dmat3(radius, 0.0, 0.0,
                                         0.0, radius, 0.0,
                                         0.0, 0.0, height / 2.0));
            // already render-frame relative (COM == renderOrigin)
            glm::mat4 ModelViewFloat = View * Model;
            g.partsshader->Bind();
            g.partsshader->setUniform_mat4(0, Projection * ModelViewFloat);
            g.partsshader->setUniform_mat4(1, glm::mat4(1.0)); // identity (GLM 1.0.0+: default ctor is zero)
            g.partsshader->setUniform_vec3(2, glm::vec3(1, 1, 1));
            // Pin the shadow/alpha/tint/flat uniforms rather than inherit the
            // last part draw's values.
            g.partsshader->setUniform_vec1(3, 1.0f);
            g.partsshader->setUniform_vec1(4, 1.0f);
            g.partsshader->setUniform_vec3(5, glm::vec3(1, 1, 1));
            g.partsshader->setUniform_vec1(6, 0.0f);

            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, g.engine_plume_texture->id);
            glEnable(GL_BLEND);
            glBlendFunc(GL_ONE, GL_ONE);
            // Additive and translucent: no depth writes, like every other
            // translucent pass here.
            glDepthMask(GL_FALSE);
            glDisable(GL_CULL_FACE);
            g.engine_plume_mesh->Draw();
            glEnable(GL_CULL_FACE);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            glBindTexture(GL_TEXTURE_2D, 0);
        }
    }
    /* end draw engine plume */

    /* RCS plume: from the COM along the armed ship-relative direction. The quad
       spans the exhaust direction and the camera axis LEAST aligned with it,
       keeping the flat strip screen-facing. */
    if(ship && ship->rcsFiring && glm::length2(ship->rcsDir) > 1e-12) {
        const glm::dvec3 z = ship->rcsWorldDir();  // thrust dir; the mesh extends toward -z
        const glm::dvec3 camR = glm::normalize(glm::cross(g.camera->forward, g.camera->up));
        const glm::dvec3 seed = (std::abs(glm::dot(camR, z)) < std::abs(glm::dot(g.camera->up, z)))
            ? camR : g.camera->up;
        const glm::dvec3 x = glm::normalize(seed - glm::dot(seed, z) * z);
        const glm::dvec3 y = glm::cross(z, x);
        /* 2 m wide, 3 m long off the COM; COM == renderOrigin here. */
        const double sx = 0.5, sz = 0.75;
        glm::dmat4 Model = glm::translate(z * sz)
            * glm::dmat4(glm::dmat3(x, y, z))
            * glm::dmat4(glm::dmat3(sx, 0.0, 0.0,
                                         0.0, sx, 0.0,
                                         0.0, 0.0, sz));
        glm::mat4 ModelViewFloat = View * Model;
        g.partsshader->Bind();
        g.partsshader->setUniform_mat4(0, Projection * ModelViewFloat);
        g.partsshader->setUniform_mat4(1, glm::mat4(1.0)); // identity (GLM 1.0.0+: default ctor is zero)
        g.partsshader->setUniform_vec3(2, glm::vec3(1, 1, 1));
        // pin the authoring uniforms like the engine plume does
        g.partsshader->setUniform_vec1(3, 1.0f);
        g.partsshader->setUniform_vec1(4, 1.0f);
        g.partsshader->setUniform_vec3(5, glm::vec3(1, 1, 1));
        g.partsshader->setUniform_vec1(6, 0.0f);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, g.engine_plume_texture->id);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);
        glDepthMask(GL_FALSE);   // additive/translucent: no depth writes
        glDisable(GL_CULL_FACE);
        g.engine_plume_mesh->Draw();
        glEnable(GL_CULL_FACE);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        glBindTexture(GL_TEXTURE_2D, 0);
    }
    /* end draw RCS plume */

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    // Flight, transfer and docking markers are all ship-relative: drawn only
    // with a ship.
    if(ship) {
    g.front_indicator->pos = facing;
    // The orbit camera already tracks the ship's roll, so the nose billboard
    // needs no roll of its own.
    g.front_indicator->Draw(camera, std::numbers::pi);
    /* Prograde and the normal pair are the CONIC's, so they read marker_vel --
       the orbital velocity in this frame. vel is the frame-relative one, which
       inside the rotating surface frame misses the ground's own motion: the
       markers then sit off the plane the orbital-plane overlay draws, and a
       slowly flying ship points nowhere near its orbit's prograde. Radial is
       the same line in both frames, so pos serves it. */
    g.prograde_indicator->pos = view.marker_vel;
    g.prograde_indicator->Draw(camera, std::numbers::pi);
    g.retrograde_indicator->pos = - view.marker_vel;
    g.retrograde_indicator->Draw(camera, std::numbers::pi);
    g.radial_in_indicator->pos = - pos;
    g.radial_in_indicator->Draw(camera, std::numbers::pi);
    g.radial_out_indicator->pos = pos;
    g.radial_out_indicator->Draw(camera, std::numbers::pi);
    g.normal_plus_indicator->pos = glm::cross(pos, view.marker_vel);
    g.normal_plus_indicator->Draw(camera, std::numbers::pi);
    g.normal_minus_indicator->pos = -glm::cross(pos, view.marker_vel);
    g.normal_minus_indicator->Draw(camera, std::numbers::pi);
    // Transfer burn direction (TARGET window, with "Compute transfer" on --
    // xfer.valid is what the planner clears when planning goes off).
    if(planner.xfer.valid && glm::length(planner.xfer.burn_dir) > 0.0) {
        g.burn_indicator->pos = planner.xfer.burn_dir;
        g.burn_indicator->Draw(camera, std::numbers::pi);
    }
    // Target ship's relative velocity: two markers (you-target / target-you),
    // when a ship is targeted or a docking port is selected.
    Vehicle *relvel_target = nullptr;
    if(planner.xfer_target >= 0 &&
       planner.xfer_target < (int)planner.xferTargets.size()) {
        relvel_target = planner.xferTargets[planner.xfer_target].ship;
    }
    if(relvel_target == nullptr &&
       ship->dockTargetShip != nullptr && ship->dockTargetPort != nullptr) {
        relvel_target = ship->dockTargetShip;
    }
    if(relvel_target != nullptr) {
        // Both velocities into the shared inertial frame (stasis + frame
        // velocity), then the difference back into this render frame.
        Frame *inertial = ship->frame->getNonRotFrame();
        Frame *sf = ship->frame;
        Frame *tsf = relvel_target->frame;
        const glm::dmat3 Os = sf->GetOrientRelTo(inertial);
        const glm::dvec3 sv = Os * (vel + sf->GetStasisVelocity(com))
            + sf->GetVelocityRelTo(inertial);
        const glm::dvec3 tcom = relvel_target->get_center_of_mass();
        const glm::dmat3 Ot = tsf->GetOrientRelTo(inertial);
        const glm::dvec3 tv = Ot * (relvel_target->GetVel()
            + tsf->GetStasisVelocity(tcom))
            + tsf->GetVelocityRelTo(inertial);
        const glm::dvec3 relvel = glm::transpose(Os) * (tv - sv);  // target − you
        if(glm::length(relvel) > 1e-9) {
            // you − target: the prograde (diamond) icon
            g.relvel_indicator->pos = -relvel;
            g.relvel_indicator->Draw(camera, std::numbers::pi);
            // target − you: the retrograde (X) icon
            g.relvel_retro_indicator->pos = relvel;
            g.relvel_retro_indicator->Draw(camera, std::numbers::pi);
        }
    }
    }
    // horizon_indicator->pos = groundHed;
    // horizon_indicator->Draw(camera, std::numbers::pi);

    /* Both overlays drop the view's TRANSLATION, so every ring is centred on the
       camera: they read as orientation rings around the viewpoint rather than
       lines painted at a fixed place in the world. --plane-log reports what the
       orbital-plane overlay draws, so asking for the log draws it: the overlay
       is otherwise a settings checkbox and the flag would print nothing. */
    if(g.draw_ref_circles || g.draw_orbit_plane || g.args.plane_log) {
        glLineWidth(4);
        g.lineshader->Bind();
        const glm::dmat4 VP = glm::dmat4(camera->GetProjection())
                            * glm::dmat4(glm::dmat3(camera->GetView()));
        if(g.draw_ref_circles) {
            g.lineshader->setUniform_mat4(0, VP);
            // XZ plane (flat / orbital-equatorial reference): green
            g.lineshader->setUniform_vec4(1, glm::vec4(0, 1, 0, 0.5));
            g.ref_circle_xz->Draw(GL_LINE_LOOP);
            // XY plane (vertical / meridian reference): magenta
            g.lineshader->setUniform_vec4(1, glm::vec4(1, 0, 1, 0.5));
            g.ref_circle_xy->Draw(GL_LINE_LOOP);
        }
        if(ship && (g.draw_orbit_plane || g.args.plane_log)) {
            drawOrbitPlane(g, VP);
        }
        // Restore: the width is global state, and the Bullet debug drawer draws
        // GL_LINES with whatever the last setter left.
        glLineWidth(1.0f);
    }

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);

    if(g.physics_debug_drawing == true) {
        glDisable(GL_DEPTH_TEST);
        debug_draw(camera);
        glEnable(GL_DEPTH_TEST);
    }
}

/* The VAB build tree's shroud test: same shared childBelow as
   Vehicle::hasChildBelow (shipdef.h). */
static bool vabChildBelow(const BuildShip &bs, size_t i) {
    const BuildPart &bp = bs.parts[i];
    for(size_t j = 0; j < bs.parts.size(); j++) {
        if(j == i || bs.parts[j].parent != (int)i) { continue; }
        if(childBelow(bp.def->radius, bp.localPos, bp.localRot,
                      bs.parts[j].localPos)) { return true; }
    }
    return false;
}

/* The editor's state half (#212): no LOD -- running it here would build ground
   collision around the build centre. The sim still coasts while you build. */
void refreshVab(Game &g) {
    /* Render frame centred on the build tree; ref = identity so screen-up is
       world +Z. */
    Camera *cam = g.camera;
    cam->renderOrigin = g.vab.center;
    cam->ref = glm::dmat3(1.0);
    cam->Follow(g.vab.center);
    cam->ComputeView();

    const glm::dvec3 com =
        g.ship ? g.ship->get_center_of_mass() : glm::dvec3(0.0);
    refreshShipState(g, com);
}

void paintVab(Game &g) {
    Camera *cam = g.camera;
    glm::vec3 sunlight = glm::normalize(glm::vec3(0.4f, 0.8f, 0.35f));
    for(size_t i = 0; i < g.vab.build.parts.size(); i++) {
        const BuildPart &bp = g.vab.build.parts[i];
        if(bp.def == nullptr) { continue; }
        Mesh *m = get_mesh(std::string("res/") + bp.def->mesh);
        Texture *t = get_texture(std::string("res/") + bp.def->texture);
        if(m == nullptr || t == nullptr) { continue; }
        const glm::dmat4 model = glm::translate(bp.localPos)
                               * glm::dmat4(bp.localRot);
        DrawOpts opts;
        opts.flat = 0.8f;   // mostly studio light, a touch of bias (the editor look)
        if((int)i == g.vab.selected) { opts.tint = glm::vec3(1.0f, 0.75f, 0.2f); }
        else if((int)i == g.vab.hover) { opts.tint = glm::vec3(0.6f, 1.0f, 0.6f); }
        DrawModelAt(cam, m, g.partsshader, t, model, sunlight, 1.0f,
                    glm::dmat4(1.0), opts);

        /* Engine shroud (PartDef.shroud): a part attached on the exhaust face
           hides the engine. */
        if(!bp.def->shroud.empty() && vabChildBelow(g.vab.build, i)) {
            Mesh *sm = get_mesh(std::string("res/") + bp.def->shroud);
            Texture *st = get_texture(std::string("res/") + bp.def->shroud_texture);
            if(sm != nullptr && st != nullptr) {
                DrawModelAt(cam, sm, g.partsshader, st, model, sunlight,
                            1.0f, glm::dmat4(1.0), opts);
            }
        }
    }

    // The armed part's (or subassembly's) ghost at the hovered attach target.
    if(g.vab.ghostValid) {
        DrawOpts go;
        go.alpha = 0.4f;
        go.flat = 0.8f;   // the ghosts share the editor's lighting
        if(g.vab.ghostAssembly >= 0
           && (size_t)g.vab.ghostAssembly < g.vab.subassemblies.size()) {
            /* The whole tree rides each solved root pose: the primary plus each
               symmetry clone. */
            const BuildShip &sub = g.vab.subassemblies[(size_t)g.vab.ghostAssembly].ship;
            for(size_t pass = 0; pass < 1 + g.vab.ghostClones.size(); pass++) {
                const glm::dvec3 rp = (pass == 0)
                    ? g.vab.ghostPos : g.vab.ghostClones[pass - 1].pose.childPos;
                const glm::dmat3 rr = (pass == 0)
                    ? g.vab.ghostRot : g.vab.ghostClones[pass - 1].pose.childRot;
                for(size_t i = 0; i < sub.parts.size(); i++) {
                    const BuildPart &bp = sub.parts[i];
                    if(bp.def == nullptr) { continue; }
                    Mesh *m = get_mesh(std::string("res/") + bp.def->mesh);
                    Texture *t = get_texture(std::string("res/") + bp.def->texture);
                    if(m == nullptr || t == nullptr) { continue; }
                    const glm::dmat4 model =
                        glm::translate(rp + rr * bp.localPos)
                        * glm::dmat4(rr * bp.localRot);
                    DrawModelAt(cam, m, g.partsshader, t, model, sunlight, 1.0f,
                                glm::dmat4(1.0), go);
                }
            }
        } else if(!g.vab.armed.empty()) {
            const PartDef *ad = g.ships.catalog().find(g.vab.armed);
            if(ad != nullptr) {
                Mesh *gm = get_mesh(std::string("res/") + ad->mesh);
                Texture *gt = get_texture(std::string("res/") + ad->texture);
                if(gm != nullptr && gt != nullptr) {
                    const glm::dmat4 gmodel = glm::translate(g.vab.ghostPos)
                                            * glm::dmat4(g.vab.ghostRot);
                    DrawModelAt(cam, gm, g.partsshader, gt, gmodel, sunlight,
                                1.0f, glm::dmat4(1.0), go);
                    for(size_t k = 0; k < g.vab.ghostClones.size(); k++) {
                        const AttachPose &cp = g.vab.ghostClones[k].pose;
                        const glm::dmat4 cmodel = glm::translate(cp.childPos)
                                                * glm::dmat4(cp.childRot);
                        DrawModelAt(cam, gm, g.partsshader, gt, cmodel, sunlight,
                                    1.0f, glm::dmat4(1.0), go);
                    }
                }
            }
        }
    }
}
