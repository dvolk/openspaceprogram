// render.cpp -- the 3D render pass (see render.h).
#include "render.h"

#include <cmath>
#include <numbers>
#include <cstdlib>
#include <GL/glew.h>   // glBlendFunc / glLineWidth / the GL enums

#include "billboard.h"
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

/* Camera shake at high acceleration: the felt g's set the amplitude, which
   is low-passed toward random targets and decays to zero when quiet. Orbit
   camera on the ship only; --cam-shake scales (0 = off). */
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
    // Low-pass each axis toward a fresh random target. The time constant is
    // wall-clock seconds so the rumble rate does not scale with the render
    // fps.
    const double frame_dt = (now_ms - g.shake_last_ms) * 0.001;
    g.shake_last_ms = now_ms;
    const double tau = 0.05;    // s: ~3-frame correlation at 60 fps
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
        if(now_ms - g.shake_log_last_ms >= g.orbit_log_interval_ms) {
            g.shake_log_last_ms = now_ms;
            printf("[shakelog] t=%.3fs a=%.3f m/s2 amp=%.4f m "
                   "off=[%+.4f %+.4f %+.4f]\n",
                   g.time, a, target,
                   g.shake_off.x, g.shake_off.y, g.shake_off.z);
            fflush(stdout);
        }
    }
}

// The active ship's per-frame state snapshot (Game::view). Reads only ship
// + frame state -- no camera -- so a scene that runs the sim without the GL
// pass (the live Tracking Station) can refresh it each frame.
void updateShipView(Game &g) {
    Vehicle *ship = g.ship;
    if(ship == nullptr) { return; }
    ShipView &view = g.view;
    double &mu = view.mu;
    glm::dvec3 &pos = view.pos;
    glm::dvec3 &vel = view.vel;
    glm::dvec3 &orbit_pos = view.orbit_pos;
    glm::dvec3 &orbit_vel = view.orbit_vel;
    glm::dvec3 &surf_pos = view.surf_pos;
    glm::dvec3 &surf_vel = view.surf_vel;
    OrbitElements &o = view.o;
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

    // surf pos??
    mu = ship->m_parent->mu;
    pos = com;
    /* orbital velocity */
    vel = ship->GetVel();

    // The orbit is a Kepler conic in the body's INERTIAL (non-rotating)
    // frame -- the frame the spawn/switching code targets.
    orbit_pos = pos;
    orbit_vel = vel;
    if(ship->frame->isRotFrame() == true) {
        Frame *inertial = ship->frame->getNonRotFrame();
        orbit_vel += ship->frame->GetStasisVelocity(orbit_pos);
        orbit_vel = ship->frame->GetOrientRelTo(inertial) * orbit_vel + ship->frame->GetVelocityRelTo(inertial);
        orbit_pos = ship->frame->GetOrientRelTo(inertial) * orbit_pos + ship->frame->GetPositionRelTo(inertial);
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
    distance = o.distance;
    speed = o.speed;

    // Telemetry: e and |h| are conserved 2-body constants (drift =
    // integrator error; steps = burns/staging).
    energy_series.push(g.time, o.energy);
    angmom_series.push(g.time, o.ang_momentum);

    up = ship->partAxis(ship->controller, 1);
    facing = ship->partAxis(ship->controller, 2);
    other = ship->partAxis(ship->controller, 0);

    facing_dir = glm::normalize(facing);
    vel_dir = glm::normalize(vel);

    /* A3: heading/pitch/roll are SURFACE-relative (KSP convention),
       measured against the ground's north/east in the rotating frame. */
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

    longitude = atan2(dir.x, dir.z);
    latitude = asin(dir.y);
}

void draw3d(Game &g) {
    TransferPlanner &planner = g.xferPlanner;
    Vehicle *ship = g.ship;
    // Render frame: the active ship's frame, or the home body's frame when
    // there is no ship. Every Draw site transforms world geometry into it.
    Frame *rf = ship ? ship->frame : g.home->frame;
    // The body "we're on" (pads cull to it; drawn at origin).
    TerrainBody *localBody = ship ? ship->m_parent : g.home;
    const std::vector<TerrainBody *> &planets = g.sys.bodies;
    TerrainBody *sun = g.sun;
    Camera *camera = g.camera;
    PostFX *postfx = g.postfx;

    // Per-frame state lives in g.view so the UI readouts read one snapshot.
    ShipView &view = g.view;
    glm::dvec3 &pos = view.pos;
    glm::dvec3 &vel = view.vel;
    glm::dvec3 &facing = view.facing;

    const glm::dvec3 com = ship ? ship->get_center_of_mass() : glm::dvec3(0.0);
    if(g.camera->mode == CAM_ORBIT) {
        camera->Follow(g.focusWorldPos(g.focusBody));
        // The orientation the orbit offset lives in: the ship's attitude
        // when focused on the ship, the body's rotating frame when focused
        // on a body -- same convention as the body transforms below.
        if(g.focusTargets[g.focusBody].body == nullptr) {
            if(ship->isEva()) {
                // North-up on the body's local vertical: screen-up = the
                // kerbal's radial (what eva.cpp drives its nose to), and the
                // tangent plane from the WORLD axes -- not the frame's orient
                // (spins with the planet) and not the kerbal's own axes
                // (locks the camera to its yaw and feeds back into the
                // attitude law). Tangent seam at |radial.y| = 0.9.
                const glm::dvec3 radial = glm::normalize(com);
                // Tangent "north": prefer +Y; fall back to +X near +/-Y so
                // the projection never vanishes.
                const glm::dvec3 ref_dir = (std::fabs(radial.y) > 0.9)
                    ? glm::dvec3(1.0, 0.0, 0.0) : glm::dvec3(0.0, 1.0, 0.0);
                const glm::dvec3 right = glm::normalize(
                    ref_dir - radial * glm::dot(ref_dir, radial));
                const glm::dvec3 back = glm::cross(right, radial);
                camera->ref = glm::dmat3(back, right, radial);
            } else {
            // The orbit camera expects ref as [back, right, up]; the ship's
            // attitude is [right, up, nose] (axis 0/1/2). Feed the columns
            // in the camera's order so its up is the ship's UP, not its nose
            // (the nose-as-up swaps the visual pitch/yaw responses).
            camera->ref = glm::dmat3(ship->partAxis(ship->controller, 2),   // back = nose
                                     ship->partAxis(ship->controller, 0),   // right
                                     ship->partAxis(ship->controller, 1));  // up
            }
        } else {
            TerrainBody *b = g.focusTargets[g.focusBody].body;
            camera->ref = b->frame->getRotFrame()->GetOrientRelTo(rf);
        }
    }

    // Camera shake (camShakeStep above): shift the focus point and wobble
    // the orbit basis. Orbit camera on the ship only.
    if(ship) { camShakeStep(g, ship); }
    if(g.camera->mode == CAM_ORBIT && ship &&
       g.focusTargets[g.focusBody].body == nullptr) {
        camera->focusPoint += g.shake_off;
        camera->ref = shakeRot(g.shake_ang) * camera->ref;
    }

    // Render frame origin = the active ship's COM, so the float32 cast
    // works on ship-relative numbers. Draw sites shift by -renderOrigin.
    camera->renderOrigin = com;
    camera->ComputeView();

    // Starfield first, as a pure background -- depth test and depth write
    // both off. Every body draws over it, including far ones whose
    // reverse-Z depth has collapsed to the 0.0 background value.
    // skyRot maps the inertial starfield into the ship's frame.
    if(g.draw_starfield) {
        glDepthMask(GL_FALSE);
        glDisable(GL_DEPTH_TEST);
        g.skybox->Draw(camera, g.skyboxshader, sun->frame->GetOrientRelTo(rf));
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
        // render frame = the active ship's frame; idle ships in a different
        // frame are transformed into it in Vehicle::Draw. Body lists hold
        // the free ships + EVA characters (aboard crew live on their ship).
        for(auto *b : planets) {
            for(auto *s : b->ships) {
                s->Draw(camera, rf);
            }
        }
    }

    for(auto&& planet : planets) {
        planet->transform = planet->frame->GetBodyDrawTransform(rf);
    }

    for(auto&& planet : planets) {
        // LOD may post async subdivision jobs; their continuations run in
        // jobs.poll() BEFORE this pass draws.
        planet->Update(camera, g.args.terrain_px, g.jobs);

        /* --terrain-log: the local body's tree as the LOD just left it.
           deep_off is the camera-to-nearest-deepest-leaf distance (a patch
           width or two is healthy); collision counts leaves with a Bullet
           body (the ground exists only where the camera is -- issue #24). */
        if(g.args.terrain_log && planet == localBody && planet->ready) {
            const Uint32 now_ms = SDL_GetTicks();
            if(now_ms - g.terrain_log_last_ms >= g.orbit_log_interval_ms) {
                g.terrain_log_last_ms = now_ms;
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

        if(g.world_drawing == true) {
            planet->Draw(camera, sun, rf);
        }
    }

    // The active ship's per-frame state (g.view): between the world draw
    // and the atmosphere rims (where it always ran).
    updateShipView(g);

    // Atmosphere rims: transparent Fresnel shells over the opaque bodies.
    // Depth-write off. Cloud decks draw first (under the rim).
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
            // Rings last: the depth buffer already hides the far arc.
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
            /* an air-breathing engine has no rocket plume -- suppress the
               flame mesh. */
            if(p->isJet()) { continue; }
            /* only engines actually thrusting THIS tick (armed in
               ApplyThrust). m_thrust above is a coarse "something fired"
               flag -- without this a lit stage would paint fake plumes on
               un-ignited higher stages. */
            if(p->armedThrust <= 0.0f) { continue; }
            /* the plume mesh is authored for the base part (radius 1 m,
               height 2 m): scale it to this thruster's size */
            const double radius = p->def->radius;
            const double height = p->def->height;
            /* Built from the part's COM-relative pose: the plume is only on
               the ACTIVE ship, whose COM IS the renderOrigin -- no huge
               absolute coord is materialized (precision). Also independent
               of Body::model_matrix (a Body::Draw side-effect cache). */
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
            // pin the shadow/alpha/tint/flat uniforms rather than
            // inheriting the last part draw's values
            g.partsshader->setUniform_vec1(3, 1.0f);
            g.partsshader->setUniform_vec1(4, 1.0f);
            g.partsshader->setUniform_vec3(5, glm::vec3(1, 1, 1));
            g.partsshader->setUniform_vec1(6, 0.0f);

            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, g.engine_plume_texture->id);
            glEnable(GL_BLEND);
            glBlendFunc(GL_ONE, GL_ONE);
            glDisable(GL_CULL_FACE);
            g.engine_plume_mesh->Draw();
            glEnable(GL_CULL_FACE);
            glDisable(GL_BLEND);
            glBindTexture(GL_TEXTURE_2D, 0);
        }
    }
    /* end draw engine plume */

    /* draw the RCS plume: the puff is drawn from the COM along the armed
       ship-relative direction. The quad's plane is spanned by the exhaust
       direction and the camera axis LEAST aligned with it, so the flat
       strip stays near screen-facing. */
    if(ship && ship->rcsFiring && glm::length2(ship->rcsDir) > 1e-12) {
        const glm::dvec3 z = ship->rcsWorldDir();  // thrust dir; the mesh extends toward -z
        const glm::dvec3 camR = glm::normalize(glm::cross(g.camera->forward, g.camera->up));
        const glm::dvec3 seed = (std::abs(glm::dot(camR, z)) < std::abs(glm::dot(g.camera->up, z)))
            ? camR : g.camera->up;
        const glm::dvec3 x = glm::normalize(seed - glm::dot(seed, z) * z);
        const glm::dvec3 y = glm::cross(z, x);
        /* 2 m wide, 3 m long off the COM. COM == renderOrigin here, so the
           small offset is the whole render-frame translation. */
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
        glDisable(GL_CULL_FACE);
        g.engine_plume_mesh->Draw();
        glEnable(GL_CULL_FACE);
        glDisable(GL_BLEND);
        glBindTexture(GL_TEXTURE_2D, 0);
    }
    /* end draw RCS plume */

    /* Transfer planner: rebuild targets, recompute the solution (com / vel
       are this render pass's ship COM / velocity). */
    planner.update(com, vel);

    // P key: the one-shot flag is set in poll_events; it runs here, where
    // the planner + this pass's ship snapshot both exist.
    if(g.porkchop_compute_requested) {
        planner.porkchopCompute();
        g.porkchop_compute_requested = false;
    }

    // M key: one-shot, same pattern as P. The sweep runs on the background
    // worker; the last map stays on screen until the job lands.
    if(g.surfmap_compute_requested) {
        surfmapCompute(g);
        g.surfmap_compute_requested = false;
    }

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    // The flight reference markers and transfer/docking markers are all
    // ship-relative: drawn only with a ship.
    if(ship) {
    g.front_indicator->pos = facing;
    // Fixed-orientation nose marker: the orbit camera already tracks the
    // ship's roll, so the billboard is roll-invariant.
    g.front_indicator->Draw(camera, std::numbers::pi);
    g.prograde_indicator->pos = vel;
    g.prograde_indicator->Draw(camera, std::numbers::pi);
    g.retrograde_indicator->pos = - vel;
    g.retrograde_indicator->Draw(camera, std::numbers::pi);
    g.radial_in_indicator->pos = - pos;
    g.radial_in_indicator->Draw(camera, std::numbers::pi);
    g.radial_out_indicator->pos = pos;
    g.radial_out_indicator->Draw(camera, std::numbers::pi);
    g.normal_plus_indicator->pos = glm::cross(pos, vel);
    g.normal_plus_indicator->Draw(camera, std::numbers::pi);
    g.normal_minus_indicator->pos = -glm::cross(pos, vel);
    g.normal_minus_indicator->Draw(camera, std::numbers::pi);
    // Transfer burn direction (TRANSFER window target selected):
    // KSP-blue prograde icon pointing where the departure burn goes.
    if(planner.xfer.valid && glm::length(planner.xfer.burn_dir) > 0.0) {
        g.burn_indicator->pos = planner.xfer.burn_dir;
        g.burn_indicator->Draw(camera, std::numbers::pi);
    }
    // Target ship's relative velocity: two pink markers (you-target /
    // target-you), shown when a ship is targeted or a docking port is
    // selected.
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
        // Both velocities in the shared inertial frame (stasis + frame
        // velocity carry a rotating/moving frame's contribution), then the
        // difference in this render frame.
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

    if(g.draw_skylines) {
        glLineWidth(4);
        g.lineshader->Bind();
        g.lineshader->setUniform_mat4(0, glm::dmat4(camera->GetProjection()) * glm::dmat4(glm::dmat3(camera->GetView())));
        // XZ plane (flat / orbital-equatorial reference): green
        g.lineshader->setUniform_vec4(1, glm::vec4(0, 1, 0, 0.5));
        g.skyline_xz->Draw(GL_LINE_LOOP);
        // XY plane (vertical / meridian reference): magenta
        g.lineshader->setUniform_vec4(1, glm::vec4(1, 0, 1, 0.5));
        g.skyline_xy->Draw(GL_LINE_LOOP);
    }

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);

    if(g.physics_debug_drawing == true) {
        glDisable(GL_DEPTH_TEST);
        debug_draw(camera);
        glEnable(GL_DEPTH_TEST);
    }

    postfx->End();  // no-op unless --postfx effects are active
}

/* The shroud condition for the VAB build tree: same shared test as
   Vehicle::hasChildBelow (childBelow in shipdef.h). */
static bool vabChildBelow(const BuildShip &bs, size_t i) {
    const BuildPart &bp = bs.parts[i];
    for(size_t j = 0; j < bs.parts.size(); j++) {
        if(j == i || bs.parts[j].parent != (int)i) { continue; }
        if(childBelow(bp.def->radius, bp.localPos, bp.localRot,
                      bs.parts[j].localPos)) { return true; }
    }
    return false;
}

void drawVab(Game &g) {
    /* Center the render frame on the build tree; ref = identity so
       screen-up is world +Z (the ship stands nose-up, as on the pad). */
    Camera *cam = g.camera;
    cam->renderOrigin = g.vab.center;
    cam->ref = glm::dmat3(1.0);
    cam->Follow(g.vab.center);
    cam->ComputeView();

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
        opts.flat = 1.0f;   // uniform studio light (the editor look)
        if((int)i == g.vab.selected) { opts.tint = glm::vec3(1.0f, 0.75f, 0.2f); }
        else if((int)i == g.vab.hover) { opts.tint = glm::vec3(0.6f, 1.0f, 0.6f); }
        DrawModelAt(cam, m, g.partsshader, t, model, sunlight, 1.0f,
                    glm::dmat4(1.0), opts);

        /* Engine shroud (see PartDef.shroud): while a part is attached on
           the exhaust face, the plain cylinder hides the engine. Shares the
           part's editor opts (studio light + hover/selection tint). */
        if(!bp.def->shroud.empty() && vabChildBelow(g.vab.build, i)) {
            Mesh *sm = get_mesh(std::string("res/") + bp.def->shroud);
            Texture *st = get_texture(std::string("res/") + bp.def->shroud_texture);
            if(sm != nullptr && st != nullptr) {
                DrawModelAt(cam, sm, g.partsshader, st, model, sunlight,
                            1.0f, glm::dmat4(1.0), opts);
            }
        }
    }

    // the armed part's (or subassembly's) translucent ghost at the hovered
    // attach target
    if(g.vab.ghostValid) {
        DrawOpts go;
        go.alpha = 0.4f;
        go.flat = 1.0f;   // the ghosts share the studio light
        if(g.vab.ghostAssembly >= 0
           && (size_t)g.vab.ghostAssembly < g.vab.subassemblies.size()) {
            /* an assembly ghost: the whole tree rides the solved root pose,
               once per root pose (the primary plus each symmetry clone). */
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
