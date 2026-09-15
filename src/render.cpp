// render.cpp -- the 3D render pass (declared in render.h).
//
// This was the render section of main's loop: the world draw (pads,
// ships, planets, atmospheres), the active ship's per-frame state (the
// orbit + surface + attitude + lat-lon math, the telemetry samples) and
// the overlays (starbox, engine plume, the maneuver indicators, the
// reference skylines, the physics debug). It moved out as-is: main's
// locals became Game members (aliased below so the body reads the same),
// and the per-frame state now lands in Game::view (ShipView) instead of
// main's locals, so the UI readouts read one snapshot. The transfer
// planner's per-frame update keeps its exact position in the pass (it is
// a parameter, so the call order with the burn indicator is unchanged).
#include "render.h"

#include <cmath>
#include <cstdlib>
#include <GL/glew.h>   // glBlendFunc / glLineWidth / the GL enums

#include "billboard.h"   // Billboard::Draw + the icon pos
#include "mesh.h"        // Mesh::Draw (the reference skylines)
#include "orbit.h"       // computeOrbitElements + the plane math
#include "physics.h"     // debug_draw
#include "shader.h"      // Shader::Bind / setUniform_*
#include "skybox.h"      // Skybox::Draw
#include "surfmap.h"     // surfmapCompute (the M key's surface map)
#include "texture.h"     // the plume texture's id

// GLM's gtx extensions (the attitude math) hard-error without this.
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/projection.hpp>    // glm::proj
#include <glm/gtx/vector_angle.hpp>  // glm::orientedAngle

// Small-angle rotation (a is ~1e-3 rad): R ~= I + [a]x. The shake wobble
// never approaches anything where the linearisation would matter.
static glm::dmat3 shakeRot(const glm::dvec3 &a) {
    return glm::dmat3(1.0, a.z, -a.y,
                      -a.z, 1.0, a.x,
                      a.y, -a.x, 1.0);
}

/* Camera shake at high acceleration: the crew's felt acceleration
   (thrust + aero over mass -- gravity excluded, so coasting and free
   fall are steady) sets the target amplitude. Each frame the state
   (g.shake_off + g.shake_ang) is low-passed toward fresh random targets,
   so the rumble is a smooth shake instead of a per-frame strobe, and
   decays to zero on its own when the engine goes quiet. Active only for
   the orbit camera focused on the ship (a body focus is a stable view,
   the free cam is a debug tool); --cam-shake scales the whole thing
   (0 = off). */
static void camShakeStep(Game &g, Vehicle *ship) {
    // The felt acceleration (what the crew feels): thrust + aero over
    // mass -- gravity excluded, so coasting and free fall read zero.
    // --shake-log reports it regardless of camera mode; the shake itself
    // acts only for the orbit camera on the ship (a body focus is a
    // stable view, the free cam is a debug tool).
    double a = 0.0;
    const double m = (double)ship->getMass();
    if(g.time_accel > 0 && !ship->onRails && m > 0.0) {
        a = glm::length(ship->lastThrustForce + ship->lastAeroForce) / m;
    }
    // Steady below the threshold (RCS-only flight, idle); above it the
    // amplitude grows with the felt g's and saturates.
    const bool active = a > 1.0
        && g.args.cam_shake > 0.0
        && g.camera->mode == CAM_ORBIT
        && g.focusTargets[g.focusBody].body == nullptr;
    const double target = active
        ? (double)g.args.cam_shake * std::min(0.15, (a - 1.0) * 0.008)
        : 0.0;
    const Uint32 now_ms = SDL_GetTicks();
    // Low-pass each axis toward a fresh random target: correlated jitter
    // (a smooth rumble), toward zero when the engine is quiet. The time
    // constant is in wall-clock seconds, so the rumble rate does not
    // scale with the render fps (per-frame alpha would rumble 4x faster
    // at 240 fps than at 60).
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
    // --shake-log: the felt accel + the live target amplitude, at the
    // --orbit-interval cadence (its own clock, like the other log gates).
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

void draw3d(Game &g, TransferPlanner &planner) {
    // The pass body is verbatim from main's render section; its globals
    // are Game members (aliased so the body reads the same).
    Vehicle *ship = g.ship;
    const std::vector<TerrainBody *> &planets = g.sys.bodies;
    TerrainBody *sun = g.sun;
    Camera *camera = g.camera;
    PostFX *postfx = g.postfx;

    // The per-frame state: was local consts in main's render section, now
    // lives in g.view (ShipView) so the UI readouts read one snapshot.
    // The references keep the body's names.
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
    if(g.camera->mode == CAM_ORBIT) {
        camera->Follow(g.focusWorldPos(g.focusBody));
        // The orientation the orbit offset lives in: the ship's attitude
        // when focused on the ship (the camera chases its turns, KSP /
        // Pioneer style), the body's rotating frame when focused on a body
        // (the camera rides the spin) -- same convention as the body
        // transforms below.
        if(g.focusTargets[g.focusBody].body == nullptr) {
            if(ship->isEva()) {
                // The kerbal slews to face the camera (src/eva.cpp), so
                // chasing its attitude would be a feedback loop. Orbit in
                // the surface frame instead: up = local vertical, the
                // mouse orbits around the kerbal.
                camera->ref = (ship->frame->isRotFrame())
                    ? glm::dmat3(1.0)
                    : glm::dmat3(ship->frame->getRotFrame()->orient);
            } else {
            // The orbit camera builds its basis from ref as
            //   back (offset) = ref * x̂   (column 0)
            //   up   (screen) = ref * ẑ   (column 2)
            //   right          = ref * ŷ   (column 1)
            // i.e. it expects ref laid out as [back, right, up]. But the
            // ship's attitude matrix is [right, up, nose] (axis 0/1/2).
            // Feed the columns in the camera's order so its up is the
            // ship's UP, not its nose. With the nose as the up basis the
            // screen plane is the ship's right/up plane, so pitch (nose
            // along the up axis) read as left/right and yaw (nose along
            // the right axis) as up/down -- the two looked swapped.
            camera->ref = glm::dmat3(ship->partAxis(ship->controller, 2),   // back = nose
                                     ship->partAxis(ship->controller, 0),   // right
                                     ship->partAxis(ship->controller, 1));  // up
            }
        } else {
            TerrainBody *b = g.focusTargets[g.focusBody].body;
            camera->ref = (b == ship->m_parent && ship->frame->isRotFrame())
                ? glm::dmat3(1.0)
                : glm::dmat3(b->frame->getRotFrame()->orient);
        }
    }

    // Camera shake at high acceleration (camShakeStep above): step the
    // state once per frame, then -- for the orbit camera on the ship
    // only -- rigidly shift the focus point by the offset and wobble the
    // orbit basis. The chase cam follows both, so the whole view shakes
    // with the ship (KSP's engine-rumble feel); a body focus and the
    // free cam stay rock-steady.
    camShakeStep(g, ship);
    if(g.camera->mode == CAM_ORBIT &&
       g.focusTargets[g.focusBody].body == nullptr) {
        camera->focusPoint += g.shake_off;
        camera->ref = shakeRot(g.shake_ang) * camera->ref;
    }

    // Render frame origin = the active ship's COM (both are in
    // ship->frame, the render frame). The view is built there and
    // the Draw sites shift geometry by -renderOrigin, so the
    // float32 cast works on ship-relative numbers.
    camera->renderOrigin = com;
    camera->ComputeView();

    /*
      standard 3d stuff drawn here
    */

    // Starfield first, as a pure background -- depth test and depth write both
    // off. Every body then simply draws over it, including far ones (the sun
    // at Neptune range) whose reverse-Z depth has collapsed to the 0.0
    // background value; if the starfield were drawn last and depth-tested it
    // would win that tie and paint them out. skyRot maps the inertial
    // starfield into the ship's frame (it drifts once per sidereal day on a
    // spinning planet; identity = an inertial world).
    if(g.draw_starfield) {
        glDepthMask(GL_FALSE);
        glDisable(GL_DEPTH_TEST);
        g.skybox->Draw(camera, g.skyboxshader, sun->frame->GetOrientRelTo(ship->frame));
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
    }

    if(g.world_drawing == true) {
        // one per home body; StaticBuilding::Draw culls itself when
        // the active ship is not on that body
        for(auto *b : planets) {
            for(auto *p : b->pads) {
                p->Draw(camera, ship->m_parent, ship->frame);
            }
        }
        // render frame = the active ship's frame; idle ships in a
        // different frame are transformed into it in Vehicle::Draw. The
        // body lists hold the free ships + EVA characters (the aboard
        // crew live on their ship, not here), so no aboard-skip is needed.
        for(auto *b : planets) {
            for(auto *s : b->ships) {
                s->Draw(camera, ship->frame);
            }
        }
    }

    for(auto&& planet : planets) {
        if(planet == ship->m_parent) {
            //this is the planet we're on. This means its position is always 0, 0, 0

            if(ship->frame->isRotFrame()) {
                // we're in its rotational frame
                planet->transform = glm::dmat4(1.0);
            }
            else {
                // we're in its inertial frame
                planet->transform = glm::dmat4(planet->frame->getRotFrame()->orient);
            }
        }
        else {
            // other planets
            glm::dvec3 translate = planet->frame->GetPositionRelTo(ship->frame);
            planet->transform = glm::translate(translate) * glm::dmat4(planet->frame->getRotFrame()->orient);
        }
    }

    for(auto&& planet : planets) {
        // The terrain LOD may post async subdivision jobs (g.jobs); their
        // continuations run in the main loop's jobs.poll() BEFORE this
        // pass draws, so new children are attached before the render.
        planet->Update(camera, g.args.terrain_px, g.jobs);
        if(g.world_drawing == true) {
            planet->Draw(camera, sun, ship->frame);
        }
    }

    /*
      end 3d stuff drawn here
    */

    mu = ship->m_parent->mu;

    // surf pos??
    pos = com;
    /* orbital velocity */
    vel = ship->GetVel();

    // The orbit is a Kepler conic in the body's INERTIAL (non-rotating)
    // frame — that is the frame the spawn/switching code targets and
    // the frame in which the ship's trajectory is a conic.
    orbit_pos = pos;
    orbit_vel = vel;
    if(ship->frame->isRotFrame() == true) {
        Frame *inertial = ship->frame->getNonRotFrame();
        orbit_vel += ship->frame->GetStasisVelocity(orbit_pos);
        orbit_vel = ship->frame->GetOrientRelTo(inertial) * orbit_vel + ship->frame->GetVelocityRelTo(inertial);
        orbit_pos = ship->frame->GetOrientRelTo(inertial) * orbit_pos + ship->frame->GetPositionRelTo(inertial);
    }

    // Surface-relative state: the ship's position/velocity in the
    // ROTATING frame (i.e. relative to the ground).
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

    // Telemetry: e and |h| are the two conserved 2-body constants,
    // so a drifting plot = integrator drift; steps = burns/staging.
    energy_series.push(g.time, o.energy);
    angmom_series.push(g.time, o.ang_momentum);

    up = ship->partAxis(ship->controller, 1);
    facing = ship->partAxis(ship->controller, 2);
    other = ship->partAxis(ship->controller, 0);

    facing_dir = glm::normalize(facing);
    vel_dir = glm::normalize(vel);

    const glm::dvec3 _up = glm::normalize(pos);
    const glm::dvec3 _north = glm::normalize(projectVecOntoPlane(glm::dvec3(0, 1, 0), _up));
    const glm::dvec3 _east = glm::cross(_up, _north);

    ver_speed = glm::length(glm::proj(surf_vel, pos)); // m/s
    hor_speed2 = glm::length(projectVecOntoPlane(surf_vel, _up)); // m/s

    const glm::dvec3 groundHed = glm::normalize(projectVecOntoPlane(facing, _up));

    const double hedNorth = glm::dot(groundHed, _north);
    const double hedEast = glm::dot(groundHed, _east);
    heading = wrapAngleToPositive(atan2(hedEast, hedNorth));

    pitch = asin(glm::dot(_up, facing));
    roll =
        glm::orientedAngle(glm::normalize(projectVecOntoPlane(-pos, glm::normalize(facing))),
                           glm::normalize(-up),
                           glm::normalize(facing));

    // ImGui::Text("pos: %.2f %.2f %.2f", pos.x, pos.y, pos.z);
    // ImGui::Text("facing: %.2f %.2f %.2f", facing.x, facing.y, facing.z);
    // ImGui::Text("up: %.2f %.2f %.2f", up.x, up.y, up.z);
    // ImGui::Text("other: %.2f %.2f %.2f", other.x, other.y, other.z);
    // ImGui::Text("Ground hed: %.2f %.2f %.2f", groundHed.x, groundHed.y, groundHed.z);
    // ImGui::Text("Pitch: %.2f", glm::degrees(pitch));
    // ImGui::Text("Heading: %.2f", glm::degrees(heading));
    // ImGui::Text("up: %.2f, %.2f, %.2f", up.x, up.y, up.z);
    // ImGui::Text("facing: %.2f, %.2f, %.2f", facing.x, facing.y, facing.z);

    const glm::dvec3 dir = glm::normalize(surf_pos);

    longitude = atan2(dir.x, dir.z);
    latitude = asin(dir.y);

    // Atmosphere rims: transparent Fresnel shells, drawn over the opaque
    // bodies (and the starfield background, which was drawn first) so the
    // rim ring blends over them and the horizon haze blends over the
    // already-drawn terrain. Depth-write off; no-ops for bodies without an
    // atmosphere. See reports/atmosphere2026_08_25. Cloud decks draw first
    // in the loop (under the rim): the deck is the solid white layer, the
    // rim the thin air around it.
    if(g.world_drawing == true) {
        for(auto&& planet : planets) {
            if(!g.args.no_ocean) {
                planet->DrawOcean(camera, sun, ship->frame, g.time);
            }
            if(!g.args.no_clouds) {
                planet->DrawClouds(camera, sun, ship->frame, g.time);
            }
            if(!g.args.no_atmosphere) {
                planet->DrawAtmosphere(camera, sun, ship->frame);
            }
        }
    }

    /* draw engine plume */
    glm::dmat4 View = camera->GetView();
    glm::mat4 Projection = camera->GetProjection();
    if(ship->m_thrust > 0) {
        for(Part *p : ship->parts) {
            if(!p->isThruster()) { continue; }
            /* an air-breathing engine has no rocket plume (its exhaust is a
               short faint puff, not a flame) -- suppress the flame mesh.
               Thrust feedback comes from the ship's acceleration + HUD. */
            if(p->isJet()) { continue; }
            /* only engines actually thrusting THIS tick (armed in
               ApplyThrust: ignited = stage <= counter, AND drew propellant).
               m_thrust above is a coarse "something fired" flag, so without
               this check a lit stage would paint fake plumes on the engines
               of un-ignited higher stages. */
            if(p->armedThrust <= 0.0f) { continue; }
            /* the plume mesh is authored for the base part
               (radius 1 m, height 2 m): scale it to this thruster's
               size so the tail lands on the engine tail (-h/2) */
            const double radius = p->def->radius;
            const double height = p->def->height;
            /* Built from the part's world pose rather than read off
               Body::model_matrix: that field is a cache Body::Draw fills as
               a side effect, so using it here made the plume depend on
               Vehicle::Draw having already run for this ship this frame. */
            glm::dvec3 plumePos; glm::dmat3 plumeRot;
            ship->partWorldPose(p, plumePos, plumeRot);
            glm::dmat4 Model = glm::translate(plumePos) * glm::dmat4(plumeRot)
                * glm::dmat4(glm::dmat3(radius, 0.0, 0.0,
                                         0.0, radius, 0.0,
                                         0.0, 0.0, height / 2.0));
            // shifted into the render frame, like the view
            glm::mat4 ModelViewFloat = View * glm::translate(-camera->GetRenderOrigin()) * Model;
            g.partsshader->Bind();
            g.partsshader->setUniform_mat4(0, Projection * ModelViewFloat);
            g.partsshader->setUniform_mat4(1, glm::mat4(1.0)); // identity (GLM 1.0.0+: default ctor is zero)
            g.partsshader->setUniform_vec3(2, glm::vec3(1, 1, 1));
            // the plume is an unshadowed, untinted additive glow: pin the
            // shadow/alpha/tint uniforms rather than inheriting the last
            // part draw's values
            g.partsshader->setUniform_vec1(3, 1.0f);
            g.partsshader->setUniform_vec1(4, 1.0f);
            g.partsshader->setUniform_vec3(5, glm::vec3(1, 1, 1));

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

    /* draw the RCS plume: the temporary magical RCS applies its whole
       translation at the COM (applyRcsForce), so the puff is drawn from
       the COM along the armed ship-relative direction, reusing the
       engine plume's quad + texture. The quad's plane is spanned by the
       exhaust direction and the camera axis LEAST aligned with it, so
       the flat strip stays near screen-facing for every RCS direction
       (edge-on it would vanish). */
    if(ship->rcsFiring && glm::length2(ship->rcsDir) > 1e-12) {
        const glm::dvec3 z = ship->rcsWorldDir();  // thrust dir; the mesh extends toward -z
        const glm::dvec3 camR = glm::normalize(glm::cross(g.camera->forward, g.camera->up));
        const glm::dvec3 seed = (std::abs(glm::dot(camR, z)) < std::abs(glm::dot(g.camera->up, z)))
            ? camR : g.camera->up;
        const glm::dvec3 x = glm::normalize(seed - glm::dot(seed, z) * z);
        const glm::dvec3 y = glm::cross(z, x);
        /* 2 m wide, 3 m long off the COM (a radius-1 engine's plume is
           4 m long) -- long enough to clearly read as coming off the
           ship, since the tail starts at the COM inside the hull. The
           translate offset puts the mesh tail (local z = -1) on the COM. */
        const double sx = 0.5, sz = 0.75;
        glm::dmat4 Model = glm::translate(com + z * sz)
            * glm::dmat4(glm::dmat3(x, y, z))
            * glm::dmat4(glm::dmat3(sx, 0.0, 0.0,
                                         0.0, sx, 0.0,
                                         0.0, 0.0, sz));
        glm::mat4 ModelViewFloat = View * glm::translate(-camera->GetRenderOrigin()) * Model;
        g.partsshader->Bind();
        g.partsshader->setUniform_mat4(0, Projection * ModelViewFloat);
        g.partsshader->setUniform_mat4(1, glm::mat4(1.0)); // identity (GLM 1.0.0+: default ctor is zero)
        g.partsshader->setUniform_vec3(2, glm::vec3(1, 1, 1));

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

    /* Transfer planner: rebuild the target list, recompute the
       solution on input change or every 30 frames, and fire the
       --xfer-log (transferplanner.cpp). com / vel are this
       render pass's ship COM / velocity. */
    planner.update(com, vel);

    // P key: compute the porkchop plot (the one-shot flag is set in
    // poll_events; it runs here, where the planner + this pass's ship
    // snapshot both exist).
    if(g.porkchop_compute_requested) {
        planner.porkchopCompute();
        g.porkchop_compute_requested = false;
    }

    // M key: request the surface map (one-shot, same pattern as P). The
    // sweep runs on the background worker; the last map stays on screen
    // until the job lands.
    if(g.surfmap_compute_requested) {
        surfmapCompute(g);
        g.surfmap_compute_requested = false;
    }

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    g.front_indicator->pos = facing;
    // Fixed-orientation nose marker: the orbit camera already tracks the
    // ship's roll (up = the nose), so the billboard is roll-invariant and
    // no explicit roll term is needed to keep it aligned with the ship.
    g.front_indicator->Draw(camera, M_PI);
    g.prograde_indicator->pos = vel;
    g.prograde_indicator->Draw(camera, M_PI);
    g.retrograde_indicator->pos = - vel;
    g.retrograde_indicator->Draw(camera, M_PI);
    g.radial_in_indicator->pos = - pos;
    g.radial_in_indicator->Draw(camera, M_PI);
    g.radial_out_indicator->pos = pos;
    g.radial_out_indicator->Draw(camera, M_PI);
    g.normal_plus_indicator->pos = glm::cross(pos, vel);
    g.normal_plus_indicator->Draw(camera, M_PI);
    g.normal_minus_indicator->pos = -glm::cross(pos, vel);
    g.normal_minus_indicator->Draw(camera, M_PI);
    // Transfer burn direction (TRANSFER window target selected):
    // KSP-blue prograde icon pointing where the departure burn goes.
    if(planner.xfer.valid && glm::length(planner.xfer.burn_dir) > 0.0) {
        g.burn_indicator->pos = planner.xfer.burn_dir;
        g.burn_indicator->Draw(camera, M_PI);
    }
    // Target ship's relative velocity: two pink markers, shown when a ship
    // is targeted in the TRANSFER window, or when a docking port on another
    // ship is selected (the dock intent). The prograde (diamond) icon marks
    // you − target, the retrograde (X) icon marks target − you -- opposite
    // sides of the ship, both the same magnitude.
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
        // Both velocities in the shared inertial frame (the same idiom as
        // the planner's shipInertial/targetInertial: stasis + frame
        // velocity carry a rotating/moving frame's contribution), then the
        // difference expressed in this render frame (ship->frame).
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
            // you − target: the prograde (diamond) icon, on the opposite side.
            g.relvel_indicator->pos = -relvel;
            g.relvel_indicator->Draw(camera, M_PI);
            // target − you: the retrograde (X) icon.
            g.relvel_retro_indicator->pos = relvel;
            g.relvel_retro_indicator->Draw(camera, M_PI);
        }
    }
    // horizon_indicator->pos = groundHed;
    // horizon_indicator->Draw(camera, M_PI);

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

void drawVab(Game &g) {
    /* The build tree lives in its own frame S; center the render frame on the
       ship and orbit around it. ref = identity so screen-up is world +Z (the
       ship stands nose-up, as on the pad). */
    Camera *cam = g.camera;
    cam->renderOrigin = g.vab_center;
    cam->ref = glm::dmat3(1.0);
    cam->Follow(g.vab_center);
    cam->ComputeView();

    glm::vec3 sunlight = glm::normalize(glm::vec3(0.4f, 0.8f, 0.35f));
    for(size_t i = 0; i < g.vab.parts.size(); i++) {
        const BuildPart &bp = g.vab.parts[i];
        if(bp.def == nullptr) { continue; }
        Mesh *m = get_mesh(std::string("./res/") + bp.def->mesh);
        Texture *t = get_texture(std::string("./res/") + bp.def->texture);
        if(m == nullptr || t == nullptr) { continue; }
        const glm::dmat4 model = glm::translate(bp.localPos)
                               * glm::dmat4(bp.localRot);
        DrawOpts opts;
        if((int)i == g.vab_selected) { opts.tint = glm::vec3(1.0f, 0.75f, 0.2f); }
        else if((int)i == g.vab_hover) { opts.tint = glm::vec3(0.6f, 1.0f, 0.6f); }
        DrawModelAt(cam, m, g.partsshader, t, model, sunlight, 1.0f,
                    glm::dmat4(1.0), opts);
    }

    // the armed part's translucent ghost at the hovered attach target
    if(g.vab_ghostValid && !g.vab_armed.empty()) {
        const PartDef *ad = g.ships.catalog().find(g.vab_armed);
        if(ad != nullptr) {
            Mesh *gm = get_mesh(std::string("./res/") + ad->mesh);
            Texture *gt = get_texture(std::string("./res/") + ad->texture);
            if(gm != nullptr && gt != nullptr) {
                DrawOpts go;
                go.alpha = 0.4f;
                // the primary ghost + its radial-symmetry clones (all
                // unshifted S-frame poses: DrawModelAt applies the
                // -renderOrigin (= -vab_center) shift like every model here)
                const glm::dmat4 gmodel = glm::translate(g.vab_ghostPos)
                                        * glm::dmat4(g.vab_ghostRot);
                DrawModelAt(cam, gm, g.partsshader, gt, gmodel, sunlight, 1.0f,
                            glm::dmat4(1.0), go);
                for(size_t k = 0; k < g.vab_ghostClones.size(); k++) {
                    const AttachPose &cp = g.vab_ghostClones[k].pose;
                    const glm::dmat4 cmodel = glm::translate(cp.childPos)
                                            * glm::dmat4(cp.childRot);
                    DrawModelAt(cam, gm, g.partsshader, gt, cmodel, sunlight,
                                1.0f, glm::dmat4(1.0), go);
                }
            }
        }
    }
}
