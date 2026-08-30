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
#include <GL/glew.h>   // glBlendFunc / glLineWidth / the GL enums

#include "billboard.h"   // Billboard::Draw + the icon pos
#include "mesh.h"        // Mesh::Draw (the reference skylines)
#include "model.h"       // Model (the engine plume)
#include "orbit.h"       // computeOrbitElements + the plane math
#include "physics.h"     // getRelAxis_ / debug_draw
#include "shader.h"      // Shader::Bind / setUniform_*
#include "skybox.h"      // Skybox::Draw
#include "texture.h"     // the plume texture's id

// GLM's gtx extensions (the attitude math) hard-error without this.
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/projection.hpp>    // glm::proj
#include <glm/gtx/vector_angle.hpp>  // glm::orientedAngle

void draw3d(Game &g, TransferPlanner &planner) {
    // The pass body is verbatim from main's render section; its globals
    // are Game members (aliased so the body reads the same).
    Vehicle *ship = g.ship;
    Ships &ships = g.ships;
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
            camera->ref = glm::dmat3(getRelAxis_(ship->controller, 0),
                                     getRelAxis_(ship->controller, 1),
                                     getRelAxis_(ship->controller, 2));
        } else {
            TerrainBody *b = g.focusTargets[g.focusBody].body;
            camera->ref = (b == ship->m_parent && ship->frame->isRotFrame())
                ? glm::dmat3(1.0)
                : glm::dmat3(b->frame->getRotFrame()->orient);
        }
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

    if(g.world_drawing == true) {
        // one per home body; StaticBuilding::Draw culls itself when
        // the active ship is not on that body
        for(auto &kv : ships.pads()) {
            kv.second->Draw(camera, ship->m_parent, ship->frame);
        }
        // render frame = the active ship's frame; idle ships in a
        // different frame are transformed into it in Vehicle::Draw
        for(auto *s : ships) { s->Draw(camera, ship->frame); }
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
        planet->Update(camera, g.args.terrain_px);
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

    up = getRelAxis_(ship->controller, 1);
    facing = getRelAxis_(ship->controller, 2);
    other = getRelAxis_(ship->controller, 0);

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

    if(g.draw_starfield) {
        g.skybox->Draw(camera, g.skyboxshader, sun->frame->GetOrientRelTo(ship->frame));
    }

    // Atmosphere rims: transparent Fresnel shells, drawn after the
    // skybox (the starfield is the background) so the rim ring blends
    // over it and the horizon haze blends over the already-drawn
    // terrain. Depth-write off; no-ops for bodies without an
    // atmosphere. See reports/atmosphere2026_08_25.
    if(g.world_drawing == true) {
        for(auto&& planet : planets) {
            planet->DrawAtmosphere(camera, sun, ship->frame);
        }
    }

    /* draw engine plume */
    glm::dmat4 View = camera->GetView();
    glm::mat4 Projection = camera->GetProjection();
    if(ship->m_thrust > 0) {
        for(size_t t = 0; t < ship->m_thrusters.size(); t++) {
            /* the plume mesh is authored for the base part
               (radius 1 m, height 2 m): scale it to this thruster's
               size so the tail lands on the engine tail (-h/2) */
            const glm::dvec2 &dim = ship->m_thrusterDims[t];
            glm::dmat4 Model = ship->m_thrusters[t]->model_matrix
                * glm::dmat4(glm::dmat3(dim.x, 0.0, 0.0,
                                         0.0, dim.x, 0.0,
                                         0.0, 0.0, dim.y / 2.0));
            // shifted into the render frame, like the view
            glm::mat4 ModelViewFloat = View * glm::translate(-camera->GetRenderOrigin()) * Model;
            g.engine_plume_model->shader->Bind();
            g.engine_plume_model->shader->setUniform_mat4(0, Projection * ModelViewFloat);
            g.engine_plume_model->shader->setUniform_mat4(1, glm::mat4(1.0)); // identity (GLM 1.0.0+: default ctor is zero)
            g.engine_plume_model->shader->setUniform_vec3(2, glm::vec3(1, 1, 1));

            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, g.engine_plume_model->texture->id);
            glEnable(GL_BLEND);
            glBlendFunc(GL_ONE, GL_ONE);
            glDisable(GL_CULL_FACE);
            g.engine_plume_model->mesh->Draw();
            glEnable(GL_CULL_FACE);
            glDisable(GL_BLEND);
            glBindTexture(GL_TEXTURE_2D, 0);
        }
    }
    /* end draw engine plume */

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

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    g.front_indicator->pos = facing;
    g.front_indicator->Draw(camera, M_PI /* <- ?? */ + roll);
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
