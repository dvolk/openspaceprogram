// tick.cpp -- the game's fixed-timestep logic tick (declared in tick.h).
//
// This was the "LOGIC" section inside main's loop: the frame accumulator,
// the per-tick command arming (thrust / rotation from the held keys, plus
// the free-camera WASD), the substepped physics step, and the --spin-log /
// --orbit-log / --dbg-log output. Every state access goes through Game
// (main locals -> g members). The EVENTS section is in events.cpp and the
// RENDER section stays in main.
#include "tick.h"

#include <cstdio>

#include "SDL2/SDL_scancode.h"   // SDL_SCANCODE_* (the command keys)
#include "orbit.h"               // OrbitElements, computeOrbitElements
#include "physics.h"             // physics_tick()

void tick(Game &g) {
    double newTime = (double)(SDL_GetTicks()) * 0.001;
    double frameTime = newTime - g.currentTime;
    g.currentTime = newTime;
    g.accumulator += frameTime;

    if(g.accumulator > 10 * g.dt) {
        g.accumulator = 10 * g.dt;
    }

    // clear stats and stuff
    for(auto *s : g.ships) { s->m_thrust = 0.0; }

    while (g.accumulator >= g.dt) {
        // is this logic? ;_;
        // Thrust and rotation are armed once per tick (if the keys are
        // held, below) and then re-applied before every substep; clear
        // them first so a tick without the keys doesn't keep pushing or
        // slewing from the last one.
        g.ship->clearThrust();
        g.ship->clearRotCmd();

        const Uint8* key = SDL_GetKeyboardState(NULL);
        /* --sim-press: a synthetic key is "down" from its down time to
           its up time. SDL_PushEvent does not update the state array
           above (verified on this SDL), so held commands OR in each
           entry's window instead. */
        auto isDown = [&](SDL_Scancode sc) -> bool {
            if(key[sc]) { return true; }
            for(size_t i = 0; i < g.args.sim_presses.size(); i++) {
                if(g.args.sim_presses[i].sc == sc && g.args.sim_presses[i].down_sent
                   && !g.args.sim_presses[i].up_sent) {
                    return true;
                }
            }
            return false;
        };

        if (g.camMode == CAM_FREE) {
            if (isDown(SDL_SCANCODE_W)) { g.camera->MoveForward(g.cam_speed); }
            else if (isDown(SDL_SCANCODE_S)) { g.camera->MoveForward(-g.cam_speed); }

            if (isDown(SDL_SCANCODE_A)) { g.camera->MoveRight(-g.cam_speed); }
            else if (isDown(SDL_SCANCODE_D)) { g.camera->MoveRight(g.cam_speed); }

            if (isDown(SDL_SCANCODE_Q)) { g.camera->Roll(-0.05); }
            else if (isDown(SDL_SCANCODE_E)) { g.camera->Roll(0.05); }

            if (isDown(SDL_SCANCODE_LSHIFT) || isDown(SDL_SCANCODE_RSHIFT)) { g.camera->MoveUp(g.cam_speed); }
            else if (isDown(SDL_SCANCODE_LCTRL) || isDown(SDL_SCANCODE_RCTRL)) { g.camera->MoveUp(-g.cam_speed); }
        }

        if (g.camMode == CAM_ORBIT) {
            bool game_running = (g.time_accel > 0);
            /* touching the controls wakes a ship parked on rails: it
               re-enters physics and rails warp drops to 1x (you cannot
               maneuver on rails). */
            if(g.ship->onRails && g.time_accel >= kRailsWarp) {
                if(isDown(SDL_SCANCODE_W) || isDown(SDL_SCANCODE_S) ||
                   isDown(SDL_SCANCODE_A) || isDown(SDL_SCANCODE_D) ||
                   isDown(SDL_SCANCODE_Q) || isDown(SDL_SCANCODE_E) ||
                   isDown(SDL_SCANCODE_I) || isDown(SDL_SCANCODE_X) ||
                   isDown(SDL_SCANCODE_B) || isDown(SDL_SCANCODE_N) ||
                   isDown(SDL_SCANCODE_R) || isDown(SDL_SCANCODE_F)) {
                    g.ship->leaveRails();
                    g.time_accel = 1;
                    printf("Control input: '%s' left the rails, warp -> 1\n",
                           g.ship->name.c_str());
                }
            }
            // pitch
            if (isDown(SDL_SCANCODE_W)) { g.ship->Command(ShipCmd(Pitch, +1.0f), game_running); }
            if (isDown(SDL_SCANCODE_S)) { g.ship->Command(ShipCmd(Pitch, -1.0f), game_running); }
            // yaw
            if (isDown(SDL_SCANCODE_A)) { g.ship->Command(ShipCmd(Yaw, +1.0f), game_running); }
            if (isDown(SDL_SCANCODE_D)) { g.ship->Command(ShipCmd(Yaw, -1.0f), game_running); }
            // roll
            if (isDown(SDL_SCANCODE_Q)) { g.ship->Command(ShipCmd(Roll, +1.0f), game_running); }
            if (isDown(SDL_SCANCODE_E)) { g.ship->Command(ShipCmd(Roll, -1.0f), game_running); }

            if (isDown(SDL_SCANCODE_I)) { g.ship->Command(ShipCmd(Thrust), game_running, g.dt * g.time_accel); }
            if (isDown(SDL_SCANCODE_X)) { g.ship->Command(ShipCmd(KillRot), game_running); }

            if (isDown(SDL_SCANCODE_B)) { g.ship->Command(ShipCmd(Prograde), game_running); }
            if (isDown(SDL_SCANCODE_N)) { g.ship->Command(ShipCmd(Retrograde), game_running); }

            if (isDown(SDL_SCANCODE_R)) { g.ship->Command(ShipCmd(ThrottleUp), game_running); }
            if (isDown(SDL_SCANCODE_F)) { g.ship->Command(ShipCmd(ThrottleDown), game_running); }
        }

        // Advance the analytic sim clock by exactly the physics timestep
        // (g.dt * g.time_accel), matching physics_tick(g.dt * g.time_accel) below.
        // The frame tree's analytic motion must run on the same clock as
        // the ship's integration. The old 1/60.0 constant disagreed with
        // dt (1/50), so the physics clock ran 20% faster than the analytic
        // body positions and the ship systematically outran the planets.
        g.time += g.dt * g.time_accel;

        if(g.time_accel != 0) {
            g.sun->frame->UpdateOrbitRails(g.time);

            if(g.time_accel >= kRailsWarp) {
                /* Rails warp: every ship coasts analytically (or sits
                   frozen on the ground) and the Bullet world is not
                   stepped at all -- O(ships) per tick instead of a
                   substep count that explodes with the accel. */
                for(auto *s : g.ships) { s->railsTick(g.dt * g.time_accel); }
            } else {

            // per-ship SOI bookkeeping: each ship tracks its own
            // position in the shared frame tree (an idle ship can
            // cross a boundary while we fly another one). Railed
            // ships advance their analytic conic here instead --
            // exact for any step size, at any time accel.
            for(auto *s : g.ships) {
                if(s->onRails) { s->railsTick(g.dt * g.time_accel); }
                else { s->switchFrames(); }
            }

            // Integrate the (time-accelerated) step in substeps,
            // re-applying gravity + the rotating-frame fictitious forces
            // + the engine thrust + the armed rotation commands before
            // EACH substep. Two reasons:
            //  1. Bullet clears accumulated forces at the end of every
            //     stepSimulation call, so applying gravity once and then
            //     stepping multiple substeps would leave the ship
            //     force-free for all but the first substep.
            //  2. Re-applying per substep keeps the central-force
            //     direction and the velocity-dependent Coriolis term
            //     accurate across the step instead of frozen at the
            //     step's start.
            // Keep >=3 substeps so low-accel behavior matches the old
            // 3-substep step, and grow the count so the substep stays
            // <= kMaxSubStep at high time-accel.
            const double step = g.dt * g.time_accel;
            const double kMaxSubStep = 0.1;
            int n = 3;
            int need = (int)(step / kMaxSubStep + 0.5);
            if (need > n) { n = need; }
            if (n > 2000) { n = 2000; }
            const double h = step / n;
            for (int i = 0; i < n; i++) {
                // every NON-RAILED ship feels its own gravity/thrust/
                // rotation each substep; physics_tick then steps the
                // shared Bullet world all of them at once. Railed ships
                // have no bodies in the world -- their conic already
                // advanced this tick in railsTick.
                for(auto *s : g.ships) {
                    if(s->onRails) { continue; }
                    s->processGravity();
                    s->applyThrustForce();
                    s->applyRotationForce(h);
                }
                physics_tick(h);
            }

            } // end physics-warp branch (g.time_accel < kRailsWarp)

            /* --spin-log (or --radial-test): spin diagnostics, once per
               0.5 s of sim time (after the last substep's solve, so the
               reported impulses are that solve's). */
            if(g.args.spin_log_enabled || !g.args.radial_test.empty()) {
                static double last_spin_log = -1e30;
                if(g.time - last_spin_log >= 0.5) {
                    last_spin_log = g.time;
                    spin_log(g.ship, g.time);
                }
            }
        }

        // --orbit-log: orbital elements, fit in the body's inertial
        // frame, where the ship's trajectory is a Kepler conic.
        if(g.args.orbit_log) {
            const Uint32 now_ms = SDL_GetTicks();
            if(now_ms - g.orbit_log_last_ms >= g.orbit_log_interval_ms) {
                g.orbit_log_last_ms = now_ms;

                const double mu = g.ship->m_parent->mu;
                glm::dvec3 o_pos = g.ship->get_center_of_mass();
                glm::dvec3 o_vel = g.ship->GetVel();
                if(g.ship->frame->isRotFrame()) {
                    Frame *inertial = g.ship->frame->getNonRotFrame();
                    o_vel += g.ship->frame->GetStasisVelocity(o_pos);
                    o_vel = g.ship->frame->GetOrientRelTo(inertial) * o_vel
                          + g.ship->frame->GetVelocityRelTo(inertial);
                    o_pos = g.ship->frame->GetOrientRelTo(inertial) * o_pos
                          + g.ship->frame->GetPositionRelTo(inertial);
                }
                OrbitElements o = computeOrbitElements(o_pos, o_vel, mu);
                printf("[orbitlog] t=%.1fs frame=\"%s\" r=%.6g m v=%.6g m/s "
                       "sma=%.6g m ecc=%.6g peri=%.6g m apo=%.6g m "
                       "inc=%.4f deg T=%.6g s ttAp=%.6g s ttPe=%.6g s "
                       "|h|=%.6f m2/s E=%.6f J/kg\n",
                       g.time, g.ship->frame->name.c_str(), o.distance, o.speed,
                       o.semi_major, o.ecc, o.periapsis, o.apoapsis,
                       glm::degrees(o.inclination), o.period,
                       o.time_to_apo, o.time_to_peri,
                       o.ang_momentum, o.energy);
                fflush(stdout);
            }
        }

        // --dbg-log: ship pos/alt/vel in its own frame
        if(g.args.dbg_log) {
            const Uint32 now_ms = SDL_GetTicks();
            if(now_ms - g.dbg_log_last_ms >= g.orbit_log_interval_ms) {
                g.dbg_log_last_ms = now_ms;
                glm::dvec3 p = g.ship->get_center_of_mass();
                glm::dvec3 v = g.ship->GetVel();
                double r = glm::length(p);
                double alt = r - g.ship->m_parent->GetTerrainHeight(glm::normalize(p));
                printf("[dbg] t=%.1fs pos=[%.1f %.1f %.1f] alt=%.1f m "
                       "vel=[%.2f %.2f %.2f] |v|=%.2f m/s\n",
                       g.time, p.x, p.y, p.z, alt, v.x, v.y, v.z,
                       glm::length(v));
                fflush(stdout);
            }
        }

        g.accumulator -= g.dt;
    }

    g.redraw = true;
}
