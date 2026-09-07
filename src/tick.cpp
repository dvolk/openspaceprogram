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

#include "keys.h"                // Slot, slotHeld, slotSimKey (the command map)
#include "eva.h"                 // evaArmCommands (the kerbal's controls)
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

    // The canonical ship list this tick walks: bodies in file order, each
    // body's ships, each ship's crew right after it (collectVehicles).
    // Snapshotted once up front so a ship that crosses a SoI boundary mid-tick
    // (switchFrames moves it between bodies' lists) is still updated exactly
    // once -- the pointers stay valid across the move, and substeps never
    // move ships.
    std::vector<Vehicle *> all = collectVehicles(g.sys);

    // clear stats and stuff; sync the exhaust-velocity test scale
    // (--exhaust-scale / the Settings slider) onto every ship.
    for(auto *s : all) {
        s->m_thrust = 0.0;
        s->exhaust_scale = g.args.exhaust_scale;
    }

    while (g.accumulator >= g.dt) {
        // is this logic? ;_;
        // Thrust and rotation are armed once per tick (if the keys are
        // held, below) and then re-applied before every substep; clear
        // them first so a tick without the keys doesn't keep pushing or
        // slewing from the last one. RCS translation is armed the same way
        // (setRcsDir below) and consumed in applyRcsForce.
        g.ship->clearThrust();
        g.ship->clearRotCmd();
        g.ship->clearRcs();

        const Uint8* key = SDL_GetKeyboardState(NULL);
        const Uint16 modState = SDL_GetModState();
        /* --sim-press: a synthetic key is "down" from its down time to
           its up time. SDL_PushEvent does not update the key array above
           (verified on this SDL), so a held command ORs in each entry's
           window. Synthetic keys carry a scancode but no modifier state,
           so they only back a PLAIN binding (slotSimKey). */
        auto slotActive = [&](Slot s) -> bool {
            if(slotHeld(s, key, modState, g.binds)) { return true; }
            for(size_t i = 0; i < g.args.sim_presses.size(); i++) {
                if(g.args.sim_presses[i].down_sent && !g.args.sim_presses[i].up_sent
                   && slotSimKey(s, g.args.sim_presses[i].sc, g.binds)) {
                    return true;
                }
            }
            return false;
        };

        if (g.camera->mode == CAM_FREE) {
            if (slotActive(Slot::CamForward)) { g.camera->MoveForward(g.cam_speed); }
            else if (slotActive(Slot::CamBack)) { g.camera->MoveForward(-g.cam_speed); }

            if (slotActive(Slot::CamStrafeLeft)) { g.camera->MoveRight(-g.cam_speed); }
            else if (slotActive(Slot::CamStrafeRight)) { g.camera->MoveRight(g.cam_speed); }

            if (slotActive(Slot::CamRollLeft)) { g.camera->Roll(-0.05); }
            else if (slotActive(Slot::CamRollRight)) { g.camera->Roll(0.05); }

            if (slotActive(Slot::CamUp)) { g.camera->MoveUp(g.cam_speed); }
            else if (slotActive(Slot::CamDown)) { g.camera->MoveUp(-g.cam_speed); }
        }

        if (g.camera->mode == CAM_ORBIT) {
            bool game_running = (g.time_accel > 0);
            /* touching the controls wakes a railed active ship: it
               re-enters physics (you cannot maneuver on rails). A rails
               warp (accel > 10) drops to 1x on the way out; a ship railed
               at a low warp (an SOI handoff drops the warp to 1 while the
               ship keeps its conic) just wakes in place. */
            if(g.ship->onRails && g.time_accel > 0) {
                // any flight control wakes a railed ship; in EVA the
                // kerbal's jump (Space) is the extra -- the walk + up/down
                // keys share the flight physical keys (W/S/A/D/Q/E/R/F).
                bool wake = slotActive(Slot::PitchUp) || slotActive(Slot::PitchDown) ||
                            slotActive(Slot::YawLeft) || slotActive(Slot::YawRight) ||
                            slotActive(Slot::RollLeft) || slotActive(Slot::RollRight) ||
                            slotActive(Slot::Thrust) || slotActive(Slot::KillRot) ||
                            slotActive(Slot::ThrottleUp) || slotActive(Slot::ThrottleDown) ||
                            slotActive(Slot::RcsForward) || slotActive(Slot::RcsBack) ||
                            slotActive(Slot::RcsUp) || slotActive(Slot::RcsDown) ||
                            slotActive(Slot::RcsLeft) || slotActive(Slot::RcsRight);
                if(g.ship->isEva()) { wake = wake || slotActive(Slot::Space); }
                if(wake) {
                    g.ship->leaveRails();
                    if(g.time_accel >= kRailsWarp) {
                        g.time_accel = 1;
                        g.toast("Control input: left the rails, warp 1x");
                    }
                    printf("Control input: '%s' left the rails, warp -> %d\n",
                           g.ship->name.c_str(), g.time_accel);
                }
            }
            if(g.ship->isEva()) {
                /* EVA: arm the kerbal's walking/RCS controls for this tick
                   (src/eva.cpp). Like the ship's Command path, nothing is
                   armed while paused. */
                if(game_running) { evaArmCommands(g, slotActive); }
            } else {
            /* The Autopilot window's engaged mode (set by its toggle
               buttons): apply it after clearRotCmd (above) so the ship
               keeps slewing toward the target and holding, waking a railed
               ship just like a control key. The X kill-rot key below
               overrides it while held. */
            if(game_running && g.ship->slewRequest != SlewNone) {
                if(g.ship->onRails) {
                    g.ship->leaveRails();
                    if(g.time_accel >= kRailsWarp) {
                        g.time_accel = 1;
                        g.toast("Control input: left the rails, warp 1x");
                    }
                }
                g.ship->slew = g.ship->slewRequest;
            }
            // Control-axis flips. The baseline amounts below already bake in
            // the default orientation: viewed from the front the ship's
            // left/right are mirrored, so yaw and roll are pre-flipped to
            // respond in your screen direction (pitch is not mirrored, so it
            // is not). Each flip_* setting (Settings -> Controls) inverts its
            // axis away from that default.
            const float f_pitch = g.flip_pitch ? -1.0f : 1.0f;
            const float f_yaw   = g.flip_yaw   ? -1.0f : 1.0f;
            const float f_roll  = g.flip_roll  ? -1.0f : 1.0f;
            // pitch: about the ship's right axis (PitchUp/Down, default W/S)
            if (slotActive(Slot::PitchUp)) { g.ship->Command(ShipCmd(Pitch,  f_pitch * +1.0f), game_running); }
            if (slotActive(Slot::PitchDown)) { g.ship->Command(ShipCmd(Pitch,  f_pitch * -1.0f), game_running); }
            // yaw: about the ship's up axis (baseline pre-flipped)
            if (slotActive(Slot::YawLeft)) { g.ship->Command(ShipCmd(Yaw,    f_yaw   * -1.0f), game_running); }
            if (slotActive(Slot::YawRight)) { g.ship->Command(ShipCmd(Yaw,    f_yaw   * +1.0f), game_running); }
            // roll: about the ship's nose (baseline pre-flipped)
            if (slotActive(Slot::RollLeft)) { g.ship->Command(ShipCmd(Roll,   f_roll  * -1.0f), game_running); }
            if (slotActive(Slot::RollRight)) { g.ship->Command(ShipCmd(Roll,   f_roll  * +1.0f), game_running); }

            // The thrust latch (g.thrust_latched, toggled by the ThrustLatch
            // slot) keeps the engines lit even with the thrust key released:
            // it ORs into the held check so Command(Thrust) re-arms every tick.
            if (slotActive(Slot::Thrust) || g.thrust_latched) { g.ship->Command(ShipCmd(Thrust), game_running, g.dt * g.time_accel); }
            if (slotActive(Slot::KillRot)) { g.ship->Command(ShipCmd(KillRot), game_running); }

            if (slotActive(Slot::ThrottleUp)) { g.ship->Command(ShipCmd(ThrottleUp), game_running); }
            if (slotActive(Slot::ThrottleDown)) { g.ship->Command(ShipCmd(ThrottleDown), game_running); }

            // RCS translation (camera-relative, KSP-style): arm a unit
            // direction from the held slots against the camera basis. forward
            // and up are the live view axes (both set in ComputeView, valid
            // here in orbit mode); right is derived (cross) since the
            // camera's own `right` member is only kept current in free mode.
            // The direction is stored on the ship and consumed in
            // applyRcsForce before every substep (the EVA kerbal's own
            // translation runs its own path, so this is ship-only).
            {
                const glm::dvec3 f = g.camera->forward;
                const glm::dvec3 u = g.camera->up;
                const glm::dvec3 r = glm::normalize(glm::cross(f, u));
                glm::dvec3 d(0.0);
                if (slotActive(Slot::RcsForward)) { d += f; }
                if (slotActive(Slot::RcsBack))    { d -= f; }
                if (slotActive(Slot::RcsUp))      { d += u; }
                if (slotActive(Slot::RcsDown))    { d -= u; }
                if (slotActive(Slot::RcsLeft))    { d -= r; }
                if (slotActive(Slot::RcsRight))   { d += r; }
                if (game_running && glm::length2(d) > 1e-12) {
                    g.ship->setRcsDir(glm::normalize(d));
                }
            }
            }
        }

        // Advance the analytic sim clock by exactly the physics timestep
        // (g.dt * g.time_accel), matching physics_tick(g.dt * g.time_accel) below.
        // The frame tree's analytic motion must run on the same clock as
        // the ship's integration. The old 1/60.0 constant disagreed with
        // dt (1/50), so the physics clock ran 20% faster than the analytic
        // body positions and the ship systematically outran the planets.
        g.time += g.dt * g.time_accel;
        g.phys_steps++;   // one substep ran (the --perf breakdown counts these)

        if(g.time_accel != 0) {
            // The active ship's SOI owner before this tick's frame
            // bookkeeping (checked after the branch, below): crossing into
            // a different body's SOI drops warp to 1x.
            TerrainBody *soiOwner = g.ship->m_parent;

            g.sun->frame->UpdateOrbitRails(g.time);

            // Proximity: wake ships near the active ship (and, on a close
            // approach, wake the active ship + cap the warp). Runs before the
            // branch so a dropped warp routes this tick into the physics path.
            g.updateProximity();

            if(g.time_accel >= kRailsWarp) {
                /* Rails warp: every ship coasts analytically (or sits
                   frozen on the ground) and the Bullet world is not
                   stepped at all -- O(ships) per tick instead of a
                   substep count that explodes with the accel. */
                for(auto *s : all) { s->railsTick(g.dt * g.time_accel); }
            } else {

            // per-ship SOI bookkeeping: each ship tracks its own
            // position in the shared frame tree (an idle ship can
            // cross a boundary while we fly another one). Railed
            // ships advance their analytic conic here instead --
            // exact for any step size, at any time accel.
            for(auto *s : all) {
                if(s->onRails) { s->railsTick(g.dt * g.time_accel); }
                else { s->switchFrames(); }
                /* The compound's COM has to track the mass distribution, and
                   a burn moves it. Checked here rather than at each mass
                   writer so one call site covers all of them, and it only
                   rebuilds once the drift is worth it (see
                   Vehicle::refreshCompound). Not run on the rails-warp path
                   above: nothing burns there, and that path is deliberately
                   O(ships) per tick. */
                s->refreshCompound();
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
                // every NON-RAILED ship feels its own gravity + armed
                // control forces each substep (ships: thrust + rotation;
                // the EVA kerbal: walking/RCS -- see applyControlForces);
                // physics_tick then steps the shared Bullet world all of
                // them at once. Railed ships have no bodies in the world
                // -- their conic already advanced this tick in railsTick.
                for(auto *s : all) {
                    if(s->onRails) { continue; }
                    s->processGravity();
                    /* Electrical resolution BEFORE the control forces, so
                       the power gate (powered_) is current when the
                       reaction wheels are applied (a ship that runs out of
                       power becomes uncontrolled this substep). */
                    s->powerTick(h);
                    s->applyControlForces(h);
                }
                physics_tick(h);
            }

            } // end physics-warp branch (g.time_accel < kRailsWarp)

            /* SOI handoff: the active ship crossed into a different body's
               SOI (or back out to the parent's) -- drop warp to 1x so the
               encounter is playable instead of warped straight through.
               A pause (0) is the player's call and stays put. */
            if(soiOwner != g.ship->m_parent && g.time_accel > 1) {
                printf("SOI switch: '%s' now around %s (was %s), warp -> 1\n",
                       g.ship->name.c_str(), g.ship->m_parent->name.c_str(),
                       soiOwner->name.c_str());
                g.time_accel = 1;
                g.toast("SOI: now around %s, warp 1x",
                        g.ship->m_parent->name.c_str());
            }

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

            /* --fuel-log: each fuel group's fuel mass + the fuel links,
               once per 0.5 s of sim time (the heavy_two radial drain
               instrument: the symmetric radial groups must stay equal). */
            if(g.args.fuel_log) {
                static double last_fuel_log = -1e30;
                if(g.time - last_fuel_log >= 0.5) {
                    last_fuel_log = g.time;
                    g.ship->fuel_log(g.time);
                }
            }

            /* --drain-log: each fuel group's drain rate (kg/s), once per
               0.5 s of sim time -- the "how is the fuel flowing"
               instrument: the outer groups drain, the inner stay at 0. */
            if(g.args.drain_log) {
                static double last_drain_log = -1e30;
                if(g.time - last_drain_log >= 0.5) {
                    last_drain_log = g.time;
                    g.ship->drain_log(g.time);
                }
            }

            /* --power-log: the ship's power balance (generation, constant
               draw, stored charge, and the wheel gate) once per 0.5 s of
               sim time -- the "is the ship losing power?" instrument. */
            if(g.args.power_log) {
                static double last_power_log = -1e30;
                if(g.time - last_power_log >= 0.5) {
                    last_power_log = g.time;
                    g.ship->power_log(g.time);
                }
            }

            /* --slew-log: autopilot (prograde/retrograde/kill-rot) state,
               once per 0.1 s of sim time -- fine enough to resolve the
               slew's ~1 s timescale and any oscillation around the target. */
            if(g.args.slew_log_enabled) {
                static double last_slew_log = -1e30;
                if(g.time - last_slew_log >= 0.1) {
                    last_slew_log = g.time;
                    g.ship->slew_log(g.time);
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

        // --att-log: the ship's nose + angular velocity (attitude-physics e2e)
        if(g.args.att_log) {
            const Uint32 now_ms = SDL_GetTicks();
            if(now_ms - g.att_log_last_ms >= g.orbit_log_interval_ms) {
                g.att_log_last_ms = now_ms;
                g.ship->att_log(g.time);
            }
        }

        /* --compound-check: every ship's compound-vs-parts agreement. Not
           gated on time_accel (a paused ship still has live part poses to
           compare) and over `all`, not just the active ship: the idle and
           railed ships are where the frozen-in deformation shows up. */
        if(g.args.compound_check) {
            static Uint32 last_compound_ms = 0;
            const Uint32 now_ms = SDL_GetTicks();
            if(now_ms - last_compound_ms >= g.orbit_log_interval_ms) {
                last_compound_ms = now_ms;
                for(auto *s : all) { s->compoundCheck(g.time); }
            }
        }

        // --eva-log: the kerbal's mode + state (the EVA e2e assertions)
        if(g.args.eva_log && g.ship->isEva()) {
            const Uint32 now_ms = SDL_GetTicks();
            if(now_ms - g.eva_log_last_ms >= g.orbit_log_interval_ms) {
                g.eva_log_last_ms = now_ms;
                Kerbal *k = static_cast<Kerbal *>(g.ship);
                const glm::dvec3 p = k->get_center_of_mass();
                const glm::dvec3 v = k->GetVel();
                printf("[evalog] t=%.1fs mode=%s grounded=%d "
                       "pos=[%.1f %.1f %.1f] vel=[%.2f %.2f %.2f] alt=%.2f m "
                       "mass=%.3fkg\n",
                       g.time, (k->mode == EVA_GROUND) ? "ground" : "space",
                       (int)k->grounded, p.x, p.y, p.z, v.x, v.y, v.z,
                       glm::length(p) - (double)k->m_parent->GetTerrainHeight(
                           glm::vec3(glm::normalize(p))),
                       k->getMass());
                fflush(stdout);
            }
        }

        g.accumulator -= g.dt;
    }

    g.redraw = true;
}
