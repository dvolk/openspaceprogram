// tick.cpp -- the fixed-timestep logic tick (see tick.h).
#include "tick.h"

#include <cstdio>

#include "keys.h"                // Slot, slotHeld, slotSimKey (the command map)
#include "eva.h"                 // evaArmCommands (the kerbal's controls)
#include "orbit.h"               // OrbitElements, computeOrbitElements
#include "physics.h"             // physics_tick()
#include "timestep.h"            // substepCount()

void tick(Game &g) {
    double newTime = (double)(SDL_GetTicks()) * 0.001;
    double frameTime = newTime - g.currentTime;
    g.currentTime = newTime;
    g.accumulator += frameTime;

    /* Spiral-of-death clamp: at most 10 ticks of catch-up, i.e. 10/physics_hz
       seconds of wall-clock hitch tolerance -- 167 ms at the 60 Hz default,
       but only 10 ms at --physics-hz 1000. A frame slower than that silently
       drops sim time. */
    if(g.accumulator > 10 * g.dt) {
        g.accumulator = 10 * g.dt;
    }

    // Snapshot the ship list once so a mid-tick SoI move is walked once.
    static thread_local std::vector<Vehicle *> all;
    collectVehiclesInto(g.sys, all);

    // clear stats; sync the difficulty scale onto every ship.
    for(auto *s : all) {
        s->m_thrust = 0.0;
        s->exhaust_scale = g.args.exhaust_scale;
        s->drag_cd = g.args.drag_cd;
    }

    while (g.accumulator >= g.dt) {
        // Clear per-tick commands so a tick without the keys doesn't keep
        // pushing from the last one. No ship -> nothing to clear.
        if(g.ship) {
            g.ship->clearThrust();
            g.ship->clearRotCmd();
            g.ship->clearRcs();
        }

        const bool *key = SDL_GetKeyboardState(nullptr);
        const Uint16 modState = SDL_GetModState();
        /* --sim-press: a synthetic key is "down" from its down time to
           its up time. SDL_PushEvent does not update the key array, so a
           held command ORs in each entry's window. */
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
            /* cam_speed (the [/] ladder, shown as "Cam speed" in the HUD) and
               the roll step are per-TICK amounts. They were tuned at 50 Hz, so
               scaling by dt*60 re-bases them onto the 60 Hz default -- 20%
               faster than the old per-tick constants -- and keeps the free
               cam's felt speed independent of --physics-hz. */
            const double cam_rate = g.dt * 60.0;
            if (slotActive(Slot::CamForward)) { g.camera->MoveForward(g.cam_speed * cam_rate); }
            else if (slotActive(Slot::CamBack)) { g.camera->MoveForward(-g.cam_speed * cam_rate); }

            if (slotActive(Slot::CamStrafeLeft)) { g.camera->MoveRight(-g.cam_speed * cam_rate); }
            else if (slotActive(Slot::CamStrafeRight)) { g.camera->MoveRight(g.cam_speed * cam_rate); }

            if (slotActive(Slot::CamRollLeft)) { g.camera->Roll(-0.05 * cam_rate); }
            else if (slotActive(Slot::CamRollRight)) { g.camera->Roll(0.05 * cam_rate); }

            if (slotActive(Slot::CamUp)) { g.camera->MoveUp(g.cam_speed * cam_rate); }
            else if (slotActive(Slot::CamDown)) { g.camera->MoveUp(-g.cam_speed * cam_rate); }
        }

        // Piloting (orbit / first person). Free cam is a flier: no sticks.
        if (g.camera->mode != CAM_FREE) {
            bool game_running = (g.time_accel > 0);
            // Active-ship controls: only with a ship and in a pilot scene.
            // A running sim in a non-pilot scene coasts.
            if(g.ship && curScene(g).pilot) {
            // Touching the controls wakes a railed active ship (you cannot
            // maneuver on rails). A rails warp drops to 1x on the way out.
            if(g.ship->onRails && g.time_accel > 0) {
                // any flight control wakes; in EVA Space is the extra
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
                /* EVA: arm the kerbal's controls for this tick (eva.cpp).
                   Nothing is armed while paused. */
                if(game_running) { evaArmCommands(g, slotActive); }
            } else {
            // Autopilot slew: apply after clearRotCmd so the ship keeps
            // holding. X kill-rot below overrides while held.
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
            // Baseline sticks are pre-flipped for screen-direction yaw/roll;
            // flip_* settings invert away from that default.
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

            // Thrust latch keeps engines lit with the key released.
            if (slotActive(Slot::Thrust) || g.thrust_latched) { g.ship->Command(ShipCmd(Thrust), game_running, g.dt * g.time_accel); }
            if (slotActive(Slot::KillRot)) { g.ship->Command(ShipCmd(KillRot), game_running); }

            // Throttle is a control input, so it ramps on WALL time (g.dt),
            // not on the warped sim interval Thrust takes.
            if (slotActive(Slot::ThrottleUp)) { g.ship->Command(ShipCmd(ThrottleUp), game_running, g.dt); }
            if (slotActive(Slot::ThrottleDown)) { g.ship->Command(ShipCmd(ThrottleDown), game_running, g.dt); }

            // RCS translation (ship-relative): each held slot arms one of
            // the ship's own axes; applyRcsForce resolves the direction.
            if (slotActive(Slot::RcsForward)) { g.ship->Command(ShipCmd(RcsNose,  +1.0f), game_running); }
            if (slotActive(Slot::RcsBack))    { g.ship->Command(ShipCmd(RcsNose,  -1.0f), game_running); }
            if (slotActive(Slot::RcsUp))      { g.ship->Command(ShipCmd(RcsUp,    +1.0f), game_running); }
            if (slotActive(Slot::RcsDown))    { g.ship->Command(ShipCmd(RcsUp,    -1.0f), game_running); }
            if (slotActive(Slot::RcsLeft))    { g.ship->Command(ShipCmd(RcsRight, -1.0f), game_running); }
            if (slotActive(Slot::RcsRight))   { g.ship->Command(ShipCmd(RcsRight, +1.0f), game_running); }
            }
            }
        }

        // Advance the analytic clock by exactly the physics timestep --
        // the frame tree and the ship integration must share the clock.
        g.time += g.dt * g.time_accel;
        g.logic_ticks++;   // one logic tick ran (its substeps are not counted)

        if(g.time_accel != 0) {
            /* Re-snapshot the ship list for THIS step: updateDocking() can
               delete the absorbed ship at the end of a step, and a stale
               snapshot would dangle on the next one. */
            collectVehiclesInto(g.sys, all);

            // SOI owner before this tick's frame bookkeeping (handoff drops
            // warp to 1x).
            TerrainBody *soiOwner = g.ship ? g.ship->m_parent : nullptr;

            // Proximity wakes ships near the active ship. Runs BEFORE
            // UpdateOrbitRails so a woken ship is re-expressed against last
            // tick's frame transforms (the same every live Bullet pose used).
            g.updateProximity();

            g.sun->frame->UpdateOrbitRails(g.time);

            if(g.time_accel >= kRailsWarp) {
                /* Rails warp: analytic coast (or frozen on the ground);
                   Bullet is not stepped. */
                for(auto *s : all) {
                    s->railsTick(g.time, g.dt * g.time_accel);
                }
            } else {

            // Railed ships advance their analytic conic; live ones track
            // their SOI in the shared frame tree.
            for(auto *s : all) {
                if(s->onRails) { s->railsTick(g.time, g.dt * g.time_accel); }
                else { s->switchFrames(g.time); }
                // Keep the compound COM tracking the mass distribution.
                // Not on the rails-warp path: nothing burns there.
                s->refreshCompound();
            }

            // Integrate the step in substeps, re-applying gravity + thrust +
            // rotation before EACH: Bullet clears forces at each step, and
            // velocity-dependent terms must not freeze at the step start.
            // >=3 substeps matches the old low-accel behavior; grow so the
            // substep stays <= kMaxSubStep at high time-accel.
            const double step = g.dt * g.time_accel;
            const int n = substepCount(step);
            const double h = step / n;
            for (int i = 0; i < n; i++) {
                // every NON-RAILED ship feels gravity + drag + control each
                // substep; railed ships' conics already advanced in railsTick
                // (and so feel no drag -- the known rails gap).
                for(auto *s : all) {
                    if(s->onRails) { continue; }
                    s->processGravity();
                    s->applyAeroForce(h);
                    // Power before control forces so the gate is current.
                    s->powerTick(h);
                    s->applyControlForces(h);
                }
                physics_tick(h);
                if(g.args.debug_accel) {
                    for(auto *s : all) {
                        if(s->onRails) { continue; }
                        const glm::dvec3 v = GetVelocity(s->hull);
                        printf("[accel] %-28s |F|=%9.0fN  v=(%8.2f,%8.2f,%8.2f) |v|=%7.2f\n",
                               s->name.c_str(), glm::length(s->lastThrustForce),
                               v.x, v.y, v.z, glm::length(v));
                    }
                }
            }

            // Docking: once per tick, after the substeps.
            g.updateDocking();

            } // end physics-warp branch (g.time_accel < kRailsWarp)

            // SOI handoff: drop warp to 1x so the encounter is playable.
            // A pause (0) is the player's call and stays put.
            if(g.ship && soiOwner != g.ship->m_parent && g.time_accel > 1) {
                printf("SOI switch: '%s' now around %s (was %s), warp -> 1\n",
                       g.ship->name.c_str(), g.ship->m_parent->name.c_str(),
                       soiOwner->name.c_str());
                g.time_accel = 1;
                g.toast("SOI: now around %s, warp 1x",
                        g.ship->m_parent->name.c_str());
            }

            /* --spin-log (or --radial-test): spin diagnostics, once per
               0.5 s of sim time. */
            if(g.ship && (g.args.spin_log_enabled || !g.args.radial_test.empty())) {
                static double last_spin_log = -1e30;
                if(g.time - last_spin_log >= 0.5) {
                    last_spin_log = g.time;
                    spin_log(g.ship, g.time);
                }
            }

            /* --fuel-log: fuel mass + links, once per 0.5 s of sim time. */
            if(g.ship && g.args.fuel_log) {
                static double last_fuel_log = -1e30;
                if(g.time - last_fuel_log >= 0.5) {
                    last_fuel_log = g.time;
                    g.ship->fuel_log(g.time);
                }
            }

            /* --drain-log: fuel drain rates, once per 0.5 s of sim time. */
            if(g.ship && g.args.drain_log) {
                static double last_drain_log = -1e30;
                if(g.time - last_drain_log >= 0.5) {
                    last_drain_log = g.time;
                    g.ship->drain_log(g.time);
                }
            }

            /* --power-log: power balance, once per 0.5 s of sim time. */
            if(g.ship && g.args.power_log) {
                static double last_power_log = -1e30;
                if(g.time - last_power_log >= 0.5) {
                    last_power_log = g.time;
                    g.ship->power_log(g.time);
                }
            }

            /* --slew-log: autopilot state, once per 0.1 s of sim time. */
            if(g.ship && g.args.slew_log_enabled) {
                static double last_slew_log = -1e30;
                if(g.time - last_slew_log >= 0.1) {
                    last_slew_log = g.time;
                    g.ship->slew_log(g.time);
                }
            }
        }

        // --orbit-log: orbital elements in the body's inertial frame.
        if(g.ship && g.args.orbit_log) {
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
                // Measured in the plane the orbit map shows, like the flight
                // readout (#171). lan/lpe trail the line so ORBIT_RE (e2e)
                // keeps matching.
                const PlaneAngles pa = orbitPlaneAngles(
                    o, uiRefPlane(g.ship->m_parent->frame, o.h_hat, g.map_plane));
                char lan_s[32], lpe_s[32];
                if(pa.node_ok) { snprintf(lan_s, sizeof lan_s, "%.4f deg", glm::degrees(pa.lan)); }
                else { snprintf(lan_s, sizeof lan_s, "-"); }
                if(pa.lpe_ok) { snprintf(lpe_s, sizeof lpe_s, "%.4f deg", glm::degrees(pa.lpe)); }
                else { snprintf(lpe_s, sizeof lpe_s, "-"); }
                printf("[orbitlog] t=%.1fs frame=\"%s\" r=%.6g m v=%.6g m/s "
                       "sma=%.6g m ecc=%.6g peri=%.6g m apo=%.6g m "
                       "inc=%.4f deg T=%.6g s ttAp=%.6g s ttPe=%.6g s "
                       "|h|=%.6f m2/s E=%.6f J/kg plane=%s lan=%s lpe=%s\n",
                       g.time, g.ship->frame->name.c_str(), o.distance, o.speed,
                       o.semi_major, o.ecc, o.periapsis, o.apoapsis,
                       glm::degrees(pa.inc), o.period,
                       o.time_to_apo, o.time_to_peri,
                       o.ang_momentum, o.energy,
                       refPlaneName(g.map_plane), lan_s, lpe_s);
                fflush(stdout);
            }
        }

        // --dbg-log: ship pos/alt/vel in its own frame
        if(g.ship && g.args.dbg_log) {
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

        /* --drag-log: the active ship's aero (last substep). rho>0 with
           Cd/A/|F| all 0 is the density floor reporting vacuum, not broken
           geometry. */
        if(g.ship && g.args.drag_log) {
            const Uint32 now_ms = SDL_GetTicks();
            if(now_ms - g.drag_log_last_ms >= g.orbit_log_interval_ms) {
                g.drag_log_last_ms = now_ms;
                const glm::dvec3 v = g.ship->GetVel();
                printf("[drag] t=%.1fs alt=%.1f m rho=%.5g kg/m3 "
                       "|v|=%.2f m/s |F|=%.2f N |L|=%.2f N |tau|=%.2f Nm "
                       "Cd=%.3f A=%.2f m2 AoA=%.1f deg Chute=%.2f m2\n",
                       g.time, g.ship->lastDragAlt, g.ship->lastDragRho,
                       glm::length(v),
                       glm::length(g.ship->lastAeroForce),
                       glm::length(g.ship->lastLiftForce),
                       glm::length(g.ship->lastAeroTorque),
                       g.ship->lastDragCd, g.ship->lastDragArea,
                       glm::degrees(g.ship->lastDragAlpha),
                       g.ship->lastChuteCdA);
                // Control surfaces' applied deflections.
                if(!g.ship->lastControlDeflections.empty()) {
                    printf("[ctrl]");
                    for(const Vehicle::ControlDeflection &cd :
                        g.ship->lastControlDeflections) {
                        printf(" %s[%d](%s)=%+.1fdeg", cd.def->name.c_str(),
                               cd.index, controlAxisName(cd.def->control_axis),
                               glm::degrees(cd.deflection));
                    }
                    printf("\n");
                }
                fflush(stdout);
            }
        }

        // --att-log: the ship's nose + angular velocity (attitude-physics e2e)
        if(g.ship && g.args.att_log) {
            const Uint32 now_ms = SDL_GetTicks();
            if(now_ms - g.att_log_last_ms >= g.orbit_log_interval_ms) {
                g.att_log_last_ms = now_ms;
                g.ship->att_log(g.time);
            }
        }

        // --tq-log: the spurious-torque probe, once per TICK (the per-tick
        // quantity is the point; a per-second log would average it out).
        if(g.ship && g.args.tq_log && !g.ship->onRails) {
            g.ship->tq_log(g.time);
        }

        /* --compound-check: compound-vs-parts agreement. Not gated on
           time_accel; runs over all ships (frozen-in deformation shows on
           idle/railed ones). */
        if(g.args.compound_check) {
            static Uint32 last_compound_ms = 0;
            const Uint32 now_ms = SDL_GetTicks();
            if(now_ms - last_compound_ms >= g.orbit_log_interval_ms) {
                last_compound_ms = now_ms;
                for(auto *s : all) { s->compoundCheck(g.time); }
            }
        }

        // --eva-log: the kerbal's mode + state (the EVA e2e assertions)
        if(g.ship && g.args.eva_log && g.ship->isEva()) {
            const Uint32 now_ms = SDL_GetTicks();
            if(now_ms - g.eva_log_last_ms >= g.orbit_log_interval_ms) {
                g.eva_log_last_ms = now_ms;
                Kerbal *k = static_cast<Kerbal *>(g.ship);
                const glm::dvec3 p = k->get_center_of_mass();
                const glm::dvec3 v = k->GetVel();
                const glm::dvec3 face = k->partAxis(k->controller, 1);
                printf("[evalog] t=%.1fs mode=%s grounded=%d "
                       "pos=[%.1f %.1f %.1f] vel=[%.2f %.2f %.2f] "
                       "face=[%.3f %.3f %.3f] alt=%.2f m "
                       "mass=%.3fkg\n",
                       g.time, (k->mode == EVA_GROUND) ? "ground" : "space",
                       (int)k->grounded, p.x, p.y, p.z, v.x, v.y, v.z,
                       face.x, face.y, face.z,
                       glm::length(p) - (double)k->m_parent->GetTerrainHeight(
                           glm::vec3(glm::normalize(p))),
                       k->getMass());
                fflush(stdout);
            }
        }

        g.accumulator -= g.dt;
    }
}
