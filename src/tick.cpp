// tick.cpp -- the fixed-timestep logic tick (see tick.h).
#include "tick.h"

#include <cassert>
#include <cstdio>

#include "keys.h"                // Slot, SlotProbe (the command map)
#include "angmath.h"             // acosUnit: the one acos(clamp) spelling (#622)
#include "eva.h"                 // Kerbal::armInput (the kerbal's controls)
#include "orbit.h"               // OrbitElements, computeOrbitElements
#include "physics.h"             // physics_tick()
#include "timestep.h"            // substepCount()

void tick(Game &g) {
    /* --sim-steps (#472): exactly one tick per frame, decided by the sim rather
       than by how long this frame happened to take. Wall-clock stepping is what
       makes a long flight irreproducible -- a slow frame drops the leftover time
       below, so two runs of one case step a different number of times. */
    const bool scripted = (g.sim_steps_left >= 0);
    if(scripted) {
        g.accumulator = g.dt;
    } else {
        double newTime = (double)(SDL_GetTicks()) * 0.001;
        double frameTime = newTime - g.currentTime;
        g.currentTime = newTime;
        g.accumulator += frameTime;

        /* Spiral-of-death clamp: at most 10 ticks of catch-up, i.e. 10/physics_hz
           seconds of wall-clock hitch tolerance. A frame slower than that
           silently drops sim time -- so price what it drops (#402): at the warp this
           frame ran at, the leftover was worth this much simulated time. Nothing
           warns the PLAYER here; a game hitched in a menu losing catch-up time is
           exactly what the cap is for. main.cpp reports the total when a run was
           driven by --timeout, where the loss quietly shortens whatever the run
           measured. */
        if(g.accumulator > kMaxCatchUpTicks * g.dt) {
            g.sim_seconds_dropped += (g.accumulator - kMaxCatchUpTicks * g.dt)
                                   * g.time_accel;
            g.accumulator = kMaxCatchUpTicks * g.dt;
        }
    }
    const long long ticks_before = g.logic_ticks;

    /* The live list, not a snapshot: an SoI crossing re-homes a Vehicle without
       moving it between containers, so a mid-tick crossing cannot reshuffle or
       dangle the walk. Only a death changes it, and Game::retire erases the
       vehicle before anything can see the frame again. */
    std::vector<Vehicle *> &vehicles = g.vehicles;

    // Sync the difficulty scale onto every ship.
    for(auto *s : vehicles) {
        s->m_thrust = 0.0;
        s->exhaust_scale = g.args.exhaust_scale;
        s->drag_cd = g.args.drag_cd;
        s->dv_log_enabled = g.args.dv_log;
        s->sas_policy = g.sas_policy;   // the pilot-vs-SAS rule, per tick (#421)
        // The F4 force overlay records on the active ship only (the only
        // one drawn), and only in a piloting scene (a VAB/studio ship's
        // world coords through that scene's camera are meaningless). Off
        // edge: clear, so a ship that stops being recorded cannot paint
        // its weeks-old arrows when recording reaches it again (while
        // paused, where the per-substep clear never runs).
        const bool wantRecord = g.show_force_overlay && s == g.ship
                             && curScene(g).pilot;
        if(!wantRecord && s->recordForceVecs) { s->lastForceVecs.clear(); }
        s->recordForceVecs = wantRecord;
        /* #485: the per-surface alpha/q readout is asked for by the overlay (it
           labels each surface's own arrow), by --drag-log (its [surface] row) and by
           an open part window on a STEERING part (#492: the trim slider needs the
           angle its own surface is reading, or setting trim is blind). Steering and
           only steering: every lifting part is already measured whether or not
           anything is watching (vehicle.cpp's lift loop), so this flag only ever
           decides whether a NEUTRAL control surface pays for the flow and mount frame
           its early-out exists to skip -- asking for a wing's window would buy a wing
           row that is already free, and would quietly change what the SURFACE window
           counts. part_sels is a handful of entries, so the scan is nothing next to
           the aero pass. */
        bool partWindowWants = false;
        for(const PartSel &ps : g.part_sels) {
            if(ps.ship != s || ps.part >= s->parts.size()) { continue; }
            const PartDef *pd = s->parts[ps.part]->def;
            if(pd != nullptr && pd->isControlSurface()) {
                partWindowWants = true;
                break;
            }
        }
        s->recordSurfaceAero = wantRecord
            || (g.args.drag_log && s == g.ship && curScene(g).pilot)
            || (partWindowWants && curScene(g).pilot);
    }

    while (g.accumulator >= g.dt) {
        // Clear per-tick commands so a tick without the keys does not keep
        // pushing from the last one. No ship -> nothing to clear.
        if(g.ship) {
            g.ship->clearThrust();
            g.ship->clearRotCmd();
            g.ship->clearRcs();
        }

        const bool *key = SDL_GetKeyboardState(nullptr);
        const Uint16 modState = SDL_GetModState();
        /* The tick's held-path probe (keys.h): SDL's key array ORed with each
           --sim-press entry's held window, because SDL_PushEvent does not update the
           array. A named type rather than a closure because it is handed to
           Kerbal::armInput, where a std::function heap-allocated it once a tick (#581). */
        const SlotProbe slotActive{key, modState, &g.binds, &g.args.sim_presses};

        if (g.camera->mode == CAM_FREE) {
            /* cam_speed (the [/] ladder) and the roll step are per-TICK
               amounts: scaling by dt*60 keeps the free cam's felt speed
               independent of --physics-hz. */
            const double cam_rate = g.dt * 60.0;
            if (slotActive.held(Slot::CamForward)) { g.camera->MoveForward(g.cam_speed * cam_rate); }
            else if (slotActive.held(Slot::CamBack)) { g.camera->MoveForward(-g.cam_speed * cam_rate); }

            if (slotActive.held(Slot::CamStrafeLeft)) { g.camera->MoveRight(-g.cam_speed * cam_rate); }
            else if (slotActive.held(Slot::CamStrafeRight)) { g.camera->MoveRight(g.cam_speed * cam_rate); }

            if (slotActive.held(Slot::CamRollLeft)) { g.camera->Roll(-0.05 * cam_rate); }
            else if (slotActive.held(Slot::CamRollRight)) { g.camera->Roll(0.05 * cam_rate); }

            if (slotActive.held(Slot::CamUp)) { g.camera->MoveUp(g.cam_speed * cam_rate); }
            else if (slotActive.held(Slot::CamDown)) { g.camera->MoveUp(-g.cam_speed * cam_rate); }
        }

        // Piloting (orbit / first person). Free cam is a flier: no sticks.
        if (g.camera->mode != CAM_FREE) {
            bool game_running = (g.time_accel > 0);
            if(g.ship && curScene(g).pilot) {
            // Touching the controls wakes a railed active ship (you cannot
            // maneuver on rails). A rails warp drops to 1x on the way out.
            if(g.ship->onRails && g.time_accel > 0) {
                bool wake = slotActive.held(Slot::PitchUp) || slotActive.held(Slot::PitchDown) ||
                            slotActive.held(Slot::YawLeft) || slotActive.held(Slot::YawRight) ||
                            slotActive.held(Slot::RollLeft) || slotActive.held(Slot::RollRight) ||
                            slotActive.held(Slot::Thrust) || slotActive.held(Slot::KillRot) ||
                            slotActive.held(Slot::ThrottleUp) || slotActive.held(Slot::ThrottleDown) ||
                            slotActive.held(Slot::RcsForward) || slotActive.held(Slot::RcsBack) ||
                            slotActive.held(Slot::RcsUp) || slotActive.held(Slot::RcsDown) ||
                            slotActive.held(Slot::RcsLeft) || slotActive.held(Slot::RcsRight);
                if(g.ship->isEva()) { wake = wake || slotActive.held(Slot::Space); }
                if(wake) {
                    g.ship->leaveRails();
                    if(g.time_accel >= kRailsWarp) {
                        g.time_accel = 1;
                        g.toast("Control input: left the rails, warp 1x");
                    }
                    printf("Control input: '%s' left the rails, warp -> %d\n",
                           g.ship->displayName().c_str(), g.time_accel);
                }
            }
            if(g.ship->isEva()) {
                /* EVA: arm the kerbal's controls for this tick (eva.cpp).
                   Nothing is armed while paused. */
                if(game_running) {
                    static_cast<Kerbal *>(g.ship)->armInput(g, slotActive);
                }
            } else {
            // Autopilot slew: apply after clearRotCmd so the ship keeps
            // holding. X kill-rot below overrides while held.
            if(g.ship->applySlewRequest(game_running)) {
                if(g.ship->onRails) {
                    g.ship->leaveRails();
                    if(g.time_accel >= kRailsWarp) {
                        g.time_accel = 1;
                        g.toast("Control input: left the rails, warp 1x");
                    }
                }
            }
            // Baseline sticks are pre-flipped for screen-direction yaw/roll;
            // flip_* settings invert away from that default.
            const float f_pitch = g.flip_pitch ? -1.0f : 1.0f;
            const float f_yaw   = g.flip_yaw   ? -1.0f : 1.0f;
            const float f_roll  = g.flip_roll  ? -1.0f : 1.0f;
            // pitch: about the ship's right axis (PitchUp/Down, default W/S)
            if (slotActive.held(Slot::PitchUp)) { g.ship->Command(ShipCmd(Pitch,  f_pitch * +1.0f), game_running); }
            if (slotActive.held(Slot::PitchDown)) { g.ship->Command(ShipCmd(Pitch,  f_pitch * -1.0f), game_running); }
            // yaw: about the ship's up axis (baseline pre-flipped)
            if (slotActive.held(Slot::YawLeft)) { g.ship->Command(ShipCmd(Yaw,    f_yaw   * -1.0f), game_running); }
            if (slotActive.held(Slot::YawRight)) { g.ship->Command(ShipCmd(Yaw,    f_yaw   * +1.0f), game_running); }
            // roll: about the ship's nose (baseline pre-flipped)
            if (slotActive.held(Slot::RollLeft)) { g.ship->Command(ShipCmd(Roll,   f_roll  * -1.0f), game_running); }
            if (slotActive.held(Slot::RollRight)) { g.ship->Command(ShipCmd(Roll,   f_roll  * +1.0f), game_running); }

            // Thrust latch keeps engines lit with the key released.
            if (slotActive.held(Slot::Thrust) || g.thrust_latched) { g.ship->Command(ShipCmd(Thrust), game_running, g.dt * g.time_accel); }
            if (slotActive.held(Slot::KillRot)) { g.ship->Command(ShipCmd(KillRot), game_running); }

            // Throttle is a control input, so it ramps on WALL time (g.dt),
            // not on the warped sim interval Thrust takes.
            if (slotActive.held(Slot::ThrottleUp)) { g.ship->Command(ShipCmd(ThrottleUp), game_running, g.dt); }
            if (slotActive.held(Slot::ThrottleDown)) { g.ship->Command(ShipCmd(ThrottleDown), game_running, g.dt); }

            /* The ascent autopilot, after the keys and before the substeps: after,
               so the law sees this tick's throttle claim and yields to it (#400 F3);
               before, so its Aim/Thrust/ThrottleSet are armed for the physics that
               follows. Gated on game_running because a paused sim runs no laws --
               the phase machine would otherwise advance on wall ticks while nothing
               moves. dt here is the WARPED sim interval, the same number the Thrust
               command is given, because that is how far the burn actually reaches.
               And gated on the hull it was armed on: the law has no idea which ship
               it is talking to, so switching to another one used to hand the autopilot
               to that ship, which then launched itself out from under the player
               (#610 review). */
            if(g.ascent && game_running && g.ship == g.ascent_ship) {
                const bool was_flying = g.ascent->active();
                flyAscent(*g.ship, *g.ascent, g.dt * g.time_accel);
                /* The verdict hands the clock back with the hull. A mission only
                   exists to fly the burn; once it has answered, leaving warp
                   wherever the burn left it leaves the player coasting at 10x
                   through an orbit they are no longer being flown. Always 1x, not
                   just out of rails warp as the other hand-backs do (#612). */
                if(was_flying && !g.ascent->active() && g.time_accel > 1) {
                    g.time_accel = 1;
                    g.toast("Ascent %s: warp 1x", g.ascent->phaseName());
                }
            }

            // RCS translation (ship-relative); applyRcsForce resolves the
            // direction.
            if (slotActive.held(Slot::RcsForward)) { g.ship->Command(ShipCmd(RcsNose,  +1.0f), game_running); }
            if (slotActive.held(Slot::RcsBack))    { g.ship->Command(ShipCmd(RcsNose,  -1.0f), game_running); }
            if (slotActive.held(Slot::RcsUp))      { g.ship->Command(ShipCmd(RcsUp,    +1.0f), game_running); }
            if (slotActive.held(Slot::RcsDown))    { g.ship->Command(ShipCmd(RcsUp,    -1.0f), game_running); }
            if (slotActive.held(Slot::RcsLeft))    { g.ship->Command(ShipCmd(RcsRight, -1.0f), game_running); }
            if (slotActive.held(Slot::RcsRight))   { g.ship->Command(ShipCmd(RcsRight, +1.0f), game_running); }
            }
            }
        }

        // Advance the analytic clock by exactly the physics timestep --
        // the frame tree and the ship integration must share the clock.
        g.time += g.dt * g.time_accel;
        g.logic_ticks++;   // one logic tick ran (its substeps are not counted)
        g.sim_ticks_run++; // the --sim-steps clock's own count: never rewound
        /* How much sim this run actually simulated, summed per tick so a load
           that rewinds g.time cannot corrupt it (--sim-steps exit report). */
        g.sim_seconds_ran += g.dt * g.time_accel;

        if(g.time_accel != 0) {
            /* No re-snapshot here. updateDocking() retires the absorbed ship at
               the end of a step, but retiring takes it out of the live list
               first, so the next step's walk simply stops seeing it -- which is
               exactly what the old per-step collectVehiclesInto was
               reconstructing. The free itself waits for the frame's sweep. */

            // SOI owner before this tick's frame bookkeeping (handoff drops
            // warp to 1x).
            TerrainBody *soiOwner = g.ship ? g.ship->m_parent : nullptr;

            // Proximity wakes ships near the active ship. Runs BEFORE
            // UpdateOrbitRails so a woken ship is re-expressed against last
            // tick's frame transforms.
            g.updateProximity();

            g.sun->frame->UpdateOrbitRails(g.time);

            if(g.time_accel >= kRailsWarp) {
                /* Rails warp: analytic coast (or frozen on the ground);
                   Bullet is not stepped. */
                const double step = g.dt * g.time_accel;
                RailEnd end = RailEndNone;
                Vehicle *ended = nullptr;
                for(auto *s : vehicles) {
                    const RailEnd e = s->railsTick(g.time, step);
                    if(e != RailEndNone && ended == nullptr) { end = e; ended = s; }
                }
                if(ended != nullptr) {
                    /* Air and ground are not a conic: hand the ship back to
                       physics and stop the clock so the player flies the rest. */
                    g.time_accel = 1;
                    const char *why = (end == RailEndGround) ? "hit the ground"
                                                             : "entered the atmosphere";
                    printf("Rails warp: '%s' %s, warp -> 1\n",
                           ended->displayName().c_str(), why);
                    g.toast("Rails: '%s' %s, warp 1x", ended->displayName().c_str(), why);
                }
            } else {

            for(auto *s : vehicles) {
                if(s->onRails) { s->railsTick(g.time, g.dt * g.time_accel); }
                else { s->switchFrames(g.time); }
                // Keep the compound COM tracking the mass distribution.
                // Not on the rails-warp path: nothing burns there.
                s->refreshCompound();
            }

            // Integrate the step in substeps, re-applying gravity + thrust +
            // rotation before EACH: Bullet clears forces at each step, and
            // velocity-dependent terms must not freeze at the step start.
            // Grow the count so the substep stays <= kMaxSubStep at high
            // time-accel.
            const double step = g.dt * g.time_accel;
            const int n = substepCount(step);
            const double h = step / n;
            for (int i = 0; i < n; i++) {
                // every NON-RAILED ship feels gravity + drag + control each
                // substep; railed ships' conics already advanced in railsTick
                // (and so feel no drag -- the known rails gap).
                for(auto *s : vehicles) {
                    if(s->onRails) { continue; }
                    // The F4 force overlay: one substep's worth, opened
                    // here -- the single boundary owner for every applier
                    // below, whichever order they run in.
                    s->beginForceRecordSubstep();
                    s->processGravity();
                    /* Power before control forces so the gate is current. This is not
                       the only door: railed hulls bill their EC per warp hop inside
                       railsTick (#647). That bill is skipped for a hop that reaches the
                       air, so the hull arriving here is billed exactly once -- here. */
                    s->powerTick(h);
                    /* #426: control BEFORE aero, so the deflection the one actuator
                       solve asked each control surface for is the deflection the air
                       feels in the SAME substep. Nothing above moves pos/vel/w --
                       physics_tick below is what integrates -- so aero loses no
                       freshness of q or of the flow at the surface by going second, and
                       applyControlForces already orders rotation before thrust for the
                       same "same substep, no lag" reason. The other order delays every
                       plate by a whole substep, i.e. puts an actuator lag inside a
                       deadbeat attitude law, where it rings. */
                    s->applyControlForces(h);
                    s->applyAeroForce(h);
                }
                physics_tick(h);
                if(g.args.debug_accel) {
                    for(auto *s : vehicles) {
                        if(s->onRails) { continue; }
                        const glm::dvec3 v = GetVelocity(s->hull);
                        printf("[accel] %-28s |F|=%9.0fN  v=(%8.2f,%8.2f,%8.2f) |v|=%7.2f\n",
                               s->displayName().c_str(), glm::length(s->lastThrustForce),
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
                       g.ship->displayName().c_str(), g.ship->m_parent->name.c_str(),
                       soiOwner->name.c_str());
                g.time_accel = 1;
                g.toast("SOI: now around %s, warp 1x",
                        g.ship->m_parent->name.c_str());
            }
        }

        /* The periodic state logs. Wall-clock gated like --orbit-log below, and
           deliberately OUTSIDE the time_accel gate: a SIM-clock gate prints
           nothing while the player is paused (a load sets time_accel = 0), goes
           quiet for the rest of a run once New Game rewinds g.time to 0 under a
           process-wide stamp, and fires on EVERY tick once one tick advances
           more than the interval at warp (#374). --sim-steps flips that choice
           for a scripted run, because there the wall clock is the thing being
           removed: a wall-gated row samples a different substep on a faster
           machine, so a case's max/min metrics move between runs (#472). */

        /* --spin-log (or --radial-test): spin diagnostics. */
        if(g.ship && (g.args.spin_log_enabled || !g.args.radial_test.empty())) {
            if(g.logDue(kGateSpin, g.logClockMs(), kStateLogMs)) {
                spin_log(g.ship, g.time);
            }
        }

        /* --mass-log: every vehicle on the SAME tick, so a split's mass can
           be added up -- the survivor of a decoupler is a different Vehicle
           from g.ship, and a per-ship sample taken on different ticks is not
           one snapshot of the fleet (#367). */
        if(g.args.mass_log) {
            if(g.logDue(kGateMass, g.logClockMs(), kStateLogMs)) {
                for(auto *s : vehicles) { mass_log(s, g.time); }
            }
        }

        /* --fuel-log: fuel mass + links. */
        if(g.ship && g.args.fuel_log) {
            if(g.logDue(kGateFuel, g.logClockMs(), kStateLogMs)) {
                g.ship->fuel_log(g.time);
            }
        }

        /* --drain-log: fuel drain rates. The sampler runs on every stepping
           tick and accumulates, so the interval a rate describes is SIM time
           and every sim second between two prints is counted once (#376); the
           print stays on the wall-clock gate like the rest of the family, and
           dt= in the row says how much sim that row covers. A paused ship
           never steps, so it prints nothing rather than repeating a row. */
        if(g.ship && g.args.drain_log) {
            if(g.time_accel != 0) { g.ship->drainSample(g.dt * g.time_accel); }
            if(g.logDue(kGateDrain, g.logClockMs(), kStateLogMs)) {
                g.ship->drain_log(g.time);
            }
        }

        /* --power-log: power balance. */
        if(g.ship && g.args.power_log) {
            if(g.logDue(kGatePower, g.logClockMs(), kStateLogMs)) {
                g.ship->power_log(g.time);
            }
        }

        /* --steering-log: the attitude bill. Same 0.5s state gate as --power-log
           because it answers the same kind of question ("can it, and what is it
           paying?") about the same wall-clock cadence, and the row re-derives the
           row set rather than riding on the sim's -- so it must not run per substep. */
        if(g.ship && g.args.steer_log) {
            if(g.logDue(kGateSteer, g.logClockMs(), kStateLogMs)) {
                g.ship->steering_log(g.time);
            }
        }

        /* --slew-log: autopilot state. 10Hz, not 2Hz: a slew is a turn being
           steered, and at 2Hz the error curve is undersampled. */
        if(g.ship && g.args.slew_log_enabled) {
            if(g.logDue(kGateSlew, g.logClockMs(), kSlewLogMs)) {
                g.ship->slew_log(g.time);
            }
        }

        /* --ascent-log: the law's own state, so a case can CHECK the phase and the
           cutoff against the same rows that drove the flight, and a failed run says
           why in one line instead of being inferred from the orbit rows. */
        if(g.ship && g.ascent && g.ship == g.ascent_ship && g.ship->m_parent && g.args.ascent_log) {
            const Uint32 now_ms = g.logClockMs();
            if(g.logDue(kGateAscent, now_ms, g.orbit_log_interval_ms)) {
                const AscentLaw &law = *g.ascent;
                const OrbitElements &o = law.elements();
                const bool conic = law.conicOk();
                /* nan, not a number: a railed hull or a mu-less body never got a
                   conic, and elements() still held the previous tick's -- or a
                   default-constructed one, which printed "periapsis 0 m" as though
                   it had been measured. The e2e harness reads nan as a float, so a
                   CHECK against such a row fails loudly instead of agreeing with an
                   invented number (#610 review). */
                auto optNum = [](char *buf, size_t n, double v, bool ok) {
                    std::snprintf(buf, n, ok ? "%.6g" : "nan", v);
                };
                const double sea_r = g.ship->m_parent->seaLevelRadius();
                char peri_txt[32], apo_txt[32], inc_txt[32], node_txt[32];
                optNum(peri_txt, sizeof peri_txt, periapsisAsl(o, sea_r), conic);
                if(conic && o.traj == TrajectoryClass::Elliptic) {
                    optNum(apo_txt, sizeof apo_txt, apoapsisAsl(o, sea_r), true);
                } else {
                    std::snprintf(apo_txt, sizeof apo_txt, "nan");
                }
                /* Inclination against the body's equator, from the pole (see
                   flyAscent: omega() is zero for a body that does not spin, which
                   used to make this field print -1.00 as a stand-in for "unknown"). */
                double inc_spin = 0.0;
                if(conic && o.ang_momentum > 0.0) {
                    Frame *inertial = g.ship->frame->getNonRotFrame();
                    if(inertial == nullptr) { inertial = g.ship->frame; }
                    const double c = glm::dot(o.h_hat, g.ship->frame->spinAxisRelTo(inertial));
                    inc_spin = glm::degrees(acosUnit(c));
                }
                optNum(inc_txt, sizeof inc_txt, inc_spin, conic && o.ang_momentum > 0.0);
                optNum(node_txt, sizeof node_txt, law.node_r(), law.node_r() > 0.0);
                printf("[ascent] t=%.1fs phase=%s lever=%.2f pitch=%.1f deg "
                       "peri_asl=%s m apo_asl=%s m inc_eq=%s deg node_r=%s m why=\"%s\"\n",
                       g.time, law.phaseName(), g.ship->throttle(), law.pitch_deg(),
                       peri_txt, apo_txt, inc_txt, node_txt, law.why());
            }
        }
        // --orbit-log: orbital elements in the body's inertial frame.
        if(g.ship && g.args.orbit_log) {
            const Uint32 now_ms = g.logClockMs();
            if(g.logDue(kGateOrbit, now_ms, g.orbit_log_interval_ms)) {
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
                /* Measured in the plane the orbit map shows, like the flight
                   readout (#171). lan/lpe trail the line so ORBIT_RE (e2e)
                   keeps matching. The target normal comes from ShipView, NOT
                   from the planner: this tick runs before the render pass
                   rebuilds xferTargets, so asking the planner here would pair
                   the PREVIOUS frame's target list with the focus the ship may
                   have just switched to (an SOI crossing prints a meaningless
                   inc for one tick), and would dereference vessel pointers a
                   recover or load may already have freed. */
                const RefPlane ref = uiRefPlane(g.ship->m_parent->frame,
                                                o.h_hat, g.hud.view.target_h_hat,
                                                g.map.plane);
                const PlaneAngles pa = orbitPlaneAngles(o, ref);
                char lan_s[32], lpe_s[32];
                // LAN needs a node AND a zero longitude to measure it from: the
                // Target-orbit slot has the node line but no chosen longitude.
                if(pa.node_ok && pa.lon_ok) { snprintf(lan_s, sizeof lan_s, "%.4f deg", glm::degrees(pa.lan)); }
                else { snprintf(lan_s, sizeof lan_s, "-"); }
                if(pa.lpe_ok) { snprintf(lpe_s, sizeof lpe_s, "%.4f deg", glm::degrees(pa.lpe)); }
                else { snprintf(lpe_s, sizeof lpe_s, "-"); }
                /* peri_r/apo_r are RADII from the body centre and peri_asl/apo_asl
                   are metres above its mean sea level (#404) -- printing only one
                   of the pair is what let a suborbital "periapsis 561 km
                   underground" read as "a 38 x 881 km orbit". An OPEN conic has
                   no apoapsis, so its two keys are absent rather than holding a
                   sentinel: a gate like `apo_asl < target` on an escaping ship
                   must fail loudly, not read true off a -1. */
                const double sea_r = g.ship->m_parent->seaLevelRadius();
                char apo_s[64];
                if(o.traj == TrajectoryClass::Elliptic) {
                    const int n = snprintf(apo_s, sizeof apo_s,
                                           "apo_r=%.6g m apo_asl=%.6g m",
                                           o.apoapsis_r, apoapsisAsl(o, sea_r));
                    // A truncated row parses as nothing at all, so a silently
                    // short buffer loses samples instead of failing loudly.
                    assert(n > 0 && n < (int)sizeof apo_s);
                } else {
                    apo_s[0] = '\0';
                }
                printf("[orbitlog] t=%.1fs frame=\"%s\" r=%.6g m v=%.6g m/s "
                       "sma=%.6g m ecc=%.6g peri_r=%.6g m peri_asl=%.6g m %s "
                       "inc=%.4f deg T=%.6g s ttAp=%.6g s ttPe=%.6g s "
                       "|h|=%.6f m2/s E=%.6f J/kg plane=%s lan=%s lpe=%s\n",
                       g.time, g.ship->frame->name.c_str(), o.distance, o.speed,
                       o.semi_major, o.ecc, o.periapsis_r, periapsisAsl(o, sea_r),
                       apo_s,
                       glm::degrees(pa.inc), o.period,
                       o.time_to_apo, o.time_to_peri,
                       o.ang_momentum, o.energy,
                       refPlaneName(ref.mode), lan_s, lpe_s);
                fflush(stdout);
            }
        }

        /* --dv-log: the delta-v budget. Its own gate slot, but it reuses the
           --orbit-log interval so the two logs sample at the same rate and a
           CHECK can compare their first/last rows without resampling either.
           The t= values still differ: the gates are independent, so they fire
           on different ticks. */
        if(g.ship && g.args.dv_log) {
            const Uint32 now_ms = g.logClockMs();
            if(g.logDue(kGateDv, now_ms, g.orbit_log_interval_ms)) {
                g.ship->dv_log(g.time);
            }
        }

        // --dbg-log: ship pos/alt/vel in its own frame
        if(g.ship && g.args.dbg_log) {
            const Uint32 now_ms = g.logClockMs();
            if(g.logDue(kGateDbg, now_ms, g.orbit_log_interval_ms)) {
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

        /* --drag-log: the active ship's aero (last substep). alt_asl= is above the
           sea-level DATUM, so it reads NEGATIVE exactly where rho is at its maximum
           (#245); it is named like [surfinfo]'s alt_asl because that pair -- not this
           line -- is the readout that says where the hull actually is (alt_agl beside
           it). It used to say bare `alt=`: [surfinfo] already names its datum with an
           alt_agl/alt_asl pair, while [dbg] and [evalog] print a bare alt= that means
           ABOVE TERRAIN (#626), so three logs spelled one name two ways -- and once
           this line carries the air at that height (T=, p=, a=, Mach= at the end), a
           bare altitude is exactly what a reader mixes up (#552 A3). rho>0 with |F| 0
           means the pass found nothing to bill: the density floor (kRhoFloor, i.e.
           above the atmosphere's top) or a hull sitting still in its own air -- the A=
           figure then reads 0 because no flow direction was usable, while Cd
           still prints the hull's shape. Neither is broken geometry.
           AoA prints '-' with no value when there is no usable flow
           (kAoAFlowFloor), like [flight] and [surfinfo] do: a parked ship's
           ~1e-4 m/s jitter reads as a full +/-90 deg swing, and a number in a
           log gets treated as a measurement.
           WHAT DASHES AND WHY (#560). The rule is where a number comes from, not
           whether the pass felt confident:
             * Read off a DIRECTION, so a jitter flow invents them: AoA, Beta, CpL,
               CLa, SM, Profile, FormCp, Ld (#694) -- all dashed behind lastDragAlphaValid
               (Ld also dashes on its own denominator: no along-flow drag to divide by).
             * Induced= joins them, and it earned it on measurement rather than by
               symmetry. Its CdA is q-NORMALISED (vehicle.cpp:applyInducedDrag bills
               CL_sys = sum(L_i/q_i) / sum(S_i), then inducedCd(CL_sys, ...)), so the
               tiny q of a parked hull cancels out of the coefficient and leaves a
               whole wing area charged against a force of 0.00 N. Measured on the
               Kerbin pad, skiff at rest, every row of a 25 s run dashes its AoA and
               Induced printed 1.442 m2 at t=5, 0.000 at t=6, then 0.234, 0.273 and
               0.473 m2 at t=11, 13 and 21 of the same run.
               e2e/cases/flight/560-parked-hull-bills-nothing.txt pins it.
             * Summed off FORCES or solved from the BODY, so a jitter flow cannot
               fill them: Cd=, A=, Chute= (the EFFECTIVE area of a canopy that IS
               deployed, capped by kChuteAccelLimit x mass / q -- a fact about the rig
               that dashing would hide) and the four air fields at the end of the line.
             * Mach= is a special case of the last group: it is a MAGNITUDE, not a
               direction, so the unusable flow that dashes AoA puts 0.000 there, which
               is what it is -- a hull at rest in its own air. */
        if(g.ship && g.args.drag_log) {
            const Uint32 now_ms = g.logClockMs();
            if(g.logDue(kGateDrag, now_ms, g.orbit_log_interval_ms)) {
                const glm::dvec3 v = g.ship->GetVel();
                char aoa_txt[24];
                if(g.ship->lastDragAlphaValid) {
                    snprintf(aoa_txt, sizeof aoa_txt, "AoA=%.1f deg",
                             glm::degrees(g.ship->lastDragAlpha));
                } else {
                    snprintf(aoa_txt, sizeof aoa_txt, "AoA=-");
                }
                /* #560: the induced bill dashes with the flow fields. Its coefficient
                   is q-normalised (vehicle.cpp:applyInducedDrag), so the same jitter
                   that makes AoA junk inflates this CdA while the FORCE it stands for
                   is 0.00 N -- a parked skiff charged 1.442 m2 of induced drag on a row
                   that dashes its own AoA, and 0.000 on the row after it. Dashed rather
                   than zeroed for exactly that reason: 0.000 is also what a genuinely
                   unloaded wing prints, so 0.000 cannot tell the two apart. */
                char ind_txt[48];
                if(g.ship->lastDragAlphaValid) {
                    snprintf(ind_txt, sizeof ind_txt, "Induced=%.3f m2",
                             g.ship->lastInducedCdA);
                } else {
                    snprintf(ind_txt, sizeof ind_txt, "Induced=-");
                }
                printf("[drag] t=%.1fs alt_asl=%.1f m rho=%.5g kg/m3 "
                       "|v|=%.2f m/s |F|=%.2f N |L|=%.2f N L_up=%.2f N Up=%.2f "
                       "|tau|=%.2f Nm "
                       "Cd=%.3f A=%.2f m2 %s Chute=%.2f m2 %s",
                       g.time, g.ship->lastDragAlt, g.ship->lastDragRho,
                       glm::length(v),
                       glm::length(g.ship->lastAeroForce),
                       glm::length(g.ship->lastLiftForce),
                       g.ship->lastLiftVert, g.ship->lastUpVert,
                       glm::length(g.ship->lastAeroTorque),
                       g.ship->lastDragCd, g.ship->lastDragArea,
                       aoa_txt,
                       g.ship->lastChuteCdA,
                       ind_txt);
                /* The flow's yaw-plane angle, printed last so the existing
                   parsers keep working. A silhouette is cached per DIRECTION, and
                   a direction needs both angles: a log that prints only AoA
                   cannot explain its own area (#463). The UI readout still waits
                   for a yaw law (#450) -- this is the debug line. */
                if(g.ship->lastDragAlphaValid) {
                    printf(" Beta=%.1f deg", glm::degrees(g.ship->lastDragBeta));
                } else {
                    printf(" Beta=-");
                }
                /* #486: the hull's weathervane lever, signed metres (+ = cp ahead
                   of the COM). Hull drag answers to the flow AT cp now, and the
                   sweep there opposes a rotation whatever the sign; the sign decides
                   the OTHER half -- hull drag acting at an angle of attack restores
                   the nose only with cp behind, and diverges it with cp ahead
                   (#454). Printed last, like Beta, so parsers hold. Dashed on the
                   same test as AoA/Beta: 0.00 is also what a cp exactly over the COM
                   prints, and a --drag-cd 0 pass has no lever worth quoting. */
                if(g.ship->lastDragAlphaValid) {
                    printf(" CpL=%.2f m", g.ship->lastCpLever);
                } else {
                    printf(" CpL=-");
                }
                /* #454: the LIFT half of the same judgement, in two numbers that only
                   mean anything together. CLa is how much lift an angle-of-attack
                   change buys, CL·A in m^2/rad (times q for the force); SM is where
                   that change lands, metres BEHIND the COM along the nose, so POSITIVE
                   = an overshoot makes more lift behind the COM and the airframe pushes
                   the nose back down on its own. Restoring stiffness is the product:
                   q * CLa * SM about the pitch axis. Read SM against CpL and you know
                   which half of the airframe is doing the weathervaning -- CLa says
                   whether either of them is doing anything at all, because past the
                   stall the slopes droop and CLa collapses while a symmetric ship's SM
                   stays blandly positive. SM is dashed when nothing had a slope this
                   pass (no lifting part, a mount edge-on, everything stalled flat) --
                   the ratio would be 0/0, and quoting 0.00 there would claim a margin
                   exactly at the COM, which is a different statement. SM is the only
                   SIGNED field on this line, on purpose: its sign is the payload. Both
                   dash together on the same test as AoA/Beta/CpL, so a winged ship in
                   vacuum does not print the same numbers as a ship with no wings. */
                if(g.ship->lastDragAlphaValid) {
                    printf(" CLa=%.2f m2/rad", g.ship->lastLiftSlope);
                    if(g.ship->lastStaticValid) {
                        printf(" SM=%+.2f m", g.ship->lastStaticMargin);
                    } else {
                        printf(" SM=-");
                    }
                } else {
                    printf(" CLa=- SM=-");
                }
                /* #455: what the lifting and steering parts pay for their SHAPE, in
                   the same CdA units as Chute= and Induced=. Before this they paid it
                   inside A=, mixed into the hull's union at the hull's cd mean; now
                   A=/Cd= describe the body alone and this is the rest of the bill.
                   Read the two together: the airframe's total form drag is
                   drag_cd*Cd*A + Profile -- A= and Cd= are the RAW hull product, and
                   the multiplier is applied to it in the force, exactly as it is
                   already inside Profile= -- with Induced= charged on top, once.
                   Since #543 A5b this column also moves with speed: each surface's
                   chordwise cd carries the wave bill drag.h::waveCd answers for its own
                   thickness at its own Mach, so a wing's shape bill climbs through the
                   sound bar and thins again above Mach 1.25. Printed to three decimals
                   because the bill just inside the band is smaller than two: 0.0004 of
                   cd on this airframe is 0.0005 m2, and 0.45 -> 0.45 tells a reader the
                   law is doing nothing when it is. */
                if(g.ship->lastDragAlphaValid) {
                    printf(" Profile=%.3f m2", g.ship->lastSurfCdA);
                } else {
                    /* No usable flow direction, so this would be a sum of bills
                       against a jitter direction (a parked glider read 12.16 m2 this
                       way with |F| = 0). Dash it like CpL/SM; the force is still
                       applied, because below the flow floor it is nothing. */
                    printf(" Profile=-");
                }
                /* #455: the lever of that combined form drag -- hull union plus
                   every surface's own bill, weighted by the CdA each pays. CpL above
                   is the HULL's answer and went narrower the moment the surfaces left
                   the union; this is the one that says whether a tail 3 m aft is
                   holding the airframe straight. Signed like CpL: + = the form drag
                   pushes AHEAD of the COM (an angle of attack drives the nose further
                   in), - = behind it -- which is the OPPOSITE reading to SM just
                   above, positive when the neutral point is BEHIND the CG. The
                   SURFACE window spells both out. Dashed with no usable flow, the
                   same test as CpL. */
                if(g.ship->lastDragAlphaValid) {
                    printf(" FormCp=%+.2f m", g.ship->lastFormCpLever);
                } else {
                    printf(" FormCp=-");
                }
                /* THE AIR ITSELF (#552 A2/A3): what this body's air profile says at
                   the altitude on this line -- the same seam the atlas dossier reads
                   (vehicle.h:dragAtm), so the log and the dossier cannot mean
                   different airs. On a scale-height body that is one number for the
                   whole sky, implied by H; on a layer table (#552 A4) it varies with
                   altitude, which is the whole point of that row. Units K / Pa / m/s,
                   one number per field: the window's "150.4 K (-122.8 C)" gloss is for
                   the human, and a parsed field takes one scale. Kelvin because that
                   is the scale the thermodynamics are linear in.
                   Mach is over the AIR-relative speed at the COM -- NOT the |v|= this
                   line prints. On a spinning body the air itself moves (#97), so the
                   two differ, and the flow regime the ratio describes is the one the
                   air sees. They differ by more than that, so do not divide one by the
                   other to check this field: |v|= is live GetVel() at print time, while
                   the air numbers are what the aero pass left behind (this line prints
                   on a 1 s gate, the pass runs every substep). flight/245 at t=10:
                   |v|=62.84 against Mach 0.228 x a 274.8 = 62.65 m/s. A reader who
                   wants the flow itself should read the per-surface mach= on the
                   [surface] line, which is the same quantity where it is paid for.
                   Dashed when this body has no air at all -- an airless body, or above
                   the shell top, where drag.h:airDensity answers 0. That is a DIFFERENT
                   question from "the pass refused to bill": a hull sitting still in
                   thick air bills nothing and still prints the air around it.
                   Appended last, like every other field here, so the existing
                   parsers keep working. */
                if(g.ship->lastDragRho > 0.0) {
                    printf(" T=%.1f p=%.0f a=%.1f Mach=%.3f",
                           g.ship->lastAirT, g.ship->lastAirP, g.ship->lastAirA,
                           g.ship->lastAirV / g.ship->lastAirA);
                } else {
                    printf(" T=- p=- a=- Mach=-");
                }
                /* #694: L and D of the air's force about the air-relative FLOW -- the
                   textbook pair (vehicle.h:lastFlowLift / lastFlowDrag), and the ratio a
                   glide ratio equals. In the settled 200 s glide it prints 7.19 while the
                   hull's own track says 41.20 m over 5.66 m of sink, 7.28; the 1.3 % left
                   over is a glide that has not quite settled. It is NOT |L|= divided by
                   anything, and the distance from the two readings res/ships/README.md
                   could mean is fully accounted for: |L| over the printed RAW bills x q
                   reads 8.65 there (README's 8.6), and 6 % of that is simply --drag-cd --
                   A=/Cd= print the raw hull product while the force charges 1.2 x them, as
                   the block above this one says. Charge the bills and they add up: 745.0 N
                   against the 744.0 N this row's lastFlowDrag measures, leaving 8.12 of
                   |L| over the drag. The last step, 8.12 -> 7.19, is the numerator: the
                   lift law's sum leaves a control surface's deflection force out, and on a
                   hull trimmed by a 12 deg tab that is 12.9 % of what holds it up (#699).
                   Dashed on the same test as AoA/Beta/CpL, whenever the air is PUSHING the
                   hull rather than holding it back (a negative denominator makes the ratio
                   meaningless, not large), and whenever the along-flow half is under a
                   thousandth of the force -- a ratio there is four digits, and that is not
                   a glider, it is a row with no drag in it to compare against. */
                if(g.ship->lastDragAlphaValid && g.ship->lastFlowDrag > 1e-6
                   && g.ship->lastFlowDrag * 1000.0 > glm::length(g.ship->lastAeroForce)) {
                    printf(" Ld=%.2f", g.ship->lastFlowLift / g.ship->lastFlowDrag);
                } else {
                    printf(" Ld=-");
                }
                printf("\n");
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
                /* #485: what each lifting or steering SURFACE's law actually read on
                   the last substep -- its own angle of attack and the dynamic pressure
                   of the flow standing AT it (#483). The [drag] line's AoA is one
                   attitude number for the whole ship; these are the angles the forces
                   come from, and until now the only way to see them was a temporary
                   probe in the aero pass. A surface the pass found unusable (no flow,
                   an edge-on mount) is skipped rather than quoted at 0.
                   mach= (#552 A3) is that surface's OWN flow speed over the sound
                   speed its air implies -- the ship-level Mach on the [drag] line is
                   the COM's, and #483's whole point is that a rotating airframe gives
                   each surface a different flow. Since #543 A5 it is also the number
                   that surface's lift slope is corrected with, and since A5b the number
                   that bills its wave drag, so the number in the log and the number in
                   the law are one number.
                   Labelled by its def id when it has one (wing_l, tail_r) because a
                   twin pair is exactly the case this row exists to tell apart. */
                {
                    bool any = false;
                    for(const Part *p : g.ship->parts) {
                        if(!p->aeroValid || p->def == nullptr) { continue; }
                        if(p->def->lift_area <= 0.0 && p->def->control_area <= 0.0) {
                            continue;
                        }
                        if(!any) { printf("[surface]"); any = true; }
                        printf(" %s a=%+.1f q=%.0f mach=%.3f",
                               p->id.empty() ? p->def->name.c_str() : p->id.c_str(),
                               p->aeroAlphaDeg, p->aeroQ, p->aeroMach);
                    }
                    if(any) { printf("\n"); }
                }
                /* RCS nozzle allocation (#428): which BELLS fired for the
                   commanded move -- [tag] names the bell (part-local), the
                   thrust it produces acts opposite the tag. ach < cmd (or
                   nothing firing at all -- a demand outside every nozzle's
                   cone) is the hardware saying it cannot deliver. */
                if(glm::length2(g.ship->lastRcsCmd) > 0.0) {
                    const glm::dvec3 cmd = g.ship->lastRcsCmd;
                    const glm::dvec3 ach = g.ship->lastRcsAch;
                    printf("[rcs] cmd=(%.0f %.0f %.0f)N ach=(%.0f %.0f %.0f)N",
                           cmd.x, cmd.y, cmd.z, ach.x, ach.y, ach.z);
                    for(const Vehicle::RcsNozzleLog &nl : g.ship->lastRcsNozzles) {
                        printf(" %s[%s]=%.0fN", nl.part.c_str(), nl.tag.c_str(),
                               nl.thrust);
                    }
                    printf("\n");
                }
                fflush(stdout);
            }
        }

        /* --record: the flight recorder (#662, #674). Every channel is a number the
           game already publishes for the tick it is sampled in -- the physics pass's
           readouts and the ShipView snapshot the SURFACE and ORBITAL windows draw --
           so a recorded value and a logged value cannot disagree (#665's seam). It
           is gated on its own clock (--record-interval) because the log rate and the
           trace rate want different things: a trace at 1 s rows is useless next to a
           burn, and one per tick is a 3 MB file.
           Two cadences share a row, on purpose. The aero block's fields are the last
           SUBSTEP's; the frame group is the last REFRESH's (sc.refresh fills hud.view
           after the logic phase, main.cpp:1392), so those ten columns lag the physics
           by part of a frame. That is not sloppiness: they are what the pilot's own
           windows were showing at that instant, and the lag is centimetres at 4 Hz.
           t= counts the LOG clock (run time), not g.time: a load moves the mission
           clock backwards (#304) and would otherwise drop rows out of order in the
           middle of the file. The header's t0= is g.time at the first sample, which
           maps one onto the other. */
        if(g.args.record && g.ship != nullptr) {
            const Uint32 now_ms = g.logClockMs();
            /* The first due sample opens the file (Game::openRecorder): by then
               the run has claimed a game dir if it was going to, and a run that
               never flies leaves no empty trace behind. A failed open says so once
               and the recorder stays off. */
            if(g.logDue(kGateRec, now_ms, g.record_interval_ms) &&
               (g.rec.f != nullptr || (!g.rec.failed && g.openRecorder(now_ms)))) {
                Vehicle *v = g.ship;
                const ShipView &view = g.hud.view;
                RecSample s;
                s.t = (now_ms - g.rec.started_ms) / 1000.0;
                s.alt_asl = v->lastDragAlt;
                s.v = glm::length(v->GetVel());
                s.dv = v->getDeltaV();
                s.acc = v->feltAccel();
                /* The windows' own numbers. A snapshot taken for some other hull --
                   or never taken at all, since an unfilled ShipView reads zeros
                   (#235) -- quotes nothing: the group dashes together on one test,
                   because a 0 in a log gets read as a measurement. */
                if(view.ship == v) {
                    s.alt_agl = view.alt_agl;
                    s.v_surf = glm::length(view.surf_vel);
                    s.v_orb = view.speed;
                    s.vs = view.ver_speed;
                    s.hs = view.hor_speed2;
                    s.lat = glm::degrees(view.latitude);
                    s.lon = glm::degrees(view.longitude);
                    s.pitch = glm::degrees(view.pitch);
                    s.roll = glm::degrees(view.roll);
                    s.hdg = glm::degrees(view.heading);
                } else {
                    s.alt_agl = s.v_surf = s.v_orb = s.vs = s.hs = NAN;
                    s.lat = s.lon = s.pitch = s.roll = s.hdg = NAN;
                }
                s.rho = v->lastDragRho;
                s.F = glm::length(v->lastAeroForce);
                s.L = glm::length(v->lastLiftForce);
                s.Cd = v->lastDragCd;
                s.A = v->lastDragArea;
                s.chute = v->lastChuteCdA;
                if(v->lastDragAlphaValid) {
                    s.aoa = glm::degrees(v->lastDragAlpha);
                    s.beta = glm::degrees(v->lastDragBeta);
                    s.cpl = v->lastCpLever;
                    s.ind = v->lastInducedCdA;
                } else {
                    s.aoa = s.beta = s.cpl = s.ind = NAN;
                }
                if(v->lastDragRho > 0.0) {
                    s.T = v->lastAirT;
                    s.p = v->lastAirP;
                    s.a = v->lastAirA;
                    s.mach = v->lastAirV / v->lastAirA;
                } else {
                    s.T = s.p = s.a = s.mach = NAN;
                }
                s.mass = v->getMass();
                s.thr = v->throttle();
                s.stg = v->activeStage();
                char row[768];
                const int wrote = recRow(s, row, sizeof row);
                if(wrote > 0) {   // -1 = the row did not fit, so none was written
                    fprintf(g.rec.f, "[rec] %s\n", row);
                    g.rec.rows++;
                }
            }
        }

        // --att-log: the ship's nose + angular velocity (attitude-physics e2e)
        if(g.ship && g.args.att_log) {
            const Uint32 now_ms = g.logClockMs();
            if(g.logDue(kGateAtt, now_ms, g.orbit_log_interval_ms)) {
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
            if(g.logDue(kGateCompound, g.logClockMs(), g.orbit_log_interval_ms)) {
                for(auto *s : vehicles) { s->compoundCheck(g.time); }
            }
        }

        // --eva-log: the kerbal's mode + state (the EVA e2e assertions)
        if(g.ship && g.args.eva_log && g.ship->isEva()) {
            const Uint32 now_ms = g.logClockMs();
            if(g.logDue(kGateEva, now_ms, g.orbit_log_interval_ms)) {
                Kerbal *k = static_cast<Kerbal *>(g.ship);
                const glm::dvec3 p = k->get_center_of_mass();
                const glm::dvec3 v = k->GetVel();
                const glm::dvec3 face = k->partAxis(k->controller, 1);
                // viewyaw=, not yaw=: [camlog] already uses yaw= for the camera
                // azimuth, and the two are the other half of the same attitude
                // law (#241). Ground rows print the offset anyway -- it is the
                // one that applies on the next space transition.
                printf("[evalog] t=%.1fs mode=%s grounded=%d "
                       "pos=[%.1f %.1f %.1f] vel=[%.2f %.2f %.2f] "
                       "face=[%.3f %.3f %.3f] alt=%.2f m viewyaw=%.3f "
                       "rcs=%d mass=%.3fkg\n",
                       g.time, (k->evaMode() == EVA_GROUND) ? "ground" : "space",
                       (int)k->evaGrounded(), p.x, p.y, p.z, v.x, v.y, v.z,
                       face.x, face.y, face.z,
                       glm::length(p) - (double)k->m_parent->GetTerrainHeight(
                           glm::vec3(glm::normalize(p))),
                       k->viewYawOffset(), (int)k->rcsBurned(),
                       k->getMass());
                fflush(stdout);
            }
        }

        g.accumulator -= g.dt;
    }

    /* --sim-steps (#472): spend the budget on the ticks actually run, then stop
       the main loop. In this mode the accumulator is set to exactly dt, so one
       frame runs one tick; counting ticks instead of frames keeps the promise if
       that ever changes. */
    if(g.sim_steps_left >= 0) {
        g.sim_steps_left -= (g.logic_ticks - ticks_before);
        if(g.sim_steps_left <= 0) {
            /* sim_seconds_ran is what this run simulated, summed tick by tick:
               neither a load rewinding g.time (which brings its own t0, so a
               difference reads like a confident -9999996.92 s of negative
               flight) nor warp at 1e5 can mislead it. Print both numbers: the
               sim clock alone says nothing to a run that reloaded. */
            printf("Sim steps reached (%lld ticks); the sim advanced %.2f s "
                   "(clock t = %.2f s); exiting main loop.\n",
                   g.args.sim_steps, g.sim_seconds_ran, g.time);
            /* A paused sim still spends ticks and scripted presses, so every log
               row samples the same state. Warn over the WHOLE run -- a per-frame
               check would cry wolf for a run that paused on its last tick and
               miss a run that was frozen for all of it. A case that means a
               paused sim (--time-accel 0 with logs to read) still sees this, so
               the message says which of the two it is. */
            if(!(g.sim_seconds_ran > 0.0)) {
                printf("WARNING: --sim-steps ran %lld ticks and the sim never advanced "
                       "once (--time-accel 0? paused?) -- scripted input fired into a "
                       "frozen world and every log row sampled the same state. Unless "
                       "this case means a paused sim, it measured nothing.\n",
                       g.args.sim_steps);
            }
            fflush(stdout);
            g.running = false;
        }
    }
}
