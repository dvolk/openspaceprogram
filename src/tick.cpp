// tick.cpp -- the fixed-timestep logic tick (see tick.h).
#include "tick.h"

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <limits>

#include "keys.h"                // Slot, SlotProbe (the command map)
#include "angmath.h"             // acosUnit: the one acos(clamp) spelling (#622)
#include "eva.h"                 // Kerbal::armInput (the kerbal's controls)
#include "drag.h"                // formatAlphaDeg: the one spelling of a refused AoA (#1058)
#include "orbit.h"               // OrbitElements, computeOrbitElements
#include "solar.h"               // sunSample: the --solar-log flux (#687)
#include "insolation.h"          // skyAtZenith: the --solar-log elevation and cosine (#724)
#include "groundtemp.h"          // groundTemperature / groundAnchor_K: the surf_T column (#724)
#include "physics.h"             // physics_tick()
#include "timestep.h"            // substepCount()

/* The flight recorder's ONE sampler (#662, #674, #675, #972). Every channel is a
   number the game already publishes for the tick it is sampled in -- the physics
   pass's readouts and the ShipView snapshot the SURFACE and ORBITAL windows draw --
   so a recorded value and a logged value cannot disagree (#665's seam). It is a
   function now because one sample has two sinks: --record's rows, and the ring the
   Recorder window draws (#972). One sampler is not a style preference -- a second
   path built for the plot is exactly how a graph and a trace start telling
   different stories about the same flight.

   Two cadences share a sample, on purpose. The aero block's fields are the last
   SUBSTEP's; the frame group is the last REFRESH's (sc.refresh fills hud.view
   after the logic phase, main.cpp:1780), so those ten columns lag the physics by
   part of a frame. That is not sloppiness: they are what the pilot's own windows
   were showing at that instant, and the lag is centimetres at 4 Hz.

   t= is the run-clock delta the CALLER counts from -- the file from the trace's
   opening (started_ms), the ring from its own first sample (ring_t0_ms) -- so both
   sinks can stamp their own baseline off the one sample. The ring also carries
   g.time, which is what its plot is against. */
static RecSample recSample(Vehicle &v, const ShipView &view, double t) {
    RecSample s;
    s.t = t;
    s.alt_asl = v.lastDragAlt;
    s.v = glm::length(v.GetVel());
    s.dv = v.getDeltaV();
    s.acc = v.feltAccel();
    /* The windows' own numbers. A snapshot taken for some other hull --
       or never taken at all, since an unfilled ShipView reads zeros
       (#235) -- quotes nothing: the group dashes together on one test,
       because a 0 in a log gets read as a measurement. */
    if(view.ship == &v) {
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
    s.rho = v.lastDragRho;
    s.F = glm::length(v.lastAeroForce);
    s.L = glm::length(v.lastLiftForce);
    s.Cd = v.lastDragCd;
    s.A = v.lastDragArea;
    s.chute = v.lastChuteCdA;
    if(v.lastDragAlphaValid) {
        s.aoa = glm::degrees(v.lastDragAlpha);
        s.cpl = v.lastCpLever;
        s.ind = v.lastInducedCdA;
        s.side = v.lastSideCdA;
        s.sidecp = v.lastSideCpLever;
    } else {
        s.aoa = s.cpl = s.ind = NAN;
        s.side = s.sidecp = NAN;
    }
    /* Beta has its own floor: the flow that gives AoA a direction need not
       give yaw one (AeroFrame::betaValid, #787). A recorded NaN prints as
       "-" in the flight log, which is what a row with no sideslip means. */
    s.beta = v.lastDragBetaValid ? glm::degrees(v.lastDragBeta) : NAN;
    if(v.lastDragRho > 0.0) {
        s.T = v.lastAirT;
        s.p = v.lastAirP;
        s.a = v.lastAirA;
        s.mach = v.lastAirV / v.lastAirA;
    } else {
        s.T = s.p = s.a = s.mach = NAN;
    }
    s.mass = v.getMass();
    s.thr = v.throttle();
    s.stg = v.activeStage();
    /* #675: the ascent profile. q and the four totals are the hull's own
       last substep, so they ride no gate -- 0 is what they read with no
       air and with no burn. gamma is ShipView's, so it takes the
       snapshot test like the frame group above it, and it is already NaN
       for a state with no flight path (orbit.h), which prints as "-"
       rather than as level flight. */
    s.q = v.lastDynPressure;
    s.gamma = (view.ship == &v) ? glm::degrees(view.flight_path) : NAN;
    const AscentLosses &loss = v.ascentLosses();
    s.dv_exp = loss.dv;
    s.grav = loss.grav;
    s.drag = loss.drag;
    s.steer = loss.steer;
    s.burns = loss.burns;
    /* #972: the hull's own dock count, so a trace can say WHERE it changed size.
       A reading, not a derivation: Game::undock mints a vessel by extractSubtreeAsShip
       and updateDocking's absorbShip retires the target, and neither reassigns
       Game::ship -- so this column is the only thing that marks those seams, on the
       plot or in the file. */
    s.seams = (int)v.seams.size();
    return s;
}

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
        /* The suit's half of that rule (#215). A kerbal's laws run per substep for
           EVERY live hull, but its control state is armed once a tick for one ship
           only -- g.ship, below, and only in a piloting scene outside free cam. So
           every live kerbal is neutralized here and the arm below re-arms the one it
           owns: without this, switching away from a spacewalker leaves it walking on
           the demand it last had, and free cam leaves even the ACTIVE suit wearing the
           constructor's "standing" default, whose walk brake pulls 10 m/s^2 out of an
           orbit. (A stale rcsDir is the same hazard and burns hydrazine doing it; a
           handover already clears it -- Vehicle::releaseControl, #580 -- but nothing
           else cleared it, and free cam is not a handover.)
           Skipped while paused and while railed, for the same reason the substep loop
           skips them: no substeps, no law to neutralize. That leaves one window INSIDE
           this tick -- the proximity pass below takes hulls off the rails after this
           loop has passed them -- so game.cpp::updateProximity() disarms each suit as
           it wakes it. */
        if(g.time_accel != 0) {
            for(Vehicle *s : vehicles) {
                /* Skip the active suit ONLY when the arm below will actually
                   run (same doors: not free cam, a piloting scene, running):
                   armInput re-derives everything disarm would and must KEEP
                   the grab/jump edges this batch raised -- disarm spends
                   them (#1073). Free cam arms nothing, so its active suit
                   still owes a neutral tick here (#215). */
                const bool willArm = s == g.ship && g.camera->mode != CAM_FREE
                                     && curScene(g).pilot && g.time_accel > 0;
                if(s->onRails || !s->isEva() || willArm) { continue; }
                static_cast<Kerbal *>(s)->disarm(vehicles);
            }
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
                bool want_stage = false;
                flyAscent(*g.ship, *g.ascent, g.dt * g.time_accel, &want_stage);
                if(want_stage) {
                    /* The press lives here, not in the driver: separating a hull
                       restructures the world -- a new Vehicle in the list, chutes,
                       a deduped name -- so it is a Game action, and the law stays
                       pure (#928). Before the physics substeps, so the newly
                       ignited engines light on the tick the law asked.
                       The outcome is printed because a refusal the player cannot
                       see is a mission that quietly stops staging: crew aboard is
                       the common one, and the law will not ask again (#928). */
                    /* Read the law's two reasons before acting on them. The law
                       itself stays a yes/no: which half fired is a property of the
                       hull, and printing it costs two predicates on a tick that is
                       already separating a rocket. Without this line a dry-stage
                       press and a due-stage press look identical in the log, and
                       they mean different things to a designer (#928). */
                    const bool due = g.ship->stageIsDue();
                    const bool dry = !g.ship->canBurn();
                    const Game::StageOutcome oc = g.stageVehicle(g.ship);
                    printf("[stage] t=%.1fs autopilot press: %s (%s%s)\n", g.time,
                           Game::stageOutcomeName(oc),
                           due ? "stage drained" : "",
                           dry ? (due ? " + dry" : "tanks dry") : "");
                    /* The flying stack stays the hull the law was armed on:
                       extractSubtreeAsShip keeps the parent's parts in the parent
                       (parts.swap(keep)) and stageVehicle never reassigns Game::ship,
                       so neither ship nor ascent_ship needs repointing. Asserted
                       rather than assumed -- if that swap ever changes, the law would
                       be left flying a husk with nothing said. */
                    assert(g.ship == g.ascent_ship);
                    /* Tell the law back what happened (#941). Its only feedback was
                       stages_left, which a refusal leaves exactly where it was, so the
                       once-per-value latch made every refusal permanent. Both refusals
                       are reported, for different reasons: a crew refusal is fixable
                       mid-flight -- EVA her, and the next ask separates -- so the law
                       releases its latch and re-asks about once a second; a root refusal
                       is not fixable up here (which seam is the ship's root is decided
                       in the VAB), so re-asking would print one refusal per second until
                       the mission ended, and the law only needs to remember it to stop
                       the verdict saying "the tanks are dry" with stages still attached.
                       NothingSeparated already stepped the counter, which is the law's
                       normal signal, and NoHull means there is nothing left to fly. */
                    if(oc == Game::StageOutcome::CrewRefused) { g.ascent->pressRefused(PressRefusal::Crew); }
                    else if(oc == Game::StageOutcome::RootRefused) { g.ascent->pressRefused(PressRefusal::Root); }
                }
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

        // Driven buildings (orbital docks/stations, air platforms) ride the
        // clock even while PAUSED: the pose is a pure f(time), so driving it
        // here keeps a building placed at the current time the moment a ship
        // spawns beside it, rather than only appearing once the sim unpauses.
        // Rails warp (>= kRailsWarp) skips the substep loop too, so it drives
        // here as well. While PHYSICS runs, the substep loop owns the pose (it
        // ends exactly on g.time); driving here too would jump the deck a full
        // step ahead and the first substep would snap it back -- a shudder.
        if(g.time_accel == 0 || g.time_accel >= kRailsWarp) {
            g.driveBuildingPoses();
        }
        // Runs paused too, so a ship spawned beside a driven building has its
        // collider right away (see Game::gateBuildingColliders).
        g.gateBuildingColliders();
        /* A stopped world still HAS a power balance, so record it with a
           zero-length tick: netWh is 0, so this pays nothing and charges nothing.
           #659 moved the Systems Net row and --power-log's net= onto the rate
           powerTick strikes its balance on, and a hull that has not ticked in this
           process -- a load, a quickload, a fresh spawn -- would otherwise print a
           record of a tick that never happened next to live term rows: "+0.0 W
           (charging)" under 298.4 W of generation and 600.0 W of life support, in
           exactly the state a player pauses to decide whether to raise the warp
           dial. The EC gate and the wheels' brown-out latch that this also refreshes
           are DURATIONS, never the caller's step (#898), so a paused call reaches
           the verdict the first unpaused substep would reach, one frame earlier and
           with no Wh spent to get there. */
        if(g.time_accel == 0 && g.ship) { g.ship->powerTick(0.0); }
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
            // tick's frame transforms. The cull rides the same pass: it reads
            // the same distances, and a hull it retires is out of the list
            // before anything else walks it this tick.
            g.updateProximity();
            g.cullFarDebris();

            g.sun->frame->UpdateOrbitRails(g.time);

            if(g.time_accel >= kRailsWarp) {
                /* Rails warp: analytic coast (or frozen on the ground);
                   Bullet is not stepped. */
                const double step = g.dt * g.time_accel;
                RailEnd end = RailEndNone;
                Vehicle *ended = nullptr;
                for(auto *s : vehicles) {
                    /* This branch steps CONICS only: railAdvance() answers
                       RailEndNone for an off-rails hull, so such a hull gets no
                       gravity, no forces and no integration here at all -- it
                       stands still while g.time runs on. Loud beats silent: every
                       leaveRails() caller bounds the warp as it leaves (control
                       wake above, select_vehicle, stage, undock), and
                       Game::updateProximity() now clamps its cap below
                       kRailsWarp, because it is the one caller that takes hulls
                       off the rails without the player asking. */
                    assert(s->onRails
                           && "off-rails hull at rails warp: it will not integrate");
                    const RailEnd e = s->railsTick(g.time, step);
                    if(e != RailEndNone && ended == nullptr) { end = e; ended = s; }
                }
                // A rail SoI hop can move the active ship's frame too.
                g.syncWorldFrame();
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
            // The Bullet world's static content rides the active ship's frame;
            // re-express it before the substeps if that frame just changed.
            g.syncWorldFrame();

            // Integrate the step in substeps, re-applying gravity + thrust +
            // rotation before EACH: Bullet clears forces at each step, and
            // velocity-dependent terms must not freeze at the step start.
            // Grow the count so the substep stays <= kMaxSubStep at high
            // time-accel.
            const double step = g.dt * g.time_accel;
            const int n = substepCount(step);
            const double h = step / n;
            for (int i = 0; i < n; i++) {
                // Driven buildings advance their POSE on the substep clock (the
                // gate, [bldg] log and draw read posePos). A LIVE deck's collider
                // does not move: it is static at its bubble origin (#850,
                // writeColliderPose), so there is no per-tick teleport to bounce
                // a parked ship. The substeps span [g.time-step, g.time]: the
                // first continues from the previous tick's last substep, the
                // last lands exactly on the clock the gate/log/draw read -- no
                // lead, no boundary jump, smooth across pause and warp changes.
                // Pure f(time).
                const double tSub = g.time - step + (i + 1) * h;
                for(TerrainBody *b : g.sys.bodies) {
                    for(StaticBuilding *p : b->pads) {
                        if(p->driven()) { p->tickPose(tSub); }
                    }
                }
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
                    s->applyAeroForce(h, g.sun);
                    // Gear ground reaction (a no-op without wheels). Same fresh
                    // pose/velocity as aero; physics_tick below integrates it.
                    s->applyGroundForces(h);
                    /* #675: last, so it reads the three forces THIS substep handed
                       the integrator -- thrust, gravity, air -- in one breath. It
                       charges nothing Bullet has not already been given, and a
                       railed hull never reaches here: no substep, nothing to
                       charge, the same rails gap as drag. */
                    s->lossTick(h);
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

        // --dbg-log: each driven building's pose (in its poseFrame: inertial
        // for orbital, rotating for air), so a test can pin that it propagates.
        // AFTER the substeps so the pose is exactly at g.time (the substep loop
        // ends on the clock); outside the pause gate so a paused spawn is
        // observable too (the paused drive above posed it).
        if(g.args.dbg_log
           && g.logDue(kGateBldg, g.logClockMs(), g.orbit_log_interval_ms)) {
            for(TerrainBody *b : g.sys.bodies) {
                for(StaticBuilding *p : b->pads) {
                    if(!p->driven()) { continue; }
                    printf("[bldg] t=%.1fs id=%s pos=[%.1f %.1f %.1f] r=%.1f m "
                           "live=%d\n",
                           g.time, p->id.c_str(), p->posePos.x, p->posePos.y,
                           p->posePos.z, glm::length(p->posePos), p->live ? 1 : 0);
                }
            }
            fflush(stdout);
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
                /* A dash, not a number: a railed hull or a mu-less body never got a
                   conic, and elements() still held the previous tick's -- or a
                   default-constructed one, which printed this row's periapsis as a
                   confident number the law had never measured. '-' is what every other
                   row-based log prints for "no value" ([orbitlog] lan=/lpe=, [planelog]
                   prg=, [drag] AoA=, [slew] w_cap=), and the unit lives in the buffer
                   for the same reason those printers put it there: a dashed field
                   carries no unit, so e2e's ASCENT_RE can tell the two apart without a
                   numeric arm that eats a lone '-' and hands float() something it cannot
                   read. A nan here is no longer something the harness reads: it matches
                   neither arm, so the row drops and e2e names it (#408). The dash also
                   makes a CHECK against an absent field fail against None instead of
                   quietly comparing a float nan (#610 review, #894). */
                auto optNum = [](char *buf, size_t n, const char *unit, double v,
                                 bool ok) {
                    if(ok) { std::snprintf(buf, n, "%.6g%s", v, unit); }
                    else { std::snprintf(buf, n, "-"); }
                };
                const double sea_r = g.ship->m_parent->seaLevelRadius();
                char peri_txt[32], apo_txt[32], inc_txt[32], node_txt[32];
                optNum(peri_txt, sizeof peri_txt, " m", periapsisAsl(o, sea_r), conic);
                /* apoapsisAsl() asserts a closed conic, so the dash lives here rather
                   than in an `ok` argument -- the value must never be computed. */
                if(conic && o.traj == TrajectoryClass::Elliptic) {
                    optNum(apo_txt, sizeof apo_txt, " m", apoapsisAsl(o, sea_r), true);
                } else {
                    std::snprintf(apo_txt, sizeof apo_txt, "-");
                }
                /* Inclination against the body's equator, from the pole (see
                   flyAscent: omega() is zero for a body that does not spin, which
                   used to make this field print -1.00 as a stand-in for "unknown").
                   "Is there a plane" is orbit.h's conicNormalOk, the same test the
                   autopilot's normal pair and the drawn marker use: `ang_momentum > 0.0`
                   was a test for exactly zero, and a near-radial state's h is not near
                   zero -- it is r |v| sin(gamma), a large number whose direction is the
                   phase of the noise (#900). Such a hull has no inclination to print. */
                double inc_spin = 0.0;
                const bool plane_ok = conic && conicNormalOk(o.ang_momentum, o.distance);
                if(plane_ok) {
                    Frame *inertial = g.ship->frame->getNonRotFrame();
                    if(inertial == nullptr) { inertial = g.ship->frame; }
                    const double c = glm::dot(o.h_hat, g.ship->frame->spinAxisRelTo(inertial));
                    inc_spin = glm::degrees(acosUnit(c));
                }
                optNum(inc_txt, sizeof inc_txt, " deg", inc_spin, plane_ok);
                optNum(node_txt, sizeof node_txt, " m", law.node_r(),
                       law.node_r() > 0.0);
                /* The flight-path angle of the conic the law is steering (#675) --
                   the same helper ShipView publishes, so the row and the UI cannot
                   disagree about what gamma means. Dashed like the other apsides on
                   a state with no velocity to have an angle through. */
                const double gamma = flightPathAngle(o);
                char gamma_txt[32];
                optNum(gamma_txt, sizeof gamma_txt, " deg",
                       std::isfinite(gamma) ? glm::degrees(gamma) : 0.0,
                       std::isfinite(gamma));
                /* The two numbers the law's lift-off gate compares, printed so a
                   refusal like "this stage cannot lift off" can be checked against
                   the hull instead of taken on trust. accel is availableAccel() --
                   rated full throttle, current mass, atmosphere and all -- and g is
                   the body's pull at the hull's own radius. */
                const double amax = g.ship->availableAccel();
                const double r_hull = glm::length(
                    g.ship->posInBodyRot(g.ship->get_center_of_mass()));
                const double g_pull = (r_hull > 1.0)
                    ? g.ship->m_parent->mu / (r_hull * r_hull) : 0.0;
                /* #675: what the flight is made of, so a case can pin a loss total
                   against the same rows that drove the flight. Appended AFTER why=
                   rather than woven in: e2e's ASCENT_RE is positional, so a column
                   inserted in the middle matches nothing, an empty match list makes
                   every all(...) CHECK vacuously true, and run.py documents exactly
                   that trap (e2e/run.py:703-710). alt_asl is above the body's datum
                   (the name the repo uses for that datum, #626), v_orb is the
                   conic's speed -- the law's own, not the ground's -- q is the
                   hull's dynamic pressure against its AIR (0 where there is no
                   air), and
                   dv/grav/drag/steer are the hull's running totals from losses.h:
                   what its burns cost, with burns= how many burns they cover. */
                const AscentLosses &loss = g.ship->ascentLosses();
                printf("[ascent] t=%.1fs phase=%s lever=%.2f pitch=%.1f deg "
                       "peri_asl=%s apo_asl=%s inc_eq=%s node_r=%s "
                       "accel=%.2f m/s2 g=%.2f m/s2 why=\"%s\""
                       " alt_asl=%.1f m v_orb=%.2f m/s q=%.1f Pa gamma=%s"
                       " dv=%.1f m/s grav=%.1f m/s drag=%.1f m/s steer=%.1f m/s"
                       " burns=%d\n",
                       g.time, law.phaseName(), g.ship->throttle(), law.pitch_deg(),
                       peri_txt, apo_txt, inc_txt, node_txt, amax, g_pull, law.why(),
                       r_hull - sea_r, o.speed, g.ship->lastDynPressure, gamma_txt,
                       loss.dv, loss.grav, loss.drag, loss.steer, loss.burns);
            }
        }
        // --orbit-log: orbital elements in the body's inertial frame.
        if(g.ship && g.args.orbit_log) {
            const Uint32 now_ms = g.logClockMs();
            if(g.logDue(kGateOrbit, now_ms, g.orbit_log_interval_ms)) {
                const double mu = g.ship->m_parent->mu;
                glm::dvec3 o_pos, o_vel;
                /* One spelling of the lift to the body's inertial node, valid
                   from the rot frame and from a building bubble alike (#850). */
                g.ship->comStateIn(g.ship->m_parent->frame, o_pos, o_vel);
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
                /* pos/vel stay frame-local (honest about the ship's frame);
                   alt is body-AGL from the rot-frame position, so it reads the
                   same whether the hull rides the rot frame or a building
                   bubble (#850). */
                const glm::dvec3 p_rot = g.ship->posInBodyRot(p);
                double r = glm::length(p_rot);
                double alt = r - g.ship->m_parent->GetTerrainHeight(glm::normalize(p_rot));
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
           |L| is every lifting surface PLUS every deflected control surface (#699): a
           tab's force is perpendicular to its own flow and pays induced drag like any
           lift, so a column that leaves it out is a lift LAW's sum, not the lift the air
           applies. On a trimmed hull that omission is 13 % of the force holding it up.
           The tell was |L| > |F| -- a part outweighing the whole it came from, which 47 of
           flight/492's 50 rows used to print and which flight/492 now CHECKs. It is a
           tell, not a law: every surface stands in its OWN flow (#483), so on a hull
           pitching hard enough for w x r to matter, "lift" leaks along the flow and
           "drag" leaks across it. Where the surfaces do agree, the ratio sits near
           1 - 1/(2 Ld^2) -- measured 0.988..0.991 on the yardstick skiff at Ld 7.17.
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
        /* #687: the star's light as an instrument, and the first number in the
           game that varies with WHERE and WHEN a hull is. One row per interval:
           distance to the star; the flux crossing a surface NORMAL to the beam
           there; the star's elevation over this hull's own local horizon (negative
           = night); the flux on a horizontal plate there (what a ground radiometer
           reads); then the WORLD's three claims -- the authored Bond albedo, the
           authored surface temperature, and the black-body temperature the flux and
           albedo imply. Read bb_T against ground_T: the gap is what an atmosphere
           does. Earth's own sheet says 254 K of black-body under 288 K of mean, and
           nothing here models that 34 K yet -- which is the honest summary of #687.
           The last three columns DASH on "nobody authored this", not on zero: a 0 K
           ground and a perfectly absorbing (albedo 0) world are both real numbers
           this convention cannot tell from a blank, the same trade the atmosphere's
           `height == 0 means derived` already makes.
           NO EXTINCTION, NO ECLIPSE: `flux` is top-of-atmosphere and unshadowed, so
           flying under Venus's 92-bar sky dims nothing (solar.h states why, so a
           reader who expects a blue-filtered number finds the reason in the law). */
        if(g.ship && g.args.solar_log) {
            const Uint32 now_ms = g.logClockMs();
            if(g.logDue(kGateSolar, now_ms, g.orbit_log_interval_ms)) {
                const glm::dvec3 p = g.ship->GetRootPos();
                const double lum = (g.sun && g.sun->frame)
                                 ? g.sun->star_luminosity : 0.0;
                const glm::dvec3 star = (g.sun && g.sun->frame)
                                      ? g.sun->frame->root_pos : glm::dvec3(0.0);
                const SunSample ss = sunSample(lum, star, p);
                /* The local vertical, from the SoI body the hull is in. A hull at a
                   body's exact centre (a fixture, never a flight) has no horizon, and
                   then elev/flat dash rather than asin(0) = 'overhead'.
                   Both vectors stay in root axes here, which is enough: the dot is
                   frame-invariant and a hull has a position, not a (lon, lat). A sample
                   of a PLACE goes through starFromBody(), which does the rotating-frame
                   step, and lands in the same cosine and elevation (#724). */
                StarFromBody sv;
                sv.to_sun = ss.to_sun;
                sv.valid = ss.valid;
                const glm::dvec3 up = (g.ship->m_parent && g.ship->m_parent->frame)
                                    ? p - g.ship->m_parent->frame->root_pos
                                    : glm::dvec3(0.0);
                const SkyAtZenith sky = skyAtZenith(sv, up);
                const bool have_horizon = sky.valid;
                char flux_s[32], elev_s[32], flat_s[32], dist_s[32];
                if(ss.valid) {
                    snprintf(dist_s, sizeof dist_s, "%.6g", ss.dist_m);
                    snprintf(flux_s, sizeof flux_s, "%.6g", ss.flux);
                } else {
                    snprintf(dist_s, sizeof dist_s, "-");
                    snprintf(flux_s, sizeof flux_s, "-");
                }
                if(have_horizon) {
                    snprintf(elev_s, sizeof elev_s, "%.2f",
                             glm::degrees(sky.elev_rad));
                    snprintf(flat_s, sizeof flat_s, "%.6g",
                             ss.flux * sky.cos_inc);
                } else {
                    snprintf(elev_s, sizeof elev_s, "-");
                    snprintf(flat_s, sizeof flat_s, "-");
                }
                /* The WORLD's claims, read off the SoI body underneath. A hull in
                   the star's own SoI has no ground under it: the star's authored
                   temperature is its photosphere, not a surface a radiometer sits on,
                   so those columns DASH (the load-time audit prints it as a star and
                   means it there). Same answer as a hull over an unsourced moon. */
                const TerrainBody *over = g.ship->m_parent;
                /* #738's other half, in two tiers. The dossier stopped calling a banded
                   giant's authored figure a ground ("temp at 1 bar", "level swing"); this log
                   had kept printing `ground_T=` for Jupiter, one fact and two verdicts
                   (#687). A column that renames itself per row would break every parser, so
                   the giant's GROUND columns dash here -- there is no surface for a hull to
                   stand on -- while its light columns stay: a Bond albedo and an airless mean
                   at Jupiter's rail are as meaningful for a giant as for a rock, and the
                   dossier prints both for it. */
                const bool world_here = over && !over->isStar();
                const bool ground_here = world_here && !over->surface.bands;
                /* The no-ground branch takes the SENTINEL, not 0.0 (#726). The dash
                   this prints looks the same either way -- and that is the point: the
                   READOUT has two absences to draw, but the STATE no longer merges
                   "there is no body under me", "nobody sourced this body" and "this
                   body absorbs everything the star sends" into one value. The first
                   two dash; the third prints 0.000 and a bb_T computed from it, which
                   is the hottest case in the game and used to be unspellable.
                   This guard is also what keeps the sentinel out of
                   blackbodyTemperature, which asserts albedo >= 0: a computation must
                   never see an absence. */
                const double alb = world_here ? over->surface.bond_albedo
                                              : kAlbedoUnauthored;
                const double groundT = ground_here ? over->surface.temperature
                                                : kThermalUnauthored;
                char alb_s[24], gt_s[24], bb_s[24], surf_s[24], swing_s[24], lag_s[24];
                /* bb_T needs BOTH the albedo and a live star sample: printing 0.00 K
                   for "no sun in range" is exactly the 0-pretending-to-be-a-reading
                   this block refuses elsewhere. */
                const double bb_here = derivedBlackbody_K(ss.flux, alb);
                if(bb_here > 0.0) {
                    snprintf(alb_s, sizeof alb_s, "%.3f", alb);
                    snprintf(bb_s, sizeof bb_s, "%.2f", bb_here);
                } else {
                    snprintf(alb_s, sizeof alb_s, "-");
                    snprintf(bb_s, sizeof bb_s, "-");
                }
                if(thermalAuthored(groundT)) {
                    snprintf(gt_s, sizeof gt_s, "%.2f", groundT);
                } else {
                    snprintf(gt_s, sizeof gt_s, "-");
                }
                /* The GROUND under this hull as this star lights it at this instant (#724
                   rung C, and named UNDERFOOT because it is this hull's place and instant,
                   unlike the body's one authored reading in ground_T): the body's ANCHOR -- a
                   stated ground temperature, else the datum of the air on that ground, else
                   the derived black-body, in that order (groundtemp.h spells it once) -- swung
                   by the cosine this row already prints. Both columns dash unless the world
                   has a PRICED knob: there is no default swing (groundtemp.h explains why a
                   fallback there would print a hot ground at midnight, or invent a swing no
                   source stated), and the dash is the honest answer. Where a swing IS authored
                   the knob prints beside it, so the two ends are checkable arithmetic: Venus's
                   1.000 is a STATED no-range, and an em-dash means the source said nothing --
                   NOT the same claim, which is #726's whole complaint.
                   Note the air itself still answers from its own profile: this is the ground,
                   not what a hull flies through (groundtemp.h says why). */
                const double anchor = ground_here
                    ? groundAnchor_K(groundT, airDatum_K(over->surface.atmosphere), bb_here)
                    : 0.0;
                /* #801: WHICH rung answered, named the way the Surface Map names it
                   (`anchor_src=`, surfmap.h). `underfoot_T` is one of three places the engine
                   states a ground temperature -- the atlas dossier is the second, the Surface
                   Map's cursor the third, which carries this same decision in ONE word
                   (`ground +30.7 C (DERIVED)`, #858) because a window line has no room for the
                   phrase -- and for most of a generated system this one is the rung-3
                   black-body: sun + albedo, our arithmetic -- printed beside a `ground_T` that
                   is the source's own reading. Without the rung the two read as the same kind
                   of fact, which is what #722 refuses. The file's own provenance string wins
                   when it carries one (a SOLVED-from-a-range mean is not a stated mean), so
                   this is groundAnchorSource and not the rung alone. */
                const GroundAnchorRung anchor_rung = ground_here
                    ? groundAnchorRung(groundT, airDatum_K(over->surface.atmosphere), bb_here)
                    : kAnchorNone;
                const bool underfoot_here
                    = anchor > 0.0 && sky.valid
                      && evenShareAuthored(over->surface.even_share);
                char src_s[128];
                if(!underfoot_here) {
                    // Dashes with the number it describes: naming the rung of a ground this row
                    // refuses to price would advertise an anchor over a '-'. It is NOT the same
                    // gate as the swing=/lag= one further down, which asks only for a ground and
                    // an authored knob -- a row missing its air or its anchor does print knobs
                    // beside underfoot_T=-. What must never print is a source for no number.
                    snprintf(src_s, sizeof src_s, "-");
                } else {
                    assert(anchor_rung != kAnchorNone);   // underfoot_here needs anchor > 0
                    const std::string src
                        = groundAnchorSource(anchor_rung, over->surface.temperature_src);
                    const int want = snprintf(src_s, sizeof src_s, "\"%s\"", src.c_str());
                    assert(want > 0 && want < (int)sizeof src_s);
                }
                if(underfoot_here) {
                    /* #793: this ground stands at a height, and the body's own profile says what
                       that costs. The height is the GROUND's elevation under the hull, sampled the
                       same way the HUD's AGL and the aero pass sample it -- NOT the hull's own
                       altitude. A hull on the ground makes the two identical, but `flying-trim`
                       and every cruise altitude do not: cooling the reading by the lapse to the
                       hull's height would print the air's temperature and call it underfoot, and
                       would take back, in the ground term, the very lapse the profile column
                       already carries. Not `lastDragAlt` either: the aero pass leaves 0 there for
                       a hull on rails (and for a body with no physical air), while this column
                       recomputes for a rails hull too -- a 0 read as a measurement is the exact
                       thing #687 exists to forbid.
                       The DIRECTION is the body's rotating frame, which is exactly the case this
                       column exists for: a railed hull's coordinates are its inertial node's, and
                       the terrain has spun underneath since the epoch (render.cpp:139's note on
                       AGL). Asking with them reads a height a few thousand kilometres of
                       longitude away. */
                    const glm::dvec3 here = g.ship->posInBodyRot(g.ship->comPos());
                    const double underfoot_elev
                        = (double)over->GetTerrainHeight(glm::normalize(here))
                          - over->seaLevelRadius();
                    /* #829: this column and the surface map are the same ground, so they answer
                       with the same light. A world that authored a phase lag paints a hot spot
                       hours east of noon; asking here for THIS instant's cosine answered 38.6 K
                       hotter than the pixel under the same point at noon, and 77 K colder at hour
                       angle +80 (measured at tau 1, anchor 240, f 0.05) -- one instant, two ground
                       temperatures, and the flight log is the one a player trusts.
                       The lag is a function of LONGITUDE, so the pair goes into the body's
                       spinning axes, and it goes there the same way the map and the aero pass get
                       there: `here` is the height sample above, already rotated, and starFromBody
                       wants ROOT positions and hands back rotating axes. NOT
                       `over->frame->GetOrientRelTo(over->rot_frame)`, which takes a vector in
                       `frame`'s own axes: fed root axes it drops the rail's orientation and prices
                       a different TIME OF DAY while every other column in this row stays put
                       (+5.4 K on an inclined copy of the lag fixture). Taking the star from the
                       body's centre rather than from this hull also puts the log, the map and the
                       air on one subsolar point instead of two a body radius apart.
                       Frame-invariant quantities stay untouched: with no authored tau this is
                       `sky.cos_inc`, the same bit pattern as before, which is why no flight
                       golden moves. */
                    const bool lag = lagTauActive(over->surface.lag_tau);
                    const double forcing = lag
                        ? groundForcingLagged(
                              over->surface.lag_tau, glm::normalize(here),
                              starFromBody(over->frame->root_pos, star,
                                           over->rot_frame->root_orient).to_sun)
                        : sky.cos_inc;
                    snprintf(surf_s, sizeof surf_s, "%.2f",
                             groundTemperatureAt(anchor, over->surface.even_share,
                                                 forcing,
                                                 over->surface.relief_K(underfoot_elev)));
                } else {
                    snprintf(surf_s, sizeof surf_s, "-");
                }
                /* The two knobs of this law gate together. A lag is the RESPONSE of the
                   day/night law that the share prices, so a row printing `lag=1.0000` while
                   `swing=-` and `underfoot_T=-` would advertise a knob the engine refuses to
                   use -- and that is exactly what this column first did (src/system.cpp now
                   refuses a lag on a body that states no share, so this is the readout half of
                   the same rule, and it stays true if that refusal ever moves). */
                const bool law_here = ground_here
                                    && evenShareAuthored(over->surface.even_share);
                if(law_here) {
                    snprintf(swing_s, sizeof swing_s, "%.3f", over->surface.even_share);
                } else {
                    snprintf(swing_s, sizeof swing_s, "-");
                }
                /* #837: the third knob of the same law, printed beside the swing it modifies.
                   `underfoot_T` above is a LAGGED answer whenever a tau is authored (that is
                   #829), so a row that shows the number and not the knob asks the reader to
                   already know it. Same authored/unauthored rule as the dossier's `ground lag`
                   row (#830): a dash means the file states nothing, and a stated 0.0 prints
                   0.0000 -- "this ground peaks at noon" is a reading, not an absence (#727).
                   Radians of phase, four decimals, the unit the file authors and the precision
                   --surfmap-log and --atlas-dump print; hours would be a second unit for one
                   knob, and the atlas row is where the readable form lives.
                   NOT gated by onRails, unlike the four air columns below: those are stale
                   MEASUREMENTS from the last live substep, while this is read off the file the
                   same way `swing` and the recomputed `underfoot_T` are. */
                if(law_here && lagTauAuthored(over->surface.lag_tau)) {
                    snprintf(lag_s, sizeof lag_s, "%.4f", over->surface.lag_tau);
                } else {
                    snprintf(lag_s, sizeof lag_s, "-");
                }
                /* air_T / prof_T / w / follow are MEASURED, not recomputed here: they are
                   the aero pass's own lastAirT, lastAirProfileT and lastAirP, the same
                   numbers its Mach number divides by (vehicle.cpp). Recomputing any of them
                   for the log would be a second authority over one fact, taken at a later
                   instant than the flight model used, and the two would drift (#687) -- and
                   `follow` is printed rather than assumed by the test, so the row's knob is
                   not a third spelling of a constant the file already states.
                   A RAILS hull skips applyAeroForce entirely (tick.cpp's substep loop), so
                   its lastAir* are whatever the last live substep left: printing them would
                   be a frozen noon measurement wearing a live timestamp, and the row would
                   contradict its own recomputed underfoot column. Rails dashes. A hull in
                   vacuum dashes too -- applyAeroForce zeroes these at entry. */
                char air_s[24], prof_s[24], w_s[24], follow_s[24];
                const bool measured = !g.ship->onRails;
                if(measured && g.ship->lastAirT > 0.0) {
                    snprintf(air_s, sizeof air_s, "%.2f", g.ship->lastAirT);
                    snprintf(prof_s, sizeof prof_s, "%.2f", g.ship->lastAirProfileT);
                    snprintf(w_s, sizeof w_s, "%.4f",
                             followWeight(g.ship->lastAirP,
                                          over ? over->surface.atmosphere.sea_level_pressure
                                               : 0.0));
                } else {
                    snprintf(air_s, sizeof air_s, "-");
                    snprintf(prof_s, sizeof prof_s, "-");
                    snprintf(w_s, sizeof w_s, "-");
                }
                if(measured && over
                   && followAuthored(over->surface.atmosphere.air_follow)) {
                    snprintf(follow_s, sizeof follow_s, "%.3f",
                             over->surface.atmosphere.air_follow);
                } else {
                    snprintf(follow_s, sizeof follow_s, "-");
                }
                printf("[solar] t=%.1fs star=\"%s\" dist=%s m flux=%s W/m2 "
                       "elev=%s deg flat=%s W/m2 body=\"%s\" albedo=%s "
                       "ground_T=%s K bb_T=%s K underfoot_T=%s K underfoot_src=%s "
                       "swing=%s "
                       "lag=%s air_T=%s K prof_T=%s K w=%s follow=%s\n",
                       g.time,
                       g.sun ? g.sun->name.c_str() : "(none)",
                       dist_s, flux_s, elev_s, flat_s,
                       g.ship->m_parent ? g.ship->m_parent->name.c_str()
                                        : "(none)",
                       alb_s, gt_s, bb_s, surf_s, src_s, swing_s, lag_s, air_s, prof_s, w_s,
                       follow_s);            }
        }

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
                   for a yaw law (#450) -- this is the debug line.
                   Dashed on BETA's floor, not AoA's: a flow can point anywhere
                   except along the hull's up axis and still have no yaw to name
                   (#787), which is exactly the broadside a parked hull sits in. */
                if(g.ship->lastDragBetaValid) {
                    printf(" Beta=%.1f deg", glm::degrees(g.ship->lastDragBeta));
                } else {
                    printf(" Beta=-");
                }
                /* #486: the hull's weathervane lever, signed metres (+ = cp ahead
                   of the COM). Hull drag answers to the flow AT cp now, and the
                   sweep there opposes a rotation whatever the sign; the sign decides
                   the OTHER half -- hull drag acting at an angle of attack restores
                   the nose only with cp behind, and diverges it with cp ahead
                   (#454). Printed last, like Beta, so parsers hold. Dashed on the same
                   test as AoA (Beta dashes on a stricter one of its own, #787): 0.00 is
                   also what a cp exactly over the COM prints, and a --drag-cd 0 pass has
                   no lever worth quoting. */
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
                   dash together on the same test as AoA/CpL, so a winged ship in
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
                   T= and a= are the profile's ONLY where the body does not author
                   air_follow. Where it does (#724 rung D), the aero pass warms this
                   air toward the ground under the hull and recomputes the sound speed
                   from it (vehicle.cpp: coupledAirTemperature / soundSpeedFromT), so
                   T and a are that COUPLED pair while p and rho stay on the profile.
                   The row therefore does not close as an ideal gas: the 8.5 km cruise
                   bed prints T=239.6 p=33704 rho=0.50195, and p/(rho R) is 233.9 K --
                   the profile, which is also what --solar-log prints as prof_T and what
                   the thermometer gauge reads. The 5.7 K split is the coupling, not a
                   stray variable, and the Mach below divides by the coupled a because
                   that is the speed of sound the wave-drag bill pays by (#687). #783 is
                   open on which half moves -- density onto the coupled air, or the
                   readout back onto the profile -- so do not "fix" this printf to
                   prof_T without it: that would undo #724's Mach silently.
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
                   over is a glide that has not quite settled. Since #699 this field also
                   agrees with the row's own numbers: |L| over the CHARGED bills -- q x
                   (1.2 CdA + Induced + Profile) -- reads 6.67..7.27 on flight/492's hold,
                   median 7.17, which is this field's median to the digit. Over the RAW
                   printed bills it reads 7.14..7.76 (median 7.66), and that 6 % is
                   --drag-cd and nothing else: A=/Cd= print the raw hull product while the
                   force charges 1.2 x them, as the block above this one says; charge them
                   and they add up (745.0 N against the 744.0 N this row's lastFlowDrag
                   measures). res/ships/README.md's skiff L/D of 8.6 was built off the raw
                   bills AND off the pre-#699 lift sum, so it is stale on both counts;
                   which ratio it should lead with is still #694's open call.
                   Dashed on the same test as AoA/CpL, whenever the air is PUSHING the
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
                /* #947: the CROSSFLOW NORMAL bill and its lever. CpL above says
                   where the axial union pushes; on a stacked rocket that landed on
                   the nose caps (+3.7 m on the ascending mars_explorer_mk1) because
                   the partition credits hidden bodies nothing at all. SideCdA= is
                   what the sides of those hidden bodies pay instead -- cn x the
                   side strip their own shadow refuses, m^2 -- and SideCp= is the
                   arm of that push (+ ahead of the COM, signed like CpL; on a
                   stack it lands AFT, which is the restoring half the tail used to
                   carry alone). The two are NOT folded into CpL or FormCp: normal
                   force and axial drag are perpendicular forces, and one lever
                   cannot stand for two lines of action. Dashed with no usable flow,
                   like CpL. SideCdA=0.00 with a real flow is honest three ways --
                   every body part exposed (paying its flank in the axial bill
                   instead), the flow along the parts' axes (the sin lives in the
                   strip), or no body parts at all -- so read SideCp only beside a
                   nonzero SideCdA. */
                if(g.ship->lastDragAlphaValid) {
                    printf(" SideCdA=%.2f m2 SideCp=%+.2f m",
                           g.ship->lastSideCdA, g.ship->lastSideCpLever);
                } else {
                    printf(" SideCdA=- SideCp=-");
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
                   twin pair is exactly the case this row exists to tell apart.
                   watts= (#730) is the rate the air is taking energy off that surface --
                   its own q x its own CdA x its own flow speed -- which is where a wing
                   root, a tip and a control surface differ by 20x while the [drag] line
                   still says one drag coefficient for the ship. Both of the surface's
                   bills are in it: the form drag of #455 and the induced price of its
                   lift (#556), which lands at the same mount on the same q. It is a
                   power, not a temperature: no material property is involved, and most
                   of those watts leave in the wake.
                   The closure to check by hand: take each part's CdA back out of its
                   watts (watts over q x the speed its own q implies) and the parts of a
                   row add back to the [drag] line's Profile= plus Induced= -- 1.00
                   measured over a Kerbin cruise, pinned by flight/730. That is #699's
                   lesson applied to a per-part row before the bug exists. The row covers
                   every surface the form-drag block billed (#732), which is what makes
                   that closure add up: a plate broadside to the flow has no lift frame,
                   so its ANGLE is refused -- a=- rather than the 0.0 that would invent an
                   incidence -- while its planform is still being charged as a flat plate,
                   and its q, Mach and watts ride the flow without needing a frame. */
                {
                    bool any = false;
                    for(const Part *p : g.ship->parts) {
                        if(!p->aeroValid || p->def == nullptr) { continue; }
                        char a[8];
                        /* One formatter for this column, the part window's AoA: row and
                           the F4 arrow labels (#1058). The non-finite guard and the
                           width assert live inside it (drag.h::formatAlphaDeg), which is
                           where the note about "+inf" breaking the runner's ITEM grammar
                           now sits -- as does #732's dash for a refused lift frame. */
                        formatAlphaDeg(a, sizeof a, p->aeroAngleValid,
                                       (double)p->aeroAlphaDeg);
                        if(!any) { printf("[surface]"); any = true; }
                        printf(" %s a=%s q=%.0f mach=%.3f watts=%.0f",
                               p->id.empty() ? p->def->name.c_str() : p->id.c_str(),
                               a, p->aeroQ, p->aeroMach,
                               p->aeroDragWatts);
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

        /* --record and the recorder ring (#662, #674, #972): ONE sample per due
           tick, two sinks. It is gated on its own clock (--record-interval)
           because the log rate and the trace rate want different things: a trace
           at 1 s rows is useless next to a burn, and one per tick is a 3 MB file.
           The RING fills whether or not --record was passed -- the window a player
           opens has no flag to pass -- while the FILE keeps its
           open-on-first-due-sample rule, so a run that never flies leaves no empty
           trace behind, and a failed open says so once and the recorder stays off.
           t= counts the LOG clock (run time), not g.time: a load moves the mission
           clock backwards (#304) and would otherwise drop rows out of order in the
           middle of the file. The header's t0= is g.time at the first sample, which
           maps one onto the other. The ring carries g.time alongside it (#972)
           because the window plots MISSION time, so a warp coast stays proportional
           even though the gate is run time; and a load clears the ring
           (Game::clearEphemeralState), so the backwards clock is not the plot's
           problem either. */
        if(g.ship != nullptr) {
            const Uint32 now_ms = g.logClockMs();
            if(g.logDue(kGateRec, now_ms, g.record_interval_ms)) {
                // The ring's clock starts at ITS first sample: the trace's
                // started_ms only exists if --record opened a file.
                if(!g.rec.ring_armed) {
                    g.rec.ring_armed = true;
                    g.rec.ring_t0_ms = now_ms;
                }
                RecSample s = recSample(*g.ship, g.hud.view,
                                        (now_ms - g.rec.ring_t0_ms) / 1000.0);
                /* The window's Pause drops the ring copy only: the file goes on,
                   and the sampler runs either way, so resuming continues from the
                   state the hull is in NOW rather than leaving a hole the size of
                   the pause. One sampler, two sinks (#972). */
                if(!g.rec.paused) {
                    g.rec.ring.push(RecPoint{.s = s, .sim_t = g.time});
                }
                if(g.args.record &&
                   (g.rec.f != nullptr ||
                    (!g.rec.failed && g.openRecorder(now_ms)))) {
                    s.t = (now_ms - g.rec.started_ms) / 1000.0;
                    char row[1024];
                    const int wrote = recRow(s, row, sizeof row);
                    if(wrote > 0) {   // -1 = the row did not fit, so none was written
                        fprintf(g.rec.f, "[rec] %s\n", row);
                        g.rec.rows++;
                    }
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
                /* alt is body-AGL from the rot-frame position, like [dbg]:
                   |p| alone is the building distance in a bubble (#850). */
                const glm::dvec3 p_rot = k->posInBodyRot(p);
                // viewyaw=, not yaw=: [camlog] already uses yaw= for the camera
                // azimuth, and the two are the other half of the same attitude
                // law (#241). Ground rows print the offset anyway -- it is the
                // one that applies on the next space transition.
                const char *modeName = (k->evaMode() == EVA_GROUND) ? "ground"
                                     : (k->evaMode() == EVA_LADDER) ? "ladder"
                                     : "space";
                printf("[evalog] t=%.1fs mode=%s grounded=%d "
                       "pos=[%.1f %.1f %.1f] vel=[%.2f %.2f %.2f] "
                       "face=[%.3f %.3f %.3f] alt=%.2f m viewyaw=%.3f "
                       "rcs=%d mass=%.3fkg ladder=%llu\n",
                       g.time, modeName,
                       (int)k->evaGrounded(), p.x, p.y, p.z, v.x, v.y, v.z,
                       face.x, face.y, face.z,
                       glm::length(p_rot) - (double)k->m_parent->GetTerrainHeight(
                           glm::vec3(glm::normalize(p_rot))),
                       k->viewYawOffset(), (int)k->rcsBurned(),
                       k->getMass(), (unsigned long long)k->ladderGrip());
                fflush(stdout);
            }
        }

        /* #652: an EC pool that runs dry takes the wheels away, and at warp the hop that
           finishes it can land in the middle of a transfer -- seconds before the player
           reaches the far end, tries to point for a brake burn and finds the drums shut.
           Say it when it happens rather than leaving it to the EC bar. One notice per
           crossing (Vehicle::takeEcDryNotice), so a hull parked at empty since a hop does
           not shout every frame. There is nothing to hedge and no watts to re-read: the
           latch floors the pool at `cost * kWheelsCutS / 3600` on any hull whose generator
           covers its air, so a hull that reaches literal zero has a generator that cannot
           (test_power's test_surplus_floors_the_pool_and_never_crosses). Toasts draw on one
           unwrapped line, so the sentence stays under 70 characters with a 10-character
           hull name; the Systems window's `Status:` row (#649) is the durable version. */
        for(auto *s : vehicles) {
            if(!s->takeEcDryNotice()) { continue; }
            g.toast("EC dry on '%s': wheels cut, nothing aboard recharges them",
                    s->displayName().c_str());
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
