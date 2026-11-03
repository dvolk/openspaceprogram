/* Ascent guidance: fly a ship from a body's surface to a circular orbit.
   See the bottom of this header for what the law does NOT do yet.

   The law is split in two on purpose:

     AscentLaw  -- PURE. It takes a state (position, velocity, mu, what the ship
                   can do) and returns an aim and a throttle. It has no Vehicle,
                   no Frame, no renderer, so a test can feed it a synthetic
                   trajectory and watch the phases, the cutoff and the NaN
                   boundaries without a hull or a world.
     flyAscent -- the driver (guidance.cpp). It does the frame dance, feeds
                   AscentLaw, and sends the result through Vehicle::Command --
                   the same door a key or an Autopilot-window click uses (#424).

   Everything the law computes is in the body's NON-ROTATING frame. That is not
   tidiness: inside a rotating surface frame GetVel() is the ground-relative
   velocity, which on a ship crawling off a pad points nowhere near its orbit's
   prograde, and a 100 km orbit over the Mun sits right on the 300 km rotating-
   frame shell edge (the frame switch bands are 290/310 km), so "which frame am
   I in" is not answerable at design time. The driver converts once per tick and
   converts the aim back; the law never sees a frame at all.
*/
#ifndef GUIDANCE_H
#define GUIDANCE_H

#include <cassert>
#include <glm/glm.hpp>
#include "orbit.h"

/* What the mission wants. Altitudes are metres ABOVE SEA LEVEL (the body's
   datum), because that is what periapsisAsl()/apoapsisAsl() return and what a
   player would read off the navball -- not radii (#404).
   peri_asl is the FLOOR and apo_asl the CEILING, so peri <= apo: the phase
   machine stops its climb at apo and stops its node burn when peri is caught,
   and an ask with them swapped has no stopping point (asserted in the ctor). */
struct AscentTarget {
    double apo_asl = 100e3;
    double peri_asl = 100e3;
    double inc_rad = -1.0;   // <0: keep whatever the launch geometry gives us.
                             // A sentinel, not a range: an inclination is 0..180,
                             // so a caller that parses a field must ask whether a
                             // value was GIVEN, not test the number for < 0.
};

/* The fields a player types into the Guidance window, held apart from the law
   they arm (#612). These outlive a mission -- the loop is read the readout,
   tweak a field, re-fly -- while an AscentLaw is thrown away and rebuilt from
   them, phase machine and all. */
struct AscentAsk {
    double peri_asl = 100e3;
    double apo_asl = 100e3;
    /* Whether a plane was ASKED is its own flag, not a magic number: an
       inclination is 0..180 deg, so no value is free to mean "none" -- reading
       a negative field as the law's sentinel made a due-equator ask unsayable
       (#610 review). Off: fly the plane this pad already gives us. */
    bool ask_inc = false;
    double inc_deg = 0.0;

    AscentTarget target() const {
        AscentTarget t;
        t.peri_asl = peri_asl;
        t.apo_asl = apo_asl;
        t.inc_rad = ask_inc ? glm::radians(inc_deg) : -1.0;   // the law's "not asked"
        return t;
    }
};

enum class AscentPhase {
    Pad,      // still on the ground (or just cleared it): vertical
    Ascent,   // burning, commanding a path angle, until Ap reaches the ask
    Coast,    // Ap made: falling toward the node with the engine off
    Circ,     // the circularisation burn, centred on the node
    Done,     // periapsis at or above the ask: the orbit is the deliverable
    Failed    // gave up; why() says how
};

/* A snapshot of what the ship knows about itself and its sky, all in the
   non-rotating frame of the body it is launching from. */
struct GuidanceState {
    glm::dvec3 pos = glm::dvec3(0.0);   // m from the body centre
    glm::dvec3 vel = glm::dvec3(0.0);   // m/s, ORBITAL (conic) velocity
    double mu = 0.0;                    // m^3/s^2
    double sea_r = 0.0;                 // the body's sea-level radius, m
    glm::dvec3 spin_axis = glm::dvec3(0.0);  // unit pole (Frame::spinAxisRelTo). A
                                        // body that does not still spin has a pole
                                        // all the same, and it is what "east" and
                                        // the asked inclination are measured from.
    double accel_max = 0.0;             // availableAccel(), m/s^2 at full throttle
    bool has_propellant = true;         // THIS ship can still burn: the driver
                                        // fills it from enginePropellantMask() +
                                        // fuelMassMasked(), i.e. the resources its
                                        // thrusters actually draw. The driver MUST
                                        // fill it: availableAccel() is a potential
                                        // that never reads the tanks (#607), so on
                                        // its own it reads healthy at zero fuel.
    bool grounded = false;
    bool railed = false;                // Bullet is not stepping this hull
};

struct AscentCommand {
    glm::dvec3 aim = glm::dvec3(0.0);   // unit, same frame as GuidanceState.
                                        // Zero = "no direction to point at",
                                        // which is an answer, not a failure --
                                        // and it cancels the aim only, never the
                                        // throttle below.
    double throttle = 0.0;              // 0..1 of rated thrust, from the speed ceiling
    bool burn = false;                  // hold the engine lit this tick? The
                                        // driver MUST honour this one: the lever
                                        // latches, so a stale one burns alone.
};

class AscentLaw {
public:
    AscentLaw(const AscentTarget &t) : target(t) {
        /* The two apsides are a floor and a ceiling. Swapped, the climb ends at
           once (apoapsis is past the ask before it starts) and the node burn has
           no periapsis it can catch, so the law coasts for ever looking for a
           verdict that cannot arrive. The Guidance window refuses the pair at
           the field; this is the contract for every other caller. */
        assert(t.peri_asl <= t.apo_asl);
    }

    /* One call per logic tick. The law takes no dt: nothing it computes scales
       with the tick length -- the node-burn lead is sized from the burn's own
       length, not from the clock. What does depend on the tick rate is the
       trajectory the hull integrates between calls, because the middle of the
       profile is an unguided gravity turn. Measured on Moho against a 100 km
       ask: apoapsis 103.2 km at a 1/120 harness step, 103.5 km at 1/60, and
       101.3 km at the 1/6 s step a 10x-warp flight gives it. So shape_band is
       mostly a finite-burn allowance rather than a tick-rate one -- pinned by
       tests/test_guidance.cpp, not assumed. Widening it is #611's question. */
    AscentCommand step(const GuidanceState &s);

    AscentPhase phase() const { return phase_; }
    bool active() const { return phase_ != AscentPhase::Done && phase_ != AscentPhase::Failed; }
    const char *phaseName() const;
    const char *why() const { return why_; }
    OrbitElements elements() const { return last_el_; }   // the tick's conic
    /* Is elements() this tick's conic? step() bails before solving one when the
       hull is railed or the body has no mu, and last_el_ would otherwise be a
       default-constructed conic that a log would print as real numbers. */
    bool conicOk() const { return conic_ok_; }
    glm::dvec3 launchHeading() const { return launch_h_; }  // the heading the turn follows
    double node_r() const { return circ_node_r_; }        // the node the Circ burn was sized against
    double pitch_deg() const { return pitch_; }           // the pitch schedule, degrees

private:
    AscentTarget target;
    AscentPhase phase_ = AscentPhase::Pad;
    OrbitElements last_el_ = {};
    bool conic_ok_ = false;
    glm::dvec3 launch_h_ = glm::dvec3(0.0);  // the horizontal the turn aims along
    double circ_node_r_ = -1.0;
    double pitch_ = 90.0;
    const char *why_ = "";

    /* The horizontal launch direction that puts the conic at the asked
       inclination, worked out once from the pad's own geometry: sin(az) =
       cos(i)/cos(lat), the textbook relation, derived here from the two
       in-plane basis vectors rather than from a latitude we do not have. */
    void setLaunchHeading(const GuidanceState &s);
};

/* The driver. One call per logic tick, AFTER the key input (so the law sees
   this tick's throttle keys) and BEFORE the physics substeps (so the commands
   are armed for them). Commands go through Vehicle::Command only:
     Sas/SlewFree  -- a LATCH, and Command's Sas case TOGGLES, so it is sent
                      only while the mode is not already latched, and flyAscent
                      releases it on the first tick it decides not to fly. Leave
                      it latched and the ship keeps steering after the law is
                      gone, the keys that used to be SAS now fight the autopilot,
                      and a latched slew keeps a hull off rails forever.
     Aim           -- a per-tick SETPOINT: restate it every tick.
     Thrust        -- arms thrust for THIS tick only (tick.cpp clears it every
                      tick), so it goes out only when the law says burn.
     ThrottleSet   -- sets the LEVER, which latches separately from armed thrust.
                      The driver zeroes it on the ticks the law says coast: a
                      stale full lever burns through its own coast phase.
   All throttle writes are skipped while the player holds a throttle key (#400
   F3's yield rule).
   The caller must keep calling it on the hull the law was armed on -- the law
   has no idea which ship it is flying, so feed it a different one and it will
   cheerfully launch that one instead (Game::ascent_ship is the gate).
   One law per hull, and one hull per law: ascent, descent and transfer all
   steer the same two channels -- the aim setpoint and the throttle lever -- so
   a second mission armed on the same hull would fight the first for both. With
   only this law existing there is nothing to arbitrate yet, so the Guidance
   window enforces the rule by refusing a second Go (#612); the arbiter lands
   with the second law.
   Returns false when there was no aim to send -- a railed hull, a bail-out tick,
   or the tick the law finished on. A zero aim cancels the aim only: the throttle
   write still goes out, because the lever latches and a law that stops mid-burn
   without putting it back has left an engine open. */
class Vehicle;
bool flyAscent(Vehicle &v, AscentLaw &law, double dt);

/* Let go of a hull, once: releases the SlewFree latch and closes the throttle
   lever (unless a throttle key is down -- #400 F3's yield rule, and the player's
   key wins over a Cancel too). A Cancel can land in the middle of a burn, where
   no verdict tick is coming to write the lever back, so the caller cannot rely
   on flyAscent's going-away pass, which releases the latch only. */
void releaseAscent(Vehicle &v);

/* What v1 does NOT do, stated here so the next phase is a decision and not a
   discovery: no atmosphere (drag/pressure are not even in GuidanceState), no
   descent or landing, no transfers, no staging (it will burn a single stage
   until the stage's thrust reads zero and then Fail), no plane change after
   launch (the inclination is set by the launch heading, not steered), and no
   mid-course re-burn -- if the circularisation cutoff is missed, the orbit is
   what it is and the law reports Done/Failed rather than trying again. */
#endif
