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

#include <glm/glm.hpp>
#include "orbit.h"

/* What the mission wants. Altitudes are metres ABOVE SEA LEVEL (the body's
   datum), because that is what periapsisAsl()/apoapsisAsl() return and what a
   player would read off the navball -- not radii (#404). */
struct AscentTarget {
    double apo_asl = 100e3;
    double peri_asl = 100e3;
    double inc_rad = -1.0;   // <0: keep whatever the launch geometry gives us
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
    glm::dvec3 spin_axis = glm::dvec3(0.0);  // unit; zero if the body does not spin
    double accel_max = 0.0;             // availableAccel(), m/s^2 at full throttle
    bool has_propellant = true;         // anything left to burn. The driver MUST
                                        // fill this: availableAccel() is a potential
                                        // that never reads the tanks (#607), so on
                                        // its own it reads healthy at zero fuel.
    bool grounded = false;
    bool railed = false;                // Bullet is not stepping this hull
};

struct AscentCommand {
    glm::dvec3 aim = glm::dvec3(0.0);   // unit, same frame as GuidanceState.
                                        // Zero = "no direction to point at",
                                        // which is an answer, not a failure.
    double throttle = 0.0;              // 0..1 of rated thrust, from the speed ceiling
    bool burn = false;                  // hold the engine lit this tick? The
                                        // driver MUST honour this one: the lever
                                        // latches, so a stale one burns alone.
};

class AscentLaw {
public:
    AscentLaw(const AscentTarget &t) : target(t) {}

    /* One logic tick. dt is the sim interval this tick advances; the law uses
       it only to size the coast-to-cutoff floor, so a law ticked at 1x or at
       10x time compression aims at the same orbit. */
    AscentCommand step(const GuidanceState &s, double dt);

    AscentPhase phase() const { return phase_; }
    bool active() const { return phase_ != AscentPhase::Done && phase_ != AscentPhase::Failed; }
    const char *phaseName() const;
    const char *why() const { return why_; }
    OrbitElements elements() const { return last_el_; }   // the tick's conic
    glm::dvec3 launchHeading() const { return launch_h_; }  // the heading the turn follows
    double node_r() const { return circ_node_r_; }        // the node the Circ burn was sized against
    double pitch_deg() const { return pitch_; }           // the pitch schedule, degrees

private:
    AscentTarget target;
    AscentPhase phase_ = AscentPhase::Pad;
    OrbitElements last_el_ = {};
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
   are armed for them). Commands go through Vehicle::Command only: a latched
   Sas/SlewFree once, ShipCmd(Aim, ...) every tick, ShipCmd(Thrust) every tick
   (armed thrust is cleared each tick), and ShipCmd(ThrottleSet) every tick
   EXCEPT while the player is holding a throttle key (#400 F3's yield rule).
   Returns false if it did nothing (a railed hull, a ship that is not flying a
   law), which is how callers know not to trust the trace. */
class Vehicle;
bool flyAscent(Vehicle &v, AscentLaw &law, double dt);

/* What v1 does NOT do, stated here so the next phase is a decision and not a
   discovery: no atmosphere (drag/pressure are not even in GuidanceState), no
   descent or landing, no transfers, no staging (it will burn a single stage
   until the stage's thrust reads zero and then Fail), no plane change after
   launch (the inclination is set by the launch heading, not steered), and no
   mid-course re-burn -- if the circularisation cutoff is missed, the orbit is
   what it is and the law reports Done/Failed rather than trying again. */
#endif
