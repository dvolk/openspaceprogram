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
#include <cmath>     // std::isfinite, on the air facts the ctor guards
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

    /* "Did the player touch these fields at all?" is then one comparison: the save
       writes the ask only when it differs from the default (#612 stage 2), and a
       test can ask the same question without spelling four fields. */
    bool operator==(const AscentAsk &) const = default;

    AscentTarget target() const {
        AscentTarget t;
        t.peri_asl = peri_asl;
        t.apo_asl = apo_asl;
        t.inc_rad = ask_inc ? glm::radians(inc_deg) : -1.0;   // the law's "not asked"
        return t;
    }
};

/* The air a mission starts inside, measured ONCE from the hull it is armed on and
   held by the law for the whole flight (#961). Both numbers count UP from the launch
   point (#962): "how much air is above this hull", not "how much air does this body
   have". A pad under the whole sky therefore gets the body's own figures, and a hull
   that begins inside its own sky -- Venus's cloud deck sits at 56 km of a 116 km air,
   kerbin_dock sits above all 85 km of Kerbin's -- gets only what is left above it.
   Crediting the whole column to a hull at 56 km scheduled a 40 km turn around a ship
   that had 60 km to climb and 5300 kg/m^2, not 1.04e6, to push back.

   These are not per-tick readings, which is why they are not in GuidanceState: the
   body a launch is armed on does not change mid-flight, and a state struct whose
   neighbours all move makes these look like live readings. It also removes the trap
   the pair used to be -- both defaulted to 0 in a struct the driver had to fill, and
   0 means "fly the airless schedule", so a caller that filled one and not the other
   flew a plausible mission with a quietly different profile. Here the arming door
   answers and the compiler asks. */
struct AirFacts {
    double overhead = 0.0;   // [m] of air above the launch point, up to the air top
    double column = 0.0;     // [kg/m^2] of THAT air: the weight that can push back
};

/* The arithmetic half, on its own so a test can weigh a deck without a hull, a body
   or a frame. `launch_alt_asl` is metres above the body's datum (negative = under it,
   which buys the same overhead as the datum: the profile is flat below it, #245), and
   `pressure_at_launch` is the profile's own [Pa] there -- dividing by the datum's g is
   the same hydrostatic reading the loader and dragAtm() weigh air with (#697), so the
   law cannot size its turn from a different figure than the one that bills a hull.
   Guards fail CLOSED on the !(x > 0.0) shape: a NaN top or pressure answers "no air"
   rather than walking into the schedule (src/angmath.h). */
inline AirFacts airOverhead(double air_top_asl, double launch_alt_asl,
                            double pressure_at_launch, double datum_g) {
    AirFacts a;
    if(!(air_top_asl > 0.0) || !(datum_g > 0.0)) { return a; }
    const double alt = launch_alt_asl > 0.0 ? launch_alt_asl : 0.0;
    if(air_top_asl <= alt) { return a; }   // born at or above the sky: nothing overhead
    a.overhead = air_top_asl - alt;
    a.column = pressure_at_launch > 0.0 ? pressure_at_launch / datum_g : 0.0;
    return a;
}

enum class AscentPhase {
    Pad,      // still on the ground (or just cleared it): vertical
    Ascent,   // burning, commanding a path angle, until Ap reaches the ask
    Coast,    // Ap made: falling toward the node with the engine off
    Circ,     // the circularisation burn, centred on the node
    Done,     // periapsis at or above the ask: the orbit is the deliverable
    Failed    // gave up; why() says how
};

/* A snapshot of what the ship knows about itself, all in the non-rotating frame of
   the body it is launching from. Everything here MOVES; the sky it launches into is
   arm-time furniture and lives in AscentLaw as AirFacts (#961). */
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
    bool has_propellant = true;         // THIS stage can actually burn: the driver
                                        // fills it from Vehicle::canBurn(), which asks
                                        // per ignited engine whether every resource it
                                        // draws is available to it (#607). The driver
                                        // MUST fill it: availableAccel() is a potential
                                        // that never reads the tanks, so on its own it
                                        // reads healthy at zero fuel.
    /* The staging plan, as two facts the hull owns and the law only reads (#928).
       stages_left is how many presses are still reachable (activeStage - minStage);
       stage_due is Vehicle::stageIsDue(), which asks the VAB's own question on live
       tanks -- has this stage taken everything the parts it would drop can give, or
       (having nothing drinkable in them) everything it can reach. The law does not
       get fuel groups, drain layers or a part list: those are hull structure, and
       the same question is asked by staging.cpp when it draws the table the VAB
       shows, so one owner answers it twice. */
    int stages_left = 0;
    bool stage_due = false;
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
    bool want_stage = false;            // press the stage key THIS tick (#928). This
                                        // is a REQUEST: the law cannot separate a
                                        // hull, because a press restructures the
                                        // vehicle list rather than one hull's state,
                                        // so it goes through Game::stageVehicle and
                                        // the caller acts on it. One ask per value of
                                        // GuidanceState::stages_left -- see
                                        // stage_asked_at_.
};

/* Which way the caller said no to a press the law asked for (#941), or None until
   it has. The names are the two Game::StageOutcome values that are refusals;
   guidance.cpp stays free of Game, so the caller maps one onto the other. */
enum class PressRefusal { None, Crew, Root };

class AscentLaw {
public:
    /* The ask, and the air the hull starts in. Neither is optional (#961): a law
       built without the sky silently flies the airless schedule, which is the exact
       mistake the pair used to make from inside GuidanceState. airAboveHull() reads
       the second argument off a hull at arm time; a caller with no air to speak of
       says so with AirFacts{}. */
    AscentLaw(const AscentTarget &t, const AirFacts &air) : target(t), air_(air) {
        /* The two apsides are a floor and a ceiling. Swapped, the climb ends at
           once (apoapsis is past the ask before it starts) and the node burn has
           no periapsis it can catch, so the law coasts for ever looking for a
           verdict that cannot arrive. The Guidance window refuses the pair at
           the field; this is the contract for every other caller. */
        assert(t.peri_asl <= t.apo_asl);
        /* 0 means no air overhead; a negative or NaN one is not a thing. NaN is
           worth naming here because min/max SWALLOW it: min(1.0, NaN) answers 1.0,
           so a NaN column would buy the FULL Kerbin bend -- the worst of the silent
           answers (#960 review, now checked once at arm time rather than a tick
           later, because the number cannot move after the law exists). */
        assert(std::isfinite(air_.overhead) && air_.overhead >= 0.0);
        assert(std::isfinite(air_.column) && air_.column >= 0.0);
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

    /* The caller refused the press the law asked for (#941). The law's only staging
       feedback was the stages_left counter, which a refusal leaves exactly where it
       was, so its once-per-value latch made a refusal permanent: the mission burned
       that stage dry and reported "the tanks are dry" with stages still folded under
       it, even after the player put the kerbal on EVA. So the caller reports the
       answer back here.
         Crew -- fixable mid-flight (EVA the kerbal in the drop set), so the latch is
                 released and the re-ask is throttled: an unfixed refusal is one line
                 a second, not one a tick, and a fixed one gets its stage.
         Root -- not fixable mid-flight (which seam is the ship's root is decided in
                 the VAB), so it is remembered for the verdict and never re-asked.
       NothingSeparated is NOT a refusal and does not call this: it steps the counter,
       which remains the law's ordinary feedback. Either way, remembering is only
       about telling the truth at the end -- a stage that runs dry while the press is
       unanswered still ends the mission, because a law that waits on a player for an
       unbounded time flies missions that can never be scored. */
    void pressRefused(PressRefusal kind);

    AscentPhase phase() const { return phase_; }
    bool active() const { return phase_ != AscentPhase::Done && phase_ != AscentPhase::Failed; }
    const char *phaseName() const;
    const char *why() const { return why_; }
    /* The air this mission starts inside, as the law holds it (#961). The Guidance
       window is where a player would want to read it -- "85 km of air weighing 10328
       kg/m^2 above this pad, so the turn is scheduled at 34 km" -- and printing from
       here means the readout and the schedule cannot quote different skies. */
    AirFacts airFacts() const { return air_; }
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
    /* The air overhead, settled when the mission was armed (#961/#962). Nothing in
       the flight can change it -- the law is thrown away and rebuilt to fly
       somewhere else -- so it is not a thing step() re-reads. */
    AirFacts air_;
    AscentPhase phase_ = AscentPhase::Pad;
    OrbitElements last_el_ = {};
    bool conic_ok_ = false;
    glm::dvec3 launch_h_ = glm::dvec3(0.0);  // the horizontal the turn aims along
    double circ_node_r_ = -1.0;
    double pitch_ = 90.0;
    const char *why_ = "";
    /* Highest altitude the pitch schedule has seen (m above the datum), -1 until
       the first aim tick. pathAngle() is a schedule over a coordinate, so a hull
       that loses height would be told to point further up than it did at the pad
       (#930); the schedule reads this ratchet instead of the live altitude. */
    double apex_alt_ = -1.0;

    /* The stages_left value this law last asked for a press at (#928), -1 until it
       asks. One ask per value is the retry policy for everything the caller does NOT
       report: a press that separated nothing still stepped the counter, so the law
       can ask again later for the new number. A refusal used to be invisible here,
       which made it permanent -- see pressRefused(), which is #941's answer back. */
    int stage_asked_at_ = -1;

    /* Logic ticks to wait before the next ask is allowed after a reported refusal
       (#941). A tick count, not a clock: step() deliberately takes no dt. */
    int stage_retry_in_ = 0;

    /* The last refusal the caller reported (#941). Held so the dry verdict can say
       why the stack is still in one piece: "the tanks are dry" is true but useless
       when the reason nothing separated is a press someone said no to. Cleared when
       stages_left moves, because a press HAS happened by then and an old refusal is
       no longer the reason. */
    PressRefusal press_refused_ = PressRefusal::None;
    int last_stages_seen_ = -1;

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
   Staging is the one thing the law asks that is NOT a Command (#928): a press
   restructures the world -- extractSubtreeAsShip, a new Vehicle in Game::vehicles,
   chutes deploying, a name deduped against the roster -- so it belongs to Game, not
   to a hull. The driver reports the law's request through *want_stage and the caller
   acts on it, which keeps guidance.cpp free of Game and keeps the law pure. The
   caller presses BEFORE the physics substeps, so the newly ignited engines light on
   the same tick the law asked.
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
/* Read the air above THIS hull, once, at the moment a mission is armed (#961's
   fill, #962's measurement). It asks the body the same questions the aero pass asks
   -- physical() for "is there air here", dragAtm()'s profile for the pressure at the
   hull, airGravity() for what that air weighs -- so the schedule cannot be sized from
   a different air than the one that will bill the flight. Answers AirFacts{} for a
   hull with no body, no air, or no air left above it (kerbin_dock at 300 km is over
   all of Kerbin's sky, and a mission armed there gets the airless bend rather than
   Kerbin's 34 km one). */
AirFacts airAboveHull(const Vehicle &v);
/* want_stage is NOT optional: the law spends its once-per-stage press latch the
   tick it asks (#928), so a caller that cannot act on the ask strands that stage
   for the rest of the mission and never hears about it. */
bool flyAscent(Vehicle &v, AscentLaw &law, double dt, bool *want_stage);

/* Let go of a hull, once: releases the SlewFree latch and closes the throttle
   lever (unless a throttle key is down -- #400 F3's yield rule, and the player's
   key wins over a Cancel too). A Cancel can land in the middle of a burn, where
   no verdict tick is coming to write the lever back, so the caller cannot rely
   on flyAscent's going-away pass, which releases the latch only. */
void releaseAscent(Vehicle &v);

/* What v1 does NOT do, stated here so the next phase is a decision and not a
   discovery: the law still bills no drag and reads no pressure. GuidanceState
   carries the air's HEIGHT (#960), because that is all the pitch schedule needs
   to know, and nothing else about the air: no q, no flight-path closure, no
   throttle response to drag. No descent or landing -- a hull that comes back to
   the ground FAILS with that reason rather than being flown again (#931), no
   transfers, no plane change after launch (the inclination is set by the launch
   heading, not steered), and no mid-course re-burn -- if the circularisation
   cutoff is missed, the orbit is what it is and the law reports Done/Failed
   rather than trying again.
   Staging landed with #928, as an ASK only: the law presses when the hull says
   the stage is due (Vehicle::stageIsDue, the VAB's own criterion) or has run dry
   with presses left. It does not resequence the plan, choose a drop for
   performance, or know what the press did -- see stage_asked_at_ for the retry
   rule, and Game::StageOutcome for what the caller learns instead. */
#endif
