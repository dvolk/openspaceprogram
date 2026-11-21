#pragma once

// eva.h -- the EVA kerbal: a one-part Vehicle subclass + its control laws.
// Pure geometry lives in evamath.h. Grounded = Bullet contact OR analytic
// terrain height (meshes only exist at max-LOD leaves, so the analytic side
// is also the fall-through guard).

#include "keys.h"                // Slot, SlotProbe (the arm signature)
#include "vehicle.h"             // Vehicle
#include "evamath.h"             // the pure control-law geometry

struct Game;

enum EvaMode { EVA_GROUND, EVA_SPACE, EVA_LADDER };

struct Kerbal : Vehicle {
    /* Frictionless feet: foot friction pairs with the COM walk force (see
       applyEva) into a tipping couple the attitude authority cannot
       out-torque. Vehicle::hullFriction carries this through every build
       path, save/load rebuild included. */
    Kerbal() { hullFriction = 0.0; }

    /* Identity is OWNED by the suit Part (part.h: Part::kerbal), not by this
       vehicle -- the walker is just the person's current location. */
    const KerbalProfile *profile() const {
        return parts.empty() ? nullptr : parts[0]->kerbal.get();
    }
    uint64_t profileId() const {
        const KerbalProfile *p = profile();
        return p != nullptr ? p->id : 0;
    }

    /* THE CONTROL STATE (#578) -- the suit's half of what #575 made structural for
       ships. "The control law arms the state once per tick and the laws run it per
       substep" used to be only a comment: a TU-level free function (evaArmCommands)
       assigned these fields from outside the class, and the jump key was a bare
       `k->jumpPressed = true` in events.cpp. armInput() and disarm() are the PER-TICK
       writers -- a live, off-rails kerbal gets one or the other every tick, #215 --
       applyEva() the reader, and settleContact() is the loader's writer of
       mode/grounded (a load starts paused, so it cannot wait for the first tick), and
       the doors below are what anyone else uses. settleContact()'s rule is the weaker
       one -- pose only, no Bullet contact, because a hull freshly added to the world has
       no manifolds yet -- and after the first tick that steps physics it is overwritten
       by updateContact(); it only earns its keep on a loaded hull that stays on rails.
         mode / grounded     which regime the laws run in this tick
         jumping             post-jump: ignore grounded until the outward kick is spent
         jumpPressed         the jump key's down EDGE: pressJump() raises it and
                             armInput() spends it in EITHER mode, so a press cannot
                             outlive the tick it arms -- a paused sim arms nothing, so
                             like every other held key it waits for the sim. It used to
                             be read only on the ground branch, and SPACE is the staging
                             key everywhere else in the game: a spacewalker who tapped
                             it out of ship habit collected an unasked 2.5 m/s hop on
                             landing.
         jumpRequested       armed at arm; the first substep fires it (applyEva spends it)
         walkDir             tangent walk heading, unit or 0
         viewYaw             QE yaw about the view axis, rad (accumulated)
         camBasis            [right, up, fwd] snapshot taken at arm
       The RCS translation demand is the inherited Vehicle::rcsDir, armed through
       Vehicle::armRcs() -- protected, because arming is a member of this class now. */
protected:
    EvaMode mode = EVA_GROUND;
    bool grounded = true;
    bool jumping = false;
    bool jumpPressed = false;
    bool jumpRequested = false;
    glm::dvec3 walkDir = glm::dvec3(0.0);
    double viewYaw = 0.0;                  // rad, accumulated
    glm::dmat3 camBasis = glm::dmat3(1.0); // [right, up, fwd] snapshot at arm
    /* THE LADDER GRIP (#1073). ladderUid is the durable half -- the grabbed
       part's uid, saved with the kerbal; 0 = not on a ladder. ladderPart is
       the per-tick resolved cache (the uid looked up among live nearby
       vehicles), valid only while mode == EVA_LADDER; applyLadder reads it
       per substep. Everything else about the hang -- where along the segment,
       which way out -- is re-derived from the pose, never remembered.
         grabPressed       the grab key's down EDGE: pressGrab() raises it,
                           armInput() spends it (grab nearest / let go)
         climbDir          -1..1 along the segment toward its top end, armed
                           per tick (W/S); a disarmed suit hangs at 0
         ladderS           the hold point's target arc length along the
                           segment: armInput advances it by the climb demand,
                           the servo flies the suit to it. Re-projected onto
                           the segment whenever the grip changes (grab,
                           transfer, load), so a hang never jumps. */
    uint64_t ladderUid = 0;
    Part *ladderPart = nullptr;
    bool grabPressed = false;
    double climbDir = 0.0;
    double ladderS = 0.0;

public:
    /* Arm this kerbal's controls for one tick: keys + camera basis + ground state.
       tick.cpp calls it once a tick for the active ship, and only where a pilot could
       be holding keys -- a non-piloting scene or a paused sim arms nothing. `active`
       is the tick's SlotProbe (keys.h), handed by reference rather than as a
       std::function so arming the suit allocates nothing (#581). */
    void armInput(Game &g, const SlotProbe &active);

    /* Neutralize one tick for a kerbal that armInput() did NOT run on (#215). The
       laws below run per substep for every live hull in the fleet, so a suit whose arm
       stopped running -- the player switched to another ship, went to free cam, or is
       in a scene that does not pilot -- otherwise keeps the last law it wore: a stale
       walkDir drives it at kWalkSpeed, a stale rcsDir burns hydrazine until the tank is
       dry, and a suit that was never armed at all wears the constructor's "standing"
       default and brakes an orbital kerbal at kWalkAccel. No demands, and the regime
       re-derived from the pose rather than remembered (remembering it IS the bug).
       A ladder grip SURVIVES the disarm (climbDir goes to 0): a kerbal left on a
       ladder hangs there while the player flies another ship -- the grip is state,
       not a demand, and validating it needs the fleet.
       tick.cpp calls this every tick for every live, off-rails kerbal when the tick
       steps physics, before the arm; game.cpp's proximity pass calls it on a suit it
       wakes later in the same tick. */
    void disarm(const std::vector<Vehicle *> &vehicles);

    /* The jump key went down (events.cpp). The rule lives on jumpPressed above. */
    void pressJump() { jumpPressed = true; }

    /* The grab key went down (events.cpp): armInput spends it -- let go when
       gripping, else grab the nearest ladder in reach (#1073). */
    void pressGrab() { grabPressed = true; }

    /* The grip, for the save and the log. The uid is the whole durable state;
       the loader restores it and the first resolution finds the part again
       (a gone part -- staged away between save and load -- clears it). */
    uint64_t ladderGrip() const { return ladderUid; }
    void restoreLadder(uint64_t uid) { ladderUid = uid; }

    /* The hatch hold pose on a capsule's ladder, in the capsule vehicle's
       frame: the top end (the hatch) plus the hang offset, body along the
       slope facing the hull. Game::kerbalEVA spawns the walker here (#1073).
       False if the def carries no ladder (the caller's fallback). */
    static bool hatchPose(const Part *cap, glm::dvec3 &pos, glm::dmat3 &rot);

    /* What the regime is, for the --eva-log row (#421's e2e assertions read it). */
    EvaMode evaMode() const { return mode; }
    bool evaGrounded() const { return grounded; }

    /* The QE yaw offset, for the save and the log. It is the difference between the
       pose the save holds and the camera-facing attitude the space law would slew back
       to without it (#241), so it has to round-trip. */
    double viewYawOffset() const { return viewYaw; }
    /* The loader's door (save.cpp). No range check: the offset is an accumulator that
       never wraps by design, and the JSON reader already refuses a non-number, so a
       value reaching here is an angle however large. */
    void restoreViewYaw(double yaw) { viewYaw = yaw; }

    // Already world axes. The suit's RCS demand (the inherited Vehicle::rcsDir,
    // armed by armInput above) is in the RENDER frame, not ship axes: EVA thrust
    // follows the view, which the suit's own rotation lags during a slew. Hence
    // this override is the identity.
    glm::dvec3 rcsWorldDir() const override { return rcsDir; }

    bool isEva() const override { return true; }

    /* The person's name lives on the roster profile the suit owns; this walker
       keeps no copy of it. Vehicle::name stays empty, so a stray rename cannot
       leave the walker and its seat disagreeing. */
    const std::string &displayName() const override {
        const KerbalProfile *p = profile();
        return p != nullptr ? p->name : Vehicle::name;
    }

    /* A Kerbal vehicle exists ONLY while the person is walking. Seated, there is
       no object here at all: the suit Part sits in the capsule as a seat
       (part.h), and minting this class IS going EVA (Game::kerbalEVA), while
       deleting one IS boarding (Game::kerbalBoard). So a Kerbal is never
       contained and never railed for being aboard. */

    // Capsule-center altitude above the surface when standing at rest: half
    // the suit plus the two collision margins that meet there. The surface's
    // margin is the terrain's on the ground, the deck's own in a platform
    // bubble (eva.cpp, #850).
    double restAlt() const;

    /* The ground under the suit (#850): in an air platform's bubble it is the
       DECK -- local +Z up, deck top at z=0 by the authoring convention -- not
       the planet far below. In a station bubble there is no down: weightless,
       EVA_SPACE. Elsewhere the body's radial + analytic terrain, as always. */
    bool onDeck() const;
    bool inStationBubble() const;
    bool overDeck() const;   // a ray down hits the deck (not off the edge)
    glm::dvec3 groundUp() const;
    double groundAltAt(const glm::dvec3 &pos) const;

    /* mode/grounded are per-tick state, so a rebuilt kerbal wears the
       constructor's "standing at the KSC" default until its first tick. A load
       starts paused, so nothing ticks: derive the regime from the loaded pose
       instead of letting the first read (or --eva-log's first row) claim an
       orbital kerbal is standing. A restored ladder grip resolves here too
       (the fleet is built by the time the load commit calls this). */
    void settleContact(const std::vector<Vehicle *> &vehicles);

    // Per-substep EVA law: ground walking + jump, RCS + camera-facing
    // attitude, or the ladder hang; analytic fall-through guard in the first
    // two (a hanging kerbal must never terrain-snap).
    void applyEva(double h);

    void applyControlForces(double h) override {
        /* The overlay's per-substep clear is owned by the tick loop. The
           suit's jets bypass the recording appliers entirely: #435. */
        applyEva(h);
    }

private:
    // Authority-bounded PD slew toward `target` (braking-curve rate, torque
    // capped).
    void slewTo(const glm::dmat3 &target, double h, double authority);

    /* mode/grounded from the pose and Bullet contact, keys aside: the half of
       armInput() that disarm() needs too (#215). */
    void updateContact();

    /* The ladder grip (#1073), resolved per tick wherever the regime is
       derived. resolveLadder validates the uid among live nearby vehicles,
       caches the part, and handles the two generous transitions: climbing
       off an END onto another segment within reach, and climbing down to the
       ground at the bottom end. True = still hanging (mode EVA_LADDER). */
    bool resolveLadder(const std::vector<Vehicle *> &vehicles);
    /* Nearest ladder part whose segment lies within `reach` of the suit,
       among live vehicles; excludeUid skips the current grip (transfer). */
    Part *nearestLadder(const std::vector<Vehicle *> &vehicles, double reach,
                        uint64_t excludeUid);
    /* The grabbed segment in the KERBAL's frame (partPoseRelTo), off the
       cached part. */
    void ladderSegment(glm::dvec3 &A, glm::dvec3 &B, glm::dvec3 &out) const;
    /* Per-substep hang: along-axis climb servo + perpendicular position PD
       toward the hold point, attitude along the ladder facing the hull. */
    void applyLadder(double h);
};
