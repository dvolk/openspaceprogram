#pragma once

// eva.h -- the EVA kerbal: a one-part Vehicle subclass + its control laws.
// Pure geometry lives in evamath.h. Grounded = Bullet contact OR analytic
// terrain height (meshes only exist at max-LOD leaves, so the analytic side
// is also the fall-through guard).

#include "keys.h"                // Slot, SlotProbe (the arm signature)
#include "vehicle.h"             // Vehicle
#include "evamath.h"             // the pure control-law geometry

struct Game;

enum EvaMode { EVA_GROUND, EVA_SPACE };

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
       `k->jumpPressed = true` in events.cpp. armInput() is the only PER-TICK writer
       and applyEva() the reader; settleContact() is the loader's writer of
       mode/grounded (a load starts paused, so it cannot wait for the first tick), and
       the doors below are what anyone else uses.
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

public:
    /* Arm this kerbal's controls for one tick: keys + camera basis + ground state.
       tick.cpp calls it once a tick for the active ship, and only where a pilot could
       be holding keys -- a non-piloting scene or a paused sim arms nothing. `active`
       is the tick's SlotProbe (keys.h), handed by reference rather than as a
       std::function so arming the suit allocates nothing (#581). */
    void armInput(Game &g, const SlotProbe &active);

    /* The jump key went down (events.cpp). The rule lives on jumpPressed above. */
    void pressJump() { jumpPressed = true; }

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

    // Capsule-center altitude above the analytic surface when standing at
    // rest: half the suit plus the two collision margins that meet there.
    double restAlt() const {
        return parts[0]->def->height / 2.0 + hullMargin() + terrainMargin();
    }

    /* mode/grounded are per-tick state, so a rebuilt kerbal wears the
       constructor's "standing at the KSC" default until its first tick. A load
       starts paused, so nothing ticks: derive the regime from the loaded pose
       instead of letting the first read (or --eva-log's first row) claim an
       orbital kerbal is standing. */
    void settleContact();

    // Per-substep EVA law: ground walking + jump, or RCS + camera-facing
    // attitude; analytic fall-through guard in both modes.
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
};
