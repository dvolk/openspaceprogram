#pragma once

// eva.h -- the EVA kerbal: a one-part Vehicle subclass + its control laws.
// Pure geometry lives in evamath.h. Grounded = Bullet contact OR analytic
// terrain height (meshes only exist at max-LOD leaves, so the analytic side
// is also the fall-through guard).

#include <functional>

#include "keys.h"                // Slot (the arm signature)
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

    // --- per-ticked armed state (evaArmCommands writes, applyEva consumes) ---
    EvaMode mode = EVA_GROUND;
    /* Identity is OWNED by the suit Part (part.h: Part::kerbal), not by this
       vehicle -- the walker is just the person's current location. */
    const KerbalProfile *profile() const {
        return parts.empty() ? nullptr : parts[0]->kerbal.get();
    }
    uint64_t profileId() const {
        const KerbalProfile *p = profile();
        return p != nullptr ? p->id : 0;
    }
    bool grounded = true;
    bool jumping = false;        // post-jump: ignore grounded until the outward kick is spent
    bool jumpPressed = false;    // space KEYDOWN edge (events.cpp)
    bool jumpRequested = false;  // armed this tick; the first substep fires it
    glm::dvec3 walkDir = glm::dvec3(0.0);  // tangent walk heading, unit or 0
    /* The RCS translation dir (inherited Vehicle::rcsDir) is armed in the
       RENDER frame, not ship axes: EVA thrust follows the view, which the
       suit's own rotation lags during a slew. Hence rcsWorldDir() below is
       the identity. */
    double viewYaw = 0.0;        // QE yaw about the view axis, rad (accumulated)
    glm::dmat3 camBasis = glm::dmat3(1.0); // [right, up, fwd] snapshot at arm

    // Already world axes.
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
        // The base clears lastForceVecs per substep; keep that contract
        // even though the suit's law applies its forces through other
        // doors (which is why the overlay shows nothing for EVA today).
        lastForceVecs.clear();
        applyEva(h);
    }

private:
    // Authority-bounded PD slew toward `target` (braking-curve rate, torque
    // capped).
    void slewTo(const glm::dmat3 &target, double h, double authority);
};

// Arm the active kerbal's controls for this tick (keys + camera + ground state).
void evaArmCommands(Game &g, const std::function<bool(Slot)> &active);
