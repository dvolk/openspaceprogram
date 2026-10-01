#pragma once

// eva.h -- the EVA kerbal: a one-part Vehicle subclass + its control laws.
// Pure geometry lives in evamath.h. Grounded = Bullet contact OR analytic
// terrain height (meshes only at max-LOD leaves, so the analytic side is
// also the fall-through guard -- see applyEva).

#include <functional>

#include "keys.h"                // Slot (the arm signature)
#include "vehicle.h"             // Vehicle
#include "evamath.h"             // the pure control-law geometry

struct Game;

enum EvaMode { EVA_GROUND, EVA_SPACE };

struct Kerbal : Vehicle {
    // --- per-ticked armed state (evaArmCommands writes, applyEva consumes) ---
    EvaMode mode = EVA_GROUND;
    bool grounded = true;
    bool jumping = false;        // post-jump: ignore grounded until floor contact clears
    bool jumpPressed = false;    // space KEYDOWN edge (events.cpp)
    bool jumpRequested = false;  // armed this tick; the first substep fires it
    glm::dvec3 walkDir = glm::dvec3(0.0);  // tangent walk heading, unit or 0
    glm::dvec3 rcsDir = glm::dvec3(0.0);   // RCS translation dir, unit or 0
    double viewYaw = 0.0;        // QE yaw about the view axis, rad (accumulated)
    glm::dmat3 camBasis = glm::dmat3(1.0); // [right, up, fwd] snapshot at arm

    bool isEva() const override { return true; }
    bool isCrewAboard() const override { return isAboard(); }
    Part *capsulePart() const override { return aboardPart; }

    // Aboard a ship = parked in a capsule Part (body out of the physics world);
    // free = on EVA. `aboardPart` is the single source of truth: a Part* is
    // stable across a merge/split, so no reindex machinery is needed.
    Part *aboardPart = nullptr;  // the capsule Part it sits in; nullptr = free (on EVA)
    bool isAboard() const { return aboardPart != nullptr; }
    Vehicle *aboard() const {
        return (aboardPart != nullptr) ? aboardPart->owner : nullptr;
    }

    // Capsule-center altitude above the analytic surface when standing at rest.
    double restAlt() const {
        return parts[0]->def->height / 2.0 + 0.6;
    }

    // Per-substep EVA law: walking + jump + upright on the ground, RCS +
    // camera-facing attitude in free fall, analytic fall-through guard in both.
    void applyEva(double h);

    void applyControlForces(double h) override {
        applyEva(h);
    }

private:
    // Authority-bounded PD slew toward `target` (braking-curve rate, torque
    // capped). Ground gets more authority: it must win against foot friction.
    void slewTo(const glm::dmat3 &target, double h, double authority);
};

// Arm the active kerbal's controls for this tick (keys + camera + ground state).
void evaArmCommands(Game &g, const std::function<bool(Slot)> &active);
