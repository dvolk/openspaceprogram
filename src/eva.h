#pragma once

// eva.h -- the EVA kerbal: a one-part Vehicle subclass + its control laws.
//
//   Kerbal          the vehicle (inherits frames, gravity, rails, the HUD)
//   evaArmCommands  per-tick control arming (keys + camera -> armed state)
//
// A kerbal IS a one-part ship as far as the game is concerned (see the
// design notes in reports/eva2026_09_02/): it rides the fleet list, F6
// cycling, the rails, the SOI bookkeeping and the readouts unchanged.
// What's kerbal-specific is the control law: on a surface it walks
// (camera-relative WASD projected onto the tangent plane, space = jump),
// in free fall it flies RCS-style (camera-relative WASD translation, QE
// yaw about the view axis, attitude slewed to face the camera). The pure
// geometry the laws use lives in evamath.h (headless-testable).
//
// Grounded is two-layered: Bullet contact (works on the pad too) OR the
// analytic terrain height within the standing band. Terrain collision
// meshes exist only at max-LOD leaf patches (camera proximity), so the
// analytic height is also the fall-through guard -- see applyEva.

#include <functional>

#include "SDL2/SDL_scancode.h"   // SDL_Scancode (the arm signature)
#include "vehicle.h"             // Vehicle
#include "evamath.h"             // the pure control-law geometry

struct Game;

enum EvaMode { EVA_GROUND, EVA_SPACE };

struct Kerbal : Vehicle {
    // --- per-tick armed state (evaArmCommands writes it once per tick;
    //     applyEva consumes it before every substep) --------------------
    EvaMode mode = EVA_GROUND;
    bool grounded = true;
    bool jumping = false;        // post-jump: ignore grounded until the
                                 // contact-margin band is clear
    bool jumpPressed = false;    // space KEYDOWN edge (events.cpp); armed
                                 // on the next tick
    bool jumpRequested = false;  // armed this tick; the first substep fires it
    glm::dvec3 walkDir = glm::dvec3(0.0);  // tangent walk heading, unit or 0
    glm::dvec3 rcsDir = glm::dvec3(0.0);   // RCS translation dir, unit or 0
    double viewYaw = 0.0;        // QE yaw about the view axis, rad (accumulated)
    glm::dmat3 camBasis = glm::dmat3(1.0); // [right, up, fwd] snapshot at arm

    bool isEva() const override { return true; }

    /* The capsule-center altitude above the analytic surface when standing
       at rest: half the part height + the collision margins (0.5 terrain +
       0.1 hull -- the same 0.6 the pad placement lifts ships by). */
    double restAlt() const {
        return partDefs[0]->height / 2.0 + 0.6;
    }

    /* The per-substep EVA law (the applyControlForces override): walking
       steering + jump + upright torque on the ground, RCS translation +
       camera-facing attitude in free fall, and the analytic fall-through
       guard in both. */
    void applyEva(double h);

    void applyControlForces(double h) override {
        applyEva(h);
    }

private:
    /* Authority-bounded PD slew of the capsule toward `target` (the same
       law style as the ship's slewToward/killRotStep: drive the angular
       velocity toward the correction, torque capped at the EVA authority,
       so no substep is more forceful than a maxed command). */
    void slewTo(const glm::dmat3 &target, double h);
};

/* Arm the active kerbal's controls for this tick from the keys, the
   camera and the ground state (tick.cpp calls it instead of the ship's
   Command path). isDown = the tick's key-state closure (keyboard OR the
   --sim-press windows). */
void evaArmCommands(Game &g, const std::function<bool(SDL_Scancode)> &isDown);
