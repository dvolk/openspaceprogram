// eva.cpp -- the EVA kerbal's control laws (declarations in eva.h).

#include "eva.h"

#include <cassert>
#include <cstdio>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>      // length2

#include "camera.h"    // Camera (the arm reads the view basis)
#include "game.h"      // Game (the active kerbal, the clock)
#include "inventory.h" // inventoryDrain (the pocket draw, phase 4.5)
#include "physics.h"   // BodyInContact, ApplyCentralForce, ApplyTorque, ...

// --- tuning -----------------------------------------------------------
static const double kWalkSpeed   = 2.5;     // m/s
static const double kWalkAccel   = 10.0;    // m/s^2 toward walkSpeed (damping)
static const double kJumpSpeed   = 2.5;     // m/s radial kick
// RCS: fixed thrust (N), not fixed accel -- no speed cap; propellant is the limiter.
static const double kRcsIsp      = 220.0;   // s, monopropellant efficiency
static const double kRcsForce    = 194.1;   // N; ~2 m/s^2 on the wet suit
static const double kRcsFlow     = 0.090;   // kg/s = kRcsForce / (kRcsIsp * 9.81)
static const double kEvaTorque   = 100.0;   // N m attitude authority (space)
static const double kGroundTorque = 300.0;  // N m attitude authority (ground)
static const double kMaxRate     = 6.0;     // rad/s attitude slew cap
static const double kYawRate     = 1.5;     // rad/s QE yaw about the view axis
static const double kGroundBand  = 0.25;    // m above restAlt still "grounded"
static const double kFloorDrop   = 0.4;     // m below restAlt -> snap back up
// Ladder (#1073): the hang is a force servo, so imperfectly placed segments
// compose -- the kerbal is PULLED to the line, and a transfer across a gap or
// a lateral offset is a smooth slide, not a teleport. e2e pins the behavior
// (climb direction/speed band, transfer, stable hang), not these numbers.
static const double kClimbSpeed    = 1.5;   // m/s the hold point travels along the segment
static const double kLadderHold    = 0.45;  // m off the line (clears both margins)
static const double kLadderP       = 200.0; // /s^2 position spring (gravity sag g/kP ~ 5 cm)
static const double kLadderD       = 28.0;  // /s damping (~critical: 2*sqrt(kP))
static const double kLadderHoldAccel = 30.0;// m/s^2 servo clamp
static const double kLadderTorque  = 300.0; // N m attitude authority (ground's)
static const double kLadderGrabRange = 1.5; // m COM-to-segment grab reach
static const double kLadderTransfer  = 0.9; // m end-of-segment transfer reach
static const double kLadderTransferEnd = 0.5;// m of the NEXT segment's opposite
                                            // end the suit must enter at (half
                                            // the reach: gap yes, overlap no)
static const double kLadderEndEps    = 0.1; // m of the end counts as "at the end"
                                            // (> the gravity sag g/kLadderP, which
                                            // on a vertical ladder holds the suit
                                            // ~5 cm short of the top end)
static const double kLadderGroundEnd = 0.6; // m of the bottom end: ground drop band
                                            // (a body length -- the feet touch down
                                            // before the COM reaches the end hold)

bool Kerbal::onDeck() const {
    return frame != nullptr && frame->building != nullptr && frame->building->air;
}

double Kerbal::restAlt() const {
    /* The surface margin must be the one the contact actually uses: the
       terrain's on the ground, the DECK's own effective margin in a platform
       bubble. Reading terrainMargin there rests the suit a fraction of a
       metre above the contact, and the floor guard fights the solver in a
       bounce limit cycle (#850). Both helpers respect OSP_*_MARGIN. */
    const double surf = onDeck() ? effectiveHullMargin(frame->building->body)
                                 : terrainMargin();
    return parts[0]->def->height / 2.0 + hullMargin() + surf;
}

bool Kerbal::inStationBubble() const {
    return frame != nullptr && frame->building != nullptr && !frame->building->air;
}

glm::dvec3 Kerbal::groundUp() const {
    if(onDeck()) { return glm::dvec3(0.0, 0.0, 1.0); }
    return glm::normalize(comPos());
}

double Kerbal::groundAltAt(const glm::dvec3 &pos) const {
    if(onDeck()) { return pos.z; }   // deck top authored at local z=0
    return glm::length(pos)
        - (double)m_parent->GetTerrainHeight(glm::vec3(glm::normalize(pos)));
}

bool Kerbal::overDeck() const {
    /* True when a ray straight down from the suit hits the platform deck:
       over the footprint, not off the edge. The target sits a little below
       the deck plane so a standing suit always reaches it; a suit fallen
       past it is off the deck and must keep falling -- the floor is the
       deck, not an infinite plane at z=0 (#850). */
    if(!onDeck()) { return false; }
    const glm::dvec3 pos = comPos();
    glm::dvec3 hit, n;
    Body *gb = nullptr;
    return groundRay(pos, glm::dvec3(pos.x, pos.y, -(kFloorDrop + 1.0)),
                     hit, n, hull, &gb)
        && gb == frame->building->body;
}

/* The analytic half of armInput's contact test, without the Bullet query:
   a hull freshly added by enterWorld has no manifolds to find, and the pose
   already says everything. A kerbal loaded standing on a structure that rises
   above the analytic surface reads "space" until the first tick's
   BodyInContact flips it back -- except on a platform deck, whose bubble
   knows its own floor (#850). */
void Kerbal::settleContact(const std::vector<Vehicle *> &vehicles) {
    assert(m_parent != nullptr && !parts.empty());
    /* A restored grip wins over any contact reading: the hang is a regime of
       its own, and the hull may not even be in the world yet at load. */
    if(resolveLadder(vehicles)) {
        grounded = false;
        mode = EVA_LADDER;
        return;
    }
    if(inStationBubble()) {
        grounded = false;   // weightless: no deck to stand on (#850)
        mode = EVA_SPACE;
        return;
    }
    const double alt = groundAltAt(comPos());
    grounded = alt < restAlt() + kGroundBand && (!onDeck() || overDeck());
    mode = grounded ? EVA_GROUND : EVA_SPACE;
}

/* The tick's regime, keys aside (#215): armed or disarmed, a suit asks the same
   question -- is it standing, or is it falling/flying? Bullet contact first, because
   the meshes are only there at max-LOD leaves, with the analytic height as the guard
   (and the deck's own floor inside a platform bubble, #850). */
void Kerbal::updateContact() {
    const glm::dvec3 radial = groundUp();
    const double alt = groundAltAt(get_center_of_mass());
    const double rest = restAlt();
    grounded = !inStationBubble()
        && (BodyInContact(hull)
            || (alt < rest + kGroundBand && (!onDeck() || overDeck())));
    if(jumping) {
        /* Release once the jump's outward kick is spent. Releasing on
           contact-clear never fires for a hop that never leaves contact;
           releasing on altitude never fires when the apex is under the band.
           Either way the latch sticks and the kerbal is stuck in space mode,
           where W burns suit RCS instead of walking. Gravity spends the kick
           on any body. */
        if(glm::dot(GetVel(), radial) > 0.0) { grounded = false; }
        else { jumping = false; }
    }
    mode = grounded ? EVA_GROUND : EVA_SPACE;
}

/* --- the ladder grip (#1073) ------------------------------------------- */

void Kerbal::ladderSegment(glm::dvec3 &A, glm::dvec3 &B, glm::dvec3 &out) const {
    assert(ladderPart != nullptr && ladderPart->isLadder());
    glm::dvec3 p; glm::dmat3 r;
    ladderPart->owner->partPoseRelTo(ladderPart, frame, p, r);
    A = p + r * ladderPart->def->ladder_a;
    B = p + r * ladderPart->def->ladder_b;
    out = r * ladderPart->def->ladder_out;
}

Part *Kerbal::nearestLadder(const std::vector<Vehicle *> &vehicles, double reach,
                            uint64_t excludeUid) {
    Part *best = nullptr;
    double bestD = reach;
    const glm::dvec3 pos = partPos(controller);
    for(Vehicle *v : vehicles) {
        /* A grip needs a live hull: a railed ship's pose is a frozen park, and
           the hang law reads its parts per substep. */
        if(v == this || v->onRails || !v->hullInWorld()) { continue; }
        for(Part *p : v->parts) {
            if(!p->isLadder() || p->uid == excludeUid) { continue; }
            glm::dvec3 pp; glm::dmat3 pr;
            v->partPoseRelTo(p, frame, pp, pr);
            const glm::dvec3 A = pp + pr * p->def->ladder_a;
            const glm::dvec3 B = pp + pr * p->def->ladder_b;
            const double L = glm::length(B - A);
            assert(L > 0.0);   // the parse door refuses coincident ends
            const double d = ladderDist(A, (B - A) / L, L, pos);
            if(d < bestD) { bestD = d; best = p; }
        }
    }
    return best;
}

bool Kerbal::canRail() {
    /* A grip is a park the suit does not own: applyLadder steers it against the
       hull it hangs from, and railsTick has no such law -- parked, the suit is a
       frozen pose in mid-air, and the hull parked beside it takes the grip away
       the next tick (resolveLadder's on-rails refusal, #1073). Letting the suit
       rail is what made #1162's second face: switch ships while hanging and
       select_vehicle() parks the suit itself -- isGrounded() counts a hang 34 m up
       as "grounded around Kerbin", so the park is refused by nobody upstream
       ("Ash frozen on rails"). Everything else rails as it always did: standing
       on the pad, or weightless off-world. */
    if(ladderUid != 0) { return false; }
    return Vehicle::canRail();
}

bool Kerbal::gripsLadderOf(const Vehicle *v) const {
    if(ladderUid == 0 || v == nullptr) { return false; }
    for(const Part *p : v->parts) {
        if(p->uid == ladderUid) { return true; }
    }
    return false;
}

bool Kerbal::resolveLadder(const std::vector<Vehicle *> &vehicles) {
    const uint64_t prevUid = ladderUid;
    const Part *prevPart = ladderPart;
    if(ladderUid == 0) { ladderPart = nullptr; return false; }
    /* The uid is the durable grip; the part is looked up fresh every tick, so
       staging the ladder away, undocking it or parking its ship drops the
       grip with no pointer lifetime to manage. */
    Part *found = nullptr;
    for(Vehicle *v : vehicles) {
        if(v == this || v->onRails || !v->hullInWorld()) { continue; }
        for(Part *p : v->parts) {
            if(p->uid == ladderUid) { found = p; break; }
        }
        if(found != nullptr) { break; }
    }
    if(found == nullptr || !found->isLadder()) {
        ladderUid = 0;
        ladderPart = nullptr;
        return false;
    }
    ladderPart = found;
    glm::dvec3 A, B, out;
    ladderSegment(A, B, out);
    const glm::dvec3 u = glm::normalize(B - A);
    const double L = glm::length(B - A);
    const double s = ladderProject(A, u, L, partPos(controller));
    /* Climbing DOWN off the bottom end onto GROUND: let go onto your feet.
       Only while climbing down -- grabbing the bottom end while standing
       keeps the grip (W climbs up from there). A ray, not BodyInContact or
       the analytic altitude: at a capsule rim the suit touches the ship's
       hull (not the ground), and on a raised pad the analytic terrain is
       metres below the deck. Ground is terrain (no user pointer) or a
       building (a Body, but not any vehicle's hull); a vehicle hull -- the
       grabbed stack, another ship, another suit -- is not ground. The band is
       a body length, not kLadderEndEps: the feet touch down while collision
       still holds the COM above the end's hold point. */
    if(s < kLadderGroundEnd && climbDir < 0.0 && !inStationBubble()) {
        const glm::dvec3 pos = comPos();
        glm::dvec3 hit, nrm;
        Body *hitBody = nullptr;
        if(groundRay(pos, pos - groundUp() * (restAlt() + kGroundBand),
                     hit, nrm, hull, &hitBody)) {
            bool vehicleHull = false;
            for(Vehicle *v : vehicles) {
                if(v->hull == hitBody) { vehicleHull = true; break; }
            }
            if(!vehicleHull) {
                ladderUid = 0;
                ladderPart = nullptr;
                return false;
            }
        }
    }
    /* At an end, another segment within reach takes over -- the generosity
       that lets a capsule ladder meet a slightly misplaced tank ladder. Two
       hysteresis gates, because a junction is the bottom of one segment and
       the top of the next: the climb demand must push PAST this end (a
       disarmed hang keeps its segment), and the suit must enter the NEXT
       segment at its opposite end -- overlapping ladders (a part placed over
       the hatch ladder) otherwise transfer into each other's end bands and
       ping-pong every tick. The entry band is half the transfer reach: wide
       enough for a sloppy gap, narrow enough to refuse a mid-segment entry. */
    if((s > L - kLadderEndEps && climbDir > 0.0)
       || (s < kLadderEndEps && climbDir < 0.0)) {
        Part *next = nearestLadder(vehicles, kLadderTransfer, ladderUid);
        if(next != nullptr) {
            glm::dvec3 p2; glm::dmat3 r2;
            next->owner->partPoseRelTo(next, frame, p2, r2);
            const glm::dvec3 A2 = p2 + r2 * next->def->ladder_a;
            const glm::dvec3 B2 = p2 + r2 * next->def->ladder_b;
            const double L2 = glm::length(B2 - A2);
            const double s2 = ladderProject(A2, (B2 - A2) / L2, L2,
                                            partPos(controller));
            const bool enters = (climbDir > 0.0)
                ? (s2 < kLadderTransferEnd)
                : (s2 > L2 - kLadderTransferEnd);
            if(enters) {
                ladderUid = next->uid;
                ladderPart = next;
            }
        }
    }
    /* A new grip (grab, transfer, spawn/load restore -- the cache was empty)
       starts its hold target at the current projection: the servo flies the
       suit in, no teleport. */
    if(ladderUid != prevUid || prevPart == nullptr) {
        glm::dvec3 A2, B2, out2;
        ladderSegment(A2, B2, out2);
        ladderS = ladderProject(A2, glm::normalize(B2 - A2),
                                glm::length(B2 - A2), partPos(controller));
    }
    return true;
}

void Kerbal::applyLadder(double h) {
    Body *b = hull;
    assert(ladderPart != nullptr && ladderPart->isLadder());
    glm::dvec3 A, B, out;
    ladderSegment(A, B, out);
    const glm::dvec3 u = glm::normalize(B - A);
    const glm::dvec3 pos = partPos(controller);
    const glm::dvec3 hold = ladderHold(A, u, ladderS, out, kLadderHold);
    /* One position PD toward the hold point, all three axes: it pulls the
       kerbal onto the line on grab, slides it across a transfer gap, holds
       it against gravity (sag g/kLadderP ~ 5 cm) and tracks the climbing
       hold point armInput advances. The damping is on the velocity RELATIVE
       to the grabbed ship, not the frame: in a rotating frame an orbiting
       hull carries ~km/s of frame-local velocity, and damping that brakes
       the suit out of its orbit while the hold point races on. The ship's
       COM velocity in the suit's frame stands in for the hold point's (the
       ladder is rigid with the ship; the spin lever is a small disturbance
       the spring absorbs). */
    glm::dvec3 shipPos, shipVel;
    ladderPart->owner->comStateIn(frame, shipPos, shipVel);
    glm::dvec3 a = kLadderP * (hold - pos)
                 - kLadderD * (partVel(controller) - shipVel);
    const double alen = glm::length(a);
    if(alen > kLadderHoldAccel) { a *= kLadderHoldAccel / alen; }
    ApplyCentralForce(b, b->mass * a);
    /* Body along the ladder, face to the hull: the stand target with the
       ladder axis as "up" and the inward normal as the face hint. */
    slewTo(evaStandTarget(u, -out), h, kLadderTorque);
}

bool Kerbal::hatchPose(const Part *cap, glm::dvec3 &pos, glm::dmat3 &rot) {
    assert(cap != nullptr && cap->owner != nullptr);
    if(!cap->isLadder()) { return false; }
    glm::dvec3 p; glm::dmat3 r;
    cap->owner->partWorldPose(cap, p, r);
    const glm::dvec3 A = p + r * cap->def->ladder_a;
    const glm::dvec3 B = p + r * cap->def->ladder_b;
    const glm::dvec3 out = r * cap->def->ladder_out;
    pos = B + out * kLadderHold;
    rot = evaStandTarget(glm::normalize(B - A), -out);
    return true;
}

void Kerbal::armInput(Game &g, const SlotProbe &active) {
    const Camera *cam = g.camera;

    // Camera positions/directions live in the render frame (= the kerbal's frame).
    const glm::dvec3 fwd = glm::normalize(cam->forward);
    const glm::dvec3 up = glm::normalize(evaOntoPlane(cam->up, fwd));
    const glm::dvec3 sright = evaScreenRight(fwd, up);
    camBasis = evaCamBasis(fwd, up);

    /* The grab edge (#1073), spent whichever mode runs: let go when gripping,
       else grab the nearest ladder in reach. The servo does the rest -- the
       kerbal is pulled onto the line, no teleport. */
    const bool grabWanted = grabPressed;
    grabPressed = false;
    /* No toast here: the arm runs in the headless test tier too (TBASE has no
       game.o), and the servo's pull-in is the player's feedback, as the jump's
       silence is. The printf is the e2e oracle. */
    if(grabWanted) {
        if(ladderUid != 0) {
            ladderUid = 0;
            ladderPart = nullptr;
            printf("[crew] t=%.1f Let go: '%s'\n", g.time, displayName().c_str());
        } else {
            Part *p = nearestLadder(g.vehicles, kLadderGrabRange, 0);
            if(p != nullptr) {
                /* Only the uid: resolveLadder caches the part and, seeing the
                   cache was empty, re-projects ladderS onto the NEW segment
                   at the suit's current position. Setting the part here would
                   skip that and fly the suit to the previous grip's arc
                   length -- a teleport up the ladder. */
                ladderUid = p->uid;
                /* A fresh grip takes a fresh climb demand: walking on the ground never
                   touches climbDir, so a stale -1 from the last descent would make
                   resolveLadder()'s bottom-end release fire against this grip in the
                   same tick it was taken. (#1095) */
                climbDir = 0.0;
                printf("[crew] t=%.1f Grab: '%s' on '%s' ladder\n",
                       g.time, displayName().c_str(),
                       p->owner->displayName().c_str());
            } else {
                printf("[crew] t=%.1f No ladder in reach of '%s'\n",
                       g.time, displayName().c_str());
            }
        }
    }

    const glm::dvec3 radial = groundUp();
    /* The regime is derived here, from the pose, keys aside -- the tick's
       disarm pass skips the active suit (it would spend the grab/jump edges
       this arm exists to consume, #1073). The ladder grip wins over any
       contact reading: a hang is a regime of its own. */
    if(resolveLadder(g.vehicles)) {
        grounded = false;
        mode = EVA_LADDER;
    } else {
        updateContact();
    }
    /* Spend the jump edge whichever mode is running (#578). It used to be read only on
       the ground branch, so a press during a spacewalk survived the tick and fired on
       the next landing: SPACE is the staging key for every ship, so tapping it out of
       habit while flying the suit collected an unasked 2.5 m/s hop on touchdown. */
    const bool jumpWanted = jumpPressed;
    jumpPressed = false;

    if(mode == EVA_LADDER) {
        /* W/S climb toward the segment's top/bottom end (ladder-relative,
           like KSP); A/D idle in v1. The demand moves the hold TARGET; the
           servo flies the suit to it (applyLadder). */
        climbDir = 0.0;
        if(active.held(Slot::EvaForward)) { climbDir += 1.0; }
        if(active.held(Slot::EvaBack)) { climbDir -= 1.0; }
        walkDir = glm::dvec3(0.0);
        armRcs(glm::dvec3(0.0));   // hanging, not flying
        glm::dvec3 A, B, out;
        ladderSegment(A, B, out);
        /* The tick spans g.dt * time_accel of sim time (tick.cpp's substep
           loop); the hold target must advance on the SAME clock or the climb
           crawls at physics warp while the ship's physics runs full speed. */
        ladderS = glm::clamp(ladderS + climbDir * kClimbSpeed * g.dt * g.time_accel,
                             0.0, glm::length(B - A));
        if(jumpWanted) {
            /* SPACE on a ladder: let go with an outward kick. The space law
               runs this tick's substeps; the next tick re-derives the regime. */
            SetVelocity(hull, partVel(controller) + out * kJumpSpeed);
            ladderUid = 0;
            ladderPart = nullptr;
            jumping = true;
            mode = EVA_SPACE;
            printf("[crew] t=%.1f Let go: '%s' (jump)\n",
                   g.time, displayName().c_str());
        }
    } else if(mode == EVA_GROUND) {
        glm::dvec3 w(0.0);
        if(active.held(Slot::EvaForward)) { w += fwd; }
        if(active.held(Slot::EvaBack)) { w -= fwd; }
        if(active.held(Slot::EvaRight)) { w += sright; }
        if(active.held(Slot::EvaLeft)) { w -= sright; }
        w = evaOntoPlane(w, radial);   // walk along the tangent
        walkDir = (glm::length2(w) > 1e-9) ? glm::normalize(w)
                                           : glm::dvec3(0.0);
        armRcs(glm::dvec3(0.0));   // walking, not flying: no translation demand
        if(jumpWanted) {           // EVA_GROUND means grounded (two lines above)
            jumpRequested = true;
            jumping = true;
        }
    } else {
        glm::dvec3 t(0.0);
        if(active.held(Slot::EvaForward)) { t += fwd; }
        if(active.held(Slot::EvaBack)) { t -= fwd; }
        if(active.held(Slot::EvaRight)) { t += sright; }
        if(active.held(Slot::EvaLeft)) { t -= sright; }
        if(active.held(Slot::EvaUp)) { t += up; }
        if(active.held(Slot::EvaDown)) { t -= up; }
        armRcs((glm::length2(t) > 1e-9)
            ? glm::normalize(t) : glm::dvec3(0.0));
        walkDir = glm::dvec3(0.0);
        double yaw = 0.0;
        if(active.held(Slot::EvaYawLeft)) { yaw += 1.0; }
        if(active.held(Slot::EvaYawRight)) { yaw -= 1.0; }
        viewYaw += yaw * kYawRate * g.dt;
    }
}

/* A tick with no arm on this suit (#215). applyEva() runs per substep for every live
   hull, and these fields are the only thing deciding what it does with them, so the
   tick loop owes every kerbal it did NOT arm a neutral set -- the same rule that makes
   the ships clearThrust/clearRotCmd/clearRcs every tick. The regime is re-derived
   rather than kept: the constructor's defaults are "standing on the ground", which on
   an orbital suit means the walk law's brake pulls 10 m/s^2 out of a 2.2 km/s orbit.
   camBasis and viewYaw deliberately survive: they are the pose the suit was facing,
   and the space law holds it instead of hunting for a view nobody is driving. */
void Kerbal::disarm(const std::vector<Vehicle *> &vehicles) {
    if(resolveLadder(vehicles)) {
        grounded = false;
        mode = EVA_LADDER;
    } else {
        updateContact();
    }
    walkDir = glm::dvec3(0.0);
    climbDir = 0.0;   // the grip survives; the climb demand does not
    /* Not armRcs(): clearRcs() is the whole armed-translation state -- the demand, the
       request the window writes, and the rcsFiring flag the render pass gates the puff
       on -- and a suit with no pilot should hold none of it. */
    clearRcs();
    jumpRequested = false;
    /* A press cannot outlive the tick it arms: an edge raised in the same
       event batch as a ship switch would otherwise latch on the now-disarmed
       suit and fire on return (a surprise let-go / hop). The ACTIVE suit is
       skipped by the tick's disarm pass -- its arm below spends the edges. */
    grabPressed = false;
    jumpPressed = false;
}

void Kerbal::applyEva(double h) {
    Body *b = hull;
    /* The ladder hang (#1073): a regime of its own, BEFORE the floor guard --
       a hanging kerbal is meant to be off the ground, and a terrain snap
       would teleport it off the ladder. */
    if(mode == EVA_LADDER) {
        applyLadder(h);
        return;
    }
    const glm::dvec3 pos = partPos(controller);
    const glm::dvec3 radial = groundUp();
    const double rest = restAlt();

    // Analytic floor guard: snap back to standing height where terrain meshes
    // are not loaded. In a platform bubble the floor is the DECK, not an
    // infinite plane at z=0: off the edge the suit keeps falling, and the
    // space-mode laws below MUST still run (an early return here killed RCS
    // and camera-facing mid-fall, #850). In a STATION bubble there is no floor
    // -- weightless, and snapping would teleport the suit a planet away.
    if(!inStationBubble() && groundAltAt(pos) < rest - kFloorDrop
       && (!onDeck() || overDeck())) {
        const glm::dvec3 snap = onDeck()
            ? glm::dvec3(pos.x, pos.y, rest)
            : radial * ((double)m_parent->GetTerrainHeight(
                            glm::vec3(glm::normalize(pos))) + rest);
        placeShipAtCom(snap, partRot(controller));
        const glm::dvec3 v = partVel(controller);
        const double vr = glm::dot(v, radial);
        if(vr < 0.0) { SetVelocity(b, v - radial * vr); }
        return;
    }

    if(mode == EVA_GROUND) {
        if(jumpRequested) {
            jumpRequested = false;
            SetVelocity(b, partVel(controller) + radial * kJumpSpeed);
        }
        const glm::dvec3 v = partVel(controller);
        /* The ground is stationary in the kerbal's frame: terrain in the rot
           frame, a platform deck in its bubble (#850). The feet are
           frictionless, so this law is the kerbal's only horizontal grip. */
        const glm::dvec3 vh = evaOntoPlane(v, radial);
        glm::dvec3 a = (walkDir * kWalkSpeed - vh) / h;
        const double amax = kWalkAccel;
        const double alen = glm::length(a);
        if(alen > amax) { a *= amax / alen; }
        ApplyCentralForce(b, b->mass * a);

        // Feet are frictionless (Kerbal ctor) so the COM steering force cannot
        // tip the capsule; overwriting the pose would stall translation instead.
        const glm::dvec3 faceHint = (glm::length2(walkDir) > 0.0)
            ? walkDir : partAxis(controller, 1);
        slewTo(evaStandTarget(radial, faceHint), h, kGroundTorque);
    } else {
        // RCS: fixed thrust while held AND hydrazine is available (consume-then-arm).
        if(glm::length2(rcsDir) > 0.0) {
            const double flow = kRcsFlow * h;
            // Suit tank first, then pocket inventory (not in this ship's fuel groups).
            bool haveFuel = consumeResourceMass(ResourceType::Hydrazine, flow, parts[0]).ok;
            if(!haveFuel) {
                haveFuel = inventoryDrain(parts[0], (int)ResourceType::Hydrazine, flow);
            }
            if(haveFuel) {
                ApplyCentralForce(b, kRcsForce * rcsWorldDir());
                // The render pass draws the COM puff off this (render.cpp).
                markRcsBurned();
            }
        }
        // Upright on screen, facing the camera direction, plus QE yaw.
        const glm::dvec3 fwd = camBasis[2];
        slewTo(rotAbout(fwd, viewYaw) * evaSpaceTarget(camBasis), h,
               kEvaTorque);
    }
}

void Kerbal::slewTo(const glm::dmat3 &target, double h, double authority) {
    Body *b = hull;
    const glm::dmat3 R = partRot(controller);
    glm::dvec3 axis;
    const double ang = evaRotAxisAngle(target * glm::transpose(R), axis);
    const glm::dvec3 w = hullAngVel();
    const glm::dmat3 I = getInertia();
    glm::dvec3 tq(0.0);
    if(ang < 1e-9) {
        if(glm::length(w) < 1e-4) { return; }   // aligned: kill residual spin
        tq = -(I * w) / h;
    } else {
        // Braking curve: fastest rate that can still stop exactly on target.
        // A plain linear rate law overshoots and oscillates under the torque cap.
        const double Ieff = glm::dot(axis, I * axis);
        const double alpha = (Ieff > 0.0) ? authority / Ieff : 0.0;
        const double w_des = glm::min(glm::min(std::sqrt(2.0 * alpha * ang),
                                               ang / (2.0 * h)), kMaxRate);
        tq = I * (axis * w_des - w) / h;
    }
    const double m = glm::length(tq);
    if(m > authority) { tq *= authority / m; }
    ApplyTorque(b, tq);
}
