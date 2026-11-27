#include "camera.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <numbers>

/* One fov range everywhere: --fov (cli.cpp), the Settings slider and the
   settings.json load all use [10,120]; this is the last line of defence against
   a caller handing over garbage. Outside (0,180)deg tan(fov/2) goes negative,
   and that sign lands on BOTH the projection scale and computeFrustumAngles'
   half angles, so every patchCulled() test in the draw pass rejects: the world
   renders empty and nothing anywhere reports an error. */
static float clampFovRad(float fov) {
    const float deg = (float)std::numbers::pi / 180.0f;
    if(!std::isfinite(fov)) {
        // std::clamp passes NaN straight through (both comparisons false) -- the
        // same silent empty world the clamp exists to stop.
        return 60.0f * deg;
    }
    return std::clamp(fov, 10.0f * deg, 120.0f * deg);
}

// Reverse-Z infinite-far perspective (1.0 near -> 0.0 far). Pairs with
// glClipControl(GL_ZERO_TO_ONE) / glDepthFunc(GL_GEQUAL) in display.cpp.
static glm::mat4 reverseZInfinitePerspective(float fov, float aspect, float zNear) {
    const float t = 1.0f / std::tan(fov * 0.5f);
    const float x = t / aspect;   // vertical fov
    const float y = t;
    return glm::mat4(
        x,    0.0f, 0.0f, 0.0f,    // col 0
        0.0f, y,    0.0f, 0.0f,    // col 1
        0.0f, 0.0f, 0.0f, -1.0f,   // col 2: clip.z = zNear
        0.0f, 0.0f, zNear, 0.0f);  // col 3: clip.w = -view.z
}

/* Reverse-Z ortho for the same GL state (GL_ZERO_TO_ONE + GL_GEQUAL): the depth
   window runs 1.0 at nearZ down to 0.0 at farZ, exactly like the perspective
   branch above, so nothing downstream has to know which projection is live.
   View space puts the subject at negative z, so
       depth(z) = (farZ + z) / (farZ - nearZ)
   reads 1 at z = -nearZ and 0 at z = -farZ. The x/y scales are the frame's own
   half-extents in metres; clip.w stays 1, which is the whole difference -- no
   perspective divide, so nothing scales with distance. */
static glm::mat4 reverseZOrtho(float halfH, float aspect, float nearZ, float farZ) {
    const float halfW = halfH * aspect;
    const float span = farZ - nearZ;
    return glm::mat4(
        1.0f / halfW, 0.0f,         0.0f,        0.0f,
        0.0f,         1.0f / halfH, 0.0f,        0.0f,
        0.0f,         0.0f,         1.0f / span, 0.0f,
        0.0f,         0.0f,         farZ / span, 1.0f);
}

// fov is VERTICAL; the horizontal half-angle comes from fov*aspect.
void Camera::computeFrustumAngles() {
    if(ortho) {
        /* An ortho frame has no side planes fanning from the eye: its walls are
           parallel to the view axis, and the wall test the draw pass writes
           (zv*sin - |xv|*cos >= 0) cannot describe a parallel wall in those
           terms. sin=1, cos=0 leaves the test as "in front of the camera", which
           is the fail-safe direction: geometry is never culled that should draw,
           and what is lost is only the CPU skip. A fitted view draws a hull and a
           sky, so that is the cheap side to be loose on. */
        frustumSinX = frustumSinY = 1.0;
        frustumCosX = frustumCosY = 0.0;
        return;
    }
    const double hy = (double)fov * 0.5;
    // tan(horizontal half) = tan(vertical half) * aspect.
    const double hx = std::atan(std::tan(hy) * (double)aspect);
    frustumSinY = std::sin(hy); frustumCosY = std::cos(hy);
    frustumSinX = std::sin(hx); frustumCosX = std::cos(hx);
}

void Camera::rebuildProjection() {
    if(ortho) {
        projection = reverseZOrtho((float)orthoHalfH, aspect,
                                   (float)orthoNear, (float)orthoFar);
    } else {
        projection = reverseZInfinitePerspective(fov, aspect, zNear);
    }
    computeFrustumAngles();
}

void Camera::setOrtho(bool on, double halfH, double nearZ, double farZ) {
    /* A zero or negative half-height makes the x/y scales infinite and the whole
       frame collapses onto the view axis; an inverted or zero-width depth window
       puts the subject outside [0,1] and GL_GEQUAL erases it. Both are arithmetic
       errors rather than states a caller can recover from, so they assert. */
    assert(std::isfinite(halfH) && halfH > 0.0);
    assert(std::isfinite(nearZ) && std::isfinite(farZ));
    assert(nearZ > 0.0 && farZ > nearZ);
    ortho = on;
    orthoHalfH = halfH;
    orthoNear = nearZ;
    orthoFar = farZ;
    rebuildProjection();
}

void Camera::zoomOrtho(double factor) {
    if(!ortho) { return; }
    assert(std::isfinite(factor) && factor > 0.0);
    /* Clamped at both ends: a box below a palm keeps the parts' own triangles
       coarser than the pixels and the hull reads as a blob; a box past a body's
       horizon puts the whole world in the frame at one metre per pixel. */
    orthoHalfH = std::clamp(orthoHalfH * factor, 0.25, 5.0e4);
    rebuildProjection();
}

double Camera::fitHalfHeight(double radius) const {
    // The box must hold the sphere on BOTH axes; halfW = halfH * aspect, so the
    // binding axis is the narrower one and the half-height divides by min(1,aspect).
    assert(radius > 0.0);
    return radius / std::min(1.0, (double)aspect);
}

double Camera::fitDistance(double radius) const {
    /* Perspective: the sphere fits when its angular radius clears the shorter
       half-angle, so sin(hmin) = radius / distance. The half-angles come from the
       same members the projection is built from, which is why this is a member
       and not a free function: change the fov slider and the fit moves with it. */
    assert(radius > 0.0);
    const double hy = (double)fov * 0.5;
    const double hx = std::atan(std::tan(hy) * (double)aspect);
    const double hmin = std::min(hx, hy);
    assert(hmin > 0.0);   // clampFovRad() holds fov inside [10,120]deg
    return radius / std::sin(hmin);
}

void Camera::setAspect(float _aspect) {
    // Store it: setFov() rebuilds the projection from the member, so a stale
    // aspect would snap the horizontal fov back to its launch value.
    this->aspect = _aspect;
    rebuildProjection();
}

void Camera::setViewport(int w, int h) {
    this->viewport_w = w;
    this->viewport_h = h;
}

void Camera::setFov(float _fov) {
    this->fov = clampFovRad(_fov);
    rebuildProjection();
}

const glm::dvec3& Camera::GetPos() const {
    return pos;
}

const glm::dvec3& Camera::GetForward() const {
    return forward;
}

const glm::mat4 &Camera::GetProjection() const {
    return projection;
}

const glm::dmat4 &Camera::GetView() const {
    return view;
}

Camera::Camera(const glm::dvec3& focusPos, float fov, float aspect, float zNear, float zFar)
    : fov(clampFovRad(fov)), aspect(aspect), zNear(zNear), zFar(zFar) {
    // Uses the clamped member, not the parameter (which shadows it).
    rebuildProjection();
    // Seed pos/forward/up so a reader before the first ComputeView() gets
    // sensible values.
    this->mode = CAM_ORBIT;
    this->focusPoint = focusPos;
    this->distance = 10;
    this->pos = focusPoint + glm::dvec3(distance, 0, 0);
    this->forward = glm::normalize(focusPoint - pos);
    this->up = glm::dvec3(0, 0, 1);   // ref up (ref is identity at spawn)
    this->view = glm::translate(pos);
}

double Camera::orbitHiStop() const {
    // min(kPitchLimit, downLimit) and not min(downLimit, kPitchLimit): a NaN
    // stop then picks kPitchLimit instead of poisoning both comparisons and
    // opening the turntable (the same trap as clampFovRad above).
    return std::min(kPitchLimit, downLimit);
}

void Camera::clampOrbitPitch() {
    const double hi = orbitHiStop();
    if (orbitPitch > hi) { orbitPitch = hi; }
    if (orbitPitch < -kPitchLimit) { orbitPitch = -kPitchLimit; }
}

double Camera::panPxPerRad() const {
    /* Pixels of drag per radian of pan, measured at the focus: the screen's
       own scale, so the subject moves with the cursor instead of racing it.
       Same identity GeoPatch uses for its LOD budget (terragen.h
       lodPxPerRad): px per rad = (viewport_h/2) / tan(fov/2) -- vertical fov,
       and the horizontal scale is identical because tan(half-angle) scales
       with aspect exactly as the viewport width does. First order by
       construction (the projection is a tan, the pan is an angle): at fov
       60's frame edge the subject is ~9% eager, which is in the noise for a
       drag and keeps the gesture linear rather than fighting tan at the end
       of every stroke. */
    const double h = (double)std::max(1, viewport_h);
    return h * 0.5 / std::tan((double)fov * 0.5);
}

void Camera::clampPan() {
    /* The frame edge, in the order the rotations compose. Pitch first: its stop
       is the vertical half-angle and the aim reads it straight back (aim_y =
       -panPitch). Yaw second: it is applied to a look line already tilted by
       panPitch, so the aim lands at atan(tan(panYaw)/cos(panPitch)) -- further
       off-centre than the angle that produced it. Clamping yaw to the raw
       horizontal half-angle therefore pushed the subject PAST the side edge
       (fov 60 at 4:3: 41.6 deg of aim against a 37.6 deg frame). Undoing the
       cos() puts the corner of the two stops exactly on the corner of the
       frame, which is also why no separate radius guard is wanted: both stops
       are under 90 deg, so the focus can never end up behind the camera. */
    const double limY = panLimitY();
    panPitch = std::clamp(panPitch, -limY, limY);
    const double limX = std::atan(std::tan(panLimitX()) * std::cos(panPitch));
    panYaw = std::clamp(panYaw, -limX, limX);
}

void Camera::PanPx(double dxPx, double dyPx) {
    if (mode != CAM_ORBIT) { return; }
    const double k = 1.0 / panPxPerRad();
    panYaw += dxPx * k;
    panPitch += dyPx * k;
    clampPan();
}

void Camera::resetPan() {
    /* Mode-agnostic on purpose: the offset only shows in the orbit view, but
       zeroing it in Free or First is what stops a framing built before a
       free-cam excursion reappearing when the player drops back onto the
       turntable. */
    panYaw = 0.0;
    panPitch = 0.0;
}

void Camera::setSubject(const glm::dvec3& p) {
    // The framing belongs to whatever the camera was aimed at, so a NEW subject
    // starts centred again (#919). Per-frame re-aiming at the SAME subject goes
    // through Follow(), which keeps it.
    resetPan();
    Follow(p);
}

glm::dvec3 Camera::orbitOffset() const {
    const double cp = std::cos(orbitPitch), sp = std::sin(orbitPitch);
    const double cy = std::cos(orbitYaw),   sy = std::sin(orbitYaw);
    return glm::dvec3(cp * cy, cp * sy, -sp);
}

void Camera::ComputeView() {
    if (mode == CAM_ORBIT) {
        // The caller's stop may have moved since the last drag -- zooming out
        // at the pad shrinks it, and a parked pitch can sit past where the new
        // stop is (#876). Enforce it here rather than leaving the view
        // underground until the player happens to pan.
        clampOrbitPitch();
        // Same reason for the aim offset: fov is live (Settings slider, --fov).
        clampPan();
        const glm::dvec3 off = ref * orbitOffset() * distance;
        pos = focusPoint + off;
        const glm::dvec3 refUp = ref * glm::dvec3(0, 0, 1);
        glm::dvec3 dir = -off;
        if (panYaw != 0.0 || panPitch != 0.0) {
            /* Rotate the look direction in the UN-PANNED view basis: pitch
               about its right first, then yaw about its up. right is
               cross(forward, up) -- the same right buildView() derives -- which
               makes a positive panYaw put the subject right and a positive
               panPitch put it down: the image follows the cursor. Rotating the
               aim here rather than moving focusPoint is what keeps every clamp
               about the POSITION valid (#876's deck stop included) while the
               subject slides around the frame (#919). */
            const glm::dvec3 f0 = glm::normalize(dir);
            /* |cross(f0, refUp)| = cos(orbitPitch), and clampOrbitPitch() keeps
               orbitPitch inside kPitchLimit's 87 deg, so the squared length
               below is never under cos^2(1.52) = 2.6e-3 -- the pan cannot bring
               the view line near the turntable's pole either, since it never
               touches f0. A silent fallback here would only hide a broken pitch
               clamp. */
            glm::dvec3 viewRight = glm::cross(f0, refUp);
            assert(glm::dot(viewRight, viewRight) > 1e-3);
            viewRight = glm::normalize(viewRight);
            const glm::dvec3 viewUp = glm::cross(viewRight, f0);
            dir = glm::dmat3(glm::rotate(panYaw, viewUp))
                * (glm::dmat3(glm::rotate(panPitch, viewRight)) * f0);
        }
        forward = glm::normalize(dir);
        // Guard the degenerate projection at the pole (Pitch clamps away
        // from it, but be safe).
        glm::dvec3 up = refUp - forward * glm::dot(refUp, forward);
        if (glm::dot(up, up) < 1e-12) {
            up = (std::abs(forward.y) < 0.99) ? glm::dvec3(0, 1, 0) : glm::dvec3(1, 0, 0);
            up = up - forward * glm::dot(up, forward);
        }
        // Precision: (focus - renderOrigin) + off, not (focus+off) -
        // renderOrigin -- the latter rounds on the absolute ULP grid and
        // jitters the view at extreme ranges.
        buildView(-forward, up, (focusPoint - renderOrigin) + off);
        return;
    }
    // Free / First: pos is primary; up is the stored free-camera up.
    buildView(-forward, up, pos - renderOrigin);
}

void Camera::buildView(const glm::dvec3& zAxis, const glm::dvec3& upHintIn, const glm::dvec3& cam) {
    // Hand-built instead of glm::lookAt: lookAt's cross(up, z) goes to zero
    // (normalize -> NaN) when looking along the up. Swap in a safe up only
    // in that degenerate case.
    glm::dvec3 upHint = glm::normalize(upHintIn);
    if (glm::abs(glm::dot(upHint, zAxis)) > 0.9999) {
        upHint = (std::abs(zAxis.y) < 0.99) ? glm::dvec3(0, 1, 0) : glm::dvec3(1, 0, 0);
    }
    const glm::dvec3 xAxis = glm::normalize(glm::cross(upHint, zAxis));
    const glm::dvec3 yAxis = glm::cross(zAxis, xAxis);
    up = yAxis;     // keep the stored basis orthonormal
    right = xAxis;  // free-mode basis axis (harmless in Orbit mode)

    // View translation in the render frame (origin = renderOrigin).
    glm::dmat4 m;
    m[0] = glm::dvec4(xAxis.x, yAxis.x, zAxis.x, 0.0);
    m[1] = glm::dvec4(xAxis.y, yAxis.y, zAxis.y, 0.0);
    m[2] = glm::dvec4(xAxis.z, yAxis.z, zAxis.z, 0.0);
    m[3] = glm::dvec4(-glm::dot(xAxis, cam),
                       -glm::dot(yAxis, cam),
                       -glm::dot(zAxis, cam), 1.0);
    view = m;
}

void Camera::toFree() {
    if (mode == CAM_FREE) { return; }
    right = glm::normalize(glm::cross(forward, up));
    mode = CAM_FREE;
}

void Camera::toOrbit(const glm::dvec3& focus) {
    if (mode == CAM_ORBIT) { Follow(focus); return; }
    focusPoint = focus;
    // Free -> orbit: the flier can be anywhere, so the orbit radius is its
    // current standoff. First -> orbit: keep the parked radius/angles -- C must
    // return the framing the cockpit view interrupted.
    if (mode == CAM_FREE) {
        double dist = glm::length(pos - focus);
        if (dist < 10.0) { dist = 10.0; }
        distance = dist;
    }
    mode = CAM_ORBIT;
}

void Camera::toFirst() {
    mode = CAM_FIRST;   // pose is re-derived from the ship every frame
}

void Camera::Follow(const glm::dvec3& p) {
    if (mode != CAM_ORBIT) { return; }
    focusPoint = p;
}

void Camera::wheel(double amt) {
    if(ortho && mode == CAM_FREE) {
        // A fitted orthographic view (the VAB's six) has no distance to pull in
        // and no focus to cross, so the wheel shrinks the box around the subject
        // instead. Same proportional step as the orbit zoom, same sign: scroll up
        // = closer.
        zoomOrtho(std::exp(-amt * 0.25));
        return;
    }
    if (mode != CAM_ORBIT) { return; }
    // Proportional zoom, clamped so the camera can never cross the focus.
    distance *= std::exp(-amt * 0.25);
    if (distance < 2.0) { distance = 2.0; }
    if (distance > 1e9) { distance = 1e9; }
}

void Camera::MoveForward(double amt) {
    if (mode != CAM_FREE) { return; }
    pos += forward * amt;
}

void Camera::MoveRight(double amt) {
    if (mode != CAM_FREE) { return; }
    pos += right * amt;
}

void Camera::MoveUp(double amt) {
    if (mode != CAM_FREE) { return; }
    pos += up * amt;
}

void Camera::Pitch(double angle) {
    if (mode == CAM_ORBIT) {
        // Pitch the turntable. The -side stop is the pole where up vanishes;
        // the +side stop is the caller's, which sits at the deck while the
        // frame is ground-locked (#876).
        orbitPitch += angle;
        clampOrbitPitch();
    } else if (mode == CAM_FREE) {
        const glm::dmat3 rot = glm::dmat3(glm::rotate(angle, right));
        forward = rot * forward;
        up = rot * up;
    }
}

void Camera::RotateY(double angle) {
    if (mode == CAM_ORBIT) {
        // Yaw the turntable around the ref up.
        orbitYaw += angle;
    } else if (mode == CAM_FREE) {
        const glm::dmat3 rot = glm::dmat3(glm::rotate(angle, up));
        forward = rot * forward;
        right = rot * right;
    }
}

void Camera::Roll(double angle) {
    if (mode != CAM_FREE) { return; }
    const glm::dmat3 rot = glm::dmat3(glm::rotate(angle, forward));
    up = rot * up;
    right = rot * right;
}

void Camera::setFreePose(const glm::dvec3& p, const glm::dvec3& fwd, const glm::dvec3& upv) {
    this->pos = p;
    this->forward = glm::normalize(fwd);
    this->up = glm::normalize(upv - this->forward * glm::dot(this->forward, upv));
    this->right = glm::normalize(glm::cross(this->forward, this->up));
    this->mode = CAM_FREE;
    this->ComputeView();
}
