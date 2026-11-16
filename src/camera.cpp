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

// fov is VERTICAL; the horizontal half-angle comes from fov*aspect.
void Camera::computeFrustumAngles() {
    const double hy = (double)fov * 0.5;
    // tan(horizontal half) = tan(vertical half) * aspect.
    const double hx = std::atan(std::tan(hy) * (double)aspect);
    frustumSinY = std::sin(hy); frustumCosY = std::cos(hy);
    frustumSinX = std::sin(hx); frustumCosX = std::cos(hx);
}

void Camera::setAspect(float _aspect) {
    // Store it: setFov() rebuilds the projection from the member, so a stale
    // aspect would snap the horizontal fov back to its launch value.
    this->aspect = _aspect;
    this->projection = reverseZInfinitePerspective(fov, _aspect, zNear);
    computeFrustumAngles();
}

void Camera::setViewport(int w, int h) {
    this->viewport_w = w;
    this->viewport_h = h;
}

void Camera::setFov(float _fov) {
    this->fov = clampFovRad(_fov);
    this->projection = reverseZInfinitePerspective(fov, aspect, zNear);
    computeFrustumAngles();
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
    this->projection = reverseZInfinitePerspective(this->fov, aspect, zNear);
    computeFrustumAngles();
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
    const double limX = panLimitX(), limY = panLimitY();
    panYaw = std::clamp(panYaw, -limX, limX);
    panPitch = std::clamp(panPitch, -limY, limY);
    /* Two independent axis clamps describe a rectangle whose CORNER is further
       off the view line than either edge: at fov 120 deg the corner is
       1.68 rad, which points the camera past sideways -- the focus would go
       behind the camera and aim= would read a mirrored position the player
       cannot chase back. Pull any corner back onto the circle. */
    const double r = std::hypot(panYaw, panPitch);
    if (r > kPanCircleLimit) {
        const double k = kPanCircleLimit / r;
        panYaw *= k;
        panPitch *= k;
    }
}

void Camera::PanPx(double dxPx, double dyPx) {
    if (mode != CAM_ORBIT) { return; }
    const double k = 1.0 / panPxPerRad();
    panYaw += dxPx * k;
    panPitch += dyPx * k;
    clampPan();
}

void Camera::resetPan() {
    // Orbit only, like PanPx: a pan left behind a Free-mode excursion would
    // otherwise reappear when the player drops back onto the turntable.
    if (mode != CAM_ORBIT) { return; }
    panYaw = 0.0;
    panPitch = 0.0;
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
            /* Rotate the look direction in the UN-PANNED view basis. right is
               cross(forward, up) -- the same right buildView() derives -- which
               makes a positive panYaw put the subject right and a positive
               panPitch put it down: the image follows the cursor. Rotating the
               aim here rather than moving focusPoint is what keeps every clamp
               about the POSITION valid (#876's deck stop included) while the
               subject slides around the frame (#919). */
            const glm::dvec3 f0 = glm::normalize(dir);
            /* |cross(f0, refUp)| = cos(orbitPitch), and clampOrbitPitch() keeps
               orbitPitch inside kPitchLimit's 87 deg, so the smallest this can
               be is 0.0509 -- the pan never touches f0, so it cannot bring the
               view line near the turntable's pole either. A silent fallback here
               would only hide a broken pitch clamp. */
            glm::dvec3 right = glm::cross(f0, refUp);
            assert(glm::dot(right, right) > 1e-12);
            right = glm::normalize(right - f0 * glm::dot(right, f0));
            const glm::dvec3 up0 = glm::cross(right, f0);
            dir = glm::dmat3(glm::rotate(panYaw, up0))
                * (glm::dmat3(glm::rotate(panPitch, right)) * f0);
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
