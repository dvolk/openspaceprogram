#include "camera.h"

#include <cmath>

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

void Camera::setAspect(float _aspect) {
    this->projection = reverseZInfinitePerspective(fov, _aspect, zNear);
}

void Camera::setViewport(int w, int h) {
    this->viewport_w = w;
    this->viewport_h = h;
}

void Camera::setFov(float _fov) {
    this->fov = _fov;
    this->projection = reverseZInfinitePerspective(fov, aspect, zNear);
}

const glm::dvec3& Camera::GetPos() const {
    return pos;
}

const glm::dvec3& Camera::GetForward() const {
    return forward;
}

glm::mat4 Camera::GetProjection() const {
    return projection;
}

glm::dmat4 Camera::GetView() const {
    return view;
}

glm::dmat4 *Camera::GetView_() {
    return &view;
}

Camera::Camera(const glm::dvec3& focusPos, float fov, float aspect, float zNear, float zFar)
    : fov(fov), aspect(aspect), zNear(zNear), zFar(zFar) {
    this->projection = reverseZInfinitePerspective(fov, aspect, zNear);
    // Derive pos/forward/up once so a caller that reads them before the
    // first ComputeView() gets sensible values.
    this->mode = CAM_ORBIT;
    this->focusPoint = focusPos;
    this->distance = 10;
    this->pos = focusPoint + glm::dvec3(distance, 0, 0);
    this->forward = glm::normalize(focusPoint - pos);
    this->up = glm::dvec3(0, 0, 1);   // ref up (ref is identity at spawn)
    this->view = glm::translate(pos);
}

glm::dvec3 Camera::orbitOffset() const {
    const double cp = std::cos(orbitPitch), sp = std::sin(orbitPitch);
    const double cy = std::cos(orbitYaw),   sy = std::sin(orbitYaw);
    return glm::dvec3(cp * cy, cp * sy, -sp);
}

void Camera::ComputeView() {
    if (mode == CAM_ORBIT) {
        const glm::dvec3 off = ref * orbitOffset() * distance;
        pos = focusPoint + off;
        forward = glm::normalize(-off);
        // Guard the degenerate projection at the pole (Pitch clamps away
        // from it, but be safe).
        const glm::dvec3 refUp = ref * glm::dvec3(0, 0, 1);
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
    // current standoff. First -> orbit: keep the parked radius/angles -- C
    // must return the framing the cockpit view interrupted.
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
        // Pitch the turntable; clamp short of the pole where up vanishes.
        orbitPitch += angle;
        const double lim = 1.52;   // rad (~87 deg), just short of the pole
        if (orbitPitch > lim) { orbitPitch = lim; }
        if (orbitPitch < -lim) { orbitPitch = -lim; }
    } else if (mode == CAM_FREE) {
        // Rotate the view direction and up around the right axis.
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
        // Yaw: rotate the view direction and right around the up axis.
        const glm::dmat3 rot = glm::dmat3(glm::rotate(angle, up));
        forward = rot * forward;
        right = rot * right;
    }
}

void Camera::Roll(double angle) {
    if (mode != CAM_FREE) { return; }
    // Roll: rotate up and right around the view direction.
    const glm::dmat3 rot = glm::dmat3(glm::rotate(angle, forward));
    up = rot * up;
    right = rot * right;
}

void Camera::setFreePose(const glm::dvec3& p, const glm::dvec3& fwd, const glm::dvec3& upv) {
    this->pos = p;
    this->forward = glm::normalize(fwd);
    // Orthogonalise up against forward so the basis is well-defined.
    this->up = glm::normalize(upv - this->forward * glm::dot(this->forward, upv));
    this->right = glm::normalize(glm::cross(this->forward, this->up));
    this->mode = CAM_FREE;
    this->ComputeView();
}
