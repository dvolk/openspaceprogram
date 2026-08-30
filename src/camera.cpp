#include "camera.h"

#include <glm/gtx/polar_coordinates.hpp>

void Camera::setAspect(float _aspect) {
    this->projection = glm::perspective(fov, _aspect, zNear, zFar);
}

void Camera::setViewport(int w, int h) {
    this->viewport_w = w;
    this->viewport_h = h;
}

void Camera::setFov(float _fov) {
    this->fov = _fov;
    this->projection = glm::perspective(fov, aspect, zNear, zFar);
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
    return view;//glm::lookAt(pos, pos + forward, up);
}

glm::dmat4 *Camera::GetView_() {
    return &view;
}

Camera::Camera(const glm::dvec3& focusPos, float fov, float aspect, float zNear, float zFar)
    : fov(fov), aspect(aspect), zNear(zNear), zFar(zFar) {
    this->projection = glm::perspective(fov, aspect, zNear, zFar);
    // Start in Orbit mode focused on focusPos, 10 m out (the old OrbitCamera
    // ctor). pos/forward/up are derived here once so a caller that reads
    // them before the first ComputeView() gets sensible values.
    this->mode = CAM_ORBIT;
    this->focusPoint = focusPos;
    this->distance = 10;
    this->orient = glm::dmat3(1.0);
    this->pos = focusPoint + glm::dvec3(distance, 0, 0);
    this->forward = focusPoint - pos;
    this->up = glm::normalize(pos);
    this->view = glm::translate(pos);
}

void Camera::ComputeView() {
    if (mode == CAM_ORBIT) {
        pos = focusPoint + orient * glm::dvec3(distance, 0, 0);
        forward = glm::normalize(focusPoint - pos);
        // Radial up (out of the planet): the old OrbitCamera's upHint.
        buildView(-forward, glm::normalize(pos), pos - renderOrigin);
        return;
    }
    // Free: pos is primary; up is the stored free-camera up.
    buildView(-forward, up, pos - renderOrigin);
}

void Camera::buildView(const glm::dvec3& zAxis, const glm::dvec3& upHintIn, const glm::dvec3& cam) {
    // Build the camera basis by hand instead of glm::lookAt. lookAt takes
    // our up vector and computes xAxis = normalize(cross(up, zAxis)); the
    // instant the camera is pitched to look straight up or down along that
    // up, cross(up, zAxis) -> 0 and normalize(0) -> NaN, so the whole view
    // matrix becomes NaN and the sun flickers out of view exactly when you
    // point at it. Substituting a safe up only in that degenerate case keeps
    // the (intended) up everywhere else, so the view is identical to lookAt
    // except that it stays finite when looking along the up.
    glm::dvec3 upHint = glm::normalize(upHintIn);
    if (glm::abs(glm::dot(upHint, zAxis)) > 0.9999) {
        upHint = (std::abs(zAxis.y) < 0.99) ? glm::dvec3(0, 1, 0) : glm::dvec3(1, 0, 0);
    }
    const glm::dvec3 xAxis = glm::normalize(glm::cross(upHint, zAxis));
    const glm::dvec3 yAxis = glm::cross(zAxis, xAxis);
    up = yAxis;     // keep the stored basis orthonormal
    right = xAxis;  // free-mode basis axis (harmless in Orbit mode)

    // View translation in the render frame (origin = renderOrigin): the
    // planet centre sits at -renderOrigin there, so a radial upHint
    // (world normalize(pos)) is exactly normalize(cam - (-renderOrigin)).
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
    // Keep the current view: pos/forward/up already hold the live orbit
    // values (same object now). Just derive the free basis axis.
    right = glm::normalize(glm::cross(forward, up));
    mode = CAM_FREE;
}

void Camera::toOrbit(const glm::dvec3& focus) {
    if (mode == CAM_ORBIT) { Follow(focus); return; }
    focusPoint = focus;
    double dist = glm::length(pos - focus);
    if (dist < 10.0) { dist = 10.0; }
    distance = dist;
    // orient is left as-is (stale from the last orbit session) -- matches
    // the old behavior where the orbit camera's orient persisted across a
    // free detour, so returning to orbit lands on that same spot on the
    // sphere of radius `distance` around the new focus.
    mode = CAM_ORBIT;
}

void Camera::Follow(const glm::dvec3& p) {
    if (mode != CAM_ORBIT) { return; }
    focusPoint = p;
}

void Camera::wheel(double amt) {
    if (mode != CAM_ORBIT) { return; }
    distance -=  amt * sqrt(distance);
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
        glm::dvec3 right = glm::normalize(glm::cross(forward, up));
        orient = glm::dmat3(glm::rotate(angle, right)) * orient;
    } else {
        // Rotate the view direction and up around the right axis.
        const glm::dmat3 rot = glm::dmat3(glm::rotate(angle, right));
        forward = rot * forward;
        up = rot * up;
    }
}

void Camera::RotateY(double angle) {
    if (mode == CAM_ORBIT) {
        glm::dvec3 worldUp = glm::normalize(pos); // radial out of planet
        orient = glm::dmat3(glm::rotate(angle, worldUp)) * orient;
    } else {
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
