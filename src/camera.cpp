#include "camera.h"

#include <glm/gtx/polar_coordinates.hpp>

void Camera::setAspect(float _aspect) {
    this->projection = glm::perspective(fov, _aspect, zNear, zFar);
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

OrbitCamera::OrbitCamera(const glm::dvec3& shipPos, float fov, float aspect, float zNear, float zFar) {
    this->fov = fov;
    this->aspect = aspect;
    this->zNear = zNear;
    this->zFar = zFar;
    this->focusPoint = shipPos;
    this->distance = 10;
    this->x = 0;
    this->y = 0;
    this->pos = focusPoint + glm::dvec3(distance, 0, 0);
    this->forward = focusPoint - pos;
    this->up = glm::normalize(pos);
    this->projection = glm::perspective(fov, aspect, zNear, zFar);
    this->view = glm::translate(pos);
    this->orient = glm::dmat3(1.0);
}

void OrbitCamera::wheel(double amt) {
    distance -=  amt * sqrt(distance);
}

void OrbitCamera::ComputeView()
{
    pos = focusPoint + orient * glm::dvec3(distance, 0, 0);
    forward = glm::normalize(focusPoint - pos);

    // Build the camera basis by hand instead of glm::lookAt. lookAt takes
    // our up vector (radial = normalize(pos)) and computes
    // xAxis = normalize(cross(up, zAxis)); the instant the camera is pitched
    // to look straight up or down along that radial, cross(up, zAxis) -> 0
    // and normalize(0) -> NaN, so the whole view matrix becomes NaN and the
    // sun flickers out of view exactly when you point at it. Substituting a
    // safe up only in that degenerate case keeps the (intended) radial up
    // everywhere else, so the view is identical to lookAt except that it
    // stays finite when looking along the radial.
    const glm::dvec3 zAxis = -forward; // unit view direction toward the target

    glm::dvec3 upHint = glm::normalize(pos);
    // TODO is this supposed to fix the rapid flipping when looking directly up or down? Doesn't seem to work.
    if (glm::abs(glm::dot(upHint, zAxis)) > 0.9999) {
        upHint = (std::abs(zAxis.y) < 0.99) ? glm::dvec3(0, 1, 0) : glm::dvec3(1, 0, 0);
    }

    const glm::dvec3 xAxis = glm::normalize(glm::cross(upHint, zAxis));
    const glm::dvec3 yAxis = glm::cross(zAxis, xAxis);
    up = yAxis;

    glm::dmat4 m;
    m[0] = glm::dvec4(xAxis.x, yAxis.x, zAxis.x, 0.0);
    m[1] = glm::dvec4(xAxis.y, yAxis.y, zAxis.y, 0.0);
    m[2] = glm::dvec4(xAxis.z, yAxis.z, zAxis.z, 0.0);
    m[3] = glm::dvec4(-glm::dot(xAxis, pos),
                       -glm::dot(yAxis, pos),
                       -glm::dot(zAxis, pos), 1.0);
    view = m;
}

void OrbitCamera::Follow(const glm::dvec3 p) {
    focusPoint = p;
}

void OrbitCamera::Pitch(double angle) {
    glm::dvec3 right = glm::normalize(glm::cross(forward, up));
    orient = glm::dmat3(glm::rotate(angle, right)) * orient;
}

void OrbitCamera::RotateY(double angle) {
    glm::dvec3 worldUp = glm::normalize(pos); // radial out of planet
    orient = glm::dmat3(glm::rotate(angle, worldUp)) * orient;
}

FreeCamera::FreeCamera(const glm::dvec3& p, const glm::dvec3& fwd, const glm::dvec3& upv,
                       float fov, float aspect, float zNear, float zFar) {
    this->fov = fov;
    this->aspect = aspect;
    this->zNear = zNear;
    this->zFar = zFar;
    this->pos = p;
    this->forward = glm::normalize(fwd);
    // Orthogonalise up against forward so the basis is well-defined.
    this->up = glm::normalize(upv - this->forward * glm::dot(this->forward, upv));
    this->right = glm::normalize(glm::cross(this->forward, this->up));
    // Finite perspective, matching OrbitCamera and Camera::setAspect (resize).
    this->projection = glm::perspective(fov, aspect, zNear, zFar);
    this->view = glm::translate(pos);
    this->ComputeView();
}

void FreeCamera::ComputeView() {
    // Camera basis: local -z is the view direction (forward), local +x is
    // right, local +y is up. Same NaN-safe construction as OrbitCamera.
    const glm::dvec3 zAxis = -forward;

    glm::dvec3 upHint = up;
    if (glm::abs(glm::dot(glm::normalize(upHint), zAxis)) > 0.9999) {
        upHint = (std::abs(zAxis.y) < 0.99) ? glm::dvec3(0, 1, 0) : glm::dvec3(1, 0, 0);
    }

    const glm::dvec3 xAxis = glm::normalize(glm::cross(upHint, zAxis));
    const glm::dvec3 yAxis = glm::cross(zAxis, xAxis);
    up = yAxis;    // keep the stored basis orthonormal
    right = xAxis;

    glm::dmat4 m;
    m[0] = glm::dvec4(xAxis.x, yAxis.x, zAxis.x, 0.0);
    m[1] = glm::dvec4(xAxis.y, yAxis.y, zAxis.y, 0.0);
    m[2] = glm::dvec4(xAxis.z, yAxis.z, zAxis.z, 0.0);
    m[3] = glm::dvec4(-glm::dot(xAxis, pos),
                       -glm::dot(yAxis, pos),
                       -glm::dot(zAxis, pos), 1.0);
    view = m;
}

void FreeCamera::MoveForward(double amt) {
    pos += forward * amt;
}

void FreeCamera::MoveRight(double amt) {
    pos += right * amt;
}

void FreeCamera::MoveUp(double amt) {
    pos += up * amt;
}

void FreeCamera::Pitch(double angle) {
    // Rotate the view direction and up around the right axis.
    const glm::dmat3 rot = glm::dmat3(glm::rotate(angle, right));
    forward = rot * forward;
    up = rot * up;
}

void FreeCamera::RotateY(double angle) {
    // Yaw: rotate the view direction and right around the up axis.
    const glm::dmat3 rot = glm::dmat3(glm::rotate(angle, up));
    forward = rot * forward;
    right = rot * right;
}

void FreeCamera::Roll(double angle) {
    // Roll: rotate up and right around the view direction.
    const glm::dmat3 rot = glm::dmat3(glm::rotate(angle, forward));
    up = rot * up;
    right = rot * right;
}
