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
    // Finite perspective (not infinitePerspective): reverse-Z + a 32-bit depth
    // buffer handle the far plane cleanly, and a finite zFar keeps the
    // projection consistent with Camera::setAspect (used on resize).
    this->projection = glm::perspective(fov, aspect, zNear, zFar);
    this->view = glm::translate(pos);
    this->orient = glm::dmat3();
}

void OrbitCamera::wheel(double amt) {
    distance -=  amt * sqrt(distance);
}

void OrbitCamera::ComputeView()
{
    pos = focusPoint + orient * glm::dvec3(distance, 0, 0);
    forward = glm::normalize(focusPoint - pos);

    // Build the camera basis by hand instead of glm::lookAt. lookAt takes our
    // up vector (radial = normalize(pos)) and computes
    // xAxis = normalize(cross(up, zAxis)); the instant the camera is pitched
    // to look straight up or down along that radial, cross(up, zAxis) -> 0
    // and normalize(0) -> NaN, so the whole view matrix becomes NaN and the
    // far-plane sun disc flickers out of view exactly when you point at the
    // sun. Substituting a safe up only in that degenerate case keeps the
    // (intended) radial up everywhere else, so the view is identical to
    // lookAt except that it stays finite when looking along the radial.
    const glm::dvec3 zAxis = -forward; // unit view direction toward the target

    glm::dvec3 upHint = glm::normalize(pos);
    if (glm::abs(glm::dot(upHint, zAxis)) > 0.9999) {
        // Looking along the radial: any non-parallel world axis works.
        upHint = (std::abs(zAxis.y) < 0.99) ? glm::dvec3(0, 1, 0) : glm::dvec3(1, 0, 0);
    }

    const glm::dvec3 xAxis = glm::normalize(glm::cross(upHint, zAxis));
    const glm::dvec3 yAxis = glm::cross(zAxis, xAxis);
    up = yAxis; // the camera's actual up direction

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
    // 1.  Re-compute current basis from the *current* orient
    glm::dvec3 right = glm::normalize(glm::cross(forward, up));

    // 2.  Rotate around that right axis in WORLD space
    orient = glm::dmat3(glm::rotate(angle, right)) * orient;
}

void OrbitCamera::RotateY(double angle) {
    // 1.  World-space up at the focus point is always POS normalised
    glm::dvec3 worldUp = glm::normalize(pos);   // radial out of planet

    // 2.  Rotate around that up axis in WORLD space
    orient = glm::dmat3(glm::rotate(angle, worldUp)) * orient;
}

WeirdCamera::WeirdCamera(const glm::vec3& pos, float fov, float aspect, float zNear, float zFar) {
    this->fov = fov;
    this->aspect = aspect;
    this->zNear = zNear;
    this->zFar = zFar;
    this->pos = pos;
    this->forward = glm::vec3(0.0f, 0.0f, 1.0f);
    this->up = glm::vec3(0.0f, 1.0f, 0.0f);
    this->projection = glm::perspective(fov, aspect, zNear, zFar);
    this->view = glm::translate(pos);
}

void WeirdCamera::ComputeView() {
    view = glm::lookAt(pos, pos + forward, up);
}
void WeirdCamera::Follow(const glm::dvec3 p) {
    pos = p - glm::dvec3(-15, 0, 0);
}

void WeirdCamera::MoveForward(double amt) {
    pos += forward * amt;
}

void WeirdCamera::MoveRight(double amt) {
    pos += glm::cross(up, forward) * amt;
}

void WeirdCamera::Pitch(double angle) {
    glm::dvec3 right = glm::normalize(glm::cross(up, forward));

    forward = glm::dvec3(glm::normalize(glm::rotate(angle, right) * glm::vec4(forward, 0.0)));
    up = glm::normalize(glm::cross(forward, right));
}

void WeirdCamera::RotateY(double angle) {
    static const glm::dvec3 UP(0.0f, -1.0f, 0.0f);

    glm::mat4 rotation = glm::rotate(angle, UP);

    forward = glm::dvec3(glm::normalize(rotation * glm::dvec4(forward, 0.0)));
    up = glm::dvec3(glm::normalize(rotation * glm::vec4(up, 0.0)));
}
