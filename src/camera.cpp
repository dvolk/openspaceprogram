#include "camera.h"

#include <cmath>

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
    return view;
}

glm::dmat4 *Camera::GetView_() {
    return &view;
}

Camera::Camera(const glm::dvec3& focusPos, float fov, float aspect, float zNear, float zFar)
    : fov(fov), aspect(aspect), zNear(zNear), zFar(zFar) {
    this->projection = glm::perspective(fov, aspect, zNear, zFar);
    // Start in Orbit mode focused on focusPos, 10 m out along the ref
    // basis X̂ (the caller sets `ref` every frame; identity at spawn).
    // pos/forward/up are derived here once so a caller that reads them
    // before the first ComputeView() gets sensible values.
    this->mode = CAM_ORBIT;
    this->focusPoint = focusPos;
    this->distance = 10;
    this->pos = focusPoint + glm::dvec3(distance, 0, 0);
    this->forward = glm::normalize(focusPoint - pos);
    this->up = glm::normalize(pos);
    this->view = glm::translate(pos);
}

void Camera::ComputeView() {
    if (mode == CAM_ORBIT) {
        if (clamp_above_horizon) {
            // Keep the camera above the local horizon (the body's radial --
            // the body centre is the origin of the render frame): pitch it
            // up to 5 degrees if it has dropped below. The pitch is a
            // rotation d around the camera right (the orient basis Ŷ),
            // which moves the offset (basis X̂) to
            //   off' = off*cos(d) - up_cam*sin(d)
            // so the elevation condition is A*cos(d) - B*sin(d) = sin(5deg)
            // with A = off.lup, B = up_cam.lup; solve the two candidate
            // roots and take the smaller motion.
            const glm::dmat3 R = ref * orient;
            const glm::dvec3 offw = R * glm::dvec3(1, 0, 0);
            const glm::dvec3 upw = R * glm::dvec3(0, 0, 1);
            const glm::dvec3 lup = glm::normalize(focusPoint);
            const double A = glm::dot(offw, lup);
            const double B = glm::dot(upw, lup);
            const double cMin = std::sin(glm::radians(5.0));
            if (A < cMin) {
                const double M = std::sqrt(A * A + B * B);
                const double phi = std::atan2(B, A);
                double d;
                if (M < cMin) {
                    d = -phi;   // no exact root; best effort (degenerate)
                } else {
                    const double a = std::acos(cMin / M);
                    const double d1 = a - phi;
                    const double d2 = -a - phi;
                    d = (std::abs(d1) <= std::abs(d2)) ? d1 : d2;
                }
                orient = glm::transpose(ref)
                       * glm::dmat3(glm::rotate(d, R * glm::dvec3(0, 1, 0)))
                       * ref * orient;
            }
        }
        pos = focusPoint + ref * (orient * glm::dvec3(1, 0, 0)) * distance;
        forward = glm::normalize(focusPoint - pos);
        buildView(-forward, ref * (orient * glm::dvec3(0, 0, 1)), pos - renderOrigin);
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
    // orient is left as-is (stale from the last orbit session) -- the
    // orbit camera's orient persists across a free detour, so returning
    // to orbit lands on that same spot on the sphere of radius `distance`
    // around the new focus.
    mode = CAM_ORBIT;
}

void Camera::Follow(const glm::dvec3& p) {
    if (mode != CAM_ORBIT) { return; }
    focusPoint = p;
}

void Camera::wheel(double amt) {
    if (mode != CAM_ORBIT) { return; }
    // Proportional zoom (a notch always changes distance by the same
    // fraction), clamped so the camera can never cross the focus.
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
        if (clamp_above_horizon) {
            // Landed: stop the camera at the horizon instead of letting
            // it keep rotating under the ship (that would flip its up
            // and show the ship upside down). The pitch by `angle`
            // (around the camera right) sends the offset's elevation
            // from sin = A to A*cos(angle) - B*sin(angle) with
            // A = off.lup, B = up_cam.lup; if that drops below sin(5deg)
            // take the closest root that keeps it there (absorb the rest).
            const glm::dmat3 R = ref * orient;
            const glm::dvec3 offw = R * glm::dvec3(1, 0, 0);
            const glm::dvec3 upw = R * glm::dvec3(0, 0, 1);
            const glm::dvec3 lup = glm::normalize(focusPoint);
            const double A = glm::dot(offw, lup);
            const double B = glm::dot(upw, lup);
            const double cMin = std::sin(glm::radians(5.0));
            if (A * std::cos(angle) - B * std::sin(angle) < cMin) {
                const double M = std::sqrt(A * A + B * B);
                if (M < cMin) {
                    angle = 0.0;   // no root; absorb entirely
                } else {
                    const double phi = std::atan2(B, A);
                    const double a = std::acos(cMin / M);
                    const double d1 = a - phi;
                    const double d2 = -a - phi;
                    angle = (std::abs(d1) <= std::abs(d2)) ? d1 : d2;
                }
            }
        }
        // Pitch around the camera right (the orient basis Ŷ): the camera
        // swings over the top / under the bottom of the focus. Up (the
        // orient basis ẑ) turns with it, so it stays continuous through
        // the top -- no horizon roll.
        orient = glm::dmat3(glm::rotate(angle, orient * glm::dvec3(0, 1, 0))) * orient;
    } else {
        // Rotate the view direction and up around the right axis.
        const glm::dmat3 rot = glm::dmat3(glm::rotate(angle, right));
        forward = rot * forward;
        up = rot * up;
    }
}

void Camera::RotateY(double angle) {
    if (mode == CAM_ORBIT) {
        // Yaw around the camera up (the orient basis ẑ): a turntable
        // around the camera's own vertical, which chases the ship's
        // attitude in flight (ref * ẑ) and is the world vertical on the
        // pad.
        orient = glm::dmat3(glm::rotate(angle, orient * glm::dvec3(0, 0, 1))) * orient;
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
