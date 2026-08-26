#pragma once

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

class Camera {
public:
    virtual ~Camera() {}

    glm::dmat4 view = glm::dmat4(1.0);
    glm::mat4 projection = glm::mat4(1.0);
    glm::dvec3 pos;
    glm::dvec3 forward;
    glm::dvec3 up;
    float fov, aspect, zNear, zFar;

    // Origin of the render frame, in world coordinates (e.g. the active
    // ship's COM). ComputeView() builds the view in this frame, so its
    // translation column stays small; geometry drawn against it must be
    // shifted by -renderOrigin to match (see the Draw sites). The whole
    // scene moves rigidly, so the image is unchanged -- the point is that
    // the float32 MVP cast then quantizes ship-relative numbers (metres)
    // instead of planet-centre ones (~1e7, ~0.5 m per quantum).
    glm::dvec3 renderOrigin = glm::dvec3(0.0);

    virtual void ComputeView() {}
    virtual void Follow(const glm::dvec3 p) {}
    virtual void MoveForward(double amt) {}
    virtual void MoveRight(double amt) {}
    virtual void MoveUp(double amt) {}
    virtual void Pitch(double angle) {}
    virtual void RotateY(double angle) {}
    virtual void Roll(double angle) {}
    virtual void wheel(double amt) {}

    void setAspect(float _aspect);
    const glm::dvec3& GetPos() const;
    const glm::dvec3& GetRenderOrigin() const { return renderOrigin; }
    const glm::dvec3& GetForward() const;
    glm::mat4 GetProjection() const;
    glm::dmat4 GetView() const;
    glm::dmat4 *GetView_();
};

// Body-orbit camera: keeps a fixed offset (distance) from a focus point and
// lets you look around it. The focus point is set each frame with Follow()
// (the ship, or whichever body is selected).
class OrbitCamera : public Camera {
public:
    glm::dvec3 focusPoint;
    glm::dmat3 orient = glm::dmat3(1.0); // GLM 1.0.0+: default mat ctor is zero
    double x, y;
    double distance;

    OrbitCamera(const glm::dvec3& shipPos, float fov, float aspect, float zNear, float zFar);

    void ComputeView() override;
    void Follow(const glm::dvec3 p) override;

    void MoveForward(double amt) override { }
    void MoveRight(double amt) override { }
    void MoveUp(double amt) override { }
    void Roll(double angle) override { }

    void Pitch(double angle) override;
    void RotateY(double angle) override;

    void wheel(double amt) override;
};

// Free-flight camera: a full 6DOF camera. pos/forward/up describe the
// camera in world (ship-frame) coordinates.
class FreeCamera : public Camera {
public:
    glm::dvec3 right;

    FreeCamera(const glm::dvec3& pos, const glm::dvec3& forward, const glm::dvec3& up,
               float fov, float aspect, float zNear, float zFar);

    void ComputeView() override;
    void Follow(const glm::dvec3 p) override { }

    void MoveForward(double amt) override;
    void MoveRight(double amt) override;
    void MoveUp(double amt) override;

    void Pitch(double angle) override;
    void RotateY(double angle) override;
    void Roll(double angle) override;

    void wheel(double amt) override { }
};
