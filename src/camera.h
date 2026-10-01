#pragma once

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

// Orbit keeps a fixed offset from a focus; Free is full 6DOF. One object
// holds both state sets -- `mode` picks which is live.
enum CameraMode { CAM_ORBIT, CAM_FREE };

class Camera {
public:
    CameraMode mode = CAM_ORBIT;

    glm::dmat4 view = glm::dmat4(1.0);
    glm::mat4 projection = glm::mat4(1.0);
    glm::dvec3 pos;
    glm::dvec3 forward;
    glm::dvec3 up;
    float fov, aspect, zNear, zFar;
    int viewport_w = 1;   // window size [px]; setViewport() on create/resize
    int viewport_h = 1;   // used for screen-space terrain LOD (GeoPatch::Update)

    // Origin of the render frame in world coords (e.g. the active ship's
    // COM). The view is built in this frame so the float32 MVP cast
    // quantizes ship-relative numbers, not planet-scale ones. Geometry drawn
    // against it must be shifted by -renderOrigin (see the Draw sites).
    glm::dvec3 renderOrigin = glm::dvec3(0.0);

    // Orbit-mode state: a turntable + distance around the focus.
    //   pos = focus + ref * offset(yaw, pitch) * distance
    // `ref` is the orientation the caller sets every frame (ship attitude /
    // body spin), so the camera chases it. Up is the ref up projected off
    // the view -- path-independent (no trackball holonomy); the cost is the
    // usual orbit-camera pole, which Pitch() clamps short of.
    glm::dvec3 focusPoint;
    glm::dmat3 ref = glm::dmat3(1.0);
    double distance = 10.0;
    double orbitYaw = 0.0;     // ref-frame azimuth, rad (0 = ref x̂)
    double orbitPitch = 0.0;   // ref-frame elevation, rad (clamped near pole)

    // Free-mode state. pos is PRIMARY; right is derived each ComputeView().
    glm::dvec3 right;

    // Construct in Orbit mode, focused on focusPos, 10 m out.
    Camera(const glm::dvec3& focusPos, float fov, float aspect, float zNear, float zFar);

    void ComputeView();

    // Mode transitions (the C key). toFree keeps the live pose; toOrbit
    // re-derives distance from the current position around the new focus.
    void toFree();
    void toOrbit(const glm::dvec3& focus);

    // Free flight: move along local axes (no-op in Orbit).
    void MoveForward(double amt);
    void MoveRight(double amt);
    void MoveUp(double amt);

    // Orbit: point at a new focus (no-op in Free).
    void Follow(const glm::dvec3& p);
    // Orbit zoom (clamped so it can never cross the focus) (no-op in Free).
    void wheel(double amt);

    // Look controls, valid in both modes.
    void Pitch(double angle);
    void RotateY(double angle);
    void Roll(double angle);

    void setAspect(float _aspect);
    void setViewport(int w, int h);
    void setFov(float _fov);
    const glm::dvec3& GetPos() const;
    const glm::dvec3& GetRenderOrigin() const { return renderOrigin; }
    const glm::dvec3& GetForward() const;
    glm::mat4 GetProjection() const;
    glm::dmat4 GetView() const;
    glm::dmat4 *GetView_();

    // Start (or return) to Free mode at an explicit pose (the --free-cam-*
    // init). Orthogonalises up against forward, then recomputes the view.
    void setFreePose(const glm::dvec3& p, const glm::dvec3& fwd, const glm::dvec3& up);

private:
    // Unit focus->camera offset in the ref frame, from the turntable angles.
    // Pitch positive toward ref -ẑ so Pitch()/RotateY() keep their old feel.
    glm::dvec3 orbitOffset() const;

    // Shared view-matrix construction (NaN-safe when looking along up).
    // cam is the camera position in the render frame; Orbit passes the more
    // exact (focusPoint - renderOrigin) + off.
    void buildView(const glm::dvec3& zAxis, const glm::dvec3& upHint, const glm::dvec3& cam);
};
