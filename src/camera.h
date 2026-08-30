#pragma once

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

// Which way the camera is driven. Orbit keeps a fixed offset (distance) from
// a focus point and lets you look around it; Free is a full 6DOF fly-by.
// One Camera object holds BOTH state sets -- `mode` picks which is live and
// ComputeView() emits the matching view, so switching (the C key) is just a
// mode change instead of copying pose between two objects.
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

    // Origin of the render frame, in world coordinates (e.g. the active
    // ship's COM). ComputeView() builds the view in this frame, so its
    // translation column stays small; geometry drawn against it must be
    // shifted by -renderOrigin to match (see the Draw sites). The whole
    // scene moves rigidly, so the image is unchanged -- the point is that
    // the float32 MVP cast then quantizes ship-relative numbers (metres)
    // instead of planet-centre ones (~1e7, ~0.5 m per quantum).
    glm::dvec3 renderOrigin = glm::dvec3(0.0);

    // Orbit-mode state: a trackball (orient) + distance around the focus.
    // The camera sits at
    //   pos = focus + ref * (orient * (1,0,0)) * distance
    // and looks at the focus with up = ref * (orient * (0,0,1)):
    //  - `ref` is the orientation the camera chases (the caller sets it
    //    every frame: the ship's attitude when focused on the ship, so the
    //    camera turns with the ship -- KSP's chase / Pioneer's sidereal
    //    style -- and the body's rotating frame when focused on a body, so
    //    the camera rides its spin).
    //  - `orient` is the user's accumulated orbit: mouse yaw around the
    //    camera up (the orient basis ẑ), pitch around the camera right
    //    (the orient basis Ŷ) -- RotateY / Pitch below.
    // Up is a fixed camera basis vector (not a radial hint), so it stays
    // continuous as the view passes over the top: the old radial up
    // snapped ~90 degrees there (the "weird" horizon roll) and the lookAt
    // basis NaN'd at the pole.
    glm::dvec3 focusPoint;
    glm::dmat3 orient = glm::dmat3(1.0);
    glm::dmat3 ref = glm::dmat3(1.0);
    double distance = 10.0;

    // Free-mode state. Here pos is PRIMARY (Move* edits it) and right is the
    // derived basis axis (recomputed every ComputeView()).
    glm::dvec3 right;

    // Construct in Orbit mode, focused on focusPos, 10 m out. Use
    // setFreePose() to start (or return) in Free mode instead.
    Camera(const glm::dvec3& focusPos, float fov, float aspect, float zNear, float zFar);

    void ComputeView();

    // Mode transitions (the C key). toFree keeps pos/forward/up as-is
    // (they already hold the live orbit values); toOrbit keeps the
    // orbit's orient as-is and re-derives the distance from the camera's
    // current position, so returning to orbit lands on the same spot on
    // the sphere of radius `distance` around the (new) focus.
    void toFree();
    void toOrbit(const glm::dvec3& focus);

    // Free flight: move the camera along its local axes (no-op in Orbit).
    void MoveForward(double amt);
    void MoveRight(double amt);
    void MoveUp(double amt);

    // Orbit: point at a new focus (no-op in Free).
    void Follow(const glm::dvec3& p);
    // Orbit zoom: distance scales with the wheel (clamped, so it can never
    // cross the focus) (no-op in Free).
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
    // init). Orthogonalises up against forward, matching the old FreeCamera
    // ctor, then recomputes the view.
    void setFreePose(const glm::dvec3& p, const glm::dvec3& fwd, const glm::dvec3& up);

private:
    // Shared view-matrix construction -- the NaN-safe basis that was
    // copy-pasted across the two old subclasses. zAxis is the unit view
    // direction (-forward), upHint the intended up (exact for Orbit, the
    // stored up for Free), and cam the camera position already in the
    // render frame (pos - renderOrigin).
    void buildView(const glm::dvec3& zAxis, const glm::dvec3& upHint, const glm::dvec3& cam);
};
