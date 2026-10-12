#pragma once

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

// Orbit keeps a fixed offset from a focus; Free is full 6DOF; First is a
// rigid eye on the ship's controller (pose set each frame). One object holds
// the state sets -- `mode` picks which is live.
enum CameraMode { CAM_ORBIT, CAM_FREE, CAM_FIRST };

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

    /* Frustum half-angle sines/cosines (vertical fov, horizontal = fov*aspect),
       rebuilt whenever the projection is. For the draw-pass culling test:
       with (xv,yv,zv) a point in VIEW space (zv = depth in front of the
       camera), it is inside the side planes iff
         zv*frustumSinX - fabs(xv)*frustumCosX >= 0   (same for Y)
       and a bounding sphere of radius r is not provably outside iff that
       quantity is >= -r. There is no far plane: the projection is
       reverse-Z infinite-far (camera.cpp). */
    double frustumSinX = 1.0, frustumCosX = 0.0;
    double frustumSinY = 1.0, frustumCosY = 0.0;

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

    // Mode transitions (C / Shift-C). toFree keeps the live pose; toFirst
    // leaves it alone (the caller re-poses it every frame). toOrbit
    // re-derives distance from the live pose out of Free, but restores the
    // parked turntable state out of First (C is a true toggle).
    void toFree();
    void toOrbit(const glm::dvec3& focus);
    void toFirst();

    // Free flight: move along local axes (no-op in Orbit).
    void MoveForward(double amt);
    void MoveRight(double amt);
    void MoveUp(double amt);

    // Orbit: point at a new focus (no-op in Free).
    void Follow(const glm::dvec3& p);
    // Orbit zoom (clamped so it can never cross the focus) (no-op in Free).
    void wheel(double amt);

    // Look controls (Orbit turntable / Free 6DOF). No-op in First: the view
    // is locked to the ship axis.
    void Pitch(double angle);
    void RotateY(double angle);
    void Roll(double angle);

    void setAspect(float _aspect);
    void setViewport(int w, int h);
    void setFov(float _fov);
    const glm::dvec3& GetPos() const;
    const glm::dvec3& GetRenderOrigin() const { return renderOrigin; }
    const glm::dvec3& GetForward() const;
    // Both by reference: the draw-pass cull reads GetView once per visited
    // terrain patch and the per-leaf MVP reads GetProjection once per leaf, so
    // by value was a 64-128 byte copy per node.
    const glm::mat4 &GetProjection() const;
    const glm::dmat4 &GetView() const;

    // Start (or return) to Free mode at an explicit pose (the --free-cam-*
    // init). Orthogonalises up against forward, then recomputes the view.
    void setFreePose(const glm::dvec3& p, const glm::dvec3& fwd, const glm::dvec3& up);

private:
    // Unit focus->camera offset in the ref frame, from the turntable angles.
    // Pitch positive toward ref -ẑ so Pitch()/RotateY() keep their old feel.
    glm::dvec3 orbitOffset() const;

    // Refresh frustumSinX/CosX/SinY/CosY from fov and aspect (call whenever
    // either changes -- the projection is rebuilt at the same sites).
    void computeFrustumAngles();

    // Shared view-matrix construction (NaN-safe when looking along up).
    // cam is the camera position in the render frame; Orbit passes the more
    // exact (focusPoint - renderOrigin) + off.
    void buildView(const glm::dvec3& zAxis, const glm::dvec3& upHint, const glm::dvec3& cam);
};
