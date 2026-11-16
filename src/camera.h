#pragma once

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

// Orbit: fixed offset from a focus. Free: full 6DOF. First: rigid eye on the
// ship's controller (pose set each frame). `mode` picks which state set is live.
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
       rebuilt whenever the projection is. Cull test, with (xv,yv,zv) in VIEW
       space (zv = depth in front of the camera): inside the side planes iff
         zv*frustumSinX - fabs(xv)*frustumCosX >= 0   (same for Y)
       and a bounding sphere of radius r is not provably outside iff that
       quantity is >= -r. No far plane: the projection is reverse-Z
       infinite-far (camera.cpp). */
    double frustumSinX = 1.0, frustumCosX = 0.0;
    double frustumSinY = 1.0, frustumCosY = 0.0;

    // Origin of the render frame in world coords (e.g. the active ship's COM):
    // the float32 MVP cast then quantizes ship-relative numbers, not
    // planet-scale ones. Geometry drawn against it must be shifted by
    // -renderOrigin.
    glm::dvec3 renderOrigin = glm::dvec3(0.0);

    // Orbit state: pos = focus + ref * offset(yaw, pitch) * distance. `ref` is
    // the orientation the caller sets every frame (ship attitude / body spin).
    // Up is the ref up projected off the view -- path-independent (no trackball
    // holonomy); the cost is the usual orbit-camera pole, which Pitch() clamps
    // short of.
    glm::dvec3 focusPoint;
    glm::dmat3 ref = glm::dmat3(1.0);
    double distance = 10.0;
    double orbitYaw = 0.0;     // ref-frame azimuth, rad (0 = ref x̂)
    double orbitPitch = 0.0;   // ref-frame elevation, rad (clamped near pole)

    // How far Pitch() may go from ref's plane, rad (~87 deg). Named because
    // #876's --cam-log prints whether this clamp is holding, and the log must
    // not restate the literal to say so.
    static constexpr double kPitchLimit = 1.52;

    /* The +side stop, rad: how far the camera may go BELOW ref's plane
       (positive orbitPitch rides toward ref -ẑ). Defaults to kPitchLimit, which
       is right while the hull holds the frame -- there the stop is the pole,
       where the projected up vanishes. A ground-locked frame instead lowers it
       to the deck the ship stands on, so panning below the horizon gives a low
       view and not an underground one (#876).
       The caller writes it every frame next to ref, and ComputeView() ENFORCES
       it, not just Pitch(): a stop that moves under a pitch already at the old
       one -- zooming out at the pad, or scene.cpp popping a parked pitch -- has
       to pull the view back, or the turntable keeps drawing underground. The
       -side stop stays at the pole. */
    double downLimit = kPitchLimit;

    // Free-mode state. pos is PRIMARY; right is derived each ComputeView().
    glm::dvec3 right;

    // Construct in Orbit mode, focused on focusPos, 10 m out.
    Camera(const glm::dvec3& focusPos, float fov, float aspect, float zNear, float zFar);

    void ComputeView();

    // Mode transitions (C / Shift-C). toFree keeps the live pose; toFirst leaves
    // it alone (the caller re-poses it every frame). toOrbit re-derives distance
    // from the live pose out of Free but restores the parked turntable state out
    // of First (C is a true toggle).
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
    // The live +side stop, rad: what clampOrbitPitch() holds orbitPitch to and
    // what #876's cam-log prints as hi= beside pin=.
    double orbitHiStop() const;
    const glm::dvec3& GetPos() const;
    const glm::dvec3& GetRenderOrigin() const { return renderOrigin; }
    const glm::dvec3& GetForward() const;
    // By reference: the draw cull and the per-leaf MVP call these once per node.
    const glm::mat4 &GetProjection() const;
    const glm::dmat4 &GetView() const;

    // Enter (or return to) Free at an explicit pose (the --free-cam-* init).
    // Orthogonalises up against forward, then recomputes the view.
    void setFreePose(const glm::dvec3& p, const glm::dvec3& fwd, const glm::dvec3& up);

private:
    // Unit focus->camera offset in the ref frame. Pitch positive toward ref -ẑ.
    glm::dvec3 orbitOffset() const;

    // Hold orbitPitch inside [-kPitchLimit, orbitHiStop()]. Pitch() calls it and
    // ComputeView() calls it, so a stop that moved since the last drag cannot
    // leave the view outside it.
    void clampOrbitPitch();

    // Refresh frustumSinX/CosX/SinY/CosY from fov and aspect (call whenever
    // either changes -- the projection is rebuilt at the same sites).
    void computeFrustumAngles();

    // Shared view-matrix construction (NaN-safe when looking along up). cam is
    // the camera position in the render frame.
    void buildView(const glm::dvec3& zAxis, const glm::dvec3& upHint, const glm::dvec3& cam);
};
