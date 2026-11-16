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

    // Pixels of RMB drag per radian of orbit. Named because PanPx() converts
    // its own drag from the screen's scale, and the two rates belong next to
    // each other in one header (#919).
    static constexpr double kCamOrbitPx = 200.0;

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

    /* Aim offset, rad: how far the LOOK direction is rotated off the line to
       the focus -- yaw about the un-panned view up, then pitch about its right.
       The position stays exactly where orbitYaw/orbitPitch/distance put it, so
       the subject holds its PIXEL offset in the frame while the player keeps
       orbiting and zooming: this is framing, not a second camera (#919).
       Clamped to the frame edge on each axis (panLimitX/Y), so the subject can
       go to the border but never off-screen where the player can't find it
       again, and inside a circle so the two corners together can't spin the
       look line past sideways. MMB drags it, a tap zeros it. */
    double panYaw = 0.0, panPitch = 0.0;

    /* How far the pan circle reaches, rad. The camera looks AT the focus, so
       the focus is on the view line; rotating the view by more than the
       vertical half-angle drops it below the frame, and past 90 deg puts it
       BEHIND the camera -- aim= would then read a mirror image of where the
       ship is. Both stops are live: fov is a slider and --fov, aspect changes
       on resize, so ComputeView() re-clamps every frame rather than leaving a
       pan the current lens no longer shows. */
    double panLimitY() const { return (double)fov * 0.5; }
    double panLimitX() const { return std::atan(std::tan((double)fov * 0.5) * (double)aspect); }
    // The composed guard: past this the two axis clamps together could leave
    // the focus near or behind the camera. fov maxes at 120 deg, where the
    // diagonal reaches 1.19 rad, so this only bites at the widest lenses.
    static constexpr double kPanCircleLimit = 1.2;

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

    // Aim offset in screen pixels (Orbit only, no-op in Free/First). Dragging
    // right puts the subject right, dragging down puts it down: the image
    // follows the cursor.
    void PanPx(double dxPx, double dyPx);
    // Back to the focus dead centre.
    void resetPan();

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

    // Hold panYaw/panPitch inside the frame-edge stops above, then inside the
    // composed circle. PanPx() calls it and ComputeView() calls it, so an fov
    // change (Settings slider, --fov) or a resize cannot leave a pan that the
    // current view no longer shows.
    void clampPan();

    // The screen's own px-per-radian at the focus, so an MMB drag carries the
    // subject with the cursor instead of at a made-up rate (#919).
    double panPxPerRad() const;

    // Refresh frustumSinX/CosX/SinY/CosY from fov and aspect (call whenever
    // either changes -- the projection is rebuilt at the same sites).
    void computeFrustumAngles();

    // Shared view-matrix construction (NaN-safe when looking along up). cam is
    // the camera position in the render frame.
    void buildView(const glm::dvec3& zAxis, const glm::dvec3& upHint, const glm::dvec3& cam);
};
