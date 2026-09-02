#pragma once

// evamath.h -- pure EVA control math (glm only, no GL/Bullet), so the
// control-law geometry can be unit-tested headless (the same split as
// shipdef.h / terragen.h). The Kerbal that consumes these lives in eva.h.

#include <cmath>

// glm::length2 lives in gtx/norm; the experimental opt-in must precede the
// include (this header is self-contained: tests include it alone).
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

/* Rotation of `ang` radians about the unit axis `a` (Rodrigues). */
static inline glm::dmat3 rotAbout(const glm::dvec3 &a, double ang) {
    const double c = cos(ang), s = sin(ang), t = 1.0 - c;
    // glm::dmat3(cols...): column k holds M(row, k)
    return glm::dmat3(
        glm::dvec3(c + t*a.x*a.x,     t*a.x*a.y + s*a.z,  t*a.x*a.z - s*a.y),
        glm::dvec3(t*a.x*a.y - s*a.z, c + t*a.y*a.y,      t*a.y*a.z + s*a.x),
        glm::dvec3(t*a.x*a.z + s*a.y, t*a.y*a.z - s*a.x,  c + t*a.z*a.z));
}

/* v with the component along unit n removed (projection onto the plane
   perpendicular to n). */
static inline glm::dvec3 evaOntoPlane(const glm::dvec3 &v, const glm::dvec3 &n) {
    return v - n * glm::dot(v, n);
}

/* Orthonormal camera basis as a ship-convention attitude matrix (columns
   [right, up, nose], det +1): nose (local +Z) = the camera's forward,
   up = the camera's up off the forward. The kerbal's own right axis is
   then the MIRROR of the screen right (it faces away down the view), the
   same mirroring a ship has when seen from the front. */
static inline glm::dmat3 evaCamBasis(const glm::dvec3 &fwdIn, const glm::dvec3 &upIn) {
    const glm::dvec3 fwd = glm::normalize(fwdIn);
    glm::dvec3 up = evaOntoPlane(glm::normalize(upIn), fwd);
    if(glm::length2(up) < 1e-12) {
        const glm::dvec3 ref = (std::fabs(fwd.y) > 0.9)
            ? glm::dvec3(1.0, 0.0, 0.0) : glm::dvec3(0.0, 1.0, 0.0);
        up = glm::normalize(evaOntoPlane(ref, fwd));
    } else {
        up = glm::normalize(up);
    }
    const glm::dvec3 right = glm::cross(up, fwd);
    return glm::dmat3(right, up, fwd);
}

/* The screen-right direction for a camera looking `fwd` with `up` up
   (the direction D should strafe). Perpendicular to both, unit. */
static inline glm::dvec3 evaScreenRight(const glm::dvec3 &fwd, const glm::dvec3 &up) {
    glm::dvec3 r = glm::cross(glm::normalize(fwd), glm::normalize(up));
    if(glm::length2(r) < 1e-12) {
        const glm::dvec3 ref = (std::fabs(fwd.y) > 0.9)
            ? glm::dvec3(1.0, 0.0, 0.0) : glm::dvec3(0.0, 1.0, 0.0);
        r = glm::cross(glm::normalize(fwd), ref);
    }
    return glm::normalize(r);
}

/* Target attitude for a standing kerbal: the cucumber's long axis
   (the part's nose, column 2) = local vertical, and the "face"
   (column 1) = faceHint projected into the tangent plane (any tangent
   direction when the hint is degenerate, e.g. straight up). The walk /
   camera direction is what the kerbal FACES, not what it stands along. */
static inline glm::dmat3 evaStandTarget(glm::dvec3 nose, glm::dvec3 faceHint) {
    nose = glm::normalize(nose);
    glm::dvec3 face = evaOntoPlane(faceHint, nose);
    if(glm::length2(face) < 1e-12) {
        const glm::dvec3 ref = (std::fabs(nose.y) > 0.9)
            ? glm::dvec3(1.0, 0.0, 0.0) : glm::dvec3(0.0, 1.0, 0.0);
        face = evaOntoPlane(ref, nose);
    }
    face = glm::normalize(face);
    const glm::dvec3 right = glm::cross(face, nose);
    return glm::dmat3(right, face, nose);
}

/* Target attitude in space: the cucumber stands "upright" on screen
   (long axis along the camera up) with its face toward the viewer
   (the kerbal faces the camera direction), built from the camera basis
   [right, up, fwd]: columns [right, -fwd, up]. */
static inline glm::dmat3 evaSpaceTarget(const glm::dmat3 &camBasis) {
    return glm::dmat3(camBasis[0], -camBasis[2], camBasis[1]);
}

/* Axis-angle decomposition of a rotation matrix: the angle (0..pi) and
   the unit axis such that rotAbout(axis, angle) == R (axis = the zero
   vector when the angle ~ 0). At ~180 deg the antisymmetric part
   vanishes, so the axis falls back to the symmetric part. */
static inline double evaRotAxisAngle(const glm::dmat3 &R, glm::dvec3 &axis) {
    const double tr = R[0][0] + R[1][1] + R[2][2];
    const double ang = glm::acos(glm::clamp((tr - 1.0) * 0.5, -1.0, 1.0));
    if(ang < 1e-9) { axis = glm::dvec3(0.0); return ang; }
    // M(row,col) == glm's R[col][row]; axis = (M21-M12, M02-M20, M10-M01)
    const glm::dvec3 v(R[1][2] - R[2][1],
                       R[2][0] - R[0][2],
                       R[0][1] - R[1][0]);
    if(glm::length2(v) < 1e-12) {
        // near 180 deg: axis from the diagonal of (R + I) / 2; the sign
        // convention is arbitrary there (rotAbout(a, pi) == rotAbout(-a, pi))
        glm::dvec3 a(std::sqrt(glm::max(0.0, (R[0][0] + 1.0) / 2.0)),
                     std::sqrt(glm::max(0.0, (R[1][1] + 1.0) / 2.0)),
                     std::sqrt(glm::max(0.0, (R[2][2] + 1.0) / 2.0)));
        if(a.x >= a.y && a.x >= a.z) {
            if(R[1][0] + R[0][1] < 0.0) { a.y = -a.y; }
            if(R[0][2] + R[2][0] < 0.0) { a.z = -a.z; }
        } else if(a.y >= a.z) {
            if(R[1][0] + R[0][1] < 0.0) { a.x = -a.x; }
            if(R[2][1] + R[1][2] < 0.0) { a.z = -a.z; }
        } else {
            if(R[0][2] + R[2][0] < 0.0) { a.x = -a.x; }
            if(R[2][1] + R[1][2] < 0.0) { a.y = -a.y; }
        }
        axis = glm::normalize(a);
    } else {
        axis = glm::normalize(v);
    }
    return ang;
}
