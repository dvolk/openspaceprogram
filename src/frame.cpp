#include "frame.h"

#include <glm/gtx/transform.hpp>

// // doesn't consider stasis velocity
// vector3d Frame::GetVelocityRelTo(const Frame *relTo) const
// {
// 	if (this == relTo) return vector3d(0,0,0); // early-out to avoid unnecessary computation
// 	vector3d diff = m_rootVel - relTo->m_rootVel;
// 	if (relTo->IsRotFrame()) return diff * relTo->m_rootOrient;
// 	else return diff;
// }

glm::dvec3 Frame::GetVelocityRelTo(Frame *relTo)
{
  if (this == relTo) return glm::dvec3(0, 0, 0);
  glm::dvec3 diff = root_vel - relTo->root_vel;
  if(relTo->isRotFrame()) {
    return diff * relTo->root_orient;
  }
  else {
    return diff;
  }
}

glm::dvec3 Frame::GetPositionRelTo(Frame *relTo)
{
  glm::dvec3 diff = root_pos - relTo->root_pos;
  if(relTo->isRotFrame()) {
    return diff * relTo->root_orient;
  }
  else {
    return diff;
  }
}

glm::dmat3 Frame::GetOrientRelTo(Frame *relTo)
{
  if (this == relTo) return glm::dmat3();
  return glm::transpose(relTo->root_orient) * root_orient;
}

void Frame::UpdateRootRelative(double time, double timestep) {
  if(parent == NULL) {
  }
  else {
    root_pos = parent->root_orient * pos + parent->root_pos;
    root_vel = parent->root_orient * vel + parent->root_vel;
    root_orient = parent->root_orient * orient;
  }
}

void Frame::UpdateOrbitRails(double time, double timestep) {
  if(parent != NULL and body != NULL and not rotating) {
    // translate body in orbit
    // orb_ang = fmod(orb_ang_speed * time, 2 * M_PI);

    if(orb_ang_speed != 0) {
      pos = glm::dmat3(glm::rotate(orb_ang_speed * timestep, glm::dvec3(0, 1, 0))) * pos;
    }
  }

  if(rotating) {
    ang = fmod(rot_ang_speed * time * timestep, 2 * M_PI);
    if(ang != 0) {
      orient = initial_orient * glm::dmat3(glm::rotate(-ang , glm::dvec3(0, 1, 0)));
      // printf("%s name rot %.2f", name, ang);
    }
  }

  // pos = orient * initial_pos;
  // printf("%s name pos %.0f, %.0f, %.0f\n", name, pos.x, pos.y, pos.z);

  UpdateRootRelative(time, timestep);

  for(Frame *child : children) {
    child->UpdateOrbitRails(time, timestep);
  }
}
