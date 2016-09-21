#pragma once

#include <vector>

#include <glm/glm.hpp>

struct TerrainBody;

struct Frame {
  const char *name;

  Frame *parent; /* NULL if root */
  TerrainBody *body;
  std::vector<Frame *> children;
  bool rotating;
  bool has_rot_frame;

  double soi; // sphere of influence

  /* relative to parent */
  glm::dvec3 pos;
  glm::dvec3 initial_pos;
  glm::dmat3 initial_orient;
  glm::dmat3 orient;
  glm::dvec3 vel;
  double orb_ang_speed;
  double rot_ang_speed;

  /* relative to universe root (i.e. the sun) */
  glm::dvec3 root_pos;
  glm::dvec3 root_vel;
  glm::dmat3 root_orient;

  double ang;
  double orb_ang;

  void UpdateRootRelative(double time, double timestep);
  void UpdateOrbitRails(double time, double timestep);

  glm::dvec3 GetVelocityRelTo(Frame *relTo);
  glm::dvec3 GetPositionRelTo(Frame *relTo);
  glm::dmat3 GetOrientRelTo(Frame *relTo);

  bool isRotFrame() { return rotating; }
  bool hasRotFrame() { return has_rot_frame; }
  Frame *getNonRotFrame() {
    if(isRotFrame() == true) {
      return parent;
    } else {
      return this;
    }
  }

  Frame *getRotFrame() {
    if(hasRotFrame() == true) {
      return children.front();
    }
    else {
      return this;
    }
  }
  // For an object in a rotating frame, relative to non-rotating frames it
  // must attain this velocity within rotating frame to be stationary.
  // vector3d GetStasisVelocity(const vector3d &pos) const { return -vector3d(0,m_angSpeed,0).Cross(pos); }

  glm::dvec3 GetStasisVelocity(const glm::dvec3& pos) {
    return -glm::cross(glm::dvec3(0, rot_ang_speed, 0), pos);
  }
};
