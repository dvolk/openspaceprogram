#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

class Camera {
 public:
  glm::dmat4 view;
  glm::mat4 projection;
  glm::dvec3 pos;
  glm::dvec3 forward;
  glm::dvec3 up;
  float fov, aspect, zNear, zFar;

  void ComputeView() {};
  void Follow(const glm::dvec3 p) {};
  void MoveForward(double amt) {};
  void MoveRight(double amt) {};
  void Pitch(double angle) {};
  void RotateY(double angle) {};
  void wheel(double amt) {};

  void setAspect(float _aspect);
  const glm::dvec3& GetPos() const;
  const glm::dvec3& GetForward() const;
  glm::mat4 GetProjection() const;
  glm::dmat4 GetView() const;
  glm::dmat4 *GetView_();
};

class OrbitCamera : public Camera {
 public:
  glm::dvec3 focusPoint;
  glm::dmat3 orient;
  double distance;

  OrbitCamera(const glm::dvec3& shipPos, float fov, float aspect, float zNear, float zFar);

  void ComputeView();
  void Follow(const glm::dvec3 p);

  void MoveForward(double amt) { }
  void MoveRight(double amt) { }

  void Pitch(double angle);
  void RotateY(double angle);

  void wheel(double amt);
};

struct WeirdCamera : public Camera {
 public:
  WeirdCamera(const glm::vec3& pos, float fov, float aspect, float zNear, float zFar);

  void ComputeView();
  void Follow(const glm::dvec3 p);

  void MoveForward(double amt);
  void MoveRight(double amt);

  void Pitch(double angle);
  void RotateY(double angle);

  void wheel(double amt) {}
};
