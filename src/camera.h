#ifndef CAMERA_INCLUDED_H
#define CAMERA_INCLUDED_H

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

  void Follow(const glm::dvec3 p);
  void MoveForward(double amt);
  void MoveRight(double amt);
  void Pitch(double angle);
  void RotateY(double angle);

  void setAspect(float _aspect) {
    this->projection = glm::perspective(fov, _aspect, zNear, zFar);
  }
  const glm::dvec3& GetPos() const {
    return pos;
  }
  const glm::dvec3& GetForward() const {
    return forward;
  }
  glm::mat4 GetProjection() const {
    return projection;
  }
  glm::dmat4 GetView() const {
    return view;//glm::lookAt(pos, pos + forward, up);
  }
  glm::dmat4 *GetView_() {
    return &view;
  }
};

class OrbitCamera : public Camera {
 public:
  glm::dvec3 focusPoint;
  glm::dmat3 orient;

  OrbitCamera(const glm::dvec3& shipPos, float fov, float aspect, float zNear, float zFar) {
    this->fov = fov;
    this->aspect = aspect;
    this->zNear = zNear;
    this->zFar = zFar;
    this->focusPoint = shipPos;
    this->pos = focusPoint + glm::dvec3(10, 0, 0);
    this->forward = focusPoint - pos;
    this->up = glm::normalize(pos);
    this->projection = glm::perspective(fov, aspect, zNear, zFar);
    this->view = glm::translate(pos);
    this->orient = glm::dmat3();
  }
  void ComputeView()
  {
    pos = focusPoint + orient * glm::dvec3(10, 0, 0);
    forward = glm::normalize(focusPoint - pos);
    up = glm::normalize(pos);
    view = glm::lookAt(pos, pos + forward, up);
  }

  void Follow(const glm::dvec3 p) {
    focusPoint = p;
  }

  void MoveForward(double amt) { }
  void MoveRight(double amt) { }

  void Pitch(double angle)
  {
    orient = orient * glm::dmat3(glm::rotate(angle, glm::dvec3(0, -1, 0)));
  }

  void RotateY(double angle)
  {
    orient = orient * glm::dmat3(glm::rotate(angle, glm::dvec3(0,  0, -1)));
  }
};

struct WeirdCamera : public Camera {
 public:
  WeirdCamera(const glm::vec3& pos, float fov, float aspect, float zNear, float zFar) {
    this->fov = fov;
    this->aspect = aspect;
    this->zNear = zNear;
    this->zFar = zFar;
    this->pos = pos;
    this->forward = glm::vec3(0.0f, 0.0f, 1.0f);
    this->up = glm::vec3(0.0f, 1.0f, 0.0f);
    this->projection = glm::perspective(fov, aspect, zNear, zFar);
    this->view = glm::translate(pos);
  }

  void ComputeView() {
    view = glm::lookAt(pos, pos + forward, up);
  }
  void Follow(const glm::dvec3 p) {
    pos = p - glm::dvec3(-15, 0, 0);
  }

  void MoveForward(double amt) {
    pos += forward * amt;
  }

  void MoveRight(double amt) {
    pos += glm::cross(up, forward) * amt;
  }

  void Pitch(double angle) {
    glm::dvec3 right = glm::normalize(glm::cross(up, forward));

    forward = glm::dvec3(glm::normalize(glm::rotate(angle, right) * glm::vec4(forward, 0.0)));
    up = glm::normalize(glm::cross(forward, right));
  }

  void RotateY(double angle) {
    static const glm::dvec3 UP(0.0f, 1.0f, 0.0f);

    glm::mat4 rotation = glm::rotate(angle, UP);

    forward = glm::dvec3(glm::normalize(rotation * glm::dvec4(forward, 0.0)));
    up = glm::dvec3(glm::normalize(rotation * glm::vec4(up, 0.0)));
  }

};

#endif
