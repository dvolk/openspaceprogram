#ifndef CAMERA_INCLUDED_H
#define CAMERA_INCLUDED_H

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

struct Camera
{
public:
    Camera(const glm::vec3& pos, float fov, float aspect, float zNear, float zFar)
    {
        this->pos = pos;
        this->forward = glm::vec3(0.0f, 0.0f, 1.0f);
        this->up = glm::vec3(0.0f, 1.0f, 0.0f);
        this->projection = glm::perspective(fov, aspect, zNear, zFar);
        this->view = glm::translate(pos);
    }

    inline glm::mat4 GetProjection() const
    {
        return projection;
    }
    void ComputeView()
    {
      view = glm::lookAt(pos, pos + forward, up);
    }
    inline glm::dmat4 GetView() const
    {
      return view;//glm::lookAt(pos, pos + forward, up);
    }
    inline glm::dmat4 *GetView_()
    {
      return &view;
    }

    void Follow(const glm::dvec3 p) {
        pos = p - glm::dvec3(-15, 0, 0);
    }

    void MoveForward(double amt)
    {
    	pos += forward * amt;
    }

    void MoveRight(double amt)
    {
    	pos += glm::cross(up, forward) * amt;
    }

    void Pitch(double angle)
    {
    	glm::dvec3 right = glm::normalize(glm::cross(up, forward));

    	forward = glm::dvec3(glm::normalize(glm::rotate(angle, right) * glm::vec4(forward, 0.0)));
    	up = glm::normalize(glm::cross(forward, right));
    }

    void RotateY(double angle)
    {
    	static const glm::dvec3 UP(0.0f, 1.0f, 0.0f);

    	glm::mat4 rotation = glm::rotate(angle, UP);

    	forward = glm::dvec3(glm::normalize(rotation * glm::dvec4(forward, 0.0)));
    	up = glm::dvec3(glm::normalize(rotation * glm::vec4(up, 0.0)));
    }

    const glm::dvec3& GetPos() const {
        return pos;
    }

protected:
private:
    glm::dmat4 view;
    glm::mat4 projection;
    glm::dvec3 pos;
    glm::dvec3 forward;
    glm::dvec3 up;
};

#endif
