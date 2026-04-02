#pragma once

#include "Engine/GL/RenderItem.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

namespace VCX::Labs::RigidBody {
    struct Ball {
        int       _id;
        float                               _mass;
        float                               _radius;
        glm::vec3                           _pos;
        glm::vec3                           _velocity;

        Ball(int id, float mass, float radius, glm::vec3 pos, glm::vec3 velocity = glm::vec3(0.0f)) :
            _id(id),
            _mass(mass),
            _radius(radius),
            _pos(pos),
            _velocity(velocity) {
        }
    };
} // namespace VCX::Labs::RigidBody