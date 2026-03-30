#pragma once

#include "Engine/GL/RenderItem.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

namespace VCX::Labs::RigidBody {
    struct Box {
        int _id;
        glm::vec4 _color;
        float _mass;
        glm::vec3 _dim;
        glm::vec3 _pos;
        glm::mat3 _I;
        glm::mat3                           _I_world;
        glm::quat _q;
        glm::vec3 _velocity;
        glm::vec3 _omega;
        std::vector<glm::vec3> _verticesPos;
        Engine::GL::UniqueIndexedRenderItem _triangleItem;
        Engine::GL::UniqueIndexedRenderItem _lineItem;

        Box(const Box &)             = delete;
        Box & operator=(const Box &) = delete;
        Box(Box &&)                  = default;
        Box & operator=(Box &&)      = default;

        Box(int id, float mass, glm::vec3 dim, glm::vec3 position, glm::vec3 velocity = glm::vec3(0.0f), glm::vec3 omega = glm::vec3(0.0f), glm::vec4 color = glm::vec4(0.0f, 0.5f, 0.5f, 0.8f), glm::quat q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f)):
            _id(id),
            _color(color),
            _mass(mass),
            _dim(dim),
            _I(1.0f / 12.0f * mass * glm::mat3(dim.y * dim.y + dim.z * dim.z, 0.0f, 0.0f,
                0.0f, dim.x * dim.x + dim.z * dim.z, 0.0f,
                0.0f, 0.0f, dim.x * dim.x + dim.y * dim.y)),
            _pos(position),
            _q(glm::normalize(q)),
            _velocity(velocity),
            _omega(omega),
            _verticesPos({ glm::vec3(-dim.x, dim.y, dim.z) / 2.0f,
                glm::vec3(dim.x, dim.y, dim.z) / 2.0f,
                glm::vec3(dim.x, dim.y, -dim.z) / 2.0f,
                glm::vec3(-dim.x, dim.y, -dim.z) / 2.0f,
                glm::vec3(-dim.x, -dim.y, dim.z) / 2.0f,
                glm::vec3(dim.x, -dim.y, dim.z) / 2.0f,
                glm::vec3(dim.x, -dim.y, -dim.z) / 2.0f,
                glm::vec3(-dim.x, -dim.y, -dim.z) / 2.0f }),
            _triangleItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
            _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
            glm::mat3 R = glm::mat3_cast(q);
            _I_world     = R * _I * glm::transpose(R);
            //     3-----2
            //    /|    /|
            //   0 --- 1 |
            //   | 7 - | 6
            //   |/    |/
            //   4 --- 5
            const std::vector<std::uint32_t> boxLineIndex { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 };
            const std::vector<std::uint32_t> boxTriIndex { 0, 1, 2, 0, 2, 3, 1, 0, 4, 1, 4, 5, 1, 5, 6, 1, 6, 2, 2, 7, 3, 2, 6, 7, 0, 3, 7, 0, 7, 4, 4, 6, 5, 4, 7, 6 };
            _triangleItem.UpdateElementBuffer(boxTriIndex);
            _lineItem.UpdateElementBuffer(boxLineIndex);
        }
    };
} // namespace VCX::Labs::RigidBody