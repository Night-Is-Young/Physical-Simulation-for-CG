#pragma once

#include "Engine/GL/RenderItem.h"
#include <glm/glm.hpp>
#include <vector>

namespace VCX::Labs::RigidBody {
    struct Wall {
        int                                 _id;
        glm::vec3                           _color;
        glm::vec2                           _dim;
        glm::vec3                           _pos;
        glm::vec3                           _tg1;
        glm::vec3                           _tg2;
        glm::vec3                           _normal;
        std::vector<glm::vec3>              _verticesPos;
        Engine::GL::UniqueIndexedRenderItem _triangleItem;
        Engine::GL::UniqueIndexedRenderItem _lineItem;

        Wall(int id, glm::vec3 pos, glm::vec2 dim, glm::vec3 tg1, glm::vec3 tg2, glm::vec3 color = glm::vec3(0.1f, 0.1f, 0.1f)):
            _id(id),
            _color(color),
            _pos(pos),
            _dim(dim),
            _tg1(glm::normalize(tg1)),
            _tg2(glm::normalize(tg2)),
            _normal(glm::normalize(glm::cross(tg1, tg2))),
            _verticesPos({ 
                pos + tg1 * dim.x / 2.0f + tg2 * dim.y / 2.0f,
                pos - tg1 * dim.x / 2.0f + tg2 * dim.y / 2.0f,
                pos - tg1 * dim.x / 2.0f - tg2 * dim.y / 2.0f,
                pos + tg1 * dim.x / 2.0f - tg2 * dim.y / 2.0f,
                }),
            _triangleItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
            _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
            //   1 --- 0
            //   |     |
            //   |     |
            //   2 --- 3
            const std::vector<std::uint32_t> wallLineIndex { 0, 1, 1, 2, 2, 3, 3, 0 };
            const std::vector<std::uint32_t> wallTriIndex { 0, 1, 2, 0, 2, 3 };
            _triangleItem.UpdateElementBuffer(wallTriIndex);
            _lineItem.UpdateElementBuffer(wallLineIndex);
        }
    };
} // namespace VCX::Labs::RigidBody