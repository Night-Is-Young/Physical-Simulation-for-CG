#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace VCX::Labs::FEM {
    struct Vertex {
        int       _id;
        int       _type; // 0: inside; 1: surface; 2: edge; 3: corner
        float     _mass;
        glm::vec3 _pos;
        glm::vec3 _vel;
        glm::vec3 _force;
        glm::vec4 _color;

        Vertex():
            _id(0),
            _type(0),
            _mass(0.0f),
            _pos(glm::vec3(0.0f)),
            _vel(glm::vec3(0.0f)),
            _force(glm::vec3(0.0f)),
            _color(glm::vec4(1.0f)) {
        }

        Vertex(int id, glm::vec3 pos):
            _id(id),
            _type(0),
            _mass(0.0f),
            _pos(pos),
            _vel(glm::vec3(0.0f)),
            _force(glm::vec3(0.0f)),
            _color(glm::vec4(1.0f)) {
        }
    };
}