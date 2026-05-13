#pragma once

#include "Labs/3-FEM/Vertex.h"
#include <glm/glm.hpp>
#include <vector>

namespace VCX::Labs::FEM {
    struct Triangle {
        int              _id;
        glm::mat2 _Dm_inv; // Inverse of the edge matrix
        std::vector<Vertex *> _vertices; // Pointers to the vertices of the triangle
        std::vector<glm::vec2> _uv;

        Triangle():
            _id(0),
            _Dm_inv(glm::mat2(1.0f)) {
            _vertices.resize(3); // A triangle has 3 vertices
            _uv.resize(3);       // Each vertex has a corresponding UV coordinate
        }
        Triangle(int id):
            _id(id),
            _Dm_inv(glm::mat2(1.0f)) {
            _vertices.resize(3); // A triangle has 3 vertices
            _uv.resize(3);       // Each vertex has a corresponding UV coordinate
        }
    };
}