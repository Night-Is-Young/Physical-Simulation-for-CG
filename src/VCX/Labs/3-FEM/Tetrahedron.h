#pragma once

#include "Labs/3-FEM/Vertex.h"
#include <glm/glm.hpp>
#include <vector>

namespace VCX::Labs::FEM {
    struct Tetrahedron {
        int              _id;
        glm::mat3 _E_inv; // Inverse of the edge matrix
        std::vector<Vertex *> _vertices; // Pointers to the vertices of the tetrahedron

        Tetrahedron():
            _id(0),
            _E_inv(glm::mat3(1.0f)) {
            _vertices.reserve(4); // A tetrahedron has 4 vertices
        }
        Tetrahedron(int id):
            _id(id),
            _E_inv(glm::mat3(1.0f)) {
            _vertices.reserve(4); // A tetrahedron has 4 vertices
        }
    };
}