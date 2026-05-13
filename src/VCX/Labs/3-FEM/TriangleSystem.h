#pragma once

#include "Labs/3-FEM/Tetrahedron.h"
#include <glm/glm.hpp>
#include <vector>

namespace VCX::Labs::FEM {
    struct TetrahedronSystem {
        float Lx { 8.0f }, Ly { 2.0f }, Lz { 2.0f }; // Lengths of the system in x, y, z directions
        int   nx { 32 }, ny { 8 }, nz { 8 };       // Number of subdivisions in x, y, z directions
        glm::vec4 facecolor { 0.0f, 0.5f, 0.3f, 0.5f };
        glm::vec4 edgecolor { 1.0f, 1.0f, 1.0f, 1.0f };

        float _youngs_modulus { 20000.0f }; // Young's modulus for the material
        float _rho { 400.0f };              // Density of the material
        float _nu { 0.2f };                 // Poisson's ratio for the material
        float _gravity { -0.05f };           // Gravitational acceleration

        float _lambda { (_youngs_modulus / _rho) *_nu / ((1 + _nu) * (1 - 2 * _nu)) }; // Lamé's first parameter
        float _mu { (_youngs_modulus / _rho) / (2 * (1 + _nu)) };                              // Lamé's second parameter

        bool _gravity_on { true }; // Flag to turn off gravity
        bool _friction_on { true }; // Flag to turn off friction
        float _friction_ratio { 0.98f }; // Velocity reduction ratio for friction
        float _max_vel { 0.0f };    // Maximum velocity for clamping

        std::vector<Tetrahedron> _tetrahedra; // List of tetrahedra in the system
        std::vector<Vertex>      _vertices;   // List of vertices in the system
        std::vector<std::pair<int, int>> _edges; // List of edges in the system
        std::vector<std::vector<int>>    _surfaceTriangles; // List of triangles in the system on the surface

        int GetVertexIndex(int i, int j, int k) const {
            return i * (ny + 1) * (nz + 1) + j * (nz + 1) + k;
        }
        glm::vec4 ColorMap(glm::vec3 v) {
            float ratio = std::min(glm::length(v) / (0.2f * std::sqrt(_youngs_modulus / _rho)), 1.0f);
            return glm::vec4(1.0f, 1.0f, 1.0f - ratio, 0.8f);
        }

        void InitializeSystem();
        void AdvanceTetrahedronSystem(float dt);
        int  _numperstep { 5 }; // Number of substeps for each time step
        void SimulateTimeStep(float dt) {
            for (int i = 0; i < _numperstep; ++i) {
                AdvanceTetrahedronSystem(dt / _numperstep);
            }
        }

    };
}