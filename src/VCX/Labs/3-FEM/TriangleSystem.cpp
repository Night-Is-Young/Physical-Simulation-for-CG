#include "Labs/3-FEM/TriangleSystem.h"
#include <iostream>

namespace VCX::Labs::FEM {

    void TriangleSystem::InitializeSystem() {
        const int verticesCnt  = (nx + 1) * (ny + 1);
        const int trianglesCnt = nx * ny * 2;

        _vertices.clear();
        _vertices.resize(verticesCnt);
        _triangles.clear();
        _triangles.resize(trianglesCnt);
        _edges.clear();
        std::vector<std::vector<bool>> edgeVisited(verticesCnt, std::vector<bool>(verticesCnt, false));
        _surfaceTriangles.clear();

        float hx = Lx / nx, hy = Ly / ny;

        for (int i = 0; i <= nx; ++i) {
            for (int j = 0; j <= ny; ++j) {
                int vertexId        = GetVertexIndex(i, j);
                _vertices[vertexId] = Vertex(vertexId, glm::vec3(i * hx, j * hy, 0.0f));

                int type = 1;
                if (i == 0 || i == nx) type++;
                if (j == 0 || j == ny) type++;
                _vertices[vertexId]._type = type;

                _vertices[vertexId]._color = ColorMap(_vertices[vertexId]._vel);
            }
        }

        int triangleIndex = 0;
        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < ny; j++) {
                Vertex * v00 = &_vertices[GetVertexIndex(i, j)];
                Vertex * v01 = &_vertices[GetVertexIndex(i, j + 1)];
                Vertex * v10 = &_vertices[GetVertexIndex(i + 1, j)];
                Vertex * v11 = &_vertices[GetVertexIndex(i + 1, j + 1)];

                _triangles[triangleIndex]           = Triangle(triangleIndex);
                _triangles[triangleIndex]._vertices = { v00, v01, v11 }; // Triangle 1
                _triangles[triangleIndex]._uv       = { glm::vec2(i * hx, j * hy), glm::vec2(i * hx, (j + 1) * hy), glm::vec2((i + 1) * hx, (j + 1) * hy) };
                triangleIndex++;

                _triangles[triangleIndex]           = Triangle(triangleIndex);
                _triangles[triangleIndex]._vertices = { v00, v10, v11 }; // Triangle 2
                _triangles[triangleIndex]._uv       = { glm::vec2(i * hx, j * hy), glm::vec2((i + 1) * hx, (j + 1) * hy), glm::vec2((i + 1) * hx, (j + 1) * hy) };
                triangleIndex++;
            }
        }

        for (auto & tri : _triangles) {
            glm::mat2 Dm;

            Dm[0] = tri._uv[1] - tri._uv[0];
            Dm[1] = tri._uv[2] - tri._uv[0];

            tri._Dm_inv = glm::inverse(Dm);
        }

        for (auto & tri : _triangles) {
            for (auto vertex : tri._vertices) {
                vertex->_mass += 1.0f / 4.0f;
            }
        }

        for (auto & tri : _triangles) {
            for (int i = 0; i < 3; i++) {
                for (int j = i + 1; j < 3; j++) {
                    int v1Id = tri._vertices[i]->_id;
                    int v2Id = tri._vertices[j]->_id;
                    if (! edgeVisited[v1Id][v2Id]) {
                        _edges.emplace_back(v1Id, v2Id);
                        edgeVisited[v1Id][v2Id] = true;
                        edgeVisited[v2Id][v1Id] = true;
                    }
                }
            }
        }

        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < ny; j++) {
                _surfaceTriangles.push_back({ GetVertexIndex(i, j), GetVertexIndex(i, j + 1), GetVertexIndex(i + 1, j + 1) });
                _surfaceTriangles.push_back({ GetVertexIndex(i, j), GetVertexIndex(i + 1, j), GetVertexIndex(i + 1, j + 1) });
            }
        }
    }

    void TriangleSystem::AdvanceTetrahedronSystem(float dt) {
        for (auto & vertex : _vertices) {
            vertex._force = glm::vec3(0.0f);
        }

        for (auto & tri : _triangles) {
            glm::vec3 x0 = tri._vertices[0]->_pos;
            glm::vec3 x1 = tri._vertices[1]->_pos;
            glm::vec3 x2 = tri._vertices[2]->_pos;

            glm::mat2x3 Ds;
            Ds[0] = x1 - x0;
            Ds[1] = x2 - x0;

            glm::mat2x3 F = Ds * tri._Dm_inv;
            glm::mat2   G = 0.5f * (glm::transpose(F) * F - glm::mat2(1.0f));

            float     tr = G[0][0] + G[1][1];
            glm::mat2 S  = 2.0f * _mu * G + _lambda * tr * glm::mat2(1.0f);
            glm::mat2x3 P  = F * S;
            glm::mat2x3 H  = -P * glm::transpose(tri._Dm_inv);

            tri._vertices[0]->_force -= H[0] + H[1];
            tri._vertices[1]->_force += H[0];
            tri._vertices[2]->_force += H[1];
        }
        float maxV { 0.0f };
        for (auto & vertex : _vertices) {
            if (vertex._id < nx * (ny + 1)) {
                vertex._vel += dt * vertex._force / vertex._mass;
                if (_gravity_on) {
                    vertex._vel += dt * glm::vec3(0.0f, 0.0f, _gravity);
                }
                if (_friction_on) {
                    vertex._vel *= _friction_ratio;
                }

                maxV          = std::max(maxV, glm::length(vertex._vel));
                vertex._color = ColorMap(vertex._vel);
                vertex._pos += dt * vertex._vel;
            }
        }
        _max_vel = maxV;
    }
} // namespace VCX::Labs::FEM