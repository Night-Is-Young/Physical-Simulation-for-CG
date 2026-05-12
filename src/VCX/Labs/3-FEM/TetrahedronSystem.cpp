#include "Labs/3-FEM/TetrahedronSystem.h"
#include <iostream>

namespace VCX::Labs::FEM {

    void TetrahedronSystem::InitializeSystem() {
        const int verticesCnt = (nx + 1) * (ny + 1) * (nz + 1);
        const int tetrahedraCnt = nx * ny * nz * 6;

        _vertices.clear();
        _vertices.resize(verticesCnt);
        _tetrahedra.clear();
        _tetrahedra.resize(tetrahedraCnt);
        _edges.clear();
        std::vector<std::vector<bool>> edgeVisited(verticesCnt, std::vector<bool>(verticesCnt, false));
        _surfaceTriangles.clear();

        float hx = Lx / nx, hy = Ly / ny, hz = Lz / nz;

        for (int i = 0; i <= nx; ++i) {
            for (int j = 0; j <= ny; ++j) {
                for (int k = 0; k <= nz; ++k) {

                    int vertexId        = GetVertexIndex(i, j, k);
                    _vertices[vertexId] = Vertex(vertexId, glm::vec3(i * hx, j * hy, k * hz));

                    int type = 0;
                    if (i == 0 || i == nx) type++;
                    if (j == 0 || j == ny) type++;
                    if (k == 0 || k == nz) type++;
                    _vertices[vertexId]._type = type;

                    _vertices[vertexId]._color = ColorMap(_vertices[vertexId]._vel);
                }
            }
        }

        //std::vector<int> tetrahedronVertexIndices = {
        //    0, 1, 3, 7, 
        //    0, 2, 3, 7, 
        //    0, 1, 5, 7, 
        //    0, 4, 5, 7, 
        //    0, 2, 6, 7, 
        //    0, 4, 6, 7
        //};

        //int tetrahedronIndex = 0;
        //for (int i = 0; i < nx; i++) {
        //    for (int j = 0; j < ny; j++) {
        //        for (int k = 0; k < nz; k++) {

        //            std::vector<Vertex *> vptr;
        //            vptr.resize(8);
        //            for (int di = 0; di <= 1; di++) {
        //                for (int dj = 0; dj <= 1; dj++) {
        //                    for (int dk = 0; dk <= 1; dk++) {
        //                        vptr[di * 4 + dj * 2 + dk] = &_vertices[GetVertexIndex(i + di, j + dj, k + dk)];
        //                    }
        //                }
        //            }

        //            for (int t = 0; t < 6; t++) {
        //                _tetrahedra[tetrahedronIndex] = Tetrahedron(tetrahedronIndex);
        //                for (int v = 0; v < 4; v++) {
        //                    _tetrahedra[tetrahedronIndex]._vertices[v] = vptr[t * 4 + v];
        //                }
        //                tetrahedronIndex++;
        //            }
        //        }
        //    }
        //}
        int TetId = 0;
        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < ny; j++) {
                for (int k = 0; k < nz; k++) {
                    // 获取当前立方体的8个顶点指针
                    Vertex * v000 = &_vertices[GetVertexIndex(i, j, k)];
                    Vertex * v001 = &_vertices[GetVertexIndex(i, j, k + 1)];
                    Vertex * v010 = &_vertices[GetVertexIndex(i, j + 1, k)];
                    Vertex * v011 = &_vertices[GetVertexIndex(i, j + 1, k + 1)];
                    Vertex * v100 = &_vertices[GetVertexIndex(i + 1, j, k)];
                    Vertex * v101 = &_vertices[GetVertexIndex(i + 1, j, k + 1)];
                    Vertex * v110 = &_vertices[GetVertexIndex(i + 1, j + 1, k)];
                    Vertex * v111 = &_vertices[GetVertexIndex(i + 1, j + 1, k + 1)];

                    // 将一个立方体分割为6个四面体（直接通过索引赋值）
                    _tetrahedra[TetId]          = Tetrahedron(TetId);
                    _tetrahedra[TetId]._vertices = { v000, v001, v011, v111 }; // Tet1
                    TetId++;

                    _tetrahedra[TetId]          = Tetrahedron(TetId);
                    _tetrahedra[TetId]._vertices = { v000, v010, v011, v111 }; // Tet2
                    TetId++;

                    _tetrahedra[TetId]          = Tetrahedron(TetId);
                    _tetrahedra[TetId]._vertices = { v000, v001, v101, v111 }; // Tet3
                    TetId++;

                    _tetrahedra[TetId]          = Tetrahedron(TetId);
                    _tetrahedra[TetId]._vertices = { v000, v100, v101, v111 }; // Tet4
                    TetId++;

                    _tetrahedra[TetId]          = Tetrahedron(TetId);
                    _tetrahedra[TetId]._vertices = { v000, v010, v110, v111 }; // Tet5
                    TetId++;

                    _tetrahedra[TetId]          = Tetrahedron(TetId);
                    _tetrahedra[TetId]._vertices = { v000, v100, v110, v111 }; // Tet6
                    TetId++;
                }
            }
        }

        for (auto & tet : _tetrahedra) {
            glm::mat3 E;
            for (int i = 0; i < 3; i++) {
                E[0][i] = tet._vertices[1]->_pos[i] - tet._vertices[0]->_pos[i];
                E[1][i] = tet._vertices[2]->_pos[i] - tet._vertices[0]->_pos[i];
                E[2][i] = tet._vertices[3]->_pos[i] - tet._vertices[0]->_pos[i];
            }
            tet._E_inv = glm::inverse(E);
        }

        for (auto & tet : _tetrahedra) {
            for (auto vertex : tet._vertices) {
                vertex->_mass += 1.0f / 4.0f;
            }
        }

        for (auto & tet : _tetrahedra) {
            for (int i = 0; i < 4; i++) {
                for (int j = i + 1; j < 4; j++) {
                    int v1Id = tet._vertices[i]->_id;
                    int v2Id = tet._vertices[j]->_id;
                    if (! edgeVisited[v1Id][v2Id]) {
                        _edges.emplace_back(v1Id, v2Id);
                        edgeVisited[v1Id][v2Id] = true;
                        edgeVisited[v2Id][v1Id] = true;
                    }
                }
            }
        }

        for (int j = 0; j < ny; j++) {
            for (int k = 0; k < nz; k++) {
                _surfaceTriangles.push_back({ GetVertexIndex(0, j, k), GetVertexIndex(0, j, k + 1), GetVertexIndex(0, j + 1, k + 1) });
                _surfaceTriangles.push_back({ GetVertexIndex(0, j, k), GetVertexIndex(0, j + 1, k), GetVertexIndex(0, j + 1, k + 1) });
                _surfaceTriangles.push_back({ GetVertexIndex(nx, j, k), GetVertexIndex(nx, j, k + 1), GetVertexIndex(nx, j + 1, k + 1) });
                _surfaceTriangles.push_back({ GetVertexIndex(nx, j, k), GetVertexIndex(nx, j + 1, k), GetVertexIndex(nx, j + 1, k + 1) });
            }
        }

        for (int i = 0; i < nx; i++) {
            for (int k = 0; k < nz; k++) {
                _surfaceTriangles.push_back({ GetVertexIndex(i, 0, k), GetVertexIndex(i, 0, k + 1), GetVertexIndex(i + 1, 0, k + 1) });
                _surfaceTriangles.push_back({ GetVertexIndex(i, 0, k), GetVertexIndex(i + 1, 0, k), GetVertexIndex(i + 1, 0, k + 1) });
                _surfaceTriangles.push_back({ GetVertexIndex(i, ny, k), GetVertexIndex(i, ny, k + 1), GetVertexIndex(i + 1, ny, k + 1) });
                _surfaceTriangles.push_back({ GetVertexIndex(i, ny, k), GetVertexIndex(i + 1, ny, k), GetVertexIndex(i + 1, ny, k + 1) });
            }
        }

        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < ny; j++) {
                _surfaceTriangles.push_back({ GetVertexIndex(i, j, 0), GetVertexIndex(i, j + 1, 0), GetVertexIndex(i + 1, j + 1, 0) });
                _surfaceTriangles.push_back({ GetVertexIndex(i, j, 0), GetVertexIndex(i + 1, j, 0), GetVertexIndex(i + 1, j + 1, 0) });
                _surfaceTriangles.push_back({ GetVertexIndex(i, j, nz), GetVertexIndex(i, j + 1, nz), GetVertexIndex(i + 1, j + 1, nz) });
                _surfaceTriangles.push_back({ GetVertexIndex(i, j, nz), GetVertexIndex(i + 1, j, nz), GetVertexIndex(i + 1, j + 1, nz) });
            }
        }

        std::cout << "Initialized Tetrahedron System with " << verticesCnt << " vertices, " << tetrahedraCnt << " tetrahedra, " << _edges.size() << " edges, and " << _surfaceTriangles.size() << " surface triangles." << std::endl;
    }

    void TetrahedronSystem::AdvanceTetrahedronSystem(float dt) {
        int i = 1;
    }
}