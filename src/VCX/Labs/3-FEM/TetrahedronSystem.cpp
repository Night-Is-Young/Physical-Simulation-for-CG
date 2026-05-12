#include "Labs/3-FEM/TetrahedronSystem.h"

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

        std::vector<int> tetrahedronVertexIndices = {
            0, 1, 3, 7, 
            0, 2, 3, 7, 
            0, 1, 5, 7, 
            0, 4, 5, 7, 
            0, 2, 6, 7, 
            0, 4, 6, 7
        };

        int tetrahedronIndex = 0;
        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < ny; j++) {
                for (int k = 0; k < nz; k++) {

                    std::vector<Vertex *> vptr(8);
                    for (int di = 0; di <= 1; di++) {
                        for (int dj = 0; dj <= 1; dj++) {
                            for (int dk = 0; dk <= 1; dk++) {
                                vptr[di * 4 + dj * 2 + dk] = &_vertices[GetVertexIndex(i + di, j + dj, k + dk)];
                            }
                        }
                    }

                    for (int t = 0; t < 6; t++) {
                        _tetrahedra[tetrahedronIndex] = Tetrahedron(tetrahedronIndex);
                        for (int v = 0; v < 4; v++) {
                            _tetrahedra[tetrahedronIndex]._vertices[v] = vptr[t * 4 + v];
                        }
                        tetrahedronIndex++;
                    }
                }
            }
        }
    }
}