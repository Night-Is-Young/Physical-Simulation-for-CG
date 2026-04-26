#include "FluidSimulator.h"

using namespace VCX::Labs::Fluid;

void Simulator::integrateParticles(float timeStep) {
    for (int i { 0 }; i < m_iNumSpheres; i++) {
        m_particleVel[i] += timeStep * gravity;
        m_particlePos[i] += timeStep * m_particleVel[i];
    }
}

void Simulator::buildHashTable() {
    m_hashtableindex.assign(m_iNumCells + 1, 0);

    int n = m_iCellY * m_iCellZ;
    int m = m_iCellZ;

    for (int p = 0; p < m_iNumSpheres; p++) {
        glm::ivec3 gridIdx = posToGridIndex(m_particlePos[p]);
        int        index   = gridIdx.x * n + gridIdx.y * m + gridIdx.z;
        m_hashtableindex[index + 1]++;
    }

    for (int i = 1; i <= m_iNumCells; i++) {
        m_hashtableindex[i] += m_hashtableindex[i - 1];
    }

    m_hashtable.resize(m_iNumSpheres);
    std::vector<int> count(m_iNumCells, 0);
    for (int p = 0; p < m_iNumSpheres; p++) {
        glm::ivec3 gridIdx  = posToGridIndex(m_particlePos[p]);
        int        index    = gridIdx.x * n + gridIdx.y * m + gridIdx.z;
        int        offset   = m_hashtableindex[index] + count[index];
        m_hashtable[offset] = p;
        count[index]++;
    }
}

void Simulator::pushParticlesApart(int numIters) {
    for (int iter = 0; iter < numIters; iter++) {
        buildHashTable();

        for (int pi = 0; pi < m_iNumSpheres; pi++) {
            glm::ivec3 gridIdx = posToGridIndex(m_particlePos[pi]);

            for (int di = -1; di <= 1; di++) {
                for (int dj = -1; dj <= 1; dj++) {
                    for (int dk = -1; dk <= 1; dk++) {
                        int ni = gridIdx.x + di;
                        int nj = gridIdx.y + dj;
                        int nk = gridIdx.z + dk;

                        if (ni < 0 || ni >= m_iCellX || nj < 0 || nj >= m_iCellY || nk < 0 || nk >= m_iCellZ)
                            continue;

                        int cellIdx = ni * m_iCellY * m_iCellZ + nj * m_iCellZ + nk;
                        int start   = m_hashtableindex[cellIdx];
                        int end     = m_hashtableindex[cellIdx + 1];

                        for (int idx = start; idx < end; idx++) {
                            int pj = m_hashtable[idx];
                            if (pi == pj) continue;

                            glm::vec3 d       = m_particlePos[pj] - m_particlePos[pi];
                            float     lenth_d = glm::length(d);
                            if (lenth_d < 2 * m_particleRadius) {
                                glm::vec3 s = (2 * m_particleRadius - lenth_d) * (d / lenth_d);
                                m_particlePos[pj] += s;
                                m_particlePos[pi] -= s;
                            }
                        }
                    }
                }
            }
        }
    }
}

void Simulator::updateParticleColors() {}