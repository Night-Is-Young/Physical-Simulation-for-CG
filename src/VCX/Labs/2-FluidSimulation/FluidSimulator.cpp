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

void Simulator::handleParticleCollisions(glm::vec3 obstaclePos, float obstacleRadius, glm::vec3 obstacleVel) {
    for (int p = 0; p < m_iNumSpheres; p++) {
        if (m_particlePos[p].x < xmin + m_h + m_particleRadius) {
            m_particlePos[p].x = xmin + m_h + m_particleRadius;
            m_particleVel[p].x = 0.0f;
        }
        if (m_particlePos[p].y < ymin + m_h + m_particleRadius) {
            m_particlePos[p].y = ymin + m_h + m_particleRadius;
            m_particleVel[p].y = 0.0f;
        }
        if (m_particlePos[p].z < zmin + m_h + m_particleRadius) {
            m_particlePos[p].z = zmin + m_h + m_particleRadius;
            m_particleVel[p].z = 0.0f;
        }
        if (m_particlePos[p].x > xmax - m_h - m_particleRadius) {
            m_particlePos[p].x = xmax - m_h - m_particleRadius;
            m_particleVel[p].x = 0.0f;
        }
        if (m_particlePos[p].y > ymax - m_h - m_particleRadius) {
            m_particlePos[p].y = ymax - m_h - m_particleRadius;
            m_particleVel[p].y = 0.0f;
        }
        if (m_particlePos[p].z > zmax - m_h - m_particleRadius) {
            m_particlePos[p].z = zmax - m_h - m_particleRadius;
            m_particleVel[p].z = 0.0f;
        }
        glm::vec3 d        = m_particlePos[p] - obstaclePos;
        float     length_d = glm::length(d);
        glm::vec3 n        = d / length_d;
        if (length_d < obstacleRadius + m_particleRadius) {
            glm::vec3 s = (obstacleRadius + m_particleRadius - length_d) * n;
            m_particlePos[p] += s;
        }
    }
}
void Simulator::updateParticleDensity() {
    m_particleDensity.clear();
    m_particleDensity.resize(m_iNumCells, 0.0f);

    int n = m_iCellY * m_iCellZ;
    int m = m_iCellZ;
    for (int p = 0; p < m_iNumSpheres; p++) {
        float xp0      = m_particlePos[p].x - xmin;
        float yp0      = m_particlePos[p].y - xmin;
        float zp0      = m_particlePos[p].z - zmin;
        float xp0_half = xp0 - m_h / 2;
        float yp0_half = yp0 - m_h / 2;
        float zp0_half = zp0 - m_h / 2;
        int   i_half   = static_cast<int>(xp0_half / m_h);
        int   j_half   = static_cast<int>(yp0_half / m_h);
        int   k_half   = static_cast<int>(zp0_half / m_h);

        float Dx = xp0_half - i_half * m_h;
        float Dy = yp0_half - j_half * m_h;
        float Dz = zp0_half - k_half * m_h;

        const float w[8] = {
            (1 - Dx) * (1 - Dy) * (1 - Dz),
            (1 - Dx) * (1 - Dy) * Dz,
            (1 - Dx) * Dy * (1 - Dz),
            (1 - Dx) * Dy * Dz,
            Dx * (1 - Dy) * (1 - Dz),
            Dx * (1 - Dy) * Dz,
            Dx * Dy * (1 - Dz),
            Dx * Dy * Dz
        };

        const int di[8] = { 0, 0, 0, 0, 1, 1, 1, 1 };
        const int dj[8] = { 0, 0, 1, 1, 0, 0, 1, 1 };
        const int dk[8] = { 0, 1, 0, 1, 0, 1, 0, 1 };

        for (int idx = 0; idx < 8; idx++) {
            int index = (i_half + di[idx]) * n + (j_half + dj[idx]) * m + (k_half + dk[idx]);
            m_particleDensity[index] += w[idx];
        }
    }
}

void Simulator::updateParticleColors() {}