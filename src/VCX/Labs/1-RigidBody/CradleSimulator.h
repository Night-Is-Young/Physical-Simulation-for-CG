#pragma once

#include <glm/glm.hpp>
#include <Labs/1-RigidBody/Ball.h>
#include <vector>

namespace VCX::Labs::RigidBody {
    struct Simulator{ 
        std::vector<Ball> Balls;

        void SimulateTimestep(float const dt, float const g, float const l) {
            for (std::size_t i = 0; i < Balls.size(); i++) {
                glm::vec3 pos0 = Balls[i]._pos;
                Balls[i]._velocity += glm::vec3(0.0f, 0.0f, -g) * dt;
                Balls[i]._pos += Balls[i]._velocity * dt;
                glm::vec3 d = Balls[i]._pos - glm::vec3(float(i) * 2.f * Balls[i]._radius, 0, 0);
                Balls[i]._pos = glm::vec3(float(i) * 2.f * Balls[i]._radius, 0, 0) + glm::normalize(d) * l;
                Balls[i]._velocity = (Balls[i]._pos - pos0) / dt;
            }
        }
    };
}