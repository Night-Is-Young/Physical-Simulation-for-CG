#pragma once

#include <glm/glm.hpp>
#include <Labs/1-RigidBody/Ball.h>
#include <fcl/narrowphase/collision.h>
#include <vector>
#include <algorithm>

namespace VCX::Labs::RigidBody {
    struct Simulator{ 
        std::vector<Ball> Balls;

        void SimulateTimestep(float const dt, float const g, float const l) {
            const int Iterations = 5;
            for (int i = 0; i < Iterations; i++) {
                ResolveCollision();
            }
            for (std::size_t i = 0; i < Balls.size(); i++) {
                glm::vec3 pos0 = Balls[i]._pos;
                Balls[i]._velocity += glm::vec3(0.0f, 0.0f, -g) * dt;
                Balls[i]._pos += Balls[i]._velocity * dt;
                glm::vec3 d = Balls[i]._pos - glm::vec3(float(i) * 2.f * Balls[i]._radius, 0, 0);
                Balls[i]._pos = glm::vec3(float(i) * 2.f * Balls[i]._radius, 0, 0) + glm::normalize(d) * l;
                Balls[i]._velocity = (Balls[i]._pos - pos0) / dt;
            }
        }

        void ResolveCollision() {
            using ColGeoPtr_t = std::shared_ptr<fcl::CollisionGeometry<float>>;

            std::vector<ColGeoPtr_t> geos;
            for (const auto& ball : Balls) {
                geos.push_back(ColGeoPtr_t(new fcl::Sphere<float>(ball._radius)));
            }

            for (std::size_t i = 0; i < Balls.size(); i++) {
                for (std::size_t j = i + 1; j < Balls.size(); j++) {
                    fcl::Transform3f trans1(Eigen::Translation3f(Eigen::Vector3f(Balls[i]._pos.x, Balls[i]._pos.y, Balls[i]._pos.z)));
                    fcl::Transform3f trans2(Eigen::Translation3f(Eigen::Vector3f(Balls[j]._pos.x, Balls[j]._pos.y, Balls[j]._pos.z)));
                    fcl::CollisionObject<float> obj1(geos[i], trans1);
                    fcl::CollisionObject<float> obj2(geos[j], trans2);
                    fcl::CollisionRequest<float> colRequest(8, true);
                    fcl::CollisionResult<float> colRes;
                    fcl::collide(&obj1, &obj2, colRequest, colRes);
                    if (colRes.isCollision()) {
                        std::vector<fcl::Contact<float>> contacts;
                        colRes.getContacts(contacts);
                        if (contacts.empty()) continue;
                        glm::vec3 colPos { 0.0f };
                        glm::vec3 colNormal { 0.0f };
                        float     colDepth = 0.0f;
                        int       counter { 0 };
                        for (const auto & contact : contacts) {
                            colPos += glm::vec3(contact.pos[0], contact.pos[1], contact.pos[2]);
                            colNormal += glm::vec3(contact.normal[0], contact.normal[1], contact.normal[2]);
                            colDepth += contact.penetration_depth;
                            counter++;
                        }
                        
                        colPos /= counter;
                        colDepth /= counter;
                        colNormal = glm::normalize(colNormal);

                        glm::vec3 vRel = Balls[j]._velocity - Balls[i]._velocity;
                        float dotProduct = glm::dot(vRel, colNormal);
                        
                        if (dotProduct < 0) {

                            float e = (glm::abs(dotProduct) < 0.05f) ? 0.0f : 1.0f;
                            
                            float J = -(1.0f + e) * dotProduct / ((1.0f / Balls[i]._mass) + (1.0f / Balls[j]._mass));
                            glm::vec3 impulse = J * colNormal;

                            Balls[i]._velocity -= impulse / Balls[i]._mass;
                            Balls[j]._velocity += impulse / Balls[j]._mass;
                            
                            const float percent = 0.5f;
                            const float slop = 0.001f;
                            glm::vec3 correction = (std::max(colDepth - slop, 0.0f) / ((1.0f / Balls[i]._mass) + (1.0f / Balls[j]._mass))) * percent * colNormal;
                            
                            Balls[i]._pos -= correction / Balls[i]._mass;
                            Balls[j]._pos += correction / Balls[j]._mass;
                        }
                    }
                }
            }
        }
    };
}