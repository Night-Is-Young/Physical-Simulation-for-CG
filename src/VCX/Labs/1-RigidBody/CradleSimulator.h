#pragma once

#include <glm/glm.hpp>
#include <Labs/1-RigidBody/Ball.h>
#include <fcl/narrowphase/collision.h>
#include <vector>

namespace VCX::Labs::RigidBody {
    struct Simulator{ 
        std::vector<Ball> Balls;

        void SimulateTimestep(float const dt, float const g, float const l) {
            ResolveCollision();
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
            
            // 为所有球创建对应的碰撞几何体 (Sphere)
            std::vector<ColGeoPtr_t> geos;
            for (const auto& ball : Balls) {
                geos.push_back(ColGeoPtr_t(new fcl::Sphere<float>(ball._radius)));
            }

            // 两两遍历进行碰撞检测及解算
            for (std::size_t i = 0; i < Balls.size(); i++) {
                for (std::size_t j = i + 1; j < Balls.size(); j++) {
                    // 构建 FCL Transform 结构 (基于Eigen)
                    fcl::Transform3f trans1(Eigen::Translation3f(Eigen::Vector3f(Balls[i]._pos.x, Balls[i]._pos.y, Balls[i]._pos.z)));
                    fcl::Transform3f trans2(Eigen::Translation3f(Eigen::Vector3f(Balls[j]._pos.x, Balls[j]._pos.y, Balls[j]._pos.z)));
                    
                    // 构建具体碰撞对象
                    fcl::CollisionObject<float> obj1(geos[i], trans1);
                    fcl::CollisionObject<float> obj2(geos[j], trans2);
                    
                    // 设置碰撞请求与结果收集器
                    fcl::CollisionRequest<float> colRequest(1, true); // 最多只取1个 contact 就可以处理重叠方向
                    fcl::CollisionResult<float> colRes;
                    
                    // FCL 狭义碰撞检测
                    fcl::collide(&obj1, &obj2, colRequest, colRes);
                    
                    if (colRes.isCollision()) {
                        std::vector<fcl::Contact<float>> contacts;
                        colRes.getContacts(contacts);
                        if (contacts.empty()) continue;
                        
                        // 获取碰撞法线（通常由 obj1 指向 obj2）
                        glm::vec3 normal(contacts[0].normal[0], contacts[0].normal[1], contacts[0].normal[2]); 
                        normal = glm::normalize(normal);
                        
                        // 相对速度 (v2 - v1)
                        glm::vec3 vRel = Balls[j]._velocity - Balls[i]._velocity;
                        float dotProduct = glm::dot(vRel, normal);
                        
                        // dotProduct < 0 代表两球相向运动，正在接近，需要冲量将其推开 
                        if (dotProduct < 0) {
                            // 牛顿摆假设恢复系数(restitution)为1（即完全弹性碰撞）
                            float e = 1.0f;
                            
                            // 计算动量冲量大小（1D弹性碰撞公式）
                            float j_impulse = -(1.0f + e) * dotProduct / ((1.0f / Balls[i]._mass) + (1.0f / Balls[j]._mass));
                            
                            glm::vec3 impulse = j_impulse * normal;
                            
                            // 应用冲量解算速度
                            Balls[i]._velocity -= impulse / Balls[i]._mass;
                            Balls[j]._velocity += impulse / Balls[j]._mass;
                        }
                    }
                }
            }
        }
    };
}