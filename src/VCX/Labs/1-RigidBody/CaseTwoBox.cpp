#include "Labs/1-RigidBody/CaseTwoBox.h"
#include "Labs/Common/ImGuiHelper.h"
#include "Engine/app.h"
#include <fcl/narrowphase/collision.h>

namespace VCX::Labs::RigidBody {

    CaseTwoBox::CaseTwoBox():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        box1(1, 100.f, glm::vec3(1.f, 2.f, 3.f), glm::vec3(-5.f, 0.0f, 0.0f)),
        box2(2, 100.f, glm::vec3(1.f, 2.f, 3.f), glm::vec3(5.f, 0.0f, 0.0f)) {
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
    }

    void CaseTwoBox::OnSetupPropsUI() {
        if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation"))
            _stopped = ! _stopped;
        if (ImGui::CollapsingHeader("Box1", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("Position_1: (%.2f, %.2f, %.2f)", box1._pos.x, box1._pos.y, box1._pos.z);
            ImGui::Text("Orientation_1: (%.2f, %.2f, %.2f, %.2f)", box1._q.w, box1._q.x, box1._q.y, box1._q.z);
            ImGui::SliderFloat3("Velocity_1", glm::value_ptr(box1._velocity), -5.0f, 5.0f);
            ImGui::SliderFloat3("Omega_1", glm::value_ptr(box1._omega), -5.0f, 5.0f);
        }
        if (ImGui::CollapsingHeader("Box2", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("Position_2: (%.2f, %.2f, %.2f)", box2._pos.x, box2._pos.y, box2._pos.z);
            ImGui::Text("Orientation_2: (%.2f, %.2f, %.2f, %.2f)", box2._q.w, box2._q.x, box2._q.y, box2._q.z);
            ImGui::SliderFloat3("Velocity_2", glm::value_ptr(box2._velocity), -5.0f, 5.0f);
            ImGui::SliderFloat3("Omega_2", glm::value_ptr(box2._omega), -5.0f, 5.0f);
        }
    }

    void CaseTwoBox::ResolveCollision(Box & box1, Box & box2) {
        using ColGeoPtr_t = std::shared_ptr<fcl::CollisionGeometry<float>>;
        ColGeoPtr_t geo1(new fcl::Box<float>(box1._dim.x, box1._dim.y, box1._dim.z));
        ColGeoPtr_t geo2(new fcl::Box<float>(box2._dim.x, box2._dim.y, box2._dim.z));
        fcl::Transform3f trans1(Eigen::Translation3f(Eigen::Vector3f(box1._pos.x, box1._pos.y, box1._pos.z)) * Eigen::Quaternionf(box1._q.w, box1._q.x, box1._q.y, box1._q.z));
        fcl::Transform3f trans2(Eigen::Translation3f(Eigen::Vector3f(box2._pos.x, box2._pos.y, box2._pos.z)) * Eigen::Quaternionf(box2._q.w, box2._q.x, box2._q.y, box2._q.z));
        fcl::CollisionObject<float> obj1(geo1, trans1);
        fcl::CollisionObject<float> obj2(geo2, trans2);
        fcl::CollisionRequest<float> colRequest(8, true);
        fcl::CollisionResult<float>  colRes;
        fcl::collide(&obj1, &obj2, colRequest, colRes);
        if (colRes.isCollision()) {
            std::vector<fcl::Contact<float>> contacts;
            colRes.getContacts(contacts);
            glm::vec3 colPos { 0.0f };
            glm::vec3 colNormal { 0.0f };
            float     colDepth = 0.0f;
            int       counter {0};
            for (const auto & contact : contacts) {
                colPos += glm::vec3(contact.pos[0], contact.pos[1], contact.pos[2]);
                colNormal += glm::vec3(contact.normal[0], contact.normal[1], contact.normal[2]);
                colDepth += contact.penetration_depth;
                counter++;
            }
            static_cast<float>(counter);
            colPos /= counter;
            colDepth /= counter;
            colNormal = glm::normalize(colNormal);
            glm::vec3 Rr1 = colPos - box1._pos;
            glm::vec3 Rr2 = colPos - box2._pos;
            glm::vec3 colVel1 = box1._velocity + glm::cross(box1._omega, Rr1);
            glm::vec3 colVel2 = box2._velocity + glm::cross(box2._omega, Rr2);
            glm::vec3 vRel = colVel2 - colVel1;
            float     dotProduct = glm::dot(vRel, colNormal);
            if ( dotProduct < 0) {
                // Resolve Collision
                glm::vec3 vRelN = dotProduct * colNormal;
                glm::vec3 vRelT = vRel - vRelN;
                glm::vec3 vRelNn = -_muN * vRelN;
                float     alpha  = fmax(1.0f - _muT * (1 + _muN) * glm::length(vRelN) / glm::length(vRelT), 0.0f);
                glm::vec3 vRelTn = alpha * vRelT;
                glm::vec3 vReln  = vRelNn + vRelTn;

                glm::mat3 r1(0.0f, -Rr1.z, Rr1.y, Rr1.z, 0.0f, -Rr1.x, -Rr1.y, Rr1.x, 0.0f);
                glm::mat3 R1 = glm::mat3_cast(box1._q);
                glm::mat3 I_world1 = R1 * box1._I * glm::transpose(R1);
                glm::mat3 I_world_inv1 = glm::inverse(I_world1);
                glm::mat3 K1           = 1.0f / box1._mass * glm::mat3(1.0f) - r1 * I_world_inv1 * r1;

                glm::mat3 r2(0.0f, -Rr2.z, Rr2.y, Rr2.z, 0.0f, -Rr2.x, -Rr2.y, Rr2.x, 0.0f);
                glm::mat3 R2 = glm::mat3_cast(box2._q);
                glm::mat3 I_world2 = R2 * box2._I * glm::transpose(R2);
                glm::mat3 I_world_inv2 = glm::inverse(I_world2);
                glm::mat3 K2           = 1.0f / box2._mass * glm::mat3(1.0f) - r2 * I_world_inv2 * r2;

                glm::mat3 K = K1 + K2;
                glm::vec3 j = glm::inverse(K) * (vReln - vRel);
                box1._velocity += -j / box1._mass;
                box2._velocity += j / box2._mass;
                box1._omega += I_world_inv1 * glm::cross(Rr1, -j);
                box2._omega += I_world_inv2 * glm::cross(Rr2, j);
            }
        }
    }

    void CaseTwoBox::AdvanceTwoBox(float dt) {
        // Resolve Collision
        ResolveCollision(box1, box2);
        // Translational Motion
        box1._pos += box1._velocity * dt;
        box2._pos += box2._velocity * dt;
        // Rotational Motion
        glm::mat3 R1          = glm::mat3_cast(box1._q);
        glm::mat3 I_world1     = R1 * box1._I * glm::transpose(R1);
        glm::mat3 I_world_inv1 = glm::inverse(I_world1);
        box1._omega += dt * I_world_inv1 * (-glm::cross(box1._omega, I_world1 * box1._omega));
        box1._q = glm::normalize(glm::exp(0.5f * dt * glm::quat(0.f, box1._omega)) * box1._q);

        glm::mat3 R2           = glm::mat3_cast(box2._q);
        glm::mat3 I_world2     = R2 * box2._I * glm::transpose(R2);
        glm::mat3 I_world_inv2 = glm::inverse(I_world2);
        box2._omega += dt * I_world_inv2 * (-glm::cross(box2._omega, I_world2 * box2._omega));
        box2._q = glm::normalize(glm::exp(0.5f * dt * glm::quat(0.f, box2._omega)) * box2._q);
    }

    Common::CaseRenderResult CaseTwoBox::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        // apply mouse control first
        OnProcessMouseControl(_cameraManager.getMouseMove());
        if (! _stopped) {
            AdvanceTwoBox(Engine::GetDeltaTime());
        }
        // rendering
        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(.5f);

        // Draw Box1
        std::vector<glm::vec3> VertsPosition1;
        glm::mat3              rotMat1 = glm::mat3_cast(box1._q);
        for (const auto & vertPos : box1._verticesPos) {
            VertsPosition1.push_back(box1._pos + rotMat1 * vertPos);
        }
        auto span_bytes1 = Engine::make_span_bytes<glm::vec3>(VertsPosition1);

        _program.GetUniforms().SetByName("u_Color", box1._color);
        box1._triangleItem.UpdateVertexBuffer("position", span_bytes1);
        box1._triangleItem.Draw({ _program.Use() });

        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        box1._lineItem.UpdateVertexBuffer("position", span_bytes1);
        box1._lineItem.Draw({ _program.Use() });

        //Draw Box2
        std::vector<glm::vec3> VertsPosition2;
        glm::mat3              rotMat2 = glm::mat3_cast(box2._q);
        for (const auto & vertPos : box2._verticesPos) {
            VertsPosition2.push_back(box2._pos + rotMat2 * vertPos);
        }
        auto span_bytes2 = Engine::make_span_bytes<glm::vec3>(VertsPosition2);
        _program.GetUniforms().SetByName("u_Color", box2._color);
        box2._triangleItem.UpdateVertexBuffer("position", span_bytes2);
        box2._triangleItem.Draw({ _program.Use() });

        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        box2._lineItem.UpdateVertexBuffer("position", span_bytes2);
        box2._lineItem.Draw({ _program.Use() });

        glLineWidth(1.f);
        glPointSize(1.f);
        glDisable(GL_LINE_SMOOTH);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseTwoBox::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

    void CaseTwoBox::OnProcessMouseControl(glm::vec3 mouseDelta) {
        float movingScale = 0.1f;
        //_center += mouseDelta * movingScale;
    }

} // namespace VCX::Labs::RigidBody
