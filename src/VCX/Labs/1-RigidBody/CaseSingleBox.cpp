#include "Labs/1-RigidBody/CaseSingleBox.h"
#include "Labs/Common/ImGuiHelper.h"
#include "Engine/app.h"

namespace VCX::Labs::RigidBody {

    CaseSingleBox::CaseSingleBox():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _boxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        _I = _mass / 12.f * glm::mat3(
            _dim.y * _dim.y + _dim.z * _dim.z, 0.f, 0.f,
            0.f, _dim.x * _dim.x + _dim.z * _dim.z, 0.f,
            0.f, 0.f, _dim.x * _dim.x + _dim.y * _dim.y);
        //     3-----2
        //    /|    /|
        //   0 --- 1 |
        //   | 7 - | 6
        //   |/    |/
        //   4 --- 5
        const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 }; // line index
        _lineItem.UpdateElementBuffer(line_index);

        // const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 4, 0, 1, 4, 5, 1, 6, 5, 1, 2, 6, 2, 3, 7, 2, 6, 7, 0, 3, 7, 0, 4, 7, 4, 5, 6, 4, 6, 7 };
        const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 0, 4, 1, 4, 5, 1, 5, 6, 1, 6, 2, 2, 7, 3, 2, 6, 7, 0, 3, 7, 0, 7, 4, 4, 6, 5, 4, 7, 6 };
        _boxItem.UpdateElementBuffer(tri_index);
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
    }

    void CaseSingleBox::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Box Color", glm::value_ptr(_boxColor));
            ImGui::SliderFloat("x", &_dim[0], 0.5, 4);
            ImGui::SliderFloat("y", &_dim[1], 0.5, 4);
            ImGui::SliderFloat("z", &_dim[2], 0.5, 4);

            ImGui::InputFloat("pos_x", &_center[0]);
            ImGui::InputFloat("pos_y", &_center[1]);
            ImGui::InputFloat("pos_z", &_center[2]);

            ImGui::SliderFloat3("Velocity", glm::value_ptr(_velocity), -5.0f, 5.0f);
        }
        if (ImGui::CollapsingHeader("Force", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SliderFloat("force_pos_x", &_forcePos.x, 0, _dim[0]);
            ImGui::SliderFloat("force_pos_y", &_forcePos.y, 0, _dim[1]);
            ImGui::SliderFloat("force_pos_z", &_forcePos.z, 0, _dim[2]);

            ImGui::SliderFloat("force_x", &_force.x, -5.f, 5.f);
            ImGui::SliderFloat("force_y", &_force.y, -5.f, 5.f);
            ImGui::SliderFloat("force_z", &_force.z, -5.f, 5.f);
        }
        if (ImGui::CollapsingHeader("Rotate", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SliderFloat("omega_x", &_omega.x, -5.f, 5.f);
            ImGui::SliderFloat("omega_y", &_omega.z, -5.f, 5.f);
            ImGui::SliderFloat("omega_y", &_omega.z, -5.f, 5.f);
            ImGui::Text("Orientation: (%.2f, %.2f, %.2f, %.2f)", _q.w, _q.x, _q.y, _q.z);
        }
        ImGui::Spacing();
    }

    void CaseSingleBox::AdvanceSingleBox(float dt) {
        // Translational Motion
        _velocity += dt * _force / _mass;
        _center += _velocity * dt;
        // Rotational Motion
        glm::mat3 R           = glm::mat3_cast(_q);
        glm::vec3 torque      = .0f * glm::cross(R * (_forcePos - _dim * .5f), _force) / _mass;
        glm::mat3 I_world     = R * _I * glm::transpose(R);
        glm::mat3 I_world_inv = glm::inverse(I_world);
        _omega += I_world_inv * (torque - glm::cross(_omega, I_world * _omega));
        glm::quat rotQuat = glm::exp(0.5f * dt * glm::quat(0.f, _omega));
        _q                = glm::normalize(rotQuat * _q);
    }

    Common::CaseRenderResult CaseSingleBox::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        // apply mouse control first
        OnProcessMouseControl(_cameraManager.getMouseMove());
        if (! _isStopped) {
            AdvanceSingleBox(Engine::GetDeltaTime());
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

        std::vector<glm::vec3> VertsPosition;
        glm::mat3              rotMat = glm::mat3_cast(_q);
        std::vector<glm::vec3> localVerts = {
            { -_dim[0] / 2,  _dim[1] / 2,  _dim[2] / 2 },
            {  _dim[0] / 2,  _dim[1] / 2,  _dim[2] / 2 },
            {  _dim[0] / 2,  _dim[1] / 2, -_dim[2] / 2 },
            { -_dim[0] / 2,  _dim[1] / 2, -_dim[2] / 2 },
            { -_dim[0] / 2, -_dim[1] / 2,  _dim[2] / 2 },
            {  _dim[0] / 2, -_dim[1] / 2,  _dim[2] / 2 },
            {  _dim[0] / 2, -_dim[1] / 2, -_dim[2] / 2 },
            { -_dim[0] / 2, -_dim[1] / 2, -_dim[2] / 2 }
        };
        for (const auto & localVert : localVerts) {
            VertsPosition.push_back(_center + rotMat * localVert);
        }
        auto span_bytes = Engine::make_span_bytes<glm::vec3>(VertsPosition);

        _program.GetUniforms().SetByName("u_Color", _boxColor);
        _boxItem.UpdateVertexBuffer("position", span_bytes);
        _boxItem.Draw({ _program.Use() });

        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        _lineItem.UpdateVertexBuffer("position", span_bytes);
        _lineItem.Draw({ _program.Use() });

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

    void CaseSingleBox::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);

        //ImGuiIO & io = ImGui::GetIO();
        //glm::vec3 forceDirection(0.f);
        //if (io.KeysDown[ImGuiKey_W]) forceDirection.z -= 1.f;
        //if (io.KeysDown[ImGuiKey_S]) forceDirection.z += 1.f;
        //if (io.KeysDown[ImGuiKey_A]) forceDirection.x -= 1.f;
        //if (io.KeysDown[ImGuiKey_D]) forceDirection.x += 1.f;
        //if (io.KeysDown[ImGuiKey_Q]) forceDirection.y += 1.f;
        //if (io.KeysDown[ImGuiKey_E]) forceDirection.y -= 1.f;

        //if (glm::length(forceDirection) > 0.f) {
        //    glm::mat3 viewRot = glm::mat3(_camera.GetViewMatrix());
        //    _force = viewRot * glm::normalize(forceDirection) * 0.001f;
        //} else {
        //    _force = glm::vec3(0.f);
        //}
    }

    void CaseSingleBox::OnProcessMouseControl(glm::vec3 mouseDelta) {
        float movingScale = 0.1f;
        _center += mouseDelta * movingScale;
    }

} // namespace VCX::Labs::GettingStarted
