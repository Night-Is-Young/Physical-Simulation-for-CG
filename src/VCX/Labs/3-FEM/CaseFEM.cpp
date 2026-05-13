#include "Engine/app.h"
#include "Labs/3-FEM/CaseFEM.h"
#include "Labs/Common/ImGuiHelper.h"
#include <GLFW/glfw3.h>
#include <ranges>

namespace VCX::Labs::FEM {
    CaseFEM::CaseFEM():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat_vec4.vert"), Engine::GL::SharedShader("assets/shaders/flat_vec4.frag") })),
        _verticesItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0).Add<glm::vec4>("color", Engine::GL::DrawFrequency::Stream, 1), Engine::GL::PrimitiveType::Points),
        _linesItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines),
        _trianglesItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _wallItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _arrowItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles) {
        _cameraManager.AutoRotate = false;
        ResetSystem();

        std::vector<std::uint32_t> lineIdx;
        for (std::uint32_t i { 0 }; i < static_cast<uint32_t>(2 * _tetsystem._edges.size()); i++) {
            lineIdx.push_back(i);
        }
        _linesItem.UpdateElementBuffer(lineIdx);

        std::vector<std::uint32_t> triIdx;
        for (std::uint32_t i { 0 }; i < static_cast<uint32_t>(3 * _tetsystem._surfaceTriangles.size()); i++) {
            triIdx.push_back(i);
        }
        _trianglesItem.UpdateElementBuffer(triIdx);

        std::vector<std::uint32_t> wallIdx { 0, 1, 2, 0, 2, 3 };
        _wallItem.UpdateElementBuffer(wallIdx);
    }

    void CaseFEM::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Reset System")) ResetSystem();
            ImGui::SameLine();
            if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
        }
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Physics", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button(_tetsystem._gravity_on ? "Gravity On" : "Gravity Off")) _tetsystem._gravity_on = ! _tetsystem._gravity_on;
            ImGui::SameLine();
            if (ImGui::Button(_tetsystem._friction_on ? "Friction On" : "Friction Off")) _tetsystem._friction_on = ! _tetsystem._friction_on;
            ImGui::SliderFloat("Poisson's Ratio: ", &_tetsystem._nu, -1.0f, 0.5f, "%.2f");
            ImGui::SliderFloat("Friction Coefficient:", &_tetsystem._friction_ratio, 0.95f, 1.0f, "%.4f");
            ImGui::SliderFloat("Friction Coefficient:", &impulseMagnitude, 0.0f, 5.0f, "%.2f");

        }
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Appearance")) {
            if (ImGui::Button("Rendering Surface")) _showSurface = ! _showSurface;
            if (ImGui::Button("Rendering Edges")) _showEdges = ! _showEdges;
            if (ImGui::Button("Rendering Vertices")) _showVertices = ! _showVertices;
        }
    }

    Common::CaseRenderResult CaseFEM::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (! _stopped) {
            _tetsystem.SimulateTimeStep(Engine::GetDeltaTime());
        }

        // rendering walls
        _wallItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_wallPositions));
        _frame.Resize(desiredSize);
        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_LINE_SMOOTH);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glPointSize(_vertexSize);
        glLineWidth(_lineWidth);

        _program.GetUniforms().SetByName("useUniformColor", 1);
        _program.GetUniforms().SetByName("u_Color", glm::vec4(0.5f, 0.5f, 0.5f, 0.5f));
        _wallItem.Draw({ _program.Use() });

        if (_showSurface) {
            // rendering surfaces
            std::vector<glm::vec3> triVertexPositions;
            for (const auto tri : _tetsystem._surfaceTriangles) {
                triVertexPositions.push_back(_tetsystem._vertices[tri[0]]._pos);
                triVertexPositions.push_back(_tetsystem._vertices[tri[1]]._pos);
                triVertexPositions.push_back(_tetsystem._vertices[tri[2]]._pos);
            }
            _trianglesItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(triVertexPositions));
            _program.GetUniforms().SetByName("useUniformColor", 1);
            _program.GetUniforms().SetByName("u_Color", _triangleColor);
            _trianglesItem.Draw({ _program.Use() });
        }
        if (_showEdges) {
            // redering edges
            std::vector<glm::vec3> lineVerticesPositions;
            for (const auto line : _tetsystem._edges) {
                lineVerticesPositions.push_back(_tetsystem._vertices[line.first]._pos);
                lineVerticesPositions.push_back(_tetsystem._vertices[line.second]._pos);
            }
            _linesItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(lineVerticesPositions));
            _program.GetUniforms().SetByName("useUniformColor", 1);
            _program.GetUniforms().SetByName("u_Color", _lineColor);
            _linesItem.Draw({ _program.Use() });
        }
        if (_showVertices) {
            // rendering vertices
            std::vector<glm::vec3> verticesPositions;
            std::vector<glm::vec4> verticesColors;
            for (const auto vertex : _tetsystem._vertices) {
                verticesPositions.push_back(vertex._pos);
                verticesColors.push_back(vertex._color);
            }
            _verticesItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(verticesPositions));
            _verticesItem.UpdateVertexBuffer("color", Engine::make_span_bytes<glm::vec4>(verticesColors));
            _program.GetUniforms().SetByName("useUniformColor", 0);
            _verticesItem.Draw({ _program.Use() });
        }

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

    void CaseFEM::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);

        if (ImGui::IsKeyPressed(ImGuiKey_1)) {
            ProcessKeyInput(GLFW_KEY_1, GLFW_PRESS);
        }
        if (ImGui::IsKeyPressed(ImGuiKey_2)) {
            ProcessKeyInput(GLFW_KEY_2, GLFW_PRESS);
        }

        if (ImGui::IsKeyPressed(ImGuiKey_3)) {
            ProcessKeyInput(GLFW_KEY_3, GLFW_PRESS);
        }

        if (ImGui::IsKeyPressed(ImGuiKey_4)) {
            ProcessKeyInput(GLFW_KEY_4, GLFW_PRESS);
        }
    }

    void CaseFEM::ProcessKeyInput(int key, int action) {
        if (action != GLFW_PRESS) return;

        glm::vec3 impulse(0.0f);
        glm::vec3 direction(1.0f, 0.0f, 0.0f);
        char      type { 'x' };
        switch (key) {
        case GLFW_KEY_1:
            impulse = glm::vec3(impulseMagnitude, 0.0f, 0.0f);
            break;

        case GLFW_KEY_2:
            impulse = glm::vec3(-impulseMagnitude, 0.0f, 0.0f);
            direction = glm::vec3(-1.0f, 0.0f, 0.0f);
            break;

        case GLFW_KEY_3:
            impulse   = glm::vec3(0.0f, -impulseMagnitude, 0.0f);
            direction = glm::vec3(0.0f, -1.0f, 0.0f);
            break;

        case GLFW_KEY_4:
            impulse   = glm::vec3(0.0f, impulseMagnitude, 0.0f);
            direction = glm::vec3(0.0f, 1.0f, 0.0f);
            break;
        }

        glm::vec3 rightEndPos(0.0f);
        int       rightEndVertexCount = 0;
        for (auto & vertex : _tetsystem._vertices) {
            int i = vertex._id / ((_tetsystem.ny + 1) * (_tetsystem.nz + 1));
            if (i == _tetsystem.nx) {
                vertex._vel += impulse / vertex._mass;
                rightEndPos += vertex._pos;
                rightEndVertexCount++;
            }
        }

        if (rightEndVertexCount > 0) {
            rightEndPos /= rightEndVertexCount;
        }
    }

    void CaseFEM::ResetSystem() {
        _tetsystem.InitializeSystem();
        _camera.Eye    = glm::vec3(20.0f, -10.0f, 10.0f);
        _camera.Target = glm::vec3(4.0f, 2.0f, -2.0f);
        _camera.Up     = glm::vec3(0.0f, 0.0f, 1.0f);
        _camera.Fovy   = 45.0f;
        _cameraManager.Save(_camera);
    }
} // namespace VCX::Labs::FEM