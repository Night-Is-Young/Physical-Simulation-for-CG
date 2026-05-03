#include "Labs/2-FluidSimulation/CaseRendering.h"
#include "Engine/app.h"
#include "Labs/Common/ImGuiHelper.h"
#include <iostream>

namespace VCX::Labs::FluidSimulation {
    CaseRendering::CaseRendering():
        _programDepth(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/depth.vert"),
                                        Engine::GL::SharedShader("assets/shaders/depth.frag") })),
        _programSmooth(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/full_screen.vert"),
                                        Engine::GL::SharedShader("assets/shaders/smooth.frag") })),
        _programShade(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/full_screen.vert"),
                                        Engine::GL::SharedShader("assets/shaders/shade.frag") })),
        _particlesItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0).Add<glm::vec3>("velocity", Engine::GL::DrawFrequency::Stream, 1), Engine::GL::PrimitiveType::Points) {
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
        ResetSystem();
    }

    void CaseRendering::OnSetupPropsUI() {
        if (ImGui::Button("Reset System"))
            ResetSystem();
        ImGui::SameLine();
        if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation"))
            _stopped = ! _stopped;
        ImGui::Spacing();
        ImGui::Checkbox("Compensate Drift", &_simulation.compensateDrift);
        ImGui::SliderFloat("Rest Density", &_simulation.m_particleRestDensity, 0.0f, 10.0f, "%.2f");
        ImGui::SliderFloat("Compensate Drift k", &_simulation.ko, 0.0f, 2.0f, "%.2f");
        ImGui::Spacing();
        ImGui::SliderFloat("FLIP Ratio", &_simulation.m_fRatio, 0.0f, 1.0f, "%.2f");
        ImGui::SliderFloat("Time Step", &_simulation.dt, 0.001f, 0.05f, "%.3f");
    }

    Common::CaseRenderResult CaseRendering::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (! _stopped) _simulation.SimulateTimestep(_simulation.dt);

        std::vector<glm::vec3> positions;
        positions.reserve(_simulation.m_particlePos.size());
        for (size_t i = 0; i < _simulation.m_particlePos.size(); ++i) {
            positions.push_back(_simulation.m_particlePos[i]);
        }
        _particlesItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(positions));
        
        _cameraManager.Update(_camera);

        _depthFrame.Resize(desiredSize);
        _smoothFrame.Resize(desiredSize);
        _sFrame.Resize(desiredSize);
        _frame.Resize(desiredSize);

        {
            gl_using(_depthFrame);
            glClear(GL_DEPTH_BUFFER_BIT);

            _programDepth.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
            _programDepth.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());
            _programDepth.GetUniforms().SetByName("u_PointRadius", 0.05f * 0.5f);
            _programDepth.GetUniforms().SetByName("u_ScreenHeight", float(desiredSize.second));

            glEnable(GL_PROGRAM_POINT_SIZE);
            glEnable(GL_DEPTH_TEST);
            _particlesItem.Draw({ _programDepth.Use() });
            glDisable(GL_PROGRAM_POINT_SIZE);
            glDisable(GL_DEPTH_TEST);
        }

        Engine::GL::UniqueTexture2D const * pResultTexture = &_depthFrame.GetDepthStencilAttachment();

        if (_filterIterations > 0) {
            {
                gl_using(_smoothFrame);
                glClearColor(1, 1, 1, 1);
                glClear(GL_COLOR_BUFFER_BIT);

                _programSmooth.GetUniforms().SetByName("u_DepthMap", 0);
                _programSmooth.GetUniforms().SetByName("u_TexelSize", glm::vec2(1.0f / desiredSize.first, 1.0f / desiredSize.second));
                _programSmooth.GetUniforms().SetByName("u_BlurScale", _blurScale);
                _programSmooth.GetUniforms().SetByName("u_BlurDepthFalloff", _blurDepthFalloff);

                auto const useProgram = _programSmooth.Use();
                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, _depthFrame.GetDepthStencilAttachment().Get());

                glBindVertexArray(_emptyVAO.Get());
                glDrawArrays(GL_TRIANGLES, 0, 3);
            }

            pResultTexture = &_smoothFrame.GetColorAttachment();
            for (int i = 1; i < _filterIterations; ++i) {
                auto & target = (i % 2 == 1) ? _sFrame : _smoothFrame;
                auto & source = (i % 2 == 1) ? _smoothFrame : _sFrame;
                gl_using(target);
                glClearColor(1, 1, 1, 1);
                glClear(GL_COLOR_BUFFER_BIT);
                _programSmooth.GetUniforms().SetByName("u_DepthMap", 0);
                auto const useProgram = _programSmooth.Use();
                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, source.GetColorAttachment().Get());
                glBindVertexArray(_emptyVAO.Get());
                glDrawArrays(GL_TRIANGLES, 0, 3);
                pResultTexture = &target.GetColorAttachment();
            }
        }

        {
            gl_using(_frame);
            glClearColor(0.f, 0.f, 0.f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glm::mat4 projection = _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second));

            _programShade.GetUniforms().SetByName("u_SmoothDepthMap", 0);
            _programShade.GetUniforms().SetByName("u_InvProjection", glm::inverse(projection));
            _programShade.GetUniforms().SetByName("u_LightDir", glm::vec3(5.f, 5.0f, 5.f));
            _programShade.GetUniforms().SetByName("u_LightColor", glm::vec3(0.12f, 0.56f, 1.0f));
            _programShade.GetUniforms().SetByName("u_DarkColor", glm::vec3(0.05f, 0.31f, 0.73f));

            auto const useProgram = _programShade.Use();
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, pResultTexture->Get());

            glBindVertexArray(_emptyVAO.Get());
            glDrawArrays(GL_TRIANGLES, 0, 3);
        }

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseRendering::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

    void CaseRendering::ResetSystem() {
        _simulation.setupScene(_res);
    }
} // namespace VCX::Labs::FluidSimulation