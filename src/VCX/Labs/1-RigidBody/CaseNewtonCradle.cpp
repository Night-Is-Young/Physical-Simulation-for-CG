#include <spdlog/spdlog.h>
#include "Engine/app.h"
#include "Labs/1-RigidBody/CaseNewtonCradle.h"
#include "Labs/Common/ImGuiHelper.h"
#include <iostream>

namespace VCX::Labs::RigidBody {

    CaseNewtonCradle::CaseNewtonCradle(std::initializer_list<Assets::ExampleScene> && scenes) :
        _scenes(scenes),
        _program(
            Engine::GL::UniqueProgram({
                Engine::GL::SharedShader("assets/shaders/sphere_phong.vert"),
                Engine::GL::SharedShader("assets/shaders/phong.frag") })),
        _sceneObject(1) { 
        _cameraManager.AutoRotate = false;
        _program.BindUniformBlock("PassConstants", 1);
        _program.GetUniforms().SetByName("u_DiffuseMap" , 0);
        _program.GetUniforms().SetByName("u_SpecularMap", 1);
        _program.GetUniforms().SetByName("u_HeightMap"  , 2);
        ResetSystem();
        _sphere = Engine::Model { Engine::Sphere(_res, _r), 0 };
    }

    void CaseNewtonCradle::OnSetupPropsUI() {
        if(ImGui::Button("Reset System")) 
            ResetSystem();
        ImGui::SameLine();
        if(ImGui::Button(_stopped ? "Start Simulation":"Stop Simulation"))
            _stopped = ! _stopped;
    }


    Common::CaseRenderResult CaseNewtonCradle::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (_recompute) {
            _recompute = false;
            _sceneObject.ReplaceScene(GetScene(_sceneIdx));
            _cameraManager.Save(_sceneObject.Camera);
        }
        if (! _stopped) _simulation.SimulateTimestep(Engine::GetDeltaTime(), _gravity, _length);
        
        _frame.Resize(desiredSize);

        _cameraManager.Update(_sceneObject.Camera);
        _sceneObject.PassConstantsBlock.Update(&VCX::Labs::Rendering::SceneObject::PassConstants::Projection, _sceneObject.Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _sceneObject.PassConstantsBlock.Update(&VCX::Labs::Rendering::SceneObject::PassConstants::View, _sceneObject.Camera.GetViewMatrix());
        _sceneObject.PassConstantsBlock.Update(&VCX::Labs::Rendering::SceneObject::PassConstants::ViewPosition, _sceneObject.Camera.Eye);
        
        if (_uniformDirty) {
            _uniformDirty = false;
            _program.GetUniforms().SetByName("u_AmbientScale"      , _ambientScale);
            _program.GetUniforms().SetByName("u_UseBlinn"          , _useBlinn);
            _program.GetUniforms().SetByName("u_Shininess"         , _shininess);
            _program.GetUniforms().SetByName("u_UseGammaCorrection", int(_useGammaCorrection));
            _program.GetUniforms().SetByName("u_AttenuationOrder"  , _attenuationOrder);            
            _program.GetUniforms().SetByName("u_BumpMappingBlend"  , _bumpMappingPercent * .01f);            
        }
        
        gl_using(_frame);

        glEnable(GL_DEPTH_TEST);
        glLineWidth(_BndWidth);
        glLineWidth(1.f);

        std::vector<glm::vec3> positions;
        for (auto const & ball : _simulation.Balls) {
            positions.push_back(ball._pos);
        }

        Rendering::ModelObject m        = Rendering::ModelObject(_sphere, positions);
        auto const &           material = _sceneObject.Materials[0];
        m.Mesh.Draw({ material.Albedo.Use(),  material.MetaSpec.Use(), material.Height.Use(),_program.Use() },
            _sphere.Mesh.Indices.size(), 0, numofSpheres);
        
        glDepthFunc(GL_LEQUAL);
        glDepthFunc(GL_LESS);
        glDisable(GL_DEPTH_TEST);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseNewtonCradle::OnProcessInput(ImVec2 const& pos) {
        _cameraManager.ProcessInput(_sceneObject.Camera, pos);
    }

    void CaseNewtonCradle::ResetSystem(){
        _simulation.Balls.clear();
        for (int i = 0; i < numofSpheres; i++) {
            _simulation.Balls.emplace_back(Ball(i, _mass, _r, glm::vec3(float(i) * 2.f * _r, 0, -_length)));
        }
        for (int i = 0; i < numofSpheresPulled; i++) {
            _simulation.Balls[i]._pos.x -= _length * .5f;
            _simulation.Balls[i]._pos.z += _length * (1 - std::cos(glm::radians(30.f)));
        }
    }
}