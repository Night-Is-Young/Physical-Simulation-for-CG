#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/UniformBlock.hpp"
#include "Engine/Sphere.h"
#include "Labs/1-RigidBody/CradleSimulator.h"
#include "Labs/1-RigidBody/Ball.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Scene/Content.h"
#include "Labs/Scene/SceneObject.h"


namespace VCX::Labs::RigidBody {

    class CaseNewtonCradle : public Common::ICase {
    public:
        CaseNewtonCradle(std::initializer_list<Assets::ExampleScene> && scenes);

        virtual std::string_view const GetName() override { return "Fluid Simulation"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        std::vector<Assets::ExampleScene> const _scenes;

        Engine::GL::UniqueProgram         _program;
        Engine::GL::UniqueRenderFrame     _frame;
        VCX::Labs::Rendering::SceneObject _sceneObject;
        std::size_t                       _sceneIdx { 0 };
        bool                              _recompute { true };
        bool                              _uniformDirty { true };
        int                               _msaa { 2 };
        int                               _useBlinn { 0 };
        float                             _shininess { 32 };
        float                             _ambientScale { 1 };
        bool                              _useGammaCorrection { true };
        int                               _attenuationOrder { 2 };
        int                               _bumpMappingPercent { 20 };

        Common::OrbitCameraManager          _cameraManager;
        float                               _BndWidth { 2.0 };
        bool                                _stopped { false };
        Engine::Model                       _sphere;

        int                                 _res { 30 };
        float                               _r { 1.f };
        float                               _mass { 1.f };
        float                               _length { 5.f };
        float                               _gravity { 9.81f };

        int                                 _numofSpheres { 5 };
        int                                 _numofSpheresPulled { 1 };
        RigidBody::Simulator                _simulation;

        char const *          GetSceneName(std::size_t const i) const { return VCX::Labs::Rendering::Content::SceneNames[std::size_t(_scenes[i])].c_str(); }
        Engine::Scene const & GetScene(std::size_t const i) const { return VCX::Labs::Rendering::Content::Scenes[std::size_t(_scenes[i])]; }
        void                  ResetSystem();
    };
} // namespace VCX::Labs::RigidBody
