#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/1-RigidBody/Box.h"

namespace VCX::Labs::RigidBody {

    class CaseTwoBox : public Common::ICase {
    public:
        CaseTwoBox();

        virtual std::string_view const GetName() override { return "Two Boxes Simulation"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void OnProcessMouseControl(glm::vec3 mouseDelta);
        void AdvanceTwoBox(float dt);
        void ResolveCollision(Box & box1, Box & box2);

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager          _cameraManager;
        
        Box box1;
        Box box2;
        bool _isStopped = false;
        float _muN = 0.8f;
        float _muT = 0.52;
    };
} // namespace VCX::Labs::RigidBody
