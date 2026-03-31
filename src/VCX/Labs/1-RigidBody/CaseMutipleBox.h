#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/1-RigidBody/Box.h"
#include "Labs/1-RigidBody/Wall.h"

namespace VCX::Labs::RigidBody {

    class CaseMutipleBox : public Common::ICase {
    public:
        CaseMutipleBox();

        virtual std::string_view const GetName() override { return "Multiple Boxes Simulation"; }
        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void OnProcessMouseControl(glm::vec3 mouseDelta);
        void AdvanceMultipleBox(float dt);
        void ResolveCollisionB2B(Box & box1, Box & box2);
        void ResolveCollisionB2W(Box & box, Wall & wall);

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager          _cameraManager;
        
        std::vector<Wall> _walls;
        std::vector<Box>  _boxes;
        int               _numBoxes  = 4;
        float             _gravity   = 9.8f;
        bool              _isStopped = false;
        float             _muN       = 0.9f;
        float             _muT       = 0.5f;
        float             _c         = 0.99f;
    };
} // namespace VCX::Labs::RigidBody
