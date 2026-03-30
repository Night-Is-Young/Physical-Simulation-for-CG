#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"

namespace VCX::Labs::RigidBody {

    class CaseSingleBox : public Common::ICase {
    public:
        CaseSingleBox();

        virtual std::string_view const GetName() override { return "Draw a 3D Box"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void OnProcessMouseControl(glm::vec3 mouseDelta);
        void AdvanceSingleBox(float dt);

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager          _cameraManager;
        Engine::GL::UniqueIndexedRenderItem _boxItem;  // render the box
        Engine::GL::UniqueIndexedRenderItem _lineItem; // render line on box
        glm::vec3                           _center { 0.f, 0.f, 0.f };
        glm::vec3                           _dim { 1.f, 2.f, 3.f };
        glm::vec3                           _boxColor { 121.0f / 255, 207.0f / 255, 171.0f / 255 };
        
        float     _mass = 100.f;
        glm::vec3 _velocity { 0.f };
        glm::quat _q { 1.f, 0.f, 0.f, 0.f };
        glm::vec3 _omega { 0.f };
        glm::vec3 _force { 0.f };
        glm::vec3 _forcePos { 0.f };
        glm::mat3 _I;
        bool      _isStopped = false;
    };
} // namespace VCX::Labs::RigidBody
