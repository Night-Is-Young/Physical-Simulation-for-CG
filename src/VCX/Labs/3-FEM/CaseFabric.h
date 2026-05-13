#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/UniformBlock.hpp"
#include "Labs/3-FEM/TriangleSystem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Scene/Content.h"
#include "Labs/Scene/SceneObject.h"

namespace VCX::Labs::FEM {
    class CaseFabric : public Common::ICase {
    public:
        CaseFabric();

        virtual std::string_view const GetName() override { return "Fabric Simulation"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void ProcessKeyInput(int key, int action);

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(2.0f, -2.0f, 1.0f), .Target = glm::vec3(0.0f, 0.0f, 0.0f), .Up = glm::vec3(0.0f, 0.0f, 1.0f) };
        Common::OrbitCameraManager          _cameraManager;
        Engine::GL::UniqueRenderItem        _verticesItem;
        Engine::GL::UniqueIndexedRenderItem _linesItem;
        Engine::GL::UniqueIndexedRenderItem _trianglesItem;
        TriangleSystem                      _trisystem;
        bool                                _stopped { false };
        bool                                _showSurface { true };
        bool                                _showEdges { true };
        bool                                _showVertices { true };
        float                               _vertexSize { 5 };
        float                               _lineWidth { 2 };
        glm::vec4                           _vertexColor { 0.1f, 0.1f, 0.9f, 1.0f };
        glm::vec4                           _lineColor { 1.0f, 1.0f, 1.0f, 1.0f };
        glm::vec4                           _triangleColor { 0.1f, 0.1f, 0.5f, 0.6f };

        float                               impulseMagnitude { 5.0f };

        void ResetSystem();
    };
}   