#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/UniformBlock.hpp"
#include "Labs/3-FEM/TetrahedronSystem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Scene/Content.h"
#include "Labs/Scene/SceneObject.h"

namespace VCX::Labs::FEM {
    class CaseFEM : public Common::ICase {
    public:
        CaseFEM();

        virtual std::string_view const GetName() override { return "FEM Simulation"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void ProcessKeyInput(int key, int action);
        void UpdateArrow(const glm::vec3 & startPos, const glm::vec3 & direction, char type);

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(4.0f, -10.0f, 1.0f), .Target = glm::vec3(4.0f, 0.0f, 0.0f), .Up = glm::vec3(0.0f, 0.0f, 1.0f) };
        Common::OrbitCameraManager          _cameraManager;
        Engine::GL::UniqueRenderItem        _verticesItem;
        Engine::GL::UniqueIndexedRenderItem _linesItem;
        Engine::GL::UniqueIndexedRenderItem _trianglesItem;
        TetrahedronSystem                   _tetsystem;
        bool                                _stopped { false };
        bool                                _showSurface { true };
        bool                                _showEdges { true };
        bool                                _showVertices { true };
        float                               _vertexSize { 5 };
        float                               _lineWidth { 2 };
        glm::vec4                           _vertexColor { 0.1f, 0.1f, 0.9f, 1.0f };
        glm::vec4                           _lineColor { 1.0f, 1.0f, 1.0f, 1.0f };
        glm::vec4                           _triangleColor { 0.1f, 0.1f, 0.5f, 0.6f };
        std::vector<glm::vec3>              _wallPositions {
            glm::vec3(0.0f, -5.0f, -5.0f),
            glm::vec3(0.0f, 5.0f, -5.0f),
            glm::vec3(0.0f, 5.0f, 5.0f),
            glm::vec3(0.0f, -5.0f, 5.0f)
        };
        Engine::GL::UniqueIndexedRenderItem _wallItem;
        float                               impulseMagnitude { 5.0f };

        std::vector<glm::vec3>              _arrowVertices;
        Engine::GL::UniqueIndexedRenderItem _arrowItem;
        float                               _arrowScale { 2.0f };
        bool                                _showArrow { false };

        void ResetSystem();
    };
}   