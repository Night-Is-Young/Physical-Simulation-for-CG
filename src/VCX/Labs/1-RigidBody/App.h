#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/1-RigidBody/Box.h"
#include "Labs/1-RigidBody/Wall.h"
#include "Labs/1-RigidBody/CaseSingleBox.h"
#include "Labs/1-RigidBody/CaseTwoBox.h"
#include "Labs/1-RigidBody/CaseMultipleBox.h"
#include "Labs/1-RigidBody/CaseNewtonCradle.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::RigidBody {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        CaseSingleBox    _CaseSingleBox;
        CaseTwoBox       _CaseTwoBox;
        CaseMultipleBox  _CaseMultipleBox;
        CaseNewtonCradle _CaseNewtonCradle;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _CaseSingleBox, _CaseTwoBox, _CaseMultipleBox, _CaseNewtonCradle };

    public:
        App();

        void OnFrame() override;
    };
}
