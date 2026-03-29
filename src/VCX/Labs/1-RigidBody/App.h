#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/1-RigidBody/CaseSingleBox.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::RigidBody {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        CaseSingleBox        _CaseSingleBox;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _CaseSingleBox };

    public:
        App();

        void OnFrame() override;
    };
}
