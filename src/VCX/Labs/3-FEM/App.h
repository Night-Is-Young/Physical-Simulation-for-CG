#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/3-FEM/CaseFEM.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::FEM {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        CaseFEM _CaseFEM;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _CaseFEM };

    public:
        App();
        void OnFrame() override;
    };
}
