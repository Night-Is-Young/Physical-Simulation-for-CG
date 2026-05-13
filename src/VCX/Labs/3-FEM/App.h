#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/3-FEM/CaseFEM.h"
#include "Labs/3-FEM/CaseFabric.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::FEM {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        CaseFEM _CaseFEM;
        CaseFabric _CaseFabric;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _CaseFEM, _CaseFabric };

    public:
        App();
        void OnFrame() override;
    };
}
