#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/2-FluidSimulation/CaseFluid.h"
#include "Labs/2-FluidSimulation/CaseRendering.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::FluidSimulation {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        CaseFluid _CaseFluid;
        CaseRendering _CaseRendering;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _CaseFluid, _CaseRendering };

    public:
        App();
        void OnFrame() override;
    };
}
