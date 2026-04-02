#include "Assets/bundled.h"
#include "Labs/1-RigidBody/App.h"

namespace VCX::Labs::RigidBody {

    App::App():
        _ui(Labs::Common::UIOptions {}),
        _CaseNewtonCradle({ Assets::ExampleScene::Fluid }) {
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}
