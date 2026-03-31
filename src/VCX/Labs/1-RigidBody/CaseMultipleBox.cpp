#include "Labs/1-RigidBody/CaseMultipleBox.h"
#include "Labs/Common/ImGuiHelper.h"
#include "Engine/app.h"
#include <fcl/narrowphase/collision.h>

namespace VCX::Labs::RigidBody {

    CaseMultipleBox::CaseMultipleBox():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })) {
        // Initialize Walls' Datas
        const std::vector<glm::vec3> wallPositions {
            glm::vec3(0.0f, 0.0f, -10.0f),
            glm::vec3(10.0f, 0.0f, -5.0f),
            glm::vec3(0.0f, 10.0f, -5.0f),
            glm::vec3(-10.0f, 0.0f, -5.0f),
            glm::vec3(0.0f, -10.0f, -5.0f),
        };
        const std::vector<glm::vec2> wallDims {
            glm::vec2(20.0f, 20.0f),
            glm::vec2(20.0f, 10.0f),
            glm::vec2(20.0f, 10.0f),
            glm::vec2(20.0f, 10.0f),
            glm::vec2(20.0f, 10.0f),
        };
        const std::vector<glm::vec3> wallTg1 {
            glm::vec3(1.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, 1.0f, 0.0f),
            glm::vec3(-1.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, -1.0f, 0.0f),
            glm::vec3(1.0f, 0.0f, 0.0f)
        };
        const std::vector<glm::vec3> wallTg2 {
            glm::vec3(0.0f, 1.0f, 0.0f),
            glm::vec3(0.0f, 0.0f, -1.0f),
            glm::vec3(0.0f, 0.0f, -1.0f),
            glm::vec3(0.0f, 0.0f, -1.0f),
            glm::vec3(0.0f, 0.0f, -1.0f),
        };
        // Construct Walls
        for (int id = 0; id < wallPositions.size(); id++) {
            _walls.emplace_back(id, wallPositions[id], wallDims[id], wallTg1[id], wallTg2[id]);
        }
        for (auto& wall : _walls) {
            auto span_bytes = Engine::make_span_bytes<glm::vec3>(wall._verticesPos);
            wall._triangleItem.UpdateVertexBuffer("position", span_bytes);
            wall._lineItem.UpdateVertexBuffer("position", span_bytes);
        }

        // Initialize Boxes
        float Lx = _walls[0]._dim.x;
        float Ly = _walls[0]._dim.y;
        float boxSize { 2.0f };
        float horizontalSpacing { 1.0f };
        float verticalSpacing { 1.5f };
        int   Nx = static_cast<int>((Lx - boxSize - 2 * horizontalSpacing) / (boxSize + horizontalSpacing)) + 1;
        int   Ny = static_cast<int>((Ly - boxSize - 2 * horizontalSpacing) / (boxSize + horizontalSpacing)) + 1;

        for (int id { 0 }; id < _numBoxes; id++) {
            int   k = id / (Nx * Ny);
            int   j = (id - k * Nx * Ny) / Nx;
            int   i = id - k * Nx * Ny - j * Nx;
            float x = (-Lx / 2.0f + horizontalSpacing + boxSize / 2.0f) + i * (boxSize + horizontalSpacing);
            float y = (-Ly / 2.0f + horizontalSpacing + boxSize / 2.0f) + j * (boxSize + horizontalSpacing);
            float z = (_walls[0]._pos.z + verticalSpacing + boxSize / 2.0f) + k * (boxSize + verticalSpacing);
            Box   box(id, 1.0f, glm::vec3(boxSize), glm::vec3(x, y, z), glm::sphericalRand(0.5f), glm::sphericalRand(1.0f));
            _boxes.push_back(std::move(box));
        }

        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
    }


    void CaseMultipleBox::OnSetupPropsUI() {
        //
    }

    void CaseMultipleBox::ResolveCollisionB2B(Box & box1, Box & box2) {
        using ColGeoPtr_t = std::shared_ptr<fcl::CollisionGeometry<float>>;
        ColGeoPtr_t geo1(new fcl::Box<float>(box1._dim.x, box1._dim.y, box1._dim.z));
        ColGeoPtr_t geo2(new fcl::Box<float>(box2._dim.x, box2._dim.y, box2._dim.z));
        fcl::Transform3f trans1(Eigen::Translation3f(Eigen::Vector3f(box1._pos.x, box1._pos.y, box1._pos.z)) * Eigen::Quaternionf(box1._q.w, box1._q.x, box1._q.y, box1._q.z));
        fcl::Transform3f trans2(Eigen::Translation3f(Eigen::Vector3f(box2._pos.x, box2._pos.y, box2._pos.z)) * Eigen::Quaternionf(box2._q.w, box2._q.x, box2._q.y, box2._q.z));
        fcl::CollisionObject<float> obj1(geo1, trans1);
        fcl::CollisionObject<float> obj2(geo2, trans2);
        fcl::CollisionRequest<float> colRequest(8, true);
        fcl::CollisionResult<float>  colRes;
        fcl::collide(&obj1, &obj2, colRequest, colRes);
        if (colRes.isCollision()) {
            std::vector<fcl::Contact<float>> contacts;
            colRes.getContacts(contacts);
            glm::vec3 colPos { 0.0f };
            glm::vec3 colNormal { 0.0f };
            float     colDepth = 0.0f;
            int       counter {0};
            for (const auto & contact : contacts) {
                colPos += glm::vec3(contact.pos[0], contact.pos[1], contact.pos[2]);
                colNormal += glm::vec3(contact.normal[0], contact.normal[1], contact.normal[2]);
                colDepth += contact.penetration_depth;
                counter++;
            }
            static_cast<float>(counter);
            colPos /= counter;
            colDepth /= counter;
            colNormal = glm::normalize(colNormal);
            glm::vec3 Rr1 = colPos - box1._pos;
            glm::vec3 Rr2 = colPos - box2._pos;
            glm::vec3 colVel1 = box1._velocity + glm::cross(box1._omega, Rr1);
            glm::vec3 colVel2 = box2._velocity + glm::cross(box2._omega, Rr2);
            glm::vec3 vRel = colVel2 - colVel1;
            float     dotProduct = glm::dot(vRel, colNormal);
            if ( dotProduct < 0) {
                // Resolve Collision
                glm::vec3 vRelN = dotProduct * colNormal;
                glm::vec3 vRelT = vRel - vRelN;
                glm::vec3 vRelNn = -_muN * vRelN;
                float     alpha  = fmax(1.0f - _muT * (1 + _muN) * glm::length(vRelN) / glm::length(vRelT), 0.0f);
                glm::vec3 vRelTn = alpha * vRelT;
                glm::vec3 vReln  = vRelNn + vRelTn;

                glm::mat3 r1(0.0f, -Rr1.z, Rr1.y, Rr1.z, 0.0f, -Rr1.x, -Rr1.y, Rr1.x, 0.0f);
                glm::mat3 R1 = glm::mat3_cast(box1._q);
                glm::mat3 I_world1 = R1 * box1._I * glm::transpose(R1);
                glm::mat3 I_world_inv1 = glm::inverse(I_world1);
                glm::mat3 K1           = 1.0f / box1._mass * glm::mat3(1.0f) - r1 * I_world_inv1 * r1;

                glm::mat3 r2(0.0f, -Rr2.z, Rr2.y, Rr2.z, 0.0f, -Rr2.x, -Rr2.y, Rr2.x, 0.0f);
                glm::mat3 R2 = glm::mat3_cast(box2._q);
                glm::mat3 I_world2 = R2 * box2._I * glm::transpose(R2);
                glm::mat3 I_world_inv2 = glm::inverse(I_world2);
                glm::mat3 K2           = 1.0f / box2._mass * glm::mat3(1.0f) - r2 * I_world_inv2 * r2;

                glm::mat3 K = K1 + K2;
                glm::vec3 j = glm::inverse(K) * (vReln - vRel);
                box1._velocity += -j / box1._mass;
                box2._velocity += j / box2._mass;
                box1._omega += I_world_inv1 * glm::cross(Rr1, -j);
                box2._omega += I_world_inv2 * glm::cross(Rr2, j);
            }
        }
    }

    void CaseMultipleBox::ResolveCollisionB2W(Box & box, const Wall & wall) {
        glm::vec3 n = wall._normal;
        glm::mat3 R = glm::mat3_cast(box._q);
        int       counter { 0 };
        glm::vec3 colPos { 0.0f };
        float     colDepth = 0.0f;
        for (const auto & vertPos : box._verticesPos) {
            glm::vec3 vertWorldPos = box._pos + R * vertPos;
            glm::vec3 vertVel      = box._velocity + glm::cross(box._omega, vertWorldPos - box._pos);
            float     dist = glm::dot(vertWorldPos - wall._pos, n);
            if (dist < 0) {
                colPos += vertWorldPos;
                colDepth += -dist;
                counter++;
            }
        }
        if (counter > 0) {
            static_cast<float>(counter);
            colPos /= counter;
            colDepth /= counter;
            box._pos += colDepth * n;
            colPos += colDepth * n;
            glm::vec3 Rr = colPos - box._pos;
            glm::vec3 colVel = box._velocity + glm::cross(box._omega, Rr);
            float     dotProduct = glm::dot(colVel, n);
            if (dotProduct < 0) {
                // Resolve Collision
                glm::mat3 I_world = box._I_world;
                glm::mat3 I_world_inv = glm::inverse(I_world);
                float     J           = (-1.0f - _c) * dotProduct / (1.0f / box._mass + glm::dot(n, glm::cross(I_world_inv * glm::cross(Rr, n), Rr)));
                box._velocity += J * n / box._mass;
                box._omega += I_world_inv * glm::cross(Rr, J * n);
            }
        }
    }

    void CaseMultipleBox::AdvanceMultipleBox(float dt) {
        for (int id = 0; id < _numBoxes; id++) {
            for (const auto & wall : _walls) {
                ResolveCollisionB2W(_boxes[id], wall);
            }
            for (int jd = id + 1; jd < _numBoxes; jd++) {
                ResolveCollisionB2B(_boxes[id], _boxes[jd]);
            }
            _boxes[id]._omega += dt * glm::inverse(_boxes[id]._I_world) * (-glm::cross(_boxes[id]._omega, _boxes[id]._I_world * _boxes[id]._omega));
            _boxes[id]._velocity += dt * glm::vec3(0.0f, 0.0f, -_gravity);
            _boxes[id]._pos += _boxes[id]._velocity * dt;
            _boxes[id]._q = glm::normalize(glm::exp(0.5f * dt * glm::quat(0.f, _boxes[id]._omega)) * _boxes[id]._q);
            glm::mat3 R   = glm::mat3_cast(_boxes[id]._q);
            _boxes[id]._I_world = R * _boxes[id]._I * glm::transpose(R);
        }
    }

    Common::CaseRenderResult CaseMultipleBox::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        // apply mouse control first
        OnProcessMouseControl(_cameraManager.getMouseMove());
        if (! _isStopped) {
            AdvanceMultipleBox(Engine::GetDeltaTime());
        }
        // rendering
        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glLineWidth(.5f);

        glDepthMask(GL_TRUE);
        for (auto & box : _boxes) {
            std::vector<glm::vec3> VertsPosition;
            glm::mat3              R = glm::mat3_cast(box._q);
            for (const auto & vertexPosition : box._verticesPos) {
                VertsPosition.push_back(glm::vec3(box._pos + R * vertexPosition));
            }
            auto span_bytes = Engine::make_span_bytes<glm::vec3>(VertsPosition);
            box._triangleItem.UpdateVertexBuffer("position", span_bytes);
            box._lineItem.UpdateVertexBuffer("position", span_bytes);
            _program.GetUniforms().SetByName("u_Color", box._color);
            box._triangleItem.Draw({ _program.Use() });
            box._lineItem.Draw({ _program.Use() });
        }

        glDepthMask(GL_FALSE);
        for (const auto & wall : _walls) {
            _program.GetUniforms().SetByName("u_Color", wall._color);
            wall._triangleItem.Draw({ _program.Use() });
            wall._lineItem.Draw({ _program.Use() });
        }
        glDepthMask(GL_TRUE);

        glLineWidth(1.f);
        glPointSize(1.f);
        glDisable(GL_LINE_SMOOTH);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseMultipleBox::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

    void CaseMultipleBox::OnProcessMouseControl(glm::vec3 mouseDelta) {
        float movingScale = 0.1f;
        //_center += mouseDelta * movingScale;
    }

} // namespace VCX::Labs::RigidBody
