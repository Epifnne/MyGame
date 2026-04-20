#pragma once

#include <ECS/World.h>
#include <Graphics/Mesh.h>
#include <Graphics/Material.h>
#include <Graphics/Camera.h>

namespace Runtime { namespace Graphics { class Renderer; } }

namespace Game {
namespace Systems {

class CubeRenderSystem {
public:
    static void Render(Runtime::ECS::World& world,
                       Runtime::Graphics::Renderer& renderer,
                       Runtime::Graphics::Mesh& mesh,
                       Runtime::Graphics::Material& material,
                       const Runtime::Graphics::Camera& camera);
};

} // namespace Systems
} // namespace Game
