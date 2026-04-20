#include "Systems/CubeRenderSystem.h"

#include "Components/CubeRenderComponent.h"
#include "Components/RotationComponent.h"
#include "Components/Transform3DComponent.h"

#include <Graphics/Renderer.h>
#include <glm/gtc/matrix_transform.hpp>

namespace Game {
namespace Systems {

void CubeRenderSystem::Render(Runtime::ECS::World& world,
                              Runtime::Graphics::Renderer& renderer,
                              Runtime::Graphics::Mesh& mesh,
                              Runtime::Graphics::Material& material,
                              const Runtime::Graphics::Camera& camera) {
    auto entities = world.RegistryRef().EntitiesWith<Components::CubeRenderComponent>();
    for (auto entity : entities) {
        if (!world.RegistryRef().HasComponent<Components::Transform3DComponent>(entity)) {
            continue;
        }
        if (!world.RegistryRef().HasComponent<Components::RotationComponent>(entity)) {
            continue;
        }

        const auto& transform = world.RegistryRef().GetComponent<Components::Transform3DComponent>(entity);
        const auto& rotation = world.RegistryRef().GetComponent<Components::RotationComponent>(entity);

        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, transform.position);
        model = glm::rotate(model, rotation.angle, rotation.axis);
        model = glm::scale(model, transform.scale);

        renderer.Submit(mesh, material, model);
    }

    renderer.Flush(camera);
}

} // namespace Systems
} // namespace Game
