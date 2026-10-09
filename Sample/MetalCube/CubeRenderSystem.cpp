#include "CubeRenderSystem.h"

#include "CubeRenderComponent.h"
#include <ECS/Component.h>

#include <Graphics/Renderer.h>
#include <glm/gtc/matrix_transform.hpp>

namespace Sample {
namespace Systems {

using Runtime::ECS::Components::Rotation;
using Runtime::ECS::Components::Transform3D;

void CubeRenderSystem::Render(Runtime::ECS::World& world,
                              Runtime::Graphics::Renderer& renderer,
                              Runtime::Graphics::Mesh& mesh,
                              Runtime::Graphics::Material& material,
                              const Runtime::Graphics::Camera& camera) {
    auto entities = world.RegistryRef().EntitiesWith<Components::CubeRenderComponent>();
    for (auto entity : entities) {
        if (!world.RegistryRef().HasComponent<Transform3D>(entity)) {
            continue;
        }
        if (!world.RegistryRef().HasComponent<Rotation>(entity)) {
            continue;
        }

        const auto& transform = world.RegistryRef().GetComponent<Transform3D>(entity);
        const auto& rotation = world.RegistryRef().GetComponent<Rotation>(entity);

        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, transform.position);
        model = glm::rotate(model, rotation.angle, rotation.axis);
        model = glm::scale(model, transform.scale);

        renderer.Submit(mesh, material, model);
    }

    renderer.Flush(camera);
}

} // namespace Systems
} // namespace Sample
