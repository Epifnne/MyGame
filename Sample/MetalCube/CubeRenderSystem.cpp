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
    world.View<Components::CubeRenderComponent, Transform3D, Rotation>().Each(
        [&](Runtime::ECS::Entity, const Components::CubeRenderComponent&,
            const Transform3D& transform, const Rotation& rotation) {
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, transform.position);
        model = glm::rotate(model, rotation.angle, rotation.axis);
        model = glm::scale(model, transform.scale);

        renderer.Submit(mesh, material, model);
    });

    renderer.Flush(camera);
}

} // namespace Systems
} // namespace Sample
