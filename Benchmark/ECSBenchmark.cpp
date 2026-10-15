#include <ECS/Component.h>
#include <ECS/Registry.h>
#include <chrono>
#include <iostream>

int main() {
    using namespace Runtime::ECS;
    constexpr int entityCount = 100000;
    constexpr int iterations = 200;
    Registry registry;
    for (int i = 0; i < entityCount; ++i) {
        const auto entity = registry.CreateEntity();
        registry.AddComponent<Components::Transform2D>(entity);
        registry.AddComponent<Components::Velocity2D>(entity, Components::Velocity2D{1.0f, 2.0f});
    }
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < iterations; ++i) {
        registry.View<Components::Velocity2D, Components::Transform2D>().Each(
            [](Entity, const Components::Velocity2D& velocity, Components::Transform2D& transform) {
            transform.x += velocity.vx;
            transform.y += velocity.vy;
        });
    }
    const auto elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    double checksum = 0.0;
    for (auto entity : registry.EntitiesWith<Components::Transform2D>()) {
        const auto& transform = registry.GetComponent<Components::Transform2D>(entity);
        checksum += transform.x + transform.y;
    }
    std::cout << "{\"storage\":\"archetype\",\"query\":\"View::Each\",\"entities\":"
              << entityCount << ",\"iterations\":" << iterations
              << ",\"update_ms\":" << elapsed << ",\"checksum\":" << checksum << "}\n";
    return checksum == entityCount * iterations * 3.0 ? 0 : 1;
}
