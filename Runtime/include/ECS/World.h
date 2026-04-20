// Runtime/include/ECS/World.h
#pragma once

#include "Registry.h"
#include "SystemPool.h"
#include "EventBus.h"

namespace Runtime {
namespace ECS {

class World {
public:
	World() = default;

	Entity CreateEntity() { return m_registry.CreateEntity(); }
	void DestroyEntity(Entity e) { m_registry.DestroyEntity(e); }

	template<typename T, typename... Args>
	void AddComponent(Entity e, Args&&... args) { m_registry.AddComponent<T>(e, std::forward<Args>(args)...); }

	template<typename T>
	bool HasComponent(Entity e) const { return m_registry.HasComponent<T>(e); }

	void Update(float dt) { m_systems.UpdateAll(m_registry, dt); }

	SystemManager& Systems() { return m_systems; }
	Registry& RegistryRef() { return m_registry; }
	EventBus& Events() { return m_events; }

private:
	Registry m_registry;
	SystemManager m_systems;
	EventBus m_events;
};

} // namespace ECS
} // namespace Runtime

