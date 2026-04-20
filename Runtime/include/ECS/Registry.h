// Runtime/include/ECS/Registry.h
#pragma once

#include <unordered_map>
#include <memory>
#include <typeindex>
#include <stdexcept>
#include "ComponentPool.h"
#include "EntityPool.h"

namespace Runtime {
namespace ECS {

class Registry {
public:
	Registry() = default;

	Entity CreateEntity() { return m_entities.Create(); }

	void DestroyEntity(Entity e) {
		if (!m_entities.IsAlive(e)) return;
		// remove component from all pools
		for (auto &kv : m_pools) kv.second->Erase(e);
		m_entities.Destroy(e);
	}

	template<typename T, typename... Args>
	void AddComponent(Entity e, Args&&... args) {
		GetOrCreatePool<T>()->EmplaceConstruct(e, std::forward<Args>(args)...);
	}

	template<typename T>
	void RemoveComponent(Entity e) {
		auto it = m_pools.find(std::type_index(typeid(T)));
		if (it == m_pools.end()) return;
		static_cast<ComponentPoolWrapper<T>*>(it->second.get())->Erase(e);
	}

	template<typename T>
	bool HasComponent(Entity e) const {
		auto it = m_pools.find(std::type_index(typeid(T)));
		if (it == m_pools.end()) return false;
		return static_cast<const ComponentPoolWrapper<T>*>(it->second.get())->Has(e);
	}

	template<typename T>
	T& GetComponent(Entity e) {
		return GetOrCreatePool<T>()->Get(e);
	}

	template<typename T>
	const T& GetComponent(Entity e) const {
		auto it = m_pools.find(std::type_index(typeid(T)));
		if (it == m_pools.end()) {
			throw std::runtime_error("Component type not registered");
		}
		return static_cast<const ComponentPoolWrapper<T>*>(it->second.get())->Get(e);
	}

	template<typename T>
	std::vector<Entity> EntitiesWith() const {
		auto it = m_pools.find(std::type_index(typeid(T)));
		if (it == m_pools.end()) return {};
		return it->second->Entities();
	}

private:
	template<typename T>
	ComponentPoolWrapper<T>* GetOrCreatePool() {
		auto idx = std::type_index(typeid(T));
		auto it = m_pools.find(idx);
		if (it == m_pools.end()) {
			auto ptr = std::make_unique<ComponentPoolWrapper<T>>();
			ComponentPoolWrapper<T>* raw = ptr.get();
			m_pools.emplace(idx, std::move(ptr));
			return raw;
		}
		return static_cast<ComponentPoolWrapper<T>*>(it->second.get());
	}

	EntityPool m_entities;
	std::unordered_map<std::type_index, std::unique_ptr<IComponentPool> > m_pools;
};

} // namespace ECS
} // namespace Runtime

