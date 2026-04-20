// Runtime/include/ECS/SystemPool.h
#pragma once

#include "System.h"
#include <vector>
#include <memory>

namespace Runtime {
namespace ECS {

class SystemManager {
public:
	void AddSystem(std::unique_ptr<System> sys) { m_systems.push_back(std::move(sys)); }
	void UpdateAll(Registry& registry, float dt) {
		for (auto& s : m_systems) s->Update(registry, dt);
	}

private:
	std::vector<std::unique_ptr<System>> m_systems;
};

} // namespace ECS
} // namespace Runtime
