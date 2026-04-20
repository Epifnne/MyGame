// Runtime/include/ECS/EntityPool.h
#pragma once

#include "Entity.h"
#include <vector>

namespace Runtime {
namespace ECS {

class EntityPool {
public:
	EntityPool() {
		m_generations.push_back(0); /* index 0 reserved for NullEntity */
		m_alive.push_back(false);
	}

	Entity Create() {
		if (!m_free.empty()) {
			Entity e = m_free.back(); m_free.pop_back(); m_alive[e] = true; return e;
		}
		Entity id = static_cast<Entity>(m_generations.size());
		m_generations.push_back(1);
		m_alive.push_back(true);
		return id;
	}

	void Destroy(Entity e) {
		if (e == NullEntity || e >= static_cast<Entity>(m_generations.size())) return;
		if (!m_alive[e]) return;
		++m_generations[e];
		m_alive[e] = false;
		m_free.push_back(e);
	}

	bool IsAlive(Entity e) const {
		return e > 0 && e < static_cast<Entity>(m_generations.size()) && m_alive[e];
	}

	uint32_t Generation(Entity e) const {
		if (e >= static_cast<Entity>(m_generations.size())) return 0;
		return m_generations[e];
	}

private:
	std::vector<uint32_t> m_generations; // index -> generation
	std::vector<bool> m_alive;
	std::vector<Entity> m_free;
};

} // namespace ECS
} // namespace Runtime

