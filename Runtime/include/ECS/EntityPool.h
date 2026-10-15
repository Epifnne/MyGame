// Runtime/include/ECS/EntityPool.h
#pragma once

#include "Entity.h"
#include <cstddef>
#include <vector>
#include <limits>
#include <stdexcept>

namespace Runtime {
namespace ECS {

class EntityPool {
public:
	EntityPool() {
		m_generations.push_back(0); /* index 0 reserved for NullEntity */
		m_alive.push_back(false);
	}

	[[nodiscard("created entity must be retained for later access or destruction")]] Entity Create() {
		if (!m_free.empty()) {
			const auto index = m_free.back();
			m_free.pop_back();
			m_alive[index] = true;
			return MakeEntity(index, m_generations[index]);
		}
		if (m_generations.size() > std::numeric_limits<uint32_t>::max()) {
			throw std::overflow_error("Entity index space exhausted");
		}
		const auto index = static_cast<uint32_t>(m_generations.size());
		if (m_generations.size() == m_generations.capacity()) {
			m_generations.reserve(m_generations.capacity() * 2);
		}
		if (m_alive.size() == m_alive.capacity()) m_alive.reserve(m_alive.capacity() * 2);
		m_generations.push_back(1);
		m_alive.push_back(true);
		return MakeEntity(index, 1);
	}

	void Destroy(Entity e) {
		if (!IsAlive(e)) return;
		const auto index = EntityIndex(e);
		// Retire exhausted slots rather than resurrecting a stale handle.
		if (m_generations[index] != std::numeric_limits<uint32_t>::max()) {
			m_free.push_back(index);
			++m_generations[index];
		}
		m_alive[index] = false;
	}

	[[nodiscard]] bool IsAlive(Entity e) const {
		const auto index = EntityIndex(e);
		return index > 0 && index < m_generations.size() && m_alive[index]
			&& EntityGeneration(e) == m_generations[index];
	}

	[[nodiscard]] uint32_t Generation(Entity e) const {
		const auto index = EntityIndex(e);
		if (index >= m_generations.size()) return 0;
		return m_generations[index];
	}

	[[nodiscard]] size_t SlotCount() const { return m_generations.size(); }

private:
	std::vector<uint32_t> m_generations; // index -> generation
	std::vector<bool> m_alive;
	std::vector<uint32_t> m_free;
};

} // namespace ECS
} // namespace Runtime
