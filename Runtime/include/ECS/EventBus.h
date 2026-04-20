// Runtime/include/ECS/EventBus.h
#pragma once

#include <unordered_map>
#include <vector>
#include <functional>
#include <typeindex>
#include <memory>

namespace Runtime {
namespace ECS {

class EventBus {
public:
	template<typename Event>
	void Emit(const Event& e) const {
		auto it = m_listeners.find(std::type_index(typeid(Event)));
		if (it == m_listeners.end()) return;
		for (auto &f : it->second) {
			f(reinterpret_cast<const void*>(&e));
		}
	}

	template<typename Event>
	void Subscribe(std::function<void(const Event&)> fn) {
		auto wrapper = [fn](const void* data) {
			fn(*reinterpret_cast<const Event*>(data));
		};
		m_listeners[std::type_index(typeid(Event))].push_back(wrapper);
	}

private:
	std::unordered_map<std::type_index, std::vector<std::function<void(const void*)>>> m_listeners;
};

} // namespace ECS
} // namespace Runtime

