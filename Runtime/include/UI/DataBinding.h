#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace Runtime {
namespace UI {

using BindingValue = std::variant<std::monostate, bool, int, float, std::string>;
using BindingCallback = std::function<void(const BindingValue&)>;

class DataBindingContext {
public:
	using ListenerId = uint64_t;

	void SetValue(const std::string& key, BindingValue value) {
		m_values[key] = std::move(value);
		Notify(key);
	}

	[[nodiscard]] BindingValue GetValue(const std::string& key) const {
		const auto it = m_values.find(key);
		if (it == m_values.end()) {
			return std::monostate{};
		}
		return it->second;
	}

	[[nodiscard("listener id is required to unsubscribe")]] ListenerId Subscribe(const std::string& key, BindingCallback callback) {
		const ListenerId id = ++m_listenerSeed;
		m_listeners[key].push_back(Listener{id, std::move(callback)});
		return id;
	}

	void Unsubscribe(const std::string& key, ListenerId id) {
		auto it = m_listeners.find(key);
		if (it == m_listeners.end()) {
			return;
		}
		auto& listeners = it->second;
		listeners.erase(
			std::remove_if(listeners.begin(), listeners.end(), [id](const Listener& item) { return item.id == id; }),
			listeners.end());
	}

private:
	struct Listener {
		ListenerId id = 0;
		BindingCallback callback;
	};

	void Notify(const std::string& key) {
		const auto valueIt = m_values.find(key);
		const auto listenerIt = m_listeners.find(key);
		if (valueIt == m_values.end() || listenerIt == m_listeners.end()) {
			return;
		}

		for (const Listener& listener : listenerIt->second) {
			if (listener.callback) {
				listener.callback(valueIt->second);
			}
		}
	}

	std::unordered_map<std::string, BindingValue> m_values;
	std::unordered_map<std::string, std::vector<Listener>> m_listeners;
	ListenerId m_listenerSeed = 0;
};

} // namespace UI
} // namespace Runtime
