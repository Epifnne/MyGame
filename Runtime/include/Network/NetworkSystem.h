#pragma once

#include <functional>

#include "ECS/System.h"
#include "Network/NetworkManager.h"

namespace Runtime {
namespace Network {

struct NetworkConnectedEvent {};
struct NetworkDisconnectedEvent {};
struct NetworkErrorEvent {
	const char* message = nullptr;
};
struct NetworkMessageEvent {
	Message message;
};

class NetworkSystem : public ECS::System {
public:
	explicit NetworkSystem(NetworkManager* manager) : m_manager(manager) {}

	void Update(ECS::Registry& registry, float dt) override;

	void SetOnConnected(std::function<void()> fn) { m_onConnected = std::move(fn); }
	void SetOnDisconnected(std::function<void()> fn) { m_onDisconnected = std::move(fn); }
	void SetOnError(std::function<void(const char*)> fn) { m_onError = std::move(fn); }
	void SetOnMessage(std::function<void(const Message&)> fn) { m_onMessage = std::move(fn); }

private:
	NetworkManager* m_manager = nullptr;
	std::function<void()> m_onConnected;
	std::function<void()> m_onDisconnected;
	std::function<void(const char*)> m_onError;
	std::function<void(const Message&)> m_onMessage;
};

} // namespace Network
} // namespace Runtime
